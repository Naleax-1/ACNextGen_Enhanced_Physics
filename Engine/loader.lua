-- Startup only: discover -> decode -> validate -> resolve host slots -> prepare -> cache.
local JSON = require('Engine.json')
local Compiler = require('Engine.compiler')
local Brake = require('Engine.brake')
local M = {}
local function read(path)
    local file, err = io.open(path, 'rb')
    assert(file, err)
    local text = file:read(2097153)
    file:close()
    assert(text, 'empty definition file')
    return text
end
function M.new(options)
    local loader = {models={},errors={},plans={},stats={reads=0,parses=0,compiles=0}}
    local schedule, candidates, seen = {}, {}, {}
    for i, entry in ipairs(options.schedule) do
        schedule[entry.name] = {index=i,enabled=entry.enabled,hz=options.hz[entry.name] or 60}
    end
    local base = options.base or '.'
    local files = options.files
    if not files then
        local ok, result = pcall(function()
            assert(io.scanDir, 'CSP io.scanDir unavailable; pilot not activated')
            return io.scanDir(base .. '/modules', '*.json')
        end)
        if not ok or type(result) ~= 'table' then
            loader.errors.discovery = tostring(result)
            return loader
        end
        files = result
    end
    local ordered = {}
    if type(files) ~= 'table' then
        loader.errors.discovery = 'Expected an array of definition basenames'
        return loader
    end
    for index, filename in pairs(files) do
        if type(index) ~= 'number' or index < 1 or index % 1 ~= 0 or type(filename) ~= 'string' then
            loader.errors['discovery:' .. tostring(index)] = 'Expected a string definition basename at an array index'
        else ordered[#ordered + 1] = filename end
    end
    table.sort(ordered) -- deterministic diagnostics only; never determines physics order
    for _, filename in ipairs(ordered) do
        local id = type(filename) == 'string' and filename:match('^([%w_]+)%.json$')
        local ok, result = pcall(function()
            assert(id, 'expected a definition basename, not a path')
            assert(not seen[id], 'duplicate definition file')
            seen[id] = true
            loader.stats.reads = loader.stats.reads + 1
            local text = (options.read or read)(base .. '/modules/' .. filename)
            loader.stats.parses = loader.stats.parses + 1
            local def = JSON.decode(text)
            assert(type(def) == 'table' and def.module_id == id, 'module_id must match filename')
            local slot = assert(schedule[id], 'definition has no audited host slot')
            assert(slot.enabled, 'definition cannot enable a disabled legacy slot')
            local plan = Compiler.prepare(def)
            loader.stats.compiles = loader.stats.compiles + 1
            assert(def.category == 'brake', 'domain not migrated yet: ' .. def.category)
            assert(def.execution_order.host_index == slot.index, 'host order differs from definition')
            assert(def.execution_order.target_hz == slot.hz, 'host frequency differs from definition')
            for _, dep in ipairs(def.dependencies) do
                local source = schedule[dep.module_id]
                assert(source or not dep.required, 'required dependency missing: ' .. dep.module_id)
                if source and dep.timing == 'earlier_slot' then
                    assert(source.index < slot.index, 'dependency would require reordering: ' .. dep.module_id)
                end
            end
            return plan
        end)
        if ok then candidates[id] = result
        else
            loader.errors[id or tostring(filename)] = tostring(result)
            if id then candidates[id] = nil end
        end
    end
    -- Fail closed for required definition dependencies; never fabricate substitutes.
    local visiting, resolved = {}, {}
    local function resolve(id)
        if resolved[id] then return resolved[id] == 'ok' end
        if visiting[id] then loader.errors[id] = 'cyclic required dependency'; return false end
        if not candidates[id] or loader.errors[id] then return false end
        visiting[id] = true
        for _, dep in ipairs(candidates[id].definition.dependencies) do
            if dep.required and not resolve(dep.module_id) then
                loader.errors[id] = 'required definition unavailable: ' .. dep.module_id
                resolved[id] = 'failed'; visiting[id] = nil
                return false
            end
        end
        visiting[id] = nil; resolved[id] = 'ok'
        return true
    end
    local engine = Brake.new(options.bus, options.readRaw)
    for _, entry in ipairs(options.schedule) do
        local id = entry.name
        if resolve(id) then
            local ok, result = pcall(engine.attach, candidates[id])
            if ok then loader.models[id] = result; loader.plans[id] = candidates[id]
            else loader.errors[id] = tostring(result) end
        end
    end
    loader.engine = engine
    if not loader.models.brake_fade and not loader.errors.brake_fade then
        loader.errors.brake_fade = 'brake_fade.json was not discovered; legacy retained'
    end
    return loader
end
return M
