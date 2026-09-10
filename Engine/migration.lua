-- Temporary seam for one-at-a-time migration. The legacy scheduler stays authoritative.
local Config = require('Engine.config')
local M = {}
local moduleSource = debug and debug.getinfo and debug.getinfo(1, 'S').source or ''
local function basePath()
    local source = moduleSource:gsub('^@', ''):gsub('\\', '/')
    if source == 'Engine/migration.lua' then return '.' end
    return assert(source:match('^(.*)/Engine/migration%.lua$'), 'Cannot locate app directory from module source')
end
function M.new(schedule, hz)
    local migration = {enabled=Config.mode == 'pilot',errors={},fallbacks={},outputCapability='telemetry_only'}
    assert(Config.mode == 'pilot' or Config.mode == 'legacy', 'unknown migration mode')
    if not migration.enabled then return migration end
    local Send = require('Send.physics')
    local sender = Send.new(ac and ac.store)
    local bus = require('Engine.result_bus').new(sender.write)
    local loader = require('Engine.loader').new({base=basePath(),schedule=schedule,hz=hz,bus=bus,
        readRaw=function(key) if ac and ac.load then return ac.load(key) end end})
    migration.errors = loader.errors
    migration.loader = loader
    migration.sender = sender
    for id, err in pairs(loader.errors) do bus.fail(id, err) end
    -- Missing/invalid definitions safely retain the known legacy implementation at startup.
    -- Runtime failures NEVER switch implementations, which would discard state history.
    function migration.resolve(name)
        local model = loader.models[name]
        if model then return model end
        if name == 'brake_fade' then migration.fallbacks[name] = 'legacy: ' .. tostring(loader.errors[name]) end
    end
    local ok, observer = pcall(function()
        return require('Observer.observer').new(bus.snapshot, Config.observerHz)
    end)
    if ok then migration.observer = observer
    else migration.errors.observer = tostring(observer) end
    function migration.updateObserver(dt)
        if not migration.observer then return end
        local success, err = pcall(migration.observer.update, dt)
        if not success then migration.errors.observer = tostring(err); migration.observer = nil end
    end
    function migration.drawUI(ui)
        if migration.observer then
            local success, err = pcall(migration.observer.drawUI, ui)
            if not success then migration.errors.observer = tostring(err); migration.observer = nil end
        end
        for id, err in pairs(migration.errors) do ui.text('Migration ' .. id .. ': ' .. err) end
        if sender.lastError then ui.text('Send: ' .. sender.lastError) end
    end
    return migration
end
return M
