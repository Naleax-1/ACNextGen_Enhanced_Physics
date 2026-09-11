-- Compile a closed, structured instruction set once. JSON never contains Lua source.
-- The generated chunk has no access to globals, files, require(), AC or UI.
local M = {}
local JSON = require('Engine.json')
local function num(value, fallback)
    local n = tonumber(value)
    if n == nil or n ~= n then return fallback or 0.0 end
    return n
end
local function clamp(value, lo, hi)
    value = num(value, lo)
    if value < lo then return lo end
    if value > hi then return hi end
    return value
end
-- These reproduce source access semantics, including intentionally unprotected
-- accesses represented by the separate direct mode. They add no physics model.
local function safeField(object, key, fallback)
    if not object then return fallback end
    local ok, value = pcall(function() return object[key] end)
    if not ok or value == nil then return fallback end
    return value
end
local function protectedPath(object, first, second)
    local ok, value = pcall(function() return object[first][second] end)
    if ok then return value end
    return nil
end
local function finite(v)
    return type(v) == 'number' and v == v and v ~= math.huge and v ~= -math.huge
end
local function quote(v) return string.format('%q', v) end
local function name(v)
    assert(type(v) == 'string' and v:match('^[a-zA-Z_][a-zA-Z_0-9]*$'), 'invalid identifier')
    return v
end
local function only(object, keys)
    assert(type(object) == 'table' and object ~= JSON.null, 'expected non-null object')
    for key in pairs(object) do assert(keys[key], 'unknown field: ' .. tostring(key)) end
end
local function array(t)
    assert(type(t) == 'table' and t ~= JSON.null, 'expected non-null array')
    local n = #t
    for i=1,n do assert(t[i] ~= nil, 'expected dense array') end
    for k in pairs(t) do assert(type(k) == 'number' and k >= 1 and k <= n and k % 1 == 0, 'expected dense array') end
    return t
end
local binary = { add='+', sub='-', mul='*', div='/', lt='<', le='<=', gt='>', ge='>=', eq='==', ne='~=', ['and']='and', ['or']='or' }
local arities = { max=2, min=2, num=2, clamp=3, not_nil=1, bool01=1 }
function M.prepare(d)
    only(d, {module_id=true, version=true, category=true, inputs=true, parameters=true,
        constants=true, formula=true, intermediate=true, state=true, dependencies=true,
        execution_order=true, outputs=true, entrypoints=true, source=true, wheels=true})
    name(d.module_id)
    assert(d.version == 1, 'unsupported definition version')
    name(d.category)
    for _, field in ipairs({'inputs','parameters','constants','formula','intermediate','state','dependencies','execution_order','outputs','entrypoints','wheels'}) do
        assert(type(d[field]) == 'table' and d[field] ~= JSON.null, 'missing or null section: ' .. field)
    end
    only(d.wheels, {first=true,last=true})
    assert(d.wheels.first == 0 and d.wheels.last == 3, 'this runtime supports AC wheel indices 0..3')
    only(d.execution_order, {host_index=true,target_hz=true,policy=true})
    assert(finite(d.execution_order.host_index) and d.execution_order.host_index >= 1 and d.execution_order.host_index % 1 == 0, 'invalid host index')
    assert(finite(d.execution_order.target_hz) and d.execution_order.target_hz > 0, 'invalid target Hz')
    assert(d.execution_order.policy == 'preserve_host_slot', 'reordering is forbidden')
    local dependenciesSeen = {}
    for _, dep in ipairs(array(d.dependencies)) do
        only(dep, {module_id=true,timing=true,required=true,reason=true})
        name(dep.module_id)
        assert(not dependenciesSeen[dep.module_id], 'duplicate dependency: ' .. dep.module_id)
        dependenciesSeen[dep.module_id] = true
        assert(dep.module_id ~= d.module_id, 'self dependency')
        assert(type(dep.required) == 'boolean', 'dependency required flag missing')
        assert(dep.timing == 'latest_available' or dep.timing == 'earlier_slot', 'invalid dependency timing')
    end
    for _, field in ipairs({'parameters','constants'}) do
        for key, value in pairs(d[field]) do name(key); assert(finite(value), 'non-finite ' .. field .. '.' .. key) end
    end
    for key, spec in pairs(d.intermediate) do name(key); only(spec, {}) end
    for key, spec in pairs(d.state) do
        name(key); only(spec, {initial=true,wheel=true})
        local kind = type(spec.initial)
        assert(kind == 'string' or kind == 'boolean' or finite(spec.initial), 'invalid initial state: ' .. key)
        assert(type(spec.wheel) == 'boolean', 'state wheel flag required')
    end
    local inputKeys = {}
    for key, spec in pairs(d.inputs) do
        name(key); only(spec, {keys=true,wheel=true,selection=true})
        assert(type(spec.wheel) == 'boolean' and spec.selection == 'lua_or', 'invalid input selection')
        assert(#array(spec.keys) > 0, 'input requires keys')
        inputKeys[key] = {}
        for _, prefix in ipairs(spec.keys) do
            assert(type(prefix) == 'string' and prefix:match('^ngp_[%w_]+$'), 'invalid input key')
        end
        for i = 0, 3 do
            local keys = {}
            for _, prefix in ipairs(spec.keys) do keys[#keys + 1] = prefix .. (spec.wheel and tostring(i) or '') end
            inputKeys[key][i] = keys
        end
    end
    local nodes = 0
    local function ref(e, wheel, write)
        only(e, {state=true,parameter=true,constant=true,temporary=true,wheel=true})
        assert(e.wheel == nil or type(e.wheel) == 'boolean', 'invalid wheel reference flag')
        local prefix, key, section
        for _, kind in ipairs({'state','parameter','constant','temporary'}) do
            if e[kind] ~= nil then
                assert(not key, 'ambiguous reference')
                key = name(e[kind]); section = ({state='state',parameter='parameters',constant='constants',temporary='intermediate'})[kind]
                prefix = ({state='s',parameter='p',constant='c',temporary='v'})[kind]
            end
        end
        assert(key and d[section][key] ~= nil, 'unknown reference: ' .. tostring(key))
        assert(not write or prefix == 's' or prefix == 'v', 'cannot assign parameter or constant')
        if prefix == 's' then
            assert((e.wheel == true) == d.state[key].wheel, 'state wheel mismatch: ' .. key)
        else assert(not e.wheel, 'only state is wheel indexed') end
        if e.wheel then assert(wheel, 'wheel reference outside wheel phase') end
        return prefix .. '[' .. quote(key) .. ']' .. (e.wheel and '[i]' or '')
    end
    local expression
    expression = function(e, wheel, depth)
        depth = (depth or 0) + 1; nodes = nodes + 1
        assert(depth <= 64 and nodes <= 20000, 'expression complexity limit')
        if type(e) == 'number' then assert(finite(e), 'non-finite literal'); return string.format('%.17g', e) end
        if type(e) == 'boolean' then return tostring(e) end
        assert(type(e) == 'table', 'expressions must be structured nodes, not source strings')
        if e.literal ~= nil then
            only(e, {literal=true}); assert(type(e.literal) == 'string', 'invalid literal'); return quote(e.literal)
        end
        if e.dt then only(e, {dt=true}); assert(e.dt == true); return 'dt' end
        if e.absent then only(e, {absent=true}); assert(e.absent == true); return 'nil' end
        if e.context then
            only(e, {context=true})
            assert(e.context == 'car' or (e.context == 'wheel_index' and wheel), 'invalid context reference')
            return e.context == 'car' and 'car' or 'i'
        end
        if e.get_car then only(e, {get_car=true}); assert(e.get_car == true); return 'getCar()' end
        if e.access then
            only(e, {access=true})
            local access = e.access
            only(access, {object=true,keys=true,mode=true,fallback=true})
            local keys = array(access.keys)
            assert(#keys >= 1 and #keys <= 2, 'access path must contain one or two keys')
            local object, compiled = expression(access.object, wheel, depth), {}
            for j, key in ipairs(keys) do
                if type(key) == 'string' then name(key); compiled[j] = quote(key)
                else
                    only(key, {context=true})
                    assert(key.context == 'wheel_index' and wheel, 'only current wheel index is allowed')
                    compiled[j] = 'i'
                end
            end
            if access.mode == 'safe_field' then
                assert(#keys == 1 and access.fallback ~= nil, 'safe field requires one key and explicit fallback')
                return 'safeField(' .. object .. ',' .. compiled[1] .. ',' .. expression(access.fallback, wheel, depth) .. ')'
            end
            assert(access.fallback == nil, 'fallback only allowed for safe_field')
            if access.mode == 'protected_path' then
                assert(#keys == 2, 'protected path requires two keys')
                return 'protectedPath(' .. object .. ',' .. table.concat(compiled, ',') .. ')'
            end
            assert(access.mode == 'direct', 'invalid access mode')
            return '(' .. object .. ')[' .. table.concat(compiled, '][') .. ']'
        end
        if not e.op then return ref(e, wheel) end
        only(e, {op=true,args=true})
        local op, args = e.op, array(e.args)
        local arity = binary[op] and 2 or arities[op]
        assert(arity and #args == arity, 'invalid operator or arity: ' .. tostring(op))
        local a = {}
        for j, value in ipairs(args) do a[j] = expression(value, wheel, depth) end
        if binary[op] then return '(' .. a[1] .. ' ' .. binary[op] .. ' ' .. a[2] .. ')' end
        if op == 'not_nil' then return '(' .. a[1] .. ' ~= nil)' end
        if op == 'bool01' then return '(' .. a[1] .. ' and 1 or 0)' end
        return op .. '(' .. table.concat(a, ',') .. ')'
    end
    local outputKeys, outputSeen = {}, {}
    for group, outputs in pairs(d.outputs) do
        name(group); outputKeys[group] = {}
        for j, output in ipairs(array(outputs)) do
            only(output, {key=true,value=true,wheel=true})
            assert(type(output.key) == 'string' and output.key:match('^ngp_[%w_]+$'), 'invalid output key')
            assert(type(output.wheel) == 'boolean', 'output wheel flag required')
            expression(output.value, output.wheel) -- Validate even unreferenced output groups.
            outputKeys[group][j] = {}
            for i = 0, 3 do
                local key = output.key .. (output.wheel and tostring(i) or '')
                if output.wheel or i == 0 then assert(not outputSeen[key], 'duplicate output: ' .. key); outputSeen[key] = true end
                outputKeys[group][j][i] = key
            end
        end
    end
    local instructionFields = {
        set={set=true,value=true}, read={read=true,into=true},
        branch={branch=true,yes=true,no=true}, call={call=true},
        foreach_wheel={foreach_wheel=true}, publish={publish=true},
        stop={stop=true}, dt={dt=true},
    }
    local code, visiting, done = {}, {}, {}
    local compilePhase
    local function block(statements, wheel, depth)
        assert(depth <= 32, 'statement nesting limit')
        local lines = {}
        for _, stmt in ipairs(array(statements)) do
            only(stmt, {set=true,value=true,read=true,into=true,branch=true,yes=true,no=true,call=true,foreach_wheel=true,publish=true,stop=true,dt=true})
            local count, instruction = 0, nil
            for k in pairs(instructionFields) do
                if stmt[k] ~= nil then count = count + 1; instruction = k end
            end
            assert(count == 1, 'statement must have exactly one instruction')
            only(stmt, instructionFields[instruction])
            if stmt.set then
                assert(stmt.value ~= nil, 'assignment value required')
                lines[#lines + 1] = ref(stmt.set, wheel, true) .. '=' .. expression(stmt.value, wheel)
            elseif stmt.dt then lines[#lines + 1] = 'dt=' .. expression(stmt.dt, wheel)
            elseif stmt.read then
                local spec = assert(d.inputs[stmt.read], 'unknown input')
                assert(not spec.wheel or wheel, 'wheel input outside wheel phase')
                lines[#lines + 1] = ref({temporary=stmt.into}, wheel, true) .. '=read(' .. quote(stmt.read) .. ',' .. (spec.wheel and 'i' or '0') .. ')'
            elseif stmt.branch ~= nil then
                lines[#lines + 1] = 'if ' .. expression(stmt.branch, wheel) .. ' then\n' .. block(stmt.yes, wheel, depth + 1) .. '\nelse\n' .. block(stmt.no or {}, wheel, depth + 1) .. '\nend'
            elseif stmt.call then
                local phase = assert(d.formula[stmt.call], 'unknown phase')
                assert(phase.scope == 'global' or wheel, 'wheel phase outside wheel loop')
                compilePhase(stmt.call)
                lines[#lines + 1] = 'if f[' .. quote(stmt.call) .. '](dt,i,car) then return true end'
            elseif stmt.foreach_wheel then
                assert(not wheel, 'nested wheel loop')
                lines[#lines + 1] = 'for i=0,3 do\n' .. block(stmt.foreach_wheel, true, depth + 1) .. '\nend'
            elseif stmt.publish then
                local outputs = assert(d.outputs[stmt.publish], 'unknown output group')
                for j, output in ipairs(outputs) do
                    assert(not output.wheel or wheel, 'wheel output outside wheel loop')
                    local k = 'keys[' .. quote(stmt.publish) .. '][' .. j .. '][' .. (output.wheel and 'i' or '0') .. ']'
                    lines[#lines + 1] = 'emit(' .. k .. ',' .. expression(output.value, wheel) .. ')'
                end
            elseif stmt.stop then
                assert(stmt.stop == true); lines[#lines + 1] = 'do return true end'
            else error('invalid instruction') end
        end
        return table.concat(lines, '\n')
    end
    compilePhase = function(key)
        if done[key] then return end
        assert(not visiting[key], 'cyclic phase dependency: ' .. key); visiting[key] = true
        name(key)
        local phase = assert(d.formula[key], 'missing phase')
        only(phase, {scope=true,steps=true})
        assert(phase.scope == 'global' or phase.scope == 'wheel', 'invalid phase scope')
        local body = block(phase.steps, phase.scope == 'wheel', 1)
        code[#code + 1] = 'f[' .. quote(key) .. ']=function(dt,i,car)\n' .. body .. '\nend'
        visiting[key], done[key] = nil, true
    end
    only(d.entrypoints, {init=true,update=true})
    for _, key in ipairs({'init','update'}) do
        local phase = assert(d.formula[d.entrypoints[key]], 'missing entrypoint')
        assert(phase.scope == 'global', 'entrypoint must be global')
    end
    for key in pairs(d.formula) do compilePhase(key) end
    local source = 'return function(p,c,s,read,emit,keys,getCar)\nlocal v,f={},{}\n' .. table.concat(code, '\n') .. '\nreturn f[' .. quote(d.entrypoints.init) .. '], f[' .. quote(d.entrypoints.update) .. ']\nend'
    assert(#source <= 262144, 'compiled definition too large')
    local environment = {num=num, clamp=clamp, min=math.min, max=math.max,safeField=safeField,protectedPath=protectedPath}
    local chunk, err
    if loadstring then
        chunk, err = loadstring(source, '@definition/' .. d.module_id)
        if chunk then setfenv(chunk, environment) end
    else chunk, err = load(source, '@definition/' .. d.module_id, 't', environment) end
    assert(chunk, err)
    return {definition=d, factory=chunk(), inputKeys=inputKeys, outputKeys=outputKeys}
end
function M.instantiate(plan, readRaw, emit, getCar)
    local d, state, params, constants = plan.definition, {}, {}, {}
    for key, value in pairs(d.parameters) do params[key] = value end
    for key, value in pairs(d.constants) do constants[key] = value end
    for key, spec in pairs(d.state) do
        if spec.wheel then
            state[key] = {}
            for i=0,3 do state[key][i] = spec.initial end
        else state[key] = spec.initial end
    end
    local function read(key, i)
        local value
        for _, k in ipairs(plan.inputKeys[key][i]) do
            local ok, result = pcall(readRaw, k)
            value = ok and result or nil
            -- Preserve false on the final alias, and Lua's truthiness (0 is true).
            if ok then value = result end
            if value then return value end
        end
        return value
    end
    local function getCarSafely()
        if type(getCar) ~= 'function' then return nil end
        local ok, car = pcall(getCar, 0)
        if ok then return car end
        return nil
    end
    local init, update = plan.factory(params, constants, state, read, emit, plan.outputKeys, getCarSafely)
    return {params=params,state=state,debug=state,init=init,update=update}
end
return M
