-- Private result ownership; observers receive detached data, never engine tables.
local M = {}
local function copy(value)
    if type(value) ~= 'table' then return value end
    local out = {}
    for key, item in pairs(value) do out[key] = copy(item) end
    return out
end
function M.new(send)
    local values, states, errors, generations = {}, {}, {}, {}
    local bus = {}
    function bus.emit(id, key, value)
        local result = values[id]
        if not result then result = {}; values[id] = result end
        result[key] = value
        if send then send(key, value) end
    end
    function bus.complete(id, state)
        states[id] = state
        generations[id] = (generations[id] or 0) + 1
        errors[id] = nil
    end
    function bus.fail(id, err) errors[id] = tostring(err) end
    function bus.snapshot()
        return {values=copy(values),state=copy(states),errors=copy(errors),generation=copy(generations)}
    end
    return bus
end
return M
