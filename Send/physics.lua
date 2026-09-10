-- Transport only. ac.store publishes app telemetry; it does NOT apply tire forces.
-- Preserve the legacy write order/timing while other modules still read ac.load.
local M = {}
function M.new(store)
    local sender = {failures=0,lastError=nil,capability='telemetry_only'}
    function sender.write(key, value)
        if type(store) ~= 'function' then
            sender.failures = sender.failures + 1
            sender.lastError = 'ac.store unavailable: no output was applied'
            return false
        end
        local ok, err = pcall(store, key, value)
        if not ok then
            sender.failures = sender.failures + 1
            sender.lastError = tostring(err)
        end
        return ok
    end
    return sender
end
return M
