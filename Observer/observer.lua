-- Receives only a snapshot capability. No AC APIs, mutable physics state or Send.
local M = {}
function M.new(snapshot, hz)
    assert(type(snapshot) == 'function', 'snapshot reader required')
    assert(hz == 10 or hz == 20 or hz == 30 or hz == 60, 'observer rate must be 10/20/30/60 Hz')
    local interval, elapsed, latest = 1 / hz, 1 / hz, nil
    local observer = {enabled=true,samples=0}
    function observer.update(dt)
        if not observer.enabled then return end
        elapsed = elapsed + dt
        if elapsed < interval then return end
        elapsed = elapsed % interval
        latest = snapshot()
        observer.samples = observer.samples + 1
    end
    function observer.drawUI(ui)
        if not observer.enabled or not ui then return end
        ui.separator()
        ui.text('Definition pilot: brake_fade only / other modules remain legacy')
        ui.text('Output: ac.store telemetry only (not force application)')
        if not latest then ui.text('No definition results sampled'); return end
        for id, err in pairs(latest.errors) do ui.text(id .. ': ' .. err) end
        local s = latest.state.brake_fade
        if not s then ui.text('Definition unavailable; see migration errors'); return end
        ui.text('Brake fade: ' .. tostring(s.status))
        for i = 0, 3 do
            ui.text(string.format('W%d Mu %.6f Target %.6f Fade %.6f Temp %.3f', i,
                s.mu[i], s.targetMu[i], s.fade[i], s.temp[i]))
        end
    end
    return observer
end
return M
