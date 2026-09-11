-- One domain engine owns all prepared brake definitions, not require-per-definition.
-- Dispatch remains at each original host slot: merging slots would change feedback.
local Compiler = require('Engine.compiler')
local M = {}
function M.new(bus, readRaw, getCar)
    local engine = {models={}}
    function engine.attach(plan)
        local id = plan.definition.module_id
        assert(not engine.models[id], 'duplicate brake model')
        local model = Compiler.instantiate(plan, readRaw, function(key, value) bus.emit(id, key, value) end, getCar)
        local function guarded(fn, ...)
            local ok, err = pcall(fn, ...)
            if not ok then bus.fail(id, err); error(err, 0) end
            bus.complete(id, model.state)
        end
        local adapter = {state=model.state,debug=model.state,params=model.params}
        function adapter.init() guarded(model.init, 0) end
        function adapter.update(dt, car) guarded(model.update, dt, nil, car) end
        -- Preserve existing public getter behavior for the migrated module.
        if id == 'brake_fade' then
            function adapter.getMu(i) return model.state.mu[i] or 1.0 end
            function adapter.getFade(i) return model.state.fade[i] or 0.0 end
            function adapter.getTargetMu(i) return model.state.targetMu[i] or 1.0 end
        elseif id == 'brake_system' then
            function adapter.getTemp(i) return model.state.discTemp[i] or model.params.ambient end
            function adapter.getRootHeat(i) return model.state.rootHeat[i] or 0.0 end
        end
        engine.models[id] = adapter
        return adapter
    end
    return engine
end
return M
