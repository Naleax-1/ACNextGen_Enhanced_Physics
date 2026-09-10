-- Real repository modules with synthetic AC API structs (not a CSP emulator).
-- Loaded independently in each test VM: no legacy module bodies are stubbed.
return function(mode, baselineHost, corruptDefinition, observerFailure)
    package.path='./?.lua;'..package.path
    local api={store={},logs={},reads=0,writes=0,hasCar=true,uiCalls=0}
    local car={wheels={}}
    local function vec(x,y,z) return {x=x,y=y,z=z} end
    ac={
        getCar=function() if api.hasCar then return car end end,
        getCarID=function() return 'synthetic_car' end,
        getCarConfig=function() return nil end,
        getFolder=function() return './tests/no_car_files' end,
        getSim=function() return {ambientTemperature=25,roadTemperature=30} end,
        FolderID={Root=1,ContentCars=2},
        load=function(k) api.reads=api.reads+1;return api.store[k] end,
        store=function(k,v) api.writes=api.writes+1;api.store[k]=v end,
        log=function(message) api.logs[#api.logs+1]=message end,
    }
    ui={text=function() api.uiCalls=api.uiCalls+1 end,separator=function() end}
    -- Remove nondeterministic profiling from both sides; preserve scheduling/timers.
    os.clock=function() return 0 end
    local realOpen=io.open
    io.scanDir=function() return {'brake_fade.json'} end
    io.open=function(path,mode)
        assert(not mode or not mode:find('[wa+]'), 'unexpected write during integration replay')
        if path:match('%.lua$') or path:match('%.json$') then
            if corruptDefinition and path:match('modules/brake_fade%.json$') then
                return {read=function() return '{broken' end,close=function() end}
            end
            return realOpen(path,mode)
        end
        return nil,'Synthetic car data is intentionally absent'
    end
    local config=require('Engine.config');config.mode=mode
    local migration
    local module=require('Engine.migration')
    local create=module.new
    module.new=function(...)
        migration=create(...)
        if observerFailure and migration.observer then
            migration.observer.update=function() error('injected observer update failure') end
        end
        return migration
    end
    assert(loadfile(baselineHost and 'Legacy/ACNextGen.lua' or 'ACNextGen.lua'))()
    function api.step(frame)
        local phase=frame/17
        api.hasCar=not (frame>=80 and frame<=85)
        car.speedKmh=math.abs(math.sin(phase))*180
        car.rpm=800+math.abs(math.sin(phase*.7))*7200
        car.gear=frame%7;car.gas=(frame%20)/20
        car.brake=frame%30<12 and .85 or .02
        car.clutch=frame%17<3 and .1 or 1
        car.steer=math.sin(phase)*.8;car.handbrake=frame%53==0 and 1 or 0
        car.mass=1350;car.fuel=40
        car.localVelocity=vec(.2*math.sin(phase),.01,car.speedKmh/3.6)
        car.velocity=car.localVelocity
        car.localAngularVelocity=vec(.1*math.sin(phase),.2*math.sin(phase),.04)
        car.acceleration=vec(math.sin(phase)*4,.2,math.cos(phase)*6)
        car.accelerationG=vec(math.sin(phase)*.4,.02,math.cos(phase)*.6)
        car.wheels={}
        if frame%31~=0 then
            for i=0,3 do
                car.wheels[i]={load=2800+900*math.sin(phase+i),slipRatio=math.sin(phase+i)*.4,
                    slipAngle=math.cos(phase+i)*.15,angularSpeed=car.speedKmh/(3.6*.32),
                    tyreCoreTemperature=75+i,tyreTemperature=80+i,brakeTemperature=350+frame,
                    suspensionTravel=.07+.035*math.sin(phase+i),radius=.32,pressure=26,
                    isInContact=frame%47~=0}
            end
        end
        api.reads,api.writes=0,0
        local dt=({1/144,1/60,.033,.001,.1})[frame%5+1]
        if frame==60 then dt=2 end
        update(dt)
        if frame%40==0 then windowMain() end
        local states={}
        for key,value in pairs(package.loaded) do
            local id=key:match('^modules%.([%w_]+)$')
            if id and type(value)=='table' and type(value.state)=='table' then states[id]=value.state end
        end
        if migration and migration.loader and migration.loader.models.brake_fade then
            states.brake_fade=migration.loader.models.brake_fade.state
        end
        return {states=states,store=api.store,reads=api.reads,writes=api.writes}
    end
    function api.status()
        local errors={}
        if migration then for k,v in pairs(migration.errors) do errors[k]=v end end
        return {migration_errors=errors,ui_calls=api.uiCalls,
            legacy_brake_loaded=type(package.loaded['modules.brake_fade'])=='table',
            runtime_active_errors=api.store.ngp_runtime_active_error_count,
            runtime_last_error=api.store.ngp_runtime_last_error,
            pilot_active=migration and migration.loader and migration.loader.models.brake_fade~=nil or false}
    end
    return api
end
