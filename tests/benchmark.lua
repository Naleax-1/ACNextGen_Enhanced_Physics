-- Synthetic brake_fade microbenchmark, NOT an AC/FPS performance gate.
-- Load once per fresh interpreter. Warm-up and startup are outside timed updates.
package.path = './?.lua;' .. package.path
local JSON = require('Engine.json')
local Compiler = require('Engine.compiler')
local Brake = require('Engine.brake')
local Bus = require('Engine.result_bus')
local Send = require('Send.physics')
local f = assert(io.open('modules/brake_fade.json', 'rb'))
local definitionText = f:read('*a'); f:close()
local function instance(mode)
    local store = {ngp_condition_brake=.2,ngp_brake_cooling=.8,ngp_brake_input_smoothed=.7}
    for i=0,3 do
        store['ngp_brake_disc_temp_'..i] = 550 + i * 50
        store['ngp_brake_lock_'..i] = .15
        store['ngp_brake_root_heat_'..i] = .3
        store['ngp_tire_contact_loss_'..i] = .1
    end
    local api = {load=function(k) return store[k] end,store=function(k,v) store[k]=v end}
    ac = api
    local model
    if mode == 'legacy' then
        model = assert(loadfile('Legacy/modules/brake_fade.lua'))()
    else
        local sender = Send.new(api.store)
        local bus = Bus.new(sender.write)
        local engine = Brake.new(bus,api.load)
        model = engine.attach(Compiler.prepare(JSON.decode(definitionText)))
    end
    model.init()
    return model
end
local function loop(model,n)
    for i=1,n do model.update(.05) end
end
local function sample(mode,updates)
    collectgarbage('collect')
    local start = os.clock()
    local model = instance(mode)
    local startupMs = (os.clock() - start) * 1000
    loop(model,2000)
    collectgarbage('collect')
    start = os.clock();loop(model,updates)
    local elapsed = os.clock() - start
    collectgarbage('collect')
    local before = collectgarbage('count')
    collectgarbage('stop')
    loop(model,1000)
    local transient = collectgarbage('count') - before
    collectgarbage('restart');collectgarbage('collect')
    local retained = collectgarbage('count') - before
    return {cpu_us_per_update=elapsed/updates*1e6,startup_ms=startupMs,
        transient_heap_delta_kib_1000_updates=transient,retained_heap_delta_kib_1000_updates=retained}
end
local report={runtime=jit and jit.version or _VERSION,updates_per_sample=20000,warmup_updates=2000,
    rounds=7,legacy={},pilot={},observer_enabled=false,dt=.05,
    scope='brake_fade only; table-backed mock ac.load/ac.store; default GC; no AC, CSP, rendering or real physics output'}
-- Alternate order to reduce systematic warm-cache/drift bias.
for round=1,report.rounds do
    local first,second='legacy','pilot'
    if round%2==0 then first,second=second,first end
    report[first][round]=sample(first,report.updates_per_sample)
    report[second][round]=sample(second,report.updates_per_sample)
end
return report
