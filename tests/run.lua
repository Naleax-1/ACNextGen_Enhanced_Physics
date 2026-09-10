-- Run from repository root: luajit tests/run.lua (also supports Lua 5.4).
package.path = './?.lua;' .. package.path
local JSON = require('Engine.json')
local Compiler = require('Engine.compiler')
local Bus = require('Engine.result_bus')
local Brake = require('Engine.brake')
local Send = require('Send.physics')
local Loader = require('Engine.loader')
local Observer = require('Observer.observer')
local function read(path)
    local f = assert(io.open(path, 'rb')); local s = f:read('*a'); f:close(); return s
end
local text = read('modules/brake_fade.json')
local function definition() return JSON.decode(text) end
local plan = Compiler.prepare(definition())
local tests, checks, maxError = 0, 0, 0
local function equal(a, b, path)
    checks = checks + 1; path = path or 'root'
    assert(type(a) == type(b), path .. ' type differs: ' .. type(a) .. '/' .. type(b))
    if type(a) == 'table' then
        for key, value in pairs(a) do equal(value, b[key], path .. '.' .. tostring(key)) end
        for key in pairs(b) do assert(a[key] ~= nil, path .. ' extra key ' .. tostring(key)) end
    elseif type(a) == 'number' then
        if a ~= a or b ~= b then assert(a ~= a and b ~= b, path .. ': NaN mismatch')
        elseif a == math.huge or a == -math.huge then assert(a == b, path .. ': infinity mismatch')
        else
            local diff = math.abs(a - b); maxError = math.max(maxError, diff)
            assert(diff <= 1e-12 + 1e-12 * math.max(math.abs(a), math.abs(b)), path .. ': ' .. tostring(a) .. ' vs ' .. tostring(b))
        end
    else assert(a == b, path .. ': ' .. tostring(a) .. ' vs ' .. tostring(b)) end
end
local function test(name, fn)
    fn(); tests = tests + 1; print('PASS ' .. name)
end
local function rejects(fn)
    local ok = pcall(fn); assert(not ok, 'invalid input was accepted')
end
local function pair()
    local stores, trace = {{},{}}, {{},{}}
    local failures = {read={},write={}}
    local function input(which, key)
        trace[which][#trace[which]+1] = {'read',key}
        if failures.read[key] then error('injected input failure') end
        return stores[which][key]
    end
    local function output(which,key,value)
        trace[which][#trace[which]+1] = {'write',key,value}
        if failures.write[key] then error('injected output failure') end
        stores[which][key] = value
    end
    ac = {load=function(k) return input(1,k) end,store=function(k,v) output(1,k,v) end}
    local old = assert(loadfile('Legacy/modules/brake_fade.lua'))()
    local sender = Send.new(function(k,v) output(2,k,v) end)
    local bus = Bus.new(sender.write)
    local engine = Brake.new(bus,function(k) return input(2,k) end)
    local new = engine.attach(plan)
    local p = {old=old,new=new,bus=bus,sender=sender,failures=failures,stores=stores}
    function p.put(key,value) stores[1][key],stores[2][key] = value,value end
    function p.clearTrace() trace[1],trace[2] = {},{} end
    function p.compare()
        equal(old.state,new.state,'state'); equal(stores[1],stores[2],'stores'); equal(trace[1],trace[2],'trace')
    end
    function p.step(dt)
        p.clearTrace(); old.update(dt); new.update(dt); p.compare()
    end
    old.init(); new.init(); p.compare(); equal(old.params,new.params,'params')
    return p
end

test('strict JSON: escapes, arrays, unicode, duplicates, malformed and non-finite',function()
    local d = JSON.decode('{"v":[1,-2.5e-2,true,false,null],"s":"a\\n\\u65e5\\ud83d\\ude00"}')
    assert(d.v[5] == JSON.null and d.v[4] == false and d.v[2] == -.025)
    assert(d.s == 'a\n' .. string.char(230,151,165,240,159,152,128))
    for _, bad in ipairs({'', '{', '[1,]', '{"a":1,"a":2}', '{"a":false,"a":0}', '01', '1.', '+1', '1e', '1e999', 'NaN', 'true false', '"\\uD800"', '"\\uDC00"', '"\\x01"', '"\n"'}) do
        rejects(function() JSON.decode(bad) end)
    end
end)
test('compiler rejects arbitrary code, missing fields, bad references, cycles and arity',function()
    for _, mutate in ipairs({
        function(d) d.formula.update.steps[1].dt = 'os.execute("no")' end,
        function(d) d.parameters.minMu = math.huge end,
        function(d) d.constants = nil end,
        function(d) d.dependencies = JSON.null end,
        function(d) d.intermediate.raw = JSON.null end,
        function(d) d.formula.update.steps[1].yes = {} end,
        function(d) d.outputs.unused = {{key='ngp_unused',wheel=false,value='os.execute()'}} end,
        function(d) d.dependencies[2] = d.dependencies[1] end,
        function(d) d.formula.update.steps[1].dt = {state='mu',wheel='yes'} end,
        function(d) d.formula.update.steps[1].dt = {op='eval',args={}} end,
        function(d) d.formula.update.steps[1].dt = {op='add',args={1}} end,
        function(d) d.formula.update.steps[1].dt = {parameter='missing'} end,
        function(d) d.formula.update.steps = {{call='update'}} end,
        function(d) d.formula.update.steps = {{set={state='mu'},value=1}} end,
        function(d) d.formula.update.steps = {{set={parameter='minMu'},value=1}} end,
        function(d) d.formula.update.steps = {{set={state='mu',wheel=true},value=1}} end,
        function(d) d.outputs.wheel[1].key = '../arbitrary' end,
        function(d) d.outputs.wheel[2].key = d.outputs.wheel[1].key end,
        function(d) d.state.mu.initial = {} end,
        function(d) d.execution_order.policy = 'sort_by_category' end,
    }) do
        local d=definition(); mutate(d); rejects(function() Compiler.prepare(d) end)
    end
    local d = definition(); d.parameters.normalMu = .9
    local model = Compiler.instantiate(Compiler.prepare(d),function() end,function() end)
    model.init(); model.update(.1); assert(model.state.mu[0] == .9, 'JSON edit did not affect runtime')
end)
test('exact initialization, BAD DT, alias truthiness, thresholds and recovery',function()
    local p=pair()
    for _, dt in ipairs({0,-1,0/0,'bad',false,.001,.049,.05,.1,1}) do p.step(dt) end
    p.step(nil)
    for _, temp in ipairs({-100,25,349.999999,350,350.000001,550,749.999999,750,750.000001,1200}) do
        for i=0,3 do p.put('ngp_brake_disc_temp_'..i,temp+i) end
        p.step(.1)
    end
    -- Last false alias is not nil; primary false must fall through; zero must not.
    p.put('ngp_brake_disc_temp_0',false);p.put('ngp_brake_temp_0',false);p.step(.1)
    p.put('ngp_brake_disc_temp_0',0);p.put('ngp_brake_temp_0',1000);p.step(.1)
    p.put('ngp_brake_disc_temp_0','550');p.step(.1)
    p.put('ngp_brake_disc_temp_0',0/0);p.step(.1)
    p.put('ngp_brake_disc_temp_0',math.huge);p.step(.1)
    for i=0,3 do p.put('ngp_brake_disc_temp_'..i,25) end
    for i=1,1000 do p.step(.1) end
    assert(p.new.state.avgMu > .999999)
    p.clearTrace();p.old.init();p.new.init();p.compare() -- re-init preserves wheel memory
end)
test('seeded mixed-rate replay with sparse aliases, heating, cooling and API failures',function()
    local p=pair(); local seed=971
    local function random() seed=(seed*16807)%2147483647; return seed/2147483647 end
    local keys={}
    for _,spec in pairs(definition().inputs) do
        for _,key in ipairs(spec.keys) do
            if spec.wheel then for i=0,3 do keys[#keys+1]=key..i end else keys[#keys+1]=key end
        end
    end
    table.sort(keys)
    local dts={1/240,1/144,1/60,.033,.05,.1,.2,1.2,0,-.01}
    for frame=1,3000 do
        for _,key in ipairs(keys) do
            local r=random(); local value
            if r < .15 then value=nil
            elseif r < .2 then value=false
            elseif r < .25 then value='not-a-number'
            elseif r < .3 then value='0.8'
            elseif key:find('temp') then value=random()*1300-100
            else value=random()*3-.5 end
            p.put(key,value)
            p.failures.read[key] = frame%79 == 0
        end
        p.failures.write.ngp_brake_mu_2 = frame%43 == 0
        p.step(dts[frame%#dts+1])
    end
    assert(p.sender.failures > 0 and p.sender.lastError)
end)
test('observer snapshots detached, rates independent, Send survives observer failure',function()
    local p=pair();p.step(.1)
    local snapshot=p.bus.snapshot();snapshot.state.brake_fade.mu[0]=-500
    assert(p.new.state.mu[0] ~= -500)
    for _,hz in ipairs({10,20,30,60}) do
        local observer=Observer.new(p.bus.snapshot,hz)
        for i=1,600 do observer.update(1/600) end
        assert(observer.samples >= hz and observer.samples <= hz+1)
        observer.enabled=false;local count=observer.samples
        for i=1,100 do observer.update(.1);p.step(.1) end
        assert(observer.samples==count)
    end
    local broken=Observer.new(function() error('observer failure') end,20)
    assert(not pcall(broken.update,.1));p.step(.1)
    assert(p.stores[2].ngp_brake_fade_update_count == p.new.state.updateCount)
    local unavailable=Send.new(nil);assert(not unavailable.write('x',1));assert(unavailable.capability=='telemetry_only')
end)
local catalog=JSON.decode(read('Migration_Catalog.json'))
local schedule,hz={},{}
for _,e in ipairs(catalog.execution_order) do
    schedule[#schedule+1]={name=e.module_id,enabled=e.enabled,critical=e.critical,diagnostic=e.diagnostic}
    hz[e.module_id]=e.target_hz
end
local function load(files, documents)
    return Loader.new({schedule=schedule,hz=hz,files=files,bus=Bus.new(),readRaw=function() end,
        read=function(path) local file=path:match('([^/]+)$'); return assert(documents[file],'missing file') end})
end
test('loader validates order, isolates corruption, duplicates and missing dependencies',function()
    local good=load({'brake_fade.json'},{['brake_fade.json']=text});assert(good.models.brake_fade)
    local bad=load({'brake_fade.json'},{['brake_fade.json']='{'})
    assert(not bad.models.brake_fade and bad.errors.brake_fade)
    local mixed=load({'broken.json','brake_fade.json'},{['broken.json']='{',['brake_fade.json']=text})
    assert(mixed.models.brake_fade and mixed.errors.broken)
    local duplicate=load({'brake_fade.json','brake_fade.json'},{['brake_fade.json']=text})
    assert(not duplicate.models.brake_fade and duplicate.errors.brake_fade)
    local missing=load({},{});assert(not missing.models.brake_fade and missing.errors.brake_fade)
    local traversal=load({'../brake_fade.json'},{});assert(next(traversal.errors))
    local malformedList=load({12,'brake_fade.json'},{['brake_fade.json']=text})
    assert(malformedList.models.brake_fade and malformedList.errors['discovery:1'])
    assert(load('invalid list',{}).errors.discovery)
    -- Edit serialized metadata independently of the physics payload.
    local reordered=text:gsub('"host_index": 45','"host_index": 44')
    assert(load({'brake_fade.json'},{['brake_fade.json']=reordered}).errors.brake_fade)
    local changedHz=text:gsub('"target_hz": 10','"target_hz": 60')
    assert(load({'brake_fade.json'},{['brake_fade.json']=changedHz}).errors.brake_fade)
    local required=text:gsub('"required": false','"required": true',1)
    assert(load({'brake_fade.json'},{['brake_fade.json']=required}).errors.brake_fade)
end)
test('runtime does not read, parse or compile definitions again',function()
    local loader=load({'brake_fade.json'},{['brake_fade.json']=text})
    local model=assert(loader.models.brake_fade);model.init()
    local oldOpen,oldDecode,oldCompile=io.open,JSON.decode,Compiler.prepare
    io.open=function() error('runtime I/O') end
    JSON.decode=function() error('runtime parse') end
    Compiler.prepare=function() error('runtime compile') end
    for i=1,1000 do model.update(.1) end
    io.open,JSON.decode,Compiler.prepare=oldOpen,oldDecode,oldCompile
    equal(loader.stats,{reads=1,parses=1,compiles=1})
end)
test('original host and pilot preserve init order, dt accumulation and downstream visibility',function()
    local Config=require('Engine.config')
    local function run(path,mode,brokenObserver)
        local calls,stored={},{}
        Config.mode=mode
        package.loaded['Engine.migration']=nil
        package.loaded['Observer.observer']=nil
        if brokenObserver then package.preload['Observer.observer']=function() error('missing optional observer') end
        else package.preload['Observer.observer']=nil end
        io.scanDir=function() return {'brake_fade.json'} end
        ac={getCar=function() return {wheels={}} end,load=function(k) return stored[k] end,store=function(k,v) stored[k]=v end,log=function() end}
        for _,entry in ipairs(schedule) do
            local name=entry.name
            package.loaded['modules.'..name]=nil
            package.preload['modules.'..name]=function()
                if name=='brake_fade' then return assert(loadfile('Legacy/modules/brake_fade.lua'))() end
                return {init=function() calls[#calls+1]={'init',name} end,update=function(dt)
                    calls[#calls+1]={'update',name,dt}
                    if name=='brake_system' then
                        for i=0,3 do stored['ngp_brake_disc_temp_'..i]=500+#calls%600 end
                    elseif name=='brake_lock' then
                        calls[#calls+1]={'brake_mu_seen',stored.ngp_brake_mu_0}
                        for i=0,3 do stored['ngp_brake_lock_'..i]=#calls%10/10 end
                    end
                end}
            end
        end
        local oldClock=os.clock;os.clock=function() return 0 end
        assert(loadfile(path))()
        local times={1/144,1/60,.001,.033,.1,2,0,-1,'bad',0/0}
        for i=1,400 do update(times[i%#times+1]) end
        os.clock=oldClock
        for key in pairs(stored) do if key:match('^ngp_prof_') then stored[key]=nil end end
        if mode=='pilot' then assert(not package.loaded['modules.brake_fade'],'pilot ran legacy brake_fade too') end
        return {calls=calls,stored=stored}
    end
    local baseline=run('Legacy/ACNextGen.lua','legacy',false)
    equal(baseline,run('ACNextGen.lua','legacy',false),'default_host')
    equal(baseline,run('ACNextGen.lua','pilot',false),'pilot_host')
    equal(baseline,run('ACNextGen.lua','pilot',true),'observer_failure_host')
    Config.mode='legacy';package.preload['Observer.observer']=nil;package.loaded['Observer.observer']=nil
end)
print(string.format('PASS %d suites / %d comparisons / max absolute numeric difference %.17g',tests,checks,maxError))
