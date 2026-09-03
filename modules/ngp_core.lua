-- ngp_core.lua
-- Shared helpers for ACNextGen V1.1

local M = {}

M.version = "1.1-stable-core"

local lastStore = {}

function M.num(v, fallback)
    local n = tonumber(v)
    if n == nil then return fallback or 0 end
    if n ~= n then return fallback or 0 end
    return n
end

function M.bool01(v)
    return v and 1 or 0
end

function M.clamp(v, a, b)
    v = M.num(v, a)
    if v < a then return a end
    if v > b then return b end
    return v
end

function M.abs(v)
    v = M.num(v, 0)
    if v < 0 then return -v end
    return v
end

function M.sign(v)
    v = M.num(v, 0)
    if v > 0 then return 1 end
    if v < 0 then return -1 end
    return 0
end

function M.lerp(a, b, t)
    t = M.clamp(t, 0, 1)
    return a + (b - a) * t
end

function M.invLerp(a, b, v)
    if a == b then return 0 end
    return M.clamp((v - a) / (b - a), 0, 1)
end

function M.lowPass(current, target, dt, tau)
    current = M.num(current, 0)
    target = M.num(target, 0)
    dt = M.num(dt, 0)
    tau = math.max(M.num(tau, 0.05), 0.0001)

    local a = dt / (tau + dt)
    return current + (target - current) * M.clamp(a, 0, 1)
end

function M.approach(current, target, step)
    current = M.num(current, 0)
    target = M.num(target, 0)
    step = math.max(M.num(step, 0), 0)

    if current < target then
        return math.min(current + step, target)
    elseif current > target then
        return math.max(current - step, target)
    end

    return target
end

function M.safeStore(key, value)
    if not key then return false end
    local ok = pcall(function()
        ac.store(key, value)
    end)
    return ok
end

function M.safeLoad(key, fallback)
    if not key then return fallback end

    local ok, v = pcall(function()
        return ac.load(key)
    end)

    if not ok or v == nil then
        return fallback
    end

    return v
end

function M.storeInterval(key, value, interval)
    interval = interval or 0.10

    local now = 0
    pcall(function()
        now = os.clock()
    end)

    if now <= 0 then
        M.safeStore(key, value)
        return true
    end

    local last = lastStore[key] or 0
    if now - last >= interval then
        lastStore[key] = now
        M.safeStore(key, value)
        return true
    end

    return false
end

function M.getCar()
    local ok, car = pcall(function()
        return ac.getCar(0)
    end)

    if ok and car then
        return car
    end

    return nil
end

function M.getWheel(car, i)
    if not car then return nil end
    if not car.wheels then return nil end

    local ok, w = pcall(function()
        return car.wheels[i]
    end)

    if ok then return w end
    return nil
end

function M.wheelValue(car, i, key, fallback)
    local w = M.getWheel(car, i)
    if not w then return fallback or 0 end

    local ok, v = pcall(function()
        return w[key]
    end)

    if ok and v ~= nil then
        return M.num(v, fallback or 0)
    end

    return fallback or 0
end

function M.carValue(car, key, fallback)
    if not car then return fallback or 0 end

    local ok, v = pcall(function()
        return car[key]
    end)

    if ok and v ~= nil then
        return M.num(v, fallback or 0)
    end

    return fallback or 0
end

function M.speedKmh(car)
    if not car then return 0 end

    local ok, v = pcall(function()
        return car.speedKmh
    end)

    if ok and v ~= nil then
        return M.num(v, 0)
    end

    local vel = car.velocity
    if vel and vel.length then
        return M.num(vel:length(), 0) * 3.6
    end

    return 0
end

function M.setModuleStatus(name, ok, err)
    if not name then return end

    local prefix = "ngp_mod_" .. name

    M.safeStore(prefix .. "_ok", ok and 1 or 0)

    if err then
        M.safeStore(prefix .. "_err", tostring(err))
    elseif ok then
        M.safeStore(prefix .. "_err", "")
    end
end

function M.moduleAlive(name)
    if not name then return end
    M.safeStore("ngp_mod_" .. name .. "_alive", 1)
end

return M