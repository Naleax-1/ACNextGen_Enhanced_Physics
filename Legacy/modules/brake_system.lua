---@diagnostic disable: undefined-global

--============================================================
-- brake_system.lua
-- ACNextGen V1.1.5 Stable
-- Brake Thermal Core
--============================================================

local M = {}

M.params = {
    ambient = 25.0,

    heatScale = 0.080,
    coolBase  = 0.015,
    coolSpeed = 0.002,

    maxTemp = 900.0,

    frontBias = 1.08,
    rearBias  = 0.92,

    brakeTau = 0.040,

    rootHeatGain = 0.10,
    lockHeatGain = 0.08,
    contactLossHeatGain = 0.05,
    contactLossCoolingGain = 0.06,

    loadRef = 3500.0,

    maxRootHeat = 0.35,

    debugStoreInterval = 0.25,
}

local state = {
    discTemp = { [0] = 25.0, [1] = 25.0, [2] = 25.0, [3] = 25.0 },
    rootHeat = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    lock = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    contactLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    load = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    heat = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    cool = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    brakeInput = 0.0,
    brakeInputSmoothed = 0.0,
    speedKmh = 0.0,
    cooling = 0.0,

    avgTemp = 25.0,
    maxTempNow = 25.0,
    rootHeatAvg = 0.0,
    avgHeat = 0.0,
    avgCool = 0.0,

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    lockLinked = false,
    tireLinked = false,
    loadLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

local function safeNumber(value, defaultValue)
    local n = tonumber(value)
    if n == nil or n ~= n then
        return defaultValue or 0.0
    end
    return n
end

local function clamp(v, minValue, maxValue)
    v = safeNumber(v, minValue)
    if v < minValue then return minValue end
    if v > maxValue then return maxValue end
    return v
end

local function safeLoadRaw(key)
    if not ac or not ac.load then
        return nil
    end

    local ok, value = pcall(function()
        return ac.load(key)
    end)

    if not ok then
        return nil
    end

    return value
end

local function safeLoad(key, defaultValue)
    local value = safeLoadRaw(key)
    if value == nil then
        return defaultValue or 0.0
    end
    return safeNumber(value, defaultValue or 0.0)
end

local function safeStore(key, value)
    if not ac or not ac.store then
        return false
    end

    local ok = pcall(function()
        ac.store(key, value)
    end)

    return ok == true
end

local function safeField(obj, field, defaultValue)
    if not obj then
        return defaultValue
    end

    local ok, value = pcall(function()
        return obj[field]
    end)

    if not ok or value == nil then
        return defaultValue
    end

    return value
end

local function lowPass(current, target, dt, tau)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    dt = safeNumber(dt, 0.0)
    tau = math.max(safeNumber(tau, 0.001), 0.0001)

    local a = dt / (tau + dt)
    a = clamp(a, 0.0, 1.0)

    return current + (target - current) * a
end

local function updateDebugGate(dt)
    state.debugStoreTimer =
        (state.debugStoreTimer or 0.0)
        +
        safeNumber(dt, 0.0)

    if state.debugStoreTimer >= M.params.debugStoreInterval then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    else
        state.debugStoreNow = false
    end
end

local function getCar()
    if ac and ac.getCar then
        local ok, car = pcall(function()
            return ac.getCar(0)
        end)

        if ok then
            return car
        end
    end

    return nil
end

local function getWheel(car, index)
    if not car or not car.wheels then
        return nil
    end

    local ok, wheel = pcall(function()
        return car.wheels[index]
    end)

    if ok then
        return wheel
    end

    return nil
end

local function readWheelLoad(wheel, index)
    local raw =
        safeLoadRaw("ngp_wheel_load_" .. index)
        or
        safeLoadRaw("ngp_load_wheel_" .. index)
        or
        safeLoadRaw("ngp_dlt_load_" .. index)
        or
        safeLoadRaw("ngp_sprung_load_" .. index)

    if raw ~= nil then
        state.loadLinked = true
        return math.max(safeNumber(raw, 0.0), 0.0)
    end

    if wheel then
        return math.max(safeNumber(safeField(wheel, "load", 0.0), 0.0), 0.0)
    end

    return 0.0
end

local function readLock(index)
    local raw = safeLoadRaw("ngp_brake_lock_" .. index)

    if raw ~= nil then
        state.lockLinked = true
    end

    state.lock[index] =
        clamp(
            safeNumber(raw, 0.0),
            0.0,
            1.0
        )
end

local function readContactLoss(index)
    local raw =
        safeLoadRaw("ngp_tire_contact_loss_" .. index)
        or
        safeLoadRaw("ngp_tcr_contact_loss_" .. index)

    if raw == nil then
        local quality =
            safeLoadRaw("ngp_contact_quality_" .. index)
            or
            safeLoadRaw("ngp_tc_contact_" .. index)
            or
            safeLoadRaw("ngp_tire_contact_" .. index)

        if quality ~= nil then
            raw = 1.0 - safeNumber(quality, 1.0)
        end
    end

    if raw ~= nil then
        state.tireLinked = true
    end

    state.contactLoss[index] =
        clamp(
            safeNumber(raw, 0.0),
            0.0,
            1.0
        )
end

local function axleBias(index)
    if index <= 1 then
        return M.params.frontBias
    end

    return M.params.rearBias
end

local function updateRootHeat(index)
    local bias = axleBias(index)
    local biasHeat = math.max(bias - 1.0, 0.0) * M.params.rootHeatGain

    state.rootHeat[index] =
        clamp(
            biasHeat
            +
            state.lock[index] * M.params.lockHeatGain
            +
            state.contactLoss[index] * M.params.contactLossHeatGain,
            0.0,
            M.params.maxRootHeat
        )
end

local function exportWheel(index)
    safeStore("ngp_brake_temp_" .. index, state.discTemp[index])
    safeStore("ngp_brake_disc_temp_" .. index, state.discTemp[index])
    safeStore("ngp_brake_root_heat_" .. index, state.rootHeat[index])

    if state.debugStoreNow then
        safeStore("ngp_brake_heat_" .. index, state.heat[index])
        safeStore("ngp_brake_cool_" .. index, state.cool[index])
        safeStore("ngp_brake_load_" .. index, state.load[index])
        safeStore("ngp_brake_contact_loss_" .. index, state.contactLoss[index])
        safeStore("ngp_brake_lock_input_" .. index, state.lock[index])
    end
end

local function exportGlobal()
    safeStore("ngp_brake_input", state.brakeInput)
    safeStore("ngp_brake_input_smoothed", state.brakeInputSmoothed)
    safeStore("ngp_brake_speed_kmh", state.speedKmh)
    safeStore("ngp_brake_cooling", state.cooling)

    safeStore("ngp_brake_avg_temp", state.avgTemp)
    safeStore("ngp_brake_temp_avg", state.avgTemp)
    safeStore("ngp_brake_max_temp", state.maxTempNow)
    safeStore("ngp_brake_temp_max", state.maxTempNow)
    safeStore("ngp_brake_root_heat_avg", state.rootHeatAvg)

    safeStore("ngp_brake_status", state.status)
    safeStore("ngp_brake_system_status", state.status)
    safeStore("ngp_brake_system_update_count", state.updateCount)
    safeStore("ngp_brake_system_wheels_valid", state.wheelsValid and 1 or 0)

    if state.debugStoreNow then
        safeStore("ngp_brake_system_lock_linked", state.lockLinked and 1 or 0)
        safeStore("ngp_brake_system_tire_linked", state.tireLinked and 1 or 0)
        safeStore("ngp_brake_system_load_linked", state.loadLinked and 1 or 0)
        safeStore("ngp_brake_heat_avg", state.avgHeat)
        safeStore("ngp_brake_cool_avg", state.avgCool)
    end
end

function M.init()
    state.status = "INIT"

    for i = 0, 3 do
        exportWheel(i)
    end

    exportGlobal()
end

function M.update(dt, car, runtime)
    state.updateCount =
        (state.updateCount or 0)
        +
        1

    dt = safeNumber(dt, 0.0)

    updateDebugGate(dt)

    if dt <= 0.0 then
        state.status = "BAD DT"
        exportGlobal()
        return
    end

    car = car or getCar()

    if not car or not car.wheels then
        state.status = "NO CAR"
        state.wheelsValid = false
        exportGlobal()
        return
    end

    state.status = "RUNNING"
    state.wheelsValid = true

    state.lockLinked = false
    state.tireLinked = false
    state.loadLinked = false

    state.brakeInput =
        clamp(
            safeNumber(
                safeField(car, "brake", 0.0),
                0.0
            ),
            0.0,
            1.0
        )

    state.brakeInputSmoothed =
        lowPass(
            state.brakeInputSmoothed,
            state.brakeInput,
            dt,
            M.params.brakeTau
        )

    state.speedKmh =
        math.max(
            safeNumber(
                safeField(car, "speedKmh", 0.0),
                0.0
            ),
            0.0
        )

    state.cooling =
        math.max(
            M.params.coolBase
            +
            state.speedKmh * M.params.coolSpeed,
            0.0
        )

    local sumTemp = 0.0
    local sumRoot = 0.0
    local sumHeat = 0.0
    local sumCool = 0.0
    local maxTempNow = M.params.ambient

    for i = 0, 3 do
        local wheel = getWheel(car, i)

        state.load[i] =
            readWheelLoad(wheel, i)

        readLock(i)
        readContactLoss(i)
        updateRootHeat(i)

        local loadNorm =
            clamp(
                state.load[i] / math.max(M.params.loadRef, 1.0),
                0.0,
                2.0
            )

        local bias = axleBias(i)

        local heat =
            state.brakeInputSmoothed
            *
            loadNorm
            *
            M.params.loadRef
            *
            M.params.heatScale
            *
            bias
            *
            (1.0 + state.rootHeat[i])
            *
            dt

        local coolingScale =
            math.max(
                state.cooling
                *
                (1.0 - state.contactLoss[i] * M.params.contactLossCoolingGain),
                0.0
            )

        local cool =
            (state.discTemp[i] - M.params.ambient)
            *
            coolingScale
            *
            dt

        state.discTemp[i] =
            clamp(
                state.discTemp[i] + heat - cool,
                M.params.ambient,
                M.params.maxTemp
            )

        state.heat[i] = heat
        state.cool[i] = cool

        sumTemp = sumTemp + state.discTemp[i]
        sumRoot = sumRoot + state.rootHeat[i]
        sumHeat = sumHeat + heat
        sumCool = sumCool + cool

        if state.discTemp[i] > maxTempNow then
            maxTempNow = state.discTemp[i]
        end

        exportWheel(i)
    end

    state.avgTemp = sumTemp * 0.25
    state.maxTempNow = maxTempNow
    state.rootHeatAvg = sumRoot * 0.25
    state.avgHeat = sumHeat * 0.25
    state.avgCool = sumCool * 0.25

    exportGlobal()
end

function M.getTemp(i)
    return state.discTemp[i] or M.params.ambient
end

function M.getRootHeat(i)
    return state.rootHeat[i] or 0.0
end

function M.debugStr(index)
    if index ~= nil then
        local i = clamp(index, 0, 3)

        return string.format(
            "W%d Temp %.1f / Heat %.3f / Cool %.3f / Load %.0f / Lock %.2f / Root %.3f / ContactLoss %.2f",
            i,
            state.discTemp[i] or M.params.ambient,
            state.heat[i] or 0.0,
            state.cool[i] or 0.0,
            state.load[i] or 0.0,
            state.lock[i] or 0.0,
            state.rootHeat[i] or 0.0,
            state.contactLoss[i] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "Brake %.2f Smooth %.2f / Speed %.1f / Cooling %.3f\n" ..
        "Temp %.1f %.1f %.1f %.1f / Avg %.1f Max %.1f\n" ..
        "RootHeat %.3f / Links Lock:%s Tire:%s Load:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",

        state.brakeInput or 0.0,
        state.brakeInputSmoothed or 0.0,
        state.speedKmh or 0.0,
        state.cooling or 0.0,

        state.discTemp[0] or M.params.ambient,
        state.discTemp[1] or M.params.ambient,
        state.discTemp[2] or M.params.ambient,
        state.discTemp[3] or M.params.ambient,
        state.avgTemp or M.params.ambient,
        state.maxTempNow or M.params.ambient,

        state.rootHeatAvg or 0.0,

        state.lockLinked and "OK" or "NIL",
        state.tireLinked and "OK" or "NIL",
        state.loadLinked and "OK" or "NIL"
    )
end

return M
