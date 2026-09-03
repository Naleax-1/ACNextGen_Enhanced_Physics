---@diagnostic disable: undefined-global

--============================================================
-- brake_lock.lua
-- ACNextGen V1.1.5 Stable
-- Wheel lock transition model
--============================================================

local M = {}

M.params = {
    lockSlip = 0.18,
    recoverSlip = 0.08,
    transitionTau = 0.08,
    minSpeed = 10.0,

    brakeInputMin = 0.05,
    lowMuLockGain = 0.12,
    contactLossLockGain = 0.10,
    lowLoadLockGain = 0.12,
    tireHopLockGain = 0.08,
    wheelLoadRef = 3500.0,
    fadeRecoverLoss = 0.08,

    weightBiasLockGain = 0.08,
    chassisYawLockGain = 0.04,
    virtualYawLockGain = 0.04,
    suspensionStressLockGain = 0.06,
    fadeRootLockGain = 0.08,
    brakeThermalLockGain = 0.05,
    frontRearBiasLockGain = 0.04,

    maxRootLockAdd = 0.20,
    globalReadInterval = 0.05,
    debugStoreInterval = 0.25,
}

local state = {
    lock = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    slip = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    target = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    load = { [0] = 3500.0, [1] = 3500.0, [2] = 3500.0, [3] = 3500.0 },
    loadNorm = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    mu = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    contactLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireHop = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    weightBias = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    fadeRoot = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    rootLockAdd = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    brakeInput = 0.0,
    speedKmh = 0.0,

    chassisYaw = 0.0,
    virtualYaw = 0.0,
    suspensionStress = 0.0,
    thermalStress = 0.0,
    frontBias = 0.5,
    rearBias = 0.5,

    avgLock = 0.0,
    maxLock = 0.0,
    avgSlip = 0.0,
    avgRootAdd = 0.0,

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    tireStateLinked = false,
    loadLinked = false,
    fadeLinked = false,
    tireLinked = false,
    weightLinked = false,
    chassisLinked = false,
    virtualLinked = false,
    suspensionLinked = false,
    thermalLinked = false,
    fadeRootLinked = false,
    brakeInputLinked = false,

    globalReadTimer = 999.0,
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

local function abs(v)
    v = safeNumber(v, 0.0)
    if v < 0.0 then return -v end
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
        return
    end

    pcall(function()
        ac.store(key, value)
    end)
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

local function safeGetCar()
    if not ac or not ac.getCar then
        return nil
    end

    local ok, car = pcall(function()
        return ac.getCar(0)
    end)

    if ok then
        return car
    end

    return nil
end

local function safeWheel(car, index)
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

local function lowPass(current, target, alpha)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    alpha = clamp(alpha, 0.0, 1.0)
    return current + (target - current) * alpha
end

local function updateDebugGate(dt)
    state.debugStoreTimer = (state.debugStoreTimer or 0.0) + (dt or 0.0)

    if state.debugStoreTimer >= M.params.debugStoreInterval then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    else
        state.debugStoreNow = false
    end
end

local function resetLinks()
    state.tireStateLinked = false
    state.loadLinked = false
    state.fadeLinked = false
    state.tireLinked = false
    state.weightLinked = false
    state.chassisLinked = false
    state.virtualLinked = false
    state.suspensionLinked = false
    state.thermalLinked = false
    state.fadeRootLinked = false
    state.brakeInputLinked = false
end

local function readBrakeInput(car)
    local input =
        safeLoadRaw("ngp_brake_input_smoothed")
        or
        safeLoadRaw("ngp_brake_input")

    if input ~= nil then
        state.brakeInputLinked = true
        return clamp(safeNumber(input, 0.0), 0.0, 1.0)
    end

    return clamp(safeNumber(safeField(car, "brake", 0.0), 0.0), 0.0, 1.0)
end

local function readSpeed(car)
    local speed =
        safeLoadRaw("ngp_speed_kmh")
        or
        safeLoadRaw("ngp_vehicle_speed_kmh")

    if speed ~= nil then
        return safeNumber(speed, 0.0)
    end

    return safeNumber(safeField(car, "speedKmh", 0.0), 0.0)
end

local function getWheelSlipRatio(wheel, index)
    local slip =
        safeLoadRaw("ngp_tire_slip_ratio_" .. index)
        or
        safeLoadRaw("ngp_slip_ratio_" .. index)
        or
        safeLoadRaw("ngp_filtered_slip_ratio_" .. index)
        or
        safeLoadRaw("ngp_tire_state_slip_ratio_" .. index)

    if slip ~= nil then
        state.tireStateLinked = true
    elseif wheel then
        slip = safeField(wheel, "slipRatio", 0.0)
    end

    return abs(safeNumber(slip, 0.0))
end

local function getWheelLoad(wheel, index)
    local load =
        safeLoadRaw("ngp_wheel_load_" .. index)
        or
        safeLoadRaw("ngp_load_wheel_" .. index)
        or
        safeLoadRaw("ngp_dlt_load_" .. index)
        or
        safeLoadRaw("ngp_sprung_load_" .. index)

    if load ~= nil then
        state.loadLinked = true
        return safeNumber(load, M.params.wheelLoadRef)
    end

    if wheel then
        return safeNumber(safeField(wheel, "load", M.params.wheelLoadRef), M.params.wheelLoadRef)
    end

    return M.params.wheelLoadRef
end

local function readWheelRootInputs(index)
    local mu =
        safeLoadRaw("ngp_brake_mu_" .. index)
        or
        safeLoadRaw("ngp_brake_target_mu_" .. index)

    if mu ~= nil then
        state.fadeLinked = true
    end

    state.mu[index] = clamp(safeNumber(mu, 1.0), 0.3, 1.2)

    local contactLoss =
        safeLoadRaw("ngp_tire_contact_loss_" .. index)
        or
        safeLoadRaw("ngp_tcr_contact_loss_" .. index)
        or
        safeLoadRaw("ngp_contact_loss_" .. index)

    local contactQuality =
        safeLoadRaw("ngp_contact_quality_" .. index)
        or
        safeLoadRaw("ngp_tc_contact_" .. index)
        or
        safeLoadRaw("ngp_tire_contact_" .. index)

    if contactLoss == nil and contactQuality ~= nil then
        contactLoss = 1.0 - safeNumber(contactQuality, 1.0)
    end

    local hop =
        safeLoadRaw("ngp_tire_hop_energy_" .. index)
        or
        safeLoadRaw("ngp_tire_hop_" .. index)

    if contactLoss ~= nil or contactQuality ~= nil or hop ~= nil then
        state.tireLinked = true
    end

    state.contactLoss[index] = clamp(safeNumber(contactLoss, 0.0), 0.0, 1.0)
    state.tireHop[index] = clamp(safeNumber(hop, 0.0), 0.0, 1.0)

    local fadeRoot =
        safeLoadRaw("ngp_brake_fade_root_add_" .. index)
        or
        safeLoadRaw("ngp_brake_fade_" .. index)

    if fadeRoot ~= nil then
        state.fadeRootLinked = true
    end

    state.fadeRoot[index] = clamp(safeNumber(fadeRoot, 0.0), 0.0, 1.0)
end

local function readGlobalRootInputs(dt)
    state.globalReadTimer = (state.globalReadTimer or 0.0) + (dt or 0.0)

    if state.globalReadTimer < M.params.globalReadInterval then
        return
    end

    state.globalReadTimer = 0.0

    local front =
        safeLoadRaw("ngp_weight_front_bias")
        or
        safeLoadRaw("ngp_front_bias")
        or
        safeLoadRaw("ngp_load_front_ratio")

    local rear =
        safeLoadRaw("ngp_weight_rear_bias")
        or
        safeLoadRaw("ngp_rear_bias")
        or
        safeLoadRaw("ngp_load_rear_ratio")

    local left =
        safeLoadRaw("ngp_weight_left_bias")
        or
        safeLoadRaw("ngp_left_bias")
        or
        safeLoadRaw("ngp_load_left_ratio")

    local right =
        safeLoadRaw("ngp_weight_right_bias")
        or
        safeLoadRaw("ngp_right_bias")
        or
        safeLoadRaw("ngp_load_right_ratio")

    state.weightLinked =
        front ~= nil or rear ~= nil or left ~= nil or right ~= nil

    local f = clamp(safeNumber(front, state.frontBias or 0.5), 0.25, 0.75)
    local r = clamp(safeNumber(rear, state.rearBias or 0.5), 0.25, 0.75)
    local l = clamp(safeNumber(left, 0.5), 0.25, 0.75)
    local rr = clamp(safeNumber(right, 0.5), 0.25, 0.75)

    state.frontBias = f
    state.rearBias = r

    state.weightBias[0] = clamp(f * l * 2.0, 0.40, 1.60)
    state.weightBias[1] = clamp(f * rr * 2.0, 0.40, 1.60)
    state.weightBias[2] = clamp(r * l * 2.0, 0.40, 1.60)
    state.weightBias[3] = clamp(r * rr * 2.0, 0.40, 1.60)

    local chassisYaw =
        safeLoadRaw("ngp_chassis_yaw_hint")
        or
        safeLoadRaw("ngp_chassis_roll_root_yaw_add")
        or
        safeLoadRaw("ngp_body_yaw_hint")

    state.chassisLinked = chassisYaw ~= nil
    state.chassisYaw = clamp(safeNumber(chassisYaw, 0.0), -1.0, 1.0)

    local virtualYaw =
        safeLoadRaw("ngp_virtual_yaw")
        or
        safeLoadRaw("ngp_virtual_yaw_moment")

    state.virtualLinked = virtualYaw ~= nil
    state.virtualYaw = clamp(safeNumber(virtualYaw, 0.0), -2.5, 2.5)

    local suspension =
        safeLoadRaw("ngp_weight_suspension_stress")
        or
        safeLoadRaw("ngp_chassis_flex_suspension_stress")
        or
        safeLoadRaw("ngp_chassis_roll_suspension_stress")
        or
        safeLoadRaw("ngp_body_suspension_stress")

    state.suspensionLinked = suspension ~= nil
    state.suspensionStress = clamp(safeNumber(suspension, 0.0), 0.0, 1.0)

    local thermal =
        safeLoadRaw("ngp_brake_root_heat_avg")
        or
        safeLoadRaw("ngp_brake_fade_root_add_avg")
        or
        safeLoadRaw("ngp_virtual_thermal_stress")

    state.thermalLinked = thermal ~= nil
    state.thermalStress = clamp(safeNumber(thermal, 0.0), 0.0, 1.0)
end

local function updateRootLockAdd(index)
    local p = M.params

    local side =
        (index == 0 or index == 2)
        and
        -1.0
        or
        1.0

    local isFront = index < 2
    local axleBias =
        isFront
        and
        state.frontBias
        or
        state.rearBias

    local weightAdd =
        math.max(1.0 - (state.weightBias[index] or 1.0), 0.0)
        *
        p.weightBiasLockGain

    local yawAdd =
        math.max(state.chassisYaw * side, 0.0)
        *
        p.chassisYawLockGain
        +
        math.max((state.virtualYaw * 0.4) * side, 0.0)
        *
        p.virtualYawLockGain

    local axleAdd =
        math.max(0.5 - safeNumber(axleBias, 0.5), 0.0)
        *
        p.frontRearBiasLockGain

    state.rootLockAdd[index] =
        clamp(
            weightAdd
            +
            yawAdd
            +
            axleAdd
            +
            (state.suspensionStress or 0.0) * p.suspensionStressLockGain
            +
            (state.fadeRoot[index] or 0.0) * p.fadeRootLockGain
            +
            (state.thermalStress or 0.0) * p.brakeThermalLockGain,
            0.0,
            p.maxRootLockAdd
        )
end

local function calculateTargetLock(index, slip)
    local p = M.params

    local loadNorm =
        clamp(
            (state.load[index] or p.wheelLoadRef) / math.max(p.wheelLoadRef, 1.0),
            0.0,
            2.5
        )

    state.loadNorm[index] = loadNorm

    local effectiveLockSlip =
        p.lockSlip
        -
        (1.0 - state.mu[index]) * p.lowMuLockGain
        -
        state.contactLoss[index] * p.contactLossLockGain
        -
        math.max(1.0 - loadNorm, 0.0) * p.lowLoadLockGain
        -
        state.tireHop[index] * p.tireHopLockGain
        -
        state.rootLockAdd[index]

    effectiveLockSlip = clamp(effectiveLockSlip, 0.05, p.lockSlip)

    local effectiveRecoverSlip =
        p.recoverSlip
        +
        (1.0 - state.mu[index]) * p.fadeRecoverLoss

    effectiveRecoverSlip = clamp(effectiveRecoverSlip, p.recoverSlip, 0.16)

    local target = state.lock[index] or 0.0

    if state.brakeInput < p.brakeInputMin then
        target = 0.0
    elseif slip > effectiveLockSlip then
        target = 1.0
    elseif slip < effectiveRecoverSlip then
        target = 0.0
    end

    return target
end

local function exportWheel(index)
    safeStore("ngp_brake_lock_" .. index, state.lock[index])
    safeStore("ngp_brake_lock_target_" .. index, state.target[index])
    safeStore("ngp_brake_lock_slip_" .. index, state.slip[index])
    safeStore("ngp_brake_lock_load_" .. index, state.load[index])
    safeStore("ngp_brake_lock_load_norm_" .. index, state.loadNorm[index])
    safeStore("ngp_brake_lock_root_add_" .. index, state.rootLockAdd[index])
    safeStore("ngp_brake_lock_weight_bias_" .. index, state.weightBias[index])
    safeStore("ngp_brake_lock_fade_root_" .. index, state.fadeRoot[index])
end

local function exportGlobal()
    safeStore("ngp_brake_lock_status", state.status)
    safeStore("ngp_brake_lock_update_count", state.updateCount)
    safeStore("ngp_brake_lock_wheels_valid", state.wheelsValid and 1 or 0)
    safeStore("ngp_brake_lock_brake_input", state.brakeInput)
    safeStore("ngp_brake_lock_speed_kmh", state.speedKmh)

    safeStore("ngp_brake_lock_avg", state.avgLock)
    safeStore("ngp_brake_lock_max", state.maxLock)
    safeStore("ngp_brake_lock_avg_slip", state.avgSlip)
    safeStore("ngp_brake_lock_root_add_avg", state.avgRootAdd)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_brake_lock_tire_state_linked", state.tireStateLinked and 1 or 0)
    safeStore("ngp_brake_lock_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_brake_lock_fade_linked", state.fadeLinked and 1 or 0)
    safeStore("ngp_brake_lock_tire_linked", state.tireLinked and 1 or 0)
    safeStore("ngp_brake_lock_weight_linked", state.weightLinked and 1 or 0)
    safeStore("ngp_brake_lock_chassis_linked", state.chassisLinked and 1 or 0)
    safeStore("ngp_brake_lock_virtual_linked", state.virtualLinked and 1 or 0)
    safeStore("ngp_brake_lock_suspension_linked", state.suspensionLinked and 1 or 0)
    safeStore("ngp_brake_lock_thermal_linked", state.thermalLinked and 1 or 0)
    safeStore("ngp_brake_lock_fade_root_linked", state.fadeRootLinked and 1 or 0)
    safeStore("ngp_brake_lock_input_linked", state.brakeInputLinked and 1 or 0)

    safeStore("ngp_brake_lock_chassis_yaw", state.chassisYaw)
    safeStore("ngp_brake_lock_virtual_yaw", state.virtualYaw)
    safeStore("ngp_brake_lock_suspension_stress", state.suspensionStress)
    safeStore("ngp_brake_lock_thermal_stress", state.thermalStress)
end

local function exportAllWheels()
    for i = 0, 3 do
        exportWheel(i)
    end
end

function M.init()
    state.status = "INIT"
    exportAllWheels()
    exportGlobal()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = safeNumber(dt, 0.0)

    if dt <= 0.0 then
        state.status = "BAD DT"
        exportGlobal()
        return
    end

    updateDebugGate(dt)
    resetLinks()

    car = car or safeGetCar()

    if not car or not car.wheels then
        state.status = "NO CAR"
        state.wheelsValid = false
        exportGlobal()
        return
    end

    state.status = "RUNNING"
    state.wheelsValid = true
    state.speedKmh = readSpeed(car)
    state.brakeInput = readBrakeInput(car)

    readGlobalRootInputs(dt)

    local alpha =
        dt
        /
        (
            M.params.transitionTau
            +
            dt
        )

    alpha = clamp(alpha, 0.0, 1.0)

    local sumLock = 0.0
    local sumSlip = 0.0
    local sumRoot = 0.0
    local maxLock = 0.0

    for i = 0, 3 do
        local wheel = safeWheel(car, i)

        if state.speedKmh < M.params.minSpeed or not wheel then
            state.lock[i] = lowPass(state.lock[i], 0.0, alpha)
            state.slip[i] = 0.0
            state.target[i] = 0.0

            if not wheel then
                state.load[i] = 0.0
                state.loadNorm[i] = 0.0
            end
        else
            local slip = getWheelSlipRatio(wheel, i)
            local load = getWheelLoad(wheel, i)

            state.slip[i] = slip
            state.load[i] = load

            readWheelRootInputs(i)
            updateRootLockAdd(i)

            local target = calculateTargetLock(i, slip)
            state.target[i] = target
            state.lock[i] = lowPass(state.lock[i], target, alpha)
        end

        state.lock[i] = clamp(state.lock[i], 0.0, 1.0)

        sumLock = sumLock + state.lock[i]
        sumSlip = sumSlip + state.slip[i]
        sumRoot = sumRoot + state.rootLockAdd[i]
        maxLock = math.max(maxLock, state.lock[i])

        exportWheel(i)
    end

    state.avgLock = sumLock * 0.25
    state.avgSlip = sumSlip * 0.25
    state.avgRootAdd = sumRoot * 0.25
    state.maxLock = maxLock

    exportGlobal()
end

function M.getLock(i)
    return state.lock[i] or 0.0
end

function M.getAvgLock()
    return state.avgLock or 0.0
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0
        return string.format(
            "%d Lock %.2f Target %.2f Slip %.3f Load %.0f Root %.3f Mu %.2f",
            i,
            state.lock[i] or 0.0,
            state.target[i] or 0.0,
            state.slip[i] or 0.0,
            state.load[i] or 0.0,
            state.rootLockAdd[i] or 0.0,
            state.mu[i] or 1.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "Lock %.2f %.2f %.2f %.2f / Avg %.2f Max %.2f\n" ..
        "Slip %.3f %.3f %.3f %.3f / Brake %.2f Speed %.1f\n" ..
        "RootAdd %.3f %.3f %.3f %.3f / Avg %.3f\n" ..
        "Links TS:%s Load:%s Fade:%s Tire:%s Weight:%s Chassis:%s Virtual:%s Susp:%s Thermal:%s FadeRoot:%s Input:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",

        state.lock[0] or 0.0,
        state.lock[1] or 0.0,
        state.lock[2] or 0.0,
        state.lock[3] or 0.0,
        state.avgLock or 0.0,
        state.maxLock or 0.0,

        state.slip[0] or 0.0,
        state.slip[1] or 0.0,
        state.slip[2] or 0.0,
        state.slip[3] or 0.0,
        state.brakeInput or 0.0,
        state.speedKmh or 0.0,

        state.rootLockAdd[0] or 0.0,
        state.rootLockAdd[1] or 0.0,
        state.rootLockAdd[2] or 0.0,
        state.rootLockAdd[3] or 0.0,
        state.avgRootAdd or 0.0,

        state.tireStateLinked and "OK" or "NIL",
        state.loadLinked and "OK" or "NIL",
        state.fadeLinked and "OK" or "NIL",
        state.tireLinked and "OK" or "NIL",
        state.weightLinked and "OK" or "NIL",
        state.chassisLinked and "OK" or "NIL",
        state.virtualLinked and "OK" or "NIL",
        state.suspensionLinked and "OK" or "NIL",
        state.thermalLinked and "OK" or "NIL",
        state.fadeRootLinked and "OK" or "NIL",
        state.brakeInputLinked and "OK" or "NIL"
    )
end

return M
