---@diagnostic disable: undefined-global

--============================================================
-- brake_fade.lua
-- ACNextGen V1.1.5
-- Brake Pad Friction Fade Model / Stable Runtime
--============================================================

local M = {}

M.params = {
    normalMu = 1.00,
    fadeStart = 350.0,
    fadeEnd = 750.0,
    minMu = 0.55,

    recoveryTau = 3.00,
    minTau = 0.05,

    contactLossFadeGain = 0.08,
    lockFadeGain = 0.06,
    vehicleConditionFadeGain = 0.08,

    rootHeatFadeGain = 0.08,
    chassisEnergyFadeGain = 0.05,
    suspensionStressFadeGain = 0.05,
    thermalStressFadeGain = 0.06,
    brakeInputFadeGain = 0.04,

    coolingRecoveryGain = 0.06,
    maxRootFadeAdd = 0.20,

    globalReadInterval = 0.05,
    debugStoreInterval = 0.25,
}

local state = {
    mu = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    targetMu = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    temp = { [0] = 25.0, [1] = 25.0, [2] = 25.0, [3] = 25.0 },
    fade = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    contactLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    lock = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    rootHeat = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    rootFadeAdd = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    avgMu = 1.0,
    minMuNow = 1.0,
    avgFade = 0.0,
    avgRootAdd = 0.0,
    avgTemp = 25.0,

    vehicleCondition = 0.0,
    chassisEnergy = 0.0,
    suspensionStress = 0.0,
    thermalStress = 0.0,
    brakeInput = 0.0,
    cooling = 0.0,

    brakeSystemLinked = false,
    lockLinked = false,
    tireLinked = false,
    conditionLinked = false,
    rootHeatLinked = false,
    chassisLinked = false,
    suspensionLinked = false,
    thermalLinked = false,
    brakeInputLinked = false,
    coolingLinked = false,

    status = "INIT",
    updateCount = 0,

    globalReadTimer = 999.0,
    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

local function safeNumber(value, fallback)
    local n = tonumber(value)
    if n == nil or n ~= n then
        return fallback or 0.0
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
    if not ac or not ac.load or key == nil then
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

local function safeLoad(key, fallback)
    local value = safeLoadRaw(key)
    if value == nil then
        return fallback or 0.0
    end
    return safeNumber(value, fallback or 0.0)
end

local function safeStore(key, value)
    if not ac or not ac.store or key == nil then
        return false
    end

    local ok = pcall(function()
        ac.store(key, value)
    end)

    return ok
end

local function lowPass(current, target, dt, tau)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    dt = safeNumber(dt, 0.0)
    tau = math.max(safeNumber(tau, M.params.minTau), M.params.minTau)

    local alpha = dt / (tau + dt)
    alpha = clamp(alpha, 0.0, 1.0)

    return current + (target - current) * alpha
end

local function updateTimers(dt)
    dt = safeNumber(dt, 0.0)

    state.globalReadTimer = (state.globalReadTimer or 999.0) + dt
    state.debugStoreTimer = (state.debugStoreTimer or 999.0) + dt

    state.debugStoreNow = false
    if state.debugStoreTimer >= M.params.debugStoreInterval then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    end
end

local function resetLinks()
    state.brakeSystemLinked = false
    state.lockLinked = false
    state.tireLinked = false
    state.conditionLinked = false
    state.rootHeatLinked = false
    state.chassisLinked = false
    state.suspensionLinked = false
    state.thermalLinked = false
    state.brakeInputLinked = false
    state.coolingLinked = false
end

local function readBrakeTemperature(index)
    local value =
        safeLoadRaw("ngp_brake_disc_temp_" .. index)
        or safeLoadRaw("ngp_brake_temp_" .. index)

    if value ~= nil then
        state.brakeSystemLinked = true
    end

    return safeNumber(value, 25.0)
end

local function readWheelInputs(index)
    local contactLoss =
        safeLoadRaw("ngp_tire_contact_loss_" .. index)
        or safeLoadRaw("ngp_tcr_contact_loss_" .. index)

    if contactLoss ~= nil then
        state.tireLinked = true
    end

    state.contactLoss[index] =
        clamp(safeNumber(contactLoss, 0.0), 0.0, 1.0)

    local lock = safeLoadRaw("ngp_brake_lock_" .. index)

    if lock ~= nil then
        state.lockLinked = true
    end

    state.lock[index] =
        clamp(safeNumber(lock, 0.0), 0.0, 1.0)

    local rootHeat = safeLoadRaw("ngp_brake_root_heat_" .. index)

    if rootHeat ~= nil then
        state.rootHeatLinked = true
    end

    state.rootHeat[index] =
        clamp(safeNumber(rootHeat, 0.0), 0.0, 1.0)
end

local function readGlobalInputs()
    local condition =
        safeLoadRaw("ngp_condition_brake")
        or safeLoadRaw("ngp_vehicle_brake_condition")

    state.conditionLinked = condition ~= nil
    state.vehicleCondition =
        clamp(safeNumber(condition, 0.0), 0.0, 1.0)

    local chassisEnergy =
        safeLoadRaw("ngp_chassis_energy")
        or safeLoadRaw("ngp_chassis_body_energy")

    state.chassisLinked = chassisEnergy ~= nil
    state.chassisEnergy =
        clamp(safeNumber(chassisEnergy, 0.0), 0.0, 1.0)

    local suspension =
        safeLoadRaw("ngp_weight_suspension_stress")
        or safeLoadRaw("ngp_chassis_flex_suspension_stress")
        or safeLoadRaw("ngp_body_suspension_stress")

    state.suspensionLinked = suspension ~= nil
    state.suspensionStress =
        clamp(safeNumber(suspension, 0.0), 0.0, 1.0)

    local thermal =
        safeLoadRaw("ngp_brake_root_heat_avg")
        or safeLoadRaw("ngp_virtual_thermal_stress")
        or safeLoadRaw("ngp_chassis_roll_thermal_stress")

    state.thermalLinked = thermal ~= nil
    state.thermalStress =
        clamp(safeNumber(thermal, 0.0), 0.0, 1.0)

    local brakeInput =
        safeLoadRaw("ngp_brake_input_smoothed")
        or safeLoadRaw("ngp_brake_input")

    state.brakeInputLinked = brakeInput ~= nil
    state.brakeInput =
        clamp(safeNumber(brakeInput, 0.0), 0.0, 1.0)

    local cooling = safeLoadRaw("ngp_brake_cooling")

    state.coolingLinked = cooling ~= nil
    state.cooling =
        clamp(safeNumber(cooling, 0.0), 0.0, 2.0)
end

local function fadeFromTemperature(temp)
    local p = M.params

    if temp <= p.fadeStart then
        return 0.0
    end

    local range = math.max(p.fadeEnd - p.fadeStart, 1.0)
    return clamp((temp - p.fadeStart) / range, 0.0, 1.0)
end

local function calculateFade(index)
    local p = M.params
    local tempFade = fadeFromTemperature(state.temp[index] or 25.0)

    local rootAdd =
        state.rootHeat[index] * p.rootHeatFadeGain
        + state.chassisEnergy * p.chassisEnergyFadeGain
        + state.suspensionStress * p.suspensionStressFadeGain
        + state.thermalStress * p.thermalStressFadeGain
        + state.brakeInput * p.brakeInputFadeGain

    rootAdd = clamp(rootAdd, 0.0, p.maxRootFadeAdd)
    state.rootFadeAdd[index] = rootAdd

    local fade =
        tempFade
        + state.contactLoss[index] * p.contactLossFadeGain
        + state.lock[index] * p.lockFadeGain
        + state.vehicleCondition * p.vehicleConditionFadeGain
        + rootAdd

    return clamp(fade, 0.0, 1.0)
end

local function calculateTargetMu(index)
    local p = M.params
    local fade = calculateFade(index)

    state.fade[index] = fade

    return clamp(
        p.normalMu - fade * (p.normalMu - p.minMu),
        p.minMu,
        p.normalMu
    )
end

local function exportWheel(index)
    safeStore("ngp_brake_mu_" .. index, state.mu[index] or 1.0)
    safeStore("ngp_brake_fade_" .. index, state.fade[index] or 0.0)
    safeStore("ngp_brake_target_mu_" .. index, state.targetMu[index] or 1.0)
    safeStore("ngp_brake_fade_temp_" .. index, state.temp[index] or 25.0)
    safeStore("ngp_brake_fade_root_add_" .. index, state.rootFadeAdd[index] or 0.0)
    safeStore("ngp_brake_fade_root_heat_" .. index, state.rootHeat[index] or 0.0)
end

local function exportGlobal()
    safeStore("ngp_brake_fade_status", state.status or "UNKNOWN")
    safeStore("ngp_brake_fade_update_count", state.updateCount or 0)

    safeStore("ngp_brake_mu_avg", state.avgMu or 1.0)
    safeStore("ngp_brake_mu_min", state.minMuNow or 1.0)
    safeStore("ngp_brake_fade_avg", state.avgFade or 0.0)
    safeStore("ngp_brake_fade_root_add_avg", state.avgRootAdd or 0.0)
    safeStore("ngp_brake_fade_temp_avg", state.avgTemp or 25.0)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_brake_fade_brake_system_linked", state.brakeSystemLinked and 1 or 0)
    safeStore("ngp_brake_fade_lock_linked", state.lockLinked and 1 or 0)
    safeStore("ngp_brake_fade_tire_linked", state.tireLinked and 1 or 0)
    safeStore("ngp_brake_fade_condition_linked", state.conditionLinked and 1 or 0)
    safeStore("ngp_brake_fade_root_heat_linked", state.rootHeatLinked and 1 or 0)
    safeStore("ngp_brake_fade_chassis_linked", state.chassisLinked and 1 or 0)
    safeStore("ngp_brake_fade_suspension_linked", state.suspensionLinked and 1 or 0)
    safeStore("ngp_brake_fade_thermal_linked", state.thermalLinked and 1 or 0)
    safeStore("ngp_brake_fade_input_linked", state.brakeInputLinked and 1 or 0)
    safeStore("ngp_brake_fade_cooling_linked", state.coolingLinked and 1 or 0)

    safeStore("ngp_brake_fade_vehicle_condition", state.vehicleCondition or 0.0)
    safeStore("ngp_brake_fade_chassis_energy", state.chassisEnergy or 0.0)
    safeStore("ngp_brake_fade_suspension_stress", state.suspensionStress or 0.0)
    safeStore("ngp_brake_fade_thermal_stress", state.thermalStress or 0.0)
    safeStore("ngp_brake_fade_input", state.brakeInput or 0.0)
    safeStore("ngp_brake_fade_cooling", state.cooling or 0.0)
end

local function updateSummary()
    local sumMu = 0.0
    local sumFade = 0.0
    local sumRootAdd = 0.0
    local sumTemp = 0.0
    local minMuNow = 1.0

    for i = 0, 3 do
        sumMu = sumMu + (state.mu[i] or 1.0)
        sumFade = sumFade + (state.fade[i] or 0.0)
        sumRootAdd = sumRootAdd + (state.rootFadeAdd[i] or 0.0)
        sumTemp = sumTemp + (state.temp[i] or 25.0)
        minMuNow = math.min(minMuNow, state.mu[i] or 1.0)
    end

    state.avgMu = sumMu * 0.25
    state.avgFade = sumFade * 0.25
    state.avgRootAdd = sumRootAdd * 0.25
    state.avgTemp = sumTemp * 0.25
    state.minMuNow = minMuNow
end

function M.init()
    state.status = "INIT"
    state.updateCount = 0
    state.globalReadTimer = 999.0
    state.debugStoreTimer = 999.0
    state.debugStoreNow = true

    updateSummary()

    for i = 0, 3 do
        exportWheel(i)
    end

    exportGlobal()
end

function M.update(dt, car, runtime)
    dt = safeNumber(dt, 0.0)

    state.updateCount = (state.updateCount or 0) + 1

    if dt <= 0.0 then
        state.status = "BAD DT"
        exportGlobal()
        return
    end

    updateTimers(dt)
    resetLinks()

    if state.globalReadTimer >= M.params.globalReadInterval then
        state.globalReadTimer = 0.0
        readGlobalInputs()
    end

    state.status = "RUNNING"

    local tau =
        M.params.recoveryTau
        / (1.0 + (state.cooling or 0.0) * M.params.coolingRecoveryGain)

    tau = math.max(tau, M.params.minTau)

    for i = 0, 3 do
        readWheelInputs(i)

        state.temp[i] = readBrakeTemperature(i)
        state.targetMu[i] = calculateTargetMu(i)

        state.mu[i] =
            lowPass(
                state.mu[i],
                state.targetMu[i],
                dt,
                tau
            )

        state.mu[i] =
            clamp(
                state.mu[i],
                M.params.minMu,
                M.params.normalMu
            )

        exportWheel(i)
    end

    updateSummary()
    exportGlobal()
end

function M.getMu(i)
    return state.mu[i] or 1.0
end

function M.getFade(i)
    return state.fade[i] or 0.0
end

function M.getTargetMu(i)
    return state.targetMu[i] or 1.0
end

function M.debugStr(index)
    if index ~= nil then
        index = tonumber(index) or 0
        index = math.max(0, math.min(3, index))

        return string.format(
            "Status %s / W%d Mu %.2f Target %.2f Fade %.2f Temp %.0f / Lock %.2f Root %.2f",
            tostring(state.status),
            index,
            state.mu[index] or 1.0,
            state.targetMu[index] or 1.0,
            state.fade[index] or 0.0,
            state.temp[index] or 25.0,
            state.lock[index] or 0.0,
            state.rootFadeAdd[index] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f\n" ..
        "Mu %.2f %.2f %.2f %.2f / Avg %.2f Min %.2f Fade %.2f Temp %.0f\n" ..
        "RootAdd %.3f / Links Brake:%s Lock:%s Tire:%s Cond:%s Root:%s Chassis:%s Susp:%s Thermal:%s Input:%s Cool:%s",
        tostring(state.status),
        state.updateCount or 0,

        state.mu[0] or 1.0,
        state.mu[1] or 1.0,
        state.mu[2] or 1.0,
        state.mu[3] or 1.0,
        state.avgMu or 1.0,
        state.minMuNow or 1.0,
        state.avgFade or 0.0,
        state.avgTemp or 25.0,

        state.avgRootAdd or 0.0,

        state.brakeSystemLinked and "OK" or "NIL",
        state.lockLinked and "OK" or "NIL",
        state.tireLinked and "OK" or "NIL",
        state.conditionLinked and "OK" or "NIL",
        state.rootHeatLinked and "OK" or "NIL",
        state.chassisLinked and "OK" or "NIL",
        state.suspensionLinked and "OK" or "NIL",
        state.thermalLinked and "OK" or "NIL",
        state.brakeInputLinked and "OK" or "NIL",
        state.coolingLinked and "OK" or "NIL"
    )
end

return M
