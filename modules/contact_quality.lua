---@diagnostic disable: undefined-global

--============================================================
-- contact_quality.lua
-- ACNextGen V1.1.5 Stable
-- Contact Quality / Trust Bridge
--============================================================

local M = {}

local WHEEL_NAME = {
    [0] = "FL",
    [1] = "FR",
    [2] = "RL",
    [3] = "RR",
}

M.params = {
    loadMin = 120.0,
    loadGood = 1400.0,
    loadRef = 3000.0,

    overloadStart = 6500.0,
    overloadFull = 9500.0,
    overloadPenalty = 0.18,

    slipRatioGood = 0.045,
    slipRatioBad = 0.35,

    slipAngleGood = 0.035,
    slipAngleBad = 0.22,

    combinedSlipGood = 0.45,
    combinedSlipBad = 1.35,

    loadDropWarn = 35000.0,
    loadDropBad = 120000.0,

    lowSpeedKmh = 3.0,
    lowSpeedSlipWeight = 0.35,

    carcassSupportWeight = 0.30,
    carcassGripWeight = 0.26,
    verticalWeight = 0.16,
    hysteresisWeight = 0.14,
    delayWeight = 0.16,

    verticalGood = 0.08,
    verticalBad = 0.78,

    hysteresisGood = 0.08,
    hysteresisBad = 0.75,

    delayGood = 0.08,
    delayBad = 0.70,

    recoveryAssist = 0.08,
    impactHystThreshold = 0.62,

    tauRise = 0.095,
    tauFall = 0.030,
    tauCarcass = 0.055,

    minQuality = 0.0,
    maxQuality = 1.0,

    noCarReturnTau = 0.180,
    maxDt = 0.050,

    debugStoreInterval = 0.25,
}

local state = {
    initialized = false,
    status = "INIT",
    updateCount = 0,

    wheelsValid = false,
    carcassLinked = false,
    loadLinked = false,
    tireStateLinked = false,
    wheelApiLinked = false,

    speedKmh = 0.0,

    avgQuality = 0.0,
    avgRawQuality = 0.0,
    avgTrust = 0.0,
    avgLoss = 1.0,

    minQualityNow = 0.0,
    maxCombinedSlip = 0.0,

    debugStoreTimer = 999.0,
    debugStoreNow = true,

    quality = {},
    rawQuality = {},
    trust = {},
    loss = {},

    load = {},
    prevLoad = {},
    loadRate = {},

    slipRatio = {},
    slipAngle = {},
    combinedSlip = {},

    loadScore = {},
    slipScore = {},
    stabilityScore = {},

    carcassSupport = {},
    carcassGripGate = {},
    carcassVertical = {},
    carcassHysteresis = {},
    carcassDelay = {},
    carcassRecovery = {},
    carcassHeatSeed = {},
    carcassHistoryStress = {},
    carcassScore = {},
    verticalScore = {},
    hysteresisScore = {},
    delayScore = {},

    statusId = {},
    statusText = {},
}

for i = 0, 3 do
    state.quality[i] = 0.0
    state.rawQuality[i] = 0.0
    state.trust[i] = 0.0
    state.loss[i] = 1.0

    state.load[i] = 0.0
    state.prevLoad[i] = 0.0
    state.loadRate[i] = 0.0

    state.slipRatio[i] = 0.0
    state.slipAngle[i] = 0.0
    state.combinedSlip[i] = 0.0

    state.loadScore[i] = 0.0
    state.slipScore[i] = 0.0
    state.stabilityScore[i] = 0.0

    state.carcassSupport[i] = 1.0
    state.carcassGripGate[i] = 1.0
    state.carcassVertical[i] = 0.0
    state.carcassHysteresis[i] = 0.0
    state.carcassDelay[i] = 0.0
    state.carcassRecovery[i] = 0.0
    state.carcassHeatSeed[i] = 0.0
    state.carcassHistoryStress[i] = 0.0
    state.carcassScore[i] = 1.0
    state.verticalScore[i] = 1.0
    state.hysteresisScore[i] = 1.0
    state.delayScore[i] = 1.0

    state.statusId[i] = 0
    state.statusText[i] = "INIT"
end

M.state = state
M.debug = state

local function num(v, fallback)
    local n = tonumber(v)

    if n == nil or n ~= n or n == math.huge or n == -math.huge then
        return fallback or 0.0
    end

    return n
end

local function abs(v)
    return math.abs(num(v, 0.0))
end

local function clamp(v, minV, maxV)
    v = num(v, minV)

    if v < minV then
        return minV
    end

    if v > maxV then
        return maxV
    end

    return v
end

local function lerp(a, b, t)
    return a + (b - a) * clamp(t, 0.0, 1.0)
end

local function smoothstep(edge0, edge1, x)
    if edge0 == edge1 then
        return x >= edge1 and 1.0 or 0.0
    end

    local t = clamp((num(x, 0.0) - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)
end

local function lowPass(current, target, tau, dt)
    current = num(current, 0.0)
    target = num(target, 0.0)
    tau = num(tau, 0.0)
    dt = num(dt, 0.0)

    if tau <= 0.0 then
        return target
    end

    return current + (target - current) * clamp(dt / math.max(tau + dt, 0.0001), 0.0, 1.0)
end

local function safeStore(key, value)
    if not ac or not ac.store then
        return
    end

    pcall(function()
        ac.store(key, value)
    end)
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

local function safeLoad(key, fallback)
    local value = safeLoadRaw(key)

    if value == nil then
        return fallback or 0.0
    end

    return num(value, fallback or 0.0)
end

local function safeField(obj, field, fallback)
    if not obj then
        return fallback
    end

    local ok, value = pcall(function()
        return obj[field]
    end)

    if ok and value ~= nil then
        return value
    end

    return fallback
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

local function getWheels(car)
    if not car then
        return nil
    end

    local ok, wheels = pcall(function()
        return car.wheels
    end)

    if ok then
        return wheels
    end

    return nil
end

local function getWheel(car, index)
    local wheels = getWheels(car)

    if not wheels then
        return nil
    end

    local ok, wheel = pcall(function()
        return wheels[index] or wheels[index + 1]
    end)

    if ok then
        return wheel
    end

    return nil
end

local function vecLength(v)
    if not v then
        return 0.0
    end

    local ok, result = pcall(function()
        if type(v.length) == "function" then
            return v:length()
        end

        if type(v.length) == "number" then
            return v.length
        end

        local x = num(v.x, 0.0)
        local y = num(v.y, 0.0)
        local z = num(v.z, 0.0)

        return math.sqrt(x * x + y * y + z * z)
    end)

    if ok then
        return num(result, 0.0)
    end

    return 0.0
end

local function getSpeedKmh(car)
    if not car then
        return 0.0
    end

    local speedKmh = safeField(car, "speedKmh", nil)

    if speedKmh ~= nil then
        return num(speedKmh, 0.0)
    end

    local speed = nil

    if speed ~= nil then
        return num(speed, 0.0)
    end

    local velocity = safeField(car, "velocity", nil)

    if velocity then
        return vecLength(velocity) * 3.6
    end

    local localVelocity = safeField(car, "localVelocity", nil)

    if localVelocity then
        return vecLength(localVelocity) * 3.6
    end

    return 0.0
end

local function firstNumber(defaultValue, ...)
    local keys = { ... }

    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])

        if value ~= nil then
            return num(value, defaultValue or 0.0), keys[i]
        end
    end

    return defaultValue or 0.0, nil
end

local function getWheelLoad(wheel, index)
    if wheel then
        local load =
            safeField(wheel, "load", nil)
            or safeField(wheel, "loadK", nil)
            or nil
            or nil

        if load ~= nil then
            state.wheelApiLinked = true
            return num(load, 0.0)
        end
    end

    local load, key =
        firstNumber(
            M.params.loadRef,
            "ngp_tire_carcass_load_" .. index,
            "ngp_carcass_load_" .. index,
            "ngp_tire_state_load_" .. index,
            "ngp_tire_load_" .. index,
            "ngp_wheel_load_" .. index,
            "ngp_load_wheel_" .. index,
            "ngp_sprung_load_" .. index
        )

    if key then
        state.loadLinked = true
        return load
    end

    local dltLoad = safeLoadRaw("ngp_dlt_load_" .. index)

    if dltLoad ~= nil then
        state.loadLinked = true
        return M.params.loadRef + num(dltLoad, 0.0)
    end

    local contactLoad = safeLoadRaw("ngp_contact_patch_load_" .. index)

    if contactLoad ~= nil then
        state.loadLinked = true
        return num(contactLoad, M.params.loadRef)
    end

    return 0.0
end

local function getSlipRatio(wheel, index)
    local value, key =
        firstNumber(
            0.0,
            "ngp_tire_carcass_slip_ratio_" .. index,
            "ngp_carcass_slip_ratio_" .. index,
            "ngp_tire_slip_ratio_" .. index,
            "ngp_filtered_slip_ratio_" .. index,
            "ngp_slip_ratio_" .. index
        )

    if key then
        state.tireStateLinked = true
        return value
    end

    if wheel then
        local slip =
            safeField(wheel, "slipRatio", nil)
            or nil
            or safeField(wheel, "slip", nil)

        if slip ~= nil then
            state.wheelApiLinked = true
            return num(slip, 0.0)
        end
    end

    return safeLoad("ngp_contact_slip_ratio_" .. index, 0.0)
end

local function getSlipAngle(wheel, index)
    local value, key =
        firstNumber(
            0.0,
            "ngp_tire_carcass_slip_angle_" .. index,
            "ngp_carcass_slip_angle_" .. index,
            "ngp_tire_slip_angle_" .. index,
            "ngp_filtered_slip_angle_" .. index,
            "ngp_slip_angle_" .. index
        )

    if key then
        state.tireStateLinked = true
        return value
    end

    if wheel then
        local slip =
            safeField(wheel, "slipAngle", nil)
            or nil

        if slip ~= nil then
            state.wheelApiLinked = true
            return num(slip, 0.0)
        end
    end

    return safeLoad("ngp_contact_slip_angle_" .. index, 0.0)
end

local function readCarcass(index)
    local linked = false

    local support =
        safeLoadRaw("ngp_carcass_support_" .. index)
        or safeLoadRaw("ngp_tire_carcass_support_" .. index)

    local gripGate =
        safeLoadRaw("ngp_carcass_grip_gate_" .. index)
        or safeLoadRaw("ngp_tire_carcass_grip_gate_" .. index)

    local vertical =
        safeLoadRaw("ngp_carcass_vertical_norm_" .. index)
        or safeLoadRaw("ngp_tire_carcass_vertical_norm_" .. index)
        or safeLoadRaw("ngp_tire_carcass_crush_" .. index)
        or safeLoadRaw("ngp_carcass_crush_" .. index)

    local hysteresis =
        safeLoadRaw("ngp_carcass_hysteresis_" .. index)
        or safeLoadRaw("ngp_tire_carcass_hysteresis_" .. index)

    local delay =
        safeLoadRaw("ngp_contact_delay_" .. index)
        or safeLoadRaw("ngp_tire_contact_delay_" .. index)
        or safeLoadRaw("ngp_carcass_delay_" .. index)

    local recovery =
        safeLoadRaw("ngp_carcass_recovery_bias_" .. index)
        or safeLoadRaw("ngp_tire_carcass_recovery_bias_" .. index)
        or safeLoadRaw("ngp_tire_return_force_" .. index)

    local heatSeed =
        safeLoadRaw("ngp_carcass_heat_seed_" .. index)
        or safeLoadRaw("ngp_tire_carcass_heat_seed_" .. index)

    local history =
        safeLoadRaw("ngp_carcass_history_stress_" .. index)
        or safeLoadRaw("ngp_tire_carcass_history_stress_" .. index)

    if support ~= nil
        or gripGate ~= nil
        or vertical ~= nil
        or hysteresis ~= nil
        or delay ~= nil
        or recovery ~= nil
        or heatSeed ~= nil
        or history ~= nil then
        linked = true
    end

    return {
        linked = linked,
        support = clamp(num(support, 1.0), 0.0, 1.2),
        gripGate = clamp(num(gripGate, 1.0), 0.0, 1.2),
        vertical = clamp(num(vertical, 0.0), 0.0, 1.5),
        hysteresis = clamp(num(hysteresis, 0.0), 0.0, 1.5),
        delay = clamp(num(delay, 0.0), 0.0, 1.5),
        recovery = clamp(num(recovery, 0.0), 0.0, 1.5),
        heatSeed = clamp(num(heatSeed, 0.0), 0.0, 1.5),
        history = clamp(num(history, 0.0), 0.0, 1.5),
    }
end

local function statusFromScores(loadScore, slipScore, stabilityScore, combinedSlip, rawQuality, carcassScore, delayScore, hysteresis)
    if loadScore < 0.15 then
        return 4, "UNLOADED"
    end

    if hysteresis > M.params.impactHystThreshold then
        return 7, "IMPACT"
    end

    if stabilityScore < 0.35 then
        return 5, "UNSTABLE"
    end

    if combinedSlip > 1.20 then
        return 3, "SATURATED"
    end

    if delayScore < 0.45 then
        return 8, "DELAYED"
    end

    if slipScore < 0.45 then
        return 2, "SLIPPING"
    end

    if carcassScore < 0.55 then
        return 9, "WEAK_SUPPORT"
    end

    if rawQuality < 0.70 then
        return 1, "MARGINAL"
    end

    return 6, "STABLE"
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

local function exportWheel(index)
    safeStore("ngp_contact_quality_" .. index, state.quality[index] or 0.0)
    safeStore("ngp_contact_raw_" .. index, state.rawQuality[index] or 0.0)
    safeStore("ngp_contact_trust_" .. index, state.trust[index] or 0.0)
    safeStore("ngp_contact_loss_" .. index, state.loss[index] or 1.0)

    safeStore("ngp_tire_contact_quality_" .. index, state.quality[index] or 0.0)
    safeStore("ngp_tire_contact_trust_" .. index, state.trust[index] or 0.0)
    safeStore("ngp_tire_contact_loss_" .. index, state.loss[index] or 1.0)
    safeStore("ngp_tcr_quality_" .. index, state.quality[index] or 0.0)
    safeStore("ngp_tcr_contact_loss_" .. index, state.loss[index] or 1.0)

    safeStore("ngp_contact_load_" .. index, state.load[index] or 0.0)
    safeStore("ngp_contact_load_rate_" .. index, state.loadRate[index] or 0.0)
    safeStore("ngp_contact_slip_ratio_" .. index, state.slipRatio[index] or 0.0)
    safeStore("ngp_contact_slip_angle_" .. index, state.slipAngle[index] or 0.0)
    safeStore("ngp_contact_combined_slip_" .. index, state.combinedSlip[index] or 0.0)

    safeStore("ngp_contact_load_score_" .. index, state.loadScore[index] or 0.0)
    safeStore("ngp_contact_slip_score_" .. index, state.slipScore[index] or 0.0)
    safeStore("ngp_contact_stability_score_" .. index, state.stabilityScore[index] or 0.0)

    safeStore("ngp_contact_carcass_support_" .. index, state.carcassSupport[index] or 1.0)
    safeStore("ngp_contact_grip_gate_" .. index, state.carcassGripGate[index] or 1.0)
    safeStore("ngp_contact_vertical_norm_" .. index, state.carcassVertical[index] or 0.0)
    safeStore("ngp_contact_vertical_score_" .. index, state.verticalScore[index] or 1.0)
    safeStore("ngp_contact_hysteresis_" .. index, state.carcassHysteresis[index] or 0.0)
    safeStore("ngp_contact_hysteresis_score_" .. index, state.hysteresisScore[index] or 1.0)
    safeStore("ngp_contact_quality_delay_" .. index, state.carcassDelay[index] or 0.0)
    safeStore("ngp_contact_delay_score_" .. index, state.delayScore[index] or 1.0)
    safeStore("ngp_contact_carcass_score_" .. index, state.carcassScore[index] or 1.0)
    safeStore("ngp_contact_recovery_bias_" .. index, state.carcassRecovery[index] or 0.0)
    safeStore("ngp_contact_heat_seed_" .. index, state.carcassHeatSeed[index] or 0.0)
    safeStore("ngp_contact_history_stress_" .. index, state.carcassHistoryStress[index] or 0.0)

    safeStore("ngp_contact_status_" .. index, state.statusId[index] or 0)
    safeStore("ngp_contact_status_text_" .. index, state.statusText[index] or "UNKNOWN")
end

local function exportGlobal()
    safeStore("ngp_contact_status_global", state.status or "UNKNOWN")
    safeStore("ngp_contact_quality_status", state.status or "UNKNOWN")
    safeStore("ngp_contact_update_count", state.updateCount or 0)
    safeStore("ngp_contact_quality_update_count", state.updateCount or 0)
    safeStore("ngp_contact_wheels_valid", state.wheelsValid and 1 or 0)

    safeStore("ngp_contact_speed_kmh", state.speedKmh or 0.0)
    safeStore("ngp_contact_carcass_linked", state.carcassLinked and 1 or 0)
    safeStore("ngp_contact_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_contact_tire_state_linked", state.tireStateLinked and 1 or 0)
    safeStore("ngp_contact_wheel_api_linked", state.wheelApiLinked and 1 or 0)

    safeStore("ngp_contact_avg_quality", state.avgQuality or 0.0)
    safeStore("ngp_contact_avg_raw", state.avgRawQuality or 0.0)
    safeStore("ngp_contact_avg_trust", state.avgTrust or 0.0)
    safeStore("ngp_contact_avg_loss", state.avgLoss or 1.0)
    safeStore("ngp_contact_min_quality", state.minQualityNow or 0.0)
    safeStore("ngp_contact_max_combined_slip", state.maxCombinedSlip or 0.0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i)
    end

    exportGlobal()
end

local function clearWheel(index, statusText, dt)
    state.quality[index] = lowPass(state.quality[index], 0.0, M.params.noCarReturnTau, dt)
    state.rawQuality[index] = 0.0
    state.trust[index] = 0.0
    state.loss[index] = 1.0

    state.load[index] = lowPass(state.load[index], 0.0, M.params.noCarReturnTau, dt)
    state.loadRate[index] = 0.0
    state.slipRatio[index] = 0.0
    state.slipAngle[index] = 0.0
    state.combinedSlip[index] = 0.0

    state.loadScore[index] = 0.0
    state.slipScore[index] = 1.0
    state.stabilityScore[index] = 0.0

    state.statusId[index] = 0
    state.statusText[index] = statusText or "NO WHEEL"
end

local function calculateWheel(index, car, dt)
    local wheel = getWheel(car, index)
    local p = M.params

    if not wheel then
        clearWheel(index, "NO WHEEL", dt)
        exportWheel(index)
        return 0.0, 0.0, 0.0, 1.0, 0.0
    end

    local load = math.abs(getWheelLoad(wheel, index))
    local slipRatio = abs(getSlipRatio(wheel, index))
    local slipAngle = abs(getSlipAngle(wheel, index))

    local prevLoad = state.prevLoad[index] or load

    if prevLoad <= 0.0 and load > 0.0 then
        prevLoad = load
    end

    local loadRate = (load - prevLoad) / math.max(dt, 0.001)
    local dropRate = math.max(0.0, -loadRate)

    state.load[index] = load
    state.loadRate[index] = loadRate
    state.slipRatio[index] = slipRatio
    state.slipAngle[index] = slipAngle

    local loadScore = smoothstep(p.loadMin, p.loadGood, load)
    local overload = smoothstep(p.overloadStart, p.overloadFull, load)

    loadScore = loadScore * (1.0 - overload * p.overloadPenalty)

    local ratioScore = 1.0 - smoothstep(p.slipRatioGood, p.slipRatioBad, slipRatio)
    local angleScore = 1.0 - smoothstep(p.slipAngleGood, p.slipAngleBad, slipAngle)

    local ratioNorm = slipRatio / math.max(p.slipRatioBad, 0.001)
    local angleNorm = slipAngle / math.max(p.slipAngleBad, 0.001)
    local combinedSlip = math.sqrt(ratioNorm * ratioNorm + angleNorm * angleNorm)

    local combinedScore =
        1.0
        -
        smoothstep(
            p.combinedSlipGood,
            p.combinedSlipBad,
            combinedSlip
        )

    local slipScore = math.min(ratioScore, angleScore, combinedScore)

    if state.speedKmh < p.lowSpeedKmh then
        local lowBlend = clamp(state.speedKmh / math.max(p.lowSpeedKmh, 0.001), 0.0, 1.0)
        local lowSpeedWeight = lerp(p.lowSpeedSlipWeight, 1.0, lowBlend)

        slipScore = lerp(1.0, slipScore, lowSpeedWeight)
        combinedSlip = combinedSlip * lowSpeedWeight
    end

    local stabilityScore = 1.0 - smoothstep(p.loadDropWarn, p.loadDropBad, dropRate)

    local c = readCarcass(index)

    if c.linked then
        state.carcassLinked = true
    end

    state.carcassSupport[index] =
        clamp(
            lowPass(
                state.carcassSupport[index],
                c.support,
                p.tauCarcass,
                dt
            ),
            0.0,
            1.2
        )

    state.carcassGripGate[index] =
        clamp(
            lowPass(
                state.carcassGripGate[index],
                c.gripGate,
                p.tauCarcass,
                dt
            ),
            0.0,
            1.2
        )

    state.carcassVertical[index] =
        clamp(
            lowPass(
                state.carcassVertical[index],
                c.vertical,
                p.tauCarcass,
                dt
            ),
            0.0,
            1.5
        )

    state.carcassHysteresis[index] =
        clamp(
            lowPass(
                state.carcassHysteresis[index],
                c.hysteresis,
                p.tauCarcass,
                dt
            ),
            0.0,
            1.5
        )

    state.carcassDelay[index] =
        clamp(
            lowPass(
                state.carcassDelay[index],
                c.delay,
                p.tauCarcass,
                dt
            ),
            0.0,
            1.5
        )

    state.carcassRecovery[index] =
        clamp(
            lowPass(
                state.carcassRecovery[index],
                c.recovery,
                p.tauCarcass,
                dt
            ),
            0.0,
            1.5
        )

    state.carcassHeatSeed[index] =
        clamp(
            lowPass(
                state.carcassHeatSeed[index],
                c.heatSeed,
                p.tauCarcass,
                dt
            ),
            0.0,
            1.5
        )

    state.carcassHistoryStress[index] =
        clamp(
            lowPass(
                state.carcassHistoryStress[index],
                c.history,
                p.tauCarcass,
                dt
            ),
            0.0,
            1.5
        )

    local verticalScore =
        1.0
        -
        smoothstep(
            p.verticalGood,
            p.verticalBad,
            state.carcassVertical[index]
        )

    local hysteresisScore =
        1.0
        -
        smoothstep(
            p.hysteresisGood,
            p.hysteresisBad,
            state.carcassHysteresis[index]
        )

    local delayScore =
        1.0
        -
        smoothstep(
            p.delayGood,
            p.delayBad,
            state.carcassDelay[index]
        )

    local supportScore = clamp(state.carcassSupport[index], 0.0, 1.0)
    local gripGateScore = clamp(state.carcassGripGate[index], 0.0, 1.0)
    local recoveryLift = clamp(state.carcassRecovery[index] * p.recoveryAssist, 0.0, p.recoveryAssist)

    local carcassScore =
        1.0
        -
        (1.0 - supportScore) * p.carcassSupportWeight
        -
        (1.0 - gripGateScore) * p.carcassGripWeight
        -
        (1.0 - verticalScore) * p.verticalWeight
        -
        (1.0 - hysteresisScore) * p.hysteresisWeight
        -
        (1.0 - delayScore) * p.delayWeight
        +
        recoveryLift

    carcassScore = clamp(carcassScore, 0.0, 1.0)

    state.verticalScore[index] = verticalScore
    state.hysteresisScore[index] = hysteresisScore
    state.delayScore[index] = delayScore
    state.carcassScore[index] = carcassScore

    local rawQuality =
        loadScore
        *
        slipScore
        *
        (0.62 + 0.38 * stabilityScore)
        *
        carcassScore

    rawQuality = clamp(rawQuality, p.minQuality, p.maxQuality)

    local prevQuality = state.quality[index] or rawQuality
    local tau = rawQuality < prevQuality and p.tauFall or p.tauRise

    local quality =
        clamp(
            lowPass(
                prevQuality,
                rawQuality,
                tau,
                dt
            ),
            p.minQuality,
            p.maxQuality
        )

    local trust =
        quality
        *
        (0.74 + 0.26 * stabilityScore)
        *
        (0.82 + 0.18 * carcassScore)

    trust = clamp(trust, 0.0, 1.0)

    local loss = clamp(1.0 - math.min(quality, trust), 0.0, 1.0)

    local statusId, statusText =
        statusFromScores(
            loadScore,
            slipScore,
            stabilityScore,
            combinedSlip,
            rawQuality,
            carcassScore,
            delayScore,
            state.carcassHysteresis[index]
        )

    state.quality[index] = quality
    state.rawQuality[index] = rawQuality
    state.trust[index] = trust
    state.loss[index] = loss
    state.combinedSlip[index] = combinedSlip

    state.loadScore[index] = loadScore
    state.slipScore[index] = slipScore
    state.stabilityScore[index] = stabilityScore

    state.statusId[index] = statusId
    state.statusText[index] = statusText
    state.prevLoad[index] = load

    exportWheel(index)

    return quality, rawQuality, trust, loss, combinedSlip
end

function M.init()
    state.initialized = true
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    dt = clamp(num(dt, 1.0 / 60.0), 0.0001, M.params.maxDt)

    updateDebugGate(dt)

    state.updateCount = (state.updateCount or 0) + 1

    car = car or safeGetCar()

    state.carcassLinked = false
    state.loadLinked = false
    state.tireStateLinked = false
    state.wheelApiLinked = false

    if not car then
        state.status = "NO CAR"
        state.wheelsValid = false
        state.speedKmh = 0.0

        local sumQ = 0.0
        local sumLoss = 0.0

        for i = 0, 3 do
            clearWheel(i, "NO CAR", dt)
            sumQ = sumQ + state.quality[i]
            sumLoss = sumLoss + state.loss[i]
            exportWheel(i)
        end

        state.avgQuality = sumQ * 0.25
        state.avgRawQuality = 0.0
        state.avgTrust = 0.0
        state.avgLoss = sumLoss * 0.25
        state.minQualityNow = 0.0
        state.maxCombinedSlip = 0.0

        exportGlobal()
        return
    end

    if not getWheels(car) then
        state.status = "NO WHEELS"
        state.wheelsValid = false
        state.speedKmh = getSpeedKmh(car)

        for i = 0, 3 do
            clearWheel(i, "NO WHEELS", dt)
            exportWheel(i)
        end

        state.avgQuality = 0.0
        state.avgRawQuality = 0.0
        state.avgTrust = 0.0
        state.avgLoss = 1.0
        state.minQualityNow = 0.0
        state.maxCombinedSlip = 0.0

        exportGlobal()
        return
    end

    state.status = "RUNNING"
    state.wheelsValid = true
    state.speedKmh = getSpeedKmh(car)

    local sumQ = 0.0
    local sumRaw = 0.0
    local sumTrust = 0.0
    local sumLoss = 0.0
    local minQualityNow = 1.0
    local maxCombinedSlip = 0.0

    for i = 0, 3 do
        local quality, rawQuality, trust, loss, combinedSlip =
            calculateWheel(
                i,
                car,
                dt
            )

        sumQ = sumQ + quality
        sumRaw = sumRaw + rawQuality
        sumTrust = sumTrust + trust
        sumLoss = sumLoss + loss

        minQualityNow = math.min(minQualityNow, quality)
        maxCombinedSlip = math.max(maxCombinedSlip, combinedSlip)
    end

    state.avgQuality = sumQ * 0.25
    state.avgRawQuality = sumRaw * 0.25
    state.avgTrust = sumTrust * 0.25
    state.avgLoss = sumLoss * 0.25
    state.minQualityNow = minQualityNow
    state.maxCombinedSlip = maxCombinedSlip

    state.initialized = true

    exportGlobal()
end

function M.getQuality(index)
    return state.quality[index] or 0.0
end

function M.getRawQuality(index)
    return state.rawQuality[index] or 0.0
end

function M.getTrust(index)
    return state.trust[index] or 0.0
end

function M.getLoss(index)
    return state.loss[index] or 1.0
end

function M.getStatus(index)
    return state.statusText[index] or "UNKNOWN"
end

function M.getState()
    return state
end

function M.debugStr(index)
    local i = tonumber(index or 0) or 0

    return string.format(
        "Contact %s | Q %.2f Raw %.2f Trust %.2f Loss %.2f | %s\n" ..
        "Load %.0f LR %.0f | SA %.3f SR %.3f CS %.2f\n" ..
        "Score L %.2f S %.2f Stab %.2f Carc %.2f\n" ..
        "Carc Sup %.2f Gate %.2f V %.2f H %.2f D %.2f | Link %s",
        WHEEL_NAME[i] or tostring(i),
        num(state.quality[i], 0.0),
        num(state.rawQuality[i], 0.0),
        num(state.trust[i], 0.0),
        num(state.loss[i], 1.0),
        tostring(state.statusText[i] or "UNKNOWN"),

        num(state.load[i], 0.0),
        num(state.loadRate[i], 0.0),
        num(state.slipAngle[i], 0.0),
        num(state.slipRatio[i], 0.0),
        num(state.combinedSlip[i], 0.0),

        num(state.loadScore[i], 0.0),
        num(state.slipScore[i], 0.0),
        num(state.stabilityScore[i], 0.0),
        num(state.carcassScore[i], 1.0),

        num(state.carcassSupport[i], 1.0),
        num(state.carcassGripGate[i], 1.0),
        num(state.carcassVertical[i], 0.0),
        num(state.carcassHysteresis[i], 0.0),
        num(state.carcassDelay[i], 0.0),
        state.carcassLinked and "OK" or "NIL"
    )
end

function M.drawDebug()
    if not ui or not ui.text then
        return
    end

    ui.separator()
    ui.text("=== CONTACT QUALITY V1.1.5 ===")
    ui.text(
        string.format(
            "Status: %s | Speed: %.1f km/h | AvgQ %.2f | AvgLoss %.2f | Carcass %s",
            tostring(state.status),
            num(state.speedKmh, 0.0),
            num(state.avgQuality, 0.0),
            num(state.avgLoss, 1.0),
            state.carcassLinked and "OK" or "NIL"
        )
    )

    for i = 0, 3 do
        ui.text(
            string.format(
                "%s Q %.2f Trust %.2f Loss %.2f Raw %.2f Load %.0f CS %.2f L %.2f S %.2f C %.2f %s",
                WHEEL_NAME[i] or tostring(i),
                num(state.quality[i], 0.0),
                num(state.trust[i], 0.0),
                num(state.loss[i], 1.0),
                num(state.rawQuality[i], 0.0),
                num(state.load[i], 0.0),
                num(state.combinedSlip[i], 0.0),
                num(state.loadScore[i], 0.0),
                num(state.slipScore[i], 0.0),
                num(state.carcassScore[i], 1.0),
                tostring(state.statusText[i] or "UNKNOWN")
            )
        )
    end
end

return M
