---@diagnostic disable: undefined-global

--============================================================
-- slip_recovery.lua
-- ACNextGen V1.1.5 Stable
-- Slip Recovery / Recontact Interpreter
--
-- App-side signal model only.
-- No direct AC physics rewrite.
--============================================================

local M = {}

local WHEEL_NAMES = {
    [0] = "FL",
    [1] = "FR",
    [2] = "RL",
    [3] = "RR",
}

local PHASE = {
    STABLE         = 1,
    SLIDING        = 2,
    SATURATED      = 3,
    EARLY_RECOVERY = 4,
    RECOVERING     = 5,
    RECONTACT      = 6,
    SNAP_RISK      = 7,
    UNLOADED       = 8,
    HEAT_SOAK      = 9,
    DIRTY_RETURN   = 10,
}

local PHASE_TEXT = {
    [PHASE.STABLE]         = "STABLE",
    [PHASE.SLIDING]        = "SLIDING",
    [PHASE.SATURATED]      = "SATURATED",
    [PHASE.EARLY_RECOVERY] = "EARLY_RECOVERY",
    [PHASE.RECOVERING]     = "RECOVERING",
    [PHASE.RECONTACT]      = "RECONTACT",
    [PHASE.SNAP_RISK]      = "SNAP_RISK",
    [PHASE.UNLOADED]       = "UNLOADED",
    [PHASE.HEAT_SOAK]      = "HEAT_SOAK",
    [PHASE.DIRTY_RETURN]   = "DIRTY_RETURN",
}

M.params = {
    slipGood = 0.18,
    slipRecoverStart = 0.55,
    slipBad = 1.20,
    slipSaturated = 1.55,

    slipRatioScale = 1.0,
    slipAngleScale = 1.0,

    contactGood = 0.72,
    contactBad = 0.34,
    recontactQuality = 0.62,
    recontactRate = 1.80,

    trustGood = 0.70,
    trustBad = 0.32,

    loadReference = 3200.0,
    loadLow = 250.0,
    loadReturnMin = 900.0,

    slideBuildGain = 1.22,
    saturationBuildGain = 0.58,
    memoryBuildGain = 0.40,
    carcassBuildGain = 0.30,
    lowContactBuildGain = 0.50,
    yawOversteerBuildGain = 0.30,
    loadPathLossBuildGain = 0.25,

    thermalBuildGain = 0.28,
    abrasionBuildGain = 0.24,
    historyBuildGain = 0.34,
    heatSeedBuildGain = 0.18,
    hysteresisBuildGain = 0.22,
    bristleBuildGain = 0.16,
    lowTrustBuildGain = 0.32,

    slideDecayBase = 0.34,
    slideDecayContactGain = 0.32,
    slideDecayTrustGain = 0.20,
    slideDecayGripReturnGain = 0.16,
    slideDecayHeatLoss = 0.22,
    slideDecayAbrasionLoss = 0.18,
    slideDecayHistoryLoss = 0.20,

    recoveryContactGain = 0.48,
    recoveryTrustGain = 0.36,
    recoverySlipDropGain = 0.55,
    recoveryReturnForceGain = 0.42,
    recoveryLoadGain = 0.20,
    recoveryCarcassGain = 0.18,
    recoveryYawGain = 0.16,
    recoveryScalarGain = 0.46,
    recoveryBiasGain = 0.22,

    recoveryHeatLoss = 0.32,
    recoveryAbrasionLoss = 0.26,
    recoveryHistoryLoss = 0.28,
    recoveryShockLoss = 0.35,
    recoveryHysteresisLoss = 0.18,
    recoveryDelayLoss = 0.18,

    gripReturnBase = 0.20,
    gripReturnRecoveryGain = 0.45,
    gripReturnContactGain = 0.20,
    gripReturnTrustGain = 0.18,
    gripReturnMemoryLoss = 0.30,
    gripReturnDelayLoss = 0.16,
    gripReturnHeatLoss = 0.22,
    gripReturnAbrasionLoss = 0.18,
    gripReturnHistoryLoss = 0.20,
    gripReturnShockLoss = 0.20,
    gripReturnTireGripGain = 0.30,

    snapContactGain = 0.50,
    snapTrustGain = 0.34,
    snapSlipDropGain = 0.76,
    snapMemoryGain = 0.46,
    snapReturnGain = 0.55,
    snapRecoveryBiasGain = 0.42,
    snapRecontactShockGain = 0.62,
    snapYawGain = 0.42,
    snapLoadPathGain = 0.22,
    snapHysteresisGain = 0.18,
    snapRearBias = 1.25,
    snapFrontBias = 0.72,

    biteContactGain = 0.40,
    biteSlipDropGain = 0.36,
    biteReturnGain = 0.42,
    biteTrustGain = 0.28,
    biteShockGain = 0.34,

    roadShockRecoveryLoss = 0.12,
    roadSurfaceSnapGain = 0.18,
    roadPathLossGain = 0.12,
    loadPathDeliveryGain = 0.12,
    contactLossExtraBuildGain = 0.16,

    tauSlip = 0.050,
    tauContact = 0.075,
    tauLoad = 0.120,
    tauRecoveryRise = 0.060,
    tauRecoveryFall = 0.150,
    tauGripReturn = 0.110,
    tauSnapRise = 0.050,
    tauSnapFall = 0.170,
    tauSlideMemory = 0.240,
    tauRecontact = 0.085,
    tauBite = 0.070,

    maxSlideMemory = 1.0,
    maxRecovery = 1.0,
    maxSnapRisk = 1.0,
    maxBite = 1.0,

    minDt = 0.00005,
    maxDt = 0.050,
    debugStoreInterval = 0.25,
}

M.state = {
    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    avgRecovery = 0.0,
    avgGripReturn = 1.0,
    avgSnapRisk = 0.0,
    avgSlideMemory = 0.0,
    avgRecontact = 0.0,
    avgBite = 0.0,
    avgDirtyReturn = 0.0,

    maxRecovery = 0.0,
    maxSnapRisk = 0.0,
    maxSlideMemory = 0.0,
    rearRecovery = 0.0,
    rearSnapRisk = 0.0,

    contactLinked = false,
    memoryLinked = false,
    carcassLinked = false,
    yawLinked = false,
    loadPathLinked = false,
    roadLinked = false,

    yawOversteer = 0.0,
    yawUndersteer = 0.0,
    yawRecovery = 0.0,
    yawSpinRisk = 0.0,

    debugStoreTimer = 999.0,
    debugStoreNow = true,

    wheels = {},
}

for i = 0, 3 do
    M.state.wheels[i] = {
        active = false,

        slip = 0.0,
        rawSlip = 0.0,
        prevSlip = 0.0,
        slipRate = 0.0,
        slipDrop = 0.0,

        slipAngle = 0.0,
        slipRatio = 0.0,

        contactQuality = 1.0,
        rawContactQuality = 1.0,
        contactTrust = 1.0,
        prevContactQuality = 1.0,
        contactRate = 0.0,
        contactRise = 0.0,
        contactStatus = 0,
        stability = 1.0,
        contactLoss = 0.0,

        load = 0.0,
        loadFactor = 0.0,

        tireMemory = 0.0,
        tireGrip = 1.0,
        tirePhaseId = 0,
        tireRecontact = 0.0,

        thermalMemory = 0.0,
        abrasionMemory = 0.0,
        historyMemory = 0.0,
        microHeat = 0.0,
        macroHeat = 0.0,
        virtualTemp = 25.0,
        memoryRecoveryScalar = 1.0,
        memoryRecontactShock = 0.0,

        carcassEnergy = 0.0,
        carcassDelay = 0.0,
        carcassReturn = 0.0,
        carcassDeformation = 0.0,
        carcassSupport = 1.0,
        carcassGripGate = 1.0,
        verticalNorm = 0.0,
        carcassHysteresis = 0.0,
        recoveryBias = 0.0,
        bristleLat = 0.0,
        bristleLong = 0.0,

        loadPathEfficiency = 1.0,
        loadPathLoss = 0.0,
        loadPathDelay = 0.0,
        loadPathContact = 0.0,
        loadPathIntegrity = 1.0,
        loadPathDelivery = 1.0,

        roadImpact = 0.0,
        roadShock = 0.0,
        roadSurfaceLimit = 0.0,
        roadPathLoss = 0.0,
        roadModuleHint = 0.0,

        slideIntensity = 0.0,
        slideMemory = 0.0,
        recoveryRate = 0.0,
        gripReturn = 1.0,
        snapRisk = 0.0,
        recontactSharpness = 0.0,
        bite = 0.0,
        dirtyReturn = 0.0,

        phaseId = PHASE.STABLE,
        phase = "STABLE",
    }

    M.state[i] = M.state.wheels[i]
end

M.debug = M.state

--============================================================
-- Utility
--============================================================

local function safeNumber(value, defaultValue)
    local n = tonumber(value)
    if n == nil or n ~= n or n == math.huge or n == -math.huge then
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
    return math.abs(safeNumber(v, 0.0))
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.0)
    dt = safeNumber(dt, 0.0)

    if tau <= 0.0 then
        return target
    end

    return current + (target - current) * clamp(dt / math.max(tau + dt, 0.0001), 0.0, 1.0)
end

local function smoothstep(edge0, edge1, x)
    edge0 = safeNumber(edge0, 0.0)
    edge1 = safeNumber(edge1, 1.0)
    x = safeNumber(x, 0.0)

    if edge0 == edge1 then
        return x >= edge1 and 1.0 or 0.0
    end

    local t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)
end

local function safeField(obj, field, defaultValue)
    if not obj then return defaultValue end

    local ok, value = pcall(function()
        return obj[field]
    end)

    if not ok or value == nil then
        return defaultValue
    end

    return value
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

local function safeStore(key, value)
    if not ac or not ac.store then
        return
    end

    pcall(function()
        ac.store(key, value)
    end)
end

local function loadNumAlt(defaultValue, ...)
    local keys = { ... }

    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return safeNumber(value, defaultValue), keys[i]
        end
    end

    return defaultValue, nil
end

local function updateDebugGate(dt)
    M.state.debugStoreTimer = (M.state.debugStoreTimer or 0.0) + (dt or 0.0)

    if M.state.debugStoreTimer >= M.params.debugStoreInterval then
        M.state.debugStoreTimer = 0.0
        M.state.debugStoreNow = true
    else
        M.state.debugStoreNow = false
    end
end

local function getWheel(index, car)
    if not car then
        return nil
    end

    local wheels = safeField(car, "wheels", nil)
    if not wheels then
        return nil
    end

    local ok, wheel = pcall(function()
        return wheels[index]
    end)
    if ok and wheel then
        return wheel
    end

    ok, wheel = pcall(function()
        return wheels[index + 1]
    end)
    if ok and wheel then
        return wheel
    end

    return nil
end

local function getCarSafe()
    if not ac or type(ac.getCar) ~= "function" then
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

local function hasWheels(car)
    if not car then
        return false
    end

    local wheels = safeField(car, "wheels", nil)
    return wheels ~= nil
end

--============================================================
-- Input readers
--============================================================

local function readSlip(index, wheel)
    local slipAngle = loadNumAlt(
        nil,
        "ngp_tire_slip_angle_" .. index,
        "ngp_slip_angle_" .. index,
        "ngp_filtered_slip_angle_" .. index,
        "ngp_memory_slip_angle_" .. index,
        "ngp_contact_slip_angle_" .. index,
        "ngp_hub_slipA_" .. index
    )

    if slipAngle == nil then
        slipAngle = safeField(wheel, "slipAngle", 0.0)
    end

    local slipRatio = loadNumAlt(
        nil,
        "ngp_tire_slip_ratio_" .. index,
        "ngp_slip_ratio_" .. index,
        "ngp_filtered_slip_ratio_" .. index,
        "ngp_memory_slip_ratio_" .. index,
        "ngp_contact_slip_ratio_" .. index,
        "ngp_hub_slipR_" .. index
    )

    if slipRatio == nil then
        slipRatio = safeField(wheel, "slipRatio", 0.0)
    end

    local combinedSlip = safeLoadRaw("ngp_contact_combined_slip_" .. index)
    if combinedSlip ~= nil then
        M.state.contactLinked = true
        return clamp(abs(combinedSlip), 0.0, 3.0), abs(slipAngle), abs(slipRatio)
    end

    combinedSlip = safeLoadRaw("ngp_slip_recovery_slip_" .. index)
    if combinedSlip ~= nil then
        return clamp(abs(combinedSlip), 0.0, 3.0), abs(slipAngle), abs(slipRatio)
    end

    local sr = abs(slipRatio) * M.params.slipRatioScale
    local sa = abs(slipAngle) * M.params.slipAngleScale

    local combined = math.sqrt(
        (sr / 0.35) * (sr / 0.35) +
        (sa / 0.22) * (sa / 0.22)
    )

    return clamp(combined, 0.0, 3.0), abs(slipAngle), abs(slipRatio)
end

local function readLoad(index, wheel)
    local load = safeLoadRaw("ngp_contact_load_" .. index)
    if load ~= nil then
        M.state.contactLinked = true
        return safeNumber(load, 0.0)
    end

    load = safeLoadRaw("ngp_wheel_load_" .. index)
    if load ~= nil then
        return safeNumber(load, M.params.loadReference)
    end

    load = safeLoadRaw("ngp_load_path_load_" .. index)
    if load ~= nil then
        M.state.loadPathLinked = true
        return safeNumber(load, M.params.loadReference)
    end

    load = safeLoadRaw("ngp_tire_state_load_" .. index)
    if load ~= nil then
        return safeNumber(load, 0.0)
    end

    load = safeLoadRaw("ngp_dlt_load_" .. index)
    if load ~= nil then
        local n = safeNumber(load, 0.0)
        if abs(n) > 900.0 then
            return abs(n)
        end
        return M.params.loadReference + n
    end

    if wheel then
        local wLoad = safeField(wheel, "load", nil)
        if wLoad ~= nil then return safeNumber(wLoad, 0.0) end

        wLoad = safeField(wheel, "loadK", nil)
        if wLoad ~= nil then return safeNumber(wLoad, 0.0) end

        wLoad = nil
        if wLoad ~= nil then return safeNumber(wLoad, 0.0) end

        wLoad = nil
        if wLoad ~= nil then return safeNumber(wLoad, 0.0) end

        wLoad = nil
        if wLoad ~= nil then return safeNumber(wLoad, 0.0) end
    end

    return 0.0
end

local function readContact(index, wheelState, dt)
    local contactQuality = loadNumAlt(
        nil,
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index
    )

    local rawContact = loadNumAlt(
        nil,
        "ngp_contact_raw_" .. index,
        "ngp_contact_raw_quality_" .. index
    )

    local trust = loadNumAlt(
        nil,
        "ngp_contact_trust_" .. index,
        "ngp_contact_grip_gate_" .. index,
        "ngp_carcass_grip_gate_" .. index,
        "ngp_recovery_trust_" .. index
    )

    local stability = safeLoadRaw("ngp_contact_stability_score_" .. index)
    local status = safeLoadRaw("ngp_contact_status_" .. index)
    local loss = safeLoadRaw("ngp_contact_loss_" .. index)

    if contactQuality ~= nil or rawContact ~= nil or trust ~= nil or stability ~= nil or status ~= nil or loss ~= nil then
        M.state.contactLinked = true
    end

    if contactQuality == nil and loss ~= nil then
        contactQuality = 1.0 - clamp(safeNumber(loss, 0.0), 0.0, 1.0)
    end

    wheelState.rawContactQuality = clamp(safeNumber(contactQuality, 1.0), 0.0, 1.2)

    wheelState.contactQuality = lowPass(
        wheelState.contactQuality,
        wheelState.rawContactQuality,
        M.params.tauContact,
        dt
    )

    wheelState.contactTrust = clamp(safeNumber(trust, wheelState.contactQuality), 0.0, 1.2)
    wheelState.stability = clamp(safeNumber(stability, 1.0), 0.0, 1.2)
    wheelState.contactStatus = safeNumber(status, 0.0)
    wheelState.contactLoss = clamp(1.0 - math.min(wheelState.contactQuality, wheelState.contactTrust), 0.0, 1.0)

    return rawContact
end

local function readTireMemory(index, wheelState)
    local memory = loadNumAlt(
        nil,
        "ngp_tire_memory_" .. index,
        "ngp_tyre_memory_" .. index,
        "ngp_rubber_memory_" .. index,
        "ngp_memory_" .. index
    )

    local grip = loadNumAlt(
        nil,
        "ngp_memory_grip_" .. index,
        "ngp_tire_memory_grip_" .. index,
        "ngp_tyre_memory_grip_" .. index
    )

    local phaseId = safeLoadRaw("ngp_tire_memory_phase_id_" .. index)
    if phaseId == nil then phaseId = safeLoadRaw("ngp_tyre_memory_phase_id_" .. index) end

    local recontact = safeLoadRaw("ngp_tire_memory_recontact_" .. index)
    if recontact == nil then recontact = safeLoadRaw("ngp_tyre_memory_recontact_" .. index) end

    local thermal = loadNumAlt(
        nil,
        "ngp_tire_memory_thermal_" .. index,
        "ngp_tyre_memory_thermal_" .. index,
        "ngp_memory_thermal_" .. index
    )

    local abrasion = loadNumAlt(
        nil,
        "ngp_tire_memory_abrasion_" .. index,
        "ngp_tyre_memory_abrasion_" .. index,
        "ngp_memory_abrasion_" .. index
    )

    local history = loadNumAlt(
        nil,
        "ngp_tire_memory_history_" .. index,
        "ngp_tyre_memory_history_" .. index,
        "ngp_memory_history_" .. index
    )

    local heat = loadNumAlt(
        nil,
        "ngp_tire_memory_heat_" .. index,
        "ngp_tyre_memory_heat_" .. index,
        "ngp_memory_heat_" .. index
    )

    local microHeat = safeLoadRaw("ngp_memory_micro_heat_" .. index)
    local macroHeat = safeLoadRaw("ngp_memory_macro_heat_" .. index)
    local virtualTemp = safeLoadRaw("ngp_memory_virtual_temp_" .. index)

    local recoveryScalar = loadNumAlt(
        nil,
        "ngp_tire_memory_recovery_scalar_" .. index,
        "ngp_tyre_memory_recovery_scalar_" .. index,
        "ngp_memory_recovery_scalar_" .. index
    )

    local recontactShock = loadNumAlt(
        nil,
        "ngp_memory_recontact_shock_" .. index,
        "ngp_tire_memory_recontact_shock_" .. index,
        "ngp_tyre_memory_recontact_shock_" .. index
    )

    if memory ~= nil or grip ~= nil or phaseId ~= nil or recontact ~= nil
    or thermal ~= nil or abrasion ~= nil or history ~= nil or heat ~= nil
    or recoveryScalar ~= nil or recontactShock ~= nil then
        M.state.memoryLinked = true
    end

    wheelState.tireMemory = clamp(safeNumber(memory, 0.0), 0.0, 1.0)
    wheelState.tireGrip = clamp(safeNumber(grip, 1.0), 0.0, 1.2)
    wheelState.tirePhaseId = safeNumber(phaseId, 0.0)
    wheelState.tireRecontact = clamp(safeNumber(recontact, 0.0), 0.0, 1.0)

    wheelState.thermalMemory = clamp(safeNumber(thermal, safeNumber(heat, 0.0)), 0.0, 1.0)
    wheelState.abrasionMemory = clamp(safeNumber(abrasion, 0.0), 0.0, 1.0)
    wheelState.historyMemory = clamp(safeNumber(history, wheelState.tireMemory), 0.0, 1.0)
    wheelState.microHeat = clamp(safeNumber(microHeat, wheelState.thermalMemory), 0.0, 1.0)
    wheelState.macroHeat = clamp(safeNumber(macroHeat, wheelState.thermalMemory), 0.0, 1.0)
    wheelState.virtualTemp = safeNumber(virtualTemp, 25.0)
    wheelState.memoryRecoveryScalar = clamp(safeNumber(recoveryScalar, 1.0), 0.0, 1.2)
    wheelState.memoryRecontactShock = clamp(safeNumber(recontactShock, 0.0), 0.0, 1.0)
end

local function readCarcass(index, wheelState)
    local energy = loadNumAlt(
        nil,
        "ngp_tire_sidewall_energy_" .. index,
        "ngp_sidewall_energy_" .. index
    )

    local delay = loadNumAlt(
        nil,
        "ngp_tire_contact_delay_" .. index,
        "ngp_contact_delay_" .. index
    )

    local returnForce = loadNumAlt(
        nil,
        "ngp_tire_return_force_" .. index,
        "ngp_return_force_" .. index
    )

    local deformation = loadNumAlt(
        nil,
        "ngp_tire_deformation_" .. index,
        "ngp_carcass_deformation_" .. index
    )

    local support = loadNumAlt(
        nil,
        "ngp_carcass_support_" .. index,
        "ngp_contact_carcass_support_" .. index
    )

    local gripGate = loadNumAlt(
        nil,
        "ngp_carcass_grip_gate_" .. index,
        "ngp_contact_grip_gate_" .. index
    )

    local verticalNorm = loadNumAlt(
        nil,
        "ngp_carcass_vertical_norm_" .. index,
        "ngp_contact_vertical_norm_" .. index
    )

    local hysteresis = loadNumAlt(
        nil,
        "ngp_carcass_hysteresis_" .. index,
        "ngp_contact_hysteresis_" .. index
    )

    local recoveryBias = loadNumAlt(
        nil,
        "ngp_carcass_recovery_bias_" .. index,
        "ngp_contact_recovery_bias_" .. index
    )

    local bristleLat = safeLoadRaw("ngp_carcass_bristle_lat_" .. index)
    local bristleLong = safeLoadRaw("ngp_carcass_bristle_long_" .. index)

    if energy ~= nil or delay ~= nil or returnForce ~= nil or deformation ~= nil
    or support ~= nil or gripGate ~= nil or verticalNorm ~= nil or hysteresis ~= nil
    or recoveryBias ~= nil or bristleLat ~= nil or bristleLong ~= nil then
        M.state.carcassLinked = true
    end

    wheelState.carcassEnergy = clamp(safeNumber(energy, 0.0), 0.0, 1.5)
    wheelState.carcassDelay = clamp(safeNumber(delay, 0.0), 0.0, 1.5)
    wheelState.carcassReturn = clamp(safeNumber(returnForce, 0.0), 0.0, 1.5)
    wheelState.carcassDeformation = clamp(safeNumber(deformation, 0.0), 0.0, 1.5)

    wheelState.carcassSupport = clamp(safeNumber(support, 1.0), 0.0, 1.2)
    wheelState.carcassGripGate = clamp(safeNumber(gripGate, 1.0), 0.0, 1.2)
    wheelState.verticalNorm = clamp(safeNumber(verticalNorm, 0.0), 0.0, 1.5)
    wheelState.carcassHysteresis = clamp(safeNumber(hysteresis, 0.0), 0.0, 1.2)
    wheelState.recoveryBias = clamp(safeNumber(recoveryBias, 0.0), 0.0, 1.2)
    wheelState.bristleLat = clamp(safeNumber(bristleLat, 0.0), 0.0, 1.5)
    wheelState.bristleLong = clamp(safeNumber(bristleLong, 0.0), 0.0, 1.5)
end

local function readLoadPath(index, wheelState)
    local eff = loadNumAlt(
        nil,
        "ngp_load_path_efficiency_" .. index,
        "ngp_lp_eff_" .. index
    )

    local loss = loadNumAlt(
        nil,
        "ngp_load_path_loss_" .. index,
        "ngp_lp_loss_" .. index
    )

    local delay = loadNumAlt(
        nil,
        "ngp_load_path_delay_" .. index,
        "ngp_lp_delay_" .. index
    )

    local contact = loadNumAlt(
        nil,
        "ngp_load_path_contact_" .. index,
        "ngp_lp_contact_" .. index
    )

    local integrity = loadNumAlt(
        nil,
        "ngp_load_path_integrity_" .. index,
        "ngp_lp_integrity_" .. index
    )

    local delivery = loadNumAlt(
        nil,
        "ngp_load_path_tire_delivery_" .. index,
        "ngp_lp_tire_delivery_" .. index
    )

    if eff ~= nil or loss ~= nil or delay ~= nil or contact ~= nil or integrity ~= nil or delivery ~= nil then
        M.state.loadPathLinked = true
    end

    wheelState.loadPathEfficiency = clamp(safeNumber(eff, 1.0), 0.0, 1.2)
    wheelState.loadPathLoss = clamp(safeNumber(loss, 0.0), 0.0, 1.0)
    wheelState.loadPathDelay = clamp(safeNumber(delay, 0.0), 0.0, 1.0)
    wheelState.loadPathContact = clamp(safeNumber(contact, 0.0), 0.0, 1.0)
    wheelState.loadPathIntegrity = clamp(safeNumber(integrity, 1.0), 0.0, 1.2)
    wheelState.loadPathDelivery = clamp(safeNumber(delivery, 1.0), 0.0, 1.2)
end

local function readRoadInput(index, wheelState)
    local impact = loadNumAlt(
        nil,
        "ngp_road_impact_" .. index,
        "ngp_rii_impact_" .. index,
        "ngp_road_input_impact_" .. index
    )

    local shock = loadNumAlt(
        nil,
        "ngp_road_shock_" .. index,
        "ngp_rii_shock_" .. index
    )

    local surface = loadNumAlt(
        nil,
        "ngp_road_surface_limit_" .. index,
        "ngp_rii_surface_limit_" .. index
    )

    local pathLoss = loadNumAlt(
        nil,
        "ngp_road_path_loss_" .. index,
        "ngp_rii_path_loss_" .. index
    )

    local hint = loadNumAlt(
        nil,
        "ngp_road_module_hint_" .. index,
        "ngp_rii_hint_" .. index
    )

    if impact ~= nil or shock ~= nil or surface ~= nil or pathLoss ~= nil or hint ~= nil then
        M.state.roadLinked = true
    end

    wheelState.roadImpact = clamp(safeNumber(impact, 0.0), 0.0, 1.2)
    wheelState.roadShock = clamp(safeNumber(shock, 0.0), 0.0, 1.2)
    wheelState.roadSurfaceLimit = clamp(safeNumber(surface, 0.0), 0.0, 1.2)
    wheelState.roadPathLoss = clamp(safeNumber(pathLoss, 0.0), 0.0, 1.2)
    wheelState.roadModuleHint = clamp(safeNumber(hint, 0.0), 0.0, 1.2)
end

local function readYawBudget()
    local oversteer = safeLoadRaw("ngp_yaw_oversteer_energy")
    if oversteer == nil then oversteer = safeLoadRaw("ngp_oversteer_energy") end
    if oversteer == nil then oversteer = safeLoadRaw("ngp_yaw_budget_oversteer") end

    local understeer = safeLoadRaw("ngp_yaw_understeer_energy")
    if understeer == nil then understeer = safeLoadRaw("ngp_understeer_energy") end
    if understeer == nil then understeer = safeLoadRaw("ngp_yaw_budget_understeer") end

    local recovery = safeLoadRaw("ngp_yaw_recovery_phase")
    if recovery == nil then recovery = safeLoadRaw("ngp_recovery_phase") end
    if recovery == nil then recovery = safeLoadRaw("ngp_yaw_budget_recovery") end

    local spin = safeLoadRaw("ngp_yaw_spin_risk")
    if spin == nil then spin = safeLoadRaw("ngp_spin_risk") end
    if spin == nil then spin = safeLoadRaw("ngp_yaw_budget_spin_risk") end

    if oversteer ~= nil or understeer ~= nil or recovery ~= nil or spin ~= nil then
        M.state.yawLinked = true
    end

    M.state.yawOversteer = clamp(safeNumber(oversteer, 0.0), 0.0, 1.5)
    M.state.yawUndersteer = clamp(safeNumber(understeer, 0.0), 0.0, 1.5)
    M.state.yawRecovery = clamp(safeNumber(recovery, 0.0), 0.0, 1.5)
    M.state.yawSpinRisk = clamp(safeNumber(spin, 0.0), 0.0, 1.5)
end

--============================================================
-- Core
--============================================================

local function calculateSlideIntensity(wheelState, index)
    local slipPart = smoothstep(M.params.slipGood, M.params.slipBad, wheelState.slip)
    local saturated = smoothstep(M.params.slipBad, M.params.slipSaturated, wheelState.slip)
    local lowContact = 1.0 - clamp(wheelState.contactQuality, 0.0, 1.0)
    local lowTrust = 1.0 - clamp(wheelState.contactTrust, 0.0, 1.0)
    local rearFactor = index >= 2 and 1.0 or 0.55

    local yawPart = M.state.yawOversteer * rearFactor
    local loadLoss = math.max(wheelState.loadPathLoss, wheelState.roadPathLoss * M.params.roadPathLossGain)
    local bristleEnergy = clamp((wheelState.bristleLat + wheelState.bristleLong) * 0.5, 0.0, 1.5)

    local slide =
        slipPart * M.params.slideBuildGain +
        saturated * M.params.saturationBuildGain +
        wheelState.tireMemory * M.params.memoryBuildGain +
        wheelState.carcassDelay * M.params.carcassBuildGain +
        wheelState.carcassHysteresis * M.params.hysteresisBuildGain +
        bristleEnergy * M.params.bristleBuildGain +
        lowContact * M.params.lowContactBuildGain +
        lowTrust * M.params.lowTrustBuildGain +
        yawPart * M.params.yawOversteerBuildGain +
        loadLoss * M.params.loadPathLossBuildGain +
        wheelState.thermalMemory * M.params.thermalBuildGain +
        wheelState.abrasionMemory * M.params.abrasionBuildGain +
        wheelState.historyMemory * M.params.historyBuildGain +
        math.max(wheelState.microHeat, wheelState.macroHeat) * M.params.heatSeedBuildGain +
        wheelState.contactLoss * M.params.contactLossExtraBuildGain

    return clamp(slide, 0.0, 1.0), saturated
end

local function calculateRecovery(wheelState, dt, index)
    local contactPart = smoothstep(M.params.contactBad, M.params.contactGood, wheelState.contactQuality)
    local trustPart = smoothstep(M.params.trustBad, M.params.trustGood, wheelState.contactTrust)
    local slipDropPart = clamp(wheelState.slipDrop / math.max(M.params.slipRecoverStart, 0.001), 0.0, 1.0)
    local returnPart = clamp(wheelState.carcassReturn, 0.0, 1.0)
    local loadPart = smoothstep(M.params.loadReturnMin, M.params.loadReference, wheelState.load)
    local carcassPart = clamp((1.0 - wheelState.carcassDelay) * wheelState.carcassSupport, 0.0, 1.0)
    local yawPart = clamp(M.state.yawRecovery, 0.0, 1.0)
    local scalarPart = clamp(wheelState.memoryRecoveryScalar, 0.0, 1.2)
    local recoveryBias = clamp(wheelState.recoveryBias, 0.0, 1.0)
    local deliveryPart = math.max(wheelState.loadPathDelivery - 1.0, 0.0)

    local axleBias = index >= 2 and 1.0 or 0.82

    local recovery =
        contactPart * M.params.recoveryContactGain +
        trustPart * M.params.recoveryTrustGain +
        slipDropPart * M.params.recoverySlipDropGain +
        returnPart * M.params.recoveryReturnForceGain +
        loadPart * M.params.recoveryLoadGain +
        carcassPart * M.params.recoveryCarcassGain +
        yawPart * M.params.recoveryYawGain +
        scalarPart * M.params.recoveryScalarGain +
        recoveryBias * M.params.recoveryBiasGain +
        deliveryPart * M.params.loadPathDeliveryGain

    local loss =
        wheelState.thermalMemory * M.params.recoveryHeatLoss +
        wheelState.abrasionMemory * M.params.recoveryAbrasionLoss +
        wheelState.historyMemory * M.params.recoveryHistoryLoss +
        wheelState.memoryRecontactShock * M.params.recoveryShockLoss +
        wheelState.carcassHysteresis * M.params.recoveryHysteresisLoss +
        wheelState.carcassDelay * M.params.recoveryDelayLoss +
        math.max(wheelState.roadShock, wheelState.roadImpact) * M.params.roadShockRecoveryLoss

    recovery = (recovery - loss) * axleBias
    recovery = clamp(recovery * 0.48, 0.0, M.params.maxRecovery)

    local tau = recovery > wheelState.recoveryRate and M.params.tauRecoveryRise or M.params.tauRecoveryFall
    return lowPass(wheelState.recoveryRate, recovery, tau, dt)
end

local function calculateGripReturn(wheelState, recovery)
    local contactPart = clamp(wheelState.contactQuality, 0.0, 1.0)
    local trustPart = clamp(wheelState.contactTrust, 0.0, 1.0)
    local tireGripPart = math.max(wheelState.tireGrip - 1.0, 0.0)
    local integrityPart = clamp(wheelState.loadPathIntegrity, 0.0, 1.0)

    local memoryLoss = clamp(wheelState.slideMemory * M.params.gripReturnMemoryLoss, 0.0, 1.0)
    local delayLoss = clamp(wheelState.carcassDelay * M.params.gripReturnDelayLoss, 0.0, 1.0)
    local heatLoss = clamp(wheelState.thermalMemory * M.params.gripReturnHeatLoss, 0.0, 1.0)
    local abrasionLoss = clamp(wheelState.abrasionMemory * M.params.gripReturnAbrasionLoss, 0.0, 1.0)
    local historyLoss = clamp(wheelState.historyMemory * M.params.gripReturnHistoryLoss, 0.0, 1.0)
    local shockLoss = clamp(wheelState.memoryRecontactShock * M.params.gripReturnShockLoss, 0.0, 1.0)

    local gripReturn =
        M.params.gripReturnBase +
        recovery * M.params.gripReturnRecoveryGain +
        contactPart * M.params.gripReturnContactGain +
        trustPart * M.params.gripReturnTrustGain +
        tireGripPart * M.params.gripReturnTireGripGain +
        integrityPart * 0.08 -
        memoryLoss -
        delayLoss -
        heatLoss -
        abrasionLoss -
        historyLoss -
        shockLoss

    return clamp(gripReturn, 0.0, 1.0)
end

local function calculateBite(wheelState, index)
    local rearBias = index >= 2 and 1.08 or 0.80
    local contactPart = smoothstep(M.params.recontactQuality, 1.0, wheelState.contactQuality)
    local trustPart = smoothstep(M.params.trustGood, 1.0, wheelState.contactTrust)
    local slipDropPart = clamp(wheelState.slipDrop / math.max(M.params.slipRecoverStart, 0.001), 0.0, 1.0)
    local returnPart = clamp(wheelState.carcassReturn + wheelState.recoveryBias * 0.5, 0.0, 1.0)
    local shockPart = clamp(math.max(wheelState.memoryRecontactShock, wheelState.roadShock * 0.4), 0.0, 1.0)

    local bite =
        contactPart * M.params.biteContactGain +
        trustPart * M.params.biteTrustGain +
        slipDropPart * M.params.biteSlipDropGain +
        returnPart * M.params.biteReturnGain +
        shockPart * M.params.biteShockGain

    return clamp(bite * 0.42 * rearBias, 0.0, M.params.maxBite)
end

local function calculateSnapRisk(wheelState, index)
    local rearBias = index >= 2 and M.params.snapRearBias or M.params.snapFrontBias
    local contactPart = smoothstep(M.params.recontactQuality, 1.0, wheelState.contactQuality)
    local trustPart = smoothstep(M.params.trustGood, 1.0, wheelState.contactTrust)
    local slipDropPart = clamp(wheelState.slipDrop / math.max(M.params.slipRecoverStart, 0.001), 0.0, 1.0)
    local memoryPart = clamp(math.max(wheelState.slideMemory, wheelState.historyMemory), 0.0, 1.0)
    local returnPart = clamp(wheelState.carcassReturn, 0.0, 1.0)
    local recoveryBiasPart = clamp(wheelState.recoveryBias, 0.0, 1.0)
    local shockPart = clamp(wheelState.memoryRecontactShock + wheelState.recontactSharpness * 0.35, 0.0, 1.0)
    local yawPart = clamp(M.state.yawOversteer + M.state.yawRecovery + M.state.yawSpinRisk * 0.5, 0.0, 1.5)
    local loadPathPart = clamp(wheelState.loadPathEfficiency + wheelState.loadPathContact, 0.0, 1.4) * 0.5
    local surfacePart = clamp(wheelState.roadSurfaceLimit, 0.0, 1.0)

    local snap =
        contactPart * M.params.snapContactGain +
        trustPart * M.params.snapTrustGain +
        slipDropPart * M.params.snapSlipDropGain +
        memoryPart * M.params.snapMemoryGain +
        returnPart * M.params.snapReturnGain +
        recoveryBiasPart * M.params.snapRecoveryBiasGain +
        shockPart * M.params.snapRecontactShockGain +
        yawPart * M.params.snapYawGain +
        loadPathPart * M.params.snapLoadPathGain +
        wheelState.carcassHysteresis * M.params.snapHysteresisGain +
        surfacePart * M.params.roadSurfaceSnapGain

    snap = snap * 0.29 * rearBias

    if wheelState.slideMemory < 0.08 and wheelState.slip < M.params.slipGood then
        snap = snap * 0.25
    end

    if wheelState.contactTrust < M.params.trustBad then
        snap = snap * 0.65
    end

    return clamp(snap, 0.0, M.params.maxSnapRisk)
end

local function calculateRecontact(wheelState, dt)
    local contactRise = wheelState.contactRise
    local byQuality = smoothstep(M.params.recontactQuality, 1.0, wheelState.contactQuality)
    local byTrust = smoothstep(M.params.trustGood, 1.0, wheelState.contactTrust)
    local byRate = smoothstep(M.params.recontactRate, M.params.recontactRate * 2.2, contactRise)
    local byMemory = clamp(wheelState.slideMemory, 0.0, 1.0)
    local byTireMemory = clamp(wheelState.tireRecontact, 0.0, 1.0)
    local byShock = clamp(math.max(wheelState.memoryRecontactShock, wheelState.roadShock * 0.35), 0.0, 1.0)

    local target = clamp(
        byQuality * 0.26 +
        byTrust * 0.20 +
        byRate * 0.28 +
        byMemory * 0.18 +
        byTireMemory * 0.34 +
        byShock * 0.32,
        0.0,
        1.0
    )

    return lowPass(wheelState.recontactSharpness, target, M.params.tauRecontact, dt)
end

local function calculateDirtyReturn(wheelState)
    local dirty =
        wheelState.thermalMemory * 0.28 +
        wheelState.abrasionMemory * 0.24 +
        wheelState.historyMemory * 0.30 +
        wheelState.memoryRecontactShock * 0.32 +
        wheelState.carcassHysteresis * 0.18 +
        math.max(0.0, 1.0 - wheelState.carcassGripGate) * 0.20 +
        wheelState.roadShock * 0.08

    return clamp(dirty, 0.0, 1.0)
end

local function determinePhase(wheelState, saturated)
    if wheelState.load < M.params.loadLow or wheelState.contactQuality < 0.08 then
        return PHASE.UNLOADED
    end

    if wheelState.snapRisk > 0.62 then
        return PHASE.SNAP_RISK
    end

    if wheelState.thermalMemory > 0.66 and wheelState.gripReturn < 0.46 then
        return PHASE.HEAT_SOAK
    end

    if wheelState.dirtyReturn > 0.58 and wheelState.recoveryRate > 0.20 then
        return PHASE.DIRTY_RETURN
    end

    if wheelState.recontactSharpness > 0.48 then
        return PHASE.RECONTACT
    end

    if saturated > 0.45 or wheelState.slip > M.params.slipSaturated then
        return PHASE.SATURATED
    end

    if wheelState.recoveryRate > 0.50 and wheelState.slideMemory > 0.16 then
        return PHASE.RECOVERING
    end

    if wheelState.slipDrop > 0.10 and wheelState.slideMemory > 0.10 then
        return PHASE.EARLY_RECOVERY
    end

    if wheelState.slideIntensity > 0.35 or wheelState.slip > M.params.slipRecoverStart then
        return PHASE.SLIDING
    end

    return PHASE.STABLE
end

local function updateWheel(index, wheelState, dt)
    local slideIntensity, saturated = calculateSlideIntensity(wheelState, index)
    wheelState.slideIntensity = slideIntensity

    local decay =
        M.params.slideDecayBase +
        wheelState.contactQuality * M.params.slideDecayContactGain +
        wheelState.contactTrust * M.params.slideDecayTrustGain +
        wheelState.gripReturn * M.params.slideDecayGripReturnGain -
        wheelState.thermalMemory * M.params.slideDecayHeatLoss -
        wheelState.abrasionMemory * M.params.slideDecayAbrasionLoss -
        wheelState.historyMemory * M.params.slideDecayHistoryLoss

    decay = clamp(decay, 0.05, 1.25)

    local memoryTarget = clamp(
        wheelState.slideMemory +
        slideIntensity * dt * 1.8 -
        decay * dt * 0.55,
        0.0,
        M.params.maxSlideMemory
    )

    wheelState.slideMemory = lowPass(
        wheelState.slideMemory,
        memoryTarget,
        M.params.tauSlideMemory,
        dt
    )

    wheelState.recoveryRate = calculateRecovery(wheelState, dt, index)

    local gripReturnTarget = calculateGripReturn(wheelState, wheelState.recoveryRate)
    wheelState.gripReturn = lowPass(
        wheelState.gripReturn,
        gripReturnTarget,
        M.params.tauGripReturn,
        dt
    )

    local biteTarget = calculateBite(wheelState, index)
    wheelState.bite = lowPass(wheelState.bite, biteTarget, M.params.tauBite, dt)

    wheelState.recontactSharpness = calculateRecontact(wheelState, dt)

    local snapTarget = calculateSnapRisk(wheelState, index)
    local snapTau = snapTarget > wheelState.snapRisk and M.params.tauSnapRise or M.params.tauSnapFall
    wheelState.snapRisk = lowPass(wheelState.snapRisk, snapTarget, snapTau, dt)

    wheelState.dirtyReturn = calculateDirtyReturn(wheelState)

    wheelState.phaseId = determinePhase(wheelState, saturated)
    wheelState.phase = PHASE_TEXT[wheelState.phaseId] or "UNKNOWN"
end

local function clearWheel(index, dt)
    local wheelState = M.state.wheels[index]

    wheelState.active = false
    wheelState.rawSlip = 0.0
    wheelState.slip = lowPass(wheelState.slip, 0.0, M.params.tauSlip, dt)
    wheelState.slipRate = 0.0
    wheelState.slipDrop = 0.0

    wheelState.contactQuality = lowPass(wheelState.contactQuality, 0.0, M.params.tauContact, dt)
    wheelState.contactTrust = lowPass(wheelState.contactTrust, 0.0, M.params.tauContact, dt)
    wheelState.contactLoss = 1.0
    wheelState.load = lowPass(wheelState.load, 0.0, M.params.tauLoad, dt)
    wheelState.loadFactor = 0.0

    wheelState.slideIntensity = 0.0
    wheelState.recoveryRate = lowPass(wheelState.recoveryRate, 0.0, M.params.tauRecoveryFall, dt)
    wheelState.gripReturn = lowPass(wheelState.gripReturn, 1.0, M.params.tauGripReturn, dt)
    wheelState.snapRisk = lowPass(wheelState.snapRisk, 0.0, M.params.tauSnapFall, dt)
    wheelState.recontactSharpness = lowPass(wheelState.recontactSharpness, 0.0, M.params.tauRecontact, dt)
    wheelState.bite = lowPass(wheelState.bite, 0.0, M.params.tauBite, dt)
    wheelState.dirtyReturn = lowPass(wheelState.dirtyReturn, 0.0, M.params.tauBite, dt)

    wheelState.phaseId = PHASE.UNLOADED
    wheelState.phase = "UNLOADED"
end

local function clearAllWheels(dt)
    for i = 0, 3 do
        clearWheel(i, dt)
    end
end

--============================================================
-- Export
--============================================================

local function exportWheel(index, wheelState)
    safeStore("ngp_slip_recovery_rate_" .. index, wheelState.recoveryRate or 0.0)
    safeStore("ngp_slip_grip_return_" .. index, wheelState.gripReturn or 1.0)
    safeStore("ngp_slip_snap_risk_" .. index, wheelState.snapRisk or 0.0)
    safeStore("ngp_slip_slide_memory_" .. index, wheelState.slideMemory or 0.0)
    safeStore("ngp_slip_recontact_" .. index, wheelState.recontactSharpness or 0.0)
    safeStore("ngp_slip_recovery_phase_id_" .. index, wheelState.phaseId or 0)
    safeStore("ngp_slip_recovery_phase_" .. index, wheelState.phase or "UNKNOWN")

    safeStore("ngp_recovery_rate_" .. index, wheelState.recoveryRate or 0.0)
    safeStore("ngp_grip_return_" .. index, wheelState.gripReturn or 1.0)
    safeStore("ngp_snap_risk_" .. index, wheelState.snapRisk or 0.0)
    safeStore("ngp_slide_memory_" .. index, wheelState.slideMemory or 0.0)
    safeStore("ngp_recontact_sharpness_" .. index, wheelState.recontactSharpness or 0.0)

    safeStore("ngp_slip_bite_" .. index, wheelState.bite or 0.0)
    safeStore("ngp_slip_dirty_return_" .. index, wheelState.dirtyReturn or 0.0)
    safeStore("ngp_slip_contact_trust_" .. index, wheelState.contactTrust or 1.0)
    safeStore("ngp_slip_thermal_memory_" .. index, wheelState.thermalMemory or 0.0)
    safeStore("ngp_slip_abrasion_memory_" .. index, wheelState.abrasionMemory or 0.0)
    safeStore("ngp_slip_history_memory_" .. index, wheelState.historyMemory or 0.0)
    safeStore("ngp_slip_recovery_scalar_" .. index, wheelState.memoryRecoveryScalar or 1.0)
    safeStore("ngp_slip_recontact_shock_" .. index, wheelState.memoryRecontactShock or 0.0)

    safeStore("ngp_recovery_bite_" .. index, wheelState.bite or 0.0)
    safeStore("ngp_dirty_return_" .. index, wheelState.dirtyReturn or 0.0)
    safeStore("ngp_recovery_trust_" .. index, wheelState.contactTrust or 1.0)

    safeStore("ngp_sr_rate_" .. index, wheelState.recoveryRate or 0.0)
    safeStore("ngp_sr_grip_" .. index, wheelState.gripReturn or 1.0)
    safeStore("ngp_sr_snap_" .. index, wheelState.snapRisk or 0.0)
    safeStore("ngp_sr_memory_" .. index, wheelState.slideMemory or 0.0)
    safeStore("ngp_sr_recontact_" .. index, wheelState.recontactSharpness or 0.0)
    safeStore("ngp_sr_bite_" .. index, wheelState.bite or 0.0)
    safeStore("ngp_sr_dirty_" .. index, wheelState.dirtyReturn or 0.0)

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_slip_recovery_slip_" .. index, wheelState.slip or 0.0)
    safeStore("ngp_slip_recovery_slip_rate_" .. index, wheelState.slipRate or 0.0)
    safeStore("ngp_slip_recovery_slip_drop_" .. index, wheelState.slipDrop or 0.0)
    safeStore("ngp_slip_recovery_contact_" .. index, wheelState.contactQuality or 0.0)
    safeStore("ngp_slip_recovery_contact_rate_" .. index, wheelState.contactRate or 0.0)
    safeStore("ngp_slip_recovery_load_" .. index, wheelState.load or 0.0)
    safeStore("ngp_slip_recovery_carcass_return_" .. index, wheelState.carcassReturn or 0.0)
    safeStore("ngp_slip_recovery_carcass_delay_" .. index, wheelState.carcassDelay or 0.0)
    safeStore("ngp_slip_recovery_carcass_hysteresis_" .. index, wheelState.carcassHysteresis or 0.0)
    safeStore("ngp_slip_recovery_load_path_loss_" .. index, wheelState.loadPathLoss or 0.0)
    safeStore("ngp_slip_recovery_grip_gate_" .. index, wheelState.carcassGripGate or 1.0)
    safeStore("ngp_slip_recovery_recovery_bias_" .. index, wheelState.recoveryBias or 0.0)

    safeStore("ngp_slip_recovery_road_shock_" .. index, wheelState.roadShock or 0.0)
    safeStore("ngp_slip_recovery_road_surface_" .. index, wheelState.roadSurfaceLimit or 0.0)
end

local function exportGlobal()
    safeStore("ngp_slip_recovery_status", M.state.status or "UNKNOWN")
    safeStore("ngp_slip_recovery_update_count", M.state.updateCount or 0)
    safeStore("ngp_slip_recovery_wheels_valid", M.state.wheelsValid and 1 or 0)

    safeStore("ngp_slip_recovery_avg", M.state.avgRecovery or 0.0)
    safeStore("ngp_slip_recovery_avg_grip_return", M.state.avgGripReturn or 1.0)
    safeStore("ngp_slip_recovery_avg_snap_risk", M.state.avgSnapRisk or 0.0)
    safeStore("ngp_slip_recovery_avg_slide_memory", M.state.avgSlideMemory or 0.0)
    safeStore("ngp_slip_recovery_avg_recontact", M.state.avgRecontact or 0.0)
    safeStore("ngp_slip_recovery_avg_bite", M.state.avgBite or 0.0)
    safeStore("ngp_slip_recovery_avg_dirty_return", M.state.avgDirtyReturn or 0.0)

    safeStore("ngp_recovery_avg", M.state.avgRecovery or 0.0)
    safeStore("ngp_snap_risk_avg", M.state.avgSnapRisk or 0.0)
    safeStore("ngp_slide_memory_avg", M.state.avgSlideMemory or 0.0)
    safeStore("ngp_recovery_bite_avg", M.state.avgBite or 0.0)
    safeStore("ngp_dirty_return_avg", M.state.avgDirtyReturn or 0.0)

    safeStore("ngp_sr_avg_recovery", M.state.avgRecovery or 0.0)
    safeStore("ngp_sr_avg_grip", M.state.avgGripReturn or 1.0)
    safeStore("ngp_sr_avg_snap", M.state.avgSnapRisk or 0.0)
    safeStore("ngp_sr_avg_memory", M.state.avgSlideMemory or 0.0)
    safeStore("ngp_sr_rear_recovery", M.state.rearRecovery or 0.0)
    safeStore("ngp_sr_rear_snap", M.state.rearSnapRisk or 0.0)
    safeStore("ngp_sr_max_recovery", M.state.maxRecovery or 0.0)
    safeStore("ngp_sr_max_snap", M.state.maxSnapRisk or 0.0)
    safeStore("ngp_sr_max_memory", M.state.maxSlideMemory or 0.0)

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_slip_recovery_contact_linked", M.state.contactLinked and 1 or 0)
    safeStore("ngp_slip_recovery_memory_linked", M.state.memoryLinked and 1 or 0)
    safeStore("ngp_slip_recovery_carcass_linked", M.state.carcassLinked and 1 or 0)
    safeStore("ngp_slip_recovery_yaw_linked", M.state.yawLinked and 1 or 0)
    safeStore("ngp_slip_recovery_load_path_linked", M.state.loadPathLinked and 1 or 0)
    safeStore("ngp_slip_recovery_road_linked", M.state.roadLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i, M.state.wheels[i])
    end
    exportGlobal()
end

--============================================================
-- Public API
--============================================================

function M.init()
    M.state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    M.state.updateCount = (M.state.updateCount or 0) + 1

    dt = clamp(
        safeNumber(dt, M.params.minDt),
        M.params.minDt,
        M.params.maxDt
    )

    updateDebugGate(dt)

    car = car or getCarSafe()

    M.state.contactLinked = false
    M.state.memoryLinked = false
    M.state.carcassLinked = false
    M.state.yawLinked = false
    M.state.loadPathLinked = false
    M.state.roadLinked = false

    readYawBudget()

    if not car then
        M.state.status = "NO CAR"
        M.state.wheelsValid = false
        clearAllWheels(dt)
        exportState()
        return
    end

    if not hasWheels(car) then
        M.state.status = "NO WHEELS"
        M.state.wheelsValid = false
        clearAllWheels(dt)
        exportState()
        return
    end

    M.state.status = "RUNNING"
    M.state.wheelsValid = true

    local sumRecovery = 0.0
    local sumGripReturn = 0.0
    local sumSnapRisk = 0.0
    local sumSlideMemory = 0.0
    local sumRecontact = 0.0
    local sumBite = 0.0
    local sumDirtyReturn = 0.0

    local maxRecovery = 0.0
    local maxSnap = 0.0
    local maxMemory = 0.0

    for i = 0, 3 do
        local wheel = getWheel(i, car)
        local wheelState = M.state.wheels[i]

        if wheel then
            wheelState.active = true

            local rawSlip, slipAngle, slipRatio = readSlip(i, wheel)
            wheelState.rawSlip = rawSlip
            wheelState.slipAngle = slipAngle
            wheelState.slipRatio = slipRatio

            local prevSlip = wheelState.slip or rawSlip
            wheelState.prevSlip = prevSlip
            wheelState.slip = lowPass(wheelState.slip, rawSlip, M.params.tauSlip, dt)
            wheelState.slipRate = (wheelState.slip - prevSlip) / math.max(dt, 0.001)
            wheelState.slipDrop = clamp(-wheelState.slipRate * dt, 0.0, 3.0)

            local prevContact = wheelState.contactQuality or 1.0
            wheelState.prevContactQuality = prevContact
            readContact(i, wheelState, dt)
            wheelState.contactRate = (wheelState.contactQuality - prevContact) / math.max(dt, 0.001)
            wheelState.contactRise = math.max(0.0, wheelState.contactRate)

            local rawLoad = math.abs(readLoad(i, wheel))
            wheelState.load = lowPass(wheelState.load, rawLoad, M.params.tauLoad, dt)
            wheelState.loadFactor = clamp(wheelState.load / math.max(M.params.loadReference, 1.0), 0.0, 2.0)

            readTireMemory(i, wheelState)
            readCarcass(i, wheelState)
            readLoadPath(i, wheelState)
            readRoadInput(i, wheelState)

            updateWheel(i, wheelState, dt)
        else
            clearWheel(i, dt)
        end

        sumRecovery = sumRecovery + (wheelState.recoveryRate or 0.0)
        sumGripReturn = sumGripReturn + (wheelState.gripReturn or 1.0)
        sumSnapRisk = sumSnapRisk + (wheelState.snapRisk or 0.0)
        sumSlideMemory = sumSlideMemory + (wheelState.slideMemory or 0.0)
        sumRecontact = sumRecontact + (wheelState.recontactSharpness or 0.0)
        sumBite = sumBite + (wheelState.bite or 0.0)
        sumDirtyReturn = sumDirtyReturn + (wheelState.dirtyReturn or 0.0)

        maxRecovery = math.max(maxRecovery, wheelState.recoveryRate or 0.0)
        maxSnap = math.max(maxSnap, wheelState.snapRisk or 0.0)
        maxMemory = math.max(maxMemory, wheelState.slideMemory or 0.0)

        exportWheel(i, wheelState)
    end

    M.state.avgRecovery = sumRecovery * 0.25
    M.state.avgGripReturn = sumGripReturn * 0.25
    M.state.avgSnapRisk = sumSnapRisk * 0.25
    M.state.avgSlideMemory = sumSlideMemory * 0.25
    M.state.avgRecontact = sumRecontact * 0.25
    M.state.avgBite = sumBite * 0.25
    M.state.avgDirtyReturn = sumDirtyReturn * 0.25

    M.state.maxRecovery = maxRecovery
    M.state.maxSnapRisk = maxSnap
    M.state.maxSlideMemory = maxMemory
    M.state.rearRecovery = ((M.state.wheels[2].recoveryRate or 0.0) + (M.state.wheels[3].recoveryRate or 0.0)) * 0.5
    M.state.rearSnapRisk = ((M.state.wheels[2].snapRisk or 0.0) + (M.state.wheels[3].snapRisk or 0.0)) * 0.5

    exportGlobal()
end

function M.getRecovery(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.recoveryRate or 0.0
end

function M.getGripReturn(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.gripReturn or 1.0
end

function M.getSnapRisk(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.snapRisk or 0.0
end

function M.getBite(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.bite or 0.0
end

function M.getDirtyReturn(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.dirtyReturn or 0.0
end

function M.getSlideMemory(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.slideMemory or 0.0
end

function M.getRecontact(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.recontactSharpness or 0.0
end

function M.getPhase(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.phase or "UNKNOWN"
end

function M.getState(index)
    if index == nil then
        return M.state
    end

    return M.state.wheels[index]
end

function M.debugStr(index)
    local wheelState = M.state.wheels[index or 0] or M.state.wheels[0]

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "%s Rec %.3f GripRet %.3f Snap %.3f\n" ..
        "SlideMem %.3f ReC %.3f Bite %.3f Dirty %.3f\n" ..
        "Slip %.3f Drop %.3f Trust %.2f CQ %.2f Load %.0f\n" ..
        "Heat %.2f Abr %.2f Hist %.2f Shock %.2f\n" ..
        "Ret %.2f Delay %.2f Hyst %.2f Bias %.2f\n" ..
        "Links CQ:%s TM:%s TC:%s YMB:%s LP:%s RII:%s",

        tostring(M.state.status),
        M.state.updateCount or 0,
        M.state.wheelsValid and "OK" or "NIL",

        tostring(wheelState.phase or "UNKNOWN"),
        wheelState.recoveryRate or 0.0,
        wheelState.gripReturn or 1.0,
        wheelState.snapRisk or 0.0,

        wheelState.slideMemory or 0.0,
        wheelState.recontactSharpness or 0.0,
        wheelState.bite or 0.0,
        wheelState.dirtyReturn or 0.0,

        wheelState.slip or 0.0,
        wheelState.slipDrop or 0.0,
        wheelState.contactTrust or 1.0,
        wheelState.contactQuality or 0.0,
        wheelState.load or 0.0,

        wheelState.thermalMemory or 0.0,
        wheelState.abrasionMemory or 0.0,
        wheelState.historyMemory or 0.0,
        wheelState.memoryRecontactShock or 0.0,

        wheelState.carcassReturn or 0.0,
        wheelState.carcassDelay or 0.0,
        wheelState.carcassHysteresis or 0.0,
        wheelState.recoveryBias or 0.0,

        M.state.contactLinked and "OK" or "NIL",
        M.state.memoryLinked and "OK" or "NIL",
        M.state.carcassLinked and "OK" or "NIL",
        M.state.yawLinked and "OK" or "NIL",
        M.state.loadPathLinked and "OK" or "NIL",
        M.state.roadLinked and "OK" or "NIL"
    )
end

return M
