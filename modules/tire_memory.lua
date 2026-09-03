---@diagnostic disable: undefined-global

--============================================================
-- tire_memory.lua
-- ACNextGen V1.1.5 Stable
-- Tire Memory / Rubber History / Grip Recovery Bridge
--============================================================

local M = {}

local WHEEL_NAMES = {
    [0] = "FL",
    [1] = "FR",
    [2] = "RL",
    [3] = "RR",
}

local PHASE_ID = {
    INIT = 0,
    GRIP = 1,
    LOADING = 2,
    SLIPPING = 3,
    RECOVERING = 4,
    SATURATED = 5,
    UNLOADED = 6,
    RECONTACT = 7,
    HOT = 8,
    HISTORY = 9,
    STORE_ONLY = 10,
}

M.params = {
    loadReference = 4000.0,
    loadMin = 120.0,
    loadWarmStart = 1800.0,
    loadWarmFull = 6200.0,

    slipAngleRef = 0.22,
    slipRatioRef = 0.35,

    ambientTemp = 25.0,
    optimumTemp = 90.0,
    warmRecoveryTemp = 75.0,
    hotRecoveryTemp = 105.0,
    overheatTemp = 125.0,

    slipTau = 0.16,
    loadTau = 0.24,
    contactTau = 0.070,
    heatTau = 0.45,
    memoryTau = 0.22,
    gripTau = 0.12,

    buildTau = 0.060,
    releaseTau = 0.420,
    recontactTau = 0.090,

    slipAngleBuildGain = 0.46,
    slipRatioBuildGain = 0.34,
    combinedSlipBuildGain = 0.18,
    loadBuildGain = 0.12,

    contactLossBuildGain = 0.26,
    contactTrustBuildGain = 0.20,
    contactUnstableBuildGain = 0.16,
    contactDelayBuildGain = 0.16,

    carcassHistoryBuildGain = 0.30,
    carcassHeatBuildGain = 0.22,
    carcassHysteresisBuildGain = 0.18,
    carcassVerticalBuildGain = 0.10,
    bristleBuildGain = 0.12,

    tireLimitBuildGain = 0.10,
    hopBuildGain = 0.14,
    armCamberBuildGain = 0.030,
    armToeBuildGain = 0.045,
    driveRearBuildGain = 0.08,
    lsdRearBuildGain = 0.06,
    shaftRearBuildGain = 0.05,

    roadShockBuildGain = 0.10,
    pathLossBuildGain = 0.10,
    forceSlipEnergyBuildGain = 0.14,
    slipRecoveryRiskGain = 0.10,
    thermalStressBuildGain = 0.12,

    microHeatGain = 0.22,
    macroHeatGain = 0.14,
    dampingHeatGain = 0.18,
    thermalBuildGain = 0.20,
    thermalReleaseGain = 0.055,

    abrasionBuildGain = 0.18,
    abrasionReleaseGain = 0.018,
    historyReleaseGain = 0.060,
    historyStickiness = 0.72,

    recoveryGain = 0.20,
    contactQualityRecoveryGain = 0.20,
    supportRecoveryGain = 0.18,
    gripGateRecoveryGain = 0.14,
    recoveryBiasGain = 0.22,

    slipAngleRecoveryGain = 0.56,
    slipRatioRecoveryGain = 0.74,
    contactLossRecoveryLoss = 0.16,
    hotRecoveryLoss = 0.38,
    abrasionRecoveryLoss = 0.22,
    dirtyReturnRecoveryLoss = 0.12,
    roadShockRecoveryLoss = 0.08,

    normalTempRecovery = 1.00,
    warmTempRecovery = 0.76,
    hotTempRecovery = 0.42,
    overheatRecovery = 0.24,

    memoryGripLoss = 0.30,
    thermalGripLoss = 0.12,
    abrasionGripLoss = 0.18,
    contactGripLoss = 0.08,
    recontactGripBonus = 0.03,
    forceGripLoss = 0.04,
    roadGripLoss = 0.05,

    minGrip = 0.68,
    maxGrip = 1.05,

    unloadedLoadFactor = 0.10,
    saturatedCombinedSlip = 1.20,
    slippingCombinedSlip = 0.70,
    memoryRecoveringMin = 0.055,
    loadingBuildMargin = 0.012,

    recontactQualityLow = 0.34,
    recontactQualityHigh = 0.58,
    recontactRate = 2.0,
    recontactMemoryMin = 0.08,

    hotMemoryThreshold = 0.36,
    historyThreshold = 0.42,

    maxMemory = 1.0,
    maxThermalMemory = 1.0,
    maxAbrasionMemory = 1.0,
    maxHeat = 1.0,

    minDt = 0.0001,
    maxDt = 0.100,
    debugStoreInterval = 0.25,
}

M.state = {
    status = "INIT",
    updateCount = 0,
    wheelsValid = false,
    storeOnly = false,

    avgMemory = 0.0,
    avgGrip = 1.0,
    avgBuild = 0.0,
    avgRecovery = 0.0,
    avgThermal = 0.0,
    avgAbrasion = 0.0,
    avgHeat = 0.0,
    avgHistory = 0.0,
    avgRecoveryScalar = 1.0,
    maxMemory = 0.0,
    minGrip = 1.0,
    activeCount = 0,

    contactLinked = false,
    carcassLinked = false,
    dynamicsLinked = false,
    hopLinked = false,
    armLinked = false,
    drivetrainLinked = false,
    lsdLinked = false,
    loadLinked = false,
    forceLinked = false,
    roadLinked = false,
    thermalLinked = false,
    recoveryLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,

    wheels = {},
}

for i = 0, 3 do
    M.state.wheels[i] = {
        active = false,
        name = WHEEL_NAMES[i],

        slipAngle = 0.0,
        slipRatio = 0.0,
        rawSlipAngle = 0.0,
        rawSlipRatio = 0.0,

        load = M.params.loadReference,
        rawLoad = M.params.loadReference,
        loadRate = 0.0,
        prevLoad = M.params.loadReference,

        memory = 0.0,
        thermalMemory = 0.0,
        abrasionMemory = 0.0,
        historyMemory = 0.0,
        grip = 1.0,

        build = 0.0,
        recovery = 0.0,
        recoveryScalar = 1.0,

        tyreTemp = M.params.ambientTemp,
        virtualTemp = M.params.ambientTemp,
        tempRecovery = 1.0,
        slipRecovery = 1.0,

        microHeat = 0.0,
        macroHeat = 0.0,
        heatSeed = 0.0,
        historyStress = 0.0,

        contactQuality = 1.0,
        prevContactQuality = 1.0,
        contactQualityRate = 0.0,
        contactRaw = 1.0,
        contactTrust = 1.0,
        contactCombinedSlip = 0.0,
        contactStabilityScore = 1.0,
        contactStatus = 0,
        contactLoss = 0.0,
        contactEnergy = 0.0,

        carcassSupport = 1.0,
        carcassGripGate = 1.0,
        carcassVerticalNorm = 0.0,
        carcassHysteresis = 0.0,
        carcassDelay = 0.0,
        carcassRecoveryBias = 0.0,
        carcassBristleLat = 0.0,
        carcassBristleLong = 0.0,
        sidewallEnergy = 0.0,
        deformation = 0.0,

        tireLimit = 0.0,
        tireGrip = 1.0,
        forceSlipEnergy = 0.0,
        forceCombinedSlip = 0.0,
        hopEnergy = 0.0,

        armCamber = 0.0,
        armToe = 0.0,
        driveTorque = 0.0,
        lsdLock = 0.0,
        shaftTwist = 0.0,

        roadShock = 0.0,
        roadSeverity = 0.0,
        pathLoss = 0.0,
        thermalStress = 0.0,
        slipRecoveryRisk = 0.0,
        dirtyReturn = 0.0,

        phaseId = 0,
        phaseText = "INIT",
        recontact = 0.0,
        recontactShock = 0.0,
    }

    M.state[i] = M.state.wheels[i]
end

M.debug = M.state

local function safeNumber(value, defaultValue)
    local n = tonumber(value)
    if n == nil or n ~= n or n == math.huge or n == -math.huge then
        if defaultValue == nil then return 0.0 end
        return defaultValue
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

local function smoothstep(edge0, edge1, x)
    if edge0 == edge1 then
        return x >= edge1 and 1.0 or 0.0
    end
    local t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)
end

local function lerp(a, b, t)
    return a + (b - a) * clamp(t, 0.0, 1.0)
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.0)
    dt = safeNumber(dt, 0.0)

    if tau <= 0.0 then
        return target
    end

    return current + (target - current) * clamp(dt / (tau + dt), 0.0, 1.0)
end

local function safeStore(key, value)
    if not ac or not ac.store then return end
    pcall(function() ac.store(key, value) end)
end

local function safeLoadRaw(key)
    if not ac or not ac.load then return nil end
    local ok, value = pcall(function() return ac.load(key) end)
    if not ok then return nil end
    return value
end

local function safeLoad(key, defaultValue)
    local value = safeLoadRaw(key)
    if value == nil then
        if defaultValue == nil then return nil end
        return defaultValue
    end
    return safeNumber(value, defaultValue)
end

local function firstRaw(keys)
    for n = 1, #keys do
        local v = safeLoadRaw(keys[n])
        if v ~= nil then return v, keys[n] end
    end
    return nil, nil
end

local function safeField(obj, field, defaultValue)
    if not obj then return defaultValue end
    local ok, value = pcall(function() return obj[field] end)
    if not ok or value == nil then return defaultValue end
    return value
end

local function safeGetCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function() return ac.getCar(0) end)
    if not ok then return nil end
    return car
end

local function safeGetWheels(car)
    if not car then return nil end
    local ok, wheels = pcall(function() return car.wheels end)
    if not ok then return nil end
    return wheels
end

local function getWheel(wheels, index)
    if not wheels then return nil end
    local ok, wheel = pcall(function()
        return wheels[index] or wheels[index + 1]
    end)
    if not ok then return nil end
    return wheel
end

local function setPhase(ws, name)
    ws.phaseText = name or "INIT"
    ws.phaseId = PHASE_ID[ws.phaseText] or 0
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

local function getWheelLoad(wheel, index)
    if wheel then
        local fields = { "load", "loadK" }
        for n = 1, #fields do
            local load = safeField(wheel, fields[n], nil)
            if load ~= nil then
                return abs(load), "WHEEL"
            end
        end
    end

    local direct, source = firstRaw({
        "ngp_contact_load_" .. index,
        "ngp_tire_carcass_load_" .. index,
        "ngp_tdyn_load_" .. index,
        "ngp_tire_force_load_" .. index,
        "ngp_tire_load_" .. index,
        "ngp_tire_load_input_" .. index,
        "ngp_sprung_load_" .. index,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index,
        "ngp_load_path_load_" .. index,
        "ngp_lp_load_" .. index,
    })

    if direct ~= nil then
        M.state.loadLinked = true
        return abs(direct), source or "STORE"
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        M.state.loadLinked = true
        return math.max(0.0, M.params.loadReference + safeNumber(dlt, 0.0)), "DLT"
    end

    return M.params.loadReference, "DEFAULT"
end

local function getTyreTemp(index)
    local temp = firstRaw({
        "ngp_tyre_temp_" .. index,
        "ngp_tire_temp_" .. index,
        "ngp_brake_temp_read_" .. index,
        "ngp_memory_temp_" .. index,
    })

    if temp ~= nil then
        M.state.thermalLinked = true
        return safeNumber(temp, M.params.ambientTemp)
    end

    return M.params.ambientTemp
end

local function readSlip(index, wheel)
    local slipAngle = firstRaw({
        "ngp_contact_slip_angle_" .. index,
        "ngp_tire_carcass_slip_angle_" .. index,
        "ngp_tire_force_effective_slip_angle_" .. index,
        "ngp_tdyn_effective_slip_angle_" .. index,
        "ngp_tire_slip_angle_" .. index,
        "ngp_slip_angle_" .. index,
        "ngp_filtered_slip_angle_" .. index,
    })

    local slipRatio = firstRaw({
        "ngp_contact_slip_ratio_" .. index,
        "ngp_tire_carcass_slip_ratio_" .. index,
        "ngp_tire_force_effective_slip_ratio_" .. index,
        "ngp_tdyn_effective_slip_ratio_" .. index,
        "ngp_tire_slip_ratio_" .. index,
        "ngp_slip_ratio_" .. index,
        "ngp_filtered_slip_ratio_" .. index,
    })

    if slipAngle == nil and wheel then
        slipAngle = safeField(wheel, "slipAngle", 0.0)
    end

    if slipRatio == nil and wheel then
        slipRatio = safeField(wheel, "slipRatio", 0.0)
    end

    return abs(slipAngle), abs(slipRatio)
end

local function hasInput(index, wheel)
    if wheel then return true end

    local keys = {
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tire_carcass_load_" .. index,
        "ngp_tire_force_load_" .. index,
        "ngp_tire_slip_angle_" .. index,
        "ngp_tire_slip_ratio_" .. index,
        "ngp_dlt_load_" .. index,
        "ngp_carcass_support_" .. index,
        "ngp_tire_hop_energy_" .. index,
        "ngp_tire_grip_" .. index,
        "ngp_tire_limit_" .. index,
    }

    for n = 1, #keys do
        if safeLoadRaw(keys[n]) ~= nil then
            return true
        end
    end

    return false
end

local function readContactInputs(index, ws)
    local cq = firstRaw({
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index,
        "ngp_tc_contact_" .. index,
    })

    local cr = firstRaw({
        "ngp_contact_raw_" .. index,
        "ngp_tcr_quality_" .. index,
    })

    local trust = firstRaw({
        "ngp_contact_trust_" .. index,
        "ngp_contact_stability_score_" .. index,
        "ngp_tcr_quality_" .. index,
    })

    local combined = firstRaw({
        "ngp_contact_combined_slip_" .. index,
        "ngp_tire_force_combined_slip_" .. index,
        "ngp_tdyn_combined_slip_" .. index,
        "ngp_tire_dynamics_combined_slip_" .. index,
        "ngp_tire_combined_slip_" .. index,
    })

    local stab = firstRaw({
        "ngp_contact_stability_score_" .. index,
        "ngp_contact_trust_" .. index,
    })

    local status = firstRaw({
        "ngp_contact_status_" .. index,
    })

    local contactLoss = firstRaw({
        "ngp_tire_contact_loss_" .. index,
        "ngp_tcr_contact_loss_" .. index,
        "ngp_contact_loss_" .. index,
        "ngp_contact_hop_drop_" .. index,
    })

    local contactEnergy = firstRaw({
        "ngp_tcr_energy_push_" .. index,
        "ngp_tc_energy_" .. index,
        "ngp_tire_slip_energy_" .. index,
        "ngp_tdyn_slip_energy_" .. index,
        "ngp_tire_force_slip_energy_" .. index,
    })

    if cq ~= nil or cr ~= nil or trust ~= nil or combined ~= nil or stab ~= nil or status ~= nil or contactLoss ~= nil or contactEnergy ~= nil then
        M.state.contactLinked = true
    end

    local targetQuality = clamp(safeNumber(cq, 1.0), 0.0, 1.2)
    ws.contactRaw = clamp(safeNumber(cr, targetQuality), 0.0, 1.2)
    ws.contactTrust = clamp(safeNumber(trust, targetQuality), 0.0, 1.2)
    ws.contactCombinedSlip = clamp(safeNumber(combined, 0.0), 0.0, 2.5)
    ws.contactStabilityScore = clamp(safeNumber(stab, ws.contactTrust), 0.0, 1.2)
    ws.contactStatus = safeNumber(status, 0.0)
    ws.contactLoss = clamp(safeNumber(contactLoss, 1.0 - targetQuality), 0.0, 1.0)
    ws.contactEnergy = clamp(safeNumber(contactEnergy, 0.0), 0.0, 1.0)

    return targetQuality
end

local function readCarcassInputs(index, ws)
    local support = firstRaw({
        "ngp_carcass_support_" .. index,
        "ngp_tire_carcass_support_" .. index,
        "ngp_contact_carcass_support_" .. index,
    })

    local gripGate = firstRaw({
        "ngp_carcass_grip_gate_" .. index,
        "ngp_tire_carcass_grip_gate_" .. index,
        "ngp_contact_grip_gate_" .. index,
    })

    local vertical = firstRaw({
        "ngp_carcass_vertical_norm_" .. index,
        "ngp_tire_carcass_vertical_norm_" .. index,
        "ngp_contact_vertical_norm_" .. index,
    })

    local hyst = firstRaw({
        "ngp_carcass_hysteresis_" .. index,
        "ngp_tire_carcass_hysteresis_" .. index,
        "ngp_contact_hysteresis_" .. index,
    })

    local delay = firstRaw({
        "ngp_contact_delay_" .. index,
        "ngp_tire_contact_delay_" .. index,
        "ngp_contact_quality_delay_" .. index,
    })

    local recoveryBias = firstRaw({
        "ngp_carcass_recovery_bias_" .. index,
        "ngp_tire_carcass_recovery_bias_" .. index,
        "ngp_contact_recovery_bias_" .. index,
    })

    local heatSeed = firstRaw({
        "ngp_carcass_heat_seed_" .. index,
        "ngp_tire_carcass_heat_seed_" .. index,
        "ngp_contact_heat_seed_" .. index,
    })

    local historyStress = firstRaw({
        "ngp_carcass_history_stress_" .. index,
        "ngp_tire_carcass_history_stress_" .. index,
        "ngp_contact_history_stress_" .. index,
    })

    local bristleLat = firstRaw({
        "ngp_carcass_bristle_lat_" .. index,
        "ngp_tire_carcass_bristle_lat_" .. index,
    })

    local bristleLong = firstRaw({
        "ngp_carcass_bristle_long_" .. index,
        "ngp_tire_carcass_bristle_long_" .. index,
    })

    local sidewallEnergy = firstRaw({
        "ngp_sidewall_energy_" .. index,
        "ngp_tire_sidewall_energy_" .. index,
    })

    local deformation = firstRaw({
        "ngp_tire_deformation_" .. index,
        "ngp_tire_carcass_deformation_" .. index,
    })

    if support ~= nil or gripGate ~= nil or vertical ~= nil or hyst ~= nil or delay ~= nil or recoveryBias ~= nil or heatSeed ~= nil or historyStress ~= nil or bristleLat ~= nil or bristleLong ~= nil or sidewallEnergy ~= nil or deformation ~= nil then
        M.state.carcassLinked = true
    end

    ws.carcassSupport = clamp(safeNumber(support, 1.0), 0.0, 1.2)
    ws.carcassGripGate = clamp(safeNumber(gripGate, 1.0), 0.0, 1.2)
    ws.carcassVerticalNorm = clamp(safeNumber(vertical, 0.0), 0.0, 1.5)
    ws.carcassHysteresis = clamp(safeNumber(hyst, 0.0), 0.0, 1.0)
    ws.carcassDelay = clamp(safeNumber(delay, 0.0), 0.0, 1.0)
    ws.carcassRecoveryBias = clamp(safeNumber(recoveryBias, 0.0), 0.0, 1.0)
    ws.heatSeed = clamp(safeNumber(heatSeed, 0.0), 0.0, 1.0)
    ws.historyStress = clamp(safeNumber(historyStress, 0.0), 0.0, 1.0)
    ws.carcassBristleLat = clamp(safeNumber(bristleLat, 0.0), 0.0, 1.0)
    ws.carcassBristleLong = clamp(safeNumber(bristleLong, 0.0), 0.0, 1.0)
    ws.sidewallEnergy = clamp(safeNumber(sidewallEnergy, 0.0), 0.0, 1.0)
    ws.deformation = clamp(safeNumber(deformation, 0.0), 0.0, 1.0)
end

local function readRootInputs(index, ws)
    local targetContactQuality = readContactInputs(index, ws)
    readCarcassInputs(index, ws)

    local tireLimit = firstRaw({
        "ngp_tire_limit_" .. index,
        "ngp_tyre_limit_" .. index,
        "ngp_tforce_limit_" .. index,
    })

    local tireGrip = firstRaw({
        "ngp_tire_grip_" .. index,
        "ngp_tyre_grip_" .. index,
        "ngp_tforce_grip_" .. index,
    })

    local forceSlipEnergy = firstRaw({
        "ngp_tire_force_slip_energy_" .. index,
        "ngp_tdyn_slip_energy_" .. index,
        "ngp_tire_dynamics_slip_energy_" .. index,
    })

    local forceCombined = firstRaw({
        "ngp_tire_force_combined_slip_" .. index,
        "ngp_tdyn_combined_slip_" .. index,
        "ngp_tire_dynamics_combined_slip_" .. index,
    })

    if tireLimit ~= nil or tireGrip ~= nil or forceSlipEnergy ~= nil or forceCombined ~= nil then
        M.state.dynamicsLinked = true
        M.state.forceLinked = M.state.forceLinked or forceSlipEnergy ~= nil or forceCombined ~= nil
    end

    ws.tireLimit = clamp(safeNumber(tireLimit, 0.0), 0.0, 2.0)
    ws.tireGrip = clamp(safeNumber(tireGrip, 1.0), 0.0, 1.25)
    ws.forceSlipEnergy = clamp(safeNumber(forceSlipEnergy, 0.0), 0.0, 1.0)
    ws.forceCombinedSlip = clamp(safeNumber(forceCombined, ws.contactCombinedSlip), 0.0, 2.5)

    local hop = firstRaw({
        "ngp_tirehop_energy_" .. index,
        "ngp_tire_hop_energy_" .. index,
        "ngp_tire_hop_" .. index,
        "ngp_thop_energy_" .. index,
    })

    if hop ~= nil then M.state.hopLinked = true end
    ws.hopEnergy = clamp(safeNumber(hop, 0.0), 0.0, 1.0)

    local armCamber = firstRaw({
        "ngp_control_arm_camber_" .. index,
        "ngp_arm_camber_" .. index,
        "ngp_ca_camber_" .. index,
        "ngp_compliance_virtual_camber_" .. index,
    })

    local armToe = firstRaw({
        "ngp_control_arm_toe_" .. index,
        "ngp_arm_toe_" .. index,
        "ngp_ca_toe_" .. index,
        "ngp_compliance_virtual_toe_" .. index,
    })

    if armCamber ~= nil or armToe ~= nil then M.state.armLinked = true end
    ws.armCamber = safeNumber(armCamber, 0.0)
    ws.armToe = safeNumber(armToe, 0.0)

    if index >= 2 then
        local driveTorque = firstRaw({
            "ngp_drive_torque_normalized_from_nm",
            "ngp_drive_torque",
            "ngp_drivetrain_torque",
            "ngp_dt_torque",
        })

        local lsdLock = firstRaw({
            "ngp_lsd_lock",
            "ngp_diff_lock",
        })

        local shaftTwist = firstRaw({
            "ngp_shaft_twist",
            "ngp_windup_twist",
            "ngp_driveline_twist",
        })

        if driveTorque ~= nil then M.state.drivetrainLinked = true end
        if lsdLock ~= nil then M.state.lsdLinked = true end

        ws.driveTorque = safeNumber(driveTorque, 0.0)
        ws.lsdLock = safeNumber(lsdLock, 0.0)
        ws.shaftTwist = safeNumber(shaftTwist, 0.0)
    else
        ws.driveTorque = 0.0
        ws.lsdLock = 0.0
        ws.shaftTwist = 0.0
    end

    local roadShock = firstRaw({
        "ngp_road_impact_" .. index,
        "ngp_rii_impact_" .. index,
        "ngp_road_input_severity_" .. index,
        "ngp_susp_road_impulse_" .. index,
    })

    local roadSeverity = firstRaw({
        "ngp_road_input_severity_" .. index,
        "ngp_rii_severity_" .. index,
        "ngp_rbi_wheel_input_" .. index,
    })

    local pathLoss = firstRaw({
        "ngp_load_path_loss_" .. index,
        "ngp_lp_path_loss_" .. index,
        "ngp_lp_force_loss_" .. index,
        "ngp_rii_path_loss_" .. index,
    })

    if roadShock ~= nil or roadSeverity ~= nil or pathLoss ~= nil then
        M.state.roadLinked = true
    end

    ws.roadShock = clamp(safeNumber(roadShock, 0.0), 0.0, 1.5)
    ws.roadSeverity = clamp(safeNumber(roadSeverity, 0.0), 0.0, 1.5)
    ws.pathLoss = clamp(safeNumber(pathLoss, 0.0), 0.0, 1.0)

    local thermalStress = firstRaw({
        "ngp_thermal_stress",
        "ngp_virtual_thermal_stress",
        "ngp_brake_root_heat_avg",
        "ngp_tire_memory_avg_heat",
    })

    if thermalStress ~= nil then M.state.thermalLinked = true end
    ws.thermalStress = clamp(safeNumber(thermalStress, 0.0), 0.0, 1.0)

    local snapRisk = firstRaw({
        "ngp_slip_snap_risk_" .. index,
        "ngp_snap_risk_" .. index,
    })

    local dirtyReturn = firstRaw({
        "ngp_slip_dirty_return_" .. index,
        "ngp_dirty_return_" .. index,
    })

    if snapRisk ~= nil or dirtyReturn ~= nil then
        M.state.recoveryLinked = true
    end

    ws.slipRecoveryRisk = clamp(safeNumber(snapRisk, 0.0), 0.0, 1.0)
    ws.dirtyReturn = clamp(safeNumber(dirtyReturn, 0.0), 0.0, 1.0)

    return targetContactQuality
end

local function calculateSlipDemand(ws)
    local saNorm = clamp(ws.slipAngle / math.max(M.params.slipAngleRef, 0.001), 0.0, 2.0)
    local srNorm = clamp(ws.slipRatio / math.max(M.params.slipRatioRef, 0.001), 0.0, 2.0)
    local combinedNorm = clamp(math.max(ws.contactCombinedSlip, ws.forceCombinedSlip), 0.0, 2.5)

    return clamp(
        saNorm * M.params.slipAngleBuildGain
        + srNorm * M.params.slipRatioBuildGain
        + combinedNorm * M.params.combinedSlipBuildGain,
        0.0,
        1.0
    )
end

local function updateThermalHistory(ws, dt)
    local p = M.params
    local loadFactor = clamp(ws.load / math.max(p.loadReference, 1.0), 0.0, 2.5)
    local loadHeat = smoothstep(p.loadWarmStart, p.loadWarmFull, ws.load)
    local slipDemand = calculateSlipDemand(ws)

    local bristleEnergy = math.max(ws.carcassBristleLat or 0.0, ws.carcassBristleLong or 0.0)
    local dampingEnergy = clamp(ws.carcassHysteresis * 0.55 + ws.sidewallEnergy * 0.35 + ws.deformation * 0.25, 0.0, 1.0)

    local microTarget = clamp(
        ws.heatSeed * 0.45
        + bristleEnergy * 0.28
        + slipDemand * 0.22
        + ws.forceSlipEnergy * 0.18
        + loadHeat * 0.10,
        0.0,
        p.maxHeat
    )

    local macroTarget = clamp(
        ws.historyStress * 0.45
        + dampingEnergy * p.dampingHeatGain
        + ws.carcassDelay * 0.16
        + ws.hopEnergy * 0.12
        + ws.roadShock * 0.10
        + math.max(loadFactor - 1.0, 0.0) * 0.10,
        0.0,
        p.maxHeat
    )

    ws.microHeat = lowPass(ws.microHeat, microTarget, p.heatTau, dt)
    ws.macroHeat = lowPass(ws.macroHeat, macroTarget, p.heatTau * 1.35, dt)

    local virtualTempTarget =
        p.ambientTemp
        + ws.microHeat * 38.0
        + ws.macroHeat * 52.0
        + ws.thermalStress * 18.0

    ws.virtualTemp = lowPass(ws.virtualTemp, virtualTempTarget, 1.25, dt)

    local overOptimum = smoothstep(p.optimumTemp, p.overheatTemp, math.max(ws.tyreTemp, ws.virtualTemp))
    local thermalBuild = clamp(
        ws.microHeat * 0.35
        + ws.macroHeat * 0.42
        + overOptimum * 0.34
        + ws.heatSeed * p.thermalBuildGain
        + ws.thermalStress * p.thermalStressBuildGain,
        0.0,
        p.maxThermalMemory
    )

    local thermalRelease = p.thermalReleaseGain * dt * (0.65 + 0.35 * ws.carcassSupport) * (1.0 - overOptimum * 0.65)
    ws.thermalMemory = clamp(math.max(ws.thermalMemory, thermalBuild) - thermalRelease, 0.0, p.maxThermalMemory)

    local abrasionBuild = clamp(
        ws.historyStress * p.abrasionBuildGain
        + math.max(ws.contactCombinedSlip, ws.forceCombinedSlip) * 0.10
        + math.max(1.0 - ws.contactTrust, 0.0) * 0.08
        + ws.carcassHysteresis * 0.08
        + ws.dirtyReturn * 0.08,
        0.0,
        p.maxAbrasionMemory
    )

    local abrasionRelease = p.abrasionReleaseGain * dt * (0.50 + 0.50 * ws.contactQuality)
    ws.abrasionMemory = clamp(math.max(ws.abrasionMemory, abrasionBuild) - abrasionRelease, 0.0, p.maxAbrasionMemory)

    local historyTarget = clamp(
        ws.thermalMemory * 0.36
        + ws.abrasionMemory * 0.42
        + ws.historyStress * 0.28
        + ws.carcassDelay * 0.12
        + ws.pathLoss * 0.08,
        0.0,
        1.0
    )

    local historyTau = historyTarget > ws.historyMemory and p.buildTau * 1.4 or p.releaseTau * 2.2
    ws.historyMemory = lowPass(ws.historyMemory, historyTarget, historyTau, dt)
end

local function calculateTempRecovery(ws)
    local p = M.params
    local temp = math.max(ws.tyreTemp or p.ambientTemp, ws.virtualTemp or p.ambientTemp)

    if temp < p.warmRecoveryTemp then
        return p.normalTempRecovery
    end

    if temp < p.hotRecoveryTemp then
        local t = smoothstep(p.warmRecoveryTemp, p.hotRecoveryTemp, temp)
        return lerp(p.warmTempRecovery, p.hotTempRecovery, t)
    end

    if temp < p.overheatTemp then
        local t = smoothstep(p.hotRecoveryTemp, p.overheatTemp, temp)
        return lerp(p.hotTempRecovery, p.overheatRecovery, t)
    end

    return p.overheatRecovery
end

local function calculateSlipRecovery(ws)
    local p = M.params
    local saNorm = clamp(ws.slipAngle / math.max(p.slipAngleRef, 0.001), 0.0, 2.0)
    local srNorm = clamp(ws.slipRatio / math.max(p.slipRatioRef, 0.001), 0.0, 2.0)

    local slipLoss =
        saNorm * p.slipAngleRecoveryGain
        + srNorm * p.slipRatioRecoveryGain
        + ws.contactLoss * p.contactLossRecoveryLoss
        + ws.abrasionMemory * p.abrasionRecoveryLoss
        + ws.thermalMemory * p.hotRecoveryLoss
        + ws.dirtyReturn * p.dirtyReturnRecoveryLoss
        + ws.roadShock * p.roadShockRecoveryLoss

    local supportGain = math.max(ws.carcassSupport - 0.65, 0.0) * p.supportRecoveryGain
    local qualityGain = math.max(ws.contactQuality - 0.65, 0.0) * p.contactQualityRecoveryGain
    local gripGateGain = math.max(ws.carcassGripGate - 0.65, 0.0) * p.gripGateRecoveryGain
    local recoveryBiasGain = ws.carcassRecoveryBias * p.recoveryBiasGain
    local tireGripGain = math.max(ws.tireGrip - 1.0, 0.0) * 0.06

    return clamp(
        1.0
        - slipLoss
        + supportGain
        + qualityGain
        + gripGateGain
        + recoveryBiasGain
        + tireGripGain,
        0.0,
        1.20
    )
end

local function calculateBuild(ws)
    local p = M.params
    local loadFactor = ws.load / math.max(p.loadReference, 1.0)
    local slipDemand = calculateSlipDemand(ws)

    local loadBuild = math.max(loadFactor - 0.80, 0.0) * p.loadBuildGain

    local contactBuild =
        math.max(1.0 - ws.contactQuality, 0.0) * p.contactLossBuildGain
        + math.max(1.0 - ws.contactTrust, 0.0) * p.contactTrustBuildGain
        + math.max(1.0 - ws.contactStabilityScore, 0.0) * p.contactUnstableBuildGain
        + ws.carcassDelay * p.contactDelayBuildGain

    local carcassBuild =
        ws.historyStress * p.carcassHistoryBuildGain
        + ws.heatSeed * p.carcassHeatBuildGain
        + ws.carcassHysteresis * p.carcassHysteresisBuildGain
        + ws.carcassVerticalNorm * p.carcassVerticalBuildGain
        + math.max(ws.carcassBristleLat, ws.carcassBristleLong) * p.bristleBuildGain

    local rootBuild =
        ws.tireLimit * p.tireLimitBuildGain
        + ws.hopEnergy * p.hopBuildGain
        + math.abs(ws.armCamber) * p.armCamberBuildGain
        + math.abs(ws.armToe) * p.armToeBuildGain
        + ws.forceSlipEnergy * p.forceSlipEnergyBuildGain
        + ws.slipRecoveryRisk * p.slipRecoveryRiskGain
        + ws.roadShock * p.roadShockBuildGain
        + ws.pathLoss * p.pathLossBuildGain

    if ws.driveTorque ~= 0.0 or ws.shaftTwist ~= 0.0 or ws.lsdLock ~= 0.0 then
        rootBuild =
            rootBuild
            + math.abs(ws.driveTorque) * p.driveRearBuildGain
            + math.abs(ws.shaftTwist) * p.shaftRearBuildGain
            + ws.lsdLock * p.lsdRearBuildGain
    end

    local thermalBuild =
        ws.thermalMemory * 0.28
        + ws.abrasionMemory * 0.34
        + ws.historyMemory * p.historyStickiness * 0.20
        + ws.thermalStress * p.thermalStressBuildGain

    return clamp(
        slipDemand
        + loadBuild
        + contactBuild
        + carcassBuild
        + rootBuild
        + thermalBuild,
        0.0,
        p.maxMemory
    )
end

local function updateMemory(ws, dt)
    local p = M.params

    updateThermalHistory(ws, dt)

    local build = calculateBuild(ws)
    local tempRecovery = calculateTempRecovery(ws)
    local slipRecovery = calculateSlipRecovery(ws)

    local supportRecovery = clamp(0.55 + 0.45 * ws.carcassSupport, 0.25, 1.15)
    local contactRecovery = clamp(0.35 + 0.65 * ws.contactQuality, 0.15, 1.15)
    local gateRecovery = clamp(0.50 + 0.50 * ws.carcassGripGate, 0.20, 1.15)

    local recoveryScalar = tempRecovery * slipRecovery * supportRecovery * contactRecovery * gateRecovery
    local recovery = dt * p.recoveryGain * recoveryScalar

    if build > ws.memory then
        ws.memory = lowPass(ws.memory, build, p.buildTau, dt)
    else
        local targetMemory = math.max(build, ws.historyMemory * 0.55)
        ws.memory = lowPass(ws.memory, targetMemory, p.releaseTau, dt)
        ws.memory = clamp(ws.memory - recovery, 0.0, p.maxMemory)
    end

    ws.memory = clamp(ws.memory, 0.0, p.maxMemory)
    ws.build = build
    ws.recovery = recovery
    ws.recoveryScalar = recoveryScalar
    ws.tempRecovery = tempRecovery
    ws.slipRecovery = slipRecovery
end

local function updateGrip(ws, dt)
    local p = M.params

    local memoryLoss = ws.memory * p.memoryGripLoss
    local thermalLoss = ws.thermalMemory * p.thermalGripLoss
    local abrasionLoss = ws.abrasionMemory * p.abrasionGripLoss
    local contactLoss = math.max(1.0 - ws.contactTrust, 0.0) * p.contactGripLoss
    local forceLoss = ws.forceSlipEnergy * p.forceGripLoss
    local roadLoss = math.max(ws.roadShock, ws.pathLoss) * p.roadGripLoss

    local recontactBonus = ws.recontactShock * p.recontactGripBonus
    local gate = clamp(0.72 + 0.28 * ws.carcassGripGate, 0.55, 1.08)

    local targetGrip =
        (1.0 - memoryLoss - thermalLoss - abrasionLoss - contactLoss - forceLoss - roadLoss + recontactBonus)
        * gate

    targetGrip = clamp(targetGrip, p.minGrip, p.maxGrip)

    ws.grip = lowPass(ws.grip, targetGrip, p.gripTau, dt)
    ws.grip = clamp(ws.grip, p.minGrip, p.maxGrip)
end

local function updatePhase(ws, dt)
    local p = M.params
    local loadFactor = ws.load / math.max(p.loadReference, 1.0)

    local wasLowContact = (ws.prevContactQuality or ws.contactQuality or 1.0) <= p.recontactQualityLow
    local isRecontact =
        wasLowContact
        and (ws.contactQuality or 0.0) >= p.recontactQualityHigh
        and (ws.contactQualityRate or 0.0) >= p.recontactRate
        and (ws.memory or 0.0) >= p.recontactMemoryMin

    ws.recontact = isRecontact and 1.0 or 0.0
    ws.recontactShock = lowPass(ws.recontactShock, ws.recontact, p.recontactTau, dt)

    if not ws.active then
        setPhase(ws, "STORE_ONLY")
        return
    end

    if loadFactor < p.unloadedLoadFactor or ws.contactStatus == 4 then
        setPhase(ws, "UNLOADED")
        return
    end

    if isRecontact then
        setPhase(ws, "RECONTACT")
        return
    end

    if ws.contactStatus == 3 or ws.contactCombinedSlip >= p.saturatedCombinedSlip or ws.forceCombinedSlip >= p.saturatedCombinedSlip then
        setPhase(ws, "SATURATED")
        return
    end

    if ws.contactStatus == 2 or ws.contactCombinedSlip >= p.slippingCombinedSlip or ws.forceCombinedSlip >= p.slippingCombinedSlip then
        setPhase(ws, "SLIPPING")
        return
    end

    if ws.thermalMemory >= p.hotMemoryThreshold then
        setPhase(ws, "HOT")
        return
    end

    if ws.historyMemory >= p.historyThreshold then
        setPhase(ws, "HISTORY")
        return
    end

    if ws.memory >= p.memoryRecoveringMin and ws.build <= ws.recovery + p.loadingBuildMargin then
        setPhase(ws, "RECOVERING")
        return
    end

    if ws.build > ws.recovery + p.loadingBuildMargin then
        setPhase(ws, "LOADING")
        return
    end

    setPhase(ws, "GRIP")
end

local function clearWheel(index, dt)
    dt = dt or 0.016
    local ws = M.state.wheels[index]

    ws.active = false
    ws.slipAngle = lowPass(ws.slipAngle, 0.0, M.params.slipTau, dt)
    ws.slipRatio = lowPass(ws.slipRatio, 0.0, M.params.slipTau, dt)
    ws.rawSlipAngle = 0.0
    ws.rawSlipRatio = 0.0

    ws.rawLoad = lowPass(ws.rawLoad, M.params.loadReference, M.params.loadTau, dt)
    ws.load = lowPass(ws.load, M.params.loadReference, M.params.loadTau, dt)
    ws.loadRate = 0.0

    ws.memory = lowPass(ws.memory, 0.0, M.params.releaseTau, dt)
    ws.thermalMemory = lowPass(ws.thermalMemory, 0.0, M.params.releaseTau * 2.0, dt)
    ws.abrasionMemory = lowPass(ws.abrasionMemory, 0.0, M.params.releaseTau * 4.0, dt)
    ws.historyMemory = lowPass(ws.historyMemory, 0.0, M.params.releaseTau * 3.0, dt)
    ws.grip = lowPass(ws.grip, 1.0, M.params.gripTau, dt)

    ws.build = 0.0
    ws.recovery = 0.0
    ws.recoveryScalar = 1.0
    ws.tyreTemp = M.params.ambientTemp
    ws.virtualTemp = lowPass(ws.virtualTemp, M.params.ambientTemp, 1.25, dt)
    ws.tempRecovery = 1.0
    ws.slipRecovery = 1.0
    ws.microHeat = lowPass(ws.microHeat, 0.0, M.params.heatTau, dt)
    ws.macroHeat = lowPass(ws.macroHeat, 0.0, M.params.heatTau, dt)
    ws.heatSeed = 0.0
    ws.historyStress = 0.0

    ws.contactQuality = lowPass(ws.contactQuality, 1.0, M.params.contactTau, dt)
    ws.prevContactQuality = ws.contactQuality
    ws.contactQualityRate = 0.0
    ws.contactRaw = 1.0
    ws.contactTrust = 1.0
    ws.contactCombinedSlip = 0.0
    ws.contactStabilityScore = 1.0
    ws.contactStatus = 0
    ws.contactLoss = 0.0
    ws.contactEnergy = 0.0

    ws.carcassSupport = 1.0
    ws.carcassGripGate = 1.0
    ws.carcassVerticalNorm = 0.0
    ws.carcassHysteresis = 0.0
    ws.carcassDelay = 0.0
    ws.carcassRecoveryBias = 0.0
    ws.carcassBristleLat = 0.0
    ws.carcassBristleLong = 0.0
    ws.sidewallEnergy = 0.0
    ws.deformation = 0.0

    ws.tireLimit = 0.0
    ws.tireGrip = 1.0
    ws.forceSlipEnergy = 0.0
    ws.forceCombinedSlip = 0.0
    ws.hopEnergy = 0.0
    ws.armCamber = 0.0
    ws.armToe = 0.0
    ws.driveTorque = 0.0
    ws.lsdLock = 0.0
    ws.shaftTwist = 0.0
    ws.roadShock = 0.0
    ws.roadSeverity = 0.0
    ws.pathLoss = 0.0
    ws.thermalStress = 0.0
    ws.slipRecoveryRisk = 0.0
    ws.dirtyReturn = 0.0
    ws.recontact = 0.0
    ws.recontactShock = lowPass(ws.recontactShock, 0.0, M.params.recontactTau, dt)
    setPhase(ws, "INIT")
end

local function exportWheel(index, ws)
    safeStore("ngp_memory_" .. index, ws.memory or 0.0)
    safeStore("ngp_rubber_memory_" .. index, ws.memory or 0.0)
    safeStore("ngp_tire_memory_" .. index, ws.memory or 0.0)
    safeStore("ngp_tyre_memory_" .. index, ws.memory or 0.0)

    safeStore("ngp_memory_grip_" .. index, ws.grip or 1.0)
    safeStore("ngp_tire_memory_grip_" .. index, ws.grip or 1.0)
    safeStore("ngp_tyre_memory_grip_" .. index, ws.grip or 1.0)

    safeStore("ngp_tire_memory_phase_id_" .. index, ws.phaseId or 0)
    safeStore("ngp_tire_memory_phase_" .. index, ws.phaseText or "INIT")
    safeStore("ngp_tyre_memory_phase_id_" .. index, ws.phaseId or 0)
    safeStore("ngp_tyre_memory_phase_" .. index, ws.phaseText or "INIT")
    safeStore("ngp_tire_memory_recontact_" .. index, ws.recontact or 0.0)

    safeStore("ngp_memory_thermal_" .. index, ws.thermalMemory or 0.0)
    safeStore("ngp_memory_abrasion_" .. index, ws.abrasionMemory or 0.0)
    safeStore("ngp_memory_history_" .. index, ws.historyMemory or 0.0)
    safeStore("ngp_memory_micro_heat_" .. index, ws.microHeat or 0.0)
    safeStore("ngp_memory_macro_heat_" .. index, ws.macroHeat or 0.0)
    safeStore("ngp_memory_virtual_temp_" .. index, ws.virtualTemp or M.params.ambientTemp)
    safeStore("ngp_memory_recovery_scalar_" .. index, ws.recoveryScalar or 1.0)
    safeStore("ngp_memory_recontact_shock_" .. index, ws.recontactShock or 0.0)

    safeStore("ngp_tire_memory_thermal_" .. index, ws.thermalMemory or 0.0)
    safeStore("ngp_tire_memory_abrasion_" .. index, ws.abrasionMemory or 0.0)
    safeStore("ngp_tire_memory_history_" .. index, ws.historyMemory or 0.0)
    safeStore("ngp_tire_memory_heat_" .. index, math.max(ws.microHeat or 0.0, ws.macroHeat or 0.0))
    safeStore("ngp_tire_memory_recovery_scalar_" .. index, ws.recoveryScalar or 1.0)

    safeStore("ngp_tm_memory_" .. index, ws.memory or 0.0)
    safeStore("ngp_tm_grip_" .. index, ws.grip or 1.0)
    safeStore("ngp_tm_phase_id_" .. index, ws.phaseId or 0)
    safeStore("ngp_tm_thermal_" .. index, ws.thermalMemory or 0.0)
    safeStore("ngp_tm_abrasion_" .. index, ws.abrasionMemory or 0.0)
    safeStore("ngp_tm_history_" .. index, ws.historyMemory or 0.0)
    safeStore("ngp_tm_recovery_scalar_" .. index, ws.recoveryScalar or 1.0)

    if not M.state.debugStoreNow then return end

    safeStore("ngp_memory_build_" .. index, ws.build or 0.0)
    safeStore("ngp_memory_recovery_" .. index, ws.recovery or 0.0)
    safeStore("ngp_memory_slip_angle_" .. index, ws.slipAngle or 0.0)
    safeStore("ngp_memory_slip_ratio_" .. index, ws.slipRatio or 0.0)
    safeStore("ngp_memory_load_" .. index, ws.load or 0.0)
    safeStore("ngp_memory_load_rate_" .. index, ws.loadRate or 0.0)
    safeStore("ngp_memory_temp_" .. index, ws.tyreTemp or M.params.ambientTemp)
    safeStore("ngp_memory_active_" .. index, ws.active and 1 or 0)

    safeStore("ngp_memory_contact_loss_" .. index, ws.contactLoss or 0.0)
    safeStore("ngp_memory_contact_quality_" .. index, ws.contactQuality or 1.0)
    safeStore("ngp_memory_contact_trust_" .. index, ws.contactTrust or 1.0)
    safeStore("ngp_memory_contact_combined_slip_" .. index, ws.contactCombinedSlip or 0.0)
    safeStore("ngp_memory_contact_stability_" .. index, ws.contactStabilityScore or 1.0)
    safeStore("ngp_memory_contact_quality_rate_" .. index, ws.contactQualityRate or 0.0)

    safeStore("ngp_memory_carcass_support_" .. index, ws.carcassSupport or 1.0)
    safeStore("ngp_memory_carcass_hysteresis_" .. index, ws.carcassHysteresis or 0.0)
    safeStore("ngp_memory_carcass_delay_" .. index, ws.carcassDelay or 0.0)
    safeStore("ngp_memory_carcass_heat_seed_" .. index, ws.heatSeed or 0.0)
    safeStore("ngp_memory_carcass_history_stress_" .. index, ws.historyStress or 0.0)

    safeStore("ngp_memory_tire_limit_" .. index, ws.tireLimit or 0.0)
    safeStore("ngp_memory_hop_" .. index, ws.hopEnergy or 0.0)
    safeStore("ngp_memory_lsd_" .. index, ws.lsdLock or 0.0)
    safeStore("ngp_memory_road_shock_" .. index, ws.roadShock or 0.0)
    safeStore("ngp_memory_path_loss_" .. index, ws.pathLoss or 0.0)
    safeStore("ngp_memory_force_slip_energy_" .. index, ws.forceSlipEnergy or 0.0)
    safeStore("ngp_memory_dirty_return_" .. index, ws.dirtyReturn or 0.0)
end

local function exportGlobal()
    safeStore("ngp_tire_memory_status", M.state.status or "UNKNOWN")
    safeStore("ngp_tyre_memory_status", M.state.status or "UNKNOWN")
    safeStore("ngp_tire_memory_update_count", M.state.updateCount or 0)
    safeStore("ngp_tire_memory_wheels_valid", M.state.wheelsValid and 1 or 0)

    safeStore("ngp_tire_memory_avg", M.state.avgMemory or 0.0)
    safeStore("ngp_tire_memory_avg_grip", M.state.avgGrip or 1.0)
    safeStore("ngp_tire_memory_avg_thermal", M.state.avgThermal or 0.0)
    safeStore("ngp_tire_memory_avg_abrasion", M.state.avgAbrasion or 0.0)
    safeStore("ngp_tire_memory_avg_heat", M.state.avgHeat or 0.0)

    safeStore("ngp_tm_status", M.state.status or "UNKNOWN")
    safeStore("ngp_tm_update_count", M.state.updateCount or 0)
    safeStore("ngp_tm_store_only", M.state.storeOnly and 1 or 0)
    safeStore("ngp_tm_avg_memory", M.state.avgMemory or 0.0)
    safeStore("ngp_tm_avg_grip", M.state.avgGrip or 1.0)
    safeStore("ngp_tm_avg_history", M.state.avgHistory or 0.0)
    safeStore("ngp_tm_max_memory", M.state.maxMemory or 0.0)
    safeStore("ngp_tm_min_grip", M.state.minGrip or 1.0)
    safeStore("ngp_tm_active_count", M.state.activeCount or 0)

    if not M.state.debugStoreNow then return end

    safeStore("ngp_tire_memory_avg_build", M.state.avgBuild or 0.0)
    safeStore("ngp_tire_memory_avg_recovery", M.state.avgRecovery or 0.0)
    safeStore("ngp_tire_memory_avg_recovery_scalar", M.state.avgRecoveryScalar or 1.0)

    safeStore("ngp_tire_memory_contact_linked", M.state.contactLinked and 1 or 0)
    safeStore("ngp_tire_memory_carcass_linked", M.state.carcassLinked and 1 or 0)
    safeStore("ngp_tire_memory_dynamics_linked", M.state.dynamicsLinked and 1 or 0)
    safeStore("ngp_tire_memory_hop_linked", M.state.hopLinked and 1 or 0)
    safeStore("ngp_tire_memory_arm_linked", M.state.armLinked and 1 or 0)
    safeStore("ngp_tire_memory_drivetrain_linked", M.state.drivetrainLinked and 1 or 0)
    safeStore("ngp_tire_memory_lsd_linked", M.state.lsdLinked and 1 or 0)
    safeStore("ngp_tire_memory_load_linked", M.state.loadLinked and 1 or 0)
    safeStore("ngp_tire_memory_force_linked", M.state.forceLinked and 1 or 0)
    safeStore("ngp_tire_memory_road_linked", M.state.roadLinked and 1 or 0)
    safeStore("ngp_tire_memory_thermal_linked", M.state.thermalLinked and 1 or 0)
    safeStore("ngp_tire_memory_recovery_linked", M.state.recoveryLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i, M.state.wheels[i])
    end
    exportGlobal()
end

local function resetLinks()
    M.state.contactLinked = false
    M.state.carcassLinked = false
    M.state.dynamicsLinked = false
    M.state.hopLinked = false
    M.state.armLinked = false
    M.state.drivetrainLinked = false
    M.state.lsdLinked = false
    M.state.loadLinked = false
    M.state.forceLinked = false
    M.state.roadLinked = false
    M.state.thermalLinked = false
    M.state.recoveryLinked = false
end

local function updateWheel(index, wheel, dt, storeOnly)
    local ws = M.state.wheels[index]

    if not hasInput(index, wheel) then
        clearWheel(index, dt)
        return
    end

    ws.active = not storeOnly or wheel ~= nil

    local rawSA, rawSR = readSlip(index, wheel)
    ws.rawSlipAngle = rawSA
    ws.rawSlipRatio = rawSR

    local rawLoad = math.abs(getWheelLoad(wheel, index))
    ws.rawLoad = rawLoad

    local prevLoad = ws.load or rawLoad

    ws.slipAngle = lowPass(ws.slipAngle, rawSA, M.params.slipTau, dt)
    ws.slipRatio = lowPass(ws.slipRatio, rawSR, M.params.slipTau, dt)
    ws.load = lowPass(ws.load, rawLoad, M.params.loadTau, dt)
    ws.loadRate = (ws.load - prevLoad) / math.max(dt, 0.001)
    ws.prevLoad = prevLoad

    local prevContactQuality = ws.contactQuality or 1.0
    local targetContactQuality = readRootInputs(index, ws)
    ws.contactQuality = lowPass(prevContactQuality, targetContactQuality, M.params.contactTau, dt)
    ws.contactQualityRate = (ws.contactQuality - prevContactQuality) / math.max(dt, 0.001)
    ws.prevContactQuality = prevContactQuality

    ws.tyreTemp = getTyreTemp(index)

    updateMemory(ws, dt)
    updateGrip(ws, dt)
    updatePhase(ws, dt)
end

function M.init()
    M.state.status = "INIT"
    M.state.storeOnly = false
    exportState()
end

function M.update(dt, car, runtime)
    M.state.updateCount = (M.state.updateCount or 0) + 1

    dt = safeNumber(dt, 0.0)
    if dt <= 0.0 then
        M.state.status = "BAD DT"
        exportState()
        return
    end

    dt = clamp(dt, M.params.minDt, M.params.maxDt)
    updateDebugGate(dt)

    if not car then
        car = safeGetCar()
    end

    local wheels = safeGetWheels(car)
    local storeOnly = wheels == nil

    M.state.storeOnly = storeOnly
    M.state.wheelsValid = wheels ~= nil
    M.state.status = storeOnly and "STORE ONLY" or "RUNNING"

    resetLinks()

    local sumMemory = 0.0
    local sumGrip = 0.0
    local sumBuild = 0.0
    local sumRecovery = 0.0
    local sumThermal = 0.0
    local sumAbrasion = 0.0
    local sumHeat = 0.0
    local sumHistory = 0.0
    local sumRecoveryScalar = 0.0
    local maxMemory = 0.0
    local minGrip = 999.0
    local activeCount = 0

    for i = 0, 3 do
        local wheel = getWheel(wheels, i)
        updateWheel(i, wheel, dt, storeOnly)

        local ws = M.state.wheels[i]

        if ws.active then
            activeCount = activeCount + 1
        end

        sumMemory = sumMemory + (ws.memory or 0.0)
        sumGrip = sumGrip + (ws.grip or 1.0)
        sumBuild = sumBuild + (ws.build or 0.0)
        sumRecovery = sumRecovery + (ws.recovery or 0.0)
        sumThermal = sumThermal + (ws.thermalMemory or 0.0)
        sumAbrasion = sumAbrasion + (ws.abrasionMemory or 0.0)
        sumHeat = sumHeat + math.max(ws.microHeat or 0.0, ws.macroHeat or 0.0)
        sumHistory = sumHistory + (ws.historyMemory or 0.0)
        sumRecoveryScalar = sumRecoveryScalar + (ws.recoveryScalar or 1.0)
        maxMemory = math.max(maxMemory, ws.memory or 0.0)
        minGrip = math.min(minGrip, ws.grip or 1.0)

        exportWheel(i, ws)
    end

    M.state.avgMemory = sumMemory * 0.25
    M.state.avgGrip = sumGrip * 0.25
    M.state.avgBuild = sumBuild * 0.25
    M.state.avgRecovery = sumRecovery * 0.25
    M.state.avgThermal = sumThermal * 0.25
    M.state.avgAbrasion = sumAbrasion * 0.25
    M.state.avgHeat = sumHeat * 0.25
    M.state.avgHistory = sumHistory * 0.25
    M.state.avgRecoveryScalar = sumRecoveryScalar * 0.25
    M.state.maxMemory = maxMemory
    M.state.minGrip = minGrip == 999.0 and 1.0 or minGrip
    M.state.activeCount = activeCount

    exportGlobal()
end

function M.getMemory(index)
    local ws = M.state.wheels[index]
    return ws and ws.memory or 0.0
end

function M.getGrip(index)
    local ws = M.state.wheels[index]
    return ws and ws.grip or 1.0
end

function M.getThermalMemory(index)
    local ws = M.state.wheels[index]
    return ws and ws.thermalMemory or 0.0
end

function M.getAbrasionMemory(index)
    local ws = M.state.wheels[index]
    return ws and ws.abrasionMemory or 0.0
end

function M.getHistoryMemory(index)
    local ws = M.state.wheels[index]
    return ws and ws.historyMemory or 0.0
end

function M.getRecoveryScalar(index)
    local ws = M.state.wheels[index]
    return ws and ws.recoveryScalar or 1.0
end

function M.getPhase(index)
    local ws = M.state.wheels[index]
    return ws and ws.phaseText or "UNKNOWN"
end

function M.getState(index)
    if index == nil then
        return M.state
    end
    return M.state.wheels[index]
end

function M.debugStr(index)
    local ws = M.state.wheels[index or 0] or M.state.wheels[0]

    return string.format(
        "Status %s / Count %.0f / Wheels %s / Store %s\n" ..
        "Phase %s / Mem %.3f Grip %.3f Hist %.3f\n" ..
        "Build %.3f Rec %.4f Scalar %.2f Heat %.3f Abr %.3f\n" ..
        "SA %.3f SR %.3f Load %.0f CQ %.2f Trust %.2f\n" ..
        "Carc Sup %.2f Hyst %.2f Delay %.2f ReC %.0f\n" ..
        "Road %.2f Path %.2f ForceE %.2f Dirty %.2f\n" ..
        "Links CT:%s Car:%s TD:%s Hop:%s Arm:%s DT:%s LSD:%s Force:%s Road:%s",
        tostring(M.state.status),
        M.state.updateCount or 0,
        M.state.wheelsValid and "OK" or "NIL",
        M.state.storeOnly and "YES" or "NO",

        tostring(ws.phaseText or "INIT"),
        ws.memory or 0.0,
        ws.grip or 1.0,
        ws.historyMemory or 0.0,

        ws.build or 0.0,
        ws.recovery or 0.0,
        ws.recoveryScalar or 1.0,
        math.max(ws.microHeat or 0.0, ws.macroHeat or 0.0),
        ws.abrasionMemory or 0.0,

        ws.slipAngle or 0.0,
        ws.slipRatio or 0.0,
        ws.load or 0.0,
        ws.contactQuality or 1.0,
        ws.contactTrust or 1.0,

        ws.carcassSupport or 1.0,
        ws.carcassHysteresis or 0.0,
        ws.carcassDelay or 0.0,
        ws.recontact or 0.0,

        ws.roadShock or 0.0,
        ws.pathLoss or 0.0,
        ws.forceSlipEnergy or 0.0,
        ws.dirtyReturn or 0.0,

        M.state.contactLinked and "OK" or "NIL",
        M.state.carcassLinked and "OK" or "NIL",
        M.state.dynamicsLinked and "OK" or "NIL",
        M.state.hopLinked and "OK" or "NIL",
        M.state.armLinked and "OK" or "NIL",
        M.state.drivetrainLinked and "OK" or "NIL",
        M.state.lsdLinked and "OK" or "NIL",
        M.state.forceLinked and "OK" or "NIL",
        M.state.roadLinked and "OK" or "NIL"
    )
end

return M
