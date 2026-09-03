---@diagnostic disable: undefined-global

--============================================================
-- ACNeXtGen
-- tire_dynamics.lua
-- Phase 1.x / V1.1.5 stable
-- Tire Dynamics Core
--
-- Root/trunk tire transient signal model.
-- This module does not write AC physics directly.
--============================================================

local M = {}

--============================================================
-- Parameters
--============================================================

M.params = {
    -- Slip model
    slipRatioRef = 0.12,
    slipAngleRef = 10.0,
    slipAngleRefRad = 0.174533,
    combinedSlipLongWeight = 1.00,
    combinedSlipLatWeight  = 1.00,

    postLimitGripDrop = 0.40,
    gripAvailableBase = 1.00,
    gripLimitSoftness = 0.55,

    slipEnergyBuildStart = 0.65,
    slipEnergyBuildGain  = 0.42,
    slipEnergyDecayGain  = 0.85,
    slipEnergyGripLoss   = 0.05,

    -- Load and temperature
    loadReference = 3500.0,
    defaultLoad = 3500.0,
    loadGripGain = 0.08,
    minLoadGripScale = 0.90,
    maxLoadGripScale = 1.10,

    ambientTemp = 25.0,
    warmTemp    = 45.0,
    optimumTemp = 95.0,
    tempRange = 70.0,
    coldGripBase = 0.88,
    coldGripGain = 0.18,
    optimumGrip = 1.02,
    overheatGripLoss = 0.008,
    minTempGrip = 0.72,
    maxTempGrip = 1.05,

    -- Bias and memory
    biasBase = 1.00,
    memoryBuildStart = 0.75,
    memoryBuildGain  = 0.80,
    memoryDecayGain  = 0.25,
    rubberMemoryGripLoss = 0.08,
    rubberMemoryLimitGain = 0.20,

    -- Root coupling
    hopGripLoss = 0.10,

    contactGripGain = 1.0,
    contactLatGain  = 0.50,
    contactLongGain = 0.50,
    contactQualityGripGain = 0.18,
    contactLossLimitGain = 0.16,
    responseScaleGripGain = 0.10,
    responseScaleLimitGain = 0.12,

    driveTorqueSlipGain = 0.16,
    lsdLockRearLimitGain = 0.14,
    lsdDiffRearLimitGain = 0.05,
    shaftTwistDelayGain = 0.08,
    driveLashDelayGain = 0.10,

    armCamberGripLossGain = 0.045,
    armToeGripLossGain = 0.065,
    controlArmLimitGain = 0.08,

    complianceDelayGripLoss = 0.10,
    complianceDelayLimitLoss = 0.08,

    carcassSupportGripGain = 0.22,
    carcassGateGripGain = 0.18,
    carcassDeformationLimitGain = 0.12,
    carcassDelayGripLoss = 0.08,
    recoveryBiasLimitGain = 0.08,

    slipRecoveryGripGain = 0.10,
    slipRecoverySnapLimitGain = 0.10,
    roadSeverityLimitGain = 0.08,
    roadShockGripLoss = 0.06,
    loadPathLossGripLoss = 0.08,

    thermalStressGripLoss = 0.05,
    thermalStressLimitGain = 0.04,

    -- Output limits
    maxGrip  = 1.20,
    maxLimit = 2.00,
    minGrip = 0.0,

    -- Runtime
    minDt = 0.0001,
    maxDt = 0.100,
    decayTau = 0.180,
    debugStoreInterval = 0.25,
}

--============================================================
-- State
--============================================================

local WHEEL_NAMES = { [0] = "FL", [1] = "FR", [2] = "RL", [3] = "RR" }

M.state = {
    wheels = {},

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,
    storeOnly = false,

    avgGrip = 1.0,
    avgLimit = 0.0,
    avgMemory = 0.0,
    avgResponseDelay = 0.0,
    avgCombinedSlip = 0.0,
    avgSlipEnergy = 0.0,
    minGripLive = 1.0,
    maxLimitLive = 0.0,

    drivetrainLinked = false,
    lsdLinked = false,
    contactLinked = false,
    armLinked = false,
    complianceLinked = false,
    carcassLinked = false,
    recoveryLinked = false,
    roadLinked = false,
    loadPathLinked = false,
    thermalLinked = false,
    loadLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

for i = 0, 3 do
    M.state.wheels[i] = {
        grip = 1.0,
        limit = 0.0,

        memory = 0.0,
        tempGrip = 1.0,

        slipRatio = 0.0,
        slipAngle = 0.0,

        normalizedSlip = 0.0,
        combinedSlip = 0.0,
        slipEnergy = 0.0,

        effectiveSlipRatio = 0.0,
        effectiveSlipAngle = 0.0,

        load = M.params.defaultLoad,
        loadScale = 1.0,

        bias = 0.50,
        memoryGrip = 1.0,
        hopEnergy = 0.0,

        rubberLag = 0.0,

        tyreTemp = M.params.ambientTemp,
        tempNorm = 0.0,

        contactGrip = 1.0,
        contactLat = 1.0,
        contactLong = 1.0,

        contactQuality = 1.0,
        effectiveGrip = 1.0,
        contactLoss = 0.0,
        responseScale = 1.0,
        complianceDelay = 0.0,
        complianceInput = 0.0,
        complianceScale = 1.0,

        driveTorque = 0.0,
        lsdLock = 0.0,
        lsdDiff = 0.0,
        shaftTwist = 0.0,
        driveLash = 0.0,

        armCamber = 0.0,
        armToe = 0.0,
        geometryLoss = 0.0,

        carcassSupport = 1.0,
        carcassGripGate = 1.0,
        carcassDeformation = 0.0,
        carcassDelay = 0.0,
        carcassRecoveryBias = 0.0,
        carcassHeatSeed = 0.0,

        recoveryGripReturn = 1.0,
        recoveryRate = 0.0,
        recoverySnapRisk = 0.0,
        recoveryBite = 0.0,
        recoveryDirtyReturn = 0.0,
        recoveryTrust = 1.0,

        roadSeverity = 0.0,
        roadShock = 0.0,
        roadSurfaceLimit = 0.0,
        loadPathLoss = 0.0,
        loadPathWork = 0.0,

        thermalStress = 0.0,

        finalGripRaw = 1.0,
        active = false,
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
    v = safeNumber(v, 0.0)
    if v < 0.0 then return -v end
    return v
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.0)
    dt = safeNumber(dt, 0.0)
    if tau <= 0.000001 then return target end
    return current + (target - current) * clamp(dt / (tau + dt), 0.0, 1.0)
end

local function safeLoadRaw(key)
    if not ac or not ac.load then return nil end
    local ok, value = pcall(function()
        return ac.load(key)
    end)
    if not ok then return nil end
    return value
end

local function safeLoad(key, defaultValue)
    local value = safeLoadRaw(key)
    if value == nil then return defaultValue or 0.0 end
    return safeNumber(value, defaultValue or 0.0)
end

local function loadFirst(defaultValue, ...)
    local keys = { ... }
    for n = 1, #keys do
        local value = safeLoadRaw(keys[n])
        if value ~= nil then
            return safeNumber(value, defaultValue or 0.0), keys[n]
        end
    end
    if defaultValue == nil then return nil, nil end
    return defaultValue, nil
end

local function safeStore(key, value)
    if not ac or not ac.store then return end
    pcall(function()
        ac.store(key, value)
    end)
end

local function safeField(obj, key, defaultValue)
    if not obj then return defaultValue end
    local ok, value = pcall(function()
        return obj[key]
    end)
    if not ok or value == nil then return defaultValue end
    return value
end

local function safeGetCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function()
        return ac.getCar(0)
    end)
    if not ok then return nil end
    return car
end

local function getWheels(car)
    return safeField(car, "wheels", nil)
end

local function getWheel(car, index)
    local wheels = getWheels(car)
    if not wheels then return nil end
    local wheel = safeField(wheels, index, nil)
    if wheel ~= nil then return wheel end
    return safeField(wheels, index + 1, nil)
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

--============================================================
-- Input helpers
--============================================================

local function readSlip(index, wheel)
    local sr, srKey = loadFirst(nil,
        "ngp_contact_slip_ratio_" .. index,
        "ngp_tire_state_slip_ratio_" .. index,
        "ngp_tire_effective_slip_ratio_" .. index,
        "ngp_tdyn_effective_slip_ratio_" .. index,
        "ngp_slip_ratio_" .. index,
        "ngp_filtered_slip_ratio_" .. index
    )

    local sa, saKey = loadFirst(nil,
        "ngp_contact_slip_angle_" .. index,
        "ngp_tire_state_slip_angle_" .. index,
        "ngp_tire_effective_slip_angle_" .. index,
        "ngp_tdyn_effective_slip_angle_" .. index,
        "ngp_slip_angle_" .. index,
        "ngp_filtered_slip_angle_" .. index
    )

    if sr == nil and wheel then
        sr = safeNumber(safeField(wheel, "slipRatio", 0.0), 0.0)
    end

    if sa == nil and wheel then
        sa = safeNumber(safeField(wheel, "slipAngle", 0.0), 0.0)
    end

    sr = safeNumber(sr, 0.0)
    sa = safeNumber(sa, 0.0)

    local esr = loadFirst(sr,
        "ngp_tire_effective_slip_ratio_" .. index,
        "ngp_tdyn_effective_slip_ratio_" .. index,
        "ngp_contact_effective_slip_ratio_" .. index
    )

    local esa = loadFirst(sa,
        "ngp_tire_effective_slip_angle_" .. index,
        "ngp_tdyn_effective_slip_angle_" .. index,
        "ngp_contact_effective_slip_angle_" .. index
    )

    local combined = loadFirst(nil,
        "ngp_contact_combined_slip_" .. index,
        "ngp_tire_state_combined_slip_" .. index,
        "ngp_tdyn_combined_slip_" .. index,
        "ngp_tire_dynamics_combined_slip_" .. index
    )

    local energy = loadFirst(nil,
        "ngp_tc_energy_" .. index,
        "ngp_tcr_energy_push_" .. index,
        "ngp_tire_slip_energy_" .. index,
        "ngp_tire_carcass_heat_seed_" .. index
    )

    return abs(sr), abs(sa), abs(esr), abs(esa), combined, energy, srKey ~= nil or saKey ~= nil
end

local function readTyreTemperature(index)
    local value, key = loadFirst(nil,
        "ngp_tyre_temp_" .. index,
        "ngp_tire_temp_" .. index,
        "ngp_thermal_tire_temp_" .. index,
        "ngp_thermal_tyre_temp_" .. index
    )

    if key ~= nil then M.state.thermalLinked = true end
    return safeNumber(value, M.params.ambientTemp)
end

local function getWheelLoad(index, wheel)
    local direct = nil
    if wheel then
        direct = safeField(wheel, "load", nil)
        if direct == nil then direct = safeField(wheel, "loadK", nil) end
        if direct == nil then direct = nil end
        if direct == nil then direct = nil end
        if direct == nil then direct = nil end
        if direct ~= nil then direct = abs(safeNumber(direct, M.params.loadReference)) end
    end

    local integrated, key = loadFirst(nil,
        "ngp_tire_load_input_" .. index,
        "ngp_tdyn_load_" .. index,
        "ngp_tc_load_" .. index,
        "ngp_cp_load_" .. index,
        "ngp_tcr_load_" .. index,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index,
        "ngp_tire_carcass_load_" .. index,
        "ngp_lp_load_" .. index,
        "ngp_load_path_load_" .. index
    )

    if key ~= nil then
        M.state.loadLinked = true
        integrated = abs(safeNumber(integrated, direct or M.params.loadReference))
        if direct ~= nil then
            return direct * 0.65 + integrated * 0.35
        end
        return integrated
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        M.state.loadLinked = true
        return math.max(0.0, M.params.loadReference + safeNumber(dlt, 0.0))
    end

    return direct or M.params.loadReference
end

local function getAxleBias(index)
    if index <= 1 then
        return clamp(loadFirst(0.50, "ngp_load_front", "ngp_front_bias", "ngp_weight_front_bias"), 0.0, 1.0)
    end
    return clamp(loadFirst(0.50, "ngp_load_rear", "ngp_rear_bias", "ngp_weight_rear_bias"), 0.0, 1.0)
end

local function readContactPatch(index, wheelState)
    local contactGrip, gripKey = loadFirst(nil,
        "ngp_cp_grip_" .. index,
        "ngp_tire_effective_grip_" .. index,
        "ngp_tcr_effective_grip_" .. index,
        "ngp_tc_grip_" .. index
    )

    local contactLat, latKey = loadFirst(nil,
        "ngp_cp_lat_" .. index,
        "ngp_tire_contact_lat_" .. index
    )

    local contactLong, longKey = loadFirst(nil,
        "ngp_cp_long_" .. index,
        "ngp_tire_contact_long_" .. index
    )

    if gripKey ~= nil or latKey ~= nil or longKey ~= nil then
        M.state.contactLinked = true
    end

    wheelState.contactGrip = clamp(safeNumber(contactGrip, 1.0), 0.40, 1.25)
    wheelState.contactLat = clamp(safeNumber(contactLat, 1.0), 0.40, 1.20)
    wheelState.contactLong = clamp(safeNumber(contactLong, 1.0), 0.40, 1.20)
end

local function readContactResponse(index, wheelState)
    local quality, qKey = loadFirst(nil,
        "ngp_tcr_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_contact_quality_" .. index,
        "ngp_tc_contact_" .. index
    )

    local effectiveGrip, gKey = loadFirst(nil,
        "ngp_tcr_effective_grip_" .. index,
        "ngp_tire_effective_grip_" .. index,
        "ngp_tc_grip_" .. index
    )

    local contactLoss, lKey = loadFirst(nil,
        "ngp_tcr_contact_loss_" .. index,
        "ngp_tire_contact_loss_" .. index,
        "ngp_contact_loss_" .. index
    )

    local responseScale, rKey = loadFirst(nil,
        "ngp_tcr_relax_scale_" .. index,
        "ngp_tire_response_scale_" .. index,
        "ngp_tire_relax_scale_" .. index
    )

    local complianceDelay, cKey = loadFirst(nil,
        "ngp_tire_response_delay",
        "ngp_tyre_response_delay",
        "ngp_tcomp_response_delay",
        "ngp_tire_compliance_response_delay"
    )

    local complianceInput, ciKey = loadFirst(nil,
        "ngp_tcr_compliance_input_" .. index,
        "ngp_compliance_contact_input_" .. index,
        "ngp_tc_comp_input_" .. index,
        "ngp_tcomp_input_" .. index
    )

    if qKey or gKey or lKey or rKey or cKey or ciKey then
        M.state.contactLinked = true
        M.state.complianceLinked = M.state.complianceLinked or cKey ~= nil or ciKey ~= nil
    end

    wheelState.contactQuality = clamp(safeNumber(quality, 1.0), 0.0, 1.2)
    wheelState.effectiveGrip = clamp(safeNumber(effectiveGrip, 1.0), 0.35, 1.25)
    wheelState.contactLoss = clamp(safeNumber(contactLoss, 1.0 - wheelState.contactQuality), 0.0, 1.0)
    wheelState.responseScale = clamp(safeNumber(responseScale, 1.0), 0.55, 1.55)
    wheelState.complianceDelay = clamp(safeNumber(complianceDelay, 0.0), 0.0, 1.0)
    wheelState.complianceInput = clamp(safeNumber(complianceInput, 0.0), 0.0, 1.0)
    wheelState.complianceScale = clamp(loadFirst(1.0, "ngp_tire_compliance_" .. index, "ngp_tcomp_compliance_" .. index), 0.35, 1.35)
end

local function readMemoryHop(index, wheelState)
    local memory, memKey = loadFirst(nil,
        "ngp_rubber_memory_" .. index,
        "ngp_tire_memory_" .. index,
        "ngp_memory_" .. index
    )

    local memoryGrip, mgKey = loadFirst(nil,
        "ngp_memory_grip_" .. index,
        "ngp_tire_memory_grip_" .. index
    )

    local hop, hopKey = loadFirst(nil,
        "ngp_tirehop_energy_" .. index,
        "ngp_tire_hop_energy_" .. index,
        "ngp_tire_hop_" .. index
    )

    if memKey ~= nil or mgKey ~= nil then M.state.contactLinked = M.state.contactLinked or false end
    if hopKey ~= nil then M.state.contactLinked = M.state.contactLinked or false end

    wheelState.memory = clamp(safeNumber(memory, wheelState.memory or 0.0), 0.0, 1.0)
    wheelState.memoryGrip = clamp(safeNumber(memoryGrip, 1.0 - wheelState.memory * 0.12), 0.0, 1.20)
    wheelState.hopEnergy = clamp(safeNumber(hop, 0.0), 0.0, 1.0)
end

local function readDriveAndLsd(index, wheelState)
    local rearWheel = index >= 2

    local torque, torqueKey = loadFirst(nil,
        "ngp_drive_torque",
        "ngp_drivetrain_torque",
        "ngp_dt_torque",
        "ngp_diff_input_torque_nm"
    )

    local lsdLock, lsdKey = loadFirst(nil,
        "ngp_lsd_lock",
        "ngp_diff_lock"
    )

    local lsdDiff, diffKey = loadFirst(nil,
        "ngp_lsd_diff",
        "ngp_diff_diff",
        "ngp_lsd_signed_diff"
    )

    local shaftTwist, shaftKey = loadFirst(nil,
        "ngp_shaft_twist",
        "ngp_windup_twist",
        "ngp_driveline_twist"
    )

    local driveLash, lashKey = loadFirst(nil,
        "ngp_drive_lash",
        "ngp_driveline_lash",
        "ngp_windup_lash"
    )

    if torqueKey ~= nil or shaftKey ~= nil or lashKey ~= nil then M.state.drivetrainLinked = true end
    if lsdKey ~= nil or diffKey ~= nil then M.state.lsdLinked = true end

    if rearWheel then
        wheelState.driveTorque = safeNumber(torque, 0.0)
        wheelState.lsdLock = clamp(safeNumber(lsdLock, 0.0), 0.0, 1.0)
        wheelState.lsdDiff = abs(safeNumber(lsdDiff, 0.0))
        wheelState.shaftTwist = safeNumber(shaftTwist, 0.0)
        wheelState.driveLash = safeNumber(driveLash, 0.0)
    else
        wheelState.driveTorque = 0.0
        wheelState.lsdLock = 0.0
        wheelState.lsdDiff = 0.0
        wheelState.shaftTwist = 0.0
        wheelState.driveLash = 0.0
    end
end

local function readArmGeometry(index, wheelState)
    local camber, camberKey = loadFirst(nil,
        "ngp_control_arm_camber_" .. index,
        "ngp_arm_camber_" .. index,
        "ngp_ca_camber_" .. index,
        "ngp_compliance_virtual_camber_" .. index,
        "ngp_virtual_camber_" .. index
    )

    local toe, toeKey = loadFirst(nil,
        "ngp_control_arm_toe_" .. index,
        "ngp_arm_toe_" .. index,
        "ngp_ca_toe_" .. index,
        "ngp_compliance_virtual_toe_" .. index,
        "ngp_virtual_toe_" .. index
    )

    if camberKey ~= nil or toeKey ~= nil then M.state.armLinked = true end

    wheelState.armCamber = safeNumber(camber, 0.0)
    wheelState.armToe = safeNumber(toe, 0.0)
end

local function readCarcassRecoveryRoad(index, wheelState)
    local support, supportKey = loadFirst(nil,
        "ngp_carcass_support_" .. index,
        "ngp_tire_carcass_support_" .. index
    )

    local gate, gateKey = loadFirst(nil,
        "ngp_carcass_grip_gate_" .. index,
        "ngp_tire_carcass_grip_gate_" .. index
    )

    local deformation, defKey = loadFirst(nil,
        "ngp_tire_deformation_" .. index,
        "ngp_tire_carcass_deformation_" .. index,
        "ngp_carcass_deformation_" .. index
    )

    local delay, delayKey = loadFirst(nil,
        "ngp_tire_contact_delay_" .. index,
        "ngp_contact_delay_" .. index
    )

    local recoveryBias, rbKey = loadFirst(nil,
        "ngp_carcass_recovery_bias_" .. index,
        "ngp_tire_carcass_recovery_bias_" .. index
    )

    local heatSeed, hsKey = loadFirst(nil,
        "ngp_carcass_heat_seed_" .. index,
        "ngp_tire_carcass_heat_seed_" .. index
    )

    if supportKey or gateKey or defKey or delayKey or rbKey or hsKey then
        M.state.carcassLinked = true
    end

    wheelState.carcassSupport = clamp(safeNumber(support, 1.0), 0.0, 1.2)
    wheelState.carcassGripGate = clamp(safeNumber(gate, 1.0), 0.0, 1.2)
    wheelState.carcassDeformation = clamp(safeNumber(deformation, 0.0), 0.0, 1.0)
    wheelState.carcassDelay = clamp(safeNumber(delay, 0.0), 0.0, 1.0)
    wheelState.carcassRecoveryBias = clamp(safeNumber(recoveryBias, 0.0), 0.0, 1.0)
    wheelState.carcassHeatSeed = clamp(safeNumber(heatSeed, 0.0), 0.0, 1.0)

    local recoveryRate, rrKey = loadFirst(nil,
        "ngp_slip_recovery_rate_" .. index,
        "ngp_recovery_rate_" .. index
    )

    local gripReturn, grKey = loadFirst(nil,
        "ngp_slip_grip_return_" .. index,
        "ngp_grip_return_" .. index
    )

    local snap, snapKey = loadFirst(nil,
        "ngp_slip_snap_risk_" .. index,
        "ngp_snap_risk_" .. index
    )

    local bite, biteKey = loadFirst(nil,
        "ngp_slip_bite_" .. index,
        "ngp_recovery_bite_" .. index
    )

    local dirty, dirtyKey = loadFirst(nil,
        "ngp_slip_dirty_return_" .. index,
        "ngp_dirty_return_" .. index
    )

    local trust, trustKey = loadFirst(nil,
        "ngp_recovery_trust_" .. index,
        "ngp_slip_recovery_trust_" .. index
    )

    if rrKey or grKey or snapKey or biteKey or dirtyKey or trustKey then
        M.state.recoveryLinked = true
    end

    wheelState.recoveryRate = clamp(safeNumber(recoveryRate, 0.0), 0.0, 1.0)
    wheelState.recoveryGripReturn = clamp(safeNumber(gripReturn, 1.0), 0.0, 1.2)
    wheelState.recoverySnapRisk = clamp(safeNumber(snap, 0.0), 0.0, 1.0)
    wheelState.recoveryBite = clamp(safeNumber(bite, 0.0), 0.0, 1.0)
    wheelState.recoveryDirtyReturn = clamp(safeNumber(dirty, 0.0), 0.0, 1.0)
    wheelState.recoveryTrust = clamp(safeNumber(trust, 1.0), 0.0, 1.2)

    local severity, sevKey = loadFirst(nil,
        "ngp_road_input_severity_" .. index,
        "ngp_rii_severity_" .. index,
        "ngp_road_severity_" .. index
    )

    local shock, shockKey = loadFirst(nil,
        "ngp_road_input_shock_" .. index,
        "ngp_road_impact_" .. index,
        "ngp_rii_shock_" .. index
    )

    local surfaceLimit, surfKey = loadFirst(nil,
        "ngp_road_surface_limit_" .. index,
        "ngp_rii_surface_limit_" .. index
    )

    if sevKey or shockKey or surfKey then M.state.roadLinked = true end

    wheelState.roadSeverity = clamp(safeNumber(severity, 0.0), 0.0, 1.5)
    wheelState.roadShock = clamp(safeNumber(shock, 0.0), 0.0, 1.5)
    wheelState.roadSurfaceLimit = clamp(safeNumber(surfaceLimit, 0.0), 0.0, 1.0)

    local pathLoss, lossKey = loadFirst(nil,
        "ngp_load_path_loss_" .. index,
        "ngp_lp_path_loss_" .. index,
        "ngp_lp_loss_" .. index,
        "ngp_susp_path_loss_" .. index
    )

    local pathWork, workKey = loadFirst(nil,
        "ngp_load_path_work_" .. index,
        "ngp_lp_work_" .. index,
        "ngp_lp_vertical_work_" .. index
    )

    if lossKey or workKey then M.state.loadPathLinked = true end

    wheelState.loadPathLoss = clamp(safeNumber(pathLoss, 0.0), 0.0, 1.0)
    wheelState.loadPathWork = clamp(safeNumber(pathWork, 0.0), 0.0, 1.0)

    local thermalStress, thermalKey = loadFirst(nil,
        "ngp_thermal_stress_" .. index,
        "ngp_tire_thermal_stress_" .. index,
        "ngp_virtual_thermal_stress"
    )

    if thermalKey then M.state.thermalLinked = true end
    wheelState.thermalStress = clamp(safeNumber(thermalStress, 0.0), 0.0, 1.0)
end

--============================================================
-- Core model
--============================================================

local function normalizeSlipAngle(slipAngle)
    local angle = abs(slipAngle)
    if angle <= math.pi then
        return angle / math.max(M.params.slipAngleRefRad, 0.001)
    end
    return angle / math.max(M.params.slipAngleRef, 0.001)
end

local function calculateSlipMetrics(slipRatio, slipAngle, combinedInput)
    local longNorm = abs(slipRatio) / math.max(M.params.slipRatioRef, 0.001)
    local latNorm = normalizeSlipAngle(slipAngle)

    local combined = safeNumber(combinedInput, -1.0)
    if combined < 0.0 then
        local lw = longNorm * M.params.combinedSlipLongWeight
        local aw = latNorm * M.params.combinedSlipLatWeight
        combined = math.sqrt(lw * lw + aw * aw)
    end

    combined = clamp(combined, 0.0, 3.0)

    local over = math.max(combined - 1.0, 0.0)
    local slipGrip =
        M.params.gripAvailableBase
        -
        (over * M.params.postLimitGripDrop)
        /
        (1.0 + over * M.params.gripLimitSoftness)

    slipGrip = clamp(slipGrip, 0.45, 1.08)

    local limit = clamp(combined, 0.0, M.params.maxLimit)
    local normalizedSlip = clamp((longNorm + latNorm) * 0.5, 0.0, 3.0)

    return slipGrip, combined, limit, normalizedSlip, longNorm, latNorm
end

local function updateSlipEnergy(currentEnergy, combinedSlip, inputEnergy, wheelState, dt)
    local energy = safeNumber(currentEnergy, 0.0)

    local build =
        math.max(combinedSlip - M.params.slipEnergyBuildStart, 0.0)
        * dt
        * M.params.slipEnergyBuildGain

    local decay =
        energy
        * dt
        * M.params.slipEnergyDecayGain

    energy = energy + build - decay

    if inputEnergy ~= nil then
        energy = math.max(energy, clamp(safeNumber(inputEnergy, 0.0), 0.0, 1.0))
    end

    energy = energy + (wheelState.carcassHeatSeed or 0.0) * 0.02
    energy = energy + (wheelState.roadShock or 0.0) * 0.015

    return clamp(energy, 0.0, 1.0)
end

local function updateRubberMemory(index, currentMemory, limit, wheelState, dt)
    local rubberMemory = clamp(currentMemory, 0.0, 1.0)

    local extraStress =
        (wheelState.slipEnergy or 0.0) * 0.12
        + (wheelState.carcassDeformation or 0.0) * 0.08
        + (wheelState.thermalStress or 0.0) * 0.08

    local build =
        math.max(limit - M.params.memoryBuildStart, 0.0)
        * dt
        * M.params.memoryBuildGain
        + extraStress * dt * 0.12

    local decay =
        rubberMemory
        * dt
        * M.params.memoryDecayGain
        * clamp((wheelState.contactQuality or 1.0), 0.25, 1.0)

    rubberMemory = clamp(rubberMemory + build - decay, 0.0, 1.0)

    safeStore("ngp_rubber_memory_" .. index, rubberMemory)
    safeStore("ngp_tire_memory_" .. index, rubberMemory)

    return rubberMemory
end

local function calculateTemperatureGrip(tyreTemp)
    local tempNorm =
        clamp(
            (tyreTemp - M.params.ambientTemp) / math.max(M.params.tempRange, 1.0),
            0.0,
            1.0
        )

    local heatGrip = 1.0
    if tyreTemp < M.params.warmTemp then
        heatGrip = M.params.coldGripBase + tempNorm * M.params.coldGripGain
    elseif tyreTemp < M.params.optimumTemp then
        heatGrip = M.params.optimumGrip
    else
        heatGrip = M.params.optimumGrip - (tyreTemp - M.params.optimumTemp) * M.params.overheatGripLoss
    end

    heatGrip = heatGrip * (1.0 - clamp((tyreTemp - M.params.optimumTemp) / 180.0, 0.0, 1.0) * (M.params.thermalStressGripLoss * 0.5))
    heatGrip = clamp(heatGrip, M.params.minTempGrip, M.params.maxTempGrip)

    return heatGrip, tempNorm
end

local function calculateLoadGripScale(load)
    local loadScale =
        clamp(
            safeNumber(load, M.params.loadReference) / math.max(M.params.loadReference, 1.0),
            0.0,
            2.0
        )

    local gripScale =
        1.0
        + (loadScale - 1.0) * M.params.loadGripGain

    return clamp(gripScale, M.params.minLoadGripScale, M.params.maxLoadGripScale), loadScale
end

local function calculateRubberLag(tempNorm, wheelState)
    local baseLag =
        (0.85 + tempNorm * 0.30 - 1.0) * 0.15

    local driveDelay =
        abs(wheelState.shaftTwist or 0.0) * M.params.shaftTwistDelayGain
        + abs(wheelState.driveLash or 0.0) * M.params.driveLashDelayGain

    local rootDelay =
        (wheelState.complianceDelay or 0.0) * 0.25
        + (wheelState.carcassDelay or 0.0) * 0.18
        + (wheelState.carcassRecoveryBias or 0.0) * 0.08
        + (wheelState.recoveryDirtyReturn or 0.0) * 0.06

    return clamp(baseLag + driveDelay + rootDelay, -0.50, 2.00)
end

local function applyExternalModifiers(index, grip, heatGrip, rubberMemory, wheelState)
    local contactCombined =
        1.0
        + (wheelState.contactGrip - 1.0) * M.params.contactGripGain

    contactCombined =
        contactCombined
        * (1.0 - (1.0 - wheelState.contactLat) * M.params.contactLatGain)
        * (1.0 - (1.0 - wheelState.contactLong) * M.params.contactLongGain)

    local responseGrip =
        1.0
        + (wheelState.responseScale - 1.0) * M.params.responseScaleGripGain
        - wheelState.contactLoss * M.params.contactQualityGripGain
        - wheelState.complianceDelay * M.params.complianceDelayGripLoss
        - wheelState.complianceInput * 0.04

    responseGrip = clamp(responseGrip, 0.70, 1.12)

    local geometryLoss =
        abs(wheelState.armCamber or 0.0) * M.params.armCamberGripLossGain
        + abs(wheelState.armToe or 0.0) * M.params.armToeGripLossGain

    wheelState.geometryLoss = clamp(geometryLoss, 0.0, 0.20)

    local carcassScale =
        1.0
        + (wheelState.carcassSupport - 1.0) * M.params.carcassSupportGripGain
        + (wheelState.carcassGripGate - 1.0) * M.params.carcassGateGripGain
        - wheelState.carcassDelay * M.params.carcassDelayGripLoss

    carcassScale = clamp(carcassScale, 0.65, 1.12)

    local recoveryScale =
        1.0
        + (wheelState.recoveryGripReturn - 1.0) * M.params.slipRecoveryGripGain
        - wheelState.recoverySnapRisk * 0.05
        - wheelState.recoveryDirtyReturn * 0.04

    recoveryScale = clamp(recoveryScale, 0.72, 1.10)

    local roadScale =
        1.0
        - wheelState.roadShock * M.params.roadShockGripLoss
        - wheelState.loadPathLoss * M.params.loadPathLossGripLoss
        - wheelState.thermalStress * M.params.thermalStressGripLoss

    roadScale = clamp(roadScale, 0.72, 1.05)

    local finalGrip =
        grip
        * wheelState.memoryGrip
        * heatGrip
        * contactCombined
        * wheelState.effectiveGrip
        * responseGrip
        * carcassScale
        * recoveryScale
        * roadScale
        * (1.0 - rubberMemory * M.params.rubberMemoryGripLoss)
        * (1.0 - wheelState.hopEnergy * M.params.hopGripLoss)
        * (1.0 - wheelState.geometryLoss)
        * (1.0 - (wheelState.slipEnergy or 0.0) * M.params.slipEnergyGripLoss)

    return finalGrip
end

local function applyDriveLimit(index, limit, wheelState)
    if index < 2 then return limit end

    local driveLimit =
        abs(wheelState.driveTorque or 0.0) * M.params.driveTorqueSlipGain
        + wheelState.lsdLock * M.params.lsdLockRearLimitGain
        + clamp(wheelState.lsdDiff / 8.0, 0.0, 1.0) * M.params.lsdDiffRearLimitGain
        + abs(wheelState.armToe or 0.0) * M.params.controlArmLimitGain

    return limit + driveLimit
end

local function applyContactLimit(limit, wheelState)
    local contactLimit =
        wheelState.contactLoss * M.params.contactLossLimitGain
        + (wheelState.responseScale - 1.0) * M.params.responseScaleLimitGain
        + wheelState.complianceDelay * M.params.complianceDelayLimitLoss
        + wheelState.carcassDeformation * M.params.carcassDeformationLimitGain
        + wheelState.carcassRecoveryBias * M.params.recoveryBiasLimitGain
        + wheelState.recoverySnapRisk * M.params.slipRecoverySnapLimitGain
        + wheelState.roadSeverity * M.params.roadSeverityLimitGain
        + wheelState.roadSurfaceLimit * 0.05
        + wheelState.thermalStress * M.params.thermalStressLimitGain

    return limit + contactLimit
end

local function decayWheel(index, dt)
    local wheelState = M.state.wheels[index]

    wheelState.grip = lowPass(wheelState.grip, 1.0, M.params.decayTau, dt)
    wheelState.limit = lowPass(wheelState.limit, 0.0, M.params.decayTau, dt)
    wheelState.memory = lowPass(wheelState.memory, 0.0, M.params.decayTau * 4.0, dt)
    wheelState.tempGrip = lowPass(wheelState.tempGrip, 1.0, M.params.decayTau, dt)
    wheelState.slipRatio = lowPass(wheelState.slipRatio, 0.0, M.params.decayTau, dt)
    wheelState.slipAngle = lowPass(wheelState.slipAngle, 0.0, M.params.decayTau, dt)
    wheelState.normalizedSlip = lowPass(wheelState.normalizedSlip, 0.0, M.params.decayTau, dt)
    wheelState.combinedSlip = lowPass(wheelState.combinedSlip, 0.0, M.params.decayTau, dt)
    wheelState.slipEnergy = lowPass(wheelState.slipEnergy, 0.0, M.params.decayTau, dt)
    wheelState.effectiveSlipRatio = 0.0
    wheelState.effectiveSlipAngle = 0.0
    wheelState.load = lowPass(wheelState.load, M.params.defaultLoad, M.params.decayTau, dt)
    wheelState.loadScale = lowPass(wheelState.loadScale, 1.0, M.params.decayTau, dt)
    wheelState.bias = lowPass(wheelState.bias, 0.5, M.params.decayTau, dt)
    wheelState.memoryGrip = lowPass(wheelState.memoryGrip, 1.0, M.params.decayTau, dt)
    wheelState.hopEnergy = lowPass(wheelState.hopEnergy, 0.0, M.params.decayTau, dt)
    wheelState.rubberLag = lowPass(wheelState.rubberLag, 0.0, M.params.decayTau, dt)
    wheelState.tyreTemp = lowPass(wheelState.tyreTemp, M.params.ambientTemp, M.params.decayTau * 8.0, dt)
    wheelState.tempNorm = lowPass(wheelState.tempNorm, 0.0, M.params.decayTau, dt)
    wheelState.contactGrip = lowPass(wheelState.contactGrip, 1.0, M.params.decayTau, dt)
    wheelState.contactLat = lowPass(wheelState.contactLat, 1.0, M.params.decayTau, dt)
    wheelState.contactLong = lowPass(wheelState.contactLong, 1.0, M.params.decayTau, dt)
    wheelState.contactQuality = lowPass(wheelState.contactQuality, 1.0, M.params.decayTau, dt)
    wheelState.effectiveGrip = lowPass(wheelState.effectiveGrip, 1.0, M.params.decayTau, dt)
    wheelState.contactLoss = lowPass(wheelState.contactLoss, 0.0, M.params.decayTau, dt)
    wheelState.responseScale = lowPass(wheelState.responseScale, 1.0, M.params.decayTau, dt)
    wheelState.complianceDelay = lowPass(wheelState.complianceDelay, 0.0, M.params.decayTau, dt)
    wheelState.complianceInput = lowPass(wheelState.complianceInput, 0.0, M.params.decayTau, dt)
    wheelState.driveTorque = 0.0
    wheelState.lsdLock = 0.0
    wheelState.lsdDiff = 0.0
    wheelState.shaftTwist = 0.0
    wheelState.driveLash = 0.0
    wheelState.armCamber = 0.0
    wheelState.armToe = 0.0
    wheelState.geometryLoss = lowPass(wheelState.geometryLoss, 0.0, M.params.decayTau, dt)
    wheelState.carcassSupport = lowPass(wheelState.carcassSupport, 1.0, M.params.decayTau, dt)
    wheelState.carcassGripGate = lowPass(wheelState.carcassGripGate, 1.0, M.params.decayTau, dt)
    wheelState.carcassDeformation = lowPass(wheelState.carcassDeformation, 0.0, M.params.decayTau, dt)
    wheelState.carcassDelay = lowPass(wheelState.carcassDelay, 0.0, M.params.decayTau, dt)
    wheelState.recoveryGripReturn = lowPass(wheelState.recoveryGripReturn, 1.0, M.params.decayTau, dt)
    wheelState.recoverySnapRisk = lowPass(wheelState.recoverySnapRisk, 0.0, M.params.decayTau, dt)
    wheelState.roadSeverity = lowPass(wheelState.roadSeverity, 0.0, M.params.decayTau, dt)
    wheelState.roadShock = lowPass(wheelState.roadShock, 0.0, M.params.decayTau, dt)
    wheelState.loadPathLoss = lowPass(wheelState.loadPathLoss, 0.0, M.params.decayTau, dt)
    wheelState.thermalStress = lowPass(wheelState.thermalStress, 0.0, M.params.decayTau, dt)
    wheelState.finalGripRaw = wheelState.grip
    wheelState.active = false
end

local function updateWheel(index, wheel, dt)
    local wheelState = M.state.wheels[index]
    local hasInput = false

    readContactPatch(index, wheelState)
    readContactResponse(index, wheelState)
    readMemoryHop(index, wheelState)
    readDriveAndLsd(index, wheelState)
    readArmGeometry(index, wheelState)
    readCarcassRecoveryRoad(index, wheelState)

    local slipRatio, slipAngle, effectiveSlipRatio, effectiveSlipAngle, combinedSlipInput, slipEnergyInput, slipLinked =
        readSlip(index, wheel)

    local load = getWheelLoad(index, wheel)
    hasInput = wheel ~= nil or slipLinked or M.state.contactLinked or M.state.loadLinked or M.state.carcassLinked or M.state.recoveryLinked

    if not hasInput then
        decayWheel(index, dt)
        return
    end

    wheelState.active = wheel ~= nil or M.state.storeOnly
    wheelState.slipRatio = slipRatio
    wheelState.slipAngle = slipAngle
    wheelState.effectiveSlipRatio = effectiveSlipRatio
    wheelState.effectiveSlipAngle = effectiveSlipAngle

    local grip, combinedSlip, limit, normalizedSlip =
        calculateSlipMetrics(effectiveSlipRatio, effectiveSlipAngle, combinedSlipInput)

    wheelState.normalizedSlip = normalizedSlip
    wheelState.combinedSlip = combinedSlip
    wheelState.slipEnergy =
        updateSlipEnergy(
            wheelState.slipEnergy,
            combinedSlip,
            slipEnergyInput,
            wheelState,
            dt
        )

    local bias = getAxleBias(index)
    wheelState.bias = bias

    local loadGripScale, loadScale = calculateLoadGripScale(load)
    wheelState.load = load
    wheelState.loadScale = loadScale

    local biasFactor =
        1.0
        + (bias - 0.5) * 0.20

    grip =
        grip
        * clamp(biasFactor, 0.84, 1.16)
        * loadGripScale

    local rubberMemory =
        updateRubberMemory(
            index,
            wheelState.memory,
            limit,
            wheelState,
            dt
        )

    local tyreTemp = readTyreTemperature(index)
    local heatGrip, tempNorm = calculateTemperatureGrip(tyreTemp)

    local finalGrip =
        applyExternalModifiers(
            index,
            grip,
            heatGrip,
            rubberMemory,
            wheelState
        )

    limit =
        (limit + rubberMemory * M.params.rubberMemoryLimitGain)
        * clamp(0.85 + tempNorm * 0.30, 0.75, 1.15)

    limit = applyDriveLimit(index, limit, wheelState)
    limit = applyContactLimit(limit, wheelState)

    wheelState.finalGripRaw = finalGrip
    wheelState.grip = clamp(finalGrip, M.params.minGrip, M.params.maxGrip)
    wheelState.limit = clamp(limit, 0.0, M.params.maxLimit)
    wheelState.memory = rubberMemory
    wheelState.tempGrip = heatGrip
    wheelState.rubberLag = calculateRubberLag(tempNorm, wheelState)
    wheelState.tyreTemp = tyreTemp
    wheelState.tempNorm = tempNorm
end

--============================================================
-- Export
--============================================================

local function exportWheel(index, wheelState)
    safeStore("ngp_tire_grip_" .. index, wheelState.grip or 1.0)
    safeStore("ngp_tyre_grip_" .. index, wheelState.grip or 1.0)
    safeStore("ngp_tire_limit_" .. index, wheelState.limit or 0.0)
    safeStore("ngp_tyre_limit_" .. index, wheelState.limit or 0.0)
    safeStore("ngp_rubber_lag_" .. index, wheelState.rubberLag or 0.0)
    safeStore("ngp_tire_norm_slip_" .. index, wheelState.normalizedSlip or 0.0)
    safeStore("ngp_tdyn_combined_slip_" .. index, wheelState.combinedSlip or 0.0)
    safeStore("ngp_tire_dynamics_combined_slip_" .. index, wheelState.combinedSlip or 0.0)
    safeStore("ngp_tdyn_slip_energy_" .. index, wheelState.slipEnergy or 0.0)
    safeStore("ngp_tire_dynamics_slip_energy_" .. index, wheelState.slipEnergy or 0.0)

    -- Short compatibility keys.
    safeStore("ngp_td_grip_" .. index, wheelState.grip or 1.0)
    safeStore("ngp_td_limit_" .. index, wheelState.limit or 0.0)
    safeStore("ngp_td_combined_" .. index, wheelState.combinedSlip or 0.0)
    safeStore("ngp_td_energy_" .. index, wheelState.slipEnergy or 0.0)
    safeStore("ngp_td_lag_" .. index, wheelState.rubberLag or 0.0)

    if not M.state.debugStoreNow then return end

    safeStore("ngp_tire_temp_grip_" .. index, wheelState.tempGrip or 1.0)
    safeStore("ngp_tyre_temp_grip_" .. index, wheelState.tempGrip or 1.0)
    safeStore("ngp_tire_slip_ratio_" .. index, wheelState.slipRatio or 0.0)
    safeStore("ngp_tire_slip_angle_" .. index, wheelState.slipAngle or 0.0)
    safeStore("ngp_tdyn_effective_slip_ratio_" .. index, wheelState.effectiveSlipRatio or 0.0)
    safeStore("ngp_tdyn_effective_slip_angle_" .. index, wheelState.effectiveSlipAngle or 0.0)
    safeStore("ngp_tire_effective_slip_ratio_" .. index, wheelState.effectiveSlipRatio or 0.0)
    safeStore("ngp_tire_effective_slip_angle_" .. index, wheelState.effectiveSlipAngle or 0.0)
    safeStore("ngp_tdyn_load_" .. index, wheelState.load or M.params.defaultLoad)
    safeStore("ngp_tdyn_load_scale_" .. index, wheelState.loadScale or 1.0)
    safeStore("ngp_tire_memory_out_" .. index, wheelState.memory or 0.0)
    safeStore("ngp_tire_hop_energy_read_" .. index, wheelState.hopEnergy or 0.0)
    safeStore("ngp_tire_contact_grip_read_" .. index, wheelState.contactGrip or 1.0)
    safeStore("ngp_tire_contact_lat_read_" .. index, wheelState.contactLat or 1.0)
    safeStore("ngp_tire_contact_long_read_" .. index, wheelState.contactLong or 1.0)
    safeStore("ngp_tdyn_contact_quality_" .. index, wheelState.contactQuality or 1.0)
    safeStore("ngp_tdyn_effective_grip_" .. index, wheelState.effectiveGrip or 1.0)
    safeStore("ngp_tdyn_contact_loss_" .. index, wheelState.contactLoss or 0.0)
    safeStore("ngp_tdyn_response_scale_" .. index, wheelState.responseScale or 1.0)
    safeStore("ngp_tdyn_compliance_delay_" .. index, wheelState.complianceDelay or 0.0)
    safeStore("ngp_tdyn_drive_torque_" .. index, wheelState.driveTorque or 0.0)
    safeStore("ngp_tdyn_lsd_lock_" .. index, wheelState.lsdLock or 0.0)
    safeStore("ngp_tdyn_lsd_diff_" .. index, wheelState.lsdDiff or 0.0)
    safeStore("ngp_tdyn_arm_camber_" .. index, wheelState.armCamber or 0.0)
    safeStore("ngp_tdyn_arm_toe_" .. index, wheelState.armToe or 0.0)
    safeStore("ngp_tdyn_geometry_loss_" .. index, wheelState.geometryLoss or 0.0)
    safeStore("ngp_tdyn_carcass_support_" .. index, wheelState.carcassSupport or 1.0)
    safeStore("ngp_tdyn_carcass_gate_" .. index, wheelState.carcassGripGate or 1.0)
    safeStore("ngp_tdyn_recovery_snap_" .. index, wheelState.recoverySnapRisk or 0.0)
    safeStore("ngp_tdyn_road_severity_" .. index, wheelState.roadSeverity or 0.0)
    safeStore("ngp_tdyn_load_path_loss_" .. index, wheelState.loadPathLoss or 0.0)
end

local function exportGlobal()
    safeStore("ngp_tire_dynamics_status", M.state.status or "UNKNOWN")
    safeStore("ngp_tire_dynamics_update_count", M.state.updateCount or 0)
    safeStore("ngp_tire_dynamics_wheels_valid", M.state.wheelsValid and 1 or 0)
    safeStore("ngp_tire_dynamics_store_only", M.state.storeOnly and 1 or 0)
    safeStore("ngp_tire_dynamics_avg_grip", M.state.avgGrip or 1.0)
    safeStore("ngp_tire_dynamics_avg_limit", M.state.avgLimit or 0.0)

    safeStore("ngp_td_avg_grip", M.state.avgGrip or 1.0)
    safeStore("ngp_td_avg_limit", M.state.avgLimit or 0.0)
    safeStore("ngp_td_avg_combined", M.state.avgCombinedSlip or 0.0)
    safeStore("ngp_td_avg_energy", M.state.avgSlipEnergy or 0.0)
    safeStore("ngp_td_min_grip", M.state.minGripLive or 1.0)
    safeStore("ngp_td_max_limit", M.state.maxLimitLive or 0.0)

    if not M.state.debugStoreNow then return end

    safeStore("ngp_tire_dynamics_avg_memory", M.state.avgMemory or 0.0)
    safeStore("ngp_tire_dynamics_avg_response_delay", M.state.avgResponseDelay or 0.0)
    safeStore("ngp_tire_dynamics_avg_combined_slip", M.state.avgCombinedSlip or 0.0)
    safeStore("ngp_tire_dynamics_avg_slip_energy", M.state.avgSlipEnergy or 0.0)

    safeStore("ngp_tire_dynamics_drivetrain_linked", M.state.drivetrainLinked and 1 or 0)
    safeStore("ngp_tire_dynamics_lsd_linked", M.state.lsdLinked and 1 or 0)
    safeStore("ngp_tire_dynamics_contact_linked", M.state.contactLinked and 1 or 0)
    safeStore("ngp_tire_dynamics_arm_linked", M.state.armLinked and 1 or 0)
    safeStore("ngp_tire_dynamics_compliance_linked", M.state.complianceLinked and 1 or 0)
    safeStore("ngp_tire_dynamics_carcass_linked", M.state.carcassLinked and 1 or 0)
    safeStore("ngp_tire_dynamics_recovery_linked", M.state.recoveryLinked and 1 or 0)
    safeStore("ngp_tire_dynamics_road_linked", M.state.roadLinked and 1 or 0)
    safeStore("ngp_tire_dynamics_load_path_linked", M.state.loadPathLinked and 1 or 0)
    safeStore("ngp_tire_dynamics_thermal_linked", M.state.thermalLinked and 1 or 0)
    safeStore("ngp_tire_dynamics_load_linked", M.state.loadLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i, M.state.wheels[i])
    end
    exportGlobal()
end

--============================================================
-- Update
--============================================================

function M.init()
    M.state.status = "INIT"
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

    car = car or safeGetCar()
    local wheels = getWheels(car)

    M.state.wheelsValid = wheels ~= nil
    M.state.storeOnly = wheels == nil

    if not car then
        M.state.status = "STORE ONLY"
    elseif not wheels then
        M.state.status = "NO WHEELS / STORE ONLY"
    else
        M.state.status = "RUNNING"
    end

    M.state.drivetrainLinked = false
    M.state.lsdLinked = false
    M.state.contactLinked = false
    M.state.armLinked = false
    M.state.complianceLinked = false
    M.state.carcassLinked = false
    M.state.recoveryLinked = false
    M.state.roadLinked = false
    M.state.loadPathLinked = false
    M.state.thermalLinked = false
    M.state.loadLinked = false

    local sumGrip = 0.0
    local sumLimit = 0.0
    local sumMemory = 0.0
    local sumDelay = 0.0
    local sumCombined = 0.0
    local sumEnergy = 0.0
    local minGrip = 999.0
    local maxLimit = 0.0

    for i = 0, 3 do
        local wheel = getWheel(car, i)

        updateWheel(i, wheel, dt)

        local wheelState = M.state.wheels[i]

        sumGrip = sumGrip + (wheelState.grip or 1.0)
        sumLimit = sumLimit + (wheelState.limit or 0.0)
        sumMemory = sumMemory + (wheelState.memory or 0.0)
        sumDelay = sumDelay + (wheelState.complianceDelay or 0.0)
        sumCombined = sumCombined + (wheelState.combinedSlip or 0.0)
        sumEnergy = sumEnergy + (wheelState.slipEnergy or 0.0)

        if (wheelState.grip or 1.0) < minGrip then minGrip = wheelState.grip or 1.0 end
        if (wheelState.limit or 0.0) > maxLimit then maxLimit = wheelState.limit or 0.0 end

        exportWheel(i, wheelState)
    end

    M.state.avgGrip = sumGrip * 0.25
    M.state.avgLimit = sumLimit * 0.25
    M.state.avgMemory = sumMemory * 0.25
    M.state.avgResponseDelay = sumDelay * 0.25
    M.state.avgCombinedSlip = sumCombined * 0.25
    M.state.avgSlipEnergy = sumEnergy * 0.25
    M.state.minGripLive = minGrip == 999.0 and 1.0 or minGrip
    M.state.maxLimitLive = maxLimit

    exportGlobal()
end

--============================================================
-- Public API
--============================================================

function M.getWheel(index)
    if index == nil then return M.state.wheels end
    return M.state.wheels[index]
end

function M.getGrip(index)
    local wheelState = M.state.wheels[index]
    if not wheelState then return 1.0 end
    return wheelState.grip or 1.0
end

function M.getLimit(index)
    local wheelState = M.state.wheels[index]
    if not wheelState then return 0.0 end
    return wheelState.limit or 0.0
end

function M.getMemory(index)
    local wheelState = M.state.wheels[index]
    if not wheelState then return 0.0 end
    return wheelState.memory or 0.0
end

function M.getCombinedSlip(index)
    local wheelState = M.state.wheels[index]
    if not wheelState then return 0.0 end
    return wheelState.combinedSlip or 0.0
end

function M.getSlipEnergy(index)
    local wheelState = M.state.wheels[index]
    if not wheelState then return 0.0 end
    return wheelState.slipEnergy or 0.0
end

function M.getState(index)
    if index == nil then return M.state end
    return M.state.wheels[index]
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0
        local wheelState = M.state.wheels[i] or M.state.wheels[0]

        return string.format(
            "Status %s / Count %.0f\n" ..
            "W%d Grip %.3f Limit %.3f Mem %.3f TempGrip %.3f\n" ..
            "SlipR %.3f SlipA %.3f Comb %.3f E %.3f\n" ..
            "Load %.0f Scale %.2f Bias %.3f\n" ..
            "ContactQ %.2f Eff %.2f Loss %.2f Resp %.2f\n" ..
            "LSD %.2f Drive %.3f Arm C %.3f T %.3f Hop %.3f",
            tostring(M.state.status),
            M.state.updateCount or 0,

            i,
            wheelState.grip or 1.0,
            wheelState.limit or 0.0,
            wheelState.memory or 0.0,
            wheelState.tempGrip or 1.0,

            wheelState.slipRatio or 0.0,
            wheelState.slipAngle or 0.0,
            wheelState.combinedSlip or 0.0,
            wheelState.slipEnergy or 0.0,

            wheelState.load or 0.0,
            wheelState.loadScale or 1.0,
            wheelState.bias or 0.5,

            wheelState.contactQuality or 1.0,
            wheelState.effectiveGrip or 1.0,
            wheelState.contactLoss or 0.0,
            wheelState.responseScale or 1.0,

            wheelState.lsdLock or 0.0,
            wheelState.driveTorque or 0.0,
            wheelState.armCamber or 0.0,
            wheelState.armToe or 0.0,
            wheelState.hopEnergy or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s / Store %s\n" ..
        "Grip %.3f %.3f %.3f %.3f / Avg %.3f Min %.3f\n" ..
        "Limit %.3f %.3f %.3f %.3f / Avg %.3f Max %.3f\n" ..
        "Combined %.3f %.3f %.3f %.3f / E %.3f\n" ..
        "Memory %.3f %.3f %.3f %.3f\n" ..
        "Links DT:%s LSD:%s CT:%s ARM:%s CMP:%s CAR:%s REC:%s ROAD:%s",
        tostring(M.state.status),
        M.state.updateCount or 0,
        M.state.wheelsValid and "OK" or "NIL",
        M.state.storeOnly and "YES" or "NO",

        M.state.wheels[0].grip or 1.0,
        M.state.wheels[1].grip or 1.0,
        M.state.wheels[2].grip or 1.0,
        M.state.wheels[3].grip or 1.0,
        M.state.avgGrip or 1.0,
        M.state.minGripLive or 1.0,

        M.state.wheels[0].limit or 0.0,
        M.state.wheels[1].limit or 0.0,
        M.state.wheels[2].limit or 0.0,
        M.state.wheels[3].limit or 0.0,
        M.state.avgLimit or 0.0,
        M.state.maxLimitLive or 0.0,

        M.state.wheels[0].combinedSlip or 0.0,
        M.state.wheels[1].combinedSlip or 0.0,
        M.state.wheels[2].combinedSlip or 0.0,
        M.state.wheels[3].combinedSlip or 0.0,
        M.state.avgSlipEnergy or 0.0,

        M.state.wheels[0].memory or 0.0,
        M.state.wheels[1].memory or 0.0,
        M.state.wheels[2].memory or 0.0,
        M.state.wheels[3].memory or 0.0,

        M.state.drivetrainLinked and "OK" or "NIL",
        M.state.lsdLinked and "OK" or "NIL",
        M.state.contactLinked and "OK" or "NIL",
        M.state.armLinked and "OK" or "NIL",
        M.state.complianceLinked and "OK" or "NIL",
        M.state.carcassLinked and "OK" or "NIL",
        M.state.recoveryLinked and "OK" or "NIL",
        M.state.roadLinked and "OK" or "NIL"
    )
end

return M
