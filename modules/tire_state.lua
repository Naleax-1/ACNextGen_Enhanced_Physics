---@diagnostic disable: undefined-global

--============================================================
-- ACNextGen
-- tire_state.lua
-- Phase 1 / V1.1.5 Stable
-- Tire Relaxation Length / Load Sensitivity / Slip Energy Model
--============================================================

local M = {}

--============================================================
-- Parameters
--============================================================

M.params = {
    relaxLateral = 0.18,
    relaxLongitudinal = 0.20,

    minSpeed = 0.5,
    minRelaxLength = 0.001,

    memoryLagGain = 0.20,
    complianceLagGain = 0.25,

    defaultLoad = 3000.0,
    loadReference = 3000.0,
    preferIntegratedLoad = true,

    loadRelaxGain = 0.28,
    minLoadRelaxScale = 0.72,
    maxLoadRelaxScale = 1.42,

    slipAngleRefRad = 0.174533,
    slipRatioRef = 0.12,
    combinedSlipRatioWeight = 1.00,
    combinedSlipAngleWeight = 1.00,

    slipEnergyTau = 0.120,
    slipEnergyGain = 0.70,
    slipEnergyOverStart = 0.55,
    maxCombinedSlip = 3.00,
    maxSlipEnergy = 2.50,

    contactLossLagGain = 0.18,
    contactEnergyLagGain = 0.12,
    contactQualityRecoverGain = 0.08,

    tireLimitLagGain = 0.14,
    tireGripRecoverGain = 0.06,

    tireHopLagGain = 0.16,

    armCamberLatGain = 0.06,
    armToeLongGain = 0.08,
    armLagGain = 0.04,

    driveRearLongLagGain = 0.10,
    lsdRearLongLagGain = 0.08,
    shaftRearLongLagGain = 0.06,
    windupRearLongLagGain = 0.06,

    carcassDelayLagGain = 0.14,
    carcassHysteresisLagGain = 0.08,
    carcassSupportRecoverGain = 0.06,

    recoveryLagReduceGain = 0.08,
    roadInputLagGain = 0.08,
    loadPathLagGain = 0.05,
    thermalLagGain = 0.06,

    maxLag = 2.00,
    minLag = -0.50,

    noWheelDecayTau = 0.160,
    noCarDecayTau = 0.320,

    debugStoreInterval = 0.25,
}

--============================================================
-- State
--============================================================

M.state = {
    wheels = {},

    slipAngle = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    slipRatio = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    rawSlipAngleLegacy = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    rawSlipRatioLegacy = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    load = { [0]=3000.0, [1]=3000.0, [2]=3000.0, [3]=3000.0 },
    omega = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    active = { [0]=false, [1]=false, [2]=false, [3]=false },
    valid = false,

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    speedMs = 0.0,
    speedKmh = 0.0,

    avgCombinedSlip = 0.0,
    avgSlipEnergy = 0.0,
    avgLoad = 3000.0,
    avgLag = 0.0,
    activeCount = 0,

    contactLinked = false,
    memoryLinked = false,
    complianceLinked = false,
    dynamicsLinked = false,
    hopLinked = false,
    armLinked = false,
    drivetrainLinked = false,
    lsdLinked = false,
    loadLinked = false,
    carcassLinked = false,
    recoveryLinked = false,
    roadLinked = false,
    loadPathLinked = false,
    thermalLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

for i = 0, 3 do
    M.state.wheels[i] = {
        filteredSlipAngle = 0.0,
        filteredSlipRatio = 0.0,

        rawSlipAngle = 0.0,
        rawSlipRatio = 0.0,

        effectiveSlipAngle = 0.0,
        effectiveSlipRatio = 0.0,
        combinedSlip = 0.0,
        combinedSlipRaw = 0.0,
        slipEnergy = 0.0,

        loadScale = 1.0,
        loadRelaxScale = 1.0,
        loadSource = "DEFAULT",

        slipAngle = 0.0,
        slipRatio = 0.0,

        load = M.params.defaultLoad,
        normalLoad = M.params.defaultLoad,
        wheelLoad = M.params.defaultLoad,

        omega = 0.0,
        angularSpeed = 0.0,

        lag = 0.0,
        rubberLag = 0.0,
        memory = 0.0,
        memoryGrip = 1.0,
        responseDelay = 0.0,

        contactLoss = 0.0,
        contactEnergy = 0.0,
        contactQuality = 1.0,

        tireLimit = 0.0,
        tireGrip = 1.0,

        hopEnergy = 0.0,

        armCamber = 0.0,
        armToe = 0.0,

        driveTorque = 0.0,
        driveTorqueNm = 0.0,
        lsdLock = 0.0,
        lsdDiff = 0.0,
        shaftTwist = 0.0,
        driveLash = 0.0,

        carcassSupport = 1.0,
        carcassDelay = 0.0,
        carcassHysteresis = 0.0,
        carcassRecoveryBias = 0.0,

        slipRecoveryRate = 0.0,
        dirtyReturn = 0.0,
        snapRisk = 0.0,

        roadSeverity = 0.0,
        roadImpact = 0.0,
        loadPathLoss = 0.0,
        thermalStress = 0.0,

        lateralRelaxLength = M.params.relaxLateral,
        longitudinalRelaxLength = M.params.relaxLongitudinal,

        active = false,
        storeOnly = false,
    }

    M.state[i] = M.state.wheels[i]
end

M.debug = M.state

--============================================================
-- Utility
--============================================================

local function clamp(v, minValue, maxValue)
    v = tonumber(v) or 0.0
    if v ~= v then return minValue end
    if v < minValue then return minValue end
    if v > maxValue then return maxValue end
    return v
end

local function safeNumber(value, defaultValue)
    local n = tonumber(value)
    if n == nil or n ~= n or n == math.huge or n == -math.huge then
        return defaultValue or 0.0
    end
    return n
end

local function abs(v)
    return math.abs(safeNumber(v, 0.0))
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
    if value == nil then
        if defaultValue == nil then return nil end
        return defaultValue
    end
    return safeNumber(value, defaultValue)
end

local function safeStore(key, value)
    if not ac or not ac.store then return end
    pcall(function()
        ac.store(key, value)
    end)
end

local function safeField(obj, field, defaultValue)
    if not obj then return defaultValue end
    local ok, value = pcall(function()
        return obj[field]
    end)
    if not ok or value == nil then return defaultValue end
    return value
end

local function getCarSafe(car)
    if car then return car end
    if not ac or not ac.getCar then return nil end
    local ok, value = pcall(function()
        return ac.getCar(0)
    end)
    if not ok then return nil end
    return value
end

local function getWheelsSafe(car)
    if not car then return nil end
    return safeField(car, "wheels", nil)
end

local function getWheelSafe(wheels, index)
    if not wheels then return nil end
    return safeField(wheels, index, nil) or safeField(wheels, index + 1, nil)
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.0)
    dt = safeNumber(dt, 0.0)
    if tau <= 0.0 then return target end
    local alpha = clamp(dt / (tau + dt), 0.0, 1.0)
    return current + (target - current) * alpha
end

local function loadFirst(defaultValue, ...)
    for i = 1, select("#", ...) do
        local key = select(i, ...)
        if key then
            local value = safeLoadRaw(key)
            if value ~= nil then
                return safeNumber(value, defaultValue or 0.0), key
            end
        end
    end
    if defaultValue == nil then return nil, nil end
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

local function resetLinks()
    M.state.contactLinked = false
    M.state.memoryLinked = false
    M.state.complianceLinked = false
    M.state.dynamicsLinked = false
    M.state.hopLinked = false
    M.state.armLinked = false
    M.state.drivetrainLinked = false
    M.state.lsdLinked = false
    M.state.loadLinked = false
    M.state.carcassLinked = false
    M.state.recoveryLinked = false
    M.state.roadLinked = false
    M.state.loadPathLinked = false
    M.state.thermalLinked = false
end

--============================================================
-- Input helpers
--============================================================

local function readSpeed(car)
    local speedKmh = safeNumber(safeField(car, "speedKmh", nil), nil)
    if speedKmh ~= nil then
        return math.max(speedKmh, 0.0), math.max(speedKmh / 3.6, 0.0)
    end

    local speed = safeNumber(nil, nil)
    if speed ~= nil then
        if math.abs(speed) <= 140.0 then
            return math.max(speed * 3.6, 0.0), math.max(speed, 0.0)
        end
        return math.max(speed, 0.0), math.max(speed / 3.6, 0.0)
    end

    local storedKmh = safeLoadRaw("ngp_tire_state_speed_kmh")
    if storedKmh ~= nil then
        storedKmh = safeNumber(storedKmh, 0.0)
        return math.max(storedKmh, 0.0), math.max(storedKmh / 3.6, 0.0)
    end

    return 0.0, 0.0
end

local function readWheelLoadField(wheel)
    if not wheel then return nil end
    local fields = { "load", "loadK" }
    for _, field in ipairs(fields) do
        local value = safeNumber(safeField(wheel, field, nil), nil)
        if value ~= nil then
            return math.abs(value), "AC_" .. string.upper(field)
        end
    end
    return nil
end

local function readIntegratedLoad(index)
    local value, key = loadFirst(nil,
        "ngp_tire_load_input_" .. index,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index,
        "ngp_contact_load_" .. index,
        "ngp_tire_carcass_load_" .. index,
        "ngp_tire_force_load_" .. index,
        "ngp_load_path_load_" .. index,
        "ngp_lp_load_" .. index,
        "ngp_sprung_load_" .. index
    )

    if value ~= nil then
        M.state.loadLinked = true
        return math.abs(value), key or "NGP_LOAD"
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        M.state.loadLinked = true
        return math.max(M.params.defaultLoad + safeNumber(dlt, 0.0), 0.0), "DLT"
    end

    return nil, nil
end

local function getWheelLoad(wheel, index)
    local integrated, integratedSource = readIntegratedLoad(index)
    local direct, directSource = readWheelLoadField(wheel)

    if M.params.preferIntegratedLoad and integrated ~= nil then
        return integrated, integratedSource
    end

    if direct ~= nil then
        return direct, directSource
    end

    if integrated ~= nil then
        return integrated, integratedSource
    end

    return M.params.defaultLoad, "DEFAULT"
end

local function readRawSlip(index, wheel, previous)
    local rawSA = safeNumber(safeField(wheel, "slipAngle", nil), nil)
    local rawSR = safeNumber(safeField(wheel, "slipRatio", nil), nil)

    local linked = false

    if rawSA == nil then
        rawSA = loadFirst(nil,
            "ngp_contact_slip_angle_" .. index,
            "ngp_tire_carcass_slip_angle_" .. index,
            "ngp_tire_force_effective_slip_angle_" .. index,
            "ngp_tdyn_effective_slip_angle_" .. index,
            "ngp_slip_recovery_slip_angle_" .. index
        )
        if rawSA ~= nil then linked = true end
    end

    if rawSR == nil then
        rawSR = loadFirst(nil,
            "ngp_contact_slip_ratio_" .. index,
            "ngp_tire_carcass_slip_ratio_" .. index,
            "ngp_tire_force_effective_slip_ratio_" .. index,
            "ngp_tdyn_effective_slip_ratio_" .. index,
            "ngp_slip_recovery_slip_ratio_" .. index
        )
        if rawSR ~= nil then linked = true end
    end

    if rawSA == nil then rawSA = previous and previous.filteredSlipAngle or 0.0 end
    if rawSR == nil then rawSR = previous and previous.filteredSlipRatio or 0.0 end

    return safeNumber(rawSA, 0.0), safeNumber(rawSR, 0.0), linked
end

local function readOmega(index, wheel)
    local omega = safeNumber(safeField(wheel, "angularSpeed", nil), nil)
    if omega == nil then omega = safeNumber(nil, nil) end
    if omega == nil then
        omega = loadFirst(0.0,
            "ngp_wheel_omega_" .. index,
            "ngp_hub_omega_" .. index,
            "ngp_tire_omega_" .. index
        )
    end
    return safeNumber(omega, 0.0)
end

local function readExternalInputs(index, wheelState)
    local rubberLag, lagKey = loadFirst(0.0,
        "ngp_rubber_lag_" .. index,
        "ngp_tdyn_rubber_lag_" .. index,
        "ngp_tire_force_lag_" .. index
    )

    local memoryRaw, memoryKey = loadFirst(nil,
        "ngp_memory_" .. index,
        "ngp_rubber_memory_" .. index,
        "ngp_tire_memory_" .. index,
        "ngp_tyre_memory_" .. index,
        "ngp_tm_memory_" .. index
    )
    if memoryKey then M.state.memoryLinked = true end

    local memoryGrip, memoryGripKey = loadFirst(1.0,
        "ngp_memory_grip_" .. index,
        "ngp_tire_memory_grip_" .. index,
        "ngp_tyre_memory_grip_" .. index,
        "ngp_tm_grip_" .. index
    )
    if memoryGripKey then M.state.memoryLinked = true end

    local responseDelayRaw, responseKey = loadFirst(nil,
        "ngp_tire_response_delay",
        "ngp_tyre_response_delay",
        "ngp_tcomp_response_delay",
        "ngp_tire_compliance_response_delay"
    )
    if responseKey then M.state.complianceLinked = true end

    local relaxScale, relaxKey = loadFirst(nil,
        "ngp_tire_relax_scale_" .. index,
        "ngp_tcr_relax_scale_" .. index,
        "ngp_tcomp_relax_scale_" .. index
    )
    if relaxKey then M.state.complianceLinked = true end

    local contactLoss, contactLossKey = loadFirst(nil,
        "ngp_tire_contact_loss_" .. index,
        "ngp_tcr_contact_loss_" .. index,
        "ngp_contact_loss_" .. index,
        "ngp_contact_hop_drop_" .. index
    )

    local contactQuality, contactQualityKey = loadFirst(nil,
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index,
        "ngp_tc_contact_" .. index
    )

    local contactEnergy, contactEnergyKey = loadFirst(nil,
        "ngp_tcr_energy_push_" .. index,
        "ngp_tc_energy_" .. index,
        "ngp_tire_slip_energy_" .. index,
        "ngp_tire_force_slip_energy_" .. index
    )

    if contactLossKey or contactQualityKey or contactEnergyKey then
        M.state.contactLinked = true
    end

    local tireLimit, tireLimitKey = loadFirst(nil,
        "ngp_tire_limit_" .. index,
        "ngp_tyre_limit_" .. index,
        "ngp_tire_force_limit_" .. index,
        "ngp_td_limit_" .. index
    )

    local tireGrip, tireGripKey = loadFirst(nil,
        "ngp_tire_grip_" .. index,
        "ngp_tyre_grip_" .. index,
        "ngp_tire_force_grip_" .. index,
        "ngp_td_grip_" .. index
    )

    if tireLimitKey or tireGripKey then M.state.dynamicsLinked = true end

    local hop, hopKey = loadFirst(nil,
        "ngp_tire_hop_energy_" .. index,
        "ngp_tirehop_energy_" .. index,
        "ngp_thop_energy_" .. index
    )
    if hopKey then M.state.hopLinked = true end

    local armCamber, camberKey = loadFirst(nil,
        "ngp_control_arm_camber_" .. index,
        "ngp_arm_camber_" .. index,
        "ngp_compliance_camber_" .. index,
        "ngp_uc_camber_" .. index
    )

    local armToe, toeKey = loadFirst(nil,
        "ngp_control_arm_toe_" .. index,
        "ngp_arm_toe_" .. index,
        "ngp_compliance_toe_" .. index,
        "ngp_uc_toe_" .. index
    )

    if camberKey or toeKey then M.state.armLinked = true end

    local carcassSupport, supportKey = loadFirst(nil,
        "ngp_carcass_support_" .. index,
        "ngp_tire_carcass_support_" .. index,
        "ngp_contact_carcass_support_" .. index
    )

    local carcassDelay, delayKey = loadFirst(nil,
        "ngp_contact_delay_" .. index,
        "ngp_tire_contact_delay_" .. index,
        "ngp_carcass_contact_delay_" .. index
    )

    local carcassHyst, hystKey = loadFirst(nil,
        "ngp_carcass_hysteresis_" .. index,
        "ngp_tire_carcass_hysteresis_" .. index,
        "ngp_contact_hysteresis_" .. index
    )

    local carcassRecoveryBias, recoveryBiasKey = loadFirst(nil,
        "ngp_carcass_recovery_bias_" .. index,
        "ngp_contact_recovery_bias_" .. index
    )

    if supportKey or delayKey or hystKey or recoveryBiasKey then M.state.carcassLinked = true end

    local slipRecoveryRate, srKey = loadFirst(nil,
        "ngp_slip_recovery_rate_" .. index,
        "ngp_recovery_rate_" .. index,
        "ngp_sr_recovery_" .. index
    )

    local dirtyReturn, dirtyKey = loadFirst(nil,
        "ngp_slip_dirty_return_" .. index,
        "ngp_dirty_return_" .. index,
        "ngp_sr_dirty_return_" .. index
    )

    local snapRisk, snapKey = loadFirst(nil,
        "ngp_slip_snap_risk_" .. index,
        "ngp_snap_risk_" .. index,
        "ngp_sr_snap_risk_" .. index
    )

    if srKey or dirtyKey or snapKey then M.state.recoveryLinked = true end

    local roadSeverity, roadKey = loadFirst(nil,
        "ngp_road_input_severity_" .. index,
        "ngp_rii_severity_" .. index,
        "ngp_road_severity_" .. index
    )

    local roadImpact, impactKey = loadFirst(nil,
        "ngp_road_impact_" .. index,
        "ngp_rii_impact_" .. index,
        "ngp_impact_strength_" .. index
    )

    if roadKey or impactKey then M.state.roadLinked = true end

    local loadPathLoss, pathKey = loadFirst(nil,
        "ngp_load_path_loss_" .. index,
        "ngp_lp_loss_" .. index,
        "ngp_road_path_loss_" .. index
    )
    if pathKey then M.state.loadPathLinked = true end

    local thermalStress, thermalKey = loadFirst(nil,
        "ngp_thermal_stress_" .. index,
        "ngp_tire_memory_heat_" .. index,
        "ngp_memory_thermal_" .. index,
        "ngp_tire_temp_grip_" .. index
    )
    if thermalKey then M.state.thermalLinked = true end

    if index >= 2 then
        local driveTorque, driveKey = loadFirst(nil,
            "ngp_drive_torque",
            "ngp_drivetrain_torque",
            "ngp_drive_torque_normalized_from_nm"
        )

        local driveTorqueNm, driveNmKey = loadFirst(nil,
            "ngp_drive_torque_nm",
            "ngp_wheel_drive_torque_nm",
            "ngp_diff_input_torque_nm"
        )

        local lsdLock, lsdKey = loadFirst(nil,
            "ngp_lsd_lock",
            "ngp_diff_lock"
        )

        local lsdDiff, lsdDiffKey = loadFirst(nil,
            "ngp_lsd_diff",
            "ngp_diff_diff"
        )

        local shaftTwist, shaftKey = loadFirst(nil,
            "ngp_shaft_twist",
            "ngp_windup_twist",
            "ngp_driveline_windup_twist"
        )

        local driveLash, lashKey = loadFirst(nil,
            "ngp_drive_lash",
            "ngp_driveline_lash",
            "ngp_windup_lash"
        )

        if driveKey or driveNmKey or shaftKey or lashKey then M.state.drivetrainLinked = true end
        if lsdKey or lsdDiffKey then M.state.lsdLinked = true end

        wheelState.driveTorque = safeNumber(driveTorque, 0.0)
        wheelState.driveTorqueNm = safeNumber(driveTorqueNm, 0.0)
        wheelState.lsdLock = clamp(safeNumber(lsdLock, 0.0), 0.0, 1.0)
        wheelState.lsdDiff = safeNumber(lsdDiff, 0.0)
        wheelState.shaftTwist = safeNumber(shaftTwist, 0.0)
        wheelState.driveLash = safeNumber(driveLash, 0.0)
    else
        wheelState.driveTorque = 0.0
        wheelState.driveTorqueNm = 0.0
        wheelState.lsdLock = 0.0
        wheelState.lsdDiff = 0.0
        wheelState.shaftTwist = 0.0
        wheelState.driveLash = 0.0
    end

    wheelState.rubberLag = safeNumber(rubberLag, 0.0)
    wheelState.memory = clamp(safeNumber(memoryRaw, 0.0), 0.0, 1.0)
    wheelState.memoryGrip = clamp(safeNumber(memoryGrip, 1.0), 0.45, 1.15)
    wheelState.responseDelay = clamp(safeNumber(responseDelayRaw, 0.0), 0.0, 1.0)
    wheelState.relaxScaleInput = clamp(safeNumber(relaxScale, 1.0), 0.50, 1.75)
    wheelState.contactLoss = clamp(safeNumber(contactLoss, 0.0), 0.0, 1.0)
    wheelState.contactQuality = clamp(safeNumber(contactQuality, 1.0), 0.0, 1.2)
    wheelState.contactEnergy = clamp(safeNumber(contactEnergy, 0.0), 0.0, 1.0)
    wheelState.tireLimit = clamp(safeNumber(tireLimit, 0.0), 0.0, 2.5)
    wheelState.tireGrip = clamp(safeNumber(tireGrip, 1.0), 0.0, 1.35)
    wheelState.hopEnergy = clamp(safeNumber(hop, 0.0), 0.0, 1.0)
    wheelState.armCamber = safeNumber(armCamber, 0.0)
    wheelState.armToe = safeNumber(armToe, 0.0)
    wheelState.carcassSupport = clamp(safeNumber(carcassSupport, 1.0), 0.0, 1.2)
    wheelState.carcassDelay = clamp(safeNumber(carcassDelay, 0.0), 0.0, 1.0)
    wheelState.carcassHysteresis = clamp(safeNumber(carcassHyst, 0.0), 0.0, 1.0)
    wheelState.carcassRecoveryBias = clamp(safeNumber(carcassRecoveryBias, 0.0), 0.0, 1.0)
    wheelState.slipRecoveryRate = clamp(safeNumber(slipRecoveryRate, 0.0), 0.0, 1.2)
    wheelState.dirtyReturn = clamp(safeNumber(dirtyReturn, 0.0), 0.0, 1.0)
    wheelState.snapRisk = clamp(safeNumber(snapRisk, 0.0), 0.0, 1.0)
    wheelState.roadSeverity = clamp(safeNumber(roadSeverity, 0.0), 0.0, 1.5)
    wheelState.roadImpact = clamp(safeNumber(roadImpact, 0.0), 0.0, 1.5)
    wheelState.loadPathLoss = clamp(safeNumber(loadPathLoss, 0.0), 0.0, 1.0)
    wheelState.thermalStress = clamp(safeNumber(thermalStress, 0.0), 0.0, 1.5)

    local lag = wheelState.rubberLag
        + wheelState.memory * M.params.memoryLagGain
        + wheelState.responseDelay * M.params.complianceLagGain
        + math.max(wheelState.relaxScaleInput - 1.0, 0.0) * 0.22
        + wheelState.contactLoss * M.params.contactLossLagGain
        + wheelState.contactEnergy * M.params.contactEnergyLagGain
        + wheelState.tireLimit * M.params.tireLimitLagGain
        + wheelState.hopEnergy * M.params.tireHopLagGain
        + (1.0 - clamp(wheelState.memoryGrip, 0.0, 1.0)) * 0.16
        + wheelState.carcassDelay * M.params.carcassDelayLagGain
        + wheelState.carcassHysteresis * M.params.carcassHysteresisLagGain
        + wheelState.dirtyReturn * 0.08
        + wheelState.snapRisk * 0.08
        + wheelState.roadSeverity * M.params.roadInputLagGain
        + wheelState.roadImpact * M.params.roadInputLagGain
        + wheelState.loadPathLoss * M.params.loadPathLagGain
        + math.max(wheelState.thermalStress - 1.0, 0.0) * M.params.thermalLagGain
        + (math.abs(wheelState.armCamber) + math.abs(wheelState.armToe)) * M.params.armLagGain

    lag = lag
        - math.max(wheelState.contactQuality - 1.0, 0.0) * M.params.contactQualityRecoverGain
        - math.max(wheelState.tireGrip - 1.0, 0.0) * M.params.tireGripRecoverGain
        - math.max(wheelState.carcassSupport - 1.0, 0.0) * M.params.carcassSupportRecoverGain
        - wheelState.carcassRecoveryBias * M.params.recoveryLagReduceGain
        - wheelState.slipRecoveryRate * M.params.recoveryLagReduceGain

    if index >= 2 then
        lag = lag
            + math.abs(wheelState.driveTorque) * M.params.driveRearLongLagGain
            + math.abs(wheelState.driveTorqueNm) / 3000.0 * M.params.driveRearLongLagGain
            + wheelState.lsdLock * M.params.lsdRearLongLagGain
            + math.min(math.abs(wheelState.lsdDiff) / 8.0, 1.0) * M.params.lsdRearLongLagGain
            + math.abs(wheelState.shaftTwist) * M.params.shaftRearLongLagGain
            + math.abs(wheelState.driveLash) * M.params.windupRearLongLagGain
    end

    return clamp(lag, M.params.minLag, M.params.maxLag)
end

--============================================================
-- Relaxation model
--============================================================

local function relaxation(current, target, speed, length, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    speed = safeNumber(speed, 0.0)
    length = safeNumber(length, M.params.minRelaxLength)
    dt = safeNumber(dt, 0.0)

    if speed < M.params.minSpeed then
        return target
    end

    local distance = math.max(speed * dt, 0.0)
    local relaxLength = math.max(length, M.params.minRelaxLength)
    local alpha = 1.0 - math.exp(-distance / relaxLength)
    alpha = clamp(alpha, 0.0, 1.0)

    return current + (target - current) * alpha
end

local function calculateLoadRelaxScale(load)
    local ref = math.max(M.params.loadReference or M.params.defaultLoad, 1.0)
    local loadScale = clamp(safeNumber(load, M.params.defaultLoad) / ref, 0.10, 3.00)

    local relaxScale = 1.0 + (loadScale - 1.0) * M.params.loadRelaxGain

    return loadScale, clamp(relaxScale, M.params.minLoadRelaxScale, M.params.maxLoadRelaxScale)
end

local function calculateCombinedSlip(wheelState)
    local saNorm = abs(wheelState.filteredSlipAngle) / math.max(M.params.slipAngleRefRad, 0.001)
    local srNorm = abs(wheelState.filteredSlipRatio) / math.max(M.params.slipRatioRef, 0.001)

    local combined = math.sqrt(
        (saNorm * M.params.combinedSlipAngleWeight) * (saNorm * M.params.combinedSlipAngleWeight)
        + (srNorm * M.params.combinedSlipRatioWeight) * (srNorm * M.params.combinedSlipRatioWeight)
    )

    local raw = math.sqrt(
        (wheelState.filteredSlipAngle or 0.0) * (wheelState.filteredSlipAngle or 0.0)
        + (wheelState.filteredSlipRatio or 0.0) * (wheelState.filteredSlipRatio or 0.0)
    )

    wheelState.combinedSlip = clamp(combined, 0.0, M.params.maxCombinedSlip)
    wheelState.combinedSlipRaw = clamp(raw, 0.0, 10.0)
end

local function updateSlipEnergy(wheelState, speedMs, dt)
    calculateCombinedSlip(wheelState)

    local speedFactor = clamp(math.max(speedMs, 0.0) / 35.0, 0.0, 1.25)
    local slipBuild = math.max((wheelState.combinedSlip or 0.0) - M.params.slipEnergyOverStart, 0.0)

    local targetEnergy = slipBuild * M.params.slipEnergyGain * (0.35 + 0.65 * speedFactor)
        + (wheelState.contactEnergy or 0.0) * 0.18
        + (wheelState.tireLimit or 0.0) * 0.10
        + (wheelState.hopEnergy or 0.0) * 0.12
        + (wheelState.roadImpact or 0.0) * 0.08

    targetEnergy = clamp(targetEnergy, 0.0, M.params.maxSlipEnergy)

    wheelState.slipEnergy = lowPass(
        wheelState.slipEnergy or 0.0,
        targetEnergy,
        M.params.slipEnergyTau,
        dt
    )
end

local function syncLegacyWheel(index)
    local wheelState = M.state.wheels[index]
    if not wheelState then return end

    wheelState.slipAngle = wheelState.filteredSlipAngle or 0.0
    wheelState.slipRatio = wheelState.filteredSlipRatio or 0.0
    wheelState.normalLoad = wheelState.load or M.params.defaultLoad
    wheelState.wheelLoad = wheelState.load or M.params.defaultLoad
    wheelState.angularSpeed = wheelState.omega or 0.0

    M.state.slipAngle[index] = wheelState.slipAngle
    M.state.slipRatio[index] = wheelState.slipRatio
    M.state.rawSlipAngleLegacy[index] = wheelState.rawSlipAngle or 0.0
    M.state.rawSlipRatioLegacy[index] = wheelState.rawSlipRatio or 0.0
    M.state.load[index] = wheelState.load or M.params.defaultLoad
    M.state.omega[index] = wheelState.omega or 0.0
    M.state.active[index] = wheelState.active and true or false
end

local function updateWheelState(index, wheel, speedMs, dt, storeOnly)
    local wheelState = M.state.wheels[index]

    local load, loadSource = getWheelLoad(wheel, index)
    wheelState.load = load
    wheelState.loadSource = loadSource or "UNKNOWN"
    wheelState.loadScale, wheelState.loadRelaxScale = calculateLoadRelaxScale(wheelState.load)

    local lag = readExternalInputs(index, wheelState)

    local rawSA, rawSR, externalSlip = readRawSlip(index, wheel, wheelState)
    if externalSlip then M.state.contactLinked = true end

    wheelState.rawSlipAngle = rawSA
    wheelState.rawSlipRatio = rawSR
    wheelState.omega = readOmega(index, wheel)

    local lateralLength = M.params.relaxLateral
        * (1.0 + lag + (wheelState.loadRelaxScale - 1.0) + math.abs(wheelState.armCamber) * M.params.armCamberLatGain)

    local longitudinalLength = M.params.relaxLongitudinal
        * (1.0 + lag + (wheelState.loadRelaxScale - 1.0) + math.abs(wheelState.armToe) * M.params.armToeLongGain)

    lateralLength = math.max(lateralLength, M.params.minRelaxLength)
    longitudinalLength = math.max(longitudinalLength, M.params.minRelaxLength)

    wheelState.filteredSlipAngle = relaxation(
        wheelState.filteredSlipAngle,
        wheelState.rawSlipAngle,
        speedMs,
        lateralLength,
        dt
    )

    wheelState.filteredSlipRatio = relaxation(
        wheelState.filteredSlipRatio,
        wheelState.rawSlipRatio,
        speedMs,
        longitudinalLength,
        dt
    )

    wheelState.effectiveSlipAngle = wheelState.filteredSlipAngle
    wheelState.effectiveSlipRatio = wheelState.filteredSlipRatio

    updateSlipEnergy(wheelState, speedMs, dt)

    wheelState.lag = lag
    wheelState.lateralRelaxLength = lateralLength
    wheelState.longitudinalRelaxLength = longitudinalLength
    wheelState.active = not not wheel
    wheelState.storeOnly = storeOnly and true or false

    syncLegacyWheel(index)
end

local function decayWheel(index, dt, status)
    local wheelState = M.state.wheels[index]
    local tau = status == "NO CAR" and M.params.noCarDecayTau or M.params.noWheelDecayTau

    wheelState.rawSlipAngle = 0.0
    wheelState.rawSlipRatio = 0.0
    wheelState.filteredSlipAngle = lowPass(wheelState.filteredSlipAngle, 0.0, tau, dt)
    wheelState.filteredSlipRatio = lowPass(wheelState.filteredSlipRatio, 0.0, tau, dt)
    wheelState.effectiveSlipAngle = wheelState.filteredSlipAngle
    wheelState.effectiveSlipRatio = wheelState.filteredSlipRatio
    wheelState.slipEnergy = lowPass(wheelState.slipEnergy, 0.0, tau, dt)
    wheelState.combinedSlip = lowPass(wheelState.combinedSlip, 0.0, tau, dt)
    wheelState.combinedSlipRaw = lowPass(wheelState.combinedSlipRaw, 0.0, tau, dt)
    wheelState.omega = lowPass(wheelState.omega, 0.0, tau, dt)
    wheelState.active = false
    wheelState.storeOnly = true

    syncLegacyWheel(index)
end

--============================================================
-- Export
--============================================================

local function exportWheel(index)
    local wheelState = M.state.wheels[index]

    safeStore("ngp_slip_angle_" .. index, wheelState.filteredSlipAngle or 0.0)
    safeStore("ngp_slip_ratio_" .. index, wheelState.filteredSlipRatio or 0.0)
    safeStore("ngp_filtered_slip_angle_" .. index, wheelState.filteredSlipAngle or 0.0)
    safeStore("ngp_filtered_slip_ratio_" .. index, wheelState.filteredSlipRatio or 0.0)

    safeStore("ngp_tire_slip_angle_" .. index, wheelState.filteredSlipAngle or 0.0)
    safeStore("ngp_tire_slip_ratio_" .. index, wheelState.filteredSlipRatio or 0.0)
    safeStore("ngp_tire_filtered_slip_angle_" .. index, wheelState.filteredSlipAngle or 0.0)
    safeStore("ngp_tire_filtered_slip_ratio_" .. index, wheelState.filteredSlipRatio or 0.0)

    safeStore("ngp_tire_state_active_" .. index, wheelState.active and 1 or 0)
    safeStore("ngp_tire_state_store_only_" .. index, wheelState.storeOnly and 1 or 0)

    safeStore("ngp_tire_effective_slip_angle_" .. index, wheelState.effectiveSlipAngle or 0.0)
    safeStore("ngp_tire_effective_slip_ratio_" .. index, wheelState.effectiveSlipRatio or 0.0)
    safeStore("ngp_tire_combined_slip_" .. index, wheelState.combinedSlip or 0.0)
    safeStore("ngp_tire_combined_slip_raw_" .. index, wheelState.combinedSlipRaw or 0.0)
    safeStore("ngp_tire_slip_energy_" .. index, wheelState.slipEnergy or 0.0)

    safeStore("ngp_tire_load_" .. index, wheelState.load or M.params.defaultLoad)
    safeStore("ngp_wheel_load_" .. index, wheelState.load or M.params.defaultLoad)
    safeStore("ngp_wheel_omega_" .. index, wheelState.omega or 0.0)

    safeStore("ngp_ts_slip_angle_" .. index, wheelState.filteredSlipAngle or 0.0)
    safeStore("ngp_ts_slip_ratio_" .. index, wheelState.filteredSlipRatio or 0.0)
    safeStore("ngp_ts_combined_slip_" .. index, wheelState.combinedSlip or 0.0)
    safeStore("ngp_ts_slip_energy_" .. index, wheelState.slipEnergy or 0.0)
    safeStore("ngp_ts_load_" .. index, wheelState.load or M.params.defaultLoad)
    safeStore("ngp_ts_lag_" .. index, wheelState.lag or 0.0)

    if not M.state.debugStoreNow then return end

    safeStore("ngp_raw_slip_angle_" .. index, wheelState.rawSlipAngle or 0.0)
    safeStore("ngp_raw_slip_ratio_" .. index, wheelState.rawSlipRatio or 0.0)
    safeStore("ngp_tire_raw_slip_angle_" .. index, wheelState.rawSlipAngle or 0.0)
    safeStore("ngp_tire_raw_slip_ratio_" .. index, wheelState.rawSlipRatio or 0.0)

    safeStore("ngp_tire_state_load_" .. index, wheelState.load or 0.0)
    safeStore("ngp_tire_state_load_scale_" .. index, wheelState.loadScale or 1.0)
    safeStore("ngp_tire_state_load_relax_scale_" .. index, wheelState.loadRelaxScale or 1.0)
    safeStore("ngp_tire_state_load_source_" .. index, wheelState.loadSource or "UNKNOWN")

    safeStore("ngp_tire_relax_lat_" .. index, wheelState.lateralRelaxLength or M.params.relaxLateral)
    safeStore("ngp_tire_relax_long_" .. index, wheelState.longitudinalRelaxLength or M.params.relaxLongitudinal)
    safeStore("ngp_tire_state_lag_" .. index, wheelState.lag or 0.0)

    safeStore("ngp_tire_state_contact_loss_" .. index, wheelState.contactLoss or 0.0)
    safeStore("ngp_tire_state_contact_quality_" .. index, wheelState.contactQuality or 1.0)
    safeStore("ngp_tire_state_hop_" .. index, wheelState.hopEnergy or 0.0)
    safeStore("ngp_tire_state_lsd_" .. index, wheelState.lsdLock or 0.0)

    safeStore("ngp_tire_state_memory_" .. index, wheelState.memory or 0.0)
    safeStore("ngp_tire_state_memory_grip_" .. index, wheelState.memoryGrip or 1.0)
    safeStore("ngp_tire_state_response_delay_" .. index, wheelState.responseDelay or 0.0)
    safeStore("ngp_tire_state_carcass_delay_" .. index, wheelState.carcassDelay or 0.0)
    safeStore("ngp_tire_state_recovery_rate_" .. index, wheelState.slipRecoveryRate or 0.0)
    safeStore("ngp_tire_state_road_severity_" .. index, wheelState.roadSeverity or 0.0)
end

local function exportGlobal()
    safeStore("ngp_tire_state_status", M.state.status or "UNKNOWN")
    safeStore("ngp_tire_state_update_count", M.state.updateCount or 0)
    safeStore("ngp_tire_state_wheels_valid", M.state.wheelsValid and 1 or 0)
    safeStore("ngp_tire_state_valid", M.state.valid and 1 or 0)

    safeStore("ngp_tire_state_speed_ms", M.state.speedMs or 0.0)
    safeStore("ngp_tire_state_speed_kmh", M.state.speedKmh or 0.0)

    safeStore("ngp_tire_state_avg_combined_slip", M.state.avgCombinedSlip or 0.0)
    safeStore("ngp_tire_state_avg_slip_energy", M.state.avgSlipEnergy or 0.0)
    safeStore("ngp_tire_state_avg_load", M.state.avgLoad or M.params.defaultLoad)
    safeStore("ngp_tire_state_avg_lag", M.state.avgLag or 0.0)
    safeStore("ngp_tire_state_active_count", M.state.activeCount or 0)

    safeStore("ngp_ts_avg_combined_slip", M.state.avgCombinedSlip or 0.0)
    safeStore("ngp_ts_avg_slip_energy", M.state.avgSlipEnergy or 0.0)
    safeStore("ngp_ts_avg_load", M.state.avgLoad or M.params.defaultLoad)
    safeStore("ngp_ts_active_count", M.state.activeCount or 0)

    if not M.state.debugStoreNow then return end

    safeStore("ngp_tire_state_contact_linked", M.state.contactLinked and 1 or 0)
    safeStore("ngp_tire_state_memory_linked", M.state.memoryLinked and 1 or 0)
    safeStore("ngp_tire_state_compliance_linked", M.state.complianceLinked and 1 or 0)
    safeStore("ngp_tire_state_dynamics_linked", M.state.dynamicsLinked and 1 or 0)
    safeStore("ngp_tire_state_hop_linked", M.state.hopLinked and 1 or 0)
    safeStore("ngp_tire_state_arm_linked", M.state.armLinked and 1 or 0)
    safeStore("ngp_tire_state_drivetrain_linked", M.state.drivetrainLinked and 1 or 0)
    safeStore("ngp_tire_state_lsd_linked", M.state.lsdLinked and 1 or 0)
    safeStore("ngp_tire_state_load_linked", M.state.loadLinked and 1 or 0)
    safeStore("ngp_tire_state_carcass_linked", M.state.carcassLinked and 1 or 0)
    safeStore("ngp_tire_state_recovery_linked", M.state.recoveryLinked and 1 or 0)
    safeStore("ngp_tire_state_road_linked", M.state.roadLinked and 1 or 0)
    safeStore("ngp_tire_state_load_path_linked", M.state.loadPathLinked and 1 or 0)
    safeStore("ngp_tire_state_thermal_linked", M.state.thermalLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i)
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

    dt = clamp(dt, 0.0001, 0.100)
    updateDebugGate(dt)
    resetLinks()

    car = getCarSafe(car)
    local wheels = getWheelsSafe(car)

    local speedKmh, speedMs = readSpeed(car)
    M.state.speedKmh = speedKmh
    M.state.speedMs = speedMs

    local status = "RUNNING"
    local wheelsValid = wheels ~= nil
    local valid = car ~= nil and wheels ~= nil

    if not car then
        status = "NO CAR"
    elseif not wheels then
        status = "NO WHEELS"
    end

    local sumCombined = 0.0
    local sumEnergy = 0.0
    local sumLoad = 0.0
    local sumLag = 0.0
    local activeCount = 0

    for i = 0, 3 do
        local wheel = getWheelSafe(wheels, i)

        if wheel then
            updateWheelState(i, wheel, speedMs, dt, false)
        else
            local integrated = readIntegratedLoad(i)
            local externalSlip = loadFirst(nil,
                "ngp_contact_slip_angle_" .. i,
                "ngp_tire_carcass_slip_angle_" .. i,
                "ngp_tire_force_effective_slip_angle_" .. i
            )

            if integrated ~= nil or externalSlip ~= nil or car ~= nil then
                updateWheelState(i, nil, speedMs, dt, true)
            else
                decayWheel(i, dt, status)
            end
        end

        local ws = M.state.wheels[i]
        if ws.active then activeCount = activeCount + 1 end
        sumCombined = sumCombined + (ws.combinedSlip or 0.0)
        sumEnergy = sumEnergy + (ws.slipEnergy or 0.0)
        sumLoad = sumLoad + (ws.load or M.params.defaultLoad)
        sumLag = sumLag + (ws.lag or 0.0)

        exportWheel(i)
    end

    M.state.avgCombinedSlip = sumCombined * 0.25
    M.state.avgSlipEnergy = sumEnergy * 0.25
    M.state.avgLoad = sumLoad * 0.25
    M.state.avgLag = sumLag * 0.25
    M.state.activeCount = activeCount

    M.state.status = status
    M.state.wheelsValid = wheelsValid
    M.state.valid = valid

    exportGlobal()
end

--============================================================
-- Public API
--============================================================

function M.getState(index)
    if index == nil then return M.state end
    return M.state.wheels[index]
end

function M.getSlipAngle(index)
    local wheelState = M.state.wheels[index]
    if not wheelState then return 0.0 end
    return wheelState.filteredSlipAngle or 0.0
end

function M.getSlipRatio(index)
    local wheelState = M.state.wheels[index]
    if not wheelState then return 0.0 end
    return wheelState.filteredSlipRatio or 0.0
end

function M.getRawSlipAngle(index)
    local wheelState = M.state.wheels[index]
    if not wheelState then return 0.0 end
    return wheelState.rawSlipAngle or 0.0
end

function M.getRawSlipRatio(index)
    local wheelState = M.state.wheels[index]
    if not wheelState then return 0.0 end
    return wheelState.rawSlipRatio or 0.0
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

function M.getLoad(index)
    local wheelState = M.state.wheels[index]
    if not wheelState then return M.params.defaultLoad end
    return wheelState.load or M.params.defaultLoad
end

function M.debugStr(index)
    if index ~= nil then
        local wheelState = M.state.wheels[index] or M.state.wheels[0]
        return string.format(
            "Status %s / Count %.0f\n" ..
            "SA %.3f->%.3f SR %.3f->%.3f\n" ..
            "Comb %.2f E %.2f Load %.0f/%s Lag %.3f\n" ..
            "Mem %.3f Resp %.3f CT %.2f Hop %.2f LSD %.2f Store %s",
            tostring(M.state.status),
            M.state.updateCount or 0,
            wheelState.rawSlipAngle or 0.0,
            wheelState.filteredSlipAngle or 0.0,
            wheelState.rawSlipRatio or 0.0,
            wheelState.filteredSlipRatio or 0.0,
            wheelState.combinedSlip or 0.0,
            wheelState.slipEnergy or 0.0,
            wheelState.load or 0.0,
            tostring(wheelState.loadSource or "NIL"),
            wheelState.lag or 0.0,
            wheelState.memory or 0.0,
            wheelState.responseDelay or 0.0,
            wheelState.contactLoss or 0.0,
            wheelState.hopEnergy or 0.0,
            wheelState.lsdLock or 0.0,
            wheelState.storeOnly and "YES" or "NO"
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s / Active %.0f\n" ..
        "SA FL %.3f FR %.3f RL %.3f RR %.3f\n" ..
        "SR FL %.3f FR %.3f RL %.3f RR %.3f\n" ..
        "Comb %.2f E %.2f Load %.0f Lag %.3f Speed %.1f\n" ..
        "Links CT:%s MEM:%s CP:%s TD:%s HOP:%s ARM:%s DT:%s LSD:%s CAR:%s REC:%s",
        tostring(M.state.status),
        M.state.updateCount or 0,
        M.state.wheelsValid and "OK" or "NIL",
        M.state.activeCount or 0,
        M.state.wheels[0].filteredSlipAngle or 0.0,
        M.state.wheels[1].filteredSlipAngle or 0.0,
        M.state.wheels[2].filteredSlipAngle or 0.0,
        M.state.wheels[3].filteredSlipAngle or 0.0,
        M.state.wheels[0].filteredSlipRatio or 0.0,
        M.state.wheels[1].filteredSlipRatio or 0.0,
        M.state.wheels[2].filteredSlipRatio or 0.0,
        M.state.wheels[3].filteredSlipRatio or 0.0,
        M.state.avgCombinedSlip or 0.0,
        M.state.avgSlipEnergy or 0.0,
        M.state.avgLoad or 0.0,
        M.state.avgLag or 0.0,
        M.state.speedKmh or 0.0,
        M.state.contactLinked and "OK" or "NIL",
        M.state.memoryLinked and "OK" or "NIL",
        M.state.complianceLinked and "OK" or "NIL",
        M.state.dynamicsLinked and "OK" or "NIL",
        M.state.hopLinked and "OK" or "NIL",
        M.state.armLinked and "OK" or "NIL",
        M.state.drivetrainLinked and "OK" or "NIL",
        M.state.lsdLinked and "OK" or "NIL",
        M.state.carcassLinked and "OK" or "NIL",
        M.state.recoveryLinked and "OK" or "NIL"
    )
end

return M
