---@diagnostic disable: undefined-global

--============================================================
-- ACNextGen
-- tire_force.lua
-- Phase 1 / V1.1.5 stable
-- Tire Transient Force / Combined Slip Output
--
-- Policy:
--   - Do not rewrite AC physics directly.
--   - Merge tire_state / tire_dynamics / contact / caster / arms /
--     drivetrain / LSD into shared transient force signals.
--   - Export stable root/trunk values for load_transfer, suspension,
--     road_input_interpreter, observer and later physics bridges.
--============================================================

local M = {}

--============================================================
-- Parameters
--============================================================

M.params = {
    lateralScale = 300.0,
    longitudinalScale = 105.0,
    maxForce = 2500.0,

    loadReference = 4000.0,
    defaultLoad = 3000.0,
    minLoad = 80.0,
    minLoadScale = 0.0,
    maxLoadScale = 1.5,
    loadBlend = 0.35,

    contactGripInfluence = 0.20,
    dynamicsGripInfluence = 0.15,
    contactLatInfluence = 0.30,
    contactLongInfluence = 0.30,
    complianceDelayGain = 0.35,
    casterGripInfluence = 1.0,

    tireLimitForceLoss = 0.18,
    tireLagForceDelay = 0.12,

    lsdRearLongGain = 0.10,
    driveTorqueLongGain = 0.18,
    shaftTwistLongGain = 0.08,

    armToeLatLossGain = 0.08,
    armCamberLatLossGain = 0.06,

    contactQualityForceGain = 0.16,
    contactLossForceDrop = 0.18,
    hopForceDrop = 0.12,

    rf2MuScaleInfluence = 0.15,
    rf2SlidingForceDrop = 0.05,
    ultraCamberInfluence = 0.08,

    combinedSlipForceDrop = 0.22,
    slipEnergyForceDrop = 0.16,
    combinedSlipBalanceGain = 0.35,
    maxCombinedSlip = 2.50,
    maxSlipEnergy = 1.00,

    diffInputTorqueReferenceNm = 3000.0,
    diffInputTorqueLongGain = 8.0,
    driveTorqueNmLongGain = 5.0,

    loadForceExponent = 0.92,
    minEffectiveLoadScale = 0.08,
    maxEffectiveLoadScale = 1.65,

    roadInputForceReference = 2200.0,

    carcassSupportInfluence = 0.20,
    carcassGripGateInfluence = 0.18,
    carcassDeformationDrop = 0.12,
    carcassDelayGain = 0.16,

    recoveryGripGain = 0.10,
    recoverySnapDrop = 0.08,
    recoveryDirtyDrop = 0.05,

    roadShockForceDrop = 0.08,
    roadPathLossDrop = 0.08,
    roadTireDeliveryGain = 0.10,
    loadPathLossDrop = 0.10,
    thermalStressDrop = 0.05,

    forceTau = 0.030,
    releaseTau = 0.055,
    storeOnlyTau = 0.085,

    maxDt = 0.100,
    minDt = 0.0001,

    debugStoreInterval = 0.25,
}

--============================================================
-- State
--============================================================

local WHEEL_NAMES = { [0] = "FL", [1] = "FR", [2] = "RL", [3] = "RR" }

M.lastForce = {}

for i = 0, 3 do
    M.lastForce[i] = {
        lateral = 0.0,
        longitudinal = 0.0,
        rawLateral = 0.0,
        rawLongitudinal = 0.0,

        slipAngle = 0.0,
        slipRatio = 0.0,
        filteredSlipAngle = 0.0,
        filteredSlipRatio = 0.0,
        dSlipAngle = 0.0,
        dSlipRatio = 0.0,

        load = M.params.defaultLoad,
        loadScale = 1.0,
        effectiveLoadScale = 1.0,
        loadSource = "INIT",

        effectiveSlipAngle = 0.0,
        effectiveSlipRatio = 0.0,
        combinedSlip = 0.0,
        slipEnergy = 0.0,

        contactGrip = 1.0,
        contactLat = 1.0,
        contactLong = 1.0,
        contactQuality = 1.0,
        contactLoss = 0.0,

        casterGrip = 1.0,
        tireGrip = 1.0,
        tireLimit = 0.0,
        tireLag = 0.0,
        responseDelay = 0.0,

        lsdLock = 0.0,
        driveTorque = 0.0,
        driveTorqueNm = 0.0,
        diffInputTorqueNm = 0.0,
        shaftTwist = 0.0,
        driveLash = 0.0,

        armCamber = 0.0,
        armToe = 0.0,
        hopEnergy = 0.0,

        rf2MuScale = 1.0,
        rf2Sliding = 0.0,
        rf2Fy = 0.0,

        ucMuCamber = 1.0,
        ucCamber = 0.0,
        ucToe = 0.0,

        carcassSupport = 1.0,
        carcassGripGate = 1.0,
        carcassDeformation = 0.0,
        carcassDelay = 0.0,
        carcassEnergy = 0.0,

        recoveryRate = 0.0,
        gripReturn = 0.0,
        snapRisk = 0.0,
        dirtyReturn = 0.0,

        roadSeverity = 0.0,
        roadShock = 0.0,
        roadPathLoss = 0.0,
        roadTireDelivery = 1.0,
        loadPathLoss = 0.0,
        thermalStress = 0.0,

        finalScale = 1.0,
        forceMagnitude = 0.0,
        roadInput = 0.0,
        combinedBalance = 1.0,

        active = false,
    }
end

M.state = {
    force = M.lastForce,

    tireStateLinked = false,
    contactLinked = false,
    dynamicsLinked = false,
    complianceLinked = false,
    casterLinked = false,
    drivetrainLinked = false,
    lsdLinked = false,
    armLinked = false,
    rf2Linked = false,
    ultraChassisLinked = false,
    carcassLinked = false,
    recoveryLinked = false,
    roadLinked = false,
    loadPathLinked = false,
    thermalLinked = false,

    wheelsValid = false,
    storeOnly = false,

    avgLatForce = 0.0,
    avgLongForce = 0.0,
    avgForceScale = 1.0,
    avgCombinedSlip = 0.0,
    avgSlipEnergy = 0.0,
    avgRoadInput = 0.0,
    maxForceMagnitude = 0.0,
    maxRoadInput = 0.0,
    activeCount = 0,

    status = "INIT",
    updateCount = 0,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

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

local function safeStore(key, value)
    if not ac or not ac.store then return end
    pcall(function()
        ac.store(key, value)
    end)
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

local function safeField(obj, field, defaultValue)
    if not obj then return defaultValue end
    local ok, value = pcall(function()
        return obj[field]
    end)
    if not ok or value == nil then return defaultValue end
    return value
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.001)
    dt = safeNumber(dt, 0.0)

    if tau <= 0.0001 then return target end

    local alpha = clamp(dt / (tau + dt), 0.0, 1.0)
    return current + (target - current) * alpha
end

local function loadFirstRaw(...)
    for n = 1, select("#", ...) do
        local key = select(n, ...)
        if key ~= nil then
            local value = safeLoadRaw(key)
            if value ~= nil then
                return value, key
            end
        end
    end
    return nil, nil
end

local function loadFirstNumber(defaultValue, ...)
    local value, key = loadFirstRaw(...)
    if value == nil then
        return defaultValue, nil
    end
    return safeNumber(value, defaultValue), key
end

local function safeGetCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function()
        return ac.getCar(0)
    end)
    if not ok then return nil end
    return car
end

local function safeGetWheels(car)
    return safeField(car, "wheels", nil)
end

local function getWheelFromWheels(wheels, index)
    if not wheels then return nil end
    return safeField(wheels, index, nil) or safeField(wheels, index + 1, nil)
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

local function readSlipState(index, wheel)
    local linked = false

    local slipAngle, slipAngleKey = loadFirstNumber(
        nil,
        "ngp_tire_slip_angle_" .. index,
        "ngp_slip_angle_" .. index,
        "ngp_contact_slip_angle_" .. index,
        "ngp_tire_state_slip_angle_" .. index
    )

    local slipRatio, slipRatioKey = loadFirstNumber(
        nil,
        "ngp_tire_slip_ratio_" .. index,
        "ngp_slip_ratio_" .. index,
        "ngp_contact_slip_ratio_" .. index,
        "ngp_tire_state_slip_ratio_" .. index
    )

    local filteredSlipAngle, filteredAngleKey = loadFirstNumber(
        nil,
        "ngp_filtered_slip_angle_" .. index,
        "ngp_tire_filtered_slip_angle_" .. index,
        "ngp_tire_state_filtered_slip_angle_" .. index
    )

    local filteredSlipRatio, filteredRatioKey = loadFirstNumber(
        nil,
        "ngp_filtered_slip_ratio_" .. index,
        "ngp_tire_filtered_slip_ratio_" .. index,
        "ngp_tire_state_filtered_slip_ratio_" .. index
    )

    if slipAngleKey or slipRatioKey or filteredAngleKey or filteredRatioKey then
        linked = true
    end

    if slipAngle == nil then
        slipAngle = safeNumber(safeField(wheel, "slipAngle", 0.0), 0.0)
    end

    if slipRatio == nil then
        slipRatio = safeNumber(safeField(wheel, "slipRatio", 0.0), 0.0)
    end

    if filteredSlipAngle == nil then filteredSlipAngle = slipAngle end
    if filteredSlipRatio == nil then filteredSlipRatio = slipRatio end

    return slipAngle, slipRatio, filteredSlipAngle, filteredSlipRatio, linked
end

local function estimateCombinedSlip(angle, ratio)
    local lateralNorm = abs(angle) / 0.174533
    local longNorm = abs(ratio) / 0.12
    return math.sqrt(lateralNorm * lateralNorm + longNorm * longNorm)
end

local function readTireDynamicsRoot(index, filteredSlipAngle, filteredSlipRatio)
    local linked = false

    local effectiveSlipAngle, effAngleKey = loadFirstNumber(
        nil,
        "ngp_tire_effective_slip_angle_" .. index,
        "ngp_tdyn_effective_slip_angle_" .. index,
        "ngp_tire_dynamics_effective_slip_angle_" .. index
    )

    local effectiveSlipRatio, effRatioKey = loadFirstNumber(
        nil,
        "ngp_tire_effective_slip_ratio_" .. index,
        "ngp_tdyn_effective_slip_ratio_" .. index,
        "ngp_tire_dynamics_effective_slip_ratio_" .. index
    )

    local combinedSlip, combinedKey = loadFirstNumber(
        nil,
        "ngp_tdyn_combined_slip_" .. index,
        "ngp_tire_dynamics_combined_slip_" .. index,
        "ngp_tire_combined_slip_" .. index,
        "ngp_contact_combined_slip_" .. index
    )

    local slipEnergy, energyKey = loadFirstNumber(
        nil,
        "ngp_tdyn_slip_energy_" .. index,
        "ngp_tire_dynamics_slip_energy_" .. index,
        "ngp_tc_energy_" .. index,
        "ngp_tire_slip_energy_" .. index
    )

    if effAngleKey or effRatioKey or combinedKey or energyKey then
        linked = true
    end

    effectiveSlipAngle = safeNumber(effectiveSlipAngle, filteredSlipAngle or 0.0)
    effectiveSlipRatio = safeNumber(effectiveSlipRatio, filteredSlipRatio or 0.0)

    combinedSlip = clamp(
        safeNumber(combinedSlip, estimateCombinedSlip(effectiveSlipAngle, effectiveSlipRatio)),
        0.0,
        M.params.maxCombinedSlip
    )

    slipEnergy = clamp(
        safeNumber(slipEnergy, math.max(combinedSlip - 1.0, 0.0)),
        0.0,
        M.params.maxSlipEnergy
    )

    return effectiveSlipAngle, effectiveSlipRatio, combinedSlip, slipEnergy, linked
end

local function readWheelLoadField(wheel)
    if not wheel then return nil end

    local keys = {
        "load",
        "loadK",
    }

    for _, key in ipairs(keys) do
        local value = safeField(wheel, key, nil)
        if value ~= nil then
            return safeNumber(value, M.params.defaultLoad), "AC_" .. key
        end
    end

    return nil, nil
end

local function readStoreLoad(index)
    local value, key = loadFirstNumber(
        nil,
        "ngp_tire_load_input_" .. index,
        "ngp_tdyn_load_" .. index,
        "ngp_tc_load_" .. index,
        "ngp_cp_load_" .. index,
        "ngp_contact_load_" .. index,
        "ngp_load_path_load_" .. index,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index
    )

    if value ~= nil then
        return value, key
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        return M.params.defaultLoad + safeNumber(dlt, 0.0), "DLT"
    end

    return nil, nil
end

local function getWheelLoad(wheel, index)
    local storeLoad, storeSource = readStoreLoad(index)
    local wheelLoad, wheelSource = readWheelLoadField(wheel)

    if storeLoad ~= nil and wheelLoad ~= nil then
        return
            clamp(
                wheelLoad * (1.0 - M.params.loadBlend) + storeLoad * M.params.loadBlend,
                0.0,
                20000.0
            ),
            "BLEND"
    end

    if storeLoad ~= nil then
        return clamp(storeLoad, 0.0, 20000.0), storeSource or "STORE"
    end

    if wheelLoad ~= nil then
        return clamp(wheelLoad, 0.0, 20000.0), wheelSource or "AC_WHEEL"
    end

    return M.params.defaultLoad, "DEFAULT"
end

local function readContact(index)
    local linked = false

    local contactGrip, gripKey = loadFirstNumber(
        nil,
        "ngp_tire_effective_grip_" .. index,
        "ngp_tcr_effective_grip_" .. index,
        "ngp_cp_grip_" .. index,
        "ngp_tc_grip_" .. index,
        "ngp_contact_grip_" .. index
    )

    local contactLat, latKey = loadFirstNumber(
        nil,
        "ngp_cp_lat_" .. index,
        "ngp_tcp_lat_" .. index,
        "ngp_contact_patch_lat_" .. index
    )

    local contactLong, longKey = loadFirstNumber(
        nil,
        "ngp_cp_long_" .. index,
        "ngp_tcp_long_" .. index,
        "ngp_contact_patch_long_" .. index
    )

    local quality, qualityKey = loadFirstNumber(
        nil,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index,
        "ngp_contact_quality_" .. index,
        "ngp_tc_contact_" .. index
    )

    local loss, lossKey = loadFirstNumber(
        nil,
        "ngp_tire_contact_loss_" .. index,
        "ngp_tcr_contact_loss_" .. index,
        "ngp_contact_loss_" .. index,
        "ngp_tcr_contact_drop_" .. index
    )

    if gripKey or latKey or longKey or qualityKey or lossKey then
        linked = true
    end

    quality = clamp(safeNumber(quality, 1.0), 0.0, 1.2)
    loss = clamp(safeNumber(loss, 1.0 - quality), 0.0, 1.0)

    return
        clamp(safeNumber(contactGrip, 1.0), 0.40, 1.25),
        clamp(safeNumber(contactLat, 1.0), 0.40, 1.25),
        clamp(safeNumber(contactLong, 1.0), 0.40, 1.25),
        quality,
        loss,
        linked
end

local function readDynamics(index)
    local grip, gripKey = loadFirstNumber(
        nil,
        "ngp_tire_grip_" .. index,
        "ngp_tyre_grip_" .. index,
        "ngp_td_grip_" .. index
    )

    local limit, limitKey = loadFirstNumber(
        nil,
        "ngp_tire_limit_" .. index,
        "ngp_tyre_limit_" .. index,
        "ngp_td_limit_" .. index
    )

    local lag, lagKey = loadFirstNumber(
        nil,
        "ngp_rubber_lag_" .. index,
        "ngp_tire_lag_" .. index,
        "ngp_td_lag_" .. index
    )

    return
        clamp(safeNumber(grip, 1.0), 0.0, 1.35),
        clamp(safeNumber(limit, 0.0), 0.0, 2.5),
        clamp(safeNumber(lag, 0.0), -1.0, 2.0),
        gripKey ~= nil or limitKey ~= nil or lagKey ~= nil
end

local function readCompliance(index)
    local responseDelay, delayKey = loadFirstNumber(
        nil,
        "ngp_tire_response_delay",
        "ngp_tyre_response_delay",
        "ngp_tcomp_response_delay",
        "ngp_tire_compliance_response_delay"
    )

    local relaxScale, relaxKey = loadFirstNumber(
        nil,
        "ngp_tire_relax_scale_" .. index,
        "ngp_tcr_relax_scale_" .. index,
        "ngp_tire_response_scale_" .. index,
        "ngp_tcomp_relax_scale_" .. index
    )

    responseDelay = clamp(safeNumber(responseDelay, 0.0), 0.0, 1.0)
    relaxScale = clamp(safeNumber(relaxScale, 1.0), 0.55, 1.55)

    return responseDelay, relaxScale, delayKey ~= nil or relaxKey ~= nil
end

local function readCaster(index)
    local casterGrip, key = loadFirstNumber(
        nil,
        "ngp_caster_grip_" .. index,
        "ngp_caster_effect_grip_" .. index,
        "ngp_steer_caster_grip_" .. index
    )

    return clamp(safeNumber(casterGrip, 1.0), 0.50, 1.10), key ~= nil
end

local function readDrivetrain(index)
    if index < 2 then
        return 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0
    end

    local driveTorque, torqueKey = loadFirstNumber(
        nil,
        "ngp_drive_torque_normalized_from_nm",
        "ngp_drive_torque",
        "ngp_drivetrain_torque",
        "ngp_windup_transmitted"
    )

    local driveTorqueNm, torqueNmKey = loadFirstNumber(
        nil,
        "ngp_drive_torque_nm",
        "ngp_wheel_drive_torque_nm",
        "ngp_drivetrain_torque_nm",
        "ngp_driveline_drive_torque"
    )

    local diffInputTorqueNm, diffNmKey = loadFirstNumber(
        nil,
        "ngp_diff_input_torque_nm",
        "ngp_lsd_input_torque_nm",
        "ngp_lsd_torque_capacity_nm"
    )

    local shaftTwist, shaftKey = loadFirstNumber(
        nil,
        "ngp_shaft_twist",
        "ngp_windup_shaft_twist",
        "ngp_driveline_shaft_twist"
    )

    local driveLash, lashKey = loadFirstNumber(
        nil,
        "ngp_drive_lash",
        "ngp_drivetrain_lash",
        "ngp_windup_lash"
    )

    local lsdLock, lsdKey = loadFirstNumber(
        nil,
        "ngp_lsd_lock",
        "ngp_diff_lock"
    )

    local linked =
        torqueKey ~= nil or torqueNmKey ~= nil or diffNmKey ~= nil or
        shaftKey ~= nil or lashKey ~= nil or lsdKey ~= nil

    return
        safeNumber(driveTorque, 0.0),
        safeNumber(shaftTwist, 0.0),
        clamp(safeNumber(lsdLock, 0.0), 0.0, 1.0),
        linked,
        safeNumber(driveTorqueNm, 0.0),
        safeNumber(diffInputTorqueNm, 0.0),
        safeNumber(driveLash, 0.0)
end

local function readArm(index)
    local camber, camberKey = loadFirstNumber(
        nil,
        "ngp_control_arm_camber_" .. index,
        "ngp_ca_camber_" .. index,
        "ngp_arm_camber_" .. index,
        "ngp_camber_" .. index
    )

    local toe, toeKey = loadFirstNumber(
        nil,
        "ngp_control_arm_toe_" .. index,
        "ngp_ca_toe_" .. index,
        "ngp_arm_toe_" .. index,
        "ngp_toe_" .. index
    )

    return safeNumber(camber, 0.0), safeNumber(toe, 0.0), camberKey ~= nil or toeKey ~= nil
end

local function readHop(index)
    local value, key = loadFirstNumber(
        0.0,
        "ngp_tirehop_energy_" .. index,
        "ngp_tire_hop_energy_" .. index,
        "ngp_tire_hop_" .. index,
        "ngp_susp_hop_" .. index
    )

    return clamp(value, 0.0, 1.0), key ~= nil
end

local function readRF2Root(index)
    local muScale, muKey = loadFirstNumber(nil, "ngp_rf2_mu_scale_" .. index)
    local sliding, slideKey = loadFirstNumber(nil, "ngp_rf2_sliding_" .. index)
    local fy, fyKey = loadFirstNumber(nil, "ngp_rf2_fy_" .. index)

    return
        clamp(safeNumber(muScale, 1.0), 0.35, 1.20),
        clamp(safeNumber(sliding, 0.0), 0.0, 1.0),
        safeNumber(fy, 0.0),
        muKey ~= nil or slideKey ~= nil or fyKey ~= nil
end

local function readUltraChassis(index)
    local muCamber, muKey = loadFirstNumber(nil, "ngp_uc_mu_camber_" .. index)
    local camber, camberKey = loadFirstNumber(nil, "ngp_uc_camber_" .. index)
    local toe, toeKey = loadFirstNumber(nil, "ngp_uc_toe_" .. index)

    return
        clamp(safeNumber(muCamber, 1.0), 0.75, 1.05),
        safeNumber(camber, 0.0),
        safeNumber(toe, 0.0),
        muKey ~= nil or camberKey ~= nil or toeKey ~= nil
end

local function readCarcass(index)
    local support, supportKey = loadFirstNumber(
        nil,
        "ngp_carcass_support_" .. index,
        "ngp_tire_carcass_support_" .. index
    )

    local gripGate, gripKey = loadFirstNumber(
        nil,
        "ngp_carcass_grip_gate_" .. index,
        "ngp_tc_grip_gate_" .. index
    )

    local deformation, defKey = loadFirstNumber(
        nil,
        "ngp_tire_deformation_" .. index,
        "ngp_carcass_deformation_" .. index,
        "ngp_tc_deformation_" .. index
    )

    local delay, delayKey = loadFirstNumber(
        nil,
        "ngp_tire_contact_delay_" .. index,
        "ngp_contact_delay_" .. index,
        "ngp_carcass_recovery_bias_" .. index
    )

    local energy, energyKey = loadFirstNumber(
        nil,
        "ngp_tire_sidewall_energy_" .. index,
        "ngp_sidewall_energy_" .. index,
        "ngp_carcass_hysteresis_" .. index
    )

    return
        clamp(safeNumber(support, 1.0), 0.0, 1.2),
        clamp(safeNumber(gripGate, 1.0), 0.0, 1.2),
        clamp(safeNumber(deformation, 0.0), 0.0, 1.0),
        clamp(safeNumber(delay, 0.0), 0.0, 1.0),
        clamp(safeNumber(energy, 0.0), 0.0, 1.0),
        supportKey ~= nil or gripKey ~= nil or defKey ~= nil or delayKey ~= nil or energyKey ~= nil
end

local function readRecovery(index)
    local recoveryRate, recKey = loadFirstNumber(
        nil,
        "ngp_slip_recovery_rate_" .. index,
        "ngp_recovery_rate_" .. index,
        "ngp_sr_recovery_" .. index
    )

    local gripReturn, gripKey = loadFirstNumber(
        nil,
        "ngp_slip_grip_return_" .. index,
        "ngp_grip_return_" .. index,
        "ngp_sr_grip_return_" .. index
    )

    local snapRisk, snapKey = loadFirstNumber(
        nil,
        "ngp_slip_snap_risk_" .. index,
        "ngp_snap_risk_" .. index,
        "ngp_sr_snap_risk_" .. index
    )

    local dirtyReturn, dirtyKey = loadFirstNumber(
        nil,
        "ngp_slip_dirty_return_" .. index,
        "ngp_dirty_return_" .. index
    )

    return
        clamp(safeNumber(recoveryRate, 0.0), 0.0, 1.0),
        clamp(safeNumber(gripReturn, 0.0), 0.0, 1.0),
        clamp(safeNumber(snapRisk, 0.0), 0.0, 1.0),
        clamp(safeNumber(dirtyReturn, 0.0), 0.0, 1.0),
        recKey ~= nil or gripKey ~= nil or snapKey ~= nil or dirtyKey ~= nil
end

local function readRoadInput(index)
    local severity, sevKey = loadFirstNumber(
        nil,
        "ngp_road_input_severity_" .. index,
        "ngp_rii_severity_" .. index,
        "ngp_rbi_wheel_input_" .. index
    )

    local shock, shockKey = loadFirstNumber(
        nil,
        "ngp_road_impact_" .. index,
        "ngp_rii_shock_" .. index,
        "ngp_impact_wheel_" .. index
    )

    local pathLoss, pathKey = loadFirstNumber(
        nil,
        "ngp_rii_path_loss_" .. index,
        "ngp_road_path_loss_" .. index,
        "ngp_lp_loss_" .. index
    )

    local tireDelivery, delKey = loadFirstNumber(
        nil,
        "ngp_rii_tire_delivery_" .. index,
        "ngp_road_tire_delivery_" .. index
    )

    local loadPathLoss, lpKey = loadFirstNumber(
        nil,
        "ngp_load_path_loss_" .. index,
        "ngp_lp_force_leak_" .. index,
        "ngp_lp_loss_" .. index
    )

    return
        clamp(safeNumber(severity, 0.0), 0.0, 1.5),
        clamp(safeNumber(shock, 0.0), 0.0, 1.5),
        clamp(safeNumber(pathLoss, 0.0), 0.0, 1.0),
        clamp(safeNumber(tireDelivery, 1.0), 0.0, 1.25),
        clamp(safeNumber(loadPathLoss, 0.0), 0.0, 1.0),
        sevKey ~= nil or shockKey ~= nil or pathKey ~= nil or delKey ~= nil,
        lpKey ~= nil
end

local function readThermal(index)
    local stress, stressKey = loadFirstNumber(
        nil,
        "ngp_thermal_stress_" .. index,
        "ngp_virtual_thermal_stress",
        "ngp_brake_root_heat_avg"
    )

    return clamp(safeNumber(stress, 0.0), 0.0, 1.0), stressKey ~= nil
end

--============================================================
-- Force core
--============================================================

local function calculateLoadScale(load)
    return clamp(
        safeNumber(load, M.params.defaultLoad) / math.max(M.params.loadReference, 1.0),
        M.params.minLoadScale,
        M.params.maxLoadScale
    )
end

local function calculateEffectiveLoadScale(loadScale)
    local effective = math.max(loadScale, 0.0) ^ M.params.loadForceExponent
    return clamp(
        effective,
        M.params.minEffectiveLoadScale,
        M.params.maxEffectiveLoadScale
    )
end

local function calculateScale(wheelState)
    local scale = 1.0

    scale = scale * (1.0 + (wheelState.contactGrip - 1.0) * M.params.contactGripInfluence)
    scale = scale * (1.0 + (wheelState.tireGrip - 1.0) * M.params.dynamicsGripInfluence)
    scale = scale * (1.0 - wheelState.tireLimit * M.params.tireLimitForceLoss)
    scale = scale * (1.0 - wheelState.responseDelay * M.params.complianceDelayGain)
    scale = scale * (1.0 - wheelState.contactLoss * M.params.contactLossForceDrop)
    scale = scale * (1.0 + (wheelState.contactQuality - 1.0) * M.params.contactQualityForceGain)
    scale = scale * (1.0 - wheelState.hopEnergy * M.params.hopForceDrop)

    local geometryLoss =
        abs(wheelState.armToe) * M.params.armToeLatLossGain +
        abs(wheelState.armCamber) * M.params.armCamberLatLossGain

    scale = scale * (1.0 - clamp(geometryLoss, 0.0, 0.35))

    scale = scale * (1.0 + (wheelState.rf2MuScale - 1.0) * M.params.rf2MuScaleInfluence)
    scale = scale * (1.0 - wheelState.rf2Sliding * M.params.rf2SlidingForceDrop)
    scale = scale * (1.0 + (wheelState.ucMuCamber - 1.0) * M.params.ultraCamberInfluence)

    scale = scale * (1.0 - math.max((wheelState.combinedSlip or 0.0) - 1.0, 0.0) * M.params.combinedSlipForceDrop)
    scale = scale * (1.0 - (wheelState.slipEnergy or 0.0) * M.params.slipEnergyForceDrop)

    scale = scale * (1.0 + (wheelState.carcassSupport - 1.0) * M.params.carcassSupportInfluence)
    scale = scale * (1.0 + (wheelState.carcassGripGate - 1.0) * M.params.carcassGripGateInfluence)
    scale = scale * (1.0 - wheelState.carcassDeformation * M.params.carcassDeformationDrop)
    scale = scale * (1.0 - wheelState.roadShock * M.params.roadShockForceDrop)
    scale = scale * (1.0 - wheelState.roadPathLoss * M.params.roadPathLossDrop)
    scale = scale * (1.0 + (wheelState.roadTireDelivery - 1.0) * M.params.roadTireDeliveryGain)
    scale = scale * (1.0 - wheelState.loadPathLoss * M.params.loadPathLossDrop)
    scale = scale * (1.0 - wheelState.thermalStress * M.params.thermalStressDrop)
    scale = scale * (1.0 + wheelState.gripReturn * M.params.recoveryGripGain)
    scale = scale * (1.0 - wheelState.snapRisk * M.params.recoverySnapDrop)
    scale = scale * (1.0 - wheelState.dirtyReturn * M.params.recoveryDirtyDrop)

    return clamp(scale, 0.20, 1.40)
end

local function updateWheelForce(index, wheelState, dt)
    local loadScale = wheelState.effectiveLoadScale or wheelState.loadScale or 1.0

    local latScale =
        1.0 +
        (wheelState.contactLat - 1.0) * M.params.contactLatInfluence

    local longScale =
        1.0 +
        (wheelState.contactLong - 1.0) * M.params.contactLongInfluence

    local casterScale =
        1.0 +
        (wheelState.casterGrip - 1.0) * M.params.casterGripInfluence

    local finalScale = calculateScale(wheelState)
    wheelState.finalScale = finalScale

    local driveLongInput = 0.0

    if index >= 2 then
        local diffTorqueNorm = clamp(
            (wheelState.diffInputTorqueNm or 0.0) /
            math.max(M.params.diffInputTorqueReferenceNm, 1.0),
            -1.0,
            1.0
        )

        local driveTorqueNmNorm = clamp(
            (wheelState.driveTorqueNm or 0.0) /
            math.max(M.params.diffInputTorqueReferenceNm, 1.0),
            -1.0,
            1.0
        )

        driveLongInput =
            wheelState.driveTorque * M.params.driveTorqueLongGain +
            driveTorqueNmNorm * M.params.driveTorqueNmLongGain +
            diffTorqueNorm * M.params.diffInputTorqueLongGain +
            wheelState.shaftTwist * M.params.shaftTwistLongGain +
            wheelState.lsdLock * M.params.lsdRearLongGain
    end

    local latSlipInput =
        wheelState.effectiveSlipAngle or
        wheelState.filteredSlipAngle or
        0.0

    local longSlipInput =
        wheelState.effectiveSlipRatio or
        wheelState.filteredSlipRatio or
        0.0

    local combinedBalance = 1.0
    if (wheelState.combinedSlip or 0.0) > 1.0 then
        combinedBalance =
            1.0 /
            (
                1.0 +
                ((wheelState.combinedSlip or 0.0) - 1.0) *
                M.params.combinedSlipBalanceGain
            )
    end

    wheelState.combinedBalance = clamp(combinedBalance, 0.45, 1.0)

    local rawLateral =
        latSlipInput *
        M.params.lateralScale *
        loadScale *
        latScale *
        casterScale *
        finalScale *
        wheelState.combinedBalance

    local rawLongitudinal =
        (
            longSlipInput *
            M.params.longitudinalScale +
            driveLongInput
        ) *
        loadScale *
        longScale *
        finalScale *
        wheelState.combinedBalance

    rawLateral = clamp(rawLateral, -M.params.maxForce, M.params.maxForce)
    rawLongitudinal = clamp(rawLongitudinal, -M.params.maxForce, M.params.maxForce)

    wheelState.rawLateral = rawLateral
    wheelState.rawLongitudinal = rawLongitudinal

    local tauLat =
        M.params.forceTau *
        (
            1.0 +
            abs(wheelState.tireLag) * M.params.tireLagForceDelay +
            (wheelState.slipEnergy or 0.0) * 0.18 +
            (wheelState.carcassDelay or 0.0) * M.params.carcassDelayGain
        )

    if abs(rawLateral) < abs(wheelState.lateral) then
        tauLat = M.params.releaseTau
    end

    wheelState.lateral = lowPass(wheelState.lateral, rawLateral, tauLat, dt)

    local tauLong =
        M.params.forceTau *
        (
            1.0 +
            (wheelState.slipEnergy or 0.0) * 0.12 +
            abs(wheelState.driveLash or 0.0) * 0.08
        )

    if abs(rawLongitudinal) < abs(wheelState.longitudinal) then
        tauLong = M.params.releaseTau
    end

    wheelState.longitudinal = lowPass(wheelState.longitudinal, rawLongitudinal, tauLong, dt)

    wheelState.lateral = clamp(wheelState.lateral, -M.params.maxForce, M.params.maxForce)
    wheelState.longitudinal = clamp(wheelState.longitudinal, -M.params.maxForce, M.params.maxForce)

    wheelState.forceMagnitude =
        math.sqrt(
            wheelState.lateral * wheelState.lateral +
            wheelState.longitudinal * wheelState.longitudinal
        )

    wheelState.roadInput =
        clamp(
            wheelState.forceMagnitude /
            math.max(M.params.roadInputForceReference, 1.0),
            0.0,
            1.5
        )
end

local function decayWheel(index, dt)
    local wheelState = M.lastForce[index]
    local tau = M.params.storeOnlyTau

    wheelState.lateral = lowPass(wheelState.lateral, 0.0, tau, dt)
    wheelState.longitudinal = lowPass(wheelState.longitudinal, 0.0, tau, dt)
    wheelState.rawLateral = lowPass(wheelState.rawLateral, 0.0, tau, dt)
    wheelState.rawLongitudinal = lowPass(wheelState.rawLongitudinal, 0.0, tau, dt)
    wheelState.forceMagnitude =
        math.sqrt(
            wheelState.lateral * wheelState.lateral +
            wheelState.longitudinal * wheelState.longitudinal
        )
    wheelState.roadInput = lowPass(wheelState.roadInput, 0.0, tau, dt)
    wheelState.finalScale = lowPass(wheelState.finalScale, 1.0, tau, dt)
    wheelState.combinedBalance = lowPass(wheelState.combinedBalance, 1.0, tau, dt)
    wheelState.active = false
end

--============================================================
-- Export
--============================================================

local function exportWheel(index, wheelState)
    safeStore("ngp_tire_force_lat_" .. index, wheelState.lateral or 0.0)
    safeStore("ngp_tire_force_long_" .. index, wheelState.longitudinal or 0.0)
    safeStore("ngp_tire_force_lateral_" .. index, wheelState.lateral or 0.0)
    safeStore("ngp_tire_force_longitudinal_" .. index, wheelState.longitudinal or 0.0)
    safeStore("ngp_tire_force_scale_" .. index, wheelState.finalScale or 1.0)

    safeStore("ngp_tf_lat_" .. index, wheelState.lateral or 0.0)
    safeStore("ngp_tf_long_" .. index, wheelState.longitudinal or 0.0)
    safeStore("ngp_tf_scale_" .. index, wheelState.finalScale or 1.0)

    safeStore("ngp_tire_force_combined_" .. index, wheelState.forceMagnitude or 0.0)
    safeStore("ngp_tf_combined_" .. index, wheelState.forceMagnitude or 0.0)

    safeStore("ngp_tire_force_road_input_" .. index, wheelState.roadInput or 0.0)
    safeStore("ngp_tf_road_input_" .. index, wheelState.roadInput or 0.0)

    safeStore("ngp_tire_force_combined_slip_" .. index, wheelState.combinedSlip or 0.0)
    safeStore("ngp_tire_force_slip_energy_" .. index, wheelState.slipEnergy or 0.0)

    safeStore("ngp_tf_combined_slip_" .. index, wheelState.combinedSlip or 0.0)
    safeStore("ngp_tf_slip_energy_" .. index, wheelState.slipEnergy or 0.0)

    safeStore("ngp_tforce_lat_" .. index, wheelState.lateral or 0.0)
    safeStore("ngp_tforce_long_" .. index, wheelState.longitudinal or 0.0)
    safeStore("ngp_tforce_road_" .. index, wheelState.roadInput or 0.0)

    if not M.state.debugStoreNow then return end

    safeStore("ngp_tire_force_raw_lat_" .. index, wheelState.rawLateral or 0.0)
    safeStore("ngp_tire_force_raw_long_" .. index, wheelState.rawLongitudinal or 0.0)
    safeStore("ngp_tire_force_load_" .. index, wheelState.load or 0.0)
    safeStore("ngp_tire_force_load_scale_" .. index, wheelState.loadScale or 0.0)
    safeStore("ngp_tire_force_effective_load_scale_" .. index, wheelState.effectiveLoadScale or wheelState.loadScale or 0.0)
    safeStore("ngp_tire_force_load_source_" .. index, wheelState.loadSource or "UNKNOWN")

    safeStore("ngp_tire_force_effective_slip_angle_" .. index, wheelState.effectiveSlipAngle or 0.0)
    safeStore("ngp_tire_force_effective_slip_ratio_" .. index, wheelState.effectiveSlipRatio or 0.0)
    safeStore("ngp_tire_force_combined_balance_" .. index, wheelState.combinedBalance or 1.0)

    safeStore("ngp_tire_force_grip_" .. index, wheelState.tireGrip or 1.0)
    safeStore("ngp_tire_force_limit_" .. index, wheelState.tireLimit or 0.0)
    safeStore("ngp_tire_force_contact_quality_" .. index, wheelState.contactQuality or 1.0)
    safeStore("ngp_tire_force_contact_loss_" .. index, wheelState.contactLoss or 0.0)

    safeStore("ngp_tire_force_lsd_lock_" .. index, wheelState.lsdLock or 0.0)
    safeStore("ngp_tire_force_drive_torque_" .. index, wheelState.driveTorque or 0.0)
    safeStore("ngp_tire_force_drive_torque_nm_" .. index, wheelState.driveTorqueNm or 0.0)
    safeStore("ngp_tire_force_diff_input_torque_nm_" .. index, wheelState.diffInputTorqueNm or 0.0)
    safeStore("ngp_tire_force_arm_toe_" .. index, wheelState.armToe or 0.0)
    safeStore("ngp_tire_force_arm_camber_" .. index, wheelState.armCamber or 0.0)
    safeStore("ngp_tire_force_hop_" .. index, wheelState.hopEnergy or 0.0)

    safeStore("ngp_tire_force_rf2_mu_scale_" .. index, wheelState.rf2MuScale or 1.0)
    safeStore("ngp_tire_force_rf2_sliding_" .. index, wheelState.rf2Sliding or 0.0)
    safeStore("ngp_tire_force_uc_mu_camber_" .. index, wheelState.ucMuCamber or 1.0)
    safeStore("ngp_tire_force_uc_toe_" .. index, wheelState.ucToe or 0.0)

    safeStore("ngp_tire_force_carcass_support_" .. index, wheelState.carcassSupport or 1.0)
    safeStore("ngp_tire_force_carcass_deformation_" .. index, wheelState.carcassDeformation or 0.0)
    safeStore("ngp_tire_force_recovery_rate_" .. index, wheelState.recoveryRate or 0.0)
    safeStore("ngp_tire_force_snap_risk_" .. index, wheelState.snapRisk or 0.0)
    safeStore("ngp_tire_force_road_severity_" .. index, wheelState.roadSeverity or 0.0)
    safeStore("ngp_tire_force_thermal_stress_" .. index, wheelState.thermalStress or 0.0)
end

local function exportGlobal()
    safeStore("ngp_tire_force_status", M.state.status or "UNKNOWN")
    safeStore("ngp_tire_force_update_count", M.state.updateCount or 0)
    safeStore("ngp_tire_force_wheels_valid", M.state.wheelsValid and 1 or 0)
    safeStore("ngp_tire_force_store_only", M.state.storeOnly and 1 or 0)

    safeStore("ngp_tire_force_avg_lat", M.state.avgLatForce or 0.0)
    safeStore("ngp_tire_force_avg_long", M.state.avgLongForce or 0.0)
    safeStore("ngp_tire_force_avg_scale", M.state.avgForceScale or 1.0)
    safeStore("ngp_tire_force_avg_combined_slip", M.state.avgCombinedSlip or 0.0)
    safeStore("ngp_tire_force_avg_slip_energy", M.state.avgSlipEnergy or 0.0)
    safeStore("ngp_tire_force_avg_road_input", M.state.avgRoadInput or 0.0)
    safeStore("ngp_tire_force_max_magnitude", M.state.maxForceMagnitude or 0.0)
    safeStore("ngp_tire_force_max_road_input", M.state.maxRoadInput or 0.0)
    safeStore("ngp_tire_force_active_count", M.state.activeCount or 0)

    safeStore("ngp_tforce_avg_lat", M.state.avgLatForce or 0.0)
    safeStore("ngp_tforce_avg_long", M.state.avgLongForce or 0.0)
    safeStore("ngp_tforce_avg_road", M.state.avgRoadInput or 0.0)

    if not M.state.debugStoreNow then return end

    safeStore("ngp_tire_force_tire_state_linked", M.state.tireStateLinked and 1 or 0)
    safeStore("ngp_tire_force_contact_linked", M.state.contactLinked and 1 or 0)
    safeStore("ngp_tire_force_dynamics_linked", M.state.dynamicsLinked and 1 or 0)
    safeStore("ngp_tire_force_compliance_linked", M.state.complianceLinked and 1 or 0)
    safeStore("ngp_tire_force_caster_linked", M.state.casterLinked and 1 or 0)
    safeStore("ngp_tire_force_drivetrain_linked", M.state.drivetrainLinked and 1 or 0)
    safeStore("ngp_tire_force_lsd_linked", M.state.lsdLinked and 1 or 0)
    safeStore("ngp_tire_force_arm_linked", M.state.armLinked and 1 or 0)
    safeStore("ngp_tire_force_rf2_linked", M.state.rf2Linked and 1 or 0)
    safeStore("ngp_tire_force_ultra_chassis_linked", M.state.ultraChassisLinked and 1 or 0)
    safeStore("ngp_tire_force_carcass_linked", M.state.carcassLinked and 1 or 0)
    safeStore("ngp_tire_force_recovery_linked", M.state.recoveryLinked and 1 or 0)
    safeStore("ngp_tire_force_road_linked", M.state.roadLinked and 1 or 0)
    safeStore("ngp_tire_force_load_path_linked", M.state.loadPathLinked and 1 or 0)
    safeStore("ngp_tire_force_thermal_linked", M.state.thermalLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i, M.lastForce[i])
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

local function updateWheel(index, wheel, dt)
    local wheelState = M.lastForce[index]
    wheelState.active = wheel ~= nil

    local slipAngle,
          slipRatio,
          filteredSlipAngle,
          filteredSlipRatio,
          tireStateLinked =
        readSlipState(index, wheel)

    wheelState.slipAngle = slipAngle
    wheelState.slipRatio = slipRatio
    wheelState.dSlipAngle = filteredSlipAngle - (wheelState.filteredSlipAngle or 0.0)
    wheelState.dSlipRatio = filteredSlipRatio - (wheelState.filteredSlipRatio or 0.0)
    wheelState.filteredSlipAngle = filteredSlipAngle
    wheelState.filteredSlipRatio = filteredSlipRatio

    local effectiveSlipAngle,
          effectiveSlipRatio,
          combinedSlip,
          slipEnergy,
          tireDynamicsRootLinked =
        readTireDynamicsRoot(index, filteredSlipAngle, filteredSlipRatio)

    wheelState.effectiveSlipAngle = effectiveSlipAngle
    wheelState.effectiveSlipRatio = effectiveSlipRatio
    wheelState.combinedSlip = combinedSlip
    wheelState.slipEnergy = slipEnergy

    local load,
          loadSource =
        getWheelLoad(wheel, index)

    wheelState.load = load
    wheelState.loadSource = loadSource
    wheelState.loadScale = calculateLoadScale(load)
    wheelState.effectiveLoadScale = calculateEffectiveLoadScale(wheelState.loadScale)

    local contactGrip,
          contactLat,
          contactLong,
          contactQuality,
          contactLoss,
          contactLinked =
        readContact(index)

    wheelState.contactGrip = contactGrip
    wheelState.contactLat = contactLat
    wheelState.contactLong = contactLong
    wheelState.contactQuality = contactQuality
    wheelState.contactLoss = contactLoss

    local tireGrip,
          tireLimit,
          tireLag,
          dynamicsLinked =
        readDynamics(index)

    wheelState.tireGrip = tireGrip
    wheelState.tireLimit = tireLimit
    wheelState.tireLag = tireLag

    local responseDelay,
          relaxScale,
          complianceLinked =
        readCompliance(index)

    wheelState.responseDelay = responseDelay
    wheelState.responseScale = relaxScale

    local casterGrip,
          casterLinked =
        readCaster(index)

    wheelState.casterGrip = casterGrip

    local driveTorque,
          shaftTwist,
          lsdLock,
          drivetrainLinked,
          driveTorqueNm,
          diffInputTorqueNm,
          driveLash =
        readDrivetrain(index)

    wheelState.driveTorque = driveTorque
    wheelState.driveTorqueNm = driveTorqueNm
    wheelState.diffInputTorqueNm = diffInputTorqueNm
    wheelState.shaftTwist = shaftTwist
    wheelState.lsdLock = lsdLock
    wheelState.driveLash = driveLash

    local armCamber,
          armToe,
          armLinked =
        readArm(index)

    wheelState.armCamber = armCamber
    wheelState.armToe = armToe

    local hopEnergy,
          hopLinked =
        readHop(index)

    wheelState.hopEnergy = hopEnergy

    local rf2MuScale,
          rf2Sliding,
          rf2Fy,
          rf2Linked =
        readRF2Root(index)

    wheelState.rf2MuScale = rf2MuScale
    wheelState.rf2Sliding = rf2Sliding
    wheelState.rf2Fy = rf2Fy

    local ucMuCamber,
          ucCamber,
          ucToe,
          ultraChassisLinked =
        readUltraChassis(index)

    wheelState.ucMuCamber = ucMuCamber
    wheelState.ucCamber = ucCamber
    wheelState.ucToe = ucToe

    local carcassSupport,
          carcassGripGate,
          carcassDeformation,
          carcassDelay,
          carcassEnergy,
          carcassLinked =
        readCarcass(index)

    wheelState.carcassSupport = carcassSupport
    wheelState.carcassGripGate = carcassGripGate
    wheelState.carcassDeformation = carcassDeformation
    wheelState.carcassDelay = carcassDelay
    wheelState.carcassEnergy = carcassEnergy

    local recoveryRate,
          gripReturn,
          snapRisk,
          dirtyReturn,
          recoveryLinked =
        readRecovery(index)

    wheelState.recoveryRate = recoveryRate
    wheelState.gripReturn = gripReturn
    wheelState.snapRisk = snapRisk
    wheelState.dirtyReturn = dirtyReturn

    local roadSeverity,
          roadShock,
          roadPathLoss,
          roadTireDelivery,
          loadPathLoss,
          roadLinked,
          loadPathLinked =
        readRoadInput(index)

    wheelState.roadSeverity = roadSeverity
    wheelState.roadShock = roadShock
    wheelState.roadPathLoss = roadPathLoss
    wheelState.roadTireDelivery = roadTireDelivery
    wheelState.loadPathLoss = loadPathLoss

    local thermalStress,
          thermalLinked =
        readThermal(index)

    wheelState.thermalStress = thermalStress

    updateWheelForce(index, wheelState, dt)

    M.state.tireStateLinked = M.state.tireStateLinked or tireStateLinked or tireDynamicsRootLinked
    M.state.contactLinked = M.state.contactLinked or contactLinked
    M.state.dynamicsLinked = M.state.dynamicsLinked or dynamicsLinked
    M.state.complianceLinked = M.state.complianceLinked or complianceLinked
    M.state.casterLinked = M.state.casterLinked or casterLinked
    M.state.drivetrainLinked = M.state.drivetrainLinked or drivetrainLinked
    M.state.lsdLinked = M.state.lsdLinked or (lsdLock ~= 0.0)
    M.state.armLinked = M.state.armLinked or armLinked
    M.state.rf2Linked = M.state.rf2Linked or rf2Linked
    M.state.ultraChassisLinked = M.state.ultraChassisLinked or ultraChassisLinked
    M.state.carcassLinked = M.state.carcassLinked or carcassLinked
    M.state.recoveryLinked = M.state.recoveryLinked or recoveryLinked
    M.state.roadLinked = M.state.roadLinked or roadLinked
    M.state.loadPathLinked = M.state.loadPathLinked or loadPathLinked
    M.state.thermalLinked = M.state.thermalLinked or thermalLinked
end

local function resetLinks()
    M.state.tireStateLinked = false
    M.state.contactLinked = false
    M.state.dynamicsLinked = false
    M.state.complianceLinked = false
    M.state.casterLinked = false
    M.state.drivetrainLinked = false
    M.state.lsdLinked = false
    M.state.armLinked = false
    M.state.rf2Linked = false
    M.state.ultraChassisLinked = false
    M.state.carcassLinked = false
    M.state.recoveryLinked = false
    M.state.roadLinked = false
    M.state.loadPathLinked = false
    M.state.thermalLinked = false
end

local function updateAverages()
    local sumLat = 0.0
    local sumLong = 0.0
    local sumScale = 0.0
    local sumCombinedSlip = 0.0
    local sumSlipEnergy = 0.0
    local sumRoadInput = 0.0
    local maxForce = 0.0
    local maxRoad = 0.0
    local activeCount = 0

    for i = 0, 3 do
        local ws = M.lastForce[i]
        sumLat = sumLat + abs(ws.lateral)
        sumLong = sumLong + abs(ws.longitudinal)
        sumScale = sumScale + (ws.finalScale or 1.0)
        sumCombinedSlip = sumCombinedSlip + (ws.combinedSlip or 0.0)
        sumSlipEnergy = sumSlipEnergy + (ws.slipEnergy or 0.0)
        sumRoadInput = sumRoadInput + (ws.roadInput or 0.0)
        maxForce = math.max(maxForce, ws.forceMagnitude or 0.0)
        maxRoad = math.max(maxRoad, ws.roadInput or 0.0)

        if ws.active then
            activeCount = activeCount + 1
        end
    end

    M.state.avgLatForce = sumLat * 0.25
    M.state.avgLongForce = sumLong * 0.25
    M.state.avgForceScale = sumScale * 0.25
    M.state.avgCombinedSlip = sumCombinedSlip * 0.25
    M.state.avgSlipEnergy = sumSlipEnergy * 0.25
    M.state.avgRoadInput = sumRoadInput * 0.25
    M.state.maxForceMagnitude = maxForce
    M.state.maxRoadInput = maxRoad
    M.state.activeCount = activeCount
end

function M.update(dt, car, runtime)
    M.state.updateCount = (M.state.updateCount or 0) + 1

    dt = safeNumber(dt, 0.0)

    if dt <= 0.0 then
        M.state.status = "BAD DT"
        exportGlobal()
        return
    end

    dt = clamp(dt, M.params.minDt, M.params.maxDt)

    updateDebugGate(dt)
    resetLinks()

    car = car or safeGetCar()
    local wheels = safeGetWheels(car)

    if not car then
        M.state.status = "NO CAR"
        M.state.wheelsValid = false
        M.state.storeOnly = true

        for i = 0, 3 do
            updateWheel(i, nil, dt)
            decayWheel(i, dt)
            exportWheel(i, M.lastForce[i])
        end

        updateAverages()
        exportGlobal()
        return
    end

    if not wheels then
        M.state.status = "NO WHEELS"
        M.state.wheelsValid = false
        M.state.storeOnly = true

        for i = 0, 3 do
            updateWheel(i, nil, dt)
            decayWheel(i, dt)
            exportWheel(i, M.lastForce[i])
        end

        updateAverages()
        exportGlobal()
        return
    end

    M.state.status = "RUNNING"
    M.state.wheelsValid = true
    M.state.storeOnly = false

    for i = 0, 3 do
        local wheel = getWheelFromWheels(wheels, i)

        if wheel then
            updateWheel(i, wheel, dt)
        else
            updateWheel(i, nil, dt)
            decayWheel(i, dt)
        end

        exportWheel(i, M.lastForce[i])
    end

    updateAverages()
    exportGlobal()
end

--============================================================
-- Public API
--============================================================

function M.getForce(index)
    if index == nil then
        return M.lastForce
    end
    return M.lastForce[index]
end

function M.getLateral(index)
    local wheelState = M.lastForce[index]
    if not wheelState then return 0.0 end
    return wheelState.lateral or 0.0
end

function M.getLongitudinal(index)
    local wheelState = M.lastForce[index]
    if not wheelState then return 0.0 end
    return wheelState.longitudinal or 0.0
end

function M.getCombined(index)
    local wheelState = M.lastForce[index]
    if not wheelState then return 0.0 end
    return wheelState.forceMagnitude or 0.0
end

function M.getRoadInput(index)
    local wheelState = M.lastForce[index]
    if not wheelState then return 0.0 end
    return wheelState.roadInput or 0.0
end

function M.getCombinedSlip(index)
    local wheelState = M.lastForce[index]
    if not wheelState then return 0.0 end
    return wheelState.combinedSlip or 0.0
end

function M.getSlipEnergy(index)
    local wheelState = M.lastForce[index]
    if not wheelState then return 0.0 end
    return wheelState.slipEnergy or 0.0
end

function M.getState(index)
    if index == nil then
        return M.state
    end
    return M.lastForce[index]
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0
        local wheelState = M.lastForce[i]

        if not wheelState then
            return "tire_force wheel NIL"
        end

        return string.format(
            "%s Lat %+.0f Long %+.0f Scale %.2f Road %.2f\n" ..
            "SlipA %.3f SlipR %.3f EffA %.3f EffR %.3f\n" ..
            "Comb %.2f Energy %.2f Load %.0f/%s Ls %.2f\n" ..
            "Grip %.2f Limit %.2f Contact %.2f Caster %.2f\n" ..
            "Drive %.3f Nm %.0f Tin %.0f LSD %.2f Hop %.3f\n" ..
            "Carcass %.2f Def %.2f Recovery %.2f Snap %.2f",

            WHEEL_NAMES[i] or tostring(i),
            wheelState.lateral or 0.0,
            wheelState.longitudinal or 0.0,
            wheelState.finalScale or 1.0,
            wheelState.roadInput or 0.0,

            wheelState.filteredSlipAngle or 0.0,
            wheelState.filteredSlipRatio or 0.0,
            wheelState.effectiveSlipAngle or 0.0,
            wheelState.effectiveSlipRatio or 0.0,

            wheelState.combinedSlip or 0.0,
            wheelState.slipEnergy or 0.0,
            wheelState.load or 0.0,
            tostring(wheelState.loadSource or "NIL"),
            wheelState.effectiveLoadScale or wheelState.loadScale or 0.0,

            wheelState.tireGrip or 1.0,
            wheelState.tireLimit or 0.0,
            wheelState.contactQuality or 1.0,
            wheelState.casterGrip or 1.0,

            wheelState.driveTorque or 0.0,
            wheelState.driveTorqueNm or 0.0,
            wheelState.diffInputTorqueNm or 0.0,
            wheelState.lsdLock or 0.0,
            wheelState.hopEnergy or 0.0,

            wheelState.carcassSupport or 1.0,
            wheelState.carcassDeformation or 0.0,
            wheelState.recoveryRate or 0.0,
            wheelState.snapRisk or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s / StoreOnly %s\n" ..
        "Lat %.0f %.0f %.0f %.0f / Long %.0f %.0f %.0f %.0f\n" ..
        "Comb %.2f Energy %.2f Road %.2f MaxF %.0f\n" ..
        "Links TS:%s CT:%s TD:%s CP:%s CA:%s DT:%s LSD:%s ARM:%s CAR:%s REC:%s",

        tostring(M.state.status),
        M.state.updateCount or 0,
        M.state.wheelsValid and "OK" or "NIL",
        M.state.storeOnly and "YES" or "NO",

        M.lastForce[0].lateral or 0.0,
        M.lastForce[1].lateral or 0.0,
        M.lastForce[2].lateral or 0.0,
        M.lastForce[3].lateral or 0.0,

        M.lastForce[0].longitudinal or 0.0,
        M.lastForce[1].longitudinal or 0.0,
        M.lastForce[2].longitudinal or 0.0,
        M.lastForce[3].longitudinal or 0.0,

        M.state.avgCombinedSlip or 0.0,
        M.state.avgSlipEnergy or 0.0,
        M.state.avgRoadInput or 0.0,
        M.state.maxForceMagnitude or 0.0,

        M.state.tireStateLinked and "OK" or "NIL",
        M.state.contactLinked and "OK" or "NIL",
        M.state.dynamicsLinked and "OK" or "NIL",
        M.state.complianceLinked and "OK" or "NIL",
        M.state.casterLinked and "OK" or "NIL",
        M.state.drivetrainLinked and "OK" or "NIL",
        M.state.lsdLinked and "OK" or "NIL",
        M.state.armLinked and "OK" or "NIL",
        M.state.carcassLinked and "OK" or "NIL",
        M.state.recoveryLinked and "OK" or "NIL"
    )
end

return M
