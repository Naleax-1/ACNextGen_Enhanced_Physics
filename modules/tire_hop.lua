---@diagnostic disable: undefined-global

--============================================================
-- ACNextGen
-- tire_hop.lua
-- V1.1.5 stable
-- Tire Hop / Wheel Hop Model
--============================================================

local M = {}

local WHEEL_NAMES = {
    [0] = "FL",
    [1] = "FR",
    [2] = "RL",
    [3] = "RR",
}

M.params = {
    buildTau = 0.030,
    decayTau = 0.120,

    oscTau = 0.020,
    oscDamp = 0.88,
    oscRate = 42.0,

    stickSlipLow = 0.055,
    stickSlipHigh = 0.180,
    slipFull = 0.420,
    slipSlopeGain = 2.20,
    slipTransitionGain = 1.80,

    baseVerticalDamping = 8500.0,
    baseLongitudinalStiffness = 42000.0,
    baseGeometryPhi = 0.115,

    driveTorqueGain = 5200.0,
    brakeTorqueGain = 4600.0,

    muSlopeBase = 0.18,
    muSlopeSharpness = 1.35,

    stabilityCritical = 1.00,
    instabilityGain = 1.25,

    driveGain = 0.95,
    brakeGain = 0.75,
    lsdGain = 0.30,

    contactGain = 0.70,
    loadNormGain = 0.35,

    bodyFlexGain = 0.28,
    bodyTorsionGain = 0.25,
    bodyDampingSuppress = 0.22,

    damperSuppressGain = 0.20,
    springSuppressGain = 0.08,

    drivetrainTorqueGain = 0.32,
    shaftTwistGain = 0.14,
    driveLashGain = 0.10,
    lsdDiffGain = 0.08,

    contactLossGain = 0.28,
    responseEnergyGain = 0.20,
    memoryGain = 0.16,

    tireLimitGain = 0.18,
    tireGripSuppressGain = 0.12,

    armToePhiGain = 0.15,
    armCamberPhiGain = 0.10,

    complianceDelayGain = 0.16,

    carcassEnergyGain = 0.18,
    carcassDelayGain = 0.10,
    roadShockGain = 0.18,
    loadPathLossGain = 0.12,
    brakeLockGain = 0.12,

    hopForceGain = 1650.0,
    loadPulseGain = 1350.0,
    contactDropGain = 0.22,

    maxHop = 1.00,
    maxLoadPulse = 2600.0,
    maxVerticalKick = 1.00,

    minSpeedKmh = 1.0,
    lowSpeedBoostKmh = 35.0,

    minLoad = 300.0,
    loadRef = 3500.0,
    maxLoad = 12000.0,

    debugStoreInterval = 0.25,
}

local state = {
    energy = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    osc = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    oscVel = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    hop = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadPulse = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    verticalKick = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    prevSlip = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    slipSlope = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    stickSlip = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    stabilityIndex = { [0] = 99.0, [1] = 99.0, [2] = 99.0, [3] = 99.0 },
    instability = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    torqueInput = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    geometryPhi = { [0] = 0.115, [1] = 0.115, [2] = 0.115, [3] = 0.115 },

    load = { [0] = 3500.0, [1] = 3500.0, [2] = 3500.0, [3] = 3500.0 },
    loadNorm = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    loadVelocity = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    contact = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    contactLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    contactEnergy = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    memory = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireLimit = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireGrip = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    carcassEnergy = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    carcassDelay = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    roadShock = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadPathLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    brakeLock = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    armCamber = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    armToe = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    driveTorque = 0.0,
    driveTorqueNm = 0.0,
    diffInputTorqueNm = 0.0,
    shaftTwist = 0.0,
    driveLash = 0.0,
    lsdLock = 0.0,
    lsdDiff = 0.0,

    speedKmh = 0.0,
    activeCount = 0,

    mode = { [0] = "STABLE", [1] = "STABLE", [2] = "STABLE", [3] = "STABLE" },

    avgHop = 0.0,
    avgEnergy = 0.0,
    avgInstability = 0.0,
    avgLoadPulse = 0.0,
    maxHop = 0.0,
    maxLoadPulse = 0.0,
    maxVerticalKick = 0.0,

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    contactLinked = false,
    memoryLinked = false,
    dynamicsLinked = false,
    armLinked = false,
    drivetrainLinked = false,
    lsdLinked = false,
    carcassLinked = false,
    roadLinked = false,
    loadPathLinked = false,
    brakeLinked = false,
    storeOnly = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

local function clamp(v, minValue, maxValue)
    v = tonumber(v) or 0.0
    if v ~= v then
        return minValue
    end
    if v < minValue then
        return minValue
    end
    if v > maxValue then
        return maxValue
    end
    return v
end

local function abs(v)
    v = tonumber(v) or 0.0
    if v < 0.0 then
        return -v
    end
    return v
end

local function safeNumber(value, defaultValue)
    local n = tonumber(value)
    if n == nil or n ~= n or n == math.huge or n == -math.huge then
        return defaultValue or 0.0
    end
    return n
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

local function safeLoadFirst(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return safeNumber(value, defaultValue or 0.0), true
        end
    end
    return defaultValue or 0.0, false
end

local function safeStore(key, value)
    if not ac or not ac.store then
        return
    end
    pcall(function()
        ac.store(key, value)
    end)
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.001)
    dt = safeNumber(dt, 0.0)
    if tau <= 0.0001 then
        return target
    end
    return current + (target - current) * clamp(dt / (tau + dt), 0.0, 1.0)
end

local function safeField(obj, key, defaultValue)
    if not obj then
        return defaultValue
    end
    local ok, value = pcall(function()
        return obj[key]
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
    if not ok then
        return nil
    end
    return car
end

local function safeGetWheels(car)
    return safeField(car, "wheels", nil)
end

local function readWheel(car, index)
    local wheels = safeGetWheels(car)
    if not wheels then
        return nil
    end
    return safeField(wheels, index, safeField(wheels, index + 1, nil))
end

local function readWheelNumber(wheel, key, defaultValue)
    return safeNumber(safeField(wheel, key, defaultValue), defaultValue or 0.0)
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
    state.contactLinked = false
    state.memoryLinked = false
    state.dynamicsLinked = false
    state.armLinked = false
    state.drivetrainLinked = false
    state.lsdLinked = false
    state.carcassLinked = false
    state.roadLinked = false
    state.loadPathLinked = false
    state.brakeLinked = false
end

local function readSpeedKmh(car)
    local speedKmh = safeNumber(safeField(car, "speedKmh", nil), nil)
    if speedKmh ~= nil then
        return speedKmh
    end
    local speed = safeNumber(nil, nil)
    if speed ~= nil then
        if math.abs(speed) < 120.0 then
            return speed * 3.6
        end
        return speed
    end
    return safeLoad("ngp_speed_kmh", safeLoad("ngp_thermal_speed", 0.0))
end

local function readThrottleBrake(car)
    local throttle = safeNumber(safeField(car, "gas", nil), nil)
    if throttle == nil then
        throttle = safeNumber(nil, nil)
    end
    if throttle == nil then
        throttle = safeLoad("ngp_input_throttle", 0.0)
    end

    local brake = safeNumber(safeField(car, "brake", nil), nil)
    if brake == nil then
        brake = safeLoad("ngp_input_brake", 0.0)
    end

    return clamp(throttle or 0.0, 0.0, 1.0), clamp(brake or 0.0, 0.0, 1.0)
end

local function readLoad(index, wheel, dt)
    local rawDirect = nil

    if wheel then
        rawDirect = safeNumber(safeField(wheel, "load", nil), nil)
        if rawDirect == nil then rawDirect = safeNumber(safeField(wheel, "loadK", nil), nil) end
        if rawDirect == nil then rawDirect = safeNumber(nil, nil) end
        if rawDirect == nil then rawDirect = safeNumber(nil, nil) end
        if rawDirect == nil then rawDirect = safeNumber(nil, nil) end
    end

    local value, linked = safeLoadFirst(nil,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index,
        "ngp_contact_load_" .. index,
        "ngp_tire_carcass_load_" .. index,
        "ngp_tire_force_load_" .. index,
        "ngp_tdyn_load_" .. index,
        "ngp_tire_load_" .. index,
        "ngp_tire_load_input_" .. index,
        "ngp_load_path_wheel_load_" .. index,
        "ngp_lp_load_" .. index
    )

    local load
    if linked and value ~= nil then
        load = value
    elseif rawDirect ~= nil then
        load = rawDirect
    else
        local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
        if dlt ~= nil then
            linked = true
            load = M.params.loadRef + safeNumber(dlt, 0.0)
        else
            load = state.load[index] or M.params.loadRef
        end
    end

    load = clamp(load, 0.0, M.params.maxLoad)
    state.loadVelocity[index] = (load - (state.load[index] or load)) / math.max(dt, 0.0001)
    state.load[index] = lowPass(state.load[index] or load, load, 0.045, dt)

    local normRaw = safeLoadRaw("ngp_wheel_load_norm_" .. index)
    if normRaw == nil then
        normRaw = safeLoadRaw("ngp_brake_load_norm_" .. index)
    end

    if normRaw ~= nil then
        state.loadNorm[index] = clamp(safeNumber(normRaw, 1.0), 0.0, 2.5)
        linked = true
    else
        state.loadNorm[index] = clamp((state.load[index] or M.params.loadRef) / math.max(M.params.loadRef, 1.0), 0.0, 2.5)
    end

    if linked then
        state.loadPathLinked = true
    end

    return state.load[index], state.loadNorm[index], linked
end

local function readSlipInputs(index, wheel)
    local sr, srLinked = safeLoadFirst(nil,
        "ngp_tire_force_effective_slip_ratio_" .. index,
        "ngp_tdyn_effective_slip_ratio_" .. index,
        "ngp_tire_effective_slip_ratio_" .. index,
        "ngp_tire_slip_ratio_" .. index,
        "ngp_slip_ratio_" .. index,
        "ngp_filtered_slip_ratio_" .. index,
        "ngp_cp_slip_ratio_" .. index,
        "ngp_tc_slip_ratio_" .. index
    )

    local sa, saLinked = safeLoadFirst(nil,
        "ngp_tire_force_effective_slip_angle_" .. index,
        "ngp_tdyn_effective_slip_angle_" .. index,
        "ngp_tire_effective_slip_angle_" .. index,
        "ngp_tire_slip_angle_" .. index,
        "ngp_slip_angle_" .. index,
        "ngp_filtered_slip_angle_" .. index,
        "ngp_cp_slip_angle_" .. index,
        "ngp_tc_slip_angle_" .. index
    )

    if sr == nil then
        sr = readWheelNumber(wheel, "slipRatio", 0.0)
    end

    if sa == nil then
        sa = readWheelNumber(wheel, "slipAngle", 0.0)
    end

    local combined, combinedLinked = safeLoadFirst(nil,
        "ngp_tire_force_combined_slip_" .. index,
        "ngp_tdyn_combined_slip_" .. index,
        "ngp_tire_dynamics_combined_slip_" .. index,
        "ngp_tire_combined_slip_" .. index,
        "ngp_contact_combined_slip_" .. index,
        "ngp_tire_carcass_combined_slip_" .. index,
        "ngp_recovery_slip_" .. index
    )

    local energy, energyLinked = safeLoadFirst(nil,
        "ngp_tire_force_slip_energy_" .. index,
        "ngp_tdyn_slip_energy_" .. index,
        "ngp_tire_dynamics_slip_energy_" .. index,
        "ngp_tire_slip_energy_" .. index,
        "ngp_tc_energy_" .. index,
        "ngp_tcr_energy_push_" .. index
    )

    local angle = abs(sa)
    local angleNorm
    if angle <= math.pi then
        angleNorm = angle / 0.174533
    else
        angleNorm = angle / 10.0
    end

    local ratioNorm = abs(sr) / math.max(M.params.slipFull, 0.001)
    local fallbackCombined = math.sqrt(ratioNorm * ratioNorm + angleNorm * angleNorm * 0.16)

    combined = clamp(safeNumber(combined, fallbackCombined), 0.0, 3.0)
    energy = clamp(safeNumber(energy, math.max(combined - 1.0, 0.0)), 0.0, 1.0)

    if srLinked or saLinked or combinedLinked or energyLinked then
        state.dynamicsLinked = true
    end

    return abs(sr), abs(sa), combined, energy
end

local function readRootState(index)
    local contact, contactLinked = safeLoadFirst(nil,
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index,
        "ngp_tc_contact_" .. index,
        "ngp_tire_contact_" .. index,
        "ngp_carcass_support_" .. index,
        "ngp_carcass_grip_gate_" .. index
    )

    if contactLinked then
        state.contactLinked = true
    end

    state.contact[index] = clamp(safeNumber(contact, 1.0), 0.0, 1.2)

    local loss, lossLinked = safeLoadFirst(nil,
        "ngp_contact_loss_" .. index,
        "ngp_tire_contact_loss_" .. index,
        "ngp_tcr_contact_loss_" .. index,
        "ngp_contact_hop_drop_" .. index
    )

    if lossLinked then
        state.contactLinked = true
    end

    state.contactLoss[index] = clamp(safeNumber(loss, 1.0 - math.min(state.contact[index], 1.0)), 0.0, 1.0)

    local energy, energyLinked = safeLoadFirst(nil,
        "ngp_tcr_energy_push_" .. index,
        "ngp_tc_energy_" .. index,
        "ngp_tire_slip_energy_" .. index,
        "ngp_tdyn_slip_energy_" .. index,
        "ngp_tire_force_slip_energy_" .. index
    )

    if energyLinked then
        state.contactLinked = true
    end

    state.contactEnergy[index] = clamp(safeNumber(energy, 0.0), 0.0, 1.0)

    local mem, memLinked = safeLoadFirst(nil,
        "ngp_tire_memory_" .. index,
        "ngp_rubber_memory_" .. index,
        "ngp_memory_" .. index,
        "ngp_carcass_history_stress_" .. index,
        "ngp_slip_slide_memory_" .. index,
        "ngp_slide_memory_" .. index
    )

    if memLinked then
        state.memoryLinked = true
    end

    state.memory[index] = clamp(safeNumber(mem, 0.0), 0.0, 1.0)

    local limit, gripLinkedA = safeLoadFirst(nil,
        "ngp_tire_limit_" .. index,
        "ngp_tyre_limit_" .. index,
        "ngp_tdyn_limit_" .. index
    )

    local grip, gripLinkedB = safeLoadFirst(nil,
        "ngp_tire_grip_" .. index,
        "ngp_tyre_grip_" .. index,
        "ngp_tdyn_effective_grip_" .. index,
        "ngp_tire_effective_grip_" .. index,
        "ngp_tcr_effective_grip_" .. index
    )

    if gripLinkedA or gripLinkedB then
        state.dynamicsLinked = true
    end

    state.tireLimit[index] = clamp(safeNumber(limit, 0.0), 0.0, 2.0)
    state.tireGrip[index] = clamp(safeNumber(grip, 1.0), 0.0, 1.25)

    local carcassEnergy, carcassLinkedA = safeLoadFirst(nil,
        "ngp_tire_sidewall_energy_" .. index,
        "ngp_sidewall_energy_" .. index,
        "ngp_tire_carcass_avg_energy",
        "ngp_carcass_hysteresis_" .. index,
        "ngp_carcass_vertical_norm_" .. index
    )

    local carcassDelay, carcassLinkedB = safeLoadFirst(nil,
        "ngp_tire_contact_delay_" .. index,
        "ngp_contact_delay_" .. index,
        "ngp_carcass_recovery_bias_" .. index
    )

    if carcassLinkedA or carcassLinkedB then
        state.carcassLinked = true
    end

    state.carcassEnergy[index] = clamp(safeNumber(carcassEnergy, 0.0), 0.0, 1.0)
    state.carcassDelay[index] = clamp(safeNumber(carcassDelay, 0.0), 0.0, 1.0)

    local roadShock, roadLinked = safeLoadFirst(nil,
        "ngp_road_impact_" .. index,
        "ngp_road_input_severity_" .. index,
        "ngp_rii_severity_" .. index,
        "ngp_rbi_wheel_input_" .. index,
        "ngp_body_road_wheel_input_" .. index
    )

    if roadLinked then
        state.roadLinked = true
    end

    state.roadShock[index] = clamp(safeNumber(roadShock, 0.0), 0.0, 1.5)

    local pathLoss, pathLinked = safeLoadFirst(nil,
        "ngp_load_path_loss_" .. index,
        "ngp_lp_loss_" .. index,
        "ngp_load_path_force_leak_" .. index,
        "ngp_rii_path_loss_" .. index
    )

    if pathLinked then
        state.loadPathLinked = true
    end

    state.loadPathLoss[index] = clamp(safeNumber(pathLoss, 0.0), 0.0, 1.0)

    local brakeLock, brakeLinked = safeLoadFirst(nil,
        "ngp_brake_lock_" .. index,
        "ngp_brake_lock_state_" .. index
    )

    if brakeLinked then
        state.brakeLinked = true
    end

    state.brakeLock[index] = clamp(safeNumber(brakeLock, 0.0), 0.0, 1.0)

    local camber, camberLinked = safeLoadFirst(nil,
        "ngp_control_arm_camber_" .. index,
        "ngp_arm_camber_" .. index,
        "ngp_compliance_virtual_camber_" .. index,
        "ngp_virtual_camber_" .. index,
        "ngp_uc_camber_" .. index
    )

    local toe, toeLinked = safeLoadFirst(nil,
        "ngp_control_arm_toe_" .. index,
        "ngp_arm_toe_" .. index,
        "ngp_compliance_virtual_toe_" .. index,
        "ngp_virtual_toe_" .. index,
        "ngp_uc_toe_" .. index
    )

    if camberLinked or toeLinked then
        state.armLinked = true
    end

    state.armCamber[index] = safeNumber(camber, 0.0)
    state.armToe[index] = safeNumber(toe, 0.0)
end

local function readDrivetrainState()
    local driveTorque, driveLinkedA = safeLoadFirst(nil,
        "ngp_drive_torque_normalized_from_nm",
        "ngp_drive_torque",
        "ngp_drivetrain_torque",
        "ngp_dt_torque"
    )

    local driveTorqueNm, driveLinkedB = safeLoadFirst(nil,
        "ngp_drive_torque_nm",
        "ngp_wheel_drive_torque_nm",
        "ngp_drivetrain_torque_nm"
    )

    local diffInputTorqueNm, driveLinkedC = safeLoadFirst(nil,
        "ngp_diff_input_torque_nm",
        "ngp_lsd_input_torque_nm"
    )

    local shaftTwist, driveLinkedD = safeLoadFirst(nil,
        "ngp_shaft_twist",
        "ngp_windup_shaft_twist",
        "ngp_driveline_shaft_twist"
    )

    local driveLash, driveLinkedE = safeLoadFirst(nil,
        "ngp_drive_lash",
        "ngp_windup_lash",
        "ngp_driveline_lash"
    )

    if driveLinkedA or driveLinkedB or driveLinkedC or driveLinkedD or driveLinkedE then
        state.drivetrainLinked = true
    end

    state.driveTorque = safeNumber(driveTorque, 0.0)
    state.driveTorqueNm = safeNumber(driveTorqueNm, 0.0)
    state.diffInputTorqueNm = safeNumber(diffInputTorqueNm, 0.0)
    state.shaftTwist = safeNumber(shaftTwist, 0.0)
    state.driveLash = safeNumber(driveLash, 0.0)

    local lsdLock, lsdLinkedA = safeLoadFirst(nil,
        "ngp_lsd_lock",
        "ngp_diff_lock"
    )

    local lsdDiff, lsdLinkedB = safeLoadFirst(nil,
        "ngp_lsd_diff",
        "ngp_diff_diff",
        "ngp_thermal_lsd_diff"
    )

    if lsdLinkedA or lsdLinkedB then
        state.lsdLinked = true
    end

    state.lsdLock = clamp(safeNumber(lsdLock, 0.0), 0.0, 1.0)
    state.lsdDiff = safeNumber(lsdDiff, 0.0)
end

local function readInputs(car, wheel, index, dt)
    local slipRatio, slipAngle, combinedSlip, slipEnergy = readSlipInputs(index, wheel)
    readLoad(index, wheel, dt)
    readRootState(index)

    local slip = math.max(
        abs(slipRatio),
        combinedSlip * 0.18,
        abs(slipAngle) * (abs(slipAngle) <= math.pi and 1.8 or 0.030)
    )

    slip = clamp(slip, 0.0, 3.0)

    local prev = state.prevSlip[index] or 0.0
    local slope = abs(slip - prev) / math.max(dt, 0.0001)

    state.slipSlope[index] =
        lowPass(
            state.slipSlope[index],
            slope,
            0.025,
            dt
        )

    state.prevSlip[index] = slip

    state.contactEnergy[index] = math.max(state.contactEnergy[index] or 0.0, slipEnergy * 0.65)

    return slip, slipRatio, slipAngle, combinedSlip, slipEnergy
end

local function readTorqueInput(car, index, slip)
    local p = M.params
    local throttle, brake = readThrottleBrake(car)

    local driveBias = 0.35
    if index >= 2 then
        driveBias = 1.00
    end

    local brakeBias = 0.70
    if index <= 1 then
        brakeBias = 1.00
    end

    local driveTorque =
        throttle *
        p.driveTorqueGain *
        driveBias *
        (1.0 + state.lsdLock * p.lsdGain)

    if index >= 2 then
        local torqueNmNorm =
            clamp(abs(state.driveTorqueNm) / 3000.0, 0.0, 1.5)

        local diffInputNorm =
            clamp(abs(state.diffInputTorqueNm) / 3000.0, 0.0, 1.5)

        driveTorque =
            driveTorque
            + abs(state.driveTorque) * p.drivetrainTorqueGain
            + torqueNmNorm * 3800.0 * 0.12
            + diffInputNorm * 3800.0 * 0.12
            + abs(state.shaftTwist) * p.shaftTwistGain
            + abs(state.driveLash) * p.driveLashGain
            + abs(state.lsdDiff) * p.lsdDiffGain
    end

    local brakeTorque =
        brake *
        p.brakeTorqueGain *
        brakeBias

    local torque =
        (
            driveTorque * p.driveGain
            + brakeTorque * p.brakeGain
        )
        *
        (
            0.65
            + clamp(slip / math.max(p.slipFull, 0.001), 0.0, 1.0) * 0.35
        )

    torque =
        torque *
        (
            1.0
            + state.contactLoss[index] * 0.18
            + state.memory[index] * 0.10
            + state.brakeLock[index] * p.brakeLockGain
        )

    state.torqueInput[index] = torque

    return torque
end

local function readBodyAndSuspensionModifiers(index)
    local p = M.params

    local flex =
        clamp(
            safeLoad("ngp_body_flex_factor", 1.0),
            0.25,
            1.80
        )

    local torsion =
        clamp(
            safeLoad("ngp_body_torsion_factor", 1.0),
            0.35,
            2.00
        )

    local damping =
        clamp(
            safeLoad("ngp_body_damping_factor", 1.0),
            0.50,
            1.80
        )

    local damperScale =
        clamp(
            abs(safeLoad("ngp_damper_coeff_scale_" .. index, safeLoad("ngp_susp_int_damper_" .. index, 1.0)) - 1.0),
            0.0,
            1.0
        )

    local springScale =
        clamp(
            abs(safeLoad("ngp_spring_rate_scale_" .. index, safeLoad("ngp_susp_int_spring_" .. index, 1.0)) - 1.0),
            0.0,
            1.0
        )

    local flexAmplify =
        1.0
        + math.max(flex - 1.0, 0.0) * p.bodyFlexGain
        + math.max(torsion - 1.0, 0.0) * p.bodyTorsionGain

    local dampingSuppress =
        1.0
        /
        (
            1.0
            + math.max(damping - 1.0, 0.0) * p.bodyDampingSuppress
            + damperScale * p.damperSuppressGain
            + springScale * p.springSuppressGain
        )

    return flexAmplify, dampingSuppress
end

local function calculateStickSlip(index, slip, dt)
    local p = M.params

    local slipWindow =
        clamp(
            (slip - p.stickSlipLow) /
            math.max(p.stickSlipHigh - p.stickSlipLow, 0.001),
            0.0,
            1.0
        )

    local slopePart =
        clamp(
            state.slipSlope[index] * 0.018 * p.slipSlopeGain,
            0.0,
            1.0
        )

    local stickSlip =
        slipWindow *
        (
            0.45
            + slopePart * 0.55
        )

    stickSlip =
        stickSlip
        + state.contactLoss[index] * 0.10
        + state.contactEnergy[index] * 0.08
        + state.tireLimit[index] * 0.08
        + state.roadShock[index] * 0.08
        + state.brakeLock[index] * 0.08

    state.stickSlip[index] =
        lowPass(
            state.stickSlip[index],
            clamp(stickSlip, 0.0, 1.0),
            0.035,
            dt
        )

    return state.stickSlip[index]
end

local function calculateStabilityIndex(index, torqueInput, stickSlip)
    local p = M.params

    local loadNorm =
        clamp(
            state.loadNorm[index] or 1.0,
            0.10,
            2.50
        )

    local contact =
        clamp(
            state.contact[index] or 1.0,
            0.0,
            1.2
        )

    local damperAbs =
        abs(
            safeLoad("ngp_damper_" .. index, safeLoad("ngp_susp_damper_" .. index, 0.0))
        )

    local cz =
        p.baseVerticalDamping *
        (0.80 + loadNorm * 0.20) *
        (0.70 + contact * 0.30) +
        damperAbs * 0.15

    local rigidity =
        clamp(
            safeLoad("ngp_body_rigidity", 1.0),
            0.20,
            1.35
        )

    local flex =
        clamp(
            safeLoad("ngp_body_flex_factor", 1.0),
            0.25,
            1.80
        )

    local kx =
        p.baseLongitudinalStiffness *
        rigidity /
        math.max(flex, 0.25)

    kx =
        kx /
        (
            1.0
            + state.memory[index] * 0.15
            + state.contactLoss[index] * 0.20
            + state.carcassDelay[index] * 0.10
        )

    local phi = p.baseGeometryPhi

    if index >= 2 then
        phi = phi * 1.18
    end

    phi =
        phi *
        (
            1.0
            + abs(safeLoad("ngp_susp_contact_input_" .. index, 0.0)) * p.contactGain
            + abs(state.armToe[index]) * p.armToePhiGain
            + abs(state.armCamber[index]) * p.armCamberPhiGain
            + state.loadPathLoss[index] * p.loadPathLossGain
        )

    state.geometryPhi[index] = phi

    local muSlope =
        p.muSlopeBase
        + stickSlip * p.muSlopeSharpness
        + clamp(state.slipSlope[index] * 0.004, 0.0, 0.80)
        + state.contactLoss[index] * 0.12
        + state.memory[index] * 0.10
        + state.brakeLock[index] * 0.10

    local denominator =
        math.max(
            phi * math.max(torqueInput, 1.0) * muSlope,
            1.0
        )

    local stabilityIndex =
        (cz * kx) /
        denominator

    stabilityIndex =
        stabilityIndex /
        160000.0

    stabilityIndex =
        clamp(stabilityIndex, 0.0, 5.0)

    state.stabilityIndex[index] = stabilityIndex

    local instability =
        clamp(
            (p.stabilityCritical - stabilityIndex) * p.instabilityGain,
            0.0,
            1.0
        )

    instability =
        instability *
        (
            0.30
            + clamp(contact, 0.0, 1.0) * 0.70
        )

    state.instability[index] = instability

    return instability
end

local function updateOscillator(index, target, dt)
    local p = M.params

    local x = state.osc[index] or 0.0
    local v = state.oscVel[index] or 0.0

    local accel = (target - x) * p.oscRate

    v = (v + accel * dt) * p.oscDamp
    x = x + v * dt

    x = clamp(x, -1.0, 1.0)
    v = clamp(v, -8.0, 8.0)

    state.osc[index] = x
    state.oscVel[index] = v

    return x
end

local function exportWheel(index)
    local contactDrop =
        clamp(
            state.verticalKick[index] *
            M.params.contactDropGain,
            0.0,
            1.0
        )

    safeStore("ngp_tirehop_" .. index, state.loadPulse[index])
    safeStore("ngp_tire_hop_" .. index, state.hop[index])
    safeStore("ngp_tire_hop_energy_" .. index, state.energy[index])
    safeStore("ngp_tire_hop_loadpulse_" .. index, state.loadPulse[index])
    safeStore("ngp_tire_hop_kick_" .. index, state.verticalKick[index])
    safeStore("ngp_susp_tire_hop_input_" .. index, state.verticalKick[index])
    safeStore("ngp_susp_tire_hop_load_" .. index, state.loadPulse[index])
    safeStore("ngp_contact_hop_drop_" .. index, contactDrop)

    safeStore("ngp_thop_hop_" .. index, state.hop[index])
    safeStore("ngp_thop_energy_" .. index, state.energy[index])
    safeStore("ngp_thop_loadpulse_" .. index, state.loadPulse[index])
    safeStore("ngp_thop_kick_" .. index, state.verticalKick[index])
    safeStore("ngp_hop_risk_" .. index, state.energy[index])
    safeStore("ngp_susp_road_impulse_" .. index, math.max(safeLoad("ngp_susp_road_impulse_" .. index, 0.0), state.verticalKick[index] * 0.25))

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_tire_hop_instability_" .. index, state.instability[index])
    safeStore("ngp_tire_hop_stability_index_" .. index, state.stabilityIndex[index])
    safeStore("ngp_tire_hop_stickslip_" .. index, state.stickSlip[index])
    safeStore("ngp_tire_hop_slipslope_" .. index, state.slipSlope[index])
    safeStore("ngp_tire_hop_torque_" .. index, state.torqueInput[index])
    safeStore("ngp_tire_hop_phi_" .. index, state.geometryPhi[index])
    safeStore("ngp_tire_hop_mode_" .. index, state.mode[index])
    safeStore("ngp_tire_hop_contact_loss_" .. index, state.contactLoss[index])
    safeStore("ngp_tire_hop_memory_" .. index, state.memory[index])
    safeStore("ngp_tire_hop_lsd_" .. index, state.lsdLock)
    safeStore("ngp_tire_hop_load_" .. index, state.load[index])
    safeStore("ngp_tire_hop_load_norm_" .. index, state.loadNorm[index])
    safeStore("ngp_tire_hop_load_velocity_" .. index, state.loadVelocity[index])
    safeStore("ngp_tire_hop_road_shock_" .. index, state.roadShock[index])
    safeStore("ngp_tire_hop_carcass_energy_" .. index, state.carcassEnergy[index])
end

local function updateWheel(index, car, wheel, dt, speedKmh)
    local p = M.params

    local slip = readInputs(car, wheel, index, dt)

    local torqueInput =
        readTorqueInput(
            car,
            index,
            slip
        )

    local stickSlip =
        calculateStickSlip(
            index,
            slip,
            dt
        )

    local instability =
        calculateStabilityIndex(
            index,
            torqueInput,
            stickSlip
        )

    local flexAmplify, dampingSuppress =
        readBodyAndSuspensionModifiers(
            index
        )

    local speedBoost = 1.0

    if speedKmh < p.lowSpeedBoostKmh then
        speedBoost =
            1.0
            + (1.0 - speedKmh / math.max(p.lowSpeedBoostKmh, 1.0)) * 0.35
    end

    local loadNorm =
        clamp(
            state.loadNorm[index],
            0.0,
            2.5
        )

    local loadPart =
        clamp(
            loadNorm * p.loadNormGain,
            0.0,
            1.0
        )

    local contactPart =
        clamp(
            state.contact[index],
            0.0,
            1.0
        )

    local disturbance =
        1.0
        + state.contactLoss[index] * p.contactLossGain
        + state.contactEnergy[index] * p.responseEnergyGain
        + state.memory[index] * p.memoryGain
        + state.tireLimit[index] * p.tireLimitGain
        + (1.0 - clamp(state.tireGrip[index], 0.0, 1.0)) * p.tireGripSuppressGain
        + abs(safeLoad("ngp_tire_response_delay", safeLoad("ngp_tyre_response_delay", 0.0))) * p.complianceDelayGain
        + state.carcassEnergy[index] * p.carcassEnergyGain
        + state.carcassDelay[index] * p.carcassDelayGain
        + state.roadShock[index] * p.roadShockGain
        + state.loadPathLoss[index] * p.loadPathLossGain
        + state.brakeLock[index] * p.brakeLockGain

    local targetEnergy =
        stickSlip
        * instability
        * flexAmplify
        * dampingSuppress
        * speedBoost
        * disturbance
        * (0.50 + loadPart * 0.50)
        * (0.45 + contactPart * 0.55)

    if (state.load[index] or 0.0) < p.minLoad then
        targetEnergy = targetEnergy * 0.25
    end

    targetEnergy =
        clamp(
            targetEnergy,
            0.0,
            p.maxHop
        )

    local tau =
        targetEnergy > state.energy[index]
        and p.buildTau
        or p.decayTau

    state.energy[index] =
        lowPass(
            state.energy[index],
            targetEnergy,
            tau,
            dt
        )

    local osc =
        updateOscillator(
            index,
            state.energy[index],
            dt
        )

    local hop =
        clamp(
            abs(osc) * state.energy[index],
            0.0,
            p.maxHop
        )

    state.hop[index] = hop

    state.verticalKick[index] =
        clamp(
            hop * (0.65 + instability * 0.35),
            0.0,
            p.maxVerticalKick
        )

    state.loadPulse[index] =
        clamp(
            hop * p.loadPulseGain * (0.65 + loadNorm * 0.35),
            0.0,
            p.maxLoadPulse
        )

    if hop < 0.04 then
        state.mode[index] = "STABLE"
    elseif instability > 0.65 and stickSlip > 0.45 then
        state.mode[index] = "HOP"
    elseif stickSlip > 0.30 then
        state.mode[index] = "STICK-SLIP"
    else
        state.mode[index] = "BUILD"
    end

    exportWheel(index)
end

local function exportGlobal()
    safeStore("ngp_tire_hop_avg", state.avgHop)
    safeStore("ngp_tire_hop_avg_energy", state.avgEnergy)
    safeStore("ngp_tire_hop_avg_instability", state.avgInstability)
    safeStore("ngp_tire_hop_status", state.status)
    safeStore("ngp_tire_hop_update_count", state.updateCount)
    safeStore("ngp_tire_hop_wheels_valid", state.wheelsValid and 1 or 0)

    safeStore("ngp_thop_avg", state.avgHop)
    safeStore("ngp_thop_avg_energy", state.avgEnergy)
    safeStore("ngp_thop_avg_instability", state.avgInstability)
    safeStore("ngp_thop_avg_loadpulse", state.avgLoadPulse)
    safeStore("ngp_thop_max_hop", state.maxHop)
    safeStore("ngp_thop_max_loadpulse", state.maxLoadPulse)
    safeStore("ngp_thop_max_kick", state.maxVerticalKick)
    safeStore("ngp_thop_active_count", state.activeCount)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_tire_hop_contact_linked", state.contactLinked and 1 or 0)
    safeStore("ngp_tire_hop_memory_linked", state.memoryLinked and 1 or 0)
    safeStore("ngp_tire_hop_dynamics_linked", state.dynamicsLinked and 1 or 0)
    safeStore("ngp_tire_hop_arm_linked", state.armLinked and 1 or 0)
    safeStore("ngp_tire_hop_drivetrain_linked", state.drivetrainLinked and 1 or 0)
    safeStore("ngp_tire_hop_lsd_linked", state.lsdLinked and 1 or 0)
    safeStore("ngp_tire_hop_carcass_linked", state.carcassLinked and 1 or 0)
    safeStore("ngp_tire_hop_road_linked", state.roadLinked and 1 or 0)
    safeStore("ngp_tire_hop_load_path_linked", state.loadPathLinked and 1 or 0)
    safeStore("ngp_tire_hop_brake_linked", state.brakeLinked and 1 or 0)
    safeStore("ngp_tire_hop_store_only", state.storeOnly and 1 or 0)
    safeStore("ngp_tire_hop_speed_kmh", state.speedKmh or 0.0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i)
    end
    exportGlobal()
end

function M.init()
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = safeNumber(dt, 0.0)

    if dt <= 0.0 then
        state.status = "BAD DT"
        exportGlobal()
        return
    end

    dt = clamp(dt, 0.0001, 0.100)

    updateDebugGate(dt)
    resetLinks()

    car = car or safeGetCar()

    local wheels = safeGetWheels(car)
    local hasWheels = wheels ~= nil

    state.wheelsValid = hasWheels
    state.storeOnly = not hasWheels

    if hasWheels then
        state.status = "RUNNING"
    else
        state.status = car and "NO WHEELS" or "STORE ONLY"
    end

    readDrivetrainState()

    local speedKmh =
        readSpeedKmh(
            car
        )

    state.speedKmh =
        speedKmh

    local sumHop = 0.0
    local sumEnergy = 0.0
    local sumInstability = 0.0
    local sumLoadPulse = 0.0
    local maxHop = 0.0
    local maxLoadPulse = 0.0
    local maxVerticalKick = 0.0
    local activeCount = 0

    for i = 0, 3 do
        local wheel =
            readWheel(
                car,
                i
            )

        updateWheel(
            i,
            car,
            wheel,
            dt,
            speedKmh
        )

        if wheel or state.storeOnly then
            activeCount = activeCount + 1
        end

        sumHop = sumHop + (state.hop[i] or 0.0)
        sumEnergy = sumEnergy + (state.energy[i] or 0.0)
        sumInstability = sumInstability + (state.instability[i] or 0.0)
        sumLoadPulse = sumLoadPulse + (state.loadPulse[i] or 0.0)

        if (state.hop[i] or 0.0) > maxHop then
            maxHop = state.hop[i] or 0.0
        end

        if (state.loadPulse[i] or 0.0) > maxLoadPulse then
            maxLoadPulse = state.loadPulse[i] or 0.0
        end

        if (state.verticalKick[i] or 0.0) > maxVerticalKick then
            maxVerticalKick = state.verticalKick[i] or 0.0
        end
    end

    state.avgHop = sumHop * 0.25
    state.avgEnergy = sumEnergy * 0.25
    state.avgInstability = sumInstability * 0.25
    state.avgLoadPulse = sumLoadPulse * 0.25
    state.maxHop = maxHop
    state.maxLoadPulse = maxLoadPulse
    state.maxVerticalKick = maxVerticalKick
    state.activeCount = activeCount

    exportGlobal()
end

function M.getHop(index)
    return state.hop[index] or 0.0
end

function M.getEnergy(index)
    return state.energy[index] or 0.0
end

function M.getLoadPulse(index)
    return state.loadPulse[index] or 0.0
end

function M.getVerticalKick(index)
    return state.verticalKick[index] or 0.0
end

function M.getInstability(index)
    return state.instability[index] or 0.0
end

function M.getState(index)
    if index == nil then
        return state
    end
    return {
        hop = state.hop[index] or 0.0,
        energy = state.energy[index] or 0.0,
        loadPulse = state.loadPulse[index] or 0.0,
        verticalKick = state.verticalKick[index] or 0.0,
        instability = state.instability[index] or 0.0,
        stabilityIndex = state.stabilityIndex[index] or 0.0,
        stickSlip = state.stickSlip[index] or 0.0,
        mode = state.mode[index] or "UNKNOWN",
        load = state.load[index] or 0.0,
        contact = state.contact[index] or 1.0,
    }
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0

        return string.format(
            "%s Hop %.3f E %.3f LP %.0f SI %.2f Inst %.2f SS %.2f Tq %.0f %s",
            tostring(WHEEL_NAMES[i] or i),
            state.hop[i] or 0.0,
            state.energy[i] or 0.0,
            state.loadPulse[i] or 0.0,
            state.stabilityIndex[i] or 0.0,
            state.instability[i] or 0.0,
            state.stickSlip[i] or 0.0,
            state.torqueInput[i] or 0.0,
            tostring(state.mode[i] or "UNKNOWN")
        )
    end

    return string.format(
        "Status %s / Count %.0f / AvgHop %.3f AvgE %.3f AvgInst %.3f\n" ..
        "Pulse Avg %.0f Max %.0f Kick %.2f Active %.0f\n" ..
        "Links CT:%s MEM:%s TD:%s ARM:%s DT:%s LSD:%s CAR:%s ROAD:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.avgHop or 0.0,
        state.avgEnergy or 0.0,
        state.avgInstability or 0.0,
        state.avgLoadPulse or 0.0,
        state.maxLoadPulse or 0.0,
        state.maxVerticalKick or 0.0,
        state.activeCount or 0,
        state.contactLinked and "OK" or "NIL",
        state.memoryLinked and "OK" or "NIL",
        state.dynamicsLinked and "OK" or "NIL",
        state.armLinked and "OK" or "NIL",
        state.drivetrainLinked and "OK" or "NIL",
        state.lsdLinked and "OK" or "NIL",
        state.carcassLinked and "OK" or "NIL",
        state.roadLinked and "OK" or "NIL"
    )
end

function M.drawDebug()
    if not ui then
        return
    end

    ui.text("=== TIRE HOP / WHEEL HOP ===")
    ui.text(M.debugStr())

    for i = 0, 3 do
        ui.text(M.debugStr(i))
    end
end

return M
