---@diagnostic disable: undefined-global

--============================================================
-- vehicle_condition.lua
-- ACNextGen V1.1.5 Stable
-- Vehicle Condition / Wear Accumulation Model
--
-- Safe observer/bridge layer only.
-- It does not directly rewrite AC physics.
--============================================================

local M = {}

M.params = {
    impactGain = 0.002,
    loadGain = 0.0005,
    heatGain = 0.0008,
    recovery = 0.00001,

    damageStateGain = 0.20,
    damageEventGain = 0.08,

    tireHopGain = 0.0012,
    tireHopLoadGain = 0.00000020,

    contactLossGain = 0.0007,
    tireMemoryGain = 0.0008,
    tireLimitGain = 0.0006,

    drivetrainStressGain = 0.0009,
    shaftTwistGain = 0.0008,
    driveLashGain = 0.00035,
    lsdHeatGain = 0.0006,
    lsdLockStressGain = 0.0005,

    suspensionInputGain = 0.00035,
    damperStressGain = 0.00000018,
    armGeometryGain = 0.00045,
    bodyFlexGain = 0.00028,

    roadInputGain = 0.00035,
    loadPathLossGain = 0.00045,
    brakeFadeGain = 0.00055,
    thermalStressGain = 0.00045,

    gravity = 9.81,

    suspensionLoadReference = 12000.0,
    wheelLoadReference = 3500.0,

    brakeHeatStart = 300.0,
    gearboxHeatStart = 80.0,
    hubHeatStart = 90.0,
    tireHeatStart = 95.0,

    tireLimitReference = 1.0,
    drivetrainStressReference = 1.0,

    maxDamage = 1.0,

    minDt = 0.00005,
    maxDt = 0.050,

    storeOnlyDecayTau = 2.0,
    debugStoreInterval = 0.25,
}

local state = {
    chassis = 0.0,
    suspension = 0.0,
    drivetrain = 0.0,
    brake = 0.0,
    tire = 0.0,
    total = 0.0,

    impact = 0.0,
    impactG = 0.0,
    externalDamage = 0.0,
    damageEvent = 0.0,

    suspensionLoad = 0.0,
    suspensionOverload = 0.0,
    maxWheelLoad = 0.0,

    brakeTemp = 0.0,
    gearboxTemp = 0.0,
    hubTemp = 0.0,
    tireTemp = 0.0,

    tireHopAvg = 0.0,
    tireHopLoad = 0.0,
    contactLossAvg = 0.0,
    tireMemoryAvg = 0.0,
    tireLimitAvg = 0.0,
    tireSlipEnergyAvg = 0.0,

    drivetrainStress = 0.0,
    driveTorque = 0.0,
    shaftTwist = 0.0,
    driveLash = 0.0,
    lsdHeat = 0.0,
    lsdLock = 0.0,

    suspensionInputAvg = 0.0,
    damperStress = 0.0,
    armGeometryStress = 0.0,
    bodyFlexStress = 0.0,
    roadInputAvg = 0.0,
    loadPathLossAvg = 0.0,
    thermalStress = 0.0,

    speedKmh = 0.0,

    wheelsValid = false,

    status = "INIT",
    updateCount = 0,
    resetCount = 0,

    tireLinked = false,
    drivetrainLinked = false,
    suspensionLinked = false,
    armLinked = false,
    bodyLinked = false,
    damageLinked = false,
    thermalLinked = false,
    roadLinked = false,

    activeInputCount = 0,
    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

local function safeNumber(value, defaultValue)
    local n = tonumber(value)
    if n == nil or n ~= n or n == math.huge or n == -math.huge then
        return defaultValue or 0.0
    end
    return n
end

local function clamp(v, minValue, maxValue)
    v = safeNumber(v, minValue)
    if v < minValue then
        return minValue
    end
    if v > maxValue then
        return maxValue
    end
    return v
end

local function abs(v)
    return math.abs(safeNumber(v, 0.0))
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

local function loadFirst(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return safeNumber(value, defaultValue or 0.0), keys[i]
        end
    end
    return defaultValue or 0.0, nil
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.0)
    dt = safeNumber(dt, 0.0)

    if tau <= 0.0 then
        return target
    end

    local k = dt / (tau + dt)
    return current + (target - current) * clamp(k, 0.0, 1.0)
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

local function readWheel(car, index)
    local wheels = safeField(car, "wheels", nil)
    if not wheels then
        return nil
    end

    local wheel = safeField(wheels, index, nil)
    if wheel then
        return wheel
    end

    return safeField(wheels, index + 1, nil)
end

local function hasWheels(car)
    return safeField(car, "wheels", nil) ~= nil
end

local function readSpeedKmh(car)
    local speedKmh = safeField(car, "speedKmh", nil)
    if speedKmh ~= nil then
        return abs(speedKmh)
    end

    local speed = nil
    if speed ~= nil then
        local n = abs(speed)
        if n < 120.0 then
            return n * 3.6
        end
        return n
    end

    local localVelocity = safeField(car, "localVelocity", nil)
    if localVelocity then
        local x = safeNumber(safeField(localVelocity, "x", 0.0), 0.0)
        local y = safeNumber(safeField(localVelocity, "y", 0.0), 0.0)
        local z = safeNumber(safeField(localVelocity, "z", 0.0), 0.0)
        return math.sqrt(x * x + y * y + z * z) * 3.6
    end

    return safeLoad("ngp_tire_state_speed_kmh", 0.0)
end

local function calculateImpact(car)
    local storedImpact, impactKey =
        loadFirst(nil,
            "ngp_impact_strength",
            "ngp_impact_total",
            "ngp_damage_event_intensity",
            "ngp_damage_event_peak"
        )

    local storedPeakG, peakKey =
        loadFirst(nil,
            "ngp_impact_peak_g",
            "ngp_damage_event_peak_g",
            "ngp_damage_event_g",
            "ngp_impact_g"
        )

    if impactKey ~= nil or peakKey ~= nil then
        state.damageLinked = true
    end

    local impact = safeNumber(storedImpact, 0.0)
    local gImpact = math.max(safeNumber(storedPeakG, 0.0) - 1.0, 0.0)

    local acceleration = safeField(car, "acceleration", nil)
    if acceleration then
        local ax = safeNumber(safeField(acceleration, "x", 0.0), 0.0)
        local ay = safeNumber(safeField(acceleration, "y", 0.0), 0.0)
        local az = safeNumber(safeField(acceleration, "z", 0.0), 0.0)
        local g = math.sqrt(ax * ax + ay * ay + az * az) / math.max(M.params.gravity, 0.001)
        gImpact = math.max(gImpact, math.max(g - 1.15, 0.0))
    end

    impact = math.max(impact, gImpact)
    state.impactG = gImpact

    return clamp(impact, 0.0, 20.0)
end

local function interpretDltLoad(value)
    if value == nil then
        return nil
    end
    return M.params.wheelLoadReference + safeNumber(value, 0.0)
end

local function getWheelLoad(car, index)
    local wheel = readWheel(car, index)

    local integrated, integratedKey =
        loadFirst(nil,
            "ngp_wheel_load_" .. index,
            "ngp_load_wheel_" .. index,
            "ngp_tire_load_" .. index,
            "ngp_tire_state_load_" .. index,
            "ngp_tire_force_load_" .. index,
            "ngp_cp_load_" .. index,
            "ngp_tc_load_" .. index
        )

    if integratedKey ~= nil then
        state.suspensionLinked = true
        return safeNumber(integrated, M.params.wheelLoadReference), integratedKey
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        state.suspensionLinked = true
        return interpretDltLoad(dlt), "DLT"
    end

    if wheel then
        local load = safeField(wheel, "load", nil)
        if load == nil then
            load = safeField(wheel, "loadK", nil)
        end
        if load == nil then
            load = nil
        end
        if load == nil then
            load = nil
        end
        if load == nil then
            load = nil
        end

        if load ~= nil then
            return safeNumber(load, M.params.wheelLoadReference), "AC_WHEEL"
        end
    end

    return M.params.wheelLoadReference, "DEFAULT"
end

local function calculateSuspensionLoad(car)
    local totalLoad = 0.0
    local maxLoad = 0.0
    local count = 0

    for i = 0, 3 do
        local load = getWheelLoad(car, i)
        load = abs(load)

        totalLoad = totalLoad + load
        maxLoad = math.max(maxLoad, load)

        if load > 1.0 then
            count = count + 1
        end
    end

    state.maxWheelLoad = maxLoad
    if count > 0 then
        state.suspensionLinked = true
    end

    return totalLoad
end

local function getMaxBrakeTemp()
    local maxTemp = 0.0
    local linked = false

    for i = 0, 3 do
        local temp, key =
            loadFirst(nil,
                "ngp_brake_temp_" .. i,
                "ngp_brake_disc_temp_" .. i,
                "ngp_brake_temp_read_" .. i,
                "ngp_hub_temp_" .. i,
                "ngp_thermal_brake_temp_" .. i
            )

        if key ~= nil then
            linked = true
        end

        maxTemp = math.max(maxTemp, safeNumber(temp, 0.0))
    end

    if linked then
        state.thermalLinked = true
    end

    return maxTemp
end

local function getMaxHubTemp()
    local maxTemp = 0.0
    local linked = false

    for i = 0, 3 do
        local temp, key =
            loadFirst(nil,
                "ngp_hub_temp_" .. i,
                "ngp_rf2_carcass_temp_" .. i,
                "ngp_tire_carcass_temp_" .. i,
                "ngp_tire_temp_" .. i,
                "ngp_tyre_temp_" .. i
            )

        if key ~= nil then
            linked = true
        end

        maxTemp = math.max(maxTemp, safeNumber(temp, 0.0))
    end

    if linked then
        state.thermalLinked = true
    end

    return maxTemp
end

local function getMaxTireTemp()
    local maxTemp = 0.0
    local linked = false

    for i = 0, 3 do
        local temp, key =
            loadFirst(nil,
                "ngp_rf2_tread_temp_" .. i,
                "ngp_tire_temp_" .. i,
                "ngp_tyre_temp_" .. i,
                "ngp_memory_virtual_temp_" .. i,
                "ngp_tire_memory_heat_" .. i
            )

        if key ~= nil then
            linked = true
        end

        maxTemp = math.max(maxTemp, safeNumber(temp, 0.0))
    end

    if linked then
        state.thermalLinked = true
    end

    return maxTemp
end

local function getGearboxTemp()
    local value, key =
        loadFirst(0.0,
            "ngp_gearbox_temp",
            "ngp_thermal_gearbox_temp",
            "ngp_drive_gearbox_temp",
            "ngp_drivetrain_gearbox_temp"
        )

    local heat, heatKey =
        loadFirst(nil,
            "ngp_gearbox_heat",
            "ngp_drive_gearbox_heat",
            "ngp_drivetrain_heat",
            "ngp_transmission_heat"
        )

    if key ~= nil or heatKey ~= nil then
        state.thermalLinked = true
    end

    if value > 0.0 then
        return value
    end

    return safeNumber(heat, 0.0)
end

local function getExternalDamage()
    local damageState, stateKey =
        loadFirst(nil,
            "ngp_damage_chassis",
            "ngp_damage_total",
            "ngp_condition_damage_chassis"
        )

    local damageEvent, eventKey =
        loadFirst(nil,
            "ngp_damage_event_total",
            "ngp_damage_event_intensity",
            "ngp_impact_damage_event"
        )

    if stateKey ~= nil or eventKey ~= nil then
        state.damageLinked = true
    end

    state.externalDamage = safeNumber(damageState, 0.0)
    state.damageEvent = safeNumber(damageEvent, 0.0)

    return state.externalDamage, state.damageEvent
end

local function readTireConditionInputs()
    local sumHop = 0.0
    local sumContactLoss = 0.0
    local sumMemory = 0.0
    local sumLimit = 0.0
    local sumHopLoad = 0.0
    local sumSlipEnergy = 0.0
    local linked = false

    for i = 0, 3 do
        local hop, hopKey =
            loadFirst(nil,
                "ngp_tire_hop_energy_" .. i,
                "ngp_tire_hop_" .. i,
                "ngp_thop_energy_" .. i
            )

        local hopLoad, hopLoadKey =
            loadFirst(nil,
                "ngp_tire_hop_loadpulse_" .. i,
                "ngp_tirehop_" .. i,
                "ngp_susp_tire_hop_load_" .. i,
                "ngp_thop_load_pulse_" .. i
            )

        local contactLoss, contactKey =
            loadFirst(nil,
                "ngp_tire_contact_loss_" .. i,
                "ngp_tcr_contact_loss_" .. i,
                "ngp_contact_loss_" .. i,
                "ngp_contact_hop_drop_" .. i
            )

        local memory, memoryKey =
            loadFirst(nil,
                "ngp_rubber_memory_" .. i,
                "ngp_tire_memory_" .. i,
                "ngp_tyre_memory_" .. i,
                "ngp_memory_" .. i
            )

        local limit, limitKey =
            loadFirst(nil,
                "ngp_tire_limit_" .. i,
                "ngp_tyre_limit_" .. i,
                "ngp_tire_force_limit_" .. i
            )

        local slipEnergy, slipKey =
            loadFirst(nil,
                "ngp_tire_slip_energy_" .. i,
                "ngp_tire_force_slip_energy_" .. i,
                "ngp_tdyn_slip_energy_" .. i,
                "ngp_tire_dynamics_slip_energy_" .. i
            )

        if hopKey or hopLoadKey or contactKey or memoryKey or limitKey or slipKey then
            linked = true
        end

        sumHop = sumHop + clamp(safeNumber(hop, 0.0), 0.0, 1.0)
        sumHopLoad = sumHopLoad + abs(hopLoad)
        sumContactLoss = sumContactLoss + clamp(safeNumber(contactLoss, 0.0), 0.0, 1.0)
        sumMemory = sumMemory + clamp(safeNumber(memory, 0.0), 0.0, 1.0)
        sumLimit = sumLimit + clamp(safeNumber(limit, 0.0), 0.0, 2.0)
        sumSlipEnergy = sumSlipEnergy + clamp(safeNumber(slipEnergy, 0.0), 0.0, 2.5)
    end

    state.tireLinked = linked
    state.tireHopAvg = sumHop * 0.25
    state.tireHopLoad = sumHopLoad
    state.contactLossAvg = sumContactLoss * 0.25
    state.tireMemoryAvg = sumMemory * 0.25
    state.tireLimitAvg = sumLimit * 0.25
    state.tireSlipEnergyAvg = sumSlipEnergy * 0.25
end

local function readDrivetrainConditionInputs()
    local driveTorque, driveKey =
        loadFirst(nil,
            "ngp_drive_torque",
            "ngp_drivetrain_torque",
            "ngp_dt_torque",
            "ngp_drive_torque_normalized_from_nm"
        )

    local shiftStress, shiftKey =
        loadFirst(nil,
            "ngp_shift_stress",
            "ngp_shift_shock",
            "ngp_drive_shift_shock",
            "ngp_drivetrain_shift_stress"
        )

    local shaftTwist, twistKey =
        loadFirst(nil,
            "ngp_shaft_twist",
            "ngp_windup_twist",
            "ngp_driveline_twist"
        )

    local driveLash, lashKey =
        loadFirst(nil,
            "ngp_drive_lash",
            "ngp_driveline_lash",
            "ngp_windup_lash"
        )

    local lsdHeat, heatKey =
        loadFirst(nil,
            "ngp_lsd_heat",
            "ngp_diff_heat",
            "ngp_lsd_thermal",
            "ngp_diff_temp"
        )

    local lsdLock, lockKey =
        loadFirst(nil,
            "ngp_lsd_lock",
            "ngp_diff_lock"
        )

    if driveKey or shiftKey or twistKey or lashKey or heatKey or lockKey then
        state.drivetrainLinked = true
    end

    state.driveTorque = safeNumber(driveTorque, 0.0)
    state.drivetrainStress =
        abs(shiftStress)
        + abs(state.driveTorque) * 0.5
        + abs(driveLash) * 0.35

    state.shaftTwist = safeNumber(shaftTwist, 0.0)
    state.driveLash = safeNumber(driveLash, 0.0)
    state.lsdHeat = safeNumber(lsdHeat, 0.0)
    state.lsdLock = safeNumber(lsdLock, 0.0)
end

local function readSuspensionAndBodyInputs()
    local suspSum = 0.0
    local damperStress = 0.0
    local armStress = 0.0
    local roadSum = 0.0
    local loadPathLossSum = 0.0

    for i = 0, 3 do
        local suspInput, suspKey =
            loadFirst(nil,
                "ngp_susp_contact_input_" .. i,
                "ngp_susp_contact_scale_" .. i,
                "ngp_sci_input_" .. i,
                "ngp_susp_tire_hop_input_" .. i
            )

        local damper, damperKey =
            loadFirst(nil,
                "ngp_damper_" .. i,
                "ngp_susp_" .. i,
                "ngp_damper_force_" .. i,
                "ngp_damper_hyst_vertical_damping_" .. i
            )

        local camber, camberKey =
            loadFirst(nil,
                "ngp_control_arm_camber_" .. i,
                "ngp_arm_camber_" .. i,
                "ngp_uc_camber_" .. i,
                "ngp_ca_camber_" .. i
            )

        local toe, toeKey =
            loadFirst(nil,
                "ngp_control_arm_toe_" .. i,
                "ngp_arm_toe_" .. i,
                "ngp_uc_toe_" .. i,
                "ngp_ca_toe_" .. i
            )

        local roadInput, roadKey =
            loadFirst(nil,
                "ngp_tire_force_road_input_" .. i,
                "ngp_tf_road_input_" .. i,
                "ngp_road_input_severity_" .. i,
                "ngp_rii_severity_" .. i,
                "ngp_rbi_input_" .. i
            )

        local pathLoss, pathKey =
            loadFirst(nil,
                "ngp_load_path_loss_" .. i,
                "ngp_lp_loss_" .. i,
                "ngp_rii_path_loss_" .. i
            )

        if suspKey or damperKey then
            state.suspensionLinked = true
        end

        if camberKey or toeKey then
            state.armLinked = true
        end

        if roadKey or pathKey then
            state.roadLinked = true
        end

        suspSum = suspSum + abs(safeNumber(suspInput, 1.0) - 1.0)
        damperStress = damperStress + abs(damper)
        armStress = armStress + abs(camber) + abs(toe)
        roadSum = roadSum + clamp(abs(roadInput), 0.0, 2.0)
        loadPathLossSum = loadPathLossSum + clamp(abs(pathLoss), 0.0, 2.0)
    end

    state.suspensionInputAvg = suspSum * 0.25
    state.damperStress = damperStress
    state.armGeometryStress = armStress * 0.25
    state.roadInputAvg = roadSum * 0.25
    state.loadPathLossAvg = loadPathLossSum * 0.25

    local flex, flexKey =
        loadFirst(nil,
            "ngp_body_flex_factor",
            "ngp_chassis_flex_yaw",
            "ngp_chassis_flex_roll",
            "ngp_flex_energy"
        )

    local torsion, torsionKey =
        loadFirst(nil,
            "ngp_body_torsion_factor",
            "ngp_chassis_torsion",
            "ngp_body_twist"
        )

    local bodySteer, steerKey =
        loadFirst(nil,
            "ngp_body_steer",
            "ngp_steer_body",
            "ngp_chassis_delay"
        )

    if flexKey or torsionKey or steerKey then
        state.bodyLinked = true
    end

    state.bodyFlexStress =
        abs(safeNumber(flex, 1.0) - 1.0)
        + abs(safeNumber(torsion, 1.0) - 1.0)
        + abs(bodySteer)
end

local function readThermalStress()
    local hubOver = math.max((state.hubTemp or 0.0) - M.params.hubHeatStart, 0.0) / 140.0
    local tireOver = math.max((state.tireTemp or 0.0) - M.params.tireHeatStart, 0.0) / 100.0
    local brakeFade, fadeKey = loadFirst(nil, "ngp_brake_fade_avg", "ngp_brake_root_heat_avg", "ngp_brake_mu_loss")
    local thermalStress, stressKey = loadFirst(nil, "ngp_thermal_stress", "ngp_virtual_thermal_stress")

    if fadeKey or stressKey then
        state.thermalLinked = true
    end

    state.thermalStress =
        clamp(hubOver, 0.0, 2.0) * 0.25
        + clamp(tireOver, 0.0, 2.0) * 0.25
        + clamp(abs(brakeFade), 0.0, 2.0) * 0.25
        + clamp(abs(thermalStress), 0.0, 2.0) * 0.25
end

local function damageStep(current, stress, recoveryScale, dt)
    current = safeNumber(current, 0.0)
    stress = math.max(safeNumber(stress, 0.0), 0.0)
    recoveryScale = safeNumber(recoveryScale, 1.0)

    return clamp(
        current
        + stress * dt
        - M.params.recovery * recoveryScale * dt,
        0.0,
        M.params.maxDamage
    )
end

local function updateChassisCondition(impact, externalDamage, damageEvent, dt)
    local stress =
        impact * M.params.impactGain
        + externalDamage * M.params.damageStateGain
        + damageEvent * M.params.damageEventGain
        + state.bodyFlexStress * M.params.bodyFlexGain
        + state.roadInputAvg * M.params.roadInputGain

    state.chassis = damageStep(state.chassis, stress, 1.0, dt)
end

local function updateSuspensionCondition(suspensionLoad, dt)
    local overload =
        math.max(
            suspensionLoad - M.params.suspensionLoadReference,
            0.0
        )

    state.suspensionOverload = overload

    local rootStress =
        overload * M.params.loadGain
        + state.tireHopLoad * M.params.tireHopLoadGain
        + state.tireHopAvg * M.params.tireHopGain
        + state.suspensionInputAvg * M.params.suspensionInputGain
        + state.damperStress * M.params.damperStressGain
        + state.armGeometryStress * M.params.armGeometryGain
        + state.loadPathLossAvg * M.params.loadPathLossGain

    state.suspension = damageStep(state.suspension, rootStress, 1.0, dt)
end

local function updateDrivetrainCondition(gearboxTemp, dt)
    local heat =
        math.max(
            gearboxTemp - M.params.gearboxHeatStart,
            0.0
        )

    local rootStress =
        heat * M.params.heatGain
        + state.drivetrainStress * M.params.drivetrainStressGain
        + abs(state.shaftTwist) * M.params.shaftTwistGain
        + abs(state.driveLash) * M.params.driveLashGain
        + math.max(state.lsdHeat - 5.0, 0.0) * M.params.lsdHeatGain
        + state.lsdLock * M.params.lsdLockStressGain

    state.drivetrain = damageStep(state.drivetrain, rootStress, 1.0, dt)
end

local function updateBrakeCondition(brakeTemp, dt)
    local heat =
        math.max(
            brakeTemp - M.params.brakeHeatStart,
            0.0
        )

    local stress =
        heat * M.params.heatGain
        + state.thermalStress * M.params.brakeFadeGain

    state.brake = damageStep(state.brake, stress, 1.0, dt)
end

local function updateTireCondition(dt)
    local heatStress =
        math.max((state.tireTemp or 0.0) - M.params.tireHeatStart, 0.0)
        * M.params.thermalStressGain

    local stress =
        state.contactLossAvg * M.params.contactLossGain
        + state.tireMemoryAvg * M.params.tireMemoryGain
        + state.tireLimitAvg * M.params.tireLimitGain
        + state.tireHopAvg * M.params.tireHopGain
        + state.tireSlipEnergyAvg * M.params.tireMemoryGain
        + heatStress

    state.tire = damageStep(state.tire, stress, 1.0, dt)
end

local function updateTotalCondition()
    state.total =
        clamp(
            (
                (state.chassis or 0.0)
                + (state.suspension or 0.0)
                + (state.drivetrain or 0.0)
                + (state.brake or 0.0)
                + (state.tire or 0.0)
            ) * 0.20,
            0.0,
            M.params.maxDamage
        )
end

local function updateDebug(impact, suspensionLoad, brakeTemp, gearboxTemp)
    state.impact = impact
    state.suspensionLoad = suspensionLoad
    state.brakeTemp = brakeTemp
    state.gearboxTemp = gearboxTemp
end

local function updateActiveInputCount()
    local count = 0

    if state.tireLinked then count = count + 1 end
    if state.drivetrainLinked then count = count + 1 end
    if state.suspensionLinked then count = count + 1 end
    if state.armLinked then count = count + 1 end
    if state.bodyLinked then count = count + 1 end
    if state.damageLinked then count = count + 1 end
    if state.thermalLinked then count = count + 1 end
    if state.roadLinked then count = count + 1 end

    state.activeInputCount = count
end

local function exportState()
    safeStore("ngp_vehicle_condition_status", state.status or "UNKNOWN")
    safeStore("ngp_vehicle_condition_update_count", state.updateCount or 0)
    safeStore("ngp_vehicle_condition_reset_count", state.resetCount or 0)
    safeStore("ngp_vehicle_condition_wheels_valid", state.wheelsValid and 1 or 0)

    safeStore("ngp_condition_chassis", state.chassis or 0.0)
    safeStore("ngp_condition_suspension", state.suspension or 0.0)
    safeStore("ngp_condition_drivetrain", state.drivetrain or 0.0)
    safeStore("ngp_condition_brake", state.brake or 0.0)
    safeStore("ngp_condition_tire", state.tire or 0.0)
    safeStore("ngp_condition_total", state.total or 0.0)

    safeStore("ngp_vehicle_chassis_condition", state.chassis or 0.0)
    safeStore("ngp_vehicle_suspension_condition", state.suspension or 0.0)
    safeStore("ngp_vehicle_drivetrain_condition", state.drivetrain or 0.0)
    safeStore("ngp_vehicle_brake_condition", state.brake or 0.0)
    safeStore("ngp_vehicle_tire_condition", state.tire or 0.0)
    safeStore("ngp_vehicle_condition_total", state.total or 0.0)

    safeStore("ngp_condition_impact", state.impact or 0.0)
    safeStore("ngp_condition_external_damage", state.externalDamage or 0.0)
    safeStore("ngp_condition_damage_event", state.damageEvent or 0.0)
    safeStore("ngp_condition_susp_load", state.suspensionLoad or 0.0)
    safeStore("ngp_condition_susp_overload", state.suspensionOverload or 0.0)
    safeStore("ngp_condition_brake_temp", state.brakeTemp or 0.0)
    safeStore("ngp_condition_gearbox_temp", state.gearboxTemp or 0.0)
    safeStore("ngp_condition_speed", state.speedKmh or 0.0)

    safeStore("ngp_vc_total", state.total or 0.0)
    safeStore("ngp_vc_chassis", state.chassis or 0.0)
    safeStore("ngp_vc_suspension", state.suspension or 0.0)
    safeStore("ngp_vc_drivetrain", state.drivetrain or 0.0)
    safeStore("ngp_vc_brake", state.brake or 0.0)
    safeStore("ngp_vc_tire", state.tire or 0.0)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_condition_tire_hop_avg", state.tireHopAvg or 0.0)
    safeStore("ngp_condition_contact_loss_avg", state.contactLossAvg or 0.0)
    safeStore("ngp_condition_tire_memory_avg", state.tireMemoryAvg or 0.0)
    safeStore("ngp_condition_tire_limit_avg", state.tireLimitAvg or 0.0)
    safeStore("ngp_condition_tire_slip_energy_avg", state.tireSlipEnergyAvg or 0.0)

    safeStore("ngp_condition_drive_stress", state.drivetrainStress or 0.0)
    safeStore("ngp_condition_drive_torque", state.driveTorque or 0.0)
    safeStore("ngp_condition_shaft_twist", state.shaftTwist or 0.0)
    safeStore("ngp_condition_drive_lash", state.driveLash or 0.0)
    safeStore("ngp_condition_lsd_heat", state.lsdHeat or 0.0)
    safeStore("ngp_condition_lsd_lock", state.lsdLock or 0.0)

    safeStore("ngp_condition_susp_input_avg", state.suspensionInputAvg or 0.0)
    safeStore("ngp_condition_arm_geometry_stress", state.armGeometryStress or 0.0)
    safeStore("ngp_condition_body_flex_stress", state.bodyFlexStress or 0.0)
    safeStore("ngp_condition_damper_stress", state.damperStress or 0.0)
    safeStore("ngp_condition_road_input_avg", state.roadInputAvg or 0.0)
    safeStore("ngp_condition_load_path_loss_avg", state.loadPathLossAvg or 0.0)
    safeStore("ngp_condition_thermal_stress", state.thermalStress or 0.0)
    safeStore("ngp_condition_hub_temp", state.hubTemp or 0.0)
    safeStore("ngp_condition_tire_temp", state.tireTemp or 0.0)
    safeStore("ngp_condition_max_wheel_load", state.maxWheelLoad or 0.0)
    safeStore("ngp_condition_active_input_count", state.activeInputCount or 0)

    safeStore("ngp_vehicle_condition_tire_linked", state.tireLinked and 1 or 0)
    safeStore("ngp_vehicle_condition_drivetrain_linked", state.drivetrainLinked and 1 or 0)
    safeStore("ngp_vehicle_condition_suspension_linked", state.suspensionLinked and 1 or 0)
    safeStore("ngp_vehicle_condition_arm_linked", state.armLinked and 1 or 0)
    safeStore("ngp_vehicle_condition_body_linked", state.bodyLinked and 1 or 0)
    safeStore("ngp_vehicle_condition_damage_linked", state.damageLinked and 1 or 0)
    safeStore("ngp_vehicle_condition_thermal_linked", state.thermalLinked and 1 or 0)
    safeStore("ngp_vehicle_condition_road_linked", state.roadLinked and 1 or 0)
end

local function clearLinks()
    state.tireLinked = false
    state.drivetrainLinked = false
    state.suspensionLinked = false
    state.armLinked = false
    state.bodyLinked = false
    state.damageLinked = false
    state.thermalLinked = false
    state.roadLinked = false
end

function M.init()
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = clamp(safeNumber(dt, 0.0), M.params.minDt, M.params.maxDt)
    updateDebugGate(dt)

    if not car then
        car = safeGetCar()
    end

    local carAvailable = car ~= nil
    local wheelsAvailable = carAvailable and hasWheels(car)

    if not carAvailable then
        state.status = "STORE ONLY"
        state.wheelsValid = false
    elseif not wheelsAvailable then
        state.status = "STORE ONLY"
        state.wheelsValid = false
    else
        state.status = "RUNNING"
        state.wheelsValid = true
    end

    state.speedKmh = readSpeedKmh(car)
    clearLinks()

    local impact = calculateImpact(car)
    local suspensionLoad = calculateSuspensionLoad(car)
    local brakeTemp = getMaxBrakeTemp()
    local gearboxTemp = getGearboxTemp()

    state.hubTemp = getMaxHubTemp()
    state.tireTemp = getMaxTireTemp()

    local externalDamage, damageEvent = getExternalDamage()

    readTireConditionInputs()
    readDrivetrainConditionInputs()
    readSuspensionAndBodyInputs()
    readThermalStress()
    updateActiveInputCount()

    updateChassisCondition(impact, externalDamage, damageEvent, dt)
    updateSuspensionCondition(suspensionLoad, dt)
    updateDrivetrainCondition(gearboxTemp, dt)
    updateBrakeCondition(brakeTemp, dt)
    updateTireCondition(dt)
    updateTotalCondition()

    updateDebug(impact, suspensionLoad, brakeTemp, gearboxTemp)

    exportState()
end

function M.getState()
    return state
end

function M.getTotal()
    return state.total or 0.0
end

function M.getChassis()
    return state.chassis or 0.0
end

function M.getSuspension()
    return state.suspension or 0.0
end

function M.getDrivetrain()
    return state.drivetrain or 0.0
end

function M.getBrake()
    return state.brake or 0.0
end

function M.getTire()
    return state.tire or 0.0
end

function M.getComponent(name)
    if name == "chassis" then return M.getChassis() end
    if name == "suspension" then return M.getSuspension() end
    if name == "drivetrain" then return M.getDrivetrain() end
    if name == "brake" then return M.getBrake() end
    if name == "tire" or name == "tyre" then return M.getTire() end
    if name == "total" then return M.getTotal() end
    return 0.0
end

function M.reset()
    state.chassis = 0.0
    state.suspension = 0.0
    state.drivetrain = 0.0
    state.brake = 0.0
    state.tire = 0.0
    state.total = 0.0

    state.impact = 0.0
    state.impactG = 0.0
    state.externalDamage = 0.0
    state.damageEvent = 0.0

    state.suspensionLoad = 0.0
    state.suspensionOverload = 0.0
    state.maxWheelLoad = 0.0

    state.brakeTemp = 0.0
    state.gearboxTemp = 0.0
    state.hubTemp = 0.0
    state.tireTemp = 0.0

    state.tireHopAvg = 0.0
    state.tireHopLoad = 0.0
    state.contactLossAvg = 0.0
    state.tireMemoryAvg = 0.0
    state.tireLimitAvg = 0.0
    state.tireSlipEnergyAvg = 0.0

    state.drivetrainStress = 0.0
    state.driveTorque = 0.0
    state.shaftTwist = 0.0
    state.driveLash = 0.0
    state.lsdHeat = 0.0
    state.lsdLock = 0.0

    state.suspensionInputAvg = 0.0
    state.damperStress = 0.0
    state.armGeometryStress = 0.0
    state.bodyFlexStress = 0.0
    state.roadInputAvg = 0.0
    state.loadPathLossAvg = 0.0
    state.thermalStress = 0.0

    state.resetCount = (state.resetCount or 0) + 1
    state.status = "RESET"

    exportState()
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f / Wheels %s / Inputs %.0f\n" ..
        "Condition Total %.3f\n" ..
        "Chassis %.3f Susp %.3f Tire %.3f\n" ..
        "Drive %.3f Brake %.3f\n" ..
        "Impact %.2f G %.2f ExtDmg %.3f Event %.3f\n" ..
        "Load %.0f Over %.0f MaxW %.0f\n" ..
        "BrakeT %.1f GearT %.1f HubT %.1f TireT %.1f\n" ..
        "Links Tire:%s Drive:%s Susp:%s Arm:%s Body:%s Dmg:%s Th:%s Road:%s",

        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",
        state.activeInputCount or 0,

        state.total or 0.0,

        state.chassis or 0.0,
        state.suspension or 0.0,
        state.tire or 0.0,

        state.drivetrain or 0.0,
        state.brake or 0.0,

        state.impact or 0.0,
        state.impactG or 0.0,
        state.externalDamage or 0.0,
        state.damageEvent or 0.0,

        state.suspensionLoad or 0.0,
        state.suspensionOverload or 0.0,
        state.maxWheelLoad or 0.0,

        state.brakeTemp or 0.0,
        state.gearboxTemp or 0.0,
        state.hubTemp or 0.0,
        state.tireTemp or 0.0,

        state.tireLinked and "OK" or "NIL",
        state.drivetrainLinked and "OK" or "NIL",
        state.suspensionLinked and "OK" or "NIL",
        state.armLinked and "OK" or "NIL",
        state.bodyLinked and "OK" or "NIL",
        state.damageLinked and "OK" or "NIL",
        state.thermalLinked and "OK" or "NIL",
        state.roadLinked and "OK" or "NIL"
    )
end

function M.drawDebug()
    if not ui then
        return
    end

    ui.text("=== VEHICLE CONDITION ===")
    ui.text(M.debugStr())
end

return M
