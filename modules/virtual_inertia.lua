---@diagnostic disable: undefined-global

--============================================================
-- virtual_inertia.lua
-- ACNextGen V1.1.5 Stable
-- Virtual Rotational Inertia / Root Rotation Response Core
--============================================================

local M = {}

--============================================================
-- Parameters
--============================================================

M.params = {
    yawTau   = 0.120,
    pitchTau = 0.100,
    rollTau  = 0.090,
    rearTau  = 0.180,

    chassisYawGain   = 0.45,
    chassisPitchGain = 0.38,
    chassisRollGain  = 0.42,

    chassisEnergyYawGain   = 0.28,
    chassisEnergyPitchGain = 0.18,
    chassisEnergyRollGain  = 0.22,
    chassisReleaseGain = 0.35,

    loadYawGain   = 0.35,
    loadPitchGain = 0.28,
    loadRollGain  = 0.32,

    pitchCGYawGain = 0.90,
    rollCGYawGain  = 0.70,
    pitchCGPitchGain = 0.75,
    rollCGRollGain   = 0.75,
    maxInertiaShift = 0.35,

    yawGain   = 1.00,
    pitchGain = 1.00,
    rollGain  = 1.00,
    rearGain = 0.75,
    frontYawDamp = 0.82,
    bodyYawGain  = 0.40,

    maxYaw   = 2.0,
    maxPitch = 2.0,
    maxRoll  = 2.0,
    maxVirtualYaw   = 2.5,
    maxVirtualPitch = 2.5,
    maxVirtualRoll  = 2.5,

    bodyRigidityInfluence = 0.16,
    bodyFlexYawDelay = 0.12,
    bodyFlexRollDelay = 0.10,
    bodyFlexPitchDelay = 0.08,
    bodyTorsionRearYawGain = 0.10,
    bodyTorsionYawLoss = 0.07,
    bodyBendingPitchLoss = 0.07,
    bodyStiffYawGain = 0.05,
    bodyStiffRollGain = 0.04,
    bodyStiffPitchGain = 0.04,
    bodyFlexInstabilityGain = 0.10,
    bodyDampingInstabilityLoss = 0.08,
    minBodyInertiaScale = 0.86,
    maxBodyInertiaScale = 1.12,

    massScaleYawGain = 0.45,
    massScalePitchGain = 0.35,
    massScaleRollGain = 0.35,
    yawInertiaScaleGain = 0.70,
    cgRearShiftYawGain = 0.22,
    cgRearShiftPitchGain = 0.18,

    tireHopInstabilityGain = 0.10,
    contactLossInstabilityGain = 0.08,
    damageInstabilityGain = 0.12,
    drivetrainYawLagGain = 0.06,
    lsdYawStabilizeGain = 0.05,

    driveTorqueReference = 2500.0,
    shaftTwistYawLagGain = 0.04,
    brakePitchInertiaGain = 0.04,
    brakeLockInstabilityGain = 0.06,
    thermalInstabilityGain = 0.04,
    roadInputInstabilityGain = 0.06,
    loadPathYawGain = 0.08,
    maxRootExtraInstability = 0.20,

    defaultWheelLoad = 3500.0,
    loadAxisRef = 14000.0,

    storeOnlyDecayTau = 0.420,
    minDt = 0.00005,
    maxDt = 0.050,
    debugStoreInterval = 0.25,
}

--============================================================
-- State
--============================================================

local state = {
    yaw = 0.0,
    pitch = 0.0,
    roll = 0.0,
    rearYaw = 0.0,
    bodyYaw = 0.0,

    loadYaw = 0.0,
    loadPitch = 0.0,
    loadRoll = 0.0,
    loadFront = 0.5,
    loadRear = 0.5,
    loadLeft = 0.5,
    loadRight = 0.5,

    chassisYaw = 0.0,
    chassisPitch = 0.0,
    chassisRoll = 0.0,
    chassisHeave = 0.0,
    chassisEnergy = 0.0,
    chassisRelease = 0.0,

    virtualYaw = 0.0,
    virtualPitch = 0.0,
    virtualRoll = 0.0,
    virtualRearYaw = 0.0,

    yawInertiaScale = 1.0,
    pitchInertiaScale = 1.0,
    rollInertiaScale = 1.0,

    pitchCG = 0.0,
    rollCG = 0.0,
    massScale = 1.0,
    massYawInertiaScale = 1.0,
    cgRearShift = 0.0,

    tireHopAvg = 0.0,
    contactLossAvg = 0.0,
    roadInputAvg = 0.0,
    vehicleDamage = 0.0,

    driveTorque = 0.0,
    driveTorqueNorm = 0.0,
    shaftTwist = 0.0,
    driveLash = 0.0,
    lsdLock = 0.0,

    brakeInput = 0.0,
    brakeLockAvg = 0.0,
    thermalStress = 0.0,
    rootExtraInstability = 0.0,
    instability = 0.0,

    yawState = "NEUTRAL",
    pitchState = "NEUTRAL",
    rollState = "NEUTRAL",
    transition = "STABLE",

    status = "INIT",
    updateCount = 0,
    carAvailable = false,
    storeOnly = false,

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyBendingFactor = 1.0,
    bodyDampingFactor = 1.0,

    bodyYawScale = 1.0,
    bodyPitchScale = 1.0,
    bodyRollScale = 1.0,
    bodyRearYawScale = 1.0,

    bodyRigidityLinked = false,
    massBalanceLinked = false,
    loadLinked = false,
    chassisLinked = false,
    tireLinked = false,
    drivetrainLinked = false,
    damageLinked = false,
    brakeLinked = false,
    thermalLinked = false,
    roadInputLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

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
    if not ok then
        return nil
    end
    return car
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target  = safeNumber(target, 0.0)
    tau     = safeNumber(tau, 0.001)
    dt      = safeNumber(dt, 0.0)
    if tau <= 0.0001 then
        return target
    end
    local k = clamp(dt / (tau + dt), 0.0, 1.0)
    return current + (target - current) * k
end

local function loadFirst(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local v = safeLoadRaw(keys[i])
        if v ~= nil then
            return safeNumber(v, defaultValue or 0.0), true
        end
    end
    if defaultValue == nil then
        return nil, false
    end
    return defaultValue, false
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

--============================================================
-- Input readers
--============================================================

local function resetLinks()
    state.bodyRigidityLinked = false
    state.massBalanceLinked = false
    state.loadLinked = false
    state.chassisLinked = false
    state.tireLinked = false
    state.drivetrainLinked = false
    state.damageLinked = false
    state.brakeLinked = false
    state.thermalLinked = false
    state.roadInputLinked = false
end

local function readBodyRigidity()
    local count = safeLoadRaw("ngp_body_rigidity_update_count")
        or safeLoadRaw("ngp_body_update_count")

    state.bodyRigidity = clamp(loadFirst(1.0,
        "ngp_body_rigidity",
        "ngp_rigidity_body",
        "ngp_chassis_rigidity"), 0.20, 1.35)

    state.bodyFlexFactor = clamp(loadFirst(1.0,
        "ngp_body_flex_factor",
        "ngp_flex_factor",
        "ngp_chassis_flex_factor"), 0.25, 1.80)

    state.bodyTorsionFactor = clamp(loadFirst(1.0,
        "ngp_body_torsion_factor",
        "ngp_chassis_torsion_factor"), 0.35, 2.00)

    state.bodyBendingFactor = clamp(loadFirst(1.0,
        "ngp_body_bending_factor",
        "ngp_chassis_bending_factor"), 0.35, 2.00)

    state.bodyDampingFactor = clamp(loadFirst(1.0,
        "ngp_body_damping_factor",
        "ngp_chassis_damping_factor"), 0.50, 1.80)

    if count ~= nil
    or safeLoadRaw("ngp_body_rigidity") ~= nil
    or safeLoadRaw("ngp_body_flex_factor") ~= nil then
        state.bodyRigidityLinked = true
    end
end

local function readCarAngularInput(car)
    local rawYaw, rawPitch, rawRoll = 0.0, 0.0, 0.0

    local av = safeField(car, "localAngularVelocity", nil)
    if av then
        rawPitch = safeNumber(safeField(av, "x", 0.0), 0.0)
        rawYaw   = safeNumber(safeField(av, "y", 0.0), 0.0)
        rawRoll  = safeNumber(safeField(av, "z", 0.0), 0.0)
        return rawYaw, rawPitch, rawRoll
    end

    av = safeField(car, "angularVelocity", nil)
    if av then
        rawPitch = safeNumber(safeField(av, "x", 0.0), 0.0)
        rawYaw   = safeNumber(safeField(av, "y", 0.0), 0.0)
        rawRoll  = safeNumber(safeField(av, "z", 0.0), 0.0)
        return rawYaw, rawPitch, rawRoll
    end

    rawYaw = loadFirst(0.0,
        "ngp_car_yaw_rate",
        "ngp_yaw_rate",
        "ngp_virtual_raw_yaw_input")

    rawPitch = loadFirst(0.0,
        "ngp_car_pitch_rate",
        "ngp_pitch_rate",
        "ngp_virtual_raw_pitch_input")

    rawRoll = loadFirst(0.0,
        "ngp_car_roll_rate",
        "ngp_roll_rate",
        "ngp_virtual_raw_roll_input")

    return rawYaw, rawPitch, rawRoll
end

local function readChassisInput()
    local rollRaw = safeLoadRaw("ngp_chassis_roll")
        or safeLoadRaw("ngp_body_roll")
        or safeLoadRaw("ngp_chassis_roll_energy")

    local pitchRaw = safeLoadRaw("ngp_chassis_pitch")
        or safeLoadRaw("ngp_body_pitch")
        or safeLoadRaw("ngp_chassis_pitch_energy")

    local heaveRaw = safeLoadRaw("ngp_chassis_heave")
        or safeLoadRaw("ngp_body_heave")
        or safeLoadRaw("ngp_rbi_heave")

    local yawRaw = safeLoadRaw("ngp_chassis_yaw_hint")
        or safeLoadRaw("ngp_body_yaw_hint")
        or safeLoadRaw("ngp_yaw_deflection")

    state.chassisRoll = clamp(safeNumber(rollRaw, 0.0), -2.0, 2.0)
    state.chassisPitch = clamp(safeNumber(pitchRaw, 0.0), -2.0, 2.0)
    state.chassisHeave = clamp(safeNumber(heaveRaw, 0.0), -2.0, 2.0)
    state.chassisYaw = clamp(safeNumber(yawRaw, 0.0), -2.0, 2.0)

    state.chassisEnergy = clamp(loadFirst(0.0,
        "ngp_chassis_body_energy",
        "ngp_body_chassis_energy",
        "ngp_chassis_energy",
        "ngp_chassis_target_energy"), 0.0, 1.5)

    state.chassisRelease = clamp(loadFirst(0.0,
        "ngp_chassis_body_release",
        "ngp_body_chassis_release",
        "ngp_chassis_release"), 0.0, 1.5)

    state.chassisLinked = rollRaw ~= nil or pitchRaw ~= nil or heaveRaw ~= nil or yawRaw ~= nil
end

local function readWheelLoad(index)
    local wheelLoad, linked = loadFirst(nil,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index,
        "ngp_tire_load_" .. index,
        "ngp_tire_state_load_" .. index,
        "ngp_contact_load_" .. index,
        "ngp_cp_load_" .. index)

    if linked then
        return wheelLoad, true
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        return M.params.defaultWheelLoad + safeNumber(dlt, 0.0), true
    end

    return M.params.defaultWheelLoad, false
end

local function normalizeLoadAxis(front, rear, left, right)
    local maxVal = math.max(abs(front), abs(rear), abs(left), abs(right))
    if maxVal > 2.0 then
        local longTotal = math.max(front + rear, 1.0)
        local latTotal = math.max(left + right, 1.0)
        return front / longTotal, rear / longTotal, left / latTotal, right / latTotal
    end
    return front, rear, left, right
end

local function readLoadInput()
    local frontRaw, frontLinked = loadFirst(nil, "ngp_load_abs_front", "ngp_load_front", "ngp_front_load")
    local rearRaw, rearLinked = loadFirst(nil, "ngp_load_abs_rear", "ngp_load_rear", "ngp_rear_load")
    local leftRaw, leftLinked = loadFirst(nil, "ngp_load_abs_left", "ngp_load_left", "ngp_left_load")
    local rightRaw, rightLinked = loadFirst(nil, "ngp_load_abs_right", "ngp_load_right", "ngp_right_load")

    local wheelLinked = false
    if frontRaw == nil or rearRaw == nil or leftRaw == nil or rightRaw == nil then
        local fl, l0 = readWheelLoad(0)
        local fr, l1 = readWheelLoad(1)
        local rl, l2 = readWheelLoad(2)
        local rr, l3 = readWheelLoad(3)
        wheelLinked = l0 or l1 or l2 or l3

        frontRaw = frontRaw or (fl + fr)
        rearRaw = rearRaw or (rl + rr)
        leftRaw = leftRaw or (fl + rl)
        rightRaw = rightRaw or (fr + rr)
    end

    local front, rear, left, right = normalizeLoadAxis(
        safeNumber(frontRaw, 0.5),
        safeNumber(rearRaw, 0.5),
        safeNumber(leftRaw, 0.5),
        safeNumber(rightRaw, 0.5)
    )

    state.loadFront = clamp(front, 0.0, 1.5)
    state.loadRear = clamp(rear, 0.0, 1.5)
    state.loadLeft = clamp(left, 0.0, 1.5)
    state.loadRight = clamp(right, 0.0, 1.5)

    state.loadPitch = clamp(state.loadFront - state.loadRear, -1.0, 1.0)
    state.loadRoll = clamp(state.loadRight - state.loadLeft, -1.0, 1.0)
    state.loadYaw = clamp(state.loadRoll * 0.65 + state.loadPitch * state.chassisYaw * 0.35, -1.0, 1.0)

    state.loadLinked = frontLinked or rearLinked or leftLinked or rightLinked or wheelLinked
end

local function readCGInput()
    state.pitchCG = clamp(loadFirst(0.0,
        "ngp_pitch_cg",
        "ngp_sprung_pitch_cg",
        "ngp_mass_pitch_cg"), -1.0, 1.0)

    state.rollCG = clamp(loadFirst(0.0,
        "ngp_roll_cg",
        "ngp_sprung_roll_cg",
        "ngp_mass_roll_cg"), -1.0, 1.0)
end

local function readMassBalanceInput()
    local massScaleRaw = safeLoadRaw("ngp_vehicle_mass_scale")
        or safeLoadRaw("ngp_mass_balance_scale")
        or safeLoadRaw("ngp_mass_scale")

    local yawScaleRaw = safeLoadRaw("ngp_vehicle_yaw_inertia_scale")
        or safeLoadRaw("ngp_mass_yaw_inertia_scale")
        or safeLoadRaw("ngp_yaw_inertia_scale")

    local cgRearRaw = safeLoadRaw("ngp_vehicle_cg_rear_shift")
        or safeLoadRaw("ngp_mass_cg_rear_shift")
        or safeLoadRaw("ngp_rear_cg_shift")

    state.massScale = clamp(safeNumber(massScaleRaw, 1.0), 0.85, 1.20)
    state.massYawInertiaScale = clamp(safeNumber(yawScaleRaw, 1.0), 0.80, 1.35)
    state.cgRearShift = clamp(safeNumber(cgRearRaw, 0.0), -0.20, 0.20)

    state.massBalanceLinked = massScaleRaw ~= nil or yawScaleRaw ~= nil or cgRearRaw ~= nil
end

local function readTireDamageDrivetrainInput()
    local hopSum = 0.0
    local contactLossSum = 0.0
    local roadSum = 0.0
    local tireLinked = false
    local roadLinked = false

    for i = 0, 3 do
        local hop = safeLoadRaw("ngp_tire_hop_energy_" .. i)
            or safeLoadRaw("ngp_tire_hop_" .. i)
            or safeLoadRaw("ngp_thop_energy_" .. i)

        local loss = safeLoadRaw("ngp_tire_contact_loss_" .. i)
            or safeLoadRaw("ngp_tcr_contact_loss_" .. i)
            or safeLoadRaw("ngp_contact_loss_" .. i)
            or safeLoadRaw("ngp_cp_contact_loss_" .. i)

        local road = safeLoadRaw("ngp_tire_force_road_input_" .. i)
            or safeLoadRaw("ngp_tf_road_input_" .. i)
            or safeLoadRaw("ngp_road_input_" .. i)
            or safeLoadRaw("ngp_load_path_work_norm_" .. i)

        if hop ~= nil or loss ~= nil then
            tireLinked = true
        end
        if road ~= nil then
            roadLinked = true
        end

        hopSum = hopSum + clamp(safeNumber(hop, 0.0), 0.0, 1.0)
        contactLossSum = contactLossSum + clamp(safeNumber(loss, 0.0), 0.0, 1.0)
        roadSum = roadSum + clamp(safeNumber(road, 0.0), 0.0, 2.0)
    end

    state.tireHopAvg = hopSum * 0.25
    state.contactLossAvg = contactLossSum * 0.25
    state.roadInputAvg = roadSum * 0.25
    state.tireLinked = tireLinked
    state.roadInputLinked = roadLinked

    local damageRaw = safeLoadRaw("ngp_condition_total")
        or safeLoadRaw("ngp_vehicle_condition_total")
        or safeLoadRaw("ngp_damage_chassis")
        or safeLoadRaw("ngp_damage_total")

    state.vehicleDamage = clamp(safeNumber(damageRaw, 0.0), 0.0, 1.0)
    state.damageLinked = damageRaw ~= nil

    local torqueRaw = safeLoadRaw("ngp_drive_torque_normalized_from_nm")
        or safeLoadRaw("ngp_drive_torque")
        or safeLoadRaw("ngp_drivetrain_torque")
        or safeLoadRaw("ngp_dt_torque")

    local shaftRaw = safeLoadRaw("ngp_shaft_twist")
        or safeLoadRaw("ngp_windup_twist")
        or safeLoadRaw("ngp_driveline_twist")

    local lashRaw = safeLoadRaw("ngp_drive_lash")
        or safeLoadRaw("ngp_driveline_lash")

    local lsdRaw = safeLoadRaw("ngp_lsd_lock")
        or safeLoadRaw("ngp_diff_lock")

    state.driveTorque = safeNumber(torqueRaw, 0.0)
    state.driveTorqueNorm = clamp(abs(state.driveTorque) / math.max(M.params.driveTorqueReference, 1.0), 0.0, 1.0)
    if abs(state.driveTorque) <= 1.5 and torqueRaw ~= nil then
        state.driveTorqueNorm = clamp(abs(state.driveTorque), 0.0, 1.0)
    end

    state.shaftTwist = safeNumber(shaftRaw, 0.0)
    state.driveLash = safeNumber(lashRaw, 0.0)
    state.lsdLock = clamp(safeNumber(lsdRaw, 0.0), 0.0, 1.0)
    state.drivetrainLinked = torqueRaw ~= nil or shaftRaw ~= nil or lashRaw ~= nil or lsdRaw ~= nil

    local brakeRaw = safeLoadRaw("ngp_brake_input")
        or safeLoadRaw("ngp_brake")
    state.brakeInput = clamp(safeNumber(brakeRaw, 0.0), 0.0, 1.0)

    local lockSum = 0.0
    local brakeLinked = brakeRaw ~= nil
    for i = 0, 3 do
        local lockRaw = safeLoadRaw("ngp_brake_lock_" .. i)
            or safeLoadRaw("ngp_brake_lock_state_" .. i)
        if lockRaw ~= nil then
            brakeLinked = true
        end
        lockSum = lockSum + clamp(safeNumber(lockRaw, 0.0), 0.0, 1.0)
    end
    state.brakeLockAvg = lockSum * 0.25
    state.brakeLinked = brakeLinked

    local tempMax = safeLoadRaw("ngp_brake_temp_max")
        or safeLoadRaw("ngp_brake_temp_avg")
        or safeLoadRaw("ngp_thermal_stress")
        or safeLoadRaw("ngp_gearbox_temp")
        or safeLoadRaw("ngp_gearbox_heat")

    if tempMax ~= nil then
        local temp = safeNumber(tempMax, 25.0)
        if temp <= 1.5 then
            state.thermalStress = clamp(temp, 0.0, 1.0)
        else
            state.thermalStress = clamp((temp - 25.0) / 700.0, 0.0, 1.0)
        end
    else
        state.thermalStress = 0.0
    end

    state.thermalLinked = tempMax ~= nil

    state.rootExtraInstability = clamp(
        state.brakeLockAvg * M.params.brakeLockInstabilityGain
        + state.thermalStress * M.params.thermalInstabilityGain
        + state.roadInputAvg * M.params.roadInputInstabilityGain,
        0.0,
        M.params.maxRootExtraInstability
    )
end

--============================================================
-- Core calculation
--============================================================

local function updateInertiaScale()
    local p = M.params

    local yawScale =
        1.0
        + abs(state.pitchCG) * p.pitchCGYawGain
        + abs(state.rollCG) * p.rollCGYawGain
        + state.chassisEnergy * 0.12
        + (state.massScale - 1.0) * p.massScaleYawGain
        + (state.massYawInertiaScale - 1.0) * p.yawInertiaScaleGain
        + abs(state.cgRearShift) * p.cgRearShiftYawGain
        + state.driveTorqueNorm * p.drivetrainYawLagGain
        + abs(state.shaftTwist) * p.shaftTwistYawLagGain
        + abs(state.driveLash) * 0.03

    yawScale = yawScale - state.lsdLock * p.lsdYawStabilizeGain

    local pitchScale =
        1.0
        + abs(state.pitchCG) * p.pitchCGPitchGain
        + state.chassisEnergy * 0.08
        + (state.massScale - 1.0) * p.massScalePitchGain
        + abs(state.cgRearShift) * p.cgRearShiftPitchGain
        + state.brakeInput * p.brakePitchInertiaGain

    local rollScale =
        1.0
        + abs(state.rollCG) * p.rollCGRollGain
        + state.chassisEnergy * 0.10
        + (state.massScale - 1.0) * p.massScaleRollGain

    state.yawInertiaScale = clamp(yawScale, 1.0 - p.maxInertiaShift, 1.0 + p.maxInertiaShift)
    state.pitchInertiaScale = clamp(pitchScale, 1.0 - p.maxInertiaShift, 1.0 + p.maxInertiaShift)
    state.rollInertiaScale = clamp(rollScale, 1.0 - p.maxInertiaShift, 1.0 + p.maxInertiaShift)
end

local function updateVirtualInertia(rawYaw, rawPitch, rawRoll, dt)
    local p = M.params

    state.yaw = lowPass(state.yaw, rawYaw, p.yawTau * state.yawInertiaScale, dt)
    state.pitch = lowPass(state.pitch, rawPitch, p.pitchTau * state.pitchInertiaScale, dt)
    state.roll = lowPass(state.roll, rawRoll, p.rollTau * state.rollInertiaScale, dt)

    state.yaw = state.yaw
        + state.chassisYaw * p.chassisYawGain
        + state.chassisEnergy * p.chassisEnergyYawGain
        - state.chassisRelease * p.chassisReleaseGain * 0.35
        + state.roadInputAvg * p.loadPathYawGain

    state.pitch = state.pitch
        + state.chassisPitch * p.chassisPitchGain
        + state.chassisEnergy * p.chassisEnergyPitchGain
        - state.chassisRelease * p.chassisReleaseGain * 0.20

    state.roll = state.roll
        + state.chassisRoll * p.chassisRollGain
        + state.chassisEnergy * p.chassisEnergyRollGain
        - state.chassisRelease * p.chassisReleaseGain * 0.22

    state.yaw = state.yaw + state.loadYaw * p.loadYawGain
    state.pitch = state.pitch + state.loadPitch * p.loadPitchGain
    state.roll = state.roll + state.loadRoll * p.loadRollGain

    state.yaw = clamp(state.yaw, -p.maxYaw, p.maxYaw)
    state.pitch = clamp(state.pitch, -p.maxPitch, p.maxPitch)
    state.roll = clamp(state.roll, -p.maxRoll, p.maxRoll)

    state.rearYaw = lowPass(state.rearYaw, state.yaw, p.rearTau, dt)
    state.bodyYaw = state.yaw * p.frontYawDamp + state.rearYaw * p.bodyYawGain + state.loadYaw
end

local function applyBodyRigidityToVirtualOutputs(dt)
    local p = M.params

    if not state.bodyRigidityLinked then
        state.bodyYawScale = lowPass(state.bodyYawScale, 1.0, 0.060, dt)
        state.bodyPitchScale = lowPass(state.bodyPitchScale, 1.0, 0.060, dt)
        state.bodyRollScale = lowPass(state.bodyRollScale, 1.0, 0.060, dt)
        state.bodyRearYawScale = lowPass(state.bodyRearYawScale, 1.0, 0.070, dt)
        return
    end

    local rigidity = clamp(state.bodyRigidity, 0.20, 1.35)
    local flex = clamp(state.bodyFlexFactor, 0.25, 1.80)
    local torsion = clamp(state.bodyTorsionFactor, 0.35, 2.00)
    local bending = clamp(state.bodyBendingFactor, 0.35, 2.00)
    local damping = clamp(state.bodyDampingFactor, 0.50, 1.80)

    local softness = clamp(flex - 1.0, 0.0, 0.80)
    local stiffness = clamp(rigidity - 1.0, 0.0, 0.35)
    local torsionSoft = clamp(torsion - 1.0, 0.0, 1.0)
    local bendingSoft = clamp(bending - 1.0, 0.0, 1.0)

    local yawScaleTarget = 1.0 - softness * p.bodyFlexYawDelay - torsionSoft * p.bodyTorsionYawLoss + stiffness * p.bodyStiffYawGain
    local pitchScaleTarget = 1.0 - softness * p.bodyFlexPitchDelay - bendingSoft * p.bodyBendingPitchLoss + stiffness * p.bodyStiffPitchGain
    local rollScaleTarget = 1.0 - softness * p.bodyFlexRollDelay - torsionSoft * 0.05 + stiffness * p.bodyStiffRollGain
    local rearYawScaleTarget = 1.0 + torsionSoft * p.bodyTorsionRearYawGain + softness * 0.04 - stiffness * 0.03

    state.bodyYawScale = lowPass(state.bodyYawScale, clamp(yawScaleTarget, p.minBodyInertiaScale, p.maxBodyInertiaScale), 0.060, dt)
    state.bodyPitchScale = lowPass(state.bodyPitchScale, clamp(pitchScaleTarget, p.minBodyInertiaScale, p.maxBodyInertiaScale), 0.060, dt)
    state.bodyRollScale = lowPass(state.bodyRollScale, clamp(rollScaleTarget, p.minBodyInertiaScale, p.maxBodyInertiaScale), 0.060, dt)
    state.bodyRearYawScale = lowPass(state.bodyRearYawScale, clamp(rearYawScaleTarget, p.minBodyInertiaScale, p.maxBodyInertiaScale), 0.070, dt)

    local instabilityAdd = softness * p.bodyFlexInstabilityGain + torsionSoft * 0.06 - (damping - 1.0) * p.bodyDampingInstabilityLoss
    state.instability = clamp(state.instability + instabilityAdd, 0.0, 1.5)
end

local function updatePerception()
    if abs(state.yaw) < 0.05 then
        state.yawState = "NEUTRAL"
    elseif state.yaw > 0.0 then
        state.yawState = "TURN RIGHT"
    else
        state.yawState = "TURN LEFT"
    end

    if abs(state.pitch) < 0.05 then
        state.pitchState = "NEUTRAL"
    elseif state.pitch > 0.0 then
        state.pitchState = "NOSE DOWN"
    else
        state.pitchState = "NOSE UP"
    end

    if abs(state.roll) < 0.05 then
        state.rollState = "NEUTRAL"
    elseif state.roll > 0.0 then
        state.rollState = "ROLL RIGHT"
    else
        state.rollState = "ROLL LEFT"
    end

    local rearDiff = abs(state.yaw - state.rearYaw)

    state.instability = clamp(
        rearDiff * 1.50
        + abs(state.loadYaw) * 0.80
        + abs(state.chassisYaw) * 0.65
        + state.chassisEnergy * 0.40
        + state.tireHopAvg * M.params.tireHopInstabilityGain
        + state.contactLossAvg * M.params.contactLossInstabilityGain
        + state.vehicleDamage * M.params.damageInstabilityGain
        + state.rootExtraInstability,
        0.0,
        1.5
    )

    if state.instability < 0.10 then
        state.transition = "STABLE"
    elseif state.instability < 0.25 then
        state.transition = "LOAD SHIFT"
    elseif state.instability < 0.45 then
        state.transition = "LIMIT APPROACH"
    else
        state.transition = "BREAK TRANSITION"
    end
end

local function decayStoreOnly(dt)
    local tau = M.params.storeOnlyDecayTau
    state.yaw = lowPass(state.yaw, 0.0, tau, dt)
    state.pitch = lowPass(state.pitch, 0.0, tau, dt)
    state.roll = lowPass(state.roll, 0.0, tau, dt)
    state.rearYaw = lowPass(state.rearYaw, state.yaw, tau, dt)
    state.bodyYaw = lowPass(state.bodyYaw, 0.0, tau, dt)
end

--============================================================
-- Export
--============================================================

local function exportState()
    local p = M.params

    state.virtualYaw = clamp(state.bodyYaw * p.yawGain * state.bodyYawScale, -p.maxVirtualYaw, p.maxVirtualYaw)
    state.virtualPitch = clamp(state.pitch * p.pitchGain * state.bodyPitchScale, -p.maxVirtualPitch, p.maxVirtualPitch)
    state.virtualRoll = clamp(state.roll * p.rollGain * state.bodyRollScale, -p.maxVirtualRoll, p.maxVirtualRoll)
    state.virtualRearYaw = clamp(state.rearYaw * p.rearGain * state.bodyRearYawScale, -p.maxVirtualYaw, p.maxVirtualYaw)

    safeStore("ngp_virtual_yaw", state.virtualYaw)
    safeStore("ngp_virtual_pitch", state.virtualPitch)
    safeStore("ngp_virtual_roll", state.virtualRoll)
    safeStore("ngp_virtual_rear_yaw", state.virtualRearYaw)

    safeStore("ngp_virtual_body_yaw", state.bodyYaw)
    safeStore("ngp_virtual_yaw_raw", state.yaw)
    safeStore("ngp_virtual_pitch_raw", state.pitch)
    safeStore("ngp_virtual_roll_raw", state.roll)

    safeStore("ngp_virtual_yaw_inertia", state.yawInertiaScale)
    safeStore("ngp_virtual_pitch_inertia", state.pitchInertiaScale)
    safeStore("ngp_virtual_roll_inertia", state.rollInertiaScale)

    safeStore("ngp_virtual_instability", state.instability)
    safeStore("ngp_virtual_transition", state.transition)
    safeStore("ngp_virtual_yaw_state", state.yawState)
    safeStore("ngp_virtual_pitch_state", state.pitchState)
    safeStore("ngp_virtual_roll_state", state.rollState)

    safeStore("ngp_virtual_inertia_status", state.status)
    safeStore("ngp_virtual_inertia_update_count", state.updateCount)
    safeStore("ngp_virtual_inertia_car_available", state.carAvailable and 1 or 0)
    safeStore("ngp_virtual_inertia_store_only", state.storeOnly and 1 or 0)

    safeStore("ngp_body_yaw", state.bodyYaw)
    safeStore("ngp_body_pitch_inertia", state.virtualPitch)
    safeStore("ngp_body_roll_inertia", state.virtualRoll)
    safeStore("ngp_body_rear_yaw", state.virtualRearYaw)

    safeStore("ngp_vi_yaw", state.virtualYaw)
    safeStore("ngp_vi_pitch", state.virtualPitch)
    safeStore("ngp_vi_roll", state.virtualRoll)
    safeStore("ngp_vi_rear_yaw", state.virtualRearYaw)
    safeStore("ngp_vi_instability", state.instability)
    safeStore("ngp_vi_transition", state.transition)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_virtual_body_rigidity_linked", state.bodyRigidityLinked and 1 or 0)
    safeStore("ngp_virtual_mass_balance_linked", state.massBalanceLinked and 1 or 0)
    safeStore("ngp_virtual_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_virtual_chassis_linked", state.chassisLinked and 1 or 0)
    safeStore("ngp_virtual_tire_linked", state.tireLinked and 1 or 0)
    safeStore("ngp_virtual_drivetrain_linked", state.drivetrainLinked and 1 or 0)
    safeStore("ngp_virtual_damage_linked", state.damageLinked and 1 or 0)
    safeStore("ngp_virtual_brake_linked", state.brakeLinked and 1 or 0)
    safeStore("ngp_virtual_thermal_linked", state.thermalLinked and 1 or 0)
    safeStore("ngp_virtual_road_input_linked", state.roadInputLinked and 1 or 0)

    safeStore("ngp_virtual_mass_scale", state.massScale)
    safeStore("ngp_virtual_mass_yaw_inertia_scale", state.massYawInertiaScale)
    safeStore("ngp_virtual_cg_rear_shift", state.cgRearShift)

    safeStore("ngp_virtual_load_front", state.loadFront)
    safeStore("ngp_virtual_load_rear", state.loadRear)
    safeStore("ngp_virtual_load_left", state.loadLeft)
    safeStore("ngp_virtual_load_right", state.loadRight)
    safeStore("ngp_virtual_load_yaw", state.loadYaw)
    safeStore("ngp_virtual_load_pitch", state.loadPitch)
    safeStore("ngp_virtual_load_roll", state.loadRoll)

    safeStore("ngp_virtual_tire_hop_avg", state.tireHopAvg)
    safeStore("ngp_virtual_contact_loss_avg", state.contactLossAvg)
    safeStore("ngp_virtual_road_input_avg", state.roadInputAvg)
    safeStore("ngp_virtual_damage", state.vehicleDamage)
    safeStore("ngp_virtual_drive_torque_norm", state.driveTorqueNorm)
    safeStore("ngp_virtual_drive_torque", state.driveTorque)
    safeStore("ngp_virtual_shaft_twist", state.shaftTwist)
    safeStore("ngp_virtual_drive_lash", state.driveLash)
    safeStore("ngp_virtual_lsd_lock", state.lsdLock)
    safeStore("ngp_virtual_brake_input", state.brakeInput)
    safeStore("ngp_virtual_brake_lock_avg", state.brakeLockAvg)
    safeStore("ngp_virtual_thermal_stress", state.thermalStress)
    safeStore("ngp_virtual_root_extra_instability", state.rootExtraInstability)

    safeStore("ngp_virtual_body_yaw_scale", state.bodyYawScale)
    safeStore("ngp_virtual_body_pitch_scale", state.bodyPitchScale)
    safeStore("ngp_virtual_body_roll_scale", state.bodyRollScale)
    safeStore("ngp_virtual_body_rear_yaw_scale", state.bodyRearYawScale)
end

--============================================================
-- Update
--============================================================

function M.init()
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = safeNumber(dt, 0.0)
    if dt <= 0.0 then
        state.status = "BAD DT"
        exportState()
        return
    end
    dt = clamp(dt, M.params.minDt, M.params.maxDt)

    updateDebugGate(dt)
    resetLinks()

    if not car then
        car = safeGetCar()
    end

    state.carAvailable = car ~= nil
    state.storeOnly = not state.carAvailable

    if state.carAvailable then
        state.status = "RUNNING"
    else
        state.status = "STORE ONLY"
    end

    readBodyRigidity()
    readChassisInput()
    readLoadInput()
    readCGInput()
    readMassBalanceInput()
    readTireDamageDrivetrainInput()

    local rawYaw, rawPitch, rawRoll = readCarAngularInput(car)

    updateInertiaScale()

    if state.carAvailable then
        updateVirtualInertia(rawYaw, rawPitch, rawRoll, dt)
    else
        decayStoreOnly(dt)
        updateVirtualInertia(rawYaw, rawPitch, rawRoll, dt)
    end

    applyBodyRigidityToVirtualOutputs(dt)
    updatePerception()
    exportState()
end

--============================================================
-- Public API
--============================================================

function M.getYaw()
    return state.virtualYaw or 0.0
end

function M.getPitch()
    return state.virtualPitch or 0.0
end

function M.getRoll()
    return state.virtualRoll or 0.0
end

function M.getRearYaw()
    return state.virtualRearYaw or 0.0
end

function M.getInstability()
    return state.instability or 0.0
end

function M.getInertiaScale(axis)
    if axis == "yaw" then
        return state.yawInertiaScale or 1.0
    end
    if axis == "pitch" then
        return state.pitchInertiaScale or 1.0
    end
    if axis == "roll" then
        return state.rollInertiaScale or 1.0
    end
    return state.yawInertiaScale or 1.0,
           state.pitchInertiaScale or 1.0,
           state.rollInertiaScale or 1.0
end

function M.getState()
    return state
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f / Car %s Store %s\n" ..
        "Virtual Y/P/R %.3f %.3f %.3f RearYaw %.3f\n" ..
        "Raw     Y/P/R %.3f %.3f %.3f BodyYaw %.3f\n" ..
        "Inertia Y/P/R %.2f %.2f %.2f Inst %.2f\n" ..
        "MassScale %.3f YawScale %.3f CGRear %.3f\n" ..
        "DriveNorm %.3f Shaft %.3f Brake %.2f Lock %.2f Therm %.2f Road %.2f\n" ..
        "Transition %s\n" ..
        "Links Body:%s Mass:%s Load:%s Chassis:%s Tire:%s Drive:%s Damage:%s Brake:%s Therm:%s Road:%s",

        tostring(state.status),
        state.updateCount or 0,
        state.carAvailable and "OK" or "NIL",
        state.storeOnly and "YES" or "NO",

        state.virtualYaw or 0.0,
        state.virtualPitch or 0.0,
        state.virtualRoll or 0.0,
        state.virtualRearYaw or 0.0,

        state.yaw or 0.0,
        state.pitch or 0.0,
        state.roll or 0.0,
        state.bodyYaw or 0.0,

        state.yawInertiaScale or 1.0,
        state.pitchInertiaScale or 1.0,
        state.rollInertiaScale or 1.0,
        state.instability or 0.0,

        state.massScale or 1.0,
        state.massYawInertiaScale or 1.0,
        state.cgRearShift or 0.0,

        state.driveTorqueNorm or 0.0,
        state.shaftTwist or 0.0,
        state.brakeInput or 0.0,
        state.brakeLockAvg or 0.0,
        state.thermalStress or 0.0,
        state.roadInputAvg or 0.0,

        tostring(state.transition),

        state.bodyRigidityLinked and "OK" or "NIL",
        state.massBalanceLinked and "OK" or "NIL",
        state.loadLinked and "OK" or "NIL",
        state.chassisLinked and "OK" or "NIL",
        state.tireLinked and "OK" or "NIL",
        state.drivetrainLinked and "OK" or "NIL",
        state.damageLinked and "OK" or "NIL",
        state.brakeLinked and "OK" or "NIL",
        state.thermalLinked and "OK" or "NIL",
        state.roadInputLinked and "OK" or "NIL"
    )
end

function M.drawDebug()
    if not ui then
        return
    end
    ui.text("=== VIRTUAL INERTIA ===")
    ui.text(M.debugStr())
end

return M
