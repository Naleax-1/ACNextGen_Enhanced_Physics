---@diagnostic disable: undefined-global

--============================================================
-- sprung_mass.lua
-- ACNextGen V1.1.5 Stable
-- Sprung Mass / Dynamic CG Approximation
--
-- Policy:
--   Does not rewrite AC physics.
--   Reads root/trunk signals and exports stable pitch/roll CG hints.
--============================================================

local M = {}

local FL, FR, RL, RR = 0, 1, 2, 3
local BASE_MASS = 1310.0

M.params = {
    frontSpringRef = 32000.0,
    rearSpringRef = 36000.0,

    pitchCGScale = 0.10,
    rollCGScale = 0.08,

    pitchTau = 0.100,
    rollTau = 0.080,

    maxPitchCG = 0.080,
    maxRollCG = 0.060,

    chassisEnergyPitchGain = 0.004,
    chassisReleasePitchGain = 0.006,
    chassisEnergyRollGain = 0.003,
    chassisReleaseRollGain = 0.005,

    springForceGain = 0.000006,
    damperForceGain = 0.000004,
    tireHopCGGain = 0.004,
    brakePitchGain = 0.006,
    drivePitchGain = 0.004,

    bodyFlexCGGain = 0.08,
    bodyStiffCGLoss = 0.04,

    suspensionStressCGGain = 0.025,
    brakeLockCGGain = 0.018,
    weightBiasCGGain = 0.020,
    damageRuntimeCGGain = 0.015,
    chassisFlexCGGain = 0.018,

    roadBodyPitchGain = 0.012,
    roadBodyRollGain = 0.010,
    loadPathPitchGain = 0.010,
    loadPathRollGain = 0.009,
    impactCGGain = 0.010,

    maxRootCGAdd = 0.035,
    maxCompression = 0.20,

    loadReference = 3200.0,
    minDt = 0.00005,
    maxDt = 0.050,

    debugStoreInterval = 0.25,
}

local state = {
    frontCompression = 0.0,
    rearCompression = 0.0,
    leftCompression = 0.0,
    rightCompression = 0.0,

    targetFrontCompression = 0.0,
    targetRearCompression = 0.0,
    targetLeftCompression = 0.0,
    targetRightCompression = 0.0,

    pitchCG = 0.0,
    rollCG = 0.0,
    targetPitchCG = 0.0,
    targetRollCG = 0.0,
    pitchState = "NEUTRAL",
    rollState = "NEUTRAL",

    load = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    springForce = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    damperForce = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireHop = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    loadPathLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadPathBody = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    roadVertical = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    chassisEnergy = 0.0,
    chassisRelease = 0.0,
    brakeInput = 0.0,
    driveTorque = 0.0,

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyBendingFactor = 1.0,
    bodyDampingFactor = 1.0,

    roadBodyPitch = 0.0,
    roadBodyRoll = 0.0,
    roadBodyHeave = 0.0,
    roadBodyYaw = 0.0,

    suspensionStress = 0.0,
    brakeLockAvg = 0.0,
    weightPitch = 0.0,
    weightRoll = 0.0,
    damageRuntimeAdd = 0.0,
    chassisFlexEnergy = 0.0,
    impactValue = 0.0,

    rootPitchCGAdd = 0.0,
    rootRollCGAdd = 0.0,
    avgCompression = 0.0,
    maxAbsCG = 0.0,

    status = "INIT",
    updateCount = 0,

    carLinked = false,
    loadLinked = false,
    springLinked = false,
    damperLinked = false,
    tireLinked = false,
    brakeLinked = false,
    driveLinked = false,
    energyLinked = false,
    bodyLinked = false,
    suspensionLinked = false,
    weightLinked = false,
    damageRuntimeLinked = false,
    chassisFlexLinked = false,
    roadBodyLinked = false,
    loadPathLinked = false,
    impactLinked = false,

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

local function clamp(value, minValue, maxValue)
    value = safeNumber(value, minValue or 0.0)
    if value < minValue then return minValue end
    if value > maxValue then return maxValue end
    return value
end

local function abs(value)
    return math.abs(safeNumber(value, 0.0))
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.001)
    dt = safeNumber(dt, 0.0)

    if tau <= 0.0001 then
        return target
    end

    return current + (target - current) * clamp(dt / math.max(tau + dt, 0.0001), 0.0, 1.0)
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

local function safeLoadAlt(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return safeNumber(value, defaultValue or 0.0), keys[i]
        end
    end
    return defaultValue or 0.0, nil
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

    if ok then
        return car
    end

    return nil
end

local function updateDebugGate(dt)
    state.debugStoreTimer = (state.debugStoreTimer or 0.0) + (dt or 0.0)

    if state.debugStoreTimer >= (M.params.debugStoreInterval or 0.25) then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    else
        state.debugStoreNow = false
    end
end

local function getWheel(car, index)
    local wheels = safeField(car, "wheels", nil)
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

local function getWheelFieldLoad(wheel)
    if not wheel then
        return nil
    end

    local candidates = { "load", "loadK" }
    for i = 1, #candidates do
        local value = safeField(wheel, candidates[i], nil)
        if value ~= nil then
            return safeNumber(value, nil)
        end
    end

    return nil
end

local function readWheelLoad(index, wheel)
    local value, key = safeLoadAlt(nil,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index,
        "ngp_tire_load_input_" .. index,
        "ngp_susp_load_input_" .. index,
        "ngp_load_path_load_" .. index,
        "ngp_contact_load_" .. index,
        "ngp_tire_state_load_" .. index
    )

    if key ~= nil then
        state.loadLinked = true
        return clamp(value, 0.0, 12000.0)
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        state.loadLinked = true
        local n = safeNumber(dlt, 0.0)
        if abs(n) > 900.0 then
            return clamp(abs(n), 0.0, 12000.0)
        end
        return clamp((M.params.loadReference or 3200.0) + n, 0.0, 12000.0)
    end

    local wheelLoad = getWheelFieldLoad(wheel)
    if wheelLoad ~= nil then
        state.carLinked = true
        return clamp(abs(wheelLoad), 0.0, 12000.0)
    end

    return M.params.loadReference or 3200.0
end

local function readInputs(car)
    state.carLinked = car ~= nil
    state.loadLinked = false
    state.springLinked = false
    state.damperLinked = false
    state.tireLinked = false
    state.brakeLinked = false
    state.driveLinked = false
    state.energyLinked = false
    state.bodyLinked = false
    state.suspensionLinked = false
    state.weightLinked = false
    state.damageRuntimeLinked = false
    state.chassisFlexLinked = false
    state.roadBodyLinked = false
    state.loadPathLinked = false
    state.impactLinked = false

    for i = 0, 3 do
        local wheel = getWheel(car, i)
        state.load[i] = readWheelLoad(i, wheel)

        local spring, springKey = safeLoadAlt(0.0,
            "ngp_spring_force_" .. i,
            "ngp_progressive_spring_" .. i,
            "ngp_spring_raw_" .. i
        )
        state.springLinked = state.springLinked or springKey ~= nil
        state.springForce[i] = spring

        local damper, damperKey = safeLoadAlt(0.0,
            "ngp_damper_force_" .. i,
            "ngp_damper_" .. i,
            "ngp_damper_model_force_" .. i,
            "ngp_damper_hyst_force_" .. i
        )
        state.damperLinked = state.damperLinked or damperKey ~= nil
        state.damperForce[i] = damper

        local hop, hopKey = safeLoadAlt(0.0,
            "ngp_tire_hop_energy_" .. i,
            "ngp_tire_hop_" .. i,
            "ngp_tirehop_energy_" .. i,
            "ngp_rii_hop_" .. i
        )
        state.tireLinked = state.tireLinked or hopKey ~= nil
        state.tireHop[i] = clamp(hop, 0.0, 1.0)

        local lpLoss, lpLossKey = safeLoadAlt(0.0,
            "ngp_load_path_loss_" .. i,
            "ngp_lp_loss_" .. i,
            "ngp_road_path_loss_" .. i,
            "ngp_rii_path_loss_" .. i
        )
        local lpBody, lpBodyKey = safeLoadAlt(0.0,
            "ngp_load_path_body_absorb_" .. i,
            "ngp_road_body_absorb_" .. i,
            "ngp_lp_body_" .. i
        )
        local roadVertical, roadKey = safeLoadAlt(0.0,
            "ngp_road_vertical_flow_" .. i,
            "ngp_lp_vertical_flow_" .. i,
            "ngp_rii_vertical_" .. i,
            "ngp_road_input_vertical_" .. i
        )
        state.loadPathLinked = state.loadPathLinked or lpLossKey ~= nil or lpBodyKey ~= nil or roadKey ~= nil
        state.loadPathLoss[i] = clamp(lpLoss, 0.0, 1.2)
        state.loadPathBody[i] = clamp(lpBody, 0.0, 1.2)
        state.roadVertical[i] = clamp(roadVertical, 0.0, 1.5)
    end

    local energy, energyKey = safeLoadAlt(0.0,
        "ngp_chassis_energy",
        "ngp_chassis_body_energy",
        "ngp_chassis_target_energy"
    )
    local release, releaseKey = safeLoadAlt(0.0,
        "ngp_chassis_release",
        "ngp_chassis_body_release"
    )
    state.energyLinked = energyKey ~= nil or releaseKey ~= nil
    state.chassisEnergy = clamp(energy, 0.0, 1.5)
    state.chassisRelease = clamp(release, 0.0, 1.5)

    local brakeRaw = safeLoadRaw("ngp_brake_input")
    state.brakeInput = clamp(safeNumber(brakeRaw, safeField(car, "brake", 0.0)), 0.0, 1.0)
    state.brakeLinked = brakeRaw ~= nil

    local drive, driveKey = safeLoadAlt(0.0,
        "ngp_drive_torque",
        "ngp_drive_transmitted_torque",
        "ngp_driveline_torque",
        "ngp_dt_torque"
    )
    state.driveLinked = driveKey ~= nil
    state.driveTorque = drive

    local bodyRaw = safeLoadRaw("ngp_body_rigidity")
    local bodyFlexRaw = safeLoadRaw("ngp_body_flex_factor")
    state.bodyLinked = bodyRaw ~= nil or bodyFlexRaw ~= nil
    state.bodyRigidity = clamp(safeNumber(bodyRaw, 1.0), 0.20, 1.35)
    state.bodyFlexFactor = clamp(safeNumber(bodyFlexRaw, 1.0), 0.25, 1.80)
    state.bodyTorsionFactor = clamp(safeLoad("ngp_body_torsion_factor", 1.0), 0.35, 2.00)
    state.bodyBendingFactor = clamp(safeLoad("ngp_body_bending_factor", 1.0), 0.35, 2.00)
    state.bodyDampingFactor = clamp(safeLoad("ngp_body_damping_factor", 1.0), 0.50, 1.80)

    local rHeave, rHeaveKey = safeLoadAlt(0.0, "ngp_body_heave_input", "ngp_road_body_heave", "ngp_rbi_heave")
    local rPitch, rPitchKey = safeLoadAlt(0.0, "ngp_body_pitch_input", "ngp_road_body_pitch", "ngp_rbi_pitch")
    local rRoll, rRollKey = safeLoadAlt(0.0, "ngp_body_roll_input", "ngp_road_body_roll", "ngp_rbi_roll")
    local rYaw, rYawKey = safeLoadAlt(0.0, "ngp_body_yaw_hint", "ngp_road_body_yaw", "ngp_rbi_yaw")
    state.roadBodyLinked = rHeaveKey ~= nil or rPitchKey ~= nil or rRollKey ~= nil or rYawKey ~= nil
    state.roadBodyHeave = clamp(rHeave, -2.0, 2.0)
    state.roadBodyPitch = clamp(rPitch, -2.0, 2.0)
    state.roadBodyRoll = clamp(rRoll, -2.0, 2.0)
    state.roadBodyYaw = clamp(rYaw, -2.0, 2.0)

    local susp, suspKey = safeLoadAlt(0.0,
        "ngp_weight_suspension_stress",
        "ngp_chassis_flex_suspension_stress",
        "ngp_suspension_stress",
        "ngp_load_path_avg_work"
    )
    state.suspensionLinked = suspKey ~= nil
    state.suspensionStress = clamp(susp, 0.0, 1.0)

    local lockSum = 0.0
    local lockLinked = false
    for i = 0, 3 do
        local lock = safeLoadRaw("ngp_brake_lock_" .. i)
        if lock ~= nil then lockLinked = true end
        lockSum = lockSum + clamp(safeNumber(lock, 0.0), 0.0, 1.0)
    end
    state.brakeLockAvg = lockSum * 0.25
    state.brakeLinked = state.brakeLinked or lockLinked

    local fb = safeLoadRaw("ngp_weight_front_bias") or safeLoadRaw("ngp_load_front") or safeLoadRaw("ngp_front_bias")
    local rb = safeLoadRaw("ngp_weight_rear_bias") or safeLoadRaw("ngp_load_rear") or safeLoadRaw("ngp_rear_bias")
    local lb = safeLoadRaw("ngp_weight_left_bias") or safeLoadRaw("ngp_load_left") or safeLoadRaw("ngp_left_bias")
    local rr = safeLoadRaw("ngp_weight_right_bias") or safeLoadRaw("ngp_load_right") or safeLoadRaw("ngp_right_bias")
    state.weightLinked = fb ~= nil or rb ~= nil or lb ~= nil or rr ~= nil
    state.weightPitch = clamp(safeNumber(fb, 0.5) - safeNumber(rb, 0.5), -1.0, 1.0)
    state.weightRoll = clamp(safeNumber(lb, 0.5) - safeNumber(rr, 0.5), -1.0, 1.0)

    local damage, damageKey = safeLoadAlt(0.0,
        "ngp_damage_runtime_add",
        "ngp_vehicle_condition_total",
        "ngp_condition_total",
        "ngp_damage_total"
    )
    state.damageRuntimeLinked = damageKey ~= nil
    state.damageRuntimeAdd = clamp(damage, 0.0, 1.0)

    local flex, flexKey = safeLoadAlt(0.0,
        "ngp_chassis_flex_energy",
        "ngp_flex_energy",
        "ngp_chassis_energy"
    )
    state.chassisFlexLinked = flexKey ~= nil
    state.chassisFlexEnergy = clamp(flex, 0.0, 1.0)

    local impact, impactKey = safeLoadAlt(0.0,
        "ngp_impact_value",
        "ngp_impact_root_value",
        "ngp_rii_avg_impact",
        "ngp_road_impact"
    )
    state.impactLinked = impactKey ~= nil
    state.impactValue = clamp(impact, 0.0, 1.0)

    local pitchPath =
        ((state.loadPathLoss[FL] + state.loadPathLoss[FR]) - (state.loadPathLoss[RL] + state.loadPathLoss[RR])) * 0.5
    local rollPath =
        ((state.loadPathBody[FL] + state.loadPathBody[RL]) - (state.loadPathBody[FR] + state.loadPathBody[RR])) * 0.5

    state.rootPitchCGAdd = clamp(
        state.suspensionStress * M.params.suspensionStressCGGain
        + state.brakeLockAvg * M.params.brakeLockCGGain
        + state.weightPitch * M.params.weightBiasCGGain
        + state.damageRuntimeAdd * M.params.damageRuntimeCGGain
        + state.chassisFlexEnergy * M.params.chassisFlexCGGain
        + state.roadBodyPitch * M.params.roadBodyPitchGain
        + pitchPath * M.params.loadPathPitchGain
        + state.impactValue * M.params.impactCGGain,
        -M.params.maxRootCGAdd,
        M.params.maxRootCGAdd
    )

    state.rootRollCGAdd = clamp(
        state.suspensionStress * M.params.suspensionStressCGGain
        + state.weightRoll * M.params.weightBiasCGGain
        + state.damageRuntimeAdd * M.params.damageRuntimeCGGain
        + state.chassisFlexEnergy * M.params.chassisFlexCGGain
        + state.roadBodyRoll * M.params.roadBodyRollGain
        + rollPath * M.params.loadPathRollGain,
        -M.params.maxRootCGAdd,
        M.params.maxRootCGAdd
    )
end

local function decayWhenNoCar(dt)
    state.frontCompression = lowPass(state.frontCompression, 0.0, M.params.pitchTau, dt)
    state.rearCompression = lowPass(state.rearCompression, 0.0, M.params.pitchTau, dt)
    state.leftCompression = lowPass(state.leftCompression, 0.0, M.params.rollTau, dt)
    state.rightCompression = lowPass(state.rightCompression, 0.0, M.params.rollTau, dt)

    state.targetFrontCompression = 0.0
    state.targetRearCompression = 0.0
    state.targetLeftCompression = 0.0
    state.targetRightCompression = 0.0

    state.pitchCG = lowPass(state.pitchCG, 0.0, M.params.pitchTau, dt)
    state.rollCG = lowPass(state.rollCG, 0.0, M.params.rollTau, dt)
    state.targetPitchCG = 0.0
    state.targetRollCG = 0.0
    state.rootPitchCGAdd = lowPass(state.rootPitchCGAdd, 0.0, M.params.pitchTau, dt)
    state.rootRollCGAdd = lowPass(state.rootRollCGAdd, 0.0, M.params.rollTau, dt)

    state.pitchState = "NEUTRAL"
    state.rollState = "NEUTRAL"
    state.avgCompression = 0.0
    state.maxAbsCG = math.max(abs(state.pitchCG), abs(state.rollCG))
end

local function calculateTargetCompression()
    local fl = state.load[FL] or 0.0
    local fr = state.load[FR] or 0.0
    local rl = state.load[RL] or 0.0
    local rr = state.load[RR] or 0.0

    local frontLoad = (fl + fr) * 0.5
    local rearLoad = (rl + rr) * 0.5
    local leftLoad = (fl + rl) * 0.5
    local rightLoad = (fr + rr) * 0.5

    local frontSpring = math.max(M.params.frontSpringRef or 32000.0, 1.0)
    local rearSpring = math.max(M.params.rearSpringRef or 36000.0, 1.0)
    local avgSpring = math.max((frontSpring + rearSpring) * 0.5, 1.0)

    local springFront = (state.springForce[FL] + state.springForce[FR]) * 0.5 * M.params.springForceGain
    local springRear = (state.springForce[RL] + state.springForce[RR]) * 0.5 * M.params.springForceGain
    local damperFront = (abs(state.damperForce[FL]) + abs(state.damperForce[FR])) * 0.5 * M.params.damperForceGain
    local damperRear = (abs(state.damperForce[RL]) + abs(state.damperForce[RR])) * 0.5 * M.params.damperForceGain

    local verticalFront = (state.roadVertical[FL] + state.roadVertical[FR]) * 0.5 * 0.012
    local verticalRear = (state.roadVertical[RL] + state.roadVertical[RR]) * 0.5 * 0.012

    state.targetFrontCompression = clamp(
        frontLoad / frontSpring + springFront + damperFront + verticalFront,
        -M.params.maxCompression,
        M.params.maxCompression
    )

    state.targetRearCompression = clamp(
        rearLoad / rearSpring + springRear + damperRear + verticalRear,
        -M.params.maxCompression,
        M.params.maxCompression
    )

    state.targetLeftCompression = clamp(leftLoad / avgSpring, -M.params.maxCompression, M.params.maxCompression)
    state.targetRightCompression = clamp(rightLoad / avgSpring, -M.params.maxCompression, M.params.maxCompression)
end

local function updateCompression(dt)
    local bodyTau =
        1.0
        + math.max((state.bodyFlexFactor or 1.0) - 1.0, 0.0) * 0.30
        - math.max((state.bodyRigidity or 1.0) - 1.0, 0.0) * 0.08

    bodyTau = clamp(bodyTau / math.max(state.bodyDampingFactor or 1.0, 0.50), 0.75, 1.35)

    state.frontCompression = lowPass(state.frontCompression, state.targetFrontCompression, M.params.pitchTau * bodyTau, dt)
    state.rearCompression = lowPass(state.rearCompression, state.targetRearCompression, M.params.pitchTau * bodyTau, dt)
    state.leftCompression = lowPass(state.leftCompression, state.targetLeftCompression, M.params.rollTau * bodyTau, dt)
    state.rightCompression = lowPass(state.rightCompression, state.targetRightCompression, M.params.rollTau * bodyTau, dt)

    state.avgCompression =
        (abs(state.frontCompression) + abs(state.rearCompression) + abs(state.leftCompression) + abs(state.rightCompression)) * 0.25
end

local function calculateCG()
    local bodyScale =
        1.0
        + math.max((state.bodyFlexFactor or 1.0) - 1.0, 0.0) * M.params.bodyFlexCGGain
        - math.max((state.bodyRigidity or 1.0) - 1.0, 0.0) * M.params.bodyStiffCGLoss

    bodyScale = clamp(bodyScale, 0.85, 1.20)

    local pitchCG =
        (state.frontCompression - state.rearCompression) * M.params.pitchCGScale * bodyScale
        + state.chassisEnergy * M.params.chassisEnergyPitchGain
        - state.chassisRelease * M.params.chassisReleasePitchGain
        + state.brakeInput * M.params.brakePitchGain
        - abs(state.driveTorque) * M.params.drivePitchGain
        + state.roadBodyPitch * M.params.roadBodyPitchGain
        + state.rootPitchCGAdd

    local rollCG =
        (state.leftCompression - state.rightCompression) * M.params.rollCGScale * bodyScale
        + state.chassisEnergy * M.params.chassisEnergyRollGain
        - state.chassisRelease * M.params.chassisReleaseRollGain
        + ((state.tireHop[FL] + state.tireHop[RL]) - (state.tireHop[FR] + state.tireHop[RR])) * 0.5 * M.params.tireHopCGGain
        + state.roadBodyRoll * M.params.roadBodyRollGain
        + state.rootRollCGAdd

    state.targetPitchCG = clamp(pitchCG, -M.params.maxPitchCG, M.params.maxPitchCG)
    state.targetRollCG = clamp(rollCG, -M.params.maxRollCG, M.params.maxRollCG)

    state.pitchCG = lowPass(state.pitchCG, state.targetPitchCG, M.params.pitchTau, M._dt or 0.016)
    state.rollCG = lowPass(state.rollCG, state.targetRollCG, M.params.rollTau, M._dt or 0.016)

    state.pitchState = abs(state.pitchCG) < 0.005 and "NEUTRAL" or (state.pitchCG > 0.0 and "FRONT CG" or "REAR CG")
    state.rollState = abs(state.rollCG) < 0.005 and "NEUTRAL" or (state.rollCG > 0.0 and "LEFT CG" or "RIGHT CG")
    state.maxAbsCG = math.max(abs(state.pitchCG), abs(state.rollCG))
end

local function exportState()
    safeStore("ngp_pitch_cg", state.pitchCG)
    safeStore("ngp_roll_cg", state.rollCG)
    safeStore("ngp_sprung_front_compression", state.frontCompression)
    safeStore("ngp_sprung_rear_compression", state.rearCompression)
    safeStore("ngp_sprung_left_compression", state.leftCompression)
    safeStore("ngp_sprung_right_compression", state.rightCompression)
    safeStore("ngp_sprung_pitch_state", state.pitchState)
    safeStore("ngp_sprung_roll_state", state.rollState)
    safeStore("ngp_sprung_mass_status", state.status)
    safeStore("ngp_sprung_mass_update_count", state.updateCount)
    safeStore("ngp_sprung_root_pitch_cg_add", state.rootPitchCGAdd)
    safeStore("ngp_sprung_root_roll_cg_add", state.rootRollCGAdd)

    safeStore("ngp_sprung_target_pitch_cg", state.targetPitchCG)
    safeStore("ngp_sprung_target_roll_cg", state.targetRollCG)
    safeStore("ngp_sprung_avg_compression", state.avgCompression)
    safeStore("ngp_sprung_max_abs_cg", state.maxAbsCG)

    safeStore("ngp_sprung_front_target", state.targetFrontCompression)
    safeStore("ngp_sprung_rear_target", state.targetRearCompression)
    safeStore("ngp_sprung_left_target", state.targetLeftCompression)
    safeStore("ngp_sprung_right_target", state.targetRightCompression)

    safeStore("ngp_sm_pitch_cg", state.pitchCG)
    safeStore("ngp_sm_roll_cg", state.rollCG)
    safeStore("ngp_sm_avg_compression", state.avgCompression)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_sprung_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_sprung_spring_linked", state.springLinked and 1 or 0)
    safeStore("ngp_sprung_damper_linked", state.damperLinked and 1 or 0)
    safeStore("ngp_sprung_tire_linked", state.tireLinked and 1 or 0)
    safeStore("ngp_sprung_energy_linked", state.energyLinked and 1 or 0)
    safeStore("ngp_sprung_body_linked", state.bodyLinked and 1 or 0)
    safeStore("ngp_sprung_suspension_linked", state.suspensionLinked and 1 or 0)
    safeStore("ngp_sprung_weight_linked", state.weightLinked and 1 or 0)
    safeStore("ngp_sprung_damage_runtime_linked", state.damageRuntimeLinked and 1 or 0)
    safeStore("ngp_sprung_chassis_flex_linked", state.chassisFlexLinked and 1 or 0)

    safeStore("ngp_sprung_car_linked", state.carLinked and 1 or 0)
    safeStore("ngp_sprung_road_body_linked", state.roadBodyLinked and 1 or 0)
    safeStore("ngp_sprung_load_path_linked", state.loadPathLinked and 1 or 0)
    safeStore("ngp_sprung_impact_linked", state.impactLinked and 1 or 0)

    for i = 0, 3 do
        safeStore("ngp_sprung_load_" .. i, state.load[i] or 0.0)
        safeStore("ngp_sprung_spring_force_" .. i, state.springForce[i] or 0.0)
        safeStore("ngp_sprung_damper_force_" .. i, state.damperForce[i] or 0.0)
        safeStore("ngp_sprung_hop_" .. i, state.tireHop[i] or 0.0)
    end
end

function M.init()
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = clamp(safeNumber(dt, 0.0), M.params.minDt, M.params.maxDt)
    M._dt = dt
    updateDebugGate(dt)

    car = car or safeGetCar()

    if not car then
        state.status = "NO CAR"
        state.carLinked = false
        readInputs(nil)
        decayWhenNoCar(dt)
        exportState()
        return
    end

    state.status = "RUNNING"

    readInputs(car)
    calculateTargetCompression()
    updateCompression(dt)
    calculateCG()

    exportState()
end

function M.getPitchCG()
    return state.pitchCG or 0.0
end

function M.getRollCG()
    return state.rollCG or 0.0
end

function M.getCompression()
    return {
        front = state.frontCompression or 0.0,
        rear = state.rearCompression or 0.0,
        left = state.leftCompression or 0.0,
        right = state.rightCompression or 0.0,
    }
end

function M.getState()
    return state
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f\n" ..
        "PitchCG %.4f -> %.4f / %s\n" ..
        "RollCG %.4f -> %.4f / %s\n" ..
        "Comp F/R %.4f %.4f / L/R %.4f %.4f Avg %.4f\n" ..
        "Target F/R %.4f %.4f / L/R %.4f %.4f\n" ..
        "RootCG P/R %.4f %.4f / MaxCG %.4f\n" ..
        "Inputs Susp %.3f BrakeLock %.3f Damage %.3f Flex %.3f Impact %.3f\n" ..
        "Links Load:%s Spring:%s Damper:%s Tire:%s Energy:%s Body:%s RBI:%s LP:%s",
        tostring(state.status),
        state.updateCount or 0,

        state.pitchCG or 0.0,
        state.targetPitchCG or 0.0,
        tostring(state.pitchState),

        state.rollCG or 0.0,
        state.targetRollCG or 0.0,
        tostring(state.rollState),

        state.frontCompression or 0.0,
        state.rearCompression or 0.0,
        state.leftCompression or 0.0,
        state.rightCompression or 0.0,
        state.avgCompression or 0.0,

        state.targetFrontCompression or 0.0,
        state.targetRearCompression or 0.0,
        state.targetLeftCompression or 0.0,
        state.targetRightCompression or 0.0,

        state.rootPitchCGAdd or 0.0,
        state.rootRollCGAdd or 0.0,
        state.maxAbsCG or 0.0,

        state.suspensionStress or 0.0,
        state.brakeLockAvg or 0.0,
        state.damageRuntimeAdd or 0.0,
        state.chassisFlexEnergy or 0.0,
        state.impactValue or 0.0,

        state.loadLinked and "OK" or "NIL",
        state.springLinked and "OK" or "NIL",
        state.damperLinked and "OK" or "NIL",
        state.tireLinked and "OK" or "NIL",
        state.energyLinked and "OK" or "NIL",
        state.bodyLinked and "OK" or "NIL",
        state.roadBodyLinked and "OK" or "NIL",
        state.loadPathLinked and "OK" or "NIL"
    )
end

return M
