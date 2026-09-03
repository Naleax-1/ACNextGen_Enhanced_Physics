---@diagnostic disable: undefined-global

--============================================================
-- ACNeXtGen
-- steering_dynamics.lua
-- Phase 8.1 / V1.1.5
-- Steering Rack / Tie Rod / Knuckle Dynamics
-- Root/trunk steering transient model
--============================================================

local M = {}

M.params = {
    rackTau    = 0.045,
    tieRodTau  = 0.030,
    knuckleTau = 0.040,

    tieRodGain  = 0.00008,
    knuckleGain = 0.00035,

    friction = 0.08,
    fxKnuckleMix = 0.075,
    minFrictionRack = 0.01,

    maxReaction = 1.50,
    maxOutput   = 2.00,

    bodySteerInputGain = 0.32,
    flexDelayGain      = 0.28,
    flexYawGain        = 0.22,
    flexRollGain       = 0.08,
    flexEnergyReactionGain = 0.18,
    flexReleaseReturnGain  = 0.22,

    maxEffectiveInput = 1.60,

    bodyReadInterval = 0.25,
    bodyFlexRackTauGain    = 0.36,
    bodyTorsionRackTauGain = 0.22,
    bodyDampingTauGain     = 0.12,
    bodyStiffRackTauLoss   = 0.08,

    bodyFlexReactionGain  = 0.10,
    bodyStiffReactionLoss = 0.06,

    minBodyTauScale = 0.82,
    maxBodyTauScale = 1.32,
    minBodyReactionScale = 0.84,
    maxBodyReactionScale = 1.18,

    casterGripReactionGain = 0.16,
    casterToeReactionGain = 0.018,
    casterCamberReactionGain = 0.010,
    casterGripFrictionGain = 0.12,

    minCasterScale = 0.86,
    maxCasterScale = 1.14,

    frontAxisInputGain = 0.055,
    frontAxisReactionGain = 0.16,
    frontAxisFrictionGain = 0.08,
    roadShockReactionGain = 0.07,
    pathLossReactionLoss = 0.08,

    noCarDecayTau = 0.120,
    debugStoreInterval = 0.25,
}

local state = {
    input = 0.0,
    effectiveInput = 0.0,

    rack = 0.0,
    tieRod = 0.0,
    knuckle = 0.0,

    reaction = 0.0,
    friction = 0.0,
    output = 0.0,

    fy = 0.0,
    mz = 0.0,
    fx = 0.0,

    speedKmh = 0.0,

    bodySteer = 0.0,
    flexDelay = 0.0,
    flexYaw = 0.0,
    flexRoll = 0.0,
    flexPitch = 0.0,
    flexEnergy = 0.0,
    flexRelease = 0.0,

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyBendingFactor = 1.0,
    bodyDampingFactor = 1.0,
    bodyTauScale = 1.0,
    bodyReactionScale = 1.0,
    bodyReadTimer = 999.0,

    casterGripFL = 1.0,
    casterGripFR = 1.0,
    casterGripAvg = 1.0,
    casterGripLoss = 0.0,
    casterToeFL = 0.0,
    casterToeFR = 0.0,
    casterToeAbs = 0.0,
    casterCamberFL = 0.0,
    casterCamberFR = 0.0,
    casterCamberAbs = 0.0,
    casterScale = 1.0,

    frontAxisAnchor = 0.0,
    frontAxisAuthority = 0.0,
    frontAxisYawResist = 0.0,
    frontAxisSteerWeight = 0.0,
    frontAxisSlipDamp = 0.0,

    roadShock = 0.0,
    roadSurfaceLimit = 0.0,
    pathLoss = 0.0,
    tireDelivery = 1.0,

    wheelsValid = false,
    frontWheelsValid = false,

    bodyLinked = false,
    flexLinked = false,
    casterLinked = false,
    frontAxisLinked = false,
    roadLinked = false,
    forceLinked = false,

    status = "INIT",
    updateCount = 0,

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
    if v < minValue then return minValue end
    if v > maxValue then return maxValue end
    return v
end

local function sign(v)
    v = safeNumber(v, 0.0)
    if v < 0.0 then return -1.0 end
    if v > 0.0 then return 1.0 end
    return 0.0
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

    if state.debugStoreTimer >= M.params.debugStoreInterval then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    else
        state.debugStoreNow = false
    end
end

local function getWheels(car)
    return safeField(car, "wheels", nil)
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

local function getSpeedKmh(car)
    local speedKmh = safeField(car, "speedKmh", nil)
    if speedKmh ~= nil then
        return safeNumber(speedKmh, 0.0)
    end

    local speed = nil
    if speed ~= nil then
        local n = safeNumber(speed, 0.0)
        if n > 0.0 and n < 120.0 then
            return n * 3.6
        end
        return n
    end

    local velocity = safeField(car, "velocity", nil)
    if velocity then
        local x = safeNumber(safeField(velocity, "x", 0.0), 0.0)
        local y = safeNumber(safeField(velocity, "y", 0.0), 0.0)
        local z = safeNumber(safeField(velocity, "z", 0.0), 0.0)
        return math.sqrt(x * x + y * y + z * z) * 3.6
    end

    return 0.0
end

local function getSteer(car)
    local steer = safeField(car, "steer", nil)
    if steer == nil then
        steer = nil
    end
    if steer == nil then
        steer = safeLoadRaw("ngp_driver_steer")
    end
    return clamp(safeNumber(steer, 0.0), -1.5, 1.5)
end

local function readWheelForceField(wheel, ...)
    if not wheel then
        return nil
    end

    local keys = { ... }
    for i = 1, #keys do
        local value = safeField(wheel, keys[i], nil)
        if value ~= nil then
            return safeNumber(value, 0.0)
        end
    end

    return nil
end

local function readLatForce(index, wheel)
    local value = readWheelForceField(wheel, "fy", "Fy", "lateralForce", "forceY")
    if value ~= nil then
        return value
    end

    local stored = safeLoadAlt(
        0.0,
        "ngp_hub_force_lat_" .. index,
        "ngp_tire_lat_" .. index,
        "ngp_lat_" .. index,
        "ngp_cp_lat_" .. index,
        "ngp_tire_force_lat_" .. index
    )

    return stored
end

local function readLongForce(index, wheel)
    local value = readWheelForceField(wheel, "fx", "Fx", "longitudinalForce", "forceX")
    if value ~= nil then
        return value
    end

    local stored = safeLoadAlt(
        0.0,
        "ngp_hub_force_long_" .. index,
        "ngp_tire_long_" .. index,
        "ngp_long_" .. index,
        "ngp_cp_long_" .. index,
        "ngp_tire_force_long_" .. index
    )

    return stored
end

local function readAligningMoment(index, wheel)
    local value = readWheelForceField(wheel, "mz", "Mz", "aligningTorque", "aligningMoment")
    if value ~= nil then
        return value
    end

    local stored = safeLoadAlt(
        0.0,
        "ngp_caster_aligning_torque_" .. index,
        "ngp_aligning_torque_" .. index,
        "ngp_tire_mz_" .. index,
        "ngp_steer_mz_" .. index,
        "ngp_tire_aligning_" .. index
    )

    return stored
end

local function readFrontForces(car)
    local fl = getWheel(car, 0)
    local fr = getWheel(car, 1)

    state.frontWheelsValid = fl ~= nil and fr ~= nil

    local fyL = readLatForce(0, fl)
    local fyR = readLatForce(1, fr)
    local fxL = readLongForce(0, fl)
    local fxR = readLongForce(1, fr)
    local mzL = readAligningMoment(0, fl)
    local mzR = readAligningMoment(1, fr)

    state.forceLinked =
        fyL ~= 0.0 or fyR ~= 0.0 or fxL ~= 0.0 or fxR ~= 0.0 or mzL ~= 0.0 or mzR ~= 0.0

    return (fyL + fyR) * 0.5, (mzL + mzR) * 0.5, (fxL + fxR) * 0.5
end

local function readChassisFlex()
    local bodySteer, kBodySteer = safeLoadAlt(0.0, "ngp_body_steer", "ngp_steer_body", "ngp_chassis_body_steer")
    local flexDelay, kDelay = safeLoadAlt(0.0, "ngp_chassis_flex_delay", "ngp_flex_delay", "ngp_chassis_delay")
    local flexYaw, kYaw = safeLoadAlt(0.0, "ngp_chassis_flex_yaw", "ngp_flex_yaw", "ngp_yaw_deflection")
    local flexRoll, kRoll = safeLoadAlt(0.0, "ngp_chassis_flex_roll", "ngp_flex_roll", "ngp_body_roll")
    local flexPitch, kPitch = safeLoadAlt(0.0, "ngp_chassis_flex_pitch", "ngp_flex_pitch", "ngp_body_pitch")
    local flexEnergy, kEnergy = safeLoadAlt(0.0, "ngp_chassis_flex_energy", "ngp_flex_energy", "ngp_chassis_energy")
    local flexRelease, kRelease = safeLoadAlt(0.0, "ngp_chassis_flex_release", "ngp_flex_release", "ngp_chassis_release")

    state.flexLinked =
        kBodySteer ~= nil or kDelay ~= nil or kYaw ~= nil or kRoll ~= nil or
        kPitch ~= nil or kEnergy ~= nil or kRelease ~= nil

    state.bodySteer = clamp(bodySteer, -1.0, 1.0)
    state.flexDelay = clamp(flexDelay, -1.2, 1.2)
    state.flexYaw = clamp(flexYaw, -1.2, 1.2)
    state.flexRoll = clamp(flexRoll, -1.2, 1.2)
    state.flexPitch = clamp(flexPitch, -1.2, 1.2)
    state.flexEnergy = clamp(flexEnergy, 0.0, 1.5)
    state.flexRelease = clamp(flexRelease, 0.0, 1.5)
end

local function readBodyRigidity(dt)
    state.bodyReadTimer = (state.bodyReadTimer or 999.0) + (dt or 0.0)

    if state.bodyLinked and state.bodyReadTimer < M.params.bodyReadInterval then
        return
    end

    state.bodyReadTimer = 0.0

    local bodyCount = safeLoadRaw("ngp_body_rigidity_update_count")
    local bodyRigidity = safeLoadRaw("ngp_body_rigidity")
    local flex = safeLoadRaw("ngp_body_flex_factor")
    local torsion = safeLoadRaw("ngp_body_torsion_factor")
    local bending = safeLoadRaw("ngp_body_bending_factor")
    local damping = safeLoadRaw("ngp_body_damping_factor")

    state.bodyLinked = bodyCount ~= nil or bodyRigidity ~= nil or flex ~= nil or torsion ~= nil or bending ~= nil or damping ~= nil

    state.bodyRigidity = clamp(safeNumber(bodyRigidity, 1.0), 0.20, 1.35)
    state.bodyFlexFactor = clamp(safeNumber(flex, 1.0), 0.25, 1.80)
    state.bodyTorsionFactor = clamp(safeNumber(torsion, 1.0), 0.35, 2.00)
    state.bodyBendingFactor = clamp(safeNumber(bending, 1.0), 0.35, 2.00)
    state.bodyDampingFactor = clamp(safeNumber(damping, 1.0), 0.50, 1.80)
end

local function updateBodyScales(dt)
    local p = M.params

    if not state.bodyLinked then
        state.bodyTauScale = lowPass(state.bodyTauScale, 1.0, 0.080, dt)
        state.bodyReactionScale = lowPass(state.bodyReactionScale, 1.0, 0.080, dt)
        return
    end

    local softness = clamp((state.bodyFlexFactor or 1.0) - 1.0, 0.0, 0.80)
    local torsionSoft = clamp((state.bodyTorsionFactor or 1.0) - 1.0, 0.0, 1.0)
    local stiff = clamp((state.bodyRigidity or 1.0) - 1.0, 0.0, 0.35)
    local dampingExtra = clamp((state.bodyDampingFactor or 1.0) - 1.0, 0.0, 0.80)

    local tauTarget =
        1.0 +
        softness * p.bodyFlexRackTauGain +
        torsionSoft * p.bodyTorsionRackTauGain +
        dampingExtra * p.bodyDampingTauGain -
        stiff * p.bodyStiffRackTauLoss

    local reactionTarget =
        1.0 +
        softness * p.bodyFlexReactionGain -
        stiff * p.bodyStiffReactionLoss

    state.bodyTauScale = lowPass(
        state.bodyTauScale,
        clamp(tauTarget, p.minBodyTauScale, p.maxBodyTauScale),
        0.080,
        dt
    )

    state.bodyReactionScale = lowPass(
        state.bodyReactionScale,
        clamp(reactionTarget, p.minBodyReactionScale, p.maxBodyReactionScale),
        0.080,
        dt
    )
end

local function readCasterInput()
    local gripFL, k0 = safeLoadAlt(1.0, "ngp_caster_grip_0", "ngp_caster_effect_grip_0", "ngp_caster_camber_grip_0")
    local gripFR, k1 = safeLoadAlt(1.0, "ngp_caster_grip_1", "ngp_caster_effect_grip_1", "ngp_caster_camber_grip_1")

    local toeFL, k2 = safeLoadAlt(0.0, "ngp_caster_total_toe_0", "ngp_caster_toe_0", "ngp_ca_toe_0")
    local toeFR, k3 = safeLoadAlt(0.0, "ngp_caster_total_toe_1", "ngp_caster_toe_1", "ngp_ca_toe_1")

    local camberFL, k4 = safeLoadAlt(0.0, "ngp_caster_total_camber_0", "ngp_caster_camber_0", "ngp_ca_camber_0")
    local camberFR, k5 = safeLoadAlt(0.0, "ngp_caster_total_camber_1", "ngp_caster_camber_1", "ngp_ca_camber_1")

    state.casterLinked = k0 ~= nil or k1 ~= nil or k2 ~= nil or k3 ~= nil or k4 ~= nil or k5 ~= nil

    state.casterGripFL = clamp(gripFL, 0.40, 1.10)
    state.casterGripFR = clamp(gripFR, 0.40, 1.10)
    state.casterGripAvg = (state.casterGripFL + state.casterGripFR) * 0.5
    state.casterGripLoss = clamp(1.0 - state.casterGripAvg, 0.0, 0.60)

    state.casterToeFL = toeFL
    state.casterToeFR = toeFR
    state.casterToeAbs = (math.abs(toeFL) + math.abs(toeFR)) * 0.5

    state.casterCamberFL = camberFL
    state.casterCamberFR = camberFR
    state.casterCamberAbs = (math.abs(camberFL) + math.abs(camberFR)) * 0.5

    local scaleTarget =
        1.0 +
        state.casterGripLoss * M.params.casterGripReactionGain +
        state.casterToeAbs * M.params.casterToeReactionGain +
        state.casterCamberAbs * M.params.casterCamberReactionGain

    state.casterScale = clamp(scaleTarget, M.params.minCasterScale, M.params.maxCasterScale)
end

local function readFrontAxisInput()
    local anchor, k0 = safeLoadAlt(0.0, "ngp_front_axis_anchor", "ngp_faa_anchor")
    local authority, k1 = safeLoadAlt(0.0, "ngp_front_axis_authority", "ngp_faa_authority")
    local yawResist, k2 = safeLoadAlt(0.0, "ngp_front_axis_yaw_resist", "ngp_faa_yaw_resist")
    local steerWeight, k3 = safeLoadAlt(0.0, "ngp_front_axis_steer_weight", "ngp_faa_steer_weight")
    local slipDamp, k4 = safeLoadAlt(0.0, "ngp_front_axis_slip_damp", "ngp_faa_slip_damp")

    state.frontAxisLinked = k0 ~= nil or k1 ~= nil or k2 ~= nil or k3 ~= nil or k4 ~= nil

    state.frontAxisAnchor = clamp(anchor, 0.0, 1.25)
    state.frontAxisAuthority = clamp(authority, 0.0, 1.25)
    state.frontAxisYawResist = clamp(yawResist, 0.0, 1.15)
    state.frontAxisSteerWeight = clamp(steerWeight, 0.0, 1.20)
    state.frontAxisSlipDamp = clamp(slipDamp, 0.0, 1.0)
end

local function readRoadAndPathInput()
    local shock, k0 = safeLoadAlt(0.0, "ngp_road_shock", "ngp_rii_avg_shock", "ngp_road_input_avg_impact")
    local surface, k1 = safeLoadAlt(0.0, "ngp_road_surface_limit", "ngp_rii_avg_surface_limit")
    local pathLoss, k2 = safeLoadAlt(0.0, "ngp_load_path_avg_loss", "ngp_lp_avg_loss", "ngp_road_path_loss")
    local delivery, k3 = safeLoadAlt(1.0, "ngp_load_path_front_delivery", "ngp_lp_front_delivery", "ngp_road_tire_delivery")

    state.roadLinked = k0 ~= nil or k1 ~= nil or k2 ~= nil or k3 ~= nil

    state.roadShock = clamp(shock, 0.0, 1.0)
    state.roadSurfaceLimit = clamp(surface, 0.0, 1.0)
    state.pathLoss = clamp(pathLoss, 0.0, 1.0)
    state.tireDelivery = clamp(delivery, 0.0, 1.2)
end

local function calculateEffectiveInput(steer)
    local p = M.params

    local frontAxisTerm =
        state.frontAxisSteerWeight *
        state.frontAxisAuthority *
        sign(steer) *
        p.frontAxisInputGain

    local effective =
        steer +
        state.bodySteer * p.bodySteerInputGain -
        state.flexDelay * p.flexDelayGain +
        state.flexYaw * p.flexYawGain +
        state.flexRoll * p.flexRollGain +
        frontAxisTerm

    effective = effective * (1.0 - state.frontAxisSlipDamp * 0.06)

    return clamp(effective, -p.maxEffectiveInput, p.maxEffectiveInput)
end

local function updateRack(steer, dt)
    local tauScale =
        (state.bodyTauScale or 1.0) *
        (1.0 + state.frontAxisSlipDamp * 0.06 + state.roadShock * 0.04)

    state.rack = lowPass(
        state.rack,
        steer,
        M.params.rackTau * tauScale,
        dt
    )
end

local function updateTieRod(fy, dt)
    state.tieRod = lowPass(state.tieRod, fy, M.params.tieRodTau, dt)
end

local function updateKnuckle(mz, fx, dt)
    local knuckleInput = mz + fx * M.params.fxKnuckleMix
    state.knuckle = lowPass(state.knuckle, knuckleInput, M.params.knuckleTau, dt)
end

local function calculateReaction()
    local mechanical =
        state.tieRod * M.params.tieRodGain +
        state.knuckle * M.params.knuckleGain

    local frontAxisReaction =
        state.frontAxisYawResist *
        state.frontAxisAuthority *
        sign(state.rack) *
        M.params.frontAxisReactionGain

    local roadReaction =
        state.roadShock *
        sign(state.rack) *
        M.params.roadShockReactionGain

    local reaction =
        (mechanical + frontAxisReaction + roadReaction) *
        (state.bodyReactionScale or 1.0) *
        (state.casterScale or 1.0) *
        clamp(0.84 + state.tireDelivery * 0.16 - state.pathLoss * M.params.pathLossReactionLoss, 0.75, 1.10)

    reaction =
        reaction +
        state.flexEnergy * M.params.flexEnergyReactionGain * sign(state.rack) -
        state.flexRelease * M.params.flexReleaseReturnGain * sign(state.rack)

    return clamp(reaction, -M.params.maxReaction, M.params.maxReaction)
end

local function calculateFriction()
    if math.abs(state.rack) <= M.params.minFrictionRack then
        return 0.0
    end

    local casterFriction =
        1.0 +
        state.casterGripLoss * M.params.casterGripFrictionGain

    local flexFriction =
        1.0 +
        clamp(state.flexEnergy, 0.0, 1.0) * 0.06

    local frontAxisFriction =
        1.0 +
        state.frontAxisSteerWeight * M.params.frontAxisFrictionGain +
        state.roadSurfaceLimit * 0.05

    return sign(state.rack) * M.params.friction * casterFriction * flexFriction * frontAxisFriction
end

local function calculateOutput()
    local output =
        state.rack +
        state.reaction -
        state.friction

    return clamp(output, -M.params.maxOutput, M.params.maxOutput)
end

local function decayToNeutral(dt)
    local tau = M.params.noCarDecayTau

    state.input = lowPass(state.input, 0.0, tau, dt)
    state.effectiveInput = lowPass(state.effectiveInput, 0.0, tau, dt)
    state.rack = lowPass(state.rack, 0.0, tau, dt)
    state.tieRod = lowPass(state.tieRod, 0.0, tau, dt)
    state.knuckle = lowPass(state.knuckle, 0.0, tau, dt)
    state.reaction = lowPass(state.reaction, 0.0, tau, dt)
    state.friction = lowPass(state.friction, 0.0, tau, dt)
    state.output = lowPass(state.output, 0.0, tau, dt)

    state.fy = 0.0
    state.mz = 0.0
    state.fx = 0.0

    state.bodySteer = lowPass(state.bodySteer, 0.0, tau, dt)
    state.flexDelay = lowPass(state.flexDelay, 0.0, tau, dt)
    state.flexYaw = lowPass(state.flexYaw, 0.0, tau, dt)
    state.flexRoll = lowPass(state.flexRoll, 0.0, tau, dt)
    state.flexPitch = lowPass(state.flexPitch, 0.0, tau, dt)
    state.flexEnergy = lowPass(state.flexEnergy, 0.0, tau, dt)
    state.flexRelease = lowPass(state.flexRelease, 0.0, tau, dt)
end

local function exportState()
    safeStore("ngp_steering_dynamics_status", state.status or "UNKNOWN")
    safeStore("ngp_steering_dynamics_update_count", state.updateCount or 0)
    safeStore("ngp_steering_dynamics_wheels_valid", state.wheelsValid and 1 or 0)

    safeStore("ngp_steer_mech", state.output or 0.0)
    safeStore("ngp_steer_reaction", state.reaction or 0.0)
    safeStore("ngp_steer_rack", state.rack or 0.0)
    safeStore("ngp_steer_input", state.input or 0.0)
    safeStore("ngp_steer_effective_input", state.effectiveInput or 0.0)

    -- Short aliases.
    safeStore("ngp_sd_output", state.output or 0.0)
    safeStore("ngp_sd_reaction", state.reaction or 0.0)
    safeStore("ngp_sd_rack", state.rack or 0.0)
    safeStore("ngp_sd_input", state.input or 0.0)
    safeStore("ngp_sd_effective_input", state.effectiveInput or 0.0)
    safeStore("ngp_sd_friction", state.friction or 0.0)

    safeStore("ngp_steer_front_axis_anchor", state.frontAxisAnchor or 0.0)
    safeStore("ngp_steer_front_axis_authority", state.frontAxisAuthority or 0.0)
    safeStore("ngp_steer_front_axis_yaw_resist", state.frontAxisYawResist or 0.0)
    safeStore("ngp_steer_front_axis_slip_damp", state.frontAxisSlipDamp or 0.0)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_steer_tierod", state.tieRod or 0.0)
    safeStore("ngp_steer_knuckle", state.knuckle or 0.0)
    safeStore("ngp_steer_friction", state.friction or 0.0)
    safeStore("ngp_steer_fx", state.fx or 0.0)
    safeStore("ngp_steer_fy", state.fy or 0.0)
    safeStore("ngp_steer_mz", state.mz or 0.0)
    safeStore("ngp_steer_speed", state.speedKmh or 0.0)

    safeStore("ngp_steer_body_linked", state.bodyLinked and 1 or 0)
    safeStore("ngp_steer_flex_linked", state.flexLinked and 1 or 0)
    safeStore("ngp_steer_caster_linked", state.casterLinked and 1 or 0)
    safeStore("ngp_steer_front_axis_linked", state.frontAxisLinked and 1 or 0)
    safeStore("ngp_steer_road_linked", state.roadLinked and 1 or 0)
    safeStore("ngp_steer_force_linked", state.forceLinked and 1 or 0)

    safeStore("ngp_steer_body_rigidity", state.bodyRigidity or 1.0)
    safeStore("ngp_steer_body_flex_factor", state.bodyFlexFactor or 1.0)
    safeStore("ngp_steer_body_tau_scale", state.bodyTauScale or 1.0)
    safeStore("ngp_steer_body_reaction_scale", state.bodyReactionScale or 1.0)
    safeStore("ngp_steer_body_steer", state.bodySteer or 0.0)
    safeStore("ngp_steer_flex_delay", state.flexDelay or 0.0)
    safeStore("ngp_steer_flex_yaw", state.flexYaw or 0.0)
    safeStore("ngp_steer_flex_energy", state.flexEnergy or 0.0)
    safeStore("ngp_steer_flex_release", state.flexRelease or 0.0)

    safeStore("ngp_steer_caster_grip_avg", state.casterGripAvg or 1.0)
    safeStore("ngp_steer_caster_grip_loss", state.casterGripLoss or 0.0)
    safeStore("ngp_steer_caster_scale", state.casterScale or 1.0)

    safeStore("ngp_steer_road_shock", state.roadShock or 0.0)
    safeStore("ngp_steer_path_loss", state.pathLoss or 0.0)
    safeStore("ngp_steer_tire_delivery", state.tireDelivery or 1.0)
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
        exportState()
        return
    end

    updateDebugGate(dt)

    car = car or safeGetCar()

    if not car then
        state.status = "NO CAR"
        state.wheelsValid = false
        state.frontWheelsValid = false
        state.speedKmh = 0.0
        decayToNeutral(dt)
        exportState()
        return
    end

    state.speedKmh = getSpeedKmh(car)
    state.input = getSteer(car)

    local wheels = getWheels(car)
    if not wheels then
        state.status = "NO WHEELS"
        state.wheelsValid = false
        state.frontWheelsValid = false
        decayToNeutral(dt)
        exportState()
        return
    end

    state.wheelsValid = true

    readChassisFlex()
    readBodyRigidity(dt)
    updateBodyScales(dt)
    readCasterInput()
    readFrontAxisInput()
    readRoadAndPathInput()

    local fy, mz, fx = readFrontForces(car)
    state.fy = fy
    state.mz = mz
    state.fx = fx

    if not state.frontWheelsValid then
        state.status = "NO FRONT WHEELS"
    else
        state.status = "RUNNING"
    end

    state.effectiveInput = calculateEffectiveInput(state.input)

    updateRack(state.effectiveInput, dt)
    updateTieRod(fy, dt)
    updateKnuckle(mz, fx, dt)

    state.reaction = calculateReaction()
    state.friction = calculateFriction()
    state.output = calculateOutput()

    exportState()
end

function M.getOutput()
    return state.output or 0.0
end

function M.getReaction()
    return state.reaction or 0.0
end

function M.getRack()
    return state.rack or 0.0
end

function M.getTieRod()
    return state.tieRod or 0.0
end

function M.getKnuckle()
    return state.knuckle or 0.0
end

function M.getEffectiveInput()
    return state.effectiveInput or 0.0
end

function M.getFrontAxisAuthority()
    return state.frontAxisAuthority or 0.0
end

function M.getState()
    return state
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f / Wheels %s / Front %s\n" ..
        "Input %.3f Eff %.3f Rack %.3f Out %.3f\n" ..
        "Tie %.2f Knuckle %.2f React %.3f Fric %.3f\n" ..
        "Body:%s Flex:%s Cast:%s FAA:%s Road:%s Force:%s\n" ..
        "Tau %.2f ReactScale %.2f Cast %.2f FAA %.2f Shock %.2f",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",
        state.frontWheelsValid and "OK" or "NIL",

        state.input or 0.0,
        state.effectiveInput or 0.0,
        state.rack or 0.0,
        state.output or 0.0,

        state.tieRod or 0.0,
        state.knuckle or 0.0,
        state.reaction or 0.0,
        state.friction or 0.0,

        state.bodyLinked and "OK" or "NIL",
        state.flexLinked and "OK" or "NIL",
        state.casterLinked and "OK" or "NIL",
        state.frontAxisLinked and "OK" or "NIL",
        state.roadLinked and "OK" or "NIL",
        state.forceLinked and "OK" or "NIL",

        state.bodyTauScale or 1.0,
        state.bodyReactionScale or 1.0,
        state.casterScale or 1.0,
        state.frontAxisAuthority or 0.0,
        state.roadShock or 0.0
    )
end

return M
