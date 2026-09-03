---@diagnostic disable: undefined-global

--============================================================
-- ACNextGen
-- steering_mechanism.lua
-- Phase 8.5 / V1.1.5
-- Steering Knuckle / Diagnostic FFB Assist Signal
--============================================================

local M = {}

M.params = {
    mzScale = 0.00065,
    fxScale = 0.00020,
    fyScale = 0.00020,

    bumpScale = 0.25,
    dampingScale = 0.30,

    maxSpeedFactor = 1.80,
    maxFFB = 2.50,
    minFFB = -2.50,
    minSpeed = 3.0,

    knuckleTau = 0.050,
    ffbTau = 0.045,
    decayTau = 0.160,

    mzLimit = 1.20,
    fxLimit = 0.80,
    fyLimit = 1.00,
    bumpLimit = 0.50,
    dampLimit = 0.80,

    frontAxisReactionGain = 0.18,
    frontAxisFrictionGain = 0.10,
    steerDynamicsGain = 0.20,

    roadShockGain = 0.10,
    roadSurfaceLossGain = 0.08,
    loadPathLossGain = 0.08,
    tireDeliveryGain = 0.10,

    casterReactionGain = 0.08,
    bodyFlexGain = 0.06,

    minDt = 0.00005,
    maxDt = 0.050,

    -- Keep disabled by default. This module exports diagnostic/shared values first.
    applyFFB = false,

    debugStoreInterval = 0.25,
}

local state = {
    prevSteer = 0.0,
    prevSusp = { [0] = 0.0, [1] = 0.0 },

    knuckle = { fx = 0.0, fy = 0.0, mz = 0.0 },

    mzForce = 0.0,
    fxForce = 0.0,
    fyForce = 0.0,
    bumpForce = 0.0,
    dampForce = 0.0,

    finalFFB = 0.0,
    rawFFB = 0.0,

    knuckleFX = 0.0,
    knuckleFY = 0.0,
    knuckleMZ = 0.0,

    speedKmh = 0.0,
    speedFactor = 0.0,
    steer = 0.0,

    frontAxisAnchor = 0.0,
    frontAxisAuthority = 0.0,
    frontAxisYawResist = 0.0,
    frontAxisSteerWeight = 0.0,
    frontAxisSlipDamp = 0.0,

    steerDynamicsMech = 0.0,
    steerDynamicsReaction = 0.0,
    steerDynamicsRack = 0.0,

    roadShock = 0.0,
    roadSurfaceLimit = 0.0,
    loadPathLoss = 0.0,
    tireDelivery = 1.0,

    casterScale = 1.0,
    casterGripLoss = 0.0,
    bodyFlex = 0.0,

    active = false,
    ffbApplied = false,
    initialized = false,

    wheelsValid = false,
    carLinked = false,
    forceLinked = false,
    suspensionLinked = false,
    steeringDynamicsLinked = false,
    frontAxisLinked = false,
    roadLinked = false,
    loadPathLinked = false,
    casterLinked = false,
    bodyLinked = false,

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
    v = safeNumber(v, minValue or 0.0)
    if v < minValue then return minValue end
    if v > maxValue then return maxValue end
    return v
end

local function abs(v)
    return math.abs(safeNumber(v, 0.0))
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

    if tau <= 0.0001 then
        return target
    end

    local k = clamp(dt / math.max(tau + dt, 0.0001), 0.0, 1.0)
    return current + (target - current) * k
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

local function safeSetSteeringFFB(value)
    if not ac or not ac.setSteeringFFB then
        return false
    end

    local ok = pcall(function()
        ac.setSteeringFFB(value)
    end)

    return ok and true or false
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
    if not car then
        return nil
    end

    return safeField(car, "wheels", nil)
end

local function getWheel(car, zeroIndex)
    local wheels = getWheels(car)
    if not wheels then
        return nil
    end

    local ok0, wheel0 = pcall(function()
        return wheels[zeroIndex]
    end)
    if ok0 and wheel0 then
        return wheel0
    end

    local ok1, wheel1 = pcall(function()
        return wheels[zeroIndex + 1]
    end)
    if ok1 and wheel1 then
        return wheel1
    end

    return nil
end

local function readWheelValue(wheel, ...)
    if not wheel then
        return nil
    end

    local fields = { ... }
    for i = 1, #fields do
        local value = safeField(wheel, fields[i], nil)
        if value ~= nil then
            return safeNumber(value, 0.0), fields[i]
        end
    end

    return nil, nil
end

local function readForceFromStores(index, axis)
    if axis == "fx" then
        return safeLoadAlt(nil,
            "ngp_tire_long_" .. index,
            "ngp_long_" .. index,
            "ngp_cp_long_" .. index,
            "ngp_hub_force_long_" .. index,
            "ngp_force_long_" .. index,
            "ngp_wheel_fx_" .. index
        )
    elseif axis == "fy" then
        return safeLoadAlt(nil,
            "ngp_tire_lat_" .. index,
            "ngp_lat_" .. index,
            "ngp_cp_lat_" .. index,
            "ngp_hub_force_lat_" .. index,
            "ngp_force_lat_" .. index,
            "ngp_wheel_fy_" .. index
        )
    else
        return safeLoadAlt(nil,
            "ngp_tire_mz_" .. index,
            "ngp_aligning_moment_" .. index,
            "ngp_caster_aligning_hint_" .. index,
            "ngp_steer_mz_" .. index,
            "ngp_wheel_mz_" .. index
        )
    end
end

local function readWheelForce(index, wheel, axis)
    local value, field = nil, nil

    if axis == "fx" then
        value, field = readWheelValue(wheel, "fx", "longitudinalForce", "longitudinalForceN")
    elseif axis == "fy" then
        value, field = readWheelValue(wheel, "fy", "lateralForce", "lateralForceN")
    else
        value, field = readWheelValue(wheel, "mz", "aligningMoment", "selfAligningTorque", "aligningTorque")
    end

    if value ~= nil then
        state.forceLinked = true
        return value
    end

    local storeValue, key = readForceFromStores(index, axis)
    if key ~= nil then
        state.forceLinked = true
        return storeValue
    end

    return 0.0
end

local function readFrontWheels(car)
    local frontLeft = getWheel(car, 0)
    local frontRight = getWheel(car, 1)
    return frontLeft, frontRight
end

local function readWheelForces(frontLeft, frontRight)
    local fx =
        (readWheelForce(0, frontLeft, "fx") + readWheelForce(1, frontRight, "fx")) * 0.5

    local fy =
        (readWheelForce(0, frontLeft, "fy") + readWheelForce(1, frontRight, "fy")) * 0.5

    local mz =
        (readWheelForce(0, frontLeft, "mz") + readWheelForce(1, frontRight, "mz")) * 0.5

    return fx, fy, mz
end

local function readSuspensionTravel(index, wheel, fallback)
    local value = nil

    if wheel then
        value = select(1, readWheelValue(
            wheel,
            "suspensionTravel",
            "suspensionTravelM",
            "suspensionLength",
            "travel",
            "damperTravel",
            "suspensionPosition"
        ))
    end

    if value ~= nil then
        state.suspensionLinked = true
        return clamp(value, -2.0, 2.0)
    end

    local storeValue, key = safeLoadAlt(nil,
        "ngp_spring_travel_" .. index,
        "ngp_spring_effective_travel_" .. index,
        "ngp_damper_hyst_travel_" .. index,
        "ngp_damper_travel_" .. index,
        "ngp_susp_travel_" .. index
    )

    if key ~= nil then
        state.suspensionLinked = true
        return clamp(storeValue, -2.0, 2.0)
    end

    return fallback or 0.0
end

local function readSpeedKmh(car)
    local speed = safeField(car, "speedKmh", nil)
    if speed ~= nil then
        return safeNumber(speed, 0.0)
    end

    speed = nil
    if speed ~= nil then
        local n = safeNumber(speed, 0.0)
        if n < 90.0 then
            return n * 3.6
        end
        return n
    end

    return safeLoad("ngp_hub_speed", 0.0)
end

local function readSteer(car)
    local steer = safeField(car, "steer", nil)
    if steer ~= nil then
        return clamp(safeNumber(steer, 0.0), -1.5, 1.5)
    end

    return clamp(safeLoad("ngp_steer_input", 0.0), -1.5, 1.5)
end

local function readSharedInputs()
    local v, key

    v, key = safeLoadAlt(0.0, "ngp_front_axis_anchor", "ngp_mass_front_axis_anchor")
    state.frontAxisAnchor = clamp(v, 0.0, 1.5)

    v, key = safeLoadAlt(0.0, "ngp_front_axis_authority", "ngp_steer_front_authority")
    state.frontAxisAuthority = clamp(v, 0.0, 1.5)
    state.frontAxisLinked = key ~= nil or safeLoadRaw("ngp_front_axis_yaw_resist") ~= nil

    state.frontAxisYawResist = clamp(safeLoad("ngp_front_axis_yaw_resist", 0.0), 0.0, 1.5)
    state.frontAxisSteerWeight = clamp(safeLoad("ngp_front_axis_steer_weight", 0.0), 0.0, 1.5)
    state.frontAxisSlipDamp = clamp(safeLoad("ngp_front_axis_slip_damp", 0.0), 0.0, 1.5)

    v, key = safeLoadAlt(0.0, "ngp_steer_mech", "ngp_sd_output")
    state.steerDynamicsMech = clamp(v, -2.5, 2.5)

    v, key = safeLoadAlt(0.0, "ngp_steer_reaction", "ngp_sd_reaction")
    state.steerDynamicsReaction = clamp(v, -2.5, 2.5)

    v, key = safeLoadAlt(0.0, "ngp_steer_rack", "ngp_sd_rack")
    state.steerDynamicsRack = clamp(v, -2.5, 2.5)
    state.steeringDynamicsLinked =
        safeLoadRaw("ngp_steering_dynamics_update_count") ~= nil
        or safeLoadRaw("ngp_steer_mech") ~= nil

    state.roadShock = clamp(safeLoad("ngp_road_shock", safeLoad("ngp_road_input_avg_impact", 0.0)), 0.0, 1.5)
    state.roadSurfaceLimit = clamp(safeLoad("ngp_road_surface_limit", 0.0), 0.0, 1.5)
    state.roadLinked =
        safeLoadRaw("ngp_road_input_update_count") ~= nil
        or safeLoadRaw("ngp_road_shock") ~= nil

    state.loadPathLoss = clamp(safeLoad("ngp_load_path_avg_loss", safeLoad("ngp_road_path_loss", 0.0)), 0.0, 1.5)
    state.tireDelivery = clamp(safeLoad("ngp_load_path_avg_tire_delivery", safeLoad("ngp_road_tire_delivery", 1.0)), 0.0, 1.2)
    state.loadPathLinked =
        safeLoadRaw("ngp_load_path_update_count") ~= nil
        or safeLoadRaw("ngp_load_path_avg_loss") ~= nil

    state.casterScale = clamp(safeLoad("ngp_steer_caster_scale", 1.0), 0.75, 1.30)
    state.casterGripLoss = clamp(safeLoad("ngp_steer_caster_grip_loss", 0.0), 0.0, 1.0)
    state.casterLinked =
        safeLoadRaw("ngp_caster_grip_0") ~= nil
        or safeLoadRaw("ngp_steer_caster_scale") ~= nil

    state.bodyFlex = clamp(safeLoad("ngp_body_flex_factor", safeLoad("ngp_steer_body_flex_factor", 1.0)) - 1.0, 0.0, 1.0)
    state.bodyLinked =
        safeLoadRaw("ngp_body_rigidity_update_count") ~= nil
        or safeLoadRaw("ngp_steer_body_flex_factor") ~= nil
end

local function updateKnuckleFilter(fx, fy, mz, dt)
    state.knuckle.fx = lowPass(state.knuckle.fx, fx, M.params.knuckleTau, dt)
    state.knuckle.fy = lowPass(state.knuckle.fy, fy, M.params.knuckleTau, dt)
    state.knuckle.mz = lowPass(state.knuckle.mz, mz, M.params.knuckleTau, dt)
end

local function calculateForceComponents(steer)
    local authorityScale = 1.0 + state.frontAxisAuthority * M.params.frontAxisReactionGain
    local casterScale = 1.0 + state.casterGripLoss * M.params.casterReactionGain
    local deliveryScale = 1.0 - clamp((1.0 - state.tireDelivery) * M.params.tireDeliveryGain, 0.0, 0.25)

    local mzForce = clamp(
        state.knuckle.mz * M.params.mzScale * authorityScale * casterScale * deliveryScale,
        -M.params.mzLimit,
        M.params.mzLimit
    )

    local fxForce = clamp(
        state.knuckle.fx * M.params.fxScale * abs(steer + 0.01),
        -M.params.fxLimit,
        M.params.fxLimit
    )

    local fyForce = clamp(
        -state.knuckle.fy * M.params.fyScale * authorityScale * deliveryScale,
        -M.params.fyLimit,
        M.params.fyLimit
    )

    return mzForce, fxForce, fyForce
end

local function calculateBumpForce(frontLeft, frontRight, dt)
    local suspFL = readSuspensionTravel(0, frontLeft, state.prevSusp[0])
    local suspFR = readSuspensionTravel(1, frontRight, state.prevSusp[1])

    if not state.initialized then
        state.prevSusp[0] = suspFL
        state.prevSusp[1] = suspFR
        return 0.0
    end

    local velFL = (suspFL - (state.prevSusp[0] or suspFL)) / math.max(dt, 0.001)
    local velFR = (suspFR - (state.prevSusp[1] or suspFR)) / math.max(dt, 0.001)

    state.prevSusp[0] = suspFL
    state.prevSusp[1] = suspFR

    local bumpForce =
        (velFL + velFR) * 0.5
        * M.params.bumpScale
        * 0.1

    bumpForce = bumpForce + state.roadShock * M.params.roadShockGain
    bumpForce = bumpForce - state.roadSurfaceLimit * M.params.roadSurfaceLossGain * sign(bumpForce)

    return clamp(bumpForce, -M.params.bumpLimit, M.params.bumpLimit)
end

local function calculateSteerDamping(steer, dt)
    if not state.initialized then
        state.prevSteer = steer
        return 0.0
    end

    local steerVelocity = (steer - (state.prevSteer or steer)) / math.max(dt, 0.001)
    state.prevSteer = steer

    local dampScale =
        1.0
        + state.frontAxisSlipDamp * M.params.frontAxisFrictionGain
        + state.loadPathLoss * M.params.loadPathLossGain
        + state.bodyFlex * M.params.bodyFlexGain

    local dampForce =
        -steerVelocity
        * M.params.dampingScale
        * dampScale

    return clamp(dampForce, -M.params.dampLimit, M.params.dampLimit)
end

local function calculateSpeedFactor(speedKmh)
    if speedKmh < M.params.minSpeed then
        return 0.0
    end

    local speedMs = speedKmh / 3.6
    return clamp(speedMs * 0.03, 0.0, M.params.maxSpeedFactor)
end

local function calculateFinalFFB(mzForce, fxForce, fyForce, bumpForce, dampForce, speedFactor)
    local steerDynamicsPart =
        (state.steerDynamicsReaction + state.steerDynamicsMech * 0.35)
        * M.params.steerDynamicsGain

    local axisPart =
        state.frontAxisYawResist * sign(state.steer)
        * M.params.frontAxisReactionGain

    local raw =
        (mzForce + fxForce + fyForce + bumpForce + dampForce + steerDynamicsPart + axisPart)
        * speedFactor

    return clamp(raw, M.params.minFFB, M.params.maxFFB)
end

local function decayState(dt, status)
    state.status = status or "DECAY"
    state.active = false
    state.wheelsValid = false

    state.knuckle.fx = lowPass(state.knuckle.fx, 0.0, M.params.decayTau, dt)
    state.knuckle.fy = lowPass(state.knuckle.fy, 0.0, M.params.decayTau, dt)
    state.knuckle.mz = lowPass(state.knuckle.mz, 0.0, M.params.decayTau, dt)

    state.mzForce = lowPass(state.mzForce, 0.0, M.params.decayTau, dt)
    state.fxForce = lowPass(state.fxForce, 0.0, M.params.decayTau, dt)
    state.fyForce = lowPass(state.fyForce, 0.0, M.params.decayTau, dt)
    state.bumpForce = lowPass(state.bumpForce, 0.0, M.params.decayTau, dt)
    state.dampForce = lowPass(state.dampForce, 0.0, M.params.decayTau, dt)
    state.rawFFB = lowPass(state.rawFFB, 0.0, M.params.decayTau, dt)
    state.finalFFB = lowPass(state.finalFFB, 0.0, M.params.decayTau, dt)

    state.knuckleFX = state.knuckle.fx
    state.knuckleFY = state.knuckle.fy
    state.knuckleMZ = state.knuckle.mz

    if M.params.applyFFB then
        state.ffbApplied = safeSetSteeringFFB(0.0)
    else
        state.ffbApplied = false
    end
end

local function exportState()
    safeStore("ngp_steering_mechanism_status", state.status or "UNKNOWN")
    safeStore("ngp_steering_mechanism_update_count", state.updateCount or 0)

    safeStore("ngp_steering_ffb", state.finalFFB or 0.0)
    safeStore("ngp_steering_mz_force", state.mzForce or 0.0)
    safeStore("ngp_steering_fx_force", state.fxForce or 0.0)
    safeStore("ngp_steering_fy_force", state.fyForce or 0.0)
    safeStore("ngp_steering_bump_force", state.bumpForce or 0.0)
    safeStore("ngp_steering_damp_force", state.dampForce or 0.0)

    safeStore("ngp_steering_knuckle_fx", state.knuckleFX or 0.0)
    safeStore("ngp_steering_knuckle_fy", state.knuckleFY or 0.0)
    safeStore("ngp_steering_knuckle_mz", state.knuckleMZ or 0.0)

    safeStore("ngp_steering_speed_factor", state.speedFactor or 0.0)
    safeStore("ngp_steering_speed", state.speedKmh or 0.0)
    safeStore("ngp_steering_input", state.steer or 0.0)
    safeStore("ngp_steering_active", state.active and 1 or 0)
    safeStore("ngp_steering_ffb_applied", state.ffbApplied and 1 or 0)

    -- Short aliases for V1.1 modules.
    safeStore("ngp_sm_ffb", state.finalFFB or 0.0)
    safeStore("ngp_sm_reaction", (state.mzForce or 0.0) + (state.fyForce or 0.0))
    safeStore("ngp_sm_bump", state.bumpForce or 0.0)
    safeStore("ngp_sm_damp", state.dampForce or 0.0)
    safeStore("ngp_sm_knuckle_mz", state.knuckleMZ or 0.0)
    safeStore("ngp_sm_force_linked", state.forceLinked and 1 or 0)
    safeStore("ngp_sm_wheels_valid", state.wheelsValid and 1 or 0)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_steering_raw_ffb", state.rawFFB or 0.0)
    safeStore("ngp_steering_front_axis_linked", state.frontAxisLinked and 1 or 0)
    safeStore("ngp_steering_dynamics_linked", state.steeringDynamicsLinked and 1 or 0)
    safeStore("ngp_steering_road_linked", state.roadLinked and 1 or 0)
    safeStore("ngp_steering_load_path_linked", state.loadPathLinked and 1 or 0)
    safeStore("ngp_steering_caster_linked", state.casterLinked and 1 or 0)
    safeStore("ngp_steering_body_linked", state.bodyLinked and 1 or 0)
    safeStore("ngp_steering_suspension_linked", state.suspensionLinked and 1 or 0)

    safeStore("ngp_steering_front_axis_authority", state.frontAxisAuthority or 0.0)
    safeStore("ngp_steering_front_axis_yaw_resist", state.frontAxisYawResist or 0.0)
    safeStore("ngp_steering_road_shock", state.roadShock or 0.0)
    safeStore("ngp_steering_load_path_loss", state.loadPathLoss or 0.0)
    safeStore("ngp_steering_tire_delivery", state.tireDelivery or 1.0)
    safeStore("ngp_steering_caster_grip_loss", state.casterGripLoss or 0.0)
end

function M.init()
    state.status = "INIT"
    state.initialized = false
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = clamp(safeNumber(dt, 0.0), M.params.minDt, M.params.maxDt)
    updateDebugGate(dt)

    state.carLinked = false
    state.forceLinked = false
    state.suspensionLinked = false
    state.steeringDynamicsLinked = false
    state.frontAxisLinked = false
    state.roadLinked = false
    state.loadPathLinked = false
    state.casterLinked = false
    state.bodyLinked = false

    car = car or safeGetCar()

    readSharedInputs()

    if not car then
        state.speedKmh = 0.0
        state.speedFactor = 0.0
        decayState(dt, "NO CAR")
        exportState()
        return
    end

    state.carLinked = true
    state.speedKmh = readSpeedKmh(car)
    state.steer = readSteer(car)

    local frontLeft, frontRight = readFrontWheels(car)

    if state.speedKmh < M.params.minSpeed then
        state.speedFactor = 0.0
        state.prevSteer = state.steer
        decayState(dt, "LOW SPEED")
        exportState()
        return
    end

    if not frontLeft or not frontRight then
        decayState(dt, "NO FRONT WHEELS")
        exportState()
        return
    end

    state.status = "RUNNING"
    state.wheelsValid = true
    state.active = true

    local fx, fy, mz = readWheelForces(frontLeft, frontRight)

    updateKnuckleFilter(fx, fy, mz, dt)

    local mzForce, fxForce, fyForce = calculateForceComponents(state.steer)
    local bumpForce = calculateBumpForce(frontLeft, frontRight, dt)
    local dampForce = calculateSteerDamping(state.steer, dt)
    local speedFactor = calculateSpeedFactor(state.speedKmh)

    local rawFFB = calculateFinalFFB(mzForce, fxForce, fyForce, bumpForce, dampForce, speedFactor)

    state.mzForce = mzForce
    state.fxForce = fxForce
    state.fyForce = fyForce
    state.bumpForce = bumpForce
    state.dampForce = dampForce
    state.rawFFB = rawFFB
    state.finalFFB = lowPass(state.finalFFB, rawFFB, M.params.ffbTau, dt)

    state.knuckleFX = state.knuckle.fx
    state.knuckleFY = state.knuckle.fy
    state.knuckleMZ = state.knuckle.mz
    state.speedFactor = speedFactor

    state.initialized = true

    if M.params.applyFFB then
        state.ffbApplied = safeSetSteeringFFB(state.finalFFB)
    else
        state.ffbApplied = false
    end

    exportState()
end

function M.getFFB()
    return state.finalFFB or 0.0
end

function M.getMZForce()
    return state.mzForce or 0.0
end

function M.getFXForce()
    return state.fxForce or 0.0
end

function M.getFYForce()
    return state.fyForce or 0.0
end

function M.getBumpForce()
    return state.bumpForce or 0.0
end

function M.getDampForce()
    return state.dampForce or 0.0
end

function M.getKnuckle()
    return state.knuckle
end

function M.getState()
    return state
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "FFB %.3f Raw %.3f Active:%s Applied:%s\n" ..
        "MZ %.3f FX %.3f FY %.3f\n" ..
        "Bump %.3f Damp %.3f\n" ..
        "Speed %.1f Factor %.3f Steer %.3f\n" ..
        "Links Force:%s Susp:%s SD:%s FA:%s Road:%s LP:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",

        state.finalFFB or 0.0,
        state.rawFFB or 0.0,
        state.active and "YES" or "NO",
        state.ffbApplied and "YES" or "NO",

        state.mzForce or 0.0,
        state.fxForce or 0.0,
        state.fyForce or 0.0,

        state.bumpForce or 0.0,
        state.dampForce or 0.0,

        state.speedKmh or 0.0,
        state.speedFactor or 0.0,
        state.steer or 0.0,

        state.forceLinked and "OK" or "NIL",
        state.suspensionLinked and "OK" or "NIL",
        state.steeringDynamicsLinked and "OK" or "NIL",
        state.frontAxisLinked and "OK" or "NIL",
        state.roadLinked and "OK" or "NIL",
        state.loadPathLinked and "OK" or "NIL"
    )
end

return M
