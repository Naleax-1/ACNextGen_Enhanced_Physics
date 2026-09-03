---@diagnostic disable: undefined-global

--============================================================
-- compliance_stack.lua
-- ACNextGen V1.1.5 Stable
-- Compliance Stack / Root Signal Bridge
--============================================================

local M = {}

local FL, FR, RL, RR = 0, 1, 2, 3

local PHASE_TEXT = {
    [0]  = "INIT",
    [1]  = "STABLE",
    [2]  = "STEER_FLEX",
    [3]  = "DRIVE_FLEX",
    [4]  = "BRAKE_FLEX",
    [5]  = "LOAD_FLEX",
    [6]  = "RECOVERING",
    [7]  = "IMPACT",
    [8]  = "UNLOADED",
    [9]  = "AXLE_WINDUP",
    [10] = "BODY_FLEX",
}

M.params = {
    flexTau = 0.090,
    bushingTau = 0.115,
    knuckleTau = 0.075,
    subframeTau = 0.140,
    bodyTau = 0.180,

    energyRiseTau = 0.060,
    energyFallTau = 0.220,
    returnTau = 0.150,

    loadRef = 3200.0,
    loadFlexStart = 0.70,
    loadFlexFull = 1.70,
    loadHarshStart = 1.55,
    loadHarshFull = 2.80,

    contactLossGain = 0.42,
    contactTrustGain = 0.22,
    memoryGain = 0.30,
    historyGain = 0.18,
    damperGain = 0.22,
    impactGain = 0.55,

    lateralFlexScaleFront = 0.030,
    lateralFlexScaleRear = 0.038,
    longitudinalFlexScaleFront = 0.020,
    longitudinalFlexScaleRear = 0.032,

    steerFlexGain = 0.36,
    yawFlexGain = 0.18,
    slipAngleFlexGain = 1.15,
    slipRatioFlexGain = 0.85,

    brakeFlexGain = 0.36,
    driveFlexGain = 0.28,
    handbrakeRearGain = 0.45,

    armCamberGain = 0.35,
    armToeGain = 0.45,

    bushingLatGain = 0.62,
    bushingLongGain = 0.54,
    bushingContactLossGain = 0.30,

    knuckleSlipGain = 0.46,
    knuckleLoadGain = 0.28,
    knuckleImpactGain = 0.38,

    subframeYawGain = 0.34,
    subframeDriveGain = 0.32,
    subframeBrakeGain = 0.24,
    subframeDamperGain = 0.20,

    bodyLoadGain = 0.30,
    bodyYawGain = 0.24,
    bodyHystGain = 0.22,

    rearAxleCoupling = 0.34,
    rearAxleToeCoupling = 0.18,
    rearAxleCamberCoupling = 0.08,

    slipRecoveryGain = 0.20,
    dirtyReturnGain = 0.24,
    biteGain = 0.22,
    snapRiskGain = 0.18,

    yawBudgetGain = 0.16,
    yawBalanceGain = 0.12,
    yawBiteGain = 0.18,

    drivelineRearPushGain = 0.22,
    drivelineTwistGain = 0.18,

    toeFromLatFlexFront = 0.95,
    toeFromLatFlexRear = 1.25,
    toeFromLongFlex = 0.38,

    camberFromLatFlexFront = 0.78,
    camberFromLatFlexRear = 0.62,
    camberFromLoadFlex = 0.035,

    maxLatFlex = 0.080,
    maxLongFlex = 0.075,
    maxBushing = 1.0,
    maxKnuckle = 1.0,
    maxSubframe = 1.0,
    maxBody = 1.0,
    maxVirtualToe = 0.090,
    maxVirtualCamber = 0.120,

    steerDelayGain = 1.00,
    forceLeakGain = 0.68,
    loadPathLossGain = 0.55,

    unloadedQuality = 0.18,
    impactThreshold = 0.55,
    energyActive = 0.22,
    axleWindupThreshold = 0.26,
    bodyFlexThreshold = 0.34,

    inputReadInterval = 0.050,
    debugStoreInterval = 0.250,
}

local function newWheelState()
    return {
        load = 0.0,
        loadFactor = 0.0,

        contactQuality = 1.0,
        contactTrust = 1.0,
        contactLoss = 0.0,
        contactStatus = 0,

        tireMemory = 0.0,
        tirePhaseId = 0.0,
        tireHistory = 0.0,
        tireThermal = 0.0,
        tireAbrasion = 0.0,
        tireRecoveryScalar = 1.0,
        tireRecontact = 0.0,

        slipRecovery = 0.0,
        gripReturn = 1.0,
        snapRisk = 0.0,
        bite = 0.0,
        dirtyReturn = 0.0,

        slipAngle = 0.0,
        slipRatio = 0.0,
        combinedSlip = 0.0,

        damperHyst = 0.0,
        damperImpact = 0.0,
        damperVelocity = 0.0,
        damperVertical = 0.0,
        damperHysteretic = 0.0,
        damperPulse = 0.0,
        roadMemory = 0.0,

        carcassSupport = 1.0,
        carcassGripGate = 1.0,
        carcassHysteresis = 0.0,
        carcassRecoveryBias = 0.0,

        armCamber = 0.0,
        armToe = 0.0,

        bushingLat = 0.0,
        bushingLong = 0.0,
        bushingEnergy = 0.0,

        knuckleDeflection = 0.0,
        subframeDeflection = 0.0,
        bodyDeflection = 0.0,

        latFlex = 0.0,
        longFlex = 0.0,
        energy = 0.0,
        memory = 0.0,

        returnBias = 0.0,
        forceLeak = 0.0,
        loadPathLoss = 0.0,
        steerDelay = 0.0,

        virtualToe = 0.0,
        virtualCamber = 0.0,

        targetLatFlex = 0.0,
        targetLongFlex = 0.0,
        targetEnergy = 0.0,

        phaseId = 0,
        phaseText = "INIT",
        active = false,
    }
end

local state = {
    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    contactLinked = false,
    memoryLinked = false,
    damperLinked = false,
    armLinked = false,
    loadLinked = false,
    carcassLinked = false,
    slipRecoveryLinked = false,
    yawLinked = false,
    drivelineLinked = false,

    speedKmh = 0.0,
    steer = 0.0,
    gas = 0.0,
    brake = 0.0,
    handbrake = 0.0,
    yawRate = 0.0,

    yawBudget = 0.0,
    yawBalance = 0.0,
    yawBite = 0.0,
    yawDirty = 0.0,

    drivelineTwist = 0.0,
    drivelineRearPush = 0.0,

    avgEnergy = 0.0,
    avgLatFlex = 0.0,
    avgLongFlex = 0.0,
    avgToe = 0.0,
    avgCamber = 0.0,
    avgBushing = 0.0,
    avgKnuckle = 0.0,
    avgSubframe = 0.0,
    avgBody = 0.0,
    avgForceLeak = 0.0,

    frontSteerDelay = 0.0,
    rearToeCompliance = 0.0,
    chassisCompliance = 0.0,
    bodyFlex = 0.0,
    subframeTwist = 0.0,
    rearAxleWindup = 0.0,
    loadPathLoss = 0.0,
    forceLeak = 0.0,

    inputTimer = 999.0,
    debugStoreTimer = 999.0,
    debugStoreNow = true,

    wheels = {},
}

for i = 0, 3 do
    state.wheels[i] = newWheelState()
    state[i] = state.wheels[i]
end

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
    if v > 0.0001 then return 1.0 end
    if v < -0.0001 then return -1.0 end
    return 0.0
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.001)
    dt = safeNumber(dt, 0.0)

    if tau <= 0.0001 then
        return target
    end

    local k = clamp(dt / (tau + dt), 0.0, 1.0)
    return current + (target - current) * k
end

local function approach(current, target, riseTau, fallTau, dt)
    local tau = target > current and riseTau or fallTau
    return lowPass(current, target, tau, dt)
end

local function smoothstep(edge0, edge1, x)
    if edge0 == edge1 then
        return x >= edge1 and 1.0 or 0.0
    end

    local t = clamp((safeNumber(x, 0.0) - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)
end

local function safeField(obj, field, defaultValue)
    if not obj then return defaultValue end

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

local function vecLength(v)
    if not v then return 0.0 end

    local ok, result = pcall(function()
        if type(v.length) == "function" then
            return v:length()
        end

        if type(v.length) == "number" then
            return v.length
        end

        local x = safeNumber(v.x, 0.0)
        local y = safeNumber(v.y, 0.0)
        local z = safeNumber(v.z, 0.0)
        return math.sqrt(x * x + y * y + z * z)
    end)

    if ok then
        return safeNumber(result, 0.0)
    end

    return 0.0
end

local function getSpeedKmh(car)
    if not car then return 0.0 end

    local speedKmh = safeField(car, "speedKmh", nil)
    if speedKmh ~= nil then
        return safeNumber(speedKmh, 0.0)
    end

    local speed = nil
    if speed ~= nil then
        return safeNumber(speed, 0.0)
    end

    local vel = safeField(car, "velocity", nil)
    if vel then
        return vecLength(vel) * 3.6
    end

    local lvel = safeField(car, "localVelocity", nil)
    if lvel then
        return vecLength(lvel) * 3.6
    end

    return 0.0
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

local function getWheel(car, index)
    if not car or not car.wheels then
        return nil
    end

    local ok, wheel = pcall(function()
        return car.wheels[index]
    end)

    if ok and wheel then
        return wheel
    end

    ok, wheel = pcall(function()
        return car.wheels[index + 1]
    end)

    if ok then
        return wheel
    end

    return nil
end

local function wheelSide(index)
    if index == FL or index == RL then
        return -1.0
    end
    return 1.0
end

local function isFront(index)
    return index < 2
end

local function updateDebugGate(dt)
    state.debugStoreTimer = (state.debugStoreTimer or 0.0) + dt

    if state.debugStoreTimer >= M.params.debugStoreInterval then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    else
        state.debugStoreNow = false
    end
end

local function shouldReadInputs(dt)
    state.inputTimer = (state.inputTimer or 0.0) + dt

    if state.inputTimer >= M.params.inputReadInterval then
        state.inputTimer = 0.0
        return true
    end

    return false
end

local function getWheelLoad(wheel, index)
    local fromContact = safeLoadRaw("ngp_contact_load_" .. index)
    if fromContact ~= nil then
        state.contactLinked = true
        return safeNumber(fromContact, 0.0)
    end

    local fromWheel = safeLoadRaw("ngp_wheel_load_" .. index)
    if fromWheel ~= nil then
        state.loadLinked = true
        return safeNumber(fromWheel, M.params.loadRef)
    end

    local fromLoadPath = safeLoadRaw("ngp_load_path_wheel_load_" .. index)
    if fromLoadPath ~= nil then
        state.loadLinked = true
        return safeNumber(fromLoadPath, M.params.loadRef)
    end

    local fromDlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if fromDlt ~= nil then
        state.loadLinked = true
        return M.params.loadRef + safeNumber(fromDlt, 0.0)
    end

    if wheel then
        return safeNumber(
            safeField(wheel, "load", safeField(wheel, "loadK", 0.0)),
            0.0
        )
    end

    return 0.0
end

local function readSignedSlip(wheel, index)
    local sa = safeLoadRaw("ngp_contact_slip_angle_" .. index)
    if sa == nil then sa = safeLoadRaw("ngp_slip_angle_" .. index) end
    if sa == nil then sa = safeLoadRaw("ngp_tire_slip_angle_" .. index) end
    if sa == nil then sa = safeLoadRaw("ngp_filtered_slip_angle_" .. index) end
    if sa == nil and wheel then sa = safeField(wheel, "slipAngle", 0.0) end

    local sr = safeLoadRaw("ngp_contact_slip_ratio_" .. index)
    if sr == nil then sr = safeLoadRaw("ngp_slip_ratio_" .. index) end
    if sr == nil then sr = safeLoadRaw("ngp_tire_slip_ratio_" .. index) end
    if sr == nil then sr = safeLoadRaw("ngp_filtered_slip_ratio_" .. index) end
    if sr == nil and wheel then sr = safeField(wheel, "slipRatio", 0.0) end

    local cs = safeLoadRaw("ngp_contact_combined_slip_" .. index)
    if cs == nil then
        local saAbs = abs(sa)
        local srAbs = abs(sr)
        cs = math.sqrt((saAbs / 0.22) * (saAbs / 0.22) + (srAbs / 0.35) * (srAbs / 0.35))
    end

    return safeNumber(sa, 0.0), safeNumber(sr, 0.0), clamp(safeNumber(cs, 0.0), 0.0, 4.0)
end

local function readYawRate(car)
    local av = safeField(car, "localAngularVelocity", nil)
    if av then
        return safeNumber(av.y, 0.0)
    end

    local v = safeLoadAlt(
        0.0,
        "ngp_yaw_rate",
        "ngp_body_yaw_rate",
        "ngp_chassis_yaw_rate"
    )

    return v
end

local function resetLinks()
    state.contactLinked = false
    state.memoryLinked = false
    state.damperLinked = false
    state.armLinked = false
    state.loadLinked = false
    state.carcassLinked = false
    state.slipRecoveryLinked = false
    state.yawLinked = false
    state.drivelineLinked = false
end

local function readCarInputs(car)
    state.speedKmh = getSpeedKmh(car)
    state.steer = safeNumber(safeField(car, "steer", 0.0), 0.0)
    state.gas = clamp(safeNumber(safeField(car, "gas", 0.0), 0.0), 0.0, 1.0)
    state.brake = clamp(safeNumber(safeField(car, "brake", 0.0), 0.0), 0.0, 1.0)
    state.handbrake = clamp(safeNumber(safeField(car, "handbrake", 0.0), 0.0), 0.0, 1.0)
    state.yawRate = readYawRate(car)

    local yawBudget, yKey = safeLoadAlt(0.0, "ngp_yaw_budget")
    local balance, bKey = safeLoadAlt(0.0, "ngp_yaw_balance")
    local yawBite, biteKey = safeLoadAlt(0.0, "ngp_yaw_bite", "ngp_yaw_bite_energy")
    local yawDirty, dirtyKey = safeLoadAlt(0.0, "ngp_yaw_dirty_return", "ngp_yaw_dirty_energy")

    if yKey or bKey or biteKey or dirtyKey then
        state.yawLinked = true
    end

    state.yawBudget = clamp(yawBudget, -1.0, 1.0)
    state.yawBalance = clamp(balance, -1.0, 1.0)
    state.yawBite = clamp(yawBite, 0.0, 1.0)
    state.yawDirty = clamp(yawDirty, 0.0, 1.0)

    local twist, tKey = safeLoadAlt(0.0, "ngp_shaft_twist", "ngp_driveline_twist", "ngp_windup_value", "ngp_driveline_windup")
    local rearPush, rpKey = safeLoadAlt(0.0, "ngp_drive_soft_rear_push", "ngp_driveline_rear_push", "ngp_windup_rear_push")

    if tKey or rpKey then
        state.drivelineLinked = true
    end

    state.drivelineTwist = clamp(twist, -1.5, 1.5)
    state.drivelineRearPush = clamp(rearPush, 0.0, 1.5)
end

local function readRootInputs(index, wheelState, wheel)
    local cq = safeLoadRaw("ngp_contact_quality_" .. index)
    local cStatus = safeLoadRaw("ngp_contact_status_" .. index)
    local trust = safeLoadRaw("ngp_contact_trust_" .. index)

    if cq == nil then cq = safeLoadRaw("ngp_tire_contact_quality_" .. index) end
    if cq == nil then cq = safeLoadRaw("ngp_tcr_quality_" .. index) end
    if trust == nil then trust = safeLoadRaw("ngp_cq_trust_" .. index) end

    if cq ~= nil or cStatus ~= nil or trust ~= nil then
        state.contactLinked = true
    end

    wheelState.contactQuality = clamp(safeNumber(cq, 1.0), 0.0, 1.2)
    wheelState.contactTrust = clamp(safeNumber(trust, wheelState.contactQuality), 0.0, 1.2)
    wheelState.contactLoss = clamp(1.0 - math.min(wheelState.contactQuality, wheelState.contactTrust), 0.0, 1.0)
    wheelState.contactStatus = safeNumber(cStatus, 0.0)

    local mem = safeLoadRaw("ngp_tire_memory_" .. index)
    if mem == nil then mem = safeLoadRaw("ngp_rubber_memory_" .. index) end
    if mem == nil then mem = safeLoadRaw("ngp_memory_" .. index) end

    local phaseId = safeLoadRaw("ngp_tire_memory_phase_id_" .. index)
    local recontact = safeLoadRaw("ngp_tire_memory_recontact_" .. index)
    local thermal = safeLoadRaw("ngp_memory_thermal_" .. index)
    local abrasion = safeLoadRaw("ngp_memory_abrasion_" .. index)
    local history = safeLoadRaw("ngp_memory_history_" .. index)
    local recoveryScalar = safeLoadRaw("ngp_memory_recovery_scalar_" .. index)

    if mem ~= nil or phaseId ~= nil or recontact ~= nil or thermal ~= nil or abrasion ~= nil or history ~= nil or recoveryScalar ~= nil then
        state.memoryLinked = true
    end

    wheelState.tireMemory = clamp(safeNumber(mem, 0.0), 0.0, 1.0)
    wheelState.tirePhaseId = safeNumber(phaseId, 0.0)
    wheelState.tireRecontact = clamp(safeNumber(recontact, 0.0), 0.0, 1.0)
    wheelState.tireThermal = clamp(safeNumber(thermal, 0.0), 0.0, 1.0)
    wheelState.tireAbrasion = clamp(safeNumber(abrasion, 0.0), 0.0, 1.0)
    wheelState.tireHistory = clamp(safeNumber(history, wheelState.tireMemory), 0.0, 1.0)
    wheelState.tireRecoveryScalar = clamp(safeNumber(recoveryScalar, 1.0), 0.0, 1.2)

    local rec = safeLoadRaw("ngp_slip_recovery_rate_" .. index)
    local gripRet = safeLoadRaw("ngp_slip_grip_return_" .. index)
    local snap = safeLoadRaw("ngp_slip_snap_risk_" .. index)
    local bite = safeLoadRaw("ngp_slip_bite_" .. index)
    local dirty = safeLoadRaw("ngp_slip_dirty_return_" .. index)

    if rec ~= nil or gripRet ~= nil or snap ~= nil or bite ~= nil or dirty ~= nil then
        state.slipRecoveryLinked = true
    end

    wheelState.slipRecovery = clamp(safeNumber(rec, 0.0), 0.0, 1.2)
    wheelState.gripReturn = clamp(safeNumber(gripRet, 1.0), 0.0, 1.2)
    wheelState.snapRisk = clamp(safeNumber(snap, 0.0), 0.0, 1.0)
    wheelState.bite = clamp(safeNumber(bite, 0.0), 0.0, 1.0)
    wheelState.dirtyReturn = clamp(safeNumber(dirty, 0.0), 0.0, 1.0)

    local dh = safeLoadRaw("ngp_damper_hyst_" .. index)
    local impact = safeLoadRaw("ngp_damper_hyst_impact_" .. index)
    local vel = safeLoadRaw("ngp_damper_hyst_velocity_" .. index)
    local vertical = safeLoadRaw("ngp_damper_vertical_" .. index)
    local hystDamp = safeLoadRaw("ngp_damper_hysteretic_" .. index)
    local pulse = safeLoadRaw("ngp_damper_vertical_pulse_" .. index)
    local roadMem = safeLoadRaw("ngp_damper_road_memory_" .. index)

    if dh ~= nil or impact ~= nil or vel ~= nil or vertical ~= nil or hystDamp ~= nil or pulse ~= nil or roadMem ~= nil then
        state.damperLinked = true
    end

    wheelState.damperHyst = clamp(safeNumber(dh, 0.0), 0.0, 1.0)
    wheelState.damperImpact = clamp(safeNumber(impact, 0.0), 0.0, 1.0)
    wheelState.damperVelocity = safeNumber(vel, 0.0)
    wheelState.damperVertical = clamp(safeNumber(vertical, 0.0), 0.0, 1.0)
    wheelState.damperHysteretic = clamp(safeNumber(hystDamp, wheelState.damperHyst), 0.0, 1.0)
    wheelState.damperPulse = clamp(safeNumber(pulse, 0.0), 0.0, 1.0)
    wheelState.roadMemory = clamp(safeNumber(roadMem, 0.0), 0.0, 1.0)

    local support = safeLoadRaw("ngp_carcass_support_" .. index)
    local gripGate = safeLoadRaw("ngp_carcass_grip_gate_" .. index)
    local carcassHyst = safeLoadRaw("ngp_carcass_hysteresis_" .. index)
    local recoveryBias = safeLoadRaw("ngp_carcass_recovery_bias_" .. index)

    if support ~= nil or gripGate ~= nil or carcassHyst ~= nil or recoveryBias ~= nil then
        state.carcassLinked = true
    end

    wheelState.carcassSupport = clamp(safeNumber(support, 1.0), 0.0, 1.2)
    wheelState.carcassGripGate = clamp(safeNumber(gripGate, 1.0), 0.0, 1.2)
    wheelState.carcassHysteresis = clamp(safeNumber(carcassHyst, 0.0), 0.0, 1.0)
    wheelState.carcassRecoveryBias = clamp(safeNumber(recoveryBias, 0.0), 0.0, 1.0)

    local armCamber = safeLoadRaw("ngp_control_arm_camber_" .. index)
    if armCamber == nil then armCamber = safeLoadRaw("ngp_arm_camber_" .. index) end
    if armCamber == nil then armCamber = safeLoadRaw("ngp_ultra_camber_" .. index) end
    if armCamber == nil then armCamber = safeLoadRaw("ngp_caster_total_camber_" .. index) end

    local armToe = safeLoadRaw("ngp_control_arm_toe_" .. index)
    if armToe == nil then armToe = safeLoadRaw("ngp_arm_toe_" .. index) end
    if armToe == nil then armToe = safeLoadRaw("ngp_ultra_toe_" .. index) end
    if armToe == nil then armToe = safeLoadRaw("ngp_caster_total_toe_" .. index) end

    if armCamber ~= nil or armToe ~= nil then
        state.armLinked = true
    end

    wheelState.armCamber = safeNumber(armCamber, 0.0)
    wheelState.armToe = safeNumber(armToe, 0.0)

    local sa, sr, cs = readSignedSlip(wheel, index)
    wheelState.slipAngle = sa
    wheelState.slipRatio = sr
    wheelState.combinedSlip = cs
end

local function classifyPhase(index, wheelState)
    if wheelState.contactStatus == 4 or wheelState.contactQuality < M.params.unloadedQuality then
        return 8, PHASE_TEXT[8]
    end

    if wheelState.damperImpact >= M.params.impactThreshold or wheelState.damperPulse > 0.55 then
        return 7, PHASE_TEXT[7]
    end

    if index >= 2 and (abs(wheelState.subframeDeflection) > M.params.axleWindupThreshold or wheelState.bite > 0.45) then
        return 9, PHASE_TEXT[9]
    end

    if wheelState.bodyDeflection > M.params.bodyFlexThreshold then
        return 10, PHASE_TEXT[10]
    end

    if wheelState.tirePhaseId == 4 or wheelState.tireRecontact > 0.1 or wheelState.slipRecovery > 0.38 then
        return 6, PHASE_TEXT[6]
    end

    if state.brake > 0.18 or (state.handbrake > 0.15 and index >= 2) then
        return 4, PHASE_TEXT[4]
    end

    if index >= 2 and state.gas > 0.20 and abs(wheelState.slipRatio) > 0.025 then
        return 3, PHASE_TEXT[3]
    end

    if isFront(index) and abs(state.steer) > 0.07 and abs(wheelState.latFlex) > 0.006 then
        return 2, PHASE_TEXT[2]
    end

    if wheelState.energy > M.params.energyActive then
        return 5, PHASE_TEXT[5]
    end

    return 1, PHASE_TEXT[1]
end

local function updateWheel(index, wheelState, dt, wheel)
    local p = M.params
    local side = wheelSide(index)
    local front = isFront(index)

    local load = math.abs(getWheelLoad(wheel, index))
    wheelState.load = lowPass(wheelState.load, load, 0.050, dt)
    wheelState.loadFactor = wheelState.load / math.max(p.loadRef, 1.0)

    readRootInputs(index, wheelState, wheel)

    local loadFlex = smoothstep(p.loadFlexStart, p.loadFlexFull, wheelState.loadFactor)
    local harshLoad = smoothstep(p.loadHarshStart, p.loadHarshFull, wheelState.loadFactor)

    local latScale = front and p.lateralFlexScaleFront or p.lateralFlexScaleRear
    local longScale = front and p.longitudinalFlexScaleFront or p.longitudinalFlexScaleRear

    local supportLoss = clamp(1.0 - math.min(wheelState.carcassSupport, wheelState.carcassGripGate), 0.0, 1.0)
    local contactLoss = wheelState.contactLoss
    local memoryFlex = wheelState.tireMemory * p.memoryGain + wheelState.tireHistory * p.historyGain
    local damperFlex = math.max(wheelState.damperHyst, wheelState.damperHysteretic) * p.damperGain
    local impactFlex = math.max(wheelState.damperImpact, wheelState.damperPulse) * p.impactGain
    local trustFlex = clamp(1.0 - wheelState.contactTrust, 0.0, 1.0) * p.contactTrustGain

    local signedLatInput =
        wheelState.slipAngle * p.slipAngleFlexGain
        + state.steer * (front and p.steerFlexGain or (p.steerFlexGain * 0.25))
        + state.yawRate * p.yawFlexGain
        + wheelState.armToe * p.armToeGain
        + state.yawBalance * p.yawBalanceGain
        + state.yawBite * p.yawBiteGain

    local signedLongInput =
        wheelState.slipRatio * p.slipRatioFlexGain
        + state.brake * p.brakeFlexGain * (front and 1.0 or 0.75)
        - state.gas * p.driveFlexGain * (front and 0.10 or 1.0)
        + state.handbrake * p.handbrakeRearGain * (front and 0.0 or 1.0)

    if not front then
        signedLongInput =
            signedLongInput
            - state.drivelineRearPush * p.drivelineRearPushGain
            + state.drivelineTwist * p.drivelineTwistGain
    end

    local bushingLatTarget =
        clamp(
            abs(signedLatInput) * p.bushingLatGain
            + contactLoss * p.bushingContactLossGain
            + trustFlex
            + supportLoss * 0.22,
            0.0,
            p.maxBushing
        )

    local bushingLongTarget =
        clamp(
            abs(signedLongInput) * p.bushingLongGain
            + contactLoss * p.bushingContactLossGain
            + harshLoad * 0.12
            + supportLoss * 0.18,
            0.0,
            p.maxBushing
        )

    wheelState.bushingLat = lowPass(wheelState.bushingLat, bushingLatTarget, p.bushingTau, dt)
    wheelState.bushingLong = lowPass(wheelState.bushingLong, bushingLongTarget, p.bushingTau, dt)
    wheelState.bushingEnergy = clamp((wheelState.bushingLat + wheelState.bushingLong) * 0.5, 0.0, 1.0)

    local knuckleTarget =
        clamp(
            wheelState.combinedSlip * p.knuckleSlipGain
            + loadFlex * p.knuckleLoadGain
            + impactFlex * p.knuckleImpactGain
            + abs(wheelState.armCamber) * 0.35
            + abs(wheelState.armToe) * 0.35,
            0.0,
            p.maxKnuckle
        )

    wheelState.knuckleDeflection = lowPass(wheelState.knuckleDeflection, knuckleTarget, p.knuckleTau, dt)

    local subframeTarget =
        clamp(
            abs(state.yawRate) * p.subframeYawGain
            + state.gas * p.subframeDriveGain * (front and 0.25 or 1.0)
            + state.brake * p.subframeBrakeGain
            + wheelState.damperVertical * p.subframeDamperGain
            + abs(state.drivelineTwist) * (front and 0.05 or 0.22)
            + wheelState.bite * p.biteGain
            + wheelState.dirtyReturn * p.dirtyReturnGain,
            0.0,
            p.maxSubframe
        )

    wheelState.subframeDeflection = lowPass(wheelState.subframeDeflection, subframeTarget, p.subframeTau, dt)

    local bodyTarget =
        clamp(
            loadFlex * p.bodyLoadGain
            + abs(state.yawBudget) * p.bodyYawGain
            + wheelState.carcassHysteresis * p.bodyHystGain
            + wheelState.damperHysteretic * 0.18
            + wheelState.roadMemory * 0.16
            + wheelState.snapRisk * p.snapRiskGain,
            0.0,
            p.maxBody
        )

    wheelState.bodyDeflection = lowPass(wheelState.bodyDeflection, bodyTarget, p.bodyTau, dt)

    local stackLat =
        wheelState.bushingLat * 0.30
        + wheelState.knuckleDeflection * 0.24
        + wheelState.subframeDeflection * 0.20
        + wheelState.bodyDeflection * 0.14
        + loadFlex * 0.12

    local stackLong =
        wheelState.bushingLong * 0.34
        + wheelState.subframeDeflection * 0.26
        + wheelState.bodyDeflection * 0.15
        + loadFlex * 0.12
        + wheelState.dirtyReturn * 0.13

    local commonFlex =
        loadFlex
        + contactLoss * p.contactLossGain
        + memoryFlex
        + damperFlex
        + impactFlex
        + trustFlex
        + supportLoss * 0.22

    local targetLat =
        clamp(
            signedLatInput * latScale
            + side * commonFlex * latScale * 0.45
            + sign(signedLatInput) * stackLat * latScale,
            -p.maxLatFlex,
            p.maxLatFlex
        )

    local targetLong =
        clamp(
            signedLongInput * longScale
            + sign(signedLongInput) * commonFlex * longScale * 0.35
            + sign(signedLongInput) * stackLong * longScale,
            -p.maxLongFlex,
            p.maxLongFlex
        )

    wheelState.targetLatFlex = targetLat
    wheelState.targetLongFlex = targetLong

    wheelState.latFlex = lowPass(wheelState.latFlex, targetLat, p.flexTau, dt)
    wheelState.longFlex = lowPass(wheelState.longFlex, targetLong, p.flexTau, dt)

    local targetEnergy =
        clamp(
            abs(wheelState.latFlex) / math.max(p.maxLatFlex, 0.001) * 0.30
            + abs(wheelState.longFlex) / math.max(p.maxLongFlex, 0.001) * 0.22
            + wheelState.bushingEnergy * 0.16
            + wheelState.knuckleDeflection * 0.12
            + wheelState.subframeDeflection * 0.10
            + wheelState.bodyDeflection * 0.06
            + wheelState.tireRecontact * 0.04,
            0.0,
            1.0
        )

    wheelState.targetEnergy = targetEnergy

    local tau = targetEnergy > wheelState.energy and p.energyRiseTau or p.energyFallTau
    wheelState.energy = lowPass(wheelState.energy, targetEnergy, tau, dt)

    local memoryTarget =
        math.max(
            wheelState.energy,
            wheelState.tireMemory * 0.75,
            wheelState.tireHistory * 0.55,
            wheelState.damperHyst * 0.45,
            wheelState.bodyDeflection * 0.42
        )

    wheelState.memory = approach(wheelState.memory, memoryTarget, p.energyRiseTau, p.returnTau, dt)

    wheelState.returnBias =
        clamp(
            (1.0 - wheelState.tireRecoveryScalar) * 0.38
            + wheelState.dirtyReturn * 0.26
            + wheelState.carcassRecoveryBias * 0.22
            + wheelState.memory * 0.14,
            0.0,
            1.0
        )

    wheelState.forceLeak =
        clamp(
            wheelState.energy * 0.34
            + wheelState.bushingEnergy * 0.24
            + wheelState.bodyDeflection * 0.16
            + wheelState.contactLoss * 0.14
            + supportLoss * 0.12,
            0.0,
            1.0
        )

    wheelState.loadPathLoss =
        clamp(
            wheelState.forceLeak * p.loadPathLossGain
            + wheelState.returnBias * 0.20
            + wheelState.damperPulse * 0.18
            + wheelState.snapRisk * 0.12,
            0.0,
            1.0
        )

    wheelState.steerDelay =
        clamp(
            (front and abs(wheelState.latFlex) / math.max(p.maxLatFlex, 0.001) or 0.0) * p.steerDelayGain
            + wheelState.bushingEnergy * 0.14
            + wheelState.bodyDeflection * 0.10,
            0.0,
            1.0
        )

    local toeGain = front and p.toeFromLatFlexFront or p.toeFromLatFlexRear
    local camberGain = front and p.camberFromLatFlexFront or p.camberFromLatFlexRear

    wheelState.virtualToe =
        clamp(
            wheelState.armToe
            + wheelState.latFlex * toeGain
            + wheelState.longFlex * p.toeFromLongFlex
            + side * wheelState.subframeDeflection * 0.006
            + side * wheelState.bushingEnergy * 0.004,
            -p.maxVirtualToe,
            p.maxVirtualToe
        )

    wheelState.virtualCamber =
        clamp(
            wheelState.armCamber
            - side * wheelState.latFlex * camberGain
            - loadFlex * p.camberFromLoadFlex
            - side * wheelState.knuckleDeflection * 0.006,
            -p.maxVirtualCamber,
            p.maxVirtualCamber
        )

    wheelState.phaseId, wheelState.phaseText = classifyPhase(index, wheelState)
    wheelState.active = true
end

local function applyRearAxleCoupling(dt)
    local p = M.params
    local rl = state.wheels[RL]
    local rr = state.wheels[RR]

    if not rl or not rr then
        return
    end

    local latDiff = (rr.latFlex or 0.0) - (rl.latFlex or 0.0)
    local longAvg = ((rl.longFlex or 0.0) + (rr.longFlex or 0.0)) * 0.5
    local loadDiff = clamp(((rr.load or 0.0) - (rl.load or 0.0)) / math.max(p.loadRef, 1.0), -1.5, 1.5)

    local axleTarget =
        clamp(
            abs(latDiff) * 3.0
            + abs(longAvg) * 2.0
            + abs(loadDiff) * 0.18
            + state.drivelineRearPush * 0.20
            + state.yawBite * 0.18,
            0.0,
            1.0
        )

    state.rearAxleWindup = lowPass(state.rearAxleWindup, axleTarget, 0.120, dt)

    local toeOffset = clamp(latDiff * p.rearAxleToeCoupling + loadDiff * 0.006, -0.018, 0.018)
    local camberOffset = clamp(abs(loadDiff) * p.rearAxleCamberCoupling * 0.012, 0.0, 0.014)

    rl.virtualToe = clamp((rl.virtualToe or 0.0) + toeOffset, -p.maxVirtualToe, p.maxVirtualToe)
    rr.virtualToe = clamp((rr.virtualToe or 0.0) + toeOffset, -p.maxVirtualToe, p.maxVirtualToe)

    rl.virtualCamber = clamp((rl.virtualCamber or 0.0) - camberOffset, -p.maxVirtualCamber, p.maxVirtualCamber)
    rr.virtualCamber = clamp((rr.virtualCamber or 0.0) - camberOffset, -p.maxVirtualCamber, p.maxVirtualCamber)

    rl.subframeDeflection = clamp((rl.subframeDeflection or 0.0) + state.rearAxleWindup * p.rearAxleCoupling * 0.10, 0.0, 1.0)
    rr.subframeDeflection = clamp((rr.subframeDeflection or 0.0) + state.rearAxleWindup * p.rearAxleCoupling * 0.10, 0.0, 1.0)
end

local function clearWheel(index, dt)
    local p = M.params
    local w = state.wheels[index]

    if not w then
        return
    end

    w.load = lowPass(w.load, 0.0, p.returnTau, dt)
    w.loadFactor = 0.0
    w.contactQuality = 0.0
    w.contactTrust = 0.0
    w.contactLoss = 1.0
    w.contactStatus = 0
    w.tireMemory = 0.0
    w.tireHistory = 0.0
    w.tireThermal = 0.0
    w.tireAbrasion = 0.0
    w.tireRecoveryScalar = 1.0
    w.tireRecontact = 0.0
    w.slipRecovery = 0.0
    w.gripReturn = 1.0
    w.snapRisk = 0.0
    w.bite = 0.0
    w.dirtyReturn = 0.0
    w.slipAngle = 0.0
    w.slipRatio = 0.0
    w.combinedSlip = 0.0
    w.damperHyst = 0.0
    w.damperImpact = 0.0
    w.damperVelocity = 0.0
    w.damperVertical = 0.0
    w.damperHysteretic = 0.0
    w.damperPulse = 0.0
    w.roadMemory = 0.0
    w.carcassSupport = 1.0
    w.carcassGripGate = 1.0
    w.carcassHysteresis = 0.0
    w.carcassRecoveryBias = 0.0
    w.armCamber = 0.0
    w.armToe = 0.0

    w.bushingLat = lowPass(w.bushingLat, 0.0, p.returnTau, dt)
    w.bushingLong = lowPass(w.bushingLong, 0.0, p.returnTau, dt)
    w.bushingEnergy = lowPass(w.bushingEnergy, 0.0, p.returnTau, dt)
    w.knuckleDeflection = lowPass(w.knuckleDeflection, 0.0, p.returnTau, dt)
    w.subframeDeflection = lowPass(w.subframeDeflection, 0.0, p.returnTau, dt)
    w.bodyDeflection = lowPass(w.bodyDeflection, 0.0, p.returnTau, dt)
    w.latFlex = lowPass(w.latFlex, 0.0, p.returnTau, dt)
    w.longFlex = lowPass(w.longFlex, 0.0, p.returnTau, dt)
    w.energy = lowPass(w.energy, 0.0, p.returnTau, dt)
    w.memory = lowPass(w.memory, 0.0, p.returnTau, dt)
    w.returnBias = lowPass(w.returnBias, 0.0, p.returnTau, dt)
    w.forceLeak = lowPass(w.forceLeak, 0.0, p.returnTau, dt)
    w.loadPathLoss = lowPass(w.loadPathLoss, 0.0, p.returnTau, dt)
    w.steerDelay = lowPass(w.steerDelay, 0.0, p.returnTau, dt)

    w.virtualToe = 0.0
    w.virtualCamber = 0.0
    w.targetLatFlex = 0.0
    w.targetLongFlex = 0.0
    w.targetEnergy = 0.0
    w.phaseId = 0
    w.phaseText = "NO WHEEL"
    w.active = false
end

local function updateGlobalAggregates(dt)
    local p = M.params
    local w0 = state.wheels[FL]
    local w1 = state.wheels[FR]
    local w2 = state.wheels[RL]
    local w3 = state.wheels[RR]

    state.frontSteerDelay =
        clamp(
            ((w0.steerDelay or 0.0) + (w1.steerDelay or 0.0)) * 0.5,
            0.0,
            1.0
        )

    state.rearToeCompliance =
        clamp(
            (abs(w2.virtualToe or 0.0) + abs(w3.virtualToe or 0.0)) * 0.5 / math.max(p.maxVirtualToe, 0.001),
            0.0,
            1.0
        )

    state.bodyFlex =
        lowPass(
            state.bodyFlex,
            ((w0.bodyDeflection or 0.0) + (w1.bodyDeflection or 0.0) + (w2.bodyDeflection or 0.0) + (w3.bodyDeflection or 0.0)) * 0.25,
            p.bodyTau,
            dt
        )

    state.subframeTwist =
        lowPass(
            state.subframeTwist,
            ((w2.subframeDeflection or 0.0) + (w3.subframeDeflection or 0.0)) * 0.5
            - ((w0.subframeDeflection or 0.0) + (w1.subframeDeflection or 0.0)) * 0.5,
            p.subframeTau,
            dt
        )

    state.forceLeak =
        clamp(
            ((w0.forceLeak or 0.0) + (w1.forceLeak or 0.0) + (w2.forceLeak or 0.0) + (w3.forceLeak or 0.0)) * 0.25 * p.forceLeakGain,
            0.0,
            1.0
        )

    state.loadPathLoss =
        clamp(
            ((w0.loadPathLoss or 0.0) + (w1.loadPathLoss or 0.0) + (w2.loadPathLoss or 0.0) + (w3.loadPathLoss or 0.0)) * 0.25,
            0.0,
            1.0
        )

    state.chassisCompliance =
        clamp(
            state.avgEnergy * 0.54
            + state.avgToe * 1.15
            + state.avgCamber * 0.58
            + state.bodyFlex * 0.24
            + abs(state.subframeTwist) * 0.18
            + state.forceLeak * 0.20,
            0.0,
            1.0
        )
end

local function exportWheel(index, w)
    safeStore("ngp_compliance_energy_" .. index, w.energy or 0.0)
    safeStore("ngp_compliance_memory_" .. index, w.memory or 0.0)
    safeStore("ngp_compliance_lat_flex_" .. index, w.latFlex or 0.0)
    safeStore("ngp_compliance_long_flex_" .. index, w.longFlex or 0.0)
    safeStore("ngp_compliance_virtual_toe_" .. index, w.virtualToe or 0.0)
    safeStore("ngp_compliance_virtual_camber_" .. index, w.virtualCamber or 0.0)

    safeStore("ngp_virtual_toe_" .. index, w.virtualToe or 0.0)
    safeStore("ngp_virtual_camber_" .. index, w.virtualCamber or 0.0)
    safeStore("ngp_compliance_stack_energy_" .. index, w.energy or 0.0)

    safeStore("ngp_compliance_bushing_lat_" .. index, w.bushingLat or 0.0)
    safeStore("ngp_compliance_bushing_long_" .. index, w.bushingLong or 0.0)
    safeStore("ngp_compliance_bushing_energy_" .. index, w.bushingEnergy or 0.0)
    safeStore("ngp_compliance_knuckle_deflection_" .. index, w.knuckleDeflection or 0.0)
    safeStore("ngp_compliance_subframe_deflection_" .. index, w.subframeDeflection or 0.0)
    safeStore("ngp_compliance_body_deflection_" .. index, w.bodyDeflection or 0.0)
    safeStore("ngp_compliance_force_leak_" .. index, w.forceLeak or 0.0)
    safeStore("ngp_compliance_load_path_loss_" .. index, w.loadPathLoss or 0.0)
    safeStore("ngp_compliance_return_bias_" .. index, w.returnBias or 0.0)
    safeStore("ngp_compliance_steer_delay_" .. index, w.steerDelay or 0.0)

    safeStore("ngp_force_path_loss_" .. index, w.forceLeak or 0.0)
    safeStore("ngp_body_compliance_" .. index, w.bodyDeflection or 0.0)
    safeStore("ngp_bushing_energy_" .. index, w.bushingEnergy or 0.0)
    safeStore("ngp_knuckle_deflection_" .. index, w.knuckleDeflection or 0.0)

    safeStore("ngp_compliance_phase_id_" .. index, w.phaseId or 0)
    safeStore("ngp_compliance_phase_" .. index, w.phaseText or "UNKNOWN")

    safeStore("ngp_cs_energy_" .. index, w.energy or 0.0)
    safeStore("ngp_cs_toe_" .. index, w.virtualToe or 0.0)
    safeStore("ngp_cs_camber_" .. index, w.virtualCamber or 0.0)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_compliance_load_" .. index, w.load or 0.0)
    safeStore("ngp_compliance_contact_quality_" .. index, w.contactQuality or 0.0)
    safeStore("ngp_compliance_contact_trust_" .. index, w.contactTrust or 0.0)
    safeStore("ngp_compliance_tire_memory_" .. index, w.tireMemory or 0.0)
    safeStore("ngp_compliance_tire_history_" .. index, w.tireHistory or 0.0)
    safeStore("ngp_compliance_damper_hyst_" .. index, w.damperHyst or 0.0)
    safeStore("ngp_compliance_damper_impact_" .. index, w.damperImpact or 0.0)
    safeStore("ngp_compliance_damper_vertical_" .. index, w.damperVertical or 0.0)
    safeStore("ngp_compliance_slip_angle_" .. index, w.slipAngle or 0.0)
    safeStore("ngp_compliance_slip_ratio_" .. index, w.slipRatio or 0.0)
    safeStore("ngp_compliance_combined_slip_" .. index, w.combinedSlip or 0.0)
    safeStore("ngp_compliance_arm_camber_" .. index, w.armCamber or 0.0)
    safeStore("ngp_compliance_arm_toe_" .. index, w.armToe or 0.0)
    safeStore("ngp_compliance_bite_" .. index, w.bite or 0.0)
    safeStore("ngp_compliance_dirty_return_" .. index, w.dirtyReturn or 0.0)
    safeStore("ngp_compliance_snap_risk_" .. index, w.snapRisk or 0.0)
end

local function exportGlobal()
    safeStore("ngp_compliance_status", state.status or "UNKNOWN")
    safeStore("ngp_compliance_update_count", state.updateCount or 0)
    safeStore("ngp_compliance_wheels_valid", state.wheelsValid and 1 or 0)

    safeStore("ngp_compliance_avg_energy", state.avgEnergy or 0.0)
    safeStore("ngp_compliance_avg_lat_flex", state.avgLatFlex or 0.0)
    safeStore("ngp_compliance_avg_long_flex", state.avgLongFlex or 0.0)
    safeStore("ngp_compliance_avg_toe", state.avgToe or 0.0)
    safeStore("ngp_compliance_avg_camber", state.avgCamber or 0.0)

    safeStore("ngp_compliance_avg_bushing", state.avgBushing or 0.0)
    safeStore("ngp_compliance_avg_knuckle", state.avgKnuckle or 0.0)
    safeStore("ngp_compliance_avg_subframe", state.avgSubframe or 0.0)
    safeStore("ngp_compliance_avg_body", state.avgBody or 0.0)
    safeStore("ngp_compliance_avg_force_leak", state.avgForceLeak or 0.0)

    safeStore("ngp_compliance_front_steer_delay", state.frontSteerDelay or 0.0)
    safeStore("ngp_compliance_rear_toe", state.rearToeCompliance or 0.0)
    safeStore("ngp_compliance_chassis", state.chassisCompliance or 0.0)
    safeStore("ngp_compliance_body_flex", state.bodyFlex or 0.0)
    safeStore("ngp_compliance_subframe_twist", state.subframeTwist or 0.0)
    safeStore("ngp_compliance_rear_axle_windup", state.rearAxleWindup or 0.0)
    safeStore("ngp_compliance_load_path_loss", state.loadPathLoss or 0.0)
    safeStore("ngp_compliance_force_leak", state.forceLeak or 0.0)

    safeStore("ngp_chassis_compliance", state.chassisCompliance or 0.0)
    safeStore("ngp_body_flex", state.bodyFlex or 0.0)
    safeStore("ngp_subframe_twist", state.subframeTwist or 0.0)
    safeStore("ngp_rear_axle_windup", state.rearAxleWindup or 0.0)
    safeStore("ngp_load_path_compliance_loss", state.loadPathLoss or 0.0)
    safeStore("ngp_force_leak", state.forceLeak or 0.0)

    safeStore("ngp_compliance_stack_status", state.status or "UNKNOWN")
    safeStore("ngp_compliance_stack_update_count", state.updateCount or 0)
    safeStore("ngp_compliance_stack_avg_energy", state.avgEnergy or 0.0)

    safeStore("ngp_cs_status", state.status or "UNKNOWN")
    safeStore("ngp_cs_count", state.updateCount or 0)
    safeStore("ngp_cs_energy", state.avgEnergy or 0.0)
    safeStore("ngp_cs_force_leak", state.forceLeak or 0.0)
    safeStore("ngp_cs_load_path_loss", state.loadPathLoss or 0.0)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_compliance_contact_linked", state.contactLinked and 1 or 0)
    safeStore("ngp_compliance_memory_linked", state.memoryLinked and 1 or 0)
    safeStore("ngp_compliance_damper_linked", state.damperLinked and 1 or 0)
    safeStore("ngp_compliance_arm_linked", state.armLinked and 1 or 0)
    safeStore("ngp_compliance_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_compliance_carcass_linked", state.carcassLinked and 1 or 0)
    safeStore("ngp_compliance_slip_recovery_linked", state.slipRecoveryLinked and 1 or 0)
    safeStore("ngp_compliance_yaw_linked", state.yawLinked and 1 or 0)
    safeStore("ngp_compliance_driveline_linked", state.drivelineLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i, state.wheels[i])
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
        exportState()
        return
    end

    updateDebugGate(dt)

    car = car or safeGetCar()

    if not car then
        state.status = "NO CAR"
        state.wheelsValid = false
        exportState()
        return
    end

    if not safeField(car, "wheels", nil) then
        state.status = "NO WHEELS"
        state.wheelsValid = false
        exportState()
        return
    end

    state.status = "RUNNING"
    state.wheelsValid = true

    resetLinks()

    if shouldReadInputs(dt) then
        readCarInputs(car)
    else
        state.yawRate = readYawRate(car)
        state.steer = safeNumber(safeField(car, "steer", state.steer), state.steer)
        state.gas = clamp(safeNumber(safeField(car, "gas", state.gas), state.gas), 0.0, 1.0)
        state.brake = clamp(safeNumber(safeField(car, "brake", state.brake), state.brake), 0.0, 1.0)
        state.handbrake = clamp(safeNumber(safeField(car, "handbrake", state.handbrake), state.handbrake), 0.0, 1.0)
    end

    local sumEnergy = 0.0
    local sumLat = 0.0
    local sumLong = 0.0
    local sumToe = 0.0
    local sumCamber = 0.0
    local sumBushing = 0.0
    local sumKnuckle = 0.0
    local sumSubframe = 0.0
    local sumBody = 0.0
    local sumForceLeak = 0.0

    for i = 0, 3 do
        local wheel = getWheel(car, i)
        local wheelState = state.wheels[i]

        if wheel then
            updateWheel(i, wheelState, dt, wheel)
        else
            clearWheel(i, dt)
        end
    end

    applyRearAxleCoupling(dt)

    for i = 0, 3 do
        local wheelState = state.wheels[i]

        sumEnergy = sumEnergy + (wheelState.energy or 0.0)
        sumLat = sumLat + abs(wheelState.latFlex)
        sumLong = sumLong + abs(wheelState.longFlex)
        sumToe = sumToe + abs(wheelState.virtualToe)
        sumCamber = sumCamber + abs(wheelState.virtualCamber)
        sumBushing = sumBushing + (wheelState.bushingEnergy or 0.0)
        sumKnuckle = sumKnuckle + (wheelState.knuckleDeflection or 0.0)
        sumSubframe = sumSubframe + (wheelState.subframeDeflection or 0.0)
        sumBody = sumBody + (wheelState.bodyDeflection or 0.0)
        sumForceLeak = sumForceLeak + (wheelState.forceLeak or 0.0)

        exportWheel(i, wheelState)
    end

    state.avgEnergy = sumEnergy * 0.25
    state.avgLatFlex = sumLat * 0.25
    state.avgLongFlex = sumLong * 0.25
    state.avgToe = sumToe * 0.25
    state.avgCamber = sumCamber * 0.25
    state.avgBushing = sumBushing * 0.25
    state.avgKnuckle = sumKnuckle * 0.25
    state.avgSubframe = sumSubframe * 0.25
    state.avgBody = sumBody * 0.25
    state.avgForceLeak = sumForceLeak * 0.25

    updateGlobalAggregates(dt)
    exportGlobal()
end

function M.getEnergy(index)
    local w = state.wheels[index]
    return w and w.energy or 0.0
end

function M.getVirtualToe(index)
    local w = state.wheels[index]
    return w and w.virtualToe or 0.0
end

function M.getVirtualCamber(index)
    local w = state.wheels[index]
    return w and w.virtualCamber or 0.0
end

function M.getForceLeak(index)
    local w = state.wheels[index]
    return w and w.forceLeak or 0.0
end

function M.getLoadPathLoss(index)
    local w = state.wheels[index]
    return w and w.loadPathLoss or 0.0
end

function M.getChassisCompliance()
    return state.chassisCompliance or 0.0
end

function M.getState(index)
    if index == nil then
        return state
    end
    return state.wheels[index]
end

function M.debugStr(index)
    local w = state.wheels[index or 0] or state.wheels[0]

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "Phase %s / E %.3f / Mem %.3f / Leak %.3f\n" ..
        "Flex Lat %+.4f / Long %+.4f / Bush %.2f Kn %.2f\n" ..
        "Toe %+.4f / Camber %+.4f / Sub %.2f Body %.2f\n" ..
        "CQ %.2f Trust %.2f / TM %.2f / DH %.2f / I %.2f\n" ..
        "Links CQ:%s TM:%s DH:%s TC:%s SR:%s Yaw:%s Drive:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",

        tostring(w.phaseText or "UNKNOWN"),
        w.energy or 0.0,
        w.memory or 0.0,
        w.forceLeak or 0.0,

        w.latFlex or 0.0,
        w.longFlex or 0.0,
        w.bushingEnergy or 0.0,
        w.knuckleDeflection or 0.0,

        w.virtualToe or 0.0,
        w.virtualCamber or 0.0,
        w.subframeDeflection or 0.0,
        w.bodyDeflection or 0.0,

        w.contactQuality or 0.0,
        w.contactTrust or 0.0,
        w.tireMemory or 0.0,
        w.damperHyst or 0.0,
        w.damperImpact or 0.0,

        state.contactLinked and "OK" or "NIL",
        state.memoryLinked and "OK" or "NIL",
        state.damperLinked and "OK" or "NIL",
        state.carcassLinked and "OK" or "NIL",
        state.slipRecoveryLinked and "OK" or "NIL",
        state.yawLinked and "OK" or "NIL",
        state.drivelineLinked and "OK" or "NIL"
    )
end

return M
