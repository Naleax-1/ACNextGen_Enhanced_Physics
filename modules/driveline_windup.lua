---@diagnostic disable: undefined-global

--============================================================
-- driveline_windup.lua
-- ACNextGen V1.1.5 Stable
-- Driveline Windup / Pseudo Soft-Body Driveline Bridge
--
-- Purpose:
--  - Keep drivetrain.lua alive; do not replace it.
--  - Observe torque take-up, shaft windup, backlash, reconnect shock and rear push.
--  - Share root/trunk signals for LSD, yaw budget, tire memory and slip recovery.
--  - No direct AC physics overwrite here.
--============================================================

local M = {}

local WHEEL_NAMES = {
    [0] = "FL",
    [1] = "FR",
    [2] = "RL",
    [3] = "RR",
}

--============================================================
-- Parameters
--============================================================

M.params = {
    --------------------------------------------------------
    -- AE86/rF2 reference feel
    -- AE86_ProjectD.hdv:
    -- ClutchInertia=0.05 / ClutchTorque=250 / ClutchFriction=3.10
    -- ClutchEngageRate=1.5 / UpshiftDelay=0.10 / DownshiftDelay=0.10
    --------------------------------------------------------
    clutchInertiaRef = 0.05,
    clutchTorqueRef = 250.0,
    clutchFrictionRef = 3.10,
    clutchEngageRate = 1.50,

    upshiftDelayRef = 0.10,
    downshiftDelayRef = 0.10,
    downshiftBlipThrottle = 0.40,

    --------------------------------------------------------
    -- Demand / filtering
    --------------------------------------------------------
    inputTauDrive = 0.055,
    inputTauCoast = 0.075,
    inputTauReverse = 0.045,

    torqueScale = 1.0,
    fallbackTorqueScale = 1.0,
    engineBrakeGain = 0.16,

    --------------------------------------------------------
    -- Pseudo soft-body driveline
    --------------------------------------------------------
    windupStiffness = 7.8,
    windupDamping = 1.55,
    windupBuildGain = 1.20,
    windupReleaseGain = 1.45,

    shaftTwistGain = 1.00,
    shaftTorsionGain = 0.42,

    maxWindup = 1.0,
    maxWindupVelocity = 8.0,
    maxShaftTwist = 1.0,

    --------------------------------------------------------
    -- Backlash / lash
    --------------------------------------------------------
    backlashThreshold = 0.060,
    backlashBuildGain = 0.95,
    backlashDecayGain = 2.10,
    backlashShockGain = 0.45,

    throttleReapplyThreshold = 0.12,
    torqueReverseThreshold = 0.16,

    --------------------------------------------------------
    -- Clutch / shift / reconnect
    --------------------------------------------------------
    clutchKickThrottle = 0.35,
    clutchReleaseRate = 2.50,
    clutchShockGain = 0.38,
    shiftShockGain = 0.55,
    shiftShockDecay = 5.0,
    reconnectDecayTau = 0.060,

    --------------------------------------------------------
    -- Rear wheel / LSD / contact coupling
    --------------------------------------------------------
    rearSlipReleaseGain = 0.55,
    rearContactHoldGain = 0.30,
    rearTrustHoldGain = 0.24,
    lsdHoldGain = 0.22,
    rearDeltaYawGain = 0.35,

    wheelOmegaScale = 80.0,
    rearDiffScale = 28.0,

    --------------------------------------------------------
    -- V1.1 coupling: slip_recovery / yaw_moment_budget
    --------------------------------------------------------
    biteShockGain = 0.55,
    dirtyReturnReleaseGain = 0.42,
    dirtyReturnTorqueLoss = 0.22,
    snapReleaseGain = 0.35,
    gripReturnHoldGain = 0.22,
    recoveryDragGain = 0.36,
    yawBudgetYawGain = 0.20,
    yawBiteYawGain = 0.28,
    yawDirtyDampGain = 0.18,

    memoryRecoveryGain = 0.25,
    recontactShockGain = 0.38,
    carcassHysteresisReleaseGain = 0.24,
    carcassRecoveryBiasGain = 0.18,

    --------------------------------------------------------
    -- Phase thresholds
    --------------------------------------------------------
    lowSpeedKmh = 3.0,
    driveThreshold = 0.08,
    coastThreshold = -0.05,
    windupThreshold = 0.16,
    releaseThreshold = 0.18,
    spinReleaseThreshold = 0.45,
    biteThreshold = 0.32,
    dirtyThreshold = 0.38,

    --------------------------------------------------------
    -- Output shaping
    --------------------------------------------------------
    softTorqueGain = 1.00,
    rearPushGain = 0.75,
    yawHintGain = 0.45,

    --------------------------------------------------------
    -- Runtime safety
    --------------------------------------------------------
    minDt = 0.0005,
    maxDt = 0.100,
    offlineDecayTau = 0.250,

    --------------------------------------------------------
    -- Debug
    --------------------------------------------------------
    debugStoreInterval = 0.25,
}

--============================================================
-- State
--============================================================

M.state = {
    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    phaseId = 0,
    phaseText = "INIT",

    speedKmh = 0.0,
    rpm = 0.0,
    gear = 0,

    throttle = 0.0,
    brake = 0.0,
    clutch = 1.0,
    handbrake = 0.0,

    rawDemand = 0.0,
    demand = 0.0,
    prevDemand = 0.0,
    demandRate = 0.0,

    driveTorqueIn = 0.0,
    torqueSource = "FALLBACK",

    clutchEngagement = 1.0,
    clutchRate = 0.0,
    clutchShock = 0.0,

    windup = 0.0,
    windupVelocity = 0.0,
    windupEnergy = 0.0,
    windupTorque = 0.0,

    shaftTwist = 0.0,
    shaftTwistVelocity = 0.0,
    torsionLoad = 0.0,

    transmittedTorque = 0.0,
    softTorque = 0.0,
    rearPush = 0.0,
    yawHint = 0.0,

    release = 0.0,
    recoveryDrag = 0.0,
    backlash = 0.0,
    backlashShock = 0.0,
    shiftShock = 0.0,
    reconnectShock = 0.0,
    biteShock = 0.0,
    dirtyReturn = 0.0,

    rearOmegaL = 0.0,
    rearOmegaR = 0.0,
    rearOmegaAvg = 0.0,
    rearOmegaDiff = 0.0,
    rearSlip = 0.0,
    rearContact = 1.0,
    rearTrust = 1.0,
    rearGripReturn = 1.0,
    rearSnapRisk = 0.0,
    rearBite = 0.0,
    rearDirtyReturn = 0.0,
    rearRecoveryRate = 0.0,
    rearRecontact = 0.0,

    lsdLock = 0.0,

    yawBudget = 0.0,
    yawBite = 0.0,
    yawDirty = 0.0,
    yawSpinRisk = 0.0,

    prevGear = 0,
    prevClutch = 1.0,
    prevThrottle = 0.0,
    prevTorqueSign = 0,

    linkedDrivetrain = false,
    linkedLSD = false,
    linkedContact = false,
    linkedYawBudget = false,
    linkedSlipRecovery = false,
    linkedMemory = false,
    linkedCarcass = false,

    wheels = {},

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

for i = 0, 3 do
    M.state.wheels[i] = {
        omega = 0.0,
        slipRatio = 0.0,

        contactQuality = 1.0,
        contactTrust = 1.0,
        carcassSupport = 1.0,
        carcassHysteresis = 0.0,
        carcassRecoveryBias = 0.0,

        gripReturn = 1.0,
        snapRisk = 0.0,
        bite = 0.0,
        dirtyReturn = 0.0,
        recoveryRate = 0.0,
        recontact = 0.0,

        memory = 0.0,
        recoveryScalar = 1.0,
        recontactShock = 0.0,

        windupShare = 0.0,
        torqueShare = 0.0,
        releaseShare = 0.0,
        biteShare = 0.0,
        dirtyShare = 0.0,
    }

    M.state[i] = M.state.wheels[i]
end

M.debug = M.state

--============================================================
-- Utility
--============================================================

local function safeNumber(value, defaultValue)
    local n = tonumber(value)
    if n == nil or n ~= n then
        return defaultValue or 0.0
    end
    if n == math.huge or n == -math.huge then
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
    if v > 0.0001 then return 1 end
    if v < -0.0001 then return -1 end
    return 0
end

local function tanhSafe(x)
    x = clamp(x, -20.0, 20.0)
    local e2 = math.exp(2.0 * x)
    return (e2 - 1.0) / (e2 + 1.0)
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.0)
    dt = safeNumber(dt, 0.0)

    if tau <= 0.0 then
        return target
    end

    return current + (target - current) * (dt / math.max(tau + dt, 0.0001))
end

local function smoothstep(edge0, edge1, x)
    if edge0 == edge1 then
        return x >= edge1 and 1.0 or 0.0
    end

    local t = clamp((safeNumber(x, 0.0) - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)
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

local function getWheel(car, i)
    if not car then
        return nil
    end

    local wheels = safeField(car, "wheels", nil)
    if not wheels then
        return nil
    end

    local ok, wheel = pcall(function()
        return wheels[i] or wheels[i + 1]
    end)

    if ok then
        return wheel
    end

    return nil
end

local function getWheelOmega(wheel)
    if not wheel then return 0.0 end

    return safeNumber(
        safeField(wheel, "angularSpeed", nil)
        or nil
        or nil
        or 0.0,
        0.0
    )
end

local function getWheelSlipRatio(wheel, i)
    local stored = safeLoadRaw("ngp_contact_slip_ratio_" .. i)
    if stored ~= nil then
        return abs(stored)
    end

    stored = safeLoadRaw("ngp_slip_ratio_" .. i)
    if stored ~= nil then
        return abs(stored)
    end

    stored = safeLoadRaw("ngp_slip_recovery_slip_" .. i)
    if stored ~= nil then
        return abs(stored)
    end

    if not wheel then
        return 0.0
    end

    return abs(
        safeField(wheel, "slipRatio", nil)
        or nil
        or safeField(wheel, "slip", nil)
        or 0.0
    )
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
-- Input readers
--============================================================

local function readInputs(car)
    local st = M.state

    st.speedKmh = getSpeedKmh(car)
    st.rpm = safeNumber(safeField(car, "rpm", 0.0), 0.0)
    st.gear = math.floor(safeNumber(safeField(car, "gear", 0.0), 0.0))

    st.throttle = clamp(
        safeNumber(
            safeField(car, "gas", 0.0),
            0.0
        ),
        0.0,
        1.0
    )

    st.brake = clamp(safeNumber(safeField(car, "brake", 0.0), 0.0), 0.0, 1.0)
    st.handbrake = clamp(safeNumber(safeField(car, "handbrake", 0.0), 0.0), 0.0, 1.0)

    -- Clutch semantics vary between AC/CSP environments. Use it mainly for rate/shift detection.
    st.clutch = clamp(safeNumber(safeField(car, "clutch", 1.0), 1.0), 0.0, 1.0)
end

local function readLinkedSignals()
    local st = M.state

    st.linkedDrivetrain = false
    st.linkedLSD = false
    st.linkedYawBudget = false
    st.linkedSlipRecovery = false
    st.linkedMemory = false
    st.linkedCarcass = false

    -- Avoid self-feedback. This module exports ngp_drive_torque and
    -- ngp_drive_transmitted_torque for later modules, so it should only read
    -- those names when an external drivetrain module is explicitly running.
    local externalDrivetrain =
        safeLoadRaw("ngp_drivetrain_update_count") ~= nil
        or safeLoadRaw("ngp_drivetrain_status") ~= nil
        or safeLoadRaw("ngp_dt_update_count") ~= nil

    local torque = 0.0
    local torqueKey = nil

    torque, torqueKey = loadFirst(
        0.0,
        "ngp_drivetrain_transmitted_torque",
        "ngp_drivetrain_drive_torque",
        "ngp_dt_transmitted_torque",
        "ngp_dt_torque",
        "ngp_flywheel_drive_torque",
        "ngp_engine_drive_torque"
    )

    if torqueKey == nil and externalDrivetrain then
        torque, torqueKey = loadFirst(
            0.0,
            "ngp_drive_transmitted_torque",
            "ngp_drive_torque"
        )
    end

    if torqueKey ~= nil then
        st.linkedDrivetrain = true
        st.torqueSource = torqueKey
    else
        st.torqueSource = "FALLBACK"
    end

    local lsdLock, lsdKey = loadFirst(
        0.0,
        "ngp_lsd_lock",
        "ngp_diff_lock",
        "ngp_windup_lsd_lock"
    )

    if lsdKey ~= nil then
        st.linkedLSD = true
    end

    if lsdLock > 1.5 then
        lsdLock = lsdLock / 100.0
    end

    st.lsdLock = clamp(lsdLock, 0.0, 1.0)
    st.driveTorqueIn = torque

    local yawBudget = safeLoadRaw("ngp_yaw_budget")
    if yawBudget ~= nil then
        st.linkedYawBudget = true
    end

    st.yawBudget = clamp(safeNumber(yawBudget, 0.0), -1.0, 1.0)
    st.yawBite = clamp(safeLoad("ngp_yaw_bite", 0.0), 0.0, 1.0)
    st.yawDirty = clamp(safeLoad("ngp_yaw_dirty_return", safeLoad("ngp_yaw_dirty_energy", 0.0)), 0.0, 1.0)
    st.yawSpinRisk = clamp(safeLoad("ngp_yaw_spin_risk", 0.0), 0.0, 1.0)

    if safeLoadRaw("ngp_yaw_bite") ~= nil
    or safeLoadRaw("ngp_yaw_dirty_return") ~= nil
    or safeLoadRaw("ngp_yaw_dirty_energy") ~= nil
    or safeLoadRaw("ngp_yaw_spin_risk") ~= nil then
        st.linkedYawBudget = true
    end
end

local function readWheelSignals(car)
    local st = M.state

    st.linkedContact = false

    local rearContactSum = 0.0
    local rearTrustSum = 0.0
    local rearSlipSum = 0.0
    local rearGripReturnSum = 0.0
    local rearSnapSum = 0.0
    local rearBiteSum = 0.0
    local rearDirtySum = 0.0
    local rearRecoverySum = 0.0
    local rearRecontactSum = 0.0

    for i = 0, 3 do
        local wheel = getWheel(car, i)
        local ws = st.wheels[i]

        ws.omega = getWheelOmega(wheel)
        ws.slipRatio = getWheelSlipRatio(wheel, i)

        local cq = safeLoadRaw("ngp_contact_quality_" .. i)
        if cq == nil then cq = safeLoadRaw("ngp_tire_contact_quality_" .. i) end
        if cq == nil then cq = safeLoadRaw("ngp_tcr_quality_" .. i) end

        local trust = safeLoadRaw("ngp_contact_trust_" .. i)
        if trust == nil then trust = safeLoadRaw("ngp_tire_contact_trust_" .. i) end

        local loss = safeLoadRaw("ngp_contact_loss_" .. i)
        if loss == nil then loss = safeLoadRaw("ngp_tire_contact_loss_" .. i) end
        if loss == nil then loss = safeLoadRaw("ngp_tcr_contact_loss_" .. i) end

        if cq ~= nil or trust ~= nil or loss ~= nil then
            st.linkedContact = true
        end

        if cq == nil and loss ~= nil then
            cq = 1.0 - safeNumber(loss, 0.0)
        end

        ws.contactQuality = clamp(safeNumber(cq, 1.0), 0.0, 1.2)
        ws.contactTrust = clamp(safeNumber(trust, ws.contactQuality), 0.0, 1.2)

        local support = safeLoadRaw("ngp_carcass_support_" .. i)
        local hyst = safeLoadRaw("ngp_carcass_hysteresis_" .. i)
        local recoveryBias = safeLoadRaw("ngp_carcass_recovery_bias_" .. i)

        if support ~= nil or hyst ~= nil or recoveryBias ~= nil then
            st.linkedCarcass = true
        end

        ws.carcassSupport = clamp(safeNumber(support, 1.0), 0.0, 1.2)
        ws.carcassHysteresis = clamp(safeNumber(hyst, 0.0), 0.0, 1.0)
        ws.carcassRecoveryBias = clamp(safeNumber(recoveryBias, 0.0), 0.0, 1.0)

        local gripReturn = safeLoadRaw("ngp_slip_grip_return_" .. i)
        local snapRisk = safeLoadRaw("ngp_slip_snap_risk_" .. i)
        local bite = safeLoadRaw("ngp_slip_bite_" .. i)
        local dirtyReturn = safeLoadRaw("ngp_slip_dirty_return_" .. i)
        local recoveryRate = safeLoadRaw("ngp_slip_recovery_rate_" .. i)
        local recontact = safeLoadRaw("ngp_slip_recontact_" .. i)
        if recontact == nil then recontact = safeLoadRaw("ngp_slip_recovery_recontact_" .. i) end

        if gripReturn ~= nil or snapRisk ~= nil or bite ~= nil or dirtyReturn ~= nil or recoveryRate ~= nil or recontact ~= nil then
            st.linkedSlipRecovery = true
        end

        ws.gripReturn = clamp(safeNumber(gripReturn, 1.0), 0.0, 1.2)
        ws.snapRisk = clamp(safeNumber(snapRisk, 0.0), 0.0, 1.0)
        ws.bite = clamp(safeNumber(bite, 0.0), 0.0, 1.0)
        ws.dirtyReturn = clamp(safeNumber(dirtyReturn, 0.0), 0.0, 1.0)
        ws.recoveryRate = clamp(safeNumber(recoveryRate, 0.0), 0.0, 1.0)
        ws.recontact = clamp(safeNumber(recontact, 0.0), 0.0, 1.0)

        local mem = safeLoadRaw("ngp_tire_memory_" .. i)
        if mem == nil then mem = safeLoadRaw("ngp_memory_" .. i) end

        local recScalar = safeLoadRaw("ngp_memory_recovery_scalar_" .. i)
        if recScalar == nil then recScalar = safeLoadRaw("ngp_tire_memory_recovery_scalar_" .. i) end

        local recShock = safeLoadRaw("ngp_memory_recontact_shock_" .. i)

        if mem ~= nil or recScalar ~= nil or recShock ~= nil then
            st.linkedMemory = true
        end

        ws.memory = clamp(safeNumber(mem, 0.0), 0.0, 1.0)
        ws.recoveryScalar = clamp(safeNumber(recScalar, 1.0), 0.0, 1.2)
        ws.recontactShock = clamp(safeNumber(recShock, 0.0), 0.0, 1.0)

        if i >= 2 then
            rearContactSum = rearContactSum + ws.contactQuality
            rearTrustSum = rearTrustSum + ws.contactTrust
            rearSlipSum = rearSlipSum + ws.slipRatio
            rearGripReturnSum = rearGripReturnSum + ws.gripReturn
            rearSnapSum = rearSnapSum + ws.snapRisk
            rearBiteSum = rearBiteSum + ws.bite
            rearDirtySum = rearDirtySum + ws.dirtyReturn
            rearRecoverySum = rearRecoverySum + ws.recoveryRate
            rearRecontactSum = rearRecontactSum + ws.recontact
        end
    end

    st.rearOmegaL = st.wheels[2].omega or 0.0
    st.rearOmegaR = st.wheels[3].omega or 0.0
    st.rearOmegaAvg = (st.rearOmegaL + st.rearOmegaR) * 0.5
    st.rearOmegaDiff = st.rearOmegaR - st.rearOmegaL

    st.rearContact = clamp(rearContactSum * 0.5, 0.0, 1.2)
    st.rearTrust = clamp(rearTrustSum * 0.5, 0.0, 1.2)
    st.rearSlip = clamp(rearSlipSum * 0.5, 0.0, 2.0)
    st.rearGripReturn = clamp(rearGripReturnSum * 0.5, 0.0, 1.2)
    st.rearSnapRisk = clamp(rearSnapSum * 0.5, 0.0, 1.0)
    st.rearBite = clamp(rearBiteSum * 0.5, 0.0, 1.0)
    st.rearDirtyReturn = clamp(rearDirtySum * 0.5, 0.0, 1.0)
    st.rearRecoveryRate = clamp(rearRecoverySum * 0.5, 0.0, 1.0)
    st.rearRecontact = clamp(rearRecontactSum * 0.5, 0.0, 1.0)
end

--============================================================
-- Model
--============================================================

local function updateClutchModel(dt)
    local st = M.state
    local p = M.params

    local target = st.clutch

    -- During neutral/shift, reduce engagement gently. Keep gamepad AutoClutch stable.
    if st.gear == 0 then
        target = target * 0.35
    end

    if st.shiftShock > 0.05 then
        target = target * (1.0 - clamp(st.shiftShock * 0.28, 0.0, 0.45))
    end

    target = clamp(target, 0.0, 1.0)

    local prev = st.clutchEngagement or target
    local tau = 1.0 / math.max(p.clutchEngageRate * 10.0, 0.001)
    st.clutchEngagement = lowPass(prev, target, tau, dt)
    st.clutchRate = (st.clutchEngagement - prev) / math.max(dt, 0.001)

    local releaseSpike = math.max(0.0, st.clutchRate - p.clutchReleaseRate * 0.35)
    local targetShock = clamp(releaseSpike * p.clutchShockGain + st.shiftShock * 0.22, 0.0, 1.0)
    st.clutchShock = lowPass(st.clutchShock, targetShock, 0.070, dt)
end

local function calculateRawDemand()
    local st = M.state
    local p = M.params

    local torqueNorm = 0.0

    if st.linkedDrivetrain and math.abs(st.driveTorqueIn) > 0.0001 then
        -- Existing drivetrain outputs may use different scales, normalize conservatively.
        torqueNorm = tanhSafe((st.driveTorqueIn or 0.0) * 0.01) * p.torqueScale
    else
        -- Fallback: build safe drive demand from throttle/brake/coast.
        torqueNorm =
            (st.throttle * 1.00)
            - (st.brake * 0.22)
            - (st.handbrake * 0.08)

        if st.throttle < 0.02 and st.gear ~= 0 and st.speedKmh > p.lowSpeedKmh then
            torqueNorm = torqueNorm - p.engineBrakeGain * clamp(st.speedKmh / 120.0, 0.0, 1.0)
        end

        torqueNorm = torqueNorm * p.fallbackTorqueScale
    end

    -- Keep neutral/near-stop demand conservative.
    if st.gear == 0 then
        torqueNorm = torqueNorm * 0.15
    end

    -- Treat clutch as a soft engagement factor, not as an absolute truth.
    local clutchFactor = 0.20 + 0.80 * clamp(st.clutchEngagement, 0.0, 1.0)
    torqueNorm = torqueNorm * clutchFactor

    return clamp(torqueNorm, -1.0, 1.0)
end

local function detectEvents(dt)
    local st = M.state
    local p = M.params

    local gearChanged = st.gear ~= st.prevGear and st.prevGear ~= 0
    local clutchRate = (st.clutch - st.prevClutch) / math.max(dt, 0.001)
    local throttleRate = (st.throttle - st.prevThrottle) / math.max(dt, 0.001)

    local reconnect = 0.0

    if throttleRate > p.throttleReapplyThreshold and st.throttle > p.clutchKickThrottle then
        reconnect = reconnect + clamp(throttleRate * 0.10, 0.0, 1.0)
    end

    if clutchRate > p.clutchReleaseRate and st.throttle > p.clutchKickThrottle then
        reconnect = reconnect + clamp(clutchRate * 0.12, 0.0, 1.0)
    end

    reconnect = reconnect + st.rearRecontact * p.recontactShockGain
    reconnect = reconnect + ((st.wheels[2].recontactShock or 0.0) + (st.wheels[3].recontactShock or 0.0)) * 0.5 * p.recontactShockGain

    st.reconnectShock = lowPass(
        st.reconnectShock,
        clamp(reconnect, 0.0, 1.0),
        p.reconnectDecayTau,
        dt
    )

    if gearChanged then
        st.shiftShock = math.min(1.0, (st.shiftShock or 0.0) + p.shiftShockGain)
    else
        st.shiftShock = math.max(0.0, (st.shiftShock or 0.0) - dt * p.shiftShockDecay)
    end
end

local function updateWindup(dt)
    local st = M.state
    local p = M.params

    updateClutchModel(dt)

    local rawDemand = calculateRawDemand()

    st.rawDemand = rawDemand

    local inputTau = p.inputTauDrive
    if rawDemand < st.demand then
        inputTau = p.inputTauCoast
    end
    if sign(rawDemand) ~= 0 and sign(rawDemand) ~= sign(st.demand) then
        inputTau = p.inputTauReverse
    end

    st.prevDemand = st.demand or 0.0
    st.demand = lowPass(st.demand, rawDemand, inputTau, dt)
    st.demandRate = (st.demand - st.prevDemand) / math.max(dt, 0.001)

    local rearOmegaNorm = clamp(math.abs(st.rearOmegaAvg) / math.max(p.wheelOmegaScale, 1.0), 0.0, 1.5)
    local rearDiffNorm = clamp(math.abs(st.rearOmegaDiff) / math.max(p.rearDiffScale, 1.0), 0.0, 1.5)

    local rearSlipRelease = clamp(st.rearSlip * p.rearSlipReleaseGain, 0.0, 1.0)
    local snapRelease = clamp(st.rearSnapRisk * p.snapReleaseGain, 0.0, 1.0)
    local dirtyRelease = clamp(st.rearDirtyReturn * p.dirtyReturnReleaseGain, 0.0, 1.0)
    local hystRelease = clamp(((st.wheels[2].carcassHysteresis or 0.0) + (st.wheels[3].carcassHysteresis or 0.0)) * 0.5 * p.carcassHysteresisReleaseGain, 0.0, 1.0)

    local contactHold = clamp(st.rearContact * p.rearContactHoldGain, 0.0, 0.60)
    local trustHold = clamp(st.rearTrust * p.rearTrustHoldGain, 0.0, 0.45)
    local gripHold = clamp(st.rearGripReturn * p.gripReturnHoldGain, 0.0, 0.30)
    local lsdHold = clamp(st.lsdLock * p.lsdHoldGain, 0.0, 0.45)

    -- Accumulate difference between drive demand and rear response as windup.
    -- Rear slip, poor contact, and dirty return release stored energy.
    local targetWindup =
        st.demand
        * (1.0 + contactHold + trustHold + gripHold + lsdHold)
        * (1.0 - rearSlipRelease * 0.55)
        * (1.0 - dirtyRelease * 0.28)

    if st.speedKmh < p.lowSpeedKmh then
        targetWindup = targetWindup * clamp(st.speedKmh / math.max(p.lowSpeedKmh, 0.001), 0.20, 1.0)
    end

    local spring = (targetWindup - st.windup) * p.windupStiffness
    local damper = -st.windupVelocity * p.windupDamping
    local releaseForce = (rearSlipRelease + snapRelease + dirtyRelease + hystRelease) * p.windupReleaseGain * sign(st.windup)

    local accel = spring + damper - releaseForce

    st.windupVelocity = clamp(
        st.windupVelocity + accel * dt,
        -p.maxWindupVelocity,
        p.maxWindupVelocity
    )

    st.windup = clamp(
        st.windup + st.windupVelocity * dt,
        -p.maxWindup,
        p.maxWindup
    )

    st.windupEnergy = clamp(math.abs(st.windup), 0.0, 1.0)
    st.windupTorque = clamp(st.windup * p.softTorqueGain, -1.0, 1.0)

    -- shaftTwist is a compatibility signal shaped from windup.
    local shaftTarget = clamp(st.windup * p.shaftTwistGain + st.demandRate * 0.015, -p.maxShaftTwist, p.maxShaftTwist)
    local prevShaftTwist = st.shaftTwist or 0.0
    st.shaftTwist = lowPass(st.shaftTwist, shaftTarget, 0.070, dt)
    st.shaftTwistVelocity = lowPass(st.shaftTwistVelocity, (st.shaftTwist - prevShaftTwist) / math.max(dt, 0.001), 0.120, dt)
    st.torsionLoad = clamp(math.abs(st.shaftTwist) * p.shaftTorsionGain + st.windupEnergy * 0.40, 0.0, 1.0)

    -- Release rises when stored torque escapes through slip, demand drop, or dirty return.
    local demandDrop = math.max(0.0, -st.demandRate * 0.08)
    local recoveryBias = ((st.wheels[2].carcassRecoveryBias or 0.0) + (st.wheels[3].carcassRecoveryBias or 0.0)) * 0.5
    local recoveryDragTarget = clamp(
        st.rearDirtyReturn * p.recoveryDragGain
        + (1.0 - st.rearGripReturn) * 0.28
        + recoveryBias * p.carcassRecoveryBiasGain
        + (1.0 - ((st.wheels[2].recoveryScalar or 1.0) + (st.wheels[3].recoveryScalar or 1.0)) * 0.5) * p.memoryRecoveryGain,
        0.0,
        1.0
    )

    st.recoveryDrag = lowPass(st.recoveryDrag, recoveryDragTarget, 0.100, dt)

    local releaseTarget = clamp(
        st.windupEnergy * (rearSlipRelease + demandDrop + rearDiffNorm * 0.15 + st.recoveryDrag + snapRelease * 0.60),
        0.0,
        1.0
    )

    st.release = lowPass(st.release, releaseTarget, 0.070, dt)

    -- Bite shock: sudden grip return after slip.
    local biteTarget = clamp(
        st.rearBite * p.biteShockGain
        + st.yawBite * p.yawBiteYawGain
        + st.reconnectShock * 0.25
        + st.shiftShock * 0.12,
        0.0,
        1.0
    )

    st.biteShock = lowPass(st.biteShock, biteTarget, 0.060, dt)

    st.dirtyReturn = lowPass(
        st.dirtyReturn,
        clamp(st.rearDirtyReturn * 0.70 + st.yawDirty * 0.30 + st.recoveryDrag * 0.25, 0.0, 1.0),
        0.120,
        dt
    )

    -- Backlash increases on torque reversal, reconnect, and zero crossing.
    local torqueSign = sign(st.demand)
    local reverseEvent =
        torqueSign ~= 0
        and st.prevTorqueSign ~= 0
        and torqueSign ~= st.prevTorqueSign

    local backlashTarget = 0.0

    if reverseEvent and math.abs(st.demand) > p.torqueReverseThreshold then
        backlashTarget = 1.0
    elseif math.abs(st.demand) < p.backlashThreshold then
        backlashTarget = 0.45
    end

    backlashTarget = math.max(backlashTarget, st.reconnectShock * 0.65, st.shiftShock * 0.55, st.biteShock * 0.22)

    if backlashTarget > st.backlash then
        st.backlash = lowPass(st.backlash, backlashTarget, 1.0 / math.max(p.backlashBuildGain, 0.001), dt)
    else
        st.backlash = math.max(0.0, st.backlash - dt * p.backlashDecayGain)
    end

    st.backlashShock = clamp(st.backlash * p.backlashShockGain + st.reconnectShock * 0.45 + st.shiftShock * 0.35 + st.biteShock * 0.25, 0.0, 1.0)

    -- Output is softened transmitted torque after windup, clutch, backlash, and return drag.
    local demandSign = sign(st.demand)
    st.transmittedTorque = clamp(
        st.demand
        - st.release * demandSign
        - st.backlash * 0.10 * demandSign
        - st.recoveryDrag * 0.08 * demandSign,
        -1.0,
        1.0
    )

    local dirtyLoss = 1.0 - clamp(st.dirtyReturn * p.dirtyReturnTorqueLoss, 0.0, 0.45)
    st.softTorque = clamp((st.transmittedTorque + st.windupTorque * 0.35) * dirtyLoss, -1.0, 1.0)

    -- Rear push trunk signal.
    st.rearPush = clamp(
        math.max(0.0, st.softTorque)
        * p.rearPushGain
        * (0.70 + st.lsdLock * 0.30)
        * (0.55 + st.rearTrust * 0.45)
        * (1.0 - st.dirtyReturn * 0.20),
        0.0,
        1.0
    )

    -- Drive-derived yaw tendency from rear wheel speed difference.
    local yawFromDiff =
        (st.rearOmegaDiff / math.max(p.rearDiffScale, 1.0))
        * p.rearDeltaYawGain

    local yawFromDrive =
        st.rearPush
        * st.lsdLock
        * p.yawHintGain
        * sign(st.rearOmegaDiff)

    local yawBudgetPart = st.yawBudget * p.yawBudgetYawGain
    local dirtyDamp = 1.0 - clamp(st.dirtyReturn * p.yawDirtyDampGain, 0.0, 0.35)

    st.yawHint = clamp((yawFromDiff + yawFromDrive + yawBudgetPart) * dirtyDamp, -1.0, 1.0)

    if torqueSign ~= 0 then
        st.prevTorqueSign = torqueSign
    end

    -- Keep rearOmegaNorm for debug/failsafe reference.
    st._rearOmegaNorm = rearOmegaNorm
end

local function classifyPhase()
    local st = M.state
    local p = M.params

    if st.speedKmh < p.lowSpeedKmh and math.abs(st.demand) < p.driveThreshold then
        return 1, "LOW_SPEED"
    end

    if st.gear == 0 then
        return 2, "NEUTRAL"
    end

    if st.shiftShock > 0.20 then
        return 8, "SHIFT_SHOCK"
    end

    if st.reconnectShock > 0.25 then
        return 7, "RECONNECT"
    end

    if st.biteShock > p.biteThreshold then
        return 11, "BITE_RECONTACT"
    end

    if st.dirtyReturn > p.dirtyThreshold then
        return 12, "DIRTY_RETURN"
    end

    if st.backlash > 0.35 then
        return 6, "BACKLASH"
    end

    if st.release > p.releaseThreshold or st.rearSlip > p.spinReleaseThreshold then
        return 5, "RELEASE"
    end

    if math.abs(st.windup) > p.windupThreshold then
        if st.windup > 0.0 then
            return 4, "WINDUP_DRIVE"
        else
            return 9, "WINDUP_COAST"
        end
    end

    if st.demand > p.driveThreshold then
        return 3, "TAKEUP"
    end

    if st.demand < p.coastThreshold or st.brake > 0.08 then
        return 10, "COAST_DRAG"
    end

    return 0, "IDLE"
end

local function updateWheelShares()
    local st = M.state

    for i = 0, 3 do
        local ws = st.wheels[i]

        if i >= 2 then
            local sideSign = i == 2 and -1.0 or 1.0
            local diffShare = clamp(st.rearOmegaDiff / 60.0, -1.0, 1.0) * sideSign

            ws.windupShare = clamp(st.windupEnergy * (0.45 + 0.35 * ws.contactTrust + 0.20 * ws.carcassSupport), 0.0, 1.0)
            ws.torqueShare = clamp(st.softTorque * 0.5 + diffShare * st.lsdLock * 0.20, -1.0, 1.0)
            ws.releaseShare = clamp(st.release * (0.70 + ws.slipRatio * 0.20 + ws.snapRisk * 0.25), 0.0, 1.0)
            ws.biteShare = clamp(st.biteShock * (0.45 + ws.bite * 0.55), 0.0, 1.0)
            ws.dirtyShare = clamp(st.dirtyReturn * (0.50 + ws.dirtyReturn * 0.50), 0.0, 1.0)
        else
            ws.windupShare = 0.0
            ws.torqueShare = 0.0
            ws.releaseShare = 0.0
            ws.biteShare = 0.0
            ws.dirtyShare = 0.0
        end
    end
end


local function decayOffline(dt)
    local st = M.state
    local tau = M.params.offlineDecayTau or 0.250

    st.rawDemand = lowPass(st.rawDemand, 0.0, tau, dt)
    st.demand = lowPass(st.demand, 0.0, tau, dt)
    st.demandRate = 0.0
    st.windup = lowPass(st.windup, 0.0, tau, dt)
    st.windupVelocity = lowPass(st.windupVelocity, 0.0, tau, dt)
    st.windupEnergy = lowPass(st.windupEnergy, 0.0, tau, dt)
    st.windupTorque = lowPass(st.windupTorque, 0.0, tau, dt)
    st.shaftTwist = lowPass(st.shaftTwist, 0.0, tau, dt)
    st.shaftTwistVelocity = lowPass(st.shaftTwistVelocity, 0.0, tau, dt)
    st.torsionLoad = lowPass(st.torsionLoad, 0.0, tau, dt)
    st.transmittedTorque = lowPass(st.transmittedTorque, 0.0, tau, dt)
    st.softTorque = lowPass(st.softTorque, 0.0, tau, dt)
    st.rearPush = lowPass(st.rearPush, 0.0, tau, dt)
    st.yawHint = lowPass(st.yawHint, 0.0, tau, dt)
    st.release = lowPass(st.release, 0.0, tau, dt)
    st.recoveryDrag = lowPass(st.recoveryDrag, 0.0, tau, dt)
    st.backlash = lowPass(st.backlash, 0.0, tau, dt)
    st.backlashShock = lowPass(st.backlashShock, 0.0, tau, dt)
    st.shiftShock = lowPass(st.shiftShock, 0.0, tau, dt)
    st.reconnectShock = lowPass(st.reconnectShock, 0.0, tau, dt)
    st.biteShock = lowPass(st.biteShock, 0.0, tau, dt)
    st.dirtyReturn = lowPass(st.dirtyReturn, 0.0, tau, dt)
    st.clutchShock = lowPass(st.clutchShock, 0.0, tau, dt)

    for i = 0, 3 do
        local ws = st.wheels[i]
        ws.windupShare = lowPass(ws.windupShare, 0.0, tau, dt)
        ws.torqueShare = lowPass(ws.torqueShare, 0.0, tau, dt)
        ws.releaseShare = lowPass(ws.releaseShare, 0.0, tau, dt)
        ws.biteShare = lowPass(ws.biteShare, 0.0, tau, dt)
        ws.dirtyShare = lowPass(ws.dirtyShare, 0.0, tau, dt)
    end
end

--============================================================
-- Export
--============================================================

local function exportWheel(index, ws)
    safeStore("ngp_windup_wheel_omega_" .. index, ws.omega or 0.0)
    safeStore("ngp_windup_wheel_slip_" .. index, ws.slipRatio or 0.0)
    safeStore("ngp_windup_wheel_contact_" .. index, ws.contactQuality or 0.0)
    safeStore("ngp_windup_wheel_trust_" .. index, ws.contactTrust or 0.0)
    safeStore("ngp_windup_wheel_energy_" .. index, ws.windupShare or 0.0)
    safeStore("ngp_windup_wheel_torque_" .. index, ws.torqueShare or 0.0)
    safeStore("ngp_windup_wheel_release_" .. index, ws.releaseShare or 0.0)
    safeStore("ngp_windup_wheel_bite_" .. index, ws.biteShare or 0.0)
    safeStore("ngp_windup_wheel_dirty_" .. index, ws.dirtyShare or 0.0)
end

local function exportState()
    local st = M.state

    safeStore("ngp_windup_status", st.status or "UNKNOWN")
    safeStore("ngp_windup_update_count", st.updateCount or 0)
    safeStore("ngp_windup_wheels_valid", st.wheelsValid and 1 or 0)
    safeStore("ngp_windup_phase", st.phaseText or "UNKNOWN")
    safeStore("ngp_windup_phase_id", st.phaseId or 0)

    safeStore("ngp_windup_demand", st.demand or 0.0)
    safeStore("ngp_windup_raw_demand", st.rawDemand or 0.0)
    safeStore("ngp_windup_demand_rate", st.demandRate or 0.0)

    safeStore("ngp_windup_value", st.windup or 0.0)
    safeStore("ngp_windup_velocity", st.windupVelocity or 0.0)
    safeStore("ngp_windup_energy", st.windupEnergy or 0.0)
    safeStore("ngp_windup_torque", st.windupTorque or 0.0)

    safeStore("ngp_windup_shaft_twist", st.shaftTwist or 0.0)
    safeStore("ngp_windup_torsion_load", st.torsionLoad or 0.0)

    safeStore("ngp_windup_transmitted_torque", st.transmittedTorque or 0.0)
    safeStore("ngp_drive_soft_torque", st.softTorque or 0.0)
    safeStore("ngp_drive_soft_rear_push", st.rearPush or 0.0)
    safeStore("ngp_drive_soft_yaw", st.yawHint or 0.0)

    safeStore("ngp_windup_release", st.release or 0.0)
    safeStore("ngp_windup_recovery_drag", st.recoveryDrag or 0.0)
    safeStore("ngp_windup_backlash", st.backlash or 0.0)
    safeStore("ngp_windup_backlash_shock", st.backlashShock or 0.0)
    safeStore("ngp_windup_shift_shock", st.shiftShock or 0.0)
    safeStore("ngp_windup_reconnect_shock", st.reconnectShock or 0.0)
    safeStore("ngp_windup_bite", st.biteShock or 0.0)
    safeStore("ngp_windup_dirty_return", st.dirtyReturn or 0.0)
    safeStore("ngp_windup_clutch_engagement", st.clutchEngagement or 0.0)
    safeStore("ngp_windup_clutch_shock", st.clutchShock or 0.0)

    safeStore("ngp_windup_rear_omega_avg", st.rearOmegaAvg or 0.0)
    safeStore("ngp_windup_rear_omega_diff", st.rearOmegaDiff or 0.0)
    safeStore("ngp_windup_rear_slip", st.rearSlip or 0.0)
    safeStore("ngp_windup_rear_contact", st.rearContact or 0.0)
    safeStore("ngp_windup_rear_trust", st.rearTrust or 0.0)
    safeStore("ngp_windup_rear_bite", st.rearBite or 0.0)
    safeStore("ngp_windup_rear_dirty", st.rearDirtyReturn or 0.0)
    safeStore("ngp_windup_rear_snap", st.rearSnapRisk or 0.0)
    safeStore("ngp_windup_lsd_lock", st.lsdLock or 0.0)

    -- short aliases for trunk modules / existing readers
    safeStore("ngp_driveline_windup", st.windup or 0.0)
    safeStore("ngp_driveline_energy", st.windupEnergy or 0.0)
    safeStore("ngp_driveline_release", st.release or 0.0)
    safeStore("ngp_driveline_backlash", st.backlash or 0.0)
    safeStore("ngp_driveline_rear_push", st.rearPush or 0.0)
    safeStore("ngp_driveline_yaw_hint", st.yawHint or 0.0)
    safeStore("ngp_driveline_twist", st.shaftTwist or 0.0)
    safeStore("ngp_driveline_shaft_velocity", st.shaftTwistVelocity or 0.0)
    safeStore("ngp_driveline_lash", st.backlash or 0.0)
    safeStore("ngp_driveline_lash_shock", st.backlashShock or 0.0)
    safeStore("ngp_driveline_shift_shock", st.shiftShock or 0.0)
    safeStore("ngp_driveline_reconnect_shock", st.reconnectShock or 0.0)
    safeStore("ngp_driveline_bite", st.biteShock or 0.0)
    safeStore("ngp_driveline_dirty_return", st.dirtyReturn or 0.0)
    safeStore("ngp_driveline_torque_source", st.torqueSource or "UNKNOWN")

    -- keys read by yaw_moment_budget / tyre_memory
    safeStore("ngp_drive_torque", st.softTorque or 0.0)
    safeStore("ngp_drive_transmitted_torque", st.transmittedTorque or 0.0)
    safeStore("ngp_shaft_twist", st.shaftTwist or 0.0)
    safeStore("ngp_shaft_velocity", st.shaftTwistVelocity or 0.0)
    safeStore("ngp_drive_lash", st.backlash or 0.0)
    safeStore("ngp_drive_lash_shock", st.backlashShock or 0.0)
    safeStore("ngp_shift_shock", st.shiftShock or 0.0)
    safeStore("ngp_drive_contact_loss", clamp(1.0 - (st.rearTrust or 1.0), 0.0, 1.0))
    safeStore("ngp_drive_hop_shock", st.reconnectShock or 0.0)
    safeStore("ngp_drive_soft_rear_push", st.rearPush or 0.0)
    safeStore("ngp_drive_soft_yaw_hint", st.yawHint or 0.0)

    if st.debugStoreNow then
        safeStore("ngp_windup_torque_source", st.torqueSource or "UNKNOWN")
        safeStore("ngp_windup_link_drivetrain", st.linkedDrivetrain and 1 or 0)
        safeStore("ngp_windup_link_lsd", st.linkedLSD and 1 or 0)
        safeStore("ngp_windup_link_contact", st.linkedContact and 1 or 0)
        safeStore("ngp_windup_link_yaw_budget", st.linkedYawBudget and 1 or 0)
        safeStore("ngp_windup_link_slip_recovery", st.linkedSlipRecovery and 1 or 0)
        safeStore("ngp_windup_link_memory", st.linkedMemory and 1 or 0)
        safeStore("ngp_windup_link_carcass", st.linkedCarcass and 1 or 0)

        safeStore("ngp_windup_throttle", st.throttle or 0.0)
        safeStore("ngp_windup_brake", st.brake or 0.0)
        safeStore("ngp_windup_clutch", st.clutch or 0.0)
        safeStore("ngp_windup_gear", st.gear or 0)
        safeStore("ngp_windup_speed_kmh", st.speedKmh or 0.0)
        safeStore("ngp_windup_yaw_budget", st.yawBudget or 0.0)
    end

    for i = 0, 3 do
        exportWheel(i, st.wheels[i])
    end
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
    dt = clamp(dt, M.params.minDt or 0.0005, M.params.maxDt or 0.100)

    updateDebugGate(dt)

    if not car and ac and ac.getCar then
        local ok, value = pcall(function()
            return ac.getCar(0)
        end)
        if ok then
            car = value
        end
    end

    if not car then
        M.state.status = "NO CAR"
        M.state.wheelsValid = false
        decayOffline(dt)
        exportState()
        return
    end

    if not safeField(car, "wheels", nil) then
        M.state.status = "NO WHEELS"
        M.state.wheelsValid = false
        decayOffline(dt)
        exportState()
        return
    end

    M.state.status = "RUNNING"
    M.state.wheelsValid = true

    readInputs(car)
    readLinkedSignals()
    readWheelSignals(car)
    detectEvents(dt)
    updateWindup(dt)
    updateWheelShares()

    local phaseId, phaseText = classifyPhase()
    M.state.phaseId = phaseId
    M.state.phaseText = phaseText

    M.state.prevGear = M.state.gear
    M.state.prevClutch = M.state.clutch
    M.state.prevThrottle = M.state.throttle

    exportState()
end

--============================================================
-- Public API
--============================================================

function M.getWindup()
    return M.state.windup or 0.0
end

function M.getEnergy()
    return M.state.windupEnergy or 0.0
end

function M.getSoftTorque()
    return M.state.softTorque or 0.0
end

function M.getRearPush()
    return M.state.rearPush or 0.0
end

function M.getRelease()
    return M.state.release or 0.0
end

function M.getBacklash()
    return M.state.backlash or 0.0
end

function M.getShaftTwist()
    return M.state.shaftTwist or 0.0
end

function M.getYawHint()
    return M.state.yawHint or 0.0
end

function M.getState()
    return M.state
end

function M.debugStr(index)
    local st = M.state

    if index ~= nil then
        local i = tonumber(index) or 0
        local ws = st.wheels[i] or st.wheels[0]

        return string.format(
            "%s %s / W %.3f / T %+.3f / R %.3f\n" ..
            "Omega %.2f / Slip %.3f / CQ %.2f Trust %.2f\n" ..
            "Bite %.2f Dirty %.2f",
            tostring(WHEEL_NAMES[i] or i),
            tostring(st.phaseText),
            ws.windupShare or 0.0,
            ws.torqueShare or 0.0,
            ws.releaseShare or 0.0,
            ws.omega or 0.0,
            ws.slipRatio or 0.0,
            ws.contactQuality or 0.0,
            ws.contactTrust or 0.0,
            ws.biteShare or 0.0,
            ws.dirtyShare or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Phase %s\n" ..
        "Demand %+.3f / Windup %+.3f / Energy %.3f / SoftT %+.3f\n" ..
        "Release %.3f / Backlash %.3f / Shift %.3f / Reconnect %.3f\n" ..
        "Bite %.3f / Dirty %.3f / Drag %.3f / Clutch %.3f\n" ..
        "RearPush %.3f / YawHint %+.3f / ROmega %.2f d%.2f\n" ..
        "Links DT:%s LSD:%s CQ:%s YMB:%s SR:%s MEM:%s CS:%s Source:%s",
        tostring(st.status),
        st.updateCount or 0,
        tostring(st.phaseText),
        st.demand or 0.0,
        st.windup or 0.0,
        st.windupEnergy or 0.0,
        st.softTorque or 0.0,
        st.release or 0.0,
        st.backlash or 0.0,
        st.shiftShock or 0.0,
        st.reconnectShock or 0.0,
        st.biteShock or 0.0,
        st.dirtyReturn or 0.0,
        st.recoveryDrag or 0.0,
        st.clutchEngagement or 0.0,
        st.rearPush or 0.0,
        st.yawHint or 0.0,
        st.rearOmegaAvg or 0.0,
        st.rearOmegaDiff or 0.0,
        st.linkedDrivetrain and "OK" or "NIL",
        st.linkedLSD and "OK" or "NIL",
        st.linkedContact and "OK" or "NIL",
        st.linkedYawBudget and "OK" or "NIL",
        st.linkedSlipRecovery and "OK" or "NIL",
        st.linkedMemory and "OK" or "NIL",
        st.linkedCarcass and "OK" or "NIL",
        tostring(st.torqueSource)
    )
end

return M
