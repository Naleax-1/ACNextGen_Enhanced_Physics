---@diagnostic disable: undefined-global

--============================================================
-- road_input_interpreter.lua
-- ACNextGen V1.1.5 Stable
-- Road Input Interpreter / Module Communication Layer
--============================================================

local M = {}

local PHASE = {
    QUIET          = 0,
    SMOOTH_INPUT   = 1,
    IMPACT         = 2,
    KERB           = 3,
    ROAD_NOISE     = 4,
    LANDING        = 5,
    BOTTOMING      = 6,
    HOP_RISK       = 7,
    UNLOADED       = 8,
    PATH_LOSS      = 9,
    SURFACE_LIMIT  = 10,
}

local PHASE_TEXT = {
    [PHASE.QUIET]         = "QUIET",
    [PHASE.SMOOTH_INPUT]  = "SMOOTH_INPUT",
    [PHASE.IMPACT]        = "IMPACT",
    [PHASE.KERB]          = "KERB",
    [PHASE.ROAD_NOISE]    = "ROAD_NOISE",
    [PHASE.LANDING]       = "LANDING",
    [PHASE.BOTTOMING]     = "BOTTOMING",
    [PHASE.HOP_RISK]      = "HOP_RISK",
    [PHASE.UNLOADED]      = "UNLOADED",
    [PHASE.PATH_LOSS]     = "PATH_LOSS",
    [PHASE.SURFACE_LIMIT] = "SURFACE_LIMIT",
}

local WHEEL_NAMES = {
    [0] = "FL",
    [1] = "FR",
    [2] = "RL",
    [3] = "RR",
}

M.params = {
    loadRef = 3000.0,
    loadMin = 150.0,
    speedMinKmh = 2.0,

    minDt = 0.00005,
    maxDt = 0.050,

    inputTau = 0.050,
    eventTau = 0.080,
    decayTau = 0.180,
    loadRateTau = 0.060,

    textureTauRise = 0.060,
    textureTauFall = 0.240,
    shockTauRise = 0.035,
    shockTauFall = 0.210,
    pathTauRise = 0.070,
    pathTauFall = 0.260,

    missingWheelTau = 0.220,
    noCarTau = 0.260,

    smallInputVelocity = 0.035,
    roadNoiseFull = 0.280,

    impactLoadRateWarn = 28000.0,
    impactLoadRateFull = 125000.0,
    impactDamperWarn = 0.16,
    impactDamperFull = 0.85,
    impactAccelWarn = 0.70,
    impactAccelFull = 3.20,

    unloadQualityThreshold = 0.32,
    unloadLoadFactor = 0.18,
    landingLoadRateWarn = 45000.0,
    landingLoadRateFull = 150000.0,

    kerbNoiseThreshold = 0.25,
    kerbImpactThreshold = 0.20,
    kerbSpeedMinKmh = 8.0,

    bottomLoadFactor = 1.85,
    bottomDamperThreshold = 0.65,
    hopLoadOscThreshold = 0.38,

    pathLossThreshold = 0.42,
    surfaceLimitThreshold = 0.50,

    damperImpactWeight = 0.40,
    verticalPulseImpactWeight = 0.28,
    loadRateImpactWeight = 0.30,
    bodyInputImpactWeight = 0.22,
    contactLossImpactWeight = 0.18,

    noiseDamperWeight = 0.36,
    noiseBodyWeight = 0.22,
    noiseLoadOscWeight = 0.20,
    noiseRoadMemoryWeight = 0.22,

    pathLossWeight = 0.34,
    forceLeakWeight = 0.24,
    verticalFlowWeight = 0.18,
    integrityLossWeight = 0.24,

    carcassSupportWeight = 0.14,
    contactTrustWeight = 0.16,
    tireDeliveryWeight = 0.22,

    impactSensorWeight = 0.18,
    roadBodyWeight = 0.12,
    loadPathHintWeight = 0.16,
    complianceHintWeight = 0.10,

    maxValue = 1.0,
    debugStoreInterval = 0.10,
}

local function newWheelState()
    return {
        load = 0.0,
        prevLoad = 0.0,
        loadRate = 0.0,
        loadOsc = 0.0,

        contactQuality = 1.0,
        contactTrust = 1.0,
        contactLoss = 0.0,
        contactLimit = 0.0,

        damperVelocity = 0.0,
        damperImpact = 0.0,
        damperReversal = 0.0,
        damperVertical = 0.0,
        damperHysteretic = 0.0,
        verticalPulse = 0.0,
        damperHeat = 0.0,
        roadMemory = 0.0,

        suspensionForce = 0.0,
        hopEnergy = 0.0,
        hopPulse = 0.0,

        loadPathLoss = 0.0,
        loadPathDelay = 0.0,
        loadPathContact = 0.0,
        loadPathIntegrity = 1.0,
        tireDelivery = 1.0,
        verticalFlow = 0.0,
        forceLeak = 0.0,
        bodyAbsorb = 0.0,
        pathCompression = 0.0,
        loadPathHint = 0.0,

        complianceForceLeak = 0.0,
        complianceLoadLoss = 0.0,
        bodyFlex = 0.0,
        subframeTwist = 0.0,
        rearAxleWindup = 0.0,

        carcassSupport = 1.0,
        carcassGripGate = 1.0,
        carcassVerticalNorm = 0.0,
        carcassHysteresis = 0.0,

        verticalInput = 0.0,
        lateralInput = 0.0,
        longitudinalInput = 0.0,

        roadNoise = 0.0,
        roadImpact = 0.0,
        kerbLike = 0.0,
        unloadEvent = 0.0,
        landingEvent = 0.0,
        bottomingRisk = 0.0,
        hopRisk = 0.0,
        severity = 0.0,

        roadTexture = 0.0,
        roadShock = 0.0,
        pathLossEvent = 0.0,
        surfaceLimit = 0.0,
        moduleHint = 0.0,

        unloadMemory = 0.0,
        phaseId = PHASE.QUIET,
        phaseText = "QUIET",
        active = false,
    }
end

M.state = {
    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    avgNoise = 0.0,
    avgImpact = 0.0,
    avgKerb = 0.0,
    avgUnload = 0.0,
    avgLanding = 0.0,
    avgBottoming = 0.0,
    avgHop = 0.0,
    avgSeverity = 0.0,

    avgTexture = 0.0,
    avgShock = 0.0,
    avgPathLoss = 0.0,
    avgSurfaceLimit = 0.0,
    avgVerticalFlow = 0.0,
    avgTireDelivery = 1.0,
    avgModuleHint = 0.0,
    avgLoadRate = 0.0,

    maxSeverity = 0.0,
    maxImpact = 0.0,
    dominantPhaseId = PHASE.QUIET,
    dominantPhase = "QUIET",

    bodyHeave = 0.0,
    bodyPitch = 0.0,
    bodyRoll = 0.0,
    bodyYawHint = 0.0,
    speedKmh = 0.0,

    impactSensorValue = 0.0,
    impactSensorRoot = 0.0,

    roadBodyLinked = false,
    contactLinked = false,
    damperLinked = false,
    hopLinked = false,
    loadLinked = false,
    suspensionLinked = false,
    loadPathLinked = false,
    complianceLinked = false,
    carcassLinked = false,
    impactLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,

    wheels = {},
}

for i = 0, 3 do
    M.state.wheels[i] = newWheelState()
    M.state[i] = M.state.wheels[i]
end

M.debug = M.state

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

local function max3(a, b, c)
    return math.max(safeNumber(a, 0.0), math.max(safeNumber(b, 0.0), safeNumber(c, 0.0)))
end

local function max4(a, b, c, d)
    return math.max(max3(a, b, c), safeNumber(d, 0.0))
end

local function smoothstep(edge0, edge1, x)
    edge0 = safeNumber(edge0, 0.0)
    edge1 = safeNumber(edge1, 1.0)
    x = safeNumber(x, 0.0)

    if edge0 == edge1 then
        return x >= edge1 and 1.0 or 0.0
    end

    local t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.0)
    dt = safeNumber(dt, 0.0)

    if tau <= 0.0 then
        return target
    end

    local a = clamp(dt / math.max(tau + dt, 0.001), 0.0, 1.0)
    return current + (target - current) * a
end

local function approachEvent(current, target, dt)
    local tau = target > current and M.params.eventTau or M.params.decayTau
    return lowPass(current, target, tau, dt)
end

local function approachCustom(current, target, riseTau, fallTau, dt)
    local tau = target > current and riseTau or fallTau
    return lowPass(current, target, tau, dt)
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

    if not ok then
        return nil
    end

    return car
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

local function vecLength(v)
    if not v then
        return 0.0
    end

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
    if not car then
        return 0.0
    end

    local speedKmh = safeField(car, "speedKmh", nil)
    if speedKmh ~= nil then
        return safeNumber(speedKmh, 0.0)
    end

    local speed = nil
    if speed ~= nil then
        speed = safeNumber(speed, 0.0)
        if speed >= 0.0 and speed < 120.0 then
            return speed * 3.6
        end
        return speed
    end

    local velocity = safeField(car, "velocity", nil)
    if velocity then
        return vecLength(velocity) * 3.6
    end

    local localVelocity = safeField(car, "localVelocity", nil)
    if localVelocity then
        return vecLength(localVelocity) * 3.6
    end

    return 0.0
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
        return wheels[index]
    end)

    if ok and wheel then
        return wheel
    end

    ok, wheel = pcall(function()
        return wheels[index + 1]
    end)

    if ok and wheel then
        return wheel
    end

    return nil
end

local function readWheelApiLoad(wheel)
    if not wheel then
        return nil
    end

    local fields = { "load", "loadK" }
    for i = 1, #fields do
        local value = safeField(wheel, fields[i], nil)
        if value ~= nil then
            return safeNumber(value, 0.0)
        end
    end

    return nil
end

local function dltLoadToAbsolute(value)
    local n = safeNumber(value, 0.0)
    if math.abs(n) > 900.0 then
        return math.abs(n)
    end
    return M.params.loadRef + n
end

local function getWheelLoad(wheel, index)
    local apiLoad = readWheelApiLoad(wheel)
    if apiLoad ~= nil and apiLoad > 1.0 then
        return apiLoad
    end

    local contactLoad = safeLoadRaw("ngp_contact_load_" .. index)
    if contactLoad ~= nil then
        M.state.contactLinked = true
        return safeNumber(contactLoad, M.params.loadRef)
    end

    local loadPathLoad = safeLoadRaw("ngp_load_path_load_" .. index)
    if loadPathLoad ~= nil then
        M.state.loadPathLinked = true
        return safeNumber(loadPathLoad, M.params.loadRef)
    end

    local wheelLoad = safeLoadRaw("ngp_wheel_load_" .. index)
    if wheelLoad ~= nil then
        M.state.loadLinked = true
        return safeNumber(wheelLoad, M.params.loadRef)
    end

    local dltLoad = safeLoadRaw("ngp_dlt_load_" .. index)
    if dltLoad ~= nil then
        M.state.loadLinked = true
        return dltLoadToAbsolute(dltLoad)
    end

    return M.params.loadRef
end

local function resetLinkFlags()
    local st = M.state
    st.roadBodyLinked = false
    st.contactLinked = false
    st.damperLinked = false
    st.hopLinked = false
    st.loadLinked = false
    st.suspensionLinked = false
    st.loadPathLinked = false
    st.complianceLinked = false
    st.carcassLinked = false
    st.impactLinked = false
end

local function readBodyInputs(car)
    local st = M.state

    local heave, heaveKey = safeLoadAlt(0.0, "ngp_body_heave_input", "ngp_rbi_heave", "ngp_road_body_heave")
    local pitch, pitchKey = safeLoadAlt(0.0, "ngp_body_pitch_input", "ngp_rbi_pitch", "ngp_road_body_pitch")
    local roll, rollKey = safeLoadAlt(0.0, "ngp_body_roll_input", "ngp_rbi_roll", "ngp_road_body_roll")
    local yaw, yawKey = safeLoadAlt(0.0, "ngp_body_yaw_hint", "ngp_rbi_yaw", "ngp_road_body_yaw")

    if heaveKey or pitchKey or rollKey or yawKey then
        st.roadBodyLinked = true
    end

    local acc = safeField(car, "acceleration", nil)
    if acc then
        local ax = safeNumber(safeField(acc, "x", 0.0), 0.0)
        local ay = safeNumber(safeField(acc, "y", 0.0), 0.0)
        local az = safeNumber(safeField(acc, "z", 0.0), 0.0)

        heave = math.max(heave, abs(ay) * 0.08, abs(az) * 0.04)
        pitch = pitch + ax * 0.018
        roll = roll + az * 0.012
    end

    local impactValue = safeLoadRaw("ngp_impact_value")
    local impactRoot = safeLoadRaw("ngp_impact_root_value")

    if impactValue ~= nil or impactRoot ~= nil then
        st.impactLinked = true
    end

    st.impactSensorValue = clamp(safeNumber(impactValue, 0.0), 0.0, 1.5)
    st.impactSensorRoot = clamp(safeNumber(impactRoot, 0.0), 0.0, 1.5)

    st.bodyHeave = clamp(heave, -2.0, 2.0)
    st.bodyPitch = clamp(pitch, -2.0, 2.0)
    st.bodyRoll = clamp(roll, -2.0, 2.0)
    st.bodyYawHint = clamp(yaw, -2.0, 2.0)
end

local function readContact(index, w)
    local contactQuality, qKey = safeLoadAlt(
        1.0,
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index,
        "ngp_tc_contact_" .. index
    )

    local contactTrust, trustKey = safeLoadAlt(
        contactQuality,
        "ngp_contact_trust_" .. index,
        "ngp_recovery_trust_" .. index,
        "ngp_contact_stability_score_" .. index
    )

    local contactLoss, lossKey = safeLoadAlt(
        nil,
        "ngp_contact_loss_" .. index,
        "ngp_tire_contact_loss_" .. index,
        "ngp_tcr_contact_loss_" .. index
    )

    local combined, combinedKey = safeLoadAlt(
        0.0,
        "ngp_contact_combined_slip_" .. index,
        "ngp_slip_recovery_slip_" .. index,
        "ngp_combined_slip_" .. index
    )

    if qKey or trustKey or lossKey or combinedKey then
        M.state.contactLinked = true
    end

    w.contactQuality = clamp(contactQuality, 0.0, 1.2)
    w.contactTrust = clamp(contactTrust, 0.0, 1.2)

    if lossKey then
        w.contactLoss = clamp(contactLoss, 0.0, 1.0)
    else
        w.contactLoss = clamp(1.0 - math.min(w.contactQuality, w.contactTrust), 0.0, 1.0)
    end

    w.contactLimit = clamp(combined / 1.35, 0.0, 1.0)
end

local function readDamper(index, w)
    local damperVel, velKey = safeLoadAlt(0.0, "ngp_damper_hyst_velocity_" .. index, "ngp_damper_velocity_" .. index)
    local damperImpact, impactKey = safeLoadAlt(0.0, "ngp_damper_hyst_impact_" .. index, "ngp_damper_impact_" .. index, "ngp_impact_damper_" .. index)
    local damperReversal, revKey = safeLoadAlt(0.0, "ngp_damper_hyst_reversal_" .. index, "ngp_damper_reversal_" .. index)
    local damperVertical, vertKey = safeLoadAlt(0.0, "ngp_damper_vertical_" .. index, "ngp_vertical_damping_" .. index)
    local hysteretic, hystKey = safeLoadAlt(0.0, "ngp_damper_hysteretic_" .. index, "ngp_hysteretic_damping_" .. index, "ngp_damper_hyst_" .. index)
    local verticalPulse, pulseKey = safeLoadAlt(0.0, "ngp_damper_vertical_pulse_" .. index, "ngp_road_vertical_pulse_" .. index)
    local heat, heatKey = safeLoadAlt(0.0, "ngp_damper_heat_" .. index, "ngp_damping_heat_" .. index)
    local memory, memKey = safeLoadAlt(0.0, "ngp_damper_road_memory_" .. index, "ngp_road_damper_memory_" .. index, "ngp_damper_hyst_memory_" .. index)

    if velKey or impactKey or revKey or vertKey or hystKey or pulseKey or heatKey or memKey then
        M.state.damperLinked = true
    end

    w.damperVelocity = safeNumber(damperVel, 0.0)
    w.damperImpact = clamp(damperImpact, 0.0, 1.2)
    w.damperReversal = clamp(damperReversal, 0.0, 1.2)
    w.damperVertical = clamp(damperVertical, 0.0, 1.2)
    w.damperHysteretic = clamp(hysteretic, 0.0, 1.2)
    w.verticalPulse = clamp(verticalPulse, 0.0, 1.2)
    w.damperHeat = clamp(heat, 0.0, 1.2)
    w.roadMemory = clamp(memory, 0.0, 1.2)
end

local function readLoadPath(index, w)
    local loss, lossKey = safeLoadAlt(0.0, "ngp_load_path_loss_" .. index, "ngp_lp_loss_" .. index)
    local delay, delayKey = safeLoadAlt(0.0, "ngp_load_path_delay_" .. index, "ngp_lp_delay_" .. index)
    local contact, contactKey = safeLoadAlt(0.0, "ngp_load_path_contact_" .. index, "ngp_lp_contact_" .. index)
    local integrity, integrityKey = safeLoadAlt(1.0, "ngp_load_path_integrity_" .. index, "ngp_lp_integrity_" .. index)
    local delivery, deliveryKey = safeLoadAlt(1.0, "ngp_load_path_tire_delivery_" .. index, "ngp_lp_tire_delivery_" .. index)
    local verticalFlow, flowKey = safeLoadAlt(0.0, "ngp_load_path_vertical_flow_" .. index, "ngp_lp_vertical_flow_" .. index)
    local forceLeak, leakKey = safeLoadAlt(0.0, "ngp_load_path_force_leak_" .. index, "ngp_lp_force_leak_" .. index, "ngp_force_path_loss_" .. index)
    local bodyAbsorb, bodyKey = safeLoadAlt(0.0, "ngp_load_path_body_absorb_" .. index, "ngp_road_body_absorb_" .. index)
    local compression, compKey = safeLoadAlt(0.0, "ngp_load_path_compression_" .. index, "ngp_lp_compression_" .. index)
    local hint, hintKey = safeLoadAlt(0.0, "ngp_load_path_work_" .. index, "ngp_lp_work_" .. index)

    if lossKey or delayKey or contactKey or integrityKey or deliveryKey or flowKey or leakKey or bodyKey or compKey or hintKey then
        M.state.loadPathLinked = true
    end

    w.loadPathLoss = clamp(loss, 0.0, 1.2)
    w.loadPathDelay = clamp(delay, 0.0, 1.2)
    w.loadPathContact = clamp(contact, 0.0, 1.2)
    w.loadPathIntegrity = clamp(integrity, 0.0, 1.2)
    w.tireDelivery = clamp(delivery, 0.0, 1.2)
    w.verticalFlow = clamp(verticalFlow, 0.0, 1.2)
    w.forceLeak = clamp(forceLeak, 0.0, 1.2)
    w.bodyAbsorb = clamp(bodyAbsorb, 0.0, 1.2)
    w.pathCompression = clamp(compression, 0.0, 1.2)
    w.loadPathHint = clamp(hint, 0.0, 1.2)
end

local function readCompliance(index, w)
    local forceLeak, leakKey = safeLoadAlt(0.0, "ngp_compliance_force_leak_" .. index, "ngp_cs_force_leak_" .. index, "ngp_force_leak")
    local loadLoss, lossKey = safeLoadAlt(0.0, "ngp_compliance_load_path_loss_" .. index, "ngp_cs_load_path_loss_" .. index, "ngp_load_path_compliance_loss")
    local bodyFlex, bodyKey = safeLoadAlt(0.0, "ngp_compliance_body_deflection_" .. index, "ngp_body_compliance_" .. index, "ngp_body_flex")
    local subframe, subKey = safeLoadAlt(0.0, "ngp_compliance_subframe_deflection_" .. index, "ngp_subframe_twist")
    local rearAxle, axleKey = safeLoadAlt(0.0, "ngp_rear_axle_windup", "ngp_windup_rear_axle", "ngp_driveline_windup")

    if leakKey or lossKey or bodyKey or subKey or axleKey then
        M.state.complianceLinked = true
    end

    w.complianceForceLeak = clamp(forceLeak, 0.0, 1.2)
    w.complianceLoadLoss = clamp(loadLoss, 0.0, 1.2)
    w.bodyFlex = clamp(bodyFlex, 0.0, 1.2)
    w.subframeTwist = clamp(subframe, 0.0, 1.2)
    w.rearAxleWindup = clamp(rearAxle, 0.0, 1.2)
end

local function readCarcass(index, w)
    local support, supportKey = safeLoadAlt(1.0, "ngp_carcass_support_" .. index, "ngp_tire_carcass_support_" .. index)
    local gripGate, gateKey = safeLoadAlt(1.0, "ngp_carcass_grip_gate_" .. index, "ngp_tire_carcass_grip_gate_" .. index)
    local verticalNorm, vertKey = safeLoadAlt(0.0, "ngp_carcass_vertical_norm_" .. index, "ngp_tire_carcass_vertical_norm_" .. index)
    local hyst, hystKey = safeLoadAlt(0.0, "ngp_carcass_hysteresis_" .. index, "ngp_tire_carcass_hysteresis_" .. index)

    if supportKey or gateKey or vertKey or hystKey then
        M.state.carcassLinked = true
    end

    w.carcassSupport = clamp(support, 0.0, 1.2)
    w.carcassGripGate = clamp(gripGate, 0.0, 1.2)
    w.carcassVerticalNorm = clamp(verticalNorm, 0.0, 1.2)
    w.carcassHysteresis = clamp(hyst, 0.0, 1.2)
end

local function readSuspensionAndHop(index, w)
    local suspForce = safeLoadRaw("ngp_susp_" .. index)
    if suspForce == nil then suspForce = safeLoadRaw("ngp_susp_int_damper_" .. index) end
    if suspForce == nil then suspForce = safeLoadRaw("ngp_susp_integrated_force_" .. index) end

    if suspForce ~= nil then
        M.state.suspensionLinked = true
    end

    w.suspensionForce = safeNumber(suspForce, 0.0)

    local hop = safeLoadRaw("ngp_tire_hop_energy_" .. index)
    if hop == nil then hop = safeLoadRaw("ngp_tirehop_energy_" .. index) end
    if hop == nil then hop = safeLoadRaw("ngp_tire_hop_" .. index) end

    local hopPulse = safeLoadRaw("ngp_tire_hop_loadpulse_" .. index)
    if hopPulse == nil then hopPulse = safeLoadRaw("ngp_tirehop_" .. index) end

    if hop ~= nil or hopPulse ~= nil then
        M.state.hopLinked = true
    end

    w.hopEnergy = clamp(safeNumber(hop, 0.0), 0.0, 1.2)
    w.hopPulse = clamp(abs(hopPulse) / 5000.0, 0.0, 1.2)
end

local function readWheelInputs(index, w, wheel, dt)
    local p = M.params

    local load = math.max(0.0, getWheelLoad(wheel, index))
    local prevLoad = w.load or load
    local rawLoadRate = (load - prevLoad) / math.max(dt, 0.001)

    w.prevLoad = prevLoad
    w.load = lowPass(w.load, load, p.inputTau, dt)
    w.loadRate = lowPass(w.loadRate, rawLoadRate, p.loadRateTau, dt)

    readContact(index, w)
    readDamper(index, w)
    readLoadPath(index, w)
    readCompliance(index, w)
    readCarcass(index, w)
    readSuspensionAndHop(index, w)

    w.verticalInput = clamp(
        abs(M.state.bodyHeave) * 0.45
        + abs(w.damperVelocity) * 0.25
        + w.verticalPulse * 0.22
        + w.verticalFlow * 0.18
        + w.carcassVerticalNorm * 0.14,
        0.0,
        2.0
    )

    w.lateralInput = clamp(
        abs(M.state.bodyRoll) * 0.55
        + abs(M.state.bodyYawHint) * 0.22
        + w.subframeTwist * 0.14
        + w.forceLeak * 0.10,
        0.0,
        2.0
    )

    w.longitudinalInput = clamp(
        abs(M.state.bodyPitch) * 0.60
        + w.rearAxleWindup * 0.18
        + w.loadPathDelay * 0.12,
        0.0,
        2.0
    )
end

local function classifyPhase(w)
    if w.unloadEvent > 0.55 then
        return PHASE.UNLOADED, PHASE_TEXT[PHASE.UNLOADED]
    end

    if w.landingEvent > 0.50 then
        return PHASE.LANDING, PHASE_TEXT[PHASE.LANDING]
    end

    if w.bottomingRisk > 0.58 then
        return PHASE.BOTTOMING, PHASE_TEXT[PHASE.BOTTOMING]
    end

    if w.pathLossEvent > 0.58 then
        return PHASE.PATH_LOSS, PHASE_TEXT[PHASE.PATH_LOSS]
    end

    if w.surfaceLimit > 0.55 then
        return PHASE.SURFACE_LIMIT, PHASE_TEXT[PHASE.SURFACE_LIMIT]
    end

    if w.kerbLike > 0.45 then
        return PHASE.KERB, PHASE_TEXT[PHASE.KERB]
    end

    if w.roadImpact > 0.48 or w.roadShock > 0.55 then
        return PHASE.IMPACT, PHASE_TEXT[PHASE.IMPACT]
    end

    if w.hopRisk > 0.45 then
        return PHASE.HOP_RISK, PHASE_TEXT[PHASE.HOP_RISK]
    end

    if w.roadNoise > 0.24 or w.roadTexture > 0.30 then
        return PHASE.ROAD_NOISE, PHASE_TEXT[PHASE.ROAD_NOISE]
    end

    if w.severity > 0.08 then
        return PHASE.SMOOTH_INPUT, PHASE_TEXT[PHASE.SMOOTH_INPUT]
    end

    return PHASE.QUIET, PHASE_TEXT[PHASE.QUIET]
end

local function updateWheelInterpretation(index, w, dt)
    local p = M.params

    local speedFactor = smoothstep(p.speedMinKmh, p.speedMinKmh + 10.0, M.state.speedKmh)
    local loadFactor = w.load / math.max(p.loadRef, 1.0)
    local absLoadRate = abs(w.loadRate)
    local positiveLoadRate = math.max(0.0, w.loadRate)
    local negativeLoadRate = math.max(0.0, -w.loadRate)

    local loadRateImpact = smoothstep(p.impactLoadRateWarn, p.impactLoadRateFull, absLoadRate)
    local damperImpact = math.max(
        w.damperImpact,
        smoothstep(p.impactDamperWarn, p.impactDamperFull, abs(w.damperVelocity))
    )

    local bodyImpact = smoothstep(
        p.impactAccelWarn,
        p.impactAccelFull,
        abs(M.state.bodyHeave) + abs(M.state.bodyPitch) * 0.35 + abs(M.state.bodyRoll) * 0.35
    )

    local impactSensor = clamp(math.max(M.state.impactSensorValue, M.state.impactSensorRoot), 0.0, 1.0)
    local contactImpact = w.contactLoss
    local verticalImpact = clamp(math.max(w.verticalPulse, w.verticalFlow * 0.65, w.hopPulse * 0.50), 0.0, 1.0)

    local rawImpact = clamp(
        damperImpact * p.damperImpactWeight
        + verticalImpact * p.verticalPulseImpactWeight
        + loadRateImpact * p.loadRateImpactWeight
        + bodyImpact * p.bodyInputImpactWeight
        + contactImpact * p.contactLossImpactWeight
        + impactSensor * p.impactSensorWeight,
        0.0,
        p.maxValue
    )

    local loadOscTarget = clamp(absLoadRate / math.max(p.impactLoadRateFull, 1.0), 0.0, 1.0)
    w.loadOsc = lowPass(w.loadOsc, loadOscTarget, p.inputTau, dt)

    local damperNoise = smoothstep(p.smallInputVelocity, p.roadNoiseFull, abs(w.damperVelocity))
    local bodyNoise = clamp((abs(M.state.bodyHeave) + abs(M.state.bodyRoll) + abs(M.state.bodyPitch)) * 0.33, 0.0, 1.0)
    local loadNoise = smoothstep(0.06, p.hopLoadOscThreshold, w.loadOsc)
    local memoryNoise = clamp(w.roadMemory * 0.75 + w.damperHeat * 0.25, 0.0, 1.0)

    local rawNoise = clamp(
        damperNoise * p.noiseDamperWeight
        + bodyNoise * p.noiseBodyWeight
        + loadNoise * p.noiseLoadOscWeight
        + memoryNoise * p.noiseRoadMemoryWeight,
        0.0,
        p.maxValue
    ) * speedFactor

    local rawTexture = clamp(
        rawNoise * 0.34
        + w.roadMemory * 0.28
        + w.damperHysteretic * 0.18
        + w.carcassHysteresis * 0.12
        + w.contactLimit * 0.08,
        0.0,
        1.0
    ) * speedFactor

    local rawShock = clamp(
        rawImpact * 0.50
        + w.verticalPulse * 0.22
        + w.damperImpact * 0.16
        + loadRateImpact * 0.12,
        0.0,
        1.0
    )

    local rawKerb = 0.0
    if M.state.speedKmh >= p.kerbSpeedMinKmh then
        rawKerb = clamp(
            smoothstep(p.kerbNoiseThreshold, 0.75, rawNoise)
            * smoothstep(p.kerbImpactThreshold, 0.85, rawImpact)
            * (0.62 + 0.22 * math.max(abs(M.state.bodyRoll), abs(M.state.bodyYawHint)) + 0.16 * w.damperReversal),
            0.0,
            1.0
        )
    end

    local lowLoad = 1.0 - smoothstep(p.loadMin, p.loadRef * p.unloadLoadFactor, w.load)
    local contactUnload = smoothstep(p.unloadQualityThreshold, 0.0, w.contactQuality)
    local loadDropUnload = smoothstep(p.impactLoadRateWarn, p.impactLoadRateFull, negativeLoadRate)

    local rawUnload = clamp(math.max(lowLoad, contactUnload, loadDropUnload * 0.80), 0.0, 1.0)

    w.unloadMemory = lowPass(
        w.unloadMemory,
        rawUnload,
        rawUnload > w.unloadMemory and 0.040 or 0.220,
        dt
    )

    local landingLoad = smoothstep(p.landingLoadRateWarn, p.landingLoadRateFull, positiveLoadRate)
    local rawLanding = clamp(w.unloadMemory * landingLoad + rawImpact * 0.25, 0.0, 1.0)

    local bottomLoad = smoothstep(p.bottomLoadFactor, p.bottomLoadFactor + 1.10, loadFactor)
    local bottomDamper = smoothstep(p.bottomDamperThreshold, 1.0, damperImpact)

    local rawBottoming = clamp(
        bottomLoad * 0.48
        + bottomDamper * 0.28
        + w.pathCompression * 0.16
        + w.bodyAbsorb * 0.08,
        0.0,
        1.0
    )

    local rawHop = clamp(
        math.max(
            w.hopEnergy,
            w.hopPulse,
            loadNoise * 0.54 + w.damperReversal * 0.28 + w.verticalPulse * 0.18
        ),
        0.0,
        1.0
    )

    local integrityLoss = clamp(1.0 - w.loadPathIntegrity, 0.0, 1.0)
    local deliveryLoss = clamp(1.0 - w.tireDelivery, 0.0, 1.0)

    local rawPathLoss = clamp(
        w.loadPathLoss * p.pathLossWeight
        + math.max(w.forceLeak, w.complianceForceLeak) * p.forceLeakWeight
        + w.verticalFlow * p.verticalFlowWeight
        + integrityLoss * p.integrityLossWeight
        + w.complianceLoadLoss * 0.20
        + deliveryLoss * 0.18
        + w.loadPathHint * p.loadPathHintWeight
        + (w.complianceLoadLoss + w.complianceForceLeak) * 0.5 * p.complianceHintWeight,
        0.0,
        1.0
    )

    local rawSurfaceLimit = clamp(
        w.contactLimit * 0.25
        + w.contactLoss * 0.22
        + (1.0 - clamp(w.carcassSupport, 0.0, 1.0)) * p.carcassSupportWeight
        + (1.0 - clamp(w.contactTrust, 0.0, 1.0)) * p.contactTrustWeight
        + deliveryLoss * p.tireDeliveryWeight
        + w.carcassVerticalNorm * 0.10,
        0.0,
        1.0
    )

    w.roadNoise = approachEvent(w.roadNoise, rawNoise, dt)
    w.roadImpact = approachEvent(w.roadImpact, rawImpact, dt)
    w.kerbLike = approachEvent(w.kerbLike, rawKerb, dt)
    w.unloadEvent = approachEvent(w.unloadEvent, rawUnload, dt)
    w.landingEvent = approachEvent(w.landingEvent, rawLanding, dt)
    w.bottomingRisk = approachEvent(w.bottomingRisk, rawBottoming, dt)
    w.hopRisk = approachEvent(w.hopRisk, rawHop, dt)

    w.roadTexture = approachCustom(w.roadTexture, rawTexture, p.textureTauRise, p.textureTauFall, dt)
    w.roadShock = approachCustom(w.roadShock, rawShock, p.shockTauRise, p.shockTauFall, dt)
    w.pathLossEvent = approachCustom(w.pathLossEvent, rawPathLoss, p.pathTauRise, p.pathTauFall, dt)
    w.surfaceLimit = approachCustom(w.surfaceLimit, rawSurfaceLimit, p.eventTau, p.decayTau, dt)

    w.severity = clamp(
        max4(w.roadImpact, w.kerbLike, w.landingEvent, w.roadShock) * 0.45
        + max4(w.unloadEvent, w.bottomingRisk, w.hopRisk, w.pathLossEvent) * 0.35
        + max3(w.roadTexture, w.surfaceLimit, w.roadNoise) * 0.20,
        0.0,
        1.0
    )

    w.moduleHint = clamp(
        w.severity * 0.35
        + w.pathLossEvent * 0.25
        + w.surfaceLimit * 0.20
        + w.roadTexture * 0.12
        + w.hopRisk * 0.08,
        0.0,
        1.0
    )

    w.phaseId, w.phaseText = classifyPhase(w)
end

local function decayWheel(index, dt, unloaded)
    local w = M.state.wheels[index]
    local tau = unloaded and M.params.missingWheelTau or M.params.noCarTau

    w.load = lowPass(w.load, unloaded and 0.0 or M.params.loadRef, tau, dt or 0.016)
    w.prevLoad = w.load
    w.loadRate = lowPass(w.loadRate, 0.0, tau, dt or 0.016)
    w.loadOsc = lowPass(w.loadOsc, 0.0, tau, dt or 0.016)

    w.contactQuality = lowPass(w.contactQuality, unloaded and 0.0 or 1.0, tau, dt or 0.016)
    w.contactTrust = lowPass(w.contactTrust, unloaded and 0.0 or 1.0, tau, dt or 0.016)
    w.contactLoss = lowPass(w.contactLoss, unloaded and 1.0 or 0.0, tau, dt or 0.016)
    w.contactLimit = lowPass(w.contactLimit, 0.0, tau, dt or 0.016)

    w.damperVelocity = 0.0
    w.damperImpact = lowPass(w.damperImpact, 0.0, tau, dt or 0.016)
    w.damperReversal = lowPass(w.damperReversal, 0.0, tau, dt or 0.016)
    w.damperVertical = lowPass(w.damperVertical, 0.0, tau, dt or 0.016)
    w.damperHysteretic = lowPass(w.damperHysteretic, 0.0, tau, dt or 0.016)
    w.verticalPulse = lowPass(w.verticalPulse, 0.0, tau, dt or 0.016)
    w.damperHeat = lowPass(w.damperHeat, 0.0, tau, dt or 0.016)
    w.roadMemory = lowPass(w.roadMemory, 0.0, tau, dt or 0.016)

    w.suspensionForce = 0.0
    w.hopEnergy = lowPass(w.hopEnergy, 0.0, tau, dt or 0.016)
    w.hopPulse = lowPass(w.hopPulse, 0.0, tau, dt or 0.016)

    w.loadPathLoss = lowPass(w.loadPathLoss, 0.0, tau, dt or 0.016)
    w.loadPathDelay = lowPass(w.loadPathDelay, 0.0, tau, dt or 0.016)
    w.loadPathContact = lowPass(w.loadPathContact, 0.0, tau, dt or 0.016)
    w.loadPathIntegrity = lowPass(w.loadPathIntegrity, unloaded and 0.0 or 1.0, tau, dt or 0.016)
    w.tireDelivery = lowPass(w.tireDelivery, unloaded and 0.0 or 1.0, tau, dt or 0.016)
    w.verticalFlow = lowPass(w.verticalFlow, 0.0, tau, dt or 0.016)
    w.forceLeak = lowPass(w.forceLeak, 0.0, tau, dt or 0.016)
    w.bodyAbsorb = lowPass(w.bodyAbsorb, 0.0, tau, dt or 0.016)
    w.pathCompression = lowPass(w.pathCompression, 0.0, tau, dt or 0.016)
    w.loadPathHint = lowPass(w.loadPathHint, 0.0, tau, dt or 0.016)

    w.complianceForceLeak = lowPass(w.complianceForceLeak, 0.0, tau, dt or 0.016)
    w.complianceLoadLoss = lowPass(w.complianceLoadLoss, 0.0, tau, dt or 0.016)
    w.bodyFlex = lowPass(w.bodyFlex, 0.0, tau, dt or 0.016)
    w.subframeTwist = lowPass(w.subframeTwist, 0.0, tau, dt or 0.016)
    w.rearAxleWindup = lowPass(w.rearAxleWindup, 0.0, tau, dt or 0.016)

    w.carcassSupport = lowPass(w.carcassSupport, unloaded and 0.0 or 1.0, tau, dt or 0.016)
    w.carcassGripGate = lowPass(w.carcassGripGate, unloaded and 0.0 or 1.0, tau, dt or 0.016)
    w.carcassVerticalNorm = lowPass(w.carcassVerticalNorm, 0.0, tau, dt or 0.016)
    w.carcassHysteresis = lowPass(w.carcassHysteresis, 0.0, tau, dt or 0.016)

    w.verticalInput = lowPass(w.verticalInput, 0.0, tau, dt or 0.016)
    w.lateralInput = lowPass(w.lateralInput, 0.0, tau, dt or 0.016)
    w.longitudinalInput = lowPass(w.longitudinalInput, 0.0, tau, dt or 0.016)

    w.roadNoise = lowPass(w.roadNoise, 0.0, M.params.decayTau, dt or 0.016)
    w.roadImpact = lowPass(w.roadImpact, 0.0, M.params.decayTau, dt or 0.016)
    w.kerbLike = lowPass(w.kerbLike, 0.0, M.params.decayTau, dt or 0.016)
    w.unloadEvent = lowPass(w.unloadEvent, unloaded and 1.0 or 0.0, M.params.decayTau, dt or 0.016)
    w.landingEvent = lowPass(w.landingEvent, 0.0, M.params.decayTau, dt or 0.016)
    w.bottomingRisk = lowPass(w.bottomingRisk, 0.0, M.params.decayTau, dt or 0.016)
    w.hopRisk = lowPass(w.hopRisk, 0.0, M.params.decayTau, dt or 0.016)
    w.roadTexture = lowPass(w.roadTexture, 0.0, M.params.decayTau, dt or 0.016)
    w.roadShock = lowPass(w.roadShock, 0.0, M.params.decayTau, dt or 0.016)
    w.pathLossEvent = lowPass(w.pathLossEvent, 0.0, M.params.decayTau, dt or 0.016)
    w.surfaceLimit = lowPass(w.surfaceLimit, 0.0, M.params.decayTau, dt or 0.016)

    if unloaded then
        w.severity = w.unloadEvent
        w.phaseId = PHASE.UNLOADED
        w.phaseText = PHASE_TEXT[PHASE.UNLOADED]
    else
        w.severity = lowPass(w.severity, 0.0, M.params.decayTau, dt or 0.016)
        w.phaseId = PHASE.QUIET
        w.phaseText = PHASE_TEXT[PHASE.QUIET]
    end

    w.moduleHint = lowPass(w.moduleHint, 0.0, M.params.decayTau, dt or 0.016)
    w.active = false
end

local function updateAverages()
    local st = M.state

    local sumNoise = 0.0
    local sumImpact = 0.0
    local sumKerb = 0.0
    local sumUnload = 0.0
    local sumLanding = 0.0
    local sumBottoming = 0.0
    local sumHop = 0.0
    local sumSeverity = 0.0
    local sumTexture = 0.0
    local sumShock = 0.0
    local sumPathLoss = 0.0
    local sumSurfaceLimit = 0.0
    local sumVerticalFlow = 0.0
    local sumTireDelivery = 0.0
    local sumHint = 0.0
    local sumLoadRate = 0.0

    local maxSeverity = 0.0
    local maxImpact = 0.0
    local dominantId = PHASE.QUIET
    local dominantText = "QUIET"

    for i = 0, 3 do
        local w = st.wheels[i]

        sumNoise = sumNoise + (w.roadNoise or 0.0)
        sumImpact = sumImpact + (w.roadImpact or 0.0)
        sumKerb = sumKerb + (w.kerbLike or 0.0)
        sumUnload = sumUnload + (w.unloadEvent or 0.0)
        sumLanding = sumLanding + (w.landingEvent or 0.0)
        sumBottoming = sumBottoming + (w.bottomingRisk or 0.0)
        sumHop = sumHop + (w.hopRisk or 0.0)
        sumSeverity = sumSeverity + (w.severity or 0.0)
        sumTexture = sumTexture + (w.roadTexture or 0.0)
        sumShock = sumShock + (w.roadShock or 0.0)
        sumPathLoss = sumPathLoss + (w.pathLossEvent or 0.0)
        sumSurfaceLimit = sumSurfaceLimit + (w.surfaceLimit or 0.0)
        sumVerticalFlow = sumVerticalFlow + (w.verticalFlow or 0.0)
        sumTireDelivery = sumTireDelivery + (w.tireDelivery or 1.0)
        sumHint = sumHint + (w.moduleHint or 0.0)
        sumLoadRate = sumLoadRate + abs(w.loadRate or 0.0)

        if (w.severity or 0.0) > maxSeverity then
            maxSeverity = w.severity or 0.0
            dominantId = w.phaseId or PHASE.QUIET
            dominantText = w.phaseText or "QUIET"
        end

        maxImpact = math.max(maxImpact, w.roadImpact or 0.0)
    end

    st.avgNoise = sumNoise * 0.25
    st.avgImpact = sumImpact * 0.25
    st.avgKerb = sumKerb * 0.25
    st.avgUnload = sumUnload * 0.25
    st.avgLanding = sumLanding * 0.25
    st.avgBottoming = sumBottoming * 0.25
    st.avgHop = sumHop * 0.25
    st.avgSeverity = sumSeverity * 0.25

    st.avgTexture = sumTexture * 0.25
    st.avgShock = sumShock * 0.25
    st.avgPathLoss = sumPathLoss * 0.25
    st.avgSurfaceLimit = sumSurfaceLimit * 0.25
    st.avgVerticalFlow = sumVerticalFlow * 0.25
    st.avgTireDelivery = sumTireDelivery * 0.25
    st.avgModuleHint = sumHint * 0.25
    st.avgLoadRate = sumLoadRate * 0.25

    st.maxSeverity = maxSeverity
    st.maxImpact = maxImpact
    st.dominantPhaseId = dominantId
    st.dominantPhase = dominantText
end

local function exportWheel(index, w)
    safeStore("ngp_road_noise_" .. index, w.roadNoise or 0.0)
    safeStore("ngp_road_impact_" .. index, w.roadImpact or 0.0)
    safeStore("ngp_kerb_like_" .. index, w.kerbLike or 0.0)
    safeStore("ngp_unload_event_" .. index, w.unloadEvent or 0.0)
    safeStore("ngp_landing_event_" .. index, w.landingEvent or 0.0)
    safeStore("ngp_bottoming_risk_" .. index, w.bottomingRisk or 0.0)
    safeStore("ngp_hop_risk_" .. index, w.hopRisk or 0.0)
    safeStore("ngp_road_input_severity_" .. index, w.severity or 0.0)
    safeStore("ngp_road_input_phase_id_" .. index, w.phaseId or 0)
    safeStore("ngp_road_input_phase_" .. index, w.phaseText or "UNKNOWN")

    safeStore("ngp_rii_noise_" .. index, w.roadNoise or 0.0)
    safeStore("ngp_rii_impact_" .. index, w.roadImpact or 0.0)
    safeStore("ngp_rii_kerb_" .. index, w.kerbLike or 0.0)
    safeStore("ngp_rii_unload_" .. index, w.unloadEvent or 0.0)
    safeStore("ngp_rii_landing_" .. index, w.landingEvent or 0.0)
    safeStore("ngp_rii_bottoming_" .. index, w.bottomingRisk or 0.0)
    safeStore("ngp_rii_hop_" .. index, w.hopRisk or 0.0)
    safeStore("ngp_rii_severity_" .. index, w.severity or 0.0)

    safeStore("ngp_road_texture_" .. index, w.roadTexture or 0.0)
    safeStore("ngp_road_shock_" .. index, w.roadShock or 0.0)
    safeStore("ngp_road_path_loss_" .. index, w.pathLossEvent or 0.0)
    safeStore("ngp_road_surface_limit_" .. index, w.surfaceLimit or 0.0)
    safeStore("ngp_road_module_hint_" .. index, w.moduleHint or 0.0)

    safeStore("ngp_road_vertical_flow_" .. index, w.verticalFlow or 0.0)
    safeStore("ngp_road_force_leak_" .. index, w.forceLeak or 0.0)
    safeStore("ngp_road_tire_delivery_" .. index, w.tireDelivery or 1.0)
    safeStore("ngp_road_integrity_" .. index, w.loadPathIntegrity or 1.0)
    safeStore("ngp_road_body_absorb_" .. index, w.bodyAbsorb or 0.0)

    safeStore("ngp_rii_texture_" .. index, w.roadTexture or 0.0)
    safeStore("ngp_rii_shock_" .. index, w.roadShock or 0.0)
    safeStore("ngp_rii_path_loss_" .. index, w.pathLossEvent or 0.0)
    safeStore("ngp_rii_surface_limit_" .. index, w.surfaceLimit or 0.0)
    safeStore("ngp_rii_hint_" .. index, w.moduleHint or 0.0)

    safeStore("ngp_rii_vertical_" .. index, w.verticalInput or 0.0)
    safeStore("ngp_rii_lateral_" .. index, w.lateralInput or 0.0)
    safeStore("ngp_rii_longitudinal_" .. index, w.longitudinalInput or 0.0)

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_road_input_load_" .. index, w.load or 0.0)
    safeStore("ngp_road_input_load_rate_" .. index, w.loadRate or 0.0)
    safeStore("ngp_road_input_contact_" .. index, w.contactQuality or 0.0)
    safeStore("ngp_road_input_trust_" .. index, w.contactTrust or 0.0)
    safeStore("ngp_road_input_damper_velocity_" .. index, w.damperVelocity or 0.0)
    safeStore("ngp_road_input_damper_impact_" .. index, w.damperImpact or 0.0)
    safeStore("ngp_road_input_vertical_" .. index, w.verticalInput or 0.0)
    safeStore("ngp_road_input_lateral_" .. index, w.lateralInput or 0.0)
    safeStore("ngp_road_input_longitudinal_" .. index, w.longitudinalInput or 0.0)

    safeStore("ngp_road_input_vertical_pulse_" .. index, w.verticalPulse or 0.0)
    safeStore("ngp_road_input_damper_heat_" .. index, w.damperHeat or 0.0)
    safeStore("ngp_road_input_road_memory_" .. index, w.roadMemory or 0.0)
    safeStore("ngp_road_input_load_path_loss_" .. index, w.loadPathLoss or 0.0)
    safeStore("ngp_road_input_force_leak_" .. index, w.forceLeak or 0.0)
    safeStore("ngp_road_input_carcass_support_" .. index, w.carcassSupport or 0.0)
    safeStore("ngp_road_input_name_" .. index, WHEEL_NAMES[index] or tostring(index))
end

local function exportGlobal()
    local st = M.state

    safeStore("ngp_road_input_status", st.status or "UNKNOWN")
    safeStore("ngp_road_input_update_count", st.updateCount or 0)
    safeStore("ngp_road_input_wheels_valid", st.wheelsValid and 1 or 0)

    safeStore("ngp_road_input_avg_noise", st.avgNoise or 0.0)
    safeStore("ngp_road_input_avg_impact", st.avgImpact or 0.0)
    safeStore("ngp_road_input_avg_kerb", st.avgKerb or 0.0)
    safeStore("ngp_road_input_avg_unload", st.avgUnload or 0.0)
    safeStore("ngp_road_input_avg_landing", st.avgLanding or 0.0)
    safeStore("ngp_road_input_avg_bottoming", st.avgBottoming or 0.0)
    safeStore("ngp_road_input_avg_hop", st.avgHop or 0.0)
    safeStore("ngp_road_input_avg_severity", st.avgSeverity or 0.0)

    safeStore("ngp_road_noise", st.avgNoise or 0.0)
    safeStore("ngp_road_impact", st.avgImpact or 0.0)
    safeStore("ngp_kerb_like", st.avgKerb or 0.0)
    safeStore("ngp_unload_event", st.avgUnload or 0.0)
    safeStore("ngp_landing_event", st.avgLanding or 0.0)
    safeStore("ngp_bottoming_risk", st.avgBottoming or 0.0)
    safeStore("ngp_hop_risk", st.avgHop or 0.0)
    safeStore("ngp_road_input_severity", st.avgSeverity or 0.0)

    safeStore("ngp_rii_avg_noise", st.avgNoise or 0.0)
    safeStore("ngp_rii_avg_impact", st.avgImpact or 0.0)
    safeStore("ngp_rii_avg_kerb", st.avgKerb or 0.0)
    safeStore("ngp_rii_avg_unload", st.avgUnload or 0.0)
    safeStore("ngp_rii_avg_landing", st.avgLanding or 0.0)
    safeStore("ngp_rii_avg_bottoming", st.avgBottoming or 0.0)
    safeStore("ngp_rii_avg_hop", st.avgHop or 0.0)
    safeStore("ngp_rii_avg_severity", st.avgSeverity or 0.0)

    safeStore("ngp_road_texture", st.avgTexture or 0.0)
    safeStore("ngp_road_shock", st.avgShock or 0.0)
    safeStore("ngp_road_path_loss", st.avgPathLoss or 0.0)
    safeStore("ngp_road_surface_limit", st.avgSurfaceLimit or 0.0)
    safeStore("ngp_road_vertical_flow", st.avgVerticalFlow or 0.0)
    safeStore("ngp_road_tire_delivery", st.avgTireDelivery or 1.0)

    safeStore("ngp_rii_avg_texture", st.avgTexture or 0.0)
    safeStore("ngp_rii_avg_shock", st.avgShock or 0.0)
    safeStore("ngp_rii_avg_path_loss", st.avgPathLoss or 0.0)
    safeStore("ngp_rii_avg_surface_limit", st.avgSurfaceLimit or 0.0)
    safeStore("ngp_rii_avg_vertical_flow", st.avgVerticalFlow or 0.0)
    safeStore("ngp_rii_avg_tire_delivery", st.avgTireDelivery or 1.0)

    safeStore("ngp_road_input_avg_hint", st.avgModuleHint or 0.0)
    safeStore("ngp_road_input_avg_load_rate", st.avgLoadRate or 0.0)
    safeStore("ngp_road_input_max_severity", st.maxSeverity or 0.0)
    safeStore("ngp_road_input_max_impact", st.maxImpact or 0.0)
    safeStore("ngp_road_input_dominant_id", st.dominantPhaseId or 0)
    safeStore("ngp_road_input_dominant", st.dominantPhase or "QUIET")

    safeStore("ngp_rii_avg_hint", st.avgModuleHint or 0.0)
    safeStore("ngp_rii_max_severity", st.maxSeverity or 0.0)
    safeStore("ngp_rii_dominant_id", st.dominantPhaseId or 0)
    safeStore("ngp_rii_dominant", st.dominantPhase or "QUIET")

    if not st.debugStoreNow then
        return
    end

    safeStore("ngp_road_input_speed_kmh", st.speedKmh or 0.0)
    safeStore("ngp_road_input_body_heave", st.bodyHeave or 0.0)
    safeStore("ngp_road_input_body_pitch", st.bodyPitch or 0.0)
    safeStore("ngp_road_input_body_roll", st.bodyRoll or 0.0)
    safeStore("ngp_road_input_body_yaw", st.bodyYawHint or 0.0)

    safeStore("ngp_road_input_body_linked", st.roadBodyLinked and 1 or 0)
    safeStore("ngp_road_input_contact_linked", st.contactLinked and 1 or 0)
    safeStore("ngp_road_input_damper_linked", st.damperLinked and 1 or 0)
    safeStore("ngp_road_input_hop_linked", st.hopLinked and 1 or 0)
    safeStore("ngp_road_input_load_linked", st.loadLinked and 1 or 0)
    safeStore("ngp_road_input_suspension_linked", st.suspensionLinked and 1 or 0)
    safeStore("ngp_road_input_load_path_linked", st.loadPathLinked and 1 or 0)
    safeStore("ngp_road_input_compliance_linked", st.complianceLinked and 1 or 0)
    safeStore("ngp_road_input_carcass_linked", st.carcassLinked and 1 or 0)
    safeStore("ngp_road_input_impact_linked", st.impactLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i, M.state.wheels[i])
    end

    exportGlobal()
end

local function decayAll(dt, status)
    M.state.status = status
    M.state.wheelsValid = false

    M.state.bodyHeave = lowPass(M.state.bodyHeave, 0.0, M.params.noCarTau, dt)
    M.state.bodyPitch = lowPass(M.state.bodyPitch, 0.0, M.params.noCarTau, dt)
    M.state.bodyRoll = lowPass(M.state.bodyRoll, 0.0, M.params.noCarTau, dt)
    M.state.bodyYawHint = lowPass(M.state.bodyYawHint, 0.0, M.params.noCarTau, dt)
    M.state.speedKmh = lowPass(M.state.speedKmh, 0.0, M.params.noCarTau, dt)
    M.state.impactSensorValue = lowPass(M.state.impactSensorValue, 0.0, M.params.noCarTau, dt)
    M.state.impactSensorRoot = lowPass(M.state.impactSensorRoot, 0.0, M.params.noCarTau, dt)

    for i = 0, 3 do
        decayWheel(i, dt, status == "NO WHEEL")
    end

    updateAverages()
    exportState()
end

function M.init()
    M.state.status = "INIT"
    M.state.debugStoreNow = true
    exportState()
end

function M.update(dt, car, runtime)
    M.state.updateCount = (M.state.updateCount or 0) + 1

    dt = clamp(safeNumber(dt, 0.0), M.params.minDt, M.params.maxDt)
    updateDebugGate(dt)

    car = car or safeGetCar()

    if not car then
        resetLinkFlags()
        decayAll(dt, "NO CAR")
        return
    end

    if not getWheels(car) then
        resetLinkFlags()
        readBodyInputs(car)
        decayAll(dt, "NO WHEELS")
        return
    end

    M.state.status = "RUNNING"
    M.state.wheelsValid = true
    M.state.speedKmh = getSpeedKmh(car)

    resetLinkFlags()
    readBodyInputs(car)

    for i = 0, 3 do
        local wheel = getWheel(car, i)
        local w = M.state.wheels[i]

        if wheel then
            w.active = true
            readWheelInputs(i, w, wheel, dt)
            updateWheelInterpretation(i, w, dt)
        else
            decayWheel(i, dt, true)
        end

        exportWheel(i, w)
    end

    updateAverages()
    exportGlobal()
end

function M.getWheel(index)
    return M.state.wheels[index]
end

function M.getRoadNoise(index)
    local w = M.state.wheels[index]
    return w and w.roadNoise or 0.0
end

function M.getImpact(index)
    local w = M.state.wheels[index]
    return w and w.roadImpact or 0.0
end

function M.getTexture(index)
    local w = M.state.wheels[index]
    return w and w.roadTexture or 0.0
end

function M.getPathLoss(index)
    local w = M.state.wheels[index]
    return w and w.pathLossEvent or 0.0
end

function M.getSeverity(index)
    local w = M.state.wheels[index]
    return w and w.severity or 0.0
end

function M.getModuleHint(index)
    local w = M.state.wheels[index]
    return w and w.moduleHint or 0.0
end

function M.getPhase(index)
    local w = M.state.wheels[index]
    return w and w.phaseText or "UNKNOWN"
end

function M.getDominantPhase()
    return M.state.dominantPhase or "QUIET", M.state.dominantPhaseId or 0
end

function M.getState(index)
    if index ~= nil then
        return M.state.wheels[index]
    end
    return M.state
end

function M.debugStr(index)
    local i = tonumber(index or 0) or 0
    local w = M.state.wheels[i] or M.state.wheels[0]

    return string.format(
        "Status %s / Count %.0f / Wheels %s / Dom %s\n" ..
        "%s Phase %s / Sev %.2f / Hint %.2f / Noise %.2f / Impact %.2f\n" ..
        "Kerb %.2f / Unload %.2f / Land %.2f / Bottom %.2f / Hop %.2f\n" ..
        "Texture %.2f / Shock %.2f / Path %.2f / Surf %.2f\n" ..
        "Load %.0f / dL %.0f / CQ %.2f / Trust %.2f / VFlow %.2f\n" ..
        "Links RBI:%s CQ:%s DH:%s LP:%s CS:%s TC:%s Hop:%s Impact:%s",
        tostring(M.state.status),
        M.state.updateCount or 0,
        M.state.wheelsValid and "OK" or "NIL",
        tostring(M.state.dominantPhase or "QUIET"),

        tostring(WHEEL_NAMES[i] or i),
        tostring(w.phaseText or "UNKNOWN"),
        w.severity or 0.0,
        w.moduleHint or 0.0,
        w.roadNoise or 0.0,
        w.roadImpact or 0.0,

        w.kerbLike or 0.0,
        w.unloadEvent or 0.0,
        w.landingEvent or 0.0,
        w.bottomingRisk or 0.0,
        w.hopRisk or 0.0,

        w.roadTexture or 0.0,
        w.roadShock or 0.0,
        w.pathLossEvent or 0.0,
        w.surfaceLimit or 0.0,

        w.load or 0.0,
        w.loadRate or 0.0,
        w.contactQuality or 0.0,
        w.contactTrust or 0.0,
        w.verticalFlow or 0.0,

        M.state.roadBodyLinked and "OK" or "NIL",
        M.state.contactLinked and "OK" or "NIL",
        M.state.damperLinked and "OK" or "NIL",
        M.state.loadPathLinked and "OK" or "NIL",
        M.state.complianceLinked and "OK" or "NIL",
        M.state.carcassLinked and "OK" or "NIL",
        M.state.hopLinked and "OK" or "NIL",
        M.state.impactLinked and "OK" or "NIL"
    )
end

return M
