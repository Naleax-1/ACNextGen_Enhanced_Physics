---@diagnostic disable: undefined-global

--============================================================
-- damper_hysteresis.lua
-- ACNextGen V1.1.5 Stable
-- Damper hysteresis / compression-rebound memory bridge
--============================================================

local M = {}

local WHEEL_NAMES = {
    [0] = "FL",
    [1] = "FR",
    [2] = "RL",
    [3] = "RR",
}

local PHASE_TEXT = {
    [0]  = "INIT",
    [1]  = "HOLD",
    [2]  = "BUMP_LOW",
    [3]  = "REBOUND_LOW",
    [4]  = "BUMP_HIGH",
    [5]  = "REBOUND_HIGH",
    [6]  = "REVERSAL",
    [7]  = "IMPACT",
    [8]  = "UNLOADED",
    [9]  = "ROAD_MEMORY",
    [10] = "HYSTERETIC",
}

M.params = {
    invertVelocity = false,
    deriveVelocityFromTravel = true,

    deadVelocity = 0.006,
    lowSpeedVelocity = 0.030,
    highSpeedVelocity = 0.180,
    impactVelocity = 0.520,

    accelRef = 3.50,
    reversalVelocity = 0.018,
    reversalHoldTime = 0.080,

    lowSpeedBumpDamping = 0.62,
    lowSpeedReboundDamping = 0.70,
    highSpeedBumpDamping = 0.92,
    highSpeedReboundDamping = 1.00,

    hystereticVerticalGain = 0.78,
    verticalDampingGain = 0.45,
    ringLagGain = 0.34,
    beltLagGain = 0.42,

    ringTauRise = 0.040,
    ringTauFall = 0.180,
    beltTauRise = 0.070,
    beltTauFall = 0.260,

    heatBuildGain = 0.42,
    heatCoolGain = 0.16,

    roadMemoryGain = 0.28,
    roadMemoryTauRise = 0.050,
    roadMemoryTauFall = 0.480,

    velocityTau = 0.045,
    accelTau = 0.060,

    bumpTau = 0.110,
    reboundTau = 0.130,
    impactTau = 0.050,

    memoryTauRise = 0.075,
    memoryTauFall = 0.420,

    hystTauRise = 0.055,
    hystTauFall = 0.260,

    forceRef = 4500.0,
    workScale = 0.000020,

    velocityBuildGain = 0.66,
    forceBuildGain = 0.16,
    impactBuildGain = 0.42,
    reversalBuildGain = 0.24,
    contactLossBuildGain = 0.18,
    roadInputBuildGain = 0.22,

    memoryHystGain = 0.35,
    energyHystGain = 0.24,
    impactHystGain = 0.16,
    reversalHystGain = 0.10,
    heatHystGain = 0.10,
    carcassHystGain = 0.12,

    unloadedQuality = 0.18,
    unloadedLoad = 120.0,

    contactTrustGain = 0.22,
    carcassSupportGain = 0.18,
    carcassGripGateGain = 0.16,
    carcassVerticalGain = 0.18,
    carcassHysteresisReadGain = 0.12,

    maxMemory = 1.0,
    maxEnergy = 1.0,
    maxHysteresis = 1.0,
    maxHeat = 1.0,

    loadRef = 3000.0,
    minDt = 0.00005,
    maxDt = 0.050,

    outputEnabled = true,
    debugStoreInterval = 0.25,
}

local function newWheel()
    return {
        active = false,
        initialized = false,

        travel = 0.0,
        prevTravel = 0.0,

        rawVelocity = 0.0,
        velocity = 0.0,
        prevVelocity = 0.0,
        accel = 0.0,

        force = 0.0,
        proxyForce = 0.0,
        absVelocity = 0.0,

        bump = 0.0,
        rebound = 0.0,
        highSpeed = 0.0,
        impact = 0.0,
        reversal = 0.0,
        reversalTimer = 0.0,

        lowSpeedBand = 0.0,
        highSpeedBand = 0.0,

        lowSpeedDamping = 0.0,
        highSpeedDamping = 0.0,
        verticalDamping = 0.0,
        hystereticDamping = 0.0,

        ringLag = 0.0,
        beltLag = 0.0,
        verticalPulse = 0.0,

        work = 0.0,
        energy = 0.0,
        memory = 0.0,
        roadMemory = 0.0,
        dampingHeat = 0.0,
        hysteresis = 0.0,
        bias = 0.0,

        contactQuality = 1.0,
        contactTrust = 1.0,
        contactStability = 1.0,
        contactSlip = 0.0,
        roadInput = 0.0,
        load = 0.0,

        carcassSupport = 1.0,
        carcassGripGate = 1.0,
        carcassVerticalNorm = 0.0,
        carcassHysteresis = 0.0,
        carcassRecoveryBias = 0.0,

        phaseId = 0,
        phaseText = "INIT",
    }
end

M.state = {
    wheels = {},

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    avgHysteresis = 0.0,
    avgMemory = 0.0,
    avgAbsVelocity = 0.0,
    avgImpact = 0.0,
    avgHeat = 0.0,
    avgRoadMemory = 0.0,
    avgVerticalPulse = 0.0,
    avgVerticalDamping = 0.0,

    frontBias = 0.0,
    rearBias = 0.0,
    leftBias = 0.0,
    rightBias = 0.0,

    maxImpact = 0.0,
    maxAbsVelocity = 0.0,

    damperLinked = false,
    contactLinked = false,
    carcassLinked = false,
    roadLinked = false,
    travelLinked = false,
    loadLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

for i = 0, 3 do
    M.state.wheels[i] = newWheel()
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
    v = safeNumber(v, minValue)
    if v < minValue then return minValue end
    if v > maxValue then return maxValue end
    return v
end

local function abs(v)
    return math.abs(safeNumber(v, 0.0))
end

local function sign(v)
    v = safeNumber(v, 0.0)
    if v > 0.000001 then return 1 end
    if v < -0.000001 then return -1 end
    return 0
end

local function smoothstep(edge0, edge1, x)
    if edge0 == edge1 then
        return x >= edge1 and 1.0 or 0.0
    end
    local t = clamp((safeNumber(x, 0.0) - edge0) / (edge1 - edge0), 0.0, 1.0)
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

    return current + (target - current) * clamp(dt / math.max(tau + dt, 0.0001), 0.0, 1.0)
end

local function approach(current, target, riseTau, fallTau, dt)
    local tau = target > current and riseTau or fallTau
    return lowPass(current, target, tau, dt)
end

local function safeStore(key, value)
    if not M.params.outputEnabled then return end
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

local function safeGetCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function()
        return ac.getCar(0)
    end)
    if not ok then return nil end
    return car
end

local function safeWheel(car, index)
    if not car then return nil end
    local wheels = safeField(car, "wheels", nil)
    if not wheels then return nil end
    local ok, wheel = pcall(function()
        return wheels[index] or wheels[index + 1]
    end)
    if not ok then return nil end
    return wheel
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

local function resetLinkFlags()
    M.state.damperLinked = false
    M.state.contactLinked = false
    M.state.carcassLinked = false
    M.state.roadLinked = false
    M.state.travelLinked = false
    M.state.loadLinked = false
end

local function readWheelLoad(wheel, index)
    if wheel then
        local load =
            safeField(wheel, "load", nil)
            or safeField(wheel, "loadK", nil)
            or nil
            or nil

        if load ~= nil then
            M.state.loadLinked = true
            return abs(load)
        end
    end

    local contactLoad = safeLoadRaw("ngp_contact_load_" .. index)
    if contactLoad ~= nil then
        M.state.contactLinked = true
        M.state.loadLinked = true
        return abs(contactLoad)
    end

    local tireStateLoad = safeLoadRaw("ngp_tire_state_load_" .. index)
    if tireStateLoad ~= nil then
        M.state.loadLinked = true
        return abs(tireStateLoad)
    end

    local carcassLoad = safeLoadRaw("ngp_tire_carcass_load_" .. index)
    if carcassLoad ~= nil then
        M.state.carcassLinked = true
        M.state.loadLinked = true
        return abs(carcassLoad)
    end

    local wheelLoad = safeLoadRaw("ngp_wheel_load_" .. index)
    if wheelLoad ~= nil then
        M.state.loadLinked = true
        return abs(wheelLoad)
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        M.state.loadLinked = true
        return math.max(0.0, M.params.loadRef + safeNumber(dlt, 0.0))
    end

    local sprung = safeLoadRaw("ngp_sprung_load_" .. index)
    if sprung ~= nil then
        M.state.loadLinked = true
        return abs(sprung)
    end

    return 0.0
end

local function readTravel(wheel, index)
    local v, key = safeLoadAlt(0.0,
        "ngp_damper_travel_" .. index,
        "ngp_susp_damper_travel_" .. index,
        "ngp_susp_travel_" .. index
    )

    if key then
        M.state.travelLinked = true
        return v
    end

    if wheel then
        local fields = {
            "damperTravel",
            "suspensionTravel",
            "travel",
            "suspensionLength",
            "rideHeight",
        }

        for i = 1, #fields do
            local fieldValue = safeField(wheel, fields[i], nil)
            if fieldValue ~= nil then
                M.state.travelLinked = true
                return safeNumber(fieldValue, 0.0)
            end
        end
    end

    return 0.0
end

local function readDamperVelocity(wheelState, wheel, index, dt)
    local v, key = safeLoadAlt(0.0,
        "ngp_damper_velocity_" .. index,
        "ngp_damper_vel_" .. index,
        "ngp_susp_damper_velocity_" .. index,
        "ngp_susp_velocity_" .. index,
        "ngp_susp_speed_" .. index
    )

    if key then
        M.state.damperLinked = true
    else
        local travel = readTravel(wheel, index)
        wheelState.travel = travel

        if M.params.deriveVelocityFromTravel and wheelState.initialized then
            v = (travel - (wheelState.prevTravel or travel)) / math.max(dt, 0.001)
        else
            v = 0.0
        end
    end

    if M.params.invertVelocity then
        v = -v
    end

    return safeNumber(v, 0.0)
end

local function readDamperForce(index)
    local f, key = safeLoadAlt(0.0,
        "ngp_damper_force_" .. index,
        "ngp_susp_external_damper_" .. index,
        "ngp_susp_int_damper_" .. index,
        "ngp_susp_integrated_force_" .. index,
        "ngp_susp_int_force_" .. index
    )

    if key then
        M.state.damperLinked = true
        return f
    end

    return 0.0
end

local function readContactInputs(wheelState, index)
    local q = safeLoadRaw("ngp_contact_quality_" .. index)
    if q == nil then q = safeLoadRaw("ngp_tire_contact_quality_" .. index) end
    if q == nil then q = safeLoadRaw("ngp_tcr_quality_" .. index) end

    local trust = safeLoadRaw("ngp_contact_trust_" .. index)
    local stability = safeLoadRaw("ngp_contact_stability_score_" .. index)
    local slip = safeLoadRaw("ngp_contact_combined_slip_" .. index)
    local loss = safeLoadRaw("ngp_contact_loss_" .. index)
    if loss == nil then loss = safeLoadRaw("ngp_tire_contact_loss_" .. index) end
    if loss == nil then loss = safeLoadRaw("ngp_tcr_contact_loss_" .. index) end

    if q ~= nil or trust ~= nil or stability ~= nil or slip ~= nil or loss ~= nil then
        M.state.contactLinked = true
    end

    if q == nil and loss ~= nil then
        q = 1.0 - safeNumber(loss, 0.0)
    end

    wheelState.contactQuality = clamp(safeNumber(q, 1.0), 0.0, 1.2)
    wheelState.contactTrust = clamp(safeNumber(trust, wheelState.contactQuality), 0.0, 1.2)
    wheelState.contactStability = clamp(safeNumber(stability, wheelState.contactTrust), 0.0, 1.0)
    wheelState.contactSlip = clamp(safeNumber(slip, 0.0), 0.0, 2.0)
end

local function readCarcassInputs(wheelState, index)
    local support, k0 = safeLoadAlt(1.0,
        "ngp_carcass_support_" .. index,
        "ngp_tire_carcass_support_" .. index,
        "ngp_contact_carcass_support_" .. index
    )

    local gate, k1 = safeLoadAlt(1.0,
        "ngp_carcass_grip_gate_" .. index,
        "ngp_tire_carcass_grip_gate_" .. index,
        "ngp_contact_grip_gate_" .. index
    )

    local vertical, k2 = safeLoadAlt(0.0,
        "ngp_carcass_vertical_norm_" .. index,
        "ngp_tire_carcass_vertical_norm_" .. index,
        "ngp_carcass_vertical_deflect_" .. index,
        "ngp_tire_carcass_crush_" .. index,
        "ngp_damper_carcass_vertical_" .. index
    )

    if not k2 then
        local score = safeLoadRaw("ngp_contact_vertical_score_" .. index)
        if score ~= nil then
            vertical = 1.0 - clamp(safeNumber(score, 1.0), 0.0, 1.0)
            k2 = "ngp_contact_vertical_score_" .. index
        end
    end

    local hyst, k3 = safeLoadAlt(0.0,
        "ngp_carcass_hysteresis_" .. index,
        "ngp_tire_carcass_hyst_" .. index,
        "ngp_tire_carcass_hysteresis_" .. index,
        "ngp_contact_hysteresis_" .. index
    )

    local recBias, k4 = safeLoadAlt(0.0,
        "ngp_carcass_recovery_bias_" .. index,
        "ngp_tire_return_force_" .. index,
        "ngp_contact_recovery_bias_" .. index
    )

    if k0 or k1 or k2 or k3 or k4 then
        M.state.carcassLinked = true
    end

    wheelState.carcassSupport = clamp(support, 0.0, 1.2)
    wheelState.carcassGripGate = clamp(gate, 0.0, 1.2)
    wheelState.carcassVerticalNorm = clamp(vertical, 0.0, 1.5)
    wheelState.carcassHysteresis = clamp(hyst, 0.0, 1.2)
    wheelState.carcassRecoveryBias = clamp(recBias, 0.0, 1.2)
end

local function readRoadInput(index)
    local wheelInput, key = safeLoadAlt(0.0,
        "ngp_sci_road_input_" .. index,
        "ngp_susp_contact_input_" .. index,
        "ngp_road_input_" .. index,
        "ngp_road_input_wheel_" .. index,
        "ngp_load_path_road_input_" .. index
    )

    if key then
        M.state.roadLinked = true
        return clamp(abs(wheelInput), 0.0, 1.0)
    end

    local heave = abs(safeLoad("ngp_body_heave_input", 0.0))
    local pitch = abs(safeLoad("ngp_body_pitch_input", 0.0))
    local roll = abs(safeLoad("ngp_body_roll_input", 0.0))
    local input = clamp(heave + pitch * 0.5 + roll * 0.5, 0.0, 1.0)

    if input > 0.0 then
        M.state.roadLinked = true
    end

    return input
end

local function classifyPhase(wheelState)
    if wheelState.load < M.params.unloadedLoad or wheelState.contactQuality < M.params.unloadedQuality then
        return 8, PHASE_TEXT[8]
    end

    if wheelState.impact > 0.70 then
        return 7, PHASE_TEXT[7]
    end

    if wheelState.reversal > 0.35 then
        return 6, PHASE_TEXT[6]
    end

    if wheelState.roadMemory > 0.50 and wheelState.absVelocity < M.params.highSpeedVelocity then
        return 9, PHASE_TEXT[9]
    end

    if wheelState.hystereticDamping > 0.58 then
        return 10, PHASE_TEXT[10]
    end

    if wheelState.absVelocity < M.params.deadVelocity then
        return 1, PHASE_TEXT[1]
    end

    if wheelState.velocity >= M.params.highSpeedVelocity then
        return 4, PHASE_TEXT[4]
    end

    if wheelState.velocity <= -M.params.highSpeedVelocity then
        return 5, PHASE_TEXT[5]
    end

    if wheelState.velocity > 0.0 then
        return 2, PHASE_TEXT[2]
    end

    return 3, PHASE_TEXT[3]
end

local function updateWheel(index, wheelState, wheel, dt)
    wheelState.active = true

    local travel = readTravel(wheel, index)
    wheelState.travel = travel

    local rawVelocity = readDamperVelocity(wheelState, wheel, index, dt)
    local force = readDamperForce(index)

    readContactInputs(wheelState, index)
    readCarcassInputs(wheelState, index)

    wheelState.roadInput = readRoadInput(index)
    wheelState.load = readWheelLoad(wheel, index)

    wheelState.rawVelocity = rawVelocity
    wheelState.velocity = lowPass(wheelState.velocity, rawVelocity, M.params.velocityTau, dt)

    local rawAccel = (wheelState.velocity - (wheelState.prevVelocity or 0.0)) / math.max(dt, 0.001)
    wheelState.accel = lowPass(wheelState.accel, rawAccel, M.params.accelTau, dt)

    wheelState.force = force
    wheelState.absVelocity = abs(wheelState.velocity)

    local prevSign = sign(wheelState.prevVelocity)
    local nowSign = sign(wheelState.velocity)

    local reversed =
        prevSign ~= 0
        and nowSign ~= 0
        and prevSign ~= nowSign
        and wheelState.absVelocity > M.params.reversalVelocity

    if reversed then
        wheelState.reversalTimer = M.params.reversalHoldTime
    else
        wheelState.reversalTimer = math.max(0.0, (wheelState.reversalTimer or 0.0) - dt)
    end

    local reversalTarget = 0.0
    if wheelState.reversalTimer > 0.0 then
        reversalTarget = clamp(wheelState.absVelocity / math.max(M.params.highSpeedVelocity, 0.001), 0.0, 1.0)
    end

    wheelState.reversal = approach(wheelState.reversal, reversalTarget, 0.025, 0.140, dt)

    local bumpTarget = 0.0
    local reboundTarget = 0.0

    if wheelState.velocity > M.params.deadVelocity then
        bumpTarget = clamp(wheelState.velocity / math.max(M.params.highSpeedVelocity, 0.001), 0.0, 1.0)
    elseif wheelState.velocity < -M.params.deadVelocity then
        reboundTarget = clamp(-wheelState.velocity / math.max(M.params.highSpeedVelocity, 0.001), 0.0, 1.0)
    end

    wheelState.bump = approach(wheelState.bump, bumpTarget, M.params.bumpTau, M.params.bumpTau * 1.4, dt)
    wheelState.rebound = approach(wheelState.rebound, reboundTarget, M.params.reboundTau, M.params.reboundTau * 1.4, dt)

    wheelState.highSpeed = smoothstep(M.params.highSpeedVelocity, M.params.impactVelocity, wheelState.absVelocity)

    local accelImpact = smoothstep(M.params.accelRef, M.params.accelRef * 2.5, abs(wheelState.accel))
    local velocityImpact = smoothstep(M.params.highSpeedVelocity, M.params.impactVelocity, wheelState.absVelocity)
    local contactImpact = 1.0 - wheelState.contactStability
    local contactLoss = 1.0 - clamp(wheelState.contactQuality, 0.0, 1.0)

    local impactTarget = clamp(
        velocityImpact * 0.42
        + accelImpact * 0.28
        + contactImpact * 0.14
        + wheelState.roadInput * 0.10
        + wheelState.carcassVerticalNorm * 0.08,
        0.0,
        1.0
    )

    wheelState.impact = approach(wheelState.impact, impactTarget, M.params.impactTau, M.params.impactTau * 3.0, dt)

    local velocityNorm = clamp(wheelState.absVelocity / math.max(M.params.highSpeedVelocity, 0.001), 0.0, 1.0)
    local forceNorm = clamp(abs(wheelState.force) / math.max(M.params.forceRef, 1.0), 0.0, 1.0)

    wheelState.lowSpeedBand = clamp(
        1.0 - smoothstep(M.params.lowSpeedVelocity, M.params.highSpeedVelocity, wheelState.absVelocity),
        0.0,
        1.0
    )

    wheelState.highSpeedBand = clamp(
        smoothstep(M.params.lowSpeedVelocity, M.params.impactVelocity, wheelState.absVelocity),
        0.0,
        1.0
    )

    local bumpDamp =
        wheelState.lowSpeedBand * M.params.lowSpeedBumpDamping
        + wheelState.highSpeedBand * M.params.highSpeedBumpDamping

    local reboundDamp =
        wheelState.lowSpeedBand * M.params.lowSpeedReboundDamping
        + wheelState.highSpeedBand * M.params.highSpeedReboundDamping

    wheelState.lowSpeedDamping = clamp(wheelState.lowSpeedBand * (wheelState.bump + wheelState.rebound) * 0.5, 0.0, 1.0)
    wheelState.highSpeedDamping = clamp(wheelState.highSpeedBand * (wheelState.bump + wheelState.rebound) * 0.5, 0.0, 1.0)

    local selectedDamp = wheelState.velocity >= 0.0 and bumpDamp or reboundDamp

    local verticalDampingTarget = clamp(
        velocityNorm * M.params.verticalDampingGain
        + selectedDamp * 0.34
        + wheelState.highSpeedDamping * 0.24
        + wheelState.carcassVerticalNorm * M.params.carcassVerticalGain,
        0.0,
        1.0
    )

    wheelState.verticalDamping = approach(wheelState.verticalDamping, verticalDampingTarget, 0.050, 0.180, dt)

    local ringTarget = clamp(
        wheelState.highSpeed * M.params.ringLagGain
        + wheelState.reversal * 0.20
        + wheelState.impact * 0.28
        + wheelState.roadInput * 0.14,
        0.0,
        1.0
    )

    wheelState.ringLag = approach(wheelState.ringLag, ringTarget, M.params.ringTauRise, M.params.ringTauFall, dt)

    local beltTarget = clamp(
        wheelState.verticalDamping * M.params.beltLagGain
        + wheelState.impact * 0.24
        + wheelState.carcassHysteresis * 0.26
        + wheelState.roadMemory * 0.18,
        0.0,
        1.0
    )

    wheelState.beltLag = approach(wheelState.beltLag, beltTarget, M.params.beltTauRise, M.params.beltTauFall, dt)

    local roadMemoryTarget = clamp(
        wheelState.roadInput * M.params.roadMemoryGain
        + wheelState.impact * 0.26
        + wheelState.highSpeedDamping * 0.18
        + contactLoss * 0.12,
        0.0,
        1.0
    )

    wheelState.roadMemory = approach(
        wheelState.roadMemory,
        roadMemoryTarget,
        M.params.roadMemoryTauRise,
        M.params.roadMemoryTauFall,
        dt
    )

    local heatTarget = clamp(
        abs(wheelState.velocity * selectedDamp) * M.params.heatBuildGain
        + wheelState.work * 0.20
        + wheelState.impact * 0.18
        + wheelState.roadMemory * 0.10,
        0.0,
        M.params.maxHeat
    )

    wheelState.dampingHeat = clamp(
        wheelState.dampingHeat
        + (heatTarget * M.params.heatBuildGain - M.params.heatCoolGain * (0.20 + wheelState.dampingHeat)) * dt,
        0.0,
        M.params.maxHeat
    )

    local hystereticDampingTarget = clamp(
        wheelState.verticalDamping * 0.32
        + wheelState.ringLag * 0.20
        + wheelState.beltLag * 0.22
        + wheelState.reversal * 0.12
        + wheelState.carcassHysteresis * M.params.carcassHysteresisReadGain
        + contactLoss * 0.10,
        0.0,
        1.0
    )

    wheelState.hystereticDamping = approach(
        wheelState.hystereticDamping,
        hystereticDampingTarget,
        M.params.hystTauRise,
        M.params.hystTauFall,
        dt
    )

    local proxyForce = selectedDamp * wheelState.velocity * M.params.forceRef
    if wheelState.force ~= 0.0 then
        proxyForce = wheelState.force
    end

    wheelState.proxyForce = proxyForce

    local work = abs(proxyForce * wheelState.velocity) * M.params.workScale
    wheelState.work = clamp(work, 0.0, 1.0)

    local energyTarget = clamp(
        velocityNorm * 0.38
        + forceNorm * 0.16
        + wheelState.verticalDamping * 0.16
        + wheelState.impact * 0.16
        + wheelState.roadInput * 0.08
        + contactLoss * 0.06,
        0.0,
        1.0
    )

    wheelState.energy = approach(wheelState.energy, energyTarget, 0.070, 0.260, dt)

    local memoryTarget = clamp(
        velocityNorm * M.params.velocityBuildGain
        + forceNorm * M.params.forceBuildGain
        + wheelState.impact * M.params.impactBuildGain
        + wheelState.reversal * M.params.reversalBuildGain
        + contactLoss * M.params.contactLossBuildGain
        + wheelState.roadInput * M.params.roadInputBuildGain
        + wheelState.hystereticDamping * 0.22
        + wheelState.dampingHeat * 0.16,
        0.0,
        M.params.maxMemory
    )

    wheelState.memory = approach(wheelState.memory, memoryTarget, M.params.memoryTauRise, M.params.memoryTauFall, dt)
    wheelState.bias = clamp(wheelState.bump - wheelState.rebound, -1.0, 1.0)

    local contactSupport =
        clamp(wheelState.contactTrust, 0.0, 1.0) * M.params.contactTrustGain
        + clamp(wheelState.carcassSupport, 0.0, 1.0) * M.params.carcassSupportGain
        + clamp(wheelState.carcassGripGate, 0.0, 1.0) * M.params.carcassGripGateGain

    wheelState.verticalPulse = clamp(
        wheelState.impact * 0.42
        + wheelState.highSpeedDamping * 0.22
        + wheelState.reversal * 0.16
        + wheelState.roadMemory * 0.20,
        0.0,
        1.0
    ) * clamp(0.60 + contactSupport, 0.0, 1.0)

    wheelState.hysteresis = clamp(
        wheelState.memory * M.params.memoryHystGain
        + abs(wheelState.bias) * M.params.energyHystGain
        + wheelState.impact * M.params.impactHystGain
        + wheelState.reversal * M.params.reversalHystGain
        + wheelState.dampingHeat * M.params.heatHystGain
        + wheelState.carcassHysteresis * M.params.carcassHystGain,
        0.0,
        M.params.maxHysteresis
    )

    local phaseId, phaseText = classifyPhase(wheelState)
    wheelState.phaseId = phaseId
    wheelState.phaseText = phaseText

    wheelState.prevVelocity = wheelState.velocity
    wheelState.prevTravel = wheelState.travel
    wheelState.initialized = true
end

local function clearWheel(index, dt)
    local w = M.state.wheels[index]
    if not w then return end

    w.active = false
    w.rawVelocity = 0.0
    w.velocity = lowPass(w.velocity, 0.0, 0.120, dt)
    w.accel = lowPass(w.accel, 0.0, 0.120, dt)
    w.force = 0.0
    w.proxyForce = 0.0
    w.absVelocity = abs(w.velocity)
    w.bump = lowPass(w.bump, 0.0, 0.180, dt)
    w.rebound = lowPass(w.rebound, 0.0, 0.180, dt)
    w.highSpeed = lowPass(w.highSpeed, 0.0, 0.180, dt)
    w.impact = lowPass(w.impact, 0.0, 0.180, dt)
    w.reversal = lowPass(w.reversal, 0.0, 0.180, dt)
    w.lowSpeedDamping = lowPass(w.lowSpeedDamping, 0.0, 0.180, dt)
    w.highSpeedDamping = lowPass(w.highSpeedDamping, 0.0, 0.180, dt)
    w.verticalDamping = lowPass(w.verticalDamping, 0.0, 0.220, dt)
    w.hystereticDamping = lowPass(w.hystereticDamping, 0.0, 0.260, dt)
    w.ringLag = lowPass(w.ringLag, 0.0, 0.220, dt)
    w.beltLag = lowPass(w.beltLag, 0.0, 0.260, dt)
    w.verticalPulse = lowPass(w.verticalPulse, 0.0, 0.180, dt)
    w.energy = lowPass(w.energy, 0.0, 0.260, dt)
    w.memory = lowPass(w.memory, 0.0, 0.420, dt)
    w.roadMemory = lowPass(w.roadMemory, 0.0, 0.480, dt)
    w.dampingHeat = lowPass(w.dampingHeat, 0.0, 0.600, dt)
    w.hysteresis = lowPass(w.hysteresis, 0.0, 0.260, dt)
    w.contactQuality = lowPass(w.contactQuality, 0.0, 0.180, dt)
    w.contactTrust = lowPass(w.contactTrust, 0.0, 0.180, dt)
    w.load = lowPass(w.load, 0.0, 0.180, dt)
    w.phaseId = 0
    w.phaseText = "NO WHEEL"
end

local function exportWheel(index, w)
    safeStore("ngp_damper_hyst_" .. index, w.hysteresis or 0.0)
    safeStore("ngp_damper_hyst_memory_" .. index, w.memory or 0.0)
    safeStore("ngp_damper_hyst_velocity_" .. index, w.velocity or 0.0)
    safeStore("ngp_damper_hyst_abs_velocity_" .. index, w.absVelocity or 0.0)
    safeStore("ngp_damper_hyst_phase_id_" .. index, w.phaseId or 0)
    safeStore("ngp_damper_hyst_bias_" .. index, w.bias or 0.0)
    safeStore("ngp_damper_hyst_bump_" .. index, w.bump or 0.0)
    safeStore("ngp_damper_hyst_rebound_" .. index, w.rebound or 0.0)
    safeStore("ngp_damper_hyst_impact_" .. index, w.impact or 0.0)
    safeStore("ngp_damper_hyst_reversal_" .. index, w.reversal or 0.0)

    safeStore("ngp_damper_low_speed_" .. index, w.lowSpeedDamping or 0.0)
    safeStore("ngp_damper_high_speed_" .. index, w.highSpeedDamping or 0.0)
    safeStore("ngp_damper_vertical_" .. index, w.verticalDamping or 0.0)
    safeStore("ngp_damper_hysteretic_" .. index, w.hystereticDamping or 0.0)
    safeStore("ngp_damper_ring_lag_" .. index, w.ringLag or 0.0)
    safeStore("ngp_damper_belt_lag_" .. index, w.beltLag or 0.0)
    safeStore("ngp_damper_vertical_pulse_" .. index, w.verticalPulse or 0.0)
    safeStore("ngp_damper_heat_" .. index, w.dampingHeat or 0.0)
    safeStore("ngp_damper_road_memory_" .. index, w.roadMemory or 0.0)

    safeStore("ngp_hyst_damper_" .. index, w.hysteresis or 0.0)
    safeStore("ngp_road_vertical_pulse_" .. index, w.verticalPulse or 0.0)
    safeStore("ngp_road_damper_memory_" .. index, w.roadMemory or 0.0)
    safeStore("ngp_vertical_damping_" .. index, w.verticalDamping or 0.0)
    safeStore("ngp_hysteretic_damping_" .. index, w.hystereticDamping or 0.0)

    safeStore("ngp_damper_memory_" .. index, w.memory or 0.0)
    safeStore("ngp_damper_energy_" .. index, w.energy or 0.0)
    safeStore("ngp_damper_impact_" .. index, w.impact or 0.0)
    safeStore("ngp_damper_phase_id_" .. index, w.phaseId or 0)

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_damper_hyst_phase_" .. index, w.phaseText or "UNKNOWN")
    safeStore("ngp_damper_hyst_raw_velocity_" .. index, w.rawVelocity or 0.0)
    safeStore("ngp_damper_hyst_accel_" .. index, w.accel or 0.0)
    safeStore("ngp_damper_hyst_force_" .. index, w.force or 0.0)
    safeStore("ngp_damper_proxy_force_" .. index, w.proxyForce or 0.0)
    safeStore("ngp_damper_hyst_work_" .. index, w.work or 0.0)
    safeStore("ngp_damper_hyst_energy_" .. index, w.energy or 0.0)
    safeStore("ngp_damper_hyst_highspeed_" .. index, w.highSpeed or 0.0)
    safeStore("ngp_damper_low_band_" .. index, w.lowSpeedBand or 0.0)
    safeStore("ngp_damper_high_band_" .. index, w.highSpeedBand or 0.0)
    safeStore("ngp_damper_hyst_contact_quality_" .. index, w.contactQuality or 1.0)
    safeStore("ngp_damper_hyst_contact_trust_" .. index, w.contactTrust or 1.0)
    safeStore("ngp_damper_hyst_contact_stability_" .. index, w.contactStability or 1.0)
    safeStore("ngp_damper_hyst_road_input_" .. index, w.roadInput or 0.0)
    safeStore("ngp_damper_hyst_load_" .. index, w.load or 0.0)
    safeStore("ngp_damper_carcass_vertical_" .. index, w.carcassVerticalNorm or 0.0)
    safeStore("ngp_damper_carcass_hyst_" .. index, w.carcassHysteresis or 0.0)
end

local function exportGlobal()
    safeStore("ngp_damper_hyst_status", M.state.status or "UNKNOWN")
    safeStore("ngp_damper_hyst_update_count", M.state.updateCount or 0)
    safeStore("ngp_damper_hyst_wheels_valid", M.state.wheelsValid and 1 or 0)

    safeStore("ngp_damper_hyst_avg", M.state.avgHysteresis or 0.0)
    safeStore("ngp_damper_hyst_avg_memory", M.state.avgMemory or 0.0)
    safeStore("ngp_damper_hyst_avg_abs_velocity", M.state.avgAbsVelocity or 0.0)
    safeStore("ngp_damper_hyst_avg_impact", M.state.avgImpact or 0.0)
    safeStore("ngp_damper_hyst_avg_heat", M.state.avgHeat or 0.0)
    safeStore("ngp_damper_hyst_avg_road_memory", M.state.avgRoadMemory or 0.0)

    safeStore("ngp_damper_hyst_front_bias", M.state.frontBias or 0.0)
    safeStore("ngp_damper_hyst_rear_bias", M.state.rearBias or 0.0)
    safeStore("ngp_damper_hyst_left_bias", M.state.leftBias or 0.0)
    safeStore("ngp_damper_hyst_right_bias", M.state.rightBias or 0.0)

    safeStore("ngp_damper_avg_hysteresis", M.state.avgHysteresis or 0.0)
    safeStore("ngp_damper_avg_impact", M.state.avgImpact or 0.0)
    safeStore("ngp_damper_avg_heat", M.state.avgHeat or 0.0)
    safeStore("ngp_damper_avg_road_memory", M.state.avgRoadMemory or 0.0)
    safeStore("ngp_damper_avg_vertical_pulse", M.state.avgVerticalPulse or 0.0)
    safeStore("ngp_damper_avg_vertical_damping", M.state.avgVerticalDamping or 0.0)
    safeStore("ngp_damper_max_impact", M.state.maxImpact or 0.0)
    safeStore("ngp_damper_max_abs_velocity", M.state.maxAbsVelocity or 0.0)

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_damper_hyst_damper_linked", M.state.damperLinked and 1 or 0)
    safeStore("ngp_damper_hyst_contact_linked", M.state.contactLinked and 1 or 0)
    safeStore("ngp_damper_hyst_carcass_linked", M.state.carcassLinked and 1 or 0)
    safeStore("ngp_damper_hyst_road_linked", M.state.roadLinked and 1 or 0)
    safeStore("ngp_damper_hyst_travel_linked", M.state.travelLinked and 1 or 0)
    safeStore("ngp_damper_hyst_load_linked", M.state.loadLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i, M.state.wheels[i])
    end
    exportGlobal()
end

function M.init()
    M.state.status = "INIT"
    exportState()
end

local function updateAggregates()
    local fl = M.state.wheels[0]
    local fr = M.state.wheels[1]
    local rl = M.state.wheels[2]
    local rr = M.state.wheels[3]

    M.state.frontBias = ((fl.bias or 0.0) + (fr.bias or 0.0)) * 0.5
    M.state.rearBias = ((rl.bias or 0.0) + (rr.bias or 0.0)) * 0.5
    M.state.leftBias = ((fl.bias or 0.0) + (rl.bias or 0.0)) * 0.5
    M.state.rightBias = ((fr.bias or 0.0) + (rr.bias or 0.0)) * 0.5
end

function M.update(dt, car, runtime)
    M.state.updateCount = (M.state.updateCount or 0) + 1

    dt = safeNumber(dt, M.params.minDt)
    if dt <= 0.0 then
        dt = M.params.minDt
        M.state.status = "BAD DT"
    else
        M.state.status = "RUNNING"
    end

    dt = clamp(dt, M.params.minDt, M.params.maxDt)

    updateDebugGate(dt)

    car = car or safeGetCar()

    if not car then
        M.state.status = "NO CAR"
        M.state.wheelsValid = false
        resetLinkFlags()

        for i = 0, 3 do
            clearWheel(i, dt)
            exportWheel(i, M.state.wheels[i])
        end

        M.state.avgHysteresis = 0.0
        M.state.avgMemory = 0.0
        M.state.avgAbsVelocity = 0.0
        M.state.avgImpact = 0.0
        M.state.avgHeat = 0.0
        M.state.avgRoadMemory = 0.0
        M.state.avgVerticalPulse = 0.0
        M.state.avgVerticalDamping = 0.0
        M.state.maxImpact = 0.0
        M.state.maxAbsVelocity = 0.0
        exportGlobal()
        return
    end

    if not safeField(car, "wheels", nil) then
        M.state.status = "NO WHEELS"
        M.state.wheelsValid = false
        resetLinkFlags()

        for i = 0, 3 do
            clearWheel(i, dt)
            exportWheel(i, M.state.wheels[i])
        end

        exportGlobal()
        return
    end

    M.state.wheelsValid = true
    resetLinkFlags()

    local sumHyst = 0.0
    local sumMemory = 0.0
    local sumAbsVel = 0.0
    local sumImpact = 0.0
    local sumHeat = 0.0
    local sumRoadMemory = 0.0
    local sumPulse = 0.0
    local sumVD = 0.0
    local maxImpact = 0.0
    local maxAbsVelocity = 0.0

    for i = 0, 3 do
        local wheel = safeWheel(car, i)
        local wheelState = M.state.wheels[i]

        if wheel then
            updateWheel(i, wheelState, wheel, dt)
        else
            clearWheel(i, dt)
        end

        sumHyst = sumHyst + (wheelState.hysteresis or 0.0)
        sumMemory = sumMemory + (wheelState.memory or 0.0)
        sumAbsVel = sumAbsVel + (wheelState.absVelocity or 0.0)
        sumImpact = sumImpact + (wheelState.impact or 0.0)
        sumHeat = sumHeat + (wheelState.dampingHeat or 0.0)
        sumRoadMemory = sumRoadMemory + (wheelState.roadMemory or 0.0)
        sumPulse = sumPulse + (wheelState.verticalPulse or 0.0)
        sumVD = sumVD + (wheelState.verticalDamping or 0.0)
        maxImpact = math.max(maxImpact, wheelState.impact or 0.0)
        maxAbsVelocity = math.max(maxAbsVelocity, wheelState.absVelocity or 0.0)

        exportWheel(i, wheelState)
    end

    M.state.avgHysteresis = sumHyst * 0.25
    M.state.avgMemory = sumMemory * 0.25
    M.state.avgAbsVelocity = sumAbsVel * 0.25
    M.state.avgImpact = sumImpact * 0.25
    M.state.avgHeat = sumHeat * 0.25
    M.state.avgRoadMemory = sumRoadMemory * 0.25
    M.state.avgVerticalPulse = sumPulse * 0.25
    M.state.avgVerticalDamping = sumVD * 0.25
    M.state.maxImpact = maxImpact
    M.state.maxAbsVelocity = maxAbsVelocity

    updateAggregates()
    exportGlobal()
end

function M.getHysteresis(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.hysteresis or 0.0
end

function M.getMemory(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.memory or 0.0
end

function M.getImpact(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.impact or 0.0
end

function M.getVerticalPulse(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.verticalPulse or 0.0
end

function M.getVerticalDamping(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.verticalDamping or 0.0
end

function M.getRoadMemory(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.roadMemory or 0.0
end

function M.getPhase(index)
    local wheelState = M.state.wheels[index]
    return wheelState and wheelState.phaseText or "UNKNOWN"
end

function M.getState(index)
    if index == nil then
        return M.state
    end
    return M.state.wheels[index]
end

function M.debugStr(index)
    local i = math.floor(clamp(index or 0, 0, 3))
    local wheelState = M.state.wheels[i] or M.state.wheels[0]

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "Phase %s / Hyst %.3f / Mem %.3f / Heat %.3f\n" ..
        "Vel %+.4f / Acc %+.3f / Force %+.0f\n" ..
        "B %.2f R %.2f HS %.2f Imp %.2f Rev %.2f\n" ..
        "VD %.2f HD %.2f Ring %.2f Belt %.2f Pulse %.2f\n" ..
        "CQ %.2f Trust %.2f Road %.2f Load %.0f\n" ..
        "Links DMP:%s CQ:%s TC:%s Road:%s Trav:%s Load:%s",
        tostring(M.state.status),
        M.state.updateCount or 0,
        M.state.wheelsValid and "OK" or "NIL",

        tostring(wheelState.phaseText or "UNKNOWN"),
        wheelState.hysteresis or 0.0,
        wheelState.memory or 0.0,
        wheelState.dampingHeat or 0.0,

        wheelState.velocity or 0.0,
        wheelState.accel or 0.0,
        wheelState.force or 0.0,

        wheelState.bump or 0.0,
        wheelState.rebound or 0.0,
        wheelState.highSpeed or 0.0,
        wheelState.impact or 0.0,
        wheelState.reversal or 0.0,

        wheelState.verticalDamping or 0.0,
        wheelState.hystereticDamping or 0.0,
        wheelState.ringLag or 0.0,
        wheelState.beltLag or 0.0,
        wheelState.verticalPulse or 0.0,

        wheelState.contactQuality or 1.0,
        wheelState.contactTrust or 1.0,
        wheelState.roadInput or 0.0,
        wheelState.load or 0.0,

        M.state.damperLinked and "OK" or "NIL",
        M.state.contactLinked and "OK" or "NIL",
        M.state.carcassLinked and "OK" or "NIL",
        M.state.roadLinked and "OK" or "NIL",
        M.state.travelLinked and "OK" or "NIL",
        M.state.loadLinked and "OK" or "NIL"
    )
end

function M.drawDebug()
    if not ui or not ui.text then
        return
    end

    ui.separator()
    ui.text("=== DAMPER HYSTERESIS ===")
    ui.text(string.format(
        "Status %s / Avg %.3f / Mem %.3f / Vel %.4f / Heat %.3f",
        tostring(M.state.status),
        M.state.avgHysteresis or 0.0,
        M.state.avgMemory or 0.0,
        M.state.avgAbsVelocity or 0.0,
        M.state.avgHeat or 0.0
    ))

    for i = 0, 3 do
        local w = M.state.wheels[i]
        ui.text(string.format(
            "%s %s H %.2f M %.2f V %+.3f I %.2f P %.2f",
            WHEEL_NAMES[i] or tostring(i),
            tostring(w.phaseText or "UNKNOWN"),
            w.hysteresis or 0.0,
            w.memory or 0.0,
            w.velocity or 0.0,
            w.impact or 0.0,
            w.verticalPulse or 0.0
        ))
    end
end

return M
