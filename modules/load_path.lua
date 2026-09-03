---@diagnostic disable: undefined-global

--============================================================
-- load_path.lua
-- ACNextGen V1.1.5 Stable
--
-- Load Path / Component Work Interpreter
-- App-side observer/bridge only. No direct AC physics overwrite.
--============================================================

local M = {}

local WHEEL_NAMES = { [0] = "FL", [1] = "FR", [2] = "RL", [3] = "RR" }

local PHASE = {
    BALANCED        = 1,
    TIRE_PATH       = 2,
    SUSPENSION_PATH = 3,
    COMPLIANCE_PATH = 4,
    BODY_PATH       = 5,
    CONTACT_LIMIT   = 6,
    LOAD_PEAK       = 7,
    IMPACT_PATH     = 8,
    UNLOADED        = 9,
    FORCE_LEAK      = 10,
    ROAD_INPUT      = 11,
}

local PHASE_TEXT = {
    [PHASE.BALANCED]        = "BALANCED",
    [PHASE.TIRE_PATH]       = "TIRE_PATH",
    [PHASE.SUSPENSION_PATH] = "SUSPENSION_PATH",
    [PHASE.COMPLIANCE_PATH] = "COMPLIANCE_PATH",
    [PHASE.BODY_PATH]       = "BODY_PATH",
    [PHASE.CONTACT_LIMIT]   = "CONTACT_LIMIT",
    [PHASE.LOAD_PEAK]       = "LOAD_PEAK",
    [PHASE.IMPACT_PATH]     = "IMPACT_PATH",
    [PHASE.UNLOADED]        = "UNLOADED",
    [PHASE.FORCE_LEAK]      = "FORCE_LEAK",
    [PHASE.ROAD_INPUT]      = "ROAD_INPUT",
}

M.PHASE = PHASE

M.params = {
    loadReference       = 3200.0,
    loadGood            = 900.0,
    loadUnload          = 250.0,
    loadPeakNorm        = 1.45,
    loadRateReference   = 85000.0,
    dltAsAbsoluteAbove  = 900.0,

    suspForceReference  = 2500.0,
    damperVelocityRef   = 0.35,
    verticalDampingRef  = 1.0,

    latFlexRef          = 0.035,
    longFlexRef         = 0.030,

    carcassNormRef      = 1.0,
    complianceLossRef   = 1.0,
    forceLeakRef        = 1.0,

    tireWeight          = 0.25,
    suspensionWeight    = 0.22,
    complianceWeight    = 0.22,
    bodyWeight          = 0.14,
    contactWeight       = 0.17,
    roadWeight          = 0.12,

    roadInputGain       = 0.40,
    chassisInputGain    = 0.35,

    contactLossGain     = 0.28,
    complianceLossGain  = 0.24,
    forceLeakGain       = 0.30,
    bodyAbsorbGain      = 0.16,
    delayLossGain       = 0.20,
    dirtyReturnGain     = 0.14,

    tireDeliveryGain       = 0.62,
    suspensionDeliveryGain = 0.16,
    complianceDeliveryLoss = 0.22,

    workTauRise         = 0.065,
    workTauFall         = 0.170,
    delayTau            = 0.125,
    efficiencyTau       = 0.105,
    integrityTau        = 0.090,
    deliveryTau         = 0.090,
    leakTau             = 0.080,

    maxWork             = 1.0,
    minEfficiency       = 0.0,
    maxEfficiency       = 1.0,

    minDt               = 0.0005,
    maxDt               = 0.050,
    noCarDecayTau       = 0.220,

    debugStoreInterval  = 0.25,
}

M.state = {
    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    avgWork = 0.0,
    avgEfficiency = 1.0,
    avgIntegrity = 1.0,
    avgLoss = 0.0,
    avgDelay = 0.0,
    avgTireDelivery = 1.0,
    avgForceLeak = 0.0,
    avgRoadWork = 0.0,
    avgVerticalFlow = 0.0,
    maxWork = 0.0,
    maxLoss = 0.0,

    frontWork = 0.0,
    rearWork = 0.0,
    leftWork = 0.0,
    rightWork = 0.0,

    frontIntegrity = 1.0,
    rearIntegrity = 1.0,
    frontDelivery = 1.0,
    rearDelivery = 1.0,

    tireShare = 0.0,
    suspensionShare = 0.0,
    complianceShare = 0.0,
    bodyShare = 0.0,
    contactShare = 0.0,
    roadShare = 0.0,

    dominantPathId = PHASE.BALANCED,
    dominantPath = "BALANCED",

    loadLinked = false,
    contactLinked = false,
    carcassLinked = false,
    damperLinked = false,
    complianceLinked = false,
    bodyLinked = false,
    suspensionLinked = false,
    recoveryLinked = false,
    memoryLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,

    wheels = {},
}

local function newWheelState()
    return {
        load = 0.0,
        prevLoad = 0.0,
        loadRate = 0.0,
        loadNorm = 0.0,

        work = 0.0,
        rawWork = 0.0,
        energy = 0.0,
        efficiency = 1.0,
        integrity = 1.0,
        loss = 0.0,
        delay = 0.0,

        tireDelivery = 1.0,
        forceLeak = 0.0,
        bodyAbsorb = 0.0,
        pathCompression = 0.0,
        verticalFlow = 0.0,

        tireWork = 0.0,
        suspensionWork = 0.0,
        complianceWork = 0.0,
        bodyWork = 0.0,
        contactWork = 0.0,
        roadWork = 0.0,

        contactQuality = 1.0,
        contactTrust = 1.0,
        contactStability = 1.0,
        combinedSlip = 0.0,
        contactLimit = 0.0,

        carcassDeformation = 0.0,
        sidewallEnergy = 0.0,
        contactDelay = 0.0,
        returnForce = 0.0,
        carcassSupport = 1.0,
        carcassGripGate = 1.0,
        verticalNorm = 0.0,
        carcassHysteresis = 0.0,

        damperHyst = 0.0,
        damperMemory = 0.0,
        damperImpact = 0.0,
        damperVelocity = 0.0,
        verticalDamping = 0.0,
        hystereticDamping = 0.0,
        verticalPulse = 0.0,
        dampingHeat = 0.0,
        roadMemory = 0.0,

        complianceEnergy = 0.0,
        latFlex = 0.0,
        longFlex = 0.0,
        complianceForceLeak = 0.0,
        complianceLoadPathLoss = 0.0,
        bushingEnergy = 0.0,
        knuckleDeflection = 0.0,
        subframeDeflection = 0.0,
        bodyDeflection = 0.0,

        tireMemory = 0.0,
        memoryGrip = 1.0,
        dirtyReturn = 0.0,
        bite = 0.0,
        snapRisk = 0.0,
        recoveryRate = 0.0,

        suspensionForce = 0.0,

        phaseId = PHASE.BALANCED,
        phaseText = "BALANCED",
        active = false,
    }
end

for i = 0, 3 do
    M.state.wheels[i] = newWheelState()
    M.state[i] = M.state.wheels[i]
end

M.debug = M.state

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

local function numberOrNil(value)
    local n = tonumber(value)
    if n == nil or n ~= n or n == math.huge or n == -math.huge then
        return nil
    end
    return n
end

local function clamp(v, minV, maxV)
    v = safeNumber(v, minV or 0.0)
    if v < minV then return minV end
    if v > maxV then return maxV end
    return v
end

local function abs(v)
    return math.abs(safeNumber(v, 0.0))
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

local function safeLoadAlt(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local v = safeLoadRaw(keys[i])
        if v ~= nil then
            return safeNumber(v, defaultValue or 0.0), keys[i]
        end
    end
    if defaultValue == nil then
        return nil, nil
    end
    return defaultValue, nil
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
    M.state.debugStoreTimer = (M.state.debugStoreTimer or 0.0) + (dt or 0.0)

    if M.state.debugStoreTimer >= M.params.debugStoreInterval then
        M.state.debugStoreTimer = 0.0
        M.state.debugStoreNow = true
    else
        M.state.debugStoreNow = false
    end
end

local function getWheel(car, index)
    if not car then
        return nil
    end

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

local function resetLinks()
    local st = M.state
    st.loadLinked = false
    st.contactLinked = false
    st.carcassLinked = false
    st.damperLinked = false
    st.complianceLinked = false
    st.bodyLinked = false
    st.suspensionLinked = false
    st.recoveryLinked = false
    st.memoryLinked = false
end

--============================================================
-- Input readers
--============================================================

local function readLoad(index, wheel)
    local wheelLoad = nil

    if wheel then
        local fields = { "load", "loadK" }
        for n = 1, #fields do
            local candidate = numberOrNil(safeField(wheel, fields[n], nil))
            if candidate ~= nil and math.abs(candidate) > 1.0 then
                wheelLoad = candidate
                break
            end
        end
    end

    if wheelLoad ~= nil then
        return math.abs(wheelLoad)
    end

    local contactLoad = safeLoadRaw("ngp_contact_load_" .. index)
    if contactLoad ~= nil then
        M.state.loadLinked = true
        return math.abs(safeNumber(contactLoad, M.params.loadReference))
    end

    local tireLoad = safeLoadRaw("ngp_tire_state_load_" .. index)
    if tireLoad ~= nil then
        M.state.loadLinked = true
        return math.abs(safeNumber(tireLoad, M.params.loadReference))
    end

    local sprungLoad = safeLoadRaw("ngp_sprung_load_" .. index)
    if sprungLoad ~= nil then
        M.state.loadLinked = true
        return math.abs(safeNumber(sprungLoad, M.params.loadReference))
    end

    local dltLoad = safeLoadRaw("ngp_dlt_load_" .. index)
    if dltLoad ~= nil then
        M.state.loadLinked = true

        local n = safeNumber(dltLoad, 0.0)
        if math.abs(n) > M.params.dltAsAbsoluteAbove then
            return math.abs(n)
        end

        return math.max(0.0, M.params.loadReference + n)
    end

    return M.params.loadReference
end

local function readContact(index, ws)
    local q, kq = safeLoadAlt(nil,
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index,
        "ngp_tc_contact_" .. index,
        "ngp_tire_contact_" .. index
    )

    local trust, kt = safeLoadAlt(nil,
        "ngp_contact_trust_" .. index,
        "ngp_tire_contact_trust_" .. index
    )

    local stability, ks = safeLoadAlt(nil,
        "ngp_contact_stability_score_" .. index,
        "ngp_contact_stability_" .. index
    )

    local combined, kc = safeLoadAlt(nil,
        "ngp_contact_combined_slip_" .. index,
        "ngp_tire_force_combined_slip_" .. index,
        "ngp_tdyn_combined_slip_" .. index
    )

    local loss, kl = safeLoadAlt(nil,
        "ngp_contact_loss_" .. index,
        "ngp_tire_contact_loss_" .. index,
        "ngp_tcr_contact_loss_" .. index
    )

    if kq or kt or ks or kc or kl then
        M.state.contactLinked = true
    end

    if q == nil and loss ~= nil then
        q = 1.0 - clamp(loss, 0.0, 1.0)
    end

    ws.contactQuality = clamp(safeNumber(q, 1.0), 0.0, 1.2)
    ws.contactTrust = clamp(safeNumber(trust, ws.contactQuality), 0.0, 1.2)
    ws.contactStability = clamp(safeNumber(stability, 1.0), 0.0, 1.0)
    ws.combinedSlip = clamp(safeNumber(combined, 0.0), 0.0, 2.5)

    ws.contactLimit = clamp(
        (1.0 - clamp(ws.contactQuality, 0.0, 1.0)) * 0.50
        + (1.0 - clamp(ws.contactTrust, 0.0, 1.0)) * 0.30
        + ws.combinedSlip * 0.20,
        0.0,
        1.0
    )
end

local function readCarcass(index, ws)
    local deformation, k0 = safeLoadAlt(0.0,
        "ngp_tire_deformation_" .. index,
        "ngp_carcass_deformation_" .. index
    )

    local sidewall, k1 = safeLoadAlt(0.0,
        "ngp_tire_sidewall_energy_" .. index,
        "ngp_carcass_sidewall_energy_" .. index
    )

    local delay, k2 = safeLoadAlt(0.0,
        "ngp_tire_contact_delay_" .. index,
        "ngp_carcass_contact_delay_" .. index
    )

    local ret, k3 = safeLoadAlt(0.0,
        "ngp_tire_return_force_" .. index,
        "ngp_carcass_return_force_" .. index
    )

    local support, k4 = safeLoadAlt(1.0,
        "ngp_carcass_support_" .. index,
        "ngp_contact_carcass_support_" .. index
    )

    local gripGate, k5 = safeLoadAlt(1.0,
        "ngp_carcass_grip_gate_" .. index,
        "ngp_contact_grip_gate_" .. index
    )

    local verticalNorm, k6 = safeLoadAlt(0.0,
        "ngp_carcass_vertical_norm_" .. index,
        "ngp_carcass_vertical_deflect_" .. index,
        "ngp_contact_vertical_score_" .. index
    )

    local hyst, k7 = safeLoadAlt(0.0,
        "ngp_carcass_hysteresis_" .. index,
        "ngp_tire_carcass_hyst_" .. index,
        "ngp_contact_hysteresis_" .. index
    )

    if k0 or k1 or k2 or k3 or k4 or k5 or k6 or k7 then
        M.state.carcassLinked = true
    end

    ws.carcassDeformation = clamp(deformation, 0.0, 1.5)
    ws.sidewallEnergy = clamp(sidewall, 0.0, 1.5)
    ws.contactDelay = clamp(delay, 0.0, 1.5)
    ws.returnForce = clamp(ret, 0.0, 1.5)
    ws.carcassSupport = clamp(support, 0.0, 1.2)
    ws.carcassGripGate = clamp(gripGate, 0.0, 1.2)
    ws.verticalNorm = clamp(verticalNorm, 0.0, 1.5)
    ws.carcassHysteresis = clamp(hyst, 0.0, 1.2)
end

local function readDamper(index, ws)
    local hyst, k0 = safeLoadAlt(0.0,
        "ngp_damper_hyst_" .. index,
        "ngp_hyst_damper_" .. index
    )

    local mem, k1 = safeLoadAlt(0.0,
        "ngp_damper_hyst_memory_" .. index,
        "ngp_road_damper_memory_" .. index
    )

    local impact, k2 = safeLoadAlt(0.0,
        "ngp_damper_hyst_impact_" .. index,
        "ngp_damper_impact_" .. index
    )

    local vel, k3 = safeLoadAlt(0.0,
        "ngp_damper_hyst_velocity_" .. index,
        "ngp_damper_velocity_" .. index,
        "ngp_susp_damper_velocity_" .. index
    )

    local vertical, k4 = safeLoadAlt(0.0,
        "ngp_damper_vertical_" .. index,
        "ngp_vertical_damping_" .. index
    )

    local hysteretic, k5 = safeLoadAlt(0.0,
        "ngp_damper_hysteretic_" .. index,
        "ngp_hysteretic_damping_" .. index
    )

    local pulse, k6 = safeLoadAlt(0.0,
        "ngp_damper_vertical_pulse_" .. index,
        "ngp_road_vertical_pulse_" .. index
    )

    local heat, k7 = safeLoadAlt(0.0,
        "ngp_damper_heat_" .. index,
        "ngp_damper_hyst_heat_" .. index
    )

    local roadMem, k8 = safeLoadAlt(0.0,
        "ngp_damper_road_memory_" .. index,
        "ngp_road_damper_memory_" .. index
    )

    if k0 or k1 or k2 or k3 or k4 or k5 or k6 or k7 or k8 then
        M.state.damperLinked = true
    end

    ws.damperHyst = clamp(hyst, 0.0, 1.5)
    ws.damperMemory = clamp(mem, 0.0, 1.5)
    ws.damperImpact = clamp(impact, 0.0, 1.5)
    ws.damperVelocity = safeNumber(vel, 0.0)
    ws.verticalDamping = clamp(vertical, 0.0, 1.5)
    ws.hystereticDamping = clamp(hysteretic, 0.0, 1.5)
    ws.verticalPulse = clamp(pulse, 0.0, 1.5)
    ws.dampingHeat = clamp(heat, 0.0, 1.5)
    ws.roadMemory = clamp(roadMem, 0.0, 1.5)
end

local function readCompliance(index, ws)
    local energy, k0 = safeLoadAlt(0.0,
        "ngp_compliance_energy_" .. index,
        "ngp_compliance_stack_energy_" .. index,
        "ngp_cs_energy_" .. index
    )

    local lat, k1 = safeLoadAlt(0.0,
        "ngp_compliance_lat_flex_" .. index,
        "ngp_arm_lat_flex_" .. index
    )

    local long, k2 = safeLoadAlt(0.0,
        "ngp_compliance_long_flex_" .. index,
        "ngp_arm_long_flex_" .. index
    )

    local forceLeak, k3 = safeLoadAlt(0.0,
        "ngp_compliance_force_leak_" .. index,
        "ngp_force_path_loss_" .. index,
        "ngp_lp_force_leak_" .. index
    )

    local loadPathLoss, k4 = safeLoadAlt(0.0,
        "ngp_compliance_load_path_loss_" .. index,
        "ngp_load_path_compliance_loss_" .. index
    )

    local bushingEnergy, k5 = safeLoadAlt(0.0,
        "ngp_compliance_bushing_energy_" .. index,
        "ngp_bushing_energy_" .. index
    )

    local knuckle, k6 = safeLoadAlt(0.0,
        "ngp_compliance_knuckle_deflection_" .. index,
        "ngp_knuckle_deflection_" .. index
    )

    local subframe, k7 = safeLoadAlt(0.0,
        "ngp_compliance_subframe_deflection_" .. index,
        "ngp_subframe_deflection_" .. index
    )

    local body, k8 = safeLoadAlt(0.0,
        "ngp_compliance_body_deflection_" .. index,
        "ngp_body_compliance_" .. index
    )

    if k0 or k1 or k2 or k3 or k4 or k5 or k6 or k7 or k8 then
        M.state.complianceLinked = true
    end

    ws.complianceEnergy = clamp(energy, 0.0, 1.5)
    ws.latFlex = safeNumber(lat, 0.0)
    ws.longFlex = safeNumber(long, 0.0)
    ws.complianceForceLeak = clamp(forceLeak, 0.0, 1.5)
    ws.complianceLoadPathLoss = clamp(loadPathLoss, 0.0, 1.5)
    ws.bushingEnergy = clamp(bushingEnergy, 0.0, 1.5)
    ws.knuckleDeflection = clamp(knuckle, 0.0, 1.5)
    ws.subframeDeflection = clamp(subframe, 0.0, 1.5)
    ws.bodyDeflection = clamp(body, 0.0, 1.5)
end

local function readMemoryAndRecovery(index, ws)
    local mem, k0 = safeLoadAlt(0.0,
        "ngp_tire_memory_" .. index,
        "ngp_memory_" .. index,
        "ngp_rubber_memory_" .. index
    )

    local grip, k1 = safeLoadAlt(1.0,
        "ngp_memory_grip_" .. index,
        "ngp_tire_memory_grip_" .. index
    )

    local dirty, k2 = safeLoadAlt(0.0,
        "ngp_slip_dirty_return_" .. index,
        "ngp_dirty_return_" .. index
    )

    local bite, k3 = safeLoadAlt(0.0,
        "ngp_slip_bite_" .. index,
        "ngp_recovery_bite_" .. index
    )

    local snap, k4 = safeLoadAlt(0.0,
        "ngp_slip_snap_risk_" .. index,
        "ngp_snap_risk_" .. index
    )

    local rec, k5 = safeLoadAlt(0.0,
        "ngp_slip_recovery_rate_" .. index,
        "ngp_recovery_rate_" .. index
    )

    if k0 or k1 then
        M.state.memoryLinked = true
    end

    if k2 or k3 or k4 or k5 then
        M.state.recoveryLinked = true
    end

    ws.tireMemory = clamp(mem, 0.0, 1.0)
    ws.memoryGrip = clamp(grip, 0.0, 1.2)
    ws.dirtyReturn = clamp(dirty, 0.0, 1.2)
    ws.bite = clamp(bite, 0.0, 1.2)
    ws.snapRisk = clamp(snap, 0.0, 1.2)
    ws.recoveryRate = clamp(rec, 0.0, 1.2)
end

local function readSuspension(index, ws)
    local force, key = safeLoadAlt(0.0,
        "ngp_susp_" .. index,
        "ngp_susp_int_damper_" .. index,
        "ngp_susp_int_force_" .. index,
        "ngp_susp_integrated_force_" .. index,
        "ngp_damper_force_" .. index
    )

    if key then
        M.state.suspensionLinked = true
    end

    ws.suspensionForce = safeNumber(force, 0.0)
end

local function readBodyInputs()
    local heave, k0 = safeLoadAlt(0.0, "ngp_body_heave_input", "ngp_rbi_heave", "ngp_body_heave")
    local pitch, k1 = safeLoadAlt(0.0, "ngp_body_pitch_input", "ngp_pitch_cg", "ngp_body_pitch")
    local roll,  k2 = safeLoadAlt(0.0, "ngp_body_roll_input", "ngp_roll_cg", "ngp_body_roll")
    local yaw,   k3 = safeLoadAlt(0.0, "ngp_body_yaw_hint", "ngp_chassis_flex_yaw", "ngp_body_yaw_hint_live")

    local chassisEnergy, k4 = safeLoadAlt(0.0,
        "ngp_chassis_energy",
        "ngp_chassis_flex_energy",
        "ngp_body_rigidity_energy",
        "ngp_chassis_compliance"
    )

    local bodyFlex, k5 = safeLoadAlt(0.0,
        "ngp_body_flex",
        "ngp_compliance_body_flex",
        "ngp_chassis_flex_yaw"
    )

    local subframeTwist, k6 = safeLoadAlt(0.0,
        "ngp_subframe_twist",
        "ngp_compliance_subframe_twist"
    )

    local rearAxleWindup, k7 = safeLoadAlt(0.0,
        "ngp_rear_axle_windup",
        "ngp_compliance_rear_axle_windup",
        "ngp_driveline_windup"
    )

    local impact, k8 = safeLoadAlt(0.0,
        "ngp_impact_root_value",
        "ngp_impact_value"
    )

    if k0 or k1 or k2 or k3 or k4 or k5 or k6 or k7 or k8 then
        M.state.bodyLinked = true
    end

    local bodyInput = clamp(
        abs(heave) * M.params.roadInputGain
        + abs(pitch) * 0.25
        + abs(roll) * 0.25
        + abs(yaw) * 0.20
        + abs(chassisEnergy) * M.params.chassisInputGain
        + abs(bodyFlex) * 0.22
        + abs(subframeTwist) * 0.18
        + abs(rearAxleWindup) * 0.14
        + abs(impact) * 0.18,
        0.0,
        1.5
    )

    return bodyInput, bodyFlex, subframeTwist, rearAxleWindup
end

--============================================================
-- Core
--============================================================

local function dominantPhase(ws)
    if ws.load < M.params.loadUnload or ws.contactQuality < 0.12 then
        return PHASE.UNLOADED, PHASE_TEXT[PHASE.UNLOADED]
    end

    if ws.forceLeak > 0.58 or ws.complianceLoadPathLoss > 0.58 then
        return PHASE.FORCE_LEAK, PHASE_TEXT[PHASE.FORCE_LEAK]
    end

    if ws.roadWork > 0.62 or ws.verticalPulse > 0.55 or ws.roadMemory > 0.62 then
        return PHASE.ROAD_INPUT, PHASE_TEXT[PHASE.ROAD_INPUT]
    end

    if ws.damperImpact > 0.45 or ws.contactStability < 0.35 then
        return PHASE.IMPACT_PATH, PHASE_TEXT[PHASE.IMPACT_PATH]
    end

    if ws.loadNorm > M.params.loadPeakNorm then
        return PHASE.LOAD_PEAK, PHASE_TEXT[PHASE.LOAD_PEAK]
    end

    if ws.contactWork > 0.62 then
        return PHASE.CONTACT_LIMIT, PHASE_TEXT[PHASE.CONTACT_LIMIT]
    end

    local phase = PHASE.BALANCED
    local best = 0.22

    if ws.tireWork > best then
        best = ws.tireWork
        phase = PHASE.TIRE_PATH
    end

    if ws.suspensionWork > best then
        best = ws.suspensionWork
        phase = PHASE.SUSPENSION_PATH
    end

    if ws.complianceWork > best then
        best = ws.complianceWork
        phase = PHASE.COMPLIANCE_PATH
    end

    if ws.bodyWork > best then
        best = ws.bodyWork
        phase = PHASE.BODY_PATH
    end

    return phase, PHASE_TEXT[phase] or "UNKNOWN"
end

local function calculateWheel(ws, bodyInput, bodyFlex, subframeTwist, rearAxleWindup, dt)
    local p = M.params

    ws.loadNorm = clamp(ws.load / math.max(p.loadReference, 1.0), 0.0, 2.5)

    local loadRateNorm = clamp(abs(ws.loadRate) / math.max(p.loadRateReference, 1.0), 0.0, 1.5)
    local contactLoss = clamp(1.0 - ws.contactQuality, 0.0, 1.0)
    local trustLoss = clamp(1.0 - ws.contactTrust, 0.0, 1.0)
    local supportLoss = clamp(1.0 - ws.carcassSupport, 0.0, 1.0)
    local gripGateLoss = clamp(1.0 - ws.carcassGripGate, 0.0, 1.0)
    local stabilityLoss = clamp(1.0 - ws.contactStability, 0.0, 1.0)
    local loadExcess = clamp(ws.loadNorm - 1.0, 0.0, 1.5)

    local tireWorkRaw =
        ws.carcassDeformation * 0.26
        + ws.sidewallEnergy * 0.20
        + ws.verticalNorm * 0.18
        + ws.contactDelay * 0.13
        + ws.returnForce * 0.08
        + ws.carcassHysteresis * 0.10
        + ws.combinedSlip * 0.12
        + loadExcess * 0.12
        + supportLoss * 0.12
        + gripGateLoss * 0.10

    local suspVel = clamp(abs(ws.damperVelocity) / math.max(p.damperVelocityRef, 0.001), 0.0, 1.5)
    local suspForce = clamp(abs(ws.suspensionForce) / math.max(p.suspForceReference, 1.0), 0.0, 1.5)
    local verticalDampingNorm = clamp(ws.verticalDamping / math.max(p.verticalDampingRef, 0.001), 0.0, 1.5)

    local suspensionWorkRaw =
        ws.damperHyst * 0.22
        + ws.damperMemory * 0.16
        + ws.damperImpact * 0.23
        + ws.hystereticDamping * 0.16
        + verticalDampingNorm * 0.12
        + suspVel * 0.11
        + suspForce * 0.08
        + loadRateNorm * 0.13
        + ws.dampingHeat * 0.08

    local latFlexNorm = clamp(abs(ws.latFlex) / math.max(p.latFlexRef, 0.001), 0.0, 1.5)
    local longFlexNorm = clamp(abs(ws.longFlex) / math.max(p.longFlexRef, 0.001), 0.0, 1.5)

    local complianceWorkRaw =
        ws.complianceEnergy * 0.26
        + latFlexNorm * 0.14
        + longFlexNorm * 0.14
        + ws.bushingEnergy * 0.14
        + ws.knuckleDeflection * 0.10
        + ws.subframeDeflection * 0.10
        + ws.complianceForceLeak * 0.16
        + ws.complianceLoadPathLoss * 0.16
        + loadRateNorm * 0.07
        + contactLoss * 0.06

    local bodyWorkRaw =
        bodyInput * 0.34
        + ws.bodyDeflection * 0.22
        + abs(bodyFlex) * 0.12
        + abs(subframeTwist) * 0.10
        + abs(rearAxleWindup) * 0.08
        + loadRateNorm * 0.12
        + loadExcess * 0.07
        + ws.damperMemory * 0.07

    local contactWorkRaw =
        contactLoss * 0.28
        + trustLoss * 0.20
        + stabilityLoss * 0.22
        + ws.combinedSlip * 0.12
        + ws.damperImpact * 0.08
        + ws.snapRisk * 0.06
        + ws.bite * 0.05

    local roadWorkRaw =
        ws.verticalPulse * 0.32
        + ws.roadMemory * 0.28
        + ws.damperImpact * 0.16
        + verticalDampingNorm * 0.10
        + loadRateNorm * 0.10
        + bodyInput * 0.04

    ws.tireWork = clamp(tireWorkRaw, 0.0, p.maxWork)
    ws.suspensionWork = clamp(suspensionWorkRaw, 0.0, p.maxWork)
    ws.complianceWork = clamp(complianceWorkRaw, 0.0, p.maxWork)
    ws.bodyWork = clamp(bodyWorkRaw, 0.0, p.maxWork)
    ws.contactWork = clamp(contactWorkRaw, 0.0, p.maxWork)
    ws.roadWork = clamp(roadWorkRaw, 0.0, p.maxWork)

    local rawWork =
        ws.tireWork * p.tireWeight
        + ws.suspensionWork * p.suspensionWeight
        + ws.complianceWork * p.complianceWeight
        + ws.bodyWork * p.bodyWeight
        + ws.contactWork * p.contactWeight
        + ws.roadWork * p.roadWeight

    rawWork = clamp(rawWork, 0.0, p.maxWork)

    local tau = rawWork > ws.work and p.workTauRise or p.workTauFall
    ws.work = clamp(lowPass(ws.work, rawWork, tau, dt), 0.0, p.maxWork)
    ws.rawWork = rawWork
    ws.energy = clamp(lowPass(ws.energy, ws.work, p.workTauFall, dt), 0.0, p.maxWork)

    local rawDelay =
        ws.contactDelay * 0.25
        + ws.damperMemory * 0.16
        + ws.roadMemory * 0.14
        + ws.complianceEnergy * 0.15
        + ws.complianceLoadPathLoss * 0.12
        + stabilityLoss * 0.10
        + loadRateNorm * 0.08

    ws.delay = clamp(lowPass(ws.delay, rawDelay, p.delayTau, dt), 0.0, 1.0)

    ws.forceLeak = clamp(lowPass(ws.forceLeak,
        ws.complianceForceLeak * 0.46
        + ws.complianceLoadPathLoss * 0.30
        + ws.bodyDeflection * 0.10
        + ws.bushingEnergy * 0.07
        + ws.dirtyReturn * 0.07,
        p.leakTau,
        dt
    ), 0.0, 1.0)

    local rawLoss =
        ws.contactWork * p.contactLossGain
        + ws.complianceLoadPathLoss * p.complianceLossGain
        + ws.forceLeak * p.forceLeakGain
        + ws.bodyWork * p.bodyAbsorbGain
        + ws.delay * p.delayLossGain
        + ws.dirtyReturn * p.dirtyReturnGain
        + supportLoss * 0.10
        + gripGateLoss * 0.08
        + stabilityLoss * 0.10

    ws.loss = clamp(rawLoss, 0.0, 1.0)
    ws.efficiency = clamp(lowPass(ws.efficiency, 1.0 - ws.loss, p.efficiencyTau, dt), p.minEfficiency, p.maxEfficiency)

    local rawIntegrity =
        clamp(ws.contactTrust, 0.0, 1.0) * 0.30
        + clamp(ws.carcassSupport, 0.0, 1.0) * 0.22
        + clamp(ws.carcassGripGate, 0.0, 1.0) * 0.14
        + clamp(ws.efficiency, 0.0, 1.0) * 0.22
        + clamp(1.0 - ws.forceLeak, 0.0, 1.0) * 0.12

    ws.integrity = clamp(lowPass(ws.integrity, rawIntegrity, p.integrityTau, dt), 0.0, 1.0)

    local rawDelivery =
        ws.efficiency * p.tireDeliveryGain
        + clamp(1.0 - ws.complianceLoadPathLoss, 0.0, 1.0) * 0.18
        + clamp(ws.contactTrust, 0.0, 1.0) * 0.12
        + clamp(ws.carcassSupport, 0.0, 1.0) * 0.08
        - ws.forceLeak * p.complianceDeliveryLoss

    ws.tireDelivery = clamp(lowPass(ws.tireDelivery, rawDelivery, p.deliveryTau, dt), 0.0, 1.0)

    ws.bodyAbsorb = clamp(
        ws.bodyWork * 0.42
        + ws.bodyDeflection * 0.30
        + abs(bodyFlex) * 0.16
        + abs(subframeTwist) * 0.12,
        0.0,
        1.0
    )

    ws.pathCompression = clamp(
        ws.loadNorm * 0.20
        + ws.verticalNorm * 0.22
        + ws.carcassDeformation * 0.22
        + ws.damperHyst * 0.14
        + ws.complianceEnergy * 0.12
        + ws.bodyAbsorb * 0.10,
        0.0,
        1.0
    )

    ws.verticalFlow = clamp(
        ws.verticalPulse * 0.28
        + ws.verticalDamping * 0.20
        + ws.verticalNorm * 0.18
        + ws.damperImpact * 0.14
        + loadRateNorm * 0.12
        + ws.roadMemory * 0.08,
        0.0,
        1.0
    )

    ws.phaseId, ws.phaseText = dominantPhase(ws)
end

local function decayWheel(ws, dt)
    ws.active = false
    ws.load = lowPass(ws.load, 0.0, M.params.noCarDecayTau, dt)
    ws.loadRate = lowPass(ws.loadRate, 0.0, M.params.noCarDecayTau, dt)
    ws.work = lowPass(ws.work, 0.0, M.params.workTauFall, dt)
    ws.rawWork = lowPass(ws.rawWork, 0.0, M.params.workTauFall, dt)
    ws.energy = lowPass(ws.energy, 0.0, M.params.workTauFall, dt)
    ws.efficiency = lowPass(ws.efficiency, 1.0, M.params.efficiencyTau, dt)
    ws.integrity = lowPass(ws.integrity, 1.0, M.params.integrityTau, dt)
    ws.tireDelivery = lowPass(ws.tireDelivery, 1.0, M.params.deliveryTau, dt)
    ws.loss = lowPass(ws.loss, 0.0, M.params.workTauFall, dt)
    ws.delay = lowPass(ws.delay, 0.0, M.params.delayTau, dt)
    ws.forceLeak = lowPass(ws.forceLeak, 0.0, M.params.leakTau, dt)
    ws.roadWork = lowPass(ws.roadWork, 0.0, M.params.workTauFall, dt)
    ws.verticalFlow = lowPass(ws.verticalFlow, 0.0, M.params.workTauFall, dt)
    ws.phaseId = PHASE.UNLOADED
    ws.phaseText = PHASE_TEXT[PHASE.UNLOADED]
end

--============================================================
-- Export
--============================================================

local function exportWheel(index, ws)
    safeStore("ngp_load_path_work_" .. index, ws.work or 0.0)
    safeStore("ngp_load_path_energy_" .. index, ws.energy or 0.0)
    safeStore("ngp_load_path_efficiency_" .. index, ws.efficiency or 1.0)
    safeStore("ngp_load_path_integrity_" .. index, ws.integrity or 1.0)
    safeStore("ngp_load_path_loss_" .. index, ws.loss or 0.0)
    safeStore("ngp_load_path_delay_" .. index, ws.delay or 0.0)
    safeStore("ngp_load_path_tire_delivery_" .. index, ws.tireDelivery or 1.0)

    safeStore("ngp_load_path_tire_" .. index, ws.tireWork or 0.0)
    safeStore("ngp_load_path_suspension_" .. index, ws.suspensionWork or 0.0)
    safeStore("ngp_load_path_compliance_" .. index, ws.complianceWork or 0.0)
    safeStore("ngp_load_path_body_" .. index, ws.bodyWork or 0.0)
    safeStore("ngp_load_path_contact_" .. index, ws.contactWork or 0.0)
    safeStore("ngp_load_path_road_" .. index, ws.roadWork or 0.0)

    safeStore("ngp_load_path_force_leak_" .. index, ws.forceLeak or 0.0)
    safeStore("ngp_load_path_body_absorb_" .. index, ws.bodyAbsorb or 0.0)
    safeStore("ngp_load_path_compression_" .. index, ws.pathCompression or 0.0)
    safeStore("ngp_load_path_vertical_flow_" .. index, ws.verticalFlow or 0.0)
    safeStore("ngp_load_path_contact_limit_" .. index, ws.contactLimit or 0.0)

    safeStore("ngp_load_path_phase_id_" .. index, ws.phaseId or 0)
    safeStore("ngp_load_path_phase_" .. index, ws.phaseText or "UNKNOWN")

    safeStore("ngp_load_path_load_" .. index, ws.load or 0.0)
    safeStore("ngp_load_path_load_rate_" .. index, ws.loadRate or 0.0)

    safeStore("ngp_lp_work_" .. index, ws.work or 0.0)
    safeStore("ngp_lp_eff_" .. index, ws.efficiency or 1.0)
    safeStore("ngp_lp_integrity_" .. index, ws.integrity or 1.0)
    safeStore("ngp_lp_loss_" .. index, ws.loss or 0.0)
    safeStore("ngp_lp_delay_" .. index, ws.delay or 0.0)
    safeStore("ngp_lp_tire_" .. index, ws.tireWork or 0.0)
    safeStore("ngp_lp_susp_" .. index, ws.suspensionWork or 0.0)
    safeStore("ngp_lp_comp_" .. index, ws.complianceWork or 0.0)
    safeStore("ngp_lp_body_" .. index, ws.bodyWork or 0.0)
    safeStore("ngp_lp_contact_" .. index, ws.contactWork or 0.0)
    safeStore("ngp_lp_road_" .. index, ws.roadWork or 0.0)
    safeStore("ngp_lp_tire_delivery_" .. index, ws.tireDelivery or 1.0)
    safeStore("ngp_lp_force_leak_" .. index, ws.forceLeak or 0.0)
    safeStore("ngp_lp_vertical_flow_" .. index, ws.verticalFlow or 0.0)
end

local function exportGlobal()
    local st = M.state

    safeStore("ngp_load_path_status", st.status or "UNKNOWN")
    safeStore("ngp_load_path_update_count", st.updateCount or 0)
    safeStore("ngp_load_path_wheels_valid", st.wheelsValid and 1 or 0)

    safeStore("ngp_load_path_avg_work", st.avgWork or 0.0)
    safeStore("ngp_load_path_avg_efficiency", st.avgEfficiency or 1.0)
    safeStore("ngp_load_path_avg_integrity", st.avgIntegrity or 1.0)
    safeStore("ngp_load_path_avg_loss", st.avgLoss or 0.0)
    safeStore("ngp_load_path_avg_delay", st.avgDelay or 0.0)
    safeStore("ngp_load_path_avg_tire_delivery", st.avgTireDelivery or 1.0)
    safeStore("ngp_load_path_avg_force_leak", st.avgForceLeak or 0.0)
    safeStore("ngp_load_path_avg_road_work", st.avgRoadWork or 0.0)
    safeStore("ngp_load_path_avg_vertical_flow", st.avgVerticalFlow or 0.0)
    safeStore("ngp_load_path_max_work", st.maxWork or 0.0)
    safeStore("ngp_load_path_max_loss", st.maxLoss or 0.0)

    safeStore("ngp_load_path_front_work", st.frontWork or 0.0)
    safeStore("ngp_load_path_rear_work", st.rearWork or 0.0)
    safeStore("ngp_load_path_left_work", st.leftWork or 0.0)
    safeStore("ngp_load_path_right_work", st.rightWork or 0.0)

    safeStore("ngp_load_path_front_integrity", st.frontIntegrity or 1.0)
    safeStore("ngp_load_path_rear_integrity", st.rearIntegrity or 1.0)
    safeStore("ngp_load_path_front_delivery", st.frontDelivery or 1.0)
    safeStore("ngp_load_path_rear_delivery", st.rearDelivery or 1.0)

    safeStore("ngp_load_path_tire_share", st.tireShare or 0.0)
    safeStore("ngp_load_path_suspension_share", st.suspensionShare or 0.0)
    safeStore("ngp_load_path_compliance_share", st.complianceShare or 0.0)
    safeStore("ngp_load_path_body_share", st.bodyShare or 0.0)
    safeStore("ngp_load_path_contact_share", st.contactShare or 0.0)
    safeStore("ngp_load_path_road_share", st.roadShare or 0.0)

    safeStore("ngp_load_path_dominant_id", st.dominantPathId or 0)
    safeStore("ngp_load_path_dominant", st.dominantPath or "UNKNOWN")

    safeStore("ngp_lp_avg_work", st.avgWork or 0.0)
    safeStore("ngp_lp_avg_eff", st.avgEfficiency or 1.0)
    safeStore("ngp_lp_avg_integrity", st.avgIntegrity or 1.0)
    safeStore("ngp_lp_avg_loss", st.avgLoss or 0.0)
    safeStore("ngp_lp_avg_force_leak", st.avgForceLeak or 0.0)
    safeStore("ngp_lp_rear_delivery", st.rearDelivery or 1.0)
    safeStore("ngp_lp_front_delivery", st.frontDelivery or 1.0)
    safeStore("ngp_lp_road_share", st.roadShare or 0.0)
    safeStore("ngp_lp_dominant", st.dominantPath or "UNKNOWN")

    safeStore("ngp_load_path_load_linked", st.loadLinked and 1 or 0)
    safeStore("ngp_load_path_contact_linked", st.contactLinked and 1 or 0)
    safeStore("ngp_load_path_carcass_linked", st.carcassLinked and 1 or 0)
    safeStore("ngp_load_path_damper_linked", st.damperLinked and 1 or 0)
    safeStore("ngp_load_path_compliance_linked", st.complianceLinked and 1 or 0)
    safeStore("ngp_load_path_body_linked", st.bodyLinked and 1 or 0)
    safeStore("ngp_load_path_suspension_linked", st.suspensionLinked and 1 or 0)
    safeStore("ngp_load_path_recovery_linked", st.recoveryLinked and 1 or 0)
    safeStore("ngp_load_path_memory_linked", st.memoryLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i, M.state.wheels[i])
    end

    exportGlobal()
end

--============================================================
-- Update
--============================================================

function M.init()
    M.state.status = "INIT"
    exportState()
end

local function updateAggregates()
    local st = M.state

    local sumWork = 0.0
    local sumEfficiency = 0.0
    local sumIntegrity = 0.0
    local sumLoss = 0.0
    local sumDelay = 0.0
    local sumTireDelivery = 0.0
    local sumForceLeak = 0.0
    local sumRoad = 0.0
    local sumVerticalFlow = 0.0
    local maxWork = 0.0
    local maxLoss = 0.0

    local sumTire = 0.0
    local sumSusp = 0.0
    local sumComp = 0.0
    local sumBody = 0.0
    local sumContact = 0.0
    local sumRoadShare = 0.0

    for i = 0, 3 do
        local ws = st.wheels[i]

        sumWork = sumWork + (ws.work or 0.0)
        sumEfficiency = sumEfficiency + (ws.efficiency or 1.0)
        sumIntegrity = sumIntegrity + (ws.integrity or 1.0)
        sumLoss = sumLoss + (ws.loss or 0.0)
        sumDelay = sumDelay + (ws.delay or 0.0)
        sumTireDelivery = sumTireDelivery + (ws.tireDelivery or 1.0)
        sumForceLeak = sumForceLeak + (ws.forceLeak or 0.0)
        sumRoad = sumRoad + (ws.roadWork or 0.0)
        sumVerticalFlow = sumVerticalFlow + (ws.verticalFlow or 0.0)
        maxWork = math.max(maxWork, ws.work or 0.0)
        maxLoss = math.max(maxLoss, ws.loss or 0.0)

        sumTire = sumTire + (ws.tireWork or 0.0)
        sumSusp = sumSusp + (ws.suspensionWork or 0.0)
        sumComp = sumComp + (ws.complianceWork or 0.0)
        sumBody = sumBody + (ws.bodyWork or 0.0)
        sumContact = sumContact + (ws.contactWork or 0.0)
        sumRoadShare = sumRoadShare + (ws.roadWork or 0.0)
    end

    st.avgWork = sumWork * 0.25
    st.avgEfficiency = sumEfficiency * 0.25
    st.avgIntegrity = sumIntegrity * 0.25
    st.avgLoss = sumLoss * 0.25
    st.avgDelay = sumDelay * 0.25
    st.avgTireDelivery = sumTireDelivery * 0.25
    st.avgForceLeak = sumForceLeak * 0.25
    st.avgRoadWork = sumRoad * 0.25
    st.avgVerticalFlow = sumVerticalFlow * 0.25
    st.maxWork = maxWork
    st.maxLoss = maxLoss

    st.frontWork = ((st.wheels[0].work or 0.0) + (st.wheels[1].work or 0.0)) * 0.5
    st.rearWork = ((st.wheels[2].work or 0.0) + (st.wheels[3].work or 0.0)) * 0.5
    st.leftWork = ((st.wheels[0].work or 0.0) + (st.wheels[2].work or 0.0)) * 0.5
    st.rightWork = ((st.wheels[1].work or 0.0) + (st.wheels[3].work or 0.0)) * 0.5

    st.frontIntegrity = ((st.wheels[0].integrity or 1.0) + (st.wheels[1].integrity or 1.0)) * 0.5
    st.rearIntegrity = ((st.wheels[2].integrity or 1.0) + (st.wheels[3].integrity or 1.0)) * 0.5
    st.frontDelivery = ((st.wheels[0].tireDelivery or 1.0) + (st.wheels[1].tireDelivery or 1.0)) * 0.5
    st.rearDelivery = ((st.wheels[2].tireDelivery or 1.0) + (st.wheels[3].tireDelivery or 1.0)) * 0.5

    local totalShare = math.max(sumTire + sumSusp + sumComp + sumBody + sumContact + sumRoadShare, 0.001)
    st.tireShare = sumTire / totalShare
    st.suspensionShare = sumSusp / totalShare
    st.complianceShare = sumComp / totalShare
    st.bodyShare = sumBody / totalShare
    st.contactShare = sumContact / totalShare
    st.roadShare = sumRoadShare / totalShare

    local bestId = PHASE.BALANCED
    local bestName = "BALANCED"
    local bestShare = 0.0

    local shares = {
        { PHASE.TIRE_PATH,       "TIRE_PATH",       st.tireShare },
        { PHASE.SUSPENSION_PATH, "SUSPENSION_PATH", st.suspensionShare },
        { PHASE.COMPLIANCE_PATH, "COMPLIANCE_PATH", st.complianceShare },
        { PHASE.BODY_PATH,       "BODY_PATH",       st.bodyShare },
        { PHASE.CONTACT_LIMIT,   "CONTACT_LIMIT",   st.contactShare },
        { PHASE.ROAD_INPUT,      "ROAD_INPUT",      st.roadShare },
    }

    for i = 1, #shares do
        if shares[i][3] > bestShare then
            bestId = shares[i][1]
            bestName = shares[i][2]
            bestShare = shares[i][3]
        end
    end

    st.dominantPathId = bestId
    st.dominantPath = bestName
end

function M.update(dt, car, runtime)
    M.state.updateCount = (M.state.updateCount or 0) + 1

    dt = safeNumber(dt, 0.0)
    if dt <= 0.0 then
        M.state.status = "BAD DT"
        exportState()
        return
    end

    dt = clamp(dt, M.params.minDt, M.params.maxDt)

    updateDebugGate(dt)

    car = car or safeGetCar()

    if not car then
        M.state.status = "NO CAR"
        M.state.wheelsValid = false
        resetLinks()

        for i = 0, 3 do
            decayWheel(M.state.wheels[i], dt)
        end

        updateAggregates()
        exportState()
        return
    end

    local wheels = safeField(car, "wheels", nil)
    if not wheels then
        M.state.status = "NO WHEELS"
        M.state.wheelsValid = false
        resetLinks()

        for i = 0, 3 do
            decayWheel(M.state.wheels[i], dt)
        end

        updateAggregates()
        exportState()
        return
    end

    local st = M.state

    st.status = "RUNNING"
    st.wheelsValid = true
    resetLinks()

    local bodyInput, bodyFlex, subframeTwist, rearAxleWindup = readBodyInputs()

    for i = 0, 3 do
        local wheel = getWheel(car, i)
        local ws = st.wheels[i]

        if wheel then
            ws.active = true

            local load = math.abs(readLoad(i, wheel))
            local prevLoad = ws.load or load

            ws.prevLoad = prevLoad
            ws.load = load
            ws.loadRate = (load - prevLoad) / math.max(dt, 0.001)

            readContact(i, ws)
            readCarcass(i, ws)
            readDamper(i, ws)
            readCompliance(i, ws)
            readMemoryAndRecovery(i, ws)
            readSuspension(i, ws)

            calculateWheel(ws, bodyInput, bodyFlex, subframeTwist, rearAxleWindup, dt)
        else
            decayWheel(ws, dt)
        end

        exportWheel(i, ws)
    end

    updateAggregates()
    exportGlobal()
end

--============================================================
-- Public API / Debug
--============================================================

function M.getWork(index)
    local ws = M.state.wheels[index]
    return ws and ws.work or 0.0
end

function M.getEfficiency(index)
    local ws = M.state.wheels[index]
    return ws and ws.efficiency or 1.0
end

function M.getIntegrity(index)
    local ws = M.state.wheels[index]
    return ws and ws.integrity or 1.0
end

function M.getTireDelivery(index)
    local ws = M.state.wheels[index]
    return ws and ws.tireDelivery or 1.0
end

function M.getLoss(index)
    local ws = M.state.wheels[index]
    return ws and ws.loss or 0.0
end

function M.getForceLeak(index)
    local ws = M.state.wheels[index]
    return ws and ws.forceLeak or 0.0
end

function M.getDominantPath()
    return M.state.dominantPath or "UNKNOWN", M.state.dominantPathId or 0
end

function M.getState(index)
    if index == nil then
        return M.state
    end
    return M.state.wheels[index]
end

function M.debugStr(index)
    local ws = M.state.wheels[index or 0] or M.state.wheels[0]

    return string.format(
        "Status %s / Count %.0f / Wheels %s / Dominant %s\n" ..
        "%s Work %.3f Eff %.3f Int %.3f Loss %.3f Delay %.3f\n" ..
        "Tire %.2f Susp %.2f Comp %.2f Body %.2f Contact %.2f Road %.2f\n" ..
        "Delivery %.2f Leak %.2f VFlow %.2f Load %.0f Rate %+.0f\n" ..
        "Links L:%s CQ:%s TC:%s DH:%s CS:%s B:%s S:%s R:%s",
        tostring(M.state.status),
        M.state.updateCount or 0,
        M.state.wheelsValid and "OK" or "NIL",
        tostring(M.state.dominantPath or "UNKNOWN"),

        tostring(ws.phaseText or "NIL"),
        ws.work or 0.0,
        ws.efficiency or 1.0,
        ws.integrity or 1.0,
        ws.loss or 0.0,
        ws.delay or 0.0,

        ws.tireWork or 0.0,
        ws.suspensionWork or 0.0,
        ws.complianceWork or 0.0,
        ws.bodyWork or 0.0,
        ws.contactWork or 0.0,
        ws.roadWork or 0.0,

        ws.tireDelivery or 1.0,
        ws.forceLeak or 0.0,
        ws.verticalFlow or 0.0,
        ws.load or 0.0,
        ws.loadRate or 0.0,

        M.state.loadLinked and "OK" or "NIL",
        M.state.contactLinked and "OK" or "NIL",
        M.state.carcassLinked and "OK" or "NIL",
        M.state.damperLinked and "OK" or "NIL",
        M.state.complianceLinked and "OK" or "NIL",
        M.state.bodyLinked and "OK" or "NIL",
        M.state.suspensionLinked and "OK" or "NIL",
        M.state.recoveryLinked and "OK" or "NIL"
    )
end

return M
