---@diagnostic disable: undefined-global

--============================================================
-- tire_carcass.lua
-- ACNextGen V1.1.7 XRay-calibrated v2.1
-- Tire carcass / sidewall / bristle state bridge
-- v2.1 fixes a cross-vehicle saturation issue found by the
-- direct ACXRayLab carcass probe: the energy reservoir could
-- remain pinned at 1.0 and therefore never produce release rate.
-- Calibrated from ACXRayLab baseline observations:
--   front slip -> saturation response: ~0.10 s
--   rear  slip -> saturation response: ~0.05 s
--   front slip -> energy response:     ~0.45 s
--   rear  slip -> energy response:     ~0.35 s
-- O(1) per-wheel state update: no history scan or correlation loop.
-- Safe observation/state module. No direct AC physics rewrite.
--============================================================

local M = {}
M.version = "1.1.7-XRayV2.1"

local WHEEL_NAMES = { [0] = "FL", [1] = "FR", [2] = "RL", [3] = "RR" }

local PHASE = {
    INIT = 0,
    SUPPORTED = 1,
    SIDEWALL_LAT = 2,
    CARCASS_LONG = 3,
    COMPRESSED = 4,
    DELAYED = 5,
    RETURNING = 6,
    SATURATED = 7,
    UNLOADED = 8,
    IMPACT = 9,
}

local PHASE_TEXT = {
    [PHASE.INIT] = "INIT",
    [PHASE.SUPPORTED] = "SUPPORTED",
    [PHASE.SIDEWALL_LAT] = "SIDEWALL_LAT",
    [PHASE.CARCASS_LONG] = "CARCASS_LONG",
    [PHASE.COMPRESSED] = "COMPRESSED",
    [PHASE.DELAYED] = "DELAYED",
    [PHASE.RETURNING] = "RETURNING",
    [PHASE.SATURATED] = "SATURATED",
    [PHASE.UNLOADED] = "UNLOADED",
    [PHASE.IMPACT] = "IMPACT",
}

M.params = {
    loadRef = 3200.0,
    loadMin = 120.0,
    loadCrushStart = 1800.0,
    loadCrushFull = 6200.0,

    virtualSpringRate = 65000.0,
    virtualDamperRate = 1125.0,
    verticalDeflectRef = 0.050,
    maxVerticalDeflect = 0.085,

    bristleLengthRef = 0.11,
    bristleLatGain = 0.86,
    bristleLongGain = 0.74,

    staticMuRef = 2.783,
    slidingMuRef = 1.796,

    latPeakMin = 0.150,
    latPeakMax = 0.266,
    longPeakMin = 0.140,
    longPeakMax = 0.210,
    peakLoadRef = 10500.0,

    slipAngleRef = 0.22,
    slipRatioRef = 0.35,

    latSlipGain = 0.72,
    longSlipGain = 0.58,
    loadCrushGain = 0.42,
    loadVelocityGain = 0.18,
    contactLossGain = 0.36,
    tireMemoryGain = 0.24,
    damperImpactGain = 0.24,
    complianceGain = 0.24,
    driveWindupGain = 0.20,

    roadShockGain = 0.18,
    roadTextureGain = 0.10,
    roadPathLossGain = 0.18,
    roadSurfaceLimitGain = 0.16,
    loadPathLossGain = 0.18,
    forceLeakGain = 0.12,
    impactGain = 0.16,
    thermalStressGain = 0.12,

    -- Fast input cleanup only. The real carcass response is
    -- applied separately by axle-specific response constants.
    inputFilterTau = 0.012,
    bristleTauLat = 0.014,
    bristleTauLong = 0.012,

    -- ACXRayLab measured response windows.
    frontResponseTau = 0.100,
    rearResponseTau = 0.050,
    frontReleaseTau = 0.160,
    rearReleaseTau = 0.120,

    verticalTau = 0.040,
    verticalReleaseTau = 0.090,

    -- Slow sidewall/carcass energy reservoir.
    frontEnergyBuildTau = 0.450,
    rearEnergyBuildTau = 0.350,
    energyReleaseTau = 0.650,

    -- Energy is a bounded weighted state, not a raw sum.
    -- Keeping headroom is essential so build and release can
    -- both be observed on different cars and tyre sets.
    energyTargetMax = 0.92,
    energySlipWeight = 0.28,
    energyDeformationWeight = 0.24,
    energyResponseErrorWeight = 0.20,
    energyHysteresisWeight = 0.10,
    energyImpactWeight = 0.07,
    energyCombinedSlipWeight = 0.06,
    energyVerticalWeight = 0.03,
    energyIntegrityWeight = 0.02,

    delayTau = 0.100,
    returnTau = 0.110,

    -- Kept for compatibility with decay/clear paths.
    buildTau = 0.040,
    releaseTau = 0.155,
    clearTau = 0.220,

    hysteresisBuildGain = 1.10,
    hysteresisReleaseGain = 0.52,
    hysteresisTau = 0.080,

    lowQualityThreshold = 0.34,
    unloadedQualityThreshold = 0.18,
    deformationThreshold = 0.18,
    compressedThreshold = 0.30,
    saturatedThreshold = 0.72,
    returnThreshold = 0.16,
    delayThreshold = 0.18,
    impactThreshold = 0.55,

    maxCarcass = 1.0,
    maxEnergy = 1.0,
    maxDelay = 1.0,
    maxReturn = 1.0,
    maxHysteresis = 1.0,

    minDt = 0.0001,
    maxDt = 0.100,
    debugStoreInterval = 0.25,
}

local function newWheelState(i)
    return {
        active = false,
        name = WHEEL_NAMES[i],

        load = 0.0,
        prevLoad = 0.0,
        rawLoad = 0.0,
        loadVelocity = 0.0,
        slipAngle = 0.0,
        slipRatio = 0.0,
        rawSlipAngle = 0.0,
        rawSlipRatio = 0.0,

        contactQuality = 1.0,
        contactRaw = 1.0,
        contactTrust = 1.0,
        combinedSlip = 0.0,
        contactLoss = 0.0,
        contactStatus = 0,

        tireMemory = 0.0,
        tireMemoryGrip = 1.0,
        tireMemoryPhaseId = 0,
        memoryThermal = 0.0,
        memoryAbrasion = 0.0,
        memoryHistory = 0.0,
        memoryHeat = 0.0,

        damperHyst = 0.0,
        damperImpact = 0.0,
        damperVelocity = 0.0,
        damperVerticalPulse = 0.0,
        damperPhaseId = 0,

        complianceEnergy = 0.0,
        complianceForceLeak = 0.0,
        virtualToe = 0.0,
        virtualCamber = 0.0,

        loadPathLoss = 0.0,
        loadPathDelay = 0.0,
        loadPathIntegrity = 1.0,
        tireDelivery = 1.0,
        verticalFlow = 0.0,
        forceLeak = 0.0,

        roadShock = 0.0,
        roadTexture = 0.0,
        roadSurfaceLimit = 0.0,
        roadPathLoss = 0.0,
        roadModuleHint = 0.0,
        impactValue = 0.0,
        thermalStress = 0.0,

        windupEnergy = 0.0,
        rearPush = 0.0,

        bristleLat = 0.0,
        bristleLong = 0.0,
        verticalDeflect = 0.0,
        verticalTarget = 0.0,
        verticalNorm = 0.0,
        carcassLat = 0.0,
        carcassLong = 0.0,
        sidewallEnergy = 0.0,
        contactDelay = 0.0,
        returnForce = 0.0,
        deformation = 0.0,
        crush = 0.0,
        hysteresis = 0.0,
        support = 1.0,
        gripGate = 1.0,
        recoveryBias = 0.0,
        heatSeed = 0.0,
        historyStress = 0.0,

        -- XRay-calibrated two-stage response state.
        slipDemand = 0.0,
        responseTau = 0.0,
        energyTau = 0.0,
        responseError = 0.0,
        energyReservoir = 0.0,
        releaseRate = 0.0,
        energyTargetRaw = 0.0,
        energyHeadroom = 1.0,
        energySaturated = false,

        latTarget = 0.0,
        longTarget = 0.0,
        energyTarget = 0.0,
        delayTarget = 0.0,
        returnTarget = 0.0,

        phaseId = PHASE.INIT,
        phaseText = PHASE_TEXT[PHASE.INIT],
    }
end

local state = {
    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    avgLat = 0.0,
    avgLong = 0.0,
    avgEnergy = 0.0,
    avgDelay = 0.0,
    avgReturn = 0.0,
    avgDeformation = 0.0,
    avgSupport = 1.0,
    avgHysteresis = 0.0,
    avgVerticalNorm = 0.0,
    avgGripGate = 1.0,
    avgSlipDemand = 0.0,
    avgResponseTau = 0.0,
    avgEnergyTau = 0.0,
    avgResponseError = 0.0,
    avgReleaseRate = 0.0,
    maxDeformation = 0.0,
    maxEnergyLive = 0.0,
    maxDelayLive = 0.0,

    contactLinked = false,
    memoryLinked = false,
    damperLinked = false,
    complianceLinked = false,
    windupLinked = false,
    loadPathLinked = false,
    roadLinked = false,
    impactLinked = false,
    thermalLinked = false,
    storeOnly = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,

    wheels = {},
}

for i = 0, 3 do
    state.wheels[i] = newWheelState(i)
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

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.0)
    dt = safeNumber(dt, 0.0)
    if tau <= 0.0 then return target end
    return current + (target - current) * clamp(dt / math.max(tau + dt, 0.0001), 0.0, 1.0)
end

local function smoothstep(edge0, edge1, x)
    edge0 = safeNumber(edge0, 0.0)
    edge1 = safeNumber(edge1, 1.0)
    x = safeNumber(x, 0.0)
    if edge0 == edge1 then return x >= edge1 and 1.0 or 0.0 end
    local t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)
end

local function safeStore(key, value)
    if not ac or not ac.store then return end
    pcall(function() ac.store(key, value) end)
end

local function safeLoadRaw(key)
    if not ac or not ac.load then return nil end
    local ok, value = pcall(function() return ac.load(key) end)
    if not ok then return nil end
    return value
end

local function safeLoad(key, defaultValue)
    local value = safeLoadRaw(key)
    if value == nil then return defaultValue or 0.0 end
    return safeNumber(value, defaultValue or 0.0)
end

local function loadAlt(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return safeNumber(value, defaultValue or 0.0), keys[i]
        end
    end
    return defaultValue, nil
end

local function safeField(obj, field, defaultValue)
    if not obj then return defaultValue end
    local ok, value = pcall(function() return obj[field] end)
    if not ok or value == nil then return defaultValue end
    return value
end

local function safeGetCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function() return ac.getCar(0) end)
    if not ok then return nil end
    return car
end

local function getWheel(car, index)
    local wheels = safeField(car, "wheels", nil)
    if not wheels then return nil end

    local ok0, w0 = pcall(function() return wheels[index] end)
    if ok0 and w0 ~= nil then return w0 end

    local ok1, w1 = pcall(function() return wheels[index + 1] end)
    if ok1 and w1 ~= nil then return w1 end

    return nil
end

local function readWheelLoadField(wheel)
    if not wheel then return nil end
    local keys = { "load", "loadK" }
    for i = 1, #keys do
        local value = safeField(wheel, keys[i], nil)
        if value ~= nil then return safeNumber(value, nil) end
    end
    return nil
end

local function interpretDeltaOrAbsoluteLoad(value)
    if value == nil then return nil end
    local v = safeNumber(value, 0.0)
    local absV = math.abs(v)
    local ref = M.params.loadRef

    if v < 0.0 then
        return math.max(0.0, ref + v)
    end

    if absV >= ref * 0.85 and absV <= ref * 2.50 then
        return absV
    end

    return math.max(0.0, ref + v)
end

local function getWheelLoad(wheel, index)
    local load = readWheelLoadField(wheel)
    if load ~= nil then return load end

    local direct, directKey = loadAlt(nil,
        "ngp_contact_load_" .. index,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index,
        "ngp_tire_state_load_" .. index,
        "ngp_sprung_load_" .. index,
        "ngp_load_path_load_" .. index,
        "ngp_hub_load_" .. index
    )
    if directKey ~= nil then return math.max(0.0, direct or 0.0) end

    local dltLoad = safeLoadRaw("ngp_dlt_load_" .. index)
    if dltLoad ~= nil then return interpretDeltaOrAbsoluteLoad(dltLoad) end

    return M.params.loadRef
end

local function readSlip(index, wheel)
    local sa, saKey = loadAlt(nil,
        "ngp_contact_slip_angle_" .. index,
        "ngp_tire_slip_angle_" .. index,
        "ngp_slip_angle_" .. index,
        "ngp_filtered_slip_angle_" .. index,
        "ngp_memory_slip_angle_" .. index,
        "ngp_tire_carcass_slip_angle_" .. index
    )

    local sr, srKey = loadAlt(nil,
        "ngp_contact_slip_ratio_" .. index,
        "ngp_tire_slip_ratio_" .. index,
        "ngp_slip_ratio_" .. index,
        "ngp_filtered_slip_ratio_" .. index,
        "ngp_memory_slip_ratio_" .. index,
        "ngp_tire_carcass_slip_ratio_" .. index
    )

    if saKey == nil then sa = safeField(wheel, "slipAngle", 0.0) end
    if srKey == nil then sr = safeField(wheel, "slipRatio", 0.0) end

    return abs(sa), abs(sr)
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

local function resetLinkFlags()
    state.contactLinked = false
    state.memoryLinked = false
    state.damperLinked = false
    state.complianceLinked = false
    state.windupLinked = false
    state.loadPathLinked = false
    state.roadLinked = false
    state.impactLinked = false
    state.thermalLinked = false
end

local function estimatePeakSlip(load, minPeak, maxPeak)
    local t = clamp(load / math.max(M.params.peakLoadRef, 1.0), 0.0, 1.0)
    return minPeak + (maxPeak - minPeak) * t
end

local function estimateVerticalDeflection(load)
    local p = M.params
    local deflect = math.max(load, 0.0) / math.max(p.virtualSpringRate, 1.0)
    return clamp(deflect, 0.0, p.maxVerticalDeflect)
end

local function estimateContactQuality(ws)
    local loadGate = smoothstep(M.params.loadMin, M.params.loadRef * 0.55, ws.load)
    local slipPenalty = clamp(ws.combinedSlip * 0.32, 0.0, 0.65)
    local unloadPenalty = clamp(math.max(-ws.loadVelocity, 0.0) / math.max(M.params.loadRef * 12.0, 1.0), 0.0, 0.35)
    return clamp(loadGate - slipPenalty - unloadPenalty, 0.0, 1.0)
end

local function readContact(index, ws)
    local cq, cqKey = loadAlt(nil,
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index,
        "ngp_contact_quality_live_" .. index
    )

    local cr, crKey = loadAlt(nil,
        "ngp_contact_raw_" .. index,
        "ngp_contact_raw_quality_" .. index,
        "ngp_tcr_raw_" .. index
    )

    local trust, trustKey = loadAlt(nil,
        "ngp_contact_trust_" .. index,
        "ngp_recovery_trust_" .. index,
        "ngp_contact_grip_gate_" .. index,
        "ngp_carcass_grip_gate_" .. index
    )

    local cs, csKey = loadAlt(nil,
        "ngp_contact_combined_slip_" .. index,
        "ngp_slip_recovery_slip_" .. index,
        "ngp_tdyn_combined_slip_" .. index,
        "ngp_tire_force_combined_slip_" .. index,
        "ngp_susp_tire_combined_slip_" .. index
    )

    local cst, stKey = loadAlt(nil,
        "ngp_contact_status_" .. index,
        "ngp_tcr_status_" .. index
    )

    if cqKey or crKey or trustKey or csKey or stKey then
        state.contactLinked = true
    end

    local latPeak = estimatePeakSlip(ws.load, M.params.latPeakMin, M.params.latPeakMax)
    local longPeak = estimatePeakSlip(ws.load, M.params.longPeakMin, M.params.longPeakMax)
    local slipLatN = ws.slipAngle / math.max(latPeak, 0.001)
    local slipLongN = ws.slipRatio / math.max(longPeak, 0.001)
    local internalCombined = math.sqrt(slipLatN * slipLatN + slipLongN * slipLongN)

    ws.combinedSlip = clamp(safeNumber(cs, internalCombined), 0.0, 2.5)
    local internalQuality = estimateContactQuality(ws)

    ws.contactQuality = clamp(safeNumber(cq, internalQuality), 0.0, 1.2)
    ws.contactRaw = clamp(safeNumber(cr, ws.contactQuality), 0.0, 1.2)
    ws.contactTrust = clamp(safeNumber(trust, ws.contactQuality), 0.0, 1.2)
    ws.contactLoss = clamp(1.0 - math.min(ws.contactQuality, ws.contactTrust, 1.0), 0.0, 1.0)
    ws.contactStatus = safeNumber(cst, 0.0)
end

local function readMemory(index, ws)
    local mem, memKey = loadAlt(nil,
        "ngp_tire_memory_" .. index,
        "ngp_tyrememory_" .. index,
        "ngp_tyre_memory_" .. index,
        "ngp_rubber_memory_" .. index,
        "ngp_memory_" .. index
    )

    local grip, gripKey = loadAlt(nil,
        "ngp_memory_grip_" .. index,
        "ngp_tire_memory_grip_" .. index,
        "ngp_tyrememory_grip_" .. index
    )

    local phase, phaseKey = loadAlt(nil,
        "ngp_tire_memory_phase_id_" .. index,
        "ngp_memory_phase_id_" .. index
    )

    local thermal, thermalKey = loadAlt(nil,
        "ngp_tire_memory_thermal_" .. index,
        "ngp_memory_thermal_" .. index,
        "ngp_slip_thermal_memory_" .. index
    )

    local abrasion, abrasionKey = loadAlt(nil,
        "ngp_tire_memory_abrasion_" .. index,
        "ngp_memory_abrasion_" .. index,
        "ngp_slip_abrasion_memory_" .. index
    )

    local history, historyKey = loadAlt(nil,
        "ngp_tire_memory_history_" .. index,
        "ngp_memory_history_" .. index,
        "ngp_slip_history_memory_" .. index,
        "ngp_carcass_history_stress_" .. index
    )

    local heat, heatKey = loadAlt(nil,
        "ngp_carcass_heat_seed_" .. index,
        "ngp_memory_heat_" .. index,
        "ngp_tire_memory_heat_" .. index,
        "ngp_thermal_stress_" .. index
    )

    if memKey or gripKey or phaseKey or thermalKey or abrasionKey or historyKey or heatKey then
        state.memoryLinked = true
    end

    local internalMemory = clamp(ws.historyStress * 0.75 + math.max(ws.combinedSlip - 0.85, 0.0) * 0.20, 0.0, 1.0)
    ws.tireMemory = clamp(safeNumber(mem, internalMemory), 0.0, 1.2)
    ws.tireMemoryGrip = clamp(safeNumber(grip, 1.0 - ws.tireMemory * 0.16), 0.0, 1.2)
    ws.tireMemoryPhaseId = safeNumber(phase, 0.0)
    ws.memoryThermal = clamp(safeNumber(thermal, 0.0), 0.0, 1.0)
    ws.memoryAbrasion = clamp(safeNumber(abrasion, 0.0), 0.0, 1.0)
    ws.memoryHistory = clamp(safeNumber(history, ws.tireMemory), 0.0, 1.0)
    ws.memoryHeat = clamp(safeNumber(heat, ws.memoryThermal), 0.0, 1.0)
end

local function readDamper(index, ws)
    local dh, dhKey = loadAlt(nil,
        "ngp_damper_hyst_" .. index,
        "ngp_damper_hysteretic_" .. index,
        "ngp_hysteretic_damping_" .. index
    )

    local impact, impactKey = loadAlt(nil,
        "ngp_damper_hyst_impact_" .. index,
        "ngp_damper_impact_" .. index,
        "ngp_road_shock_" .. index
    )

    local velocity, velKey = loadAlt(nil,
        "ngp_damper_hyst_velocity_" .. index,
        "ngp_damper_velocity_" .. index,
        "ngp_susp_speed_" .. index
    )

    local pulse, pulseKey = loadAlt(nil,
        "ngp_damper_vertical_pulse_" .. index,
        "ngp_road_vertical_pulse_" .. index,
        "ngp_road_input_vertical_pulse_" .. index
    )

    local phase, phaseKey = loadAlt(nil,
        "ngp_damper_hyst_phase_id_" .. index,
        "ngp_damper_phase_id_" .. index
    )

    if dhKey or impactKey or velKey or pulseKey or phaseKey then
        state.damperLinked = true
    end

    local loadImpact = math.max(ws.loadVelocity / math.max(M.params.loadRef * 25.0, 1.0), 0.0)
    ws.damperHyst = clamp(safeNumber(dh, ws.hysteresis * 0.55), 0.0, 1.2)
    ws.damperImpact = clamp(safeNumber(impact, loadImpact), 0.0, 1.2)
    ws.damperVelocity = safeNumber(velocity, 0.0)
    ws.damperVerticalPulse = clamp(safeNumber(pulse, 0.0), 0.0, 1.2)
    ws.damperPhaseId = safeNumber(phase, 0.0)
end

local function readCompliance(index, ws)
    local ce, ceKey = loadAlt(nil,
        "ngp_compliance_energy_" .. index,
        "ngp_compliance_stack_energy_" .. index,
        "ngp_cs_energy_" .. index
    )

    local leak, leakKey = loadAlt(nil,
        "ngp_compliance_force_leak_" .. index,
        "ngp_load_path_force_leak_" .. index,
        "ngp_road_force_leak_" .. index,
        "ngp_force_leak"
    )

    local toe, toeKey = loadAlt(nil,
        "ngp_virtual_toe_" .. index,
        "ngp_compliance_virtual_toe_" .. index,
        "ngp_control_arm_toe_" .. index,
        "ngp_arm_toe_" .. index
    )

    local camber, camberKey = loadAlt(nil,
        "ngp_virtual_camber_" .. index,
        "ngp_compliance_virtual_camber_" .. index,
        "ngp_control_arm_camber_" .. index,
        "ngp_arm_camber_" .. index
    )

    if ceKey or leakKey or toeKey or camberKey then
        state.complianceLinked = true
    end

    ws.complianceEnergy = clamp(safeNumber(ce, 0.0), 0.0, 1.2)
    ws.complianceForceLeak = clamp(safeNumber(leak, 0.0), 0.0, 1.2)
    ws.virtualToe = safeNumber(toe, 0.0)
    ws.virtualCamber = safeNumber(camber, 0.0)
end

local function readLoadPath(index, ws)
    local loss, lossKey = loadAlt(nil,
        "ngp_load_path_loss_" .. index,
        "ngp_lp_loss_" .. index,
        "ngp_road_path_loss_" .. index,
        "ngp_rii_path_loss_" .. index
    )

    local delay, delayKey = loadAlt(nil,
        "ngp_load_path_delay_" .. index,
        "ngp_lp_delay_" .. index
    )

    local integrity, integrityKey = loadAlt(nil,
        "ngp_load_path_integrity_" .. index,
        "ngp_lp_integrity_" .. index,
        "ngp_road_integrity_" .. index
    )

    local delivery, deliveryKey = loadAlt(nil,
        "ngp_load_path_tire_delivery_" .. index,
        "ngp_lp_tire_delivery_" .. index,
        "ngp_road_tire_delivery_" .. index
    )

    local verticalFlow, flowKey = loadAlt(nil,
        "ngp_load_path_vertical_flow_" .. index,
        "ngp_lp_vertical_flow_" .. index,
        "ngp_road_vertical_flow_" .. index,
        "ngp_rii_avg_vertical_flow"
    )

    local forceLeak, forceKey = loadAlt(nil,
        "ngp_load_path_force_leak_" .. index,
        "ngp_lp_force_leak_" .. index,
        "ngp_road_force_leak_" .. index
    )

    if lossKey or delayKey or integrityKey or deliveryKey or flowKey or forceKey then
        state.loadPathLinked = true
    end

    ws.loadPathLoss = clamp(safeNumber(loss, 0.0), 0.0, 1.2)
    ws.loadPathDelay = clamp(safeNumber(delay, 0.0), 0.0, 1.2)
    ws.loadPathIntegrity = clamp(safeNumber(integrity, 1.0), 0.0, 1.2)
    ws.tireDelivery = clamp(safeNumber(delivery, 1.0), 0.0, 1.2)
    ws.verticalFlow = clamp(safeNumber(verticalFlow, 0.0), 0.0, 1.2)
    ws.forceLeak = clamp(safeNumber(forceLeak, 0.0), 0.0, 1.2)
end

local function readRoadAndImpact(index, ws)
    local shock, shockKey = loadAlt(nil,
        "ngp_road_shock_" .. index,
        "ngp_rii_shock_" .. index,
        "ngp_road_impact_" .. index,
        "ngp_rii_impact_" .. index
    )

    local texture, textureKey = loadAlt(nil,
        "ngp_road_texture_" .. index,
        "ngp_rii_texture_" .. index,
        "ngp_road_noise_" .. index,
        "ngp_rii_noise_" .. index
    )

    local surface, surfaceKey = loadAlt(nil,
        "ngp_road_surface_limit_" .. index,
        "ngp_rii_surface_limit_" .. index
    )

    local path, pathKey = loadAlt(nil,
        "ngp_road_path_loss_" .. index,
        "ngp_rii_path_loss_" .. index
    )

    local hint, hintKey = loadAlt(nil,
        "ngp_road_module_hint_" .. index,
        "ngp_rii_hint_" .. index
    )

    local impact, impactKey = loadAlt(nil,
        "ngp_impact_wheel_" .. index,
        "ngp_impact_value_" .. index,
        "ngp_damage_event_wheel_" .. index
    )

    local thermal, thermalKey = loadAlt(nil,
        "ngp_thermal_stress_" .. index,
        "ngp_virtual_thermal_stress",
        "ngp_brake_root_heat_avg"
    )

    if shockKey or textureKey or surfaceKey or pathKey or hintKey then
        state.roadLinked = true
    end
    if impactKey then state.impactLinked = true end
    if thermalKey then state.thermalLinked = true end

    ws.roadShock = clamp(safeNumber(shock, 0.0), 0.0, 1.2)
    ws.roadTexture = clamp(safeNumber(texture, 0.0), 0.0, 1.2)
    ws.roadSurfaceLimit = clamp(safeNumber(surface, 0.0), 0.0, 1.2)
    ws.roadPathLoss = clamp(safeNumber(path, ws.loadPathLoss), 0.0, 1.2)
    ws.roadModuleHint = clamp(safeNumber(hint, 0.0), 0.0, 1.2)
    ws.impactValue = clamp(safeNumber(impact, 0.0), 0.0, 1.2)
    ws.thermalStress = clamp(safeNumber(thermal, 0.0), 0.0, 1.2)
end

local function readWindup(index, ws)
    local windup, windupKey = loadAlt(nil,
        "ngp_windup_energy",
        "ngp_driveline_energy",
        "ngp_drive_windup_energy",
        "ngp_drive_lash"
    )

    local push, pushKey = loadAlt(nil,
        "ngp_drive_soft_rear_push",
        "ngp_driveline_rear_push",
        "ngp_rear_axle_windup"
    )

    if windupKey or pushKey then state.windupLinked = true end

    if index >= 2 then
        ws.windupEnergy = clamp(safeNumber(windup, 0.0), 0.0, 1.2)
        ws.rearPush = clamp(safeNumber(push, 0.0), 0.0, 1.5)
    else
        ws.windupEnergy = 0.0
        ws.rearPush = 0.0
    end
end

local function readCoupledInputs(index, ws)
    readContact(index, ws)
    readMemory(index, ws)
    readDamper(index, ws)
    readCompliance(index, ws)
    readLoadPath(index, ws)
    readRoadAndImpact(index, ws)
    readWindup(index, ws)
end

local function phaseFromState(ws)
    if ws.contactQuality <= M.params.unloadedQualityThreshold or ws.load <= M.params.loadMin then
        return PHASE.UNLOADED, PHASE_TEXT[PHASE.UNLOADED]
    end
    if math.max(ws.damperImpact, ws.impactValue, ws.roadShock) > M.params.impactThreshold then
        return PHASE.IMPACT, PHASE_TEXT[PHASE.IMPACT]
    end
    if ws.deformation > M.params.saturatedThreshold or ws.combinedSlip > 1.25 then
        return PHASE.SATURATED, PHASE_TEXT[PHASE.SATURATED]
    end
    if ws.returnForce > M.params.returnThreshold and ws.deformation < ws.sidewallEnergy then
        return PHASE.RETURNING, PHASE_TEXT[PHASE.RETURNING]
    end
    if ws.contactDelay > M.params.delayThreshold then
        return PHASE.DELAYED, PHASE_TEXT[PHASE.DELAYED]
    end
    if ws.verticalNorm > M.params.compressedThreshold then
        return PHASE.COMPRESSED, PHASE_TEXT[PHASE.COMPRESSED]
    end
    if ws.carcassLat > ws.carcassLong and ws.carcassLat > M.params.deformationThreshold then
        return PHASE.SIDEWALL_LAT, PHASE_TEXT[PHASE.SIDEWALL_LAT]
    end
    if ws.carcassLong >= ws.carcassLat and ws.carcassLong > M.params.deformationThreshold then
        return PHASE.CARCASS_LONG, PHASE_TEXT[PHASE.CARCASS_LONG]
    end
    return PHASE.SUPPORTED, PHASE_TEXT[PHASE.SUPPORTED]
end

local function updateHistory(ws, dt)
    local stressTarget = clamp(
        math.max(ws.combinedSlip - 0.70, 0.0) * 0.55
        + ws.hysteresis * 0.22
        + ws.damperImpact * 0.18
        + ws.roadShock * 0.12
        + ws.roadSurfaceLimit * 0.14
        + ws.thermalStress * M.params.thermalStressGain
        + (1.0 - ws.contactQuality) * 0.20,
        0.0,
        1.0
    )

    local tau = stressTarget > ws.historyStress and 0.22 or 1.80
    ws.historyStress = clamp(lowPass(ws.historyStress, stressTarget, tau, dt), 0.0, 1.0)

    ws.heatSeed = clamp(
        ws.historyStress * 0.55
        + ws.combinedSlip * 0.20
        + ws.hysteresis * 0.16
        + ws.damperImpact * 0.12
        + ws.memoryHeat * 0.12
        + ws.thermalStress * 0.10,
        0.0,
        1.0
    )
end

local function updateCarcass(index, ws, dt)
    local p = M.params

    local rearAxle = index >= 2
    local responseTau =
        rearAxle and
        p.rearResponseTau or
        p.frontResponseTau

    local responseReleaseTau =
        rearAxle and
        p.rearReleaseTau or
        p.frontReleaseTau

    local energyBuildTau =
        rearAxle and
        p.rearEnergyBuildTau or
        p.frontEnergyBuildTau

    ws.responseTau = responseTau
    ws.energyTau = energyBuildTau

    local latPeak = estimatePeakSlip(ws.load, p.latPeakMin, p.latPeakMax)
    local longPeak = estimatePeakSlip(ws.load, p.longPeakMin, p.longPeakMax)

    local saNorm = clamp(ws.slipAngle / math.max(latPeak, 0.001), 0.0, 2.0)
    local srNorm = clamp(ws.slipRatio / math.max(longPeak, 0.001), 0.0, 2.0)

    local crush = smoothstep(p.loadCrushStart, p.loadCrushFull, ws.load)
    local contactLoss = clamp(1.0 - math.min(ws.contactQuality, ws.contactTrust, 1.0), 0.0, 1.0)
    local gripLoss = clamp(1.0 - ws.tireMemoryGrip, 0.0, 1.0)
    local integrityLoss = clamp(1.0 - math.min(ws.loadPathIntegrity, 1.0), 0.0, 1.0)
    local deliveryLoss = clamp(1.0 - math.min(ws.tireDelivery, 1.0), 0.0, 1.0)

    ws.crush = crush

    local verticalTarget = estimateVerticalDeflection(ws.load)
    local verticalTau = verticalTarget > ws.verticalDeflect and p.verticalTau or p.verticalReleaseTau
    ws.verticalDeflect = clamp(lowPass(ws.verticalDeflect, verticalTarget, verticalTau, dt), 0.0, p.maxVerticalDeflect)
    ws.verticalTarget = verticalTarget
    ws.verticalNorm = clamp(ws.verticalDeflect / math.max(p.verticalDeflectRef, 0.001), 0.0, 1.5)

    local loadVelNorm = clamp(math.abs(ws.loadVelocity) / math.max(p.loadRef * 18.0, 1.0), 0.0, 1.0)
    local roadLoad = clamp(ws.roadTexture * 0.28 + ws.roadShock * 0.45 + ws.verticalFlow * 0.18 + ws.impactValue * 0.26, 0.0, 1.2)

    local bristleLatTarget = clamp(
        saNorm * p.bristleLatGain
        + math.abs(ws.virtualToe) * 0.75
        + ws.roadSurfaceLimit * 0.10
        + ws.forceLeak * 0.05,
        0.0,
        p.maxCarcass
    )

    local bristleLongTarget = clamp(
        srNorm * p.bristleLongGain
        + ws.rearPush * 0.08
        + ws.windupEnergy * 0.06
        + ws.loadPathDelay * 0.06,
        0.0,
        p.maxCarcass
    )

    ws.slipDemand = clamp(
        math.sqrt(
            bristleLatTarget * bristleLatTarget
            + bristleLongTarget * bristleLongTarget
        ) * 0.70710678,
        0.0,
        1.0
    )

    ws.bristleLat = clamp(lowPass(ws.bristleLat, bristleLatTarget, p.bristleTauLat, dt), 0.0, p.maxCarcass)
    ws.bristleLong = clamp(lowPass(ws.bristleLong, bristleLongTarget, p.bristleTauLong, dt), 0.0, p.maxCarcass)

    local latTarget =
        ws.bristleLat
        + contactLoss * p.contactLossGain
        + ws.tireMemory * p.tireMemoryGain
        + ws.complianceEnergy * p.complianceGain
        + ws.complianceForceLeak * 0.10
        + math.abs(ws.virtualToe) * 0.60
        + math.abs(ws.virtualCamber) * 0.32
        + ws.roadSurfaceLimit * p.roadSurfaceLimitGain
        + math.max(ws.loadPathLoss, ws.roadPathLoss) * p.loadPathLossGain
        + ws.forceLeak * p.forceLeakGain

    local longTarget =
        ws.bristleLong
        + crush * p.loadCrushGain
        + ws.verticalNorm * 0.18
        + loadVelNorm * p.loadVelocityGain
        + contactLoss * 0.22
        + gripLoss * 0.20
        + ws.windupEnergy * p.driveWindupGain
        + ws.rearPush * 0.10
        + roadLoad * p.roadShockGain
        + deliveryLoss * 0.10

    local loading = math.max(latTarget + longTarget - (ws.carcassLat + ws.carcassLong), 0.0)
    local unloading = math.max((ws.carcassLat + ws.carcassLong) - (latTarget + longTarget), 0.0)
    local hystTarget = clamp(
        loading * p.hysteresisBuildGain
        + unloading * p.hysteresisReleaseGain
        + loadVelNorm * 0.28
        + ws.damperHyst * 0.20
        + ws.roadTexture * p.roadTextureGain,
        0.0,
        p.maxHysteresis
    )
    ws.hysteresis = clamp(lowPass(ws.hysteresis, hystTarget, p.hysteresisTau, dt), 0.0, p.maxHysteresis)

    local responseErrorTarget = clamp(
        math.max(
            math.abs(latTarget - ws.carcassLat),
            math.abs(longTarget - ws.carcassLong)
        ),
        0.0,
        1.0
    )

    --========================================================
    -- XRay v2.1 normalized energy target
    --
    -- The previous raw sum could exceed 1.0 even at modest
    -- demand, pinning the reservoir at maxEnergy. A weighted
    -- bounded mixture keeps cross-vehicle headroom and lets
    -- the reservoir build and release continuously.
    --========================================================

    local deformationDemand =
        clamp(
            math.max(latTarget, longTarget),
            0.0,
            1.0
        )

    local impactDemand =
        clamp(
            math.max(
                ws.damperImpact,
                ws.impactValue,
                ws.roadShock
            ),
            0.0,
            1.0
        )

    local combinedDemand =
        clamp(
            ws.combinedSlip / 1.5,
            0.0,
            1.0
        )

    local verticalEnergy =
        clamp(
            ws.verticalNorm / 1.5,
            0.0,
            1.0
        )

    local energyTargetRaw =
        ws.slipDemand
            * p.energySlipWeight
        + deformationDemand
            * p.energyDeformationWeight
        + responseErrorTarget
            * p.energyResponseErrorWeight
        + ws.hysteresis
            * p.energyHysteresisWeight
        + impactDemand
            * p.energyImpactWeight
        + combinedDemand
            * p.energyCombinedSlipWeight
        + verticalEnergy
            * p.energyVerticalWeight
        + integrityLoss
            * p.energyIntegrityWeight

    local energyTarget =
        clamp(
            energyTargetRaw,
            0.0,
            math.min(
                p.energyTargetMax,
                p.maxEnergy
            )
        )

    ws.energyTargetRaw = energyTargetRaw

    local delayTarget =
        ws.tireMemory * 0.24
        + ws.complianceEnergy * 0.20
        + contactLoss * 0.34
        + ws.hysteresis * 0.20
        + ws.loadPathDelay * 0.14
        + math.max(ws.roadPathLoss, ws.loadPathLoss) * p.roadPathLossGain
        + math.max(0.0, energyTarget - ws.sidewallEnergy) * 0.24
        + responseErrorTarget * 0.28

    local returnTarget =
        math.max(ws.sidewallEnergy - energyTarget, 0.0) * 0.82
        + math.max(ws.carcassLat - latTarget, 0.0) * 0.38
        + math.max(ws.carcassLong - longTarget, 0.0) * 0.34
        + math.max(ws.verticalDeflect - verticalTarget, 0.0) / math.max(p.verticalDeflectRef, 0.001) * 0.16
        + math.max(0.0, ws.contactQuality - ws.contactRaw) * 0.12

    latTarget = clamp(latTarget, 0.0, p.maxCarcass)
    longTarget = clamp(longTarget, 0.0, p.maxCarcass)
    energyTarget = clamp(energyTarget, 0.0, p.maxEnergy)
    delayTarget = clamp(delayTarget, 0.0, p.maxDelay)
    returnTarget = clamp(returnTarget, 0.0, p.maxReturn)

    local tauLat =
        latTarget > ws.carcassLat and
        responseTau or
        responseReleaseTau

    local tauLong =
        longTarget > ws.carcassLong and
        responseTau or
        responseReleaseTau

    local tauEnergy =
        energyTarget > ws.sidewallEnergy and
        energyBuildTau or
        p.energyReleaseTau

    local previousEnergy = ws.sidewallEnergy

    ws.carcassLat = clamp(
        lowPass(
            ws.carcassLat,
            latTarget,
            tauLat,
            dt
        ),
        0.0,
        p.maxCarcass
    )

    ws.carcassLong = clamp(
        lowPass(
            ws.carcassLong,
            longTarget,
            tauLong,
            dt
        ),
        0.0,
        p.maxCarcass
    )

    ws.sidewallEnergy = clamp(
        lowPass(
            ws.sidewallEnergy,
            energyTarget,
            tauEnergy,
            dt
        ),
        0.0,
        p.maxEnergy
    )

    ws.energyReservoir = ws.sidewallEnergy
    ws.energyHeadroom =
        clamp(
            p.maxEnergy - ws.sidewallEnergy,
            0.0,
            p.maxEnergy
        )

    ws.energySaturated =
        ws.sidewallEnergy >=
        (
            p.energyTargetMax
            - 0.005
        )

    ws.responseError = clamp(
        lowPass(
            ws.responseError,
            responseErrorTarget,
            p.delayTau,
            dt
        ),
        0.0,
        1.0
    )

    ws.releaseRate = clamp(
        math.max(
            previousEnergy
            - ws.sidewallEnergy,
            0.0
        )
        /
        math.max(dt, p.minDt),
        0.0,
        8.0
    )

    ws.contactDelay = clamp(lowPass(ws.contactDelay, delayTarget, p.delayTau, dt), 0.0, p.maxDelay)
    ws.returnForce = clamp(lowPass(ws.returnForce, returnTarget, p.returnTau, dt), 0.0, p.maxReturn)

    ws.deformation = clamp((ws.carcassLat + ws.carcassLong + ws.sidewallEnergy + ws.verticalNorm * 0.55) / 3.55, 0.0, 1.0)

    local muDrop = clamp((p.staticMuRef - p.slidingMuRef) / math.max(p.staticMuRef, 0.001), 0.0, 1.0)
    ws.support = clamp(
        1.0
        - ws.deformation * 0.26
        - contactLoss * 0.42
        - ws.historyStress * 0.15
        - ws.loadPathLoss * 0.10
        - integrityLoss * 0.12
        - deliveryLoss * 0.10
        - muDrop * math.max(ws.combinedSlip - 1.0, 0.0) * 0.20,
        0.0,
        1.0
    )
    ws.gripGate = clamp(ws.support * (1.0 - ws.contactDelay * 0.18) * (1.0 - ws.roadSurfaceLimit * 0.08), 0.0, 1.0)
    ws.recoveryBias = clamp(ws.returnForce * 0.65 + ws.hysteresis * 0.22 + ws.historyStress * 0.12 + ws.memoryHistory * 0.08, 0.0, 1.0)

    ws.latTarget = latTarget
    ws.longTarget = longTarget
    ws.energyTarget = energyTarget
    ws.delayTarget = delayTarget
    ws.returnTarget = returnTarget

    updateHistory(ws, dt)

    ws.phaseId, ws.phaseText = phaseFromState(ws)
end

local function decayWheel(index, dt, noWheelPhase)
    local ws = state.wheels[index]
    ws.active = false
    ws.rawLoad = lowPass(ws.rawLoad, M.params.loadRef, M.params.clearTau, dt)
    ws.load = lowPass(ws.load, M.params.loadRef, M.params.clearTau, dt)
    ws.prevLoad = ws.load
    ws.loadVelocity = lowPass(ws.loadVelocity, 0.0, M.params.clearTau, dt)
    ws.slipAngle = lowPass(ws.slipAngle, 0.0, M.params.releaseTau, dt)
    ws.slipRatio = lowPass(ws.slipRatio, 0.0, M.params.releaseTau, dt)
    ws.rawSlipAngle = 0.0
    ws.rawSlipRatio = 0.0

    ws.contactQuality = lowPass(ws.contactQuality, 1.0, M.params.clearTau, dt)
    ws.contactRaw = lowPass(ws.contactRaw, 1.0, M.params.clearTau, dt)
    ws.contactTrust = lowPass(ws.contactTrust, 1.0, M.params.clearTau, dt)
    ws.combinedSlip = lowPass(ws.combinedSlip, 0.0, M.params.releaseTau, dt)
    ws.contactLoss = lowPass(ws.contactLoss, 0.0, M.params.clearTau, dt)
    ws.contactStatus = 0

    ws.bristleLat = lowPass(ws.bristleLat, 0.0, M.params.releaseTau, dt)
    ws.bristleLong = lowPass(ws.bristleLong, 0.0, M.params.releaseTau, dt)
    ws.verticalDeflect = lowPass(ws.verticalDeflect, 0.0, M.params.verticalReleaseTau, dt)
    ws.verticalTarget = 0.0
    ws.verticalNorm = lowPass(ws.verticalNorm, 0.0, M.params.verticalReleaseTau, dt)
    ws.carcassLat = lowPass(ws.carcassLat, 0.0, M.params.releaseTau, dt)
    ws.carcassLong = lowPass(ws.carcassLong, 0.0, M.params.releaseTau, dt)
    ws.sidewallEnergy = lowPass(ws.sidewallEnergy, 0.0, M.params.releaseTau, dt)
    ws.contactDelay = lowPass(ws.contactDelay, 0.0, M.params.delayTau, dt)
    ws.returnForce = lowPass(ws.returnForce, 0.0, M.params.returnTau, dt)
    ws.hysteresis = lowPass(ws.hysteresis, 0.0, M.params.hysteresisTau, dt)
    ws.deformation = lowPass(ws.deformation, 0.0, M.params.releaseTau, dt)
    ws.crush = lowPass(ws.crush, 0.0, M.params.releaseTau, dt)
    ws.support = lowPass(ws.support, 1.0, M.params.clearTau, dt)
    ws.gripGate = lowPass(ws.gripGate, 1.0, M.params.clearTau, dt)
    ws.recoveryBias = lowPass(ws.recoveryBias, 0.0, M.params.releaseTau, dt)
    ws.heatSeed = lowPass(ws.heatSeed, 0.0, M.params.clearTau, dt)
    ws.historyStress = lowPass(ws.historyStress, 0.0, 1.2, dt)
    ws.slipDemand = lowPass(ws.slipDemand, 0.0, M.params.releaseTau, dt)
    ws.responseError = lowPass(ws.responseError, 0.0, M.params.releaseTau, dt)
    ws.energyReservoir = ws.sidewallEnergy
    ws.releaseRate = lowPass(ws.releaseRate, 0.0, M.params.clearTau, dt)
    ws.energyTargetRaw = lowPass(ws.energyTargetRaw, 0.0, M.params.clearTau, dt)
    ws.energyHeadroom = clamp(
        M.params.maxEnergy - ws.sidewallEnergy,
        0.0,
        M.params.maxEnergy
    )
    ws.energySaturated = false

    ws.phaseId = noWheelPhase or PHASE.INIT
    ws.phaseText = noWheelPhase == PHASE.UNLOADED and "NO WHEEL" or "STORE ONLY"
end

local function exportWheel(index, ws)
    safeStore("ngp_tire_carcass_lat_" .. index, ws.carcassLat or 0.0)
    safeStore("ngp_tire_carcass_long_" .. index, ws.carcassLong or 0.0)
    safeStore("ngp_tire_sidewall_energy_" .. index, ws.sidewallEnergy or 0.0)
    safeStore("ngp_tire_contact_delay_" .. index, ws.contactDelay or 0.0)
    safeStore("ngp_tire_return_force_" .. index, ws.returnForce or 0.0)
    safeStore("ngp_tire_deformation_" .. index, ws.deformation or 0.0)
    safeStore("ngp_tire_carcass_phase_id_" .. index, ws.phaseId or 0.0)
    safeStore("ngp_tire_carcass_phase_" .. index, ws.phaseText or "UNKNOWN")

    safeStore("ngp_carcass_lat_" .. index, ws.carcassLat or 0.0)
    safeStore("ngp_carcass_long_" .. index, ws.carcassLong or 0.0)
    safeStore("ngp_sidewall_energy_" .. index, ws.sidewallEnergy or 0.0)
    safeStore("ngp_contact_delay_" .. index, ws.contactDelay or 0.0)
    safeStore("ngp_return_force_" .. index, ws.returnForce or 0.0)

    safeStore("ngp_carcass_bristle_lat_" .. index, ws.bristleLat or 0.0)
    safeStore("ngp_carcass_bristle_long_" .. index, ws.bristleLong or 0.0)
    safeStore("ngp_carcass_vertical_deflect_" .. index, ws.verticalDeflect or 0.0)
    safeStore("ngp_carcass_vertical_norm_" .. index, ws.verticalNorm or 0.0)
    safeStore("ngp_carcass_hysteresis_" .. index, ws.hysteresis or 0.0)
    safeStore("ngp_carcass_support_" .. index, ws.support or 1.0)
    safeStore("ngp_carcass_grip_gate_" .. index, ws.gripGate or 1.0)
    safeStore("ngp_carcass_recovery_bias_" .. index, ws.recoveryBias or 0.0)
    safeStore("ngp_carcass_heat_seed_" .. index, ws.heatSeed or 0.0)
    safeStore("ngp_carcass_history_stress_" .. index, ws.historyStress or 0.0)
    safeStore("ngp_carcass_crush_" .. index, ws.crush or 0.0)

    -- XRay-calibrated dynamic state.
    safeStore("ngp_carcass_slip_demand_" .. index, ws.slipDemand or 0.0)
    safeStore("ngp_carcass_response_tau_" .. index, ws.responseTau or 0.0)
    safeStore("ngp_carcass_energy_tau_" .. index, ws.energyTau or 0.0)
    safeStore("ngp_carcass_response_error_" .. index, ws.responseError or 0.0)
    safeStore("ngp_carcass_energy_reservoir_" .. index, ws.energyReservoir or 0.0)
    safeStore("ngp_carcass_release_rate_" .. index, ws.releaseRate or 0.0)
    safeStore("ngp_carcass_energy_target_" .. index, ws.energyTarget or 0.0)
    safeStore("ngp_carcass_energy_target_raw_" .. index, ws.energyTargetRaw or 0.0)
    safeStore("ngp_carcass_energy_headroom_" .. index, ws.energyHeadroom or 0.0)
    safeStore("ngp_carcass_energy_saturated_" .. index, ws.energySaturated and 1 or 0)

    safeStore("ngp_tc_lat_" .. index, ws.carcassLat or 0.0)
    safeStore("ngp_tc_long_" .. index, ws.carcassLong or 0.0)
    safeStore("ngp_tc_energy_" .. index, ws.sidewallEnergy or 0.0)
    safeStore("ngp_tc_delay_" .. index, ws.contactDelay or 0.0)
    safeStore("ngp_tc_return_" .. index, ws.returnForce or 0.0)
    safeStore("ngp_tc_support_" .. index, ws.support or 1.0)
    safeStore("ngp_tc_grip_gate_" .. index, ws.gripGate or 1.0)

    if not state.debugStoreNow then return end

    safeStore("ngp_tire_carcass_load_" .. index, ws.load or 0.0)
    safeStore("ngp_tire_carcass_load_velocity_" .. index, ws.loadVelocity or 0.0)
    safeStore("ngp_tire_carcass_slip_angle_" .. index, ws.slipAngle or 0.0)
    safeStore("ngp_tire_carcass_slip_ratio_" .. index, ws.slipRatio or 0.0)
    safeStore("ngp_tire_carcass_cq_" .. index, ws.contactQuality or 1.0)
    safeStore("ngp_tire_carcass_combined_slip_" .. index, ws.combinedSlip or 0.0)
    safeStore("ngp_tire_carcass_memory_" .. index, ws.tireMemory or 0.0)
    safeStore("ngp_tire_carcass_hyst_" .. index, ws.damperHyst or 0.0)
    safeStore("ngp_tire_carcass_compliance_" .. index, ws.complianceEnergy or 0.0)
    safeStore("ngp_tire_carcass_crush_" .. index, ws.crush or 0.0)
    safeStore("ngp_tire_carcass_support_" .. index, ws.support or 1.0)
    safeStore("ngp_tire_carcass_trust_" .. index, ws.contactTrust or 1.0)
    safeStore("ngp_tire_carcass_road_shock_" .. index, ws.roadShock or 0.0)
    safeStore("ngp_tire_carcass_path_loss_" .. index, ws.loadPathLoss or 0.0)
end

local function exportGlobal()
    safeStore("ngp_tire_carcass_status", state.status or "UNKNOWN")
    safeStore("ngp_tire_carcass_update_count", state.updateCount or 0)
    safeStore("ngp_tire_carcass_wheels_valid", state.wheelsValid and 1 or 0)

    safeStore("ngp_tire_carcass_avg_lat", state.avgLat or 0.0)
    safeStore("ngp_tire_carcass_avg_long", state.avgLong or 0.0)
    safeStore("ngp_tire_carcass_avg_energy", state.avgEnergy or 0.0)
    safeStore("ngp_tire_carcass_avg_delay", state.avgDelay or 0.0)
    safeStore("ngp_tire_carcass_avg_return", state.avgReturn or 0.0)
    safeStore("ngp_tire_carcass_avg_deformation", state.avgDeformation or 0.0)
    safeStore("ngp_tire_carcass_avg_support", state.avgSupport or 1.0)
    safeStore("ngp_tire_carcass_avg_hysteresis", state.avgHysteresis or 0.0)

    safeStore("ngp_carcass_avg_vertical_norm", state.avgVerticalNorm or 0.0)
    safeStore("ngp_carcass_avg_grip_gate", state.avgGripGate or 1.0)
    safeStore("ngp_carcass_avg_slip_demand", state.avgSlipDemand or 0.0)
    safeStore("ngp_carcass_avg_response_tau", state.avgResponseTau or 0.0)
    safeStore("ngp_carcass_avg_energy_tau", state.avgEnergyTau or 0.0)
    safeStore("ngp_carcass_avg_response_error", state.avgResponseError or 0.0)
    safeStore("ngp_carcass_avg_release_rate", state.avgReleaseRate or 0.0)
    safeStore("ngp_carcass_max_deformation", state.maxDeformation or 0.0)
    safeStore("ngp_carcass_max_energy", state.maxEnergyLive or 0.0)
    safeStore("ngp_carcass_max_delay", state.maxDelayLive or 0.0)
    safeStore("ngp_tc_avg_deformation", state.avgDeformation or 0.0)
    safeStore("ngp_tc_avg_support", state.avgSupport or 1.0)
    safeStore("ngp_tc_max_deformation", state.maxDeformation or 0.0)

    if not state.debugStoreNow then return end

    safeStore("ngp_tire_carcass_contact_linked", state.contactLinked and 1 or 0)
    safeStore("ngp_tire_carcass_memory_linked", state.memoryLinked and 1 or 0)
    safeStore("ngp_tire_carcass_damper_linked", state.damperLinked and 1 or 0)
    safeStore("ngp_tire_carcass_compliance_linked", state.complianceLinked and 1 or 0)
    safeStore("ngp_tire_carcass_windup_linked", state.windupLinked and 1 or 0)
    safeStore("ngp_tire_carcass_load_path_linked", state.loadPathLinked and 1 or 0)
    safeStore("ngp_tire_carcass_road_linked", state.roadLinked and 1 or 0)
    safeStore("ngp_tire_carcass_impact_linked", state.impactLinked and 1 or 0)
    safeStore("ngp_tire_carcass_thermal_linked", state.thermalLinked and 1 or 0)
    safeStore("ngp_tire_carcass_store_only", state.storeOnly and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i, state.wheels[i])
    end
    exportGlobal()
end

local function updateWheelFromInputs(index, wheel, dt)
    local ws = state.wheels[index]
    ws.active = wheel ~= nil

    local rawSA, rawSR = readSlip(index, wheel)
    ws.rawSlipAngle = rawSA
    ws.rawSlipRatio = rawSR
    ws.slipAngle = lowPass(ws.slipAngle, rawSA, M.params.inputFilterTau, dt)
    ws.slipRatio = lowPass(ws.slipRatio, rawSR, M.params.inputFilterTau, dt)

    local rawLoad = math.abs(getWheelLoad(wheel, index))
    ws.rawLoad = rawLoad
    if (ws.load or 0.0) <= 0.0 then
        ws.prevLoad = rawLoad
        ws.load = rawLoad
        ws.loadVelocity = 0.0
    else
        ws.prevLoad = ws.load
        ws.load = lowPass(ws.load, rawLoad, 0.055, dt)
        ws.loadVelocity = (ws.load - ws.prevLoad) / math.max(dt, M.params.minDt)
    end

    readCoupledInputs(index, ws)
    updateCarcass(index, ws, dt)
end

local function updateAverages()
    local sumLat, sumLong, sumEnergy, sumDelay = 0.0, 0.0, 0.0, 0.0
    local sumReturn, sumDeformation, sumSupport, sumHysteresis = 0.0, 0.0, 0.0, 0.0
    local sumVertical, sumGrip = 0.0, 0.0
    local sumDemand, sumResponseTau, sumEnergyTau = 0.0, 0.0, 0.0
    local sumResponseError, sumReleaseRate = 0.0, 0.0
    local maxDef, maxEnergy, maxDelay = 0.0, 0.0, 0.0

    for i = 0, 3 do
        local ws = state.wheels[i]
        sumLat = sumLat + (ws.carcassLat or 0.0)
        sumLong = sumLong + (ws.carcassLong or 0.0)
        sumEnergy = sumEnergy + (ws.sidewallEnergy or 0.0)
        sumDelay = sumDelay + (ws.contactDelay or 0.0)
        sumReturn = sumReturn + (ws.returnForce or 0.0)
        sumDeformation = sumDeformation + (ws.deformation or 0.0)
        sumSupport = sumSupport + (ws.support or 1.0)
        sumHysteresis = sumHysteresis + (ws.hysteresis or 0.0)
        sumVertical = sumVertical + (ws.verticalNorm or 0.0)
        sumGrip = sumGrip + (ws.gripGate or 1.0)
        sumDemand = sumDemand + (ws.slipDemand or 0.0)
        sumResponseTau = sumResponseTau + (ws.responseTau or 0.0)
        sumEnergyTau = sumEnergyTau + (ws.energyTau or 0.0)
        sumResponseError = sumResponseError + (ws.responseError or 0.0)
        sumReleaseRate = sumReleaseRate + (ws.releaseRate or 0.0)
        maxDef = math.max(maxDef, ws.deformation or 0.0)
        maxEnergy = math.max(maxEnergy, ws.sidewallEnergy or 0.0)
        maxDelay = math.max(maxDelay, ws.contactDelay or 0.0)
    end

    state.avgLat = sumLat * 0.25
    state.avgLong = sumLong * 0.25
    state.avgEnergy = sumEnergy * 0.25
    state.avgDelay = sumDelay * 0.25
    state.avgReturn = sumReturn * 0.25
    state.avgDeformation = sumDeformation * 0.25
    state.avgSupport = sumSupport * 0.25
    state.avgHysteresis = sumHysteresis * 0.25
    state.avgVerticalNorm = sumVertical * 0.25
    state.avgGripGate = sumGrip * 0.25
    state.avgSlipDemand = sumDemand * 0.25
    state.avgResponseTau = sumResponseTau * 0.25
    state.avgEnergyTau = sumEnergyTau * 0.25
    state.avgResponseError = sumResponseError * 0.25
    state.avgReleaseRate = sumReleaseRate * 0.25
    state.maxDeformation = maxDef
    state.maxEnergyLive = maxEnergy
    state.maxDelayLive = maxDelay
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

    dt = clamp(dt, M.params.minDt, M.params.maxDt)
    updateDebugGate(dt)
    resetLinkFlags()
    state.storeOnly = false

    car = car or safeGetCar()

    if not car then
        state.status = "NO CAR"
        state.wheelsValid = false
        state.storeOnly = true
        for i = 0, 3 do
            updateWheelFromInputs(i, nil, dt)
        end
        updateAverages()
        exportState()
        return
    end

    local wheels = safeField(car, "wheels", nil)
    if not wheels then
        state.status = "NO WHEELS"
        state.wheelsValid = false
        state.storeOnly = true
        for i = 0, 3 do
            updateWheelFromInputs(i, nil, dt)
        end
        updateAverages()
        exportState()
        return
    end

    state.status = "RUNNING"
    state.wheelsValid = true

    for i = 0, 3 do
        local wheel = getWheel(car, i)
        if wheel then
            updateWheelFromInputs(i, wheel, dt)
        else
            decayWheel(i, dt, PHASE.UNLOADED)
        end
        exportWheel(i, state.wheels[i])
    end

    updateAverages()
    exportGlobal()
end

function M.getState(index)
    if index == nil then return state end
    return state.wheels[index]
end

function M.getDeformation(index)
    local ws = state.wheels[index]
    return ws and ws.deformation or 0.0
end

function M.getSidewallEnergy(index)
    local ws = state.wheels[index]
    return ws and ws.sidewallEnergy or 0.0
end

function M.getSupport(index)
    local ws = state.wheels[index]
    return ws and ws.support or 1.0
end

function M.getGripGate(index)
    local ws = state.wheels[index]
    return ws and ws.gripGate or 1.0
end

function M.getRecoveryBias(index)
    local ws = state.wheels[index]
    return ws and ws.recoveryBias or 0.0
end

function M.getPhase(index)
    local ws = state.wheels[index]
    return ws and ws.phaseText or "UNKNOWN"
end

function M.getResponseTau(index)
    local ws = state.wheels[index]
    return ws and ws.responseTau or 0.0
end

function M.getEnergyReservoir(index)
    local ws = state.wheels[index]
    return ws and ws.energyReservoir or 0.0
end

function M.getResponseError(index)
    local ws = state.wheels[index]
    return ws and ws.responseError or 0.0
end

function M.debugStr(index)
    local ws = state.wheels[index or 0] or state.wheels[0]

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "Phase %s / Def %.3f / E %.3f / Sup %.3f / Gate %.3f\n" ..
        "Lat %.3f Long %.3f BrL %.3f BrG %.3f\n" ..
        "Demand %.3f / RespTau %.3f / EnergyTau %.3f / Err %.3f / Rel %.3f\n" ..
        "Reservoir %.3f / Target %.3f / Head %.3f / Sat %s\n" ..
        "Vert %.3f Hys %.3f Delay %.3f Return %.3f\n" ..
        "SA %.3f SR %.3f Load %.0f dLoad %.0f CQ %.2f CS %.2f\n" ..
        "Road %.2f Path %.2f Leak %.2f Thermal %.2f\n" ..
        "Links CQ:%s Mem:%s DH:%s CS:%s DW:%s LP:%s RII:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",
        tostring(ws.phaseText or "UNKNOWN"),
        ws.deformation or 0.0,
        ws.sidewallEnergy or 0.0,
        ws.support or 1.0,
        ws.gripGate or 1.0,
        ws.carcassLat or 0.0,
        ws.carcassLong or 0.0,
        ws.bristleLat or 0.0,
        ws.bristleLong or 0.0,
        ws.slipDemand or 0.0,
        ws.responseTau or 0.0,
        ws.energyTau or 0.0,
        ws.responseError or 0.0,
        ws.releaseRate or 0.0,
        ws.energyReservoir or 0.0,
        ws.energyTarget or 0.0,
        ws.energyHeadroom or 0.0,
        tostring(ws.energySaturated == true),
        ws.verticalNorm or 0.0,
        ws.hysteresis or 0.0,
        ws.contactDelay or 0.0,
        ws.returnForce or 0.0,
        ws.slipAngle or 0.0,
        ws.slipRatio or 0.0,
        ws.load or 0.0,
        ws.loadVelocity or 0.0,
        ws.contactQuality or 1.0,
        ws.combinedSlip or 0.0,
        ws.roadShock or 0.0,
        ws.loadPathLoss or 0.0,
        ws.forceLeak or 0.0,
        ws.thermalStress or 0.0,
        state.contactLinked and "OK" or "NIL",
        state.memoryLinked and "OK" or "NIL",
        state.damperLinked and "OK" or "NIL",
        state.complianceLinked and "OK" or "NIL",
        state.windupLinked and "OK" or "NIL",
        state.loadPathLinked and "OK" or "NIL",
        state.roadLinked and "OK" or "NIL"
    )
end

return M
