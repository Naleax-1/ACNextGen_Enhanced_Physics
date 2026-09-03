---@diagnostic disable: undefined-global

--============================================================
-- yaw_moment_budget.lua
-- ACNextGen V1.1.5 stable
-- Yaw Moment Budget / Rotation State Interpreter
--
-- Root-state interpreter only. It does not write directly to AC physics.
-- It reads tire contact, memory, load, driveline, LSD, compliance,
-- recovery and chassis signals, then exports yaw budget diagnostics.
--============================================================

local M = {}

local WHEEL_NAMES = {
    [0] = "FL",
    [1] = "FR",
    [2] = "RL",
    [3] = "RR",
}

M.params = {
    lowSpeedKmh = 5.0,
    speedFullKmh = 90.0,

    loadRef = 3200.0,
    yawRateRef = 1.35,
    driveTorqueRef = 650.0,
    shaftTwistRef = 1.0,

    contactWeight = 0.42,
    gripWeight = 0.22,
    loadWeight = 0.18,
    memoryWeight = 0.10,
    slipWeight = 0.08,

    frontSlipUnderGain = 0.52,
    responseUnderGain = 0.48,
    rearSlipOverGain = 0.50,
    yawOverGain = 0.30,
    rearLossOverGain = 0.20,

    driveOverGain = 0.16,
    lsdOverGain = 0.12,
    complianceYawGain = 0.20,

    recoveryAuthorityGain = 0.08,
    carcassSupportAuthorityGain = 0.06,
    trustAuthorityGain = 0.10,
    dirtyAuthorityLoss = 0.14,
    snapAuthorityLoss = 0.18,

    biteOverGain = 0.18,
    dirtyOverGain = 0.12,
    recoveryOverTrimGain = 0.10,

    slipRecoveryPhaseGain = 0.24,
    slipGripReturnGain = 0.18,
    recontactYawGain = 0.16,
    dirtyRecoveryLoss = 0.22,

    slipSnapGain = 0.46,
    biteSnapGain = 0.34,
    dirtySnapGain = 0.24,
    historySpinGain = 0.16,

    thermalYawLossGain = 0.08,
    abrasionYawLossGain = 0.10,

    recoveryMemoryGain = 0.40,
    recoveryContactGain = 0.38,
    recoveryPhaseGain = 0.22,

    spinYawGain = 0.36,
    spinRearSlipGain = 0.34,
    spinRearLossGain = 0.20,
    spinOppositeGain = 0.10,

    roadYawGain = 0.08,
    forceYawGain = 0.10,
    thermalGripLossGain = 0.06,
    ultraChassisToeGain = 0.08,
    ultraChassisCamberGain = 0.04,

    tauFast = 0.050,
    tauSlow = 0.160,
    tauRisk = 0.090,

    minDt = 0.00005,
    maxDt = 0.050,

    debugStoreInterval = 0.25,
}

M.state = {
    status = "INIT",
    updateCount = 0,
    wheelsValid = false,
    storeOnly = false,

    speedKmh = 0.0,
    steer = 0.0,
    gas = 0.0,
    brake = 0.0,
    handbrake = 0.0,
    yawRate = 0.0,

    frontContact = 1.0,
    rearContact = 1.0,
    frontLoad = 0.0,
    rearLoad = 0.0,
    frontSlip = 0.0,
    rearSlip = 0.0,
    frontMemory = 0.0,
    rearMemory = 0.0,
    frontAuthority = 1.0,
    rearAuthority = 1.0,

    steerDemand = 0.0,
    yawResponse = 0.0,
    yawBudget = 0.0,
    rotationIntent = 0.0,
    understeerEnergy = 0.0,
    oversteerEnergy = 0.0,
    recoveryPhase = 0.0,
    spinRisk = 0.0,
    balance = 0.0,

    driveYaw = 0.0,
    lsdYaw = 0.0,
    complianceYaw = 0.0,
    loadYaw = 0.0,
    roadYaw = 0.0,
    forceYaw = 0.0,

    recoveryYaw = 0.0,
    biteYaw = 0.0,
    dirtyYaw = 0.0,
    recontactYaw = 0.0,
    slipRecoveryRear = 0.0,
    slipSnapRear = 0.0,

    phaseId = 0,
    phaseText = "INIT",

    avgAuthority = 1.0,
    authorityBalance = 0.0,
    maxWheelSlip = 0.0,
    activeCount = 0,

    contactLinked = false,
    tireMemoryLinked = false,
    complianceLinked = false,
    drivetrainLinked = false,
    lsdLinked = false,
    loadLinked = false,
    slipRecoveryLinked = false,
    carcassLinked = false,
    tireForceLinked = false,
    roadLinked = false,
    thermalLinked = false,
    ultraChassisLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,

    wheels = {},
}

for i = 0, 3 do
    M.state.wheels[i] = {
        load = M.params.loadRef,
        contact = 1.0,
        rawContact = 1.0,
        contactTrust = 1.0,
        stability = 1.0,
        slipAngle = 0.0,
        slipRatio = 0.0,
        combinedSlip = 0.0,
        memory = 0.0,
        grip = 1.0,
        memoryPhaseId = 0,
        memoryPhase = "NIL",

        thermalMemory = 0.0,
        abrasionMemory = 0.0,
        historyMemory = 0.0,
        recoveryScalar = 1.0,
        recontactShock = 0.0,

        recoveryRate = 0.0,
        gripReturn = 1.0,
        snapRisk = 0.0,
        slideMemory = 0.0,
        recontactSharpness = 0.0,
        bite = 0.0,
        dirtyReturn = 0.0,
        recoveryTrust = 1.0,

        carcassSupport = 1.0,
        gripGate = 1.0,
        carcassHysteresis = 0.0,
        carcassRecoveryBias = 0.0,
        bristleLat = 0.0,
        bristleLong = 0.0,

        complianceEnergy = 0.0,
        virtualToe = 0.0,
        virtualCamber = 0.0,
        ultraToe = 0.0,
        ultraCamber = 0.0,
        authority = 1.0,

        tireForceLat = 0.0,
        tireForceLong = 0.0,
        tireForceCombined = 0.0,
        tireForceRoadInput = 0.0,
        roadSeverity = 0.0,
        thermalMuScale = 1.0,

        active = false,
    }

    M.state[i] = M.state.wheels[i]
end

M.debug = M.state

--============================================================
-- Utility
--============================================================

local function safeNumber(value, defaultValue)
    local n = tonumber(value)
    if n == nil or n ~= n or n == math.huge or n == -math.huge then
        if defaultValue == nil then
            return nil
        end
        return defaultValue
    end
    return n
end

local function numberOr(value, defaultValue)
    local n = safeNumber(value, nil)
    if n == nil then
        return defaultValue or 0.0
    end
    return n
end

local function clamp(v, minValue, maxValue)
    v = numberOr(v, minValue or 0.0)
    if v < minValue then
        return minValue
    end
    if v > maxValue then
        return maxValue
    end
    return v
end

local function abs(v)
    return math.abs(numberOr(v, 0.0))
end

local function sign(v)
    v = numberOr(v, 0.0)
    if v > 0.0001 then
        return 1.0
    end
    if v < -0.0001 then
        return -1.0
    end
    return 0.0
end

local function lowPass(current, target, tau, dt)
    current = numberOr(current, 0.0)
    target = numberOr(target, 0.0)
    tau = numberOr(tau, 0.0)
    dt = numberOr(dt, 0.0)
    if tau <= 0.0 then
        return target
    end
    return current + (target - current) * clamp(dt / math.max(tau + dt, 0.0001), 0.0, 1.0)
end

local function smoothstep(edge0, edge1, x)
    if edge0 == edge1 then
        return x >= edge1 and 1.0 or 0.0
    end
    local t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
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
    return numberOr(value, defaultValue or 0.0)
end

local function safeLoadAlt(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return numberOr(value, defaultValue or 0.0), keys[i]
        end
    end
    return defaultValue or 0.0, nil
end

local function safeLoadText(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return tostring(value), keys[i]
        end
    end
    return defaultValue or "", nil
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
        local x = numberOr(v.x, 0.0)
        local y = numberOr(v.y, 0.0)
        local z = numberOr(v.z, 0.0)
        return math.sqrt(x * x + y * y + z * z)
    end)
    if ok then
        return numberOr(result, 0.0)
    end
    return 0.0
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

local function hasWheels(car)
    if not car then
        return false
    end
    local ok, wheels = pcall(function()
        return car.wheels
    end)
    return ok and wheels ~= nil
end

local function getWheel(car, index)
    if not hasWheels(car) then
        return nil
    end
    local ok, wheel = pcall(function()
        return car.wheels[index]
    end)
    if ok and wheel ~= nil then
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

local function getSpeedKmh(car)
    if car then
        local speedKmh = safeField(car, "speedKmh", nil)
        if speedKmh ~= nil then
            return numberOr(speedKmh, 0.0)
        end

        local speed = nil
        if speed ~= nil then
            local n = numberOr(speed, 0.0)
            if math.abs(n) < 120.0 then
                return n * 3.6
            end
            return n
        end

        local velocity = safeField(car, "velocity", nil)
        if velocity then
            return vecLength(velocity) * 3.6
        end

        local localVelocity = safeField(car, "localVelocity", nil)
        if localVelocity then
            return vecLength(localVelocity) * 3.6
        end
    end

    return safeLoadAlt(M.state.speedKmh or 0.0,
        "ngp_yaw_speed_kmh",
        "ngp_speed_kmh",
        "ngp_observer_speed_kmh",
        "ngp_car_speed_kmh")
end

local function getYawRate(car)
    if car then
        local av = safeField(car, "localAngularVelocity", nil)
        if av then
            return numberOr(safeField(av, "y", 0.0), 0.0)
        end

        local yaw = nil
        if yaw ~= nil then
            return numberOr(yaw, 0.0)
        end
    end

    return safeLoadAlt(M.state.yawRate or 0.0,
        "ngp_yaw_rate",
        "ngp_observer_yaw_rate",
        "ngp_virtual_yaw_raw",
        "ngp_virtual_yaw")
end

local function getInputValue(car, carField, defaultValue, ...)
    local stored = safeLoadRaw(select(1, ...))
    if stored ~= nil then
        return numberOr(stored, defaultValue or 0.0)
    end

    local keys = { ... }
    for i = 2, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return numberOr(value, defaultValue or 0.0)
        end
    end

    if car then
        return numberOr(safeField(car, carField, defaultValue or 0.0), defaultValue or 0.0)
    end

    return defaultValue or 0.0
end

local function dynamicLoadToAbsolute(value)
    if value == nil then
        return nil
    end
    local v = numberOr(value, 0.0)
    if math.abs(v) < M.params.loadRef * 0.85 then
        return M.params.loadRef + v
    end
    return v
end

local function getWheelLoad(wheel, index)
    local value, key = safeLoadAlt(nil,
        "ngp_contact_load_" .. index,
        "ngp_tire_state_load_" .. index,
        "ngp_tire_load_" .. index,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index,
        "ngp_load_path_abs_load_" .. index,
        "ngp_lp_abs_load_" .. index)

    if key ~= nil then
        if key:find("contact") then
            M.state.contactLinked = true
        else
            M.state.loadLinked = true
        end
        return clamp(math.abs(numberOr(value, M.params.loadRef)), 0.0, M.params.loadRef * 3.5)
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        M.state.loadLinked = true
        return clamp(math.abs(dynamicLoadToAbsolute(dlt)), 0.0, M.params.loadRef * 3.5)
    end

    if wheel then
        local raw = safeField(wheel, "load", nil)
        if raw == nil then
            raw = safeField(wheel, "loadK", nil)
        end
        if raw == nil then
            raw = nil
        end
        if raw == nil then
            raw = nil
        end
        if raw == nil then
            raw = nil
        end
        if raw ~= nil then
            return clamp(math.abs(numberOr(raw, M.params.loadRef)), 0.0, M.params.loadRef * 3.5)
        end
    end

    return M.params.loadRef
end

local function readRecoveryInputs(index, ws)
    local rec = safeLoadRaw("ngp_slip_recovery_rate_" .. index)
    if rec == nil then rec = safeLoadRaw("ngp_recovery_rate_" .. index) end
    if rec == nil then rec = safeLoadRaw("ngp_sr_recovery_" .. index) end

    local gripReturn = safeLoadRaw("ngp_slip_grip_return_" .. index)
    if gripReturn == nil then gripReturn = safeLoadRaw("ngp_grip_return_" .. index) end
    if gripReturn == nil then gripReturn = safeLoadRaw("ngp_sr_grip_return_" .. index) end

    local snap = safeLoadRaw("ngp_slip_snap_risk_" .. index)
    if snap == nil then snap = safeLoadRaw("ngp_snap_risk_" .. index) end
    if snap == nil then snap = safeLoadRaw("ngp_sr_snap_risk_" .. index) end

    local slideMemory = safeLoadRaw("ngp_slip_slide_memory_" .. index)
    if slideMemory == nil then slideMemory = safeLoadRaw("ngp_slide_memory_" .. index) end

    local recontact = safeLoadRaw("ngp_slip_recontact_" .. index)
    if recontact == nil then recontact = safeLoadRaw("ngp_recontact_sharpness_" .. index) end
    if recontact == nil then recontact = safeLoadRaw("ngp_sr_recontact_" .. index) end

    local bite = safeLoadRaw("ngp_slip_bite_" .. index)
    if bite == nil then bite = safeLoadRaw("ngp_recovery_bite_" .. index) end
    if bite == nil then bite = safeLoadRaw("ngp_sr_bite_" .. index) end

    local dirty = safeLoadRaw("ngp_slip_dirty_return_" .. index)
    if dirty == nil then dirty = safeLoadRaw("ngp_dirty_return_" .. index) end
    if dirty == nil then dirty = safeLoadRaw("ngp_sr_dirty_return_" .. index) end

    local trust = safeLoadRaw("ngp_slip_contact_trust_" .. index)
    if trust == nil then trust = safeLoadRaw("ngp_recovery_trust_" .. index) end
    if trust == nil then trust = safeLoadRaw("ngp_contact_trust_" .. index) end

    if rec ~= nil or gripReturn ~= nil or snap ~= nil or slideMemory ~= nil
        or recontact ~= nil or bite ~= nil or dirty ~= nil or trust ~= nil then
        M.state.slipRecoveryLinked = true
    end

    ws.recoveryRate = clamp(numberOr(rec, 0.0), 0.0, 1.2)
    ws.gripReturn = clamp(numberOr(gripReturn, 1.0), 0.0, 1.2)
    ws.snapRisk = clamp(numberOr(snap, 0.0), 0.0, 1.2)
    ws.slideMemory = clamp(numberOr(slideMemory, 0.0), 0.0, 1.2)
    ws.recontactSharpness = clamp(numberOr(recontact, 0.0), 0.0, 1.2)
    ws.bite = clamp(numberOr(bite, 0.0), 0.0, 1.2)
    ws.dirtyReturn = clamp(numberOr(dirty, 0.0), 0.0, 1.2)
    ws.recoveryTrust = clamp(numberOr(trust, ws.contact or 1.0), 0.0, 1.2)
end

local function readMemoryExtended(index, ws)
    local thermal, tk = safeLoadAlt(0.0,
        "ngp_memory_thermal_" .. index,
        "ngp_tire_memory_thermal_" .. index,
        "ngp_slip_thermal_memory_" .. index)

    local abrasion = safeLoadAlt(0.0,
        "ngp_memory_abrasion_" .. index,
        "ngp_tire_memory_abrasion_" .. index,
        "ngp_slip_abrasion_memory_" .. index)

    local history = safeLoadAlt(0.0,
        "ngp_memory_history_" .. index,
        "ngp_tire_memory_history_" .. index,
        "ngp_slip_history_memory_" .. index)

    local recoveryScalar = safeLoadAlt(1.0,
        "ngp_memory_recovery_scalar_" .. index,
        "ngp_tire_memory_recovery_scalar_" .. index,
        "ngp_slip_recovery_scalar_" .. index)

    local recontactShock = safeLoadAlt(0.0,
        "ngp_memory_recontact_shock_" .. index,
        "ngp_slip_recontact_shock_" .. index)

    if tk ~= nil then
        M.state.tireMemoryLinked = true
    end

    ws.thermalMemory = clamp(thermal, 0.0, 1.2)
    ws.abrasionMemory = clamp(abrasion, 0.0, 1.2)
    ws.historyMemory = clamp(history, 0.0, 1.2)
    ws.recoveryScalar = clamp(recoveryScalar, 0.0, 1.2)
    ws.recontactShock = clamp(recontactShock, 0.0, 1.2)
end

local function readCarcassExtended(index, ws)
    local support = safeLoadRaw("ngp_carcass_support_" .. index)
    if support == nil then support = safeLoadRaw("ngp_contact_carcass_support_" .. index) end
    if support == nil then support = safeLoadRaw("ngp_tc_support_" .. index) end

    local gate = safeLoadRaw("ngp_carcass_grip_gate_" .. index)
    if gate == nil then gate = safeLoadRaw("ngp_contact_grip_gate_" .. index) end

    local hyst = safeLoadRaw("ngp_carcass_hysteresis_" .. index)
    if hyst == nil then hyst = safeLoadRaw("ngp_contact_hysteresis_" .. index) end

    local recoveryBias = safeLoadRaw("ngp_carcass_recovery_bias_" .. index)
    if recoveryBias == nil then recoveryBias = safeLoadRaw("ngp_contact_recovery_bias_" .. index) end

    local bristleLat = safeLoadRaw("ngp_carcass_bristle_lat_" .. index)
    local bristleLong = safeLoadRaw("ngp_carcass_bristle_long_" .. index)

    if support ~= nil or gate ~= nil or hyst ~= nil or recoveryBias ~= nil or bristleLat ~= nil or bristleLong ~= nil then
        M.state.carcassLinked = true
    end

    ws.carcassSupport = clamp(numberOr(support, 1.0), 0.0, 1.2)
    ws.gripGate = clamp(numberOr(gate, 1.0), 0.0, 1.2)
    ws.carcassHysteresis = clamp(numberOr(hyst, 0.0), 0.0, 1.2)
    ws.carcassRecoveryBias = clamp(numberOr(recoveryBias, 0.0), 0.0, 1.2)
    ws.bristleLat = clamp(numberOr(bristleLat, 0.0), 0.0, 1.5)
    ws.bristleLong = clamp(numberOr(bristleLong, 0.0), 0.0, 1.5)
end

local function readForceAndRoadInputs(index, ws)
    local lat, latKey = safeLoadAlt(0.0,
        "ngp_tire_force_lat_" .. index,
        "ngp_tf_lat_" .. index,
        "ngp_tforce_lat_" .. index)

    local long = safeLoadAlt(0.0,
        "ngp_tire_force_long_" .. index,
        "ngp_tf_long_" .. index,
        "ngp_tforce_long_" .. index)

    local combined = safeLoadAlt(0.0,
        "ngp_tire_force_combined_" .. index,
        "ngp_tf_combined_" .. index,
        "ngp_tforce_combined_" .. index)

    local road, roadKey = safeLoadAlt(0.0,
        "ngp_tire_force_road_input_" .. index,
        "ngp_tf_road_input_" .. index,
        "ngp_road_wheel_severity_" .. index,
        "ngp_rii_wheel_severity_" .. index,
        "ngp_load_path_road_input_" .. index)

    if latKey ~= nil then
        M.state.tireForceLinked = true
    end
    if roadKey ~= nil then
        M.state.roadLinked = true
    end

    ws.tireForceLat = lat
    ws.tireForceLong = long
    ws.tireForceCombined = math.max(abs(combined), math.sqrt(lat * lat + long * long))
    ws.tireForceRoadInput = clamp(abs(road), 0.0, 2.0)
    ws.roadSeverity = ws.tireForceRoadInput
end

local function readThermalAndGeometry(index, ws)
    local muScale, muKey = safeLoadAlt(1.0,
        "ngp_rf2_mu_scale_" .. index,
        "ngp_ttb_mu_scale_" .. index,
        "ngp_thermal_mu_scale_" .. index)

    if muKey ~= nil then
        M.state.thermalLinked = true
    end

    local ucToe, toeKey = safeLoadAlt(0.0,
        "ngp_uc_toe_" .. index,
        "ngp_ultra_chassis_toe_" .. index,
        "ngp_virtual_toe_" .. index,
        "ngp_compliance_virtual_toe_" .. index)

    local ucCamber = safeLoadAlt(0.0,
        "ngp_uc_camber_" .. index,
        "ngp_ultra_chassis_camber_" .. index,
        "ngp_virtual_camber_" .. index,
        "ngp_compliance_virtual_camber_" .. index)

    if toeKey ~= nil then
        M.state.ultraChassisLinked = true
    end

    ws.thermalMuScale = clamp(muScale, 0.35, 1.15)
    ws.ultraToe = ucToe
    ws.ultraCamber = ucCamber
    ws.virtualToe = ucToe
    ws.virtualCamber = ucCamber
end

local function readWheelInputs(index, wheel)
    local ws = M.state.wheels[index]

    local contact = safeLoadRaw("ngp_contact_quality_" .. index)
    if contact == nil then contact = safeLoadRaw("ngp_tire_contact_quality_" .. index) end
    if contact == nil then contact = safeLoadRaw("ngp_tcr_quality_" .. index) end
    if contact == nil then contact = safeLoadRaw("ngp_tc_contact_" .. index) end

    if contact ~= nil then
        M.state.contactLinked = true
    end

    ws.contact = clamp(numberOr(contact, 1.0), 0.0, 1.2)
    ws.rawContact = clamp(safeLoadAlt(ws.contact, "ngp_contact_raw_" .. index, "ngp_contact_quality_raw_" .. index), 0.0, 1.2)
    ws.stability = clamp(safeLoadAlt(1.0, "ngp_contact_stability_score_" .. index, "ngp_contact_stability_" .. index), 0.0, 1.0)
    ws.contactTrust = clamp(safeLoadAlt(ws.contact, "ngp_contact_trust_" .. index, "ngp_tcr_trust_" .. index), 0.0, 1.2)

    local slipAngle = safeLoadRaw("ngp_contact_slip_angle_" .. index)
    if slipAngle == nil then slipAngle = safeLoadRaw("ngp_tire_slip_angle_" .. index) end
    if slipAngle == nil then slipAngle = safeLoadRaw("ngp_slip_angle_" .. index) end
    if slipAngle == nil then slipAngle = safeLoadRaw("ngp_filtered_slip_angle_" .. index) end
    if slipAngle == nil and wheel then slipAngle = safeField(wheel, "slipAngle", 0.0) end

    local slipRatio = safeLoadRaw("ngp_contact_slip_ratio_" .. index)
    if slipRatio == nil then slipRatio = safeLoadRaw("ngp_tire_slip_ratio_" .. index) end
    if slipRatio == nil then slipRatio = safeLoadRaw("ngp_slip_ratio_" .. index) end
    if slipRatio == nil then slipRatio = safeLoadRaw("ngp_filtered_slip_ratio_" .. index) end
    if slipRatio == nil and wheel then slipRatio = safeField(wheel, "slipRatio", 0.0) end

    ws.slipAngle = abs(slipAngle)
    ws.slipRatio = abs(slipRatio)

    local combined = safeLoadRaw("ngp_contact_combined_slip_" .. index)
    if combined == nil then combined = safeLoadRaw("ngp_tire_combined_slip_" .. index) end
    if combined == nil then combined = safeLoadRaw("ngp_tdyn_combined_slip_" .. index) end
    if combined == nil then combined = safeLoadRaw("ngp_tire_force_combined_slip_" .. index) end

    if combined ~= nil then
        ws.combinedSlip = clamp(numberOr(combined, 0.0), 0.0, 4.0)
    else
        local saNorm = ws.slipAngle / 0.22
        local srNorm = ws.slipRatio / 0.35
        ws.combinedSlip = clamp(math.sqrt(saNorm * saNorm + srNorm * srNorm), 0.0, 4.0)
    end

    ws.load = getWheelLoad(wheel, index)

    local memory = safeLoadRaw("ngp_tire_memory_" .. index)
    if memory == nil then memory = safeLoadRaw("ngp_rubber_memory_" .. index) end
    if memory == nil then memory = safeLoadRaw("ngp_memory_" .. index) end

    if memory ~= nil then
        M.state.tireMemoryLinked = true
    end

    ws.memory = clamp(numberOr(memory, 0.0), 0.0, 1.0)
    ws.grip = clamp(safeLoadAlt(1.0,
        "ngp_memory_grip_" .. index,
        "ngp_tire_memory_grip_" .. index,
        "ngp_tire_grip_" .. index,
        "ngp_tdyn_grip_" .. index), 0.0, 1.2)
    ws.memoryPhaseId = safeLoadAlt(0.0, "ngp_tire_memory_phase_id_" .. index, "ngp_tyre_memory_phase_id_" .. index)
    ws.memoryPhase = safeLoadText("NIL", "ngp_tire_memory_phase_" .. index, "ngp_tyre_memory_phase_" .. index)

    local compliance = safeLoadRaw("ngp_compliance_energy_" .. index)
    if compliance == nil then compliance = safeLoadRaw("ngp_cs_energy_" .. index) end
    if compliance == nil then compliance = safeLoadRaw("ngp_tire_compliance_" .. index) end
    if compliance ~= nil then
        M.state.complianceLinked = true
    end

    ws.complianceEnergy = clamp(numberOr(compliance, 0.0), 0.0, 1.0)

    readRecoveryInputs(index, ws)
    readMemoryExtended(index, ws)
    readCarcassExtended(index, ws)
    readForceAndRoadInputs(index, ws)
    readThermalAndGeometry(index, ws)

    ws.active = wheel ~= nil or contact ~= nil or memory ~= nil or combined ~= nil
end

--============================================================
-- Calculation
--============================================================

local function loadScore(load)
    local norm = numberOr(load, M.params.loadRef) / math.max(M.params.loadRef, 1.0)
    local score = smoothstep(0.10, 0.80, norm)
    local overload = smoothstep(1.85, 3.00, norm)
    return clamp(score * (1.0 - overload * 0.12), 0.0, 1.0)
end

local function slipScore(combinedSlip)
    return clamp(1.0 - smoothstep(0.60, 1.90, combinedSlip), 0.0, 1.0)
end

local function calculateWheelAuthority(ws)
    local contactPart = clamp(ws.contact, 0.0, 1.0)
    local trustPart = clamp(ws.contactTrust or ws.contact or 1.0, 0.0, 1.15)
    local gripPart = clamp((ws.grip or 1.0) * (ws.thermalMuScale or 1.0), 0.0, 1.05)
    local loadPart = loadScore(ws.load)
    local memoryLoad = clamp(
        (ws.memory or 0.0) * 0.50 +
        (ws.thermalMemory or 0.0) * 0.18 +
        (ws.abrasionMemory or 0.0) * 0.16 +
        (ws.historyMemory or 0.0) * 0.16,
        0.0,
        1.25)
    local memoryPart = 1.0 - clamp(memoryLoad, 0.0, 1.0) * 0.55
    local slipPart = slipScore(ws.combinedSlip)

    local recoveryHelp = clamp(ws.gripReturn or 1.0, 0.0, 1.15) * M.params.recoveryAuthorityGain
    local supportHelp = clamp(ws.carcassSupport or 1.0, 0.0, 1.15) * M.params.carcassSupportAuthorityGain
    local trustHelp = trustPart * M.params.trustAuthorityGain

    local dirtyLoss = clamp(ws.dirtyReturn or 0.0, 0.0, 1.0) * M.params.dirtyAuthorityLoss
    local snapLoss = clamp(ws.snapRisk or 0.0, 0.0, 1.0) * M.params.snapAuthorityLoss
    local thermalLoss = math.max(1.0 - (ws.thermalMuScale or 1.0), 0.0) * M.params.thermalGripLossGain
    local gate = clamp(ws.gripGate or 1.0, 0.0, 1.05)

    local authority =
        contactPart * M.params.contactWeight +
        gripPart * M.params.gripWeight +
        loadPart * M.params.loadWeight +
        memoryPart * M.params.memoryWeight +
        slipPart * M.params.slipWeight +
        recoveryHelp +
        supportHelp +
        trustHelp -
        dirtyLoss -
        snapLoss -
        thermalLoss

    authority = authority * (0.82 + 0.18 * gate)
    return clamp(authority, 0.0, 1.15)
end

local function classifyPhase(st)
    local speed = st.speedKmh or 0.0

    if speed < M.params.lowSpeedKmh then
        return 0, "LOW_SPEED"
    end
    if st.spinRisk > 0.70 or st.slipSnapRear > 0.62 then
        return 7, "SPIN_RISK"
    end
    if st.recontactYaw > 0.48 and st.biteYaw > 0.36 then
        return 9, "RECONTACT_YAW"
    end
    if st.oversteerEnergy > 0.48 and (st.brake > 0.18 or st.handbrake > 0.10) then
        return 6, "ENTRY_ROTATION"
    end
    if st.oversteerEnergy > 0.48 and st.gas > 0.18 then
        return 5, "POWER_OVERSTEER"
    end
    if st.oversteerEnergy > 0.45 then
        return 4, "OVERSTEER"
    end
    if st.understeerEnergy > 0.45 then
        return 3, "UNDERSTEER"
    end
    if st.recoveryPhase > 0.45 then
        return 2, "RECOVERING"
    end
    if st.rotationIntent > 0.36 then
        return 1, "ROTATING"
    end
    return 8, "NEUTRAL"
end

local function updateBudget(dt)
    local st = M.state
    local p = M.params

    local ws0 = st.wheels[0]
    local ws1 = st.wheels[1]
    local ws2 = st.wheels[2]
    local ws3 = st.wheels[3]

    local authoritySum = 0.0
    local activeCount = 0
    local maxWheelSlip = 0.0

    for i = 0, 3 do
        st.wheels[i].authority = calculateWheelAuthority(st.wheels[i])
        authoritySum = authoritySum + st.wheels[i].authority
        maxWheelSlip = math.max(maxWheelSlip, st.wheels[i].combinedSlip or 0.0)
        if st.wheels[i].active then
            activeCount = activeCount + 1
        end
    end

    local frontContact = (ws0.contact + ws1.contact) * 0.5
    local rearContact = (ws2.contact + ws3.contact) * 0.5
    local frontLoad = (ws0.load + ws1.load) * 0.5
    local rearLoad = (ws2.load + ws3.load) * 0.5
    local frontSlip = (ws0.combinedSlip + ws1.combinedSlip) * 0.5
    local rearSlip = (ws2.combinedSlip + ws3.combinedSlip) * 0.5
    local frontMemory = (ws0.memory + ws1.memory) * 0.5
    local rearMemory = (ws2.memory + ws3.memory) * 0.5
    local frontAuthority = (ws0.authority + ws1.authority) * 0.5
    local rearAuthority = (ws2.authority + ws3.authority) * 0.5

    local frontRecovery = (ws0.recoveryRate + ws1.recoveryRate) * 0.5
    local rearRecovery = (ws2.recoveryRate + ws3.recoveryRate) * 0.5
    local frontGripReturn = (ws0.gripReturn + ws1.gripReturn) * 0.5
    local rearGripReturn = (ws2.gripReturn + ws3.gripReturn) * 0.5
    local frontSnap = (ws0.snapRisk + ws1.snapRisk) * 0.5
    local rearSnap = (ws2.snapRisk + ws3.snapRisk) * 0.5
    local frontBite = (ws0.bite + ws1.bite) * 0.5
    local rearBite = (ws2.bite + ws3.bite) * 0.5
    local frontDirty = (ws0.dirtyReturn + ws1.dirtyReturn) * 0.5
    local rearDirty = (ws2.dirtyReturn + ws3.dirtyReturn) * 0.5
    local rearRecontact = (ws2.recontactSharpness + ws3.recontactSharpness) * 0.5
    local rearHistory = (ws2.historyMemory + ws3.historyMemory) * 0.5
    local rearThermal = (ws2.thermalMemory + ws3.thermalMemory) * 0.5
    local rearAbrasion = (ws2.abrasionMemory + ws3.abrasionMemory) * 0.5

    local speedFactor = clamp((st.speedKmh - p.lowSpeedKmh) / math.max(p.speedFullKmh - p.lowSpeedKmh, 1.0), 0.0, 1.0)
    local steerAbs = abs(st.steer)
    local yawAbs = abs(st.yawRate)

    local steerDemandRaw = st.steer * speedFactor
    local yawResponseRaw = clamp(st.yawRate / math.max(p.yawRateRef, 0.001), -2.0, 2.0)

    local desiredSign = sign(st.steer)
    if desiredSign == 0.0 then
        desiredSign = sign(st.yawRate)
    end

    local yawInDesired = yawResponseRaw * desiredSign
    local responseDeficit = math.max(0.0, abs(steerDemandRaw) - math.max(0.0, yawInDesired))
    local yawExcess = math.max(0.0, math.max(0.0, yawInDesired) - abs(steerDemandRaw))
    local oppositeYaw = yawResponseRaw * desiredSign < -0.12 and 1.0 or 0.0

    local frontSlipNorm = clamp(frontSlip / 1.65, 0.0, 1.4)
    local rearSlipNorm = clamp(rearSlip / 1.65, 0.0, 1.4)

    local frontLoss = clamp(1.0 - frontAuthority, 0.0, 1.0)
    local rearLoss = clamp(1.0 - rearAuthority, 0.0, 1.0)

    local driveTorque, driveKey = safeLoadAlt(0.0,
        "ngp_drive_torque",
        "ngp_drive_transmitted_torque",
        "ngp_drivetrain_torque",
        "ngp_dt_torque")
    if driveKey ~= nil then
        st.drivetrainLinked = true
    end

    local shaftTwist = safeLoadAlt(0.0,
        "ngp_shaft_twist",
        "ngp_driveline_twist",
        "ngp_windup_twist")
    local driveNorm = clamp(abs(driveTorque) / math.max(p.driveTorqueRef, 1.0), 0.0, 1.5)
    local shaftNorm = clamp(abs(shaftTwist) / math.max(p.shaftTwistRef, 0.001), 0.0, 1.0)

    local lsdLock, lsdKey = safeLoadAlt(0.0,
        "ngp_lsd_lock",
        "ngp_diff_lock")
    if lsdKey ~= nil then
        st.lsdLinked = true
    end
    if lsdLock > 1.5 then
        lsdLock = lsdLock / 100.0
    end
    lsdLock = clamp(lsdLock, 0.0, 1.0)

    local compFront = (ws0.complianceEnergy + ws1.complianceEnergy) * 0.5
    local compRear = (ws2.complianceEnergy + ws3.complianceEnergy) * 0.5
    local ultraToeYaw = (ws3.ultraToe - ws2.ultraToe) * p.ultraChassisToeGain
    local ultraCamberYaw = (abs(ws2.ultraCamber) - abs(ws3.ultraCamber)) * p.ultraChassisCamberGain
    local compYawRaw = clamp((compRear - compFront) * 0.60 + (ws3.virtualToe - ws2.virtualToe) * 4.0 + ultraToeYaw + ultraCamberYaw, -1.0, 1.0)

    local loadYawRaw = clamp(((ws1.load - ws0.load) + (ws3.load - ws2.load)) / math.max(p.loadRef * 2.0, 1.0), -1.0, 1.0)

    local roadRear = (ws2.tireForceRoadInput + ws3.tireForceRoadInput) * 0.5
    local roadFront = (ws0.tireForceRoadInput + ws1.tireForceRoadInput) * 0.5
    local roadYawRaw = clamp((roadRear - roadFront) * p.roadYawGain, -0.5, 0.5)

    local forceYawRaw = clamp(((ws2.tireForceCombined + ws3.tireForceCombined) - (ws0.tireForceCombined + ws1.tireForceCombined)) / 12000.0 * p.forceYawGain, -0.5, 0.5)

    local driveYawRaw = clamp((driveNorm * 0.75 + shaftNorm * 0.25) * rearSlipNorm * (0.35 + lsdLock * 0.65), 0.0, 1.0)
    local lsdYawRaw = clamp(lsdLock * rearSlipNorm * rearContact, 0.0, 1.0)

    local underRaw = clamp(
        responseDeficit * p.responseUnderGain +
        math.max(0.0, frontSlipNorm - rearSlipNorm * 0.55) * p.frontSlipUnderGain +
        frontLoss * 0.22 +
        math.max(0.0, frontDirty - rearDirty) * 0.06,
        0.0,
        1.0)

    local overRaw = clamp(
        math.max(0.0, rearSlipNorm - frontSlipNorm * 0.45) * p.rearSlipOverGain +
        yawExcess * p.yawOverGain +
        rearLoss * p.rearLossOverGain +
        driveYawRaw * p.driveOverGain +
        lsdYawRaw * p.lsdOverGain +
        abs(compYawRaw) * p.complianceYawGain +
        rearBite * p.biteOverGain +
        rearDirty * p.dirtyOverGain +
        math.max(0.0, roadRear - roadFront) * p.roadYawGain +
        forceYawRaw -
        rearRecovery * p.recoveryOverTrimGain,
        0.0,
        1.0)

    local memoryRecover = clamp(rearMemory * (rearContact - rearSlipNorm * 0.25), 0.0, 1.0)
    local contactRise = clamp(rearContact - (st.rearContact or rearContact), 0.0, 1.0)

    local phaseRecover = 0.0
    if ws2.memoryPhaseId == 4 or ws2.memoryPhaseId == 7 then
        phaseRecover = phaseRecover + 0.5
    end
    if ws3.memoryPhaseId == 4 or ws3.memoryPhaseId == 7 then
        phaseRecover = phaseRecover + 0.5
    end

    local recoveryRaw = clamp(
        memoryRecover * p.recoveryMemoryGain +
        contactRise * p.recoveryContactGain +
        phaseRecover * p.recoveryPhaseGain +
        rearRecovery * p.slipRecoveryPhaseGain +
        rearGripReturn * p.slipGripReturnGain +
        rearRecontact * p.recontactYawGain -
        rearDirty * p.dirtyRecoveryLoss,
        0.0,
        1.0)

    local spinRaw = clamp(
        clamp(yawAbs / math.max(p.yawRateRef * 1.25, 0.001), 0.0, 1.4) * p.spinYawGain +
        rearSlipNorm * p.spinRearSlipGain +
        rearLoss * p.spinRearLossGain +
        oppositeYaw * p.spinOppositeGain +
        rearSnap * p.slipSnapGain +
        rearBite * p.biteSnapGain +
        rearDirty * p.dirtySnapGain +
        rearHistory * p.historySpinGain,
        0.0,
        1.0)

    local rotationRaw = clamp(
        math.max(0.0, yawInDesired) * 0.34 +
        overRaw * 0.28 +
        driveYawRaw * 0.16 +
        lsdYawRaw * 0.10 +
        rearBite * 0.10 +
        rearRecovery * 0.08 +
        speedFactor * steerAbs * 0.12,
        0.0,
        1.0)

    local yawBudgetRaw = clamp(
        rotationRaw + overRaw * 0.55 + recoveryRaw * 0.16 - underRaw * 0.65 - spinRaw * 0.28 - rearDirty * 0.10,
        -1.0,
        1.0)

    local balanceRaw = clamp(overRaw - underRaw, -1.0, 1.0)

    st.frontContact = lowPass(st.frontContact, frontContact, p.tauFast, dt)
    st.rearContact = lowPass(st.rearContact, rearContact, p.tauFast, dt)
    st.frontLoad = lowPass(st.frontLoad, frontLoad, p.tauFast, dt)
    st.rearLoad = lowPass(st.rearLoad, rearLoad, p.tauFast, dt)
    st.frontSlip = lowPass(st.frontSlip, frontSlip, p.tauFast, dt)
    st.rearSlip = lowPass(st.rearSlip, rearSlip, p.tauFast, dt)
    st.frontMemory = lowPass(st.frontMemory, frontMemory, p.tauSlow, dt)
    st.rearMemory = lowPass(st.rearMemory, rearMemory, p.tauSlow, dt)
    st.frontAuthority = lowPass(st.frontAuthority, frontAuthority, p.tauFast, dt)
    st.rearAuthority = lowPass(st.rearAuthority, rearAuthority, p.tauFast, dt)

    st.steerDemand = lowPass(st.steerDemand, steerDemandRaw, p.tauFast, dt)
    st.yawResponse = lowPass(st.yawResponse, yawResponseRaw, p.tauFast, dt)
    st.understeerEnergy = lowPass(st.understeerEnergy, underRaw, p.tauSlow, dt)
    st.oversteerEnergy = lowPass(st.oversteerEnergy, overRaw, p.tauSlow, dt)
    st.recoveryPhase = lowPass(st.recoveryPhase, recoveryRaw, p.tauRisk, dt)
    st.spinRisk = lowPass(st.spinRisk, spinRaw, p.tauRisk, dt)
    st.rotationIntent = lowPass(st.rotationIntent, rotationRaw, p.tauFast, dt)
    st.yawBudget = lowPass(st.yawBudget, yawBudgetRaw, p.tauFast, dt)
    st.balance = lowPass(st.balance, balanceRaw, p.tauFast, dt)

    st.driveYaw = lowPass(st.driveYaw, driveYawRaw, p.tauFast, dt)
    st.lsdYaw = lowPass(st.lsdYaw, lsdYawRaw, p.tauFast, dt)
    st.complianceYaw = lowPass(st.complianceYaw, compYawRaw, p.tauFast, dt)
    st.loadYaw = lowPass(st.loadYaw, loadYawRaw, p.tauFast, dt)
    st.roadYaw = lowPass(st.roadYaw, roadYawRaw, p.tauFast, dt)
    st.forceYaw = lowPass(st.forceYaw, forceYawRaw, p.tauFast, dt)

    st.recoveryYaw = lowPass(st.recoveryYaw, rearRecovery, p.tauRisk, dt)
    st.biteYaw = lowPass(st.biteYaw, rearBite, p.tauRisk, dt)
    st.dirtyYaw = lowPass(st.dirtyYaw, rearDirty, p.tauRisk, dt)
    st.recontactYaw = lowPass(st.recontactYaw, rearRecontact, p.tauRisk, dt)
    st.slipRecoveryRear = lowPass(st.slipRecoveryRear, rearGripReturn, p.tauRisk, dt)
    st.slipSnapRear = lowPass(st.slipSnapRear, rearSnap, p.tauRisk, dt)

    local historyLoss = clamp(rearThermal * p.thermalYawLossGain + rearAbrasion * p.abrasionYawLossGain, 0.0, 0.20)
    st.yawBudget = clamp(st.yawBudget - historyLoss * sign(st.yawBudget), -1.0, 1.0)

    st.avgAuthority = authoritySum * 0.25
    st.authorityBalance = clamp(rearAuthority - frontAuthority, -1.0, 1.0)
    st.maxWheelSlip = maxWheelSlip
    st.activeCount = activeCount

    st.phaseId, st.phaseText = classifyPhase(st)
end

--============================================================
-- Export
--============================================================

local function exportWheel(index, ws)
    safeStore("ngp_yaw_wheel_authority_" .. index, ws.authority or 0.0)
    safeStore("ngp_yaw_wheel_contact_" .. index, ws.contact or 0.0)
    safeStore("ngp_yaw_wheel_slip_" .. index, ws.combinedSlip or 0.0)
    safeStore("ngp_yaw_wheel_memory_" .. index, ws.memory or 0.0)
    safeStore("ngp_yaw_wheel_recovery_" .. index, ws.recoveryRate or 0.0)
    safeStore("ngp_yaw_wheel_grip_return_" .. index, ws.gripReturn or 1.0)
    safeStore("ngp_yaw_wheel_snap_" .. index, ws.snapRisk or 0.0)
    safeStore("ngp_yaw_wheel_bite_" .. index, ws.bite or 0.0)
    safeStore("ngp_yaw_wheel_dirty_return_" .. index, ws.dirtyReturn or 0.0)

    safeStore("ngp_ymb_wheel_authority_" .. index, ws.authority or 0.0)
    safeStore("ngp_ymb_wheel_slip_" .. index, ws.combinedSlip or 0.0)
    safeStore("ngp_ymb_wheel_load_" .. index, ws.load or 0.0)

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_yaw_wheel_load_" .. index, ws.load or 0.0)
    safeStore("ngp_yaw_wheel_grip_" .. index, ws.grip or 1.0)
    safeStore("ngp_yaw_wheel_compliance_" .. index, ws.complianceEnergy or 0.0)
    safeStore("ngp_yaw_wheel_vtoe_" .. index, ws.virtualToe or 0.0)
    safeStore("ngp_yaw_wheel_vcamber_" .. index, ws.virtualCamber or 0.0)
    safeStore("ngp_yaw_wheel_road_" .. index, ws.tireForceRoadInput or 0.0)
    safeStore("ngp_yaw_wheel_force_" .. index, ws.tireForceCombined or 0.0)
    safeStore("ngp_yaw_wheel_mu_scale_" .. index, ws.thermalMuScale or 1.0)
end

local function exportGlobal()
    local st = M.state

    safeStore("ngp_yaw_budget_status", st.status or "UNKNOWN")
    safeStore("ngp_yaw_budget_update_count", st.updateCount or 0)
    safeStore("ngp_yaw_budget_phase_id", st.phaseId or 0)
    safeStore("ngp_yaw_budget_phase", st.phaseText or "UNKNOWN")

    safeStore("ngp_yaw_budget", st.yawBudget or 0.0)
    safeStore("ngp_yaw_balance", st.balance or 0.0)
    safeStore("ngp_yaw_rotation_intent", st.rotationIntent or 0.0)
    safeStore("ngp_yaw_understeer_energy", st.understeerEnergy or 0.0)
    safeStore("ngp_yaw_oversteer_energy", st.oversteerEnergy or 0.0)
    safeStore("ngp_yaw_recovery_phase", st.recoveryPhase or 0.0)
    safeStore("ngp_yaw_spin_risk", st.spinRisk or 0.0)

    safeStore("ngp_understeer_energy", st.understeerEnergy or 0.0)
    safeStore("ngp_oversteer_energy", st.oversteerEnergy or 0.0)
    safeStore("ngp_rotation_intent", st.rotationIntent or 0.0)
    safeStore("ngp_recovery_phase", st.recoveryPhase or 0.0)
    safeStore("ngp_spin_risk", st.spinRisk or 0.0)

    safeStore("ngp_yaw_front_authority", st.frontAuthority or 0.0)
    safeStore("ngp_yaw_rear_authority", st.rearAuthority or 0.0)
    safeStore("ngp_yaw_front_contact", st.frontContact or 0.0)
    safeStore("ngp_yaw_rear_contact", st.rearContact or 0.0)
    safeStore("ngp_yaw_front_slip", st.frontSlip or 0.0)
    safeStore("ngp_yaw_rear_slip", st.rearSlip or 0.0)
    safeStore("ngp_yaw_front_memory", st.frontMemory or 0.0)
    safeStore("ngp_yaw_rear_memory", st.rearMemory or 0.0)

    safeStore("ngp_yaw_steer_demand", st.steerDemand or 0.0)
    safeStore("ngp_yaw_response", st.yawResponse or 0.0)
    safeStore("ngp_yaw_rate", st.yawRate or 0.0)
    safeStore("ngp_yaw_speed_kmh", st.speedKmh or 0.0)

    safeStore("ngp_yaw_drive", st.driveYaw or 0.0)
    safeStore("ngp_yaw_lsd", st.lsdYaw or 0.0)
    safeStore("ngp_yaw_compliance", st.complianceYaw or 0.0)
    safeStore("ngp_yaw_load", st.loadYaw or 0.0)
    safeStore("ngp_yaw_road", st.roadYaw or 0.0)
    safeStore("ngp_yaw_force", st.forceYaw or 0.0)

    safeStore("ngp_yaw_recovery", st.recoveryYaw or 0.0)
    safeStore("ngp_yaw_bite", st.biteYaw or 0.0)
    safeStore("ngp_yaw_dirty_return", st.dirtyYaw or 0.0)
    safeStore("ngp_yaw_recontact", st.recontactYaw or 0.0)
    safeStore("ngp_yaw_slip_grip_return", st.slipRecoveryRear or 1.0)
    safeStore("ngp_yaw_slip_snap_rear", st.slipSnapRear or 0.0)

    safeStore("ngp_yaw_bite_energy", st.biteYaw or 0.0)
    safeStore("ngp_yaw_dirty_energy", st.dirtyYaw or 0.0)
    safeStore("ngp_yaw_recontact_energy", st.recontactYaw or 0.0)

    safeStore("ngp_ymb_status", st.status or "UNKNOWN")
    safeStore("ngp_ymb_budget", st.yawBudget or 0.0)
    safeStore("ngp_ymb_balance", st.balance or 0.0)
    safeStore("ngp_ymb_understeer", st.understeerEnergy or 0.0)
    safeStore("ngp_ymb_oversteer", st.oversteerEnergy or 0.0)
    safeStore("ngp_ymb_recovery", st.recoveryPhase or 0.0)
    safeStore("ngp_ymb_spin_risk", st.spinRisk or 0.0)
    safeStore("ngp_ymb_phase_id", st.phaseId or 0)
    safeStore("ngp_ymb_phase", st.phaseText or "UNKNOWN")
    safeStore("ngp_ymb_avg_authority", st.avgAuthority or 0.0)
    safeStore("ngp_ymb_authority_balance", st.authorityBalance or 0.0)
    safeStore("ngp_ymb_max_wheel_slip", st.maxWheelSlip or 0.0)
    safeStore("ngp_ymb_active_count", st.activeCount or 0)
    safeStore("ngp_ymb_store_only", st.storeOnly and 1 or 0)

    if not st.debugStoreNow then
        return
    end

    safeStore("ngp_yaw_contact_linked", st.contactLinked and 1 or 0)
    safeStore("ngp_yaw_memory_linked", st.tireMemoryLinked and 1 or 0)
    safeStore("ngp_yaw_compliance_linked", st.complianceLinked and 1 or 0)
    safeStore("ngp_yaw_drivetrain_linked", st.drivetrainLinked and 1 or 0)
    safeStore("ngp_yaw_lsd_linked", st.lsdLinked and 1 or 0)
    safeStore("ngp_yaw_load_linked", st.loadLinked and 1 or 0)
    safeStore("ngp_yaw_slip_recovery_linked", st.slipRecoveryLinked and 1 or 0)
    safeStore("ngp_yaw_carcass_linked", st.carcassLinked and 1 or 0)
    safeStore("ngp_yaw_tire_force_linked", st.tireForceLinked and 1 or 0)
    safeStore("ngp_yaw_road_linked", st.roadLinked and 1 or 0)
    safeStore("ngp_yaw_thermal_linked", st.thermalLinked and 1 or 0)
    safeStore("ngp_yaw_ultra_chassis_linked", st.ultraChassisLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i, M.state.wheels[i])
    end
    exportGlobal()
end

local function resetLinks()
    local st = M.state
    st.contactLinked = false
    st.tireMemoryLinked = false
    st.complianceLinked = false
    st.drivetrainLinked = false
    st.lsdLinked = false
    st.loadLinked = false
    st.slipRecoveryLinked = false
    st.carcassLinked = false
    st.tireForceLinked = false
    st.roadLinked = false
    st.thermalLinked = false
    st.ultraChassisLinked = false
end

local function updateInputs(car)
    local st = M.state

    st.speedKmh = getSpeedKmh(car)
    st.steer = clamp(getInputValue(car, "steer", st.steer or 0.0, "ngp_steer", "ngp_steer_input"), -1.0, 1.0)
    st.gas = clamp(getInputValue(car, "gas", st.gas or 0.0, "ngp_gas", "ngp_throttle", "ngp_throttle_input"), 0.0, 1.0)
    st.brake = clamp(getInputValue(car, "brake", st.brake or 0.0, "ngp_brake", "ngp_brake_input"), 0.0, 1.0)
    st.handbrake = clamp(getInputValue(car, "handbrake", st.handbrake or 0.0, "ngp_handbrake", "ngp_handbrake_input"), 0.0, 1.0)
    st.yawRate = getYawRate(car)
end

--============================================================
-- Public API
--============================================================

function M.init()
    M.state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    local st = M.state
    st.updateCount = (st.updateCount or 0) + 1

    dt = numberOr(dt, 0.0)
    if dt <= 0.0 then
        st.status = "BAD DT"
        exportState()
        return
    end
    dt = clamp(dt, M.params.minDt, M.params.maxDt)

    updateDebugGate(dt)

    if not car then
        car = safeGetCar()
    end

    local wheelsValid = hasWheels(car)
    st.wheelsValid = wheelsValid
    st.storeOnly = not wheelsValid

    resetLinks()
    updateInputs(car)

    for i = 0, 3 do
        local wheel = nil
        if wheelsValid then
            wheel = getWheel(car, i)
        end
        readWheelInputs(i, wheel)
    end

    updateBudget(dt)

    if car == nil then
        st.status = "NO CAR / STORE ONLY"
    elseif not wheelsValid then
        st.status = "NO WHEELS / STORE ONLY"
    else
        st.status = "RUNNING"
    end

    exportState()
end

function M.getBudget()
    return M.state.yawBudget or 0.0
end

function M.getUndersteer()
    return M.state.understeerEnergy or 0.0
end

function M.getOversteer()
    return M.state.oversteerEnergy or 0.0
end

function M.getSpinRisk()
    return M.state.spinRisk or 0.0
end

function M.getBalance()
    return M.state.balance or 0.0
end

function M.getPhase()
    return M.state.phaseId or 0, M.state.phaseText or "UNKNOWN"
end

function M.getWheelAuthority(index)
    local ws = M.state.wheels[index]
    return ws and (ws.authority or 0.0) or 0.0
end

function M.getState(index)
    if index == nil then
        return M.state
    end
    return M.state.wheels[index]
end

function M.debugStr()
    local st = M.state

    return string.format(
        "Status %s / Count %.0f / Phase %s\n" ..
        "Budget %+.3f / Bal %+.3f / Rot %.3f / Spin %.3f\n" ..
        "US %.3f / OS %.3f / Rec %.3f\n" ..
        "Auth F %.3f R %.3f / Slip F %.2f R %.2f\n" ..
        "Yaw %.3f / Demand %+.3f / Resp %+.3f\n" ..
        "Drive %.3f LSD %.3f Comp %+.3f Load %+.3f\n" ..
        "Road %+.3f Force %+.3f / StoreOnly %s\n" ..
        "RecYaw %.3f Bite %.3f Dirty %.3f ReC %.3f\n" ..
        "Links CQ:%s MEM:%s CS:%s DT:%s LSD:%s SR:%s TF:%s RD:%s TH:%s UC:%s",

        tostring(st.status),
        st.updateCount or 0,
        tostring(st.phaseText),

        st.yawBudget or 0.0,
        st.balance or 0.0,
        st.rotationIntent or 0.0,
        st.spinRisk or 0.0,

        st.understeerEnergy or 0.0,
        st.oversteerEnergy or 0.0,
        st.recoveryPhase or 0.0,

        st.frontAuthority or 0.0,
        st.rearAuthority or 0.0,
        st.frontSlip or 0.0,
        st.rearSlip or 0.0,

        st.yawRate or 0.0,
        st.steerDemand or 0.0,
        st.yawResponse or 0.0,

        st.driveYaw or 0.0,
        st.lsdYaw or 0.0,
        st.complianceYaw or 0.0,
        st.loadYaw or 0.0,

        st.roadYaw or 0.0,
        st.forceYaw or 0.0,
        st.storeOnly and "YES" or "NO",

        st.recoveryYaw or 0.0,
        st.biteYaw or 0.0,
        st.dirtyYaw or 0.0,
        st.recontactYaw or 0.0,

        st.contactLinked and "OK" or "NIL",
        st.tireMemoryLinked and "OK" or "NIL",
        st.complianceLinked and "OK" or "NIL",
        st.drivetrainLinked and "OK" or "NIL",
        st.lsdLinked and "OK" or "NIL",
        st.slipRecoveryLinked and "OK" or "NIL",
        st.tireForceLinked and "OK" or "NIL",
        st.roadLinked and "OK" or "NIL",
        st.thermalLinked and "OK" or "NIL",
        st.ultraChassisLinked and "OK" or "NIL"
    )
end

function M.drawDebug()
    if not ui or not ui.text then
        return
    end
    ui.text("=== YAW MOMENT BUDGET ===")
    ui.text(M.debugStr())
end

return M
