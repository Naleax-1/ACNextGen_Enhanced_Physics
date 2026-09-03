---@diagnostic disable: undefined-global

--============================================================
-- tire_contact_core.lua
-- ACNextGen V1.1.5 Stable
-- Tire Contact Foundation
--============================================================

local M = {}

local WHEEL_NAMES = { [0] = "FL", [1] = "FR", [2] = "RL", [3] = "RR" }

M.params = {
    loadRef = 3200.0,
    minGroundLoad = 80.0,

    contactLoadStart = 80.0,
    contactLoadFull  = 1800.0,

    slipRatioSoft = 0.08,
    slipRatioHard = 0.45,
    slipAngleSoft = 0.06,
    slipAngleHard = 0.35,

    contactRiseTau = 0.035,
    contactFallTau = 0.020,
    gripRiseTau = 0.075,
    gripFallTau = 0.040,
    energyBuildTau = 0.045,
    energyDecayTau = 0.160,
    loadTau = 0.060,
    externalTau = 0.075,

    minSpeedKmh = 1.0,
    minRollingContact = 0.15,
    driftGripFloor = 0.28,

    integratedLoadInfluence = 0.35,
    tireMemoryGripLoss = 0.10,
    tireHopContactLoss = 0.12,
    armGeometryContactLoss = 0.08,
    lsdRearEnergyGain = 0.08,
    driveRearEnergyGain = 0.10,

    contactQualityInfluence = 0.30,
    contactLossInfluence = 0.28,
    contactTrustInfluence = 0.12,
    carcassSupportInfluence = 0.18,
    carcassGripGateInfluence = 0.16,
    carcassDelayGripLoss = 0.08,
    complianceDelayGripLoss = 0.08,
    recoveryGripGain = 0.10,
    snapGripLoss = 0.12,
    roadShockEnergyGain = 0.08,
    roadSurfaceGripLoss = 0.10,
    loadPathLossGripLoss = 0.10,
    thermalGripLoss = 0.08,

    maxSlipEnergy = 1.0,
    maxLoad = 12000.0,

    unavailableDecayTau = 0.30,
    debugStoreInterval = 0.25,
}

local state = {
    contact = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    grip    = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    load    = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadN   = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    slipRatio = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    slipAngle = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    slipState   = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    patchEnergy = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    grounded    = { [0] = false, [1] = false, [2] = false, [3] = false },

    memory = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    hop    = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    armCamber = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    armToe    = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    contactQuality = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    contactTrust   = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    contactLoss    = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    carcassSupport = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    carcassGripGate = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    carcassDelay = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    carcassEnergy = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    tireCompliance = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    relaxScale = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    recovery = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    gripReturn = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    snapRisk = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    roadShock = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    surfaceLimit = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    pathLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireDelivery = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    thermalStress = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    avgContact = 0.0,
    avgGrip = 0.0,
    avgEnergy = 0.0,
    avgLoad = 0.0,
    minGrip = 1.0,
    maxEnergy = 0.0,
    groundedCount = 0,
    speedKmh = 0.0,

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,
    storeOnly = false,

    loadLinked = false,
    memoryLinked = false,
    hopLinked = false,
    armLinked = false,
    drivetrainLinked = false,
    lsdLinked = false,
    contactQualityLinked = false,
    carcassLinked = false,
    complianceLinked = false,
    recoveryLinked = false,
    roadLinked = false,
    loadPathLinked = false,
    thermalLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

local function safeNumber(value, defaultValue)
    local n = tonumber(value)
    if n == nil or n ~= n or n == math.huge or n == -math.huge then
        if defaultValue == nil then return nil end
        return defaultValue
    end
    return n
end

local function clamp(v, mn, mx)
    v = safeNumber(v, mn)
    if v < mn then return mn end
    if v > mx then return mx end
    return v
end

local function abs(v)
    return math.abs(safeNumber(v, 0.0))
end

local function lerp(a, b, t)
    t = clamp(t, 0.0, 1.0)
    return a + (b - a) * t
end

local function approachTau(current, target, dt, tau)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    dt = safeNumber(dt, 0.0)
    tau = safeNumber(tau, 0.001)
    if tau <= 0.0001 then return target end
    local k = 1.0 - math.exp(-dt / tau)
    return lerp(current, target, k)
end

local function safeLoadRaw(key)
    if not ac or not ac.load then return nil end
    local ok, value = pcall(function() return ac.load(key) end)
    if not ok then return nil end
    return value
end

local function safeLoad(key, fallback)
    local value = safeLoadRaw(key)
    if value == nil then return fallback or 0.0 end
    return safeNumber(value, fallback or 0.0)
end

local function safeStore(key, value)
    if not ac or not ac.store then return end
    pcall(function() ac.store(key, value) end)
end

local function safeField(obj, key, defaultValue)
    if not obj then return defaultValue end
    local ok, value = pcall(function() return obj[key] end)
    if not ok or value == nil then return defaultValue end
    return value
end

local function firstNumber(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return safeNumber(value, defaultValue or 0.0), keys[i]
        end
    end
    return defaultValue, nil
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

local function safeGetCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function() return ac.getCar(0) end)
    if not ok then return nil end
    return car
end

local function getWheels(car)
    return safeField(car, "wheels", nil)
end

local function getWheel(car, index)
    local wheels = getWheels(car)
    if not wheels then return nil end
    local wheel = safeField(wheels, index, nil)
    if wheel ~= nil then return wheel end
    return safeField(wheels, index + 1, nil)
end

local function getSpeedKmh(car)
    local speedKmh = safeNumber(safeField(car, "speedKmh", nil), nil)
    if speedKmh ~= nil then return speedKmh end
    local speed = safeNumber(nil, nil)
    if speed ~= nil then return speed * 3.6 end
    return safeLoad("ngp_hub_speed_kmh", safeLoad("ngp_speed_kmh", state.speedKmh or 0.0))
end

local function dynamicLoadAsAbsolute(value)
    local v = safeNumber(value, 0.0)
    if v >= M.params.loadRef * 0.65 and v <= M.params.maxLoad then
        return v
    end
    return M.params.loadRef + v
end

local function getWheelLoad(wheel, index)
    local direct = nil
    if wheel then
        direct = safeNumber(safeField(wheel, "load", nil), nil)
        if direct == nil then direct = safeNumber(safeField(wheel, "loadK", nil), nil) end
        if direct == nil then direct = safeNumber(nil, nil) end
        if direct == nil then direct = safeNumber(nil, nil) end
        if direct == nil then direct = safeNumber(nil, nil) end
    end

    local integrated, key = firstNumber(nil,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index,
        "ngp_hub_wheel_load_" .. index,
        "ngp_tire_load_input_" .. index,
        "ngp_contact_load_" .. index,
        "ngp_tire_state_load_" .. index)

    if key ~= nil then
        state.loadLinked = true
        if direct ~= nil then
            return clamp(direct * (1.0 - M.params.integratedLoadInfluence) + integrated * M.params.integratedLoadInfluence, 0.0, M.params.maxLoad), true
        end
        return clamp(integrated, 0.0, M.params.maxLoad), true
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        state.loadLinked = true
        local absolute = dynamicLoadAsAbsolute(dlt)
        if direct ~= nil then
            return clamp(direct * (1.0 - M.params.integratedLoadInfluence) + absolute * M.params.integratedLoadInfluence, 0.0, M.params.maxLoad), true
        end
        return clamp(absolute, 0.0, M.params.maxLoad), true
    end

    if direct ~= nil then
        return clamp(direct, 0.0, M.params.maxLoad), false
    end

    if state.storeOnly then
        return approachTau(state.load[index] or 0.0, 0.0, 0.016, M.params.unavailableDecayTau), false
    end

    return M.params.loadRef, false
end

local function readSlip(wheel, index)
    local sr, srKey = firstNumber(nil,
        "ngp_contact_slip_ratio_" .. index,
        "ngp_tire_slip_ratio_" .. index,
        "ngp_slip_ratio_" .. index,
        "ngp_filtered_slip_ratio_" .. index,
        "ngp_memory_slip_ratio_" .. index,
        "ngp_tire_carcass_slip_ratio_" .. index)

    local sa, saKey = firstNumber(nil,
        "ngp_contact_slip_angle_" .. index,
        "ngp_tire_slip_angle_" .. index,
        "ngp_slip_angle_" .. index,
        "ngp_filtered_slip_angle_" .. index,
        "ngp_memory_slip_angle_" .. index,
        "ngp_tire_carcass_slip_angle_" .. index)

    if sr == nil and wheel then sr = safeField(wheel, "slipRatio", 0.0) end
    if sa == nil and wheel then sa = safeField(wheel, "slipAngle", 0.0) end

    if srKey ~= nil or saKey ~= nil then
        state.contactQualityLinked = true
    end

    return abs(sr), abs(sa)
end

local function readExternal(index, dt)
    local mem, memKey = firstNumber(nil,
        "ngp_tire_memory_" .. index,
        "ngp_rubber_memory_" .. index,
        "ngp_memory_" .. index,
        "ngp_slide_memory_" .. index,
        "ngp_slip_slide_memory_" .. index)
    if memKey ~= nil then state.memoryLinked = true end
    state.memory[index] = approachTau(state.memory[index], clamp(safeNumber(mem, 0.0), 0.0, 1.0), dt, M.params.externalTau)

    local hop, hopKey = firstNumber(nil,
        "ngp_tirehop_energy_" .. index,
        "ngp_tire_hop_energy_" .. index,
        "ngp_tire_hop_" .. index,
        "ngp_road_input_hop_" .. index)
    if hopKey ~= nil then state.hopLinked = true end
    state.hop[index] = approachTau(state.hop[index], clamp(safeNumber(hop, 0.0), 0.0, 1.0), dt, M.params.externalTau)

    local camber, camberKey = firstNumber(nil,
        "ngp_control_arm_camber_" .. index,
        "ngp_arm_camber_" .. index,
        "ngp_compliance_virtual_camber_" .. index,
        "ngp_virtual_camber_" .. index)
    local toe, toeKey = firstNumber(nil,
        "ngp_control_arm_toe_" .. index,
        "ngp_arm_toe_" .. index,
        "ngp_compliance_virtual_toe_" .. index,
        "ngp_virtual_toe_" .. index)
    if camberKey ~= nil or toeKey ~= nil then state.armLinked = true end
    state.armCamber[index] = safeNumber(camber, 0.0)
    state.armToe[index] = safeNumber(toe, 0.0)

    local quality, qKey = firstNumber(nil,
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index)
    local trust, trustKey = firstNumber(nil,
        "ngp_contact_trust_" .. index,
        "ngp_contact_grip_gate_" .. index,
        "ngp_recovery_trust_" .. index)
    local loss, lossKey = firstNumber(nil,
        "ngp_contact_loss_" .. index,
        "ngp_tire_contact_loss_" .. index,
        "ngp_tcr_contact_loss_" .. index)
    if qKey ~= nil or trustKey ~= nil or lossKey ~= nil then state.contactQualityLinked = true end
    state.contactQuality[index] = approachTau(state.contactQuality[index], clamp(safeNumber(quality, 1.0), 0.0, 1.2), dt, M.params.externalTau)
    state.contactTrust[index] = approachTau(state.contactTrust[index], clamp(safeNumber(trust, state.contactQuality[index]), 0.0, 1.2), dt, M.params.externalTau)
    state.contactLoss[index] = approachTau(state.contactLoss[index], clamp(safeNumber(loss, 0.0), 0.0, 1.0), dt, M.params.externalTau)

    local support, supportKey = firstNumber(nil,
        "ngp_carcass_support_" .. index,
        "ngp_tire_carcass_support_" .. index)
    local gripGate, gateKey = firstNumber(nil,
        "ngp_carcass_grip_gate_" .. index,
        "ngp_slip_recovery_grip_gate_" .. index)
    local delay, delayKey = firstNumber(nil,
        "ngp_tire_contact_delay_" .. index,
        "ngp_contact_delay_" .. index,
        "ngp_slip_recovery_carcass_delay_" .. index)
    local carcassEnergy, energyKey = firstNumber(nil,
        "ngp_tire_sidewall_energy_" .. index,
        "ngp_sidewall_energy_" .. index,
        "ngp_tire_carcass_avg_energy")
    if supportKey ~= nil or gateKey ~= nil or delayKey ~= nil or energyKey ~= nil then state.carcassLinked = true end
    state.carcassSupport[index] = approachTau(state.carcassSupport[index], clamp(safeNumber(support, 1.0), 0.0, 1.2), dt, M.params.externalTau)
    state.carcassGripGate[index] = approachTau(state.carcassGripGate[index], clamp(safeNumber(gripGate, 1.0), 0.0, 1.2), dt, M.params.externalTau)
    state.carcassDelay[index] = approachTau(state.carcassDelay[index], clamp(safeNumber(delay, 0.0), 0.0, 1.0), dt, M.params.externalTau)
    state.carcassEnergy[index] = approachTau(state.carcassEnergy[index], clamp(safeNumber(carcassEnergy, 0.0), 0.0, 1.0), dt, M.params.externalTau)

    local compliance, compKey = firstNumber(nil,
        "ngp_tire_compliance_" .. index,
        index < 2 and "ngp_tire_comp_front" or "ngp_tire_comp_rear",
        index < 2 and "ngp_tyre_comp_front" or "ngp_tyre_comp_rear")
    local relax, relaxKey = firstNumber(nil,
        "ngp_tire_relax_scale_" .. index,
        "ngp_tyre_relax_scale_" .. index,
        "ngp_tire_response_delay")
    if compKey ~= nil or relaxKey ~= nil then state.complianceLinked = true end
    state.tireCompliance[index] = approachTau(state.tireCompliance[index], clamp(safeNumber(compliance, 1.0), 0.4, 1.4), dt, M.params.externalTau)
    state.relaxScale[index] = approachTau(state.relaxScale[index], clamp(safeNumber(relax, 1.0), 0.8, 1.8), dt, M.params.externalTau)

    local recovery, recoveryKey = firstNumber(nil,
        "ngp_slip_recovery_rate_" .. index,
        "ngp_recovery_rate_" .. index)
    local gripReturn, gripReturnKey = firstNumber(nil,
        "ngp_slip_grip_return_" .. index,
        "ngp_grip_return_" .. index)
    local snapRisk, snapKey = firstNumber(nil,
        "ngp_slip_snap_risk_" .. index,
        "ngp_snap_risk_" .. index)
    if recoveryKey ~= nil or gripReturnKey ~= nil or snapKey ~= nil then state.recoveryLinked = true end
    state.recovery[index] = approachTau(state.recovery[index], clamp(safeNumber(recovery, 0.0), 0.0, 1.0), dt, M.params.externalTau)
    state.gripReturn[index] = approachTau(state.gripReturn[index], clamp(safeNumber(gripReturn, 1.0), 0.0, 1.2), dt, M.params.externalTau)
    state.snapRisk[index] = approachTau(state.snapRisk[index], clamp(safeNumber(snapRisk, 0.0), 0.0, 1.0), dt, M.params.externalTau)

    local roadShock, roadKey = firstNumber(nil,
        "ngp_road_shock_" .. index,
        "ngp_rii_shock_" .. index,
        "ngp_road_input_severity_" .. index)
    local surfaceLimit, surfaceKey = firstNumber(nil,
        "ngp_road_surface_limit_" .. index,
        "ngp_rii_surface_limit_" .. index)
    if roadKey ~= nil or surfaceKey ~= nil then state.roadLinked = true end
    state.roadShock[index] = approachTau(state.roadShock[index], clamp(safeNumber(roadShock, 0.0), 0.0, 1.0), dt, M.params.externalTau)
    state.surfaceLimit[index] = approachTau(state.surfaceLimit[index], clamp(safeNumber(surfaceLimit, 0.0), 0.0, 1.0), dt, M.params.externalTau)

    local pathLoss, pathKey = firstNumber(nil,
        "ngp_load_path_loss_" .. index,
        "ngp_road_path_loss_" .. index,
        "ngp_rii_path_loss_" .. index)
    local delivery, deliveryKey = firstNumber(nil,
        "ngp_load_path_tire_delivery_" .. index,
        "ngp_road_tire_delivery_" .. index,
        "ngp_rii_avg_tire_delivery")
    if pathKey ~= nil or deliveryKey ~= nil then state.loadPathLinked = true end
    state.pathLoss[index] = approachTau(state.pathLoss[index], clamp(safeNumber(pathLoss, 0.0), 0.0, 1.0), dt, M.params.externalTau)
    state.tireDelivery[index] = approachTau(state.tireDelivery[index], clamp(safeNumber(delivery, 1.0), 0.0, 1.2), dt, M.params.externalTau)

    local tireTemp, tempKey = firstNumber(nil,
        "ngp_tire_temp_" .. index,
        "ngp_tyre_temp_" .. index,
        "ngp_memory_virtual_temp_" .. index)
    if tempKey ~= nil then state.thermalLinked = true end
    local thermal = 0.0
    if tireTemp ~= nil then
        thermal = clamp((safeNumber(tireTemp, 25.0) - 85.0) / 95.0, 0.0, 1.0)
    end
    state.thermalStress[index] = approachTau(state.thermalStress[index], thermal, dt, M.params.externalTau)

    if index >= 2 then
        if safeLoadRaw("ngp_drive_torque") ~= nil or safeLoadRaw("ngp_drivetrain_torque") ~= nil then
            state.drivetrainLinked = true
        end
        if safeLoadRaw("ngp_lsd_lock") ~= nil or safeLoadRaw("ngp_diff_lock") ~= nil then
            state.lsdLinked = true
        end
    end
end

local function smoothStep(edge0, edge1, x)
    local t = clamp((safeNumber(x, 0.0) - edge0) / math.max(edge1 - edge0, 0.0001), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)
end

local function contactFromLoad(load)
    local c = smoothStep(M.params.contactLoadStart, M.params.contactLoadFull, load)
    if load > M.params.minGroundLoad then
        c = math.max(c, M.params.minRollingContact)
    end
    return clamp(c, 0.0, 1.0)
end

local function slipEnergyFromSlip(sr, sa)
    local long = smoothStep(M.params.slipRatioSoft, M.params.slipRatioHard, sr)
    local lat = smoothStep(M.params.slipAngleSoft, M.params.slipAngleHard, sa)
    return clamp(math.max(long, lat), 0.0, M.params.maxSlipEnergy)
end

local function addRearDriveEnergy(index, energy)
    if index < 2 then return energy end
    local drive = abs(safeLoad("ngp_drive_torque", safeLoad("ngp_drivetrain_torque", 0.0))) * M.params.driveRearEnergyGain
    local lsd = safeLoad("ngp_lsd_lock", safeLoad("ngp_diff_lock", 0.0)) * M.params.lsdRearEnergyGain
    return clamp(energy + drive + lsd, 0.0, M.params.maxSlipEnergy)
end

local function contactTarget(index, loadContact)
    local externalQuality = state.contactQuality[index]
    local trust = state.contactTrust[index]
    local loss = state.contactLoss[index]
    local support = state.carcassSupport[index]

    local target = loadContact
    target = target * (1.0 + (externalQuality - 1.0) * M.params.contactQualityInfluence)
    target = target * (1.0 + (trust - 1.0) * M.params.contactTrustInfluence)
    target = target * (1.0 + (support - 1.0) * M.params.carcassSupportInfluence)
    target = target * (1.0 - loss * M.params.contactLossInfluence)
    target = target * (1.0 - state.surfaceLimit[index] * M.params.roadSurfaceGripLoss * 0.50)
    target = target * (1.0 - state.pathLoss[index] * M.params.loadPathLossGripLoss * 0.50)
    return clamp(target, 0.0, 1.0)
end

local function gripFromContactAndSlip(contact, slipEnergy, index)
    local grip = contact * (1.0 - slipEnergy * 0.72)
    grip = math.max(grip, M.params.driftGripFloor * contact)

    grip = grip * (1.0 - state.memory[index] * M.params.tireMemoryGripLoss)
    grip = grip * (1.0 - state.hop[index] * M.params.tireHopContactLoss)
    grip = grip * (1.0 - state.contactLoss[index] * M.params.contactLossInfluence)
    grip = grip * (1.0 + (state.carcassSupport[index] - 1.0) * M.params.carcassSupportInfluence)
    grip = grip * (1.0 + (state.carcassGripGate[index] - 1.0) * M.params.carcassGripGateInfluence)
    grip = grip * (1.0 - state.carcassDelay[index] * M.params.carcassDelayGripLoss)
    grip = grip * (1.0 - math.max(1.0 - state.tireCompliance[index], 0.0) * M.params.complianceDelayGripLoss)
    grip = grip * (1.0 + math.max(state.gripReturn[index] - 1.0, 0.0) * M.params.recoveryGripGain)
    grip = grip * (1.0 - state.snapRisk[index] * M.params.snapGripLoss)
    grip = grip * (1.0 - state.surfaceLimit[index] * M.params.roadSurfaceGripLoss)
    grip = grip * (1.0 - state.pathLoss[index] * M.params.loadPathLossGripLoss)
    grip = grip * clamp(state.tireDelivery[index], 0.0, 1.2)
    grip = grip * (1.0 - state.thermalStress[index] * M.params.thermalGripLoss)

    local geometryLoss = (abs(state.armCamber[index]) + abs(state.armToe[index])) * M.params.armGeometryContactLoss
    grip = grip * (1.0 - clamp(geometryLoss, 0.0, 0.20))

    return clamp(grip, 0.0, 1.20)
end

local function updateWheel(index, wheel, dt)
    readExternal(index, dt)

    local load = getWheelLoad(wheel, index)
    local sr, sa = readSlip(wheel, index)
    local loadN = clamp(load / math.max(M.params.loadRef, 1.0), 0.0, 2.0)
    local loadContact = contactFromLoad(load)

    if state.speedKmh < M.params.minSpeedKmh and load > M.params.minGroundLoad then
        loadContact = math.max(loadContact, M.params.minRollingContact)
    end

    local rawEnergy = slipEnergyFromSlip(sr, sa)
    rawEnergy = addRearDriveEnergy(index, rawEnergy)
    rawEnergy = clamp(rawEnergy + state.roadShock[index] * M.params.roadShockEnergyGain + state.carcassEnergy[index] * 0.06, 0.0, M.params.maxSlipEnergy)

    local targetContact = contactTarget(index, loadContact)
    local contactTau = targetContact > state.contact[index] and M.params.contactRiseTau or M.params.contactFallTau
    state.contact[index] = clamp(approachTau(state.contact[index], targetContact, dt, contactTau), 0.0, 1.2)

    local energyTau = rawEnergy > state.patchEnergy[index] and M.params.energyBuildTau or M.params.energyDecayTau
    state.patchEnergy[index] = clamp(approachTau(state.patchEnergy[index], rawEnergy, dt, energyTau), 0.0, M.params.maxSlipEnergy)

    local targetGrip = gripFromContactAndSlip(state.contact[index], state.patchEnergy[index], index)
    local gripTau = targetGrip > state.grip[index] and M.params.gripRiseTau or M.params.gripFallTau
    state.grip[index] = clamp(approachTau(state.grip[index], targetGrip, dt, gripTau), 0.0, 1.20)

    state.load[index] = approachTau(state.load[index] or load, load, dt, M.params.loadTau)
    state.loadN[index] = loadN
    state.slipRatio[index] = sr
    state.slipAngle[index] = sa
    state.slipState[index] = rawEnergy
    state.grounded[index] = load > M.params.minGroundLoad and targetContact > 0.02
end

local function decayWheel(index, dt)
    state.contact[index] = approachTau(state.contact[index], 0.0, dt, M.params.unavailableDecayTau)
    state.grip[index] = approachTau(state.grip[index], 0.0, dt, M.params.unavailableDecayTau)
    state.load[index] = approachTau(state.load[index], 0.0, dt, M.params.unavailableDecayTau)
    state.loadN[index] = approachTau(state.loadN[index], 0.0, dt, M.params.unavailableDecayTau)
    state.slipRatio[index] = approachTau(state.slipRatio[index], 0.0, dt, M.params.unavailableDecayTau)
    state.slipAngle[index] = approachTau(state.slipAngle[index], 0.0, dt, M.params.unavailableDecayTau)
    state.slipState[index] = approachTau(state.slipState[index], 0.0, dt, M.params.unavailableDecayTau)
    state.patchEnergy[index] = approachTau(state.patchEnergy[index], 0.0, dt, M.params.energyDecayTau)
    state.grounded[index] = false
end

local function exportWheel(i)
    safeStore("ngp_tc_contact_" .. i, state.contact[i] or 0.0)
    safeStore("ngp_tc_grip_" .. i, state.grip[i] or 0.0)
    safeStore("ngp_tc_load_" .. i, state.load[i] or 0.0)
    safeStore("ngp_tc_loadn_" .. i, state.loadN[i] or 0.0)
    safeStore("ngp_tc_slip_" .. i, state.slipState[i] or 0.0)
    safeStore("ngp_tc_energy_" .. i, state.patchEnergy[i] or 0.0)
    safeStore("ngp_tc_grounded_" .. i, state.grounded[i] and 1 or 0)

    safeStore("ngp_tire_contact_" .. i, state.contact[i] or 0.0)
    safeStore("ngp_tire_grip_contact_" .. i, state.grip[i] or 0.0)
    safeStore("ngp_tire_slip_energy_" .. i, state.patchEnergy[i] or 0.0)

    safeStore("ngp_tcc_contact_" .. i, state.contact[i] or 0.0)
    safeStore("ngp_tcc_grip_" .. i, state.grip[i] or 0.0)
    safeStore("ngp_tcc_energy_" .. i, state.patchEnergy[i] or 0.0)
    safeStore("ngp_tcc_load_" .. i, state.load[i] or 0.0)
    safeStore("ngp_tcc_quality_" .. i, state.contactQuality[i] or 1.0)
    safeStore("ngp_tcc_loss_" .. i, state.contactLoss[i] or 0.0)

    if not state.debugStoreNow then return end

    safeStore("ngp_tc_slip_ratio_" .. i, state.slipRatio[i] or 0.0)
    safeStore("ngp_tc_slip_angle_" .. i, state.slipAngle[i] or 0.0)
    safeStore("ngp_tc_memory_" .. i, state.memory[i] or 0.0)
    safeStore("ngp_tc_hop_" .. i, state.hop[i] or 0.0)
    safeStore("ngp_tc_contact_quality_" .. i, state.contactQuality[i] or 1.0)
    safeStore("ngp_tc_contact_trust_" .. i, state.contactTrust[i] or 1.0)
    safeStore("ngp_tc_contact_loss_" .. i, state.contactLoss[i] or 0.0)
    safeStore("ngp_tc_carcass_support_" .. i, state.carcassSupport[i] or 1.0)
    safeStore("ngp_tc_carcass_grip_gate_" .. i, state.carcassGripGate[i] or 1.0)
    safeStore("ngp_tc_recovery_" .. i, state.recovery[i] or 0.0)
    safeStore("ngp_tc_road_shock_" .. i, state.roadShock[i] or 0.0)
    safeStore("ngp_tc_path_loss_" .. i, state.pathLoss[i] or 0.0)
end

local function exportGlobal()
    safeStore("ngp_tc_status", state.status or "UNKNOWN")
    safeStore("ngp_tc_update_count", state.updateCount or 0)
    safeStore("ngp_tc_wheels_valid", state.wheelsValid and 1 or 0)
    safeStore("ngp_tc_store_only", state.storeOnly and 1 or 0)
    safeStore("ngp_tc_avg_contact", state.avgContact or 0.0)
    safeStore("ngp_tc_avg_grip", state.avgGrip or 0.0)
    safeStore("ngp_tc_avg_energy", state.avgEnergy or 0.0)
    safeStore("ngp_tc_avg_load", state.avgLoad or 0.0)
    safeStore("ngp_tc_min_grip", state.minGrip or 0.0)
    safeStore("ngp_tc_max_energy", state.maxEnergy or 0.0)
    safeStore("ngp_tc_grounded_count", state.groundedCount or 0)
    safeStore("ngp_tc_speed_kmh", state.speedKmh or 0.0)

    safeStore("ngp_tcc_avg_contact", state.avgContact or 0.0)
    safeStore("ngp_tcc_avg_grip", state.avgGrip or 0.0)
    safeStore("ngp_tcc_avg_energy", state.avgEnergy or 0.0)

    if not state.debugStoreNow then return end

    safeStore("ngp_tc_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_tc_memory_linked", state.memoryLinked and 1 or 0)
    safeStore("ngp_tc_hop_linked", state.hopLinked and 1 or 0)
    safeStore("ngp_tc_arm_linked", state.armLinked and 1 or 0)
    safeStore("ngp_tc_drivetrain_linked", state.drivetrainLinked and 1 or 0)
    safeStore("ngp_tc_lsd_linked", state.lsdLinked and 1 or 0)
    safeStore("ngp_tc_contact_quality_linked", state.contactQualityLinked and 1 or 0)
    safeStore("ngp_tc_carcass_linked", state.carcassLinked and 1 or 0)
    safeStore("ngp_tc_compliance_linked", state.complianceLinked and 1 or 0)
    safeStore("ngp_tc_recovery_linked", state.recoveryLinked and 1 or 0)
    safeStore("ngp_tc_road_linked", state.roadLinked and 1 or 0)
    safeStore("ngp_tc_load_path_linked", state.loadPathLinked and 1 or 0)
    safeStore("ngp_tc_thermal_linked", state.thermalLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i)
    end
    exportGlobal()
end

local function resetLinkFlags()
    state.loadLinked = false
    state.memoryLinked = false
    state.hopLinked = false
    state.armLinked = false
    state.drivetrainLinked = false
    state.lsdLinked = false
    state.contactQualityLinked = false
    state.carcassLinked = false
    state.complianceLinked = false
    state.recoveryLinked = false
    state.roadLinked = false
    state.loadPathLinked = false
    state.thermalLinked = false
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

    dt = clamp(dt, 0.0001, 0.100)
    updateDebugGate(dt)

    car = car or safeGetCar()
    local wheels = getWheels(car)

    state.speedKmh = getSpeedKmh(car)
    state.wheelsValid = wheels ~= nil
    state.storeOnly = not state.wheelsValid
    resetLinkFlags()

    local sumContact = 0.0
    local sumGrip = 0.0
    local sumEnergy = 0.0
    local sumLoad = 0.0
    local minGrip = 1.20
    local maxEnergy = 0.0
    local groundedCount = 0
    local storeActivity = false

    for i = 0, 3 do
        local wheel = getWheel(car, i)

        if wheel or state.storeOnly then
            updateWheel(i, wheel, dt)
        else
            decayWheel(i, dt)
        end

        if state.load[i] > M.params.minGroundLoad or state.patchEnergy[i] > 0.001 or state.contact[i] > 0.001 then
            storeActivity = true
        end

        if state.grounded[i] then
            groundedCount = groundedCount + 1
        end

        sumContact = sumContact + (state.contact[i] or 0.0)
        sumGrip = sumGrip + (state.grip[i] or 0.0)
        sumEnergy = sumEnergy + (state.patchEnergy[i] or 0.0)
        sumLoad = sumLoad + (state.load[i] or 0.0)
        minGrip = math.min(minGrip, state.grip[i] or 0.0)
        maxEnergy = math.max(maxEnergy, state.patchEnergy[i] or 0.0)
    end

    state.avgContact = sumContact * 0.25
    state.avgGrip = sumGrip * 0.25
    state.avgEnergy = sumEnergy * 0.25
    state.avgLoad = sumLoad * 0.25
    state.minGrip = minGrip
    state.maxEnergy = maxEnergy
    state.groundedCount = groundedCount

    if state.wheelsValid then
        state.status = "RUNNING"
    elseif storeActivity then
        state.status = "STORE ONLY"
    elseif not car then
        state.status = "NO CAR"
    else
        state.status = "NO WHEELS"
    end

    exportState()
end

function M.getContact(index)
    return state.contact[index] or 0.0
end

function M.getGrip(index)
    return state.grip[index] or 0.0
end

function M.getEnergy(index)
    return state.patchEnergy[index] or 0.0
end

function M.getLoad(index)
    return state.load[index] or 0.0
end

function M.getState(index)
    if index == nil then return state end
    return {
        name = WHEEL_NAMES[index] or tostring(index),
        contact = state.contact[index] or 0.0,
        grip = state.grip[index] or 0.0,
        energy = state.patchEnergy[index] or 0.0,
        load = state.load[index] or 0.0,
        grounded = state.grounded[index] and true or false,
    }
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0
        return string.format(
            "%s Contact %.2f Grip %.2f Energy %.2f Load %.0f\n" ..
            "SlipR %.3f SlipA %.3f Ground %s CQ %.2f Loss %.2f",
            WHEEL_NAMES[i] or tostring(i),
            state.contact[i] or 0.0,
            state.grip[i] or 0.0,
            state.patchEnergy[i] or 0.0,
            state.load[i] or 0.0,
            state.slipRatio[i] or 0.0,
            state.slipAngle[i] or 0.0,
            state.grounded[i] and "YES" or "NO",
            state.contactQuality[i] or 1.0,
            state.contactLoss[i] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s / Store %s\n" ..
        "Contact %.2f %.2f %.2f %.2f Avg %.2f\n" ..
        "Grip    %.2f %.2f %.2f %.2f Avg %.2f Min %.2f\n" ..
        "Energy  %.2f %.2f %.2f %.2f Avg %.2f Max %.2f\n" ..
        "Links LD:%s MEM:%s HOP:%s ARM:%s DT:%s LSD:%s CQ:%s TC:%s SR:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",
        state.storeOnly and "YES" or "NO",
        state.contact[0] or 0.0,
        state.contact[1] or 0.0,
        state.contact[2] or 0.0,
        state.contact[3] or 0.0,
        state.avgContact or 0.0,
        state.grip[0] or 0.0,
        state.grip[1] or 0.0,
        state.grip[2] or 0.0,
        state.grip[3] or 0.0,
        state.avgGrip or 0.0,
        state.minGrip or 0.0,
        state.patchEnergy[0] or 0.0,
        state.patchEnergy[1] or 0.0,
        state.patchEnergy[2] or 0.0,
        state.patchEnergy[3] or 0.0,
        state.avgEnergy or 0.0,
        state.maxEnergy or 0.0,
        state.loadLinked and "OK" or "NIL",
        state.memoryLinked and "OK" or "NIL",
        state.hopLinked and "OK" or "NIL",
        state.armLinked and "OK" or "NIL",
        state.drivetrainLinked and "OK" or "NIL",
        state.lsdLinked and "OK" or "NIL",
        state.contactQualityLinked and "OK" or "NIL",
        state.carcassLinked and "OK" or "NIL",
        state.recoveryLinked and "OK" or "NIL"
    )
end

return M
