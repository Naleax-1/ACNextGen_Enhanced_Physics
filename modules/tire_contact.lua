---@diagnostic disable: undefined-global

--============================================================
-- ACNextGen
-- tire_contact.lua
-- V1.1.5 Stable
-- Tire contact patch state bridge
--============================================================

local M = {}

M.params = {
    nominalLoad = 3000.0,
    loadSensitivity = 0.25,

    patchLateralScale = 0.80,
    patchLongScale = 0.60,
    patchInfluence = 0.12,

    minLoad = 100.0,
    defaultRadius = 0.33,
    defaultWidth = 0.225,
    defaultPressure = 1.80,
    minPressure = 0.50,

    minGripScale = 0.50,
    maxGripScale = 1.20,

    coreContactInfluence = 0.22,
    coreGripInfluence = 0.18,
    responseGripInfluence = 0.22,
    responseLossInfluence = 0.16,

    tireDynamicsGripInfluence = 0.12,
    tireLimitLossInfluence = 0.08,

    armCamberPatchLoss = 0.035,
    armToePatchLoss = 0.045,

    hopPatchLoss = 0.08,
    memoryPatchLoss = 0.06,

    loadBlend = 0.35,

    contactQualityInfluence = 0.18,
    contactTrustInfluence = 0.10,
    contactLossInfluence = 0.18,

    carcassSupportInfluence = 0.16,
    carcassGripGateInfluence = 0.12,
    carcassDeformationLoss = 0.10,

    slipRecoveryGain = 0.08,
    loadPathLossGain = 0.10,
    tireDeliveryGain = 0.12,
    surfaceLimitLoss = 0.08,
    thermalLossGain = 0.06,

    loadFilterTau = 0.050,
    areaTau = 0.060,
    gripTau = 0.055,
    corrTau = 0.050,
    storeOnlyDecayTau = 0.220,

    debugStoreInterval = 0.25,
}

local state = {
    area = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    gripScale = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    latCorr = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    longCorr = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    load = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    rawLoad = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    slipAngle = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    slipRatio = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    radius = { [0] = 0.33, [1] = 0.33, [2] = 0.33, [3] = 0.33 },
    width = { [0] = 0.225, [1] = 0.225, [2] = 0.225, [3] = 0.225 },
    pressure = { [0] = 1.8, [1] = 1.8, [2] = 1.8, [3] = 1.8 },

    coreContact = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    coreGrip = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    responseGrip = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    responseLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    contactQuality = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    contactTrust = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    contactLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    armCamber = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    armToe = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    memory = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    hop = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    carcassSupport = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    carcassGripGate = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    carcassDeformation = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    recoveryGripReturn = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    recoverySnapRisk = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    loadPathLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireDelivery = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    surfaceLimit = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    thermalStress = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    loadFront = 0.5,
    loadRear = 0.5,
    loadLeft = 0.5,
    loadRight = 0.5,

    avgArea = 0.0,
    avgGrip = 1.0,
    minGrip = 1.0,
    maxLoad = 0.0,

    linkedTireState = false,
    linkedLoadState = false,
    coreLinked = false,
    responseLinked = false,
    dynamicsLinked = false,
    armLinked = false,
    memoryLinked = false,
    hopLinked = false,
    contactQualityLinked = false,
    carcassLinked = false,
    recoveryLinked = false,
    loadPathLinked = false,
    roadLinked = false,
    thermalLinked = false,

    wheelsValid = false,
    storeOnly = false,

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
        return defaultValue
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
    return math.abs(safeNumber(v, 0.0) or 0.0)
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0) or 0.0
    target = safeNumber(target, 0.0) or 0.0
    tau = safeNumber(tau, 0.0) or 0.0
    dt = safeNumber(dt, 0.0) or 0.0

    if tau <= 0.0 then
        return target
    end

    return current + (target - current) * (dt / math.max(tau + dt, 0.0001))
end

local function safeField(obj, key, defaultValue)
    if not obj then return defaultValue end

    local ok, value = pcall(function()
        return obj[key]
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
        return defaultValue
    end

    return safeNumber(value, defaultValue)
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

local function getWheels(car)
    return safeField(car, "wheels", nil)
end

local function getWheel(wheels, index)
    if not wheels then
        return nil
    end

    local wheel = safeField(wheels, index, nil)
    if wheel ~= nil then
        return wheel
    end

    return safeField(wheels, index + 1, nil)
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

local function loadAlt(defaultValue, ...)
    local keys = { ... }

    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return safeNumber(value, defaultValue), keys[i]
        end
    end

    return defaultValue, nil
end

local function loadFromDlt(index)
    local value = safeLoadRaw("ngp_dlt_load_" .. index)
    if value == nil then
        return nil
    end

    local n = safeNumber(value, 0.0) or 0.0

    if math.abs(n) < M.params.nominalLoad * 0.80 then
        return M.params.nominalLoad + n
    end

    return n
end

local function readLoadState()
    local rawFront = safeLoadRaw("ngp_load_front")
    if rawFront == nil then rawFront = safeLoadRaw("ngp_front_bias") end

    local rawRear = safeLoadRaw("ngp_load_rear")
    if rawRear == nil then rawRear = safeLoadRaw("ngp_rear_bias") end

    local rawLeft = safeLoadRaw("ngp_load_left")
    if rawLeft == nil then rawLeft = safeLoadRaw("ngp_left_bias") end

    local rawRight = safeLoadRaw("ngp_load_right")
    if rawRight == nil then rawRight = safeLoadRaw("ngp_right_bias") end

    state.loadFront = clamp(safeNumber(rawFront, 0.5) or 0.5, 0.0, 1.0)
    state.loadRear = clamp(safeNumber(rawRear, 0.5) or 0.5, 0.0, 1.0)
    state.loadLeft = clamp(safeNumber(rawLeft, 0.5) or 0.5, 0.0, 1.0)
    state.loadRight = clamp(safeNumber(rawRight, 0.5) or 0.5, 0.0, 1.0)

    state.linkedLoadState = rawFront ~= nil or rawRear ~= nil or rawLeft ~= nil or rawRight ~= nil
end

local function readTireSlipState(index, wheel)
    local linked = false

    local slipAngle = safeLoadRaw("ngp_contact_slip_angle_" .. index)
    if slipAngle == nil then slipAngle = safeLoadRaw("ngp_tire_slip_angle_" .. index) end
    if slipAngle == nil then slipAngle = safeLoadRaw("ngp_slip_angle_" .. index) end
    if slipAngle == nil then slipAngle = safeLoadRaw("ngp_filtered_slip_angle_" .. index) end
    if slipAngle == nil then slipAngle = safeLoadRaw("ngp_memory_slip_angle_" .. index) end

    local slipRatio = safeLoadRaw("ngp_contact_slip_ratio_" .. index)
    if slipRatio == nil then slipRatio = safeLoadRaw("ngp_tire_slip_ratio_" .. index) end
    if slipRatio == nil then slipRatio = safeLoadRaw("ngp_slip_ratio_" .. index) end
    if slipRatio == nil then slipRatio = safeLoadRaw("ngp_filtered_slip_ratio_" .. index) end
    if slipRatio == nil then slipRatio = safeLoadRaw("ngp_memory_slip_ratio_" .. index) end

    if slipAngle ~= nil or slipRatio ~= nil then
        linked = true
    end

    slipAngle = safeNumber(slipAngle, safeNumber(safeField(wheel, "slipAngle", 0.0), 0.0)) or 0.0
    slipRatio = safeNumber(slipRatio, safeNumber(safeField(wheel, "slipRatio", 0.0), 0.0)) or 0.0

    state.slipAngle[index] = slipAngle
    state.slipRatio[index] = slipRatio

    return slipAngle, slipRatio, linked
end

local function readRootInputs(index)
    local coreContact = safeLoadRaw("ngp_tc_contact_" .. index)
    local coreGrip = safeLoadRaw("ngp_tc_grip_" .. index)

    if coreContact ~= nil or coreGrip ~= nil then
        state.coreLinked = true
    end

    state.coreContact[index] = clamp(safeNumber(coreContact, 1.0) or 1.0, 0.0, 1.2)
    state.coreGrip[index] = clamp(safeNumber(coreGrip, 1.0) or 1.0, 0.0, 1.25)

    local responseGrip = safeLoadRaw("ngp_tire_effective_grip_" .. index)
    if responseGrip == nil then responseGrip = safeLoadRaw("ngp_tcr_effective_grip_" .. index) end
    if responseGrip == nil then responseGrip = safeLoadRaw("ngp_contact_grip_gate_" .. index) end

    local responseLoss = safeLoadRaw("ngp_tire_contact_loss_" .. index)
    if responseLoss == nil then responseLoss = safeLoadRaw("ngp_tcr_contact_loss_" .. index) end
    if responseLoss == nil then responseLoss = safeLoadRaw("ngp_contact_loss_" .. index) end

    if responseGrip ~= nil or responseLoss ~= nil then
        state.responseLinked = true
    end

    state.responseGrip[index] = clamp(safeNumber(responseGrip, 1.0) or 1.0, 0.35, 1.25)
    state.responseLoss[index] = clamp(safeNumber(responseLoss, 0.0) or 0.0, 0.0, 1.0)

    local contactQuality = safeLoadRaw("ngp_contact_quality_" .. index)
    if contactQuality == nil then contactQuality = safeLoadRaw("ngp_tire_contact_quality_" .. index) end
    if contactQuality == nil then contactQuality = safeLoadRaw("ngp_tcr_quality_" .. index) end

    local contactTrust = safeLoadRaw("ngp_contact_trust_" .. index)
    if contactTrust == nil then contactTrust = safeLoadRaw("ngp_recovery_trust_" .. index) end

    local contactLoss = safeLoadRaw("ngp_contact_loss_" .. index)
    if contactLoss == nil then contactLoss = safeLoadRaw("ngp_tire_contact_loss_" .. index) end
    if contactLoss == nil then contactLoss = safeLoadRaw("ngp_tcr_contact_loss_" .. index) end

    if contactQuality ~= nil or contactTrust ~= nil or contactLoss ~= nil then
        state.contactQualityLinked = true
    end

    state.contactQuality[index] = clamp(safeNumber(contactQuality, 1.0) or 1.0, 0.0, 1.2)
    state.contactTrust[index] = clamp(safeNumber(contactTrust, state.contactQuality[index]) or state.contactQuality[index], 0.0, 1.2)
    state.contactLoss[index] = clamp(safeNumber(contactLoss, math.max(1.0 - state.contactQuality[index], 0.0)) or 0.0, 0.0, 1.0)

    local tireGrip = safeLoadRaw("ngp_tire_grip_" .. index)
    if tireGrip == nil then tireGrip = safeLoadRaw("ngp_memory_grip_" .. index) end

    local tireLimit = safeLoadRaw("ngp_tire_limit_" .. index)
    if tireLimit == nil then tireLimit = safeLoadRaw("ngp_tdyn_tire_limit_" .. index) end

    if tireGrip ~= nil or tireLimit ~= nil then
        state.dynamicsLinked = true
    end

    local camber = safeLoadRaw("ngp_control_arm_camber_" .. index)
    if camber == nil then camber = safeLoadRaw("ngp_arm_camber_" .. index) end
    if camber == nil then camber = safeLoadRaw("ngp_ca_camber_" .. index) end

    local toe = safeLoadRaw("ngp_control_arm_toe_" .. index)
    if toe == nil then toe = safeLoadRaw("ngp_arm_toe_" .. index) end
    if toe == nil then toe = safeLoadRaw("ngp_ca_toe_" .. index) end

    if camber ~= nil or toe ~= nil then
        state.armLinked = true
    end

    state.armCamber[index] = safeNumber(camber, 0.0) or 0.0
    state.armToe[index] = safeNumber(toe, 0.0) or 0.0

    local memory = safeLoadRaw("ngp_rubber_memory_" .. index)
    if memory == nil then memory = safeLoadRaw("ngp_tire_memory_" .. index) end
    if memory == nil then memory = safeLoadRaw("ngp_memory_" .. index) end
    if memory == nil then memory = safeLoadRaw("ngp_slide_memory_" .. index) end

    if memory ~= nil then
        state.memoryLinked = true
    end

    state.memory[index] = clamp(safeNumber(memory, 0.0) or 0.0, 0.0, 1.0)

    local hop = safeLoadRaw("ngp_tirehop_energy_" .. index)
    if hop == nil then hop = safeLoadRaw("ngp_tire_hop_energy_" .. index) end
    if hop == nil then hop = safeLoadRaw("ngp_tire_hop_" .. index) end

    if hop ~= nil then
        state.hopLinked = true
    end

    state.hop[index] = clamp(safeNumber(hop, 0.0) or 0.0, 0.0, 1.0)

    local support = safeLoadRaw("ngp_carcass_support_" .. index)
    if support == nil then support = safeLoadRaw("ngp_tire_carcass_support_" .. index) end

    local gripGate = safeLoadRaw("ngp_carcass_grip_gate_" .. index)
    if gripGate == nil then gripGate = safeLoadRaw("ngp_contact_grip_gate_" .. index) end

    local deformation = safeLoadRaw("ngp_tire_deformation_" .. index)
    if deformation == nil then deformation = safeLoadRaw("ngp_carcass_deformation_" .. index) end

    if support ~= nil or gripGate ~= nil or deformation ~= nil then
        state.carcassLinked = true
    end

    state.carcassSupport[index] = clamp(safeNumber(support, 1.0) or 1.0, 0.0, 1.2)
    state.carcassGripGate[index] = clamp(safeNumber(gripGate, 1.0) or 1.0, 0.0, 1.2)
    state.carcassDeformation[index] = clamp(safeNumber(deformation, 0.0) or 0.0, 0.0, 1.5)

    local gripReturn = safeLoadRaw("ngp_slip_grip_return_" .. index)
    if gripReturn == nil then gripReturn = safeLoadRaw("ngp_grip_return_" .. index) end

    local snap = safeLoadRaw("ngp_slip_snap_risk_" .. index)
    if snap == nil then snap = safeLoadRaw("ngp_snap_risk_" .. index) end

    if gripReturn ~= nil or snap ~= nil then
        state.recoveryLinked = true
    end

    state.recoveryGripReturn[index] = clamp(safeNumber(gripReturn, 1.0) or 1.0, 0.0, 1.2)
    state.recoverySnapRisk[index] = clamp(safeNumber(snap, 0.0) or 0.0, 0.0, 1.2)

    local pathLoss = safeLoadRaw("ngp_load_path_loss_" .. index)
    if pathLoss == nil then pathLoss = safeLoadRaw("ngp_lp_loss_" .. index) end
    if pathLoss == nil then pathLoss = safeLoadRaw("ngp_road_path_loss_" .. index) end

    local delivery = safeLoadRaw("ngp_load_path_tire_delivery_" .. index)
    if delivery == nil then delivery = safeLoadRaw("ngp_lp_tire_delivery_" .. index) end
    if delivery == nil then delivery = safeLoadRaw("ngp_road_tire_delivery_" .. index) end

    if pathLoss ~= nil or delivery ~= nil then
        state.loadPathLinked = true
    end

    state.loadPathLoss[index] = clamp(safeNumber(pathLoss, 0.0) or 0.0, 0.0, 1.2)
    state.tireDelivery[index] = clamp(safeNumber(delivery, 1.0) or 1.0, 0.0, 1.2)

    local surfaceLimit = safeLoadRaw("ngp_road_surface_limit_" .. index)
    if surfaceLimit == nil then surfaceLimit = safeLoadRaw("ngp_rii_surface_limit_" .. index) end

    if surfaceLimit ~= nil then
        state.roadLinked = true
    end

    state.surfaceLimit[index] = clamp(safeNumber(surfaceLimit, 0.0) or 0.0, 0.0, 1.2)

    local thermal = safeLoadRaw("ngp_thermal_stress_" .. index)
    if thermal == nil then thermal = safeLoadRaw("ngp_tire_heat_seed_" .. index) end
    if thermal == nil then thermal = safeLoadRaw("ngp_carcass_heat_seed_" .. index) end

    if thermal ~= nil then
        state.thermalLinked = true
    end

    state.thermalStress[index] = clamp(safeNumber(thermal, 0.0) or 0.0, 0.0, 1.2)

    return
        clamp(safeNumber(tireGrip, 1.0) or 1.0, 0.0, 1.25),
        clamp(safeNumber(tireLimit, 0.0) or 0.0, 0.0, 2.0)
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

    local integrated = safeLoadRaw("ngp_wheel_load_" .. index)
    if integrated == nil then integrated = safeLoadRaw("ngp_load_wheel_" .. index) end
    if integrated == nil then integrated = safeLoadRaw("ngp_contact_load_" .. index) end
    if integrated == nil then integrated = safeLoadRaw("ngp_tire_state_load_" .. index) end
    if integrated == nil then integrated = safeLoadRaw("ngp_tire_load_input_" .. index) end
    if integrated == nil then integrated = safeLoadRaw("ngp_hub_wheel_load_" .. index) end

    if integrated ~= nil then
        state.linkedLoadState = true
        integrated = safeNumber(integrated, direct or M.params.nominalLoad) or M.params.nominalLoad

        if direct ~= nil then
            return direct * (1.0 - M.params.loadBlend) + integrated * M.params.loadBlend
        end

        return integrated
    end

    local dlt = loadFromDlt(index)
    if dlt ~= nil then
        state.linkedLoadState = true

        if direct ~= nil then
            return direct * (1.0 - M.params.loadBlend) + dlt * M.params.loadBlend
        end

        return dlt
    end

    if direct ~= nil then
        return direct
    end

    return M.params.nominalLoad
end

local function readTyreRadius(wheel)
    local radius = safeNumber(safeField(wheel, "tyreRadius", nil), nil)
    if radius ~= nil then return radius end

    radius = safeNumber(safeField(wheel, "tyreRadius", nil), nil)
    if radius ~= nil then return radius end

    return M.params.defaultRadius
end

local function readTyreWidth(wheel)
    local width = safeNumber(safeField(wheel, "tyreWidth", nil), nil)
    if width ~= nil then return width end

    width = safeNumber(safeField(wheel, "tyreWidth", nil), nil)
    if width ~= nil then return width end

    return M.params.defaultWidth
end

local function readTyrePressure(wheel, index)
    local pressure = safeNumber(safeField(wheel, "tyrePressure", nil), nil)
    if pressure ~= nil then return pressure end

    pressure = safeNumber(safeField(wheel, "tyrePressure", nil), nil)
    if pressure ~= nil then return pressure end

    return safeLoad("ngp_tyre_pressure_" .. index, safeLoad("ngp_tire_pressure_" .. index, M.params.defaultPressure))
end

local function calculateContactArea(wheel, index, load)
    local radius = safeNumber(readTyreRadius(wheel), M.params.defaultRadius) or M.params.defaultRadius
    local width = safeNumber(readTyreWidth(wheel), M.params.defaultWidth) or M.params.defaultWidth
    local pressure = safeNumber(readTyrePressure(wheel, index), M.params.defaultPressure) or M.params.defaultPressure

    pressure = math.max(pressure, M.params.minPressure)
    width = math.max(width, 0.001)
    radius = math.max(radius, 0.001)
    load = math.max(safeNumber(load, M.params.nominalLoad) or M.params.nominalLoad, 0.0)

    state.radius[index] = radius
    state.width[index] = width
    state.pressure[index] = pressure

    local denominator = pressure * 100000.0 * width + 1.0
    local area = 2.0 * math.sqrt(math.max(load * radius, 0.0) / denominator) * width

    local carcassSpread =
        1.0
        + clamp(1.0 - state.carcassSupport[index], 0.0, 1.0) * 0.10
        + clamp(state.carcassDeformation[index], 0.0, 1.0) * 0.05

    return math.max(area * carcassSpread, 0.0)
end

local function calculateLoadGripScale(wheel, load)
    load = safeNumber(load, M.params.nominalLoad) or M.params.nominalLoad

    local loadRatio = load / math.max(M.params.nominalLoad, 1.0)
    local gripScale = 1.0 - M.params.loadSensitivity * (loadRatio - 1.0)
    gripScale = clamp(gripScale, M.params.minGripScale, M.params.maxGripScale)

    local surfaceGrip = safeNumber(safeField(wheel, "surfaceGrip", 1.0), 1.0) or 1.0

    return gripScale * surfaceGrip
end

local function applyLoadTransferGrip(index, gripScale)
    if not state.linkedLoadState then
        return gripScale
    end

    local axle = index <= 1 and state.loadFront or state.loadRear
    local side = (index == 0 or index == 2) and state.loadLeft or state.loadRight
    local factor = (axle * 0.75) + (side * 0.25)

    return gripScale * (0.92 + factor * 0.16)
end

local function applyCasterGrip(index, gripScale)
    local casterGrip = safeLoad("ngp_caster_grip_" .. index, 1.0) or 1.0
    return gripScale * casterGrip
end

local function applyRootGrip(index, gripScale, tireGrip, tireLimit)
    local scale = gripScale

    scale = scale * (1.0 + (state.coreContact[index] - 1.0) * M.params.coreContactInfluence)
    scale = scale * (1.0 + (state.coreGrip[index] - 1.0) * M.params.coreGripInfluence)
    scale = scale * (1.0 + (state.responseGrip[index] - 1.0) * M.params.responseGripInfluence)
    scale = scale * (1.0 - state.responseLoss[index] * M.params.responseLossInfluence)

    scale = scale * (1.0 + (state.contactQuality[index] - 1.0) * M.params.contactQualityInfluence)
    scale = scale * (1.0 + (state.contactTrust[index] - 1.0) * M.params.contactTrustInfluence)
    scale = scale * (1.0 - state.contactLoss[index] * M.params.contactLossInfluence)

    scale = scale * (1.0 + (tireGrip - 1.0) * M.params.tireDynamicsGripInfluence)
    scale = scale * (1.0 - tireLimit * M.params.tireLimitLossInfluence)

    local geometryLoss =
        math.abs(state.armCamber[index]) * M.params.armCamberPatchLoss
        + math.abs(state.armToe[index]) * M.params.armToePatchLoss

    scale = scale * (1.0 - clamp(geometryLoss, 0.0, 0.20))
    scale = scale * (1.0 - state.memory[index] * M.params.memoryPatchLoss)
    scale = scale * (1.0 - state.hop[index] * M.params.hopPatchLoss)

    scale = scale * (1.0 + (state.carcassSupport[index] - 1.0) * M.params.carcassSupportInfluence)
    scale = scale * (1.0 + (state.carcassGripGate[index] - 1.0) * M.params.carcassGripGateInfluence)
    scale = scale * (1.0 - state.carcassDeformation[index] * M.params.carcassDeformationLoss)

    scale = scale * (1.0 + (state.recoveryGripReturn[index] - 1.0) * M.params.slipRecoveryGain)
    scale = scale * (1.0 - state.recoverySnapRisk[index] * 0.04)

    scale = scale * (1.0 - state.loadPathLoss[index] * M.params.loadPathLossGain)
    scale = scale * (1.0 + (state.tireDelivery[index] - 1.0) * M.params.tireDeliveryGain)
    scale = scale * (1.0 - state.surfaceLimit[index] * M.params.surfaceLimitLoss)
    scale = scale * (1.0 - state.thermalStress[index] * M.params.thermalLossGain)

    return clamp(scale, M.params.minGripScale, M.params.maxGripScale)
end

local function calculatePatchCorrections(slipAngle, slipRatio, index)
    slipAngle = safeNumber(slipAngle, 0.0) or 0.0
    slipRatio = safeNumber(slipRatio, 0.0) or 0.0

    local lat = math.cos(math.min(math.abs(slipAngle) * M.params.patchLateralScale, math.pi / 4.0))
    local long = 1.0 / (1.0 + math.abs(slipRatio) * M.params.patchLongScale)

    local support = clamp(state.carcassSupport[index], 0.0, 1.2)
    local gate = clamp(state.carcassGripGate[index], 0.0, 1.2)

    lat = lat * (0.92 + support * 0.08) * (0.96 + gate * 0.04)
    long = long * (0.92 + support * 0.08) * (0.96 + gate * 0.04)

    return clamp(lat, 0.0, 1.2), clamp(long, 0.0, 1.2)
end

local function exportWheel(index)
    safeStore("ngp_cp_area_" .. index, state.area[index] or 0.0)
    safeStore("ngp_cp_grip_" .. index, state.gripScale[index] or 0.0)
    safeStore("ngp_cp_lat_" .. index, state.latCorr[index] or 0.0)
    safeStore("ngp_cp_long_" .. index, state.longCorr[index] or 0.0)
    safeStore("ngp_cp_load_" .. index, state.load[index] or 0.0)

    safeStore("ngp_tire_contact_area_" .. index, state.area[index] or 0.0)
    safeStore("ngp_tire_contact_grip_" .. index, state.gripScale[index] or 0.0)
    safeStore("ngp_tire_contact_lat_" .. index, state.latCorr[index] or 0.0)
    safeStore("ngp_tire_contact_long_" .. index, state.longCorr[index] or 0.0)
    safeStore("ngp_tire_contact_load_" .. index, state.load[index] or 0.0)

    safeStore("ngp_tire_patch_area_" .. index, state.area[index] or 0.0)
    safeStore("ngp_tire_patch_grip_" .. index, state.gripScale[index] or 0.0)
    safeStore("ngp_tire_patch_lat_" .. index, state.latCorr[index] or 0.0)
    safeStore("ngp_tire_patch_long_" .. index, state.longCorr[index] or 0.0)

    safeStore("ngp_tcp_area_" .. index, state.area[index] or 0.0)
    safeStore("ngp_tcp_grip_" .. index, state.gripScale[index] or 0.0)
    safeStore("ngp_tcp_lat_" .. index, state.latCorr[index] or 0.0)
    safeStore("ngp_tcp_long_" .. index, state.longCorr[index] or 0.0)
    safeStore("ngp_tcp_load_" .. index, state.load[index] or 0.0)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_cp_slip_angle_" .. index, state.slipAngle[index] or 0.0)
    safeStore("ngp_cp_slip_ratio_" .. index, state.slipRatio[index] or 0.0)
    safeStore("ngp_cp_pressure_" .. index, state.pressure[index] or M.params.defaultPressure)
    safeStore("ngp_cp_width_" .. index, state.width[index] or M.params.defaultWidth)
    safeStore("ngp_cp_radius_" .. index, state.radius[index] or M.params.defaultRadius)
    safeStore("ngp_cp_core_contact_" .. index, state.coreContact[index] or 1.0)
    safeStore("ngp_cp_response_grip_" .. index, state.responseGrip[index] or 1.0)
    safeStore("ngp_cp_response_loss_" .. index, state.responseLoss[index] or 0.0)
    safeStore("ngp_cp_contact_quality_" .. index, state.contactQuality[index] or 1.0)
    safeStore("ngp_cp_contact_trust_" .. index, state.contactTrust[index] or 1.0)
    safeStore("ngp_cp_contact_loss_" .. index, state.contactLoss[index] or 0.0)
    safeStore("ngp_cp_carcass_support_" .. index, state.carcassSupport[index] or 1.0)
    safeStore("ngp_cp_tire_delivery_" .. index, state.tireDelivery[index] or 1.0)
end

local function exportGlobal()
    safeStore("ngp_tire_contact_status", state.status or "UNKNOWN")
    safeStore("ngp_tire_contact_update_count", state.updateCount or 0)
    safeStore("ngp_tire_contact_wheels_valid", state.wheelsValid and 1 or 0)
    safeStore("ngp_tire_contact_store_only", state.storeOnly and 1 or 0)

    safeStore("ngp_tire_contact_linked_tire_state", state.linkedTireState and 1 or 0)
    safeStore("ngp_tire_contact_linked_load_state", state.linkedLoadState and 1 or 0)

    safeStore("ngp_tire_contact_avg_area", state.avgArea or 0.0)
    safeStore("ngp_tire_contact_avg_grip", state.avgGrip or 1.0)
    safeStore("ngp_tire_contact_min_grip", state.minGrip or 1.0)
    safeStore("ngp_tire_contact_max_load", state.maxLoad or 0.0)

    safeStore("ngp_tcp_avg_area", state.avgArea or 0.0)
    safeStore("ngp_tcp_avg_grip", state.avgGrip or 1.0)
    safeStore("ngp_tcp_min_grip", state.minGrip or 1.0)
    safeStore("ngp_tcp_max_load", state.maxLoad or 0.0)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_tire_contact_load_front", state.loadFront or 0.5)
    safeStore("ngp_tire_contact_load_rear", state.loadRear or 0.5)
    safeStore("ngp_tire_contact_load_left", state.loadLeft or 0.5)
    safeStore("ngp_tire_contact_load_right", state.loadRight or 0.5)

    safeStore("ngp_tire_contact_core_linked", state.coreLinked and 1 or 0)
    safeStore("ngp_tire_contact_response_linked", state.responseLinked and 1 or 0)
    safeStore("ngp_tire_contact_dynamics_linked", state.dynamicsLinked and 1 or 0)
    safeStore("ngp_tire_contact_arm_linked", state.armLinked and 1 or 0)
    safeStore("ngp_tire_contact_memory_linked", state.memoryLinked and 1 or 0)
    safeStore("ngp_tire_contact_hop_linked", state.hopLinked and 1 or 0)
    safeStore("ngp_tire_contact_quality_linked", state.contactQualityLinked and 1 or 0)
    safeStore("ngp_tire_contact_carcass_linked", state.carcassLinked and 1 or 0)
    safeStore("ngp_tire_contact_recovery_linked", state.recoveryLinked and 1 or 0)
    safeStore("ngp_tire_contact_load_path_linked", state.loadPathLinked and 1 or 0)
    safeStore("ngp_tire_contact_road_linked", state.roadLinked and 1 or 0)
    safeStore("ngp_tire_contact_thermal_linked", state.thermalLinked and 1 or 0)
end

local function softenWheel(index, dt, noContact)
    local targetGrip = noContact and 0.0 or 1.0
    local targetCorr = noContact and 0.0 or 1.0
    local targetArea = noContact and 0.0 or state.area[index]

    state.area[index] = lowPass(state.area[index], targetArea, M.params.storeOnlyDecayTau, dt)
    state.gripScale[index] = lowPass(state.gripScale[index], targetGrip, M.params.storeOnlyDecayTau, dt)
    state.latCorr[index] = lowPass(state.latCorr[index], targetCorr, M.params.storeOnlyDecayTau, dt)
    state.longCorr[index] = lowPass(state.longCorr[index], targetCorr, M.params.storeOnlyDecayTau, dt)

    if noContact then
        state.load[index] = lowPass(state.load[index], 0.0, M.params.storeOnlyDecayTau, dt)
        state.slipAngle[index] = 0.0
        state.slipRatio[index] = 0.0
    end
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i)
    end

    exportGlobal()
end

local function resetLinks()
    state.coreLinked = false
    state.responseLinked = false
    state.dynamicsLinked = false
    state.armLinked = false
    state.memoryLinked = false
    state.hopLinked = false
    state.contactQualityLinked = false
    state.carcassLinked = false
    state.recoveryLinked = false
    state.loadPathLinked = false
    state.roadLinked = false
    state.thermalLinked = false
end

local function updateWheel(index, wheel, dt)
    local load = getWheelLoad(wheel, index)
    load = math.max(safeNumber(load, M.params.nominalLoad) or M.params.nominalLoad, 0.0)

    state.rawLoad[index] = load
    state.load[index] = lowPass(state.load[index], load, M.params.loadFilterTau, dt)

    local filteredLoad = state.load[index]

    if filteredLoad < M.params.minLoad then
        softenWheel(index, dt, true)
        exportWheel(index)
        return
    end

    local area = calculateContactArea(wheel, index, filteredLoad)
    local gripScale = calculateLoadGripScale(wheel, filteredLoad)

    gripScale = applyLoadTransferGrip(index, gripScale)
    gripScale = applyCasterGrip(index, gripScale)

    local slipAngle, slipRatio, tireLinked = readTireSlipState(index, wheel)

    if tireLinked then
        state.linkedTireState = true
    end

    local tireGrip, tireLimit = readRootInputs(index)
    gripScale = applyRootGrip(index, gripScale, tireGrip, tireLimit)
    gripScale = clamp(gripScale, M.params.minGripScale, M.params.maxGripScale)

    local latCorr, longCorr = calculatePatchCorrections(slipAngle, slipRatio, index)

    state.area[index] = lowPass(state.area[index], area, M.params.areaTau, dt)
    state.gripScale[index] = lowPass(state.gripScale[index], gripScale, M.params.gripTau, dt)
    state.latCorr[index] = lowPass(state.latCorr[index], latCorr, M.params.corrTau, dt)
    state.longCorr[index] = lowPass(state.longCorr[index], longCorr, M.params.corrTau, dt)

    exportWheel(index)
end

function M.init()
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = safeNumber(dt, 0.0) or 0.0

    if dt <= 0.0 then
        state.status = "BAD DT"
        exportState()
        return
    end

    dt = clamp(dt, 0.0001, 0.100)

    updateDebugGate(dt)

    if not car then
        car = safeGetCar()
    end

    local wheels = getWheels(car)
    local hasCar = car ~= nil
    local hasWheels = wheels ~= nil

    state.wheelsValid = hasWheels
    state.storeOnly = not hasWheels

    if not hasCar then
        state.status = "STORE ONLY"
    elseif not hasWheels then
        state.status = "NO WHEELS"
    else
        state.status = "RUNNING"
    end

    state.linkedTireState = false
    state.linkedLoadState = false

    resetLinks()
    readLoadState()

    local sumArea = 0.0
    local sumGrip = 0.0
    local minGrip = 999.0
    local maxLoad = 0.0

    for i = 0, 3 do
        local wheel = getWheel(wheels, i)

        if hasWheels and not wheel then
            softenWheel(i, dt, true)
            exportWheel(i)
        else
            updateWheel(i, wheel, dt)
        end

        sumArea = sumArea + (state.area[i] or 0.0)
        sumGrip = sumGrip + (state.gripScale[i] or 1.0)
        minGrip = math.min(minGrip, state.gripScale[i] or 1.0)
        maxLoad = math.max(maxLoad, state.load[i] or 0.0)
    end

    state.avgArea = sumArea * 0.25
    state.avgGrip = sumGrip * 0.25
    state.minGrip = minGrip == 999.0 and 1.0 or minGrip
    state.maxLoad = maxLoad

    exportGlobal()
end

function M.getArea(index)
    return state.area[index] or 0.0
end

function M.getGripScale(index)
    return state.gripScale[index] or 0.0
end

function M.getLateralCorrection(index)
    return state.latCorr[index] or 0.0
end

function M.getLongitudinalCorrection(index)
    return state.longCorr[index] or 0.0
end

function M.getLoad(index)
    return state.load[index] or 0.0
end

function M.getState(index)
    if index == nil then
        return state
    end

    return {
        area = state.area[index] or 0.0,
        gripScale = state.gripScale[index] or 0.0,
        latCorr = state.latCorr[index] or 0.0,
        longCorr = state.longCorr[index] or 0.0,
        load = state.load[index] or 0.0,
        slipAngle = state.slipAngle[index] or 0.0,
        slipRatio = state.slipRatio[index] or 0.0,
        pressure = state.pressure[index] or M.params.defaultPressure,
        width = state.width[index] or M.params.defaultWidth,
        radius = state.radius[index] or M.params.defaultRadius,
    }
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0

        return string.format(
            "Status %s / Count %.0f\n" ..
            "Area %.4f Grip %.3f Lat %.3f Long %.3f\n" ..
            "Load %.0f SlipA %.3f SlipR %.3f\n" ..
            "CQ %.2f Trust %.2f Loss %.2f Carc %.2f Del %.2f",
            tostring(state.status),
            state.updateCount or 0,
            state.area[i] or 0.0,
            state.gripScale[i] or 0.0,
            state.latCorr[i] or 0.0,
            state.longCorr[i] or 0.0,
            state.load[i] or 0.0,
            state.slipAngle[i] or 0.0,
            state.slipRatio[i] or 0.0,
            state.contactQuality[i] or 1.0,
            state.contactTrust[i] or 1.0,
            state.contactLoss[i] or 0.0,
            state.carcassSupport[i] or 1.0,
            state.tireDelivery[i] or 1.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s / StoreOnly %s\n" ..
        "CP Grip FL %.3f FR %.3f RL %.3f RR %.3f\n" ..
        "Area    FL %.4f FR %.4f RL %.4f RR %.4f\n" ..
        "LatCorr %.3f %.3f %.3f %.3f\n" ..
        "LongCr  %.3f %.3f %.3f %.3f\n" ..
        "Links TS:%s LT:%s Core:%s Resp:%s Dyn:%s Arm:%s CQ:%s Carc:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",
        state.storeOnly and "YES" or "NO",
        state.gripScale[0] or 0.0,
        state.gripScale[1] or 0.0,
        state.gripScale[2] or 0.0,
        state.gripScale[3] or 0.0,
        state.area[0] or 0.0,
        state.area[1] or 0.0,
        state.area[2] or 0.0,
        state.area[3] or 0.0,
        state.latCorr[0] or 0.0,
        state.latCorr[1] or 0.0,
        state.latCorr[2] or 0.0,
        state.latCorr[3] or 0.0,
        state.longCorr[0] or 0.0,
        state.longCorr[1] or 0.0,
        state.longCorr[2] or 0.0,
        state.longCorr[3] or 0.0,
        state.linkedTireState and "OK" or "NIL",
        state.linkedLoadState and "OK" or "NIL",
        state.coreLinked and "OK" or "NIL",
        state.responseLinked and "OK" or "NIL",
        state.dynamicsLinked and "OK" or "NIL",
        state.armLinked and "OK" or "NIL",
        state.contactQualityLinked and "OK" or "NIL",
        state.carcassLinked and "OK" or "NIL"
    )
end

return M
