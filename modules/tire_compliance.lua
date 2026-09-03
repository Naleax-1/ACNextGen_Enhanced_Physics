---@diagnostic disable: undefined-global

--============================================================
-- tire_compliance.lua
-- ACNextGen V1.1.5 Stable
-- Tire Compliance / Response Delay Model
-- Root/trunk signal bridge only
--============================================================

local M = {}

M.params = {
    complianceTau = 0.120,
    deflectionTau = 0.100,
    responseTau = 0.090,

    loadReference = 3500.0,
    loadLow = 250.0,

    frontGain = 1.0,
    rearGain = 1.0,

    maxDeflection = 1.0,
    maxComplianceDrop = 0.42,

    loadDropGain = 0.35,

    minCompliance = 0.50,
    maxCompliance = 1.20,

    contactInputGain = 0.55,
    contactFlexGain = 0.035,
    contactTau = 0.070,

    contactLossDropGain = 0.18,
    contactQualityRecoverGain = 0.10,
    energyComplianceDropGain = 0.12,

    memoryDropGain = 0.08,
    hopDropGain = 0.10,

    armCamberDropGain = 0.035,
    armToeDropGain = 0.050,

    driveRearDropGain = 0.08,
    lsdRearDelayGain = 0.06,
    shaftDelayGain = 0.05,

    carcassDelayGain = 0.16,
    carcassDeformationGain = 0.16,
    carcassSupportGain = 0.14,
    slipRecoveryDelayGain = 0.12,
    roadShockDelayGain = 0.10,
    pathLossDelayGain = 0.12,
    thermalDelayGain = 0.08,

    tireDeliveryRecoverGain = 0.08,
    gripGateRecoverGain = 0.08,

    minResponseDelay = 0.0,
    maxResponseDelay = 1.0,

    dtMin = 0.0001,
    dtMax = 0.100,

    debugStoreInterval = 0.25,
}

local WHEEL_NAMES = { [0] = "FL", [1] = "FR", [2] = "RL", [3] = "RR" }

M.state = {
    frontCompliance = 1.0,
    rearCompliance  = 1.0,

    frontDeflection = 0.0,
    rearDeflection  = 0.0,

    responseDelay = 0.0,
    targetResponseDelay = 0.0,

    frontLoad = 0.0,
    rearLoad  = 0.0,

    targetFrontCompliance = 1.0,
    targetRearCompliance  = 1.0,

    frontDrop = 0.0,
    rearDrop = 0.0,

    avgContactInput = 0.0,
    avgContactLoss = 0.0,
    avgCarcassDelay = 0.0,
    avgRoadShock = 0.0,
    avgPathLoss = 0.0,

    load = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    contactInput = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    contactFlex = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    contactLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    contactQuality = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    memory = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    hop = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    armCamber = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    armToe = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    carcassDelay = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    carcassDeformation = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    carcassSupport = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    carcassGripGate = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    carcassReturn = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    recoveryRate = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    snapRisk = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    gripReturn = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    roadShock = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    pathLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireDelivery = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    thermalStress = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    compliance = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    deflection = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    relaxScale = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    loadLinked = false,
    contactLinked = false,
    memoryLinked = false,
    hopLinked = false,
    armLinked = false,
    drivetrainLinked = false,
    lsdLinked = false,
    carcassLinked = false,
    recoveryLinked = false,
    roadLinked = false,
    loadPathLinked = false,
    thermalLinked = false,

    wheelsValid = false,
    storeOnly = false,

    status = "INIT",
    updateCount = 0,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

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

local function safeLoad(key, fallback)
    local value = safeLoadRaw(key)
    if value == nil then
        return fallback or 0.0
    end
    return safeNumber(value, fallback or 0.0)
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

local function getWheels(car)
    return safeField(car, "wheels", nil)
end

local function getWheel(car, index)
    local wheels = getWheels(car)
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

local function readWheelLoadField(wheel)
    if not wheel then
        return nil
    end

    local fields = { "load", "loadK" }
    for i = 1, #fields do
        local value = safeField(wheel, fields[i], nil)
        if value ~= nil then
            return safeNumber(value, nil)
        end
    end

    return nil
end

local function normalizeDynamicLoad(value)
    value = safeNumber(value, 0.0)

    if value > M.params.loadReference * 0.55 then
        return value
    end

    return M.params.loadReference + value
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
    M.state.loadLinked = false
    M.state.contactLinked = false
    M.state.memoryLinked = false
    M.state.hopLinked = false
    M.state.armLinked = false
    M.state.drivetrainLinked = false
    M.state.lsdLinked = false
    M.state.carcassLinked = false
    M.state.recoveryLinked = false
    M.state.roadLinked = false
    M.state.loadPathLinked = false
    M.state.thermalLinked = false
end

local function getWheelLoad(car, index)
    local wheel = getWheel(car, index)
    local wheelLoad = readWheelLoadField(wheel)

    if wheelLoad ~= nil then
        return math.max(0.0, wheelLoad), false
    end

    local contactLoad = safeLoadRaw("ngp_contact_load_" .. index)
    if contactLoad ~= nil then
        M.state.contactLinked = true
        return math.max(0.0, safeNumber(contactLoad, M.params.loadReference)), true
    end

    local integrated = safeLoadRaw("ngp_wheel_load_" .. index)
    if integrated == nil then integrated = safeLoadRaw("ngp_load_wheel_" .. index) end
    if integrated == nil then integrated = safeLoadRaw("ngp_tire_load_" .. index) end
    if integrated == nil then integrated = safeLoadRaw("ngp_tire_carcass_load_" .. index) end
    if integrated == nil then integrated = safeLoadRaw("ngp_load_path_load_" .. index) end

    if integrated ~= nil then
        M.state.loadLinked = true
        return math.max(0.0, safeNumber(integrated, M.params.loadReference)), true
    end

    local dynamicLoad = safeLoadRaw("ngp_dlt_load_" .. index)
    if dynamicLoad ~= nil then
        M.state.loadLinked = true
        return math.max(0.0, normalizeDynamicLoad(dynamicLoad)), true
    end

    return M.params.loadReference, true
end

local function readAxleLoads(car)
    local loadFL = getWheelLoad(car, 0)
    local loadFR = getWheelLoad(car, 1)
    local loadRL = getWheelLoad(car, 2)
    local loadRR = getWheelLoad(car, 3)

    M.state.load[0] = loadFL
    M.state.load[1] = loadFR
    M.state.load[2] = loadRL
    M.state.load[3] = loadRR

    local frontLoad = (loadFL + loadFR) * 0.5
    local rearLoad = (loadRL + loadRR) * 0.5

    return frontLoad, rearLoad
end

local function readContactInput(index, dt)
    local contactInput, inputKey = safeLoadAlt(
        0.0,
        "ngp_compliance_contact_input_" .. index,
        "ngp_tcr_compliance_input_" .. index,
        "ngp_susp_contact_input_" .. index,
        "ngp_sci_input_" .. index,
        "ngp_road_module_hint_" .. index,
        "ngp_rii_hint_" .. index
    )

    if inputKey ~= nil then
        M.state.contactLinked = true
    end

    local contactQuality, qualityKey = safeLoadAlt(
        1.0,
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index,
        "ngp_contact_raw_quality_" .. index
    )

    local contactLoss, lossKey = safeLoadAlt(
        clamp(1.0 - contactQuality, 0.0, 1.0),
        "ngp_contact_loss_" .. index,
        "ngp_tire_contact_loss_" .. index,
        "ngp_tcr_contact_loss_" .. index,
        "ngp_contact_drop_" .. index,
        "ngp_susp_contact_drop_" .. index
    )

    if qualityKey ~= nil or lossKey ~= nil then
        M.state.contactLinked = true
    end

    M.state.contactInput[index] = clamp(contactInput, 0.0, 1.2)
    M.state.contactQuality[index] = clamp(contactQuality, 0.0, 1.2)
    M.state.contactLoss[index] = clamp(contactLoss, 0.0, 1.0)

    M.state.contactFlex[index] = lowPass(
        M.state.contactFlex[index],
        M.state.contactInput[index],
        M.params.contactTau,
        dt
    )
end

local function readMemoryHopArm(index)
    local memory, memKey = safeLoadAlt(
        0.0,
        "ngp_tire_memory_" .. index,
        "ngp_tyre_memory_" .. index,
        "ngp_rubber_memory_" .. index,
        "ngp_memory_" .. index,
        "ngp_slide_memory_" .. index
    )

    if memKey ~= nil then
        M.state.memoryLinked = true
    end

    M.state.memory[index] = clamp(memory, 0.0, 1.0)

    local hop, hopKey = safeLoadAlt(
        0.0,
        "ngp_tirehop_energy_" .. index,
        "ngp_tire_hop_energy_" .. index,
        "ngp_tire_hop_" .. index,
        "ngp_hop_risk_" .. index
    )

    if hopKey ~= nil then
        M.state.hopLinked = true
    end

    M.state.hop[index] = clamp(hop, 0.0, 1.0)

    local camber, camberKey = safeLoadAlt(
        0.0,
        "ngp_control_arm_camber_" .. index,
        "ngp_arm_camber_" .. index,
        "ngp_ca_camber_" .. index,
        "ngp_compliance_virtual_camber_" .. index,
        "ngp_virtual_camber_" .. index
    )

    local toe, toeKey = safeLoadAlt(
        0.0,
        "ngp_control_arm_toe_" .. index,
        "ngp_arm_toe_" .. index,
        "ngp_ca_toe_" .. index,
        "ngp_compliance_virtual_toe_" .. index,
        "ngp_virtual_toe_" .. index
    )

    if camberKey ~= nil or toeKey ~= nil then
        M.state.armLinked = true
    end

    M.state.armCamber[index] = safeNumber(camber, 0.0)
    M.state.armToe[index] = safeNumber(toe, 0.0)
end

local function readCarcassRecoveryRoad(index)
    local delay, delayKey = safeLoadAlt(
        0.0,
        "ngp_tire_contact_delay_" .. index,
        "ngp_contact_delay_" .. index,
        "ngp_carcass_delay_" .. index
    )

    local deformation, deformationKey = safeLoadAlt(
        0.0,
        "ngp_tire_deformation_" .. index,
        "ngp_carcass_deformation_" .. index,
        "ngp_tire_carcass_deformation_" .. index
    )

    local support, supportKey = safeLoadAlt(
        1.0,
        "ngp_carcass_support_" .. index,
        "ngp_tire_carcass_support_" .. index
    )

    local gripGate, gateKey = safeLoadAlt(
        1.0,
        "ngp_carcass_grip_gate_" .. index,
        "ngp_recovery_trust_" .. index
    )

    local returnForce, returnKey = safeLoadAlt(
        0.0,
        "ngp_tire_return_force_" .. index,
        "ngp_return_force_" .. index
    )

    if delayKey ~= nil or deformationKey ~= nil or supportKey ~= nil or gateKey ~= nil or returnKey ~= nil then
        M.state.carcassLinked = true
    end

    M.state.carcassDelay[index] = clamp(delay, 0.0, 1.2)
    M.state.carcassDeformation[index] = clamp(deformation, 0.0, 1.2)
    M.state.carcassSupport[index] = clamp(support, 0.0, 1.2)
    M.state.carcassGripGate[index] = clamp(gripGate, 0.0, 1.2)
    M.state.carcassReturn[index] = clamp(returnForce, 0.0, 1.2)

    local recovery, recoveryKey = safeLoadAlt(
        0.0,
        "ngp_slip_recovery_rate_" .. index,
        "ngp_recovery_rate_" .. index
    )

    local snap, snapKey = safeLoadAlt(
        0.0,
        "ngp_slip_snap_risk_" .. index,
        "ngp_snap_risk_" .. index
    )

    local gripReturn, gripKey = safeLoadAlt(
        1.0,
        "ngp_slip_grip_return_" .. index,
        "ngp_grip_return_" .. index
    )

    if recoveryKey ~= nil or snapKey ~= nil or gripKey ~= nil then
        M.state.recoveryLinked = true
    end

    M.state.recoveryRate[index] = clamp(recovery, 0.0, 1.2)
    M.state.snapRisk[index] = clamp(snap, 0.0, 1.2)
    M.state.gripReturn[index] = clamp(gripReturn, 0.0, 1.2)

    local shock, shockKey = safeLoadAlt(
        0.0,
        "ngp_road_shock_" .. index,
        "ngp_rii_shock_" .. index,
        "ngp_road_impact_" .. index,
        "ngp_rii_impact_" .. index
    )

    local pathLoss, pathKey = safeLoadAlt(
        0.0,
        "ngp_road_path_loss_" .. index,
        "ngp_rii_path_loss_" .. index,
        "ngp_load_path_loss_" .. index,
        "ngp_lp_loss_" .. index
    )

    local delivery, deliveryKey = safeLoadAlt(
        1.0,
        "ngp_road_tire_delivery_" .. index,
        "ngp_load_path_tire_delivery_" .. index,
        "ngp_lp_tire_delivery_" .. index
    )

    if shockKey ~= nil then
        M.state.roadLinked = true
    end

    if pathKey ~= nil or deliveryKey ~= nil then
        M.state.loadPathLinked = true
    end

    M.state.roadShock[index] = clamp(shock, 0.0, 1.2)
    M.state.pathLoss[index] = clamp(pathLoss, 0.0, 1.2)
    M.state.tireDelivery[index] = clamp(delivery, 0.0, 1.2)

    local thermal, thermalKey = safeLoadAlt(
        0.0,
        "ngp_slip_thermal_memory_" .. index,
        "ngp_tire_memory_thermal_" .. index,
        "ngp_thermal_stress",
        "ngp_virtual_thermal_stress"
    )

    if thermalKey ~= nil then
        M.state.thermalLinked = true
    end

    M.state.thermalStress[index] = clamp(thermal, 0.0, 1.2)
end

local function readDrivetrainLinks()
    local driveTorque = safeLoadRaw("ngp_drive_torque")
    if driveTorque == nil then driveTorque = safeLoadRaw("ngp_drivetrain_torque") end
    if driveTorque == nil then driveTorque = safeLoadRaw("ngp_dt_torque") end

    if driveTorque ~= nil then
        M.state.drivetrainLinked = true
    end

    local lsdLock = safeLoadRaw("ngp_lsd_lock")
    if lsdLock == nil then lsdLock = safeLoadRaw("ngp_diff_lock") end

    if lsdLock ~= nil then
        M.state.lsdLinked = true
    end
end

local function readWheelRootInput(index, dt)
    readContactInput(index, dt)
    readMemoryHopArm(index)
    readCarcassRecoveryRoad(index)

    if index >= 2 then
        readDrivetrainLinks()
    end
end

local function calculateAxleDrop(startWheel, load, gain)
    local loadRatio = load / math.max(M.params.loadReference, 1.0)
    local overload = math.max(0.0, loadRatio - 1.0)
    local underload = clamp(1.0 - load / math.max(M.params.loadLow, 1.0), 0.0, 1.0)

    local drop = overload * M.params.loadDropGain * gain
    drop = drop + underload * 0.08

    local contactDrop = 0.0

    for n = 0, 1 do
        local i = startWheel + n

        local supportLoss = clamp(1.0 - math.min(M.state.carcassSupport[i], M.state.carcassGripGate[i]), 0.0, 1.0)
        local deliveryLoss = clamp(1.0 - M.state.tireDelivery[i], 0.0, 1.0)
        local recoverySoften = clamp(1.0 - M.state.gripReturn[i], 0.0, 1.0)

        contactDrop =
            contactDrop
            + M.state.contactInput[i] * M.params.contactInputGain
            + M.state.contactLoss[i] * M.params.contactLossDropGain
            - (M.state.contactQuality[i] - 1.0) * M.params.contactQualityRecoverGain
            + M.state.memory[i] * M.params.memoryDropGain
            + M.state.hop[i] * M.params.hopDropGain
            + abs(M.state.armCamber[i]) * M.params.armCamberDropGain
            + abs(M.state.armToe[i]) * M.params.armToeDropGain
            + M.state.carcassDelay[i] * M.params.carcassDelayGain
            + M.state.carcassDeformation[i] * M.params.carcassDeformationGain
            + supportLoss * M.params.carcassSupportGain
            + M.state.snapRisk[i] * M.params.slipRecoveryDelayGain
            + M.state.roadShock[i] * M.params.roadShockDelayGain
            + M.state.pathLoss[i] * M.params.pathLossDelayGain
            + M.state.thermalStress[i] * M.params.thermalDelayGain
            + deliveryLoss * 0.10
            + recoverySoften * 0.08
            - math.max(M.state.carcassReturn[i], 0.0) * 0.04
            - math.max(M.state.recoveryRate[i], 0.0) * 0.035
            - math.max(M.state.tireDelivery[i] - 1.0, 0.0) * M.params.tireDeliveryRecoverGain
            - math.max(M.state.carcassGripGate[i] - 1.0, 0.0) * M.params.gripGateRecoverGain
    end

    drop = drop + contactDrop * 0.5

    return clamp(drop, 0.0, M.params.maxComplianceDrop)
end

local function calculateTargetCompliance(load, gain, axleIndex)
    local startWheel = axleIndex == 0 and 0 or 2
    local drop = calculateAxleDrop(startWheel, load, gain)

    if axleIndex == 1 then
        drop =
            drop
            + abs(safeLoad("ngp_drive_torque", safeLoad("ngp_drivetrain_torque", 0.0))) * M.params.driveRearDropGain
            + safeLoad("ngp_lsd_lock", safeLoad("ngp_diff_lock", 0.0)) * M.params.lsdRearDelayGain * 0.30
    end

    drop = clamp(drop, 0.0, M.params.maxComplianceDrop)

    if axleIndex == 0 then
        M.state.frontDrop = drop
    else
        M.state.rearDrop = drop
    end

    local compliance = 1.0 - drop
    return clamp(compliance, M.params.minCompliance, M.params.maxCompliance)
end

local function updateDeflectionAndDelay(dt)
    local frontTarget = clamp((1.0 - M.state.frontCompliance) * M.params.frontGain, 0.0, M.params.maxDeflection)
    local rearTarget = clamp((1.0 - M.state.rearCompliance) * M.params.rearGain, 0.0, M.params.maxDeflection)

    M.state.frontDeflection = lowPass(M.state.frontDeflection, frontTarget, M.params.deflectionTau, dt)
    M.state.rearDeflection = lowPass(M.state.rearDeflection, rearTarget, M.params.deflectionTau, dt)

    local rearDriveDelay =
        abs(safeLoad("ngp_drive_torque", safeLoad("ngp_drivetrain_torque", 0.0))) * M.params.driveRearDropGain
        + safeLoad("ngp_lsd_lock", safeLoad("ngp_diff_lock", 0.0)) * M.params.lsdRearDelayGain
        + abs(safeLoad("ngp_shaft_twist", safeLoad("ngp_windup_shaft_twist", 0.0))) * M.params.shaftDelayGain

    local carcassDelay =
        (
            M.state.carcassDelay[0] + M.state.carcassDelay[1]
            + M.state.carcassDelay[2] + M.state.carcassDelay[3]
        ) * 0.25

    local roadShock =
        (
            M.state.roadShock[0] + M.state.roadShock[1]
            + M.state.roadShock[2] + M.state.roadShock[3]
        ) * 0.25

    local pathLoss =
        (
            M.state.pathLoss[0] + M.state.pathLoss[1]
            + M.state.pathLoss[2] + M.state.pathLoss[3]
        ) * 0.25

    M.state.avgCarcassDelay = carcassDelay
    M.state.avgRoadShock = roadShock
    M.state.avgPathLoss = pathLoss

    local targetDelay =
        (M.state.frontDeflection + M.state.rearDeflection) * 0.5
        + rearDriveDelay
        + carcassDelay * M.params.carcassDelayGain
        + roadShock * M.params.roadShockDelayGain
        + pathLoss * M.params.pathLossDelayGain

    M.state.targetResponseDelay = clamp(targetDelay, M.params.minResponseDelay, M.params.maxResponseDelay)

    M.state.responseDelay = lowPass(
        M.state.responseDelay,
        M.state.targetResponseDelay,
        M.params.responseTau,
        dt
    )

    M.state.responseDelay = clamp(M.state.responseDelay, M.params.minResponseDelay, M.params.maxResponseDelay)
end

local function updateWheelDerivedOutputs()
    for i = 0, 3 do
        local axleCompliance = i < 2 and M.state.frontCompliance or M.state.rearCompliance
        local axleDeflection = i < 2 and M.state.frontDeflection or M.state.rearDeflection

        local localFlex =
            M.state.contactFlex[i] * M.params.contactFlexGain
            + M.state.carcassDelay[i] * 0.035
            + M.state.pathLoss[i] * 0.025

        M.state.compliance[i] = clamp(axleCompliance - localFlex, M.params.minCompliance, M.params.maxCompliance)
        M.state.deflection[i] = clamp(axleDeflection + localFlex, 0.0, M.params.maxDeflection)

        M.state.relaxScale[i] = clamp(
            1.0
            + M.state.responseDelay * 0.25
            + M.state.carcassDelay[i] * 0.08
            + M.state.roadShock[i] * 0.04,
            0.75,
            1.45
        )
    end
end

local function decayWhenUnavailable(dt)
    M.state.frontCompliance = lowPass(M.state.frontCompliance, 1.0, M.params.complianceTau, dt)
    M.state.rearCompliance = lowPass(M.state.rearCompliance, 1.0, M.params.complianceTau, dt)
    M.state.targetFrontCompliance = lowPass(M.state.targetFrontCompliance, 1.0, M.params.complianceTau, dt)
    M.state.targetRearCompliance = lowPass(M.state.targetRearCompliance, 1.0, M.params.complianceTau, dt)

    for i = 0, 3 do
        M.state.contactInput[i] = lowPass(M.state.contactInput[i], 0.0, M.params.contactTau, dt)
        M.state.contactFlex[i] = lowPass(M.state.contactFlex[i], 0.0, M.params.contactTau, dt)
        M.state.contactLoss[i] = lowPass(M.state.contactLoss[i], 0.0, M.params.contactTau, dt)
        M.state.contactQuality[i] = lowPass(M.state.contactQuality[i], 1.0, M.params.contactTau, dt)
        M.state.memory[i] = lowPass(M.state.memory[i], 0.0, M.params.complianceTau, dt)
        M.state.hop[i] = lowPass(M.state.hop[i], 0.0, M.params.complianceTau, dt)
        M.state.carcassDelay[i] = lowPass(M.state.carcassDelay[i], 0.0, M.params.complianceTau, dt)
        M.state.carcassDeformation[i] = lowPass(M.state.carcassDeformation[i], 0.0, M.params.complianceTau, dt)
        M.state.roadShock[i] = lowPass(M.state.roadShock[i], 0.0, M.params.complianceTau, dt)
        M.state.pathLoss[i] = lowPass(M.state.pathLoss[i], 0.0, M.params.complianceTau, dt)
        M.state.tireDelivery[i] = lowPass(M.state.tireDelivery[i], 1.0, M.params.complianceTau, dt)
    end

    updateDeflectionAndDelay(dt)
    updateWheelDerivedOutputs()
end

local function exportWheel(index)
    local axleComp = index < 2 and M.state.frontCompliance or M.state.rearCompliance
    local axleDef = index < 2 and M.state.frontDeflection or M.state.rearDeflection

    safeStore("ngp_tire_load_" .. index, M.state.load[index] or 0.0)
    safeStore("ngp_tc_comp_input_" .. index, M.state.contactInput[index] or 0.0)
    safeStore("ngp_tc_comp_flex_" .. index, M.state.contactFlex[index] or 0.0)
    safeStore("ngp_tire_relax_scale_" .. index, M.state.relaxScale[index] or 1.0)

    safeStore("ngp_tire_compliance_" .. index, axleComp or 1.0)
    safeStore("ngp_tire_deflection_" .. index, axleDef or 0.0)

    safeStore("ngp_tire_compliance_local_" .. index, M.state.compliance[index] or 1.0)
    safeStore("ngp_tire_deflection_local_" .. index, M.state.deflection[index] or 0.0)

    safeStore("ngp_tcomp_compliance_" .. index, M.state.compliance[index] or 1.0)
    safeStore("ngp_tcomp_deflection_" .. index, M.state.deflection[index] or 0.0)
    safeStore("ngp_tcomp_relax_" .. index, M.state.relaxScale[index] or 1.0)

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_tire_compliance_contact_quality_" .. index, M.state.contactQuality[index] or 1.0)
    safeStore("ngp_tire_compliance_contact_loss_" .. index, M.state.contactLoss[index] or 0.0)
    safeStore("ngp_tire_compliance_memory_" .. index, M.state.memory[index] or 0.0)
    safeStore("ngp_tire_compliance_hop_" .. index, M.state.hop[index] or 0.0)
    safeStore("ngp_tire_compliance_carcass_delay_" .. index, M.state.carcassDelay[index] or 0.0)
    safeStore("ngp_tire_compliance_path_loss_" .. index, M.state.pathLoss[index] or 0.0)
    safeStore("ngp_tire_compliance_road_shock_" .. index, M.state.roadShock[index] or 0.0)
    safeStore("ngp_tire_compliance_tire_delivery_" .. index, M.state.tireDelivery[index] or 1.0)
end

local function exportState()
    safeStore("ngp_tire_compliance_status", M.state.status or "UNKNOWN")
    safeStore("ngp_tire_compliance_update_count", M.state.updateCount or 0)
    safeStore("ngp_tire_compliance_wheels_valid", M.state.wheelsValid and 1 or 0)
    safeStore("ngp_tire_compliance_store_only", M.state.storeOnly and 1 or 0)
    safeStore("ngp_tire_compliance_load_linked", M.state.loadLinked and 1 or 0)

    safeStore("ngp_tire_comp_front", M.state.frontCompliance or 1.0)
    safeStore("ngp_tire_comp_rear", M.state.rearCompliance or 1.0)

    safeStore("ngp_tire_def_front", M.state.frontDeflection or 0.0)
    safeStore("ngp_tire_def_rear", M.state.rearDeflection or 0.0)

    safeStore("ngp_tire_response_delay", M.state.responseDelay or 0.0)

    safeStore("ngp_tire_front_load", M.state.frontLoad or 0.0)
    safeStore("ngp_tire_rear_load", M.state.rearLoad or 0.0)

    safeStore("ngp_tire_target_comp_front", M.state.targetFrontCompliance or 1.0)
    safeStore("ngp_tire_target_comp_rear", M.state.targetRearCompliance or 1.0)

    safeStore("ngp_tyre_comp_front", M.state.frontCompliance or 1.0)
    safeStore("ngp_tyre_comp_rear", M.state.rearCompliance or 1.0)
    safeStore("ngp_tyre_response_delay", M.state.responseDelay or 0.0)

    safeStore("ngp_tcomp_front", M.state.frontCompliance or 1.0)
    safeStore("ngp_tcomp_rear", M.state.rearCompliance or 1.0)
    safeStore("ngp_tcomp_delay", M.state.responseDelay or 0.0)
    safeStore("ngp_tcomp_front_def", M.state.frontDeflection or 0.0)
    safeStore("ngp_tcomp_rear_def", M.state.rearDeflection or 0.0)

    safeStore("ngp_tire_compliance_front_drop", M.state.frontDrop or 0.0)
    safeStore("ngp_tire_compliance_rear_drop", M.state.rearDrop or 0.0)
    safeStore("ngp_tire_compliance_avg_contact", M.state.avgContactInput or 0.0)
    safeStore("ngp_tire_compliance_avg_loss", M.state.avgContactLoss or 0.0)
    safeStore("ngp_tire_compliance_avg_carcass_delay", M.state.avgCarcassDelay or 0.0)
    safeStore("ngp_tire_compliance_avg_path_loss", M.state.avgPathLoss or 0.0)

    for i = 0, 3 do
        exportWheel(i)
    end

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_tire_compliance_contact_linked", M.state.contactLinked and 1 or 0)
    safeStore("ngp_tire_compliance_memory_linked", M.state.memoryLinked and 1 or 0)
    safeStore("ngp_tire_compliance_hop_linked", M.state.hopLinked and 1 or 0)
    safeStore("ngp_tire_compliance_arm_linked", M.state.armLinked and 1 or 0)
    safeStore("ngp_tire_compliance_drivetrain_linked", M.state.drivetrainLinked and 1 or 0)
    safeStore("ngp_tire_compliance_lsd_linked", M.state.lsdLinked and 1 or 0)
    safeStore("ngp_tire_compliance_carcass_linked", M.state.carcassLinked and 1 or 0)
    safeStore("ngp_tire_compliance_recovery_linked", M.state.recoveryLinked and 1 or 0)
    safeStore("ngp_tire_compliance_road_linked", M.state.roadLinked and 1 or 0)
    safeStore("ngp_tire_compliance_load_path_linked", M.state.loadPathLinked and 1 or 0)
    safeStore("ngp_tire_compliance_thermal_linked", M.state.thermalLinked and 1 or 0)
end

local function updateCore(dt, car)
    resetLinkFlags()

    for i = 0, 3 do
        readWheelRootInput(i, dt)
    end

    local frontLoad, rearLoad = readAxleLoads(car)

    M.state.frontLoad = frontLoad
    M.state.rearLoad = rearLoad

    local targetFront = calculateTargetCompliance(frontLoad, M.params.frontGain, 0)
    local targetRear = calculateTargetCompliance(rearLoad, M.params.rearGain, 1)

    M.state.targetFrontCompliance = targetFront
    M.state.targetRearCompliance = targetRear

    M.state.frontCompliance = lowPass(
        M.state.frontCompliance,
        targetFront,
        M.params.complianceTau,
        dt
    )

    M.state.rearCompliance = lowPass(
        M.state.rearCompliance,
        targetRear,
        M.params.complianceTau,
        dt
    )

    M.state.frontCompliance = clamp(M.state.frontCompliance, M.params.minCompliance, M.params.maxCompliance)
    M.state.rearCompliance = clamp(M.state.rearCompliance, M.params.minCompliance, M.params.maxCompliance)

    local sumContact = 0.0
    local sumLoss = 0.0
    for i = 0, 3 do
        sumContact = sumContact + (M.state.contactInput[i] or 0.0)
        sumLoss = sumLoss + (M.state.contactLoss[i] or 0.0)
    end

    M.state.avgContactInput = sumContact * 0.25
    M.state.avgContactLoss = sumLoss * 0.25

    updateDeflectionAndDelay(dt)
    updateWheelDerivedOutputs()
end

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

    dt = clamp(dt, M.params.dtMin, M.params.dtMax)

    updateDebugGate(dt)

    if not car then
        car = safeGetCar()
    end

    local wheels = getWheels(car)

    if not car then
        M.state.status = "STORE ONLY"
        M.state.wheelsValid = false
        M.state.storeOnly = true
        updateCore(dt, nil)
        exportState()
        return
    end

    if not wheels then
        M.state.status = "NO WHEELS"
        M.state.wheelsValid = false
        M.state.storeOnly = true
        updateCore(dt, nil)
        exportState()
        return
    end

    M.state.status = "RUNNING"
    M.state.wheelsValid = true
    M.state.storeOnly = false

    updateCore(dt, car)
    exportState()
end

function M.getFrontCompliance()
    return M.state.frontCompliance or 1.0
end

function M.getRearCompliance()
    return M.state.rearCompliance or 1.0
end

function M.getResponseDelay()
    return M.state.responseDelay or 0.0
end

function M.getFrontDeflection()
    return M.state.frontDeflection or 0.0
end

function M.getRearDeflection()
    return M.state.rearDeflection or 0.0
end

function M.getCompliance(index)
    if index == nil then
        return {
            front = M.state.frontCompliance or 1.0,
            rear = M.state.rearCompliance or 1.0,
        }
    end

    return M.state.compliance[index] or (index < 2 and M.state.frontCompliance or M.state.rearCompliance) or 1.0
end

function M.getDeflection(index)
    if index == nil then
        return {
            front = M.state.frontDeflection or 0.0,
            rear = M.state.rearDeflection or 0.0,
        }
    end

    return M.state.deflection[index] or (index < 2 and M.state.frontDeflection or M.state.rearDeflection) or 0.0
end

function M.getLoad(index)
    if index == nil then
        return M.state.frontLoad or 0.0, M.state.rearLoad or 0.0
    end

    return M.state.load[index] or 0.0
end

function M.getState(index)
    if index == nil then
        return M.state
    end

    return {
        name = WHEEL_NAMES[index] or tostring(index),
        load = M.state.load[index] or 0.0,
        contactInput = M.state.contactInput[index] or 0.0,
        contactFlex = M.state.contactFlex[index] or 0.0,
        contactLoss = M.state.contactLoss[index] or 0.0,
        contactQuality = M.state.contactQuality[index] or 1.0,
        compliance = M.state.compliance[index] or 1.0,
        deflection = M.state.deflection[index] or 0.0,
        relaxScale = M.state.relaxScale[index] or 1.0,
        carcassDelay = M.state.carcassDelay[index] or 0.0,
        pathLoss = M.state.pathLoss[index] or 0.0,
        roadShock = M.state.roadShock[index] or 0.0,
    }
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f / Wheels %s / Store %s\n" ..
        "FrontLoad %.0f Comp %.3f Target %.3f Def %.3f Drop %.3f\n" ..
        "RearLoad  %.0f Comp %.3f Target %.3f Def %.3f Drop %.3f\n" ..
        "Delay %.3f Target %.3f Contact %.3f Loss %.3f Path %.3f\n" ..
        "Links CT:%s MEM:%s HOP:%s ARM:%s DT:%s LSD:%s CAR:%s SR:%s ROAD:%s LP:%s TH:%s",
        tostring(M.state.status),
        M.state.updateCount or 0,
        M.state.wheelsValid and "OK" or "NIL",
        M.state.storeOnly and "YES" or "NO",

        M.state.frontLoad or 0.0,
        M.state.frontCompliance or 1.0,
        M.state.targetFrontCompliance or 1.0,
        M.state.frontDeflection or 0.0,
        M.state.frontDrop or 0.0,

        M.state.rearLoad or 0.0,
        M.state.rearCompliance or 1.0,
        M.state.targetRearCompliance or 1.0,
        M.state.rearDeflection or 0.0,
        M.state.rearDrop or 0.0,

        M.state.responseDelay or 0.0,
        M.state.targetResponseDelay or 0.0,
        M.state.avgContactInput or 0.0,
        M.state.avgContactLoss or 0.0,
        M.state.avgPathLoss or 0.0,

        M.state.contactLinked and "OK" or "NIL",
        M.state.memoryLinked and "OK" or "NIL",
        M.state.hopLinked and "OK" or "NIL",
        M.state.armLinked and "OK" or "NIL",
        M.state.drivetrainLinked and "OK" or "NIL",
        M.state.lsdLinked and "OK" or "NIL",
        M.state.carcassLinked and "OK" or "NIL",
        M.state.recoveryLinked and "OK" or "NIL",
        M.state.roadLinked and "OK" or "NIL",
        M.state.loadPathLinked and "OK" or "NIL",
        M.state.thermalLinked and "OK" or "NIL"
    )
end

return M
