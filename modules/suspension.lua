---@diagnostic disable: undefined-global

--============================================================
-- ACNextGen
-- suspension.lua
-- V1.1.5 stable
-- Suspension Compliance + Road Input Integration
--============================================================

local M = {}

local WHEEL_COUNT = 4

M.params = {
    damperScale = 600.0,
    maxForce = 1200.0,
    minSuspSpeed = 0.002,

    slowBumpScale = 600.0,
    fastBumpScale = 1200.0,
    reboundScale  = 900.0,
    bumpThreshold = 0.15,

    rollBarScale = 250.0,
    frontARB = 3000.0,
    rearARB  = 3000.0,
    arbTau = 0.080,

    camberScale = 80.0,
    toeScale = 120.0,
    armFlexScale = 0.0008,
    bushTau = 0.080,

    casterAngle = 6.5,
    casterTrail = 0.035,
    casterScale = 1.2,
    casterTau = 0.060,

    springRate = { [0] = 35000.0, [1] = 35000.0, [2] = 45000.0, [3] = 45000.0 },
    springLimit = 8000.0,

    tireHopGain = 0.15,
    loadBiasGain = 120.0,

    externalDamperGain = 0.055,
    externalSpringGain = 0.032,
    contactForceGain = 0.12,
    impulseForceGain = 0.10,
    contactDropLoss = 0.10,
    damperScaleInfluence = 0.08,
    springScaleInfluence = 0.06,
    integratedForceTau = 0.045,
    minIntegratedScale = 0.82,
    maxIntegratedScale = 1.22,

    tireRoadInputGain = 0.14,
    tireRoadImpulseGain = 0.08,
    tireSlipEnergyGain = 0.08,
    tireCombinedSlipGain = 0.06,
    tireRoadForceGain = 72.0,
    tireLatLongForceReference = 2200.0,
    tireRoadInputTau = 0.040,
    minTireRoadInput = 0.0,
    maxTireRoadInput = 1.50,

    bodyRigidityInfluence = 0.16,
    bodyFlexForceLoss = 0.10,
    bodyTorsionForceLoss = 0.08,
    bodyBendingForceLoss = 0.08,
    bodyFlexTauGain = 0.45,
    bodySoftContactGain = 0.08,
    bodyStiffSharpGain = 0.06,
    minBodyRigidityScale = 0.86,
    maxBodyRigidityScale = 1.10,

    loadRef = 3000.0,
    noCarDecayTau = 0.180,
    minDt = 0.00005,
    maxDt = 0.050,
    maxSuspSpeed = 12.0,
    debugExportInterval = 0.25,
}

local prevSusp = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 }
local initialized = false

local arbState = { front = 0.0, rear = 0.0 }
local casterState = { force = 0.0 }
local suspensionBuffer = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 }
local forceStoreBuffer = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 }

local state = {
    suspTravel = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    suspSpeed = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    damperForce = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    force = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    springForce = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    camber = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    toe = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    caster = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    arbForce = { front = 0.0, rear = 0.0 },
    casterTotal = 0.0,

    wheelLoad = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadFront = 0.5,
    loadRear = 0.5,
    loadLeft = 0.5,
    loadRight = 0.5,

    armCamber = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    armToe = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireHop = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    externalDamper = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    externalSpring = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    contactInput = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    contactImpulse = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    contactDrop = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    tireRoadInput = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireRoadImpulse = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireCombinedSlip = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireSlipEnergy = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireForceMagnitude = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    roadImpact = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    roadShock = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadPathLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    integratedScale = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    integratedForce = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    bodyRigidityScale = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyBendingFactor = 1.0,
    bodyDampingFactor = 1.0,

    avgForce = 0.0,
    avgAbsForce = 0.0,
    avgIntegratedScale = 1.0,
    avgTireRoadInput = 0.0,
    avgTireSlipEnergy = 0.0,
    maxAbsForce = 0.0,

    loadLinked = false,
    armLinked = false,
    hopLinked = false,
    externalLinked = false,
    contactLinked = false,
    tireForceLinked = false,
    bodyRigidityLinked = false,
    roadInputLinked = false,
    loadPathLinked = false,

    wheelsValid = false,
    status = "INIT",
    updateCount = 0,
    debugExportTimer = 999.0,
    debugExportNow = true,
    bodyReadTimer = 999.0,
}

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
    if tau <= 0.0001 then return target end
    return current + (target - current) * clamp(dt / math.max(tau + dt, 0.0001), 0.0, 1.0)
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
    if defaultValue == nil then
        return nil, nil
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
    if ok then return car end
    return nil
end

local function getWheels(car)
    return safeField(car, "wheels", nil)
end

local function getWheel(car, index)
    local wheels = getWheels(car)
    if not wheels then return nil end
    local ok, wheel = pcall(function() return wheels[index] end)
    if ok and wheel then return wheel end
    ok, wheel = pcall(function() return wheels[index + 1] end)
    if ok then return wheel end
    return nil
end

local function updateDebugExportGate(dt)
    state.debugExportTimer = (state.debugExportTimer or 0.0) + (dt or 0.0)
    if state.debugExportTimer >= (M.params.debugExportInterval or 0.25) then
        state.debugExportTimer = 0.0
        state.debugExportNow = true
    else
        state.debugExportNow = false
    end
end

local function readWheelTravel(wheel, fallback)
    if wheel then
        local keys = {
            "suspensionTravel",
            "suspensionTravelM",
            "suspensionLength",
            "travel",
            "damperTravel",
            "suspensionPosition",
        }
        for k = 1, #keys do
            local value = safeField(wheel, keys[k], nil)
            if value ~= nil then return safeNumber(value, fallback or 0.0) end
        end
    end
    return fallback or 0.0
end

local function readSuspensionTravel(car, index)
    local wheel = getWheel(car, index)
    local travel = readWheelTravel(wheel, nil)
    if travel ~= nil then return travel end

    if physics and physics.getExtendedSuspensionTravel then
        local ok, value = pcall(function() return physics.getExtendedSuspensionTravel(0, index) end)
        if ok and type(value) == "number" then return value end
    end

    return prevSusp[index] or 0.0
end

local function initializeSuspension(car)
    for i = 0, 3 do
        prevSusp[i] = readSuspensionTravel(car, i)
        state.suspTravel[i] = prevSusp[i]
        state.suspSpeed[i] = 0.0
        state.integratedForce[i] = state.force[i] or 0.0
    end
    initialized = true
end

local function readBodyRigidity(dt)
    state.bodyReadTimer = (state.bodyReadTimer or 999.0) + (dt or 0.0)
    if state.bodyRigidityLinked and state.bodyReadTimer < 0.25 then return end
    state.bodyReadTimer = 0.0

    local rigidityRaw = safeLoadRaw("ngp_body_rigidity")
    local updateCount = safeLoadRaw("ngp_body_rigidity_update_count")

    state.bodyRigidityLinked = rigidityRaw ~= nil or safeNumber(updateCount, 0.0) > 0.0
    state.bodyRigidity = clamp(safeNumber(rigidityRaw, 1.0), 0.20, 1.35)
    state.bodyFlexFactor = clamp(safeLoad("ngp_body_flex_factor", 1.0), 0.25, 1.80)
    state.bodyTorsionFactor = clamp(safeLoad("ngp_body_torsion_factor", 1.0), 0.35, 2.00)
    state.bodyBendingFactor = clamp(safeLoad("ngp_body_bending_factor", 1.0), 0.35, 2.00)
    state.bodyDampingFactor = clamp(safeLoad("ngp_body_damping_factor", 1.0), 0.50, 1.80)
end

local function readLoadState()
    local front, frontKey = loadAlt(0.5, "ngp_load_front", "ngp_front_bias", "ngp_hub_weight_front")
    local rear, rearKey = loadAlt(0.5, "ngp_load_rear", "ngp_rear_bias", "ngp_hub_weight_rear")
    local left, leftKey = loadAlt(0.5, "ngp_load_left", "ngp_left_bias", "ngp_hub_weight_left")
    local right, rightKey = loadAlt(0.5, "ngp_load_right", "ngp_right_bias", "ngp_hub_weight_right")

    state.loadFront = clamp(front, 0.0, 1.0)
    state.loadRear = clamp(rear, 0.0, 1.0)
    state.loadLeft = clamp(left, 0.0, 1.0)
    state.loadRight = clamp(right, 0.0, 1.0)
    state.loadLinked = frontKey ~= nil or rearKey ~= nil or leftKey ~= nil or rightKey ~= nil

    for i = 0, 3 do
        local load, key = loadAlt(nil,
            "ngp_wheel_load_" .. i,
            "ngp_load_wheel_" .. i,
            "ngp_hub_load_" .. i,
            "ngp_contact_load_" .. i,
            "ngp_load_path_load_" .. i
        )

        if key == nil then
            local dlt = safeLoadRaw("ngp_dlt_load_" .. i)
            if dlt ~= nil then
                local n = safeNumber(dlt, 0.0)
                if math.abs(n) < 1800.0 then
                    load = M.params.loadRef + n
                else
                    load = n
                end
                key = "ngp_dlt_load_" .. i
            end
        end

        if key ~= nil then state.loadLinked = true end
        state.wheelLoad[i] = clamp(safeNumber(load, M.params.loadRef), 0.0, 12000.0)
    end
end

local function readArmState()
    local linked = false
    for i = 0, 3 do
        local camber, camberKey = loadAlt(0.0,
            "ngp_control_arm_camber_" .. i,
            "ngp_ca_camber_" .. i,
            "ngp_arm_camber_" .. i,
            "ngp_arm_compliance_camber_" .. i
        )
        local toe, toeKey = loadAlt(0.0,
            "ngp_control_arm_toe_" .. i,
            "ngp_ca_toe_" .. i,
            "ngp_arm_toe_" .. i,
            "ngp_arm_compliance_toe_" .. i
        )
        if camberKey or toeKey then linked = true end
        state.armCamber[i] = camber
        state.armToe[i] = toe
    end
    state.armLinked = linked
end

local function readTireHopState()
    local linked = false
    for i = 0, 3 do
        local hop, key = loadAlt(0.0,
            "ngp_tire_hop_energy_" .. i,
            "ngp_tire_hop_" .. i,
            "ngp_tirehop_" .. i,
            "ngp_hub_hop_" .. i
        )
        if key then linked = true end
        state.tireHop[i] = clamp(hop, 0.0, 1.2)
    end
    state.hopLinked = linked
end

local function readTireForceState(dt)
    local linked = false
    local sumRoad = 0.0
    local sumEnergy = 0.0

    for i = 0, 3 do
        local roadInput, roadKey = loadAlt(nil,
            "ngp_tire_force_road_input_" .. i,
            "ngp_tf_road_input_" .. i,
            "ngp_road_input_vertical_" .. i,
            "ngp_rii_impact_" .. i,
            "ngp_road_impact_" .. i
        )
        local combinedForce, combinedKey = loadAlt(nil,
            "ngp_tire_force_combined_" .. i,
            "ngp_tf_combined_" .. i
        )
        local latForce, latKey = loadAlt(nil,
            "ngp_tire_force_lat_" .. i,
            "ngp_tf_lat_" .. i,
            "ngp_lat_" .. i,
            "ngp_hub_force_lat_" .. i
        )
        local longForce, longKey = loadAlt(nil,
            "ngp_tire_force_long_" .. i,
            "ngp_tf_long_" .. i,
            "ngp_long_" .. i,
            "ngp_hub_force_long_" .. i
        )
        local combinedSlip, slipKey = loadAlt(0.0,
            "ngp_tire_force_combined_slip_" .. i,
            "ngp_tdyn_combined_slip_" .. i,
            "ngp_contact_combined_slip_" .. i,
            "ngp_slip_recovery_slip_" .. i
        )
        local slipEnergy, energyKey = loadAlt(0.0,
            "ngp_tire_force_slip_energy_" .. i,
            "ngp_tdyn_slip_energy_" .. i,
            "ngp_slip_slide_memory_" .. i,
            "ngp_rii_surface_limit_" .. i
        )
        local roadShock, shockKey = loadAlt(0.0,
            "ngp_rii_shock_" .. i,
            "ngp_road_shock_" .. i,
            "ngp_impact_wheel_" .. i
        )

        if roadKey or combinedKey or latKey or longKey or slipKey or energyKey or shockKey then linked = true end

        latForce = safeNumber(latForce, 0.0)
        longForce = safeNumber(longForce, 0.0)
        local fallbackMagnitude = math.sqrt(latForce * latForce + longForce * longForce)
        local magnitude = safeNumber(combinedForce, fallbackMagnitude)
        local fallbackRoad = magnitude / math.max(M.params.tireLatLongForceReference, 1.0)

        roadInput = clamp(safeNumber(roadInput, fallbackRoad) + roadShock * 0.16, M.params.minTireRoadInput, M.params.maxTireRoadInput)
        combinedSlip = clamp(combinedSlip, 0.0, 2.50)
        slipEnergy = clamp(slipEnergy, 0.0, 1.20)

        local prevRoad = state.tireRoadInput[i] or 0.0
        local filteredRoad = lowPass(prevRoad, roadInput, M.params.tireRoadInputTau, dt)
        state.tireRoadImpulse[i] = clamp(abs(roadInput - prevRoad), 0.0, 1.0)
        state.tireRoadInput[i] = filteredRoad
        state.tireCombinedSlip[i] = combinedSlip
        state.tireSlipEnergy[i] = slipEnergy
        state.tireForceMagnitude[i] = magnitude
        state.roadShock[i] = roadShock

        sumRoad = sumRoad + filteredRoad
        sumEnergy = sumEnergy + slipEnergy
    end

    state.tireForceLinked = linked
    state.avgTireRoadInput = sumRoad * 0.25
    state.avgTireSlipEnergy = sumEnergy * 0.25
end

local function readRoadAndLoadPathState()
    local roadLinked = false
    local pathLinked = false
    for i = 0, 3 do
        local impact, impactKey = loadAlt(0.0, "ngp_rii_impact_" .. i, "ngp_road_impact_" .. i)
        local shock, shockKey = loadAlt(0.0, "ngp_rii_shock_" .. i, "ngp_road_shock_" .. i)
        local loss, lossKey = loadAlt(0.0, "ngp_load_path_loss_" .. i, "ngp_lp_loss_" .. i, "ngp_rii_path_loss_" .. i)
        if impactKey or shockKey then roadLinked = true end
        if lossKey then pathLinked = true end
        state.roadImpact[i] = clamp(impact, 0.0, 1.2)
        state.roadShock[i] = math.max(state.roadShock[i] or 0.0, clamp(shock, 0.0, 1.2))
        state.loadPathLoss[i] = clamp(loss, 0.0, 1.2)
    end
    state.roadInputLinked = roadLinked
    state.loadPathLinked = pathLinked
end

local function readExternalState(dt)
    readLoadState()
    readArmState()
    readTireHopState()
    readTireForceState(dt)
    readRoadAndLoadPathState()
end

local function calculateDamperForce(speed)
    local absSpeed = abs(speed)
    if absSpeed < M.params.minSuspSpeed then return 0.0 end
    if speed > 0.0 then
        local scale = absSpeed < M.params.bumpThreshold and M.params.slowBumpScale or M.params.fastBumpScale
        return -speed * scale
    end
    return -speed * M.params.reboundScale
end

local function calculateSpringForce(index, travel)
    local rate = M.params.springRate[index] or 35000.0
    return clamp(-travel * rate, -M.params.springLimit, M.params.springLimit)
end

local function applyTireHop(index, force)
    return force + (state.tireHop[index] or 0.0) * M.params.tireHopGain
end

local function applyLoadBias(index, force)
    local bias = 0.0
    if index < 2 then
        bias = safeNumber(state.loadFront, 0.5) - 0.5
    else
        bias = safeNumber(state.loadRear, 0.5) - 0.5
    end
    return force - bias * M.params.loadBiasGain
end

local function updateWheelForces(suspension, dt)
    local store = forceStoreBuffer
    for i = 0, 3 do
        local speed = (suspension[i] - (prevSusp[i] or suspension[i])) / math.max(dt, 0.001)
        speed = clamp(speed, -M.params.maxSuspSpeed, M.params.maxSuspSpeed)

        local damperForce = calculateDamperForce(speed)
        local springForce = calculateSpringForce(i, suspension[i])
        local force = damperForce + springForce
        force = applyTireHop(i, force)
        force = applyLoadBias(i, force)
        force = clamp(force, -M.params.maxForce, M.params.maxForce)

        state.suspTravel[i] = suspension[i]
        state.suspSpeed[i] = speed
        state.damperForce[i] = damperForce
        state.springForce[i] = springForce
        state.force[i] = force
        store[i] = force
        prevSusp[i] = suspension[i]
    end
    return store
end

local function applyAntiRollBar(suspension, store, dt)
    local frontTwist = (suspension[0] or 0.0) - (suspension[1] or 0.0)
    local rearTwist = (suspension[2] or 0.0) - (suspension[3] or 0.0)

    arbState.front = lowPass(arbState.front, frontTwist, M.params.arbTau, dt)
    arbState.rear = lowPass(arbState.rear, rearTwist, M.params.arbTau, dt)

    local rollFront = arbState.front * M.params.frontARB
    local rollRear = arbState.rear * M.params.rearARB
    state.arbForce.front = rollFront
    state.arbForce.rear = rollRear

    store[0] = (store[0] or 0.0) - rollFront
    store[1] = (store[1] or 0.0) + rollFront
    store[2] = (store[2] or 0.0) - rollRear
    store[3] = (store[3] or 0.0) + rollRear

    return rollFront, rollRear
end

local function updateCaster(car, dt)
    local steer = safeNumber(safeField(car, "steer", 0.0), 0.0)
    local casterAssist = safeLoad("ngp_caster_aligning_torque", 0.0) * 0.001
    local target = math.rad(M.params.casterAngle) * steer * M.params.casterTrail + casterAssist
    casterState.force = lowPass(casterState.force, target, M.params.casterTau, dt)
    state.casterTotal = casterState.force
end

local function updateAlignment(index, travel)
    local camber = -travel * 0.01 * M.params.camberScale + (state.armCamber[index] or 0.0)
    local toe = travel * 0.005 * M.params.toeScale + (state.armToe[index] or 0.0)
    local caster = index < 2 and casterState.force * M.params.casterScale or 0.0

    state.camber[index] = camber
    state.toe[index] = toe
    state.caster[index] = caster
    return camber, toe, caster
end

local function integrateExternalSuspension(index, baseForce, dt)
    local p = M.params

    local damperForce, damperKey = loadAlt(0.0,
        "ngp_damper_" .. index,
        "ngp_damper_force_" .. index,
        "ngp_damper_hyst_force_" .. index,
        "ngp_damper_vertical_force_" .. index
    )
    local springForce, springKey = loadAlt(0.0,
        "ngp_progressive_spring_" .. index,
        "ngp_spring_force_" .. index
    )
    local damperScale = loadAlt(1.0, "ngp_damper_coeff_scale_" .. index, "ngp_damper_hyst_coeff_" .. index)
    local springScale = loadAlt(1.0, "ngp_spring_rate_scale_" .. index)

    local contactInput, contactKey = loadAlt(0.0,
        "ngp_susp_contact_input_" .. index,
        "ngp_sci_input_" .. index,
        "ngp_contact_quality_input_" .. index
    )
    local contactImpulse, impulseKey = loadAlt(0.0,
        "ngp_susp_road_impulse_" .. index,
        "ngp_sci_impulse_" .. index,
        "ngp_damper_vertical_pulse_" .. index,
        "ngp_impact_wheel_" .. index
    )
    local contactDrop, dropKey = loadAlt(0.0,
        "ngp_susp_contact_drop_" .. index,
        "ngp_sci_drop_" .. index,
        "ngp_contact_loss_" .. index
    )

    state.externalLinked = damperKey ~= nil or springKey ~= nil
    state.contactLinked = contactKey ~= nil or impulseKey ~= nil or dropKey ~= nil

    local tireRoadInput = clamp(state.tireRoadInput[index] or 0.0, p.minTireRoadInput, p.maxTireRoadInput)
    local tireRoadImpulse = clamp(state.tireRoadImpulse[index] or 0.0, 0.0, 1.0)
    local tireCombinedSlip = clamp(state.tireCombinedSlip[index] or 0.0, 0.0, 2.50)
    local tireSlipEnergy = clamp(state.tireSlipEnergy[index] or 0.0, 0.0, 1.20)
    local roadShock = clamp(state.roadShock[index] or 0.0, 0.0, 1.20)
    local loadPathLoss = clamp(state.loadPathLoss[index] or 0.0, 0.0, 1.20)

    contactInput = clamp(contactInput + tireRoadInput * p.tireRoadInputGain + roadShock * 0.08, 0.0, 1.50)
    contactImpulse = clamp(contactImpulse + tireRoadImpulse * p.tireRoadImpulseGain + tireSlipEnergy * 0.04 + roadShock * 0.05, 0.0, 1.50)
    contactDrop = clamp(contactDrop + loadPathLoss * 0.08, 0.0, 1.0)
    damperScale = clamp(damperScale, 0.50, 1.60)
    springScale = clamp(springScale, 0.50, 1.60)

    local tireRoadForce = tireRoadInput * p.tireRoadForceGain
    local externalForce = damperForce * p.externalDamperGain + springForce * p.externalSpringGain + tireRoadForce
    local tireLimitInput = math.max(tireCombinedSlip - 1.0, 0.0)

    local integratedScale = 1.0
        + contactInput * p.contactForceGain
        + contactImpulse * p.impulseForceGain
        - contactDrop * p.contactDropLoss
        + tireRoadInput * p.tireRoadInputGain
        + tireRoadImpulse * p.tireRoadImpulseGain
        + tireLimitInput * p.tireCombinedSlipGain
        - tireSlipEnergy * p.tireSlipEnergyGain
        - loadPathLoss * 0.04
        + (damperScale - 1.0) * p.damperScaleInfluence
        + (springScale - 1.0) * p.springScaleInfluence

    integratedScale = clamp(integratedScale, p.minIntegratedScale, p.maxIntegratedScale)

    local targetForce = clamp((baseForce + externalForce) * integratedScale, -p.maxForce, p.maxForce)
    local finalForce = lowPass(state.integratedForce[index] or baseForce, targetForce, p.integratedForceTau, dt)
    finalForce = clamp(finalForce, -p.maxForce, p.maxForce)

    state.externalDamper[index] = damperForce
    state.externalSpring[index] = springForce
    state.contactInput[index] = contactInput
    state.contactImpulse[index] = contactImpulse
    state.contactDrop[index] = contactDrop
    state.integratedScale[index] = integratedScale
    state.integratedForce[index] = finalForce

    return finalForce
end

local function applyBodyRigidityToSuspension(index, baseForce, dt)
    if not state.bodyRigidityLinked then
        state.bodyRigidityScale[index] = lowPass(state.bodyRigidityScale[index] or 1.0, 1.0, M.params.integratedForceTau, dt)
        return baseForce
    end

    local flex = clamp(state.bodyFlexFactor or 1.0, 0.25, 1.80)
    local torsion = clamp(state.bodyTorsionFactor or 1.0, 0.35, 2.00)
    local bending = clamp(state.bodyBendingFactor or 1.0, 0.35, 2.00)
    local rigidity = clamp(state.bodyRigidity or 1.0, 0.20, 1.35)

    local axleBias = (index == 0 or index == 1) and bending or torsion
    local softness = clamp(flex - 1.0, 0.0, 0.80)
    local stiffness = clamp(rigidity - 1.0, 0.0, 0.35)

    local forceLoss = softness * M.params.bodyFlexForceLoss
        + math.max(torsion - 1.0, 0.0) * M.params.bodyTorsionForceLoss
        + math.max(bending - 1.0, 0.0) * M.params.bodyBendingForceLoss

    local scaleTarget = 1.0 - forceLoss * axleBias + stiffness * M.params.bodyStiffSharpGain
    scaleTarget = clamp(scaleTarget, M.params.minBodyRigidityScale, M.params.maxBodyRigidityScale)

    local tau = M.params.integratedForceTau * (1.0 + softness * M.params.bodyFlexTauGain)
    state.bodyRigidityScale[index] = lowPass(state.bodyRigidityScale[index] or 1.0, scaleTarget, tau, dt)
    return baseForce * (state.bodyRigidityScale[index] or 1.0)
end

local function decayWheel(index, dt)
    state.suspSpeed[index] = lowPass(state.suspSpeed[index] or 0.0, 0.0, M.params.noCarDecayTau, dt)
    state.damperForce[index] = lowPass(state.damperForce[index] or 0.0, 0.0, M.params.noCarDecayTau, dt)
    state.force[index] = lowPass(state.force[index] or 0.0, 0.0, M.params.noCarDecayTau, dt)
    state.springForce[index] = lowPass(state.springForce[index] or 0.0, 0.0, M.params.noCarDecayTau, dt)
    state.integratedForce[index] = lowPass(state.integratedForce[index] or 0.0, 0.0, M.params.noCarDecayTau, dt)
    state.integratedScale[index] = lowPass(state.integratedScale[index] or 1.0, 1.0, M.params.noCarDecayTau, dt)
    state.bodyRigidityScale[index] = lowPass(state.bodyRigidityScale[index] or 1.0, 1.0, M.params.noCarDecayTau, dt)
    state.contactInput[index] = lowPass(state.contactInput[index] or 0.0, 0.0, M.params.noCarDecayTau, dt)
    state.contactImpulse[index] = lowPass(state.contactImpulse[index] or 0.0, 0.0, M.params.noCarDecayTau, dt)
    state.contactDrop[index] = lowPass(state.contactDrop[index] or 0.0, 0.0, M.params.noCarDecayTau, dt)
    state.tireRoadInput[index] = lowPass(state.tireRoadInput[index] or 0.0, 0.0, M.params.noCarDecayTau, dt)
    state.tireRoadImpulse[index] = lowPass(state.tireRoadImpulse[index] or 0.0, 0.0, M.params.noCarDecayTau, dt)
    state.tireCombinedSlip[index] = lowPass(state.tireCombinedSlip[index] or 0.0, 0.0, M.params.noCarDecayTau, dt)
    state.tireSlipEnergy[index] = lowPass(state.tireSlipEnergy[index] or 0.0, 0.0, M.params.noCarDecayTau, dt)
end

local function updateAverages()
    local sumForce = 0.0
    local sumAbs = 0.0
    local sumScale = 0.0
    local maxAbs = 0.0
    for i = 0, 3 do
        local f = state.force[i] or 0.0
        sumForce = sumForce + f
        sumAbs = sumAbs + abs(f)
        sumScale = sumScale + (state.integratedScale[i] or 1.0)
        maxAbs = math.max(maxAbs, abs(f))
    end
    state.avgForce = sumForce * 0.25
    state.avgAbsForce = sumAbs * 0.25
    state.avgIntegratedScale = sumScale * 0.25
    state.maxAbsForce = maxAbs
end

local function exportWheel(index, force, camber, toe, caster)
    safeStore("ngp_susp_" .. index, force)
    safeStore("ngp_susp_int_force_" .. index, state.integratedForce[index] or force or 0.0)
    safeStore("ngp_susp_integrated_force_" .. index, state.integratedForce[index] or force or 0.0)
    safeStore("ngp_susp_int_scale_" .. index, state.integratedScale[index] or 1.0)
    safeStore("ngp_susp_integrated_scale_" .. index, state.integratedScale[index] or 1.0)

    safeStore("ngp_camber_" .. index, camber)
    safeStore("ngp_toe_" .. index, toe)
    safeStore("ngp_caster_" .. index, caster)

    safeStore("ngp_su_force_" .. index, force)
    safeStore("ngp_su_scale_" .. index, state.integratedScale[index] or 1.0)
    safeStore("ngp_su_travel_" .. index, state.suspTravel[index] or 0.0)
    safeStore("ngp_su_speed_" .. index, state.suspSpeed[index] or 0.0)

    if not state.debugExportNow then return end

    safeStore("ngp_susp_travel_" .. index, state.suspTravel[index] or 0.0)
    safeStore("ngp_susp_speed_" .. index, state.suspSpeed[index] or 0.0)
    safeStore("ngp_susp_damper_" .. index, state.damperForce[index] or 0.0)
    safeStore("ngp_susp_spring_" .. index, state.springForce[index] or 0.0)
    safeStore("ngp_susp_int_damper_" .. index, state.externalDamper[index] or 0.0)
    safeStore("ngp_susp_int_spring_" .. index, state.externalSpring[index] or 0.0)
    safeStore("ngp_susp_int_contact_" .. index, state.contactInput[index] or 0.0)
    safeStore("ngp_susp_int_impulse_" .. index, state.contactImpulse[index] or 0.0)
    safeStore("ngp_susp_int_drop_" .. index, state.contactDrop[index] or 0.0)
    safeStore("ngp_susp_tire_road_input_" .. index, state.tireRoadInput[index] or 0.0)
    safeStore("ngp_susp_tire_road_impulse_" .. index, state.tireRoadImpulse[index] or 0.0)
    safeStore("ngp_susp_tire_combined_slip_" .. index, state.tireCombinedSlip[index] or 0.0)
    safeStore("ngp_susp_tire_slip_energy_" .. index, state.tireSlipEnergy[index] or 0.0)
    safeStore("ngp_susp_tire_force_mag_" .. index, state.tireForceMagnitude[index] or 0.0)
    safeStore("ngp_susp_body_rigidity_scale_" .. index, state.bodyRigidityScale[index] or 1.0)
    safeStore("ngp_susp_road_shock_" .. index, state.roadShock[index] or 0.0)
    safeStore("ngp_susp_load_path_loss_" .. index, state.loadPathLoss[index] or 0.0)
end

local function exportGlobal(rollFront, rollRear)
    safeStore("ngp_roll_front", rollFront or 0.0)
    safeStore("ngp_roll_rear", rollRear or 0.0)
    safeStore("ngp_suspension_status", state.status or "UNKNOWN")
    safeStore("ngp_suspension_update_count", state.updateCount or 0)
    safeStore("ngp_suspension_wheels_valid", state.wheelsValid and 1 or 0)

    safeStore("ngp_su_status", state.status or "UNKNOWN")
    safeStore("ngp_su_update_count", state.updateCount or 0)
    safeStore("ngp_su_avg_force", state.avgForce or 0.0)
    safeStore("ngp_su_avg_abs_force", state.avgAbsForce or 0.0)
    safeStore("ngp_su_avg_scale", state.avgIntegratedScale or 1.0)
    safeStore("ngp_su_max_abs_force", state.maxAbsForce or 0.0)

    if not state.debugExportNow then return end

    safeStore("ngp_caster_total", state.casterTotal or 0.0)
    safeStore("ngp_suspension_initialized", initialized and 1 or 0)
    safeStore("ngp_suspension_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_suspension_arm_linked", state.armLinked and 1 or 0)
    safeStore("ngp_suspension_control_arm_linked", state.armLinked and 1 or 0)
    safeStore("ngp_suspension_hop_linked", state.hopLinked and 1 or 0)
    safeStore("ngp_suspension_tire_force_linked", state.tireForceLinked and 1 or 0)
    safeStore("ngp_suspension_external_linked", state.externalLinked and 1 or 0)
    safeStore("ngp_suspension_contact_linked", state.contactLinked and 1 or 0)
    safeStore("ngp_suspension_body_linked", state.bodyRigidityLinked and 1 or 0)
    safeStore("ngp_suspension_road_input_linked", state.roadInputLinked and 1 or 0)
    safeStore("ngp_suspension_load_path_linked", state.loadPathLinked and 1 or 0)
    safeStore("ngp_susp_avg_tire_road_input", state.avgTireRoadInput or 0.0)
    safeStore("ngp_susp_avg_tire_slip_energy", state.avgTireSlipEnergy or 0.0)
    safeStore("ngp_susp_load_front", state.loadFront or 0.5)
    safeStore("ngp_susp_load_rear", state.loadRear or 0.5)
    safeStore("ngp_susp_load_left", state.loadLeft or 0.5)
    safeStore("ngp_susp_load_right", state.loadRight or 0.5)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i, state.force[i] or 0.0, state.camber[i] or 0.0, state.toe[i] or 0.0, state.caster[i] or 0.0)
    end
    exportGlobal(state.arbForce.front or 0.0, state.arbForce.rear or 0.0)
end

local function decayAll(dt)
    for i = 0, 3 do decayWheel(i, dt) end
    arbState.front = lowPass(arbState.front, 0.0, M.params.noCarDecayTau, dt)
    arbState.rear = lowPass(arbState.rear, 0.0, M.params.noCarDecayTau, dt)
    state.arbForce.front = lowPass(state.arbForce.front or 0.0, 0.0, M.params.noCarDecayTau, dt)
    state.arbForce.rear = lowPass(state.arbForce.rear or 0.0, 0.0, M.params.noCarDecayTau, dt)
    state.casterTotal = lowPass(state.casterTotal or 0.0, 0.0, M.params.noCarDecayTau, dt)
    updateAverages()
end

function M.init(car)
    if car then initializeSuspension(car) end
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1
    dt = clamp(safeNumber(dt, 0.0), M.params.minDt, M.params.maxDt)
    updateDebugExportGate(dt)

    car = car or safeGetCar()
    if not car then
        state.status = "NO CAR"
        state.wheelsValid = false
        decayAll(dt)
        exportState()
        return
    end

    local wheels = getWheels(car)
    if not wheels then
        state.status = "NO WHEELS"
        state.wheelsValid = false
        decayAll(dt)
        exportState()
        return
    end

    readBodyRigidity(dt)
    state.wheelsValid = true

    if not initialized then
        initializeSuspension(car)
        state.status = "INITIALIZED"
        updateAverages()
        exportState()
        return
    end

    for i = 0, 3 do
        suspensionBuffer[i] = readSuspensionTravel(car, i)
    end

    readExternalState(dt)

    local store = updateWheelForces(suspensionBuffer, dt)
    local rollFront, rollRear = applyAntiRollBar(suspensionBuffer, store, dt)
    updateCaster(car, dt)

    for i = 0, 3 do
        local camber, toe, caster = updateAlignment(i, suspensionBuffer[i])
        local baseForce = clamp(store[i] or 0.0, -M.params.maxForce, M.params.maxForce)
        local integratedForce = integrateExternalSuspension(i, baseForce, dt)
        local finalForce = applyBodyRigidityToSuspension(i, integratedForce, dt)
        finalForce = clamp(finalForce, -M.params.maxForce, M.params.maxForce)
        state.force[i] = finalForce
        exportWheel(i, finalForce, camber, toe, caster)
    end

    state.status = "RUNNING"
    updateAverages()
    exportGlobal(rollFront, rollRear)
end

function M.getForce(index)
    return state.force[index] or 0.0
end

function M.getTravel(index)
    return state.suspTravel[index] or 0.0
end

function M.getSpeed(index)
    return state.suspSpeed[index] or 0.0
end

function M.getCamber(index)
    return state.camber[index] or 0.0
end

function M.getToe(index)
    return state.toe[index] or 0.0
end

function M.getCaster(index)
    return state.caster[index] or 0.0
end

function M.getIntegratedScale(index)
    return state.integratedScale[index] or 1.0
end

function M.getState()
    return state
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0
        return string.format(
            "F %+.0f Scale %.2f Damp %+.0f Spring %+.0f Contact %.2f Imp %.2f Drop %.2f\n" ..
            "Road %.2f RImp %.2f CSlip %.2f SE %.2f / Travel %.4f Speed %.4f Cam %.3f Toe %.3f",
            state.force[i] or 0.0,
            state.integratedScale[i] or 1.0,
            state.externalDamper[i] or 0.0,
            state.externalSpring[i] or 0.0,
            state.contactInput[i] or 0.0,
            state.contactImpulse[i] or 0.0,
            state.contactDrop[i] or 0.0,
            state.tireRoadInput[i] or 0.0,
            state.tireRoadImpulse[i] or 0.0,
            state.tireCombinedSlip[i] or 0.0,
            state.tireSlipEnergy[i] or 0.0,
            state.suspTravel[i] or 0.0,
            state.suspSpeed[i] or 0.0,
            state.camber[i] or 0.0,
            state.toe[i] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "Force FL:%+.0f FR:%+.0f RL:%+.0f RR:%+.0f / AvgAbs %.0f\n" ..
        "Scale %.2f %.2f %.2f %.2f / Avg %.2f\n" ..
        "Contact %.2f %.2f %.2f %.2f\n" ..
        "Road %.2f %.2f %.2f %.2f / SE %.2f %.2f %.2f %.2f\n" ..
        "Damp %.0f %.0f %.0f %.0f / Spring %.0f %.0f %.0f %.0f\n" ..
        "ARB F:%+.0f R:%+.0f / Links LT:%s AC:%s Hop:%s TF:%s RBI:%s RII:%s LP:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",
        state.force[0] or 0.0,
        state.force[1] or 0.0,
        state.force[2] or 0.0,
        state.force[3] or 0.0,
        state.avgAbsForce or 0.0,
        state.integratedScale[0] or 1.0,
        state.integratedScale[1] or 1.0,
        state.integratedScale[2] or 1.0,
        state.integratedScale[3] or 1.0,
        state.avgIntegratedScale or 1.0,
        state.contactInput[0] or 0.0,
        state.contactInput[1] or 0.0,
        state.contactInput[2] or 0.0,
        state.contactInput[3] or 0.0,
        state.tireRoadInput[0] or 0.0,
        state.tireRoadInput[1] or 0.0,
        state.tireRoadInput[2] or 0.0,
        state.tireRoadInput[3] or 0.0,
        state.tireSlipEnergy[0] or 0.0,
        state.tireSlipEnergy[1] or 0.0,
        state.tireSlipEnergy[2] or 0.0,
        state.tireSlipEnergy[3] or 0.0,
        state.externalDamper[0] or 0.0,
        state.externalDamper[1] or 0.0,
        state.externalDamper[2] or 0.0,
        state.externalDamper[3] or 0.0,
        state.externalSpring[0] or 0.0,
        state.externalSpring[1] or 0.0,
        state.externalSpring[2] or 0.0,
        state.externalSpring[3] or 0.0,
        state.arbForce.front or 0.0,
        state.arbForce.rear or 0.0,
        state.loadLinked and "OK" or "NIL",
        state.armLinked and "OK" or "NIL",
        state.hopLinked and "OK" or "NIL",
        state.tireForceLinked and "OK" or "NIL",
        state.bodyRigidityLinked and "OK" or "NIL",
        state.roadInputLinked and "OK" or "NIL",
        state.loadPathLinked and "OK" or "NIL"
    )
end

return M
