---@diagnostic disable: undefined-global

--============================================================
-- damage_state.lua
-- ACNextGen V1.1.5 stable
-- Damage State Core
--============================================================

local M = {}

M.params = {
    minDamage = 0.0,
    maxDamage = 1.0,

    chassisGain = 0.35,
    wheelGain = 0.20,

    impactSensorGain = 0.18,
    bodyRuntimeGain = 0.08,
    brakeThermalGain = 0.06,
    suspensionStressGain = 0.07,
    tireContactGain = 0.06,

    runtimeAccumulationScale = 1.0,
    damageDecayWhenRepaired = 1.0,
    maxRuntimeDamageAdd = 0.20,

    minDt = 0.0001,
    maxDt = 0.050,

    debugStoreInterval = 0.25,
}

local state = {
    chassis = 0.0,
    wheel = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    totalDamage = 0.0,
    wheelAvg = 0.0,
    wheelMax = 0.0,

    lastEventTotal = 0.0,
    lastEventDelta = 0.0,
    lastEventType = "NONE",

    lastEventFront = 0.0,
    lastEventRear = 0.0,
    lastEventLeft = 0.0,
    lastEventRight = 0.0,

    lastRepairCount = 0.0,

    impactSensorValue = 0.0,
    runtimeDamageAdd = 0.0,

    bodyRuntimePenalty = 0.0,
    brakeThermalStress = 0.0,
    suspensionStress = 0.0,
    contactLossAvg = 0.0,

    bodyLinked = false,
    brakeLinked = false,
    suspensionLinked = false,
    tireLinked = false,
    impactLinked = false,
    eventLinked = false,

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
    v = safeNumber(v, 0.0)
    return v < 0.0 and -v or v
end

local function clampDamage(value)
    return clamp(value, M.params.minDamage, M.params.maxDamage)
end

local function isValidWheelIndex(index)
    return index == 0 or index == 1 or index == 2 or index == 3
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

local function safeLoadNumber(key, defaultValue)
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

local function safeLoadString(key, defaultValue)
    local value = safeLoadRaw(key)
    if value == nil then
        return defaultValue or ""
    end
    return tostring(value)
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
    state.debugStoreTimer = (state.debugStoreTimer or 0.0) + (dt or 0.0)
    if state.debugStoreTimer >= M.params.debugStoreInterval then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    else
        state.debugStoreNow = false
    end
end

local function updateTotals()
    local sum = 0.0
    local maxWheel = 0.0

    for i = 0, 3 do
        local v = clampDamage(state.wheel[i] or 0.0)
        state.wheel[i] = v
        sum = sum + v
        if v > maxWheel then
            maxWheel = v
        end
    end

    state.chassis = clampDamage(state.chassis or 0.0)
    state.wheelAvg = clampDamage(sum * 0.25)
    state.wheelMax = clampDamage(maxWheel)
    state.totalDamage = clampDamage(state.chassis * 0.55 + state.wheelAvg * 0.45)
end

local function readRuntimeDamageInputs()
    local impact, impactKey = safeLoadAlt(
        0.0,
        "ngp_impact_value",
        "ngp_impact_strength",
        "ngp_damage_event_last",
        "ngp_damage_event_preserve_root"
    )

    state.impactLinked = impactKey ~= nil
    state.impactSensorValue = clampDamage(impact)

    local body, bodyKey = safeLoadAlt(
        0.0,
        "ngp_body_runtime_flex_penalty",
        "ngp_chassis_flex_body_runtime_penalty"
    )

    state.bodyLinked = bodyKey ~= nil
    state.bodyRuntimePenalty = clampDamage(body)

    local thermal, thermalKey = safeLoadAlt(
        0.0,
        "ngp_brake_root_heat_avg",
        "ngp_brake_fade_root_add_avg",
        "ngp_virtual_thermal_stress",
        "ngp_chassis_roll_thermal_stress"
    )

    state.brakeLinked = thermalKey ~= nil
    state.brakeThermalStress = clampDamage(thermal)

    local suspension, suspensionKey = safeLoadAlt(
        0.0,
        "ngp_weight_suspension_stress",
        "ngp_chassis_flex_suspension_stress",
        "ngp_chassis_roll_suspension_stress"
    )

    state.suspensionLinked = suspensionKey ~= nil
    state.suspensionStress = clampDamage(suspension)

    local contactSum = 0.0
    local tireLinked = false

    for i = 0, 3 do
        local loss, lossKey = safeLoadAlt(
            0.0,
            "ngp_tire_contact_loss_" .. i,
            "ngp_tcr_contact_loss_" .. i,
            "ngp_contact_loss_" .. i
        )

        if lossKey ~= nil then
            tireLinked = true
        end

        contactSum = contactSum + clampDamage(loss)
    end

    state.tireLinked = tireLinked
    state.contactLossAvg = clampDamage(contactSum * 0.25)

    state.runtimeDamageAdd = clampDamage(
        state.impactSensorValue * M.params.impactSensorGain
        + state.bodyRuntimePenalty * M.params.bodyRuntimeGain
        + state.brakeThermalStress * M.params.brakeThermalGain
        + state.suspensionStress * M.params.suspensionStressGain
        + state.contactLossAvg * M.params.tireContactGain
    )

    if state.runtimeDamageAdd > M.params.maxRuntimeDamageAdd then
        state.runtimeDamageAdd = M.params.maxRuntimeDamageAdd
    end
end

local function exportAll()
    updateTotals()

    safeStore("ngp_damage_chassis", state.chassis)
    safeStore("ngp_damage_total", state.totalDamage)
    safeStore("ngp_damage_wheel_avg", state.wheelAvg)
    safeStore("ngp_damage_wheel_max", state.wheelMax)

    for i = 0, 3 do
        safeStore("ngp_damage_wheel_" .. i, state.wheel[i] or 0.0)
    end

    safeStore("ngp_damage_state_status", state.status or "UNKNOWN")
    safeStore("ngp_damage_state_update_count", state.updateCount or 0)
    safeStore("ngp_damage_state_event_total", state.lastEventTotal or 0.0)
    safeStore("ngp_damage_state_event_delta", state.lastEventDelta or 0.0)
    safeStore("ngp_damage_state_event_type", state.lastEventType or "NONE")

    safeStore("ngp_damage_runtime_add", state.runtimeDamageAdd or 0.0)
    safeStore("ngp_damage_impact_sensor_value", state.impactSensorValue or 0.0)
    safeStore("ngp_damage_body_runtime", state.bodyRuntimePenalty or 0.0)
    safeStore("ngp_damage_brake_thermal", state.brakeThermalStress or 0.0)
    safeStore("ngp_damage_suspension_stress", state.suspensionStress or 0.0)
    safeStore("ngp_damage_contact_loss_avg", state.contactLossAvg or 0.0)

    safeStore("ngp_damage_body_linked", state.bodyLinked and 1 or 0)
    safeStore("ngp_damage_brake_linked", state.brakeLinked and 1 or 0)
    safeStore("ngp_damage_suspension_linked", state.suspensionLinked and 1 or 0)
    safeStore("ngp_damage_tire_linked", state.tireLinked and 1 or 0)
    safeStore("ngp_damage_impact_linked", state.impactLinked and 1 or 0)
    safeStore("ngp_damage_event_linked", state.eventLinked and 1 or 0)

    safeStore("ngp_condition_total", state.totalDamage)
    safeStore("ngp_vehicle_condition_total", state.totalDamage)
    safeStore("ngp_condition_chassis", state.chassis)
    safeStore("ngp_vehicle_chassis_damage", state.chassis)

    for i = 0, 3 do
        safeStore("ngp_condition_wheel_" .. i, state.wheel[i] or 0.0)
        safeStore("ngp_vehicle_wheel_damage_" .. i, state.wheel[i] or 0.0)
    end

    safeStore("ngp_damage_front_delta", state.lastEventFront or 0.0)
    safeStore("ngp_damage_rear_delta", state.lastEventRear or 0.0)
    safeStore("ngp_damage_left_delta", state.lastEventLeft or 0.0)
    safeStore("ngp_damage_right_delta", state.lastEventRight or 0.0)
end

local function applyChassisDamage(value)
    state.chassis = clampDamage((state.chassis or 0.0) + safeNumber(value, 0.0))
end

local function applyWheelDamage(index, value)
    if not isValidWheelIndex(index) then
        return
    end

    state.wheel[index] = clampDamage((state.wheel[index] or 0.0) + safeNumber(value, 0.0))
end

local function applyDirectionalDamage(frontDelta, rearDelta, leftDelta, rightDelta)
    local p = M.params
    local used = false

    if frontDelta > 0.0 then
        local amount = frontDelta * p.wheelGain
        applyWheelDamage(0, amount * 0.5)
        applyWheelDamage(1, amount * 0.5)
        used = true
    end

    if rearDelta > 0.0 then
        local amount = rearDelta * p.wheelGain
        applyWheelDamage(2, amount * 0.5)
        applyWheelDamage(3, amount * 0.5)
        used = true
    end

    if leftDelta > 0.0 then
        local amount = leftDelta * p.wheelGain
        applyWheelDamage(0, amount * 0.5)
        applyWheelDamage(2, amount * 0.5)
        used = true
    end

    if rightDelta > 0.0 then
        local amount = rightDelta * p.wheelGain
        applyWheelDamage(1, amount * 0.5)
        applyWheelDamage(3, amount * 0.5)
        used = true
    end

    return used
end

local function applyEventDamage(delta, eventType)
    delta = safeNumber(delta, 0.0)
    if delta <= 0.0 then
        return
    end

    applyChassisDamage(delta * M.params.chassisGain)

    local wheelAmount = delta * M.params.wheelGain

    if eventType == "VERTICAL" then
        for i = 0, 3 do
            applyWheelDamage(i, wheelAmount * 0.25)
        end
        return
    end

    if eventType == "SIDE" then
        for i = 0, 3 do
            applyWheelDamage(i, wheelAmount * 0.20)
        end
        return
    end

    if eventType == "FRONT/REAR" or eventType == "LONGITUDINAL" then
        for i = 0, 3 do
            applyWheelDamage(i, wheelAmount * 0.20)
        end
        return
    end
end

local function applyRuntimeDamage(dt)
    if state.runtimeDamageAdd <= 0.0 then
        return
    end

    local amount = state.runtimeDamageAdd * dt * M.params.runtimeAccumulationScale

    applyChassisDamage(amount)

    for i = 0, 3 do
        applyWheelDamage(i, amount * 0.25)
    end
end

local function handleRepair(eventTotal)
    local repaired = safeLoadNumber("ngp_damage_event_repaired", 0.0)

    if repaired > state.lastRepairCount and eventTotal <= 0.0 then
        M.reset(false)
        state.lastRepairCount = repaired
        state.status = "RESET BY EVENT"
        return true
    end

    state.lastRepairCount = repaired
    return false
end

function M.init()
    state.status = "INIT"
    exportAll()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = clamp(safeNumber(dt, 0.0), M.params.minDt, M.params.maxDt)
    updateDebugGate(dt)

    car = car or safeGetCar()
    if not car then
        state.status = "NO CAR"
        readRuntimeDamageInputs()
        exportAll()
        return
    end

    readRuntimeDamageInputs()

    local eventRaw = safeLoadRaw("ngp_damage_event_total")
    state.eventLinked = eventRaw ~= nil

    local eventTotal = safeNumber(eventRaw, state.lastEventTotal or 0.0)
    local eventType = safeLoadString("ngp_damage_event_type", "NONE")

    local eventFront = safeLoadNumber("ngp_damage_event_front", 0.0)
    local eventRear = safeLoadNumber("ngp_damage_event_rear", 0.0)
    local eventLeft = safeLoadNumber("ngp_damage_event_left", 0.0)
    local eventRight = safeLoadNumber("ngp_damage_event_right", 0.0)

    if handleRepair(eventTotal) then
        exportAll()
        return
    end

    local delta = eventTotal - (state.lastEventTotal or 0.0)
    if delta < 0.0 then
        delta = 0.0
        state.status = "EVENT RESET"
    else
        state.status = "RUNNING"
    end

    local frontDelta = math.max(eventFront - (state.lastEventFront or 0.0), 0.0)
    local rearDelta = math.max(eventRear - (state.lastEventRear or 0.0), 0.0)
    local leftDelta = math.max(eventLeft - (state.lastEventLeft or 0.0), 0.0)
    local rightDelta = math.max(eventRight - (state.lastEventRight or 0.0), 0.0)

    state.lastEventTotal = eventTotal
    state.lastEventDelta = delta
    state.lastEventType = eventType
    state.lastEventFront = eventFront
    state.lastEventRear = eventRear
    state.lastEventLeft = eventLeft
    state.lastEventRight = eventRight

    if delta > 0.0 then
        local directionalUsed = applyDirectionalDamage(frontDelta, rearDelta, leftDelta, rightDelta)
        if not directionalUsed then
            applyEventDamage(delta, eventType)
        else
            applyChassisDamage(delta * M.params.chassisGain)
        end
    end

    applyRuntimeDamage(dt)

    exportAll()
end

function M.damageChassis(value)
    applyChassisDamage(value)
    exportAll()
end

function M.damageWheel(index, value)
    applyWheelDamage(index, value)
    exportAll()
end

function M.setChassis(value)
    state.chassis = clampDamage(value)
    exportAll()
end

function M.setWheel(index, value)
    if not isValidWheelIndex(index) then
        return
    end

    state.wheel[index] = clampDamage(value)
    exportAll()
end

function M.reset(preserveRepairCounter)
    state.chassis = 0.0

    for i = 0, 3 do
        state.wheel[i] = 0.0
    end

    state.totalDamage = 0.0
    state.wheelAvg = 0.0
    state.wheelMax = 0.0

    state.lastEventTotal = 0.0
    state.lastEventDelta = 0.0
    state.lastEventType = "NONE"
    state.lastEventFront = 0.0
    state.lastEventRear = 0.0
    state.lastEventLeft = 0.0
    state.lastEventRight = 0.0

    state.runtimeDamageAdd = 0.0
    state.impactSensorValue = 0.0

    if not preserveRepairCounter then
        state.lastRepairCount = safeLoadNumber("ngp_damage_event_repaired", state.lastRepairCount or 0.0)
    end

    state.status = "RESET"
    exportAll()
end

function M.get()
    return state
end

function M.getChassis()
    return state.chassis or 0.0
end

function M.getWheel(index)
    if not isValidWheelIndex(index) then
        return 0.0
    end

    return state.wheel[index] or 0.0
end

function M.getTotal()
    return state.totalDamage or 0.0
end

function M.getRuntimeDamage()
    return state.runtimeDamageAdd or 0.0
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f\n" ..
        "Damage Total %.3f / Chassis %.3f / WAvg %.3f WMax %.3f\n" ..
        "Wheel %.3f %.3f %.3f %.3f\n" ..
        "EventTotal %.4f / Delta %.4f / Type %s\n" ..
        "Runtime %.4f Impact %.3f Body %.3f Brake %.3f Susp %.3f Tire %.3f\n" ..
        "Links Event:%s Impact:%s Body:%s Brake:%s Susp:%s Tire:%s",
        tostring(state.status),
        state.updateCount or 0,

        state.totalDamage or 0.0,
        state.chassis or 0.0,
        state.wheelAvg or 0.0,
        state.wheelMax or 0.0,

        state.wheel[0] or 0.0,
        state.wheel[1] or 0.0,
        state.wheel[2] or 0.0,
        state.wheel[3] or 0.0,

        state.lastEventTotal or 0.0,
        state.lastEventDelta or 0.0,
        tostring(state.lastEventType),

        state.runtimeDamageAdd or 0.0,
        state.impactSensorValue or 0.0,
        state.bodyRuntimePenalty or 0.0,
        state.brakeThermalStress or 0.0,
        state.suspensionStress or 0.0,
        state.contactLossAvg or 0.0,

        state.eventLinked and "OK" or "NIL",
        state.impactLinked and "OK" or "NIL",
        state.bodyLinked and "OK" or "NIL",
        state.brakeLinked and "OK" or "NIL",
        state.suspensionLinked and "OK" or "NIL",
        state.tireLinked and "OK" or "NIL"
    )
end

exportAll()

return M
