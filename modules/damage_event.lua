---@diagnostic disable: undefined-global

--============================================================
-- damage_event.lua
-- ACNextGen V1.1.5 stable
-- Impact event recorder
--============================================================

local M = {}

M.params = {
    gravity = 9.81,

    impactThreshold = 4.0,
    impactGain = 1.0,

    pitRepairSpeed = 5.0,

    minDt = 0.0001,
    maxDt = 0.050,

    signedDirection = true,
    directionDeadbandG = 0.08,

    debugStoreInterval = 0.25,
}

local state = {
    total = 0.0,

    lastImpact = 0.0,
    lastG = 0.0,
    peakG = 0.0,

    lastType = "NONE",
    lastDirection = "NONE",

    lateralG = 0.0,
    longitudinalG = 0.0,
    verticalG = 0.0,

    signedLateralG = 0.0,
    signedLongitudinalG = 0.0,
    signedVerticalG = 0.0,

    front = 0.0,
    rear = 0.0,
    left = 0.0,
    right = 0.0,

    speedKmh = 0.0,
    inPit = false,
    repaired = 0,
    wasInPitRepair = false,

    impactActive = false,

    status = "INIT",
    updateCount = 0,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

local function safeNumber(value, defaultValue)
    local n = tonumber(value)
    if n == nil or n ~= n then
        return defaultValue or 0.0
    end
    if n == math.huge or n == -math.huge then
        return defaultValue or 0.0
    end
    return n
end

local function clamp(value, minValue, maxValue)
    value = safeNumber(value, minValue)
    if value < minValue then
        return minValue
    end
    if value > maxValue then
        return maxValue
    end
    return value
end

local function abs(value)
    value = safeNumber(value, 0.0)
    return value < 0.0 and -value or value
end

local function safeField(object, field, defaultValue)
    if not object then
        return defaultValue
    end

    local ok, value = pcall(function()
        return object[field]
    end)

    if not ok or value == nil then
        return defaultValue
    end

    return value
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

local function readVec3Component(vector, field)
    if not vector then
        return 0.0
    end

    local value = safeField(vector, field, 0.0)
    return safeNumber(value, 0.0)
end

local function getAccelerationG(car)
    local acc = safeField(car, "acceleration", nil)

    local ax = readVec3Component(acc, "x")
    local ay = readVec3Component(acc, "y")
    local az = readVec3Component(acc, "z")

    local gravity = math.max(M.params.gravity, 0.001)

    local signedLateralG = ax / gravity
    local signedLongitudinalG = ay / gravity
    local signedVerticalG = az / gravity

    return
        abs(signedLateralG),
        abs(signedLongitudinalG),
        abs(signedVerticalG),
        signedLateralG,
        signedLongitudinalG,
        signedVerticalG
end

local function getImpactType(lateralG, longitudinalG, verticalG)
    if verticalG >= lateralG and verticalG >= longitudinalG then
        return "VERTICAL"
    end

    if lateralG > longitudinalG then
        return "SIDE"
    end

    return "FRONT/REAR"
end

local function getImpactDirection(impactType, signedLateralG, signedLongitudinalG, signedVerticalG)
    local deadband = M.params.directionDeadbandG or 0.08

    if impactType == "VERTICAL" then
        if signedVerticalG > deadband then
            return "UP"
        end
        if signedVerticalG < -deadband then
            return "DOWN"
        end
        return "VERTICAL"
    end

    if impactType == "SIDE" then
        if signedLateralG > deadband then
            return "RIGHT"
        end
        if signedLateralG < -deadband then
            return "LEFT"
        end
        return "SIDE"
    end

    if impactType == "FRONT/REAR" then
        if signedLongitudinalG > deadband then
            return "FRONT"
        end
        if signedLongitudinalG < -deadband then
            return "REAR"
        end
        return "FRONT/REAR"
    end

    return "NONE"
end

local function accumulateDirection(impactType, direction, strength)
    strength = safeNumber(strength, 0.0)

    if impactType == "VERTICAL" then
        return
    end

    if not M.params.signedDirection then
        if impactType == "SIDE" then
            state.left = state.left + strength * 0.5
            state.right = state.right + strength * 0.5
            return
        end

        if impactType == "FRONT/REAR" then
            state.front = state.front + strength * 0.5
            state.rear = state.rear + strength * 0.5
            return
        end
    end

    if direction == "LEFT" then
        state.left = state.left + strength
        return
    end

    if direction == "RIGHT" then
        state.right = state.right + strength
        return
    end

    if direction == "FRONT" then
        state.front = state.front + strength
        return
    end

    if direction == "REAR" then
        state.rear = state.rear + strength
        return
    end

    if impactType == "SIDE" then
        state.left = state.left + strength * 0.5
        state.right = state.right + strength * 0.5
        return
    end

    if impactType == "FRONT/REAR" then
        state.front = state.front + strength * 0.5
        state.rear = state.rear + strength * 0.5
    end
end

local function exportState()
    safeStore("ngp_damage_event_total", state.total or 0.0)
    safeStore("ngp_damage_event_last", state.lastImpact or 0.0)
    safeStore("ngp_damage_event_g", state.lastG or 0.0)
    safeStore("ngp_damage_event_peak_g", state.peakG or 0.0)
    safeStore("ngp_damage_event_type", state.lastType or "NONE")
    safeStore("ngp_damage_event_direction", state.lastDirection or "NONE")
    safeStore("ngp_damage_event_repaired", state.repaired or 0)

    safeStore("ngp_damage_event_front", state.front or 0.0)
    safeStore("ngp_damage_event_rear", state.rear or 0.0)
    safeStore("ngp_damage_event_left", state.left or 0.0)
    safeStore("ngp_damage_event_right", state.right or 0.0)

    safeStore("ngp_damage_event_lateral_g", state.lateralG or 0.0)
    safeStore("ngp_damage_event_longitudinal_g", state.longitudinalG or 0.0)
    safeStore("ngp_damage_event_vertical_g", state.verticalG or 0.0)

    safeStore("ngp_damage_event_signed_lateral_g", state.signedLateralG or 0.0)
    safeStore("ngp_damage_event_signed_longitudinal_g", state.signedLongitudinalG or 0.0)
    safeStore("ngp_damage_event_signed_vertical_g", state.signedVerticalG or 0.0)

    safeStore("ngp_damage_event_speed", state.speedKmh or 0.0)
    safeStore("ngp_damage_event_in_pit", state.inPit and 1 or 0)
    safeStore("ngp_damage_event_active", state.impactActive and 1 or 0)

    safeStore("ngp_damage_event_status", state.status or "UNKNOWN")
    safeStore("ngp_damage_event_update_count", state.updateCount or 0)

    safeStore("ngp_impact_total", state.total or 0.0)
    safeStore("ngp_impact_last", state.lastImpact or 0.0)
    safeStore("ngp_impact_g", state.lastG or 0.0)
    safeStore("ngp_impact_peak_g", state.peakG or 0.0)
    safeStore("ngp_impact_type", state.lastType or "NONE")
    safeStore("ngp_impact_direction", state.lastDirection or "NONE")

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_damage_event_threshold", M.params.impactThreshold)
    safeStore("ngp_damage_event_lateral_total", (state.left or 0.0) + (state.right or 0.0))
    safeStore("ngp_damage_event_longitudinal_total", (state.front or 0.0) + (state.rear or 0.0))
end

function M.init()
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = safeNumber(dt, 0.0)
    if dt <= 0.0 then
        dt = M.params.minDt
    end

    dt = clamp(dt, M.params.minDt, M.params.maxDt)

    updateDebugGate(dt)

    car = car or safeGetCar()

    if not car then
        state.status = "NO CAR"
        state.speedKmh = 0.0
        state.inPit = false
        state.impactActive = false
        state.lastImpact = 0.0
        exportState()
        return
    end

    local speedKmh = safeNumber(safeField(car, "speedKmh", 0.0), 0.0)
    local inPit = safeField(car, "isInPit", false) and true or false

    state.speedKmh = speedKmh
    state.inPit = inPit

    local pitRepairActive =
        inPit
        and
        speedKmh < M.params.pitRepairSpeed

    if pitRepairActive and not state.wasInPitRepair then
        M.reset("PIT REPAIR")
        state.wasInPitRepair = true
        state.status = "PIT REPAIR"
        exportState()
        return
    end

    if not pitRepairActive then
        state.wasInPitRepair = false
    end

    local lateralG,
          longitudinalG,
          verticalG,
          signedLateralG,
          signedLongitudinalG,
          signedVerticalG =
        getAccelerationG(car)

    state.lateralG = lateralG
    state.longitudinalG = longitudinalG
    state.verticalG = verticalG

    state.signedLateralG = signedLateralG
    state.signedLongitudinalG = signedLongitudinalG
    state.signedVerticalG = signedVerticalG

    local impactG =
        math.max(
            lateralG,
            longitudinalG,
            verticalG
        )

    state.lastG = impactG
    state.peakG = math.max(state.peakG or 0.0, impactG)

    if impactG > M.params.impactThreshold then
        local strength =
            (impactG - M.params.impactThreshold)
            *
            dt
            *
            M.params.impactGain

        state.lastImpact = strength
        state.total = (state.total or 0.0) + strength

        state.lastType =
            getImpactType(
                lateralG,
                longitudinalG,
                verticalG
            )

        state.lastDirection =
            getImpactDirection(
                state.lastType,
                signedLateralG,
                signedLongitudinalG,
                signedVerticalG
            )

        accumulateDirection(
            state.lastType,
            state.lastDirection,
            strength
        )

        state.impactActive = true
    else
        state.lastImpact = 0.0
        state.impactActive = false
    end

    state.status = "RUNNING"
    exportState()
end

function M.reset(reason)
    state.total = 0.0

    state.lastImpact = 0.0
    state.lastG = 0.0
    state.peakG = 0.0
    state.lastType = "NONE"
    state.lastDirection = "NONE"

    state.lateralG = 0.0
    state.longitudinalG = 0.0
    state.verticalG = 0.0

    state.signedLateralG = 0.0
    state.signedLongitudinalG = 0.0
    state.signedVerticalG = 0.0

    state.front = 0.0
    state.rear = 0.0
    state.left = 0.0
    state.right = 0.0

    state.impactActive = false
    state.repaired = (state.repaired or 0) + 1
    state.status = tostring(reason or "RESET")

    exportState()
end

function M.get()
    return state
end

function M.getTotal()
    return state.total or 0.0
end

function M.getLastImpact()
    return state.lastImpact or 0.0
end

function M.getLastType()
    return state.lastType or "NONE"
end

function M.getLastDirection()
    return state.lastDirection or "NONE"
end

function M.getPeakG()
    return state.peakG or 0.0
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f / Active %s\n" ..
        "ImpactG %.2f Peak %.2f / Last %.4f / Type %s / Dir %s\n" ..
        "LatG %.2f LongG %.2f VertG %.2f\n" ..
        "Total %.4f / F %.4f R %.4f L %.4f Rt %.4f\n" ..
        "Pit %s Speed %.1f / Repaired %d",

        tostring(state.status),
        state.updateCount or 0,
        state.impactActive and "YES" or "NO",

        state.lastG or 0.0,
        state.peakG or 0.0,
        state.lastImpact or 0.0,
        tostring(state.lastType),
        tostring(state.lastDirection),

        state.lateralG or 0.0,
        state.longitudinalG or 0.0,
        state.verticalG or 0.0,

        state.total or 0.0,
        state.front or 0.0,
        state.rear or 0.0,
        state.left or 0.0,
        state.right or 0.0,

        state.inPit and "YES" or "NO",
        state.speedKmh or 0.0,
        state.repaired or 0
    )
end

return M
