---@diagnostic disable: undefined-global

--============================================================
-- ACNeXtGen
-- wheel_audit.lua
-- Small UI helper for AC wheel API inspection
-- Strict-struct safe version
-- ホイールAPI安全監査 / 診断補助
--============================================================

local M = {}

local WHEEL_NAME = {
    [0] = "FL",
    [1] = "FR",
    [2] = "RL",
    [3] = "RR",
}

--============================================================
-- State
--============================================================

local state = {
    status = "INIT",
    updateCount = 0,

    carValid = false,
    wheelsValid = false,

    wheelExists = {
        [0] = false,
        [1] = false,
        [2] = false,
        [3] = false,
    },

    wheel = {},

    available = {},
}

for i = 0, 3 do
    state.wheel[i] = {
        load = nil,
        normalLoad = nil,
        wheelLoad = nil,

        slipRatio = nil,
        slipAngle = nil,

        angularSpeed = nil,

        suspensionTravel = nil,
        suspensionTravelM = nil,
        suspensionLength = nil,
        travel = nil,
        damperTravel = nil,
        suspensionPosition = nil,

        surfaceGrip = nil,

        tyreRadius = nil,
        tireRadius = nil,

        tyreWidth = nil,
        tireWidth = nil,

        tyrePressure = nil,
        tirePressure = nil,

        brakeTemperature = nil,
        discTemperature = nil,

        tyreCoreTemperature = nil,
        tireCoreTemperature = nil,
        tyreTemperature = nil,
        tireTemperature = nil,
    }

    state.available[i] = {}
end

M.state = state
M.debug = state

--============================================================
-- Utility
--============================================================

local function num(value, defaultValue)
    local n =
        tonumber(value)

    if n == nil or n ~= n then
        return defaultValue or 0.0
    end

    return n
end

local function safeStore(key, value)
    pcall(
        function()
            ac.store(
                key,
                value
            )
        end
    )
end

local function drawLine(line)
    if ui and ui.text then
        ui.text(
            tostring(line)
        )
    end
end

------------------------------------------------------------
-- IMPORTANT:
-- AC/CSP wheel object can be strict-struct.
-- Accessing a missing member directly can crash the app.
------------------------------------------------------------

local function safeField(obj, key, defaultValue)
    if not obj then
        return defaultValue
    end

    local ok, value =
        pcall(
            function()
                return obj[key]
            end
        )

    if not ok or value == nil then
        return defaultValue
    end

    return value
end

local function hasField(obj, key)
    if not obj then
        return false
    end

    local ok, value =
        pcall(
            function()
                return obj[key]
            end
        )

    return
        ok
        and
        value ~= nil
end

local function fmt(value, format)
    if value == nil then
        return "NIL"
    end

    return string.format(
        format,
        num(
            value,
            0.0
        )
    )
end

local function getWheel(car, index)
    if not car then
        return nil
    end

    local wheels =
        safeField(
            car,
            "wheels",
            nil
        )

    if not wheels then
        return nil
    end

    local ok, wheel =
        pcall(
            function()
                return wheels[index]
            end
        )

    if not ok then
        return nil
    end

    return wheel
end

local function readFieldSet(wheel, index)
    local w = state.wheel[index]

    -- SDK-confirmed ac.StateWheel members only. Strict structs must not be probed for aliases.
    w.load = safeField(wheel, "load", nil)
    w.normalLoad = safeField(wheel, "loadK", nil)
    w.wheelLoad = nil

    w.slipRatio = safeField(wheel, "slipRatio", nil)
    w.slipAngle = safeField(wheel, "slipAngle", nil)
    w.angularSpeed = safeField(wheel, "angularSpeed", nil)

    w.suspensionTravel = safeField(wheel, "suspensionTravel", nil)
    w.suspensionTravelM = nil
    w.suspensionLength = nil
    w.travel = nil
    w.damperTravel = nil
    w.suspensionPosition = nil

    w.surfaceGrip = safeField(wheel, "surfaceGrip", nil)
    w.tyreRadius = safeField(wheel, "tyreRadius", nil)
    w.tireRadius = w.tyreRadius
    w.tyreWidth = safeField(wheel, "tyreWidth", nil)
    w.tireWidth = w.tyreWidth
    w.tyrePressure = safeField(wheel, "tyrePressure", nil)
    w.tirePressure = w.tyrePressure

    w.brakeTemperature = safeField(wheel, "brakeTemperature", nil)
    w.discTemperature = safeField(wheel, "discTemperature", nil)
    w.tyreCoreTemperature = safeField(wheel, "tyreCoreTemperature", nil)
    w.tireCoreTemperature = w.tyreCoreTemperature
    w.tyreTemperature = w.tyreCoreTemperature
    w.tireTemperature = w.tyreCoreTemperature
end

local function updateAvailability(wheel, index)
    local a = state.available[index]

    a.load = hasField(wheel, "load")
    a.normalLoad = hasField(wheel, "loadK")
    a.wheelLoad = false
    a.slipRatio = hasField(wheel, "slipRatio")
    a.slipAngle = hasField(wheel, "slipAngle")
    a.angularSpeed = hasField(wheel, "angularSpeed")
    a.suspensionTravel = hasField(wheel, "suspensionTravel")
    a.suspensionTravelM = false
    a.suspensionLength = false
    a.travel = false
    a.damperTravel = false
    a.suspensionPosition = false
    a.surfaceGrip = hasField(wheel, "surfaceGrip")
    a.tyreRadius = hasField(wheel, "tyreRadius")
    a.tireRadius = a.tyreRadius
    a.tyreWidth = hasField(wheel, "tyreWidth")
    a.tireWidth = a.tyreWidth
    a.tyrePressure = hasField(wheel, "tyrePressure")
    a.tirePressure = a.tyrePressure
    a.brakeTemperature = hasField(wheel, "brakeTemperature")
    a.discTemperature = hasField(wheel, "discTemperature")
    a.tyreCoreTemperature = hasField(wheel, "tyreCoreTemperature")
    a.tireCoreTemperature = a.tyreCoreTemperature
    a.tyreTemperature = a.tyreCoreTemperature
    a.tireTemperature = a.tyreCoreTemperature
end

local function exportWheel(index)
    local w =
        state.wheel[index]

    safeStore(
        "ngp_wheel_audit_exists_" .. index,
        state.wheelExists[index] and 1 or 0
    )

    safeStore(
        "ngp_wheel_audit_load_" .. index,
        num(
            w.load,
            0.0
        )
    )

    safeStore(
        "ngp_wheel_audit_normal_load_" .. index,
        num(
            w.normalLoad,
            0.0
        )
    )

    safeStore(
        "ngp_wheel_audit_wheel_load_" .. index,
        num(
            w.wheelLoad,
            0.0
        )
    )

    safeStore(
        "ngp_wheel_audit_slip_ratio_" .. index,
        num(
            w.slipRatio,
            0.0
        )
    )

    safeStore(
        "ngp_wheel_audit_slip_angle_" .. index,
        num(
            w.slipAngle,
            0.0
        )
    )

    safeStore(
        "ngp_wheel_audit_angular_speed_" .. index,
        num(
            w.angularSpeed,
            0.0
        )
    )

    safeStore(
        "ngp_wheel_audit_suspension_travel_" .. index,
        num(
            w.suspensionTravel,
            0.0
        )
    )

    safeStore(
        "ngp_wheel_audit_suspension_length_" .. index,
        num(
            w.suspensionLength,
            0.0
        )
    )

    safeStore(
        "ngp_wheel_audit_surface_grip_" .. index,
        num(
            w.surfaceGrip,
            0.0
        )
    )

    safeStore(
        "ngp_wheel_audit_tyre_pressure_" .. index,
        num(
            w.tyrePressure or w.tirePressure,
            0.0
        )
    )
end

local function exportGlobal()
    safeStore(
        "ngp_wheel_audit_status",
        state.status or "UNKNOWN"
    )

    safeStore(
        "ngp_wheel_audit_update_count",
        state.updateCount or 0
    )

    safeStore(
        "ngp_wheel_audit_car_valid",
        state.carValid and 1 or 0
    )

    safeStore(
        "ngp_wheel_audit_wheels_valid",
        state.wheelsValid and 1 or 0
    )
end

local function exportState()
    for i = 0, 3 do
        exportWheel(
            i
        )
    end

    exportGlobal()
end

--============================================================
-- Update
--============================================================

function M.update(dt, car)
    state.updateCount =
        state.updateCount
        +
        1

    dt =
        num(
            dt,
            0.0
        )

    if dt <= 0.0 then
        state.status = "BAD DT"
        exportState()
        return
    end

    car =
        car
        or
        ac.getCar(0)

    if not car then
        state.status = "NO CAR"
        state.carValid = false
        state.wheelsValid = false
        exportState()
        return
    end

    state.carValid =
        true

    local wheels =
        safeField(
            car,
            "wheels",
            nil
        )

    if not wheels then
        state.status = "NO WHEELS"
        state.wheelsValid = false
        exportState()
        return
    end

    state.wheelsValid =
        true

    state.status =
        "RUNNING"

    for i = 0, 3 do
        local wheel =
            getWheel(
                car,
                i
            )

        if wheel then
            state.wheelExists[i] =
                true

            readFieldSet(
                wheel,
                i
            )

            updateAvailability(
                wheel,
                i
            )
        else
            clearWheel(
                i
            )
        end
    end

    exportState()
end

--============================================================
-- Draw
--============================================================

function M.draw(car)
    car =
        car
        or
        ac.getCar(0)

    --------------------------------------------------------
    -- drawだけ呼ばれた時でも最低限更新
    --------------------------------------------------------

    M.update(
        0.016,
        car
    )

    if not car then
        drawLine("Wheel audit: car NIL")
        return
    end

    local wheels =
        safeField(
            car,
            "wheels",
            nil
        )

    if not wheels then
        drawLine("Wheel audit: wheels NIL")
        return
    end

    drawLine("=== WHEEL AUDIT ===")
    drawLine(
        string.format(
            "Status %s / Count %.0f",
            tostring(state.status),
            state.updateCount or 0
        )
    )

    for i = 0, 3 do
        local name =
            WHEEL_NAME[i]
            or tostring(i)

        local w =
            state.wheel[i]

        if not state.wheelExists[i] then
            drawLine(
                name .. " NIL"
            )
        else
            drawLine(
                string.format(
                    "%s load %s nLoad %s wLoad %s",
                    name,
                    fmt(
                        w.load,
                        "%.0f"
                    ),
                    fmt(
                        w.normalLoad,
                        "%.0f"
                    ),
                    fmt(
                        w.wheelLoad,
                        "%.0f"
                    )
                )
            )

            drawLine(
                string.format(
                    "   slipR %s slipA %s omega %s",
                    fmt(
                        w.slipRatio,
                        "%.4f"
                    ),
                    fmt(
                        w.slipAngle,
                        "%.4f"
                    ),
                    fmt(
                        w.angularSpeed,
                        "%.2f"
                    )
                )
            )

            drawLine(
                string.format(
                    "   suspTravel %s suspTravelM %s suspLength %s",
                    fmt(
                        w.suspensionTravel,
                        "%.4f"
                    ),
                    fmt(
                        w.suspensionTravelM,
                        "%.4f"
                    ),
                    fmt(
                        w.suspensionLength,
                        "%.4f"
                    )
                )
            )

            drawLine(
                string.format(
                    "   travel %s damper %s suspPos %s",
                    fmt(
                        w.travel,
                        "%.4f"
                    ),
                    fmt(
                        w.damperTravel,
                        "%.4f"
                    ),
                    fmt(
                        w.suspensionPosition,
                        "%.4f"
                    )
                )
            )

            drawLine(
                string.format(
                    "   grip %s radius %s/%s width %s/%s pressure %s/%s",
                    fmt(
                        w.surfaceGrip,
                        "%.3f"
                    ),
                    fmt(
                        w.tyreRadius,
                        "%.3f"
                    ),
                    fmt(
                        w.tireRadius,
                        "%.3f"
                    ),
                    fmt(
                        w.tyreWidth,
                        "%.3f"
                    ),
                    fmt(
                        w.tireWidth,
                        "%.3f"
                    ),
                    fmt(
                        w.tyrePressure,
                        "%.2f"
                    ),
                    fmt(
                        w.tirePressure,
                        "%.2f"
                    )
                )
            )

            drawLine(
                string.format(
                    "   brakeT %s discT %s tyreT %s/%s",
                    fmt(
                        w.brakeTemperature,
                        "%.1f"
                    ),
                    fmt(
                        w.discTemperature,
                        "%.1f"
                    ),
                    fmt(
                        w.tyreCoreTemperature or w.tyreTemperature,
                        "%.1f"
                    ),
                    fmt(
                        w.tireCoreTemperature or w.tireTemperature,
                        "%.1f"
                    )
                )
            )
        end
    end
end

--============================================================
-- Debug string
--============================================================

function M.debugStr(car)
    car =
        car
        or
        ac.getCar(0)

    if car then
        M.update(
            0.016,
            car
        )
    end

    if not car then
        return "Wheel audit: car NIL"
    end

    if not state.wheelsValid then
        return "Wheel audit: wheels NIL"
    end

    local out = {}

    out[#out + 1] =
        string.format(
            "Status %s / Count %.0f / Wheels %s",
            tostring(state.status),
            state.updateCount or 0,
            state.wheelsValid and "OK" or "NIL"
        )

    for i = 0, 3 do
        local name =
            WHEEL_NAME[i]
            or tostring(i)

        local w =
            state.wheel[i]

        if state.wheelExists[i] then
            out[#out + 1] =
                string.format(
                    "%s L:%s NL:%s SR:%s SA:%s W:%s",
                    name,
                    fmt(
                        w.load,
                        "%.0f"
                    ),
                    fmt(
                        w.normalLoad,
                        "%.0f"
                    ),
                    fmt(
                        w.slipRatio,
                        "%.3f"
                    ),
                    fmt(
                        w.slipAngle,
                        "%.3f"
                    ),
                    fmt(
                        w.angularSpeed,
                        "%.2f"
                    )
                )
        else
            out[#out + 1] =
                name .. " NIL"
        end
    end

    return table.concat(
        out,
        "\n"
    )
end

return M