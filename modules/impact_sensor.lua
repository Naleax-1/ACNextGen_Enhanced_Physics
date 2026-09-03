---@diagnostic disable: undefined-global

--============================================================
-- ACNeXtGen
-- impact_sensor.lua
-- Phase 11.1
-- Impact Sensor
-- G衝撃検出 → damage_stateへ送信
--============================================================

-- 保守型V1.1追記:
--   元の G衝撃検出を維持し、
--   tire / suspension / chassis / body / brake / thermal のroot impactを追加する。

local M = {}

--============================================================
-- Parameters
--============================================================

M.params = {
    gravity = 9.81,

    impactThreshold = 3.0,

    impactGain = 0.05,

    verticalWeight     = 1.00,
    lateralWeight      = 0.70,
    longitudinalWeight = 0.50,

    wheelDamageLoadThreshold = 1000.0,
    wheelDamageScale = 0.50,

    --------------------------------------------------------
    -- V1.1 preserve-mode additions
    --------------------------------------------------------

    tireHopImpactGain = 0.08,
    tireHopLoadImpactGain = 0.000030,
    contactDropImpactGain = 0.06,
    suspensionPulseImpactGain = 0.000025,

    chassisEnergyImpactGain = 0.06,
    bodyRuntimeImpactGain = 0.06,
    brakeLockImpactGain = 0.05,
    thermalImpactGain = 0.04,

    maxRootImpact = 0.45,

    debugStoreInterval = 0.25,
}

--============================================================
-- Dependencies
--============================================================

local function safeRequireDamage()
    local ok, mod =
        pcall(
            require,
            "modules.damage_state"
        )

    if ok and mod then
        return true, mod
    end

    ok, mod =
        pcall(
            require,
            "modules/damage_state"
        )

    if ok and mod then
        return true, mod
    end

    return false, nil
end

local okDamage, damage =
    safeRequireDamage()

--============================================================
-- State
--============================================================

local state = {
    g = 0.0,

    verticalG     = 0.0,
    lateralG      = 0.0,
    longitudinalG = 0.0,

    impact = 0.0,
    rootImpact = 0.0,

    type = "NONE",

    damageLinked = okDamage,

    ax = 0.0,
    ay = 0.0,
    az = 0.0,

    wheelLoad = {
        [0] = 0.0,
        [1] = 0.0,
        [2] = 0.0,
        [3] = 0.0,
    },

    wheelDamaged = {
        [0] = 0,
        [1] = 0,
        [2] = 0,
        [3] = 0,
    },

    tireLinked = false,
    suspensionLinked = false,
    chassisLinked = false,
    bodyLinked = false,
    brakeLinked = false,
    thermalLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,

    status = "INIT",
    updateCount = 0,
}

M.debug = state

--============================================================
-- Utility
--============================================================

local function safeNumber(value, defaultValue)
    return tonumber(value) or defaultValue or 0.0
end


local function safeLoadRaw(key)
    if not ac or not ac.load then
        return nil
    end

    local ok, value =
        pcall(
            function()
                return ac.load(key)
            end
        )

    if not ok then
        return nil
    end

    return value
end

local function safeStore(key, value)
    if not ac or not ac.store then
        return
    end

    pcall(
        function()
            ac.store(key, value)
        end
    )
end

local function clamp(value, minValue, maxValue)
    value =
        safeNumber(
            value,
            0.0
        )

    if value < minValue then
        return minValue
    end

    if value > maxValue then
        return maxValue
    end

    return value
end

local function updateDebugGate(dt)
    state.debugStoreTimer =
        (state.debugStoreTimer or 0.0)
        +
        (tonumber(dt) or 0.0)

    if state.debugStoreTimer >= (M.params.debugStoreInterval or 0.25) then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    else
        state.debugStoreNow = false
    end
end

local function safeCall(fn, ...)
    if type(fn) ~= "function" then
        return false
    end

    local ok =
        pcall(
            fn,
            ...
        )

    return ok
end

local function getAcceleration(car)
    local ax = 0.0
    local ay = 0.0
    local az = 0.0

    if car and car.acceleration then
        ax =
            safeNumber(
                car.acceleration.x,
                0.0
            )

        ay =
            safeNumber(
                car.acceleration.y,
                0.0
            )

        az =
            safeNumber(
                car.acceleration.z,
                0.0
            )
    end

    return ax, ay, az
end

local function getWheelLoad(wheel)
    if not wheel then
        return 0.0
    end

    if wheel.load ~= nil then
        return safeNumber(
            wheel.load,
            0.0
        )
    end

    if wheel.loadK ~= nil then
        return safeNumber(
            wheel.loadK,
            0.0
        )
    end

    return 0.0
end

--============================================================
-- Core
--============================================================

local function calculateImpact(ax, ay, az)
    local p =
        M.params

    local verticalG =
        math.abs(
            az / p.gravity
        )

    local lateralG =
        math.abs(
            ax / p.gravity
        )

    local longitudinalG =
        math.abs(
            ay / p.gravity
        )

    --------------------------------------------------------
    -- Impact priority
    -- 着地 > 横衝撃 > 縦衝撃
    --------------------------------------------------------

    local weightedVertical =
        verticalG
        *
        p.verticalWeight

    local weightedLateral =
        lateralG
        *
        p.lateralWeight

    local weightedLongitudinal =
        longitudinalG
        *
        p.longitudinalWeight

    local impactG =
        math.max(
            weightedVertical,
            weightedLateral,
            weightedLongitudinal
        )

    local impactType =
        "NONE"

    if impactG > p.impactThreshold then
        if impactG == weightedVertical then
            impactType = "VERTICAL"

        elseif impactG == weightedLateral then
            impactType = "LATERAL"

        elseif impactG == weightedLongitudinal then
            impactType = "LONGITUDINAL"
        end
    end

    local impact = 0.0

    if impactG > p.impactThreshold then
        impact =
            (
                impactG
                -
                p.impactThreshold
            )
            *
            p.impactGain
    end

    return
        impact,
        impactG,
        verticalG,
        lateralG,
        longitudinalG,
        impactType
end


local function readRootImpact()
    local root = 0.0
    local tireLinked = false
    local suspensionLinked = false

    for i = 0, 3 do
        local hop =
            safeLoadRaw("ngp_tire_hop_energy_" .. i)
            or
            safeLoadRaw("ngp_tire_hop_" .. i)

        local hopLoad =
            safeLoadRaw("ngp_tire_hop_loadpulse_" .. i)
            or
            safeLoadRaw("ngp_tirehop_" .. i)

        local contactDrop =
            safeLoadRaw("ngp_susp_contact_drop_" .. i)
            or
            safeLoadRaw("ngp_sci_drop_" .. i)

        local susp =
            safeLoadRaw("ngp_susp_integrated_force_" .. i)
            or
            safeLoadRaw("ngp_susp_int_force_" .. i)
            or
            safeLoadRaw("ngp_susp_" .. i)

        if hop ~= nil or hopLoad ~= nil then
            tireLinked = true
        end

        if contactDrop ~= nil or susp ~= nil then
            suspensionLinked = true
        end

        root =
            root
            +
            clamp(safeNumber(hop, 0.0), 0.0, 1.0) * M.params.tireHopImpactGain
            +
            math.min(math.abs(safeNumber(hopLoad, 0.0)) * M.params.tireHopLoadImpactGain, 0.20)
            +
            clamp(safeNumber(contactDrop, 0.0), 0.0, 1.0) * M.params.contactDropImpactGain
            +
            math.min(math.abs(safeNumber(susp, 0.0)) * M.params.suspensionPulseImpactGain, 0.20)
    end

    state.tireLinked = tireLinked
    state.suspensionLinked = suspensionLinked

    local chassis =
        safeLoadRaw("ngp_chassis_energy")
        or
        safeLoadRaw("ngp_chassis_body_energy")

    state.chassisLinked =
        chassis ~= nil

    root =
        root
        +
        clamp(safeNumber(chassis, 0.0), 0.0, 1.0) * M.params.chassisEnergyImpactGain

    local body =
        safeLoadRaw("ngp_body_runtime_flex_penalty")

    state.bodyLinked =
        body ~= nil

    root =
        root
        +
        clamp(safeNumber(body, 0.0), 0.0, 0.55) * M.params.bodyRuntimeImpactGain

    local lockSum = 0.0
    local brakeLinked = false

    for i = 0, 3 do
        local lock =
            safeLoadRaw("ngp_brake_lock_" .. i)

        if lock ~= nil then
            brakeLinked = true
        end

        lockSum =
            lockSum
            +
            clamp(safeNumber(lock, 0.0), 0.0, 1.0)
    end

    state.brakeLinked = brakeLinked

    root =
        root
        +
        lockSum
        *
        0.25
        *
        M.params.brakeLockImpactGain

    local thermal =
        safeLoadRaw("ngp_brake_root_heat_avg")
        or
        safeLoadRaw("ngp_virtual_thermal_stress")

    state.thermalLinked =
        thermal ~= nil

    root =
        root
        +
        clamp(safeNumber(thermal, 0.0), 0.0, 1.0) * M.params.thermalImpactGain

    return
        clamp(
            root,
            0.0,
            M.params.maxRootImpact
        )
end

local function applyDamage(car, impact)
    if impact <= 0 then
        return
    end

    if not okDamage or not damage then
        state.status = "NO DAMAGE_STATE"
        return
    end

    --------------------------------------------------------
    -- Chassis damage
    --------------------------------------------------------

    safeCall(
        damage.damageChassis,
        impact
    )

    --------------------------------------------------------
    -- Wheel damage
    --------------------------------------------------------

    if not car.wheels then
        return
    end

    for i = 0, 3 do
        state.wheelDamaged[i] = 0

        local wheel =
            car.wheels[i]

        if wheel then
            local load =
                getWheelLoad(
                    wheel
                )

            state.wheelLoad[i] =
                load

            if load > M.params.wheelDamageLoadThreshold then
                local ok =
                    safeCall(
                        damage.damageWheel,
                        i,
                        impact
                        *
                        M.params.wheelDamageScale
                    )

                state.wheelDamaged[i] =
                    ok and 1 or 0
            end
        else
            state.wheelLoad[i] =
                0.0
        end
    end
end

--============================================================
-- Export
--============================================================

local function exportState()
    ac.store(
        "ngp_impact_status",
        state.status or "UNKNOWN"
    )

    ac.store(
        "ngp_impact_update_count",
        state.updateCount or 0
    )

    ac.store(
        "ngp_impact_g",
        state.g or 0.0
    )

    ac.store(
        "ngp_impact_value",
        state.impact or 0.0
    )

    safeStore(
        "ngp_impact_root_value",
        state.rootImpact or 0.0
    )

    ac.store(
        "ngp_impact_type",
        state.type or "NONE"
    )

    ac.store(
        "ngp_impact_vertical_g",
        state.verticalG or 0.0
    )

    ac.store(
        "ngp_impact_lateral_g",
        state.lateralG or 0.0
    )

    ac.store(
        "ngp_impact_longitudinal_g",
        state.longitudinalG or 0.0
    )

    ac.store(
        "ngp_impact_ax",
        state.ax or 0.0
    )

    ac.store(
        "ngp_impact_ay",
        state.ay or 0.0
    )

    ac.store(
        "ngp_impact_az",
        state.az or 0.0
    )

    ac.store(
        "ngp_impact_damage_linked",
        state.damageLinked and 1 or 0
    )

    for i = 0, 3 do
        ac.store(
            "ngp_impact_wheel_load_" .. i,
            state.wheelLoad[i] or 0.0
        )

        ac.store(
            "ngp_impact_wheel_damaged_" .. i,
            state.wheelDamaged[i] or 0
        )
    end

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_impact_tire_linked", state.tireLinked and 1 or 0)
    safeStore("ngp_impact_suspension_linked", state.suspensionLinked and 1 or 0)
    safeStore("ngp_impact_chassis_linked", state.chassisLinked and 1 or 0)
    safeStore("ngp_impact_body_linked", state.bodyLinked and 1 or 0)
    safeStore("ngp_impact_brake_linked", state.brakeLinked and 1 or 0)
    safeStore("ngp_impact_thermal_linked", state.thermalLinked and 1 or 0)
end

--============================================================
-- Update
--============================================================

function M.update(dt, car)
    state.updateCount =
        state.updateCount
        +
        1

    dt = tonumber(dt) or 0.0

    if dt <= 0 then
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
        exportState()
        return
    end

    state.status = "RUNNING"

    --------------------------------------------------------
    -- Dependency refresh
    -- damage_state が後から読み込まれる場合に備える
    --------------------------------------------------------

    if not okDamage or not damage then
        okDamage, damage =
            safeRequireDamage()
    end

    state.damageLinked =
        okDamage
        and
        damage ~= nil

    --------------------------------------------------------
    -- Acceleration
    --------------------------------------------------------

    local ax, ay, az =
        getAcceleration(
            car
        )

    state.ax = ax
    state.ay = ay
    state.az = az

    --------------------------------------------------------
    -- Impact calculation
    --------------------------------------------------------

    local impact,
          impactG,
          verticalG,
          lateralG,
          longitudinalG,
          impactType =
        calculateImpact(
            ax,
            ay,
            az
        )

    state.g =
        impactG

    state.verticalG =
        verticalG

    state.lateralG =
        lateralG

    state.longitudinalG =
        longitudinalG

    state.rootImpact =
        readRootImpact()

    if state.rootImpact > impact then
        impact =
            state.rootImpact

        if impactType == "NONE" then
            impactType = "ROOT"
        end
    end

    state.impact =
        impact

    state.type =
        impactType

    --------------------------------------------------------
    -- Damage transfer
    --------------------------------------------------------

    applyDamage(
        car,
        impact
    )

    --------------------------------------------------------
    -- Export
    --------------------------------------------------------

    exportState()
end

--============================================================
-- Public API
--============================================================

function M.getImpact()
    return state.impact or 0.0
end

function M.getImpactG()
    return state.g or 0.0
end

function M.getType()
    return state.type or "NONE"
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f Damage:%s\n" ..
        "Impact %.4f Root %.4f Type %s\n" ..
        "G %.2f V/L/Long %.2f %.2f %.2f\n" ..
        "Links Tire:%s Susp:%s Chassis:%s Body:%s Brake:%s Therm:%s",

        tostring(state.status),
        state.updateCount or 0,
        state.damageLinked and "OK" or "NIL",

        state.impact or 0.0,
        state.rootImpact or 0.0,
        tostring(state.type),

        state.g or 0.0,
        state.verticalG or 0.0,
        state.lateralG or 0.0,
        state.longitudinalG or 0.0,

        state.tireLinked and "OK" or "NIL",
        state.suspensionLinked and "OK" or "NIL",
        state.chassisLinked and "OK" or "NIL",
        state.bodyLinked and "OK" or "NIL",
        state.brakeLinked and "OK" or "NIL",
        state.thermalLinked and "OK" or "NIL"
    )
end

return M
