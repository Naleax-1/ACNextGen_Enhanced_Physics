---@diagnostic disable: undefined-global

--============================================================
-- ACNextGen.lua
-- ACNextGen V1.1.6
-- Performance Recovery / Strict Struct Safe Runtime
--============================================================

local APP_NAME = "ACNextGen"
local VERSION  = "V1.1.6 Performance Recovery / Struct Safe"

local RUNTIME_STORE_INTERVAL = 0.25
local PROFILE_EXPORT_INTERVAL = 2.00
local PROFILE_SAMPLE_INTERVAL = 0.50
local ERROR_SCAN_INTERVAL = 0.25
local GC_STEP_INTERVAL = 0.50

--============================================================
-- Runtime state
--============================================================

local runtime = {
    frame = 0,
    time = 0.0,

    carOK = false,
    wheelsOK = false,

    loadedCount = 0,
    enabledCount = 0,
    activeErrorCount = 0,
    totalErrorCount = 0,

    -- Kept for observer compatibility.
    errorCount = 0,
    lastError = "",

    moduleStatus = {},
    moduleErrors = {},

    initialized = false,
}

local timers = {
    runtimeStore = 0.0,
    profileStore = 0.0,
    profileSample = 0.0,
    errorScan = 0.0,
    gc = 0.0,
}

--============================================================
-- Helpers
--============================================================

local function num(v, fallback)
    local n = tonumber(v)
    if n == nil or n ~= n then
        return fallback or 0.0
    end
    return n
end

local function bool01(v)
    return v and 1 or 0
end

local function safeName(name)
    local s = tostring(name or "unknown")
    return s:gsub("[^%w_]", "_")
end

local function log(msg)
    if ac and ac.log then
        pcall(ac.log, "[" .. APP_NAME .. "] " .. tostring(msg))
    end
end

local function safeStore(key, value)
    if not ac or not ac.store then
        return false
    end

    local ok = pcall(ac.store, key, value)

    return ok
end

local function safeClock()
    if os and os.clock then
        local ok, value = pcall(os.clock)
        if ok and value then
            return value
        end
    end
    return 0.0
end

local function safeGetCar()
    if not ac or not ac.getCar then
        return nil
    end

    local ok, car = pcall(ac.getCar, 0)

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

local function shortError(value)
    local s = tostring(value or "")
    s = s:gsub("\\", "/")

    local fileLine = s:match("([^/]+%.lua:%d+.*)$")
    if fileLine then
        return fileLine
    end

    if #s > 160 then
        return s:sub(1, 157) .. "..."
    end

    return s
end

--============================================================
-- Safe require
--============================================================

local requireAliases = {
    tire_memory = { "tyre_memory" },
}

local function buildRequireCandidates(name)
    local list = { tostring(name or "") }
    local aliases = requireAliases[name]

    if aliases then
        for i = 1, #aliases do
            list[#list + 1] = aliases[i]
        end
    end

    return list
end

local function tryRequireName(name)
    local pathDot = "modules." .. name
    local pathSlash = "modules/" .. name

    local ok, mod = pcall(require, pathDot)
    if ok and mod then
        return mod, nil
    end

    local errDot = mod

    ok, mod = pcall(require, pathSlash)
    if ok and mod then
        return mod, nil
    end

    local errSlash = mod

    return nil, tostring(errDot) .. " | " .. tostring(errSlash)
end

local function safeRequire(name)
    local candidates = buildRequireCandidates(name)
    local errors = {}

    for i = 1, #candidates do
        local candidate = candidates[i]
        local mod, err = tryRequireName(candidate)

        if mod then
            return mod, nil
        end

        errors[#errors + 1] =
            tostring(candidate) .. ": " .. tostring(err)
    end

    return nil, table.concat(errors, " || ")
end

--============================================================
-- Module definitions
--============================================================

local moduleDefs = {
    { name = "observer",                  enabled = true,  critical = true,  diagnostic = true  },

    { name = "body_rigidity_estimator",   enabled = true,  critical = false, diagnostic = false },
    { name = "load_transfer",             enabled = true,  critical = false, diagnostic = false },
    { name = "road_body_input",           enabled = true,  critical = false, diagnostic = false },

    { name = "suspension_contact_input",  enabled = true,  critical = false, diagnostic = false },
    { name = "damper_model",              enabled = true,  critical = false, diagnostic = false },
    { name = "damper_hysteresis",         enabled = true,  critical = false, diagnostic = false },
    { name = "progressive_spring",        enabled = true,  critical = false, diagnostic = false },
    { name = "sprung_mass",               enabled = true,  critical = false, diagnostic = false },
    { name = "weight_distribution",       enabled = true,  critical = false, diagnostic = false },
    { name = "suspension",                enabled = true,  critical = false, diagnostic = false },

    { name = "arm_compliance",            enabled = true,  critical = false, diagnostic = false },
    { name = "control_arm",               enabled = true,  critical = false, diagnostic = false },
    { name = "ultra_chassis",             enabled = true,  critical = false, diagnostic = false },
    { name = "caster_effect",             enabled = true,  critical = false, diagnostic = false },
    { name = "compliance_stack",          enabled = true,  critical = false, diagnostic = false },

    { name = "tire_contact_core",         enabled = true,  critical = false, diagnostic = false },
    { name = "tire_contact_response",     enabled = true,  critical = false, diagnostic = false },
    { name = "tire_carcass",              enabled = true,  critical = false, diagnostic = false },
    { name = "contact_quality",           enabled = true,  critical = false, diagnostic = false },
    { name = "tire_state",                enabled = true,  critical = false, diagnostic = false },
    { name = "tire_memory",               enabled = true,  critical = false, diagnostic = false },
    { name = "tire_thermal_brush",        enabled = true,  critical = false, diagnostic = false },
    { name = "tire_contact",              enabled = true,  critical = false, diagnostic = false },
    { name = "tire_dynamics",             enabled = true,  critical = false, diagnostic = false },
    { name = "tire_force",                enabled = true,  critical = false, diagnostic = false },
    { name = "tire_compliance",           enabled = true,  critical = false, diagnostic = false },

    { name = "load_path",                 enabled = true,  critical = false, diagnostic = false },
    { name = "road_input_interpreter",    enabled = true,  critical = false, diagnostic = false },

    { name = "diff_lsd",                  enabled = true,  critical = false, diagnostic = false },
    { name = "drivetrain",                enabled = true,  critical = false, diagnostic = false },
    { name = "driveline_windup",          enabled = true,  critical = false, diagnostic = false },
    { name = "tire_hop",                  enabled = true,  critical = false, diagnostic = false },

    { name = "slip_recovery",             enabled = true,  critical = false, diagnostic = false },
    { name = "yaw_moment_budget",         enabled = true,  critical = false, diagnostic = false },

    { name = "thermal",                   enabled = true,  critical = false, diagnostic = false },

    { name = "mass_balance",              enabled = true,  critical = false, diagnostic = false },
    { name = "chassis_roll",              enabled = true,  critical = false, diagnostic = false },
    { name = "chassis_energy",            enabled = true,  critical = false, diagnostic = false },
    { name = "chassis_flex",              enabled = true,  critical = false, diagnostic = false },
    { name = "virtual_inertia",           enabled = true,  critical = false, diagnostic = false },

    { name = "steering_dynamics",         enabled = true,  critical = false, diagnostic = false },
    { name = "steering_mechanism",        enabled = false, critical = false, diagnostic = false },

    { name = "brake_system",              enabled = true,  critical = false, diagnostic = false },
    { name = "brake_fade",                enabled = true,  critical = false, diagnostic = false },
    { name = "brake_lock",                enabled = true,  critical = false, diagnostic = false },

    { name = "damage_state",              enabled = true,  critical = false, diagnostic = false },
    { name = "damage_event",              enabled = true,  critical = false, diagnostic = false },
    { name = "impact_sensor",             enabled = false, critical = false, diagnostic = false },
    { name = "impact_state",              enabled = false, critical = false, diagnostic = false },
    { name = "vehicle_condition",         enabled = true,  critical = false, diagnostic = false },

    { name = "physics",                   enabled = true,  critical = false, diagnostic = false },
    { name = "wheel_audit",               enabled = false, critical = false, diagnostic = true  },
}

-- Module update rates. Fast physical paths stay at 60 Hz; slow memory/thermal/diagnostic paths are reduced.
local moduleHz = {
    observer = 5,
    body_rigidity_estimator = 15,
    load_transfer = 60,
    road_body_input = 60,
    suspension_contact_input = 60,
    damper_model = 60,
    damper_hysteresis = 60,
    progressive_spring = 60,
    sprung_mass = 30,
    weight_distribution = 30,
    suspension = 60,
    arm_compliance = 60,
    control_arm = 60,
    ultra_chassis = 30,
    caster_effect = 30,
    compliance_stack = 60,
    tire_contact_core = 60,
    tire_contact_response = 60,
    tire_carcass = 60,
    contact_quality = 60,
    tire_state = 60,
    tire_memory = 30,
    tire_thermal_brush = 20,
    tire_contact = 60,
    tire_dynamics = 60,
    tire_force = 60,
    tire_compliance = 60,
    load_path = 60,
    road_input_interpreter = 60,
    diff_lsd = 60,
    drivetrain = 60,
    driveline_windup = 60,
    tire_hop = 60,
    slip_recovery = 60,
    yaw_moment_budget = 60,
    thermal = 10,
    mass_balance = 30,
    chassis_roll = 30,
    chassis_energy = 30,
    chassis_flex = 30,
    virtual_inertia = 30,
    steering_dynamics = 60,
    steering_mechanism = 30,
    brake_system = 60,
    brake_fade = 10,
    brake_lock = 60,
    damage_state = 10,
    damage_event = 10,
    impact_sensor = 30,
    impact_state = 30,
    vehicle_condition = 10,
    physics = 30,
    wheel_audit = 2,
}

local modules = {}
local observerModule = nil

local profile = {
    worstEverName = "none",
    worstEverMs = 0.0,
    maxMs = {},
    lastMs = {},
    avgMs = {},
    count = {},
}

--============================================================
-- Status helpers
--============================================================

local function setRuntimeError(name, err)
    local msg = tostring(name or "unknown") .. ": " .. tostring(err or "")
    runtime.lastError = msg
    runtime.totalErrorCount = (runtime.totalErrorCount or 0) + 1
    log(shortError(msg))
end

local function setEntryStatus(entry, status, err)
    if not entry then
        return
    end

    entry.lastStatus = tostring(status or "UNKNOWN")

    if err and tostring(err) ~= "" then
        local msg = tostring(err)
        entry.error = msg

        if entry.lastLoggedError ~= msg then
            entry.lastLoggedError = msg
            setRuntimeError(entry.name, msg)
        end
    else
        entry.error = ""
    end

    runtime.moduleStatus[entry.name] = entry.lastStatus
    runtime.moduleErrors[entry.name] = entry.error or ""
end

local function updateActiveErrorCount()
    local count = 0

    for i = 1, #modules do
        local entry = modules[i]
        if entry then
            local status = tostring(entry.lastStatus or "")
            local err = tostring(entry.error or "")

            if err ~= ""
            or status == "ERROR"
            or status == "LOAD ERROR"
            or status == "INIT ERROR" then
                count = count + 1
            end
        end
    end

    runtime.activeErrorCount = count
    runtime.errorCount = count
end

--============================================================
-- Module loading
--============================================================

local function callInit(entry)
    if not entry or not entry.module then
        return
    end

    if type(entry.module.init) ~= "function" then
        return
    end

    local ok, err = pcall(entry.module.init)

    if not ok then
        setEntryStatus(entry, "INIT ERROR", err)
    end
end

local function loadModules()
    modules = {}
    observerModule = nil

    runtime.loadedCount = 0
    runtime.enabledCount = 0
    runtime.activeErrorCount = 0
    runtime.errorCount = 0
    runtime.lastError = ""
    runtime.moduleStatus = {}
    runtime.moduleErrors = {}
    runtime.initialized = false

    for i = 1, #moduleDefs do
        local def = moduleDefs[i]

        local entry = {
            name = def.name,
            module = nil,

            enabled = def.enabled == true,
            critical = def.critical == true,
            diagnostic = def.diagnostic == true,

            loaded = false,
            loadError = "",

            updateCount = 0,
            skippedCount = 0,
            targetHz = moduleHz[def.name] or 60,
            interval = 1.0 / math.max(moduleHz[def.name] or 60, 1),
            accumulator = 1.0 / math.max(moduleHz[def.name] or 60, 1),
            error = "",
            lastStatus = "INIT",
            lastLoggedError = "",
        }

        if not entry.enabled then
            setEntryStatus(entry, "DISABLED", nil)
            modules[#modules + 1] = entry
        else
            local mod, err = safeRequire(def.name)

            entry.module = mod
            entry.loaded = mod ~= nil
            entry.loadError = err or ""

            if entry.loaded then
                runtime.loadedCount = runtime.loadedCount + 1
                runtime.enabledCount = runtime.enabledCount + 1

                setEntryStatus(entry, "READY", nil)

                if entry.name == "observer" then
                    observerModule = entry.module
                end

                callInit(entry)
            else
                entry.enabled = false
                setEntryStatus(entry, "LOAD ERROR", err)
            end

            modules[#modules + 1] = entry
        end
    end

    updateActiveErrorCount()
    runtime.initialized = true
end

--============================================================
-- Safe update
--============================================================

local function updateProfile(name, elapsedMs)
    profile.lastMs[name] = elapsedMs

    profile.maxMs[name] =
        math.max(
            profile.maxMs[name] or 0.0,
            elapsedMs
        )

    profile.count[name] =
        (profile.count[name] or 0) + 1

    local oldAvg =
        profile.avgMs[name]
        or
        elapsedMs

    profile.avgMs[name] =
        oldAvg + (elapsedMs - oldAvg) * 0.05
end

local function safeUpdate(entry, dt, car, profileNow)
    if not entry then
        return
    end

    if entry.enabled == false then
        if entry.lastStatus ~= "LOAD ERROR" and entry.lastStatus ~= "DISABLED" then
            setEntryStatus(entry, "DISABLED", nil)
        end
        return
    end

    if not entry.module then
        setEntryStatus(entry, "NO MODULE", nil)
        return
    end

    if type(entry.module.update) ~= "function" then
        if entry.lastStatus ~= "NO UPDATE" then
            setEntryStatus(entry, "NO UPDATE", nil)
        end
        return
    end

    if not car and not entry.critical and not entry.diagnostic then
        if entry.lastStatus ~= "NO CAR" then
            setEntryStatus(entry, "NO CAR", nil)
        end
        return
    end

    local interval = entry.interval or 0.0
    local updateDt = dt

    if interval > 0.0 then
        entry.accumulator = (entry.accumulator or 0.0) + dt

        if entry.accumulator + 0.000001 < interval then
            entry.skippedCount = (entry.skippedCount or 0) + 1
            return
        end

        updateDt = math.min(entry.accumulator, 0.050)
        entry.accumulator = entry.accumulator - interval

        -- Prevent a long pause from causing a catch-up storm.
        if entry.accumulator > interval * 2.0 then
            entry.accumulator = interval
        elseif entry.accumulator < 0.0 then
            entry.accumulator = 0.0
        end
    end

    local ok, err
    local elapsedMs = 0.0

    if profileNow then
        local t0 = safeClock()
        ok, err = pcall(entry.module.update, updateDt, car, runtime)
        elapsedMs = (safeClock() - t0) * 1000.0
        updateProfile(entry.name or "unknown", elapsedMs)
    else
        ok, err = pcall(entry.module.update, updateDt, car, runtime)
    end

    if ok then
        entry.lastStatus = "OK"
        entry.error = ""
        runtime.moduleStatus[entry.name] = "OK"
        runtime.moduleErrors[entry.name] = ""
        entry.updateCount = (entry.updateCount or 0) + 1
    else
        setEntryStatus(entry, "ERROR", err)
    end
end

--============================================================
-- Runtime stores
--============================================================

local function storeRuntime()
    safeStore("ngp_runtime_frame", runtime.frame)
    safeStore("ngp_runtime_time", runtime.time)

    safeStore("ngp_runtime_car_ok", bool01(runtime.carOK))
    safeStore("ngp_runtime_wheels_ok", bool01(runtime.wheelsOK))

    safeStore("ngp_runtime_loaded_count", runtime.loadedCount)
    safeStore("ngp_runtime_enabled_count", runtime.enabledCount)
    safeStore("ngp_runtime_error_count", runtime.errorCount)
    safeStore("ngp_runtime_active_error_count", runtime.activeErrorCount)
    safeStore("ngp_runtime_total_error_count", runtime.totalErrorCount)

    safeStore("ngp_runtime_last_error", runtime.lastError or "")
    safeStore("ngp_runtime_version", VERSION)
    safeStore("ngp_runtime_scheduler", "60/30/20/15/10/5Hz")
    safeStore(
        "ngp_runtime_root_order",
        "DAMPER>COMPLIANCE>CARCASS>CONTACT>MEMORY>LOADPATH>ROAD>SLIP>DRIVELINE>YAW"
    )
end

local function storeProfile()
    local worstName = "none"
    local worstMs = 0.0

    for name, ms in pairs(profile.lastMs) do
        if ms > worstMs then
            worstMs = ms
            worstName = name
        end

        safeStore("ngp_prof_last_" .. safeName(name), ms)
        safeStore("ngp_prof_avg_" .. safeName(name), profile.avgMs[name] or 0.0)
        safeStore("ngp_prof_max_" .. safeName(name), profile.maxMs[name] or 0.0)
    end

    if worstMs > (profile.worstEverMs or 0.0) then
        profile.worstEverMs = worstMs
        profile.worstEverName = worstName
    end

    safeStore("ngp_prof_worst_name", worstName)
    safeStore("ngp_prof_worst_ms", worstMs)
    safeStore("ngp_prof_worst_ever_name", profile.worstEverName or "none")
    safeStore("ngp_prof_worst_ever_ms", profile.worstEverMs or 0.0)
end

local function stepGC(dt)
    timers.gc = timers.gc + dt

    if timers.gc < GC_STEP_INTERVAL then
        return
    end

    timers.gc = 0.0

    if collectgarbage then
        pcall(collectgarbage, "step", 16)
    end
end

--============================================================
-- Main update
--============================================================

function update(dt)
    dt = num(dt, 0.0)

    if dt <= 0.0 then
        dt = 0.001
    end

    runtime.frame = runtime.frame + 1
    runtime.time = runtime.time + dt

    local car = safeGetCar()

    runtime.carOK = car ~= nil
    runtime.wheelsOK = hasWheels(car)

    timers.profileSample = timers.profileSample + dt
    local profileNow = false
    if timers.profileSample >= PROFILE_SAMPLE_INTERVAL then
        timers.profileSample = 0.0
        profileNow = true
    end

    for i = 1, #modules do
        safeUpdate(modules[i], dt, car, profileNow)
    end

    timers.errorScan = timers.errorScan + dt
    if timers.errorScan >= ERROR_SCAN_INTERVAL then
        timers.errorScan = 0.0
        updateActiveErrorCount()
    end

    timers.profileStore = timers.profileStore + dt
    if timers.profileStore >= PROFILE_EXPORT_INTERVAL then
        timers.profileStore = 0.0
        storeProfile()
    end

    timers.runtimeStore = timers.runtimeStore + dt
    if timers.runtimeStore >= RUNTIME_STORE_INTERVAL then
        timers.runtimeStore = 0.0
        storeRuntime()
    end

    stepGC(dt)
end

--============================================================
-- UI
--============================================================

local function drawFallbackUI()
    ui.text("ACNextGen")
    ui.text(VERSION)
    ui.separator()

    ui.text("observer.drawUI not found")
    ui.separator()

    ui.text("Runtime:")
    ui.text("Car OK: " .. tostring(runtime.carOK))
    ui.text("Wheels OK: " .. tostring(runtime.wheelsOK))
    ui.text("Loaded Modules: " .. tostring(runtime.loadedCount))
    ui.text("Enabled Modules: " .. tostring(runtime.enabledCount))
    ui.text("Active Errors: " .. tostring(runtime.errorCount))
    ui.text("Total Errors: " .. tostring(runtime.totalErrorCount))

    if runtime.lastError ~= "" then
        ui.separator()
        ui.text("Last Error:")
        ui.text(shortError(runtime.lastError))
    end
end

function windowMain()
    if observerModule
    and type(observerModule.drawUI) == "function" then
        local ok, err =
            pcall(
                observerModule.drawUI,
                runtime,
                modules
            )

        if not ok then
            setRuntimeError("observer.drawUI", err)

            ui.text("ACNextGen observer draw error")
            ui.text(shortError(err))
        end
    else
        drawFallbackUI()
    end
end

--============================================================
-- App init
--============================================================

loadModules()
storeRuntime()

log(VERSION .. " loaded")
