---@diagnostic disable: undefined-global

--============================================================
-- observer.lua
-- ACNextGen Observer / Compact UI v1.1.4 One Point One Integration
-- Runtime-driven UI version
--
-- 目的:
--   ・重複表示を削除
--   ・Tire → Suspension → Body の統合ラインを見やすくする
--   ・各モジュールの debugStr() を安全に呼ぶ
--   ・observer自身に不要な state/debugStr を持たせない
--============================================================

local M = {}

-- Keep the on-screen monitor lightweight during normal driving. Set false for full development diagnostics.
local PERFORMANCE_LITE = true

--============================================================
-- Optional extended observers
--============================================================

local diff_lsd_observer = nil

do
    local ok,
          module =
        pcall(
            require,
            'modules.diff_lsd_observer'
        )

    if ok and module then
        diff_lsd_observer =
            module

        if ac and ac.log then
            ac.log("[ACNextGen] diff_lsd_observer linked")
        end
    else
        diff_lsd_observer =
            nil

        if ac and ac.log then
            ac.log("[ACNextGen] diff_lsd_observer not found")
        end
    end
end


local frameCount = 0
local logTimer = 0
local LOG_INTERVAL = 1.0

--============================================================
-- UI definitions
--============================================================

local TITLES = {
    wheel_audit                = "DIAG: Wheel API Audit",
    physics                    = "CORE: Physics Hub",

    tire_contact_core          = "PHASE 1.0: Tire Contact Core",
    tire_contact_response      = "PHASE 1.1: Tire Contact Response",
    contact_quality            = "PHASE 1.15: Contact Quality",
    tire_state                 = "PHASE 1.2: Tire Relaxation",
    tire_memory                = "PHASE 1.3: Tire Memory",
    tire_carcass               = "PHASE 1.35: Tire Carcass",
    slip_recovery             = "PHASE 1.36: Slip Recovery",
    tire_compliance            = "PHASE 1.4: Tire Compliance",
    tire_thermal_brush         = "PHASE 1.95:  Tire Thermal Brush",
    tire_contact               = "PHASE 1.5: Contact Patch",
    tire_dynamics              = "PHASE 1.8: Tire Dynamics",
    tire_force                 = "PHASE 1.9: Tire Force",
    tire_hop                   = "PHASE 4.7: Tire Hop",

    diff_lsd                   = "PHASE 2.5: Advanced LSD",
    diff_lsd_observer          = "PHASE 2.5B: LSD Mechanical Observer",
    drivetrain                 = "PHASE 3: Drivetrain",
    driveline_windup           = "PHASE 3.1: Driveline Windup",

    suspension_contact_input   = "PHASE 4.0: Suspension Contact Input",
    suspension                 = "PHASE 4.1: Suspension Integrated",
    arm_compliance             = "PHASE 4.6: Arm Compliance",
    control_arm                = "PHASE 4.65: Control Arm",
    ultra_chassis              = "PHASE 4.7: UltraChassis Geometry",
    damper_model               = "PHASE 4.8: Damper Model",
    damper_hysteresis          = "PHASE 4.81: Damper Hysteresis",
    progressive_spring         = "PHASE 4.9: Progressive Spring",
    caster_effect              = "PHASE 4.85: Caster Effect",
    compliance_stack           = "PHASE 4.95: Compliance Stack",
    yaw_moment_budget          = "PHASE 4.99: Yaw Moment Budget",

    steering_dynamics          = "PHASE 8.1: Steering Dynamics",
    steering_mechanism         = "PHASE 8.5: Steering Mechanism",

    thermal                    = "PHASE 5: Thermal",

    body_rigidity_estimator = "PHASE 8.0B: Body Rigidity Estimator",
    load_transfer              = "PHASE 6: Load Transfer",
    load_path                  = "PHASE 6.1: Load Path",
    road_body_input = "PHASE 6.2: Road Body Input",
    road_input_interpreter   = "PHASE 6.3: Road Input Interpreter",
    sprung_mass                = "PHASE 13: Sprung Mass",
    weight_distribution        = "PHASE 15.4: Weight Distribution",
    mass_balance               = "PHASE 7: Mass Balance",
    virtual_inertia            = "PHASE 7.3: Virtual Inertia",

    chassis_flex               = "PHASE 8: Chassis Flex",
    chassis_roll               = "PHASE 8.1: Chassis Roll",
    chassis_energy             = "PHASE 16: Chassis Energy",

    brake_system               = "PHASE 10.0: Brake System",
    brake_thermal              = "PHASE 10.1: Brake Thermal",
    brake_fade                 = "PHASE 10.2: Brake Fade",
    brake_lock                 = "PHASE 10.3: Brake Lock",

    damage_event               = "PHASE 11: Damage Event",
    damage_state               = "PHASE 11.2: Damage State",
    impact_sensor              = "PHASE 11.1: Impact Sensor",
    impact_state               = "PHASE 11.3: Impact State",
    vehicle_condition          = "PHASE 12: Vehicle Condition",
}

local GROUPS = {
    {
        title = "=== CORE / DIAGNOSTIC ===",
        modules = {
            "physics",
            "wheel_audit",
        },
        wheel = false,
    },

    {
        title = "=== ROOT ORDER 1: BODY / LOAD / RAW ROAD ===",
        modules = {
            "body_rigidity_estimator",
            "load_transfer",
            "road_body_input",
        },
        wheel = false,
    },

    {
        title = "=== ROOT ORDER 2: SUSPENSION / DAMPER ===",
        modules = {
            "suspension_contact_input",
            "damper_model",
            "damper_hysteresis",
            "progressive_spring",
            "sprung_mass",
            "weight_distribution",
            "suspension",
        },
        wheel = true,
    },

    {
        title = "=== ROOT ORDER 3: GEOMETRY / COMPLIANCE ===",
        modules = {
            "arm_compliance",
            "control_arm",
            "ultra_chassis",
            "caster_effect",
            "compliance_stack",
        },
        wheel = false,
    },

    {
        title = "=== ROOT ORDER 4: TIRE / CONTACT / MEMORY ===",
        modules = {
            "tire_contact_core",
            "tire_contact_response",
            "tire_carcass",
            "contact_quality",
            "tire_state",
            "tire_memory",
            "tire_thermal_brush",
            "tire_contact",
            "tire_dynamics",
            "tire_force",
            "tire_compliance",
        },
        wheel = true,
    },

    {
        title = "=== ROOT ORDER 5: LOAD PATH / ROAD INTERPRETER ===",
        modules = {
            "load_path",
            "road_input_interpreter",
        },
        wheel = true,
    },

    {
        title = "=== ROOT ORDER 6: DIFF / DRIVELINE / HOP ===",
        modules = {
            "diff_lsd",
            "drivetrain",
            "driveline_windup",
            "tire_hop",
        },
        wheel = false,
    },

    {
        title = "=== ROOT ORDER 7: SLIP RECOVERY / YAW BUDGET ===",
        modules = {
            "slip_recovery",
            "yaw_moment_budget",
        },
        wheel = false,
    },

    {
        title = "=== THERMAL / MASS / CHASSIS ===",
        modules = {
            "thermal",
            "mass_balance",
            "chassis_roll",
            "chassis_energy",
            "chassis_flex",
            "virtual_inertia",
        },
        wheel = false,
    },

    {
        title = "=== STEERING / BRAKE / DAMAGE ===",
        modules = {
            "steering_dynamics",
            "steering_mechanism",
            "brake_system",
            "brake_thermal",
            "brake_fade",
            "brake_lock",
            "damage_event",
            "damage_state",
            "impact_sensor",
            "impact_state",
            "vehicle_condition",
        },
        wheel = false,
    },
}

local WHEEL_NAMES = {
    [0] = "FL",
    [1] = "FR",
    [2] = "RL",
    [3] = "RR",
}

--============================================================
-- Utility
--============================================================

local function num(v, defaultValue)
    local n = tonumber(v)

    if n == nil or n ~= n then
        return defaultValue or 0.0
    end

    return n
end

local function safeField(obj, field, defaultValue)
    if not obj then
        return defaultValue
    end

    local ok, result =
        pcall(
            function()
                return obj[field]
            end
        )

    if not ok or result == nil then
        return defaultValue
    end

    return result
end

local function loadNum(key, fallback)
    if not ac or not ac.load then
        return fallback or 0.0
    end

    local ok, value =
        pcall(
            function()
                return ac.load(key)
            end
        )

    if not ok or value == nil then
        return fallback or 0.0
    end

    return tonumber(value) or fallback or 0.0
end

local function loadRaw(key, fallback)
    if not ac or not ac.load then
        return fallback
    end

    local ok, value =
        pcall(
            function()
                return ac.load(key)
            end
        )

    if not ok or value == nil then
        return fallback
    end

    return value
end

local function loadNumAlt(defaultValue, ...)
    local keys = { ... }

    for i = 1, #keys do
        local value =
            loadRaw(
                keys[i],
                nil
            )

        if value ~= nil then
            local n =
                tonumber(value)

            if n ~= nil then
                return n
            end
        end
    end

    return defaultValue or 0.0
end


local MAX_TEXT_WIDTH = 92
local MAX_DEBUG_LINES = 8

local function shortenText(value, maxLen)
    local s =
        tostring(
            value or ""
        )

    maxLen =
        tonumber(
            maxLen
        )
        or
        MAX_TEXT_WIDTH

    if #s <= maxLen then
        return s
    end

    return
        string.sub(
            s,
            1,
            maxLen - 3
        )
        ..
        "..."
end

local function basenamePath(value)
    local s =
        tostring(
            value or ""
        )

    s =
        s:gsub(
            "\\",
            "/"
        )

    local name =
        s:match(
            "([^/]+%.lua:%d+.*)$"
        )

    if name then
        return name
    end

    return shortenText(
        s,
        MAX_TEXT_WIDTH
    )
end

local function drawCompactText(value, maxLines, maxLen)
    local s =
        tostring(
            value or ""
        )

    maxLines =
        tonumber(
            maxLines
        )
        or
        MAX_DEBUG_LINES

    maxLen =
        tonumber(
            maxLen
        )
        or
        MAX_TEXT_WIDTH

    local count =
        0

    for line in string.gmatch(s .. "\n", "(.-)\n") do
        count =
            count
            +
            1

        if count > maxLines then
            ui.text("...debug truncated...")
            return
        end

        ui.text(
            shortenText(
                line,
                maxLen
            )
        )
    end
end

local function textKV(label, value, fmt)
    if fmt then
        ui.text(
            string.format(
                "%-24s : " .. fmt,
                tostring(label),
                value
            )
        )
    else
        ui.text(
            string.format(
                "%-24s : %s",
                tostring(label),
                tostring(value)
            )
        )
    end
end

local function buildEntryMap(modules)
    local map = {}

    if not modules then
        return map
    end

    for i = 1, #modules do
        local entry =
            modules[i]

        if entry and entry.name then
            map[entry.name] =
                entry
        end
    end

    return map
end

local function getEntry(name, entryMap)
    if not entryMap then
        return nil
    end

    return entryMap[name]
end

local function getEntryStatus(entry)
    if not entry then
        return "NOT REGISTERED"
    end

    if entry.enabled == false then
        return "DISABLED"
    end

    if entry.loaded == false then
        return "LOAD ERROR"
    end

    if entry.lastStatus then
        return tostring(entry.lastStatus)
    end

    return "UNKNOWN"
end

local function safeDebugStr(module, index)
    if not module or type(module.debugStr) ~= "function" then
        return false, "debugStr not found"
    end

    if index ~= nil then
        return pcall(
            function()
                return tostring(module.debugStr(index))
            end
        )
    end

    return pcall(
        function()
            return module.debugStr()
        end
    )
end

--============================================================
-- Basic UI blocks
--============================================================

local function drawCarSummary(car)
    ui.text("=== ACNextGen Observer v1.1.4 ===")
    ui.separator()

    textKV(
        "RPM",
        num(
            safeField(
                car,
                "rpm",
                0.0
            ),
            0.0
        ),
        "%.0f"
    )

    textKV(
        "Gear",
        num(
            safeField(
                car,
                "gear",
                0.0
            ),
            0.0
        ),
        "%d"
    )

    textKV(
        "Speed",
        num(
            safeField(
                car,
                "speedKmh",
                0.0
            ),
            0.0
        ),
        "%.1f km/h"
    )

    local av =
        safeField(
            car,
            "localAngularVelocity",
            nil
        )

    if av then
        textKV(
            "YawRate",
            num(
                av.y,
                0.0
            ),
            "%.5f"
        )
    end

    textKV(
        "PhysicsAlive",
        loadNum(
            "ngp_physics_alive",
            0.0
        ),
        "%.3f"
    )

    textKV(
        "HubAlive",
        loadNum(
            "ngp_hub_alive",
            0.0
        ),
        "%.0f"
    )

    textKV(
        "HubPreserve",
        loadNum(
            "ngp_hub_preserve_avg",
            0.0
        ),
        "%.3f"
    )
end

local function drawInputs(car)
    ui.separator()
    ui.text("=== INPUTS ===")

    textKV("Steer", num(safeField(car, "steer", 0.0), 0.0), "%.3f")
    textKV("Gas", num(safeField(car, "gas", 0.0), 0.0), "%.3f")
    textKV("Brake", num(safeField(car, "brake", 0.0), 0.0), "%.3f")
    textKV("Clutch", num(safeField(car, "clutch", 0.0), 0.0), "%.3f")
    textKV("Handbrake", num(safeField(car, "handbrake", 0.0), 0.0), "%.3f")
end
    

--============================================================
-- Important integrated stores
--============================================================

local function drawTireContactSummary()
    ui.separator()
    ui.text("=== TIRE CONTACT SUMMARY ===")

    textKV(
        "TC Status",
        tostring(
            loadRaw(
                "ngp_tc_status",
                "NONE"
            )
        )
    )

    textKV("Avg Contact", loadNum("ngp_tc_avg_contact", 0.0), "%.3f")
    textKV("Avg Grip",    loadNum("ngp_tc_avg_grip", 0.0), "%.3f")
    textKV("Avg Energy",  loadNum("ngp_tc_avg_energy", 0.0), "%.3f")

    ui.text("--- Wheel Contact ---")

    for i = 0, 3 do
        ui.text(
            string.format(
                "%s C %.2f / G %.2f / E %.2f / L %.0f",
                WHEEL_NAMES[i] or tostring(i),
                loadNum("ngp_tc_contact_" .. i, 0.0),
                loadNum("ngp_tc_grip_" .. i, 0.0),
                loadNum("ngp_tc_energy_" .. i, 0.0),
                loadNum("ngp_tc_load_" .. i, 0.0)
            )
        )
    end
end

local function drawTireResponseSummary()
    ui.separator()
    ui.text("=== TIRE CONTACT RESPONSE ===")

    textKV(
        "TCR Status",
        tostring(
            loadRaw(
                "ngp_tcr_status",
                "NONE"
            )
        )
    )

    textKV("Avg Quality", loadNum("ngp_tcr_avg_quality", 0.0), "%.3f")
    textKV("Avg Eff Grip", loadNum("ngp_tcr_avg_grip", 0.0), "%.3f")
    textKV("Avg Loss", loadNum("ngp_tcr_avg_loss", 0.0), "%.3f")
    textKV("Avg Energy", loadNum("ngp_tcr_avg_energy", 0.0), "%.3f")
end

local function drawContactQualitySummary()
    ui.separator()
    ui.text("=== CONTACT QUALITY ===")

    textKV("CQ Status", tostring(loadRaw("ngp_contact_status_global", loadRaw("ngp_contact_quality_status", "NONE"))))
    textKV("CQ Count", loadNumAlt(0.0, "ngp_contact_update_count", "ngp_contact_quality_update_count"), "%.0f")
    textKV("Avg Quality", loadNumAlt(0.0, "ngp_contact_avg_quality", "ngp_contact_quality_avg"), "%.3f")
    textKV("Avg Trust", loadNum("ngp_contact_avg_trust", 0.0), "%.3f")
    textKV("Avg Raw", loadNumAlt(0.0, "ngp_contact_avg_raw", "ngp_contact_quality_avg_raw"), "%.3f")
    textKV("Carcass Link", loadNum("ngp_contact_carcass_linked", 0.0), "%.0f")

    ui.text("--- Wheel Contact Quality / Carcass Gate ---")

    for i = 0, 3 do
        ui.text(
            string.format(
                "%s %s / Q %.2f T %.2f / L %.0f / CS %.2f / Sup %.2f Gate %.2f H %.2f",
                WHEEL_NAMES[i] or tostring(i),
                tostring(loadRaw("ngp_contact_status_text_" .. i, tostring(loadNum("ngp_contact_status_" .. i, 0.0)))),
                loadNum("ngp_contact_quality_" .. i, 0.0),
                loadNum("ngp_contact_trust_" .. i, 0.0),
                loadNum("ngp_contact_load_" .. i, 0.0),
                loadNum("ngp_contact_combined_slip_" .. i, 0.0),
                loadNum("ngp_contact_carcass_support_" .. i, 1.0),
                loadNum("ngp_contact_grip_gate_" .. i, 1.0),
                loadNum("ngp_contact_hysteresis_" .. i, 0.0)
            )
        )
    end
end


local function drawTireCarcassSummary()
    ui.separator()
    ui.text("=== TIRE CARCASS / SIDEWALL ===")

    textKV("Carcass Status", tostring(loadRaw("ngp_tire_carcass_status", "NONE")))
    textKV("Carcass Count", loadNum("ngp_tire_carcass_update_count", 0.0), "%.0f")
    textKV("Avg Lat", loadNum("ngp_tire_carcass_avg_lat", 0.0), "%.3f")
    textKV("Avg Long", loadNum("ngp_tire_carcass_avg_long", 0.0), "%.3f")
    textKV("Avg Energy", loadNum("ngp_tire_carcass_avg_energy", 0.0), "%.3f")
    textKV("Avg Delay", loadNum("ngp_tire_carcass_avg_delay", 0.0), "%.3f")
    textKV("Avg Support", loadNum("ngp_tire_carcass_avg_support", 1.0), "%.3f")
    textKV("Avg Hyst", loadNum("ngp_tire_carcass_avg_hysteresis", 0.0), "%.3f")

    ui.text("--- Wheel Carcass State ---")

    for i = 0, 3 do
        ui.text(
            string.format(
                "%s %s / Def %.2f V %.2f / BLat %.2f BLong %.2f / Sup %.2f Gate %.2f / Heat %.2f Hist %.2f",
                WHEEL_NAMES[i] or tostring(i),
                tostring(loadRaw("ngp_tire_carcass_phase_" .. i, "NIL")),
                loadNum("ngp_tire_deformation_" .. i, 0.0),
                loadNum("ngp_carcass_vertical_norm_" .. i, 0.0),
                loadNum("ngp_carcass_bristle_lat_" .. i, 0.0),
                loadNum("ngp_carcass_bristle_long_" .. i, 0.0),
                loadNum("ngp_carcass_support_" .. i, 1.0),
                loadNum("ngp_carcass_grip_gate_" .. i, 1.0),
                loadNum("ngp_carcass_heat_seed_" .. i, 0.0),
                loadNum("ngp_carcass_history_stress_" .. i, 0.0)
            )
        )
    end
end


local function drawTireMemorySummary()
    ui.separator()
    ui.text("=== TIRE MEMORY / RUBBER HISTORY ===")

    textKV("TM Status", tostring(loadRaw("ngp_tire_memory_status", loadRaw("ngp_tyre_memory_status", "NONE"))))
    textKV("TM Count", loadNum("ngp_tire_memory_update_count", 0.0), "%.0f")
    textKV("Avg Memory", loadNum("ngp_tire_memory_avg", 0.0), "%.3f")
    textKV("Avg Grip", loadNum("ngp_tire_memory_avg_grip", 1.0), "%.3f")
    textKV("Avg Thermal", loadNum("ngp_tire_memory_avg_thermal", 0.0), "%.3f")
    textKV("Avg Abrasion", loadNum("ngp_tire_memory_avg_abrasion", 0.0), "%.3f")
    textKV("Avg Heat", loadNum("ngp_tire_memory_avg_heat", 0.0), "%.3f")

    ui.text("--- Wheel Tire Memory ---")

    for i = 0, 3 do
        ui.text(
            string.format(
                "%s %s / Mem %.2f Grip %.2f / Th %.2f Abr %.2f Hist %.2f / Rec %.2f Shock %.2f",
                WHEEL_NAMES[i] or tostring(i),
                tostring(loadRaw("ngp_tire_memory_phase_" .. i, "NIL")),
                loadNum("ngp_tire_memory_" .. i, 0.0),
                loadNum("ngp_memory_grip_" .. i, 1.0),
                loadNum("ngp_memory_thermal_" .. i, 0.0),
                loadNum("ngp_memory_abrasion_" .. i, 0.0),
                loadNum("ngp_memory_history_" .. i, 0.0),
                loadNum("ngp_memory_recovery_scalar_" .. i, 0.0),
                loadNum("ngp_memory_recontact_shock_" .. i, 0.0)
            )
        )
    end
end

local function drawTireDynamicsSummary()
    ui.separator()
    ui.text("=== TIRE DYNAMICS COUPLING ===")

    for i = 0, 3 do
        ui.text(
            string.format(
                "%s GMul %.2f / LMul %.2f / Delay %.2f / Loss %.2f",
                WHEEL_NAMES[i] or tostring(i),
                loadNum("ngp_tdyn_contact_grip_mul_" .. i, 1.0),
                loadNum("ngp_tdyn_contact_limit_mul_" .. i, 1.0),
                loadNum("ngp_tdyn_compliance_delay_" .. i, 0.0),
                loadNum("ngp_tdyn_contact_loss_" .. i, 0.0)
            )
        )
    end
end

local function drawSuspensionBridgeSummary()
    ui.separator()
    ui.text("=== SUSPENSION CONTACT INPUT ===")

    textKV(
        "SCI Status",
        tostring(
            loadRaw(
                "ngp_sci_status",
                "NONE"
            )
        )
    )

    textKV("Front Input", loadNum("ngp_sci_front_input", 0.0), "%.3f")
    textKV("Rear Input", loadNum("ngp_sci_rear_input", 0.0), "%.3f")
    textKV("Left Input", loadNum("ngp_sci_left_input", 0.0), "%.3f")
    textKV("Right Input", loadNum("ngp_sci_right_input", 0.0), "%.3f")
    textKV("Avg Road Input", loadNum("ngp_sci_avg_road_input", 0.0), "%.3f")
    textKV("Avg Susp Scale", loadNum("ngp_sci_avg_susp_scale", 1.0), "%.3f")
end

local function drawSuspensionIntegratedSummary()
    ui.separator()
    ui.text("=== SUSPENSION INTEGRATED SUMMARY ===")

    textKV(
        "Susp Status",
        tostring(
            loadRaw(
                "ngp_suspension_status",
                "NONE"
            )
        )
    )

    textKV("Initialized", loadNum("ngp_suspension_initialized", 0.0), "%.0f")
    textKV("Wheels Valid", loadNum("ngp_suspension_wheels_valid", 0.0), "%.0f")

    ui.text("--- Integrated Wheel Force ---")

    for i = 0, 3 do
        ui.text(
            string.format(
                "%s F %+.0f / Scale %.2f / Contact %.2f / Damp %+.0f / Spring %+.0f / Preserve %.3f",
                WHEEL_NAMES[i] or tostring(i),
                loadNum("ngp_susp_" .. i, 0.0),
                loadNumAlt(
                    1.0,
                    "ngp_susp_int_scale_" .. i,
                    "ngp_susp_integrated_scale_" .. i
                ),
                loadNumAlt(
                    0.0,
                    "ngp_susp_int_contact_" .. i,
                    "ngp_susp_contact_integrated_" .. i
                ),
                loadNumAlt(
                    0.0,
                    "ngp_susp_int_damper_" .. i,
                    "ngp_susp_external_damper_" .. i
                ),
                loadNumAlt(
                    0.0,
                    "ngp_susp_int_spring_" .. i,
                    "ngp_susp_external_spring_" .. i
                ),
                loadNum("ngp_susp_preserve_force_add_" .. i, 0.0)
            )
        )
    end
end

local function drawDamperSpringSummary()
    ui.separator()
    ui.text("=== DAMPER / SPRING SUMMARY ===")

    textKV("Damper Status", tostring(loadRaw("ngp_damper_status", "NONE")))
    textKV("Damper Avg Abs", loadNum("ngp_damper_avg_abs_force", 0.0), "%.0f")
    textKV("Damper Avg Scale", loadNum("ngp_damper_avg_coeff_scale", 1.0), "%.3f")

    textKV("Spring Status", tostring(loadRaw("ngp_spring_status", "NONE")))
    textKV("Spring Avg Abs", loadNum("ngp_spring_avg_abs_force", 0.0), "%.0f")
    textKV("Spring Avg Scale", loadNum("ngp_spring_avg_rate_scale", 1.0), "%.3f")
end



local function drawDamperHysteresisSummary()
    ui.separator()
    ui.text("=== DAMPER HYSTERESIS / VERTICAL ROAD FEED ===")

    textKV("DH Status", tostring(loadRaw("ngp_damper_hyst_status", "NONE")))
    textKV("DH Count", loadNum("ngp_damper_hyst_update_count", 0.0), "%.0f")
    textKV("Avg Hyst", loadNum("ngp_damper_hyst_avg", 0.0), "%.3f")
    textKV("Avg Mem", loadNum("ngp_damper_hyst_avg_memory", 0.0), "%.3f")
    textKV("Avg Impact", loadNum("ngp_damper_hyst_avg_impact", 0.0), "%.3f")
    textKV("Avg Heat", loadNum("ngp_damper_hyst_avg_heat", 0.0), "%.3f")
    textKV("Avg RoadMem", loadNum("ngp_damper_hyst_avg_road_memory", 0.0), "%.3f")

    ui.text("--- Damper Wheels ---")

    for i = 0, 3 do
        ui.text(
            string.format(
                "%s %s / H %.2f V %+.3f / Low %.2f High %.2f / Vert %.2f Pulse %.2f / Heat %.2f",
                WHEEL_NAMES[i] or tostring(i),
                tostring(loadRaw("ngp_damper_hyst_phase_" .. i, "NIL")),
                loadNum("ngp_damper_hyst_" .. i, 0.0),
                loadNum("ngp_damper_hyst_velocity_" .. i, 0.0),
                loadNum("ngp_damper_low_speed_" .. i, 0.0),
                loadNum("ngp_damper_high_speed_" .. i, 0.0),
                loadNum("ngp_damper_vertical_" .. i, 0.0),
                loadNum("ngp_damper_vertical_pulse_" .. i, 0.0),
                loadNum("ngp_damper_heat_" .. i, 0.0)
            )
        )
    end
end


local function drawComplianceStackSummary()
    ui.separator()
    ui.text("=== COMPLIANCE STACK ===")

    textKV("CS Status", tostring(loadRaw("ngp_compliance_status", "NONE")))
    textKV("CS Count", loadNum("ngp_compliance_update_count", 0.0), "%.0f")
    textKV("Avg Energy", loadNum("ngp_compliance_avg_energy", 0.0), "%.3f")
    textKV("Avg Bushing", loadNum("ngp_compliance_avg_bushing", 0.0), "%.3f")
    textKV("Avg Knuckle", loadNum("ngp_compliance_avg_knuckle", 0.0), "%.3f")
    textKV("Avg Body", loadNum("ngp_compliance_avg_body", 0.0), "%.3f")
    textKV("Force Leak", loadNum("ngp_compliance_force_leak", 0.0), "%.3f")
    textKV("LoadPath Loss", loadNum("ngp_compliance_load_path_loss", 0.0), "%.3f")
    textKV("Rear Axle Windup", loadNum("ngp_compliance_rear_axle_windup", 0.0), "%.3f")

    ui.text("--- Compliance Wheels ---")

    for i = 0, 3 do
        ui.text(
            string.format(
                "%s %s / E %.2f / Toe %+.4f Cam %+.4f / Bush %.2f Kn %.2f Sub %.2f Leak %.2f",
                WHEEL_NAMES[i] or tostring(i),
                tostring(loadRaw("ngp_compliance_phase_" .. i, "NIL")),
                loadNum("ngp_compliance_energy_" .. i, 0.0),
                loadNum("ngp_compliance_virtual_toe_" .. i, 0.0),
                loadNum("ngp_compliance_virtual_camber_" .. i, 0.0),
                loadNum("ngp_compliance_bushing_energy_" .. i, 0.0),
                loadNum("ngp_compliance_knuckle_deflection_" .. i, 0.0),
                loadNum("ngp_compliance_subframe_deflection_" .. i, 0.0),
                loadNum("ngp_compliance_force_leak_" .. i, 0.0)
            )
        )
    end
end


local function drawYawMomentBudgetSummary()
    ui.separator()
    ui.text("=== YAW MOMENT BUDGET ===")

    textKV("YMB Status", tostring(loadRaw("ngp_yaw_budget_status", "NONE")))
    textKV("YMB Count", loadNum("ngp_yaw_budget_update_count", 0.0), "%.0f")
    textKV("Phase", tostring(loadRaw("ngp_yaw_budget_phase", "NIL")))
    textKV("Budget", loadNum("ngp_yaw_budget", 0.0), "%+.3f")
    textKV("Balance", loadNum("ngp_yaw_balance", 0.0), "%+.3f")
    textKV("Rotation", loadNum("ngp_yaw_rotation_intent", 0.0), "%.3f")
    textKV("Understeer", loadNum("ngp_yaw_understeer_energy", 0.0), "%.3f")
    textKV("Oversteer", loadNum("ngp_yaw_oversteer_energy", 0.0), "%.3f")
    textKV("Recovery", loadNum("ngp_yaw_recovery_phase", 0.0), "%.3f")
    textKV("SpinRisk", loadNum("ngp_yaw_spin_risk", 0.0), "%.3f")

    ui.text("--- Recovery / Bite / Dirty Return ---")
    ui.text(
        string.format(
            "YawRec %.2f / Bite %.2f / Dirty %.2f / ReContact %.2f / GripRet %.2f / RearSnap %.2f",
            loadNum("ngp_yaw_recovery", 0.0),
            loadNum("ngp_yaw_bite", 0.0),
            loadNum("ngp_yaw_dirty_return", 0.0),
            loadNum("ngp_yaw_recontact", 0.0),
            loadNum("ngp_yaw_slip_grip_return", 1.0),
            loadNum("ngp_yaw_slip_snap_rear", 0.0)
        )
    )

    ui.text("--- Authority / Slip ---")
    ui.text(
        string.format(
            "F Auth %.2f / R Auth %.2f / F Slip %.2f / R Slip %.2f / F CQ %.2f / R CQ %.2f",
            loadNum("ngp_yaw_front_authority", 0.0),
            loadNum("ngp_yaw_rear_authority", 0.0),
            loadNum("ngp_yaw_front_slip", 0.0),
            loadNum("ngp_yaw_rear_slip", 0.0),
            loadNum("ngp_yaw_front_contact", 0.0),
            loadNum("ngp_yaw_rear_contact", 0.0)
        )
    )

    ui.text("--- Moment Sources ---")
    ui.text(
        string.format(
            "Yaw %.3f / Demand %+.3f / Resp %+.3f / Drive %.2f / LSD %.2f / Comp %+.2f / Load %+.2f",
            loadNum("ngp_yaw_rate", 0.0),
            loadNum("ngp_yaw_steer_demand", 0.0),
            loadNum("ngp_yaw_response", 0.0),
            loadNum("ngp_yaw_drive", 0.0),
            loadNum("ngp_yaw_lsd", 0.0),
            loadNum("ngp_yaw_compliance", 0.0),
            loadNum("ngp_yaw_load", 0.0)
        )
    )

    ui.text("--- Wheel Authority ---")
    for i = 0, 3 do
        ui.text(
            string.format(
                "%s Auth %.2f / CQ %.2f / Slip %.2f / Mem %.2f / Rec %.2f / Bite %.2f / Dirty %.2f",
                WHEEL_NAMES[i] or tostring(i),
                loadNum("ngp_yaw_wheel_authority_" .. i, 0.0),
                loadNum("ngp_yaw_wheel_contact_" .. i, 0.0),
                loadNum("ngp_yaw_wheel_slip_" .. i, 0.0),
                loadNum("ngp_yaw_wheel_memory_" .. i, 0.0),
                loadNum("ngp_yaw_wheel_recovery_" .. i, 0.0),
                loadNum("ngp_yaw_wheel_bite_" .. i, 0.0),
                loadNum("ngp_yaw_wheel_dirty_return_" .. i, 0.0)
            )
        )
    end
end


local function drawBrakeSystemSummary()
    ui.separator()
    ui.text("=== BRAKE SYSTEM SUMMARY ===")

    textKV("BrakeSys Status", tostring(loadRaw("ngp_brake_status", loadRaw("ngp_brake_system_status", "NONE"))))
    textKV("Brake Input", loadNumAlt(0.0, "ngp_brake_input_smoothed", "ngp_brake_input"), "%.3f")
    textKV("Root Heat Avg", loadNum("ngp_brake_root_heat_avg", 0.0), "%.3f")
    textKV("Fade Root Avg", loadNum("ngp_brake_fade_root_add_avg", 0.0), "%.3f")
    textKV("Lock Root Avg", loadNum("ngp_brake_lock_root_add_avg", 0.0), "%.3f")

    ui.text("--- Brake Wheels ---")

    for i = 0, 3 do
        ui.text(
            string.format(
                "%s Temp %.0f / Lock %.2f / FadeRoot %.3f / Heat %.3f",
                WHEEL_NAMES[i] or tostring(i),
                loadNumAlt(25.0, "ngp_brake_temp_" .. i, "ngp_brake_disc_temp_" .. i),
                loadNum("ngp_brake_lock_" .. i, 0.0),
                loadNum("ngp_brake_fade_root_add_" .. i, 0.0),
                loadNum("ngp_brake_root_heat_" .. i, 0.0)
            )
        )
    end
end

local function drawDiffSummary()
    ui.separator()
    ui.text("=== DIFF / LSD SUMMARY ===")

    textKV(
        "LSD Status",
        tostring(
            loadRaw(
                "ngp_lsd_status",
                loadRaw(
                    "ngp_diff_status",
                    "NONE"
                )
            )
        )
    )

    textKV(
        "LSD Lock",
        loadNumAlt(
            0.0,
            "ngp_lsd_lock",
            "ngp_diff_lock"
        )
        *
        100.0,
        "%.1f %%"
    )

    textKV(
        "LSD Diff",
        loadNumAlt(
            0.0,
            "ngp_lsd_diff",
            "ngp_diff_diff"
        ),
        "%.3f"
    )

    textKV(
        "LSD Heat",
        loadNumAlt(
            0.0,
            "ngp_lsd_heat",
            "ngp_diff_heat"
        ),
        "%.1f"
    )

    textKV(
        "DriveTorque",
        loadNumAlt(
            0.0,
            "ngp_drive_torque",
            "ngp_drive_transmitted_torque",
            "ngp_drivetrain_torque"
        ),
        "%.3f"
    )
end


local function drawDrivelineWindupSummary()
    ui.separator()
    ui.text("=== DRIVELINE WINDUP ===")

    textKV("DW Status", tostring(loadRaw("ngp_windup_status", "NONE")))
    textKV("DW Count", loadNum("ngp_windup_update_count", 0.0), "%.0f")
    textKV("Phase", tostring(loadRaw("ngp_windup_phase", "NIL")))
    textKV("Demand", loadNum("ngp_windup_demand", 0.0), "%+.3f")
    textKV("Windup", loadNum("ngp_windup_value", 0.0), "%+.3f")
    textKV("Energy", loadNum("ngp_windup_energy", 0.0), "%.3f")
    textKV("ShaftTwist", loadNum("ngp_windup_shaft_twist", 0.0), "%+.3f")
    textKV("Soft Torque", loadNum("ngp_drive_soft_torque", 0.0), "%+.3f")
    textKV("Rear Push", loadNum("ngp_drive_soft_rear_push", 0.0), "%.3f")
    textKV("Yaw Hint", loadNum("ngp_drive_soft_yaw", 0.0), "%+.3f")
    textKV("Recovery Drag", loadNum("ngp_windup_recovery_drag", 0.0), "%.3f")
    textKV("Bite", loadNum("ngp_windup_bite", 0.0), "%.3f")
    textKV("Dirty Return", loadNum("ngp_windup_dirty_return", 0.0), "%.3f")

    ui.text("--- Rear Driveline ---")
    ui.text(
        string.format(
            "OmegaAvg %.2f / OmegaDiff %+.2f / Slip %.3f / CQ %.2f / Trust %.2f / LSD %.2f / Snap %.2f",
            loadNum("ngp_windup_rear_omega_avg", 0.0),
            loadNum("ngp_windup_rear_omega_diff", 0.0),
            loadNum("ngp_windup_rear_slip", 0.0),
            loadNum("ngp_windup_rear_contact", 0.0),
            loadNum("ngp_windup_rear_trust", 0.0),
            loadNum("ngp_windup_lsd_lock", 0.0),
            loadNum("ngp_windup_rear_snap", 0.0)
        )
    )

    ui.text("--- Links ---")
    ui.text(
        string.format(
            "DT %.0f / LSD %.0f / CQ %.0f / YMB %.0f / SR %.0f / MEM %.0f / TC %.0f / Source %s",
            loadNum("ngp_windup_link_drivetrain", 0.0),
            loadNum("ngp_windup_link_lsd", 0.0),
            loadNum("ngp_windup_link_contact", 0.0),
            loadNum("ngp_windup_link_yaw_budget", 0.0),
            loadNum("ngp_windup_link_slip_recovery", 0.0),
            loadNum("ngp_windup_link_memory", 0.0),
            loadNum("ngp_windup_link_carcass", 0.0),
            tostring(loadRaw("ngp_windup_torque_source", "NIL"))
        )
    )

    ui.text("--- Wheel Shares ---")
    for i = 0, 3 do
        ui.text(
            string.format(
                "%s Om %.1f / Slip %.3f / Tr %.2f / W %.2f / T %+.2f / R %.2f / Bite %.2f / Dirty %.2f",
                WHEEL_NAMES[i] or tostring(i),
                loadNum("ngp_windup_wheel_omega_" .. i, 0.0),
                loadNum("ngp_windup_wheel_slip_" .. i, 0.0),
                loadNum("ngp_windup_wheel_trust_" .. i, 0.0),
                loadNum("ngp_windup_wheel_energy_" .. i, 0.0),
                loadNum("ngp_windup_wheel_torque_" .. i, 0.0),
                loadNum("ngp_windup_wheel_release_" .. i, 0.0),
                loadNum("ngp_windup_wheel_bite_" .. i, 0.0),
                loadNum("ngp_windup_wheel_dirty_" .. i, 0.0)
            )
        )
    end
end


local function drawSlipRecoverySummary()
    ui.separator()
    ui.text("=== SLIP RECOVERY ===")

    textKV("SR Status", tostring(loadRaw("ngp_slip_recovery_status", "NONE")))
    textKV("SR Count", loadNum("ngp_slip_recovery_update_count", 0.0), "%.0f")
    textKV("Avg Recovery", loadNum("ngp_slip_recovery_avg", 0.0), "%.3f")
    textKV("Avg GripReturn", loadNum("ngp_slip_recovery_avg_grip_return", 1.0), "%.3f")
    textKV("Avg SnapRisk", loadNum("ngp_slip_recovery_avg_snap_risk", 0.0), "%.3f")
    textKV("Avg Bite", loadNum("ngp_slip_recovery_avg_bite", 0.0), "%.3f")
    textKV("Avg DirtyReturn", loadNum("ngp_slip_recovery_avg_dirty_return", 0.0), "%.3f")

    ui.text("--- Wheel Recovery ---")

    for i = 0, 3 do
        ui.text(
            string.format(
                "%s %s / Rec %.2f Grip %.2f Snap %.2f Bite %.2f Dirty %.2f ReC %.2f",
                WHEEL_NAMES[i] or tostring(i),
                tostring(loadRaw("ngp_slip_recovery_phase_" .. i, "NIL")),
                loadNum("ngp_slip_recovery_rate_" .. i, 0.0),
                loadNum("ngp_slip_grip_return_" .. i, 1.0),
                loadNum("ngp_slip_snap_risk_" .. i, 0.0),
                loadNum("ngp_slip_bite_" .. i, 0.0),
                loadNum("ngp_slip_dirty_return_" .. i, 0.0),
                loadNum("ngp_slip_recontact_" .. i, 0.0)
            )
        )
    end

    ui.text("--- Links ---")
    ui.text(
        string.format(
            "CQ %.0f / TM %.0f / TC %.0f / YMB %.0f / LP %.0f",
            loadNum("ngp_slip_recovery_contact_linked", 0.0),
            loadNum("ngp_slip_recovery_memory_linked", 0.0),
            loadNum("ngp_slip_recovery_carcass_linked", 0.0),
            loadNum("ngp_slip_recovery_yaw_linked", 0.0),
            loadNum("ngp_slip_recovery_load_path_linked", 0.0)
        )
    )
end


local function drawLoadPathSummary()
    ui.separator()
    ui.text("=== LOAD PATH / COMPONENT WORK ===")

    textKV("LP Status", tostring(loadRaw("ngp_load_path_status", "NONE")))
    textKV("LP Count", loadNum("ngp_load_path_update_count", 0.0), "%.0f")
    textKV("Dominant", tostring(loadRaw("ngp_load_path_dominant", "NIL")))
    textKV("Avg Work", loadNum("ngp_load_path_avg_work", 0.0), "%.3f")
    textKV("Avg Eff", loadNum("ngp_load_path_avg_efficiency", 1.0), "%.3f")
    textKV("Avg Integrity", loadNum("ngp_load_path_avg_integrity", 1.0), "%.3f")
    textKV("Avg Delivery", loadNum("ngp_load_path_avg_tire_delivery", 1.0), "%.3f")
    textKV("Avg ForceLeak", loadNum("ngp_load_path_avg_force_leak", 0.0), "%.3f")
    textKV("Avg RoadWork", loadNum("ngp_load_path_avg_road_work", 0.0), "%.3f")

    ui.text("--- Axle Delivery ---")
    ui.text(
        string.format(
            "F Work %.3f / R Work %.3f / F Int %.2f / R Int %.2f / F Del %.2f / R Del %.2f",
            loadNum("ngp_load_path_front_work", 0.0),
            loadNum("ngp_load_path_rear_work", 0.0),
            loadNum("ngp_load_path_front_integrity", 1.0),
            loadNum("ngp_load_path_rear_integrity", 1.0),
            loadNum("ngp_load_path_front_delivery", 1.0),
            loadNum("ngp_load_path_rear_delivery", 1.0)
        )
    )

    ui.text("--- Component Shares ---")
    ui.text(
        string.format(
            "Tire %.2f / Susp %.2f / Comp %.2f / Body %.2f / Contact %.2f / Road %.2f",
            loadNum("ngp_load_path_tire_share", 0.0),
            loadNum("ngp_load_path_suspension_share", 0.0),
            loadNum("ngp_load_path_compliance_share", 0.0),
            loadNum("ngp_load_path_body_share", 0.0),
            loadNum("ngp_load_path_contact_share", 0.0),
            loadNum("ngp_load_path_road_share", 0.0)
        )
    )

    ui.text("--- Wheel Load Path ---")

    for i = 0, 3 do
        ui.text(
            string.format(
                "%s %s / W %.2f Eff %.2f Int %.2f Del %.2f / Leak %.2f VF %.2f Road %.2f",
                WHEEL_NAMES[i] or tostring(i),
                tostring(loadRaw("ngp_load_path_phase_" .. i, "NIL")),
                loadNum("ngp_load_path_work_" .. i, 0.0),
                loadNum("ngp_load_path_efficiency_" .. i, 1.0),
                loadNum("ngp_load_path_integrity_" .. i, 1.0),
                loadNum("ngp_load_path_tire_delivery_" .. i, 1.0),
                loadNum("ngp_load_path_force_leak_" .. i, 0.0),
                loadNum("ngp_load_path_vertical_flow_" .. i, 0.0),
                loadNum("ngp_load_path_road_" .. i, 0.0)
            )
        )
    end

    ui.text("--- Links ---")
    ui.text(
        string.format(
            "L %.0f / CQ %.0f / TC %.0f / DH %.0f / CS %.0f / B %.0f / S %.0f / SR %.0f / TM %.0f",
            loadNum("ngp_load_path_load_linked", 0.0),
            loadNum("ngp_load_path_contact_linked", 0.0),
            loadNum("ngp_load_path_carcass_linked", 0.0),
            loadNum("ngp_load_path_damper_linked", 0.0),
            loadNum("ngp_load_path_compliance_linked", 0.0),
            loadNum("ngp_load_path_body_linked", 0.0),
            loadNum("ngp_load_path_suspension_linked", 0.0),
            loadNum("ngp_load_path_recovery_linked", 0.0),
            loadNum("ngp_load_path_memory_linked", 0.0)
        )
    )
end


local function drawRoadInputInterpreterSummary()
    ui.separator()
    ui.text("=== ROAD INPUT INTERPRETER / HOU-REN-SOU ===")

    textKV("RII Status", tostring(loadRaw("ngp_road_input_status", "NONE")))
    textKV("RII Count", loadNum("ngp_road_input_update_count", 0.0), "%.0f")
    textKV("Avg Noise", loadNum("ngp_road_input_avg_noise", 0.0), "%.3f")
    textKV("Avg Impact", loadNum("ngp_road_input_avg_impact", 0.0), "%.3f")
    textKV("Avg Texture", loadNum("ngp_rii_avg_texture", 0.0), "%.3f")
    textKV("Avg Shock", loadNum("ngp_rii_avg_shock", 0.0), "%.3f")
    textKV("Avg PathLoss", loadNum("ngp_rii_avg_path_loss", 0.0), "%.3f")
    textKV("Avg SurfaceLimit", loadNum("ngp_rii_avg_surface_limit", 0.0), "%.3f")
    textKV("Severity", loadNum("ngp_road_input_avg_severity", 0.0), "%.3f")

    ui.text("--- Body Input ---")
    ui.text(
        string.format(
            "Speed %.1f / Heave %.3f / Pitch %+.3f / Roll %+.3f / Yaw %+.3f",
            loadNum("ngp_road_input_speed_kmh", 0.0),
            loadNum("ngp_road_input_body_heave", 0.0),
            loadNum("ngp_road_input_body_pitch", 0.0),
            loadNum("ngp_road_input_body_roll", 0.0),
            loadNum("ngp_road_input_body_yaw", 0.0)
        )
    )

    ui.text("--- Wheel Road Input ---")
    for i = 0, 3 do
        ui.text(
            string.format(
                "%s %s / N %.2f I %.2f Tex %.2f Shock %.2f PL %.2f Surf %.2f Hint %.2f",
                WHEEL_NAMES[i] or tostring(i),
                tostring(loadRaw("ngp_road_input_phase_" .. i, "NIL")),
                loadNum("ngp_road_noise_" .. i, 0.0),
                loadNum("ngp_road_impact_" .. i, 0.0),
                loadNum("ngp_road_texture_" .. i, 0.0),
                loadNum("ngp_road_shock_" .. i, 0.0),
                loadNum("ngp_road_path_loss_" .. i, 0.0),
                loadNum("ngp_road_surface_limit_" .. i, 0.0),
                loadNum("ngp_road_module_hint_" .. i, 0.0)
            )
        )
    end

    ui.text("--- Links ---")
    ui.text(
        string.format(
            "RBI %.0f / CQ %.0f / DH %.0f / Hop %.0f / Load %.0f / Susp %.0f / LP %.0f / CS %.0f / TC %.0f",
            loadNum("ngp_road_input_body_linked", 0.0),
            loadNum("ngp_road_input_contact_linked", 0.0),
            loadNum("ngp_road_input_damper_linked", 0.0),
            loadNum("ngp_road_input_hop_linked", 0.0),
            loadNum("ngp_road_input_load_linked", 0.0),
            loadNum("ngp_road_input_suspension_linked", 0.0),
            loadNum("ngp_road_input_load_path_linked", 0.0),
            loadNum("ngp_road_input_compliance_linked", 0.0),
            loadNum("ngp_road_input_carcass_linked", 0.0)
        )
    )
end


local function drawLoadBodySummary()
    ui.separator()
    ui.text("=== LOAD / BODY SUMMARY ===")

    textKV(
        "Load FL",
        loadNumAlt(
            0.0,
            "ngp_dlt_load_0",
            "ngp_sprung_load_0"
        ),
        "%.0f"
    )

    textKV(
        "Load FR",
        loadNumAlt(
            0.0,
            "ngp_dlt_load_1",
            "ngp_sprung_load_1"
        ),
        "%.0f"
    )

    textKV(
        "Load RL",
        loadNumAlt(
            0.0,
            "ngp_dlt_load_2",
            "ngp_sprung_load_2"
        ),
        "%.0f"
    )

    textKV(
        "Load RR",
        loadNumAlt(
            0.0,
            "ngp_dlt_load_3",
            "ngp_sprung_load_3"
        ),
        "%.0f"
    )

    textKV("PitchCG", loadNum("ngp_pitch_cg", 0.0), "%.3f")
    textKV("RollCG", loadNum("ngp_roll_cg", 0.0), "%.3f")
    textKV("Instability", loadNum("ngp_instability", 0.0), "%.3f")
    textKV("Condition", loadNum("ngp_condition_total", 0.0), "%.3f")

        ui.separator()
    ui.text("=== ROAD BODY INPUT ===")

    textKV("RBI Status", tostring(loadRaw("ngp_rbi_status", "NONE")))
    textKV("Heave", loadNum("ngp_body_heave_input", 0.0), "%.3f")
    textKV("Pitch", loadNum("ngp_body_pitch_input", 0.0), "%+.3f")
    textKV("Roll", loadNum("ngp_body_roll_input", 0.0), "%+.3f")
    textKV("YawHint", loadNum("ngp_body_yaw_hint", 0.0), "%+.3f")
    
        ui.separator()
    ui.text("=== CHASSIS ENERGY ===")

    textKV("CE Status", tostring(loadRaw("ngp_chassis_energy_status", "NONE")))
    textKV("Energy", loadNum("ngp_chassis_energy", 0.0), "%.3f")
    textKV("Target", loadNum("ngp_chassis_target_energy", 0.0), "%.3f")
    textKV("Release", loadNum("ngp_chassis_release", 0.0), "%.3f")
    textKV("YawE", loadNum("ngp_chassis_yaw_energy", 0.0), "%.3f")
    textKV("RollE", loadNum("ngp_chassis_roll_energy", 0.0), "%.3f")
    textKV("PitchE", loadNum("ngp_chassis_pitch_energy", 0.0), "%.3f")
    textKV("VirtualE", loadNum("ngp_chassis_virtual_energy", 0.0), "%.3f")
    textKV("BodyInputE", loadNum("ngp_chassis_body_input_energy", 0.0), "%.3f")

    ui.separator()
    ui.text("=== CHASSIS FLEX ===")

    textKV("Flex Status", tostring(loadRaw("ngp_chassis_flex_status", "NONE")))
    textKV("BodySteer", loadNum("ngp_body_steer", 0.0), "%+.3f")
    textKV("Delay", loadNum("ngp_chassis_flex_delay", 0.0), "%+.3f")
    textKV("FlexYaw", loadNum("ngp_chassis_flex_yaw", 0.0), "%+.3f")
    textKV("FlexRoll", loadNum("ngp_chassis_flex_roll", 0.0), "%+.3f")
    textKV("FlexPitch", loadNum("ngp_chassis_flex_pitch", 0.0), "%+.3f")
    textKV("FlexEnergy", loadNum("ngp_chassis_flex_energy", 0.0), "%.3f")

end

local function drawProfilerSummary()
    ui.separator()
    ui.text("=== ACNEXTGEN PROFILER ===")

    textKV(
        "Worst Now",
        tostring(
            loadRaw(
                "ngp_prof_worst_name",
                "none"
            )
        )
    )

    textKV(
        "Worst Now ms",
        loadNum(
            "ngp_prof_worst_ms",
            0.0
        ),
        "%.3f ms"
    )

    textKV(
        "Worst Ever",
        tostring(
            loadRaw(
                "ngp_prof_worst_ever_name",
                "none"
            )
        )
    )

    textKV(
        "Worst Ever ms",
        loadNum(
            "ngp_prof_worst_ever_ms",
            0.0
        ),
        "%.3f ms"
    )

    ui.text("--- Important Modules ---")

    local names = {
        "damper_hysteresis",
        "compliance_stack",
        "tire_carcass",
        "contact_quality",
        "tire_memory",
        "load_path",
        "road_input_interpreter",
        "slip_recovery",
        "driveline_windup",
        "yaw_moment_budget",
        "damper_model",
        "suspension",
        "diff_lsd",
        "drivetrain",
        "tire_hop",
        "load_transfer",
        "road_body_input",
        "chassis_roll",
        "virtual_inertia",
    }
    for i = 1, #names do
        local name =
            names[i]

        ui.text(
            string.format(
                "%-22s Last %.3f / Avg %.3f / Max %.3f ms",
                name,
                loadNum(
                    "ngp_prof_last_" .. name,
                    0.0
                ),
                loadNum(
                    "ngp_prof_avg_" .. name,
                    0.0
                ),
                loadNum(
                    "ngp_prof_max_" .. name,
                    0.0
                )
            )
        )
    end
end

local function drawRootHealth()
    ui.separator()
    ui.text("=== ACNextGen V1.1 ROOT HEALTH ===")

    local dhStatus = loadRaw("ngp_damper_hyst_status", "NIL")
    local dhCount  = loadNum("ngp_damper_hyst_update_count", 0.0)
    local dhAvg    = loadNum("ngp_damper_hyst_avg", 0.0)

    local csStatus = loadRaw("ngp_compliance_status", "NIL")
    local csCount  = loadNum("ngp_compliance_update_count", 0.0)
    local csAvg    = loadNum("ngp_compliance_avg_energy", 0.0)
    local csLeak   = loadNum("ngp_compliance_force_leak", 0.0)

    local tcStatus = loadRaw("ngp_tire_carcass_status", "NIL")
    local tcCount  = loadNum("ngp_tire_carcass_update_count", 0.0)
    local tcSupport = loadNum("ngp_tire_carcass_avg_support", 1.0)

    local cqStatus = loadRaw("ngp_contact_status_global", loadRaw("ngp_contact_quality_status", "NIL"))
    local cqCount  = loadNumAlt(0.0, "ngp_contact_update_count", "ngp_contact_quality_update_count")
    local cqAvg    = loadNumAlt(0.0, "ngp_contact_avg_quality", "ngp_contact_quality_avg")
    local cqTrust  = loadNum("ngp_contact_avg_trust", 0.0)

    local tmStatus = loadRaw("ngp_tire_memory_status", loadRaw("ngp_tyre_memory_status", "NIL"))
    local tmCount  = loadNum("ngp_tire_memory_update_count", 0.0)
    local tmAvg    = loadNum("ngp_tire_memory_avg", 0.0)
    local tmGrip   = loadNum("ngp_tire_memory_avg_grip", 1.0)

    local lpStatus = loadRaw("ngp_load_path_status", "NIL")
    local lpCount  = loadNum("ngp_load_path_update_count", 0.0)
    local lpInt    = loadNum("ngp_load_path_avg_integrity", 1.0)
    local lpDel    = loadNum("ngp_load_path_avg_tire_delivery", 1.0)

    local riiStatus = loadRaw("ngp_road_input_status", "NIL")
    local riiCount  = loadNum("ngp_road_input_update_count", 0.0)
    local riiHint   = loadNum("ngp_rii_avg_severity", 0.0)
    local riiPath   = loadNum("ngp_rii_avg_path_loss", 0.0)

    local srStatus = loadRaw("ngp_slip_recovery_status", "NIL")
    local srCount  = loadNum("ngp_slip_recovery_update_count", 0.0)
    local srAvg    = loadNum("ngp_slip_recovery_avg", 0.0)
    local srBite   = loadNum("ngp_slip_recovery_avg_bite", 0.0)

    local dwStatus = loadRaw("ngp_windup_status", "NIL")
    local dwCount  = loadNum("ngp_windup_update_count", 0.0)
    local dwEnergy = loadNum("ngp_windup_energy", 0.0)
    local dwTwist  = loadNum("ngp_windup_shaft_twist", 0.0)

    local ymbStatus = loadRaw("ngp_yaw_budget_status", "NIL")
    local ymbCount  = loadNum("ngp_yaw_budget_update_count", 0.0)
    local ymbPhase  = loadRaw("ngp_yaw_budget_phase", "NIL")
    local ymbBudget = loadNum("ngp_yaw_budget", 0.0)
    local ymbSpin   = loadNum("ngp_yaw_spin_risk", 0.0)

    ui.text(string.format("1 DamperHyst : %s / Count %.0f / H %.3f", tostring(dhStatus), dhCount, dhAvg))
    ui.text(string.format("2 Compliance : %s / Count %.0f / E %.3f / Leak %.3f", tostring(csStatus), csCount, csAvg, csLeak))
    ui.text(string.format("3 Carcass    : %s / Count %.0f / Support %.3f", tostring(tcStatus), tcCount, tcSupport))
    ui.text(string.format("4 ContactQ   : %s / Count %.0f / Q %.3f / Trust %.3f", tostring(cqStatus), cqCount, cqAvg, cqTrust))
    ui.text(string.format("5 TireMemory : %s / Count %.0f / Mem %.3f / Grip %.3f", tostring(tmStatus), tmCount, tmAvg, tmGrip))
    ui.text(string.format("6 LoadPath   : %s / Count %.0f / Int %.3f / Delivery %.3f", tostring(lpStatus), lpCount, lpInt, lpDel))
    ui.text(string.format("7 RoadInput  : %s / Count %.0f / Sev %.3f / PathLoss %.3f", tostring(riiStatus), riiCount, riiHint, riiPath))
    ui.text(string.format("8 SlipRec    : %s / Count %.0f / Rec %.3f / Bite %.3f", tostring(srStatus), srCount, srAvg, srBite))
    ui.text(string.format("9 Driveline  : %s / Count %.0f / E %.3f / Twist %+.3f", tostring(dwStatus), dwCount, dwEnergy, dwTwist))
    ui.text(string.format("10 YawBudget : %s / Count %.0f / %s / B %+.3f / Spin %.3f", tostring(ymbStatus), ymbCount, tostring(ymbPhase), ymbBudget, ymbSpin))

    local dhOK = dhStatus == "RUNNING" and dhCount > 0
    local csOK = csStatus == "RUNNING" and csCount > 0
    local tcOK = tcStatus == "RUNNING" and tcCount > 0
    local cqOK = cqStatus == "RUNNING" and cqCount > 0
    local tmOK = tmStatus == "RUNNING" and tmCount > 0
    local lpOK = lpStatus == "RUNNING" and lpCount > 0
    local riiOK = riiStatus == "RUNNING" and riiCount > 0
    local srOK = srStatus == "RUNNING" and srCount > 0
    local dwOK = dwStatus == "RUNNING" and dwCount > 0
    local ymbOK = ymbStatus == "RUNNING" and ymbCount > 0

    ui.text(string.format(
        "ROOT CHAIN OK DH:%s CS:%s TC:%s CQ:%s TM:%s LP:%s RII:%s SR:%s DW:%s YMB:%s",
        dhOK and "OK" or "NG",
        csOK and "OK" or "NG",
        tcOK and "OK" or "NG",
        cqOK and "OK" or "NG",
        tmOK and "OK" or "NG",
        lpOK and "OK" or "NG",
        riiOK and "OK" or "NG",
        srOK and "OK" or "NG",
        dwOK and "OK" or "NG",
        ymbOK and "OK" or "NG"
    ))
end


local function drawImportantStores()
    ui.separator()
    ui.text("=== ACNEXTGEN ROOT / TRUNK SIGNALS ===")

    drawProfilerSummary()

    -- One Point One root chain display order:
    -- damper -> compliance -> carcass -> contact -> memory -> load path -> road -> slip -> driveline -> yaw
    drawDamperSpringSummary()
    drawDamperHysteresisSummary()
    drawComplianceStackSummary()
    drawTireCarcassSummary()
    drawContactQualitySummary()
    drawTireMemorySummary()
    drawLoadPathSummary()
    drawRoadInputInterpreterSummary()
    drawSlipRecoverySummary()
    drawDiffSummary()
    drawDrivelineWindupSummary()
    drawYawMomentBudgetSummary()

    if diff_lsd_observer and diff_lsd_observer.drawCompact then
        ui.separator()
        local ok,
              err =
            pcall(
                function()
                    diff_lsd_observer.drawCompact()
                end
            )

        if not ok then
            ui.text("diff_lsd_observer draw error")
            ui.text(tostring(err))
        end
    end

    drawTireContactSummary()
    drawTireResponseSummary()
    drawTireDynamicsSummary()
    drawSuspensionBridgeSummary()
    drawSuspensionIntegratedSummary()
    drawLoadBodySummary()
end


local function drawModule(name, wheelMode, entryMap)
    local entry =
        getEntry(
            name,
            entryMap
        )

    ui.separator()
    ui.text("--- " .. (TITLES[name] or name) .. " ---")

    if not entry then
        ui.text("Status : NOT REGISTERED")
        return
    end

    ui.text(
        string.format(
            "Status : %s / Updates : %.0f",
            getEntryStatus(entry),
            num(
                entry.updateCount,
                0.0
            )
        )
    )

    if entry.error and entry.error ~= "" then
        ui.text(
            "Error  : "
            ..
            basenamePath(
                entry.error
            )
        )
    end

    if not entry.module then
        return
    end

    if type(entry.module.debugStr) ~= "function" then
        return
    end

    if wheelMode then
        for i = 0, 3 do
            local ok, s =
                safeDebugStr(
                    entry.module,
                    i
                )

            ui.text(
                tostring(WHEEL_NAMES[i])
                ..
                " "
            )

            drawCompactText(
                ok
                and
                tostring(s)
                or
                (
                    "debug error: "
                    ..
                    tostring(s)
                ),
                4,
                88
            )
        end
    else
        local ok, s =
            safeDebugStr(
                entry.module,
                nil
            )

        drawCompactText(
            ok
            and
            tostring(s)
            or
            (
                "debug error: "
                ..
                tostring(s)
            ),
            8,
            92
        )
    end
end

local function drawGroups(entryMap)
    for _, group in ipairs(GROUPS) do
        ui.separator()
        ui.text(group.title)

        for _, name in ipairs(group.modules) do
            drawModule(
                name,
                group.wheel,
                entryMap
            )
        end
    end
end

local function drawWheelAudit(car, entryMap)
    local entry =
        getEntry(
            "wheel_audit",
            entryMap
        )

    if not entry or not entry.module then
        return
    end

    if type(entry.module.draw) ~= "function" then
        return
    end

    ui.separator()

    local ok, err =
        pcall(
            function()
                entry.module.draw(car)
            end
        )

    if not ok then
        ui.text("wheel_audit draw error")
        ui.text(tostring(err))
    end
end


local FRONT_ERROR_LIMIT = 8

local function drawErrorFront(runtime, modules)
    local hasError = false

    if runtime and runtime.lastError and runtime.lastError ~= "" then
        hasError = true
    end

    if runtime and num(runtime.errorCount, 0.0) > 0 then
        hasError = true
    end

    if modules then
        for i = 1, #modules do
            local entry = modules[i]

            if entry then
                local status = tostring(entry.lastStatus or "")
                local err = tostring(entry.error or "")

                if err ~= "" or status == "ERROR" or status == "LOAD ERROR" then
                    hasError = true
                    break
                end
            end
        end
    end

    ui.separator()
    ui.text("=== ERROR CHECK ===")

    if not hasError then
        ui.text("No active errors")
        return
    end

    if runtime and runtime.lastError and runtime.lastError ~= "" then
        ui.text("Last Error:")
        ui.text(
            basenamePath(
                runtime.lastError
            )
        )
    end

    if not modules then
        return
    end

    local shown = 0

    for i = 1, #modules do
        local entry = modules[i]

        if entry then
            local status = tostring(entry.lastStatus or "")
            local err = tostring(entry.error or "")

            if err ~= "" or status == "ERROR" or status == "LOAD ERROR" then
                shown = shown + 1

                ui.text(
                    string.format(
                        "%-22s : %s",
                        shortenText(tostring(entry.name), 22),
                        status ~= "" and status or "ERROR"
                    )
                )

                if err ~= "" then
                    ui.text(
                        "  "
                        ..
                        basenamePath(
                            err
                        )
                    )
                end

                if shown >= FRONT_ERROR_LIMIT then
                    ui.text("...more errors hidden...")
                    return
                end
            end
        end
    end
end

local function drawRuntimeStatus(runtime, modules)
    ui.separator()
    ui.text("=== ACNextGen Runtime ===")

    if not runtime then
        ui.text("Runtime NIL")
        return
    end

    textKV("Frame", num(runtime.frame, 0.0), "%d")
    textKV("Time", num(runtime.time, 0.0), "%.2f")
    textKV("Car OK", tostring(runtime.carOK))
    textKV("Wheels OK", tostring(runtime.wheelsOK))
    textKV("Loaded", num(runtime.loadedCount, 0.0), "%d")
    textKV("Enabled", num(runtime.enabledCount, 0.0), "%d")
    textKV("Errors", num(runtime.errorCount, 0.0), "%d")

    if runtime.lastError and runtime.lastError ~= "" then
        ui.text("Last Error:")
        ui.text(
            basenamePath(
                runtime.lastError
            )
        )
    end

    if PERFORMANCE_LITE then
        ui.text("Monitor : PERFORMANCE LITE")
        return
    end

    if not modules then
        return
    end

    ui.separator()
    ui.text("=== MODULE STATUS SUMMARY ===")

    for i = 1, #modules do
        local entry =
            modules[i]

        if entry then
            ui.text(
                string.format(
                    "%-22s : %s",
                    shortenText(tostring(entry.name), 22),
                    tostring(
                        entry.lastStatus
                        or
                        "UNKNOWN"
                    )
                )
            )
        end
    end
end

--============================================================
-- Public API
--============================================================

function M.init()
    if ac and ac.log then
        ac.log("[ACNextGen] Observer v1.1.4 One Point One Integration Loaded")
    end
end

function M.update(dt, car)
    car =
        car
        or
        ac.getCar(0)

    if not car then
        return
    end

    frameCount =
        frameCount
        +
        1

    logTimer =
        logTimer
        +
        num(
            dt,
            0.0
        )

    if logTimer >= LOG_INTERVAL then
        logTimer =
            0.0

        if ac and ac.log then
            ac.log(
                string.format(
                    "[NGP] RPM=%.0f Gear=%d Speed=%.1f",
                    num(
                        safeField(
                            car,
                            "rpm",
                            0.0
                        ),
                        0.0
                    ),
                    num(
                        safeField(
                            car,
                            "gear",
                            0.0
                        ),
                        0.0
                    ),
                    num(
                        safeField(
                            car,
                            "speedKmh",
                            0.0
                        ),
                        0.0
                    )
                )
            )
        end
    end
end

function M.drawUI(runtime, modules)
    local car = ac.getCar(0)

    if not car then
        ui.text("Vehicle not found")
        drawErrorFront(runtime, modules)
        drawRuntimeStatus(runtime, modules)
        return
    end

    drawCarSummary(car)
    drawInputs(car)
    drawErrorFront(runtime, modules)

    if PERFORMANCE_LITE then
        drawRootHealth()
        drawRuntimeStatus(runtime, modules)
        return
    end

    local entryMap = buildEntryMap(modules)
    drawRootHealth()
    drawImportantStores()
    drawGroups(entryMap)
    drawWheelAudit(car, entryMap)
    drawRuntimeStatus(runtime, modules)
end

return M