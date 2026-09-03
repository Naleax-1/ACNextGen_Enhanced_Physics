---@diagnostic disable: undefined-global

--============================================================
-- suspension_contact_input.lua
-- ACNextGen V1.1.5 Stable
-- Tire Contact Response -> Suspension Input Bridge
--
-- Purpose:
--   Convert tire/contact/road/load-path information into shared
--   suspension input signals for suspension.lua, damper_model.lua,
--   progressive_spring.lua, and road_body_input.lua.
--
-- Policy:
--   No direct AC physics overwrite.
--   Keep existing store-key compatibility.
--============================================================

local M = {}

local WHEEL_NAMES = { [0] = "FL", [1] = "FR", [2] = "RL", [3] = "RR" }

M.params = {
    inputTau = 0.055,
    reboundTau = 0.120,

    energyInputGain = 0.52,
    lossInputGain = 0.34,
    qualityGain = 0.24,

    hopInputGain = 0.28,
    hopLoadGain = 0.00012,
    memoryInputGain = 0.08,
    tireLimitInputGain = 0.08,

    loadNormInputGain = 0.10,
    damperFeedbackGain = 0.000035,
    springFeedbackGain = 0.000020,

    minSuspScale = 0.70,
    maxSuspScale = 1.35,

    minRoadInput = 0.0,
    maxRoadInput = 1.0,

    frontGain = 0.95,
    rearGain = 1.10,

    contactDropGain = 0.45,
    impulseGain = 0.42,
    impulseDecayTau = 0.180,

    bodyFlexInputGain = 0.06,
    bodyStiffInputLoss = 0.04,

    damageInputGain = 0.08,

    brakeLockInputGain = 0.05,
    brakeThermalInputGain = 0.04,
    chassisEnergyInputGain = 0.06,
    chassisFlexInputGain = 0.06,
    weightBiasInputGain = 0.04,
    damageRuntimeInputGain = 0.06,

    roadImpactInputGain = 0.10,
    roadShockInputGain = 0.09,
    roadTextureInputGain = 0.05,
    roadUnloadDropGain = 0.16,
    roadBottomingInputGain = 0.08,
    roadSurfaceLimitGain = 0.07,

    loadPathLossGain = 0.10,
    loadPathForceLeakGain = 0.08,
    loadPathVerticalFlowGain = 0.07,
    loadPathDelayGain = 0.05,
    loadPathBodyAbsorbGain = 0.04,
    tireDeliveryLossGain = 0.08,

    bodyHeaveGain = 0.04,
    bodyPitchRollGain = 0.025,

    maxPreserveInputAdd = 0.25,
    loadReference = 3000.0,

    minDt = 0.00005,
    maxDt = 0.050,

    debugStoreInterval = 0.25,
}

local state = {
    contact = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    quality = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    trust = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    loss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    energy = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    roadInput = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    suspScale = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    contactDrop = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    impulse = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    hop = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    hopLoad = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    memory = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireLimit = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    load = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadNorm = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    damperFeedback = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    springFeedback = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    roadImpact = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    roadShock = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    roadTexture = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    roadUnload = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    roadBottoming = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    roadSurfaceLimit = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    loadPathLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadPathForceLeak = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadPathVerticalFlow = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadPathDelay = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadPathBodyAbsorb = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireDelivery = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    frontInput = 0.0,
    rearInput = 0.0,
    leftInput = 0.0,
    rightInput = 0.0,
    avgRoadInput = 0.0,
    avgSuspScale = 1.0,
    avgImpulse = 0.0,
    avgDrop = 0.0,
    maxRoadInput = 0.0,
    maxImpulse = 0.0,
    maxDrop = 0.0,

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyHeave = 0.0,
    bodyPitch = 0.0,
    bodyRoll = 0.0,

    vehicleDamage = 0.0,

    brakeLockAvg = 0.0,
    brakeThermalStress = 0.0,
    chassisEnergy = 0.0,
    chassisFlexEnergy = 0.0,
    weightBiasStress = 0.0,
    damageRuntimeAdd = 0.0,
    preserveInputAdd = 0.0,

    status = "INIT",
    updateCount = 0,

    contactLinked = false,
    hopLinked = false,
    memoryLinked = false,
    loadLinked = false,
    bodyLinked = false,
    damageLinked = false,
    feedbackLinked = false,
    brakeLinked = false,
    thermalLinked = false,
    chassisLinked = false,
    weightLinked = false,
    damageRuntimeLinked = false,
    roadInputLinked = false,
    loadPathLinked = false,
    carLinked = false,
    wheelLinked = false,

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

local function clamp(v, mn, mx)
    v = safeNumber(v, mn or 0.0)
    if v < mn then return mn end
    if v > mx then return mx end
    return v
end

local function abs(v)
    return math.abs(safeNumber(v, 0.0))
end

local function safeLoadRaw(key)
    if not ac or not ac.load then return nil end
    local ok, value = pcall(function()
        return ac.load(key)
    end)
    if not ok then return nil end
    return value
end

local function safeLoad(key, defaultValue)
    local value = safeLoadRaw(key)
    if value == nil then return defaultValue or 0.0 end
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
    return defaultValue, nil
end

local function safeStore(key, value)
    if not ac or not ac.store then return end
    pcall(function()
        ac.store(key, value)
    end)
end

local function safeField(obj, field, defaultValue)
    if not obj then return defaultValue end
    local ok, value = pcall(function()
        return obj[field]
    end)
    if not ok or value == nil then return defaultValue end
    return value
end

local function lowPass(cur, tgt, tau, dt)
    cur = safeNumber(cur, 0.0)
    tgt = safeNumber(tgt, 0.0)
    tau = safeNumber(tau, 0.001)
    dt = safeNumber(dt, 0.0)

    if tau <= 0.0001 then return tgt end
    return cur + (tgt - cur) * clamp(dt / (tau + dt), 0.0, 1.0)
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
    local ok, car = pcall(function()
        return ac.getCar(0)
    end)
    if ok then return car end
    return nil
end

local function getWheel(car, index)
    if not car then return nil end

    local wheels = safeField(car, "wheels", nil)
    if not wheels then return nil end

    local ok, wheel = pcall(function()
        return wheels[index]
    end)
    if ok and wheel ~= nil then return wheel end

    ok, wheel = pcall(function()
        return wheels[index + 1]
    end)
    if ok then return wheel end

    return nil
end

local function axleGain(index)
    if index <= 1 then return M.params.frontGain end
    return M.params.rearGain
end

local function resetLinks()
    state.contactLinked = false
    state.hopLinked = false
    state.memoryLinked = false
    state.loadLinked = false
    state.bodyLinked = false
    state.damageLinked = false
    state.feedbackLinked = false
    state.brakeLinked = false
    state.thermalLinked = false
    state.chassisLinked = false
    state.weightLinked = false
    state.damageRuntimeLinked = false
    state.roadInputLinked = false
    state.loadPathLinked = false
    state.carLinked = false
    state.wheelLinked = false
end

local function readBodyAndDamage()
    local bodyRaw = safeLoadRaw("ngp_body_rigidity")
    local flexRaw = safeLoadRaw("ngp_body_flex_factor")
    local rbiRaw = safeLoadRaw("ngp_rbi_body_rigidity")

    state.bodyLinked = bodyRaw ~= nil or flexRaw ~= nil or rbiRaw ~= nil

    state.bodyRigidity = clamp(
        safeNumber(bodyRaw, safeNumber(rbiRaw, 1.0)),
        0.20,
        1.35
    )

    state.bodyFlexFactor = clamp(
        safeNumber(flexRaw, safeLoad("ngp_rbi_body_flex_factor", 1.0)),
        0.25,
        1.80
    )

    state.bodyHeave = clamp(
        safeLoad("ngp_body_heave_input", safeLoad("ngp_road_body_heave", 0.0)),
        -2.0,
        2.0
    )

    state.bodyPitch = clamp(
        safeLoad("ngp_body_pitch_input", safeLoad("ngp_road_body_pitch", 0.0)),
        -2.0,
        2.0
    )

    state.bodyRoll = clamp(
        safeLoad("ngp_body_roll_input", safeLoad("ngp_road_body_roll", 0.0)),
        -2.0,
        2.0
    )

    if state.bodyHeave ~= 0.0 or state.bodyPitch ~= 0.0 or state.bodyRoll ~= 0.0 then
        state.bodyLinked = true
    end

    local damageRaw =
        safeLoadRaw("ngp_condition_total")
        or safeLoadRaw("ngp_vehicle_condition_total")
        or safeLoadRaw("ngp_damage_chassis")

    state.damageLinked = damageRaw ~= nil
    state.vehicleDamage = clamp(safeNumber(damageRaw, 0.0), 0.0, 1.0)
end

local function readPreserveInputs()
    local lockSum = 0.0
    local brakeLinked = false

    for i = 0, 3 do
        local lockRaw = safeLoadRaw("ngp_brake_lock_" .. i)
        if lockRaw ~= nil then brakeLinked = true end
        lockSum = lockSum + clamp(safeNumber(lockRaw, 0.0), 0.0, 1.0)
    end

    state.brakeLockAvg = lockSum * 0.25
    state.brakeLinked = brakeLinked

    local thermalRaw =
        safeLoadRaw("ngp_brake_root_heat_avg")
        or safeLoadRaw("ngp_virtual_thermal_stress")
        or safeLoadRaw("ngp_brake_temp_avg")

    state.thermalLinked = thermalRaw ~= nil
    state.brakeThermalStress = clamp(safeNumber(thermalRaw, 0.0), 0.0, 1.0)

    local energyRaw =
        safeLoadRaw("ngp_chassis_energy")
        or safeLoadRaw("ngp_chassis_body_energy")
        or safeLoadRaw("ngp_body_energy")

    local flexRaw =
        safeLoadRaw("ngp_chassis_flex_energy")
        or safeLoadRaw("ngp_flex_energy")

    state.chassisLinked = energyRaw ~= nil or flexRaw ~= nil
    state.chassisEnergy = clamp(safeNumber(energyRaw, 0.0), 0.0, 1.0)
    state.chassisFlexEnergy = clamp(safeNumber(flexRaw, 0.0), 0.0, 1.0)

    local fb = safeLoadRaw("ngp_weight_front_bias")
    local rb = safeLoadRaw("ngp_weight_rear_bias")
    local lb = safeLoadRaw("ngp_weight_left_bias")
    local rr = safeLoadRaw("ngp_weight_right_bias")

    state.weightLinked = fb ~= nil or rb ~= nil or lb ~= nil or rr ~= nil

    state.weightBiasStress = clamp(
        abs(safeNumber(fb, 0.5) - safeNumber(rb, 0.5))
        + abs(safeNumber(lb, 0.5) - safeNumber(rr, 0.5)),
        0.0,
        1.0
    )

    local runtimeDamage =
        safeLoadRaw("ngp_damage_runtime_add")
        or safeLoadRaw("ngp_damage_event_runtime_add")

    state.damageRuntimeLinked = runtimeDamage ~= nil
    state.damageRuntimeAdd = clamp(safeNumber(runtimeDamage, 0.0), 0.0, 1.0)

    state.preserveInputAdd = clamp(
        state.brakeLockAvg * M.params.brakeLockInputGain
        + state.brakeThermalStress * M.params.brakeThermalInputGain
        + state.chassisEnergy * M.params.chassisEnergyInputGain
        + state.chassisFlexEnergy * M.params.chassisFlexInputGain
        + state.weightBiasStress * M.params.weightBiasInputGain
        + state.damageRuntimeAdd * M.params.damageRuntimeInputGain,
        0.0,
        M.params.maxPreserveInputAdd
    )
end

local function readWheelLoad(index, wheel)
    local value, key = safeLoadAlt(
        nil,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index,
        "ngp_contact_load_" .. index,
        "ngp_load_path_load_" .. index,
        "ngp_hub_load_" .. index
    )

    if key ~= nil then
        state.loadLinked = true
        return clamp(value or 0.0, 0.0, 12000.0)
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        state.loadLinked = true
        local n = safeNumber(dlt, 0.0)
        if n > 1500.0 then
            return clamp(n, 0.0, 12000.0)
        end
        return clamp(M.params.loadReference + n, 0.0, 12000.0)
    end

    if wheel then
        local wLoad =
            safeField(wheel, "load", nil)
            or safeField(wheel, "loadK", nil)
            or nil
            or nil
            or nil

        if wLoad ~= nil then
            state.wheelLinked = true
            return clamp(safeNumber(wLoad, M.params.loadReference), 0.0, 12000.0)
        end
    end

    return M.params.loadReference
end

local function readContact(index)
    local contact, contactKey = safeLoadAlt(
        nil,
        "ngp_tc_contact_" .. index,
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index
    )

    local quality, qualityKey = safeLoadAlt(
        nil,
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index,
        "ngp_sci_quality_" .. index
    )

    local trust, trustKey = safeLoadAlt(
        nil,
        "ngp_contact_trust_" .. index,
        "ngp_recovery_trust_" .. index,
        "ngp_carcass_grip_gate_" .. index
    )

    local loss, lossKey = safeLoadAlt(
        nil,
        "ngp_contact_loss_" .. index,
        "ngp_tire_contact_loss_" .. index,
        "ngp_tcr_contact_loss_" .. index,
        "ngp_contact_patch_loss_" .. index
    )

    local energy, energyKey = safeLoadAlt(
        nil,
        "ngp_tcr_energy_push_" .. index,
        "ngp_tc_energy_" .. index,
        "ngp_tire_slip_energy_" .. index,
        "ngp_tdyn_slip_energy_" .. index,
        "ngp_slip_recovery_slip_" .. index
    )

    if contactKey or qualityKey or trustKey or lossKey or energyKey then
        state.contactLinked = true
    end

    state.contact[index] = clamp(safeNumber(contact, 1.0), 0.0, 1.2)
    state.quality[index] = clamp(safeNumber(quality, state.contact[index]), 0.0, 1.2)
    state.trust[index] = clamp(safeNumber(trust, state.quality[index]), 0.0, 1.2)

    if loss == nil then
        loss = 1.0 - math.min(state.contact[index], state.quality[index], state.trust[index])
    end

    state.loss[index] = clamp(safeNumber(loss, 0.0), 0.0, 1.0)
    state.energy[index] = clamp(safeNumber(energy, 0.0), 0.0, 1.0)
end

local function readHopMemoryLimit(index)
    local hop, hopKey = safeLoadAlt(
        nil,
        "ngp_tire_hop_energy_" .. index,
        "ngp_tire_hop_" .. index,
        "ngp_tirehop_energy_" .. index,
        "ngp_rii_hop_" .. index,
        "ngp_hop_risk_" .. index
    )

    local hopLoad, hopLoadKey = safeLoadAlt(
        nil,
        "ngp_tire_hop_loadpulse_" .. index,
        "ngp_tirehop_" .. index,
        "ngp_impact_load_pulse_" .. index
    )

    if hopKey or hopLoadKey then state.hopLinked = true end

    state.hop[index] = clamp(safeNumber(hop, 0.0), 0.0, 1.0)
    state.hopLoad[index] = abs(safeNumber(hopLoad, 0.0))

    local memory, memKey = safeLoadAlt(
        nil,
        "ngp_tire_memory_" .. index,
        "ngp_tyrememory_" .. index,
        "ngp_rubber_memory_" .. index,
        "ngp_slide_memory_" .. index,
        "ngp_slip_slide_memory_" .. index
    )

    if memKey ~= nil then state.memoryLinked = true end
    state.memory[index] = clamp(safeNumber(memory, 0.0), 0.0, 1.0)

    local limit, limitKey = safeLoadAlt(
        nil,
        "ngp_tire_limit_" .. index,
        "ngp_road_surface_limit_" .. index,
        "ngp_rii_surface_limit_" .. index,
        "ngp_slip_snap_risk_" .. index,
        "ngp_snap_risk_" .. index
    )

    if limitKey ~= nil then state.roadInputLinked = true end
    state.tireLimit[index] = clamp(safeNumber(limit, 0.0), 0.0, 2.0)
end

local function readFeedback(index)
    local damper, damperKey = safeLoadAlt(
        nil,
        "ngp_damper_force_" .. index,
        "ngp_damper_" .. index,
        "ngp_damper_hyst_force_" .. index,
        "ngp_susp_damper_" .. index
    )

    local spring, springKey = safeLoadAlt(
        nil,
        "ngp_spring_force_" .. index,
        "ngp_progressive_spring_" .. index,
        "ngp_spring_raw_" .. index,
        "ngp_susp_spring_" .. index
    )

    if damperKey or springKey then state.feedbackLinked = true end

    state.damperFeedback[index] = safeNumber(damper, 0.0)
    state.springFeedback[index] = safeNumber(spring, 0.0)
end

local function readRoadInputs(index)
    local impact, impactKey = safeLoadAlt(
        nil,
        "ngp_rii_impact_" .. index,
        "ngp_road_impact_" .. index,
        "ngp_impact_value_" .. index
    )

    local shock, shockKey = safeLoadAlt(
        nil,
        "ngp_rii_shock_" .. index,
        "ngp_road_shock_" .. index,
        "ngp_road_input_shock_" .. index
    )

    local texture, textureKey = safeLoadAlt(
        nil,
        "ngp_rii_texture_" .. index,
        "ngp_road_texture_" .. index
    )

    local unload, unloadKey = safeLoadAlt(
        nil,
        "ngp_rii_unload_" .. index,
        "ngp_unload_event_" .. index
    )

    local bottoming, bottomingKey = safeLoadAlt(
        nil,
        "ngp_rii_bottoming_" .. index,
        "ngp_bottoming_risk_" .. index
    )

    local surface, surfaceKey = safeLoadAlt(
        nil,
        "ngp_rii_surface_limit_" .. index,
        "ngp_road_surface_limit_" .. index
    )

    if impactKey or shockKey or textureKey or unloadKey or bottomingKey or surfaceKey then
        state.roadInputLinked = true
    end

    state.roadImpact[index] = clamp(safeNumber(impact, 0.0), 0.0, 1.2)
    state.roadShock[index] = clamp(safeNumber(shock, 0.0), 0.0, 1.2)
    state.roadTexture[index] = clamp(safeNumber(texture, 0.0), 0.0, 1.2)
    state.roadUnload[index] = clamp(safeNumber(unload, 0.0), 0.0, 1.2)
    state.roadBottoming[index] = clamp(safeNumber(bottoming, 0.0), 0.0, 1.2)
    state.roadSurfaceLimit[index] = clamp(safeNumber(surface, 0.0), 0.0, 1.2)
end

local function readLoadPath(index)
    local loss, lossKey = safeLoadAlt(
        nil,
        "ngp_load_path_loss_" .. index,
        "ngp_lp_loss_" .. index,
        "ngp_road_path_loss_" .. index
    )

    local leak, leakKey = safeLoadAlt(
        nil,
        "ngp_load_path_force_leak_" .. index,
        "ngp_lp_force_leak_" .. index,
        "ngp_road_force_leak_" .. index
    )

    local flow, flowKey = safeLoadAlt(
        nil,
        "ngp_load_path_vertical_flow_" .. index,
        "ngp_lp_vertical_flow_" .. index,
        "ngp_road_vertical_flow_" .. index
    )

    local delay, delayKey = safeLoadAlt(
        nil,
        "ngp_load_path_delay_" .. index,
        "ngp_lp_delay_" .. index
    )

    local absorb, absorbKey = safeLoadAlt(
        nil,
        "ngp_load_path_body_absorb_" .. index,
        "ngp_road_body_absorb_" .. index
    )

    local delivery, deliveryKey = safeLoadAlt(
        nil,
        "ngp_load_path_tire_delivery_" .. index,
        "ngp_road_tire_delivery_" .. index,
        "ngp_lp_tire_delivery_" .. index
    )

    if lossKey or leakKey or flowKey or delayKey or absorbKey or deliveryKey then
        state.loadPathLinked = true
    end

    state.loadPathLoss[index] = clamp(safeNumber(loss, 0.0), 0.0, 1.2)
    state.loadPathForceLeak[index] = clamp(safeNumber(leak, 0.0), 0.0, 1.2)
    state.loadPathVerticalFlow[index] = clamp(safeNumber(flow, 0.0), 0.0, 1.2)
    state.loadPathDelay[index] = clamp(safeNumber(delay, 0.0), 0.0, 1.2)
    state.loadPathBodyAbsorb[index] = clamp(safeNumber(absorb, 0.0), 0.0, 1.2)
    state.tireDelivery[index] = clamp(safeNumber(delivery, 1.0), 0.0, 1.2)
end

local function readLoadNorm(index, wheel)
    state.load[index] = readWheelLoad(index, wheel)

    local loadNorm, normKey = safeLoadAlt(
        nil,
        "ngp_wheel_load_norm_" .. index,
        "ngp_brake_load_norm_" .. index,
        "ngp_load_path_load_norm_" .. index,
        "ngp_lp_load_norm_" .. index
    )

    if normKey ~= nil then
        state.loadLinked = true
        state.loadNorm[index] = clamp(loadNorm or 1.0, 0.0, 2.5)
    else
        state.loadNorm[index] = clamp(
            state.load[index] / math.max(M.params.loadReference, 1.0),
            0.0,
            2.5
        )
    end
end

local function updateWheel(index, car, dt)
    local wheel = getWheel(car, index)
    if wheel ~= nil then
        state.wheelLinked = true
    end

    readContact(index)
    readHopMemoryLimit(index)
    readFeedback(index)
    readRoadInputs(index)
    readLoadPath(index)
    readLoadNorm(index, wheel)

    local bodySoft =
        math.max(state.bodyFlexFactor - 1.0, 0.0) * M.params.bodyFlexInputGain
        - math.max(state.bodyRigidity - 1.0, 0.0) * M.params.bodyStiffInputLoss

    local bodyRoad =
        abs(state.bodyHeave) * M.params.bodyHeaveGain
        + (abs(state.bodyPitch) + abs(state.bodyRoll)) * M.params.bodyPitchRollGain

    local deliveryLoss = clamp(1.0 - math.min(state.tireDelivery[index], 1.0), 0.0, 1.0)

    local contactDropTarget = clamp(
        state.loss[index] * M.params.contactDropGain
        + state.roadUnload[index] * M.params.roadUnloadDropGain
        + deliveryLoss * 0.10,
        0.0,
        1.0
    )

    state.contactDrop[index] =
        lowPass(state.contactDrop[index], contactDropTarget, M.params.inputTau, dt)

    local impulseTarget = clamp(
        (state.energy[index] * 0.60 + state.loss[index] * 0.28 + state.hop[index] * 0.12) * M.params.impulseGain
        + state.hopLoad[index] * M.params.hopLoadGain
        + state.roadImpact[index] * 0.16
        + state.roadShock[index] * 0.14
        + state.loadPathVerticalFlow[index] * 0.08,
        0.0,
        1.0
    )

    local impulseTau =
        impulseTarget > state.impulse[index]
        and M.params.inputTau
        or M.params.impulseDecayTau

    state.impulse[index] =
        lowPass(state.impulse[index], impulseTarget, impulseTau, dt)

    local roadTarget = clamp(
        state.energy[index] * M.params.energyInputGain
        + state.loss[index] * M.params.lossInputGain
        + (1.0 - clamp(state.quality[index], 0.0, 1.0)) * M.params.qualityGain
        + state.hop[index] * M.params.hopInputGain
        + state.memory[index] * M.params.memoryInputGain
        + state.tireLimit[index] * M.params.tireLimitInputGain
        + math.max(state.loadNorm[index] - 1.0, 0.0) * M.params.loadNormInputGain
        + abs(state.damperFeedback[index]) * M.params.damperFeedbackGain
        + abs(state.springFeedback[index]) * M.params.springFeedbackGain
        + state.roadImpact[index] * M.params.roadImpactInputGain
        + state.roadShock[index] * M.params.roadShockInputGain
        + state.roadTexture[index] * M.params.roadTextureInputGain
        + state.roadBottoming[index] * M.params.roadBottomingInputGain
        + state.roadSurfaceLimit[index] * M.params.roadSurfaceLimitGain
        + state.loadPathLoss[index] * M.params.loadPathLossGain
        + state.loadPathForceLeak[index] * M.params.loadPathForceLeakGain
        + state.loadPathVerticalFlow[index] * M.params.loadPathVerticalFlowGain
        + state.loadPathDelay[index] * M.params.loadPathDelayGain
        + state.loadPathBodyAbsorb[index] * M.params.loadPathBodyAbsorbGain
        + deliveryLoss * M.params.tireDeliveryLossGain
        + bodySoft
        + bodyRoad
        + state.vehicleDamage * M.params.damageInputGain
        + state.preserveInputAdd,
        M.params.minRoadInput,
        M.params.maxRoadInput
    )

    state.roadInput[index] =
        lowPass(state.roadInput[index], roadTarget * axleGain(index), M.params.inputTau, dt)

    local scaleTarget = clamp(
        1.0
        + state.roadInput[index] * 0.25
        + state.impulse[index] * 0.06
        - state.contactDrop[index] * 0.18,
        M.params.minSuspScale,
        M.params.maxSuspScale
    )

    state.suspScale[index] =
        lowPass(state.suspScale[index], scaleTarget, M.params.reboundTau, dt)
end

local function calculateGlobal()
    state.frontInput = (state.roadInput[0] + state.roadInput[1]) * 0.5
    state.rearInput = (state.roadInput[2] + state.roadInput[3]) * 0.5
    state.leftInput = (state.roadInput[0] + state.roadInput[2]) * 0.5
    state.rightInput = (state.roadInput[1] + state.roadInput[3]) * 0.5

    local sumInput = 0.0
    local sumScale = 0.0
    local sumImpulse = 0.0
    local sumDrop = 0.0
    local maxInput = 0.0
    local maxImpulse = 0.0
    local maxDrop = 0.0

    for i = 0, 3 do
        local input = state.roadInput[i] or 0.0
        local impulse = state.impulse[i] or 0.0
        local drop = state.contactDrop[i] or 0.0

        sumInput = sumInput + input
        sumScale = sumScale + (state.suspScale[i] or 1.0)
        sumImpulse = sumImpulse + impulse
        sumDrop = sumDrop + drop

        if input > maxInput then maxInput = input end
        if impulse > maxImpulse then maxImpulse = impulse end
        if drop > maxDrop then maxDrop = drop end
    end

    state.avgRoadInput = sumInput * 0.25
    state.avgSuspScale = sumScale * 0.25
    state.avgImpulse = sumImpulse * 0.25
    state.avgDrop = sumDrop * 0.25
    state.maxRoadInput = maxInput
    state.maxImpulse = maxImpulse
    state.maxDrop = maxDrop
end

local function exportWheel(index)
    safeStore("ngp_susp_contact_input_" .. index, state.roadInput[index])
    safeStore("ngp_susp_contact_scale_" .. index, state.suspScale[index])
    safeStore("ngp_susp_road_impulse_" .. index, state.impulse[index])
    safeStore("ngp_susp_contact_drop_" .. index, state.contactDrop[index])

    safeStore("ngp_sci_input_" .. index, state.roadInput[index])
    safeStore("ngp_sci_scale_" .. index, state.suspScale[index])
    safeStore("ngp_sci_impulse_" .. index, state.impulse[index])
    safeStore("ngp_sci_drop_" .. index, state.contactDrop[index])

    safeStore("ngp_sci_road_input_" .. index, state.roadInput[index])
    safeStore("ngp_sci_susp_scale_" .. index, state.suspScale[index])

    if not state.debugStoreNow then return end

    safeStore("ngp_sci_contact_" .. index, state.contact[index])
    safeStore("ngp_sci_quality_" .. index, state.quality[index])
    safeStore("ngp_sci_trust_" .. index, state.trust[index])
    safeStore("ngp_sci_loss_" .. index, state.loss[index])
    safeStore("ngp_sci_energy_" .. index, state.energy[index])
    safeStore("ngp_sci_hop_" .. index, state.hop[index])
    safeStore("ngp_sci_memory_" .. index, state.memory[index])
    safeStore("ngp_sci_load_" .. index, state.load[index])
    safeStore("ngp_sci_load_norm_" .. index, state.loadNorm[index])
    safeStore("ngp_sci_road_impact_" .. index, state.roadImpact[index])
    safeStore("ngp_sci_road_shock_" .. index, state.roadShock[index])
    safeStore("ngp_sci_load_path_loss_" .. index, state.loadPathLoss[index])
    safeStore("ngp_sci_tire_delivery_" .. index, state.tireDelivery[index])
end

local function exportGlobal()
    safeStore("ngp_susp_contact_input_front", state.frontInput)
    safeStore("ngp_susp_contact_input_rear", state.rearInput)
    safeStore("ngp_susp_contact_input_left", state.leftInput)
    safeStore("ngp_susp_contact_input_right", state.rightInput)
    safeStore("ngp_susp_contact_input_avg", state.avgRoadInput)
    safeStore("ngp_susp_contact_scale_avg", state.avgSuspScale)

    safeStore("ngp_sci_status", state.status)
    safeStore("ngp_sci_update_count", state.updateCount)
    safeStore("ngp_sci_preserve_input_add", state.preserveInputAdd)

    safeStore("ngp_sci_avg_input", state.avgRoadInput)
    safeStore("ngp_sci_avg_scale", state.avgSuspScale)
    safeStore("ngp_sci_avg_impulse", state.avgImpulse)
    safeStore("ngp_sci_avg_drop", state.avgDrop)
    safeStore("ngp_sci_max_input", state.maxRoadInput)
    safeStore("ngp_sci_max_impulse", state.maxImpulse)
    safeStore("ngp_sci_max_drop", state.maxDrop)

    if not state.debugStoreNow then return end

    safeStore("ngp_sci_contact_linked", state.contactLinked and 1 or 0)
    safeStore("ngp_sci_hop_linked", state.hopLinked and 1 or 0)
    safeStore("ngp_sci_memory_linked", state.memoryLinked and 1 or 0)
    safeStore("ngp_sci_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_sci_body_linked", state.bodyLinked and 1 or 0)
    safeStore("ngp_sci_damage_linked", state.damageLinked and 1 or 0)
    safeStore("ngp_sci_feedback_linked", state.feedbackLinked and 1 or 0)
    safeStore("ngp_sci_brake_linked", state.brakeLinked and 1 or 0)
    safeStore("ngp_sci_thermal_linked", state.thermalLinked and 1 or 0)
    safeStore("ngp_sci_chassis_linked", state.chassisLinked and 1 or 0)
    safeStore("ngp_sci_weight_linked", state.weightLinked and 1 or 0)
    safeStore("ngp_sci_damage_runtime_linked", state.damageRuntimeLinked and 1 or 0)
    safeStore("ngp_sci_road_input_linked", state.roadInputLinked and 1 or 0)
    safeStore("ngp_sci_load_path_linked", state.loadPathLinked and 1 or 0)
    safeStore("ngp_sci_car_linked", state.carLinked and 1 or 0)
    safeStore("ngp_sci_wheel_linked", state.wheelLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i)
    end
    exportGlobal()
end

function M.init()
    state.status = "INIT"
    calculateGlobal()
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = clamp(safeNumber(dt, 0.0), M.params.minDt, M.params.maxDt)
    updateDebugGate(dt)

    if not car then
        car = safeGetCar()
    end

    resetLinks()

    state.carLinked = car ~= nil

    readBodyAndDamage()
    readPreserveInputs()

    for i = 0, 3 do
        updateWheel(i, car, dt)
        exportWheel(i)
    end

    calculateGlobal()

    if state.carLinked then
        state.status = state.wheelLinked and "RUNNING" or "STORE ONLY"
    else
        state.status = "STORE ONLY"
    end

    exportGlobal()
end

function M.getInput(index)
    return state.roadInput[index] or 0.0
end

function M.getScale(index)
    return state.suspScale[index] or 1.0
end

function M.getImpulse(index)
    return state.impulse[index] or 0.0
end

function M.getDrop(index)
    return state.contactDrop[index] or 0.0
end

function M.getState(index)
    if index == nil then return state end
    return {
        input = state.roadInput[index] or 0.0,
        scale = state.suspScale[index] or 1.0,
        impulse = state.impulse[index] or 0.0,
        drop = state.contactDrop[index] or 0.0,
        contact = state.contact[index] or 1.0,
        quality = state.quality[index] or 1.0,
        loss = state.loss[index] or 0.0,
        load = state.load[index] or 0.0,
    }
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0
        return string.format(
            "%s Input %.3f Scale %.2f Imp %.3f Drop %.3f / CQ %.2f Loss %.2f LoadN %.2f",
            tostring(WHEEL_NAMES[i] or i),
            state.roadInput[i] or 0.0,
            state.suspScale[i] or 1.0,
            state.impulse[i] or 0.0,
            state.contactDrop[i] or 0.0,
            state.quality[i] or 1.0,
            state.loss[i] or 0.0,
            state.loadNorm[i] or 1.0
        )
    end

    return string.format(
        "Status %s / Count %.0f\n" ..
        "Input %.3f %.3f %.3f %.3f / Avg %.3f Max %.3f / Preserve %.3f\n" ..
        "Scale %.2f %.2f %.2f %.2f / Avg %.2f\n" ..
        "Impulse %.3f %.3f %.3f %.3f / Drop %.3f %.3f %.3f %.3f\n" ..
        "Links Contact:%s Hop:%s Mem:%s Load:%s Body:%s Damage:%s Brake:%s Therm:%s Chassis:%s Weight:%s Road:%s LP:%s",
        tostring(state.status),
        state.updateCount or 0,

        state.roadInput[0] or 0.0,
        state.roadInput[1] or 0.0,
        state.roadInput[2] or 0.0,
        state.roadInput[3] or 0.0,
        state.avgRoadInput or 0.0,
        state.maxRoadInput or 0.0,
        state.preserveInputAdd or 0.0,

        state.suspScale[0] or 1.0,
        state.suspScale[1] or 1.0,
        state.suspScale[2] or 1.0,
        state.suspScale[3] or 1.0,
        state.avgSuspScale or 1.0,

        state.impulse[0] or 0.0,
        state.impulse[1] or 0.0,
        state.impulse[2] or 0.0,
        state.impulse[3] or 0.0,

        state.contactDrop[0] or 0.0,
        state.contactDrop[1] or 0.0,
        state.contactDrop[2] or 0.0,
        state.contactDrop[3] or 0.0,

        state.contactLinked and "OK" or "NIL",
        state.hopLinked and "OK" or "NIL",
        state.memoryLinked and "OK" or "NIL",
        state.loadLinked and "OK" or "NIL",
        state.bodyLinked and "OK" or "NIL",
        state.damageLinked and "OK" or "NIL",
        state.brakeLinked and "OK" or "NIL",
        state.thermalLinked and "OK" or "NIL",
        state.chassisLinked and "OK" or "NIL",
        state.weightLinked and "OK" or "NIL",
        state.roadInputLinked and "OK" or "NIL",
        state.loadPathLinked and "OK" or "NIL"
    )
end

return M
