---@diagnostic disable: undefined-global

--============================================================
-- damper_model.lua
-- ACNextGen V1.1.5 Stable
-- High / Low Speed Damper Model
--============================================================

local M = {}

M.params = {
    lowSpeedBump = 2500.0,
    lowSpeedRebound = 3500.0,
    highSpeedBump = 7000.0,
    highSpeedRebound = 9000.0,
    transition = 0.15,
    maxForce = 12000.0,

    contactInputGain = 0.22,
    contactScaleGain = 0.18,
    roadImpulseGain = 0.30,
    contactDropGain = 0.20,

    forceTau = 0.035,
    inputTau = 0.050,

    minCoeffScale = 0.70,
    maxCoeffScale = 1.35,
    minTravel = -1.0,
    maxTravel = 1.0,

    bodyFlexCoeffLoss = 0.10,
    bodyTorsionCoeffLoss = 0.08,
    bodyBendingCoeffLoss = 0.06,
    bodyStiffCoeffGain = 0.06,
    bodyFlexTauGain = 0.45,
    bodyStiffTauGain = 0.12,
    bodyImpulseSoftening = 0.16,

    tireHopCoeffGain = 0.08,
    damageCoeffLoss = 0.10,
    brakeHeatCoeffLoss = 0.05,

    preserveInputCoeffGain = 0.10,
    chassisFlexCoeffGain = 0.06,
    damageRuntimeCoeffLoss = 0.08,
    weightBiasCoeffGain = 0.05,

    maxPreserveCoeffAdd = 0.20,

    tireRoadCoeffGain = 0.14,
    tireRoadImpulseCoeffGain = 0.10,
    tireCombinedSlipCoeffGain = 0.04,
    tireSlipEnergyCoeffLoss = 0.06,

    suspIntegratedForceGain = 0.055,

    tireForceReference = 2500.0,
    suspForceReference = 1200.0,

    tireRoadInputTau = 0.040,

    minTireRoadInput = 0.0,
    maxTireRoadInput = 1.50,

    noCarReturnTau = 0.180,
    minDt = 0.00005,
    maxDt = 0.050,

    debugStoreInterval = 0.25,
}

local WHEEL_NAMES = { [0] = "FL", [1] = "FR", [2] = "RL", [3] = "RR" }

local state = {
    force = { [0]=0,[1]=0,[2]=0,[3]=0 },
    rawForce = { [0]=0,[1]=0,[2]=0,[3]=0 },
    velocity = { [0]=0,[1]=0,[2]=0,[3]=0 },
    prevTravel = { [0]=0,[1]=0,[2]=0,[3]=0 },
    travel = { [0]=0,[1]=0,[2]=0,[3]=0 },
    initialized = { [0]=false,[1]=false,[2]=false,[3]=false },

    contactInput = { [0]=0,[1]=0,[2]=0,[3]=0 },
    contactScale = { [0]=1,[1]=1,[2]=1,[3]=1 },
    roadImpulse = { [0]=0,[1]=0,[2]=0,[3]=0 },
    contactDrop = { [0]=0,[1]=0,[2]=0,[3]=0 },
    coeffScale = { [0]=1,[1]=1,[2]=1,[3]=1 },

    bodyRigidityScale = { [0]=1,[1]=1,[2]=1,[3]=1 },
    bodyForceTau = { [0]=0.035,[1]=0.035,[2]=0.035,[3]=0.035 },

    hop = { [0]=0,[1]=0,[2]=0,[3]=0 },
    damage = { [0]=0,[1]=0,[2]=0,[3]=0 },
    brakeTemp = { [0]=25,[1]=25,[2]=25,[3]=25 },

    preserveInputAdd = 0.0,
    chassisFlexEnergy = 0.0,
    damageRuntimeAdd = 0.0,
    weightBias = { [0]=1,[1]=1,[2]=1,[3]=1 },
    preserveCoeffAdd = { [0]=0,[1]=0,[2]=0,[3]=0 },

    tireRoadInput = { [0]=0,[1]=0,[2]=0,[3]=0 },
    tireRoadImpulse = { [0]=0,[1]=0,[2]=0,[3]=0 },
    tireCombinedSlip = { [0]=0,[1]=0,[2]=0,[3]=0 },
    tireSlipEnergy = { [0]=0,[1]=0,[2]=0,[3]=0 },
    tireForceMag = { [0]=0,[1]=0,[2]=0,[3]=0 },

    suspIntegratedForce = { [0]=0,[1]=0,[2]=0,[3]=0 },
    suspIntegratedScale = { [0]=1,[1]=1,[2]=1,[3]=1 },
    suspForceFeed = { [0]=0,[1]=0,[2]=0,[3]=0 },

    hystImpact = { [0]=0,[1]=0,[2]=0,[3]=0 },
    hystRoadMemory = { [0]=0,[1]=0,[2]=0,[3]=0 },
    hystVerticalPulse = { [0]=0,[1]=0,[2]=0,[3]=0 },

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyBendingFactor = 1.0,
    bodyDampingFactor = 1.0,
    bodyRigidityLinked = false,

    mode = { [0]="INIT",[1]="INIT",[2]="INIT",[3]="INIT" },

    avgForce = 0.0,
    avgAbsForce = 0.0,
    avgContactInput = 0.0,
    avgCoeffScale = 1.0,
    avgTireRoadInput = 0.0,
    avgTireSlipEnergy = 0.0,
    avgSuspForceFeed = 0.0,
    avgVelocity = 0.0,
    maxAbsForce = 0.0,

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    contactLinked = false,
    bodyLinked = false,
    tireLinked = false,
    damageLinked = false,
    brakeLinked = false,
    preserveLinked = false,
    chassisLinked = false,
    damageRuntimeLinked = false,
    weightLinked = false,
    tireForceLinked = false,
    suspensionLinked = false,
    hystLinked = false,
    travelLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

local function safeNumber(v, d)
    local n = tonumber(v)
    if n == nil or n ~= n or n == math.huge or n == -math.huge then
        return d or 0.0
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
    v = safeNumber(v, 0.0)
    return v < 0.0 and -v or v
end

local function safeLoadRaw(k)
    if not ac or not ac.load then return nil end
    local ok, v = pcall(function() return ac.load(k) end)
    if not ok then return nil end
    return v
end

local function safeLoad(k, d)
    local v = safeLoadRaw(k)
    if v == nil then return d or 0.0 end
    return safeNumber(v, d or 0.0)
end

local function safeLoadAlt(d, ...)
    local keys = { ... }
    for i = 1, #keys do
        local v = safeLoadRaw(keys[i])
        if v ~= nil then
            return safeNumber(v, d or 0.0), keys[i]
        end
    end
    return d or 0.0, nil
end

local function safeStore(k, v)
    if not ac or not ac.store then return end
    pcall(function() ac.store(k, v) end)
end

local function safeField(o, f, d)
    if not o then return d end
    local ok, v = pcall(function() return o[f] end)
    if not ok or v == nil then return d end
    return v
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

local function getCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function() return ac.getCar(0) end)
    if not ok then return nil end
    return car
end

local function getWheel(car, i)
    if not car then return nil end
    local wheels = safeField(car, "wheels", nil)
    if not wheels then return nil end

    local ok0, w0 = pcall(function() return wheels[i] end)
    if ok0 and w0 then return w0 end

    local ok1, w1 = pcall(function() return wheels[i + 1] end)
    if ok1 and w1 then return w1 end

    return nil
end

local function resetLinks()
    state.contactLinked = false
    state.bodyLinked = false
    state.tireLinked = false
    state.damageLinked = false
    state.brakeLinked = false
    state.preserveLinked = false
    state.chassisLinked = false
    state.damageRuntimeLinked = false
    state.weightLinked = false
    state.tireForceLinked = false
    state.suspensionLinked = false
    state.hystLinked = false
    state.travelLinked = false
end

local function readBody()
    local bodyRaw = safeLoadRaw("ngp_body_rigidity")
    local flexRaw = safeLoadRaw("ngp_body_flex_factor")
    local torsionRaw = safeLoadRaw("ngp_body_torsion_factor")
    local dampRaw = safeLoadRaw("ngp_body_damping_factor")
    local updateRaw = safeLoadRaw("ngp_body_rigidity_update_count")

    state.bodyLinked = bodyRaw ~= nil or flexRaw ~= nil or torsionRaw ~= nil or dampRaw ~= nil or updateRaw ~= nil
    state.bodyRigidityLinked = safeNumber(updateRaw, 0.0) > 0.0 or state.bodyLinked

    state.bodyRigidity = clamp(safeNumber(bodyRaw, 1.0), 0.20, 1.35)
    state.bodyFlexFactor = clamp(safeNumber(flexRaw, 1.0), 0.25, 1.80)
    state.bodyTorsionFactor = clamp(safeNumber(torsionRaw, 1.0), 0.35, 2.00)
    state.bodyBendingFactor = clamp(safeLoad("ngp_body_bending_factor", 1.0), 0.35, 2.00)
    state.bodyDampingFactor = clamp(safeNumber(dampRaw, 1.0), 0.50, 1.80)
end

local function readPreserveInputs()
    local preserve = safeLoadRaw("ngp_sci_preserve_input_add")
        or safeLoadRaw("ngp_susp_contact_preserve_add")
        or safeLoadRaw("ngp_contact_preserve_add")

    state.preserveLinked = preserve ~= nil
    state.preserveInputAdd = clamp(safeNumber(preserve, 0.0), 0.0, 0.25)

    local flex = safeLoadRaw("ngp_chassis_flex_energy")
        or safeLoadRaw("ngp_flex_energy")
        or safeLoadRaw("ngp_chassis_energy")

    state.chassisLinked = flex ~= nil
    state.chassisFlexEnergy = clamp(safeNumber(flex, 0.0), 0.0, 1.0)

    local damage = safeLoadRaw("ngp_damage_runtime_add")
        or safeLoadRaw("ngp_condition_total")
        or safeLoadRaw("ngp_vehicle_condition_total")

    state.damageRuntimeLinked = damage ~= nil
    state.damageRuntimeAdd = clamp(safeNumber(damage, 0.0), 0.0, 1.0)

    local fb = safeLoadRaw("ngp_weight_front_bias")
        or safeLoadRaw("ngp_load_front_ratio")
        or safeLoadRaw("ngp_load_front")
    local rb = safeLoadRaw("ngp_weight_rear_bias")
        or safeLoadRaw("ngp_load_rear_ratio")
        or safeLoadRaw("ngp_load_rear")
    local lb = safeLoadRaw("ngp_weight_left_bias")
        or safeLoadRaw("ngp_load_left_ratio")
        or safeLoadRaw("ngp_load_left")
    local rr = safeLoadRaw("ngp_weight_right_bias")
        or safeLoadRaw("ngp_load_right_ratio")
        or safeLoadRaw("ngp_load_right")

    state.weightLinked = fb ~= nil or rb ~= nil or lb ~= nil or rr ~= nil

    local f = clamp(safeNumber(fb, 0.5), 0.25, 0.75)
    local r = clamp(safeNumber(rb, 0.5), 0.25, 0.75)
    local l = clamp(safeNumber(lb, 0.5), 0.25, 0.75)
    local right = clamp(safeNumber(rr, 0.5), 0.25, 0.75)

    state.weightBias[0] = clamp(f * l * 2.0, 0.50, 1.50)
    state.weightBias[1] = clamp(f * right * 2.0, 0.50, 1.50)
    state.weightBias[2] = clamp(r * l * 2.0, 0.50, 1.50)
    state.weightBias[3] = clamp(r * right * 2.0, 0.50, 1.50)
end

local function applyBodyScale(i, coeff, impulse)
    if not state.bodyRigidityLinked then
        state.bodyRigidityScale[i] = 1.0
        state.bodyForceTau[i] = M.params.forceTau
        return coeff, M.params.forceTau
    end

    local softness = clamp(state.bodyFlexFactor - 1.0, 0.0, 0.80)
    local stiffness = clamp(state.bodyRigidity - 1.0, 0.0, 0.35)
    local torsionSoft = clamp(state.bodyTorsionFactor - 1.0, 0.0, 1.0)
    local bendingSoft = clamp(state.bodyBendingFactor - 1.0, 0.0, 1.0)
    local damping = math.max(state.bodyDampingFactor, 0.50)

    local scale =
        1.0
        - softness * M.params.bodyFlexCoeffLoss
        - torsionSoft * M.params.bodyTorsionCoeffLoss
        - bendingSoft * M.params.bodyBendingCoeffLoss
        - clamp(impulse, 0.0, 1.0) * softness * M.params.bodyImpulseSoftening
        + stiffness * M.params.bodyStiffCoeffGain

    scale = clamp(scale, 0.84, 1.12)

    local tau =
        M.params.forceTau
        * (1.0 + softness * M.params.bodyFlexTauGain)
        / (1.0 + stiffness * M.params.bodyStiffTauGain)
        / damping

    state.bodyRigidityScale[i] = scale
    state.bodyForceTau[i] = clamp(tau, 0.020, 0.080)

    return coeff * scale, state.bodyForceTau[i]
end

local function readTravel(wheel, i)
    local external, key = safeLoadAlt(0.0,
        "ngp_susp_damper_travel_" .. i,
        "ngp_susp_travel_" .. i,
        "ngp_wheel_travel_" .. i,
        "ngp_suspension_travel_" .. i
    )

    if key then
        state.travelLinked = true
        return clamp(external, M.params.minTravel, M.params.maxTravel)
    end

    if wheel then
        local keys = {
            "damperTravel",
            "suspensionTravel",
            "suspensionTravelM",
            "suspensionLength",
            "travel",
            "suspensionPosition"
        }

        for n = 1, #keys do
            local v = safeField(wheel, keys[n], nil)
            if v ~= nil then
                return clamp(safeNumber(v, state.prevTravel[i] or 0.0), M.params.minTravel, M.params.maxTravel)
            end
        end
    end

    return state.prevTravel[i] or 0.0
end

local function readContactInputs(i)
    local ci = safeLoadRaw("ngp_susp_contact_input_" .. i) or safeLoadRaw("ngp_sci_input_" .. i)
    local cs = safeLoadRaw("ngp_susp_contact_scale_" .. i) or safeLoadRaw("ngp_sci_scale_" .. i)
    local ri = safeLoadRaw("ngp_susp_road_impulse_" .. i) or safeLoadRaw("ngp_sci_impulse_" .. i)
    local cd = safeLoadRaw("ngp_susp_contact_drop_" .. i) or safeLoadRaw("ngp_sci_drop_" .. i)

    local cq = safeLoadRaw("ngp_contact_quality_" .. i)
        or safeLoadRaw("ngp_tire_contact_quality_" .. i)
        or safeLoadRaw("ngp_tcr_quality_" .. i)
    local trust = safeLoadRaw("ngp_contact_trust_" .. i)
    local loss = safeLoadRaw("ngp_contact_loss_" .. i)
        or safeLoadRaw("ngp_tire_contact_loss_" .. i)
        or safeLoadRaw("ngp_tcr_contact_loss_" .. i)

    if ci ~= nil or cs ~= nil or ri ~= nil or cd ~= nil or cq ~= nil or trust ~= nil or loss ~= nil then
        state.contactLinked = true
    end

    local contactLoss = clamp(safeNumber(loss, 1.0 - math.min(safeNumber(cq, 1.0), safeNumber(trust, safeNumber(cq, 1.0)))), 0.0, 1.0)
    local contactQuality = clamp(safeNumber(cq, 1.0), 0.0, 1.2)

    state.contactInput[i] = clamp(math.max(safeNumber(ci, 0.0), contactLoss * 0.60), 0.0, 1.5)
    state.contactScale[i] = clamp(safeNumber(cs, 0.80 + contactQuality * 0.20), 0.50, 1.70)
    state.roadImpulse[i] = clamp(safeNumber(ri, 0.0), 0.0, 1.5)
    state.contactDrop[i] = clamp(math.max(safeNumber(cd, 0.0), contactLoss), 0.0, 1.0)
end

local function readTireForceInputs(i, dt)
    local tfRoad =
        safeLoadRaw("ngp_tire_force_road_input_" .. i)
        or safeLoadRaw("ngp_tf_road_input_" .. i)
        or safeLoadRaw("ngp_road_input_" .. i)

    local tfCombined =
        safeLoadRaw("ngp_tire_force_combined_" .. i)
        or safeLoadRaw("ngp_tf_combined_" .. i)

    local tfLat =
        safeLoadRaw("ngp_tire_force_lat_" .. i)
        or safeLoadRaw("ngp_tf_lat_" .. i)

    local tfLong =
        safeLoadRaw("ngp_tire_force_long_" .. i)
        or safeLoadRaw("ngp_tf_long_" .. i)

    local combinedSlip =
        safeLoadRaw("ngp_tire_force_combined_slip_" .. i)
        or safeLoadRaw("ngp_tdyn_combined_slip_" .. i)
        or safeLoadRaw("ngp_tire_dynamics_combined_slip_" .. i)
        or safeLoadRaw("ngp_contact_combined_slip_" .. i)

    local slipEnergy =
        safeLoadRaw("ngp_tire_force_slip_energy_" .. i)
        or safeLoadRaw("ngp_tdyn_slip_energy_" .. i)
        or safeLoadRaw("ngp_tire_dynamics_slip_energy_" .. i)

    if tfRoad ~= nil or tfCombined ~= nil or tfLat ~= nil or tfLong ~= nil or combinedSlip ~= nil or slipEnergy ~= nil then
        state.tireForceLinked = true
        state.tireLinked = true
    end

    local forceMag = math.max(
        abs(tfCombined),
        math.sqrt((safeNumber(tfLat, 0.0) * safeNumber(tfLat, 0.0)) + (safeNumber(tfLong, 0.0) * safeNumber(tfLong, 0.0)))
    )

    local tireRoadTarget = math.max(safeNumber(tfRoad, 0.0), forceMag / math.max(M.params.tireForceReference, 1.0))
    tireRoadTarget = clamp(tireRoadTarget, M.params.minTireRoadInput, M.params.maxTireRoadInput)

    state.tireRoadInput[i] = lowPass(
        state.tireRoadInput[i] or 0.0,
        tireRoadTarget,
        M.params.tireRoadInputTau,
        dt
    )

    state.tireRoadImpulse[i] = clamp(math.max(state.tireRoadInput[i] or 0.0, state.roadImpulse[i] or 0.0), 0.0, 1.5)
    state.tireCombinedSlip[i] = clamp(safeNumber(combinedSlip, 0.0), 0.0, 3.0)
    state.tireSlipEnergy[i] = clamp(safeNumber(slipEnergy, 0.0), 0.0, 1.5)
    state.tireForceMag[i] = forceMag
end

local function readSuspensionInputs(i)
    local suspForce =
        safeLoadRaw("ngp_susp_int_force_" .. i)
        or safeLoadRaw("ngp_susp_integrated_force_" .. i)
        or safeLoadRaw("ngp_susp_" .. i)

    local suspScale =
        safeLoadRaw("ngp_susp_int_scale_" .. i)
        or safeLoadRaw("ngp_susp_integrated_scale_" .. i)

    if suspForce ~= nil or suspScale ~= nil then
        state.suspensionLinked = true
    end

    state.suspIntegratedForce[i] = safeNumber(suspForce, 0.0)
    state.suspIntegratedScale[i] = clamp(safeNumber(suspScale, 1.0), 0.50, 1.70)

    state.suspForceFeed[i] = clamp(
        (state.suspIntegratedForce[i] or 0.0) / math.max(M.params.suspForceReference, 1.0),
        -1.5,
        1.5
    )
end

local function readOtherWheelInputs(i)
    local hop = safeLoadRaw("ngp_tire_hop_energy_" .. i) or safeLoadRaw("ngp_tire_hop_" .. i)
    if hop ~= nil then state.tireLinked = true end
    state.hop[i] = clamp(safeNumber(hop, 0.0), 0.0, 1.0)

    local damage = safeLoadRaw("ngp_damage_wheel_" .. i)
    if damage ~= nil then state.damageLinked = true end
    state.damage[i] = clamp(safeNumber(damage, 0.0), 0.0, 1.0)

    local temp = safeLoadRaw("ngp_brake_temp_" .. i) or safeLoadRaw("ngp_brake_disc_temp_" .. i)
    if temp ~= nil then state.brakeLinked = true end
    state.brakeTemp[i] = safeNumber(temp, 25.0)

    local hi = safeLoadRaw("ngp_damper_hyst_impact_" .. i)
    local hr = safeLoadRaw("ngp_damper_road_memory_" .. i)
    local hp = safeLoadRaw("ngp_damper_vertical_pulse_" .. i)

    if hi ~= nil or hr ~= nil or hp ~= nil then
        state.hystLinked = true
    end

    state.hystImpact[i] = clamp(safeNumber(hi, 0.0), 0.0, 1.0)
    state.hystRoadMemory[i] = clamp(safeNumber(hr, 0.0), 0.0, 1.0)
    state.hystVerticalPulse[i] = clamp(safeNumber(hp, 0.0), 0.0, 1.0)
end

local function readRootInputs(i, dt)
    readContactInputs(i)
    readTireForceInputs(i, dt)
    readSuspensionInputs(i)
    readOtherWheelInputs(i)
end

local function updateWheel(i, car, dt)
    local wheel = getWheel(car, i)
    if not wheel then
        state.velocity[i] = lowPass(state.velocity[i], 0.0, M.params.noCarReturnTau, dt)
        state.force[i] = lowPass(state.force[i], 0.0, M.params.noCarReturnTau, dt)
        state.rawForce[i] = 0.0
        state.mode[i] = "NO WHEEL"
        return false
    end

    local travel = readTravel(wheel, i)
    local velocity = 0.0

    if state.initialized[i] then
        velocity = (travel - (state.prevTravel[i] or travel)) / math.max(dt, M.params.minDt)
    else
        velocity = 0.0
        state.initialized[i] = true
    end

    state.prevTravel[i] = travel
    state.travel[i] = travel
    state.velocity[i] = velocity

    readRootInputs(i, dt)

    local absVel = abs(velocity)
    local t = clamp(absVel / math.max(M.params.transition, 0.001), 0.0, 1.0)

    local coeff
    if velocity >= 0.0 then
        coeff = M.params.lowSpeedBump * (1.0 - t) + M.params.highSpeedBump * t
        state.mode[i] = t > 0.5 and "HIGH BUMP" or "LOW BUMP"
    else
        coeff = M.params.lowSpeedRebound * (1.0 - t) + M.params.highSpeedRebound * t
        state.mode[i] = t > 0.5 and "HIGH REBOUND" or "LOW REBOUND"
    end

    local coeffScale =
        1.0
        + state.contactInput[i] * M.params.contactInputGain
        + (state.contactScale[i] - 1.0) * M.params.contactScaleGain
        + state.roadImpulse[i] * M.params.roadImpulseGain
        + state.tireRoadInput[i] * M.params.tireRoadCoeffGain
        + state.tireRoadImpulse[i] * M.params.tireRoadImpulseCoeffGain
        + state.tireCombinedSlip[i] * M.params.tireCombinedSlipCoeffGain
        - state.tireSlipEnergy[i] * M.params.tireSlipEnergyCoeffLoss
        + math.max(state.suspIntegratedScale[i] - 1.0, 0.0) * 0.08
        + state.hop[i] * M.params.tireHopCoeffGain
        + state.hystImpact[i] * 0.04
        + state.hystVerticalPulse[i] * 0.05
        + state.hystRoadMemory[i] * 0.03
        - state.contactDrop[i] * M.params.contactDropGain
        - state.damage[i] * M.params.damageCoeffLoss
        - math.max(state.brakeTemp[i] - 500.0, 0.0) / 400.0 * M.params.brakeHeatCoeffLoss
        + state.preserveInputAdd * M.params.preserveInputCoeffGain
        + state.chassisFlexEnergy * M.params.chassisFlexCoeffGain
        + math.max(state.weightBias[i] - 1.0, 0.0) * M.params.weightBiasCoeffGain
        - state.damageRuntimeAdd * M.params.damageRuntimeCoeffLoss

    state.preserveCoeffAdd[i] = clamp(
        state.preserveInputAdd * M.params.preserveInputCoeffGain
        + state.chassisFlexEnergy * M.params.chassisFlexCoeffGain
        + math.max(state.weightBias[i] - 1.0, 0.0) * M.params.weightBiasCoeffGain,
        0.0,
        M.params.maxPreserveCoeffAdd
    )

    coeffScale = clamp(coeffScale, M.params.minCoeffScale, M.params.maxCoeffScale)

    local bodyCoeff, tau = applyBodyScale(i, coeff * coeffScale, math.max(state.roadImpulse[i], state.hystVerticalPulse[i]))

    state.coeffScale[i] = clamp(coeffScale * state.bodyRigidityScale[i], M.params.minCoeffScale * 0.80, M.params.maxCoeffScale * 1.20)

    local rawForce =
        bodyCoeff * velocity
        + state.suspIntegratedForce[i] * M.params.suspIntegratedForceGain

    rawForce = clamp(rawForce, -M.params.maxForce, M.params.maxForce)

    state.rawForce[i] = rawForce
    state.force[i] = lowPass(state.force[i], rawForce, tau, dt)

    return true
end

local function exportWheel(i)
    safeStore("ngp_damper_" .. i, state.force[i] or 0.0)
    safeStore("ngp_damper_force_" .. i, state.force[i] or 0.0)
    safeStore("ngp_damper_raw_" .. i, state.rawForce[i] or 0.0)
    safeStore("ngp_damper_velocity_" .. i, state.velocity[i] or 0.0)
    safeStore("ngp_damper_travel_" .. i, state.travel[i] or 0.0)
    safeStore("ngp_damper_coeff_scale_" .. i, state.coeffScale[i] or 1.0)
    safeStore("ngp_damper_body_scale_" .. i, state.bodyRigidityScale[i] or 1.0)
    safeStore("ngp_damper_preserve_coeff_add_" .. i, state.preserveCoeffAdd[i] or 0.0)

    safeStore("ngp_damper_tire_road_input_" .. i, state.tireRoadInput[i] or 0.0)
    safeStore("ngp_damper_susp_force_feed_" .. i, state.suspForceFeed[i] or 0.0)

    safeStore("ngp_susp_external_damper_" .. i, state.force[i] or 0.0)
    safeStore("ngp_damper_model_force_" .. i, state.force[i] or 0.0)
    safeStore("ngp_damper_model_velocity_" .. i, state.velocity[i] or 0.0)

    if not state.debugStoreNow then return end

    safeStore("ngp_damper_contact_input_" .. i, state.contactInput[i] or 0.0)
    safeStore("ngp_damper_road_impulse_" .. i, state.roadImpulse[i] or 0.0)
    safeStore("ngp_damper_contact_drop_" .. i, state.contactDrop[i] or 0.0)

    safeStore("ngp_damper_tire_road_impulse_" .. i, state.tireRoadImpulse[i] or 0.0)
    safeStore("ngp_damper_tire_combined_slip_" .. i, state.tireCombinedSlip[i] or 0.0)
    safeStore("ngp_damper_tire_slip_energy_" .. i, state.tireSlipEnergy[i] or 0.0)
    safeStore("ngp_damper_tire_force_mag_" .. i, state.tireForceMag[i] or 0.0)
    safeStore("ngp_damper_susp_integrated_force_" .. i, state.suspIntegratedForce[i] or 0.0)
    safeStore("ngp_damper_susp_integrated_scale_" .. i, state.suspIntegratedScale[i] or 1.0)

    safeStore("ngp_damper_hyst_impact_input_" .. i, state.hystImpact[i] or 0.0)
    safeStore("ngp_damper_hyst_pulse_input_" .. i, state.hystVerticalPulse[i] or 0.0)
    safeStore("ngp_damper_mode_" .. i, state.mode[i] or "UNKNOWN")
end

local function exportGlobal()
    safeStore("ngp_damper_status", state.status or "UNKNOWN")
    safeStore("ngp_damper_update_count", state.updateCount or 0)
    safeStore("ngp_damper_wheels_valid", state.wheelsValid and 1 or 0)

    safeStore("ngp_damper_avg_force", state.avgForce or 0.0)
    safeStore("ngp_damper_avg_abs_force", state.avgAbsForce or 0.0)
    safeStore("ngp_damper_avg_contact_input", state.avgContactInput or 0.0)
    safeStore("ngp_damper_avg_coeff_scale", state.avgCoeffScale or 1.0)
    safeStore("ngp_damper_avg_tire_road_input", state.avgTireRoadInput or 0.0)
    safeStore("ngp_damper_avg_tire_slip_energy", state.avgTireSlipEnergy or 0.0)
    safeStore("ngp_damper_avg_susp_force_feed", state.avgSuspForceFeed or 0.0)
    safeStore("ngp_damper_avg_velocity", state.avgVelocity or 0.0)
    safeStore("ngp_damper_max_abs_force", state.maxAbsForce or 0.0)

    if not state.debugStoreNow then return end

    safeStore("ngp_damper_contact_linked", state.contactLinked and 1 or 0)
    safeStore("ngp_damper_body_linked", state.bodyLinked and 1 or 0)
    safeStore("ngp_damper_tire_linked", state.tireLinked and 1 or 0)
    safeStore("ngp_damper_damage_linked", state.damageLinked and 1 or 0)
    safeStore("ngp_damper_brake_linked", state.brakeLinked and 1 or 0)
    safeStore("ngp_damper_preserve_linked", state.preserveLinked and 1 or 0)
    safeStore("ngp_damper_chassis_linked", state.chassisLinked and 1 or 0)
    safeStore("ngp_damper_damage_runtime_linked", state.damageRuntimeLinked and 1 or 0)
    safeStore("ngp_damper_weight_linked", state.weightLinked and 1 or 0)
    safeStore("ngp_damper_tire_force_linked", state.tireForceLinked and 1 or 0)
    safeStore("ngp_damper_suspension_linked", state.suspensionLinked and 1 or 0)
    safeStore("ngp_damper_hyst_linked", state.hystLinked and 1 or 0)
    safeStore("ngp_damper_travel_linked", state.travelLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i)
    end
    exportGlobal()
end

local function decayAll(dt)
    for i = 0, 3 do
        state.force[i] = lowPass(state.force[i], 0.0, M.params.noCarReturnTau, dt)
        state.rawForce[i] = 0.0
        state.velocity[i] = lowPass(state.velocity[i], 0.0, M.params.noCarReturnTau, dt)
        state.contactInput[i] = lowPass(state.contactInput[i], 0.0, M.params.noCarReturnTau, dt)
        state.roadImpulse[i] = lowPass(state.roadImpulse[i], 0.0, M.params.noCarReturnTau, dt)
        state.contactDrop[i] = lowPass(state.contactDrop[i], 0.0, M.params.noCarReturnTau, dt)
        state.coeffScale[i] = lowPass(state.coeffScale[i], 1.0, M.params.noCarReturnTau, dt)
        state.mode[i] = "NO CAR"
        exportWheel(i)
    end

    state.avgForce = (state.force[0] + state.force[1] + state.force[2] + state.force[3]) * 0.25
    state.avgAbsForce = (abs(state.force[0]) + abs(state.force[1]) + abs(state.force[2]) + abs(state.force[3])) * 0.25
    state.avgContactInput = 0.0
    state.avgCoeffScale = (state.coeffScale[0] + state.coeffScale[1] + state.coeffScale[2] + state.coeffScale[3]) * 0.25
    state.avgTireRoadInput = 0.0
    state.avgTireSlipEnergy = 0.0
    state.avgSuspForceFeed = 0.0
    state.avgVelocity = 0.0
    state.maxAbsForce = 0.0
end

function M.init()
    state.status = "INIT"
    state.wheelsValid = false
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1
    dt = clamp(safeNumber(dt, 0.0), M.params.minDt, M.params.maxDt)

    updateDebugGate(dt)

    car = car or getCar()
    if not car then
        state.status = "NO CAR"
        state.wheelsValid = false
        resetLinks()
        decayAll(dt)
        exportGlobal()
        return
    end

    local wheels = safeField(car, "wheels", nil)
    if not wheels then
        state.status = "NO WHEELS"
        state.wheelsValid = false
        resetLinks()
        decayAll(dt)
        exportGlobal()
        return
    end

    resetLinks()
    readBody()
    readPreserveInputs()

    local validCount = 0
    local sumForce = 0.0
    local sumAbs = 0.0
    local sumInput = 0.0
    local sumCoeff = 0.0
    local sumTireRoad = 0.0
    local sumSlipEnergy = 0.0
    local sumSuspFeed = 0.0
    local sumVelocity = 0.0
    local maxAbsForce = 0.0

    for i = 0, 3 do
        local ok = updateWheel(i, car, dt)
        if ok then validCount = validCount + 1 end

        sumForce = sumForce + (state.force[i] or 0.0)
        sumAbs = sumAbs + abs(state.force[i])
        sumInput = sumInput + (state.contactInput[i] or 0.0)
        sumCoeff = sumCoeff + (state.coeffScale[i] or 1.0)
        sumTireRoad = sumTireRoad + (state.tireRoadInput[i] or 0.0)
        sumSlipEnergy = sumSlipEnergy + (state.tireSlipEnergy[i] or 0.0)
        sumSuspFeed = sumSuspFeed + abs(state.suspForceFeed[i] or 0.0)
        sumVelocity = sumVelocity + (state.velocity[i] or 0.0)
        maxAbsForce = math.max(maxAbsForce, abs(state.force[i]))

        exportWheel(i)
    end

    state.wheelsValid = validCount > 0
    state.avgForce = sumForce * 0.25
    state.avgAbsForce = sumAbs * 0.25
    state.avgContactInput = sumInput * 0.25
    state.avgCoeffScale = sumCoeff * 0.25
    state.avgTireRoadInput = sumTireRoad * 0.25
    state.avgTireSlipEnergy = sumSlipEnergy * 0.25
    state.avgSuspForceFeed = sumSuspFeed * 0.25
    state.avgVelocity = sumVelocity * 0.25
    state.maxAbsForce = maxAbsForce

    state.status = state.wheelsValid and "RUNNING" or "NO WHEEL DATA"
    exportGlobal()
end

function M.getForce(index)
    return state.force[index] or 0.0
end

function M.getRawForce(index)
    return state.rawForce[index] or 0.0
end

function M.getVelocity(index)
    return state.velocity[index] or 0.0
end

function M.getCoeffScale(index)
    return state.coeffScale[index] or 1.0
end

function M.getMode(index)
    return state.mode[index] or "UNKNOWN"
end

function M.getState(index)
    if index == nil then
        return state
    end

    return {
        force = state.force[index] or 0.0,
        rawForce = state.rawForce[index] or 0.0,
        velocity = state.velocity[index] or 0.0,
        travel = state.travel[index] or 0.0,
        coeffScale = state.coeffScale[index] or 1.0,
        mode = state.mode[index] or "UNKNOWN",
    }
end

function M.debugStr(index)
    if index ~= nil then
        local i = math.floor(clamp(index, 0, 3))
        return string.format(
            "%s Force %.0f Raw %.0f Vel %+.4f Travel %.4f\nCoeff %.2f Body %.2f Mode %s\nContact %.2f Road %.2f Drop %.2f TireRoad %.2f SuspFeed %.2f",
            WHEEL_NAMES[i] or tostring(i),
            state.force[i] or 0.0,
            state.rawForce[i] or 0.0,
            state.velocity[i] or 0.0,
            state.travel[i] or 0.0,
            state.coeffScale[i] or 1.0,
            state.bodyRigidityScale[i] or 1.0,
            tostring(state.mode[i] or "UNKNOWN"),
            state.contactInput[i] or 0.0,
            state.roadImpulse[i] or 0.0,
            state.contactDrop[i] or 0.0,
            state.tireRoadInput[i] or 0.0,
            state.suspForceFeed[i] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "Force %.0f %.0f %.0f %.0f AvgAbs %.0f Max %.0f\n" ..
        "Coeff %.2f %.2f %.2f %.2f Preserve %.3f\n" ..
        "TireRoad %.2f %.2f %.2f %.2f SlipE %.2f\n" ..
        "SuspFeed %.2f %.2f %.2f %.2f\n" ..
        "Links Contact:%s Body:%s Tire:%s Susp:%s Damage:%s Brake:%s Preserve:%s Chassis:%s Weight:%s Hyst:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",

        state.force[0] or 0.0,
        state.force[1] or 0.0,
        state.force[2] or 0.0,
        state.force[3] or 0.0,
        state.avgAbsForce or 0.0,
        state.maxAbsForce or 0.0,

        state.coeffScale[0] or 1.0,
        state.coeffScale[1] or 1.0,
        state.coeffScale[2] or 1.0,
        state.coeffScale[3] or 1.0,
        state.preserveInputAdd or 0.0,

        state.tireRoadInput[0] or 0.0,
        state.tireRoadInput[1] or 0.0,
        state.tireRoadInput[2] or 0.0,
        state.tireRoadInput[3] or 0.0,
        state.avgTireSlipEnergy or 0.0,

        state.suspForceFeed[0] or 0.0,
        state.suspForceFeed[1] or 0.0,
        state.suspForceFeed[2] or 0.0,
        state.suspForceFeed[3] or 0.0,

        state.contactLinked and "OK" or "NIL",
        state.bodyLinked and "OK" or "NIL",
        state.tireForceLinked and "OK" or "NIL",
        state.suspensionLinked and "OK" or "NIL",
        state.damageLinked and "OK" or "NIL",
        state.brakeLinked and "OK" or "NIL",
        state.preserveLinked and "OK" or "NIL",
        state.chassisLinked and "OK" or "NIL",
        state.weightLinked and "OK" or "NIL",
        state.hystLinked and "OK" or "NIL"
    )
end

return M
