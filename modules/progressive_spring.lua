---@diagnostic disable: undefined-global

--============================================================
-- progressive_spring.lua
-- ACNextGen V1.1.5 Stable
-- Nonlinear Suspension Spring / Rootlinked
--============================================================

local M = {}

local WHEEL_NAMES = { [0] = "FL", [1] = "FR", [2] = "RL", [3] = "RR" }

M.params = {
    baseRate = 30000.0,
    progressiveRate = 90000.0,
    startTravel = 0.35,
    maxForce = 25000.0,

    tau = 0.050,
    inputTau = 0.060,
    forceTau = 0.045,

    contactRateGain = 0.18,
    contactScaleGain = 0.16,
    roadImpulseRateGain = 0.24,
    contactDropRateLoss = 0.20,

    travelInputGain = 0.035,
    impulseTravelGain = 0.020,
    dropTravelLoss = 0.025,

    minRateScale = 0.70,
    maxRateScale = 1.35,
    minTravel = -1.0,
    maxTravel = 1.0,

    bodyFlexRateLoss = 0.10,
    bodyTorsionRateLoss = 0.07,
    bodyBendingRateLoss = 0.08,
    bodyStiffRateGain = 0.05,
    bodyFlexTravelGain = 0.10,
    bodyBendingTravelGain = 0.08,
    bodyFlexTauGain = 0.40,
    bodyStiffTauGain = 0.10,
    bodyImpulseSoftening = 0.10,

    tireHopRateGain = 0.08,
    loadNormRateGain = 0.08,
    damageRateLoss = 0.12,

    preserveInputRateGain = 0.08,
    damperPreserveRateGain = 0.06,
    chassisEnergyRateGain = 0.05,
    weightBiasRateGain = 0.04,
    damageRuntimeRateLoss = 0.08,
    maxPreserveRateAdd = 0.20,

    damperHystRateGain = 0.04,
    damperVerticalRateGain = 0.03,
    damperImpactTravelGain = 0.012,

    loadPathLossRateLoss = 0.06,
    loadPathDeliveryRateGain = 0.04,

    minDt = 0.00005,
    maxDt = 0.050,

    noCarReturnTau = 0.220,
    noCarForceTau = 0.180,

    debugStoreInterval = 0.25,
}

local state = {
    force = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    rawForce = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    travel = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    rawTravel = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    effectiveTravel = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    rate = { [0]=30000.0, [1]=30000.0, [2]=30000.0, [3]=30000.0 },
    rateScale = { [0]=1.0, [1]=1.0, [2]=1.0, [3]=1.0 },
    progressiveAmount = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    contactInput = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    contactScale = { [0]=1.0, [1]=1.0, [2]=1.0, [3]=1.0 },
    roadImpulse = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    contactDrop = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    hop = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    loadNorm = { [0]=1.0, [1]=1.0, [2]=1.0, [3]=1.0 },
    damage = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    preserveInputAdd = 0.0,
    chassisEnergy = 0.0,
    damageRuntimeAdd = 0.0,
    weightBias = { [0]=1.0, [1]=1.0, [2]=1.0, [3]=1.0 },
    damperPreserve = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    preserveRateAdd = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    damperHyst = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    damperVertical = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    damperImpact = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    loadPathLoss = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    loadPathDelivery = { [0]=1.0, [1]=1.0, [2]=1.0, [3]=1.0 },

    bodyRigidityScale = { [0]=1.0, [1]=1.0, [2]=1.0, [3]=1.0 },
    bodyTravelOffset = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    bodyForceTau = { [0]=0.045, [1]=0.045, [2]=0.045, [3]=0.045 },

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyBendingFactor = 1.0,
    bodyDampingFactor = 1.0,
    bodyRigidityLinked = false,

    avgForce = 0.0,
    avgAbsForce = 0.0,
    avgRate = 0.0,
    avgRateScale = 1.0,
    avgContactInput = 0.0,
    maxForceLive = 0.0,
    maxTravelLive = 0.0,
    avgPreserveRateAdd = 0.0,

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    contactLinked = false,
    bodyLinked = false,
    tireLinked = false,
    loadLinked = false,
    damageLinked = false,
    preserveLinked = false,
    chassisLinked = false,
    damageRuntimeLinked = false,
    weightLinked = false,
    damperLinked = false,
    loadPathLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

for i = 0, 3 do
    state[i] = {
        force = 0.0,
        travel = 0.0,
        rate = M.params.baseRate,
        rateScale = 1.0,
    }
end

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

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.001)
    dt = safeNumber(dt, 0.0)
    if tau <= 0.0001 then return target end
    return current + (target - current) * clamp(dt / (tau + dt), 0.0, 1.0)
end

local function safeGetCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function()
        return ac.getCar(0)
    end)
    if not ok then return nil end
    return car
end

local function hasWheels(car)
    if not car then return false end
    local ok, wheels = pcall(function()
        return car.wheels
    end)
    return ok and wheels ~= nil
end

local function getWheel(car, index)
    if not hasWheels(car) then return nil end

    local ok, wheel = pcall(function()
        return car.wheels[index]
    end)
    if ok and wheel ~= nil then return wheel end

    ok, wheel = pcall(function()
        return car.wheels[index + 1]
    end)
    if ok and wheel ~= nil then return wheel end

    return nil
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

local function resetLinkFlags()
    state.contactLinked = false
    state.bodyLinked = false
    state.tireLinked = false
    state.loadLinked = false
    state.damageLinked = false
    state.preserveLinked = false
    state.chassisLinked = false
    state.damageRuntimeLinked = false
    state.weightLinked = false
    state.damperLinked = false
    state.loadPathLinked = false
end

local function readBody()
    local bodyRaw = safeLoadRaw("ngp_body_rigidity")
    local flexRaw = safeLoadRaw("ngp_body_flex_factor")
    local torsionRaw = safeLoadRaw("ngp_body_torsion_factor")
    local bendingRaw = safeLoadRaw("ngp_body_bending_factor")
    local dampingRaw = safeLoadRaw("ngp_body_damping_factor")
    local countRaw = safeLoadRaw("ngp_body_rigidity_update_count")

    state.bodyLinked =
        bodyRaw ~= nil or flexRaw ~= nil or torsionRaw ~= nil
        or bendingRaw ~= nil or dampingRaw ~= nil or countRaw ~= nil

    state.bodyRigidity = clamp(safeNumber(bodyRaw, 1.0), 0.20, 1.35)
    state.bodyFlexFactor = clamp(safeNumber(flexRaw, 1.0), 0.25, 1.80)
    state.bodyTorsionFactor = clamp(safeNumber(torsionRaw, 1.0), 0.35, 2.00)
    state.bodyBendingFactor = clamp(safeNumber(bendingRaw, 1.0), 0.35, 2.00)
    state.bodyDampingFactor = clamp(safeNumber(dampingRaw, 1.0), 0.50, 1.80)
    state.bodyRigidityLinked = safeNumber(countRaw, 0.0) > 0.0 or state.bodyLinked
end

local function readPreserveInputs()
    local preserve = safeLoadRaw("ngp_sci_preserve_input_add")
    if preserve == nil then preserve = safeLoadRaw("ngp_susp_preserve_input_add") end
    if preserve == nil then preserve = safeLoadRaw("ngp_preserve_input_add") end

    state.preserveLinked = preserve ~= nil
    state.preserveInputAdd = clamp(safeNumber(preserve, 0.0), 0.0, 0.25)

    local energy = safeLoadRaw("ngp_chassis_energy")
    if energy == nil then energy = safeLoadRaw("ngp_chassis_body_energy") end
    if energy == nil then energy = safeLoadRaw("ngp_chassis_flex_energy") end

    state.chassisLinked = energy ~= nil
    state.chassisEnergy = clamp(safeNumber(energy, 0.0), 0.0, 1.0)

    local damage = safeLoadRaw("ngp_damage_runtime_add")
    if damage == nil then damage = safeLoadRaw("ngp_damage_runtime_total") end
    if damage == nil then damage = safeLoadRaw("ngp_vehicle_condition_total") end

    state.damageRuntimeLinked = damage ~= nil
    state.damageRuntimeAdd = clamp(safeNumber(damage, 0.0), 0.0, 1.0)

    local fb = safeLoadRaw("ngp_weight_front_bias")
    if fb == nil then fb = safeLoadRaw("ngp_front_bias") end

    local rb = safeLoadRaw("ngp_weight_rear_bias")
    if rb == nil then rb = safeLoadRaw("ngp_rear_bias") end

    local lb = safeLoadRaw("ngp_weight_left_bias")
    if lb == nil then lb = safeLoadRaw("ngp_left_bias") end

    local rt = safeLoadRaw("ngp_weight_right_bias")
    if rt == nil then rt = safeLoadRaw("ngp_right_bias") end

    state.weightLinked = fb ~= nil or rb ~= nil or lb ~= nil or rt ~= nil

    local f = clamp(safeNumber(fb, 0.5), 0.25, 0.75)
    local r = clamp(safeNumber(rb, 0.5), 0.25, 0.75)
    local l = clamp(safeNumber(lb, 0.5), 0.25, 0.75)
    local right = clamp(safeNumber(rt, 0.5), 0.25, 0.75)

    state.weightBias[0] = clamp(f * l * 2.0, 0.55, 1.45)
    state.weightBias[1] = clamp(f * right * 2.0, 0.55, 1.45)
    state.weightBias[2] = clamp(r * l * 2.0, 0.55, 1.45)
    state.weightBias[3] = clamp(r * right * 2.0, 0.55, 1.45)

    local damperLinked = false
    local loadPathLinked = false

    for i = 0, 3 do
        local dp = safeLoadRaw("ngp_damper_preserve_coeff_add_" .. i)
        if dp == nil then dp = safeLoadRaw("ngp_damper_preserve_rate_add_" .. i) end
        if dp == nil then dp = safeLoadRaw("ngp_damper_hyst_preserve_" .. i) end
        if dp ~= nil then damperLinked = true end
        state.damperPreserve[i] = clamp(safeNumber(dp, 0.0), 0.0, 0.25)

        local dh = safeLoadRaw("ngp_damper_hyst_" .. i)
        if dh == nil then dh = safeLoadRaw("ngp_damper_hysteretic_" .. i) end

        local dv = safeLoadRaw("ngp_damper_vertical_" .. i)
        if dv == nil then dv = safeLoadRaw("ngp_vertical_damping_" .. i) end

        local di = safeLoadRaw("ngp_damper_hyst_impact_" .. i)
        if di == nil then di = safeLoadRaw("ngp_damper_vertical_pulse_" .. i) end

        if dh ~= nil or dv ~= nil or di ~= nil then damperLinked = true end

        state.damperHyst[i] = clamp(safeNumber(dh, 0.0), 0.0, 1.5)
        state.damperVertical[i] = clamp(safeNumber(dv, 0.0), 0.0, 1.5)
        state.damperImpact[i] = clamp(safeNumber(di, 0.0), 0.0, 1.5)

        local lpLoss = safeLoadRaw("ngp_load_path_loss_" .. i)
        if lpLoss == nil then lpLoss = safeLoadRaw("ngp_lp_loss_" .. i) end

        local lpDelivery = safeLoadRaw("ngp_load_path_tire_delivery_" .. i)
        if lpDelivery == nil then lpDelivery = safeLoadRaw("ngp_lp_tire_delivery_" .. i) end

        if lpLoss ~= nil or lpDelivery ~= nil then loadPathLinked = true end

        state.loadPathLoss[i] = clamp(safeNumber(lpLoss, 0.0), 0.0, 1.0)
        state.loadPathDelivery[i] = clamp(safeNumber(lpDelivery, 1.0), 0.0, 1.2)
    end

    state.damperLinked = damperLinked
    state.loadPathLinked = loadPathLinked
end

local function readTravelFromWheel(wheel, index)
    if wheel then
        local keys = {
            "suspensionTravel",
            "suspensionTravelM",
            "suspensionLength",
            "travel",
            "damperTravel",
            "suspensionPosition",
            "springTravel",
        }

        for _, key in ipairs(keys) do
            local value = safeField(wheel, key, nil)
            if value ~= nil then
                return clamp(safeNumber(value, 0.0), M.params.minTravel, M.params.maxTravel)
            end
        end
    end

    local stored = safeLoadRaw("ngp_susp_travel_" .. index)
    if stored == nil then stored = safeLoadRaw("ngp_damper_travel_" .. index) end
    if stored == nil then stored = safeLoadRaw("ngp_spring_travel_input_" .. index) end

    if stored ~= nil then
        return clamp(safeNumber(stored, 0.0), M.params.minTravel, M.params.maxTravel)
    end

    return state.rawTravel[index] or 0.0
end

local function readRoot(index)
    local ci = safeLoadRaw("ngp_susp_contact_input_" .. index)
    if ci == nil then ci = safeLoadRaw("ngp_sci_input_" .. index) end

    local cs = safeLoadRaw("ngp_susp_contact_scale_" .. index)
    if cs == nil then cs = safeLoadRaw("ngp_sci_scale_" .. index) end

    local ri = safeLoadRaw("ngp_susp_road_impulse_" .. index)
    if ri == nil then ri = safeLoadRaw("ngp_sci_impulse_" .. index) end
    if ri == nil then ri = safeLoadRaw("ngp_impact_root_value") end

    local cd = safeLoadRaw("ngp_susp_contact_drop_" .. index)
    if cd == nil then cd = safeLoadRaw("ngp_sci_drop_" .. index) end
    if cd == nil then cd = safeLoadRaw("ngp_contact_loss_" .. index) end

    if ci ~= nil or cs ~= nil or ri ~= nil or cd ~= nil then
        state.contactLinked = true
    end

    state.contactInput[index] = clamp(safeNumber(ci, 0.0), 0.0, 1.5)
    state.contactScale[index] = clamp(safeNumber(cs, 1.0), 0.5, 1.7)
    state.roadImpulse[index] = clamp(safeNumber(ri, 0.0), 0.0, 1.5)
    state.contactDrop[index] = clamp(safeNumber(cd, 0.0), 0.0, 1.0)

    local hop = safeLoadRaw("ngp_tire_hop_energy_" .. index)
    if hop == nil then hop = safeLoadRaw("ngp_tire_hop_" .. index) end
    if hop == nil then hop = safeLoadRaw("ngp_tirehop_" .. index) end
    if hop ~= nil then state.tireLinked = true end
    state.hop[index] = clamp(safeNumber(hop, 0.0), 0.0, 1.0)

    local loadNorm = safeLoadRaw("ngp_wheel_load_norm_" .. index)
    if loadNorm == nil then
        local load = safeLoadRaw("ngp_wheel_load_" .. index)
        if load ~= nil then
            loadNorm = safeNumber(load, 3000.0) / 3500.0
        end
    end
    if loadNorm ~= nil then state.loadLinked = true end
    state.loadNorm[index] = clamp(safeNumber(loadNorm, 1.0), 0.0, 2.5)

    local dmg = safeLoadRaw("ngp_damage_wheel_" .. index)
    if dmg == nil then dmg = safeLoadRaw("ngp_condition_wheel_" .. index) end
    if dmg ~= nil then state.damageLinked = true end
    state.damage[index] = clamp(safeNumber(dmg, 0.0), 0.0, 1.0)
end

local function applyBody(index, rateScale, travel)
    if not state.bodyRigidityLinked then
        state.bodyRigidityScale[index] = 1.0
        state.bodyTravelOffset[index] = 0.0
        state.bodyForceTau[index] = M.params.forceTau
        return rateScale, travel, M.params.forceTau
    end

    local softness = clamp(state.bodyFlexFactor - 1.0, 0.0, 0.80)
    local stiffness = clamp(state.bodyRigidity - 1.0, 0.0, 0.35)
    local torsionSoft = clamp(state.bodyTorsionFactor - 1.0, 0.0, 1.0)
    local bendingSoft = clamp(state.bodyBendingFactor - 1.0, 0.0, 1.0)
    local impulse = state.roadImpulse[index] or 0.0

    local scale =
        1.0
        - softness * M.params.bodyFlexRateLoss
        - torsionSoft * M.params.bodyTorsionRateLoss
        - bendingSoft * M.params.bodyBendingRateLoss
        - impulse * softness * M.params.bodyImpulseSoftening
        + stiffness * M.params.bodyStiffRateGain

    local offset =
        softness * M.params.bodyFlexTravelGain
        + bendingSoft * M.params.bodyBendingTravelGain
        + impulse * M.params.impulseTravelGain

    local tau =
        M.params.forceTau
        * (1.0 + softness * M.params.bodyFlexTauGain)
        / (1.0 + stiffness * M.params.bodyStiffTauGain)
        / math.max(state.bodyDampingFactor, 0.50)

    state.bodyRigidityScale[index] = clamp(scale, 0.86, 1.10)
    state.bodyTravelOffset[index] = clamp(offset, -0.20, 0.25)
    state.bodyForceTau[index] = clamp(tau, 0.020, 0.080)

    return rateScale * state.bodyRigidityScale[index], travel + state.bodyTravelOffset[index], state.bodyForceTau[index]
end

local function updateWheel(index, car, dt)
    local wheel = getWheel(car, index)
    local rawTravel = readTravelFromWheel(wheel, index)
    state.rawTravel[index] = rawTravel

    readRoot(index)

    local travel =
        rawTravel
        + state.contactInput[index] * M.params.travelInputGain
        + state.roadImpulse[index] * M.params.impulseTravelGain
        - state.contactDrop[index] * M.params.dropTravelLoss
        + state.damperImpact[index] * M.params.damperImpactTravelGain

    local rateScale =
        1.0
        + state.contactInput[index] * M.params.contactRateGain
        + (state.contactScale[index] - 1.0) * M.params.contactScaleGain
        + state.roadImpulse[index] * M.params.roadImpulseRateGain
        - state.contactDrop[index] * M.params.contactDropRateLoss
        + state.hop[index] * M.params.tireHopRateGain
        + math.max(state.loadNorm[index] - 1.0, 0.0) * M.params.loadNormRateGain
        - state.damage[index] * M.params.damageRateLoss
        + state.preserveInputAdd * M.params.preserveInputRateGain
        + state.damperPreserve[index] * M.params.damperPreserveRateGain
        + state.chassisEnergy * M.params.chassisEnergyRateGain
        + math.max(state.weightBias[index] - 1.0, 0.0) * M.params.weightBiasRateGain
        - state.damageRuntimeAdd * M.params.damageRuntimeRateLoss
        + state.damperHyst[index] * M.params.damperHystRateGain
        + state.damperVertical[index] * M.params.damperVerticalRateGain
        - state.loadPathLoss[index] * M.params.loadPathLossRateLoss
        + math.max(state.loadPathDelivery[index] - 1.0, 0.0) * M.params.loadPathDeliveryRateGain

    state.preserveRateAdd[index] =
        clamp(
            state.preserveInputAdd * M.params.preserveInputRateGain
            + state.damperPreserve[index] * M.params.damperPreserveRateGain
            + state.chassisEnergy * M.params.chassisEnergyRateGain
            + state.damperHyst[index] * M.params.damperHystRateGain,
            0.0,
            M.params.maxPreserveRateAdd
        )

    rateScale = clamp(rateScale, M.params.minRateScale, M.params.maxRateScale)

    local finalRateScale, effectiveTravel, tau = applyBody(index, rateScale, travel)

    state.rateScale[index] = clamp(finalRateScale, M.params.minRateScale * 0.85, M.params.maxRateScale * 1.08)
    state.effectiveTravel[index] = clamp(effectiveTravel, M.params.minTravel, M.params.maxTravel)

    state.travel[index] = lowPass(
        state.travel[index],
        state.effectiveTravel[index],
        M.params.inputTau,
        dt
    )

    local compression = math.max(state.travel[index] - M.params.startTravel, 0.0)
    local baseRate = M.params.baseRate + compression * M.params.progressiveRate

    state.rate[index] = math.max(0.0, baseRate * state.rateScale[index])
    state.progressiveAmount[index] = compression

    local rawForce = clamp(
        state.rate[index] * state.travel[index],
        -M.params.maxForce,
        M.params.maxForce
    )

    state.rawForce[index] = rawForce
    state.force[index] = lowPass(state.force[index], rawForce, tau, dt)

    local proxy = state[index]
    if proxy then
        proxy.force = state.force[index]
        proxy.travel = state.travel[index]
        proxy.rate = state.rate[index]
        proxy.rateScale = state.rateScale[index]
    end
end

local function decayWheel(index, dt)
    state.rawTravel[index] = lowPass(state.rawTravel[index], 0.0, M.params.noCarReturnTau, dt)
    state.travel[index] = lowPass(state.travel[index], 0.0, M.params.noCarReturnTau, dt)
    state.effectiveTravel[index] = lowPass(state.effectiveTravel[index], 0.0, M.params.noCarReturnTau, dt)
    state.rawForce[index] = lowPass(state.rawForce[index], 0.0, M.params.noCarForceTau, dt)
    state.force[index] = lowPass(state.force[index], 0.0, M.params.noCarForceTau, dt)
    state.rate[index] = lowPass(state.rate[index], M.params.baseRate, M.params.noCarReturnTau, dt)
    state.rateScale[index] = lowPass(state.rateScale[index], 1.0, M.params.noCarReturnTau, dt)
    state.progressiveAmount[index] = lowPass(state.progressiveAmount[index], 0.0, M.params.noCarReturnTau, dt)
    state.preserveRateAdd[index] = lowPass(state.preserveRateAdd[index], 0.0, M.params.noCarReturnTau, dt)
    state.contactInput[index] = lowPass(state.contactInput[index], 0.0, M.params.noCarReturnTau, dt)
    state.roadImpulse[index] = lowPass(state.roadImpulse[index], 0.0, M.params.noCarReturnTau, dt)
    state.contactDrop[index] = lowPass(state.contactDrop[index], 0.0, M.params.noCarReturnTau, dt)
    state.loadNorm[index] = lowPass(state.loadNorm[index], 1.0, M.params.noCarReturnTau, dt)
end

local function updateAverages()
    local sumForce = 0.0
    local sumAbs = 0.0
    local sumRate = 0.0
    local sumScale = 0.0
    local sumInput = 0.0
    local sumPreserve = 0.0
    local maxForceLive = 0.0
    local maxTravelLive = 0.0

    for i = 0, 3 do
        sumForce = sumForce + (state.force[i] or 0.0)
        sumAbs = sumAbs + abs(state.force[i])
        sumRate = sumRate + (state.rate[i] or M.params.baseRate)
        sumScale = sumScale + (state.rateScale[i] or 1.0)
        sumInput = sumInput + (state.contactInput[i] or 0.0)
        sumPreserve = sumPreserve + (state.preserveRateAdd[i] or 0.0)
        maxForceLive = math.max(maxForceLive, abs(state.force[i]))
        maxTravelLive = math.max(maxTravelLive, abs(state.travel[i]))
    end

    state.avgForce = sumForce * 0.25
    state.avgAbsForce = sumAbs * 0.25
    state.avgRate = sumRate * 0.25
    state.avgRateScale = sumScale * 0.25
    state.avgContactInput = sumInput * 0.25
    state.avgPreserveRateAdd = sumPreserve * 0.25
    state.maxForceLive = maxForceLive
    state.maxTravelLive = maxTravelLive
end

local function exportWheel(index)
    safeStore("ngp_progressive_spring_" .. index, state.force[index])
    safeStore("ngp_spring_force_" .. index, state.force[index])
    safeStore("ngp_spring_raw_" .. index, state.rawForce[index])
    safeStore("ngp_spring_rate_" .. index, state.rate[index])
    safeStore("ngp_spring_rate_scale_" .. index, state.rateScale[index])
    safeStore("ngp_spring_travel_" .. index, state.travel[index])
    safeStore("ngp_spring_effective_travel_" .. index, state.effectiveTravel[index])
    safeStore("ngp_spring_body_scale_" .. index, state.bodyRigidityScale[index])
    safeStore("ngp_spring_preserve_rate_add_" .. index, state.preserveRateAdd[index])

    safeStore("ngp_spring_progressive_amount_" .. index, state.progressiveAmount[index])
    safeStore("ngp_spring_load_norm_" .. index, state.loadNorm[index])
    safeStore("ngp_spring_damper_hyst_" .. index, state.damperHyst[index])
    safeStore("ngp_spring_vertical_damping_" .. index, state.damperVertical[index])

    safeStore("ngp_ps_force_" .. index, state.force[index])
    safeStore("ngp_ps_rate_" .. index, state.rate[index])
    safeStore("ngp_ps_scale_" .. index, state.rateScale[index])
    safeStore("ngp_ps_travel_" .. index, state.travel[index])

    if not state.debugStoreNow then return end

    safeStore("ngp_spring_contact_input_" .. index, state.contactInput[index])
    safeStore("ngp_spring_impulse_" .. index, state.roadImpulse[index])
    safeStore("ngp_spring_drop_" .. index, state.contactDrop[index])
    safeStore("ngp_spring_raw_travel_" .. index, state.rawTravel[index])
    safeStore("ngp_spring_contact_scale_" .. index, state.contactScale[index])
    safeStore("ngp_spring_hop_" .. index, state.hop[index])
    safeStore("ngp_spring_damage_" .. index, state.damage[index])
    safeStore("ngp_spring_body_travel_offset_" .. index, state.bodyTravelOffset[index])
    safeStore("ngp_spring_body_tau_" .. index, state.bodyForceTau[index])
end

local function exportGlobal()
    safeStore("ngp_progressive_spring_status", state.status)
    safeStore("ngp_progressive_spring_update_count", state.updateCount)
    safeStore("ngp_progressive_spring_wheels_valid", state.wheelsValid and 1 or 0)

    safeStore("ngp_spring_avg_force", state.avgForce)
    safeStore("ngp_spring_avg_abs_force", state.avgAbsForce)
    safeStore("ngp_spring_avg_rate", state.avgRate)
    safeStore("ngp_spring_avg_rate_scale", state.avgRateScale)
    safeStore("ngp_spring_avg_contact_input", state.avgContactInput)
    safeStore("ngp_spring_avg_preserve_rate_add", state.avgPreserveRateAdd)
    safeStore("ngp_spring_max_force", state.maxForceLive)
    safeStore("ngp_spring_max_travel", state.maxTravelLive)

    safeStore("ngp_ps_status", state.status)
    safeStore("ngp_ps_count", state.updateCount)
    safeStore("ngp_ps_avg_force", state.avgAbsForce)
    safeStore("ngp_ps_avg_rate", state.avgRate)
    safeStore("ngp_ps_avg_scale", state.avgRateScale)

    if not state.debugStoreNow then return end

    safeStore("ngp_spring_contact_linked", state.contactLinked and 1 or 0)
    safeStore("ngp_spring_body_linked", state.bodyLinked and 1 or 0)
    safeStore("ngp_spring_tire_linked", state.tireLinked and 1 or 0)
    safeStore("ngp_spring_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_spring_damage_linked", state.damageLinked and 1 or 0)
    safeStore("ngp_spring_preserve_linked", state.preserveLinked and 1 or 0)
    safeStore("ngp_spring_chassis_linked", state.chassisLinked and 1 or 0)
    safeStore("ngp_spring_damage_runtime_linked", state.damageRuntimeLinked and 1 or 0)
    safeStore("ngp_spring_weight_linked", state.weightLinked and 1 or 0)
    safeStore("ngp_spring_damper_linked", state.damperLinked and 1 or 0)
    safeStore("ngp_spring_load_path_linked", state.loadPathLinked and 1 or 0)

    safeStore("ngp_spring_body_rigidity", state.bodyRigidity)
    safeStore("ngp_spring_body_flex_factor", state.bodyFlexFactor)
    safeStore("ngp_spring_chassis_energy", state.chassisEnergy)
    safeStore("ngp_spring_damage_runtime_add", state.damageRuntimeAdd)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i)
    end
    exportGlobal()
end

function M.init(car)
    state.status = "INIT"
    state.updateCount = state.updateCount or 0
    readBody()
    readPreserveInputs()
    updateAverages()
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = safeNumber(dt, 0.0)
    if dt <= 0.0 then
        state.status = "BAD DT"
        updateAverages()
        exportState()
        return
    end

    dt = clamp(dt, M.params.minDt, M.params.maxDt)
    updateDebugGate(dt)

    if not car then
        car = safeGetCar()
    end

    resetLinkFlags()
    readBody()
    readPreserveInputs()

    if not car then
        state.status = "NO CAR"
        state.wheelsValid = false
        for i = 0, 3 do
            decayWheel(i, dt)
        end
        updateAverages()
        exportState()
        return
    end

    if not hasWheels(car) then
        state.status = "NO WHEELS"
        state.wheelsValid = false
        for i = 0, 3 do
            decayWheel(i, dt)
        end
        updateAverages()
        exportState()
        return
    end

    state.status = "RUNNING"
    state.wheelsValid = true

    for i = 0, 3 do
        updateWheel(i, car, dt)
    end

    updateAverages()
    exportState()
end

function M.getForce(index)
    if index == nil then return state.avgForce or 0.0 end
    return state.force[index] or 0.0
end

function M.getAbsForce(index)
    if index == nil then return state.avgAbsForce or 0.0 end
    return abs(state.force[index])
end

function M.getRate(index)
    if index == nil then return state.avgRate or M.params.baseRate end
    return state.rate[index] or M.params.baseRate
end

function M.getRateScale(index)
    if index == nil then return state.avgRateScale or 1.0 end
    return state.rateScale[index] or 1.0
end

function M.getTravel(index)
    if index == nil then return state.maxTravelLive or 0.0 end
    return state.travel[index] or 0.0
end

function M.getState(index)
    if index ~= nil then
        return {
            force = state.force[index] or 0.0,
            rawForce = state.rawForce[index] or 0.0,
            travel = state.travel[index] or 0.0,
            rawTravel = state.rawTravel[index] or 0.0,
            effectiveTravel = state.effectiveTravel[index] or 0.0,
            rate = state.rate[index] or M.params.baseRate,
            rateScale = state.rateScale[index] or 1.0,
            progressiveAmount = state.progressiveAmount[index] or 0.0,
            preserveRateAdd = state.preserveRateAdd[index] or 0.0,
        }
    end

    return state
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0
        return string.format(
            "%s Spring %.0f Raw %.0f Rate %.0f Scale %.2f\n" ..
            "Travel %.3f Eff %.3f Prog %.3f Preserve %.3f\n" ..
            "CI %.2f Imp %.2f Drop %.2f Load %.2f Body %.2f",
            tostring(WHEEL_NAMES[i] or i),
            state.force[i] or 0.0,
            state.rawForce[i] or 0.0,
            state.rate[i] or 0.0,
            state.rateScale[i] or 1.0,
            state.travel[i] or 0.0,
            state.effectiveTravel[i] or 0.0,
            state.progressiveAmount[i] or 0.0,
            state.preserveRateAdd[i] or 0.0,
            state.contactInput[i] or 0.0,
            state.roadImpulse[i] or 0.0,
            state.contactDrop[i] or 0.0,
            state.loadNorm[i] or 1.0,
            state.bodyRigidityScale[i] or 1.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "Spring %.0f %.0f %.0f %.0f / AvgAbs %.0f Max %.0f\n" ..
        "Rate %.0f %.0f %.0f %.0f / Avg %.0f\n" ..
        "RateScale %.2f %.2f %.2f %.2f / Avg %.2f\n" ..
        "Travel %.3f %.3f %.3f %.3f / Max %.3f\n" ..
        "Preserve %.3f Avg %.3f / Chassis %.3f Damage %.3f\n" ..
        "Links Contact:%s Body:%s Tire:%s Load:%s Damage:%s Preserve:%s Chassis:%s Weight:%s Damper:%s LP:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",

        state.force[0] or 0.0,
        state.force[1] or 0.0,
        state.force[2] or 0.0,
        state.force[3] or 0.0,
        state.avgAbsForce or 0.0,
        state.maxForceLive or 0.0,

        state.rate[0] or 0.0,
        state.rate[1] or 0.0,
        state.rate[2] or 0.0,
        state.rate[3] or 0.0,
        state.avgRate or 0.0,

        state.rateScale[0] or 1.0,
        state.rateScale[1] or 1.0,
        state.rateScale[2] or 1.0,
        state.rateScale[3] or 1.0,
        state.avgRateScale or 1.0,

        state.travel[0] or 0.0,
        state.travel[1] or 0.0,
        state.travel[2] or 0.0,
        state.travel[3] or 0.0,
        state.maxTravelLive or 0.0,

        state.preserveInputAdd or 0.0,
        state.avgPreserveRateAdd or 0.0,
        state.chassisEnergy or 0.0,
        state.damageRuntimeAdd or 0.0,

        state.contactLinked and "OK" or "NIL",
        state.bodyLinked and "OK" or "NIL",
        state.tireLinked and "OK" or "NIL",
        state.loadLinked and "OK" or "NIL",
        state.damageLinked and "OK" or "NIL",
        state.preserveLinked and "OK" or "NIL",
        state.chassisLinked and "OK" or "NIL",
        state.weightLinked and "OK" or "NIL",
        state.damperLinked and "OK" or "NIL",
        state.loadPathLinked and "OK" or "NIL"
    )
end

return M
