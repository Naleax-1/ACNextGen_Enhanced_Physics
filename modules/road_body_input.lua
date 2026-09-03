---@diagnostic disable: undefined-global

--============================================================
-- road_body_input.lua
-- ACNextGen V1.1.5 Stable
-- Road / Suspension / Load -> Body Input Bridge
--============================================================

local M = {}

local WHEEL_NAMES = { [0] = "FL", [1] = "FR", [2] = "RL", [3] = "RR" }

M.params = {
    inputTau = 0.070,
    heaveTau = 0.090,
    pitchTau = 0.100,
    rollTau  = 0.085,
    yawTau   = 0.120,

    wheelLoadRef = 3500.0,
    loadInputGain = 0.38,
    loadNormGain  = 0.26,
    loadRateGain  = 0.060,
    loadRateRef   = 85000.0,

    suspForceGain = 0.00018,
    damperGain    = 0.00010,
    springGain    = 0.00006,

    contactGain = 0.45,
    impulseGain = 0.55,
    dropGain    = 0.60,
    contactLossGain = 0.30,
    contactQualityGain = 0.18,

    damperPulseGain = 0.30,
    damperRoadMemoryGain = 0.18,
    loadPathRoadGain = 0.24,
    verticalFlowGain = 0.20,
    impactGain = 0.22,
    drivelineYawGain = 0.18,

    heaveGain = 1.00,
    pitchGain = 1.00,
    rollGain  = 1.00,
    yawGain   = 0.35,

    maxWheelInput = 1.0,
    maxHeave = 1.0,
    maxPitch = 1.0,
    maxRoll  = 1.0,
    maxYaw   = 1.0,

    bodyRigidityInfluence = 0.16,
    bodyFlexInputLoss = 0.10,
    bodyTorsionRollLoss = 0.08,
    bodyBendingPitchLoss = 0.08,
    bodyHeaveSoftening = 0.06,
    bodyStiffInputGain = 0.05,
    bodyFlexTauGain = 0.40,
    bodyStiffTauGain = 0.10,
    minBodyInputScale = 0.86,
    maxBodyInputScale = 1.10,

    decayTau = 0.180,
    minDt = 0.00005,
    maxDt = 0.050,
    debugStoreInterval = 0.25,
}

local state = {
    wheelInput = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },
    rawWheelInput = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },
    wheelLoad = { [0]=3500.0,[1]=3500.0,[2]=3500.0,[3]=3500.0 },
    prevWheelLoad = { [0]=3500.0,[1]=3500.0,[2]=3500.0,[3]=3500.0 },
    wheelLoadRate = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },
    wheelLoadNorm = { [0]=1.0,[1]=1.0,[2]=1.0,[3]=1.0 },

    suspForce = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },
    damperForce = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },
    springForce = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },
    contactInput = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },
    impulse = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },
    drop = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },
    contactQuality = { [0]=1.0,[1]=1.0,[2]=1.0,[3]=1.0 },
    contactTrust = { [0]=1.0,[1]=1.0,[2]=1.0,[3]=1.0 },
    contactLoss = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },
    damperPulse = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },
    roadMemory = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },
    loadPathRoad = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },
    verticalFlow = { [0]=0.0,[1]=0.0,[2]=0.0,[3]=0.0 },

    heaveInput = 0.0,
    pitchInput = 0.0,
    rollInput  = 0.0,
    yawHint    = 0.0,

    rawHeave = 0.0,
    rawPitch = 0.0,
    rawRoll = 0.0,
    rawYaw = 0.0,

    frontInput = 0.0,
    rearInput  = 0.0,
    leftInput  = 0.0,
    rightInput = 0.0,
    avgWheelInput = 0.0,
    maxWheelInputLive = 0.0,

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyBendingFactor = 1.0,
    bodyDampingFactor = 1.0,
    bodyHeaveScale = 1.0,
    bodyPitchScale = 1.0,
    bodyRollScale = 1.0,
    bodyYawScale = 1.0,
    bodyInputTau = 0.070,

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,
    bodyRigidityLinked = false,
    loadLinked = false,
    suspensionLinked = false,
    contactLinked = false,
    damperLinked = false,
    springLinked = false,
    loadPathLinked = false,
    impactLinked = false,
    drivelineLinked = false,
    carLinked = false,

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
    v = safeNumber(v, minValue)
    if v < minValue then return minValue end
    if v > maxValue then return maxValue end
    return v
end

local function abs(v)
    return math.abs(safeNumber(v, 0.0))
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

local function loadFirst(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local v = safeLoadRaw(keys[i])
        if v ~= nil then return safeNumber(v, defaultValue or 0.0), keys[i] end
    end
    return defaultValue or 0.0, nil
end

local function safeStore(key, value)
    if not ac or not ac.store then return end
    pcall(function() ac.store(key, value) end)
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

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.0)
    dt = safeNumber(dt, 0.0)
    if tau <= 0.0001 then return target end
    return current + (target - current) * clamp(dt / (tau + dt), 0.0, 1.0)
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

local function getWheel(car, index)
    if not car then return nil end
    local wheels = safeField(car, "wheels", nil)
    if not wheels then return nil end
    local ok, wheel = pcall(function() return wheels[index] or wheels[index + 1] end)
    if ok then return wheel end
    return nil
end

local function getWheelLoadFromCar(wheel)
    if not wheel then return nil end
    local keys = { "load", "loadK" }
    for i = 1, #keys do
        local value = safeField(wheel, keys[i], nil)
        if value ~= nil then
            local n = safeNumber(value, nil)
            if n ~= nil and n > 0.0 then return n end
        end
    end
    return nil
end

local function hasWheels(car)
    if not car then return false end
    local wheels = safeField(car, "wheels", nil)
    if not wheels then return false end
    return getWheel(car, 0) ~= nil or getWheel(car, 1) ~= nil or getWheel(car, 2) ~= nil or getWheel(car, 3) ~= nil
end

local function resetLinkFlags()
    state.loadLinked = false
    state.suspensionLinked = false
    state.contactLinked = false
    state.damperLinked = false
    state.springLinked = false
    state.loadPathLinked = false
    state.impactLinked = false
    state.drivelineLinked = false
end

local function readBodyRigidity()
    local bodyCount = safeLoadRaw("ngp_body_rigidity_update_count")
    local bodyRigidity = safeLoadRaw("ngp_body_rigidity")
    local bodyFlex = safeLoadRaw("ngp_body_flex_factor")

    state.bodyRigidityLinked = bodyCount ~= nil or bodyRigidity ~= nil or bodyFlex ~= nil
    state.bodyRigidity = clamp(safeNumber(bodyRigidity, 1.0), 0.20, 1.35)
    state.bodyFlexFactor = clamp(safeLoad("ngp_body_flex_factor", 1.0), 0.25, 1.80)
    state.bodyTorsionFactor = clamp(safeLoad("ngp_body_torsion_factor", 1.0), 0.35, 2.00)
    state.bodyBendingFactor = clamp(safeLoad("ngp_body_bending_factor", 1.0), 0.35, 2.00)
    state.bodyDampingFactor = clamp(safeLoad("ngp_body_damping_factor", 1.0), 0.50, 1.80)
end

local function calculateBodyScales(dt)
    local p = M.params

    if not state.bodyRigidityLinked then
        state.bodyHeaveScale = lowPass(state.bodyHeaveScale, 1.0, p.inputTau, dt)
        state.bodyPitchScale = lowPass(state.bodyPitchScale, 1.0, p.inputTau, dt)
        state.bodyRollScale = lowPass(state.bodyRollScale, 1.0, p.inputTau, dt)
        state.bodyYawScale = lowPass(state.bodyYawScale, 1.0, p.inputTau, dt)
        state.bodyInputTau = p.inputTau
        return
    end

    local rigidity = clamp(state.bodyRigidity or 1.0, 0.20, 1.35)
    local flex = clamp(state.bodyFlexFactor or 1.0, 0.25, 1.80)
    local torsion = clamp(state.bodyTorsionFactor or 1.0, 0.35, 2.00)
    local bending = clamp(state.bodyBendingFactor or 1.0, 0.35, 2.00)
    local damping = clamp(state.bodyDampingFactor or 1.0, 0.50, 1.80)

    local softness = clamp(flex - 1.0, 0.0, 0.80)
    local stiffness = clamp(rigidity - 1.0, 0.0, 0.35)
    local torsionSoft = clamp(torsion - 1.0, 0.0, 1.0)
    local bendingSoft = clamp(bending - 1.0, 0.0, 1.0)

    local heaveTarget = 1.0 - softness * p.bodyHeaveSoftening + stiffness * p.bodyStiffInputGain
    local pitchTarget = 1.0 - softness * p.bodyFlexInputLoss - bendingSoft * p.bodyBendingPitchLoss + stiffness * p.bodyStiffInputGain
    local rollTarget = 1.0 - softness * p.bodyFlexInputLoss - torsionSoft * p.bodyTorsionRollLoss + stiffness * p.bodyStiffInputGain
    local yawTarget = 1.0 - softness * p.bodyFlexInputLoss - torsionSoft * p.bodyTorsionRollLoss * 0.75 + stiffness * p.bodyStiffInputGain

    heaveTarget = clamp(heaveTarget, p.minBodyInputScale, p.maxBodyInputScale)
    pitchTarget = clamp(pitchTarget, p.minBodyInputScale, p.maxBodyInputScale)
    rollTarget = clamp(rollTarget, p.minBodyInputScale, p.maxBodyInputScale)
    yawTarget = clamp(yawTarget, p.minBodyInputScale, p.maxBodyInputScale)

    state.bodyHeaveScale = lowPass(state.bodyHeaveScale, heaveTarget, p.inputTau, dt)
    state.bodyPitchScale = lowPass(state.bodyPitchScale, pitchTarget, p.inputTau, dt)
    state.bodyRollScale = lowPass(state.bodyRollScale, rollTarget, p.inputTau, dt)
    state.bodyYawScale = lowPass(state.bodyYawScale, yawTarget, p.inputTau, dt)

    local tau = p.inputTau * (1.0 + softness * p.bodyFlexTauGain) / (1.0 + stiffness * p.bodyStiffTauGain)
    tau = tau / math.max(damping, 0.50)
    state.bodyInputTau = clamp(tau, 0.025, 0.090)
end

local function readWheelLoad(index, wheel)
    local load, key = loadFirst(
        nil,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index,
        "ngp_tire_load_input_" .. index,
        "ngp_load_path_load_" .. index,
        "ngp_contact_load_" .. index,
        "ngp_tire_state_load_" .. index
    )

    if key ~= nil then state.loadLinked = true end

    if load == nil then
        local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
        if dlt ~= nil then
            state.loadLinked = true
            local n = safeNumber(dlt, 0.0)
            if abs(n) > 900.0 then
                load = abs(n)
            else
                load = M.params.wheelLoadRef + n
            end
        end
    end

    if load == nil then
        load = getWheelLoadFromCar(wheel)
        if load ~= nil then state.carLinked = true end
    end

    return clamp(safeNumber(load, M.params.wheelLoadRef), 0.0, 12000.0)
end

local function readWheelSignals(index, wheel, dt)
    local p = M.params

    local load = readWheelLoad(index, wheel)
    local prev = state.wheelLoad[index] or load
    state.prevWheelLoad[index] = prev
    state.wheelLoad[index] = load
    state.wheelLoadRate[index] = (load - prev) / math.max(dt, 0.001)

    local norm, normKey = loadFirst(
        nil,
        "ngp_wheel_load_norm_" .. index,
        "ngp_load_path_load_norm_" .. index,
        "ngp_lp_load_norm_" .. index
    )

    if normKey ~= nil then state.loadLinked = true end
    state.wheelLoadNorm[index] = clamp(safeNumber(norm, load / math.max(p.wheelLoadRef, 1.0)), 0.0, 2.5)

    local susp, sk = loadFirst(0.0, "ngp_susp_integrated_force_" .. index, "ngp_susp_int_force_" .. index, "ngp_susp_" .. index)
    local damper, dk = loadFirst(0.0, "ngp_damper_" .. index, "ngp_damper_force_" .. index, "ngp_damper_hyst_force_" .. index)
    local spring, spk = loadFirst(0.0, "ngp_progressive_spring_" .. index, "ngp_spring_force_" .. index)

    if sk ~= nil then state.suspensionLinked = true end
    if dk ~= nil then state.damperLinked = true end
    if spk ~= nil then state.springLinked = true end

    state.suspForce[index] = susp
    state.damperForce[index] = damper
    state.springForce[index] = spring

    local ci, cik = loadFirst(0.0, "ngp_susp_contact_input_" .. index, "ngp_susp_int_contact_" .. index, "ngp_sci_input_" .. index)
    local impulse, ik = loadFirst(0.0, "ngp_susp_road_impulse_" .. index, "ngp_susp_int_impulse_" .. index, "ngp_sci_impulse_" .. index)
    local drop, dropKey = loadFirst(0.0, "ngp_susp_contact_drop_" .. index, "ngp_susp_int_drop_" .. index, "ngp_sci_drop_" .. index)

    if cik ~= nil or ik ~= nil or dropKey ~= nil then state.contactLinked = true end

    state.contactInput[index] = clamp(ci, 0.0, 1.5)
    state.impulse[index] = clamp(impulse, 0.0, 1.5)
    state.drop[index] = clamp(drop, 0.0, 1.0)

    local q, qk = loadFirst(1.0, "ngp_contact_quality_" .. index, "ngp_tire_contact_quality_" .. index, "ngp_tcr_quality_" .. index, "ngp_tc_contact_" .. index)
    local trust, tk = loadFirst(q, "ngp_contact_trust_" .. index, "ngp_tire_contact_trust_" .. index)
    local loss, lk = loadFirst(nil, "ngp_contact_loss_" .. index, "ngp_tire_contact_loss_" .. index, "ngp_tcr_contact_loss_" .. index)

    if qk ~= nil or tk ~= nil or lk ~= nil then state.contactLinked = true end

    state.contactQuality[index] = clamp(q, 0.0, 1.2)
    state.contactTrust[index] = clamp(trust, 0.0, 1.2)
    state.contactLoss[index] = clamp(safeNumber(loss, 1.0 - clamp(state.contactQuality[index], 0.0, 1.0)), 0.0, 1.0)

    local pulse, pk = loadFirst(0.0, "ngp_damper_vertical_pulse_" .. index, "ngp_road_vertical_pulse_" .. index, "ngp_damper_hyst_impact_" .. index)
    local roadMem, rmk = loadFirst(0.0, "ngp_damper_road_memory_" .. index, "ngp_road_damper_memory_" .. index)
    if pk ~= nil or rmk ~= nil then state.damperLinked = true end
    state.damperPulse[index] = clamp(pulse, 0.0, 1.5)
    state.roadMemory[index] = clamp(roadMem, 0.0, 1.5)

    local road, lpk = loadFirst(0.0, "ngp_load_path_road_" .. index, "ngp_lp_road_" .. index)
    local vflow, vfk = loadFirst(0.0, "ngp_load_path_vertical_flow_" .. index, "ngp_lp_vertical_flow_" .. index)
    if lpk ~= nil or vfk ~= nil then state.loadPathLinked = true end
    state.loadPathRoad[index] = clamp(road, 0.0, 1.5)
    state.verticalFlow[index] = clamp(vflow, 0.0, 1.5)
end

local function calculateWheelInput(index, dt)
    local p = M.params

    local loadPart = clamp(abs(state.wheelLoadNorm[index] - 1.0) * p.loadNormGain, 0.0, 1.0)
    local loadRatePart = clamp(abs(state.wheelLoadRate[index]) / math.max(p.loadRateRef, 1.0) * p.loadRateGain, 0.0, 0.35)

    local forcePart =
        abs(state.suspForce[index]) * p.suspForceGain +
        abs(state.damperForce[index]) * p.damperGain +
        abs(state.springForce[index]) * p.springGain

    local contactPart =
        state.contactInput[index] * p.contactGain +
        state.impulse[index] * p.impulseGain -
        state.drop[index] * p.dropGain +
        state.contactLoss[index] * p.contactLossGain +
        clamp(1.0 - state.contactTrust[index], 0.0, 1.0) * p.contactQualityGain

    local roadPart =
        state.damperPulse[index] * p.damperPulseGain +
        state.roadMemory[index] * p.damperRoadMemoryGain +
        state.loadPathRoad[index] * p.loadPathRoadGain +
        state.verticalFlow[index] * p.verticalFlowGain

    local target = clamp(loadPart + loadRatePart + forcePart + contactPart + roadPart, 0.0, p.maxWheelInput)
    state.rawWheelInput[index] = target
    state.wheelInput[index] = lowPass(state.wheelInput[index], target, p.inputTau, dt)
end

local function readGlobalCoupling()
    local impact, impactKey = loadFirst(0.0, "ngp_impact_value", "ngp_impact_root_value", "ngp_damage_event_last")
    if impactKey ~= nil then state.impactLinked = true end

    local driveYaw, yawKey = loadFirst(0.0, "ngp_driveline_yaw_hint", "ngp_drive_soft_yaw", "ngp_windup_yaw_hint")
    if yawKey ~= nil then state.drivelineLinked = true end

    return clamp(impact, 0.0, 1.0), clamp(driveYaw, -1.0, 1.0)
end

local function updateBodyInputs(dt)
    local p = M.params

    local fl = state.wheelInput[0] or 0.0
    local fr = state.wheelInput[1] or 0.0
    local rl = state.wheelInput[2] or 0.0
    local rr = state.wheelInput[3] or 0.0

    local front = (fl + fr) * 0.5
    local rear = (rl + rr) * 0.5
    local left = (fl + rl) * 0.5
    local right = (fr + rr) * 0.5

    local impact, driveYaw = readGlobalCoupling()

    local heaveTarget = clamp((fl + fr + rl + rr) * 0.25 * p.heaveGain + impact * p.impactGain, 0.0, p.maxHeave)
    local pitchTarget = clamp((front - rear) * p.pitchGain, -p.maxPitch, p.maxPitch)
    local rollTarget = clamp((right - left) * p.rollGain, -p.maxRoll, p.maxRoll)
    local yawTarget = clamp(((fr + rl) - (fl + rr)) * 0.5 * p.yawGain + driveYaw * p.drivelineYawGain, -p.maxYaw, p.maxYaw)

    calculateBodyScales(dt)

    state.rawHeave = heaveTarget
    state.rawPitch = pitchTarget
    state.rawRoll = rollTarget
    state.rawYaw = yawTarget

    heaveTarget = clamp(heaveTarget * state.bodyHeaveScale, 0.0, p.maxHeave)
    pitchTarget = clamp(pitchTarget * state.bodyPitchScale, -p.maxPitch, p.maxPitch)
    rollTarget = clamp(rollTarget * state.bodyRollScale, -p.maxRoll, p.maxRoll)
    yawTarget = clamp(yawTarget * state.bodyYawScale, -p.maxYaw, p.maxYaw)

    state.frontInput = front
    state.rearInput = rear
    state.leftInput = left
    state.rightInput = right
    state.avgWheelInput = heaveTarget
    state.maxWheelInputLive = math.max(fl, fr, rl, rr)

    local tauScale = state.bodyInputTau or p.inputTau
    state.heaveInput = lowPass(state.heaveInput, heaveTarget, math.max(p.heaveTau, tauScale), dt)
    state.pitchInput = lowPass(state.pitchInput, pitchTarget, math.max(p.pitchTau, tauScale), dt)
    state.rollInput  = lowPass(state.rollInput,  rollTarget,  math.max(p.rollTau, tauScale), dt)
    state.yawHint    = lowPass(state.yawHint,    yawTarget,   math.max(p.yawTau, tauScale), dt)
end

local function decayState(dt)
    for i = 0, 3 do
        state.wheelInput[i] = lowPass(state.wheelInput[i], 0.0, M.params.decayTau, dt)
        state.rawWheelInput[i] = 0.0
        state.wheelLoad[i] = lowPass(state.wheelLoad[i], M.params.wheelLoadRef, M.params.decayTau, dt)
        state.wheelLoadNorm[i] = lowPass(state.wheelLoadNorm[i], 1.0, M.params.decayTau, dt)
        state.wheelLoadRate[i] = lowPass(state.wheelLoadRate[i], 0.0, M.params.decayTau, dt)
    end

    state.heaveInput = lowPass(state.heaveInput, 0.0, M.params.decayTau, dt)
    state.pitchInput = lowPass(state.pitchInput, 0.0, M.params.decayTau, dt)
    state.rollInput = lowPass(state.rollInput, 0.0, M.params.decayTau, dt)
    state.yawHint = lowPass(state.yawHint, 0.0, M.params.decayTau, dt)
    state.frontInput = 0.0
    state.rearInput = 0.0
    state.leftInput = 0.0
    state.rightInput = 0.0
    state.avgWheelInput = 0.0
    state.maxWheelInputLive = 0.0
end

local function exportWheel(index)
    safeStore("ngp_rbi_wheel_input_" .. index, state.wheelInput[index] or 0.0)
    safeStore("ngp_body_wheel_input_" .. index, state.wheelInput[index] or 0.0)
    safeStore("ngp_road_body_wheel_input_" .. index, state.wheelInput[index] or 0.0)
    safeStore("ngp_rbi_raw_wheel_input_" .. index, state.rawWheelInput[index] or 0.0)
    safeStore("ngp_rbi_wheel_load_" .. index, state.wheelLoad[index] or M.params.wheelLoadRef)
    safeStore("ngp_rbi_wheel_load_norm_" .. index, state.wheelLoadNorm[index] or 1.0)
    safeStore("ngp_rbi_wheel_load_rate_" .. index, state.wheelLoadRate[index] or 0.0)

    if state.debugStoreNow then
        safeStore("ngp_rbi_susp_force_" .. index, state.suspForce[index] or 0.0)
        safeStore("ngp_rbi_damper_force_" .. index, state.damperForce[index] or 0.0)
        safeStore("ngp_rbi_spring_force_" .. index, state.springForce[index] or 0.0)
        safeStore("ngp_rbi_contact_input_" .. index, state.contactInput[index] or 0.0)
        safeStore("ngp_rbi_impulse_" .. index, state.impulse[index] or 0.0)
        safeStore("ngp_rbi_drop_" .. index, state.drop[index] or 0.0)
        safeStore("ngp_rbi_contact_loss_" .. index, state.contactLoss[index] or 0.0)
        safeStore("ngp_rbi_vertical_flow_" .. index, state.verticalFlow[index] or 0.0)
    end
end

local function exportState()
    safeStore("ngp_rbi_status", state.status or "UNKNOWN")
    safeStore("ngp_rbi_update_count", state.updateCount or 0)
    safeStore("ngp_rbi_wheels_valid", state.wheelsValid and 1 or 0)

    safeStore("ngp_body_road_input_front", state.frontInput or 0.0)
    safeStore("ngp_body_road_input_rear", state.rearInput or 0.0)
    safeStore("ngp_body_road_input_left", state.leftInput or 0.0)
    safeStore("ngp_body_road_input_right", state.rightInput or 0.0)

    safeStore("ngp_body_heave_input", state.heaveInput or 0.0)
    safeStore("ngp_body_pitch_input", state.pitchInput or 0.0)
    safeStore("ngp_body_roll_input", state.rollInput or 0.0)
    safeStore("ngp_body_yaw_hint", state.yawHint or 0.0)
    safeStore("ngp_body_road_avg", state.avgWheelInput or 0.0)

    safeStore("ngp_road_body_heave", state.heaveInput or 0.0)
    safeStore("ngp_road_body_pitch", state.pitchInput or 0.0)
    safeStore("ngp_road_body_roll", state.rollInput or 0.0)
    safeStore("ngp_road_body_yaw", state.yawHint or 0.0)

    safeStore("ngp_rbi_heave", state.heaveInput or 0.0)
    safeStore("ngp_rbi_pitch", state.pitchInput or 0.0)
    safeStore("ngp_rbi_roll", state.rollInput or 0.0)
    safeStore("ngp_rbi_yaw", state.yawHint or 0.0)
    safeStore("ngp_rbi_avg_wheel_input", state.avgWheelInput or 0.0)
    safeStore("ngp_rbi_max_wheel_input", state.maxWheelInputLive or 0.0)

    safeStore("ngp_rbi_body_rigidity_linked", state.bodyRigidityLinked and 1 or 0)
    safeStore("ngp_rbi_body_rigidity", state.bodyRigidity or 1.0)
    safeStore("ngp_rbi_body_flex_factor", state.bodyFlexFactor or 1.0)
    safeStore("ngp_rbi_body_heave_scale", state.bodyHeaveScale or 1.0)
    safeStore("ngp_rbi_body_pitch_scale", state.bodyPitchScale or 1.0)
    safeStore("ngp_rbi_body_roll_scale", state.bodyRollScale or 1.0)
    safeStore("ngp_rbi_body_yaw_scale", state.bodyYawScale or 1.0)
    safeStore("ngp_rbi_body_input_tau", state.bodyInputTau or M.params.inputTau)

    for i = 0, 3 do
        exportWheel(i)
    end

    if state.debugStoreNow then
        safeStore("ngp_rbi_load_linked", state.loadLinked and 1 or 0)
        safeStore("ngp_rbi_suspension_linked", state.suspensionLinked and 1 or 0)
        safeStore("ngp_rbi_contact_linked", state.contactLinked and 1 or 0)
        safeStore("ngp_rbi_damper_linked", state.damperLinked and 1 or 0)
        safeStore("ngp_rbi_spring_linked", state.springLinked and 1 or 0)
        safeStore("ngp_rbi_load_path_linked", state.loadPathLinked and 1 or 0)
        safeStore("ngp_rbi_impact_linked", state.impactLinked and 1 or 0)
        safeStore("ngp_rbi_driveline_linked", state.drivelineLinked and 1 or 0)
        safeStore("ngp_rbi_car_linked", state.carLinked and 1 or 0)
        safeStore("ngp_rbi_raw_heave", state.rawHeave or 0.0)
        safeStore("ngp_rbi_raw_pitch", state.rawPitch or 0.0)
        safeStore("ngp_rbi_raw_roll", state.rawRoll or 0.0)
        safeStore("ngp_rbi_raw_yaw", state.rawYaw or 0.0)
    end
end

function M.init()
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1
    dt = clamp(safeNumber(dt, 0.0), M.params.minDt, M.params.maxDt)

    updateDebugGate(dt)
    resetLinkFlags()
    readBodyRigidity()

    car = car or safeGetCar()
    state.carLinked = car ~= nil
    state.wheelsValid = hasWheels(car)

    if not car and not ac then
        state.status = "NO AC"
        decayState(dt)
        exportState()
        return
    end

    if not car then
        state.status = "NO CAR"
    elseif not state.wheelsValid then
        state.status = "NO WHEELS"
    else
        state.status = "RUNNING"
    end

    for i = 0, 3 do
        local wheel = getWheel(car, i)
        readWheelSignals(i, wheel, dt)
        calculateWheelInput(i, dt)
    end

    updateBodyInputs(dt)

    if not car and not (state.loadLinked or state.suspensionLinked or state.contactLinked or state.loadPathLinked) then
        decayState(dt)
    elseif not car then
        state.status = "ROOT ONLY"
    end

    exportState()
end

function M.getHeave()
    return state.heaveInput or 0.0
end

function M.getPitch()
    return state.pitchInput or 0.0
end

function M.getRoll()
    return state.rollInput or 0.0
end

function M.getYawHint()
    return state.yawHint or 0.0
end

function M.getWheelInput(index)
    return state.wheelInput[index] or 0.0
end

function M.getState()
    return state
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0
        return string.format(
            "%s WI %.3f Raw %.3f Load %.0f N %.2f Rate %+.0f\n" ..
            "Susp %+.0f Damp %+.0f Spring %+.0f C %.2f Imp %.2f Drop %.2f",
            tostring(WHEEL_NAMES[i] or i),
            state.wheelInput[i] or 0.0,
            state.rawWheelInput[i] or 0.0,
            state.wheelLoad[i] or 0.0,
            state.wheelLoadNorm[i] or 1.0,
            state.wheelLoadRate[i] or 0.0,
            state.suspForce[i] or 0.0,
            state.damperForce[i] or 0.0,
            state.springForce[i] or 0.0,
            state.contactInput[i] or 0.0,
            state.impulse[i] or 0.0,
            state.drop[i] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "H %.3f P %+.3f R %+.3f Y %+.3f / Avg %.3f Max %.3f\n" ..
        "F %.3f R %.3f L %.3f Rt %.3f\n" ..
        "Body:%s Scales H/P/R/Y %.2f %.2f %.2f %.2f\n" ..
        "Links Load:%s Susp:%s Contact:%s Damper:%s Spring:%s LP:%s Impact:%s Drive:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",
        state.heaveInput or 0.0,
        state.pitchInput or 0.0,
        state.rollInput or 0.0,
        state.yawHint or 0.0,
        state.avgWheelInput or 0.0,
        state.maxWheelInputLive or 0.0,
        state.frontInput or 0.0,
        state.rearInput or 0.0,
        state.leftInput or 0.0,
        state.rightInput or 0.0,
        state.bodyRigidityLinked and "OK" or "NIL",
        state.bodyHeaveScale or 1.0,
        state.bodyPitchScale or 1.0,
        state.bodyRollScale or 1.0,
        state.bodyYawScale or 1.0,
        state.loadLinked and "OK" or "NIL",
        state.suspensionLinked and "OK" or "NIL",
        state.contactLinked and "OK" or "NIL",
        state.damperLinked and "OK" or "NIL",
        state.springLinked and "OK" or "NIL",
        state.loadPathLinked and "OK" or "NIL",
        state.impactLinked and "OK" or "NIL",
        state.drivelineLinked and "OK" or "NIL"
    )
end

function M.drawDebug()
    if not ui then return end
    ui.text("=== ROAD BODY INPUT ===")
    ui.text(M.debugStr())
    for i = 0, 3 do
        ui.text(M.debugStr(i))
    end
end

return M
