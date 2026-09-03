---@diagnostic disable: undefined-global

--============================================================
-- chassis_flex.lua
-- ACNextGen V1.1.5 Stable
-- Virtual Chassis Compliance
--============================================================

local M = {}

M.params = {
    flexTau = 0.120,
    rearTau = 0.180,
    returnTau = 0.220,

    frontCompliance = 0.65,
    rearCompliance = 0.35,

    bodyFrontMix = 0.70,
    bodyRearMix = 0.30,

    roadRollGain  = 0.24,
    roadPitchGain = 0.18,
    roadYawGain   = 0.22,
    heaveGain     = 0.12,

    chassisRollGain  = 0.30,
    chassisPitchGain = 0.22,
    chassisYawGain   = 0.28,

    virtualYawGain   = 0.25,
    virtualRollGain  = 0.18,
    virtualPitchGain = 0.16,

    energyFlexGain = 0.30,
    releaseReturnGain = 0.40,

    maxBodySteer = 1.00,
    maxFlexYaw = 1.00,
    maxFlexRoll = 1.00,
    maxFlexPitch = 1.00,
    maxFlexHeave = 1.00,
    maxDelay = 1.00,

    bodyFlexSteerGain = 0.08,
    bodyFlexYawGain = 0.12,
    bodyFlexRollGain = 0.10,
    bodyBendingPitchGain = 0.08,
    bodyStiffSteerGain = 0.04,
    bodyStiffFlexGain = 0.05,

    bodyTauFlexGain = 0.35,
    bodyTauStiffGain = 0.10,

    minBodyFlexScale = 0.86,
    maxBodyFlexScale = 1.18,

    tireHopFlexGain = 0.08,
    contactLossFlexGain = 0.06,
    drivetrainTwistGain = 0.08,
    brakeFlexGain = 0.04,
    damageFlexGain = 0.14,

    rollRootFlexGain = 0.18,
    pitchRootFlexGain = 0.14,
    heaveRootFlexGain = 0.10,
    yawRootFlexGain = 0.16,

    chassisEnergyDirectGain = 0.20,
    chassisReleaseReturnGain = 0.16,

    sprungPitchFlexGain = 0.12,
    sprungRollFlexGain = 0.12,

    suspensionStressFlexGain = 0.10,
    brakeLockFlexGain = 0.06,
    brakeThermalFlexGain = 0.05,

    bodyRuntimePenaltyFlexGain = 0.18,

    maxRootFlexAdd = 0.35,

    rootReadInterval = 0.050,
    debugStoreInterval = 0.25,
}

local state = {
    steer = 0.0,
    front = 0.0,
    rear = 0.0,
    bodySteer = 0.0,
    delay = 0.0,

    flexYaw = 0.0,
    flexRoll = 0.0,
    flexPitch = 0.0,
    flexHeave = 0.0,

    roadRoll = 0.0,
    roadPitch = 0.0,
    roadHeave = 0.0,
    roadYaw = 0.0,

    chassisRoll = 0.0,
    chassisPitch = 0.0,
    chassisYaw = 0.0,

    virtualYaw = 0.0,
    virtualRoll = 0.0,
    virtualPitch = 0.0,

    chassisEnergy = 0.0,
    chassisRelease = 0.0,

    flexEnergy = 0.0,
    flexRelease = 0.0,

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyBendingFactor = 1.0,
    bodyDampingFactor = 1.0,

    bodySteerScale = 1.0,
    bodyYawScale = 1.0,
    bodyRollScale = 1.0,
    bodyPitchScale = 1.0,
    bodyHeaveScale = 1.0,
    bodyTauScale = 1.0,

    tireHopAvg = 0.0,
    contactLossAvg = 0.0,
    driveTwist = 0.0,
    brakeInput = 0.0,
    brakeLockAvg = 0.0,
    brakeThermalStress = 0.0,
    suspensionStress = 0.0,
    vehicleDamage = 0.0,

    rootRollAdd = 0.0,
    rootPitchAdd = 0.0,
    rootHeaveAdd = 0.0,
    rootYawAdd = 0.0,

    sprungPitchCG = 0.0,
    sprungRollCG = 0.0,
    bodyRuntimePenalty = 0.0,

    rootFlexYawAdd = 0.0,
    rootFlexRollAdd = 0.0,
    rootFlexPitchAdd = 0.0,
    rootFlexHeaveAdd = 0.0,
    rootSteerAdd = 0.0,

    mode = "STABLE",
    status = "INIT",
    updateCount = 0,

    bodyRigidityLinked = false,
    chassisLinked = false,
    virtualLinked = false,
    energyLinked = false,
    tireLinked = false,
    drivetrainLinked = false,
    brakeLinked = false,
    damageLinked = false,
    rollRootLinked = false,
    sprungLinked = false,
    suspensionLinked = false,
    thermalLinked = false,
    bodyRuntimeLinked = false,

    rootReadTimer = 999.0,
    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

local function clamp(v, mn, mx)
    v = tonumber(v) or 0.0
    if v ~= v then return mn end
    if v < mn then return mn end
    if v > mx then return mx end
    return v
end

local function abs(v)
    v = tonumber(v) or 0.0
    return v < 0.0 and -v or v
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
    return tonumber(value) or defaultValue or 0.0
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
    current = tonumber(current) or 0.0
    target = tonumber(target) or 0.0
    tau = math.max(tonumber(tau) or 0.001, 0.0001)
    dt = math.max(tonumber(dt) or 0.0, 0.0)
    local k = clamp(dt / (tau + dt), 0.0, 1.0)
    return current + (target - current) * k
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

local function shouldReadRoots(dt)
    state.rootReadTimer = (state.rootReadTimer or 0.0) + (dt or 0.0)

    if state.rootReadTimer >= M.params.rootReadInterval then
        state.rootReadTimer = 0.0
        return true
    end

    return false
end

local function readBodyRigidity()
    local rigidityRaw = safeLoadRaw("ngp_body_rigidity")
    local flexRaw = safeLoadRaw("ngp_body_flex_factor")
    local torsionRaw = safeLoadRaw("ngp_body_torsion_factor")
    local bendingRaw = safeLoadRaw("ngp_body_bending_factor")
    local dampingRaw = safeLoadRaw("ngp_body_damping_factor")
    local countRaw = safeLoadRaw("ngp_body_rigidity_update_count")

    state.bodyRigidityLinked =
        rigidityRaw ~= nil
        or flexRaw ~= nil
        or torsionRaw ~= nil
        or bendingRaw ~= nil
        or countRaw ~= nil

    state.bodyRigidity = clamp(tonumber(rigidityRaw) or 1.0, 0.20, 1.35)
    state.bodyFlexFactor = clamp(tonumber(flexRaw) or 1.0, 0.25, 1.80)
    state.bodyTorsionFactor = clamp(tonumber(torsionRaw) or 1.0, 0.35, 2.00)
    state.bodyBendingFactor = clamp(tonumber(bendingRaw) or 1.0, 0.35, 2.00)
    state.bodyDampingFactor = clamp(tonumber(dampingRaw) or 1.0, 0.50, 1.80)
end

local function updateBodyScales(dt)
    local p = M.params

    if not state.bodyRigidityLinked then
        state.bodySteerScale = lowPass(state.bodySteerScale, 1.0, 0.080, dt)
        state.bodyYawScale = lowPass(state.bodyYawScale, 1.0, 0.080, dt)
        state.bodyRollScale = lowPass(state.bodyRollScale, 1.0, 0.080, dt)
        state.bodyPitchScale = lowPass(state.bodyPitchScale, 1.0, 0.080, dt)
        state.bodyHeaveScale = lowPass(state.bodyHeaveScale, 1.0, 0.080, dt)
        state.bodyTauScale = lowPass(state.bodyTauScale, 1.0, 0.080, dt)
        return
    end

    local rigidity = clamp(state.bodyRigidity, 0.20, 1.35)
    local flex = clamp(state.bodyFlexFactor, 0.25, 1.80)
    local torsion = clamp(state.bodyTorsionFactor, 0.35, 2.00)
    local bending = clamp(state.bodyBendingFactor, 0.35, 2.00)

    local softness = clamp(flex - 1.0, 0.0, 0.80)
    local stiffness = clamp(rigidity - 1.0, 0.0, 0.35)
    local torsionSoft = clamp(torsion - 1.0, 0.0, 1.0)
    local bendingSoft = clamp(bending - 1.0, 0.0, 1.0)

    state.bodySteerScale = lowPass(
        state.bodySteerScale,
        clamp(1.0 + softness * p.bodyFlexSteerGain - stiffness * p.bodyStiffSteerGain, p.minBodyFlexScale, p.maxBodyFlexScale),
        0.080,
        dt
    )

    state.bodyYawScale = lowPass(
        state.bodyYawScale,
        clamp(1.0 + torsionSoft * p.bodyFlexYawGain + softness * 0.04 - stiffness * p.bodyStiffFlexGain, p.minBodyFlexScale, p.maxBodyFlexScale),
        0.080,
        dt
    )

    state.bodyRollScale = lowPass(
        state.bodyRollScale,
        clamp(1.0 + torsionSoft * p.bodyFlexRollGain + softness * 0.03 - stiffness * p.bodyStiffFlexGain, p.minBodyFlexScale, p.maxBodyFlexScale),
        0.080,
        dt
    )

    state.bodyPitchScale = lowPass(
        state.bodyPitchScale,
        clamp(1.0 + bendingSoft * p.bodyBendingPitchGain + softness * 0.02 - stiffness * p.bodyStiffFlexGain, p.minBodyFlexScale, p.maxBodyFlexScale),
        0.080,
        dt
    )

    state.bodyHeaveScale = lowPass(
        state.bodyHeaveScale,
        clamp(1.0 + softness * 0.06 + bendingSoft * 0.03 - stiffness * 0.03, p.minBodyFlexScale, p.maxBodyFlexScale),
        0.080,
        dt
    )

    state.bodyTauScale = lowPass(
        state.bodyTauScale,
        clamp(1.0 + softness * p.bodyTauFlexGain - stiffness * p.bodyTauStiffGain, 0.90, 1.30),
        0.100,
        dt
    )
end

local function readCarInput(car)
    state.steer = clamp(tonumber(safeField(car, "steer", 0.0)) or 0.0, -1.0, 1.0)
    state.brakeInput = clamp(safeLoad("ngp_brake_input", safeField(car, "brake", 0.0)), 0.0, 1.0)
end

local function readRootInputs()
    local cr = safeLoadRaw("ngp_chassis_roll")
    local cp = safeLoadRaw("ngp_chassis_pitch")
    local cy = safeLoadRaw("ngp_chassis_yaw_hint")

    state.chassisLinked = cr ~= nil or cp ~= nil or cy ~= nil
    state.chassisRoll = clamp(tonumber(cr) or 0.0, -2.0, 2.0)
    state.chassisPitch = clamp(tonumber(cp) or 0.0, -2.0, 2.0)
    state.chassisYaw = clamp(tonumber(cy) or 0.0, -2.0, 2.0)

    local vy = safeLoadRaw("ngp_virtual_yaw")
    local vr = safeLoadRaw("ngp_virtual_roll")
    local vp = safeLoadRaw("ngp_virtual_pitch")

    state.virtualLinked = vy ~= nil or vr ~= nil or vp ~= nil
    state.virtualYaw = clamp(tonumber(vy) or 0.0, -3.0, 3.0)
    state.virtualRoll = clamp(tonumber(vr) or 0.0, -3.0, 3.0)
    state.virtualPitch = clamp(tonumber(vp) or 0.0, -3.0, 3.0)

    local ce = safeLoadRaw("ngp_chassis_energy") or safeLoadRaw("ngp_chassis_body_energy")
    local rel = safeLoadRaw("ngp_chassis_release") or safeLoadRaw("ngp_chassis_body_release")

    state.energyLinked = ce ~= nil or rel ~= nil
    state.chassisEnergy = clamp(tonumber(ce) or 0.0, 0.0, 1.0)
    state.chassisRelease = clamp(tonumber(rel) or 0.0, 0.0, 1.0)

    state.roadRoll = clamp(safeLoad("ngp_body_roll_input", safeLoad("ngp_road_body_roll", 0.0)), -2.0, 2.0)
    state.roadPitch = clamp(safeLoad("ngp_body_pitch_input", safeLoad("ngp_road_body_pitch", 0.0)), -2.0, 2.0)
    state.roadHeave = clamp(safeLoad("ngp_body_heave_input", safeLoad("ngp_road_body_heave", 0.0)), -2.0, 2.0)
    state.roadYaw = clamp(safeLoad("ngp_body_yaw_hint", safeLoad("ngp_road_body_yaw", 0.0)), -2.0, 2.0)

    local hopSum = 0.0
    local lossSum = 0.0
    local tireLinked = false

    for i = 0, 3 do
        local hop = safeLoadRaw("ngp_tire_hop_energy_" .. i) or safeLoadRaw("ngp_tire_hop_" .. i)
        local loss =
            safeLoadRaw("ngp_tire_contact_loss_" .. i)
            or safeLoadRaw("ngp_tcr_contact_loss_" .. i)
            or (1.0 - safeLoad("ngp_contact_quality_" .. i, 1.0))

        if hop ~= nil or loss ~= nil then
            tireLinked = true
        end

        hopSum = hopSum + clamp(tonumber(hop) or 0.0, 0.0, 1.0)
        lossSum = lossSum + clamp(tonumber(loss) or 0.0, 0.0, 1.0)
    end

    state.tireLinked = tireLinked
    state.tireHopAvg = hopSum * 0.25
    state.contactLossAvg = lossSum * 0.25

    local twistRaw =
        safeLoadRaw("ngp_shaft_twist")
        or safeLoadRaw("ngp_driveline_twist")
        or safeLoadRaw("ngp_driveline_windup_twist")

    state.drivetrainLinked = twistRaw ~= nil
    state.driveTwist = clamp(tonumber(twistRaw) or 0.0, -2.0, 2.0)

    local damageRaw =
        safeLoadRaw("ngp_condition_total")
        or safeLoadRaw("ngp_vehicle_condition_total")
        or safeLoadRaw("ngp_damage_chassis")

    state.damageLinked = damageRaw ~= nil
    state.vehicleDamage = clamp(tonumber(damageRaw) or 0.0, 0.0, 1.0)

    local rra = safeLoadRaw("ngp_chassis_roll_root_roll_add")
    local rpa = safeLoadRaw("ngp_chassis_roll_root_pitch_add")
    local rha = safeLoadRaw("ngp_chassis_roll_root_heave_add")
    local rya = safeLoadRaw("ngp_chassis_roll_root_yaw_add")

    state.rollRootLinked = rra ~= nil or rpa ~= nil or rha ~= nil or rya ~= nil
    state.rootRollAdd = clamp(tonumber(rra) or 0.0, -0.35, 0.35)
    state.rootPitchAdd = clamp(tonumber(rpa) or 0.0, -0.35, 0.35)
    state.rootHeaveAdd = clamp(tonumber(rha) or 0.0, -0.35, 0.35)
    state.rootYawAdd = clamp(tonumber(rya) or 0.0, -0.35, 0.35)

    local pcg = safeLoadRaw("ngp_pitch_cg")
    local rcg = safeLoadRaw("ngp_roll_cg")

    state.sprungLinked = pcg ~= nil or rcg ~= nil
    state.sprungPitchCG = clamp(tonumber(pcg) or 0.0, -0.12, 0.12)
    state.sprungRollCG = clamp(tonumber(rcg) or 0.0, -0.12, 0.12)

    local lockSum = 0.0
    local lockLinked = false

    for i = 0, 3 do
        local lockRaw = safeLoadRaw("ngp_brake_lock_" .. i)
        if lockRaw ~= nil then lockLinked = true end
        lockSum = lockSum + clamp(tonumber(lockRaw) or 0.0, 0.0, 1.0)
    end

    state.brakeLockAvg = lockSum * 0.25

    local brakeRaw = safeLoadRaw("ngp_brake_input")
    state.brakeLinked = brakeRaw ~= nil or lockLinked
    state.brakeInput = clamp(tonumber(brakeRaw) or state.brakeInput or 0.0, 0.0, 1.0)

    local tempRaw =
        safeLoadRaw("ngp_brake_temp_max")
        or safeLoadRaw("ngp_brake_max_temp")
        or safeLoadRaw("ngp_brake_temp_avg")
        or safeLoadRaw("ngp_brake_avg_temp")

    state.thermalLinked = tempRaw ~= nil
    state.brakeThermalStress = clamp(((tonumber(tempRaw) or 25.0) - 25.0) / 700.0, 0.0, 1.0)

    local suspSum = 0.0
    local suspLinked = false

    for i = 0, 3 do
        local susp =
            safeLoadRaw("ngp_susp_integrated_force_" .. i)
            or safeLoadRaw("ngp_susp_int_force_" .. i)
            or safeLoadRaw("ngp_susp_" .. i)
            or safeLoadRaw("ngp_damper_force_" .. i)

        if susp ~= nil then
            suspLinked = true
        end

        suspSum = suspSum + math.min(abs(tonumber(susp) or 0.0) / 25000.0, 1.0)
    end

    state.suspensionLinked = suspLinked
    state.suspensionStress = clamp(suspSum * 0.25, 0.0, 1.0)

    local runtimePenaltyRaw = safeLoadRaw("ngp_body_runtime_flex_penalty")
    state.bodyRuntimeLinked = runtimePenaltyRaw ~= nil
    state.bodyRuntimePenalty = clamp(tonumber(runtimePenaltyRaw) or 0.0, 0.0, 0.55)
end

local function updateRootAdds()
    local p = M.params

    state.rootFlexYawAdd =
        clamp(
            state.rootYawAdd * p.yawRootFlexGain
            + state.rootRollAdd * 0.04
            + state.brakeLockAvg * p.brakeLockFlexGain
            + state.bodyRuntimePenalty * p.bodyRuntimePenaltyFlexGain,
            -p.maxRootFlexAdd,
            p.maxRootFlexAdd
        )

    state.rootFlexRollAdd =
        clamp(
            state.rootRollAdd * p.rollRootFlexGain
            + state.sprungRollCG * p.sprungRollFlexGain
            + state.suspensionStress * p.suspensionStressFlexGain
            + state.brakeThermalStress * p.brakeThermalFlexGain,
            -p.maxRootFlexAdd,
            p.maxRootFlexAdd
        )

    state.rootFlexPitchAdd =
        clamp(
            state.rootPitchAdd * p.pitchRootFlexGain
            + state.sprungPitchCG * p.sprungPitchFlexGain
            + state.brakeInput * p.brakeFlexGain
            + state.brakeLockAvg * p.brakeLockFlexGain
            + state.brakeThermalStress * p.brakeThermalFlexGain,
            -p.maxRootFlexAdd,
            p.maxRootFlexAdd
        )

    state.rootFlexHeaveAdd =
        clamp(
            state.rootHeaveAdd * p.heaveRootFlexGain
            + state.suspensionStress * p.suspensionStressFlexGain
            + state.tireHopAvg * p.tireHopFlexGain,
            0.0,
            p.maxRootFlexAdd
        )

    state.rootSteerAdd =
        clamp(
            state.rootYawAdd * 0.08
            + state.rootRollAdd * 0.04,
            -p.maxRootFlexAdd,
            p.maxRootFlexAdd
        )
end

local function updateMode()
    if state.flexEnergy < 0.05 then
        state.mode = "STABLE"
    elseif state.flexRelease > 0.08 then
        state.mode = "RELEASE"
    elseif state.flexEnergy < 0.30 then
        state.mode = "FLEX"
    else
        state.mode = "LOADED"
    end
end

local function exportState()
    safeStore("ngp_body_steer", state.bodySteer)

    safeStore("ngp_chassis_flex_yaw", state.flexYaw)
    safeStore("ngp_chassis_flex_roll", state.flexRoll)
    safeStore("ngp_chassis_flex_pitch", state.flexPitch)
    safeStore("ngp_chassis_flex_heave", state.flexHeave)

    safeStore("ngp_chassis_flex_delay", state.delay)
    safeStore("ngp_chassis_flex_energy", state.flexEnergy)
    safeStore("ngp_chassis_flex_release", state.flexRelease)

    safeStore("ngp_body_flex_yaw", state.flexYaw)
    safeStore("ngp_body_flex_roll", state.flexRoll)
    safeStore("ngp_body_flex_pitch", state.flexPitch)
    safeStore("ngp_body_flex_heave", state.flexHeave)

    safeStore("ngp_flex_energy", state.flexEnergy)
    safeStore("ngp_flex_release", state.flexRelease)

    safeStore("ngp_chassis_flex_status", state.status)
    safeStore("ngp_chassis_flex_update_count", state.updateCount)
    safeStore("ngp_chassis_flex_mode", state.mode)

    safeStore("ngp_chassis_flex_root_yaw_add", state.rootFlexYawAdd)
    safeStore("ngp_chassis_flex_root_roll_add", state.rootFlexRollAdd)
    safeStore("ngp_chassis_flex_root_pitch_add", state.rootFlexPitchAdd)
    safeStore("ngp_chassis_flex_root_heave_add", state.rootFlexHeaveAdd)
    safeStore("ngp_chassis_flex_root_steer_add", state.rootSteerAdd)

    safeStore("ngp_chassis_flex_body_tau_scale", state.bodyTauScale)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_chassis_flex_body_linked", state.bodyRigidityLinked and 1 or 0)
    safeStore("ngp_chassis_flex_chassis_linked", state.chassisLinked and 1 or 0)
    safeStore("ngp_chassis_flex_virtual_linked", state.virtualLinked and 1 or 0)
    safeStore("ngp_chassis_flex_energy_linked", state.energyLinked and 1 or 0)
    safeStore("ngp_chassis_flex_tire_linked", state.tireLinked and 1 or 0)
    safeStore("ngp_chassis_flex_drive_linked", state.drivetrainLinked and 1 or 0)
    safeStore("ngp_chassis_flex_brake_linked", state.brakeLinked and 1 or 0)
    safeStore("ngp_chassis_flex_damage_linked", state.damageLinked and 1 or 0)
    safeStore("ngp_chassis_flex_roll_root_linked", state.rollRootLinked and 1 or 0)
    safeStore("ngp_chassis_flex_sprung_linked", state.sprungLinked and 1 or 0)
    safeStore("ngp_chassis_flex_suspension_linked", state.suspensionLinked and 1 or 0)
    safeStore("ngp_chassis_flex_thermal_linked", state.thermalLinked and 1 or 0)
    safeStore("ngp_chassis_flex_body_runtime_linked", state.bodyRuntimeLinked and 1 or 0)

    safeStore("ngp_chassis_flex_brake_lock_avg", state.brakeLockAvg)
    safeStore("ngp_chassis_flex_thermal_stress", state.brakeThermalStress)
    safeStore("ngp_chassis_flex_suspension_stress", state.suspensionStress)
    safeStore("ngp_chassis_flex_body_runtime_penalty", state.bodyRuntimePenalty)

    safeStore("ngp_chassis_flex_body_steer_scale", state.bodySteerScale)
    safeStore("ngp_chassis_flex_body_yaw_scale", state.bodyYawScale)
    safeStore("ngp_chassis_flex_body_roll_scale", state.bodyRollScale)
    safeStore("ngp_chassis_flex_body_pitch_scale", state.bodyPitchScale)
    safeStore("ngp_chassis_flex_body_heave_scale", state.bodyHeaveScale)
end

function M.init()
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1
    dt = tonumber(dt) or 0.0

    if dt <= 0.0 then
        state.status = "BAD DT"
        exportState()
        return
    end

    updateDebugGate(dt)

    if not car and ac and ac.getCar then
        local ok, result = pcall(function()
            return ac.getCar(0)
        end)

        if ok then
            car = result
        end
    end

    if not car then
        state.status = "NO CAR"
        readBodyRigidity()

        if shouldReadRoots(dt) then
            readRootInputs()
            updateRootAdds()
        end

        updateBodyScales(dt)
        updateMode()
        exportState()
        return
    end

    state.status = "RUNNING"

    readCarInput(car)
    readBodyRigidity()

    if shouldReadRoots(dt) then
        readRootInputs()
        updateRootAdds()
    end

    updateBodyScales(dt)

    local p = M.params
    local damageScale = 1.0 + state.vehicleDamage * p.damageFlexGain
    local tauScale = state.bodyTauScale or 1.0

    local targetFront =
        (
            state.steer * p.frontCompliance
            + state.rootSteerAdd
            + state.roadYaw * p.roadYawGain
            + state.chassisYaw * p.chassisYawGain
            + state.virtualYaw * p.virtualYawGain
            + state.rootFlexYawAdd
        )
        * state.bodySteerScale
        * damageScale

    local targetRear =
        (
            state.roadYaw * p.roadYawGain
            + state.chassisYaw * p.chassisYawGain
            + state.virtualYaw * p.virtualYawGain
            + state.rootFlexYawAdd
            + state.driveTwist * p.drivetrainTwistGain
        )
        * p.rearCompliance
        * state.bodyYawScale
        * damageScale

    state.front = lowPass(state.front, targetFront, p.flexTau * tauScale, dt)
    state.rear = lowPass(state.rear, targetRear, p.rearTau * tauScale, dt)

    state.bodySteer =
        clamp(
            state.front * p.bodyFrontMix
            + state.rear * p.bodyRearMix,
            -p.maxBodySteer,
            p.maxBodySteer
        )

    local targetYaw =
        (
            state.roadYaw * p.roadYawGain
            + state.chassisYaw * p.chassisYawGain
            + state.virtualYaw * p.virtualYawGain
            + state.contactLossAvg * p.contactLossFlexGain
            + state.rootFlexYawAdd
        )
        * state.bodyYawScale
        * damageScale

    local targetRoll =
        (
            state.roadRoll * p.roadRollGain
            + state.chassisRoll * p.chassisRollGain
            + state.virtualRoll * p.virtualRollGain
            + state.tireHopAvg * p.tireHopFlexGain
            + state.rootFlexRollAdd
        )
        * state.bodyRollScale
        * damageScale

    local targetPitch =
        (
            state.roadPitch * p.roadPitchGain
            + state.chassisPitch * p.chassisPitchGain
            + state.virtualPitch * p.virtualPitchGain
            + state.brakeInput * p.brakeFlexGain
            + state.rootFlexPitchAdd
        )
        * state.bodyPitchScale
        * damageScale

    local targetHeave =
        (
            state.roadHeave * p.heaveGain
            + state.tireHopAvg * p.tireHopFlexGain
            + state.rootFlexHeaveAdd
        )
        * state.bodyHeaveScale
        * damageScale

    state.flexYaw = lowPass(state.flexYaw, clamp(targetYaw, -p.maxFlexYaw, p.maxFlexYaw), p.flexTau * tauScale, dt)
    state.flexRoll = lowPass(state.flexRoll, clamp(targetRoll, -p.maxFlexRoll, p.maxFlexRoll), p.flexTau * tauScale, dt)
    state.flexPitch = lowPass(state.flexPitch, clamp(targetPitch, -p.maxFlexPitch, p.maxFlexPitch), p.flexTau * tauScale, dt)
    state.flexHeave = lowPass(state.flexHeave, clamp(targetHeave, -p.maxFlexHeave, p.maxFlexHeave), p.flexTau * tauScale, dt)

    local flexTarget =
        clamp(
            abs(state.bodySteer)
            + abs(state.flexYaw)
            + abs(state.flexRoll)
            + abs(state.flexPitch)
            + abs(state.flexHeave)
            + state.chassisEnergy * p.energyFlexGain
            + state.chassisEnergy * p.chassisEnergyDirectGain
            + state.bodyRuntimePenalty * p.bodyRuntimePenaltyFlexGain
            + state.suspensionStress * p.suspensionStressFlexGain
            + state.brakeLockAvg * p.brakeLockFlexGain
            + state.brakeThermalStress * p.brakeThermalFlexGain,
            0.0,
            1.0
        )

    state.flexEnergy =
        lowPass(
            state.flexEnergy,
            flexTarget,
            p.flexTau * tauScale,
            dt
        )

    local releaseTarget =
        math.max(
            state.chassisRelease * p.releaseReturnGain
            + state.chassisRelease * p.chassisReleaseReturnGain
            + (state.flexEnergy - flexTarget),
            0.0
        )

    state.flexRelease =
        lowPass(
            state.flexRelease,
            clamp(releaseTarget, 0.0, 1.0),
            p.returnTau,
            dt
        )

    state.delay =
        clamp(
            state.flexEnergy
            - state.flexRelease * p.releaseReturnGain,
            0.0,
            p.maxDelay
        )

    updateMode()
    exportState()
end

function M.getBodySteer()
    return state.bodySteer or 0.0
end

function M.getFlexYaw()
    return state.flexYaw or 0.0
end

function M.getFlexRoll()
    return state.flexRoll or 0.0
end

function M.getFlexPitch()
    return state.flexPitch or 0.0
end

function M.getEnergy()
    return state.flexEnergy or 0.0
end

function M.getRelease()
    return state.flexRelease or 0.0
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f Mode %s\n" ..
        "BodySteer %.3f Delay %.3f Energy %.3f Release %.3f\n" ..
        "Flex Y/R/P/H %.3f %.3f %.3f %.3f\n" ..
        "RootAdd Y/R/P/H/S %.3f %.3f %.3f %.3f %.3f\n" ..
        "Links Body:%s Chassis:%s Virtual:%s Energy:%s Tire:%s Drive:%s Brake:%s Damage:%s RollRoot:%s Sprung:%s Susp:%s Therm:%s BodyRun:%s",
        tostring(state.status),
        state.updateCount or 0,
        tostring(state.mode),
        state.bodySteer or 0.0,
        state.delay or 0.0,
        state.flexEnergy or 0.0,
        state.flexRelease or 0.0,
        state.flexYaw or 0.0,
        state.flexRoll or 0.0,
        state.flexPitch or 0.0,
        state.flexHeave or 0.0,
        state.rootFlexYawAdd or 0.0,
        state.rootFlexRollAdd or 0.0,
        state.rootFlexPitchAdd or 0.0,
        state.rootFlexHeaveAdd or 0.0,
        state.rootSteerAdd or 0.0,
        state.bodyRigidityLinked and "OK" or "NIL",
        state.chassisLinked and "OK" or "NIL",
        state.virtualLinked and "OK" or "NIL",
        state.energyLinked and "OK" or "NIL",
        state.tireLinked and "OK" or "NIL",
        state.drivetrainLinked and "OK" or "NIL",
        state.brakeLinked and "OK" or "NIL",
        state.damageLinked and "OK" or "NIL",
        state.rollRootLinked and "OK" or "NIL",
        state.sprungLinked and "OK" or "NIL",
        state.suspensionLinked and "OK" or "NIL",
        state.thermalLinked and "OK" or "NIL",
        state.bodyRuntimeLinked and "OK" or "NIL"
    )
end

return M
