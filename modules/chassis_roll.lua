---@diagnostic disable: undefined-global

--============================================================
-- chassis_roll.lua
-- ACNextGen V1.1 Stable
-- Phase 8.1
-- Chassis Roll / Pitch / Heave Response
--============================================================

local M = {}

M.params = {
    rollTau  = 0.140,
    pitchTau = 0.160,
    heaveTau = 0.120,
    yawTau   = 0.180,

    rollReturnTau  = 0.220,
    pitchReturnTau = 0.240,
    heaveReturnTau = 0.180,
    yawReturnTau   = 0.260,

    rollGain  = 0.85,
    pitchGain = 0.75,
    heaveGain = 0.65,
    yawGain   = 0.35,

    loadRollGain  = 0.22,
    loadPitchGain = 0.18,

    maxRoll  = 1.0,
    maxPitch = 1.0,
    maxHeave = 1.0,
    maxYaw   = 1.0,

    energyGain = 0.70,
    releaseGain = 0.45,

    bodyFlexRollLoss = 0.10,
    bodyFlexPitchLoss = 0.08,
    bodyFlexHeaveLoss = 0.06,
    bodyFlexYawLoss = 0.08,
    bodyTorsionRollLoss = 0.08,
    bodyBendingPitchLoss = 0.08,
    bodyStiffGain = 0.05,

    bodyFlexTauGain = 0.42,
    bodyStiffTauGain = 0.10,

    minBodyChassisScale = 0.86,
    maxBodyChassisScale = 1.10,

    tireHopHeaveGain = 0.18,
    contactLossInputGain = 0.08,
    brakePitchGain = 0.12,
    drivePitchGain = 0.08,
    damageResponseLoss = 0.12,

    virtualYawInputGain = 0.10,
    virtualRollInputGain = 0.08,
    virtualPitchInputGain = 0.07,

    sprungPitchGain = 0.65,
    sprungRollGain = 0.65,

    brakeLockPitchGain = 0.05,
    brakeThermalResponseLoss = 0.05,

    suspensionStressGain = 0.06,

    maxRootInputAdd = 0.25,

    rootReadInterval = 0.050,
    debugStoreInterval = 0.25,
}

local state = {
    roll = 0.0,
    pitch = 0.0,
    heave = 0.0,
    yawHint = 0.0,

    inputRoll = 0.0,
    inputPitch = 0.0,
    inputHeave = 0.0,
    inputYaw = 0.0,

    loadRoll = 0.0,
    loadPitch = 0.0,

    bodyEnergy = 0.0,
    bodyRelease = 0.0,

    rollState = "NEUTRAL",
    pitchState = "NEUTRAL",
    heaveState = "NEUTRAL",
    transition = "STABLE",

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyBendingFactor = 1.0,
    bodyDampingFactor = 1.0,

    bodyRollScale = 1.0,
    bodyPitchScale = 1.0,
    bodyHeaveScale = 1.0,
    bodyYawScale = 1.0,

    bodyRollTau = 0.140,
    bodyPitchTau = 0.160,
    bodyHeaveTau = 0.120,
    bodyYawTau = 0.180,

    tireHopAvg = 0.0,
    contactLossAvg = 0.0,
    brakeInput = 0.0,
    brakeLockAvg = 0.0,
    brakeThermalStress = 0.0,

    driveTorque = 0.0,
    driveTorqueNorm = 0.0,

    vehicleDamage = 0.0,

    virtualYaw = 0.0,
    virtualRoll = 0.0,
    virtualPitch = 0.0,

    sprungPitchCG = 0.0,
    sprungRollCG = 0.0,

    suspensionStress = 0.0,

    rootRollAdd = 0.0,
    rootPitchAdd = 0.0,
    rootHeaveAdd = 0.0,
    rootYawAdd = 0.0,

    status = "INIT",
    updateCount = 0,

    bodyRigidityLinked = false,
    loadLinked = false,
    roadLinked = false,
    tireLinked = false,
    brakeLinked = false,
    drivetrainLinked = false,
    damageLinked = false,
    virtualLinked = false,
    sprungLinked = false,
    suspensionLinked = false,
    thermalLinked = false,

    rootReadTimer = 999.0,
    rootReadNow = true,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

local function clamp(v, mn, mx)
    v = tonumber(v) or 0.0
    if v ~= v then
        return mn
    end
    if v < mn then
        return mn
    end
    if v > mx then
        return mx
    end
    return v
end

local function abs(v)
    v = tonumber(v) or 0.0
    return v < 0.0 and -v or v
end

local function safeNumber(v, d)
    local n = tonumber(v)
    if n == nil or n ~= n then
        return d or 0.0
    end
    return n
end

local function safeLoadRaw(k)
    if not ac or not ac.load then
        return nil
    end

    local ok, v = pcall(function()
        return ac.load(k)
    end)

    if not ok then
        return nil
    end

    return v
end

local function safeLoad(k, d)
    local v = safeLoadRaw(k)
    if v == nil then
        return d or 0.0
    end
    return safeNumber(v, d or 0.0)
end

local function safeStore(k, v)
    if not ac or not ac.store then
        return
    end

    pcall(function()
        ac.store(k, v)
    end)
end

local function safeField(o, f, d)
    if not o then
        return d
    end

    local ok, v = pcall(function()
        return o[f]
    end)

    if not ok or v == nil then
        return d
    end

    return v
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

local function lowPass(cur, tgt, tau, dt)
    cur = safeNumber(cur, 0.0)
    tgt = safeNumber(tgt, 0.0)
    tau = safeNumber(tau, 0.001)
    dt = safeNumber(dt, 0.0)

    if tau <= 0.0001 then
        return tgt
    end

    local k = clamp(dt / (tau + dt), 0.0, 1.0)
    return cur + (tgt - cur) * k
end

local function updateGates(dt)
    dt = safeNumber(dt, 0.0)

    state.rootReadTimer = (state.rootReadTimer or 0.0) + dt
    if state.rootReadTimer >= M.params.rootReadInterval then
        state.rootReadTimer = 0.0
        state.rootReadNow = true
    else
        state.rootReadNow = false
    end

    state.debugStoreTimer = (state.debugStoreTimer or 0.0) + dt
    if state.debugStoreTimer >= M.params.debugStoreInterval then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    else
        state.debugStoreNow = false
    end
end

local function readBodyRigidity()
    state.bodyRigidity = clamp(safeLoad("ngp_body_rigidity", 1.0), 0.20, 1.35)
    state.bodyFlexFactor = clamp(safeLoad("ngp_body_flex_factor", 1.0), 0.25, 1.80)
    state.bodyTorsionFactor = clamp(safeLoad("ngp_body_torsion_factor", 1.0), 0.35, 2.00)
    state.bodyBendingFactor = clamp(safeLoad("ngp_body_bending_factor", 1.0), 0.35, 2.00)
    state.bodyDampingFactor = clamp(safeLoad("ngp_body_damping_factor", 1.0), 0.50, 1.80)
    state.bodyRigidityLinked = safeLoad("ngp_body_rigidity_update_count", 0.0) > 0.0
end

local function applyBodyRigidity(roll, pitch, heave, yawHint, dt)
    local p = M.params

    if not state.bodyRigidityLinked then
        state.bodyRollScale = lowPass(state.bodyRollScale, 1.0, 0.050, dt)
        state.bodyPitchScale = lowPass(state.bodyPitchScale, 1.0, 0.050, dt)
        state.bodyHeaveScale = lowPass(state.bodyHeaveScale, 1.0, 0.050, dt)
        state.bodyYawScale = lowPass(state.bodyYawScale, 1.0, 0.050, dt)

        state.bodyRollTau = p.rollTau
        state.bodyPitchTau = p.pitchTau
        state.bodyHeaveTau = p.heaveTau
        state.bodyYawTau = p.yawTau

        return roll, pitch, heave, yawHint
    end

    local rigidity = clamp(state.bodyRigidity, 0.20, 1.35)
    local flex = clamp(state.bodyFlexFactor, 0.25, 1.80)
    local torsion = clamp(state.bodyTorsionFactor, 0.35, 2.00)
    local bending = clamp(state.bodyBendingFactor, 0.35, 2.00)
    local damping = clamp(state.bodyDampingFactor, 0.50, 1.80)

    local softness = clamp(flex - 1.0, 0.0, 0.80)
    local stiffness = clamp(rigidity - 1.0, 0.0, 0.35)
    local torsionSoft = clamp(torsion - 1.0, 0.0, 1.0)
    local bendingSoft = clamp(bending - 1.0, 0.0, 1.0)

    state.bodyRollScale = lowPass(
        state.bodyRollScale,
        clamp(
            1.0
            - softness * p.bodyFlexRollLoss
            - torsionSoft * p.bodyTorsionRollLoss
            + stiffness * p.bodyStiffGain,
            p.minBodyChassisScale,
            p.maxBodyChassisScale
        ),
        0.050,
        dt
    )

    state.bodyPitchScale = lowPass(
        state.bodyPitchScale,
        clamp(
            1.0
            - softness * p.bodyFlexPitchLoss
            - bendingSoft * p.bodyBendingPitchLoss
            + stiffness * p.bodyStiffGain,
            p.minBodyChassisScale,
            p.maxBodyChassisScale
        ),
        0.050,
        dt
    )

    state.bodyHeaveScale = lowPass(
        state.bodyHeaveScale,
        clamp(
            1.0
            - softness * p.bodyFlexHeaveLoss
            + stiffness * p.bodyStiffGain,
            p.minBodyChassisScale,
            p.maxBodyChassisScale
        ),
        0.050,
        dt
    )

    state.bodyYawScale = lowPass(
        state.bodyYawScale,
        clamp(
            1.0
            - softness * p.bodyFlexYawLoss
            - torsionSoft * p.bodyTorsionRollLoss * 0.75
            + stiffness * p.bodyStiffGain,
            p.minBodyChassisScale,
            p.maxBodyChassisScale
        ),
        0.050,
        dt
    )

    local tauMul =
        (1.0 + softness * p.bodyFlexTauGain)
        /
        (1.0 + stiffness * p.bodyStiffTauGain)

    tauMul = tauMul / math.max(damping, 0.50)

    state.bodyRollTau = clamp(p.rollTau * tauMul, 0.030, 0.180)
    state.bodyPitchTau = clamp(p.pitchTau * tauMul, 0.035, 0.200)
    state.bodyHeaveTau = clamp(p.heaveTau * tauMul, 0.040, 0.180)
    state.bodyYawTau = clamp(p.yawTau * tauMul, 0.040, 0.220)

    return
        roll * state.bodyRollScale,
        pitch * state.bodyPitchScale,
        heave * state.bodyHeaveScale,
        yawHint * state.bodyYawScale
end

local function resetLinkFlags()
    state.bodyRigidityLinked = false
    state.loadLinked = false
    state.roadLinked = false
    state.tireLinked = false
    state.brakeLinked = false
    state.drivetrainLinked = false
    state.damageLinked = false
    state.virtualLinked = false
    state.sprungLinked = false
    state.suspensionLinked = false
    state.thermalLinked = false
end

local function readRootInputs(car)
    local roadRollRaw = safeLoadRaw("ngp_body_roll_input") or safeLoadRaw("ngp_road_body_roll")
    local roadPitchRaw = safeLoadRaw("ngp_body_pitch_input") or safeLoadRaw("ngp_road_body_pitch")
    local roadHeaveRaw = safeLoadRaw("ngp_body_heave_input") or safeLoadRaw("ngp_road_body_heave")
    local roadYawRaw = safeLoadRaw("ngp_body_yaw_hint") or safeLoadRaw("ngp_road_body_yaw")

    state.roadLinked =
        roadRollRaw ~= nil
        or roadPitchRaw ~= nil
        or roadHeaveRaw ~= nil
        or roadYawRaw ~= nil

    state.inputRoll = safeLoad("ngp_body_roll_input", safeLoad("ngp_road_body_roll", 0.0))
    state.inputPitch = safeLoad("ngp_body_pitch_input", safeLoad("ngp_road_body_pitch", 0.0))
    state.inputHeave = safeLoad("ngp_body_heave_input", safeLoad("ngp_road_body_heave", 0.0))
    state.inputYaw = safeLoad("ngp_body_yaw_hint", safeLoad("ngp_road_body_yaw", 0.0))

    local loadLeft = safeLoadRaw("ngp_load_left") or safeLoadRaw("ngp_load_abs_left")
    local loadRight = safeLoadRaw("ngp_load_right") or safeLoadRaw("ngp_load_abs_right")
    local loadFront = safeLoadRaw("ngp_load_front") or safeLoadRaw("ngp_load_abs_front")
    local loadRear = safeLoadRaw("ngp_load_rear") or safeLoadRaw("ngp_load_abs_rear")

    state.loadLinked =
        loadLeft ~= nil
        or loadRight ~= nil
        or loadFront ~= nil
        or loadRear ~= nil

    state.loadRoll = clamp(
        safeLoad("ngp_load_right", safeLoad("ngp_load_abs_right", 0.5))
        -
        safeLoad("ngp_load_left", safeLoad("ngp_load_abs_left", 0.5)),
        -1.0,
        1.0
    )

    state.loadPitch = clamp(
        safeLoad("ngp_load_front", safeLoad("ngp_load_abs_front", 0.5))
        -
        safeLoad("ngp_load_rear", safeLoad("ngp_load_abs_rear", 0.5)),
        -1.0,
        1.0
    )

    local hopSum = 0.0
    local lossSum = 0.0
    local tireLinked = false

    for i = 0, 3 do
        local hop =
            safeLoadRaw("ngp_tire_hop_energy_" .. i)
            or
            safeLoadRaw("ngp_tire_hop_" .. i)

        local loss =
            safeLoadRaw("ngp_tire_contact_loss_" .. i)
            or
            safeLoadRaw("ngp_tcr_contact_loss_" .. i)

        if hop ~= nil or loss ~= nil then
            tireLinked = true
        end

        hopSum = hopSum + clamp(safeNumber(hop, 0.0), 0.0, 1.0)
        lossSum = lossSum + clamp(safeNumber(loss, 0.0), 0.0, 1.0)
    end

    state.tireLinked = tireLinked
    state.tireHopAvg = hopSum * 0.25
    state.contactLossAvg = lossSum * 0.25

    local brakeRaw =
        safeLoadRaw("ngp_brake_input_smoothed")
        or
        safeLoadRaw("ngp_brake_input")

    state.brakeLinked = brakeRaw ~= nil
    state.brakeInput = clamp(
        safeNumber(
            brakeRaw,
            safeNumber(safeField(car, "brake", 0.0), 0.0)
        ),
        0.0,
        1.0
    )

    local driveRaw =
        safeLoadRaw("ngp_drive_torque")
        or
        safeLoadRaw("ngp_drivetrain_torque")
        or
        safeLoadRaw("ngp_shaft_torque")

    state.drivetrainLinked = driveRaw ~= nil
    state.driveTorque = safeNumber(driveRaw, 0.0)

    local damageRaw =
        safeLoadRaw("ngp_condition_total")
        or
        safeLoadRaw("ngp_vehicle_condition_total")

    state.damageLinked = damageRaw ~= nil
    state.vehicleDamage = clamp(safeNumber(damageRaw, 0.0), 0.0, 1.0)

    local vy = safeLoadRaw("ngp_virtual_yaw")
    local vr = safeLoadRaw("ngp_virtual_roll")
    local vp = safeLoadRaw("ngp_virtual_pitch")

    state.virtualLinked = vy ~= nil or vr ~= nil or vp ~= nil
    state.virtualYaw = clamp(safeNumber(vy, 0.0), -2.5, 2.5)
    state.virtualRoll = clamp(safeNumber(vr, 0.0), -2.5, 2.5)
    state.virtualPitch = clamp(safeNumber(vp, 0.0), -2.5, 2.5)

    local pcg = safeLoadRaw("ngp_pitch_cg")
    local rcg = safeLoadRaw("ngp_roll_cg")

    state.sprungLinked = pcg ~= nil or rcg ~= nil
    state.sprungPitchCG = clamp(safeNumber(pcg, 0.0), -0.12, 0.12)
    state.sprungRollCG = clamp(safeNumber(rcg, 0.0), -0.12, 0.12)

    local lockSum = 0.0
    local lockLinked = false

    for i = 0, 3 do
        local lockRaw = safeLoadRaw("ngp_brake_lock_" .. i)

        if lockRaw ~= nil then
            lockLinked = true
        end

        lockSum = lockSum + clamp(safeNumber(lockRaw, 0.0), 0.0, 1.0)
    end

    state.brakeLockAvg = lockSum * 0.25
    state.brakeLinked = state.brakeLinked or lockLinked

    local tempRaw =
        safeLoadRaw("ngp_brake_temp_max")
        or
        safeLoadRaw("ngp_brake_max_temp")
        or
        safeLoadRaw("ngp_brake_temp_avg")
        or
        safeLoadRaw("ngp_brake_avg_temp")

    state.thermalLinked = tempRaw ~= nil
    state.brakeThermalStress =
        clamp(
            (safeNumber(tempRaw, 25.0) - 25.0) / 700.0,
            0.0,
            1.0
        )

    local suspSum = 0.0
    local suspLinked = false

    for i = 0, 3 do
        local susp =
            safeLoadRaw("ngp_susp_integrated_force_" .. i)
            or
            safeLoadRaw("ngp_susp_int_force_" .. i)
            or
            safeLoadRaw("ngp_susp_" .. i)
            or
            safeLoadRaw("ngp_damper_force_" .. i)

        if susp ~= nil then
            suspLinked = true
        end

        suspSum = suspSum + math.min(abs(safeNumber(susp, 0.0)) / 25000.0, 1.0)
    end

    state.suspensionLinked = suspLinked
    state.suspensionStress = clamp(suspSum * 0.25, 0.0, 1.0)

    state.driveTorqueNorm = clamp(abs(state.driveTorque) / 2500.0, 0.0, 1.0)

    state.rootRollAdd =
        clamp(
            state.virtualRoll * M.params.virtualRollInputGain
            +
            state.sprungRollCG * M.params.sprungRollGain
            +
            state.suspensionStress * M.params.suspensionStressGain,
            -M.params.maxRootInputAdd,
            M.params.maxRootInputAdd
        )

    state.rootPitchAdd =
        clamp(
            state.virtualPitch * M.params.virtualPitchInputGain
            +
            state.sprungPitchCG * M.params.sprungPitchGain
            +
            state.brakeLockAvg * M.params.brakeLockPitchGain,
            -M.params.maxRootInputAdd,
            M.params.maxRootInputAdd
        )

    state.rootHeaveAdd =
        clamp(
            state.suspensionStress * M.params.suspensionStressGain
            +
            state.tireHopAvg * 0.04,
            0.0,
            M.params.maxRootInputAdd
        )

    state.rootYawAdd =
        clamp(
            state.virtualYaw * M.params.virtualYawInputGain
            -
            state.driveTorqueNorm * 0.04,
            -M.params.maxRootInputAdd,
            M.params.maxRootInputAdd
        )
end

local function updatePerception()
    state.rollState =
        abs(state.roll) < 0.05
        and
        "NEUTRAL"
        or
        (
            state.roll > 0.0
            and
            "ROLL RIGHT"
            or
            "ROLL LEFT"
        )

    state.pitchState =
        abs(state.pitch) < 0.05
        and
        "NEUTRAL"
        or
        (
            state.pitch > 0.0
            and
            "NOSE DOWN"
            or
            "NOSE UP"
        )

    state.heaveState =
        abs(state.heave) < 0.05
        and
        "NEUTRAL"
        or
        (
            state.heave > 0.0
            and
            "HEAVE UP"
            or
            "HEAVE DOWN"
        )

    local energy =
        clamp(
            abs(state.roll)
            +
            abs(state.pitch)
            +
            abs(state.heave)
            +
            abs(state.yawHint),
            0.0,
            2.0
        )

    if energy < 0.15 then
        state.transition = "STABLE"
    elseif energy < 0.45 then
        state.transition = "LOAD SHIFT"
    else
        state.transition = "BODY TRANSITION"
    end
end

local function exportState()
    safeStore("ngp_chassis_roll", state.roll)
    safeStore("ngp_chassis_pitch", state.pitch)
    safeStore("ngp_chassis_heave", state.heave)
    safeStore("ngp_chassis_yaw_hint", state.yawHint)

    safeStore("ngp_body_roll", state.roll)
    safeStore("ngp_body_pitch", state.pitch)
    safeStore("ngp_body_heave", state.heave)
    safeStore("ngp_body_yaw_hint_live", state.yawHint)

    safeStore("ngp_chassis_body_energy", state.bodyEnergy)
    safeStore("ngp_chassis_body_release", state.bodyRelease)

    safeStore("ngp_chassis_roll_state", state.rollState)
    safeStore("ngp_chassis_pitch_state", state.pitchState)
    safeStore("ngp_chassis_heave_state", state.heaveState)
    safeStore("ngp_chassis_transition", state.transition)

    safeStore("ngp_chassis_roll_status", state.status)
    safeStore("ngp_chassis_roll_update_count", state.updateCount)

    safeStore("ngp_chassis_status", state.status)
    safeStore("ngp_chassis_update_count", state.updateCount)

    safeStore("ngp_chassis_roll_root_roll_add", state.rootRollAdd)
    safeStore("ngp_chassis_roll_root_pitch_add", state.rootPitchAdd)
    safeStore("ngp_chassis_roll_root_heave_add", state.rootHeaveAdd)
    safeStore("ngp_chassis_roll_root_yaw_add", state.rootYawAdd)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_chassis_roll_body_linked", state.bodyRigidityLinked and 1 or 0)
    safeStore("ngp_chassis_roll_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_chassis_roll_road_linked", state.roadLinked and 1 or 0)
    safeStore("ngp_chassis_roll_tire_linked", state.tireLinked and 1 or 0)
    safeStore("ngp_chassis_roll_brake_linked", state.brakeLinked and 1 or 0)
    safeStore("ngp_chassis_roll_drive_linked", state.drivetrainLinked and 1 or 0)
    safeStore("ngp_chassis_roll_damage_linked", state.damageLinked and 1 or 0)
    safeStore("ngp_chassis_roll_virtual_linked", state.virtualLinked and 1 or 0)
    safeStore("ngp_chassis_roll_sprung_linked", state.sprungLinked and 1 or 0)
    safeStore("ngp_chassis_roll_suspension_linked", state.suspensionLinked and 1 or 0)
    safeStore("ngp_chassis_roll_thermal_linked", state.thermalLinked and 1 or 0)

    safeStore("ngp_chassis_roll_body_roll_scale", state.bodyRollScale)
    safeStore("ngp_chassis_roll_body_pitch_scale", state.bodyPitchScale)
    safeStore("ngp_chassis_roll_body_heave_scale", state.bodyHeaveScale)
    safeStore("ngp_chassis_roll_body_yaw_scale", state.bodyYawScale)

    safeStore("ngp_chassis_roll_body_roll_tau", state.bodyRollTau)
    safeStore("ngp_chassis_roll_body_pitch_tau", state.bodyPitchTau)
    safeStore("ngp_chassis_roll_body_heave_tau", state.bodyHeaveTau)
    safeStore("ngp_chassis_roll_body_yaw_tau", state.bodyYawTau)

    safeStore("ngp_chassis_roll_brake_lock_avg", state.brakeLockAvg)
    safeStore("ngp_chassis_roll_thermal_stress", state.brakeThermalStress)
    safeStore("ngp_chassis_roll_suspension_stress", state.suspensionStress)

    safeStore("ngp_chassis_roll_input_roll", state.inputRoll)
    safeStore("ngp_chassis_roll_input_pitch", state.inputPitch)
    safeStore("ngp_chassis_roll_input_heave", state.inputHeave)
    safeStore("ngp_chassis_roll_input_yaw", state.inputYaw)

    safeStore("ngp_chassis_roll_load_roll", state.loadRoll)
    safeStore("ngp_chassis_roll_load_pitch", state.loadPitch)
end

function M.init()
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1
    dt = safeNumber(dt, 0.0)

    if dt <= 0.0 then
        state.status = "BAD DT"
        exportState()
        return
    end

    updateGates(dt)

    car = car or safeGetCar()

    if state.rootReadNow then
        resetLinkFlags()
        readBodyRigidity()
        readRootInputs(car)
    else
        readBodyRigidity()
    end

    applyBodyRigidity(0.0, 0.0, 0.0, 0.0, dt)

    if not car then
        state.status = "NO CAR"
        updatePerception()
        exportState()
        return
    end

    local targetRoll =
        state.inputRoll * M.params.rollGain
        +
        state.loadRoll * M.params.loadRollGain
        +
        state.contactLossAvg * M.params.contactLossInputGain
        +
        state.rootRollAdd

    local targetPitch =
        state.inputPitch * M.params.pitchGain
        +
        state.loadPitch * M.params.loadPitchGain
        +
        state.brakeInput * M.params.brakePitchGain
        -
        state.driveTorqueNorm * M.params.drivePitchGain
        +
        state.rootPitchAdd

    local targetHeave =
        state.inputHeave * M.params.heaveGain
        +
        state.tireHopAvg * M.params.tireHopHeaveGain
        +
        state.rootHeaveAdd

    local targetYaw =
        state.inputYaw * M.params.yawGain
        +
        state.loadRoll * 0.12
        +
        state.contactLossAvg * 0.05
        +
        state.rootYawAdd

    targetRoll, targetPitch, targetHeave, targetYaw =
        applyBodyRigidity(
            targetRoll,
            targetPitch,
            targetHeave,
            targetYaw,
            dt
        )

    local damageScale =
        clamp(
            1.0
            -
            state.vehicleDamage * M.params.damageResponseLoss
            -
            state.brakeThermalStress * M.params.brakeThermalResponseLoss,
            0.65,
            1.05
        )

    targetRoll = targetRoll * damageScale
    targetPitch = targetPitch * damageScale
    targetHeave = targetHeave * damageScale
    targetYaw = targetYaw * damageScale

    state.roll =
        lowPass(
            state.roll,
            clamp(targetRoll, -M.params.maxRoll, M.params.maxRoll),
            state.bodyRollTau,
            dt
        )

    state.pitch =
        lowPass(
            state.pitch,
            clamp(targetPitch, -M.params.maxPitch, M.params.maxPitch),
            state.bodyPitchTau,
            dt
        )

    state.heave =
        lowPass(
            state.heave,
            clamp(targetHeave, -M.params.maxHeave, M.params.maxHeave),
            state.bodyHeaveTau,
            dt
        )

    state.yawHint =
        lowPass(
            state.yawHint,
            clamp(targetYaw, -M.params.maxYaw, M.params.maxYaw),
            state.bodyYawTau,
            dt
        )

    local energyTarget =
        clamp(
            abs(state.roll)
            +
            abs(state.pitch)
            +
            abs(state.heave)
            +
            abs(state.yawHint),
            0.0,
            1.0
        )

    local prevEnergy = state.bodyEnergy

    state.bodyEnergy =
        lowPass(
            state.bodyEnergy,
            energyTarget,
            0.120,
            dt
        )

    local releaseTarget =
        clamp(
            math.max(prevEnergy - energyTarget, 0.0)
            *
            M.params.releaseGain,
            0.0,
            1.0
        )

    state.bodyRelease =
        lowPass(
            state.bodyRelease,
            releaseTarget,
            0.180,
            dt
        )

    state.status = "RUNNING"

    updatePerception()
    exportState()
end

function M.getRoll()
    return state.roll or 0.0
end

function M.getPitch()
    return state.pitch or 0.0
end

function M.getHeave()
    return state.heave or 0.0
end

function M.getYawHint()
    return state.yawHint or 0.0
end

function M.getEnergy()
    return state.bodyEnergy or 0.0
end

function M.getRelease()
    return state.bodyRelease or 0.0
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f\n" ..
        "Roll %.3f Pitch %.3f Heave %.3f Yaw %.3f\n" ..
        "Energy %.3f Release %.3f %s\n" ..
        "RootAdd R/P/H/Y %.3f %.3f %.3f %.3f\n" ..
        "Links Body:%s Load:%s Road:%s Tire:%s Brake:%s Drive:%s Damage:%s Virtual:%s Sprung:%s Susp:%s Therm:%s",
        tostring(state.status),
        state.updateCount or 0,

        state.roll or 0.0,
        state.pitch or 0.0,
        state.heave or 0.0,
        state.yawHint or 0.0,

        state.bodyEnergy or 0.0,
        state.bodyRelease or 0.0,
        tostring(state.transition),

        state.rootRollAdd or 0.0,
        state.rootPitchAdd or 0.0,
        state.rootHeaveAdd or 0.0,
        state.rootYawAdd or 0.0,

        state.bodyRigidityLinked and "OK" or "NIL",
        state.loadLinked and "OK" or "NIL",
        state.roadLinked and "OK" or "NIL",
        state.tireLinked and "OK" or "NIL",
        state.brakeLinked and "OK" or "NIL",
        state.drivetrainLinked and "OK" or "NIL",
        state.damageLinked and "OK" or "NIL",
        state.virtualLinked and "OK" or "NIL",
        state.sprungLinked and "OK" or "NIL",
        state.suspensionLinked and "OK" or "NIL",
        state.thermalLinked and "OK" or "NIL"
    )
end

return M
