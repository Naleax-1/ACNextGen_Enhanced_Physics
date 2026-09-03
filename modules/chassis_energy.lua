---@diagnostic disable: undefined-global

--============================================================
-- chassis_energy.lua
-- ACNextGen V1.1.5 Stable
-- Phase 16 / Chassis Energy Storage and Release
--============================================================

local M = {}

M.params = {
    buildTau = 0.100,
    releaseTau = 0.180,

    yawGain = 0.42,
    rollGain = 0.35,
    pitchRateGain = 0.28,
    steerGain = 0.26,

    chassisRollGain = 0.52,
    chassisPitchGain = 0.42,
    chassisHeaveGain = 0.34,
    chassisYawGain = 0.38,

    virtualYawGain = 0.36,
    virtualPitchGain = 0.24,
    virtualRollGain = 0.28,
    rearYawLagGain = 0.30,

    instabilityGain = 0.40,

    rollCGGain = 0.30,
    pitchCGGain = 0.22,
    loadRollGain = 0.24,
    loadPitchGain = 0.20,

    tireHopGain = 0.14,
    contactLossGain = 0.10,

    brakeEnergyGain = 0.08,
    brakeLockEnergyGain = 0.06,
    brakeThermalEnergyGain = 0.05,

    suspensionStressEnergyGain = 0.10,
    sprungCGEnergyGain = 0.08,
    chassisRollRootGain = 0.24,
    chassisRollThermalGain = 0.05,
    chassisRollSuspensionGain = 0.08,

    damageEnergyGain = 0.18,
    bodyRuntimePenaltyGain = 0.12,

    bodyFlexEnergyGain = 0.12,
    bodyTorsionEnergyGain = 0.08,
    bodyBendingEnergyGain = 0.07,
    bodyStiffEnergyLoss = 0.05,

    bodyFlexReleaseLoss = 0.14,
    bodyStiffReleaseGain = 0.06,

    minBodyEnergyScale = 0.88,
    maxBodyEnergyScale = 1.18,
    minBodyReleaseScale = 0.82,
    maxBodyReleaseScale = 1.10,

    maxRootEnergyAdd = 0.25,
    maxEnergy = 1.00,
    maxRelease = 1.00,

    minDt = 0.0001,
    maxDt = 0.050,

    globalReadInterval = 0.050,
    debugStoreInterval = 0.250,
}

local state = {
    yawEnergy = 0.0,
    rollEnergy = 0.0,
    pitchEnergy = 0.0,
    heaveEnergy = 0.0,
    steerEnergy = 0.0,

    virtualEnergy = 0.0,
    bodyInputEnergy = 0.0,
    instabilityEnergy = 0.0,
    loadEnergy = 0.0,
    tireEnergy = 0.0,
    brakeEnergy = 0.0,
    brakeLockEnergy = 0.0,
    thermalEnergy = 0.0,
    suspensionStressEnergy = 0.0,
    sprungCGEnergy = 0.0,
    rootChassisRollEnergy = 0.0,
    bodyRuntimeEnergy = 0.0,

    totalEnergy = 0.0,
    targetEnergy = 0.0,
    release = 0.0,
    prevTargetEnergy = 0.0,

    yawRate = 0.0,
    pitchRate = 0.0,
    rollRate = 0.0,
    steer = 0.0,

    chassisRoll = 0.0,
    chassisPitch = 0.0,
    chassisHeave = 0.0,
    chassisYaw = 0.0,

    virtualYaw = 0.0,
    virtualPitch = 0.0,
    virtualRoll = 0.0,
    virtualRearYaw = 0.0,
    instability = 0.0,

    rollCG = 0.0,
    pitchCG = 0.0,
    loadRoll = 0.0,
    loadPitch = 0.0,

    tireHopAvg = 0.0,
    contactLossAvg = 0.0,
    brakeInput = 0.0,
    brakeLockAvg = 0.0,
    brakeThermalStress = 0.0,
    suspensionStress = 0.0,

    sprungPitchCG = 0.0,
    sprungRollCG = 0.0,

    rootRollAdd = 0.0,
    rootPitchAdd = 0.0,
    rootHeaveAdd = 0.0,
    rootYawAdd = 0.0,
    bodyRuntimePenalty = 0.0,
    vehicleDamage = 0.0,

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyBendingFactor = 1.0,
    bodyDampingFactor = 1.0,
    bodyEnergyScale = 1.0,
    bodyReleaseScale = 1.0,

    mode = "STABLE",
    status = "INIT",
    updateCount = 0,

    bodyRigidityLinked = false,
    chassisLinked = false,
    virtualLinked = false,
    loadLinked = false,
    tireLinked = false,
    brakeLinked = false,
    damageLinked = false,
    thermalLinked = false,
    suspensionLinked = false,
    sprungLinked = false,
    rollRootLinked = false,

    globalReadTimer = 999.0,
    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

--============================================================
-- Utilities
--============================================================

local function num(v, d)
    local n = tonumber(v)
    if n == nil or n ~= n then
        return d or 0.0
    end
    return n
end

local function abs(v)
    v = num(v, 0.0)
    if v < 0.0 then return -v end
    return v
end

local function clamp(v, mn, mx)
    v = num(v, mn or 0.0)
    if v < mn then return mn end
    if v > mx then return mx end
    return v
end

local function safeLoadRaw(key)
    if not ac or not ac.load then return nil end
    local ok, value = pcall(function()
        return ac.load(key)
    end)
    if not ok then return nil end
    return value
end

local function safeLoad(key, fallback)
    local value = safeLoadRaw(key)
    if value == nil then return fallback or 0.0 end
    return num(value, fallback or 0.0)
end

local function safeStore(key, value)
    if not ac or not ac.store then return end
    pcall(function()
        ac.store(key, value)
    end)
end

local function safeField(obj, field, fallback)
    if not obj then return fallback end
    local ok, value = pcall(function()
        return obj[field]
    end)
    if not ok or value == nil then return fallback end
    return value
end

local function getCarSafe()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function()
        return ac.getCar(0)
    end)
    if not ok then return nil end
    return car
end

local function lowPass(current, target, tau, dt)
    current = num(current, 0.0)
    target = num(target, 0.0)
    tau = math.max(num(tau, 0.001), 0.0001)
    dt = math.max(num(dt, 0.0), 0.0)

    local k = clamp(dt / (tau + dt), 0.0, 1.0)
    return current + (target - current) * k
end

local function updateGates(dt)
    state.globalReadTimer = (state.globalReadTimer or 0.0) + dt
    state.debugStoreTimer = (state.debugStoreTimer or 0.0) + dt

    if state.debugStoreTimer >= M.params.debugStoreInterval then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    else
        state.debugStoreNow = false
    end
end

local function consumeGlobalRead()
    if state.globalReadTimer >= M.params.globalReadInterval then
        state.globalReadTimer = 0.0
        return true
    end
    return false
end

--============================================================
-- Input
--============================================================

local function readBodyRigidity()
    local linked =
        safeLoadRaw("ngp_body_rigidity_update_count") ~= nil
        or
        safeLoadRaw("ngp_body_rigidity") ~= nil

    state.bodyRigidityLinked = linked

    state.bodyRigidity =
        clamp(safeLoad("ngp_body_rigidity", 1.0), 0.20, 1.35)

    state.bodyFlexFactor =
        clamp(safeLoad("ngp_body_flex_factor", 1.0), 0.25, 1.80)

    state.bodyTorsionFactor =
        clamp(safeLoad("ngp_body_torsion_factor", 1.0), 0.35, 2.00)

    state.bodyBendingFactor =
        clamp(safeLoad("ngp_body_bending_factor", 1.0), 0.35, 2.00)

    state.bodyDampingFactor =
        clamp(safeLoad("ngp_body_damping_factor", 1.0), 0.50, 1.80)

    state.bodyRuntimePenalty =
        clamp(safeLoad("ngp_body_runtime_flex_penalty", 0.0), 0.0, 0.55)
end

local function updateBodyScales(dt)
    if not state.bodyRigidityLinked then
        state.bodyEnergyScale = lowPass(state.bodyEnergyScale, 1.0, 0.08, dt)
        state.bodyReleaseScale = lowPass(state.bodyReleaseScale, 1.0, 0.08, dt)
        return
    end

    local p = M.params

    local softness =
        clamp((state.bodyFlexFactor or 1.0) - 1.0, 0.0, 0.80)

    local torsionSoft =
        clamp((state.bodyTorsionFactor or 1.0) - 1.0, 0.0, 1.0)

    local bendingSoft =
        clamp((state.bodyBendingFactor or 1.0) - 1.0, 0.0, 1.0)

    local stiffness =
        clamp((state.bodyRigidity or 1.0) - 1.0, 0.0, 0.35)

    local energyTarget =
        1.0
        + softness * p.bodyFlexEnergyGain
        + torsionSoft * p.bodyTorsionEnergyGain
        + bendingSoft * p.bodyBendingEnergyGain
        - stiffness * p.bodyStiffEnergyLoss

    local releaseTarget =
        1.0
        - softness * p.bodyFlexReleaseLoss
        + stiffness * p.bodyStiffReleaseGain

    state.bodyEnergyScale =
        lowPass(
            state.bodyEnergyScale,
            clamp(energyTarget, p.minBodyEnergyScale, p.maxBodyEnergyScale),
            0.080,
            dt
        )

    state.bodyReleaseScale =
        lowPass(
            state.bodyReleaseScale,
            clamp(releaseTarget, p.minBodyReleaseScale, p.maxBodyReleaseScale),
            0.080,
            dt
        )
end

local function readCarMotion(car)
    local av = safeField(car, "localAngularVelocity", nil)

    if av then
        state.pitchRate = num(av.x, 0.0)
        state.yawRate = num(av.y, 0.0)
        state.rollRate = num(av.z, 0.0)
    else
        state.pitchRate = 0.0
        state.yawRate = 0.0
        state.rollRate = 0.0
    end

    state.steer =
        clamp(num(safeField(car, "steer", 0.0), 0.0), -1.0, 1.0)

    local brakeFromCar =
        clamp(num(safeField(car, "brake", 0.0), 0.0), 0.0, 1.0)

    state.brakeInput =
        clamp(safeLoad("ngp_brake_input", brakeFromCar), 0.0, 1.0)
end

local function readChassisInputs()
    local cr = safeLoadRaw("ngp_chassis_roll")
    local cp = safeLoadRaw("ngp_chassis_pitch")
    local ch = safeLoadRaw("ngp_chassis_heave")
    local cy = safeLoadRaw("ngp_chassis_yaw_hint")

    state.chassisLinked = cr ~= nil or cp ~= nil or ch ~= nil or cy ~= nil

    state.chassisRoll = clamp(num(cr, 0.0), -2.0, 2.0)
    state.chassisPitch = clamp(num(cp, 0.0), -2.0, 2.0)
    state.chassisHeave = clamp(num(ch, 0.0), -2.0, 2.0)
    state.chassisYaw = clamp(num(cy, 0.0), -2.0, 2.0)
end

local function readVirtualInputs()
    local vy = safeLoadRaw("ngp_virtual_yaw")
    local vp = safeLoadRaw("ngp_virtual_pitch")
    local vr = safeLoadRaw("ngp_virtual_roll")
    local ry = safeLoadRaw("ngp_virtual_rear_yaw")
    local inst = safeLoadRaw("ngp_virtual_instability")

    state.virtualLinked =
        vy ~= nil or vp ~= nil or vr ~= nil or ry ~= nil or inst ~= nil

    state.virtualYaw = clamp(num(vy, 0.0), -2.5, 2.5)
    state.virtualPitch = clamp(num(vp, 0.0), -2.5, 2.5)
    state.virtualRoll = clamp(num(vr, 0.0), -2.5, 2.5)
    state.virtualRearYaw = clamp(num(ry, 0.0), -2.5, 2.5)
    state.instability = clamp(num(inst, 0.0), 0.0, 1.0)
end

local function readLoadInputs()
    local frontRaw =
        safeLoadRaw("ngp_load_front")
        or
        safeLoadRaw("ngp_load_path_front_delivery")
        or
        safeLoadRaw("ngp_load_path_front_integrity")

    local rearRaw =
        safeLoadRaw("ngp_load_rear")
        or
        safeLoadRaw("ngp_load_path_rear_delivery")
        or
        safeLoadRaw("ngp_load_path_rear_integrity")

    local leftRaw =
        safeLoadRaw("ngp_load_left")

    local rightRaw =
        safeLoadRaw("ngp_load_right")

    state.loadLinked =
        frontRaw ~= nil or rearRaw ~= nil or leftRaw ~= nil or rightRaw ~= nil

    local front = clamp(num(frontRaw, 0.5), 0.0, 1.5)
    local rear = clamp(num(rearRaw, 0.5), 0.0, 1.5)
    local left = clamp(num(leftRaw, 0.5), 0.0, 1.5)
    local right = clamp(num(rightRaw, 0.5), 0.0, 1.5)

    state.loadPitch = clamp(front - rear, -1.0, 1.0)
    state.loadRoll = clamp(right - left, -1.0, 1.0)

    state.rollCG = clamp(safeLoad("ngp_roll_cg", 0.0), -1.5, 1.5)
    state.pitchCG = clamp(safeLoad("ngp_pitch_cg", 0.0), -1.5, 1.5)
end

local function readTireInputs()
    local linked = false
    local hopSum = 0.0
    local lossSum = 0.0

    for i = 0, 3 do
        local hop =
            safeLoadRaw("ngp_tire_hop_energy_" .. i)
            or
            safeLoadRaw("ngp_tire_hop_" .. i)

        local loss =
            safeLoadRaw("ngp_tire_contact_loss_" .. i)
            or
            safeLoadRaw("ngp_tcr_contact_loss_" .. i)

        local cq =
            safeLoadRaw("ngp_contact_quality_" .. i)

        if hop ~= nil or loss ~= nil or cq ~= nil then
            linked = true
        end

        if loss == nil and cq ~= nil then
            loss = 1.0 - clamp(num(cq, 1.0), 0.0, 1.0)
        end

        hopSum = hopSum + clamp(num(hop, 0.0), 0.0, 1.0)
        lossSum = lossSum + clamp(num(loss, 0.0), 0.0, 1.0)
    end

    state.tireLinked = linked
    state.tireHopAvg = hopSum * 0.25
    state.contactLossAvg = lossSum * 0.25
end

local function readBrakeInputs()
    local lockLinked = false
    local lockSum = 0.0

    for i = 0, 3 do
        local lockRaw = safeLoadRaw("ngp_brake_lock_" .. i)
        if lockRaw ~= nil then lockLinked = true end
        lockSum = lockSum + clamp(num(lockRaw, 0.0), 0.0, 1.0)
    end

    local tempRaw =
        safeLoadRaw("ngp_brake_temp_max")
        or
        safeLoadRaw("ngp_brake_max_temp")
        or
        safeLoadRaw("ngp_brake_temp_avg")
        or
        safeLoadRaw("ngp_brake_avg_temp")

    state.brakeLockAvg = lockSum * 0.25
    state.thermalLinked = tempRaw ~= nil
    state.brakeLinked = safeLoadRaw("ngp_brake_input") ~= nil or lockLinked

    state.brakeThermalStress =
        clamp((num(tempRaw, 25.0) - 25.0) / 700.0, 0.0, 1.0)
end

local function readSuspensionInputs()
    local linked = false
    local sum = 0.0

    for i = 0, 3 do
        local susp =
            safeLoadRaw("ngp_susp_integrated_force_" .. i)
            or
            safeLoadRaw("ngp_susp_int_force_" .. i)
            or
            safeLoadRaw("ngp_susp_" .. i)

        local damper =
            safeLoadRaw("ngp_damper_hyst_" .. i)
            or
            safeLoadRaw("ngp_damper_force_" .. i)
            or
            safeLoadRaw("ngp_damper_" .. i)

        if susp ~= nil or damper ~= nil then
            linked = true
        end

        sum =
            sum
            +
            math.min(
                (abs(num(susp, 0.0)) + abs(num(damper, 0.0))) / 25000.0,
                1.0
            )
    end

    state.suspensionLinked = linked
    state.suspensionStress = clamp(sum * 0.25, 0.0, 1.0)
end

local function readSprungAndRootInputs()
    local pcg = safeLoadRaw("ngp_sprung_pitch_cg") or safeLoadRaw("ngp_pitch_cg")
    local rcg = safeLoadRaw("ngp_sprung_roll_cg") or safeLoadRaw("ngp_roll_cg")

    state.sprungLinked = pcg ~= nil or rcg ~= nil
    state.sprungPitchCG = clamp(num(pcg, 0.0), -0.12, 0.12)
    state.sprungRollCG = clamp(num(rcg, 0.0), -0.12, 0.12)

    local rra = safeLoadRaw("ngp_chassis_roll_root_roll_add")
    local rpa = safeLoadRaw("ngp_chassis_roll_root_pitch_add")
    local rha = safeLoadRaw("ngp_chassis_roll_root_heave_add")
    local rya = safeLoadRaw("ngp_chassis_roll_root_yaw_add")

    state.rollRootLinked = rra ~= nil or rpa ~= nil or rha ~= nil or rya ~= nil

    state.rootRollAdd = clamp(num(rra, 0.0), -0.25, 0.25)
    state.rootPitchAdd = clamp(num(rpa, 0.0), -0.25, 0.25)
    state.rootHeaveAdd = clamp(num(rha, 0.0), -0.25, 0.25)
    state.rootYawAdd = clamp(num(rya, 0.0), -0.25, 0.25)
end

local function readDamageInputs()
    local damageRaw =
        safeLoadRaw("ngp_condition_total")
        or
        safeLoadRaw("ngp_vehicle_condition_total")
        or
        safeLoadRaw("ngp_damage_chassis")

    state.damageLinked = damageRaw ~= nil
    state.vehicleDamage = clamp(num(damageRaw, 0.0), 0.0, 1.0)
end

local function readInputs(car)
    readCarMotion(car)

    if consumeGlobalRead() then
        readBodyRigidity()
        readChassisInputs()
        readVirtualInputs()
        readLoadInputs()
        readTireInputs()
        readBrakeInputs()
        readSuspensionInputs()
        readSprungAndRootInputs()
        readDamageInputs()
    end
end

--============================================================
-- Core
--============================================================

local function calculateEnergy()
    local p = M.params

    state.yawEnergy =
        clamp(abs(state.yawRate) * p.yawGain, 0.0, 1.0)

    state.rollEnergy =
        clamp(abs(state.rollRate) * p.rollGain + abs(state.chassisRoll) * p.chassisRollGain, 0.0, 1.0)

    state.pitchEnergy =
        clamp(abs(state.pitchRate) * p.pitchRateGain + abs(state.chassisPitch) * p.chassisPitchGain, 0.0, 1.0)

    state.heaveEnergy =
        clamp(abs(state.chassisHeave) * p.chassisHeaveGain, 0.0, 1.0)

    state.steerEnergy =
        clamp(abs(state.steer) * p.steerGain, 0.0, 1.0)

    state.virtualEnergy =
        clamp(
            abs(state.virtualYaw) * p.virtualYawGain
            + abs(state.virtualPitch) * p.virtualPitchGain
            + abs(state.virtualRoll) * p.virtualRollGain
            + abs(state.virtualRearYaw) * p.rearYawLagGain,
            0.0,
            1.0
        )

    state.bodyInputEnergy =
        clamp(
            abs(state.chassisYaw) * p.chassisYawGain
            + abs(state.chassisRoll) * p.chassisRollGain
            + abs(state.chassisPitch) * p.chassisPitchGain
            + abs(state.chassisHeave) * p.chassisHeaveGain,
            0.0,
            1.0
        )

    state.instabilityEnergy =
        clamp(state.instability * p.instabilityGain, 0.0, 1.0)

    state.loadEnergy =
        clamp(
            abs(state.loadRoll) * p.loadRollGain
            + abs(state.loadPitch) * p.loadPitchGain
            + abs(state.rollCG) * p.rollCGGain
            + abs(state.pitchCG) * p.pitchCGGain,
            0.0,
            1.0
        )

    state.tireEnergy =
        clamp(
            state.tireHopAvg * p.tireHopGain
            + state.contactLossAvg * p.contactLossGain,
            0.0,
            1.0
        )

    state.brakeEnergy =
        clamp(state.brakeInput * p.brakeEnergyGain, 0.0, 1.0)

    state.brakeLockEnergy =
        clamp(state.brakeLockAvg * p.brakeLockEnergyGain, 0.0, 1.0)

    state.thermalEnergy =
        clamp(state.brakeThermalStress * p.brakeThermalEnergyGain, 0.0, 1.0)

    state.suspensionStressEnergy =
        clamp(state.suspensionStress * p.suspensionStressEnergyGain, 0.0, 1.0)

    state.sprungCGEnergy =
        clamp(
            (abs(state.sprungPitchCG) + abs(state.sprungRollCG)) * p.sprungCGEnergyGain,
            0.0,
            1.0
        )

    state.rootChassisRollEnergy =
        clamp(
            (abs(state.rootRollAdd) + abs(state.rootPitchAdd) + abs(state.rootHeaveAdd) + abs(state.rootYawAdd))
            * p.chassisRollRootGain
            + state.brakeThermalStress * p.chassisRollThermalGain
            + state.suspensionStress * p.chassisRollSuspensionGain,
            0.0,
            p.maxRootEnergyAdd
        )

    state.bodyRuntimeEnergy =
        clamp(state.bodyRuntimePenalty * p.bodyRuntimePenaltyGain, 0.0, 1.0)

    local target =
        state.yawEnergy
        + state.rollEnergy
        + state.pitchEnergy
        + state.heaveEnergy
        + state.steerEnergy
        + state.virtualEnergy
        + state.bodyInputEnergy
        + state.instabilityEnergy
        + state.loadEnergy
        + state.tireEnergy
        + state.brakeEnergy
        + state.brakeLockEnergy
        + state.thermalEnergy
        + state.suspensionStressEnergy
        + state.sprungCGEnergy
        + state.rootChassisRollEnergy
        + state.bodyRuntimeEnergy
        + state.vehicleDamage * p.damageEnergyGain

    target = clamp(target * 0.35, 0.0, p.maxEnergy)
    target = clamp(target * state.bodyEnergyScale, 0.0, p.maxEnergy)

    state.targetEnergy = target
end

local function updateMode()
    if state.totalEnergy < 0.08 then
        state.mode = "STABLE"
    elseif state.release > 0.10 then
        state.mode = "RELEASE"
    elseif state.totalEnergy < 0.35 then
        state.mode = "BUILD"
    else
        state.mode = "LOADED"
    end
end

--============================================================
-- Export
--============================================================

local function exportState()
    safeStore("ngp_chassis_energy", state.totalEnergy)
    safeStore("ngp_chassis_target_energy", state.targetEnergy)
    safeStore("ngp_chassis_release", state.release)

    safeStore("ngp_chassis_body_energy", state.totalEnergy)
    safeStore("ngp_chassis_body_release", state.release)
    safeStore("ngp_body_chassis_energy", state.totalEnergy)
    safeStore("ngp_body_chassis_release", state.release)

    safeStore("ngp_chassis_energy_mode", state.mode)
    safeStore("ngp_chassis_energy_status", state.status)
    safeStore("ngp_chassis_energy_update_count", state.updateCount)

    safeStore("ngp_chassis_energy_body_energy_scale", state.bodyEnergyScale)
    safeStore("ngp_chassis_energy_body_release_scale", state.bodyReleaseScale)

    safeStore("ngp_chassis_yaw_energy", state.yawEnergy)
    safeStore("ngp_chassis_roll_energy", state.rollEnergy)
    safeStore("ngp_chassis_pitch_energy", state.pitchEnergy)
    safeStore("ngp_chassis_virtual_energy", state.virtualEnergy)
    safeStore("ngp_chassis_body_input_energy", state.bodyInputEnergy)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_chassis_energy_yaw", state.yawEnergy)
    safeStore("ngp_chassis_energy_roll", state.rollEnergy)
    safeStore("ngp_chassis_energy_pitch", state.pitchEnergy)
    safeStore("ngp_chassis_energy_heave", state.heaveEnergy)
    safeStore("ngp_chassis_energy_steer", state.steerEnergy)
    safeStore("ngp_chassis_energy_virtual", state.virtualEnergy)
    safeStore("ngp_chassis_energy_body_input", state.bodyInputEnergy)
    safeStore("ngp_chassis_energy_instability", state.instabilityEnergy)
    safeStore("ngp_chassis_energy_load", state.loadEnergy)
    safeStore("ngp_chassis_energy_tire", state.tireEnergy)
    safeStore("ngp_chassis_energy_brake", state.brakeEnergy)
    safeStore("ngp_chassis_energy_brake_lock", state.brakeLockEnergy)
    safeStore("ngp_chassis_energy_thermal", state.thermalEnergy)
    safeStore("ngp_chassis_energy_suspension_stress", state.suspensionStressEnergy)
    safeStore("ngp_chassis_energy_sprung_cg", state.sprungCGEnergy)
    safeStore("ngp_chassis_energy_roll_root", state.rootChassisRollEnergy)
    safeStore("ngp_chassis_energy_body_runtime", state.bodyRuntimeEnergy)

    safeStore("ngp_chassis_energy_body_linked", state.bodyRigidityLinked and 1 or 0)
    safeStore("ngp_chassis_energy_chassis_linked", state.chassisLinked and 1 or 0)
    safeStore("ngp_chassis_energy_virtual_linked", state.virtualLinked and 1 or 0)
    safeStore("ngp_chassis_energy_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_chassis_energy_tire_linked", state.tireLinked and 1 or 0)
    safeStore("ngp_chassis_energy_brake_linked", state.brakeLinked and 1 or 0)
    safeStore("ngp_chassis_energy_damage_linked", state.damageLinked and 1 or 0)
    safeStore("ngp_chassis_energy_thermal_linked", state.thermalLinked and 1 or 0)
    safeStore("ngp_chassis_energy_suspension_linked", state.suspensionLinked and 1 or 0)
    safeStore("ngp_chassis_energy_sprung_linked", state.sprungLinked and 1 or 0)
    safeStore("ngp_chassis_energy_roll_root_linked", state.rollRootLinked and 1 or 0)

    safeStore("ngp_chassis_energy_yaw_rate", state.yawRate)
    safeStore("ngp_chassis_energy_roll_rate", state.rollRate)
    safeStore("ngp_chassis_energy_pitch_rate", state.pitchRate)
    safeStore("ngp_chassis_energy_steer_input", state.steer)
end

--============================================================
-- Public API
--============================================================

function M.init()
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = clamp(num(dt, 0.0), M.params.minDt, M.params.maxDt)
    updateGates(dt)

    car = car or getCarSafe()

    if not car then
        state.status = "NO CAR"
        exportState()
        return
    end

    state.status = "RUNNING"

    readInputs(car)
    updateBodyScales(dt)
    calculateEnergy()

    local tau =
        state.targetEnergy > state.totalEnergy
        and
        M.params.buildTau
        or
        M.params.releaseTau

    state.totalEnergy =
        lowPass(
            state.totalEnergy,
            state.targetEnergy,
            tau,
            dt
        )

    local releaseTarget =
        clamp(
            math.max((state.prevTargetEnergy or 0.0) - (state.targetEnergy or 0.0), 0.0)
            *
            state.bodyReleaseScale,
            0.0,
            M.params.maxRelease
        )

    state.release =
        lowPass(
            state.release,
            releaseTarget,
            M.params.releaseTau,
            dt
        )

    state.prevTargetEnergy = state.targetEnergy

    updateMode()
    exportState()
end

function M.getEnergy()
    return state.totalEnergy or 0.0
end

function M.getRelease()
    return state.release or 0.0
end

function M.getTargetEnergy()
    return state.targetEnergy or 0.0
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f\n" ..
        "Energy %.3f Target %.3f Release %.3f Mode %s\n" ..
        "Y %.2f R %.2f P %.2f H %.2f V %.2f Load %.2f Tire %.2f Brake %.2f\n" ..
        "Root Lock %.2f Therm %.2f Susp %.2f Sprung %.2f RollRoot %.2f BodyRun %.2f\n" ..
        "Links Body:%s Chassis:%s Virtual:%s Load:%s Tire:%s Brake:%s Damage:%s Therm:%s Susp:%s Sprung:%s RollRoot:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.totalEnergy or 0.0,
        state.targetEnergy or 0.0,
        state.release or 0.0,
        tostring(state.mode),
        state.yawEnergy or 0.0,
        state.rollEnergy or 0.0,
        state.pitchEnergy or 0.0,
        state.heaveEnergy or 0.0,
        state.virtualEnergy or 0.0,
        state.loadEnergy or 0.0,
        state.tireEnergy or 0.0,
        state.brakeEnergy or 0.0,
        state.brakeLockEnergy or 0.0,
        state.thermalEnergy or 0.0,
        state.suspensionStressEnergy or 0.0,
        state.sprungCGEnergy or 0.0,
        state.rootChassisRollEnergy or 0.0,
        state.bodyRuntimeEnergy or 0.0,
        state.bodyRigidityLinked and "OK" or "NIL",
        state.chassisLinked and "OK" or "NIL",
        state.virtualLinked and "OK" or "NIL",
        state.loadLinked and "OK" or "NIL",
        state.tireLinked and "OK" or "NIL",
        state.brakeLinked and "OK" or "NIL",
        state.damageLinked and "OK" or "NIL",
        state.thermalLinked and "OK" or "NIL",
        state.suspensionLinked and "OK" or "NIL",
        state.sprungLinked and "OK" or "NIL",
        state.rollRootLinked and "OK" or "NIL"
    )
end

return M
