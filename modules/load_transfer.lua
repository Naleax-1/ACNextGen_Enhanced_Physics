---@diagnostic disable: undefined-global

--============================================================
-- ACNextGen
-- load_transfer.lua
-- Phase 6 / V1.1.5 Stable
-- Dynamic Load Transfer / Perception Mapping
--
-- App-side observer/bridge only. No direct AC physics overwrite.
--============================================================

local M = {}

--============================================================
-- Geometry / Parameters
--============================================================

local initialized = false
local geometry = {
    wheelbase = 2.50,
    trackF = 1.45,
    trackR = 1.42,
}

M.params = {
    tauPitch = 0.060,
    tauRoll = 0.040,

    maxPitch = 4500.0,
    maxRoll = 3500.0,

    defaultMass = 1300.0,
    defaultCGH = 0.55,

    tireHopLoadGain = 0.25,

    rollStiffnessFrontBias = 0.58,
    pitchTransferGain = 1.00,
    rollTransferGain = 1.00,

    minRollAxleBias = 0.35,
    maxRollAxleBias = 0.75,

    wheelLoadTau = 0.080,
    wheelLoadRef = 3500.0,

    minWheelLoad = 0.0,
    maxWheelLoad = 9000.0,

    modelLoadGain = 0.18,
    suspensionForceGain = 0.10,

    damperLoadGain = 0.012,
    springLoadGain = 0.006,

    contactLoadGain = 350.0,
    impulseLoadGain = 280.0,
    dropUnloadGain = 450.0,

    tireHopPulseGain = 0.22,
    contactLossUnloadGain = 320.0,
    tireMemoryLoadLossGain = 120.0,
    tireLimitLoadLossGain = 90.0,

    drivetrainRearLoadGain = 60.0,
    lsdRearLoadGain = 90.0,

    armGeometryLoadLossGain = 80.0,

    conditionSuspLossGain = 220.0,
    damageWheelLossGain = 450.0,

    bodyRigidityInfluence = 0.18,

    bodyFlexTauGain = 0.45,
    bodyStiffTauGain = 0.12,

    bodyFlexTransferLoss = 0.10,
    bodyTorsionRollLoss = 0.08,
    bodyBendingPitchLoss = 0.08,

    bodyStiffTransferGain = 0.05,

    minBodyTransferScale = 0.86,
    maxBodyTransferScale = 1.10,

    minDt = 0.0005,
    maxDt = 0.050,

    noCarDecayTau = 0.250,
    noWheelsDecayTau = 0.220,

    bodyReadInterval = 0.050,
    geometryRetryInterval = 1.000,

    debugStoreInterval = 0.25,
}

--============================================================
-- State
--============================================================

local filtered = {
    pitch = 0.0,
    roll = 0.0,
    rollFront = 0.0,
    rollRear = 0.0,
}

M.state = {
    front = 0.5,
    rear = 0.5,
    left = 0.5,
    right = 0.5,

    pitchState = "NEUTRAL",
    rollState = "NEUTRAL",
    transition = "STABLE",

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyBendingFactor = 1.0,
    bodyDampingFactor = 1.0,

    bodyPitchScale = 1.0,
    bodyRollScale = 1.0,
    bodyPitchTau = 0.060,
    bodyRollTau = 0.040,
    bodyRigidityLinked = false,
    bodyReadTimer = 999.0,

    tireLinked = false,
    suspensionLinked = false,
    drivetrainLinked = false,
    armLinked = false,
    damageLinked = false,
    contactLinked = false,
    damperLinked = false,
    complianceLinked = false,
    windupLinked = false,

    avgWheelLoad = 3500.0,
    frontWheelLoad = 3500.0,
    rearWheelLoad = 3500.0,
    leftWheelLoad = 3500.0,
    rightWheelLoad = 3500.0,
    maxWheelLoadLive = 3500.0,
    minWheelLoadLive = 3500.0,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
    geometryRetryTimer = 999.0,
}

M.debug = {
    ax = 0.0,
    az = 0.0,

    mass = 0.0,
    cgH = 0.0,

    rawPitch = 0.0,
    rawRoll = 0.0,
    rawRollFront = 0.0,
    rawRollRear = 0.0,

    filtPitch = 0.0,
    filtRoll = 0.0,
    filtRollFront = 0.0,
    filtRollRear = 0.0,

    load = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadPitch = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadRoll = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    wheelLoadRaw = { [0] = 3500.0, [1] = 3500.0, [2] = 3500.0, [3] = 3500.0 },
    wheelLoadFiltered = { [0] = 3500.0, [1] = 3500.0, [2] = 3500.0, [3] = 3500.0 },
    wheelLoadFinal = { [0] = 3500.0, [1] = 3500.0, [2] = 3500.0, [3] = 3500.0 },
    wheelLoadNorm = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    modelLoadAdd = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    suspensionLoadAdd = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    rootLoadAdd = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    absFront = 0.5,
    absRear = 0.5,
    absLeft = 0.5,
    absRight = 0.5,

    pitchRatio = 0.0,
    rollRatio = 0.0,

    status = "INIT",
    updateCount = 0,
    bodyPitchScale = 1.0,
    bodyRollScale = 1.0,
    bodyPitchTau = 0.060,
    bodyRollTau = 0.040,
}

--============================================================
-- Utility
--============================================================

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

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.0)
    dt = safeNumber(dt, 0.0)

    if tau <= 0.0 then
        return target
    end

    return current + (target - current) * clamp(dt / math.max(tau + dt, 0.0001), 0.0, 1.0)
end

local function safeLoadRaw(key)
    if not ac or not ac.load then
        return nil
    end

    local ok, value = pcall(function()
        return ac.load(key)
    end)

    if not ok then
        return nil
    end

    return value
end

local function safeLoad(key, defaultValue)
    local value = safeLoadRaw(key)
    if value == nil then
        return defaultValue or 0.0
    end
    return safeNumber(value, defaultValue or 0.0)
end

local function safeStore(key, value)
    if not ac or not ac.store then
        return
    end

    pcall(function()
        ac.store(key, value)
    end)
end

local function safeField(obj, field, defaultValue)
    if not obj then
        return defaultValue
    end

    local ok, value = pcall(function()
        return obj[field]
    end)

    if not ok or value == nil then
        return defaultValue
    end

    return value
end

local function safeGetCar()
    if not ac or not ac.getCar then
        return nil
    end

    local ok, car = pcall(function()
        return ac.getCar(0)
    end)

    if ok then
        return car
    end

    return nil
end

local function safeConfig(section, key, defaultValue)
    if not ac or not ac.getCarConfig then
        return defaultValue
    end

    local ok, value = pcall(function()
        return ac.getCarConfig(0, section, key, defaultValue)
    end)

    if not ok or value == nil then
        return defaultValue
    end

    return safeNumber(value, defaultValue)
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

local function getWheel(car, index)
    if not hasWheels(car) then
        return nil
    end

    local ok, wheel = pcall(function()
        return car.wheels[index] or car.wheels[index + 1]
    end)

    if ok then
        return wheel
    end

    return nil
end

local function updateDebugGate(dt)
    M.state.debugStoreTimer = (M.state.debugStoreTimer or 0.0) + (dt or 0.0)

    if M.state.debugStoreTimer >= M.params.debugStoreInterval then
        M.state.debugStoreTimer = 0.0
        M.state.debugStoreNow = true
    else
        M.state.debugStoreNow = false
    end
end

local function setStatus(status)
    M.state.status = status or "UNKNOWN"
    M.debug.status = M.state.status
end

--============================================================
-- Geometry
--============================================================

local function initGeometry(force)
    if initialized and not force then
        return
    end

    geometry.wheelbase = safeConfig("BASIC", "WHEELBASE", geometry.wheelbase)
    geometry.trackF = safeConfig("BASIC", "FRONT_TRACK", geometry.trackF)
    geometry.trackR = safeConfig("BASIC", "REAR_TRACK", geometry.trackR)

    if geometry.wheelbase <= 0.0 then geometry.wheelbase = 2.50 end
    if geometry.trackF <= 0.0 then geometry.trackF = 1.45 end
    if geometry.trackR <= 0.0 then geometry.trackR = 1.42 end

    initialized = true

    if ac and ac.log then
        pcall(function()
            ac.log(string.format(
                "[NGP LoadTransfer] WB=%.2f TF=%.2f TR=%.2f",
                geometry.wheelbase,
                geometry.trackF,
                geometry.trackR
            ))
        end)
    end
end

local function updateGeometryRetry(dt)
    M.state.geometryRetryTimer = (M.state.geometryRetryTimer or 0.0) + (dt or 0.0)

    if (not initialized) or M.state.geometryRetryTimer >= M.params.geometryRetryInterval then
        M.state.geometryRetryTimer = 0.0
        initGeometry(not initialized)
    end
end

--============================================================
-- Inputs
--============================================================

local function vecLength(v)
    if not v then return 0.0 end

    local ok, result = pcall(function()
        if type(v.length) == "function" then
            return v:length()
        end
        if type(v.length) == "number" then
            return v.length
        end

        local x = safeNumber(v.x, 0.0)
        local y = safeNumber(v.y, 0.0)
        local z = safeNumber(v.z, 0.0)
        return math.sqrt(x * x + y * y + z * z)
    end)

    if ok then
        return safeNumber(result, 0.0)
    end

    return 0.0
end

local function readAcceleration(car)
    local ax = 0.0
    local az = 0.0

    local acceleration = safeField(car, "acceleration", nil)
    if acceleration then
        ax = safeNumber(safeField(acceleration, "x", 0.0), 0.0)
        az = safeNumber(safeField(acceleration, "z", 0.0), 0.0)
    end

    if ax == 0.0 and az == 0.0 then
        local localVelocity = safeField(car, "localVelocity", nil)
        if localVelocity then
            ax = safeNumber(safeField(localVelocity, "x", 0.0), 0.0) * 0.0
            az = safeNumber(safeField(localVelocity, "z", 0.0), 0.0) * 0.0
        end
    end

    return ax, az
end

local function readMass(car)
    local mass = safeNumber(safeField(car, "mass", nil), nil)

    if mass == nil or mass <= 0.0 then
        mass = safeLoad("ngp_mass_total", M.params.defaultMass)
    end

    if mass == nil or mass <= 0.0 then
        mass = M.params.defaultMass
    end

    return clamp(mass, 400.0, 3000.0)
end

local function readCGHeight()
    local cgH = safeConfig("BASIC", "CG_HEIGHT", M.params.defaultCGH)

    if cgH <= 0.0 then
        cgH = safeLoad("ngp_cg_height", M.params.defaultCGH)
    end

    if cgH <= 0.0 then
        cgH = M.params.defaultCGH
    end

    return clamp(cgH, 0.15, 1.20)
end

local function readWheelLoadField(wheel)
    if not wheel then return nil end

    local fields = {
        "load",
        "loadK",
    }

    for i = 1, #fields do
        local value = safeField(wheel, fields[i], nil)
        if value ~= nil then
            local n = safeNumber(value, nil)
            if n ~= nil and n == n then
                return abs(n)
            end
        end
    end

    return nil
end

local function readWheelLoad(car, index, fallback)
    local wheel = getWheel(car, index)
    local value = readWheelLoadField(wheel)

    if value ~= nil and value > 1.0 then
        return value
    end

    local contactLoad = safeLoadRaw("ngp_contact_load_" .. index)
    if contactLoad ~= nil then
        return abs(contactLoad)
    end

    local tireStateLoad = safeLoadRaw("ngp_tire_state_load_" .. index)
    if tireStateLoad ~= nil then
        return abs(tireStateLoad)
    end

    local sprungLoad = safeLoadRaw("ngp_sprung_load_" .. index)
    if sprungLoad ~= nil then
        return abs(sprungLoad)
    end

    local last = M.debug.wheelLoadFiltered[index]
    if last ~= nil and last > 1.0 then
        return last
    end

    return fallback or M.params.wheelLoadRef
end

--============================================================
-- Body rigidity coupling
--============================================================

local function readBodyRigidity(dt)
    M.state.bodyReadTimer = (M.state.bodyReadTimer or 999.0) + (dt or 0.0)

    if M.state.bodyReadTimer < M.params.bodyReadInterval then
        return
    end

    M.state.bodyReadTimer = 0.0

    local bodyCount = safeLoadRaw("ngp_body_rigidity_update_count")
    local bodyStatus = safeLoadRaw("ngp_body_status")

    M.state.bodyRigidity = clamp(safeLoad("ngp_body_rigidity", 1.0), 0.20, 1.35)
    M.state.bodyFlexFactor = clamp(safeLoad("ngp_body_flex_factor", 1.0), 0.25, 1.80)
    M.state.bodyTorsionFactor = clamp(safeLoad("ngp_body_torsion_factor", 1.0), 0.35, 2.00)
    M.state.bodyBendingFactor = clamp(safeLoad("ngp_body_bending_factor", 1.0), 0.35, 2.00)
    M.state.bodyDampingFactor = clamp(safeLoad("ngp_body_damping_factor", 1.0), 0.50, 1.80)

    M.state.bodyRigidityLinked =
        safeNumber(bodyCount, 0.0) > 0.0
        or bodyStatus ~= nil
end

local function applyBodyRigidityToTransfer(rawPitch, rawRoll, dt)
    local p = M.params

    if not M.state.bodyRigidityLinked then
        M.state.bodyPitchScale = lowPass(M.state.bodyPitchScale or 1.0, 1.0, 0.100, dt)
        M.state.bodyRollScale = lowPass(M.state.bodyRollScale or 1.0, 1.0, 0.100, dt)
        M.state.bodyPitchTau = p.tauPitch
        M.state.bodyRollTau = p.tauRoll
        return rawPitch, rawRoll
    end

    local rigidity = clamp(M.state.bodyRigidity or 1.0, 0.20, 1.35)
    local flex = clamp(M.state.bodyFlexFactor or 1.0, 0.25, 1.80)
    local torsion = clamp(M.state.bodyTorsionFactor or 1.0, 0.35, 2.00)
    local bending = clamp(M.state.bodyBendingFactor or 1.0, 0.35, 2.00)
    local damping = clamp(M.state.bodyDampingFactor or 1.0, 0.50, 1.80)

    local softness = clamp(flex - 1.0, 0.0, 0.80)
    local stiffness = clamp(rigidity - 1.0, 0.0, 0.35)
    local torsionSoft = clamp(torsion - 1.0, 0.0, 1.0)
    local bendingSoft = clamp(bending - 1.0, 0.0, 1.0)

    local pitchScaleTarget =
        1.0
        - softness * p.bodyFlexTransferLoss
        - bendingSoft * p.bodyBendingPitchLoss
        + stiffness * p.bodyStiffTransferGain

    local rollScaleTarget =
        1.0
        - softness * p.bodyFlexTransferLoss
        - torsionSoft * p.bodyTorsionRollLoss
        + stiffness * p.bodyStiffTransferGain

    pitchScaleTarget = clamp(pitchScaleTarget, p.minBodyTransferScale, p.maxBodyTransferScale)
    rollScaleTarget = clamp(rollScaleTarget, p.minBodyTransferScale, p.maxBodyTransferScale)

    M.state.bodyPitchScale = lowPass(M.state.bodyPitchScale or 1.0, pitchScaleTarget, 0.050, dt)
    M.state.bodyRollScale = lowPass(M.state.bodyRollScale or 1.0, rollScaleTarget, 0.050, dt)

    local pitchTau =
        p.tauPitch
        * (1.0 + softness * p.bodyFlexTauGain)
        / (1.0 + stiffness * p.bodyStiffTauGain)

    local rollTau =
        p.tauRoll
        * (1.0 + softness * p.bodyFlexTauGain)
        / (1.0 + stiffness * p.bodyStiffTauGain)

    pitchTau = pitchTau / math.max(damping, 0.50)
    rollTau = rollTau / math.max(damping, 0.50)

    M.state.bodyPitchTau = clamp(pitchTau, 0.035, 0.110)
    M.state.bodyRollTau = clamp(rollTau, 0.025, 0.090)

    return rawPitch * M.state.bodyPitchScale, rawRoll * M.state.bodyRollScale
end

--============================================================
-- Core transfer
--============================================================

local function calculateRawTransfer(ax, az, mass, cgH)
    local p = M.params

    local rawPitch =
        mass
        * az
        * cgH
        / math.max(geometry.wheelbase, 0.001)

    rawPitch = rawPitch * p.pitchTransferGain

    local rollMoment = mass * ax * cgH

    local frontBias = clamp(
        p.rollStiffnessFrontBias,
        p.minRollAxleBias,
        p.maxRollAxleBias
    )

    local rearBias = 1.0 - frontBias

    local rawRollFront =
        rollMoment
        * frontBias
        / math.max(geometry.trackF, 0.001)

    local rawRollRear =
        rollMoment
        * rearBias
        / math.max(geometry.trackR, 0.001)

    rawRollFront = rawRollFront * p.rollTransferGain
    rawRollRear = rawRollRear * p.rollTransferGain

    local rawRoll = rawRollFront + rawRollRear

    rawPitch = clamp(rawPitch, -p.maxPitch, p.maxPitch)
    rawRoll = clamp(rawRoll, -p.maxRoll, p.maxRoll)
    rawRollFront = clamp(rawRollFront, -p.maxRoll, p.maxRoll)
    rawRollRear = clamp(rawRollRear, -p.maxRoll, p.maxRoll)

    return rawPitch, rawRoll, rawRollFront, rawRollRear
end

local function updateFilteredTransfer(rawPitch, rawRoll, rawRollFront, rawRollRear, dt)
    filtered.pitch = lowPass(
        filtered.pitch,
        rawPitch,
        M.state.bodyPitchTau or M.params.tauPitch,
        dt
    )

    filtered.roll = lowPass(
        filtered.roll,
        rawRoll,
        M.state.bodyRollTau or M.params.tauRoll,
        dt
    )

    filtered.rollFront = lowPass(
        filtered.rollFront,
        rawRollFront,
        M.state.bodyRollTau or M.params.tauRoll,
        dt
    )

    filtered.rollRear = lowPass(
        filtered.rollRear,
        rawRollRear,
        M.state.bodyRollTau or M.params.tauRoll,
        dt
    )

    M.debug.filtPitch = filtered.pitch
    M.debug.filtRoll = filtered.roll
    M.debug.filtRollFront = filtered.rollFront
    M.debug.filtRollRear = filtered.rollRear
end

local function distributeWheelLoads()
    local pitch = filtered.pitch
    local rollFront = filtered.rollFront
    local rollRear = filtered.rollRear

    M.debug.loadPitch[0] = -pitch * 0.5
    M.debug.loadPitch[1] = -pitch * 0.5
    M.debug.loadPitch[2] =  pitch * 0.5
    M.debug.loadPitch[3] =  pitch * 0.5

    M.debug.loadRoll[0] = -rollFront * 0.5
    M.debug.loadRoll[1] =  rollFront * 0.5
    M.debug.loadRoll[2] = -rollRear * 0.5
    M.debug.loadRoll[3] =  rollRear * 0.5

    for i = 0, 3 do
        M.debug.load[i] = (M.debug.loadPitch[i] or 0.0) + (M.debug.loadRoll[i] or 0.0)
    end
end

local function applyTireHopLoadPulse()
    for i = 0, 3 do
        local hop =
            safeLoadRaw("ngp_tire_hop_loadpulse_" .. i)
            or safeLoadRaw("ngp_tirehop_" .. i)
            or safeLoadRaw("ngp_tire_hop_load_pulse_" .. i)

        if hop ~= nil then
            M.state.tireLinked = true
            M.debug.load[i] = (M.debug.load[i] or 0.0) + safeNumber(hop, 0.0) * M.params.tireHopLoadGain
        end
    end
end

--============================================================
-- Root load adds
--============================================================

local function readRootWheelLoadAdd(index)
    local p = M.params
    local add = 0.0

    local hopPulse =
        safeLoadRaw("ngp_tire_hop_loadpulse_" .. index)
        or safeLoadRaw("ngp_tirehop_" .. index)
        or safeLoadRaw("ngp_tire_hop_load_pulse_" .. index)

    local contactLoss =
        safeLoadRaw("ngp_tire_contact_loss_" .. index)
        or safeLoadRaw("ngp_tcr_contact_loss_" .. index)
        or safeLoadRaw("ngp_contact_loss_" .. index)
        or safeLoadRaw("ngp_contact_hop_drop_" .. index)

    local memory =
        safeLoadRaw("ngp_rubber_memory_" .. index)
        or safeLoadRaw("ngp_tire_memory_" .. index)
        or safeLoadRaw("ngp_memory_" .. index)

    local tireLimit =
        safeLoadRaw("ngp_tire_limit_" .. index)
        or safeLoadRaw("ngp_slip_limit_" .. index)

    if hopPulse ~= nil or contactLoss ~= nil or memory ~= nil or tireLimit ~= nil then
        M.state.tireLinked = true
    end

    add =
        add
        + abs(hopPulse) * p.tireHopPulseGain
        - clamp(safeNumber(contactLoss, 0.0), 0.0, 1.0) * p.contactLossUnloadGain
        - clamp(safeNumber(memory, 0.0), 0.0, 1.0) * p.tireMemoryLoadLossGain
        - clamp(safeNumber(tireLimit, 0.0), 0.0, 2.0) * p.tireLimitLoadLossGain

    local armCamber =
        safeLoadRaw("ngp_control_arm_camber_" .. index)
        or safeLoadRaw("ngp_arm_camber_" .. index)
        or safeLoadRaw("ngp_ca_camber_" .. index)

    local armToe =
        safeLoadRaw("ngp_control_arm_toe_" .. index)
        or safeLoadRaw("ngp_arm_toe_" .. index)
        or safeLoadRaw("ngp_ca_toe_" .. index)

    if armCamber ~= nil or armToe ~= nil then
        M.state.armLinked = true
    end

    add = add - (abs(armCamber) + abs(armToe)) * p.armGeometryLoadLossGain

    local complianceLoss =
        safeLoadRaw("ngp_load_path_compliance_loss_" .. index)
        or safeLoadRaw("ngp_compliance_load_path_loss_" .. index)
        or safeLoadRaw("ngp_force_path_loss_" .. index)

    if complianceLoss ~= nil then
        M.state.complianceLinked = true
        add = add - clamp(safeNumber(complianceLoss, 0.0), 0.0, 1.0) * 120.0
    end

    if index >= 2 then
        local driveTorque =
            safeLoadRaw("ngp_drivetrain_torque")
            or safeLoadRaw("ngp_dt_torque")
            or safeLoadRaw("ngp_drive_soft_torque")
            or safeLoadRaw("ngp_drive_torque")

        local lsdLock =
            safeLoadRaw("ngp_lsd_lock")
            or safeLoadRaw("ngp_diff_lock")

        local windup =
            safeLoadRaw("ngp_driveline_windup")
            or safeLoadRaw("ngp_windup_value")

        if driveTorque ~= nil or lsdLock ~= nil or windup ~= nil then
            M.state.drivetrainLinked = true
        end

        if windup ~= nil then
            M.state.windupLinked = true
        end

        add =
            add
            + abs(driveTorque) * p.drivetrainRearLoadGain
            + clamp(safeNumber(lsdLock, 0.0), 0.0, 1.0) * p.lsdRearLoadGain
            + abs(windup) * 45.0
    end

    local conditionSusp =
        safeLoadRaw("ngp_condition_suspension")

    local damageWheel =
        safeLoadRaw("ngp_damage_wheel_" .. index)

    if conditionSusp ~= nil or damageWheel ~= nil then
        M.state.damageLinked = true
    end

    add =
        add
        - clamp(safeNumber(conditionSusp, 0.0), 0.0, 1.0) * p.conditionSuspLossGain
        - clamp(safeNumber(damageWheel, 0.0), 0.0, 1.0) * p.damageWheelLossGain

    return add
end

local function readSuspensionLoadAdd(index)
    local p = M.params

    local suspForce =
        safeLoadRaw("ngp_susp_integrated_force_" .. index)
        or safeLoadRaw("ngp_susp_int_force_" .. index)
        or safeLoadRaw("ngp_susp_" .. index)

    local damperForce =
        safeLoadRaw("ngp_damper_force_" .. index)
        or safeLoadRaw("ngp_damper_" .. index)

    local springForce =
        safeLoadRaw("ngp_progressive_spring_" .. index)

    local contactInput =
        safeLoadRaw("ngp_susp_contact_input_" .. index)
        or safeLoadRaw("ngp_sci_input_" .. index)

    local impulse =
        safeLoadRaw("ngp_susp_road_impulse_" .. index)
        or safeLoadRaw("ngp_sci_impulse_" .. index)

    local drop =
        safeLoadRaw("ngp_susp_contact_drop_" .. index)
        or safeLoadRaw("ngp_contact_hop_drop_" .. index)
        or safeLoadRaw("ngp_sci_drop_" .. index)

    if suspForce ~= nil
    or damperForce ~= nil
    or springForce ~= nil
    or contactInput ~= nil
    or impulse ~= nil
    or drop ~= nil then
        M.state.suspensionLinked = true
    end

    local add =
        safeNumber(suspForce, 0.0) * p.suspensionForceGain
        + safeNumber(damperForce, 0.0) * p.damperLoadGain
        + safeNumber(springForce, 0.0) * p.springLoadGain
        + clamp(safeNumber(contactInput, 0.0), 0.0, 1.5) * p.contactLoadGain
        + clamp(safeNumber(impulse, 0.0), 0.0, 1.5) * p.impulseLoadGain
        - clamp(safeNumber(drop, 0.0), 0.0, 1.0) * p.dropUnloadGain

    return add
end

local function updateIntegratedWheelLoads(car, dt)
    local p = M.params

    for i = 0, 3 do
        local rawLoad = readWheelLoad(car, i, p.wheelLoadRef)
        rawLoad = clamp(rawLoad, p.minWheelLoad, p.maxWheelLoad)
        M.debug.wheelLoadRaw[i] = rawLoad

        M.debug.wheelLoadFiltered[i] = lowPass(
            M.debug.wheelLoadFiltered[i],
            rawLoad,
            p.wheelLoadTau,
            dt
        )

        local modelAdd = (M.debug.load[i] or 0.0) * p.modelLoadGain
        M.debug.modelLoadAdd[i] = modelAdd

        local suspensionAdd = readSuspensionLoadAdd(i)
        M.debug.suspensionLoadAdd[i] = suspensionAdd

        local rootAdd = readRootWheelLoadAdd(i)
        M.debug.rootLoadAdd[i] = rootAdd

        local finalLoad =
            (M.debug.wheelLoadFiltered[i] or p.wheelLoadRef)
            + modelAdd
            + suspensionAdd
            + rootAdd

        finalLoad = clamp(finalLoad, p.minWheelLoad, p.maxWheelLoad)

        M.debug.wheelLoadFinal[i] = finalLoad
        M.debug.wheelLoadNorm[i] = clamp(finalLoad / math.max(p.wheelLoadRef, 1.0), 0.0, 2.5)
    end

    local fl = M.debug.wheelLoadFinal[0] or 0.0
    local fr = M.debug.wheelLoadFinal[1] or 0.0
    local rl = M.debug.wheelLoadFinal[2] or 0.0
    local rr = M.debug.wheelLoadFinal[3] or 0.0

    local total = math.max(fl + fr + rl + rr, 1.0)

    M.debug.absFront = (fl + fr) / total
    M.debug.absRear = (rl + rr) / total
    M.debug.absLeft = (fl + rl) / total
    M.debug.absRight = (fr + rr) / total

    M.state.avgWheelLoad = total * 0.25
    M.state.frontWheelLoad = (fl + fr) * 0.5
    M.state.rearWheelLoad = (rl + rr) * 0.5
    M.state.leftWheelLoad = (fl + rl) * 0.5
    M.state.rightWheelLoad = (fr + rr) * 0.5
    M.state.maxWheelLoadLive = math.max(fl, fr, rl, rr)
    M.state.minWheelLoadLive = math.min(fl, fr, rl, rr)
end

--============================================================
-- Perception mapping
--============================================================

local function updatePerceptionState()
    local pitchRatio = clamp(filtered.pitch / math.max(M.params.maxPitch, 1.0), -1.0, 1.0)
    local rollRatio = clamp(filtered.roll / math.max(M.params.maxRoll, 1.0), -1.0, 1.0)

    M.debug.pitchRatio = pitchRatio
    M.debug.rollRatio = rollRatio

    M.state.front = clamp(0.5 - pitchRatio * 0.5, 0.0, 1.0)
    M.state.rear = clamp(0.5 + pitchRatio * 0.5, 0.0, 1.0)

    M.state.left = clamp(0.5 - rollRatio * 0.5, 0.0, 1.0)
    M.state.right = clamp(0.5 + rollRatio * 0.5, 0.0, 1.0)

    if abs(pitchRatio) < 0.1 then
        M.state.pitchState = "NEUTRAL"
    elseif pitchRatio > 0.0 then
        M.state.pitchState = "BRAKE LOAD"
    else
        M.state.pitchState = "DRIVE LOAD"
    end

    if abs(rollRatio) < 0.1 then
        M.state.rollState = "NEUTRAL"
    elseif rollRatio > 0.0 then
        M.state.rollState = "RIGHT LOAD"
    else
        M.state.rollState = "LEFT LOAD"
    end

    if abs(pitchRatio) + abs(rollRatio) > 0.8 then
        M.state.transition = "LIMIT TRANSITION"
    else
        M.state.transition = "NORMAL"
    end
end

local function decayWhenUnavailable(dt, tau)
    filtered.pitch = lowPass(filtered.pitch, 0.0, tau, dt)
    filtered.roll = lowPass(filtered.roll, 0.0, tau, dt)
    filtered.rollFront = lowPass(filtered.rollFront, 0.0, tau, dt)
    filtered.rollRear = lowPass(filtered.rollRear, 0.0, tau, dt)

    for i = 0, 3 do
        M.debug.loadPitch[i] = lowPass(M.debug.loadPitch[i], 0.0, tau, dt)
        M.debug.loadRoll[i] = lowPass(M.debug.loadRoll[i], 0.0, tau, dt)
        M.debug.load[i] = lowPass(M.debug.load[i], 0.0, tau, dt)
        M.debug.modelLoadAdd[i] = lowPass(M.debug.modelLoadAdd[i], 0.0, tau, dt)
        M.debug.suspensionLoadAdd[i] = lowPass(M.debug.suspensionLoadAdd[i], 0.0, tau, dt)
        M.debug.rootLoadAdd[i] = lowPass(M.debug.rootLoadAdd[i], 0.0, tau, dt)
        M.debug.wheelLoadFiltered[i] = lowPass(M.debug.wheelLoadFiltered[i], M.params.wheelLoadRef, tau, dt)
        M.debug.wheelLoadFinal[i] = lowPass(M.debug.wheelLoadFinal[i], M.params.wheelLoadRef, tau, dt)
        M.debug.wheelLoadNorm[i] = clamp((M.debug.wheelLoadFinal[i] or M.params.wheelLoadRef) / math.max(M.params.wheelLoadRef, 1.0), 0.0, 2.5)
    end

    updateIntegratedWheelLoads(nil, dt)
    updatePerceptionState()
end

--============================================================
-- Export
--============================================================

local function exportState()
    for i = 0, 3 do
        safeStore("ngp_dlt_load_" .. i, M.debug.load[i] or 0.0)
        safeStore("ngp_dlt_pitch_load_" .. i, M.debug.loadPitch[i] or 0.0)
        safeStore("ngp_dlt_roll_load_" .. i, M.debug.loadRoll[i] or 0.0)

        safeStore("ngp_wheel_load_" .. i, M.debug.wheelLoadFinal[i] or 0.0)
        safeStore("ngp_wheel_load_raw_" .. i, M.debug.wheelLoadRaw[i] or 0.0)
        safeStore("ngp_wheel_load_model_" .. i, M.debug.modelLoadAdd[i] or 0.0)
        safeStore("ngp_wheel_load_susp_" .. i, M.debug.suspensionLoadAdd[i] or 0.0)
        safeStore("ngp_wheel_load_root_" .. i, M.debug.rootLoadAdd[i] or 0.0)
        safeStore("ngp_wheel_load_norm_" .. i, M.debug.wheelLoadNorm[i] or 1.0)

        safeStore("ngp_load_wheel_" .. i, M.debug.wheelLoadFinal[i] or 0.0)
        safeStore("ngp_tire_load_input_" .. i, M.debug.wheelLoadFinal[i] or 0.0)
        safeStore("ngp_susp_load_input_" .. i, M.debug.wheelLoadFinal[i] or 0.0)

        safeStore("ngp_load_transfer_load_" .. i, M.debug.wheelLoadFinal[i] or 0.0)
        safeStore("ngp_load_transfer_delta_" .. i, M.debug.load[i] or 0.0)
    end

    safeStore("ngp_load_abs_front", M.debug.absFront or 0.5)
    safeStore("ngp_load_abs_rear", M.debug.absRear or 0.5)
    safeStore("ngp_load_abs_left", M.debug.absLeft or 0.5)
    safeStore("ngp_load_abs_right", M.debug.absRight or 0.5)

    safeStore("ngp_dlt_pitch", filtered.pitch or 0.0)
    safeStore("ngp_dlt_roll", filtered.roll or 0.0)
    safeStore("ngp_dlt_roll_front", filtered.rollFront or 0.0)
    safeStore("ngp_dlt_roll_rear", filtered.rollRear or 0.0)

    safeStore("ngp_load_front", M.state.front or 0.5)
    safeStore("ngp_load_rear", M.state.rear or 0.5)
    safeStore("ngp_load_left", M.state.left or 0.5)
    safeStore("ngp_load_right", M.state.right or 0.5)

    safeStore("ngp_front_bias", M.state.front or 0.5)
    safeStore("ngp_rear_bias", M.state.rear or 0.5)
    safeStore("ngp_left_bias", M.state.left or 0.5)
    safeStore("ngp_right_bias", M.state.right or 0.5)

    safeStore("ngp_load_transfer_status", M.state.status or "UNKNOWN")
    safeStore("ngp_load_transfer_update_count", M.state.updateCount or 0)
    safeStore("ngp_load_transfer_wheels_valid", M.state.wheelsValid and 1 or 0)

    safeStore("ngp_load_transfer_ax", M.debug.ax or 0.0)
    safeStore("ngp_load_transfer_az", M.debug.az or 0.0)
    safeStore("ngp_load_transfer_mass", M.debug.mass or 0.0)
    safeStore("ngp_load_transfer_cgh", M.debug.cgH or 0.0)

    safeStore("ngp_load_transfer_raw_pitch", M.debug.rawPitch or 0.0)
    safeStore("ngp_load_transfer_raw_roll", M.debug.rawRoll or 0.0)
    safeStore("ngp_load_transfer_raw_roll_front", M.debug.rawRollFront or 0.0)
    safeStore("ngp_load_transfer_raw_roll_rear", M.debug.rawRollRear or 0.0)

    safeStore("ngp_load_transfer_filt_pitch", M.debug.filtPitch or 0.0)
    safeStore("ngp_load_transfer_filt_roll", M.debug.filtRoll or 0.0)
    safeStore("ngp_load_transfer_filt_roll_front", M.debug.filtRollFront or 0.0)
    safeStore("ngp_load_transfer_filt_roll_rear", M.debug.filtRollRear or 0.0)

    safeStore("ngp_load_transfer_roll_front_bias", M.params.rollStiffnessFrontBias or 0.58)
    safeStore("ngp_load_transfer_pitch_ratio", M.debug.pitchRatio or 0.0)
    safeStore("ngp_load_transfer_roll_ratio", M.debug.rollRatio or 0.0)
    safeStore("ngp_load_transfer_wheelbase", geometry.wheelbase or 2.50)
    safeStore("ngp_load_transfer_track_f", geometry.trackF or 1.45)
    safeStore("ngp_load_transfer_track_r", geometry.trackR or 1.42)

    safeStore("ngp_load_transfer_body_rigidity_linked", M.state.bodyRigidityLinked and 1 or 0)
    safeStore("ngp_load_transfer_body_rigidity", M.state.bodyRigidity or 1.0)
    safeStore("ngp_load_transfer_body_flex_factor", M.state.bodyFlexFactor or 1.0)
    safeStore("ngp_load_transfer_body_pitch_scale", M.state.bodyPitchScale or 1.0)
    safeStore("ngp_load_transfer_body_roll_scale", M.state.bodyRollScale or 1.0)
    safeStore("ngp_load_transfer_body_pitch_tau", M.state.bodyPitchTau or M.params.tauPitch)
    safeStore("ngp_load_transfer_body_roll_tau", M.state.bodyRollTau or M.params.tauRoll)

    safeStore("ngp_load_transfer_avg_wheel_load", M.state.avgWheelLoad or M.params.wheelLoadRef)
    safeStore("ngp_load_transfer_front_wheel_load", M.state.frontWheelLoad or M.params.wheelLoadRef)
    safeStore("ngp_load_transfer_rear_wheel_load", M.state.rearWheelLoad or M.params.wheelLoadRef)
    safeStore("ngp_load_transfer_left_wheel_load", M.state.leftWheelLoad or M.params.wheelLoadRef)
    safeStore("ngp_load_transfer_right_wheel_load", M.state.rightWheelLoad or M.params.wheelLoadRef)
    safeStore("ngp_load_transfer_max_wheel_load", M.state.maxWheelLoadLive or M.params.wheelLoadRef)
    safeStore("ngp_load_transfer_min_wheel_load", M.state.minWheelLoadLive or M.params.wheelLoadRef)

    safeStore("ngp_lt_status", M.state.status or "UNKNOWN")
    safeStore("ngp_lt_count", M.state.updateCount or 0)
    safeStore("ngp_lt_front", M.state.front or 0.5)
    safeStore("ngp_lt_rear", M.state.rear or 0.5)
    safeStore("ngp_lt_left", M.state.left or 0.5)
    safeStore("ngp_lt_right", M.state.right or 0.5)

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_load_transfer_tire_linked", M.state.tireLinked and 1 or 0)
    safeStore("ngp_load_transfer_suspension_linked", M.state.suspensionLinked and 1 or 0)
    safeStore("ngp_load_transfer_drivetrain_linked", M.state.drivetrainLinked and 1 or 0)
    safeStore("ngp_load_transfer_arm_linked", M.state.armLinked and 1 or 0)
    safeStore("ngp_load_transfer_damage_linked", M.state.damageLinked and 1 or 0)
    safeStore("ngp_load_transfer_contact_linked", M.state.contactLinked and 1 or 0)
    safeStore("ngp_load_transfer_damper_linked", M.state.damperLinked and 1 or 0)
    safeStore("ngp_load_transfer_compliance_linked", M.state.complianceLinked and 1 or 0)
    safeStore("ngp_load_transfer_windup_linked", M.state.windupLinked and 1 or 0)
end

--============================================================
-- Lifecycle
--============================================================

function M.init(car)
    initGeometry(false)
    setStatus("INIT")
    exportState()
end

function M.update(dt, car, runtime)
    M.state.updateCount = (M.state.updateCount or 0) + 1
    M.debug.updateCount = M.state.updateCount

    dt = safeNumber(dt, 0.0)

    if dt <= 0.0 then
        setStatus("BAD DT")
        exportState()
        return
    end

    dt = clamp(dt, M.params.minDt, M.params.maxDt)

    updateDebugGate(dt)
    updateGeometryRetry(dt)

    car = car or safeGetCar()

    if not car then
        setStatus("NO CAR")
        M.state.wheelsValid = false
        decayWhenUnavailable(dt, M.params.noCarDecayTau)
        exportState()
        return
    end

    if not hasWheels(car) then
        setStatus("NO WHEELS")
        M.state.wheelsValid = false
        decayWhenUnavailable(dt, M.params.noWheelsDecayTau)
        exportState()
        return
    end

    setStatus("RUNNING")
    M.state.wheelsValid = true

    M.state.tireLinked = false
    M.state.suspensionLinked = false
    M.state.drivetrainLinked = false
    M.state.armLinked = false
    M.state.damageLinked = false
    M.state.contactLinked = false
    M.state.damperLinked = false
    M.state.complianceLinked = false
    M.state.windupLinked = false

    local ax, az = readAcceleration(car)
    local mass = readMass(car)
    local cgH = readCGHeight()

    local rawPitch, rawRoll, rawRollFront, rawRollRear = calculateRawTransfer(ax, az, mass, cgH)

    readBodyRigidity(dt)

    local bodyPitch, bodyRoll = applyBodyRigidityToTransfer(rawPitch, rawRoll, dt)

    local rollScale = 1.0
    if abs(rawRoll) > 0.001 then
        rollScale = bodyRoll / rawRoll
    end

    local bodyRollFront = rawRollFront * rollScale
    local bodyRollRear = rawRollRear * rollScale

    updateFilteredTransfer(bodyPitch, bodyRoll, bodyRollFront, bodyRollRear, dt)
    distributeWheelLoads()
    applyTireHopLoadPulse()
    updateIntegratedWheelLoads(car, dt)

    M.debug.ax = ax
    M.debug.az = az
    M.debug.mass = mass
    M.debug.cgH = cgH
    M.debug.rawPitch = bodyPitch
    M.debug.rawRoll = bodyRoll
    M.debug.rawRollFront = bodyRollFront
    M.debug.rawRollRear = bodyRollRear
    M.debug.bodyPitchScale = M.state.bodyPitchScale or 1.0
    M.debug.bodyRollScale = M.state.bodyRollScale or 1.0
    M.debug.bodyPitchTau = M.state.bodyPitchTau or M.params.tauPitch
    M.debug.bodyRollTau = M.state.bodyRollTau or M.params.tauRoll

    updatePerceptionState()
    exportState()
end

--============================================================
-- Public API
--============================================================

function M.getWheelLoad(index)
    index = tonumber(index) or 0
    return M.debug.wheelLoadFinal[index] or M.params.wheelLoadRef
end

function M.getDeltaLoad(index)
    index = tonumber(index) or 0
    return M.debug.load[index] or 0.0
end

function M.getPitch()
    return filtered.pitch or 0.0
end

function M.getRoll()
    return filtered.roll or 0.0
end

function M.getFrontBias()
    return M.state.front or 0.5
end

function M.getRearBias()
    return M.state.rear or 0.5
end

function M.getState()
    return M.state
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "Mass:%.0f CGH:%.3f\n" ..
        "Pitch raw:%+.0f filt:%+.0f %s\n" ..
        "Roll  raw:%+.0f filt:%+.0f %s\n" ..
        "RollSplit F:%+.0f R:%+.0f Bias:%.2f\n" ..
        "DLT  FL:%+.0f FR:%+.0f RL:%+.0f RR:%+.0f\n" ..
        "LOAD FL:%.0f FR:%.0f RL:%.0f RR:%.0f\n" ..
        "NORM %.2f %.2f %.2f %.2f\n" ..
        "ABS F/R %.2f/%.2f  L/R %.2f/%.2f\n" ..
        "BodyRig:%s Rig %.2f Flex %.2f PScale %.2f RScale %.2f\n" ..
        "Links Tire:%s Susp:%s Drive:%s Arm:%s Damage:%s Wind:%s",

        tostring(M.state.status),
        M.state.updateCount or 0,
        M.state.wheelsValid and "OK" or "NIL",

        M.debug.mass or 0.0,
        M.debug.cgH or 0.0,

        M.debug.rawPitch or 0.0,
        M.debug.filtPitch or 0.0,
        tostring(M.state.pitchState),

        M.debug.rawRoll or 0.0,
        M.debug.filtRoll or 0.0,
        tostring(M.state.rollState),

        M.debug.filtRollFront or 0.0,
        M.debug.filtRollRear or 0.0,
        M.params.rollStiffnessFrontBias or 0.58,

        M.debug.load[0] or 0.0,
        M.debug.load[1] or 0.0,
        M.debug.load[2] or 0.0,
        M.debug.load[3] or 0.0,

        M.debug.wheelLoadFinal[0] or 0.0,
        M.debug.wheelLoadFinal[1] or 0.0,
        M.debug.wheelLoadFinal[2] or 0.0,
        M.debug.wheelLoadFinal[3] or 0.0,

        M.debug.wheelLoadNorm[0] or 1.0,
        M.debug.wheelLoadNorm[1] or 1.0,
        M.debug.wheelLoadNorm[2] or 1.0,
        M.debug.wheelLoadNorm[3] or 1.0,

        M.debug.absFront or 0.5,
        M.debug.absRear or 0.5,
        M.debug.absLeft or 0.5,
        M.debug.absRight or 0.5,

        M.state.bodyRigidityLinked and "YES" or "NO",
        M.state.bodyRigidity or 1.0,
        M.state.bodyFlexFactor or 1.0,
        M.state.bodyPitchScale or 1.0,
        M.state.bodyRollScale or 1.0,

        M.state.tireLinked and "OK" or "NIL",
        M.state.suspensionLinked and "OK" or "NIL",
        M.state.drivetrainLinked and "OK" or "NIL",
        M.state.armLinked and "OK" or "NIL",
        M.state.damageLinked and "OK" or "NIL",
        M.state.windupLinked and "OK" or "NIL"
    )
end

return M
