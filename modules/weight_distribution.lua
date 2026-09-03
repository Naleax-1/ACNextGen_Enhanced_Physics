---@diagnostic disable: undefined-global

--============================================================
-- ACNeXtGen
-- weight_distribution.lua
-- Phase 15.4 / V1.1.5 stable
-- Dynamic Weight Distribution
--============================================================

local M = {}

--============================================================
-- Parameters
--============================================================

M.params = {
    pitchGain = 1.50,
    rollGain  = 1.50,

    minBias = 0.35,
    maxBias = 0.65,

    neutralThreshold = 0.01,

    biasTau = 0.060,

    loadTransferPitchGain = 0.18,
    loadTransferRollGain = 0.18,

    chassisRollGain = 0.10,
    chassisPitchGain = 0.10,
    chassisHeaveGain = 0.04,
    chassisYawGain = 0.03,

    virtualPitchGain = 0.06,
    virtualRollGain = 0.06,
    virtualYawGain = 0.03,

    brakePitchBiasGain = 0.035,
    brakeLockPitchGain = 0.020,
    driveRearBiasGain = 0.030,
    shaftRearBiasGain = 0.012,
    lsdRearStabilizeGain = 0.010,

    suspensionStressBiasGain = 0.035,
    tireContactBiasGain = 0.025,
    roadInputBiasGain = 0.020,
    loadPathLossBiasGain = 0.018,

    bodyFlexBiasGain = 0.025,
    bodyRuntimeBiasGain = 0.025,
    bodyRigidityRecoverGain = 0.010,

    massRearShiftGain = 0.050,
    massYawBiasGain = 0.010,

    maxRootBiasAdd = 0.08,

    defaultWheelLoad = 3500.0,
    wheelLoadReference = 3500.0,
    driveTorqueReference = 2500.0,
    suspensionForceReference = 25000.0,

    minDt = 0.00005,
    maxDt = 0.050,

    storeOnlyDecayTau = 0.75,
    debugStoreInterval = 0.25,
}

--============================================================
-- State
--============================================================

M.state = {
    frontBias = 0.50,
    rearBias  = 0.50,

    leftBias  = 0.50,
    rightBias = 0.50,

    targetFrontBias = 0.50,
    targetRearBias  = 0.50,

    targetLeftBias  = 0.50,
    targetRightBias = 0.50,

    pitchCG = 0.0,
    rollCG  = 0.0,

    stateName = "NEUTRAL",
    rollName  = "NEUTRAL",

    status = "INIT",
    updateCount = 0,

    linkedCG = false,
    wheelsValid = false,
    storeOnly = false,

    speedKmh = 0.0,

    loadPitch = 0.0,
    loadRoll = 0.0,
    loadYaw = 0.0,

    chassisRoll = 0.0,
    chassisPitch = 0.0,
    chassisHeave = 0.0,
    chassisYaw = 0.0,

    virtualPitch = 0.0,
    virtualRoll = 0.0,
    virtualYaw = 0.0,

    brakeInput = 0.0,
    brakeLockAvg = 0.0,
    driveTorqueNorm = 0.0,
    shaftTwist = 0.0,
    lsdLock = 0.0,

    suspensionStress = 0.0,
    suspensionInputAvg = 0.0,
    roadInputAvg = 0.0,
    loadPathLossAvg = 0.0,
    contactLossAvg = 0.0,

    bodyFlexFactor = 1.0,
    bodyRuntimePenalty = 0.0,
    bodyRigidity = 1.0,

    massScale = 1.0,
    cgRearShift = 0.0,
    yawInertiaScale = 1.0,

    wheelLoad = { [0]=3500.0, [1]=3500.0, [2]=3500.0, [3]=3500.0 },
    maxWheelLoad = 3500.0,
    avgWheelLoad = 3500.0,

    rootFrontAdd = 0.0,
    rootRearAdd = 0.0,
    rootLeftAdd = 0.0,
    rootRightAdd = 0.0,

    activeRootInputs = 0,

    loadLinked = false,
    chassisLinked = false,
    virtualLinked = false,
    brakeLinked = false,
    drivetrainLinked = false,
    suspensionLinked = false,
    tireLinked = false,
    bodyLinked = false,
    massLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.debug = M.state

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

local function abs(value)
    value = safeNumber(value, 0.0)
    if value < 0.0 then return -value end
    return value
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

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target  = safeNumber(target, 0.0)
    tau     = safeNumber(tau, 0.0)
    dt      = safeNumber(dt, 0.0)

    if tau <= 0.0 then
        return target
    end

    local k = dt / (tau + dt)
    k = clamp(k, 0.0, 1.0)

    return current + (target - current) * k
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

local function getWheel(car, index)
    if not car then
        return nil
    end

    local wheels = safeField(car, "wheels", nil)
    if not wheels then
        return nil
    end

    local ok, wheel = pcall(function()
        return wheels[index]
    end)

    if ok and wheel then
        return wheel
    end

    ok, wheel = pcall(function()
        return wheels[index + 1]
    end)

    if ok then
        return wheel
    end

    return nil
end

local function readWheelNumber(wheel, key, defaultValue)
    local value = safeField(wheel, key, nil)
    if value == nil then
        return defaultValue
    end
    return safeNumber(value, defaultValue)
end

local function updateDebugGate(dt)
    M.state.debugStoreTimer = (M.state.debugStoreTimer or 0.0) + (dt or 0.0)

    if M.state.debugStoreTimer >= (M.params.debugStoreInterval or 0.25) then
        M.state.debugStoreTimer = 0.0
        M.state.debugStoreNow = true
    else
        M.state.debugStoreNow = false
    end
end

local function resetLinks()
    M.state.linkedCG = false
    M.state.loadLinked = false
    M.state.chassisLinked = false
    M.state.virtualLinked = false
    M.state.brakeLinked = false
    M.state.drivetrainLinked = false
    M.state.suspensionLinked = false
    M.state.tireLinked = false
    M.state.bodyLinked = false
    M.state.massLinked = false
    M.state.activeRootInputs = 0
end

local function mark(flagName, active)
    if active then
        M.state[flagName] = true
        M.state.activeRootInputs = (M.state.activeRootInputs or 0) + 1
    end
end

--============================================================
-- Input readers
--============================================================

local function readSpeed(car)
    local speedKmh = safeField(car, "speedKmh", nil)
    if speedKmh ~= nil then
        M.state.speedKmh = abs(speedKmh)
        return
    end

    local speed = nil
    if speed ~= nil then
        M.state.speedKmh = abs(speed) * 3.6
        return
    end

    M.state.speedKmh = 0.0
end

local function readCG()
    local rawPitch =
        safeLoadRaw("ngp_pitch_cg") or
        safeLoadRaw("ngp_sprung_pitch_state") or
        safeLoadRaw("ngp_sprung_pitch_cg") or
        safeLoadRaw("ngp_sm_pitch_cg")

    local rawRoll =
        safeLoadRaw("ngp_roll_cg") or
        safeLoadRaw("ngp_sprung_roll_state") or
        safeLoadRaw("ngp_sprung_roll_cg") or
        safeLoadRaw("ngp_sm_roll_cg")

    M.state.linkedCG = rawPitch ~= nil or rawRoll ~= nil

    M.state.pitchCG = clamp(safeNumber(rawPitch, 0.0), -0.30, 0.30)
    M.state.rollCG = clamp(safeNumber(rawRoll, 0.0), -0.30, 0.30)
end

local function loadDltAsAbsolute(index)
    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt == nil then
        return nil
    end
    return M.params.defaultWheelLoad + safeNumber(dlt, 0.0)
end

local function readWheelLoad(car, index)
    local candidates = {
        safeLoadRaw("ngp_weight_wheel_load_" .. index),
        safeLoadRaw("ngp_wheel_load_" .. index),
        safeLoadRaw("ngp_load_wheel_" .. index),
        safeLoadRaw("ngp_tire_load_" .. index),
        safeLoadRaw("ngp_tire_state_load_" .. index),
        safeLoadRaw("ngp_tdyn_load_" .. index),
        safeLoadRaw("ngp_contact_load_" .. index),
        loadDltAsAbsolute(index),
    }

    for _, value in ipairs(candidates) do
        if value ~= nil then
            M.state.loadLinked = true
            return clamp(safeNumber(value, M.params.defaultWheelLoad), 0.0, 12000.0)
        end
    end

    local wheel = getWheel(car, index)
    if wheel then
        local raw =
            readWheelNumber(wheel, "load", nil) or
            readWheelNumber(wheel, "loadK", nil)

        if raw ~= nil then
            return clamp(safeNumber(raw, M.params.defaultWheelLoad), 0.0, 12000.0)
        end
    end

    return M.params.defaultWheelLoad
end

local function normalizePair(aRaw, bRaw, defaultA, defaultB)
    if aRaw == nil and bRaw == nil then
        return defaultA or 0.5, defaultB or 0.5, false
    end

    local a = safeNumber(aRaw, defaultA or 0.5)
    local b = safeNumber(bRaw, defaultB or 0.5)

    if a < 0.0 then a = 0.0 end
    if b < 0.0 then b = 0.0 end

    local sum = a + b
    if sum > 1.5 then
        if sum > 0.0001 then
            return a / sum, b / sum, true
        end
        return 0.5, 0.5, true
    end

    return clamp(a, 0.0, 1.0), clamp(b, 0.0, 1.0), true
end

local function readLoadTransfer(car)
    local frontRaw =
        safeLoadRaw("ngp_load_front") or
        safeLoadRaw("ngp_load_abs_front") or
        safeLoadRaw("ngp_weight_input_front")

    local rearRaw =
        safeLoadRaw("ngp_load_rear") or
        safeLoadRaw("ngp_load_abs_rear") or
        safeLoadRaw("ngp_weight_input_rear")

    local leftRaw =
        safeLoadRaw("ngp_load_left") or
        safeLoadRaw("ngp_load_abs_left") or
        safeLoadRaw("ngp_weight_input_left")

    local rightRaw =
        safeLoadRaw("ngp_load_right") or
        safeLoadRaw("ngp_load_abs_right") or
        safeLoadRaw("ngp_weight_input_right")

    local front, rear, linkedFR = normalizePair(frontRaw, rearRaw, 0.5, 0.5)
    local left, right, linkedLR = normalizePair(leftRaw, rightRaw, 0.5, 0.5)

    if not linkedFR or not linkedLR then
        local fl = readWheelLoad(car, 0)
        local fr = readWheelLoad(car, 1)
        local rl = readWheelLoad(car, 2)
        local rr = readWheelLoad(car, 3)

        M.state.wheelLoad[0] = fl
        M.state.wheelLoad[1] = fr
        M.state.wheelLoad[2] = rl
        M.state.wheelLoad[3] = rr

        local total = math.max(fl + fr + rl + rr, 0.0001)
        local fAbs = fl + fr
        local rAbs = rl + rr
        local lAbs = fl + rl
        local rSideAbs = fr + rr

        if not linkedFR then
            front = fAbs / total
            rear = rAbs / total
        end

        if not linkedLR then
            left = lAbs / total
            right = rSideAbs / total
        end

        M.state.maxWheelLoad = math.max(fl, fr, rl, rr)
        M.state.avgWheelLoad = total * 0.25
    end

    M.state.loadPitch = clamp(front - rear, -1.0, 1.0)
    M.state.loadRoll = clamp(left - right, -1.0, 1.0)
    M.state.loadYaw = clamp(M.state.loadRoll * 0.65 + M.state.loadPitch * 0.35, -1.0, 1.0)

    mark("loadLinked", linkedFR or linkedLR or M.state.loadLinked)
end

local function readChassisAndVirtual()
    local cr = safeLoadRaw("ngp_chassis_roll") or safeLoadRaw("ngp_body_roll")
    local cp = safeLoadRaw("ngp_chassis_pitch") or safeLoadRaw("ngp_body_pitch")
    local ch = safeLoadRaw("ngp_chassis_heave") or safeLoadRaw("ngp_body_heave")
    local cy = safeLoadRaw("ngp_chassis_yaw_hint") or safeLoadRaw("ngp_yaw_deflection")

    M.state.chassisRoll = clamp(safeNumber(cr, 0.0), -1.5, 1.5)
    M.state.chassisPitch = clamp(safeNumber(cp, 0.0), -1.5, 1.5)
    M.state.chassisHeave = clamp(safeNumber(ch, 0.0), -1.5, 1.5)
    M.state.chassisYaw = clamp(safeNumber(cy, 0.0), -1.5, 1.5)

    mark("chassisLinked", cr ~= nil or cp ~= nil or ch ~= nil or cy ~= nil)

    local vp = safeLoadRaw("ngp_virtual_pitch") or safeLoadRaw("ngp_vi_pitch")
    local vr = safeLoadRaw("ngp_virtual_roll") or safeLoadRaw("ngp_vi_roll")
    local vy = safeLoadRaw("ngp_virtual_yaw") or safeLoadRaw("ngp_vi_yaw")

    M.state.virtualPitch = clamp(safeNumber(vp, 0.0), -2.5, 2.5)
    M.state.virtualRoll = clamp(safeNumber(vr, 0.0), -2.5, 2.5)
    M.state.virtualYaw = clamp(safeNumber(vy, 0.0), -2.5, 2.5)

    mark("virtualLinked", vp ~= nil or vr ~= nil or vy ~= nil)
end

local function readBrakeAndDrivetrain(car)
    local brakeRaw =
        safeLoadRaw("ngp_brake_input") or
        safeLoadRaw("ngp_brake_pedal")

    M.state.brakeInput = clamp(safeNumber(brakeRaw, safeNumber(safeField(car, "brake", 0.0), 0.0)), 0.0, 1.0)

    local lockSum = 0.0
    local lockLinked = brakeRaw ~= nil
    for i = 0, 3 do
        local lockRaw = safeLoadRaw("ngp_brake_lock_" .. i)
        if lockRaw == nil then lockRaw = safeLoadRaw("ngp_brake_lock_ratio_" .. i) end
        if lockRaw ~= nil then lockLinked = true end
        lockSum = lockSum + clamp(safeNumber(lockRaw, 0.0), 0.0, 1.0)
    end
    M.state.brakeLockAvg = lockSum * 0.25
    mark("brakeLinked", lockLinked)

    local torqueRaw =
        safeLoadRaw("ngp_drive_torque_normalized_from_nm") or
        safeLoadRaw("ngp_drive_torque") or
        safeLoadRaw("ngp_drivetrain_torque")

    local torqueNm =
        safeLoadRaw("ngp_drive_torque_nm") or
        safeLoadRaw("ngp_diff_input_torque_nm") or
        safeLoadRaw("ngp_drivetrain_transmitted_torque")

    local shaft =
        safeLoadRaw("ngp_shaft_twist") or
        safeLoadRaw("ngp_windup_twist") or
        safeLoadRaw("ngp_driveline_twist")

    local lsd =
        safeLoadRaw("ngp_lsd_lock") or
        safeLoadRaw("ngp_diff_lock")

    local normFromTorque = abs(safeNumber(torqueRaw, 0.0))
    if normFromTorque > 1.2 then
        normFromTorque = normFromTorque / math.max(M.params.driveTorqueReference, 1.0)
    end

    local normFromNm = abs(safeNumber(torqueNm, 0.0)) / math.max(M.params.driveTorqueReference, 1.0)

    M.state.driveTorqueNorm = clamp(math.max(normFromTorque, normFromNm), 0.0, 1.0)
    M.state.shaftTwist = clamp(safeNumber(shaft, 0.0), -2.0, 2.0)
    M.state.lsdLock = clamp(safeNumber(lsd, 0.0), 0.0, 1.0)

    mark("drivetrainLinked", torqueRaw ~= nil or torqueNm ~= nil or shaft ~= nil or lsd ~= nil)
end

local function readSuspensionAndTire()
    local suspSum = 0.0
    local suspInputSum = 0.0
    local roadInputSum = 0.0
    local lossSum = 0.0
    local pathLossSum = 0.0
    local suspLinked = false
    local tireLinked = false

    for i = 0, 3 do
        local susp =
            safeLoadRaw("ngp_susp_integrated_force_" .. i) or
            safeLoadRaw("ngp_susp_int_force_" .. i) or
            safeLoadRaw("ngp_susp_" .. i) or
            safeLoadRaw("ngp_damper_model_force_" .. i) or
            safeLoadRaw("ngp_damper_" .. i)

        if susp ~= nil then suspLinked = true end
        suspSum = suspSum + math.min(abs(safeNumber(susp, 0.0)) / math.max(M.params.suspensionForceReference, 1.0), 1.0)

        local suspInput =
            safeLoadRaw("ngp_susp_contact_input_" .. i) or
            safeLoadRaw("ngp_susp_tire_hop_input_" .. i) or
            safeLoadRaw("ngp_sci_input_" .. i)

        if suspInput ~= nil then suspLinked = true end
        suspInputSum = suspInputSum + abs(safeNumber(suspInput, 0.0))

        local road =
            safeLoadRaw("ngp_tire_force_road_input_" .. i) or
            safeLoadRaw("ngp_tf_road_input_" .. i) or
            safeLoadRaw("ngp_road_severity_" .. i) or
            safeLoadRaw("ngp_rii_severity_" .. i)

        if road ~= nil then tireLinked = true end
        roadInputSum = roadInputSum + clamp(safeNumber(road, 0.0), 0.0, 1.5)

        local loss =
            safeLoadRaw("ngp_tire_contact_loss_" .. i) or
            safeLoadRaw("ngp_tcr_contact_loss_" .. i) or
            safeLoadRaw("ngp_contact_loss_" .. i)

        if loss ~= nil then tireLinked = true end
        lossSum = lossSum + clamp(safeNumber(loss, 0.0), 0.0, 1.0)

        local pathLoss =
            safeLoadRaw("ngp_load_path_loss_" .. i) or
            safeLoadRaw("ngp_lp_loss_" .. i)

        if pathLoss ~= nil then tireLinked = true end
        pathLossSum = pathLossSum + clamp(safeNumber(pathLoss, 0.0), 0.0, 1.0)
    end

    M.state.suspensionStress = clamp(suspSum * 0.25, 0.0, 1.0)
    M.state.suspensionInputAvg = clamp(suspInputSum * 0.25, 0.0, 1.5)
    M.state.roadInputAvg = clamp(roadInputSum * 0.25, 0.0, 1.5)
    M.state.contactLossAvg = clamp(lossSum * 0.25, 0.0, 1.0)
    M.state.loadPathLossAvg = clamp(pathLossSum * 0.25, 0.0, 1.0)

    mark("suspensionLinked", suspLinked)
    mark("tireLinked", tireLinked)
end

local function readBodyAndMass()
    local bodyFlex =
        safeLoadRaw("ngp_body_flex_factor") or
        safeLoadRaw("ngp_chassis_flex_factor")

    local bodyPenalty =
        safeLoadRaw("ngp_body_runtime_flex_penalty") or
        safeLoadRaw("ngp_body_root_flex_penalty")

    local rigidity =
        safeLoadRaw("ngp_body_rigidity") or
        safeLoadRaw("ngp_chassis_rigidity")

    M.state.bodyFlexFactor = clamp(safeNumber(bodyFlex, 1.0), 0.25, 1.80)
    M.state.bodyRuntimePenalty = clamp(safeNumber(bodyPenalty, 0.0), 0.0, 0.60)
    M.state.bodyRigidity = clamp(safeNumber(rigidity, 1.0), 0.20, 1.35)

    mark("bodyLinked", bodyFlex ~= nil or bodyPenalty ~= nil or rigidity ~= nil)

    local massScale =
        safeLoadRaw("ngp_vehicle_mass_scale") or
        safeLoadRaw("ngp_mass_scale")

    local rearShift =
        safeLoadRaw("ngp_vehicle_cg_rear_shift") or
        safeLoadRaw("ngp_front_axis_rear_shift") or
        safeLoadRaw("ngp_mass_rear_shift")

    local yawScale =
        safeLoadRaw("ngp_vehicle_yaw_inertia_scale") or
        safeLoadRaw("ngp_mass_yaw_inertia_scale")

    M.state.massScale = clamp(safeNumber(massScale, 1.0), 0.80, 1.30)
    M.state.cgRearShift = clamp(safeNumber(rearShift, 0.0), -0.30, 0.30)
    M.state.yawInertiaScale = clamp(safeNumber(yawScale, 1.0), 0.75, 1.45)

    mark("massLinked", massScale ~= nil or rearShift ~= nil or yawScale ~= nil)
end

local function readRootInputs(car)
    readLoadTransfer(car)
    readChassisAndVirtual()
    readBrakeAndDrivetrain(car)
    readSuspensionAndTire()
    readBodyAndMass()

    local p = M.params

    local frontAdd =
        M.state.loadPitch * p.loadTransferPitchGain +
        M.state.chassisPitch * p.chassisPitchGain +
        M.state.virtualPitch * p.virtualPitchGain +
        M.state.brakeInput * p.brakePitchBiasGain +
        M.state.brakeLockAvg * p.brakeLockPitchGain +
        M.state.suspensionInputAvg * p.chassisHeaveGain

    local rearAdd =
        -frontAdd +
        M.state.driveTorqueNorm * p.driveRearBiasGain +
        abs(M.state.shaftTwist) * p.shaftRearBiasGain +
        M.state.cgRearShift * p.massRearShiftGain -
        M.state.lsdLock * p.lsdRearStabilizeGain

    local leftAdd =
        M.state.loadRoll * p.loadTransferRollGain +
        M.state.chassisRoll * p.chassisRollGain +
        M.state.virtualRoll * p.virtualRollGain +
        M.state.chassisYaw * p.chassisYawGain +
        M.state.virtualYaw * p.virtualYawGain +
        (M.state.yawInertiaScale - 1.0) * p.massYawBiasGain

    local rightAdd = -leftAdd

    local stressAdd =
        M.state.suspensionStress * p.suspensionStressBiasGain +
        M.state.contactLossAvg * p.tireContactBiasGain +
        M.state.roadInputAvg * p.roadInputBiasGain +
        M.state.loadPathLossAvg * p.loadPathLossBiasGain +
        math.max(M.state.bodyFlexFactor - 1.0, 0.0) * p.bodyFlexBiasGain +
        M.state.bodyRuntimePenalty * p.bodyRuntimeBiasGain -
        math.max(M.state.bodyRigidity - 1.0, 0.0) * p.bodyRigidityRecoverGain

    frontAdd = frontAdd + stressAdd
    rearAdd = rearAdd + stressAdd

    M.state.rootFrontAdd = clamp(frontAdd, -p.maxRootBiasAdd, p.maxRootBiasAdd)
    M.state.rootRearAdd = clamp(rearAdd, -p.maxRootBiasAdd, p.maxRootBiasAdd)
    M.state.rootLeftAdd = clamp(leftAdd, -p.maxRootBiasAdd, p.maxRootBiasAdd)
    M.state.rootRightAdd = clamp(rightAdd, -p.maxRootBiasAdd, p.maxRootBiasAdd)
end

--============================================================
-- Core calculation
--============================================================

local function updatePitchBias(pitchCG)
    local p = M.params

    M.state.targetFrontBias =
        clamp(
            0.50 + pitchCG * p.pitchGain + (M.state.rootFrontAdd or 0.0),
            p.minBias,
            p.maxBias
        )

    M.state.targetRearBias =
        clamp(
            0.50 - pitchCG * p.pitchGain + (M.state.rootRearAdd or 0.0),
            p.minBias,
            p.maxBias
        )

    local total = M.state.targetFrontBias + M.state.targetRearBias
    if total > 0.0001 then
        M.state.targetFrontBias = M.state.targetFrontBias / total
        M.state.targetRearBias = M.state.targetRearBias / total
    else
        M.state.targetFrontBias = 0.5
        M.state.targetRearBias = 0.5
    end
end

local function updateRollBias(rollCG)
    local p = M.params

    M.state.targetLeftBias =
        clamp(
            0.50 + rollCG * p.rollGain + (M.state.rootLeftAdd or 0.0),
            p.minBias,
            p.maxBias
        )

    M.state.targetRightBias =
        clamp(
            0.50 - rollCG * p.rollGain + (M.state.rootRightAdd or 0.0),
            p.minBias,
            p.maxBias
        )

    local total = M.state.targetLeftBias + M.state.targetRightBias
    if total > 0.0001 then
        M.state.targetLeftBias = M.state.targetLeftBias / total
        M.state.targetRightBias = M.state.targetRightBias / total
    else
        M.state.targetLeftBias = 0.5
        M.state.targetRightBias = 0.5
    end
end

local function smoothBias(dt)
    local p = M.params

    M.state.frontBias = lowPass(M.state.frontBias, M.state.targetFrontBias, p.biasTau, dt)
    M.state.rearBias = lowPass(M.state.rearBias, M.state.targetRearBias, p.biasTau, dt)
    M.state.leftBias = lowPass(M.state.leftBias, M.state.targetLeftBias, p.biasTau, dt)
    M.state.rightBias = lowPass(M.state.rightBias, M.state.targetRightBias, p.biasTau, dt)

    local fb = clamp(M.state.frontBias, p.minBias, p.maxBias)
    M.state.frontBias = fb
    M.state.rearBias = 1.0 - fb

    local lb = clamp(M.state.leftBias, p.minBias, p.maxBias)
    M.state.leftBias = lb
    M.state.rightBias = 1.0 - lb
end

local function decayStoreOnly(dt)
    local tau = M.params.storeOnlyDecayTau or 0.75
    M.state.rootFrontAdd = lowPass(M.state.rootFrontAdd, 0.0, tau, dt)
    M.state.rootRearAdd = lowPass(M.state.rootRearAdd, 0.0, tau, dt)
    M.state.rootLeftAdd = lowPass(M.state.rootLeftAdd, 0.0, tau, dt)
    M.state.rootRightAdd = lowPass(M.state.rootRightAdd, 0.0, tau, dt)
end

local function updateStateNames(pitchCG, rollCG)
    if abs(pitchCG + (M.state.rootFrontAdd or 0.0)) < M.params.neutralThreshold then
        M.state.stateName = "BALANCED"
    elseif pitchCG + (M.state.rootFrontAdd or 0.0) > 0.0 then
        M.state.stateName = "FRONT LOADED"
    else
        M.state.stateName = "REAR LOADED"
    end

    if abs(rollCG + (M.state.rootLeftAdd or 0.0)) < M.params.neutralThreshold then
        M.state.rollName = "CENTERED"
    elseif rollCG + (M.state.rootLeftAdd or 0.0) > 0.0 then
        M.state.rollName = "LEFT LOADED"
    else
        M.state.rollName = "RIGHT LOADED"
    end
end

--============================================================
-- Export
--============================================================

local function exportState()
    safeStore("ngp_front_bias", M.state.frontBias or 0.5)
    safeStore("ngp_rear_bias", M.state.rearBias or 0.5)
    safeStore("ngp_left_bias", M.state.leftBias or 0.5)
    safeStore("ngp_right_bias", M.state.rightBias or 0.5)

    safeStore("ngp_weight_front_bias", M.state.frontBias or 0.5)
    safeStore("ngp_weight_rear_bias", M.state.rearBias or 0.5)
    safeStore("ngp_weight_left_bias", M.state.leftBias or 0.5)
    safeStore("ngp_weight_right_bias", M.state.rightBias or 0.5)

    safeStore("ngp_weight_target_front_bias", M.state.targetFrontBias or 0.5)
    safeStore("ngp_weight_target_rear_bias", M.state.targetRearBias or 0.5)
    safeStore("ngp_weight_target_left_bias", M.state.targetLeftBias or 0.5)
    safeStore("ngp_weight_target_right_bias", M.state.targetRightBias or 0.5)

    safeStore("ngp_weight_pitch_cg", M.state.pitchCG or 0.0)
    safeStore("ngp_weight_roll_cg", M.state.rollCG or 0.0)

    safeStore("ngp_weight_distribution_status", M.state.status or "UNKNOWN")
    safeStore("ngp_weight_distribution_update_count", M.state.updateCount or 0)
    safeStore("ngp_weight_distribution_wheels_valid", M.state.wheelsValid and 1 or 0)
    safeStore("ngp_weight_distribution_store_only", M.state.storeOnly and 1 or 0)
    safeStore("ngp_weight_distribution_linked_cg", M.state.linkedCG and 1 or 0)
    safeStore("ngp_weight_distribution_state", M.state.stateName or "BALANCED")
    safeStore("ngp_weight_distribution_roll_state", M.state.rollName or "CENTERED")

    safeStore("ngp_weight_root_front_add", M.state.rootFrontAdd or 0.0)
    safeStore("ngp_weight_root_rear_add", M.state.rootRearAdd or 0.0)
    safeStore("ngp_weight_root_left_add", M.state.rootLeftAdd or 0.0)
    safeStore("ngp_weight_root_right_add", M.state.rootRightAdd or 0.0)

    safeStore("ngp_weight_load_pitch", M.state.loadPitch or 0.0)
    safeStore("ngp_weight_load_roll", M.state.loadRoll or 0.0)
    safeStore("ngp_weight_load_yaw", M.state.loadYaw or 0.0)
    safeStore("ngp_weight_suspension_stress", M.state.suspensionStress or 0.0)
    safeStore("ngp_weight_suspension_input_avg", M.state.suspensionInputAvg or 0.0)
    safeStore("ngp_weight_contact_loss_avg", M.state.contactLossAvg or 0.0)
    safeStore("ngp_weight_road_input_avg", M.state.roadInputAvg or 0.0)
    safeStore("ngp_weight_load_path_loss_avg", M.state.loadPathLossAvg or 0.0)

    safeStore("ngp_wd_front", M.state.frontBias or 0.5)
    safeStore("ngp_wd_rear", M.state.rearBias or 0.5)
    safeStore("ngp_wd_left", M.state.leftBias or 0.5)
    safeStore("ngp_wd_right", M.state.rightBias or 0.5)
    safeStore("ngp_wd_pitch", M.state.loadPitch or 0.0)
    safeStore("ngp_wd_roll", M.state.loadRoll or 0.0)
    safeStore("ngp_wd_active_inputs", M.state.activeRootInputs or 0)

    safeStore("ngp_weight_max_wheel_load", M.state.maxWheelLoad or 0.0)
    safeStore("ngp_weight_avg_wheel_load", M.state.avgWheelLoad or 0.0)
    safeStore("ngp_weight_speed_kmh", M.state.speedKmh or 0.0)

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_weight_load_linked", M.state.loadLinked and 1 or 0)
    safeStore("ngp_weight_chassis_linked", M.state.chassisLinked and 1 or 0)
    safeStore("ngp_weight_virtual_linked", M.state.virtualLinked and 1 or 0)
    safeStore("ngp_weight_brake_linked", M.state.brakeLinked and 1 or 0)
    safeStore("ngp_weight_drivetrain_linked", M.state.drivetrainLinked and 1 or 0)
    safeStore("ngp_weight_suspension_linked", M.state.suspensionLinked and 1 or 0)
    safeStore("ngp_weight_tire_linked", M.state.tireLinked and 1 or 0)
    safeStore("ngp_weight_body_linked", M.state.bodyLinked and 1 or 0)
    safeStore("ngp_weight_mass_linked", M.state.massLinked and 1 or 0)

    safeStore("ngp_weight_brake_input", M.state.brakeInput or 0.0)
    safeStore("ngp_weight_brake_lock_avg", M.state.brakeLockAvg or 0.0)
    safeStore("ngp_weight_drive_torque_norm", M.state.driveTorqueNorm or 0.0)
    safeStore("ngp_weight_shaft_twist", M.state.shaftTwist or 0.0)
    safeStore("ngp_weight_lsd_lock", M.state.lsdLock or 0.0)
    safeStore("ngp_weight_body_flex_factor", M.state.bodyFlexFactor or 1.0)
    safeStore("ngp_weight_body_runtime_penalty", M.state.bodyRuntimePenalty or 0.0)
    safeStore("ngp_weight_cg_rear_shift", M.state.cgRearShift or 0.0)
end

--============================================================
-- Update
--============================================================

function M.init()
    M.state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    M.state.updateCount = (M.state.updateCount or 0) + 1

    dt = safeNumber(dt, 0.0)
    if dt <= 0.0 then
        M.state.status = "BAD DT"
        exportState()
        return
    end

    dt = clamp(dt, M.params.minDt, M.params.maxDt)
    updateDebugGate(dt)

    car = car or safeGetCar()

    M.state.storeOnly = car == nil
    M.state.wheelsValid = car ~= nil and safeField(car, "wheels", nil) ~= nil

    resetLinks()
    readSpeed(car)
    readCG()
    readRootInputs(car)

    if M.state.storeOnly then
        decayStoreOnly(dt)
    end

    updatePitchBias(M.state.pitchCG)
    updateRollBias(M.state.rollCG)
    smoothBias(dt)
    updateStateNames(M.state.pitchCG, M.state.rollCG)

    if M.state.storeOnly then
        if M.state.activeRootInputs > 0 or M.state.linkedCG then
            M.state.status = "STORE ONLY"
        else
            M.state.status = "NO CAR / NEUTRAL"
        end
    elseif M.state.linkedCG then
        M.state.status = "RUNNING"
    elseif M.state.activeRootInputs > 0 then
        M.state.status = "ROOT RUNNING"
    else
        M.state.status = "NO CG / NEUTRAL"
    end

    exportState()
end

--============================================================
-- Public API
--============================================================

function M.getState()
    return M.state
end

function M.getFrontBias()
    return M.state.frontBias or 0.5
end

function M.getRearBias()
    return M.state.rearBias or 0.5
end

function M.getLeftBias()
    return M.state.leftBias or 0.5
end

function M.getRightBias()
    return M.state.rightBias or 0.5
end

function M.getTargetFrontBias()
    return M.state.targetFrontBias or 0.5
end

function M.getTargetRearBias()
    return M.state.targetRearBias or 0.5
end

function M.getRootAdds()
    return M.state.rootFrontAdd or 0.0,
           M.state.rootRearAdd or 0.0,
           M.state.rootLeftAdd or 0.0,
           M.state.rootRightAdd or 0.0
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f / CG:%s / Store:%s\n" ..
        "Front %.1f%% Rear %.1f%% / Left %.1f%% Right %.1f%%\n" ..
        "Target F/R/L/R %.3f %.3f %.3f %.3f\n" ..
        "PitchCG %.3f RollCG %.3f / Load P/R/Y %.3f %.3f %.3f\n" ..
        "RootAdd F/R/L/R %.3f %.3f %.3f %.3f\n" ..
        "Susp %.3f Road %.3f ContactLoss %.3f BodyFlex %.2f\n" ..
        "%s / %s\n" ..
        "Links Load:%s Chassis:%s Virtual:%s Brake:%s Drive:%s Susp:%s Tire:%s Body:%s Mass:%s",

        tostring(M.state.status),
        M.state.updateCount or 0,
        M.state.linkedCG and "OK" or "NIL",
        M.state.storeOnly and "YES" or "NO",

        (M.state.frontBias or 0.5) * 100.0,
        (M.state.rearBias or 0.5) * 100.0,
        (M.state.leftBias or 0.5) * 100.0,
        (M.state.rightBias or 0.5) * 100.0,

        M.state.targetFrontBias or 0.5,
        M.state.targetRearBias or 0.5,
        M.state.targetLeftBias or 0.5,
        M.state.targetRightBias or 0.5,

        M.state.pitchCG or 0.0,
        M.state.rollCG or 0.0,
        M.state.loadPitch or 0.0,
        M.state.loadRoll or 0.0,
        M.state.loadYaw or 0.0,

        M.state.rootFrontAdd or 0.0,
        M.state.rootRearAdd or 0.0,
        M.state.rootLeftAdd or 0.0,
        M.state.rootRightAdd or 0.0,

        M.state.suspensionStress or 0.0,
        M.state.roadInputAvg or 0.0,
        M.state.contactLossAvg or 0.0,
        M.state.bodyFlexFactor or 1.0,

        tostring(M.state.stateName),
        tostring(M.state.rollName),

        M.state.loadLinked and "OK" or "NIL",
        M.state.chassisLinked and "OK" or "NIL",
        M.state.virtualLinked and "OK" or "NIL",
        M.state.brakeLinked and "OK" or "NIL",
        M.state.drivetrainLinked and "OK" or "NIL",
        M.state.suspensionLinked and "OK" or "NIL",
        M.state.tireLinked and "OK" or "NIL",
        M.state.bodyLinked and "OK" or "NIL",
        M.state.massLinked and "OK" or "NIL"
    )
end

function M.drawDebug()
    if not ui then
        return
    end

    ui.text("=== WEIGHT DISTRIBUTION ===")
    ui.text(M.debugStr())
end

return M
