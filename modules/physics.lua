---@diagnostic disable: undefined-global

--============================================================
-- physics.lua
-- ACNextGen V1.1.5 Stable
-- Universal Physics Hub
--
-- App-side only. This module does not require vehicle-side script.lua
-- and does not directly rewrite AC physics. It collects root/trunk
-- signals into stable hub keys for observers and downstream modules.
--============================================================

local M = {}

M.params = {
    wheelLoadRef = 3500.0,
    loadRatioMin = 0.30,
    loadRatioMax = 2.70,

    noCarDecayTau = 0.180,
    moduleReadInterval = 0.0,

    minDt = 0.0001,
    maxDt = 0.050,

    debugStoreInterval = 0.250,
}

local WHEEL_NAMES = { [0] = "FL", [1] = "FR", [2] = "RL", [3] = "RR" }

local state = {
    alive = 0,
    updateCount = 0,
    status = "INIT",
    wheelsValid = false,

    speedKmh = 0.0,
    yawRate = 0.0,
    steer = 0.0,
    gas = 0.0,
    brake = 0.0,
    clutch = 1.0,
    handbrake = 0.0,
    gear = 0,
    rpm = 0.0,

    load = { [0]=0,[1]=0,[2]=0,[3]=0 },
    integratedLoad = { [0]=3500,[1]=3500,[2]=3500,[3]=3500 },
    dltLoad = { [0]=0,[1]=0,[2]=0,[3]=0 },
    loadRatio = { [0]=1,[1]=1,[2]=1,[3]=1 },

    slipRatio = { [0]=0,[1]=0,[2]=0,[3]=0 },
    slipAngle = { [0]=0,[1]=0,[2]=0,[3]=0 },
    omega = { [0]=0,[1]=0,[2]=0,[3]=0 },

    forceLat = { [0]=0,[1]=0,[2]=0,[3]=0 },
    forceLong = { [0]=0,[1]=0,[2]=0,[3]=0 },
    forceRoad = { [0]=0,[1]=0,[2]=0,[3]=0 },

    contactQuality = { [0]=1,[1]=1,[2]=1,[3]=1 },
    contactTrust = { [0]=1,[1]=1,[2]=1,[3]=1 },
    contactLoss = { [0]=0,[1]=0,[2]=0,[3]=0 },

    suspForce = { [0]=0,[1]=0,[2]=0,[3]=0 },
    suspScale = { [0]=1,[1]=1,[2]=1,[3]=1 },
    damperForce = { [0]=0,[1]=0,[2]=0,[3]=0 },
    damperVelocity = { [0]=0,[1]=0,[2]=0,[3]=0 },

    brakeLock = { [0]=0,[1]=0,[2]=0,[3]=0 },
    brakeTemp = { [0]=25,[1]=25,[2]=25,[3]=25 },

    loadPathWork = { [0]=0,[1]=0,[2]=0,[3]=0 },
    loadPathEfficiency = { [0]=1,[1]=1,[2]=1,[3]=1 },
    loadPathLoss = { [0]=0,[1]=0,[2]=0,[3]=0 },

    diffLock = 0.0,
    diffDiff = 0.0,
    diffHeat = 0.0,
    diffMode = "NONE",

    lsdLock = 0.0,
    lsdDiff = 0.0,
    lsdHeat = 0.0,
    lsdTarget = 0.0,
    lsdInputTorqueNm = 0.0,
    lsdLockTorqueNm = 0.0,
    diffPower = 0.0,
    diffCoast = 0.0,
    diffPreload = 0.0,
    diffForceL = 0.0,
    diffForceR = 0.0,

    driveTorque = 0.0,
    driveTorqueNm = 0.0,
    driveTransmittedTorque = 0.0,
    diffInputTorqueNm = 0.0,
    shaftTwist = 0.0,
    shaftVelocity = 0.0,
    driveLash = 0.0,
    shiftShock = 0.0,

    windup = 0.0,
    windupEnergy = 0.0,
    windupRelease = 0.0,
    softTorque = 0.0,
    rearPush = 0.0,
    driveYawHint = 0.0,

    bodyRigidity = 1.0,
    chassisEnergy = 0.0,
    chassisFlex = 0.0,
    damageTotal = 0.0,
    conditionTotal = 0.0,

    virtualYaw = 0.0,
    virtualPitch = 0.0,
    virtualRoll = 0.0,

    weightFront = 0.5,
    weightRear = 0.5,
    weightLeft = 0.5,
    weightRight = 0.5,

    loadFront = 0.5,
    loadRear = 0.5,
    loadLeft = 0.5,
    loadRight = 0.5,

    frontAxisAnchor = 0.0,
    frontAxisAuthority = 0.0,
    frontAxisYawResist = 0.0,
    frontAxisSteerWeight = 0.0,
    frontAxisSlipDamp = 0.0,

    loadPathAvgWork = 0.0,
    loadPathAvgEfficiency = 1.0,
    loadPathAvgIntegrity = 1.0,
    loadPathAvgLoss = 0.0,
    loadPathDominant = "UNKNOWN",

    preserveAvg = 0.0,

    links = {
        car = false,
        wheels = false,
        loadTransfer = false,
        tireForce = false,
        contact = false,
        suspension = false,
        damper = false,
        brake = false,
        body = false,
        chassis = false,
        damage = false,
        diff = false,
        drivetrain = false,
        windup = false,
        massBalance = false,
        loadPath = false,
    },

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

--============================================================
-- Utility
--============================================================

local function num(v, defaultValue)
    local n = tonumber(v)
    if n == nil or n ~= n or n == math.huge or n == -math.huge then
        return defaultValue or 0.0
    end
    return n
end

local function clamp(v, mn, mx)
    v = num(v, mn)
    if v < mn then return mn end
    if v > mx then return mx end
    return v
end

local function lowPass(current, target, tau, dt)
    current = num(current, 0.0)
    target = num(target, 0.0)
    tau = num(tau, 0.0)
    dt = num(dt, 0.0)

    if tau <= 0.00001 then
        return target
    end

    local a = clamp(dt / (tau + dt), 0.0, 1.0)
    return current + (target - current) * a
end

local function safeField(obj, field, defaultValue)
    if not obj then return defaultValue end
    local ok, v = pcall(function() return obj[field] end)
    if not ok or v == nil then return defaultValue end
    return v
end

local function safeLoadRaw(key)
    if not ac or not ac.load then return nil end
    local ok, v = pcall(function() return ac.load(key) end)
    if not ok then return nil end
    return v
end

local function loadNumber(key, defaultValue)
    local v = safeLoadRaw(key)
    if v == nil then return defaultValue or 0.0 end
    return num(v, defaultValue or 0.0)
end

local function loadString(key, defaultValue)
    local v = safeLoadRaw(key)
    if v == nil then return defaultValue or "" end
    return tostring(v)
end

local function loadFirst(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local v = safeLoadRaw(keys[i])
        if v ~= nil then
            return num(v, defaultValue or 0.0), keys[i]
        end
    end
    return defaultValue or 0.0, nil
end

local function loadFirstString(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local v = safeLoadRaw(keys[i])
        if v ~= nil then
            return tostring(v), keys[i]
        end
    end
    return tostring(defaultValue or ""), nil
end

local function storeValue(key, value)
    if not ac or not ac.store then return end
    pcall(function() ac.store(key, value) end)
end

local function safeGetCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function() return ac.getCar(0) end)
    if not ok then return nil end
    return car
end

local function getWheel(car, index)
    local wheels = safeField(car, "wheels", nil)
    if not wheels then return nil end

    local ok, wheel = pcall(function()
        return wheels[index] or wheels[index + 1]
    end)

    if ok then return wheel end
    return nil
end

local function vectorLength(v)
    if not v then return 0.0 end
    local ok, result = pcall(function()
        if type(v.length) == "function" then
            return v:length()
        end
        if type(v.length) == "number" then
            return v.length
        end
        local x = num(v.x, 0.0)
        local y = num(v.y, 0.0)
        local z = num(v.z, 0.0)
        return math.sqrt(x * x + y * y + z * z)
    end)
    if ok then return num(result, 0.0) end
    return 0.0
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

local function resetLinks()
    for k, _ in pairs(state.links) do
        state.links[k] = false
    end
end

local function markLink(name, linked)
    if linked then
        state.links[name] = true
    end
end

--============================================================
-- Car data
--============================================================

local function getSpeedKmh(car)
    local speedKmh = safeField(car, "speedKmh", nil)
    if speedKmh ~= nil then return num(speedKmh, 0.0) end

    local speed = nil
    if speed ~= nil then return num(speed, 0.0) end

    local velocity = safeField(car, "velocity", nil) or safeField(car, "localVelocity", nil)
    return vectorLength(velocity) * 3.6
end

local function readYawRate(car)
    local av = safeField(car, "localAngularVelocity", nil)
    if av then
        if safeField(av, "y", nil) ~= nil then
            return num(safeField(av, "y", 0.0), 0.0)
        end
        if safeField(av, "z", nil) ~= nil then
            return num(safeField(av, "z", 0.0), 0.0)
        end
    end

    return loadNumber("ngp_yaw_rate", 0.0)
end

local function readWheelLoadFromAPI(wheel)
    local v =
        safeField(wheel, "load", nil)
        or safeField(wheel, "loadK", nil)
        or nil
        or nil
        or nil

    return num(v, nil)
end

local function decayCarValues(dt)
    local tau = M.params.noCarDecayTau

    state.speedKmh = lowPass(state.speedKmh, 0.0, tau, dt)
    state.yawRate = lowPass(state.yawRate, 0.0, tau, dt)
    state.steer = lowPass(state.steer, 0.0, tau, dt)
    state.gas = lowPass(state.gas, 0.0, tau, dt)
    state.brake = lowPass(state.brake, 0.0, tau, dt)
    state.handbrake = lowPass(state.handbrake, 0.0, tau, dt)
    state.clutch = lowPass(state.clutch, 1.0, tau, dt)
    state.rpm = lowPass(state.rpm, 0.0, tau, dt)
    state.gear = 0

    for i = 0, 3 do
        state.load[i] = lowPass(state.load[i], 0.0, tau, dt)
        state.slipRatio[i] = lowPass(state.slipRatio[i], 0.0, tau, dt)
        state.slipAngle[i] = lowPass(state.slipAngle[i], 0.0, tau, dt)
        state.omega[i] = lowPass(state.omega[i], 0.0, tau, dt)
    end
end

local function readCar(car, dt)
    if not car then
        state.status = "NO CAR"
        state.wheelsValid = false
        state.links.car = false
        state.links.wheels = false
        decayCarValues(dt)
        return false
    end

    state.links.car = true

    state.speedKmh = getSpeedKmh(car)
    state.yawRate = readYawRate(car)
    state.steer = clamp(num(safeField(car, "steer", 0.0), 0.0), -1.0, 1.0)
    state.gas = clamp(num(safeField(car, "gas", 0.0), 0.0), 0.0, 1.0)
    state.brake = clamp(num(safeField(car, "brake", 0.0), 0.0), 0.0, 1.0)
    state.clutch = clamp(num(safeField(car, "clutch", 1.0), 1.0), 0.0, 1.0)
    state.handbrake = clamp(num(safeField(car, "handbrake", 0.0), 0.0), 0.0, 1.0)
    state.gear = math.floor(num(safeField(car, "gear", 0), 0))
    state.rpm = num(safeField(car, "rpm", 0.0), 0.0)

    local hasAnyWheel = false

    for i = 0, 3 do
        local wheel = getWheel(car, i)
        if wheel then
            hasAnyWheel = true

            local load = readWheelLoadFromAPI(wheel)
            if load ~= nil then
                state.load[i] = clamp(load, 0.0, 12000.0)
            end

            state.slipRatio[i] =
                num(
                    safeField(wheel, "slipRatio", safeField(wheel, "slip", state.slipRatio[i])),
                    state.slipRatio[i]
                )

            state.slipAngle[i] =
                num(
                    safeField(wheel, "slipAngle", state.slipAngle[i]),
                    state.slipAngle[i]
                )

            state.omega[i] =
                num(
                    safeField(wheel, "angularSpeed", state.omega[i]),
                    state.omega[i]
                )
        else
            state.load[i] = lowPass(state.load[i], 0.0, M.params.noCarDecayTau, dt)
            state.slipRatio[i] = lowPass(state.slipRatio[i], 0.0, M.params.noCarDecayTau, dt)
            state.slipAngle[i] = lowPass(state.slipAngle[i], 0.0, M.params.noCarDecayTau, dt)
            state.omega[i] = lowPass(state.omega[i], 0.0, M.params.noCarDecayTau, dt)
        end
    end

    state.wheelsValid = hasAnyWheel
    state.links.wheels = hasAnyWheel

    if not hasAnyWheel then
        state.status = "NO WHEELS"
        return false
    end

    state.status = "RUNNING"
    return true
end

--============================================================
-- Module readers
--============================================================

local function readWheelModules(i)
    local dlt, dltKey = loadFirst(0.0,
        "ngp_dlt_load_" .. i,
        "ngp_delta_load_" .. i
    )

    state.dltLoad[i] = dlt
    markLink("loadTransfer", dltKey ~= nil)

    local integrated, integratedKey = loadFirst(state.load[i],
        "ngp_wheel_load_" .. i,
        "ngp_load_wheel_" .. i,
        "ngp_tire_load_input_" .. i,
        "ngp_susp_load_input_" .. i,
        "ngp_load_path_load_" .. i
    )

    if integratedKey ~= nil then
        state.integratedLoad[i] = clamp(integrated, 0.0, 12000.0)
        markLink("loadTransfer", true)
    else
        local base = state.load[i]
        if base > 1.0 then
            state.integratedLoad[i] = clamp(base + dlt, 0.0, 12000.0)
        else
            state.integratedLoad[i] = lowPass(state.integratedLoad[i], M.params.wheelLoadRef, M.params.noCarDecayTau, M.params.maxDt)
        end
    end

    local baseLoad = math.max(state.load[i], 1.0)
    if baseLoad > 50.0 then
        state.loadRatio[i] = clamp(state.integratedLoad[i] / baseLoad, M.params.loadRatioMin, M.params.loadRatioMax)
    else
        state.loadRatio[i] = clamp(state.integratedLoad[i] / math.max(M.params.wheelLoadRef, 1.0), M.params.loadRatioMin, M.params.loadRatioMax)
    end

    local lat, latKey = loadFirst(0.0,
        "ngp_tire_force_lat_" .. i,
        "ngp_tf_lat_" .. i,
        "ngp_lat_" .. i,
        "ngp_tire_lat_" .. i
    )

    local long, longKey = loadFirst(0.0,
        "ngp_tire_force_long_" .. i,
        "ngp_tf_long_" .. i,
        "ngp_long_" .. i,
        "ngp_tire_long_" .. i
    )

    local cpLat, cpLatKey = loadFirst(0.0, "ngp_cp_lat_" .. i)
    local cpLong, cpLongKey = loadFirst(0.0, "ngp_cp_long_" .. i)
    local road, roadKey = loadFirst(0.0,
        "ngp_tire_force_road_input_" .. i,
        "ngp_tf_road_input_" .. i,
        "ngp_damper_tire_road_input_" .. i
    )

    state.forceLat[i] = lat + cpLat
    state.forceLong[i] = long + cpLong
    state.forceRoad[i] = road

    markLink("tireForce", latKey ~= nil or longKey ~= nil or cpLatKey ~= nil or cpLongKey ~= nil or roadKey ~= nil)

    local cq, cqKey = loadFirst(1.0,
        "ngp_contact_quality_" .. i,
        "ngp_tire_contact_quality_" .. i,
        "ngp_tcr_quality_" .. i,
        "ngp_tc_contact_" .. i
    )

    local trust, trustKey = loadFirst(cq,
        "ngp_contact_trust_" .. i,
        "ngp_tire_contact_trust_" .. i
    )

    local loss, lossKey = loadFirst(1.0 - clamp(cq, 0.0, 1.0),
        "ngp_contact_loss_" .. i,
        "ngp_tire_contact_loss_" .. i,
        "ngp_tcr_contact_loss_" .. i
    )

    state.contactQuality[i] = clamp(cq, 0.0, 1.2)
    state.contactTrust[i] = clamp(trust, 0.0, 1.2)
    state.contactLoss[i] = clamp(loss, 0.0, 1.0)

    markLink("contact", cqKey ~= nil or trustKey ~= nil or lossKey ~= nil)

    local susp, suspKey = loadFirst(0.0,
        "ngp_susp_integrated_force_" .. i,
        "ngp_susp_int_force_" .. i,
        "ngp_susp_" .. i
    )

    local suspScale, suspScaleKey = loadFirst(1.0,
        "ngp_susp_integrated_scale_" .. i,
        "ngp_susp_int_scale_" .. i
    )

    state.suspForce[i] = susp
    state.suspScale[i] = clamp(suspScale, 0.0, 2.0)
    markLink("suspension", suspKey ~= nil or suspScaleKey ~= nil)

    local damper, damperKey = loadFirst(0.0,
        "ngp_damper_force_" .. i,
        "ngp_damper_" .. i
    )

    local damperVel, damperVelKey = loadFirst(0.0,
        "ngp_damper_velocity_" .. i,
        "ngp_damper_hyst_velocity_" .. i
    )

    state.damperForce[i] = damper
    state.damperVelocity[i] = damperVel
    markLink("damper", damperKey ~= nil or damperVelKey ~= nil)

    local lock, lockKey = loadFirst(0.0, "ngp_brake_lock_" .. i)
    local temp, tempKey = loadFirst(25.0,
        "ngp_brake_temp_" .. i,
        "ngp_brake_disc_temp_" .. i
    )

    state.brakeLock[i] = clamp(lock, 0.0, 1.0)
    state.brakeTemp[i] = temp
    markLink("brake", lockKey ~= nil or tempKey ~= nil)

    local lpWork, lpWorkKey = loadFirst(0.0,
        "ngp_load_path_work_" .. i,
        "ngp_lp_work_" .. i
    )

    local lpEff, lpEffKey = loadFirst(1.0,
        "ngp_load_path_efficiency_" .. i,
        "ngp_lp_eff_" .. i
    )

    local lpLoss, lpLossKey = loadFirst(0.0,
        "ngp_load_path_loss_" .. i,
        "ngp_lp_loss_" .. i
    )

    state.loadPathWork[i] = clamp(lpWork, 0.0, 1.5)
    state.loadPathEfficiency[i] = clamp(lpEff, 0.0, 1.2)
    state.loadPathLoss[i] = clamp(lpLoss, 0.0, 1.0)
    markLink("loadPath", lpWorkKey ~= nil or lpEffKey ~= nil or lpLossKey ~= nil)
end

local function readGlobalModules()
    local v, key

    state.bodyRigidity, key = loadFirst(1.0,
        "ngp_body_rigidity",
        "ngp_hub_body_rigidity"
    )
    state.bodyRigidity = clamp(state.bodyRigidity, 0.0, 2.0)
    markLink("body", key ~= nil)

    state.chassisEnergy, key = loadFirst(0.0,
        "ngp_chassis_energy",
        "ngp_chassis_body_energy"
    )
    markLink("chassis", key ~= nil)

    state.chassisFlex, key = loadFirst(0.0,
        "ngp_chassis_flex_energy",
        "ngp_flex_energy"
    )
    markLink("chassis", key ~= nil)

    state.damageTotal, key = loadFirst(0.0,
        "ngp_damage_total",
        "ngp_damage_chassis",
        "ngp_damage_state_total",
        "ngp_vehicle_condition_total"
    )
    markLink("damage", key ~= nil)

    state.conditionTotal, key = loadFirst(0.0,
        "ngp_condition_total",
        "ngp_vehicle_condition_total"
    )
    markLink("damage", key ~= nil)

    state.virtualYaw, key = loadFirst(0.0,
        "ngp_virtual_yaw",
        "ngp_virtual_inertia_yaw"
    )

    state.virtualPitch = loadNumber("ngp_virtual_pitch", 0.0)
    state.virtualRoll = loadNumber("ngp_virtual_roll", 0.0)

    state.weightFront = loadNumber("ngp_weight_front_bias", loadNumber("ngp_front_bias", loadNumber("ngp_load_front", 0.5)))
    state.weightRear = loadNumber("ngp_weight_rear_bias", loadNumber("ngp_rear_bias", loadNumber("ngp_load_rear", 0.5)))
    state.weightLeft = loadNumber("ngp_weight_left_bias", loadNumber("ngp_left_bias", loadNumber("ngp_load_left", 0.5)))
    state.weightRight = loadNumber("ngp_weight_right_bias", loadNumber("ngp_right_bias", loadNumber("ngp_load_right", 0.5)))

    state.loadFront = loadNumber("ngp_load_abs_front", loadNumber("ngp_load_front", 0.5))
    state.loadRear = loadNumber("ngp_load_abs_rear", loadNumber("ngp_load_rear", 0.5))
    state.loadLeft = loadNumber("ngp_load_abs_left", loadNumber("ngp_load_left", 0.5))
    state.loadRight = loadNumber("ngp_load_abs_right", loadNumber("ngp_load_right", 0.5))

    state.diffLock, key = loadFirst(0.0,
        "ngp_lsd_lock",
        "ngp_diff_lock"
    )
    markLink("diff", key ~= nil)

    state.diffDiff = loadNumber("ngp_lsd_diff", loadNumber("ngp_diff_diff", 0.0))
    state.diffHeat = loadNumber("ngp_lsd_heat", loadNumber("ngp_diff_heat", 0.0))
    state.diffMode = loadString("ngp_lsd_mode", loadString("ngp_diff_mode", "NONE"))

    state.lsdLock = state.diffLock
    state.lsdDiff = state.diffDiff
    state.lsdHeat = state.diffHeat
    state.lsdTarget = loadNumber("ngp_lsd_target", state.lsdLock)
    state.lsdInputTorqueNm = loadNumber("ngp_lsd_input_torque_nm", loadNumber("ngp_diff_input_torque_nm", 0.0))
    state.lsdLockTorqueNm = loadNumber("ngp_lsd_lock_torque_nm", loadNumber("ngp_lsd_lock_torque", 0.0))

    state.diffPower = loadNumber("ngp_diff_power", loadNumber("ngp_lsd_power_ratio", 0.0))
    state.diffCoast = loadNumber("ngp_diff_coast", loadNumber("ngp_lsd_coast_ratio", 0.0))
    state.diffPreload = loadNumber("ngp_diff_preload", loadNumber("ngp_lsd_preload_torque_nm", 0.0))
    state.diffForceL = loadNumber("ngp_diff_forceL", 0.0)
    state.diffForceR = loadNumber("ngp_diff_forceR", 0.0)

    state.driveTorque, key = loadFirst(0.0,
        "ngp_drivetrain_torque",
        "ngp_dt_torque",
        "ngp_drive_torque"
    )
    markLink("drivetrain", key ~= nil)

    state.driveTorqueNm = loadNumber("ngp_drive_torque_nm", 0.0)
    state.driveTransmittedTorque = loadNumber("ngp_drive_transmitted_torque", 0.0)
    state.diffInputTorqueNm = loadNumber("ngp_diff_input_torque_nm", state.lsdInputTorqueNm)
    state.shaftTwist = loadNumber("ngp_shaft_twist", loadNumber("ngp_windup_shaft_twist", 0.0))
    state.shaftVelocity = loadNumber("ngp_shaft_velocity", 0.0)
    state.driveLash = loadNumber("ngp_drive_lash", 0.0)
    state.shiftShock = loadNumber("ngp_shift_shock", loadNumber("ngp_windup_shift_shock", 0.0))

    state.windup, key = loadFirst(0.0,
        "ngp_driveline_windup",
        "ngp_windup_value"
    )
    markLink("windup", key ~= nil)

    state.windupEnergy = loadNumber("ngp_driveline_energy", loadNumber("ngp_windup_energy", 0.0))
    state.windupRelease = loadNumber("ngp_driveline_release", loadNumber("ngp_windup_release", 0.0))
    state.softTorque = loadNumber("ngp_drive_soft_torque", 0.0)
    state.rearPush = loadNumber("ngp_drive_soft_rear_push", loadNumber("ngp_driveline_rear_push", 0.0))
    state.driveYawHint = loadNumber("ngp_drive_soft_yaw", loadNumber("ngp_driveline_yaw_hint", 0.0))

    state.frontAxisAnchor, key = loadFirst(0.0,
        "ngp_front_axis_anchor",
        "ngp_mass_front_axis_anchor"
    )
    markLink("massBalance", key ~= nil)

    state.frontAxisAuthority = loadNumber("ngp_front_axis_authority", 0.0)
    state.frontAxisYawResist = loadNumber("ngp_front_axis_yaw_resist", 0.0)
    state.frontAxisSteerWeight = loadNumber("ngp_front_axis_steer_weight", 0.0)
    state.frontAxisSlipDamp = loadNumber("ngp_front_axis_slip_damp", 0.0)

    state.loadPathAvgWork, key = loadFirst(0.0,
        "ngp_load_path_avg_work",
        "ngp_lp_avg_work"
    )
    markLink("loadPath", key ~= nil)

    state.loadPathAvgEfficiency = loadNumber("ngp_load_path_avg_efficiency", loadNumber("ngp_lp_avg_eff", 1.0))
    state.loadPathAvgIntegrity = loadNumber("ngp_load_path_avg_integrity", loadNumber("ngp_lp_avg_integrity", 1.0))
    state.loadPathAvgLoss = loadNumber("ngp_load_path_avg_loss", loadNumber("ngp_lp_avg_loss", 0.0))
    state.loadPathDominant = loadString("ngp_load_path_dominant", "UNKNOWN")

    local preserveSum = 0.0
    for i = 0, 3 do
        preserveSum = preserveSum + loadNumber("ngp_susp_preserve_force_add_" .. i, 0.0)
    end
    state.preserveAvg = preserveSum * 0.25
end

local function readModules()
    for i = 0, 3 do
        readWheelModules(i)
    end
    readGlobalModules()
end

--============================================================
-- Publish
--============================================================

local function publishCore()
    storeValue("ngp_hub_alive", state.alive)
    storeValue("ngp_hub_update_count", state.updateCount)
    storeValue("ngp_hub_status", state.status)
    storeValue("ngp_hub_wheels_valid", state.wheelsValid and 1 or 0)

    storeValue("ngp_hub_speed", state.speedKmh)
    storeValue("ngp_hub_yaw", state.yawRate)
    storeValue("ngp_hub_steer", state.steer)
    storeValue("ngp_hub_gas", state.gas)
    storeValue("ngp_hub_brake", state.brake)
    storeValue("ngp_hub_clutch", state.clutch)
    storeValue("ngp_hub_handbrake", state.handbrake)
    storeValue("ngp_hub_gear", state.gear)
    storeValue("ngp_hub_rpm", state.rpm)

    storeValue("ngp_hub_diff_lock", state.diffLock)
    storeValue("ngp_hub_diff_diff", state.diffDiff)
    storeValue("ngp_hub_diff_heat", state.diffHeat)
    storeValue("ngp_hub_diff_mode", state.diffMode)

    storeValue("ngp_hub_lsd_lock", state.lsdLock)
    storeValue("ngp_hub_lsd_diff", state.lsdDiff)
    storeValue("ngp_hub_lsd_heat", state.lsdHeat)
    storeValue("ngp_hub_lsd_target", state.lsdTarget)
    storeValue("ngp_hub_lsd_input_torque_nm", state.lsdInputTorqueNm)
    storeValue("ngp_hub_lsd_lock_torque_nm", state.lsdLockTorqueNm)

    storeValue("ngp_hub_diff_power", state.diffPower)
    storeValue("ngp_hub_diff_coast", state.diffCoast)
    storeValue("ngp_hub_diff_preload", state.diffPreload)
    storeValue("ngp_hub_diff_forceL", state.diffForceL)
    storeValue("ngp_hub_diff_forceR", state.diffForceR)

    storeValue("ngp_hub_drive_torque", state.driveTorque)
    storeValue("ngp_hub_drive_torque_nm", state.driveTorqueNm)
    storeValue("ngp_hub_drive_transmitted_torque", state.driveTransmittedTorque)
    storeValue("ngp_hub_diff_input_torque_nm", state.diffInputTorqueNm)
    storeValue("ngp_hub_shaft_twist", state.shaftTwist)
    storeValue("ngp_hub_shaft_velocity", state.shaftVelocity)
    storeValue("ngp_hub_drive_lash", state.driveLash)
    storeValue("ngp_hub_shift_shock", state.shiftShock)

    storeValue("ngp_hub_windup", state.windup)
    storeValue("ngp_hub_windup_energy", state.windupEnergy)
    storeValue("ngp_hub_windup_release", state.windupRelease)
    storeValue("ngp_hub_soft_torque", state.softTorque)
    storeValue("ngp_hub_rear_push", state.rearPush)
    storeValue("ngp_hub_drive_yaw_hint", state.driveYawHint)

    storeValue("ngp_hub_body_rigidity", state.bodyRigidity)
    storeValue("ngp_hub_chassis_energy", state.chassisEnergy)
    storeValue("ngp_hub_chassis_flex", state.chassisFlex)
    storeValue("ngp_hub_damage_total", state.damageTotal)
    storeValue("ngp_hub_condition_total", state.conditionTotal)

    storeValue("ngp_hub_virtual_yaw", state.virtualYaw)
    storeValue("ngp_hub_virtual_pitch", state.virtualPitch)
    storeValue("ngp_hub_virtual_roll", state.virtualRoll)

    storeValue("ngp_hub_weight_front", state.weightFront)
    storeValue("ngp_hub_weight_rear", state.weightRear)
    storeValue("ngp_hub_weight_left", state.weightLeft)
    storeValue("ngp_hub_weight_right", state.weightRight)

    storeValue("ngp_hub_load_front", state.loadFront)
    storeValue("ngp_hub_load_rear", state.loadRear)
    storeValue("ngp_hub_load_left", state.loadLeft)
    storeValue("ngp_hub_load_right", state.loadRight)

    storeValue("ngp_hub_front_axis_anchor", state.frontAxisAnchor)
    storeValue("ngp_hub_front_axis_authority", state.frontAxisAuthority)
    storeValue("ngp_hub_front_axis_yaw_resist", state.frontAxisYawResist)
    storeValue("ngp_hub_front_axis_steer_weight", state.frontAxisSteerWeight)
    storeValue("ngp_hub_front_axis_slip_damp", state.frontAxisSlipDamp)

    storeValue("ngp_hub_load_path_avg_work", state.loadPathAvgWork)
    storeValue("ngp_hub_load_path_avg_efficiency", state.loadPathAvgEfficiency)
    storeValue("ngp_hub_load_path_avg_integrity", state.loadPathAvgIntegrity)
    storeValue("ngp_hub_load_path_avg_loss", state.loadPathAvgLoss)
    storeValue("ngp_hub_load_path_dominant", state.loadPathDominant)

    storeValue("ngp_hub_preserve_avg", state.preserveAvg)
end

local function publishWheel(i)
    storeValue("ngp_hub_load_" .. i, state.load[i])
    storeValue("ngp_hub_wheel_load_" .. i, state.integratedLoad[i])
    storeValue("ngp_hub_dlt_" .. i, state.dltLoad[i])
    storeValue("ngp_hub_ratio_" .. i, state.loadRatio[i])

    storeValue("ngp_hub_slipR_" .. i, state.slipRatio[i])
    storeValue("ngp_hub_slipA_" .. i, state.slipAngle[i])
    storeValue("ngp_hub_omega_" .. i, state.omega[i])

    storeValue("ngp_hub_force_lat_" .. i, state.forceLat[i])
    storeValue("ngp_hub_force_long_" .. i, state.forceLong[i])
    storeValue("ngp_hub_force_road_" .. i, state.forceRoad[i])

    storeValue("ngp_hub_contact_quality_" .. i, state.contactQuality[i])
    storeValue("ngp_hub_contact_trust_" .. i, state.contactTrust[i])
    storeValue("ngp_hub_contact_loss_" .. i, state.contactLoss[i])

    storeValue("ngp_hub_susp_force_" .. i, state.suspForce[i])
    storeValue("ngp_hub_susp_scale_" .. i, state.suspScale[i])
    storeValue("ngp_hub_damper_force_" .. i, state.damperForce[i])
    storeValue("ngp_hub_damper_velocity_" .. i, state.damperVelocity[i])

    storeValue("ngp_hub_brake_lock_" .. i, state.brakeLock[i])
    storeValue("ngp_hub_brake_temp_" .. i, state.brakeTemp[i])

    storeValue("ngp_hub_load_path_work_" .. i, state.loadPathWork[i])
    storeValue("ngp_hub_load_path_efficiency_" .. i, state.loadPathEfficiency[i])
    storeValue("ngp_hub_load_path_loss_" .. i, state.loadPathLoss[i])
end

local function publishLinks()
    if not state.debugStoreNow then return end

    storeValue("ngp_hub_link_car", state.links.car and 1 or 0)
    storeValue("ngp_hub_link_wheels", state.links.wheels and 1 or 0)
    storeValue("ngp_hub_link_load_transfer", state.links.loadTransfer and 1 or 0)
    storeValue("ngp_hub_link_tire_force", state.links.tireForce and 1 or 0)
    storeValue("ngp_hub_link_contact", state.links.contact and 1 or 0)
    storeValue("ngp_hub_link_suspension", state.links.suspension and 1 or 0)
    storeValue("ngp_hub_link_damper", state.links.damper and 1 or 0)
    storeValue("ngp_hub_link_brake", state.links.brake and 1 or 0)
    storeValue("ngp_hub_link_body", state.links.body and 1 or 0)
    storeValue("ngp_hub_link_chassis", state.links.chassis and 1 or 0)
    storeValue("ngp_hub_link_damage", state.links.damage and 1 or 0)
    storeValue("ngp_hub_link_diff", state.links.diff and 1 or 0)
    storeValue("ngp_hub_link_drivetrain", state.links.drivetrain and 1 or 0)
    storeValue("ngp_hub_link_windup", state.links.windup and 1 or 0)
    storeValue("ngp_hub_link_mass_balance", state.links.massBalance and 1 or 0)
    storeValue("ngp_hub_link_load_path", state.links.loadPath and 1 or 0)
end

local function publish()
    publishCore()

    for i = 0, 3 do
        publishWheel(i)
    end

    publishLinks()
end

--============================================================
-- Main update
--============================================================

function M.init(car)
    state.status = "INIT"
    state.alive = 0
    publish()
end

function M.update(dt, car, runtime)
    state.alive = (state.alive or 0) + 1
    state.updateCount = (state.updateCount or 0) + 1

    dt = clamp(num(dt, 0.0), M.params.minDt, M.params.maxDt)

    updateDebugGate(dt)
    resetLinks()

    car = car or safeGetCar()

    local carOK = readCar(car, dt)

    readModules()

    if carOK then
        state.status = "RUNNING"
    end

    publish()
end

--============================================================
-- Public API
--============================================================

function M.getState()
    return state
end

function M.getLoadRatio(i)
    return state.loadRatio[tonumber(i) or 0] or 1.0
end

function M.getWheelLoad(i)
    return state.integratedLoad[tonumber(i) or 0] or M.params.wheelLoadRef
end

function M.getRawLoad(i)
    return state.load[tonumber(i) or 0] or 0.0
end

function M.getDeltaLoad(i)
    return state.dltLoad[tonumber(i) or 0] or 0.0
end

function M.getForceLat(i)
    return state.forceLat[tonumber(i) or 0] or 0.0
end

function M.getForceLong(i)
    return state.forceLong[tonumber(i) or 0] or 0.0
end

function M.getDiffLock()
    return state.diffLock or 0.0
end

function M.getDriveTorque()
    return state.driveTorque or 0.0
end

function M.getFrontAxisAnchor()
    return state.frontAxisAnchor or 0.0
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0
        return string.format(
            "%s Hub / Load %.0f -> %.0f / dLT %+.0f / Ratio %.3f\n" ..
            "SlipR %.3f SlipA %.3f Omega %.2f / CQ %.2f Trust %.2f\n" ..
            "Force %.1f %.1f / Susp %.0f Damper %.0f / Brake %.2f",
            tostring(WHEEL_NAMES[i] or i),
            state.load[i] or 0.0,
            state.integratedLoad[i] or 0.0,
            state.dltLoad[i] or 0.0,
            state.loadRatio[i] or 1.0,
            state.slipRatio[i] or 0.0,
            state.slipAngle[i] or 0.0,
            state.omega[i] or 0.0,
            state.contactQuality[i] or 0.0,
            state.contactTrust[i] or 0.0,
            state.forceLat[i] or 0.0,
            state.forceLong[i] or 0.0,
            state.suspForce[i] or 0.0,
            state.damperForce[i] or 0.0,
            state.brakeLock[i] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Alive %.0f\n" ..
        "Speed %.1f km/h / Yaw %.5f / Wheels %s\n" ..
        "Diff Lock %.1f%% / Diff %.3f / Heat %.1f / Mode %s\n" ..
        "Drive %.3f / Tin %.0fNm / Shaft %.3f / Windup %.3f\n" ..
        "Load %.0f %.0f %.0f %.0f\n" ..
        "WheelLoad %.0f %.0f %.0f %.0f\n" ..
        "Ratio %.3f %.3f %.3f %.3f\n" ..
        "FrontAxis %.3f Auth %.3f / LoadPath %.3f %s\n" ..
        "Links Car:%s LT:%s TF:%s CQ:%s Susp:%s Diff:%s DT:%s Wind:%s LP:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.alive or 0,

        state.speedKmh or 0.0,
        state.yawRate or 0.0,
        state.wheelsValid and "OK" or "NIL",

        (state.diffLock or 0.0) * 100.0,
        state.diffDiff or 0.0,
        state.diffHeat or 0.0,
        tostring(state.diffMode),

        state.driveTorque or 0.0,
        state.diffInputTorqueNm or 0.0,
        state.shaftTwist or 0.0,
        state.windup or 0.0,

        state.load[0] or 0.0,
        state.load[1] or 0.0,
        state.load[2] or 0.0,
        state.load[3] or 0.0,

        state.integratedLoad[0] or 0.0,
        state.integratedLoad[1] or 0.0,
        state.integratedLoad[2] or 0.0,
        state.integratedLoad[3] or 0.0,

        state.loadRatio[0] or 1.0,
        state.loadRatio[1] or 1.0,
        state.loadRatio[2] or 1.0,
        state.loadRatio[3] or 1.0,

        state.frontAxisAnchor or 0.0,
        state.frontAxisAuthority or 0.0,
        state.loadPathAvgWork or 0.0,
        tostring(state.loadPathDominant),

        state.links.car and "OK" or "NIL",
        state.links.loadTransfer and "OK" or "NIL",
        state.links.tireForce and "OK" or "NIL",
        state.links.contact and "OK" or "NIL",
        state.links.suspension and "OK" or "NIL",
        state.links.diff and "OK" or "NIL",
        state.links.drivetrain and "OK" or "NIL",
        state.links.windup and "OK" or "NIL",
        state.links.loadPath and "OK" or "NIL"
    )
end

--============================================================
-- Debug draw
--============================================================

function M.drawUI()
    if not ui then return end

    ui.separator()
    ui.text("=== ACNextGen Physics Hub V1.1.5 ===")
    ui.text(string.format("Status : %s / Alive %.0f / Count %.0f", tostring(state.status), state.alive or 0, state.updateCount or 0))
    ui.text(string.format("Speed %.1f km/h | Yaw %.5f | Wheels %s", state.speedKmh or 0.0, state.yawRate or 0.0, state.wheelsValid and "OK" or "NIL"))
    ui.text(string.format("Diff Lock %.1f%% | Diff %.3f | Heat %.1f | Mode %s", (state.diffLock or 0.0) * 100.0, state.diffDiff or 0.0, state.diffHeat or 0.0, tostring(state.diffMode)))
    ui.text(string.format("Drive %.3f | Tin %.0fNm | Shaft %.3f | Windup %.3f", state.driveTorque or 0.0, state.diffInputTorqueNm or 0.0, state.shaftTwist or 0.0, state.windup or 0.0))
    ui.text(string.format("FrontAxis %.3f / Auth %.3f | LoadPath %.3f %s", state.frontAxisAnchor or 0.0, state.frontAxisAuthority or 0.0, state.loadPathAvgWork or 0.0, tostring(state.loadPathDominant)))

    ui.separator()

    for i = 0, 3 do
        ui.text(string.format(
            "%s load %.0f -> %.0f dLT %+.0f ratio %.3f slipR %.3f slipA %.3f",
            tostring(WHEEL_NAMES[i] or i),
            state.load[i] or 0.0,
            state.integratedLoad[i] or 0.0,
            state.dltLoad[i] or 0.0,
            state.loadRatio[i] or 1.0,
            state.slipRatio[i] or 0.0,
            state.slipAngle[i] or 0.0
        ))

        ui.text(string.format(
            "   force %.1f / %.1f omega %.2f CQ %.2f Trust %.2f Susp %.0f",
            state.forceLat[i] or 0.0,
            state.forceLong[i] or 0.0,
            state.omega[i] or 0.0,
            state.contactQuality[i] or 0.0,
            state.contactTrust[i] or 0.0,
            state.suspForce[i] or 0.0
        ))
    end
end

return M
