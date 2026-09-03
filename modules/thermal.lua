---@diagnostic disable: undefined-global

--============================================================
-- ACNextGen
-- thermal.lua
-- V1.1.5 stable
-- Hub / diff / gearbox thermal hub
--============================================================

local M = {}

M.params = {
    brakeToHub = 0.15,
    tyreToHub = 0.05,

    hubCoolBase = 0.020,
    hubCoolSpeed = 0.004,

    diffHeatScale = 8.0,
    diffLSDHeatGain = 0.06,
    diffTorqueHeatGain = 0.000018,
    diffCoolBase = 0.015,
    diffCoolSpeed = 0.003,

    gearboxHeatScale = 0.80,
    drivetrainHeatGain = 1.00,
    driveTorqueHeatGain = 0.000010,
    gearboxCoolBase = 0.010,
    gearboxCoolSpeed = 0.002,

    brakeLockHeatGain = 18.0,
    brakeRootHeatGain = 22.0,
    tireMemoryTempGain = 6.0,
    tireHeatTempGain = 18.0,
    roadShockHubGain = 3.0,
    contactLossHubGain = 2.0,

    ambientTemp = 25.0,
    maxTemp = 1600.0,

    stressRefHub = 180.0,
    stressRefBrake = 850.0,
    stressRefTyre = 140.0,
    stressRefDiff = 180.0,
    stressRefGearbox = 170.0,

    unavailableTau = 0.80,
    debugStoreInterval = 0.25,
}

local state = {
    hubTemp = { [0] = 25.0, [1] = 25.0, [2] = 25.0, [3] = 25.0 },
    brakeTemp = { [0] = 25.0, [1] = 25.0, [2] = 25.0, [3] = 25.0 },
    tyreTemp = { [0] = 25.0, [1] = 25.0, [2] = 25.0, [3] = 25.0 },

    rubberMemory = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    tireMemoryHeat = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    brakeLock = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    roadShock = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    contactLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    diffTemp = 25.0,
    gearboxTemp = 25.0,

    diffHeat = 0.0,
    gearboxHeat = 0.0,

    wheelDiff = 0.0,
    lsdDiff = 0.0,
    lsdHeat = 0.0,
    lsdLock = 0.0,
    diffTorque = 0.0,

    drivetrainHeat = 0.0,
    driveTorque = 0.0,
    shiftShock = 0.0,

    rpm = 0.0,
    gas = 0.0,
    gear = 0.0,
    speed = 0.0,

    avgHubTemp = 25.0,
    maxHubTemp = 25.0,
    avgBrakeTemp = 25.0,
    maxBrakeTemp = 25.0,
    avgTyreTemp = 25.0,
    maxTyreTemp = 25.0,

    hubStress = 0.0,
    brakeStress = 0.0,
    tyreStress = 0.0,
    diffStress = 0.0,
    gearboxStress = 0.0,
    thermalStress = 0.0,

    wheelsValid = false,
    carLinked = false,
    brakeLinked = false,
    tyreLinked = false,
    memoryLinked = false,
    diffLinked = false,
    drivetrainLinked = false,
    roadLinked = false,
    contactLinked = false,

    status = "INIT",
    updateCount = 0,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

local function num(value, defaultValue)
    local n = tonumber(value)
    if n == nil or n ~= n or n == math.huge or n == -math.huge then
        return defaultValue or 0.0
    end
    return n
end

local function clamp(value, minValue, maxValue)
    value = num(value, minValue or 0.0)
    if value < minValue then return minValue end
    if value > maxValue then return maxValue end
    return value
end

local function abs(value)
    return math.abs(num(value, 0.0))
end

local function safeField(object, field, defaultValue)
    if not object then return defaultValue end
    local ok, value = pcall(function()
        return object[field]
    end)
    if not ok or value == nil then return defaultValue end
    return value
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
    return num(value, defaultValue or 0.0)
end

local function loadFirst(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return num(value, defaultValue or 0.0), keys[i]
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

local function safeGetCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function()
        return ac.getCar(0)
    end)
    if ok then return car end
    return nil
end

local function getWheels(car)
    return safeField(car, "wheels", nil)
end

local function getWheel(car, index)
    local wheels = getWheels(car)
    if not wheels then return nil end

    local ok, wheel = pcall(function()
        return wheels[index]
    end)
    if ok and wheel then return wheel end

    ok, wheel = pcall(function()
        return wheels[index + 1]
    end)
    if ok and wheel then return wheel end

    return nil
end

local function vecLength(v)
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

local function getSpeedKmh(car)
    local speed = safeField(car, "speedKmh", nil)
    if speed ~= nil then return num(speed, 0.0) end

    speed = nil
    if speed ~= nil then return num(speed, 0.0) * 3.6 end

    local velocity = safeField(car, "velocity", nil)
    if velocity then return vecLength(velocity) * 3.6 end

    local localVelocity = safeField(car, "localVelocity", nil)
    if localVelocity then return vecLength(localVelocity) * 3.6 end

    return safeLoad("ngp_hub_speed", safeLoad("ngp_thermal_speed", 0.0))
end

local function lowPass(current, target, tau, dt)
    current = num(current, 0.0)
    target = num(target, 0.0)
    tau = num(tau, 0.0)
    dt = num(dt, 0.0)
    if tau <= 0.0001 then return target end
    return current + (target - current) * clamp(dt / (tau + dt), 0.0, 1.0)
end

local function updateDebugGate(dt)
    state.debugStoreTimer = (state.debugStoreTimer or 0.0) + (dt or 0.0)
    if state.debugStoreTimer >= (M.params.debugStoreInterval or 0.25) then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    else
        state.debugStoreNow = false
    end
end

local function approachThermal(currentTemp, heatInput, coolRate, ambient, dt)
    currentTemp = num(currentTemp, ambient)
    heatInput = num(heatInput, 0.0)
    coolRate = num(coolRate, 0.0)
    ambient = num(ambient, M.params.ambientTemp)
    dt = num(dt, 0.0)

    local nextTemp =
        currentTemp
        + heatInput * dt
        - (currentTemp - ambient) * coolRate * dt

    nextTemp = math.max(ambient, nextTemp)
    return clamp(nextTemp, ambient, M.params.maxTemp)
end

local function normalizedTemp(temp, ambient, ref)
    return clamp((num(temp, ambient) - ambient) / math.max(ref - ambient, 1.0), 0.0, 1.0)
end

local function readBrakeTemperature(wheel, index, ambient)
    local value = safeField(wheel, "brakeTemperature", nil)
    if value ~= nil then
        state.brakeLinked = true
        return num(value, ambient)
    end

    value = safeField(wheel, "discTemperature", nil)
    if value ~= nil then
        state.brakeLinked = true
        return num(value, ambient)
    end

    local storeValue, key = loadFirst(
        nil,
        "ngp_brake_temp_" .. index,
        "ngp_brake_disc_temp_" .. index,
        "ngp_brake_system_temp_" .. index,
        "ngp_brake_thermal_temp_" .. index,
        "ngp_hub_brake_temp_" .. index
    )

    if key ~= nil then
        state.brakeLinked = true
        return num(storeValue, ambient)
    end

    return state.brakeTemp[index] or ambient
end

local function readTyreTemperature(wheel, index, ambient)
    local fields = {
        "tyreCoreTemperature",
        "tyreMiddleTemperature",
        "tyreInsideTemperature",
        "tyreOutsideTemperature",
    }

    for i = 1, #fields do
        local value = safeField(wheel, fields[i], nil)
        if value ~= nil then
            state.tyreLinked = true
            return num(value, ambient)
        end
    end

    local storeValue, key = loadFirst(
        nil,
        "ngp_tyre_temp_" .. index,
        "ngp_tire_temp_" .. index,
        "ngp_memory_virtual_temp_" .. index,
        "ngp_tire_memory_virtual_temp_" .. index,
        "ngp_thermal_tire_temp_" .. index
    )

    if key ~= nil then
        state.tyreLinked = true
        return num(storeValue, ambient)
    end

    return state.tyreTemp[index] or ambient
end

local function readWheelSideInputs(index)
    local memory, memoryKey = loadFirst(
        0.0,
        "ngp_rubber_memory_" .. index,
        "ngp_tire_memory_" .. index,
        "ngp_memory_" .. index,
        "ngp_slide_memory_" .. index
    )

    if memoryKey ~= nil then state.memoryLinked = true end
    state.rubberMemory[index] = clamp(memory, 0.0, 1.5)

    local memoryHeat, heatKey = loadFirst(
        0.0,
        "ngp_tire_memory_heat_" .. index,
        "ngp_memory_heat_" .. index,
        "ngp_memory_micro_heat_" .. index,
        "ngp_memory_macro_heat_" .. index,
        "ngp_slip_thermal_memory_" .. index
    )

    if heatKey ~= nil then state.memoryLinked = true end
    state.tireMemoryHeat[index] = clamp(memoryHeat, 0.0, 1.5)

    local lock, lockKey = loadFirst(
        0.0,
        "ngp_brake_lock_" .. index,
        "ngp_brake_lock_state_" .. index
    )

    if lockKey ~= nil then state.brakeLinked = true end
    state.brakeLock[index] = clamp(lock, 0.0, 1.5)

    local shock, shockKey = loadFirst(
        0.0,
        "ngp_road_shock_" .. index,
        "ngp_rii_shock_" .. index,
        "ngp_road_impact_" .. index,
        "ngp_rii_impact_" .. index,
        "ngp_impact_wheel_" .. index
    )

    if shockKey ~= nil then state.roadLinked = true end
    state.roadShock[index] = clamp(shock, 0.0, 1.5)

    local loss, lossKey = loadFirst(
        0.0,
        "ngp_contact_loss_" .. index,
        "ngp_tire_contact_loss_" .. index,
        "ngp_tcr_contact_loss_" .. index,
        "ngp_road_surface_limit_" .. index,
        "ngp_rii_surface_limit_" .. index
    )

    if lossKey ~= nil then state.contactLinked = true end
    state.contactLoss[index] = clamp(loss, 0.0, 1.5)
end

local function updateHubTemperature(car, index, speedKmh, ambient, dt)
    local wheel = getWheel(car, index)

    local brakeTemp = readBrakeTemperature(wheel, index, ambient)
    local tyreTemp = readTyreTemperature(wheel, index, ambient)
    readWheelSideInputs(index)

    tyreTemp =
        tyreTemp
        + state.rubberMemory[index] * M.params.tireMemoryTempGain
        + state.tireMemoryHeat[index] * M.params.tireHeatTempGain

    local hubCool =
        M.params.hubCoolBase
        + speedKmh * M.params.hubCoolSpeed

    local hubHeat =
        (brakeTemp - state.hubTemp[index]) * M.params.brakeToHub
        + (tyreTemp - state.hubTemp[index]) * M.params.tyreToHub
        + state.brakeLock[index] * M.params.brakeLockHeatGain
        + state.roadShock[index] * M.params.roadShockHubGain
        + state.contactLoss[index] * M.params.contactLossHubGain

    state.hubTemp[index] = approachThermal(
        state.hubTemp[index],
        hubHeat,
        hubCool,
        ambient,
        dt
    )

    state.brakeTemp[index] = clamp(brakeTemp, ambient, M.params.maxTemp)
    state.tyreTemp[index] = clamp(tyreTemp, ambient, M.params.maxTemp)
end

local function readWheelOmega(wheel)
    return num(
        safeField(wheel, "angularSpeed", 0.0),
        0.0
    )
end

local function readRearWheelDiff(car)
    local rearLeft = getWheel(car, 2)
    local rearRight = getWheel(car, 3)

    if rearLeft or rearRight then
        local omegaL = readWheelOmega(rearLeft)
        local omegaR = readWheelOmega(rearRight)
        return abs(omegaL - omegaR)
    end

    local hubL = safeLoadRaw("ngp_hub_omega_2")
    local hubR = safeLoadRaw("ngp_hub_omega_3")
    if hubL ~= nil or hubR ~= nil then
        return abs(num(hubL, 0.0) - num(hubR, 0.0))
    end

    return 0.0
end

local function updateDiffTemperature(car, speedKmh, ambient, dt)
    local wheelDiff = readRearWheelDiff(car)

    local lsdDiff, diffKey = loadFirst(
        wheelDiff,
        "ngp_lsd_diff",
        "ngp_diff_diff",
        "ngp_lsd_sdiff",
        "ngp_diff_sdiff",
        "ngp_thermal_lsd_diff"
    )

    local lsdHeat, heatKey = loadFirst(
        0.0,
        "ngp_lsd_heat",
        "ngp_diff_heat",
        "ngp_thermal_lsd_heat"
    )

    local lsdLock, lockKey = loadFirst(
        0.0,
        "ngp_lsd_lock",
        "ngp_diff_lock",
        "ngp_hub_lsd_lock"
    )

    local diffTorque, torqueKey = loadFirst(
        0.0,
        "ngp_diff_input_torque_nm",
        "ngp_drivetrain_diff_input_torque_nm",
        "ngp_drive_torque",
        "ngp_dt_torque"
    )

    if diffKey or heatKey or lockKey or torqueKey then
        state.diffLinked = true
    end

    state.wheelDiff = abs(wheelDiff)
    state.lsdDiff = abs(lsdDiff)
    state.lsdHeat = clamp(lsdHeat, 0.0, 1000.0)
    state.lsdLock = clamp(lsdLock, 0.0, 1.0)
    state.diffTorque = abs(diffTorque)

    local diffActivity = math.max(state.wheelDiff, state.lsdDiff)

    local diffHeat =
        diffActivity * M.params.diffHeatScale
        + state.lsdHeat * M.params.diffLSDHeatGain
        + state.lsdLock * diffActivity * 0.65
        + state.diffTorque * M.params.diffTorqueHeatGain

    local diffCool =
        M.params.diffCoolBase
        + speedKmh * M.params.diffCoolSpeed

    state.diffTemp = approachThermal(
        state.diffTemp,
        diffHeat,
        diffCool,
        ambient,
        dt
    )

    state.diffHeat = diffHeat
end

local function updateGearboxTemperature(car, speedKmh, ambient, dt)
    local rpm = num(safeField(car, "rpm", safeLoad("ngp_engine_rpm", 0.0)), 0.0)
    local gas = num(safeField(car, "gas", safeLoad("ngp_throttle", 0.0)), 0.0)
    local gear = num(safeField(car, "gear", safeLoad("ngp_gear", 0.0)), 0.0)

    state.rpm = rpm
    state.gas = gas
    state.gear = gear

    local rpmRatio = clamp(rpm / 8000.0, 0.0, 1.2)
    local gearLoad = gas * math.max(abs(gear), 1.0) * 0.15

    local drivetrainHeat, heatKey = loadFirst(
        nil,
        "ngp_gearbox_heat",
        "ngp_drive_gearbox_heat",
        "ngp_drivetrain_gearbox_heat",
        "ngp_dt_gearbox_heat"
    )

    if drivetrainHeat == nil then
        drivetrainHeat = 20.0
    end

    local driveTorque, torqueKey = loadFirst(
        0.0,
        "ngp_drive_torque",
        "ngp_drivetrain_torque",
        "ngp_dt_torque",
        "ngp_drive_transmitted_torque"
    )

    local shiftShock, shockKey = loadFirst(
        0.0,
        "ngp_shift_shock",
        "ngp_drive_shift_shock",
        "ngp_drivetrain_shift_shock"
    )

    if heatKey or torqueKey or shockKey then
        state.drivetrainLinked = true
    end

    state.drivetrainHeat = num(drivetrainHeat, 0.0) / 120.0
    state.driveTorque = abs(driveTorque)
    state.shiftShock = clamp(shiftShock, 0.0, 2.0)

    local gearboxHeat =
        (
            rpmRatio
            + gearLoad
            + state.drivetrainHeat * M.params.drivetrainHeatGain
            + state.driveTorque * M.params.driveTorqueHeatGain
            + state.shiftShock * 0.16
        )
        * M.params.gearboxHeatScale

    local gearboxCool =
        M.params.gearboxCoolBase
        + speedKmh * M.params.gearboxCoolSpeed

    state.gearboxTemp = approachThermal(
        state.gearboxTemp,
        gearboxHeat,
        gearboxCool,
        ambient,
        dt
    )

    state.gearboxHeat = gearboxHeat
end

local function updateStress(ambient)
    local sumHub = 0.0
    local sumBrake = 0.0
    local sumTyre = 0.0
    local maxHub = ambient
    local maxBrake = ambient
    local maxTyre = ambient

    for i = 0, 3 do
        local hub = state.hubTemp[i] or ambient
        local brake = state.brakeTemp[i] or ambient
        local tyre = state.tyreTemp[i] or ambient

        sumHub = sumHub + hub
        sumBrake = sumBrake + brake
        sumTyre = sumTyre + tyre

        if hub > maxHub then maxHub = hub end
        if brake > maxBrake then maxBrake = brake end
        if tyre > maxTyre then maxTyre = tyre end
    end

    state.avgHubTemp = sumHub * 0.25
    state.avgBrakeTemp = sumBrake * 0.25
    state.avgTyreTemp = sumTyre * 0.25

    state.maxHubTemp = maxHub
    state.maxBrakeTemp = maxBrake
    state.maxTyreTemp = maxTyre

    state.hubStress = normalizedTemp(maxHub, ambient, M.params.stressRefHub)
    state.brakeStress = normalizedTemp(maxBrake, ambient, M.params.stressRefBrake)
    state.tyreStress = normalizedTemp(maxTyre, ambient, M.params.stressRefTyre)
    state.diffStress = normalizedTemp(state.diffTemp, ambient, M.params.stressRefDiff)
    state.gearboxStress = normalizedTemp(state.gearboxTemp, ambient, M.params.stressRefGearbox)

    state.thermalStress = clamp(
        math.max(state.hubStress, state.brakeStress, state.tyreStress, state.diffStress, state.gearboxStress),
        0.0,
        1.0
    )
end

local function resetLinks()
    state.carLinked = false
    state.brakeLinked = false
    state.tyreLinked = false
    state.memoryLinked = false
    state.diffLinked = false
    state.drivetrainLinked = false
    state.roadLinked = false
    state.contactLinked = false
end

local function exportState()
    safeStore("ngp_thermal_status", state.status or "UNKNOWN")
    safeStore("ngp_thermal_update_count", state.updateCount or 0)
    safeStore("ngp_thermal_wheels_valid", state.wheelsValid and 1 or 0)
    safeStore("ngp_thermal_speed", state.speed or 0.0)

    for i = 0, 3 do
        safeStore("ngp_hub_temp_" .. i, state.hubTemp[i] or M.params.ambientTemp)
        safeStore("ngp_thermal_hub_temp_" .. i, state.hubTemp[i] or M.params.ambientTemp)
        safeStore("ngp_th_hub_temp_" .. i, state.hubTemp[i] or M.params.ambientTemp)

        safeStore("ngp_brake_temp_read_" .. i, state.brakeTemp[i] or M.params.ambientTemp)
        safeStore("ngp_thermal_brake_temp_" .. i, state.brakeTemp[i] or M.params.ambientTemp)
        safeStore("ngp_th_brake_temp_" .. i, state.brakeTemp[i] or M.params.ambientTemp)

        safeStore("ngp_tyre_temp_" .. i, state.tyreTemp[i] or M.params.ambientTemp)
        safeStore("ngp_tire_temp_" .. i, state.tyreTemp[i] or M.params.ambientTemp)
        safeStore("ngp_thermal_tire_temp_" .. i, state.tyreTemp[i] or M.params.ambientTemp)
        safeStore("ngp_th_tire_temp_" .. i, state.tyreTemp[i] or M.params.ambientTemp)

        safeStore("ngp_rubber_memory_temp_" .. i, state.rubberMemory[i] or 0.0)
        safeStore("ngp_thermal_memory_heat_" .. i, state.tireMemoryHeat[i] or 0.0)
        safeStore("ngp_th_memory_heat_" .. i, state.tireMemoryHeat[i] or 0.0)
    end

    safeStore("ngp_diff_temp", state.diffTemp or M.params.ambientTemp)
    safeStore("ngp_gearbox_temp", state.gearboxTemp or M.params.ambientTemp)

    safeStore("ngp_thermal_diff_temp", state.diffTemp or M.params.ambientTemp)
    safeStore("ngp_thermal_gearbox_temp", state.gearboxTemp or M.params.ambientTemp)
    safeStore("ngp_th_diff_temp", state.diffTemp or M.params.ambientTemp)
    safeStore("ngp_th_gearbox_temp", state.gearboxTemp or M.params.ambientTemp)

    safeStore("ngp_thermal_diff_heat", state.diffHeat or 0.0)
    safeStore("ngp_thermal_gearbox_heat", state.gearboxHeat or 0.0)
    safeStore("ngp_thermal_wheel_diff", state.wheelDiff or 0.0)
    safeStore("ngp_thermal_lsd_diff", state.lsdDiff or 0.0)
    safeStore("ngp_thermal_lsd_heat", state.lsdHeat or 0.0)
    safeStore("ngp_thermal_drivetrain_heat", state.drivetrainHeat or 0.0)

    safeStore("ngp_thermal_avg_hub_temp", state.avgHubTemp or M.params.ambientTemp)
    safeStore("ngp_thermal_max_hub_temp", state.maxHubTemp or M.params.ambientTemp)
    safeStore("ngp_thermal_avg_brake_temp", state.avgBrakeTemp or M.params.ambientTemp)
    safeStore("ngp_thermal_max_brake_temp", state.maxBrakeTemp or M.params.ambientTemp)
    safeStore("ngp_thermal_avg_tire_temp", state.avgTyreTemp or M.params.ambientTemp)
    safeStore("ngp_thermal_max_tire_temp", state.maxTyreTemp or M.params.ambientTemp)

    safeStore("ngp_thermal_hub_stress", state.hubStress or 0.0)
    safeStore("ngp_thermal_brake_stress", state.brakeStress or 0.0)
    safeStore("ngp_thermal_tire_stress", state.tyreStress or 0.0)
    safeStore("ngp_thermal_diff_stress", state.diffStress or 0.0)
    safeStore("ngp_thermal_gearbox_stress", state.gearboxStress or 0.0)
    safeStore("ngp_thermal_stress", state.thermalStress or 0.0)
    safeStore("ngp_virtual_thermal_stress", state.thermalStress or 0.0)

    safeStore("ngp_brake_root_heat_avg", state.brakeStress or 0.0)
    safeStore("ngp_th_stress", state.thermalStress or 0.0)

    if not state.debugStoreNow then return end

    safeStore("ngp_thermal_car_linked", state.carLinked and 1 or 0)
    safeStore("ngp_thermal_brake_linked", state.brakeLinked and 1 or 0)
    safeStore("ngp_thermal_tyre_linked", state.tyreLinked and 1 or 0)
    safeStore("ngp_thermal_memory_linked", state.memoryLinked and 1 or 0)
    safeStore("ngp_thermal_diff_linked", state.diffLinked and 1 or 0)
    safeStore("ngp_thermal_drivetrain_linked", state.drivetrainLinked and 1 or 0)
    safeStore("ngp_thermal_road_linked", state.roadLinked and 1 or 0)
    safeStore("ngp_thermal_contact_linked", state.contactLinked and 1 or 0)

    safeStore("ngp_thermal_lsd_lock", state.lsdLock or 0.0)
    safeStore("ngp_thermal_diff_torque", state.diffTorque or 0.0)
    safeStore("ngp_thermal_drive_torque", state.driveTorque or 0.0)
    safeStore("ngp_thermal_shift_shock", state.shiftShock or 0.0)
end

local function updateAll(dt, car)
    local ambient = M.params.ambientTemp
    local speedKmh = getSpeedKmh(car)

    state.speed = speedKmh
    resetLinks()

    if car then
        state.carLinked = true
    end

    for i = 0, 3 do
        updateHubTemperature(car, i, speedKmh, ambient, dt)
    end

    updateDiffTemperature(car, speedKmh, ambient, dt)
    updateGearboxTemperature(car, speedKmh, ambient, dt)
    updateStress(ambient)
end

function M.init()
    state.status = "INIT"
    updateStress(M.params.ambientTemp)
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = num(dt, 0.0)
    if dt <= 0.0 then
        state.status = "BAD DT"
        exportState()
        return
    end

    updateDebugGate(dt)

    car = car or safeGetCar()

    local wheels = getWheels(car)
    if car and wheels then
        state.status = "RUNNING"
        state.wheelsValid = true
        updateAll(dt, car)
    elseif car then
        state.status = "NO WHEELS"
        state.wheelsValid = false
        updateAll(dt, car)
    else
        state.status = "NO CAR"
        state.wheelsValid = false
        updateAll(dt, nil)
    end

    exportState()
end

function M.getHubTemp(index)
    return state.hubTemp[index] or M.params.ambientTemp
end

function M.getBrakeTemp(index)
    return state.brakeTemp[index] or M.params.ambientTemp
end

function M.getTyreTemp(index)
    return state.tyreTemp[index] or M.params.ambientTemp
end

function M.getTireTemp(index)
    return M.getTyreTemp(index)
end

function M.getDiffTemp()
    return state.diffTemp or M.params.ambientTemp
end

function M.getGearboxTemp()
    return state.gearboxTemp or M.params.ambientTemp
end

function M.getStress()
    return state.thermalStress or 0.0
end

function M.getState()
    return state
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0
        return string.format(
            "%s Hub %.1f Brake %.1f Tire %.1f Mem %.2f Heat %.2f Lock %.2f Shock %.2f Loss %.2f",
            tostring(({ [0] = "FL", [1] = "FR", [2] = "RL", [3] = "RR" })[i] or i),
            state.hubTemp[i] or M.params.ambientTemp,
            state.brakeTemp[i] or M.params.ambientTemp,
            state.tyreTemp[i] or M.params.ambientTemp,
            state.rubberMemory[i] or 0.0,
            state.tireMemoryHeat[i] or 0.0,
            state.brakeLock[i] or 0.0,
            state.roadShock[i] or 0.0,
            state.contactLoss[i] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "Hub %.1f %.1f %.1f %.1f / Avg %.1f Max %.1f\n" ..
        "Brake %.1f %.1f %.1f %.1f / Tire %.1f %.1f %.1f %.1f\n" ..
        "Diff %.1f Gearbox %.1f / Heat D %.2f G %.2f\n" ..
        "Stress H %.2f B %.2f T %.2f D %.2f G %.2f Total %.2f\n" ..
        "Links Car:%s Brake:%s Tire:%s Mem:%s Diff:%s Drive:%s Road:%s CQ:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",

        state.hubTemp[0] or M.params.ambientTemp,
        state.hubTemp[1] or M.params.ambientTemp,
        state.hubTemp[2] or M.params.ambientTemp,
        state.hubTemp[3] or M.params.ambientTemp,
        state.avgHubTemp or M.params.ambientTemp,
        state.maxHubTemp or M.params.ambientTemp,

        state.brakeTemp[0] or M.params.ambientTemp,
        state.brakeTemp[1] or M.params.ambientTemp,
        state.brakeTemp[2] or M.params.ambientTemp,
        state.brakeTemp[3] or M.params.ambientTemp,

        state.tyreTemp[0] or M.params.ambientTemp,
        state.tyreTemp[1] or M.params.ambientTemp,
        state.tyreTemp[2] or M.params.ambientTemp,
        state.tyreTemp[3] or M.params.ambientTemp,

        state.diffTemp or M.params.ambientTemp,
        state.gearboxTemp or M.params.ambientTemp,
        state.diffHeat or 0.0,
        state.gearboxHeat or 0.0,

        state.hubStress or 0.0,
        state.brakeStress or 0.0,
        state.tyreStress or 0.0,
        state.diffStress or 0.0,
        state.gearboxStress or 0.0,
        state.thermalStress or 0.0,

        state.carLinked and "OK" or "NIL",
        state.brakeLinked and "OK" or "NIL",
        state.tyreLinked and "OK" or "NIL",
        state.memoryLinked and "OK" or "NIL",
        state.diffLinked and "OK" or "NIL",
        state.drivetrainLinked and "OK" or "NIL",
        state.roadLinked and "OK" or "NIL",
        state.contactLinked and "OK" or "NIL"
    )
end

return M
