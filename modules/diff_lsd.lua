---@diagnostic disable: undefined-global

--============================================================
-- diff_lsd.lua
-- ACNextGen V1.1.5 stable
-- Mechanical LSD torque capacity and observed fallback model
--============================================================

local M = {}

M.params = {
    autoReadDrivetrainIni = true,
    drivetrainIniRelativePath = "data/drivetrain.ini",
    drivetrainReloadEachUpdate = false,

    defaultPowerLock = 0.35,
    defaultCoastLock = 0.15,
    defaultPreloadTorqueNm = 40.0,

    defaultCamPowerDeg = 45.0,
    defaultCamCoastDeg = 75.0,
    clutchMu = 0.12,
    frictionFaces = 8.0,
    plateMeanRadius = 0.055,
    camPinRadius = 0.028,
    minCamAngleDeg = 15.0,
    maxCamAngleDeg = 85.0,

    normalizedDriveTorqueThreshold = 4.0,
    normalizedDriveTorqueToNm = 650.0,
    engineTorqueGuessNm = 260.0,
    coastTorqueGuessNm = 55.0,
    drivetrainEfficiency = 0.88,
    minTinForCapacity = 8.0,
    preloadOnlyTinNm = 80.0,
    maxInputTorqueNm = 4200.0,
    maxLockTorqueNm = 2200.0,

    tauLock = 0.050,
    tauUnlock = 0.100,
    torqueTau = 0.045,

    minDt = 0.0005,
    maxDt = 0.050,

    lowSpeedKmh = 2.0,
    lowOmegaThreshold = 0.20,
    omegaHoldSpeedKmh = 3.0,
    omegaFilterTau = 0.018,
    diffMemoryTau = 0.055,
    lockFloorMemory = 0.03,

    minLock = 0.0,
    maxLock = 1.0,

    loadRef = 3000.0,
    loadDiffReference = 2000.0,
    loadUnlockGain = 0.22,

    rearBiasGain = 0.22,
    memoryUnlockGain = 0.10,
    shiftShockUnlockGain = 0.10,
    contactLossUnlockGain = 0.14,
    hopUnlockGain = 0.16,
    armTwistUnlockGain = 0.10,
    lashUnlockGain = 0.08,
    brakeCoastAssist = 0.10,

    minStateFactor = 0.55,
    maxStateFactor = 1.18,

    observedFallbackEnabled = true,
    observedTau = 0.65,
    observedMinSpeedKmh = 3.0,
    observedTorqueRefNm = 650.0,
    observedMinLock = 0.05,
    observedMaxLock = 0.75,
    observedBaseLock = 0.10,
    observedTorqueGain = 0.42,
    observedLoadBalanceGain = 0.18,
    observedDiffUnlockGain = 0.55,
    observedCoastScale = 0.55,

    resistancePerRad = 90.0,
    heatGain = 0.010,
    heatCoolRate = 0.35,

    debugStoreInterval = 0.25,
}

local state = {
    lockRatio = 0.0,
    targetLock = 0.0,
    mechanicalLockRatio = 0.0,
    observedLockRatio = 0.0,
    observedTargetLock = 0.0,

    heat = 0.0,

    diff = 0.0,
    signedDiff = 0.0,
    diffFiltered = 0.0,
    signedDiffFiltered = 0.0,

    mode = "COAST",
    source = "INIT",
    status = "INIT",
    updateCount = 0,

    loadRL = 0.0,
    loadRR = 0.0,
    loadDiff = 0.0,
    rearBias = 0.50,
    rearMemory = 0.0,

    driveTorque = 0.0,
    driveTorqueAbs = 0.0,
    driveTorqueNm = 0.0,
    inputTorqueNm = 0.0,
    estimatedInputTorqueNm = 0.0,
    torqueSource = "NONE",

    shaftTwist = 0.0,
    shaftVelocity = 0.0,
    driveLash = 0.0,
    shiftShock = 0.0,
    contactLoss = 0.0,
    hopShock = 0.0,
    armTwist = 0.0,

    omegaL = 0.0,
    omegaR = 0.0,
    rawOmegaL = 0.0,
    rawOmegaR = 0.0,
    omegaLinked = false,

    lockTorqueNm = 0.0,
    preloadTorqueNm = 0.0,
    camTorqueNm = 0.0,
    actualResistanceTorqueNm = 0.0,
    klockPercent = 0.0,

    powerLockRatio = 0.0,
    coastLockRatio = 0.0,
    activeLockRatio = 0.0,
    camPowerDeg = 0.0,
    camCoastDeg = 0.0,
    activeCamDeg = 0.0,

    configLoaded = false,
    configAttempted = false,
    configStatus = "NOT LOADED",
    configSource = "NONE",
    configPath = "",
    configWarning = "NONE",
    drivetrainType = "UNKNOWN",

    finalRatio = 1.0,
    gearRatio = 1.0,
    gear = 0,
    gearRatios = {},

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

local function clamp(value, minValue, maxValue)
    value = safeNumber(value, minValue or 0.0)
    if value < minValue then return minValue end
    if value > maxValue then return maxValue end
    return value
end

local function abs(value)
    value = safeNumber(value, 0.0)
    return value < 0.0 and -value or value
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

local function safeLoadAlt(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return safeNumber(value, defaultValue or 0.0), keys[i]
        end
    end
    return defaultValue or 0.0, nil
end

local function safeStore(key, value)
    if not ac or not ac.store then return end
    pcall(function() ac.store(key, value) end)
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.001)
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

local function normalizeRatio(value, defaultValue)
    local v = safeNumber(value, defaultValue or 0.0)
    if v > 1.5 then v = v / 100.0 end
    return clamp(v, 0.0, 1.0)
end

local function trimString(value)
    return tostring(value or ""):gsub("^%s+", ""):gsub("%s+$", "")
end

local function updateEquivalentCamAngles()
    local p = M.params
    local geom = math.max(p.clutchMu * p.frictionFaces * p.plateMeanRadius, 0.00001)
    local rp = math.max(p.camPinRadius, 0.00001)

    local function fromRatio(ratio, fallback)
        ratio = clamp(ratio, 0.01, 1.25)
        local theta = math.deg(math.atan(geom / (rp * ratio)))
        return clamp(theta, p.minCamAngleDeg, p.maxCamAngleDeg)
    end

    state.camPowerDeg = fromRatio(state.powerLockRatio, p.defaultCamPowerDeg)
    state.camCoastDeg = fromRatio(state.coastLockRatio, p.defaultCamCoastDeg)
end

local function readTextIni(path)
    if not io or not io.open or not path then return nil end

    local okOpen, file = pcall(function() return io.open(path, "r") end)
    if not okOpen or not file then return nil end

    local ini = {}
    local section = nil

    local okRead = pcall(function()
        for line in file:lines() do
            line = tostring(line or "")
            line = line:gsub(";.*$", "")
            line = line:gsub("//.*$", "")
            line = trimString(line)

            if line ~= "" then
                local sectionName = line:match("^%[(.-)%]$")
                if sectionName then
                    section = trimString(sectionName)
                    ini[section] = ini[section] or {}
                else
                    local key, value = line:match("^([%w_]+)%s*=%s*(.+)$")
                    if key and value and section then
                        ini[section][trimString(key)] = trimString(value)
                    end
                end
            end
        end
    end)

    pcall(function() file:close() end)
    if not okRead then return nil end
    return ini
end

local function getCarId(car)
    local id = nil
    if id ~= nil then return tostring(id) end

    if ac and ac.getCarID then
        local ok, result = pcall(function() return ac.getCarID(0) end)
        if ok and result then return tostring(result) end
    end

    return nil
end

local function buildDrivetrainIniPath(car)
    local carId = getCarId(car)
    if not carId then return nil end

    if ac and ac.getFolder and ac.FolderID and ac.FolderID.ContentCars then
        local ok, root = pcall(function()
            return ac.getFolder(ac.FolderID.ContentCars)
        end)

        if ok and root then
            return tostring(root) .. "/" .. carId .. "/" .. M.params.drivetrainIniRelativePath
        end
    end

    return "content/cars/" .. carId .. "/" .. M.params.drivetrainIniRelativePath
end

local function applyConfigFallback(reason)
    state.configLoaded = true
    state.configAttempted = true
    state.configStatus = "FALLBACK: " .. tostring(reason or "UNKNOWN")
    state.configSource = "OBSERVED_FALLBACK"
    state.configWarning = tostring(reason or "UNKNOWN")
    state.drivetrainType = "UNKNOWN"

    state.powerLockRatio = M.params.defaultPowerLock
    state.coastLockRatio = M.params.defaultCoastLock
    state.preloadTorqueNm = M.params.defaultPreloadTorqueNm
    state.finalRatio = 1.0
    state.gearRatios = {}

    updateEquivalentCamAngles()
end

local function firstSection(ini, names)
    for _, name in ipairs(names) do
        if ini[name] then return ini[name], name end
    end
    return nil, ""
end

local function parseGearRatios(gears)
    state.gearRatios = {}
    state.finalRatio = 1.0

    if not gears then return end

    state.finalRatio = safeNumber(gears.FINAL, safeNumber(gears.FINAL_RATIO, 1.0))
    if state.finalRatio == 0.0 then state.finalRatio = 1.0 end

    for i = 1, 10 do
        local key = "GEAR_" .. i
        local ratio = safeNumber(gears[key], nil)
        if ratio ~= nil and ratio ~= 0.0 then
            state.gearRatios[i] = ratio
        end
    end
end

local function loadDrivetrainConfig(car)
    if not M.params.autoReadDrivetrainIni then
        applyConfigFallback("AUTO DISABLED")
        return false
    end

    if state.configLoaded and not M.params.drivetrainReloadEachUpdate then
        return state.configSource == "INI"
    end

    state.configAttempted = true
    state.configLoaded = false
    state.configStatus = "LOADING"

    local path = buildDrivetrainIniPath(car)
    state.configPath = tostring(path or "")

    if not path then
        applyConfigFallback("NO CAR PATH")
        return false
    end

    local ini = readTextIni(path)
    if not ini then
        applyConfigFallback("INI NOT FOUND OR ACD PACKED")
        return false
    end

    local diff, diffName = firstSection(ini, {
        "DIFFERENTIAL",
        "DIFFERENTIAL_0",
        "DIFF",
        "REAR_DIFFERENTIAL",
    })

    if not diff then
        applyConfigFallback("NO DIFFERENTIAL SECTION")
        return false
    end

    local traction = ini.TRACTION or ini.DRIVETRAIN
    if traction and traction.TYPE then
        state.drivetrainType = tostring(traction.TYPE)
    else
        state.drivetrainType = "UNKNOWN"
    end

    parseGearRatios(ini.GEARS or ini.GEARBOX)

    state.powerLockRatio = normalizeRatio(diff.POWER, M.params.defaultPowerLock)
    state.coastLockRatio = normalizeRatio(diff.COAST, M.params.defaultCoastLock)
    state.preloadTorqueNm = clamp(safeNumber(diff.PRELOAD, M.params.defaultPreloadTorqueNm), 0.0, 500.0)

    updateEquivalentCamAngles()

    if diff.CAM_POWER or diff.POWER_ANGLE or diff.CAM_1 then
        state.camPowerDeg = clamp(
            safeNumber(diff.CAM_POWER, safeNumber(diff.POWER_ANGLE, safeNumber(diff.CAM_1, M.params.defaultCamPowerDeg))),
            M.params.minCamAngleDeg,
            M.params.maxCamAngleDeg
        )
    end

    if diff.CAM_COAST or diff.COAST_ANGLE or diff.CAM_2 then
        state.camCoastDeg = clamp(
            safeNumber(diff.CAM_COAST, safeNumber(diff.COAST_ANGLE, safeNumber(diff.CAM_2, M.params.defaultCamCoastDeg))),
            M.params.minCamAngleDeg,
            M.params.maxCamAngleDeg
        )
    end

    state.configLoaded = true
    state.configSource = "INI"
    state.configStatus = "OK: " .. tostring(diffName)
    state.configWarning = "NONE"
    return true
end

local function getWheel(car, index)
    if not car or not car.wheels then return nil end
    local ok, wheel = pcall(function() return car.wheels[index] end)
    if ok and wheel then return wheel end
    ok, wheel = pcall(function() return car.wheels[index + 1] end)
    if ok then return wheel end
    return nil
end

local function readOmegaFromWheel(wheel, index)
    if wheel then
        local fields = {
            "angularSpeed",
        }

        for i = 1, #fields do
            local value = safeField(wheel, fields[i], nil)
            if value ~= nil then
                return safeNumber(value, 0.0), "WHEEL"
            end
        end
    end

    local value, key = safeLoadAlt(0.0,
        "ngp_wheel_omega_" .. index,
        "ngp_tire_omega_" .. index,
        "ngp_tire_state_omega_" .. index,
        "ngp_tdyn_omega_" .. index
    )

    if key then return value, key end
    return nil, "NONE"
end

local function readRearWheelOmega(car)
    local rl = getWheel(car, 2)
    local rr = getWheel(car, 3)
    if not rl and not rr then return nil, nil end

    local omegaL, srcL = readOmegaFromWheel(rl, 2)
    local omegaR, srcR = readOmegaFromWheel(rr, 3)

    if omegaL == nil or omegaR == nil then
        return nil, nil
    end

    state.omegaLinked = srcL ~= "NONE" or srcR ~= "NONE"
    return omegaL, omegaR
end

local function stabilizeLowSpeedOmega(speedKmh, omegaL, omegaR, dt)
    state.rawOmegaL = omegaL
    state.rawOmegaR = omegaR

    local omegaSum = abs(omegaL) + abs(omegaR)
    if speedKmh < M.params.lowSpeedKmh and omegaSum < M.params.lowOmegaThreshold then
        return state.omegaL, state.omegaR
    end

    local hold = speedKmh < M.params.omegaHoldSpeedKmh and omegaSum < M.params.lowOmegaThreshold
    if hold then
        return state.omegaL, state.omegaR
    end

    local filteredL = lowPass(state.omegaL, omegaL, M.params.omegaFilterTau, dt)
    local filteredR = lowPass(state.omegaR, omegaR, M.params.omegaFilterTau, dt)
    return filteredL, filteredR
end

local function readRearLoad(index)
    local value, key = safeLoadAlt(nil,
        "ngp_contact_load_" .. index,
        "ngp_tire_state_load_" .. index,
        "ngp_wheel_load_" .. index,
        "ngp_sprung_load_" .. index,
        "ngp_load_path_wheel_load_" .. index
    )

    if key then return abs(value) end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        return math.max(0.0, M.params.loadRef + safeNumber(dlt, 0.0))
    end

    return 0.0
end

local function updateLoadState()
    state.loadRL = readRearLoad(2)
    state.loadRR = readRearLoad(3)
    state.loadDiff = abs(state.loadRL - state.loadRR)
end

local function getRearMemory()
    local memRL = safeLoad("ngp_tire_memory_2", safeLoad("ngp_rubber_memory_2", safeLoad("ngp_memory_2", 0.0)))
    local memRR = safeLoad("ngp_tire_memory_3", safeLoad("ngp_rubber_memory_3", safeLoad("ngp_memory_3", 0.0)))
    return clamp((memRL + memRR) * 0.5, 0.0, 1.0)
end

local function readRearContactLoss()
    local loss2 = safeLoadRaw("ngp_contact_loss_2")
    local loss3 = safeLoadRaw("ngp_contact_loss_3")

    if loss2 == nil then
        local q = safeLoadRaw("ngp_contact_quality_2") or safeLoadRaw("ngp_tire_contact_quality_2") or safeLoadRaw("ngp_tcr_quality_2")
        loss2 = q ~= nil and (1.0 - safeNumber(q, 1.0)) or nil
    end

    if loss3 == nil then
        local q = safeLoadRaw("ngp_contact_quality_3") or safeLoadRaw("ngp_tire_contact_quality_3") or safeLoadRaw("ngp_tcr_quality_3")
        loss3 = q ~= nil and (1.0 - safeNumber(q, 1.0)) or nil
    end

    local driveLoss = safeLoadRaw("ngp_drive_contact_loss")
    if driveLoss ~= nil then
        return clamp(safeNumber(driveLoss, 0.0), 0.0, 1.0)
    end

    return clamp((safeNumber(loss2, 0.0) + safeNumber(loss3, 0.0)) * 0.5, 0.0, 1.0)
end

local function readDrivetrainState()
    state.driveTorque = safeLoad("ngp_drive_torque", safeLoad("ngp_driveline_torque", 0.0))
    state.driveTorqueAbs = abs(state.driveTorque)

    state.shaftTwist = safeLoad("ngp_shaft_twist", safeLoad("ngp_driveline_twist", safeLoad("ngp_windup_value", 0.0)))
    state.shaftVelocity = safeLoad("ngp_shaft_velocity", safeLoad("ngp_windup_velocity", 0.0))
    state.driveLash = safeLoad("ngp_drive_lash", safeLoad("ngp_driveline_lash", 0.0))
    state.shiftShock = safeLoad("ngp_shift_shock", safeLoad("ngp_driveline_shift_shock", 0.0))

    state.contactLoss = readRearContactLoss()

    state.hopShock = clamp(
        safeLoad("ngp_drive_hop_shock", safeLoad("ngp_tire_hop_avg_instability", safeLoad("ngp_damper_hyst_avg_impact", 0.0))),
        0.0,
        1.0
    )

    state.armTwist = abs(safeLoad("ngp_control_arm_total", safeLoad("ngp_arm_total", safeLoad("ngp_compliance_rear_axle_windup", 0.0))))
end

local function readGear(car)
    local gear = safeNumber(safeField(car, "gear", 0), 0)
    state.gear = gear

    local ratio = state.gearRatios[gear]
    if ratio == nil or ratio == 0.0 then ratio = 1.0 end

    state.gearRatio = ratio
    return gear, ratio
end

local function estimateInputTorqueNm(car, gas, brake, powerMode)
    local _, gearRatio = readGear(car)
    local finalRatio = state.finalRatio or 1.0
    local totalRatio = abs(gearRatio * finalRatio)
    if totalRatio < 0.05 then totalRatio = 1.0 end

    local engineTorque
    if powerMode then
        engineTorque = M.params.engineTorqueGuessNm * clamp(gas, 0.0, 1.0)
    else
        engineTorque = M.params.coastTorqueGuessNm * (0.35 + 0.65 * clamp(brake, 0.0, 1.0))
    end

    local tin = engineTorque * totalRatio * M.params.drivetrainEfficiency
    return clamp(abs(tin), 0.0, M.params.maxInputTorqueNm)
end

local function calculateInputTorqueNm(car, gas, brake, powerMode, dt)
    local raw = state.driveTorque
    local absRaw = abs(raw)
    local tin

    if absRaw > 0.0001 then
        if absRaw <= M.params.normalizedDriveTorqueThreshold then
            tin = absRaw * M.params.normalizedDriveTorqueToNm
            state.torqueSource = "NGP_NORMALIZED"
        else
            tin = absRaw
            state.torqueSource = "NGP_NM"
        end
    else
        tin = estimateInputTorqueNm(car, gas, brake, powerMode)
        state.torqueSource = "ESTIMATED"
    end

    tin = clamp(tin, 0.0, M.params.maxInputTorqueNm)
    state.estimatedInputTorqueNm = tin
    state.inputTorqueNm = lowPass(state.inputTorqueNm, tin, M.params.torqueTau, dt)
    state.driveTorqueNm = state.inputTorqueNm
    return state.inputTorqueNm
end

local function calculateCamLockRatioFromAngle(thetaDeg)
    local p = M.params
    local theta = math.rad(clamp(thetaDeg, p.minCamAngleDeg, p.maxCamAngleDeg))
    local tanTheta = math.max(math.tan(theta), 0.001)

    local ratio =
        (p.clutchMu * p.frictionFaces * p.plateMeanRadius)
        /
        (math.max(p.camPinRadius, 0.00001) * tanTheta)

    return clamp(ratio, 0.0, 1.25)
end

local function calculateMechanicalCapacity(tin, powerMode)
    local p = M.params
    local activeRatio
    local activeCam

    if powerMode then
        activeRatio = state.powerLockRatio
        activeCam = state.camPowerDeg
        state.mode = "POWER"
    else
        activeRatio = state.coastLockRatio
        activeCam = state.camCoastDeg
        state.mode = "COAST"
    end

    local camRatio = calculateCamLockRatioFromAngle(activeCam)
    if state.configSource == "INI" then
        camRatio = activeRatio
    end

    state.activeLockRatio = activeRatio
    state.activeCamDeg = activeCam

    local effectiveTin = math.max(tin, p.minTinForCapacity)
    state.preloadTorqueNm = clamp(state.preloadTorqueNm or p.defaultPreloadTorqueNm, 0.0, 500.0)
    state.camTorqueNm = clamp(effectiveTin * camRatio, 0.0, p.maxLockTorqueNm)
    state.lockTorqueNm = clamp(state.camTorqueNm + state.preloadTorqueNm, 0.0, p.maxLockTorqueNm)

    local capacityRatio = state.lockTorqueNm / effectiveTin
    if tin < p.preloadOnlyTinNm then
        local preloadOnly = state.preloadTorqueNm / math.max(p.preloadOnlyTinNm, 1.0)
        capacityRatio = math.max(preloadOnly, camRatio * (tin / p.preloadOnlyTinNm))
    end

    state.klockPercent = (state.lockTorqueNm / math.max(effectiveTin, 1.0)) * 100.0
    state.mechanicalLockRatio = clamp(capacityRatio, p.minLock, p.maxLock)
    return state.mechanicalLockRatio
end

local function applyStateCorrections(targetLock, brake)
    local p = M.params

    local rearBias = safeLoad("ngp_rear_bias", safeLoad("ngp_load_rear", 0.50))
    state.rearBias = clamp(rearBias, 0.0, 1.0)

    local rearBiasFactor = 1.0 + (state.rearBias - 0.50) * p.rearBiasGain

    local loadRatio = clamp(state.loadDiff / p.loadDiffReference, 0.0, 1.0)
    local loadFactor = 1.0 - loadRatio * p.loadUnlockGain

    state.rearMemory = getRearMemory()
    local memoryFactor = 1.0 - state.rearMemory * p.memoryUnlockGain

    local disturbance =
        state.shiftShock * p.shiftShockUnlockGain
        + state.contactLoss * p.contactLossUnlockGain
        + state.hopShock * p.hopUnlockGain
        + state.armTwist * p.armTwistUnlockGain
        + abs(state.driveLash) * p.lashUnlockGain

    disturbance = clamp(disturbance, 0.0, 0.35)

    local factor = rearBiasFactor * loadFactor * memoryFactor * (1.0 - disturbance)
    factor = clamp(factor, p.minStateFactor, p.maxStateFactor)

    local corrected = targetLock * factor
    if state.mode == "COAST" then
        corrected = corrected + clamp(brake, 0.0, 1.0) * p.brakeCoastAssist
    end

    if state.diffFiltered > 0.05 then
        corrected = math.max(corrected, p.lockFloorMemory)
    end

    return clamp(corrected, p.minLock, p.maxLock)
end

local function calculateObservedFallbackLock(speedKmh, tin, powerMode)
    local p = M.params

    if not p.observedFallbackEnabled then
        return p.defaultPowerLock
    end

    local omegaAbs = math.max(abs(state.omegaL), abs(state.omegaR), 1.0)
    local diffNorm = clamp(state.diff / omegaAbs, 0.0, 1.0)
    local torqueLevel = clamp(tin / math.max(p.observedTorqueRefNm, 1.0), 0.0, 1.0)

    local loadTotal = math.max(state.loadRL + state.loadRR, 1.0)
    local loadBalance = clamp(1.0 - state.loadDiff / loadTotal, 0.0, 1.0)

    local target =
        p.observedBaseLock
        + torqueLevel * p.observedTorqueGain
        + loadBalance * p.observedLoadBalanceGain
        - diffNorm * p.observedDiffUnlockGain

    if not powerMode then
        target = target * p.observedCoastScale
    end

    if speedKmh < p.observedMinSpeedKmh then
        target = math.max(p.observedBaseLock, state.observedTargetLock * 0.70)
    end

    target = clamp(target, p.observedMinLock, p.observedMaxLock)
    state.observedTargetLock = target
    state.observedLockRatio = lowPass(state.observedLockRatio, target, p.observedTau, M.params._dt or 0.001)
    return clamp(state.observedLockRatio, p.minLock, p.maxLock)
end

local function updateDiffFilters(dt)
    state.diffFiltered = lowPass(state.diffFiltered, state.diff, M.params.diffMemoryTau, dt)
    state.signedDiffFiltered = lowPass(state.signedDiffFiltered, state.signedDiff, M.params.diffMemoryTau, dt)
end

local function updateResistanceTorque()
    local demand = abs(state.diffFiltered) * M.params.resistancePerRad
    state.actualResistanceTorqueNm = math.min(state.lockTorqueNm, demand)
end

local function updateHeat(dt)
    local heatRate =
        abs(state.actualResistanceTorqueNm)
        * abs(state.diffFiltered)
        * M.params.heatGain

    local cool = state.heat * M.params.heatCoolRate * dt
    state.heat = math.max(0.0, state.heat + heatRate * dt - cool)
end

local function calculateTargetLock(car, speedKmh, gas, brake, powerMode, dt)
    local tin = calculateInputTorqueNm(car, gas, brake, powerMode, dt)

    if state.configSource == "INI" then
        state.source = "INI_MECHANICAL"
        local mechanical = calculateMechanicalCapacity(tin, powerMode)
        return applyStateCorrections(mechanical, brake)
    end

    state.source = "OBSERVED_FALLBACK"
    local observed = calculateObservedFallbackLock(speedKmh, tin, powerMode)

    state.preloadTorqueNm = M.params.defaultPreloadTorqueNm
    state.camTorqueNm = tin * observed
    state.lockTorqueNm = clamp(state.camTorqueNm + state.preloadTorqueNm, 0.0, M.params.maxLockTorqueNm)
    state.klockPercent = (state.lockTorqueNm / math.max(tin, M.params.minTinForCapacity)) * 100.0
    state.mechanicalLockRatio = observed

    return applyStateCorrections(observed, brake)
end

local function exportState()
    safeStore("ngp_lsd_lock", state.lockRatio)
    safeStore("ngp_diff_lock", state.lockRatio)

    safeStore("ngp_lsd_diff", state.diff)
    safeStore("ngp_diff_diff", state.diff)

    safeStore("ngp_lsd_sdiff", state.signedDiff)
    safeStore("ngp_lsd_heat", state.heat)
    safeStore("ngp_diff_heat", state.heat)

    safeStore("ngp_lsd_mode", state.mode)
    safeStore("ngp_lsd_target", state.targetLock)
    safeStore("ngp_lsd_status", state.status)
    safeStore("ngp_lsd_update_count", state.updateCount)

    safeStore("ngp_lsd_source", state.source)
    safeStore("ngp_lsd_config_source", state.configSource)

    safeStore("ngp_lsd_input_torque_nm", state.inputTorqueNm)
    safeStore("ngp_lsd_lock_torque", state.lockTorqueNm)
    safeStore("ngp_lsd_lock_torque_nm", state.lockTorqueNm)
    safeStore("ngp_lsd_preload_torque", state.preloadTorqueNm)
    safeStore("ngp_lsd_preload_torque_nm", state.preloadTorqueNm)
    safeStore("ngp_lsd_cam_torque", state.camTorqueNm)
    safeStore("ngp_lsd_cam_torque_nm", state.camTorqueNm)
    safeStore("ngp_lsd_resist_torque", state.actualResistanceTorqueNm)
    safeStore("ngp_lsd_resist_torque_nm", state.actualResistanceTorqueNm)
    safeStore("ngp_lsd_klock_percent", state.klockPercent)
    safeStore("ngp_lsd_mechanical_lock", state.mechanicalLockRatio)
    safeStore("ngp_lsd_observed_lock", state.observedLockRatio)

    safeStore("ngp_diff_power", state.powerLockRatio)
    safeStore("ngp_diff_coast", state.coastLockRatio)
    safeStore("ngp_diff_preload", state.preloadTorqueNm)
    safeStore("ngp_diff_forceL", -state.actualResistanceTorqueNm)
    safeStore("ngp_diff_forceR", state.actualResistanceTorqueNm)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_lsd_omega_l", state.omegaL)
    safeStore("ngp_lsd_omega_r", state.omegaR)
    safeStore("ngp_lsd_raw_omega_l", state.rawOmegaL)
    safeStore("ngp_lsd_raw_omega_r", state.rawOmegaR)
    safeStore("ngp_lsd_omega_linked", state.omegaLinked and 1 or 0)

    safeStore("ngp_lsd_load_diff", state.loadDiff)
    safeStore("ngp_lsd_load_rl", state.loadRL)
    safeStore("ngp_lsd_load_rr", state.loadRR)

    safeStore("ngp_lsd_diff_filtered", state.diffFiltered)
    safeStore("ngp_lsd_sdiff_filtered", state.signedDiffFiltered)

    safeStore("ngp_lsd_drive_torque", state.driveTorque)
    safeStore("ngp_lsd_drive_torque_nm", state.driveTorqueNm)
    safeStore("ngp_lsd_torque_source", state.torqueSource)

    safeStore("ngp_lsd_power_ratio", state.powerLockRatio)
    safeStore("ngp_lsd_coast_ratio", state.coastLockRatio)
    safeStore("ngp_lsd_active_ratio", state.activeLockRatio)
    safeStore("ngp_lsd_cam_power_deg", state.camPowerDeg)
    safeStore("ngp_lsd_cam_coast_deg", state.camCoastDeg)
    safeStore("ngp_lsd_active_cam_deg", state.activeCamDeg)

    safeStore("ngp_lsd_config_status", state.configStatus)
    safeStore("ngp_lsd_config_warning", state.configWarning)
    safeStore("ngp_lsd_config_path", state.configPath)
    safeStore("ngp_lsd_drivetrain_type", state.drivetrainType)
    safeStore("ngp_lsd_final_ratio", state.finalRatio)
    safeStore("ngp_lsd_gear", state.gear)
    safeStore("ngp_lsd_gear_ratio", state.gearRatio)

    safeStore("ngp_lsd_shaft_twist", state.shaftTwist)
    safeStore("ngp_lsd_shift_shock", state.shiftShock)
    safeStore("ngp_lsd_contact_loss", state.contactLoss)
    safeStore("ngp_lsd_hop_shock", state.hopShock)
    safeStore("ngp_lsd_arm_twist", state.armTwist)
    safeStore("ngp_lsd_rear_memory", state.rearMemory)
end

function M.init(car)
    state.status = "INIT"
    car = car or safeGetCar()
    loadDrivetrainConfig(car)
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

    dt = clamp(dt, M.params.minDt, M.params.maxDt)
    M.params._dt = dt
    updateDebugGate(dt)

    car = car or safeGetCar()

    if not car then
        state.status = "NO CAR"
        exportState()
        return
    end

    local wheels = safeField(car, "wheels", nil)
    if not wheels then
        state.status = "NO WHEELS"
        exportState()
        return
    end

    if not state.configLoaded or M.params.drivetrainReloadEachUpdate then
        loadDrivetrainConfig(car)
    end

    local omegaL, omegaR = readRearWheelOmega(car)
    if omegaL == nil or omegaR == nil then
        state.status = "NO OMEGA"
        omegaL, omegaR = state.omegaL, state.omegaR
    else
        state.status = "RUNNING"
    end

    local speedKmh = safeNumber(safeField(car, "speedKmh", 0.0), 0.0)
    omegaL, omegaR = stabilizeLowSpeedOmega(speedKmh, omegaL, omegaR, dt)

    state.omegaL = omegaL
    state.omegaR = omegaR
    state.signedDiff = omegaL - omegaR
    state.diff = abs(state.signedDiff)

    updateDiffFilters(dt)
    updateLoadState()
    readDrivetrainState()

    local gas = clamp(safeNumber(safeField(car, "gas", 0.0), 0.0), 0.0, 1.0)
    local brake = clamp(safeNumber(safeField(car, "brake", 0.0), 0.0), 0.0, 1.0)

    local powerMode
    if state.driveTorque < -0.01 then
        powerMode = false
    elseif state.driveTorque > 0.01 then
        powerMode = true
    else
        powerMode = gas > 0.05
    end

    state.targetLock = calculateTargetLock(car, speedKmh, gas, brake, powerMode, dt)

    local tau = state.targetLock > state.lockRatio and M.params.tauLock or M.params.tauUnlock
    state.lockRatio = lowPass(state.lockRatio, state.targetLock, tau, dt)
    state.lockRatio = clamp(state.lockRatio, M.params.minLock, M.params.maxLock)

    updateResistanceTorque()
    updateHeat(dt)

    exportState()
end

function M.getLock()
    return state.lockRatio or 0.0
end

function M.getTargetLock()
    return state.targetLock or 0.0
end

function M.getDiff()
    return state.diff or 0.0
end

function M.getSignedDiff()
    return state.signedDiff or 0.0
end

function M.getHeat()
    return state.heat or 0.0
end

function M.getMode()
    return state.mode or "UNKNOWN"
end

function M.getLockTorque()
    return state.lockTorqueNm or 0.0
end

function M.getInputTorque()
    return state.inputTorqueNm or 0.0
end

function M.getState()
    return state
end

function M.debugStr()
    return string.format(
        "Status %s / Lock %.1f%% Target %.1f%% Mode %s Src %s\n" ..
        "Tin %.0fNm Tlock %.0fNm Pre %.0f Cam %.0f K %.1f%%\n" ..
        "Diff %.3f Signed %.3f Filt %.3f Heat %.1f\n" ..
        "INI %s P %.2f C %.2f Cam %.0f/%.0f\n" ..
        "OmegaL %.2f OmegaR %.2f LoadDiff %.0f",
        tostring(state.status),
        (state.lockRatio or 0.0) * 100.0,
        (state.targetLock or 0.0) * 100.0,
        tostring(state.mode),
        tostring(state.source),
        state.inputTorqueNm or 0.0,
        state.lockTorqueNm or 0.0,
        state.preloadTorqueNm or 0.0,
        state.camTorqueNm or 0.0,
        state.klockPercent or 0.0,
        state.diff or 0.0,
        state.signedDiff or 0.0,
        state.diffFiltered or 0.0,
        state.heat or 0.0,
        tostring(state.configSource),
        state.powerLockRatio or 0.0,
        state.coastLockRatio or 0.0,
        state.camPowerDeg or 0.0,
        state.camCoastDeg or 0.0,
        state.omegaL or 0.0,
        state.omegaR or 0.0,
        state.loadDiff or 0.0
    )
end

return M
