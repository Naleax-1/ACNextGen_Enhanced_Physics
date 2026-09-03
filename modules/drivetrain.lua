---@diagnostic disable: undefined-global

--============================================================
-- ACNeXtGen
-- drivetrain.lua
-- Phase 3.1 / V1.1.5 Stable
-- Drivetrain / Lash / Shaft / Torque Path Model
-- App-side observer/bridge. No direct AC physics overwrite.
--============================================================

local M = {}

M.params = {
    rpmTau = 0.080,
    gearInertia = 0.120,
    lash = 0.080,
    lashTau = 0.040,
    shaftStiffness = 18.0,
    shaftDamping = 2.0,
    clutchTau = 0.150,
    shiftShockScale = 0.015,
    shiftShockGain = 0.22,
    shiftShockDecay = 5.0,
    gearboxHeatGain = 0.100,
    lsdResistanceGain = 0.150,
    hopShockReference = 5000.0,
    hopTorqueLossGain = 0.120,
    gearboxAmbient = 20.0,
    gearboxCoolRate = 0.0025,

    contactTorqueLossGain = 0.10,
    contactImpulseShockGain = 0.08,
    loadDiffTorqueLossGain = 0.08,
    loadDiffReference = 2400.0,
    loadRef = 3000.0,

    controlArmTwistGain = 0.10,
    chassisFlexTorqueLossGain = 0.08,
    chassisReleaseKickGain = 0.06,

    bodyFlexLashGain = 0.22,
    bodyTorsionShaftGain = 0.16,
    bodyDampingShaftGain = 0.08,
    bodyStiffShaftLoss = 0.06,
    minBodyLashScale = 0.86,
    maxBodyLashScale = 1.24,
    minBodyShaftScale = 0.84,
    maxBodyShaftScale = 1.22,

    maxShaftAngle = 2.50,
    maxShaftVelocity = 12.0,

    configReadInterval = 1.00,
    fallbackPeakTorqueNm = 360.0,
    fallbackRedlineRpm = 7200.0,
    engineBrakeTorqueNm = 70.0,
    drivelineEfficiency = 0.92,
    clutchTorqueCapacityNm = 520.0,
    clutchSlipLossGain = 0.18,
    normalizedTorqueReferenceNm = 1200.0,
    maxEngineTorqueNm = 1400.0,
    maxDiffInputTorqueNm = 22000.0,
    maxWheelTorqueNm = 12000.0,
    torqueNmTau = 0.035,

    defaultFinalRatio = 4.10,
    defaultGearRatio1 = 3.20,
    defaultGearRatio2 = 2.10,
    defaultGearRatio3 = 1.50,
    defaultGearRatio4 = 1.15,
    defaultGearRatio5 = 0.90,
    defaultGearRatio6 = 0.75,
    minGearRatioAbs = 0.001,
    usePowerLut = true,

    minDt = 0.0005,
    maxDt = 0.100,
    debugStoreInterval = 0.25,
    bodyReadInterval = 0.25,
}

local state = {
    status = "INIT",
    updateCount = 0,

    rawRPM = 0.0,
    filteredRPM = 0.0,
    gas = 0.0,
    brake = 0.0,
    clutchInput = 1.0,
    clutch = 1.0,
    gear = 0,
    prevGear = 0,

    engineTorque = 0.0,
    transmittedTorque = 0.0,
    driveTorque = 0.0,

    engineTorqueNm = 0.0,
    engineBrakeTorqueNm = 0.0,
    clutchTorqueNm = 0.0,
    gearboxOutputTorqueNm = 0.0,
    diffInputTorqueNm = 0.0,
    wheelDriveTorqueNm = 0.0,
    normalizedTorqueFromNm = 0.0,

    gearRatio = 0.0,
    finalRatio = 4.10,
    drivelineEfficiency = 0.92,
    peakTorqueNm = 360.0,
    redlineRpm = 7200.0,

    configLoaded = false,
    configSource = "UNLOADED",
    configStatus = "INIT",
    configWarning = "NONE",
    configPath = "",
    carId = "",
    drivetrainType = "UNKNOWN",
    gearCount = 0,
    gearRatios = {},
    powerLut = {},
    powerLutLoaded = false,
    configReadTimer = 999.0,

    shaftAngle = 0.0,
    shaftVelocity = 0.0,
    lashState = 0.0,

    shiftShock = 0.0,
    shiftStress = 0.0,
    gearboxHeat = 20.0,

    lsdResistance = 0.0,
    lsdLock = 0.0,
    lsdDiff = 0.0,

    hopShock = 0.0,
    contactShock = 0.0,
    contactLoss = 0.0,
    contactQualityRear = 1.0,
    contactTrustRear = 1.0,

    loadRL = 3000.0,
    loadRR = 3000.0,
    loadDiff = 0.0,
    loadLoss = 0.0,

    controlArmTotal = 0.0,
    armTwistLoss = 0.0,

    bodySteer = 0.0,
    flexEnergy = 0.0,
    flexRelease = 0.0,
    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyDampingFactor = 1.0,
    bodyLashScale = 1.0,
    bodyShaftScale = 1.0,
    bodyLinked = false,
    bodyReadTimer = 999.0,

    finalTorqueScale = 1.0,

    linkedLSD = false,
    linkedContact = false,
    linkedBody = false,
    linkedArm = false,
    linkedHop = false,
    linkedLoad = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

local function toNumber(value)
    local n = tonumber(value)
    if n == nil or n ~= n or n == math.huge or n == -math.huge then
        return nil
    end
    return n
end

local function safeNumber(value, defaultValue)
    local n = toNumber(value)
    if n == nil then
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
    v = safeNumber(v, 0.0)
    return v < 0.0 and -v or v
end

local function sign(v)
    v = safeNumber(v, 0.0)
    if v > 0.000001 then return 1.0 end
    if v < -0.000001 then return -1.0 end
    return 0.0
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.0)
    dt = safeNumber(dt, 0.0)
    if tau <= 0.0 then return target end
    local alpha = clamp(dt / math.max(tau + dt, 0.000001), 0.0, 1.0)
    return current + (target - current) * alpha
end

local function safeField(obj, field, defaultValue)
    if not obj then return defaultValue end
    local ok, value = pcall(function() return obj[field] end)
    if not ok or value == nil then return defaultValue end
    return value
end

local function safeLoadRaw(key)
    if not ac or not ac.load then return nil end
    local ok, value = pcall(function() return ac.load(key) end)
    if ok then return value end
    return nil
end

local function safeLoad(key, defaultValue)
    local value = safeLoadRaw(key)
    if value == nil then return defaultValue or 0.0 end
    return safeNumber(value, defaultValue or 0.0)
end

local function loadFirst(defaultValue, ...)
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

local function safeStoreString(key, value)
    if not ac or not ac.store then return end
    pcall(function() ac.store(key, tostring(value or "")) end)
end

local function safeGetCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function() return ac.getCar(0) end)
    if ok then return car end
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

local function trim(value)
    local s = tostring(value or "")
    s = s:gsub("^%s+", "")
    s = s:gsub("%s+$", "")
    return s
end

local function stripComment(value)
    local s = tostring(value or "")
    local p1 = s:find(";", 1, true)
    local p2 = s:find("#", 1, true)
    local p = nil
    if p1 and p2 then p = math.min(p1, p2) else p = p1 or p2 end
    if p then s = s:sub(1, p - 1) end
    return trim(s)
end

local function readTextFile(path)
    if not io or not io.open or not path or path == "" then return nil end
    local ok, content = pcall(function()
        local f = io.open(path, "r")
        if not f then return nil end
        local data = f:read("*a")
        f:close()
        return data
    end)
    if ok then return content end
    return nil
end

local function parseIni(text)
    local ini = {}
    local section = ""
    if not text then return ini end
    for line in tostring(text):gmatch("[^\r\n]+") do
        local clean = stripComment(line)
        if clean ~= "" then
            local sec = clean:match("^%[([^%]]+)%]$")
            if sec then
                section = trim(sec):upper()
                ini[section] = ini[section] or {}
            else
                local key, value = clean:match("^([^=]+)=(.*)$")
                if key and value and section ~= "" then
                    ini[section] = ini[section] or {}
                    ini[section][trim(key):upper()] = trim(value)
                end
            end
        end
    end
    return ini
end

local function iniString(ini, section, key, defaultValue)
    section = tostring(section or ""):upper()
    key = tostring(key or ""):upper()
    if ini and ini[section] and ini[section][key] ~= nil then
        return tostring(ini[section][key])
    end
    return defaultValue
end

local function iniNumber(ini, section, key, defaultValue)
    local value = iniString(ini, section, key, nil)
    local n = toNumber(value)
    if n == nil then return defaultValue end
    return n
end

local function safeCarConfig(section, key, defaultValue)
    if not ac or not ac.getCarConfig then return defaultValue end
    local ok, value = pcall(function()
        return ac.getCarConfig(0, section, key, defaultValue)
    end)
    if ok and value ~= nil then return value end
    return defaultValue
end

local function safeCarConfigNumber(section, key, defaultValue)
    local value = safeCarConfig(section, key, nil)
    local n = toNumber(value)
    if n == nil then return defaultValue end
    return n
end

local function safeGetFolder(folderId)
    if not ac or not ac.getFolder or folderId == nil then return nil end
    local ok, value = pcall(function() return ac.getFolder(folderId) end)
    if ok and value ~= nil then return tostring(value) end
    return nil
end

local function joinPath(a, b)
    if not a or a == "" then return tostring(b or "") end
    local s = tostring(a):gsub("\\", "/")
    if s:sub(-1) == "/" then return s .. tostring(b or "") end
    return s .. "/" .. tostring(b or "")
end

local function getCarId(car)
    local id = nil
    if ac and ac.getCarID then
        local ok, value = pcall(function() return ac.getCarID(0) end)
        if ok and value ~= nil then id = tostring(value) end
    end
    if not id or id == "" then id = tostring("" or "") end
    if not id or id == "" then id = tostring("" or "") end
    if not id or id == "" then id = tostring("" or "") end
    id = tostring(id or ""):gsub("\\", "/")
    local last = id:match("([^/]+)$")
    return last or id
end

local function buildCarPathCandidates(carId, relativePath)
    local candidates = {}
    carId = tostring(carId or "")
    relativePath = tostring(relativePath or "")
    if carId == "" then return candidates end

    if ac and ac.FolderID then
        local contentCars = safeGetFolder(ac.FolderID.ContentCars)
        if contentCars then
            candidates[#candidates + 1] = joinPath(joinPath(contentCars, carId), relativePath)
        end

        local root = safeGetFolder(ac.FolderID.Root or ac.FolderID.AC or ac.FolderID.AssettoCorsa)
        if root then
            candidates[#candidates + 1] = joinPath(joinPath(joinPath(joinPath(root, "content"), "cars"), carId), relativePath)
        end
    end

    candidates[#candidates + 1] = joinPath(joinPath("content/cars", carId), relativePath)
    candidates[#candidates + 1] = joinPath(joinPath("../content/cars", carId), relativePath)
    candidates[#candidates + 1] = joinPath(joinPath("../../content/cars", carId), relativePath)
    candidates[#candidates + 1] = joinPath(joinPath("../../../content/cars", carId), relativePath)

    return candidates
end

local function readFirstExisting(candidates)
    for i = 1, #candidates do
        local text = readTextFile(candidates[i])
        if text and text ~= "" then return text, candidates[i] end
    end
    return nil, ""
end

local function parsePowerLut(fileText)
    local points = {}
    if not fileText then return points end
    for line in tostring(fileText):gmatch("[^\r\n]+") do
        local clean = stripComment(line)
        clean = clean:gsub("|", " "):gsub(",", " ")
        local rpm, torque = clean:match("([%-%d%.]+)%s+([%-%d%.]+)")
        rpm = tonumber(rpm)
        torque = tonumber(torque)
        if rpm and torque then points[#points + 1] = { rpm = rpm, torque = torque } end
    end
    table.sort(points, function(a, b) return (a.rpm or 0.0) < (b.rpm or 0.0) end)
    return points
end

local function interpolatePowerLut(points, rpm)
    if not points or #points == 0 then return nil end
    rpm = safeNumber(rpm, 0.0)
    if rpm <= points[1].rpm then return points[1].torque end
    local last = points[#points]
    if rpm >= last.rpm then return last.torque end
    for i = 1, #points - 1 do
        local a = points[i]
        local b = points[i + 1]
        if rpm >= a.rpm and rpm <= b.rpm then
            local t = clamp((rpm - a.rpm) / math.max(b.rpm - a.rpm, 1.0), 0.0, 1.0)
            return a.torque + (b.torque - a.torque) * t
        end
    end
    return last.torque
end

local function getGearRatioFromDefaults(gear)
    gear = math.floor(safeNumber(gear, 0))
    if gear == 1 then return M.params.defaultGearRatio1 end
    if gear == 2 then return M.params.defaultGearRatio2 end
    if gear == 3 then return M.params.defaultGearRatio3 end
    if gear == 4 then return M.params.defaultGearRatio4 end
    if gear == 5 then return M.params.defaultGearRatio5 end
    if gear == 6 then return M.params.defaultGearRatio6 end
    if gear < 0 then return -M.params.defaultGearRatio1 end
    return 0.0
end

local function getCurrentGearRatio(gear)
    gear = math.floor(safeNumber(gear, 0))
    if gear == 0 then return 0.0 end
    local value = state.gearRatios[gear]
    if value ~= nil and abs(value) > M.params.minGearRatioAbs then
        return safeNumber(value, 0.0)
    end
    return getGearRatioFromDefaults(gear)
end

local function loadDrivetrainConfig(car, dt)
    state.configReadTimer = (state.configReadTimer or 999.0) + (dt or 0.0)
    local carId = getCarId(car)

    if state.configLoaded and state.carId == carId and state.configReadTimer < M.params.configReadInterval then
        return
    end

    state.configReadTimer = 0.0

    if carId == "" then
        state.configSource = "FALLBACK"
        state.configStatus = "NO CAR ID"
        state.configWarning = "CAR ID UNAVAILABLE"
        state.configLoaded = true
        return
    end

    state.carId = carId
    state.gearRatios = {}
    state.powerLut = {}
    state.powerLutLoaded = false
    state.finalRatio = M.params.defaultFinalRatio
    state.peakTorqueNm = M.params.fallbackPeakTorqueNm
    state.redlineRpm = M.params.fallbackRedlineRpm
    state.drivetrainType = "UNKNOWN"
    state.configSource = "FALLBACK"
    state.configStatus = "DEFAULTS"
    state.configWarning = "USING FALLBACK DRIVETRAIN"
    state.configPath = ""
    state.gearCount = 0

    local cfgFinal = safeCarConfigNumber("GEARS", "FINAL", nil)
    if cfgFinal ~= nil and cfgFinal > 0.0 then
        state.finalRatio = cfgFinal
        state.configSource = "AC_CONFIG"
        state.configStatus = "READ VIA ac.getCarConfig"
        state.configWarning = "NONE"
    end

    local cfgCount = safeCarConfigNumber("GEARS", "COUNT", nil)
    if cfgCount ~= nil then state.gearCount = math.floor(cfgCount) end

    for g = 1, 10 do
        local gr = safeCarConfigNumber("GEARS", "GEAR_" .. g, nil)
        if gr ~= nil and abs(gr) > M.params.minGearRatioAbs then
            state.gearRatios[g] = gr
            if state.configSource == "FALLBACK" then
                state.configSource = "AC_CONFIG"
                state.configStatus = "READ VIA ac.getCarConfig"
                state.configWarning = "NONE"
            end
        end
    end

    local rev = safeCarConfigNumber("GEARS", "GEAR_R", nil)
    if rev ~= nil and abs(rev) > M.params.minGearRatioAbs then
        state.gearRatios[-1] = rev
    end

    local dtype = safeCarConfig("DRIVETRAIN", "TYPE", nil)
    if dtype ~= nil then state.drivetrainType = tostring(dtype) end

    local dtText, dtPath = readFirstExisting(buildCarPathCandidates(carId, "data/drivetrain.ini"))
    if dtText then
        local ini = parseIni(dtText)
        state.configSource = "INI"
        state.configStatus = "READ drivetrain.ini"
        state.configWarning = "NONE"
        state.configPath = dtPath

        state.finalRatio = iniNumber(ini, "GEARS", "FINAL", state.finalRatio)
        state.gearCount = math.floor(iniNumber(ini, "GEARS", "COUNT", state.gearCount))

        for g = 1, 10 do
            local gr = iniNumber(ini, "GEARS", "GEAR_" .. g, state.gearRatios[g])
            if gr ~= nil and abs(gr) > M.params.minGearRatioAbs then state.gearRatios[g] = gr end
        end

        local gr = iniNumber(ini, "GEARS", "GEAR_R", state.gearRatios[-1])
        if gr ~= nil and abs(gr) > M.params.minGearRatioAbs then state.gearRatios[-1] = gr end

        state.drivetrainType = iniString(ini, "DRIVETRAIN", "TYPE", state.drivetrainType)
    end

    local engineText = readFirstExisting(buildCarPathCandidates(carId, "data/engine.ini"))
    local powerLutName = "power.lut"

    if engineText then
        local ini = parseIni(engineText)
        state.redlineRpm = iniNumber(ini, "ENGINE_DATA", "LIMITER", iniNumber(ini, "HEADER", "LIMITER", state.redlineRpm))
        powerLutName = iniString(ini, "HEADER", "POWER_CURVE", powerLutName)
        local maxTorque = iniNumber(ini, "ENGINE_DATA", "MAX_TORQUE", nil)
        if maxTorque ~= nil and maxTorque > 0.0 then state.peakTorqueNm = maxTorque end
    end

    if M.params.usePowerLut then
        local lutText = readFirstExisting(buildCarPathCandidates(carId, "data/" .. tostring(powerLutName or "power.lut")))
        if not lutText then lutText = readFirstExisting(buildCarPathCandidates(carId, "data/power.lut")) end
        if lutText then
            state.powerLut = parsePowerLut(lutText)
            if #state.powerLut > 0 then
                state.powerLutLoaded = true
                local peak = 0.0
                for i = 1, #state.powerLut do
                    peak = math.max(peak, safeNumber(state.powerLut[i].torque, 0.0))
                end
                if peak > 0.0 then state.peakTorqueNm = peak end
            end
        end
    end

    if not state.gearRatios[1] then
        for g = 1, 6 do state.gearRatios[g] = getGearRatioFromDefaults(g) end
        state.gearRatios[-1] = -M.params.defaultGearRatio1
        if state.configSource == "FALLBACK" then
            state.configStatus = "FALLBACK RATIOS"
            state.configWarning = "GEAR RATIOS NOT READ"
        end
    end

    state.finalRatio = abs(safeNumber(state.finalRatio, M.params.defaultFinalRatio))
    if state.finalRatio <= 0.0 then state.finalRatio = M.params.defaultFinalRatio end
    state.drivelineEfficiency = M.params.drivelineEfficiency
    state.configLoaded = true
end

local function readInputs(car)
    return {
        rpm = safeNumber(safeField(car, "rpm", 0.0), 0.0),
        gas = clamp(safeNumber(safeField(car, "gas", 0.0), 0.0), 0.0, 1.0),
        brake = clamp(safeNumber(safeField(car, "brake", 0.0), 0.0), 0.0, 1.0),
        clutch = clamp(safeNumber(safeField(car, "clutch", 1.0), 1.0), 0.0, 1.0),
        gear = math.floor(safeNumber(safeField(car, "gear", 0), 0)),
    }
end

local function readBodyRigidity(dt)
    state.bodyReadTimer = (state.bodyReadTimer or 999.0) + (dt or 0.0)
    if state.bodyLinked and state.bodyReadTimer < M.params.bodyReadInterval then return end
    state.bodyReadTimer = 0.0

    state.bodyRigidity = clamp(safeLoad("ngp_body_rigidity", 1.0), 0.20, 1.35)
    state.bodyFlexFactor = clamp(safeLoad("ngp_body_flex_factor", 1.0), 0.25, 1.80)
    state.bodyTorsionFactor = clamp(safeLoad("ngp_body_torsion_factor", 1.0), 0.35, 2.00)
    state.bodyDampingFactor = clamp(safeLoad("ngp_body_damping_factor", 1.0), 0.50, 1.80)
    state.bodyLinked = safeLoad("ngp_body_rigidity_update_count", 0.0) > 0.0
    state.linkedBody = state.bodyLinked
end

local function updateBodyScales()
    if not state.bodyLinked then
        state.bodyLashScale = 1.0
        state.bodyShaftScale = 1.0
        return
    end

    local p = M.params
    local softness = clamp((state.bodyFlexFactor or 1.0) - 1.0, 0.0, 0.80)
    local torsionSoft = clamp((state.bodyTorsionFactor or 1.0) - 1.0, 0.0, 1.0)
    local dampingExtra = clamp((state.bodyDampingFactor or 1.0) - 1.0, 0.0, 0.80)
    local stiff = clamp((state.bodyRigidity or 1.0) - 1.0, 0.0, 0.35)

    state.bodyLashScale = clamp(
        1.0 + softness * p.bodyFlexLashGain + torsionSoft * 0.08 - stiff * 0.05,
        p.minBodyLashScale,
        p.maxBodyLashScale
    )

    state.bodyShaftScale = clamp(
        1.0 + torsionSoft * p.bodyTorsionShaftGain + dampingExtra * p.bodyDampingShaftGain - stiff * p.bodyStiffShaftLoss,
        p.minBodyShaftScale,
        p.maxBodyShaftScale
    )
end

local function readLoad(index)
    local value, key = loadFirst(0.0,
        "ngp_wheel_load_" .. index,
        "ngp_contact_load_" .. index,
        "ngp_tire_state_load_" .. index,
        "ngp_sprung_load_" .. index
    )
    if key ~= nil then return abs(value), true end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        return abs(M.params.loadRef + safeNumber(dlt, 0.0)), true
    end

    return M.params.loadRef, false
end

local function readContactLoss(index)
    local loss, key = loadFirst(0.0,
        "ngp_contact_loss_" .. index,
        "ngp_tire_contact_loss_" .. index,
        "ngp_tcr_contact_loss_" .. index
    )
    if key ~= nil then return clamp(loss, 0.0, 1.0), true end

    local q, qKey = loadFirst(1.0,
        "ngp_contact_quality_" .. index,
        "ngp_contact_trust_" .. index,
        "ngp_tcr_quality_" .. index,
        "ngp_tc_contact_" .. index,
        "ngp_tire_contact_" .. index
    )
    if qKey ~= nil then
        return clamp(1.0 - clamp(q, 0.0, 1.0), 0.0, 1.0), true
    end

    return 0.0, false
end

local function readRootChain()
    state.linkedLSD = false
    state.linkedContact = false
    state.linkedArm = false
    state.linkedHop = false
    state.linkedLoad = false

    state.bodySteer = safeLoad("ngp_body_steer", 0.0)
    state.flexEnergy = safeLoad("ngp_chassis_flex_energy", safeLoad("ngp_flex_energy", 0.0))
    state.flexRelease = safeLoad("ngp_chassis_flex_release", safeLoad("ngp_flex_release", 0.0))

    local armRaw = safeLoadRaw("ngp_control_arm_total")
    if armRaw == nil then armRaw = safeLoadRaw("ngp_arm_total") end
    state.linkedArm = armRaw ~= nil
    state.controlArmTotal = safeNumber(armRaw, 0.0)

    local lsdRaw = safeLoadRaw("ngp_lsd_lock")
    if lsdRaw == nil then lsdRaw = safeLoadRaw("ngp_diff_lock") end
    state.linkedLSD = lsdRaw ~= nil
    state.lsdLock = clamp(safeNumber(lsdRaw, 0.0), 0.0, 1.0)
    state.lsdDiff = safeLoad("ngp_lsd_diff", safeLoad("ngp_diff_diff", 0.0))

    state.loadRL, state.linkedLoad = readLoad(2)
    local loadRROk = false
    state.loadRR, loadRROk = readLoad(3)
    state.linkedLoad = state.linkedLoad or loadRROk
    state.loadDiff = abs(state.loadRL - state.loadRR)
    state.loadLoss = clamp(state.loadDiff / math.max(M.params.loadDiffReference, 1.0), 0.0, 1.0)

    local loss2, link2 = readContactLoss(2)
    local loss3, link3 = readContactLoss(3)
    state.linkedContact = link2 or link3
    state.contactLoss = clamp((loss2 + loss3) * 0.5, 0.0, 1.0)

    local q2 = clamp(1.0 - loss2, 0.0, 1.0)
    local q3 = clamp(1.0 - loss3, 0.0, 1.0)
    state.contactQualityRear = (q2 + q3) * 0.5

    local t2 = safeLoad("ngp_contact_trust_2", q2)
    local t3 = safeLoad("ngp_contact_trust_3", q3)
    state.contactTrustRear = clamp((t2 + t3) * 0.5, 0.0, 1.0)

    local impulse2 = safeLoad("ngp_susp_road_impulse_2", safeLoad("ngp_sci_impulse_2", safeLoad("ngp_damper_road_memory_2", 0.0)))
    local impulse3 = safeLoad("ngp_susp_road_impulse_3", safeLoad("ngp_sci_impulse_3", safeLoad("ngp_damper_road_memory_3", 0.0)))
    state.contactShock = clamp((abs(impulse2) + abs(impulse3)) * 0.5, 0.0, 1.0)
end

local function estimateFallbackTorqueCurve(rpm)
    rpm = safeNumber(rpm, 0.0)
    local redline = math.max(state.redlineRpm or M.params.fallbackRedlineRpm, 1000.0)
    local x = clamp(rpm / redline, 0.0, 1.25)
    local low = 0.42 + 0.58 * clamp(x / 0.55, 0.0, 1.0)
    local highDrop = 1.0 - clamp((x - 0.72) / 0.38, 0.0, 1.0) * 0.38
    return (state.peakTorqueNm or M.params.fallbackPeakTorqueNm) * clamp(low * highDrop, 0.30, 1.05)
end

local function calculateEngineTorqueNm(gas, rpm, gear)
    gas = clamp(gas, 0.0, 1.0)
    rpm = safeNumber(rpm, 0.0)

    local baseTorque = nil
    if state.powerLutLoaded then baseTorque = interpolatePowerLut(state.powerLut, rpm) end
    if baseTorque == nil then baseTorque = estimateFallbackTorqueCurve(rpm) end
    baseTorque = clamp(baseTorque, 0.0, M.params.maxEngineTorqueNm)

    local rpmRatio = clamp(rpm / math.max(state.redlineRpm or M.params.fallbackRedlineRpm, 1000.0), 0.0, 1.2)
    local throttleTorque = baseTorque * gas
    local engineBrake = 0.0

    if abs(gear) > 0.0 then
        engineBrake = M.params.engineBrakeTorqueNm * (0.35 + 0.65 * rpmRatio) * (1.0 - gas)
    end

    state.engineBrakeTorqueNm = engineBrake
    return clamp(throttleTorque - engineBrake, -M.params.maxEngineTorqueNm, M.params.maxEngineTorqueNm)
end

local function updateTorqueNmPath(dt)
    local gearRatio = getCurrentGearRatio(state.gear)
    state.gearRatio = gearRatio
    state.finalRatio = abs(safeNumber(state.finalRatio, M.params.defaultFinalRatio))
    if state.finalRatio <= 0.0 then state.finalRatio = M.params.defaultFinalRatio end

    if state.gear == 0 or abs(gearRatio) <= M.params.minGearRatioAbs then
        state.engineTorqueNm = lowPass(state.engineTorqueNm, 0.0, M.params.torqueNmTau, dt)
        state.clutchTorqueNm = lowPass(state.clutchTorqueNm, 0.0, M.params.torqueNmTau, dt)
        state.gearboxOutputTorqueNm = lowPass(state.gearboxOutputTorqueNm, 0.0, M.params.torqueNmTau, dt)
        state.diffInputTorqueNm = lowPass(state.diffInputTorqueNm, 0.0, M.params.torqueNmTau, dt)
        state.wheelDriveTorqueNm = lowPass(state.wheelDriveTorqueNm, 0.0, M.params.torqueNmTau, dt)
        state.normalizedTorqueFromNm = 0.0
        return
    end

    local engineNm = calculateEngineTorqueNm(state.gas, state.filteredRPM, state.gear)
    local clutchFactor = clamp(state.clutch or 0.0, 0.0, 1.0)
    local clutchCapacity = math.max(M.params.clutchTorqueCapacityNm, 1.0)
    local clutchSlipLoss = clamp(math.max(abs(engineNm) - clutchCapacity, 0.0) / clutchCapacity * M.params.clutchSlipLossGain, 0.0, 0.45)

    local clutchNm = engineNm * clutchFactor * (1.0 - clutchSlipLoss)
    local gearboxNm = clutchNm * gearRatio
    local diffNm = gearboxNm * state.finalRatio * (state.drivelineEfficiency or M.params.drivelineEfficiency) * (state.finalTorqueScale or 1.0)
    diffNm = clamp(diffNm, -M.params.maxDiffInputTorqueNm, M.params.maxDiffInputTorqueNm)

    local wheelNm = clamp(diffNm * 0.5, -M.params.maxWheelTorqueNm, M.params.maxWheelTorqueNm)

    state.engineTorqueNm = lowPass(state.engineTorqueNm, engineNm, M.params.torqueNmTau, dt)
    state.clutchTorqueNm = lowPass(state.clutchTorqueNm, clutchNm, M.params.torqueNmTau, dt)
    state.gearboxOutputTorqueNm = lowPass(state.gearboxOutputTorqueNm, gearboxNm, M.params.torqueNmTau, dt)
    state.diffInputTorqueNm = lowPass(state.diffInputTorqueNm, diffNm, M.params.torqueNmTau, dt)
    state.wheelDriveTorqueNm = lowPass(state.wheelDriveTorqueNm, wheelNm, M.params.torqueNmTau, dt)
    state.normalizedTorqueFromNm = clamp(state.diffInputTorqueNm / math.max(M.params.normalizedTorqueReferenceNm, 1.0), -1.0, 1.0)
end

local function calculateEngineTorqueNorm(gas)
    local rpmRatio = clamp(state.filteredRPM / math.max(state.redlineRpm or 9000.0, 1000.0), 0.0, 1.0)
    local rpmFactor = math.max(0.35, 1.0 - rpmRatio)
    return gas * rpmFactor
end

local function calculateTransmittedTorqueNorm(engineTorque, gear)
    local gearLoad = math.max(abs(gear), 1.0)
    local transmitted = engineTorque / (1.0 + gearLoad * M.params.gearInertia)
    return transmitted * clamp(state.clutch, 0.0, 1.0)
end

local function updateShiftShock(rpm, gear, dt)
    local gearChanged = gear ~= state.prevGear
    local rpmDiff = abs(rpm - state.filteredRPM)
    if gearChanged then
        local add = clamp(rpmDiff * M.params.shiftShockScale + M.params.shiftShockGain, 0.0, 1.0)
        state.shiftShock = clamp(math.max(state.shiftShock, add), 0.0, 1.0)
    else
        state.shiftShock = math.max(0.0, state.shiftShock - dt * M.params.shiftShockDecay)
    end
    state.prevGear = gear
end

local function updateLash(transmittedTorque, dt)
    local lashThreshold = M.params.lash * (state.bodyLashScale or 1.0)
    local lashTarget = 0.0
    if abs(transmittedTorque) > lashThreshold then lashTarget = transmittedTorque end
    state.lashState = lowPass(state.lashState, lashTarget, M.params.lashTau * (state.bodyLashScale or 1.0), dt)
end

local function updateShaft(dt)
    local shaftScale = state.bodyShaftScale or 1.0
    local shaftForce = state.lashState * M.params.shaftStiffness / math.max(shaftScale, 0.001)
    local dampingForce = state.shaftVelocity * M.params.shaftDamping * shaftScale

    state.shaftVelocity = state.shaftVelocity + (shaftForce - dampingForce) * dt
    state.shaftVelocity = clamp(state.shaftVelocity, -M.params.maxShaftVelocity, M.params.maxShaftVelocity)

    state.shaftAngle = state.shaftAngle + state.shaftVelocity * dt
    state.shaftAngle = clamp(state.shaftAngle, -M.params.maxShaftAngle, M.params.maxShaftAngle)
end

local function getHopShock()
    local hopRL, k0 = loadFirst(0.0, "ngp_tire_hop_energy_2", "ngp_tirehop_2", "ngp_tire_hop_2", "ngp_drive_hop_shock_2")
    local hopRR, k1 = loadFirst(0.0, "ngp_tire_hop_energy_3", "ngp_tirehop_3", "ngp_tire_hop_3", "ngp_drive_hop_shock_3")
    state.linkedHop = k0 ~= nil or k1 ~= nil
    if abs(hopRL) <= 1.5 and abs(hopRR) <= 1.5 then
        return clamp((abs(hopRL) + abs(hopRR)) * 0.5, 0.0, 1.0)
    end
    return clamp((abs(hopRL) + abs(hopRR)) / math.max(M.params.hopShockReference, 1.0), 0.0, 1.0)
end

local function updateDriveTorque()
    local p = M.params

    state.hopShock = getHopShock()
    state.lsdResistance = state.lsdLock * p.lsdResistanceGain
    state.armTwistLoss = clamp(abs(state.controlArmTotal) * p.controlArmTwistGain, 0.0, 0.20)

    local flexLoss = clamp(
        state.flexEnergy * p.chassisFlexTorqueLossGain - state.flexRelease * p.chassisReleaseKickGain,
        -0.06,
        0.16
    )

    local contactLoss = state.contactLoss * p.contactTorqueLossGain
    local loadLoss = state.loadLoss * p.loadDiffTorqueLossGain
    local shockLoss = state.hopShock * p.hopTorqueLossGain + state.contactShock * p.contactImpulseShockGain

    state.finalTorqueScale = clamp(
        1.0 - state.lsdResistance - state.armTwistLoss - flexLoss - contactLoss - loadLoss - shockLoss,
        0.55,
        1.08
    )

    state.driveTorque = clamp(state.lashState * state.finalTorqueScale, -1.0, 1.0)
end

local function updateStress(engineTorque, transmittedTorque, clutchInput)
    local clutchSlip = abs(clutchInput - state.clutch)
    state.shiftStress =
        abs(engineTorque)
        + abs(transmittedTorque)
        + clutchSlip * 5.0
        + (state.filteredRPM / 10000.0) * 3.0
        + abs(state.shaftVelocity) * 0.20
        + state.shiftShock * 0.50
end

local function updateGearboxHeat(dt)
    local gearboxLoad = abs(state.lashState) + abs(state.shiftShock) + abs(state.shaftVelocity) + abs(state.lsdDiff) * 0.05
    state.gearboxHeat = state.gearboxHeat + gearboxLoad * M.params.gearboxHeatGain * dt
    state.gearboxHeat = state.gearboxHeat - (state.gearboxHeat - M.params.gearboxAmbient) * M.params.gearboxCoolRate
    state.gearboxHeat = math.max(M.params.gearboxAmbient, state.gearboxHeat)
end

local function decayState(dt)
    state.filteredRPM = lowPass(state.filteredRPM, 0.0, 0.20, dt)
    state.engineTorque = lowPass(state.engineTorque, 0.0, 0.15, dt)
    state.transmittedTorque = lowPass(state.transmittedTorque, 0.0, 0.15, dt)
    state.driveTorque = lowPass(state.driveTorque, 0.0, 0.15, dt)
    state.engineTorqueNm = lowPass(state.engineTorqueNm, 0.0, 0.15, dt)
    state.clutchTorqueNm = lowPass(state.clutchTorqueNm, 0.0, 0.15, dt)
    state.gearboxOutputTorqueNm = lowPass(state.gearboxOutputTorqueNm, 0.0, 0.15, dt)
    state.diffInputTorqueNm = lowPass(state.diffInputTorqueNm, 0.0, 0.15, dt)
    state.wheelDriveTorqueNm = lowPass(state.wheelDriveTorqueNm, 0.0, 0.15, dt)
    state.normalizedTorqueFromNm = lowPass(state.normalizedTorqueFromNm, 0.0, 0.15, dt)
    state.shaftAngle = lowPass(state.shaftAngle, 0.0, 0.20, dt)
    state.shaftVelocity = lowPass(state.shaftVelocity, 0.0, 0.20, dt)
    state.lashState = lowPass(state.lashState, 0.0, 0.15, dt)
    state.shiftShock = lowPass(state.shiftShock, 0.0, 0.12, dt)
    state.shiftStress = lowPass(state.shiftStress, 0.0, 0.20, dt)
    state.contactLoss = lowPass(state.contactLoss, 0.0, 0.20, dt)
    state.hopShock = lowPass(state.hopShock, 0.0, 0.20, dt)
end

local function exportState()
    safeStore("ngp_drivetrain_status", state.status or "UNKNOWN")
    safeStore("ngp_drivetrain_update_count", state.updateCount or 0)

    safeStore("ngp_drive_torque", state.driveTorque or 0.0)
    safeStore("ngp_drivetrain_torque", state.driveTorque or 0.0)
    safeStore("ngp_dt_torque", state.driveTorque or 0.0)

    safeStore("ngp_diff_input_torque_nm", state.diffInputTorqueNm or 0.0)
    safeStore("ngp_drivetrain_diff_input_torque_nm", state.diffInputTorqueNm or 0.0)
    safeStore("ngp_drive_torque_nm", state.diffInputTorqueNm or 0.0)

    safeStore("ngp_engine_torque_nm", state.engineTorqueNm or 0.0)
    safeStore("ngp_clutch_torque_nm", state.clutchTorqueNm or 0.0)
    safeStore("ngp_gearbox_output_torque_nm", state.gearboxOutputTorqueNm or 0.0)
    safeStore("ngp_wheel_drive_torque_nm", state.wheelDriveTorqueNm or 0.0)
    safeStore("ngp_drive_torque_normalized_from_nm", state.normalizedTorqueFromNm or 0.0)

    safeStore("ngp_drive_engine_torque", state.engineTorque or 0.0)
    safeStore("ngp_drive_transmitted_torque", state.transmittedTorque or 0.0)

    safeStore("ngp_drive_rpm_filtered", state.filteredRPM or 0.0)
    safeStore("ngp_drive_rpm_raw", state.rawRPM or 0.0)

    safeStore("ngp_shaft_twist", state.shaftAngle or 0.0)
    safeStore("ngp_shaft_velocity", state.shaftVelocity or 0.0)
    safeStore("ngp_drive_lash", state.lashState or 0.0)
    safeStore("ngp_drive_lash_state", state.lashState or 0.0)

    safeStore("ngp_drive_lsd_lock", state.lsdLock or 0.0)
    safeStore("ngp_lsd_resistance", state.lsdResistance or 0.0)

    safeStore("ngp_shift_shock", state.shiftShock or 0.0)
    safeStore("ngp_shift_stress", state.shiftStress or 0.0)
    safeStore("ngp_gearbox_heat", state.gearboxHeat or 0.0)

    safeStore("ngp_drive_hop_shock", state.hopShock or 0.0)
    safeStore("ngp_drive_contact_loss", state.contactLoss or 0.0)
    safeStore("ngp_drive_contact_shock", state.contactShock or 0.0)
    safeStore("ngp_drive_load_loss", state.loadLoss or 0.0)
    safeStore("ngp_drive_arm_twist_loss", state.armTwistLoss or 0.0)
    safeStore("ngp_drive_final_torque_scale", state.finalTorqueScale or 1.0)

    safeStore("ngp_drive_gear", state.gear or 0)
    safeStore("ngp_drive_gear_ratio", state.gearRatio or 0.0)
    safeStore("ngp_drive_final_ratio", state.finalRatio or M.params.defaultFinalRatio)
    safeStore("ngp_drive_clutch", state.clutch or 0.0)
    safeStore("ngp_drive_clutch_input", state.clutchInput or 0.0)
    safeStore("ngp_drive_gas", state.gas or 0.0)
    safeStore("ngp_drive_brake", state.brake or 0.0)

    safeStore("ngp_drive_peak_torque_nm", state.peakTorqueNm or M.params.fallbackPeakTorqueNm)
    safeStore("ngp_drive_engine_brake_torque_nm", state.engineBrakeTorqueNm or 0.0)
    safeStore("ngp_drive_power_lut_loaded", state.powerLutLoaded and 1 or 0)

    safeStoreString("ngp_drive_config_source", state.configSource or "UNKNOWN")
    safeStoreString("ngp_drive_config_status", state.configStatus or "UNKNOWN")
    safeStoreString("ngp_drive_config_warning", state.configWarning or "NONE")
    safeStoreString("ngp_drive_config_path", state.configPath or "")
    safeStoreString("ngp_drive_car_id", state.carId or "")
    safeStoreString("ngp_drive_type", state.drivetrainType or "UNKNOWN")

    safeStore("ngp_drive_body_linked", state.bodyLinked and 1 or 0)
    safeStore("ngp_drive_body_lash_scale", state.bodyLashScale or 1.0)
    safeStore("ngp_drive_body_shaft_scale", state.bodyShaftScale or 1.0)

    if not state.debugStoreNow then return end

    safeStore("ngp_drive_link_lsd", state.linkedLSD and 1 or 0)
    safeStore("ngp_drive_link_contact", state.linkedContact and 1 or 0)
    safeStore("ngp_drive_link_body", state.linkedBody and 1 or 0)
    safeStore("ngp_drive_link_arm", state.linkedArm and 1 or 0)
    safeStore("ngp_drive_link_hop", state.linkedHop and 1 or 0)
    safeStore("ngp_drive_link_load", state.linkedLoad and 1 or 0)

    safeStore("ngp_drive_load_rl", state.loadRL or 0.0)
    safeStore("ngp_drive_load_rr", state.loadRR or 0.0)
    safeStore("ngp_drive_load_diff", state.loadDiff or 0.0)
    safeStore("ngp_drive_contact_quality_rear", state.contactQualityRear or 1.0)
    safeStore("ngp_drive_contact_trust_rear", state.contactTrustRear or 1.0)
    safeStore("ngp_drive_flex_energy", state.flexEnergy or 0.0)
    safeStore("ngp_drive_flex_release", state.flexRelease or 0.0)
    safeStore("ngp_drive_control_arm_total", state.controlArmTotal or 0.0)
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
    dt = clamp(dt, M.params.minDt, M.params.maxDt)

    updateDebugGate(dt)

    car = car or safeGetCar()
    if not car then
        state.status = "NO CAR"
        decayState(dt)
        exportState()
        return
    end

    state.status = "RUNNING"

    local input = readInputs(car)
    state.rawRPM = input.rpm
    state.gas = input.gas
    state.brake = input.brake
    state.clutchInput = input.clutch
    state.gear = input.gear

    loadDrivetrainConfig(car, dt)

    readBodyRigidity(dt)
    updateBodyScales()
    readRootChain()

    state.filteredRPM = lowPass(state.filteredRPM, input.rpm, M.params.rpmTau, dt)
    state.clutch = clamp(lowPass(state.clutch, input.clutch, M.params.clutchTau, dt), 0.0, 1.0)

    updateTorqueNmPath(dt)

    local engineTorque = calculateEngineTorqueNorm(input.gas)
    local transmittedTorque = calculateTransmittedTorqueNorm(engineTorque, input.gear)

    state.engineTorque = engineTorque
    state.transmittedTorque = transmittedTorque

    updateShiftShock(input.rpm, input.gear, dt)
    updateLash(transmittedTorque, dt)
    updateShaft(dt)
    updateDriveTorque()
    updateStress(engineTorque, transmittedTorque, input.clutch)
    updateGearboxHeat(dt)

    exportState()
end

function M.getDriveTorque()
    return state.driveTorque or 0.0
end

function M.getDiffInputTorqueNm()
    return state.diffInputTorqueNm or 0.0
end

function M.getShaftTwist()
    return state.shaftAngle or 0.0
end

function M.getShaftVelocity()
    return state.shaftVelocity or 0.0
end

function M.getClutch()
    return state.clutch or 0.0
end

function M.getGearboxHeat()
    return state.gearboxHeat or 0.0
end

function M.getLash()
    return state.lashState or 0.0
end

function M.getFinalTorqueScale()
    return state.finalTorqueScale or 1.0
end

function M.getState()
    return state
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f / Config %s\n" ..
        "RPM %.0f -> %.0f / Gear %.0f GR %.3f Final %.3f\n" ..
        "Gas %.3f Brake %.3f / Clutch %.3f / Type %s\n" ..
        "Norm Engine %.3f / Trans %.3f / Drive %.3f Scale %.2f\n" ..
        "Nm Engine %.0f / Clutch %.0f / DiffTin %.0f / Wheel %.0f\n" ..
        "Shaft %.3f Vel %.3f / Lash %.3f\n" ..
        "Shift %.3f Stress %.3f Heat %.1f\n" ..
        "LSD %.3f Hop %.3f Contact %.3f Arm %.3f Body:%s",
        tostring(state.status),
        state.updateCount or 0,
        tostring(state.configSource or "NIL"),
        state.rawRPM or 0.0,
        state.filteredRPM or 0.0,
        state.gear or 0,
        state.gearRatio or 0.0,
        state.finalRatio or 0.0,
        state.gas or 0.0,
        state.brake or 0.0,
        state.clutch or 0.0,
        tostring(state.drivetrainType or "UNKNOWN"),
        state.engineTorque or 0.0,
        state.transmittedTorque or 0.0,
        state.driveTorque or 0.0,
        state.finalTorqueScale or 1.0,
        state.engineTorqueNm or 0.0,
        state.clutchTorqueNm or 0.0,
        state.diffInputTorqueNm or 0.0,
        state.wheelDriveTorqueNm or 0.0,
        state.shaftAngle or 0.0,
        state.shaftVelocity or 0.0,
        state.lashState or 0.0,
        state.shiftShock or 0.0,
        state.shiftStress or 0.0,
        state.gearboxHeat or 0.0,
        state.lsdLock or 0.0,
        state.hopShock or 0.0,
        state.contactLoss or 0.0,
        state.controlArmTotal or 0.0,
        state.bodyLinked and "YES" or "NO"
    )
end

exportState()

return M
