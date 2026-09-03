---@diagnostic disable: undefined-global

--============================================================
-- caster_effect.lua
-- ACNextGen V1.1.5 stable
-- Caster / KPI / trail / scrub geometry signal model
--============================================================

local M = {}

local FL, FR, RL, RR = 0, 1, 2, 3

M.params = {
    casterAngleDeg = 7.5,
    kpiAngleDeg = 12.0,
    maxSteerAngleDeg = 35.0,

    autoGeometryEnabled = true,
    iniRelativePath = "data/suspensions.ini",
    geometryReloadEachUpdate = false,

    fallbackCasterAngleDeg = 7.5,
    fallbackKpiAngleDeg = 12.0,
    casterSign = -1.0,

    minCasterAngleDeg = -3.0,
    maxCasterAngleDeg = 18.0,
    minKpiAngleDeg = 0.0,
    maxKpiAngleDeg = 28.0,
    minKingpinVerticalDistance = 0.020,

    mechanicalTrail = 0.060,
    pneumaticTrailBase = 0.045,
    scrubRadius = 0.035,

    trailLoadReference = 3200.0,
    wheelLoadReference = 3000.0,

    casterCamberGain = 1.0,
    kpiCamberGain = 0.85,
    casterToeGain = 0.35,
    scrubContactShiftGain = 1.0,

    aligningTorqueGain = 0.0018,
    trailTorqueGain = 1.0,

    ultraChassisInfluence = 0.25,
    bodyFlexInfluence = 0.12,

    optimalCamberDeg = -0.50,
    camberGripSensitivity = 0.035,
    toeGripSensitivity = 0.090,
    trailGripSensitivity = 0.030,
    slipGripSensitivity = 0.35,
    contactGripInfluence = 0.10,

    minGrip = 0.68,
    maxGrip = 1.08,

    slipCollapseAngleRad = 0.28,
    slipCollapseStrength = 0.55,
    minPneumaticTrail = 0.005,
    maxPneumaticTrail = 0.075,

    tauGeometry = 0.055,
    tauGrip = 0.070,
    tauTorque = 0.050,

    minDt = 0.00005,
    maxDt = 0.030,

    estimateCasterFromCamber = true,
    estimateMinSteerDeg = 8.0,
    estimateMaxSteerDeg = 28.0,
    estimateMinSpeedKmh = 1.0,
    estimateMaxSpeedKmh = 80.0,
    estimateMinCamberDeltaDeg = 0.05,
    estimateTau = 1.25,
    estimatedCasterMinDeg = 1.0,
    estimatedCasterMaxDeg = 14.0,
    useEstimatedCasterWhenIniMissing = true,

    outputEnabled = true,
    debugStoreInterval = 0.25,
}

local state = {
    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    geometryLoaded = false,
    geometryAttempted = false,
    geometryStatus = "NOT LOADED",
    geometryPath = "",
    geometrySource = "NONE",
    geometryWarning = "NONE",

    suspensionTypeFront = "UNKNOWN",
    suspensionTypeRear = "UNKNOWN",

    geometryCasterFrontDeg = 0.0,
    geometryKpiFrontDeg = 0.0,
    geometryCasterRearDeg = 0.0,
    geometryKpiRearDeg = 0.0,
    geometryTieToeFront = 0.0,
    geometryTieToeRear = 0.0,

    geometryCasterRawDeg = 0.0,
    geometryKpiRawDeg = 0.0,
    geometryAxisX = 0.0,
    geometryAxisY = 0.0,
    geometryAxisZ = 0.0,

    estimatedCasterDeg = 0.0,
    estimatedCasterValid = false,
    estimatedCasterSamples = 0,
    lastCasterEstimateDeg = 0.0,

    steerNorm = 0.0,
    steerDeg = 0.0,
    steerRad = 0.0,

    casterDeg = 7.5,
    casterRad = 0.0,
    kpiDeg = 12.0,
    kpiRad = 0.0,

    frontAxisAnchor = 0.0,
    frontAxisAuthority = 0.0,
    frontAxisSteerWeight = 0.0,

    bodySteer = 0.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyRigidity = 1.0,

    load = { [0] = 3000.0, [1] = 3000.0, [2] = 3000.0, [3] = 3000.0 },
    slipAngle = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    fy = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    contact = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    ucCamber = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    ucToe = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    casterCamberDeg = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    kpiCamberDeg = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    totalCamberDeg = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    casterToeDeg = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    totalToeDeg = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    pneumaticTrail = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    mechanicalTrail = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    contactShift = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    trailTorque = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    aligningHint = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    grip = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    targetGrip = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    geometryLinked = false,
    tireForceLinked = false,
    frontAxisLinked = false,
    bodyLinked = false,
    contactLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
    _dt = 0.001,
}

M.state = state
M.debug = state

local function safeNumber(value, defaultValue)
    local n = tonumber(value)
    if n == nil or n ~= n then
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

local function sign(v)
    v = safeNumber(v, 0.0)
    if v < 0.0 then return -1.0 end
    if v > 0.0 then return 1.0 end
    return 0.0
end

local function atan2(y, x)
    if math.atan2 then
        return math.atan2(y, x)
    end
    return math.atan(y, x)
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

local function safeStore(key, value)
    if not M.params.outputEnabled then return end
    if not ac or not ac.store then return end
    pcall(function() ac.store(key, value) end)
end

local function safeField(obj, field, defaultValue)
    if not obj then return defaultValue end
    local ok, value = pcall(function() return obj[field] end)
    if not ok or value == nil then return defaultValue end
    return value
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = math.max(safeNumber(tau, 0.001), 0.0001)
    dt = safeNumber(dt, 0.0)
    local alpha = clamp(dt / (tau + dt), 0.0, 1.0)
    return current + (target - current) * alpha
end

local function updateDebugGate(dt)
    state.debugStoreTimer = (state.debugStoreTimer or 0.0) + safeNumber(dt, 0.0)
    if state.debugStoreTimer >= M.params.debugStoreInterval then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    else
        state.debugStoreNow = false
    end
end

local function trimString(value)
    return tostring(value or ""):gsub("^%s+", ""):gsub("%s+$", "")
end

local function parseVec3(value)
    if value == nil then return nil end
    local x, y, z = tostring(value):match("([%-%+%.%d]+)%s*,%s*([%-%+%.%d]+)%s*,%s*([%-%+%.%d]+)")
    if not x then return nil end
    return { x = safeNumber(x, 0.0), y = safeNumber(y, 0.0), z = safeNumber(z, 0.0) }
end

local function subVec3(a, b)
    if not a or not b then return nil end
    return {
        x = safeNumber(a.x, 0.0) - safeNumber(b.x, 0.0),
        y = safeNumber(a.y, 0.0) - safeNumber(b.y, 0.0),
        z = safeNumber(a.z, 0.0) - safeNumber(b.z, 0.0),
    }
end

local function readTextIni(path)
    if not io or not io.open then return nil end

    local ok, file = pcall(function()
        return io.open(path, "r")
    end)

    if not ok or not file then
        return nil
    end

    local ini = {}
    local section = nil

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

    file:close()
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

local function buildSuspensionIniPath(car)
    local carId = getCarId(car)
    if not carId then return nil end

    if ac and ac.getFolder and ac.FolderID and ac.FolderID.ContentCars then
        local ok, root = pcall(function()
            return ac.getFolder(ac.FolderID.ContentCars)
        end)

        if ok and root then
            return tostring(root) .. "/" .. carId .. "/" .. M.params.iniRelativePath
        end
    end

    return "content/cars/" .. carId .. "/" .. M.params.iniRelativePath
end

local function detectSuspensionType(section)
    if not section then return "UNKNOWN" end
    if section.STRUT_CAR ~= nil and section.WBTYRE_BOTTOM ~= nil then
        return "STRUT"
    end
    if section.WBTYRE_TOP ~= nil and section.WBTYRE_BOTTOM ~= nil then
        return "DOUBLE_WISHBONE"
    end
    return "UNKNOWN"
end

local function getKingpinPoints(section, suspensionType)
    if not section then return nil, nil end

    if suspensionType == "STRUT" then
        return parseVec3(section.STRUT_CAR), parseVec3(section.WBTYRE_BOTTOM)
    end

    if suspensionType == "DOUBLE_WISHBONE" then
        return parseVec3(section.WBTYRE_TOP), parseVec3(section.WBTYRE_BOTTOM)
    end

    return nil, nil
end

local function calculateCasterFromKingpin(topPoint, bottomPoint)
    local axis = subVec3(topPoint, bottomPoint)
    if not axis then
        state.geometryWarning = "NO KINGPIN AXIS"
        return M.params.fallbackCasterAngleDeg
    end

    local vertical = math.abs(safeNumber(axis.y, 0.0))
    local longitudinal = safeNumber(axis.z, 0.0)

    state.geometryAxisX = safeNumber(axis.x, 0.0)
    state.geometryAxisY = safeNumber(axis.y, 0.0)
    state.geometryAxisZ = safeNumber(axis.z, 0.0)

    if vertical < M.params.minKingpinVerticalDistance then
        state.geometryWarning = "KINGPIN VERTICAL TOO SMALL"
        return M.params.fallbackCasterAngleDeg
    end

    local rawCaster = M.params.casterSign * math.deg(atan2(longitudinal, vertical))
    state.geometryCasterRawDeg = rawCaster

    local caster = clamp(rawCaster, M.params.minCasterAngleDeg, M.params.maxCasterAngleDeg)
    if math.abs(caster - rawCaster) > 0.001 then
        state.geometryWarning = "CASTER CLAMPED"
    end

    return caster
end

local function calculateKpiFromKingpin(topPoint, bottomPoint)
    local axis = subVec3(topPoint, bottomPoint)
    if not axis then
        state.geometryWarning = "NO KINGPIN AXIS"
        return M.params.fallbackKpiAngleDeg
    end

    local vertical = math.abs(safeNumber(axis.y, 0.0))
    local lateral = math.abs(safeNumber(axis.x, 0.0))

    if vertical < M.params.minKingpinVerticalDistance then
        state.geometryWarning = "KINGPIN VERTICAL TOO SMALL"
        return M.params.fallbackKpiAngleDeg
    end

    local rawKpi = math.deg(atan2(lateral, vertical))
    state.geometryKpiRawDeg = rawKpi

    local kpi = clamp(rawKpi, M.params.minKpiAngleDeg, M.params.maxKpiAngleDeg)
    if math.abs(kpi - rawKpi) > 0.001 then
        state.geometryWarning = "KPI CLAMPED"
    end

    return kpi
end

local function calculateTieToeHint(section)
    if not section then return 0.0 end

    local steerTyre = parseVec3(section.WBTYRE_STEER)
    local bottomTyre = parseVec3(section.WBTYRE_BOTTOM)

    if not steerTyre or not bottomTyre then
        return 0.0
    end

    local arm = subVec3(steerTyre, bottomTyre)
    if not arm then return 0.0 end

    local len = math.sqrt(arm.x * arm.x + arm.z * arm.z)
    if len < 0.0001 then return 0.0 end

    return clamp(arm.z / len, -1.0, 1.0)
end

local function calculateSectionGeometry(section)
    local suspensionType = detectSuspensionType(section)
    local topPoint, bottomPoint = getKingpinPoints(section, suspensionType)

    if not topPoint or not bottomPoint then
        return {
            suspensionType = suspensionType,
            casterDeg = M.params.fallbackCasterAngleDeg,
            kpiDeg = M.params.fallbackKpiAngleDeg,
            tieToe = 0.0,
            fallback = true,
        }
    end

    return {
        suspensionType = suspensionType,
        casterDeg = calculateCasterFromKingpin(topPoint, bottomPoint),
        kpiDeg = calculateKpiFromKingpin(topPoint, bottomPoint),
        tieToe = calculateTieToeHint(section),
        fallback = false,
    }
end

local function applyGeometryFallback(reason)
    state.geometryLoaded = true
    state.geometryStatus = "FALLBACK"
    state.geometrySource = "FALLBACK"
    state.geometryWarning = tostring(reason or "UNKNOWN")

    state.suspensionTypeFront = "FALLBACK"
    state.suspensionTypeRear = "FALLBACK"

    state.geometryCasterFrontDeg = M.params.fallbackCasterAngleDeg
    state.geometryKpiFrontDeg = M.params.fallbackKpiAngleDeg
    state.geometryCasterRearDeg = 0.0
    state.geometryKpiRearDeg = 0.0
    state.geometryTieToeFront = 0.0
    state.geometryTieToeRear = 0.0

    M.params.casterAngleDeg = state.geometryCasterFrontDeg
    M.params.kpiAngleDeg = state.geometryKpiFrontDeg
end

local function loadSuspensionGeometry(car)
    if not M.params.autoGeometryEnabled then
        applyGeometryFallback("AUTO DISABLED")
        return false
    end

    state.geometryAttempted = true
    state.geometryStatus = "LOADING"
    state.geometryWarning = "NONE"

    local path = buildSuspensionIniPath(car)
    state.geometryPath = tostring(path or "")

    if not path then
        applyGeometryFallback("NO CAR PATH")
        return false
    end

    local ini = readTextIni(path)
    if not ini then
        applyGeometryFallback("INI NOT FOUND")
        return false
    end

    local frontSection = ini.FRONT or ini.FRONT_0 or ini.SUSPENSION_FRONT
    local rearSection = ini.REAR or ini.REAR_0 or ini.SUSPENSION_REAR

    if not frontSection then
        applyGeometryFallback("NO FRONT SECTION")
        return false
    end

    local front = calculateSectionGeometry(frontSection)
    local rear = rearSection and calculateSectionGeometry(rearSection) or {
        suspensionType = "UNKNOWN",
        casterDeg = 0.0,
        kpiDeg = 0.0,
        tieToe = 0.0,
        fallback = true,
    }

    state.suspensionTypeFront = front.suspensionType
    state.suspensionTypeRear = rear.suspensionType
    state.geometryCasterFrontDeg = front.casterDeg
    state.geometryKpiFrontDeg = front.kpiDeg
    state.geometryCasterRearDeg = rear.casterDeg
    state.geometryKpiRearDeg = rear.kpiDeg
    state.geometryTieToeFront = front.tieToe
    state.geometryTieToeRear = rear.tieToe

    M.params.casterAngleDeg = state.geometryCasterFrontDeg
    M.params.kpiAngleDeg = state.geometryKpiFrontDeg

    state.geometryLoaded = true
    state.geometryStatus = front.fallback and "FALLBACK" or "OK"
    state.geometrySource = front.fallback and "FALLBACK" or "INI"
    if state.geometryWarning == "" then state.geometryWarning = "NONE" end

    return state.geometrySource == "INI"
end

local function readSpeedKmh(car)
    if not car then return 0.0 end

    local speed = safeField(car, "speedKmh", nil)
    if speed ~= nil then
        return safeNumber(speed, 0.0)
    end

    local velocity = safeField(car, "velocity", nil)
    if velocity and velocity.length then
        local ok, result = pcall(function()
            return velocity:length() * 3.6
        end)
        if ok then return safeNumber(result, 0.0) end
    end

    return 0.0
end

local function readSteer(car)
    local steer = safeNumber(safeField(car, "steer", 0.0), 0.0)

    local bodySteer = safeLoadRaw("ngp_body_steer")
    if bodySteer ~= nil then
        state.bodySteer = safeNumber(bodySteer, 0.0)
    else
        state.bodySteer = 0.0
    end

    steer = clamp(steer + state.bodySteer * M.params.bodyFlexInfluence, -1.0, 1.0)

    state.steerNorm = steer
    state.steerDeg = steer * M.params.maxSteerAngleDeg
    state.steerRad = math.rad(state.steerDeg)
end

local function readBody()
    local rigidity = safeLoadRaw("ngp_body_rigidity")
    local flex = safeLoadRaw("ngp_body_flex_factor")
    local torsion = safeLoadRaw("ngp_body_torsion_factor")

    state.bodyLinked = rigidity ~= nil or flex ~= nil or torsion ~= nil

    state.bodyRigidity = clamp(safeNumber(rigidity, 1.0), 0.20, 1.35)
    state.bodyFlexFactor = clamp(safeNumber(flex, 1.0), 0.25, 1.80)
    state.bodyTorsionFactor = clamp(safeNumber(torsion, 1.0), 0.25, 1.80)
end

local function readFrontAxis()
    local anchor = safeLoadRaw("ngp_front_axis_anchor")
    local authority = safeLoadRaw("ngp_front_axis_authority")
    local steerWeight = safeLoadRaw("ngp_front_axis_steer_weight")

    state.frontAxisLinked = anchor ~= nil or authority ~= nil or steerWeight ~= nil

    state.frontAxisAnchor = clamp(safeNumber(anchor, 0.0), 0.0, 1.25)
    state.frontAxisAuthority = clamp(safeNumber(authority, 0.0), 0.0, 1.25)
    state.frontAxisSteerWeight = clamp(safeNumber(steerWeight, 0.0), 0.0, 1.20)
end

local function readWheelLoad(index, wheel)
    local value =
        safeLoadRaw("ngp_tire_state_load_" .. index) or
        safeLoadRaw("ngp_tire_load_" .. index) or
        safeLoadRaw("ngp_wheel_load_" .. index) or
        safeLoadRaw("ngp_sprung_load_" .. index)

    if value == nil then
        local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
        if dlt ~= nil then
            value = M.params.wheelLoadReference + safeNumber(dlt, 0.0)
        end
    end

    if value == nil and wheel then
        value = safeField(wheel, "load", M.params.wheelLoadReference)
    end

    state.load[index] = clamp(safeNumber(value, M.params.wheelLoadReference), 0.0, 12000.0)
end

local function readSlipAngle(index, wheel)
    local value =
        safeLoadRaw("ngp_tire_slip_angle_" .. index) or
        safeLoadRaw("ngp_slip_angle_" .. index) or
        safeLoadRaw("ngp_filtered_slip_angle_" .. index)

    if value == nil and wheel then
        value = safeField(wheel, "slipAngle", 0.0)
    end

    state.slipAngle[index] = clamp(safeNumber(value, 0.0), -0.80, 0.80)
end

local function readLateralForce(index)
    local value =
        safeLoadRaw("ngp_rf2_fy_" .. index) or
        safeLoadRaw("ngp_tire_force_lat_" .. index) or
        safeLoadRaw("ngp_tf_lat_" .. index)

    state.tireForceLinked = state.tireForceLinked or value ~= nil

    if value == nil then
        value = -state.slipAngle[index] * state.load[index] * 2.2
    end

    state.fy[index] = clamp(safeNumber(value, 0.0), -8000.0, 8000.0)
end

local function readContact(index)
    local value =
        safeLoadRaw("ngp_contact_quality_" .. index) or
        safeLoadRaw("ngp_tc_contact_" .. index) or
        safeLoadRaw("ngp_tire_contact_" .. index)

    state.contactLinked = state.contactLinked or value ~= nil
    state.contact[index] = clamp(safeNumber(value, 1.0), 0.0, 1.2)
end

local function readGeometryContext(index)
    local camber =
        safeLoadRaw("ngp_uc_camber_" .. index) or
        safeLoadRaw("ngp_arm_camber_" .. index) or
        safeLoadRaw("ngp_control_arm_camber_" .. index)

    local toe =
        safeLoadRaw("ngp_uc_toe_" .. index) or
        safeLoadRaw("ngp_arm_toe_" .. index) or
        safeLoadRaw("ngp_control_arm_toe_" .. index)

    state.geometryLinked = state.geometryLinked or camber ~= nil or toe ~= nil

    state.ucCamber[index] = clamp(safeNumber(camber, 0.0), -0.35, 0.35)
    state.ucToe[index] = clamp(safeNumber(toe, 0.0), -0.25, 0.25)
end

local function readWheelInputs(car)
    for i = 0, 3 do
        local wheel = nil
        if car and car.wheels then
            wheel = car.wheels[i]
        end

        readWheelLoad(i, wheel)
        readSlipAngle(i, wheel)
        readLateralForce(i)
        readContact(i)
        readGeometryContext(i)
    end
end

local function estimateCasterFromCamber(car)
    local p = M.params

    if not p.estimateCasterFromCamber then return end
    if state.geometrySource == "INI" then return end

    local speedKmh = readSpeedKmh(car)
    if speedKmh < p.estimateMinSpeedKmh or speedKmh > p.estimateMaxSpeedKmh then
        return
    end

    local steerDeg = math.abs(state.steerDeg or 0.0)
    if steerDeg < p.estimateMinSteerDeg or steerDeg > p.estimateMaxSteerDeg then
        return
    end

    local sinSteer = math.sin(math.abs(state.steerRad or 0.0))
    if math.abs(sinSteer) < 0.001 then return end

    local camberFLDeg = safeNumber(state.ucCamber[FL], 0.0) * 180.0 / math.pi
    local camberFRDeg = safeNumber(state.ucCamber[FR], 0.0) * 180.0 / math.pi
    local camberDeltaDeg = math.abs(camberFLDeg - camberFRDeg)

    if camberDeltaDeg < p.estimateMinCamberDeltaDeg then
        return
    end

    local estimateDeg = clamp(
        camberDeltaDeg / (2.0 * sinSteer),
        p.estimatedCasterMinDeg,
        p.estimatedCasterMaxDeg
    )

    state.lastCasterEstimateDeg = estimateDeg

    if not state.estimatedCasterValid then
        state.estimatedCasterDeg = estimateDeg
        state.estimatedCasterValid = true
    else
        state.estimatedCasterDeg = lowPass(state.estimatedCasterDeg, estimateDeg, p.estimateTau, state._dt)
    end

    state.estimatedCasterSamples = (state.estimatedCasterSamples or 0) + 1

    if p.useEstimatedCasterWhenIniMissing then
        state.geometrySource = "ACD_ESTIMATE"
        state.geometryStatus = "ESTIMATED"
        state.geometryWarning = "CASTER ESTIMATED FROM CAMBER"
        M.params.casterAngleDeg = state.estimatedCasterDeg
    end
end

local function wheelSideSign(index)
    if index == FR or index == RR then
        return 1.0
    end
    return -1.0
end

local function isFront(index)
    return index == FL or index == FR
end

local function calculateWheelGeometry(index)
    local p = M.params
    local side = wheelSideSign(index)

    if not isFront(index) then
        state.casterCamberDeg[index] = lowPass(state.casterCamberDeg[index], 0.0, p.tauGeometry, state._dt)
        state.kpiCamberDeg[index] = lowPass(state.kpiCamberDeg[index], 0.0, p.tauGeometry, state._dt)
        state.casterToeDeg[index] = lowPass(state.casterToeDeg[index], 0.0, p.tauGeometry, state._dt)
        state.totalCamberDeg[index] = lowPass(state.totalCamberDeg[index], state.ucCamber[index] * 180.0 / math.pi, p.tauGeometry, state._dt)
        state.totalToeDeg[index] = lowPass(state.totalToeDeg[index], state.ucToe[index] * 180.0 / math.pi, p.tauGeometry, state._dt)
        state.contactShift[index] = lowPass(state.contactShift[index], 0.0, p.tauGeometry, state._dt)
        return
    end

    local steerRad = state.steerRad or 0.0

    local casterTerm = math.sin(state.casterRad) * math.sin(steerRad)
    local kpiTerm = (math.sin(state.kpiRad) * math.cos(steerRad) - math.sin(state.kpiRad)) * -side

    local casterCamberRad = math.asin(clamp(casterTerm, -1.0, 1.0)) * p.casterCamberGain
    local kpiCamberRad = math.asin(clamp(kpiTerm, -1.0, 1.0)) * p.kpiCamberGain

    local casterToeRad =
        steerRad *
        (math.sin(state.casterRad) * p.casterToeGain + state.geometryTieToeFront * 0.08) *
        (0.65 + 0.35 * state.frontAxisAuthority)

    local ucCamberRad = state.ucCamber[index] * p.ultraChassisInfluence
    local ucToeRad = state.ucToe[index] * p.ultraChassisInfluence

    local totalCamberRad = casterCamberRad + kpiCamberRad + ucCamberRad
    local totalToeRad = casterToeRad + ucToeRad

    state.casterCamberDeg[index] = lowPass(state.casterCamberDeg[index], casterCamberRad * 180.0 / math.pi, p.tauGeometry, state._dt)
    state.kpiCamberDeg[index] = lowPass(state.kpiCamberDeg[index], kpiCamberRad * 180.0 / math.pi, p.tauGeometry, state._dt)
    state.totalCamberDeg[index] = lowPass(state.totalCamberDeg[index], totalCamberRad * 180.0 / math.pi, p.tauGeometry, state._dt)
    state.casterToeDeg[index] = lowPass(state.casterToeDeg[index], casterToeRad * 180.0 / math.pi, p.tauGeometry, state._dt)
    state.totalToeDeg[index] = lowPass(state.totalToeDeg[index], totalToeRad * 180.0 / math.pi, p.tauGeometry, state._dt)

    local contactShift = math.sin(steerRad) * p.scrubRadius * side * p.scrubContactShiftGain
    state.contactShift[index] = lowPass(state.contactShift[index], contactShift, p.tauGeometry, state._dt)
end

local function calculateTrailAndTorque(index)
    local p = M.params

    if not isFront(index) then
        state.mechanicalTrail[index] = lowPass(state.mechanicalTrail[index], 0.0, p.tauTorque, state._dt)
        state.pneumaticTrail[index] = lowPass(state.pneumaticTrail[index], 0.0, p.tauTorque, state._dt)
        state.trailTorque[index] = lowPass(state.trailTorque[index], 0.0, p.tauTorque, state._dt)
        state.aligningHint[index] = lowPass(state.aligningHint[index], 0.0, p.tauTorque, state._dt)
        return
    end

    local loadScale = clamp(state.load[index] / math.max(p.trailLoadReference, 1.0), 0.0, 1.35)
    local slip = math.abs(state.slipAngle[index] or 0.0)
    local slipCollapse = clamp(slip / math.max(p.slipCollapseAngleRad, 0.001), 0.0, 1.0)

    local trailLive =
        p.pneumaticTrailBase *
        loadScale *
        (1.0 - slipCollapse * p.slipCollapseStrength) *
        (0.70 + 0.30 * state.frontAxisAnchor)

    trailLive = clamp(trailLive, p.minPneumaticTrail, p.maxPneumaticTrail)

    local mechanicalTrail = p.mechanicalTrail * (0.85 + 0.15 * state.frontAxisAnchor)
    local torque = (state.fy[index] or 0.0) * (mechanicalTrail + trailLive) * p.trailTorqueGain
    local aligning = clamp(torque * p.aligningTorqueGain, -1.50, 1.50)

    state.mechanicalTrail[index] = lowPass(state.mechanicalTrail[index], mechanicalTrail, p.tauTorque, state._dt)
    state.pneumaticTrail[index] = lowPass(state.pneumaticTrail[index], trailLive, p.tauTorque, state._dt)
    state.trailTorque[index] = lowPass(state.trailTorque[index], torque, p.tauTorque, state._dt)
    state.aligningHint[index] = lowPass(state.aligningHint[index], aligning, p.tauTorque, state._dt)
end

local function calculateGrip(index)
    local p = M.params

    if not isFront(index) then
        state.targetGrip[index] = 1.0
        state.grip[index] = lowPass(state.grip[index], 1.0, p.tauGrip, state._dt)
        return
    end

    local camberError = math.abs((state.totalCamberDeg[index] or 0.0) - p.optimalCamberDeg)
    local toeAbs = math.abs(state.totalToeDeg[index] or 0.0)
    local slipAbs = math.abs(state.slipAngle[index] or 0.0)
    local trail = state.pneumaticTrail[index] or p.pneumaticTrailBase
    local trailLoss = math.abs(trail - p.pneumaticTrailBase) * p.trailGripSensitivity
    local contact = clamp(state.contact[index] or 1.0, 0.0, 1.2)

    local grip =
        1.0 -
        camberError * p.camberGripSensitivity -
        toeAbs * p.toeGripSensitivity -
        slipAbs * p.slipGripSensitivity -
        trailLoss

    grip = grip * (0.88 + 0.12 * state.frontAxisAnchor)
    grip = grip * (1.0 - (state.bodyFlexFactor - 1.0) * 0.03)
    grip = grip * (1.0 - (1.0 - clamp(contact, 0.0, 1.0)) * p.contactGripInfluence)

    state.targetGrip[index] = clamp(grip, p.minGrip, p.maxGrip)
    state.grip[index] = lowPass(state.grip[index], state.targetGrip[index], p.tauGrip, state._dt)
end

local function calculateAll()
    if state.geometrySource == "INI" then
        state.casterDeg = safeNumber(state.geometryCasterFrontDeg, M.params.casterAngleDeg)
        state.kpiDeg = safeNumber(state.geometryKpiFrontDeg, M.params.kpiAngleDeg)
    elseif state.estimatedCasterValid and M.params.useEstimatedCasterWhenIniMissing then
        state.casterDeg = safeNumber(state.estimatedCasterDeg, M.params.casterAngleDeg)
        state.kpiDeg = M.params.kpiAngleDeg
        state.geometrySource = "ACD_ESTIMATE"
    else
        state.casterDeg = M.params.casterAngleDeg
        state.kpiDeg = M.params.kpiAngleDeg
        if state.geometrySource ~= "INI" and state.geometrySource ~= "ACD_ESTIMATE" then
            state.geometrySource = "FALLBACK"
        end
    end

    state.casterRad = math.rad(state.casterDeg)
    state.kpiRad = math.rad(state.kpiDeg)

    for i = 0, 3 do
        calculateWheelGeometry(i)
        calculateTrailAndTorque(i)
        calculateGrip(i)
    end
end

local function exportWheel(index)
    safeStore("ngp_caster_grip_" .. index, state.grip[index] or 1.0)
    safeStore("ngp_caster_camber_" .. index, state.totalCamberDeg[index] or 0.0)
    safeStore("ngp_caster_toe_" .. index, state.totalToeDeg[index] or 0.0)

    safeStore("ngp_caster_camber_gain_" .. index, state.casterCamberDeg[index] or 0.0)
    safeStore("ngp_caster_kpi_camber_" .. index, state.kpiCamberDeg[index] or 0.0)
    safeStore("ngp_caster_total_camber_" .. index, state.totalCamberDeg[index] or 0.0)
    safeStore("ngp_caster_total_toe_" .. index, state.totalToeDeg[index] or 0.0)

    safeStore("ngp_caster_pneumatic_trail_" .. index, state.pneumaticTrail[index] or 0.0)
    safeStore("ngp_caster_mechanical_trail_" .. index, state.mechanicalTrail[index] or 0.0)
    safeStore("ngp_caster_trail_torque_" .. index, state.trailTorque[index] or 0.0)
    safeStore("ngp_caster_aligning_hint_" .. index, state.aligningHint[index] or 0.0)
    safeStore("ngp_caster_contact_shift_" .. index, state.contactShift[index] or 0.0)

    if not state.debugStoreNow then return end

    safeStore("ngp_caster_fy_" .. index, state.fy[index] or 0.0)
    safeStore("ngp_caster_load_" .. index, state.load[index] or 0.0)
    safeStore("ngp_caster_slip_angle_" .. index, state.slipAngle[index] or 0.0)
    safeStore("ngp_caster_contact_" .. index, state.contact[index] or 1.0)
    safeStore("ngp_caster_target_grip_" .. index, state.targetGrip[index] or 1.0)
end

local function exportState()
    safeStore("ngp_caster_status", state.status or "UNKNOWN")
    safeStore("ngp_caster_update_count", state.updateCount or 0)
    safeStore("ngp_caster_wheels_valid", state.wheelsValid and 1 or 0)

    safeStore("ngp_caster_steer_deg", state.steerDeg or 0.0)
    safeStore("ngp_caster_angle_deg", state.casterDeg or 0.0)
    safeStore("ngp_caster_kpi_deg", state.kpiDeg or 0.0)

    safeStore("ngp_caster_geometry_source", state.geometrySource or "NONE")
    safeStore("ngp_caster_geometry_warning", state.geometryWarning or "NONE")
    safeStore("ngp_caster_geometry_status", state.geometryStatus or "UNKNOWN")
    safeStore("ngp_caster_geometry_path", state.geometryPath or "")

    safeStore("ngp_caster_suspension_type_front", state.suspensionTypeFront or "UNKNOWN")
    safeStore("ngp_caster_suspension_type_rear", state.suspensionTypeRear or "UNKNOWN")

    safeStore("ngp_caster_geometry_caster_front", state.geometryCasterFrontDeg or 0.0)
    safeStore("ngp_caster_geometry_kpi_front", state.geometryKpiFrontDeg or 0.0)
    safeStore("ngp_caster_geometry_caster_raw", state.geometryCasterRawDeg or 0.0)
    safeStore("ngp_caster_geometry_kpi_raw", state.geometryKpiRawDeg or 0.0)
    safeStore("ngp_caster_geometry_tie_toe_front", state.geometryTieToeFront or 0.0)

    safeStore("ngp_caster_geometry_axis_x", state.geometryAxisX or 0.0)
    safeStore("ngp_caster_geometry_axis_y", state.geometryAxisY or 0.0)
    safeStore("ngp_caster_geometry_axis_z", state.geometryAxisZ or 0.0)

    safeStore("ngp_caster_front_axis_anchor", state.frontAxisAnchor or 0.0)
    safeStore("ngp_caster_front_axis_authority", state.frontAxisAuthority or 0.0)
    safeStore("ngp_caster_front_axis_steer_weight", state.frontAxisSteerWeight or 0.0)

    safeStore("ngp_caster_geometry_linked", state.geometryLinked and 1 or 0)
    safeStore("ngp_caster_tire_force_linked", state.tireForceLinked and 1 or 0)
    safeStore("ngp_caster_front_axis_linked", state.frontAxisLinked and 1 or 0)
    safeStore("ngp_caster_body_linked", state.bodyLinked and 1 or 0)
    safeStore("ngp_caster_contact_linked", state.contactLinked and 1 or 0)

    if state.debugStoreNow then
        safeStore("ngp_caster_estimated_caster", state.estimatedCasterDeg or 0.0)
        safeStore("ngp_caster_estimated_samples", state.estimatedCasterSamples or 0)
    end

    for i = 0, 3 do
        exportWheel(i)
    end
end

function M.init()
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    dt = clamp(safeNumber(dt, 0.0), M.params.minDt, M.params.maxDt)
    state._dt = dt

    state.updateCount = (state.updateCount or 0) + 1

    updateDebugGate(dt)

    if not car and ac and ac.getCar then
        local ok, result = pcall(function() return ac.getCar(0) end)
        if ok then car = result end
    end

    if not car then
        state.status = "NO CAR"
        state.wheelsValid = false
        exportState()
        return
    end

    if not car.wheels then
        state.status = "NO WHEELS"
        state.wheelsValid = false
        exportState()
        return
    end

    if M.params.geometryReloadEachUpdate or (not state.geometryLoaded and not state.geometryAttempted) then
        loadSuspensionGeometry(car)
    end

    state.status = "RUNNING"
    state.wheelsValid = true

    state.geometryLinked = false
    state.tireForceLinked = false
    state.frontAxisLinked = false
    state.bodyLinked = false
    state.contactLinked = false

    readSteer(car)
    readBody()
    readFrontAxis()
    readWheelInputs(car)
    estimateCasterFromCamber(car)
    calculateAll()
    exportState()
end

function M.getGrip(index)
    return state.grip[index] or 1.0
end

function M.getCamber(index)
    return state.totalCamberDeg[index] or 0.0
end

function M.getToe(index)
    return state.totalToeDeg[index] or 0.0
end

function M.getAligningHint(index)
    return state.aligningHint[index] or 0.0
end

function M.getPneumaticTrail(index)
    return state.pneumaticTrail[index] or 0.0
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0

        return string.format(
            "Caster W%d Grip %.3f / Cam %.3f / Toe %.3f\nTrail P %.3f M %.3f / Torque %.1f\nAlign %.3f / Shift %.4f / Fy %.0f",
            i,
            state.grip[i] or 1.0,
            state.totalCamberDeg[i] or 0.0,
            state.totalToeDeg[i] or 0.0,
            state.pneumaticTrail[i] or 0.0,
            state.mechanicalTrail[i] or 0.0,
            state.trailTorque[i] or 0.0,
            state.aligningHint[i] or 0.0,
            state.contactShift[i] or 0.0,
            state.fy[i] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s\nSteer %.2f deg / Caster %.2f / KPI %.2f\nSusp %s / Geo %s / Source %s\nWarn %s\nFrontAxis A %.3f Auth %.3f\nLinks Geo:%s TireF:%s FrontAxis:%s Body:%s Contact:%s\nGrip %.3f %.3f %.3f %.3f",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",
        state.steerDeg or 0.0,
        state.casterDeg or 0.0,
        state.kpiDeg or 0.0,
        tostring(state.suspensionTypeFront or "UNKNOWN"),
        tostring(state.geometryStatus or "UNKNOWN"),
        tostring(state.geometrySource or "NONE"),
        tostring(state.geometryWarning or "NONE"),
        state.frontAxisAnchor or 0.0,
        state.frontAxisAuthority or 0.0,
        state.geometryLinked and "OK" or "NIL",
        state.tireForceLinked and "OK" or "NIL",
        state.frontAxisLinked and "OK" or "NIL",
        state.bodyLinked and "OK" or "NIL",
        state.contactLinked and "OK" or "NIL",
        state.grip[0] or 1.0,
        state.grip[1] or 1.0,
        state.grip[2] or 1.0,
        state.grip[3] or 1.0
    )
end

return M
