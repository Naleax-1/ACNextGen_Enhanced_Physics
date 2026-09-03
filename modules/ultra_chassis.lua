---@diagnostic disable: undefined-global

--============================================================
-- ACNextGen
-- ultra_chassis.lua
-- V1.1.5 Stable
-- rFactor2 UltraChassis-style Micro Geometry Kinematics
--
-- Policy:
--   - Do not directly rewrite AC physics.
--   - Convert steer, roll, suspension travel, road/load inputs into
--     dynamic camber, dynamic toe, camber mu scale and slip angle offset.
--   - Export root signals for tire_state, tire_thermal_brush and tire_force.
--============================================================

local M = {}

--============================================================
-- Parameters
--============================================================

M.params = {
    dCamber_dRoll = -0.450,
    dCamber_dSteer = -0.015,

    dToe_dRoll = 0.085,
    dToe_dSteer = 0.120,

    staticCamberFront = -1.20 * math.pi / 180.0,
    staticCamberRear  = -0.80 * math.pi / 180.0,

    staticToeFront = 0.00 * math.pi / 180.0,
    staticToeRear  = 0.05 * math.pi / 180.0,

    maxSteerDeg = 35.0,
    ackermannScale = 0.00035,
    steerCamberScale = 0.12,

    travelReference = 0.080,
    travelToeScale = 0.75,
    travelCamberScale = 0.35,

    rollInputScale = 1.0,

    loadPathCamberGain = 0.018,
    loadPathToeGain = 0.010,
    roadInputToeGain = 0.012,
    tireForceToeGain = 0.000015,
    contactLossToeGain = 0.012,
    contactLossCamberGain = 0.010,

    maxCamberRad = 0.25,
    maxToeRad = 0.15,
    maxAlphaOffsetRad = 0.20,

    minMuCamber = 0.82,
    maxMuCamber = 1.03,

    camberTau = 0.070,
    toeTau = 0.060,
    rollTau = 0.080,

    minDt = 0.00005,
    maxDt = 0.030,

    debugStoreInterval = 0.25,
}

--============================================================
-- State
--============================================================

local WHEEL_NAMES = {
    [0] = "FL",
    [1] = "FR",
    [2] = "RL",
    [3] = "RR",
}

M.state = {
    camber = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    toe    = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    targetCamber = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    targetToe    = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    muCamber = { [0]=1.0, [1]=1.0, [2]=1.0, [3]=1.0 },
    alphaOffset = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    travel = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    travelNorm = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    loadPath = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    roadInput = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    contactLoss = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    tireForceLat = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    steerInput = 0.0,
    steerDeg = 0.0,
    roll = 0.0,
    rollFiltered = 0.0,

    avgCamberAbs = 0.0,
    avgToeAbs = 0.0,
    avgMuCamber = 1.0,
    maxAlphaOffset = 0.0,
    activeCount = 0,

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    suspensionLinked = false,
    rollLinked = false,
    steerLinked = false,
    loadPathLinked = false,
    roadLinked = false,
    contactLinked = false,
    tireForceLinked = false,

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
    if v < minValue then
        return minValue
    end
    if v > maxValue then
        return maxValue
    end
    return v
end

local function sign(v)
    v = safeNumber(v, 0.0)
    if v < 0.0 then
        return -1.0
    end
    if v > 0.0 then
        return 1.0
    end
    return 0.0
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

local function safeLoad(key, fallback)
    local value = safeLoadRaw(key)
    if value == nil then
        return fallback or 0.0
    end
    return safeNumber(value, fallback or 0.0)
end

local function safeStore(key, value)
    if not ac or not ac.store then
        return
    end

    pcall(function()
        ac.store(key, value)
    end)
end

local function safeField(obj, field, fallback)
    if not obj then
        return fallback
    end

    local ok, value = pcall(function()
        return obj[field]
    end)

    if not ok or value == nil then
        return fallback
    end

    return value
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.0)
    dt = safeNumber(dt, 0.0)

    if tau <= 0.0 then
        return target
    end

    local k = dt / (tau + dt)
    k = clamp(k, 0.0, 1.0)

    return current + (target - current) * k
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
        return wheels[index] or wheels[index + 1]
    end)

    if not ok then
        return nil
    end

    return wheel
end

--============================================================
-- Input readers
--============================================================

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

local function readSteer(car)
    local steer = nil
    local source = nil

    if car then
        local raw = safeField(car, "steer", nil)
        if raw ~= nil then
            steer = safeNumber(raw, 0.0)
            source = "CAR"
        end
    end

    if steer == nil then
        steer, source = loadFirst(0.0,
            "ngp_steer_body",
            "ngp_steer_input",
            "ngp_steering_input",
            "ngp_raw_steer",
            "ngp_driver_steer",
            "ngp_observer_steer"
        )
    end

    steer = clamp(steer, -1.0, 1.0)

    M.state.steerLinked = source ~= nil and true or false
    M.state.steerInput = steer
    M.state.steerDeg = steer * M.params.maxSteerDeg

    return steer, M.state.steerDeg
end

local function readTravelFromWheel(wheel)
    if not wheel then
        return nil
    end

    local fields = {
        "suspensionTravel",
        "suspensionPosition",
        "damperTravel",
        "travel",
        "suspensionCompression",
    }

    for i = 1, #fields do
        local value = safeField(wheel, fields[i], nil)
        if value ~= nil then
            return safeNumber(value, 0.0)
        end
    end

    return nil
end

local function readTravel(index, wheel)
    local v, source = loadFirst(nil,
        "ngp_susp_travel_" .. index,
        "ngp_suspension_travel_" .. index,
        "ngp_damper_travel_" .. index,
        "ngp_damper_deflection_" .. index,
        "ngp_spring_travel_" .. index,
        "ngp_road_body_travel_" .. index
    )

    if source == nil then
        v = readTravelFromWheel(wheel)
        if v ~= nil then
            source = "WHEEL"
        end
    end

    if v == nil then
        v = 0.0
    end

    if source ~= nil then
        M.state.suspensionLinked = true
    end

    local travel = clamp(v, -0.25, 0.25)

    M.state.travel[index] = travel
    M.state.travelNorm[index] = clamp(
        travel / math.max(M.params.travelReference, 0.001),
        -2.0,
        2.0
    )

    return travel
end

local function readRoll(car)
    local roll, source = loadFirst(nil,
        "ngp_roll_cg",
        "ngp_sprung_roll_cg",
        "ngp_body_roll_input",
        "ngp_body_roll",
        "ngp_chassis_roll",
        "ngp_chassis_roll_energy",
        "ngp_load_transfer_roll",
        "ngp_dlt_roll"
    )

    if source == nil then
        local rf = safeLoadRaw("ngp_roll_front")
        local rr = safeLoadRaw("ngp_roll_rear")
        if rf ~= nil or rr ~= nil then
            roll = (safeNumber(rf, 0.0) + safeNumber(rr, 0.0)) * 0.5
            source = "ROLL_AXLE"
        end
    end

    if source ~= nil then
        M.state.rollLinked = true
        return clamp(safeNumber(roll, 0.0), -0.35, 0.35)
    end

    local av = safeField(car, "localAngularVelocity", nil)
    if av and av.z ~= nil then
        return clamp(safeNumber(av.z, 0.0) * 0.035, -0.25, 0.25)
    end

    return 0.0
end

local function readRootWheelInputs(index)
    local loadPath, loadSource = loadFirst(0.0,
        "ngp_load_path_vertical_flow_" .. index,
        "ngp_lp_vertical_flow_" .. index,
        "ngp_load_path_work_" .. index,
        "ngp_lp_work_" .. index
    )

    if loadSource ~= nil then
        M.state.loadPathLinked = true
    end

    M.state.loadPath[index] = clamp(loadPath, -2.0, 2.0)

    local roadInput, roadSource = loadFirst(0.0,
        "ngp_tire_force_road_input_" .. index,
        "ngp_tf_road_input_" .. index,
        "ngp_rii_road_input_" .. index,
        "ngp_road_input_" .. index,
        "ngp_rbi_wheel_input_" .. index
    )

    if roadSource ~= nil then
        M.state.roadLinked = true
    end

    M.state.roadInput[index] = clamp(roadInput, 0.0, 2.0)

    local contactLoss, contactSource = loadFirst(0.0,
        "ngp_tire_contact_loss_" .. index,
        "ngp_tcr_contact_loss_" .. index,
        "ngp_contact_loss_" .. index,
        "ngp_contact_hop_drop_" .. index
    )

    if contactSource ~= nil then
        M.state.contactLinked = true
    end

    M.state.contactLoss[index] = clamp(contactLoss, 0.0, 1.0)

    local latForce, forceSource = loadFirst(0.0,
        "ngp_tire_force_lat_" .. index,
        "ngp_tire_force_lateral_" .. index,
        "ngp_tf_lat_" .. index,
        "ngp_rf2_fy_" .. index
    )

    if forceSource ~= nil then
        M.state.tireForceLinked = true
    end

    M.state.tireForceLat[index] = clamp(latForce, -6000.0, 6000.0)
end

--============================================================
-- Core geometry model
--============================================================

local function calculateWheel(index, steerDeg, roll)
    local p = M.params

    local isFront = index < 2
    local isRight = index == 1 or index == 3
    local sideSign = isRight and 1.0 or -1.0

    local axleStaticCamber = isFront and p.staticCamberFront or p.staticCamberRear
    local axleStaticToe = isFront and p.staticToeFront or p.staticToeRear

    local travel = M.state.travel[index] or 0.0
    local travelNorm = M.state.travelNorm[index] or 0.0
    local loadPath = M.state.loadPath[index] or 0.0
    local roadInput = M.state.roadInput[index] or 0.0
    local contactLoss = M.state.contactLoss[index] or 0.0
    local tireForceLat = M.state.tireForceLat[index] or 0.0

    local rollCamber =
        p.dCamber_dRoll *
        roll *
        sideSign *
        p.rollInputScale

    local steerCamber = 0.0
    if isFront then
        steerCamber =
            p.dCamber_dSteer *
            math.abs(steerDeg) *
            p.steerCamberScale *
            math.pi /
            180.0
    end

    local travelCamber =
        -travelNorm *
        p.travelCamberScale *
        0.035

    local loadCamber =
        clamp(loadPath, -1.5, 1.5) *
        p.loadPathCamberGain *
        sideSign

    local contactCamber =
        -contactLoss *
        p.contactLossCamberGain *
        sideSign

    local camberTarget =
        axleStaticCamber +
        rollCamber +
        steerCamber +
        travelCamber +
        loadCamber +
        contactCamber

    local ackermann = 0.0
    if isFront then
        ackermann =
            p.dToe_dSteer *
            steerDeg *
            steerDeg *
            sign(steerDeg) *
            p.ackermannScale *
            math.pi /
            180.0

        local turningRight = steerDeg > 0.0
        local isInner =
            (turningRight and isRight) or
            ((not turningRight) and (not isRight))

        if isInner then
            ackermann = ackermann * 1.18
        else
            ackermann = ackermann * 0.82
        end
    end

    local bumpSteer =
        p.dToe_dRoll *
        travel *
        p.travelToeScale

    local rollToe =
        p.dToe_dRoll *
        roll *
        sideSign *
        0.25

    local loadToe =
        clamp(loadPath, -1.5, 1.5) *
        p.loadPathToeGain *
        sideSign

    local roadToe =
        roadInput *
        p.roadInputToeGain *
        sideSign

    local forceToe =
        clamp(tireForceLat * p.tireForceToeGain, -0.035, 0.035)

    local contactToe =
        contactLoss *
        p.contactLossToeGain *
        sideSign

    local toeTarget =
        axleStaticToe +
        ackermann +
        bumpSteer +
        rollToe +
        loadToe +
        roadToe +
        forceToe +
        contactToe

    M.state.targetCamber[index] =
        clamp(camberTarget, -p.maxCamberRad, p.maxCamberRad)

    M.state.targetToe[index] =
        clamp(toeTarget, -p.maxToeRad, p.maxToeRad)
end

local function updateOutputs(index, dt)
    local p = M.params

    M.state.camber[index] =
        lowPass(
            M.state.camber[index] or 0.0,
            M.state.targetCamber[index] or 0.0,
            p.camberTau,
            dt
        )

    M.state.toe[index] =
        lowPass(
            M.state.toe[index] or 0.0,
            M.state.targetToe[index] or 0.0,
            p.toeTau,
            dt
        )

    M.state.camber[index] =
        clamp(M.state.camber[index], -p.maxCamberRad, p.maxCamberRad)

    M.state.toe[index] =
        clamp(M.state.toe[index], -p.maxToeRad, p.maxToeRad)

    local muCamber =
        math.cos(math.abs(M.state.camber[index] or 0.0))

    muCamber =
        muCamber *
        (1.0 - (M.state.contactLoss[index] or 0.0) * 0.015)

    M.state.muCamber[index] =
        clamp(muCamber, p.minMuCamber, p.maxMuCamber)

    M.state.alphaOffset[index] =
        clamp(
            M.state.toe[index] or 0.0,
            -p.maxAlphaOffsetRad,
            p.maxAlphaOffsetRad
        )
end

--============================================================
-- Export
--============================================================

local function exportWheel(index)
    safeStore("ngp_uc_camber_" .. index, M.state.camber[index] or 0.0)
    safeStore("ngp_uc_toe_" .. index, M.state.toe[index] or 0.0)
    safeStore("ngp_uc_mu_camber_" .. index, M.state.muCamber[index] or 1.0)
    safeStore("ngp_uc_alpha_offset_" .. index, M.state.alphaOffset[index] or 0.0)
    safeStore("ngp_uc_travel_" .. index, M.state.travel[index] or 0.0)

    safeStore("ngp_ultra_chassis_camber_" .. index, M.state.camber[index] or 0.0)
    safeStore("ngp_ultra_chassis_toe_" .. index, M.state.toe[index] or 0.0)
    safeStore("ngp_ultra_chassis_mu_camber_" .. index, M.state.muCamber[index] or 1.0)
    safeStore("ngp_ultra_chassis_alpha_offset_" .. index, M.state.alphaOffset[index] or 0.0)

    safeStore("ngp_uc_load_path_" .. index, M.state.loadPath[index] or 0.0)
    safeStore("ngp_uc_road_input_" .. index, M.state.roadInput[index] or 0.0)

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_uc_target_camber_" .. index, M.state.targetCamber[index] or 0.0)
    safeStore("ngp_uc_target_toe_" .. index, M.state.targetToe[index] or 0.0)
    safeStore("ngp_uc_travel_norm_" .. index, M.state.travelNorm[index] or 0.0)
    safeStore("ngp_uc_contact_loss_" .. index, M.state.contactLoss[index] or 0.0)
    safeStore("ngp_uc_tire_force_lat_" .. index, M.state.tireForceLat[index] or 0.0)
end

local function exportGlobal()
    safeStore("ngp_ultra_chassis_status", M.state.status or "UNKNOWN")
    safeStore("ngp_ultra_chassis_update_count", M.state.updateCount or 0)
    safeStore("ngp_ultra_chassis_wheels_valid", M.state.wheelsValid and 1 or 0)

    safeStore("ngp_ultra_chassis_suspension_linked", M.state.suspensionLinked and 1 or 0)
    safeStore("ngp_ultra_chassis_roll_linked", M.state.rollLinked and 1 or 0)
    safeStore("ngp_ultra_chassis_steer_linked", M.state.steerLinked and 1 or 0)

    safeStore("ngp_uc_steer_deg", M.state.steerDeg or 0.0)
    safeStore("ngp_uc_roll", M.state.rollFiltered or 0.0)

    safeStore("ngp_uc_avg_camber_abs", M.state.avgCamberAbs or 0.0)
    safeStore("ngp_uc_avg_toe_abs", M.state.avgToeAbs or 0.0)
    safeStore("ngp_uc_avg_mu_camber", M.state.avgMuCamber or 1.0)
    safeStore("ngp_uc_max_alpha_offset", M.state.maxAlphaOffset or 0.0)
    safeStore("ngp_uc_active_count", M.state.activeCount or 0)

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_uc_load_path_linked", M.state.loadPathLinked and 1 or 0)
    safeStore("ngp_uc_road_linked", M.state.roadLinked and 1 or 0)
    safeStore("ngp_uc_contact_linked", M.state.contactLinked and 1 or 0)
    safeStore("ngp_uc_tire_force_linked", M.state.tireForceLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i)
    end

    exportGlobal()
end

local function updateAverages()
    local sumCamber = 0.0
    local sumToe = 0.0
    local sumMu = 0.0
    local maxAlpha = 0.0
    local activeCount = 0

    for i = 0, 3 do
        sumCamber = sumCamber + math.abs(M.state.camber[i] or 0.0)
        sumToe = sumToe + math.abs(M.state.toe[i] or 0.0)
        sumMu = sumMu + (M.state.muCamber[i] or 1.0)
        maxAlpha = math.max(maxAlpha, math.abs(M.state.alphaOffset[i] or 0.0))
        activeCount = activeCount + 1
    end

    M.state.avgCamberAbs = sumCamber * 0.25
    M.state.avgToeAbs = sumToe * 0.25
    M.state.avgMuCamber = sumMu * 0.25
    M.state.maxAlphaOffset = maxAlpha
    M.state.activeCount = activeCount
end

--============================================================
-- Update
--============================================================

function M.init()
    M.state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    dt = clamp(safeNumber(dt, 0.0), M.params.minDt, M.params.maxDt)

    M.state.updateCount = (M.state.updateCount or 0) + 1

    updateDebugGate(dt)

    if not car then
        car = safeGetCar()
    end

    local hasCar = car ~= nil
    local hasWheels = hasCar and safeField(car, "wheels", nil) ~= nil

    M.state.suspensionLinked = false
    M.state.rollLinked = false
    M.state.steerLinked = false
    M.state.loadPathLinked = false
    M.state.roadLinked = false
    M.state.contactLinked = false
    M.state.tireForceLinked = false

    if hasCar and hasWheels then
        M.state.status = "RUNNING"
        M.state.wheelsValid = true
    elseif hasCar then
        M.state.status = "NO WHEELS"
        M.state.wheelsValid = false
    else
        M.state.status = "STORE ONLY"
        M.state.wheelsValid = false
    end

    local steer, steerDeg = readSteer(car)
    local roll = readRoll(car)

    M.state.roll = roll

    M.state.rollFiltered =
        lowPass(
            M.state.rollFiltered or 0.0,
            roll,
            M.params.rollTau,
            dt
        )

    for i = 0, 3 do
        local wheel = getWheel(car, i)

        readTravel(i, wheel)
        readRootWheelInputs(i)

        calculateWheel(
            i,
            steerDeg,
            M.state.rollFiltered
        )

        updateOutputs(
            i,
            dt
        )
    end

    updateAverages()
    exportState()
end

--============================================================
-- Public API
--============================================================

function M.getCamber(index)
    return M.state.camber[index] or 0.0
end

function M.getToe(index)
    return M.state.toe[index] or 0.0
end

function M.getMuCamber(index)
    return M.state.muCamber[index] or 1.0
end

function M.getAlphaOffset(index)
    return M.state.alphaOffset[index] or 0.0
end

function M.getTravel(index)
    return M.state.travel[index] or 0.0
end

function M.getState(index)
    if index == nil then
        return M.state
    end

    return {
        camber = M.state.camber[index] or 0.0,
        toe = M.state.toe[index] or 0.0,
        muCamber = M.state.muCamber[index] or 1.0,
        alphaOffset = M.state.alphaOffset[index] or 0.0,
        travel = M.state.travel[index] or 0.0,
        travelNorm = M.state.travelNorm[index] or 0.0,
        roadInput = M.state.roadInput[index] or 0.0,
        contactLoss = M.state.contactLoss[index] or 0.0,
    }
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0

        return string.format(
            "%s UC / Cam %.3f / Toe %.3f\n" ..
            "MuCam %.3f / AOff %.3f / Trav %.3f Norm %.2f\n" ..
            "Target C %.3f / T %.3f / Road %.2f / CL %.2f",
            tostring(WHEEL_NAMES[i] or i),
            M.state.camber[i] or 0.0,
            M.state.toe[i] or 0.0,
            M.state.muCamber[i] or 1.0,
            M.state.alphaOffset[i] or 0.0,
            M.state.travel[i] or 0.0,
            M.state.travelNorm[i] or 0.0,
            M.state.targetCamber[i] or 0.0,
            M.state.targetToe[i] or 0.0,
            M.state.roadInput[i] or 0.0,
            M.state.contactLoss[i] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "Steer %.2f deg / Roll %.3f\n" ..
        "Avg Cam %.3f Toe %.3f Mu %.3f AOff %.3f\n" ..
        "Links Susp:%s Roll:%s Steer:%s LP:%s Road:%s CT:%s TF:%s\n" ..
        "Camber %.3f %.3f %.3f %.3f\n" ..
        "Toe    %.3f %.3f %.3f %.3f",
        tostring(M.state.status),
        M.state.updateCount or 0,
        M.state.wheelsValid and "OK" or "NIL",

        M.state.steerDeg or 0.0,
        M.state.rollFiltered or 0.0,

        M.state.avgCamberAbs or 0.0,
        M.state.avgToeAbs or 0.0,
        M.state.avgMuCamber or 1.0,
        M.state.maxAlphaOffset or 0.0,

        M.state.suspensionLinked and "OK" or "NIL",
        M.state.rollLinked and "OK" or "NIL",
        M.state.steerLinked and "OK" or "NIL",
        M.state.loadPathLinked and "OK" or "NIL",
        M.state.roadLinked and "OK" or "NIL",
        M.state.contactLinked and "OK" or "NIL",
        M.state.tireForceLinked and "OK" or "NIL",

        M.state.camber[0] or 0.0,
        M.state.camber[1] or 0.0,
        M.state.camber[2] or 0.0,
        M.state.camber[3] or 0.0,

        M.state.toe[0] or 0.0,
        M.state.toe[1] or 0.0,
        M.state.toe[2] or 0.0,
        M.state.toe[3] or 0.0
    )
end

function M.drawDebug()
    if not ui then
        return
    end

    ui.text("=== ULTRA CHASSIS ===")
    ui.text(M.debugStr())

    for i = 0, 3 do
        ui.text(M.debugStr(i))
    end
end

return M
