---@diagnostic disable: undefined-global

--============================================================
-- ACNextGen
-- tire_thermal_brush.lua
-- Phase 1.9 / V1.1.5 Stable
-- rFactor2-style Tire Thermodynamics + Brush Contact Model
--============================================================

local M = {}

--============================================================
-- Parameters
--============================================================

M.params = {
    heatRolling = 5.10e-1,
    heatFriction = 4.50e-3,

    condRoad = 10.40e-3,
    condAir = 2.90e-4,
    condInternal = 0.010,

    treadHeatCapacity = 50.0,
    carcassHeatCapacity = 140.0,

    treadToCarcassRate = 0.018,
    carcassToAmbientRate = 0.0015,

    staticFriction = 1.45,
    slidingMult = 0.77,

    optimalTemp = 85.0,
    tempWidth = 40.0,

    coldMuMin = 0.72,
    hotMuMin = 0.58,

    minMu = 0.55,
    maxMu = 1.55,

    corneringStiffness = 80000.0,

    loadReference = 3500.0,
    minLoad = 250.0,
    maxLoad = 9000.0,

    maxSlipAngle = 0.85,
    maxSlipRatio = 1.00,
    maxSlipVelocity = 75.0,

    complianceGripDropGain = 0.12,
    complianceDelayHeatGain = 0.10,

    contactLossMuDropGain = 0.20,
    contactQualityMuGain = 0.08,

    memoryHeatGain = 0.06,
    memoryMuDropGain = 0.05,

    hopHeatGain = 0.08,
    hopMuDropGain = 0.06,

    carcassHeatSeedGain = 0.18,
    carcassHistoryHeatGain = 0.12,
    carcassHysteresisHeatGain = 0.10,
    carcassSupportMuGain = 0.06,
    carcassGripGateMuGain = 0.07,
    sidewallHeatGain = 0.08,
    deformationHeatGain = 0.05,
    bristleHeatGain = 0.06,

    tireLimitMuDropGain = 0.06,
    slipEnergyHeatGain = 0.10,
    combinedSlipHeatGain = 0.06,

    roadShockHeatGain = 0.08,
    loadPathLossHeatGain = 0.08,
    loadPathWorkHeatGain = 0.05,

    ultraCamberMuInfluence = 0.15,
    ultraAlphaOffsetScale = 0.60,

    loadHeatGain = 0.10,
    lowLoadCoolingGain = 0.22,

    fyRelaxLossGain = 0.12,

    tempMin = -20.0,
    tempMax = 220.0,

    muTau = 0.060,
    forceTau = 0.040,
    heatTau = 0.060,

    minDt = 0.00005,
    maxDt = 0.030,

    debugStoreInterval = 0.25,
}

--============================================================
-- State
--============================================================

M.state = {
    treadTemp = { [0]=30.0, [1]=30.0, [2]=30.0, [3]=30.0 },
    carcassTemp = { [0]=45.0, [1]=45.0, [2]=45.0, [3]=45.0 },

    muRaw = { [0]=1.0, [1]=1.0, [2]=1.0, [3]=1.0 },
    mu = { [0]=1.0, [1]=1.0, [2]=1.0, [3]=1.0 },
    muScale = { [0]=1.0, [1]=1.0, [2]=1.0, [3]=1.0 },

    alpha = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    slipRatio = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    slipVelocity = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    load = { [0]=3500.0, [1]=3500.0, [2]=3500.0, [3]=3500.0 },

    fyRaw = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    fy = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    brushXi = { [0]=1.0, [1]=1.0, [2]=1.0, [3]=1.0 },
    sliding = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    qGen = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    qLoss = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    wheels = {},

    avgTreadTemp = 30.0,
    avgCarcassTemp = 45.0,
    avgMuScale = 1.0,
    avgFyAbs = 0.0,
    avgSliding = 0.0,
    maxTreadTemp = 30.0,
    minMuScale = 1.0,
    maxSliding = 0.0,
    activeCount = 0,

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    tireStateLinked = false,
    complianceLinked = false,
    contactLinked = false,
    memoryLinked = false,
    hopLinked = false,
    carcassLinked = false,
    dynamicsLinked = false,
    roadLinked = false,
    loadPathLinked = false,
    ultraChassisLinked = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

for i = 0, 3 do
    M.state.wheels[i] = {
        active = false,
        load = M.state.load[i],
        alpha = 0.0,
        slipRatio = 0.0,
        slipVelocity = 0.0,

        compliance = 1.0,
        responseDelay = 0.0,
        contactLoss = 0.0,
        contactQuality = 1.0,
        memory = 0.0,
        hop = 0.0,

        carcassSupport = 1.0,
        carcassGripGate = 1.0,
        carcassHeatSeed = 0.0,
        carcassHistoryStress = 0.0,
        carcassHysteresis = 0.0,
        sidewallEnergy = 0.0,
        deformation = 0.0,
        bristleEnergy = 0.0,

        tireLimit = 0.0,
        tireGrip = 1.0,
        combinedSlip = 0.0,
        slipEnergy = 0.0,

        roadShock = 0.0,
        pathLoss = 0.0,
        pathWork = 0.0,

        ucMuCamber = 1.0,
        ucAlphaOffset = 0.0,

        treadTemp = M.state.treadTemp[i],
        carcassTemp = M.state.carcassTemp[i],
        mu = M.state.mu[i],
        muScale = M.state.muScale[i],
        fy = 0.0,
        sliding = 0.0,
        brushXi = 1.0,
    }

    M.state[i] = M.state.wheels[i]
end

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

local function abs(v)
    return math.abs(safeNumber(v, 0.0))
end

local function sign(v)
    v = safeNumber(v, 0.0)

    if v < 0.0 then
        return -1.0
    end

    return 1.0
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

    if not ok then
        return nil
    end

    return car
end

local function safeGetWheels(car)
    return safeField(car, "wheels", nil)
end

local function getWheel(car, index)
    local wheels = safeGetWheels(car)

    if not wheels then
        return nil
    end

    local ok, wheel = pcall(function()
        return wheels[index]
    end)

    if ok and wheel ~= nil then
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

local function resetLinks()
    M.state.tireStateLinked = false
    M.state.complianceLinked = false
    M.state.contactLinked = false
    M.state.memoryLinked = false
    M.state.hopLinked = false
    M.state.carcassLinked = false
    M.state.dynamicsLinked = false
    M.state.roadLinked = false
    M.state.loadPathLinked = false
    M.state.ultraChassisLinked = false
end

local function updateDebugGate(dt)
    M.state.debugStoreTimer =
        (M.state.debugStoreTimer or 0.0)
        +
        (dt or 0.0)

    if M.state.debugStoreTimer >= M.params.debugStoreInterval then
        M.state.debugStoreTimer = 0.0
        M.state.debugStoreNow = true
    else
        M.state.debugStoreNow = false
    end
end

--============================================================
-- Inputs
--============================================================

local function readCarSpeedMS(car)
    local speedMS =
        safeLoadRaw("ngp_tire_state_speed_ms")
        or
        safeLoadRaw("ngp_vehicle_speed_ms")
        or
        safeLoadRaw("ngp_speed_ms")

    if speedMS ~= nil then
        return abs(speedMS)
    end

    local speedKmh =
        safeLoadRaw("ngp_tire_state_speed_kmh")
        or
        safeLoadRaw("ngp_vehicle_speed_kmh")
        or
        safeLoadRaw("ngp_speed_kmh")

    if speedKmh ~= nil then
        return abs(speedKmh) / 3.6
    end

    if car then
        local carSpeedKmh = safeField(car, "speedKmh", nil)

        if carSpeedKmh ~= nil then
            return abs(carSpeedKmh) / 3.6
        end

        local speed = nil

        if speed ~= nil then
            local n = abs(speed)

            if n > 120.0 then
                return n / 3.6
            end

            return n
        end

        local velocity = safeField(car, "velocity", nil)

        if type(velocity) == "number" then
            return abs(velocity)
        end

        if velocity and velocity.length ~= nil then
            local ok, len = pcall(function()
                return velocity:length()
            end)

            if ok then
                return abs(len)
            end
        end

        local localVelocity = safeField(car, "localVelocity", nil)

        if type(localVelocity) == "table" then
            local x = safeNumber(localVelocity.x, 0.0)
            local y = safeNumber(localVelocity.y, 0.0)
            local z = safeNumber(localVelocity.z, 0.0)

            return math.sqrt(x * x + y * y + z * z)
        end
    end

    return 0.0
end

local function readTemperatureFromSim(field, fallback)
    if not ac or not ac.getSim then
        return fallback
    end

    local ok, sim = pcall(function()
        return ac.getSim()
    end)

    if ok and sim and sim[field] ~= nil then
        return safeNumber(sim[field], fallback)
    end

    return fallback
end

local function readRoadTemp()
    local road =
        safeLoadRaw("ngp_track_temp")
        or
        safeLoadRaw("ngp_road_temp")
        or
        safeLoadRaw("ngp_surface_temp")

    if road ~= nil then
        return safeNumber(road, 30.0)
    end

    return readTemperatureFromSim("roadTemperature", 30.0)
end

local function readAmbientTemp()
    local ambient =
        safeLoadRaw("ngp_ambient_temp")
        or
        safeLoadRaw("ngp_air_temp")

    if ambient ~= nil then
        return safeNumber(ambient, 25.0)
    end

    return readTemperatureFromSim("ambientTemperature", 25.0)
end

local function readLoad(index, wheel)
    local load =
        safeLoadRaw("ngp_tire_state_load_" .. index)
        or
        safeLoadRaw("ngp_tire_load_" .. index)
        or
        safeLoadRaw("ngp_tire_carcass_load_" .. index)
        or
        safeLoadRaw("ngp_cp_load_" .. index)
        or
        safeLoadRaw("ngp_tc_load_" .. index)
        or
        safeLoadRaw("ngp_wheel_load_" .. index)
        or
        safeLoadRaw("ngp_load_wheel_" .. index)

    if load ~= nil then
        M.state.tireStateLinked = true
        return clamp(safeNumber(load, M.params.loadReference), M.params.minLoad, M.params.maxLoad)
    end

    local direct =
        safeField(wheel, "load", nil)
        or
        safeField(wheel, "loadK", nil)
        or
        nil
        or
        nil
        or
        nil

    if direct ~= nil then
        return clamp(safeNumber(direct, M.params.loadReference), M.params.minLoad, M.params.maxLoad)
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)

    if dlt ~= nil then
        M.state.tireStateLinked = true
        return clamp(M.params.loadReference + safeNumber(dlt, 0.0), M.params.minLoad, M.params.maxLoad)
    end

    return M.params.loadReference
end

local function readSlipAngle(index, wheel)
    local alpha =
        safeLoadRaw("ngp_tire_alpha_eff_" .. index)
        or
        safeLoadRaw("ngp_tire_effective_slip_angle_" .. index)
        or
        safeLoadRaw("ngp_tdyn_effective_slip_angle_" .. index)
        or
        safeLoadRaw("ngp_tire_force_effective_slip_angle_" .. index)
        or
        safeLoadRaw("ngp_tire_slip_angle_" .. index)
        or
        safeLoadRaw("ngp_slip_angle_" .. index)
        or
        safeLoadRaw("ngp_filtered_slip_angle_" .. index)
        or
        safeLoadRaw("ngp_contact_slip_angle_" .. index)
        or
        safeLoadRaw("ngp_tire_carcass_slip_angle_" .. index)

    if alpha ~= nil then
        M.state.tireStateLinked = true
        return clamp(safeNumber(alpha, 0.0), -M.params.maxSlipAngle, M.params.maxSlipAngle)
    end

    local direct = safeField(wheel, "slipAngle", nil)

    if direct ~= nil then
        return clamp(safeNumber(direct, 0.0), -M.params.maxSlipAngle, M.params.maxSlipAngle)
    end

    return 0.0
end

local function readSlipRatio(index, wheel)
    local ratio =
        safeLoadRaw("ngp_tire_effective_slip_ratio_" .. index)
        or
        safeLoadRaw("ngp_tdyn_effective_slip_ratio_" .. index)
        or
        safeLoadRaw("ngp_tire_force_effective_slip_ratio_" .. index)
        or
        safeLoadRaw("ngp_tire_slip_ratio_" .. index)
        or
        safeLoadRaw("ngp_slip_ratio_" .. index)
        or
        safeLoadRaw("ngp_filtered_slip_ratio_" .. index)
        or
        safeLoadRaw("ngp_contact_slip_ratio_" .. index)
        or
        safeLoadRaw("ngp_tire_carcass_slip_ratio_" .. index)

    if ratio ~= nil then
        M.state.tireStateLinked = true
        return clamp(safeNumber(ratio, 0.0), -M.params.maxSlipRatio, M.params.maxSlipRatio)
    end

    local direct = safeField(wheel, "slipRatio", nil)

    if direct ~= nil then
        return clamp(safeNumber(direct, 0.0), -M.params.maxSlipRatio, M.params.maxSlipRatio)
    end

    return 0.0
end

local function readCoupling(index)
    local compliance =
        safeLoadRaw("ngp_tire_compliance_" .. index)
        or
        safeLoadRaw("ngp_tcomp_compliance_" .. index)

    if compliance ~= nil then
        M.state.complianceLinked = true
    end

    local responseDelay =
        safeLoadRaw("ngp_tire_response_delay")
        or
        safeLoadRaw("ngp_tyre_response_delay")
        or
        safeLoadRaw("ngp_tcomp_response_delay")

    if responseDelay ~= nil then
        M.state.complianceLinked = true
    end

    local contactLoss =
        safeLoadRaw("ngp_tire_contact_loss_" .. index)
        or
        safeLoadRaw("ngp_tcr_contact_loss_" .. index)
        or
        safeLoadRaw("ngp_contact_loss_" .. index)

    local contactQuality =
        safeLoadRaw("ngp_tire_contact_quality_" .. index)
        or
        safeLoadRaw("ngp_tcr_quality_" .. index)
        or
        safeLoadRaw("ngp_contact_quality_" .. index)
        or
        safeLoadRaw("ngp_tc_contact_" .. index)

    if contactLoss ~= nil or contactQuality ~= nil then
        M.state.contactLinked = true
    end

    local memory =
        safeLoadRaw("ngp_tire_memory_" .. index)
        or
        safeLoadRaw("ngp_tyre_memory_" .. index)
        or
        safeLoadRaw("ngp_rubber_memory_" .. index)
        or
        safeLoadRaw("ngp_memory_" .. index)

    if memory ~= nil then
        M.state.memoryLinked = true
    end

    local hop =
        safeLoadRaw("ngp_tire_hop_energy_" .. index)
        or
        safeLoadRaw("ngp_tirehop_energy_" .. index)
        or
        safeLoadRaw("ngp_thop_energy_" .. index)

    if hop ~= nil then
        M.state.hopLinked = true
    end

    local support =
        safeLoadRaw("ngp_carcass_support_" .. index)
        or
        safeLoadRaw("ngp_tire_carcass_support_" .. index)

    local gripGate =
        safeLoadRaw("ngp_carcass_grip_gate_" .. index)
        or
        safeLoadRaw("ngp_tire_carcass_grip_gate_" .. index)

    local heatSeed =
        safeLoadRaw("ngp_carcass_heat_seed_" .. index)
        or
        safeLoadRaw("ngp_tire_carcass_heat_seed_" .. index)

    local historyStress =
        safeLoadRaw("ngp_carcass_history_stress_" .. index)
        or
        safeLoadRaw("ngp_tire_carcass_history_stress_" .. index)

    local hysteresis =
        safeLoadRaw("ngp_carcass_hysteresis_" .. index)
        or
        safeLoadRaw("ngp_tire_carcass_hysteresis_" .. index)

    local sidewallEnergy =
        safeLoadRaw("ngp_sidewall_energy_" .. index)
        or
        safeLoadRaw("ngp_tire_sidewall_energy_" .. index)

    local deformation =
        safeLoadRaw("ngp_tire_deformation_" .. index)
        or
        safeLoadRaw("ngp_tire_carcass_deformation_" .. index)

    local bristleLat =
        safeLoadRaw("ngp_carcass_bristle_lat_" .. index)
        or
        safeLoadRaw("ngp_tire_carcass_bristle_lat_" .. index)

    local bristleLong =
        safeLoadRaw("ngp_carcass_bristle_long_" .. index)
        or
        safeLoadRaw("ngp_tire_carcass_bristle_long_" .. index)

    if support ~= nil
    or gripGate ~= nil
    or heatSeed ~= nil
    or historyStress ~= nil
    or hysteresis ~= nil
    or sidewallEnergy ~= nil
    or deformation ~= nil
    or bristleLat ~= nil
    or bristleLong ~= nil then
        M.state.carcassLinked = true
    end

    local tireLimit =
        safeLoadRaw("ngp_tire_limit_" .. index)
        or
        safeLoadRaw("ngp_tire_force_limit_" .. index)

    local tireGrip =
        safeLoadRaw("ngp_tire_grip_" .. index)
        or
        safeLoadRaw("ngp_tire_force_grip_" .. index)

    local combinedSlip =
        safeLoadRaw("ngp_tdyn_combined_slip_" .. index)
        or
        safeLoadRaw("ngp_tire_dynamics_combined_slip_" .. index)
        or
        safeLoadRaw("ngp_tire_combined_slip_" .. index)

    local slipEnergy =
        safeLoadRaw("ngp_tdyn_slip_energy_" .. index)
        or
        safeLoadRaw("ngp_tire_dynamics_slip_energy_" .. index)
        or
        safeLoadRaw("ngp_tire_slip_energy_" .. index)

    if tireLimit ~= nil or tireGrip ~= nil or combinedSlip ~= nil or slipEnergy ~= nil then
        M.state.dynamicsLinked = true
    end

    local roadShock =
        safeLoadRaw("ngp_rii_shock_" .. index)
        or
        safeLoadRaw("ngp_road_shock_" .. index)
        or
        safeLoadRaw("ngp_rbi_impact_" .. index)

    if roadShock ~= nil then
        M.state.roadLinked = true
    end

    local pathLoss =
        safeLoadRaw("ngp_load_path_loss_" .. index)
        or
        safeLoadRaw("ngp_lp_loss_" .. index)

    local pathWork =
        safeLoadRaw("ngp_load_path_work_" .. index)
        or
        safeLoadRaw("ngp_lp_work_" .. index)

    if pathLoss ~= nil or pathWork ~= nil then
        M.state.loadPathLinked = true
    end

    local ucMuCamber =
        safeLoadRaw("ngp_uc_mu_camber_" .. index)
        or
        safeLoadRaw("ngp_uchassis_mu_camber_" .. index)

    local ucAlphaOffset =
        safeLoadRaw("ngp_uc_alpha_offset_" .. index)
        or
        safeLoadRaw("ngp_uchassis_alpha_offset_" .. index)

    if ucMuCamber ~= nil or ucAlphaOffset ~= nil then
        M.state.ultraChassisLinked = true
    end

    return {
        compliance = clamp(safeNumber(compliance, 1.0), 0.40, 1.30),
        responseDelay = clamp(safeNumber(responseDelay, 0.0), 0.0, 1.0),

        contactLoss = clamp(safeNumber(contactLoss, 0.0), 0.0, 1.0),
        contactQuality = clamp(safeNumber(contactQuality, 1.0), 0.0, 1.2),

        memory = clamp(safeNumber(memory, 0.0), 0.0, 1.0),
        hop = clamp(safeNumber(hop, 0.0), 0.0, 1.0),

        carcassSupport = clamp(safeNumber(support, 1.0), 0.0, 1.2),
        carcassGripGate = clamp(safeNumber(gripGate, 1.0), 0.0, 1.2),
        carcassHeatSeed = clamp(safeNumber(heatSeed, 0.0), 0.0, 1.0),
        carcassHistoryStress = clamp(safeNumber(historyStress, 0.0), 0.0, 1.0),
        carcassHysteresis = clamp(safeNumber(hysteresis, 0.0), 0.0, 1.0),
        sidewallEnergy = clamp(safeNumber(sidewallEnergy, 0.0), 0.0, 1.0),
        deformation = clamp(safeNumber(deformation, 0.0), 0.0, 1.0),
        bristleEnergy = clamp(math.max(safeNumber(bristleLat, 0.0), safeNumber(bristleLong, 0.0)), 0.0, 1.0),

        tireLimit = clamp(safeNumber(tireLimit, 0.0), 0.0, 2.0),
        tireGrip = clamp(safeNumber(tireGrip, 1.0), 0.0, 1.25),
        combinedSlip = clamp(safeNumber(combinedSlip, 0.0), 0.0, 3.0),
        slipEnergy = clamp(safeNumber(slipEnergy, 0.0), 0.0, 1.5),

        roadShock = clamp(safeNumber(roadShock, 0.0), 0.0, 1.5),
        pathLoss = clamp(safeNumber(pathLoss, 0.0), 0.0, 1.0),
        pathWork = clamp(safeNumber(pathWork, 0.0), 0.0, 2.0),

        ucMuCamber = clamp(safeNumber(ucMuCamber, 1.0), 0.75, 1.05),
        ucAlphaOffset = clamp(safeNumber(ucAlphaOffset, 0.0), -0.20, 0.20),
    }
end

--============================================================
-- Thermal / friction / brush model
--============================================================

local function updateWheelMirror(index, coupling)
    local ws = M.state.wheels[index]

    if not ws then
        return
    end

    ws.active = true
    ws.load = M.state.load[index] or M.params.loadReference
    ws.alpha = M.state.alpha[index] or 0.0
    ws.slipRatio = M.state.slipRatio[index] or 0.0
    ws.slipVelocity = M.state.slipVelocity[index] or 0.0

    ws.compliance = coupling.compliance
    ws.responseDelay = coupling.responseDelay
    ws.contactLoss = coupling.contactLoss
    ws.contactQuality = coupling.contactQuality
    ws.memory = coupling.memory
    ws.hop = coupling.hop

    ws.carcassSupport = coupling.carcassSupport
    ws.carcassGripGate = coupling.carcassGripGate
    ws.carcassHeatSeed = coupling.carcassHeatSeed
    ws.carcassHistoryStress = coupling.carcassHistoryStress
    ws.carcassHysteresis = coupling.carcassHysteresis
    ws.sidewallEnergy = coupling.sidewallEnergy
    ws.deformation = coupling.deformation
    ws.bristleEnergy = coupling.bristleEnergy

    ws.tireLimit = coupling.tireLimit
    ws.tireGrip = coupling.tireGrip
    ws.combinedSlip = coupling.combinedSlip
    ws.slipEnergy = coupling.slipEnergy

    ws.roadShock = coupling.roadShock
    ws.pathLoss = coupling.pathLoss
    ws.pathWork = coupling.pathWork

    ws.ucMuCamber = coupling.ucMuCamber
    ws.ucAlphaOffset = coupling.ucAlphaOffset

    ws.treadTemp = M.state.treadTemp[index]
    ws.carcassTemp = M.state.carcassTemp[index]
    ws.mu = M.state.mu[index]
    ws.muScale = M.state.muScale[index]
    ws.fy = M.state.fy[index]
    ws.sliding = M.state.sliding[index]
    ws.brushXi = M.state.brushXi[index]
end

local function updateThermal(index, load, alpha, slipRatio, speedMS, fyPrev, roadTemp, ambientTemp, coupling, dt)
    local p = M.params

    local lateralSlipVelocity =
        math.abs(
            speedMS
            *
            math.sin(alpha)
        )

    local longitudinalSlipVelocity =
        math.abs(
            speedMS
            *
            slipRatio
            *
            0.35
        )

    local slipVelocity =
        clamp(
            math.sqrt(
                lateralSlipVelocity * lateralSlipVelocity
                +
                longitudinalSlipVelocity * longitudinalSlipVelocity
            ),
            0.0,
            p.maxSlipVelocity
        )

    M.state.slipVelocity[index] = slipVelocity

    local loadScale =
        clamp(
            load
            /
            math.max(p.loadReference, 1.0),
            0.0,
            2.5
        )

    local rootHeat =
        1.0
        +
        coupling.memory * p.memoryHeatGain
        +
        coupling.hop * p.hopHeatGain
        +
        coupling.responseDelay * p.complianceDelayHeatGain
        +
        coupling.carcassHeatSeed * p.carcassHeatSeedGain
        +
        coupling.carcassHistoryStress * p.carcassHistoryHeatGain
        +
        coupling.carcassHysteresis * p.carcassHysteresisHeatGain
        +
        coupling.sidewallEnergy * p.sidewallHeatGain
        +
        coupling.deformation * p.deformationHeatGain
        +
        coupling.bristleEnergy * p.bristleHeatGain
        +
        coupling.slipEnergy * p.slipEnergyHeatGain
        +
        clamp(coupling.combinedSlip - 1.0, 0.0, 2.0) * p.combinedSlipHeatGain
        +
        coupling.roadShock * p.roadShockHeatGain
        +
        coupling.pathLoss * p.loadPathLossHeatGain
        +
        coupling.pathWork * p.loadPathWorkHeatGain
        +
        math.max(loadScale - 1.0, 0.0) * p.loadHeatGain

    local qGen =
        (
            p.heatFriction
            *
            math.abs(fyPrev * slipVelocity)
            +
            p.heatRolling
            *
            speedMS
            *
            (0.50 + 0.50 * loadScale)
        )
        *
        rootHeat

    local treadTemp = M.state.treadTemp[index] or 30.0
    local carcassTemp = M.state.carcassTemp[index] or 45.0

    local lowLoadCooling =
        math.max(
            1.0
            -
            load
            /
            math.max(p.minLoad * 4.0, 1.0),
            0.0
        )
        *
        p.lowLoadCoolingGain

    local qLoss =
        p.condRoad * (treadTemp - roadTemp) * (0.30 + 0.70 * loadScale)
        +
        p.condAir * speedMS * (treadTemp - ambientTemp) * (1.0 + lowLoadCooling)
        +
        p.condInternal * (treadTemp - carcassTemp)

    local dTread =
        (qGen - qLoss)
        /
        math.max(p.treadHeatCapacity, 1.0)

    treadTemp =
        treadTemp
        +
        dTread
        *
        dt

    local carcassFlow =
        p.treadToCarcassRate
        *
        (treadTemp - carcassTemp)

    local carcassLoss =
        p.carcassToAmbientRate
        *
        (carcassTemp - ambientTemp)

    local dCarcass =
        (carcassFlow - carcassLoss)
        /
        math.max(p.carcassHeatCapacity, 1.0)

    carcassTemp =
        carcassTemp
        +
        dCarcass
        *
        dt

    M.state.treadTemp[index] =
        lowPass(
            M.state.treadTemp[index] or treadTemp,
            clamp(treadTemp, p.tempMin, p.tempMax),
            p.heatTau,
            dt
        )

    M.state.carcassTemp[index] =
        lowPass(
            M.state.carcassTemp[index] or carcassTemp,
            clamp(carcassTemp, p.tempMin, p.tempMax),
            p.heatTau * 2.0,
            dt
        )

    M.state.treadTemp[index] =
        clamp(M.state.treadTemp[index], p.tempMin, p.tempMax)

    M.state.carcassTemp[index] =
        clamp(M.state.carcassTemp[index], p.tempMin, p.tempMax)

    M.state.qGen[index] = qGen
    M.state.qLoss[index] = qLoss
end

local function calculateMu(index, coupling, dt)
    local p = M.params
    local treadTemp = M.state.treadTemp[index] or 30.0

    local normalized =
        (treadTemp - p.optimalTemp)
        /
        math.max(p.tempWidth, 1.0)

    local tempShape =
        math.exp(
            -normalized
            *
            normalized
        )

    local mu =
        p.staticFriction
        *
        tempShape

    if treadTemp < p.optimalTemp then
        mu =
            math.max(
                mu,
                p.staticFriction
                *
                p.coldMuMin
            )
    end

    if treadTemp > p.optimalTemp then
        mu =
            math.max(
                mu,
                p.staticFriction
                *
                p.hotMuMin
            )
    end

    local complianceDrop =
        math.max(1.0 - coupling.compliance, 0.0)
        *
        p.complianceGripDropGain

    local contactDrop =
        coupling.contactLoss
        *
        p.contactLossMuDropGain

    local contactGain =
        math.max(coupling.contactQuality - 1.0, 0.0)
        *
        p.contactQualityMuGain

    local memoryDrop =
        coupling.memory
        *
        p.memoryMuDropGain

    local hopDrop =
        coupling.hop
        *
        p.hopMuDropGain

    local carcassGain =
        math.max(coupling.carcassSupport - 1.0, 0.0)
        *
        p.carcassSupportMuGain
        +
        math.max(coupling.carcassGripGate - 1.0, 0.0)
        *
        p.carcassGripGateMuGain

    local tireLimitDrop =
        coupling.tireLimit
        *
        p.tireLimitMuDropGain

    local relaxationDrop =
        coupling.responseDelay
        *
        p.fyRelaxLossGain

    mu =
        mu
        *
        (
            1.0
            -
            complianceDrop
            -
            contactDrop
            -
            memoryDrop
            -
            hopDrop
            -
            tireLimitDrop
            -
            relaxationDrop
            +
            contactGain
            +
            carcassGain
        )

    mu =
        mu
        *
        (
            0.85
            +
            0.15
            *
            (coupling.ucMuCamber or 1.0)
        )

    mu =
        mu
        *
        (
            0.92
            +
            0.08
            *
            clamp(coupling.tireGrip, 0.0, 1.25)
        )

    mu = clamp(mu, p.minMu, p.maxMu)

    M.state.muRaw[index] = mu

    M.state.mu[index] =
        lowPass(
            M.state.mu[index] or mu,
            mu,
            p.muTau,
            dt
        )

    M.state.mu[index] =
        clamp(M.state.mu[index], p.minMu, p.maxMu)

    M.state.muScale[index] =
        clamp(
            M.state.mu[index]
            /
            math.max(p.staticFriction, 0.01),
            0.35,
            1.15
        )
end

local function calculateBrushForce(index, load, alpha, dt)
    local p = M.params

    local absAlpha =
        math.abs(
            clamp(
                alpha,
                -p.maxSlipAngle,
                p.maxSlipAngle
            )
        )

    local mu =
        clamp(
            M.state.mu[index] or 1.0,
            p.minMu,
            p.maxMu
        )

    load =
        clamp(
            load,
            p.minLoad,
            p.maxLoad
        )

    local denom =
        3.0
        *
        mu
        *
        math.max(load, p.minLoad)

    local tanAlpha =
        math.tan(absAlpha)

    local xi =
        1.0
        -
        (
            p.corneringStiffness
            *
            tanAlpha
        )
        /
        math.max(denom, 1.0)

    M.state.brushXi[index] =
        clamp(xi, -2.0, 1.0)

    local fyAbs = 0.0
    local sliding = 0.0

    if xi > 0.0 then
        fyAbs =
            mu
            *
            load
            *
            (
                1.0
                -
                xi
                *
                xi
                *
                xi
            )

        sliding =
            clamp(
                1.0 - xi,
                0.0,
                1.0
            )
    else
        fyAbs =
            mu
            *
            load
            *
            p.slidingMult

        sliding = 1.0
    end

    local fyTarget =
        fyAbs
        *
        sign(alpha)

    M.state.fyRaw[index] = fyTarget

    M.state.fy[index] =
        lowPass(
            M.state.fy[index] or fyTarget,
            fyTarget,
            p.forceTau,
            dt
        )

    M.state.sliding[index] =
        lowPass(
            M.state.sliding[index] or sliding,
            sliding,
            p.forceTau,
            dt
        )

    M.state.fy[index] =
        clamp(
            M.state.fy[index],
            -p.maxLoad * p.maxMu,
            p.maxLoad * p.maxMu
        )

    M.state.sliding[index] =
        clamp(M.state.sliding[index], 0.0, 1.0)
end

--============================================================
-- Export
--============================================================

local function exportWheel(index)
    safeStore("ngp_rf2_tread_temp_" .. index, M.state.treadTemp[index] or 30.0)
    safeStore("ngp_rf2_carcass_temp_" .. index, M.state.carcassTemp[index] or 45.0)

    safeStore("ngp_rf2_mu_" .. index, M.state.mu[index] or 1.0)
    safeStore("ngp_rf2_mu_raw_" .. index, M.state.muRaw[index] or 1.0)
    safeStore("ngp_rf2_mu_scale_" .. index, M.state.muScale[index] or 1.0)

    safeStore("ngp_rf2_fy_" .. index, M.state.fy[index] or 0.0)
    safeStore("ngp_rf2_fy_raw_" .. index, M.state.fyRaw[index] or 0.0)

    safeStore("ngp_rf2_slip_velocity_" .. index, M.state.slipVelocity[index] or 0.0)
    safeStore("ngp_rf2_sliding_" .. index, M.state.sliding[index] or 0.0)
    safeStore("ngp_rf2_brush_xi_" .. index, M.state.brushXi[index] or 1.0)

    safeStore("ngp_ttb_tread_temp_" .. index, M.state.treadTemp[index] or 30.0)
    safeStore("ngp_ttb_carcass_temp_" .. index, M.state.carcassTemp[index] or 45.0)
    safeStore("ngp_ttb_mu_" .. index, M.state.mu[index] or 1.0)
    safeStore("ngp_ttb_mu_scale_" .. index, M.state.muScale[index] or 1.0)
    safeStore("ngp_ttb_fy_" .. index, M.state.fy[index] or 0.0)
    safeStore("ngp_ttb_sliding_" .. index, M.state.sliding[index] or 0.0)

    safeStore("ngp_brush_fy_" .. index, M.state.fy[index] or 0.0)
    safeStore("ngp_brush_mu_scale_" .. index, M.state.muScale[index] or 1.0)
    safeStore("ngp_brush_sliding_" .. index, M.state.sliding[index] or 0.0)

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_rf2_q_gen_" .. index, M.state.qGen[index] or 0.0)
    safeStore("ngp_rf2_q_loss_" .. index, M.state.qLoss[index] or 0.0)
    safeStore("ngp_rf2_alpha_" .. index, M.state.alpha[index] or 0.0)
    safeStore("ngp_rf2_slip_ratio_" .. index, M.state.slipRatio[index] or 0.0)
    safeStore("ngp_rf2_load_" .. index, M.state.load[index] or 0.0)

    safeStore("ngp_ttb_q_gen_" .. index, M.state.qGen[index] or 0.0)
    safeStore("ngp_ttb_q_loss_" .. index, M.state.qLoss[index] or 0.0)
    safeStore("ngp_ttb_alpha_" .. index, M.state.alpha[index] or 0.0)
    safeStore("ngp_ttb_slip_ratio_" .. index, M.state.slipRatio[index] or 0.0)
    safeStore("ngp_ttb_load_" .. index, M.state.load[index] or 0.0)
end

local function exportGlobal()
    safeStore("ngp_rf2_tire_status", M.state.status or "UNKNOWN")
    safeStore("ngp_rf2_tire_update_count", M.state.updateCount or 0)
    safeStore("ngp_rf2_tire_wheels_valid", M.state.wheelsValid and 1 or 0)
    safeStore("ngp_rf2_tire_state_linked", M.state.tireStateLinked and 1 or 0)
    safeStore("ngp_rf2_tire_compliance_linked", M.state.complianceLinked and 1 or 0)
    safeStore("ngp_rf2_tire_contact_linked", M.state.contactLinked and 1 or 0)
    safeStore("ngp_rf2_ultra_chassis_linked", M.state.ultraChassisLinked and 1 or 0)

    safeStore("ngp_ttb_status", M.state.status or "UNKNOWN")
    safeStore("ngp_ttb_update_count", M.state.updateCount or 0)
    safeStore("ngp_ttb_wheels_valid", M.state.wheelsValid and 1 or 0)
    safeStore("ngp_ttb_avg_tread_temp", M.state.avgTreadTemp or 30.0)
    safeStore("ngp_ttb_avg_carcass_temp", M.state.avgCarcassTemp or 45.0)
    safeStore("ngp_ttb_avg_mu_scale", M.state.avgMuScale or 1.0)
    safeStore("ngp_ttb_avg_fy_abs", M.state.avgFyAbs or 0.0)
    safeStore("ngp_ttb_avg_sliding", M.state.avgSliding or 0.0)
    safeStore("ngp_ttb_active_count", M.state.activeCount or 0)

    if not M.state.debugStoreNow then
        return
    end

    safeStore("ngp_ttb_max_tread_temp", M.state.maxTreadTemp or 30.0)
    safeStore("ngp_ttb_min_mu_scale", M.state.minMuScale or 1.0)
    safeStore("ngp_ttb_max_sliding", M.state.maxSliding or 0.0)
    safeStore("ngp_ttb_tire_state_linked", M.state.tireStateLinked and 1 or 0)
    safeStore("ngp_ttb_compliance_linked", M.state.complianceLinked and 1 or 0)
    safeStore("ngp_ttb_contact_linked", M.state.contactLinked and 1 or 0)
    safeStore("ngp_ttb_memory_linked", M.state.memoryLinked and 1 or 0)
    safeStore("ngp_ttb_hop_linked", M.state.hopLinked and 1 or 0)
    safeStore("ngp_ttb_carcass_linked", M.state.carcassLinked and 1 or 0)
    safeStore("ngp_ttb_dynamics_linked", M.state.dynamicsLinked and 1 or 0)
    safeStore("ngp_ttb_road_linked", M.state.roadLinked and 1 or 0)
    safeStore("ngp_ttb_load_path_linked", M.state.loadPathLinked and 1 or 0)
    safeStore("ngp_ttb_ultra_chassis_linked", M.state.ultraChassisLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i)
    end

    exportGlobal()
end

--============================================================
-- Update
--============================================================

function M.init()
    M.state.status = "INIT"
    exportState()
end

local function updateOneWheel(index, wheel, speedMS, roadTemp, ambientTemp, dt)
    local load =
        readLoad(
            index,
            wheel
        )

    local alpha =
        readSlipAngle(
            index,
            wheel
        )

    local slipRatio =
        readSlipRatio(
            index,
            wheel
        )

    local coupling =
        readCoupling(
            index
        )

    alpha =
        clamp(
            alpha
            -
            (coupling.ucAlphaOffset or 0.0)
            *
            M.params.ultraAlphaOffsetScale,
            -M.params.maxSlipAngle,
            M.params.maxSlipAngle
        )

    M.state.load[index] = load
    M.state.alpha[index] = alpha
    M.state.slipRatio[index] = slipRatio

    updateThermal(
        index,
        load,
        alpha,
        slipRatio,
        speedMS,
        M.state.fy[index] or 0.0,
        roadTemp,
        ambientTemp,
        coupling,
        dt
    )

    calculateMu(
        index,
        coupling,
        dt
    )

    calculateBrushForce(
        index,
        load,
        alpha,
        dt
    )

    updateWheelMirror(
        index,
        coupling
    )
end

local function updateAverages()
    local sumTread = 0.0
    local sumCarcass = 0.0
    local sumMuScale = 0.0
    local sumFyAbs = 0.0
    local sumSliding = 0.0

    local maxTread = -9999.0
    local minMuScale = 9999.0
    local maxSliding = 0.0
    local activeCount = 0

    for i = 0, 3 do
        local tread = M.state.treadTemp[i] or 30.0
        local carcass = M.state.carcassTemp[i] or 45.0
        local muScale = M.state.muScale[i] or 1.0
        local fyAbs = abs(M.state.fy[i] or 0.0)
        local sliding = M.state.sliding[i] or 0.0

        sumTread = sumTread + tread
        sumCarcass = sumCarcass + carcass
        sumMuScale = sumMuScale + muScale
        sumFyAbs = sumFyAbs + fyAbs
        sumSliding = sumSliding + sliding

        maxTread = math.max(maxTread, tread)
        minMuScale = math.min(minMuScale, muScale)
        maxSliding = math.max(maxSliding, sliding)

        if M.state.wheels[i] and M.state.wheels[i].active then
            activeCount = activeCount + 1
        end
    end

    M.state.avgTreadTemp = sumTread * 0.25
    M.state.avgCarcassTemp = sumCarcass * 0.25
    M.state.avgMuScale = sumMuScale * 0.25
    M.state.avgFyAbs = sumFyAbs * 0.25
    M.state.avgSliding = sumSliding * 0.25
    M.state.maxTreadTemp = maxTread
    M.state.minMuScale = minMuScale
    M.state.maxSliding = maxSliding
    M.state.activeCount = activeCount
end

function M.update(dt, car, runtime)
    M.state.updateCount =
        (M.state.updateCount or 0)
        +
        1

    dt = safeNumber(dt, 0.0)

    if dt <= 0.0 then
        M.state.status = "BAD DT"
        exportState()
        return
    end

    dt =
        clamp(
            dt,
            M.params.minDt,
            M.params.maxDt
        )

    updateDebugGate(dt)
    resetLinks()

    car =
        car
        or
        safeGetCar()

    local wheels = safeGetWheels(car)

    if not car then
        M.state.status = "STORE ONLY"
        M.state.wheelsValid = false
    elseif not wheels then
        M.state.status = "STORE ONLY"
        M.state.wheelsValid = false
    else
        M.state.status = "RUNNING"
        M.state.wheelsValid = true
    end

    local speedMS =
        readCarSpeedMS(
            car
        )

    local roadTemp =
        readRoadTemp()

    local ambientTemp =
        readAmbientTemp()

    for i = 0, 3 do
        local wheel =
            getWheel(
                car,
                i
            )

        updateOneWheel(
            i,
            wheel,
            speedMS,
            roadTemp,
            ambientTemp,
            dt
        )
    end

    updateAverages()
    exportState()
end

--============================================================
-- Public API
--============================================================

function M.getMuScale(index)
    return M.state.muScale[index] or 1.0
end

function M.getMu(index)
    return M.state.mu[index] or 1.0
end

function M.getFy(index)
    return M.state.fy[index] or 0.0
end

function M.getTreadTemp(index)
    return M.state.treadTemp[index] or 30.0
end

function M.getCarcassTemp(index)
    return M.state.carcassTemp[index] or 45.0
end

function M.getSliding(index)
    return M.state.sliding[index] or 0.0
end

function M.getState(index)
    if index == nil then
        return M.state
    end

    return M.state.wheels[index]
end

function M.debugStr(index)
    if index ~= nil then
        local i =
            tonumber(index)
            or
            0

        return string.format(
            "RF2 Tire %d / Tread %.1f / Carcass %.1f\n" ..
            "Mu %.3f Scale %.3f / Fy %.0f / Slide %.2f\n" ..
            "Alpha %.3f / SR %.3f / Vslip %.2f / Xi %.2f",
            i,
            M.state.treadTemp[i] or 0.0,
            M.state.carcassTemp[i] or 0.0,
            M.state.mu[i] or 0.0,
            M.state.muScale[i] or 1.0,
            M.state.fy[i] or 0.0,
            M.state.sliding[i] or 0.0,
            M.state.alpha[i] or 0.0,
            M.state.slipRatio[i] or 0.0,
            M.state.slipVelocity[i] or 0.0,
            M.state.brushXi[i] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "Tread %.1f %.1f %.1f %.1f / Avg %.1f\n" ..
        "MuScale %.3f %.3f %.3f %.3f / Avg %.3f\n" ..
        "Fy %.0f %.0f %.0f %.0f / Slide %.2f\n" ..
        "Links TS:%s TC:%s CT:%s MEM:%s HOP:%s CAR:%s TD:%s RD:%s LP:%s UC:%s",
        tostring(M.state.status),
        M.state.updateCount or 0,
        M.state.wheelsValid and "OK" or "NIL",

        M.state.treadTemp[0] or 0.0,
        M.state.treadTemp[1] or 0.0,
        M.state.treadTemp[2] or 0.0,
        M.state.treadTemp[3] or 0.0,
        M.state.avgTreadTemp or 0.0,

        M.state.muScale[0] or 1.0,
        M.state.muScale[1] or 1.0,
        M.state.muScale[2] or 1.0,
        M.state.muScale[3] or 1.0,
        M.state.avgMuScale or 1.0,

        M.state.fy[0] or 0.0,
        M.state.fy[1] or 0.0,
        M.state.fy[2] or 0.0,
        M.state.fy[3] or 0.0,
        M.state.avgSliding or 0.0,

        M.state.tireStateLinked and "OK" or "NIL",
        M.state.complianceLinked and "OK" or "NIL",
        M.state.contactLinked and "OK" or "NIL",
        M.state.memoryLinked and "OK" or "NIL",
        M.state.hopLinked and "OK" or "NIL",
        M.state.carcassLinked and "OK" or "NIL",
        M.state.dynamicsLinked and "OK" or "NIL",
        M.state.roadLinked and "OK" or "NIL",
        M.state.loadPathLinked and "OK" or "NIL",
        M.state.ultraChassisLinked and "OK" or "NIL"
    )
end

return M
