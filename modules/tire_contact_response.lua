---@diagnostic disable: undefined-global

--============================================================
-- tire_contact_response.lua
-- ACNextGen V1.1.5 Stable
-- Tire Contact Response Layer
--============================================================

local M = {}

local WHEEL_NAMES = { [0] = "FL", [1] = "FR", [2] = "RL", [3] = "RR" }

M.params = {
    responseTau = 0.055,
    lossTau = 0.070,
    energyTau = 0.080,
    relaxTau = 0.075,
    complianceTau = 0.075,

    contactLossGain = 0.65,
    gripLossGain = 0.85,
    energyPushGain = 0.75,

    suspInputMin = 0.55,
    suspInputMax = 1.25,

    relaxMin = 0.75,
    relaxMax = 1.35,

    complianceGain = 0.65,

    loadRef = 3200.0,
    loadInfluence = 0.20,

    hopSuspPushGain = 0.16,
    memoryRelaxGain = 0.10,
    armComplianceGain = 0.08,
    driveRearEnergyGain = 0.08,
    lsdRearRelaxGain = 0.06,

    carcassDelayLossGain = 0.14,
    carcassReturnGripGain = 0.08,
    carcassHysteresisRelaxGain = 0.08,
    carcassSupportGain = 0.12,

    recoveryGripGain = 0.10,
    recoveryLossCutGain = 0.08,
    snapLossGain = 0.16,
    dirtyReturnLossGain = 0.08,

    roadShockEnergyGain = 0.10,
    roadSurfaceLossGain = 0.08,
    loadPathLossGain = 0.10,
    thermalLossGain = 0.08,

    tireComplianceRelaxGain = 0.08,
    tireDeflectionComplianceGain = 0.10,

    minQuality = 0.0,
    maxQuality = 1.20,
    minEffectiveGrip = 0.35,
    maxEffectiveGrip = 1.20,
    maxLoss = 1.0,
    maxEnergy = 1.0,
    maxComplianceInput = 1.0,

    minDt = 0.0001,
    maxDt = 0.100,
    debugStoreInterval = 0.25,
}

local state = {
    contactQuality = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    effectiveGrip  = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    contactLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    gripLoss    = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    energyPush  = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    suspInputScale  = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    tireRelaxScale  = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    complianceInput = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    load = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    memory = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    hop = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    armCamber = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    armToe = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    carcassSupport = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    carcassGripGate = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    carcassDelay = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    carcassReturn = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    carcassHysteresis = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    recovery = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    snapRisk = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    dirtyReturn = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    roadShock = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    surfaceLimit = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    pathLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    tireCompliance = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    tireDeflection = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    thermalStress = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    avgQuality = 1.0,
    avgGrip = 1.0,
    avgLoss = 0.0,
    avgEnergy = 0.0,
    avgSuspInput = 1.0,
    avgRelaxScale = 1.0,
    avgComplianceInput = 0.0,

    minQualityLive = 1.0,
    minGripLive = 1.0,
    maxLossLive = 0.0,
    maxEnergyLive = 0.0,
    maxComplianceLive = 0.0,

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,
    storeOnly = false,

    coreLinked = false,
    contactQualityLinked = false,
    memoryLinked = false,
    hopLinked = false,
    armLinked = false,
    drivetrainLinked = false,
    lsdLinked = false,
    carcassLinked = false,
    recoveryLinked = false,
    roadLinked = false,
    loadPathLinked = false,
    complianceLinked = false,
    thermalLinked = false,

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

local function clamp(v, mn, mx)
    v = safeNumber(v, mn)
    if v < mn then return mn end
    if v > mx then return mx end
    return v
end

local function abs(v)
    return math.abs(safeNumber(v, 0.0))
end

local function lerp(a, b, t)
    return a + (b - a) * clamp(t, 0.0, 1.0)
end

local function approachTau(current, target, dt, tau)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    dt = safeNumber(dt, 0.0)
    tau = safeNumber(tau, 0.001)
    if tau <= 0.0001 then return target end
    return lerp(current, target, 1.0 - math.exp(-dt / tau))
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
    return safeNumber(value, fallback or 0.0)
end

local function safeStore(key, value)
    if not ac or not ac.store then return end
    pcall(function()
        ac.store(key, value)
    end)
end

local function safeField(obj, key, fallback)
    if not obj then return fallback end
    local ok, value = pcall(function()
        return obj[key]
    end)
    if not ok or value == nil then return fallback end
    return value
end

local function safeGetCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function()
        return ac.getCar(0)
    end)
    if not ok then return nil end
    return car
end

local function getWheels(car)
    return safeField(car, "wheels", nil)
end

local function getWheel(car, index)
    local wheels = getWheels(car)
    if not wheels then return nil end
    return safeField(wheels, index, nil) or safeField(wheels, index + 1, nil)
end

local function loadFirst(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local v = safeLoadRaw(keys[i])
        if v ~= nil then
            return safeNumber(v, defaultValue or 0.0), keys[i]
        end
    end
    return defaultValue, nil
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

local function readWheelLoad(index, wheel)
    local direct = nil
    if wheel then
        direct = safeNumber(
            safeField(wheel, "load",
                safeField(wheel, "loadK", nil)),
            nil
        )
    end

    local integrated = safeLoadRaw("ngp_tc_load_" .. index)
    if integrated == nil then integrated = safeLoadRaw("ngp_cp_load_" .. index) end
    if integrated == nil then integrated = safeLoadRaw("ngp_wheel_load_" .. index) end
    if integrated == nil then integrated = safeLoadRaw("ngp_load_wheel_" .. index) end
    if integrated ~= nil then
        state.coreLinked = true
        integrated = safeNumber(integrated, direct or M.params.loadRef)
        if direct ~= nil then
            return direct * 0.65 + integrated * 0.35
        end
        return integrated
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        return M.params.loadRef + safeNumber(dlt, 0.0)
    end

    if direct ~= nil then
        return direct
    end

    return M.params.loadRef
end

local function readCore(index, wheel)
    local contact = safeLoadRaw("ngp_tc_contact_" .. index)
    local grip = safeLoadRaw("ngp_tc_grip_" .. index)
    local energy = safeLoadRaw("ngp_tc_energy_" .. index)
    local load = safeLoadRaw("ngp_tc_load_" .. index)

    if contact ~= nil or grip ~= nil or energy ~= nil or load ~= nil then
        state.coreLinked = true
    end

    if contact == nil then
        contact = safeLoadRaw("ngp_contact_quality_" .. index)
        if contact == nil then contact = safeLoadRaw("ngp_contact_raw_" .. index) end
        if contact ~= nil then state.contactQualityLinked = true end
    end

    if grip == nil then
        grip = safeLoadRaw("ngp_contact_grip_gate_" .. index)
        if grip == nil then grip = safeLoadRaw("ngp_carcass_grip_gate_" .. index) end
        if grip == nil then grip = safeLoadRaw("ngp_cp_grip_" .. index) end
        if grip ~= nil then state.contactQualityLinked = true end
    end

    if energy == nil then
        energy = safeLoadRaw("ngp_tire_slip_energy_" .. index)
        if energy == nil then energy = safeLoadRaw("ngp_cp_slip_energy_" .. index) end
        if energy == nil then energy = safeLoadRaw("ngp_carcass_heat_seed_" .. index) end
    end

    if load == nil then
        load = readWheelLoad(index, wheel)
    end

    return
        clamp(safeNumber(contact, 1.0), 0.0, 1.2),
        clamp(safeNumber(grip, 1.0), 0.0, 1.25),
        clamp(safeNumber(energy, 0.0), 0.0, 1.0),
        safeNumber(load, M.params.loadRef)
end

local function readMemory(index)
    local mem = safeLoadRaw("ngp_tire_memory_" .. index)
    if mem == nil then mem = safeLoadRaw("ngp_rubber_memory_" .. index) end
    if mem == nil then mem = safeLoadRaw("ngp_memory_" .. index) end
    if mem == nil then mem = safeLoadRaw("ngp_slide_memory_" .. index) end

    if mem ~= nil then state.memoryLinked = true end
    state.memory[index] = clamp(safeNumber(mem, 0.0), 0.0, 1.0)
end

local function readHop(index)
    local hop = safeLoadRaw("ngp_tirehop_energy_" .. index)
    if hop == nil then hop = safeLoadRaw("ngp_tire_hop_energy_" .. index) end
    if hop == nil then hop = safeLoadRaw("ngp_tire_hop_" .. index) end

    if hop ~= nil then state.hopLinked = true end
    state.hop[index] = clamp(safeNumber(hop, 0.0), 0.0, 1.0)
end

local function readArm(index)
    local camber = safeLoadRaw("ngp_control_arm_camber_" .. index)
    if camber == nil then camber = safeLoadRaw("ngp_arm_camber_" .. index) end
    if camber == nil then camber = safeLoadRaw("ngp_compliance_virtual_camber_" .. index) end

    local toe = safeLoadRaw("ngp_control_arm_toe_" .. index)
    if toe == nil then toe = safeLoadRaw("ngp_arm_toe_" .. index) end
    if toe == nil then toe = safeLoadRaw("ngp_compliance_virtual_toe_" .. index) end

    if camber ~= nil or toe ~= nil then state.armLinked = true end
    state.armCamber[index] = safeNumber(camber, 0.0)
    state.armToe[index] = safeNumber(toe, 0.0)
end

local function readCarcass(index)
    local support = safeLoadRaw("ngp_carcass_support_" .. index)
    local gate = safeLoadRaw("ngp_carcass_grip_gate_" .. index)
    local delay = safeLoadRaw("ngp_tire_contact_delay_" .. index)
    if delay == nil then delay = safeLoadRaw("ngp_contact_delay_" .. index) end
    local ret = safeLoadRaw("ngp_tire_return_force_" .. index)
    if ret == nil then ret = safeLoadRaw("ngp_return_force_" .. index) end
    local hyst = safeLoadRaw("ngp_carcass_hysteresis_" .. index)

    if support ~= nil or gate ~= nil or delay ~= nil or ret ~= nil or hyst ~= nil then
        state.carcassLinked = true
    end

    state.carcassSupport[index] = clamp(safeNumber(support, 1.0), 0.0, 1.2)
    state.carcassGripGate[index] = clamp(safeNumber(gate, state.carcassSupport[index]), 0.0, 1.2)
    state.carcassDelay[index] = clamp(safeNumber(delay, 0.0), 0.0, 1.0)
    state.carcassReturn[index] = clamp(safeNumber(ret, 0.0), 0.0, 1.0)
    state.carcassHysteresis[index] = clamp(safeNumber(hyst, 0.0), 0.0, 1.0)
end

local function readRecovery(index)
    local rec = safeLoadRaw("ngp_slip_recovery_rate_" .. index)
    if rec == nil then rec = safeLoadRaw("ngp_recovery_rate_" .. index) end
    local snap = safeLoadRaw("ngp_slip_snap_risk_" .. index)
    if snap == nil then snap = safeLoadRaw("ngp_snap_risk_" .. index) end
    local dirty = safeLoadRaw("ngp_slip_dirty_return_" .. index)
    if dirty == nil then dirty = safeLoadRaw("ngp_dirty_return_" .. index) end

    if rec ~= nil or snap ~= nil or dirty ~= nil then
        state.recoveryLinked = true
    end

    state.recovery[index] = clamp(safeNumber(rec, 0.0), 0.0, 1.0)
    state.snapRisk[index] = clamp(safeNumber(snap, 0.0), 0.0, 1.0)
    state.dirtyReturn[index] = clamp(safeNumber(dirty, 0.0), 0.0, 1.0)
end

local function readRoadAndPath(index)
    local shock = safeLoadRaw("ngp_road_shock_" .. index)
    if shock == nil then shock = safeLoadRaw("ngp_rii_shock_" .. index) end
    if shock == nil then shock = safeLoadRaw("ngp_road_impact_" .. index) end

    local surface = safeLoadRaw("ngp_road_surface_limit_" .. index)
    if surface == nil then surface = safeLoadRaw("ngp_rii_surface_limit_" .. index) end

    local pathLoss = safeLoadRaw("ngp_load_path_loss_" .. index)
    if pathLoss == nil then pathLoss = safeLoadRaw("ngp_road_path_loss_" .. index) end
    if pathLoss == nil then pathLoss = safeLoadRaw("ngp_rii_path_loss_" .. index) end

    if shock ~= nil or surface ~= nil then state.roadLinked = true end
    if pathLoss ~= nil then state.loadPathLinked = true end

    state.roadShock[index] = clamp(safeNumber(shock, 0.0), 0.0, 1.0)
    state.surfaceLimit[index] = clamp(safeNumber(surface, 0.0), 0.0, 1.0)
    state.pathLoss[index] = clamp(safeNumber(pathLoss, 0.0), 0.0, 1.0)
end

local function readCompliance(index)
    local comp = safeLoadRaw("ngp_tire_compliance_" .. index)
    if comp == nil then comp = safeLoadRaw("ngp_tcomp_compliance_" .. index) end

    local def = safeLoadRaw("ngp_tire_deflection_" .. index)
    if def == nil then def = safeLoadRaw("ngp_tcomp_deflection_" .. index) end

    if comp ~= nil or def ~= nil then state.complianceLinked = true end
    state.tireCompliance[index] = clamp(safeNumber(comp, 1.0), 0.35, 1.25)
    state.tireDeflection[index] = clamp(safeNumber(def, 0.0), 0.0, 1.0)
end

local function readThermal(index)
    local thermal = safeLoadRaw("ngp_slip_thermal_memory_" .. index)
    if thermal == nil then thermal = safeLoadRaw("ngp_tire_memory_thermal_" .. index) end
    if thermal == nil then thermal = safeLoadRaw("ngp_thermal_stress_" .. index) end

    if thermal == nil then
        local globalStress = safeLoadRaw("ngp_thermal_stress")
        if globalStress == nil then globalStress = safeLoadRaw("ngp_virtual_thermal_stress") end
        thermal = globalStress
    end

    if thermal ~= nil then state.thermalLinked = true end
    state.thermalStress[index] = clamp(safeNumber(thermal, 0.0), 0.0, 1.0)
end

local function readDriveline(index)
    if index < 2 then return 0.0, 0.0 end

    local drive = safeLoadRaw("ngp_drive_torque")
    if drive == nil then drive = safeLoadRaw("ngp_drivetrain_torque") end
    if drive == nil then drive = safeLoadRaw("ngp_dt_torque") end

    local lsd = safeLoadRaw("ngp_lsd_lock")
    if lsd == nil then lsd = safeLoadRaw("ngp_diff_lock") end

    if drive ~= nil then state.drivetrainLinked = true end
    if lsd ~= nil then state.lsdLinked = true end

    return abs(drive), clamp(safeNumber(lsd, 0.0), 0.0, 1.0)
end

local function readExternal(index)
    readMemory(index)
    readHop(index)
    readArm(index)
    readCarcass(index)
    readRecovery(index)
    readRoadAndPath(index)
    readCompliance(index)
    readThermal(index)
end

local function updateWheel(index, dt, wheel)
    local contact, grip, energy, load = readCore(index, wheel)
    readExternal(index)

    local driveTorque, lsdLock = readDriveline(index)

    state.load[index] = load

    local loadInfluence = clamp((load / math.max(M.params.loadRef, 1.0)) - 1.0, -1.0, 1.0) * M.params.loadInfluence
    local support = state.carcassSupport[index]
    local gripGate = state.carcassGripGate[index]

    local targetQuality = clamp(
        contact +
        loadInfluence +
        (support - 1.0) * M.params.carcassSupportGain -
        state.surfaceLimit[index] * M.params.roadSurfaceLossGain -
        state.pathLoss[index] * M.params.loadPathLossGain -
        state.thermalStress[index] * M.params.thermalLossGain,
        M.params.minQuality,
        M.params.maxQuality
    )

    local targetLoss = clamp(
        (1.0 - contact) * M.params.contactLossGain +
        (1.0 - grip) * M.params.gripLossGain +
        state.hop[index] * 0.08 +
        state.carcassDelay[index] * M.params.carcassDelayLossGain +
        state.snapRisk[index] * M.params.snapLossGain +
        state.dirtyReturn[index] * M.params.dirtyReturnLossGain +
        state.surfaceLimit[index] * M.params.roadSurfaceLossGain +
        state.pathLoss[index] * M.params.loadPathLossGain +
        state.thermalStress[index] * M.params.thermalLossGain -
        state.recovery[index] * M.params.recoveryLossCutGain,
        0.0,
        M.params.maxLoss
    )

    local targetGrip = clamp(
        grip *
        (1.0 - targetLoss * 0.35) *
        (0.84 + gripGate * 0.16) *
        (1.0 + state.carcassReturn[index] * M.params.carcassReturnGripGain) *
        (1.0 + state.recovery[index] * M.params.recoveryGripGain) *
        (1.0 - state.memory[index] * 0.06),
        M.params.minEffectiveGrip,
        M.params.maxEffectiveGrip
    )

    local targetEnergy = clamp(
        energy * M.params.energyPushGain +
        state.hop[index] * M.params.hopSuspPushGain +
        state.roadShock[index] * M.params.roadShockEnergyGain +
        state.carcassHysteresis[index] * 0.08 +
        driveTorque * M.params.driveRearEnergyGain,
        0.0,
        M.params.maxEnergy
    )

    local targetSusp = clamp(
        1.0 +
        targetEnergy * 0.35 +
        state.roadShock[index] * 0.08 -
        targetLoss * 0.22,
        M.params.suspInputMin,
        M.params.suspInputMax
    )

    local targetRelax = clamp(
        1.0 +
        targetLoss * 0.35 +
        state.memory[index] * M.params.memoryRelaxGain +
        state.carcassHysteresis[index] * M.params.carcassHysteresisRelaxGain +
        (1.0 - state.tireCompliance[index]) * M.params.tireComplianceRelaxGain +
        lsdLock * M.params.lsdRearRelaxGain,
        M.params.relaxMin,
        M.params.relaxMax
    )

    local targetCompliance = clamp(
        targetLoss * M.params.complianceGain +
        abs(state.armCamber[index]) * M.params.armComplianceGain +
        abs(state.armToe[index]) * M.params.armComplianceGain +
        state.tireDeflection[index] * M.params.tireDeflectionComplianceGain +
        state.carcassDelay[index] * 0.12 +
        state.pathLoss[index] * 0.08,
        0.0,
        M.params.maxComplianceInput
    )

    state.contactQuality[index] = approachTau(state.contactQuality[index], targetQuality, dt, M.params.responseTau)
    state.effectiveGrip[index] = approachTau(state.effectiveGrip[index], targetGrip, dt, M.params.responseTau)
    state.contactLoss[index] = approachTau(state.contactLoss[index], targetLoss, dt, M.params.lossTau)
    state.gripLoss[index] = clamp(1.0 - state.effectiveGrip[index], 0.0, 1.0)
    state.energyPush[index] = approachTau(state.energyPush[index], targetEnergy, dt, M.params.energyTau)
    state.suspInputScale[index] = approachTau(state.suspInputScale[index], targetSusp, dt, M.params.responseTau)
    state.tireRelaxScale[index] = approachTau(state.tireRelaxScale[index], targetRelax, dt, M.params.relaxTau)
    state.complianceInput[index] = approachTau(state.complianceInput[index], targetCompliance, dt, M.params.complianceTau)
end

local function exportWheel(i)
    safeStore("ngp_tcr_quality_" .. i, state.contactQuality[i])
    safeStore("ngp_tcr_effective_grip_" .. i, state.effectiveGrip[i])
    safeStore("ngp_tcr_contact_loss_" .. i, state.contactLoss[i])
    safeStore("ngp_tcr_grip_loss_" .. i, state.gripLoss[i])
    safeStore("ngp_tcr_energy_push_" .. i, state.energyPush[i])
    safeStore("ngp_tcr_susp_input_" .. i, state.suspInputScale[i])
    safeStore("ngp_tcr_relax_scale_" .. i, state.tireRelaxScale[i])
    safeStore("ngp_tcr_compliance_input_" .. i, state.complianceInput[i])

    safeStore("ngp_tire_contact_quality_" .. i, state.contactQuality[i])
    safeStore("ngp_tire_effective_grip_" .. i, state.effectiveGrip[i])
    safeStore("ngp_tire_contact_loss_" .. i, state.contactLoss[i])
    safeStore("ngp_tire_response_scale_" .. i, state.tireRelaxScale[i])
    safeStore("ngp_susp_contact_input_" .. i, state.suspInputScale[i])
    safeStore("ngp_susp_contact_scale_" .. i, state.suspInputScale[i])
    safeStore("ngp_compliance_contact_input_" .. i, state.complianceInput[i])

    safeStore("ngp_tcr_load_" .. i, state.load[i])
    safeStore("ngp_tcr_memory_" .. i, state.memory[i])
    safeStore("ngp_tcr_hop_" .. i, state.hop[i])

    safeStore("ngp_tresp_quality_" .. i, state.contactQuality[i])
    safeStore("ngp_tresp_grip_" .. i, state.effectiveGrip[i])
    safeStore("ngp_tresp_loss_" .. i, state.contactLoss[i])
    safeStore("ngp_tresp_energy_" .. i, state.energyPush[i])
    safeStore("ngp_tresp_relax_" .. i, state.tireRelaxScale[i])
    safeStore("ngp_tresp_compliance_" .. i, state.complianceInput[i])

    if not state.debugStoreNow then return end

    safeStore("ngp_tcr_carcass_support_" .. i, state.carcassSupport[i])
    safeStore("ngp_tcr_carcass_gate_" .. i, state.carcassGripGate[i])
    safeStore("ngp_tcr_carcass_delay_" .. i, state.carcassDelay[i])
    safeStore("ngp_tcr_carcass_return_" .. i, state.carcassReturn[i])
    safeStore("ngp_tcr_recovery_" .. i, state.recovery[i])
    safeStore("ngp_tcr_snap_" .. i, state.snapRisk[i])
    safeStore("ngp_tcr_road_shock_" .. i, state.roadShock[i])
    safeStore("ngp_tcr_path_loss_" .. i, state.pathLoss[i])
    safeStore("ngp_tcr_thermal_" .. i, state.thermalStress[i])
end

local function exportGlobal()
    safeStore("ngp_tcr_status", state.status)
    safeStore("ngp_tcr_update_count", state.updateCount)
    safeStore("ngp_tcr_wheels_valid", state.wheelsValid and 1 or 0)
    safeStore("ngp_tcr_store_only", state.storeOnly and 1 or 0)

    safeStore("ngp_tire_avg_quality", state.avgQuality)
    safeStore("ngp_tire_avg_effective_grip", state.avgGrip)
    safeStore("ngp_tire_avg_loss", state.avgLoss)
    safeStore("ngp_tire_avg_response_energy", state.avgEnergy)

    safeStore("ngp_tcr_avg_susp_input", state.avgSuspInput)
    safeStore("ngp_tcr_avg_relax_scale", state.avgRelaxScale)
    safeStore("ngp_tcr_avg_compliance_input", state.avgComplianceInput)
    safeStore("ngp_tcr_min_quality", state.minQualityLive)
    safeStore("ngp_tcr_min_grip", state.minGripLive)
    safeStore("ngp_tcr_max_loss", state.maxLossLive)
    safeStore("ngp_tcr_max_energy", state.maxEnergyLive)
    safeStore("ngp_tcr_max_compliance", state.maxComplianceLive)

    safeStore("ngp_tresp_avg_quality", state.avgQuality)
    safeStore("ngp_tresp_avg_grip", state.avgGrip)
    safeStore("ngp_tresp_avg_loss", state.avgLoss)
    safeStore("ngp_tresp_avg_energy", state.avgEnergy)

    if not state.debugStoreNow then return end

    safeStore("ngp_tcr_core_linked", state.coreLinked and 1 or 0)
    safeStore("ngp_tcr_contact_quality_linked", state.contactQualityLinked and 1 or 0)
    safeStore("ngp_tcr_memory_linked", state.memoryLinked and 1 or 0)
    safeStore("ngp_tcr_hop_linked", state.hopLinked and 1 or 0)
    safeStore("ngp_tcr_arm_linked", state.armLinked and 1 or 0)
    safeStore("ngp_tcr_drivetrain_linked", state.drivetrainLinked and 1 or 0)
    safeStore("ngp_tcr_lsd_linked", state.lsdLinked and 1 or 0)
    safeStore("ngp_tcr_carcass_linked", state.carcassLinked and 1 or 0)
    safeStore("ngp_tcr_recovery_linked", state.recoveryLinked and 1 or 0)
    safeStore("ngp_tcr_road_linked", state.roadLinked and 1 or 0)
    safeStore("ngp_tcr_load_path_linked", state.loadPathLinked and 1 or 0)
    safeStore("ngp_tcr_compliance_linked", state.complianceLinked and 1 or 0)
    safeStore("ngp_tcr_thermal_linked", state.thermalLinked and 1 or 0)
end

local function exportState()
    for i = 0, 3 do
        exportWheel(i)
    end
    exportGlobal()
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

    if not car then
        car = safeGetCar()
    end

    local wheels = getWheels(car)
    state.wheelsValid = wheels ~= nil
    state.storeOnly = not state.wheelsValid

    state.coreLinked = false
    state.contactQualityLinked = false
    state.memoryLinked = false
    state.hopLinked = false
    state.armLinked = false
    state.drivetrainLinked = false
    state.lsdLinked = false
    state.carcassLinked = false
    state.recoveryLinked = false
    state.roadLinked = false
    state.loadPathLinked = false
    state.complianceLinked = false
    state.thermalLinked = false

    local sumQ = 0.0
    local sumG = 0.0
    local sumL = 0.0
    local sumE = 0.0
    local sumS = 0.0
    local sumR = 0.0
    local sumC = 0.0

    local minQ = 1.20
    local minG = 1.20
    local maxL = 0.0
    local maxE = 0.0
    local maxC = 0.0

    for i = 0, 3 do
        updateWheel(i, dt, getWheel(car, i))

        sumQ = sumQ + state.contactQuality[i]
        sumG = sumG + state.effectiveGrip[i]
        sumL = sumL + state.contactLoss[i]
        sumE = sumE + state.energyPush[i]
        sumS = sumS + state.suspInputScale[i]
        sumR = sumR + state.tireRelaxScale[i]
        sumC = sumC + state.complianceInput[i]

        minQ = math.min(minQ, state.contactQuality[i])
        minG = math.min(minG, state.effectiveGrip[i])
        maxL = math.max(maxL, state.contactLoss[i])
        maxE = math.max(maxE, state.energyPush[i])
        maxC = math.max(maxC, state.complianceInput[i])

        exportWheel(i)
    end

    state.avgQuality = sumQ * 0.25
    state.avgGrip = sumG * 0.25
    state.avgLoss = sumL * 0.25
    state.avgEnergy = sumE * 0.25
    state.avgSuspInput = sumS * 0.25
    state.avgRelaxScale = sumR * 0.25
    state.avgComplianceInput = sumC * 0.25

    state.minQualityLive = minQ
    state.minGripLive = minG
    state.maxLossLive = maxL
    state.maxEnergyLive = maxE
    state.maxComplianceLive = maxC

    if state.wheelsValid then
        state.status = "RUNNING"
    elseif car then
        state.status = "NO WHEELS"
    else
        state.status = "STORE ONLY"
    end

    exportGlobal()
end

function M.getQuality(index)
    if index == nil then return state.avgQuality or 1.0 end
    return state.contactQuality[index] or 1.0
end

function M.getGrip(index)
    if index == nil then return state.avgGrip or 1.0 end
    return state.effectiveGrip[index] or 1.0
end

function M.getLoss(index)
    if index == nil then return state.avgLoss or 0.0 end
    return state.contactLoss[index] or 0.0
end

function M.getEnergy(index)
    if index == nil then return state.avgEnergy or 0.0 end
    return state.energyPush[index] or 0.0
end

function M.getSuspInput(index)
    if index == nil then return state.avgSuspInput or 1.0 end
    return state.suspInputScale[index] or 1.0
end

function M.getRelaxScale(index)
    if index == nil then return state.avgRelaxScale or 1.0 end
    return state.tireRelaxScale[index] or 1.0
end

function M.getComplianceInput(index)
    if index == nil then return state.avgComplianceInput or 0.0 end
    return state.complianceInput[index] or 0.0
end

function M.getState(index)
    if index == nil then return state end
    return {
        quality = state.contactQuality[index] or 1.0,
        grip = state.effectiveGrip[index] or 1.0,
        loss = state.contactLoss[index] or 0.0,
        energy = state.energyPush[index] or 0.0,
        suspInput = state.suspInputScale[index] or 1.0,
        relaxScale = state.tireRelaxScale[index] or 1.0,
        complianceInput = state.complianceInput[index] or 0.0,
        load = state.load[index] or 0.0,
    }
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0
        return string.format(
            "%s Q %.2f Grip %.2f Loss %.2f Energy %.2f\n" ..
            "Susp %.2f Relax %.2f Comp %.2f Load %.0f\n" ..
            "Carc Sup %.2f Delay %.2f Ret %.2f Road %.2f Path %.2f",
            WHEEL_NAMES[i] or tostring(i),
            state.contactQuality[i] or 1.0,
            state.effectiveGrip[i] or 1.0,
            state.contactLoss[i] or 0.0,
            state.energyPush[i] or 0.0,
            state.suspInputScale[i] or 1.0,
            state.tireRelaxScale[i] or 1.0,
            state.complianceInput[i] or 0.0,
            state.load[i] or 0.0,
            state.carcassSupport[i] or 1.0,
            state.carcassDelay[i] or 0.0,
            state.carcassReturn[i] or 0.0,
            state.roadShock[i] or 0.0,
            state.pathLoss[i] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s / StoreOnly %s\n" ..
        "Quality %.2f Grip %.2f Loss %.2f Energy %.2f\n" ..
        "Susp %.2f Relax %.2f Comp %.2f\n" ..
        "Links Core:%s CQ:%s Carc:%s Rec:%s Road:%s LP:%s Comp:%s Therm:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",
        state.storeOnly and "YES" or "NO",
        state.avgQuality or 1.0,
        state.avgGrip or 1.0,
        state.avgLoss or 0.0,
        state.avgEnergy or 0.0,
        state.avgSuspInput or 1.0,
        state.avgRelaxScale or 1.0,
        state.avgComplianceInput or 0.0,
        state.coreLinked and "OK" or "NIL",
        state.contactQualityLinked and "OK" or "NIL",
        state.carcassLinked and "OK" or "NIL",
        state.recoveryLinked and "OK" or "NIL",
        state.roadLinked and "OK" or "NIL",
        state.loadPathLinked and "OK" or "NIL",
        state.complianceLinked and "OK" or "NIL",
        state.thermalLinked and "OK" or "NIL"
    )
end

return M
