---@diagnostic disable: undefined-global

--============================================================
-- ACNextGen
-- control_arm.lua
-- Phase 9.2 / V1.1 stable
-- Virtual control arm and bushing compliance bridge
--============================================================

local M = {}

local FL, FR, RL, RR = 0, 1, 2, 3

M.params = {
    frontTau = 0.070,
    rearTau  = 0.110,

    frontFlex = 0.35,
    rearFlex  = 0.55,

    maxDeflection = 0.25,

    yawGain = 1.00,
    armCamberGain = 0.85,
    armToeGain    = 0.95,

    loadYawGain = 0.000035,
    contactYawGain = 0.055,

    bodySteerGain = 0.16,
    flexYawGain = 0.18,
    flexRollGain = 0.05,
    flexEnergyGain = 0.10,
    flexReleaseReturnGain = 0.12,

    complianceToeGain = 0.22,
    complianceForceLeakGain = 0.08,
    complianceLoadPathGain = 0.08,

    bodyReadInterval = 0.05,
    rootReadInterval = 0.05,

    bodyFlexTauGain = 0.30,
    bodyTorsionRearTauGain = 0.24,
    bodyDampingTauGain = 0.10,
    bodyStiffTauLoss = 0.08,

    bodyFlexDeflectionGain = 0.16,
    bodyTorsionRearDeflectionGain = 0.12,
    bodyStiffDeflectionLoss = 0.06,

    minBodyTauScale = 0.84,
    maxBodyTauScale = 1.30,

    minBodyDeflectionScale = 0.86,
    maxBodyDeflectionScale = 1.20,

    loadRef = 3000.0,
    maxGeometryOutput = 0.12,
    debugStoreInterval = 0.25,

    minDt = 0.00005,
    maxDt = 0.050,
}

local state = {
    rawYaw = 0.0,
    rawInput = 0.0,

    front = 0.0,
    rear = 0.0,
    frontDef = 0.0,
    rearDef = 0.0,
    total = 0.0,

    armCamber = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    armToe    = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    complianceToe = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    complianceCamber = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    forceLeak = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadPathLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    load = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    contact = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },

    frontCamberAvg = 0.0,
    rearCamberAvg = 0.0,
    frontToeAvg = 0.0,
    rearToeAvg = 0.0,

    frontLoadDiff = 0.0,
    rearLoadDiff = 0.0,
    frontContactLoss = 0.0,
    rearContactLoss = 0.0,

    bodySteer = 0.0,
    flexYaw = 0.0,
    flexRoll = 0.0,
    flexEnergy = 0.0,
    flexRelease = 0.0,

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyDampingFactor = 1.0,

    bodyFrontTauScale = 1.0,
    bodyRearTauScale = 1.0,
    bodyFrontDeflectionScale = 1.0,
    bodyRearDeflectionScale = 1.0,

    camberOutput = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    toeOutput = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    armLinked = false,
    loadLinked = false,
    contactLinked = false,
    flexLinked = false,
    bodyLinked = false,
    complianceLinked = false,

    bodyReadTimer = 999.0,
    rootReadTimer = 999.0,
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

local function clamp(v, minValue, maxValue)
    v = safeNumber(v, minValue or 0.0)
    if v < minValue then return minValue end
    if v > maxValue then return maxValue end
    return v
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

local function safeGetCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function() return ac.getCar(0) end)
    if not ok then return nil end
    return car
end

local function lowPass(current, target, tau, dt)
    current = safeNumber(current, 0.0)
    target = safeNumber(target, 0.0)
    tau = safeNumber(tau, 0.001)
    dt = safeNumber(dt, 0.0)
    if tau <= 0.0001 then return target end
    local alpha = clamp(dt / (tau + dt), 0.0, 1.0)
    return current + (target - current) * alpha
end

local function abs(v)
    v = safeNumber(v, 0.0)
    return v < 0.0 and -v or v
end

local function updateDebugGate(dt)
    state.debugStoreTimer = (state.debugStoreTimer or 0.0) + (dt or 0.0)
    state.debugStoreNow = state.debugStoreTimer >= M.params.debugStoreInterval
    if state.debugStoreNow then
        state.debugStoreTimer = 0.0
    end
end

local function getYawInput(car)
    local av = safeField(car, "localAngularVelocity", nil)
    if not av then return 0.0 end
    local yawRate = safeNumber(av.y, 0.0)
    local speedKmh = safeNumber(safeField(car, "speedKmh", 0.0), 0.0)
    return yawRate * (speedKmh / 3.6)
end

local function readArmCompliance()
    state.armLinked = false
    state.complianceLinked = false

    for i = 0, 3 do
        local camber, camberKey = safeLoadAlt(0.0,
            "ngp_arm_camber_" .. i,
            "ngp_compliance_arm_camber_" .. i,
            "ngp_ultra_camber_" .. i,
            "ngp_uc_camber_" .. i
        )

        local toe, toeKey = safeLoadAlt(0.0,
            "ngp_arm_toe_" .. i,
            "ngp_compliance_arm_toe_" .. i,
            "ngp_ultra_toe_" .. i,
            "ngp_uc_toe_" .. i
        )

        if camberKey or toeKey then
            state.armLinked = true
        end

        local cToe, cToeKey = safeLoadAlt(0.0,
            "ngp_compliance_virtual_toe_" .. i,
            "ngp_virtual_toe_" .. i,
            "ngp_cs_toe_" .. i
        )

        local cCamber, cCamberKey = safeLoadAlt(0.0,
            "ngp_compliance_virtual_camber_" .. i,
            "ngp_virtual_camber_" .. i,
            "ngp_cs_camber_" .. i
        )

        local leak, leakKey = safeLoadAlt(0.0,
            "ngp_compliance_force_leak_" .. i,
            "ngp_force_path_loss_" .. i,
            "ngp_force_leak_" .. i
        )

        local pathLoss, pathKey = safeLoadAlt(0.0,
            "ngp_compliance_load_path_loss_" .. i,
            "ngp_load_path_loss_" .. i
        )

        if cToeKey or cCamberKey or leakKey or pathKey then
            state.complianceLinked = true
        end

        state.armCamber[i] = clamp(camber, -M.params.maxGeometryOutput, M.params.maxGeometryOutput)
        state.armToe[i] = clamp(toe, -M.params.maxGeometryOutput, M.params.maxGeometryOutput)
        state.complianceToe[i] = clamp(cToe, -M.params.maxGeometryOutput, M.params.maxGeometryOutput)
        state.complianceCamber[i] = clamp(cCamber, -M.params.maxGeometryOutput, M.params.maxGeometryOutput)
        state.forceLeak[i] = clamp(leak, 0.0, 1.0)
        state.loadPathLoss[i] = clamp(pathLoss, 0.0, 1.0)
    end

    state.frontCamberAvg = (state.armCamber[FL] + state.armCamber[FR]) * 0.5
    state.rearCamberAvg = (state.armCamber[RL] + state.armCamber[RR]) * 0.5
    state.frontToeAvg = (state.armToe[FL] + state.armToe[FR]) * 0.5
    state.rearToeAvg = (state.armToe[RL] + state.armToe[RR]) * 0.5
end

local function readWheelLoad(index)
    local value, key = safeLoadAlt(nil,
        "ngp_contact_load_" .. index,
        "ngp_tire_state_load_" .. index,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index,
        "ngp_sprung_load_" .. index,
        "ngp_load_path_wheel_load_" .. index
    )

    if key then
        state.loadLinked = true
        return clamp(value, 0.0, 15000.0)
    end

    local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
    if dlt ~= nil then
        state.loadLinked = true
        return clamp(M.params.loadRef + safeNumber(dlt, 0.0), 0.0, 15000.0)
    end

    return 0.0
end

local function readContact(index)
    local value, key = safeLoadAlt(nil,
        "ngp_contact_trust_" .. index,
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index,
        "ngp_tc_contact_" .. index
    )

    if key then
        state.contactLinked = true
        return clamp(value, 0.0, 1.2)
    end

    local loss = safeLoadRaw("ngp_contact_loss_" .. index) or safeLoadRaw("ngp_tire_contact_loss_" .. index) or safeLoadRaw("ngp_tcr_contact_loss_" .. index)
    if loss ~= nil then
        state.contactLinked = true
        return clamp(1.0 - safeNumber(loss, 0.0), 0.0, 1.2)
    end

    return 1.0
end

local function readLoadContact(dt)
    state.rootReadTimer = (state.rootReadTimer or 0.0) + (dt or 0.0)
    if state.rootReadTimer < M.params.rootReadInterval then
        return
    end
    state.rootReadTimer = 0.0

    state.loadLinked = false
    state.contactLinked = false

    for i = 0, 3 do
        state.load[i] = readWheelLoad(i)
        state.contact[i] = readContact(i)
    end

    state.frontLoadDiff = state.load[FR] - state.load[FL]
    state.rearLoadDiff = state.load[RR] - state.load[RL]

    state.frontContactLoss = clamp(1.0 - (state.contact[FL] + state.contact[FR]) * 0.5, 0.0, 1.0)
    state.rearContactLoss = clamp(1.0 - (state.contact[RL] + state.contact[RR]) * 0.5, 0.0, 1.0)
end

local function readChassisFlex()
    state.bodySteer = safeLoad("ngp_body_steer", 0.0)
    state.flexYaw = safeLoad("ngp_chassis_flex_yaw", safeLoad("ngp_flex_yaw", 0.0))
    state.flexRoll = safeLoad("ngp_chassis_flex_roll", safeLoad("ngp_flex_roll", 0.0))
    state.flexEnergy = safeLoad("ngp_chassis_flex_energy", safeLoad("ngp_flex_energy", 0.0))
    state.flexRelease = safeLoad("ngp_chassis_flex_release", safeLoad("ngp_flex_release", 0.0))

    state.flexLinked =
        safeLoadRaw("ngp_body_steer") ~= nil or
        safeLoadRaw("ngp_chassis_flex_yaw") ~= nil or
        safeLoadRaw("ngp_chassis_flex_energy") ~= nil
end

local function readBodyRigidity(dt)
    state.bodyReadTimer = (state.bodyReadTimer or 0.0) + (dt or 0.0)
    if state.bodyReadTimer < M.params.bodyReadInterval then
        return
    end
    state.bodyReadTimer = 0.0

    local rigidityRaw = safeLoadRaw("ngp_body_rigidity")
    local flexRaw = safeLoadRaw("ngp_body_flex_factor")
    local torsionRaw = safeLoadRaw("ngp_body_torsion_factor")
    local dampingRaw = safeLoadRaw("ngp_body_damping_factor")

    state.bodyLinked = rigidityRaw ~= nil or flexRaw ~= nil or torsionRaw ~= nil or dampingRaw ~= nil or safeLoad("ngp_body_rigidity_update_count", 0.0) > 0.0

    state.bodyRigidity = clamp(safeNumber(rigidityRaw, 1.0), 0.20, 1.35)
    state.bodyFlexFactor = clamp(safeNumber(flexRaw, 1.0), 0.25, 1.80)
    state.bodyTorsionFactor = clamp(safeNumber(torsionRaw, 1.0), 0.35, 2.00)
    state.bodyDampingFactor = clamp(safeNumber(dampingRaw, 1.0), 0.50, 1.80)
end

local function updateBodyScales(dt)
    local p = M.params

    if not state.bodyLinked then
        state.bodyFrontTauScale = lowPass(state.bodyFrontTauScale, 1.0, 0.08, dt)
        state.bodyRearTauScale = lowPass(state.bodyRearTauScale, 1.0, 0.08, dt)
        state.bodyFrontDeflectionScale = lowPass(state.bodyFrontDeflectionScale, 1.0, 0.08, dt)
        state.bodyRearDeflectionScale = lowPass(state.bodyRearDeflectionScale, 1.0, 0.08, dt)
        return
    end

    local softness = clamp(state.bodyFlexFactor - 1.0, 0.0, 0.80)
    local torsionSoft = clamp(state.bodyTorsionFactor - 1.0, 0.0, 1.0)
    local dampingExtra = clamp(state.bodyDampingFactor - 1.0, 0.0, 0.80)
    local stiff = clamp(state.bodyRigidity - 1.0, 0.0, 0.35)

    local frontTauTarget = clamp(1.0 + softness * p.bodyFlexTauGain + dampingExtra * p.bodyDampingTauGain - stiff * p.bodyStiffTauLoss, p.minBodyTauScale, p.maxBodyTauScale)
    local rearTauTarget = clamp(1.0 + softness * p.bodyFlexTauGain + torsionSoft * p.bodyTorsionRearTauGain + dampingExtra * p.bodyDampingTauGain - stiff * p.bodyStiffTauLoss, p.minBodyTauScale, p.maxBodyTauScale)

    local frontDefTarget = clamp(1.0 + softness * p.bodyFlexDeflectionGain - stiff * p.bodyStiffDeflectionLoss, p.minBodyDeflectionScale, p.maxBodyDeflectionScale)
    local rearDefTarget = clamp(1.0 + softness * p.bodyFlexDeflectionGain + torsionSoft * p.bodyTorsionRearDeflectionGain - stiff * p.bodyStiffDeflectionLoss, p.minBodyDeflectionScale, p.maxBodyDeflectionScale)

    state.bodyFrontTauScale = lowPass(state.bodyFrontTauScale, frontTauTarget, 0.08, dt)
    state.bodyRearTauScale = lowPass(state.bodyRearTauScale, rearTauTarget, 0.08, dt)
    state.bodyFrontDeflectionScale = lowPass(state.bodyFrontDeflectionScale, frontDefTarget, 0.08, dt)
    state.bodyRearDeflectionScale = lowPass(state.bodyRearDeflectionScale, rearDefTarget, 0.08, dt)
end

local function calculateControlArmInput(car)
    local p = M.params

    local yawInput = getYawInput(car) * p.yawGain
    state.rawYaw = yawInput

    local geometryInput =
        (state.frontToeAvg - state.rearToeAvg) * p.armToeGain +
        (state.frontCamberAvg - state.rearCamberAvg) * p.armCamberGain

    local loadInput = (state.frontLoadDiff * 0.45 + state.rearLoadDiff * 0.55) * p.loadYawGain
    local contactInput = (state.frontContactLoss * 0.40 + state.rearContactLoss * 0.60) * p.contactYawGain

    local bodyInput =
        state.bodySteer * p.bodySteerGain +
        state.flexYaw * p.flexYawGain +
        state.flexRoll * p.flexRollGain +
        state.flexEnergy * p.flexEnergyGain -
        state.flexRelease * p.flexReleaseReturnGain

    local frontCompliance = (abs(state.complianceToe[FL]) + abs(state.complianceToe[FR])) * 0.5
    local rearCompliance = (abs(state.complianceToe[RL]) + abs(state.complianceToe[RR])) * 0.5
    local frontLeak = (state.forceLeak[FL] + state.forceLeak[FR]) * 0.5
    local rearLoss = (state.loadPathLoss[RL] + state.loadPathLoss[RR]) * 0.5

    local complianceInput =
        (frontCompliance - rearCompliance) * p.complianceToeGain +
        frontLeak * p.complianceForceLeakGain +
        rearLoss * p.complianceLoadPathGain

    local total = yawInput + geometryInput + loadInput + contactInput + bodyInput + complianceInput
    state.rawInput = total

    return clamp(total, -p.maxDeflection, p.maxDeflection)
end

local function updateOutputs(dt, input)
    local p = M.params

    local frontTau = p.frontTau * (state.bodyFrontTauScale or 1.0)
    local rearTau = p.rearTau * (state.bodyRearTauScale or 1.0)

    state.front = lowPass(state.front, input, frontTau, dt)
    state.rear = lowPass(state.rear, state.front, rearTau, dt)

    state.front = clamp(state.front, -p.maxDeflection, p.maxDeflection)
    state.rear = clamp(state.rear, -p.maxDeflection, p.maxDeflection)

    state.frontDef = state.front * p.frontFlex * (state.bodyFrontDeflectionScale or 1.0)
    state.rearDef = state.rear * p.rearFlex * (state.bodyRearDeflectionScale or 1.0)
    state.total = state.frontDef + state.rearDef

    local frontGeometry = clamp(state.frontDef * 0.18, -p.maxGeometryOutput, p.maxGeometryOutput)
    local rearGeometry = clamp(state.rearDef * 0.22, -p.maxGeometryOutput, p.maxGeometryOutput)

    state.camberOutput[FL] = state.armCamber[FL] + state.complianceCamber[FL] + frontGeometry
    state.camberOutput[FR] = state.armCamber[FR] + state.complianceCamber[FR] - frontGeometry
    state.camberOutput[RL] = state.armCamber[RL] + state.complianceCamber[RL] + rearGeometry
    state.camberOutput[RR] = state.armCamber[RR] + state.complianceCamber[RR] - rearGeometry

    state.toeOutput[FL] = state.armToe[FL] + state.complianceToe[FL] + frontGeometry * 0.55
    state.toeOutput[FR] = state.armToe[FR] + state.complianceToe[FR] - frontGeometry * 0.55
    state.toeOutput[RL] = state.armToe[RL] + state.complianceToe[RL] + rearGeometry * 0.70
    state.toeOutput[RR] = state.armToe[RR] + state.complianceToe[RR] - rearGeometry * 0.70

    for i = 0, 3 do
        state.camberOutput[i] = clamp(state.camberOutput[i], -p.maxGeometryOutput, p.maxGeometryOutput)
        state.toeOutput[i] = clamp(state.toeOutput[i], -p.maxGeometryOutput, p.maxGeometryOutput)
    end
end

local function exportState()
    safeStore("ngp_arm_front", state.frontDef)
    safeStore("ngp_arm_rear", state.rearDef)
    safeStore("ngp_arm_total", state.total)
    safeStore("ngp_arm_raw_yaw", state.rawYaw)

    safeStore("ngp_control_arm_front", state.frontDef)
    safeStore("ngp_control_arm_rear", state.rearDef)
    safeStore("ngp_control_arm_total", state.total)
    safeStore("ngp_control_arm_raw_yaw", state.rawYaw)
    safeStore("ngp_control_arm_status", state.status)
    safeStore("ngp_control_arm_update_count", state.updateCount)
    safeStore("ngp_control_arm_wheels_valid", state.wheelsValid and 1 or 0)
    safeStore("ngp_control_arm_body_linked", state.bodyLinked and 1 or 0)

    for i = 0, 3 do
        safeStore("ngp_control_arm_camber_" .. i, state.camberOutput[i])
        safeStore("ngp_control_arm_toe_" .. i, state.toeOutput[i])
        safeStore("ngp_ca_camber_" .. i, state.camberOutput[i])
        safeStore("ngp_ca_toe_" .. i, state.toeOutput[i])
    end

    if not state.debugStoreNow then return end

    safeStore("ngp_control_arm_raw_input", state.rawInput)
    safeStore("ngp_control_arm_front_tau_scale", state.bodyFrontTauScale)
    safeStore("ngp_control_arm_rear_tau_scale", state.bodyRearTauScale)
    safeStore("ngp_control_arm_front_deflection_scale", state.bodyFrontDeflectionScale)
    safeStore("ngp_control_arm_rear_deflection_scale", state.bodyRearDeflectionScale)

    safeStore("ngp_control_arm_front_toe_avg", state.frontToeAvg)
    safeStore("ngp_control_arm_rear_toe_avg", state.rearToeAvg)
    safeStore("ngp_control_arm_front_camber_avg", state.frontCamberAvg)
    safeStore("ngp_control_arm_rear_camber_avg", state.rearCamberAvg)

    safeStore("ngp_control_arm_front_load_diff", state.frontLoadDiff)
    safeStore("ngp_control_arm_rear_load_diff", state.rearLoadDiff)
    safeStore("ngp_control_arm_front_contact_loss", state.frontContactLoss)
    safeStore("ngp_control_arm_rear_contact_loss", state.rearContactLoss)

    safeStore("ngp_control_arm_flex_energy", state.flexEnergy)
    safeStore("ngp_control_arm_flex_release", state.flexRelease)

    safeStore("ngp_control_arm_arm_linked", state.armLinked and 1 or 0)
    safeStore("ngp_control_arm_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_control_arm_contact_linked", state.contactLinked and 1 or 0)
    safeStore("ngp_control_arm_flex_linked", state.flexLinked and 1 or 0)
    safeStore("ngp_control_arm_compliance_linked", state.complianceLinked and 1 or 0)
end

function M.init()
    state.status = "INIT"
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1
    dt = clamp(safeNumber(dt, 0.0), M.params.minDt, M.params.maxDt)

    updateDebugGate(dt)

    car = car or safeGetCar()
    if not car then
        state.status = "NO CAR"
        state.wheelsValid = false
        exportState()
        return
    end

    state.wheelsValid = safeField(car, "wheels", nil) ~= nil

    readArmCompliance()
    readLoadContact(dt)
    readChassisFlex()
    readBodyRigidity(dt)
    updateBodyScales(dt)

    local input = calculateControlArmInput(car)
    updateOutputs(dt, input)

    state.status = "RUNNING"
    exportState()
end

function M.getFront()
    return state.frontDef or 0.0
end

function M.getRear()
    return state.rearDef or 0.0
end

function M.getTotal()
    return state.total or 0.0
end

function M.getCamber(index)
    return state.camberOutput[index] or 0.0
end

function M.getToe(index)
    return state.toeOutput[index] or 0.0
end

function M.getState()
    return state
end

function M.debugStr(index)
    local i = index
    if i ~= nil then
        i = math.floor(clamp(i, 0, 3))
        return string.format(
            "ControlArm W%d Cam %.4f Toe %.4f\nArm C %.4f T %.4f / Comp C %.4f T %.4f\nLoad %.0f Contact %.2f Leak %.2f Loss %.2f",
            i,
            state.camberOutput[i] or 0.0,
            state.toeOutput[i] or 0.0,
            state.armCamber[i] or 0.0,
            state.armToe[i] or 0.0,
            state.complianceCamber[i] or 0.0,
            state.complianceToe[i] or 0.0,
            state.load[i] or 0.0,
            state.contact[i] or 0.0,
            state.forceLeak[i] or 0.0,
            state.loadPathLoss[i] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s / Body %s\n" ..
        "Arm Front %.4f Rear %.4f Total %.4f\n" ..
        "RawYaw %.4f RawInput %.4f\n" ..
        "Tau F %.2f R %.2f / Def F %.2f R %.2f\n" ..
        "ToeAvg F %.4f R %.4f / Links Arm:%s Load:%s Contact:%s Flex:%s Comp:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",
        state.bodyLinked and "OK" or "NIL",
        state.frontDef or 0.0,
        state.rearDef or 0.0,
        state.total or 0.0,
        state.rawYaw or 0.0,
        state.rawInput or 0.0,
        state.bodyFrontTauScale or 1.0,
        state.bodyRearTauScale or 1.0,
        state.bodyFrontDeflectionScale or 1.0,
        state.bodyRearDeflectionScale or 1.0,
        state.frontToeAvg or 0.0,
        state.rearToeAvg or 0.0,
        state.armLinked and "OK" or "NIL",
        state.loadLinked and "OK" or "NIL",
        state.contactLinked and "OK" or "NIL",
        state.flexLinked and "OK" or "NIL",
        state.complianceLinked and "OK" or "NIL"
    )
end

return M
