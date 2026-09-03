---@diagnostic disable: undefined-global

--============================================================
-- ACNextGen
-- arm_compliance.lua
-- Phase 4.6 / V1.1 stable
-- Control arm and bushing compliance approximation
--============================================================

local M = {}

--============================================================
-- Parameters
--============================================================

M.params = {
    lateralFlex = 0.020,
    brakeFlex   = 0.010,
    driveFlex   = 0.008,

    loadCamberGain = 0.0000045,
    loadToeGain    = 0.0000025,

    contactCamberGain = 0.010,
    contactToeGain    = 0.006,

    suspInputCamberGain = 0.012,
    suspInputToeGain    = 0.006,

    bodyReadInterval = 0.25,

    bodyFlexComplianceGain    = 0.22,
    bodyTorsionComplianceGain = 0.14,
    bodyBendingComplianceGain = 0.10,
    bodyStiffComplianceLoss   = 0.08,

    minBodyComplianceScale = 0.86,
    maxBodyComplianceScale = 1.22,

    tau = 0.080,

    flexEnergyTauGain  = 0.20,
    flexReleaseTauLoss = 0.08,

    minTauScale = 0.85,
    maxTauScale = 1.25,

    maxCamber   = 0.080,
    maxToe      = 0.050,
    maxLoadNorm = 1.75,

    debugStoreInterval = 0.25,
}

--============================================================
-- State
--============================================================

local state = {
    camber       = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    toe          = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    targetCamber = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    targetToe    = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    load        = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    loadNorm    = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    contact     = { [0] = 1.0, [1] = 1.0, [2] = 1.0, [3] = 1.0 },
    contactLoss = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },
    suspInput   = { [0] = 0.0, [1] = 0.0, [2] = 0.0, [3] = 0.0 },

    lateralAccel = 0.0,
    brake = 0.0,
    gas = 0.0,

    bodySteer = 0.0,
    flexEnergy = 0.0,
    flexRelease = 0.0,

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    bodyTorsionFactor = 1.0,
    bodyBendingFactor = 1.0,
    bodyDampingFactor = 1.0,
    bodyComplianceScale = 1.0,
    bodyLinked = false,
    bodyReadTimer = 999.0,

    tauScale = 1.0,

    status = "INIT",
    updateCount = 0,
    wheelsValid = false,

    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

--============================================================
-- Helpers
--============================================================

local function num(v, defaultValue)
    local n = tonumber(v)
    if n == nil or n ~= n then
        return defaultValue or 0.0
    end
    return n
end

local function clamp(v, minValue, maxValue)
    v = num(v, minValue)
    if v < minValue then return minValue end
    if v > maxValue then return maxValue end
    return v
end

local function safeField(obj, field, defaultValue)
    if not obj then return defaultValue end

    local ok, value = pcall(function()
        return obj[field]
    end)

    if not ok or value == nil then
        return defaultValue
    end

    return value
end

local function safeLoad(key, defaultValue)
    if not ac or not ac.load then
        return defaultValue or 0.0
    end

    local ok, value = pcall(function()
        return ac.load(key)
    end)

    if not ok or value == nil then
        return defaultValue or 0.0
    end

    return num(value, defaultValue or 0.0)
end

local function safeStore(key, value)
    if not ac or not ac.store then return end

    pcall(function()
        ac.store(key, value)
    end)
end

local function getCarSafe()
    if not ac or not ac.getCar then return nil end

    local ok, car = pcall(function()
        return ac.getCar(0)
    end)

    if ok then return car end
    return nil
end

local function getWheelSafe(car, index)
    if not car then return nil end

    local wheels = safeField(car, "wheels", nil)
    if not wheels then return nil end

    local ok, wheel = pcall(function()
        return wheels[index]
    end)

    if ok then return wheel end
    return nil
end

local function lowPass(current, target, tau, dt)
    current = num(current, 0.0)
    target = num(target, 0.0)
    tau = math.max(num(tau, 0.001), 0.0001)
    dt = math.max(num(dt, 0.0), 0.0)

    local alpha = dt / (tau + dt)
    alpha = clamp(alpha, 0.0, 1.0)

    return current + (target - current) * alpha
end

local function updateDebugGate(dt)
    state.debugStoreTimer = (state.debugStoreTimer or 0.0) + num(dt, 0.0)

    if state.debugStoreTimer >= M.params.debugStoreInterval then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    else
        state.debugStoreNow = false
    end
end

--============================================================
-- Input readers
--============================================================

local function readCarInput(car)
    local acc = safeField(car, "acceleration", nil)

    if acc then
        state.lateralAccel = num(safeField(acc, "x", 0.0), 0.0)
    else
        state.lateralAccel = 0.0
    end

    state.brake = num(safeField(car, "brake", 0.0), 0.0)
    state.gas = num(safeField(car, "gas", 0.0), 0.0)
end

local function readChassisFlex()
    state.bodySteer = safeLoad("ngp_body_steer", 0.0)
    state.flexEnergy = safeLoad("ngp_chassis_flex_energy", safeLoad("ngp_flex_energy", 0.0))
    state.flexRelease = safeLoad("ngp_chassis_flex_release", safeLoad("ngp_flex_release", 0.0))
end

local function readBodyRigidity(dt)
    state.bodyReadTimer = (state.bodyReadTimer or 999.0) + num(dt, 0.0)

    if state.bodyReadTimer < M.params.bodyReadInterval then
        return
    end

    state.bodyReadTimer = 0.0

    state.bodyRigidity = clamp(safeLoad("ngp_body_rigidity", 1.0), 0.20, 1.35)
    state.bodyFlexFactor = clamp(safeLoad("ngp_body_flex_factor", 1.0), 0.25, 1.80)
    state.bodyTorsionFactor = clamp(safeLoad("ngp_body_torsion_factor", 1.0), 0.35, 2.00)
    state.bodyBendingFactor = clamp(safeLoad("ngp_body_bending_factor", 1.0), 0.35, 2.00)
    state.bodyDampingFactor = clamp(safeLoad("ngp_body_damping_factor", 1.0), 0.50, 1.80)

    state.bodyLinked = safeLoad("ngp_body_rigidity_update_count", 0.0) > 0.0
end

local function updateBodyComplianceScale()
    if not state.bodyLinked then
        state.bodyComplianceScale = 1.0
        return
    end

    local p = M.params

    local flex = clamp(state.bodyFlexFactor or 1.0, 0.25, 1.80)
    local torsion = clamp(state.bodyTorsionFactor or 1.0, 0.35, 2.00)
    local bending = clamp(state.bodyBendingFactor or 1.0, 0.35, 2.00)
    local rigidity = clamp(state.bodyRigidity or 1.0, 0.20, 1.35)

    local softness = clamp(flex - 1.0, 0.0, 0.80)
    local torsionSoft = clamp(torsion - 1.0, 0.0, 1.0)
    local bendingSoft = clamp(bending - 1.0, 0.0, 1.0)
    local stiff = clamp(rigidity - 1.0, 0.0, 0.35)

    state.bodyComplianceScale = clamp(
        1.0
        + softness * p.bodyFlexComplianceGain
        + torsionSoft * p.bodyTorsionComplianceGain
        + bendingSoft * p.bodyBendingComplianceGain
        - stiff * p.bodyStiffComplianceLoss,
        p.minBodyComplianceScale,
        p.maxBodyComplianceScale
    )
end

local function readWheelInputs(car)
    local wheels = safeField(car, "wheels", nil)
    state.wheelsValid = wheels ~= nil

    for i = 0, 3 do
        local wheel = getWheelSafe(car, i)
        local wheelLoad = 0.0

        if wheel then
            wheelLoad = num(safeField(wheel, "load", 0.0), 0.0)
        end

        local modelLoad = safeLoad("ngp_wheel_load_" .. i, safeLoad("ngp_dlt_load_" .. i, wheelLoad))
        state.load[i] = modelLoad

        state.loadNorm[i] = clamp(
            safeLoad("ngp_wheel_load_norm_" .. i, modelLoad / 3200.0),
            0.0,
            M.params.maxLoadNorm
        )

        state.contact[i] = clamp(
            safeLoad("ngp_tc_contact_" .. i, safeLoad("ngp_tire_contact_" .. i, 1.0)),
            0.0,
            1.2
        )

        state.contactLoss[i] = clamp(1.0 - state.contact[i], 0.0, 1.0)

        state.suspInput[i] = safeLoad(
            "ngp_susp_contact_input_" .. i,
            safeLoad("ngp_sci_road_input_" .. i, 0.0)
        )
    end
end

--============================================================
-- Core
--============================================================

local function calculateTauScale()
    local energy = clamp(state.flexEnergy or 0.0, 0.0, 1.0)
    local release = clamp(state.flexRelease or 0.0, 0.0, 1.0)

    state.tauScale = clamp(
        1.0
        + energy * M.params.flexEnergyTauGain
        - release * M.params.flexReleaseTauLoss,
        M.params.minTauScale,
        M.params.maxTauScale
    )
end

local function updateWheelCompliance(index, dt)
    local p = M.params

    local side = (index == 0 or index == 2) and -1.0 or 1.0
    local isRear = index >= 2
    local rearBias = isRear and 1.5 or 0.5
    local frontBias = isRear and 0.65 or 1.0

    local loadDelta = clamp((state.loadNorm[index] or 1.0) - 1.0, -1.0, 1.0)
    local contactLoss = state.contactLoss[index] or 0.0
    local suspInput = state.suspInput[index] or 0.0
    local bodyScale = state.bodyComplianceScale or 1.0

    local targetCamber = (
        state.lateralAccel * side * p.lateralFlex * frontBias
        + loadDelta * side * p.loadCamberGain * 3200.0
        + contactLoss * side * p.contactCamberGain
        + suspInput * side * p.suspInputCamberGain
    ) * bodyScale

    targetCamber = clamp(targetCamber, -p.maxCamber, p.maxCamber)

    local brakeDriveToe = (state.brake * p.brakeFlex - state.gas * p.driveFlex) * rearBias
    local loadToe = loadDelta * p.loadToeGain * 3200.0 * side
    local contactToe = contactLoss * p.contactToeGain * side
    local suspToe = suspInput * p.suspInputToeGain * side
    local bodySteerToe = state.bodySteer * 0.018 * (isRear and -0.65 or 0.35)

    local targetToe = (
        brakeDriveToe
        + loadToe
        + contactToe
        + suspToe
        + bodySteerToe
    ) * bodyScale

    targetToe = clamp(targetToe, -p.maxToe, p.maxToe)

    state.targetCamber[index] = targetCamber
    state.targetToe[index] = targetToe

    local tau = p.tau * (state.tauScale or 1.0)

    state.camber[index] = lowPass(state.camber[index], targetCamber, tau, dt)
    state.toe[index] = lowPass(state.toe[index], targetToe, tau, dt)
end

--============================================================
-- Export
--============================================================

local function exportState()
    for i = 0, 3 do
        local camber = state.camber[i] or 0.0
        local toe = state.toe[i] or 0.0

        safeStore("ngp_arm_camber_" .. i, camber)
        safeStore("ngp_arm_toe_" .. i, toe)
        safeStore("ngp_control_arm_camber_" .. i, camber)
        safeStore("ngp_control_arm_toe_" .. i, toe)
    end

    safeStore("ngp_arm_status", state.status or "UNKNOWN")
    safeStore("ngp_arm_update_count", state.updateCount or 0)
    safeStore("ngp_arm_wheels_valid", state.wheelsValid and 1 or 0)
    safeStore("ngp_arm_body_linked", state.bodyLinked and 1 or 0)
    safeStore("ngp_arm_body_compliance_scale", state.bodyComplianceScale or 1.0)

    safeStore("ngp_arm_compliance_status", state.status or "UNKNOWN")
    safeStore("ngp_arm_compliance_update_count", state.updateCount or 0)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_arm_lateral_accel", state.lateralAccel or 0.0)
    safeStore("ngp_arm_brake", state.brake or 0.0)
    safeStore("ngp_arm_gas", state.gas or 0.0)
    safeStore("ngp_arm_body_rigidity", state.bodyRigidity or 1.0)
    safeStore("ngp_arm_body_flex_factor", state.bodyFlexFactor or 1.0)
    safeStore("ngp_arm_tau_scale", state.tauScale or 1.0)
    safeStore("ngp_arm_flex_energy", state.flexEnergy or 0.0)
    safeStore("ngp_arm_flex_release", state.flexRelease or 0.0)

    for i = 0, 3 do
        safeStore("ngp_arm_target_camber_" .. i, state.targetCamber[i] or 0.0)
        safeStore("ngp_arm_target_toe_" .. i, state.targetToe[i] or 0.0)
        safeStore("ngp_arm_load_" .. i, state.load[i] or 0.0)
        safeStore("ngp_arm_load_norm_" .. i, state.loadNorm[i] or 1.0)
        safeStore("ngp_arm_contact_" .. i, state.contact[i] or 1.0)
        safeStore("ngp_arm_susp_input_" .. i, state.suspInput[i] or 0.0)
    end
end

--============================================================
-- Public API
--============================================================

function M.init()
    state.status = "INIT"
    state.debugStoreNow = true
    exportState()
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1

    dt = num(dt, 0.0)
    if dt <= 0.0 then
        state.status = "BAD DT"
        state.debugStoreNow = true
        exportState()
        return
    end

    updateDebugGate(dt)

    car = car or getCarSafe()
    if not car then
        state.status = "NO CAR"
        state.wheelsValid = false
        exportState()
        return
    end

    readCarInput(car)
    readChassisFlex()
    readBodyRigidity(dt)
    updateBodyComplianceScale()
    readWheelInputs(car)
    calculateTauScale()

    for i = 0, 3 do
        updateWheelCompliance(i, dt)
    end

    state.status = state.wheelsValid and "RUNNING" or "NO WHEELS"
    exportState()
end

function M.getCamber(i)
    return state.camber[i] or 0.0
end

function M.getToe(i)
    return state.toe[i] or 0.0
end

function M.debugStr(index)
    if index ~= nil then
        local i = clamp(index, 0, 3)
        return string.format(
            "Status %s / Count %.0f / Wheels %s\n" ..
            "%s Camber %.4f Toe %.4f / Load %.0f Norm %.2f\n" ..
            "Target C %.4f T %.4f / Contact %.2f Susp %.2f",
            tostring(state.status),
            state.updateCount or 0,
            state.wheelsValid and "OK" or "NIL",
            tostring(i),
            state.camber[i] or 0.0,
            state.toe[i] or 0.0,
            state.load[i] or 0.0,
            state.loadNorm[i] or 1.0,
            state.targetCamber[i] or 0.0,
            state.targetToe[i] or 0.0,
            state.contact[i] or 1.0,
            state.suspInput[i] or 0.0
        )
    end

    return string.format(
        "Status %s / Count %.0f / Wheels %s\n" ..
        "Lat %.3f Brake %.3f Gas %.3f\n" ..
        "BodyLink %s Comp %.2f Tau %.2f\n" ..
        "Camber %.3f %.3f %.3f %.3f\n" ..
        "Toe    %.3f %.3f %.3f %.3f",
        tostring(state.status),
        state.updateCount or 0,
        state.wheelsValid and "OK" or "NIL",
        state.lateralAccel or 0.0,
        state.brake or 0.0,
        state.gas or 0.0,
        state.bodyLinked and "YES" or "NO",
        state.bodyComplianceScale or 1.0,
        state.tauScale or 1.0,
        state.camber[0] or 0.0,
        state.camber[1] or 0.0,
        state.camber[2] or 0.0,
        state.camber[3] or 0.0,
        state.toe[0] or 0.0,
        state.toe[1] or 0.0,
        state.toe[2] or 0.0,
        state.toe[3] or 0.0
    )
end

return M
