---@diagnostic disable: undefined-global

--============================================================
-- body_rigidity_estimator.lua
-- ACNextGen V1.1.5 stable
-- Body rigidity root signal estimator
--============================================================

local M = {}

M.params = {
    refMass = 1300.0,
    refWheelbase = 2.55,
    refTrack = 1.55,
    refHeight = 0.55,
    refColliderLength = 4.20,
    refColliderWidth = 1.70,
    refColliderHeight = 0.45,

    refTorsionalStiffness = 18000.0,
    minTorsionalStiffness = 6000.0,
    maxTorsionalStiffness = 42000.0,
    refTorsionalDamping = 450.0,

    minRigidity = 0.20,
    maxRigidity = 1.35,
    minFlexFactor = 0.25,
    maxFlexFactor = 1.80,

    massWeight = 0.22,
    wheelbaseWeight = 0.20,
    trackWeight = 0.12,
    heightWeight = 0.16,
    colliderWeight = 0.18,
    suspensionWeight = 0.12,

    maxColliders = 16,

    defaultMass = 1300.0,
    defaultWheelbase = 2.55,
    defaultTrackF = 1.55,
    defaultTrackR = 1.55,
    defaultCGLocation = 0.50,
    defaultRideHeight = 0.33,

    massBalanceInfluence = 0.14,
    yawInertiaInfluence = 0.10,
    cgShiftInfluence = 0.08,

    loadInstabilityFlexGain = 0.08,
    tireHopFlexGain = 0.06,
    damageFlexGain = 0.16,
    conditionFlexGain = 0.10,

    brakeThermalFlexGain = 0.04,
    brakeLockFlexGain = 0.05,
    suspensionStressFlexGain = 0.06,
    chassisEnergyFlexGain = 0.05,

    maxBasePenalty = 0.45,
    maxRuntimeFlexPenalty = 0.55,

    runtimeReadInterval = 0.05,
    debugStoreInterval = 0.25,
}

local state = {
    mass = 1300.0,
    inertiaX = 1.0,
    inertiaY = 1.0,
    inertiaZ = 1.0,

    wheelbase = 2.55,
    trackF = 1.55,
    trackR = 1.55,
    avgTrack = 1.55,
    cgLocation = 0.50,
    rideHeightApprox = 0.33,

    frontSpring = 0.0,
    rearSpring = 0.0,
    frontDamper = 0.0,
    rearDamper = 0.0,

    colliderCount = 0,
    colliderWidth = 1.70,
    colliderHeight = 0.45,
    colliderLength = 4.20,
    colliderVolume = 0.0,

    extensionFlexFound = false,
    torsionalStiffness = 0.0,
    torsionalDamping = 0.0,

    massFactor = 1.0,
    wheelbaseFactor = 1.0,
    trackFactor = 1.0,
    heightFactor = 1.0,
    colliderFactor = 1.0,
    suspensionFactor = 1.0,
    extensionFactor = 1.0,

    baseFlexPenalty = 0.0,
    runtimeFlexPenalty = 0.0,
    rootFlexPenalty = 0.0,

    brakeThermalStress = 0.0,
    brakeLockAvg = 0.0,
    suspensionStress = 0.0,
    chassisEnergy = 0.0,

    bodyRigidity = 1.0,
    bodyFlexFactor = 1.0,
    torsionFactor = 1.0,
    bendingFactor = 1.0,
    dampingFactor = 1.0,

    status = "INIT",
    updateCount = 0,
    initialized = false,

    massLinked = false,
    loadLinked = false,
    tireLinked = false,
    damageLinked = false,
    brakeLinked = false,
    suspensionLinked = false,
    chassisLinked = false,

    runtimeReadTimer = 999.0,
    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

--============================================================
-- Helpers
--============================================================

local function safeNumber(v, d)
    local n = tonumber(v)
    if n == nil or n ~= n then
        return d or 0.0
    end
    return n
end

local function clamp(v, mn, mx)
    v = safeNumber(v, mn or 0.0)
    if v < mn then return mn end
    if v > mx then return mx end
    return v
end

local function abs(v)
    v = safeNumber(v, 0.0)
    if v < 0.0 then return -v end
    return v
end

local function safeStore(k, v)
    if not ac or not ac.store or not k then return end
    pcall(function()
        ac.store(k, v)
    end)
end

local function safeLoadRaw(k)
    if not ac or not ac.load or not k then return nil end
    local ok, v = pcall(function()
        return ac.load(k)
    end)
    if not ok then return nil end
    return v
end

local function safeLoad(k, d)
    local v = safeLoadRaw(k)
    if v == nil then return d or 0.0 end
    return safeNumber(v, d or 0.0)
end

local function safeLoadAny(d, ...)
    for i = 1, select("#", ...) do
        local k = select(i, ...)
        local v = safeLoadRaw(k)
        if v ~= nil then
            return safeNumber(v, d or 0.0), true
        end
    end
    return d or 0.0, false
end

local function safeConfig(section, key, defaultValue)
    if not ac or not ac.getCarConfig then
        return defaultValue
    end

    local ok, value = pcall(function()
        return ac.getCarConfig(0, section, key, defaultValue)
    end)

    if not ok or value == nil then
        return defaultValue
    end

    return value
end

local function safeConfigNumber(section, key, defaultValue)
    return safeNumber(safeConfig(section, key, defaultValue), defaultValue)
end

local function parseVec3(value, dx, dy, dz)
    if type(value) ~= "string" then
        return dx or 0.0, dy or 0.0, dz or 0.0
    end

    local x, y, z = string.match(value, "([^,]+),([^,]+),([^,]+)")
    return safeNumber(x, dx or 0.0), safeNumber(y, dy or 0.0), safeNumber(z, dz or 0.0)
end

local function invFactor(v)
    v = math.max(safeNumber(v, 1.0), 0.001)
    return 1.0 / v
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

--============================================================
-- Static vehicle reads
--============================================================

local function readCarBasic()
    state.mass = safeConfigNumber("BASIC", "TOTALMASS", M.params.defaultMass)

    local inertiaStr = safeConfig("BASIC", "INERTIA", "1.0,1.0,1.0")
    state.inertiaX, state.inertiaY, state.inertiaZ = parseVec3(inertiaStr, 1.0, 1.0, 1.0)
end

local function readSuspensionBasic()
    state.wheelbase = safeConfigNumber("BASIC", "WHEELBASE", M.params.defaultWheelbase)
    state.cgLocation = safeConfigNumber("BASIC", "CG_LOCATION", M.params.defaultCGLocation)

    state.trackF = safeConfigNumber("FRONT", "TRACK", M.params.defaultTrackF)
    state.trackR = safeConfigNumber("REAR", "TRACK", M.params.defaultTrackR)
    state.avgTrack = (state.trackF + state.trackR) * 0.5

    local frontBaseY = safeConfigNumber("FRONT", "BASEY", -0.13)
    local rearBaseY = safeConfigNumber("REAR", "BASEY", -0.13)
    local pickupFront = safeConfigNumber("RIDE", "PICKUP_FRONT_HEIGHT", -0.33)
    local pickupRear = safeConfigNumber("RIDE", "PICKUP_REAR_HEIGHT", -0.33)

    state.rideHeightApprox = abs((pickupFront + pickupRear + frontBaseY + rearBaseY) * 0.25)

    if state.rideHeightApprox <= 0.05 then
        state.rideHeightApprox = M.params.defaultRideHeight
    end

    state.frontSpring = safeConfigNumber("FRONT", "SPRING_RATE", 0.0)
    state.rearSpring = safeConfigNumber("REAR", "SPRING_RATE", 0.0)
    state.frontDamper = safeConfigNumber("FRONT", "DAMP_BUMP", 0.0)
    state.rearDamper = safeConfigNumber("REAR", "DAMP_BUMP", 0.0)
end

local function readColliderData()
    local maxW = 0.0
    local maxH = 0.0
    local maxL = 0.0
    local totalVolume = 0.0
    local count = 0

    for i = 0, M.params.maxColliders - 1 do
        local sizeStr = safeConfig("COLLIDER_" .. tostring(i), "SIZE", nil)

        if type(sizeStr) == "string" then
            local sx, sy, sz = parseVec3(sizeStr, 0.0, 0.0, 0.0)
            sx, sy, sz = abs(sx), abs(sy), abs(sz)

            if sx > 0.0 and sy > 0.0 and sz > 0.0 then
                count = count + 1
                maxW = math.max(maxW, sx)
                maxH = math.max(maxH, sy)
                maxL = math.max(maxL, sz)
                totalVolume = totalVolume + sx * sy * sz
            end
        end
    end

    state.colliderCount = count

    if count > 0 then
        state.colliderWidth = maxW
        state.colliderHeight = maxH
        state.colliderLength = maxL
        state.colliderVolume = totalVolume
    else
        state.colliderWidth = M.params.refColliderWidth
        state.colliderHeight = M.params.refColliderHeight
        state.colliderLength = M.params.refColliderLength
        state.colliderVolume = state.colliderWidth * state.colliderHeight * state.colliderLength
    end
end

local function readExtensionFlex()
    local torsion = safeConfigNumber("_EXTENSION_FLEX", "TORSIONAL_STIFFNESS", 0.0)
    local damping = safeConfigNumber("_EXTENSION_FLEX", "TORSIONAL_DAMPING", 0.0)

    state.torsionalStiffness = torsion
    state.torsionalDamping = damping
    state.extensionFlexFound = torsion > 0.0 or damping > 0.0
end

--============================================================
-- Runtime coupling
--============================================================

local function readRootCoupling()
    local p = M.params

    local massScaleRaw = safeLoadRaw("ngp_vehicle_mass_scale")
    local yawScaleRaw = safeLoadRaw("ngp_vehicle_yaw_inertia_scale")
    local cgRearRaw = safeLoadRaw("ngp_vehicle_cg_rear_shift")

    state.massLinked = massScaleRaw ~= nil or yawScaleRaw ~= nil or cgRearRaw ~= nil

    local massScale = safeNumber(massScaleRaw, 1.0)
    local yawScale = safeNumber(yawScaleRaw, 1.0)
    local cgRear = safeNumber(cgRearRaw, 0.0)

    local loadInstability, loadLinked =
        safeLoadAny(0.0, "ngp_virtual_instability", "ngp_load_transfer_pitch_ratio")

    state.loadLinked = loadLinked
    loadInstability = abs(loadInstability)

    local tireHopSum = 0.0
    local tireLinked = false

    for i = 0, 3 do
        local hop, linked =
            safeLoadAny(0.0, "ngp_tire_hop_energy_" .. i, "ngp_tire_hop_" .. i)

        if linked then tireLinked = true end
        tireHopSum = tireHopSum + clamp(hop, 0.0, 1.0)
    end

    state.tireLinked = tireLinked

    local damage, damageLinked =
        safeLoadAny(0.0, "ngp_condition_total", "ngp_vehicle_condition_total", "ngp_damage_chassis")

    state.damageLinked = damageLinked
    damage = clamp(damage, 0.0, 1.0)

    state.baseFlexPenalty =
        clamp(
            (massScale - 1.0) * p.massBalanceInfluence
            + (yawScale - 1.0) * p.yawInertiaInfluence
            + abs(cgRear) * p.cgShiftInfluence
            + loadInstability * p.loadInstabilityFlexGain
            + (tireHopSum * 0.25) * p.tireHopFlexGain
            + damage * (p.damageFlexGain + p.conditionFlexGain),
            0.0,
            p.maxBasePenalty
        )

    local brakeTemp, brakeTempLinked =
        safeLoadAny(25.0, "ngp_brake_temp_max", "ngp_brake_temp_avg", "ngp_brake_disc_temp_avg")

    local lockSum = 0.0
    local brakeLinked = brakeTempLinked

    for i = 0, 3 do
        local lock, linked = safeLoadAny(0.0, "ngp_brake_lock_" .. i)
        if linked then brakeLinked = true end
        lockSum = lockSum + clamp(lock, 0.0, 1.0)
    end

    state.brakeLinked = brakeLinked
    state.brakeLockAvg = lockSum * 0.25
    state.brakeThermalStress = clamp((brakeTemp - 25.0) / 700.0, 0.0, 1.0)

    local suspSum = 0.0
    local suspensionLinked = false

    for i = 0, 3 do
        local suspForce, suspLinked =
            safeLoadAny(0.0,
                "ngp_susp_integrated_force_" .. i,
                "ngp_susp_int_force_" .. i,
                "ngp_susp_" .. i
            )

        local damper, damperLinked =
            safeLoadAny(0.0,
                "ngp_damper_force_" .. i,
                "ngp_damper_" .. i,
                "ngp_damper_hyst_" .. i
            )

        local spring, springLinked =
            safeLoadAny(0.0,
                "ngp_spring_force_" .. i,
                "ngp_progressive_spring_" .. i
            )

        if suspLinked or damperLinked or springLinked then
            suspensionLinked = true
        end

        suspSum =
            suspSum
            + math.min(
                (abs(suspForce) + abs(damper) + abs(spring)) / 25000.0,
                1.0
            )
    end

    state.suspensionLinked = suspensionLinked
    state.suspensionStress = clamp(suspSum * 0.25, 0.0, 1.0)

    local chassisEnergy, chassisLinked =
        safeLoadAny(0.0, "ngp_chassis_energy", "ngp_chassis_body_energy", "ngp_body_chassis_energy")

    state.chassisLinked = chassisLinked
    state.chassisEnergy = clamp(chassisEnergy, 0.0, 1.0)

    state.runtimeFlexPenalty =
        clamp(
            state.brakeThermalStress * p.brakeThermalFlexGain
            + state.brakeLockAvg * p.brakeLockFlexGain
            + state.suspensionStress * p.suspensionStressFlexGain
            + state.chassisEnergy * p.chassisEnergyFlexGain,
            0.0,
            p.maxRuntimeFlexPenalty
        )

    state.rootFlexPenalty =
        clamp(
            state.baseFlexPenalty + state.runtimeFlexPenalty,
            0.0,
            p.maxRuntimeFlexPenalty
        )
end

local function calculateFactors()
    local p = M.params

    state.massFactor = clamp(invFactor(state.mass / p.refMass), 0.65, 1.35)
    state.wheelbaseFactor = clamp(invFactor(state.wheelbase / p.refWheelbase), 0.70, 1.30)
    state.trackFactor = clamp(state.avgTrack / p.refTrack, 0.75, 1.25)
    state.heightFactor = clamp(invFactor(state.rideHeightApprox / p.refHeight), 0.70, 1.30)

    local colliderLengthFactor = invFactor(state.colliderLength / p.refColliderLength)
    local colliderWidthFactor = state.colliderWidth / p.refColliderWidth
    local colliderHeightFactor = clamp(state.colliderHeight / p.refColliderHeight, 0.45, 1.25)

    state.colliderFactor =
        clamp(
            colliderLengthFactor * 0.45
            + colliderWidthFactor * 0.35
            + colliderHeightFactor * 0.20,
            0.60,
            1.35
        )

    local springAvg = (state.frontSpring + state.rearSpring) * 0.5
    state.suspensionFactor = clamp(springAvg / 65000.0, 0.65, 1.35)

    if state.extensionFlexFound then
        local stiffnessNorm =
            clamp(
                state.torsionalStiffness,
                p.minTorsionalStiffness,
                p.maxTorsionalStiffness
            )
            / p.refTorsionalStiffness

        state.extensionFactor = clamp(stiffnessNorm, 0.45, 1.85)
    else
        state.extensionFactor = 1.0
    end
end

local function calculateRigidity()
    local p = M.params

    local estimated =
        state.massFactor * p.massWeight
        + state.wheelbaseFactor * p.wheelbaseWeight
        + state.trackFactor * p.trackWeight
        + state.heightFactor * p.heightWeight
        + state.colliderFactor * p.colliderWeight
        + state.suspensionFactor * p.suspensionWeight

    if state.extensionFlexFound then
        estimated = estimated * 0.65 + state.extensionFactor * 0.35
    end

    estimated = estimated - state.rootFlexPenalty

    state.bodyRigidity = clamp(estimated, p.minRigidity, p.maxRigidity)
    state.bodyFlexFactor = clamp(1.0 / math.max(state.bodyRigidity, 0.05), p.minFlexFactor, p.maxFlexFactor)

    state.torsionFactor =
        clamp(
            state.bodyFlexFactor
            * (state.wheelbase / p.refWheelbase)
            * (p.refTrack / math.max(state.avgTrack, 0.1)),
            0.35,
            2.00
        )

    state.bendingFactor =
        clamp(
            state.bodyFlexFactor
            * (state.mass / p.refMass)
            * (state.wheelbase / p.refWheelbase),
            0.35,
            2.00
        )

    if state.torsionalDamping > 0.0 then
        state.dampingFactor = clamp(state.torsionalDamping / p.refTorsionalDamping, 0.50, 1.80)
    else
        state.dampingFactor = 1.0
    end
end

--============================================================
-- Export
--============================================================

local function exportState()
    safeStore("ngp_body_rigidity_status", state.status)
    safeStore("ngp_body_rigidity_estimator_status", state.status)
    safeStore("ngp_body_status", state.status)

    safeStore("ngp_body_rigidity_update_count", state.updateCount)
    safeStore("ngp_body_update_count", state.updateCount)

    safeStore("ngp_body_rigidity", state.bodyRigidity)
    safeStore("ngp_body_flex_factor", state.bodyFlexFactor)
    safeStore("ngp_body_torsion_factor", state.torsionFactor)
    safeStore("ngp_body_bending_factor", state.bendingFactor)
    safeStore("ngp_body_damping_factor", state.dampingFactor)

    safeStore("ngp_body_mass", state.mass)
    safeStore("ngp_body_wheelbase", state.wheelbase)
    safeStore("ngp_body_track_avg", state.avgTrack)
    safeStore("ngp_body_ride_height", state.rideHeightApprox)
    safeStore("ngp_body_collider_count", state.colliderCount)
    safeStore("ngp_body_extension_flex_found", state.extensionFlexFound and 1 or 0)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_body_base_flex_penalty", state.baseFlexPenalty)
    safeStore("ngp_body_root_flex_penalty", state.rootFlexPenalty)
    safeStore("ngp_body_runtime_flex_penalty", state.runtimeFlexPenalty)
    safeStore("ngp_body_brake_thermal_stress", state.brakeThermalStress)
    safeStore("ngp_body_brake_lock_avg", state.brakeLockAvg)
    safeStore("ngp_body_suspension_stress", state.suspensionStress)
    safeStore("ngp_body_chassis_energy_input", state.chassisEnergy)

    safeStore("ngp_body_mass_linked", state.massLinked and 1 or 0)
    safeStore("ngp_body_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_body_tire_linked", state.tireLinked and 1 or 0)
    safeStore("ngp_body_damage_linked", state.damageLinked and 1 or 0)
    safeStore("ngp_body_brake_linked", state.brakeLinked and 1 or 0)
    safeStore("ngp_body_suspension_linked", state.suspensionLinked and 1 or 0)
    safeStore("ngp_body_chassis_linked", state.chassisLinked and 1 or 0)

    safeStore("ngp_body_factor_mass", state.massFactor)
    safeStore("ngp_body_factor_wheelbase", state.wheelbaseFactor)
    safeStore("ngp_body_factor_track", state.trackFactor)
    safeStore("ngp_body_factor_height", state.heightFactor)
    safeStore("ngp_body_factor_collider", state.colliderFactor)
    safeStore("ngp_body_factor_suspension", state.suspensionFactor)
    safeStore("ngp_body_factor_extension", state.extensionFactor)
end

local function recalcStatic()
    readCarBasic()
    readSuspensionBasic()
    readColliderData()
    readExtensionFlex()
    calculateFactors()
end

local function recalcDynamic(force)
    if force then
        state.runtimeReadTimer = M.params.runtimeReadInterval
    end

    if not force then
        if state.runtimeReadTimer < M.params.runtimeReadInterval then
            return false
        end
    end

    state.runtimeReadTimer = 0.0
    readRootCoupling()
    calculateRigidity()
    return true
end

--============================================================
-- Public API
--============================================================

function M.init()
    state.status = "INIT"
    state.updateCount = state.updateCount + 1

    recalcStatic()
    recalcDynamic(true)

    state.initialized = true
    state.status = "RUNNING"

    exportState()
end

function M.update(dt, car, runtime)
    dt = safeNumber(dt, 0.0)

    if dt <= 0.0 then
        state.status = "BAD DT"
        exportState()
        return
    end

    updateDebugGate(dt)

    if not state.initialized then
        M.init()
        return
    end

    state.updateCount = state.updateCount + 1
    state.runtimeReadTimer = (state.runtimeReadTimer or 0.0) + dt

    recalcDynamic(false)

    state.status = "RUNNING"
    exportState()
end

function M.getRigidity()
    return state.bodyRigidity or 1.0
end

function M.getFlexFactor()
    return state.bodyFlexFactor or 1.0
end

function M.getTorsionFactor()
    return state.torsionFactor or 1.0
end

function M.getBendingFactor()
    return state.bendingFactor or 1.0
end

function M.debugStr()
    return string.format(
        "Status %s / Count %.0f\n" ..
        "Rig %.3f Flex %.3f Tors %.3f Bend %.3f Damp %.3f\n" ..
        "Mass %.0f WB %.2f Track %.2f RH %.2f\n" ..
        "Penalty B %.3f R %.3f Total %.3f\n" ..
        "BrakeHeat %.2f Lock %.2f Susp %.2f Energy %.2f\n" ..
        "Links Mass:%s Load:%s Tire:%s Damage:%s Brake:%s Susp:%s Chassis:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.bodyRigidity or 1.0,
        state.bodyFlexFactor or 1.0,
        state.torsionFactor or 1.0,
        state.bendingFactor or 1.0,
        state.dampingFactor or 1.0,
        state.mass or 0.0,
        state.wheelbase or 0.0,
        state.avgTrack or 0.0,
        state.rideHeightApprox or 0.0,
        state.baseFlexPenalty or 0.0,
        state.runtimeFlexPenalty or 0.0,
        state.rootFlexPenalty or 0.0,
        state.brakeThermalStress or 0.0,
        state.brakeLockAvg or 0.0,
        state.suspensionStress or 0.0,
        state.chassisEnergy or 0.0,
        state.massLinked and "OK" or "NIL",
        state.loadLinked and "OK" or "NIL",
        state.tireLinked and "OK" or "NIL",
        state.damageLinked and "OK" or "NIL",
        state.brakeLinked and "OK" or "NIL",
        state.suspensionLinked and "OK" or "NIL",
        state.chassisLinked and "OK" or "NIL"
    )
end

return M
