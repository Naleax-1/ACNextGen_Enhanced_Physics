---@diagnostic disable: undefined-global

--============================================================
-- ACNeXtGen
-- mass_balance.lua
-- Phase 7 / V1.1.5 Stable
-- Front Axis Anchor / Mass Balance Signal Core
--
-- Policy:
--   This module does not change vehicle mass, CG, inertia, or AC physics.
--   It publishes front-axis coherence, authority, yaw resistance, and
--   steering-weight signals for downstream root/trunk modules.
--============================================================

local M = {}

local BASE_MASS = 1310.0

local FL, FR, RL, RR = 0, 1, 2, 3

local WHEEL_NAMES = {
    [0] = "FL",
    [1] = "FR",
    [2] = "RL",
    [3] = "RR",
}

M.params = {
    baseMass = BASE_MASS,

    frontLoadReference = 3200.0,
    wheelLoadReference = 3000.0,
    minLoadForAxis = 250.0,

    loadCoherenceGain = 0.75,
    contactCoherenceGain = 0.80,
    slipLossGain = 0.85,

    yawResistGain = 0.35,
    steerWeightGain = 0.55,
    authorityGain = 0.65,

    maxSlipAngle = 0.65,
    maxSlipRatio = 0.75,

    yawRateScale = 0.60,
    steerScale = 1.0,

    minAnchor = 0.0,
    maxAnchor = 1.25,

    minAuthority = 0.0,
    maxAuthority = 1.25,

    minYawResist = 0.0,
    maxYawResist = 1.15,

    minSteerWeight = 0.0,
    maxSteerWeight = 1.20,

    minSlipDamp = 0.0,
    maxSlipDamp = 1.0,

    anchorTau = 0.080,
    authorityTau = 0.070,
    yawResistTau = 0.090,
    steerWeightTau = 0.070,
    slipDampTau = 0.060,

    noCarReturnTau = 0.220,

    minDt = 0.00005,
    maxDt = 0.050,

    dltAbsoluteThreshold = 1200.0,

    debugStoreInterval = 0.25,
}

local state = {
    baseMass = BASE_MASS,
    massScale = 1.0,
    totalMassEstimate = BASE_MASS,

    load = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    contact = { [0]=1.0, [1]=1.0, [2]=1.0, [3]=1.0 },
    slipAngle = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },
    slipRatio = { [0]=0.0, [1]=0.0, [2]=0.0, [3]=0.0 },

    loadSource = { [0]="NONE", [1]="NONE", [2]="NONE", [3]="NONE" },
    contactSource = { [0]="NONE", [1]="NONE", [2]="NONE", [3]="NONE" },
    slipSource = { [0]="NONE", [1]="NONE", [2]="NONE", [3]="NONE" },

    steer = 0.0,
    yawRate = 0.0,
    speedKmh = 0.0,

    frontLoadAvg = 0.0,
    frontLoadTotal = 0.0,
    frontLoadDiff = 0.0,
    frontLoadBalance = 1.0,
    frontLoadCoherence = 1.0,

    frontContactAvg = 1.0,
    frontContactDiff = 0.0,
    frontContactBalance = 1.0,

    frontSlipAvg = 0.0,
    frontSlipLoss = 0.0,

    steerYawCoherence = 1.0,

    frontAxisAnchor = 0.0,
    frontAxisAuthority = 0.0,
    frontAxisYawResist = 0.0,
    frontAxisSteerWeight = 0.0,
    frontAxisSlipDamp = 0.0,

    targetAnchor = 0.0,
    targetAuthority = 0.0,
    targetYawResist = 0.0,
    targetSteerWeight = 0.0,
    targetSlipDamp = 0.0,

    frontAxisReserve = 0.0,
    frontAxisLoadQuality = 0.0,
    frontAxisContactQuality = 0.0,

    loadLinked = false,
    contactLinked = false,
    tireStateLinked = false,
    carLinked = false,
    wheelsValid = false,

    status = "INIT",
    updateCount = 0,
    initCount = 0,

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
    v = safeNumber(v, minValue)
    if v < minValue then return minValue end
    if v > maxValue then return maxValue end
    return v
end

local function abs(v)
    return math.abs(safeNumber(v, 0.0))
end

local function safeStore(key, value)
    if not ac or not ac.store then return end
    pcall(function() ac.store(key, value) end)
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
    if defaultValue == nil then
        return nil, nil
    end
    return defaultValue, nil
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
    tau = safeNumber(tau, 0.001)
    dt = safeNumber(dt, 0.0)

    if tau <= 0.0001 then return target end

    local alpha = clamp(dt / math.max(tau + dt, 0.0001), 0.0, 1.0)
    return current + (target - current) * alpha
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

local function safeGetCar()
    if not ac or not ac.getCar then return nil end
    local ok, car = pcall(function() return ac.getCar(0) end)
    if ok then return car end
    return nil
end

local function getWheel(car, index)
    if not car then return nil end

    local wheels = safeField(car, "wheels", nil)
    if not wheels then return nil end

    local ok, wheel = pcall(function()
        return wheels[index] or wheels[index + 1]
    end)

    if ok then return wheel end
    return nil
end

local function vecLength(v)
    if not v then return 0.0 end

    local ok, result = pcall(function()
        if type(v.length) == "function" then return v:length() end
        if type(v.length) == "number" then return v.length end

        local x = safeNumber(v.x, 0.0)
        local y = safeNumber(v.y, 0.0)
        local z = safeNumber(v.z, 0.0)
        return math.sqrt(x * x + y * y + z * z)
    end)

    if ok then return safeNumber(result, 0.0) end
    return 0.0
end

local function resetLinkFlags()
    state.loadLinked = false
    state.contactLinked = false
    state.tireStateLinked = false
    state.carLinked = false
    state.wheelsValid = false
end

local function readCarState(car)
    state.carLinked = car ~= nil

    if not car then
        state.steer = 0.0
        state.yawRate = 0.0
        state.speedKmh = 0.0
        state.baseMass = M.params.baseMass or BASE_MASS
        state.totalMassEstimate = state.baseMass
        state.massScale = 1.0
        return
    end

    state.steer = clamp(
        safeNumber(
            safeField(car, "steer",
                0.0
            ),
            0.0
        ),
        -1.0,
        1.0
    )

    local speedKmh = safeField(car, "speedKmh", nil)
    if speedKmh ~= nil then
        state.speedKmh = safeNumber(speedKmh, 0.0)
    else
        local speed = nil
        if speed ~= nil then
            state.speedKmh = safeNumber(speed, 0.0)
        else
            local velocity = safeField(car, "velocity", safeField(car, "localVelocity", nil))
            state.speedKmh = vecLength(velocity) * 3.6
        end
    end

    local av = safeField(car, "localAngularVelocity", nil)
    if av and av.y ~= nil then
        state.yawRate = safeNumber(av.y, 0.0)
    elseif av and av.z ~= nil then
        state.yawRate = safeNumber(av.z, 0.0)
    else
        state.yawRate = safeLoad("ngp_yaw_rate", 0.0)
    end

    state.baseMass = safeNumber(
        safeField(car, "mass", M.params.baseMass or BASE_MASS),
        M.params.baseMass or BASE_MASS
    )

    state.totalMassEstimate = state.baseMass
    state.massScale = 1.0
end

local function readWheelApiLoad(wheel)
    if not wheel then return nil end

    local candidates = {
        "load",
        "loadK",
    }

    for i = 1, #candidates do
        local v = safeField(wheel, candidates[i], nil)
        if v ~= nil then
            local n = safeNumber(v, nil)
            if n ~= nil and n > 1.0 then return n end
        end
    end

    return nil
end

local function readWheelLoad(index, wheel)
    local value, source = safeLoadAlt(
        nil,
        "ngp_contact_load_" .. index,
        "ngp_tire_state_load_" .. index,
        "ngp_wheel_load_" .. index,
        "ngp_load_wheel_" .. index,
        "ngp_tire_load_input_" .. index,
        "ngp_susp_load_input_" .. index,
        "ngp_tire_load_" .. index,
        "ngp_sprung_load_" .. index,
        "ngp_load_path_load_" .. index
    )

    if source ~= nil then
        state.loadLinked = true
    else
        local dlt = safeLoadRaw("ngp_dlt_load_" .. index)
        if dlt ~= nil then
            local n = safeNumber(dlt, 0.0)
            if math.abs(n) > (M.params.dltAbsoluteThreshold or 1200.0) then
                value = math.abs(n)
            else
                value = M.params.wheelLoadReference + n
            end
            source = "ngp_dlt_load_" .. index
            state.loadLinked = true
        end
    end

    if source == nil then
        local wheelLoad = readWheelApiLoad(wheel)
        if wheelLoad ~= nil then
            value = wheelLoad
            source = "wheel"
        end
    end

    if value == nil then
        value = M.params.wheelLoadReference
        source = "fallback"
    end

    state.load[index] = clamp(
        safeNumber(value, M.params.wheelLoadReference),
        0.0,
        12000.0
    )

    state.loadSource[index] = source or "NONE"
end

local function readContact(index)
    local value, source = safeLoadAlt(
        nil,
        "ngp_contact_trust_" .. index,
        "ngp_contact_quality_" .. index,
        "ngp_tire_contact_trust_" .. index,
        "ngp_tire_contact_quality_" .. index,
        "ngp_tcr_quality_" .. index,
        "ngp_tc_contact_" .. index,
        "ngp_tc_grip_" .. index,
        "ngp_load_path_integrity_" .. index
    )

    if value ~= nil then
        state.contactLinked = true
    else
        value = 1.0
        source = "fallback"
    end

    state.contact[index] = clamp(safeNumber(value, 1.0), 0.0, 1.2)
    state.contactSource[index] = source or "NONE"
end

local function readSlip(index, wheel)
    local slipAngle, angleSource = safeLoadAlt(
        nil,
        "ngp_contact_slip_angle_" .. index,
        "ngp_tire_slip_angle_" .. index,
        "ngp_slip_angle_" .. index,
        "ngp_filtered_slip_angle_" .. index,
        "ngp_tire_state_slip_angle_" .. index
    )

    local slipRatio, ratioSource = safeLoadAlt(
        nil,
        "ngp_contact_slip_ratio_" .. index,
        "ngp_tire_slip_ratio_" .. index,
        "ngp_slip_ratio_" .. index,
        "ngp_filtered_slip_ratio_" .. index,
        "ngp_tire_state_slip_ratio_" .. index,
        "ngp_slip_recovery_slip_" .. index
    )

    if slipAngle ~= nil or slipRatio ~= nil then
        state.tireStateLinked = true
    end

    if slipAngle == nil and wheel then
        slipAngle = safeField(wheel, "slipAngle", nil)
        if slipAngle ~= nil then angleSource = "wheel" end
    end

    if slipRatio == nil and wheel then
        slipRatio = safeField(wheel, "slipRatio", nil)
        if slipRatio ~= nil then ratioSource = "wheel" end
    end

    state.slipAngle[index] = clamp(
        safeNumber(slipAngle, 0.0),
        -M.params.maxSlipAngle,
        M.params.maxSlipAngle
    )

    state.slipRatio[index] = clamp(
        safeNumber(slipRatio, 0.0),
        -M.params.maxSlipRatio,
        M.params.maxSlipRatio
    )

    if angleSource or ratioSource then
        state.slipSource[index] = tostring(angleSource or ratioSource)
    else
        state.slipSource[index] = "fallback"
    end
end

local function readWheels(car)
    local anyWheel = false

    for i = 0, 3 do
        local wheel = getWheel(car, i)
        if wheel then anyWheel = true end

        readWheelLoad(i, wheel)
        readContact(i)
        readSlip(i, wheel)
    end

    state.wheelsValid = anyWheel
end

local function calculateLoadCoherence()
    local fl = state.load[FL] or 0.0
    local fr = state.load[FR] or 0.0
    local total = fl + fr
    local avg = total * 0.5
    local diff = abs(fl - fr)

    local balance = 1.0 - diff / math.max(total, 1.0)
    balance = clamp(balance, 0.0, 1.0)

    local loadScale = clamp(avg / math.max(M.params.frontLoadReference, 1.0), 0.0, 1.25)

    if avg < (M.params.minLoadForAxis or 250.0) then
        loadScale = loadScale * clamp(avg / math.max(M.params.minLoadForAxis, 1.0), 0.0, 1.0)
    end

    state.frontLoadTotal = total
    state.frontLoadAvg = avg
    state.frontLoadDiff = diff
    state.frontLoadBalance = balance

    state.frontLoadCoherence = clamp(
        loadScale * (1.0 - (1.0 - balance) * M.params.loadCoherenceGain),
        0.0,
        1.25
    )

    state.frontAxisLoadQuality = state.frontLoadCoherence
end

local function calculateContactCoherence()
    local cfl = state.contact[FL] or 1.0
    local cfr = state.contact[FR] or 1.0

    local avg = (cfl + cfr) * 0.5
    local diff = abs(cfl - cfr)
    local balance = 1.0 - diff * M.params.contactCoherenceGain

    state.frontContactAvg = clamp(avg, 0.0, 1.2)
    state.frontContactDiff = diff
    state.frontContactBalance = clamp(balance, 0.0, 1.0)
    state.frontAxisContactQuality = state.frontContactAvg * state.frontContactBalance
end

local function calculateSlipLoss()
    local sa = (abs(state.slipAngle[FL]) + abs(state.slipAngle[FR])) * 0.5
    local sr = (abs(state.slipRatio[FL]) + abs(state.slipRatio[FR])) * 0.5

    local slip = sa + sr * 0.35
    state.frontSlipAvg = slip
    state.frontSlipLoss = clamp(slip * M.params.slipLossGain, 0.0, 1.0)
end

local function calculateSteerYawCoherence()
    local steer = clamp(abs(state.steer) * M.params.steerScale, 0.0, 1.0)
    local yaw = clamp(abs(state.yawRate) * M.params.yawRateScale, 0.0, 1.0)
    local mismatch = abs(steer - yaw)

    state.steerYawCoherence = clamp(1.0 - mismatch * 0.65, 0.0, 1.0)
end

local function calculateFrontAxisTargets()
    calculateLoadCoherence()
    calculateContactCoherence()
    calculateSlipLoss()
    calculateSteerYawCoherence()

    local baseAnchor =
        state.frontLoadCoherence
        * state.frontContactAvg
        * state.frontContactBalance
        * (1.0 - state.frontSlipLoss)

    state.targetAnchor = clamp(
        baseAnchor,
        M.params.minAnchor,
        M.params.maxAnchor
    )

    state.targetAuthority = clamp(
        state.targetAnchor
        * (0.65 + 0.35 * state.steerYawCoherence)
        * M.params.authorityGain,
        M.params.minAuthority,
        M.params.maxAuthority
    )

    local steerAbs = clamp(abs(state.steer), 0.0, 1.0)
    local yawAbs = clamp(abs(state.yawRate) * M.params.yawRateScale, 0.0, 1.0)

    state.targetYawResist = clamp(
        state.targetAnchor
        * yawAbs
        * (0.50 + 0.50 * state.steerYawCoherence)
        * M.params.yawResistGain,
        M.params.minYawResist,
        M.params.maxYawResist
    )

    state.targetSteerWeight = clamp(
        state.targetAnchor
        * steerAbs
        * M.params.steerWeightGain,
        M.params.minSteerWeight,
        M.params.maxSteerWeight
    )

    state.targetSlipDamp = clamp(
        state.frontSlipLoss * state.targetAnchor,
        M.params.minSlipDamp,
        M.params.maxSlipDamp
    )

    state.frontAxisReserve = clamp(
        1.0 - math.max(state.targetSlipDamp, state.frontSlipLoss),
        0.0,
        1.0
    )
end

local function filterOutputs(dt)
    state.frontAxisAnchor = lowPass(
        state.frontAxisAnchor,
        state.targetAnchor,
        M.params.anchorTau,
        dt
    )

    state.frontAxisAuthority = lowPass(
        state.frontAxisAuthority,
        state.targetAuthority,
        M.params.authorityTau,
        dt
    )

    state.frontAxisYawResist = lowPass(
        state.frontAxisYawResist,
        state.targetYawResist,
        M.params.yawResistTau,
        dt
    )

    state.frontAxisSteerWeight = lowPass(
        state.frontAxisSteerWeight,
        state.targetSteerWeight,
        M.params.steerWeightTau,
        dt
    )

    state.frontAxisSlipDamp = lowPass(
        state.frontAxisSlipDamp,
        state.targetSlipDamp,
        M.params.slipDampTau,
        dt
    )
end

local function decayNoCar(dt)
    state.targetAnchor = 0.0
    state.targetAuthority = 0.0
    state.targetYawResist = 0.0
    state.targetSteerWeight = 0.0
    state.targetSlipDamp = 0.0

    state.frontAxisAnchor = lowPass(state.frontAxisAnchor, 0.0, M.params.noCarReturnTau, dt)
    state.frontAxisAuthority = lowPass(state.frontAxisAuthority, 0.0, M.params.noCarReturnTau, dt)
    state.frontAxisYawResist = lowPass(state.frontAxisYawResist, 0.0, M.params.noCarReturnTau, dt)
    state.frontAxisSteerWeight = lowPass(state.frontAxisSteerWeight, 0.0, M.params.noCarReturnTau, dt)
    state.frontAxisSlipDamp = lowPass(state.frontAxisSlipDamp, 0.0, M.params.noCarReturnTau, dt)
    state.frontAxisReserve = lowPass(state.frontAxisReserve, 0.0, M.params.noCarReturnTau, dt)
end

local function exportState()
    safeStore("ngp_mass_balance_status", state.status or "UNKNOWN")
    safeStore("ngp_mass_balance_update_count", state.updateCount or 0)
    safeStore("ngp_mass_balance_init_count", state.initCount or 0)

    safeStore("ngp_mass_balance_applied", 0)
    safeStore("ngp_mass_balance_rear_applied", 0)
    safeStore("ngp_mass_balance_front_applied", 0)
    safeStore("ngp_mass_balance_yaw_applied", 0)

    safeStore("ngp_mass_balance_total_added", 0.0)
    safeStore("ngp_mass_balance_rear_bias", 0.0)
    safeStore("ngp_mass_balance_front_bias", 0.0)
    safeStore("ngp_mass_balance_yaw_inertia", 0.0)

    safeStore("ngp_vehicle_base_mass", state.baseMass or BASE_MASS)
    safeStore("ngp_vehicle_mass_estimate", state.totalMassEstimate or state.baseMass or BASE_MASS)
    safeStore("ngp_vehicle_mass_scale", state.massScale or 1.0)
    safeStore("ngp_vehicle_cg_rear_shift", 0.0)
    safeStore("ngp_vehicle_yaw_inertia_scale", 1.0)

    safeStore("ngp_front_axis_anchor", state.frontAxisAnchor or 0.0)
    safeStore("ngp_front_axis_authority", state.frontAxisAuthority or 0.0)
    safeStore("ngp_front_axis_yaw_resist", state.frontAxisYawResist or 0.0)
    safeStore("ngp_front_axis_steer_weight", state.frontAxisSteerWeight or 0.0)
    safeStore("ngp_front_axis_slip_damp", state.frontAxisSlipDamp or 0.0)

    safeStore("ngp_front_axis_load_coherence", state.frontLoadCoherence or 0.0)
    safeStore("ngp_front_axis_contact_balance", state.frontContactBalance or 0.0)
    safeStore("ngp_front_axis_load_balance", state.frontLoadBalance or 0.0)
    safeStore("ngp_front_axis_slip_loss", state.frontSlipLoss or 0.0)
    safeStore("ngp_front_axis_steer_yaw_coherence", state.steerYawCoherence or 0.0)
    safeStore("ngp_front_axis_front_load_avg", state.frontLoadAvg or 0.0)

    safeStore("ngp_front_axis_reserve", state.frontAxisReserve or 0.0)
    safeStore("ngp_front_axis_load_quality", state.frontAxisLoadQuality or 0.0)
    safeStore("ngp_front_axis_contact_quality", state.frontAxisContactQuality or 0.0)
    safeStore("ngp_front_axis_front_load_total", state.frontLoadTotal or 0.0)
    safeStore("ngp_front_axis_front_load_diff", state.frontLoadDiff or 0.0)
    safeStore("ngp_front_axis_contact_avg", state.frontContactAvg or 0.0)
    safeStore("ngp_front_axis_contact_diff", state.frontContactDiff or 0.0)
    safeStore("ngp_front_axis_slip_avg", state.frontSlipAvg or 0.0)

    safeStore("ngp_faa_anchor", state.frontAxisAnchor or 0.0)
    safeStore("ngp_faa_authority", state.frontAxisAuthority or 0.0)
    safeStore("ngp_faa_yaw_resist", state.frontAxisYawResist or 0.0)
    safeStore("ngp_faa_steer_weight", state.frontAxisSteerWeight or 0.0)
    safeStore("ngp_faa_slip_damp", state.frontAxisSlipDamp or 0.0)

    safeStore("ngp_front_axis_load_linked", state.loadLinked and 1 or 0)
    safeStore("ngp_front_axis_contact_linked", state.contactLinked and 1 or 0)
    safeStore("ngp_front_axis_tire_state_linked", state.tireStateLinked and 1 or 0)
    safeStore("ngp_front_axis_car_linked", state.carLinked and 1 or 0)
    safeStore("ngp_front_axis_wheels_valid", state.wheelsValid and 1 or 0)

    safeStore("ngp_mass_state", 0)
    safeStore("ngp_yaw_inertia", 0.0)
    safeStore("ngp_extra_mass_front", 0.0)
    safeStore("ngp_extra_mass_rear", 0.0)

    if not state.debugStoreNow then return end

    safeStore("ngp_front_axis_load_fl", state.load[FL] or 0.0)
    safeStore("ngp_front_axis_load_fr", state.load[FR] or 0.0)
    safeStore("ngp_front_axis_contact_fl", state.contact[FL] or 0.0)
    safeStore("ngp_front_axis_contact_fr", state.contact[FR] or 0.0)
    safeStore("ngp_front_axis_slip_angle_fl", state.slipAngle[FL] or 0.0)
    safeStore("ngp_front_axis_slip_angle_fr", state.slipAngle[FR] or 0.0)
    safeStore("ngp_front_axis_slip_ratio_fl", state.slipRatio[FL] or 0.0)
    safeStore("ngp_front_axis_slip_ratio_fr", state.slipRatio[FR] or 0.0)
    safeStore("ngp_front_axis_yaw_rate", state.yawRate or 0.0)
    safeStore("ngp_front_axis_steer", state.steer or 0.0)
    safeStore("ngp_front_axis_speed_kmh", state.speedKmh or 0.0)

    for i = 0, 3 do
        safeStore("ngp_front_axis_load_" .. i, state.load[i] or 0.0)
        safeStore("ngp_front_axis_contact_" .. i, state.contact[i] or 0.0)
        safeStore("ngp_front_axis_slip_angle_" .. i, state.slipAngle[i] or 0.0)
        safeStore("ngp_front_axis_slip_ratio_" .. i, state.slipRatio[i] or 0.0)
        safeStore("ngp_front_axis_load_source_" .. i, state.loadSource[i] or "NONE")
        safeStore("ngp_front_axis_contact_source_" .. i, state.contactSource[i] or "NONE")
        safeStore("ngp_front_axis_slip_source_" .. i, state.slipSource[i] or "NONE")
    end
end

function M.init(car)
    state.initCount = (state.initCount or 0) + 1

    local currentCar = car or safeGetCar()
    readCarState(currentCar)

    state.status = currentCar and "FRONT AXIS INIT" or "INIT NO CAR"
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
    resetLinkFlags()

    local currentCar = car or safeGetCar()

    if not currentCar then
        state.status = "NO CAR"
        readCarState(nil)
        decayNoCar(dt)
        exportState()
        return
    end

    readCarState(currentCar)
    readWheels(currentCar)

    if state.wheelsValid then
        state.status = "RUNNING"
    else
        state.status = "STORE ONLY"
    end

    calculateFrontAxisTargets()
    filterOutputs(dt)

    exportState()
end

function M.getFrontAxisAnchor()
    return state.frontAxisAnchor or 0.0
end

function M.getFrontAxisAuthority()
    return state.frontAxisAuthority or 0.0
end

function M.getFrontAxisYawResist()
    return state.frontAxisYawResist or 0.0
end

function M.getFrontAxisSteerWeight()
    return state.frontAxisSteerWeight or 0.0
end

function M.getFrontAxisSlipDamp()
    return state.frontAxisSlipDamp or 0.0
end

function M.getMassScale()
    return 1.0
end

function M.getBaseMass()
    return state.baseMass or BASE_MASS
end

function M.getWheelLoad(index)
    return state.load[tonumber(index) or 0] or 0.0
end

function M.getState()
    return state
end

function M.isApplied()
    return false
end

function M.debugStr(index)
    if index ~= nil then
        local i = tonumber(index) or 0
        return string.format(
            "%s Load %.0f Contact %.2f SA %.3f SR %.3f\n" ..
            "Src L:%s C:%s S:%s",
            tostring(WHEEL_NAMES[i] or i),
            state.load[i] or 0.0,
            state.contact[i] or 0.0,
            state.slipAngle[i] or 0.0,
            state.slipRatio[i] or 0.0,
            tostring(state.loadSource[i] or "NONE"),
            tostring(state.contactSource[i] or "NONE"),
            tostring(state.slipSource[i] or "NONE")
        )
    end

    return string.format(
        "Status %s / Count %.0f / Init %.0f\n" ..
        "Mass %.0f / Scale %.3f / ExtraMass OFF\n" ..
        "FrontAxis Anchor %.3f / Authority %.3f\n" ..
        "YawResist %.3f / SteerWeight %.3f / SlipDamp %.3f\n" ..
        "LoadCoh %.3f / ContactBal %.3f / SlipLoss %.3f / Reserve %.3f\n" ..
        "Load FL/FR %.0f %.0f Contact %.2f %.2f\n" ..
        "Links Load:%s Contact:%s Tire:%s Car:%s Wheels:%s",
        tostring(state.status),
        state.updateCount or 0,
        state.initCount or 0,
        state.baseMass or BASE_MASS,
        state.massScale or 1.0,
        state.frontAxisAnchor or 0.0,
        state.frontAxisAuthority or 0.0,
        state.frontAxisYawResist or 0.0,
        state.frontAxisSteerWeight or 0.0,
        state.frontAxisSlipDamp or 0.0,
        state.frontLoadCoherence or 0.0,
        state.frontContactBalance or 0.0,
        state.frontSlipLoss or 0.0,
        state.frontAxisReserve or 0.0,
        state.load[FL] or 0.0,
        state.load[FR] or 0.0,
        state.contact[FL] or 0.0,
        state.contact[FR] or 0.0,
        state.loadLinked and "OK" or "NIL",
        state.contactLinked and "OK" or "NIL",
        state.tireStateLinked and "OK" or "NIL",
        state.carLinked and "OK" or "NIL",
        state.wheelsValid and "OK" or "NIL"
    )
end

return M
