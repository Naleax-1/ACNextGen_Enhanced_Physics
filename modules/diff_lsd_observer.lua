---@diagnostic disable: undefined-global

-- diff_lsd_observer.lua
-- ACNextGen V1.1.5 Stable
-- Phase 2.5 / LSD Mechanical and Fallback Diagnostic Panel

local M = {}

M.params = {
    observerStoreInterval = 0.25,
    barWidth = 24,
    pathMaxLen = 58,
    staleUpdateFrames = 0,
}

local state = {
    status = "INIT",
    updateCount = 0,
    lastDiagnosis = "INIT",
    lastSnapshot = nil,
    debugStoreTimer = 999.0,
    debugStoreNow = true,
}

M.state = state
M.debug = state

local function safeNumber(value, defaultValue)
    local n = tonumber(value)
    if n == nil or n ~= n then
        return defaultValue or 0.0
    end
    if n == math.huge or n == -math.huge then
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

local function abs(v)
    v = safeNumber(v, 0.0)
    return v < 0.0 and -v or v
end

local function safeLoadRaw(key)
    if not ac or not ac.load then return nil end
    local ok, value = pcall(function()
        return ac.load(key)
    end)
    if not ok then return nil end
    return value
end

local function safeLoadNumber(key, defaultValue)
    local value = safeLoadRaw(key)
    if value == nil then return defaultValue or 0.0 end
    return safeNumber(value, defaultValue or 0.0)
end

local function safeLoadString(key, defaultValue)
    local value = safeLoadRaw(key)
    if value == nil then return defaultValue or "NIL" end
    return tostring(value)
end

local function safeStore(key, value)
    if not ac or not ac.store then return end
    pcall(function()
        ac.store(key, value)
    end)
end

local function updateDebugGate(dt)
    state.debugStoreTimer = (state.debugStoreTimer or 0.0) + (dt or 0.0)
    if state.debugStoreTimer >= M.params.observerStoreInterval then
        state.debugStoreTimer = 0.0
        state.debugStoreNow = true
    else
        state.debugStoreNow = false
    end
end

local function uiReady()
    return ui and ui.text
end

local function uiText(text)
    if uiReady() then
        ui.text(tostring(text or ""))
    end
end

local function uiSeparator()
    if ui and ui.separator then
        ui.separator()
    else
        uiText("----------------------------------------")
    end
end

local function sourceMark(source)
    source = tostring(source or "NIL")
    if source == "INI_MECHANICAL" or source == "INI" then
        return "OK"
    end
    if source == "OBSERVED_FALLBACK" then
        return "FALLBACK"
    end
    if source == "FALLBACK" then
        return "FALLBACK"
    end
    if source == "UNLOADED" or source == "NIL" or source == "" then
        return "NIL"
    end
    return source
end

local function fmtPct01(v)
    return string.format("%.1f%%", clamp(v, 0.0, 1.0) * 100.0)
end

local function fmtNm(v)
    return string.format("%.0f Nm", safeNumber(v, 0.0))
end

local function fmtNum(v, digits)
    digits = digits or 3
    return string.format("%." .. tostring(digits) .. "f", safeNumber(v, 0.0))
end

local function shortPath(path)
    path = tostring(path or "")
    local maxLen = M.params.pathMaxLen or 58
    if #path <= maxLen then
        return path
    end
    return "..." .. string.sub(path, #path - maxLen + 4)
end

local function drawBar(label, value, minValue, maxValue, width)
    value = safeNumber(value, 0.0)
    minValue = safeNumber(minValue, 0.0)
    maxValue = safeNumber(maxValue, 1.0)
    width = width or M.params.barWidth or 24

    if maxValue <= minValue then
        maxValue = minValue + 1.0
    end

    local ratio = clamp((value - minValue) / (maxValue - minValue), 0.0, 1.0)
    local filled = math.floor(ratio * width + 0.5)
    local s = ""

    for i = 1, width do
        if i <= filled then
            s = s .. "#"
        else
            s = s .. "-"
        end
    end

    uiText(string.format("%-20s [%s] %.3f", tostring(label), s, value))
end

local function drawPair(label, a, b)
    uiText(string.format("%-20s %s / %s", tostring(label), tostring(a), tostring(b)))
end

local function loadFirstNumber(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return safeNumber(value, defaultValue or 0.0), keys[i]
        end
    end
    return defaultValue or 0.0, nil
end

local function loadFirstString(defaultValue, ...)
    local keys = { ... }
    for i = 1, #keys do
        local value = safeLoadRaw(keys[i])
        if value ~= nil then
            return tostring(value), keys[i]
        end
    end
    return defaultValue or "NIL", nil
end

function M.readSnapshot()
    local s = {}

    s.status = safeLoadString("ngp_lsd_status", "NIL")
    s.updateCount = safeLoadNumber("ngp_lsd_update_count", 0.0)

    s.source = safeLoadString("ngp_lsd_source", safeLoadString("ngp_lsd_config_source", "NIL"))
    s.configSource = safeLoadString("ngp_lsd_config_source", "NIL")
    s.configStatus = safeLoadString("ngp_lsd_config_status", "NIL")
    s.configWarning = safeLoadString("ngp_lsd_config_warning", "NONE")
    s.configPath = safeLoadString("ngp_lsd_config_path", "")
    s.drivetrainType = safeLoadString("ngp_lsd_drivetrain_type", "UNKNOWN")

    s.mode = safeLoadString("ngp_lsd_mode", "NIL")
    s.lock = safeLoadNumber("ngp_lsd_lock", safeLoadNumber("ngp_diff_lock", 0.0))
    s.target = safeLoadNumber("ngp_lsd_target", 0.0)
    s.mechanicalLock = safeLoadNumber("ngp_lsd_mechanical_lock", s.lock)
    s.observedLock = safeLoadNumber("ngp_lsd_observed_lock", 0.0)

    s.diff = safeLoadNumber("ngp_lsd_diff", safeLoadNumber("ngp_diff_diff", 0.0))
    s.signedDiff = safeLoadNumber("ngp_lsd_sdiff", safeLoadNumber("ngp_diff_sdiff", 0.0))
    s.diffFiltered = safeLoadNumber("ngp_lsd_diff_filtered", s.diff)
    s.signedDiffFiltered = safeLoadNumber("ngp_lsd_sdiff_filtered", s.signedDiff)

    s.omegaL = safeLoadNumber("ngp_lsd_omega_l", 0.0)
    s.omegaR = safeLoadNumber("ngp_lsd_omega_r", 0.0)
    s.rawOmegaL = safeLoadNumber("ngp_lsd_raw_omega_l", s.omegaL)
    s.rawOmegaR = safeLoadNumber("ngp_lsd_raw_omega_r", s.omegaR)

    s.inputTorque = safeLoadNumber("ngp_lsd_input_torque_nm", safeLoadNumber("ngp_diff_input_torque_nm", 0.0))
    s.lockTorque = safeLoadNumber("ngp_lsd_lock_torque_nm", safeLoadNumber("ngp_lsd_lock_torque", 0.0))
    s.preloadTorque = safeLoadNumber("ngp_lsd_preload_torque_nm", safeLoadNumber("ngp_lsd_preload_torque", safeLoadNumber("ngp_diff_preload", 0.0)))
    s.camTorque = safeLoadNumber("ngp_lsd_cam_torque_nm", safeLoadNumber("ngp_lsd_cam_torque", 0.0))
    s.resistTorque = safeLoadNumber("ngp_lsd_resist_torque_nm", safeLoadNumber("ngp_lsd_resist_torque", 0.0))
    s.klockPercent = safeLoadNumber("ngp_lsd_klock_percent", s.lock * 100.0)

    s.powerRatio = safeLoadNumber("ngp_lsd_power_ratio", safeLoadNumber("ngp_diff_power", 0.0))
    s.coastRatio = safeLoadNumber("ngp_lsd_coast_ratio", safeLoadNumber("ngp_diff_coast", 0.0))
    s.activeRatio = safeLoadNumber("ngp_lsd_active_ratio", 0.0)
    s.camPowerDeg = safeLoadNumber("ngp_lsd_cam_power_deg", 0.0)
    s.camCoastDeg = safeLoadNumber("ngp_lsd_cam_coast_deg", 0.0)
    s.activeCamDeg = safeLoadNumber("ngp_lsd_active_cam_deg", 0.0)

    s.finalRatio = safeLoadNumber("ngp_lsd_final_ratio", 0.0)
    s.gear = safeLoadNumber("ngp_lsd_gear", safeLoadNumber("ngp_drive_gear", 0.0))
    s.gearRatio = safeLoadNumber("ngp_lsd_gear_ratio", 0.0)

    s.loadDiff = safeLoadNumber("ngp_lsd_load_diff", 0.0)
    s.loadRL = safeLoadNumber("ngp_lsd_load_rl", 0.0)
    s.loadRR = safeLoadNumber("ngp_lsd_load_rr", 0.0)

    s.driveTorque = safeLoadNumber("ngp_lsd_drive_torque", safeLoadNumber("ngp_drive_torque", 0.0))
    s.driveTorqueNm = safeLoadNumber("ngp_lsd_drive_torque_nm", safeLoadNumber("ngp_drive_torque_nm", 0.0))
    s.driveTorqueScale = safeLoadNumber("ngp_lsd_drive_torque_scale", 1.0)
    s.torqueSource = safeLoadString("ngp_lsd_torque_source", "NIL")

    s.shaftTwist = safeLoadNumber("ngp_lsd_shaft_twist", safeLoadNumber("ngp_shaft_twist", 0.0))
    s.shiftShock = safeLoadNumber("ngp_lsd_shift_shock", safeLoadNumber("ngp_shift_shock", 0.0))
    s.contactLoss = safeLoadNumber("ngp_lsd_contact_loss", safeLoadNumber("ngp_drive_contact_loss", safeLoadNumber("ngp_contact_loss_avg", 0.0)))
    s.hopShock = safeLoadNumber("ngp_lsd_hop_shock", safeLoadNumber("ngp_drive_hop_shock", safeLoadNumber("ngp_tire_hop_avg_instability", 0.0)))
    s.armTwist = safeLoadNumber("ngp_lsd_arm_twist", safeLoadNumber("ngp_control_arm_total", safeLoadNumber("ngp_arm_total", 0.0)))
    s.heat = safeLoadNumber("ngp_lsd_heat", safeLoadNumber("ngp_diff_heat", 0.0))

    s.forceL = safeLoadNumber("ngp_diff_forceL", safeLoadNumber("ngp_lsd_force_l", 0.0))
    s.forceR = safeLoadNumber("ngp_diff_forceR", safeLoadNumber("ngp_lsd_force_r", 0.0))
    s.diffPower = safeLoadNumber("ngp_diff_power", s.powerRatio)
    s.diffCoast = safeLoadNumber("ngp_diff_coast", s.coastRatio)
    s.diffPreload = safeLoadNumber("ngp_diff_preload", s.preloadTorque)

    s.drivetrainStatus = safeLoadString("ngp_drivetrain_status", "NIL")
    s.drivetrainUpdateCount = safeLoadNumber("ngp_drivetrain_update_count", 0.0)
    s.driveEngineTorque = safeLoadNumber("ngp_drive_engine_torque", 0.0)
    s.driveTransmittedTorque = safeLoadNumber("ngp_drive_transmitted_torque", 0.0)
    s.driveFinalTorqueScale = safeLoadNumber("ngp_drive_final_torque_scale", 1.0)
    s.lsdResistance = safeLoadNumber("ngp_lsd_resistance", 0.0)

    s.contactQualityRL = safeLoadNumber("ngp_contact_quality_2", 1.0)
    s.contactQualityRR = safeLoadNumber("ngp_contact_quality_3", 1.0)
    s.contactTrustRL = safeLoadNumber("ngp_contact_trust_2", s.contactQualityRL)
    s.contactTrustRR = safeLoadNumber("ngp_contact_trust_3", s.contactQualityRR)
    s.rearContactQuality = clamp((s.contactQualityRL + s.contactQualityRR) * 0.5, 0.0, 1.2)
    s.rearContactTrust = clamp((s.contactTrustRL + s.contactTrustRR) * 0.5, 0.0, 1.2)

    return s
end

function M.diagnose(s)
    s = s or M.readSnapshot()
    local notes = {}

    if s.updateCount <= 0 then
        notes[#notes + 1] = "LSD UPDATE MISSING"
    elseif s.status ~= "RUNNING" then
        notes[#notes + 1] = "LSD " .. tostring(s.status)
    end

    if s.drivetrainUpdateCount > 0 and s.drivetrainStatus ~= "RUNNING" then
        notes[#notes + 1] = "DRIVETRAIN " .. tostring(s.drivetrainStatus)
    elseif s.drivetrainUpdateCount <= 0 then
        notes[#notes + 1] = "DRIVETRAIN NIL"
    end

    if s.source == "OBSERVED_FALLBACK" or s.configSource == "OBSERVED_FALLBACK" then
        notes[#notes + 1] = "INI fallback"
    end

    if s.configWarning ~= "NONE" and s.configWarning ~= "NIL" and s.configWarning ~= "" then
        notes[#notes + 1] = tostring(s.configWarning)
    end

    if abs(s.inputTorque) < 1.0 and abs(s.driveTorque) > 0.05 then
        notes[#notes + 1] = "Tin low vs driveTorque"
    end

    if abs(s.inputTorque) > 1.0 and s.lockTorque <= 0.0 and s.lock > 0.03 then
        notes[#notes + 1] = "lock exists but Tlock low"
    end

    if s.diff > 8.0 and s.lock < 0.10 and s.mode == "POWER" then
        notes[#notes + 1] = "POWER diff high / lock weak"
    end

    if s.diff <= 0.001 and abs(s.rawOmegaL - s.rawOmegaR) > 0.25 then
        notes[#notes + 1] = "filtered diff lost raw omega split"
    end

    if abs(s.omegaL) < 0.001 and abs(s.omegaR) < 0.001 and s.status == "RUNNING" then
        notes[#notes + 1] = "omega zero or low-speed hold"
    end

    if s.loadDiff > 2500.0 then
        notes[#notes + 1] = "rear load split high"
    end

    if s.hopShock > 0.45 then
        notes[#notes + 1] = "hop unlock active"
    end

    if s.contactLoss > 0.35 then
        notes[#notes + 1] = "contact loss active"
    end

    if s.rearContactTrust < 0.45 then
        notes[#notes + 1] = "rear contact trust low"
    end

    if #notes == 0 then
        notes[#notes + 1] = "OK"
    end

    return table.concat(notes, " / ")
end

local function exportObserverState(snapshot, diagnosis)
    safeStore("ngp_lsd_observer_status", state.status)
    safeStore("ngp_lsd_observer_update_count", state.updateCount)
    safeStore("ngp_lsd_observer_diagnosis", diagnosis or "UNKNOWN")
    safeStore("ngp_lsd_observer_lock", snapshot and snapshot.lock or 0.0)
    safeStore("ngp_lsd_observer_diff", snapshot and snapshot.diff or 0.0)
    safeStore("ngp_lsd_observer_tin", snapshot and snapshot.inputTorque or 0.0)

    if not state.debugStoreNow then
        return
    end

    safeStore("ngp_lsd_observer_source", snapshot and snapshot.source or "NIL")
    safeStore("ngp_lsd_observer_config_source", snapshot and snapshot.configSource or "NIL")
    safeStore("ngp_lsd_observer_rear_trust", snapshot and snapshot.rearContactTrust or 0.0)
    safeStore("ngp_lsd_observer_load_diff", snapshot and snapshot.loadDiff or 0.0)
end

function M.init()
    state.status = "INIT"
    local s = M.readSnapshot()
    local d = M.diagnose(s)
    state.lastSnapshot = s
    state.lastDiagnosis = d
    exportObserverState(s, d)
end

function M.update(dt, car, runtime)
    state.updateCount = (state.updateCount or 0) + 1
    dt = safeNumber(dt, 0.0)
    if dt <= 0.0 then
        dt = 1.0 / 60.0
    end

    updateDebugGate(dt)

    local s = M.readSnapshot()
    local d = M.diagnose(s)

    state.status = "RUNNING"
    state.lastSnapshot = s
    state.lastDiagnosis = d

    exportObserverState(s, d)
end

function M.drawCompact()
    if not uiReady() then return end

    local s = M.readSnapshot()
    local d = M.diagnose(s)

    uiText("=== LSD OBSERVER V1.1.5 ===")
    uiText(string.format(
        "Status %s / Count %.0f / Source %s",
        tostring(s.status),
        s.updateCount,
        sourceMark(s.source)
    ))

    uiText(string.format(
        "Lock %s / Target %s / Mode %s / K %.1f%%",
        fmtPct01(s.lock),
        fmtPct01(s.target),
        tostring(s.mode),
        s.klockPercent
    ))

    uiText(string.format(
        "Tin %s / Tlock %s / Pre %s / Cam %s",
        fmtNm(s.inputTorque),
        fmtNm(s.lockTorque),
        fmtNm(s.preloadTorque),
        fmtNm(s.camTorque)
    ))

    uiText(string.format(
        "Diff %.3f / S %.3f / RL %.2f RR %.2f / Heat %.1f",
        s.diff,
        s.signedDiff,
        s.omegaL,
        s.omegaR,
        s.heat
    ))

    uiText("Diag: " .. d)
end

function M.draw()
    if not uiReady() then return end

    local s = M.readSnapshot()
    local d = M.diagnose(s)

    uiText("================================================")
    uiText(" ACNextGen LSD Observer V1.1.5")
    uiText("================================================")

    uiText(string.format("Status      : %s / Count %.0f", tostring(s.status), s.updateCount))
    uiText(string.format("Source      : %s / Config %s / %s", sourceMark(s.source), sourceMark(s.configSource), tostring(s.configStatus)))
    uiText(string.format("Warning     : %s", tostring(s.configWarning)))
    uiText(string.format("Mode        : %s / Gear %.0f / Type %s", tostring(s.mode), s.gear, tostring(s.drivetrainType)))

    if s.configPath ~= "" then
        uiText("ConfigPath  : " .. shortPath(s.configPath))
    end

    uiText("")
    uiText("--- LOCK STATE ---")
    drawBar("Lock", s.lock, 0.0, 1.0)
    drawBar("Target", s.target, 0.0, 1.0)
    drawBar("Mechanical", s.mechanicalLock, 0.0, 1.0)
    drawBar("Observed", s.observedLock, 0.0, 1.0)
    uiText(string.format("Klock       : %.1f%%", s.klockPercent))

    uiText("")
    uiText("--- MECHANICAL TORQUE MODEL ---")
    uiText(string.format("Tin         : %s / Src %s", fmtNm(s.inputTorque), tostring(s.torqueSource)))
    uiText(string.format("Tlock       : %s", fmtNm(s.lockTorque)))
    uiText(string.format("Tpreload    : %s", fmtNm(s.preloadTorque)))
    uiText(string.format("Tcam        : %s", fmtNm(s.camTorque)))
    uiText(string.format("Tresist     : %s", fmtNm(s.resistTorque)))
    uiText(string.format("Force L/R   : %.3f / %.3f", s.forceL, s.forceR))

    uiText("")
    uiText("--- CONFIG / GEOMETRY ---")
    uiText(string.format("Power/Coast : %.3f / %.3f  Active %.3f", s.powerRatio, s.coastRatio, s.activeRatio))
    uiText(string.format("Cam P/C/Act : %.1f / %.1f / %.1f deg", s.camPowerDeg, s.camCoastDeg, s.activeCamDeg))
    uiText(string.format("Final/GearR : %.3f / %.3f", s.finalRatio, s.gearRatio))
    uiText(string.format("Diff aliases: P %.3f C %.3f Pre %.0f", s.diffPower, s.diffCoast, s.diffPreload))

    uiText("")
    uiText("--- DIFF / WHEEL STATE ---")
    uiText(string.format("Diff        : %.3f / Filt %.3f", s.diff, s.diffFiltered))
    uiText(string.format("Signed      : %.3f / Filt %.3f", s.signedDiff, s.signedDiffFiltered))
    drawPair("Omega L/R", string.format("%.2f", s.omegaL), string.format("%.2f", s.omegaR))
    drawPair("RawOmega L/R", string.format("%.2f", s.rawOmegaL), string.format("%.2f", s.rawOmegaR))
    drawPair("Load RL/RR", string.format("%.0f", s.loadRL), string.format("%.0f", s.loadRR))
    uiText(string.format("RearLoadDiff: %.0f N", s.loadDiff))
    uiText(string.format("RearContact : Q %.2f / Trust %.2f", s.rearContactQuality, s.rearContactTrust))

    uiText("")
    uiText("--- DISTURBANCE / UNLOCK FACTORS ---")
    drawBar("ContactLoss", s.contactLoss, 0.0, 1.0)
    drawBar("HopShock", s.hopShock, 0.0, 1.0)
    drawBar("ShiftShock", s.shiftShock, 0.0, 1.0)
    drawBar("ArmTwist", s.armTwist, 0.0, 1.0)

    uiText("")
    uiText("--- DRIVETRAIN LINK ---")
    uiText(string.format("Status      : %s / Count %.0f", tostring(s.drivetrainStatus), s.drivetrainUpdateCount))
    uiText(string.format("Engine/Trans/Drive : %.3f / %.3f / %.3f", s.driveEngineTorque, s.driveTransmittedTorque, s.driveTorque))
    uiText(string.format("FinalScale / LSDRes : %.3f / %.3f", s.driveFinalTorqueScale, s.lsdResistance))

    uiText("")
    uiText("--- DIAGNOSIS ---")
    uiText(d)
end

function M.drawUI()
    M.draw()
end

function M.getSnapshot()
    return state.lastSnapshot or M.readSnapshot()
end

function M.getDiagnosis()
    return state.lastDiagnosis or M.diagnose(M.readSnapshot())
end

function M.debugStr()
    local s = M.readSnapshot()
    local d = M.diagnose(s)

    return string.format(
        "LSD %s Src:%s Mode:%s\n" ..
        "Lock %.1f%% Target %.1f%% K %.1f%%\n" ..
        "Tin %.0f Tlock %.0f Pre %.0f Cam %.0f\n" ..
        "Diff %.3f S %.3f Heat %.1f\n" ..
        "Omega %.2f / %.2f Raw %.2f / %.2f LoadDiff %.0f\n" ..
        "RearTrust %.2f Diag %s",
        tostring(s.status),
        sourceMark(s.source),
        tostring(s.mode),
        s.lock * 100.0,
        s.target * 100.0,
        s.klockPercent,
        s.inputTorque,
        s.lockTorque,
        s.preloadTorque,
        s.camTorque,
        s.diff,
        s.signedDiff,
        s.heat,
        s.omegaL,
        s.omegaR,
        s.rawOmegaL,
        s.rawOmegaR,
        s.loadDiff,
        s.rearContactTrust,
        d
    )
end

return M
