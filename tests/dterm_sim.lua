-- Headless check of the PD drive's D term. Replicates speedFor() and the
-- updateRates() error-derivative EMA from cannon.lua VERBATIM, drives a
-- velocity-actuator mount with one tick of command latency (the overshoot
-- source), and confirms: (1) kd = 0 reproduces the pure P+ff loop, and
-- (2) kd > 0 reduces the step-response overshoot (correct damping sign).
--
-- Run from the repo root:  lua tests/dterm_sim.lua

local fails = 0
local function check(name, ok, detail)
  if ok then print("ok   " .. name)
  else fails = fails + 1; print("FAIL " .. name .. (detail and (": " .. detail) or "")) end
end

-- ---- speedFor, copied verbatim from cannon.lua ----
local function speedFor(diff, invert, drive, ffRate, dRate)
  local ff, d = 0, 0
  local degPerSec = drive.degPerSecPerRpm
  if type(degPerSec) == "number" and degPerSec > 0 then
    if ffRate then ff = ffRate / degPerSec end
    if dRate and drive.kd and drive.kd ~= 0 then
      d = drive.kd * dRate / degPerSec
    end
  end
  local p = math.abs(diff) * drive.speedGain
  local rpm = 0
  if p >= drive.minSpeed or math.abs(ff) > 0.5 then
    rpm = math.min(p, drive.maxSpeed)
    if diff < 0 then rpm = -rpm end
    rpm = rpm + ff + d
    if math.abs(rpm) < drive.minSpeed then
      rpm = rpm >= 0 and drive.minSpeed or -drive.minSpeed
    end
  end
  rpm = math.max(-drive.maxSpeed, math.min(rpm, drive.maxSpeed))
  if invert then rpm = -rpm end
  return rpm
end

-- ---- error-rate EMA, copied from updateRates() ----
local function newTrack() return { errRate = 0, errPrev = nil, lastT = nil } end
local function updateErrRate(tr, err, t)
  if tr.lastT then
    local dt = t - tr.lastT
    if dt > 0 and dt < 0.5 then
      local a = dt / (0.15 + dt)
      if tr.errPrev then
        tr.errRate = tr.errRate + a * ((err - tr.errPrev) / dt - tr.errRate)
      end
    elseif dt >= 0.5 then
      tr.errRate = 0
    end
  end
  tr.lastT, tr.errPrev = t, err
end

-- ---- velocity-actuator mount with rotational inertia (first-order lag) ----
-- The speed controller sets a TARGET angular velocity; the geared mount ramps
-- toward it (omega += (cmd - omega)*RAMP) rather than reaching it instantly.
-- That lag -- plus coasting on after the drive parks at the minSpeed floor --
-- is what overshoots a pure-P loop in game, and what the D term damps by
-- bleeding off approach velocity before the barrel reaches the target.
local DPS = 0.75          -- deg/s per RPM (direct-drive mount)
local DT = 0.05           -- one game tick
local RAMP = 0.3          -- fraction of the speed gap closed per tick (inertia)
local function simulate(kd, setpoint)
  local drive = { speedGain = 6, maxSpeed = 120, minSpeed = 1,
    degPerSecPerRpm = DPS, kd = kd }
  local tr = newTrack()
  local theta, omega = 0, 0   -- mount angle and current angular velocity
  local t = 0
  local peakOvershoot = 0
  local samples = {}
  for i = 1, 600 do
    local err = setpoint - theta
    updateErrRate(tr, err, t)
    local rpm = speedFor(err, false, drive, 0, tr.errRate)
    local omegaCmd = rpm * DPS
    omega = omega + (omegaCmd - omega) * RAMP   -- inertia: ramp toward target
    theta = theta + omega * DT
    if theta > setpoint then
      peakOvershoot = math.max(peakOvershoot, theta - setpoint)
    end
    samples[#samples + 1] = theta
    t = t + DT
  end
  return { overshoot = peakOvershoot, final = samples[#samples], samples = samples }
end

-- A 30 deg step at gain 6 overshoots noticeably with no damping.
local base = simulate(0, 30)
local damped = simulate(0.4, 30)

check("kd=0 overshoots (latency-driven)", base.overshoot > 0.5,
  ("overshoot %.2f deg"):format(base.overshoot))
check("kd=0.4 cuts the overshoot at least in half",
  damped.overshoot < base.overshoot * 0.5,
  ("%.2f -> %.2f deg"):format(base.overshoot, damped.overshoot))
check("both still converge to the setpoint",
  math.abs(base.final - 30) < 1 and math.abs(damped.final - 30) < 1,
  ("final %.2f / %.2f"):format(base.final, damped.final))

-- kd = 0 must be byte-for-byte the old P+ff path: D contributes nothing.
local d0 = speedFor(5, false, { speedGain = 6, maxSpeed = 120, minSpeed = 1,
  degPerSecPerRpm = 0.75, kd = 0 }, 0, -100)
local dNoArg = speedFor(5, false, { speedGain = 6, maxSpeed = 120, minSpeed = 1,
  degPerSecPerRpm = 0.75, kd = 0 }, 0, nil)
check("kd=0 ignores the error rate entirely", d0 == dNoArg,
  ("%.3f vs %.3f"):format(d0, dNoArg))

print(("base overshoot %.2f, damped %.2f"):format(base.overshoot, damped.overshoot))
print(fails == 0 and "ALL PASS" or (fails .. " FAILURES"))
if fails > 0 then os.exit(1) end
