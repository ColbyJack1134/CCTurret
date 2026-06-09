-- Offline validation of autotune.lua: run the tuner against a range of
-- simulated mounts (different gearing, inertia, loop rate, sensor noise) and
-- confirm the approach it picks actually delivers a no-overshoot, converging
-- step on each one. The mount model is the one validated against the real
-- in-game traces (RAMP ~0.49 reproduced the measured overshoot).
--
-- Run from the repo root:  lua tests/autotune_sim.lua

local AT = dofile("autotune.lua")

local fails = 0
local function check(name, ok, detail)
  if ok then print("ok   " .. name)
  else fails = fails + 1; print("FAIL " .. name .. (detail and (": " .. detail) or "")) end
end

-- speedFor copied VERBATIM from cannon.lua (the law the tuner optimizes and
-- that runs in game). invert is irrelevant in the sim (passed false).
local function speedFor(diff, invert, drive, ffRate, dRate, loopT)
  local ff, d = 0, 0
  local degPerSec = drive.degPerSecPerRpm
  local haveRate = type(degPerSec) == "number" and degPerSec > 0
  if haveRate then
    if ffRate then ff = ffRate / degPerSec end
    if dRate and drive.kd and drive.kd ~= 0 then d = drive.kd * dRate / degPerSec end
  end
  local p = math.abs(diff) * drive.speedGain
  if haveRate and drive.approach and drive.approach > 0 and loopT and loopT > 0 then
    p = math.min(p, drive.approach * math.abs(diff) / (loopT * degPerSec))
  end
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
local function control(err, drive, loopT) return speedFor(err, false, drive, nil, nil, loopT) end

-- A simulated mount driven through the io interface. omega ramps toward the
-- commanded speed each game tick (inertia); angle integrates and clamps to the
-- travel limits; readAngle carries optional sensor noise.
local function makeMount(p)
  local m = { theta = (p.lo + p.hi) / 2, omega = 0, cmd = 0, clock = 0, seed = 12345 }
  local function noise()
    if (p.noise or 0) == 0 then return 0 end
    m.seed = (m.seed * 1103515245 + 12345) % 2147483648
    return ((m.seed / 2147483648) - 0.5) * 2 * p.noise
  end
  local io = {
    dps = p.dps, minSpeed = p.minSpeed, maxSpeed = p.maxSpeed or 120,
    loopT = p.loopT, lo = p.lo, hi = p.hi,
    log = p.verbose and print or function() end,
    readAngle = function() return m.theta + noise() end,
    setRpm = function(rpm) m.cmd = rpm end,
    now = function() return m.clock end,
    wait = function(s)
      local n = math.max(1, math.floor(s / 0.05 + 0.5))
      for _ = 1, n do
        m.omega = m.omega + (m.cmd * p.dps - m.omega) * p.ramp
        m.theta = m.theta + m.omega * 0.05
        if m.theta < p.lo then m.theta, m.omega = p.lo, 0 end
        if m.theta > p.hi then m.theta, m.omega = p.hi, 0 end
      end
      m.clock = m.clock + n * 0.05
    end,
  }
  return io, m
end

-- Verify a step with a given drive: peak overshoot beyond target (degrees).
local function verifyStep(io, m, drive, fromA, toA)
  m.theta, m.omega, m.cmd = fromA, 0, 0
  local dir = toA >= fromA and 1 or -1
  local peak, t0, settledAt = fromA, m.clock, nil
  while m.clock - t0 < 6 do
    local a = m.theta
    io.setRpm(control(toA - a, drive, io.loopT))
    peak = dir > 0 and math.max(peak, a) or math.min(peak, a)
    if math.abs(toA - a) <= 0.5 then settledAt = settledAt or (m.clock - t0)
    else settledAt = nil end
    if settledAt and m.clock - t0 - settledAt > 0.5 then break end
    io.wait(io.loopT)
  end
  return (peak - toA) * dir, settledAt
end

-- Mounts: { name, dps, ramp, loopT, minSpeed, noise, lo, hi }
local MOUNTS = {
  { name = "direct-drive, slow loop", dps = 0.75, ramp = 0.49, loopT = 0.25, minSpeed = 1, noise = 0.03 },
  { name = "direct-drive, fast loop", dps = 0.75, ramp = 0.49, loopT = 0.10, minSpeed = 1, noise = 0.03 },
  { name = "geared down (slow mount)", dps = 0.30, ramp = 0.49, loopT = 0.20, minSpeed = 1, noise = 0.02 },
  { name = "geared up (fast mount)", dps = 1.50, ramp = 0.49, loopT = 0.20, minSpeed = 1, noise = 0.05 },
  { name = "high inertia", dps = 0.75, ramp = 0.25, loopT = 0.20, minSpeed = 1, noise = 0.03 },
  { name = "snappy / low inertia", dps = 0.75, ramp = 0.80, loopT = 0.20, minSpeed = 1, noise = 0.03 },
  { name = "noisy sensor", dps = 0.75, ramp = 0.49, loopT = 0.20, minSpeed = 1.5, noise = 0.15 },
}

for _, spec in ipairs(MOUNTS) do
  spec.lo, spec.hi = -60, 60
  local io, m = makeMount(spec)
  local res = AT.tuneAxis(io, control)
  -- Verify the chosen gains on a fresh step (both directions), within limits.
  local drive = { speedGain = res.speedGain, approach = res.approach, kd = 0,
    minSpeed = spec.minSpeed, maxSpeed = 120, degPerSecPerRpm = spec.dps }
  local ovUp = verifyStep(io, m, drive, -25, 25)
  local ovDn = verifyStep(io, m, drive, 25, -25)
  local ov = math.max(ovUp, ovDn)
  local ok = ov < 0.8 and res.approach >= 0.3
  check(("%-26s approach %.2f gain %.1f -> overshoot %.2f deg")
    :format(spec.name, res.approach, res.speedGain, ov), ok)
end

print(fails == 0 and "ALL PASS" or (fails .. " FAILURES"))
if fails > 0 then os.exit(1) end
