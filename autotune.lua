-- autotune.lua: measured, closed-loop auto-tuner for the CCBigCannon drive.
--
-- The drive's overshoot is governed by the `approach` cap (see speedFor): the
-- correction speed is limited to approach*|err|/(loopT*dps), and the largest
-- approach that doesn't overshoot gives both the tightest settle and the
-- fastest glide-on. So tuning is a one-parameter-per-axis search: raise
-- approach, run a real step response, and stop just before overshoot appears.
-- speedGain is set high enough that the (loop-adaptive) cap is the limiter.
--
-- It runs the REAL control law against an abstract axis `io`, so the same code
-- tunes a live mount (cannon.lua) or a simulated one (tests/autotune_sim.lua),
-- and every step is paced to the measured loop period so the result transfers
-- to normal tracking. No plant model -- it just measures what the mount does.
--
-- io = {
--   readAngle()  -> mount angle, degrees
--   setRpm(rpm)  -- command the speed controller (already sign-corrected)
--   wait(s), now() -> sleep / monotonic seconds
--   log(msg)     -- progress line (also captured to the tune log)
--   dps, minSpeed, maxSpeed, loopT   -- calibrated drive constants + live loop
--   lo, hi       -- safe travel band to step within (degrees)
-- }
-- control(err, drive, loopT) -> rpm   (the drive law: speedFor with kd off)

local M = {}

local OVERSHOOT_ONSET = 0.5    -- degrees of overshoot = "started to overshoot"
local SETTLE_BAND = 0.6       -- degrees, for the settle-time readout

-- Drive toward `target` until within band and stopped, or maxT. Paced to loopT.
local function driveTo(io, control, drive, target, maxT)
  local t0 = io.now()
  while io.now() - t0 < maxT do
    local a = io.readAngle()
    local it0 = io.now()
    io.setRpm(control(target - a, drive, io.loopT))
    if math.abs(target - a) <= SETTLE_BAND then break end
    io.wait(math.max(0.05, io.loopT - (io.now() - it0)))
  end
  io.setRpm(0)
  for _ = 1, 5 do io.setRpm(0); io.wait(io.loopT) end -- let motion bleed off
end

-- One step from the current angle to `target`: ABSOLUTE overshoot (degrees
-- past the target) and settle time. Absolute, not relative, so the onset
-- threshold is a real miss distance independent of step size.
local function stepResponse(io, control, drive, target)
  local fromA = io.readAngle()
  if math.abs(target - fromA) < 1 then return { overshoot = 0, settle = 0 } end
  local dir = target >= fromA and 1 or -1
  local peak, t0, settledAt, inBand = fromA, io.now(), nil, false
  while io.now() - t0 < 4.0 do
    local a = io.readAngle()
    local it0 = io.now()
    io.setRpm(control(target - a, drive, io.loopT))
    peak = dir > 0 and math.max(peak, a) or math.min(peak, a)
    local t = io.now() - t0
    if math.abs(target - a) <= SETTLE_BAND then
      if not inBand then inBand, settledAt = true, t end
    else inBand, settledAt = false, nil end
    if settledAt and t - settledAt > 0.4 then break end
    io.wait(math.max(0.05, io.loopT - (io.now() - it0)))
  end
  io.setRpm(0)
  return { overshoot = math.max(0, (peak - target) * dir), settle = settledAt or 6.0 }
end

-- Average an up-step and a down-step so an off-center start doesn't bias it.
-- After the up-step the axis is already at hiA, so the down-step needs no
-- reposition -- one reposition + two measured steps per probe.
local function probe(io, control, approach, loA, hiA)
  local drive = { speedGain = 1e6, approach = approach, kd = 0,
    minSpeed = io.minSpeed, maxSpeed = io.maxSpeed, degPerSecPerRpm = io.dps }
  driveTo(io, control, drive, loA, 4.0)
  local up = stepResponse(io, control, drive, hiA)
  local down = stepResponse(io, control, drive, loA)
  -- Worst direction, not the average: a build can overshoot more one way
  -- (asymmetric load), and one overshoot is one too many.
  return math.max(up.overshoot, down.overshoot), math.max(up.settle, down.settle)
end

-- Tune one axis. Returns { approach, speedGain, metrics }.
function M.tuneAxis(io, control)
  local function log(s) if io.log then io.log(s) end end
  local mid = (io.lo + io.hi) / 2
  -- Tune on a LARGE step (most of the travel) so the result is safe for the
  -- worst-case slew, not just a gentle correction. Capped so the test is brisk.
  local A = math.min(50, (io.hi - io.lo) * 0.7)
  local loA, hiA = mid - A / 2, mid + A / 2
  log(("== tune axis: loopT %.0fms, dps %.2f, step %.0f deg =="):format(
    io.loopT * 1000, io.dps, A))

  -- Raise approach until a step overshoots more than OVERSHOOT_ONSET degrees.
  local onset, lastGood, lastSettle = nil, 0.3, nil
  local a = 0.3
  while a <= 2.0 + 1e-9 do
    local ov, st = probe(io, control, a, loA, hiA)
    log(("approach %.2f  overshoot %.2f deg  settle %.2fs"):format(a, ov, st))
    if ov > OVERSHOOT_ONSET then onset = a; break end
    lastGood, lastSettle = a, st
    a = a + 0.1
  end

  -- The largest approach that stayed under the overshoot threshold is the
  -- last one that passed the scan (NOT onset*margin -- that can land above the
  -- last good value when the cliff is steep). Re-verify it and step down until
  -- a fresh step is clean, which absorbs sensor noise and sharp overshoot
  -- knees (high-inertia mounts).
  local approach = lastGood
  for _ = 1, 3 do
    local ov = probe(io, control, approach, loA, hiA)
    log(("verify approach %.2f  overshoot %.2f deg"):format(approach, ov))
    if ov <= OVERSHOOT_ONSET or approach <= 0.2 + 1e-9 then break end
    approach = math.max(0.2, approach - 0.1)
  end
  -- Keep the (loop-adaptive) cap the active limiter, not speedGain: set the P
  -- gain a comfortable margin above the effective gain the cap asks for here.
  local speedGain = math.max(2, math.min(40,
    1.5 * approach / (io.loopT * io.dps)))
  log(("-> approach %.2f, speedGain %.1f%s"):format(approach, speedGain,
    onset and "" or " (no overshoot up to 2.0; loop may be fast)"))
  return { approach = approach, speedGain = speedGain,
    metrics = { onset = onset, settle = lastSettle } }
end

return M
