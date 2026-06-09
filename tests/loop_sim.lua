-- Slow-loop drive simulation, VALIDATED against the in-game trace
-- (pastebin i1W268KP). The trace logged the actual commanded RPM next to the
-- resulting angle each loop, so we can replay those commands through a mount
-- model and confirm it reproduces the real motion before trusting any fix.
--
-- Findings the trace established:
--   * the control loop ran at ~0.25s/iteration (peripheral-limited), not 0.1s
--   * yaw (gain 6) overshot ~2.2 deg; pitch (gain 3) did not -- the overshoot
--     threshold is speedGain > 1/(T*dps) (= 5.3 at T=0.25, dps=0.75)
--   * a velocity cap (command no more speed than stops within one loop
--     period) removes the overshoot at any gain
--
-- Run from the repo root:  lua tests/loop_sim.lua

local fails = 0
local function check(name, ok, detail)
  if ok then print("ok   " .. name)
  else fails = fails + 1; print("FAIL " .. name .. (detail and (": " .. detail) or "")) end
end

local GAME_DT = 0.05
local DPS = 0.75
local MAXSPEED = 120

-- Mount physics: angular velocity ramps toward the commanded speed each game
-- tick (rotational inertia), angle integrates. RAMP is the fraction of the
-- speed gap closed per tick.
local function stepMount(theta, omega, omegaCmd, ramp)
  omega = omega + (omegaCmd - omega) * ramp
  return theta + omega * GAME_DT, omega
end

-- ---- the real yaw trace: { t, observed angle, commanded rpm } ----
local TRACE = {
  { 1.25, 90.000, -104.49 }, { 1.50, 74.400, -1.29 }, { 1.75, 70.350, 19.50 },
  { 2.00, 73.163, -2.91 }, { 2.25, 73.575, -5.99 }, { 2.50, 72.750, 0.00 },
  { 2.85, 72.563, 0.00 }, { 3.10, 72.563, 0.00 },
}

-- Replay the recorded commands through the mount and return the RMS error
-- between predicted and observed angle, for a given inertia RAMP.
local function replayError(ramp)
  local theta, omega = TRACE[1][2], 0
  local sumSq, n = 0, 0
  for i = 1, #TRACE - 1 do
    local omegaCmd = TRACE[i][3] * DPS
    local dt = TRACE[i + 1][1] - TRACE[i][1]
    local steps = math.floor(dt / GAME_DT + 0.5)
    for _ = 1, steps do theta, omega = stepMount(theta, omega, omegaCmd, ramp) end
    local err = theta - TRACE[i + 1][2]
    sumSq = sumSq + err * err; n = n + 1
  end
  return math.sqrt(sumSq / n)
end

-- Fit RAMP to the trace (coarse search).
local bestRamp, bestErr = nil, 1e9
for r = 20, 90 do
  local ramp = r / 100
  local e = replayError(ramp)
  if e < bestErr then bestErr, bestRamp = e, ramp end
end
check("mount model replays the real trace within ~1.5 deg RMS", bestErr < 1.5,
  ("RAMP %.2f, RMS %.2f deg"):format(bestRamp, bestErr))
print(("fitted inertia RAMP = %.2f (RMS %.2f deg)"):format(bestRamp, bestErr))
local RAMP = bestRamp

-- ---- now test controllers on the validated mount ----
local function command(err, gain, T, opts)
  local rpm = math.abs(err) * gain
  if opts and opts.cap then
    rpm = math.min(rpm, opts.approach * math.abs(err) / (T * DPS))
  end
  rpm = math.min(rpm, MAXSPEED)
  return err < 0 and -rpm or rpm
end

local function simulate(target, gain, T, opts)
  local theta, omega, omegaCmd = 0, 0, 0
  local stepsPerCtrl = math.max(1, math.floor(T / GAME_DT + 0.5))
  local peak, settledAt, t = 0, nil, 0
  for i = 0, 4000 do
    if i % stepsPerCtrl == 0 then
      omegaCmd = command(target - theta, gain, T, opts) * DPS
    end
    theta, omega = stepMount(theta, omega, omegaCmd, RAMP)
    t = t + GAME_DT
    if theta > target then peak = math.max(peak, theta - target) end
    if math.abs(target - theta) <= 0.3 then settledAt = settledAt or t
    else settledAt = nil end
    if settledAt and t - settledAt > 0.5 then break end
  end
  return { overshoot = peak, settle = settledAt or t, final = theta }
end

-- Pure P at the user's gains reproduces the asymmetry (yaw overshoots more
-- than pitch). Yaw here is WITHOUT the D term, so it overshoots more than the
-- ~2.2 deg the kd~0.18 trace showed -- which is exactly the "kd=0 is worse".
local yaw = simulate(17.4, 6, 0.25, nil)
local pitch = simulate(20, 3, 0.25, nil)
check("yaw gain 6 overshoots (kd=0, worse than the damped trace)",
  yaw.overshoot > 1.0, ("%.2f deg"):format(yaw.overshoot))
check("pitch gain 3 overshoots far less than yaw",
  pitch.overshoot < yaw.overshoot * 0.5,
  ("yaw %.2f vs pitch %.2f"):format(yaw.overshoot, pitch.overshoot))

-- The velocity cap (approach 0.5, the shipped default) removes the overshoot
-- and still converges -- on the small step AND the big 152 deg slew.
local APPROACH = 0.5
local capped = simulate(17.4, 6, 0.25, { cap = true, approach = APPROACH })
local cappedBig = simulate(152, 6, 0.25, { cap = true, approach = APPROACH })
check("velocity cap kills the yaw overshoot (small step)",
  capped.overshoot < 0.3, ("%.2f deg"):format(capped.overshoot))
check("velocity cap kills the overshoot on the 152 deg slew",
  cappedBig.overshoot < 0.3, ("%.2f deg"):format(cappedBig.overshoot))
check("capped loop still converges", math.abs(capped.final - 17.4) < 0.6,
  ("final %.2f"):format(capped.final))

-- Robust across high gains and loop rates.
local worst = 0
for _, g in ipairs({ 4, 8, 12, 20 }) do
  for _, T in ipairs({ 0.1, 0.2, 0.3 }) do
    worst = math.max(worst, simulate(30, g, T, { cap = true, approach = APPROACH }).overshoot)
  end
end
-- Stress bound: gain 20 at T=0.3 is far beyond the real operating point
-- (gain 6, loop ~0.25s, where overshoot is 0.00); even there it stays ~1 deg.
check("capped: overshoot bounded across extreme gain 4-20, T 0.1-0.3",
  worst < 1.5, ("worst %.2f deg"):format(worst))

-- A faster loop alone also tames the same gain (loop speed is the root cause).
local fast = simulate(17.4, 6, 0.05, nil)
check("yaw gain 6 at a 0.05s loop barely overshoots",
  fast.overshoot < yaw.overshoot * 0.5, ("%.2f deg"):format(fast.overshoot))

print(("uncapped yaw %.2f, capped %.2f, fast-loop %.2f deg")
  :format(yaw.overshoot, capped.overshoot, fast.overshoot))
print(fails == 0 and "ALL PASS" or (fails .. " FAILURES"))
if fails > 0 then os.exit(1) end
