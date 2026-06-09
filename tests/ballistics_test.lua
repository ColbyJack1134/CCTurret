-- Headless tests for ballistics.lua. Run from the repo root with
-- desktop Lua:  lua tests/ballistics_test.lua
-- Every solver answer is cross-checked by an INDEPENDENT per-tick
-- simulation of the CBC flight model (pos += v; v = (1-drag)*v + g).

local B = dofile("ballistics.lua")

local fails = 0
local function check(name, ok, detail)
  if ok then
    print("ok   " .. name)
  else
    fails = fails + 1
    print("FAIL " .. name .. (detail and (": " .. detail) or ""))
  end
end

-- Fly one shot tick-by-tick from the muzzle offset; return the height
-- (blocks, mount-relative) where it crosses horizontal distance dx and
-- the flight time in seconds (linear interpolation inside the tick).
local function simulate(v0bs, gravity, drag, muzzle, pitchDeg, dx)
  local r = math.rad(pitchDeg)
  local q = 1 - drag
  local x = muzzle * math.cos(r)
  local y = muzzle * math.sin(r)
  local vx = v0bs / 20 * math.cos(r)
  local vy = v0bs / 20 * math.sin(r)
  for t = 1, 200000 do
    local px, py = x, y
    -- CBC integration (AbstractCannonProjectile): a = -drag*v + g,
    -- pos += v + 0.5*a, v += a  ==  pos += 0.5*(v_old + v_new).
    local nvx = vx * q
    local nvy = vy * q + gravity
    x = x + 0.5 * (vx + nvx)
    y = y + 0.5 * (vy + nvy)
    vx, vy = nvx, nvy
    if x >= dx then
      local f = (dx - px) / (x - px)
      return py + (y - py) * f, (t - 1 + f) / 20
    end
  end
  return nil
end

local HE = B.PROJECTILES.he_shell
local SHOT = B.PROJECTILES.shot
local AC = B.PROJECTILES.ap_autocannon

-- Solve a spread of scenarios, then fly every returned solution and
-- demand it lands on the target height at the target distance.
local cases = {
  -- At 400 blocks the steep arc sits ~82 deg, OUTSIDE the -30..60
  -- mount envelope: only the shallow solution comes back.
  { name = "5ch 400 flat", v0 = 200, p = HE, dx = 400, dy = 0,
    muzzle = 12.5, arcs = 1 },
  { name = "5ch 650 dy-10", v0 = 200, p = HE, dx = 650, dy = -10,
    muzzle = 12.5 },
  { name = "5ch 300 dy+40", v0 = 200, p = HE, dx = 300, dy = 40,
    muzzle = 12.5 },
  { name = "5ch 600 dy-150 (terminal)", v0 = 200, p = HE, dx = 600,
    dy = -150, muzzle = 12.5 },
  { name = "2ch ship gun 150 dy+5", v0 = 80, p = SHOT, dx = 150, dy = 5,
    muzzle = 5.5 },
  { name = "autocannon 80 flat", v0 = 180, p = AC, dx = 80, dy = 0,
    muzzle = 0.5 },
  { name = "autocannon 99 dy+20", v0 = 180, p = AC, dx = 99, dy = 20,
    muzzle = 0.5 },
}
for _, c in ipairs(cases) do
  local sols = B.solve{ v0 = c.v0, gravity = c.p.gravity, drag = c.p.drag,
    dx = c.dx, dy = c.dy, muzzle = c.muzzle }
  check(c.name .. " solvable", #sols >= 1, "no solutions")
  if c.arcs then
    check(c.name .. " arc count", #sols == c.arcs, "#sols = " .. #sols)
  end
  for i, s in ipairs(sols) do
    local y, tof = simulate(c.v0, c.p.gravity, c.p.drag, c.muzzle,
      s.pitch, c.dx)
    local dyErr = y and math.abs(y - c.dy)
    check(("%s arc %d hits (pitch %+.2f)"):format(c.name, i, s.pitch),
      dyErr ~= nil and dyErr < 0.15,
      dyErr and ("miss %.3f blocks"):format(dyErr) or "no impact")
    check(("%s arc %d tof"):format(c.name, i),
      tof ~= nil and math.abs(tof - s.tof) < 0.06,
      tof and ("sim %.2f vs solver %.2f"):format(tof, s.tof) or "")
  end
end

-- Shallow arc sorts first and flies faster than the steep arc.
local both = B.solve{ v0 = 200, gravity = HE.gravity, drag = HE.drag,
  dx = 650, dy = -10, muzzle = 12.5 }
check("two arcs at 650", #both == 2, "#sols = " .. #both)
if #both == 2 then
  check("shallow first", both[1].pitch < both[2].pitch)
  check("shallow flies faster", both[1].tof < both[2].tof)
end

-- Widening the pitch envelope past the mount limit recovers the steep
-- arc that the 400-block case hides above 60 deg -- and it still hits.
local wide = B.solve{ v0 = 200, gravity = HE.gravity, drag = HE.drag,
  dx = 400, dy = 0, muzzle = 12.5, maxPitch = 89 }
check("steep arc exists past 60 deg", #wide == 2
  and wide[2].pitch > 60, "#sols = " .. #wide)
if #wide == 2 then
  local y = simulate(200, HE.gravity, HE.drag, 12.5, wide[2].pitch, 400)
  check("steep arc past 60 hits", y ~= nil and math.abs(y) < 0.15,
    y and ("miss %.3f"):format(math.abs(y)) or "no impact")
end

-- Max range boundary (5 charges, no muzzle offset): ~686 on flat ground
-- with the trapezoidal CBC integration (Euler over-predicted ~693), so
-- 685 solves and 695 does not.
local near = B.solve{ v0 = 200, gravity = HE.gravity, drag = HE.drag,
  dx = 685, dy = 0, muzzle = 0 }
local past = B.solve{ v0 = 200, gravity = HE.gravity, drag = HE.drag,
  dx = 695, dy = 0, muzzle = 0 }
check("685 within 5ch max range", #near >= 1)
check("695 past 5ch max range", #past == 0)

-- Autocannon drop compensation: at 80 blocks the solver aims a little
-- ABOVE line of sight (drop ~1 block -> ~0.7 deg), never below.
local ac = B.solve{ v0 = 180, gravity = AC.gravity, drag = AC.drag,
  dx = 80, dy = 0, muzzle = 0.5 }
check("autocannon aims above LOS",
  #ac >= 1 and ac[1].pitch > 0.2 and ac[1].pitch < 2,
  #ac >= 1 and ("pitch %.2f"):format(ac[1].pitch) or "no sols")

-- Muzzle offset changes the solution measurably on a long gun.
local withM = B.solve{ v0 = 200, gravity = HE.gravity, drag = HE.drag,
  dx = 300, dy = 0, muzzle = 12.5 }
local noM = B.solve{ v0 = 200, gravity = HE.gravity, drag = HE.drag,
  dx = 300, dy = 0, muzzle = 0 }
check("muzzle offset shifts shallow pitch",
  #withM >= 1 and #noM >= 1
  and math.abs(withM[1].pitch - noM[1].pitch) > 0.01,
  (#withM >= 1 and #noM >= 1)
  and ("%.3f vs %.3f"):format(withM[1].pitch, noM[1].pitch) or "missing sols")

-- Target inside the barrel: unsolvable, not a crash.
local inside = B.solve{ v0 = 200, gravity = HE.gravity, drag = HE.drag,
  dx = 5, dy = 0, muzzle = 12.5 }
check("target inside barrel -> no solutions", #inside == 0)

-- B.impact (forward shot) inverts B.solve: at the solver's own pitch the
-- shell's height when it reaches the target's range must equal the target
-- height (hAtTarget - dy ~ 0), for the shallow AND steep arc -- including
-- the dy+40 case the solver hits on the way UP.
for _, c in ipairs({
  { v0 = 200, p = HE, dx = 400, dy = 0,   muzzle = 12.5 },
  { v0 = 200, p = HE, dx = 650, dy = -10, muzzle = 12.5 },
  { v0 = 200, p = HE, dx = 300, dy = 40,  muzzle = 12.5 },
  { v0 = 180, p = AC, dx = 80,  dy = 0,   muzzle = 0.5 },
}) do
  local sols = B.solve{ v0 = c.v0, gravity = c.p.gravity, drag = c.p.drag,
    dx = c.dx, dy = c.dy, muzzle = c.muzzle }
  for i, s in ipairs(sols) do
    local imp = B.impact{ v0 = c.v0, gravity = c.p.gravity, drag = c.p.drag,
      muzzle = c.muzzle, pitch = s.pitch, dx = c.dx, dy = c.dy }
    check(("impact dx=%d arc %d height at target"):format(c.dx, i),
      imp.hAtTarget and math.abs(imp.hAtTarget - c.dy) < 0.2,
      imp.hAtTarget and ("%.2f vs %d"):format(imp.hAtTarget, c.dy)
        or "no crossing")
  end
end

-- Aimed 2 deg under the solution, the shell passes LOW at the target's
-- range -- the diagnostic's whole point (catches a barrel that fired
-- before it finished slewing onto the solution).
do
  local sols = B.solve{ v0 = 200, gravity = HE.gravity, drag = HE.drag,
    dx = 400, dy = 0, muzzle = 12.5 }
  local imp = B.impact{ v0 = 200, gravity = HE.gravity, drag = HE.drag,
    muzzle = 12.5, pitch = sols[1].pitch - 2, dx = 400, dy = 0 }
  check("under-elevated shot is low at target range",
    imp.hAtTarget and imp.hAtTarget < 0,
    imp.hAtTarget and ("h %.1f"):format(imp.hAtTarget) or "nil")
end

-- Profile -> muzzle speed resolution, including loud failures.
check("bigcannon 5 charges = 200 b/s",
  B.muzzleSpeed{ kind = "bigcannon", charges = 5 } == 200)
-- Autocannon speed is computed from material + barrels (20*(base+per*min(b,cap))).
check("autocannon full steel (6 barrels, cap 4) = 180 b/s",
  B.muzzleSpeed{ kind = "autocannon", material = "steel", barrels = 6 } == 180)
check("autocannon full cast iron (cap 2) = 180 b/s",
  B.muzzleSpeed{ kind = "autocannon", material = "cast_iron", barrels = 6 } == 180)
check("autocannon bronze 3 barrels (cap 3) = 150 b/s",
  B.muzzleSpeed{ kind = "autocannon", material = "bronze", barrels = 3 } == 150)
check("autocannon steel 2 barrels (under cap) = 120 b/s",
  B.muzzleSpeed{ kind = "autocannon", material = "steel", barrels = 2 } == 120)
check("autocannonSpeed barrels capped",
  B.autocannonSpeed("steel", 99) == B.autocannonSpeed("steel", 4))
check("muzzleVelocityOverride forces the speed",
  B.muzzleSpeed{ kind = "autocannon", material = "steel", barrels = 6,
    muzzleVelocityOverride = 250 } == 250)
check("override 0 falls back to the formula",
  B.muzzleSpeed{ kind = "autocannon", material = "steel", barrels = 6,
    muzzleVelocityOverride = 0 } == 180)
check("bigcannon without charges errors",
  not pcall(B.muzzleSpeed, { kind = "bigcannon" }))
check("autocannon unknown material errors",
  not pcall(B.muzzleSpeed, { kind = "autocannon", material = "titanium", barrels = 4 }))
check("unknown kind errors",
  not pcall(B.muzzleSpeed, { kind = "railgun" }))

print(fails == 0 and "ALL PASS" or (fails .. " FAILURES"))
if fails > 0 then os.exit(1) end
