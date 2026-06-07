-- ballistics.lua: CBC projectile flight model + pitch solver for
-- CCBigCannon. Constants and spawn behavior verified from the CBC
-- source (create-v6-1.21.1, June 2026); see TODO.md "Ballistics" for
-- the dig notes. Datapacks CAN override munition_properties/*, so on
-- a tuned server re-verify before trusting long shots.
--
-- Flight model (per game tick): pos = pos + v, then v = q*v + g,
-- where q = 1 - drag (0.99 for every CBC round) and g is the per-tick
-- gravity. Both axes have closed forms:
--   x(n)  = vh*(1 - q^n)/(1 - q)      ->  n(d) = ln(1 - d(1-q)/vh)/ln(q)
--   vy(k) = q^k*(vy0 - vt) + vt           vt = g/(1 - q)   (terminal)
--   y(n)  = (vy0 - vt)*(1 - q^n)/(1 - q) + vt*n
--         = (vy0 - vt)*d/vh + vt*n        (substituting the n(d) form)
-- so "height when the shell crosses horizontal distance d" is O(1) --
-- the Malex21 calculator's ln(0.99) trick, extended to the vertical
-- axis. Only the pitch root-find iterates: a sweep across the pitch
-- limits brackets each sign change of (height - target height),
-- bisection polishes, and up to two solutions come back (shallow and
-- steep arc). No solution = ballistically unreachable.
--
-- API units: blocks and SECONDS, velocities in BLOCKS/SECOND.
-- Internally blocks/tick (20 t/s), CBC-native.

local B = {}

local TPS = 20

-- Per-projectile constants (CBC datapack defaults). Big-cannon shells
-- fall at -0.05 b/t^2; autocannon rounds, the machine gun, and the
-- mortar stone at -0.025. Drag is 0.01/tick linear for all of them.
B.PROJECTILES = {
  shot               = { gravity = -0.05,  drag = 0.01 },
  ap_shot            = { gravity = -0.05,  drag = 0.01 },
  ap_shell           = { gravity = -0.05,  drag = 0.01 },
  he_shell           = { gravity = -0.05,  drag = 0.01 },
  fluid_shell        = { gravity = -0.05,  drag = 0.01 },
  shrapnel_shell     = { gravity = -0.05,  drag = 0.01 },
  smoke_shell        = { gravity = -0.05,  drag = 0.01 },
  drop_mortar_shell  = { gravity = -0.05,  drag = 0.01 },
  mortar_stone       = { gravity = -0.025, drag = 0.01 },
  ap_autocannon      = { gravity = -0.025, drag = 0.01 },
  flak_autocannon    = { gravity = -0.025, drag = 0.01 },
  machine_gun_bullet = { gravity = -0.025, drag = 0.01 },
}

-- Muzzle speed in blocks/second for a cannon profile. Big cannons get
-- 2 b/t per powder charge (propellant strength 2; big cartridges
-- match); autocannon speed is set by the cannon material + barrel
-- count, so the profile carries it directly (material formula in the
-- README). Errors loudly on a malformed profile rather than guessing.
function B.muzzleSpeed(profile)
  if profile.kind == "bigcannon" then
    if type(profile.charges) ~= "number" or profile.charges < 1 then
      error('profile.charges must be >= 1 for kind "bigcannon"', 0)
    end
    return profile.charges * 2 * TPS
  elseif profile.kind == "autocannon" then
    if type(profile.muzzleVelocity) ~= "number"
      or profile.muzzleVelocity <= 0 then
      error('profile.muzzleVelocity must be > 0 for kind "autocannon"', 0)
    end
    return profile.muzzleVelocity
  end
  error(('unknown profile.kind %q -- "autocannon" or "bigcannon"')
    :format(tostring(profile.kind)), 0)
end

-- Solve launch pitch. opts:
--   v0       muzzle speed, blocks/SECOND
--   gravity  per-tick gravity (b/t^2), e.g. -0.05
--   drag     per-tick linear drag fraction, e.g. 0.01
--   dx       horizontal distance mount -> target, blocks (> 0)
--   dy       target height above the mount, blocks
--   muzzle   launch-point offset along the barrel from the mount
--            pivot, blocks (CBC spawns ~barrelBlocks - 1.5 out)
--   minPitch/maxPitch  solver bounds in degrees (default -30..60,
--            the CBC mount envelope)
-- Returns a list of { pitch = degrees, tof = seconds }, sorted
-- shallow-first; empty when the target is unreachable.
function B.solve(opts)
  local v0 = opts.v0 / TPS
  local drag = opts.drag
  if type(drag) ~= "number" or drag <= 0 or drag >= 1 then
    error("drag must be in (0, 1) -- got " .. tostring(drag), 0)
  end
  local q = 1 - drag
  local vt = opts.gravity / drag -- terminal fall speed, b/t (negative)
  local muzzle = opts.muzzle or 0
  local lo, hi = opts.minPitch or -30, opts.maxPitch or 60
  local logq = math.log(q)

  -- Aim error at one pitch: shell height minus target height when the
  -- shell crosses the target's horizontal distance, plus flight ticks.
  -- nil = unreachable at this pitch (past the asymptotic horizontal
  -- range vh/drag, or the muzzle pokes past the target).
  local function err(pitchDeg)
    local r = math.rad(pitchDeg)
    local vh = v0 * math.cos(r)
    local d = opts.dx - muzzle * math.cos(r)
    if d <= 0 or vh <= 0 then return nil end
    local arg = 1 - d * drag / vh
    if arg <= 0 then return nil end
    local n = math.log(arg) / logq
    local vy0 = v0 * math.sin(r)
    local y = (vy0 - vt) * d / vh + vt * n
    return muzzle * math.sin(r) + y - opts.dy, n
  end

  -- Sweep for sign changes (0.5 deg grid), bisect each bracket. Two
  -- roots closer than the grid (grazing shots right at max range) can
  -- merge or vanish -- acceptable: that regime is brittle anyway.
  local sols = {}
  local STEPS = 180
  local prevP, prevE
  for i = 0, STEPS do
    local p = lo + (hi - lo) * i / STEPS
    local e = err(p)
    if e and prevE and (e > 0) ~= (prevE > 0) then
      local a, b, ea = prevP, p, prevE
      for _ = 1, 20 do
        local m = (a + b) / 2
        local em = err(m)
        if not em then break end -- can't happen inside a bracket; bail
        if (em > 0) == (ea > 0) then a, ea = m, em else b = m end
      end
      local pitch = (a + b) / 2
      local _, n = err(pitch)
      if n then sols[#sols + 1] = { pitch = pitch, tof = n / TPS } end
    end
    prevP, prevE = p, e
  end
  table.sort(sols, function(s1, s2) return s1.pitch < s2.pitch end)
  return sols
end

return B
