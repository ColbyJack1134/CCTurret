-- ballistics.lua: CBC projectile flight model + pitch solver for
-- CCBigCannon. Constants and spawn behavior verified from the CBC
-- source (create-v6-1.21.1, June 2026); see TODO.md "Ballistics" for
-- the dig notes. Datapacks CAN override munition_properties/*, so on
-- a tuned server re-verify before trusting long shots.
--
-- Flight model, taken VERBATIM from the CBC source (AbstractCannonProjectile
-- .getForces / tick): each tick a = -drag*v + g, then
--   pos = pos + v + 0.5*a   (NOT pos + v -- a half-acceleration term)
--   v   = v + a  = q*v + g,  q = 1 - drag (0.99 for every CBC round)
-- The position step is the trapezoidal rule: pos += 0.5*(v_k + v_{k+1}),
-- since a = v_{k+1} - v_k exactly. (Drag is linear: getDragForce =
-- formDrag*density*|v| applied along -v_hat, i.e. -formDrag*v; density = 1
-- overworld, quadratic_drag = false.) That half-step matters at artillery
-- range -- ignoring it (plain pos += v Euler) walks shots a few blocks low.
--
-- Closed forms. Velocity is unchanged from Euler, so vt and the q^n decay
-- are the same; only the position SUM picks up the 0.5*(1+q) trapezoid
-- weight, which collapses to a single constant K = 1/drag - 0.5:
--   x(n)  = vh*K*(1 - q^n)            ->  n(d) = ln(1 - d/(vh*K))/ln(q)
--   vy(k) = q^k*(vy0 - vt) + vt           vt = g/(1 - q) = g/drag (terminal)
--   y(n)  = (vy0 - vt)*K*(1 - q^n) + vt*n
--         = (vy0 - vt)*d/vh + vt*n        (since K*(1-q^n) = d/vh)
-- so "height when the shell crosses horizontal distance d" is still O(1) and
-- the vertical substitution is untouched -- only n(d) carries the K. Only the
-- pitch root-find iterates: a sweep across the pitch limits brackets each
-- sign change of (height - target height), bisection polishes, and up to two
-- solutions come back (shallow and steep arc). No solution = unreachable.
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

-- Autocannon muzzle speed is a CLOSED FORM of the build, not a measured
-- number: blocks/sec = 20 * (base + perBarrel * min(barrels, cap)), with
-- base/perBarrel/cap set by the cannon material. So the config carries
-- material + barrel count and we compute the speed (the same way big
-- cannons compute it from charges) -- nothing to calibrate. Constants
-- from the CBC source / verified in-game (full steel or cast-iron = 180
-- b/s); a datapack server CAN retune them, so profile.muzzleVelocityOverride
-- is the escape hatch (see B.muzzleSpeed).
--
-- `lifetime` is the round's flight TIME cap in ticks (CBC
-- AbstractAutocannonProjectile.ageRemaining, seeded per CANNON MATERIAL by
-- MountedAutocannonContraption -- machine gun bullets included; barrels
-- change speed, never lifetime). The round moves its full tick, THEN the
-- counter hits 0 and it silently despawns mid-air -- so it reaches a
-- target iff flight ticks <= lifetime. Big-cannon shells and mortar
-- stones have NO such cap (they fly until impact). Datapack-tunable, so
-- profile.lifetimeOverride mirrors the speed escape hatch.
B.AUTOCANNON_MATERIALS = {
  cast_iron = { base = 5, perBarrel = 2,   cap = 2, lifetime = 11 },
  bronze    = { base = 3, perBarrel = 1.5, cap = 3, lifetime = 25 },
  steel     = { base = 3, perBarrel = 1.5, cap = 4, lifetime = 60 },
}

-- Autocannon muzzle speed in blocks/second from material + barrel count.
-- Errors loudly on an unknown material rather than guessing a speed.
function B.autocannonSpeed(material, barrels)
  local m = B.AUTOCANNON_MATERIALS[material]
  if not m then
    local known = {}
    for k in pairs(B.AUTOCANNON_MATERIALS) do known[#known + 1] = k end
    table.sort(known)
    error(('unknown autocannon material %q -- known: %s')
      :format(tostring(material), table.concat(known, ", ")), 0)
  end
  if type(barrels) ~= "number" or barrels < 0 then
    error("profile.barrels must be a barrel count >= 0", 0)
  end
  return TPS * (m.base + m.perBarrel * math.min(barrels, m.cap))
end

-- Projectile lifetime in TICKS for a cannon profile, or nil when the
-- round has no in-flight cap (big cannons). Autocannon rounds despawn
-- when their material lifetime runs out; profile.lifetimeOverride (> 0)
-- forces a value for datapack-tuned servers. Errors loudly on a malformed
-- profile rather than guessing.
function B.lifetimeTicks(profile)
  if profile.kind == "bigcannon" then return nil end
  if profile.kind == "autocannon" then
    local override = profile.lifetimeOverride
    if type(override) == "number" and override > 0 then
      return override
    end
    local m = B.AUTOCANNON_MATERIALS[profile.material]
    if not m then
      local known = {}
      for k in pairs(B.AUTOCANNON_MATERIALS) do known[#known + 1] = k end
      table.sort(known)
      error(('unknown autocannon material %q -- known: %s')
        :format(tostring(profile.material), table.concat(known, ", ")), 0)
    end
    return m.lifetime
  end
  error(('unknown profile.kind %q -- "autocannon" or "bigcannon"')
    :format(tostring(profile.kind)), 0)
end

-- Muzzle speed in blocks/second for a cannon profile. Big cannons get
-- 2 b/t per powder charge (propellant strength 2; big cartridges match);
-- autocannon speed is computed from material + barrels by the formula
-- above, unless muzzleVelocityOverride (> 0) forces a value -- for a
-- datapack-tuned server whose numbers differ from the published ones.
-- Errors loudly on a malformed profile rather than guessing.
function B.muzzleSpeed(profile)
  if profile.kind == "bigcannon" then
    if type(profile.charges) ~= "number" or profile.charges < 1 then
      error('profile.charges must be >= 1 for kind "bigcannon"', 0)
    end
    return profile.charges * 2 * TPS
  elseif profile.kind == "autocannon" then
    local override = profile.muzzleVelocityOverride
    if type(override) == "number" and override > 0 then
      return override
    end
    return B.autocannonSpeed(profile.material, profile.barrels)
  end
  error(('unknown profile.kind %q -- "autocannon" or "bigcannon"')
    :format(tostring(profile.kind)), 0)
end

-- Ship-frame basis from a compass heading (CCMinimap convention:
-- degrees, 0 = north = -Z, 90 = east = +X): the horizontal forward
-- unit vector and its horizontal perpendicular ("right").
function B.shipFrame(headingDeg)
  local r = math.rad(headingDeg)
  return math.sin(r), -math.cos(r), -- forward x, z
    math.cos(r), math.sin(r)        -- right   x, z
end

-- Hull shapes for ship targets: a world-frame offset from the hull
-- centre (the transponder) is INSIDE the hull when hullNorm <= 1.
-- shape = { r = sphere radius, l/w/t = ellipsoid SEMI-axes (along
-- heading / across / vertical), avoid = keep-off radius around the
-- transponder block }. With a heading the oriented ellipsoid is used;
-- headingless beacons get the sphere.
function B.hullNorm(dx, dy, dz, shape, heading)
  if heading and shape.l then
    local fx, fz, rx, rz = B.shipFrame(heading)
    local a = (dx * fx + dz * fz) / shape.l
    local c = (dx * rx + dz * rz) / shape.w
    local u = dy / shape.t
    return math.sqrt(a * a + c * c + u * u)
  end
  return math.sqrt(dx * dx + dy * dy + dz * dz) / shape.r
end

-- Random aim point inside the hull shape: a world-frame offset from the
-- hull centre, uniform over the volume, never within shape.avoid of the
-- transponder. Rejection-samples the unit ball then stretches it onto
-- the shape (uniformity survives an affine map). rand is injectable for
-- deterministic tests; production passes nothing (math.random).
function B.sampleHullAim(shape, heading, rand)
  rand = rand or math.random
  local oriented = heading ~= nil and shape.l ~= nil
  local sl, sw, st
  if oriented then sl, sw, st = shape.l, shape.w, shape.t
  else sl, sw, st = shape.r, shape.r, shape.r end
  for _ = 1, 64 do
    local x = rand() * 2 - 1
    local y = rand() * 2 - 1
    local z = rand() * 2 - 1
    if x * x + y * y + z * z <= 1 then
      local dx, dy, dz
      if oriented then
        local fx, fz, rx, rz = B.shipFrame(heading)
        local a, c = x * sl, z * sw
        dx, dy, dz = fx * a + rx * c, y * st, fz * a + rz * c
      else
        dx, dy, dz = x * sl, y * st, z * sw
      end
      if dx * dx + dy * dy + dz * dz >= shape.avoid * shape.avoid then
        return dx, dy, dz
      end
    end
  end
  -- Pathological config (avoid swallows the whole shape): aim just
  -- below the protected bubble rather than looping forever.
  return 0, -(shape.avoid + 1), 0
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
  -- Trapezoidal position factor (pos += 0.5*(v_k + v_{k+1})); plain Euler
  -- would be 1/drag. The asymptotic horizontal range is vh*K.
  local posK = 1 / drag - 0.5
  local muzzle = opts.muzzle or 0
  local lo, hi = opts.minPitch or -30, opts.maxPitch or 60
  local logq = math.log(q)

  -- Aim error at one pitch: shell height minus target height when the
  -- shell crosses the target's horizontal distance, plus flight ticks.
  -- nil = unreachable at this pitch (past the asymptotic horizontal
  -- range vh*K, or the muzzle pokes past the target).
  local function err(pitchDeg)
    local r = math.rad(pitchDeg)
    local vh = v0 * math.cos(r)
    local d = opts.dx - muzzle * math.cos(r)
    if d <= 0 or vh <= 0 then return nil end
    local arg = 1 - d / (vh * posK)
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

-- Forward shot: fire a shell at a GIVEN pitch and report where it goes.
-- Fed the barrel's ACTUAL current angle (not the solver's answer) this is
-- a diagnostic -- it predicts the real shot so you can compare it to the
-- target and to where the shell is seen to land. opts:
--   v0/gravity/drag/muzzle  as in solve()
--   pitch    launch pitch in degrees
--   dx       target's horizontal distance, blocks (optional)
--   dy       target's height above the mount, blocks
-- Returns a table with whichever it can compute:
--   hAtTarget  shell height when it crosses dx -- so hAtTarget - dy is
--              the vertical miss AT the target's range, the precise aim
--              error in blocks (+ high, - low/short). nil past max range.
--   range/tof  horizontal blocks / seconds to where the shell DESCENDS
--              back through dy -- the ground impact for an arcing shot,
--              i.e. the spot to go watch. nil if it never comes down to
--              dy. Per-tick integration (CBC-native), capped so a
--              pathological angle can't spin.
function B.impact(opts)
  local q = 1 - opts.drag
  local r = math.rad(opts.pitch)
  local v0 = opts.v0 / TPS
  local muzzle = opts.muzzle or 0
  local x = muzzle * math.cos(r)
  local y = muzzle * math.sin(r)
  local vx = v0 * math.cos(r)
  local vy = v0 * math.sin(r)
  local dy, dx = opts.dy, opts.dx
  local hAtTarget, range, tof
  for t = 1, 6000 do
    local px, py = x, y
    -- CBC tick: a = -drag*v + g; pos += v + 0.5*a; v += a. Equivalently
    -- pos += 0.5*(v_old + v_new) -- the trapezoidal rule used in solve().
    local nvx = vx * q
    local nvy = vy * q + opts.gravity
    x = x + 0.5 * (vx + nvx)
    y = y + 0.5 * (vy + nvy)
    vx, vy = nvx, nvy
    if dx and not hAtTarget and px <= dx and x >= dx then
      local f = (x == px) and 0 or (dx - px) / (x - px)
      hAtTarget = py + (y - py) * f
    end
    -- Descending crossing of the target plane (vy < 0 skips the way up).
    if not range and vy < 0 and py >= dy and y <= dy then
      local f = (py == y) and 0 or (py - dy) / (py - y)
      range, tof = px + (x - px) * f, (t - 1 + f) / TPS
    end
    if range and (not dx or hAtTarget) then break end
  end
  return { hAtTarget = hAtTarget, range = range, tof = tof }
end

return B
