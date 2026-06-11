-- cannon.lua: CCBigCannon v2 -- closed-loop turret control for Create Big Cannons.
--
-- Reads the cannon mount's CannonYaw/CannonPitch NBT through a Block Reader,
-- drives two modem-attached Rotational Speed Controllers, and fires via a
-- Redstone Relay pulse.
--
-- Targeting is click-to-lock, CCMinimap-style: a TARGETS tab lists every
-- online player (getOnlinePlayers/getPlayerPos are position-independent, so
-- this works from an airship where range scans see nobody) plus every ship
-- heard on the CCMinimap transponder (rednet "airship-state" broadcasts);
-- click a row to track it, click again or [ STOP ] to release.
--
-- Keys: F = fire, A = arm/disarm, K = calibrate+tune, O = capture yawOffset
-- from the rest pose, L = diagnostic trace (drive + fire-gate), Q = quit.
-- Mouse/touch for the rest.
-- While armed, the fire line is held high whenever both axes are locked on
-- (autocannon assumption) and dropped the moment lock is lost.
--
-- Config lives in cannon.cfg (JSON). Missing keys are filled from DEFAULTS
-- on first boot and written back, CCMinimap-style. Edit the peripheral names
-- there to match your network (e.g. yaw = "Create_RotationSpeedController_0").

local Cfg = dofile("cfgutil.lua")
local Heading = dofile("heading.lua")
local Ballistics = dofile("ballistics.lua")
local Autotune = dofile("autotune.lua")

local CONFIG = "cannon.cfg"   -- hand-authored intent
local CALFILE = "cannon.cal"  -- machine-measured, safe to delete (CAL rebuilds)

local DEFAULTS = {
  -- "auto" finds the single attached peripheral of that type (errors if
  -- there are zero or several). The two speed controllers share a type,
  -- so "auto" yaw/pitch are told apart by the calibration wiggle: each
  -- controller is nudged and whichever of CannonYaw/CannonPitch moves
  -- names it. The resolved names are saved to cannon.cal. Leave them
  -- "auto" for hands-off setup, or pin explicit names here to skip it.
  peripherals = {
    yaw = "auto",
    pitch = "auto",
    blockReader = "auto",
    playerDetector = "auto",
    relay = "auto",
  },
  -- How this turret introduces itself to the Spruce C2 server (Turrets
  -- tab). Empty = fall back to the computer label, then "turret-<id>".
  -- The Spruce link itself is configured in turret.cfg (url + token).
  callsign = "",
  -- Which side of the redstone relay the fire line is wired to.
  -- Relays only accept relative names: top/bottom/front/back/left/right.
  fireSide = "top",
  firePulseSeconds = 0.1,
  -- Physical reload cycle for a manually-loaded big cannon (no
  -- autoloader). Bigcannon only; ignored for autocannons. When enabled,
  -- assemblySide is held HIGH = assembled (the default state); after a
  -- shot the sequence runs: drop assemblySide (disassemble), wait
  -- settleSeconds, pulse reloadSide to kick off the loader, wait
  -- profile.reloadSeconds for it to finish, raise assemblySide
  -- (reassemble), wait settleSeconds, ready to fire again. The motors
  -- hold (stopped) for the whole cycle since the contraption is gone.
  -- enabled = false keeps the old autoloader behavior (a plain pulse +
  -- profile.reloadSeconds time gate, no assembly line). relay = "same"
  -- shares the fire relay (fire/assembly/reload on three of its sides);
  -- name a second redstone relay to drive the assembly/reload lines from
  -- there instead. Sides take the usual relative names.
  -- park = true slews the barrel back to its rest orientation (mount-frame
  -- 0,0) BEFORE disassembling, holding there until both axes settle or
  -- parkSeconds elapses -- for builds whose loader only clears at a fixed
  -- pose. Off by default (tear down wherever the barrel happens to point).
  reload = {
    enabled = false,
    relay = "same",
    assemblySide = "back",
    reloadSide = "left",
    reloadPulseSeconds = 0.4,
    settleSeconds = 1.0,
    park = false,
    parkSeconds = 4.0,
  },
  -- What this cannon is: drives both the fire mode and the ballistics.
  -- kind "autocannon" holds the fire line while the gate is open;
  -- "bigcannon" pulses firePulseSeconds per shot and waits
  -- reloadSeconds before the next. projectile keys the constants table
  -- in ballistics.lua (big-cannon shells fall at -0.05 b/t^2,
  -- autocannon rounds at -0.025).
  --
  -- Muzzle speed is COMPUTED from the build, never typed by hand:
  --  * bigcannon: 2 b/t per powder charge -> charges.
  --  * autocannon: 20 * (base + perBarrel * min(barrels, cap)) with
  --    base/perBarrel/cap from material ("cast_iron" / "bronze" /
  --    "steel"); e.g. full-length steel or cast iron = 180 b/s. So set
  --    material + barrels, not a velocity. muzzleVelocityOverride > 0
  --    forces a speed instead (only for a datapack-tuned server whose
  --    numbers differ from the published ones); 0 = compute.
  --
  -- barrelBlocks = mount pivot -> muzzle tip in blocks: CBC spawns the
  -- shell ~barrelBlocks-1.5 along the barrel, which matters for arcing
  -- big-cannon shots. arc picks the "shallow" (flat, fast) or "steep"
  -- (lobbed) solution when both exist.
  profile = {
    kind = "autocannon",
    projectile = "ap_autocannon",
    material = "steel",          -- autocannon: cast_iron / bronze / steel
    barrels = 6,                 -- autocannon: barrel count (cap applies)
    muzzleVelocityOverride = 0,  -- autocannon: >0 forces b/s, 0 = compute
    charges = 1,                 -- bigcannon only, powder charges loaded
    barrelBlocks = 2,
    reloadSeconds = 5,           -- bigcannon only, pause between auto shots
    arc = "shallow",
  },
  -- WHERE THE CANNON IS. Every field here points at the BASE of the cannon
  -- MOUNT BLOCK (its plain block coords) -- never the muzzle or the pivot.
  -- The launch pivot is derived automatically: CBC seats the gun's rotation
  -- point at the CENTRE of the block 2 along the mount's vertical axis, i.e.
  -- base + (0.5, +2.5, 0.5) for a normal cannon, or base + (0.5, -1.5, 0.5)
  -- when upsideDown. The +0.5s are block-centre; the 2 is the trunnion step
  -- (see pivotFromBase). So you only ever enter whole-block offsets.
  --
  --  * x/y/z      manual mount-block coords; used when gps = false and not
  --               aboard a ship. A wireless modem may still be present for
  --               transponder targets only -- gps is an explicit opt-in.
  --  * gps = true locate the COMPUTER once at boot (needs a wireless modem +
  --               GPS constellation; boot fails loudly without a fix) and
  --               derive the mount as fix + offset. x/y/z are ignored.
  --  * offset     blocks from the computer to the mount base. ONE value,
  --               shared by the static-gps path AND ship mode, edited in the
  --               CONFIG tab. Static: WORLD axes (x east, y up, z south).
  --               Ship: HULL-local (x right, y up, z forward), rotated live
  --               by heading + attitude. y is vertical in both, so the pivot
  --               math is identical either way.
  --  * upsideDown the gun hangs BELOW the mount (CBC VERTICAL_DIRECTION = UP);
  --               flips the trunnion step from +2 to -2. Default false.
  cannon = { x = 0, y = 64, z = 0, gps = false, upsideDown = false,
             offset = { x = 0, y = 0, z = 0 } },
  -- Airship mode: locate the COMPUTER via GPS (wireless modem required),
  -- read ship yaw from the navigation table (CCMinimap-style needle math),
  -- and derive the cannon's world position by rotating the SHARED
  -- cannon.offset (hull-local computer->mount-base vector, x right / y up /
  -- z forward) by the live heading and gimbal attitude. While enabled,
  -- yawOffset means "cannon rest direction relative to ship-forward", so it
  -- stays correct at any heading. Position uses cannon.offset and
  -- cannon.upsideDown -- there is no separate ship offset.
  ship = {
    enabled = false,
    headingOffset = 0,   -- nav-table needle correction, same as CCMinimap
    navTable = "auto",
    -- Aeronautics gimbal sensor (type "gimbal_sensor", getAngles() ->
    -- {xAngleDeg, zAngleDeg} -- see CCMissile). "auto" finds one, a name
    -- wraps it, "none" disables (ship assumed level).
    gimbal = "auto",
    -- Raw gimbal angles -> ship attitude. Conventions the aim math needs:
    -- ship pitch is +nose-up, ship roll is +right-side-down (looking
    -- forward). Verify on the DEBUG tab ("ship pitch"/"ship roll" lines)
    -- and flip the invert flags if a tilt reads with the wrong sign.
    -- *Rest values are what the sensor reads when the ship is perfectly
    -- level; they get subtracted.
    gimbalMap = {
      pitch = "x", roll = "z",
      invertPitch = false, invertRoll = false,
      pitchRest = 0, rollRest = 0,
    },
  },
  -- Subtracted from the world-space yaw so 0 matches the cannon's rest
  -- orientation -- i.e. it IS the home/rest facing. A CALIBRATED value
  -- (cannon.cal): "auto" makes calibrate() measure it from the assembled
  -- rest yaw before any rotation (the block reader reports it the moment
  -- the gun is assembled, no wiggle needed), so each mount gets its own and
  -- a copied cannon.cfg never carries another cannon's home. Editable in the
  -- CONFIG tab (Calibrated group); set back to "auto" to re-measure, or
  -- press O (reload.enabled) to capture it from a fresh disassemble cycle.
  yawOffset = "auto",
  -- Idle park pose ("home"): the azimuth/pitch the barrel returns to when
  -- nothing is targeted. World frame for static mounts, deck frame for
  -- ships (both via mount = homeYaw - yawOffset). Press O with the barrel
  -- where you want it to capture the current pose; also editable here.
  -- This is deliberately SEPARATE from yawOffset: the frame calibration
  -- anchored home implicitly, so re-anchoring the frame used to swing the
  -- park direction.
  homeYaw = 0,
  homePitch = 0,
  -- Added to the computed pitch, in degrees: -1 aims 1 degree below the
  -- target, +1 above. Plain aim bias -- not a sign fix.
  pitchOffset = 0,
  -- Drive sign per axis. "auto" calibrates on next boot: the axis is
  -- nudged a few degrees while the block reader watches which way (and
  -- how fast) the angle actually moves; the sign lands here and the
  -- slew rate in yawDrive/pitchDrive.degPerSecPerRpm. Set back to
  -- "auto" whenever you re-gear the build.
  invertYaw = "auto",
  invertPitch = "auto",
  tolerance = 0.4,  -- degrees of acceptable aim error per axis (lock window)
  -- Best-achievable lock: also count an axis as locked once it's as close
  -- as the hardware can get -- when the drive command has fallen below the
  -- speed controller's minSpeed floor, so no finer correction is possible
  -- (the barrel would only stall or overshoot). Lets you set a tolerance
  -- tighter than the drive can hold and still fire. The achievable precision
  -- is settleBand() -- the WIDER of minSpeed/speedGain and the overshoot
  -- guard's minSpeed*loopT*dps/approach -- so raise speedGain AND/OR approach
  -- to tighten it (whichever is the binding floor). false = strict tolerance.
  lockWhenStalled = true,
  -- Auto-fire range gate: while armed, the fire line holds (status shows
  -- OUT OF RANGE) whenever the aim point is farther than this many
  -- blocks. The turret keeps tracking so fire resumes the moment the
  -- target closes back in; manual F is not gated. Mind that rounds
  -- despawn anyway (cast iron ~99, bronze ~187, steel ~540 blocks).
  maxDistance = 50,
  -- Player targets. getPlayerPos reports the player's FEET (the entity
  -- position -- its Y is the bottom of the bounding box; eyes are ~1.62
  -- above). So the hitbox is FEET-RELATIVE: a box `width` wide rising
  -- `height` blocks from the reported point (a standing player is ~1.8
  -- tall, 0.6 wide), and the turret aims `aimHeight` blocks above the feet
  -- -- 0.9 is centre of mass, the default. The fire gate opens while the
  -- shot would pass within width/2 horizontally and anywhere in 0..height
  -- vertically, even before both axes settle inside `tolerance` (which
  -- still locks on its own at long range, where the box subtends less than
  -- the deadband). Raise aimHeight toward the head, or pad width/height, if
  -- the gate feels too strict.
  playerHitbox = { width = 0.6, height = 1.8, aimHeight = 0.9 },
  -- Predictive lead for player targets: aim where the target WILL be
  -- when the shell arrives -- pos + velocity * (flight time + latency).
  -- Flight time comes from the arc solver (profile + ballistics.lua).
  -- latencySeconds covers detector staleness + redstone + loop
  -- lag on top of flight time. windowSeconds is the velocity estimation
  -- window: speed is measured newest-minus-oldest across a short
  -- position history (adjacent-tick differences are dominated by
  -- detector update jitter). Lower follows jukes faster but jitters
  -- more, higher is steadier but slower to notice turns.
  -- minSpeed (blocks/sec) is a stationary deadband: below it the velocity
  -- is treated as zero so detector jitter on a STANDING player can't wander
  -- the lead point and keep the barrel from settling (a fixed coord locks
  -- because it has no such jitter -- this makes a still player behave the
  -- same). Set it above the standing-jitter speed (~0.3) but below a walk
  -- (~4.3); 1.0 cleanly separates the two. Lead resumes above it.
  lead = {
    enabled = true,
    latencySeconds = 0.15,
    windowSeconds = 0.3,
    minSpeed = 1.0,
  },
  -- Burst hysteresis on the auto-fire gate: once the gate opens, keep
  -- the line high while the miss stays within `widen` x the normal gate
  -- (hitbox / hull ring / tolerance), and only drop it after the miss
  -- has been outside that widened gate for holdSeconds straight. Spends
  -- ammo to keep coverage on a juking target; enabled = false reverts
  -- to the strict gate (line drops the instant the gate closes).
  burst = { enabled = true, widen = 2, holdSeconds = 0.3 },
  -- Tracking loop period in seconds while a target is set. CC timers
  -- quantize to 0.05 (one game tick): 0.05 doubles the aim update rate
  -- for twice the peripheral traffic; the roster (1s) and idle ship fix
  -- (0.5s) cadences stay the same regardless.
  trackSeconds = 0.1,
  -- Ship (transponder) targets: the broadcast position IS the peer's
  -- computer, and destroying it loses the target's coords. So the turret
  -- aims 1.5*avoidRadius BELOW the transponder (into the hull), and the
  -- fire gate opens whenever the shot would land within areaRadius of the
  -- transponder but no closer than avoidRadius: hull hits anywhere in
  -- that ring count, the transponder block itself is never fired on.
  shipTargets = {
    areaRadius = 8,   -- default hull "size" in blocks around the transponder
    avoidRadius = 2,  -- protected bubble around the transponder
    -- Per-callsign overrides, e.g. { CBJK = { areaRadius = 12 } }.
    perShip = {},
  },
  -- Hard travel limits per axis, in mount-frame degrees (the block
  -- reader's CannonYaw/CannonPitch convention: 0 = the rest orientation,
  -- which yawOffset places relative to the ship). A target outside the
  -- arc clamps to the nearest edge: the barrel parks at the limit ready
  -- for re-entry, the status shows OUT OF ARC, and the fire gate only
  -- opens if shots from inside the arc would still hit. Drive errors are
  -- computed unwrapped, so the barrel never slews through the forbidden
  -- zone behind the arc (i.e. through your own ship).
  limits = {
    yaw = { min = -90, max = 90 },
    pitch = { min = -30, max = 60 },
  },
  -- Drive tuning per axis: RPM = error degrees * speedGain (capped at
  -- maxSpeed) PLUS feedforward -- the aim point's own angular rate
  -- divided by degPerSecPerRpm, so the barrel rides a moving setpoint
  -- instead of trailing it. (A pure-P loop trails a crossing target by
  -- targetSpeed/(degPerSecPerRpm*speedGain) blocks at ANY distance --
  -- enough to keep the fire gate shut on anything faster than a walk.)
  --
  -- minSpeed is the speed controller's floor (it can't turn slower than
  -- ~1 RPM). The drive never commands BETWEEN 0 and minSpeed -- a
  -- sub-floor command just stalls the mount in place -- so it either
  -- drives at >= minSpeed or parks at 0. That makes the natural settling
  -- point ~minSpeed/speedGain degrees: HIGHER speedGain parks tighter
  -- (the opposite of the pure-P intuition, because there's no deadband to
  -- overshoot -- it parks the instant it can't usefully drive). Raise
  -- speedGain until the barrel just starts to hunt, then back off a hair;
  -- pair with lockWhenStalled to fire at that floor. Only lower speedGain
  -- if an axis visibly oscillates AROUND the target (overshoot), not when
  -- it stops SHORT (that's the floor -- raise it).
  --
  -- degPerSecPerRpm "auto" is measured during the calibration wiggle (boot
  -- or the CAL button): a direct-drive CBC mount moves 0.75 deg/s per RPM,
  -- gearing changes it. Set it (or an invert flag) back to "auto" after
  -- re-gearing, or just press CAL.
  --
  -- approach is the OVERSHOOT GUARD (the main fix for pre-lock oscillation).
  -- The control loop runs at a finite period (peripheral-limited, often
  -- ~0.2-0.25s); a pure-P command builds up enough slew speed that the barrel
  -- flies PAST the target before the next correction and rings down. This
  -- caps the correction speed to what the mount can stop within one loop
  -- period: |rpm| <= approach * |err| / (loopT * degPerSecPerRpm). 0.5 holds
  -- the approach to half the "just reaches it" speed -- no overshoot even at a
  -- slow loop, verified against in-game traces. A faster loop relaxes the cap
  -- automatically (full speed). Lower = gentler/no overshoot but slower;
  -- raise toward 1+ for a snappier approach (risks overshoot at a slow loop);
  -- a large value (e.g. 10) effectively disables the cap.
  --
  -- kd is the derivative-damping gain (the "D" of a PD loop), 0 = off. With
  -- the approach cap doing the overshoot control it is usually unnecessary
  -- (and a derivative is unreliable at a slow loop -- it lags the oscillation
  -- and can pump it). Leave it 0 unless the cap alone leaves a residual hunt.
  yawDrive = { speedGain = 6, maxSpeed = 256, minSpeed = 1, degPerSecPerRpm = "auto", approach = 0.5, kd = 0 },
  pitchDrive = { speedGain = 3, maxSpeed = 256, minSpeed = 1, degPerSecPerRpm = "auto", approach = 0.5, kd = 0 },
  -- Names listed here are dimmed in the target list as a "friendly"
  -- reminder; they can still be clicked deliberately. Works for player
  -- names AND ship callsigns -- when the cannon's own ship runs CCMinimap,
  -- its transponder shows up in the roster too, so list its callsign here.
  whitelist = {},
}

-- Keys the calibration wiggle MEASURES (vs. hand-authored intent): these
-- persist to cannon.cal, kept out of cannon.cfg so the hand-edited config
-- stays clean and the measured file is safe to delete (CAL rebuilds it).
-- Everything not listed here belongs to cannon.cfg.
local CAL_PATHS = {
  { "peripherals", "yaw" }, { "peripherals", "pitch" },
  { "invertYaw" }, { "invertPitch" },
  { "yawDrive", "degPerSecPerRpm" }, { "yawDrive", "minSpeed" },
  { "pitchDrive", "degPerSecPerRpm" }, { "pitchDrive", "minSpeed" },
  { "yawOffset" }, -- measured from the assembled rest yaw, per mount
  { "homeYaw" }, { "homePitch" }, -- captured at calibration, per mount
}

local function readFile(p)
  if not fs.exists(p) then return nil end
  local f = fs.open(p, "r")
  local s = f.readAll()
  f.close()
  return s
end

local function writeFile(p, s)
  local f = fs.open(p, "w")
  f.write(s)
  f.close()
end

local function readJSON(p)
  local raw = readFile(p)
  if not raw then return nil end
  local ok, parsed = pcall(textutils.unserialiseJSON, raw)
  if ok and type(parsed) == "table" then return parsed end
  return nil
end

local function deepCopy(v)
  if type(v) ~= "table" then return v end
  local out = {}
  for k, val in pairs(v) do out[k] = deepCopy(val) end
  return out
end

-- Overwrite dst with src, recursing into matching sub-tables (src leaves win).
local function deepOverlay(dst, src)
  for k, v in pairs(src) do
    if type(v) == "table" and type(dst[k]) == "table" then
      deepOverlay(dst[k], v)
    else
      dst[k] = deepCopy(v)
    end
  end
end

local function getPath(t, path)
  for _, k in ipairs(path) do
    if type(t) ~= "table" then return nil end
    t = t[k]
  end
  return t
end

local function setPath(t, path, v)
  for i = 1, #path - 1 do
    local k = path[i]
    if type(t[k]) ~= "table" then t[k] = {} end
    t = t[k]
  end
  t[path[#path]] = v
end

local function delPath(t, path)
  for i = 1, #path - 1 do
    local k = path[i]
    if type(t[k]) ~= "table" then return end
    t = t[k]
  end
  t[path[#path]] = nil
end

-- The cannon.cal projection of a config: only the measured paths, nested.
local function calView(src)
  local out = {}
  for _, path in ipairs(CAL_PATHS) do
    local v = getPath(src, path)
    if v ~= nil then setPath(out, path, v) end
  end
  return out
end

-- The cannon.cfg projection: a deep copy with the measured paths removed.
local function cfgView(src)
  local out = deepCopy(src)
  for _, path in ipairs(CAL_PATHS) do delPath(out, path) end
  return out
end

local function writeCfg(c) writeFile(CONFIG, Cfg.jsonPretty(cfgView(c)) .. "\n") end
local function writeCal(c) writeFile(CALFILE, Cfg.jsonPretty(calView(c)) .. "\n") end

local function loadConfig()
  local cfgFile = readJSON(CONFIG) or {}
  local calFile = readJSON(CALFILE)

  -- yawOffset is now a CALIBRATED value (the home/rest facing, auto-measured
  -- into cannon.cal). Drop any copy lingering in cannon.cfg -- including the
  -- old hand-authored default -- so a cannon.cfg copied from another turret
  -- doesn't drag its home position along; it's re-measured for this mount
  -- (and migration below won't lift the stale value into cal either).
  cfgFile.yawOffset = nil

  -- Migration from the old single-file scheme: no cannon.cal yet but the
  -- cfg file carries measured values -> lift them out so a tuned
  -- degPerSecPerRpm / invert / resolved controller names aren't lost.
  local migrated = false
  if calFile == nil then
    calFile = calView(cfgFile)
    migrated = next(calFile) ~= nil
  end

  -- Effective config: defaults, then hand-authored intent (cfg), then
  -- measured values (cal) -- cal wins for its own keys so a fresh wiggle
  -- always overrides a stale copy lingering in cannon.cfg.
  local cfg = deepCopy(DEFAULTS)
  deepOverlay(cfg, cfgFile)
  deepOverlay(cfg, calFile)

  -- Migration: muzzle speed is now COMPUTED from material+barrels. An old
  -- hand-tuned profile.muzzleVelocity (or the even older lead.muzzleVelocity)
  -- becomes muzzleVelocityOverride so the tuned value still fires until the
  -- user switches over to material+barrels.
  local legacyVel = nil
  if type(cfg.lead) == "table" and cfg.lead.muzzleVelocity ~= nil then
    legacyVel = cfg.lead.muzzleVelocity
    cfg.lead.muzzleVelocity = nil
  end
  if cfg.profile.muzzleVelocity ~= nil then
    legacyVel = cfg.profile.muzzleVelocity
    cfg.profile.muzzleVelocity = nil
  end
  if legacyVel and (cfg.profile.muzzleVelocityOverride == nil
      or cfg.profile.muzzleVelocityOverride == 0) then
    cfg.profile.muzzleVelocityOverride = legacyVel
    print(("Carried muzzleVelocity %s -> profile.muzzleVelocityOverride")
      :format(tostring(legacyVel)))
  end

  -- Migration: the player hitbox is now feet-relative (height up from the
  -- reported feet, aim at aimHeight) instead of head-relative up/down +
  -- aimOffset. Drop the dead keys so they don't linger in cannon.cfg.
  if type(cfg.playerHitbox) == "table" then
    cfg.playerHitbox.up = nil
    cfg.playerHitbox.down = nil
    cfg.playerHitbox.aimOffset = nil
  end

  writeCfg(cfg)
  writeCal(cfg)
  if migrated then print("Split calibration values into " .. CALFILE) end
  return cfg
end

local cfg = loadConfig()

-- Cannon profile resolved against the ballistics tables: the projectile
-- constants, the computed muzzle speed (b/s), and the muzzle launch offset.
-- Recomputed by refreshProfile() so live CONFIG-tab edits to the gun take
-- effect without a restart. Boot calls it loudly -- a typo'd projectile or
-- bad barrel length must not become a silent default.
local proj, muzzleSpeed, muzzleLen
local function refreshProfile()
  local p = Ballistics.PROJECTILES[cfg.profile.projectile]
  if not p then
    local known = {}
    for k in pairs(Ballistics.PROJECTILES) do known[#known + 1] = k end
    table.sort(known)
    error(("unknown profile.projectile %q in %s -- known: %s")
      :format(tostring(cfg.profile.projectile), CONFIG,
        table.concat(known, ", ")), 0)
  end
  if cfg.profile.arc ~= "shallow" and cfg.profile.arc ~= "steep" then
    error(('profile.arc must be "shallow" or "steep", got %q')
      :format(tostring(cfg.profile.arc)), 0)
  end
  if type(cfg.profile.barrelBlocks) ~= "number"
    or cfg.profile.barrelBlocks < 1 then
    error("profile.barrelBlocks must be the mount->muzzle length in blocks (>= 1)", 0)
  end
  proj = p
  muzzleSpeed = Ballistics.muzzleSpeed(cfg.profile) -- blocks/sec
  -- CBC spawns the shell ~1.5 blocks short of one-past-the-tip.
  muzzleLen = math.max(0, cfg.profile.barrelBlocks - 1.5)
end
refreshProfile()

-- Spruce C2 link (SPRUCE_PLAN.md): turret.cfg holds the server URL + bearer
-- token (+ optional callsign override), provisioned by the operator next to
-- cannon.cfg. No file = standalone, the spruceLoop just idles. A file that
-- IS there but unusable is a config error, not a "run without C2" case --
-- fail loudly at boot rather than silently never showing up on the map.
local TURRETCFG = "turret.cfg"
local spruceCfg = nil
if fs.exists(TURRETCFG) then
  local t = readJSON(TURRETCFG)
  if not t or type(t.url) ~= "string" or t.url == ""
    or type(t.token) ~= "string" or t.token == "" then
    error(('%s exists but must be JSON with string "url" and "token" '
      .. 'fields (optional "callsign")'):format(TURRETCFG), 0)
  end
  if not http then
    error(TURRETCFG .. " is present but the http API is disabled"
      .. " on this computer/server", 0)
  end
  spruceCfg = {
    url = (t.url:gsub("/+$", "")),
    token = t.token,
    callsign = t.callsign,
    -- Report cadence. Deliberately slower than the ~0.25s control loop so
    -- C2 never competes with tracking accuracy; commands ride the status
    -- response, so this is also the command latency. Default 1s (sentry
    -- acquisition is local now, so C2 only coordinates); floor of 1s.
    statusSeconds = math.max(1, tonumber(t.statusSeconds) or 1),
  }
end

local function need(name, what)
  local p = peripheral.wrap(name)
  if not p then
    error(("missing peripheral %q (%s) -- fix peripherals.* in %s")
      :format(name, what, CONFIG), 0)
  end
  return p
end

-- Resolve a config entry to a peripheral: an explicit name wraps directly;
-- "auto" finds the one attached peripheral of `ptype` and errors if the
-- count is anything but exactly one.
local function resolve(name, ptype, what)
  if name ~= "auto" then return need(name, what) end
  local matches = { peripheral.find(ptype) }
  if #matches == 0 then
    error(("no %s (type %q) attached -- connect one or set an explicit name in %s")
      :format(what, ptype, CONFIG), 0)
  end
  if #matches > 1 then
    local names = {}
    for i, p in ipairs(matches) do names[i] = peripheral.getName(p) end
    error(("found %d of %s (%s) -- set an explicit name in %s")
      :format(#matches, what, table.concat(names, ", "), CONFIG), 0)
  end
  return matches[1]
end

-- yaw/pitch are wrapped here when named explicitly; an "auto" name defers
-- to the calibration wiggle, which nudges each speed controller and reads
-- whether CannonYaw or CannonPitch moved to tell them apart (resolveDrives,
-- in the calibration section). Until then these stay nil, so stopMotors
-- and friends guard for it.
local yaw, pitch
if cfg.peripherals.yaw ~= "auto" then
  yaw = need(cfg.peripherals.yaw, "yaw speed controller")
end
if cfg.peripherals.pitch ~= "auto" then
  pitch = need(cfg.peripherals.pitch, "pitch speed controller")
end
local blockReader = resolve(cfg.peripherals.blockReader, "block_reader",
  "block reader on cannon mount")
local entDet = resolve(cfg.peripherals.playerDetector, "player_detector",
  "player detector")
local relay = resolve(cfg.peripherals.relay, "redstone_relay", "redstone relay")

-- Relay driving the assembly/reload lines for the bigcannon reload cycle.
-- "same" shares the fire relay; a name wraps a second relay block. Only
-- needed when cfg.reload.enabled, but resolved loudly here at boot so a
-- typo'd name fails before the first shot rather than mid-reload.
local reloadRelay = relay
-- Re-resolve the assembly/reload relay against the live config. Called at
-- boot and whenever the reload toggle is edited from the CONFIG tab, so
-- enabling reload at runtime with a separate relay binds it (instead of
-- silently driving the fire relay's sides). Errors loudly on a bad name.
local function refreshReloadRelay()
  if cfg.reload.enabled and cfg.reload.relay ~= "same" then
    reloadRelay = need(cfg.reload.relay, "reload redstone relay")
  else
    reloadRelay = relay
  end
end
refreshReloadRelay()

-- Airship mode prerequisites: a wireless modem for gps.locate and a
-- navigation table for heading. Checked loudly at boot, not at first use.
local navSource = nil
local gimbal = nil
if cfg.ship.enabled then
  if not peripheral.find("modem", function(_, m) return m.isWireless() end) then
    error("ship.enabled but no wireless modem attached (needed for gps.locate)", 0)
  end
  navSource = Heading.discover(
    cfg.ship.navTable ~= "auto" and cfg.ship.navTable or nil)
  if not navSource then
    error("ship.enabled but no navigation table found -- set ship.navTable in "
      .. CONFIG, 0)
  end
  if cfg.ship.gimbal ~= "none" then
    if cfg.ship.gimbal ~= "auto" then
      gimbal = need(cfg.ship.gimbal, "gimbal sensor")
    else
      gimbal = peripheral.find("gimbal_sensor")
      -- nil is tolerated in auto mode (not aim-critical yet); the DEBUG
      -- tab shows NOT FOUND in red rather than failing the boot.
    end
  end
end

-- Transponder targets: CCMinimap ships broadcast a state snapshot on rednet
-- protocol "airship-state" every 0.5s (airshipName + lastPos GPS fix +
-- heading -- see CCMinimap minimap-ui.lua stateSnapshot). Any wireless modem
-- doubles as the receiver; without one the roster just lists no ships
-- (ship.enabled already requires a wireless modem anyway).
local STATE_PROTOCOL = "airship-state"
-- A private transponder protocol for our own lightweight beacons
-- (transponder.lua on a turtle): the turret tracks these, but CCMinimap
-- only listens on STATE_PROTOCOL, so a beacon on this protocol stays off
-- the minimap roster while still showing up here and in Spruce's sniffer.
local BEACON_PROTOCOL = "cannon-transponder"
local PEER_TTL = 5  -- seconds without a broadcast before a ship is dropped
local transponderModem = peripheral.find("modem",
  function(_, m) return m.isWireless() end)
if transponderModem then
  local modemName = peripheral.getName(transponderModem)
  if not rednet.isOpen(modemName) then rednet.open(modemName) end
end

-- Mount BASE -> launch pivot. CBC rotates a mounted cannon about the CENTRE
-- of the block 2 along the mount's vertical axis (the trunnion): the gun
-- assembles at mountPos.relative(VERTICAL_DIRECTION, -2) and the entity's
-- block-centre (+0.5 on every axis) is the fixed point of rotation. So from
-- the mount-block base the pivot is +0.5 on the two horizontals and, on the
-- vertical, the 2-block trunnion step plus the same +0.5 centre: +2.5 for a
-- normal cannon, -1.5 when the gun hangs below the mount (VERTICAL_DIRECTION =
-- UP). VERTICAL_DIRECTION is only ever up/down, so the step is always on y;
-- in ship mode y is hull-up and the result is rotated to world by the caller.
local function pivotFromBase(base, upsideDown)
  return {
    x = base.x + 0.5,
    y = base.y + (upsideDown and -2 or 2) + 0.5,
    z = base.z + 0.5,
  }
end

-- Static-mode cannon position: configured directly, or derived ONCE at
-- boot from a GPS fix of the computer plus a world-axis offset -- a
-- stationary build doesn't move, so one fix is enough (rerun the
-- program after relocating). Fails loudly rather than aiming from a
-- guessed position.
local staticCannon = nil
local staticBase = nil -- mount-base position before pivotFromBase, for the DEBUG tab
local gpsFix = nil -- cached computer GPS fix (static gps mode), reused on live edits
if not cfg.ship.enabled then
  if cfg.cannon.gps then
    if not transponderModem then
      error("cannon.gps = true but no wireless modem attached", 0)
    end
    local x, y, z = gps.locate(2)
    if not x then
      error("cannon.gps = true but no GPS fix -- is a GPS constellation "
        .. "in range? (or set cannon.gps = false and fill cannon x/y/z)", 0)
    end
    gpsFix = { x = x, y = y, z = z }
    local o = cfg.cannon.offset
    staticBase = { x = x + o.x, y = y + o.y, z = z + o.z }
    staticCannon = pivotFromBase(staticBase, cfg.cannon.upsideDown)
    print(("GPS fix %.1f %.1f %.1f + offset -> pivot %.1f %.1f %.1f")
      :format(x, y, z, staticCannon.x, staticCannon.y, staticCannon.z))
    sleep(1.5) -- long enough to read before the UI clears the terminal
  else
    staticBase = { x = cfg.cannon.x, y = cfg.cannon.y, z = cfg.cannon.z }
    staticCannon = pivotFromBase(staticBase, cfg.cannon.upsideDown)
  end
end

-- Live ship fix: computer world position, heading, and the derived cannon
-- position. freshUntil guards against aiming on stale data when GPS or the
-- nav table stop answering.
local ship = {
  pos = nil, heading = nil, cannon = nil, base = nil, rel = nil,
  gimbal = nil,           -- { x, z } raw gimbal-sensor degrees
  pitch = 0, roll = 0,    -- mapped attitude: +nose-up / +right-side-down
  basis = nil,            -- ship-frame unit vectors {f, u, r} in world coords
  freshUntil = 0,
}

-- Raw gimbal angles -> (pitch, roll) degrees per cfg.ship.gimbalMap.
local function shipAttitude()
  local g = ship.gimbal
  if not g then return 0, 0 end
  local m = cfg.ship.gimbalMap
  local p = (g[m.pitch] - m.pitchRest) * (m.invertPitch and -1 or 1)
  local r = (g[m.roll] - m.rollRest) * (m.invertRoll and -1 or 1)
  return p, r
end

-- a*ca + b*cb, componentwise: rotate one basis vector toward another.
local function mix(a, b, ca, cb)
  return {
    x = a.x * ca + b.x * cb,
    y = a.y * ca + b.y * cb,
    z = a.z * ca + b.z * cb,
  }
end

local function updateShip()
  if gimbal then
    local ok, angles = pcall(gimbal.getAngles)
    ship.gimbal = (ok and type(angles) == "table")
      and { x = angles[1], z = angles[2] } or nil
    -- A present-but-unreadable gimbal must not silently mean "level":
    -- skip the update so the fix goes stale and the turret holds.
    if not ship.gimbal then return end
  end
  local x, y, z = gps.locate(0.5)
  local rel = Heading.relativeAngle(navSource)
  if not (x and rel) then return end
  ship.rel = rel
  local heading = Heading.fromPositionAndRelative(
    { x = x, z = z }, rel, cfg.ship.headingOffset)
  if not heading then return end
  -- Ship basis in world coords: start level at this heading (compass
  -- convention: 0 = north/-Z, 90 = east/+X, clockwise), pitch about
  -- ship-right (+nose-up), then roll about ship-forward (+right-down).
  -- Euler-order error is negligible at hover-attitude angles.
  local pitchDeg, rollDeg = shipAttitude()
  local h = math.rad(heading)
  local th, ph = math.rad(pitchDeg), math.rad(rollDeg)
  local f = { x = math.sin(h), y = 0, z = -math.cos(h) }
  local r = { x = math.cos(h), y = 0, z = math.sin(h) }
  local u = { x = 0, y = 1, z = 0 }
  local f2 = mix(f, u, math.cos(th), math.sin(th))
  local u2 = mix(u, f, math.cos(th), -math.sin(th))
  local r2 = mix(r, u2, math.cos(ph), -math.sin(ph))
  local u3 = mix(u2, r, math.cos(ph), math.sin(ph))
  -- Shared mount-base offset (hull-local: x right, y up, z forward) rotated to
  -- world by the live basis. The mount base and the launch pivot differ only
  -- by pivotFromBase IN THE HULL FRAME, so rotate each separately: base for
  -- the DEBUG tab, pivot (base lifted by the trunnion + centre) for aiming.
  local function toWorld(v) -- v = { x right, y up, z forward }, hull-local
    return {
      x = x + f2.x * v.z + u3.x * v.y + r2.x * v.x,
      y = y + f2.y * v.z + u3.y * v.y + r2.y * v.x,
      z = z + f2.z * v.z + u3.z * v.y + r2.z * v.x,
    }
  end
  ship.pos = { x = x, y = y, z = z }
  ship.heading = heading
  ship.pitch, ship.roll = pitchDeg, rollDeg
  ship.basis = { f = f2, u = u3, r = r2 }
  ship.base = toWorld(cfg.cannon.offset)
  ship.cannon = toWorld(pivotFromBase(cfg.cannon.offset, cfg.cannon.upsideDown))
  ship.freshUntil = os.clock() + 3
end

-- Where the cannon is right now (world frame), or nil when the ship fix
-- has gone stale. Static mode returns the boot-resolved position
-- (configured coords, or the one-shot GPS fix + offset).
local function cannonPos()
  if not cfg.ship.enabled then return staticCannon end
  if os.clock() > ship.freshUntil then return nil end
  return ship.cannon
end

local whitelisted = {}
for _, name in ipairs(cfg.whitelist) do whitelisted[name] = true end

-- Shared state between the tracking loop and the input loop.
local running = true
local state = {
  targetKind = nil,  -- "player" | "ship" | "coord", or nil
  targetName = nil,  -- player name / ship callsign / coord label we track
  coordTarget = nil, -- fixed world point {x,y,z} when targetKind == "coord"
  lost = false,      -- target set but offline / other dim / transponder quiet
  noFix = false,     -- ship mode and GPS/nav stopped answering
  outOfArc = false,  -- aim solution clamped to a travel limit
  outOfRange = false,-- aim point beyond cfg.maxDistance (auto-fire held)
  locked = false,    -- both axes within tolerance
  bursting = false,  -- fire line held by burst hysteresis past a closed gate
  armed = false,     -- auto-fire master switch (ARM button / A key)
  firing = false,    -- fire line currently held high by auto-fire
  -- Spruce sentry control, pushed down on every status response (ctl field)
  -- so local acquisition never waits on a C2 round-trip. All default-off:
  -- a turret with no Spruce link behaves exactly as before.
  spruceSentry = false,    -- server says: auto-acquire hostiles locally
  spruceFriendlies = {},   -- name -> true, server's do-not-engage list
  spruceWsUp = false,      -- websocket link live (event pushes enabled)
  spruceZones = {},        -- no-fire columns over sister turrets (ctl)
  zoneBlocked = false,     -- current solution's path enters a zone: hold fire
  sentryTarget = nil,      -- name WE acquired (vs operator/brain-set), so
                           -- local release only ever drops our own pick
  rebootRequest = false,   -- remote reboot, serviced after the ack POST
  yawErr = 0,
  pitchErr = 0,
  dist = nil,        -- distance to the aim point in blocks
  tof = nil,         -- solver time-of-flight to the aim point, seconds
  hasArc = nil,      -- false = ballistically unreachable (NO ARC)
  miss = nil,        -- ship target: radial miss distance in blocks
  missH = nil,       -- player target: signed horizontal / vertical miss
  missV = nil,       --   in blocks (drives the hitbox fire gate)
  lead = nil,        -- player lead debug: { speed, blocks, tof }
  targetRaw = nil,   -- debug: raw detector/transponder point {x,y,z}
  aim = nil,         -- debug: final solved aim point after avoid/lead/aimHeight
  roster = {},       -- { {kind, name, x, y, z, dist?}, ... } sorted by distance
  peerShips = {},    -- transponder ships by callsign: {x,y,z,heading,seenAt}
  mount = nil,       -- last block-reader NBT, for the debug tab
  impact = nil,      -- static-mode diagnostic: predicted impact from the
                     -- ACTUAL barrel angle {x,y,z, off, vmiss, tof}
  flash = nil,       -- transient status message (e.g. "FIRED")
  recalRequest = false, -- UI asked for a calibrate+auto-tune; serviced in trackLoop
  offsetCalRequest = false, -- UI asked for a yaw-offset capture; serviced there too
  calibrating = false,  -- calibrate + auto-tune in progress (status line)
}

local ui = {
  tabs = {
    { id = "targets", label = " TARGETS " },
    { id = "debug", label = " DEBUG " },
    { id = "config", label = " CONFIG " },
  },
  activeTab = "targets",
  cells = {},  -- clickable regions: {col1, col2, row, cmd, name?}
  scroll = 0,
  prompt = nil,  -- active modal line editor: { kind, text, err } (XYZ/cfg)
  cfgSel = 1,    -- selected CONFIG row (index into the visible item list)
  cfgBaseline = nil, -- captured values on tab entry, for dirty/CANCEL
}

-- (Re)derive the static-mode mount position from the live cannon config so the
-- CONFIG tab can toggle gps / edit the offset or manual xyz without a reboot.
-- Manual mode = the typed xyz; gps mode = the cached computer fix plus the
-- world-axis offset (re-locates once if gps was off at boot). Quiet: flashes on
-- a missing fix and keeps the last good position (the loud check is at boot).
local function refreshStaticCannon()
  if cfg.ship.enabled then return end
  if cfg.cannon.gps then
    if not gpsFix and transponderModem then
      local x, y, z = gps.locate(2)
      if x then gpsFix = { x = x, y = y, z = z } end
    end
    if gpsFix then
      local o = cfg.cannon.offset
      staticBase = { x = gpsFix.x + o.x, y = gpsFix.y + o.y, z = gpsFix.z + o.z }
      staticCannon = pivotFromBase(staticBase, cfg.cannon.upsideDown)
    else
      state.flash = transponderModem and "NO GPS FIX" or "NO MODEM"
    end
  else
    staticBase = { x = cfg.cannon.x, y = cfg.cannon.y, z = cfg.cannon.z }
    staticCannon = pivotFromBase(staticBase, cfg.cannon.upsideDown)
  end
end

-- (Re)resolve the ship nav source + gimbal for the current cfg.ship settings,
-- or tear them down and rebuild the static position when ship mode is off.
-- The boot path above does the same resolution but fails loudly; this one
-- SOFT-fails (returns ok, err) so the CONFIG tab can toggle ship mode live
-- and revert a bad edit instead of leaving updateShip with a nil nav source
-- (which it would then call every tick). Assigns navSource/gimbal atomically
-- at the end so a mid-way failure leaves the previous binding intact.
local function refreshShip()
  if not cfg.ship.enabled then
    navSource, gimbal = nil, nil
    refreshStaticCannon()
    return true
  end
  if not transponderModem then
    return false, "ship mode needs a wireless modem (gps.locate)"
  end
  local ns = Heading.discover(
    cfg.ship.navTable ~= "auto" and cfg.ship.navTable or nil)
  if not ns then
    return false, "no navigation table found (set navTable)"
  end
  local gm
  if cfg.ship.gimbal == "none" then
    gm = nil
  elseif cfg.ship.gimbal ~= "auto" then
    local ok, g = pcall(need, cfg.ship.gimbal, "gimbal sensor")
    if not ok then
      return false, "gimbal '" .. tostring(cfg.ship.gimbal) .. "' not found"
    end
    gm = g
  else
    gm = peripheral.find("gimbal_sensor") -- nil tolerated in auto mode
  end
  navSource, gimbal = ns, gm
  ship.freshUntil = 0 -- force a fresh updateShip before the next aim
  return true
end

-- ---------------------------------------------------------------- aiming --

-- Smallest signed angle from `current` to `target`, in [-180, 180].
local function angleDiff(target, current)
  return ((target - current + 180) % 360) - 180
end

-- The smallest aim error this axis can still usefully drive on: below it
-- speedFor commands 0 and the barrel parks. TWO floors can stop it, and the
-- barrel parks at whichever is hit FIRST (the larger error):
--   * the minSpeed floor:        err < minSpeed/speedGain
--   * the overshoot guard's cap:  err < minSpeed*loopT*dps/approach
-- The guard (speedFor's approach cap) was added after this function and is
-- often the binding limit, so ignoring it made lockWhenStalled reject a
-- barrel that genuinely can't aim tighter. loopT (the live loop period) is
-- needed for the guard term; without it, fall back to the minSpeed floor.
local function settleBand(drive, loopT)
  local band = drive.minSpeed / drive.speedGain
  local dps = drive.degPerSecPerRpm
  if type(dps) == "number" and dps > 0 and drive.approach and drive.approach > 0
      and loopT and loopT > 0 then
    band = math.max(band, drive.minSpeed * loopT * dps / drive.approach)
  end
  return band
end

-- An axis is "on target" within the lock tolerance, or -- when
-- lockWhenStalled -- once it's parked at the hardware floor (settleBand),
-- where no finer correction is possible. Used by the fire gates.
local function axisSettled(err, drive, loopT)
  err = math.abs(err)
  if err < cfg.tolerance then return true end
  return cfg.lockWhenStalled and err <= settleBand(drive, loopT)
end

-- Per-axis drive command: proportional on error, feedforward of the
-- setpoint's own angular rate, and a derivative term that damps overshoot --
-- a PD + feedforward loop. ffRate (setpoint deg/s) and dRate (aim-error
-- deg/s) are both converted to RPM through the calibrated degPerSecPerRpm.
-- The D term is kd * d(err)/dt: when the error is closing fast the barrel
-- eases off so it settles instead of overshooting; acting on the ERROR (not
-- the mount velocity) means it stays out of the feedforward's way on a
-- moving target (steady tracking -> ~0 error rate). kd = 0 is pure P+ff.
-- The speed controller can't turn slower than minSpeed, so a sub-minSpeed
-- command does nothing but stall the mount -- the drive therefore commands
-- >= minSpeed or parks at 0, never in between. The park point is settleBand()
-- degrees (the wider of the minSpeed floor and the overshoot-guard cap);
-- lockWhenStalled fires there.
local function speedFor(diff, invert, drive, ffRate, dRate, loopT)
  local ff, d = 0, 0
  local degPerSec = drive.degPerSecPerRpm
  local haveRate = type(degPerSec) == "number" and degPerSec > 0
  if haveRate then
    if ffRate then ff = ffRate / degPerSec end
    if dRate and drive.kd and drive.kd ~= 0 then
      d = drive.kd * dRate / degPerSec
    end
  end
  -- Drive only when the P term can clear the floor, or the setpoint is
  -- itself moving (ff) -- a mover kept live so it doesn't trail, an error
  -- parked at its trailing edge landing every shot a touch behind, which
  -- no latencySeconds value can compensate. Otherwise park at 0: we're
  -- inside settleBand, closer than the controller can usefully turn, so
  -- driving would only stall or hunt (and detector-noise ff can't twitch
  -- a parked barrel). The D term only shapes an already-active command --
  -- it never opens this gate, so it can't drive a parked barrel backwards.
  local p = math.abs(diff) * drive.speedGain
  -- Overshoot guard: never ask for more correction speed than the mount can
  -- stop within one control-loop period. At a finite loop period a pure-P
  -- command builds up enough slew speed to fly past the target before the
  -- next correction (the in-game overshoot/ring-down); capping the P term to
  -- approach*|err|/(loopT*dps) holds the approach to a stoppable speed and
  -- kills the overshoot at any gain. Feedforward is added AFTER, uncapped --
  -- tracking a moving setpoint is a legitimate high speed, not overshoot.
  if haveRate and drive.approach and drive.approach > 0 and loopT and loopT > 0 then
    p = math.min(p, drive.approach * math.abs(diff) / (loopT * degPerSec))
  end
  local rpm = 0
  if p >= drive.minSpeed or math.abs(ff) > 0.5 then
    rpm = math.min(p, drive.maxSpeed)
    if diff < 0 then rpm = -rpm end
    rpm = rpm + ff + d
    -- Don't emit a magnitude the controller floors to zero (the terms can
    -- partly cancel); round up to minSpeed in its direction so the
    -- intended move actually happens.
    if math.abs(rpm) < drive.minSpeed then
      rpm = rpm >= 0 and drive.minSpeed or -drive.minSpeed
    end
  end
  rpm = math.max(-drive.maxSpeed, math.min(rpm, drive.maxSpeed))
  if invert then rpm = -rpm end
  return rpm
end

-- Ballistic solution toward a world-frame offset: dh blocks horizontal,
-- dy up. profile.arc picks between the two solutions when both exist.
-- The solver bounds come from the mount's pitch limits -- mount-frame,
-- which only matches world pitch on a level deck; at hover attitudes
-- the few degrees of difference just shave the extreme ends of the
-- envelope. nil = ballistically unreachable.
local function solveArc(dh, dy)
  local sols = Ballistics.solve{
    v0 = muzzleSpeed, gravity = proj.gravity, drag = proj.drag,
    dx = dh, dy = dy, muzzle = muzzleLen,
    minPitch = cfg.limits.pitch.min, maxPitch = cfg.limits.pitch.max,
  }
  if #sols == 0 then return nil end
  return cfg.profile.arc == "steep" and sols[#sols] or sols[1]
end

-- World-space target position -> desired mount yaw/pitch in degrees,
-- the distance in blocks, the shell's flight time in seconds, and
-- whether a ballistic solution exists. Yaw is geometric; pitch comes
-- from the arc solver in the world vertical plane (gravity is world-
-- down), and the resulting aim DIRECTION is projected through the ship
-- basis -- a rolled deck couples world pitch into both mount axes, so
-- the projection must see the vector, not the angle. With no solution
-- (target past max range) the barrel falls back to line-of-sight pitch
-- as a ready posture and hasArc=false keeps the fire gate shut.
-- Returns nil when the cannon position is unknown (stale ship fix).
local function anglesFor(tx, ty, tz)
  local c = cannonPos()
  if not c then return nil end
  local dx, dy, dz = tx - c.x, ty - c.y, tz - c.z
  local distance = math.sqrt(dx ^ 2 + dy ^ 2 + dz ^ 2)
  local dh = math.sqrt(dx * dx + dz * dz)
  local sol = dh > 1e-6 and solveArc(dh, dy) or nil
  local worldPitch, tof, hasArc
  if sol then
    worldPitch, tof, hasArc = sol.pitch, sol.tof, true
  else
    worldPitch = math.deg(math.asin(dy / distance))
    tof = distance / muzzleSpeed
    hasArc = false
  end
  if cfg.ship.enabled then
    -- Build the world aim direction from the solved pitch and project
    -- it onto the ship frame: the mount's yaw/pitch are deck-relative,
    -- and a rolled deck couples roll into BOTH axes -- a constant
    -- heading subtraction can't express that.
    local pr = math.rad(worldPitch)
    local hx, hz = 0, 0
    if dh > 1e-6 then hx, hz = dx / dh, dz / dh end
    local ux = hx * math.cos(pr)
    local uy = math.sin(pr)
    local uz = hz * math.cos(pr)
    local b = ship.basis
    local df = ux * b.f.x + uy * b.f.y + uz * b.f.z
    local du = ux * b.u.x + uy * b.u.y + uz * b.u.z
    local dr = ux * b.r.x + uy * b.r.y + uz * b.r.z
    local relPitch = math.deg(math.asin(math.max(-1, math.min(1, du))))
      + cfg.pitchOffset
    -- -90: deck yaw is measured from ship-forward here, while the old
    -- yaw-only formula measured from ship-right; keeps the tuned
    -- yawOffset meaning what it always meant.
    local relYaw = angleDiff(math.deg(math.atan(dr, df)) - 90 - cfg.yawOffset, 0)
    return relYaw, relPitch, distance, tof, hasArc
  end
  local relPitch = worldPitch + cfg.pitchOffset
  local worldYaw = math.deg(math.atan(dz, dx))
  return angleDiff(worldYaw - cfg.yawOffset, 0), relPitch, distance, tof, hasArc
end

-- areaRadius/avoidRadius for a ship target, per-callsign override first.
local function shipArea(name)
  local o = cfg.shipTargets.perShip[name] or {}
  return o.areaRadius or cfg.shipTargets.areaRadius,
    o.avoidRadius or cfg.shipTargets.avoidRadius
end

-- Where a shot along the barrel's CURRENT line would pass relative to a
-- point `dist` blocks away, as signed horizontal/vertical offsets in
-- blocks (positive = the shot passes on the +yaw side / above). Gates use
-- the magnitudes; the signs make readouts say which way it's missing.
local function missComponents(yawErr, pitchErr, mountPitch, dist)
  return -dist * math.sin(math.rad(yawErr)) * math.cos(math.rad(mountPitch)),
    -dist * math.sin(math.rad(pitchErr))
end

-- Radial miss distance in blocks (ship targets gate on a sphere).
local function missDistance(yawErr, pitchErr, mountPitch, dist)
  local h, v = missComponents(yawErr, pitchErr, mountPitch, dist)
  return math.sqrt(h * h + v * v)
end

-- ------------------------------------------------------------- prediction --

-- Target velocity estimate: newest-minus-oldest over a short position
-- history window (cfg.lead.windowSeconds). Adjacent-tick differences are
-- dominated by detector update jitter -- positions change on server
-- ticks, so 0.05s deltas alternate between zero and double the true
-- motion; the window averages that out. One shared track -- it resets on
-- every target change. aimYaw/aimPitch/rates feed the drive feedforward.
local track = {
  hist = {},                  -- { {x,y,z,t}, ... } newest last
  vx = 0, vy = 0, vz = 0,
  aimT = nil,                 -- last aim setpoint sample, for feedforward
  yawRate = 0, pitchRate = 0, -- setpoint angular rates, deg/s (feedforward)
  yawErrRate = 0, pitchErrRate = 0, -- aim-error rates, deg/s (D term)
  loopT = 0.2, driveT = nil, -- measured control-loop period (overshoot guard)
}
local LEAD_MIN_SPAN = 0.1  -- seconds of history before velocity is trusted

local function resetLead()
  track.hist = {}
  track.vx, track.vy, track.vz = 0, 0, 0
  track.aimT = nil
  track.yawRate, track.pitchRate = 0, 0
  track.yawErrRate, track.pitchErrRate = 0, 0
  track.yawErrPrev, track.pitchErrPrev = nil, nil
  track.driveT = nil -- keep loopT (a build property, not target-specific)
end

local function updateLead(pos)
  local hist = track.hist
  local t = os.clock()
  local last = hist[#hist]
  if last and t - last.t > 1 then
    -- Sample gap (target was LOST a while): old samples are fiction.
    resetLead()
    hist = track.hist
  end
  hist[#hist + 1] = { x = pos.x, y = pos.y, z = pos.z, t = t }
  local maxSamples = math.max(2,
    math.floor(cfg.lead.windowSeconds / cfg.trackSeconds + 0.5) + 1)
  while #hist > maxSamples do table.remove(hist, 1) end
  local o, n = hist[1], hist[#hist]
  local span = n.t - o.t
  if span >= LEAD_MIN_SPAN then
    local vx = (n.x - o.x) / span
    local vy = (n.y - o.y) / span
    local vz = (n.z - o.z) / span
    -- Stationary deadband: below minSpeed the motion is detector jitter, not
    -- travel. Zeroing it keeps the lead point (and so the aim setpoint) still
    -- enough for the barrel to settle and lock on a standing target.
    if math.sqrt(vx * vx + vy * vy + vz * vz) < (cfg.lead.minSpeed or 0) then
      vx, vy, vz = 0, 0, 0
    end
    track.vx, track.vy, track.vz = vx, vy, vz
  end
end

-- Drive-loop rates, finite-differenced between ticks and lightly smoothed:
-- the SETPOINT angular rates (clamped aim angles -> feedforward, covering
-- target motion / lead / the own ship turning, zero while parked at a
-- limit), and the AIM-ERROR rates (-> the D term, which damps overshoot).
-- Error-derivative is primed on the first sample (no kick from a nil prev).
local function updateRates(aimYaw, aimPitch, yawErr, pitchErr, yawWrap)
  local t = os.clock()
  if track.aimT then
    local dt = t - track.aimT
    if dt > 0 and dt < 0.5 then
      local a = dt / (0.15 + dt)
      -- On a continuous ring the aim point wraps at +/-180; take the
      -- shortest delta so the feedforward rate doesn't spike at the seam.
      local yawDelta = yawWrap and angleDiff(aimYaw, track.aimYaw)
        or (aimYaw - track.aimYaw)
      track.yawRate = track.yawRate
        + a * (yawDelta / dt - track.yawRate)
      track.pitchRate = track.pitchRate
        + a * ((aimPitch - track.aimPitch) / dt - track.pitchRate)
      if track.yawErrPrev then
        track.yawErrRate = track.yawErrRate
          + a * ((yawErr - track.yawErrPrev) / dt - track.yawErrRate)
        track.pitchErrRate = track.pitchErrRate
          + a * ((pitchErr - track.pitchErrPrev) / dt - track.pitchErrRate)
      end
    elseif dt >= 0.5 then
      track.yawRate, track.pitchRate = 0, 0 -- stale: reprime
      track.yawErrRate, track.pitchErrRate = 0, 0
    end
  end
  track.aimYaw, track.aimPitch, track.aimT = aimYaw, aimPitch, t
  track.yawErrPrev, track.pitchErrPrev = yawErr, pitchErr
end

-- ----------------------------------------------------------- diagnostics --

-- Diagnostic trace: while on, the track loop appends the per-tick drive
-- signals AND the fire-gate decision to cannon.trace.csv, so a recorded lock
-- / oscillation / "locked but won't fire" can be read back offline -- the
-- real sensor noise, loop latency, oscillation period, and the exact gate
-- terms the idealized model can't see. Toggle with L (or the CLI); it
-- auto-stops after TRACE_MAX seconds so a forgotten trace can't grow forever.
local TRACE_MAX = 40
local TRACE_FILE = "cannon.trace.csv"
-- Buffered in memory and written once on stop, so recording adds NO per-tick
-- file I/O -- an earlier per-row flush() was itself slowing the loop and
-- contaminating the very timing it was meant to measure. The `dt` column is
-- the real loop period (the thing the overshoot guard adapts to). The gate
-- columns (locked..dist) answer why the auto-fire line is/isn't held.
local trace = { on = false, rows = nil, t0 = 0, lastT = nil }
local function traceStart()
  if trace.on then return end
  trace.rows = { "t,dt,target,aimYaw,mountYaw,yawErr,yawErrRate,yawRpm,"
    .. "aimPitch,mountPitch,pitchErr,pitchErrRate,pitchRpm,"
    .. "locked,bursting,hasArc,outOfRange,outOfArc,missH,missV,dist,phase" }
  trace.t0 = os.clock()
  trace.lastT = nil
  trace.on = true
  state.flash = "TRACE REC"
end
local function traceStop()
  if trace.rows then
    local f = fs.open(TRACE_FILE, "w")
    if f then f.write(table.concat(trace.rows, "\n") .. "\n"); f.close() end
    trace.rows = nil
    state.flash = "TRACE SAVED"
  end
  trace.on = false
end
local function traceRow(aimYaw, mountYaw, yawRpm, aimPitch, mountPitch, pitchRpm, phase)
  if not (trace.on and trace.rows) then return end
  local t = os.clock() - trace.t0
  if t > TRACE_MAX then traceStop(); return end
  local dt = trace.lastT and (t - trace.lastT) or 0
  trace.lastT = t
  local function n(v, fmt) return v and (fmt):format(v) or "" end
  trace.rows[#trace.rows + 1] =
    ("%.3f,%.3f,%s,%.3f,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,"
      .. "%s,%s,%s,%s,%s,%s,%s,%s,%s")
      :format(t, dt, tostring(state.targetName), aimYaw, mountYaw, state.yawErr,
        track.yawErrRate, yawRpm, aimPitch, mountPitch, state.pitchErr,
        track.pitchErrRate, pitchRpm,
        tostring(state.locked), tostring(state.bursting), tostring(state.hasArc),
        tostring(state.outOfRange), tostring(state.outOfArc),
        n(state.missH, "%.2f"), n(state.missV, "%.2f"), n(state.dist, "%.0f"),
        phase or "")
end


-- Intercept point: where the target will be after the shell's flight time
-- (plus fixed latency), with one refinement pass so the flight time is
-- measured to the predicted point, not the current one. Flight time
-- comes from the arc solver (true ballistic TOF); past max range the
-- flat-line estimate keeps the readouts alive while hasArc holds fire.
-- Returns the aim position and the lead time used.
local function leadPoint(pos)
  local c = cannonPos()
  if not c then return pos, 0 end
  local function leadTime(p)
    local dx, dy, dz = p.x - c.x, p.y - c.y, p.z - c.z
    local dh = math.sqrt(dx * dx + dz * dz)
    local sol = dh > 1e-6 and solveArc(dh, dy) or nil
    local t = sol and sol.tof
      or math.sqrt(dx * dx + dy * dy + dz * dz) / muzzleSpeed
    return t + cfg.lead.latencySeconds
  end
  local t1 = leadTime(pos)
  local t2 = leadTime({ x = pos.x + track.vx * t1, y = pos.y + track.vy * t1,
    z = pos.z + track.vz * t1 })
  return {
    x = pos.x + track.vx * t2,
    y = pos.y + track.vy * t2,
    z = pos.z + track.vz * t2,
  }, t2
end

-- Ship-target fire gate: would the shot land within `area` blocks of the
-- transponder, but no closer than `avoid`? Returns gate, missBlocks.
local function hullGate(center, mount, area, avoid)
  local relYaw, relPitch, dist, _, hasArc =
    anglesFor(center.x, center.y, center.z)
  -- No arc to the hull: the "solution" is line-of-sight posture, and a
  -- miss measured against it would be fiction. Gate closed.
  if not relYaw or not hasArc then return false, nil end
  local miss = missDistance(angleDiff(relYaw, mount.CannonYaw),
    relPitch - mount.CannonPitch, mount.CannonPitch, dist)
  return miss <= area and miss >= avoid, miss
end

-- Big-cannon auto-fire pacing (cfg.reload.enabled = false, the autoloader
-- path): one firePulseSeconds pulse per shot, then hold the line low for
-- profile.reloadSeconds. The reload clock survives target changes -- the
-- loader is busy regardless of what the barrel points at.
local pulse = { offAt = 0, nextAt = 0 }

-- Physical reload cycle (cfg.reload.enabled, bigcannon only). A
-- non-blocking phase machine keyed off os.clock(): after a shot the
-- cannon tears down to be reloaded and rebuilds. Phases advance one per
-- track tick when the deadline `at` passes; any phase but "ready" means
-- the contraption is gone, so the drive holds and the fire gate is shut.
-- Ticked at the top of the track loop so the cycle finishes even if the
-- target is dropped mid-reload -- the loader is busy regardless.
local reloadSeq = { phase = "ready", at = 0 }
local function reloadActive()
  return cfg.reload.enabled and reloadSeq.phase ~= "ready"
end
local startShot -- assigned below (needs setFiring); manual fire routes through it

-- Assembly line held HIGH = assembled (the default); reload line is a
-- momentary pulse. Both no-op unless cfg.reload.enabled, so callers don't
-- have to guard. They drive reloadRelay, which is the fire relay unless a
-- separate one is named.
local function setAssembly(on)
  if cfg.reload.enabled then reloadRelay.setOutput(cfg.reload.assemblySide, on) end
end
local function setReloadLine(on)
  if cfg.reload.enabled then reloadRelay.setOutput(cfg.reload.reloadSide, on) end
end

-- Burst hysteresis (cfg.burst): the raw gate opening latches `open`; a
-- closed raw gate then keeps firing while the miss is still within the
-- WIDENED gate, and past that only after holdSeconds straight outside
-- does the latch drop. Loses its state on target change / lost target /
-- missing mount data -- never bridge a gap with stale geometry.
local burst = { open = false, outsideSince = nil }

local function resetBurst()
  burst.open = false
  burst.outsideSince = nil
end

local function burstGate(rawGate, withinWide)
  if not cfg.burst.enabled then return rawGate end
  if rawGate then
    burst.open = true
    burst.outsideSince = nil
    return true
  end
  if not burst.open then return false end
  if withinWide then
    burst.outsideSince = nil
    return true
  end
  local t = os.clock()
  burst.outsideSince = burst.outsideSince or t
  if t - burst.outsideSince < cfg.burst.holdSeconds then return true end
  resetBurst()
  return false
end

local function stopMotors()
  -- nil before the auto-detect wiggle resolves which controller is which.
  if yaw then yaw.setTargetSpeed(0) end
  if pitch then pitch.setTargetSpeed(0) end
end

local function stopAll()
  stopMotors()
  relay.setOutput(cfg.fireSide, false)
  -- Leave the gun assembled (the safe default) with the loader idle. Also
  -- the boot state: stopAll() runs before calibrate(), which needs the
  -- contraption built to nudge the mount.
  setReloadLine(false)
  setAssembly(true)
  reloadSeq.phase, reloadSeq.at = "ready", 0
end

-- True world facing of the barrel from the deck-relative mount angles:
-- the exact INVERSE of anglesFor's ship-basis projection, so displays
-- draw the barrel/arc along the line shells actually leave on a pitched
-- or rolled deck. The flat-deck heading fold only corrected yaw; a ship
-- hovering a degree nose-down sagged every rendered arc below the aim
-- point while the solver (which aims through the full basis) kept
-- hitting. Static mode reduces to the plain offset folds. Returns nil
-- without a fresh ship fix.
local function mountWorldFacing(mount)
  if type(cfg.yawOffset) ~= "number" then return nil end -- "auto" pre-cal
  local deckPitch = mount.CannonPitch - cfg.pitchOffset
  if not cfg.ship.enabled then
    return mount.CannonYaw + cfg.yawOffset, deckPitch
  end
  local b = ship.basis
  if not b or os.clock() > ship.freshUntil then return nil end
  local a = math.rad(mount.CannonYaw + cfg.yawOffset + 90)
  local p = math.rad(deckPitch)
  local ca, sa = math.cos(a), math.sin(a)
  local cp, sp = math.cos(p), math.sin(p)
  local ux = cp * (ca * b.f.x + sa * b.r.x) + sp * b.u.x
  local uy = cp * (ca * b.f.y + sa * b.r.y) + sp * b.u.y
  local uz = cp * (ca * b.f.z + sa * b.r.z) + sp * b.u.z
  return math.deg(math.atan(uz, ux)),
    math.deg(math.asin(math.max(-1, math.min(1, uy))))
end

-- Friendly-fire zones (Spruce ctl): protection columns over sister
-- turrets -- {x, y, z, top, half} where top is the world-Y ceiling a few
-- blocks above the sister's pivot and half is the horizontal half-size
-- (12x12 -> 6). Blocked = the CURRENT barrel facing's shell path enters
-- any column: fly the exact CBC tick path and test footprint + ceiling
-- each tick. This gates PATHS, not bearings -- lobbing OVER a tower at a
-- distant target stays legal, which is the whole point versus blocking
-- a wedge of yaw travel.
local function zonePathBlocked()
  local zones = state.spruceZones
  if not zones or #zones == 0 then return false end
  local mount = state.mount
  if not (mount and mount.CannonYaw and mount.CannonPitch) then return false end
  local wy, wp = mountWorldFacing(mount)
  if not wy then return false end
  local c = cannonPos()
  if not c then return false end
  local minBottom = math.huge
  for i = 1, #zones do
    -- Columns protect ALL the way down (a tower is solid to the ground);
    -- 256 below the ceiling covers any build height while still letting
    -- the integration stop once the shell is below every column.
    local zb = (tonumber(zones[i].top) or c.y) - 256
    if zb < minBottom then minBottom = zb end
  end
  local yawR, pitchR = math.rad(wy), math.rad(wp)
  local hx = math.cos(yawR) * math.cos(pitchR)
  local hy = math.sin(pitchR)
  local hz = math.sin(yawR) * math.cos(pitchR)
  local px = c.x + hx * muzzleLen
  local py = c.y + hy * muzzleLen
  local pz = c.z + hz * muzzleLen
  local v0 = muzzleSpeed / 20
  local vx, vy, vz = hx * v0, hy * v0, hz * v0
  local q = 1 - proj.drag
  local maxd2 = (cfg.maxDistance + 16) ^ 2
  for _ = 1, 600 do
    local nvx, nvy, nvz = vx * q, vy * q + proj.gravity, vz * q
    px = px + 0.5 * (vx + nvx)
    py = py + 0.5 * (vy + nvy)
    pz = pz + 0.5 * (vz + nvz)
    vx, vy, vz = nvx, nvy, nvz
    for i = 1, #zones do
      local z = zones[i]
      if py <= z.top and math.abs(px - z.x) <= z.half
          and math.abs(pz - z.z) <= z.half then
        return true
      end
    end
    local ox, oz = px - c.x, pz - c.z
    -- Past the auto-fire range gate nothing real lands; below every
    -- column's reach the path can't matter either.
    if ox * ox + oz * oz > maxd2 then break end
    if vy < 0 and py < minBottom then break end
  end
  return false
end

-- Phase 4 (Spruce bullets): ring of recent fire events riding the status
-- payload. Each records the firing SOLUTION at trigger time -- world
-- yaw/pitch, muzzle speed, launch pivot -- not a precomputed arc: the
-- browser integrates the same CBC tick model it already draws aim arcs
-- with, which also sidesteps state.impact being static-mode-only.
local spruceShots = {}
local spruceShotSeq = 0
local lastShotRecord = -math.huge
-- CBC autocannons all cycle at 300 rpm with the fire line held (big
-- cannons reload between shots, so edge records cover them). The held-line
-- stream below emits one record per cycle so the browser shows every round.
local AUTOCANNON_INTERVAL = 60 / 300
local streamNextAt = nil -- os.clock() the next held-line round leaves; nil = line low

-- Append one record stamped at `shotClock` (os.clock frame; nil = now).
-- Streamed rounds arrive backdated to when they actually left -- the
-- browser places each bullet by its ts, and the track loop (~0.25s) is
-- slower than the 0.2s gun cycle, so some ticks record two.
local function recordShotAt(shotClock)
  if not spruceCfg then return end -- standalone turret: no bookkeeping
  local mount = state.mount
  if not (mount and mount.CannonYaw and mount.CannonPitch) then return end
  if type(cfg.yawOffset) ~= "number" then return end
  local wy, wp = mountWorldFacing(mount) -- exact world frame (ship basis)
  if not wy then return end
  local p = cannonPos()
  if not p then return end
  local now = os.clock()
  shotClock = shotClock or now
  lastShotRecord = shotClock
  spruceShotSeq = spruceShotSeq + 1
  spruceShots[#spruceShots + 1] = {
    id = spruceShotSeq,
    ts = os.epoch("utc") - math.floor((now - shotClock) * 1000 + 0.5),
    clock = shotClock, -- local TTL prune only (buildSpruceStatus)
    -- COPY the position: cannonPos() returns a shared table
    -- (staticCannon / ship.cannon), and serialiseJSON refuses any table
    -- that appears twice in one payload -- the status body also carries
    -- cannonPos(), and every shot would otherwise share this reference.
    pos = { x = p.x, y = p.y, z = p.z },
    yaw = wy,   -- world azimuth, atan2(dz,dx) frame, deck attitude unwound
    pitch = wp, -- world pitch, ditto
    v0 = muzzleSpeed,
  }
  -- Over the websocket the browser can show this bullet the moment it
  -- exists instead of on the next status tick.
  if state.spruceWsUp then os.queueEvent("spruce_push") end
end

-- Edge-triggered record (manual fire / line edges). The gun itself can't
-- cycle faster than its rate, so collapse edge chatter to one record per
-- cycle (0.95: float slack so a stream record at exactly +interval passes).
local function recordShot()
  if os.clock() - lastShotRecord < AUTOCANNON_INTERVAL * 0.95 then return end
  recordShotAt()
end

-- Held-line autocannon stream: emit one record per gun cycle since the
-- last, each backdated to the moment its round left. A missing pose skips
-- the record but still advances the clock -- no catch-up burst later.
local function recordShotStream()
  if not streamNextAt or cfg.profile.kind ~= "autocannon" then return end
  local now = os.clock()
  while streamNextAt <= now do
    recordShotAt(streamNextAt)
    streamNextAt = streamNextAt + AUTOCANNON_INTERVAL
  end
end

-- Manual single pulse (F key / FIRE button).
local function fire()
  if state.firing then return end -- auto-fire already holds the line high
  if zonePathBlocked() then
    state.flash = "ZONE HOLD -- shell path crosses a friendly column"
    return
  end
  -- Bigcannon with a physical reload: a manual shot empties the gun too,
  -- so route it through the same disassemble/load/assemble cycle (the
  -- track loop drives it). Ignore the trigger while a reload is running.
  if cfg.profile.kind == "bigcannon" and cfg.reload.enabled then
    if reloadSeq.phase ~= "ready" then return end
    startShot(os.clock())
    state.flash = "FIRED"
    return
  end
  recordShot()
  relay.setOutput(cfg.fireSide, true)
  sleep(cfg.firePulseSeconds)
  relay.setOutput(cfg.fireSide, false)
  state.flash = "FIRED"
end

-- Auto-fire (autocannon): hold the fire line high while armed and locked,
-- drop it the moment lock is lost. Bigcannons instead pulse it (the
-- autoloader path) or run the full reload cycle below.
local function setFiring(on)
  if state.firing == on then return end
  state.firing = on
  -- Rising edge = a shot leaves (autocannon stream start / each autoloader
  -- or reload-cycle pulse); trackLoop's recordShotStream covers the rest
  -- of a held line at the gun's cycle rate.
  if on then
    recordShot()
    streamNextAt = os.clock() + AUTOCANNON_INTERVAL
  else
    streamNextAt = nil
  end
  relay.setOutput(cfg.fireSide, on)
end

local function toggleArm()
  state.armed = not state.armed
  if not state.armed then setFiring(false) end
  if state.spruceWsUp then os.queueEvent("spruce_push") end
end

-- Kick off a shot under the physical-reload model: pulse the fire line,
-- then hand off to tickReload, which drops it and runs the cycle.
function startShot(now)
  setFiring(true)
  reloadSeq.phase = "firing"
  reloadSeq.at = now + cfg.firePulseSeconds
end

-- Advance the physical reload cycle. One edge per call (one relay change
-- per track tick keeps the redstone clean); a phase whose deadline hasn't
-- passed is left alone. settleSeconds = 0 just means the next edge fires
-- on the following tick. Called every frame from the track loop.
local function tickReload(now)
  if reloadSeq.phase == "ready" or now < reloadSeq.at then return end
  local p = reloadSeq.phase
  if p == "firing" then
    setFiring(false)        -- end the fire pulse
    if cfg.reload.park then
      -- Slew home before tearing down; tickPark drives + advances out.
      reloadSeq.phase, reloadSeq.at = "parking", now + cfg.reload.parkSeconds
    else
      setAssembly(false)    -- disassemble to expose the breech
      reloadSeq.phase, reloadSeq.at = "disassembled", now + cfg.reload.settleSeconds
    end
  elseif p == "disassembled" then
    setReloadLine(true)     -- pulse the loader
    reloadSeq.phase, reloadSeq.at = "pulsing", now + cfg.reload.reloadPulseSeconds
  elseif p == "pulsing" then
    setReloadLine(false)
    reloadSeq.phase, reloadSeq.at = "loading", now + math.max(0, cfg.profile.reloadSeconds)
  elseif p == "loading" then
    setAssembly(true)       -- loader done: reassemble
    reloadSeq.phase, reloadSeq.at = "assembling", now + cfg.reload.settleSeconds
  elseif p == "assembling" then
    reloadSeq.phase, reloadSeq.at = "ready", 0
  end
end

-- Drive both axes toward the neutral rest pose and report whether they've
-- settled there (motors stopped once they have). Yaw parks where the barrel
-- aims at world-zero (mount-frame -yawOffset, same convention the solver
-- uses) so it matches the gun's neutral facing rather than the block reader's
-- raw zero; pitch parks at mount-frame 0 (level), no offset. Shortest path
-- home: angleDiff handles the continuous-ring +/-180 seam. Shared by the
-- pre-reload park and the idle/lost return-to-rest path.
local function driveToRest()
  local data = blockReader.getBlockData()
  -- Keep the reported mount pose live while homing/idle too. Before this,
  -- state.mount was only written in the tracking branch (and the local
  -- DEBUG tab), so Spruce saw a frozen/missing barrel pose the moment the
  -- turret went idle. (When motors are stopped elsewhere -- e.g. target
  -- lost -- the last value stays correct because nothing is moving.)
  state.mount = data
  if not (data and data.CannonYaw and data.CannonPitch and yaw and pitch) then
    stopMotors()
    return false
  end
  local yawErr = angleDiff(cfg.homeYaw - cfg.yawOffset, data.CannonYaw)
  local pitchErr = cfg.homePitch - data.CannonPitch
  if axisSettled(yawErr, cfg.yawDrive, track.loopT)
    and axisSettled(pitchErr, cfg.pitchDrive, track.loopT) then
    stopMotors()
    return true
  end
  yaw.setTargetSpeed(
    speedFor(yawErr, cfg.invertYaw, cfg.yawDrive, 0, 0, track.loopT))
  pitch.setTargetSpeed(
    speedFor(pitchErr, cfg.invertPitch, cfg.pitchDrive, 0, 0, track.loopT))
  return false
end

-- Optional pre-reload park (cfg.reload.park): drive the gun to its rest
-- pose instead of holding, and tear down only once both axes settle or the
-- parkSeconds deadline (reloadSeq.at) passes. Called from the track loop
-- wherever the reload cycle would otherwise hold the motors, so its drive
-- command is the last word for the tick (it overrides any aim the targeting
-- block computed). No-op stop outside the parking phase.
local function tickPark(now)
  if reloadSeq.phase ~= "parking" then stopMotors(); return end
  local settled = driveToRest()
  if settled or now >= reloadSeq.at then
    stopMotors()
    setAssembly(false)      -- parked (or timed out): now disassemble
    reloadSeq.phase, reloadSeq.at = "disassembled", now + cfg.reload.settleSeconds
  end
end

-- Human-readable reload state for the status / debug lines.
local RELOAD_LABEL = {
  firing = "FIRING", parking = "CENTERING", disassembled = "DISASSEMBLING",
  pulsing = "LOADING", loading = "LOADING", assembling = "ASSEMBLING",
}
local function reloadStatus()
  local label = RELOAD_LABEL[reloadSeq.phase]
  if not label then return nil end
  return label, math.max(0, reloadSeq.at - os.clock())
end

-- Auto-calibrate yawOffset from the gun's rest pose. Disassembly always
-- snaps a CBC cannon back to one fixed rest orientation regardless of where
-- it was aiming, so a disassemble/reassemble cycle (motors idle) leaves the
-- block reader reading that rest yaw -- e.g. 270. The rest IS world-zero, so
-- worldYaw = CannonYaw + yawOffset = 0 there, giving yawOffset = -restYaw
-- (normalized). Pitch is left alone (its rest is level / 0). Needs the
-- assembly relay, so reload.enabled. Returns restYaw, newOffset.
local function calibrateYawOffset()
  if cfg.profile.kind ~= "bigcannon" or not cfg.reload.enabled then
    error("yaw-offset cal needs a bigcannon with reload.enabled (assembly relay)", 0)
  end
  if not blockReader then error("no block reader to read the rest yaw", 0) end
  stopMotors()
  local settle = math.max(1.0, cfg.reload.settleSeconds) + 0.5
  setAssembly(false)        -- disassemble: the gun snaps to its rest pose
  sleep(settle)
  setAssembly(true)         -- reassemble there so the reader reports rest yaw
  sleep(settle)
  local data = blockReader.getBlockData()
  if not (data and data.CannonYaw) then
    error("block reader returned no CannonYaw at rest", 0)
  end
  local rest = data.CannonYaw
  cfg.yawOffset = angleDiff(0, rest)   -- = normalize(-rest); 270 -> 90
  reloadSeq.phase, reloadSeq.at = "ready", 0  -- left assembled, cycle idle
  writeCal(cfg)                        -- yawOffset is a calibrated value
  return rest, cfg.yawOffset
end

-- ----------------------------------------------------------- calibration --

-- Drive `controller` at `rpm` and POLL the mount angle until it has moved at
-- least `targetMove` degrees (and for >= MIN_DRIVE seconds, so spin-up doesn't
-- dominate the rate) or `maxSeconds` elapses. Polling -- rather than nudging a
-- fixed window and comparing to a fixed absolute threshold -- is what makes
-- calibration gear-ratio-agnostic: a heavily geared-down mount turns the
-- reader angle slowly (e.g. 16x reduction = ~0.047 deg/s per RPM), so a fixed
-- short nudge falls under the old 0.5/0.3 deg thresholds and the move is missed
-- (calibration stalls and minSpeed over-reads). Returns the signed delta and
-- the elapsed drive time, measured WHILE driving (before the stop/coast); the
-- caller stops the motor. delta is 0-ish with elapsed=maxSeconds if it never
-- moved. Errors only if the block reader can't see the angle at all.
local MIN_DRIVE = 0.6
local function driveUntilMove(controller, nbtKey, wraps, rpm, targetMove, maxSeconds)
  local data = blockReader.getBlockData()
  local before = data and data[nbtKey]
  if not before then
    error(("calibration failed: block reader has no %s -- is it against the cannon mount?")
      :format(nbtKey), 0)
  end
  controller.setTargetSpeed(rpm)
  local t0, delta, elapsed = os.clock(), 0, 0
  repeat
    sleep(0.1)
    local after = blockReader.getBlockData()[nbtKey]
    delta = wraps and angleDiff(after, before) or (after - before)
    elapsed = os.clock() - t0
  until (math.abs(delta) >= targetMove and elapsed >= MIN_DRIVE) or elapsed >= maxSeconds
  controller.setTargetSpeed(0)
  return delta, elapsed
end

-- Empirically determine the drive sign AND slew rate for one axis: drive the
-- controller and watch which way (and how far) the mount's angle moves. Tries
-- both directions so it still works when the axis starts resting against a
-- clamp (pitch limits). Returns invert, degPerSecPerRpm.
local function calibrateAxis(label, controller, nbtKey, wraps)
  local NUDGE_RPM, MOVE_DEG, MAX_SECONDS = 8, 1.5, 12
  for _, rpm in ipairs({ NUDGE_RPM, -NUDGE_RPM }) do
    local delta, elapsed =
      driveUntilMove(controller, nbtKey, wraps, rpm, MOVE_DEG, MAX_SECONDS)
    sleep(0.2) -- let the angle settle
    if math.abs(delta) >= MOVE_DEG then
      local invert = (delta > 0) ~= (rpm > 0)
      local rate = math.abs(delta) / elapsed / math.abs(rpm)
      print(("calibrated %s: %+d RPM moved %+.1f deg in %.1fs -> invert = %s, %.3f deg/s/RPM")
        :format(label, rpm, delta, elapsed, tostring(invert), rate))
      return invert, rate
    end
  end
  error(("calibration failed: %s axis did not move in either direction -- check gearing")
    :format(label), 0)
end

-- Every attached rotational speed controller, by peripheral name. Create
-- exposes them with "RotationSpeedController" in the type/name; the two on a
-- cannon share a type, which is why they're told apart by motion below.
local function findSpeedControllers()
  local out = {}
  for _, name in ipairs(peripheral.getNames()) do
    local t = peripheral.getType(name)
    if (t and tostring(t):find("RotationSpeedController"))
      or name:find("RotationSpeedController") then
      out[#out + 1] = name
    end
  end
  return out
end

-- Nudge one controller and report which mount axis moved ("CannonYaw" /
-- "CannonPitch"), or nil if neither did. Tries the other direction when the
-- first nudge is blocked by a travel clamp.
local function whichAxisMoves(controller)
  local NUDGE_RPM, MOVE_DEG, MAX_SECONDS = 8, 1.0, 12
  for _, rpm in ipairs({ NUDGE_RPM, -NUDGE_RPM }) do
    local before = blockReader.getBlockData()
    local by, bp = before and before.CannonYaw, before and before.CannonPitch
    if not by or not bp then
      error("auto-detect failed: block reader has no CannonYaw/CannonPitch -- "
        .. "is it against the cannon mount?", 0)
    end
    controller.setTargetSpeed(rpm)
    local t0, dyaw, dpitch, elapsed = os.clock(), 0, 0, 0
    repeat
      sleep(0.1)
      local after = blockReader.getBlockData()
      dyaw = math.abs(angleDiff(after.CannonYaw, by))
      dpitch = math.abs(after.CannonPitch - bp)
      elapsed = os.clock() - t0
    until dyaw >= MOVE_DEG or dpitch >= MOVE_DEG or elapsed >= MAX_SECONDS
    controller.setTargetSpeed(0)
    sleep(0.2)
    if dyaw >= MOVE_DEG or dpitch >= MOVE_DEG then
      return dyaw >= dpitch and "CannonYaw" or "CannonPitch"
    end
  end
  return nil
end

-- Tell the yaw and pitch controllers apart by motion when either name is
-- "auto": nudge each candidate and see which mount axis it drives, then set
-- the yaw/pitch wraps and the resolved names (saved to the cal file by the
-- caller). Returns true if it ran. Loud-fails on an ambiguous rig rather
-- than guessing which is which.
local function resolveDrives()
  if cfg.peripherals.yaw ~= "auto" and cfg.peripherals.pitch ~= "auto" then
    return false
  end
  local cands = findSpeedControllers()
  if #cands ~= 2 then
    error(("auto-detect needs exactly 2 rotation speed controllers, found %d%s "
      .. "-- set peripherals.yaw/pitch explicitly in %s"):format(#cands,
        #cands > 0 and " (" .. table.concat(cands, ", ") .. ")" or "", CONFIG), 0)
  end
  local byAxis = {}
  for _, name in ipairs(cands) do
    local axis = whichAxisMoves(peripheral.wrap(name))
    if axis then
      if byAxis[axis] then
        error(("auto-detect: %s and %s both drove %s -- name peripherals.yaw/"
          .. "pitch explicitly in %s"):format(byAxis[axis], name, axis, CONFIG), 0)
      end
      byAxis[axis] = name
    end
  end
  if not byAxis.CannonYaw or not byAxis.CannonPitch then
    error("auto-detect: a controller didn't move the mount (or only one axis "
      .. "responded) -- check gearing, or name peripherals.yaw/pitch in "
      .. CONFIG, 0)
  end
  cfg.peripherals.yaw, cfg.peripherals.pitch = byAxis.CannonYaw, byAxis.CannonPitch
  yaw, pitch = peripheral.wrap(byAxis.CannonYaw), peripheral.wrap(byAxis.CannonPitch)
  print(("detected drives: yaw=%s pitch=%s")
    :format(byAxis.CannonYaw, byAxis.CannonPitch))
  return true
end

-- Find the lowest commanded RPM that actually turns the mount -- the speed
-- controller's floor, below which a command just stalls. Probes small
-- magnitudes (the mount barely drifts) and steps each non-mover back so
-- successive probes don't walk it into a limit. Returns the floor in RPM,
-- or nil if even the top probe didn't move (caller falls back to 1).
local function probeMinSpeed(label, controller, nbtKey, wraps)
  -- Integer candidates only: setTargetSpeed floors to a whole RPM, so 0.5/1.5
  -- would just command 0/1. A small move threshold + generous timeout lets the
  -- slow drift of a geared-down mount register, so a healthy mount reads its
  -- true floor of 1 RPM instead of over-reporting when the window is too short.
  local MOVE_DEG, MAX_SECONDS = 0.1, 6
  for _, rpm in ipairs({ 1, 2, 3, 4, 5 }) do
    local delta = driveUntilMove(controller, nbtKey, wraps, rpm, MOVE_DEG, MAX_SECONDS)
    sleep(0.2)
    if math.abs(delta) >= MOVE_DEG then
      print(("calibrated %s minSpeed: %d RPM moved %.2f deg"):format(label, rpm, delta))
      return rpm
    end
    driveUntilMove(controller, nbtKey, wraps, -rpm, MOVE_DEG, MAX_SECONDS) -- step back
    sleep(0.2)
  end
  return nil
end

-- Resolve which controller is which (if "auto"), then for each axis whose
-- invert flag or drive rate is "auto", wiggle to measure sign + slew rate
-- and probe the minSpeed floor. Persists everything measured to cannon.cal
-- (not cannon.cfg). An explicit (non-auto) invert flag is kept even when
-- the rate triggers the nudge.
local function calibrate()
  -- Home/rest facing FIRST, before anything rotates the mount. A freshly
  -- assembled cannon sits at its rest yaw and the block reader reports it
  -- directly, so yawOffset = -restYaw (mount-frame 0 = the rest facing) needs
  -- no wiggle -- which is why this runs ahead of resolveDrives / the axis
  -- wiggle. Only when "auto" (unset, or reset to re-measure), so an already
  -- calibrated mount isn't re-read off-rest on a later boot.
  local changed = false
  if cfg.yawOffset == "auto" then
    if not cfg.ship.enabled then
      -- STATIC mounts need no measurement at all: CBC's CannonYaw NBT is
      -- an ABSOLUTE world yaw (MC convention -- initialized from
      -- getContraptionDirection():toYRot() and world-framed thereafter;
      -- verified against the CBC source). Converting MC yaw (south=0) to
      -- our atan2(dz,dx) frame (east=0) is a fixed +90 for EVERY static
      -- mount regardless of build facing. The brief "measure the rest
      -- yaw" approach silently assumed east-facing builds and was 90/180
      -- deg wrong on any other orientation.
      cfg.yawOffset = 90
      -- Fresh calibration also captures HOME from wherever the barrel
      -- sits right now -- on a fresh build that's its built rest
      -- direction, so a new turret parks the way it was built without
      -- any setup. Re-home any time: aim the barrel and press O.
      if blockReader then
        local data = blockReader.getBlockData()
        if data and type(data.CannonYaw) == "number" then
          cfg.homeYaw = angleDiff(data.CannonYaw + cfg.yawOffset, 0)
          cfg.homePitch = type(data.CannonPitch) == "number"
            and math.floor(data.CannonPitch * 10 + 0.5) / 10 or 0
        end
      end
      print(("Static mount: yawOffset = 90 (world-absolute); home az %s")
        :format(tostring(cfg.homeYaw)))
    else
      -- SHIP mounts: CannonYaw is absolute in the SHIP GRID frame, and
      -- the deck offset depends on how the grid axes line up with
      -- heading.lua's forward convention -- genuinely per-build, so it
      -- stays measured. A freshly assembled cannon sits at its rest yaw
      -- and the block reader reports it directly.
      if not blockReader then error("no block reader to read the rest yaw", 0) end
      local data = blockReader.getBlockData()
      if not (data and type(data.CannonYaw) == "number") then
        error("can't read the rest yaw (no CannonYaw) -- is the cannon assembled?", 0)
      end
      cfg.yawOffset = angleDiff(0, data.CannonYaw) -- = normalize(-rest); 270 -> 90
      print(("Rest yaw %.1f -> yawOffset %.1f (deck frame)"):format(
        data.CannonYaw, cfg.yawOffset))
    end
    changed = true
  end
  if resolveDrives() then changed = true end
  -- Names from cal are wrapped at load; this only fires on an edge case.
  if not yaw then yaw = need(cfg.peripherals.yaw, "yaw speed controller") end
  if not pitch then pitch = need(cfg.peripherals.pitch, "pitch speed controller") end

  local function axis(invertKey, drive, label, controller, nbtKey, wraps)
    if cfg[invertKey] ~= "auto" and drive.degPerSecPerRpm ~= "auto" then
      return
    end
    local invert, rate = calibrateAxis(label, controller, nbtKey, wraps)
    if cfg[invertKey] == "auto" then cfg[invertKey] = invert end
    drive.degPerSecPerRpm = rate
    drive.minSpeed = probeMinSpeed(label, controller, nbtKey, wraps) or 1
    changed = true
  end
  axis("invertYaw", cfg.yawDrive, "yaw", yaw, "CannonYaw", true)
  axis("invertPitch", cfg.pitchDrive, "pitch", pitch, "CannonPitch", false)
  if changed then
    writeCal(cfg)
    print("Calibration saved to " .. CALFILE)
    sleep(1)
  end
end

-- ------------------------------------------------------------- auto-tune --

-- Measure the real control-loop period WITHOUT needing a target: run the track
-- loop's per-tick peripheral work (block read + both drive writes + the paced
-- sleep) a few times and take the median os.clock delta. draw() is pure
-- terminal output (no game-tick yield), so leaving it out doesn't change the
-- period. Feeds the live overshoot guard and the auto-tune.
local function measureLoopPeriod()
  local periods = {}
  for i = 1, 16 do
    local t0 = os.clock()
    blockReader.getBlockData()
    if yaw then yaw.setTargetSpeed(0) end
    if pitch then pitch.setTargetSpeed(0) end
    sleep(math.max(0.05, cfg.trackSeconds - (os.clock() - t0)))
    if i > 4 then periods[#periods + 1] = os.clock() - t0 end
  end
  table.sort(periods)
  return periods[math.floor(#periods / 2) + 1] or cfg.trackSeconds
end

local TUNE_LOG = "cannon.tune.log"
local function tuneLog(msg)
  local f = fs.open(TUNE_LOG, "a")
  if f then f.writeLine(msg); f.close() end
end

-- Abstract axis interface autotune.lua drives: read the mount angle, command
-- the controller, carry the calibrated constants + the measured loop period.
local function tuneAxisIO(controller, nbtKey, drive, lim)
  return {
    dps = drive.degPerSecPerRpm, minSpeed = drive.minSpeed,
    maxSpeed = drive.maxSpeed, loopT = track.loopT,
    lo = lim.min, hi = lim.max, log = tuneLog,
    readAngle = function()
      local d = blockReader.getBlockData()
      return d and d[nbtKey]
    end,
    setRpm = function(rpm) controller.setTargetSpeed(rpm) end,
    wait = function(s) sleep(s) end,
    now = function() return os.clock() end,
  }
end

-- Auto-tune the per-axis approach cap by measured step responses: the largest
-- no-overshoot value, found by driving real steps through the live control law
-- on INTERNAL angle targets (no world target needed). Writes the tuned approach
-- + speedGain to cannon.cfg, logs every probe to cannon.tune.log.
local function runAutotune()
  local f = fs.open(TUNE_LOG, "w")
  if f then f.writeLine("CCBigCannon drive auto-tune"); f.close() end
  tuneLog(("loop period: %.0f ms"):format(track.loopT * 1000))
  local function ctrl(inv)
    return function(err, drive, loopT)
      return speedFor(err, inv, drive, nil, nil, loopT) -- D off; the cap damps
    end
  end
  tuneLog("--- yaw ---")
  local ry = Autotune.tuneAxis(
    tuneAxisIO(yaw, "CannonYaw", cfg.yawDrive, cfg.limits.yaw), ctrl(cfg.invertYaw))
  stopMotors()
  tuneLog("--- pitch ---")
  local rp = Autotune.tuneAxis(
    tuneAxisIO(pitch, "CannonPitch", cfg.pitchDrive, cfg.limits.pitch),
    ctrl(cfg.invertPitch))
  stopMotors()
  local function r2(v) return math.floor(v * 100 + 0.5) / 100 end
  local function r1(v) return math.floor(v * 10 + 0.5) / 10 end
  cfg.yawDrive.approach, cfg.yawDrive.speedGain = r2(ry.approach), r1(ry.speedGain)
  cfg.pitchDrive.approach, cfg.pitchDrive.speedGain = r2(rp.approach), r1(rp.speedGain)
  cfg.yawDrive.kd, cfg.pitchDrive.kd = 0, 0 -- the cap supersedes the D term
  writeCfg(cfg)
  tuneLog(("RESULT  yaw: approach %.2f gain %.1f | pitch: approach %.2f gain %.1f")
    :format(cfg.yawDrive.approach, cfg.yawDrive.speedGain,
      cfg.pitchDrive.approach, cfg.pitchDrive.speedGain))
end

-- The CAL button / K key: full automatic drive setup in ONE action. Re-detects
-- which controller is which (if "auto"), re-measures sign + slew rate + the
-- minSpeed floor (the wiggle), measures the real loop period, then auto-tunes
-- the per-axis approach cap by driving step responses on internal targets --
-- no world target needed. Takes ~1-2 min (the barrel steps on its own). Runs
-- from the track loop, which owns the motors. Boot only does the quick wiggle
-- (calibrate()); the tune lives here so it doesn't lengthen every boot.
local function recalibrate()
  cfg.invertYaw, cfg.yawDrive.degPerSecPerRpm = "auto", "auto"
  cfg.invertPitch, cfg.pitchDrive.degPerSecPerRpm = "auto", "auto"
  -- Re-measure the home/rest facing too (barrel sits at rest while idle).
  cfg.yawOffset = "auto"
  calibrate()
  track.loopT = measureLoopPeriod()
  runAutotune()
end

-- -------------------------------------------------------------- targeting --

-- Ingest one transponder broadcast (CCMinimap handlePeerState pattern).
-- lastPos is the peer computer's GPS fix -- full 3D, exactly what the aim
-- math needs. Broadcasts without a numeric y are useless here and dropped.
local function handlePeerState(msg)
  if type(msg) ~= "table" then return end
  local name = msg.airshipName
  if type(name) ~= "string" or name == "" then return end
  local pos = msg.lastPos
  if type(pos) ~= "table" or type(pos.x) ~= "number"
    or type(pos.y) ~= "number" or type(pos.z) ~= "number" then return end
  state.peerShips[name] = {
    x = pos.x, y = pos.y, z = pos.z,
    heading = msg.shipHeading,
    seenAt = os.clock(),
  }
end

local function peerFresh(peer)
  return peer ~= nil and (os.clock() - peer.seenAt) <= PEER_TTL
end

-- Current world position of the tracked target, or nil when it's lost
-- (player offline/other dim, ship transponder gone quiet).
local function targetPos()
  if state.targetKind == "coord" then
    return state.coordTarget -- a fixed point, never lost
  end
  if state.targetKind == "ship" then
    local peer = state.peerShips[state.targetName]
    return peerFresh(peer) and peer or nil
  end
  -- Second arg = decimal places (AP floors to whole blocks by default).
  local pos = entDet.getPlayerPos(state.targetName, 2)
  return (pos and pos.x) and pos or nil
end

-- Refresh the target roster: online players plus fresh transponder ships.
-- getPlayerPos is position-independent (any distance, same dimension) so
-- this works from an assembled airship; players in another dimension get
-- no coords and show as untargetable. Stale ships are evicted here.
local function refreshRoster()
  local c = cannonPos() -- may be nil on a stale ship fix: rows lose distances
  local roster = {}
  local function add(item)
    if item.x and c then
      local dx, dy, dz = item.x - c.x, item.y - c.y, item.z - c.z
      item.dist = math.floor(math.sqrt(dx * dx + dy * dy + dz * dz))
    end
    roster[#roster + 1] = item
  end
  for _, name in ipairs(entDet.getOnlinePlayers() or {}) do
    local item = { kind = "player", name = name }
    local pos = entDet.getPlayerPos(name, 2)
    if pos and pos.x then
      item.x, item.y, item.z = pos.x, pos.y, pos.z
      -- AP also reports facing (MC convention: yaw 0 = south, pitch + =
      -- down); pass it through so the Spruce 3D view can orient the
      -- player marker. Older AP builds just omit the fields.
      item.yaw, item.pitch = tonumber(pos.yaw), tonumber(pos.pitch)
    end
    add(item)
  end
  for name, peer in pairs(state.peerShips) do
    if peerFresh(peer) then
      -- heading comes from CCMinimap airship-state broadcasts (compass
      -- degrees, 0 = north); the private transponder beacons have none.
      add({ kind = "ship", name = name, x = peer.x, y = peer.y, z = peer.z,
        heading = peer.heading })
    else
      state.peerShips[name] = nil
    end
  end
  table.sort(roster, function(a, b)
    local da, db = a.dist or math.huge, b.dist or math.huge
    if da ~= db then return da < db end
    if a.kind ~= b.kind then return a.kind < b.kind end -- players first
    return a.name < b.name
  end)
  state.roster = roster
end

local function setTarget(kind, name)
  if state.targetKind == kind and state.targetName == name then
    kind, name = nil, nil -- click again to release
  end
  state.targetKind = kind
  state.targetName = name
  state.lost = false
  state.locked = false
  state.bursting = false
  state.outOfArc = false
  state.outOfRange = false
  state.miss, state.missH, state.missV = nil, nil, nil
  state.lead = nil
  state.dist = nil
  state.tof, state.hasArc = nil, nil
  state.impact = nil
  state.targetRaw, state.aim = nil, nil
  resetLead()
  resetBurst()
  setFiring(false) -- never carry a held fire line across a target change
  if not name then stopMotors() end
  if state.spruceWsUp then os.queueEvent("spruce_push") end
end

-- Parse free-form "x y z" (any mix of spaces/commas) into a world point,
-- or nil + a short reason. Accepts decimals and negatives.
local function parseCoord(s)
  local nums = {}
  for tok in tostring(s):gmatch("[-%d%.]+") do
    nums[#nums + 1] = tonumber(tok)
  end
  if #nums ~= 3 or not (nums[1] and nums[2] and nums[3]) then
    return nil, "need: x y z"
  end
  return { x = nums[1], y = nums[2], z = nums[3] }
end

-- Lock onto a fixed XYZ point (a stationary aim target, mainly for
-- range/falloff testing). Reuses setTarget so all the per-target state
-- resets the same way; the label doubles as the "name" for the status line.
local function setCoordTarget(c)
  state.coordTarget = c
  setTarget("coord", ("%g, %g, %g"):format(c.x, c.y, c.z))
end

-- Can this turret actually put a shell on the contact RIGHT NOW: a real
-- ballistic solution within the pitch limits (anglesFor/solveArc), the
-- contact inside the auto-fire range gate, and the solved yaw/pitch
-- inside the travel limits (same midpoint-re-centered clamp test the
-- track loop uses, so off-center arcs like 0..180 judge the correct
-- edge). Used to gate sentry acquisition -- a lock the fire gate could
-- never open just swings the barrel at someone we can't hit.
local function sentryCanHit(c)
  if not (c.x and c.y and c.z) then return false end
  local relYaw, relPitch, dist, _, hasArc = anglesFor(c.x, c.y, c.z)
  if relYaw == nil then return false end -- no cannon fix
  if not hasArc then return false end
  if dist > cfg.maxDistance then return false end
  local lim = cfg.limits
  if (lim.yaw.max - lim.yaw.min) < 359 then
    local yawMid = (lim.yaw.min + lim.yaw.max) / 2
    local ty = yawMid + angleDiff(relYaw, yawMid)
    if ty < lim.yaw.min or ty > lim.yaw.max then return false end
  end
  if relPitch < lim.pitch.min or relPitch > lim.pitch.max then return false end
  return true
end

-- Local sentry acquisition. When Spruce flags this turret as a sentry (ctl
-- on the status response), grab the nearest HITTABLE non-friendly from our
-- OWN roster the moment one appears: the ~1s roster tick beats the C2
-- round-trip, so first lock doesn't wait on the server. The brain still
-- coordinates -- a remote "target" command overrides our pick (e.g. to
-- distribute two hostiles across two turrets). We only ever release a
-- target WE picked: when the roster loses it, or when it stays
-- unhittable (lost / out of arc / out of range / no solution) for a few
-- seconds -- the grace keeps a target dancing on the range edge from
-- flapping the barrel between lock and rest. Operator/brain-set targets
-- are never touched.
local SENTRY_DROP_SECONDS = 2.5
local function sentryAcquire()
  if state.calibrating then return end
  -- Stand down our own pick when the roster loses it or it stays
  -- unhittable past the grace window.
  if state.sentryTarget then
    if state.sentryTarget ~= state.targetName then
      state.sentryTarget = nil -- operator/brain retargeted; theirs now
      state.sentryBadSince = nil
    else
      local still = false
      for _, c in ipairs(state.roster or {}) do
        if c.name == state.targetName then still = true break end
      end
      local bad = not still or state.lost or state.outOfArc
        or state.outOfRange or state.hasArc == false
      if not bad then
        state.sentryBadSince = nil
      elseif not still then -- gone from the roster entirely: drop now
        state.sentryTarget = nil
        state.sentryBadSince = nil
        setTarget(nil)
        state.flash = "SENTRY: contact lost"
      else
        state.sentryBadSince = state.sentryBadSince or os.clock()
        if os.clock() - state.sentryBadSince > SENTRY_DROP_SECONDS then
          state.sentryTarget = nil
          state.sentryBadSince = nil
          setTarget(nil)
          state.flash = "SENTRY: target unreachable"
        end
      end
    end
  end
  if not state.spruceSentry or state.targetName then return end
  local fr = state.spruceFriendlies or {}
  for _, c in ipairs(state.roster or {}) do -- sorted nearest-first
    if (c.kind == "player" or c.kind == "ship") and not fr[c.name]
        and sentryCanHit(c) then
      setTarget(c.kind, c.name)
      state.sentryTarget = c.name
      state.sentryBadSince = nil
      state.flash = "SENTRY: engaging " .. c.name
      return
    end
  end
end

-- ------------------------------------------------------------- config tab --

-- Sorted projectile names for the enum cycler.
local PROJECTILE_NAMES = {}
for k in pairs(Ballistics.PROJECTILES) do
  PROJECTILE_NAMES[#PROJECTILE_NAMES + 1] = k
end
table.sort(PROJECTILE_NAMES)

-- Live-editable settings, CCMinimap-style. etype: "num" (+/- step, clamped
-- to min/max, also text-editable), "enum" (cycle a fixed value list), "text"
-- (modal entry, parsed to a number when numeric). file: which file SAVE
-- writes it to (cfg = intent, cal = measured). profile=true rebuilds the
-- cached gun (muzzle speed etc.) on change. show gates visibility on kind.
local CONFIG_ITEMS = {
  { group = "Identity", label = "callsign", etype = "text", file = "cfg",
    get = function() return cfg.callsign end,
    set = function(v) cfg.callsign = v end },
  { group = "Build", label = "kind", etype = "enum", file = "cfg", profile = true,
    values = { "autocannon", "bigcannon" },
    get = function() return cfg.profile.kind end,
    set = function(v) cfg.profile.kind = v end },
  { group = "Build", label = "projectile", etype = "enum", file = "cfg", profile = true,
    values = PROJECTILE_NAMES,
    get = function() return cfg.profile.projectile end,
    set = function(v) cfg.profile.projectile = v end },
  { group = "Build", label = "material", etype = "enum", file = "cfg", profile = true,
    values = { "cast_iron", "bronze", "steel" },
    show = function() return cfg.profile.kind == "autocannon" end,
    get = function() return cfg.profile.material end,
    set = function(v) cfg.profile.material = v end },
  { group = "Build", label = "barrels", etype = "num", file = "cfg", profile = true,
    min = 0, max = 16, step = 1,
    show = function() return cfg.profile.kind == "autocannon" end,
    get = function() return cfg.profile.barrels end,
    set = function(v) cfg.profile.barrels = v end },
  { group = "Build", label = "speedOverride", etype = "num", file = "cfg", profile = true,
    min = 0, max = 400, step = 10,
    show = function() return cfg.profile.kind == "autocannon" end,
    get = function() return cfg.profile.muzzleVelocityOverride end,
    set = function(v) cfg.profile.muzzleVelocityOverride = v end },
  { group = "Build", label = "charges", etype = "num", file = "cfg", profile = true,
    min = 1, max = 10, step = 1,
    show = function() return cfg.profile.kind == "bigcannon" end,
    get = function() return cfg.profile.charges end,
    set = function(v) cfg.profile.charges = v end },
  { group = "Build", label = "barrelBlocks", etype = "num", file = "cfg", profile = true,
    min = 1, max = 24, step = 1,
    get = function() return cfg.profile.barrelBlocks end,
    set = function(v) cfg.profile.barrelBlocks = v end },
  { group = "Build", label = "arc", etype = "enum", file = "cfg", profile = true,
    values = { "shallow", "steep" },
    get = function() return cfg.profile.arc end,
    set = function(v) cfg.profile.arc = v end },
  { group = "Build", label = "reloadSecs", etype = "num", file = "cfg",
    min = 0, max = 30, step = 0.5,
    show = function() return cfg.profile.kind == "bigcannon" end,
    get = function() return cfg.profile.reloadSeconds end,
    set = function(v) cfg.profile.reloadSeconds = v end },
  { group = "Build", label = "reload enabled", etype = "enum", file = "cfg",
    values = { true, false }, reloadDep = true,
    show = function() return cfg.profile.kind == "bigcannon" end,
    get = function() return cfg.reload.enabled end,
    set = function(v) cfg.reload.enabled = v end },
  { group = "Build", label = "reload park", etype = "enum", file = "cfg",
    values = { true, false },
    show = function() return cfg.profile.kind == "bigcannon"
      and cfg.reload.enabled end,
    get = function() return cfg.reload.park end,
    set = function(v) cfg.reload.park = v end },
  { group = "Build", label = "parkSecs", etype = "num", file = "cfg",
    min = 0, max = 30, step = 0.5,
    show = function() return cfg.profile.kind == "bigcannon"
      and cfg.reload.enabled and cfg.reload.park end,
    get = function() return cfg.reload.parkSeconds end,
    set = function(v) cfg.reload.parkSeconds = v end },
  -- How long the fire line pulses per shot (manual fire, the bigcannon
  -- autoloader, and the physical-reload trigger; autocannons HOLD the
  -- line while auto-firing, so this only shapes their manual taps).
  { group = "Build", label = "firePulseSeconds", etype = "num", file = "cfg",
    min = 0.05, max = 2, step = 0.05,
    get = function() return cfg.firePulseSeconds end,
    set = function(v) cfg.firePulseSeconds = v end },
  -- Captured at calibration (per physical mount), so cal-file: it travels
  -- with yawOffset, a settings-only clone leaves it alone, and wiping
  -- cannon.cal re-derives it from the barrel's rest pose on next boot.
  { group = "Calibrated", label = "home yaw", etype = "num", file = "cal",
    min = -180, max = 180, step = 5,
    get = function() return cfg.homeYaw end,
    set = function(v) cfg.homeYaw = v end },
  { group = "Calibrated", label = "home pitch", etype = "num", file = "cal",
    min = -30, max = 60, step = 1,
    get = function() return cfg.homePitch end,
    set = function(v) cfg.homePitch = v end },
  { group = "Aim", label = "pitchOffset", etype = "num", file = "cfg",
    min = -45, max = 45, step = 1,
    get = function() return cfg.pitchOffset end,
    set = function(v) cfg.pitchOffset = v end },
  { group = "Aim", label = "tolerance", etype = "num", file = "cfg",
    min = 0.1, max = 10, step = 0.1,
    get = function() return cfg.tolerance end,
    set = function(v) cfg.tolerance = v end },
  { group = "Aim", label = "maxDistance", etype = "num", file = "cfg",
    min = 10, max = 2000, step = 10,
    get = function() return cfg.maxDistance end,
    set = function(v) cfg.maxDistance = v end },
  { group = "Aim", label = "lockStalled", etype = "enum", file = "cfg",
    values = { true, false },
    get = function() return cfg.lockWhenStalled end,
    set = function(v) cfg.lockWhenStalled = v end },
  { group = "Aim", label = "lead", etype = "enum", file = "cfg",
    values = { true, false },
    get = function() return cfg.lead.enabled end,
    set = function(v) cfg.lead.enabled = v end },
  { group = "Aim", label = "lead minSpd", etype = "num", file = "cfg",
    min = 0, max = 6, step = 0.1,
    show = function() return cfg.lead.enabled end,
    get = function() return cfg.lead.minSpeed end,
    set = function(v) cfg.lead.minSpeed = v end },
  -- Position. All coords point at the mount BASE block; the launch pivot is
  -- derived (pivotFromBase). gps toggles manual xyz vs a GPS-fix-plus-offset
  -- derivation; static = true re-derives the mount live (refreshStaticCannon)
  -- so no reboot is needed (a no-op in ship mode, where updateShip reads these
  -- every tick). upsideDown flips the trunnion step for a gun hung below its
  -- mount. The offset is the SHARED value used by static-gps AND ship mode --
  -- world axes (x east, y up, z south) static, hull-local (x right, y up,
  -- z forward) aboard a ship. Typed floats; whole blocks (the 0.5s are added).
  { group = "Position", label = "upsideDown", etype = "enum", file = "cfg", static = true,
    values = { false, true },
    get = function() return cfg.cannon.upsideDown end,
    set = function(v) cfg.cannon.upsideDown = v end },
  { group = "Position", label = "gps", etype = "enum", file = "cfg", static = true,
    values = { false, true },
    show = function() return not cfg.ship.enabled end,
    get = function() return cfg.cannon.gps end,
    set = function(v) cfg.cannon.gps = v end },
  { group = "Position", label = "mount x", etype = "float", file = "cfg", static = true,
    show = function() return not cfg.ship.enabled and not cfg.cannon.gps end,
    get = function() return cfg.cannon.x end,
    set = function(v) cfg.cannon.x = v end },
  { group = "Position", label = "mount y", etype = "float", file = "cfg", static = true,
    show = function() return not cfg.ship.enabled and not cfg.cannon.gps end,
    get = function() return cfg.cannon.y end,
    set = function(v) cfg.cannon.y = v end },
  { group = "Position", label = "mount z", etype = "float", file = "cfg", static = true,
    show = function() return not cfg.ship.enabled and not cfg.cannon.gps end,
    get = function() return cfg.cannon.z end,
    set = function(v) cfg.cannon.z = v end },
  -- The offset's axes mean different things by mode, so the labels follow:
  -- aboard a ship it's hull-local right/up/forward (left/back = negative);
  -- in static-gps mode it's world x/y/z (x east, y up, z south). Same keys.
  { group = "Position", static = true, etype = "float", file = "cfg",
    label = function() return cfg.ship.enabled and "offset right" or "offset x" end,
    show = function() return cfg.ship.enabled or cfg.cannon.gps end,
    get = function() return cfg.cannon.offset.x end,
    set = function(v) cfg.cannon.offset.x = v end },
  { group = "Position", static = true, etype = "float", file = "cfg",
    label = function() return cfg.ship.enabled and "offset up" or "offset y" end,
    show = function() return cfg.ship.enabled or cfg.cannon.gps end,
    get = function() return cfg.cannon.offset.y end,
    set = function(v) cfg.cannon.offset.y = v end },
  { group = "Position", static = true, etype = "float", file = "cfg",
    label = function() return cfg.ship.enabled and "offset fwd" or "offset z" end,
    show = function() return cfg.ship.enabled or cfg.cannon.gps end,
    get = function() return cfg.cannon.offset.z end,
    set = function(v) cfg.cannon.offset.z = v end },
  -- Airship mounting. `ship mode` toggles GPS+heading derivation live (shipDep
  -- re-resolves the nav table + gimbal; reverts if they're missing). The mount
  -- lever arm is the Position `offset` rows above (hull-local forward/up/right
  -- when ship mode is on). headingOffset + the gimbal map are read every tick,
  -- so those edits apply immediately; tune them against the DEBUG tab's heading
  -- / ship pitch / ship roll readouts.
  { group = "Ship", label = "ship mode", etype = "enum", file = "cfg", shipDep = true,
    values = { false, true },
    get = function() return cfg.ship.enabled end,
    set = function(v) cfg.ship.enabled = v end },
  { group = "Ship", label = "headingOffset", etype = "num", file = "cfg",
    min = -180, max = 180, step = 1,
    show = function() return cfg.ship.enabled end,
    get = function() return cfg.ship.headingOffset end,
    set = function(v) cfg.ship.headingOffset = v end },
  { group = "Ship", label = "navTable", etype = "text", file = "cfg", shipDep = true,
    show = function() return cfg.ship.enabled end,
    get = function() return cfg.ship.navTable end,
    set = function(v) cfg.ship.navTable = v end },
  { group = "Ship", label = "gimbal", etype = "text", file = "cfg", shipDep = true,
    show = function() return cfg.ship.enabled end,
    get = function() return cfg.ship.gimbal end,
    set = function(v) cfg.ship.gimbal = v end },
  { group = "Ship", label = "gimbal pitch ax", etype = "enum", file = "cfg",
    values = { "x", "z" },
    show = function() return cfg.ship.enabled and cfg.ship.gimbal ~= "none" end,
    get = function() return cfg.ship.gimbalMap.pitch end,
    set = function(v) cfg.ship.gimbalMap.pitch = v end },
  { group = "Ship", label = "gimbal roll ax", etype = "enum", file = "cfg",
    values = { "x", "z" },
    show = function() return cfg.ship.enabled and cfg.ship.gimbal ~= "none" end,
    get = function() return cfg.ship.gimbalMap.roll end,
    set = function(v) cfg.ship.gimbalMap.roll = v end },
  { group = "Ship", label = "invert pitch", etype = "enum", file = "cfg",
    values = { false, true },
    show = function() return cfg.ship.enabled and cfg.ship.gimbal ~= "none" end,
    get = function() return cfg.ship.gimbalMap.invertPitch end,
    set = function(v) cfg.ship.gimbalMap.invertPitch = v end },
  { group = "Ship", label = "invert roll", etype = "enum", file = "cfg",
    values = { false, true },
    show = function() return cfg.ship.enabled and cfg.ship.gimbal ~= "none" end,
    get = function() return cfg.ship.gimbalMap.invertRoll end,
    set = function(v) cfg.ship.gimbalMap.invertRoll = v end },
  { group = "Ship", label = "pitch rest", etype = "num", file = "cfg",
    min = -45, max = 45, step = 0.5,
    show = function() return cfg.ship.enabled and cfg.ship.gimbal ~= "none" end,
    get = function() return cfg.ship.gimbalMap.pitchRest end,
    set = function(v) cfg.ship.gimbalMap.pitchRest = v end },
  { group = "Ship", label = "roll rest", etype = "num", file = "cfg",
    min = -45, max = 45, step = 0.5,
    show = function() return cfg.ship.enabled and cfg.ship.gimbal ~= "none" end,
    get = function() return cfg.ship.gimbalMap.rollRest end,
    set = function(v) cfg.ship.gimbalMap.rollRest = v end },
  -- Arc travel limits (mount-frame degrees; read live by the solver and the
  -- slew clamps, so edits take effect immediately). Typed floats.
  { group = "Arc limits", label = "yaw min", etype = "float", file = "cfg",
    get = function() return cfg.limits.yaw.min end,
    set = function(v) cfg.limits.yaw.min = v end },
  { group = "Arc limits", label = "yaw max", etype = "float", file = "cfg",
    get = function() return cfg.limits.yaw.max end,
    set = function(v) cfg.limits.yaw.max = v end },
  { group = "Arc limits", label = "pitch min", etype = "float", file = "cfg",
    get = function() return cfg.limits.pitch.min end,
    set = function(v) cfg.limits.pitch.min = v end },
  { group = "Arc limits", label = "pitch max", etype = "float", file = "cfg",
    get = function() return cfg.limits.pitch.max end,
    set = function(v) cfg.limits.pitch.max = v end },
  -- Loop pacing target. The body's peripheral calls set the real floor
  -- (~0.15-0.2s), so values below ~0.1 buy little; 0.05 is CC's minimum
  -- yield. Exposed mainly because old saves carry the 0.25 default from
  -- before the pacing fix and there was no way to see or lower it.
  -- Roster/ship-idle cadences derive from this at BOOT (reboot after big
  -- changes so they re-derive).
  { group = "Drive", label = "trackSeconds", etype = "num", file = "cfg",
    min = 0.05, max = 0.5, step = 0.05,
    get = function() return cfg.trackSeconds end,
    set = function(v) cfg.trackSeconds = v end },
  { group = "Drive", label = "yaw gain", etype = "num", file = "cfg",
    min = 0.5, max = 40, step = 0.5,
    get = function() return cfg.yawDrive.speedGain end,
    set = function(v) cfg.yawDrive.speedGain = v end },
  { group = "Drive", label = "pitch gain", etype = "num", file = "cfg",
    min = 0.5, max = 40, step = 0.5,
    get = function() return cfg.pitchDrive.speedGain end,
    set = function(v) cfg.pitchDrive.speedGain = v end },
  { group = "Drive", label = "yaw approach", etype = "num", file = "cfg",
    min = 0.1, max = 10, step = 0.05,
    get = function() return cfg.yawDrive.approach end,
    set = function(v) cfg.yawDrive.approach = v end },
  { group = "Drive", label = "pitch approach", etype = "num", file = "cfg",
    min = 0.1, max = 10, step = 0.05,
    get = function() return cfg.pitchDrive.approach end,
    set = function(v) cfg.pitchDrive.approach = v end },
  { group = "Drive", label = "yaw kd", etype = "num", file = "cfg",
    min = 0, max = 5, step = 0.05,
    get = function() return cfg.yawDrive.kd end,
    set = function(v) cfg.yawDrive.kd = v end },
  { group = "Drive", label = "pitch kd", etype = "num", file = "cfg",
    min = 0, max = 5, step = 0.05,
    get = function() return cfg.pitchDrive.kd end,
    set = function(v) cfg.pitchDrive.kd = v end },
  { group = "Drive", label = "yaw maxRPM", etype = "num", file = "cfg",
    min = 5, max = 256, step = 5,
    get = function() return cfg.yawDrive.maxSpeed end,
    set = function(v) cfg.yawDrive.maxSpeed = v end },
  { group = "Drive", label = "pitch maxRPM", etype = "num", file = "cfg",
    min = 5, max = 256, step = 5,
    get = function() return cfg.pitchDrive.maxSpeed end,
    set = function(v) cfg.pitchDrive.maxSpeed = v end },
  -- cannon.cal: measured by CAL; editable here for a manual override.
  { group = "Calibrated", label = "yaw periph", etype = "text", file = "cal",
    get = function() return cfg.peripherals.yaw end,
    set = function(v) cfg.peripherals.yaw = v end },
  { group = "Calibrated", label = "pitch periph", etype = "text", file = "cal",
    get = function() return cfg.peripherals.pitch end,
    set = function(v) cfg.peripherals.pitch = v end },
  { group = "Calibrated", label = "invertYaw", etype = "enum", file = "cal",
    values = { "auto", false, true },
    get = function() return cfg.invertYaw end,
    set = function(v) cfg.invertYaw = v end },
  { group = "Calibrated", label = "invertPitch", etype = "enum", file = "cal",
    values = { "auto", false, true },
    get = function() return cfg.invertPitch end,
    set = function(v) cfg.invertPitch = v end },
  { group = "Calibrated", label = "yaw d/s/RPM", etype = "text", file = "cal",
    get = function() return cfg.yawDrive.degPerSecPerRpm end,
    set = function(v) cfg.yawDrive.degPerSecPerRpm = v end },
  { group = "Calibrated", label = "pitch d/s/RPM", etype = "text", file = "cal",
    get = function() return cfg.pitchDrive.degPerSecPerRpm end,
    set = function(v) cfg.pitchDrive.degPerSecPerRpm = v end },
  { group = "Calibrated", label = "yaw minRPM", etype = "num", file = "cal",
    min = 0.1, max = 10, step = 0.5,
    get = function() return cfg.yawDrive.minSpeed end,
    set = function(v) cfg.yawDrive.minSpeed = v end },
  { group = "Calibrated", label = "pitch minRPM", etype = "num", file = "cal",
    min = 0.1, max = 10, step = 0.5,
    get = function() return cfg.pitchDrive.minSpeed end,
    set = function(v) cfg.pitchDrive.minSpeed = v end },
  -- The home/rest facing, measured from the assembled rest yaw. A number
  -- pins it; type "auto" to re-measure on the next CAL / boot.
  { group = "Calibrated", label = "yawOffset", etype = "text", file = "cal",
    get = function() return cfg.yawOffset end,
    set = function(v) cfg.yawOffset = v end },
}

-- Heal enum values that were stored as STRINGS: the old setcfg text parse
-- saved "false"/"true" (truthy!) for boolean enums into cannon.cfg, so a
-- cloned turret could BEHAVE upside-down while displaying false. If the
-- saved value is a string naming one of the item's real (non-string)
-- values, swap the real one back in. Persists on the next normal save.
for _, it in ipairs(CONFIG_ITEMS) do
  if it.etype == "enum" and it.values then
    local cur = it.get()
    if type(cur) == "string" then
      for _, v in ipairs(it.values) do
        if type(v) ~= "string" and tostring(v) == cur then
          it.set(v)
          break
        end
      end
    end
  end
end

-- Items visible for the current kind (Build items gate on the gun type).
local function visibleConfigItems()
  local out = {}
  for _, it in ipairs(CONFIG_ITEMS) do
    if not it.show or it.show() then out[#out + 1] = it end
  end
  return out
end

local function cfgValueStr(it)
  local v = it.get()
  if type(v) == "number" then
    if v == math.floor(v) then return ("%d"):format(v) end
    -- Up to 3 decimals, trailing zeros trimmed; no sci notation (mount
    -- coords can be large, where %g would print 1.5e+03).
    return (("%.3f"):format(v):gsub("%.?0+$", ""))
  end
  return tostring(v)
end

-- A row's label, allowing a function so a label can adapt to the mode (e.g.
-- the mount offset reads as right/up/forward aboard a ship, x/y/z on land).
local function cfgLabel(it)
  return type(it.label) == "function" and it.label() or it.label
end

-- Re-resolve the gun after a profile edit; revert via `restore` and flash if
-- the new value won't render (steppers clamp, so this only bites a typo).
local function applyProfileEdit(restore)
  if pcall(refreshProfile) then return true end
  restore()
  pcall(refreshProfile)
  state.flash = "bad value"
  return false
end

-- Adjust a numeric item by dir*step (clamped) or cycle an enum by dir.
local function cfgAdjust(it, dir)
  local prev = it.get()
  if it.etype == "num" then
    local v = math.max(it.min, math.min(it.max, prev + dir * it.step))
    it.set(math.floor(v * 1000 + 0.5) / 1000) -- shave float dust from 0.1 steps
  elseif it.etype == "enum" then
    local i = 1
    for j, val in ipairs(it.values) do if val == prev then i = j; break end end
    it.set(it.values[((i - 1 + dir) % #it.values) + 1])
  else
    return
  end
  if it.profile then applyProfileEdit(function() it.set(prev) end) end
  if it.static then refreshStaticCannon() end
  if it.reloadDep then
    -- Bind/unbind the assembly relay for the new enabled state; a bad
    -- separate-relay name reverts the toggle rather than half-applying.
    local ok, err = pcall(refreshReloadRelay)
    if not ok then it.set(prev); state.flash = "reload relay: " .. tostring(err) end
  end
  if it.shipDep then
    -- Re-resolve nav table + gimbal for the new ship setting; on failure
    -- revert the edit AND re-refresh so the reverted state stays consistent
    -- (never leave updateShip pointed at a nil nav source).
    local ok, err = refreshShip()
    if not ok then it.set(prev); refreshShip(); state.flash = "ship: " .. tostring(err) end
  end
end

-- Parse a typed value: a "num"/"float" must be numeric (num clamps to
-- min/max, float is unbounded -- offsets, limit degrees); "auto" and other
-- keywords stay strings (peripheral names, degPerSecPerRpm="auto").
local function parseCfgValue(it, text)
  text = text:gsub("^%s+", ""):gsub("%s+$", "")
  if it.etype == "num" or it.etype == "float" then
    local n = tonumber(text)
    if not n then return nil, "need a number" end
    if it.etype == "float" then return n end
    return math.max(it.min, math.min(it.max, n))
  end
  if it.etype == "enum" and it.values then
    -- Match against the REAL value list and return the typed value. Enum
    -- values can be booleans (upsideDown, gps, ...) and the old raw-text
    -- fallthrough stored the STRING "false" from web setcfg/clone --
    -- which is truthy in Lua, so a turret behaved upside-down while its
    -- config printed false.
    for _, v in ipairs(it.values) do
      if tostring(v) == text then return v end
    end
    local names = {}
    for _, v in ipairs(it.values) do names[#names + 1] = tostring(v) end
    return nil, "must be one of: " .. table.concat(names, ", ")
  end
  return tonumber(text) or text
end

-- Snapshot every item's value, for dirty-detection and CANCEL.
local function cfgSnapshot()
  local snap = {}
  for i, it in ipairs(CONFIG_ITEMS) do snap[i] = it.get() end
  ui.cfgBaseline = snap
end

local function cfgDirty()
  if not ui.cfgBaseline then return false end
  for i, it in ipairs(CONFIG_ITEMS) do
    if it.get() ~= ui.cfgBaseline[i] then return true end
  end
  return false
end

local function cfgSave()
  writeCfg(cfg)
  writeCal(cfg)
  pcall(refreshProfile)
  cfgSnapshot()
  state.flash = "SAVED"
end

local function cfgCancel()
  if ui.cfgBaseline then
    for i, it in ipairs(CONFIG_ITEMS) do it.set(ui.cfgBaseline[i]) end
    pcall(refreshProfile)
    refreshStaticCannon()
  end
  state.flash = "REVERTED"
end

-- Spruce config mirror: the web UI renders THE SAME CONFIG_ITEMS table the
-- in-game tab is generated from, so the two can never drift. The status
-- payload always carries cfgRev (a cheap hash of the visible labels +
-- values -- in-game edits, remote setcfg, and kind-driven visibility flips
-- all change it); the full schema rides along only when the server's
-- echoed rev (ctl.cfgRev) disagrees, which also re-seeds a restarted
-- server. Validation stays turret-side: web edits arrive as plain setcfg.
local spruceServerCfgRev = nil

local function cfgSchemaRev()
  local h = 0
  for _, it in ipairs(visibleConfigItems()) do
    local s = cfgLabel(it) .. "=" .. tostring(it.get())
    for i = 1, #s do h = (h * 31 + s:byte(i)) % 2147483647 end
  end
  return h
end

local function buildCfgSchema()
  local items = {}
  for _, it in ipairs(visibleConfigItems()) do
    local values = nil
    if it.values then
      values = {} -- copy: serialiseJSON refuses repeated table references
      for i, v in ipairs(it.values) do values[i] = v end
    end
    items[#items + 1] = {
      group = it.group, label = cfgLabel(it), etype = it.etype,
      value = it.get(), min = it.min, max = it.max, step = it.step,
      values = values, file = it.file,
    }
  end
  return items
end

-- ---------------------------------------------------------------- drawing --

-- The CONFIG tab: grouped, scrollable, live-editable settings. The selected
-- row shows steppers ([-]/[+] for numbers, </> for enums) and an [=] text
-- entry; cal-file rows are orange. Edits apply live; SAVE/CANCEL persist or
-- revert (button bar). Mirrors drawDebugScreen's skip/clamp scroll model.
local function drawConfigScreen(w, h)
  local items = visibleConfigItems()
  if ui.cfgSel < 1 then ui.cfgSel = 1 end
  if ui.cfgSel > #items then ui.cfgSel = #items end

  -- Flatten into display rows (group headers + items) for uniform scrolling.
  local rows, lastGroup, selRow = {}, nil, 1
  for i, it in ipairs(items) do
    if it.group ~= lastGroup then
      rows[#rows + 1] = { header = it.group }
      lastGroup = it.group
    end
    rows[#rows + 1] = { item = it, idx = i }
    if i == ui.cfgSel then selRow = #rows end
  end

  local top, bot = 3, h - 1
  local vis = bot - top + 1
  -- Keep the selected row on screen, then clamp.
  if selRow - 1 < ui.scroll then ui.scroll = selRow - 1 end
  if selRow > ui.scroll + vis then ui.scroll = selRow - vis end
  local maxScroll = math.max(0, #rows - vis)
  ui.scroll = math.max(0, math.min(ui.scroll, maxScroll))

  local LBL = 15
  for r = 1, vis do
    local row = top + r - 1
    local entry = rows[r + ui.scroll]
    term.setCursorPos(1, row)
    term.setBackgroundColor(colors.black)
    term.clearLine()
    if entry and entry.header then
      term.setTextColor(colors.lightBlue)
      term.write("-- " .. entry.header)
    elseif entry then
      local it = entry.item
      local selected = entry.idx == ui.cfgSel
      term.setBackgroundColor(selected and colors.gray or colors.black)
      term.clearLine()
      term.setCursorPos(1, row)
      term.setTextColor(selected and colors.white or colors.lightGray)
      term.write(selected and ">" or " ")
      term.setTextColor(it.file == "cal" and colors.orange or colors.lightGray)
      term.write((" %-" .. (LBL - 2) .. "s"):format(cfgLabel(it):sub(1, LBL - 2)))
      local function btn(text, cmd, col)
        term.setCursorPos(col, row)
        term.setBackgroundColor(selected and colors.gray or colors.black)
        term.setTextColor(colors.yellow)
        term.write(text)
        ui.cells[#ui.cells + 1] = { col1 = col, col2 = col + #text - 1,
          row = row, cmd = cmd, idx = entry.idx }
        return col + #text + 1
      end
      if selected then
        local c = LBL + 1
        if it.etype == "num" then c = btn("[-]", "cfg_dec", c)
        elseif it.etype == "enum" then c = btn("<", "cfg_dec", c) end
        term.setCursorPos(c, row)
        term.setBackgroundColor(colors.gray)
        term.setTextColor(colors.white)
        local vstr = cfgValueStr(it):sub(1, w - c)
        term.write(vstr)
        c = c + #vstr + 1
        if it.etype == "num" then c = btn("[+]", "cfg_inc", c)
        elseif it.etype == "enum" then c = btn(">", "cfg_inc", c) end
        if it.etype ~= "enum" then btn("[=]", "cfg_edit", c) end
      else
        term.setTextColor(colors.white)
        term.write(cfgValueStr(it):sub(1, w - LBL))
        ui.cells[#ui.cells + 1] = { col1 = 1, col2 = w, row = row,
          cmd = "cfg_select", idx = entry.idx }
      end
    end
  end
end

local function drawTabBar(w)
  term.setCursorPos(1, 1)
  term.setBackgroundColor(colors.gray)
  term.write(string.rep(" ", w))
  local col = 1
  for _, tab in ipairs(ui.tabs) do
    local active = ui.activeTab == tab.id
    term.setCursorPos(col, 1)
    term.setTextColor(colors.black)
    term.setBackgroundColor(active and colors.white or colors.lightGray)
    term.write(tab.label)
    ui.cells[#ui.cells + 1] =
      { col1 = col, col2 = col + #tab.label - 1, row = 1, cmd = "tab_" .. tab.id }
    col = col + #tab.label + 1
  end
end

local function drawStatus()
  term.setCursorPos(1, 2)
  term.setBackgroundColor(colors.black)
  term.clearLine()
  if state.calibrating then
    term.setTextColor(colors.yellow)
    term.write("CALIBRATING + AUTO-TUNING -- ~1-2 min, barrel steps itself")
    return
  end
  if state.targetName then
    local isShip = state.targetKind == "ship"
    local isCoord = state.targetKind == "coord"
    term.setTextColor(colors.lightGray)
    term.write("Target ")
    term.setTextColor(isShip and colors.orange
      or (isCoord and colors.lightBlue or colors.cyan))
    term.write((isShip and "#" or (isCoord and "*" or "@"))
      .. state.targetName .. " ")
    if state.lost then
      term.setTextColor(colors.red)
      term.write("LOST")
    elseif state.noFix then
      term.setTextColor(colors.red)
      term.write("NO FIX")
    elseif state.hasArc == false then
      -- Ballistically unreachable: barrel tracks line-of-sight as a
      -- ready posture, fire stays gated until the target closes in.
      term.setTextColor(colors.red)
      term.write("NO ARC")
      if state.dist then
        term.setTextColor(colors.lightGray)
        term.write((" %dm"):format(state.dist))
      end
    elseif state.locked then
      term.setTextColor(colors.lime)
      term.write("LOCKED")
      if state.firing then
        term.setTextColor(colors.orange)
        term.write(" FIRING")
      elseif state.outOfRange then
        term.setTextColor(colors.orange)
        term.write(" OUT OF RANGE")
      end
    elseif state.outOfArc then
      term.setTextColor(colors.orange)
      term.write("OUT OF ARC")
    else
      term.setTextColor(colors.yellow)
      -- One decimal: whole-degree rounding hid "0.3 deg off, not locked"
      -- as a confusing "y+0 p+0".
      term.write(("y%+.1f p%+.1f"):format(state.yawErr, state.pitchErr))
      if state.targetKind == "ship" and state.miss then
        term.write((" miss %.0fm"):format(state.miss))
      elseif state.missH then
        term.write((" h%+.1f v%+.1f"):format(state.missH, state.missV))
      end
      if state.firing then
        -- Burst hysteresis holding the line through a momentary miss.
        term.setTextColor(colors.orange)
        term.write(" FIRING")
      elseif state.outOfRange then
        term.setTextColor(colors.orange)
        term.write(" RANGE") -- short: this line already carries the errors
      end
    end
  elseif cfg.ship.enabled then
    local c = cannonPos()
    if c then
      term.setTextColor(colors.lightGray)
      term.write(("Ship H%03d  cannon %d,%d,%d"):format(
        math.floor(ship.heading + 0.5) % 360,
        math.floor(c.x), math.floor(c.y), math.floor(c.z)))
    else
      term.setTextColor(colors.red)
      term.write("NO GPS/NAV FIX")
    end
  else
    term.setTextColor(colors.lightGray)
    term.write("No target -- click a player, ship, or XYZ")
  end
  if reloadActive() then
    local label, remain = reloadStatus()
    if label then
      term.setTextColor(colors.yellow)
      term.write(("  %s %.1fs"):format(label, remain))
    end
  end
  if state.armed and not state.firing then
    term.setTextColor(colors.red)
    term.write("  ARMED")
  end
  if trace.on then
    term.setTextColor(colors.red)
    term.write("  *REC")
  end
  if state.flash then
    term.setTextColor(colors.orange)
    term.write("  " .. state.flash)
  end
end

local function drawTargetsList(w, h)
  -- Row 3 is a standing button: open the XYZ-coordinate prompt. A fixed
  -- aim point (mainly for range/falloff testing); the active coord shows
  -- on the status line, and STOP clears it like any other target.
  term.setCursorPos(1, 3)
  term.setBackgroundColor(state.targetKind == "coord" and colors.gray
    or colors.black)
  term.clearLine()
  term.setTextColor(colors.lightBlue)
  term.write((" + set XYZ coord"):sub(1, w))
  ui.cells[#ui.cells + 1] = { col1 = 1, col2 = w, row = 3, cmd = "coordprompt" }

  local listTop, listBot = 4, h - 1
  local visRows = listBot - listTop + 1
  local maxScroll = math.max(0, #state.roster - visRows)
  if ui.scroll > maxScroll then ui.scroll = maxScroll end
  for i = 1, visRows do
    local row = listTop + i - 1
    local item = state.roster[i + ui.scroll]
    term.setCursorPos(1, row)
    term.setBackgroundColor(colors.black)
    term.clearLine()
    if item then
      local isShip = item.kind == "ship"
      local selected = item.name == state.targetName
        and item.kind == state.targetKind
      local friendly = whitelisted[item.name]
      term.setBackgroundColor(selected and colors.gray or colors.black)
      term.setTextColor(selected and colors.white or colors.lightGray)
      term.write(selected and ">" or " ")
      term.setTextColor(friendly and colors.gray
        or (isShip and colors.orange or colors.cyan))
      term.write(((isShip and "#" or "@") .. "%-16s"):format(item.name:sub(1, 16)))
      if item.dist then
        term.setTextColor(colors.white)
        term.write((" %dm"):format(item.dist))
      elseif not item.x then
        term.setTextColor(colors.gray)
        term.write(" other dim")
      end -- has coords but no cannon fix: distance unknown, leave blank
      ui.cells[#ui.cells + 1] = { col1 = 1, col2 = w, row = row,
        cmd = "select", kind = item.kind, name = item.name }
    end
  end
end

-- Convert our compass heading (0 = north/-Z, 90 = east/+X, clockwise --
-- CCMinimap convention) to the F3 debug screen's "Facing" line: MC yaw is
-- 0 = south/+Z, 90 = west/-X, range (-180, 180], i.e. ours shifted 180.
-- Verified against minecraft.wiki/w/Rotation.
local function f3Facing(heading)
  local mcYaw = heading - 180
  if mcYaw <= -180 then mcYaw = mcYaw + 360 end
  local names = {
    { "north", "-Z" }, { "east", "+X" }, { "south", "+Z" }, { "west", "-X" },
  }
  local n = names[math.floor(((heading + 45) % 360) / 90) + 1]
  return mcYaw, n[1], n[2]
end

-- Live numbers for dialing in cannon.offset / headingOffset / yawOffset:
-- everything the aim math sees, raw and derived.
local function drawDebugScreen(w, h)
  local row = 3
  local skip = ui.scroll -- mouse wheel hides the first N lines
  local total = 0        -- lines requested this draw, to clamp the scroll
  local function line(label, value, fg)
    total = total + 1
    if skip > 0 then
      skip = skip - 1
      return
    end
    if row > h - 1 then return end
    term.setCursorPos(1, row)
    term.setBackgroundColor(colors.black)
    term.clearLine()
    term.setTextColor(colors.lightGray)
    term.write(("%-14s"):format(label))
    term.setTextColor(fg or colors.white)
    term.write(tostring(value):sub(1, w - 14))
    row = row + 1
  end
  local function fmtPos(p)
    return p and ("%.1f  %.1f  %.1f"):format(p.x, p.y, p.z) or "?"
  end
  local function fmtDeg(v)
    return v and ("%.1f"):format(v) or "?"
  end
  if cfg.ship.enabled then
    local fresh = os.clock() <= ship.freshUntil
    line("ship fix", fresh and "OK" or "STALE",
      fresh and colors.lime or colors.red)
    line("needle rel", fmtDeg(ship.rel))
    line("ship heading", fmtDeg(ship.heading), colors.yellow)
    if ship.heading then
      -- Same direction in F3's language: stand facing ship-forward and
      -- this should match your "Facing" line exactly.
      local mcYaw, name, axis = f3Facing(ship.heading)
      line("  as F3", ("%s (%s)  %+.1f"):format(name, axis, mcYaw),
        colors.yellow)
    end
    -- Position cluster: computer (GPS) -> mount base (computer + offset) ->
    -- launch pivot (base + trunnion/centre). Walk these top-to-bottom to see
    -- exactly where each step lands.
    line("computer xyz", fmtPos(ship.pos))
    line("mount base xyz", fmtPos(ship.base))
    line("pivot xyz", fmtPos(ship.cannon), colors.yellow)
    line("offset r/u/f", ("%g / %g / %g  %s"):format(cfg.cannon.offset.x,
      cfg.cannon.offset.y, cfg.cannon.offset.z,
      cfg.cannon.upsideDown and "(inv)" or ""))
    line("headingOffset", cfg.ship.headingOffset)
    if cfg.ship.gimbal ~= "none" then
      if not gimbal then
        line("gimbal", "NOT FOUND (assuming level)", colors.red)
      else
        local g = ship.gimbal
        line("gimbal X/Z", g and ("%+.2f / %+.2f"):format(g.x, g.z) or "?",
          colors.orange)
        -- Mapped attitude: must read +up when the nose is up and +right
        -- when the right side is down, else flip gimbalMap inverts.
        line("ship pitch", ("%+.2f (+ = nose up)"):format(ship.pitch),
          colors.yellow)
        line("ship roll", ("%+.2f (+ = right down)"):format(ship.roll),
          colors.yellow)
      end
    end
  else
    line("mode", cfg.cannon.gps and "static (land), GPS fix"
      or "static (land)")
    -- Position cluster: computer (GPS fix, or "manual" when xyz is typed) ->
    -- mount base -> launch pivot (base + trunnion/centre).
    line("computer xyz", cfg.cannon.gps and fmtPos(gpsFix) or "manual xyz")
    line("mount base xyz", fmtPos(staticBase))
    line("pivot xyz", fmtPos(staticCannon), colors.yellow)
    if cfg.cannon.gps then
      line("offset xyz", ("%g / %g / %g  %s"):format(cfg.cannon.offset.x,
        cfg.cannon.offset.y, cfg.cannon.offset.z,
        cfg.cannon.upsideDown and "(inv)" or ""))
    end
  end
  line("yawOffset", cfg.yawOffset)
  line("profile", ("%s %s %.0f b/s"):format(cfg.profile.kind,
    cfg.profile.projectile, muzzleSpeed))
  -- Speed source: computed from material+barrels / charges, or overridden.
  local src
  if cfg.profile.kind == "bigcannon" then
    src = ("%d charges"):format(cfg.profile.charges)
  elseif type(cfg.profile.muzzleVelocityOverride) == "number"
    and cfg.profile.muzzleVelocityOverride > 0 then
    src = "override"
  else
    src = ("%s x%d barrels"):format(cfg.profile.material, cfg.profile.barrels)
  end
  line("speed from", src)
  line("drive y/p RPM", ("%s / %s d/s/RPM, floor %s/%s"):format(
    tostring(cfg.yawDrive.degPerSecPerRpm), tostring(cfg.pitchDrive.degPerSecPerRpm),
    tostring(cfg.yawDrive.minSpeed), tostring(cfg.pitchDrive.minSpeed)))
  local m = state.mount
  line("CannonYaw", fmtDeg(m and m.CannonYaw))
  line("CannonPitch", fmtDeg(m and m.CannonPitch))
  if transponderModem then
    local nShips = 0
    for _ in pairs(state.peerShips) do nShips = nShips + 1 end
    line("transponders", nShips)
  else
    line("transponders", "NO WIRELESS MODEM", colors.red)
  end
  if state.targetName then
    line("target", state.targetName, colors.cyan)
    -- Raw detector point vs the point actually solved for. A gap here is
    -- hitbox aimHeight + lead (players) or the below-transponder drop
    -- (ships) -- e.g. aim Y is the feet Y raised by aimHeight (centre mass).
    line("target xyz", fmtPos(state.targetRaw))
    line("aim xyz", fmtPos(state.aim), colors.cyan)
    line("yaw/pitch err",
      ("%+.1f / %+.1f"):format(state.yawErr, state.pitchErr))
    line("arc", state.outOfArc and "OUT (parked at limit)" or "in",
      state.outOfArc and colors.orange or colors.lime)
    line("dist / max", state.dist
      and ("%.0f / %d"):format(state.dist, cfg.maxDistance) or "?",
      state.outOfRange and colors.red or colors.lime)
    line("solution", state.hasArc == false and "NO ARC (LOS posture)"
      or (state.tof and ("%s arc, tof %.2fs"):format(
        cfg.profile.arc, state.tof) or "?"),
      state.hasArc == false and colors.red or nil)
    -- Predicted shot from the ACTUAL barrel angle (static mode): a spot to
    -- go watch, plus how high/low it crosses the target's range right now.
    if state.impact then
      if state.impact.x then
        line("impact xyz", ("%.0f %.0f %.0f"):format(
          state.impact.x, state.impact.y, state.impact.z), colors.yellow)
        line("impact off", ("%.1fm from target"):format(state.impact.off),
          state.impact.off < 2 and colors.lime or colors.orange)
      end
      if state.impact.vmiss then
        line("v.miss@range", ("%+.1f blocks"):format(state.impact.vmiss),
          math.abs(state.impact.vmiss) < 1 and colors.lime or colors.orange)
      end
    end
    if cfg.profile.kind == "bigcannon" then
      if cfg.reload.enabled then
        local label, remain = reloadStatus()
        line("reload", label and ("%s %.1fs"):format(label, remain) or "ready",
          label and colors.yellow or colors.lime)
      else
        local wait = pulse.nextAt - os.clock()
        line("reload", wait > 0 and ("%.1fs"):format(wait) or "ready",
          wait > 0 and colors.yellow or colors.lime)
      end
    end
    if cfg.burst.enabled then
      line("burst", state.bursting and "HOLDING (gate left box)"
        or (burst.open and "latched" or "idle"),
        state.bursting and colors.orange or nil)
    end
    line("aim rate y/p", ("%+.1f / %+.1f deg/s"):format(
      track.yawRate, track.pitchRate))
    -- Live control-loop period: the overshoot guard caps the approach speed
    -- against this. Smaller is better (faster, snappier); >0.15 is slow.
    line("loop period", ("%.0f ms (%.1f Hz)"):format(
      track.loopT * 1000, track.loopT > 0 and 1 / track.loopT or 0),
      track.loopT > 0.15 and colors.orange or colors.lime)
    -- Error-closing rate feeds the D term; watch this go to ~0 as the
    -- barrel settles. A big swing here right before lock is the overshoot
    -- kd damps -- raise yaw/pitch kd until it stops oscillating.
    if cfg.yawDrive.kd ~= 0 or cfg.pitchDrive.kd ~= 0 then
      line("err rate y/p", ("%+.1f / %+.1f deg/s (kd %g/%g)"):format(
        track.yawErrRate, track.pitchErrRate, cfg.yawDrive.kd, cfg.pitchDrive.kd))
    end
    if state.targetKind == "ship" then
      local area, avoid = shipArea(state.targetName)
      line("hull miss", state.miss
        and ("%.1f (fire %g..%g)"):format(state.miss, avoid, area) or "?",
        state.locked and colors.lime or colors.yellow)
    else
      local hb = cfg.playerHitbox
      line("aim miss h/v", state.missH
        and ("%+.2f / %+.2f (box w%g h%g @%g)"):format(
          state.missH, state.missV, hb.width, hb.height, hb.aimHeight)
        or "?", state.locked and colors.lime or colors.yellow)
      if cfg.lead.enabled and state.targetKind == "player" then
        line("lead", state.lead
          and ("%.1fm @ %.1fm/s (t %.2fs)"):format(
            state.lead.blocks, state.lead.speed, state.lead.tof)
          or "?", colors.orange)
      end
    end
  end
  while row <= h - 1 do -- clear leftovers from the targets list
    term.setCursorPos(1, row)
    term.setBackgroundColor(colors.black)
    term.clearLine()
    row = row + 1
  end
  -- Clamp for the next draw so the view can't scroll past the last line.
  local maxScroll = math.max(0, total - (h - 3))
  if ui.scroll > maxScroll then ui.scroll = maxScroll end
end

local function drawButtonBar(w, h)
  term.setCursorPos(1, h)
  term.setBackgroundColor(colors.black)
  term.clearLine()
  -- Modal line entry takes over the bar: shared draw() so the track loop's
  -- redraws keep the live text instead of fighting a read(). The label is
  -- "XYZ" for a coord target, or the field name for a CONFIG edit.
  if ui.prompt then
    term.setTextColor(colors.lightBlue)
    local label = ui.prompt.label or "XYZ"
    term.write(("%s: %s_"):format(label, ui.prompt.text):sub(1, w))
    if ui.prompt.err then
      term.setTextColor(colors.red)
      term.write((" [%s]"):format(ui.prompt.err))
    end
    return
  end
  local col = 1
  local function button(label, cmd, fg, enabled)
    term.setCursorPos(col, h)
    term.setBackgroundColor(enabled and colors.gray or colors.black)
    term.setTextColor(enabled and fg or colors.gray)
    term.write(label)
    if enabled then
      ui.cells[#ui.cells + 1] =
        { col1 = col, col2 = col + #label - 1, row = h, cmd = cmd }
    end
    col = col + #label + 1
  end
  -- The CONFIG tab swaps the fire controls for SAVE/CANCEL (only live when
  -- there are unsaved edits) plus CAL to (re)run the calibration wiggle.
  if ui.activeTab == "config" then
    local dirty = cfgDirty()
    button(" SAVE ", "cfg_save", colors.lime, dirty)
    button(" CANCEL ", "cfg_cancel", colors.red, dirty)
    button(" CAL ", "recal", colors.yellow, not state.calibrating)
    if cfg.profile.kind == "bigcannon" and cfg.reload.enabled then
      button(" YAW0 ", "offsetcal", colors.orange, not state.calibrating)
    end
    term.setCursorPos(w - 24, h)
    term.setBackgroundColor(colors.black)
    term.setTextColor(colors.gray)
    term.write(dirty and "unsaved -- SAVE/CANCEL" or "tap a row to edit")
    return
  end
  button(" FIRE ", "fire", state.locked and colors.lime or colors.red, true)
  button(state.armed and " DISARM " or " ARM ", "arm",
    state.armed and colors.red or colors.lime, true)
  button(" STOP ", "stop", colors.white, state.targetName ~= nil)
  button(" CAL ", "recal", colors.yellow, not state.calibrating)
  term.setCursorPos(w - 30, h)
  term.setBackgroundColor(colors.black)
  term.setTextColor(colors.gray)
  term.write("F=fire A=arm C=xyz K=cal+tune O=yaw0 L=trace Q=quit")
end

local function draw()
  local w, h = term.getSize()
  ui.cells = {}
  drawTabBar(w)
  drawStatus()
  if ui.activeTab == "debug" then
    drawDebugScreen(w, h)
  elseif ui.activeTab == "config" then
    drawConfigScreen(w, h)
  else
    drawTargetsList(w, h)
  end
  drawButtonBar(w, h)
end

-- ------------------------------------------------------------------ loops --

-- Continuous control loop: re-reads the target and the mount every tick so
-- the turret tracks moving players; refreshes the full roster about once
-- a second (one getPlayerPos call per online player).
local function trackLoop()
  local tick = 0
  -- Roster (~1s) and idle ship fix (~0.5s) cadences in loop ticks, so they
  -- hold steady as cfg.trackSeconds changes the tracking rate.
  local rosterTicks = math.max(1, math.floor(1 / cfg.trackSeconds + 0.5))
  local shipIdleTicks = math.max(1, math.floor(0.5 / cfg.trackSeconds + 0.5))
  while running do
    local loopStart = os.clock()
    -- Ship fix every tick while tracking (a climbing ship stair-steps the
    -- aim on anything slower), every 0.5s when idle; roster every 1s.
    if cfg.ship.enabled and (state.targetName or tick % shipIdleTicks == 0) then
      updateShip()
    end
    if tick % rosterTicks == 0 then
      refreshRoster()
      sentryAcquire()
    end
    -- Held fire line (autocannon stream): record every round that left
    -- since last tick, backdated to its true 300rpm cycle time.
    if state.firing then recordShotStream() end
    -- Pose stream: while tracking with a live websocket, nudge spruceLoop
    -- every track tick so the browser's barrel/lock state moves in near
    -- real time (the send side rate-limits, so this tops out ~4-6 Hz).
    if state.spruceWsUp and state.targetName then os.queueEvent("spruce_push") end
    tick = tick + 1
    -- Serviced here (not in the input loop) because calibration drives the
    -- motors and must not race the aim commands. pcall so a failed wiggle
    -- (axis didn't move) just flashes instead of killing the loop.
    -- CAL (K): the full automatic drive setup -- wiggle + loop measure +
    -- approach auto-tune (recalibrate). Owns the motors, so it's serviced here
    -- rather than racing the aim commands. ~1-2 min; the barrel steps itself.
    if state.recalRequest then
      state.recalRequest = false
      state.calibrating = true
      stopMotors()
      draw()
      local ok, err = pcall(recalibrate)
      state.calibrating = false
      if not ok then tuneLog("ERROR: " .. tostring(err)) end
      state.flash = ok and "CALIBRATED + AUTO-TUNED" or "CAL/TUNE FAILED (see log)"
      term.setBackgroundColor(colors.black)
      term.clear()
    end
    -- OFFSET CAL (O): disassemble/reassemble and read the rest yaw to set
    -- yawOffset. Owns the motors and the assembly relay, so serviced here.
    if state.offsetCalRequest then
      state.offsetCalRequest = false
      if not cfg.ship.enabled then
        -- Static: nothing to measure -- CBC's CannonYaw is world-absolute
        -- and the MC->atan2 frame conversion is a constant (see calibrate).
        -- The CURRENT barrel pose becomes home: aim it where it should
        -- park, press O.
        cfg.yawOffset = 90
        local m = state.mount
        local captured = m and type(m.CannonYaw) == "number"
          and type(m.CannonPitch) == "number"
        if captured then
          cfg.homeYaw = angleDiff(m.CannonYaw + cfg.yawOffset, 0)
          cfg.homePitch = math.floor(m.CannonPitch * 10 + 0.5) / 10
        end
        writeCfg(cfg) -- homeYaw/homePitch are cfg keys
        writeCal(cfg)
        state.flash = captured
          and ("yawOffset 90 -- home = az %.0f, pitch %.0f"):format(
            cfg.homeYaw, cfg.homePitch)
          or "yawOffset = 90 (no mount reading; home unchanged)"
      else
        state.calibrating = true
        stopMotors()
        draw()
        local ok, rest, off = pcall(calibrateYawOffset)
        state.calibrating = false
        if ok then
          -- The disassemble cycle snapped the gun to its deck rest, so
          -- the captured pose IS the rest: home = deck zero.
          cfg.homeYaw, cfg.homePitch = 0, 0
          writeCfg(cfg)
          state.flash = ("yawOffset = %.0f (deck rest yaw %.0f)"):format(off, rest)
        else
          tuneLog("ERROR: " .. tostring(rest))
          state.flash = "OFFSET CAL FAILED (see log)"
        end
        term.setBackgroundColor(colors.black)
        term.clear()
      end
    end
    -- Advance the physical reload cycle every frame, target or not, so a
    -- shot fired just before the target dropped still rebuilds the gun.
    if cfg.reload.enabled and cfg.profile.kind == "bigcannon" then
      tickReload(os.clock())
    end
    if state.targetName then
      local pos = targetPos()
      if pos then
        state.lost = false
        -- Ship targets: aim below the transponder, never at it (the
        -- broadcast position is the block keeping the target on the air).
        -- Players: the reported point is the FEET, so raise the aim by
        -- aimHeight (centre of mass) and led to the intercept when on.
        local area, avoid
        local ax, ay, az = pos.x, pos.y, pos.z
        if state.targetKind == "ship" then
          area, avoid = shipArea(state.targetName)
          ay = ay - avoid * 1.5
        elseif state.targetKind == "player" then
          if cfg.lead.enabled then
            updateLead(pos)
            local p, tof = leadPoint(pos)
            local speed = math.sqrt(track.vx * track.vx
              + track.vy * track.vy + track.vz * track.vz)
            state.lead = { speed = speed, blocks = speed * tof, tof = tof }
            ax, ay, az = p.x, p.y, p.z
          end
          ay = ay + cfg.playerHitbox.aimHeight
        end -- coord: aim at the exact point, no lead / no hitbox offset
        -- DEBUG: the raw detector/transponder point and the final solved
        -- aim point (after ship-avoid / lead / hitbox aimHeight).
        state.targetRaw = { x = pos.x, y = pos.y, z = pos.z }
        state.aim = { x = ax, y = ay, z = az }
        local relYaw, relPitch, dist, tof, hasArc = anglesFor(ax, ay, az)
        if not relYaw then
          -- Stale ship fix: hold rather than aim with old coords/heading.
          state.noFix = true
          state.locked = false
          state.bursting = false
          resetBurst()
          stopMotors()
        else
          state.noFix = false
          state.dist = dist
          state.tof = tof
          state.hasArc = hasArc
          state.outOfRange = dist > cfg.maxDistance
          local data = blockReader.getBlockData()
          state.mount = data
          if data and data.CannonYaw and data.CannonPitch then
            -- Diagnostic (static mode): forward-fly a shot from the
            -- barrel's ACTUAL current angle -- NOT the solver's answer --
            -- so the debug tab can show where this shell is really headed
            -- (a coord to go watch) and how high/low it crosses the
            -- target's range. Inverts anglesFor's static convention:
            -- worldYaw = CannonYaw + yawOffset, worldPitch = CannonPitch
            -- - pitchOffset. Shares the flight model with the solver, so
            -- it catches an unsettled barrel / wrong angle convention,
            -- NOT a wrong muzzle speed (compare the coord to where the
            -- shell is actually seen to land for that).
            if not cfg.ship.enabled then
              local cc = cannonPos()
              if cc then
                local wy = math.rad(data.CannonYaw + cfg.yawOffset)
                local wp = data.CannonPitch - cfg.pitchOffset
                local tdx, tdz = pos.x - cc.x, pos.z - cc.z
                local tdy = pos.y - cc.y
                local imp = Ballistics.impact{ v0 = muzzleSpeed,
                  gravity = proj.gravity, drag = proj.drag,
                  muzzle = muzzleLen, pitch = wp,
                  dx = math.sqrt(tdx * tdx + tdz * tdz), dy = tdy }
                local rec = { vmiss = imp.hAtTarget
                  and (imp.hAtTarget - tdy) or nil }
                if imp.range then
                  rec.x = cc.x + imp.range * math.cos(wy)
                  rec.y = pos.y
                  rec.z = cc.z + imp.range * math.sin(wy)
                  local ox, oz = rec.x - pos.x, rec.z - pos.z
                  rec.off = math.sqrt(ox * ox + oz * oz)
                  rec.tof = imp.tof
                end
                state.impact = rec
              end
            end
            -- Travel limits: drive toward the solution clamped into the
            -- arc (parks at the edge while the target is outside, ready
            -- for re-entry). Errors are plain unwrapped differences, NOT
            -- shortest-path: with the forbidden zone behind the arc, the
            -- short way through it is exactly the slew that must never
            -- happen.
            -- Yaw angles are re-centered on the ARC's midpoint before
            -- clamping (not on 0): with an off-center arc like 0..180, a
            -- target just past the max edge would otherwise normalize to
            -- ~-180 and clamp to the WRONG edge, swinging the barrel all
            -- the way across the arc.
            local lim = cfg.limits
            -- A full-circle arc (span ~360) is a continuous slew ring with no
            -- forbidden zone: skip the clamp and drive the SHORTEST path across
            -- the +/-180 seam (angleDiff error) instead of the unwrapped long
            -- way around. Bounded arcs keep the unwrapped error below.
            local yawFull = (lim.yaw.max - lim.yaw.min) >= 359
            local cy, tgtYaw, aimYaw
            if yawFull then
              cy, tgtYaw, aimYaw = data.CannonYaw, relYaw, relYaw
            else
              local yawMid = (lim.yaw.min + lim.yaw.max) / 2
              cy = yawMid + angleDiff(data.CannonYaw, yawMid)
              tgtYaw = yawMid + angleDiff(relYaw, yawMid)
              aimYaw = math.max(lim.yaw.min, math.min(tgtYaw, lim.yaw.max))
            end
            local aimPitch = math.max(lim.pitch.min,
              math.min(relPitch, lim.pitch.max))
            state.outOfArc = aimYaw ~= tgtYaw or aimPitch ~= relPitch
            state.yawErr = yawFull and angleDiff(aimYaw, cy) or (aimYaw - cy)
            state.pitchErr = aimPitch - data.CannonPitch
            local withinWide -- widened gate, feeds the burst hysteresis
            local wd = cfg.burst.widen
            if area then
              -- Hull gate replaces the per-axis tolerance lock: fire as
              -- soon as the shot would land on the hull ring, while the
              -- motors keep converging on the below-transponder aim point.
              state.locked, state.miss = hullGate(pos, data, area, avoid)
              -- Widened ring grows outward only: avoidRadius still
              -- protects the transponder during a burst hold.
              withinWide = state.miss ~= nil
                and state.miss <= area * wd and state.miss >= avoid
            elseif state.targetKind == "coord" then
              -- Fixed point: no hitbox, just the per-axis tolerance lock.
              -- missH/missV are kept for the debug tab's aim-miss line.
              state.missH, state.missV = missComponents(
                angleDiff(relYaw, data.CannonYaw),
                relPitch - data.CannonPitch, data.CannonPitch, dist)
              state.locked = not state.outOfArc
                and axisSettled(state.yawErr, cfg.yawDrive, track.loopT)
                and axisSettled(state.pitchErr, cfg.pitchDrive, track.loopT)
              withinWide = not state.outOfArc
                and math.abs(state.yawErr) < cfg.tolerance * wd
                and math.abs(state.pitchErr) < cfg.tolerance * wd
            else
              -- Hitbox gate: fire while the shot would pass through the
              -- body box rising from the reported feet, OR once both axes
              -- settle inside tolerance (long range, where the box subtends
              -- less than the deadband). Gate errors are vs the TRUE aim
              -- solution, not the clamped one -- parked at an arc edge, fire
              -- only if shots from there would still hit; the tolerance lock
              -- is meaningless at a clamped setpoint. missV is measured from
              -- the AIM point (feet + aimHeight), so the box spans the feet
              -- (-aimHeight) up to the head (height - aimHeight) around it.
              local hb = cfg.playerHitbox
              state.missH, state.missV = missComponents(
                angleDiff(relYaw, data.CannonYaw),
                relPitch - data.CannonPitch, data.CannonPitch, dist)
              local vLo, vHi = -hb.aimHeight, hb.height - hb.aimHeight
              state.locked = (math.abs(state.missH) <= hb.width / 2
                  and state.missV >= vLo and state.missV <= vHi)
                or (not state.outOfArc
                  and axisSettled(state.yawErr, cfg.yawDrive, track.loopT)
                  and axisSettled(state.pitchErr, cfg.pitchDrive, track.loopT))
              withinWide = (math.abs(state.missH) <= hb.width / 2 * wd
                  and state.missV >= vLo * wd and state.missV <= vHi * wd)
                or (not state.outOfArc
                  and math.abs(state.yawErr) < cfg.tolerance * wd
                  and math.abs(state.pitchErr) < cfg.tolerance * wd)
            end
            state.bursting = burstGate(state.locked, withinWide)
              and not state.locked
            updateRates(aimYaw, aimPitch, state.yawErr, state.pitchErr, yawFull)
            -- Live control-loop period -- the overshoot guard caps the
            -- approach speed against it, so it tracks the real (peripheral-
            -- limited) rate instead of assuming cfg.trackSeconds.
            local nowT = os.clock()
            if track.driveT then
              local dt = nowT - track.driveT
              if dt > 0 and dt < 1 then
                track.loopT = track.loopT + 0.3 * (dt - track.loopT)
              end
            end
            track.driveT = nowT
            local yawRpm = speedFor(state.yawErr, cfg.invertYaw,
              cfg.yawDrive, track.yawRate, track.yawErrRate, track.loopT)
            local pitchRpm = speedFor(state.pitchErr, cfg.invertPitch,
              cfg.pitchDrive, track.pitchRate, track.pitchErrRate, track.loopT)
            -- While a reload cycle runs the motors belong to tickReload /
            -- tickPark (below). Driving the aim here would fight them -- the
            -- barrel runs toward the target during tickPark's block-reader
            -- yield before the park command corrects it, so it never settles
            -- on 0,0. The aim math above still feeds the status/lock display.
            if not reloadActive() then
              yaw.setTargetSpeed(yawRpm)
              pitch.setTargetSpeed(pitchRpm)
            else
              yawRpm, pitchRpm = 0, 0
            end
            traceRow(aimYaw, data.CannonYaw, yawRpm,
              aimPitch, data.CannonPitch, pitchRpm, reloadSeq.phase)
          else
            -- No mount reading: don't keep reporting (or firing on) a lock
            -- computed from stale angles.
            state.locked = false
            state.bursting = false
            resetBurst()
          end
        end
      else
        state.lost = true
        state.locked = false
        state.bursting = false
        resetBurst()
        -- Nothing to track: return to the neutral rest pose, unless a reload
        -- cycle owns the motors (handled in the auto-fire block below).
        if reloadActive() then stopMotors() else driveToRest() end
      end
      -- Auto-fire actuation. The gate additionally requires a real
      -- ballistic solution -- NO ARC means the barrel is only posing --
      -- and a shell path clear of every friendly-fire column.
      state.zoneBlocked = state.targetName ~= nil and zonePathBlocked()
      local gate = state.armed and (state.locked or state.bursting)
        and not state.outOfRange and state.hasArc == true
        and not state.zoneBlocked
      if cfg.profile.kind == "bigcannon" then
        if cfg.reload.enabled then
          -- Physical reload: fire only from a fully assembled gun; while
          -- the cycle runs the contraption is gone, so hold the drive
          -- (tickReload at the loop top advances the relay sequence).
          -- tickPark holds at a stop unless the optional pre-reload park is
          -- mid-slew, in which case it drives the barrel home instead.
          if reloadSeq.phase == "ready" then
            if gate then startShot(os.clock()) end
          else
            tickPark(os.clock())
          end
        else
          -- Autoloader path: one firePulseSeconds pulse per shot, next
          -- shot no sooner than reloadSeconds after the trigger.
          local now = os.clock()
          if state.firing and now >= pulse.offAt then setFiring(false) end
          if gate and not state.firing and now >= pulse.nextAt then
            setFiring(true)
            pulse.offAt = now + cfg.firePulseSeconds
            pulse.nextAt = now + math.max(cfg.profile.reloadSeconds,
              cfg.firePulseSeconds + 0.1)
          end
        end
      else
        setFiring(gate)
      end
      draw()
      -- Sleep only the remainder of the target period: the peripheral reads
      -- and drive writes already burn game-time (they yield), so adding a
      -- full trackSeconds on top is what stretched the real loop to ~0.25s.
      -- Always yields at least one tick.
      sleep(math.max(0.05, cfg.trackSeconds - (os.clock() - loopStart)))
    else
      -- No target. A reload cycle (incl. a pre-reload park slewing home)
      -- owns the motors; otherwise return the barrel to its neutral rest
      -- pose instead of freezing wherever it last aimed.
      local homing = false
      if reloadActive() then
        tickPark(os.clock())
      else
        homing = not driveToRest()
      end
      -- Don't cut a fire pulse short or fight an in-flight reload cycle.
      if not reloadActive() then setFiring(false) end
      -- Keep the debug tab's mount line live.
      if ui.activeTab == "debug" then
        state.mount = blockReader.getBlockData()
      end
      draw()
      -- Tick fast while a reload runs OR while slewing home so deadlines and
      -- the rest-drive control loop stay crisp; otherwise idle slowly.
      sleep((reloadActive() or homing) and cfg.trackSeconds or 0.5)
    end
    state.flash = nil
  end
end

-- Open the modal line editor on a CONFIG field (idx into the visible list).
local function openCfgEdit(idx)
  local it = visibleConfigItems()[idx]
  if not it then return end
  ui.prompt = { kind = "cfg", idx = idx, text = cfgValueStr(it), label = cfgLabel(it) }
end

local function handleCommand(cell)
  if cell.cmd == "select" then
    setTarget(cell.kind, cell.name)
  elseif cell.cmd == "fire" then
    fire()
  elseif cell.cmd == "arm" then
    toggleArm()
  elseif cell.cmd == "coordprompt" then
    ui.prompt = { kind = "coord", text = "" }
  elseif cell.cmd == "stop" then
    setTarget(state.targetKind, state.targetName) -- toggle off
  elseif cell.cmd == "recal" then
    state.recalRequest = true
  elseif cell.cmd == "offsetcal" then
    state.offsetCalRequest = true
  elseif cell.cmd == "cfg_select" then
    ui.cfgSel = cell.idx
  elseif cell.cmd == "cfg_inc" then
    ui.cfgSel = cell.idx
    local it = visibleConfigItems()[cell.idx]
    if it then cfgAdjust(it, 1) end
  elseif cell.cmd == "cfg_dec" then
    ui.cfgSel = cell.idx
    local it = visibleConfigItems()[cell.idx]
    if it then cfgAdjust(it, -1) end
  elseif cell.cmd == "cfg_edit" then
    ui.cfgSel = cell.idx
    openCfgEdit(cell.idx)
  elseif cell.cmd == "cfg_save" then
    cfgSave()
  elseif cell.cmd == "cfg_cancel" then
    cfgCancel()
  elseif cell.cmd:sub(1, 4) == "tab_" then
    ui.activeTab = cell.cmd:sub(5)
    ui.scroll = 0 -- the tabs share the scroll offset; start each at the top
    if ui.activeTab == "config" then ui.cfgSel = 1; cfgSnapshot() end
  end
end

-- Transponder listener: park on the rednet protocol and ingest peer ship
-- broadcasts as they arrive. Eviction happens in refreshRoster (1s cadence)
-- and per-tick freshness in targetPos, so this loop only ever adds.
local function rednetLoop()
  if not transponderModem then
    while running do sleep(1) end
    return
  end
  while running do
    -- Listen for any protocol, then accept CCMinimap ship state OR our own
    -- private beacons; both feed the same peer-ship roster.
    local _, msg, proto = rednet.receive(nil, 1.0)
    if msg and (proto == STATE_PROTOCOL or proto == BEACON_PROTOCOL) then
      handlePeerState(msg)
    end
  end
end

-- ---- Spruce C2 reporting (SPRUCE_PLAN.md phase 1: visibility) -------------
-- POST a status snapshot to the Spruce server every ~1s so this turret shows
-- up on the operator map (position, facing, arc, roster, target). Read-only
-- with respect to the gun: it only reads state the trackLoop already
-- maintains and must never drive the motors. Comms failures are swallowed
-- (one flash when the link first drops) -- a dead server must not take the
-- local UI or tracking down with it.

-- The recent-fire-events ring (spruceShots) is declared next to the fire
-- path -- appends happen at the relay edges (fire/setFiring/trackLoop),
-- which all run before this section; only the 5s prune lives here, in
-- buildSpruceStatus.

-- cfg.callsign (the setcfg-editable Identity item) outranks the turret.cfg
-- callsign: the installer seeds turret.cfg from the server-side registry
-- name, and a later rename (setcfg) must win over that seed.
local function spruceCallsign()
  return (cfg.callsign ~= "" and cfg.callsign)
    or (spruceCfg and spruceCfg.callsign)
    or os.getComputerLabel()
    or ("turret-" .. os.getComputerID())
end

-- serialiseJSON turns {} into an object; the API contract wants [].
local function jsonList(t)
  if next(t) == nil then return textutils.empty_json_array end
  return t
end

local function buildSpruceStatus()
  local now = os.clock()
  for i = #spruceShots, 1, -1 do
    if now - spruceShots[i].clock > 5 then table.remove(spruceShots, i) end
  end
  local mount = state.mount
  local payload = {
    callsign = spruceCallsign(),
    ts = os.epoch("utc"),
    pos = cannonPos(), -- the LAUNCH PIVOT (pivotFromBase), not the mount base
    -- worldYaw/worldPitch are the barrel's TRUE world facing via
    -- mountWorldFacing (full ship-basis unwind on a deck, plain offset
    -- folds when static) -- the browser prefers them for the barrel and
    -- live arc. yaw/pitch stay as the flat-deck values for older
    -- consumers and the popup. upsideDown lets the renderer hang the
    -- base block on the correct side of the pivot.
    mount = (function()
      if not (mount and mount.CannonYaw and mount.CannonPitch) then return nil end
      local m = { yaw = mount.CannonYaw,
                  pitch = mount.CannonPitch - cfg.pitchOffset,
                  upsideDown = cfg.cannon.upsideDown or false }
      local wy, wp = mountWorldFacing(mount)
      if wy then m.worldYaw, m.worldPitch = wy, wp end
      return m
    end)(),
    -- World facing of the mount's zero; "auto" until first calibration,
    -- and the browser can't draw the arc wedge without a number. In ship
    -- mode mount.yaw is DECK-relative, and on a flat deck the barrel's
    -- world azimuth is shipHeading + CannonYaw + yawOffset (atan2(dz,dx)
    -- frame) -- fold the live heading in here so the browser needs no
    -- ship awareness and the arc wedge swings with the deck. Deck roll/
    -- pitch are not unwound (the few degrees at hover shave accuracy of
    -- the DISPLAY only; the solver handles them for real aiming).
    rest = (function()
      if type(cfg.yawOffset) ~= "number" then return nil end
      local off = cfg.yawOffset
      if cfg.ship.enabled then
        if type(ship.heading) ~= "number" then return nil end -- no fix yet
        off = off + ship.heading
      end
      return { yawOffset = off }
    end)(),
    limits = cfg.limits,
    status = {
      armed = state.armed, locked = state.locked, lost = state.lost,
      noFix = state.noFix, outOfArc = state.outOfArc,
      outOfRange = state.outOfRange, firing = state.firing,
      hasArc = state.hasArc, calibrating = state.calibrating,
      reloadPhase = reloadSeq.phase,
      shipMode = cfg.ship.enabled,
      zoneBlocked = state.zoneBlocked,
    },
    gun = {
      kind = cfg.profile.kind, projectile = cfg.profile.projectile,
      arc = cfg.profile.arc, muzzleSpeed = muzzleSpeed,
      maxDistance = cfg.maxDistance,
      -- Physics constants + muzzle offset so the browser can integrate
      -- the same trajectory the solver uses (3D arc preview / phase 4).
      gravity = proj.gravity, drag = proj.drag, muzzleLen = muzzleLen,
    },
    roster = jsonList(state.roster),
    shots = jsonList(spruceShots),
    cfgRev = cfgSchemaRev(),
  }
  -- Full config schema only when the server's copy is stale (first
  -- contact, server restart, or any local/remote edit) -- it's a few KB
  -- and the normal tick stays light without it.
  if payload.cfgRev ~= spruceServerCfgRev then
    payload.config = buildCfgSchema()
  end
  if state.targetKind then
    payload.target = {
      kind = state.targetKind, name = state.targetName,
      raw = state.targetRaw, aim = state.aim,
      dist = state.dist, tof = state.tof,
      missH = state.missH, missV = state.missV,
      lead = state.lead,
    }
  end
  return payload
end

-- Phase 2: remote commands. The outbox drains each tick and dispatches
-- through the same internals the local UI uses. Drive-touching work
-- (calibrate) only sets the state.*Request flags trackLoop services --
-- spruceLoop must never own the motors. fire() pulses the relay directly,
-- exactly like the local F key does from inputLoop.

local function findCfgItem(label)
  local hit = nil
  for _, it in ipairs(CONFIG_ITEMS) do
    if cfgLabel(it) == label then
      if hit then return nil, "ambiguous config label: " .. label end
      hit = it
    end
  end
  if not hit then return nil, "no config item labeled " .. tostring(label) end
  return hit
end

-- Remote config edit: same parse/validate/side-effect path as the local
-- CONFIG tab (commitPrompt/cfgAdjust), then an immediate save -- there is
-- no remote CANCEL, so an applied edit must not sit unsaved.
local function applySetcfg(params)
  local it, err = findCfgItem(tostring(params.label or ""))
  if not it then return false, err end
  local v, why = parseCfgValue(it, tostring(params.value))
  if v == nil then return false, why or "bad value" end
  local prev = it.get()
  it.set(v)
  if it.profile and not applyProfileEdit(function() it.set(prev) end) then
    return false, "value rejected by profile"
  end
  if it.static then refreshStaticCannon() end
  if it.reloadDep then
    local ok, e = pcall(refreshReloadRelay)
    if not ok then it.set(prev); return false, "reload relay: " .. tostring(e) end
  end
  if it.shipDep then
    local ok, e = refreshShip()
    if not ok then it.set(prev); refreshShip(); return false, "ship: " .. tostring(e) end
  end
  cfgSave()
  return true
end

local SPRUCE_COMMANDS = {
  arm = function()
    if not state.armed then toggleArm() end
    return true
  end,
  disarm = function()
    if state.armed then toggleArm() end
    return true
  end,
  stop = function()
    setTarget(nil)
    return true
  end,
  fire = function()
    fire()
    return true
  end,
  calibrate = function()
    state.recalRequest = true
    return true
  end,
  reboot = function()
    -- Deferred until after the ack POST (spruceLoop) so the operator log
    -- shows the command landed; the self-updating startup then pulls the
    -- current turret file set on the way back up.
    state.rebootRequest = true
    return true
  end,
  target = function(params)
    if params.kind == "coord" then
      local x, y, z = tonumber(params.x), tonumber(params.y), tonumber(params.z)
      if not (x and y and z) then return false, "coord target needs numeric x,y,z" end
      setCoordTarget({ x = x, y = y, z = z })
      return true
    elseif params.kind == "player" or params.kind == "ship" then
      local name = tostring(params.name or "")
      if name == "" then return false, "target needs a name" end
      -- setTarget toggles off when re-selecting the current target (local
      -- click-again-to-release); a remote "target" means SET, so no-op
      -- instead of toggling when it's already the target.
      if not (state.targetKind == params.kind and state.targetName == name) then
        setTarget(params.kind, name)
      end
      return true
    end
    return false, "unknown target kind: " .. tostring(params.kind)
  end,
  setcfg = applySetcfg,
}

local function dispatchSpruceCmd(item)
  local fn = SPRUCE_COMMANDS[item.cmd]
  if not fn then return false, "unknown cmd: " .. tostring(item.cmd) end
  local ok, res, why = pcall(fn, item.params or {})
  if not ok then return false, tostring(res) end
  if res ~= true then return false, why or "rejected" end
  return true
end

local function spruceLoop()
  if not spruceCfg then
    while running do sleep(1) end
    return
  end
  local headers = {
    ["Content-Type"] = "application/json",
    ["Authorization"] = "Bearer " .. spruceCfg.token,
  }
  local urlStatus = spruceCfg.url .. "/api/drone/turret/status"
  local urlAck = spruceCfg.url .. "/api/drone/turret/ack"
  local wsUrl = spruceCfg.url:gsub("^http", "ws") .. "/api/drone/turret/ws"

  -- POST returning the decoded JSON table, or nil on any transport/HTTP
  -- failure. http.post only hands back a handle on 2xx; non-2xx arrives
  -- as the 4th value and both need closing or CC leaks the connection.
  -- The http call yields while in flight, so trackLoop keeps running.
  local function rpc(url, body)
    local ok, res, _, errRes = pcall(http.post, url, body, headers)
    if not ok or not res then
      if ok and errRes then pcall(errRes.close) end
      return nil
    end
    local raw = res.readAll()
    pcall(res.close)
    local okJson, parsed = pcall(textutils.unserialiseJSON, raw or "")
    if okJson and type(parsed) == "table" then return parsed end
    return {}
  end

  -- Shared by HTTP responses and WS "reply" frames: consume ctl (sentry
  -- stance + friendlies + the server's config rev), dispatch any commands,
  -- and return the acks list (nil when there were none).
  local function handleReply(reply)
    local ctl = reply.ctl
    if type(ctl) == "table" then
      state.spruceSentry = ctl.sentry == true
      local fr = {}
      if type(ctl.friendlies) == "table" then
        for _, n in ipairs(ctl.friendlies) do fr[n] = true end
      end
      state.spruceFriendlies = fr
      state.spruceZones = type(ctl.zones) == "table" and ctl.zones or {}
      spruceServerCfgRev = tonumber(ctl.cfgRev)
    end
    local items = reply.items
    if type(items) ~= "table" or #items == 0 then return nil end
    -- Every drained item gets an ack (ok or err) so the operator log
    -- shows what actually happened.
    local acks = {}
    for _, item in ipairs(items) do
      local okCmd, err = dispatchSpruceCmd(item)
      acks[#acks + 1] = { id = item.id, cmd = item.cmd, ok = okCmd, err = err }
    end
    return acks
  end

  -- One status -> reply -> ack cycle over plain HTTP: the fallback
  -- transport (and the original phase-1/2 path), with its own pacing.
  -- Build + serialise under pcall: a telemetry bug must flash loudly on
  -- the turret screen, NOT crash the parallel stack and take the gun down.
  local down = false
  local function httpTick()
    local okBody, body = pcall(function()
      return textutils.serialiseJSON(buildSpruceStatus())
    end)
    local reply = okBody and rpc(urlStatus, body)
    if not okBody then
      -- Not a link problem -- OUR payload is broken. Flash the real error
      -- (don't fall into the LINK DOWN branch, which would mask it).
      state.flash = "SPRUCE STATUS ERR: " .. tostring(body)
      sleep(spruceCfg.statusSeconds)
    elseif reply then
      local acks = handleReply(reply)
      if acks then
        rpc(urlAck, textutils.serialiseJSON({ acks = acks }))
        if state.rebootRequest then os.reboot() end
      end
      if down then down = false; state.flash = "SPRUCE LINK UP" end
      sleep(spruceCfg.statusSeconds)
    else
      if not down then down = true; state.flash = "SPRUCE LINK DOWN" end
      sleep(5)
    end
  end

  -- WebSocket session: the server pushes commands the instant the operator
  -- clicks, and spruce_push events (shots, arm/target flips, the tracking
  -- pose stream) push status up the moment things happen. Status frames
  -- carry the exact HTTP payload inside {t="status", data=...}; replies
  -- and acks mirror the HTTP shapes, so handleReply is shared. Returns
  -- when the socket dies; the caller drops to httpTick and retries later.
  local WS_MIN_SEND_GAP = 0.15 -- seconds; coalesces event bursts
  -- Async connect with OUR OWN deadline. The blocking http.websocket()
  -- can hang indefinitely against a half-up server -- which is exactly
  -- what a mid-deploy container looks like, and exactly when we try to
  -- reconnect -- freezing all of C2 until a manual reboot ("turrets never
  -- came back after the update").
  local function wsConnect()
    local okA = pcall(http.websocketAsync, wsUrl, headers)
    if not okA then return nil end
    local deadline = os.startTimer(8)
    while running do
      local ev, a, b = os.pullEvent()
      if ev == "websocket_success" and a == wsUrl then
        os.cancelTimer(deadline)
        return b
      elseif ev == "websocket_failure" and a == wsUrl then
        os.cancelTimer(deadline)
        return nil
      elseif ev == "timer" and a == deadline then
        return nil -- a late success just hands us a live handle next try
      end
    end
    return nil
  end

  local function runWs()
    local ws = wsConnect()
    if not ws then return false end
    state.spruceWsUp = true
    down = false
    state.flash = "SPRUCE WS UP"
    local lastSend = -math.huge
    -- Reply watchdog: the server answers EVERY status frame, so a link
    -- that eats our sends without replying (container killed mid-TCP, no
    -- clean close event) is dead even though ws.send still "works".
    local lastReply = os.clock()
    local replyDeadline = math.max(15, spruceCfg.statusSeconds * 5)
    local statusTimer = os.startTimer(0)
    local alive = true
    while running and alive do
      local sendNow = false
      local ev, a, b = os.pullEvent()
      if ev == "websocket_closed" and a == wsUrl then
        alive = false
      elseif ev == "websocket_message" and a == wsUrl then
        local okJ, msg = pcall(textutils.unserialiseJSON, b or "")
        if okJ and type(msg) == "table" and msg.t == "reply" then
          lastReply = os.clock()
          local acks = handleReply(msg)
          if acks then
            if not pcall(ws.send, textutils.serialiseJSON({ t = "ack", acks = acks })) then
              alive = false
            end
            if state.rebootRequest then pcall(ws.close); os.reboot() end
          end
        end
      elseif ev == "timer" and a == statusTimer then
        statusTimer = os.startTimer(spruceCfg.statusSeconds)
        sendNow = true
        if os.clock() - lastReply > replyDeadline then
          alive = false -- sends vanish into a dead link; fall back + retry
        end
      elseif ev == "spruce_push" then
        sendNow = true
      end
      if alive and sendNow and os.clock() - lastSend >= WS_MIN_SEND_GAP then
        lastSend = os.clock()
        local okBody, body = pcall(function()
          return textutils.serialiseJSON({ t = "status", data = buildSpruceStatus() })
        end)
        if okBody then
          if not pcall(ws.send, body) then alive = false end
        else
          state.flash = "SPRUCE STATUS ERR: " .. tostring(body)
        end
      end
    end
    state.spruceWsUp = false
    pcall(ws.close)
    return true
  end

  local WS_RETRY_SECONDS = 10
  local lastWsTry = -math.huge
  while running do
    if os.clock() - lastWsTry >= WS_RETRY_SECONDS then
      lastWsTry = os.clock()
      if runWs() then
        -- Had a live socket and lost it; HTTP keeps C2 alive meanwhile.
        state.flash = "SPRUCE WS LOST -- HTTP fallback"
        lastWsTry = os.clock()
      end
    end
    if running then httpTick() end
  end
end

-- Commit the modal line editor: an XYZ coord target, or a CONFIG field.
-- A bad parse keeps the editor open with a hint instead of closing.
local function commitPrompt()
  if ui.prompt.kind == "cfg" then
    local it = visibleConfigItems()[ui.prompt.idx]
    if it then
      local v, why = parseCfgValue(it, ui.prompt.text)
      if v == nil then
        ui.prompt.text, ui.prompt.err = "", why
        return
      end
      local prev = it.get()
      it.set(v)
      if it.profile then applyProfileEdit(function() it.set(prev) end) end
      if it.static then refreshStaticCannon() end
      if it.shipDep then
        local ok, why2 = refreshShip()
        if not ok then it.set(prev); refreshShip(); state.flash = "ship: " .. tostring(why2) end
      end
    end
    ui.prompt = nil
  else
    local c, why = parseCoord(ui.prompt.text)
    if c then
      ui.prompt = nil
      setCoordTarget(c)
    else
      ui.prompt.text, ui.prompt.err = "", why
    end
  end
end

-- Modal line editor (XYZ target or CONFIG field). Owns the keyboard while
-- ui.prompt is set so inputLoop routes everything here; Enter commits,
-- backspace edits, escape cancels.
local function handlePromptEvent(event)
  -- Opening with the `C` key queues a stray "c" char right behind the key
  -- event; drop that one so it doesn't seed the field.
  if ui.prompt.swallow then
    ui.prompt.swallow = nil
    if event[1] == "char" then return end
  end
  if event[1] == "char" then
    ui.prompt.text = ui.prompt.text .. event[2]
  elseif event[1] == "key" then
    local k = event[2]
    if k == keys.enter then
      commitPrompt()
    elseif k == keys.backspace then
      ui.prompt.text = ui.prompt.text:sub(1, -2)
    elseif k == keys.escape then
      ui.prompt = nil
    end
  end
end

-- Single-key actions shared by EVERY tab, so a key does the same thing no
-- matter where you are (the CONFIG tab layers arrow-nav on top, then falls
-- through to here). Returns true if it handled the key.
local function handleGlobalKey(k)
  if k == keys.f then fire()
  elseif k == keys.a then toggleArm()
  elseif k == keys.c then ui.prompt = { kind = "coord", text = "", swallow = true }
  elseif k == keys.k then state.recalRequest = true -- calibrate + auto-tune
  elseif k == keys.o then state.offsetCalRequest = true -- capture yawOffset at rest
  elseif k == keys.l then
    if trace.on then traceStop() else traceStart() end
  elseif k == keys.q then running = false
  else return false end
  return true
end

local function inputLoop()
  while running do
    local event = { os.pullEvent() }
    if ui.prompt then
      handlePromptEvent(event)
    elseif event[1] == "key" then
      local k = event[2]
      if ui.activeTab == "config" then
        -- CONFIG tab keyboard nav first; anything else falls through to the
        -- shared global keys so K/L/etc. work here too.
        local items = visibleConfigItems()
        if k == keys.up then
          ui.cfgSel = math.max(1, ui.cfgSel - 1)
        elseif k == keys.down then
          ui.cfgSel = math.min(#items, ui.cfgSel + 1)
        elseif k == keys.left and items[ui.cfgSel] then
          cfgAdjust(items[ui.cfgSel], -1)
        elseif k == keys.right and items[ui.cfgSel] then
          cfgAdjust(items[ui.cfgSel], 1)
        elseif k == keys.enter then
          if items[ui.cfgSel] and items[ui.cfgSel].etype ~= "enum" then
            openCfgEdit(ui.cfgSel)
          end
        else
          handleGlobalKey(k)
        end
      else
        handleGlobalKey(k)
      end
    elseif event[1] == "mouse_click" or event[1] == "monitor_touch" then
      local x, y = event[3], event[4]
      for _, cell in ipairs(ui.cells) do
        if y == cell.row and x >= cell.col1 and x <= cell.col2 then
          handleCommand(cell)
          break
        end
      end
    elseif event[1] == "mouse_scroll" then
      -- The CONFIG screen scrolls by moving the selection (the draw keeps the
      -- selected row on screen); setting ui.scroll directly would just snap
      -- back. Other tabs use ui.scroll as the raw viewport offset.
      if ui.activeTab == "config" then
        local n = #visibleConfigItems()
        ui.cfgSel = math.max(1, math.min(n, ui.cfgSel + event[2]))
      else
        ui.scroll = math.max(0, ui.scroll + event[2])
      end
    end
    draw()
  end
end

stopAll()
calibrate()
term.setBackgroundColor(colors.black)
term.clear()
if cfg.ship.enabled then updateShip() end
refreshRoster()
draw()
local ok, err = pcall(parallel.waitForAny, trackLoop, inputLoop, rednetLoop,
  spruceLoop)
stopAll()
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
term.clear()
term.setCursorPos(1, 1)
if not ok then error(err, 0) end
print("CCBigCannon stopped.")
