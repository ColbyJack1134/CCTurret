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
-- Keys: F = fire, A = arm/disarm, Q = quit. Mouse/touch for everything else.
-- While armed, the fire line is held high whenever both axes are locked on
-- (autocannon assumption) and dropped the moment lock is lost.
--
-- Config lives in cannon.cfg (JSON). Missing keys are filled from DEFAULTS
-- on first boot and written back, CCMinimap-style. Edit the peripheral names
-- there to match your network (e.g. yaw = "Create_RotationSpeedController_0").

local Cfg = dofile("cfgutil.lua")
local Heading = dofile("heading.lua")

local CONFIG = "cannon.cfg"

local DEFAULTS = {
  -- "auto" finds the single attached peripheral of that type (errors if
  -- there are zero or several). The two speed controllers share a type,
  -- so yaw and pitch must always be named explicitly.
  peripherals = {
    yaw = "Create_RotationSpeedController_0",
    pitch = "Create_RotationSpeedController_1",
    blockReader = "auto",
    playerDetector = "auto",
    relay = "auto",
  },
  -- Which side of the redstone relay the fire line is wired to.
  -- Relays only accept relative names: top/bottom/front/back/left/right.
  fireSide = "top",
  firePulseSeconds = 0.4,
  -- Center of the cannon mount (use the mount block's coords + 0.5).
  -- Only used while ship.enabled = false; aboard a ship the position is
  -- derived live from GPS + ship.offset instead.
  cannon = { x = 0.5, y = 64.5, z = 0.5 },
  -- Airship mode: locate the COMPUTER via GPS (wireless modem required),
  -- read ship yaw from the navigation table (CCMinimap-style needle math),
  -- and derive the cannon's world position by rotating `offset` -- the
  -- ship-local vector from the computer to the cannon mount, in blocks
  -- (left = negative right, down = negative up) -- by the live heading.
  -- While enabled, yawOffset means "cannon rest direction relative to
  -- ship-forward", so it stays correct at any heading.
  ship = {
    enabled = false,
    offset = { forward = 0, up = 0, right = 0 },
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
  -- orientation. The original script's "facing south" cannon used 90.
  yawOffset = 0,
  -- Added to the computed pitch, in degrees: -1 aims 1 degree below the
  -- target, +1 above. Plain aim bias -- not a sign fix.
  pitchOffset = 0,
  -- Drive sign per axis. "auto" calibrates on next boot: the axis is
  -- nudged ~2 degrees while the block reader watches which way the angle
  -- actually moves, and the measured true/false is written back here.
  -- Set back to "auto" whenever you re-gear the build.
  invertYaw = "auto",
  invertPitch = "auto",
  tolerance = 1,    -- degrees of acceptable aim error per axis
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
  speedGain = 5,    -- RPM per degree of error
  maxSpeed = 60,    -- RPM cap for the speed controllers
  -- Names listed here are dimmed in the target list as a "friendly"
  -- reminder; they can still be clicked deliberately. Works for player
  -- names AND ship callsigns -- when the cannon's own ship runs CCMinimap,
  -- its transponder shows up in the roster too, so list its callsign here.
  whitelist = {},
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

local function loadConfig()
  local cfg = {}
  local raw = readFile(CONFIG)
  if raw then
    local ok, parsed = pcall(textutils.unserialiseJSON, raw)
    if ok and type(parsed) == "table" then cfg = parsed end
  end
  local added = Cfg.deepMergeMissing(DEFAULTS, cfg)
  writeFile(CONFIG, Cfg.jsonPretty(cfg) .. "\n")
  if #added > 0 then
    print("Added config keys: " .. table.concat(added, ", "))
  end
  return cfg
end

local cfg = loadConfig()

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

local yaw = need(cfg.peripherals.yaw, "yaw speed controller")
local pitch = need(cfg.peripherals.pitch, "pitch speed controller")
local blockReader = resolve(cfg.peripherals.blockReader, "block_reader",
  "block reader on cannon mount")
local entDet = resolve(cfg.peripherals.playerDetector, "player_detector",
  "player detector")
local relay = resolve(cfg.peripherals.relay, "redstone_relay", "redstone relay")

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
local PEER_TTL = 5  -- seconds without a broadcast before a ship is dropped
local transponderModem = peripheral.find("modem",
  function(_, m) return m.isWireless() end)
if transponderModem then
  local modemName = peripheral.getName(transponderModem)
  if not rednet.isOpen(modemName) then rednet.open(modemName) end
end

-- Live ship fix: computer world position, heading, and the derived cannon
-- position. freshUntil guards against aiming on stale data when GPS or the
-- nav table stop answering.
local ship = {
  pos = nil, heading = nil, cannon = nil, rel = nil,
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
  local off = cfg.ship.offset
  ship.pos = { x = x, y = y, z = z }
  ship.heading = heading
  ship.pitch, ship.roll = pitchDeg, rollDeg
  ship.basis = { f = f2, u = u3, r = r2 }
  ship.cannon = {
    x = x + f2.x * off.forward + u3.x * off.up + r2.x * off.right,
    y = y + f2.y * off.forward + u3.y * off.up + r2.y * off.right,
    z = z + f2.z * off.forward + u3.z * off.up + r2.z * off.right,
  }
  ship.freshUntil = os.clock() + 3
end

-- Where the cannon is right now (world frame), or nil when the ship fix
-- has gone stale. Static mode just returns the configured position.
local function cannonPos()
  if not cfg.ship.enabled then return cfg.cannon end
  if os.clock() > ship.freshUntil then return nil end
  return ship.cannon
end

local whitelisted = {}
for _, name in ipairs(cfg.whitelist) do whitelisted[name] = true end

-- Shared state between the tracking loop and the input loop.
local running = true
local state = {
  targetKind = nil,  -- "player" | "ship", or nil
  targetName = nil,  -- player name / ship callsign we're locked onto
  lost = false,      -- target set but offline / other dim / transponder quiet
  noFix = false,     -- ship mode and GPS/nav stopped answering
  locked = false,    -- both axes within tolerance
  armed = false,     -- auto-fire master switch (ARM button / A key)
  firing = false,    -- fire line currently held high by auto-fire
  yawErr = 0,
  pitchErr = 0,
  miss = nil,        -- ship target: barrel line's miss distance from the
                     -- transponder in blocks (drives the hull fire gate)
  roster = {},       -- { {kind, name, x, y, z, dist?}, ... } sorted by distance
  peerShips = {},    -- transponder ships by callsign: {x,y,z,heading,seenAt}
  mount = nil,       -- last block-reader NBT, for the debug tab
  flash = nil,       -- transient status message (e.g. "FIRED")
}

local ui = {
  tabs = {
    { id = "targets", label = " TARGETS " },
    { id = "debug", label = " DEBUG " },
  },
  activeTab = "targets",
  cells = {},  -- clickable regions: {col1, col2, row, cmd, name?}
  scroll = 0,
}

-- ---------------------------------------------------------------- aiming --

-- Smallest signed angle from `current` to `target`, in [-180, 180].
local function angleDiff(target, current)
  return ((target - current + 180) % 360) - 180
end

-- Proportional controller: RPM scales with error, capped at maxSpeed.
local function speedFor(diff, invert)
  if math.abs(diff) < cfg.tolerance then return 0 end
  local speed = math.min(math.abs(diff) * cfg.speedGain, cfg.maxSpeed)
  local sign = diff > 0 and 1 or -1
  if invert then sign = -sign end
  return speed * sign
end

-- World-space target position -> desired mount yaw/pitch in degrees, plus
-- the distance in blocks. Returns nil when the cannon position is unknown
-- (stale ship fix).
local function anglesFor(tx, ty, tz)
  local c = cannonPos()
  if not c then return nil end
  local dx, dy, dz = tx - c.x, ty - c.y, tz - c.z
  local distance = math.sqrt(dx ^ 2 + dy ^ 2 + dz ^ 2)
  if cfg.ship.enabled then
    -- Project the target direction onto the ship frame: the mount's
    -- yaw/pitch are deck-relative, and a rolled deck couples roll into
    -- BOTH axes -- a constant heading subtraction can't express that.
    local b = ship.basis
    local df = (dx * b.f.x + dy * b.f.y + dz * b.f.z) / distance
    local du = (dx * b.u.x + dy * b.u.y + dz * b.u.z) / distance
    local dr = (dx * b.r.x + dy * b.r.y + dz * b.r.z) / distance
    local relPitch = math.deg(math.asin(du)) + cfg.pitchOffset
    -- -90: deck yaw is measured from ship-forward here, while the old
    -- yaw-only formula measured from ship-right; keeps the tuned
    -- yawOffset meaning what it always meant.
    local relYaw = angleDiff(math.deg(math.atan(dr, df)) - 90 - cfg.yawOffset, 0)
    return relYaw, relPitch, distance
  end
  local relPitch = math.deg(math.asin(dy / distance)) + cfg.pitchOffset
  local worldYaw = math.deg(math.atan(dz, dx))
  return angleDiff(worldYaw - cfg.yawOffset, 0), relPitch, distance
end

-- areaRadius/avoidRadius for a ship target, per-callsign override first.
local function shipArea(name)
  local o = cfg.shipTargets.perShip[name] or {}
  return o.areaRadius or cfg.shipTargets.areaRadius,
    o.avoidRadius or cfg.shipTargets.avoidRadius
end

-- Ship-target fire gate: would a shot along the barrel's CURRENT direction
-- land within `area` blocks of the transponder, but no closer than `avoid`?
-- Returns gate, missBlocks. Perpendicular miss distance of the barrel line
-- from the transponder is dist * sin(angular error); the yaw component
-- shrinks by cos(pitch) on the way to a true angular distance.
local function hullGate(center, mount, area, avoid)
  local relYaw, relPitch, dist = anglesFor(center.x, center.y, center.z)
  if not relYaw then return false, nil end
  local ey = angleDiff(relYaw, mount.CannonYaw)
    * math.cos(math.rad(mount.CannonPitch))
  local ep = relPitch - mount.CannonPitch
  local ang = math.min(math.sqrt(ey * ey + ep * ep), 90)
  local miss = dist * math.sin(math.rad(ang))
  return miss <= area and miss >= avoid, miss
end

local function stopMotors()
  yaw.setTargetSpeed(0)
  pitch.setTargetSpeed(0)
end

local function stopAll()
  stopMotors()
  relay.setOutput(cfg.fireSide, false)
end

-- Manual single pulse (F key / FIRE button).
local function fire()
  if state.firing then return end -- auto-fire already holds the line high
  relay.setOutput(cfg.fireSide, true)
  sleep(cfg.firePulseSeconds)
  relay.setOutput(cfg.fireSide, false)
  state.flash = "FIRED"
end

-- Auto-fire (autocannon): hold the fire line high while armed and locked,
-- drop it the moment lock is lost. A regular (single-shot) big cannon will
-- need a pulse + reload-delay mode here once cannon type is configurable.
local function setFiring(on)
  if state.firing == on then return end
  state.firing = on
  relay.setOutput(cfg.fireSide, on)
end

local function toggleArm()
  state.armed = not state.armed
  if not state.armed then setFiring(false) end
end

-- ----------------------------------------------------------- calibration --

-- Empirically determine the drive sign for one axis: nudge the controller
-- and watch which way the mount's angle moves. Tries both directions so it
-- still works when the axis starts resting against a clamp (pitch limits).
local function calibrateAxis(label, controller, nbtKey, wraps)
  for _, rpm in ipairs({ 8, -8 }) do
    local data = blockReader.getBlockData()
    local before = data and data[nbtKey]
    if not before then
      error(("calibration failed: block reader has no %s -- is it against the cannon mount?")
        :format(nbtKey), 0)
    end
    controller.setTargetSpeed(rpm)
    sleep(0.6)
    controller.setTargetSpeed(0)
    sleep(0.2) -- let the angle settle before re-reading
    local after = blockReader.getBlockData()[nbtKey]
    local delta = wraps and angleDiff(after, before) or (after - before)
    if math.abs(delta) >= 0.5 then
      local invert = (delta > 0) ~= (rpm > 0)
      print(("calibrated %s: %+d RPM moved %+.1f deg -> invert%s = %s")
        :format(label, rpm, delta, label:gsub("^%l", string.upper), tostring(invert)))
      return invert
    end
  end
  error(("calibration failed: %s axis did not move in either direction -- check gearing")
    :format(label), 0)
end

-- Run once per axis while its invert flag is "auto", then persist the
-- measured sign so later boots skip the wiggle.
local function calibrate()
  local changed = false
  if cfg.invertYaw == "auto" then
    cfg.invertYaw = calibrateAxis("yaw", yaw, "CannonYaw", true)
    changed = true
  end
  if cfg.invertPitch == "auto" then
    cfg.invertPitch = calibrateAxis("pitch", pitch, "CannonPitch", false)
    changed = true
  end
  if changed then
    writeFile(CONFIG, Cfg.jsonPretty(cfg) .. "\n")
    print("Calibration saved to " .. CONFIG)
    sleep(1)
  end
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
    end
    add(item)
  end
  for name, peer in pairs(state.peerShips) do
    if peerFresh(peer) then
      add({ kind = "ship", name = name, x = peer.x, y = peer.y, z = peer.z })
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
  state.miss = nil
  setFiring(false) -- never carry a held fire line across a target change
  if not name then stopMotors() end
end

-- ---------------------------------------------------------------- drawing --

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
  if state.targetName then
    local isShip = state.targetKind == "ship"
    term.setTextColor(colors.lightGray)
    term.write("Target ")
    term.setTextColor(isShip and colors.orange or colors.cyan)
    term.write((isShip and "#" or "@") .. state.targetName .. " ")
    if state.lost then
      term.setTextColor(colors.red)
      term.write("LOST")
    elseif state.noFix then
      term.setTextColor(colors.red)
      term.write("NO FIX")
    elseif state.locked then
      term.setTextColor(colors.lime)
      term.write("LOCKED")
      if state.firing then
        term.setTextColor(colors.orange)
        term.write(" FIRING")
      end
    else
      term.setTextColor(colors.yellow)
      -- One decimal: whole-degree rounding hid "0.3 deg off, not locked"
      -- as a confusing "y+0 p+0".
      term.write(("y%+.1f p%+.1f"):format(state.yawErr, state.pitchErr))
      if state.targetKind == "ship" and state.miss then
        term.write((" miss %.0fm"):format(state.miss))
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
    term.write("No target -- click a player or ship")
  end
  if state.armed and not state.firing then
    term.setTextColor(colors.red)
    term.write("  ARMED")
  end
  if state.flash then
    term.setTextColor(colors.orange)
    term.write("  " .. state.flash)
  end
end

local function drawTargetsList(w, h)
  local listTop, listBot = 3, h - 1
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

-- Live numbers for dialing in ship.offset / headingOffset / yawOffset:
-- everything the aim math sees, raw and derived.
local function drawDebugScreen(w, h)
  local row = 3
  local function line(label, value, fg)
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
    line("gps (computer)", fmtPos(ship.pos))
    line("needle rel", fmtDeg(ship.rel))
    line("ship heading", fmtDeg(ship.heading), colors.yellow)
    if ship.heading then
      -- Same direction in F3's language: stand facing ship-forward and
      -- this should match your "Facing" line exactly.
      local mcYaw, name, axis = f3Facing(ship.heading)
      line("  as F3", ("%s (%s)  %+.1f"):format(name, axis, mcYaw),
        colors.yellow)
    end
    line("cannon xyz", fmtPos(ship.cannon), colors.yellow)
    line("offset f/u/r", ("%g / %g / %g"):format(cfg.ship.offset.forward,
      cfg.ship.offset.up, cfg.ship.offset.right))
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
    line("mode", "static (land)")
    line("cannon xyz", fmtPos(cfg.cannon), colors.yellow)
  end
  line("yawOffset", cfg.yawOffset)
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
    line("yaw/pitch err",
      ("%+.1f / %+.1f"):format(state.yawErr, state.pitchErr))
    if state.targetKind == "ship" then
      local area, avoid = shipArea(state.targetName)
      line("hull miss", state.miss
        and ("%.1f (fire %g..%g)"):format(state.miss, avoid, area) or "?",
        state.locked and colors.lime or colors.yellow)
    end
  end
  while row <= h - 1 do -- clear leftovers from the targets list
    term.setCursorPos(1, row)
    term.setBackgroundColor(colors.black)
    term.clearLine()
    row = row + 1
  end
end

local function drawButtonBar(w, h)
  term.setCursorPos(1, h)
  term.setBackgroundColor(colors.black)
  term.clearLine()
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
  button(" FIRE ", "fire", state.locked and colors.lime or colors.red, true)
  button(state.armed and " DISARM " or " ARM ", "arm",
    state.armed and colors.red or colors.lime, true)
  button(" STOP ", "stop", colors.white, state.targetName ~= nil)
  term.setCursorPos(w - 18, h)
  term.setBackgroundColor(colors.black)
  term.setTextColor(colors.gray)
  term.write("F=fire A=arm Q=quit")
end

local function draw()
  local w, h = term.getSize()
  ui.cells = {}
  drawTabBar(w)
  drawStatus()
  if ui.activeTab == "debug" then
    drawDebugScreen(w, h)
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
  while running do
    -- Ship fix every tick while tracking (a climbing ship stair-steps the
    -- aim on anything slower), every 0.5s when idle; roster every 1s.
    if cfg.ship.enabled and (state.targetName or tick % 5 == 0) then
      updateShip()
    end
    if tick % 10 == 0 then refreshRoster() end
    tick = tick + 1
    if state.targetName then
      local pos = targetPos()
      if pos then
        state.lost = false
        -- Ship targets: aim below the transponder, never at it (the
        -- broadcast position is the block keeping the target on the air).
        local area, avoid
        local aimY = pos.y
        if state.targetKind == "ship" then
          area, avoid = shipArea(state.targetName)
          aimY = pos.y - avoid * 1.5
        end
        local relYaw, relPitch = anglesFor(pos.x, aimY, pos.z)
        if not relYaw then
          -- Stale ship fix: hold rather than aim with old coords/heading.
          state.noFix = true
          state.locked = false
          stopMotors()
        else
          state.noFix = false
          local data = blockReader.getBlockData()
          state.mount = data
          if data and data.CannonYaw and data.CannonPitch then
            state.yawErr = angleDiff(relYaw, data.CannonYaw)
            state.pitchErr = relPitch - data.CannonPitch
            if area then
              -- Hull gate replaces the per-axis tolerance lock: fire as
              -- soon as the shot would land on the hull ring, while the
              -- motors keep converging on the below-transponder aim point.
              state.locked, state.miss = hullGate(pos, data, area, avoid)
            else
              state.locked = math.abs(state.yawErr) < cfg.tolerance
                and math.abs(state.pitchErr) < cfg.tolerance
            end
            yaw.setTargetSpeed(speedFor(state.yawErr, cfg.invertYaw))
            pitch.setTargetSpeed(speedFor(state.pitchErr, cfg.invertPitch))
          else
            -- No mount reading: don't keep reporting (or firing on) a lock
            -- computed from stale angles.
            state.locked = false
          end
        end
      else
        state.lost = true
        state.locked = false
        stopMotors()
      end
      setFiring(state.armed and state.locked)
      draw()
      sleep(0.1)
    else
      stopMotors()
      setFiring(false)
      -- Idle aim loop doesn't touch the mount; keep the debug tab live.
      if ui.activeTab == "debug" then
        state.mount = blockReader.getBlockData()
      end
      draw()
      sleep(0.5)
    end
    state.flash = nil
  end
end

local function handleCommand(cell)
  if cell.cmd == "select" then
    setTarget(cell.kind, cell.name)
  elseif cell.cmd == "fire" then
    fire()
  elseif cell.cmd == "arm" then
    toggleArm()
  elseif cell.cmd == "stop" then
    setTarget(state.targetKind, state.targetName) -- toggle off
  elseif cell.cmd:sub(1, 4) == "tab_" then
    ui.activeTab = cell.cmd:sub(5)
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
    local _, msg = rednet.receive(STATE_PROTOCOL, 1.0)
    if msg then handlePeerState(msg) end
  end
end

local function inputLoop()
  while running do
    local event = { os.pullEvent() }
    if event[1] == "key" then
      if event[2] == keys.f then
        fire()
      elseif event[2] == keys.a then
        toggleArm()
      elseif event[2] == keys.q then
        running = false
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
      ui.scroll = math.max(0, ui.scroll + event[2])
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
local ok, err = pcall(parallel.waitForAny, trackLoop, inputLoop, rednetLoop)
stopAll()
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
term.clear()
term.setCursorPos(1, 1)
if not ok then error(err, 0) end
print("CCBigCannon stopped.")
