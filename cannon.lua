-- cannon.lua: CCBigCannon v2 -- closed-loop turret control for Create Big Cannons.
--
-- Reads the cannon mount's CannonYaw/CannonPitch NBT through a Block Reader,
-- drives two modem-attached Rotational Speed Controllers, and fires via a
-- Redstone Relay pulse.
--
-- Targeting is click-to-lock, CCMinimap-style: a TARGETS tab lists every
-- online player (getOnlinePlayers/getPlayerPos are position-independent, so
-- this works from an airship where range scans see nobody); click a row to
-- track that player, click again or [ STOP ] to release.
--
-- Keys: F = fire, Q = quit. Mouse/touch for everything else.
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
  speedGain = 5,    -- RPM per degree of error
  maxSpeed = 60,    -- RPM cap for the speed controllers
  -- Names listed here are dimmed in the target list as a "friendly"
  -- reminder; they can still be clicked deliberately.
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
end

-- Live ship fix: computer world position, heading, and the derived cannon
-- position. freshUntil guards against aiming on stale data when GPS or the
-- nav table stop answering.
local ship = { pos = nil, heading = nil, cannon = nil, rel = nil, freshUntil = 0 }

local function updateShip()
  local x, y, z = gps.locate(0.5)
  local rel = Heading.relativeAngle(navSource)
  if not (x and rel) then return end
  ship.rel = rel
  local heading = Heading.fromPositionAndRelative(
    { x = x, z = z }, rel, cfg.ship.headingOffset)
  if not heading then return end
  -- Ship-forward and ship-right unit vectors in world XZ for this heading
  -- (compass convention: 0 = north/-Z, 90 = east/+X, clockwise).
  local h, off = math.rad(heading), cfg.ship.offset
  local fx, fz = math.sin(h), -math.cos(h)
  local rx, rz = math.cos(h), math.sin(h)
  ship.pos = { x = x, y = y, z = z }
  ship.heading = heading
  ship.cannon = {
    x = x + fx * off.forward + rx * off.right,
    y = y + off.up,
    z = z + fz * off.forward + rz * off.right,
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
  targetName = nil,  -- player we're locked onto, or nil
  lost = false,      -- target set but offline / other dimension
  noFix = false,     -- ship mode and GPS/nav stopped answering
  locked = false,    -- both axes within tolerance
  yawErr = 0,
  pitchErr = 0,
  roster = {},       -- { {name, x, y, z, dist?}, ... } sorted by distance
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

-- World-space target position -> desired mount yaw/pitch in degrees.
-- Returns nil when the cannon position is unknown (stale ship fix).
local function anglesFor(tx, ty, tz)
  local c = cannonPos()
  if not c then return nil end
  local dx, dy, dz = tx - c.x, ty - c.y, tz - c.z
  local distance = math.sqrt(dx ^ 2 + dy ^ 2 + dz ^ 2)
  local relPitch = math.deg(math.asin(dy / distance)) + cfg.pitchOffset
  local worldYaw = math.deg(math.atan(dz, dx))
  -- Aboard a ship the mount angle is ship-relative, so the live heading
  -- comes off the world-frame bearing before yawOffset.
  if cfg.ship.enabled then worldYaw = worldYaw - ship.heading end
  return angleDiff(worldYaw - cfg.yawOffset, 0), relPitch
end

local function stopMotors()
  yaw.setTargetSpeed(0)
  pitch.setTargetSpeed(0)
end

local function stopAll()
  stopMotors()
  relay.setOutput(cfg.fireSide, false)
end

local function fire()
  relay.setOutput(cfg.fireSide, true)
  sleep(cfg.firePulseSeconds)
  relay.setOutput(cfg.fireSide, false)
  state.flash = "FIRED"
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

-- Refresh the online-player roster. getPlayerPos is position-independent
-- (any distance, same dimension) so this works from an assembled airship;
-- players in another dimension get no coords and show as untargetable.
local function refreshRoster()
  local c = cannonPos() -- may be nil on a stale ship fix: rows lose distances
  local roster = {}
  for _, name in ipairs(entDet.getOnlinePlayers() or {}) do
    local item = { name = name }
    local pos = entDet.getPlayerPos(name)
    if pos and pos.x then
      item.x, item.y, item.z = pos.x, pos.y, pos.z
      if c then
        local dx, dy, dz = pos.x - c.x, pos.y - c.y, pos.z - c.z
        item.dist = math.floor(math.sqrt(dx * dx + dy * dy + dz * dz))
      end
    end
    roster[#roster + 1] = item
  end
  table.sort(roster, function(a, b)
    local da, db = a.dist or math.huge, b.dist or math.huge
    if da ~= db then return da < db end
    return a.name < b.name
  end)
  state.roster = roster
end

local function setTarget(name)
  if state.targetName == name then name = nil end -- click again to release
  state.targetName = name
  state.lost = false
  state.locked = false
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
    term.setTextColor(colors.lightGray)
    term.write("Target ")
    term.setTextColor(colors.cyan)
    term.write(state.targetName .. " ")
    if state.lost then
      term.setTextColor(colors.red)
      term.write("LOST")
    elseif state.noFix then
      term.setTextColor(colors.red)
      term.write("NO FIX")
    elseif state.locked then
      term.setTextColor(colors.lime)
      term.write("LOCKED")
    else
      term.setTextColor(colors.yellow)
      term.write(("y%+.0f p%+.0f"):format(state.yawErr, state.pitchErr))
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
    term.write("No target -- click a player")
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
      local selected = item.name == state.targetName
      local friendly = whitelisted[item.name]
      term.setBackgroundColor(selected and colors.gray or colors.black)
      term.setTextColor(selected and colors.white or colors.lightGray)
      term.write(selected and ">" or " ")
      term.setTextColor(friendly and colors.gray or colors.cyan)
      term.write(("@%-16s"):format(item.name:sub(1, 16)))
      if item.dist then
        term.setTextColor(colors.white)
        term.write((" %dm"):format(item.dist))
      elseif not item.x then
        term.setTextColor(colors.gray)
        term.write(" other dim")
      end -- has coords but no cannon fix: distance unknown, leave blank
      ui.cells[#ui.cells + 1] =
        { col1 = 1, col2 = w, row = row, cmd = "select", name = item.name }
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
  else
    line("mode", "static (land)")
    line("cannon xyz", fmtPos(cfg.cannon), colors.yellow)
  end
  line("yawOffset", cfg.yawOffset)
  local m = state.mount
  line("CannonYaw", fmtDeg(m and m.CannonYaw))
  line("CannonPitch", fmtDeg(m and m.CannonPitch))
  if state.targetName then
    line("target", state.targetName, colors.cyan)
    line("yaw/pitch err",
      ("%+.1f / %+.1f"):format(state.yawErr, state.pitchErr))
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
  button(" STOP ", "stop", colors.white, state.targetName ~= nil)
  term.setCursorPos(w - 12, h)
  term.setBackgroundColor(colors.black)
  term.setTextColor(colors.gray)
  term.write("F=fire Q=quit")
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
    -- Ship fix every 0.5s (gps.locate is a rednet round-trip; 0.1s would
    -- spam the channel), roster every 1s, aim every 0.1s.
    if cfg.ship.enabled and tick % 5 == 0 then updateShip() end
    if tick % 10 == 0 then refreshRoster() end
    tick = tick + 1
    if state.targetName then
      local pos = entDet.getPlayerPos(state.targetName)
      if pos and pos.x then
        state.lost = false
        local relYaw, relPitch = anglesFor(pos.x, pos.y, pos.z)
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
            state.locked = math.abs(state.yawErr) < cfg.tolerance
              and math.abs(state.pitchErr) < cfg.tolerance
            yaw.setTargetSpeed(speedFor(state.yawErr, cfg.invertYaw))
            pitch.setTargetSpeed(speedFor(state.pitchErr, cfg.invertPitch))
          end
        end
      else
        state.lost = true
        state.locked = false
        stopMotors()
      end
      draw()
      sleep(0.1)
    else
      stopMotors()
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
    setTarget(cell.name)
  elseif cell.cmd == "fire" then
    fire()
  elseif cell.cmd == "stop" then
    setTarget(state.targetName) -- toggle off
  elseif cell.cmd:sub(1, 4) == "tab_" then
    ui.activeTab = cell.cmd:sub(5)
  end
end

local function inputLoop()
  while running do
    local event = { os.pullEvent() }
    if event[1] == "key" then
      if event[2] == keys.f then
        fire()
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
local ok, err = pcall(parallel.waitForAny, trackLoop, inputLoop)
stopAll()
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
term.clear()
term.setCursorPos(1, 1)
if not ok then error(err, 0) end
print("CCBigCannon stopped.")
