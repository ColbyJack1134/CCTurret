-- cannon.lua: CCBigCannon v1 -- closed-loop turret control for Create Big Cannons.
--
-- Reads the cannon mount's CannonYaw/CannonPitch NBT through a Block Reader,
-- drives two modem-attached Rotational Speed Controllers to aim at the
-- nearest non-whitelisted player, and fires via a Redstone Relay pulse.
--
-- Keys: F = fire, Q = quit.
--
-- Config lives in cannon.cfg (JSON). Missing keys are filled from DEFAULTS
-- on first boot and written back, CCMinimap-style. Edit the peripheral names
-- there to match your network (e.g. yaw = "Create_RotationSpeedController_0").

local Cfg = dofile("cfgutil.lua")

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
  cannon = { x = 0.5, y = 64.5, z = 0.5 },
  -- Subtracted from the world-space yaw so 0 matches the cannon's rest
  -- orientation. The original script's "facing south" cannon used 90.
  yawOffset = 0,
  -- Flip these if an axis spins away from the target instead of toward it
  -- (depends on your gearing).
  invertYaw = false,
  invertPitch = false,
  tolerance = 1,    -- degrees of acceptable aim error per axis
  speedGain = 5,    -- RPM per degree of error
  maxSpeed = 60,    -- RPM cap for the speed controllers
  detectionRange = 30,
  whitelist = {},   -- player names the turret must never target
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
local blockReader = resolve(cfg.peripherals.blockReader, "blockReader",
  "block reader on cannon mount")
local entDet = resolve(cfg.peripherals.playerDetector, "playerDetector",
  "player detector")
local relay = resolve(cfg.peripherals.relay, "redstone_relay", "redstone relay")

-- Shared state between the tracking loop and the input loop.
local running = true
local status = { target = nil, locked = false, yawErr = 0, pitchErr = 0 }

local whitelisted = {}
for _, name in ipairs(cfg.whitelist) do whitelisted[name] = true end

local function findTarget()
  for _, name in ipairs(entDet.getPlayersInRange(cfg.detectionRange)) do
    if not whitelisted[name] then return name end
  end
  return nil
end

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
local function anglesFor(tx, ty, tz)
  local dx, dy, dz = tx - cfg.cannon.x, ty - cfg.cannon.y, tz - cfg.cannon.z
  local distance = math.sqrt(dx ^ 2 + dy ^ 2 + dz ^ 2)
  local relPitch = math.deg(math.asin(dy / distance))
  local relYaw = angleDiff(math.deg(math.atan(dz, dx)) - cfg.yawOffset, 0)
  return relYaw, relPitch
end

local function stopAll()
  yaw.setTargetSpeed(0)
  pitch.setTargetSpeed(0)
  relay.setOutput(cfg.fireSide, false)
end

local function fire()
  relay.setOutput(cfg.fireSide, true)
  sleep(cfg.firePulseSeconds)
  relay.setOutput(cfg.fireSide, false)
end

local function draw()
  term.clear()
  term.setCursorPos(1, 1)
  print("CCBigCannon v1   [F] fire  [Q] quit")
  print(("Target: %s"):format(status.target or "none"))
  if status.target then
    print(("Yaw err: %6.1f  Pitch err: %6.1f"):format(status.yawErr, status.pitchErr))
    print(status.locked and "LOCKED ON" or "tracking...")
  end
end

-- Continuous control loop: re-reads the mount and the target every tick so
-- the turret tracks moving players instead of aiming at a stale position.
local function trackLoop()
  while running do
    status.target = findTarget()
    if status.target then
      local pos = entDet.getPlayerPos(status.target)
      local data = pos and blockReader.getBlockData()
      if pos and data and data.CannonYaw and data.CannonPitch then
        local relYaw, relPitch = anglesFor(pos.x, pos.y, pos.z)
        status.yawErr = angleDiff(relYaw, data.CannonYaw)
        status.pitchErr = relPitch - data.CannonPitch
        status.locked = math.abs(status.yawErr) < cfg.tolerance
          and math.abs(status.pitchErr) < cfg.tolerance
        yaw.setTargetSpeed(speedFor(status.yawErr, cfg.invertYaw))
        pitch.setTargetSpeed(speedFor(status.pitchErr, cfg.invertPitch))
      end
      draw()
      sleep(0.1)
    else
      status.locked = false
      yaw.setTargetSpeed(0)
      pitch.setTargetSpeed(0)
      draw()
      sleep(1)
    end
  end
end

local function inputLoop()
  while running do
    local _, key = os.pullEvent("key")
    if key == keys.f then
      fire()
    elseif key == keys.q then
      running = false
    end
  end
end

stopAll()
local ok, err = pcall(parallel.waitForAny, trackLoop, inputLoop)
stopAll()
term.setCursorPos(1, select(2, term.getSize()))
if not ok then error(err, 0) end
print("CCBigCannon stopped.")
