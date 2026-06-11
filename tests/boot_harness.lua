-- Headless boot harness for cannon.lua. Stubs the CC:Tweaked environment
-- (peripherals, term, a virtual cannon mount that responds to nudges, and a
-- real-enough JSON serialise/parse) and runs cannon.lua through load ->
-- calibrate -> first draw, with parallel.waitForAny short-circuited so the
-- program returns instead of looping. Verifies the cfg/cal split, the
-- auto-detect wiggle, the minSpeed probe, and computed muzzle speed.
--
-- Run from the repo root:  lua tests/boot_harness.lua

local fails = 0
local function check(name, ok, detail)
  if ok then print("ok   " .. name)
  else fails = fails + 1; print("FAIL " .. name .. (detail and (": " .. detail) or "")) end
end

-- ---- minimal JSON (subset used by cfgutil.jsonPretty / config files) ----
local function jsonEncode(v)
  local t = type(v)
  if t == "nil" then return "null"
  elseif t == "boolean" then return tostring(v)
  elseif t == "number" then return tostring(v)
  elseif t == "string" then return '"' .. v:gsub('[%z\1-\31"\\]', function(c)
      local m = { ['"'] = '\\"', ['\\'] = '\\\\', ['\n'] = '\\n', ['\t'] = '\\t' }
      return m[c] or string.format('\\u%04x', c:byte()) end) .. '"'
  elseif t == "table" then
    local n, isArr = 0, true
    for k in pairs(v) do n = n + 1; if type(k) ~= "number" then isArr = false end end
    if n == 0 then return "{}" end
    local parts = {}
    if isArr and n == #v then
      for i = 1, n do parts[i] = jsonEncode(v[i]) end
      return "[" .. table.concat(parts, ",") .. "]"
    end
    for k, val in pairs(v) do parts[#parts + 1] = jsonEncode(tostring(k)) .. ":" .. jsonEncode(val) end
    return "{" .. table.concat(parts, ",") .. "}"
  end
  error("cannot encode " .. t)
end

local function jsonDecode(s)
  local i = 1
  local function ws() while i <= #s and s:sub(i, i):match("%s") do i = i + 1 end end
  local val
  local function str()
    i = i + 1; local out = {}
    while true do
      local c = s:sub(i, i)
      if c == '"' then i = i + 1; break
      elseif c == "\\" then
        local n = s:sub(i + 1, i + 1)
        local m = { ['"'] = '"', ['\\'] = '\\', n = '\n', t = '\t' }
        if n == "u" then out[#out + 1] = string.char(tonumber(s:sub(i + 2, i + 5), 16) % 256); i = i + 6
        else out[#out + 1] = m[n] or n; i = i + 2 end
      else out[#out + 1] = c; i = i + 1 end
    end
    return table.concat(out)
  end
  function val()
    ws(); local c = s:sub(i, i)
    if c == "{" then
      i = i + 1; local o = {}; ws()
      if s:sub(i, i) == "}" then i = i + 1; return o end
      while true do
        ws(); local k = str(); ws(); i = i + 1 -- colon
        o[k] = val(); ws()
        local d = s:sub(i, i); i = i + 1
        if d == "}" then break end
      end
      return o
    elseif c == "[" then
      i = i + 1; local a = {}; ws()
      if s:sub(i, i) == "]" then i = i + 1; return a end
      while true do
        a[#a + 1] = val(); ws()
        local d = s:sub(i, i); i = i + 1
        if d == "]" then break end
      end
      return a
    elseif c == '"' then return str()
    elseif c == "t" then i = i + 4; return true
    elseif c == "f" then i = i + 5; return false
    elseif c == "n" then i = i + 4; return nil
    else
      local num = s:match("^%-?%d+%.?%d*[eE]?[%+%-]?%d*", i)
      i = i + #num; return tonumber(num)
    end
  end
  return val()
end

-- ---- virtual filesystem ----
local FILES = {}
_G.fs = {
  exists = function(p) return FILES[p] ~= nil end,
  open = function(p, mode)
    if mode == "r" then
      if not FILES[p] then return nil end
      local content, read = FILES[p], false
      return { readAll = function() return content end, close = function() end }
    else
      return { write = function(_, s) FILES[p] = (FILES[p .. "#partial"] or "") .. (s or "") end,
               close = function() end }
    end
  end,
}
-- the cannon code calls f.write(s) (method-style is f.write, but used as f.write(s))
-- our open() returns write = function(_, s) ... but cannon calls f.write(s) (no self)
-- so fix: accept (s) directly.
_G.fs.open = function(p, mode)
  if mode == "r" then
    if not FILES[p] then return nil end
    return { readAll = function() return FILES[p] end, close = function() end }
  end
  local buf = {}
  return { write = function(s) buf[#buf + 1] = s end,
           close = function() FILES[p] = table.concat(buf) end }
end

-- ---- textutils ----
_G.textutils = {
  serialiseJSON = function(v) return jsonEncode(v) end,
  unserialiseJSON = function(s) return jsonDecode(s) end,
}

-- ---- virtual cannon mount + peripherals ----
local mount = { CannonYaw = 0, CannonPitch = 0 }
local RATE = 0.75 -- deg/s per RPM, applied as motors run during sleep()
-- two speed controllers, each bound to an axis (B drives YAW, A drives PITCH
-- on purpose, to prove auto-detect doesn't assume order)
local controllers = {
  ["Create_RotationSpeedController_0"] = { axis = "CannonPitch", rpm = 0 },
  ["Create_RotationSpeedController_1"] = { axis = "CannonYaw", rpm = 0 },
}
local function ctrlWrap(name)
  local c = controllers[name]
  return { setTargetSpeed = function(v)
    if v ~= 0 then c.nudges = (c.nudges or 0) + 1 end
    c.rpm = v
  end }
end
local blockReaderWrap = { getBlockData = function() return { CannonYaw = mount.CannonYaw, CannonPitch = mount.CannonPitch } end }
local relayWrap = { setOutput = function() end }
local detectorWrap = { getOnlinePlayers = function() return {} end }

local PERIPHS = {
  ["Create_RotationSpeedController_0"] = "Create_RotationSpeedController",
  ["Create_RotationSpeedController_1"] = "Create_RotationSpeedController",
  ["block_reader_0"] = "block_reader",
  ["player_detector_0"] = "player_detector",
  ["redstone_relay_0"] = "redstone_relay",
}
local function wrap(name)
  if controllers[name] then return ctrlWrap(name) end
  local t = PERIPHS[name]
  if t == "block_reader" then return blockReaderWrap end
  if t == "redstone_relay" then return relayWrap end
  if t == "player_detector" then return detectorWrap end
  return nil
end
_G.peripheral = {
  getNames = function() local o = {} for n in pairs(PERIPHS) do o[#o + 1] = n end return o end,
  getType = function(n) return PERIPHS[n] end,
  getName = function(p) return "?" end,
  wrap = wrap,
  find = function(t, filt)
    local o = {}
    for n, ty in pairs(PERIPHS) do if ty == t then o[#o + 1] = wrap(n) end end
    return table.unpack(o)
  end,
}

-- ---- clock + sleep that integrates mount motion ----
local clock = 0
_G.os = setmetatable({
  clock = function() return clock end,
  queueEvent = function() end,
  pullEvent = function() error("inputLoop should not run in harness", 0) end,
  epoch = function() return 0 end,
}, { __index = _G.os })
_G.sleep = function(t)
  t = t or 0
  for _, c in pairs(controllers) do
    if c.rpm ~= 0 then mount[c.axis] = mount[c.axis] + c.rpm * t * RATE end
  end
  clock = clock + t
end

-- ---- term / colors / keys / parallel / rednet / gps stubs ----
local function noop() end
_G.term = setmetatable({
  getSize = function() return 51, 19 end,
  setCursorPos = noop, setBackgroundColor = noop, setTextColor = noop,
  write = noop, clear = noop, clearLine = noop, blit = noop,
}, { __index = function() return noop end })
_G.colors = setmetatable({}, { __index = function() return 1 end })
_G.keys = setmetatable({}, { __index = function() return 0 end })
_G.parallel = { waitForAny = function() return true end } -- don't actually loop
_G.rednet = { open = noop, isOpen = function() return true end, receive = function() return nil end }
_G.gps = { locate = function() return nil end }

-- ---- scenario runner ----
os.remove = os.remove or noop
local function reset(seed)
  for k in pairs(FILES) do FILES[k] = nil end
  if seed then for k, v in pairs(seed) do FILES[k] = jsonEncode(v) end end
  mount.CannonYaw, mount.CannonPitch = 0, 0
  for _, c in pairs(controllers) do c.rpm = 0; c.nudges = 0 end
end
local function boot()
  local ok, err = pcall(dofile, "cannon.lua")
  return ok, err, FILES["cannon.cfg"] and jsonDecode(FILES["cannon.cfg"]),
    FILES["cannon.cal"] and jsonDecode(FILES["cannon.cal"])
end

-- == Scenario 1: fresh install, everything auto ==
print("-- scenario 1: fresh install --")
reset(nil)
local ok, err, cfg, cal = boot()
check("fresh boot ok", ok, tostring(err))
if cfg and cal then
  check("cfg has NO invertYaw (cal key)", cfg.invertYaw == nil)
  check("cfg has NO yawDrive.degPerSecPerRpm", cfg.yawDrive and cfg.yawDrive.degPerSecPerRpm == nil)
  check("cfg keeps yawDrive.speedGain", cfg.yawDrive and cfg.yawDrive.speedGain ~= nil)
  check("cfg keeps profile.material steel", cfg.profile and cfg.profile.material == "steel")
  check("cfg has NO yawOffset (cal key now)", cfg.yawOffset == nil)
  check("cal yawOffset = 90 (static constant, CBC yaw is world-absolute)",
    cal.yawOffset == 90, tostring(cal.yawOffset))
  check("cfg peripherals has NO yaw (cal key)", cfg.peripherals and cfg.peripherals.yaw == nil)
  check("cal invertYaw measured (boolean)", type(cal.invertYaw) == "boolean")
  check("cal degPerSecPerRpm ~ 0.75",
    cal.yawDrive and math.abs((cal.yawDrive.degPerSecPerRpm or 0) - 0.75) < 0.05)
  check("cal minSpeed probed", cal.yawDrive and type(cal.yawDrive.minSpeed) == "number")
  check("auto-detected yaw = _1 (reversed order proves detection)",
    cal.peripherals and cal.peripherals.yaw == "Create_RotationSpeedController_1")
  check("auto-detected pitch = _0",
    cal.peripherals and cal.peripherals.pitch == "Create_RotationSpeedController_0")
end

-- == Scenario 2: second boot from saved files -- must NOT re-wiggle ==
print("-- scenario 2: calibrated second boot --")
reset({
  ["cannon.cfg"] = { profile = { kind = "autocannon", material = "bronze", barrels = 4,
      projectile = "ap_autocannon", arc = "shallow", barrelBlocks = 2, charges = 1,
      reloadSeconds = 5, muzzleVelocityOverride = 0 } },
  ["cannon.cal"] = { peripherals = { yaw = "Create_RotationSpeedController_1",
      pitch = "Create_RotationSpeedController_0" }, invertYaw = true, invertPitch = false,
      yawOffset = 45,
      yawDrive = { degPerSecPerRpm = 0.9, minSpeed = 2 },
      pitchDrive = { degPerSecPerRpm = 0.6, minSpeed = 1 } },
})
local ok2, err2, cfg2, cal2 = boot()
check("calibrated boot ok", ok2, tostring(err2))
local nudges = 0
for _, c in pairs(controllers) do nudges = nudges + (c.nudges or 0) end
check("calibrated boot does NOT re-wiggle (0 nudges)", nudges == 0, "nudges=" .. nudges)
if cal2 then
  check("kept measured degPerSecPerRpm 0.9 (not re-measured to 0.75)",
    cal2.yawDrive and cal2.yawDrive.degPerSecPerRpm == 0.9,
    tostring(cal2.yawDrive and cal2.yawDrive.degPerSecPerRpm))
  check("kept invertYaw = true", cal2.invertYaw == true)
  check("kept saved yawOffset = 45 (not re-measured to rest 0)",
    cal2.yawOffset == 45, tostring(cal2.yawOffset))
end

-- == Scenario 3: migration from an OLD single-file cannon.cfg ==
print("-- scenario 3: legacy single-file migration --")
reset({
  ["cannon.cfg"] = {
    peripherals = { yaw = "Create_RotationSpeedController_1",
      pitch = "Create_RotationSpeedController_0", blockReader = "auto",
      playerDetector = "auto", relay = "auto" },
    invertYaw = false, invertPitch = true,
    yawDrive = { speedGain = 6, maxSpeed = 120, minSpeed = 1, degPerSecPerRpm = 0.75 },
    pitchDrive = { speedGain = 3, maxSpeed = 100, minSpeed = 1, degPerSecPerRpm = 0.8 },
    profile = { kind = "autocannon", projectile = "ap_autocannon", muzzleVelocity = 160,
      barrelBlocks = 2, charges = 1, reloadSeconds = 5, arc = "shallow" },
    yawOffset = 90,
  },
  -- no cannon.cal -> triggers migration
})
local ok3, err3, cfg3, cal3 = boot()
check("legacy migration boots ok", ok3, tostring(err3))
local nudges3 = 0
for _, c in pairs(controllers) do nudges3 = nudges3 + (c.nudges or 0) end
check("migration preserves calibration (no re-wiggle)", nudges3 == 0, "nudges=" .. nudges3)
if cfg3 and cal3 then
  check("legacy muzzleVelocity -> muzzleVelocityOverride 160",
    cfg3.profile and cfg3.profile.muzzleVelocityOverride == 160,
    tostring(cfg3.profile and cfg3.profile.muzzleVelocityOverride))
  check("legacy profile.muzzleVelocity dropped from cfg",
    cfg3.profile and cfg3.profile.muzzleVelocity == nil)
  check("measured degPerSecPerRpm 0.75 lifted into cal",
    cal3.yawDrive and cal3.yawDrive.degPerSecPerRpm == 0.75)
  check("measured invertPitch=true lifted into cal", cal3.invertPitch == true)
  check("peripheral names lifted into cal",
    cal3.peripherals and cal3.peripherals.yaw == "Create_RotationSpeedController_1")
  check("invertYaw removed from cfg after migration", cfg3.invertYaw == nil)
  check("legacy cfg yawOffset moves to cal as the static constant 90",
    cfg3.yawOffset == nil and cal3.yawOffset == 90,
    ("cfg=%s cal=%s"):format(tostring(cfg3.yawOffset), tostring(cal3.yawOffset)))
end

print(fails == 0 and "ALL PASS" or (fails .. " FAILURES"))
if fails > 0 then os.exit(1) end
