-- transponder.lua: a lightweight "secret" position beacon.
--
-- Drop this on a turtle (or any computer) with a wireless modem and place it
-- on a ship you want the turret to be able to target. It just GPS-locates
-- itself and rednet-broadcasts that position every half second.
--
-- It broadcasts on a PRIVATE protocol ("cannon-transponder"), NOT CCMinimap's
-- "airship-state", so the ship shows up in the turret's roster (and gets
-- picked up by Spruce's sniffer) but never appears on the CCMinimap UI.
--
-- Usage:  transponder.lua [callsign] [headingOffset]
--   callsign       optional name shown in the turret roster (as "#callsign").
--                  Defaults to the computer label, else "beacon-<id>".
--   headingOffset  degrees added to the derived heading (default 0) -- set
--                  it if the nav block's forward isn't ship-forward.
--
-- Requires a GPS constellation in range (same one CCMinimap uses). With no
-- modem it errors out; with no GPS fix it says so and holds (broadcasting
-- nothing) so the turret honestly shows the target as lost rather than
-- aiming at a stale position.
--
-- HEADING (optional): with a navigation table / compass PERIPHERAL attached
-- or adjacent, the broadcast also carries shipHeading, which upgrades the
-- turret's hull from the default sphere to the heading-oriented ellipsoid
-- (length along the ship). The needle points at world spawn (0,0) and
-- getRelativeAngle() reports it relative to the block's forward, so with
-- the GPS fix: heading = bearing-to-spawn - relativeAngle + headingOffset
-- (same math as CCMinimap; mirrors heading.lua, inlined so this file stays
-- a single-file drop-in). No nav peripheral = headingless, exactly as before.

local PROTOCOL = "cannon-transponder"
local INTERVAL = 0.5   -- seconds between broadcasts (matches the 5s turret TTL)
local GPS_TIMEOUT = 2  -- seconds to wait for a GPS fix each cycle

-- ----------------------------------------------------------------- modem --
local modem = peripheral.find("modem", function(_, m) return m.isWireless() end)
if not modem then
  error("no wireless modem attached -- a beacon needs one to broadcast", 0)
end
local modemName = peripheral.getName(modem)
if not rednet.isOpen(modemName) then rednet.open(modemName) end

-- -------------------------------------------------------------- callsign --
local args = { ... }
local callsign = args[1]
if not callsign or callsign == "" then
  callsign = os.getComputerLabel()
end
if not callsign or callsign == "" then
  callsign = "beacon-" .. os.getComputerID()
end
local headingOffset = tonumber(args[2]) or 0

-- --------------------------------------------------------------- heading --
-- Mirrors heading.lua's discovery (inlined: this file is a single-file
-- drop-in): typed finds first, then a full scan probing every attached
-- peripheral for a needle-reading method.
local NAV_TYPES = { "navigation_table", "ship_navigation_table", "compass" }
local NAV_METHODS = { "getRelativeAngle", "getYaw", "getRotationYaw", "getRotation" }
local nav = nil
local function probe(name, p)
  if not p then return nil end
  for _, m in ipairs(NAV_METHODS) do
    if type(p[m]) == "function" then
      return { name = name, p = p, method = m }
    end
  end
end
for _, t in ipairs(NAV_TYPES) do
  local p = peripheral.find(t)
  if p then
    nav = probe(peripheral.getName and peripheral.getName(p) or t, p)
    if nav then break end
  end
end
if not nav then
  for _, name in ipairs(peripheral.getNames()) do
    nav = probe(name, peripheral.wrap(name))
    if nav then break end
  end
end

-- World heading from the needle: it points at spawn (0,0), whose bearing
-- from HERE follows from our own GPS fix; the peripheral reading is the
-- needle's angle relative to the block's forward.
local function readHeading(x, z)
  if not nav then return nil end
  local ok, rel = pcall(nav.p[nav.method], nav.p)
  if not ok or rel == nil then return nil end
  if type(rel) == "table" then rel = rel.yaw or rel.heading or rel[1] end
  if type(rel) ~= "number" then return nil end
  local bearingToSpawn = math.deg(math.atan(-x, z))
  return (bearingToSpawn - rel + headingOffset) % 360
end

print(("transponder #%s -- broadcasting on '%s' via %s")
  :format(callsign, PROTOCOL, modemName))
print("(hidden from CCMinimap; visible to the turret + Spruce)")
if nav then
  print(("heading: %s.%s()%s -> oriented hull"):format(nav.name, nav.method,
    headingOffset ~= 0 and (" %+g deg"):format(headingOffset) or ""))
else
  print("no nav/compass peripheral -- headingless (turret uses sphere hull)")
end

-- ------------------------------------------------------------------ loop --
local hadFix = nil  -- nil = unknown, true/false = last logged state (logs on change)
while true do
  local x, y, z = gps.locate(GPS_TIMEOUT)
  if x then
    rednet.broadcast({
      airshipName = callsign,
      lastPos = { x = x, y = y, z = z },
      -- nil when no nav peripheral: the field is simply absent and the
      -- turret aims at its sphere hull instead of the oriented ellipsoid.
      shipHeading = readHeading(x, z),
      beacon = true,  -- marks this as one of our private beacons
    }, PROTOCOL)
    if hadFix ~= true then
      print(("GPS fix: %.0f, %.0f, %.0f -- broadcasting"):format(x, y, z))
      hadFix = true
    end
  elseif hadFix ~= false then
    print("NO GPS FIX -- holding (need a GPS constellation in range)")
    hadFix = false
  end
  sleep(INTERVAL)
end
