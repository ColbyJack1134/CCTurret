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
-- Usage:  transponder.lua [callsign]
--   callsign  optional name shown in the turret roster (as "#callsign").
--             Defaults to the computer label, else "beacon-<id>".
--
-- Requires a GPS constellation in range (same one CCMinimap uses). With no
-- modem it errors out; with no GPS fix it says so and holds (broadcasting
-- nothing) so the turret honestly shows the target as lost rather than
-- aiming at a stale position.

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
local callsign = ...                      -- first command-line argument
if not callsign or callsign == "" then
  callsign = os.getComputerLabel()
end
if not callsign or callsign == "" then
  callsign = "beacon-" .. os.getComputerID()
end

print(("transponder #%s -- broadcasting on '%s' via %s")
  :format(callsign, PROTOCOL, modemName))
print("(hidden from CCMinimap; visible to the turret + Spruce)")

-- ------------------------------------------------------------------ loop --
local hadFix = nil  -- nil = unknown, true/false = last logged state (logs on change)
while true do
  local x, y, z = gps.locate(GPS_TIMEOUT)
  if x then
    rednet.broadcast({
      airshipName = callsign,
      lastPos = { x = x, y = y, z = z },
      -- No heading: a turtle can't read one, and the turret's ship-target
      -- aim doesn't use it (it aims relative to this broadcast position).
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
