-- One-shot turret installer. Served per-turret by Spruce at
-- /api/drone/turret/install/<token>, which bakes in SERVER and TOKEN --
-- the operator runs the single line the turrets tab hands them:
--
--   wget run <server>/api/drone/turret/install/<token>
--
-- It writes turret.cfg (the Spruce link config cannon.lua reads), installs
-- the self-updating turret-startup.lua as startup.lua, and reboots; the
-- bootstrap then pulls the full turret file set and launches cannon.lua.
-- Safe to re-run (token rotation, server move): an existing callsign
-- override in turret.cfg survives, everything else is rewritten.
local SERVER = "__SERVER_URL__"
local TOKEN = "__TURRET_TOKEN__"

local cfg = { url = SERVER, token = TOKEN }
if fs.exists("turret.cfg") then
  local f = fs.open("turret.cfg", "r")
  local raw = f.readAll()
  f.close()
  local ok, old = pcall(textutils.unserialiseJSON, raw)
  if ok and type(old) == "table" and type(old.callsign) == "string" then
    cfg.callsign = old.callsign
  end
end
local f = fs.open("turret.cfg", "w")
f.write(textutils.serialiseJSON(cfg))
f.close()
print("Wrote turret.cfg for " .. SERVER)

local r = http.get(SERVER .. "/api/drone/lua/turret-startup.lua")
if not r then
  error("could not fetch turret-startup.lua from " .. SERVER
    .. " -- check the server is up and http is enabled", 0)
end
local body = r.readAll()
r.close()
if fs.exists("startup.lua") then
  print("Replacing existing startup.lua")
  fs.delete("startup.lua")
end
local out = fs.open("startup.lua", "w")
out.write(body)
out.close()

print("Installed self-updating startup.lua; rebooting...")
sleep(2)
os.reboot()
