-- Self-updating turret boot loader (SPRUCE_PLAN.md phase 0). Pulls the
-- turret file set from the Spruce server on every boot, then launches
-- cannon.lua. Modeled on Spruce's drone startup.lua but its own script:
-- turrets are not drones, so there's no role list and no config-defaults
-- merge (cannon.lua owns its cannon.cfg/cannon.cal defaults and fails
-- loudly on bad input).
--
-- Install (once) on the turret computer, then reboot:
--   wget <server>/api/drone/lua/turret-startup.lua startup.lua
--
-- The repo file is named turret-startup.lua because the drone startup owns
-- the startup.lua name on the server; on the computer it lives as
-- startup.lua. cannon.cfg / cannon.cal / turret.cfg are operator-owned and
-- never touched by the sync.
--
-- __SERVER_URL__ is substituted by the server from CLIENT_SERVER_URL.
local SERVER = "__SERVER_URL__"

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

local function fetchText(url)
  local ok, r = pcall(http.get, url)
  if not ok or not r then return nil end
  local body = r.readAll()
  r.close()
  return body
end

-- Loud on an unreachable server (once, not per file): a turret silently
-- running weeks-old disk code is the exact failure mode that bit the
-- CCMinimap fleet, and "no symptoms" is worse than a boot-time warning.
local warned = false
local function syncFile(name, localName)
  localName = localName or name
  local remote = fetchText(SERVER .. "/api/drone/lua/" .. name)
  if not remote then
    if not warned then
      warned = true
      print("WARN: no reply from " .. SERVER .. " -- booting from disk code")
    end
    return false
  end
  if readFile(localName) == remote then return false end
  if fs.exists(localName) then fs.delete(localName) end
  writeFile(localName, remote)
  print("Updated " .. localName)
  return true
end

-- Self-update first so a changed bootstrap re-runs before syncing the rest.
if syncFile("turret-startup.lua", "startup.lua") then
  print("startup.lua updated; rebooting...")
  sleep(0.5)
  os.reboot()
end

-- The turret file set. cannon.lua dofile()s the four libs from its own
-- directory, so everything syncs flat next to startup.lua. cfgutil/heading
-- are served from the CCMinimap submodule on the server (shared modules).
syncFile("cannon.lua")
syncFile("ballistics.lua")
syncFile("autotune.lua")
syncFile("cfgutil.lua")
syncFile("heading.lua")

-- transponder.lua is the standalone ship beacon, not part of the turret
-- program; keep it fresh only where an operator already installed it.
if fs.exists("transponder.lua") then syncFile("transponder.lua") end

shell.run("cannon")
