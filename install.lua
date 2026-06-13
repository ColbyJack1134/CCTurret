-- CCTurret standalone installer.
--
-- Pulls the turret file set straight from GitHub and launches cannon.lua.
-- This is the no-server path: if you run the Spruce C2 server, use the
-- one-line installer the Spruce Turrets tab hands you instead (it installs a
-- self-updating bootstrap). This script just drops the files and runs.
--
--   wget run https://raw.githubusercontent.com/ColbyJack1134/CCTurret/main/install.lua
--
-- cfgutil.lua and heading.lua live at the repo root as symlinks into the
-- vendored CCMinimap copy, so a raw-GitHub fetch of the root path returns the
-- symlink TEXT, not the file. Those two are pulled from the real vendor path
-- instead. Everything lands flat in the current directory, which is where
-- cannon.lua dofile()s its libraries from.

local BASE = "https://raw.githubusercontent.com/ColbyJack1134/CCTurret/main/"

-- on-disk name -> path within the repo
local FILES = {
  { "cannon.lua",     "cannon.lua" },
  { "ballistics.lua", "ballistics.lua" },
  { "autotune.lua",   "autotune.lua" },
  { "cfgutil.lua",    "vendor/CCMinimap/computercraft/cfgutil.lua" },
  { "heading.lua",    "vendor/CCMinimap/computercraft/heading.lua" },
}

-- Optional add-ons, installed only if you ask for them.
local OPTIONAL = {
  { "transponder.lua", "transponder.lua",
    "ship-beacon (run on a turtle aboard a target ship)" },
  { "autoloader.lua",  "autoloader.lua",
    "turtle reloader for a manually-loaded big cannon" },
}

if not http then
  error("the http API is disabled -- enable it in CC:Tweaked's config first", 0)
end

local function fetch(path)
  local url = BASE .. path
  local r = http.get(url)
  if not r then
    error("could not fetch " .. url
      .. "\n  -- is the repo public and the http API enabled?", 0)
  end
  local body = r.readAll()
  r.close()
  return body
end

local function install(disk, path)
  local body = fetch(path)
  if fs.exists(disk) then fs.delete(disk) end
  local f = fs.open(disk, "w")
  f.write(body)
  f.close()
  print("  " .. disk)
end

local function ask(q)
  write(q .. " [y/N] ")
  local a = read()
  return a:lower():sub(1, 1) == "y"
end

print("Installing CCTurret...")
for _, file in ipairs(FILES) do install(file[1], file[2]) end

for _, opt in ipairs(OPTIONAL) do
  if ask("Install " .. opt[1] .. "? (" .. opt[3] .. ")") then
    install(opt[1], opt[2])
  end
end

-- Optional startup.lua so the turret comes back up after a chunk reload.
if ask("Run cannon.lua automatically on every boot?") then
  if fs.exists("startup.lua") then fs.delete("startup.lua") end
  local f = fs.open("startup.lua", "w")
  f.write('shell.run("cannon")\n')
  f.close()
  print("  startup.lua")
end

print("")
print("Done. First run writes cannon.cfg + cannon.cal and calibrates,")
print("so DISARM is the default -- give the barrel clearance, then press")
print("CAL (K) once you've set the gun's material/barrels (or charges) and")
print("its position on the CONFIG tab.")
print("")
if ask("Launch cannon.lua now?") then
  shell.run("cannon")
end
