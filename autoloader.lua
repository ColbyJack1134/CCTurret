-- autoloader.lua: turtle-driven physical reload PoC for a Create Big Cannons
-- big gun. A single turtle, parked at the breech mouth and facing INTO the
-- bore, loads one full shell + 3 powder charges by hand using a Ram Rod.
--
-- How the ram rod works for a CC turtle
-- -------------------------------------
-- Vanilla CC:Tweaked turtles can't right-click arbitrary blocks (no
-- turtle.use()), BUT turtle.place() *does* use the currently selected item
-- against the space in front -- the same path that lets buckets place fluids
-- and wheat breed cows. The CBC Ram Rod is a usable item, so:
--   * select a munition slot + place()  -> the munition block appears at the
--     breech mouth, directly in front of the turtle.
--   * select the ram-rod slot + place() -> one ram stroke, pushing whatever
--     is in front one block deeper into the bore. Call it N times to seat the
--     munition N blocks in.
--
-- The cannon, back -> front: sliding breech (holds nothing) | chamber | chamber
-- | chamber | barrel(s). Only the shell may sit in a barrel; charges live in
-- the chambers. We load deepest-first: the shell is placed and rammed all the
-- way to the barrel, then each charge is placed and rammed to pack the chambers
-- behind it. As the bore fills, later munitions need fewer strokes -- hence the
-- 5 / 4 / 3 / 2 ram counts below.
--
-- Slots (PoC layout):
--   slot 1 = Ram Rod (one item is enough; it's a tool, not consumed)
--   slot 2 = shells   (>= 1)   -- topped up from the chest on the LEFT
--   slot 3 = charges  (>= 3)   -- topped up from the chest on the RIGHT
--
-- Restock: after each load (and once at boot) the turtle turns to face the
-- left chest and pulls shells into slot 2, then the right chest for charges
-- into slot 3, then turns back to face the breech. So it can run indefinitely
-- as long as the two chests stay stocked.
--
-- Trigger: a redstone pulse on TRIGGER_SIDE runs one full load cycle. You can
-- also press T at the console to load manually, or Q to quit. After loading,
-- the turtle goes back to waiting -- re-arm/fire is whatever drives the
-- redstone line (e.g. cannon.lua's reload phase pulsing this side).

---------------------------------------------------------------------------
-- Config
---------------------------------------------------------------------------

local SLOT = { ram = 1, shell = 2, charge = 3 }

-- One load cycle, in the order the turtle places things. Each step drops a
-- munition block at the breech mouth, then rams it `rams` blocks forward.
local SEQUENCE = {
  { slot = SLOT.shell,  name = "shell",    rams = 5 },
  { slot = SLOT.charge, name = "charge 1", rams = 4 },
  { slot = SLOT.charge, name = "charge 2", rams = 3 },
  { slot = SLOT.charge, name = "charge 3", rams = 2 },
}

local TRIGGER_SIDE = "back"  -- redstone rising edge here starts a cycle
local STROKE_DELAY = 0.25    -- seconds between place/ram strokes so the
                             -- server registers each block move before the next

-- Restock: the turtle faces the breech, so the side chests are reached by
-- turning. Left chest = shells, right chest = charges (per your build). After
-- topping up it turns back, ending where it started.
--
-- Targets are how full to keep each slot -- the turtle pulls the DIFFERENCE
-- between the current count and the target, so it refills whatever the last
-- shot used (it does NOT need to be empty; a +0 just means the slot is already
-- at/above target). Defaults "fill the slot": getItemSpace caps the pull at the
-- stack max, so a non-stacking shell never spills and a huge target is safe.
local RESTOCK = {
  enabled = true,
  shellTarget = 64,   -- keep slot 2 as full as the stack allows
  chargeTarget = 64,  -- keep slot 3 as full as the stack allows
}

---------------------------------------------------------------------------
-- Low-level actions
---------------------------------------------------------------------------

-- Count how many items sit in a given slot.
local function count(slot)
  return turtle.getItemCount(slot)
end

-- Place the block currently in `slot` into the space in front. Loud failure:
-- if the slot is empty or the space is blocked, we stop the whole cycle rather
-- than silently mis-loading the gun.
local function placeMunition(step)
  if count(step.slot) < 1 then
    error(("slot %d (%s) is empty -- refill it"):format(step.slot, step.name), 0)
  end
  turtle.select(step.slot)
  local ok, why = turtle.place()
  if not ok then
    error(("could not place %s at the breech: %s"):format(step.name, why or "blocked?"), 0)
  end
  sleep(STROKE_DELAY)
end

-- Ram the block in front forward `n` blocks. The ram rod is a usable item, so
-- turtle.place() with it selected performs one ram stroke. Note: item-use
-- actions don't always report success through place(), so a false return here
-- is logged, not fatal -- we don't want a spurious false to abort a good load.
-- (Make this strict once we've watched it behave in-world.)
local function ramForward(n)
  turtle.select(SLOT.ram)
  if count(SLOT.ram) < 1 then
    error("slot 1 has no ram rod", 0)
  end
  for i = 1, n do
    local ok = turtle.place()
    print(("  ram %d/%d%s"):format(i, n, ok and "" or "  (place() returned false)"))
    sleep(STROKE_DELAY)
  end
end

---------------------------------------------------------------------------
-- One full load cycle
---------------------------------------------------------------------------

-- Verify we have the ammo for a whole cycle before touching the gun, so we
-- never load a partial charge stack.
local function checkSupplies()
  local needShell, needCharge = 0, 0
  for _, step in ipairs(SEQUENCE) do
    if step.slot == SLOT.shell then needShell = needShell + 1 end
    if step.slot == SLOT.charge then needCharge = needCharge + 1 end
  end
  if count(SLOT.ram) < 1 then error("no ram rod in slot 1", 0) end
  if count(SLOT.shell) < needShell then
    error(("need %d shell(s) in slot 2, have %d"):format(needShell, count(SLOT.shell)), 0)
  end
  if count(SLOT.charge) < needCharge then
    error(("need %d charge(s) in slot 3, have %d"):format(needCharge, count(SLOT.charge)), 0)
  end
end

local function loadCycle()
  checkSupplies()
  print("Loading...")
  for _, step in ipairs(SEQUENCE) do
    print(("Placing %s, ramming %d."):format(step.name, step.rams))
    placeMunition(step)
    ramForward(step.rams)
  end
  print("Load complete -- ready to fire.")
end

---------------------------------------------------------------------------
-- Restock from the side chests
---------------------------------------------------------------------------

-- Pull from the inventory directly in front into `slot`, up to `target` items.
-- One at a time so we can stop the instant an item would land somewhere other
-- than `slot` (non-stacking item, or the chest holds the wrong thing) -- that
-- keeps the slot layout clean. The getItemSpace guard means a full slot is
-- skipped without sucking. Returns how many were pulled.
local function pullInto(slot, target)
  turtle.select(slot)
  local pulled = 0
  while count(slot) < target and turtle.getItemSpace(slot) > 0 do
    local before = count(slot)
    if not turtle.suck(1) then break end        -- chest in front is empty
    if count(slot) == before then break end     -- it spilled elsewhere; stop
    pulled = pulled + 1
  end
  return pulled
end

-- Face the left chest -> refill shells; face the right chest -> refill charges;
-- end facing the breech again (net zero rotation).
local function restock()
  turtle.turnLeft()
  local s = pullInto(SLOT.shell, RESTOCK.shellTarget)
  turtle.turnRight()
  turtle.turnRight()
  local c = pullInto(SLOT.charge, RESTOCK.chargeTarget)
  turtle.turnLeft()
  -- current/target makes a 0 self-explanatory: "16/64 (+0)" = already stocked,
  -- whereas "0/64 (+0)" = nothing pulled despite an empty slot -> the barrel is
  -- empty or isn't directly in front after the turn (check the side/height).
  print(("Restocked: shells %d/%d (+%d), charges %d/%d (+%d)."):format(
    count(SLOT.shell), RESTOCK.shellTarget, s,
    count(SLOT.charge), RESTOCK.chargeTarget, c))
end

-- Run one load, then restock, each guarded so a failure in either still leaves
-- the turtle waiting for the next trigger.
local function runReload()
  local ok, err = pcall(loadCycle)
  if not ok then print("LOAD ABORTED: " .. tostring(err)) end
  if RESTOCK.enabled then
    local rok, rerr = pcall(restock)
    if not rok then print("RESTOCK FAILED: " .. tostring(rerr)) end
  end
end

---------------------------------------------------------------------------
-- Main: wait for a redstone pulse or a keypress, then load
---------------------------------------------------------------------------

local function main()
  print("Autoloader ready.")
  if RESTOCK.enabled then pcall(restock) end   -- boot with a full magazine
  print(("Trigger: redstone on %s, or press T to load, Q to quit."):format(TRIGGER_SIDE))
  local lastRs = redstone.getInput(TRIGGER_SIDE)
  while true do
    local ev, p1 = os.pullEvent()
    if ev == "redstone" then
      local now = redstone.getInput(TRIGGER_SIDE)
      if now and not lastRs then            -- rising edge only
        runReload()
        -- The pulse's falling edge (and any other toggles) happened while the
        -- load was running, where sleep() silently ate the redstone events.
        -- Re-read the line so lastRs reflects reality -- trusting the pre-load
        -- value leaves it stuck HIGH and every future pulse is ignored.
        lastRs = redstone.getInput(TRIGGER_SIDE)
      else
        lastRs = now
      end
    elseif ev == "key" then
      if p1 == keys.t then
        runReload()
      elseif p1 == keys.q then
        print("Bye.")
        return
      end
    end
  end
end

main()
