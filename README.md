# CCBigCannon

Closed-loop turret control for [Create Big Cannons](https://modrinth.com/mod/create-big-cannons)
on CC:Tweaked, based on [pastebin tb4aiueb](https://pastebin.com/tb4aiueb)
(preserved in `original.lua`).

## v1 hardware

- **2x Rotational Speed Controller** (Create Crafts & Additions), modem-attached,
  driving the cannon mount's yaw and pitch inputs.
- **Block Reader** (Advanced Peripherals) against the cannon mount — reads the
  `CannonYaw` / `CannonPitch` NBT for closed-loop feedback.
- **Player Detector** (Advanced Peripherals) — target acquisition.
- **Redstone Relay** (CC:Tweaked) — one line: fire. Aiming needs no assembly
  or clutch line: the speed controllers hold at 0 RPM and the block reader
  gives absolute angles. A big cannon with a **physical reload** (`reload`
  config) adds two more lines — an assembly line held high (drop it to
  disassemble) and a momentary loader-trigger line — on the same relay's
  spare sides or a second relay.

## Usage

Shared libraries come from the CCMinimap submodule (`git submodule update
--init`) so there is exactly one tracked copy of each. Copy to the computer,
flat:

- `cannon.lua`
- `ballistics.lua`
- `autotune.lua`
- `vendor/CCMinimap/computercraft/cfgutil.lua` → `cfgutil.lua`
- `vendor/CCMinimap/computercraft/heading.lua` → `heading.lua`

Run `cannon.lua`. First boot writes `cannon.cfg` (hand-authored intent)
and `cannon.cal` (machine-measured), then calibrates. Setup is mostly
hands-off now:

- The two speed controllers no longer need naming: leave
  `peripherals.yaw`/`pitch` at `"auto"` and the calibration wiggle nudges
  each one and reads whether `CannonYaw` or `CannonPitch` moved to tell
  them apart. The resolved names are saved to `cannon.cal`.
- Muzzle speed is **computed**, never typed — set the gun's `material`
  and `barrels` (autocannon) or `charges` (bigcannon) and the b/s is
  derived from the CBC formula (see **Cannon profile**).
- `yawOffset` (the home/rest facing) is **measured**, not typed: it
  defaults to `"auto"`, and calibration reads the assembled rest yaw off
  the block reader — the first thing it does, before any rotation, since
  the gun reports its rest angle the moment it's assembled — and saves it
  to `cannon.cal`. So `cannon` position is the only thing you usually
  still set by hand (or `cannon.gps = true`), and a `cannon.cfg` copied
  to a second turret won't drag the first one's home position along.
- Drive sign, slew rate (`degPerSecPerRpm`), the `minSpeed` floor, and
  `yawOffset` are all measured by calibration and live in `cannon.cal`.

Everything calibrated lands in `cannon.cal`, which is safe to delete — the
**CAL** button (or `K`) rebuilds it. Edit any of it live on the **CONFIG**
tab (below) instead of quitting to a text editor.

Stationary builds can skip hardcoding the mount coords: set
`cannon.gps = true` (wireless modem + GPS constellation required) and
the computer locates itself once at boot, deriving the mount position
as the fix plus `cannon.offset` — mount minus computer in **world**
axes (+x east, +y up, +z south). Boot fails loudly if there's no fix;
rerun the program after moving the build. The DEBUG tab shows the
derived position.

## Airship mode

Set `ship.enabled = true` in `cannon.cfg`. Requires a wireless modem
(GPS) and a navigation table (heading, CCMinimap needle math with
`ship.headingOffset` correction). The cannon's world position is derived
each half-second from the computer's GPS fix plus `ship.offset` — the
ship-local computer→cannon-mount vector in blocks (`forward`/`up`/`right`,
left = negative right) — rotated by the live heading. `yawOffset` then
means "cannon rest direction relative to ship-forward". Ship pitch/roll
are assumed level for now. If GPS or the nav table stop answering, the
turret holds and shows NO FIX rather than aiming on stale data.

Three tabs along the top: **TARGETS** (the roster), **DEBUG** (live aim
numbers), and **CONFIG** (live settings editor — see below).

Keys: `F` fire (manual pulse), `A` arm/disarm, `C` enter an XYZ
target, `K` calibrate + auto-tune the drive, `L` toggle a diagnostic
trace, `Q` quit. On the CONFIG tab the arrow keys navigate (↑/↓ select a
row, ←/→ adjust it) and Enter opens a text edit; the single-key actions
(`F`/`A`/`C`/`K`/`L`/`Q`) work on every tab. The turret
continuously tracks the selected player; `LOCKED` means both axes are
within `tolerance`. With no target (or once a target is LOST) the barrel
returns to its neutral rest pose — yaw to its rest facing, pitch level —
rather than freezing where it last aimed; clear a target with **STOP**.
While **armed** (ARM button or `A`; disarmed by
default) the fire line is held high whenever the turret is locked on and
dropped the moment lock is lost — autocannon behavior; a pulse/reload
mode for regular cannons is planned.

Auto-fire is range-gated: beyond `maxDistance` blocks (default 50) the
line holds and the status shows OUT OF RANGE, while tracking continues
so fire resumes the moment the target closes back in. Manual `F` is not
gated. In the other direction, **burst hysteresis** (`burst` in the
config, on by default) keeps an opened gate from chattering: once
firing, the line stays high while the miss is still within
`burst.widen`× the normal gate (hitbox / hull ring / tolerance), and
only drops after `burst.holdSeconds` straight outside that widened
gate — ammo for coverage on a juking target. `burst.enabled = false`
reverts to the strict instant-drop gate. The widened hull ring grows
outward only; `avoidRadius` still protects the transponder during a
burst hold.

Player targets are aimed with **predictive lead** (`lead` in the
config): target velocity is measured across a short position history
(`windowSeconds`, newest-minus-oldest — adjacent-tick differences are
detector-jitter noise) and the turret aims where the target will be
after the shell's flight time — the arc solver's true time-of-flight
plus `latencySeconds` of fixed lag. The fire gate follows the
predicted box, so shells are gated on where the target *will* be, not
where it was. `enabled = false` reverts to aiming at the live
position. The DEBUG tab shows the live lead distance, target speed,
and lead time.

## Cannon profile

`profile` in the config describes the gun; it drives both the fire
mode and the ballistics:

- `kind`: `autocannon` holds the fire line while the gate is open;
  `bigcannon` pulses `firePulseSeconds` per shot and waits
  `profile.reloadSeconds` before the next (reload countdown on the
  DEBUG tab). By default that wait assumes an autoloader — the line
  just sits low. Set `reload.enabled` for a gun that must be torn down
  to reload (see **Reload cycle**).
- `projectile`: keys the constants table in `ballistics.lua` —
  big-cannon shells fall at −0.05 b/t², autocannon rounds and the
  mortar stone at −0.025, drag 0.99/tick for all (verified from CBC
  source; datapacks can override, so re-verify on a tuned server).
- `charges` (bigcannon): powder charges loaded — muzzle speed is
  2 b/t per charge. 5 charges = 200 b/s ≈ 686 blocks max range.
- `material` + `barrels` (autocannon): muzzle speed is a closed form of
  the build, **computed for you** — `20 × (base + perBarrel ×
  min(barrels, cap))` with `base/perBarrel/cap` from the material: cast
  iron `20×(5 + 2×min(b,2))`, bronze `20×(3 + 1.5×min(b,3))`, steel
  `20×(3 + 1.5×min(b,4))`. A full-length steel or cast-iron gun is 180
  b/s. So set `material` (`cast_iron`/`bronze`/`steel`) and the barrel
  count, not a velocity. Mind projectile lifetime: cast iron rounds
  despawn after ~99 blocks, bronze ~187, steel ~540.
- `muzzleVelocityOverride` (autocannon): `> 0` forces the muzzle speed
  in b/s instead of computing it — only for a **datapack-tuned server**
  whose numbers differ from the published ones; `0` = compute. (Old
  configs: a hand-tuned `muzzleVelocity` / `lead.muzzleVelocity`
  migrates into this override automatically.)
- `barrelBlocks`: mount pivot → muzzle tip, in blocks. CBC spawns the
  shell ~`barrelBlocks − 1.5` out along the barrel; on a long gun
  ignoring that shifts arcing shots by 15–25 blocks at range.
- `arc`: `shallow` (flat, fast) or `steep` (lobbed) when both
  solutions exist.

Pitch is solved ballistically every tick (gravity + drag, closed-form
horizontal time-of-flight + bisected pitch — `ballistics.lua`,
unit-tested against a per-tick simulation in `tests/`), launching from
the muzzle rather than the mount, and the solver's time-of-flight
feeds the lead solve. A target no arc reaches shows **NO ARC**: the
barrel tracks line-of-sight as a ready posture and auto-fire stays
gated until the target closes in. `pitchOffset` remains a plain aim
bias on top of the solution.

Player targets lock onto the centre of mass (`getPlayerPos` reports the
FEET — the entity position — so the turret aims `playerHitbox.aimHeight`
blocks above it; default 0.9, the middle of a standing player). Fire
opens early: as soon as the shot would pass through the body box rising
from the reported feet — `playerHitbox.width` wide and `playerHitbox.height`
tall (default 0.6 × 1.8) — the line goes high while the turret keeps
converging on centre mass. Raise `aimHeight` toward the head, or pad
`width`/`height`, if the gate feels too strict. The status line shows the
live horizontal / vertical miss until lock. `trackSeconds` (default 0.1, minimum 0.05 —
one game tick) sets the tracking loop period if you want faster aim
updates for more peripheral traffic.

## CONFIG tab and the cfg / cal split

Config lives in two files. `cannon.cfg` is **hand-authored intent** — the
gun profile, aim offsets, tolerances, travel limits, ship/reload wiring.
`cannon.cal` is **machine-measured** — the resolved yaw/pitch controller
names, the drive sign (`invertYaw`/`invertPitch`), the slew rate
(`degPerSecPerRpm`), the `minSpeed` floor, and `yawOffset` (the home/rest
facing, read from the assembled rest yaw). Calibration writes only
`cannon.cal`; it's safe to delete (the **CAL** button rebuilds it) and
keeps the hand-edited config clean. Because `yawOffset` lives here and is
ignored if found in `cannon.cfg`, copying a `cannon.cfg` to a new turret
never carries the old one's home position — it's re-measured per mount.

The **CONFIG** tab edits all of it live, no quitting to a text editor.
Rows are grouped (Build / Aim / Position / Arc limits / Drive /
Calibrated; calibrated rows are orange). Tap a row to select it, then use
its `[-]`/`[+]` steppers (numbers) or `<`/`>` (enums like `material`,
`arc`, `kind`), or `[=]` to type a value; arrow keys work too. Some
fields are **type-only** (just `[=]`, no steppers): the mount position /
GPS `offset` floats and the arc travel limits, where you want to enter an
exact value. Edits apply **immediately** — change `material` and the
muzzle speed recomputes; toggle **Position → gps** and edit the `offset`
and the mount position re-derives from the GPS fix, no reboot. **SAVE** writes
the changes to disk (each field to its own file), **CANCEL** reverts to
the values from when you opened the tab. Because measured values win over
the hand-edited file, pin a calibrated value by editing it here (or in
`cannon.cal`), not in `cannon.cfg`.

Setting up a new turret is then: drop the files on the computer, set the
gun's `material` + `barrels` (or `charges`) and the `cannon` position,
and press **CAL** — the drives auto-detect and calibrate themselves.

## Reload cycle

A real big cannon can't be reloaded while assembled — it has to be torn
back into blocks, loaded, and rebuilt. Set `reload.enabled = true` (a
`bigcannon` profile) and the controller drives that sequence on two
extra redstone lines instead of the autoloader's silent timer:

1. **Fire** — pulse `fireSide` for `firePulseSeconds`.
2. **Disassemble** — drop `assemblySide` (held high the rest of the
   time = assembled). Wait `reload.settleSeconds`.
3. **Load** — pulse `reloadSide` for `reload.reloadPulseSeconds` to kick
   off whatever loads the breech, then wait `profile.reloadSeconds` for
   it to finish.
4. **Reassemble** — raise `assemblySide`. Wait `reload.settleSeconds`,
   then the gun is ready for the next shot.

The cannon is **assembled by default**: the assembly line is high at
boot (the gun is built for the calibration nudge) and on quit, and only
drops during a reload. The whole cycle is non-blocking — aiming, the UI,
and transponder tracking keep running — but the barrel **holds** while
the contraption is torn down (it can't move what isn't there) and the
fire gate stays shut until reassembly. A manual `F` shot runs the same
cycle. The status line and the DEBUG `reload` line show the live phase
(`DISASSEMBLING` / `LOADING` / `ASSEMBLING`) and countdown.

Wiring: by default all three lines (`fireSide`, `assemblySide`,
`reloadSide`) are sides of the one fire relay — pick three that aren't
wired together. To drive the assembly/reload lines from a **second**
relay, set `reload.relay` to that relay's peripheral name. Don't care
about a settle pause? Set `reload.settleSeconds = 0`.

## Coordinate targets

Press `C` (or click **+ set XYZ coord** at the top of the TARGETS tab)
and type a world point as `x y z` (spaces or commas, decimals and
negatives fine) — Enter locks it, Escape cancels, a bad entry reprompts
with a hint. The point is shown on the status line as `*x, y, z` in
light blue and cleared with **STOP** like any other target.

It's a fixed aim point: no lead and no hitbox box — the turret arc-solves
straight at the coordinate and locks when both axes are within
`tolerance`, so it's the cleanest way to test ballistics. Stand off at a
known spot, read its F3 coords, target them, and watch where the shell
lands versus the DEBUG tab's predicted distance and time-of-flight.
Auto-fire still respects `maxDistance` (raise it for long shots) and the
arc gate (**NO ARC** past max range); manual `F` is ungated, so you can
lob a ranging shot at anything.

While a land target is set, the DEBUG tab forward-flies a shot from the
barrel's **actual current angle** (not the solver's answer) and shows
`impact xyz` (a world coordinate to go watch), `impact off` (how far that
landing is from the target), and `v.miss@range` (how high/low the shell
crosses the target's distance — `+` high, `−` low). It shares the flight
model with the solver, so a large `v.miss@range` means the barrel fired
before it settled onto the solution (loosen the slew or tighten
`tolerance`) or the angle convention is off; it does **not** validate
muzzle speed — for that, compare the predicted `impact xyz` to where the
shell is actually seen to land. Note `tolerance` is angular: 1° is ~3.5
blocks of miss at 200, ~10 at 600, so artillery wants it tight (≈0.2°),
which in turn needs the drive tuned enough to settle that close.

## Transponder targets

With any wireless modem attached, the TARGETS tab also lists every ship
broadcasting CCMinimap's transponder (`airship-state` rednet protocol,
0.5 s cadence) — `#callsign` rows in orange next to `@player` rows in
cyan. Ships that go quiet for 5 s drop from the roster; a tracked ship
that goes quiet shows `LOST` and the turret holds. **Note:** if the
cannon's own ship runs CCMinimap, its own callsign appears in the
roster — add it to `whitelist` so it renders dimmed.

The broadcast position is the peer's *computer* — destroying it loses
the target's coords. So ship targets use an area instead of a point
(`shipTargets` in the config, per-callsign overrides under `perShip`):
the turret aims `1.5 * avoidRadius` below the transponder, and the fire
gate opens whenever the shot would land within `areaRadius` of the
transponder but no closer than `avoidRadius`. Hull hits anywhere in
that ring fire immediately while the barrel keeps converging on the
center; the transponder block itself is never fired on. The status
line shows the live miss distance while closing in.

### Private beacon (`transponder.lua`)

To put a ship on the turret's roster *without* it appearing on
CCMinimap, run `transponder.lua` on a turtle (or computer) with a
wireless modem and place it aboard. It GPS-locates itself and
broadcasts its position every 0.5 s on a **private** rednet protocol
(`cannon-transponder`) instead of CCMinimap's `airship-state` — the
minimap only listens for the latter, so the beacon never shows up
there, but the turret tracks it exactly like any other transponder
ship (and Spruce's sniffer still sees it). Handy for marking a test
target you want to shoot at without lighting it up on everyone's map.

Run `transponder.lua [callsign]` — the optional callsign is the
`#callsign` shown in the roster (defaults to the computer label, else
`beacon-<id>`). It needs a GPS constellation in range (the same one
CCMinimap uses); with none it prints `NO GPS FIX` and holds rather than
broadcasting stale coords, so the turret honestly shows the target as
lost. It carries position only (no heading — the ship-target aim
doesn't need one).

## Travel limits

`limits.yaw` (default −90..+90) and `limits.pitch` (default −30..+60)
bound the mount in rest-relative degrees (0 = the barrel's rest
orientation, wherever `yawOffset` points it — e.g. off the ship's
starboard side). A target outside the arc shows `OUT OF ARC`: the
barrel parks at the nearest limit, ready for re-entry, and fire only
opens if shots from inside the arc would still hit. Slews never route
through the zone behind the arc (no shortest-path wrap), so the barrel
can't sweep across your own ship. For a free-standing 360° cannon set
yaw to −180..180 (note: it will then unwind the long way around rather
than crossing the ±180 seam).

The drive adds **velocity feedforward** on top of the proportional
term: the boot calibration wiggle measures each axis's slew rate
(`degPerSecPerRpm`, ~0.75 for a direct-drive mount), and the
controller feeds the aim point's own angular rate straight into the
RPM command. Without it, a pure-P loop trails any crossing target by
`speed / (rate × gain)` blocks at every distance — enough to keep the
fire gate closed against anything faster than a walk. Set
`degPerSecPerRpm` (or an invert flag) back to `"auto"` after
re-gearing to re-measure — or just press the **CAL** button (or `K`),
which re-runs the wiggle live and saves the result.

On top of P + feedforward there's an optional **derivative term**
(`yawDrive.kd` / `pitchDrive.kd`, a PD loop). It brakes the command in
proportion to how fast the aim *error* is closing —
`kd × d(err)/dt ÷ degPerSecPerRpm` — so the barrel eases onto the
target instead of overshooting and hunting just before lock. It acts on
the error, not the mount's velocity, so it stays out of the
feedforward's way on a moving target (steady tracking has ~0 error
rate), and it only shapes an already-active drive — it never pushes a
parked barrel. `kd = 0` (the default) is the pure P+feedforward loop.
If an axis oscillates around the target right before locking, raise
`kd` in the **CONFIG** tab (Drive group) a little at a time — try
0.2–0.5; too high makes convergence sluggish. The DEBUG tab's
`err rate y/p` line (shown once `kd ≠ 0`) is the thing it's damping:
watch it settle toward 0 without ringing.

If an axis spins away from the target, flip `invertYaw` / `invertPitch`
in the config instead of regearing.

## Calibrate + auto-tune (the CAL button / `K`)

**`K`** (or the **CAL** button) does the whole drive setup in one automatic
pass — no hand-tuning, no pre-steps:

1. Re-detects which speed controller is yaw vs pitch (if `"auto"`).
2. Wiggles each axis to measure the drive sign, slew rate
   (`degPerSecPerRpm`), and the `minSpeed` floor.
3. Measures the real control-loop period (by timing its own peripheral
   work — no target needed).
4. Auto-tunes each axis's `approach` cap: it drives step responses on
   **internal angle targets** (it just steps the mount in place — you do
   **not** need to be tracking anything), raises `approach` until a step
   *just* starts to overshoot, and keeps the largest value that stays
   clean (re-verified to absorb sensor noise). It writes the tuned
   `approach` + a matching `speedGain` to `cannon.cfg`, zeroes `kd` (the
   cap supersedes the D term), and logs every probe to `cannon.tune.log`.

The whole thing takes ~1–2 minutes and the barrel steps back and forth on
its own — **disarm first** (`A`) and give it clearance. Boot only does the
quick wiggle (steps 1–2) so it doesn't lengthen every startup; press `K`
once after setup for the full auto-tune. The search and its convergence are
validated offline against a mount model fitted to real in-game traces,
across a range of gearing, inertia, loop rates, and sensor noise
(`tests/autotune_sim.lua`).

## Settling and lock

A Create speed controller can't turn slower than ~1 RPM (`minSpeed`).
The drive never commands *between* 0 and that floor — a sub-floor
command just stalls the mount in place — so it drives at ≥ `minSpeed`
or parks at 0. The barrel therefore settles about `minSpeed / speedGain`
degrees from the target: **raise `speedGain` to settle tighter** (the
opposite of the usual pure-P intuition, because there's no deadband to
overshoot — it parks the instant it can't usefully drive). Only lower
`speedGain` if an axis visibly oscillates *around* the target; if it
stops *short*, that's the floor — raise the gain.

Because 1 RPM is a hard floor, you can't always reach an arbitrarily
tight `tolerance`. `lockWhenStalled` (on by default) counts an axis as
locked once it's parked at that floor — as close as the hardware can
get — so the gun still fires at its best achievable aim even when the
error can't be driven below `tolerance`. The achievable precision is
~`minSpeed / speedGain`°, so a tighter lock means a higher `speedGain`
(and, ultimately, a finer gear reduction to lower `degPerSecPerRpm`).
Set `lockWhenStalled = false` for a strict tolerance-only lock.

## Later

- Web interface for remote targeting (player or XYZ), Spruce-style.
- Multi-turret: one computer driving several mounts; rednet commander
  for ground artillery batteries.
