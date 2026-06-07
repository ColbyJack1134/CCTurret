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
- **Redstone Relay** (CC:Tweaked) — one line: fire. (Assembly and clutch lines
  from earlier plans are unnecessary: the speed controllers hold at 0 RPM and
  the block reader gives absolute angles.)

## Usage

Shared libraries come from the CCMinimap submodule (`git submodule update
--init`) so there is exactly one tracked copy of each. Copy to the computer,
flat:

- `cannon.lua`
- `ballistics.lua`
- `vendor/CCMinimap/computercraft/cfgutil.lua` → `cfgutil.lua`
- `vendor/CCMinimap/computercraft/heading.lua` → `heading.lua`

Run `cannon.lua`. First boot writes `cannon.cfg` with defaults — edit the
peripheral names, cannon position, and `yawOffset` to match your build,
then rerun.

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

Keys: `F` fire (manual pulse), `A` arm/disarm, `C` enter an XYZ
target, `Q` quit. The turret
continuously tracks the selected player; `LOCKED` means both axes are
within `tolerance`. While **armed** (ARM button or `A`; disarmed by
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
  DEBUG tab).
- `projectile`: keys the constants table in `ballistics.lua` —
  big-cannon shells fall at −0.05 b/t², autocannon rounds and the
  mortar stone at −0.025, drag 0.99/tick for all (verified from CBC
  source; datapacks can override, so re-verify on a tuned server).
- `charges` (bigcannon): powder charges loaded — muzzle speed is
  2 b/t per charge. 5 charges = 200 b/s ≈ 693 blocks max range.
- `muzzleVelocity` (autocannon, blocks/sec): set by the cannon, not
  the round: `20 × (baseSpeed + perBarrel × min(barrels, cap))` —
  cast iron `20×(5 + 2×b≤2)`, bronze `20×(3 + 1.5×b≤3)`, steel
  `20×(3 + 1.5×b≤4)`; a full-length steel or cast-iron gun is 180.
  Fine-tune on a strafing target: shots trailing behind = value too
  high, leading in front = too low. Mind projectile lifetime: cast
  iron rounds despawn after ~99 blocks, bronze ~187, steel ~540.
  (Old configs: `lead.muzzleVelocity` migrates here automatically.)
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

Player targets lock onto the head (`getPlayerPos` reports ~head level;
`playerHitbox.aimOffset` shifts the setpoint if shots ride high or
low). Fire opens early: as soon as the shot would pass through the
body box anchored to the head — `playerHitbox.width` wide, `up` above
to `down` below the reported Y (default 0.6 wide, +0.2/−1.8) — the
line goes high while the turret keeps converging on the head. The status line shows the live horizontal
/ vertical miss until lock. `trackSeconds` (default 0.1, minimum 0.05 —
one game tick) sets the tracking loop period if you want faster aim
updates for more peripheral traffic.

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
re-gearing to re-measure.

If an axis spins away from the target, flip `invertYaw` / `invertPitch`
in the config instead of regearing.

## Later

- Web interface for remote targeting (player or XYZ), Spruce-style.
- Multi-turret: one computer driving several mounts; rednet commander
  for ground artillery batteries.
