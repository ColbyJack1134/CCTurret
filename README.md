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
- `vendor/CCMinimap/computercraft/cfgutil.lua` → `cfgutil.lua`
- `vendor/CCMinimap/computercraft/heading.lua` → `heading.lua`

Run `cannon.lua`. First boot writes `cannon.cfg` with defaults — edit the
peripheral names, cannon position, and `yawOffset` to match your build,
then rerun.

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

Keys: `F` fire (manual pulse), `A` arm/disarm, `Q` quit. The turret
continuously tracks the selected player; `LOCKED` means both axes are
within `tolerance`. While **armed** (ARM button or `A`; disarmed by
default) the fire line is held high whenever the turret is locked on and
dropped the moment lock is lost — autocannon behavior; a pulse/reload
mode for regular cannons is planned.

Player targets aim at center mass (`getPlayerPos` returns the feet) and
open fire early: as soon as the shot would pass through a player-sized
box around the aim point (`playerHitbox`, default 0.6 × 1.8 — pad it if
too strict at your ranges), the line goes high while the turret keeps
converging on the exact aim. The status line shows the live horizontal
/ vertical miss until lock. `trackSeconds` (default 0.1, minimum 0.05 —
one game tick) sets the tracking loop period if you want faster aim
updates for more peripheral traffic.

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

If an axis spins away from the target, flip `invertYaw` / `invertPitch`
in the config instead of regearing.

## Later

- Ballistic pitch (projectile drop / muzzle velocity) instead of line-of-sight.
- GPS for cannon position instead of hardcoded coords.
- Web interface for remote targeting (player or XYZ), Spruce-style.
