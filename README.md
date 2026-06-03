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

Copy `cannon.lua` and `cfgutil.lua` to the computer and run `cannon.lua`.
First boot writes `cannon.cfg` with defaults — edit the peripheral names,
cannon position, and `yawOffset` to match your build, then rerun.

Keys: `F` fire, `Q` quit. The turret continuously tracks the nearest
non-whitelisted player; `LOCKED ON` means both axes are within `tolerance`.

If an axis spins away from the target, flip `invertYaw` / `invertPitch`
in the config instead of regearing.

## Later

- Ballistic pitch (projectile drop / muzzle velocity) instead of line-of-sight.
- GPS for cannon position instead of hardcoded coords.
- Web interface for remote targeting (player or XYZ), Spruce-style.
