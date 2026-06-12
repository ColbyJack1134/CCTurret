-- shipaim_test.lua: the ship-target hull geometry in ballistics.lua --
-- random aim-point sampling (sphere + heading-oriented ellipsoid) and
-- hull membership (hullNorm). Run from the repo root:
--   lua tests/shipaim_test.lua
-- Deterministic: a seeded LCG stands in for math.random.

local B = dofile("ballistics.lua")

local failures = 0
local function check(ok, msg)
  if not ok then
    failures = failures + 1
    print("FAIL: " .. msg)
  end
end

local function lcg(seed)
  local s = seed
  return function()
    s = (s * 1103515245 + 12345) % 2147483648
    return s / 2147483648
  end
end

-- ---------------------------------------------------------------- shipFrame
-- Compass convention: 0 = north = -Z, 90 = east = +X.
local fx, fz = B.shipFrame(0)
check(math.abs(fx) < 1e-9 and math.abs(fz + 1) < 1e-9,
  ("shipFrame(0) forward = north (-Z), got %g,%g"):format(fx, fz))
fx, fz = B.shipFrame(90)
check(math.abs(fx - 1) < 1e-9 and math.abs(fz) < 1e-9,
  ("shipFrame(90) forward = east (+X), got %g,%g"):format(fx, fz))
local _, _, rx, rz = B.shipFrame(0)
check(math.abs(rx - 1) < 1e-9 and math.abs(rz) < 1e-9,
  "shipFrame(0) right = east (+X)")

-- ----------------------------------------------------------------- hullNorm
local shape = { r = 4, l = 4, w = 2, t = 1.5, avoid = 1 }

-- Sphere (no heading): plain radial distance over r.
check(math.abs(B.hullNorm(4, 0, 0, shape, nil) - 1) < 1e-9,
  "sphere: |(4,0,0)|/4 = 1")
check(B.hullNorm(0, 3, 0, shape, nil) < 1, "sphere: (0,3,0) inside r=4")

-- Ellipsoid, heading 90 (east): long axis along X, width along Z.
check(math.abs(B.hullNorm(4, 0, 0, shape, 90) - 1) < 1e-9,
  "ellipsoid hdg 90: (4,0,0) on the long-axis rim")
check(B.hullNorm(0, 0, 3, shape, 90) > 1,
  "ellipsoid hdg 90: (0,0,3) outside the w=2 beam")
check(B.hullNorm(0, 1.6, 0, shape, 90) > 1,
  "ellipsoid: (0,1.6,0) above the t=1.5 deck")
check(B.hullNorm(0, 1.4, 0, shape, 90) < 1,
  "ellipsoid: (0,1.4,0) inside vertically")
-- Heading 0 (north = -Z): long axis along Z now.
check(B.hullNorm(0, 0, -3.9, shape, 0) < 1,
  "ellipsoid hdg 0: (0,0,-3.9) inside along the keel")
check(B.hullNorm(3, 0, 0, shape, 0) > 1,
  "ellipsoid hdg 0: (3,0,0) outside the beam")

-- ------------------------------------------------------------ sampleHullAim
-- Sphere sampling: inside r, outside avoid, spread over all octants.
local rand = lcg(20260612)
local sphereShape = { r = 4, avoid = 1 }
local octants = {}
local N = 4000
for _ = 1, N do
  local dx, dy, dz = B.sampleHullAim(sphereShape, nil, rand)
  local d = math.sqrt(dx * dx + dy * dy + dz * dz)
  check(d <= 4 + 1e-9, ("sphere sample escaped: |%g,%g,%g| = %g"):format(
    dx, dy, dz, d))
  check(d >= 1 - 1e-9, ("sphere sample inside avoid: %g"):format(d))
  local key = (dx >= 0 and "+" or "-") .. (dy >= 0 and "+" or "-")
    .. (dz >= 0 and "+" or "-")
  octants[key] = (octants[key] or 0) + 1
end
local nOct = 0
for _ in pairs(octants) do nOct = nOct + 1 end
check(nOct == 8, ("sphere samples cover 8 octants, got %d"):format(nOct))
-- Uniform-ish: no octant under 1/16 of the samples.
for k, n in pairs(octants) do
  check(n > N / 16, ("octant %s starved: %d of %d"):format(k, n, N))
end

-- Ellipsoid sampling, heading 90: inside the shape, outside avoid, and
-- actually USING the long axis (some samples beyond the 2-block beam).
rand = lcg(424242)
local maxX, maxY, maxZ = 0, 0, 0
for _ = 1, N do
  local dx, dy, dz = B.sampleHullAim(shape, 90, rand)
  check(B.hullNorm(dx, dy, dz, shape, 90) <= 1 + 1e-9,
    ("ellipsoid sample escaped: %g,%g,%g"):format(dx, dy, dz))
  check(dx * dx + dy * dy + dz * dz >= shape.avoid ^ 2 - 1e-9,
    "ellipsoid sample inside avoid bubble")
  maxX = math.max(maxX, math.abs(dx))
  maxY = math.max(maxY, math.abs(dy))
  maxZ = math.max(maxZ, math.abs(dz))
end
check(maxX > 3, ("long axis used: max |x| %g > 3"):format(maxX))
check(maxZ < 2 + 1e-9, ("beam respected: max |z| %g <= 2"):format(maxZ))
check(maxY < 1.5 + 1e-9, ("height respected: max |y| %g <= 1.5"):format(maxY))

-- Pathological config (avoid swallows the shape): falls back, terminates.
local dx, dy, dz = B.sampleHullAim({ r = 0.5, avoid = 1 }, nil, lcg(7))
check(dy == -2 and dx == 0 and dz == 0,
  "avoid-swallows-shape falls back below the bubble")

if failures == 0 then
  print(("shipaim_test: all checks passed (%d samples x2 shapes)"):format(N))
else
  error(("shipaim_test: %d FAILURES"):format(failures), 0)
end
