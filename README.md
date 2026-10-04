# C++ Collision Playground — Stage 3

Four equal-radius circles move and bounce inside a rectangular arena below
our title. The 1000 x 700 window keeps Stage 1's dark background and targets
60 rendering FPS. Escape or the window's X closes it. Circles now collide
with each other as equal-mass, frictionless, elastic circles. Blue and orange
start 240 pixels apart and approach at 150 pixels/second: their 50-pixel
combined radius gives a first contact after about (240 - 50) / 150 = 1.27 seconds.

## Build and run (PowerShell)

Use the existing MinGW tools. This PATH change affects only this PowerShell
session and programs launched from it. The first configure downloads pinned
raylib 5.5 from GitHub and needs internet.

```powershell
Set-Location 'C:\Users\Aarish\Desktop\cpp-physics-playground'
$env:Path = 'C:\mingw64\bin;C:\Program Files\Git\cmd;' + $env:Path

& 'C:\mingw64\bin\cmake.exe' -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER=C:/mingw64/bin/gcc.exe -DCMAKE_CXX_COMPILER=C:/mingw64/bin/g++.exe -DCMAKE_MAKE_PROGRAM=C:/mingw64/bin/ninja.exe
& 'C:\mingw64\bin\cmake.exe' --build build
& '.\build\collision_playground.exe'
```

Close the application before rebuilding. Ninja rebuilds only changed files.
raylib is linked statically; MinGW runtime DLLs still come from the toolchain
folder on PATH. CMake prepares the build, Ninja executes it, GCC compiles
raylib's C, and G++ compiles/links our C++17 application.

## Small file structure

- `src/main.cpp`: window, explicit initial bodies, colors, accumulator, drawing.
- `src/physics.h`: Vec2 arithmetic, Body, Arena, UpdatePhysics declaration.
- `src/physics.cpp`: movement, radius-aware walls, and circle-pair response.
- `CMakeLists.txt`: builds both application sources and links raylib.
- `tests/physics_checks.cpp`: lightweight assertions without raylib or a framework.
- `build/`: ignored generated files and fetched dependencies.

Physics knows nothing about raylib or colors. Body stores only position and
velocity. All circles share radius 25 pixels and implicit unit mass.

## One frame

1. `GetFrameTime()` reports elapsed seconds; add them to the accumulator.
2. While at least 1/120 second remains, run UpdatePhysics and subtract that
   time. Stop after eight updates at most.
3. If the cap was reached, `std::fmod` discards remaining whole steps and
   keeps the fractional remainder. This bounds work after a stall. Dropped
   time is never simulated: under overload motion falls behind wall time.
4. Draw the latest positions once, then let EndDrawing present and pace.

The accumulator uses double for time bookkeeping; Vec2 and the physics dt
use float. SetTargetFPS(60) is a rendering pacing request, not a physics
clock or guarantee. Normally one second supplies approximately 120 fixed
updates: a 1/60-second frame can supply two, while a 1/240-second frame with
an empty accumulator supplies none and leaves time for the following frame.
Drawing only reads body data; UpdatePhysics changes it. There is no render
interpolation, so some high-FPS frames repeat a position.

## Motion and wall handling

`position += velocity * dt` converts pixels/second into pixels. For example,
(120, 60) moves (1, 0.5) pixels per fixed step. Screen x grows rightward and
y grows downward, so negative y velocity moves upward.

The arena spans x=40..960 and y=130..660. Radius 25 means centers must stay
within x=65..935 and y=155..635. A center at x=934 with vx=240 advances to
936, then clamps to 935 and changes vx to -240. Its y velocity is unchanged.
Independent x and y checks handle corners. A sign flip preserves speed and
models a perfectly elastic bounce. Only outward velocity is reversed, so
correcting an outside body already moving inward does not send it back out.

Clamping removes penetration instead of leaving the circle outside. This
simple discrete response discards overshoot; it does not calculate exact
impact time or multiple wall hits in one step. Fixed steps give consistent
step sizes, but cannot guarantee accuracy at arbitrary speeds or stability
for more complex collision rules. There is no gravity or acceleration.

## Circle collision algorithm

Each `UpdatePhysics` call moves all bodies, resolves walls, resolves each
unique circle pair once, and rechecks walls because pair separation can
push circles outside. Rendering then reads the latest positions.

For each `i < j` pair, `ResolvePair` computes `delta = B.position - A.position`
and compares `Dot(delta, delta)` with `(2 * radius)^2`. Non-contacts return
without a square root. Touching or overlapping pairs compute distance and
the unit contact normal `n = delta / distance`, pointing from A to B.
At distances at most 0.0001 pixels, a deterministic `(1, 0)` fallback avoids
normalizing a zero or tiny vector: lower-index A moves left, B right.
The fallback is an arbitrary direction for degenerate geometry and can leave
a sub-pixel residual overlap for nearly coincident centers.

Penetration is `2 * radius - distance`. Each body moves half that amount in
opposite directions along n. This position repair happens even if velocities
are already separating. Equal correction keeps the pair's midpoint unchanged.

Next, `vn = Dot(B.velocity - A.velocity, n)` measures approach along n.
Negative means closing, zero means no normal approach, positive means separating.
Only for `vn < 0`, apply `j = -(1 + restitution) * vn / 2`, then
`A.velocity -= n * j` and `B.velocity += n * j`. The denominator is the sum
of inverse unit masses. The named `restitution` constant starts at 1.0:
an isolated elastic pair preserves total momentum and kinetic energy (within
floating-point error). Restitution 0 removes relative normal separation speed;
intermediate values lose some kinetic energy. Wall bounces stay elastic.
Tangential velocity is unchanged because frictionless contact applies no
impulse along the tangent.

For a horizontal contact with A=(100,0), B=(-50,0), n=(1,0), restitution=1:
`vn = -150`, `j = 150`, so A becomes (-50,0) and B becomes (100,0).
Only the normal components exchange; angled collisions need the dot product.
The `i < j` loops avoid self-collisions and duplicate pairs: n(n-1)/2 tests,
or O(n^2) time and O(1) extra storage. Four circles require six pair tests.

### Limitations

- Discrete detection can miss sufficiently fast collisions (tunneling).
- One pairwise pass can leave residual overlaps in dense groups or near walls;
  later corrections can disturb earlier contacts. Dense stacking is not solved.
- Equal radii and masses only; no friction or rotation.
- A fixed timestep does not eliminate tunneling or unstable dense contacts.
- No continuous collision detection or iterative solver is used.

## Lightweight physics checks

From the same PowerShell session:

```powershell
& 'C:\mingw64\bin\g++.exe' -std=c++17 -O0 -g -UNDEBUG -Wall -Wextra -Wpedantic -Isrc tests/physics_checks.cpp src/physics.cpp -o build/physics_checks.exe
& '.\build\physics_checks.exe'
```

Assertions are enabled explicitly; the source rejects builds with `NDEBUG`.
Success prints `Stage 3 physics checks passed.` Tests cover non-contact pairs,
head-on exchange, oblique tangential preservation, isolated-pair momentum and
energy conservation, separating overlaps, coincident and nearly coincident
centers, the final wall recheck, and the initial demonstration collision.
Contact-only cases use dt=0 to isolate response; the demonstration uses 1/120.
Component tolerances are 0.001; energy tolerance is 0.00001 times initial energy.
The existing assertions still cover movement, all walls, radius limits, all corners,
inward velocity during correction, speed preservation, and identical
results from identical initial data over identical fixed-step counts.
This repeatability check is for this executable, not a promise of bitwise
identical floating-point results on every platform.

## Manual visual and rendering-rate checks

- Run and confirm four distinct colors, the Stage 3 label, the arena below
  the title, and moving circles that stay inside and collide with each other.
- Blue and orange should meet after about 1.27 seconds; blue then moves left
  at 50 pixels/second and orange right at 100 pixels/second.
- Watch wall and corner bounces. Confirm Escape closes the window; restart
  and check the X button too.
- Temporarily change only SetTargetFPS(60) to 30, rebuild, and restart.
  Then repeat at 60 and 144. The first blue/orange collision should occur
  at approximately the same time at every target, provided the catch-up cap
  is not reached. Motion can look
  less smooth at 30 FPS and repeat positions at 144 FPS.
- Restore SetTargetFPS(60), rebuild, and rerun. Actual frame timing varies;
  this is an approximate real-time comparison, not pixel-exact timing.

Stage 3 ends here.
