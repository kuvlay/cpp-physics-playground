# C++ 2D Physics & Collision Playground

An interactive 2D simulation built with **C++17, raylib 5.5, and CMake**. Spawn circles, toggle gravity, and inspect how velocity and contact normals change as objects collide inside a bounded arena.

The project implements its own circle collision detection and response. raylib provides the window, input, and rendering; the physics code runs independently and can be tested without opening a window.

## Features

- Fixed 120 Hz physics timestep, separate from rendering, with bounded catch-up work.
- Equal-mass circle collisions using distance checks, overlap correction, and frictionless elastic impulse response.
- Radius-aware wall and corner bounces.
- Optional gravity using semi-implicit Euler integration.
- Interactive spawning with boundary checks, overlap rejection, and an 80-circle limit.
- Pause/resume, scene reset, and visual debugging with velocity arrows and sampled contact normals.
- Assertion-based checks for motion, collisions, conservation properties, gravity, and crowded simulations.

## Controls

| Input | Action |
| --- | --- |
| Left click | Spawn a circle in an unoccupied space inside the arena |
| P | Pause or resume physics |
| R | Restore the original four circles and default settings |
| G | Toggle gravity |
| C | Toggle velocity arrows and contact normals |
| Escape | Close the application |

Spawning also works while paused. White arrows show velocity; yellow arrows show contacts sampled during the most recent physics step. A contact may disappear before a rendered frame if another physics step has already cleared the sample.

## Build and run

### Windows

Requirements: a C/C++ compiler supporting C++17, CMake 3.24 or newer, Ninja, and Git. The first configuration downloads raylib 5.5; no separate raylib installation is needed.

From the repository root in PowerShell, with GCC/G++, CMake, Ninja, and Git on PATH:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
cmake --build build
.\build\collision_playground.exe
```

For a MinGW installation at `C:\mingw64`, add its tools to the current session first if necessary:

```powershell
$env:Path = 'C:\mingw64\bin;C:\Program Files\Git\cmd;' + $env:Path
```

Keep the MinGW runtime DLLs available on PATH when running the executable. Close the application before rebuilding it.

### Linux / WSL

Use a separate build directory; Windows and Linux build caches are not interchangeable. Linux requires raylib's desktop graphics development dependencies, and WSL needs a graphical session such as WSLg to display the window. See the [raylib Linux setup guide](https://github.com/raysan5/raylib/wiki/Working-on-GNU-Linux) for dependencies.

```bash
cmake -S . -B build-wsl -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
cmake --build build-wsl
./build-wsl/collision_playground
```

The current verification was performed with the native Windows build.

## How it works

```text
Input -> application state -> fixed physics updates -> render
                                  |
                                  v
                      Apply acceleration and move
                                  |
                           Resolve walls
                                  |
                      Detect and resolve circle pairs
                                  |
                        Resolve walls again
```

Each body stores position and velocity. The simulation uses a shared radius of 25 pixels and implicit unit mass. Positions are measured in pixels, velocity in pixels/second, and acceleration in pixels/second squared.

Elapsed frame time accumulates until a 1/120-second physics update is due. The application performs at most eight updates per rendered frame, then discards overdue whole steps to prevent unbounded catch-up work. Rendering targets 60 FPS; this is a pacing target, not a measured performance guarantee.

For each circle pair, a squared-distance test detects contact. Overlapping circles move apart equally along the contact normal. An impulse changes their normal velocities only when they are approaching; tangential velocity is preserved. Restitution is 1, so isolated collisions are elastic. The pair loop checks each unique pair once and has O(n²) cost.

## Project layout

| File | Responsibility |
| --- | --- |
| `src/main.cpp` | Input, spawning, application state, timestep accumulator, and drawing |
| `src/physics.h` | Vector operations, body/arena/contact data, and physics interface |
| `src/physics.cpp` | Integration, wall handling, and circle collision response |
| `tests/physics_checks.cpp` | Standalone physics assertions and stress checks |
| `CMakeLists.txt` | C++17 build and pinned raylib dependency |

## Tests

From the repository root on Windows:

```powershell
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Isrc tests/physics_checks.cpp src/physics.cpp -o build/physics_checks.exe
.\build\physics_checks.exe
```

On Linux, use the same compiler command with `-o build-wsl/physics_checks`, then run `./build-wsl/physics_checks`.

Do not define `NDEBUG`: these checks require assertions. The test source rejects builds that disable them.

The checks cover wall/corner bounces, head-on and glancing collisions, momentum and kinetic energy for isolated elastic pairs, separating overlaps, coincident centers, gravity, and debug-contact sampling. A stress check runs 80 circles for 7,200 steps with gravity off and again with gravity on, checking finite values, arena bounds, and unchanged physics results when contact recording is enabled. These are correctness checks, not performance benchmarks or automated UI tests.

## VS Code setup

CMake exports `compile_commands.json` so IntelliSense can use the same flags and include paths as the compiler. The checked-in configuration uses `build/` with the Windows `Win32` configuration and `build-wsl/` with the `Linux` configuration in a WSL window. Adjust the compiler path if your compiler is installed elsewhere.

After configuring CMake, save all editor files. If old diagnostics remain, run **C/C++: Reset IntelliSense Database** from the Command Palette, followed by **Developer: Reload Window**. Check that the active configuration matches the environment where VS Code is running.

## Limitations

- Discrete collision detection can miss fast-moving contacts between steps.
- A single contact pass can leave overlaps or jitter in dense groups; stable stacking is not implemented.
- Circles share mass and radius. There is no friction, rotation, or damping.
- Contact arrows show the last physics step rather than a persistent collision history.
- No spatial partitioning, FPS profiling dashboard, or performance optimization claims are included.
