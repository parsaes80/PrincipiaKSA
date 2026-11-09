# PrincipiaKSA - Realistic N-Body Gravity for KSA

This mod replaces the game's simplified gravity model
and replaces it with N-body simulation. Such a mod called Principia(https://github.com/mockingbirdnest/Principia) existed for KSP and now repo has 
ported it to KSA. N-body gravity offers much more depth to game and is very fun for people who have mastered the games own physics.

<img width="2265" height="1262" alt="Image" src="https://github.com/user-attachments/assets/27c48cb4-4580-4553-bd64-bb0cc15a01af" />

## What it does

- **Planets and moons**: move under Principia's n-body integration phsyics. They are no longer on Kepler's rails and move more accurattly to real-life.
- **Vessels**: vessels follow Principia's predicted path in space according to N-Body gravity and free from Kepler's rails, but other parts of the physics are untoched.
- **Predictions**: Instead of fixed orbits Principia integrates the path of the vessel through time, and this shows up as a prediction of where you will be,
    this path is shown in orange. As in Principia, how far it reaches is set by its number of integration steps and its tolerance, not by a time:
    the PrincipiaKSA window has Tolerance and Steps for the prediction and for the flight plan (its steps are per coast between burns). 
- **Maneuver nodes**: Principia of fully compatible with KSA's own manuver nodes and they work the same way as the base game.
- **Spheres of influence**: Since N-body gravity doesn't use SOI's they are only used for maneuver nodes and making sure they are in the right frame.
- **Auto-burn**: Auto burn and flight plans also work to a high level of accuracy with Principia's physics.
- **Ground Track**: Fully works with Principia
- **Transfer Planner**: Still uses KSA's own Kepler physics so the plans won't be accurate, 
    but they are close to Principia's path and part of the fun is fine tuning you approch to other celectials using the manuver node arrows.
- **Saves**: Fully compatible with KSA's own save files and you can load a save with Principia on and off.

## Install (players)

**REQUIREMENT**: A CPU with AVX and FMA (Intel Haswell / AMD Zen or later).

**Option 1 (Recommended)**: Install the mod using Borea, KSA's mod manager tool.

**Option 2**: 
2. Download `PrincipiaKSA.zip` from the
   [releases](https://github.com/parsaes80/PrincipiaKSA/releases) and unzip
   its `PrincipiaKSA` folder into KSA's mods folder:
   - Linux: `~/Documents/My Games/Kitten Space Agency/mods/`
   - Windows: `%USERPROFILE%\Documents\My Games\Kitten Space Agency\mods\`

3. Add to `manifest.toml`, in the folder above the mods folder:
   ```toml
   [[mods]]
   id = "PrincipiaKSA"
   enabled = true
   ```
4. Downlaod StarMap, point StarMap's path in the settings file to the KSA install dir. Start KSA through By running StarMap.

The same package runs on Linux and Windows (x64).

## Layout

| Path | What |
|---|---|
| `src/PrincipiaKSA/` | C# mod: StarMap entry point, Harmony patches (`Patches/`), drivers for bodies and vessels, predictor, spheres of influence, maneuver-node anchors, map overlay and window |
| `native/` | The C API (`principia_ksa.h`) over Principia's C++ core, its smoke test (`test/`), and `tools/sol_reference.cpp` (the JPL check above) |
| `extern/` | Principia and the dependencies it needs (abseil, protobuf, re2, gipfeli, CORE-MATH, zfp, Boost's config and multiprecision), as git submodules pinned to the commits this mod is built and tested with |
| `build.zig` | The whole build, with Zig alone: Principia, its dependencies, protoc, the native library for Linux and Windows, the C# mod, the package |
| `tools/` | The build's helpers in Zig: the zip writer and the installer/launcher |
| `packaging/` | What the package adds: the players' README, licences, the Windows test script |

## Requirements (building)

- [Zig](https://ziglang.org) 0.17. It is the only C/C++ toolchain: it builds
  both platforms' native libraries on either platform (clang 22, libc++,
  glibc 2.28 for Linux, MinGW-w64 for Windows). No CMake(I HATE CMAKE), MSVC or Visual Studio.
- .NET 10 SDK, for the C# part of the mod.

## Build

```bash
git clone --recursive https://github.com/parsaes80/PrincipiaKSA.git
cd PrincipiaKSA
zig build
```

The first build compiles Principia and its dependencies twice, for Linux
and for Windows, so it takes a while to compile. Later builds recompile
only what changed. Zig keeps its cache per compiler version, so switching
between two Zigs (say, the system's and VS Code's own) rebuilds everything
once. The output is in `zig-out/`:

| Path | What |
|---|---|
| `zig-out/PrincipiaKSA/` | The mod folder, ready to copy into KSA's mods folder: `PrincipiaKSA.dll`, `mod.toml`, the native library for Linux (`libprincipia_ksa_AVX.so`) and Windows (`principia_ksa_AVX.dll`), Principia's gravity model, `README.txt`, `VERSION`, `licenses/` |
| `zig-out/PrincipiaKSA.zip` | The same folder, zipped, for a release |
| `zig-out/windows-test/` | The native smoke test for Windows: run `run_smoke_test.bat` there, which writes `smoke_test_output.txt` |

Other steps (`zig build --help` lists them, with the options):

```bash
zig build test            # the native smoke test, on this machine
zig build game            # build, and install the mod where KSA loads it
zig build run             # ... and start KSA through StarMap (-wayland on Linux)
zig build run -Dcs        # build only the C# mod, keep the installed native library
zig build sol-reference   # the JPL check behind the μ convention
zig build update          # move extern/ to the latest Principia (below)
zig build -p <folder>     # output somewhere other than zig-out/
```

## Licence

PrincipiaKSA is under the [MIT licence](LICENSE). Principia is by the
Principia authors, also under the MIT licence; the package's `licenses/`
holds Principia's and those of the libraries in the native library (abseil,
protobuf, re2, gipfeli, zfp, Boost, CORE-MATH, LLVM's libc++ and, for
Windows, the MinGW-w64 runtime).

## Not Tested Yet

Keep in mind this is an early project in development, and has not been thoroughly tested.

- Docking is untested: vessels sharing a physics bubble run in KSA's full
  physics, coasting ones on Principia's integration and joined ones on KSA's
  with the n-body correction.
- In a rotating plotting frame (two-body rotating, body surface), a node's
  arrows are about its body in fixed axes, not along the rotating line.
- After an encounter KSA's conics get wrong, the hover marker's colour and
  readouts, and the right-click menu's circularize and plane-match Δv, are
  still KSA's.
- Each update recomputes the prediction, the flight plan and the nodes'
  tracks from the vessel's state, also where they share their path.
- A rare error in preparing a burn for the flight plan ("divide by zero") is
  caught: that burn is left out of the plan and logged once, with its
  details; the cause is not yet found.
