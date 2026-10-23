# AP Tuner, C++ version

The C++ port of AP Tuner (the JavaScript version in `../src` stays the reference until the port gives the same
results on every test log). Plan and phases: `../docs/cpp-port-plan.md`.

## Build

Windows with the portable toolchain in `D:\toolchain` (llvm-mingw, CMake, Ninja):

```
cpp\build.cmd          release build + tests  ->  cpp\build\release
cpp\build.cmd debug    debug build + tests    ->  cpp\build\debug
```

Any platform with CMake 3.21+ and a C++20 compiler (MSVC 2022 or newer, GCC 14+, Clang 16+):

```
cmake -S cpp -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## Layout

| Folder | Contents |
|---|---|
| `core/` | `aptuner_core` library: math and analysis, no I/O |
| `io/` | `aptuner_io` library: event loop + C++20 coroutines, MAVLink, TCP / UDP / serial links, auto-connect, log download |
| `cli/` | `aptuner-analyze` (command-line log analysis) and `aptuner-link` (link check) |
| `app/` | `aptuner`: the application (live tuner, guided flight, ground tune, HTTP + SSE server, web UI from `../web`) |
| `tests/` | `aptuner-tests` (own minimal framework in `check.hpp`) and CTest smoke tests |
| `packaging/` | installer, uninstaller and read-me for the release zip |

## Release

```
powershell -ExecutionPolicy Bypass -File tools\make-release.ps1     builds, tests and writes ..\AP Tuner C++.zip
```

The zip holds the three programs, `web/`, `docs/`, the licence and `Install AP Tuner.cmd` (installs for the current
user to `%LOCALAPPDATA%\Programs\AP Tuner C++` with Desktop and Start Menu shortcuts, port 8092, so it runs next to
the JavaScript edition).

## How the port is checked

Each JavaScript module is ported to C++ with the same structure and names, then compared with golden results
produced by the JavaScript version (same inputs, relative error below 1e-6 for the math, identical grades and
proposed gains for whole logs). CI (`../.github/workflows/cpp.yml`) builds and tests on Windows (MSVC),
Linux (GCC) and macOS (Clang).

## Status

| Phase | Scope | State |
|---|---|---|
| 0 | toolchain, CMake skeleton, tests, CI | done |
| 1 | core math (fft, spectra, step response, controller, plant fit, synthesis, time-domain fit, rules) | done: golden tests |
| 2 | .bin reader, log analysis, notch, issues, analyze CLI | done: identical results on 9 logs (tools/compare-cpp.js) |
| 3 | MAVLink, links (TCP / UDP / native serial), auto-connect, log download | done: SITL link test (params, set + read-back, log download) |
| 4 | live tuner, guided flight, ground tune, HTTP + SSE server, web UI | done: SITL guided flight through the C++ server (grades B to A, verdict GOOD, gains in the JS demo's range) |
| 5 | validation flights, installer, release | installer + release zip done; flights on real aircraft and the GitHub release next |
