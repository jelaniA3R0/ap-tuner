# AP Tuner: C++ port plan (2026-10-17)

Goal: an open-source, cross-platform tuner (Windows / Linux / macOS) for ArduPilot vehicles from 200 g to 500 kg,
as one small program with no runtime to install. The JavaScript version stays as the reference until the C++
version gives the same results on every test log.

## Toolchain

Either one works; the code builds with both (and with GCC/Clang on Linux/macOS):

- **A. Visual Studio 2022 C++ part (MSVC).** Needs ~7-8 GB free on C:. C: has 0.5 GB free now.
  Visual Studio Installer -> Modify (Community 2022) -> Workloads -> tick **Desktop development with C++**
  (keep MSVC v143, Windows 11 SDK, C++ CMake tools) -> Installation locations: move the download cache to D:
  -> Modify.
- **B. Portable compiler on D: (recommended here).** llvm-mingw or WinLibs GCC (~1 GB zip) + portable CMake
  and Ninja, unzipped to `D:\toolchain`. No admin rights, nothing on C:.

## Architecture

```
aptuner/
  core/      C++20 library, no I/O: fft, spectra, step response, controller (AC_PID + filters + notch),
             plant fit, time-domain fit, synthesis, rules, notch design, issues
  io/        .bin log reader, MAVLink (generated from defs.json), links: TCP, UDP, serial (native Win32 /
             POSIX termios, replacing the PowerShell serial bridge), log download over MAVLink
  app/       tuner, guided flight, ground tune, live notch, HTTP + SSE server that serves the existing web/ UI
  cli/       aptuner-analyze <log.bin> (same output as tools/analyze-log.js)
  web/       unchanged HTML/CSS/JS user interface
  tests/     golden-result tests against the JavaScript version
```

Dependencies: header-only, permissively licensed, vendored in the repo: cpp-httplib (MIT) for HTTP,
nlohmann/json (MIT) for JSON. FFT, log reader, MAVLink and all math are our own code (already written once
in JS). Build: CMake. Licence for the release: GPL-3.0 (same as ArduPilot) or MIT, the owner's choice.

## Phases

| Phase | Work | JS source (lines) | Done when |
|---|---|---|---|
| 0 | Toolchain, repo, CMake skeleton, CI (GitHub Actions: Windows, Linux, macOS) | - | empty app builds on all three |
| 1 | Core math: fft, spectra, stepresp, controller, plant + Nelder-Mead, synth, timeid, rules | ~1,100 | unit tests match JS golden files (relative error < 1e-6) |
| 2 | .bin reader, log analysis, notch, issues, param file, analyze CLI | ~800 | tandem logs + 6 SITL demo logs give the same grades, gains and notch as JS |
| 3 | MAVLink codegen, TCP/UDP/native serial links, auto-connect, log download | ~700 | connects to SITL and through the Mission Planner mirror; real radio test |
| 4 | Live tuner, guided flight, ground tune, live notch, HTTP/SSE server, web UI | ~1,800 + UI | full SITL guided-flight demo matches the JS demo |
| 5 | Validation flights (tools/osc-experiment equivalents), installer, single-file release | - | release v1.0 on GitHub |

About 5,300 lines of JavaScript become roughly 8,000-10,000 lines of C++.

## After the port: the universal tuner (200 g - 500 kg)

- Designed excitation (ArduPilot SystemID chirps) and multi-axis frequency-domain identification:
  the method used in professional flight test.
- Vehicle classes with their own gain limits, loop rates and safety rules (micro, standard, heavy-lift,
  very heavy); the heavy end never auto-applies, only proposes and checks.
- Structural / flexible modes in the model for large frames (notch and gain limits around them).
- Coupled axes (tandem, tilt-rotor, tailsitter) and the plane rate loops.
- An open validation set: logs from many airframes with known good tunes, run on every release.
