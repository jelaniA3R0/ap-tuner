# Contributing to AP Tuner

Thank you for helping. AP Tuner exists because people shared what they learned about tuning ArduPilot
aircraft; every log, bug report, test and line of code makes it better for the next builder.

## Ways to help (no programming needed)

* **Share a flight.** The model tuner for heavy drones has been proven on one airframe so far. Open an issue with
  the *Flight report* template: your airframe, what the app said, what you flew and how it flew. A `.bin` log
  (or a link to one) makes it much more useful; tell us if we may use it in the test set.
* **Report a bug** with the *Bug report* template: what you did, what you expected, what happened, the AP Tuner
  version (Help tab or `aptuner --version`) and, if possible, the log.
* **Suggest a feature** or improve the documentation (`README.md`, `docs/`).

## Safety first

AP Tuner proposes and can write autopilot settings, so a change can crash an aircraft. Contributions that touch
proposals, limits, the live tuner or parameter writing must:

* keep the existing safety behaviour (nothing written while armed unless the in-flight switch is on, verified
  writes with read-back and roll-back, UNDO files, the identity check of the vehicle);
* come with tests (`cpp/tests`) and, for tuning logic, a short note of what it was checked against
  (simulator, logs, flights);
* say plainly in the pull request what has *not* been tested.

Report a safety problem (a proposal that made an aircraft unstable, a write that went to the wrong vehicle, ...)
as described in [SECURITY.md](SECURITY.md).

## Building

Everything is C++20 with no external libraries; the user interface is plain HTML / JavaScript in `web/`.

* **Windows:** the portable toolchain described in `cpp/README.md` (llvm-mingw + CMake + Ninja), then
  `cpp\build.cmd` (builds and runs all tests). Visual Studio 2022 with the C++ workload also works.
* **Linux / macOS** (GCC 14 or newer, or Clang): `cmake -S cpp -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build && ctest --test-dir build`.
* **Run the app from the source tree:** `build\...\app\aptuner --open --web web` (or `aptuner --server` and open
  http://127.0.0.1:8090).
* **Real-log tests** need the tandem QuadPlane logs, which are not in the repository: they skip unless
  `APTUNER_LOG_DIR` points at them.
* **ArduPilot's simulator (SITL)** is the way to test live features without an aircraft: see "Building and
  testing" in the README.

## Pull requests

1. Fork the repository and make a branch (`fix/...`, `feature/...`).
2. Keep a pull request to one topic. Follow the style of the surrounding code (names, comment density, plain
   language in everything the user reads).
3. Build with warnings as errors (`-DAPTUNER_WARNINGS_AS_ERRORS=ON`) and run the tests; the GitHub Actions
   workflow builds with MSVC, GCC and Clang.
4. Update `CHANGELOG.md` (under "Unreleased") and the docs when behaviour changes.
5. Fill in the pull request template, including what you tested and what you could not test.

By contributing you agree that your contribution is licensed under the MIT licence of the project.

## Releases

The maintainer tags a version (`v0.3.0`, `v0.3.0-beta`, ...). The *release* workflow builds the Windows package
and attaches it to a GitHub release; the notes come from `CHANGELOG.md`.
