# Changelog

All notable changes. Versions follow [semantic versioning](https://semver.org/); `-beta` releases are for testing.

## Unreleased

## 0.2.0-beta (2026-10-23)

First public release.

### Added
* **Model tune** tab for heavy and slow multirotors and QuadPlanes (the method of a hand tune of a ~100 kg tandem
  QuadPlane): hover detection, plain-language diagnostics (torque imbalance, yaw authority, centre of gravity from the
  thrust split, cascade oscillation, vibration, hover throttle, altitude hold, compass), identification as a range of
  plants (yaw windowed regression, roll / pitch from the oscillation), proof by replaying the real flights, robust
  design over the range (margins, noise, time simulations, stress tests, limits from physics, one gyro filter for all
  axes), steps with exact values, parameter and UNDO files, a test card, and a next-flight check against the saved plan.
* Live recording through Mission Planner's MAVLink mirror (found automatically): every flight from arming to disarming
  is analysed after landing; live attitude, telemetry health per stream.
* "Check my values": typed values are re-judged on the model at once.
* Write a step to the drone from the app: disarmed, the same system id, the drone's current values shown first, UNDO
  file, verified write with roll-back, reboot workflow when the gyro filter changes.
* `aptuner-heavy` command-line tool.
* Tests: the axis model against the hand analysis, a simulated drone's telemetry end to end, every parameter file
  checked against the float32 the autopilot stores, the tandem logs as acceptance tests (with `APTUNER_LOG_DIR`).

### Changed
* The app is the C++ edition only; the JavaScript edition was removed (it is in the git history up to `35bc01c`).
* The parameter checker knows the acceleration and rate limits, `INPUT_TC` and the pilot yaw rate.
* Builds without warnings on Clang and GCC (warnings as errors).

### Known limitations
See "Status and limitations" in the README: proven on one airframe, live recording and in-app writing tested on
simulated telemetry and ArduPilot's simulator only.
