# AP Tuner

Live and log-based tuning of ArduPilot rate PIDs, angle gains and gyro notch filters, for
multirotors, QuadPlane VTOLs, helicopters and fixed-wing planes of any size.

It connects to the vehicle through Mission Planner's MAVLink mirror (or directly to a telemetry
link or a SITL), watches the vehicle fly, identifies the airframe's response, computes gains
with the same controller structure and rules ArduPilot uses, shows everything (graphs, margins,
predicted vs measured step response, diagnostics with a grade), and can upload the gains in
bounded steps with an oscillation watchdog that reverts automatically. The same analysis runs
on `.bin` logs, including harmonic-notch design from raw gyro data.

## Install or just run

* **Install (recommended):** double-click **`Install AP Tuner.cmd`**. It copies the app to your user
  folder, puts **AP Tuner** on the Desktop and in the Start Menu, and starts it. The app opens in its
  own window. Close the window to stop it. `Uninstall AP Tuner.cmd` removes it again.
* **Run without installing:** double-click **`AP Tuner.vbs`** (own window) or
  **`start AP Tuner.cmd`** (browser tab plus a console window).

Nothing else to install: the `node` folder holds the runtime, no internet is used, and the whole
folder is portable (copy it or the zip to any Windows PC).

## One guided flight (Flight tab, recommended)

Press **Start flight check**. The app connects, reads the vehicle, and then leads you through one flight:

1. **Take off** and hover in Stabilize, AltHold or PosHold (QuadPlane: QStabilize / QHover / QLoiter;
   plane: FBWA). The app waits until the vehicle is armed and in one of those modes.
2. **Check-up (30 s, nothing written).** Tap the sticks. You get the issues per axis in plain words
   (sluggish, bouncy, oscillating, noisy D, motors at the limit, filters) and the exact parameter changes
   proposed, with the margins and the predicted response.
3. **Tune in flight.** Press **Apply in flight** (or pick "Apply by itself" before starting). The gains
   go to the flying vehicle in steps of at most 1.5x, each watched 4 s for oscillation and reverted by
   itself if it trips; the app re-measures and repeats until the gains settle (at most 4 rounds per axis).
4. **Filters.** The harmonic notch is set through the vehicle: ESC RPM tracking if the ESCs report RPM,
   otherwise the autopilot's own 30 s FFT measurement (FFT-capable boards with FFT_ENABLE = 1). If
   neither exists you are told to set it from a log later. The gains of all chosen axes are then checked once
   more with the filter active.
5. **Overall check.** 30 s of normal flying with the final gains: grade per axis, vibration, hover
   throttle, notch state, and whether the gains could still move.
6. **Land and save.** Land and disarm. The summary compares the check-up with the final check, lists
   every write, and the parameter file is one click away.

Stop at any time; the Advanced tab keeps **Undo** per axis and **Restore original gains**.

## Telemetry + USB tab (vehicles on a telemetry radio)

A 57600 radio carries only about 15-50 PID samples a second, too few to tune roll and pitch in the air.
This tab uses the radio only to mark the time: fly a 30 s capture (it starts by itself once you hover in a
good mode), land, plug the USB cable into the autopilot, and the app downloads just the log head plus the
marked 30 s (a few MB, found by bisection on the log's timestamps), analyses it at the full logging rate,
and writes the ticked changes on the ground (UNDO file first, every value read back). Mission Planner can
stay connected over the radio with its mirror on. Code: src/engine/groundtune.js, src/mavlink/logdl.js,
web/ground.js. Developer hook: APT_TEST_USB=5763 makes a SITL TCP port stand in for the USB cable.

## Advanced tab: live tuning, step by step

1. **Connect.** Press **Connect**. The app looks for Mission Planner's MAVLink mirror, a simulator,
   and the USB / telemetry ports on this PC, and stops at the first vehicle that answers.
   With Mission Planner open: press **Ctrl+F**, click **Mavlink**, pick **TCP Host - 14550**, tick
   **Write access**, press Connect, OK the port box. Then press Connect in AP Tuner.
2. **Read the vehicle.** Reads the tuning parameters and shows a setup check. It asks before the two
   housekeeping values it needs (`GCS_PID_MASK` = 7 for PID telemetry, `*_SMAX` = 50 if it is 0).
3. **Tune.** Choose the feel (Smooth / Normal / Sporty), the axes, and whether the app only advises
   (default) or auto-applies. Press Start, fly in Stabilize / AltHold (planes: FBWA) with short sharp
   stick taps. After each 30 s window an axis card shows the grade, the measured and predicted step
   response, the margins and the proposed gains. Press **Apply** (it shows exactly what will be
   written) or let auto-apply do it. **Undo** and **Restore original gains** are always there.
4. **Notch filter.** Press **Check what is possible**: ESC RPM tracking (if the ESCs report RPM),
   the autopilot's own in-flight FFT measurement (Cube Orange / H7 boards: enable, reboot, hover,
   measure 30 s, the autopilot writes the notch itself), or the log route.
5. **Save.** Everything on the vehicle versus what it had when you connected, and a parameter file
   to download.

Telemetry rate matters: a copter's rate loop needs about 50 Hz of PID data for a trustworthy model
(USB, Wi-Fi/Ethernet, fast radios). A 57600 baud radio gives about 15 Hz: enough for diagnostics
and cautious proposals; use the Log file tab for the precise work.

## Log analysis

Log file tab: drop a `.bin` log (or pick one from the recent-logs list). You get the same per-axis
cards, the gyro-noise / notch section, and a **Download parameter file** button. Load that file in
Mission Planner: CONFIG > Full Parameter List > Load from file > Write Params.

For copters, log at the fast attitude rate (LOG_BITMASK bit "ATTITUDE_FAST" and "PID") so PIDR/PIDP/PIDY are
logged at the loop rate. For QuadPlanes the RATE message is used (300 Hz) when the PIQx messages are slower.
Planes log PIDR/PIDP at 25 Hz: enough for FF, I and the angle time constant, marginal for D.

### Notch filters

A notch design needs raw gyro data in the log: set `INS_RAW_LOG_OPT = 9` (H7 boards: pre- and
post-filter, first gyro only) or `INS_LOG_BAT_MASK = 1`, `INS_LOG_BAT_OPT = 4`, fly 1-2 minutes
including a steady hover, download the log. The app finds the motor peaks and harmonics,
recommends `INS_HNTCH_*` (throttle-based by default, ESC-telemetry mode when the log has RPM
data), shows the predicted post-filter spectrum and the phase lag at 10 Hz, and suggests the
`INS_GYRO_FILTER` you can afford once the notch is in. Without raw data it still finds peaks in
the filtered gyro and warns you.

## What the maths does (short version; details in `docs/how-it-works.md`)

* **Identification**: from rate target, actual rate and the rate-PID output it estimates the
  bare airframe frequency response (actuator → gyro) with a closed-loop-safe cross-spectral
  estimator and coherence, divides out the known gyro filter chain, and fits
  `G(s) = K·e^(−s·Td) / ((s + a)(τ·s + 1))`. Delay and lag are then calibrated by matching the
  model's closed loop (with the current gains) to the measured target→actual response.
* **Synthesis**: searches P and D (I tied by ArduPilot's rules: I = P for roll/pitch, I = 0.1·P
  yaw, I = FF for planes/heli) against ArduPilot's exact AC_PID structure (FLTT/FLTE/FLTD, the
  INS_GYRO_FILTER 2-pole filter, harmonic notches, loop rate) for the best disturbance rejection
  under gain margin, phase margin, sensitivity peak and overshoot limits (the preset), with a
  cap on D-term noise. Filters follow QuikTune (FLTT = FLTD = INS_GYRO_FILTER/2). Angle P / TCONST
  follow from the rate-loop crossover. Plane FF uses ArduPilot AUTOTUNE's event method.
* **Diagnostics**: tracking, oscillation band energy and peak, actuator saturation, D/P share,
  slew-rate limiter activity (SRate, Dmod), noise, trim (I term), graded A-F with explanations.

## Safety

Keep a hand on the mode switch, tune in calm air at a safe height, one axis at a time, with a
vehicle that already hovers (see the ArduPilot "setting up for tuning" page for MOT_THST_EXPO,
hover throttle, PWM ranges and INS_GYRO_FILTER by prop size). The app changes only the rate
PID / filter / angle-gain parameters of the chosen axes, `GCS_PID_MASK` and `*_SMAX`. ArduPilot
saves parameter writes immediately, so **Restore original gains** (or the journal in
`sessions\`) is the way back.

## Testing without a vehicle (SITL)

`sitl\run-sitl.cmd` starts an ArduCopter software-in-the-loop (Windows build from
firmware.ardupilot.org). `node sitl\pilot.js 5760 10` arms, takes off to 12 m and flies stick
taps for 10 minutes. Connect AP Tuner to `127.0.0.1:5762` (TCP). Mission Planner can watch on
5763. This is how the app was verified: a detuned SITL quad was identified, re-tuned axis by
axis with auto-apply, each step held under the watchdog, and the session journaled.

## Demonstrations

`demo/index.html` links the before/after demonstrations made with ArduPilot's simulator: for each vehicle
(a medium quadcopter and a medium quadplane VTOL) the same scripted stick sequence was flown with under-tuned
starting gains, then AP Tuner tuned the vehicle in a separate flight, and the sequence was flown again. Each
folder holds the autopilot's own logs (`before.BIN`, `tuning.BIN`, `after.BIN`), the parameter file the
app produced (`tuned.param`), a comparison report (`report.html`) and a side-by-side video (`.avi`).
`node tools/demo-run.js quad|quadplane` reproduces a demonstration with the app running.

The method itself was checked against the simulator: `tools/osc-experiment.js` raises roll P step by step
until the autopilot's own oscillation signal fires, and the gain at which that happened is compared with the
gain the app's model predicts (logs in `docs/validation/`, numbers in `docs/how-it-works.md`).

## Files

* `src/mavlink` MAVLink v2 parser/encoder generated from the dialect XML; `link.js` transport
  (TCP, UDP, serial through a PowerShell bridge), parameters, message intervals.
* `src/log/dfreader.js` DataFlash `.bin` reader.
* `src/dsp` FFT, Welch spectra / coherence, Wiener step-response estimate.
* `src/model` ArduPilot controller and filter transfer functions, plant model and fit,
  loop-shaping synthesis, tuning rules.
* `src/engine` live axis buffers and watchdog, the tuning session state machine (with a
  QuikTune-style ramp fallback), live notch setup (ESC RPM / in-flight FFT), log analysis, notch
  recommendation, parameter-file export.
* `web` the browser UI (uPlot for charts). `test` synthetic end-to-end test
  (`node test/synthetic.test.js`) and checks against real logs.
