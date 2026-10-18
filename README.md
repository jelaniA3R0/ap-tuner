# AP Tuner

Live and log-based tuning of ArduPilot rate PIDs, angle gains and gyro notch filters, for
multirotors, QuadPlane VTOLs, helicopters and fixed-wing planes of any size.

It connects to the vehicle through Mission Planner's MAVLink mirror (or directly to a telemetry
link or a SITL), watches the vehicle fly, identifies the airframe's response, computes gains
with the same controller structure and rules ArduPilot uses, shows everything (graphs, margins,
predicted vs measured step response, diagnostics with a grade), and writes the gains on the ground
(disarmed). Writing to a flying vehicle is **off by default**: the switch **Allow changes in flight** is
experimental and has been tested only in ArduPilot's simulator, never on a real aircraft. With it on, the
gains go to the vehicle in bounded steps, every change is watched and the app tries to undo it when the
vehicle starts to oscillate (see Safety). The same analysis runs on `.bin` logs, including harmonic-notch
design from raw gyro data.

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
3. **Tune in flight** (only with **Allow changes in flight** on; otherwise press **Finish**, land and apply
   the proposals on the ground). Press **Apply in flight** (or pick "Apply by itself" before starting). The
   gains go to the flying vehicle in steps of at most 1.5x, each change watched for oscillation; if the vehicle
   starts to oscillate the app pauses and undoes the change. The app re-measures and repeats until the gains
   settle (at most 4 rounds per axis).
4. **Filters** (also only with changes in flight on). The harmonic notch is set through the vehicle: ESC RPM
   tracking if the ESCs report RPM, otherwise the autopilot's own 30 s FFT measurement (FFT-capable boards with
   FFT_ENABLE = 1); every axis is watched for 10 s afterwards and the notch is switched back if the vehicle
   starts to oscillate. If neither route exists you are told to set it from a log later. The gains of all chosen
   axes are then checked once more with the filter active.
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
PID / filter / angle-gain parameters of the chosen axes, the notch parameters, `GCS_PID_MASK` and
`*_SMAX`. ArduPilot saves parameter writes immediately, so **Restore original gains** (or the journal
in `sessions\`) is the way back.

What protects the vehicle (2026-10-17 audit, R0 and R4):

* **Nothing is written while the vehicle is armed** unless **Allow changes in flight** is on (off on every
  start of the app; experimental, tested only in the simulator).
* With it on: one change at a time (a single write lock); **Pause** and **Stop** stop before the next
  parameter write, retries included. Before a change the vehicle must look calm.
* An **oscillation monitor** checks roll, pitch and yaw four times a second while a session runs, on the slew-rate
  signal ArduPilot's QuikTune uses (SRate above the limit) and Dmod below 0.9, sustained over more than half
  of 2 s. Every in-flight change (gains, filters, the notch, notch values the autopilot writes itself) is watched
  afterwards on the axes it affects (4 s; notch and filters 10 s), with an attitude-error check where the vehicle
  sends its attitude target. A watch fails when the PID data stop, come in too sparse or the link drops.
* On a trip the session pauses at once and the change is undone from evidence: when the 2 s before the change
  were calm, back to the values just before it; otherwise, or if it still oscillates, back to the values the
  session started with. No corrective write is larger than 1.5x, and the result is checked again. The message
  says "oscillating" and what was written back; if nothing helps it says "land now".
* An undo can fail (for example when the link is lost): watch the vehicle, not the app.
* The local web interface only obeys its own page: requests must come from this PC's app window (Host / Origin
  check), anything that changes something must be a POST with a random per-run token from the page, and text
  from logs or the vehicle is shown as text, never as HTML.

How a value is written (2026-10-17 audit, R3):

* **Only to the autopilot you are connected to.** The app talks to one ArduPilot autopilot and knows it by its
  system id, its board ID and its firmware (shown on the Vehicle card). Answers from anything else on the link are
  ignored and another vehicle is reported. A reconnect or another vehicle stops what was in progress, and the
  changes worked out for the old connection have to be worked out again.
* **Every value is checked.** The value is sent the way ArduPilot stores it (whole-number parameters rounded) and
  the autopilot's answer must match exactly; without an answer, or with another value, it is read again. Each
  row then says what the autopilot holds: written and read back, refused (it kept the old value), a different
  value, or uncertain (no answer: check it before flying).
* **A set is all or nothing.** The values are read again first (if one changed since you looked, nothing is
  written), the UNDO file is saved before the first write, and if one value does not hold, the values already
  written go back to their values from before (each one read back).
* **Your OK is for the exact list you saw.** The dialog shows every value, why it changes, which ones act only
  after a reboot, and the list's id and fingerprint; the app writes that list, to that vehicle, or nothing.
* **Values that act only after a reboot** (notch reference, harmonics, options, the FFT switch) are never written
  in flight. On the ground the app offers **Reboot** (disarmed only), waits for the autopilot to come back, checks
  that it is the same one and reads every value again.
* **The write journal** `sessions\journal\<start time>.jsonl` gets one line for every vehicle, proposal, OK,
  write, roll-back, reboot and export, and is never rewritten. If it cannot be written (disk full), the app makes
  no new change until it is restarted (it still undoes a change that made the vehicle oscillate).
* **Parameter files** say which vehicle they are for on their first lines, keep every value exact for Mission
  Planner and mark the values that act after a reboot; a value ArduPilot does not accept is left out with a note,
  and a value outside the documented range gets a note.

## Testing without a vehicle (SITL)

`sitl\run-sitl.cmd` starts an ArduCopter software-in-the-loop (Windows build from
firmware.ardupilot.org). `node sitl\pilot.js 5760 10` arms, takes off to 12 m and flies stick
taps for 10 minutes. Connect AP Tuner to `127.0.0.1:5762` (TCP). Mission Planner can watch on
5763. This is how the app was verified: a detuned SITL quad was identified, re-tuned axis by
axis with auto-apply, each step held under the watchdog, and the session journaled.

Safety regression tests: `node tools/sitl-f01.js --port <app port>` repeats the audit's F-01 case (the notch
switched on in flight with high pitch gains; the app must switch it back and pause within 5 s of the onset),
`node tools/flight-demo.js --port <app port>` flies a whole guided session and lists every pause and trip, and
`node tools/osc-episodes.js <log.BIN>` lists the sustained oscillations in the simulator's own log.
`node tools/sitl-ground-write.js --cpp` writes on the ground to a fresh simulator, disarmed, in both editions:
four gains written, read back and undone, a value the autopilot refuses, a whole-number value, and the notch
with its reboot, Undo and second reboot (results in `sitl\run-ground-write\`). Unit tests: `npm test`
(JavaScript, including fake vehicles with 26 safety and 41 write-verification scenarios, the API security
checks and the page-escaping check) and `cpp\build.cmd` (C++, the same scenarios).

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
  recommendation, parameter-file export; `paramwrite.js` verified writes, transactions and the reboot
  workflow, `identity.js` the vehicle identity and proposal fingerprints, `journal.js` the write journal.
* `src/model/parammeta.js` parameter types, ranges and the values that act only after a reboot.
* `web` the browser UI (uPlot for charts). `test` synthetic end-to-end test
  (`node test/synthetic.test.js`) and checks against real logs.
