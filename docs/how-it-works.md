# How AP Tuner works

This is the engineering behind the app: what was taken from the ArduPilot code base and the way
its developers tune, and what the app computes. File names refer to `src/`.

## 1. What ArduPilot does (from the source)

**Rate PID (`libraries/AC_PID/AC_PID.cpp`)**, used by Copter, QuadPlane VTOL, Heli and Plane:

```
target_f  += alpha(FLTT) * (target - target_f)          # first-order LPF on the target
error      = target_f - measurement
error_f   += alpha(FLTE) * (error - error_f)            # LPF on the error
deriv     += alpha(FLTD) * ((error_f - error_f_last)/dt - deriv)   # D on the filtered error, LPF
I         += error_f * kI * dt   (limited to IMAX, frozen when the output saturates)
out        = P*error_f + D*deriv + I   (P and D scaled by the slew limiter "Dmod")
FF         = kFF * target_f ;  DFF = kD_FF * d(target_f)/dt
alpha(f)   = dt / (dt + 1/(2*pi*f)),  f <= 0 means no filter
```

Copter/VTOL rate loops run in **rad/s** with outputs in mixer units (−1..1). Plane runs the same
object with scaled inputs: target and measurement are multiplied by `scaler²` (airspeed scaler
`SCALING_SPEED / airspeed`, clamped 0.5..2), FF by `scaler/EAS2TAS`; the output is a surface
deflection in degrees (45 = full). The logged / telemetered target and actual are deg/s for
Plane, rad/s for Copter (`PID_TUNING`, `PIDR/PIDP/PIDY`, `PIQR/PIQP/PIQY`).

**Gyro chain (`AP_InertialSensor`, `Filter/`)**: 2-pole Butterworth low-pass `INS_GYRO_FILTER`
at the backend rate (1 kHz × 2^`INS_GYRO_RATE`), then up to two harmonic notch sets
`INS_HNTCH_*` / `INS_HNTC2_*` (centre from throttle, RPM, ESC telemetry or in-flight FFT;
harmonics bitmask; single/double/triple notches; `HarmonicNotchFilter.cpp`).

**Slew limiter (`SMAX`)**: measures the slew rate of P+D; above `SMAX` it reduces P and D
(`Dmod` < 1). The measured slew rate (`SRate`) is the oscillation detector QuikTune uses.

**How the developers tune** (wiki "tuning process instructions", "setting up for tuning",
"manual roll/pitch", QuikTune, AutoTune, plane AutoTune):

* Prepare: linear thrust (`MOT_THST_EXPO`, voltage scaling), hover throttle, `INS_GYRO_FILTER`
  by prop size (80/40/20 Hz for 5/10/20"), `FLTT = FLTD = INS_GYRO_FILTER/2`, yaw `FLTE` 2-2.5.
* Notch filter from raw gyro logs before the final tune.
* Manual: raise D by 50% steps until oscillation, back off; raise P until oscillation, back
  off 25%; I = P.
* QuikTune (`AP_Scripting/applets/VTOL-quicktune.lua`): raise D then P exponentially (doubling
  every 10 s) until `SRate > 5` (lower for heavy vehicles), then multiply by 0.4; I = P
  (yaw I = P/10); yaw P ≤ 0.5, yaw D ≤ 0.01; never reduce below 80% of the original; abort on
  10° attitude error; `SMAX` must be > 0.
* AutoTune (`AC_AutoTune_Multi.cpp`): twitches; D raised until the bounce-back is 10%, P raised
  until the rate overshoots by 5%, back off 25% (`AUTOTUNE_GMBK`); angle P raised until the angle
  overshoots, backed off by (1 − 0.25)(1 − aggressiveness); accel limits from the measured
  twitches; limits `P 0.01..2`, `D ≤ 0.2`, `ANG_P 0.5..10`.
* Plane AutoTune (`APM_Control/AP_AutoTune.cpp`): flown by the pilot in FBWA; per stick event
  `FF = max_actuator / (max_rate · scaler)` (median of events); D raised ×1.3 until the slew
  limiter trips, then D = 0.3 × that; P raised ×1.3 until it trips, then P = 0.35 × that;
  I = min(P, FF) (roll) or max(0.75 P, FF) (pitch); `FLTT = 10/(2π·TCONST)`, `FLTD = gyro/2`;
  TCONST/RMAX from the `AUTOTUNE_LEVEL` table.
* The developers' web tools (`ArduPilot/WebTools`): PID Review (Wiener step-response estimate,
  FFT of PID terms), Filter Review (FFT of raw gyro with the configured notch applied in the
  frequency domain), Analytic Tune (frequency response from SystemID chirps, predicted closed
  loop with the exact PID/filter transfer functions).

## 2. What the app computes

### Identification (`engine/analysis.js identify`, `dsp/spectra.js`, `model/plant.js`)

Inputs per axis at the rate they were logged: `r` = rate target, `y` = actual rate, `u` = P+I+D+FF+DFF
(from a log: unbroken segments only, see "Log analysis"; nothing is interpolated or up-sampled).
Windowed (Hann), overlapped FFTs are averaged into cross spectra; a window never spans a logging gap,
ground time or a masked sample. Because the data are taken in
closed loop, the plant estimate uses the reference-driven ratio

`H_meas(f) = S_ry / S_ru`  (coherence = min(γ²_ry, γ²_ru))

which is unbiased by feedback as long as the excitation comes from the pilot/autopilot
reference. The known gyro chain `F(f)` is divided out: `H_bare = H_meas / F` (for planes also
scaled by the mean `scaler²`). A parametric model

`G(s) = K · e^(−s·Td) / ((s + a)(τ·s + 1))`

is fitted by Nelder-Mead on log-magnitude and phase over the contiguous band where coherence
≥ 0.3 (≥ 0.5 with few windows), capped below the gyro-filter cutoff. `a` is the aerodynamic
rate damping (≈0 for multirotors, the roll-mode pole for planes), `τ` the motor/servo lag, `Td`
the transport delay. Fit quality 1/(1 + cost/0.3) is reported with the trusted band.

Telemetry and logs rarely resolve `τ` and `Td` (they act above the excited band), so floors are
applied (copter: one loop period + 3 ms, 5 ms lag; plane: one loop + 15 ms, 30 ms lag) and then
calibrated: the model's closed loop with the **current** gains is matched to the measured
target→actual response `S_ry/S_rr` over the coherent band (`calibrateLag`). The residual
mismatch is reported; above 40% the proposal is flagged.

### Synthesis (`model/synth.js`, `model/controller.js`, `model/rules.js`)

The open loop `L = G · F · C_fb` uses the exact discrete AC_PID (target/error/D filters,
integrator) at `SCHED_LOOP_RATE` and the discrete gyro filter chain (2-pole LPF + notches at the
backend rate). The gains are then designed the way QuikTune does it, but on the model instead of on the aircraft:
starting from the current gains (halved until the model is stable), D is raised by bisection to the
value at which the loop's gain margin reaches 0 dB and a fraction of it is kept; then P is raised the
same way with I tied to it. The kept fraction is the preset: Conservative 30%, Normal 40% (QuikTune's
rule), Aggressive 50%. A safety net then lowers P if the model shows gain margin < GM_min, phase
margin < PM_min, sensitivity peak `max|1/(1+L)|` > Ms_max, or a simulated rate step (AC_PID driving
the plant, actuator limited) that overshoots more than OS_max: Conservative 6 dB / 30° / 3.0 / 12%,
Normal 4 dB / 25° / 3.5 / 20%, Aggressive 3 dB / 20° / 4.5 / 30%; these normally do not bind. D is
additionally capped by the RMS noise it would inject (measured gyro noise PSD through the D path,
≤ 4% of the mixer range, 1.5° for planes).

Then ArduPilot's rules: I = P (roll/pitch), I = 0.1 P (yaw), I = FF when FF is used (heli,
vectored axes, planes: roll min(P, FF), pitch max(0.75 P, FF)); FF = 1/K_dc (planes: from the
AUTOTUNE event method, median of ≥ 6 stick events); FLTT = FLTD = INS_GYRO_FILTER/2, FLTE 0
(yaw 2.5 unless already 0..8); SMAX 50 if zero; plane FLTT = 10/(2π TCONST); gain limits as in
AutoTune/QuikTune (yaw P ≤ 0.5, D ≤ 0.01). Angle P = crossover/4 (rad/s), clamped 3..12
(yaw 2..8); plane TCONST = 3/crossover clamped 0.15..1. Per analysis round the changes are
capped (P/I/FF ×2 or ÷2, D ×3 or ÷3, angle gains ×1.5) so the live loop converges in steps and
the log mode never jumps far from a flown state.

Copter gains are computed per rad/s (the plant K is converted), so the numbers are directly
ArduPilot's.

### Prediction and diagnostics

* Predicted rate-step response for the current and proposed gains (time simulation of AC_PID +
  plant + gyro filter), with rise time, overshoot, settling.
* Measured closed-loop step response by Wiener deconvolution of target→actual windows (the
  PID-Analyzer / PID Review method), averaged over windows with enough input.
* Diagnostics: RMS tracking error and tracking %, oscillation-band error energy and its peak
  (only counted as oscillation when the error energy at the peak exceeds the target energy there,
  i.e. not the pilot's own rhythm; peaks above 15 Hz are reported as vibration/motor noise),
  actuator saturation %, D/P output share, SRate max and Dmod min (slew limiter), steady I term
  (trim), gyro noise RMS, grade A-F.

### Live loop (`engine/tuner.js`, `engine/axis.js`)

`prepare` reads the needed parameters (per-name requests, light on radios), sets
`GCS_PID_MASK = 7` and `SMAX` if needed, requests EXTRA1 and then `PID_TUNING` at 50 Hz
(25 Hz planes), `ATTITUDE` at 25 Hz and EXTENDED_STATUS (`NAV_CONTROLLER_OUTPUT`, the attitude
target). No `PID_TUNING` sample is stored before the controller family (and so the unit) is known,
and `prepare` empties the buffers. Per axis the state machine observes (30 s, needs excitation),
analyses, proposes; with auto-apply it writes filters first, then P/I/D in multiplicative steps
≤ ×1.5 (D before P), then the angle gain (also ≤ ×1.5 per write). Rounds repeat (at most four per
axis) until the rate gains change by less than 10% and the angle gain by less than 15%, then the
next axis. Everything written is journaled in `sessions/*.json` (requested and echoed value,
armed or not) and in the append-only write journal (below), and **Restore original gains** rewrites the originals. A QuikTune-style ramp (same
doubling time, back-off and limits as the Lua applet) is available as a fallback when no model can
be identified.

**In-flight safety layer** (2026-10-17 audit R0 / R4; the same in the C++ port):

* *Opt-in.* While the vehicle is armed nothing is written unless `opts.inFlightWrites` is on (the
  "Allow changes in flight" switch, off on every start). `Tuner.setParam` is the one place every write
  goes through and checks it; housekeeping writes, restore and undo included.
* *One writer.* Every write holds the single write lock (a job). Pause and Stop cancel a job before its
  next `PARAM_SET`, link retries included (a cancelled write is read back and journaled as cancelled). A
  second Apply, Start or Resume while a job runs is refused; the observe loop has a generation counter so
  only one loop runs.
* *Monitor.* Every 250 ms while a session is active, on every axis: a trip when more than half of the
  last 2 s have `SRate` above the threshold (5 default; 3 for 50 kg, 1.5 for 100 kg) or `Dmod` below 0.9
  (samples with `Dmod` ≤ 0 carry no slew data and never count). It does not rely on the spectral check,
  which only runs on ≥ 80 Hz data. An analysis without a model while ≥ 5 % of the window shows these
  signs is reported as "oscillating", never as "fly sharper stick inputs".
* *Watch after every in-flight change* (gain steps, filter writes, the notch, notch values the autopilot's
  FFT wrote itself), on the axes it affects (all axes for vehicle-wide parameters), for 4 s (notch and
  filters 10 s) after a 0.5 s grace whose samples still describe the old values: any `SRate` above the
  threshold, `Dmod` < 0.9, the spectral check, a sustained monitor trip on any axis, or an attitude error
  above 10° for 0.8 s (`ATTITUDE` against the `NAV_CONTROLLER_OUTPUT` target) trips it. It fails closed:
  a lost link, no PID sample for 1 s, or fewer than half the expected samples counts as a trip. Before a
  change the vehicle must be calm (no trip sample in the last 2 s on any axis, up to 4 s of waiting) and
  the affected axes must stream PID data with the slew-rate signal.
* *Evidence-based revert.* On a trip the session pauses at once. If the 2 s before the change were calm,
  the change is reverted (the set just before it, which was observed calm); otherwise, or if it still
  oscillates after that, every value the session changed goes back to the session's starting values
  (observed calm in the check-up). Gains and their filters move in steps of at most stepMax, 1 s apart;
  the notch is switched off first. The result is checked (sustained criterion after 4 s); the message says
  what was written back, or "land now". Without the opt-in, and without any in-flight write of the app in
  this session, nothing is written: the pilot is told to land.

**Verified writes bound to the vehicle** (2026-10-17 audit R3; `mavlink/link.js`, `engine/paramwrite.js`,
`engine/identity.js`, `engine/journal.js`, `model/parammeta.js`; the same in the C++ port):

* *Pinned target (F-28).* The link talks to the first heartbeat from an ArduPilot autopilot (autopilot 3,
  component 1, not a GCS, gimbal, camera or ADS-B type). Messages from any other system or component are dropped
  (counted as foreign) and other vehicles are reported. A connection generation goes up on every connect and on a
  vehicle change (the target silent for 5 s while another answers, another frame type, another board UID); every
  pending request is then rejected (`link-changed`, with the number of attempts already sent) and no echo or
  `COMMAND_ACK` of an older generation resolves anything. An ACK counts only when its target is 0 or the app
  (255/190).
* *Identity (F-25).* `AUTOPILOT_VERSION` (requested with `MAV_CMD_REQUEST_MESSAGE` and the older
  `REQUEST_AUTOPILOT_CAPABILITIES`) gives firmware, git hash and board UID; `time_boot_ms` in the target's messages
  going back by more than 0.5 s counts as a reboot (pending requests rejected, identity read again). The session
  identity is `{gen, sysid, compid, type, uid, fw, gitHash, key "g<gen>:<sysid>/<compid>", paramHash}`; a
  proposal made for one key is never written on another, and a log is checked against the vehicle (`MAV_SYSID` /
  `SYSID_THISMAV`, firmware version and build) before its values can be written.
* *Exact verification (F-29 / F-30).* `verifiedSet` casts the value as the parameter's `MAV_PARAM_TYPE` stores it
  (integers rounded half away from zero and range-checked, REAL32 through `Math.fround`), sends it and compares
  the echo exactly; a missing or different echo is followed by one fresh read. Status: `verified`, `refused` (the
  value from before is still there), `different`, `not-applied`, `uncertain`, `missing`, `invalid`, `cancelled`
  or `not-sent`. Requested, sent and echoed values, the read-back, type and attempts are journaled. Every
  `PARAM_VALUE` keeps the tuner's cache current; a change the app did not make is journaled as a conflict.
* *Transactions (F-32).* `runTransaction` reads every value fresh first (a missing value or one that changed
  since the review aborts before anything is sent; the ground write saves the UNDO file at this point), writes in
  order, stops at the first value that does not hold and rolls back newest first, each roll-back verified; a
  generation change stops it and lists what is uncertain.
* *Approval (F-33).* A proposal carries an id and an FNV-1a 64 hash over `APT-PROPOSAL 1`, the identity (key,
  UID, firmware) and every `name=float32(to) from float32(from)`; Apply and the ground write take `{id, hash}`
  and refuse anything else, a stale proposal or another vehicle. The dialog shows full names, reasons, the
  values that act after a reboot and the id/hash. IMAX is written with the gains it belongs to.
* *Reboot-only values (F-35).* `INS_HNTC[H2]_REF`, `_HMNCS`, `_OPTS` and `FFT_ENABLE` (`@RebootRequired`) are
  refused while armed (`reboot-required`). On the ground `rebootAndVerify` sends `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`
  (disarmed only, an ACK from the target), waits until the autopilot is gone, reconnects when the transport
  closed, proves the restart (the boot clock went back, or the uptime is shorter than the time since the
  command), checks that it is the same vehicle and reads every expected value (after an Undo, the values from
  before). Notch values that appear only once the notch is on are written after it.
* *Journal (F-39).* `sessions/journal/<start>.jsonl`, one JSON line per event (`identity`, `proposal`, `approval`,
  `txn`, `write`, `rollback`, `reboot`, `refused`, `conflict`, `connection`, `export`), appended before the app
  goes on and never rewritten; a value not known is written as null. A lost line blocks forward writes until a
  restart (corrections still go out).
* *Export (F-43).* `paramFileChecked` writes CRLF `NAME,VALUE` lines with the shortest decimal that gives the same
  float32, whole numbers for integer parameters, the vehicle on the header line, the value from before and the
  reboot note on comment lines, and leaves out (with a `# NOT IN THIS FILE` line) a value outside ArduPilot's
  hard range, an invalid enum or an undocumented bit.

Checked on ArduCopter SITL 4.7.2-beta (`tools/sitl-ground-write.js --cpp`, both editions, disarmed): a ground write
of four gains read back, UNDO file, Undo read back; a `@READONLY` value reported as refused ("Param write denied")
with the other value rolled back; `INS_GYRO_FILTER` (INT16) 20.6 sent as 21; the notch with `REF` needing a reboot,
the reboot over TCP (the simulator re-executes, the app reconnects; about 4-6 s), every value read again, Undo and a
second reboot. ArduPilot 4.7 names the system id `MAV_SYSID`; notch parameters can be read while the notch is off.

The local server only obeys its own page: Host (and Origin when sent) must be 127.0.0.1 / localhost on its
port, `Sec-Fetch-Site: cross-site` is refused, state-changing routes accept only POST with the per-run token
the server writes into `index.html`, the page has a Content-Security-Policy, and `/api/setparam` is gone. The
web UI escapes every text from logs and the vehicle.

### Log analysis and notch design (`engine/loganalysis.js`, `engine/notch.js`)

The flight window is the longest flying segment in the right mode family (VTOL modes for a
QuadPlane's `Q_A_*`, fixed-wing modes for `RLL_RATE_*`). The fastest available source is used
(`PIDx` at the loop rate, else `RATE`). Gyro data come from `GYR` (raw logging), `ISBH/ISBD`
(batch sampler) or `IMU` (filtered). Welch spectra of the three gyro axes are peak-searched
(prominence over the median, half-power width); peaks are grouped into a fundamental and
harmonics; the recommendation is `INS_HNTCH_FREQ` = fundamental, `BW` = FREQ/2, `ATT` 40,
`HMNCS` = harmonics with ≥ 15% of the main peak (≤ 4), triple notch for broad peaks, mode 1
(throttle, `REF` = hover throttle from CTUN/QTUN) or mode 3 when ESC RPM matches the peak,
plus an `INS_GYRO_FILTER` that the notch allows (≤ f0/3, ≤ 80 Hz). The configured and the
recommended chains are evaluated as transfer functions and applied to the measured spectrum to
predict the post-filter noise in 5-40 Hz and the phase lag at 10 Hz.

**Data integrity** (audit 2026-10-17; `log/dfreader.js`, `log/integrity.js`, `engine/flightdata.js`; the C++
edition does the same, `tools/compare-cpp.js` checks it):
* The file must be a DataFlash `.bin` log: empty files, telemetry logs (`.tlog`), text / CSV and unknown files
  are refused with the reason (the command line exits with code 3). FMT definitions are checked; every record
  must be followed by the next record header, so damaged bytes are skipped and counted, not decoded as data.
* Time stamps are cleaned per table and instance: repeated and out-of-order records are dropped, a clock reset
  (two logs joined, a reboot) ends the usable part. A step longer than 2.5 typical intervals is a logging gap.
* The rate data stay at their logged rate (e.g. 300 Hz); unbroken segments are analysed one after the other and
  gaps are never filled in. Samples that are not numbers or impossible (|rate| > 5000 deg/s, |output| > 1000) are
  masked. Every metric, spectrum and model uses airborne samples only (throttle above 60% of hover, 3 s after
  lift-off and 2 s before touch-down left out; ArduPlane `CTUN.ThO` is in percent).
* The window never spans a change of a tuning parameter (gains, filters, notch, loop rate, mixer, frame;
  `STAT_*` and calibration values do not count) or the first 5 s after a fixed-wing / VTOL switch; the
  parameter values in effect then are used. VTOL modes are 17-23 and 25 (THERMAL and AUTOLAND fly as a plane).
* Peaks must be true local maxima; data logged too slowly to rule out aliasing get no oscillation verdict
  and no notch. The dynamic notch is modelled at the logged gyro rate (`IMU.GHz`) and its operating point
  (hover throttle, RPM, ESC, FFT) with the firmware's `FREQ x FM_RAT` floor and the Q fixed from `FREQ` / `BW`.
* Everything left out is said in plain language in the result (gaps, damaged bytes, dropped records, logger
  drops `DSF.Dp`, ground time, the window cut).

## 3. Limits to keep in mind

* No excitation, no model: the pilot (or SystemID mode) must move the vehicle.
* Rate loops of small copters have dynamics at 10-30 Hz; identification needs ≥ 50 Hz data.
  Telemetry radios at 57600 baud give ~15 Hz; use a fast link or the log.
* The model is linear; saturation, non-linear thrust and flexible frames are not modelled (the
  diagnostics flag saturation and high-frequency noise).
* The live tuner never changes `MOT_*`, `ATC_ACC_*_MAX`, `ATC_INPUT_TC` or other feel/limit parameters.
  The model tuner for heavy / slow vehicles (below) *proposes* `ACC_*_MAX`, `RATE_*_MAX`, the pilot yaw rate,
  `INPUT_TC`, `IMAX` and the gyro filter, each with its reason, in parameter files only; it never writes them.

## 4. Connection, installation and the live notch (second iteration)

* **Auto-connect** tries, in order: Mission Planner's MAVLink mirror (TCP 127.0.0.1:14550), the
  simulator ports (5762, 5763, 5760), a UDP listener on 14550, then every serial port at 115200 and
  57600 baud. A candidate is accepted when a vehicle heartbeat arrives within 2.5-3.5 s. Serial ports
  are read through a small PowerShell bridge (`src/mavlink/serialbridge.ps1`), so no native module
  is needed.
* **Housekeeping writes** (`GCS_PID_MASK = 7`, `*_SMAX = 50` when zero) are listed and confirmed by
  the user before they are sent; everything else goes through the same Apply confirmation.
* **Live notch** (`engine/notchlive.js`): with ESC RPM telemetry the notch is set to track the
  motors (`INS_HNTCH_MODE = 3`, `FREQ` = half the hover motor frequency as the floor, `BW` = a quarter,
  `REF = 1`, first two harmonics). On FFT-capable boards the app drives ArduPilot's own in-flight
  notch tune: `FFT_ENABLE = 1` (reboot), then `MAV_CMD_DO_AUX_FUNCTION` (218) with function 162
  high for 30 s of steady hover and low to finish; the autopilot averages its gyro FFT, picks the
  dominant harmonic family and writes `INS_HNTCH_FREQ/BW/REF/HMNCS/FM_RAT` itself
  (`AP_GyroFFT::stop_notch_tune` → `AP_InertialSensor::setup_throttle_gyro_harmonic_notch`). The app
  reads the parameters back and journals them.
* **Parameter files** are plain `NAME,VALUE` lines with `#` comment lines (previous values and the
  source), the format Mission Planner's "Load from file" reads.
* **App window**: `AP Tuner.vbs` starts the server hidden with `--app` and opens Microsoft Edge in
  app mode on it; the server exits by itself 20 s after the last window closes. `Install AP Tuner.cmd`
  copies the folder to the user's Programs folder and creates the shortcuts.

## 5. Gain selection, second iteration, and a direct check of the model

The first gain search (largest disturbance-rejection bandwidth under the margins) drifted to a high
D/P ratio with a low P, which measured worse on the simulator than the stock gains. The selection now
follows the procedure ArduPilot's QuikTune applet and the manual tuning guide use, executed on the
identified model instead of in the air (`model/synth.js synthesizeQuik`):

1. If the current gains would oscillate on the model, halve P and D until they do not.
2. Raise D (with that P) until the model's loop reaches its stability limit; keep the preset's fraction
   of it (Normal 0.4, as QuikTune's 60% reduction; Conservative 0.3; Sporty 0.5), subject to the
   D-noise cap.
3. Raise P (with that D) until the limit; keep the same fraction.
4. Lower P until phase margin, sensitivity peak and simulated overshoot are within the preset.

**Direct check on the simulator (2026-10-12).** With the stock D (0.0036) and filters, roll P was
raised in steps on a hovering simulated quad while watching ArduPilot's own slew-rate signal: P 0.36
flew normally, P 0.42 oscillated violently (SRate 80, rate error 93 deg/s RMS). The identified model
with the calibrated (near-zero) lag predicted the oscillation point at P = 0.455, within about 15% of
the measured onset (0.39-0.42). With the conservative lag floors the prediction was 0.2, far too
pessimistic, which is why the floors are only used when the measured closed loop cannot tell
(`calibrateLag`). `tools/osc-experiment.js` repeats this check.

**More sweeps (same day), quad and quadplane.** With the D filter lowered to 10 Hz the quad's onset did not
move (0.36 flew, 0.42 oscillated); with D raised to 0.0047 it fell to 0.31-0.36. A second sweep on the simulated
quadplane (Q_A roll, D 0.002, FLTD 10) oscillated between P 0.23 and 0.27 (docs/validation/). The decisive
check is the model the app identifies from the flight data: for the quad (K 17900, a 1.95, 0.5 ms lag +
0.5 ms delay) it predicts oscillation at P 0.43 for D 0.0036 and 0.39 for D 0.0047; for the quadplane
(K 25100, a 0.62, 2 ms lag) at P 0.265 for D 0.002. That is within 0-20% of the measured onsets, on the
slightly optimistic side. An extra lag chosen to best match the closed-loop response (4.2 ms quad, 11.5 ms
quadplane) made the predictions 20-30% pessimistic, so that match is now only reported as a diagnostic and the
identified lag is used as measured. Two rules follow: the app keeps the vehicle's target/D filters when they
are sane instead of lowering them, and Normal keeps QuikTune's 40% of the model's oscillation gain (30%
Conservative, 50% Aggressive). Results: simulated quad roll P 0.167, D 0.0040 (QuikTune applied to the measured
onset would give 0.144-0.168; ArduPilot's defaults for the model are 0.135 / 0.0036); simulated quadplane
P 0.098, D 0.0027 (QuikTune range 0.092-0.108).

## The guided flight session (engine/flight.js, 2026-10-13)

One flight, led by the app, on top of the Tuner and NotchLive:

| phase | what happens | written to the vehicle |
|---|---|---|
| preflight | waits for armed + a tuning-friendly mode (copter: Stabilize, AltHold, PosHold, Loiter; VTOL: QStabilize, QHover, QLoiter; plane: FBWA, FBWB, Cruise) | nothing |
| checkup | 30 s window per chosen axis, analyzeAxis as usual; issues in plain words (engine/issues.js) + the proposal | nothing |
| review | the pilot presses Apply (or the session was started in auto mode); with changes in flight off: Finish (apply on the ground) | nothing |
| tuning | Tuner.start with autoApply; the check-up analysis is used directly for round 1 (useFreshResult), then the normal rounds until convergence (rate gains < 10%, angle gain < 15%, max 4 rounds per axis) | gains, in steps <= 1.5x, each change watched (needs the in-flight opt-in) |
| filters | NotchLive: keep an existing notch; else ESC RPM tracking (mode 3); else the autopilot FFT tune (30 s hover); else advice for the log route; every axis watched 10 s, the notch switched back on a trip | INS_HNTCH_* (needs the opt-in) |
| recheck | always: Tuner rounds again over all chosen axes with the filter active | gains if they moved > 10% |
| overall | 30 s window: grades, tracking, oscillation, saturation, VIBRATION message, hover throttle re-read, notch state; verdict | nothing |
| land | waits for disarm; the pilot can also finish early | nothing |
| finished | summary: check-up vs final check per axis, gains start vs end, writes, notes; parameter file from /api/live/param | nothing |

A disarm in the middle pauses the session (resume after taking off again, or finish). Tested end to end
on the simulated quad with tools/flight-demo.js (starts SITL + the scripted pilot, drives /api/flight/*).

## Demonstration results (2026-10-12, demo/index.html)

Both ArduPilot simulator models were flown through the same 24-input stick sequence with under-tuned
starting gains, tuned by the app in a separate flight (Normal, roll + pitch, auto-apply, ESC-RPM notch),
and flown through the sequence again.

| vehicle | gains before -> after (roll) | rate tracking | rate error RMS | lag | angle-step overshoot |
|---|---|---|---|---|---|
| quad (medium X) | P 0.07 D 0.0015 ANG 3 -> P 0.121 D 0.0030 ANG 12 | 64% -> 76% | 9.3 -> 6.3 deg/s | 35 -> 20 ms | 12% -> 8% |
| quadplane VTOL | P 0.06 D 0.0008 ANG 3.5 -> P 0.086 D 0.0027 ANG 11.8 | 71% -> 78% | 5.9 -> 4.3 deg/s | 45 -> 35 ms | 15% -> 8% |

Tuning took 5.3 min (quad) and 5.4 min (quadplane) of flight; every write is in the session journal.

## Hover logs with little stick input (2026-10-16)

Ordinary hovers often carry too little deliberate stick input for the frequency-response fit. The log
analysis then:
- **Grades honestly**: a tracking figure from such a flight means nothing, so the axis is shown as
  "not gradable: too little stick input" (or graded D at most when a model exists but tracking is poor);
  without a measured model the grade is capped at C. A/B only when the tracking can actually be judged.
- **Detects saturation / trim** (`saturationCheck` in engine/analysis.js, text in engine/issues.js): a
  steady rate demand the aircraft does not follow, an I term near IMAX, motors pinned at spin max, and the
  CW/CCW motor split. Reported as "mechanical / torque imbalance: tuning can't fix this".
- **Fits the airframe in the time domain** (model/timeid.js): r_dot = b (sat(u) - trim) - a r, with
  per-10 s trims, motor lag / delay / mixer limit grid, airborne samples only, then an output-error
  refinement; rejected unless it replays the measured rate. The design is labelled lower confidence:
  conservative preset (30% of the oscillation gain), P/I 2x and D 1.5x per round (D never from 0), and on
  a weak axis P is sized by the mixer range (10 deg/s error = whole range). Log analysis only, never live.
- **Notch from post-filter IMU data** only when the peak shows up in at least 80% of at least five 10 s
  airborne parts; otherwise it asks for the batch sampler (INS_LOG_BAT_MASK) or ESC telemetry.
Worked example and the comparison with a hand tune: docs/validation/tandem-hover-logs.md.

## Model-based tuning of heavy and slow vehicles (2026-10-22, C++: `cascade.cpp`, `engine/heavy*.cpp`)

The tandem 4+1 QuadPlane (~100 kg, `docs/validation/tandem-hover-logs.md`) showed where the rules above stop
working: yaw drifted 70-110 deg after every lift-off and pitch wobbled at 0.45 Hz. The rule-based proposals
(QuikTune yaw ceiling 0.5, x2 per round, angle-P floor 3), evaluated on the validated models, gave pitch a phase
margin of 6 deg and a 41 deg yaw swing, against 42 deg and 11 deg for the hand tune that flew. The model tuner
implements that hand tune (`research/tandem-2026-10/` holds the reference scripts).

**The axis chain** (`core/cascade`, checked against the ArduPilot source):
- Pilot shaping: first order with `INPUT_TC` (yaw: `PLT_Y_RATE_TC`), then the acceleration limit `ACC_*_MAX` in
  deg/s/s (older firmware: `ACCEL_*` in cdeg/s/s).
- Angle controller: `sqrt_controller` with angle P and `clamp(ACC/2, 40..720)` deg/s/s (yaw: `10..120`). The
  correction is limited to `RATE_*_MAX`, then the feed-forward is added.
- Yaw heading error: limited to `min(inv_sqrt_controller(1/rate P, angle P, accel), 45 deg)`, with the target dragged
  along (`thrust_heading_rotation_angles`). That is 45 deg at stock gains and 31 deg at rate P 2.5, not a fixed 45.
- Rate PID: `AC_PID::update_all` (FLTT, FLTE, D on the FLTD-filtered error derivative, the integrator frozen in the
  saturated direction while LIMIT is set, IMAX).
- Then the mixer limit u_max, transport delay T, motor lag tau, and the vehicle `rate' = a (u - trim) - c rate` (+ gusts).
- Gyro: 2-pole Butterworth at `INS_GYRO_FILTER`.

**Margins.** `L(s) = G FE Cr (FT K / s + Fg)`, broken at the plant input.
- Phase margin: at the last gain crossover of the unwrapped phase.
- Gain margin: only at the first -180 deg crossing above it. The rate-I + angle-P cascade passes -180 deg far below
  the crossover with |L| >> 1; that is harmless conditional stability (the first hand draft counted it by mistake).
- Ms = max |1/(1+L)|.
- Slow axes also get the poles of `s^3 + (aP + c) s^2 + a (PK + I) s + aIK`. Stock tandem yaw: zeta 0.12, period 16 s,
  which is the swing seen in every hover.

**Hovers.**
- The controller flies (PID `RESET` flag clear; telemetry: armed) in a VTOL mode, with the throttle above 60% of the
  median, for at least 8 s, never across a tuning-parameter change.
- Lift-off is where the throttle reaches 90% of the hover's median: before that the ground holds the vehicle. (The
  controller output rises earlier, on the ground, at high gains.)

**Diagnostics**, each in plain words and marked "tuning can't fix this" where it is mechanical:
- Torque imbalance: steady yaw output, I term against IMAX, CCW motors 1+2 against CW 3+4.
- Mixer limit: the median output while LIMIT is set, but at least the 99th percentile actually delivered (telemetry
  has no flags: the latter only). Then the headroom left after the trim, the achievable acceleration both ways
  `a (u_max -+ trim)`, and the stopping distance of a drift.
- CG from the front / rear thrust split, using the thrust curve (`*_SPIN_MIN/MAX`, `*_THST_EXPO`, PWM range),
  cross-checked with the pitch I term; in cm when the motor spacing is given.
- 0.2-3 Hz oscillations: 10 s windows, the largest cluster of peaks within +-15% (a second cluster is kept), the
  target-vs-actual phase, the I-term swing, and growth within a hover.
- Motors at their top output; the slew limiter.
- Gyro and output above 15 Hz, compared with the saved plan's flights and between flights (more than x2 means
  mechanical).
- Hover throttle against `*_THST_HOVER`; altitude hold in the altitude modes; compass messages and field strength.

**Identification as a set of plants.**
- **Yaw** (a slow axis with a constant disturbance): windowed integral regression on the rate data,
  `r(t1) - r(t0) = a int(u) - c int(r) + d_k (t1 - t0)`.
  - 1 s windows, 1.5 s left out after lift-off, 0.1 s averages at the window ends, one trim `-d_k/a` per hover.
  - Motor lag tried at 0 / 0.1 / 0.2 / 0.3 s. It is not identifiable, so the set spans 0.1-0.4 s.
  - The spread of a comes from leaving one hover out at a time, and is at least x0.8 / x1.25.
  - The set: nominal; weak and slow (c = 0); strong with slow motors; strong with fast motors; large trim.
- **Roll / pitch** (an oscillating cascade): at the oscillation the loop sits at L ~ -1, so
  `G(jw0) = -1 / (FE Cr (FT K / jw0 + Fg))` with the flown gains. |G| gives a; the phase gives tau for T = 10 and 25 ms.
  - Every window frequency in the cluster, and a second wobble frequency, widens the range of a.
  - The set: a over that range (at least x0.82 / x1.2) x tau x T.
  - The naive regression of the rate on the output is not used: in closed loop it tends to -1/controller
    (R^2 0.02-0.07 on the tandem).
- **No oscillation** (a well-tuned flight): the saved model of the vehicle is used. A new fit that disagrees with it by
  more than x1.5 is added to the set ("the model is uncertain, the design must hold for both").

**Validation gate.**
- Yaw: every hover is replayed from lift-off with the logged stick, the gains in effect and the hover's own trim. A
  hover passes when the heading swing is within 5 deg (or 12%), the heading RMS is 30 deg or less, and the output at
  1 Hz and below is within 0.07 + 0.03 x rate P. The model is validated when 60% of the hovers pass.
- Roll / pitch: with the flown gains, the model must oscillate at the logged frequency +-15% (gust-driven simulation),
  have its worst phase margin in -20..+10 deg, and be able to get as lively as the flight. When the flight shows no
  oscillation, the model must predict none.
- A validated axis may jump to its design in one step (the tandem yaw went x8 and flew as predicted). Otherwise, per
  step: P and I move at most x2, D x1.5, angle P x1.5, and the step says why.

**Design**, only for the heavy / weak class: yaw a < 1 or roll / pitch a < 20 rad/s/s per unit, or a confirmed cascade.
- **Roll and pitch together**, for each gyro-filter route (as flown, and half of it with FLTT = FLTD = filter/2):
  - Search: rate P around `1 / (3 (tau + T) a)` (the bandwidth the lags allow), I/P 0.1-0.5, D x1 / x1.5 / x2.5,
    angle P 0.75-3.
  - Accepted: worst-case PM >= 40 deg, upper GM >= 10 dB and Ms <= 1.8 on every plant, and output noise at the
    measured vibration peaks (or 1.25 / 1.85 x the gyro filter) no more than +15%.
  - Ranked by time simulations on the nominal and the slow plant: lift-off with the trim and I = 0, gust wander, a
    5 deg step (overshoot and rise), late oscillation, output limited. Within 10% of the best score, the one with the
    most phase margin wins.
  - The filter route is chosen for both axes together: on the tandem, a filter change alone made roll worse.
- **Yaw:**
  - Search: rate P up to the bandwidth the slowest motors allow, I/P 0.05-0.1, angle P 1-3.
  - Accepted when the slow pair has zeta >= 0.35 on the nominal plant and >= 0.2 on every corner. The angle loop sits
    close to the rate loop by design, so the loop phase margin is reported but not used as a gate: the flown and
    confirmed Step 2 has a low one.
  - Ranked by lift-off swing, late oscillation, a yaw gust and two pilot turns, on every plant.
  - Stress tests: +50 ms delay, rotor-inertia kick x2 and x4, gyro noise.
- **Limits from physics:**
  - `ACC_R/P_MAX` <= 70% of `a (u_max - |trim|)`, the same for both axes, never above the current value.
  - `ACC_Y_MAX` = 1.25 x the stronger direction (the weak direction is the torque imbalance, reported separately).
  - `RATE_Y_MAX` = `PLT_Y_RATE` = `RATE_WPY_MAX`: the largest step whose stopping distance is 15 deg or less.
  - IMAX >= |trim| + 0.2; `INPUT_TC` >= 0.2.

**Steps and files.** Yaw first, then roll + pitch + filter. Each step has:
- the rows with their reasons;
- a `.param` file (checked values, `# was` comments, the vehicle named) and an UNDO file with the values as flown;
- the predicted results: for yaw the heading-swing range over the set and the damping; for roll / pitch the worst PM,
  whether a wobble is expected, gust wander, step overshoot and lift-off tilt;
- a reboot note when `INS_GYRO_FILTER` changes, and the test card.

On a QuadPlane the Q_ values are only in Mission Planner's Full Parameter List (Extended Tuning shows the fixed-wing ones).

**Next-flight check.** Every analysis saves a plan (`sessions/plans/sysid-N/`). The next analysis of that vehicle (not
of the same flights):
1. finds which step was flown: its values are in effect, or its parameters moved away from the plan's starting values
   ("flown with other values");
2. predicts with the saved model, for the gains actually flown and this flight's torque trim;
3. compares: the yaw swing must fall within the predicted range +-20% (or 3 deg); a wobble expected or not;
4. re-fits the model and proposes the next step, or says to stop.

On the tandem: Step 1 predicted 15-21 deg, flew 13; Step 2 predicted 8-11 deg, flew 11 (the hand predictions were
14-20 and 9-11).

**Live.** With the vehicle connected (Mission Planner's mirror is found automatically), the app:
- reads the settings it needs;
- asks for the telemetry streams (PID_TUNING needs `GCS_PID_MASK = 7`, which the app reports but does not set);
- records each flight from arming to disarming and analyses it after landing.

25 Hz telemetry is enough for these slow motions; only the vibration checks need a log. Nothing is written to the vehicle.
