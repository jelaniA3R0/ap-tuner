# Hover logs with little stick input: tandem 4+1 QuadPlane (2026-10-16)

Logs: Mission Planner `logs/FIXED_WING/1/2026-10-15 04-20-29.bin`, `04-33-02.bin`, `04-43-00.bin`
(ArduPlane 4.7.1, Cube Orange+, ~100 kg, QSTABILIZE hovers). Run: `node tools/analyze-log.js <file>`.

## What the app used to say

Every axis: "coherence too low to fit a model" (no gains), yet grade B (75) with "tracking 0%".
Notch: 41.2 Hz from one log, 28.8 Hz from another.

## What it says now

| | 04-20-29 | 04-33-02 | 04-43-00 |
|---|---|---|---|
| roll grade | not gradable | not gradable | not gradable |
| roll model | hover fit rejected (replays -111%) | rejected (-53%) | rejected (negative damping, 11 s airborne) |
| pitch grade | not gradable | not gradable | not gradable |
| pitch model (lower confidence) | b 5.6 rad/s² per unit, a 0.49, replays 78% | b 4.8, a 0.63, 62% | b 5.0, a 0.56, 69% |
| pitch proposal (this round) | P/I 0.25 -> 0.5, D 0.0036 -> 0.0054, ANG 4.5 -> 3 | same | P/I 0.5, D 0.0041 |
| pitch full model design | P 0.80 | P 0.93 | P 1.03 |
| yaw grade | F: saturated | F: saturated | F: saturated |
| yaw model (lower confidence) | b 0.15, a 0.09, mixer limit 0.5, trim 0.23-0.38 | b 0.34, a 0.28, trim 0.40-0.48 | rejected (9 s airborne, b not positive) |
| yaw proposal (this round) | P 0.18 -> 0.36, I -> 0.036, IMAX 0.5 -> 0.75, ANG 4.5 -> 3 | same | none |
| yaw full model design | P 2.9, I 0.29, ANG 0.5 | P 3.9, I 0.39, ANG 0.5 | - |
| notch | no peak | none: 41.2 Hz in 6/8 parts, 30.6 Hz in 3/8 | none: flight too short to check (1 part) |

Saturation evidence (yaw, all logs): rate demand +36 to +95 deg/s against ~0 actual; yaw I term mean
0.19-0.30, 95th percentile 0.34-0.48 of IMAX 0.5; a motor at its upper limit (1950 us) 3-12% of the
flying time; CCW motors ~1750-1780 us against CW ~1460-1490 us.
Message: "Saturated yaw ... This is a mechanical / torque imbalance: tuning can't fix this."

## Comparison with the hand tune (same logs)

Hand fit: r_dot = 0.20 (YOut - trim) - 0.165 r (rad/s), trim 0.37-0.45, mixer yaw limit ~0.63.
Hand steps: P 1.5 / 2.5 / 3.5, I 0.11 / 0.19 / 0.26, ANG_YAW 1.0 / 1.0 / 1.5, IMAX 0.7.

| | hand | app |
|---|---|---|
| control power b (rad/s² per unit) | 0.20 | 0.15 (log 1), 0.34 (log 2), +-30-40% |
| damping a (1/s) | 0.165 | 0.09, 0.28 |
| steady gain b/a | 1.2 | 1.8, 1.2 |
| trim | 0.37-0.45 | 0.23-0.38 (log 1), 0.40-0.48 (log 2) |
| mixer yaw limit | ~0.63 | 0.5 (log 1); not separable in log 2 (largest output used 0.67) |
| rate P target | 1.5 -> 3.5 in three steps | 2.9-3.9 full design; 0.36 this round |
| rate I | 0.11-0.26 (I/P ~0.075) | 0.29-0.39 (QuikTune yaw rule I = P/10) |
| yaw angle P | 1.0-1.5 | 0.5 full design; 3.0 this round (from 4.5) |
| IMAX | 0.7 | 0.75 |

Why they differ:
- **b and a** bracket the hand values. The hand fit used all five hovers with full closed-loop replays
  of the heading; the app fits each log on its own, only the steady airborne part (it drops 3 s after
  lift-off and 2 s before touch-down), and only the longest Q-mode stretch of each log. Log 3 has 9 s of
  such data, too little, so its yaw is rejected rather than guessed.
- **Rate P target** agrees (hand 3.5, app 2.9-3.9). On this model the yaw loop stays stable up to P > 14,
  so the app does not size P by stability: it sizes it so a 10 deg/s yaw-rate error uses the whole
  mixer range (0.5-0.67). That rule lands on the same values the hand steps reached.
- **Step size**: the app moves at most 2x per flight (0.18 -> 0.36) and keeps the QuikTune yaw ceiling
  (0.5) for the proposal, because the model is lower confidence. The hand steps jumped 8x at once,
  justified by the replay of all hovers. The app says that the model wants more, and why.
- **I**: the app follows QuikTune (I = P/10); the hand values used a little less (P/13).
- **Angle P**: the app uses the cascade rule (angle loop at most 1/4 of the rate-loop crossover), giving
  the AutoTune minimum 0.5 for a crossover of ~0.07-0.2 Hz. The hand value 1.0-1.5 was picked from the
  heading-swing replay. Both say "much lower than 4.5"; the app moves 4.5 -> 3 this round.
- **Roll** gets no proposal: the hover data fit does not replay the roll rate (gusts dominate), so the app
  says so instead of proposing gains. Pitch fits well (replays 62-78%) and gets a lower-confidence 2x step.

## After the log-integrity fixes (2026-10-17, branch `fix/log-integrity`)

Same logs, `cd07d42` (before) vs the fixed analysis (after); JavaScript and C++ give identical results.

| | 04-20-29 | 04-33-02 | 04-43-00 |
|---|---|---|---|
| data before | 162.7 s on a 200 Hz grid, the 51.8 s logging gap bridged by straight lines | 100.5 s up-sampled to 400 Hz | 29.6 s up-sampled to 400 Hz |
| data after | 300.03 Hz as logged; gap 230.5-282.3 s reported and left out; 110.9 s logged, 44.3 s airborne used | 300.03 Hz; 62.6 s airborne | 300.03 Hz; 12.1 s airborne |
| "parameter changes" | 65 -> 0 tuning-relevant (65 STAT_* / baro values) | 123 -> 0 (126 housekeeping) | 14 -> 0 |
| roll | no model -> frequency-response model; P/I 0.25 -> 0.5, D 0.0036 -> 0.0070, ANG 4.5 -> 3 | no model -> P/I 0.5, D 0.0054, ANG 3 | no model (fit does not replay the rate) |
| pitch | P/I 0.5, D 0.0054, ANG 3 (unchanged) | unchanged | P/I 0.25 -> 0.5 became 0.25 -> 0.125 (see below) |
| yaw | P 0.36, I 0.036, IMAX 0.75, ANG 3 (unchanged) | unchanged | none (unchanged) |
| notch | "no peak" -> none: flight too short to check that the peaks repeat | none: peaks do not repeat (unchanged) | none: too short (unchanged) |

RMS rate errors are higher now (roll 5.99 -> 9.40 deg/s on 04-20-29) because ground time, where the error is
near zero, no longer dilutes them.

To treat with care:
- **04-43-00 pitch**: only 12 s airborne; the hover-data fit replays just 11% of the measured rate but passes,
  because the gate accepts max(r2, replay). Its P 0.125 (-50%) should not be used; requiring a replay of at
  least 30% (roadmap R2) would reject it.
- **Roll** on 04-20-29 / 04-33-02 now passes the frequency-response gates on airborne-only data, but the
  excitation is gusts, not stick input (bias F-58): confirm with stick inputs before trusting it.
- The six SITL demo logs (`demo/quad`, `demo/quadplane`) lost 19-45% of the analysed window in the logger
  (`DSF.Dp` 69,247 to 438,260 dropped messages): the rate data come in unbroken pieces of 0.05-3.7 s, and
  identification is now refused with the reason (no unbroken stretch long enough, too few spectrum windows, or
  too little coherence) instead of being fitted on interpolated data.

## Regression on well-excited SITL logs

`demo/quad/{before,tuning,after}.BIN` and `demo/quadplane/{before,tuning,after}.BIN`, old (installed) vs
new code: all roll/pitch gains, grades and changes identical. Only yaw changed: those flights had no yaw
stick input, so yaw is now "not gradable" instead of A/B; the hover-data fit is not used there (the
aircraft barely moved in yaw, < 2 deg/s RMS). `test/synthetic.test.js` passes.

## Model-based tuning (2026-10-22, C++ `aptuner-heavy` / the Model tune tab)

Same airframe, the 15 Oct logs analysed together, then the 21 Oct flights checked against the saved plan
(`aptuner-heavy <logs> --plan plan.json`; `cpp/tests/test_heavy.cpp` runs all of it as a regression test).

### What the app finds (15 Oct, 5 hovers)

| | app (model tuning) | hand analysis |
|---|---|---|
| yaw control power a, damping c | 0.19, 0.15 (range 0.15-0.24) | 0.20, 0.165 |
| yaw trims per hover | 0.37, 0.42, 0.44, 0.37, 0.43 | 0.37-0.45 |
| yaw mixer limit u_max | 0.66 (LIMIT flags) | 0.57-0.67 |
| yaw replay, heading swing log / model | 67/66, 108/100, 83/82, 91/84, 76/76 deg (5 of 5 pass) | 67/66, 109/107, 84/89, 92/91, 75/77 |
| torque imbalance | flagged, "tuning can't fix", CCW 1801 us vs CW 1451 us | the same |
| pitch | cascade at 0.46 Hz, a 6.1 (4.6-7.3), tau 0.06-0.08 s, worst PM -10 deg with the flown gains | 0.43-0.48 Hz, a 5.8, tau 0.06-0.09, PM ~0 |
| roll | marginal: wobbles at 0.42 Hz and 0.76 Hz; a 5.1, range 3.7-16 (the two frequencies) | a 7-12 (assumed range) |
| CG | 5.7% of the front-rear spacing aft (44/56% thrust; pitch I -0.11 agrees) | 3% (47/53%, other windows) |
| vibration above 15 Hz, pitch gyro | 2.4 deg/s | 2.3 |
| hover throttle | 0.46 against Q_M_THST_HOVER 0.36 | the same |

The roll range is wide because the roll wobble has two frequencies (0.42 and 0.76-0.82 Hz) and the 21 Oct flights
show the higher one. The app does not pick one: the roll design must hold for the whole range.

### Old app, new app, hand tune, flown (15 Oct logs)

| | old app (rules, 16 Oct) | new app (model) | hand tune | flown outcome |
|---|---|---|---|---|
| yaw | P 0.36, I 0.036, ANG 3; model: 41 deg lift-off swing, still oscillating | P 3.1, I 0.155, IMAX 0.7, ANG 1.0, RATE_Y_MAX / PLT_Y_RATE / RATE_WPY_MAX 20, ACC_Y_MAX 15; predicted swing 9-13 deg | steps P 1.5 / 2.5 / 3.5, I/P ~0.07, ANG 1-1.5, IMAX 0.7, limits 20 / 15 / 20 | Step 1: 13 deg, Step 2: 11 deg, steady |
| pitch | P = I = 0.5, D 0.0054, ANG 3: worst PM 6 deg | gyro 10, P 0.71, I 0.21, D 0.009, ANG 0.75, ACC 90, INPUT_TC 0.2: worst PM 43 deg (41 on the hand's set) | gyro 10, P 0.5, I 0.15, D 0.009, ANG 1.0, ACC 100: PM 42 | 22 Oct: "very good" |
| roll | P = I = 0.5, D 0.007, ANG 3: worst PM 11 deg | P 0.36, I 0.072, D 0.009, ANG 0.75: worst PM 47 deg (46 on the hand's set) | P 0.4, I 0.1, D 0.009, ANG 1.0: PM 40 | 22 Oct: "very good" |

The flown 22 Oct pitch set has a worst phase margin of 43 deg on the app's pitch model (hand: 42); the flown roll set
42 deg on the 15 Oct roll model and 28 deg on the wider range that includes the 21 Oct roll frequency, which is why the
app proposes a slightly softer roll rate P after the 21 Oct flights (0.26). The app's pitch rate P is higher than the hand's
(0.71 against 0.5) with the same margin target; both pass every test on both models.

### Predicted against flown (next-flight check)

| flight | recognised as | yaw heading swing predicted / flown | pitch | verdict |
|---|---|---|---|---|
| `2026-10-21 00-34-01` (yaw P 1.5, I 0.11, ANG 1, limits 20/15/20) | Step 1, flown with other values (P 1.5 against the proposed 3.1) | 15-21 / 13 deg (with this flight's trim 0.33; hand: 14-20) | wobble expected, still there (0.46 Hz, +-5 deg) | confirmed; yaw re-fit a 0.29, c 0.22, trim 0.33 (hand re-fit 0.31, 0.11, 0.34) |
| `2026-10-21 01-11-37` (yaw P 2.5, I 0.19) | Step 1 (other values) | 8-11 / 11 deg (hand: 9-11) | growing wobble detected (+-5 deg growing to +-8) | confirmed; "keep the yaw settings you fly"; roll + pitch step proposed |

Things to know:
- On 21 Oct the pitch gyro above 15 Hz was 7.2 and 9.7 deg/s against 2.4 on 15 Oct (x3-4): flagged as mechanical
  (props, mounts, flight-controller mount), as in the hand analysis.
- The Step 1 flight's yaw model differs from 15 Oct (a 0.29 against 0.19); the design holds for both.
- The 22 Oct log (pitch / roll set flown, "very good") is not on this PC yet. Analysing it with the saved plan will check
  the pitch / roll prediction (no 0.4-0.5 Hz peak expected) and review QHOVER / QLOITER (hover throttle learning,
  altitude hold, compass).
