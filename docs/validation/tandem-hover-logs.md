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

## Regression on well-excited SITL logs

`demo/quad/{before,tuning,after}.BIN` and `demo/quadplane/{before,tuning,after}.BIN`, old (installed) vs
new code: all roll/pitch gains, grades and changes identical. Only yaw changed: those flights had no yaw
stick input, so yaw is now "not gradable" instead of A/B; the hover-data fit is not used there (the
aircraft barely moved in yaw, < 2 deg/s RMS). `test/synthetic.test.js` passes.
