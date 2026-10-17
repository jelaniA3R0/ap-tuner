// ArduPilot tuning rules and parameter naming for each vehicle family. These encode what the
// ArduPilot code and wiki do: QuikTune (VTOL-quicktune.lua), AC_AutoTune_Multi, AP_AutoTune (plane),
// and the tuning-process wiki pages.
'use strict';

const FAMILIES = {
  copter: { // ArduCopter multirotor (also used for heli with ff=true)
    axes: { roll: 'ATC_RAT_RLL_', pitch: 'ATC_RAT_PIT_', yaw: 'ATC_RAT_YAW_' },
    angleP: { roll: 'ATC_ANG_RLL_P', pitch: 'ATC_ANG_PIT_P', yaw: 'ATC_ANG_YAW_P' },
    accMax: { roll: 'ATC_ACC_R_MAX', pitch: 'ATC_ACC_P_MAX', yaw: 'ATC_ACC_Y_MAX' },
    loopRate: 'SCHED_LOOP_RATE', gyroFilter: 'INS_GYRO_FILTER', hoverThr: 'MOT_THST_HOVER',
    pidTuningMask: 'GCS_PID_MASK', pidAxisIds: { roll: 1, pitch: 2, yaw: 3 },
    logPid: { roll: 'PIDR', pitch: 'PIDP', yaw: 'PIDY' }, logRate: { roll: ['RDes', 'R', 'ROut'], pitch: ['PDes', 'P', 'POut'], yaw: ['YDes', 'Y', 'YOut'] },
    uUnits: 'mixer (-1..1)', defaultSmax: 50, uMax: 1,
  },
  vtol: {  // QuadPlane VTOL motors
    axes: { roll: 'Q_A_RAT_RLL_', pitch: 'Q_A_RAT_PIT_', yaw: 'Q_A_RAT_YAW_' },
    angleP: { roll: 'Q_A_ANG_RLL_P', pitch: 'Q_A_ANG_PIT_P', yaw: 'Q_A_ANG_YAW_P' },
    accMax: { roll: 'Q_A_ACC_R_MAX', pitch: 'Q_A_ACC_P_MAX', yaw: 'Q_A_ACC_Y_MAX' },
    loopRate: 'SCHED_LOOP_RATE', gyroFilter: 'INS_GYRO_FILTER', hoverThr: 'Q_M_THST_HOVER',
    pidTuningMask: 'GCS_PID_MASK', pidAxisIds: { roll: 1, pitch: 2, yaw: 3 },
    logPid: { roll: 'PIQR', pitch: 'PIQP', yaw: 'PIQY' }, logRate: { roll: ['RDes', 'R', 'ROut'], pitch: ['PDes', 'P', 'POut'], yaw: ['YDes', 'Y', 'YOut'] },
    uUnits: 'mixer (-1..1)', defaultSmax: 50, uMax: 1,
  },
  plane: { // fixed wing
    axes: { roll: 'RLL_RATE_', pitch: 'PTCH_RATE_', yaw: 'YAW_RATE_' },
    angleP: null, tconst: { roll: 'RLL2SRV_TCONST', pitch: 'PTCH2SRV_TCONST' },
    rmax: { roll: ['RLL2SRV_RMAX'], pitch: ['PTCH2SRV_RMAX_UP', 'PTCH2SRV_RMAX_DN'] },
    loopRate: 'SCHED_LOOP_RATE', gyroFilter: 'INS_GYRO_FILTER', scalingSpeed: 'SCALING_SPEED',
    pidTuningMask: 'GCS_PID_MASK', pidAxisIds: { roll: 1, pitch: 2, yaw: 3 },
    logPid: { roll: 'PIDR', pitch: 'PIDP', yaw: 'PIDY' }, logRate: null,
    uUnits: 'surface deg (45 = full)', defaultSmax: 150, uMax: 45,
  },
};

const SUFFIXES = ['P', 'I', 'D', 'FF', 'FLTT', 'FLTE', 'FLTD', 'SMAX', 'IMAX', 'D_FF'];

/** All parameter names the tuner needs for a family */
function neededParams(family) {
  const f = FAMILIES[family];
  const names = [f.loopRate, f.gyroFilter, f.pidTuningMask, 'INS_GYRO_RATE',
    'INS_HNTCH_ENABLE', 'INS_HNTCH_MODE', 'INS_HNTCH_FREQ', 'INS_HNTCH_BW', 'INS_HNTCH_ATT', 'INS_HNTCH_REF', 'INS_HNTCH_HMNCS', 'INS_HNTCH_OPTS', 'INS_HNTCH_FM_RAT',
    'INS_HNTC2_ENABLE', 'INS_HNTC2_MODE', 'INS_HNTC2_FREQ', 'INS_HNTC2_BW', 'INS_HNTC2_ATT', 'INS_HNTC2_REF', 'INS_HNTC2_HMNCS', 'INS_HNTC2_OPTS', 'INS_HNTC2_FM_RAT'];
  for (const ax of Object.keys(f.axes)) for (const s of SUFFIXES) names.push(f.axes[ax] + s);
  if (f.angleP) for (const ax of Object.keys(f.angleP)) names.push(f.angleP[ax]);
  if (f.accMax) for (const ax of Object.keys(f.accMax)) names.push(f.accMax[ax]);
  if (f.tconst) for (const ax of Object.keys(f.tconst)) names.push(f.tconst[ax]);
  if (f.rmax) for (const ax of Object.keys(f.rmax)) names.push(...f.rmax[ax]);
  if (f.hoverThr) names.push(f.hoverThr);
  if (f.scalingSpeed) names.push(f.scalingSpeed, 'AIRSPEED_MIN', 'AIRSPEED_MAX', 'ARSPD_USE', 'RLL_RATE_FF', 'PTCH_RATE_FF');
  if (family === 'copter') names.push('ATC_RATE_FF_ENAB', 'ATC_INPUT_TC', 'FRAME_CLASS', 'FRAME_TYPE', 'MOT_THST_EXPO', 'MOT_SPIN_MIN', 'MOT_PWM_MIN', 'MOT_PWM_MAX', 'MOT_BAT_VOLT_MAX', 'MOT_BAT_VOLT_MIN', 'ATC_THR_MIX_MAN', 'PILOT_Y_RATE_TC', 'FFT_ENABLE', 'LOG_BITMASK', 'INS_LOG_BAT_MASK', 'INS_RAW_LOG_OPT');
  if (family === 'vtol') names.push('Q_ENABLE', 'Q_FRAME_CLASS', 'Q_FRAME_TYPE', 'Q_M_THST_EXPO', 'Q_M_SPIN_MIN', 'Q_A_RATE_FF_ENAB', 'Q_TAILSIT_ENABLE', 'Q_TILT_ENABLE', 'LOG_BITMASK', 'INS_LOG_BAT_MASK', 'INS_RAW_LOG_OPT');
  return Array.from(new Set(names));
}

/** Margin presets. QuikTune's 60% back-off from the oscillation gain ≈ 8 dB gain margin. */
const PRESETS = {
  // backoff = fraction of the model's oscillation gain that is kept (QuikTune keeps 40%)
  // The design follows QuikTune: the gain kept is a fraction (backoff) of the model's oscillation gain,
  // D first, then P. The margin limits below are only a safety net for odd airframes; they are set so that
  // they do not normally bind (a direct oscillation test on the simulator showed textbook phase-margin /
  // sensitivity limits made the result far more conservative than ArduPilot's own tuners).
  // Kept fraction of the model's oscillation gain (QuikTune keeps 40%). Checked by oscillation sweeps on ArduPilot's
  // simulated quad and quadplane (docs/how-it-works.md): the model identified from flight data predicted the onset
  // within 0-20% (quad: 0.43 / 0.39 predicted vs 0.36-0.42 / 0.31-0.36 measured; quadplane 0.265 vs 0.23-0.27),
  // and with 40% kept the designs land inside the range QuikTune itself would produce from the measured onsets.
  conservative: { gmMinDb: 6, pmMinDeg: 30, msMax: 3.0, osMaxPct: 12, backoff: 0.3, label: 'Conservative (large / flexible / payload aircraft: 30% of the oscillation gain)' },
  normal: { gmMinDb: 4, pmMinDeg: 25, msMax: 3.5, osMaxPct: 20, backoff: 0.4, label: 'Normal (QuikTune rule: 40% of the oscillation gain)' },
  aggressive: { gmMinDb: 3, pmMinDeg: 20, msMax: 4.5, osMaxPct: 30, backoff: 0.5, label: 'Aggressive (small agile aircraft: 50% of the oscillation gain)' },
};

/**
 * Filter rules (QuikTune setup_filters + plane AP_AutoTune): FLTT = FLTD = INS_GYRO_FILTER/2,
 * roll/pitch FLTE = 0, yaw FLTE kept if 0 < FLTE <= 8 else 2.5 (AC_AutoTune FLTE_MIN).
 * Plane: FLTT = 10/(2 pi tau) (AP_AutoTune), FLTE = 0, FLTD = gyro/2.
 */
function filterRules(family, axis, cur, gyroHz, tau) {
  if (family === 'plane') {
    return { FLTT: +(10 / (2 * Math.PI * Math.max(tau || 0.5, 0.1))).toFixed(2), FLTE: 0, FLTD: +(gyroHz * 0.5).toFixed(1) };
  }
  // keep the vehicle's own target / D filters when they are sane (0 < f <= INS_GYRO_FILTER); only an
  // unset or too-high filter is replaced by the wiki's INS_GYRO_FILTER/2. A direct oscillation test on the
  // simulator showed the loop model is validated with the stock filters, and lowering FLTD is not required
  // for the tune.
  const sane = v => v > 0 && v <= gyroHz;
  const r = { FLTT: sane(cur.FLTT) ? cur.FLTT : +(gyroHz * 0.5).toFixed(1), FLTD: sane(cur.FLTD) ? cur.FLTD : +(gyroHz * 0.5).toFixed(1), FLTE: 0 };
  if (axis === 'yaw') r.FLTE = (cur.FLTE > 0 && cur.FLTE <= 8) ? cur.FLTE : 2.5;
  else if (cur.FLTE > 0 && cur.FLTE <= gyroHz) r.FLTE = cur.FLTE;
  return r;
}

/** I-gain rules */
function iRule(family, axis, P, FF, hasFF) {
  if (family === 'plane') {
    // AP_AutoTune: roll I = min(P, FF/1s); pitch/yaw I = max(0.75 P, FF)
    return axis === 'roll' ? Math.min(P, FF) : Math.max(0.75 * P, FF);
  }
  if (hasFF && FF > 0) return FF;   // heli / vectored axes: I = FF (one second trim time constant)
  return axis === 'yaw' ? 0.1 * P : P;   // QuikTune: RP_PI_RATIO 1, Y_PI_RATIO 10
}

/** Gain limits (AC_AutoTune + QuikTune caps) */
function gainLimits(family, axis) {
  if (family === 'plane') return { P: [0.01, 3.0], D: [0.0005, 0.3], FF: [0.01, 3.0] };
  if (axis === 'yaw') return { P: [0.02, 0.5], D: [0, 0.01], FF: [0, 1] };   // QUIK_YAW_P_MAX 0.5, QUIK_YAW_D_MAX 0.01
  return { P: [0.01, 2.0], D: [0.0005, 0.2], FF: [0, 1] };                   // AUTOTUNE_RP_MIN/MAX, RD_MAX
}

/** Angle P from rate-loop crossover (rad/s): cascade separation 1/4, clamped as AC_AutoTune (SP 0.5..10) */
function angleRule(family, axis, wcRadS, { yawMin = 2 } = {}) {
  if (family === 'plane') {
    // TCONST: 1/angle_gain; keep >= 3x the rate loop time constant, within [0.15, 1.0]
    const tau = Math.min(1.0, Math.max(0.15, 3 / Math.max(wcRadS, 1e-3)));
    return { tconst: +tau.toFixed(2) };
  }
  // yawMin: 0.5 (AC_AutoTune minimum) is allowed for a measured very weak yaw axis (big props, slow torque)
  const p = Math.min(axis === 'yaw' ? 8 : 12, Math.max(axis === 'yaw' ? yawMin : 3, wcRadS / 4));
  return { angleP: +p.toFixed(2) };
}

/** Oscillation (slew-rate) threshold by vehicle class, from VTOL-quicktune.md guidance */
function oscSmaxFor(massKg) {
  if (!massKg) return 5;
  if (massKg >= 100) return 1.5; if (massKg >= 50) return 3; if (massKg >= 15) return 4; return 5;
}

module.exports = { FAMILIES, SUFFIXES, neededParams, PRESETS, filterRules, iRule, gainLimits, angleRule, oscSmaxFor };
