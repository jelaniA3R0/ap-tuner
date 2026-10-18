// Log (.bin) analysis: extract per-axis rate-loop data for copter / VTOL / plane, run analyzeAxis,
// and run the gyro-noise / notch analysis when raw IMU data is present.
// Data integrity (audit 2026-10-17): the file type is checked; time stamps are cleaned (repeated and
// out-of-order records, clock resets); the rate data stay at their native rate and are split at logging gaps
// (nothing is interpolated across a gap); impossible or non-numeric samples are masked; every metric and
// spectrum uses airborne samples only; the analysis window never spans a change of a tuning parameter or a
// fixed-wing / VTOL transition; and all of it is reported in plain language.
'use strict';
const { parseBin } = require('../log/dfreader');
const { cleanLogTimes, splitAtGaps, medianStep } = require('../log/integrity');
const { analyzeAxis } = require('./analysis');
const { analyzeNoise } = require('./notch');
const { FAMILIES } = require('../model/rules');
const { axisIssues } = require('./issues');
const { throttleStream, airborneIntervals, inIntervals, intervalSeconds, notchOperatingPoint } = require('./flightdata');

const WANT = ['ATT', 'RATE', 'PIDR', 'PIDP', 'PIDY', 'PIQR', 'PIQP', 'PIQY', 'IMU', 'ISBH', 'ISBD', 'GYR', 'CTUN', 'QTUN', 'MODE', 'AETR', 'VIBE', 'RCOU', 'ESC', 'RPM', 'FTN1', 'FTN2', 'ARSP', 'ATUN', 'ATRP', 'QUIK', 'ANG', 'DSF', 'PARM', 'MSG'];
/** tables whose time stamps the analysis relies on: checked and cleaned after parsing */
const TIMED = ['RATE', 'PIDR', 'PIDP', 'PIDY', 'PIQR', 'PIQP', 'PIQY', 'IMU', 'GYR', 'CTUN', 'QTUN', 'MODE', 'AETR', 'RCOU', 'ESC', 'RPM', 'FTN1'];
/** values no real aircraft logs (corrupt records): rates in deg/s, controller terms in mixer units / degrees */
const RATE_LIMIT_DEGS = 5000, OUTPUT_LIMIT = 1000;
/** a gap longer than this is shortened on the analysis grid (the samples keep their logged times) */
const MAX_GAP_ON_GRID_S = 2;

function detectLogFamily(log) {
  const txt = log.text.map(x => x.msg).join(' ');
  const p = log.params;
  if (/ArduCopter/.test(txt)) return p.H_SV_MAN !== undefined || /heli/i.test(txt) && p.H_COL_MAX !== undefined ? 'heli' : 'copter';
  if (/ArduPlane/.test(txt)) {
    // VTOL data present? prefer the family with usable PID logs
    const q = log.msgs.PIQR && log.msgs.PIQR.n > 500;
    return q ? 'vtol' : 'plane';
  }
  if (log.msgs.PIQR) return 'vtol';
  return log.msgs.RATE ? 'copter' : 'plane';
}

/** nearest-sample lookup helper for a time-indexed table */
function sampler(tUs, vals) {
  let j = 0;
  return (tSec) => { const t = tSec * 1e6; while (j < tUs.length - 1 && tUs[j + 1] <= t) j++; return vals[j]; };
}

// ------------------------------------------------------------------------------------------- parameters

const TUNING_PREFIXES = ['ATC_RAT_', 'ATC_ANG_', 'ATC_ACC_', 'Q_A_RAT_', 'Q_A_ANG_', 'Q_A_ACC_', 'RLL_RATE_', 'PTCH_RATE_', 'YAW_RATE_', 'RLL2SRV_', 'PTCH2SRV_', 'INS_HNTCH_', 'INS_HNTC2_', 'MOT_SPIN_', 'MOT_PWM_', 'Q_M_SPIN_', 'Q_M_PWM_'];
const TUNING_NAMES = new Set(['ATC_RATE_FF_ENAB', 'ATC_INPUT_TC', 'Q_A_RATE_FF_ENAB', 'Q_A_INPUT_TC', 'INS_GYRO_FILTER', 'INS_GYRO_RATE', 'INS_FAST_SAMPLE', 'INS_RAW_LOG_OPT', 'INS_LOG_BAT_OPT', 'SCHED_LOOP_RATE', 'MOT_THST_EXPO', 'Q_M_THST_EXPO', 'FRAME_CLASS', 'FRAME_TYPE', 'Q_FRAME_CLASS', 'Q_FRAME_TYPE', 'SCALING_SPEED']);
/** parameters the analysis depends on (gains, filters, notch, loop rate, mixer, frame); not housekeeping such as STAT_* or calibration values */
function isTuningParam(name) {
  if (TUNING_NAMES.has(name) || TUNING_PREFIXES.some(p => name.startsWith(p))) return true;
  return /^SERVO[0-9]+_FUNCTION$/.test(name);
}

/** in-flight parameter changes [{t, name, from, to}], all of them and the tuning-relevant ones */
function parameterChanges(log) {
  const last = {};
  for (const [k, v] of Object.entries(log.params)) if (k !== '__changes') last[k] = v;
  const all = [];
  for (const c of log.params.__changes || []) { all.push({ t: c.t, name: c.name, from: last[c.name], to: c.val }); last[c.name] = c.val; }
  return { all, tuning: all.filter(c => isTuningParam(c.name)) };
}

/** the parameter values in effect at time t: boot values with every logged change up to t applied */
function paramsAt(log, t) {
  const p = {};
  for (const [k, v] of Object.entries(log.params)) if (k !== '__changes') p[k] = v;
  if (t === null || t === undefined) return p;
  for (const c of log.params.__changes || []) if (c.t <= t) p[c.name] = c.val;
  return p;
}

// ------------------------------------------------------------------------------------------- window

const Q_MODES = new Set([17, 18, 19, 20, 21, 22, 23, 25]);   // QSTABILIZE QHOVER QLOITER QLAND QRTL QAUTOTUNE QACRO LOITER_ALT_QLAND (24 THERMAL, 26 AUTOLAND fly as a plane)
const PLANE_MODE_NAMES = { 0: 'MANUAL', 1: 'CIRCLE', 2: 'STABILIZE', 3: 'TRAINING', 4: 'ACRO', 5: 'FBWA', 6: 'FBWB', 7: 'CRUISE', 8: 'AUTOTUNE', 10: 'AUTO', 11: 'RTL', 12: 'LOITER', 13: 'TAKEOFF', 14: 'AVOID_ADSB', 15: 'GUIDED', 16: 'INITIALISING', 17: 'QSTABILIZE', 18: 'QHOVER', 19: 'QLOITER', 20: 'QLAND', 21: 'QRTL', 22: 'QAUTOTUNE', 23: 'QACRO', 24: 'THERMAL', 25: 'LOITER_ALT_QLAND', 26: 'AUTOLAND' };
/** seconds left out after a switch between fixed-wing and VTOL flight (the transition, ArduPlane Q_TRANSITION_MS default 5 s) */
const TRANSITION_S = 5;

/** mode segments [{t0, t1, mode, prev}] from MODE messages */
function modeSegments(log, tEnd) {
  const m = log.msgs.MODE; if (!m || !m.n) return [];
  const out = [];
  for (let i = 0; i < m.n; i++) { const t0 = m.TimeUS[i] / 1e6; const t1 = i + 1 < m.n ? m.TimeUS[i + 1] / 1e6 : tEnd; out.push({ t0, t1, mode: m.Mode[i], prev: i > 0 ? m.Mode[i - 1] : null }); }
  return out;
}

/**
 * The analysis window: the longest stretch flown (throttle above idle) in a mode that uses the analysed
 * controller (VTOL modes for 'vtol', fixed-wing modes for 'plane', any for copter), without the first 5 s after
 * a switch between fixed-wing and VTOL flight, and never across a change of a tuning parameter, so the gains
 * analysed are the ones that were flying. An explicit [tStart, tEnd] is cut the same way at tuning changes.
 * Returns { tStart, tEnd, mode, transition: {from, to} | null, cutBy: [changes inside the flight], explicit } or null.
 */
function chooseWindow(log, family, thr, tuning, tStart = null, tEnd = null) {
  const explicit = tStart !== null || tEnd !== null;
  if (!explicit && !thr) return null;
  const idle = family === 'plane' ? 0.05 : 0.15;
  const tLast = thr ? thr.t[thr.t.length - 1] : tEnd;
  let ranges;
  if (explicit) ranges = [{ t0: tStart ?? 0, t1: tEnd ?? (tLast ?? Infinity), mode: null, transition: null }];
  else {
    const kindChange = s => (family === 'vtol' || family === 'plane') && s.prev !== null && Q_MODES.has(s.prev) !== Q_MODES.has(s.mode);
    const all = modeSegments(log, tLast);
    const segs = all.filter(s => family === 'vtol' ? Q_MODES.has(s.mode) : family === 'plane' ? !Q_MODES.has(s.mode) : true);
    // modes are logged but none of them uses the analysed controller: there is nothing to analyse
    if (all.length && !segs.length) return { none: true, reason: family === 'vtol' ? 'the log has no flight in a VTOL (Q) mode' : 'the log has no flight in a fixed-wing mode' };
    ranges = segs.length ? segs.map(s => ({ t0: kindChange(s) ? s.t0 + TRANSITION_S : s.t0, t1: s.t1, mode: s.mode, transition: kindChange(s) ? { from: s.prev, to: s.mode } : null })) : [{ t0: 0, t1: tLast, mode: null, transition: null }];
  }
  let best = null;
  for (const r of ranges) {
    // cut where a tuning parameter changed: each piece was flown with one set of values
    const cuts = tuning.filter(c => c.t > r.t0 && c.t < r.t1).map(c => c.t);
    const edges = [r.t0, ...cuts, r.t1];
    for (let k = 0; k + 1 < edges.length; k++) {
      let a = edges[k], b = edges[k + 1];
      if (!explicit) {
        let t0 = null, t1 = null;
        for (let i = 0; i < thr.t.length; i++) { const ts = thr.t[i]; if (ts < a || ts > b) continue; if (thr.v[i] > idle) { if (t0 === null) t0 = ts; t1 = ts; } }
        if (t0 === null) continue;
        a = t0; b = t1;
      }
      if (!best || b - a > best.tEnd - best.tStart) best = { tStart: a, tEnd: b, mode: r.mode, transition: r.transition, range: r, explicit };
    }
  }
  if (!best) return null;
  best.cutBy = tuning.filter(c => c.t > best.range.t0 && c.t < best.range.t1);
  return best;
}

/** longest flying stretch [t0, t1] (s) that the analysis uses, or null */
function flightWindow(log, family) {
  const w = chooseWindow(log, family, throttleStream(log, family), parameterChanges(log).tuning);
  return w && !w.none ? [w.tStart, w.tEnd] : null;
}

// ------------------------------------------------------------------------------------------- axis data

/** time (s) of grid sample g of an axis dataset (each segment keeps its logged start time) */
function gridTime(d, g) {
  const segs = d.segs;
  if (!segs || !segs.length) return d.t0 + g / d.fs;
  let lo = 0, hi = segs.length - 1;
  while (lo < hi) { const m = (lo + hi + 1) >> 1; if (segs[m].g0 <= g) lo = m; else hi = m - 1; }
  return segs[lo].t0 + (g - segs[lo].g0) / d.fs;
}
/** grid sample nearest to time t (s) inside a segment, or -1 */
function gridIndex(d, t) {
  const segs = d.segs;
  if (!segs || !segs.length) { const k = Math.round((t - d.t0) * d.fs); return k >= 0 && k < d.n ? k : -1; }
  let lo = 0, hi = segs.length - 1;
  while (lo < hi) { const m = (lo + hi + 1) >> 1; if (segs[m].t0 <= t) lo = m; else hi = m - 1; }
  const s = segs[lo];
  const g = s.g0 + Math.round((t - s.t0) * d.fs);
  return g >= s.g0 && g < s.g1 ? g : -1;
}

/**
 * The per-axis dataset, at the native rate of the rate-loop log. The samples are used as logged (ArduPilot
 * logs one sample per loop; the stamps jitter with the loop's run time, not with the sampling), unbroken
 * segments are placed one after the other on a grid at the mean sample rate inside the segments, and the
 * places where data are missing are marked (valid = 0): a gap is never filled in, a corrupt sample never used.
 * Returns {fs, t0, n, rawRate, source, target, actual, u, P,I,D,FF, srate, dmod, ss?, valid, airborne?,
 * segs: [{g0, g1, t0}], gaps: [{t0, t1}], excluded: {nonFinite, implausible}, scaler2, throttle} or null.
 */
function axisData(log, family, axis, { tStart = null, tEnd = null } = {}, ctx = null) {
  const fam = FAMILIES[family === 'heli' ? 'copter' : family];
  if (!ctx) { const thr = throttleStream(log, family); ctx = { thr, air: family !== 'plane' && thr && tStart !== null ? airborneIntervals(thr, tStart, tEnd) : null, op: null }; }
  const pidName = fam.logPid[axis];
  const pid = log.msgs[pidName];
  const rateMsg = log.msgs.RATE && fam.logRate ? log.msgs.RATE : null;
  // native rate of each stream from its typical step (robust against stamp jitter and gaps)
  const rateOf = m => { if (!m || m.n <= 200) return 0; const s = medianStep(m.TimeUS); return s > 0 ? 1e6 / s : 0; };
  const pidRate = rateOf(pid), rateRate = rateOf(rateMsg);
  let tUs, cols, source;
  if (pid && pid.n > 200 && pidRate >= rateRate * 0.5) {
    tUs = pid.TimeUS; source = pidName;
    const u = new Float64Array(pid.n);
    for (let i = 0; i < pid.n; i++) u[i] = pid.P[i] + pid.I[i] + pid.D[i] + pid.FF[i] + (pid.DFF ? pid.DFF[i] : 0);
    // copter / VTOL PIDx logs are in rad/s, plane PIDx in deg/s: keep everything in deg/s
    const k = family === 'plane' ? 1 : 57.2957795;
    const tar = k === 1 ? pid.Tar : Float64Array.from(pid.Tar, v => v * k), act = k === 1 ? pid.Act : Float64Array.from(pid.Act, v => v * k);
    cols = { target: tar, actual: act, u, P: pid.P, I: pid.I, D: pid.D, FF: pid.FF, srate: pid.SRate || new Float64Array(pid.n), dmod: pid.Dmod || new Float64Array(pid.n).fill(1) };
    if (family === 'plane' && log.msgs.AETR && log.msgs.AETR.SS) { const a = log.msgs.AETR; const get = sampler(a.TimeUS, a.SS); cols.ss = Float64Array.from(pid.TimeUS, v => get(v / 1e6) || 1); }
  } else if (rateMsg) {
    const r = rateMsg; const [des, act, out] = fam.logRate[axis];
    tUs = r.TimeUS; source = 'RATE';
    cols = { target: r[des], actual: r[act], u: r[out] };
  } else return null;
  // time window
  let i0 = 0, i1 = tUs.length;
  if (tStart !== null) while (i0 < i1 && tUs[i0] / 1e6 < tStart) i0++;
  if (tEnd !== null) while (i1 > i0 && tUs[i1 - 1] / 1e6 > tEnd) i1--;
  if (i1 - i0 < 200) return null;
  const m = i1 - i0;
  const tt = new Float64Array(m); for (let k = 0; k < m; k++) tt[k] = tUs[i0 + k] / 1e6;
  // samples that are not numbers or that no aircraft can log (corrupt records) are masked
  const checks = [['target', RATE_LIMIT_DEGS], ['actual', RATE_LIMIT_DEGS], ['u', OUTPUT_LIMIT], ['P', OUTPUT_LIMIT], ['I', OUTPUT_LIMIT], ['D', OUTPUT_LIMIT], ['FF', OUTPUT_LIMIT]];
  const good = new Uint8Array(m); let nonFinite = 0, implausible = 0;
  for (let k = 0; k < m; k++) {
    let bad = 0;
    for (const [name, lim] of checks) { const a = cols[name]; if (!a) continue; const v = a[i0 + k]; if (!Number.isFinite(v)) { bad = 1; break; } if (Math.abs(v) > lim) bad = 2; }
    if (bad === 1) nonFinite++; else if (bad === 2) implausible++; else good[k] = 1;
  }
  // unbroken segments (a step longer than 2.5 typical intervals is a logging gap) and the native rate inside them
  const sp = splitAtGaps(tt);
  let num = 0, den = 0;
  for (const [a, b] of sp.segments) if (b - a >= 2) { num += b - a - 1; den += tt[b - 1] - tt[a]; }
  if (!(den > 0)) return null;
  const fs = +(num / den).toFixed(3);
  // grid: segments one after the other, at least one empty slot between them; long gaps are shortened
  const maxGapSlots = Math.max(1, Math.round(MAX_GAP_ON_GRID_S * fs));
  const segs = [];
  for (const [a, b] of sp.segments) {
    let g0 = 0;
    if (segs.length) { const p = segs[segs.length - 1]; const slots = Math.round((tt[a] - (p.t0 + (p.g1 - p.g0) / fs)) * fs); g0 = p.g1 + Math.max(1, Math.min(maxGapSlots, slots)); }
    segs.push({ g0, g1: g0 + (b - a), t0: tt[a], k0: a });
  }
  const N = segs[segs.length - 1].g1;
  const valid = new Uint8Array(N);
  for (const s of segs) for (let g = s.g0; g < s.g1; g++) valid[g] = good[s.k0 + g - s.g0];
  const d = { fs, t0: tt[0], n: N, rawRate: sp.medianDt > 0 ? 1 / sp.medianDt : fs, source, valid, segs: segs.map(s => ({ g0: s.g0, g1: s.g1, t0: s.t0 })), gaps: sp.gaps, excluded: { nonFinite, implausible } };
  for (const [name, src] of Object.entries(cols)) {
    const a = new Float64Array(N);
    for (const s of segs) for (let g = s.g0; g < s.g1; g++) a[g] = src[i0 + s.k0 + g - s.g0];
    // slots without a usable sample hold the last usable value (they are never used: valid = 0, they only
    // keep running filters calm)
    let last = null;
    for (let g = 0; g < N; g++) { if (valid[g]) last = a[g]; else a[g] = last === null ? NaN : last; }
    let first = null; for (let g = 0; g < N && first === null; g++) if (valid[g]) first = a[g];
    for (let g = 0; g < N && !valid[g]; g++) a[g] = first === null ? 0 : first;
    d[name] = a;
  }
  // airborne (copter / VTOL): on the ground the controller output says nothing about the airframe
  if (family !== 'plane' && ctx.air) {
    const am = new Uint8Array(N);
    for (const s of segs) for (let g = s.g0; g < s.g1; g++) am[g] = inIntervals(ctx.air.intervals, tt[s.k0 + g - s.g0]) ? 1 : 0;
    d.airborne = am;
  }
  // plane: speed scaler from AETR.SS (mean of scaler^2 over the window)
  d.scaler2 = 1;
  if (family === 'plane' && log.msgs.AETR && log.msgs.AETR.SS) {
    const a = log.msgs.AETR; let s = 0, c = 0;
    for (let i = 0; i < a.n; i++) { const ts = a.TimeUS[i] / 1e6; if (ts >= tt[0] && ts <= tt[m - 1] && a.SS[i] > 0) { s += a.SS[i] * a.SS[i]; c++; } }
    if (c) d.scaler2 = s / c;
  }
  // throttle (0..1) for the notch operating point: the hover throttle while airborne
  if (ctx.op && ctx.op.throttle !== undefined) d.throttle = ctx.op.throttle;
  else if (ctx.thr) { let s = 0, c = 0; for (let i = 0; i < ctx.thr.t.length; i++) { const ts = ctx.thr.t[i]; if (ts >= tt[0] && ts <= tt[m - 1]) { s += ctx.thr.v[i]; c++; } } d.throttle = c ? s / c : 0; }
  return d;
}

/** I-term summary of an axis from the PID log (any rate), usable airborne samples of the data window only */
function iStats(log, family, axis, d) {
  const fam = FAMILIES[family === 'heli' ? 'copter' : family];
  const pid = log.msgs[fam.logPid[axis]]; if (!pid || !pid.I) return null;
  const v = []; let m = 0;
  for (let i = 0; i < pid.n; i++) {
    const g = gridIndex(d, pid.TimeUS[i] / 1e6); if (g < 0) continue;
    if (d.valid && !d.valid[g]) continue;
    if (d.airborne && !d.airborne[g]) continue;
    if (!Number.isFinite(pid.I[i])) continue;
    v.push(Math.abs(pid.I[i])); m += pid.I[i];
  }
  if (v.length < 20) return null;
  v.sort((a, b) => a - b);
  return { mean: m / v.length, p95: v[Math.floor(v.length * 0.95)], max: v[v.length - 1], n: v.length };
}

/**
 * Lift-motor outputs while flying (airborne stretches of the window): average PWM per motor, how often any
 * motor sits at its upper limit (spin max), and the split between the motors spinning each way (quad X/+),
 * which shows a steady yaw correction the controller has to hold.
 */
function motorStats(log, params, family, win, air) {
  const rc = log.msgs.RCOU; if (!rc) return null;
  const q = family === 'vtol' ? 'Q_M_' : 'MOT_';
  const chans = [];   // motor k (1-based) -> RCOU column
  for (let n = 1; n <= 14; n++) { const fn = params['SERVO' + n + '_FUNCTION']; if (fn >= 33 && fn <= 40 && rc['C' + n]) chans[fn - 32] = { col: rc['C' + n], n }; }
  const motors = []; for (let k = 1; k < chans.length; k++) if (chans[k]) motors.push(k);
  if (!motors.length) return null;
  const pmin = params[q + 'PWM_MIN'] || params['SERVO' + chans[motors[0]].n + '_MIN'] || 1000, pmax = params[q + 'PWM_MAX'] || params['SERVO' + chans[motors[0]].n + '_MAX'] || 2000;
  const top = pmin + (pmax - pmin) * (params[q + 'SPIN_MAX'] ?? 0.95) - 5;
  const sum = {}; for (const k of motors) sum[k] = 0;
  let c = 0, pinned = 0, maxPwm = 0;
  for (let i = 0; i < rc.n; i++) {
    const ts = rc.TimeUS[i] / 1e6; if (win.tStart !== null && (ts < win.tStart || ts > win.tEnd)) continue;
    if (air && !inIntervals(air.intervals, ts)) continue;
    let any = false, ok = true; for (const k of motors) if (!Number.isFinite(chans[k].col[i])) ok = false;
    if (!ok) continue;
    for (const k of motors) { const v = chans[k].col[i]; sum[k] += v; if (v >= top) any = true; if (v > maxPwm) maxPwm = v; }
    if (any) pinned++; c++;
  }
  if (c < 20) return null;
  const mean = {}; for (const k of motors) mean[k] = +(sum[k] / c).toFixed(0);
  const out = { motors, meanPwm: mean, pinnedTopPct: 100 * pinned / c, topPwm: +top.toFixed(0), maxPwm, pwmMin: pmin, pwmMax: pmax };
  // ArduPilot quad X / + : motors 1 and 2 spin counter-clockwise, 3 and 4 clockwise (a positive yaw output
  // speeds up the CCW pair). Other frames: no split.
  const cls = params[family === 'vtol' ? 'Q_FRAME_CLASS' : 'FRAME_CLASS'], typ = params[family === 'vtol' ? 'Q_FRAME_TYPE' : 'FRAME_TYPE'];
  if (cls === 1 && (typ === 0 || typ === 1) && [1, 2, 3, 4].every(k => mean[k] !== undefined)) {
    out.ccwMean = (mean[1] + mean[2]) / 2; out.cwMean = (mean[3] + mean[4]) / 2;
  }
  return out;
}

// ------------------------------------------------------------------------------------------- reporting

const f1 = v => (+v).toFixed(1);
const plural = (n, one, many) => `${n} ${n === 1 ? one : many}`;
const bytesText = n => (n >= 1048576 ? (n / 1048576).toFixed(1) + ' MB' : n >= 1024 ? (n / 1024).toFixed(0) + ' kB' : n + ' bytes');
const modeName = m => PLANE_MODE_NAMES[m] || 'mode ' + m;

/** plain-language notes about the file and its time stamps (for every axis) */
function logNotes(integ, timeIssues, tables) {
  const out = [];
  if (integ.loggerDropped > 0) out.push(`The flight controller's logger reports ${plural(integ.loggerDropped, 'message', 'messages')} it could not write (DSF.Dp): the log is incomplete (SD card too slow, too much logging, or a simulator running too fast).`);
  if (integ.skippedBytes > 0) out.push(`${bytesText(integ.skippedBytes)} of the file could not be read (damaged or blank data) and ${integ.skippedBytes === 1 ? 'was' : 'were'} skipped; reading picked up again after ${plural(integ.resyncs, 'damaged stretch', 'damaged stretches')}. Any time missing there is left out.`);
  if (integ.truncatedBytes > 0) out.push(`The file ends in the middle of a record (the last ${bytesText(integ.truncatedBytes)}): the log or its download was cut short.`);
  if (integ.badFmtCount > 0) out.push(`${plural(integ.badFmtCount, 'message definition', 'message definitions')} in the file ${integ.badFmtCount === 1 ? 'is' : 'are'} invalid and ${integ.badFmtCount === 1 ? 'was' : 'were'} ignored (${integ.badFmts.slice(0, 3).map(b => (b.name || '?') + ': ' + b.reason).join('; ')}).`);
  if (integ.fmtConflicts > 0) out.push(`${plural(integ.fmtConflicts, 'message definition appears', 'message definitions appear')} a second time with a different layout; the first one was used.`);
  for (const name of tables) {
    const r = timeIssues[name]; if (!r) continue;
    if (r.outOfOrder || r.duplicates) out.push(`${name}: ${[r.outOfOrder ? plural(r.outOfOrder, 'record', 'records') + ' with time stamps out of order' : null, r.duplicates ? plural(r.duplicates, 'repeated time stamp', 'repeated time stamps') : null].filter(Boolean).join(' and ')} left out.`);
    if (r.reset) out.push(`${name}: the log's clock jumps back from ${f1(r.reset.fromS)} s to ${f1(r.reset.toS)} s (two logs joined, or a reboot): only the part before the jump is used (${plural(r.droppedAfterReset, 'record', 'records')} after it ignored).`);
  }
  return out;
}

/** plain-language notes about one axis dataset: gaps, masked samples, ground time */
function axisNotes(d, win, airS) {
  const out = [];
  const winS = win.tStart !== null ? win.tEnd - win.tStart : (gridTime(d, d.n - 1) - d.t0);
  const big = d.gaps.filter(g => g.t1 - g.t0 >= 1), small = d.gaps.filter(g => g.t1 - g.t0 < 1);
  for (const g of big.slice(0, 3)) out.push(`Logging stopped for ${f1(g.t1 - g.t0)} s (${f1(g.t0)}-${f1(g.t1)} s) inside the analysed window: that stretch is left out (nothing is filled in).`);
  if (big.length > 3) out.push(`${big.length - 3} more logging gaps of 1 s or longer are left out as well.`);
  if (small.length) {
    const s = small.reduce((a, g) => a + g.t1 - g.t0, 0);
    const longest = d.segs.reduce((mx, x) => Math.max(mx, x.g1 - x.g0), 0) / d.fs;
    out.push(`The ${d.source} data have ${plural(small.length, 'short dropout', 'short dropouts')} (${f1(s)} s in total, ${(100 * s / Math.max(winS, 1e-9)).toFixed(0)}% of the window) where samples are missing: the log could not keep up. They are left out; spectra and models use only unbroken stretches (the longest is ${longest.toFixed(1)} s).`);
  }
  const ex = d.excluded;
  if (ex.nonFinite || ex.implausible) out.push(`${[ex.nonFinite ? plural(ex.nonFinite, 'sample was', 'samples were') + ' not a number (NaN / infinity)' : null, ex.implausible ? plural(ex.implausible, 'sample had', 'samples had') + ' impossible values (corrupt records)' : null].filter(Boolean).join(' and ')}: left out.`);
  if (airS !== null && winS > 0 && airS < 0.9 * winS) out.push(`Only ${f1(airS)} s of the ${f1(winS)} s window ${airS === 1 ? 'was' : 'were'} flown (the rest was on the ground${d.gaps.length ? ' or not logged' : ''}): every number here uses the flown part only.`);
  return out;
}

/** plain-language notes about the window choice (tuning changes, transition) */
function windowNotes(w) {
  const out = [];
  if (!w) return out;
  if (w.cutBy && w.cutBy.length) {
    const list = w.cutBy.slice(0, 4).map(c => `${c.name} ${+(+c.from).toPrecision(4)} -> ${+(+c.to).toPrecision(4)} at ${f1(c.t)} s`).join(', ');
    out.push(`Tuning parameters changed during the flight (${list}${w.cutBy.length > 4 ? ', ...' : ''}): only ${f1(w.tStart)}-${f1(w.tEnd)} s, flown with one set of values, is analysed, and the values in effect then are used.`);
  }
  if (w.transition) out.push(`The first ${TRANSITION_S} s after the switch from ${modeName(w.transition.from)} to ${modeName(w.transition.to)} (the transition) are left out.`);
  return out;
}

/** a result that refuses the file (not a DataFlash log, or empty), shaped like a normal result */
function refusal(file, fam, log) {
  const msg = log.integrity.message;
  const axes = {}; for (const a of ['roll', 'pitch', 'yaw']) axes[a] = { error: msg };
  return { file, family: fam, error: msg, format: { kind: log.integrity.format, supported: false }, params: {}, paramChanges: 0, paramChangesAll: 0, tuningChanges: [], flightWindow: { tStart: null, tEnd: null },
    integrity: Object.assign({}, log.integrity, { timeIssues: {}, notes: [msg] }), axes, counts: log.counts, noise: { available: false, reason: msg } };
}

// ------------------------------------------------------------------------------------------- analysis

function analyzeLog(file, { preset = 'normal', family = null, tStart = null, tEnd = null } = {}) {
  const log = parseBin(file, { want: WANT });
  const fam = family || detectLogFamily(log);
  if (!log.integrity.supported) return refusal(file, fam, log);
  const timeIssues = cleanLogTimes(log, TIMED);
  const changes = parameterChanges(log);
  const thr = throttleStream(log, fam);
  const found = chooseWindow(log, fam, thr, changes.tuning, tStart, tEnd);
  const none = found && found.none ? found.reason : null;
  const w = none ? null : found;
  const win = { tStart: w ? w.tStart : tStart, tEnd: w ? w.tEnd : tEnd };
  const params = paramsAt(log, win.tStart);
  const air = fam !== 'plane' && thr && win.tStart !== null ? airborneIntervals(thr, win.tStart, win.tEnd) : null;
  const op = notchOperatingPoint(log, fam, win, air);
  const ctx = { thr, air, op };
  const firmware = (log.text.find(x => /Ardu(Copter|Plane|Rover|Sub)|^Ardu/i.test(x.msg)) || log.text[0] || {}).msg;
  // the logger counts the messages it had to drop (DSF.Dp, cumulative)
  let loggerDropped = 0; const dsf = log.msgs.DSF;
  if (dsf && dsf.Dp) for (let i = 0; i < dsf.n; i++) if (dsf.Dp[i] > loggerDropped) loggerDropped = dsf.Dp[i];
  const integrity = Object.assign({}, log.integrity, { timeIssues, loggerDropped });
  const fileNotes = logNotes(integrity, timeIssues, [FAMILIES[fam === 'heli' ? 'copter' : fam].logPid.roll, 'RATE', thr ? thr.name : 'CTUN', 'MODE']);
  integrity.notes = fileNotes.concat(windowNotes(w));
  const out = { file, family: fam, firmware, format: { kind: log.integrity.format, supported: true }, params: pick(params), paramChanges: changes.tuning.length, paramChangesAll: changes.all.length,
    tuningChanges: changes.tuning.slice(0, 50).map(c => ({ t: c.t, name: c.name, from: c.from, to: c.to })), flightWindow: win, integrity, axes: {}, counts: log.counts };
  if (none) {
    // the log's modes never used the analysed controller: refuse instead of analysing the wrong flight
    const msg = `Nothing to analyse for the ${fam} controller: ${none}.`;
    for (const axis of ['roll', 'pitch', 'yaw']) out.axes[axis] = { error: msg };
    out.noise = { available: false, reason: msg };
    integrity.notes.push(msg);
    return out;
  }
  const motors = fam !== 'plane' ? motorStats(log, params, fam, win, air) : null;
  if (motors) out.motors = motors;
  for (const axis of ['roll', 'pitch', 'yaw']) {
    const d = axisData(log, fam, axis, win, ctx);
    if (!d) { out.axes[axis] = { error: 'no rate-loop data for this axis in the log' }; continue; }
    if (fam !== 'plane') { d.motors = motors; d.iStats = iStats(log, fam, axis, d); }
    // the hover-data (time-domain) fit is used only for log analysis: its result is reviewed before writing
    const res = analyzeAxis({ family: fam, axis, data: d, params, preset, gyroOp: Object.assign({}, op, { throttle: d.throttle }), scaler2: d.scaler2, timeDomain: true });
    let nValid = 0, nUse = 0; for (let g = 0; g < d.n; g++) if (d.valid[g]) { nValid++; if (!d.airborne || d.airborne[g]) nUse++; }
    const airS = d.airborne ? nUse / d.fs : null;
    res.dataInfo = { fs: d.fs, rawRateHz: d.rawRate, source: d.source, seconds: nValid / d.fs, usableSeconds: nUse / d.fs, windowSeconds: win.tStart !== null ? win.tEnd - win.tStart : null, segments: d.segs.length,
      longestSegmentS: d.segs.reduce((mx, s) => Math.max(mx, s.g1 - s.g0), 0) / d.fs, gaps: d.gaps.slice(0, 20).map(g => ({ t0: g.t0, t1: g.t1, s: g.t1 - g.t0 })), gapCount: d.gaps.length, excluded: d.excluded, scaler2: d.scaler2, throttle: d.throttle };
    res.warnings.unshift(...integrity.notes, ...axisNotes(d, win, airS));
    // time-series preview (decimated, real samples only) for the UI
    const step = Math.max(1, Math.floor(d.n / 4000));
    res.series = { t: [], target: [], actual: [], u: [] };
    for (let g = 0; g < d.n; g += step) { if (!d.valid[g]) continue; res.series.t.push(+gridTime(d, g).toFixed(3)); res.series.target.push(+d.target[g].toFixed(2)); res.series.actual.push(+d.actual[g].toFixed(2)); res.series.u.push(+d.u[g].toFixed(4)); }
    try { res.issues = axisIssues(res, fam); } catch (e) { res.issues = []; }
    out.axes[axis] = res;
  }
  out.noise = analyzeNoise(log, params, fam, win, { air, op });
  return out;
}

function pick(p) {
  const o = {}; for (const [k, v] of Object.entries(p)) if (k !== '__changes') o[k] = v; return o;
}

module.exports = { analyzeLog, axisData, detectLogFamily, flightWindow, chooseWindow, parameterChanges, paramsAt, isTuningParam, gridTime, gridIndex, iStats, motorStats, WANT, TIMED, Q_MODES };
