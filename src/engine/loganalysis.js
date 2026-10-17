// Log (.bin) analysis: extract per-axis rate-loop data for copter / VTOL / plane, run analyzeAxis,
// and run the gyro-noise / notch analysis when raw IMU data is present.
'use strict';
const { parseBin, splitByInstance } = require('../log/dfreader');
const { analyzeAxis, resample, estimateRate } = require('./analysis');
const { analyzeNoise } = require('./notch');
const { FAMILIES } = require('../model/rules');
const { axisIssues } = require('./issues');

const WANT = ['ATT', 'RATE', 'PIDR', 'PIDP', 'PIDY', 'PIQR', 'PIQP', 'PIQY', 'IMU', 'ISBH', 'ISBD', 'GYR', 'CTUN', 'QTUN', 'MODE', 'AETR', 'VIBE', 'RCOU', 'ESC', 'FTN1', 'FTN2', 'ARSP', 'ATUN', 'ATRP', 'QUIK', 'ANG', 'PARM', 'MSG'];

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

/**
 * Build the uniform per-axis dataset. Returns {fs, t0, target, actual, u, P,I,D,FF, srate, dmod, scaler2, throttle}
 */
function axisData(log, family, axis, { tStart = null, tEnd = null } = {}) {
  const fam = FAMILIES[family === 'heli' ? 'copter' : family];
  const pidName = fam.logPid[axis];
  const pid = log.msgs[pidName];
  const rateMsg = log.msgs.RATE && fam.logRate ? log.msgs.RATE : null;
  const pidRate = pid && pid.n > 200 ? estimateRate(Float64Array.from(pid.TimeUS.subarray(0, Math.min(pid.n, 2000)), v => v / 1e6)) : 0;
  const rateRate = rateMsg && rateMsg.n > 200 ? estimateRate(Float64Array.from(rateMsg.TimeUS.subarray(0, Math.min(rateMsg.n, 2000)), v => v / 1e6)) : 0;
  let t, cols;
  if (pid && pid.n > 200 && pidRate >= rateRate * 0.5) {
    t = Float64Array.from(pid.TimeUS, v => v / 1e6);
    const u = new Float64Array(pid.n);
    for (let i = 0; i < pid.n; i++) u[i] = pid.P[i] + pid.I[i] + pid.D[i] + pid.FF[i] + (pid.DFF ? pid.DFF[i] : 0);
    // copter / VTOL PIDx logs are in rad/s, plane PIDx in deg/s: keep everything in deg/s
    const k = family === 'plane' ? 1 : 57.2957795;
    const tar = k === 1 ? pid.Tar : Float64Array.from(pid.Tar, v => v * k), act = k === 1 ? pid.Act : Float64Array.from(pid.Act, v => v * k);
    cols = { target: tar, actual: act, u, P: pid.P, I: pid.I, D: pid.D, FF: pid.FF, srate: pid.SRate || new Float64Array(pid.n), dmod: pid.Dmod || new Float64Array(pid.n).fill(1) };
    if (family === 'plane' && log.msgs.AETR && log.msgs.AETR.SS) { const a = log.msgs.AETR; const get = sampler(a.TimeUS, a.SS); cols.ss = Float64Array.from(pid.TimeUS, v => get(v / 1e6) || 1); }
  } else if (rateMsg) {
    const r = rateMsg; const [des, act, out] = fam.logRate[axis];
    t = Float64Array.from(r.TimeUS, v => v / 1e6);
    cols = { target: r[des], actual: r[act], u: r[out] };
  } else return null;
  // time window
  let i0 = 0, i1 = t.length;
  if (tStart !== null) while (i0 < t.length && t[i0] < tStart) i0++;
  if (tEnd !== null) while (i1 > i0 && t[i1 - 1] > tEnd) i1--;
  if (i1 - i0 < 200) return null;
  const tt = t.subarray(i0, i1); const sub = {}; for (const k of Object.keys(cols)) sub[k] = cols[k].subarray ? cols[k].subarray(i0, i1) : cols[k].slice(i0, i1);
  const fsRaw = estimateRate(tt);
  const fs = fsRaw >= 300 ? 400 : fsRaw >= 150 ? 200 : fsRaw >= 75 ? 100 : fsRaw >= 40 ? 50 : fsRaw >= 20 ? 25 : 10;
  const rs = resample(tt, sub, fs);
  if (!rs) return null;
  const d = Object.assign({ fs, t0: rs.t0, n: rs.n, rawRate: fsRaw }, rs.cols);
  // plane: speed scaler from AETR.SS (mean of scaler^2 over the window)
  d.scaler2 = 1;
  if (family === 'plane' && log.msgs.AETR && log.msgs.AETR.SS) {
    const a = log.msgs.AETR; let s = 0, c = 0;
    for (let i = 0; i < a.n; i++) { const ts = a.TimeUS[i] / 1e6; if (ts >= tt[0] && ts <= tt[tt.length - 1] && a.SS[i] > 0) { s += a.SS[i] * a.SS[i]; c++; } }
    if (c) d.scaler2 = s / c;
  }
  // throttle for the notch operating point
  const ct = log.msgs.QTUN && family === 'vtol' ? log.msgs.QTUN : log.msgs.CTUN;
  if (ct && ct.ThO) { let s = 0, c = 0; for (let i = 0; i < ct.n; i++) { const ts = ct.TimeUS[i] / 1e6; if (ts >= tt[0] && ts <= tt[tt.length - 1]) { s += ct.ThO[i]; c++; } } d.throttle = c ? s / c : 0; }
  if (ct && ct.ThO && family !== 'plane') d.airborne = airborneMask(ct, d);
  return d;
}

/**
 * Airborne mask on the data grid (copter / VTOL): throttle out above 60% of the typical hover throttle,
 * without the first 3 s after lift-off and the last 2 s before touch-down (ground contact, ground effect).
 * A window that contains a landing (throttle at idle for a while) would otherwise be fitted as if it flew.
 */
function airborneMask(ct, d) {
  const tA = d.t0, tB = d.t0 + (d.n - 1) / d.fs;
  const thr = []; for (let i = 0; i < ct.n; i++) { const ts = ct.TimeUS[i] / 1e6; if (ts >= tA && ts <= tB && ct.ThO[i] > 0.15) thr.push(ct.ThO[i]); }
  if (thr.length < 10) return null;
  thr.sort((a, b) => a - b); const lim = Math.max(0.15, 0.6 * thr[Math.floor(thr.length / 2)]);
  const get = sampler(ct.TimeUS, ct.ThO);
  const m = new Uint8Array(d.n);
  for (let i = 0; i < d.n; i++) m[i] = get(d.t0 + i / d.fs) > lim ? 1 : 0;
  const out = new Uint8Array(d.n); const a = Math.round(3 * d.fs), b = Math.round(2 * d.fs);
  let i = 0;
  while (i < d.n) { if (!m[i]) { i++; continue; } let j = i; while (j < d.n && m[j]) j++; for (let k = i + a; k < j - b; k++) out[k] = 1; i = j; }
  return out;
}

const Q_MODES = new Set([17, 18, 19, 20, 21, 22, 23, 24, 25, 26]);   // QSTABILIZE .. (ArduPlane VTOL modes)
/** mode segments [{t0, t1, mode}] from MODE messages */
function modeSegments(log, tEnd) {
  const m = log.msgs.MODE; if (!m || !m.n) return [];
  const out = [];
  for (let i = 0; i < m.n; i++) { const t0 = m.TimeUS[i] / 1e6; const t1 = i + 1 < m.n ? m.TimeUS[i + 1] / 1e6 : tEnd; out.push({ t0, t1, mode: m.Mode[i] }); }
  return out;
}
/**
 * Flight window: the longest segment where the vehicle is flying (throttle out above idle) and in a mode
 * that uses the analysed controller (VTOL modes for 'vtol', fixed-wing modes for 'plane', any for copter).
 */
function flightWindow(log, family) {
  const ct = family === 'vtol' && log.msgs.QTUN ? log.msgs.QTUN : log.msgs.CTUN;
  if (!ct || !ct.ThO) return null;
  const thr = family === 'plane' ? 5 : 0.15;   // plane CTUN.ThO is in percent
  const tEnd = ct.TimeUS[ct.n - 1] / 1e6;
  const segs = modeSegments(log, tEnd).filter(s => family === 'vtol' ? Q_MODES.has(s.mode) : family === 'plane' ? !Q_MODES.has(s.mode) : true);
  const ranges = segs.length ? segs : [{ t0: 0, t1: tEnd }];
  let best = null;
  for (const r of ranges) {
    let t0 = null, t1 = null;
    for (let i = 0; i < ct.n; i++) { const ts = ct.TimeUS[i] / 1e6; if (ts < r.t0 || ts > r.t1) continue; if (ct.ThO[i] > thr) { if (t0 === null) t0 = ts; t1 = ts; } }
    if (t0 !== null && (!best || t1 - t0 > best[1] - best[0])) best = [t0, t1];
  }
  return best;
}

function analyzeLog(file, { preset = 'normal', family = null, tStart = null, tEnd = null } = {}) {
  const log = parseBin(file, { want: WANT });
  const fam = family || detectLogFamily(log);
  const params = log.params;
  const fw = flightWindow(log, fam);
  const win = { tStart: tStart ?? (fw ? fw[0] : null), tEnd: tEnd ?? (fw ? fw[1] : null) };
  const out = { file, family: fam, firmware: (log.text.find(x => /Ardu(Copter|Plane|Rover|Sub)|^Ardu/i.test(x.msg)) || log.text[0] || {}).msg, params: pick(params), paramChanges: (params.__changes || []).length, flightWindow: win, axes: {}, counts: log.counts };
  const motors = fam !== 'plane' ? motorStats(log, params, fam, win) : null;
  if (motors) out.motors = motors;
  for (const axis of ['roll', 'pitch', 'yaw']) {
    const d = axisData(log, fam, axis, win);
    if (!d) { out.axes[axis] = { error: 'no rate-loop data for this axis in the log' }; continue; }
    if (fam !== 'plane') { d.motors = motors; d.iStats = iStats(log, fam, axis, d); }
    // the hover-data (time-domain) fit is used only for log analysis: its result is reviewed before writing
    const res = analyzeAxis({ family: fam, axis, data: d, params, preset, gyroOp: { throttle: d.throttle }, scaler2: d.scaler2, timeDomain: true });
    res.dataInfo = { fs: d.fs, rawRateHz: d.rawRate, seconds: d.n / d.fs, scaler2: d.scaler2, throttle: d.throttle };
    // time-series preview (decimated) for the UI
    const step = Math.max(1, Math.floor(d.n / 4000));
    res.series = { t: [], target: [], actual: [], u: [] };
    for (let i = 0; i < d.n; i += step) { res.series.t.push(+(d.t0 + i / d.fs).toFixed(3)); res.series.target.push(+d.target[i].toFixed(2)); res.series.actual.push(+d.actual[i].toFixed(2)); res.series.u.push(+d.u[i].toFixed(4)); }
    try { res.issues = axisIssues(res, fam); } catch (e) { res.issues = []; }
    out.axes[axis] = res;
  }
  out.noise = analyzeNoise(log, params, fam, win);
  return out;
}

/** I-term summary of an axis from the PID log (any rate), airborne samples of the data window only */
function iStats(log, family, axis, d) {
  const fam = FAMILIES[family === 'heli' ? 'copter' : family];
  const pid = log.msgs[fam.logPid[axis]]; if (!pid || !pid.I) return null;
  const v = []; let m = 0;
  for (let i = 0; i < pid.n; i++) {
    const k = Math.round((pid.TimeUS[i] / 1e6 - d.t0) * d.fs); if (k < 0 || k >= d.n) continue;
    if (d.airborne && !d.airborne[k]) continue;
    v.push(Math.abs(pid.I[i])); m += pid.I[i];
  }
  if (v.length < 20) return null;
  v.sort((a, b) => a - b);
  return { mean: m / v.length, p95: v[Math.floor(v.length * 0.95)], max: v[v.length - 1], n: v.length };
}

/**
 * Lift-motor outputs while flying: average PWM per motor, how often any motor sits at its upper limit
 * (spin max), and the split between the motors spinning each way (quad X/+), which shows a steady yaw
 * correction the controller has to hold.
 */
function motorStats(log, params, family, win) {
  const rc = log.msgs.RCOU; if (!rc) return null;
  const q = family === 'vtol' ? 'Q_M_' : 'MOT_';
  const chans = [];   // motor k (1-based) -> RCOU column
  for (let n = 1; n <= 14; n++) { const fn = params['SERVO' + n + '_FUNCTION']; if (fn >= 33 && fn <= 40 && rc['C' + n]) chans[fn - 32] = { col: rc['C' + n], n }; }
  const motors = []; for (let k = 1; k < chans.length; k++) if (chans[k]) motors.push(k);
  if (!motors.length) return null;
  const pmin = params[q + 'PWM_MIN'] || params['SERVO' + chans[motors[0]].n + '_MIN'] || 1000, pmax = params[q + 'PWM_MAX'] || params['SERVO' + chans[motors[0]].n + '_MAX'] || 2000;
  const top = pmin + (pmax - pmin) * (params[q + 'SPIN_MAX'] ?? 0.95) - 5;
  const ct = family === 'vtol' && log.msgs.QTUN ? log.msgs.QTUN : log.msgs.CTUN;
  const hov = (() => { if (!ct || !ct.ThO) return null; const v = Array.from(ct.ThO).filter(x => x > 0.15).sort((a, b) => a - b); return v.length ? v[Math.floor(v.length / 2)] : null; })();
  const thr = ct && ct.ThO ? sampler(ct.TimeUS, ct.ThO) : null;
  const sum = {}; for (const k of motors) sum[k] = 0;
  let c = 0, pinned = 0, maxPwm = 0;
  for (let i = 0; i < rc.n; i++) {
    const ts = rc.TimeUS[i] / 1e6; if (win.tStart !== null && (ts < win.tStart || ts > win.tEnd)) continue;
    if (thr && hov && thr(ts) < 0.6 * hov) continue;
    let any = false; for (const k of motors) { const v = chans[k].col[i]; sum[k] += v; if (v >= top) any = true; if (v > maxPwm) maxPwm = v; }
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

function pick(p) {
  const o = {}; for (const [k, v] of Object.entries(p)) if (k !== '__changes') o[k] = v; return o;
}

module.exports = { analyzeLog, axisData, detectLogFamily, flightWindow, WANT };
