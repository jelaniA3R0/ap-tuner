// Flight-phase helpers shared by the log analysis and the notch analysis: the lift-throttle stream as a 0..1
// fraction, the airborne stretches, and means of other logged signals over the analysed (airborne) time.
'use strict';
const { medianStep } = require('../log/integrity');

/** an ArduPlane log (CTUN.ThO is in percent there, 0..1 in ArduCopter) */
function isPlaneLog(log, family) {
  const txt = log.text.map(x => x.msg).join(' ');
  if (/ArduPlane/.test(txt)) return true;
  if (/ArduCopter/.test(txt)) return false;
  return family === 'plane' || family === 'vtol';
}

/**
 * Throttle out as a 0..1 fraction: { t (s), v, name, percent } from QTUN (VTOL lift motors) or CTUN.
 * ArduPlane logs CTUN.ThO in percent (-100..100), ArduCopter and QTUN as a fraction.
 */
function throttleStream(log, family) {
  const q = log.msgs.QTUN;
  const ct = family === 'vtol' && q && q.ThO ? q : log.msgs.CTUN;
  if (!ct || !ct.ThO || !ct.n) return null;
  const percent = ct !== q && isPlaneLog(log, family);
  return { t: Float64Array.from(ct.TimeUS, v => v / 1e6), v: percent ? Float64Array.from(ct.ThO, x => x / 100) : ct.ThO, name: ct === q ? 'QTUN' : 'CTUN', percent };
}

/** seconds kept out after lift-off and before touch-down (ground contact, ground effect) */
const AFTER_TAKEOFF_S = 3, BEFORE_LANDING_S = 2;

/**
 * Airborne stretches (copter / VTOL): throttle out above 60% of the typical hover throttle in the window (at
 * least 0.15), without the first 3 s after lift-off and the last 2 s before touch-down. A throttle sample is
 * held for at most max(0.5 s, 3 throttle intervals): a gap in the throttle log is not taken as flight.
 * thr: throttleStream; [tA, tB]: analysis window (s). Returns { intervals: [[t0, t1]], lim, hover } or null
 * when the window holds fewer than 10 throttle samples above 0.15.
 */
function airborneIntervals(thr, tA, tB) {
  if (!thr) return null;
  const vals = [];
  for (let i = 0; i < thr.t.length; i++) if (thr.t[i] >= tA && thr.t[i] <= tB && thr.v[i] > 0.15) vals.push(thr.v[i]);
  if (vals.length < 10) return null;
  vals.sort((a, b) => a - b);
  const median = vals[Math.floor(vals.length / 2)];
  const lim = Math.max(0.15, 0.6 * median);
  const hold = Math.max(0.5, 3 * medianStep(thr.t));
  const intervals = [], hov = [];
  const n = thr.t.length;
  let i = 0, afterGap = false;
  while (i < n) {
    if (!(thr.v[i] > lim)) { i++; afterGap = false; continue; }
    let j = i;
    while (j + 1 < n && thr.v[j + 1] > lim && thr.t[j + 1] - thr.t[j] <= hold) j++;
    // a run that ends because the throttle log has a gap (the next sample still flies) is not a landing, and
    // the run after such a gap is not a take-off: no ground margins there
    const endsAtGap = j + 1 < n && thr.v[j + 1] > lim;
    const start = thr.t[i], end = j + 1 < n ? Math.min(thr.t[j + 1], thr.t[j] + hold) : thr.t[j] + hold;
    const a = start + (afterGap ? 0 : AFTER_TAKEOFF_S), b = end - (endsAtGap ? 0 : BEFORE_LANDING_S);
    if (b > a) intervals.push([a, b]);
    afterGap = endsAtGap;
    i = j + 1;
  }
  for (let k = 0; k < n; k++) if (thr.t[k] >= tA && thr.t[k] <= tB && inIntervals(intervals, thr.t[k])) hov.push(thr.v[k]);
  hov.sort((a, b) => a - b);
  return { intervals, lim, hover: hov.length ? hov[Math.floor(hov.length / 2)] : median };
}

/** t inside one of the (sorted, disjoint) intervals [t0, t1) */
function inIntervals(iv, t) {
  let lo = 0, hi = iv.length - 1;
  while (lo <= hi) { const m = (lo + hi) >> 1; if (t < iv[m][0]) hi = m - 1; else if (t >= iv[m][1]) lo = m + 1; else return true; }
  return false;
}

/** total length (s) of the intervals inside [tA, tB] */
function intervalSeconds(iv, tA, tB) {
  let s = 0; for (const [a, b] of iv) s += Math.max(0, Math.min(b, tB) - Math.max(a, tA));
  return s;
}

/**
 * Mean of a logged column over the window and, when given, the airborne intervals; only values above minVal
 * (e.g. spinning motors). null when there is no such sample.
 */
function windowMean(m, field, win, iv, minVal = -Infinity) {
  if (!m || !m[field] || !m.TimeUS) return null;
  let s = 0, c = 0;
  for (let i = 0; i < m.n; i++) {
    const t = m.TimeUS[i] / 1e6, v = m[field][i];
    if (win && win.tStart !== null && (t < win.tStart || t > win.tEnd)) continue;
    if (iv && !inIntervals(iv, t)) continue;
    if (!(v > minVal)) continue;
    s += v; c++;
  }
  return c ? s / c : null;
}

/** samples of an RPM sensor (1 or 2) as {TimeUS, rpm}: the instance format (4.4+) or the old rpm1 / rpm2 fields */
function rpmSensor(log, sensor) {
  const r = log.msgs.RPM; if (!r || !r.n) return null;
  if (r.rpm1) return { n: r.n, TimeUS: r.TimeUS, rpm: sensor === 2 ? r.rpm2 : r.rpm1 };
  if (!r.RPM) return null;
  const inst = r.I || r.Instance;
  if (!inst) return sensor === 1 ? { n: r.n, TimeUS: r.TimeUS, rpm: r.RPM } : null;
  const idx = []; for (let i = 0; i < r.n; i++) if (inst[i] === sensor - 1) idx.push(i);
  return idx.length ? { n: idx.length, TimeUS: Float64Array.from(idx, i => r.TimeUS[i]), rpm: Float64Array.from(idx, i => r.RPM[i]) } : null;
}

/**
 * What the dynamic notch follows, measured over the analysed airborne time: hover throttle (0..1), average ESC
 * motor rpm, RPM sensors, the in-flight FFT's noise frequency, and the logged gyro filter rate (IMU.GHz).
 */
function notchOperatingPoint(log, family, win, air) {
  const iv = air ? air.intervals : null;
  const op = {};
  const thr = throttleStream(log, family);
  if (air) op.throttle = air.hover;
  else if (thr) { const v = []; for (let i = 0; i < thr.t.length; i++) if ((!win || win.tStart === null || (thr.t[i] >= win.tStart && thr.t[i] <= win.tEnd)) && thr.v[i] > 0.15) v.push(thr.v[i]); v.sort((a, b) => a - b); if (v.length) op.throttle = v[Math.floor(v.length / 2)]; }
  const esc = windowMean(log.msgs.ESC, 'RPM', win, iv, 100); if (esc !== null) op.escRpm = esc;
  const r1 = rpmSensor(log, 1), r2 = rpmSensor(log, 2);
  const m1 = r1 ? windowMean(r1, 'rpm', win, iv, 0) : null, m2 = r2 ? windowMean(r2, 'rpm', win, iv, 0) : null;
  if (m1 !== null) op.rpm = m1;
  if (m2 !== null) op.rpm2 = m2;
  const fft = windowMean(log.msgs.FTN1, 'PkAvg', win, iv, 0); if (fft !== null) op.fftHz = fft;
  const imu = log.msgs.IMU;
  if (imu && imu.GHz && imu.n) {
    // the gyro filters run at the IMU's own rate: take the first instance (the usual primary), median value
    const v = []; for (let i = 0; i < imu.n; i++) if ((!imu.I || imu.I[i] === imu.I[0]) && imu.GHz[i] > 0) v.push(imu.GHz[i]);
    v.sort((a, b) => a - b); if (v.length) op.backendHz = v[Math.floor(v.length / 2)];
  }
  return op;
}

module.exports = { isPlaneLog, throttleStream, airborneIntervals, inIntervals, intervalSeconds, windowMean, rpmSensor, notchOperatingPoint, AFTER_TAKEOFF_S, BEFORE_LANDING_S };
