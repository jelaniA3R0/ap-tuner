// Time-domain identification of the rate plant, used when the frequency-domain fit fails because the
// flight had little deliberate stick input (normal hovers: the controller mostly fights gusts and trim).
//
//   r_dot(t) = b * (sat(u)_lag(t - Td) - trim) - a * r(t)
//
// r = body rate (deg/s), u = rate-controller output (mixer units), sat() = the mixer limit (a yaw demand
// beyond what the motors can give has no effect), _lag = first-order motor lag tau. The trim may change
// slowly (one value per chunk, default 10 s): it soaks up steady torque offsets (CG, motor tilt, wind)
// that would otherwise bias b and a. Fitted by least squares on the low-pass filtered equation (the same
// filter on both sides keeps the relation exact) over airborne samples only, with a grid search over
// tau, Td and the mixer limit. Checked by an output-error replay: short pieces are simulated from the
// measured u and compared with the measured rate.
//
// This is an equation-error fit on closed-loop data, so it is less trustworthy than the frequency-domain
// fit on stick-excited data: the result is always labelled lower confidence.
'use strict';

/** 2-pole Butterworth low-pass (bilinear), causal; returns a new array */
function lpf2(x, fs, fc) {
  const n = x.length, y = new Float64Array(n);
  if (!(fc > 0) || fc >= fs / 2) { y.set(x); return y; }
  const K = Math.tan(Math.PI * fc / fs), q = Math.SQRT2, norm = 1 / (1 + K * q + K * K);
  const b0 = K * K * norm, b1 = 2 * b0, b2 = b0, a1 = 2 * (K * K - 1) * norm, a2 = (1 - K * q + K * K) * norm;
  let x1 = x[0], x2 = x[0], y1 = x[0], y2 = x[0];
  for (let i = 0; i < n; i++) { const v = b0 * x[i] + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2; x2 = x1; x1 = x[i]; y2 = y1; y1 = v; y[i] = v; }
  return y;
}

/** first-order lag with time constant tau (exact discretisation) */
function lag1(x, fs, tau) {
  const n = x.length, y = new Float64Array(n);
  if (!(tau > 0)) { y.set(x); return y; }
  const al = 1 - Math.exp(-1 / (fs * tau)); let s = x[0];
  for (let i = 0; i < n; i++) { s += al * (x[i] - s); y[i] = s; }
  return y;
}

const clip = (u, L) => (L ? Float64Array.from(u, v => (v > L ? L : v < -L ? -L : v)) : u);

/** contiguous runs of valid samples, each at least minN long -> [[i0, i1)] */
function segmentsOf(valid, n, minN) {
  if (!valid) return [[0, n]];
  const out = []; let i = 0;
  while (i < n) { if (!valid[i]) { i++; continue; } let j = i; while (j < n && valid[j]) j++; if (j - i >= minN) out.push([i, j]); i = j; }
  return out;
}

/**
 * data: { fs, actual (deg/s), u (mixer), airborne? (Uint8Array mask) } uniform arrays.
 * opts: { axis, fc: equation filter (Hz), chunkS: trim chunk length (s), tdMax (s), taus: [s] }
 * Returns { ok, reason?, fit:{K(=b, deg/s^2 per unit u), a, tau, Td, uLimit, trims[], trimMean, r2, vaf, bRelErr, ...} }
 */
function identifyTimeDomain(data, { fc = null, chunkS = 10, tdMax = 0.1, taus = [0.005, 0.01, 0.02, 0.035, 0.05, 0.075, 0.1, 0.15, 0.2], axis = 'roll' } = {}) {
  const fs = data.fs, n = Math.min(data.actual.length, data.u.length);
  if (!fc) fc = axis === 'yaw' ? 1 : 2;   // the equation error is dominated by gusts/vibration above a few Hz
  fc = Math.min(fc, fs * 0.2);
  const skip = Math.ceil(Math.max(tdMax * fs, 3 * fs / fc)) + 2;   // filter / delay start-up inside each segment
  const segs = segmentsOf(data.airborne, n, skip + Math.round(5 * fs));
  const used = segs.reduce((s, [a, b]) => s + b - a - skip, 0);
  if (used < fs * 8) return { ok: false, reason: 'not enough steady airborne data (' + (Math.max(0, used) / fs).toFixed(1) + ' s)' };
  const r = data.actual.subarray(0, n), u = data.u.subarray(0, n);
  // the aircraft has to move on this axis for anything to be measured (RMS of the rate about its local mean)
  let sr = 0, mr = 0, cnt = 0;
  for (const [a, b] of segs) { let m = 0; for (let i = a + skip; i < b; i++) m += r[i]; m /= Math.max(1, b - a - skip); for (let i = a + skip; i < b; i++) { sr += (r[i] - m) ** 2; cnt++; } }
  const rRms = Math.sqrt(sr / Math.max(1, cnt));
  if (rRms < 2) return { ok: false, reason: 'the aircraft barely moved on this axis (' + rRms.toFixed(1) + ' deg/s RMS): nothing to measure' };
  const rf = lpf2(r, fs, fc);
  const rd = new Float64Array(n);
  for (let i = 1; i < n - 1; i++) rd[i] = (rf[i + 1] - rf[i - 1]) * fs / 2;
  // chunks for the slowly varying trim: inside each segment, ~chunkS long
  const chunks = [];
  for (const [a, b] of segs) {
    const s0 = a + skip, len = b - 1 - s0, k = Math.max(1, Math.round(len / (chunkS * fs)));
    for (let c = 0; c < k; c++) chunks.push([s0 + Math.floor(c * len / k), s0 + Math.floor((c + 1) * len / k)]);
  }
  const tdStep = Math.max(1, Math.round(fs * 0.005)), dMax = Math.round(tdMax * fs);
  let uMaxAbs = 0; for (const [a, b] of segs) for (let i = a; i < b; i++) uMaxAbs = Math.max(uMaxAbs, Math.abs(u[i]));
  const limits = [1.0, 0.9, 0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5, 0.45, 0.4].filter(L => L < uMaxAbs * 0.98);
  const fitAt = (tau, d, L) => fitFixed(rd, lpf2(lag1(clip(u, L), fs, tau), fs, fc), rf, d, chunks);
  const searchTauTd = (L) => {
    let best = null;
    for (const tau of taus) {
      const uf = lpf2(lag1(clip(u, L), fs, tau), fs, fc);
      for (let d = 0; d <= dMax; d += tdStep) { const q = fitFixed(rd, uf, rf, d, chunks); if (q && (!best || q.sse < best.sse)) best = Object.assign(q, { tau, d, L }); }
    }
    return best;
  };
  // coordinate search: lag/delay without a limit, then the mixer limit, then lag/delay again
  let best = searchTauTd(0);
  if (!best) return { ok: false, reason: 'least-squares fit failed (no usable data)' };
  for (const L of limits) { const q = fitAt(best.tau, best.d, L); if (q && q.sse < best.sse * 0.995) best = Object.assign(q, { tau: best.tau, d: best.d, L }); }
  if (best.L) { const q = searchTauTd(best.L); if (q && q.sse <= best.sse) best = q; }
  const fit = { K: best.b, a: best.a, tau: best.tau, Td: best.d / fs, uLimit: best.L || null, uMaxAbs, trims: best.trims, r2: best.r2, fc, chunkS, airborneS: used / fs, segments: segs.length };
  fit.trimMean = best.trims.reduce((s, v) => s + v, 0) / best.trims.length;
  fit.trimMin = Math.min(...best.trims); fit.trimMax = Math.max(...best.trims);
  if (fit.uLimit) { let c = 0, m = 0; for (const [a, b] of segs) for (let i = a; i < b; i++) { m++; if (Math.abs(u[i]) >= fit.uLimit) c++; } fit.limitPct = 100 * c / m; }
  // effective number of independent samples after the low-pass filter, for the uncertainty of b
  fit.nEff = Math.max(10, (best.m / fs) * 2 * fc);
  fit.bRelErr = Math.sqrt(best.sse / Math.max(1, fit.nEff - 2 - chunks.length) * best.invUU) / Math.abs(best.b);
  fit.uRms = best.uRms;   // controller activity after removing the trims (mixer units RMS)
  fit.a = Math.max(1e-3, fit.a);
  const horizon = axis === 'yaw' ? chunkS : 1.5;
  fit.vaf = replay(r, u, fs, fit, chunks, horizon);
  // output-error refinement: the equation fit weighs every wiggle of the rate derivative (gusts, frame
  // vibration); simulating the model over whole pieces of flight weighs the slow, large motions that set
  // the tune (on a weak yaw axis: the heading swings). Kept when it replays the flight better.
  const oe = outputErrorFit(r, u, fs, chunks, horizon, fit);
  fit.equationError = { K: fit.K, a: fit.a, uLimit: fit.uLimit, r2: fit.r2, vaf: fit.vaf };
  const oeSane = oe && oe.K > 0 && oe.K / fit.K < 2 && oe.K / fit.K > 0.5 && oe.a < Math.max(1, 3 * fit.a + 0.5);
  if (oeSane && oe.vaf > fit.vaf + 0.02) { Object.assign(fit, { K: oe.K, a: oe.a, uLimit: oe.uLimit, trims: oe.trims, vaf: oe.vaf, method: 'output error' }); fit.trimMean = oe.trims.reduce((s, v) => s + v, 0) / oe.trims.length; fit.trimMin = Math.min(...oe.trims); fit.trimMax = Math.max(...oe.trims); }
  else fit.method = 'equation error';
  if (fit.uLimit) { let c = 0, m = 0; for (const [a, b] of segs) for (let i = a; i < b; i++) { m++; if (Math.abs(u[i]) >= fit.uLimit) c++; } fit.limitPct = 100 * c / m; } else fit.limitPct = 0;
  let reason = null;
  if (!(fit.K > 0)) reason = 'fitted control power is not positive (the data do not show the airframe answering the controller)';
  else if (best.a < -0.5) reason = 'fitted damping is negative (unstable model): the data are not informative';
  else if (fit.bRelErr > 0.5) reason = 'control power too uncertain (+-' + (fit.bRelErr * 100).toFixed(0) + '%)';
  else if (fit.vaf < 0) reason = 'the model does not replay the measured rate (replay ' + (fit.vaf * 100).toFixed(0) + '%): gusts dominate this axis';
  else if (Math.max(fit.r2, fit.vaf) < 0.3) reason = 'the model explains only ' + (Math.max(fit.r2, fit.vaf) * 100).toFixed(0) + '% of the measured rate';
  // 0..1 quality for display: how well the model replays the flight, and how certain the control power is
  fit.fitQuality = Math.max(0, Math.min(1, Math.max(fit.r2, fit.vaf))) * Math.max(0, 1 - Math.min(1, fit.bRelErr));
  return { ok: !reason, reason, fit };
}

/** LS for fixed lag/delay with per-chunk intercepts (removed by demeaning inside each chunk) */
function fitFixed(rd, uf, rf, d, chunks) {
  let suu = 0, srr = 0, sur = 0, syu = 0, syr = 0, syy = 0, m = 0;
  const means = [];
  for (const [i0, i1] of chunks) {
    const k = i1 - i0; if (k < 20) continue;
    let my = 0, mu = 0, mr = 0;
    for (let i = i0; i < i1; i++) { my += rd[i]; mu += uf[i - d]; mr += rf[i]; }
    my /= k; mu /= k; mr /= k; means.push({ my, mu, mr });
    for (let i = i0; i < i1; i++) {
      const y = rd[i] - my, x1 = uf[i - d] - mu, x2 = -(rf[i] - mr);
      suu += x1 * x1; srr += x2 * x2; sur += x1 * x2; syu += y * x1; syr += y * x2; syy += y * y; m++;
    }
  }
  const det = suu * srr - sur * sur;
  if (!(det > 0) || m < 100) return null;
  const b = (syu * srr - syr * sur) / det, a = (syr * suu - syu * sur) / det;
  const sse = Math.max(0, syy - b * syu - a * syr);
  // chunk mean: my = b (mu - trim) - a mr  ->  trim = mu - (my + a mr) / b
  const trims = means.map(q => q.mu - (q.my + a * q.mr) / b);
  return { b, a, sse, r2: syy > 0 ? 1 - sse / syy : 0, m, invUU: srr / det, trims, uRms: Math.sqrt(suu / m) };
}

/** variance-accounted-for of the rate, simulating segS-second pieces of each chunk from the measured start rate */
function replay(r, u, fs, fit, chunks, segS) {
  const seg = Math.round(segS * fs), dN = Math.round(fit.Td * fs), L = fit.uLimit;
  const al = fit.tau > 0 ? 1 - Math.exp(-1 / (fs * fit.tau)) : 1, eA = Math.exp(-fit.a / fs), gA = (1 - eA) / fit.a;
  const sat = v => (L ? Math.max(-L, Math.min(L, v)) : v);
  let se = 0, sv = 0;
  chunks.forEach(([c0, c1], ci) => {
    const trim = fit.trims[Math.min(ci, fit.trims.length - 1)];
    const sg = Math.max(1, Math.min(seg, c1 - Math.max(c0, dN + 1)));
    for (let s = Math.max(c0, dN + 1); s + sg <= c1; s += sg) {
      let x = r[s], ml = sat(u[s - dN]), mr = 0;
      for (let i = s; i < s + sg; i++) mr += r[i]; mr /= sg;
      for (let i = s; i < s + sg; i++) {
        const e = r[i] - x; se += e * e; sv += (r[i] - mr) * (r[i] - mr);
        ml += al * (sat(u[i - dN]) - ml);
        x = x * eA + fit.K * (ml - trim) * gA;
      }
    }
  });
  return sv > 0 ? 1 - se / sv : 0;
}

/**
 * Output-error fit of b, a and the mixer limit (lag and delay kept from the equation fit). Each chunk is cut
 * into pieces of segS seconds simulated from the measured start rate; the response is linear in the trim,
 * so the chunk trims are solved exactly for every trial (b, a, limit).
 */
function outputErrorFit(r, u, fs, chunks, segS, init) {
  const { nelderMead } = require('./plant');
  const seg = Math.round(segS * fs), dN = Math.round(init.Td * fs);
  const al = init.tau > 0 ? 1 - Math.exp(-1 / (fs * init.tau)) : 1;
  let uMaxAbs = 0; for (const [c0, c1] of chunks) for (let i = c0; i < c1; i++) uMaxAbs = Math.max(uMaxAbs, Math.abs(u[i]));
  const evalAt = (K, a, L) => {
    const eA = Math.exp(-a / fs), gA = (1 - eA) / a;
    const sat = v => (L ? Math.max(-L, Math.min(L, v)) : v);
    let se = 0, sv = 0; const trims = [];
    for (const [c0, c1] of chunks) {
      // free response (trim 0) and the unit-trim response g for each piece
      const res = [], gs = []; let sRg = 0, sGg = 0;
      const sg = Math.max(1, Math.min(seg, c1 - Math.max(c0, dN + 1)));
      for (let s = Math.max(c0, dN + 1); s + sg <= c1; s += sg) {
        let x = r[s], g = 0, ml = sat(u[s - dN]);
        for (let i = s; i < s + sg; i++) {
          res.push(r[i] - x); gs.push(g);
          ml += al * (sat(u[i - dN]) - ml);
          x = x * eA + K * ml * gA; g = g * eA - K * gA;
        }
      }
      for (let k = 0; k < res.length; k++) { sRg += res[k] * gs[k]; sGg += gs[k] * gs[k]; }
      const trim = sGg > 0 ? sRg / sGg : 0; trims.push(trim);
      for (let k = 0; k < res.length; k++) { const e = res[k] - trim * gs[k]; se += e * e; }
      // variance of the measured rate inside each piece
      for (let s = Math.max(c0, dN + 1); s + sg <= c1; s += sg) { let mr = 0; for (let i = s; i < s + sg; i++) mr += r[i]; mr /= sg; for (let i = s; i < s + sg; i++) sv += (r[i] - mr) * (r[i] - mr); }
    }
    return { vaf: sv > 0 ? 1 - se / sv : 0, trims };
  };
  const Ls = [0, 1.0, 0.9, 0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5, 0.45, 0.4].filter(L => !L || L < uMaxAbs * 0.98);
  let best = null;
  for (const L of Ls) {
    const cost = x => 1 - evalAt(Math.exp(x[0]), Math.exp(x[1]), L).vaf;
    const res = nelderMead(cost, [Math.log(Math.max(init.K, 1e-3)), Math.log(Math.max(init.a, 0.02))], { step: 0.4, maxIter: 120, tol: 1e-5 });
    if (!best || res.f < best.f) best = { f: res.f, K: Math.exp(res.x[0]), a: Math.exp(res.x[1]), L };
  }
  if (!best) return null;
  const e = evalAt(best.K, best.a, best.L);
  return { K: best.K, a: best.a, uLimit: best.L || null, trims: e.trims, vaf: e.vaf };
}

module.exports = { identifyTimeDomain, lpf2, lag1 };
