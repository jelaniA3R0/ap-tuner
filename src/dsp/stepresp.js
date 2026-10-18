// Closed-loop step-response estimate from target/actual data (Wiener deconvolution),
// the same approach as PID-Analyzer / PIDtoolbox / ArduPilot PIDReview.
'use strict';
const { FFT, hann } = require('./fft');

/**
 * target, actual: equal-length arrays at sample rate fs.
 * Returns { t, mean, count, windows:[...] } with step response over durationS seconds (default 0.5),
 * normalised so a perfect response settles at 1.0. Windows with target peak < minInput are skipped.
 * blocks: optional [[i0, i1)] of unbroken data; windows never straddle a block edge.
 */
function stepResponse(target, actual, fs, { windowSize = 512, durationS = 0.5, minInput = 20, maxWindows = 400, overlapDiv = 8, blocks = null } = {}) {
  const N = Math.min(target.length, actual.length);
  if (N < windowSize) return { t: [], mean: [], count: 0, windows: [] };
  const fft = new FFT(windowSize);
  const w = hann(windowSize);
  const nb = windowSize;
  const stepLen = Math.min(Math.ceil(durationS * fs), windowSize);
  const t = Array.from({ length: stepLen }, (_, i) => i / fs);
  // noise regularisation: smoothed unit step as in PIDReview (gaussian with 25 Hz cutoff)
  const cutHz = 25;
  let lenLpf = Math.max(2, Math.floor(cutHz * windowSize / fs)); lenLpf += lenLpf - 2;
  const radius = Math.ceil(lenLpf * 0.5), sigma = lenLpf / 6;
  const half = windowSize / 2 + 1;
  let sn = new Float64Array(half).fill(1);
  let last = 0;
  for (let j = 0; j < Math.min(lenLpf, half); j++) { sn[j] = last + Math.exp((-0.5 / (sigma * sigma)) * (j - radius) * (j - radius)); last = sn[j]; }
  for (let j = 0; j < Math.min(lenLpf, half); j++) sn[j] /= last;
  const full = new Float64Array(windowSize);
  for (let j = 0; j < half; j++) full[j] = sn[j];
  for (let j = 1; j < half - 1; j++) full[windowSize - j] = sn[j];
  for (let j = 0; j < windowSize; j++) full[j] = 1 / ((1 - full[j] + 1e-9) * 10);
  const step = Math.max(1, Math.round(windowSize / overlapDiv));
  const mean = new Float64Array(stepLen);
  const windows = [];
  let count = 0;
  const xr = new Float64Array(nb), xi = new Float64Array(nb), yr = new Float64Array(nb), yi = new Float64Array(nb);
  const hr = new Float64Array(nb), hi = new Float64Array(nb);
  for (const [b0, b1] of blocks || [[0, N]]) for (let s = b0; s + windowSize <= Math.min(b1, N) && count < maxWindows; s += step) {
    let peak = 0;
    for (let i = 0; i < windowSize; i++) { const a = Math.abs(target[s + i]); if (a > peak) peak = a; }
    if (peak < minInput) continue;
    for (let i = 0; i < windowSize; i++) { xr[i] = target[s + i] * w[i]; xi[i] = 0; yr[i] = actual[s + i] * w[i]; yi[i] = 0; }
    fft.transform(xr, xi); fft.transform(yr, yi);
    for (let k = 0; k < nb; k++) {
      // Pyx = Y * conj(X); Pxx = X conj(X) + sn
      const pr = yr[k] * xr[k] + yi[k] * xi[k], pi = yi[k] * xr[k] - yr[k] * xi[k];
      const pxx = xr[k] * xr[k] + xi[k] * xi[k] + full[k];
      hr[k] = pr / pxx; hi[k] = pi / pxx;
    }
    fft.transform(hr, hi, true);
    const resp = new Float64Array(stepLen);
    let acc = 0;
    for (let l = 0; l < stepLen; l++) { acc += hr[l]; resp[l] = acc; }
    for (let l = 0; l < stepLen; l++) mean[l] += resp[l];
    windows.push({ t0: s / fs, resp: Array.from(resp) });
    count++;
  }
  if (count) for (let l = 0; l < stepLen; l++) mean[l] /= count;
  return { t, mean: Array.from(mean), count, windows };
}

/**
 * Metrics of a normalised step response (final value ~1): rise time 10-90 %, overshoot %, settling time (5 % band).
 * The final value is the mean of the last 30 % of the window. When the response is still moving there (it
 * drifts by more than 2 % between the first and the last third of that tail, or leaves a 5 % band around it),
 * or it ends near zero, it has not settled inside the window: then settled = false and overshoot, rise and
 * settling time are not reported (null / NaN) instead of being measured against a wrong final value.
 */
function stepMetrics(t, y) {
  if (!y.length) return null;
  const n = y.length;
  const k0 = Math.floor(n * 0.7), tail = y.slice(k0);
  const mean = a => a.reduce((s, v) => s + v, 0) / a.length;
  const finalV = mean(tail);
  let peak = -Infinity; for (const v of y) peak = v > peak || Number.isNaN(v) ? v : peak;
  const third = Math.max(1, Math.floor(tail.length / 3));
  const drift = Math.abs(mean(tail.slice(tail.length - third)) - mean(tail.slice(0, third)));
  let spread = 0; for (const v of tail) spread = Math.max(spread, Math.abs(v - finalV));
  const settled = Number.isFinite(finalV) && Math.abs(finalV) > 0.05 && drift <= 0.02 * Math.abs(finalV) && spread <= 0.05 * Math.abs(finalV);
  if (!settled) return { finalValue: Number.isFinite(finalV) ? finalV : null, peak, overshootPct: null, riseTime: NaN, settlingTime: NaN, oscillations: 0, settled: false };
  const overshoot = finalV > 0 ? Math.max(0, (peak - finalV) / finalV * 100) : 0;
  const t10 = t[y.findIndex(v => v >= 0.1 * finalV)] ?? NaN, t90 = t[y.findIndex(v => v >= 0.9 * finalV)] ?? NaN;
  let settle = NaN;
  for (let i = n - 1; i >= 0; i--) { if (Math.abs(y[i] - finalV) > 0.05 * Math.abs(finalV)) { settle = t[Math.min(i + 1, n - 1)]; break; } }
  // oscillation count: number of sign changes of (y - final) after the peak
  const ip = y.indexOf(peak); let osc = 0;
  for (let i = ip + 1; i < n; i++) if (Math.sign(y[i] - finalV) !== Math.sign(y[i - 1] - finalV) && Math.abs(y[i] - finalV) > 0.02) osc++;
  return { finalValue: finalV, peak, overshootPct: overshoot, riseTime: isFinite(t90 - t10) ? t90 - t10 : NaN, settlingTime: settle, oscillations: osc, settled: true };
}

module.exports = { stepResponse, stepMetrics };
