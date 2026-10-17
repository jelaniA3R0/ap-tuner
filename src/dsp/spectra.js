// Welch spectra, cross-spectra, coherence and frequency-response estimation.
// This mirrors the method used by ArduPilot's WebTools (AnalyticTune/PIDReview): windowed, overlapped
// FFT windows averaged into Sxx, Syy, Sxy; H = Sxy/Sxx; coherence = |Sxy|^2/(Sxx Syy).
'use strict';
const { FFT, hann } = require('./fft');

/**
 * Average cross spectra between input x and output y (same length, uniformly sampled at fs).
 * windowSize power of two; overlap fraction (0.5 typical). minRms: skip windows with weak input.
 * blocks: optional [[i0, i1)] of unbroken data; windows never straddle a block edge (gaps, ground time).
 * Returns { f, H:{re,im}, coh, Sxx, Syy, nWin }
 */
function freqResponse(x, y, fs, windowSize = 512, overlap = 0.5, minRms = 0, blocks = null) {
  const N = Math.min(x.length, y.length);
  const w = hann(windowSize);
  const fft = new FFT(windowSize);
  const nb = windowSize / 2 + 1;
  const Sxx = new Float64Array(nb), Syy = new Float64Array(nb), Sxyr = new Float64Array(nb), Sxyi = new Float64Array(nb);
  const step = Math.max(1, Math.round(windowSize * (1 - overlap)));
  let nWin = 0;
  const xr = new Float64Array(windowSize), xi = new Float64Array(windowSize), yr = new Float64Array(windowSize), yi = new Float64Array(windowSize);
  for (const [b0, b1] of blocks || [[0, N]]) for (let s = b0; s + windowSize <= Math.min(b1, N); s += step) {
    // detrend (mean removal) + window
    let mx = 0, my = 0; for (let i = 0; i < windowSize; i++) { mx += x[s + i]; my += y[s + i]; } mx /= windowSize; my /= windowSize;
    let rms = 0;
    for (let i = 0; i < windowSize; i++) { const xv = (x[s + i] - mx); rms += xv * xv; xr[i] = xv * w[i]; xi[i] = 0; yr[i] = (y[s + i] - my) * w[i]; yi[i] = 0; }
    rms = Math.sqrt(rms / windowSize);
    if (rms < minRms) continue;
    fft.transform(xr, xi); fft.transform(yr, yi);
    for (let k = 0; k < nb; k++) {
      Sxx[k] += xr[k] * xr[k] + xi[k] * xi[k];
      Syy[k] += yr[k] * yr[k] + yi[k] * yi[k];
      // Sxy = conj(X) * Y
      Sxyr[k] += xr[k] * yr[k] + xi[k] * yi[k];
      Sxyi[k] += xr[k] * yi[k] - xi[k] * yr[k];
    }
    nWin++;
  }
  const f = new Float64Array(nb); for (let k = 0; k < nb; k++) f[k] = k * fs / windowSize;
  const Hre = new Float64Array(nb), Him = new Float64Array(nb), coh = new Float64Array(nb);
  for (let k = 0; k < nb; k++) {
    if (Sxx[k] > 0) { Hre[k] = Sxyr[k] / Sxx[k]; Him[k] = Sxyi[k] / Sxx[k]; }
    const den = Sxx[k] * Syy[k];
    coh[k] = den > 0 ? (Sxyr[k] * Sxyr[k] + Sxyi[k] * Sxyi[k]) / den : 0;
  }
  // scale PSDs to amplitude^2/Hz (Welch, Hann window power correction)
  let wp = 0; for (let i = 0; i < windowSize; i++) wp += w[i] * w[i];
  const scale = nWin > 0 ? 1 / (nWin * wp * fs) : 0;
  for (let k = 0; k < nb; k++) { Sxx[k] *= scale * 2; Syy[k] *= scale * 2; }
  return { f, H: { re: Hre, im: Him }, coh, Sxx, Syy, nWin, windowSize, fs };
}

/**
 * Welch power spectral density of one signal. Returns {f, psd (amplitude^2/Hz), amp (RMS amplitude per bin)}
 * blocks: optional [[i0, i1)] of unbroken data; windows never straddle a block edge.
 */
function psd(x, fs, windowSize = 1024, overlap = 0.5, blocks = null) {
  const w = hann(windowSize);
  const fft = new FFT(windowSize);
  const nb = windowSize / 2 + 1;
  const S = new Float64Array(nb);
  const step = Math.max(1, Math.round(windowSize * (1 - overlap)));
  let nWin = 0;
  const xr = new Float64Array(windowSize), xi = new Float64Array(windowSize);
  for (const [b0, b1] of blocks || [[0, x.length]]) for (let s = b0; s + windowSize <= Math.min(b1, x.length); s += step) {
    let m = 0; for (let i = 0; i < windowSize; i++) m += x[s + i]; m /= windowSize;
    for (let i = 0; i < windowSize; i++) { xr[i] = (x[s + i] - m) * w[i]; xi[i] = 0; }
    fft.transform(xr, xi);
    for (let k = 0; k < nb; k++) S[k] += xr[k] * xr[k] + xi[k] * xi[k];
    nWin++;
  }
  let wp = 0, ws = 0; for (let i = 0; i < windowSize; i++) { wp += w[i] * w[i]; ws += w[i]; }
  const f = new Float64Array(nb), amp = new Float64Array(nb);
  for (let k = 0; k < nb; k++) {
    f[k] = k * fs / windowSize;
    const mean = nWin ? S[k] / nWin : 0;
    amp[k] = Math.sqrt(mean) * (k === 0 || k === nb - 1 ? 1 : Math.SQRT2) / ws; // linear amplitude (peak) of a sinusoid at that bin
    S[k] = nWin ? 2 * mean / (wp * fs) : 0;
  }
  return { f, psd: S, amp, nWin, windowSize, fs };
}

/** number of Welch windows of windowSize (overlap fraction) that fit inside the blocks */
function countWindows(blocks, windowSize, overlap = 0.5) {
  const step = Math.max(1, Math.round(windowSize * (1 - overlap)));
  let k = 0; for (const [b0, b1] of blocks) if (b1 - b0 >= windowSize) k += Math.floor((b1 - b0 - windowSize) / step) + 1;
  return k;
}

/** Integrate PSD between f1 and f2 -> RMS */
function bandRms(sp, f1, f2) {
  let e = 0; const df = sp.f[1] - sp.f[0];
  for (let k = 0; k < sp.f.length; k++) if (sp.f[k] >= f1 && sp.f[k] <= f2) e += sp.psd[k] * df;
  return Math.sqrt(e);
}

module.exports = { freqResponse, psd, bandRms, countWindows };
