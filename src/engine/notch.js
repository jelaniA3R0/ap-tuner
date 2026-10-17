// Gyro noise analysis and harmonic-notch recommendation from a log.
// Data sources (best first): GYR (raw gyro logging, INS_RAW_LOG_OPT), ISBH/ISBD batches
// (INS_LOG_BAT_MASK), IMU at the loop rate (LOG_BITMASK fast IMU). Produces the pre-filter spectrum,
// the currently configured filter response, the predicted post-filter spectrum, detected peaks and
// a recommended INS_HNTCH_* set with predicted result and phase lag at 10 Hz.
'use strict';
const { psd } = require('../dsp/spectra');
const { splitByInstance } = require('../log/dfreader');
const { C, gyroChain, harmonicNotch, biquadLpf, notchBaseFreq } = require('../model/controller');
const { gyroChainFromParams } = require('./analysis');
const { nextPow2 } = require('../dsp/fft');

/** pick the best gyro data for IMU instance 0 -> {fs, x, y, z, source, prefilter:boolean} */
function gyroSamples(log, win) {
  const inWin = (tUs) => !win || !win.tStart || (tUs / 1e6 >= win.tStart && tUs / 1e6 <= win.tEnd);
  // 1) raw gyro messages
  if (log.msgs.GYR && log.msgs.GYR.n > 2000) {
    const g = splitByInstance(log.msgs.GYR, 'I')[0] || Object.values(splitByInstance(log.msgs.GYR, 'I'))[0];
    const idx = []; for (let i = 0; i < g.n; i++) if (inWin(g.TimeUS[i])) idx.push(i);
    if (idx.length > 2000) {
      const t = idx.map(i => g.SampleUS ? g.SampleUS[i] / 1e6 : g.TimeUS[i] / 1e6);
      const fs = estimateFs(t);
      return { fs, t: Float64Array.from(t), x: Float64Array.from(idx, i => g.GyrX[i] * 57.2958), y: Float64Array.from(idx, i => g.GyrY[i] * 57.2958), z: Float64Array.from(idx, i => g.GyrZ[i] * 57.2958), source: 'GYR raw gyro (' + fs.toFixed(0) + ' Hz)', prefilter: true };
    }
  }
  // 2) batch sampler
  if (log.msgs.ISBH && log.msgs.ISBD && log.msgs.ISBH.n > 2) {
    const h = log.msgs.ISBH, d = log.msgs.ISBD;
    // gyro batches: type 0 = accel, 1 = gyro
    const hdr = {}; for (let i = 0; i < h.n; i++) if (h.type[i] === 1 && h.instance[i] === 0) hdr[h.N[i]] = { mul: h.mul[i], fs: h.smp_rate[i], t: h.TimeUS[i] };
    const xs = [], ys = [], zs = []; let fs = 0; let pre = true;
    for (let i = 0; i < d.n; i++) {
      const H = hdr[d.N[i]]; if (!H || !inWin(H.t)) continue;
      fs = H.fs; const mul = H.mul || 1;
      for (let k = 0; k < 32; k++) { xs.push(d.x[i][k] / mul * 57.2958); ys.push(d.y[i][k] / mul * 57.2958); zs.push(d.z[i][k] / mul * 57.2958); }
    }
    if (xs.length > 2000) {
      const opt = log.params.INS_LOG_BAT_OPT | 0; pre = !(opt & 2);
      return { fs, x: Float64Array.from(xs), y: Float64Array.from(ys), z: Float64Array.from(zs), source: 'IMU batch sampler (' + fs.toFixed(0) + ' Hz, ' + (pre ? 'pre' : 'post') + '-filter)', prefilter: pre, batched: true };
    }
  }
  // 3) IMU message (filtered gyro at the logging rate)
  if (log.msgs.IMU && log.msgs.IMU.n > 2000) {
    const im = splitByInstance(log.msgs.IMU, 'I')[0];
    if (im) {
      const idx = []; for (let i = 0; i < im.n; i++) if (inWin(im.TimeUS[i])) idx.push(i);
      const t = idx.map(i => im.TimeUS[i] / 1e6); const fs = estimateFs(t);
      if (idx.length > 2000) return { fs, t: Float64Array.from(t), x: Float64Array.from(idx, i => im.GyrX[i] * 57.2958), y: Float64Array.from(idx, i => im.GyrY[i] * 57.2958), z: Float64Array.from(idx, i => im.GyrZ[i] * 57.2958), source: 'IMU message (' + fs.toFixed(0) + ' Hz, post-filter)', prefilter: false };
    }
  }
  return null;
}

function estimateFs(t) { const d = []; for (let i = 1; i < Math.min(t.length, 5000); i++) { const x = t[i] - t[i - 1]; if (x > 0) d.push(x); } d.sort((a, b) => a - b); return d.length ? 1 / d[Math.floor(d.length / 2)] : 0; }

/** find spectral peaks: local maxima with prominence over the local median */
function findPeaks(f, amp, { fMin = 10, fMax = Infinity, maxPeaks = 6 } = {}) {
  const peaks = [];
  const n = f.length;
  const med = Float64Array.from(amp).sort()[Math.floor(n / 2)] || 1e-9;
  for (let i = 2; i < n - 2; i++) {
    if (f[i] < fMin || f[i] > fMax) continue;
    if (amp[i] > amp[i - 1] && amp[i] >= amp[i + 1] && amp[i] > amp[i - 2] && amp[i] >= amp[i + 2] && amp[i] > 4 * med) {
      // half-power width
      let lo = i, hi = i; while (lo > 0 && amp[lo] > amp[i] * 0.707) lo--; while (hi < n - 1 && amp[hi] > amp[i] * 0.707) hi++;
      peaks.push({ f: f[i], amp: amp[i], prominence: amp[i] / med, widthHz: f[hi] - f[lo] });
    }
  }
  peaks.sort((a, b) => b.amp - a.amp);
  return peaks.slice(0, maxPeaks).sort((a, b) => a.f - b.f);
}

/** group peaks into a fundamental and its harmonics */
function harmonicFamily(peaks) {
  if (!peaks.length) return null;
  let best = null;
  for (const cand of peaks) {
    const f0 = cand.f; let score = 0; const members = [];
    for (const p of peaks) { const r = p.f / f0; const k = Math.round(r); if (k >= 1 && k <= 8 && Math.abs(r - k) < 0.08) { score += p.amp; members.push({ k, f: p.f, amp: p.amp }); } }
    if (!best || score > best.score) best = { f0, score, members };
  }
  return best;
}

function analyzeNoise(log, params, family, win) {
  const gs = gyroSamples(log, win);
  if (!gs) return { available: false, reason: 'No gyro data at a usable rate. For notch setup log raw gyro: set INS_RAW_LOG_OPT = 9 (H7 boards) or INS_LOG_BAT_MASK = 1, INS_LOG_BAT_OPT = 4, then fly 1-2 minutes.' };
  const fs = gs.fs;
  const winSize = Math.min(2048, nextPow2(fs * 1));
  const spX = psd(gs.x, fs, winSize), spY = psd(gs.y, fs, winSize), spZ = psd(gs.z, fs, winSize);
  const f = Array.from(spX.f);
  const amp = f.map((_, i) => Math.sqrt((spX.amp[i] ** 2 + spY.amp[i] ** 2 + spZ.amp[i] ** 2) / 3));
  const chain = gyroChainFromParams(params, { throttle: hoverThrottle(log, family) });
  const res = { available: true, source: gs.source, fs, prefilter: gs.prefilter, f: f.map(v => +v.toFixed(2)), amp: amp.map(v => +v.toPrecision(4)),
    ampX: Array.from(spX.amp, v => +v.toPrecision(4)), ampY: Array.from(spY.amp, v => +v.toPrecision(4)), ampZ: Array.from(spZ.amp, v => +v.toPrecision(4)), chain };
  const fArr = Float64Array.from(f);
  const Hcur = C.abs(gyroChain(fArr, chain));
  res.currentFilterMag = Array.from(Hcur, v => +v.toPrecision(4));
  if (gs.prefilter) res.postCurrent = amp.map((a, i) => +(a * Hcur[i]).toPrecision(4));
  if (!gs.prefilter) res.note = 'Gyro data is post-filter (already filtered). Peaks you see here are what the controller sees. For a proper notch design, log raw gyro (INS_RAW_LOG_OPT = 9).';
  // peaks on the analysed spectrum (pre-filter if available)
  const peaks = findPeaks(f, amp, { fMin: 8, fMax: fs * 0.48 });
  res.peaks = peaks.map(p => ({ f: +p.f.toFixed(1), amp: +p.amp.toPrecision(3), prominence: +p.prominence.toFixed(1), widthHz: +p.widthHz.toFixed(1) }));
  const fam = harmonicFamily(peaks);
  res.harmonics = fam ? { f0: +fam.f0.toFixed(1), members: fam.members.map(m => ({ k: m.k, f: +m.f.toFixed(1), amp: +m.amp.toPrecision(3) })) } : null;
  // control-band noise (what matters for the D term): RMS in 0-40 Hz pre and post filter
  res.rmsBand = { pre: bandRms(f, amp, 5, Math.min(40, fs / 2)), postCurrent: gs.prefilter ? bandRms(f, res.postCurrent, 5, Math.min(40, fs / 2)) : null };
  // consistency: does the peak show up in most parts of the flight? On post-filter data a single strong
  // part (a gust, a landing bump) or filter-shaped bumps can masquerade as a motor peak.
  res.consistency = peakConsistency(gs, log, family, peaks);
  const cons = res.consistency && fam ? res.consistency.find(c => Math.abs(c.f - fam.f0) < 0.01) : null;
  const unconfirmed = !gs.prefilter && fam && fam.f0 > 15 && (!cons || cons.windows < 5 || cons.share < 0.8);
  if (unconfirmed) {
    const list = (res.consistency || []).map(c => `${c.f.toFixed(1)} Hz in ${c.hits}/${c.windows} parts`).join(', ');
    res.recommendation = `No notch recommended: the log only holds filtered gyro data (${gs.source}) and ${cons && cons.windows < 5 ? 'the flight is too short to check that the peaks repeat (' + cons.windows + ' airborne part(s) of 10 s; 5 needed)' : 'the peaks do not repeat through the flight (' + (list || 'no repeatable peak') + ')'}. A notch at the wrong frequency only adds delay. To find the motor noise properly, enable the batch sampler (INS_LOG_BAT_MASK = 1, INS_LOG_BAT_OPT = 0: pre-filter gyro) or raw gyro logging (INS_RAW_LOG_OPT = 9 on H7 boards such as the Cube Orange+), or use ESC telemetry RPM (the notch then follows the motors directly), and fly 1-2 minutes of hover.`;
    res.notchUnconfirmed = { f0: +fam.f0.toFixed(1) };
  }
  // recommendation
  if (fam && fam.f0 > 15 && !unconfirmed) {
    const thr = hoverThrottle(log, family) || 0;
    const escRpm = escMeanRpm(log, win);
    const rec = recommendNotch(fam, peaks, fs, params, thr, escRpm, family);
    const chainNew = Object.assign({}, chain, { notches: [rec.notchDef], gyroFilterHz: rec.gyroFilterHz });
    const Hnew = C.abs(gyroChain(fArr, chainNew));
    res.recommended = rec;
    // pre-filter data: predicted = measured * |H_new|; post-filter data: predicted = measured * |H_new| / |H_current|
    res.recommended.postPredicted = amp.map((a, i) => +(a * Hnew[i] / (gs.prefilter ? 1 : Math.max(Hcur[i], 1e-3))).toPrecision(4));
    if (!gs.prefilter) res.recommended.notes.push('Prediction is approximate: the log holds filtered gyro data, so the true (larger) pre-filter noise is unknown.');
    res.recommended.rmsBandPost = bandRms(f, res.recommended.postPredicted, 5, Math.min(40, fs / 2));
    res.recommended.phaseLag10Hz = { current: phaseLagAt(chain, 10), proposed: phaseLagAt(chainNew, 10) };
    if (cons && cons.windows) res.recommended.notes.push(`The ${fam.f0.toFixed(1)} Hz peak appears in ${cons.hits} of ${cons.windows} parts of the flight.`);
    if (!gs.prefilter) res.recommended.notes.push('Confirm with pre-filter data before relying on it: INS_LOG_BAT_MASK = 1, INS_LOG_BAT_OPT = 0 (batch sampler) or ESC telemetry RPM.');
  } else if (!fam) res.recommendation = 'No clear motor noise peak above 15 Hz: vibration is low or the data is too short.';
  return res;
}

/**
 * Split the gyro data into ~10 s parts (airborne parts only when throttle is known), find the peaks in each
 * and count, for every peak of the whole-flight spectrum, in how many parts it shows up (within 8% / 2 Hz).
 */
function peakConsistency(gs, log, family, peaks) {
  if (!peaks || !peaks.length) return [];
  const fs = gs.fs, segN = Math.round(10 * fs), winSize = Math.min(1024, nextPow2(fs * 1));
  const ct = family === 'vtol' && log.msgs.QTUN ? log.msgs.QTUN : log.msgs.CTUN;
  const hov = hoverThrottle(log, family);
  const thrAt = (() => { if (!gs.t || !ct || !ct.ThO || !hov) return null; let j = 0; return tSec => { const tu = tSec * 1e6; while (j < ct.n - 1 && ct.TimeUS[j + 1] <= tu) j++; return ct.ThO[j]; }; })();
  const parts = [];
  for (let s0 = 0; s0 + segN <= gs.x.length; s0 += segN) {
    if (thrAt && thrAt(gs.t[s0 + (segN >> 1)]) < 0.6 * hov) continue;   // on the ground / spooling
    const sl = a => a.subarray(s0, s0 + segN);
    const sx = psd(sl(gs.x), fs, winSize), sy = psd(sl(gs.y), fs, winSize), sz = psd(sl(gs.z), fs, winSize);
    const f = Array.from(sx.f), amp = f.map((_, i) => Math.sqrt((sx.amp[i] ** 2 + sy.amp[i] ** 2 + sz.amp[i] ** 2) / 3));
    parts.push(findPeaks(f, amp, { fMin: 8, fMax: fs * 0.48, maxPeaks: 8 }).map(p => p.f));
  }
  return peaks.map(p => {
    const tol = Math.max(2, 0.08 * p.f);
    const hits = parts.filter(list => list.some(f => Math.abs(f - p.f) <= tol)).length;
    return { f: p.f, hits, windows: parts.length, share: parts.length ? hits / parts.length : 0 };
  });
}

function bandRms(f, amp, f1, f2) { let e = 0; for (let i = 0; i < f.length; i++) if (f[i] >= f1 && f[i] <= f2) e += amp[i] * amp[i] / 2; return +Math.sqrt(e).toFixed(3); }

function phaseLagAt(chain, fHz) {
  const H = gyroChain(Float64Array.from([fHz]), chain);
  return +(-Math.atan2(H.im[0], H.re[0]) * 180 / Math.PI).toFixed(1);
}

function hoverThrottle(log, family) {
  const ct = family === 'vtol' && log.msgs.QTUN ? log.msgs.QTUN : log.msgs.CTUN;
  if (!ct || !ct.ThO) return null;
  const v = Array.from(ct.ThO).filter(x => x > 0.15).sort((a, b) => a - b);
  return v.length ? v[Math.floor(v.length / 2)] : null;
}

function escMeanRpm(log, win) {
  const e = log.msgs.ESC; if (!e || !e.RPM) return null;
  let s = 0, c = 0; for (let i = 0; i < e.n; i++) if (e.RPM[i] > 100) { s += e.RPM[i]; c++; }
  return c ? s / c : null;
}

/**
 * Recommend a harmonic notch: throttle-based (mode 1) by default, ESC telemetry (mode 3) when RPM data
 * exists in the log, following the ArduPilot wiki rules: FREQ = hover peak, REF = hover throttle,
 * BW = FREQ/2, HMNCS from the harmonics that carry energy, triple notch for broad peaks.
 */
function recommendNotch(fam, peaks, fs, params, hoverThr, escRpm, family) {
  const f0 = fam.f0;
  const members = fam.members;
  let hmncs = 0; const maxAmp = Math.max(...members.map(m => m.amp));
  for (const m of members) if (m.amp > 0.15 * maxAmp && m.k <= 4) hmncs |= 1 << (m.k - 1);
  if (!(hmncs & 1)) hmncs |= 1;
  const broad = peaks.some(p => Math.abs(p.f - f0) < 2 && p.widthHz > f0 * 0.12);
  const opts = broad ? 16 : 0;
  const bw = +(f0 / 2).toFixed(1);
  let mode = 1, ref = +(hoverThr || 0.3).toFixed(2), why = 'throttle-based tracking (mode 1): FREQ = noise peak at hover, REF = hover throttle';
  if (escRpm && Math.abs(escRpm / 60 - f0) / f0 < 0.25) { mode = 3; ref = 1; why = 'ESC telemetry RPM tracking (mode 3): the ESC RPM matches the noise peak'; }
  else if (escRpm && Math.abs(escRpm / 60 * 2 - f0) / f0 < 0.25) { mode = 3; ref = 2; why = 'ESC telemetry RPM tracking (mode 3): peak is twice the motor RPM (two-blade passing), REF = 2'; }
  const gyroCur = params.INS_GYRO_FILTER ?? 20;
  // with a good notch the gyro LPF can usually be raised: cap at f0/3 and 80 Hz (wiki: 80 Hz 5", 40 Hz 10", 20 Hz 20")
  const gyroFilterHz = Math.max(gyroCur, Math.min(80, Math.round(f0 / 3 / 5) * 5));
  const p = { INS_HNTCH_ENABLE: 1, INS_HNTCH_MODE: mode, INS_HNTCH_FREQ: +f0.toFixed(1), INS_HNTCH_BW: bw, INS_HNTCH_ATT: 40, INS_HNTCH_REF: ref, INS_HNTCH_HMNCS: hmncs, INS_HNTCH_OPTS: opts, INS_HNTCH_FM_RAT: 1 };
  if (family === 'vtol' || family === 'plane') p.INS_HNTCH_FM_RAT = 1;
  return { params: p, gyroFilterHz, why, notchDef: { enable: 1, freq: f0, bw, att: 40, hmncs, opts }, notes: [
    `Fundamental ${f0.toFixed(1)} Hz (${(f0 * 60).toFixed(0)} RPM-equivalent); harmonics with energy: ${members.map(m => m.k).join(', ')}`,
    broad ? 'Peak is broad: triple notch (OPTS bit 4) recommended' : 'Peak is narrow: single notch per harmonic',
    `INS_GYRO_FILTER can be raised to ${gyroFilterHz} Hz once the notch is in place (currently ${gyroCur})`,
    'After uploading, fly again and re-check: the post-filter spectrum should match the prediction.'] };
}

module.exports = { peakConsistency, analyzeNoise, gyroSamples, findPeaks, harmonicFamily, recommendNotch };
