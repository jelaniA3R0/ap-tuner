// Gyro noise analysis and harmonic-notch recommendation from a log.
// Data sources (best first): GYR (raw gyro logging, INS_RAW_LOG_OPT), ISBH/ISBD batches
// (INS_LOG_BAT_MASK), IMU at the loop rate (LOG_BITMASK fast IMU). Produces the pre-filter spectrum,
// the currently configured filter response, the predicted post-filter spectrum, detected peaks and
// a recommended INS_HNTCH_* set with predicted result and phase lag at 10 Hz.
// Only airborne gyro data are used, spectra are averaged over unbroken blocks (logging gaps and batch
// boundaries are never joined), and post-filter data logged too slowly to rule out aliasing get no notch.
'use strict';
const { psd } = require('../dsp/spectra');
const { splitByInstance } = require('../log/dfreader');
const { GAP_FACTOR, medianStep } = require('../log/integrity');
const { C, gyroChain } = require('../model/controller');
const { gyroChainFromParams } = require('./analysis');
const { nextPow2, prevPow2 } = require('../dsp/fft');
const { throttleStream, airborneIntervals, inIntervals, notchOperatingPoint } = require('./flightdata');

const DEG = 57.2958;

/** unbroken blocks [[i0, i1)] of a sample sequence: split where a sample was left out (index jump) or the time steps over a gap */
function blocksOfSamples(t, idx) {
  const blocks = []; if (!idx.length) return blocks;
  const dt = medianStep(t);
  let s = 0;
  for (let k = 1; k < idx.length; k++) if (idx[k] !== idx[k - 1] + 1 || (dt > 0 && t[k] - t[k - 1] > GAP_FACTOR * dt)) { blocks.push([s, k]); s = k; }
  blocks.push([s, idx.length]);
  return blocks;
}

/** mean sample rate inside unbroken blocks (gaps excluded) */
function blockRate(t, blocks) {
  let num = 0, den = 0; for (const [a, b] of blocks) if (b - a >= 2) { num += b - a - 1; den += t[b - 1] - t[a]; }
  return den > 0 ? +(num / den).toFixed(3) : 0;
}

/**
 * Pick the best gyro data for the first IMU -> {fs, t, x, y, z, blocks, source, prefilter, batched}.
 * Samples outside the window, outside the airborne stretches (air) or not finite are left out; the data are
 * cut into unbroken blocks for the spectra.
 */
function gyroSamples(log, win, air = null) {
  const inWin = tSec => !win || win.tStart === null || win.tStart === undefined || (tSec >= win.tStart && tSec <= win.tEnd);
  const flying = tSec => !air || inIntervals(air.intervals, tSec);
  const finite = (g, i) => Number.isFinite(g.GyrX[i]) && Number.isFinite(g.GyrY[i]) && Number.isFinite(g.GyrZ[i]);
  const fromTable = (g, timeOf) => {
    const idx = [], t = [];
    for (let i = 0; i < g.n; i++) { const ts = timeOf(i); if (inWin(ts) && flying(ts) && finite(g, i)) { idx.push(i); t.push(ts); } }
    const blocks = blocksOfSamples(t, idx);
    return { idx, t: Float64Array.from(t), blocks, fs: blockRate(t, blocks), x: Float64Array.from(idx, i => g.GyrX[i] * DEG), y: Float64Array.from(idx, i => g.GyrY[i] * DEG), z: Float64Array.from(idx, i => g.GyrZ[i] * DEG) };
  };
  // 1) raw gyro messages
  if (log.msgs.GYR && log.msgs.GYR.n > 2000) {
    const split = splitByInstance(log.msgs.GYR, 'I');
    const g = split[0] || Object.values(split)[0];
    // INS_RAW_LOG_OPT bit 2: GYR holds the filtered gyro; bit 3 (pre and post filter): the first instance holds
    // the raw gyro and the filtered copy is logged under instance + number of gyros
    const opt = log.params.INS_RAW_LOG_OPT | 0;
    const post = !!(opt & 4) && !(opt & 8);
    const r = fromTable(g, i => (g.SampleUS ? g.SampleUS[i] : g.TimeUS[i]) / 1e6);
    if (r.idx.length > 2000 && r.fs > 0) return { fs: r.fs, t: r.t, x: r.x, y: r.y, z: r.z, blocks: r.blocks, source: 'GYR ' + (post ? 'filtered' : 'raw') + ' gyro (' + r.fs.toFixed(0) + ' Hz' + (post ? ', post-filter: INS_RAW_LOG_OPT bit 2' : '') + ')', prefilter: !post };
  }
  // 2) batch sampler: every batch is its own block (the sampler pauses and switches sensors between batches)
  if (log.msgs.ISBH && log.msgs.ISBD && log.msgs.ISBH.n > 2) {
    const h = log.msgs.ISBH, d = log.msgs.ISBD;
    // gyro batches: type 0 = accel, 1 = gyro
    const hdr = {}; for (let i = 0; i < h.n; i++) if (h.type[i] === 1 && h.instance[i] === 0) hdr[h.N[i]] = { mul: h.mul[i], fs: h.smp_rate[i], t: h.TimeUS[i] / 1e6 };
    const xs = [], ys = [], zs = [], blocks = []; let fs = 0, cur = null, lastSeq = null, start = 0;
    for (let i = 0; i < d.n; i++) {
      const H = hdr[d.N[i]]; if (!H || !inWin(H.t) || !flying(H.t)) continue;
      if (!fs) fs = H.fs;
      if (H.fs !== fs) continue;
      const seq = d.seqno ? d.seqno[i] : null;
      // a new batch, or a missing record inside one, starts a new block
      if (cur !== d.N[i] || (seq !== null && lastSeq !== null && seq !== lastSeq + 1)) { if (xs.length > start) blocks.push([start, xs.length]); start = xs.length; cur = d.N[i]; }
      lastSeq = seq;
      const mul = H.mul || 1;
      for (let k = 0; k < 32; k++) { xs.push(d.x[i][k] / mul * DEG); ys.push(d.y[i][k] / mul * DEG); zs.push(d.z[i][k] / mul * DEG); }
    }
    if (xs.length > start) blocks.push([start, xs.length]);
    if (xs.length > 2000 && fs > 0) {
      const opt = log.params.INS_LOG_BAT_OPT | 0; const pre = !(opt & 2);
      return { fs, x: Float64Array.from(xs), y: Float64Array.from(ys), z: Float64Array.from(zs), blocks, source: 'IMU batch sampler (' + fs.toFixed(0) + ' Hz, ' + (pre ? 'pre' : 'post') + '-filter, ' + blocks.length + ' batches)', prefilter: pre, batched: true };
    }
  }
  // 3) IMU message (filtered gyro at the logging rate)
  if (log.msgs.IMU && log.msgs.IMU.n > 2000) {
    const im = splitByInstance(log.msgs.IMU, 'I')[0];
    if (im) {
      const r = fromTable(im, i => im.TimeUS[i] / 1e6);
      if (r.idx.length > 2000 && r.fs > 0) return { fs: r.fs, t: r.t, x: r.x, y: r.y, z: r.z, blocks: r.blocks, source: 'IMU message (' + r.fs.toFixed(0) + ' Hz, post-filter)', prefilter: false };
    }
  }
  return null;
}

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


/**
 * Gyro noise spectrum and notch recommendation over the analysed window.
 * opts: { air (airborne stretches, null for planes), op (notch operating point) }; computed here from the log
 * when not given.
 */
function analyzeNoise(log, params, family, win, opts = {}) {
  let { air, op } = opts;
  const w = win && win.tStart !== null && win.tStart !== undefined ? win : null;
  if (air === undefined) { const thr = throttleStream(log, family); air = family !== 'plane' && thr ? airborneIntervals(thr, w ? w.tStart : -Infinity, w ? w.tEnd : Infinity) : null; }
  if (op === undefined) op = notchOperatingPoint(log, family, w || { tStart: null, tEnd: null }, air);
  const gs = gyroSamples(log, win, air);
  if (!gs) return { available: false, reason: 'No gyro data at a usable rate' + (air ? ' while flying' : '') + '. For notch setup log raw gyro: set INS_RAW_LOG_OPT = 9 (H7 boards) or INS_LOG_BAT_MASK = 1, INS_LOG_BAT_OPT = 4, then fly 1-2 minutes.' };
  const fs = gs.fs;
  const longest = gs.blocks.reduce((m, [a, b]) => Math.max(m, b - a), 0);
  const winSize = Math.min(2048, nextPow2(fs * 1), prevPow2(longest));
  if (winSize < 64) return { available: false, reason: `The gyro data have no unbroken stretch long enough for a spectrum (longest ${(longest / fs).toFixed(2)} s): the log has gaps or dropouts.` };
  const spX = psd(gs.x, fs, winSize, 0.5, gs.blocks), spY = psd(gs.y, fs, winSize, 0.5, gs.blocks), spZ = psd(gs.z, fs, winSize, 0.5, gs.blocks);
  const f = Array.from(spX.f);
  const amp = f.map((_, i) => Math.sqrt((spX.amp[i] ** 2 + spY.amp[i] ** 2 + spZ.amp[i] ** 2) / 3));
  const chain = gyroChainFromParams(params, op);
  const nUse = gs.blocks.reduce((s, [a, b]) => s + b - a, 0);
  const res = { available: true, source: gs.source, fs, prefilter: gs.prefilter, f: f.map(v => +v.toFixed(2)), amp: amp.map(v => +v.toPrecision(4)),
    ampX: Array.from(spX.amp, v => +v.toPrecision(4)), ampY: Array.from(spY.amp, v => +v.toPrecision(4)), ampZ: Array.from(spZ.amp, v => +v.toPrecision(4)), chain,
    dataInfo: { seconds: nUse / fs, blocks: gs.blocks.length, longestBlockS: longest / fs, windows: spX.nWin, airborneOnly: !!air } };
  const fArr = Float64Array.from(f);
  const Hcur = C.abs(gyroChain(fArr, chain));
  res.currentFilterMag = Array.from(Hcur, v => +v.toPrecision(4));
  if (gs.prefilter) res.postCurrent = amp.map((a, i) => +(a * Hcur[i]).toPrecision(4));
  if (!gs.prefilter) res.note = 'Gyro data is post-filter (already filtered). Peaks you see here are what the controller sees. For a proper notch design, log raw gyro (INS_RAW_LOG_OPT = 9).';
  const gyroHz = params.INS_GYRO_FILTER ?? 20;
  // aliasing: filtered gyro sampled at the logging rate still holds what the gyro low-pass lets through above
  // fs/2 (it is only 19 dB down at 3x its cut-off), and that folds to wrong, lower frequencies
  if (!gs.prefilter && fs / 2 < 3 * (gyroHz > 0 ? gyroHz : 20)) {
    res.aliasRisk = true; res.peaks = []; res.harmonics = null; res.consistency = [];
    res.rmsBand = { pre: bandRms(f, amp, 5, Math.min(40, fs / 2)), postCurrent: null };
    res.recommendation = `No notch recommended: the gyro data in this log are logged at only ${fs.toFixed(0)} Hz after the ${gyroHz} Hz gyro filter, which still lets through vibration above ${(fs / 2).toFixed(0)} Hz. That vibration folds down to wrong, lower frequencies (aliasing), so the peaks of this spectrum cannot be trusted. Log the gyro faster or before the filters (batch sampler INS_LOG_BAT_MASK = 1, or raw gyro logging INS_RAW_LOG_OPT = 9 on H7 boards), or use ESC telemetry RPM.`;
    return res;
  }
  // peaks on the analysed spectrum (pre-filter if available)
  const peaks = findPeaks(f, amp, { fMin: 8, fMax: fs * 0.48 });
  res.peaks = peaks.map(p => ({ f: +p.f.toFixed(1), amp: +p.amp.toPrecision(3), prominence: +p.prominence.toFixed(1), widthHz: +p.widthHz.toFixed(1) }));
  const fam = harmonicFamily(peaks);
  res.harmonics = fam ? { f0: +fam.f0.toFixed(1), members: fam.members.map(m => ({ k: m.k, f: +m.f.toFixed(1), amp: +m.amp.toPrecision(3) })) } : null;
  // control-band noise (what matters for the D term): RMS in 0-40 Hz pre and post filter
  res.rmsBand = { pre: bandRms(f, amp, 5, Math.min(40, fs / 2)), postCurrent: gs.prefilter ? bandRms(f, res.postCurrent, 5, Math.min(40, fs / 2)) : null };
  // consistency: does the peak show up in most parts of the flight? On post-filter data a single strong
  // part (a gust, a landing bump) or filter-shaped bumps can masquerade as a motor peak.
  res.consistency = peakConsistency(gs, peaks);
  const cons = res.consistency && fam ? res.consistency.find(c => Math.abs(c.f - fam.f0) < 0.01) : null;
  const unconfirmed = !gs.prefilter && fam && fam.f0 > 15 && (!cons || cons.windows < 5 || cons.share < 0.8);
  if (unconfirmed) {
    const list = (res.consistency || []).map(c => `${c.f.toFixed(1)} Hz in ${c.hits}/${c.windows} parts`).join(', ');
    const short = cons && cons.windows < 5 ? (cons.windows === 0 && gs.blocks.length > 1 ? 'the gyro data have no unbroken airborne stretch of 10 s to check that the peaks repeat (the log has gaps or dropouts)' : 'the flight is too short to check that the peaks repeat (' + cons.windows + ' airborne part(s) of 10 s; 5 needed)') : null;
    res.recommendation = `No notch recommended: the log only holds filtered gyro data (${gs.source}) and ${short || 'the peaks do not repeat through the flight (' + (list || 'no repeatable peak') + ')'}. A notch at the wrong frequency only adds delay. To find the motor noise properly, enable the batch sampler (INS_LOG_BAT_MASK = 1, INS_LOG_BAT_OPT = 0: pre-filter gyro) or raw gyro logging (INS_RAW_LOG_OPT = 9 on H7 boards such as the Cube Orange+), or use ESC telemetry RPM (the notch then follows the motors directly), and fly 1-2 minutes of hover.`;
    res.notchUnconfirmed = { f0: +fam.f0.toFixed(1) };
  }
  // recommendation
  if (fam && fam.f0 > 15 && !unconfirmed) {
    const thr = op.throttle || 0;
    const rec = recommendNotch(fam, peaks, fs, params, thr, op.escRpm || null, family);
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
 * Cut the (airborne) gyro data into 10 s parts inside unbroken blocks, find the peaks in each and count, for
 * every peak of the whole-flight spectrum, in how many parts it shows up (within 8% / 2 Hz).
 */
function peakConsistency(gs, peaks) {
  if (!peaks || !peaks.length) return [];
  const fs = gs.fs, segN = Math.round(10 * fs), winSize = Math.min(1024, nextPow2(fs * 1));
  const parts = [];
  for (const [b0, b1] of gs.blocks) {
    for (let s0 = b0; s0 + segN <= b1; s0 += segN) {
      const sl = a => a.subarray(s0, s0 + segN);
      const sx = psd(sl(gs.x), fs, winSize), sy = psd(sl(gs.y), fs, winSize), sz = psd(sl(gs.z), fs, winSize);
      const f = Array.from(sx.f), amp = f.map((_, i) => Math.sqrt((sx.amp[i] ** 2 + sy.amp[i] ** 2 + sz.amp[i] ** 2) / 3));
      parts.push(findPeaks(f, amp, { fMin: 8, fMax: fs * 0.48, maxPeaks: 8 }).map(p => p.f));
    }
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
