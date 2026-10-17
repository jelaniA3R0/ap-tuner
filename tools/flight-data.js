// Extracts a flight from a SITL / vehicle .BIN log for the comparison report and video, and computes
// the comparison metrics per axis. Usage: node tools/flight-data.js <log.BIN> <family> <out.json>
'use strict';
const fs = require('fs');
const { parseBin } = require('../src/log/dfreader');
const { axisData, flightWindow, WANT } = require('../src/engine/loganalysis');
const { stepResponse, stepMetrics } = require('../src/dsp/stepresp');
const { psd, bandRms } = require('../src/dsp/spectra');
const { nextPow2 } = require('../src/dsp/fft');

function extract(file, family) {
  const log = parseBin(file, { want: WANT.concat(['RCIN', 'POS', 'BARO']) });
  const fw = flightWindow(log, family);
  const win = { tStart: fw ? fw[0] : null, tEnd: fw ? fw[1] : null };
  const out = { file, family, window: win, axes: {}, series: {} };
  // sequence start: first big stick input after the flight window starts (|RC1/RC2 - 1500| > 150)
  const rc = log.msgs.RCIN;
  let seqStart = null;
  if (rc) for (let i = 0; i < rc.n; i++) { const t = rc.TimeUS[i] / 1e6; if (win.tStart !== null && t < win.tStart + 5) continue; if (Math.abs(rc.C1[i] - 1500) > 150 || Math.abs(rc.C2[i] - 1500) > 150) { seqStart = t; break; } }
  out.seqStart = seqStart;
  for (const axis of ['roll', 'pitch']) {
    const d = axisData(log, family, axis, win);
    if (!d) continue;
    const n = d.n;
    // metrics over the sequence (from seqStart to the end of the window)
    const i0 = seqStart !== null ? Math.max(0, Math.round((seqStart - d.t0) * d.fs)) : 0;
    let se = 0, st = 0, cnt = 0; for (let i = i0; i < n; i++) { const e = d.target[i] - d.actual[i]; se += e * e; st += d.target[i] * d.target[i]; cnt++; }
    const rmsErr = Math.sqrt(se / cnt), rmsT = Math.sqrt(st / cnt);
    const err = new Float64Array(n - i0); for (let i = i0; i < n; i++) err[i - i0] = d.actual[i] - d.target[i];
    const wsz = Math.max(64, Math.min(1024, nextPow2(d.fs))); const sp = psd(err, d.fs, wsz, 0.5);
    const osc = bandRms(sp, 2, Math.min(30, d.fs * 0.45));
    const sr = stepResponse(d.target.subarray(i0), d.actual.subarray(i0), d.fs, { windowSize: Math.max(64, Math.min(1024, nextPow2(d.fs * 1.28))), minInput: 15 });
    const met = sr.count ? stepMetrics(sr.t, sr.mean) : null;
    // response lag: delay that maximises the correlation between target and actual (ms)
    let bestLag = 0, bestC = -1; const maxLag = Math.round(0.3 * d.fs);
    for (let L = 0; L <= maxLag; L++) { let c = 0; for (let i = i0 + maxLag; i < n; i++) c += d.target[i - L] * d.actual[i]; if (c > bestC) { bestC = c; bestLag = L; } }
    // peak rates reached
    let pk = 0; for (let i = i0; i < n; i++) pk = Math.max(pk, Math.abs(d.actual[i]));
    out.axes[axis] = { fs: d.fs, seconds: (n - i0) / d.fs, rmsErrorDegs: rmsErr, rmsTargetDegs: rmsT, trackingPct: rmsT > 1 ? 100 * Math.max(0, 1 - rmsErr / rmsT) : null, oscBandDegs: osc, lagMs: 1000 * bestLag / d.fs, peakRateDegs: pk,
      step: met ? { riseMs: met.riseTime * 1000, overshootPct: met.overshootPct, settleMs: met.settlingTime * 1000, finalValue: met.finalValue, windows: sr.count, t: sr.t, y: sr.mean } : null };
    // series for the video / plots at 50 Hz, relative to the sequence start
    const dec = Math.max(1, Math.round(d.fs / 50)); const s = { t: [], target: [], actual: [] };
    for (let i = i0; i < n; i += dec) { s.t.push(+((i - i0) / d.fs).toFixed(3)); s.target.push(+d.target[i].toFixed(2)); s.actual.push(+d.actual[i].toFixed(2)); }
    out.series[axis] = s;
  }
  // angle response (what the pilot feels): ATT DesRoll -> Roll, DesPitch -> Pitch, over the sequence
  const attm = log.msgs.ATT;
  if (attm && attm.DesRoll) {
    const tA = Float64Array.from(attm.TimeUS, v => v / 1e6);
    const { resample, estimateRate } = require('../src/engine/analysis');
    const t0s = seqStart !== null ? seqStart : (win.tStart || tA[0]); const t1s = win.tEnd !== null ? win.tEnd : tA[tA.length - 1];
    const idx = []; for (let i = 0; i < attm.n; i++) if (tA[i] >= t0s && tA[i] <= t1s) idx.push(i);
    if (idx.length > 100) {
      const tt = Float64Array.from(idx, i => tA[i]); const fsA = estimateRate(tt) >= 40 ? 50 : 10;
      for (const [des, act, axis] of [['DesRoll', 'Roll', 'roll'], ['DesPitch', 'Pitch', 'pitch']]) {
        const rs = resample(tt, { d: Float64Array.from(idx, i => attm[des][i]), a: Float64Array.from(idx, i => attm[act][i]) }, fsA);
        if (!rs) continue;
        let se = 0, st = 0; for (let i = 0; i < rs.n; i++) { const e = rs.cols.d[i] - rs.cols.a[i]; se += e * e; st += rs.cols.d[i] * rs.cols.d[i]; }
        const sr = stepResponse(rs.cols.d, rs.cols.a, fsA, { windowSize: Math.max(32, Math.min(256, nextPow2(fsA * 2.56))), durationS: 1.5, minInput: 3 });
        const met = sr.count ? stepMetrics(sr.t, sr.mean) : null;
        if (out.axes[axis]) out.axes[axis].angle = { fs: fsA, rmsErrorDeg: Math.sqrt(se / rs.n), rmsTargetDeg: Math.sqrt(st / rs.n), step: met ? { riseMs: met.riseTime * 1000, overshootPct: met.overshootPct, settleMs: met.settlingTime * 1000, windows: sr.count, t: sr.t, y: sr.mean } : null };
      }
    }
  }
  // attitude + sticks at 50 Hz for the video
  const att = log.msgs.ATT; const tRef = seqStart !== null ? seqStart : (win.tStart || 0);
  if (att) { const s = { t: [], roll: [], pitch: [], yaw: [] }; for (let i = 0; i < att.n; i++) { const t = att.TimeUS[i] / 1e6; if (t < tRef || (win.tEnd !== null && t > win.tEnd)) continue; s.t.push(+(t - tRef).toFixed(3)); s.roll.push(+att.Roll[i].toFixed(2)); s.pitch.push(+att.Pitch[i].toFixed(2)); s.yaw.push(+att.Yaw[i].toFixed(1)); } out.series.att = s; }
  if (rc) { const s = { t: [], roll: [], pitch: [], thr: [] }; for (let i = 0; i < rc.n; i++) { const t = rc.TimeUS[i] / 1e6; if (t < tRef || (win.tEnd !== null && t > win.tEnd)) continue; s.t.push(+(t - tRef).toFixed(3)); s.roll.push(rc.C1[i]); s.pitch.push(rc.C2[i]); s.thr.push(rc.C3[i]); } out.series.rc = s; }
  // gains in the log (boot values + last values)
  const p = log.params; const pre = family === 'vtol' ? 'Q_A_' : 'ATC_';
  out.gains = {}; for (const ax of [['RLL', 'roll'], ['PIT', 'pitch']]) { const g = {}; for (const k of ['P', 'I', 'D', 'FLTT', 'FLTD']) g[k] = p[`${pre}RAT_${ax[0]}_${k}`]; g.angleP = p[`${pre}ANG_${ax[0]}_P`]; out.gains[ax[1]] = g; }
  // last values of changed params
  const last = {}; for (const c of (p.__changes || [])) last[c.name] = c.val;
  out.gainsEnd = {}; for (const ax of [['RLL', 'roll'], ['PIT', 'pitch']]) { const g = {}; for (const k of ['P', 'I', 'D', 'FLTT', 'FLTD']) { const nm = `${pre}RAT_${ax[0]}_${k}`; g[k] = last[nm] !== undefined ? last[nm] : p[nm]; } const an = `${pre}ANG_${ax[0]}_P`; g.angleP = last[an] !== undefined ? last[an] : p[an]; out.gainsEnd[ax[1]] = g; }
  out.firmware = (log.text.find(x => /^Ardu/.test(x.msg)) || {}).msg;
  return out;
}

if (require.main === module) {
  const [file, family, outFile] = process.argv.slice(2);
  const r = extract(file, family);
  fs.writeFileSync(outFile, JSON.stringify(r));
  for (const [a, m] of Object.entries(r.axes)) console.log(a, JSON.stringify({ tracking: m.trackingPct && +m.trackingPct.toFixed(0), rmsErr: +m.rmsErrorDegs.toFixed(2), osc: +m.oscBandDegs.toFixed(2), lagMs: +m.lagMs.toFixed(0), peak: +m.peakRateDegs.toFixed(0), step: m.step && { rise: +m.step.riseMs.toFixed(0), os: +m.step.overshootPct.toFixed(0), settle: +m.step.settleMs.toFixed(0), n: m.step.windows } }));
  console.log('seqStart', r.seqStart, 'window', JSON.stringify(r.window), 'gains', JSON.stringify(r.gains), 'end', JSON.stringify(r.gainsEnd));
}
module.exports = { extract };
