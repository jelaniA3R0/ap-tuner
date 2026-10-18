// Log-analysis data integrity and signal processing (audit 2026-10-17, roadmap R1). The cases come from the
// audit's independent checks (checks/agentD spectral_tests T1-T18, more_tests, nan_tests, formats_test,
// dropout_test; checks/agentE check3, check5, check14 with indep_sim.js); every expected value is worked out
// here from the signal definition, a closed form or an independent computation, never taken from the app.
//   node test/logintegrity.test.js
'use strict';
const assert = require('assert');
const fs = require('fs');
const os = require('os');
const path = require('path');
const { spawnSync } = require('child_process');
const { parseBin, sniffFormat, f16 } = require('../src/log/dfreader');
const { cleanTimes, splitAtGaps, cleanLogTimes } = require('../src/log/integrity');
const L = require('../src/engine/loganalysis');
const { diagnostics, analyzeAxis, gyroChainFromParams, localPeak, noiseFromActual, usableMask, blocksOf } = require('../src/engine/analysis');
const { analyzeNoise } = require('../src/engine/notch');
const { notchBaseFreq, harmonicNotch, C } = require('../src/model/controller');
const { fitPlant, evalPlant } = require('../src/model/plant');
const { psd } = require('../src/dsp/spectra');
const { stepMetrics } = require('../src/dsp/stepresp');
const { LogBuilder, hoverLog } = require('./binlog');
const { simulate } = require('./indep_sim');

let passed = 0, failed = 0;
function test(name, fn) {
  try { fn(); passed++; console.log('  ok   ' + name); } catch (e) { failed++; console.log('  FAIL ' + name + '\n       ' + (e && e.stack ? e.stack.split('\n').slice(0, 3).join('\n       ') : e)); }
}
const near = (a, b, tol, what) => assert(Math.abs(a - b) <= tol, `${what}: got ${a}, expected ${b} (+-${tol})`);
// deterministic noise (the audit scripts' generator)
function noise(seed) { let s = seed >>> 0; const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; }; return () => { let u = 0; while (u === 0) u = rnd(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rnd()); }; }
const G = 57.2958;
/** a gyro-only log object as the audit's tests build it (GYR or IMU table + CTUN throttle) */
function gyroLog({ table = 'GYR', fs, seconds, x, y = () => 0, z = () => 0, params = {}, thr = 0.4, percent = false, keep = null }) {
  const ts = []; for (let i = 0; i < fs * seconds; i++) { const t = i / fs; if (!keep || keep(t)) ts.push(t); }
  const n = ts.length, TimeUS = new Float64Array(n), I = new Float64Array(n), X = new Float64Array(n), Y = new Float64Array(n), Z = new Float64Array(n);
  for (let k = 0; k < n; k++) { const t = ts[k]; TimeUS[k] = Math.round((10 + t) * 1e6); X[k] = x(t) / G; Y[k] = y(t) / G; Z[k] = z(t) / G; }
  const m = Math.round(seconds * 10), cT = new Float64Array(m), cTh = new Float64Array(m);
  for (let i = 0; i < m; i++) { cT[i] = Math.round((10 + i / 10) * 1e6); cTh[i] = percent ? thr * 100 : thr; }
  const tab = { n, TimeUS, I, GyrX: X, GyrY: Y, GyrZ: Z }; if (table === 'GYR') tab.SampleUS = TimeUS;
  return { msgs: { [table]: tab, CTUN: { n: m, TimeUS: cT, ThO: cTh } }, params, text: [] };
}

console.log('DataFlash reader (F-79)');

test('file type: empty, telemetry log, text log, CSV, unknown, DataFlash', () => {
  assert.strictEqual(sniffFormat(Buffer.alloc(0)).kind, 'empty');
  const tl = Buffer.alloc(40); tl.writeBigUInt64BE(1760000000000000n, 0); tl[8] = 0xfd;
  assert.strictEqual(sniffFormat(tl).kind, 'tlog');
  assert.strictEqual(sniffFormat(Buffer.from('FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\n')).kind, 'dataflash-text');
  assert.strictEqual(sniffFormat(Buffer.from('TimeUS,Tar,Act\n1000,0.1,0.09\n')).kind, 'text');
  assert.strictEqual(sniffFormat(Buffer.alloc(600, 0x95)).kind, 'unknown');
  assert.strictEqual(sniffFormat(new LogBuilder().fmt(70, 'TST', 'Qf', 'TimeUS,V').buffer()).kind, 'dataflash');
  for (const b of [Buffer.alloc(0), tl, Buffer.from('a,b\n1,2\n')]) { const p = parseBin(b); assert(!p.integrity.supported && p.integrity.message, 'unsupported with a message'); }
});

test("float16 (format character 'g', P3)", () => {
  assert.strictEqual(f16(0x3e00), 1.5); assert.strictEqual(f16(0xc000), -2); assert.strictEqual(f16(0x7bff), 65504);
  assert.strictEqual(f16(0x7c00), Infinity); assert.strictEqual(f16(0xfc00), -Infinity); assert(Number.isNaN(f16(0x7e00)));
  assert.strictEqual(f16(0x0001), Math.pow(2, -24));            // smallest subnormal
  assert.strictEqual(f16(0x3555), 0.333251953125);              // 2^-2 * (1 + 341/1024)
  const L1 = new LogBuilder().fmt(200, 'TST', 'QgfB', 'TimeUS,G,F,B');
  for (let k = 0; k < 3; k++) L1.rec('TST', [1e6 + k, 0x3e00, 2.25, 7]);
  const m = parseBin(L1.buffer()).msgs.TST;
  assert(m && m.n === 3 && m.G[0] === 1.5 && m.F[0] === 2.25 && m.B[2] === 7, 'fields after the float16 are read correctly');
});

test('FMT validation: short length (P4), unknown character, no crash', () => {
  const L1 = new LogBuilder().fmt(70, 'TST', 'Qf', 'TimeUS,V').fmt(201, 'BAD', 'Qffff', 'TimeUS,A,B,C,D', 15).fmt(202, 'UNK', 'Qx', 'TimeUS,X', 12);
  for (let k = 0; k < 5; k++) L1.rec('TST', [1000 * k, k]);
  const p = parseBin(L1.buffer());
  assert.strictEqual(p.integrity.badFmtCount, 2);
  assert(/length 15 does not match its format \(27 bytes\)/.test(p.integrity.badFmts[0].reason), p.integrity.badFmts[0].reason);
  assert(/unknown format character 'x'/.test(p.integrity.badFmts[1].reason));
  assert(!p.msgs.BAD && p.msgs.TST.n === 5);
});

test('records must chain: junk after a record drops that record only, then reading resumes', () => {
  const L1 = new LogBuilder().fmt(70, 'TST', 'Qf', 'TimeUS,V');
  for (let k = 0; k < 10; k++) { L1.rec('TST', [1000 * (k + 1), k]); if (k === 4) L1.raw([0x55]); }
  const p = parseBin(L1.buffer());
  assert.strictEqual(p.msgs.TST.n, 9);
  assert.deepStrictEqual(Array.from(p.msgs.TST.V), [0, 1, 2, 3, 5, 6, 7, 8, 9]);
  assert.strictEqual(p.integrity.skippedBytes, 15 + 1);   // the unverifiable record and the junk byte
  assert.strictEqual(p.integrity.resyncs, 1);
});

test('a corrupted type byte is not decoded as another message', () => {
  const L1 = new LogBuilder().fmt(70, 'TST', 'Qf', 'TimeUS,V').fmt(71, 'TS2', 'Qff', 'TimeUS,A,B');
  for (let k = 0; k < 10; k++) L1.rec('TST', [1000 * (k + 1), k]);
  const b = L1.buffer(); b[89 * 2 + 15 * 5 + 2] = 71;     // record 5 now claims to be a 19-byte TS2
  const p = parseBin(b);
  assert.strictEqual(p.msgs.TST.n, 9); assert(!p.msgs.TS2, 'no TS2 record decoded');
});

test('truncated file, zero padding, a last record that ends in zero bytes', () => {
  const L1 = new LogBuilder().fmt(70, 'TST', 'Qf', 'TimeUS,V');
  for (let k = 0; k < 10; k++) L1.rec('TST', [1000 * (k + 1), k + 1]);
  const b = L1.buffer();
  const cut = parseBin(b.subarray(0, b.length - 5));
  assert.strictEqual(cut.integrity.truncatedBytes, 10); assert.strictEqual(cut.msgs.TST.n, 9);
  const pad = parseBin(Buffer.concat([b, Buffer.alloc(1000)]));
  assert.strictEqual(pad.integrity.paddingBytes, 1000); assert.strictEqual(pad.integrity.truncatedBytes, 0); assert.strictEqual(pad.integrity.skippedBytes, 0); assert.strictEqual(pad.msgs.TST.n, 10);
  const L2 = new LogBuilder().fmt(70, 'TST', 'Qf', 'TimeUS,V'); L2.rec('TST', [1000, 1]).rec('TST', [2000, 0]);   // 0.0 = four zero bytes at the end
  const z = parseBin(L2.buffer());
  assert.strictEqual(z.msgs.TST.n, 2); assert.strictEqual(z.integrity.truncatedBytes, 0); assert.strictEqual(z.integrity.paddingBytes, 0);
});

test('parameter changes include a change back to the boot value', () => {
  const L1 = new LogBuilder().fmt(64, 'PARM', 'QNf', 'TimeUS,Name,Value');
  L1.rec('PARM', [1000, 'ATC_RAT_RLL_P', 0.1]).rec('PARM', [10e6, 'ATC_RAT_RLL_P', 0.2]).rec('PARM', [20e6, 'ATC_RAT_RLL_P', 0.1]).rec('PARM', [25e6, 'ATC_RAT_RLL_P', 0.1]);
  const p = parseBin(L1.buffer()).params;
  assert.strictEqual(p.ATC_RAT_RLL_P, Math.fround(0.1));
  assert.deepStrictEqual(p.__changes.map(c => [c.t, c.val]), [[10, Math.fround(0.2)], [20, Math.fround(0.1)]]);
});

console.log('time stamps and gaps (F-07)');

test('cleanTimes: repeated stamps, single bad stamps, a clock reset', () => {
  let c = cleanTimes([0, 1, 2, 2, 3]); assert.deepStrictEqual(c.keep, [0, 1, 2, 4]); assert.strictEqual(c.duplicates, 1);
  c = cleanTimes([0, 1, 2, -5, 3, 4]); assert.deepStrictEqual(c.keep, [0, 1, 2, 4, 5]); assert.strictEqual(c.outOfOrder, 1);
  c = cleanTimes([0, 1, 2, 1e9, 3, 4, 5]); assert.deepStrictEqual(c.keep, [0, 1, 2, 4, 5, 6]); assert.strictEqual(c.outOfOrder, 1);
  c = cleanTimes([0, 1, 2, 1e9, 2e9, 3, 4, 5, 6]); assert.deepStrictEqual(c.keep, [0, 1, 2, 5, 6, 7, 8]); assert.strictEqual(c.outOfOrder, 2);
  const reset = []; for (let i = 0; i <= 10; i++) reset.push(100 + i); for (let i = 0; i <= 20; i++) reset.push(i);
  c = cleanTimes(reset); assert.strictEqual(c.keep.length, 11); assert.deepStrictEqual(c.reset, { from: 110, to: 0 }); assert.strictEqual(c.droppedAfterReset, 21);
  c = cleanTimes([0, 1, 2, 3, -1]); assert.strictEqual(c.outOfOrder, 1); assert.strictEqual(c.reset, null);
});

test('splitAtGaps: a 10 s gap splits (T6a), one missing sample does not', () => {
  const t = []; for (let i = 0; i <= 1000; i++) t.push(i / 100); for (let i = 2000; i <= 3000; i++) t.push(i / 100);
  const s = splitAtGaps(t);
  assert.deepStrictEqual(s.segments, [[0, 1001], [1001, 2002]]); assert.deepStrictEqual(s.gaps, [{ t0: 10, t1: 20 }]);
  const t2 = []; for (let i = 0; i <= 100; i++) if (i !== 50) t2.push(i / 100);
  assert.strictEqual(splitAtGaps(t2).segments.length, 1);
});

test('cleanLogTimes: per instance, out-of-order records removed and reported', () => {
  const t = [1, 2, 3, 4, 5, 6].map(v => v * 1e6);
  const log = { msgs: { RATE: { n: 6, TimeUS: Float64Array.from([t[0], t[1], 900, t[3], t[4], t[5]]), R: Float64Array.from([1, 2, 3, 4, 5, 6]) },
    IMU: { n: 6, TimeUS: Float64Array.from([t[0], t[0], t[1], t[1], t[2], t[2]]), I: Float64Array.from([0, 1, 0, 1, 0, 1]), GyrX: new Float64Array(6) } } };
  const rep = cleanLogTimes(log, ['RATE', 'IMU']);
  assert.deepStrictEqual(Object.keys(rep), ['RATE']); assert.strictEqual(rep.RATE.outOfOrder, 1);
  assert.deepStrictEqual(Array.from(log.msgs.RATE.R), [1, 2, 4, 5, 6]); assert.strictEqual(log.msgs.IMU.n, 6);
});

test('logging gap in a flight: reported, left out, never filled in; bad samples masked', () => {
  const r = L.analyzeLog(hoverLog({ gap: [20, 25], jitterUs: 800, badAt: { nan: 30, huge: 40 } }));
  const a = r.axes.roll, di = a.dataInfo;
  assert.strictEqual(di.segments, 2); assert.strictEqual(di.gapCount, 1);
  near(di.gaps[0].t0, 20, 0.01, 'gap start'); near(di.gaps[0].t1, 25, 0.01, 'gap end');
  near(di.fs, 300, 0.01, 'native rate');
  assert.deepStrictEqual(di.excluded, { nonFinite: 1, implausible: 1 });
  assert(a.warnings.some(w => /Logging stopped for 5\.0 s/.test(w)), 'gap named in the warnings');
  assert(a.series.t.every(t => t < 20 || t > 25), 'no data shown inside the gap');
  // the error is 2 deg/s while flying and 10 deg/s on the ground: only the flown samples count (F-75)
  near(a.diagnostics.rmsErrorDegs, 2, 1e-5, 'airborne RMS error');
  near(a.diagnostics.saturation.steadyErrorDegs, 2, 1e-5, 'airborne steady error');
  // usable samples = RATE records inside the airborne stretches, counted independently from the record times:
  // throttle 0.5 from 5.0003 s, a gap 20-25 s in the throttle log too, 0 from 55.0003 s; 3 s / 2 s ground margins,
  // none at the gap; held 0.5 s at most
  const air = [[5.0003 + 3, 19.9003 + 0.5], [25.0003, 55.0003 - 2]];
  let n = 0;
  for (let i = 1; i <= 60 * 300; i++) { const tt = i / 300 + 800e-6 * Math.sin(1.7 * i); if (tt >= 20 && tt <= 25) continue; const ts = Math.round(tt * 1e6) / 1e6; if (Math.abs(tt - 30) < 0.5 / 300 || Math.abs(tt - 40) < 0.5 / 300) continue; if (air.some(([p, q]) => ts >= p && ts < q)) n++; }
  near(di.usableSeconds, n / di.fs, 1e-9, 'usable seconds');
});

test('a reset of the log clock: only the part before it is used', () => {
  const a = hoverLog({ seconds: 40, flyTo: 35 });
  const b = hoverLog({ seconds: 20, flyTo: 15 });
  const r = L.analyzeLog(Buffer.concat([a, b]));
  const ti = r.integrity.timeIssues.RATE;
  assert(ti && ti.reset, 'reset reported'); near(ti.reset.fromS, 40, 0.01, 'reset from'); near(ti.reset.toS, 1 / 300, 1e-4, 'reset to');
  assert(r.integrity.notes.some(s => /clock jumps back/.test(s)));
  near(r.flightWindow.tEnd, 34.9003, 1e-6, 'window from the first part');
});

console.log('window, parameters, modes, throttle (F-08, F-52, F-61)');

test('a gain change in flight splits the window; the values in effect are used; housekeeping is not counted', () => {
  const r = L.analyzeLog(hoverLog({ params: { STAT_RUNTIME: 1 }, changes: [{ t: 20, name: 'ATC_RAT_RLL_P', value: 0.2 }, { t: 35, name: 'STAT_RUNTIME', value: 7 }] }));
  assert.strictEqual(r.paramChanges, 1); assert.strictEqual(r.paramChangesAll, 2);
  near(r.flightWindow.tStart, 20.0003, 1e-6, 'window starts after the change'); near(r.flightWindow.tEnd, 54.9003, 1e-6, 'window end');
  assert.strictEqual(r.params.ATC_RAT_RLL_P, Math.fround(0.2)); assert.strictEqual(r.axes.roll.current.P, Math.fround(0.2));
  assert.deepStrictEqual(r.tuningChanges, [{ t: 20, name: 'ATC_RAT_RLL_P', from: Math.fround(0.1), to: Math.fround(0.2) }]);
  assert(r.integrity.notes.some(s => /Tuning parameters changed during the flight \(ATC_RAT_RLL_P 0\.1 -> 0\.2 at 20\.0 s\)/.test(s)));
  const early = L.analyzeLog(hoverLog({ changes: [{ t: 40, name: 'ATC_RAT_RLL_P', value: 0.2 }] }));
  near(early.flightWindow.tEnd, 39.9003, 1e-6, 'longest piece is before the change'); assert.strictEqual(early.params.ATC_RAT_RLL_P, Math.fround(0.1));
  assert(L.isTuningParam('INS_HNTCH_FREQ') && L.isTuningParam('SERVO5_FUNCTION') && !L.isTuningParam('STAT_RUNTIME') && !L.isTuningParam('INS_GYROFFS_X') && !L.isTuningParam('Q_M_THST_HOVER'));
});

test('VTOL modes: THERMAL / AUTOLAND excluded, transition left out, no Q-mode flight refused', () => {
  for (const m of [17, 18, 19, 20, 21, 22, 23, 25]) assert(L.Q_MODES.has(m), 'Q mode ' + m);
  for (const m of [5, 24, 26]) assert(!L.Q_MODES.has(m), 'not a Q mode ' + m);
  const plane = { firmware: 'ArduPlane V4.7.1', thrMsg: 'QTUN' };
  const r = L.analyzeLog(hoverLog(Object.assign({ modes: [{ t: 1, mode: 5 }, { t: 10, mode: 18 }] }, plane)), { family: 'vtol' });
  near(r.flightWindow.tStart, 15.0003, 1e-6, 'window starts 5 s after FBWA -> QHOVER');
  assert(r.integrity.notes.some(s => /first 5 s after the switch from FBWA to QHOVER/.test(s)));
  const none = L.analyzeLog(hoverLog(Object.assign({ modes: [{ t: 1, mode: 24 }] }, plane)), { family: 'vtol' });
  assert(/no flight in a VTOL \(Q\) mode/.test(none.axes.roll.error), none.axes.roll.error);
});

test('plane throttle in percent is used as a 0..1 fraction (F-61, T11)', () => {
  const lg = gyroLog({ fs: 1000, seconds: 60, x: t => 5 * Math.sin(2 * Math.PI * 60.06 * t) + 2 * Math.sin(2 * Math.PI * 120.12 * t), params: { INS_GYRO_FILTER: 20 }, thr: 0.45, percent: true });
  lg.text = [{ t: 0, msg: 'ArduPlane V4.7.1' }];
  const r = analyzeNoise(lg, lg.params, 'plane', null);
  assert(r.recommended, 'a notch is recommended'); assert.strictEqual(r.recommended.params.INS_HNTCH_REF, 0.45);
});

console.log('filter model (F-74, F-78, F-89, F-77)');

test('dynamic notch operating point: FREQ x FM_RAT floor, REF, measured RPM / ESC / FFT', () => {
  const cases = [
    [{ mode: 3, freq: 100, ref: 2, fm_rat: 1 }, { escRpm: 3000 }, 100],   // motors 50 Hz x REF 2
    [{ mode: 3, freq: 40, ref: 1, fm_rat: 1 }, { escRpm: 6000 }, 100],
    [{ mode: 3, freq: 50, ref: 1, fm_rat: 0.8 }, { escRpm: 1800 }, 40],   // 30 Hz is below the 40 Hz floor
    [{ mode: 3, freq: 50, ref: 1, fm_rat: 0.8 }, {}, 40],                 // no ESC data: at the floor
    [{ mode: 2, freq: 30, ref: 1, fm_rat: 1 }, { rpm: 3000 }, 50],
    [{ mode: 5, freq: 30, ref: 1, fm_rat: 1 }, { rpm: 3000, rpm2: 4200 }, 70],
    [{ mode: 4, freq: 60, ref: 1, fm_rat: 0.75 }, { fftHz: 40 }, 45],
    [{ mode: 3, freq: 80, ref: 0, fm_rat: 1 }, { escRpm: 9000 }, 80],     // REF 0: no dynamic update
    [{ mode: 1, freq: 80, ref: 0.25, fm_rat: 0.9 }, { throttle: 0.1 }, 72],
  ];
  for (const [p, op, want] of cases) near(notchBaseFreq(p, op), want, 1e-9, JSON.stringify([p, op]));
  near(notchBaseFreq({ mode: 1, freq: 80, ref: 0.25, fm_rat: 1 }, { throttle: 0.39 }), 80 * Math.sqrt(0.39 / 0.25), 1e-9, 'throttle mode');
  const ch = gyroChainFromParams({ INS_HNTCH_ENABLE: 1, INS_HNTCH_MODE: 3, INS_HNTCH_FREQ: 40, INS_HNTCH_BW: 20, INS_HNTCH_REF: 1, INS_HNTCH_FM_RAT: 1 }, { escRpm: 6000 });
  assert.strictEqual(ch.notches[0].freq, 100); assert.strictEqual(ch.notches[0].refFreq, 40); assert.strictEqual(ch.notches[0].refBw, 20);
});

/** independent ArduPilot NotchFilter magnitude at f (Hz) for centre c, quality Q, attenuation A (linear) */
function fwNotch(f, fs, c, Q, A) {
  const w = 2 * Math.PI * c / fs, al = Math.sin(w) / (2 * Q);
  const b0 = 1 + al * A * A, b1 = -2 * Math.cos(w), b2 = 1 - al * A * A, a0 = 1 + al, a1 = b1, a2 = 1 - al, z = 2 * Math.PI * f / fs;
  return Math.hypot(b0 + b1 * Math.cos(z) + b2 * Math.cos(2 * z), b1 * Math.sin(z) + b2 * Math.sin(2 * z)) / Math.hypot(a0 + a1 * Math.cos(z) + a2 * Math.cos(2 * z), a1 * Math.sin(z) + a2 * Math.sin(2 * z));
}
const Qof = (c, bw) => { const o = Math.log2(c / (c - bw / 2)) * 2; return Math.sqrt(Math.pow(2, o)) / (Math.pow(2, o) - 1); };

test('a moved harmonic notch keeps the Q of its configured FREQ / BW (T10)', () => {
  const c = 80 * Math.sqrt(0.39 / 0.25), Q = Qof(80, 40);
  near(Q, 1.7142857, 1e-6, 'Q from 80 Hz / 40 Hz');
  for (const f of [80, 90, 95, 105, 120]) {
    const app = C.abs(harmonicNotch(Float64Array.from([f]), 1000, { enable: 1, freq: c, bw: 40, att: 40, hmncs: 1, opts: 0, refFreq: 80, refBw: 40 }))[0];
    near(app, fwNotch(f, 1000, c, Q, 0.1), 1e-12, '|H(' + f + ' Hz)|');
  }
  near(fwNotch(90, 1000, c, Q, 0.1), 0.357, 5e-4, 'audit value at 90 Hz');
  // double notch: both halves use the Q of BW/2 and the spread BW / (32 FREQ), fixed from the configured values
  const sp = 40 / (32 * 80), Q2 = Qof(80, 20);
  for (const f of [85, 100, 110]) {
    const app = C.abs(harmonicNotch(Float64Array.from([f]), 1000, { enable: 1, freq: c, bw: 40, att: 40, hmncs: 1, opts: 1, refFreq: 80, refBw: 40 }))[0];
    near(app, fwNotch(f, 1000, c * (1 - sp), Q2, 0.1) * fwNotch(f, 1000, c * (1 + sp), Q2, 0.1), 1e-12, 'double notch at ' + f + ' Hz');
  }
});

test('filters modelled at the logged gyro rate (IMU.GHz, F-89)', () => {
  const p = { INS_GYRO_RATE: 1, INS_GYRO_FILTER: 20 };
  assert.deepStrictEqual([gyroChainFromParams(p, {}).backendHz, gyroChainFromParams(p, {}).backendSource], [2000, 'INS_GYRO_RATE']);
  assert.deepStrictEqual([gyroChainFromParams(p, { backendHz: 3276 }).backendHz, gyroChainFromParams(p, { backendHz: 3276 }).backendSource], [3276, 'IMU.GHz']);
  const r = L.analyzeLog(hoverLog({ imu: { rate: 300, ghz: 3276 } }));
  assert.strictEqual(r.axes.roll.gyro.backendHz, 3276);
});

test('GYR holds filtered data with INS_RAW_LOG_OPT bit 2 (F-77, T13)', () => {
  const mk = opt => gyroLog({ fs: 1000, seconds: 30, x: t => 0.3 * Math.sin(2 * Math.PI * 60.06 * t), params: { INS_RAW_LOG_OPT: opt, INS_GYRO_FILTER: 80, INS_HNTCH_ENABLE: 1, INS_HNTCH_MODE: 0, INS_HNTCH_FREQ: 60, INS_HNTCH_BW: 30, INS_HNTCH_ATT: 40, INS_HNTCH_HMNCS: 1 } });
  const post = analyzeNoise(mk(5), mk(5).params, 'copter', null);
  assert.strictEqual(post.prefilter, false); assert.strictEqual(post.postCurrent, undefined, 'not filtered a second time');
  assert.strictEqual(analyzeNoise(mk(9), mk(9).params, 'copter', null).prefilter, true);   // pre and post: instance 0 is raw
  assert.strictEqual(analyzeNoise(mk(0), mk(0).params, 'copter', null).prefilter, true);
});

console.log('spectra (F-83, F-62, F-76, F-81)');

test('batch-sampler batches are separate Welch blocks (T14)', () => {
  const floors = {};
  for (const mode of ['continuous', 'batched']) {
    const fs = 1000, B = 1024, nb = 40, mul = 1000, g = noise(14); let rs = 14; const rnd = () => { rs = (rs * 1664525 + 1013904223) >>> 0; return rs / 4294967296; };
    const H = { n: 0, TimeUS: [], N: [], type: [], instance: [], mul: [], smp_rate: [] }, D = { n: 0, TimeUS: [], N: [], x: [], y: [], z: [] };
    let tc = 0;
    for (let b = 0; b < nb; b++) {
      const ph = mode === 'batched' ? 2 * Math.PI * rnd() : 0, tb = 10 + b * 3;
      H.TimeUS.push(tb * 1e6); H.N.push(b); H.type.push(1); H.instance.push(0); H.mul.push(mul); H.smp_rate.push(fs); H.n++;
      for (let r = 0; r < B / 32; r++) { const x = new Int16Array(32), y = new Int16Array(32), z = new Int16Array(32); for (let k = 0; k < 32; k++) { const t = mode === 'batched' ? (r * 32 + k) / fs : tc / fs; tc++; x[k] = Math.round((10 * Math.sin(2 * Math.PI * 40 * t + ph) + 0.1 * g()) / G * mul); y[k] = Math.round(0.1 * g() / G * mul); z[k] = Math.round(0.1 * g() / G * mul); } D.TimeUS.push(tb * 1e6 + r * 1000); D.N.push(b); D.x.push(x); D.y.push(y); D.z.push(z); D.n++; }
    }
    for (const o of [H, D]) for (const k of ['TimeUS', 'N', 'type', 'instance', 'mul', 'smp_rate']) if (o[k]) o[k] = Float64Array.from(o[k]);
    const r = analyzeNoise({ msgs: { ISBH: H, ISBD: D }, params: { INS_LOG_BAT_OPT: 0, INS_GYRO_FILTER: 20 }, text: [] }, { INS_LOG_BAT_OPT: 0, INS_GYRO_FILTER: 20 }, 'copter', null);
    const fl = []; for (let k = 0; k < r.f.length; k++) if (r.f[k] > 100 && r.f[k] < 400) fl.push(r.amp[k]); fl.sort((a, b) => a - b);
    floors[mode] = fl[Math.floor(fl.length / 2)];
  }
  const ratio = floors.batched / floors.continuous;
  assert(ratio > 0.9 && ratio < 1.1, 'noise floor of joined-up batches = continuous record (audit before the fix: x1.81), got x' + ratio.toFixed(3));
});

test('raw gyro with dropouts: no spurious peaks from joining across the gaps (T5)', () => {
  const g = noise(5);
  const lg = gyroLog({ fs: 1000, seconds: 60, x: t => 10 * Math.sin(2 * Math.PI * 40 * t) + 0.05 * g(), keep: t => (t % 0.5) >= 0.037 });
  const r = analyzeNoise(lg, lg.params, 'copter', null);
  const df = r.f[1] - r.f[0];
  assert.strictEqual(r.peaks.length, 1, 'one peak, got ' + JSON.stringify(r.peaks.map(p => p.f)));
  assert(Math.abs(r.peaks[0].f - 40) <= df, 'peak at 40 Hz within one bin (' + df.toFixed(2) + ' Hz), got ' + r.peaks[0].f);
});

test('aliasing: 25 Hz rate data with a 30 Hz oscillation is not called a 5 Hz oscillation (T4a)', () => {
  const g = noise(4); const fs = 25, n = 25 * 60;
  const actual = Float64Array.from({ length: n }, (_, i) => 10 * Math.sin(2 * Math.PI * 30 * i / fs + 0.3) + 0.2 * g());
  const d = diagnostics({ fs, target: new Float64Array(n), actual, u: new Float64Array(n) }, {}, 'copter', 'roll');
  assert.strictEqual(d.oscillating, false); assert.strictEqual(d.aliasRisk, true); assert.strictEqual(d.peakAmbiguous, true);
  near(d.peakHz, 5, 0.5, 'the folded peak sits at |30 - 25| = 5 Hz');
  assert(d.grade.notes.some(s => /aliasing/.test(s)));
});

test('aliasing: no notch from post-filter gyro logged too slowly (T12, T12b)', () => {
  const g = noise(12);
  const t12 = gyroLog({ table: 'IMU', fs: 50, seconds: 120, x: t => 2 * Math.sin(2 * Math.PI * 130 * t + 0.1) + 0.2 * g(), y: t => 2 * Math.sin(2 * Math.PI * 130 * t + 1.1) + 0.2 * g(), params: { INS_GYRO_FILTER: 20 } });
  const r12 = analyzeNoise(t12, t12.params, 'copter', null);
  assert(!r12.recommended && r12.aliasRisk && /aliasing/.test(r12.recommendation), 'T12 refused');
  const t12b = gyroLog({ table: 'IMU', fs: 400, seconds: 120, x: t => 0.8 * Math.sin(2 * Math.PI * 250 * t) + 0.4 * Math.sin(2 * Math.PI * 500 * t + 0.7) + 0.05 * g(), params: { INS_GYRO_FILTER: 80 } });
  const r12b = analyzeNoise(t12b, t12b.params, 'copter', null);
  assert(!r12b.recommended && r12b.aliasRisk, 'T12b refused');
  const ok = gyroLog({ table: 'IMU', fs: 300, seconds: 60, x: t => Math.sin(2 * Math.PI * 41 * t), params: { INS_GYRO_FILTER: 20 } });
  assert(!analyzeNoise(ok, ok.params, 'copter', null).aliasRisk, '300 Hz after a 20 Hz filter is alias-free');
});

test('the oscillation peak is a true local maximum, not the band edge (T16)', () => {
  const g = noise(16), fs = 200, n = fs * 120; const red = new Float64Array(n); let s = 0; const a = Math.exp(-2 * Math.PI * 0.3 / fs);
  for (let i = 0; i < n; i++) { s = a * s + (1 - a) * 400 * g(); red[i] = s; }
  const d = diagnostics({ fs, target: new Float64Array(n), actual: red, u: new Float64Array(n) }, {}, 'copter', 'roll');
  assert.strictEqual(d.oscillating, false); assert(d.peakProminence < 10, 'red noise has no prominent peak: ' + d.peakProminence);
  const osc = Float64Array.from(red, (v, i) => v + 8 * Math.sin(2 * Math.PI * 6 * i / fs));
  const d2 = diagnostics({ fs, target: new Float64Array(n), actual: osc, u: new Float64Array(n) }, {}, 'copter', 'roll');
  assert.strictEqual(d2.oscillating, true); near(d2.peakHz, 6, fs / 256, 'oscillation frequency');
  assert.strictEqual(localPeak({ f: [0, 1, 2, 3, 4, 5], psd: [6, 5, 4, 3, 2, 1] }, 0.5, 5), null, 'a falling spectrum has no peak');
});

test('rate data at the native rate: no content invented above its Nyquist frequency (T18)', () => {
  const g = noise(18), sigma = 10;
  const buf = hoverLog({ seconds: 60, flyFrom: 1, flyTo: 59, signal: () => ({ des: 0, act: sigma * g(), out: 0 }) });
  const log = parseBin(buf, { want: L.WANT }); cleanLogTimes(log, L.TIMED);
  const d = L.axisData(log, 'copter', 'roll', { tStart: 2, tEnd: 58 });
  near(d.fs, 300, 0.01, 'grid at the logged 300 Hz, not 400 Hz');
  const blocks = blocksOf(usableMask(d), d.n);
  const sp = psd(d.actual, d.fs, 512, 0.5, blocks);
  near(sp.f[sp.f.length - 1], d.fs / 2, 1e-9, 'top bin is the Nyquist frequency of the data');
  let m = 0, c = 0; for (let k = 0; k < sp.f.length; k++) if (sp.f[k] >= 10 && sp.f[k] <= 140) { m += sp.psd[k]; c++; }
  near(m / c, 2 * sigma * sigma / d.fs, 0.1 * 2 * sigma * sigma / d.fs, 'white-noise level 2 sigma^2 / fs');
  near(noiseFromActual(d.actual, d.fs, blocks).rmsHigh, sigma * Math.sqrt((0.49 * d.fs - 20) / (d.fs / 2)), 0.05 * sigma, 'band RMS 20 Hz .. 0.49 fs (audit before the fix: 7.33 for 9.31)');
});

console.log('fit quality, step metrics, invalid samples (F-59, F-63)');

test('fit quality: the same misfit gives the same quality at any coherence (check5)', () => {
  const truth = { K: 2000, a: 1, tau: 0.05, Td: 0.012 }, f = Float64Array.from({ length: 64 }, (_, i) => (i + 1) * 0.390625);
  const Gt = evalPlant(f, truth), H = { re: new Float64Array(64), im: new Float64Array(64) };
  for (let i = 0; i < 64; i++) { const m = i % 2 ? 1.25 : 1 / 1.25, p = (i % 3 === 0 ? 15 : -15) * Math.PI / 180; H.re[i] = (Gt.re[i] * Math.cos(p) - Gt.im[i] * Math.sin(p)) * m; H.im[i] = (Gt.re[i] * Math.sin(p) + Gt.im[i] * Math.cos(p)) * m; }
  const q = [1.0, 0.8, 0.6, 0.5].map(c => fitPlant(f, H, new Float64Array(64).fill(c), { kind: 'copter', nWin: 90, fMin: 0.3, fMax: 25 }));
  for (const fit of q) { near(fit.fitQuality, q[0].fitQuality, 1e-9, 'quality independent of coherence'); near(fit.fitQuality, 1 / (1 + fit.misfit / 0.3), 1e-12, 'quality from the weighted mean misfit'); }
  near(q[0].fitQuality, 0.632, 0.002, 'audit value at coherence 1');
});

test('step metrics: a "not settled" state; settling uses the 5 % band (check3)', () => {
  const first = (tau, dur, fs) => { const t = [], y = []; for (let i = 0; i < Math.ceil(dur * fs); i++) { t.push(i / fs); y.push(1 - Math.exp(-i / fs / tau)); } return { t, y }; };
  let r = first(0.03, 0.5, 400), m = stepMetrics(r.t, r.y);
  assert(m.settled); near(m.overshootPct, 0, 0.01, 'no overshoot'); near(m.riseTime, 0.03 * Math.log(9), 0.0025, 'rise 10-90 % = tau ln 9'); near(m.settlingTime, 0.03 * Math.log(20), 0.0025, '5 % settling = tau ln 20');
  for (const tau of [0.15, 0.25]) { r = first(tau, 0.5, 400); m = stepMetrics(r.t, r.y); assert.strictEqual(m.settled, false, 'tau ' + tau + ' does not settle in 0.5 s'); assert.strictEqual(m.overshootPct, null); assert(Number.isNaN(m.riseTime)); }
  const z = 0.5, wn = 2 * Math.PI * 4, wd = wn * Math.sqrt(1 - z * z), t = [], y = [];
  for (let i = 0; i < 400; i++) { const tt = i / 400; t.push(tt); y.push(1 - Math.exp(-z * wn * tt) * Math.sin(wd * tt + Math.acos(z)) / Math.sqrt(1 - z * z)); }
  m = stepMetrics(t, y); assert(m.settled); near(m.overshootPct, 100 * Math.exp(-z * Math.PI / Math.sqrt(1 - z * z)), 0.05, 'second-order overshoot');
  m = stepMetrics([0, 0.1, 0.2, 0.3], [0, 0, 0, 0]); assert.strictEqual(m.settled, false); assert.strictEqual(m.finalValue, 0);
});

test('a NaN never reaches a metric, a spectrum or a model (N1, check14)', () => {
  const g = noise(8), fs = 200, n = fs * 60;
  const target = Float64Array.from({ length: n }, (_, i) => 20 * Math.sin(2 * Math.PI * 0.5 * i / fs)), actual = Float64Array.from(target, v => v + g());
  const clean = diagnostics({ fs, target, actual, u: new Float64Array(n) }, {}, 'copter', 'roll');
  actual[5000] = NaN;
  const d = diagnostics({ fs, target, actual, u: new Float64Array(n) }, {}, 'copter', 'roll');
  near(d.rmsErrorDegs, clean.rmsErrorDegs, 0.01 * clean.rmsErrorDegs, 'RMS error'); assert.strictEqual(d.grade.letter, clean.grade.letter);
  assert(Number.isFinite(d.oscRmsDegs) && Number.isFinite(d.noiseRmsDegs));
  // full analysis on the independent simulator's data (check14 3b): one NaN must not change the proposal
  const DEG = 180 / Math.PI, plant = { K: 280, a: 2, tau: 0.025, Td: 0.003 }, gains = { P: 0.135, I: 0.135, D: 0.0036, FF: 0, FLTT: 20, FLTE: 0, FLTD: 20, IMAX: 0.5 };
  const s = simulate({ plant, gains, gyroHz: 40, fsLoop: 400, dur: 60, mode: 'angle', angleP: 4.5, inputTc: 0.15, pilot: { seed: 11, angAmp: [4, 12], rateAmp: [20, 60], hold: [0.3, 1.0], gap: [0.3, 1.2] }, gyroNoise: 1 / DEG, seed: 3 });
  const m = s.target.length, u = new Float64Array(m); for (let i = 0; i < m; i++) u[i] = s.P[i] + s.I[i] + s.D[i] + s.FF[i];
  const mk = () => ({ fs: 400, t0: 0, n: m, target: Float64Array.from(s.target, v => v * DEG), actual: Float64Array.from(s.actual, v => v * DEG), u: Float64Array.from(u), P: s.P, I: s.I, D: s.D, FF: s.FF, srate: new Float64Array(m), dmod: new Float64Array(m).fill(1) });
  const params = { SCHED_LOOP_RATE: 400, INS_GYRO_RATE: 0, INS_GYRO_FILTER: 40, ATC_RAT_RLL_P: 0.135, ATC_RAT_RLL_I: 0.135, ATC_RAT_RLL_D: 0.0036, ATC_RAT_RLL_FF: 0, ATC_RAT_RLL_FLTT: 20, ATC_RAT_RLL_FLTE: 0, ATC_RAT_RLL_FLTD: 20, ATC_RAT_RLL_SMAX: 50, ATC_RAT_RLL_IMAX: 0.5, ATC_RAT_RLL_D_FF: 0, ATC_ANG_RLL_P: 4.5 };
  const a0 = analyzeAxis({ family: 'copter', axis: 'roll', data: mk(), params });
  const bad = mk(); bad.actual[5000] = NaN; bad.u[7000] = Infinity;
  const a1 = analyzeAxis({ family: 'copter', axis: 'roll', data: bad, params });
  assert(a0.identification.ok && a1.identification.ok, 'identified in both');
  near(a1.proposal.gains.P, a0.proposal.gains.P, 0.01 * a0.proposal.gains.P, 'proposed P'); assert(Number.isFinite(a1.diagnostics.rmsErrorDegs));
  // gyro spectrum with one NaN sample still finds the 80 Hz tone
  const lg = gyroLog({ fs: 1000, seconds: 60, x: t => 5 * Math.sin(2 * Math.PI * 80 * t) + 0.05 * g() }); lg.msgs.GYR.GyrX[30000] = NaN;
  const r = analyzeNoise(lg, {}, 'copter', null);
  assert(r.peaks.some(p => Math.abs(p.f - 80) < 1.5), 'tone found');
});

console.log('refusals (F-79)');

test('not a DataFlash log: an explicit refusal shaped like a result; the CLI exits with 3', () => {
  const e = L.analyzeLog(Buffer.alloc(0));
  assert.strictEqual(e.error, 'The file is empty.'); assert(e.axes.roll.error && e.axes.pitch.error && e.axes.yaw.error); assert.strictEqual(e.noise.available, false); assert.strictEqual(typeof e.family, 'string');
  const tl = Buffer.alloc(4000); tl.writeBigUInt64BE(1760000000000000n, 0); tl[8] = 0xfe;
  assert.strictEqual(L.analyzeLog(tl).format.kind, 'tlog');
  const f = path.join(os.tmpdir(), 'aptuner-empty-' + process.pid + '.bin'); fs.writeFileSync(f, Buffer.alloc(0));
  const run = spawnSync(process.execPath, [path.join(__dirname, '..', 'tools', 'analyze-log.js'), f], { encoding: 'utf8' });
  fs.unlinkSync(f);
  assert.strictEqual(run.status, 3, run.stdout + run.stderr); assert(/NOT ANALYSED \(empty\)/.test(run.stdout));
});

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
