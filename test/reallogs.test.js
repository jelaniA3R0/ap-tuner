// The owner's tandem QuadPlane hover logs (ArduPlane 4.7.1, 2026-10-15) and the audit's refusal cases
// (demo/audit-2026-10-17/run-tests.sh section 6) made from them. Skipped when the logs are not on this PC.
// Expected values are computed here from the raw records, independently of the analysis pipeline.
//   node test/reallogs.test.js [folder with the .bin logs]
'use strict';
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const { parseBin } = require('../src/log/dfreader');
const { cleanLogTimes } = require('../src/log/integrity');
const L = require('../src/engine/loganalysis');

const DIR = process.argv[2] || path.join(process.env.USERPROFILE || process.env.HOME || '', 'OneDrive', 'Documents', 'Mission Planner', 'logs', 'FIXED_WING', '1');
const LOG = name => path.join(DIR, `2026-10-15 ${name}.bin`);
let passed = 0, failed = 0, skipped = 0;
function test(name, files, fn) {
  if (!files.every(f => fs.existsSync(f))) { skipped++; console.log('  skip ' + name + ' (log not found)'); return; }
  try { fn(); passed++; console.log('  ok   ' + name); } catch (e) { failed++; console.log('  FAIL ' + name + '\n       ' + (e && e.stack ? e.stack.split('\n').slice(0, 3).join('\n       ') : e)); }
}
const near = (a, b, tol, what) => assert(Math.abs(a - b) <= tol, `${what}: got ${a}, expected ${b} (+-${tol})`);

/** independent airborne stretches from QTUN.ThO: above 60% of the median hover throttle in the window (>= 0.15), 3 s after lift-off and 2 s before touch-down left out, a sample held for 0.5 s at most */
function airborne(q, tA, tB) {
  const t = Array.from(q.TimeUS, v => v / 1e6), v = q.ThO;
  const hov = []; for (let i = 0; i < t.length; i++) if (t[i] >= tA && t[i] <= tB && v[i] > 0.15) hov.push(v[i]);
  hov.sort((a, b) => a - b); const lim = Math.max(0.15, 0.6 * hov[Math.floor(hov.length / 2)]);
  const d = []; for (let i = 1; i < t.length; i++) d.push(t[i] - t[i - 1]); d.sort((a, b) => a - b); const hold = Math.max(0.5, 3 * d[Math.floor(d.length / 2)]);
  const out = []; let i = 0, afterGap = false;
  while (i < t.length) {
    if (!(v[i] > lim)) { i++; afterGap = false; continue; }
    let j = i; while (j + 1 < t.length && v[j + 1] > lim && t[j + 1] - t[j] <= hold) j++;
    const atGap = j + 1 < t.length && v[j + 1] > lim;
    const a = t[i] + (afterGap ? 0 : 3), b = (j + 1 < t.length ? Math.min(t[j + 1], t[j] + hold) : t[j] + hold) - (atGap ? 0 : 2);
    if (b > a) out.push([a, b]);
    afterGap = atGap; i = j + 1;
  }
  return out;
}

console.log('owner logs in ' + DIR);

test('04-20-29: the 51.8 s logging gap inside the window is reported and left out', [LOG('04-20-29')], () => {
  const r = L.analyzeLog(LOG('04-20-29'));
  const raw = parseBin(LOG('04-20-29'), { want: ['RATE', 'QTUN'] });
  const tw = r.flightWindow;
  near(tw.tStart, 157.5, 0.1, 'window start'); near(tw.tEnd, 320.2, 0.1, 'window end');
  // the largest step between RATE records inside the window, from the raw records
  const t = Array.from(raw.msgs.RATE.TimeUS, v => v / 1e6).filter(v => v >= tw.tStart && v <= tw.tEnd);
  let g0 = 0, g1 = 0; for (let i = 1; i < t.length; i++) if (t[i] - t[i - 1] > g1 - g0) { g0 = t[i - 1]; g1 = t[i]; }
  near(g1 - g0, 51.775, 0.001, 'raw gap length');
  for (const axis of ['roll', 'pitch', 'yaw']) {
    const a = r.axes[axis], di = a.dataInfo;
    assert.strictEqual(di.gapCount, 1, axis + ' one gap'); near(di.gaps[0].t0, g0, 1e-9, axis + ' gap start'); near(di.gaps[0].t1, g1, 1e-9, axis + ' gap end');
    assert(a.warnings.some(w => /Logging stopped for 51\.8 s \(230\.5-282\.3 s\)/.test(w)), axis + ' gap in the warnings');
    assert(a.series.t.every(v => v <= g0 || v >= g1), axis + ' nothing shown inside the gap');
    near(di.fs, 300.03, 0.01, axis + ' native 300 Hz (the old code used a 200 Hz grid)');
  }
  // roll RMS error over the airborne RATE records, computed from the raw records
  const iv = airborne(raw.msgs.QTUN, tw.tStart, tw.tEnd); const R = raw.msgs.RATE;
  let e2 = 0, c = 0; for (let i = 0; i < R.n; i++) { const ts = R.TimeUS[i] / 1e6; if (ts < tw.tStart || ts > tw.tEnd || !iv.some(([a, b]) => ts >= a && ts < b)) continue; const e = R.RDes[i] - R.R[i]; e2 += e * e; c++; }
  near(r.axes.roll.diagnostics.rmsErrorDegs, Math.sqrt(e2 / c), 1e-9, 'airborne roll RMS error');
  near(r.axes.roll.dataInfo.usableSeconds, c / r.axes.roll.dataInfo.fs, 1e-9, 'airborne seconds');
  assert(r.axes.roll.dataInfo.usableSeconds < 0.3 * (tw.tEnd - tw.tStart), 'only 27% of the window flown');
  assert.strictEqual(r.paramChanges, 0, 'no tuning parameter changed (the 65 changes are STAT_* and baro values)');
});

test('04-33-02: a 6 s dropout cut out in memory is reported and gives the same proposal', [LOG('04-33-02')], () => {
  const buf = fs.readFileSync(LOG('04-33-02'));
  const fmts = parseBin(buf, { want: [] }).fmts; const parts = []; let i = 0, from = 0;
  while (i + 3 <= buf.length) {
    const fm = buf[i] === 0xa3 && buf[i + 1] === 0x95 ? fmts[buf[i + 2]] : null; if (!fm) { i++; continue; }
    const t = fm.labels[0] === 'TimeUS' && fm.format[0] === 'Q' ? Number(buf.readBigUInt64LE(i + 3)) / 1e6 : null;
    if (t !== null && t >= 160 && t <= 166) { parts.push(buf.subarray(from, i)); from = i + fm.len; }
    i += fm.len;
  }
  parts.push(buf.subarray(from));
  const a = L.analyzeLog(buf), b = L.analyzeLog(Buffer.concat(parts));
  assert.strictEqual(a.axes.pitch.dataInfo.gapCount, 0);
  assert.strictEqual(b.axes.pitch.dataInfo.gapCount, 1); near(b.axes.pitch.dataInfo.gaps[0].s, 6, 0.01, 'gap length');
  assert(b.axes.pitch.warnings.some(w => /Logging stopped for 6\.0 s/.test(w)));
  near(a.axes.pitch.dataInfo.usableSeconds - b.axes.pitch.dataInfo.usableSeconds, 6, 0.2, 'only the 6 s are lost');
  assert.deepStrictEqual(b.axes.pitch.proposal.changes.map(c => [c.key, +c.to.toPrecision(4)]), a.axes.pitch.proposal.changes.map(c => [c.key, +c.to.toPrecision(4)]), 'same proposal (audit before the fix: D 0.0054 -> 0.00492)');
});

test('04-43-00 with 2000 random byte errors: damage counted, time stamps cleaned, no NaN', [LOG('04-43-00')], () => {
  const buf = Buffer.from(fs.readFileSync(LOG('04-43-00'))); let s = 5; const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
  for (let k = 0; k < 2000; k++) { const p = 200000 + Math.floor(rnd() * (buf.length - 200000)); buf[p] = Math.floor(rnd() * 256); }
  const r = L.analyzeLog(buf);
  assert(r.integrity.skippedBytes > 0 && r.integrity.resyncs > 0, 'damage counted');
  assert(r.integrity.notes.some(n => /could not be read/.test(n)), 'damage reported');
  const log = parseBin(buf, { want: ['RATE'] }); cleanLogTimes(log, ['RATE']);
  const t = log.msgs.RATE.TimeUS; for (let i = 1; i < t.length; i++) assert(t[i] > t[i - 1], 'RATE stamps increase after cleaning');
  for (const axis of ['roll', 'pitch', 'yaw']) assert(Number.isFinite(r.axes[axis].diagnostics.rmsErrorDegs), axis + ' finite');
});

const T3 = LOG('04-43-00');
const tmp = name => path.join(require('os').tmpdir(), `aptuner-${process.pid}-${name}.bin`);
test('refusal cases: zeroed 4 MB, truncated, no flight, telemetry log, empty file', [T3], () => {
  const src = fs.readFileSync(T3);
  const z = Buffer.from(src); z.fill(0, 4 * 1048576, 8 * 1048576);
  const rz = L.analyzeLog(z);
  assert(rz.integrity.skippedBytes >= 4 * 1048576 - 200, 'zeroed bytes counted: ' + rz.integrity.skippedBytes);
  assert(rz.integrity.notes.some(n => /4\.0 MB of the file could not be read/.test(n)));
  // the blanked bytes are a hole in time; it must be a reported gap or lie outside the analysed window
  const t = parseBin(z, { want: ['RATE'] }).msgs.RATE.TimeUS; let h0 = 0, h1 = 0; for (let i = 1; i < t.length; i++) if (t[i] - t[i - 1] > (h1 - h0) * 1e6) { h0 = t[i - 1] / 1e6; h1 = t[i] / 1e6; }
  assert(h1 - h0 > 10, 'the blanked bytes remove time');
  const w = rz.flightWindow, overlaps = w.tStart < h1 && w.tEnd > h0;
  assert(Object.values(rz.axes).every(a => a.error || (overlaps ? a.dataInfo.gaps.some(g => Math.abs(g.t0 - h0) < 1e-6) : a.dataInfo.gapCount === 0)), 'the hole is a gap or outside the window, never filled in');
  const rt = L.analyzeLog(src.subarray(0, 6700001));
  assert(rt.integrity.truncatedBytes > 0 && rt.integrity.notes.some(n => /ends in the middle of a record/.test(n)), 'truncation reported');
  const rn = L.analyzeLog(src.subarray(0, 65536));
  assert(Object.values(rn.axes).every(a => a.error), 'no flight: no axis analysed');
  const tl = Buffer.alloc(200000); for (let k = 0; k < 2000; k++) { tl.writeBigUInt64BE(1760000000000000n + BigInt(k) * 1000n, k * 100); tl[k * 100 + 8] = 0xfd; }
  assert.strictEqual(L.analyzeLog(tl).format.kind, 'tlog');
  assert.strictEqual(L.analyzeLog(Buffer.alloc(0)).error, 'The file is empty.');
  void tmp;
});

console.log(`\n${passed} passed, ${failed} failed, ${skipped} skipped`);
process.exit(failed ? 1 : 0);
