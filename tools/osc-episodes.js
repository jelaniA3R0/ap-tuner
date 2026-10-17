// Sustained oscillation episodes in a dataflash log (SITL or real): per axis, the stretches where more than half of
// a 2 s window has the slew rate above the limit or Dmod below 0.9 (the criterion of the app's oscillation monitor),
// with the tuning-parameter changes logged around them. Used to check that no oscillation went undetected in a
// simulator run: each episode should end within a few seconds (the app reverts), none should last longer than --max.
// The PID log messages are thinned to 50 Hz first, the rate the app sees over telemetry.
// Usage: node tools/osc-episodes.js <log.BIN> [--max 3] [--smax 5]   (exit code 1 if an episode lasts longer than --max)
'use strict';
const path = require('path');
const { parseBin } = require(path.join(__dirname, '..', 'src', 'log', 'dfreader'));
const { parmRecords } = require('./parm-records');

const argv = process.argv.slice(2); const opt = (k, d) => { const i = argv.indexOf('--' + k); return i >= 0 ? parseFloat(argv[i + 1]) : d; };
const file = argv.find(a => !a.startsWith('--') && /\.bin$/i.test(a));
if (!file) { console.log('usage: node tools/osc-episodes.js <log.BIN> [--max 3] [--smax 5]'); process.exit(2); }
const MAX = opt('max', 3), SMAX = opt('smax', 5), WIN = 2, FRAC = 0.5;

function episodes(m) {
  const t = Array.from(m.TimeUS, v => v / 1e6);
  const idx = []; let next = -1e9;
  for (let i = 0; i < t.length; i++) if (t[i] >= next) { idx.push(i); next = t[i] + 0.02; }
  const bad = idx.map(i => m.SRate[i] > SMAX || (m.Dmod[i] > 0 && m.Dmod[i] < 0.9));
  const out = []; let cur = null; let k0 = 0, nBad = 0;
  for (let k = 0; k < idx.length; k++) {
    const tk = t[idx[k]];
    if (bad[k]) nBad++;
    while (t[idx[k0]] < tk - WIN) { if (bad[k0]) nBad--; k0++; }
    const n = k - k0 + 1;
    let first = null, last = null;
    if (nBad) for (let q = k0; q <= k; q++) if (bad[q]) { if (first === null) first = t[idx[q]]; last = t[idx[q]]; }
    const sustained = n >= 10 && nBad / n >= FRAC && last - first >= 0.4 * WIN;
    if (sustained) {
      if (!cur) cur = { start: first, end: last, srMax: 0, dmMin: 1 };
      cur.end = last;
    } else if (cur) { out.push(cur); cur = null; }
    if (cur) { const i = idx[k]; cur.srMax = Math.max(cur.srMax, m.SRate[i]); if (m.Dmod[i] > 0) cur.dmMin = Math.min(cur.dmMin, m.Dmod[i]); }
  }
  if (cur) out.push(cur);
  // an episode runs from its first to its last bad sample
  for (const e of out) {
    for (let k = 0; k < idx.length; k++) { const tk = t[idx[k]]; if (tk > e.start - WIN && tk <= e.start && bad[k]) { e.start = tk; break; } }
    e.duration = e.end - e.start;
  }
  return out;
}

const log = parseBin(file, { want: ['PIDR', 'PIDP', 'PIDY'] });
// every tuning-parameter write (reverts to the boot value included), each once (ArduPilot may log a set twice)
const changes = parmRecords(file).filter(c => /^(ATC_|INS_HNTCH_|INS_GYRO_FILTER|Q_A_)/.test(c.name))
  .filter((c, i, a) => !a.slice(Math.max(0, i - 12), i).some(d => d.name === c.name && d.val === c.val && c.t - d.t < 0.1));
let worst = 0, total = 0;
console.log(`${path.basename(file)}: sustained oscillation episodes (> ${FRAC * 100} % of ${WIN} s with SRate > ${SMAX} or Dmod < 0.9)`);
for (const [msg, axis] of [['PIDR', 'roll'], ['PIDP', 'pitch'], ['PIDY', 'yaw']]) {
  const m = log.msgs[msg];
  if (!m || !m.SRate) { console.log(`  ${axis}: no ${msg} with SRate in the log`); continue; }
  const eps = episodes(m);
  if (!eps.length) { console.log(`  ${axis}: none`); continue; }
  for (const e of eps) {
    total++; worst = Math.max(worst, e.duration);
    console.log(`  ${axis}: ${e.start.toFixed(2)}-${e.end.toFixed(2)} s, ${e.duration.toFixed(2)} s, slew rate up to ${e.srMax.toFixed(1)}, Dmod down to ${e.dmMin.toFixed(2)}`);
    for (const c of changes.filter(c => c.t >= e.start - 10 && c.t <= e.end + 5)) console.log(`      ${c.t.toFixed(2)} s  ${c.name} = ${+(+c.val).toPrecision(5)}`);
  }
}
console.log(`episodes: ${total}, longest ${worst.toFixed(2)} s (limit ${MAX} s): ${worst <= MAX ? 'OK' : 'TOO LONG'}`);
process.exit(worst <= MAX ? 0 : 1);
