// Builds the before/after comparison report (one self-contained HTML file; the chart library is inlined).
// Usage: node make-report.js <demo dir> (expects before.json, after.json, meta.json in it)
'use strict';
const fs = require('fs');
const path = require('path');
const dir = process.argv[2];
const before = JSON.parse(fs.readFileSync(path.join(dir, 'before.json'), 'utf8'));
const after = JSON.parse(fs.readFileSync(path.join(dir, 'after.json'), 'utf8'));
const meta = JSON.parse(fs.readFileSync(path.join(dir, 'meta.json'), 'utf8'));
const vendor = (f) => fs.readFileSync(path.join(__dirname, '..', 'web', 'vendor', f), 'utf8');
const uplotJs = vendor('uPlot.iife.min.js'), uplotCss = vendor('uPlot.min.css');
const f1 = v => (v === undefined || v === null || isNaN(v)) ? '-' : (+v).toFixed(1);
const f0 = v => (v === undefined || v === null || isNaN(v)) ? '-' : Math.round(v);
const f4 = v => (v === undefined || v === null || isNaN(v)) ? '-' : +(+v).toPrecision(4);
function better(a, b, lowerIsBetter) { if (a === undefined || b === undefined || a === null || b === null) return ''; const d = lowerIsBetter ? a - b : b - a; return d > 0.05 * Math.abs(a || 1) ? 'better' : d < -0.05 * Math.abs(a || 1) ? 'worse' : 'same'; }
const rows = [];
for (const ax of ['roll', 'pitch']) {
  const b = before.axes[ax], a = after.axes[ax]; if (!b || !a) continue;
  rows.push({ h: ax + ' axis' });
  rows.push({ n: 'rate tracking (100% = perfect)', b: f0(b.trackingPct) + '%', a: f0(a.trackingPct) + '%', c: better(b.trackingPct, a.trackingPct, false) });
  rows.push({ n: 'rate error RMS (deg/s)', b: f1(b.rmsErrorDegs), a: f1(a.rmsErrorDegs), c: better(b.rmsErrorDegs, a.rmsErrorDegs, true) });
  rows.push({ n: 'response lag (ms)', b: f0(b.lagMs), a: f0(a.lagMs), c: better(b.lagMs, a.lagMs, true) });
  rows.push({ n: 'oscillation band 2-30 Hz (deg/s RMS)', b: f1(b.oscBandDegs), a: f1(a.oscBandDegs), c: better(b.oscBandDegs, a.oscBandDegs, true) });
  if (b.step && a.step) { rows.push({ n: 'rate step: rise 10-90% (ms)', b: f0(b.step.riseMs), a: f0(a.step.riseMs), c: better(b.step.riseMs, a.step.riseMs, true) }); rows.push({ n: 'rate step: overshoot (%)', b: f0(b.step.overshootPct), a: f0(a.step.overshootPct), c: better(b.step.overshootPct, a.step.overshootPct, true) }); rows.push({ n: 'rate step: settling (ms)', b: f0(b.step.settleMs), a: f0(a.step.settleMs), c: better(b.step.settleMs, a.step.settleMs, true) }); }
  if (b.angle && a.angle) { rows.push({ n: 'angle error RMS (deg)', b: f1(b.angle.rmsErrorDeg), a: f1(a.angle.rmsErrorDeg), c: better(b.angle.rmsErrorDeg, a.angle.rmsErrorDeg, true) }); if (b.angle.step && a.angle.step) { rows.push({ n: 'angle step: rise (ms)', b: f0(b.angle.step.riseMs), a: f0(a.angle.step.riseMs), c: better(b.angle.step.riseMs, a.angle.step.riseMs, true) }); rows.push({ n: 'angle step: overshoot (%)', b: f0(b.angle.step.overshootPct), a: f0(a.angle.step.overshootPct), c: better(b.angle.step.overshootPct, a.angle.step.overshootPct, true) }); } }
}
const gainRows = [];
for (const ax of ['roll', 'pitch']) for (const k of ['P', 'I', 'D', 'FLTT', 'FLTD', 'angleP']) gainRows.push({ n: `${ax} ${k}`, b: f4(before.gains[ax][k]), a: f4(after.gainsEnd[ax][k]) });
const notchRows = (meta.notch || []).map(c => ({ n: c.name, b: f4(c.from), a: f4(c.to) }));
const data = { before: { roll: before.series.roll, pitch: before.series.pitch, stepRoll: before.axes.roll?.step, stepPitch: before.axes.pitch?.step, angRoll: before.axes.roll?.angle?.step, angPitch: before.axes.pitch?.angle?.step }, after: { roll: after.series.roll, pitch: after.series.pitch, stepRoll: after.axes.roll?.step, stepPitch: after.axes.pitch?.step, angRoll: after.axes.roll?.angle?.step, angPitch: after.axes.pitch?.angle?.step } };
const html = `<!doctype html><html><head><meta charset="utf-8"><title>${meta.title} - before vs after</title><style>${uplotCss}</style>
<style>body{margin:0;font:14px/1.45 "Segoe UI",system-ui,sans-serif;background:#0b1120;color:#e6edf7;padding:20px}h1{font-size:22px;margin:0 0 6px}h2{font-size:16px;color:#38bdf8;margin:22px 0 8px}.muted{color:#93a0b4}.card{background:#121a2b;border:1px solid #223049;border-radius:12px;padding:14px 16px;margin:12px 0}table{border-collapse:collapse;width:100%;font-size:13px}th{text-align:right;color:#93a0b4;padding:4px 8px;border-bottom:1px solid #2d3d5a}td{text-align:right;padding:5px 8px;border-bottom:1px solid #223049;font-variant-numeric:tabular-nums}th:first-child,td:first-child{text-align:left}tr.h td{color:#38bdf8;font-weight:600;padding-top:12px}.better{color:#22c55e}.worse{color:#f59e0b}.same{color:#93a0b4}.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}.chart{background:#0f172a;border:1px solid #223049;border-radius:10px;padding:8px}.u-legend{color:#93a0b4;font-size:11px}a{color:#38bdf8}ul{padding-left:20px}</style></head><body>
<h1>${meta.title}: before vs after AP Tuner</h1>
<div class="muted">${meta.subtitle}</div>
<div class="card"><h2 style="margin-top:0">What was done</h2><ol>${meta.steps.map(s => `<li>${s}</li>`).join('')}</ol>
<p class="muted">Files in this folder: <code>before.BIN</code> and <code>after.BIN</code> (the autopilot logs of the two identical sequence flights), <code>tuning.BIN</code> (the tuning flight), <code>${meta.video}</code> (side-by-side video), <code>before-analysis.json</code> / <code>after-analysis.json</code> (AP Tuner's log reports), <code>tuned.param</code> (the gains written).</p></div>
<div class="grid"><div class="card"><h2 style="margin-top:0">Gains</h2><table><tr><th>parameter</th><th>before</th><th>after</th></tr>${gainRows.map(r => `<tr><td>${r.n}</td><td>${r.b}</td><td>${r.a}</td></tr>`).join('')}</table>${notchRows.length ? `<h2>Notch filter (written in step 4)</h2><table><tr><th>parameter</th><th>before</th><th>after</th></tr>${notchRows.map(r => `<tr><td>${r.n}</td><td>${r.b}</td><td>${r.a}</td></tr>`).join('')}</table>` : ''}</div>
<div class="card"><h2 style="margin-top:0">Measured on the same stick sequence</h2><table><tr><th>metric</th><th>before</th><th>after</th><th>change</th></tr>${rows.map(r => r.h ? `<tr class="h"><td colspan="4">${r.h}</td></tr>` : `<tr><td>${r.n}</td><td>${r.b}</td><td>${r.a}</td><td class="${r.c}">${r.c}</td></tr>`).join('')}</table><p class="muted" style="font-size:12px">Rate metrics compare the demanded and the actual body rate from the autopilot's own rate controller logs; angle metrics compare the demanded and actual attitude. "Lag" is the delay that best aligns the actual rate with the demanded rate. Step responses are estimated from the whole sequence with the same method as ArduPilot's PID Review tool.</p></div></div>
<h2>Roll rate, demanded vs actual (first 30 s of the sequence)</h2><div class="grid"><div class="chart" id="r1"></div><div class="chart" id="r2"></div></div>
<h2>Pitch rate, demanded vs actual</h2><div class="grid"><div class="chart" id="p1"></div><div class="chart" id="p2"></div></div>
<h2>Step responses (1 = demanded)</h2><div class="grid"><div class="chart" id="s1"></div><div class="chart" id="s2"></div><div class="chart" id="s3"></div><div class="chart" id="s4"></div></div>
<script>${uplotJs}</script>
<script>
const D = ${JSON.stringify(data)};
const C = { b: '#f97316', a: '#38bdf8', t: '#facc15' };
function mk(el, title, series, data, xl, h) { const w = Math.max(300, document.getElementById(el).clientWidth - 20); new uPlot({ title, width: w, height: h || 220, scales: { x: { time: false } }, axes: [{ stroke: '#93a0b4', grid: { stroke: '#223049' }, label: xl }, { stroke: '#93a0b4', grid: { stroke: '#223049' } }], series: [{}, ...series] }, data, document.getElementById(el)); }
function cut(s, tmax) { const n = s.t.findIndex(v => v > tmax); return n < 0 ? s : { t: s.t.slice(0, n), target: s.target.slice(0, n), actual: s.actual.slice(0, n) }; }
const rb = cut(D.before.roll, 30), ra = cut(D.after.roll, 30), pb = cut(D.before.pitch, 30), pa = cut(D.after.pitch, 30);
mk('r1', 'BEFORE', [{ label: 'demanded', stroke: C.t, dash: [4, 4] }, { label: 'actual', stroke: C.b, width: 1.5 }], [rb.t, rb.target, rb.actual], 's');
mk('r2', 'AFTER', [{ label: 'demanded', stroke: C.t, dash: [4, 4] }, { label: 'actual', stroke: C.a, width: 1.5 }], [ra.t, ra.target, ra.actual], 's');
mk('p1', 'BEFORE', [{ label: 'demanded', stroke: C.t, dash: [4, 4] }, { label: 'actual', stroke: C.b, width: 1.5 }], [pb.t, pb.target, pb.actual], 's');
mk('p2', 'AFTER', [{ label: 'demanded', stroke: C.t, dash: [4, 4] }, { label: 'actual', stroke: C.a, width: 1.5 }], [pa.t, pa.target, pa.actual], 's');
function stepChart(el, title, b, a) { if (!b || !a) return; const t = b.t; const ai = t.map((x, i) => a.y[i] ?? null); mk(el, title, [{ label: 'before', stroke: C.b, width: 1.5 }, { label: 'after', stroke: C.a, width: 1.5 }], [t, b.y, ai], 's'); }
stepChart('s1', 'roll rate step', D.before.stepRoll, D.after.stepRoll); stepChart('s2', 'pitch rate step', D.before.stepPitch, D.after.stepPitch);
stepChart('s3', 'roll angle step', D.before.angRoll, D.after.angRoll); stepChart('s4', 'pitch angle step', D.before.angPitch, D.after.angPitch);
</script></body></html>`;
fs.writeFileSync(path.join(dir, 'report.html'), html);
console.log('wrote', path.join(dir, 'report.html'));
