// Writes demo/index.html linking the vehicle demonstrations (reports, videos, logs).
'use strict';
const fs = require('fs');
const path = require('path');
const DEMO = path.join(__dirname, '..', 'demo');
const cards = [];
for (const v of fs.readdirSync(DEMO)) {
  const d = path.join(DEMO, v); if (!fs.statSync(d).isDirectory()) continue;
  const metaF = path.join(d, 'meta.json'); if (!fs.existsSync(metaF)) continue;
  const m = JSON.parse(fs.readFileSync(metaF, 'utf8'));
  const files = fs.readdirSync(d).filter(f => /\.(BIN|avi|param|json|html)$/i.test(f) && f !== 'index.html');
  const g = (w) => m[w + 'Grades'] ? Object.entries(m[w + 'Grades']).map(([a, x]) => `${a} ${x ? x.letter + ' (' + x.score + ')' : '-'}`).join(', ') : '';
  cards.push(`<div class="card"><h2>${m.title}</h2><p class="muted">${m.subtitle || ''}</p>
    <p><a class="btn" href="${v}/report.html">Open the comparison report</a> <a class="btn" href="${v}/${m.video}">Play the side-by-side video (.avi)</a></p>
    <p>AP Tuner grades on the test sequence: <b>before</b> ${g('before')} → <b>after</b> ${g('after')}</p>
    <ol>${m.steps.map(s => `<li>${s}</li>`).join('')}</ol>
    <p class="muted">Files: ${files.map(f => `<a href="${v}/${f}">${f}</a>`).join(' · ')}</p></div>`);
}
const html = `<!doctype html><html><head><meta charset="utf-8"><title>AP Tuner demonstrations</title><style>body{margin:0;padding:20px;font:14px/1.5 "Segoe UI",system-ui,sans-serif;background:#0b1120;color:#e6edf7}h1{font-size:24px}h2{font-size:18px;margin:0 0 4px}.muted{color:#93a0b4}.card{background:#121a2b;border:1px solid #223049;border-radius:12px;padding:16px 18px;margin:14px 0}a{color:#38bdf8}.btn{display:inline-block;padding:8px 14px;border:1px solid #2d3d5a;border-radius:10px;background:#182236;color:#e6edf7;text-decoration:none;margin-right:8px}ol{padding-left:20px}</style></head><body>
<h1>AP Tuner demonstrations on ArduPilot simulation models</h1>
<p class="muted">Each vehicle was flown through the same scripted stick sequence twice: with its starting gains, and after AP Tuner tuned it in a separate flight. The logs are the autopilot's own .BIN files; the videos replay both flights side by side from those logs.</p>
${cards.join('\n')}
</body></html>`;
fs.writeFileSync(path.join(DEMO, 'index.html'), html);
console.log('wrote', path.join(DEMO, 'index.html'), cards.length, 'vehicles');
