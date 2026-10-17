'use strict';
const fs = require('fs');
const path = require('path');
const f = path.join(__dirname, '..', 'src/model/rules.js');
let r = fs.readFileSync(f, 'utf8');
const old = "  conservative: { gmMinDb: 6, pmMinDeg: 30, msMax: 3.0, osMaxPct: 15, backoff: 0.3, label: 'Conservative (large / flexible / payload aircraft)' },\n  normal: { gmMinDb: 4, pmMinDeg: 25, msMax: 3.5, osMaxPct: 25, backoff: 0.4, label: 'Normal (QuikTune rule: 40% of the oscillation gain)' },\n  aggressive: { gmMinDb: 3, pmMinDeg: 20, msMax: 4.5, osMaxPct: 35, backoff: 0.5, label: 'Aggressive (small agile aircraft: 50% of the oscillation gain)' },";
const nw = "  // Oscillation sweeps on the simulator showed the model's oscillation gain is 15-40% optimistic (it\n  // under-weights the destabilising effect of D), so the kept fractions are set below QuikTune's 0.4 to\n  // land at QuikTune's intended margin on the real vehicle: 0.3 of the model limit ~ 0.35-0.4 of the true one.\n  conservative: { gmMinDb: 6, pmMinDeg: 30, msMax: 3.0, osMaxPct: 12, backoff: 0.22, label: 'Conservative (large / flexible / payload aircraft)' },\n  normal: { gmMinDb: 4, pmMinDeg: 25, msMax: 3.5, osMaxPct: 20, backoff: 0.3, label: 'Normal (QuikTune margin)' },\n  aggressive: { gmMinDb: 3, pmMinDeg: 20, msMax: 4.5, osMaxPct: 30, backoff: 0.4, label: 'Aggressive (small agile aircraft)' },";
if (!r.includes(old)) { console.error('preset pattern not found'); process.exit(1); }
r = r.replace(old, nw);
fs.writeFileSync(f, r);
console.log('presets v2 patched');
