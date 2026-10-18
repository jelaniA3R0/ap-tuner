// Compares the C++ log analysis with the JavaScript reference on real logs:
//   node tools/compare-cpp.js <log.bin> [...]        (needs cpp/build/release/cli/aptuner-analyze.exe)
// Prints every difference beyond tolerance (numbers: relative 1e-6 or absolute 1e-9; strings / booleans exact).
'use strict';
const path = require('path');
const fs = require('fs');
const os = require('os');
const { execFileSync } = require('child_process');
const { analyzeLog } = require('../src/engine/loganalysis');

const exe = path.join(__dirname, '..', 'cpp', 'build', 'release', 'cli', process.platform === 'win32' ? 'aptuner-analyze.exe' : 'aptuner-analyze');
const REL = parseFloat(process.env.REL || '1e-6');
let total = 0;

function cmp(a, b, p, out) {
  if (a === undefined && b === undefined) return;
  if (typeof a === 'number' || typeof b === 'number') {
    const na = typeof a === 'number' && isFinite(a) ? a : null, nb = typeof b === 'number' && isFinite(b) ? b : null;
    if (na === null && nb === null) return;
    if (na === null || nb === null) { out.push(`${p}: js ${a} cpp ${b}`); return; }
    const err = Math.abs(na - nb), scale = Math.max(Math.abs(na), Math.abs(nb));
    if (err > 1e-9 && err > REL * scale) out.push(`${p}: js ${na} cpp ${nb} (rel ${(err / scale).toExponential(2)})`);
    return;
  }
  if (Array.isArray(a) || Array.isArray(b)) {
    if (!Array.isArray(a) || !Array.isArray(b)) { out.push(`${p}: js ${JSON.stringify(a)?.slice(0, 80)} cpp ${JSON.stringify(b)?.slice(0, 80)}`); return; }
    if (a.length !== b.length) { out.push(`${p}: length js ${a.length} cpp ${b.length}`); return; }
    const before = out.length; const sub = [];
    for (let i = 0; i < a.length; i++) cmp(a[i], b[i], `${p}[${i}]`, sub);
    if (sub.length > 3) out.push(...sub.slice(0, 3), `${p}: ... ${sub.length} differing elements`); else out.push(...sub);
    return;
  }
  if (a && typeof a === 'object' && b && typeof b === 'object') {
    const keys = new Set([...Object.keys(a), ...Object.keys(b)]);
    for (const k of keys) {
      if (a[k] === undefined && (b[k] === null)) continue;            // JS left the key out, C++ wrote null
      if (b[k] === undefined && (a[k] === null)) continue;
      cmp(a[k], b[k], p + '.' + k, out);
    }
    return;
  }
  if ((a === null || a === undefined) && (b === null || b === undefined)) return;
  if (a !== b) out.push(`${p}: js ${JSON.stringify(a)} cpp ${JSON.stringify(b)}`);
}

for (const file of process.argv.slice(2)) {
  const tmp = path.join(os.tmpdir(), 'aptuner-cpp-' + process.pid + '.json');
  const t0 = Date.now();
  const js = JSON.parse(JSON.stringify(analyzeLog(file, {})));
  const tJs = Date.now() - t0;
  const t1 = Date.now();
  // exit code 3: the file was refused (not a DataFlash log); the result is still written and compared
  try { execFileSync(exe, [file, '--json', tmp], { stdio: ['ignore', 'ignore', 'inherit'] }); } catch (e) { if (e.status !== 3) throw e; }
  const tCpp = Date.now() - t1;
  const cpp = JSON.parse(fs.readFileSync(tmp, 'utf8')); fs.unlinkSync(tmp);
  delete js.counts; delete cpp.counts;   // key order only (same values)
  const out = []; cmp(js, cpp, '', out);
  total += out.length;
  console.log(`\n## ${path.basename(file)}: ${out.length ? out.length + ' difference(s)' : 'identical'}  (JS ${tJs} ms, C++ ${tCpp} ms incl. start-up)`);
  for (const o of out.slice(0, 60)) console.log('   ' + o);
}
process.exit(total ? 1 : 0);
