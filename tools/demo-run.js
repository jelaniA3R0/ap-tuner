// Full before/after demonstration on a simulated vehicle, driven through the running AP Tuner app.
// Usage: node tools/demo-run.js quad|quadplane [--tune-minutes 10]
// Flight 1: fixed stick sequence with the starting gains  -> before.BIN
// Flight 2: random taps while AP Tuner tunes roll+pitch (auto-apply) -> tuning.BIN, tuned.param
// Flight 3: the same fixed sequence with the new gains -> after.BIN
// Then: AP Tuner log reports, comparison report (report.html) and the side-by-side video (.avi).
'use strict';
const fs = require('fs');
const path = require('path');
const http = require('http');
const { spawn, execFileSync } = require('child_process');
const { extract } = require('./flight-data');

const vehicle = process.argv[2] || 'quad';
const argv = process.argv.slice(3); const opt = (k, d) => { const i = argv.indexOf('--' + k); return i >= 0 ? argv[i + 1] : d; };
const tuneMinutes = parseFloat(opt('tune-minutes', '10'));
const postOnly = argv.includes('--post-only');
const retune = argv.includes('--retune');   // keep before.BIN, start again from the starting gains, redo tuning + after + post
const ROOT = path.join(__dirname, '..');
const SITL = path.join(ROOT, 'sitl');
const RUN = path.join(SITL, 'run-' + vehicle);
const DEMO = path.join(ROOT, 'demo', vehicle);
const cfg = vehicle === 'quadplane'
  ? { exe: path.join(SITL, 'ArduPlane.exe'), model: 'quadplane', defaults: path.join(SITL, 'demo-quadplane.parm'), family: 'vtol', title: 'Medium QuadPlane VTOL (ArduPilot standard "quadplane" model)', kind: 'vtol' }
  : { exe: path.join(SITL, 'ArduCopter.exe'), model: 'quad', defaults: path.join(SITL, 'demo-quad.parm'), family: 'copter', title: 'Medium quadcopter (ArduPilot standard "quad" model, X frame)', kind: 'quad' };
const sleep = ms => new Promise(r => setTimeout(r, ms));
const log = (...a) => console.log(new Date().toISOString().slice(11, 19), ...a);
const api = (p, body) => new Promise((res, rej) => { const data = body !== undefined ? JSON.stringify(body) : null; const req = http.request({ host: '127.0.0.1', port: 8090, path: '/api/' + p, method: data ? 'POST' : 'GET', headers: data ? { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(data) } : {} }, r => { let s = ''; r.on('data', d => s += d); r.on('end', () => { try { const j = JSON.parse(s); if (r.statusCode >= 400) rej(new Error(j.error || s)); else res(j); } catch (e) { rej(new Error('bad response: ' + s.slice(0, 200))); } }); }); req.on('error', rej); if (data) req.write(data); req.end(); });
const snapshot = () => new Promise((res, rej) => { let s = ''; const req = http.get('http://127.0.0.1:8090/api/events', r => { r.on('data', d => { s += d; const i = s.indexOf('event: snapshot'); if (i >= 0 && s.indexOf('\n\n', i) > 0) { req.destroy(); try { res(JSON.parse(/event: snapshot\ndata: (.*)\n/.exec(s)[1])); } catch (e) { rej(e); } } }); }); req.on('error', rej); setTimeout(() => { req.destroy(); rej(new Error('snapshot timeout')); }, 8000); });

function newestLog() { const d = path.join(RUN, 'logs'); if (!fs.existsSync(d)) return null; const f = fs.readdirSync(d).filter(x => /\.BIN$/i.test(x)).map(x => ({ x, t: fs.statSync(path.join(d, x)).mtimeMs })).sort((a, b) => b.t - a.t)[0]; return f ? path.join(d, f.x) : null; }
function startSitl() {
  fs.mkdirSync(RUN, { recursive: true });
  const p = spawn(cfg.exe, ['--model', cfg.model, '--speedup', '1', '--home', '-35.363261,149.165230,584,353', '--defaults', cfg.defaults], { cwd: RUN, stdio: ['ignore', fs.openSync(path.join(RUN, 'sitl.log'), 'a'), fs.openSync(path.join(RUN, 'sitl.log'), 'a')], windowsHide: true });
  p.on('exit', c => log('sitl exited', c));
  return p;
}
function startPilot(mode, minutes) {
  const out = fs.openSync(path.join(RUN, `pilot-${mode}.log`), 'a');
  const p = spawn(process.execPath, [path.join(SITL, 'pilot.js'), '--vehicle', vehicle, '--port', '5760', '--mode', mode, '--minutes', String(minutes)], { cwd: ROOT, stdio: ['ignore', out, out], windowsHide: true });
  return p;
}
const waitExit = (p, ms) => new Promise(res => { let done = false; p.on('exit', () => { if (!done) { done = true; res(true); } }); setTimeout(() => { if (!done) { done = true; res(false); } }, ms); });
function kill(p) { try { p.kill(); } catch (e) { /* ignore */ } }
async function waitForTakeoff(mode) { const f = path.join(RUN, `pilot-${mode}.log`); const t0 = Date.now(); while (Date.now() - t0 < 180000) { await sleep(2000); if (fs.existsSync(f) && /at altitude/.test(fs.readFileSync(f, 'utf8'))) return true; } return false; }

async function flight(mode, minutes, label) {
  log(`--- flight: ${label} (${mode}, ${minutes} min) ---`);
  const sitl = startSitl(); await sleep(2000);
  const pilot = startPilot(mode, minutes);
  const ok = await waitExit(pilot, (minutes + 5) * 60000);
  if (!ok) { log('pilot did not finish in time; stopping it'); kill(pilot); }
  await sleep(3000); kill(sitl); await sleep(2000);
  const lg = newestLog(); if (!lg) throw new Error('no log written for ' + label);
  const dst = path.join(DEMO, label + '.BIN'); fs.copyFileSync(lg, dst); log('saved', dst, (fs.statSync(dst).size / 1048576).toFixed(1), 'MB');
  return dst;
}

async function post(meta) {
  if (!postOnly) {
  // 1) before
  if (!retune) await flight('sequence', 2, 'before');
  if (!retune) meta.steps.push('Flight 1 (before): take-off to 12 m, then the fixed stick sequence (24 inputs, roll and pitch, 180-420 us, 2 min) with typical fresh-build starting gains (rate P 0.07 / 0.06, D 0.0015 / 0.0008: below the values the simulator model was tuned with, as on a new build).');
  // 2) tuning flight through the app
  log('--- flight: tuning ---');
  const sitl2 = startSitl(); await sleep(2000);
  const pilot2 = startPilot('random', tuneMinutes + 2);
  if (!(await waitForTakeoff('random'))) throw new Error('tuning flight did not take off');
  await sleep(3000);
  const ac = await api('autoconnect', {}); if (!ac.ok) throw new Error('app could not connect: ' + JSON.stringify(ac.tried)); log('app connected via', ac.found.label);
  await sleep(1500);
  const prep = await api('prepare', { confirm: true }); log('vehicle read:', prep.family, 'checks:', prep.checks.map(c => (c.ok ? 'ok ' : '!! ') + c.title).join(' | '));
  meta.steps.push(`Flight 2 (tuning): the app connected by itself (${ac.found.label}), read the vehicle (${prep.family}) and set ${prep.writes ? prep.writes.length : 0} housekeeping values with confirmation.`);
  // notch first (wiki order: filters, then gains)
  try { const ns = await api('notch/status'); if (ns.escProposal) { const r = await api('notch/esc', {}); meta.notch = Object.entries(r.applied).map(([name, to]) => ({ name, from: ns.current[name], to })); meta.steps.push('Harmonic notch set to ESC RPM tracking (the simulated ESCs report RPM): ' + Object.entries(r.applied).map(([k, v]) => k.replace('INS_HNTCH_', '') + '=' + v).join(', ') + '.'); log('notch set via ESC telemetry'); } else log('no ESC telemetry for the notch'); } catch (e) { log('notch step skipped:', e.message); }
  await api('start', { preset: 'normal', autoApply: true, axes: ['roll', 'pitch'], size: 'medium', observeSeconds: 30 });
  meta.steps.push('AP Tuner session: feel "Normal", axes roll and pitch, auto-apply on. Random stick taps from the scripted pilot. Each 30 s window: identify the airframe, propose, write in bounded steps under the oscillation watchdog, re-check.');
  const t0 = Date.now(); let last = '';
  while (Date.now() - t0 < tuneMinutes * 60000) {
    await sleep(5000);
    let s; try { s = await snapshot(); } catch (e) { continue; }
    const line = `${s.state} | ${s.info}`; if (line !== last) { log('app:', line); last = line; }
    if (s.state === 'done') break;
    if (s.state === 'paused') { log('app paused: ' + s.info + ' -> resuming'); await api('resume', {}).catch(() => {}); }
  }
  const snapEnd = await snapshot();
  meta.gainsAfter = snapEnd.gains; meta.journal = snapEnd.journal; meta.state = snapEnd.state;
  meta.steps.push(`Tuning ended in state "${snapEnd.state}" after ${((Date.now() - t0) / 60000).toFixed(1)} min with ${snapEnd.journal.length} parameter writes (all journaled).`);
  const rows = await api('live/rows'); fs.writeFileSync(path.join(DEMO, 'tuned.param'), ['# AP Tuner demo: gains on the vehicle after the tuning flight', ...rows.rows.map(r => `${r.name},${r.value}`)].join('\r\n'));
  await api('stop', {}).catch(() => {}); await api('disconnect', {}).catch(() => {});
  kill(pilot2); await sleep(3000); kill(sitl2); await sleep(2000);
  { const lg = newestLog(); if (lg) fs.copyFileSync(lg, path.join(DEMO, 'tuning.BIN')); }
  // 3) after (same eeprom -> tuned gains)
  await flight('sequence', 2, 'after');
  meta.steps.push('Flight 3 (after): the identical stick sequence with the tuned gains (the simulator kept its parameter memory).');
  }
  // 4) analyses: app log reports + comparison data
  for (const which of ['before', 'after']) {
    try { const r = await api('log/analyze', { path: path.join(DEMO, which + '.BIN'), preset: 'normal', family: cfg.family }); fs.writeFileSync(path.join(DEMO, which + '-analysis.json'), JSON.stringify(r)); log(which, 'app report: ' + ['roll', 'pitch'].map(a => a + ' ' + (r.axes[a].diagnostics ? r.axes[a].diagnostics.grade.letter + ' (' + r.axes[a].diagnostics.grade.score + ')' : '-')).join(', ')); meta[which + 'Grades'] = Object.fromEntries(['roll', 'pitch'].map(a => [a, r.axes[a].diagnostics ? r.axes[a].diagnostics.grade : null])); } catch (e) { log('app report failed:', e.message); }
    const d = extract(path.join(DEMO, which + '.BIN'), cfg.family); fs.writeFileSync(path.join(DEMO, which + '.json'), JSON.stringify(d));
    for (const [a, m] of Object.entries(d.axes)) log(which, a, `tracking ${m.trackingPct && m.trackingPct.toFixed(0)}% err ${m.rmsErrorDegs.toFixed(1)} lag ${m.lagMs.toFixed(0)} ms` + (m.step ? ` step rise ${m.step.riseMs.toFixed(0)} ms os ${m.step.overshootPct.toFixed(0)}%` : '') + (m.angle && m.angle.step ? ` angle rise ${m.angle.step.riseMs.toFixed(0)} ms os ${m.angle.step.overshootPct.toFixed(0)}%` : ''));
  }
  meta.subtitle = `${(JSON.parse(fs.readFileSync(path.join(DEMO, 'before.json'))).firmware || '')} · simulator with motor vibration and ESC telemetry · ${new Date().toLocaleString()}`;
  meta.video = `${vehicle}-before-after.avi`;
  fs.writeFileSync(path.join(DEMO, 'meta.json'), JSON.stringify(meta, null, 1));
  execFileSync(process.execPath, [path.join(__dirname, 'make-report.js'), DEMO], { stdio: 'inherit' });
  // 5) video: scene file for the renderer (first 70 s of each sequence)
  const b = JSON.parse(fs.readFileSync(path.join(DEMO, 'before.json'))), a = JSON.parse(fs.readFileSync(path.join(DEMO, 'after.json')));
  const scene = { title: cfg.title, kind: cfg.kind, fps: 12, duration: Math.min(70, Math.min(b.series.att.t.at(-1), a.series.att.t.at(-1))),
    before: { att: b.series.att, rc: b.series.rc, roll: b.series.roll, pitch: b.series.pitch, gains: { roll: Object.fromEntries(Object.entries(b.gains.roll).map(([k, v]) => [k, +(+v).toPrecision(4)])) }, metrics: b.axes },
    after: { att: a.series.att, rc: a.series.rc, roll: a.series.roll, pitch: a.series.pitch, gains: { roll: Object.fromEntries(Object.entries(a.gainsEnd.roll).map(([k, v]) => [k, +(+v).toPrecision(4)])) }, metrics: a.axes } };
  for (const side of [scene.before, scene.after]) for (const ax of ['roll', 'pitch']) if (side.metrics[ax] && side.metrics[ax].step) { side.metrics[ax].step = { riseMs: side.metrics[ax].step.riseMs, overshootPct: side.metrics[ax].step.overshootPct }; if (side.metrics[ax].angle && side.metrics[ax].angle.step) side.metrics[ax].angle.step = { riseMs: side.metrics[ax].angle.step.riseMs, overshootPct: side.metrics[ax].angle.step.overshootPct }; }
  fs.writeFileSync(path.join(DEMO, 'scene.json'), JSON.stringify(scene));
  log('rendering video frames...');
  execFileSync('powershell.exe', ['-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', path.join(__dirname, 'render-frames.ps1'), path.join(DEMO, 'scene.json'), path.join(DEMO, 'frames.bin')], { stdio: 'inherit' });
  execFileSync(process.execPath, [path.join(__dirname, 'make-avi.js'), path.join(DEMO, 'frames.bin'), path.join(DEMO, meta.video), '12', '1280', '720'], { stdio: 'inherit' });
  fs.rmSync(path.join(DEMO, 'frames.bin'), { force: true });
  log('DEMO COMPLETE:', DEMO);
}
(async () => {
  fs.mkdirSync(DEMO, { recursive: true });
  let meta;
  if (postOnly) { meta = JSON.parse(fs.readFileSync(path.join(DEMO, 'meta.json'), 'utf8')); }
  else if (retune) {
    const old = JSON.parse(fs.readFileSync(path.join(DEMO, 'meta.json'), 'utf8'));
    if (fs.existsSync(RUN)) fs.rmSync(RUN, { recursive: true, force: true });
    fs.mkdirSync(RUN, { recursive: true });
    meta = { title: cfg.title, vehicle, family: cfg.family, kind: cfg.kind, steps: [old.steps[0] || 'Flight 1 (before): the fixed stick sequence with the starting gains.'], notch: [], started: new Date().toISOString(), retuned: true };
  }
  else {
    // fresh start: new eeprom (starting gains from the defaults file), empty log folder
    if (fs.existsSync(RUN)) fs.rmSync(RUN, { recursive: true, force: true });
    fs.mkdirSync(RUN, { recursive: true });
    meta = { title: cfg.title, vehicle, family: cfg.family, kind: cfg.kind, steps: [], notch: [], started: new Date().toISOString() };
  }
  await post(meta);
})().catch(e => { console.error('DEMO FAILED:', e); process.exit(1); });
