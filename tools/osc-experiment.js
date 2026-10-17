// Experimental check of the model's gain-margin claim on the simulator: with the vehicle hovering,
// raise roll P step by step and watch ArduPilot's own oscillation signal (PID_TUNING SRate) and the
// rate error energy; report the P at which oscillation starts. Uses a second SITL instance (ports 5770+)
// so it can run beside other work.
// Usage: node tools/osc-experiment.js [--vehicle quad|quadplane] [--instance 1] [--fltd 0] [--d 0] [--plist 0.135,0.17,...]
'use strict';
const path = require('path');
const fs = require('fs');
const { spawn } = require('child_process');
const { Link } = require('../src/mavlink/link');
const argv = process.argv.slice(2); const opt = (k, d) => { const i = argv.indexOf('--' + k); return i >= 0 ? argv[i + 1] : d; };
const vehicle = opt('vehicle', 'quad');
const inst = parseInt(opt('instance', '1'), 10);
const fltd = parseFloat(opt('fltd', '0'));   // optional: set the roll FLTD / FLTT before the sweep
const setD = parseFloat(opt('d', '0'));      // optional: set the roll D before the sweep
const base = 5760 + 10 * inst;
const ROOT = path.join(__dirname, '..'); const SITL = path.join(ROOT, 'sitl'); const RUN = path.join(SITL, 'run-osc' + (inst === 1 ? '' : '-' + inst));
const cfg = vehicle === 'quadplane'
  ? { exe: path.join(SITL, 'ArduPlane.exe'), model: 'quadplane', defaults: path.join(SITL, 'demo-quadplane.parm'), pre: 'Q_A_RAT_RLL_', plist: [0.108, 0.135, 0.17, 0.2, 0.23, 0.27, 0.31, 0.36, 0.42, 0.5, 0.6] }
  : { exe: path.join(SITL, 'ArduCopter.exe'), model: 'quad', defaults: path.join(SITL, 'demo-quad.parm'), pre: 'ATC_RAT_RLL_', plist: [0.135, 0.17, 0.2, 0.23, 0.27, 0.31, 0.36, 0.42, 0.5, 0.6, 0.72] };
const plist = opt('plist', null) ? opt('plist').split(',').map(Number) : cfg.plist;
const sleep = ms => new Promise(r => setTimeout(r, ms));
const log = (...a) => console.log(new Date().toISOString().slice(11, 19), ...a);
(async () => {
  if (fs.existsSync(RUN)) fs.rmSync(RUN, { recursive: true, force: true }); fs.mkdirSync(RUN, { recursive: true });
  const sitl = spawn(cfg.exe, ['--model', cfg.model, '--speedup', '1', '--instance', String(inst), '--home', '-35.363261,149.165230,584,353', '--defaults', cfg.defaults], { cwd: RUN, stdio: ['ignore', fs.openSync(path.join(RUN, 'sitl.log'), 'a'), fs.openSync(path.join(RUN, 'sitl.log'), 'a')], windowsHide: true });
  await sleep(2000);
  const pilot = spawn(process.execPath, [path.join(SITL, 'pilot.js'), '--vehicle', vehicle, '--port', String(base), '--mode', 'random', '--minutes', '8'], { cwd: ROOT, stdio: ['ignore', fs.openSync(path.join(RUN, 'pilot.log'), 'a'), fs.openSync(path.join(RUN, 'pilot.log'), 'a')], windowsHide: true });
  const link = new Link();
  const connect = () => link.connect({ kind: 'tcp', host: '127.0.0.1', port: base + 2 });
  link.on('status', st => { if (/closed|error/.test(st) && !link.connected) setTimeout(connect, 2000); });
  await sleep(4000); connect();
  while (!link.alive) await sleep(300);
  log('measurement link alive on port', base + 2);
  // wait for the pilot's take-off
  const t0 = Date.now(); while (Date.now() - t0 < 150000) { await sleep(2000); if (/at altitude/.test(fs.readFileSync(path.join(RUN, 'pilot.log'), 'utf8'))) break; }
  await sleep(8000);
  await link.setParam('GCS_PID_MASK', 7); link.requestDataStream(10, 20); await sleep(1500); link.setMessageRate(194, 50);
  const samples = []; link.on('PID_TUNING', (f, m) => { if (f.axis === 1) samples.push({ t: m.ts, e: (f.desired - f.achieved) * 57.2958, a: f.achieved * 57.2958, sr: f.SRate, dm: f.PDmod }); });
  if (fltd > 0) { await link.setParam(cfg.pre + 'FLTD', fltd); await link.setParam(cfg.pre + 'FLTT', fltd); await sleep(2000); }
  if (setD > 0) { await link.setParam(cfg.pre + 'D', setD); await sleep(2000); }
  const D = await link.requestParam(cfg.pre + 'D'); const P0 = await link.requestParam(cfg.pre + 'P'); const fd = await link.requestParam(cfg.pre + 'FLTD');
  log(`${vehicle} start: P=${P0} D=${D} FLTD=${fd}`);
  const results = [];
  for (const P of plist) {
    await link.setParam(cfg.pre + 'P', P); await link.setParam(cfg.pre + 'I', P);
    samples.length = 0; await sleep(9000);
    const s = samples.slice(Math.floor(samples.length * 0.3));
    // oscillation measure: RMS of the error above 4 Hz (first difference of the error) and max SRate
    let hf = 0, n = 0, srMax = 0, dmMin = 1; for (let i = 1; i < s.length; i++) { const d = s[i].e - s[i - 1].e; hf += d * d; n++; if (s[i].sr > srMax) srMax = s[i].sr; if (s[i].dm < dmMin) dmMin = s[i].dm; }
    const hfRms = Math.sqrt(hf / Math.max(1, n));
    results.push({ P, hfRms, srMax, dmMin, samples: s.length });
    log(`P=${P.toFixed(3)}  high-freq error ${hfRms.toFixed(2)} deg/s  SRate max ${srMax.toFixed(2)}  Dmod min ${dmMin.toFixed(2)}  (${s.length} samples)`);
    if (srMax > 5 || dmMin < 0.8) { log('oscillation detected, stopping the sweep'); break; }
  }
  await link.setParam(cfg.pre + 'P', P0); await link.setParam(cfg.pre + 'I', P0);
  fs.writeFileSync(path.join(RUN, 'results.json'), JSON.stringify({ vehicle, D, fltd: fd, results }, null, 1));
  log('restored P; results in', path.join(RUN, 'results.json'));
  try { pilot.kill(); } catch (e) { /* ignore */ } await sleep(2000); try { sitl.kill(); } catch (e) { /* ignore */ }
  process.exit(0);
})().catch(e => { console.error(e); process.exit(1); });
