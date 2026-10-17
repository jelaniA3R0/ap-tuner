// Exercises the guided flight session on the simulator through the app's API (the app must be running).
// Starts a SITL vehicle + the scripted pilot, connects the app, starts a flight session, confirms the
// review step, and prints every phase change and the final summary.
// Usage: node tools/flight-demo.js [--port 8091] [--vehicle quad|quadplane] [--minutes 9] [--auto] [--hold-review 0]
'use strict';
const fs = require('fs');
const path = require('path');
const http = require('http');
const { spawn } = require('child_process');
const argv = process.argv.slice(2); const opt = (k, d) => { const i = argv.indexOf('--' + k); return i >= 0 ? argv[i + 1] : d; };
const PORT = parseInt(opt('port', '8091'), 10);
const vehicle = opt('vehicle', 'quad');
const minutes = parseFloat(opt('minutes', '9'));
const holdReview = parseFloat(opt('hold-review', '0'));
const auto = argv.includes('--auto');
const defaultsOpt = opt('defaults', null);
const ROOT = path.join(__dirname, '..'); const SITL = path.join(ROOT, 'sitl'); const RUN = path.join(SITL, 'run-flight');
const cfg = vehicle === 'quadplane' ? { exe: path.join(SITL, 'ArduPlane.exe'), model: 'quadplane', defaults: defaultsOpt ? path.resolve(defaultsOpt) : path.join(SITL, 'demo-quadplane.parm') } : { exe: path.join(SITL, 'ArduCopter.exe'), model: 'quad', defaults: defaultsOpt ? path.resolve(defaultsOpt) : path.join(SITL, 'demo-quad.parm') };
const sleep = ms => new Promise(r => setTimeout(r, ms));
const log = (...a) => console.log(new Date().toISOString().slice(11, 19), ...a);
const api = (p, body) => new Promise((res, rej) => { const data = body !== undefined ? JSON.stringify(body) : null; const req = http.request({ host: '127.0.0.1', port: PORT, path: '/api/' + p, method: data ? 'POST' : 'GET', headers: data ? { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(data) } : {} }, r => { let s = ''; r.on('data', d => s += d); r.on('end', () => { try { const j = JSON.parse(s); if (r.statusCode >= 400) rej(new Error(j.error || s)); else res(j); } catch (e) { rej(new Error('bad response: ' + s.slice(0, 200))); } }); }); req.on('error', rej); if (data) req.write(data); req.end(); });
const snapshot = () => new Promise((res, rej) => { let s = ''; const req = http.get(`http://127.0.0.1:${PORT}/api/events`, r => { r.on('data', d => { s += d; const i = s.indexOf('event: snapshot'); if (i >= 0 && s.indexOf('\n\n', i) > 0) { req.destroy(); try { res(JSON.parse(/event: snapshot\ndata: (.*)\n/.exec(s)[1])); } catch (e) { rej(e); } } }); }); req.on('error', rej); setTimeout(() => { req.destroy(); rej(new Error('snapshot timeout')); }, 8000); });

(async () => {
  if (fs.existsSync(RUN)) fs.rmSync(RUN, { recursive: true, force: true }); fs.mkdirSync(RUN, { recursive: true });
  for (let i = 0; i < 20; i++) { try { await api('presets'); break; } catch (e) { if (i === 19) throw new Error('app not reachable on port ' + PORT); await sleep(1000); } }
  const sitl = spawn(cfg.exe, ['--model', cfg.model, '--speedup', '1', '--home', '-35.363261,149.165230,584,353', '--defaults', cfg.defaults], { cwd: RUN, stdio: ['ignore', fs.openSync(path.join(RUN, 'sitl.log'), 'a'), fs.openSync(path.join(RUN, 'sitl.log'), 'a')], windowsHide: true });
  await sleep(2000);
  const pilot = spawn(process.execPath, [path.join(SITL, 'pilot.js'), '--vehicle', vehicle, '--port', '5760', '--mode', 'random', '--minutes', String(minutes)], { cwd: ROOT, stdio: ['ignore', fs.openSync(path.join(RUN, 'pilot.log'), 'a'), fs.openSync(path.join(RUN, 'pilot.log'), 'a')], windowsHide: true });
  const cleanup = () => { try { pilot.kill(); } catch (e) { /* ignore */ } try { sitl.kill(); } catch (e) { /* ignore */ } };
  try {
    await sleep(6000);
    const ac = await api('autoconnect', {}); if (!ac.ok) throw new Error('app could not connect: ' + JSON.stringify(ac.tried)); log('app connected via', ac.found.label);
    await sleep(1500);
    const prep = await api('prepare', { confirm: true }); log('vehicle read:', prep.family, '| checks:', prep.checks.map(c => (c.ok ? 'ok ' : '!! ') + c.title).join(' | '));
    const st = await api('flight/start', { axes: ['roll', 'pitch'], preset: 'normal', size: 'medium', auto, observeSeconds: 30 });
    log('flight session started, phase', st.flight.phase);
    let last = '', reviewed = false, lastInfo = '', verdictShown = false;
    for (;;) {
      await sleep(3000);
      let s; try { s = await snapshot(); } catch (e) { continue; }
      const f = s.flight; const line = `${f.phase} | ${f.info}`;
      if (f.phase !== last) { log('PHASE', f.phase, '-', f.info); last = f.phase; }
      else if (f.info !== lastInfo && /tuning|recheck/.test(f.phase)) { log('   ', f.info); }
      lastInfo = f.info;
      if (f.phase === 'review' && !reviewed) {
        reviewed = true;
        for (const [ax, a] of Object.entries(f.checkup.axes)) { log(`  ${ax}: grade ${a.grade ? a.grade.letter : '?'} | ` + (a.issues || []).map(i => `[${i.level}] ${i.text}`).join(' | ')); if (a.proposal) log(`  ${ax} proposed: ` + a.proposal.changes.map(c => `${c.key} ${(+c.from).toPrecision(3)}->${(+c.to).toPrecision(3)} (${Math.round(c.pct)}%)`).join(', ')); }
        if (f.checkup.vehicleIssues.length) log('  vehicle: ' + f.checkup.vehicleIssues.map(i => i.text).join(' | '));
        if (holdReview) { log(`holding at review for ${holdReview} s`); await sleep(holdReview * 1000); }
        if (f.checkup.summary.anyProposal) { await api('flight/continue', {}); log('  -> continue (apply in flight)'); } else { log('  no proposal; repeating'); await api('flight/repeat', {}); reviewed = false; }
      }
      if (f.phase === 'land' && f.overall && !verdictShown) { verdictShown = true; log('  verdict:', f.overall.verdict.ok ? 'GOOD' : 'points to look at'); for (const it of f.overall.verdict.items) log('   ', it.ok ? 'ok ' : '!! ', it.text); }
      if (f.phase === 'finished') { const sm = f.summary; log('SUMMARY:', sm.reason, '| writes', sm.writes, '| notch', sm.filters && sm.filters.result); for (const ax of sm.axes) { const b = sm.before && sm.before[ax], a = sm.after && sm.after[ax]; log(`  ${ax}: grade ${b && b.grade} -> ${a && a.grade} | tracking ${b && b.trackingPct && b.trackingPct.toFixed(0)}% -> ${a && a.trackingPct && a.trackingPct.toFixed(0)}% | err ${b && b.rmsErrorDegs && b.rmsErrorDegs.toFixed(1)} -> ${a && a.rmsErrorDegs && a.rmsErrorDegs.toFixed(1)} deg/s`); log(`  ${ax} gains: P ${sm.gainsStart[ax].P.toPrecision(3)} -> ${sm.gainsEnd[ax].P.toPrecision(3)}, D ${sm.gainsStart[ax].D.toPrecision(3)} -> ${sm.gainsEnd[ax].D.toPrecision(3)}, angleP ${sm.gainsStart[ax].angleP} -> ${sm.gainsEnd[ax].angleP}`); } fs.writeFileSync(path.join(RUN, 'summary.json'), JSON.stringify(sm, null, 1)); break; }
      if (f.phase === 'stopped') { log('session stopped'); break; }
    }
    await api('stop', {}).catch(() => {}); await api('disconnect', {}).catch(() => {});
  } finally { cleanup(); }
  log('FLIGHT DEMO COMPLETE');
  process.exit(0);
})().catch(e => { console.error('FLIGHT DEMO FAILED:', e); process.exit(1); });
