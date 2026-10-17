// Exercises the guided flight session on the simulator through the app's API (the app must be running).
// Starts a SITL vehicle + the scripted pilot, connects the app to the simulator (TCP 5762), turns on the
// experimental in-flight changes (this is a simulator run), starts a flight session, confirms the review step,
// and prints every phase change, every pause and every trip of the oscillation monitor / post-change watch,
// and the final summary. A paused session is resumed after 5 s (as a pilot would once it flies normally), but
// every pause is counted and reported (2026-10-17 audit F-55): a "completed" run is no proof that nothing tripped.
// Usage: node tools/flight-demo.js [--port 8091] [--vehicle quad|quadplane] [--minutes 9] [--auto] [--hold-review 0]
//        [--defaults file.parm] [--no-resume] [--label js]   (the run folder is sitl/run-flight[-label])
'use strict';
const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');
const argv = process.argv.slice(2); const opt = (k, d) => { const i = argv.indexOf('--' + k); return i >= 0 ? argv[i + 1] : d; };
const PORT = parseInt(opt('port', '8091'), 10);
const vehicle = opt('vehicle', 'quad');
const minutes = parseFloat(opt('minutes', '9'));
const holdReview = parseFloat(opt('hold-review', '0'));
const auto = argv.includes('--auto');
const autoResume = !argv.includes('--no-resume');
const defaultsOpt = opt('defaults', null);
const LABEL = opt('label', '');
const ROOT = path.join(__dirname, '..'); const SITL = path.join(ROOT, 'sitl'); const RUN = path.join(SITL, 'run-flight' + (LABEL ? '-' + LABEL : ''));
const cfg = vehicle === 'quadplane' ? { exe: path.join(SITL, 'ArduPlane.exe'), model: 'quadplane', defaults: defaultsOpt ? path.resolve(defaultsOpt) : path.join(SITL, 'demo-quadplane.parm') } : { exe: path.join(SITL, 'ArduCopter.exe'), model: 'quad', defaults: defaultsOpt ? path.resolve(defaultsOpt) : path.join(SITL, 'demo-quad.parm') };
const sleep = ms => new Promise(r => setTimeout(r, ms));
const log = (...a) => console.log(new Date().toISOString().slice(11, 19), ...a);
const { api, snapshot, getToken } = require('./app-client')(PORT);

(async () => {
  if (fs.existsSync(RUN)) fs.rmSync(RUN, { recursive: true, force: true }); fs.mkdirSync(RUN, { recursive: true });
  for (let i = 0; i < 20; i++) { try { await getToken(); break; } catch (e) { if (i === 19) throw new Error('app not reachable on port ' + PORT); await sleep(1000); } }
  const sitl = spawn(cfg.exe, ['--model', cfg.model, '--speedup', '1', '--home', '-35.363261,149.165230,584,353', '--defaults', cfg.defaults], { cwd: RUN, stdio: ['ignore', fs.openSync(path.join(RUN, 'sitl.log'), 'a'), fs.openSync(path.join(RUN, 'sitl.log'), 'a')], windowsHide: true });
  await sleep(2000);
  const pilot = spawn(process.execPath, [path.join(SITL, 'pilot.js'), '--vehicle', vehicle, '--port', '5760', '--mode', 'random', '--minutes', String(minutes)], { cwd: ROOT, stdio: ['ignore', fs.openSync(path.join(RUN, 'pilot.log'), 'a'), fs.openSync(path.join(RUN, 'pilot.log'), 'a')], windowsHide: true });
  const cleanup = () => { try { pilot.kill(); } catch (e) { /* ignore */ } try { sitl.kill(); } catch (e) { /* ignore */ } };
  const pauses = [], trips = [], tStart = Date.now();
  try {
    await sleep(6000);
    // straight to the simulator (autoconnect could pick up a Mission Planner mirror of a real vehicle)
    await api('connect', { kind: 'tcp', host: '127.0.0.1', port: 5762 });
    for (let i = 0; i < 40; i++) { const s = await snapshot().catch(() => null); if (s && s.alive) break; if (i === 39) throw new Error('no heartbeat from the simulator on 5762'); await sleep(500); }
    log('app connected via Simulator (SITL, port 5762)');
    await sleep(1500);
    const prep = await api('prepare', { confirm: true }); log('vehicle read:', prep.family, '| checks:', prep.checks.map(c => (c.ok ? 'ok ' : '!! ') + c.title).join(' | '));
    await api('options', { inFlightWrites: true }); log('changes in flight turned ON (experimental; this is a simulator run)');
    const st = await api('flight/start', { axes: ['roll', 'pitch'], preset: 'normal', size: 'medium', auto, observeSeconds: 30 });
    log('flight session started, phase', st.flight.phase);
    let last = '', reviewed = false, lastInfo = '', verdictShown = false, pausedAt = 0, inPause = false, lastTripAt = 0;
    for (;;) {
      await sleep(3000);
      let s; try { s = await snapshot(); } catch (e) { continue; }
      const f = s.flight;
      if (f.phase !== last) { log('PHASE', f.phase, '-', f.info); last = f.phase; }
      else if (f.info !== lastInfo && /tuning|recheck/.test(f.phase)) { log('   ', f.info); }
      lastInfo = f.info;
      const lt = s.monitor && s.monitor.lastTrip;
      if (lt && lt.at !== lastTripAt && lt.at >= tStart) { lastTripAt = lt.at; trips.push({ at: new Date(lt.at).toISOString().slice(11, 19), source: lt.source, axis: lt.axis, reasons: lt.reasons }); log(`  TRIP (${lt.source}): ${(lt.reasons || []).join('; ')}`); }
      // a paused session waits for the pilot: count the pause, then press Resume after 5 s (once nothing is being written)
      const paused = f.phase === 'paused' || /^paused/.test(f.info || '');
      if (paused && !inPause) { inPause = true; pauses.push({ at: new Date().toISOString().slice(11, 19), phase: f.phase === 'paused' ? f.resumePhase : f.phase, info: f.pausedInfo || f.info }); log(`  PAUSE ${pauses.length}: ${f.pausedInfo || f.info}`); }
      if (!paused) inPause = false;
      if (paused && autoResume) { if (!pausedAt) pausedAt = Date.now(); else if (Date.now() - pausedAt > 5000 && !s.writing) { log('  -> resume'); await api('flight/resume', {}).catch(e => log('  resume failed:', e.message)); pausedAt = 0; } } else pausedAt = 0;
      if (f.phase === 'review' && !reviewed) {
        reviewed = true;
        for (const [ax, a] of Object.entries(f.checkup.axes)) { log(`  ${ax}: grade ${a.grade ? a.grade.letter : '?'} | ` + (a.issues || []).map(i => `[${i.level}] ${i.text}`).join(' | ')); if (a.proposal) log(`  ${ax} proposed: ` + a.proposal.changes.map(c => `${c.key} ${(+c.from).toPrecision(3)}->${(+c.to).toPrecision(3)} (${Math.round(c.pct)}%)`).join(', ')); }
        if (f.checkup.vehicleIssues.length) log('  vehicle: ' + f.checkup.vehicleIssues.map(i => i.text).join(' | '));
        if (holdReview) { log(`holding at review for ${holdReview} s`); await sleep(holdReview * 1000); }
        if (f.checkup.summary.anyProposal) { await api('flight/continue', {}); log('  -> continue (apply in flight)'); } else { log('  no proposal; repeating'); await api('flight/repeat', {}); reviewed = false; }
      }
      if (f.phase === 'land' && f.overall && !verdictShown) { verdictShown = true; log('  verdict:', f.overall.verdict.ok ? 'GOOD' : 'points to look at'); for (const it of f.overall.verdict.items) log('   ', it.ok ? 'ok ' : '!! ', it.text); }
      if (f.phase === 'finished') { const sm = f.summary; log('SUMMARY:', sm.reason, '| writes', sm.writes, '| notch', sm.filters && sm.filters.result); for (const ax of sm.axes) { const b = sm.before && sm.before[ax], a = sm.after && sm.after[ax]; log(`  ${ax}: grade ${b && b.grade} -> ${a && a.grade} | tracking ${b && b.trackingPct && b.trackingPct.toFixed(0)}% -> ${a && a.trackingPct && a.trackingPct.toFixed(0)}% | err ${b && b.rmsErrorDegs && b.rmsErrorDegs.toFixed(1)} -> ${a && a.rmsErrorDegs && a.rmsErrorDegs.toFixed(1)} deg/s`); log(`  ${ax} gains: P ${sm.gainsStart[ax].P.toPrecision(3)} -> ${sm.gainsEnd[ax].P.toPrecision(3)}, D ${sm.gainsStart[ax].D.toPrecision(3)} -> ${sm.gainsEnd[ax].D.toPrecision(3)}, angleP ${sm.gainsStart[ax].angleP} -> ${sm.gainsEnd[ax].angleP}`); } fs.writeFileSync(path.join(RUN, 'summary.json'), JSON.stringify(Object.assign({ pauses, trips }, sm), null, 1)); break; }
      if (f.phase === 'stopped') { log('session stopped'); break; }
    }
    await api('stop', {}).catch(() => {}); await api('disconnect', {}).catch(() => {});
  } finally { cleanup(); }
  log(`PAUSES: ${pauses.length}${pauses.map((p, i) => `\n  ${i + 1}. ${p.at} (${p.phase}) ${p.info}`).join('')}`);
  log(`TRIPS: ${trips.length}${trips.map((t, i) => `\n  ${i + 1}. ${t.at} ${t.source} ${t.axis}: ${(t.reasons || []).join('; ')}`).join('')}`);
  const logs = path.join(RUN, 'logs'); if (fs.existsSync(logs)) log('SITL logs:', fs.readdirSync(logs).filter(n => /\.bin$/i.test(n)).map(n => path.join(logs, n)).join(', '));
  log('FLIGHT DEMO COMPLETE');
  process.exit(0);
})().catch(e => { console.error('FLIGHT DEMO FAILED:', e); process.exit(1); });
