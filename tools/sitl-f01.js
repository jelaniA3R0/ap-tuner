// SITL regression for the 2026-10-17 audit finding F-01: an in-flight notch change made the simulated quad oscillate
// for minutes while the app noticed nothing. Here the quad flies with pitch P 0.18, I 0.18, D 0.0057, angle P 12 and
// no notch (the audit run's pitch gains). The app under test (the JavaScript or C++ server on --port) connects to the
// simulator (TCP 5762), reads the vehicle, gets the experimental in-flight changes turned on, starts a session
// (observing pitch, nothing written) and, once the quad hovers, sets the notch to ESC RPM tracking through its API, as
// the Notch panel does. Pass: the app reverts the notch (INS_HNTCH_ENABLE back to 0) and pauses within 5 s of the
// oscillation onset, and the oscillation stops. Checked from the app (journal, messages, state) and from the
// simulator's own log (PARM records and PIDP slew rate). The pilot (sitl/pilot.js on 5760) taps the sticks at random.
// Usage: node tools/sitl-f01.js --port 8093 [--label js] [--minutes 2.5]
'use strict';
const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');
const argv = process.argv.slice(2); const opt = (k, d) => { const i = argv.indexOf('--' + k); return i >= 0 ? argv[i + 1] : d; };
const PORT = parseInt(opt('port', '8093'), 10);
const LABEL = opt('label', String(PORT));
const MINUTES = parseFloat(opt('minutes', '2.5'));
const ROOT = path.join(__dirname, '..'); const SITL = path.join(ROOT, 'sitl'); const RUN = path.join(SITL, 'run-f01-' + LABEL);
const sleep = ms => new Promise(r => setTimeout(r, ms));
const t00 = Date.now(); const log = (...a) => console.log(`${((Date.now() - t00) / 1000).toFixed(1).padStart(6)} s`, ...a);
const { api, snapshot, getToken } = require('./app-client')(PORT);
const { parseBin } = require(path.join(ROOT, 'src', 'log', 'dfreader'));
const { parmRecords } = require('./parm-records');

async function waitSnap(pred, ms, what) {
  const t0 = Date.now();
  for (;;) { const s = await snapshot().catch(() => null); if (s && pred(s)) return s; if (Date.now() - t0 > ms) throw new Error('timeout: ' + what); await sleep(250); }
}

(async () => {
  if (fs.existsSync(RUN)) fs.rmSync(RUN, { recursive: true, force: true }); fs.mkdirSync(RUN, { recursive: true });
  const defaults = path.join(RUN, 'f01.parm');
  // INS_HNTCH_REF acts only after a reboot, so the app never writes it in flight (R3, F-35): the quad starts with
  // REF 1 as after a notch set up on the ground, and the in-flight change switches the notch on (ENABLE, MODE, FREQ)
  fs.writeFileSync(defaults, fs.readFileSync(path.join(SITL, 'demo-quad.parm'), 'utf8') + '\n# F-01 regression: the pitch gains of the audit run, no notch\nATC_RAT_PIT_P 0.18\nATC_RAT_PIT_I 0.18\nATC_RAT_PIT_D 0.0057\nATC_ANG_PIT_P 12\nINS_HNTCH_ENABLE 0\nINS_HNTCH_REF 1\n');
  await getToken();
  const sitl = spawn(path.join(SITL, 'ArduCopter.exe'), ['--model', 'quad', '--speedup', '1', '--home', '-35.363261,149.165230,584,353', '--defaults', defaults], { cwd: RUN, stdio: ['ignore', fs.openSync(path.join(RUN, 'sitl.log'), 'a'), fs.openSync(path.join(RUN, 'sitl.log'), 'a')], windowsHide: true });
  await sleep(2000);
  const pilot = spawn(process.execPath, [path.join(SITL, 'pilot.js'), '--vehicle', 'quad', '--port', '5760', '--mode', 'random', '--minutes', String(MINUTES)], { cwd: ROOT, stdio: ['ignore', fs.openSync(path.join(RUN, 'pilot.log'), 'a'), fs.openSync(path.join(RUN, 'pilot.log'), 'a')], windowsHide: true });
  const result = { label: LABEL, port: PORT, pass: false, checks: [] };
  const check = (ok, what) => { result.checks.push({ ok: !!ok, what }); log(ok ? 'PASS' : 'FAIL', what); return ok; };
  try {
    await sleep(5000);
    await api('connect', { kind: 'tcp', host: '127.0.0.1', port: 5762 });
    await waitSnap(s => s.alive, 20000, 'heartbeat on 5762');
    await sleep(1500);
    const prep = await api('prepare', { confirm: true });
    log('vehicle read:', prep.family, '| pitch P', prep.gains.pitch.P.toPrecision(3), 'D', prep.gains.pitch.D.toPrecision(3), 'angle P', prep.gains.pitch.angleP, '| notch', prep.checks.find(c => /notch/i.test(c.title)).title);
    await api('options', { inFlightWrites: true, axes: ['pitch'], autoApply: false, preset: 'normal' });
    log('changes in flight turned ON (experimental; simulator run)');
    await waitSnap(s => s.vehicle && s.vehicle.armed, 120000, 'arming');
    log('armed: waiting for the climb to 12 m and the hover');
    await sleep(22000);
    await api('start', { axes: ['pitch'], autoApply: false, preset: 'normal' });
    log('session started (observing pitch, nothing written)');
    await sleep(10000);
    const st = await api('notch/status');
    if (!check(st.escProposal, `ESC RPM telemetry seen (${Math.round(st.escRpm || 0)} rpm): an ESC notch can be proposed`)) throw new Error('no ESC telemetry');
    log('notch proposal:', JSON.stringify(st.escProposal));
    const tReq = Date.now();
    let reply;
    try { reply = await api('notch/esc', {}); } catch (e) { reply = { error: e.message }; }
    log('notch/esc returned after', ((Date.now() - tReq) / 1000).toFixed(1), 's:', reply.error || 'kept');
    const s1 = await waitSnap(s => s.state === 'paused' && !s.writing, 30000, 'pause after the revert').catch(() => null);
    const snap = s1 || await snapshot();
    const rows = await api('live/rows');
    const en = rows.journal.filter(j => j.name === 'INS_HNTCH_ENABLE' && j.t >= tReq);   // this run's writes only
    const on = en.find(j => j.to === 1), off = en.find(j => j.to === 0 && on && j.t > on.t);
    const trip = snap.monitor && snap.monitor.lastTrip;
    const pauseMsg = (snap.messages || []).find(m => m.kind === 'tuner' && /^paused: .*oscillation detected/.test(m.text));
    result.app = { notchOn: on && on.t, notchOff: off && off.t, tripAt: trip && trip.at, tripReasons: trip && trip.reasons, pausedAt: pauseMsg && pauseMsg.t, finalState: snap.state, finalInfo: snap.info, reply: reply.error || 'kept' };
    log('app: trip', trip ? `${((trip.at - on.t) / 1000).toFixed(2)} s after the notch write: ${trip.reasons.join('; ')}` : 'none');
    log('app: state', snap.state, '|', snap.info);
    check(on, 'the app wrote INS_HNTCH_ENABLE = 1 in flight (the scenario ran)');
    check(trip, 'the app detected the oscillation (trip)');
    check(off, 'the app wrote INS_HNTCH_ENABLE back to 0 (the notch was reverted)');
    check(snap.state === 'paused', 'the app paused the session');
    if (off && trip) log(`app: revert ${((off.t - trip.at) / 1000).toFixed(2)} s after the trip; pause ${pauseMsg ? ((pauseMsg.t - trip.at) / 1000).toFixed(2) + ' s after the trip' : 'not found in the last messages'}`);
    await sleep(25000);   // fly on with the notch off: the oscillation must die out
    await api('stop', {}).catch(() => {});
    log('waiting for the pilot to land');
    await waitSnap(s => s.vehicle && !s.vehicle.armed, 240000, 'landing').catch(() => log('landing not seen'));
    await sleep(3000);
    await api('disconnect', {}).catch(() => {});
  } catch (e) { log('ERROR', e.message); result.error = e.message; }
  finally { try { pilot.kill(); } catch (e) { /* ignore */ } try { sitl.kill(); } catch (e) { /* ignore */ } }
  await sleep(1500);
  // ---- the simulator's own log: onset, revert, and whether the oscillation stopped
  const logs = path.join(RUN, 'logs');
  const bins = fs.existsSync(logs) ? fs.readdirSync(logs).filter(n => /\.bin$/i.test(n)).map(n => path.join(logs, n)).sort((a, b) => fs.statSync(b).mtimeMs - fs.statSync(a).mtimeMs) : [];
  if (check(bins.length, 'the simulator wrote a dataflash log')) {
    const bin = bins[0]; result.log = bin;
    const L = parseBin(bin, { want: ['PIDP'] });
    const ch = parmRecords(bin).filter(c => c.name === 'INS_HNTCH_ENABLE');
    const tOn = (ch.find(c => c.val === 1) || {}).t;
    const tOff = tOn === undefined ? undefined : (ch.find(c => c.val === 0 && c.t > tOn) || {}).t;
    const m = L.msgs.PIDP; const t = Array.from(m.TimeUS, v => v / 1e6);
    let onset = null; for (let i = 0; i < t.length; i++) if (tOn && t[i] > tOn && m.SRate[i] > 5) { onset = t[i]; break; }
    let srAfter = 0; for (let i = 0; i < t.length; i++) if (tOff && t[i] > tOff + 3 && t[i] < tOff + 13) srAfter = Math.max(srAfter, m.SRate[i]);
    let srPeak = 0; for (let i = 0; i < t.length; i++) if (tOn && t[i] > tOn && (!tOff || t[i] < tOff + 3)) srPeak = Math.max(srPeak, m.SRate[i]);
    result.sitl = { notchOn: tOn, notchOff: tOff, onset, revertAfterOnset: onset && tOff ? tOff - onset : null, revertAfterWrite: tOn && tOff ? tOff - tOn : null, peakSlewRate: srPeak, slewRate3to13sAfterRevert: srAfter };
    log(`log: notch on ${tOn && tOn.toFixed(2)} s, first pitch SRate > 5 at ${onset && onset.toFixed(2)} s (+${onset && tOn ? (onset - tOn).toFixed(2) : '?'} s), notch off ${tOff && tOff.toFixed(2)} s; peak SRate ${srPeak.toFixed(1)}; 3-13 s after the revert SRate up to ${srAfter.toFixed(2)}`);
    check(tOn && onset, 'the notch change made pitch oscillate in the simulator (onset found)');
    check(onset && tOff && tOff - onset <= 5, `the notch was reverted within 5 s of the onset (${onset && tOff ? (tOff - onset).toFixed(2) : '?'} s)`);
    check(tOff && srAfter < 5, 'the oscillation stopped after the revert (SRate below 5 from 3 s after it)');
  }
  result.pass = result.checks.every(c => c.ok);
  fs.writeFileSync(path.join(RUN, 'result.json'), JSON.stringify(result, null, 1));
  log(`F-01 REGRESSION (${LABEL}): ${result.pass ? 'PASS' : 'FAIL'}  (${path.join(RUN, 'result.json')})`);
  process.exit(result.pass ? 0 : 1);
})().catch(e => { console.error('F-01 regression failed:', e); process.exit(1); });
