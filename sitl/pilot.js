// SITL pilot: connects to the simulator's first MAVLink port (which also lets the simulator start),
// arms, takes off and flies either
//   sequence : a fixed, repeatable set of stick inputs (the same before and after tuning), or
//   random   : random stick taps for a tuning session.
// Usage: node sitl/pilot.js [--vehicle quad|quadplane] [--port 5760] [--mode random|sequence] [--minutes 10]
'use strict';
const { Link } = require('../src/mavlink/link');
const args = {}; const av = process.argv.slice(2);
for (let i = 0; i < av.length; i++) if (av[i].startsWith('--')) { args[av[i].slice(2)] = av[i + 1]; i++; } else if (i === 0 && /^\d+$/.test(av[i])) { args.port = av[i]; if (av[i + 1]) args.minutes = av[i + 1]; i++; }
const vehicle = args.vehicle || 'quad';
const port = parseInt(args.port || '5760', 10);
const mode = args.mode || 'random';
const minutes = parseFloat(args.minutes || (mode === 'sequence' ? '2' : '10'));
const isPlane = vehicle === 'quadplane';
const MODES = isPlane ? { STAB: 17, HOVER: 18, LOITER: 19, LAND: 20 } : { STAB: 0, HOVER: 2, LOITER: 5, LAND: 9 };   // plane: QSTABILIZE/QHOVER/QLOITER/QLAND
const link = new Link();
const log = (...a) => console.log(new Date().toISOString().slice(11, 23), ...a);
link.on('STATUSTEXT', f => log('vehicle:', f.text));
const connect = () => link.connect({ kind: 'tcp', host: '127.0.0.1', port });
link.on('status', s => { log('link:', s); if (/closed|error/.test(s) && !link.connected) setTimeout(connect, 2000); });
connect();
const sleep = ms => new Promise(r => setTimeout(r, ms));
let ekfOk = false, alt = 0, armed = false, modeNow = -1, homeAlt = null, hudAlt = 0, gpiSeen = false;
link.on('EKF_STATUS_REPORT', f => { ekfOk = (f.flags & 0x10) !== 0 || (f.flags & 0x08) !== 0; });
link.on('GLOBAL_POSITION_INT', f => { gpiSeen = true; alt = f.relative_alt / 1000; });
link.on('VFR_HUD', f => { hudAlt = f.alt; if (homeAlt !== null && !gpiSeen) alt = f.alt - homeAlt; });
link.on('heartbeat', v => { armed = v.armed; modeNow = v.mode; });
setInterval(() => log(`alt ${alt.toFixed(1)} mode ${modeNow} armed ${armed}`), 10000);
// RC overrides expire after 3 s: re-send the current values at 10 Hz
let rcNow = null;
function rc(ch) { rcNow = ch; sendRc(); }
function sendRc() { if (!rcNow) return; const f = { target_system: link.target.sysid, target_component: 1 }; for (let i = 1; i <= 18; i++) f['chan' + i + '_raw'] = rcNow[i] !== undefined ? rcNow[i] : 0; link.send('RC_CHANNELS_OVERRIDE', f); }
setInterval(sendRc, 100);
const centre = () => ({ 1: 1500, 2: 1500, 3: 1500, 4: 1500 });
async function setMode(m) { link.commandLong(176, [1, m, 0, 0, 0, 0, 0]); await sleep(500); }

// the repeatable test sequence: [axis(1 roll / 2 pitch), amplitude us, hold ms, pause ms]
const SEQUENCE = [[1, 250, 350, 1200], [1, -250, 350, 1200], [2, 250, 350, 1200], [2, -250, 350, 1200],
  [1, 350, 300, 1300], [1, -350, 300, 1300], [2, 350, 300, 1300], [2, -350, 300, 1300],
  [1, 180, 600, 1000], [1, -180, 600, 1000], [2, 180, 600, 1000], [2, -180, 600, 1000],
  [1, 420, 250, 1400], [1, -420, 250, 1400], [2, 420, 250, 1400], [2, -420, 250, 1400],
  [1, 300, 200, 700], [1, -300, 200, 700], [1, 300, 200, 700], [1, -300, 200, 1500],
  [2, 300, 200, 700], [2, -300, 200, 700], [2, 300, 200, 700], [2, -300, 200, 1500]];

(async () => {
  while (!link.alive) await sleep(300);
  log('heartbeat ok, waiting for EKF/GPS');
  link.requestDataStream(0, 4); link.requestDataStream(6, 4); link.setMessageRate(193, 1); link.setMessageRate(33, 2);
  const t0 = Date.now();
  while (!ekfOk && Date.now() - t0 < 120000) await sleep(1000);
  log('ekf', ekfOk ? 'ready' : 'timeout, trying anyway');
  await sleep(2000);
  // manual take-off in AltHold / QHOVER: throttle low, arm, raise throttle until 12 m
  rc({ 1: 1500, 2: 1500, 3: 1000, 4: 1500 });
  await setMode(MODES.HOVER);
  for (let i = 0; i < 6 && !armed; i++) { link.commandLong(400, [1, 21196, 0, 0, 0, 0, 0]); await sleep(1500); }
  log('armed:', armed); homeAlt = hudAlt;
  rc({ 1: 1500, 2: 1500, 3: 1750, 4: 1500 });
  const tk = Date.now(); while (alt < 12 && Date.now() - tk < 40000) await sleep(300);
  rc(centre());
  log('at altitude', alt.toFixed(1), '- settling 6 s');
  await sleep(6000);
  const tEnd = Date.now() + minutes * 60000;
  if (mode === 'sequence') {
    log('SEQUENCE START');
    let reps = 0;
    while (Date.now() < tEnd) {
      for (const [axis, amp, hold, pause] of SEQUENCE) {
        const ch = centre(); ch[axis] = 1500 + amp; rc(ch); await sleep(hold);
        rc(centre()); await sleep(pause);
        if (alt < 7) { rc({ 1: 1500, 2: 1500, 3: 1650, 4: 1500 }); await sleep(1500); rc(centre()); }
        if (alt > 40) { rc({ 1: 1500, 2: 1500, 3: 1350, 4: 1500 }); await sleep(1500); rc(centre()); }
        if (Date.now() >= tEnd) break;
      }
      reps++;
      log('sequence pass', reps, 'done');
    }
    log('SEQUENCE END');
  } else {
    let seed = 3; const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
    while (Date.now() < tEnd) {
      const axis = rnd() < 0.5 ? 1 : 2; const dir = rnd() < 0.5 ? -1 : 1; const amp = 150 + Math.round(250 * rnd()); const hold = 150 + Math.round(250 * rnd());
      const ch = centre(); ch[axis] = 1500 + dir * amp; rc(ch); await sleep(hold);
      const back = centre(); back[axis] = 1500 - dir * Math.round(amp * 0.6); rc(back); await sleep(hold * 0.7);
      rc(centre()); await sleep(400 + 900 * rnd());
      if (alt < 8) { rc({ 1: 1500, 2: 1500, 3: 1650, 4: 1500 }); await sleep(2000); }
      if (alt > 40) { rc({ 1: 1500, 2: 1500, 3: 1350, 4: 1500 }); await sleep(2000); }
    }
  }
  rc(centre());
  await setMode(MODES.LAND);
  log('landing');
  const tl = Date.now(); while (armed && Date.now() - tl < 90000) await sleep(1000);
  log('disarmed:', !armed);
  await sleep(2000);
  process.exit(0);
})().catch(e => { console.error(e); process.exit(1); });
