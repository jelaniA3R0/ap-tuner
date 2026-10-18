// SITL test of the ground write (2026-10-17 audit R3): ArduCopter SITL, disarmed, through the app's own modules.
//   1 identity      AUTOPILOT_VERSION decoded (firmware, git hash, board UID), disarmed
//   2 write + undo  the Telemetry + USB write of four gains (+10 %): every value verified, read back fresh, the UNDO
//                   file saved with the vehicle in its header; Undo verified and read back
//   3 refusal       a set with a value ArduPilot refuses (ATC_RAT_YAW_FLTE is @READONLY in the defaults): the refusal is
//                   reported as refused and the rest of the set is rolled back (read back)
//   4 whole number  INS_GYRO_FILTER (INT16) 20.6 is sent as 21 and verified; then 20 again
//   5 reboot        the notch on (ENABLE, FREQ, BW, REF): REF takes effect only after a reboot; the app reboots the
//                   simulator (the connection closes and is reopened), proves the restart, checks the vehicle and reads
//                   every value again; Undo, reboot again, the values from before read back
// With --cpp the same phases run in C++ (cpp/tests/test_write_verify.cpp, wv_sitl_ground_write) on a fresh simulator.
// Usage: node tools/sitl-ground-write.js [--cpp]       (sitl/ArduCopter.exe; results in sitl/run-ground-write/)
'use strict';
const fs = require('fs');
const path = require('path');
const { spawn, spawnSync } = require('child_process');
const { Link } = require('../src/mavlink/link');
const { GroundTune } = require('../src/engine/groundtune');
const { Journal } = require('../src/engine/journal');
const { verifiedSet } = require('../src/engine/paramwrite');
const { identityText, sameF32 } = require('../src/engine/identity');

const ROOT = path.join(__dirname, '..'); const SITL = path.join(ROOT, 'sitl'); const RUN = path.join(SITL, 'run-ground-write');
const WITH_CPP = process.argv.includes('--cpp');
const sleep = ms => new Promise(r => setTimeout(r, ms));
const t00 = Date.now(); const log = (...a) => console.log(`${((Date.now() - t00) / 1000).toFixed(1).padStart(6)} s`, ...a);
const result = { started: new Date().toISOString(), js: [], cpp: null, pass: false };
const check = (ok, what) => { result.js.push({ ok: !!ok, what }); log(ok ? 'PASS' : 'FAIL', what); return ok; };
const fmt = v => (typeof v === 'number' ? +v.toPrecision(6) : v);

async function connect(link) {
  const t0 = Date.now(); let last = 0;
  while (!link.alive && Date.now() - t0 < 60000) {
    if (!link.connected && Date.now() - last > 2000) { last = Date.now(); link.connect({ kind: 'tcp', host: '127.0.0.1', port: 5760 }); }
    await sleep(200);
  }
  return link.alive;
}
async function read(link, name) { try { return await link.requestParam(name, 1500, 2); } catch (e) { return undefined; } }
/** the Telemetry + USB review as after a log download from this vehicle: the log holds the values it has now */
async function review(g, link, rows) {
  const id = link.identity();
  g.result = { family: 'copter', firmware: `ArduCopter V${String(id.fw).replace(/-.*$/, '')} (${id.gitHash})`, params: { MAV_SYSID: id.sysid }, axes: {}, chosenAxes: [] };
  g.rows = rows.map(r => Object.assign({ group: 'sitl', label: r.name }, r));
  await g._review();
  g.setPhase('review', 'SITL review');
  return { id: g.proposal.id, hash: g.proposal.hash };
}

async function phases(link, g) {
  // 1 identity
  await link.waitAutopilot(5000);
  const id = link.identity();
  log('vehicle:', identityText(id));
  check(id.fw && id.gitHash && id.uid, `AUTOPILOT_VERSION decoded: ArduPilot ${id.fw} (${id.gitHash}), board UID ${id.uid}`);
  if (!check(link.vehicle && !link.vehicle.armed, 'the simulator is disarmed')) return;

  // 2 the ground write of four gains, read back, the UNDO file, Undo
  const names = ['ATC_RAT_RLL_P', 'ATC_RAT_RLL_I', 'ATC_RAT_RLL_D', 'ATC_ANG_RLL_P'];
  const before = {}; for (const n of names) before[n] = await read(link, n);
  const rows = names.map(n => ({ name: n, from: before[n], value: +(before[n] * 1.1).toPrecision(6) }));
  let ap = await review(g, link, rows);
  check(g.rows.every(r => !r.blocked && !r.same), 'review: every row can be written (' + g.rows.map(r => `${r.name} ${fmt(r.vehicle)} -> ${r.value}`).join(', ') + ')');
  let res = await g.write(names, ap);
  check(res.every(w => w.status === 'verified'), 'write: every value verified (' + res.map(w => w.status).join(', ') + ')');
  let fresh = {}; for (const n of names) fresh[n] = await read(link, n);
  check(rows.every(r => sameF32(fresh[r.name], r.value)), 'read back fresh: the vehicle holds every new value');
  const undoText = g.files.undo && fs.existsSync(g.files.undo) ? fs.readFileSync(g.files.undo, 'utf8') : '';
  check(/# For: system id 1 \(quadrotor\), ArduPilot /.test(undoText) && names.every(n => undoText.includes(`\r\n${n},`)), 'UNDO file saved with the vehicle in its header and every value from before');
  await g.undo();
  check(g.writes.every(w => w.undo === 'undone'), 'Undo: every value written back and read back (' + g.writes.map(w => w.undo).join(', ') + ')');
  fresh = {}; for (const n of names) fresh[n] = await read(link, n);
  check(names.every(n => sameF32(fresh[n], before[n])), 'read back fresh: the vehicle holds the values from before again');

  // 3 a value ArduPilot refuses: reported as refused, the rest of the set rolled back
  const pitBefore = await read(link, 'ATC_RAT_PIT_P'), flte = await read(link, 'ATC_RAT_YAW_FLTE');
  ap = await review(g, link, [{ name: 'ATC_RAT_PIT_P', from: pitBefore, value: +(pitBefore * 1.1).toPrecision(6) }, { name: 'ATC_RAT_YAW_FLTE', from: flte, value: 3 }]);
  res = await g.write(['ATC_RAT_PIT_P', 'ATC_RAT_YAW_FLTE'], ap);
  check(res[0].status === 'rolled back' && res[1].status === 'refused', `refused write: ${res.map(w => `${w.name} ${w.status}`).join(', ')} (${res[1].text})`);
  check(sameF32(await read(link, 'ATC_RAT_PIT_P'), pitBefore) && sameF32(await read(link, 'ATC_RAT_YAW_FLTE'), flte), 'after the refusal the vehicle holds the values from before (read fresh)');
  check(/refused .*went back to its value from before/.test(g.info), 'the summary says so: ' + g.info);

  // 4 a whole-number parameter: the value is cast as ArduPilot stores it, then verified
  const gf = await read(link, 'INS_GYRO_FILTER');
  const c1 = await verifiedSet(link, 'INS_GYRO_FILTER', gf + 0.6);
  check(c1.status === 'verified' && c1.sent === gf + 1 && /rounded/.test(c1.note || ''), `INS_GYRO_FILTER ${gf + 0.6} sent as ${c1.sent} (type ${c1.type}) and verified`);
  const c2 = await verifiedSet(link, 'INS_GYRO_FILTER', gf);
  check(c2.status === 'verified' && sameF32(await read(link, 'INS_GYRO_FILTER'), gf), `INS_GYRO_FILTER back to ${gf} (verified, read back)`);

  // 5 the notch, a value that acts only after a reboot, the reboot workflow; Undo and the second reboot
  const nn = ['INS_HNTCH_ENABLE', 'INS_HNTCH_FREQ', 'INS_HNTCH_BW', 'INS_HNTCH_REF'];
  const nb = {}; for (const n of nn) nb[n] = await read(link, n);
  const target = { INS_HNTCH_ENABLE: 1, INS_HNTCH_FREQ: 60, INS_HNTCH_BW: 30, INS_HNTCH_REF: 0.3 };
  ap = await review(g, link, nn.map(n => ({ name: n, from: nb[n], value: target[n], group: 'notch' })));
  res = await g.write(nn, ap);
  check(res.every(w => w.status === 'verified'), 'notch written and read back (' + res.map(w => `${w.name} ${w.status}`).join(', ') + ')');
  check(g.rebootPending.map(x => x.name).join() === 'INS_HNTCH_REF', 'INS_HNTCH_REF waits for a reboot: ' + g.info);
  const gen0 = link.generation; const t0 = Date.now();
  const rb = await g.rebootNow();
  check(rb.ok && g.rebootPending.length === 0 && g.identity.gen === link.generation, `rebooted in ${((Date.now() - t0) / 1000).toFixed(1)} s, every value read back after the reboot (connection #${gen0} -> #${link.generation}): ${g.info}`);
  fresh = {}; for (const n of nn) fresh[n] = await read(link, n);
  check(nn.every(n => sameF32(fresh[n], target[n])), 'read fresh after the reboot: ' + nn.map(n => `${n} ${fmt(fresh[n])}`).join(', '));
  await g.undo();
  check(g.writes.every(w => w.undo === 'undone') && g.rebootPending.map(x => x.name).join() === 'INS_HNTCH_REF', 'notch Undo: written back and read back; REF waits for a reboot: ' + g.info);
  const rb2 = await g.rebootNow();
  fresh = {}; for (const n of nn) fresh[n] = await read(link, n);
  check(rb2.ok && nn.every(n => sameF32(fresh[n], nb[n])), 'second reboot: the notch values from before read back (' + nn.map(n => `${n} ${fmt(fresh[n])}`).join(', ') + ')');
  const kinds = new Set(fs.readFileSync(g.audit.file, 'utf8').split('\n').filter(Boolean).map(l => JSON.parse(l).kind));
  check(['identity', 'proposal', 'approval', 'txn', 'write', 'rollback', 'reboot'].every(k => kinds.has(k)), 'journal ' + path.basename(g.audit.file) + ': ' + [...kinds].join(', '));
}

/** a fresh simulator in a folder of its own (its own EEPROM: every edition starts from the same values) */
function startSitl(dir) {
  fs.mkdirSync(dir, { recursive: true });
  // ATC_RAT_YAW_FLTE is read-only for this run: ArduPilot refuses writes to it ("Param write denied")
  fs.writeFileSync(path.join(dir, 'defaults.parm'), fs.readFileSync(path.join(SITL, 'aptuner-sitl.parm'), 'utf8') + '\n# R3 SITL test: a value ArduPilot refuses\nATC_RAT_YAW_FLTE 2.5 @READONLY\n');
  const out = fs.openSync(path.join(dir, 'sitl.log'), 'a');
  return spawn(path.join(SITL, 'ArduCopter.exe'), ['--model', 'quad', '--speedup', '1', '--home', '-35.363261,149.165230,584,353', '--defaults', 'defaults.parm'], { cwd: dir, stdio: ['ignore', out, out], windowsHide: true });
}

(async () => {
  if (!fs.existsSync(path.join(SITL, 'ArduCopter.exe'))) { console.log('sitl/ArduCopter.exe not found (see sitl/run-sitl.cmd)'); process.exit(2); }
  fs.rmSync(RUN, { recursive: true, force: true }); fs.mkdirSync(RUN, { recursive: true });
  // JavaScript edition
  log('== JavaScript');
  let sitl = startSitl(path.join(RUN, 'sitl-js'));
  try {
    await sleep(1500);
    const link = new Link();
    if (check(await connect(link), 'connected to the simulator (TCP 5760)')) {
      const g = new GroundTune(link, { sessionsDir: path.join(RUN, 'js'), journal: new Journal(path.join(RUN, 'js', 'journal')) });
      try { await phases(link, g); } catch (e) { check(false, 'error: ' + e.stack); }
      link.disconnect();
    }
  } finally { sitl.kill(); await sleep(1000); }
  // C++ edition on a fresh simulator (after its reboots the simulator does not take a second client)
  if (WITH_CPP) {
    log('== C++ (aptuner-tests wv_sitl)');
    sitl = startSitl(path.join(RUN, 'sitl-cpp'));
    try {
      await sleep(1500);
      const exe = path.join(ROOT, 'cpp', 'build', 'release', 'tests', 'aptuner-tests.exe');
      const r = spawnSync(exe, ['wv_sitl'], { env: Object.assign({}, process.env, { APT_SITL_TCP: '5760', APT_SITL_DIR: path.join(RUN, 'cpp') }), encoding: 'utf8', timeout: 300000 });
      const text = (r.stdout || '') + (r.stderr || '');
      fs.writeFileSync(path.join(RUN, 'cpp-output.txt'), text);
      for (const line of text.split(/\r?\n/)) if (/PASS|FAIL|FAILED|ok |test\(s\)|vehicle:/.test(line)) log('  ' + line.trim());
      result.cpp = { exit: r.status, ok: r.status === 0 && /1 test\(s\), 0 failed/.test(text), checks: (text.match(/^ {4}PASS /gm) || []).length, failed: (text.match(/^ {4}FAIL /gm) || []).length };
    } finally { sitl.kill(); await sleep(500); }
  }
  result.pass = result.js.length > 0 && result.js.every(c => c.ok) && (!WITH_CPP || (result.cpp && result.cpp.ok));
  fs.writeFileSync(path.join(RUN, 'result.json'), JSON.stringify(result, null, 1));
  log(result.pass ? 'ALL PASS' : 'FAILED', `(${result.js.filter(c => c.ok).length}/${result.js.length} JavaScript checks${WITH_CPP ? `, C++ ${result.cpp ? `${result.cpp.checks}/${result.cpp.checks + result.cpp.failed} checks, ${result.cpp.ok ? 'pass' : 'FAIL'}` : 'FAIL'}` : ''})`);
  process.exit(result.pass ? 0 : 1);
})().catch(e => { console.error(e); process.exit(1); });
