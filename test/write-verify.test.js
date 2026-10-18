// Verified parameter writes bound to the right aircraft (2026-10-17 audit, roadmap R3), on fake vehicles behind the
// real MAVLink link:
//   F-25 session identity (connection, system id, board UID, firmware), a reconnect / another vehicle drops proposals
//   F-28 the target is pinned (ArduPilot only, no tracker / ground station), other vehicles are ignored, pending
//        requests are cleared on a reconnect, acknowledgements bound to the request and the requester
//   F-29 / F-30 exact verification by type (refused, clamped, rounded, dropped, stale, late, missing, no answer)
//   F-32 transactions: fresh read first, verified rollback of the whole set, uncertain parameters reported
//   F-33 approval bound to the proposal id + hash, full rows, IMAX written; stale baselines refused
//   F-35 values that act only after a reboot: never in flight; the reboot workflow on the ground, checked after it
//   F-39 the append-only journal; a write that cannot be journaled is not made
//   F-43 .param export checks, fixed point and the identity header
// The same scenarios run in C++ (cpp/tests/test_write_verify.cpp). Usage: node test/write-verify.test.js [filter]
'use strict';
const assert = require('assert');
const fs = require('fs');
const os = require('os');
const path = require('path');
const { Link } = require('../src/mavlink/link');
const { Encoder } = require('../src/mavlink/mavlink');
const { Tuner } = require('../src/engine/tuner');
const { NotchLive } = require('../src/engine/notchlive');
const { GroundTune } = require('../src/engine/groundtune');
const { Journal } = require('../src/engine/journal');
const { currentGains } = require('../src/engine/analysis');
const { verifiedSet, runTransaction, rebootAndVerify, txnText } = require('../src/engine/paramwrite');
const { paramFileChecked } = require('../src/engine/report');
const { makeProposal, paramHash, fnv1a64, f32hex, sessionIdentity, identityText } = require('../src/engine/identity');
const { formatParamValue, validateParam } = require('../src/model/parammeta');
const { FakeVehicle, copterParams } = require('./fakevehicle');

const SESS = fs.mkdtempSync(path.join(os.tmpdir(), 'aptuner-wv-'));
const sleep = ms => new Promise(r => setTimeout(r, ms));
async function waitFor(pred, ms, what) { const t0 = Date.now(); while (!pred()) { if (Date.now() - t0 > ms) throw new Error('timeout waiting for ' + (what || 'condition')); await sleep(20); } return Date.now() - t0; }
const FAST = { watchSeconds: 0.6, watchGrace: 0.2, filterWatchSeconds: 1.0, revertStepSeconds: 0.1, calmWaitSeconds: 0.5 };
const QUICK = { timeoutMs: 300, retries: 1, readRetries: 1 };   // short timeouts for the transaction tests
const f32 = Math.fround;
let nJournal = 0;
const newJournal = () => new Journal(path.join(SESS, 'journal-' + (++nJournal)));
/** the journal file as events */
const events = j => (j.file && fs.existsSync(j.file) ? fs.readFileSync(j.file, 'utf8').split('\n').filter(Boolean).map(l => JSON.parse(l)) : []);

async function setup({ armed = false, inFlight = false, params = copterParams(), prepare = true, fake = {}, opts = {} } = {}) {
  const link = new Link();
  const fv = new FakeVehicle(Object.assign({ armed, params }, fake)).attach(link);
  const journal = newJournal();
  const tuner = new Tuner(link, Object.assign({ sessionDir: SESS, inFlightWrites: inFlight, journal }, FAST, opts));
  tuner.startMonitor();
  await waitFor(() => link.alive, 3000, 'heartbeat');
  if (prepare) { await tuner.prepare({ confirm: true }); await sleep(armed ? 1100 : 50); }
  const done = async () => { tuner.stopMonitor(); await tuner.stop(); fv.stop(); };
  return { link, fv, tuner, journal, done };
}
/** a proposal for an axis: the current gains with some keys changed, bound like an analysis binds it */
function propose(tuner, axis, changes) {
  const cur = currentGains('copter', axis, tuner.params);
  tuner.results[axis] = { axis, at: Date.now(), proposal: { gains: Object.assign({}, cur, changes), changes: Object.keys(changes).map(k => ({ key: k, from: cur[k], to: changes[k], pct: 100 * (changes[k] / cur[k] - 1) })) } };
  return tuner.bindProposal(axis, tuner.results[axis].proposal);
}
function approve(tuner, axis) { const p = tuner.results[axis].proposal; return { id: p.id, hash: p.hash }; }
function maxWriteRatio(fv, base) {
  const last = Object.assign({}, base); let worst = 1;
  for (const s of fv.sets) { const a = last[s.name], b = s.value; if (a > 0 && b > 0) worst = Math.max(worst, a > b ? a / b : b / a); last[s.name] = b; }
  return worst;
}
/** the Telemetry + USB review, as after a log download: the log's values and the proposed rows */
async function review(g, rows, { logParams = {}, firmware = 'ArduCopter V4.5.7 (abcdef12)' } = {}) {
  const params = Object.assign({ SYSID_THISMAV: 1 }, logParams); for (const k of Object.keys(params)) if (params[k] === undefined) delete params[k];
  g.result = { family: 'copter', firmware, params, axes: {}, chosenAxes: [] };
  g.rows = rows.map(r => Object.assign({ group: 'roll', label: r.name }, r));
  await g._review();
  g.setPhase('review', 'test review');
  return { id: g.proposal.id, hash: g.proposal.hash };
}
const ROLL_ROWS = [{ name: 'ATC_RAT_RLL_P', value: 0.16, from: f32(0.135) }, { name: 'ATC_RAT_RLL_I', value: 0.16, from: f32(0.135) }, { name: 'ATC_RAT_RLL_D', value: 0.0045, from: f32(0.0036) }];
async function groundSetup(fake = {}) {
  const link = new Link();
  const fv = new FakeVehicle(Object.assign({ armed: false }, fake)).attach(link);
  const journal = newJournal();
  const g = new GroundTune(link, { sessionsDir: path.join(SESS, 'g' + nJournal), journal });
  await waitFor(() => link.alive, 3000, 'heartbeat');
  await link.waitAutopilot(2000);
  return { link, fv, g, journal, done: () => fv.stop() };
}

const tests = [];
const test = (name, fn) => tests.push({ name, fn });

// ---------------- F-28: the target is pinned to one ArduPilot vehicle ----------------
test('wv_pin_ignores_tracker_gcs_and_companion', async () => {
  const link = new Link(); const bus = [];
  const tracker = new FakeVehicle({ sysid: 2, type: 5, armed: false }).attach(link, bus);   // ArduPilot AntennaTracker
  link.feed(new Encoder(255, 190).pack('HEARTBEAT', { type: 6, autopilot: 8, base_mode: 0, custom_mode: 0, system_status: 4, mavlink_version: 3 }));   // a ground station
  link.feed(new Encoder(1, 191).pack('HEARTBEAT', { type: 18, autopilot: 8, base_mode: 0, custom_mode: 0, system_status: 4, mavlink_version: 3 }));   // a companion computer
  assert.strictEqual(link.pinned, false, 'nothing pinned before a vehicle answers');
  const fv = new FakeVehicle({ sysid: 3, armed: false }).attach(link, bus);
  try {
    await waitFor(() => link.alive, 2000, 'vehicle');
    assert.deepStrictEqual(link.target, { sysid: 3, compid: 1 });
    assert.strictEqual(link.vehicle.type, 2);
    await sleep(300);
    assert.ok(link.stats.foreign > 0, 'the tracker\'s messages are dropped');
    const v = await link.requestParam('ATC_RAT_RLL_P');
    assert.strictEqual(v, f32(0.135));
    assert.strictEqual(tracker.reads.length, 0, 'requests go to system id 3 only');
  } finally { tracker.silence(); fv.stop(); }
});

test('wv_second_vehicle_reported_not_followed', async () => {
  const link = new Link(); const bus = [];
  const a = new FakeVehicle({ sysid: 1, armed: false }).attach(link, bus);
  await waitFor(() => link.alive, 2000);
  const statuses = []; link.on('status', st => statuses.push(st));
  const b = new FakeVehicle({ sysid: 2, armed: false, uid: 'ff'.repeat(12) }).attach(link, bus);
  b.desiredAmp = 5; b.slew.roll = 9;
  const tuner = new Tuner(link, Object.assign({ sessionDir: SESS, journal: newJournal() }, FAST));
  try {
    await tuner.prepare({ confirm: true });
    await sleep(1200);
    assert.strictEqual(link.target.sysid, 1, 'still system id 1');
    assert.ok(link.othersSeen().some(o => o.sysid === 2), 'system id 2 is reported');
    assert.ok(statuses.some(st => /another vehicle is on this link \(system id 2, MAV_TYPE_QUADROTOR\): the app stays with system id 1/.test(st)), 'the user is told: ' + statuses.join(' | '));
    // a read of system id 1 is not answered by system id 2's announcement of the same name
    const p = link.requestParam('ATC_RAT_RLL_P'); b.foreignSet('ATC_RAT_RLL_P', 0.9);
    assert.strictEqual(await p, f32(0.135));
    assert.ok(Math.max(...tuner.axes.roll.target.map(Math.abs)) < 12, 'only system id 1\'s PID data in the buffers');
    assert.strictEqual(tuner.axes.roll.slewStats(1).srateMax <= 0.5, true, 'system id 2\'s slew rate never mixed in');
    assert.strictEqual(tuner.identity.uid, a.uid + '000000000000', 'identity of system id 1');
  } finally { tuner.stopMonitor(); await tuner.stop(); a.silence(); b.stop(); }
});

test('wv_target_silent_other_vehicle_is_a_new_connection', async () => {
  const link = new Link(); const bus = [];
  const a = new FakeVehicle({ sysid: 1, armed: false }).attach(link, bus);
  await waitFor(() => link.alive, 2000);
  const journal = newJournal();
  const tuner = new Tuner(link, Object.assign({ sessionDir: SESS, journal }, FAST));
  const b = new FakeVehicle({ sysid: 2, armed: false, uid: 'ee'.repeat(12) }).attach(link, bus);
  try {
    await tuner.prepare({ confirm: true });
    propose(tuner, 'roll', { P: 0.16 }); const ap = approve(tuner, 'roll');
    const gen0 = link.generation; let changed = null;
    link.on('vehicle-change', c => { changed = c; });
    a.heartbeats = false; a.streaming = false; a.sysTimeHz = 0;
    await waitFor(() => link.target.sysid === 2, 8000, 're-pin');
    assert.ok(changed && link.generation > gen0, 'a new generation');
    await sleep(100);
    assert.strictEqual(tuner.prepared, false);
    await assert.rejects(tuner.applyProposal('roll', ap), /no proposal|another connection/);
    assert.strictEqual(b.sets.length, 0, 'nothing written to the other vehicle');
    assert.ok(events(journal).some(e => e.kind === 'connection'), 'journaled');
  } finally { tuner.stopMonitor(); await tuner.stop(); a.silence(); b.stop(); }
});

test('wv_type_change_same_sysid_is_another_vehicle', async () => {
  const S = await setup();
  try {
    propose(S.tuner, 'roll', { P: 0.16 }); const ap = approve(S.tuner, 'roll');
    const gen0 = S.link.generation;
    S.fv.type = 13;   // the mirror now shows a hexacopter with the same system id
    await waitFor(() => S.link.generation > gen0, 2500, 'new generation');
    await sleep(50);
    await assert.rejects(S.tuner.applyProposal('roll', ap), /no proposal|another connection/);
    assert.strictEqual(S.fv.sets.length, 0);
  } finally { await S.done(); }
});

test('wv_reconnect_rejects_pending_write_no_retry_to_new_vehicle', async () => {
  const S = await setup();
  try {
    S.fv.echoDelayMs = 2000;   // the old vehicle's answer would come late
    const w = verifiedSet(S.link, 'ATC_RAT_RLL_P', 0.16);
    await sleep(300);
    const fv2 = new FakeVehicle({ armed: false, uid: 'ff'.repeat(12) }).attach(S.link);   // reconnect: another vehicle, same system id
    const r = await w;
    assert.strictEqual(r.status, 'uncertain', 'the old write cannot be proven: ' + JSON.stringify(r));
    assert.match(r.error, /connection (changed|was closed)/);
    await sleep(2500);
    assert.strictEqual(fv2.setsOf('ATC_RAT_RLL_P').length, 0, 'no retry went to the new vehicle');
    assert.strictEqual(S.fv.setsOf('ATC_RAT_RLL_P').length, 1);
    assert.notStrictEqual(S.link.params.has('ATC_RAT_RLL_P') ? S.link.params.get('ATC_RAT_RLL_P').value : undefined, f32(0.16), 'the late echo of the old connection is not taken');
    fv2.stop();
  } finally { S.fv.silence(); await S.done(); }
});

test('wv_different_vehicle_after_reconnect', async () => {
  const S = await setup();
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16 }); const ap = approve(S.tuner, 'roll');
    const id0 = S.tuner.identity;
    S.fv.silence();
    const fv2 = new FakeVehicle({ armed: false, uid: 'ab'.repeat(12) }).attach(S.link);
    await waitFor(() => S.link.alive && S.link.autopilot, 2000, 'vehicle B');
    await assert.rejects(S.tuner.applyProposal('roll', ap), /no proposal|another connection/);
    assert.strictEqual(fv2.sets.length, 0, 'the proposal of vehicle A is not written to vehicle B');
    await S.tuner.prepare({ confirm: true });
    assert.strictEqual(S.tuner.identity.uid, 'ab'.repeat(12) + '000000000000');
    assert.notStrictEqual(S.tuner.identity.key, id0.key);
    fv2.stop();
  } finally { await S.done(); }
});

test('wv_ack_for_another_ground_station_ignored', async () => {
  const S = await setup({ prepare: false });
  try {
    S.fv.ackForOther = true;
    await assert.rejects(S.link.commandAck(511, [30, 40000], 600), /no ack/);
    assert.ok(S.fv.commands.some(c => c.command === 511), 'the command was sent');
    S.fv.ackForOther = false;
    assert.strictEqual((await S.link.commandAck(511, [30, 40000], 600)).result, 0);
  } finally { await S.done(); }
});

test('wv_identity_from_autopilot_version', async () => {
  const S = await setup();
  try {
    const id = S.tuner.identity;
    assert.strictEqual(id.fw, '4.5.7'); assert.strictEqual(id.gitHash, 'abcdef12'); assert.strictEqual(id.uid, '0123456789abcdef01234567000000000000');
    assert.strictEqual(id.gen, S.link.generation); assert.strictEqual(id.sysid, 1); assert.strictEqual(id.autopilot, 3);
    assert.strictEqual(id.key, `g${S.link.generation}:1/1`);
    assert.strictEqual(id.paramHash, paramHash(S.tuner.paramsAtPrepare));
    assert.match(identityText(id), /^system id 1 \(quadrotor\), ArduPilot 4\.5\.7 \(abcdef12\), board UID 0123456789abcdef01234567, connection #\d+$/);
  } finally { await S.done(); }
});

// ---------------- F-29 / F-30: exact verification ----------------
test('wv_exact_float32_verified_and_journaled', async () => {
  const S = await setup();
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16 });
    assert.strictEqual(await S.tuner.applyProposal('roll', approve(S.tuner, 'roll')), true);
    assert.strictEqual(S.fv.params.ATC_RAT_RLL_P, f32(0.16));
    const w = events(S.journal).filter(e => e.kind === 'write' && e.name === 'ATC_RAT_RLL_P');
    assert.strictEqual(w.length, 1);
    assert.strictEqual(w[0].requested, 0.16); assert.strictEqual(w[0].sent, f32(0.16)); assert.strictEqual(w[0].echo, f32(0.16)); assert.strictEqual(w[0].status, 'verified');
    assert.strictEqual(w[0].identity, S.tuner.identity.key); assert.strictEqual(w[0].type, 9);
    const kinds = events(S.journal).map(e => e.kind);
    for (const k of ['identity', 'proposal', 'approval', 'txn', 'write']) assert.ok(kinds.includes(k), 'journal has ' + k);
  } finally { await S.done(); }
});

test('wv_refused_write_rolls_back_whole_apply', async () => {
  const S = await setup();
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16, D: 0.005 });
    S.fv.readOnly.add('ATC_RAT_RLL_I');
    await assert.rejects(S.tuner.applyProposal('roll', approve(S.tuner, 'roll')), e => e.code === 'apply-failed' && /refused 0\.16 and kept 0\.135/.test(e.message) && /Rolled back/.test(e.message));
    assert.strictEqual(S.fv.params.ATC_RAT_RLL_P, f32(0.135)); assert.strictEqual(S.fv.params.ATC_RAT_RLL_D, f32(0.0036)); assert.strictEqual(S.fv.params.ATC_RAT_RLL_I, f32(0.135));
    assert.strictEqual(S.tuner.state, 'paused');
    const end = events(S.journal).filter(e => e.kind === 'txn' && e.phase === 'end').pop();
    assert.strictEqual(end.rolledBack, true); assert.strictEqual(end.failed, 'ATC_RAT_RLL_I');
  } finally { await S.done(); }
});

test('wv_clamped_write_detected', async () => {
  const S = await setup();
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16 });
    S.fv.clamp.ATC_RAT_RLL_P = [0, 0.15];
    await assert.rejects(S.tuner.applyProposal('roll', approve(S.tuner, 'roll')), /asked for 0\.16, the autopilot holds 0\.15/);
    assert.strictEqual(S.fv.params.ATC_RAT_RLL_P, f32(0.135), 'rolled back');
    const w = events(S.journal).find(e => e.kind === 'write' && e.name === 'ATC_RAT_RLL_P' && e.status === 'different');
    assert.ok(w && w.readBack === f32(0.15), 'requested, echo and read-back journaled');
  } finally { await S.done(); }
});

test('wv_integer_parameters_cast_by_type', async () => {
  const S = await setup();
  try {
    const r1 = await verifiedSet(S.link, 'INS_HNTCH_MODE', 2.7);
    assert.strictEqual(r1.status, 'verified'); assert.strictEqual(r1.sent, 3); assert.strictEqual(r1.type, 2); assert.match(r1.note, /rounded to 3/);
    const r2 = await verifiedSet(S.link, 'INS_HNTCH_ENABLE', 300);
    assert.strictEqual(r2.status, 'invalid'); assert.strictEqual(S.fv.setsOf('INS_HNTCH_ENABLE').length, 0, 'nothing sent');
    // what the cast avoids: ArduPilot truncates a float sent to a whole-number parameter (2.7 -> 2)
    assert.strictEqual(await S.link.setParam('INS_HNTCH_MODE', 2.7), 2);
    const r3 = await verifiedSet(S.link, 'ATC_RAT_RLL_D', 0.0047);
    assert.strictEqual(r3.status, 'verified'); assert.strictEqual(r3.sent, f32(0.0047));
  } finally { await S.done(); }
});

test('wv_dropped_echoes_proven_by_read_back', async () => {
  const S = await setup();
  try {
    S.fv.dropEchoes = 2;
    const r = await verifiedSet(S.link, 'ATC_RAT_RLL_P', 0.16, QUICK);
    assert.strictEqual(r.status, 'verified'); assert.strictEqual(r.echo, undefined); assert.strictEqual(r.readBack, f32(0.16));
    assert.strictEqual(S.fv.setsOf('ATC_RAT_RLL_P').length, 2, 'sent twice (retry)');
  } finally { await S.done(); }
});

test('wv_lost_writes_not_applied_and_set_rolled_back', async () => {
  const S = await setup();
  try {
    S.fv.onSet = name => { if (name === 'ATC_RAT_RLL_P') S.fv.dropSets = 2; };
    const t = await runTransaction(S.link, [{ name: 'ATC_RAT_RLL_P', to: 0.16 }, { name: 'ATC_RAT_RLL_I', to: 0.16 }], QUICK);
    assert.strictEqual(t.ok, false); assert.strictEqual(t.rows[1].status, 'not-applied'); assert.strictEqual(t.rolledBack, true);
    assert.strictEqual(S.fv.params.ATC_RAT_RLL_P, f32(0.135)); assert.strictEqual(S.fv.params.ATC_RAT_RLL_I, f32(0.135));
    assert.match(txnText(t), /no answer to the write; the autopilot still holds 0\.135\. Everything else written in this set went back/);
  } finally { await S.done(); }
});

test('wv_stale_echo_then_read_back', async () => {
  const S = await setup();
  try {
    S.fv.staleEchoes = 1;   // a late answer to an earlier read arrives first, with the old value
    const r = await verifiedSet(S.link, 'ATC_RAT_RLL_P', 0.16);
    assert.strictEqual(r.status, 'verified'); assert.strictEqual(r.echo, f32(0.135)); assert.strictEqual(r.readBack, f32(0.16));
  } finally { await S.done(); }
});

test('wv_late_echo_verified', async () => {
  const S = await setup();
  try {
    S.fv.echoDelayMs = 2000;   // longer than the 1.5 s timeout: the link retries, the late echo resolves the write
    const r = await verifiedSet(S.link, 'ATC_RAT_RLL_P', 0.16);
    assert.strictEqual(r.status, 'verified'); assert.strictEqual(S.fv.setsOf('ATC_RAT_RLL_P').length, 2);
  } finally { await S.done(); }
});

test('wv_no_answer_uncertain_and_reported', async () => {
  const S = await setup();
  try {
    S.fv.onSet = name => { if (name === 'ATC_RAT_RLL_P') { S.fv.ignoreSets = true; S.fv.ignoreReads = true; } };   // the vehicle goes quiet after the first write
    const t = await runTransaction(S.link, [{ name: 'ATC_RAT_RLL_P', to: 0.16 }, { name: 'ATC_RAT_RLL_I', to: 0.16 }], QUICK);
    assert.strictEqual(t.ok, false); assert.strictEqual(t.rows[1].status, 'uncertain'); assert.strictEqual(t.rolledBack, false);
    assert.deepStrictEqual(t.uncertain.map(u => u.name).sort(), ['ATC_RAT_RLL_I', 'ATC_RAT_RLL_P']);
    assert.match(txnText(t), /Uncertain parameters: .*check them before flying/);
  } finally { await S.done(); }
});

test('wv_missing_parameter_stops_before_writing', async () => {
  const S = await setup();
  try {
    S.fv.missing.add('ATC_RAT_RLL_IMAX');
    const t = await runTransaction(S.link, [{ name: 'ATC_RAT_RLL_P', to: 0.16 }, { name: 'ATC_RAT_RLL_IMAX', to: 0.6 }], QUICK);
    assert.strictEqual(t.ok, false); assert.match(t.aborted, /ATC_RAT_RLL_IMAX does not exist on this autopilot/);
    assert.strictEqual(S.fv.sets.length, 0, 'nothing written');
  } finally { await S.done(); }
});

// ---------------- F-32: transactions in flight and failed rollbacks ----------------
test('wv_inflight_failure_rolls_back_within_stepmax_and_pauses', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    propose(S.tuner, 'pitch', { P: 0.27, I: 0.27 });   // x2: two steps
    S.fv.clamp.ATC_RAT_PIT_I = [0, 0.2];              // step 2: I 0.27 is stored as 0.2
    const stored = []; S.fv.onSet = (name, v) => stored.push({ name, value: v });
    await assert.rejects(S.tuner.applyProposal('pitch', approve(S.tuner, 'pitch')), e => e.code === 'apply-failed' && /Rolled back/.test(e.message));
    assert.strictEqual(S.fv.params.ATC_RAT_PIT_P, f32(0.135)); assert.strictEqual(S.fv.params.ATC_RAT_PIT_I, f32(0.135));
    assert.ok(maxWriteRatio({ sets: stored }, copterParams()) <= S.tuner.opts.stepMax * (1 + 1e-6), 'no change of a stored value above stepMax: ' + JSON.stringify(stored));
    assert.strictEqual(S.tuner.state, 'paused');
  } finally { await S.done(); }
});

test('wv_failed_rollback_lists_uncertain_parameters', async () => {
  const S = await setup();
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16, D: 0.005 });
    S.fv.onSet = name => { if (name === 'ATC_RAT_RLL_D') S.fv.readOnly.add('ATC_RAT_RLL_D'); };   // D cannot be written back
    S.fv.clamp.ATC_RAT_RLL_I = [0, 0.15];
    await assert.rejects(S.tuner.applyProposal('roll', approve(S.tuner, 'roll')), e => /uncertain parameters: ATC_RAT_RLL_D: the autopilot refused 0\.0036 and kept 0\.005/.test(e.message) && /check them before flying/.test(e.message));
    assert.strictEqual(S.fv.params.ATC_RAT_RLL_P, f32(0.135)); assert.strictEqual(S.fv.params.ATC_RAT_RLL_I, f32(0.135));
    const end = events(S.journal).filter(e => e.kind === 'txn' && e.phase === 'end').pop();
    assert.strictEqual(end.rolledBack, false); assert.strictEqual(end.uncertain.length, 1);
  } finally { await S.done(); }
});

// ---------------- F-33: the approval names the proposal ----------------
test('wv_approval_must_match_proposal', async () => {
  const S = await setup();
  try {
    propose(S.tuner, 'roll', { P: 0.16 }); const old = approve(S.tuner, 'roll');
    propose(S.tuner, 'roll', { P: 0.17 });   // a newer analysis replaced it
    await assert.rejects(S.tuner.applyProposal('roll', old), e => e.code === 'approval');
    await assert.rejects(S.tuner.applyProposal('roll', { id: S.tuner.results.roll.proposal.id, hash: '0'.repeat(16) }), e => e.code === 'approval');
    await assert.rejects(S.tuner.applyProposal('roll'), e => e.code === 'approval');
    assert.strictEqual(S.fv.sets.length, 0, 'nothing written');
    assert.strictEqual(events(S.journal).filter(e => e.kind === 'refused').length, 3);
    assert.strictEqual(await S.tuner.applyProposal('roll', approve(S.tuner, 'roll')), true, 'the proposal shown now is applied');
    assert.strictEqual(S.fv.params.ATC_RAT_RLL_P, f32(0.17));
  } finally { await S.done(); }
});

test('wv_rows_full_names_reasons_and_hash', async () => {
  const S = await setup();
  try {
    const p = propose(S.tuner, 'roll', { P: 0.16, I: 0.16, IMAX: 0.7 });
    assert.deepStrictEqual(p.rows.map(r => r.name), ['ATC_RAT_RLL_IMAX', 'ATC_RAT_RLL_P', 'ATC_RAT_RLL_I']);
    assert.ok(p.rows.every(r => r.reason && r.reboot === false));
    assert.match(p.hash, /^[0-9a-f]{16}$/);
    const again = makeProposal({ source: 'live', axis: 'roll', identity: S.tuner.identity, rows: p.rows });
    assert.strictEqual(again.hash, p.hash, 'the hash depends only on the identity and the rows');
    assert.notStrictEqual(makeProposal({ source: 'live', axis: 'roll', identity: S.tuner.identity, rows: p.rows.map((r, i) => i ? r : Object.assign({}, r, { to: 0.71 })) }).hash, p.hash);
    assert.strictEqual(await S.tuner.applyProposal('roll', approve(S.tuner, 'roll')), true);
    assert.strictEqual(S.fv.params.ATC_RAT_RLL_IMAX, f32(0.7), 'IMAX is written like every value shown');
  } finally { await S.done(); }
});

test('wv_change_by_another_station_makes_proposal_stale', async () => {
  const S = await setup();
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16 });
    S.fv.foreignSet('ATC_RAT_RLL_I', 0.2);   // Mission Planner writes I after the analysis
    await sleep(50);
    assert.ok(S.tuner.messages.some(m => /ATC_RAT_RLL_I changed on the vehicle: 0\.135 -> 0\.2 \(not by AP Tuner/.test(m.text)), 'the change is reported');
    await assert.rejects(S.tuner.applyProposal('roll', approve(S.tuner, 'roll')), e => e.code === 'stale' && /ATC_RAT_RLL_I is 0\.2 on the vehicle, the proposal was made from 0\.135/.test(e.message));
    assert.strictEqual(S.fv.sets.length, 0, 'nothing written over the other station\'s value');
    assert.ok(events(S.journal).some(e => e.kind === 'conflict' && e.name === 'ATC_RAT_RLL_I'));
  } finally { await S.done(); }
});

test('wv_old_proposal_refused', async () => {
  const S = await setup();
  try {
    const p = propose(S.tuner, 'roll', { P: 0.16 });
    p.at -= 16 * 60000;
    await assert.rejects(S.tuner.applyProposal('roll', approve(S.tuner, 'roll')), e => e.code === 'stale' && /16 min old/.test(e.message));
    assert.strictEqual(S.fv.sets.length, 0);
  } finally { await S.done(); }
});

test('wv_undo_reads_fresh_and_reports', async () => {
  const S = await setup();
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16 });
    assert.strictEqual(await S.tuner.applyProposal('roll', approve(S.tuner, 'roll')), true);
    S.fv.readOnly.add('ATC_RAT_RLL_I');
    const n0 = S.fv.sets.length;
    await assert.rejects(S.tuner.undoAxis('roll'), /not written back: ATC_RAT_RLL_I: the autopilot refused 0\.135 and kept 0\.16/);
    assert.strictEqual(S.fv.params.ATC_RAT_RLL_P, f32(0.135), 'P went back');
    assert.ok(S.fv.sets.slice(n0).every(s => s.name === 'ATC_RAT_RLL_P' || s.name === 'ATC_RAT_RLL_I'), 'only the values that differ are written');
  } finally { await S.done(); }
});

// ---------------- F-35: values that act only after a reboot ----------------
test('wv_reboot_values_never_in_flight', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    await assert.rejects(S.tuner.applyFilterChange({ INS_HNTCH_REF: 1 }, 'test'), e => e.code === 'reboot-required');
    S.fv.escRpm = 6000; const notch = new NotchLive(S.link, S.tuner); await sleep(600);
    await assert.rejects(notch.applyEsc(), e => e.code === 'reboot-required' && /INS_HNTCH_REF/.test(e.message));
    await assert.rejects(notch.enableFft(), e => e.code === 'reboot-required');
    assert.strictEqual(S.fv.sets.length, 0, 'nothing written in flight');
  } finally { await S.done(); }
});

test('wv_reboot_workflow_on_the_ground_verified_after', async () => {
  const S = await setup();
  try {
    S.fv.escRpm = 6000; S.fv.rebootMs = 1500;
    const notch = new NotchLive(S.link, S.tuner); await sleep(600);
    const applied = await notch.applyEsc();
    assert.strictEqual(applied.INS_HNTCH_MODE, 3);
    assert.deepStrictEqual(S.tuner.rebootPending.map(p => p.name), ['INS_HNTCH_REF']);
    const st = await notch.status();
    assert.strictEqual(st.notchActive, false, 'not "active" before the reboot'); assert.deepStrictEqual(st.rebootPending, ['INS_HNTCH_REF']);
    const gen0 = S.link.generation;
    const r = await notch.reboot();
    assert.strictEqual(r.ok, true, r.reason); assert.strictEqual(S.fv.reboots, 1);
    assert.deepStrictEqual(r.rows.map(x => [x.name, x.ok]), [['INS_HNTCH_REF', true]]);
    assert.strictEqual(S.tuner.rebootPending.length, 0);
    assert.strictEqual(S.link.generation, gen0, 'the same connection (the fake keeps its transport)');
    assert.ok(S.link.boot.count >= 1, 'the reboot was seen in the boot clock');
    assert.strictEqual((await notch.status()).notchActive, true);
    const phases = events(S.journal).filter(e => e.kind === 'reboot').map(e => e.phase);
    assert.deepStrictEqual(phases, ['request', 'verified']);
  } finally { await S.done(); }
});

test('wv_reboot_refused_while_armed', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    const notch = new NotchLive(S.link, S.tuner);
    await assert.rejects(notch.reboot(), /refusing to reboot an armed vehicle/);
    assert.ok(!S.fv.commands.some(c => c.command === 246), 'no reboot command sent');
  } finally { await S.done(); }
});

test('wv_other_board_after_reboot_detected', async () => {
  const S = await setup();
  try {
    S.fv.rebootMs = 1500; S.fv.onReboot = () => { S.fv.uid = '55'.repeat(12); };
    const gen0 = S.link.generation;
    const r = await rebootAndVerify(S.link, [{ name: 'ATC_RAT_RLL_P', value: 0.135 }], { timeoutMs: 15000 });
    assert.strictEqual(r.ok, false); assert.match(r.reason, /a different vehicle answered after the reboot/);
    assert.ok(S.link.generation > gen0, 'treated as another vehicle');
    await sleep(1100);
    assert.strictEqual(S.tuner.prepared, false, 'the session of the old board ended');
  } finally { await S.done(); }
});

// ---------------- Telemetry + USB: the ground write ----------------
test('wv_ground_write_verified_undo_file_and_undo', async () => {
  const G = await groundSetup();
  try {
    const ap = await review(G.g, ROLL_ROWS, { logParams: { ATC_RAT_RLL_P: f32(0.135) } });
    assert.ok(G.g.rows.every(r => !r.blocked && r.vehicle !== undefined), JSON.stringify(G.g.rows));
    const res = await G.g.write(['ATC_RAT_RLL_P', 'ATC_RAT_RLL_I', 'ATC_RAT_RLL_D'], ap);
    assert.deepStrictEqual(res.map(r => r.status), ['verified', 'verified', 'verified']);
    assert.strictEqual(G.fv.params.ATC_RAT_RLL_D, f32(0.0045));
    const undo = fs.readFileSync(G.g.files.undo, 'utf8');
    assert.match(undo, /# For: system id 1 \(quadrotor\), ArduPilot 4\.5\.7 \(abcdef12\), board UID 0123456789abcdef01234567, connection #\d+\. Load it only on this vehicle\./);
    assert.match(undo, /\r\nATC_RAT_RLL_P,0\.135\r\n/); assert.match(undo, /\r\nATC_RAT_RLL_D,0\.0036\r\n/);
    assert.match(fs.readFileSync(G.g.files.param, 'utf8'), /ATC_RAT_RLL_D,0\.0045/);
    assert.match(G.g.info, /all 3 written and read back/);
    await G.g.undo();
    assert.deepStrictEqual(G.g.writes.map(w => w.undo), ['undone', 'undone', 'undone']);
    assert.strictEqual(G.fv.params.ATC_RAT_RLL_P, f32(0.135)); assert.match(G.g.info, /previous values written back and read back/);
  } finally { G.done(); }
});

test('wv_ground_refused_row_others_rolled_back_honest_status', async () => {
  const G = await groundSetup();
  try {
    const ap = await review(G.g, ROLL_ROWS);
    G.fv.readOnly.add('ATC_RAT_RLL_I');
    const res = await G.g.write(ROLL_ROWS.map(r => r.name), ap);
    assert.deepStrictEqual(res.map(r => r.status), ['rolled back', 'refused', 'not written']);
    assert.ok(!res.some(r => r.ok), 'no row claims success');
    assert.match(res[1].text, /refused 0\.16 and kept 0\.135/);
    assert.match(G.g.info, /refused .*went back to its value from before/);
    assert.strictEqual(G.fv.params.ATC_RAT_RLL_P, f32(0.135)); assert.strictEqual(G.fv.params.ATC_RAT_RLL_D, f32(0.0036));
    assert.ok(!G.g.files.param, 'no "written" file when nothing stayed written');
  } finally { G.done(); }
});

test('wv_ground_approval_and_connection_bound', async () => {
  const G = await groundSetup();
  try {
    const ap = await review(G.g, ROLL_ROWS);
    await assert.rejects(G.g.write(['ATC_RAT_RLL_P'], { id: ap.id, hash: 'x' }), /the list changed since it was shown/);
    G.fv.silence();
    const fv2 = new FakeVehicle({ armed: false }).attach(G.link);   // the USB cable was plugged again
    await waitFor(() => G.link.alive, 2000);
    await assert.rejects(G.g.write(['ATC_RAT_RLL_P'], ap), /connection changed since the review/);
    assert.strictEqual(fv2.sets.length + G.fv.sets.length, 0, 'nothing written');
    await G.g.recheck();
    const ap2 = { id: G.g.proposal.id, hash: G.g.proposal.hash };
    assert.notStrictEqual(ap2.id, ap.id);
    const res = await G.g.write(['ATC_RAT_RLL_P'], ap2);
    assert.strictEqual(res[0].status, 'verified'); assert.strictEqual(fv2.params.ATC_RAT_RLL_P, f32(0.16));
    fv2.stop();
  } finally { G.done(); }
});

test('wv_ground_value_changed_since_review_writes_nothing', async () => {
  const G = await groundSetup();
  try {
    const ap = await review(G.g, ROLL_ROWS);
    G.fv.foreignSet('ATC_RAT_RLL_I', 0.14);
    const res = await G.g.write(ROLL_ROWS.map(r => r.name), ap);
    assert.ok(res.every(r => r.status === 'not written'));
    assert.match(G.g.info, /nothing written: changed on the autopilot since the values were shown: ATC_RAT_RLL_I is 0\.14, was 0\.135/);
    assert.strictEqual(G.fv.sets.length, 0); assert.ok(!G.g.files.undo, 'not even the UNDO file');
  } finally { G.done(); }
});

test('wv_ground_log_of_another_vehicle_or_firmware_blocked', async () => {
  const G = await groundSetup();
  try {
    let ap = await review(G.g, ROLL_ROWS, { logParams: { SYSID_THISMAV: undefined, MAV_SYSID: 7 } });   // ArduPilot 4.7 name
    assert.ok(G.g.rows.every(r => /the log is from system id 7, the autopilot on this connection is system id 1/.test(r.blocked)));
    await assert.rejects(G.g.write(['ATC_RAT_RLL_P'], ap), /cannot be written/);
    ap = await review(G.g, ROLL_ROWS, { firmware: 'ArduCopter V4.5.6 (abcdef12)' });
    assert.ok(G.g.rows.every(r => /recorded with ArduPilot 4\.5\.6, the autopilot now runs 4\.5\.7/.test(r.blocked)));
    ap = await review(G.g, [{ name: 'ATC_RAT_RLL_P', value: 0.16, from: 0.12 }]);
    assert.match(G.g.rows[0].blocked, /the autopilot holds 0\.135, the log had 0\.12/);
    assert.strictEqual(G.fv.sets.length, 0);
  } finally { G.done(); }
});

test('wv_ground_undo_failure_reported', async () => {
  const G = await groundSetup();
  try {
    const ap = await review(G.g, ROLL_ROWS);
    await G.g.write(ROLL_ROWS.map(r => r.name), ap);
    G.fv.readOnly.add('ATC_RAT_RLL_P');
    await G.g.undo();
    assert.deepStrictEqual(G.g.writes.map(w => w.undo), ['refused', 'undone', 'undone']);
    assert.match(G.g.info, /undo incomplete: ATC_RAT_RLL_P: the autopilot refused 0\.135 and kept 0\.16\. Load the UNDO file in Mission Planner/);
  } finally { G.done(); }
});

test('wv_ground_notch_reboot_workflow_with_values_after_reboot', async () => {
  const p = copterParams();
  const G = await groundSetup({ params: p });
  try {
    G.fv.rebootMs = 1500;
    for (const k of ['INS_HNTCH_FREQ', 'INS_HNTCH_BW', 'INS_HNTCH_REF', 'INS_HNTCH_HMNCS']) G.fv.hidden.add(k);   // appear once the notch is on
    const rows = [{ group: 'notch', name: 'INS_HNTCH_ENABLE', value: 1, from: 0 }, { group: 'notch', name: 'INS_HNTCH_FREQ', value: 55 }, { group: 'notch', name: 'INS_HNTCH_BW', value: 27.5 },
      { group: 'notch', name: 'INS_HNTCH_REF', value: 0.35 }, { group: 'notch', name: 'INS_HNTCH_HMNCS', value: 3 }];
    const ap = await review(G.g, rows);
    assert.deepStrictEqual(G.g.rows.map(r => r.deferred), [false, true, true, true, true]);
    await G.g.write(rows.map(r => r.name), ap);
    assert.deepStrictEqual(G.g.writes.map(w => w.status), ['verified', 'after reboot', 'after reboot', 'after reboot', 'after reboot']);
    assert.match(G.g.info, /take effect only after a reboot: press Reboot/);
    await G.g.rebootNow();
    assert.strictEqual(G.fv.reboots, 1);
    assert.deepStrictEqual(G.g.writes.map(w => w.status), ['verified', 'verified', 'verified', 'verified', 'unchanged'], 'HMNCS was 3 already');
    assert.strictEqual(G.fv.params.INS_HNTCH_FREQ, f32(55)); assert.strictEqual(G.fv.params.INS_HNTCH_REF, f32(0.35));
    assert.deepStrictEqual(G.g.rebootPending.map(x => x.name), ['INS_HNTCH_REF'], 'REF changed and needs one more reboot');
    assert.match(G.g.info, /need one more reboot/);
    await G.g.rebootNow();
    assert.strictEqual(G.fv.reboots, 2); assert.strictEqual(G.g.rebootPending.length, 0);
    assert.match(G.g.info, /rebooted: every value written here was read back after the reboot/);
    assert.ok(G.g.writes.every(w => /read back after the reboot/.test(w.afterReboot || 'read back after the reboot')));
    // Undo: REF waits for a reboot again; the reboot checks the values from before (not the values written)
    const before = Object.fromEntries(G.g.writes.map(w => [w.name, w.from]));
    await G.g.undo();
    assert.deepStrictEqual(G.g.writes.map(w => w.undo), ['undone', 'undone', 'undone', 'undone', null], 'HMNCS did not change');
    assert.deepStrictEqual(G.g.rebootPending.map(x => x.name), ['INS_HNTCH_REF']);
    await G.g.rebootNow();
    assert.strictEqual(G.fv.reboots, 3); assert.strictEqual(G.g.rebootPending.length, 0);
    assert.match(G.g.info, /rebooted: every value written here was read back after the reboot/);
    assert.ok(G.g.reboot.ok && G.g.reboot.rows.every(x => x.ok && x.holds === before[x.name]), JSON.stringify(G.g.reboot.rows));
    assert.ok(G.g.writes.every(w => /read back after the reboot/.test(w.afterReboot || 'read back after the reboot')));
  } finally { G.done(); }
});

// ---------------- F-43: the exported file ----------------
test('wv_param_file_checks_format_and_identity', async () => {
  const id = { sysid: 1, compid: 1, typeName: 'MAV_TYPE_QUADROTOR', fw: '4.5.7', gitHash: 'abcdef12', uid: '0123456789abcdef01234567000000000000', gen: 3 };
  const rows = [{ group: 'roll', name: 'ATC_RAT_RLL_P', value: 0.16, from: f32(0.135) }, { group: 'roll', name: 'ATC_RAT_RLL_D', value: 4.2e-7 }, { group: 'roll', name: 'ATC_RAT_RLL_I', value: NaN },
    { group: 'notch', name: 'INS_HNTCH_OPTS', value: 64 }, { group: 'notch', name: 'INS_HNTCH_MODE', value: 2.5 }, { group: 'notch', name: 'INS_HNTCH_REF', value: 2 }, { group: 'notch', name: 'INS_HNTCH_HMNCS', value: 7 },
    { group: 'pitch', name: 'PTCH2SRV_TCONST', value: 0.15 }, { group: 'x', name: 'SOMETHING_ELSE', value: 1 }];
  const out = paramFileChecked(rows, ['test'], { identity: id });
  const lines = out.text.split('\r\n');
  assert.ok(out.text.endsWith('\r\n') && !/[^\r]\n/.test(out.text), 'CRLF line ends');
  assert.strictEqual(lines[1], '# For: system id 1 (quadrotor), ArduPilot 4.5.7 (abcdef12), board UID 0123456789abcdef01234567, connection #3. Load it only on this vehicle.');
  const values = lines.filter(l => /^[A-Z]/.test(l));
  assert.deepStrictEqual(values, ['ATC_RAT_RLL_P,0.16', 'ATC_RAT_RLL_D,0.00000042', 'INS_HNTCH_REF,2', 'INS_HNTCH_HMNCS,7', 'PTCH2SRV_TCONST,0.15']);
  assert.ok(!values.some(l => /e[+-]?\d/i.test(l.split(',')[1])), 'no exponent notation');
  assert.deepStrictEqual(out.errors.map(e => e.name), ['ATC_RAT_RLL_I', 'INS_HNTCH_OPTS', 'INS_HNTCH_MODE', 'SOMETHING_ELSE']);
  assert.deepStrictEqual(out.warnings.map(w => w.name), ['INS_HNTCH_REF', 'PTCH2SRV_TCONST']);
  assert.ok(lines.includes('# INS_HNTCH_REF takes effect only after a reboot of the autopilot'));
  assert.ok(lines.includes('# ATC_RAT_RLL_P was 0.135'));
});

test('wv_formats_and_hashes_are_fixed', async () => {
  // fixed values the C++ edition must produce bit for bit (cpp/tests/test_write_verify.cpp)
  assert.deepStrictEqual([0.16, 0.0036, 4.2e-7, 12345.678, 0.1, 1, 0, 1e-12, 123456789, 0.000123456].map(v => formatParamValue('ATC_RAT_RLL_D', v)),
    ['0.16', '0.0036', '0.00000042', '12345.678', '0.1', '1', '0', '0.000000000001', '123456792', '0.000123456']);
  assert.strictEqual(formatParamValue('INS_HNTCH_MODE', 3), '3');
  assert.strictEqual(f32hex(0.135), '3e0a3d71');
  assert.strictEqual(fnv1a64('AP Tuner'), 'ff9b411acb97fff8'); assert.strictEqual(fnv1a64(''), 'cbf29ce484222325'); assert.strictEqual(fnv1a64('a'), 'af63dc4c8601ec8c');
  assert.strictEqual(paramHash({ ATC_RAT_RLL_P: 0.135, ATC_RAT_RLL_I: 0.135, X: 'text' }), '0d6e8b666e1b99bc');
  const id = sessionIdentity({ gen: 3, sysid: 1, compid: 1, autopilot: 3, type: 2, typeName: 'MAV_TYPE_QUADROTOR', uid: 'aa', fw: '4.5.7', gitHash: 'abcdef12' }, { A: 1 });
  const p = makeProposal({ source: 'live', axis: 'roll', identity: id, rows: [{ name: 'ATC_RAT_RLL_P', from: 0.135, to: 0.16 }, { name: 'ATC_RAT_RLL_IMAX', to: 0.7 }] });
  assert.strictEqual(p.hash, '51b120c7020a222e');
  assert.deepStrictEqual(validateParam('INS_HNTCH_OPTS', 16), { ok: true, errors: [], warnings: [], reboot: true, int: true });
});

// ---------------- F-39: the journal ----------------
test('wv_journal_is_append_only', async () => {
  const S = await setup();
  try {
    propose(S.tuner, 'roll', { P: 0.16 });
    await S.tuner.applyProposal('roll', approve(S.tuner, 'roll'));
    S.tuner.setState('ready');
    const first = fs.readFileSync(S.journal.file, 'utf8');
    propose(S.tuner, 'roll', { P: 0.17 });
    await S.tuner.applyProposal('roll', approve(S.tuner, 'roll'));
    const second = fs.readFileSync(S.journal.file, 'utf8');
    assert.ok(second.startsWith(first) && second.length > first.length, 'only appended');
    for (const e of events(S.journal)) assert.ok(typeof e.t === 'number' && e.kind, 'every line has a time and a kind');
    const w = events(S.journal).filter(e => e.kind === 'write');
    assert.ok(w.every(e => 'requested' in e && 'sent' in e && 'echo' in e && 'status' in e && 'identity' in e && 'gen' in e));
    const appr = events(S.journal).filter(e => e.kind === 'approval');
    assert.strictEqual(appr.length, 2); assert.ok(appr.every(a => a.by === 'user' && a.proposal && a.hash));
  } finally { await S.done(); }
});

test('wv_unwritable_journal_blocks_forward_writes', async () => {
  const blocker = path.join(SESS, 'not-a-dir-' + Date.now());
  fs.writeFileSync(blocker, 'x');   // the journal folder cannot be created: a file is in the way
  const link = new Link(); const fv = new FakeVehicle({ armed: false }).attach(link);
  const tuner = new Tuner(link, Object.assign({ sessionDir: SESS, journal: new Journal(path.join(blocker, 'journal')) }, FAST));
  try {
    await waitFor(() => link.alive, 2000);
    await tuner.prepare({ confirm: true });
    assert.strictEqual(tuner.audit.ok, false);
    propose(tuner, 'roll', { P: 0.16 });
    await assert.rejects(tuner.applyProposal('roll', approve(tuner, 'roll')), /the write journal cannot be written/);
    assert.strictEqual(fv.sets.length, 0, 'nothing written');
    assert.ok(tuner.snapshot().audit.lost > 0, 'the page can show it');
  } finally { await tuner.stop(); fv.stop(); }
});

(async () => {
  const filter = process.argv[2];
  let failed = 0, run = 0;
  for (const t of tests) {
    if (filter && !t.name.includes(filter)) continue;
    run++;
    const t0 = Date.now();
    try { await Promise.race([t.fn(), sleep(40000).then(() => { throw new Error('test timeout'); })]); console.log(`  ok    ${t.name} (${Date.now() - t0} ms)`); }
    catch (e) { failed++; console.log(`  FAIL  ${t.name}: ${e.message}`); if (process.env.VERBOSE) console.log(e.stack); }
  }
  try { fs.rmSync(SESS, { recursive: true, force: true }); } catch (e) { /* temp */ }
  console.log(`${run} test(s), ${failed} failed`);
  process.exit(failed || !run ? 1 : 0);
})();
