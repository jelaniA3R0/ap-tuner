// In-flight safety of the live tuner (2026-10-17 audit), on a fake vehicle behind the real MAVLink link:
//   R0 / F-05 in-flight writing needs the opt-in        F-91 no samples before the family is known
//   F-02 continuous all-axis monitor                     F-01 notch change watched and reverted
//   F-26 the watch fails closed                          F-44 evidence-based revert, never above stepMax
//   F-31 / F-92 one write lock, cancel before every PARAM_SET (retries too), no Resume while writing
//   F-03 "oscillating", not "fly sharper"                F-06 attitude-error abort
// The same scenarios run in C++ (cpp/tests/test_live_safety.cpp). Usage: node test/live-safety.test.js [filter]
'use strict';
const assert = require('assert');
const fs = require('fs');
const os = require('os');
const path = require('path');
const { Link } = require('../src/mavlink/link');
const { Tuner, IN_FLIGHT_OFF } = require('../src/engine/tuner');
const { NotchLive } = require('../src/engine/notchlive');
const { FlightSession } = require('../src/engine/flight');
const { currentGains } = require('../src/engine/analysis');
const { FakeVehicle, copterParams } = require('./fakevehicle');

const SESS = fs.mkdtempSync(path.join(os.tmpdir(), 'aptuner-live-'));
const sleep = ms => new Promise(r => setTimeout(r, ms));
async function waitFor(pred, ms, what) { const t0 = Date.now(); while (!pred()) { if (Date.now() - t0 > ms) throw new Error('timeout waiting for ' + (what || 'condition')); await sleep(20); } return Date.now() - t0; }
// short watch times so the suite runs in about a minute (the defaults are 4 s / 10 s / 1 s)
const FAST = { watchSeconds: 0.6, watchGrace: 0.2, filterWatchSeconds: 1.0, revertStepSeconds: 0.1, calmWaitSeconds: 0.5 };

async function setup({ armed = true, inFlight = true, opts = {}, params = copterParams(), type = 2, prepare = true } = {}) {
  const link = new Link();
  const fv = new FakeVehicle({ armed, params, type }).attach(link);
  const tuner = new Tuner(link, Object.assign({ sessionDir: SESS, inFlightWrites: inFlight }, FAST, opts));
  tuner.startMonitor();
  await waitFor(() => link.alive, 3000, 'heartbeat');
  if (prepare) { await tuner.prepare({ confirm: true }); await sleep(1100); }   // fill the buffers (about 55 samples per axis)
  const done = async () => { tuner.stopMonitor(); await tuner.stop(); fv.stop(); };
  return { link, fv, tuner, done };
}
/** a proposal for an axis: the current gains with some keys changed */
function propose(tuner, axis, changes) {
  const cur = currentGains('copter', axis, tuner.params);
  tuner.results[axis] = { axis, at: Date.now(), proposal: { gains: Object.assign({}, cur, changes), changes: Object.keys(changes).map(k => ({ key: k, from: cur[k], to: changes[k], pct: 100 * (changes[k] / cur[k] - 1) })) } };
}
/** largest ratio between consecutive values written to the same parameter (and from its value before) */
function maxWriteRatio(fv, base) {
  const last = Object.assign({}, base); let worst = 1;
  for (const s of fv.sets) { const a = last[s.name], b = s.value; if (a > 0 && b > 0) worst = Math.max(worst, a > b ? a / b : b / a); last[s.name] = b; }
  return worst;
}

const tests = [];
const test = (name, fn) => tests.push({ name, fn });

// ---------------- R0: in-flight writing is off by default ----------------
test('r0_default_off', async () => {
  const link = new Link(); const t = new Tuner(link, { sessionDir: SESS });
  assert.strictEqual(t.opts.inFlightWrites, false, 'a fresh tuner does not write in flight');
});

test('r0_armed_without_optin_writes_nothing', async () => {
  const S = await setup({ armed: true, inFlight: false });
  try {
    const n0 = S.fv.sets.length;
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16 });
    await assert.rejects(S.tuner.applyProposal('roll'), e => e.message === IN_FLIGHT_OFF, 'Advanced Apply while armed');
    await assert.rejects(S.tuner.applyFilterChange({ INS_HNTCH_ENABLE: 1 }, 'test'), /changes in flight are turned off/, 'notch / filter write while armed');
    S.fv.escRpm = 6000; await sleep(600);
    const notch = new NotchLive(S.link, S.tuner);
    S.fv.escRpm = 6000; await sleep(400);
    await assert.rejects(notch.applyEsc(), /changes in flight are turned off/, 'live notch write while armed');
    await assert.rejects(S.tuner.restoreOriginals(), /changes in flight are turned off/, 'restore while armed');
    S.tuner.opts.autoApply = true;
    assert.throws(() => S.tuner.start(), /changes in flight are turned off/, 'auto-apply session while armed');
    S.tuner.opts.autoApply = false;
    // the guided flight: "Apply by itself" is refused, and at the review step nothing is written
    const flight = new FlightSession(S.link, S.tuner, notch);
    assert.throws(() => flight.start({ auto: true }), /Allow changes in flight/);
    flight.phase = 'review'; flight.checkup = { summary: { anyProposal: true, changes: 1 }, results: {} };
    flight.continue();
    assert.strictEqual(flight.phase, 'finished', 'review -> finished (check-up only)');
    await sleep(300);
    assert.strictEqual(S.fv.sets.length, n0, 'no PARAM_SET was sent: ' + JSON.stringify(S.fv.sets.slice(n0)));
  } finally { await S.done(); }
});

test('r0_housekeeping_while_armed_needs_optin', async () => {
  const p = copterParams(); p.GCS_PID_MASK = 0;
  const S = await setup({ armed: true, inFlight: false, params: p, prepare: false });
  try {
    await assert.rejects(S.tuner.prepare({ confirm: true }), /changes in flight are turned off/);
    assert.strictEqual(S.fv.sets.length, 0, 'GCS_PID_MASK not written while armed');
    assert.notStrictEqual(S.tuner.state, 'preparing', 'prepare does not hang in "preparing"');
  } finally { await S.done(); }
});

test('r0_on_the_ground_no_optin_needed', async () => {
  const S = await setup({ armed: false, inFlight: false });
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16 });
    const ok = await S.tuner.applyProposal('roll');
    assert.strictEqual(ok, true);
    assert.ok(Math.abs(S.fv.params.ATC_RAT_RLL_P - 0.16) < 1e-6, 'written on the ground');
  } finally { await S.done(); }
});

test('r0_optin_allows_inflight_apply', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16 });
    const ok = await S.tuner.applyProposal('roll');
    assert.strictEqual(ok, true, 'the change held: ' + S.tuner.stateInfo);
    assert.ok(Math.abs(S.fv.params.ATC_RAT_RLL_P - 0.16) < 1e-6);
    const ch = S.tuner.changes[S.tuner.changes.length - 1];
    assert.strictEqual(ch.status, 'held'); assert.strictEqual(ch.armed, true);
  } finally { await S.done(); }
});

// ---------------- F-91: nothing stored before the family is known ----------------
test('f91_plane_samples_before_prepare_not_stored', async () => {
  const S = await setup({ armed: true, type: 1, prepare: false });
  try {
    S.fv.desiredAmp = 30;   // a plane reports deg/s
    await sleep(600);
    assert.strictEqual(S.tuner.axes.roll.t.length, 0, 'PID_TUNING before prepare() is dropped');
    await S.tuner.prepare({ confirm: true });
    assert.strictEqual(S.tuner.family, 'plane');
    const t0 = Date.now() / 1000;
    await sleep(600);
    const ax = S.tuner.axes.roll;
    assert.ok(ax.t.length > 10, 'stored after prepare');
    assert.ok(ax.t[0] >= t0 - 1.5, 'only samples from after the family was known');
    const peak = Math.max(...ax.target.map(Math.abs));
    assert.ok(peak <= 31 && peak > 5, `plane rates kept in deg/s (peak ${peak.toFixed(1)}, not x57.3)`);
  } finally { await S.done(); }
});

test('f91_prepare_clears_buffers', async () => {
  const S = await setup({ armed: false });
  try {
    S.tuner.axes.roll.push(Date.now() / 1000 - 100, { desired: 1e4, achieved: 1e4, P: 0, I: 0, D: 0, FF: 0, SRate: 0.5, PDmod: 1 });
    await S.tuner.prepare({ confirm: true });
    assert.ok(!S.tuner.axes.roll.target.some(v => v === 1e4), 'old samples dropped by prepare()');
  } finally { await S.done(); }
});

// ---------------- F-02: the continuous monitor ----------------
test('f02_monitor_catches_non_tuned_axis_within_3s', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    S.tuner.opts.axes = ['roll']; S.tuner.start();
    await sleep(300);
    const t0 = Date.now(); S.fv.slew.yaw = 20;   // yaw is not being tuned
    const dt = await waitFor(() => S.tuner.state === 'paused', 3000, 'pause');
    assert.ok(dt < 3000, 'paused within 3 s (' + dt + ' ms)');
    assert.match(S.tuner.stateInfo, /yaw oscillating/);
    assert.ok(Date.now() - t0 < 3500);
    await waitFor(() => !S.tuner.writing, 3000, 'trip handling');
    assert.match(S.tuner.stateInfo, /nothing to go back to\. Land now/, S.tuner.stateInfo);
    assert.strictEqual(S.fv.sets.length, 0, 'nothing written: the app had changed nothing');
  } finally { await S.done(); }
});

test('f02_monitor_without_optin_only_tells_to_land', async () => {
  const S = await setup({ armed: true, inFlight: false });
  try {
    S.tuner.start(); await sleep(300);
    S.fv.slew.pitch = 20;
    await waitFor(() => S.tuner.state === 'paused' && !S.tuner.writing && /changed nothing in the air/.test(S.tuner.stateInfo), 3500, 'pause');
    assert.strictEqual(S.fv.sets.length, 0);
  } finally { await S.done(); }
});

test('f02_no_trip_on_calm_or_short_bursts', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    S.tuner.start(); await sleep(200);
    // a 0.3 s burst (a sharp stick tap) is not a sustained oscillation
    S.fv.slew.roll = 8; await sleep(300); S.fv.slew.roll = 0.5;
    await sleep(2500);
    assert.strictEqual(S.tuner.state, 'observing', 'still observing: ' + S.tuner.stateInfo);
  } finally { await S.done(); }
});

// ---------------- F-01: the in-flight notch change ----------------
test('f01_notch_change_reverted_and_paused_within_5s', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    const notch = new NotchLive(S.link, S.tuner);
    S.fv.escRpm = 6000; await sleep(500);
    S.tuner.start(); await sleep(200);
    let onset = null;
    // the airframe: 0.3 s after the notch comes on, pitch starts to oscillate; it stops when the notch is off again
    S.fv.onSet = (name, v) => { if (name !== 'INS_HNTCH_ENABLE') return; if (v > 0) setTimeout(() => { if (S.fv.params.INS_HNTCH_ENABLE > 0) { S.fv.slew.pitch = 25; S.fv.dmod.pitch = 0.75; onset = Date.now(); } }, 300); else { S.fv.slew.pitch = 0.5; S.fv.dmod.pitch = 1; } };
    const before = Object.assign({}, S.fv.params);
    await assert.rejects(notch.applyEsc(), e => e.code === 'reverted', 'applyEsc reports the revert');
    const on = S.fv.setsOf('INS_HNTCH_ENABLE');
    assert.strictEqual(on[0].value, 1); assert.strictEqual(on[1].value, 0, 'the notch was switched off again');
    assert.ok(onset && on[1].t - onset < 5000, `reverted ${on[1].t - onset} ms after the onset`);
    for (const k of ['INS_HNTCH_ENABLE', 'INS_HNTCH_MODE', 'INS_HNTCH_FREQ', 'INS_HNTCH_BW', 'INS_HNTCH_REF']) assert.strictEqual(S.fv.params[k], before[k], k + ' back to its value before');
    const revs = S.fv.sets.filter(s => s.t >= on[1].t);
    assert.strictEqual(revs[0].name, 'INS_HNTCH_ENABLE', 'the notch switch is the first corrective write');
    assert.strictEqual(S.tuner.state, 'paused');
    assert.match(S.tuner.stateInfo, /the change was reverted .* calm again/, S.tuner.stateInfo);
    assert.ok(S.tuner.messages.some(m => m.kind === 'tuner' && /^paused: .*oscillation detected, reverting/.test(m.text) && m.t - onset < 5000), 'paused within 5 s of the onset');
  } finally { await S.done(); }
});

// ---------------- F-26: the watch fails closed ----------------
test('f26_no_data_after_change_reverts', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16 });
    S.fv.onSet = name => { if (name === 'ATC_RAT_RLL_I' && S.fv.params.ATC_RAT_RLL_I > 0.15) { S.fv.streaming = false; setTimeout(() => { S.fv.streaming = true; }, 1600); } };
    const ok = await S.tuner.applyProposal('roll');
    assert.strictEqual(ok, false, 'the change is not kept without data');
    assert.ok(Math.abs(S.fv.params.ATC_RAT_RLL_P - 0.135) < 1e-6 && Math.abs(S.fv.params.ATC_RAT_RLL_I - 0.135) < 1e-6, 'reverted to the set before the change');
    assert.match(S.tuner.stateInfo, /no PID data/);
  } finally { await S.done(); }
});

test('f26_sparse_data_after_change_reverts', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16 });
    S.fv.onSet = name => { if (name === 'ATC_RAT_RLL_I' && S.fv.params.ATC_RAT_RLL_I > 0.15) S.fv.rateHz = 4; };
    const ok = await S.tuner.applyProposal('roll');
    assert.strictEqual(ok, false);
    assert.ok(Math.abs(S.fv.params.ATC_RAT_RLL_P - 0.135) < 1e-6, 'reverted');
    assert.match(S.tuner.stateInfo, /only \d+ PID samples after the change/);
  } finally { await S.done(); }
});

test('f26_link_lost_during_watch_fails_closed', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16 });
    S.fv.onSet = name => { if (name === 'ATC_RAT_RLL_I' && S.fv.params.ATC_RAT_RLL_I > 0.15) setTimeout(() => S.link.disconnect(), 300); };
    const ok = await S.tuner.applyProposal('roll');
    assert.strictEqual(ok, false);
    assert.strictEqual(S.tuner.state, 'paused');
    assert.ok(S.tuner.messages.some(m => /link to the vehicle was lost during the watch/.test(m.text)), 'the watch failed closed on the lost link');
    assert.match(S.tuner.stateInfo, /land now/, S.tuner.stateInfo);
  } finally { await S.done(); }
});

// ---------------- F-44: evidence-based revert ----------------
test('f44_calm_before_step_reverts_to_pre_step_set', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    propose(S.tuner, 'pitch', { P: 0.17, I: 0.17 });
    S.fv.onSet = (name, v) => { if (name === 'ATC_RAT_PIT_P') S.fv.slew.pitch = v > 0.15 ? 9 : 0.5; };
    const ok = await S.tuner.applyProposal('pitch');
    assert.strictEqual(ok, false);
    assert.ok(Math.abs(S.fv.params.ATC_RAT_PIT_P - 0.135) < 1e-6, 'back to the set before the step');
    assert.match(S.tuner.stateInfo, /the change was reverted .* calm again/, S.tuner.stateInfo);
  } finally { await S.done(); }
});

test('f44_lowering_step_reverted_upward_on_evidence', async () => {
  // the cd07d42 rule kept lowering after a trip; with the 2 s before the step calm, the step itself is undone
  const S = await setup({ armed: true, inFlight: true });
  try {
    propose(S.tuner, 'roll', { D: 0.0025 });
    S.fv.onSet = (name, v) => { if (name === 'ATC_RAT_RLL_D') S.fv.slew.roll = v < 0.003 ? 7 : 0.5; };
    const ok = await S.tuner.applyProposal('roll');
    assert.strictEqual(ok, false);
    assert.ok(Math.abs(S.fv.params.ATC_RAT_RLL_D - 0.0036) < 1e-7, 'D back up to 0.0036 (the calm set before the step)');
    assert.ok(maxWriteRatio(S.fv, copterParams()) <= S.tuner.opts.stepMax * (1 + 1e-6), 'no write above stepMax');
  } finally { await S.done(); }
});

test('f44_still_oscillating_restores_session_start_in_steps', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    S.tuner.beginSession();   // session start: D 0.0036
    // an earlier in-flight apply raised roll D to 0.0072 (held), then the vehicle starts oscillating and keeps on
    propose(S.tuner, 'roll', { D: 0.0072 });
    assert.strictEqual(await S.tuner.applyProposal('roll'), true, 'the raise held: ' + S.tuner.stateInfo);
    const n0 = S.fv.sets.length;
    S.tuner.resume(); await sleep(100);
    S.fv.slew.roll = 7;   // persistent oscillation, not caused by any write the monitor can blame
    S.fv.onSet = null;
    await waitFor(() => S.tuner.state === 'paused' && !S.tuner.writing, 8000, 'restore');
    const writes = S.fv.sets.slice(n0).filter(s => s.name === 'ATC_RAT_RLL_D');
    assert.ok(writes.length >= 2, 'restored in steps: ' + writes.map(w => w.value).join(' -> '));
    assert.ok(Math.abs(S.fv.params.ATC_RAT_RLL_D - 0.0036) < 1e-7, 'session start value restored');
    assert.ok(maxWriteRatio(S.fv, copterParams()) <= S.tuner.opts.stepMax * (1 + 1e-6), 'no write above stepMax');
    assert.match(S.tuner.stateInfo, /still oscillating with the values from the start of the session/, S.tuner.stateInfo);
  } finally { await S.done(); }
});

test('f44_revert_fails_then_session_start_restored', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    S.tuner.beginSession();
    propose(S.tuner, 'pitch', { P: 0.17, I: 0.17 });
    assert.strictEqual(await S.tuner.applyProposal('pitch'), true);
    S.tuner.resume();
    propose(S.tuner, 'pitch', { P: 0.21, I: 0.21 });
    // this step starts an oscillation that a revert of the step alone does not stop (only P <= 0.15 is calm)
    S.fv.onSet = (name, v) => { if (name === 'ATC_RAT_PIT_P') S.fv.slew.pitch = v > 0.15 ? 9 : 0.5; };
    const ok = await S.tuner.applyProposal('pitch');
    assert.strictEqual(ok, false);
    assert.ok(Math.abs(S.fv.params.ATC_RAT_PIT_P - 0.135) < 1e-6, 'escalated to the session start values');
    assert.ok(maxWriteRatio(S.fv, copterParams()) <= S.tuner.opts.stepMax * (1 + 1e-6), 'no write above stepMax');
    assert.match(S.tuner.stateInfo, /values from the start of the session were restored .* calm again/, S.tuner.stateInfo);
  } finally { await S.done(); }
});

// ---------------- F-31 / F-92: one write lock, cancellation ----------------
test('f92_second_apply_refused_and_resume_refused', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16 });
    propose(S.tuner, 'pitch', { P: 0.16, I: 0.16 });
    const a = S.tuner.applyProposal('roll');
    await sleep(50);
    await assert.rejects(S.tuner.applyProposal('pitch'), /another change is being written/);
    assert.throws(() => S.tuner.resume(), /still being written/);
    assert.throws(() => S.tuner.start(), /being written/);
    assert.strictEqual(await a, true);
    assert.strictEqual(S.fv.setsOf('ATC_RAT_PIT_P').length, 0, 'the second apply wrote nothing');
  } finally { await S.done(); }
});

test('f31_pause_stops_before_the_next_write', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    propose(S.tuner, 'roll', { P: 0.27, I: 0.27, D: 0.0072 });   // x2: two steps
    let paused = null;
    S.fv.onSet = name => { if (name === 'ATC_RAT_RLL_I' && !paused) setTimeout(() => { S.tuner.pause('paused by user'); paused = Date.now(); }, 250); };
    const ok = await S.tuner.applyProposal('roll');
    assert.strictEqual(ok, false, 'the apply stops');
    assert.ok(paused, 'paused during the watch of step 1');
    const after = S.fv.sets.filter(s => s.t > paused);
    assert.strictEqual(after.length, 0, 'no PARAM_SET after the pause: ' + JSON.stringify(after));
    assert.strictEqual(S.tuner.state, 'paused'); assert.strictEqual(S.tuner.stateInfo, 'paused by user');
    assert.ok(Math.abs(S.fv.params.ATC_RAT_RLL_P - 0.27) > 0.05, 'the last step was not written');
  } finally { await S.done(); }
});

test('f31_pause_cancels_param_set_retries', async () => {
  const S = await setup({ armed: true, inFlight: true });
  try {
    propose(S.tuner, 'roll', { P: 0.16, I: 0.16 });
    S.fv.ignoreSets = true;   // the vehicle does not answer: the link would retry every 1.5 s
    const a = S.tuner.applyProposal('roll');
    await sleep(300);
    S.tuner.pause('paused by user');
    const ok = await a;
    assert.strictEqual(ok, false);
    await sleep(2000);
    assert.strictEqual(S.fv.setsOf('ATC_RAT_RLL_P').length, 1, 'one PARAM_SET, no retry after the pause');
    assert.ok(S.tuner.journal.some(j => j.name === 'ATC_RAT_RLL_P' && /cancelled/.test(j.status || '')), 'the journal says the write was cancelled (and what was read back)');
  } finally { await S.done(); }
});

test('f31_single_loop_after_pause_resume', async () => {
  const S = await setup({ armed: true, inFlight: true, opts: { observeSeconds: 600 } });
  try {
    let calls = 0; const orig = S.tuner._observeStep.bind(S.tuner);
    S.tuner._observeStep = async () => { calls++; return orig(); };
    S.tuner.start(); await sleep(1200);
    S.tuner.pause(); S.tuner.resume(); S.tuner.pause(); S.tuner.resume();
    const c0 = calls; await sleep(3200);
    assert.ok(calls - c0 <= 4, `one observe loop (${calls - c0} calls in 3.2 s)`);
  } finally { await S.done(); }
});

// ---------------- F-03: the message names the oscillation ----------------
test('f03_low_coherence_with_high_srate_says_oscillating', async () => {
  const S = await setup({ armed: true, inFlight: true, opts: { observeSeconds: 3 } });
  try {
    S.fv.desiredAmp = 0;   // no stick input: no model can be fitted
    // every 8th sample above the limit: not sustained (the monitor stays quiet), but clearly not calm
    let k = 0; const iv = setInterval(() => { k++; S.fv.slew.roll = k % 8 === 0 ? 9 : 0.5; }, 20);
    S.tuner.opts.axes = ['roll']; S.tuner.start();
    await waitFor(() => S.tuner.state === 'paused', 8000, 'pause');
    clearInterval(iv);
    assert.match(S.tuner.stateInfo, /roll oscillating/, S.tuner.stateInfo);
    assert.ok(!/fly sharper/.test(S.tuner.stateInfo), 'never "fly sharper" while it oscillates');
  } finally { await S.done(); }
});

// ---------------- F-06: attitude-error abort ----------------
test('f06_attitude_error_during_watch_reverts', async () => {
  const S = await setup({ armed: true, inFlight: true, opts: { watchSeconds: 1.5 } });
  try {
    S.fv.attitude = { roll: 0, pitch: 0 }; S.fv.navTarget = { roll: 0, pitch: 0 };
    await sleep(500);
    propose(S.tuner, 'pitch', { P: 0.16, I: 0.16 });
    S.fv.onSet = (name, v) => { if (name === 'ATC_RAT_PIT_P') S.fv.attitude.pitch = v > 0.15 ? 15 : 0; };   // 15 deg off target, slew rate calm
    const ok = await S.tuner.applyProposal('pitch');
    assert.strictEqual(ok, false);
    assert.ok(S.tuner.messages.some(m => /attitude error 15\.0 deg/.test(m.text)), 'the attitude error tripped the watch');
    assert.ok(Math.abs(S.fv.params.ATC_RAT_PIT_P - 0.135) < 1e-6, 'reverted');
  } finally { await S.done(); }
});

// ---------------- the QuikTune-style ramp writes in flight too ----------------
test('ramp_needs_optin_while_armed', async () => {
  const S = await setup({ armed: true, inFlight: false });
  try {
    S.tuner.params.ATC_RAT_RLL_FLTD = 0;   // an unset D filter: the QuikTune filter rule has to write it first
    S.tuner._startRamp('roll');
    await assert.rejects(S.tuner._rampStep(), /changes in flight are turned off/);
    assert.strictEqual(S.fv.sets.length, 0);
  } finally { await S.done(); }
});

// ---------------- LiveAxis evidence ----------------
test('slewstats_missing_extension_fields_not_bad', async () => {
  const { LiveAxis } = require('../src/engine/axis');
  const ax = new LiveAxis('roll');
  for (let i = 0; i < 100; i++) ax.push(1000 + i / 50, { desired: 0, achieved: 0, P: 0, I: 0, D: 0, FF: 0, SRate: 0, PDmod: 0 });
  const s = ax.slewStats(2);
  assert.strictEqual(s.bad, 0, 'Dmod 0 = fields missing, not a trip');
  assert.strictEqual(s.noSlew, s.n);
  assert.ok(!ax.watchdog(2).oscillating, 'the step watchdog ignores missing Dmod too');
});

(async () => {
  const filter = process.argv[2];
  let failed = 0, run = 0;
  for (const t of tests) {
    if (filter && !t.name.includes(filter)) continue;
    run++;
    const t0 = Date.now();
    try { await Promise.race([t.fn(), sleep(30000).then(() => { throw new Error('test timeout'); })]); console.log(`  ok    ${t.name} (${Date.now() - t0} ms)`); }
    catch (e) { failed++; console.log(`  FAIL  ${t.name}: ${e.message}`); if (process.env.VERBOSE) console.log(e.stack); }
  }
  try { fs.rmSync(SESS, { recursive: true, force: true }); } catch (e) { /* temp */ }
  console.log(`${run} test(s), ${failed} failed`);
  process.exit(failed || !run ? 1 : 0);
})();
