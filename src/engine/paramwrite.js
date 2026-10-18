// Verified parameter writes (2026-10-17 audit F-28, F-29, F-30, F-32, F-35, F-51). Every value is cast to the
// parameter's type (whole numbers for integer parameters, float32 otherwise), sent, and the echo compared exactly; a
// missing or different echo is followed by one more read, so the result says what the vehicle holds, not what was
// asked for. A set of writes is a transaction: read fresh, write in order, and on the first failure write everything
// back to its value from before, newest first, each verified; what cannot be proven is reported as uncertain. Values
// that take effect only after a reboot get a reboot on the ground, after which every value is read again.
// The same logic is in cpp/app/paramwrite.cpp.
'use strict';
const { sameF32, sameVehicle, identityText } = require('./identity');

const INT_RANGE = { 1: [0, 255], 2: [-128, 127], 3: [0, 65535], 4: [-32768, 32767], 5: [0, 4294967295], 6: [-2147483648, 2147483647] };
const TYPE_NAME = { 1: 'UINT8', 2: 'INT8', 3: 'UINT16', 4: 'INT16', 5: 'UINT32', 6: 'INT32', 7: 'UINT64', 8: 'INT64', 9: 'REAL32', 10: 'REAL64' };
// results after which the vehicle may hold something else than before (they need a rollback when the set fails)
const CHANGED = new Set(['verified', 'different', 'uncertain']);

function roundHalfAway(v) { return v < 0 ? -Math.floor(-v + 0.5) : Math.floor(v + 0.5); }
function num(v) { return typeof v === 'number' && Number.isFinite(v) ? +v.toPrecision(6) : v; }
function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }

/** the value to send for a parameter of this MAV_PARAM_TYPE: {ok, value, note} or {ok: false, error} */
function castForType(value, type) {
  if (typeof value !== 'number' || !Number.isFinite(value)) return { ok: false, error: 'the value is not a number' };
  const range = INT_RANGE[type];
  if (range) {
    const r = roundHalfAway(value);
    if (r < range[0] || r > range[1]) return { ok: false, error: `${num(value)} does not fit a ${TYPE_NAME[type]} parameter (${range[0]} to ${range[1]})` };
    return { ok: true, value: r, note: r !== value ? `rounded to ${r} (a whole-number parameter)` : null };
  }
  if (type === 7 || type === 8) return { ok: false, error: '64-bit integer parameters are not written by AP Tuner' };
  const f = Math.fround(value);
  if (!Number.isFinite(f)) return { ok: false, error: `${value} does not fit a float parameter` };
  return { ok: true, value: f, note: null };
}

/** one result in plain words */
function writeText(r) {
  switch (r.status) {
    case 'verified': return `${r.name} = ${num(r.holds)} (read back)`;
    case 'unchanged': return `${r.name} already ${num(r.holds)}`;
    case 'refused': return `${r.name}: the autopilot refused ${num(r.sent)} and kept ${num(r.holds)}`;
    case 'different': return `${r.name}: asked for ${num(r.sent)}, the autopilot holds ${num(r.holds)}`;
    case 'not-applied': return `${r.name}: no answer to the write; the autopilot still holds ${num(r.holds)}`;
    case 'uncertain': return `${r.name}: the value on the autopilot is not known (${r.error || 'no answer'})`;
    case 'missing': return `${r.name} does not exist on this autopilot`;
    case 'invalid': return `${r.name}: ${r.error}`;
    case 'cancelled': return `${r.name}: not written (stopped)`;
    case 'not-sent': return `${r.name}: not written (${r.error})`;
    default: return `${r.name}: ${r.status}`;
  }
}

/**
 * Write one parameter and prove what the vehicle holds afterwards. Never throws; resolves
 *   {name, requested, sent, type, before, echo, readBack, holds, status, ok, error, note, cancelled, attempts, ms}
 * status: 'verified' (holds the value sent), 'refused' (kept the value from before), 'different' (holds another value:
 * clamped, rounded or changed by someone else), 'not-applied' (no answer to the write and the read shows the value
 * from before), 'uncertain' (no answer at all, or the connection changed: not known), 'missing' (no such parameter),
 * 'invalid' (cannot be written, nothing sent), 'cancelled' (stopped before anything was sent), 'not-sent' (the
 * connection changed before anything was sent).
 */
async function verifiedSet(link, name, value, { before, timeoutMs = 1500, retries = 3, readRetries = 2, keepGoing = null } = {}) {
  const t0 = Date.now();
  const res = { name, requested: value, sent: undefined, type: undefined, before, echo: undefined, readBack: undefined, holds: undefined,
    status: 'uncertain', ok: false, error: null, note: null, cancelled: false, attempts: 0, ms: 0 };
  const finish = () => { res.ms = Date.now() - t0; res.ok = res.status === 'verified'; return res; };
  // the type comes from the vehicle's own answer: read the parameter first when the app has not seen it
  if (!link.params.get(name)) {
    try { await link.requestParam(name, timeoutMs, readRetries); } catch (e) {
      if (e.code === 'link-changed' || e.code === 'rebooted') { res.status = 'not-sent'; res.error = e.message; return finish(); }
      res.status = 'missing'; res.error = 'no answer when it was read'; return finish();
    }
  }
  const known = link.params.get(name);
  if (!known) { res.status = 'not-sent'; res.error = 'the connection changed'; return finish(); }
  res.type = known.type;
  if (res.before === undefined) res.before = known.value;
  const c = castForType(value, known.type);
  if (!c.ok) { res.status = 'invalid'; res.error = c.error; return finish(); }
  res.sent = c.value; res.note = c.note;
  let err = null;
  try { res.echo = await link.setParam(name, c.value, timeoutMs, retries, keepGoing); res.attempts = 1; } catch (e) { err = e; res.attempts = e.attempts || 0; }
  if (err && err.code === 'cancelled' && !err.attempts) { res.status = 'cancelled'; res.cancelled = true; res.error = 'stopped before it was sent'; return finish(); }
  if (err && (err.code === 'link-changed' || err.code === 'rebooted')) {
    // it may have reached the vehicle, but it cannot be read back on another connection or during a reboot
    res.status = err.attempts ? 'uncertain' : 'not-sent'; res.error = err.message; return finish();
  }
  if (err && err.code === 'cancelled') res.cancelled = true;
  if (!err && sameF32(res.echo, c.value)) { res.status = 'verified'; res.holds = res.echo; return finish(); }
  // no echo, or an echo with another value (a refusal, clamping, a late answer to something else): ask once more
  let readErr = null;
  try { res.readBack = await link.requestParam(name, timeoutMs, readRetries); } catch (e) { readErr = e; }
  if (res.readBack !== undefined) {
    res.holds = res.readBack;
    if (sameF32(res.readBack, c.value)) res.status = 'verified';
    else if (res.before !== undefined && sameF32(res.readBack, res.before)) res.status = err ? 'not-applied' : 'refused';
    else res.status = 'different';
    if (res.status !== 'verified' && !res.error) res.error = err ? err.message : null;
  } else {
    res.status = 'uncertain';
    res.error = (err ? err.message : `the echo showed ${num(res.echo)}`) + '; reading it back got no answer' + (readErr && readErr.code === 'link-changed' ? ' (the connection changed)' : '');
  }
  return finish();
}

let txnSeq = 0;
/**
 * A set of writes that either all hold or are written back (F-32): every value is read fresh first (its type and the
 * value before; a value that no longer matches `expect`, the value the change was designed from, stops the set before
 * anything is written), then written in order, each verified. The first failure stops the set and everything written
 * so far goes back to its value from before, newest first, each verified. Never throws; resolves
 *   {txn, ok, aborted, stale, cancelled, rows: [result], rollback: [result], rolledBack, uncertain: [{name, holds, why}]}
 * items: [{name, to, expect?, reason?}]; journal(kind, data); afterPreRead([{name, before}]) may throw to stop.
 */
async function runTransaction(link, items, { journal = null, keepGoing = null, afterPreRead = null, ctx = {} } = {}) {
  const txn = ctx.txn || ++txnSeq;
  const out = { txn, ok: false, aborted: null, stale: [], cancelled: false, rows: [], rollback: [], rolledBack: false, uncertain: [] };
  const J = (kind, data) => (journal ? journal(kind, Object.assign({ txn }, ctx, data)) : true);
  const gen0 = link.generation;
  // 1. a fresh read of every value
  const pre = [];
  for (const it of items) {
    let before, error = null;
    try { before = await link.requestParam(it.name, 1500, 2); } catch (e) { error = e; }
    pre.push({ it, before, error });
    if (error) break;
  }
  const bad = pre.find(p => p.before === undefined);
  if (bad) {
    out.aborted = bad.error && bad.error.code === 'timeout' ? `${bad.it.name} does not exist on this autopilot (no answer when it was read)` : `reading ${bad.it.name} failed: ${bad.error ? bad.error.message : 'no answer'}`;
    out.missing = bad.error && bad.error.code === 'timeout' ? bad.it.name : null;
    J('txn', { phase: 'aborted', reason: out.aborted }); return out;
  }
  out.stale = pre.filter(p => typeof p.it.expect === 'number' && !sameF32(p.before, p.it.expect)).map(p => ({ name: p.it.name, expect: p.it.expect, holds: p.before }));
  if (out.stale.length) {
    out.aborted = 'changed on the autopilot since the values were shown: ' + out.stale.map(s => `${s.name} is ${num(s.holds)}, was ${num(s.expect)}`).join('; ');
    J('txn', { phase: 'aborted', reason: out.aborted, stale: out.stale }); return out;
  }
  if (afterPreRead) {
    try { await afterPreRead(pre.map(p => ({ name: p.it.name, before: p.before }))); } catch (e) { out.aborted = e.message; J('txn', { phase: 'aborted', reason: out.aborted }); return out; }
  }
  J('txn', { phase: 'begin', items: items.map((it, i) => ({ name: it.name, to: it.to, before: pre[i].before, expect: it.expect, reason: it.reason })) });
  // 2. write in order, each verified; a value the vehicle already holds is not written again
  let failed = null;
  for (let i = 0; i < items.length; i++) {
    const it = items[i], before = pre[i].before;
    const known = link.params.get(it.name);
    const c = known ? castForType(it.to, known.type) : { ok: false };
    if (c.ok && sameF32(before, c.value)) {
      const r = { name: it.name, requested: it.to, sent: undefined, type: known.type, before, holds: before, status: 'unchanged', ok: true, error: null, note: null, reason: it.reason };
      out.rows.push(r); J('write', r); continue;
    }
    const r = await verifiedSet(link, it.name, it.to, { before, keepGoing });
    r.reason = it.reason;
    out.rows.push(r); J('write', r);
    if (r.cancelled && !CHANGED.has(r.status)) { out.cancelled = true; break; }
    if (!r.ok) { failed = r; break; }
    if (r.cancelled) { out.cancelled = true; break; }
  }
  if (out.cancelled) { J('txn', { phase: 'end', ok: false, cancelled: true }); return out; }
  if (!failed) { out.ok = true; J('txn', { phase: 'end', ok: true }); return out; }
  // 3. roll back what may have changed, newest first, each verified; never across another connection
  for (const r of out.rows.slice().reverse()) {
    if (!CHANGED.has(r.status) || (r.status === 'verified' && sameF32(r.holds, r.before))) continue;
    if (link.generation !== gen0) { out.uncertain.push({ name: r.name, holds: r.holds, why: 'the connection changed during the write: check it after reconnecting' }); continue; }
    const rb = await verifiedSet(link, r.name, r.before, { before: r.holds });
    rb.rollbackOf = r.status;
    out.rollback.push(rb); J('rollback', rb);
    if (!rb.ok) out.uncertain.push({ name: r.name, holds: rb.holds, why: writeText(rb) });
  }
  out.rolledBack = !out.uncertain.length;
  J('txn', { phase: 'end', ok: false, failed: failed.name, rolledBack: out.rolledBack, uncertain: out.uncertain });
  return out;
}

/** one sentence on a failed transaction: what failed, and whether the rest went back */
function txnText(t) {
  if (t.ok) return `${t.rows.length} value(s) written and read back`;
  if (t.aborted) return 'nothing written: ' + t.aborted;
  if (t.cancelled) return 'stopped: ' + t.rows.filter(r => r.status === 'verified').map(r => r.name).join(', ') + ' written before the stop';
  const failed = t.rows[t.rows.length - 1];
  const head = writeText(failed);
  if (t.rolledBack) return `${head}. Everything else written in this set went back to its value from before (read back)`;
  return `${head}. Uncertain parameters: ${t.uncertain.map(u => `${u.name} (${u.holds !== undefined ? 'holds ' + num(u.holds) : 'value not known'})`).join(', ')}: check them before flying`;
}

/**
 * The reboot workflow for values that take effect only after a reboot (F-35): only disarmed; MAV_CMD_PREFLIGHT_REBOOT
 * (246) must be acknowledged by the target; the autopilot has to go away (heartbeat gap, closed connection or its
 * boot clock going back) and come back (a closed connection is reopened: USB, TCP); it must be the same vehicle (ids,
 * frame type, board UID and firmware where known); then every expected value is read fresh and compared exactly.
 * expect: [{name, value}]. Never throws; resolves {ok, reason, rebooted, rows: [{name, expect, holds, ok}], identity}.
 */
async function rebootAndVerify(link, expect, { journal = null, ctx = {}, timeoutMs = 60000, onInfo = null } = {}) {
  const J = (kind, data) => (journal ? journal(kind, Object.assign({}, ctx, data)) : true);
  const info = s => { if (onInfo) onInfo(s); };
  const fail = (reason, extra = {}) => { J('reboot', Object.assign({ phase: 'failed', reason }, extra)); return Object.assign({ ok: false, reason, rebooted: false, rows: [] }, extra); };
  if (!link.alive) return fail('the autopilot is not connected');
  if (link.vehicle && link.vehicle.armed) return fail('refusing to reboot an armed vehicle');
  const id0 = link.identity(); const opts0 = link.opts; const boot0 = link.boot.count; const gen0 = link.generation;
  const types = Object.fromEntries(expect.map(e => [e.name, (link.params.get(e.name) || {}).type]));
  J('reboot', { phase: 'request', identity: id0, expect });
  let ack;
  try { ack = await link.commandAck(246, [1], 3000); } catch (e) { return fail('no answer to the reboot command: ' + e.message); }
  if (ack.result !== 0) return fail(`the autopilot refused to reboot (result ${ack.result})`);
  info('rebooting: the autopilot restarts, this takes a few seconds');
  const t0 = Date.now(); let gone = false, lastTry = 0;
  while (Date.now() - t0 < timeoutMs) {
    await sleep(200);
    if (!gone && (link.generation !== gen0 || link.boot.count > boot0 || !link.connected || Date.now() - link.lastHeartbeatAt > 2500)) { gone = true; info('the autopilot is restarting'); }
    // USB and TCP close on a reboot: open the same connection again until it answers
    if (gone && !link.connected && opts0 && opts0.kind !== 'test' && Date.now() - lastTry > 2500) { lastTry = Date.now(); try { link.connect(opts0); } catch (e) { /* not back yet */ } }
    if (gone && link.alive && Date.now() - link.lastHeartbeatAt < 1500) break;
  }
  if (!gone) return fail('the autopilot did not restart (its heartbeat never stopped)');
  if (!link.alive) return fail(`the autopilot did not come back within ${Math.round(timeoutMs / 1000)} s: reconnect it, then check the values`);
  await link.waitAutopilot(3000);
  const id1 = link.identity();
  if (!sameVehicle(id0, id1)) return fail('a different vehicle answered after the reboot: ' + identityText(id1), { identity: id1 });
  info('back after the reboot: reading the values again');
  const rows = [];
  for (const e of expect) {
    let holds; try { holds = await link.requestParam(e.name, 1500, 3); } catch (err) { holds = undefined; }
    const c = castForType(e.value, (link.params.get(e.name) || {}).type || types[e.name] || 9);
    rows.push({ name: e.name, expect: c.ok ? c.value : e.value, holds, ok: c.ok && sameF32(holds, c.value) });
  }
  const ok = rows.every(r => r.ok);
  J('reboot', { phase: 'verified', ok, rows, identity: id1 });
  return { ok, reason: ok ? null : 'after the reboot: ' + rows.filter(r => !r.ok).map(r => r.holds === undefined ? `${r.name} did not answer` : `${r.name} holds ${num(r.holds)}, expected ${num(r.expect)}`).join('; '), rebooted: true, rows, identity: id1 };
}

module.exports = { castForType, verifiedSet, runTransaction, rebootAndVerify, writeText, txnText, CHANGED, TYPE_NAME };
