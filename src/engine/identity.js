// Session identity and proposal binding (2026-10-17 audit F-25, F-33). The identity of a connection is the link
// generation, the vehicle's system / component id, autopilot and frame type, AUTOPILOT_VERSION (board UID, firmware)
// and a hash of the parameter snapshot read from it. Every analysis, proposal and write carries it. A proposal has an
// id and a hash over its identity and the exact list of (parameter, value) it writes; the approval sends both back
// and nothing is written when they do not match what the app holds (a newer analysis, another connection).
// Hashes: FNV-1a 64 over a canonical text with float32 values as their 8 hex digits, so both editions agree to the bit.
'use strict';

/** the 8 hex digits of a value as a float32 (what the autopilot stores) */
function f32hex(v) { const b = Buffer.alloc(4); b.writeFloatLE(v, 0); return b.readUInt32LE(0).toString(16).padStart(8, '0'); }
/** FNV-1a 64-bit hash of a text (UTF-8), 16 hex digits */
function fnv1a64(text) {
  let h = 0xcbf29ce484222325n;
  for (const b of Buffer.from(text, 'utf8')) { h ^= BigInt(b); h = (h * 0x100000001b3n) & 0xffffffffffffffffn; }
  return h.toString(16).padStart(16, '0');
}
/** the same value once stored as a float32 (the exact write check; NaN / missing is never the same) */
function sameF32(a, b) { return typeof a === 'number' && typeof b === 'number' && Number.isFinite(a) && Number.isFinite(b) && Math.fround(a) === Math.fround(b); }

/** hash of a parameter snapshot: (name, float32 value) of every number, sorted by name */
function paramHash(params) {
  const names = Object.keys(params || {}).filter(k => typeof params[k] === 'number' && Number.isFinite(params[k])).sort();
  return fnv1a64(names.map(n => n + '=' + f32hex(params[n])).join('\n'));
}

/** the identity of a session: the link's identity() plus the key and the hash of the parameters read */
function sessionIdentity(linkIdentity, params = null) {
  const id = Object.assign({}, linkIdentity);
  id.key = `g${id.gen}:${id.sysid}/${id.compid}`;
  id.paramHash = params ? paramHash(params) : null;
  return id;
}
/** a sentence for the UI and file headers */
function identityText(id) {
  if (!id || id.sysid === null || id.sysid === undefined) return 'no vehicle';
  const type = id.typeName ? String(id.typeName).replace(/^MAV_TYPE_/, '').toLowerCase() : 'type ' + id.type;
  return `system id ${id.sysid} (${type}), ArduPilot ${id.fw || 'firmware not reported'}${id.gitHash ? ' (' + id.gitHash + ')' : ''}, board ${id.uid ? 'UID ' + id.uid.replace(/(00)+$/, '').slice(0, 24) : 'UID not reported'}, connection #${id.gen}`;
}
/** the same vehicle (across connections): ids, frame type and, where both are known, board UID and firmware */
function sameVehicle(a, b) {
  return !!(a && b && a.sysid === b.sysid && a.compid === b.compid && a.type === b.type && (!a.uid || !b.uid || a.uid === b.uid) && (!a.fw || !b.fw || a.fw === b.fw));
}

let seq = 0;
/**
 * A proposal bound to an identity: {id, hash, source, axis, identityKey, gen, sysid, compid, at, rows}; rows are
 * [{name, to, from, ...}] in write order. The hash covers the identity and every (name, from, to) as float32.
 */
function makeProposal({ source, axis = null, identity, rows }) {
  const id = `${source === 'log' ? 'G' : 'L'}${++seq}${axis ? '-' + axis : ''}`;
  const hx = v => (typeof v === 'number' && Number.isFinite(v) ? f32hex(v) : '-');
  const canon = ['APT-PROPOSAL 1', `identity ${identity ? identity.key : '-'} uid ${(identity && identity.uid) || '-'} fw ${(identity && identity.fw) || '-'}`,
    ...rows.map(r => `${r.name}=${hx(r.to)} from ${hx(r.from)}`)].join('\n');
  return { id, hash: fnv1a64(canon), source, axis, identityKey: identity ? identity.key : null, gen: identity ? identity.gen : null,
    sysid: identity ? identity.sysid : null, compid: identity ? identity.compid : null, at: Date.now(), rows };
}

module.exports = { f32hex, fnv1a64, sameF32, paramHash, sessionIdentity, identityText, sameVehicle, makeProposal };
