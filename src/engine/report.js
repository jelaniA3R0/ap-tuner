// Parameter-file builder (Mission Planner / QGC .param format: NAME,VALUE per line; # comments).
// Every value is checked before it goes into a file (2026-10-17 audit F-43: type, enumeration, bitmask, limits,
// documented range) and written in fixed point (no exponent notation); the file names the vehicle it is for.
'use strict';
const { axisParamNames } = require('./vehicle');
const { validateParam, formatParamValue } = require('../model/parammeta');
const { identityText } = require('./identity');

const PRETTY = { P: 'rate P', I: 'rate I', D: 'rate D', FF: 'feed-forward', FLTT: 'target filter (Hz)', FLTE: 'error filter (Hz)', FLTD: 'D filter (Hz)', SMAX: 'slew limit', IMAX: 'I limit', angleP: 'angle P', tconst: 'time constant' };
const WHY = { P: 'from the identified rate-loop model (preset margins)', I: 'follows P / FF (ArduPilot rule)', D: 'from the identified rate-loop model', FF: 'from the stick events (feed-forward)',
  FLTT: 'filter rule', FLTE: 'filter rule', FLTD: 'filter rule', SMAX: 'slew-rate limit (needed for the oscillation signal)', IMAX: 'the I term holds a steady correction near its limit',
  angleP: 'from the rate-loop crossover (at most x1.5 per round)', tconst: 'from the rate-loop crossover' };

/** rows [{group, name, value, from, label, pct, reason}] from a log/live analysis result; params: the values in the log (notch rows' "from") */
function proposalRows(family, axes, noise, params = null) {
  const rows = [];
  for (const [axis, a] of Object.entries(axes)) {
    if (!a || !a.proposal) continue;
    const n = axisParamNames(family, axis);
    for (const c of a.proposal.changes) { const pn = n[c.key]; if (pn) rows.push({ group: axis, name: pn, value: +c.to.toPrecision(6), from: c.from, label: PRETTY[c.key] || c.key, pct: c.pct, reason: WHY[c.key] || '' }); }
  }
  if (noise && noise.recommended) {
    const why = noise.recommended.why || 'notch from the gyro noise in the log';
    for (const [k, v] of Object.entries(noise.recommended.params)) rows.push({ group: 'notch', name: k, value: v, from: params ? params[k] : undefined, label: k.replace('INS_HNTCH_', 'notch '), reason: why });
    rows.push({ group: 'notch', name: 'INS_GYRO_FILTER', value: noise.recommended.gyroFilterHz, from: params ? params.INS_GYRO_FILTER : undefined, label: 'gyro low-pass (Hz)', reason: 'gyro low-pass that the notch allows' });
  }
  return rows;
}

/**
 * The text of a .param file and what the checks found: {text, errors: [{name, value, why}], warnings: [{name, why}]}.
 * strict (proposals): a row that fails a check is left out of the file (a comment says why); not strict (an UNDO
 * file of values read from the autopilot): every row is kept, findings become comments. identity: the vehicle the file
 * is for ({sysid, typeName, fw, gitHash, uid, gen}) or a sentence; it goes into the header.
 */
function paramFileChecked(rows, header = [], { identity = null, strict = true } = {}) {
  const lines = ['# AP Tuner parameter file. Load it in Mission Planner: CONFIG > Full Parameter List > Load from file, then Write Params.'];
  if (identity) lines.push('# For: ' + (typeof identity === 'string' ? identity : identityText(identity)) + '. Load it only on this vehicle.');
  lines.push(...header.map(h => '# ' + h));
  const errors = [], warnings = [];
  let group = null;
  for (const r of rows) {
    if (r.group !== group) { group = r.group; lines.push(`# --- ${group} ---`); }
    const v = validateParam(r.name, r.value);
    if (!v.ok && strict) { errors.push({ name: r.name, value: r.value, why: v.errors.join(', ') }); lines.push(`# NOT IN THIS FILE: ${r.name} = ${r.value} ${v.errors.join(', ')}`); continue; }
    for (const w of v.ok ? v.warnings : v.errors) { warnings.push({ name: r.name, why: w }); lines.push(`# NOTE: ${r.name} ${w}`); }
    if (v.reboot) lines.push(`# ${r.name} takes effect only after a reboot of the autopilot`);
    // Mission Planner reads NAME,VALUE lines; keep the previous value on a comment line of its own
    if (typeof r.from === 'number' && Number.isFinite(r.from)) lines.push(`# ${r.name} was ${formatParamValue(r.name, r.from)}`);
    lines.push(`${r.name},${formatParamValue(r.name, r.value)}`);
  }
  return { text: lines.join('\r\n') + '\r\n', errors, warnings };
}
function paramFile(rows, header = [], opts = {}) { return paramFileChecked(rows, header, opts).text; }

/** the vehicle a log comes from: system id (SYSID_THISMAV), firmware version and git hash from its MSG text */
function logIdentity(result) {
  const p = (result && result.params) || {};
  const m = /V(\d+)\.(\d+)\.(\d+)/.exec((result && result.firmware) || '');
  const h = /\(([0-9a-f]{6,})\)/i.exec((result && result.firmware) || '');
  return { sysid: typeof p.SYSID_THISMAV === 'number' ? p.SYSID_THISMAV : null, fw: m ? `${m[1]}.${m[2]}.${m[3]}` : null, gitHash: h ? h[1].toLowerCase() : null, firmwareText: (result && result.firmware) || null };
}
/** does the log come from the vehicle on this connection? {problems: [text] (writing is refused), notes: [text]} */
function logVehicleCheck(result, identity) {
  const L = logIdentity(result); const problems = [], notes = [];
  if (L.sysid === null) notes.push('the log has no SYSID_THISMAV: the system id could not be compared');
  else if (identity && identity.sysid !== null && identity.sysid !== undefined && L.sysid !== identity.sysid) problems.push(`the log is from system id ${L.sysid}, the autopilot on this connection is system id ${identity.sysid}`);
  const fw = identity && identity.fw ? String(identity.fw).replace(/-.*$/, '') : null;
  if (!L.fw || !fw) notes.push('the firmware version could not be compared (' + (!L.fw ? 'not in the log' : 'the autopilot did not report it') + ')');
  else if (L.fw !== fw) problems.push(`the log was recorded with ArduPilot ${L.fw}, the autopilot now runs ${identity.fw}: fly again with this firmware`);
  else if (L.gitHash && identity.gitHash && !String(L.gitHash).startsWith(String(identity.gitHash).toLowerCase()) && !String(identity.gitHash).toLowerCase().startsWith(L.gitHash)) problems.push(`the log was recorded with another build of ArduPilot ${L.fw} (${L.gitHash}, now ${identity.gitHash}): fly again with this firmware`);
  return { log: L, problems, notes };
}

module.exports = { proposalRows, paramFile, paramFileChecked, logIdentity, logVehicleCheck, PRETTY };
