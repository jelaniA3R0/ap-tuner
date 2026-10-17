// Parameter-file builder (Mission Planner / QGC .param format: NAME,VALUE per line; # comments)
'use strict';
const { axisParamNames } = require('./vehicle');

const PRETTY = { P: 'rate P', I: 'rate I', D: 'rate D', FF: 'feed-forward', FLTT: 'target filter (Hz)', FLTE: 'error filter (Hz)', FLTD: 'D filter (Hz)', SMAX: 'slew limit', IMAX: 'I limit', angleP: 'angle P', tconst: 'time constant' };

/** rows [{name, value, from, note}] from a log/live analysis result */
function proposalRows(family, axes, noise) {
  const rows = [];
  for (const [axis, a] of Object.entries(axes)) {
    if (!a || !a.proposal) continue;
    const n = axisParamNames(family, axis);
    for (const c of a.proposal.changes) { const pn = n[c.key]; if (pn) rows.push({ group: axis, name: pn, value: +c.to.toPrecision(6), from: c.from, label: PRETTY[c.key] || c.key, pct: c.pct }); }
  }
  if (noise && noise.recommended) {
    for (const [k, v] of Object.entries(noise.recommended.params)) rows.push({ group: 'notch', name: k, value: v, from: noise.chain ? undefined : undefined, label: k.replace('INS_HNTCH_', 'notch ') });
    rows.push({ group: 'notch', name: 'INS_GYRO_FILTER', value: noise.recommended.gyroFilterHz, label: 'gyro low-pass (Hz)' });
  }
  return rows;
}

function paramFile(rows, header = []) {
  const lines = ['# AP Tuner parameter file. Load it in Mission Planner: CONFIG > Full Parameter List > Load from file, then Write Params.', ...header.map(h => '# ' + h)];
  let group = null;
  for (const r of rows) {
    if (r.group !== group) { group = r.group; lines.push(`# --- ${group} ---`); }
    // Mission Planner reads NAME,VALUE lines; keep the previous value on a comment line of its own
    if (r.from !== undefined) lines.push(`# ${r.name} was ${+(+r.from).toPrecision(6)}`);
    lines.push(`${r.name},${r.value}`);
  }
  return lines.join('\r\n') + '\r\n';
}

module.exports = { proposalRows, paramFile, PRETTY };
