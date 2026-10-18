// Time-stamp integrity of log streams (audit finding F-07): duplicate and out-of-order stamps, clock resets,
// gaps, and the split of a stream into unbroken segments. Nothing here interpolates: callers get the indices
// of the samples that can be trusted and the places where data are missing.
'use strict';

/** look-ahead used to tell a single bad stamp from a real jump of the clock */
const LOOKAHEAD = 8;
/** a step longer than this many typical sample intervals is a gap (data missing) */
const GAP_FACTOR = 2.5;

/**
 * Check the stamps of one stream (in log order) and return the indices that form a strictly increasing,
 * trustworthy time base:
 *  - a repeated stamp is a duplicate (the later record is dropped);
 *  - a stamp ahead of the stamps that follow it, or behind the stamps before it, is a single bad record
 *    (dropped: a corrupted time field or a false record);
 *  - a jump back that the following records keep (the clock restarted: two logs joined, or a reboot) ends
 *    the usable part: everything from there on is dropped and reported as a reset.
 * t: stamps (any unit, e.g. microseconds). Returns { keep: index[], outOfOrder, duplicates, reset: {from, to} | null, droppedAfterReset }
 */
function cleanTimes(t) {
  const n = t.length;
  const keep = [];
  let outOfOrder = 0, duplicates = 0, reset = null, droppedAfterReset = 0;
  let last = -Infinity;
  for (let i = 0; i < n; i++) {
    const v = t[i];
    if (!Number.isFinite(v)) { outOfOrder++; continue; }
    if (v === last) { duplicates++; continue; }
    const jEnd = Math.min(n - 1, i + LOOKAHEAD);
    if (v > last) {
      // ahead of the stream: most of the next stamps fall back between the last good one and this one
      let between = 0;
      for (let j = i + 1; j <= jEnd; j++) if (t[j] > last && t[j] < v) between++;
      if (between * 2 > jEnd - i) { outOfOrder++; continue; }
      keep.push(i); last = v; continue;
    }
    // behind the stream: a single bad stamp if the stream carries on after the last good stamp (or ends
    // too soon to tell)
    let carriesOn = i + 2 >= n;
    for (let j = i + 1; j <= jEnd && !carriesOn; j++) if (t[j] > last) carriesOn = true;
    if (carriesOn) { outOfOrder++; continue; }
    reset = { from: last, to: v }; droppedAfterReset = n - i;
    break;
  }
  return { keep, outOfOrder, duplicates, reset, droppedAfterReset };
}

/** median of the positive steps of a strictly increasing sequence (0 when fewer than 2 samples) */
function medianStep(t, i0 = 0, i1 = t.length) {
  const d = [];
  for (let i = i0 + 1; i < i1; i++) { const x = t[i] - t[i - 1]; if (x > 0) d.push(x); }
  if (!d.length) return 0;
  d.sort((a, b) => a - b);
  return d[Math.floor(d.length / 2)];
}

/**
 * Split a strictly increasing time base (seconds) into unbroken segments: a step longer than GAP_FACTOR times
 * the median step is a gap. Returns { segments: [[i0, i1)], gaps: [{t0, t1}], medianDt }
 */
function splitAtGaps(t, i0 = 0, i1 = t.length) {
  const out = { segments: [], gaps: [], medianDt: medianStep(t, i0, i1) };
  if (i1 <= i0) return out;
  const lim = GAP_FACTOR * out.medianDt;
  let s = i0;
  for (let i = i0 + 1; i < i1; i++) {
    if (out.medianDt > 0 && t[i] - t[i - 1] > lim) { out.segments.push([s, i]); out.gaps.push({ t0: t[i - 1], t1: t[i] }); s = i; }
  }
  out.segments.push([s, i1]);
  return out;
}

/** the instance field of a message table, if it has one */
function instanceField(m) {
  for (const k of ['I', 'Instance', 'instance']) if (m[k] instanceof Float64Array) return k;
  return null;
}

/**
 * Clean the time bases of the named tables in place (rows reordered never; bad rows removed). Multi-instance
 * tables are checked per instance; GYR uses its sample time (SampleUS). Returns a report
 * { NAME: { outOfOrder, duplicates, reset: {fromS, toS} | null, droppedAfterReset } } of the tables with problems.
 */
function cleanLogTimes(log, names) {
  const report = {};
  for (const name of names) {
    const m = log.msgs[name];
    if (!m || !m.n) continue;
    const tKey = name === 'GYR' && m.SampleUS instanceof Float64Array ? 'SampleUS' : 'TimeUS';
    const tAll = m[tKey];
    if (!(tAll instanceof Float64Array)) continue;
    const instKey = instanceField(m);
    const groups = new Map();
    for (let i = 0; i < m.n; i++) { const g = instKey ? m[instKey][i] : 0; let a = groups.get(g); if (!a) groups.set(g, a = []); a.push(i); }
    const keepRow = new Uint8Array(m.n);
    const r = { outOfOrder: 0, duplicates: 0, reset: null, droppedAfterReset: 0 };
    for (const g of Array.from(groups.keys()).sort((a, b) => a - b)) {
      const rows = groups.get(g);
      const c = cleanTimes(Float64Array.from(rows, i => tAll[i]));
      for (const k of c.keep) keepRow[rows[k]] = 1;
      r.outOfOrder += c.outOfOrder; r.duplicates += c.duplicates; r.droppedAfterReset += c.droppedAfterReset;
      if (c.reset && !r.reset) r.reset = { fromS: c.reset.from / 1e6, toS: c.reset.to / 1e6 };
    }
    const dropped = m.n - keepRow.reduce((s, v) => s + v, 0);
    if (!dropped) continue;
    report[name] = r;
    const idx = []; for (let i = 0; i < m.n; i++) if (keepRow[i]) idx.push(i);
    for (const [k, a] of Object.entries(m)) {
      if (k === 'n') continue;
      m[k] = a instanceof Float64Array ? Float64Array.from(idx, i => a[i]) : idx.map(i => a[i]);
    }
    m.n = idx.length;
  }
  return report;
}

module.exports = { cleanTimes, cleanLogTimes, splitAtGaps, medianStep, GAP_FACTOR };
