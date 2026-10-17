// ArduPilot DataFlash (.bin) log reader. Self-describing via FMT/FMTU messages.
// Produces column arrays per message type (Float64Array per field) for fast analysis.
'use strict';
const fs = require('fs');

const HEAD1 = 0xa3, HEAD2 = 0x95;
const FMT_TYPE = 128, FMT_LEN = 89;

// format char -> [size, reader]
const FC = {
  a: [64, 'int16arr'], b: [1, 'i8'], B: [1, 'u8'], h: [2, 'i16'], H: [2, 'u16'], i: [4, 'i32'], I: [4, 'u32'],
  f: [4, 'f32'], d: [8, 'f64'], n: [4, 'str'], N: [16, 'str'], Z: [64, 'str'], c: [2, 'i16x100'], C: [2, 'u16x100'],
  e: [4, 'i32x100'], E: [4, 'u32x100'], L: [4, 'i32x1e7'], M: [1, 'u8'], q: [8, 'i64'], Q: [8, 'u64'],
};

function readField(buf, off, kind) {
  switch (kind) {
    case 'i8': return buf.readInt8(off);
    case 'u8': return buf[off];
    case 'i16': return buf.readInt16LE(off);
    case 'u16': return buf.readUInt16LE(off);
    case 'i32': return buf.readInt32LE(off);
    case 'u32': return buf.readUInt32LE(off);
    case 'f32': return buf.readFloatLE(off);
    case 'f64': return buf.readDoubleLE(off);
    case 'i16x100': return buf.readInt16LE(off) / 100;
    case 'u16x100': return buf.readUInt16LE(off) / 100;
    case 'i32x100': return buf.readInt32LE(off) / 100;
    case 'u32x100': return buf.readUInt32LE(off) / 100;
    case 'i32x1e7': return buf.readInt32LE(off) / 1e7;
    case 'i64': return Number(buf.readBigInt64LE(off));
    case 'u64': return Number(buf.readBigUInt64LE(off));
    default: return NaN;
  }
}
function readStr(buf, off, n) { let s = buf.toString('latin1', off, off + n); const z = s.indexOf('\0'); return z >= 0 ? s.slice(0, z) : s; }

/**
 * Parse a .bin log.
 * opts.want: optional Set/array of message names to keep as columns (others are counted only).
 * Returns { fmts, counts, msgs: {NAME: {TimeUS: Float64Array, field: Float64Array|array, n}}, params, text, instances }
 */
function parseBin(file, opts = {}) {
  const want = opts.want ? new Set(opts.want) : null;
  const buf = Buffer.isBuffer(file) ? file : fs.readFileSync(file);
  const fmts = {};
  fmts[FMT_TYPE] = { type: FMT_TYPE, len: FMT_LEN, name: 'FMT', format: 'BBnNZ', labels: ['Type', 'Length', 'Name', 'Format', 'Columns'] };
  const counts = {};
  const cols = {};       // name -> {labels, kinds, sizes, arrays: label -> [] , n}
  const params = {};
  const text = [];
  let i = 0;
  const N = buf.length;
  while (i + 3 <= N) {
    if (buf[i] !== HEAD1 || buf[i + 1] !== HEAD2) { i++; continue; }
    const t = buf[i + 2];
    const fm = fmts[t];
    if (!fm) { i++; continue; }
    if (i + fm.len > N) break;
    const body = i + 3;
    if (t === FMT_TYPE) {
      const type = buf[body], len = buf[body + 1];
      const name = readStr(buf, body + 2, 4), format = readStr(buf, body + 6, 16), labels = readStr(buf, body + 22, 64).split(',');
      if (!fmts[type] || type !== FMT_TYPE) fmts[type] = { type, len, name, format, labels };
      counts.FMT = (counts.FMT || 0) + 1;
      i += fm.len; continue;
    }
    counts[fm.name] = (counts[fm.name] || 0) + 1;
    if (fm.name === 'PARM') {
      // Q n f (TimeUS, Name, Value) [+ Default]
      const name = readStr(buf, body + 8, 16); const val = buf.readFloatLE(body + 24);
      if (!(name in params)) params[name] = val; // first value = boot value; later ones are in-flight changes
      params.__changes = params.__changes || [];
      if (name in params && params[name] !== val) params.__changes.push({ t: Number(buf.readBigUInt64LE(body)) / 1e6, name, val });
      i += fm.len; continue;
    }
    if (fm.name === 'MSG') {
      if (fm.msgOff === undefined) { let off = 0; fm.msgOff = 8; for (const ch of fm.format) { if (ch === 'Z') { fm.msgOff = off; break; } off += (FC[ch] || [0])[0]; } }
      text.push({ t: Number(buf.readBigUInt64LE(body)) / 1e6, msg: readStr(buf, body + fm.msgOff, 64) }); i += fm.len; continue;
    }
    if (want && !want.has(fm.name)) { i += fm.len; continue; }
    let c = cols[fm.name];
    if (!c) {
      const kinds = [], sizes = [];
      for (const ch of fm.format) { const fc = FC[ch]; if (!fc) { kinds.push(null); sizes.push(0); continue; } sizes.push(fc[0]); kinds.push(fc[1]); }
      c = cols[fm.name] = { labels: fm.labels, kinds, sizes, data: fm.labels.map(() => []), n: 0 };
    }
    let off = body;
    for (let k = 0; k < c.kinds.length; k++) {
      const kind = c.kinds[k];
      if (kind === 'str') c.data[k].push(readStr(buf, off, c.sizes[k]));
      else if (kind === 'int16arr') { const a = new Int16Array(32); for (let j = 0; j < 32; j++) a[j] = buf.readInt16LE(off + 2 * j); c.data[k].push(a); }
      else if (kind) c.data[k].push(readField(buf, off, kind));
      off += c.sizes[k];
    }
    c.n++;
    i += fm.len;
  }
  // convert numeric columns to Float64Array
  const msgs = {};
  for (const [name, c] of Object.entries(cols)) {
    const m = { n: c.n };
    c.labels.forEach((lab, k) => {
      const kind = c.kinds[k];
      m[lab] = (kind === 'str' || kind === 'int16arr' || !kind) ? c.data[k] : Float64Array.from(c.data[k]);
    });
    msgs[name] = m;
  }
  return { fmts, counts, msgs, params, text };
}

// split a message table by instance field (e.g. IMU.I) -> {inst: {field: Float64Array}}
function splitByInstance(m, field = 'I') {
  if (!m || !m[field]) return { 0: m };
  const out = {};
  const inst = m[field];
  const idx = {};
  for (let k = 0; k < m.n; k++) { const v = inst[k]; (idx[v] || (idx[v] = [])).push(k); }
  for (const [v, ks] of Object.entries(idx)) {
    const o = { n: ks.length };
    for (const [lab, arr] of Object.entries(m)) {
      if (lab === 'n') continue;
      if (arr instanceof Float64Array) { const a = new Float64Array(ks.length); for (let j = 0; j < ks.length; j++) a[j] = arr[ks[j]]; o[lab] = a; }
      else o[lab] = ks.map(j => arr[j]);
    }
    out[v] = o;
  }
  return out;
}

// ---- helpers for partial logs (a header plus a slice from the middle, e.g. downloaded over MAVLink) ----

/** read the FMT table from a log head: {type: {len, name, labels}} */
function readFormats(buf) {
  const fmts = { [FMT_TYPE]: { type: FMT_TYPE, len: FMT_LEN, name: 'FMT', labels: ['Type', 'Length', 'Name', 'Format', 'Columns'] } };
  for (let i = 0; i + FMT_LEN <= buf.length; i++) {
    if (buf[i] !== HEAD1 || buf[i + 1] !== HEAD2 || buf[i + 2] !== FMT_TYPE) continue;
    const b = i + 3; const type = buf[b];
    if (type !== FMT_TYPE) fmts[type] = { type, len: buf[b + 1], name: readStr(buf, b + 2, 4), format: readStr(buf, b + 6, 16), labels: readStr(buf, b + 22, 64).split(',') };
    i += FMT_LEN - 1;
  }
  return fmts;
}
/** a message starts at i and the next two messages chain after it (guards against false syncs inside data) */
function chainAt(buf, i, fmts, depth = 3) {
  let j = i;
  for (let k = 0; k < depth; k++) {
    if (j + 3 > buf.length) return k > 0;
    if (buf[j] !== HEAD1 || buf[j + 1] !== HEAD2) return false;
    const fm = fmts[buf[j + 2]]; if (!fm || !fm.len) return false;
    j += fm.len;
  }
  return true;
}
/** offset of the first well-chained message at or after 'from' (-1 if none) */
function firstSync(buf, fmts, from = 0) { for (let i = from; i + 3 <= buf.length; i++) if (chainAt(buf, i, fmts)) return i; return -1; }
/** byte length of the complete, chained messages from the start of buf */
function completeLength(buf, fmts) {
  let i = firstSync(buf, fmts, 0); if (i < 0) return 0;
  let last = i;
  while (i + 3 <= buf.length) {
    if (buf[i] !== HEAD1 || buf[i + 1] !== HEAD2) { const n = firstSync(buf, fmts, i + 1); if (n < 0) break; i = n; continue; }
    const fm = fmts[buf[i + 2]]; if (!fm || !fm.len) { const n = firstSync(buf, fmts, i + 1); if (n < 0) break; i = n; continue; }
    if (i + fm.len > buf.length) break;
    i += fm.len; last = i;
  }
  return last;
}
/** first TimeUS (seconds) found in a chunk from the middle of a log, or null */
function firstTime(buf, fmts) {
  let i = firstSync(buf, fmts, 0);
  while (i >= 0 && i + 3 <= buf.length) {
    const fm = fmts[buf[i + 2]];
    if (fm && fm.labels && fm.labels[0] === 'TimeUS' && /^Q/.test(fm.format || '') && i + 11 <= buf.length) return Number(buf.readBigUInt64LE(i + 3)) / 1e6;
    if (!fm || !fm.len) { i = firstSync(buf, fmts, i + 1); continue; }
    i += fm.len;
  }
  return null;
}
/** last TimeUS (seconds) in a chunk, or null */
function lastTime(buf, fmts) {
  let i = firstSync(buf, fmts, 0); let t = null;
  while (i >= 0 && i + 3 <= buf.length) {
    const fm = fmts[buf[i + 2]];
    if (!fm || !fm.len) { i = firstSync(buf, fmts, i + 1); continue; }
    if (i + fm.len > buf.length) break;
    if (fm.labels && fm.labels[0] === 'TimeUS' && /^Q/.test(fm.format || '')) { const v = Number(buf.readBigUInt64LE(i + 3)) / 1e6; if (v > 0 && v < 1e7) t = v; }
    i += fm.len;
  }
  return t;
}

module.exports = { parseBin, splitByInstance, readFormats, firstSync, completeLength, firstTime, lastTime };
