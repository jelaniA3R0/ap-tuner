// MAVLink v1/v2 frame parser and v2 encoder, driven by defs.json (generated from the XML).
'use strict';
const defs = require('./defs.json');

const MSG = defs.messages;      // id -> def
const BY_NAME = defs.byName;    // name -> id
const ENUMS = defs.enums;

function x25(crc, byte) {
  let tmp = byte ^ (crc & 0xff);
  tmp = (tmp ^ (tmp << 4)) & 0xff;
  return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
}
function crcBuf(buf, start, end, crc = 0xffff) { for (let i = start; i < end; i++) crc = x25(crc, buf[i]); return crc; }

const READ = {
  char: (b, o) => b[o], int8_t: (b, o) => b.readInt8(o), uint8_t: (b, o) => b[o],
  int16_t: (b, o) => b.readInt16LE(o), uint16_t: (b, o) => b.readUInt16LE(o),
  int32_t: (b, o) => b.readInt32LE(o), uint32_t: (b, o) => b.readUInt32LE(o),
  float: (b, o) => b.readFloatLE(o), double: (b, o) => b.readDoubleLE(o),
  int64_t: (b, o) => Number(b.readBigInt64LE(o)), uint64_t: (b, o) => Number(b.readBigUInt64LE(o)),
};
const WRITE = {
  char: (b, o, v) => { b[o] = v & 0xff; }, int8_t: (b, o, v) => b.writeInt8(v, o), uint8_t: (b, o, v) => { b[o] = v & 0xff; },
  int16_t: (b, o, v) => b.writeInt16LE(v, o), uint16_t: (b, o, v) => b.writeUInt16LE(v, o),
  int32_t: (b, o, v) => b.writeInt32LE(v, o), uint32_t: (b, o, v) => b.writeUInt32LE(v >>> 0, o),
  float: (b, o, v) => b.writeFloatLE(v, o), double: (b, o, v) => b.writeDoubleLE(v, o),
  int64_t: (b, o, v) => b.writeBigInt64LE(BigInt(Math.trunc(v)), o), uint64_t: (b, o, v) => b.writeBigUInt64LE(BigInt(Math.trunc(v)), o),
};
const SIZE = { char: 1, int8_t: 1, uint8_t: 1, int16_t: 2, uint16_t: 2, int32_t: 4, uint32_t: 4, float: 4, int64_t: 8, uint64_t: 8, double: 8 };

function decodePayload(def, payload) {
  // v2 payloads may be truncated (trailing zeros removed): zero-extend
  let p = payload;
  if (p.length < def.length) { p = Buffer.alloc(def.length); payload.copy(p); }
  const out = {};
  for (const f of def.fields) {
    const rd = READ[f.type];
    if (f.len) {
      if (f.type === 'char') {
        let s = p.toString('latin1', f.offset, f.offset + f.len);
        const z = s.indexOf('\0'); if (z >= 0) s = s.slice(0, z);
        out[f.name] = s;
      } else {
        const arr = new Array(f.len);
        for (let i = 0; i < f.len; i++) arr[i] = rd(p, f.offset + i * SIZE[f.type]);
        out[f.name] = arr;
      }
    } else {
      out[f.name] = rd(p, f.offset);
      // a number cannot hold all 64 bits (board UIDs): every unsigned 64-bit field also as exact hex, <name>_hex
      if (f.type === 'uint64_t') out[f.name + '_hex'] = p.readBigUInt64LE(f.offset).toString(16).padStart(16, '0');
    }
  }
  return out;
}

function encodePayload(def, obj) {
  const p = Buffer.alloc(def.length);
  for (const f of def.fields) {
    const v = obj[f.name];
    if (v === undefined || v === null) continue;
    const wr = WRITE[f.type];
    if (f.len) {
      if (f.type === 'char') {
        const s = String(v); p.write(s.slice(0, f.len), f.offset, 'latin1');
      } else for (let i = 0; i < f.len && i < v.length; i++) wr(p, f.offset + i * SIZE[f.type], v[i]);
    } else wr(p, f.offset, v);
  }
  return p;
}

class Encoder {
  constructor(sysid = 254, compid = 190) { this.sysid = sysid; this.compid = compid; this.seq = 0; }
  // returns a Buffer with a MAVLink v2 frame
  pack(name, obj) {
    const id = BY_NAME[name];
    if (id === undefined) throw new Error('unknown message ' + name);
    const def = MSG[id];
    let payload = encodePayload(def, obj);
    // v2 trailing-zero truncation (keep at least 1 byte)
    let n = payload.length; while (n > 1 && payload[n - 1] === 0) n--;
    payload = payload.subarray(0, n);
    const frame = Buffer.alloc(12 + n);
    frame[0] = 0xfd; frame[1] = n; frame[2] = 0; frame[3] = 0; frame[4] = this.seq; frame[5] = this.sysid; frame[6] = this.compid;
    frame[7] = id & 0xff; frame[8] = (id >> 8) & 0xff; frame[9] = (id >> 16) & 0xff;
    payload.copy(frame, 10);
    let crc = crcBuf(frame, 1, 10 + n); crc = x25(crc, def.crcExtra);
    frame[10 + n] = crc & 0xff; frame[11 + n] = crc >> 8;
    this.seq = (this.seq + 1) & 0xff;
    return frame;
  }
}

class Parser {
  constructor() { this.buf = Buffer.alloc(0); this.stats = { frames: 0, badCrc: 0, unknown: 0, dropped: 0 }; }
  // feed bytes; returns array of decoded messages {id,name,sysid,compid,seq,fields,ts}
  feed(chunk) {
    this.buf = this.buf.length ? Buffer.concat([this.buf, chunk]) : Buffer.from(chunk);
    const out = [];
    let i = 0; const b = this.buf;
    while (i < b.length) {
      const m = b[i];
      if (m !== 0xfd && m !== 0xfe) { i++; this.stats.dropped++; continue; }
      const v2 = m === 0xfd;
      const hdr = v2 ? 10 : 6;
      if (i + hdr > b.length) break;
      const len = b[i + 1];
      const incompat = v2 ? b[i + 2] : 0;
      const sigLen = (v2 && (incompat & 1)) ? 13 : 0;
      const total = hdr + len + 2 + sigLen;
      if (i + total > b.length) break;
      const id = v2 ? (b[i + 7] | (b[i + 8] << 8) | (b[i + 9] << 16)) : b[i + 5];
      const def = MSG[id];
      if (!def) { this.stats.unknown++; i++; continue; }
      let crc = crcBuf(b, i + 1, i + hdr + len); crc = x25(crc, def.crcExtra);
      const got = b[i + hdr + len] | (b[i + hdr + len + 1] << 8);
      if (crc !== got) { this.stats.badCrc++; i++; continue; }
      const seq = v2 ? b[i + 4] : b[i + 2], sysid = v2 ? b[i + 5] : b[i + 3], compid = v2 ? b[i + 6] : b[i + 4];
      const fields = decodePayload(def, b.subarray(i + hdr, i + hdr + len));
      out.push({ id, name: def.name, sysid, compid, seq, fields });
      this.stats.frames++;
      i += total;
    }
    this.buf = i >= b.length ? Buffer.alloc(0) : b.subarray(i);
    if (this.buf.length > 1 << 20) this.buf = Buffer.alloc(0); // runaway garbage guard
    return out;
  }
}

module.exports = { Parser, Encoder, MSG, BY_NAME, ENUMS, decodePayload, encodePayload };
