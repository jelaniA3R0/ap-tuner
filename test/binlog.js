// Builds small DataFlash (.bin) logs in memory for the tests: FMT definitions, records, and a ready-made
// hover flight (RATE at a set rate, throttle, mode, parameters) with optional gaps and bad samples.
'use strict';

const SIZES = { b: 1, B: 1, M: 1, h: 2, H: 2, g: 2, c: 2, C: 2, i: 4, I: 4, e: 4, E: 4, L: 4, f: 4, n: 4, N: 16, Z: 64, d: 8, q: 8, Q: 8, a: 64 };

class LogBuilder {
  constructor() { this.parts = []; this.defs = {}; }
  /** define a message type: name (<= 4 chars), format characters, comma-separated labels */
  fmt(type, name, format, labels, len = null) {
    const size = len !== null ? len : 3 + [...format].reduce((s, ch) => s + SIZES[ch], 0);
    const b = Buffer.alloc(89);
    b[0] = 0xa3; b[1] = 0x95; b[2] = 128; b[3] = type; b[4] = size;
    b.write(name, 5, 'latin1'); b.write(format, 9, 'latin1'); b.write(labels, 25, 'latin1');
    this.parts.push(b); this.defs[name] = { type, format, size };
    return this;
  }
  /** append a record: values in field order (TimeUS first, in microseconds) */
  rec(name, values) {
    const d = this.defs[name]; const b = Buffer.alloc(d.size);
    b[0] = 0xa3; b[1] = 0x95; b[2] = d.type;
    let off = 3;
    [...d.format].forEach((ch, k) => {
      const v = values[k];
      switch (ch) {
        case 'Q': b.writeBigUInt64LE(BigInt(Math.round(v)), off); break;
        case 'q': b.writeBigInt64LE(BigInt(Math.round(v)), off); break;
        case 'f': b.writeFloatLE(v, off); break;
        case 'd': b.writeDoubleLE(v, off); break;
        case 'B': case 'M': b[off] = v; break;
        case 'b': b.writeInt8(v, off); break;
        case 'H': b.writeUInt16LE(v, off); break;
        case 'h': b.writeInt16LE(v, off); break;
        case 'g': b.writeUInt16LE(v, off); break;   // raw float16 bits
        case 'I': b.writeUInt32LE(v, off); break;
        case 'i': b.writeInt32LE(v, off); break;
        case 'n': case 'N': case 'Z': b.write(String(v), off, SIZES[ch], 'latin1'); break;
        default: throw new Error('format char ' + ch + ' not supported by the test builder');
      }
      off += SIZES[ch];
    });
    this.parts.push(b);
    return this;
  }
  raw(bytes) { this.parts.push(Buffer.from(bytes)); return this; }
  buffer() { return Buffer.concat(this.parts); }
}

/** the definitions used by the ready-made flights */
function standardFormats(L) {
  return L.fmt(64, 'PARM', 'QNf', 'TimeUS,Name,Value')
    .fmt(65, 'MSG', 'QZ', 'TimeUS,Message')
    .fmt(66, 'MODE', 'QBB', 'TimeUS,Mode,ModeNum')
    .fmt(67, 'RATE', 'Qfffffffff', 'TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut')
    .fmt(68, 'QTUN', 'Qf', 'TimeUS,ThO')
    .fmt(69, 'CTUN', 'Qf', 'TimeUS,ThO')
    .fmt(70, 'IMU', 'QBfffH', 'TimeUS,I,GyrX,GyrY,GyrZ,GHz');
}

/**
 * A hover flight in time order (all times in seconds):
 *  opts.firmware ('ArduCopter V4.6.0'), opts.rate (RATE Hz, 300), opts.seconds (60), opts.flyFrom / flyTo (5 / 55):
 *  throttle 0.5 while flying, 0 on the ground; opts.thrMsg ('CTUN' | 'QTUN'), opts.thrScale (1, 100 = percent),
 *  opts.gap: [t0, t1] with no records at all, opts.jitterUs (stamp jitter amplitude), opts.params {name: value},
 *  opts.changes [{t, name, value}], opts.modes [{t, mode}], opts.badAt {nan: t, huge: t},
 *  opts.signal(t, flying) -> {des, act, out} for roll (pitch / yaw get scaled copies),
 *  opts.imu: {rate, ghz} to add IMU records.
 */
function hoverLog(opts = {}) {
  const o = Object.assign({ firmware: 'ArduCopter V4.6.0', rate: 300, seconds: 60, flyFrom: 5, flyTo: 55, thrMsg: 'CTUN', thrScale: 1, gap: null, jitterUs: 0, params: {}, changes: [], modes: [], badAt: {}, imu: null }, opts);
  const sig = o.signal || ((t, fly) => { const des = 10 * Math.sin(2 * Math.PI * 0.5 * t); const err = fly ? 2 : 10; return { des, act: des - err, out: 0.01 * err }; });
  const L = standardFormats(new LogBuilder());
  const events = [];
  const add = (t, name, values) => { if (o.gap && t >= o.gap[0] && t <= o.gap[1]) return; events.push({ t, name, values }); };
  const params = Object.assign({ INS_GYRO_FILTER: 20, INS_GYRO_RATE: 0, SCHED_LOOP_RATE: 400, ATC_RAT_RLL_P: 0.1, ATC_RAT_PIT_P: 0.1, ATC_RAT_YAW_P: 0.2 }, o.params);
  let k = 0;
  for (const [name, v] of Object.entries(params)) events.push({ t: 0.001 + 1e-6 * k++, name: 'PARM', values: [0, name, v] });
  events.push({ t: 0.002, name: 'MSG', values: [0, o.firmware] });
  for (const m of o.modes) add(m.t, 'MODE', [0, m.mode, m.mode]);
  for (const c of o.changes) add(c.t, 'PARM', [0, c.name, c.value]);
  const n = Math.round(o.seconds * o.rate);
  for (let i = 1; i <= n; i++) {
    const t = i / o.rate + (o.jitterUs ? o.jitterUs * 1e-6 * Math.sin(1.7 * i) : 0);
    const fly = t >= o.flyFrom && t < o.flyTo;
    const s = sig(t, fly);
    let des = s.des, act = s.act;
    if (o.badAt.nan !== undefined && Math.abs(t - o.badAt.nan) < 0.5 / o.rate) act = NaN;
    if (o.badAt.huge !== undefined && Math.abs(t - o.badAt.huge) < 0.5 / o.rate) des = 1e6;
    add(t, 'RATE', [0, des, act, s.out, 0.5 * des, 0.5 * act, 0.5 * s.out, 0.2 * des, 0.2 * act, 0.2 * s.out]);
  }
  for (let i = 0; i <= o.seconds * 10; i++) { const t = i / 10 + 0.0003; add(t, o.thrMsg, [0, (t >= o.flyFrom && t < o.flyTo ? 0.5 : 0) * o.thrScale]); }
  if (o.imu) for (let i = 1; i <= o.seconds * o.imu.rate; i++) { const t = i / o.imu.rate + 0.00011; add(t, 'IMU', [0, 0, 0.01 * Math.sin(2 * Math.PI * 40 * t), 0, 0, o.imu.ghz]); }
  events.sort((a, b) => a.t - b.t);
  for (const e of events) { e.values[0] = Math.round(e.t * 1e6); L.rec(e.name, e.values); }
  return L.buffer();
}

module.exports = { LogBuilder, standardFormats, hoverLog };
