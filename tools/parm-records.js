// Every PARM record of a dataflash log, in order: {t (s), name, val}. The app's log reader keeps only the boot value
// and the changes that differ from it, so a write back to the boot value (a revert) does not show there; the
// simulator checks (sitl-f01.js, osc-episodes.js) need all of them.
'use strict';
const fs = require('fs');

const FMT_TYPE = 128, FMT_LEN = 89;   // header 3 + type 1 + len 1 + name 4 + format 16 + labels 64
const str = (b, o, n) => { let e = o; while (e < o + n && b[e] !== 0) e++; return b.toString('latin1', o, e); };

function parmRecords(file) {
  const buf = fs.readFileSync(file);
  const fmts = { [FMT_TYPE]: { len: FMT_LEN, name: 'FMT' } };
  const out = [];
  let i = 0;
  while (i + 3 <= buf.length) {
    if (buf[i] !== 0xA3 || buf[i + 1] !== 0x95) { i++; continue; }
    const type = buf[i + 2];
    const fm = fmts[type];
    if (!fm || i + fm.len > buf.length) { i++; continue; }
    if (type === FMT_TYPE) fmts[buf[i + 3]] = { len: buf[i + 4], name: str(buf, i + 5, 4) };
    else if (fm.name === 'PARM') out.push({ t: Number(buf.readBigUInt64LE(i + 3)) / 1e6, name: str(buf, i + 11, 16), val: buf.readFloatLE(i + 27) });
    i += fm.len;
  }
  return out;
}

module.exports = { parmRecords };
