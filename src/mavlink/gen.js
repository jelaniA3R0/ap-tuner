// Generates defs.json (message layouts + crc_extra + enums) from the MAVLink XML dialects.
// Run once:  node src/mavlink/gen.js <dir with minimal.xml standard.xml common.xml ardupilotmega.xml>
// No dependencies: a small regex-based XML walk is enough for the MAVLink schema.
'use strict';
const fs = require('fs');
const path = require('path');

const TYPE_SIZE = { char: 1, int8_t: 1, uint8_t: 1, uint8_t_mavlink_version: 1, int16_t: 2, uint16_t: 2,
  int32_t: 4, uint32_t: 4, float: 4, int64_t: 8, uint64_t: 8, double: 8 };

function x25(crc, byte) {
  let tmp = byte ^ (crc & 0xff);
  tmp = (tmp ^ (tmp << 4)) & 0xff;
  return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
}
function x25str(crc, s) { for (let i = 0; i < s.length; i++) crc = x25(crc, s.charCodeAt(i)); return crc; }

function parseType(t) {
  const m = /^([a-z0-9_]+)(?:\[(\d+)\])?$/.exec(t);
  if (!m) throw new Error('bad type ' + t);
  let base = m[1];
  if (base === 'uint8_t_mavlink_version') base = 'uint8_t';
  return { base, len: m[2] ? parseInt(m[2], 10) : 0 };
}

function parseXml(text) {
  // comments first: newer common.xml keeps commented-out message headers ('<!-- <message id="148" ...> in standard.xml -->')
  // that a regex walk would otherwise read as real messages (AUTOPILOT_VERSION came out with LANDING_TARGET's fields)
  text = text.replace(/<!--[\s\S]*?-->/g, '');
  const messages = [];
  const enums = {};
  const msgRe = /<message\s+id="(\d+)"\s+name="([A-Z0-9_]+)"[^>]*>([\s\S]*?)<\/message>/g;
  let m;
  while ((m = msgRe.exec(text))) {
    const id = parseInt(m[1], 10), name = m[2], body = m[3];
    const fields = [];
    let ext = false;
    const tokRe = /<extensions\s*\/>|<field\s+([^>]*?)(?:\/>|>([\s\S]*?)<\/field>)/g;
    let f;
    while ((f = tokRe.exec(body))) {
      if (f[0].startsWith('<extensions')) { ext = true; continue; }
      const attrs = {};
      f[1].replace(/([a-zA-Z_]+)="([^"]*)"/g, (_, k, v) => { attrs[k] = v; return ''; });
      const { base, len } = parseType(attrs.type);
      fields.push({ name: attrs.name, type: base, len, ext, enum: attrs.enum || null, units: attrs.units || null });
    }
    messages.push({ id, name, fields });
  }
  const enumRe = /<enum\s+name="([A-Z0-9_]+)"[^>]*>([\s\S]*?)<\/enum>/g;
  while ((m = enumRe.exec(text))) {
    const name = m[1];
    const entries = enums[name] || (enums[name] = {});
    const eRe = /<entry\s+([^>]*?)(?:\/>|>)/g;
    let e;
    while ((e = eRe.exec(m[2]))) {
      const attrs = {};
      e[1].replace(/([a-zA-Z_]+)="([^"]*)"/g, (_, k, v) => { attrs[k] = v; return ''; });
      if (attrs.value !== undefined && attrs.name) entries[attrs.value] = attrs.name;
    }
  }
  return { messages, enums };
}

function finalize(msg) {
  // wire order: non-extension fields sorted by type size (desc, stable), extensions after in XML order
  const base = msg.fields.filter(f => !f.ext);
  const exts = msg.fields.filter(f => f.ext);
  base.sort((a, b) => TYPE_SIZE[b.type] - TYPE_SIZE[a.type]);
  const wire = base.concat(exts);
  let off = 0;
  for (const f of wire) { f.offset = off; off += TYPE_SIZE[f.type] * (f.len || 1); }
  let crc = 0xffff;
  crc = x25str(crc, msg.name + ' ');
  for (const f of base) {
    crc = x25str(crc, f.type + ' ');
    crc = x25str(crc, f.name + ' ');
    if (f.len) crc = x25(crc, f.len);
  }
  const crcExtra = ((crc & 0xff) ^ (crc >> 8)) & 0xff;
  return { id: msg.id, name: msg.name, crcExtra, length: off,
    fields: wire.map(f => ({ name: f.name, type: f.type, len: f.len, offset: f.offset, ext: f.ext, enum: f.enum, units: f.units })) };
}

function main() {
  const dir = process.argv[2];
  if (!dir) { console.error('usage: node gen.js <xml dir>'); process.exit(1); }
  const all = { messages: {}, byName: {}, enums: {} };
  for (const file of ['minimal.xml', 'standard.xml', 'common.xml', 'ardupilotmega.xml']) {
    const p = path.join(dir, file);
    if (!fs.existsSync(p)) continue;
    const { messages, enums } = parseXml(fs.readFileSync(p, 'utf8'));
    for (const m of messages) {
      const fin = finalize(m);
      all.messages[fin.id] = fin;
      all.byName[fin.name] = fin.id;
    }
    for (const [k, v] of Object.entries(enums)) all.enums[k] = Object.assign(all.enums[k] || {}, v);
  }
  const out = path.join(__dirname, 'defs.json');
  fs.writeFileSync(out, JSON.stringify(all));
  const n = Object.keys(all.messages).length;
  const check = ['HEARTBEAT', 'ATTITUDE', 'PARAM_VALUE', 'PID_TUNING', 'STATUSTEXT', 'RAW_IMU', 'COMMAND_LONG', 'VFR_HUD', 'AUTOPILOT_VERSION', 'LANDING_TARGET'];
  console.log(`wrote ${out}: ${n} messages, ${Object.keys(all.enums).length} enums`);
  for (const c of check) { const m = all.messages[all.byName[c]]; console.log(`  ${c} id=${m.id} crc_extra=${m.crcExtra} len=${m.length}`); }
}
main();
