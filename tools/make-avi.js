// Packs an MJPEG frame bundle (4-byte LE length + JPEG, repeated) into an AVI file (plays in
// Windows Media Player, VLC, Films & TV). Usage: node make-avi.js frames.bin out.avi fps width height
'use strict';
const fs = require('fs');
const [bundle, out, fpsS, wS, hS] = process.argv.slice(2);
const fps = parseInt(fpsS || '12', 10), W = parseInt(wS || '1280', 10), H = parseInt(hS || '720', 10);
const buf = fs.readFileSync(bundle);
const frames = []; let p = 0;
while (p + 4 <= buf.length) { const n = buf.readUInt32LE(p); p += 4; frames.push(buf.subarray(p, p + n)); p += n; }
const u32 = v => { const b = Buffer.alloc(4); b.writeUInt32LE(v >>> 0); return b; };
const u16 = v => { const b = Buffer.alloc(2); b.writeUInt16LE(v & 0xffff); return b; };
const fcc = s => Buffer.from(s, 'ascii');
const chunk = (id, data) => { const pad = data.length & 1 ? Buffer.alloc(1) : Buffer.alloc(0); return Buffer.concat([fcc(id), u32(data.length), data, pad]); };
const list = (type, ...parts) => { const body = Buffer.concat([fcc(type), ...parts]); return Buffer.concat([fcc('LIST'), u32(body.length), body]); };
const maxFrame = Math.max(...frames.map(f => f.length));
const avih = Buffer.concat([u32(Math.round(1e6 / fps)), u32(maxFrame * fps), u32(0), u32(0x10 | 0x100), u32(frames.length), u32(0), u32(1), u32(maxFrame), u32(W), u32(H), u32(0), u32(0), u32(0), u32(0)]);
const strh = Buffer.concat([fcc('vids'), fcc('MJPG'), u32(0), u16(0), u16(0), u32(0), u32(1), u32(fps), u32(0), u32(frames.length), u32(maxFrame), u32(0xffffffff), u32(0), u16(0), u16(0), u16(W), u16(H)]);
const strf = Buffer.concat([u32(40), u32(W), u32(H), u16(1), u16(24), fcc('MJPG'), u32(W * H * 3), u32(0), u32(0), u32(0), u32(0)]);
const hdrl = list('hdrl', chunk('avih', avih), list('strl', chunk('strh', strh), chunk('strf', strf)));
// movi + index (offsets relative to the 'movi' fourcc)
const moviParts = [fcc('movi')]; const idx = []; let off = 4;
for (const f of frames) { const c = chunk('00dc', f); idx.push(Buffer.concat([fcc('00dc'), u32(0x10), u32(off), u32(f.length)])); moviParts.push(c); off += c.length; }
const moviBody = Buffer.concat(moviParts);
const movi = Buffer.concat([fcc('LIST'), u32(moviBody.length), moviBody]);
const idx1 = chunk('idx1', Buffer.concat(idx));
const riffBody = Buffer.concat([fcc('AVI '), hdrl, movi, idx1]);
fs.writeFileSync(out, Buffer.concat([fcc('RIFF'), u32(riffBody.length), riffBody]));
console.log(`wrote ${out}: ${frames.length} frames, ${(frames.length / fps).toFixed(1)} s, ${(fs.statSync(out).size / 1048576).toFixed(1)} MB`);
