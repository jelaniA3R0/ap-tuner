// Generates web/icon.png and AP Tuner.ico (a gauge-style icon) with no dependencies.
'use strict';
const fs = require('fs');
const path = require('path');
const zlib = require('zlib');

function crc32(buf) { let c, crc = 0xffffffff; for (let n = 0; n < buf.length; n++) { c = (crc ^ buf[n]) & 0xff; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; crc = (crc >>> 8) ^ c; } return (crc ^ 0xffffffff) >>> 0; }
function chunk(type, data) { const len = Buffer.alloc(4); len.writeUInt32BE(data.length); const td = Buffer.concat([Buffer.from(type), data]); const crc = Buffer.alloc(4); crc.writeUInt32BE(crc32(td)); return Buffer.concat([len, td, crc]); }
function png(w, h, rgba) {
  const raw = Buffer.alloc((w * 4 + 1) * h);
  for (let y = 0; y < h; y++) { raw[y * (w * 4 + 1)] = 0; rgba.copy(raw, y * (w * 4 + 1) + 1, y * w * 4, (y + 1) * w * 4); }
  const ihdr = Buffer.alloc(13); ihdr.writeUInt32BE(w, 0); ihdr.writeUInt32BE(h, 4); ihdr[8] = 8; ihdr[9] = 6; ihdr[10] = 0; ihdr[11] = 0; ihdr[12] = 0;
  return Buffer.concat([Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]), chunk('IHDR', ihdr), chunk('IDAT', zlib.deflateSync(raw, { level: 9 })), chunk('IEND', Buffer.alloc(0))]);
}
function draw(size) {
  const img = Buffer.alloc(size * size * 4);
  const cx = size / 2, cy = size / 2, R = size * 0.46;
  const put = (x, y, r, g, b, a) => { const i = (y * size + x) * 4; const a0 = img[i + 3] / 255, a1 = a; const ao = a1 + a0 * (1 - a1); if (ao <= 0) return; img[i] = Math.round((r * a1 + img[i] * a0 * (1 - a1)) / ao); img[i + 1] = Math.round((g * a1 + img[i + 1] * a0 * (1 - a1)) / ao); img[i + 2] = Math.round((b * a1 + img[i + 2] * a0 * (1 - a1)) / ao); img[i + 3] = Math.round(ao * 255); };
  const smooth = (d, edge) => Math.max(0, Math.min(1, (edge - d) + 0.5));
  for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
    const dx = x + 0.5 - cx, dy = y + 0.5 - cy; const d = Math.hypot(dx, dy);
    // background disc: dark navy with a subtle gradient
    const a = smooth(d, R);
    if (a > 0) { const t = (y / size); put(x, y, Math.round(20 + 10 * t), Math.round(28 + 14 * t), Math.round(48 + 20 * t), a); }
    // ring (cyan)
    const ring = Math.min(smooth(d, R * 0.86), smooth(R * 0.72 - 0, d) ) ;
    const inRing = d <= R * 0.86 && d >= R * 0.72 ? Math.min(smooth(d, R * 0.86), smooth(R * 0.72, d)) : 0;
    // arc from 135 deg to 405 deg (gauge), colour sweeps green -> cyan
    const ang = (Math.atan2(dy, dx) * 180 / Math.PI + 360 + 90) % 360; // 0 at top
    const onArc = ang >= 225 || ang <= 135;
    if (inRing > 0 && onArc) { const f = ((ang + 135) % 360) / 270; put(x, y, Math.round(56 + 20 * f), Math.round(189 + 20 * f), Math.round(248 - 100 * (1 - f)), inRing); }
    void ring;
  }
  // needle: from centre towards 60 deg right of top
  const na = (-90 + 50) * Math.PI / 180, nl = R * 0.62, nw = size * 0.035;
  for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
    const dx = x + 0.5 - cx, dy = y + 0.5 - cy;
    const along = dx * Math.cos(na) + dy * Math.sin(na), across = -dx * Math.sin(na) + dy * Math.cos(na);
    if (along >= -size * 0.06 && along <= nl) { const w = nw * (1 - 0.6 * Math.max(0, along) / nl); const a = smooth(Math.abs(across), w); if (a > 0) put(x, y, 255, 255, 255, a); }
  }
  // hub
  for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) { const d = Math.hypot(x + 0.5 - cx, y + 0.5 - cy); const a = smooth(d, size * 0.07); if (a > 0) put(x, y, 255, 255, 255, a); }
  return img;
}
function ico(pngs) {
  // ICO container with PNG-compressed images (Vista+)
  const n = pngs.length; const header = Buffer.alloc(6); header.writeUInt16LE(0, 0); header.writeUInt16LE(1, 2); header.writeUInt16LE(n, 4);
  const dir = Buffer.alloc(16 * n); let off = 6 + 16 * n; const datas = [];
  pngs.forEach((p, i) => { const s = p.size >= 256 ? 0 : p.size; dir[i * 16] = s; dir[i * 16 + 1] = s; dir[i * 16 + 2] = 0; dir[i * 16 + 3] = 0; dir.writeUInt16LE(1, i * 16 + 4); dir.writeUInt16LE(32, i * 16 + 6); dir.writeUInt32LE(p.data.length, i * 16 + 8); dir.writeUInt32LE(off, i * 16 + 12); off += p.data.length; datas.push(p.data); });
  return Buffer.concat([header, dir, ...datas]);
}
const root = path.join(__dirname, '..');
const sizes = [256, 48, 32, 16];
const pngs = sizes.map(s => ({ size: s, data: png(s, s, draw(s)) }));
fs.writeFileSync(path.join(root, 'web', 'icon.png'), pngs[0].data);
fs.writeFileSync(path.join(root, 'AP Tuner.ico'), ico(pngs));
console.log('wrote web/icon.png and AP Tuner.ico');
