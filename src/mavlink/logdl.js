// Download DataFlash logs from the autopilot over MAVLink (LOG_REQUEST_LIST / LOG_REQUEST_DATA).
// Over USB a whole log can take minutes, so fetchWindow() pulls only the log head (formats and
// parameters) plus the bytes around a time window, found by bisection on small probes.
'use strict';
const { readFormats, completeLength, firstSync, firstTime, lastTime } = require('../log/dfreader');

const BLOCK = 90;   // LOG_DATA payload size

class LogDownloader {
  constructor(link) { this.link = link; }

  /** list the logs on the autopilot: [{id, size, timeUtc}] sorted by id */
  list(timeoutMs = 4000) {
    const link = this.link;
    return new Promise((resolve) => {
      const logs = new Map(); let total = null; let quiet = null;
      const done = () => { link.off('LOG_ENTRY', on); clearTimeout(hard); clearTimeout(quiet); resolve([...logs.values()].sort((a, b) => a.id - b.id)); };
      const on = f => {
        total = f.num_logs;
        if (f.num_logs === 0) return done();
        logs.set(f.id, { id: f.id, size: f.size, timeUtc: f.time_utc, last: f.last_log_num });
        clearTimeout(quiet); quiet = setTimeout(done, 700);
        if (total !== null && logs.size >= total) done();
      };
      const hard = setTimeout(done, timeoutMs);
      link.on('LOG_ENTRY', on);
      link.send('LOG_REQUEST_LIST', { target_system: link.target.sysid, target_component: link.target.compid, start: 0, end: 0xffff });
    });
  }

  /** read bytes [ofs, ofs+len) of log id; re-requests gaps; resolves a Buffer (shorter at the end of the log) */
  readRange(id, ofs, len, { onProgress = null, idleMs = 1200, maxRounds = 12 } = {}) {
    const link = this.link;
    const nBlocks = Math.ceil(len / BLOCK);
    const got = new Uint8Array(nBlocks); let gotN = 0; let eofAt = null;   // eofAt: byte length actually available
    const buf = Buffer.alloc(len);
    return new Promise((resolve, reject) => {
      let lastRx = Date.now(); let rounds = 0; let timer = null;
      const finish = (err) => { link.off('LOG_DATA', on); clearInterval(timer); if (err) reject(err); else resolve(buf.subarray(0, eofAt !== null ? eofAt : len)); };
      const on = f => {
        if (f.id !== id) return;
        lastRx = Date.now();
        const rel = f.ofs - ofs;
        if (f.count < BLOCK && rel >= 0 && rel <= len) eofAt = eofAt === null ? Math.min(len, rel + f.count) : Math.min(eofAt, rel + f.count);
        if (rel < 0 || rel >= len || rel % BLOCK) return;
        const k = rel / BLOCK;
        if (!got[k]) { got[k] = 1; gotN++; }
        const n = Math.min(f.count, len - rel);
        for (let j = 0; j < n; j++) buf[rel + j] = f.data[j];
        if (onProgress && (gotN % 64 === 0)) onProgress(Math.min(len, gotN * BLOCK), len);
        if (complete()) finish();
      };
      const needBlocks = () => eofAt === null ? nBlocks : Math.ceil(eofAt / BLOCK);
      const complete = () => { const nb = needBlocks(); for (let k = 0; k < nb; k++) if (!got[k]) return false; return true; };
      const request = () => {
        // first missing run of blocks
        const nb = needBlocks(); let a = 0; while (a < nb && got[a]) a++;
        if (a >= nb) return finish();
        let b = a; while (b < nb && !got[b]) b++;
        rounds++;
        if (rounds > maxRounds) return finish(new Error(`log download stalled at ${ofs + a * BLOCK} bytes`));
        lastRx = Date.now();
        link.send('LOG_REQUEST_DATA', { target_system: link.target.sysid, target_component: link.target.compid, id, ofs: ofs + a * BLOCK, count: Math.min(len, b * BLOCK) - a * BLOCK });
      };
      link.on('LOG_DATA', on);
      timer = setInterval(() => { if (Date.now() - lastRx > idleMs) request(); }, 200);
      request();
    });
  }

  end() { this.link.send('LOG_REQUEST_END', { target_system: this.link.target.sysid, target_component: this.link.target.compid }); }

  /**
   * Fetch the head and the part of log id that covers [tStart, tEnd] seconds (TimeUS clock).
   * Returns {buf (a parseable partial log), head, start, end, firstT, lastT, bytes} or throws.
   * With tStart === null the whole log is fetched.
   */
  async fetchWindow(log, tStart, tEnd, { onProgress = () => {}, marginBefore = 3, marginAfter = 1 } = {}) {
    const id = log.id, size = log.size;
    const HEAD = Math.min(size, 1 << 20);
    onProgress({ stage: 'head', done: 0, total: HEAD });
    const head = await this.readRange(id, 0, HEAD, { onProgress: (d, t) => onProgress({ stage: 'head', done: d, total: t }) });
    const fmts = readFormats(head);
    const names = new Set(Object.values(fmts).map(f => f.name));
    if (!['PIDR', 'PIQR', 'RATE'].some(n => names.has(n))) throw new Error('the log head has no rate-loop formats (is this a flight log?)');
    if (tStart === null || size <= HEAD) {
      const rest = size > HEAD ? await this.readRange(id, HEAD, size - HEAD, { onProgress: (d, t) => onProgress({ stage: 'whole log', done: HEAD + d, total: size }) }) : Buffer.alloc(0);
      return { buf: Buffer.concat([head, rest]), fmts, start: 0, end: size, bytes: size, whole: true };
    }
    const headEnd = completeLength(head, fmts);
    const probe = async (o) => { const b = await this.readRange(id, o, Math.min(4096, size - o), { idleMs: 800, maxRounds: 6 }); return b; };
    // log time span: last timestamp near the end
    const tail = await probe(Math.max(headEnd, size - 4096));
    const lastT = lastTime(tail, fmts);
    const firstT = firstTime(head.subarray(Math.max(0, headEnd - 8192)), fmts);
    if (lastT !== null && tEnd > lastT + 1) throw Object.assign(new Error('the marked window is after the end of this log'), { code: 'after' });
    if (firstT !== null && tStart < firstT - 30) throw Object.assign(new Error('the marked window is before the start of this log'), { code: 'before' });
    // bisection: smallest offset whose first timestamp >= t
    const locate = async (t) => {
      let lo = headEnd, hi = size; let n = 0;
      while (hi - lo > 8192 && n < 40) {
        const mid = Math.floor((lo + hi) / 2); n++;
        onProgress({ stage: 'finding the marked 30 s', done: n, total: 22 });
        const tm = firstTime(await probe(mid), fmts);
        if (tm === null || tm < t) lo = mid; else hi = mid;
      }
      return lo;
    };
    const start = await locate(tStart - marginBefore);
    const end = Math.min(size, (await locate(tEnd + marginAfter)) + 8192);
    const mid = await this.readRange(id, start, end - start, { onProgress: (d, t) => onProgress({ stage: 'downloading the marked 30 s', done: d, total: t }) });
    const s0 = firstSync(mid, fmts, 0);
    const buf = Buffer.concat([head.subarray(0, headEnd), s0 > 0 ? mid.subarray(s0) : mid]);
    return { buf, fmts, start, end, firstT, lastT, bytes: HEAD + (end - start), whole: false };
  }
}

module.exports = { LogDownloader };
