// "Telemetry + USB" session, for vehicles flown on a slow telemetry radio:
//   flight (telemetry only marks the time) -> capture (30 s of stick taps, nothing written)
//   -> land (wait for disarm) -> usb (the pilot plugs the autopilot into this PC; the app finds the port)
//   -> download (log head + the marked 30 s, at the autopilot's full logging rate) -> review (issues and
//   proposed changes) -> write (on the ground, disarmed, each value read back) -> done (undo file kept).
// Without telemetry the pilot can skip straight to USB; the longest hover in the log is used then.
'use strict';
const fs = require('fs');
const path = require('path');
const EventEmitter = require('events');
const { execFile } = require('child_process');
const { LogDownloader } = require('../mavlink/logdl');
const { analyzeLog } = require('./loganalysis');
const { proposalRows, paramFile } = require('./report');
const { axisIssues } = require('./issues');
const { GOOD_MODES } = require('./flight');

const RUNNING = new Set(['flight', 'capture', 'captured', 'usb', 'download']);

/** active serial ports from the registry: [{port, dev}] (fast; dev is e.g. \Device\USBSER000) */
function serialMap() {
  return new Promise(resolve => {
    execFile('reg', ['query', 'HKLM\\HARDWARE\\DEVICEMAP\\SERIALCOMM'], { windowsHide: true, timeout: 5000 }, (err, out) => {
      if (err || !out) return resolve([]);
      const list = [];
      for (const line of out.split(/\r?\n/)) { const m = /^\s+(\S+)\s+REG_SZ\s+(COM\d+)/i.exec(line); if (m) list.push({ port: m[2], dev: m[1] }); }
      resolve(list);
    });
  });
}

class GroundTune extends EventEmitter {
  constructor(link, { sessionsDir }) {
    super(); this.link = link; this.dir = path.join(sessionsDir, 'ground'); this.dl = new LogDownloader(link);
    this.boot = { ms: null, at: 0 };
    link.on('message', m => { const f = m.fields; if (f && f.time_boot_ms !== undefined && m.compid === 1 && f.time_boot_ms > 0) { this.boot = { ms: f.time_boot_ms, at: Date.now() }; } });
    this.reset();
  }

  reset() {
    if (this._timer) clearTimeout(this._timer); this._timer = null;
    this.phase = 'idle'; this.info = ''; this.since = Date.now();
    this.opts = { axes: ['roll', 'pitch'], preset: 'normal', seconds: 30, usb: null };
    this.mark = null; this.captureStart = null; this._tFlying = 0; this.usb = null; this.progress = null; this.result = null; this.rows = [];
    this.writes = []; this.files = {}; this.notes = []; this.error = null; this.sysid = null; this._knownPorts = null; this._busy = new Set(); this._tried = new Set();
    this.restoreMark();
  }

  // the marked window survives an app restart (the pilot may close the app before plugging in USB)
  _markFile() { return path.join(this.dir, 'last-mark.json'); }
  saveMark() { try { fs.mkdirSync(this.dir, { recursive: true }); fs.writeFileSync(this._markFile(), JSON.stringify(this.mark)); } catch (e) { /* not fatal */ } }
  restoreMark() { this.savedMark = null; try { const m = JSON.parse(fs.readFileSync(this._markFile(), 'utf8')); if (m && Date.now() - m.savedAt < 6 * 3600e3) this.savedMark = m; } catch (e) { /* none */ } }

  nowBootMs() { return this.boot.ms === null || Date.now() - this.boot.at > 5000 ? null : this.boot.ms + (Date.now() - this.boot.at); }
  familyOf(v) {
    if (!v) return null;
    if (/^Q/.test(v.modeName || '')) return 'vtol';
    return GOOD_MODES.plane.includes(v.modeName) || v.type === 1 ? 'plane' : 'copter';
  }
  modeOk(v) { const fam = this.familyOf(v); return !!(fam && (GOOD_MODES[fam] || []).includes(v.modeName)); }

  setPhase(p, info) { this.phase = p; this.since = Date.now(); this.info = info || ''; this.emit('phase', { phase: p, info: this.info }); }
  note(level, text) { this.notes.push({ t: Date.now(), level, text }); }

  start(opts = {}) {
    if (RUNNING.has(this.phase)) throw new Error('a session is already running');
    this.reset(); Object.assign(this.opts, opts);
    if (this.link.alive) {
      this.sysid = this.link.target.sysid;
      // a timestamp source for marking: most mirrors already carry ATTITUDE; ask for SYSTEM_TIME at 1 Hz as well
      this.link.setMessageRate(2, 1);
      this.setPhase('flight', 'take off and hover; the 30 s capture starts by itself');
    } else this.setPhase('flight', 'no telemetry: connect first, or press "No telemetry: go to USB" after the flight');
    this._tick();
  }
  stop() { this._stopTimer(); try { this.dl.end(); } catch (e) { /* ignore */ } this.setPhase('stopped', 'stopped by you'); }
  captureAgain() { if (!['captured', 'flight'].includes(this.phase)) throw new Error('not in flight'); this.mark = null; this._tFlying = 0; this.setPhase('flight', 'hover again; the capture starts by itself'); }
  /** go to the USB step now: after landing, or without telemetry (the log's longest hover is then used) */
  goUsb({ useSaved = false } = {}) {
    if (this.phase === 'capture') throw new Error('wait for the capture to finish');
    if (this.link.vehicle && this.link.vehicle.armed && this.link.alive) throw new Error('land and disarm first');
    if (useSaved && this.savedMark && !this.mark) { this.mark = this.savedMark; this.note('ok', 'using the 30 s marked at ' + new Date(this.mark.savedAt).toLocaleTimeString()); }
    return this._enterUsb();
  }

  _stopTimer() { if (this._timer) clearTimeout(this._timer); this._timer = null; }
  _tick() {
    this._stopTimer();
    const loop = async () => { try { await this._step(); } catch (e) { this.error = e.message; this.note('bad', e.message); this.setPhase('error', e.message); } this._timer = RUNNING.has(this.phase) ? setTimeout(loop, this.phase === 'usb' ? 1500 : 500) : null; };
    this._timer = setTimeout(loop, 200);
  }

  async _step() {
    const v = this.link.vehicle; const armed = !!(v && v.armed && this.link.alive);
    const el = (Date.now() - this.since) / 1000;
    switch (this.phase) {
      case 'flight': {
        if (!this.link.alive) { this.info = 'waiting for telemetry (Mission Planner mirror or radio)'; return; }
        if (!armed) { this._tFlying = 0; this.info = `arm, take off and hover in ${this._modesText()}`; return; }
        if (!this.modeOk(v)) { this._tFlying = 0; this.info = `flying in ${v.modeName}: switch to ${this._modesText()}`; return; }
        if (this.nowBootMs() === null) { this.info = 'no time stamps over the telemetry yet (waiting for ATTITUDE / SYSTEM_TIME)'; this.link.setMessageRate(2, 1); return; }
        if (!this._tFlying) this._tFlying = Date.now();
        if (Date.now() - this._tFlying < 4000) { this.info = `hovering in ${v.modeName}: the capture starts in a moment`; return; }
        this.captureStart = { ms: this.nowBootMs(), mode: v.modeName, family: this.familyOf(v) };
        this.setPhase('capture', `capturing: short, sharp stick taps on ${this.opts.axes.join(', ')}, hover in between`); return;
      }
      case 'capture': {
        const w = this.opts.seconds;
        if (!armed) { this.setPhase('flight', 'the vehicle disarmed during the capture: take off again'); return; }
        if (!this.modeOk(v)) { this.setPhase('flight', `left ${this.captureStart.mode} during the capture: switch back, it starts again`); this._tFlying = 0; return; }
        if (el < w) { this.info = `capturing: ${Math.ceil(w - el)} s left, keep tapping ${this.opts.axes.join(', ')}`; return; }
        const t1 = this.nowBootMs() ?? (this.captureStart.ms + w * 1000);
        this.mark = { tStart: this.captureStart.ms / 1000, tEnd: t1 / 1000, mode: this.captureStart.mode, family: this.captureStart.family, sysid: this.sysid, axes: this.opts.axes, savedAt: Date.now() };
        this.saveMark();
        this.note('ok', `captured ${w} s in ${this.mark.mode} (autopilot time ${this.mark.tStart.toFixed(0)}-${this.mark.tEnd.toFixed(0)} s)`);
        this.setPhase('captured', 'captured: land and disarm, then plug the USB cable into the autopilot'); return;
      }
      case 'captured': { if (!armed && this.link.alive) return this._enterUsb(); else if (!this.link.alive) this.info = 'captured; telemetry lost. After landing press "Landed: go to USB"'; return; }
      case 'usb': return this._scanUsb();
      default: return;
    }
  }
  _modesText() { return 'QStabilize / QHover / QLoiter (QuadPlane), Stabilize / AltHold / Loiter (copter) or FBWA (plane)'; }

  async _enterUsb() {
    this._knownPorts = new Set((await serialMap()).map(p => p.port));
    this.usbSince = Date.now();
    this.setPhase('usb', this.opts.usb ? 'connecting to the test port' : 'plug the USB cable into the autopilot (the battery may stay connected)');
    if (!this._timer) this._tick();
  }

  async _scanUsb() {
    // test hook: a fixed "USB" link (e.g. a SITL TCP port)
    let cands;
    if (this.opts.usb) cands = [Object.assign({ label: 'test port' }, this.opts.usb)];
    else {
      const ports = await serialMap();
      const usbser = ports.filter(p => /USBSER/i.test(p.dev));
      const fresh = ports.filter(p => !this._knownPorts.has(p.port) && !/Bth/i.test(p.dev));
      cands = [...usbser, ...fresh.filter(p => !usbser.includes(p))].filter(p => !this._busy.has(p.port) && !this._tried.has(p.port))
        .map(p => ({ kind: 'serial', port: p.port, baud: 115200, label: p.port }));
      if (!cands.length) {
        // nothing new: after a while try the remaining ports once more (a port seen at the start may be the autopilot)
        if (Date.now() - this.usbSince > 20000 && this._tried.size) this._tried.clear();
        this.info = 'plug the USB cable into the autopilot' + (this._busy.size ? ` (in use by another program: ${[...this._busy].join(', ')})` : ''); return;
      }
    }
    for (const c of cands) {
      this.info = `trying ${c.label}…`;
      let status = ''; const onSt = s => { status = s; }; this.link.on('status', onSt);
      this.link.connect(c);
      const t0 = Date.now(); let ok = false;
      while (Date.now() - t0 < 4500) { await new Promise(r => setTimeout(r, 200)); if (this.link.alive) { ok = true; break; } if (/cannot open/.test(status)) break; }
      this.link.off('status', onSt);
      if (ok) {
        this.usb = { label: c.label, kind: c.kind };
        if (this.sysid !== null && this.link.target.sysid !== this.sysid) this.note('warn', `the USB vehicle has system id ${this.link.target.sysid}, the flown one had ${this.sysid}`);
        this.note('ok', `connected over USB (${c.label})`);
        this._stopTimer(); this.setPhase('download', 'reading the log list');
        this._download().catch(e => { this.error = e.message; this.note('bad', e.message); this.setPhase('error', e.message); });
        return;
      }
      if (/cannot open/.test(status)) this._busy.add(c.port); else if (c.port) this._tried.add(c.port);
      this.link.disconnect();
      await new Promise(r => setTimeout(r, 500));
    }
  }

  async _download() {
    if (this.link.vehicle && this.link.vehicle.armed) throw new Error('the vehicle is armed: disarm first');
    const logs = (await this.dl.list()).filter(l => l.size > 0);
    if (!logs.length) throw new Error('the autopilot reports no logs (is an SD card in it, and LOG_BACKEND_TYPE on?)');
    const m = this.mark;
    let got = null, used = null;
    for (const log of logs.slice().reverse().slice(0, 4)) {   // newest first; a reboot after landing does not start a new log, but check
      this.info = `log ${log.id} (${(log.size / 1048576).toFixed(1)} MB)`;
      try {
        got = await this.dl.fetchWindow(log, m ? m.tStart : null, m ? m.tEnd : null, { onProgress: p => { this.progress = p; this.info = `${p.stage}: ${p.total > 1000 ? (p.done / 1048576).toFixed(1) + ' / ' + (p.total / 1048576).toFixed(1) + ' MB' : p.done + ' / ' + p.total}`; } });
        used = log; break;
      } catch (e) { if (e.code === 'after' || e.code === 'before') { this.note('warn', `log ${log.id}: ${e.message}, trying the one before`); continue; } throw e; }
    }
    this.dl.end();
    if (!got) throw new Error('none of the newest logs covers the marked 30 s');
    fs.mkdirSync(this.dir, { recursive: true });
    const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    const file = path.join(this.dir, `${stamp} log${used.id}${got.whole ? '' : ' (marked window)'}.bin`);
    fs.writeFileSync(file, got.buf); this.files.bin = file;
    this.note('ok', `downloaded ${(got.bytes / 1048576).toFixed(1)} MB of log ${used.id} (${(used.size / 1048576).toFixed(1)} MB in total)`);
    this.setPhase('download', 'analysing');
    await new Promise(r => setImmediate(r));
    const r = analyzeLog(file, { preset: this.opts.preset, family: m ? m.family : null, tStart: m ? m.tStart : null, tEnd: m ? m.tEnd : null });
    for (const ax of ['roll', 'pitch', 'yaw']) if (r.axes[ax] && !r.axes[ax].error) { try { r.axes[ax].issues = axisIssues(r.axes[ax], r.family); } catch (e) { r.axes[ax].issues = []; } }
    const chosen = new Set(m ? m.axes : this.opts.axes);
    const axes = Object.fromEntries(Object.entries(r.axes).filter(([ax]) => chosen.has(ax)));
    this.rows = proposalRows(r.family, axes, r.noise);
    this.result = r; this.result.chosenAxes = [...chosen];
    // current values on the vehicle (the log holds the values at the time of the flight)
    for (const row of this.rows) { try { row.vehicle = await this.link.requestParam(row.name, 1200, 2); } catch (e) { row.vehicle = undefined; } }
    this.setPhase('review', this.rows.length ? `${this.rows.length} change(s) proposed: check them, then press Write to the autopilot` : 'no change proposed: see the issues per axis');
  }

  /** write the chosen rows on the ground; saves the previous values first */
  async write(names) {
    if (this.phase !== 'review' && this.phase !== 'done') throw new Error('nothing to write yet');
    if (!this.link.alive) throw new Error('the autopilot is not connected');
    if (this.link.vehicle && this.link.vehicle.armed) throw new Error('the vehicle is armed: disarm first');
    const rows = this.rows.filter(r => names.includes(r.name));
    if (!rows.length) throw new Error('choose at least one change');
    this.setPhase('writing', 'writing');
    const before = [];
    for (const r of rows) { let cur; try { cur = await this.link.requestParam(r.name, 1200, 2); } catch (e) { cur = undefined; } r.vehicle = cur; if (cur !== undefined) before.push({ group: r.group, name: r.name, value: +(+cur).toPrecision(6) }); }
    const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    fs.mkdirSync(this.dir, { recursive: true });
    this.files.undo = path.join(this.dir, `${stamp} UNDO (values before AP Tuner).param`);
    fs.writeFileSync(this.files.undo, paramFile(before, ['Values on the autopilot before AP Tuner wrote, ' + new Date().toLocaleString(), 'Load this file to go back.']));
    const results = [];
    // notch: enable first; the other notch parameters only exist after a reboot when it was off
    const order = rows.slice().sort((a, b) => (a.name === 'INS_HNTCH_ENABLE' ? -1 : 0) - (b.name === 'INS_HNTCH_ENABLE' ? -1 : 0));
    for (const r of order) {
      if (r.vehicle !== undefined && Math.abs(r.vehicle - r.value) <= 1e-6 * Math.max(1, Math.abs(r.value))) { results.push({ name: r.name, from: r.vehicle, to: r.value, ok: true, same: true }); continue; }
      try {
        const echo = await this.link.setParam(r.name, r.value, 1500, 3);
        const ok = Math.abs(echo - r.value) <= 1e-4 * Math.max(1, Math.abs(r.value));
        results.push({ name: r.name, from: r.vehicle, to: echo, ok });
      } catch (e) {
        const reboot = /^INS_HNTCH_/.test(r.name) && r.name !== 'INS_HNTCH_ENABLE';
        results.push({ name: r.name, from: r.vehicle, to: undefined, ok: false, error: reboot ? 'needs a reboot first (the notch was just switched on)' : 'no answer from the autopilot' });
      }
    }
    this.writes = results;
    const after = rows.map(r => ({ group: r.group, name: r.name, value: r.value, from: r.vehicle }));
    this.files.param = path.join(this.dir, `${stamp} written by AP Tuner.param`);
    fs.writeFileSync(this.files.param, paramFile(after, ['Written to the autopilot by AP Tuner (Telemetry + USB), ' + new Date().toLocaleString()]));
    const bad = results.filter(x => !x.ok);
    this.setPhase('done', bad.length ? `${results.length - bad.length} of ${results.length} written; ${bad.length} need attention (see the list)` : `all ${results.length} written and read back. Unplug the USB and fly to check`);
    return results;
  }

  async undo() {
    if (!this.writes.length) throw new Error('nothing written yet');
    if (this.link.vehicle && this.link.vehicle.armed) throw new Error('the vehicle is armed: disarm first');
    for (const w of this.writes) if (w.ok && !w.same && w.from !== undefined) { try { await this.link.setParam(w.name, w.from, 1500, 3); w.undone = true; } catch (e) { w.undoError = true; } }
    this.setPhase('done', 'previous values written back');
  }

  snapshot() {
    const v = this.link.vehicle;
    const slimAxis = a => a ? { error: a.error || null, issues: a.issues || [], grade: a.diagnostics && a.diagnostics.grade ? a.diagnostics.grade : null, warnings: a.warnings || [], dataInfo: a.dataInfo || null,
      proposal: a.proposal ? { changes: a.proposal.changes, margins: a.proposal.margins, overshootPct: a.proposal.overshootPct } : null } : null;
    return { phase: this.phase, info: this.info, since: this.since, opts: { axes: this.opts.axes, preset: this.opts.preset, seconds: this.opts.seconds }, mark: this.mark, savedMark: this.savedMark,
      mode: v ? v.modeName : null, armed: !!(v && v.armed), modeOk: this.modeOk(v), alive: this.link.alive, linkDesc: this.link.description, usb: this.usb, progress: this.progress,
      result: this.result ? { family: this.result.family, firmware: this.result.firmware, window: this.result.flightWindow, chosenAxes: this.result.chosenAxes, axes: Object.fromEntries(Object.entries(this.result.axes).map(([k, a]) => [k, slimAxis(a)])), noise: this.result.noise ? { available: this.result.noise.available, reason: this.result.noise.reason, recommended: this.result.noise.recommended ? { params: this.result.noise.recommended.params, gyroFilterHz: this.result.noise.recommended.gyroFilterHz, why: this.result.noise.recommended.why } : null } : null } : null,
      rows: this.rows, writes: this.writes, files: this.files, notes: this.notes.slice(-20), error: this.error };
  }
}

module.exports = { GroundTune, serialMap };
