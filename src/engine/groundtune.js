// "Telemetry + USB" session, for vehicles flown on a slow telemetry radio:
//   flight (telemetry only marks the time) -> capture (30 s of stick taps, nothing written)
//   -> land (wait for disarm) -> usb (the pilot plugs the autopilot into this PC; the app finds the port)
//   -> download (log head + the marked 30 s, at the autopilot's full logging rate) -> review (issues and
//   proposed changes) -> write (on the ground, disarmed, each value read back) -> done (undo file kept).
// Without telemetry the pilot can skip straight to USB; the longest hover in the log is used then.
//
// The write (2026-10-17 audit R3: F-25, F-30, F-32, F-33, F-35, F-39, F-43): the review shows every row with its
// full name, the value in the log, the value on the autopilot now, the reason, the checks (type, range, enumeration)
// and whether it needs a reboot; a row whose value on the autopilot is not the one in the log (the change was worked
// out for other values), or from a log of another vehicle or firmware, cannot be written. The list is one proposal
// (id + hash over the connection and the exact values); Write sends both back. The write is one transaction: every
// value read fresh (a value changed since the review stops it), the UNDO file saved, each value written and verified
// exactly; on a failure everything written goes back (verified) and the rest is reported honestly per row. Values that
// act only after a reboot get the reboot workflow (verified after the reboot); Undo is verified value by value.
'use strict';
const fs = require('fs');
const path = require('path');
const EventEmitter = require('events');
const { execFile } = require('child_process');
const { LogDownloader } = require('../mavlink/logdl');
const { analyzeLog } = require('./loganalysis');
const { proposalRows, paramFile, logVehicleCheck } = require('./report');
const { validateParam } = require('../model/parammeta');
const { runTransaction, rebootAndVerify, verifiedSet, writeText, txnText, CHANGED } = require('./paramwrite');
const { sameF32, sessionIdentity, sameVehicle, identityText, makeProposal } = require('./identity');
const { Journal } = require('./journal');
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
  constructor(link, { sessionsDir, journal = null }) {
    super(); this.link = link; this.dir = path.join(sessionsDir, 'ground'); this.dl = new LogDownloader(link);
    this.audit = journal || new Journal(path.join(sessionsDir, 'journal'));
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
    this.identity = null; this.proposal = null; this.logCheck = null; this.rebootPending = []; this.afterReboot = []; this.reboot = null; this.lastTxn = null;
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
    if (this.phase === 'writing' || this.phase === 'rebooting') throw new Error('the app is writing to the autopilot: wait until it has finished');
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
    this.rows = proposalRows(r.family, axes, r.noise, r.params);
    this.result = r; this.result.chosenAxes = [...chosen];
    await this._review();
    this.setPhase('review', this.rows.length ? `${this.rows.length} change(s) proposed: check them, then press Write to the autopilot` : 'no change proposed: see the issues per axis');
  }

  /**
   * The review: the value on the autopilot now for every row, the checks, whether the log is from this vehicle; one
   * proposal (id + hash over this connection and the exact values) that Write has to name.
   */
  async _review() {
    await this.link.waitAutopilot(3000);
    this.identity = sessionIdentity(this.link.identity(), null);
    this.logCheck = logVehicleCheck(this.result, this.identity);
    for (const n of this.logCheck.notes) this.note('warn', n);
    const enablesNotch = this.rows.some(r => r.name === 'INS_HNTCH_ENABLE' && r.value > 0);
    // the values on the autopilot now, read a few at a time (a missing one costs its timeouts only once)
    let next = 0;
    const reader = async () => { while (next < this.rows.length) { const row = this.rows[next++]; try { row.vehicle = await this.link.requestParam(row.name, 1200, 2); } catch (e) { row.vehicle = undefined; } } };
    await Promise.all([reader(), reader(), reader(), reader()]);
    for (const row of this.rows) {
      const v = validateParam(row.name, row.value);
      row.reboot = v.reboot; row.warnings = v.warnings; row.errors = v.errors; row.same = false; row.deferred = false; row.blocked = null;
      if (!v.ok) row.blocked = `${row.name} ${v.errors.join(', ')}`;
      else if (row.vehicle === undefined) {
        // the notch values may only appear once the notch is switched on (after the reboot): they are written then
        if (enablesNotch && /^INS_HNTCH_/.test(row.name) && row.name !== 'INS_HNTCH_ENABLE') row.deferred = true;
        else row.blocked = 'not found on the autopilot';
      } else if (sameF32(row.vehicle, row.value)) row.same = true;
      else if (typeof row.from === 'number' && !sameF32(row.vehicle, row.from)) row.blocked = `the autopilot holds ${fmt(row.vehicle)}, the log had ${fmt(row.from)}: the change was worked out for other values (fly again with the values now on the autopilot)`;
      if (!row.blocked && this.logCheck.problems.length) row.blocked = this.logCheck.problems.join('; ');
    }
    this.proposal = makeProposal({ source: 'log', identity: this.identity, rows: this.rows.map(x => ({ name: x.name, from: x.vehicle, to: x.value })) });
    this.audit.append('identity', { identity: this.identity, via: 'ground review', log: this.logCheck.log, logProblems: this.logCheck.problems });
    this.audit.append('proposal', { proposal: this.proposal.id, hash: this.proposal.hash, source: 'log', file: this.files.bin || null, identity: this.identity.key,
      rows: this.rows.map(x => ({ name: x.name, log: x.from, vehicle: x.vehicle, to: x.value, reboot: x.reboot, blocked: x.blocked, deferred: x.deferred, same: x.same, warnings: x.warnings })) });
  }

  /** read the values on the autopilot again (after a reconnect or a reboot, or to refresh): a new proposal to review */
  async recheck() {
    if (!this.result || !['review', 'done'].includes(this.phase)) throw new Error('nothing to check yet');
    if (!this.link.alive) throw new Error('the autopilot is not connected');
    await this._review();
    this.writes = [];
    this.setPhase('review', 'values read again: check them, then press Write to the autopilot');
  }

  /**
   * Write the chosen rows on the ground: approval = {id, hash} of the reviewed list. One transaction (fresh read, UNDO
   * file, verified writes, rollback on a failure); resolves the rows with their status: verified, already set, refused,
   * different, uncertain, rolled back, not written, after the reboot.
   */
  async write(names, approval = null) {
    if (this.phase !== 'review' && this.phase !== 'done') throw new Error('nothing to write yet');
    const p = this.proposal;
    if (!p || !approval || approval.id !== p.id || approval.hash !== p.hash) {
      this.audit.append('refused', { what: 'ground write', proposal: p ? p.id : null, approval, why: 'proposal id / hash mismatch' });
      throw new Error('the list changed since it was shown: look at it again, then press Write');
    }
    if (!this.link.alive) throw new Error('the autopilot is not connected');
    const id = this.link.identity();
    if (p.gen !== this.link.generation || p.sysid !== id.sysid || p.compid !== id.compid) {
      this.audit.append('refused', { what: 'ground write', proposal: p.id, why: 'another connection', now: id });
      throw new Error('the connection changed since the review: press "Read the values again", check them, then Write');
    }
    if (this.link.vehicle && this.link.vehicle.armed) throw new Error('the vehicle is armed: disarm first');
    if (!this.audit.ok) throw new Error(`the write journal cannot be written (${this.audit.error}): nothing is written. Free some disk space and restart AP Tuner`);
    const rows = this.rows.filter(r => names.includes(r.name));
    if (!rows.length) throw new Error('choose at least one change');
    const blocked = rows.filter(r => r.blocked);
    if (blocked.length) throw new Error('cannot be written: ' + blocked.map(r => `${r.name} (${r.blocked})`).join('; '));
    this.audit.append('approval', { proposal: p.id, hash: p.hash, by: 'user', via: 'ground write', names, identity: this.identity ? this.identity.key : null });
    this.setPhase('writing', 'writing');
    const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    // the notch switch first; values that only exist once the notch is on wait for the reboot
    const now = rows.filter(r => !r.deferred).sort((a, b) => (a.name === 'INS_HNTCH_ENABLE' ? -1 : 0) - (b.name === 'INS_HNTCH_ENABLE' ? -1 : 0));
    const later = rows.filter(r => r.deferred);
    const txn = await runTransaction(this.link, now.map(r => ({ name: r.name, to: r.value, expect: r.vehicle, reason: r.label })), {
      journal: (k, d) => this.audit.append(k, d), ctx: { proposal: p.id, hash: p.hash, identity: this.identity ? this.identity.key : null, via: 'ground write' },
      // the UNDO file is on disk before the first value is written (otherwise nothing is written)
      afterPreRead: before => {
        fs.mkdirSync(this.dir, { recursive: true });
        this.files.undo = path.join(this.dir, `${stamp} UNDO (values before AP Tuner).param`);
        const undoRows = before.map(b => ({ group: (rows.find(r => r.name === b.name) || {}).group, name: b.name, value: b.before }));
        fs.writeFileSync(this.files.undo, paramFile(undoRows, ['Values on the autopilot before AP Tuner wrote, ' + new Date().toLocaleString(), 'Load this file to go back.'], { identity: this.identity, strict: false }));
      },
    });
    this.lastTxn = txn;
    this.writes = rows.map(r => this._rowResult(r, txn));
    // what acts only after a reboot (written and read back) and what can only be written after it
    this.rebootPending = this.writes.filter(w => w.reboot && w.status === 'verified').map(w => ({ name: w.name, value: w.to }));
    this.afterReboot = txn.ok ? later.map(r => ({ name: r.name, value: r.value, group: r.group, label: r.label, reboot: !!r.reboot })) : [];
    const written = this.writes.filter(w => w.status === 'verified');
    if (written.length) {
      try {
        this.files.param = path.join(this.dir, `${stamp} written by AP Tuner.param`);
        fs.writeFileSync(this.files.param, paramFile(written.map(w => ({ group: w.group, name: w.name, value: w.to, from: w.from })), ['Written to the autopilot by AP Tuner (Telemetry + USB) and read back, ' + new Date().toLocaleString()], { identity: this.identity }));
      } catch (e) { delete this.files.param; this.note('warn', 'the file of the written values could not be saved: ' + e.message); }
    }
    const pend = [...this.rebootPending.map(x => x.name), ...this.afterReboot.map(x => x.name)];
    const tail = pend.length ? ` ${pend.join(', ')} ${pend.length === 1 ? 'takes' : 'take'} effect only after a reboot: press Reboot (disarmed)` : ' Unplug the USB and fly to check';
    this.setPhase('done', txn.ok ? `all ${rows.length - later.length} written and read back.${tail}` : txnText(txn));
    return this.writes;
  }

  /** one row's outcome of a transaction, in plain words */
  _rowResult(r, txn) {
    const base = { group: r.group, name: r.name, label: r.label, requested: r.value, reboot: !!r.reboot, from: undefined, to: undefined, status: 'not written', ok: false, text: '', changed: false, undo: null };
    if (r.deferred) return Object.assign(base, { status: 'after reboot', text: `${r.name}: written after the reboot (it appears once the notch is on)` });
    const res = txn.rows.find(x => x.name === r.name);
    if (!res) return Object.assign(base, { text: txn.aborted ? 'not written: ' + txn.aborted : `${r.name}: not written (an earlier value failed)` });
    const rb = txn.rollback.find(x => x.name === r.name);
    const out = Object.assign(base, { from: res.before, to: res.holds, status: res.status, ok: res.ok, text: writeText(res) });
    if (!txn.ok && rb) {
      if (rb.ok) Object.assign(out, { status: 'rolled back', ok: false, to: rb.holds, text: `${writeText(res)}; written back to ${fmt(rb.holds)} (read back)` });
      else Object.assign(out, { status: 'uncertain', ok: false, to: rb.holds, text: `${writeText(res)}; writing it back failed: ${writeText(rb)}` });
    } else if (!txn.ok && !rb && res.ok && txn.uncertain.some(u => u.name === r.name)) Object.assign(out, { status: 'uncertain', ok: false, text: txn.uncertain.find(u => u.name === r.name).why });
    out.changed = CHANGED.has(out.status) && !(out.status === 'verified' && sameF32(out.to, out.from));
    return out;
  }

  /**
   * Reboot the autopilot (disarmed) after values that act only after a reboot: it must come back as the same vehicle,
   * every value written in this session is read again; values that only exist after it are written then (verified).
   */
  async rebootNow() {
    if (this.phase !== 'done') throw new Error('nothing written yet');
    if (!this.rebootPending.length && !this.afterReboot.length) throw new Error('nothing written here needs a reboot');
    // what the vehicle holds now: the value written, or after an Undo the value from before
    const expect = this.writes.filter(w => w.status === 'verified').map(w => ({ name: w.name, value: w.undo && typeof w.undoHolds === 'number' ? w.undoHolds : w.to }));
    this.setPhase('rebooting', 'rebooting the autopilot: it disconnects for a few seconds');
    const res = await rebootAndVerify(this.link, expect, { journal: (k, d) => this.audit.append(k, d), ctx: { via: 'ground write', identity: this.identity ? this.identity.key : null }, onInfo: t => { this.info = t; } });
    this.reboot = { at: Date.now(), ok: res.ok, reason: res.reason, rows: res.rows };
    for (const w of this.writes) { const x = res.rows.find(y => y.name === w.name); if (x) w.afterReboot = x.ok ? 'read back after the reboot' : (x.holds === undefined ? 'no answer after the reboot' : `holds ${fmt(x.holds)} after the reboot`); }
    if (!res.ok) { this.setPhase('done', 'reboot: ' + res.reason); return this.reboot; }
    this.rebootPending = [];
    // the identity of the new connection (the link may have reopened): the same vehicle was checked by rebootAndVerify
    this.identity = sessionIdentity(this.link.identity(), null);
    if (this.afterReboot.length) {
      const later = this.afterReboot; this.afterReboot = [];
      const txn = await runTransaction(this.link, later.map(r => ({ name: r.name, to: r.value, reason: r.label })), { journal: (k, d) => this.audit.append(k, d), ctx: { via: 'ground write after the reboot', identity: this.identity.key } });
      for (const r of later) { const w = this.writes.find(x => x.name === r.name); const nw = this._rowResult(Object.assign({}, r, { deferred: false }), txn); if (w) Object.assign(w, nw); }
      const pend = this.writes.filter(w => w.reboot && w.status === 'verified' && later.some(l => l.name === w.name)).map(w => ({ name: w.name, value: w.to }));
      this.rebootPending = pend;
      this.setPhase('done', txn.ok ? `rebooted and read back; then ${later.length} notch value(s) written and read back${pend.length ? ': ' + pend.map(x => x.name).join(', ') + ' need one more reboot (press Reboot)' : ''}` : 'after the reboot: ' + txnText(txn));
      return this.reboot;
    }
    this.setPhase('done', 'rebooted: every value written here was read back after the reboot. Unplug the USB and fly to check');
    return this.reboot;
  }

  /** write the values from before back, each verified on its own (an undo goes as far back as it can) */
  async undo() {
    const todo = this.writes.filter(w => w.changed && typeof w.from === 'number' && w.undo !== 'undone');
    if (!todo.length) throw new Error('nothing to undo: no value written here is different from before');
    if (!this.link.alive) throw new Error('the autopilot is not connected');
    if (this.link.vehicle && this.link.vehicle.armed) throw new Error('the vehicle is armed: disarm first');
    if (this.identity && !sameVehicle(this.identity, this.link.identity())) throw new Error('another vehicle is connected now (' + identityText(this.link.identity()) + '): the undo values are for ' + identityText(this.identity));
    this.setPhase('writing', 'writing the previous values back');
    this.audit.append('approval', { by: 'user', via: 'ground undo', names: todo.map(w => w.name), identity: this.identity ? this.identity.key : null });
    for (const w of todo) {
      let cur; try { cur = await this.link.requestParam(w.name, 1200, 2); } catch (e) { cur = undefined; }
      let r;
      if (sameF32(cur, w.from)) r = { name: w.name, status: 'unchanged', ok: true, holds: cur };
      else r = await verifiedSet(this.link, w.name, w.from, { before: cur });
      this.audit.append('write', Object.assign({ via: 'ground undo', identity: this.identity ? this.identity.key : null }, r));
      w.undo = r.ok ? 'undone' : r.status; w.undoText = r.ok ? `back to ${fmt(r.holds)} (read back)` : writeText(r); w.undoHolds = r.holds;
      w.afterReboot = null;   // the last reboot's check was about the value written, not this one
      if (r.ok && w.reboot && r.status === 'verified') this.rebootPending = this.rebootPending.filter(x => x.name !== w.name).concat([{ name: w.name, value: r.holds }]);
    }
    const bad = todo.filter(w => w.undo !== 'undone');
    this.setPhase('done', bad.length ? `undo incomplete: ${bad.map(w => w.undoText).join('; ')}. Load the UNDO file in Mission Planner to go back` : 'previous values written back and read back' + (this.rebootPending.length ? `; ${this.rebootPending.map(x => x.name).join(', ')} ${this.rebootPending.length === 1 ? 'takes' : 'take'} effect after a reboot (press Reboot)` : ''));
  }

  snapshot() {
    const v = this.link.vehicle;
    const slimAxis = a => a ? { error: a.error || null, issues: a.issues || [], grade: a.diagnostics && a.diagnostics.grade ? a.diagnostics.grade : null, warnings: a.warnings || [], dataInfo: a.dataInfo || null,
      proposal: a.proposal ? { changes: a.proposal.changes, margins: a.proposal.margins, overshootPct: a.proposal.overshootPct } : null } : null;
    return { phase: this.phase, info: this.info, since: this.since, opts: { axes: this.opts.axes, preset: this.opts.preset, seconds: this.opts.seconds }, mark: this.mark, savedMark: this.savedMark,
      mode: v ? v.modeName : null, armed: !!(v && v.armed), modeOk: this.modeOk(v), alive: this.link.alive, linkDesc: this.link.description, usb: this.usb, progress: this.progress,
      result: this.result ? { family: this.result.family, firmware: this.result.firmware, window: this.result.flightWindow, chosenAxes: this.result.chosenAxes, axes: Object.fromEntries(Object.entries(this.result.axes).map(([k, a]) => [k, slimAxis(a)])), noise: this.result.noise ? { available: this.result.noise.available, reason: this.result.noise.reason, recommended: this.result.noise.recommended ? { params: this.result.noise.recommended.params, gyroFilterHz: this.result.noise.recommended.gyroFilterHz, why: this.result.noise.recommended.why } : null } : null } : null,
      rows: this.rows, writes: this.writes, files: this.files, notes: this.notes.slice(-20), error: this.error,
      proposal: this.proposal ? { id: this.proposal.id, hash: this.proposal.hash } : null, identity: this.identity, identityText: this.identity ? identityText(this.identity) : null,
      logCheck: this.logCheck, rebootPending: this.rebootPending, afterReboot: this.afterReboot, reboot: this.reboot, audit: this.audit.status() };
  }
}

function fmt(v) { return typeof v === 'number' ? +v.toPrecision(6) : v; }

module.exports = { GroundTune, serialMap };
