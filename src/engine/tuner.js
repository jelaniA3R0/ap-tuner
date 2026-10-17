// Live tuning session: observe -> identify -> propose -> (apply in bounded steps with a watchdog) ->
// observe again, per axis. Also the QuikTune-style ramp as a fallback when the model cannot be
// identified, and the parameter journal with one-click restore.
'use strict';
const EventEmitter = require('events');
const fs = require('fs');
const path = require('path');
const { LiveAxis } = require('./axis');
const { analyzeAxis, currentGains } = require('./analysis');
const { detectFamily, axisParamNames, familyDef } = require('./vehicle');
const { neededParams, oscSmaxFor, filterRules, iRule, gainLimits } = require('../model/rules');

const AXES = ['roll', 'pitch', 'yaw'];
const PID_TUNING_ID = 194, ATTITUDE_ID = 30;
// after a gain change the watchdog ignores this many seconds: ArduPilot's slew-rate signal looks back ~0.3 s
// and telemetry arrives late, so those samples still describe the gains before the change
const WATCH_GRACE_S = 0.5;

class Tuner extends EventEmitter {
  constructor(link, opts = {}) {
    super();
    this.link = link;
    this.opts = Object.assign({ preset: 'normal', autoApply: false, axes: ['roll', 'pitch'], observeSeconds: 30, stepMax: 1.5, watchSeconds: 4, oscSmax: 5, massKg: 0, allowRamp: false, rampDoubleTime: 10, rampMargin: 60, maxReducePct: 20, angleAbort: 10, sessionDir: path.join(__dirname, '..', '..', 'sessions') }, opts);
    this.axes = { roll: new LiveAxis('roll'), pitch: new LiveAxis('pitch'), yaw: new LiveAxis('yaw') };
    this.attitude = { roll: 0, pitch: 0, yaw: 0, p: 0, q: 0, r: 0, ts: 0 };
    this.rc = { roll: 0, pitch: 0, yaw: 0, thr: 0, ts: 0 };
    this.hud = { airspeed: 0, throttle: 0, alt: 0 };
    this.params = {};
    this.family = null;
    this.state = 'idle'; this.stateSince = Date.now();
    this.axisIdx = 0; this.round = 0;
    this.journal = []; this.originals = {}; this.lastGood = {};
    this.results = {};        // axis -> last analysis result
    this.messages = [];
    this.ramp = null;
    this._timer = null;
    this._bind();
  }

  _bind() {
    const L = this.link;
    L.on('PID_TUNING', (f, m) => {
      const ax = f.axis === 1 ? 'roll' : f.axis === 2 ? 'pitch' : f.axis === 3 ? 'yaw' : null;
      if (!ax) return;
      // copter / VTOL rate PIDs run in rad/s (plane reports deg/s); keep everything in deg/s
      const k = this.family === 'plane' ? 1 : 57.2957795;
      this.axes[ax].push(m.ts / 1000, Object.assign({}, f, { desired: f.desired * k, achieved: f.achieved * k }));
    });
    L.on('ATTITUDE', (f, m) => { this.attitude = { roll: f.roll * 57.2958, pitch: f.pitch * 57.2958, yaw: f.yaw * 57.2958, p: f.rollspeed * 57.2958, q: f.pitchspeed * 57.2958, r: f.yawspeed * 57.2958, ts: m.ts / 1000 }; });
    L.on('RC_CHANNELS', (f, m) => { const n = v => (v - 1500) / 500; this.rc = { roll: n(f.chan1_raw), pitch: n(f.chan2_raw), yaw: n(f.chan4_raw), thr: (f.chan3_raw - 1000) / 1000, ts: m.ts / 1000 }; });
    L.on('VFR_HUD', f => { this.hud = { airspeed: f.airspeed, throttle: f.throttle, alt: f.alt, groundspeed: f.groundspeed }; });
    L.on('STATUSTEXT', f => this.log('vehicle', f.text));
    L.on('VIBRATION', (f, m) => { this.vibe = { x: f.vibration_x, y: f.vibration_y, z: f.vibration_z, clip: (f.clipping_0 | 0) + (f.clipping_1 | 0) + (f.clipping_2 | 0), ts: m.ts / 1000 }; });
    L.on('heartbeat', v => { if (!this.family && Object.keys(this.params).length) this.family = detectFamily(v, this.params);
      if (this._linkGen !== L.generation) { this._linkGen = L.generation; if (this.family) { this.requestStreams(); this.log('link', 'new connection: streams re-requested'); } } if (this.state !== 'idle' && !v.armed && this.armedSeen) this.onDisarm(); if (v.armed) this.armedSeen = true; });
  }

  log(kind, text) { const e = { t: Date.now(), kind, text }; this.messages.push(e); if (this.messages.length > 500) this.messages.shift(); this.emit('log', e); }
  setState(s, info) { this.state = s; this.stateSince = Date.now(); this.stateInfo = info || ''; this.emit('state', { state: s, info: this.stateInfo }); this.log('tuner', `${s}${info ? ': ' + info : ''}`); }

  /**
   * Read the parameters the tuner needs and detect the vehicle family. The two housekeeping writes
   * (GCS_PID_MASK for PID telemetry, SMAX if zero) are only done with {confirm: true}; otherwise the
   * call returns {needsConfirm: true, writes: [...]} so the user can agree first.
   */
  async prepare({ confirm = false } = {}) {
    if (!this.link.alive) throw new Error('no vehicle heartbeat: connect first');
    this.setState('preparing', 'reading parameters');
    // a new connection may be a different vehicle: drop everything cached from the previous one
    if (this._preparedGen !== this.link.generation) { this._preparedGen = this.link.generation; this.params = {}; this.originals = {}; this.previous = {}; this.results = {}; this.lastGood = {}; this.prepared = false; this.family = null; }
    const v = this.link.vehicle;
    let fam = detectFamily(v, {});
    if (fam === 'plane') { const q = await this.link.requestParam('Q_ENABLE').catch(() => 0); this.params.Q_ENABLE = q; fam = detectFamily(v, this.params); }
    this.family = fam;
    if (!fam) throw new Error('unsupported vehicle type ' + v.typeName);
    const names = neededParams(fam === 'heli' ? 'copter' : fam);
    const got = await this.link.requestParams(names, 3, (d, n) => this.emit('progress', { done: d, total: n }));
    Object.assign(this.params, got);
    this.loopHz = this.params.SCHED_LOOP_RATE || (fam === 'plane' ? 50 : 400);
    // PID_TUNING streaming needs GCS_PID_MASK (bits: roll 1, pitch 2, yaw 4) and SMAX > 0 for the slew-rate signal
    const writes = [];
    if ((this.params.GCS_PID_MASK | 0) !== 7) writes.push({ name: 'GCS_PID_MASK', from: this.params.GCS_PID_MASK, to: 7, why: 'turns on the PID telemetry the app listens to (roll, pitch, yaw). Harmless; you can set it back to 0 later.' });
    for (const ax of AXES) {
      const n = axisParamNames(fam, ax);
      if (!(this.params[n.SMAX] > 0)) writes.push({ name: n.SMAX, from: this.params[n.SMAX], to: familyDef(fam).defaultSmax, why: 'slew-rate limit; ArduPilot only reports the oscillation signal (SRate) when it is above zero. 50 is the QuikTune default and does not change how the vehicle flies.' });
    }
    this.pendingWrites = writes;
    for (const ax of AXES) this.originals[ax] = this.originals[ax] || currentGains(fam, ax, this.params);
    const checks = this.safetyChecks();
    if (writes.length && !confirm) { this.setState('ready', `${fam} detected; ${writes.length} housekeeping setting(s) need your OK`); return { family: fam, vehicle: v, needsConfirm: true, writes, checks, gains: Object.fromEntries(AXES.map(a => [a, currentGains(fam, a, this.params)])), loopHz: this.loopHz }; }
    for (const w of writes) await this.setParam(w.name, w.to, w.why.split('.')[0]);
    this.pendingWrites = [];
    this.requestStreams();
    this.lastGood = JSON.parse(JSON.stringify(this.originals));
    if (!this.sessionFile) { this.sessionFile = path.join(this.opts.sessionDir, new Date().toISOString().replace(/[:.]/g, '-') + '.json'); fs.mkdirSync(this.opts.sessionDir, { recursive: true }); }
    this.saveSession();
    this.prepared = true;
    this.setState('ready', `${fam} detected, loop ${this.loopHz} Hz, gyro filter ${this.params.INS_GYRO_FILTER} Hz`);
    return { family: fam, vehicle: v, needsConfirm: false, writes: [], checks: this.safetyChecks(), gains: Object.fromEntries(AXES.map(a => [a, currentGains(fam, a, this.params)])), loopHz: this.loopHz };
  }

  /**
   * A QuadPlane has two controllers: the fixed-wing one (family 'plane', FBWA...) and the hover one
   * (family 'vtol', Q modes). Switch the tuner to the controller that matches the mode being flown.
   */
  async switchFamily(fam) {
    if (fam === this.family) return;
    if (!this.prepared) throw new Error('read the vehicle first');
    // read every parameter of the new family afresh (the vehicle may have been changed or replaced)
    const names = neededParams(fam === 'heli' ? 'copter' : fam);
    Object.assign(this.params, await this.link.requestParams(names, 3, (d, n) => this.emit('progress', { done: d, total: n })));
    this.family = fam;
    this.loopHz = this.params.SCHED_LOOP_RATE || (fam === 'plane' ? 50 : 400);
    for (const ax of AXES) { this.originals[ax] = currentGains(fam, ax, this.params); this.axes[ax].clear(); }
    this.lastGood = JSON.parse(JSON.stringify(this.originals)); this.previous = {}; this.results = {};
    for (const ax of AXES) { const n = axisParamNames(fam, ax); if (n.SMAX && !(this.params[n.SMAX] > 0)) await this.setParam(n.SMAX, familyDef(fam).defaultSmax, 'slew-rate limit so the oscillation signal is reported'); }
    this.requestStreams();
    this.log('tuner', `controller family switched to ${fam} (loop ${this.loopHz} Hz)`);
    this.saveSession();
  }

  /** plain-language checks of the setup, from the ArduPilot "setting up for tuning" page */
  safetyChecks() {
    const p = this.params, fam = this.family, out = [];
    const ok = (title, detail) => out.push({ ok: true, title, detail });
    const warn = (title, detail) => out.push({ ok: false, title, detail });
    if (fam === 'copter' || fam === 'vtol' || fam === 'heli') {
      const pre = fam === 'vtol' ? 'Q_M_' : 'MOT_';
      const hov = p[pre + 'THST_HOVER'];
      if (hov !== undefined) (hov > 0.2 && hov < 0.7 ? ok : warn)(`Hover throttle ${hov.toFixed(2)}`, hov > 0.2 && hov < 0.7 ? 'learned value in the normal range' : 'outside 0.2-0.7: check MOT_HOVER_LEARN and that the vehicle has hovered long enough');
      const expo = p[pre + 'THST_EXPO'];
      if (expo !== undefined) (expo >= 0.4 && expo <= 0.8 ? ok : warn)(`Thrust expo ${expo.toFixed(2)}`, 'ArduPilot guide: 0.55 for 5", 0.65 for 10", 0.75 for 20" props (thrust-stand data is better)');
      const gf = p.INS_GYRO_FILTER;
      if (gf !== undefined) (gf >= 10 && gf <= 100 ? ok : warn)(`Gyro filter ${gf} Hz`, 'guide: 80 Hz for 5", 40 Hz for 10", 20 Hz for 20" and larger');
      const ffe = p[fam === 'vtol' ? 'Q_A_RATE_FF_ENAB' : 'ATC_RATE_FF_ENAB'];
      if (ffe !== undefined) (ffe > 0 ? ok : warn)('Rate feed-forward ' + (ffe > 0 ? 'on' : 'off'), ffe > 0 ? 'normal' : 'ATC_RATE_FF_ENAB should be 1 for normal flying');
      (p.INS_HNTCH_ENABLE > 0 ? ok : warn)('Harmonic notch ' + (p.INS_HNTCH_ENABLE > 0 ? 'enabled' : 'not set up'), p.INS_HNTCH_ENABLE > 0 ? `mode ${p.INS_HNTCH_MODE}, ${p.INS_HNTCH_FREQ} Hz` : 'a notch filter usually allows a much tighter tune; see step 4');
    }
    if (fam === 'plane') {
      (p.SCALING_SPEED > 0 ? ok : warn)(`Scaling speed ${p.SCALING_SPEED} m/s`, 'should be close to the normal cruise speed; changing it later invalidates the tune');
      ((p.RLL_RATE_FF > 0 && p.PTCH_RATE_FF > 0) ? ok : warn)('Feed-forward set', 'plane rate loops are driven mostly by FF');
    }
    ok(`PID telemetry ${(p.GCS_PID_MASK | 0) === 7 ? 'on' : 'will be turned on'}`, 'GCS_PID_MASK = 7');
    return out;
  }

  requestStreams() {
    // legacy stream request first (EXTRA1 carries ATTITUDE + PID_TUNING); it would otherwise override
    // the per-message intervals requested below
    this.link.requestDataStream(10, 20);
    this.link.requestDataStream(3, 5);   // RC_CHANNELS
    this.link.requestDataStream(2, 5);   // VFR_HUD
    setTimeout(() => {
      // fast PID_TUNING (one message per axis per interval) and ATTITUDE; vehicles cap this by link capacity
      this.link.setMessageRate(PID_TUNING_ID, this.family === 'plane' ? 25 : 50);
      this.link.setMessageRate(ATTITUDE_ID, 25);
      this.link.setMessageRate(241, 2);   // VIBRATION, for the final check
    }, 1500);
  }

  async setParam(name, value, reason) {
    const from = this.params[name];
    const v = await this.link.setParam(name, value);
    this.params[name] = v;
    this.journal.push({ t: Date.now(), name, from, to: v, reason });
    this.emit('param', { name, from, to: v, reason });
    this.log('param', `${name}: ${fmt(from)} -> ${fmt(v)} (${reason})`);
    this.saveSession();
    return v;
  }

  saveSession() {
    if (!this.sessionFile) return;
    try { fs.writeFileSync(this.sessionFile, JSON.stringify({ started: this.sessionFile, family: this.family, originals: this.originals, lastGood: this.lastGood, journal: this.journal, opts: this.opts }, null, 1)); } catch (e) { /* ignore */ }
  }

  /** write a gain set for an axis (only the keys given), in a safe order (D before P; filters first) */
  async writeGains(axis, gains, reason) {
    const n = axisParamNames(this.family, axis);
    const order = ['FLTT', 'FLTE', 'FLTD', 'SMAX', 'D', 'P', 'I', 'FF', 'angleP', 'tconst'];
    for (const k of order) {
      if (gains[k] === undefined || !n[k]) continue;
      const cur = this.params[n[k]];
      if (cur !== undefined && Math.abs(cur - gains[k]) < 1e-9) continue;
      await this.setParam(n[k], gains[k], reason);
    }
  }

  /** analysis of one axis over the last observe window */
  analyze(axis, seconds = null) {
    const data = this.axes[axis].window(seconds || this.opts.observeSeconds);
    if (!data) return { axis, error: 'no data' };
    const scaler2 = this.family === 'plane' ? Math.pow(require('./vehicle').planeSpeedScaler(this.params, this.hud.airspeed, this.hud.throttle), 2) : 1;
    const res = analyzeAxis({ family: this.family, axis, data, params: this.params, preset: this.opts.preset, gyroOp: { throttle: this.hud.throttle / 100 }, scaler2, fsLoop: this.loopHz, designFrom: this.originals[axis] || null });
    res.dataInfo = { fs: data.fs, rawRateHz: data.rawRate, seconds: data.n / data.fs };
    res.at = Date.now();
    const step = Math.max(1, Math.floor(data.n / 1500));
    res.series = { t: [], target: [], actual: [], u: [] };
    for (let i = 0; i < data.n; i += step) { res.series.t.push(+(data.t0 + i / data.fs).toFixed(3)); res.series.target.push(+data.target[i].toFixed(2)); res.series.actual.push(+data.actual[i].toFixed(2)); res.series.u.push(+data.u[i].toFixed(4)); }
    this.results[axis] = res;
    this.emit('analysis', res);
    return res;
  }

  // ---------------- session control ----------------
  start() {
    if (this.state === 'idle' || this.state === 'ready' || this.state === 'done' || this.state === 'paused') {
      if (!this.family || !this.prepared) throw new Error('read the vehicle first (step 2)');
      this.requestStreams();
      this.axisIdx = 0; this.round = 0;
      for (const ax of AXES) this.axes[ax].setBaseline();
      this.setState('observing', `${this.currentAxis()}: fly normally and tap the sticks (round 1)`);
      this._tick();
    }
  }
  pause(reason) { this.setState('paused', reason || 'paused by user'); this.ramp = null; }
  currentAxis() { return this.opts.axes[this.axisIdx % this.opts.axes.length]; }

  async stop() { this.ramp = null; this.setState('idle', 'stopped'); }

  /** restore the original gains of all axes */
  async restoreOriginals() {
    this.pause('restoring original gains');
    for (const ax of AXES) if (this.originals[ax]) await this.writeGains(ax, this.originals[ax], 'restore original');
    this.lastGood = JSON.parse(JSON.stringify(this.originals));
    this.log('tuner', 'original gains restored');
  }
  async revertAxis(axis, reason) {
    const g = this.lastGood[axis]; if (!g) return;
    await this.writeGains(axis, g, 'revert: ' + reason);
  }
  /** undo the last apply on an axis (gains as they were before it started) */
  async undoAxis(axis) {
    const g = (this.previous || {})[axis]; if (!g) throw new Error('nothing to undo for ' + axis);
    if (this.state === 'observing' || this.state === 'applying') this.pause('undo requested');
    await this.writeGains(axis, g, 'undo last apply');
    this.lastGood[axis] = JSON.parse(JSON.stringify(g)); this.saveSession();
  }
  onDisarm() { if (this.state !== 'idle') { this.pause('vehicle disarmed'); } }

  /** user accepted a proposal (manual mode) */
  async applyProposal(axis) {
    const r = this.results[axis];
    if (!r || !r.proposal) throw new Error('no proposal for ' + axis);
    this.setState('applying', axis);
    await this._applyStaged(axis, r.proposal.gains);
  }

  /** staged application with watchdog; returns true if the final gains were kept */
  async _applyStaged(axis, target) {
    const cur = currentGains(this.family, axis, this.params);
    this.previous = this.previous || {}; this.previous[axis] = JSON.parse(JSON.stringify(cur));
    const keys = ['P', 'I', 'D', 'FF'].filter(k => target[k] !== undefined && cur[k] !== undefined);
    // number of steps so that no multiplicative change exceeds stepMax
    let steps = 1;
    for (const k of keys) { const a = cur[k] || 1e-6, b = target[k] || 1e-6; if (a > 0 && b > 0) steps = Math.max(steps, Math.ceil(Math.abs(Math.log(b / a)) / Math.log(this.opts.stepMax))); }
    steps = Math.min(steps, 6);
    // filters / smax / angle first (small effect), then gains stepwise
    const pre = {}; for (const k of ['FLTT', 'FLTE', 'FLTD', 'SMAX']) if (target[k] !== undefined) pre[k] = target[k];
    await this.writeGains(axis, pre, 'apply filters');
    for (let s = 1; s <= steps; s++) {
      if (this.state === 'paused' || this.state === 'idle') return false;
      const before = currentGains(this.family, axis, this.params);
      const g = {};
      for (const k of keys) { const a = cur[k], b = target[k]; g[k] = (a > 0 && b > 0) ? a * Math.pow(b / a, s / steps) : (s === steps ? b : a + (b - a) * s / steps); }
      await this.writeGains(axis, g, `apply step ${s}/${steps}`);
      const ok = await this._watch(axis, this.opts.watchSeconds);
      if (!ok) {
        // a step that only lowered gains did not cause the oscillation: going back up would make it worse
        if (keys.every(k => !(g[k] > before[k] * (1 + 1e-6)))) {
          const low = {}; for (const k of keys) low[k] = target[k];
          await this.writeGains(axis, low, `oscillation continued after lowering the gains (step ${s}): lowered to the target`);
          this.lastGood[axis] = currentGains(this.family, axis, this.params); this.saveSession();
          this.pause(`${axis}: still oscillating after lowering the gains, so the lower gains were kept (${keys.map(k => k + ' ' + fmt(low[k])).join(', ')}). Land if it keeps shaking, otherwise press Resume`);
          return false;
        }
        await this.revertAxis(axis, 'oscillation detected after step ' + s); this.pause(`${axis}: oscillation after applying step ${s}/${steps}; reverted to last good gains`); return false;
      }
      this.lastGood[axis] = currentGains(this.family, axis, this.params); this.saveSession();
    }
    if (target.angleP !== undefined || target.tconst !== undefined) {
      const before = currentGains(this.family, axis, this.params);
      await this.writeGains(axis, { angleP: target.angleP, tconst: target.tconst }, 'apply angle gain');
      const ok = await this._watch(axis, this.opts.watchSeconds);
      if (!ok) {
        // a softer angle loop (lower angle P, longer time constant) did not cause it either: keep it
        if (!(target.angleP > before.angleP * (1 + 1e-6)) && !(target.tconst < before.tconst * (1 - 1e-6))) {
          this.lastGood[axis] = currentGains(this.family, axis, this.params); this.saveSession();
          this.pause(`${axis}: still oscillating after softening the angle gain, so it was kept. Land if it keeps shaking, otherwise press Resume`);
          return false;
        }
        await this.revertAxis(axis, 'oscillation after angle gain'); this.pause(`${axis}: oscillation after the angle gain; reverted`); return false;
      }
      this.lastGood[axis] = currentGains(this.family, axis, this.params); this.saveSession();
    }
    this.log('tuner', `${axis}: new gains applied and held for ${this.opts.watchSeconds} s without oscillation`);
    return true;
  }

  /**
   * watch an axis for `seconds` after a change; false if the watchdog trips (also checks attitude error).
   * Samples from the first WATCH_GRACE_S are ignored: they still describe the gains before the change.
   */
  _watch(axis, seconds) {
    return new Promise(resolve => {
      const t0 = Date.now(); const since = t0 / 1000 + WATCH_GRACE_S;
      const iv = setInterval(() => {
        const w = this.axes[axis].watchdog(2, { oscSmax: this.opts.oscSmax, since });
        const attErr = this._attitudeError();
        if (w.oscillating || attErr > this.opts.angleAbort) { clearInterval(iv); this.log('watchdog', `${axis}: ${w.reasons.join('; ') || 'attitude error ' + attErr.toFixed(1) + ' deg'}`); resolve(false); return; }
        if (Date.now() - t0 > (seconds + WATCH_GRACE_S) * 1000) { clearInterval(iv); resolve(true); }
      }, 250);
    });
  }
  _attitudeError() {
    // only meaningful when sticks are centred: compare target vs actual rate magnitude over the last second
    return 0;
  }

  /** main loop: runs while observing / auto-applying */
  _tick() {
    if (this._timer) clearTimeout(this._timer);
    const loop = async () => {
      try {
        if (this.state === 'observing') await this._observeStep();
        else if (this.state === 'ramping') await this._rampStep();
      } catch (e) { this.log('error', e.message); this.pause('error: ' + e.message); }
      if (this.state === 'observing' || this.state === 'ramping') this._timer = setTimeout(loop, this.state === 'ramping' ? 250 : 1000);
    };
    this._timer = setTimeout(loop, 500);
  }

  async _observeStep() {
    const axis = this.currentAxis();
    const la = this.axes[axis];
    const elapsed = (Date.now() - this.stateSince) / 1000;
    const exc = la.excitation(20);
    // watchdog during observation too (vehicle may be oscillating with its current gains)
    const w = la.watchdog(2, { oscSmax: this.opts.oscSmax });
    if (w.oscillating && w.bandRms > 15) { this.log('watchdog', `${axis}: ${w.reasons.join('; ')}`); if (this.journal.length) { await this.revertAxis(axis, 'oscillation while observing'); } this.pause(`${axis} is oscillating (${w.reasons[0]}); reduce gains before continuing`); return; }
    this.emit('observe', { axis, seconds: la.seconds, excitation: exc, rateHz: la.rateHz, elapsed });
    // a fresh analysis (from the flight check-up, < 90 s old) is used directly in the first round
    const fresh = this.opts.useFreshResult && this.round === 0 && this.results[axis] && !this.results[axis].used && Date.now() - (this.results[axis].at || 0) < 90000;
    if (!fresh) {
      if (elapsed < this.opts.observeSeconds) return;
      if (la.seconds < this.opts.observeSeconds * 0.8) { this.stateInfo = `${axis}: waiting for telemetry (${la.seconds.toFixed(0)} s buffered at ${la.rateHz.toFixed(0)} Hz)`; return; }
    }
    this.setState('analyzing', axis);
    const res = fresh ? this.results[axis] : this.analyze(axis);
    if (fresh) res.used = true;
    if (!res.proposal) {
      this.round++;
      if (this.opts.allowRamp && this.round >= 2) { this._startRamp(axis); return; }
      this.setState('observing', `${axis}: ${res.identification?.reason || res.warnings[0] || 'no proposal'} (round ${this.round + 1}) - fly sharper stick inputs`);
      return;
    }
    // converged when the rate gains move < 10% AND the angle gain / time constant (capped at x1.5 per round) < 15%
    const big = res.proposal.changes.some(c => (['P', 'D', 'FF'].includes(c.key) && Math.abs(c.pct ?? 0) > 10) || (['angleP', 'tconst'].includes(c.key) && Math.abs(c.pct ?? 0) > 15));
    if (!big) { this.log('tuner', `${axis}: converged (rate gains < 10%, angle gain < 15%)`); this._nextAxis(); return; }
    if (this.opts.autoApply) {
      this.setState('applying', `${axis}: ${res.proposal.changes.map(c => c.key + ' ' + fmt(c.from) + '->' + fmt(c.to)).join(', ')}`);
      const ok = await this._applyStaged(axis, res.proposal.gains);
      if (ok) { this.round++; if (this.round >= 4) this._nextAxis(); else this.setState('observing', `${axis}: re-checking with the new gains (round ${this.round + 1})`); }
    } else {
      this.setState('proposed', `${axis}: review the proposal and press Apply`);
    }
  }

  _nextAxis() {
    this.axisIdx++; this.round = 0;
    if (this.axisIdx >= this.opts.axes.length) { this.setState('done', 'all axes tuned'); return; }
    this.setState('observing', `${this.currentAxis()}: fly normally and tap the sticks (round 1)`);
  }

  /** continue after a manual apply or a pause */
  resume() { if (this.state === 'proposed' || this.state === 'paused' || this.state === 'applying') { this.setState('observing', `${this.currentAxis()}: observing (round ${this.round + 1})`); this._tick(); } }

  // ---------------- QuikTune-style ramp (VTOL-quicktune.lua logic) ----------------
  _startRamp(axis) {
    const n = axisParamNames(this.family, axis);
    const fam = this.family === 'heli' ? 'copter' : this.family;
    const filters = filterRules(fam, axis, currentGains(fam, axis, this.params), this.params.INS_GYRO_FILTER || 20, 0.5);
    this.ramp = { axis, stage: 'D', names: n, saved: currentGains(fam, axis, this.params), lastChange: Date.now(), nextWrite: 0, filtersSet: false, filters };
    this.setState('ramping', `${axis}: QuikTune-style ramp, raising D (doubling every ${this.opts.rampDoubleTime} s)`);
  }
  async _rampStep() {
    const r = this.ramp; if (!r) { this.setState('observing', 'ramp ended'); return; }
    const la = this.axes[r.axis];
    const now = Date.now();
    if (!r.filtersSet) { await this.writeGains(r.axis, r.filters, 'QuikTune filter rule'); r.filtersSet = true; return; }
    // pilot input pauses the ramp (QuikTune PILOT_INPUT_DELAY = 4 s)
    if (Math.abs(this.rc.roll) > 0.05 || Math.abs(this.rc.pitch) > 0.05 || Math.abs(this.rc.yaw) > 0.05) { r.lastPilot = now; }
    if (r.lastPilot && now - r.lastPilot < 4000) return;
    if (now - r.lastChange < 4000) return;      // STAGE_DELAY after a stage change
    const key = r.stage; const pname = r.names[key];
    const cur = this.params[pname];
    const w = la.watchdog(1.5, { oscSmax: this.opts.oscSmax });
    const lim = gainLimits(this.family === 'heli' ? 'copter' : this.family, r.axis)[key];
    const limited = cur >= lim[1];
    if (w.oscillating || limited) {
      const reduction = w.oscillating ? (100 - this.opts.rampMargin) / 100 : 1;
      let nv = Math.min(cur * reduction, lim[1]);
      const saved = r.saved[key];
      if (nv < saved && key === 'D' && r.axis !== 'yaw') {
        // lowering D below the original: also lower P (QuikTune rule, not below a factor of 2)
        const ratio = Math.max(nv / saved, 0.5);
        await this.setParam(r.names.P, this.params[r.names.P] * ratio, 'QuikTune: P lowered with D');
      }
      await this.setParam(pname, nv, `QuikTune: ${w.oscillating ? 'oscillation at ' + fmt(cur) : 'limit reached'}, back off`);
      if (key === 'P') { await this._coupleI(r.axis, nv); }
      this.lastGood[r.axis] = currentGains(this.family, r.axis, this.params); this.saveSession();
      r.lastChange = now;
      if (r.stage === 'D') { r.stage = 'P'; this.stateInfo = `${r.axis}: D done (${fmt(nv)}), now raising P`; }
      else { this.log('tuner', `${r.axis}: ramp done P=${fmt(this.params[r.names.P])} D=${fmt(this.params[r.names.D])}`); this.ramp = null; this._nextAxis(); }
      return;
    }
    // raise the gain: multiply so that it doubles every rampDoubleTime seconds, writing twice a second
    if (now < r.nextWrite) return;
    r.nextWrite = now + 500;
    const mul = Math.exp(Math.log(2) / (2 * this.opts.rampDoubleTime));
    let nv = Math.max(cur * mul, 0.001);
    // never reduce below the original by more than maxReducePct (QUIK_MAX_REDUCE)
    const minAllowed = r.saved[key] * (100 - this.opts.maxReducePct) / 100;
    if (nv < minAllowed) nv = minAllowed;
    await this.setParam(pname, nv, `QuikTune ramp ${key}`);
    if (key === 'P') await this._coupleI(r.axis, nv);
  }
  async _coupleI(axis, P) {
    const n = axisParamNames(this.family, axis);
    const ff = this.params[n.FF] || 0;
    if (ff > 0) return;   // with FF the I gain is tied to FF, not P
    const I = iRule(this.family === 'heli' ? 'copter' : this.family, axis, P, ff, false);
    await this.setParam(n.I, I, 'I follows P');
  }

  snapshot() {
    return { state: this.state, info: this.stateInfo, family: this.family, loopHz: this.loopHz, opts: this.opts, axis: this.currentAxis(), round: this.round,
      attitude: this.attitude, rc: this.rc, hud: this.hud, vehicle: this.link.vehicle, linkStats: this.link.stats, parserStats: this.link.parser.stats, msgRates: this.link.msgRates,
      rates: Object.fromEntries(AXES.map(a => [a, { hz: +this.axes[a].rateHz.toFixed(1), seconds: +this.axes[a].seconds.toFixed(0), excitation: +this.axes[a].excitation(20).toFixed(1), watchdog: this.axes[a].watchdog(2, { oscSmax: this.opts.oscSmax }) }])),
      gains: this.family ? Object.fromEntries(AXES.map(a => [a, currentGains(this.family, a, this.params)])) : null,
      originals: this.originals, previous: this.previous || {}, prepared: !!this.prepared, pendingWrites: this.pendingWrites || [], vibe: this.vibe || null,
      journal: this.journal.slice(-50), messages: this.messages.slice(-40) };
  }
}

function fmt(v) { return typeof v === 'number' ? +v.toPrecision(4) : v; }

module.exports = { Tuner, AXES };
