// Live tuning session: observe -> identify -> propose -> (apply in bounded steps, each change watched) ->
// observe again, per axis. Also the QuikTune-style ramp as a fallback when the model cannot be
// identified, and the parameter journal with one-click restore.
//
// In-flight safety layer (2026-10-17 audit: F-01, F-02, F-03, F-06, F-26, F-31, F-44, F-91, F-92):
//  - writing while the vehicle is armed needs the explicit opt-in opts.inFlightWrites (off by default:
//    experimental, simulator-tested only); every write holds the single write lock (a "job"); pause and stop
//    cancel a job before its next PARAM_SET, retries included;
//  - while a session is active a monitor checks every axis every 250 ms and acts on a sustained slew rate above
//    the limit or Dmod below 0.9 (more than half of the last 2 s);
//  - every in-flight change (gains, filters, notch, values the autopilot wrote itself) is watched afterwards on
//    the axes it affects; the watch fails closed when the PID data stop, come in too sparse or the link dies;
//  - on a trip the session pauses at once and the change is undone from evidence: back to the set before the
//    change when the 2 s before it were calm, otherwise to the values the session started from (observed calm
//    in the check-up); no corrective write is larger than stepMax, and the result is watched as well.
//
// Verified writes bound to the vehicle (2026-10-17 audit R3: F-25, F-28, F-29, F-32, F-33, F-35, F-39, F-51):
//  - every write is verified exactly (float32 / whole number by type) and journaled (requested, sent, echo, read-back,
//    status) in the append-only journal; a write that does not verify fails, pauses and the whole apply is written back
//    to the values from before it (each verified); what cannot be proven is listed as uncertain;
//  - the session identity (connection, system id, autopilot, board UID, firmware, parameter hash) is taken when the
//    vehicle is read and carried by every analysis and proposal; a proposal has an id and a hash over the exact
//    (parameter, from, to) list, the approval must send both back, and a reconnect or another vehicle drops proposals,
//    buffers and automatic writes; every value is read fresh before an apply and a changed one stops it;
//  - parameters that take effect only after a reboot are never written in flight; on the ground they are noted for the
//    reboot workflow (NotchLive.reboot, verified after the reboot); the cache follows every PARAM_VALUE of the vehicle.
'use strict';
const EventEmitter = require('events');
const fs = require('fs');
const path = require('path');
const { LiveAxis } = require('./axis');
const { analyzeAxis, currentGains } = require('./analysis');
const { detectFamily, axisParamNames, familyDef } = require('./vehicle');
const { neededParams, oscSmaxFor, filterRules, iRule, gainLimits } = require('../model/rules');
const { isRebootRequired } = require('../model/parammeta');
const { verifiedSet, writeText } = require('./paramwrite');
const { sameF32, sessionIdentity, makeProposal } = require('./identity');
const { Journal } = require('./journal');

const AXES = ['roll', 'pitch', 'yaw'];
const PID_TUNING_ID = 194, ATTITUDE_ID = 30;
// gain keys in the order they are written (filters and limits first, D before P); IMAX is written when a proposal
// changes it, like every value it shows (F-33)
const WRITE_ORDER = ['FLTT', 'FLTE', 'FLTD', 'SMAX', 'IMAX', 'D', 'P', 'I', 'FF', 'angleP', 'tconst'];
// why a proposal changes a value (shown with the full parameter name before the approval)
const REASON = { P: 'rate P from the identified model (preset margins)', I: 'rate I follows P / FF (ArduPilot rule)', D: 'rate D from the identified model', FF: 'feed-forward',
  FLTT: 'target filter (QuikTune rule)', FLTE: 'error filter (QuikTune rule)', FLTD: 'D-term filter (QuikTune rule)', SMAX: 'slew-rate limit (needed for the oscillation signal)',
  IMAX: 'I limit: the I term holds a steady correction near its limit', angleP: 'angle gain from the rate-loop crossover (at most x1.5 per round)', tconst: 'time constant from the rate-loop crossover' };
// a live proposal older than this is not applied: observe again
const PROPOSAL_MAX_AGE_MS = 15 * 60 * 1000;
// per-axis values a corrective write moves in steps of at most stepMax; LINEAR_KEYS also from / to zero (linearly)
const STEPPED_KEYS = ['P', 'I', 'D', 'FF', 'angleP', 'tconst', 'FLTT', 'FLTE', 'FLTD'];
const LINEAR_KEYS = ['P', 'I', 'D', 'FF', 'angleP', 'tconst'];
// vehicle-wide values: a change affects every axis (notch, gyro filter, FFT)
const SHARED_PARAMS = ['INS_HNTCH_ENABLE', 'INS_HNTCH_MODE', 'INS_HNTCH_FREQ', 'INS_HNTCH_BW', 'INS_HNTCH_ATT', 'INS_HNTCH_REF', 'INS_HNTCH_HMNCS', 'INS_HNTCH_OPTS', 'INS_HNTCH_FM_RAT', 'INS_GYRO_FILTER', 'FFT_ENABLE'];
// share of an observation window with slew-rate / Dmod trips that counts as "oscillating" when no model fits
const OSC_SEEN_FRACTION = 0.05;
const IN_FLIGHT_OFF = 'changes in flight are turned off: while the vehicle is armed the app only measures. Land to write the changes, or turn on "Allow changes in flight" (experimental, tested only in the simulator)';

class Tuner extends EventEmitter {
  constructor(link, opts = {}) {
    super();
    this.link = link;
    const journal = opts.journal; opts = Object.assign({}, opts); delete opts.journal;
    this.opts = Object.assign({ preset: 'normal', autoApply: false, axes: ['roll', 'pitch'], observeSeconds: 30, stepMax: 1.5, watchSeconds: 4, oscSmax: 5, massKg: 0, allowRamp: false, rampDoubleTime: 10, rampMargin: 60, maxReducePct: 20,
      inFlightWrites: false, watchGrace: 0.5, filterWatchSeconds: 10, monitorSeconds: 2, monitorFraction: 0.5, revertStepSeconds: 1, calmWaitSeconds: 4, angleAbort: 10, angleAbortSeconds: 0.8,
      sessionDir: path.join(__dirname, '..', '..', 'sessions') }, opts);
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
    this._timer = null; this._loopGen = 0;
    // safety layer
    this.sessionActive = false; this.sessionBase = null; this.changes = [];
    this._job = null; this._jobSeq = 0; this._changeSeq = 0;
    this._pendingTrip = null; this._tripLatch = {}; this.lastTrip = null;
    this._attHist = []; this._attErr = [];
    this.monitor = { active: false, axes: {}, attitude: { active: false, roll: null, pitch: null }, lastTrip: null };
    this._monitorTimer = null;
    // R3: the journal file, the session identity, values that need a reboot, names being written now
    this.audit = journal || new Journal(path.join(this.opts.sessionDir, 'journal'));
    this.identity = null; this.paramsAtPrepare = {}; this.rebootPending = []; this._writingNames = new Set(); this._txnSeq = 0;
    this.expectExternal = null;   // who changes values on the vehicle right now (the autopilot's FFT notch tune)
    this._bind();
  }

  _bind() {
    const L = this.link;
    L.on('PID_TUNING', (f, m) => {
      // the units depend on the controller family: nothing is stored before it is known (F-91)
      if (!this.family) return;
      const ax = f.axis === 1 ? 'roll' : f.axis === 2 ? 'pitch' : f.axis === 3 ? 'yaw' : null;
      if (!ax) return;
      // copter / VTOL rate PIDs run in rad/s (plane reports deg/s); keep everything in deg/s
      const k = this.family === 'plane' ? 1 : 57.2957795;
      this.axes[ax].push(m.ts / 1000, Object.assign({}, f, { desired: f.desired * k, achieved: f.achieved * k }));
    });
    L.on('ATTITUDE', (f, m) => {
      this.attitude = { roll: f.roll * 57.2958, pitch: f.pitch * 57.2958, yaw: f.yaw * 57.2958, p: f.rollspeed * 57.2958, q: f.pitchspeed * 57.2958, r: f.yawspeed * 57.2958, ts: m.ts / 1000 };
      this._attHist.push({ t: m.ts / 1000, roll: this.attitude.roll, pitch: this.attitude.pitch });
      while (this._attHist.length && this._attHist[0].t < m.ts / 1000 - 3) this._attHist.shift();
    });
    // attitude target (Copter: the attitude controller's target; Plane: nav_roll / nav_pitch) for the attitude-error abort
    L.on('NAV_CONTROLLER_OUTPUT', (f, m) => {
      const t = m.ts / 1000;
      let best = null;
      for (const a of this._attHist) if (Math.abs(a.t - t) <= 0.1 && (!best || Math.abs(a.t - t) < Math.abs(best.t - t))) best = a;
      if (best) this._attErr.push({ t, roll: wrap180(best.roll - f.nav_roll), pitch: best.pitch - f.nav_pitch });
      while (this._attErr.length && this._attErr[0].t < t - 5) this._attErr.shift();
    });
    L.on('RC_CHANNELS', (f, m) => { const n = v => (v - 1500) / 500; this.rc = { roll: n(f.chan1_raw), pitch: n(f.chan2_raw), yaw: n(f.chan4_raw), thr: (f.chan3_raw - 1000) / 1000, ts: m.ts / 1000 }; });
    L.on('VFR_HUD', f => { this.hud = { airspeed: f.airspeed, throttle: f.throttle, alt: f.alt, groundspeed: f.groundspeed }; });
    L.on('STATUSTEXT', f => this.log('vehicle', f.text));
    L.on('VIBRATION', (f, m) => { this.vibe = { x: f.vibration_x, y: f.vibration_y, z: f.vibration_z, clip: (f.clipping_0 | 0) + (f.clipping_1 | 0) + (f.clipping_2 | 0), ts: m.ts / 1000 }; });
    // every PARAM_VALUE of the vehicle keeps the cache current; a change the app did not make is reported (Q94)
    L.on('param', (name, value) => this._onParamValue(name, value));
    L.on('vehicle-change', c => this._onConnectionChange('another vehicle answered (' + c.reason + ')'));
    L.on('heartbeat', v => {
      if (this._preparedGen !== undefined && this._preparedGen !== L.generation) this._onConnectionChange('the connection changed');
      if (!this.family && Object.keys(this.params).length) this.family = detectFamily(v, this.params);
      if (this._linkGen !== L.generation) { this._linkGen = L.generation; if (this.family) { this.requestStreams(); this.log('link', 'new connection: streams re-requested'); } }
      // a disarm pauses a running session (once: a paused session keeps its message)
      if (v.armed) this.armedSeen = true;
      else if (this.armedSeen && ['observing', 'analyzing', 'applying', 'proposed', 'ramping'].includes(this.state)) this.onDisarm();
    });
  }

  _onParamValue(name, value) {
    if (!(name in this.params)) return;
    const cur = this.params[name];
    this.params[name] = value;
    if (this._writingNames.has(name) || cur === undefined || sameF32(cur, value)) return;
    const who = this.expectExternal || 'not by AP Tuner: another ground station?';
    this.log('param', `${name} changed on the vehicle: ${fmt(cur)} -> ${fmt(value)} (${who})`);
    this.audit.append('conflict', { name, from: cur, to: value, by: who, identity: this.identity ? this.identity.key : null, gen: this.link.generation });
  }

  /** a reconnect or another vehicle: proposals, live buffers and automatic writes of the old connection end (F-25) */
  _onConnectionChange(why) {
    if (this._preparedGen === undefined || this._preparedGen === this.link.generation || this._droppedGen === this.link.generation) return;
    this._droppedGen = this.link.generation;
    this.results = {}; this.previous = {}; this.prepared = false; this.family = null; this.params = {}; this.rebootPending = []; this.identity = null;
    for (const ax of AXES) this.axes[ax].clear();
    this._cancelJob(why);
    this.endSession();
    if (['observing', 'analyzing', 'applying', 'proposed', 'ramping'].includes(this.state)) this.pause(`${why}: read the vehicle again (step 2); nothing is written until then`);
    this.audit.append('connection', { why, gen: this.link.generation, identity: this.link.identity() });
    this.log('link', `${why}: proposals and live data of the previous connection were dropped`);
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
    if (this._job) throw new Error(`a change is being written (${this._job.kind}): wait until it has finished`);
    try { return await this._prepare(confirm); } catch (e) { if (this.state === 'preparing') this.setState('idle', 'reading the vehicle failed: ' + e.message); throw e; }
  }
  async _prepare(confirm) {
    this.setState('preparing', 'reading parameters');
    // a new connection may be a different vehicle: drop everything cached from the previous one
    if (this._preparedGen !== this.link.generation) { this._preparedGen = this.link.generation; this.params = {}; this.originals = {}; this.previous = {}; this.results = {}; this.lastGood = {}; this.prepared = false; this.lastTrip = null; this.rebootPending = []; this.identity = null; this.endSession(); }
    // samples buffered so far may be from another vehicle or in other units: none are kept until the family is known (F-91)
    this.family = null;
    for (const ax of AXES) this.axes[ax].clear();
    const v = this.link.vehicle;
    let fam = detectFamily(v, {});
    if (fam === 'plane') { const q = await this.link.requestParam('Q_ENABLE').catch(() => 0); this.params.Q_ENABLE = q; fam = detectFamily(v, this.params); }
    if (!fam) throw new Error('unsupported vehicle type ' + v.typeName);
    this.family = fam;
    const names = neededParams(fam === 'heli' ? 'copter' : fam);
    const got = await this.link.requestParams(names, 3, (d, n) => this.emit('progress', { done: d, total: n }));
    Object.assign(this.params, got);
    await this.link.waitAutopilot(3000);
    this.identity = sessionIdentity(this.link.identity(), this.params);
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
    if (writes.length) {
      // housekeeping only (telemetry on, slew rate reported): not watched, but while armed it needs the opt-in too
      const job = this._beginJob('prepare');
      try { for (const w of writes) await this.setParam(w.name, w.to, w.why.split('.')[0], job); } finally { this._endJob(job); }
    }
    this.pendingWrites = [];
    // the session identity: this connection, the vehicle (board UID, firmware) and a hash of every value read (F-25)
    this.identity = sessionIdentity(this.link.identity(), this.params);
    this.paramsAtPrepare = Object.assign({}, this.params);
    this.audit.append('identity', { identity: this.identity, via: 'read the vehicle', family: fam });
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
    if (this._job) throw new Error(`a change is being written (${this._job.kind}): wait until it has finished`);
    // read every parameter of the new family afresh (the vehicle may have been changed or replaced)
    const names = neededParams(fam === 'heli' ? 'copter' : fam);
    Object.assign(this.params, await this.link.requestParams(names, 3, (d, n) => this.emit('progress', { done: d, total: n })));
    this.family = fam;
    this.loopHz = this.params.SCHED_LOOP_RATE || (fam === 'plane' ? 50 : 400);
    for (const ax of AXES) { this.originals[ax] = currentGains(fam, ax, this.params); this.axes[ax].clear(); }
    this.lastGood = JSON.parse(JSON.stringify(this.originals)); this.previous = {}; this.results = {};
    const job = this._beginJob('switch family');
    try { for (const ax of AXES) { const n = axisParamNames(fam, ax); if (n.SMAX && !(this.params[n.SMAX] > 0)) await this.setParam(n.SMAX, familyDef(fam).defaultSmax, 'slew-rate limit so the oscillation signal is reported', job); } } finally { this._endJob(job); }
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
    this.link.requestDataStream(2, 5);   // EXTENDED_STATUS: NAV_CONTROLLER_OUTPUT (attitude target)
    setTimeout(() => {
      // fast PID_TUNING (one message per axis per interval) and ATTITUDE; vehicles cap this by link capacity
      this.link.setMessageRate(PID_TUNING_ID, this.family === 'plane' ? 25 : 50);
      this.link.setMessageRate(ATTITUDE_ID, 25);
      this.link.setMessageRate(241, 2);   // VIBRATION, for the final check
    }, 1500);
  }

  // ---------------- writing: one write lock, the in-flight gate, cancellation ----------------
  /** the kind of the job holding the write lock (null when nothing is being written) */
  get writing() { return this._job ? this._job.kind : null; }
  _armed() { const v = this.link.vehicle; return !!(v && v.armed); }
  /** take the write lock; handlesTrips: the job reverts its own changes when a trip comes in while it runs */
  _beginJob(kind, { corrective = false, handlesTrips = false } = {}) {
    if (this._job) throw new Error(`a change is being written (${this._job.kind}): wait until it has finished`);
    const job = { id: ++this._jobSeq, kind, corrective, handlesTrips, cancelled: null, abortWatch: false, gen: this.link.generation, touched: new Map(), txn: null };
    this._job = job;
    return job;
  }
  _endJob(job) { if (this._job === job) this._job = null; }
  /** pause / stop: the running job writes nothing more (a corrective job is never cancelled) */
  _cancelJob(reason) { if (this._job && !this._job.corrective && !this._job.cancelled) this._job.cancelled = reason || 'cancelled'; }
  async _waitNoJob(ms) { const t0 = Date.now(); while (this._job && Date.now() - t0 < ms) await sleep(50); return !this._job; }
  /** before every forward PARAM_SET: a cancel or a pending trip stops the job here */
  _checkpoint(job) {
    if (job.corrective) return;
    if (this._pendingTrip) {
      if (!job.handlesTrips) throw Object.assign(new Error('not written: the vehicle is oscillating'), { code: 'blocked' });
      const trip = this._pendingTrip; this._pendingTrip = null;
      throw Object.assign(new Error('oscillation: ' + trip.reasons.join('; ')), { code: 'trip', trip });
    }
    if (job.cancelled) throw Object.assign(new Error('stopped: ' + job.cancelled), { code: 'cancelled' });
  }
  /** a new value needs a heartbeat, the connection that was read and, while armed, the in-flight opt-in */
  _assertForward(job) {
    if (!this.link.alive) throw new Error('no vehicle heartbeat: connect first');
    if (job.kind !== 'prepare' && (!this.prepared || this._preparedGen !== this.link.generation)) throw new Error('read the vehicle first (the connection changed)');
    if (this._armed() && !this.opts.inFlightWrites) throw Object.assign(new Error(IN_FLIGHT_OFF), { code: 'inflight-off' });
  }
  /** why a change on this axis could not be watched now (null: it can) */
  _watchBlind(ax) {
    const n = axisParamNames(this.family, ax);
    if (n.SMAX && !(this.params[n.SMAX] > 0)) return `${n.SMAX} is 0, so the vehicle does not report its slew rate`;
    const la = this.axes[ax]; const now = Date.now() / 1000;
    if (!la.t.length || now - la.lastTs > 1) return 'no live PID data';
    const s = la.slewStats(2, { now });
    if (s.n < 10) return 'too little live PID data';
    if (s.noSlew === s.n) return 'the PID data carry no slew-rate signal (SRate / PDmod)';
    return null;
  }

  /**
   * The one place every parameter write goes through (job = the write-lock holder). A forward job is checked for a
   * cancel / trip before every PARAM_SET (retries included) and needs the in-flight opt-in while armed; a corrective
   * job only needs a connection.
   */
  async setParam(name, value, reason, job) {
    if (!job) throw new Error('internal: write without the write lock');
    // a change started on one connection never goes on on another one (it may be another vehicle)
    if (job.gen !== this.link.generation) throw Object.assign(new Error(`${name} not written: the connection changed since this change started`), { code: 'link-changed' });
    if (job.corrective) { if (!this.link.connected) throw new Error(`${name} not written: no connection to the vehicle`); }
    else {
      this._checkpoint(job); this._assertForward(job);
      if (this._armed() && isRebootRequired(name)) throw Object.assign(new Error(`${name} takes effect only after a reboot: it is set on the ground (disarmed), never in flight`), { code: 'reboot-required' });
      if (!this.audit.ok) throw Object.assign(new Error(`the write journal cannot be written (${this.audit.error}): nothing is written. Free some disk space and restart AP Tuner`), { code: 'journal' });
      if (!job.touched.has(name)) job.touched.set(name, this.params[name]);
    }
    const from = this.params[name];
    const armed = this._armed();
    this._writingNames.add(name);
    let r;
    try { r = await verifiedSet(this.link, name, value, { before: from, keepGoing: () => job.corrective || (!job.cancelled && !this._pendingTrip) }); }
    finally { this._writingNames.delete(name); }
    if (r.holds !== undefined) this.params[name] = r.holds;
    const status = r.cancelled ? 'cancelled: ' + (r.status === 'cancelled' ? 'not sent' : r.status === 'verified' ? 'written before the stop (read back)' : writeText(r)) : r.status;
    const entry = { t: Date.now(), name, from, to: r.holds, requested: value, sent: r.sent, echo: r.echo, readBack: r.readBack, status, reason, armed, job: job.kind, txn: job.txn };
    this.journal.push(entry);
    this.audit.append('write', Object.assign({ identity: this.identity ? this.identity.key : null, gen: this.link.generation, type: r.type, before: r.before, ok: r.ok, error: r.error, note: r.note, ms: r.ms, corrective: !!job.corrective }, entry));
    this.saveSession();
    if (r.cancelled) { this._checkpoint(job); throw Object.assign(new Error(`${name} not written: cancelled`), { code: 'cancelled' }); }
    if (!r.ok) { this.log('error', writeText(r)); throw Object.assign(new Error(writeText(r)), { code: 'verify', result: r }); }
    this.emit('param', { name, from, to: r.holds, reason });
    this.log('param', `${name}: ${fmt(from)} -> ${fmt(r.holds)} (${reason})`);
    return r.holds;
  }

  saveSession() {
    if (!this.sessionFile) return;
    try { fs.writeFileSync(this.sessionFile, JSON.stringify({ started: this.sessionFile, family: this.family, originals: this.originals, lastGood: this.lastGood, journal: this.journal, opts: this.opts }, null, 1)); } catch (e) { /* ignore */ }
  }

  /** write a gain set for an axis (only the keys given), in a safe order (D before P; filters first); not watched */
  async writeGains(axis, gains, reason, job) {
    const n = axisParamNames(this.family, axis);
    for (const k of WRITE_ORDER) {
      if (gains[k] === undefined || !n[k]) continue;
      const cur = this.params[n[k]];
      if (cur !== undefined && Math.abs(cur - gains[k]) < 1e-9) continue;
      await this.setParam(n[k], gains[k], reason, job);
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
    res.identity = this.identity ? { key: this.identity.key, paramHash: this.identity.paramHash } : null;
    if (res.proposal) this.bindProposal(axis, res.proposal);
    const step = Math.max(1, Math.floor(data.n / 1500));
    res.series = { t: [], target: [], actual: [], u: [] };
    for (let i = 0; i < data.n; i += step) { res.series.t.push(+(data.t0 + i / data.fs).toFixed(3)); res.series.target.push(+data.target[i].toFixed(2)); res.series.actual.push(+data.actual[i].toFixed(2)); res.series.u.push(+data.u[i].toFixed(4)); }
    this.results[axis] = res;
    this.emit('analysis', res);
    return res;
  }

  /**
   * The proposal gets its full parameter rows (name, from, to, reason, reboot flag: what the approval shows), an id and
   * a hash over the session identity and that exact list; Apply has to send id and hash back (F-33).
   */
  bindProposal(axis, p) {
    const n = axisParamNames(this.family, axis); const rows = [];
    for (const k of WRITE_ORDER) {
      if (p.gains[k] === undefined || !n[k]) continue;
      const from = this.params[n[k]];
      if (from !== undefined && Math.abs(from - p.gains[k]) < 1e-9) continue;   // unchanged: not written
      rows.push({ name: n[k], key: k, from, to: p.gains[k], reason: REASON[k] || '', reboot: isRebootRequired(n[k]) });
    }
    Object.assign(p, makeProposal({ source: 'live', axis, identity: this.identity, rows }));
    this.audit.append('proposal', { proposal: p.id, hash: p.hash, axis, identity: this.identity ? this.identity.key : null, rows });
    return p;
  }

  // ---------------- session control ----------------
  /** a tuning session starts: the values it may have to go back to are kept, and the monitor starts acting */
  beginSession() {
    const values = {};
    if (this.family) for (const ax of AXES) { const n = axisParamNames(this.family, ax); for (const k of WRITE_ORDER) if (n[k] && this.params[n[k]] !== undefined) values[n[k]] = this.params[n[k]]; }
    for (const k of SHARED_PARAMS) if (this.params[k] !== undefined) values[k] = this.params[k];
    this.sessionBase = { t: Date.now(), values };
    this.changes = []; this._tripLatch = {}; this._pendingTrip = null;
    this.sessionActive = true;
  }
  endSession() { this.sessionActive = false; this._pendingTrip = null; }

  start() {
    if (this._job) throw new Error(`a change is being written (${this._job.kind}): wait until it has finished`);
    if (this.state === 'idle' || this.state === 'ready' || this.state === 'done' || this.state === 'paused') {
      if (!this.family || !this.prepared) throw new Error('read the vehicle first (step 2)');
      if (this.opts.autoApply && this._armed() && !this.opts.inFlightWrites) throw new Error(IN_FLIGHT_OFF);
      if (!this.sessionActive) this.beginSession();
      this.requestStreams();
      this.axisIdx = 0; this.round = 0;
      for (const ax of AXES) this.axes[ax].setBaseline();
      this.setState('observing', `${this.currentAxis()}: fly normally and tap the sticks (round 1)`);
      this._tick();
    }
  }
  pause(reason) { this._cancelJob(reason || 'paused by user'); this.setState('paused', reason || 'paused by user'); this.ramp = null; }
  currentAxis() { return this.opts.axes[this.axisIdx % this.opts.axes.length]; }

  async stop() { this._cancelJob('stopped'); this.ramp = null; this._loopGen++; if (this._timer) clearTimeout(this._timer); this._timer = null; this.endSession(); this.setState('idle', 'stopped'); }

  /** restore the original gains of all axes (as read from the vehicle), in steps of at most stepMax */
  async restoreOriginals() {
    this.pause('restoring original gains');
    await this._takeOver();
    const targets = [];
    for (const ax of AXES) if (this.originals[ax]) { const n = axisParamNames(this.family, ax); for (const k of WRITE_ORDER) if (n[k] && this.originals[ax][k] !== undefined && !(k === 'SMAX' && !(this.originals[ax][k] > 0))) targets.push({ name: n[k], to: this.originals[ax][k] }); }
    await this._userCorrective('restore', targets, 'restore original');
    this.lastGood = JSON.parse(JSON.stringify(this.originals));
    this.log('tuner', 'original gains restored');
  }
  /** undo the last apply on an axis (gains as they were before it started) */
  async undoAxis(axis) {
    const g = (this.previous || {})[axis]; if (!g) throw new Error('nothing to undo for ' + axis);
    if (this.state === 'observing' || this.state === 'applying' || this.state === 'analyzing') this.pause('undo requested');
    await this._takeOver();
    const n = axisParamNames(this.family, axis); const targets = [];
    for (const k of WRITE_ORDER) if (n[k] && g[k] !== undefined && !(k === 'SMAX' && !(g[k] > 0))) targets.push({ name: n[k], to: g[k] });
    await this._userCorrective('undo', targets, 'undo last apply');
    this.lastGood[axis] = JSON.parse(JSON.stringify(g)); this.saveSession();
  }
  /** a running forward job is stopped (its watch ends early: the user takes over); a revert in progress is waited for */
  async _takeOver() {
    if (!this._job) return;
    if (!this._job.corrective) { this._cancelJob('taken over by the user'); this._job.abortWatch = true; }
    if (!(await this._waitNoJob(15000))) throw new Error('the app is still busy writing: try again in a moment');
  }
  /** a user-requested return to earlier values: while armed it needs the opt-in like any other in-flight write */
  async _userCorrective(kind, targets, reason) {
    if (!this.link.alive) throw new Error('no vehicle heartbeat: connect first');
    // the values belong to the vehicle read on this connection: after a reconnect it may be another vehicle
    if (!this.prepared || this._preparedGen !== this.link.generation) throw new Error('read the vehicle first (the connection changed)');
    if (this._armed() && !this.opts.inFlightWrites) throw new Error('the vehicle is armed and ' + IN_FLIGHT_OFF);
    const job = this._beginJob(kind, { corrective: true });
    job.txn = ++this._txnSeq;
    this.audit.append('txn', { txn: job.txn, phase: 'begin', kind, reason, items: targets });
    try {
      // read back before going back (F-51): every value is read fresh, and only what differs is written (each verified)
      const r = await this._corrective(job, targets, reason, { fresh: true });
      this.audit.append('txn', { txn: job.txn, phase: 'end', ok: !r.failed.length, failed: r.failedText });
      if (r.failed.length) throw new Error(`not written back: ${r.failedText.join('; ')}`);
    } finally { this._endJob(job); }
  }
  onDisarm() { if (this.state !== 'idle') this.pause('vehicle disarmed'); }

  /**
   * The user approved a proposal (manual mode): approval = {id, hash} of the proposal shown. True if every change
   * held; a write that does not verify rolls the whole apply back and throws (code 'apply-failed', the session pauses).
   */
  async applyProposal(axis, approval = null) {
    const r = this.results[axis];
    if (!r || !r.proposal) throw new Error('no proposal for ' + axis);
    this._checkApproval(r.proposal, approval);
    if (this.state === 'applying' || this._job) throw new Error('another change is being written: wait until it has finished');
    if (this._armed() && !this.opts.inFlightWrites) throw new Error(IN_FLIGHT_OFF);
    const job = this._beginJob('apply ' + axis, { handlesTrips: true });
    try {
      this.setState('applying', axis);
      this.audit.append('approval', { proposal: r.proposal.id, hash: r.proposal.hash, axis, by: 'user', via: 'Apply', identity: this.identity ? this.identity.key : null });
      return await this._applyBound(axis, r.proposal, job);
    } catch (e) {
      if (e.code === 'cancelled') return false;
      if (this.state === 'applying') this.pause(`${axis}: apply stopped: ${e.message}`);
      throw e;
    } finally { this._endJob(job); }
  }

  /** the approval must name the proposal held now (id and hash), made on this connection for this vehicle, not too old */
  _checkApproval(p, approval) {
    if (!p.id || !approval || approval.id !== p.id || approval.hash !== p.hash) {
      this.audit.append('refused', { what: 'apply', proposal: p.id || null, hash: p.hash || null, approval: approval || null, why: 'proposal id / hash mismatch' });
      throw Object.assign(new Error('the proposal changed since it was shown (a newer analysis): look at it again, then press Apply'), { code: 'approval' });
    }
    const id = this.link.identity();
    if (!this.identity || p.gen !== this.link.generation || p.gen !== this.identity.gen || p.sysid !== id.sysid || p.compid !== id.compid) {
      this.audit.append('refused', { what: 'apply', proposal: p.id, why: 'another connection or vehicle', proposalIdentity: p.identityKey, now: id });
      throw Object.assign(new Error('the proposal was made on another connection or for another vehicle: read the vehicle again and analyse again'), { code: 'identity' });
    }
    if (Date.now() - p.at > PROPOSAL_MAX_AGE_MS) {
      this.audit.append('refused', { what: 'apply', proposal: p.id, why: 'too old' });
      throw Object.assign(new Error(`the proposal is ${Math.round((Date.now() - p.at) / 60000)} min old: observe again for a fresh one`), { code: 'stale' });
    }
  }

  /**
   * A bound proposal as one transaction: every value is read fresh first (one that is not what the proposal was made
   * from stops it, nothing written); then the staged apply; a write that does not verify rolls the whole apply back to
   * the values from before it (each verified, within stepMax) and throws code 'apply-failed' (the session is paused).
   */
  async _applyBound(axis, p, job) {
    await this._freshCheck(p.rows);
    job.txn = ++this._txnSeq;
    this.audit.append('txn', { txn: job.txn, phase: 'begin', proposal: p.id, hash: p.hash, axis, armed: this._armed(), items: p.rows.map(r => ({ name: r.name, from: r.from, to: r.to })) });
    try {
      const ok = await this._applyStaged(axis, p.gains, job);
      this.audit.append('txn', { txn: job.txn, phase: 'end', ok });
      return ok;
    } catch (e) {
      if (e.code !== 'verify') { this.audit.append('txn', { txn: job.txn, phase: 'end', ok: false, error: e.message }); throw e; }
      await this._rollback(axis, job, e);
      throw Object.assign(new Error(this.stateInfo), { code: 'apply-failed' });
    }
  }

  /** read every value a change will write; one that is no longer what the change was made from stops it (F-25, Q94) */
  async _freshCheck(rows) {
    const stale = [];
    for (const r of rows) {
      let v;
      try { v = await this.link.requestParam(r.name, 1200, 2); } catch (e) { throw Object.assign(new Error(`${r.name} could not be read before writing (${e.message}): nothing was written`), { code: 'stale' }); }
      if (r.from !== undefined && !sameF32(v, r.from)) stale.push(`${r.name} is ${fmt(v)} on the vehicle, the proposal was made from ${fmt(r.from)}`);
    }
    if (stale.length) {
      this.audit.append('refused', { what: 'apply', why: 'changed on the vehicle', stale });
      throw Object.assign(new Error(`changed on the vehicle since the analysis: ${stale.join('; ')}. Nothing was written: analyse again`), { code: 'stale' });
    }
  }

  /**
   * A write that did not verify: every value this job touched goes back to its value from before (each verified; in
   * flight within stepMax and watched); what cannot be proven is listed as uncertain. The session pauses with the result.
   */
  async _rollback(axis, job, err) {
    job.corrective = true;
    const targets = [...job.touched].filter(([, to]) => typeof to === 'number').map(([name, to]) => ({ name, to }));
    const armed = this._armed();
    const what = axis ? axis : 'notch / filters';
    this.log('tuner', `${what}: ${err.message}: writing back the values from before`);
    const r = targets.length ? await this._corrective(job, targets, 'roll back: a write did not verify') : { written: [], failed: [], failedText: [] };
    const w = armed && r.written.length ? await this._watchCorrective(axis ? [axis] : this._sharedAxes()) : { ok: true, reasons: [] };
    this.audit.append('txn', { txn: job.txn, phase: 'end', ok: false, failed: err.result ? err.result.name : null, rolledBack: !r.failed.length, uncertain: r.failedText });
    if (r.failed.length) this.pause(`${what}: ${err.message}. Writing back failed, uncertain parameters: ${r.failedText.join('; ')}${armed ? ': land now' : ': check them before flying'}`);
    else this.pause(`${what}: ${err.message}. Rolled back: every value written by this apply is back to what it was before (read back)${w.ok ? '' : `, but ${w.reasons.join('; ')}: land now`}`);
  }

  /** staged application, each change watched in flight; returns true if the final gains were kept */
  async _applyStaged(axis, target, job) {
    const cur = currentGains(this.family, axis, this.params);
    this.previous = this.previous || {}; this.previous[axis] = JSON.parse(JSON.stringify(cur));
    const keys = ['P', 'I', 'D', 'FF'].filter(k => target[k] !== undefined && cur[k] !== undefined);
    // number of steps so that no multiplicative change exceeds stepMax
    let steps = 1;
    for (const k of keys) { const a = cur[k] || 1e-6, b = target[k] || 1e-6; if (a > 0 && b > 0) steps = Math.max(steps, Math.ceil(Math.abs(Math.log(b / a)) / Math.log(this.opts.stepMax) - 1e-9)); }
    steps = Math.min(steps, 6);
    // filters / smax first (small effect), then the gains stepwise, the angle gain last
    const pre = {}; for (const k of ['FLTT', 'FLTE', 'FLTD', 'SMAX']) if (target[k] !== undefined) pre[k] = target[k];
    if (!(await this._change(job, axis, pre, 'apply filters'))) return false;
    for (let s = 1; s <= steps; s++) {
      const g = {};
      for (const k of keys) { const a = cur[k], b = target[k]; g[k] = (a > 0 && b > 0) ? a * Math.pow(b / a, s / steps) : (s === steps ? b : a + (b - a) * s / steps); }
      if (!(await this._change(job, axis, g, `apply step ${s}/${steps}`))) return false;
      this.lastGood[axis] = currentGains(this.family, axis, this.params); this.saveSession();
    }
    const akeys = ['angleP', 'tconst'].filter(k => target[k] !== undefined && cur[k] !== undefined);
    const asteps = stepsFor(cur, target, akeys, this.opts.stepMax);
    for (let s = 1; s <= asteps; s++) {
      const g = {}; for (const k of akeys) g[k] = interp(cur[k], target[k], s, asteps);
      if (!(await this._change(job, axis, g, asteps > 1 ? `apply angle gain ${s}/${asteps}` : 'apply angle gain'))) return false;
      this.lastGood[axis] = currentGains(this.family, axis, this.params); this.saveSession();
    }
    this.log('tuner', this._armed() ? `${axis}: new gains applied, each change held ${this.opts.watchSeconds} s without oscillation` : `${axis}: new gains written (vehicle disarmed)`);
    return true;
  }

  /** the axes a vehicle-wide change (notch, filter) is watched on: the tuned ones and every axis that streams */
  _sharedAxes() { const now = Date.now() / 1000; return AXES.filter(ax => this.opts.axes.includes(ax) || (this.axes[ax].t.length && now - this.axes[ax].lastTs < 1)); }

  /**
   * One watched change: the gate, the calm-before evidence, the writes, the watch and, on a trip, the evidence-based
   * revert (the session is then paused). axis: 'roll' | 'pitch' | 'yaw' with gain keys, or null with parameter names
   * (vehicle-wide: notch, filters). Returns true if the change held (or nothing needed writing), false if not.
   */
  async _change(job, axis, values, reason, { kind = null } = {}) {
    const shared = axis === null;
    const entries = [];
    if (shared) { for (const [name, to] of Object.entries(values)) entries.push({ name, to }); }
    else { const n = axisParamNames(this.family, axis); for (const k of WRITE_ORDER) if (values[k] !== undefined && n[k]) entries.push({ name: n[k], to: values[k] }); }
    const todo = entries.filter(e => !(this.params[e.name] !== undefined && Math.abs(this.params[e.name] - e.to) < 1e-9));
    if (!todo.length) return true;
    const axes = shared ? this._sharedAxes() : [axis];
    const armed = this._armed();
    let change = null;
    try {
      this._checkpoint(job); this._assertForward(job);
      let calm = null;
      if (armed) {
        // a value that takes effect only after a reboot cannot be watched in flight (it acts on the next flight): ground only (F-35)
        const rb = todo.filter(e => isRebootRequired(e.name)).map(e => e.name);
        if (rb.length) throw Object.assign(new Error(`${rb.join(', ')} take${rb.length > 1 ? '' : 's'} effect only after a reboot: ${rb.length > 1 ? 'they are' : 'it is'} set on the ground (disarmed, then reboot), never in flight`), { code: 'reboot-required' });
        for (const ax of axes) { const why = this._watchBlind(ax); if (why) throw new Error(`${ax}: ${why}: the change could not be watched, so it was not written`); }
        if (!this.sessionActive) this.beginSession();
        // nothing is changed while the vehicle shows oscillation: wait a moment for calm, otherwise stop here
        calm = this._calmBefore();
        const t0 = Date.now();
        while (!calm.calm && Date.now() - t0 < this.opts.calmWaitSeconds * 1000) { await sleep(250); this._checkpoint(job); calm = this._calmBefore(); }
        if (!calm.calm) { this.pause(`not calm before "${reason}" (${calm.reasons.join('; ')}): nothing more is written. Land if it keeps shaking`); return false; }
      }
      change = { id: ++this._changeSeq, t: Date.now(), kind: kind || (shared ? 'filters' : 'gains'), axis, axes, shared, reason, armed,
        params: todo.map(e => ({ name: e.name, from: this.params[e.name], to: e.to })), written: [], calmBefore: calm, status: armed ? 'writing' : 'written' };
      this.changes.push(change); if (this.changes.length > 200) this.changes.shift();
      for (const e of todo) { await this.setParam(e.name, e.to, reason, job); change.written.push(e.name); }
    } catch (err) {
      if (err.code === 'trip') { if (change) change.status = 'partial'; await this._handleTrip(job, err.trip, change); return false; }
      if (err.code === 'verify') { if (change) change.status = 'failed'; throw err; }   // the whole set is rolled back by the caller
      // stopped or failed midway in flight: the part that was written is watched like any change
      if (!change || !armed || !change.written.length) { if (err.code === 'cancelled') return false; throw err; }
      change.status = 'partial';
      const res = await this._watchChange(job, change);
      if (!res.ok) { await this._handleTrip(job, res.trip, change); return false; }
      if (err.code === 'cancelled') return false;
      throw err;
    }
    if (!armed) return true;
    const res = await this._watchChange(job, change);
    if (res.ok) { change.status = res.aborted ? 'watch ended early' : 'held'; return true; }
    await this._handleTrip(job, res.trip, change);
    return false;
  }

  /** a vehicle-wide change (notch, gyro filter, FFT) through the write lock, watched in flight; true if it held */
  async applyFilterChange(values, reason) {
    if (this._job) throw new Error(`a change is being written (${this._job.kind}): wait until it has finished`);
    const job = this._beginJob('filters', { handlesTrips: true });
    try {
      // fresh values first: the change is worked out against what the vehicle holds now
      for (const name of Object.keys(values)) { try { await this.link.requestParam(name, 1200, 2); } catch (e) { throw new Error(`${name} could not be read (${e.message}): nothing was written`); } }
      job.txn = ++this._txnSeq;
      this.audit.append('txn', { txn: job.txn, phase: 'begin', kind: 'filters', reason, armed: this._armed(), items: Object.entries(values).map(([name, to]) => ({ name, from: this.params[name], to })) });
      let held;
      try { held = await this._change(job, null, values, reason, { kind: 'notch' }); }
      catch (e) {
        if (e.code !== 'verify') { this.audit.append('txn', { txn: job.txn, phase: 'end', ok: false, error: e.message }); throw e; }
        await this._rollback(null, job, e);
        throw Object.assign(new Error(this.stateInfo), { code: 'apply-failed' });
      }
      this.audit.append('txn', { txn: job.txn, phase: 'end', ok: held });
      // written on the ground: values that act only after a reboot wait for the reboot workflow (verified after it)
      if (held && !this._armed()) for (const name of job.touched.keys()) if (isRebootRequired(name)) this._notePendingReboot(name);
      return held;
    } finally { this._endJob(job); }
  }
  /** a value written on the ground that takes effect only after a reboot (checked again after the reboot) */
  _notePendingReboot(name) {
    this.rebootPending = this.rebootPending.filter(p => p.name !== name).concat([{ name, value: this.params[name] }]);
    this.log('tuner', `${name} takes effect after a reboot of the autopilot (disarmed)`);
  }

  /** values the vehicle changed by itself (the autopilot FFT notch tune) are watched like the app's own changes */
  async watchExternalChange(changed, reason) {
    const names = Object.keys(changed);
    if (!this._armed() || !names.length) return true;
    if (this._job && !(await this._waitNoJob(15000))) throw new Error('the app is still busy writing');
    const job = this._beginJob('watch', { handlesTrips: true });
    try {
      if (!this.sessionActive) this.beginSession();
      const change = { id: ++this._changeSeq, t: Date.now(), kind: 'autopilot', axis: null, axes: this._sharedAxes(), shared: true, reason, armed: true,
        params: names.map(k => ({ name: k, from: changed[k].from, to: changed[k].to })), written: names, calmBefore: this._calmBefore(), status: 'watching' };
      this.changes.push(change); if (this.changes.length > 200) this.changes.shift();
      const res = await this._watchChange(job, change);
      if (res.ok) { change.status = 'held'; return true; }
      await this._handleTrip(job, res.trip, change);
      return false;
    } finally { this._endJob(job); }
  }

  /** were the last 2 s calm on every axis that streams (not one slew-rate or Dmod trip)? */
  _calmBefore() {
    const now = Date.now() / 1000; const reasons = [];
    for (const ax of AXES) {
      const la = this.axes[ax]; if (!la.t.length || now - la.lastTs > 1) continue;
      const s = la.slewStats(2, { oscSmax: this.opts.oscSmax, now });
      if (s.bad) reasons.push(`${ax}: slew rate ${s.srateMax.toFixed(1)}${s.dmodMin < 0.9 ? ', Dmod ' + s.dmodMin.toFixed(2) : ''}`);
    }
    return { calm: !reasons.length, reasons, t: Date.now() };
  }

  /**
   * Watch a change on the axes it affects for watchSeconds (vehicle-wide changes: filterWatchSeconds) after a grace
   * period whose samples still describe the old values. It trips on a slew-rate / Dmod / spectral trip on an affected
   * axis, a sustained oscillation on any axis (the monitor), an attitude error above angleAbort, and it fails closed
   * when the link dies, the PID data stop, or too few samples came in. Resolves {ok, aborted, trip}.
   */
  _watchChange(job, change) {
    const dur = change.shared ? Math.max(this.opts.watchSeconds, this.opts.filterWatchSeconds) : this.opts.watchSeconds;
    const axes = change.axes;
    const rate0 = Object.fromEntries(axes.map(ax => [ax, this.axes[ax].rateHz]));
    return new Promise(resolve => {
      const since = Date.now() / 1000 + this.opts.watchGrace;
      change.status = 'watching';
      const done = (ok, reasons = [], ax = null, failClosed = false, aborted = false) => {
        clearInterval(iv);
        resolve({ ok, aborted, trip: ok ? null : { source: 'watch', axis: ax, reasons, failClosed, t: Date.now() } });
      };
      const iv = setInterval(() => {
        const now = Date.now() / 1000;
        if (job.abortWatch) return done(true, [], null, false, true);
        if (this._pendingTrip) { const t = this._pendingTrip; this._pendingTrip = null; return done(false, t.reasons, t.axis); }
        if (!this.link.alive) return done(false, ['the link to the vehicle was lost during the watch'], axes[0], true);
        for (const ax of axes) {
          const la = this.axes[ax];
          if (now >= since + 1 && now - la.lastTs > 1) return done(false, [`${ax}: no PID data for ${(now - la.lastTs).toFixed(1)} s after the change`], ax, true);
          const w = la.watchdog(2, { oscSmax: this.opts.oscSmax, since });
          if (w.oscillating) return done(false, w.reasons.map(r => `${ax}: ${r}`), ax);
        }
        const att = this._attitudeTrip(axes, since);
        if (att) return done(false, [att.text], att.axis);
        if (now >= since + dur) {
          // fail closed: the watch only counts if enough samples came in after the change
          for (const ax of axes) {
            const n = this.axes[ax].slewStats(dur + 1, { since, now }).n;
            const need = Math.max(10, Math.floor(0.5 * (rate0[ax] || 0) * dur));
            if (n < need) return done(false, [n ? `${ax}: only ${n} PID samples after the change (${need} needed to judge it)` : `${ax}: no PID data after the change`], ax, true);
          }
          return done(true);
        }
      }, 250);
    });
  }

  /**
   * Did a corrective write work? After watchGrace + watchSeconds the last monitorSeconds (samples after the write
   * only) must not show a sustained oscillation on any axis that streams, with enough samples and a live link.
   */
  async _watchCorrective(axes) {
    const rate0 = Object.fromEntries(axes.map(ax => [ax, this.axes[ax].rateHz]));
    const since = Date.now() / 1000 + this.opts.watchGrace;
    await sleep((this.opts.watchGrace + this.opts.watchSeconds) * 1000);
    const now = Date.now() / 1000;
    if (!this.link.alive) return { ok: false, reasons: ['the link to the vehicle is lost: the result is not known'] };
    for (const ax of axes) {
      const s = this.axes[ax].slewStats(this.opts.monitorSeconds, { oscSmax: this.opts.oscSmax, since, now });
      const need = Math.max(10, Math.floor(0.5 * (rate0[ax] || 0) * Math.min(this.opts.monitorSeconds, this.opts.watchSeconds)));
      if (s.n < need) return { ok: false, reasons: [`${ax}: only ${s.n} PID samples after the correction: the result is not known`] };
      if (s.frac >= this.opts.monitorFraction) return { ok: false, reasons: [`${ax}: slew rate ${s.srateMax.toFixed(1)} in ${Math.round(100 * s.frac)} % of the last ${this.opts.monitorSeconds} s`] };
    }
    return { ok: true, reasons: [] };
  }

  /** a sustained oscillation now (more than monitorFraction of the last monitorSeconds): {axis, reasons} or null */
  _sustained(ax) {
    const s = this.axes[ax].slewStats(this.opts.monitorSeconds, { oscSmax: this.opts.oscSmax, now: Date.now() / 1000 });
    if (s.n >= 10 && s.frac >= this.opts.monitorFraction && s.badLast - s.badFirst >= 0.4 * this.opts.monitorSeconds)
      return { axis: ax, reasons: [`${ax} oscillating: slew rate ${s.srateMax.toFixed(1)} (limit ${this.opts.oscSmax})${s.dmodMin < 0.9 ? `, Dmod ${s.dmodMin.toFixed(2)}` : ''} in ${Math.round(100 * s.frac)} % of the last ${this.opts.monitorSeconds} s`] };
    return null;
  }

  /** attitude error (ATTITUDE vs the NAV_CONTROLLER_OUTPUT target) above angleAbort for angleAbortSeconds, roll / pitch */
  _attitudeTrip(axes, since) {
    const now = Date.now() / 1000; const lim = this.opts.angleAbort;
    const last = this._attErr[this._attErr.length - 1];
    if (!last || now - last.t > 1) return null;   // no attitude target streaming: this check is inactive
    for (const ax of axes) {
      if (ax !== 'roll' && ax !== 'pitch') continue;
      let n = 0, oldest = null, worst = 0;
      for (let k = this._attErr.length - 1; k >= 0 && this._attErr[k].t >= since && Math.abs(this._attErr[k][ax]) > lim; k--) { n++; oldest = this._attErr[k].t; worst = Math.max(worst, Math.abs(this._attErr[k][ax])); }
      if (n >= 3 && last.t - oldest >= this.opts.angleAbortSeconds) return { axis: ax, text: `${ax}: attitude error ${worst.toFixed(1)} deg for ${(last.t - oldest).toFixed(1)} s (limit ${lim} deg)` };
    }
    return null;
  }

  // ---------------- trips: the evidence-based revert ----------------
  /** a trip seen outside a watch (monitor, observation): the job holding the lock handles it, or a revert job starts */
  _raiseTrip(trip) {
    if (this._job && this._job.corrective) return;   // already reverting
    if (!this._pendingTrip) this._pendingTrip = trip;
    if (!this._job) this._startCorrective();
  }
  _startCorrective() {
    const trip = this._pendingTrip; this._pendingTrip = null;
    if (!trip) return;
    const job = this._beginJob('revert', { corrective: true });
    this._handleTrip(job, trip, null).catch(e => this.log('error', 'revert failed: ' + e.message)).finally(() => this._endJob(job));
  }
  /** automatic reverts in flight: only with the opt-in, or if the app itself wrote in flight in this session */
  _correctiveAllowed() { return !this._armed() || this.opts.inFlightWrites || this.changes.some(c => c.armed); }
  /** the values that differ from the start of the session: what restoring the session's start values writes */
  _sessionDiff() {
    const out = [];
    if (!this.sessionBase) return out;
    for (const [name, to] of Object.entries(this.sessionBase.values)) {
      const cur = this.params[name];
      if (cur === undefined || Math.abs(cur - to) <= 1e-9 * Math.max(1, Math.abs(to))) continue;
      if (/_SMAX$/.test(name) && !(to > 0)) continue;   // never blind the slew-rate detector
      out.push({ name, to });
    }
    return out;
  }

  /**
   * The session pauses at once. If the 2 s before the change were calm, the change is reverted (back to the set before
   * it); otherwise, or if it still oscillates after that, the values from the start of the session are restored. Every
   * corrective write stays within stepMax and the result is watched. Without the opt-in and without an in-flight
   * write of the app in this session nothing is written: the pilot is told to land.
   */
  async _handleTrip(job, trip, change) {
    job.corrective = true;
    const what = trip.reasons.join('; ');
    const after = change ? ` after "${change.reason}"` : '';
    this.lastTrip = Object.assign({}, trip, { change: change ? change.id : null, at: Date.now() });
    for (const ax of AXES) this._tripLatch[ax] = true;
    if (change) change.status = 'tripped';
    this.pause(`${what}${after}: oscillation detected, reverting`);
    this.log('watchdog', `${what}${after}`);
    this.emit('trip', this.lastTrip);
    if (!this._correctiveAllowed()) {
      this.pause(`${what}. Changes in flight are off, so the app changed nothing in the air: land and check the gains`);
      return;
    }
    let reverted = null;   // what the revert of the change wrote and why it was not enough
    if (change && change.calmBefore && change.calmBefore.calm) {
      const targets = change.params.filter(p => p.from !== undefined).map(p => ({ name: p.name, to: p.from }));
      const r = await this._corrective(job, targets, 'revert: oscillation' + after);
      const w = await this._watchCorrective(AXES.filter(ax => change.axes.includes(ax) || ax === trip.axis));
      const list = r.written.map(e => e.name + ' ' + fmt(e.to)).join(', ') || 'nothing to write';
      if (w.ok && !r.failed.length) {
        this.pause(`${what}${after}: the change was reverted (${list}) and the vehicle is calm again. Press Resume to continue, or land`);
        return;
      }
      reverted = { list, why: r.failed.length ? `not written back: ${r.failedText.join('; ')}` : w.reasons.join('; ') };
      this.log('watchdog', `after the revert (${list}): ${reverted.why}: restoring the values from the start of the session`);
    }
    const targets = this._sessionDiff();
    if (!targets.length) {
      this.pause(reverted ? `${what}${after}: the change was reverted (${reverted.list}), but ${reverted.why}: land now`
        : `${what}${after}: the vehicle has the values it started the session with, so there is nothing to go back to. Land now and lower the gains`);
      return;
    }
    const r = await this._corrective(job, targets, 'restore the session start values: oscillation');
    const w = await this._watchCorrective(AXES.filter(ax => this.axes[ax].t.length));
    const list = r.written.map(e => e.name + ' ' + fmt(e.to)).join(', ');
    if (r.failed.length) this.pause(`${what}${after}: restoring the values from the start of the session failed (uncertain parameters: ${r.failedText.join('; ')}): land now`);
    else if (w.ok) this.pause(`${what}${after}: the values from the start of the session were restored (${list}) and the vehicle is calm again. Land and check, or press Resume`);
    else this.pause(`${what}${after}: still oscillating with the values from the start of the session (${w.reasons.join('; ')}): land now`);
  }

  /**
   * Corrective writes towards targets [{name, to}]: the notch switch first; rate / angle gains and their filters in
   * steps of at most stepMax (linear from / to zero), revertStepSeconds apart; everything else in one write. Best
   * effort: a failed write is reported and the rest is still written. Returns {written: [{name, to}], failed: [name]}.
   */
  async _corrective(job, targets, reason, { fresh = false } = {}) {
    const gainKey = new Map();
    if (this.family) for (const ax of AXES) { const n = axisParamNames(this.family, ax); for (const k of STEPPED_KEYS) if (n[k]) gainKey.set(n[k], k); }
    // fresh: read every value first and leave out what the vehicle provably holds already; otherwise nothing is left
    // out on the strength of the cache (F-51: a stale cache must not skip a revert)
    if (fresh) for (const t of targets) { try { await this.link.requestParam(t.name, 1200, 2); } catch (e) { /* the write will tell */ } }
    const stepped = [], direct = [];
    for (const t of targets) {
      const cur = this.params[t.name];
      if (fresh && sameF32(cur, t.to)) continue;
      const key = gainKey.get(t.name);
      if (key && cur !== undefined && !sameF32(cur, t.to) && ((cur > 0 && t.to > 0) || LINEAR_KEYS.includes(key))) stepped.push({ name: t.name, from: cur, to: t.to });
      else direct.push(t);
    }
    direct.sort((a, b) => (b.name === 'INS_HNTCH_ENABLE') - (a.name === 'INS_HNTCH_ENABLE'));
    const last = new Map(), failed = [], failedText = [];
    const write = async (name, value) => { try { await this.setParam(name, value, reason, job); last.set(name, value); } catch (e) { if (!failed.includes(name)) { failed.push(name); failedText.push(e.result ? writeText(e.result) : `${name}: ${e.message}`); } this.log('error', `${name}: ${e.message}`); } };
    for (const t of direct) await write(t.name, t.to);
    const steps = stepsFor(Object.fromEntries(stepped.map(s => [s.name, s.from])), Object.fromEntries(stepped.map(s => [s.name, s.to])), stepped.map(s => s.name), this.opts.stepMax);
    for (let k = 1; k <= steps && stepped.length; k++) {
      if (k > 1) await sleep(this.opts.revertStepSeconds * 1000);
      for (const s of stepped) if (!failed.includes(s.name)) await write(s.name, interp(s.from, s.to, k, steps));
    }
    return { written: [...last].map(([name, to]) => ({ name, to })), failed, failedText };
  }

  // ---------------- the oscillation monitor (every axis, every 250 ms) ----------------
  startMonitor() { if (!this._monitorTimer) this._monitorTimer = setInterval(() => { try { this._monitorTick(); } catch (e) { this.log('error', 'monitor: ' + e.message); } }, 250); }
  stopMonitor() { if (this._monitorTimer) clearInterval(this._monitorTimer); this._monitorTimer = null; }
  _monitorTick() {
    const now = Date.now() / 1000; const axes = {};
    for (const ax of AXES) {
      const la = this.axes[ax];
      const s = la.slewStats(this.opts.monitorSeconds, { oscSmax: this.opts.oscSmax, now });
      const n = this.family ? axisParamNames(this.family, ax) : {};
      let state;
      if (!this.family) state = 'no vehicle';
      else if (!la.t.length || now - la.lastTs > 1 || s.n < 10) state = 'no data';
      else if (n.SMAX && !(this.params[n.SMAX] > 0)) state = 'blind: SMAX is 0';
      else if (s.noSlew === s.n) state = 'blind: no slew-rate data';
      else if (this._sustained(ax)) state = 'oscillating';
      else state = s.bad ? 'unsettled' : 'calm';
      if (state === 'calm') this._tripLatch[ax] = false;
      axes[ax] = { state, srateMax: +s.srateMax.toFixed(2), dmodMin: +s.dmodMin.toFixed(3), frac: +s.frac.toFixed(2), samples: s.n };
    }
    const last = this._attErr[this._attErr.length - 1];
    this.monitor = { active: this.sessionActive, axes, attitude: { active: !!(last && now - last.t < 1), roll: last ? +last.roll.toFixed(1) : null, pitch: last ? +last.pitch.toFixed(1) : null }, lastTrip: this.lastTrip };
    if (!this.sessionActive) return;
    const osc = AXES.find(ax => axes[ax].state === 'oscillating' && !this._tripLatch[ax]);
    if (osc) this._raiseTrip(Object.assign({ source: 'monitor', t: Date.now() }, this._sustained(osc)));
    else if (this._pendingTrip && !this._job) this._startCorrective();
  }

  /** main loop: runs while observing / auto-applying (one loop only: an older loop ends at its next turn) */
  _tick() {
    if (this._timer) clearTimeout(this._timer);
    const gen = ++this._loopGen;
    const loop = async () => {
      if (gen !== this._loopGen) return;
      try {
        if (this.state === 'observing') await this._observeStep();
        else if (this.state === 'ramping') await this._rampStep();
      } catch (e) { if (e.code !== 'cancelled') { this.log('error', e.message); this.pause('error: ' + e.message); } }
      if (gen === this._loopGen && (this.state === 'observing' || this.state === 'ramping')) this._timer = setTimeout(loop, this.state === 'ramping' ? 250 : 1000);
    };
    this._timer = setTimeout(loop, 500);
  }

  async _observeStep() {
    const axis = this.currentAxis();
    const la = this.axes[axis];
    const elapsed = (Date.now() - this.stateSince) / 1000;
    const exc = la.excitation(20);
    // an oscillation is caught by the monitor (every axis, every 250 ms) while a session is active
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
      // no model: when the slew rate / Dmod show oscillation, say so and stop (never "fly sharper" while it shakes, F-03)
      const os = la.slewStats(this.opts.observeSeconds, { oscSmax: this.opts.oscSmax, now: Date.now() / 1000 });
      if (os.n >= 10 && os.frac >= OSC_SEEN_FRACTION) {
        const why = `${axis} oscillating: slew rate up to ${os.srateMax.toFixed(1)} (limit ${this.opts.oscSmax})${os.dmodMin < 0.9 ? `, Dmod down to ${os.dmodMin.toFixed(2)}` : ''} in ${Math.round(100 * os.frac)} % of the last ${this.opts.observeSeconds} s, so no gains were worked out`;
        this._raiseTrip({ source: 'observe', axis, reasons: [why], t: Date.now() });
        if (this.state === 'analyzing') this.pause(`${why}. Land if it keeps shaking`);
        return;
      }
      this.round++;
      if (this.opts.allowRamp && this.round >= 2) { this._startRamp(axis); return; }
      this.setState('observing', `${axis}: ${res.identification?.reason || (res.warnings || [])[0] || res.error || 'no proposal'} (round ${this.round + 1}) - fly sharper stick inputs`);
      return;
    }
    // converged when the rate gains move < 10% AND the angle gain / time constant (capped at x1.5 per round) < 15%
    const big = res.proposal.changes.some(c => (['P', 'D', 'FF'].includes(c.key) && Math.abs(c.pct ?? 0) > 10) || (['angleP', 'tconst'].includes(c.key) && Math.abs(c.pct ?? 0) > 15));
    if (!big) { this.log('tuner', `${axis}: converged (rate gains < 10%, angle gain < 15%)`); this._nextAxis(); return; }
    if (this.opts.autoApply) {
      if (this._armed() && !this.opts.inFlightWrites) { this.pause(`${axis}: ${IN_FLIGHT_OFF}`); return; }
      if (this._job) { this.setState('observing', `${axis}: waiting for another change to finish (${this._job.kind})`); return; }
      this.setState('applying', `${axis}: ${res.proposal.changes.map(c => c.key + ' ' + fmt(c.from) + '->' + fmt(c.to)).join(', ')}`);
      const job = this._beginJob('apply ' + axis, { handlesTrips: true });
      let ok = false;
      try {
        this.audit.append('approval', { proposal: res.proposal.id, hash: res.proposal.hash, axis, by: 'auto', via: 'auto-apply (authorised for this session when it started)', identity: this.identity ? this.identity.key : null });
        ok = await this._applyBound(axis, res.proposal, job);
      } catch (e) { if (e.code !== 'apply-failed') throw e; }   // rolled back, the session is paused with the reason
      finally { this._endJob(job); }
      if (ok && this.state === 'applying') { this.round++; if (this.round >= 4) this._nextAxis(); else this.setState('observing', `${axis}: re-checking with the new gains (round ${this.round + 1})`); }
    } else {
      this.setState('proposed', `${axis}: review the proposal and press Apply`);
    }
  }

  _nextAxis() {
    this.axisIdx++; this.round = 0;
    if (this.axisIdx >= this.opts.axes.length) { this.setState('done', 'all axes tuned'); return; }
    this.setState('observing', `${this.currentAxis()}: fly normally and tap the sticks (round 1)`);
  }

  /** continue after a manual apply or a pause (never while a change is being written: one loop, one writer) */
  resume() {
    if (this._job) throw new Error(`a change is still being written (${this._job.kind}): wait until it has finished`);
    if (this.state === 'proposed' || this.state === 'paused' || this.state === 'applying') { this.setState('observing', `${this.currentAxis()}: observing (round ${this.round + 1})`); this._tick(); }
  }

  // ---------------- QuikTune-style ramp (VTOL-quicktune.lua logic) ----------------
  _startRamp(axis) {
    const n = axisParamNames(this.family, axis);
    const fam = this.family === 'heli' ? 'copter' : this.family;
    const filters = filterRules(fam, axis, currentGains(fam, axis, this.params), this.params.INS_GYRO_FILTER || 20, 0.5);
    this.ramp = { axis, stage: 'D', names: n, saved: currentGains(fam, axis, this.params), lastChange: Date.now(), nextWrite: 0, filtersSet: false, filters };
    this.setState('ramping', `${axis}: QuikTune-style ramp, raising D (doubling every ${this.opts.rampDoubleTime} s)`);
  }
  async _rampStep() {
    if (this._job) return;
    const job = this._beginJob('ramp', { handlesTrips: true });
    job.txn = ++this._txnSeq;
    try { await this._rampWrite(job); }
    catch (e) {
      if (e.code === 'trip') { await this._handleTrip(job, e.trip, null); return; }
      if (e.code === 'verify') { const axis = this.ramp ? this.ramp.axis : this.currentAxis(); this.ramp = null; await this._rollback(axis, job, e); return; }
      throw e;
    }
    finally { this._endJob(job); }
  }
  async _rampWrite(job) {
    const r = this.ramp; if (!r) { this.setState('observing', 'ramp ended'); return; }
    const la = this.axes[r.axis];
    const now = Date.now();
    // in flight the ramp's own detector needs live data with the slew-rate signal: without it, stop (fail closed)
    if (this._armed()) { const why = this._watchBlind(r.axis); if (why) { this.pause(`${r.axis}: ${why}: the ramp stopped (it cannot see an oscillation)`); return; } }
    if (!r.filtersSet) { await this.writeGains(r.axis, r.filters, 'QuikTune filter rule', job); r.filtersSet = true; return; }
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
        await this.setParam(r.names.P, this.params[r.names.P] * ratio, 'QuikTune: P lowered with D', job);
      }
      await this.setParam(pname, nv, `QuikTune: ${w.oscillating ? 'oscillation at ' + fmt(cur) : 'limit reached'}, back off`, job);
      if (key === 'P') { await this._coupleI(r.axis, nv, job); }
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
    await this.setParam(pname, nv, `QuikTune ramp ${key}`, job);
    if (key === 'P') await this._coupleI(r.axis, nv, job);
  }
  async _coupleI(axis, P, job) {
    const n = axisParamNames(this.family, axis);
    const ff = this.params[n.FF] || 0;
    if (ff > 0) return;   // with FF the I gain is tied to FF, not P
    const I = iRule(this.family === 'heli' ? 'copter' : this.family, axis, P, ff, false);
    await this.setParam(n.I, I, 'I follows P', job);
  }

  snapshot() {
    return { state: this.state, info: this.stateInfo, family: this.family, loopHz: this.loopHz, opts: this.opts, axis: this.currentAxis(), round: this.round,
      attitude: this.attitude, rc: this.rc, hud: this.hud, vehicle: this.link.vehicle, linkStats: this.link.stats, parserStats: this.link.parser.stats, msgRates: this.link.msgRates,
      rates: Object.fromEntries(AXES.map(a => [a, { hz: +this.axes[a].rateHz.toFixed(1), seconds: +this.axes[a].seconds.toFixed(0), excitation: +this.axes[a].excitation(20).toFixed(1), watchdog: this.axes[a].watchdog(2, { oscSmax: this.opts.oscSmax }) }])),
      gains: this.family ? Object.fromEntries(AXES.map(a => [a, currentGains(this.family, a, this.params)])) : null,
      originals: this.originals, previous: this.previous || {}, prepared: !!this.prepared, pendingWrites: this.pendingWrites || [], vibe: this.vibe || null,
      monitor: this.monitor, sessionActive: this.sessionActive, writing: this.writing,
      identity: this.identity, linkIdentity: this.link.identity(), otherVehicles: this.link.othersSeen(), rebootPending: this.rebootPending, audit: this.audit.status(),
      changes: this.changes.slice(-10).map(c => ({ id: c.id, t: c.t, kind: c.kind, axis: c.axis, reason: c.reason, armed: c.armed, status: c.status, params: c.params })),
      journal: this.journal.slice(-50), messages: this.messages.slice(-40) };
  }
}

/** steps so that no multiplicative change of these keys exceeds stepMax (keys with a zero end do not count) */
function stepsFor(cur, target, keys, stepMax) {
  let steps = 1;
  for (const k of keys) { const a = cur[k], b = target[k]; if (a > 0 && b > 0) steps = Math.max(steps, Math.ceil(Math.abs(Math.log(b / a)) / Math.log(stepMax) - 1e-9)); }
  return steps;
}
/** value at step s of steps: geometric between positive ends, linear otherwise; the end value at the last step */
function interp(a, b, s, steps) { if (s === steps) return b; return (a > 0 && b > 0) ? a * Math.pow(b / a, s / steps) : a + (b - a) * s / steps; }
function wrap180(d) { return ((d + 540) % 360) - 180; }
function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }
function fmt(v) { return typeof v === 'number' ? +v.toPrecision(4) : v; }

module.exports = { Tuner, AXES, IN_FLIGHT_OFF, SHARED_PARAMS };
