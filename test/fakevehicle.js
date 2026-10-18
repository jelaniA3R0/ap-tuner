// A scriptable fake ArduPilot copter on the app's real MAVLink Link (no sockets, no simulator). It answers
// PARAM_REQUEST_READ / PARAM_SET / COMMAND_LONG the way ArduPilot does (float32 values, whole-number parameters stored
// as ArduPilot casts them, echo of the stored value, AUTOPILOT_VERSION, reboot) and streams HEARTBEAT, PID_TUNING (three
// axes), ATTITUDE, NAV_CONTROLLER_OUTPUT, SYSTEM_TIME and ESC telemetry. A test scripts it: oscillation on an axis
// (SRate), telemetry dropouts, sparse data, attitude errors, ignored / dropped / refused / clamped writes, late or stale
// echoes, missing parameters, another ground station writing, another vehicle on the same link, a reboot.
// Same behaviour as cpp/tests/fakevehicle.hpp (the C++ tests run the same scenarios).
'use strict';
const { Encoder, Parser } = require('../src/mavlink/mavlink');

const AXIS_ID = { roll: 1, pitch: 2, yaw: 3 };
// MAV_PARAM_TYPE of the whole-number parameters (everything else is REAL32 = 9)
const INT_TYPES = { INS_HNTCH_ENABLE: 2, INS_HNTCH_MODE: 2, INS_HNTCH_HMNCS: 2, INS_HNTCH_OPTS: 2, FFT_ENABLE: 2, ATC_RATE_FF_ENAB: 2, GCS_PID_MASK: 4, SCHED_LOOP_RATE: 4, INS_GYRO_FILTER: 4 };
const INT_RANGE = { 2: [-128, 127], 4: [-32768, 32767], 6: [-2147483648, 2147483647] };

function copterParams() {
  const p = { SCHED_LOOP_RATE: 400, INS_GYRO_FILTER: 20, GCS_PID_MASK: 7, ATC_RATE_FF_ENAB: 1, MOT_THST_HOVER: 0.39, MOT_THST_EXPO: 0.65,
    INS_HNTCH_ENABLE: 0, INS_HNTCH_MODE: 1, INS_HNTCH_FREQ: 80, INS_HNTCH_BW: 40, INS_HNTCH_ATT: 40, INS_HNTCH_REF: 0, INS_HNTCH_HMNCS: 3, INS_HNTCH_OPTS: 0, INS_HNTCH_FM_RAT: 1, FFT_ENABLE: 0 };
  for (const [ax, P, I, D, E] of [['RLL', 0.135, 0.135, 0.0036, 0], ['PIT', 0.135, 0.135, 0.0036, 0], ['YAW', 0.18, 0.018, 0, 2.5]]) {
    Object.assign(p, { [`ATC_RAT_${ax}_P`]: P, [`ATC_RAT_${ax}_I`]: I, [`ATC_RAT_${ax}_D`]: D, [`ATC_RAT_${ax}_FF`]: 0, [`ATC_RAT_${ax}_FLTT`]: 20, [`ATC_RAT_${ax}_FLTE`]: E, [`ATC_RAT_${ax}_FLTD`]: 20, [`ATC_RAT_${ax}_SMAX`]: 50, [`ATC_RAT_${ax}_IMAX`]: 0.5, [`ATC_RAT_${ax}_D_FF`]: 0 });
  }
  Object.assign(p, { ATC_ANG_RLL_P: 4.5, ATC_ANG_PIT_P: 4.5, ATC_ANG_YAW_P: 4.5 });
  for (const k of Object.keys(p)) p[k] = Math.fround(p[k]);
  return p;
}

/** how ArduPilot stores a float sent for a whole-number parameter (AP_Param::set_float: +-0.01, clamp, truncate) */
function apStore(type, v) {
  const r = INT_RANGE[type];
  if (!r) return Math.fround(v);
  const x = Math.min(r[1], Math.max(r[0], v + (v < 0 ? -0.01 : 0.01)));
  return Math.trunc(x);
}

class FakeVehicle {
  constructor({ params = copterParams(), armed = true, mode = 2, type = 2, rateHz = 50, sysid = 1, uid = '0123456789abcdef01234567', fw = [4, 5, 7], gitHash = 'abcdef12' } = {}) {
    this.params = Object.assign({}, params);
    this.types = Object.assign({}, INT_TYPES);
    this.armed = armed; this.mode = mode; this.type = type; this.rateHz = rateHz;
    this.sysid = sysid; this.uid = uid; this.fw = fw; this.gitHash = gitHash;
    this.enc = new Encoder(sysid, 1); this.parser = new Parser();
    this.sets = [];               // every PARAM_SET received: {t, name, value}
    this.reads = [];              // every PARAM_REQUEST_READ received: {t, name}
    this.commands = [];           // every COMMAND_LONG received: {t, command, p1}
    this.streaming = true;        // PID_TUNING on / off (dropout)
    this.heartbeats = true;
    this.ignoreSets = false;      // never answer PARAM_SET (the link then retries)
    this.replyDelayMs = 5;
    this.echoDelayMs = null;      // delay of the echo of a PARAM_SET (null: replyDelayMs): late acknowledgements
    this.dropEchoes = 0;          // the next N writes are stored but their echo is lost
    this.dropSets = 0;            // the next N PARAM_SETs are lost on the way (not stored, no echo)
    this.staleEchoes = 0;         // the next N writes: first a PARAM_VALUE with the old value (a late answer), then the echo
    this.readOnly = new Set();    // refused: ArduPilot keeps the value and echoes it ("Param write denied")
    this.clamp = {};              // name -> [lo, hi]: stored clamped (a vehicle-side limit)
    this.missing = new Set();     // no such parameter: no answer at all
    this.hidden = new Set();      // exist only after the next reboot (no answer until then)
    this.ackForOther = false;     // COMMAND_ACKs name another ground station as the requester
    this.rebootMs = 2000;         // silent for this long after MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    this.onReboot = null;         // () => void: what the reboot changes (hidden parameters appear ...)
    this.reboots = 0;
    this.slew = { roll: 0.5, pitch: 0.5, yaw: 0.5 };   // SRate per axis (above OSC_SMAX 5 = oscillating)
    this.dmod = { roll: 1, pitch: 1, yaw: 1 };
    this.desiredAmp = 0.2;        // rad/s (deg/s on a plane): a slow sine as the rate target
    this.onSet = null;            // (name, value) => void: the scripted reaction of the airframe to a write
    this.attitude = null;         // {roll, pitch} deg: ATTITUDE at 25 Hz
    this.navTarget = null;        // {roll, pitch} deg: NAV_CONTROLLER_OUTPUT at 5 Hz
    this.escRpm = 0;              // > 0: ESC_TELEMETRY_1_TO_4 at 5 Hz
    this.sysTimeHz = 2;           // SYSTEM_TIME (time_boot_ms: the boot clock)
    this.timers = [];
    this._down = false;
  }

  /** attach to the link as its only vehicle (bus: share one link with other fake vehicles, see joinBus) */
  attach(link, bus = null) {
    this.link = link;
    if (bus) { bus.push(this); if (bus.length === 1) link.attachTransport({ send: buf => { for (const v of bus.slice()) v._onSend(buf); }, close: () => {} }, 'fake vehicles'); }
    else link.attachTransport({ send: buf => this._onSend(buf), close: () => {} }, 'fake vehicle');
    this._t0 = Date.now(); this._nextPid = Date.now(); this._nextAtt = Date.now(); this._nextSlow = Date.now(); this._nextSys = Date.now();
    this._hb();
    this.timers.push(setInterval(() => this._hb(), 1000));
    this.timers.push(setInterval(() => this._tick(), 5));
    return this;
  }
  stop() { for (const t of this.timers) clearInterval(t); this.timers = []; if (this.link) this.link.disconnect(); }
  /** stop this vehicle's timers only (the link stays: other vehicles on the bus go on) */
  silence() { for (const t of this.timers) clearInterval(t); this.timers = []; }
  feed(name, fields) { if (this.link && this.link.transport && !this._down) this.link.feed(this.enc.pack(name, fields)); }
  setsOf(name) { return this.sets.filter(s => s.name === name); }
  typeOf(name) { return this.types[name] || 9; }

  /** another ground station writes a value (ArduPilot announces the change to every channel) */
  foreignSet(name, value) {
    this.params[name] = apStore(this.typeOf(name), value);
    this._value(name, -1);
  }

  _hb() { if (this.heartbeats) this.feed('HEARTBEAT', { type: this.type, autopilot: 3, base_mode: this.armed ? 129 : 1, custom_mode: this.mode, system_status: this.armed ? 4 : 3, mavlink_version: 3 }); }
  _tick() {
    const now = Date.now();
    if (this._down) { this._nextPid = now; return; }
    if (!this.streaming) this._nextPid = now;
    // PID_TUNING at rateHz on average (timers are coarse on Windows: catch up in bursts)
    for (let n = 0; this.streaming && this._nextPid <= now && n < 20; n++) {
      this._nextPid += 1000 / this.rateHz;
      const t = (now - this._t0) / 1000;
      for (const ax of ['roll', 'pitch', 'yaw']) {
        const des = this.desiredAmp * Math.sin(2 * Math.PI * 0.5 * t + AXIS_ID[ax]);
        this.feed('PID_TUNING', { axis: AXIS_ID[ax], desired: des, achieved: 0.9 * des, FF: 0, P: 0.01 * des, I: 0, D: 0, SRate: this.slew[ax], PDmod: this.dmod[ax] });
      }
    }
    if (this.attitude && this._nextAtt <= now) { this._nextAtt = now + 40; const d = Math.PI / 180; this.feed('ATTITUDE', { time_boot_ms: now - this._t0, roll: this.attitude.roll * d, pitch: this.attitude.pitch * d, yaw: 0, rollspeed: 0, pitchspeed: 0, yawspeed: 0 }); }
    if (this.sysTimeHz > 0 && this._nextSys <= now) { this._nextSys = now + 1000 / this.sysTimeHz; this.feed('SYSTEM_TIME', { time_unix_usec: now * 1000, time_boot_ms: now - this._t0 }); }
    if (this._nextSlow <= now) {
      this._nextSlow = now + 200;
      if (this.navTarget) this.feed('NAV_CONTROLLER_OUTPUT', { nav_roll: this.navTarget.roll, nav_pitch: this.navTarget.pitch, nav_bearing: 0, target_bearing: 0, wp_dist: 0, alt_error: 0, aspd_error: 0, xtrack_error: 0 });
      if (this.escRpm > 0) this.feed('ESC_TELEMETRY_1_TO_4', { temperature: [30, 30, 30, 30], voltage: [1600, 1600, 1600, 1600], current: [500, 500, 500, 500], totalcurrent: [0, 0, 0, 0], rpm: [this.escRpm, this.escRpm, this.escRpm, this.escRpm], count: [1, 1, 1, 1] });
    }
  }
  _exists(name) { return !this.missing.has(name) && !this.hidden.has(name); }
  _value(name, index = 1, value = undefined) {
    const v = value !== undefined ? value : (this.params[name] !== undefined ? this.params[name] : 0);   // an unknown name reads as 0 (keeps prepare quick)
    this.feed('PARAM_VALUE', { param_id: name, param_value: v, param_type: this.typeOf(name), param_count: 900, param_index: index < 0 ? 65535 : index });   // ArduPilot: -1 for a change it announces
  }
  _reply(name, delay = this.replyDelayMs) { setTimeout(() => this._value(name), delay); }
  _set(name, value) {
    this.sets.push({ t: Date.now(), name, value });
    if (this.ignoreSets || !this._exists(name)) return;
    if (this.dropSets > 0) { this.dropSets--; return; }
    const old = this.params[name] !== undefined ? this.params[name] : 0;
    const delay = this.echoDelayMs !== null ? this.echoDelayMs : this.replyDelayMs;
    if (this.readOnly.has(name)) {
      this.feed('STATUSTEXT', { severity: 4, text: `Param write denied (${name})` });
      setTimeout(() => this._value(name, -1), delay);
      return;
    }
    let v = apStore(this.typeOf(name), value);
    if (this.clamp[name]) v = Math.fround(Math.min(this.clamp[name][1], Math.max(this.clamp[name][0], v)));
    this.params[name] = v;
    if (this.staleEchoes > 0) { this.staleEchoes--; setTimeout(() => this._value(name, 1, old), 1); }
    if (this.dropEchoes > 0) this.dropEchoes--;
    else setTimeout(() => this._value(name, -1), delay);
    if (this.onSet) this.onSet(name, this.params[name]);
  }
  _command(m) {
    const f = m.fields;
    this.commands.push({ t: Date.now(), command: f.command, p1: f.param1 });
    let result = 0;
    if (f.command === 246) result = this.armed ? 4 : 0;   // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: refused while armed (MAV_RESULT_FAILED)
    const ack = { command: f.command, result, progress: 0, result_param2: 0, target_system: this.ackForOther ? 254 : m.sysid, target_component: this.ackForOther ? 1 : m.compid };
    setTimeout(() => this.feed('COMMAND_ACK', ack), this.replyDelayMs);
    if ((f.command === 512 && f.param1 === 148) || f.command === 520) setTimeout(() => this._autopilotVersion(), this.replyDelayMs);
    if (f.command === 246 && result === 0) setTimeout(() => this._reboot(), this.replyDelayMs + 20);
  }
  _autopilotVersion() {
    const uid2 = new Array(18).fill(0);
    for (let i = 0; i < this.uid.length / 2 && i < 18; i++) uid2[i] = parseInt(this.uid.substr(2 * i, 2), 16);
    const git = Array.from(this.gitHash.slice(0, 8)).map(c => c.charCodeAt(0)); while (git.length < 8) git.push(0);
    this.feed('AUTOPILOT_VERSION', { capabilities: 0, flight_sw_version: ((this.fw[0] << 24) | (this.fw[1] << 16) | (this.fw[2] << 8) | 255) >>> 0, middleware_sw_version: 0, os_sw_version: 0, board_version: 0x8c0000,
      flight_custom_version: git, middleware_custom_version: [0, 0, 0, 0, 0, 0, 0, 0], os_custom_version: [0, 0, 0, 0, 0, 0, 0, 0], vendor_id: 0x2dae, product_id: 0x1016, uid: 0, uid2 });
  }
  _reboot() {
    this._down = true; this.reboots++;
    setTimeout(() => {
      this.hidden.clear();
      if (this.onReboot) this.onReboot();
      this._t0 = Date.now(); this._down = false; this._hb();
    }, this.rebootMs);
  }
  _onSend(buf) {
    for (const m of this.parser.feed(Buffer.from(buf))) {
      const f = m.fields;
      if (f.target_system !== undefined && f.target_system !== 0 && f.target_system !== this.sysid) continue;   // for another vehicle
      if (this._down) continue;   // rebooting: nothing is heard
      if (m.name === 'PARAM_REQUEST_READ') { this.reads.push({ t: Date.now(), name: f.param_id }); if (this._exists(f.param_id)) this._reply(f.param_id); }
      else if (m.name === 'PARAM_SET') this._set(f.param_id, f.param_value);
      else if (m.name === 'COMMAND_LONG') this._command(m);
    }
  }
}

module.exports = { FakeVehicle, copterParams, apStore };
