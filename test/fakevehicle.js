// A scriptable fake ArduPilot copter on the app's real MAVLink Link (no sockets, no simulator). It answers
// PARAM_REQUEST_READ / PARAM_SET / COMMAND_LONG the way ArduPilot does (float32 values, echo of the stored value)
// and streams HEARTBEAT, PID_TUNING (three axes), ATTITUDE, NAV_CONTROLLER_OUTPUT and ESC telemetry. A test
// scripts it: oscillation on an axis (SRate), telemetry dropouts, sparse data, attitude errors, ignored writes.
// Same behaviour as cpp/tests/fakevehicle.hpp (the C++ tests run the same scenarios).
'use strict';
const { Encoder, Parser } = require('../src/mavlink/mavlink');

const AXIS_ID = { roll: 1, pitch: 2, yaw: 3 };

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

class FakeVehicle {
  constructor({ params = copterParams(), armed = true, mode = 2, type = 2, rateHz = 50 } = {}) {
    this.params = Object.assign({}, params);
    this.armed = armed; this.mode = mode; this.type = type; this.rateHz = rateHz;
    this.enc = new Encoder(1, 1); this.parser = new Parser();
    this.sets = [];               // every PARAM_SET received: {t, name, value}
    this.streaming = true;        // PID_TUNING on / off (dropout)
    this.heartbeats = true;
    this.ignoreSets = false;      // never answer PARAM_SET (the link then retries)
    this.replyDelayMs = 5;
    this.slew = { roll: 0.5, pitch: 0.5, yaw: 0.5 };   // SRate per axis (above OSC_SMAX 5 = oscillating)
    this.dmod = { roll: 1, pitch: 1, yaw: 1 };
    this.desiredAmp = 0.2;        // rad/s (deg/s on a plane): a slow sine as the rate target
    this.onSet = null;            // (name, value) => void: the scripted reaction of the airframe to a write
    this.attitude = null;         // {roll, pitch} deg: ATTITUDE at 25 Hz
    this.navTarget = null;        // {roll, pitch} deg: NAV_CONTROLLER_OUTPUT at 5 Hz
    this.escRpm = 0;              // > 0: ESC_TELEMETRY_1_TO_4 at 5 Hz
    this.timers = [];
  }

  attach(link) {
    this.link = link;
    link.attachTransport({ send: buf => this._onSend(buf), close: () => {} }, 'fake vehicle');
    this._t0 = Date.now(); this._nextPid = Date.now(); this._nextAtt = Date.now(); this._nextSlow = Date.now();
    this._hb();
    this.timers.push(setInterval(() => this._hb(), 1000));
    this.timers.push(setInterval(() => this._tick(), 5));
    return this;
  }
  stop() { for (const t of this.timers) clearInterval(t); this.timers = []; if (this.link) this.link.disconnect(); }
  feed(name, fields) { if (this.link && this.link.transport) this.link.feed(this.enc.pack(name, fields)); }
  setsOf(name) { return this.sets.filter(s => s.name === name); }

  _hb() { if (this.heartbeats) this.feed('HEARTBEAT', { type: this.type, autopilot: 3, base_mode: this.armed ? 129 : 1, custom_mode: this.mode, system_status: this.armed ? 4 : 3, mavlink_version: 3 }); }
  _tick() {
    const now = Date.now();
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
    if (this._nextSlow <= now) {
      this._nextSlow = now + 200;
      if (this.navTarget) this.feed('NAV_CONTROLLER_OUTPUT', { nav_roll: this.navTarget.roll, nav_pitch: this.navTarget.pitch, nav_bearing: 0, target_bearing: 0, wp_dist: 0, alt_error: 0, aspd_error: 0, xtrack_error: 0 });
      if (this.escRpm > 0) this.feed('ESC_TELEMETRY_1_TO_4', { temperature: [30, 30, 30, 30], voltage: [1600, 1600, 1600, 1600], current: [500, 500, 500, 500], totalcurrent: [0, 0, 0, 0], rpm: [this.escRpm, this.escRpm, this.escRpm, this.escRpm], count: [1, 1, 1, 1] });
    }
  }
  _reply(name) {
    const v = this.params[name] !== undefined ? this.params[name] : 0;   // an unknown name reads as 0 (keeps prepare quick)
    setTimeout(() => this.feed('PARAM_VALUE', { param_id: name, param_value: v, param_type: 9, param_count: 900, param_index: 1 }), this.replyDelayMs);
  }
  _onSend(buf) {
    for (const m of this.parser.feed(Buffer.from(buf))) {
      const f = m.fields;
      if (m.name === 'PARAM_REQUEST_READ') this._reply(f.param_id);
      else if (m.name === 'PARAM_SET') {
        this.sets.push({ t: Date.now(), name: f.param_id, value: f.param_value });
        if (this.ignoreSets) continue;
        this.params[f.param_id] = Math.fround(f.param_value);
        this._reply(f.param_id);
        if (this.onSet) this.onSet(f.param_id, this.params[f.param_id]);
      } else if (m.name === 'COMMAND_LONG') setTimeout(() => this.feed('COMMAND_ACK', { command: f.command, result: 0 }), this.replyDelayMs);
    }
  }
}

module.exports = { FakeVehicle, copterParams };
