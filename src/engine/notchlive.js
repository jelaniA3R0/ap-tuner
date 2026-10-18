// Live harmonic-notch setup through the vehicle itself:
//  1) the autopilot's in-flight FFT notch tune (RC option 162, driven here over MAVLink), which makes
//     the vehicle measure its own motor noise in hover and write INS_HNTCH_* (needs FFT_ENABLE = 1 and
//     an FFT-capable board such as the Cube Orange / H7 class);
//  2) ESC RPM telemetry: when the ESCs report RPM, the notch can simply track them (mode 3);
//  3) otherwise: the log route (raw gyro logging + Log tab).
// Values that act only after a reboot (INS_HNTCH_REF / HMNCS / OPTS, FFT_ENABLE) are written on the ground only;
// the reboot is then done through the app and every value is read again after it (2026-10-17 audit F-35).
'use strict';
const { rebootAndVerify } = require('./paramwrite');
const { isRebootRequired } = require('../model/parammeta');
const { sameF32 } = require('./identity');
const NOTCH_PARAMS = ['INS_HNTCH_ENABLE', 'INS_HNTCH_MODE', 'INS_HNTCH_FREQ', 'INS_HNTCH_BW', 'INS_HNTCH_ATT', 'INS_HNTCH_REF', 'INS_HNTCH_HMNCS', 'INS_HNTCH_OPTS', 'INS_HNTCH_FM_RAT', 'INS_GYRO_FILTER', 'FFT_ENABLE', 'FFT_MINHZ', 'FFT_MAXHZ'];
const AUX_FFT_NOTCH_TUNE = 162;

class NotchLive {
  constructor(link, tuner) {
    this.link = link; this.tuner = tuner;
    this.esc = { seen: false, rpm: [], lastTs: 0, hoverRpm: null };
    this.fftMessages = [];
    this.tune = null;
    link.on('ESC_TELEMETRY_1_TO_4', (f, m) => this._esc(f, m));
    link.on('ESC_TELEMETRY_5_TO_8', (f, m) => this._esc(f, m));
    link.on('STATUSTEXT', f => { if (/^FFT/.test(f.text)) { this.fftMessages.push({ t: Date.now(), text: f.text }); if (this.fftMessages.length > 30) this.fftMessages.shift(); } });
  }
  _esc(f, m) {
    const rpm = f.rpm.filter(v => v > 0);
    if (!rpm.length) return;
    this.esc.seen = true; this.esc.lastTs = m.ts;
    this.esc.rpm.push({ t: m.ts, mean: rpm.reduce((a, b) => a + b, 0) / rpm.length, n: rpm.length });
    if (this.esc.rpm.length > 600) this.esc.rpm.shift();
  }

  /** read the notch-related parameters and summarise what is possible */
  async status() {
    const got = await this.link.requestParams(NOTCH_PARAMS, 3);
    const p = Object.assign(this.tuner.params, got);
    const fftCapable = got.FFT_ENABLE !== undefined;
    const now = Date.now();
    const recent = this.esc.rpm.filter(r => now - r.t < 20000);
    const escRpm = recent.length ? recent.reduce((a, b) => a + b.mean, 0) / recent.length : null;
    const out = {
      current: Object.fromEntries(NOTCH_PARAMS.map(k => [k, p[k]])),
      fftCapable, fftEnabled: fftCapable && p.FFT_ENABLE > 0,
      escTelemetry: this.esc.seen && now - this.esc.lastTs < 20000, escRpm,
      notchActive: p.INS_HNTCH_ENABLE > 0 && !this.rebootNeeded().some(n => /^INS_HNTCH_/.test(n)), mode: p.INS_HNTCH_MODE | 0,
      rebootPending: this.rebootNeeded(),
      armed: !!this.link.vehicle?.armed,
      messages: this.fftMessages.slice(-8),
      options: [],
    };
    if (out.escTelemetry) {
      const f0 = escRpm / 60;
      out.escProposal = { INS_HNTCH_ENABLE: 1, INS_HNTCH_MODE: 3, INS_HNTCH_FREQ: +Math.max(10, f0 * 0.5).toFixed(1), INS_HNTCH_BW: +Math.max(5, f0 * 0.25).toFixed(1), INS_HNTCH_ATT: 40, INS_HNTCH_REF: 1, INS_HNTCH_HMNCS: 3, INS_HNTCH_OPTS: 0, INS_HNTCH_FM_RAT: 1 };
      out.options.push({ id: 'esc', title: 'Track the ESC RPM telemetry (recommended)', detail: `Your ESCs report RPM (about ${Math.round(escRpm)} rpm now, ${f0.toFixed(1)} Hz). The notch follows the motors exactly, no measurement flight needed.` });
    }
    if (fftCapable) out.options.push({ id: 'fft', title: out.fftEnabled ? 'Measure the motor noise with the autopilot\'s own FFT' : 'Enable the autopilot\'s FFT (needs one reboot), then measure', detail: 'In a steady hover the autopilot runs an FFT on its raw gyro for 30 s and writes the notch frequency, bandwidth, reference throttle and harmonics itself (ArduPilot RC option 162).' });
    out.options.push({ id: 'log', title: 'Design from a log file', detail: 'Log raw gyro (INS_RAW_LOG_OPT = 9) on a 1-2 minute hover, then use the Log tab. Most precise, and it shows you the noise spectrum.' });
    if (!out.options.some(o => o.id !== 'log') && !fftCapable && !out.escTelemetry) out.note = 'This autopilot reports neither an FFT nor ESC RPM telemetry: use the log route.';
    return out;
  }

  /** names written on the ground that take effect only after a reboot */
  rebootNeeded() { return (this.tuner.rebootPending || []).map(p => p.name); }
  /** the values of a change that act only after a reboot and differ from what the vehicle holds */
  rebootOnly(values) { return Object.keys(values).filter(k => isRebootRequired(k) && !sameF32(this.tuner.params[k], values[k])); }

  /**
   * apply the ESC-telemetry notch (in flight: watched on every axis and reverted if the vehicle starts to oscillate;
   * values that act only after a reboot are refused in flight: set the notch on the ground, then reboot)
   */
  async applyEsc() {
    const st = await this.status();
    if (!st.escProposal) throw new Error('no ESC RPM telemetry seen in the last 20 s');
    const held = await this.tuner.applyFilterChange(st.escProposal, 'notch: ESC RPM tracking');
    if (!held) throw Object.assign(new Error('the notch was not kept: ' + (this.tuner.stateInfo || 'see the messages')), { code: 'reverted' });
    return st.escProposal;
  }

  /** set FFT_ENABLE = 1 on the ground (takes effect after a reboot: the reboot workflow checks it) */
  async enableFft() {
    const held = await this.tuner.applyFilterChange({ FFT_ENABLE: 1 }, 'notch: enable in-flight FFT');
    if (!held) throw Object.assign(new Error('FFT_ENABLE was not kept: ' + (this.tuner.stateInfo || 'see the messages')), { code: 'reverted' });
    return { FFT_ENABLE: this.tuner.params.FFT_ENABLE, rebootNeeded: true };
  }

  /**
   * reboot the autopilot (refused while armed) and check it: it must come back as the same vehicle and every value
   * written on the ground that waits for the reboot is read again ({ok, reason, rows})
   */
  async reboot() {
    if (!this.link.alive) throw new Error('no vehicle heartbeat: connect first');
    if (this.link.vehicle?.armed) throw new Error('refusing to reboot an armed vehicle');
    if (this.tuner.writing) throw new Error(`a change is being written (${this.tuner.writing}): wait until it has finished`);
    const expect = (this.tuner.rebootPending || []).map(p => ({ name: p.name, value: p.value }));
    const res = await rebootAndVerify(this.link, expect, { journal: (k, d) => this.tuner.audit.append(k, d), ctx: { via: 'notch panel' } });
    if (res.ok) this.tuner.rebootPending = [];
    this.tuner.log('tuner', res.ok ? (expect.length ? 'rebooted: ' + expect.map(e => e.name).join(', ') + ' read back after the reboot' : 'rebooted') : 'reboot: ' + res.reason);
    return { ok: res.ok, reason: res.reason, rows: res.rows, rebooted: res.rebooted };
  }

  /** start the in-flight FFT notch tune (vehicle must hover steadily for ~30 s) */
  async startFftTune() {
    if (!this.link.vehicle?.armed) throw new Error('the vehicle must be flying (armed, steady hover)');
    const st = await this.status();
    if (!st.fftEnabled) throw new Error('FFT_ENABLE is 0: enable it and reboot first');
    const want = {};
    if (!(this.tuner.params.INS_HNTCH_ENABLE > 0)) want.INS_HNTCH_ENABLE = 1;
    if (st.mode !== 1) want.INS_HNTCH_MODE = 1;
    if (Object.keys(want).length && !(await this.tuner.applyFilterChange(want, 'notch: throttle tracking for the FFT tune'))) throw Object.assign(new Error('the notch was not kept: ' + (this.tuner.stateInfo || 'see the messages')), { code: 'reverted' });
    this.fftMessages = [];
    const before = Object.fromEntries(NOTCH_PARAMS.map(k => [k, this.tuner.params[k]]));
    const r = await this.link.commandAck(218, [AUX_FFT_NOTCH_TUNE, 2]).catch(() => ({ result: -1 }));
    if (r.result !== 0) throw new Error('the autopilot did not accept the FFT tune command (result ' + r.result + '); firmware may lack the GyroFFT feature');
    this.tune = { startedAt: Date.now(), done: false, before };
    this.tuner.expectExternal = 'written by the autopilot FFT notch tune';
    return { started: true };
  }

  /** stop the FFT tune, then read back what the vehicle wrote */
  async stopFftTune() {
    const r = await this.link.commandAck(218, [AUX_FFT_NOTCH_TUNE, 0]).catch(() => ({ result: -1 }));
    await new Promise(res => setTimeout(res, 2500));
    // compared with the values from before the tune (the cache follows the vehicle's own announcements)
    const before = Object.assign({}, this.tuner.params, (this.tune && this.tune.before) || {});
    const got = await this.link.requestParams(NOTCH_PARAMS, 3);
    this.tuner.expectExternal = null;
    Object.assign(this.tuner.params, got);
    const changed = {};
    for (const k of NOTCH_PARAMS) if (got[k] !== undefined && got[k] !== before[k]) changed[k] = { from: before[k], to: got[k] };
    for (const [k, c] of Object.entries(changed)) {
      const e = { t: Date.now(), name: k, from: c.from, to: c.to, reason: 'written by the autopilot FFT notch tune', armed: !!this.link.vehicle?.armed, status: 'by the autopilot' };
      this.tuner.journal.push(e); this.tuner.audit.append('write', Object.assign({ by: 'autopilot', identity: this.tuner.identity ? this.tuner.identity.key : null }, e));
    }
    this.tuner.saveSession();
    this.tune = { done: true, result: r.result, changed, messages: this.fftMessages.slice(-8) };
    // what the autopilot wrote in flight is watched like the app's own changes (reverted on a trip)
    const held = await this.tuner.watchExternalChange(changed, 'written by the autopilot FFT notch tune');
    const ok = held && (Object.keys(changed).length > 0 || this.fftMessages.some(m => /selected/.test(m.text)));
    return { ok, held, changed, messages: this.fftMessages.slice(-8), current: Object.fromEntries(NOTCH_PARAMS.map(k => [k, this.tuner.params[k]])) };
  }
}

module.exports = { NotchLive, NOTCH_PARAMS };
