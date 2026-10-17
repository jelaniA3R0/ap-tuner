// Live per-axis data: ring buffer of PID_TUNING samples, uniform windows for analysis, and the
// oscillation watchdog (QuikTune-style slew-rate detector + spectral check).
'use strict';
const { resample, estimateRate } = require('./analysis');
const { psd, bandRms } = require('../dsp/spectra');
const { nextPow2 } = require('../dsp/fft');

class LiveAxis {
  constructor(name, keepSeconds = 180) {
    this.name = name;
    this.keep = keepSeconds;
    this.t = []; this.target = []; this.actual = []; this.P = []; this.I = []; this.D = []; this.FF = []; this.srate = []; this.dmod = [];
    this.lastTs = 0; this.count = 0;
    this.baselineOsc = null;
  }
  push(ts, f) {
    // f: PID_TUNING fields {desired, achieved, FF, P, I, D, SRate, PDmod}
    this.t.push(ts); this.target.push(f.desired); this.actual.push(f.achieved); this.P.push(f.P); this.I.push(f.I); this.D.push(f.D); this.FF.push(f.FF);
    this.srate.push(f.SRate ?? 0); this.dmod.push(f.PDmod ?? 1);
    this.count++; this.lastTs = ts;
    const cut = ts - this.keep;
    if (this.t.length > 10 && this.t[0] < cut) {
      let k = 0; while (k < this.t.length && this.t[k] < cut) k++;
      for (const arr of [this.t, this.target, this.actual, this.P, this.I, this.D, this.FF, this.srate, this.dmod]) arr.splice(0, k);
    }
  }
  /** drop all buffered samples (e.g. after switching the controller family, when the units change) */
  clear() { for (const arr of [this.t, this.target, this.actual, this.P, this.I, this.D, this.FF, this.srate, this.dmod]) arr.length = 0; this.baselineOsc = null; }
  get seconds() { return this.t.length > 1 ? this.t[this.t.length - 1] - this.t[0] : 0; }
  get rateHz() { return estimateRate(this.t.slice(-400)); }

  /** uniform dataset over the last `seconds` (or all) */
  window(seconds = null) {
    let i0 = 0;
    if (seconds !== null) { const cut = this.lastTs - seconds; while (i0 < this.t.length && this.t[i0] < cut) i0++; }
    const t = this.t.slice(i0);
    if (t.length < 20) return null;
    const raw = estimateRate(t);
    const fs = raw >= 80 ? 100 : raw >= 40 ? 50 : raw >= 20 ? 25 : raw >= 8 ? 10 : 5;
    const sub = {};
    const u = new Float64Array(t.length);
    for (let i = 0; i < t.length; i++) u[i] = this.P[i0 + i] + this.I[i0 + i] + this.D[i0 + i] + this.FF[i0 + i];
    sub.target = Float64Array.from(this.target.slice(i0)); sub.actual = Float64Array.from(this.actual.slice(i0)); sub.u = u;
    sub.P = Float64Array.from(this.P.slice(i0)); sub.I = Float64Array.from(this.I.slice(i0)); sub.D = Float64Array.from(this.D.slice(i0)); sub.FF = Float64Array.from(this.FF.slice(i0));
    sub.srate = Float64Array.from(this.srate.slice(i0)); sub.dmod = Float64Array.from(this.dmod.slice(i0));
    const rs = resample(Float64Array.from(t), sub, fs);
    if (!rs) return null;
    return Object.assign({ fs, t0: rs.t0, n: rs.n, rawRate: raw }, rs.cols);
  }

  /** excitation over the last `seconds`: RMS of the rate target */
  excitation(seconds = 20) {
    let i0 = this.t.length - 1; const cut = this.lastTs - seconds;
    let s = 0, n = 0;
    while (i0 >= 0 && this.t[i0] >= cut) { s += this.target[i0] * this.target[i0]; n++; i0--; }
    return n ? Math.sqrt(s / n) : 0;
  }

  /**
   * Oscillation watchdog over the last `seconds`. Returns {oscillating, reasons[], srateMax, dmodMin, bandRms, peakHz}
   * thresholds: {oscSmax (QuikTune QUIK_OSC_SMAX, default 5), bandRmsDegs (default 8), ratio vs baseline (3x)}
   */
  watchdog(seconds = 2, { oscSmax = 5, bandRmsDegs = 8, dmodMin = 0.9 } = {}) {
    let i0 = this.t.length - 1; const cut = this.lastTs - seconds;
    let srMax = 0, dmMin = 1, n = 0;
    while (i0 >= 0 && this.t[i0] >= cut) { if (this.srate[i0] > srMax) srMax = this.srate[i0]; if (this.dmod[i0] < dmMin) dmMin = this.dmod[i0]; n++; i0--; }
    const reasons = [];
    if (srMax > oscSmax) reasons.push(`slew rate ${srMax.toFixed(1)} > ${oscSmax}`);
    if (dmMin < dmodMin) reasons.push(`Dmod ${dmMin.toFixed(2)} (slew limiter cutting gains)`);
    let band = 0, peakHz = 0;
    const w = this.window(Math.max(seconds, 3));
    if (w && w.n >= 32) {
      const win = Math.min(nextPow2(w.n), 256);
      if (w.n >= win) {
        const err = new Float64Array(w.n); for (let i = 0; i < w.n; i++) err[i] = w.actual[i] - w.target[i];
        const sp = psd(err, w.fs, win, 0.5);
        band = bandRms(sp, 1.5, w.fs * 0.49);
        let pk = 0; for (let i = 1; i < sp.f.length; i++) if (sp.f[i] >= 1.5 && sp.psd[i] > pk) { pk = sp.psd[i]; peakHz = sp.f[i]; }
        if (band > bandRmsDegs && (this.baselineOsc === null || band > 3 * this.baselineOsc)) reasons.push(`rate error oscillation ${band.toFixed(1)} deg/s RMS at ~${peakHz.toFixed(1)} Hz`);
      }
    }
    return { oscillating: reasons.length > 0, reasons, srateMax: srMax, dmodMin: dmMin, bandRms: band, peakHz, samples: n };
  }
  setBaseline() { const w = this.watchdog(5, { oscSmax: 1e9, bandRmsDegs: 1e9, dmodMin: -1 }); this.baselineOsc = w.bandRms || 0.5; }
}

module.exports = { LiveAxis };
