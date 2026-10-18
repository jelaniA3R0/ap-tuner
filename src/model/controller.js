// Frequency-domain models of the ArduPilot control chain, evaluated on an arbitrary grid of
// frequencies (Hz). Everything returns complex arrays {re, im}.
//  - AC_PID (ArduCopter/ArduPlane rate PID): target LPF (FLTT), error LPF (FLTE), D on filtered
//    error with its own LPF (FLTD), integrator, FF on filtered target. Discrete at the loop rate.
//    This is a direct transcription of AC_PID::update_all (libraries/AC_PID/AC_PID.cpp).
//  - Gyro chain: 2-pole low-pass INS_GYRO_FILTER (DigitalBiquad, as AP_InertialSensor uses) +
//    harmonic notch sets (INS_HNTCH_*, INS_HNTC2_*), discrete at the gyro backend rate.
'use strict';

const C = {
  of(n) { return { re: new Float64Array(n), im: new Float64Array(n) }; },
  mul(a, b) { const n = a.re.length, o = C.of(n); for (let i = 0; i < n; i++) { o.re[i] = a.re[i] * b.re[i] - a.im[i] * b.im[i]; o.im[i] = a.re[i] * b.im[i] + a.im[i] * b.re[i]; } return o; },
  div(a, b) { const n = a.re.length, o = C.of(n); for (let i = 0; i < n; i++) { const d = b.re[i] * b.re[i] + b.im[i] * b.im[i] || 1e-300; o.re[i] = (a.re[i] * b.re[i] + a.im[i] * b.im[i]) / d; o.im[i] = (a.im[i] * b.re[i] - a.re[i] * b.im[i]) / d; } return o; },
  add(a, b) { const n = a.re.length, o = C.of(n); for (let i = 0; i < n; i++) { o.re[i] = a.re[i] + b.re[i]; o.im[i] = a.im[i] + b.im[i]; } return o; },
  scale(a, k) { const n = a.re.length, o = C.of(n); for (let i = 0; i < n; i++) { o.re[i] = a.re[i] * k; o.im[i] = a.im[i] * k; } return o; },
  addReal(a, k) { const o = C.of(a.re.length); for (let i = 0; i < a.re.length; i++) { o.re[i] = a.re[i] + k; o.im[i] = a.im[i]; } return o; },
  one(n) { const o = C.of(n); o.re.fill(1); return o; },
  abs(a) { const n = a.re.length, o = new Float64Array(n); for (let i = 0; i < n; i++) o[i] = Math.hypot(a.re[i], a.im[i]); return o; },
  phaseDeg(a) { const n = a.re.length, o = new Float64Array(n); for (let i = 0; i < n; i++) o[i] = Math.atan2(a.im[i], a.re[i]) * 180 / Math.PI; return o; },
  // z^-k for each frequency at sample rate fs
  zinv(freqs, fs, k = 1) { const n = freqs.length, o = C.of(n); for (let i = 0; i < n; i++) { const th = -2 * Math.PI * freqs[i] / fs * k; o.re[i] = Math.cos(th); o.im[i] = Math.sin(th); } return o; },
  delay(freqs, Td) { const n = freqs.length, o = C.of(n); for (let i = 0; i < n; i++) { const th = -2 * Math.PI * freqs[i] * Td; o.re[i] = Math.cos(th); o.im[i] = Math.sin(th); } return o; },
};

// ArduPilot first-order LPF: y += alpha (x - y), alpha = dt/(dt + 1/(2 pi fc)); fc<=0 -> pass-through
function lpf1(freqs, fs, fc) {
  const n = freqs.length;
  if (!(fc > 0)) return C.one(n);
  const dt = 1 / fs, rc = 1 / (2 * Math.PI * fc), alpha = dt / (dt + rc);
  // H(z) = alpha / (1 - (1-alpha) z^-1)
  const z1 = C.zinv(freqs, fs);
  const den = C.addReal(C.scale(z1, -(1 - alpha)), 1);
  const num = C.of(n); num.re.fill(alpha);
  return C.div(num, den);
}

// 2-pole Butterworth low-pass as in ArduPilot DigitalBiquadFilter (LowPassFilter2p)
function biquadLpf(freqs, fs, fc) {
  const n = freqs.length;
  if (!(fc > 0) || fc >= fs / 2) return C.one(n);
  const fr = fs / fc, ohm = Math.tan(Math.PI / fr), c = 1 + 2 * Math.cos(Math.PI / 4) * ohm + ohm * ohm;
  const b0 = ohm * ohm / c, b1 = 2 * b0, b2 = b0, a1 = 2 * (ohm * ohm - 1) / c, a2 = (1 - 2 * Math.cos(Math.PI / 4) * ohm + ohm * ohm) / c;
  return biquad(freqs, fs, b0, b1, b2, 1, a1, a2);
}

function biquad(freqs, fs, b0, b1, b2, a0, a1, a2) {
  const z1 = C.zinv(freqs, fs, 1), z2 = C.zinv(freqs, fs, 2);
  const num = C.addReal(C.add(C.scale(z1, b1), C.scale(z2, b2)), b0);
  const den = C.addReal(C.add(C.scale(z1, a1), C.scale(z2, a2)), a0);
  return C.div(num, den);
}

/** quality factor of an ArduPilot notch from its centre and bandwidth (NotchFilter::calculate_A_and_Q) */
function notchQ(center, bw) {
  const octaves = Math.log2(center / (center - bw / 2)) * 2;
  return Math.sqrt(Math.pow(2, octaves)) / (Math.pow(2, octaves) - 1);
}

// ArduPilot NotchFilter (libraries/Filter/NotchFilter.cpp): center, bandwidth, attenuation dB
function notch(freqs, fs, center, bw, attDb) {
  const n = freqs.length;
  if (!(center > 0.5 * bw) || !(center < 0.5 * fs)) return C.one(n);
  return notchAQ(freqs, fs, center, notchQ(center, bw), attDb);
}

/** NotchFilter with a given Q (NotchFilter::init_with_A_and_Q): how a moving harmonic notch is updated */
function notchAQ(freqs, fs, center, Q, attDb) {
  const n = freqs.length;
  if (!(center > 0) || !(center < 0.5 * fs) || !(Q > 0)) return C.one(n);
  const A = Math.pow(10, -attDb / 40);
  const omega = 2 * Math.PI * center / fs, alpha = Math.sin(omega) / (2 * Q);
  const b0 = 1 + alpha * A * A, b1 = -2 * Math.cos(omega), b2 = 1 - alpha * A * A;
  const a0 = 1 + alpha, a1 = b1, a2 = 1 - alpha;
  return biquad(freqs, fs, b0, b1, b2, a0, a1, a2);
}

/**
 * Harmonic notch set as ArduPilot builds it (HarmonicNotchFilter.cpp): base frequency given by the
 * caller (already resolved for the operating point: throttle/RPM/FFT), harmonics bitmask, options
 * (bit0 double, bit4 triple), bandwidth, attenuation.
 * refFreq / refBw: the configured INS_HNTCH_FREQ / BW. The firmware computes A, Q and the spread of the
 * double / triple notches once from them (HarmonicNotchFilter::init) and only moves the centres when the
 * operating point changes, so a notch that tracks to a higher frequency keeps its Q (it gets wider in Hz).
 * Without refFreq the notch is treated as sitting at its configured frequency.
 */
function harmonicNotch(freqs, fs, { enable, freq, bw, att = 40, hmncs = 3, opts = 0, chained = 1, refFreq = 0, refBw = 0 }) {
  let H = C.one(freqs.length);
  if (!enable || !(freq > 0)) return H;
  const composite = (opts & 1) ? 2 : (opts & 16) ? 3 : 1;
  const f0 = refFreq > 0 ? refFreq : freq, b0 = refFreq > 0 && refBw > 0 ? refBw : bw;
  if (!(f0 > 0.5 * b0 / composite)) return H;
  const Q = notchQ(f0, b0 / composite), spread = b0 / (32 * f0);
  for (let h = 0; h < 8; h++) {
    if (!(hmncs & (1 << h))) continue;
    const mul = h + 1;
    let center = freq * mul; const bwh = bw * mul;
    const nyq = fs * 0.48, bwl = bwh * 0.52;
    center = Math.min(Math.max(center, bwl), nyq);
    for (let c = 0; c < chained; c++) {
      if (composite !== 2 && center < nyq) H = C.mul(H, notchAQ(freqs, fs, center, Q, att));
      if (composite > 1) {
        const lo = center * (1 - spread), hi = center * (1 + spread);
        if (lo < nyq) H = C.mul(H, notchAQ(freqs, fs, lo, Q, att));
        if (hi < nyq) H = C.mul(H, notchAQ(freqs, fs, hi, Q, att));
      }
    }
  }
  return H;
}

/**
 * The notch base frequency at an operating point, as AP_Vehicle::update_dynamic_notch sets it:
 *  mode 0 (fixed): FREQ; REF = 0 also keeps the notch at FREQ (dynamic updates off)
 *  mode 1 (throttle): FREQ * max(FM_RAT, sqrt(throttle / REF))
 *  mode 2 / 5 (RPM sensor 1 / 2): max(FREQ * FM_RAT, rpm / 60 * REF)
 *  mode 3 (ESC telemetry): max(FREQ * FM_RAT, average motor rpm / 60 * REF)
 *  mode 4 (in-flight FFT): max(FREQ * FM_RAT, measured noise frequency)
 * FREQ * FM_RAT is the lowest frequency the firmware lets the notch go. Without a measurement (no RPM / ESC /
 * FFT data) the notch is modelled at that floor.
 * p: {mode, freq, ref, fm_rat}; op: {throttle (0..1), rpm, rpm2 (RPM sensors 1 / 2), escRpm (rpm), fftHz}
 */
function notchBaseFreq(p, op = {}) {
  const mode = p.mode | 0;
  if (mode === 0 || !(p.ref > 0)) return p.freq;
  const fmRat = p.fm_rat ?? 1, floor = p.freq * fmRat;
  if (mode === 1) { const thr = Math.max(0, op.throttle ?? p.ref); return p.freq * Math.max(fmRat, Math.sqrt(thr / p.ref)); }
  if (mode === 2) return op.rpm > 0 ? Math.max(floor, op.rpm / 60 * p.ref) : floor;
  if (mode === 5) return op.rpm2 > 0 ? Math.max(floor, op.rpm2 / 60 * p.ref) : floor;
  if (mode === 3) return op.escRpm > 0 ? Math.max(floor, op.escRpm / 60 * p.ref) : floor;
  if (mode === 4) return op.fftHz > 0 ? Math.max(floor, op.fftHz) : floor;
  return p.freq;
}

/** Full gyro filter chain response at the backend rate: LPF + notch sets */
function gyroChain(freqs, { backendHz = 1000, gyroFilterHz = 20, notches = [] }) {
  let H = biquadLpf(freqs, backendHz, gyroFilterHz);
  for (const nset of notches) H = C.mul(H, harmonicNotch(freqs, backendHz, nset));
  return H;
}

/**
 * AC_PID pieces at loop rate fs for gains g = {P,I,D,FF,DFF,FLTT,FLTE,FLTD}.
 * Returns { T (target filter), Cfb (error -> P+I+D), Cff (filtered target -> FF+DFF), parts }
 */
function acPid(freqs, fs, g) {
  const n = freqs.length, dt = 1 / fs;
  const T = lpf1(freqs, fs, g.FLTT);
  const E = lpf1(freqs, fs, g.FLTE);
  const Dlp = lpf1(freqs, fs, g.FLTD);
  const z1 = C.zinv(freqs, fs);
  const oneLessZ1 = C.addReal(C.scale(z1, -1), 1);          // (1 - z^-1)
  // integrator: I*dt * 1/(1 - z^-1)   (AC_PID integrates the filtered error each loop)
  const Iz = C.div(C.one(n), oneLessZ1);
  const P = C.scale(E, g.P);
  const I = C.scale(C.mul(E, Iz), g.I * dt);
  const D = C.scale(C.mul(C.mul(E, oneLessZ1), Dlp), g.D / dt);
  const Cfb = C.add(C.add(P, I), D);
  const DFF = C.scale(oneLessZ1, (g.DFF || 0) / dt);
  const Cff = C.addReal(DFF, g.FF || 0);
  return { T, Cfb, Cff, parts: { P, I, D } };
}

module.exports = { C, lpf1, biquadLpf, notch, notchQ, notchAQ, harmonicNotch, notchBaseFreq, gyroChain, acPid };
