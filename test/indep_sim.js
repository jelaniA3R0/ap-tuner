// Independent closed-loop simulator of one multirotor rate axis, written for the audit from the ArduPilot
// formulas (AC_PID::update_all / update_i, calc_lowpass_alpha_dt, DigitalBiquadFilter coefficients, linear
// angle P + input-shaping feed-forward). It does NOT import any AP Tuner code.
//   plant (continuous, 8 kHz sub-steps):  w' = K * m(t) - a * w + d(t),  m = first-order lag (tau) of u(t - Td)
//   gyro: sampled at 1 kHz, + white noise, 2-pole Butterworth (DigitalBiquadFilter) at INS_GYRO_FILTER
//   rate PID at 400 Hz reads the latest filtered gyro sample; output held (ZOH) until the next loop
//   units: rad/s, rad/s^2, mixer units (as ArduPilot Copter); logs converted to deg/s by the caller
'use strict';

function makeRng(seed) {
  let s = seed >>> 0;
  const u = () => { s += 0x6D2B79F5; let t = s; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  let spare = null;
  const n = () => { if (spare !== null) { const v = spare; spare = null; return v; } let a, b, r; do { a = 2 * u() - 1; b = 2 * u() - 1; r = a * a + b * b; } while (r >= 1 || r === 0); const m = Math.sqrt(-2 * Math.log(r) / r); spare = b * m; return a * m; };
  return { u, n };
}

function biquadCoeffs(fs, fc) {
  if (!(fc > 0)) return null;
  const fr = fs / fc, ohm = Math.tan(Math.PI / fr), c = 1 + 2 * Math.cos(Math.PI / 4) * ohm + ohm * ohm;
  const b0 = ohm * ohm / c; return { b0, b1: 2 * b0, b2: b0, a1: 2 * (ohm * ohm - 1) / c, a2: (1 - 2 * Math.cos(Math.PI / 4) * ohm + ohm * ohm) / c };
}

/**
 * opts: { plant:{K,a,tau,Td}, gains:{P,I,D,FF,FLTT,FLTE,FLTD,IMAX}, fsLoop=400, backendHz=1000, subHz=8000, gyroHz,
 *         dur (s), mode 'acro'|'angle'|'none', angleP, inputTc, pilot {seed, rateAmp:[lo,hi] deg/s, angAmp:[lo,hi] deg, hold:[lo,hi], gap:[lo,hi]},
 *         gust {sigma (rad/s^2), tauD (s)}, gyroNoise (rad/s RMS at backend rate), uLimit (|u| clamp, default Infinity), kick (rad/s initial rate), seed }
 * returns per-loop arrays (rad/s units) and pilot/plant traces
 */
function simulate(o) {
  const fsLoop = o.fsLoop || 400, backend = o.backendHz || 1000, sub = o.subHz || 8000;
  const h = 1 / sub, nPerGyro = Math.round(sub / backend), nPerLoop = Math.round(sub / fsLoop), dt = 1 / fsLoop;
  const p = o.plant, g = o.gains, rng = makeRng(o.seed || 1), prng = makeRng((o.pilot && o.pilot.seed) || 99);
  const N = Math.round((o.dur || 60) * sub), nLoop = Math.floor(N / nPerLoop);
  const bq = biquadCoeffs(backend, o.gyroHz);
  let gx1 = 0, gx2 = 0, gy1 = 0, gy2 = 0, gyroF = 0;   // direct form I state
  const alpha = (fc) => (fc > 0 ? dt / (dt + 1 / (2 * Math.PI * fc)) : 1);
  const aT = alpha(g.FLTT), aE = alpha(g.FLTE), aD = alpha(g.FLTD);
  let tgtF = 0, errF = 0, der = 0, integ = 0, first = true;
  const nd = Math.max(0, Math.round(p.Td / h)); const dq = new Float64Array(nd + 1); let di = 0;
  const lagA = p.tau > 0 ? 1 - Math.exp(-h / p.tau) : 1, eA = Math.exp(-p.a * h), gA = p.a > 1e-12 ? (1 - eA) / p.a : h;
  let w = o.kick || 0, th = 0, m = 0, d = 0, u = 0;
  const gust = o.gust || null; const eD = gust ? Math.exp(-h / gust.tauD) : 0, sD = gust ? gust.sigma * Math.sqrt(1 - eD * eD) : 0;
  const uLim = o.uLimit ?? Infinity;
  // pilot
  const pil = o.pilot || {}; const mode = o.mode || 'acro';
  let cmd = 0, cmdLeft = 0, inHold = false; let thT = 0, rateShaped = 0;
  const nextCmd = () => {
    if (inHold) { inHold = false; cmd = 0; cmdLeft = Math.round((pil.gap[0] + (pil.gap[1] - pil.gap[0]) * prng.u()) * fsLoop); }
    else { inHold = true; const amp = mode === 'angle' ? pil.angAmp : pil.rateAmp; cmd = (prng.u() < 0.5 ? -1 : 1) * (amp[0] + (amp[1] - amp[0]) * prng.u()) * Math.PI / 180; cmdLeft = Math.round((pil.hold[0] + (pil.hold[1] - pil.hold[0]) * prng.u()) * fsLoop); }
  };
  const out = { fs: fsLoop, target: new Float64Array(nLoop), actual: new Float64Array(nLoop), P: new Float64Array(nLoop), I: new Float64Array(nLoop), D: new Float64Array(nLoop), FF: new Float64Array(nLoop), wTrue: new Float64Array(nLoop), theta: new Float64Array(nLoop), dist: new Float64Array(nLoop) };
  let k = 0;
  for (let s = 0; s < N; s++) {
    if (s % nPerGyro === 0) {
      const x = w + (o.gyroNoise ? o.gyroNoise * rng.n() : 0);
      if (bq) { const y = bq.b0 * x + bq.b1 * gx1 + bq.b2 * gx2 - bq.a1 * gy1 - bq.a2 * gy2; gx2 = gx1; gx1 = x; gy2 = gy1; gy1 = y; gyroF = y; } else gyroF = x;
    }
    if (s % nPerLoop === 0 && k < nLoop) {
      // pilot / outer loop -> rate target (rad/s)
      let rt = 0;
      if (mode !== 'none' && o.pilot) { if (cmdLeft <= 0) nextCmd(); cmdLeft--; }
      if (mode === 'acro') { if (o.inputTc > 0) { rateShaped += (cmd - rateShaped) * (dt / o.inputTc); rt = rateShaped; } else rt = o.pilot ? cmd : 0; }
      else if (mode === 'angle') { const ff = (cmd - thT) / o.inputTc; thT += ff * dt; rt = ff + o.angleP * (thT - th); }
      // AC_PID::update_all (measurement = latest filtered gyro), then update_i, out = P + I + D (+ FF from get_ff)
      if (first) { tgtF = rt; errF = tgtF - gyroF; der = 0; first = false; }
      else { const eLast = errF; tgtF += aT * (rt - tgtF); errF += aE * ((tgtF - gyroF) - errF); der += aD * ((errF - eLast) / dt - der); }
      const limited = Math.abs(u) >= uLim;   // ArduPilot freezes I growth when the mixer reports a limit
      if (g.I > 0) { if (!limited || (integ > 0 && errF < 0) || (integ < 0 && errF > 0)) { integ += errF * g.I * dt; if (g.IMAX > 0) integ = Math.max(-g.IMAX, Math.min(g.IMAX, integ)); } } else integ = 0;
      const Pout = g.P * errF, Dout = g.D * der, FFout = (g.FF || 0) * tgtF;
      u = Pout + Dout + integ + FFout; if (u > uLim) u = uLim; else if (u < -uLim) u = -uLim;
      out.target[k] = tgtF; out.actual[k] = gyroF; out.P[k] = Pout; out.I[k] = integ; out.D[k] = Dout; out.FF[k] = FFout; out.wTrue[k] = w; out.theta[k] = th; out.dist[k] = d; k++;
    }
    dq[di] = u; di = (di + 1) % (nd + 1); const ud = nd ? dq[di] : u;
    m += (ud - m) * lagA;
    if (gust) d = d * eD + sD * rng.n();
    w = w * eA + (p.K * m + d) * gA;
    th += w * h;
  }
  return out;
}

/** growth test: RMS of the first difference of the true rate early vs late (linear loop, no noise) */
function growth(plant, gains, gyroHz, { fsLoop = 400, dur = 8 } = {}) {
  const r = simulate({ plant, gains, gyroHz, fsLoop, dur, mode: 'none', kick: 0.05, seed: 3 });
  const w = r.wTrue, fs = r.fs; const rms = (a, b) => { let s = 0, c = 0; for (let i = Math.max(1, Math.round(a * fs)); i < Math.round(b * fs); i++) { const dd = w[i] - w[i - 1]; s += dd * dd; c++; } return Math.sqrt(s / c); };
  return rms(dur - 1.5, dur - 0.5) / (rms(1.0, 2.0) + 1e-300);
}
/** P at which the simulated loop starts to grow (bisection, I = iRatio * P) */
function measuredPosc(plant, gainsBase, gyroHz, { iRatio = 1, lo = 0.005, hi = 5, fsLoop = 400 } = {}) {
  const unstable = P => growth(plant, Object.assign({}, gainsBase, { P, I: iRatio * P }), gyroHz, { fsLoop }) > 1;
  if (unstable(lo)) return { P: lo, note: 'unstable at lower bound' };
  if (!unstable(hi)) return { P: hi, note: 'stable at upper bound' };
  let a = Math.log(lo), b = Math.log(hi);
  for (let i = 0; i < 22; i++) { const mm = 0.5 * (a + b); if (unstable(Math.exp(mm))) b = mm; else a = mm; }
  return { P: Math.exp(0.5 * (a + b)) };
}
/** D at which the simulated loop starts to grow for fixed P */
function measuredDosc(plant, gainsBase, gyroHz, { lo = 1e-6, hi = 0.5, fsLoop = 400 } = {}) {
  const unstable = D => growth(plant, Object.assign({}, gainsBase, { D }), gyroHz, { fsLoop }) > 1;
  if (unstable(lo)) return { D: lo, note: 'unstable at lower bound' };
  if (!unstable(hi)) return { D: hi, note: 'stable at upper bound' };
  let a = Math.log(lo), b = Math.log(hi);
  for (let i = 0; i < 22; i++) { const mm = 0.5 * (a + b); if (unstable(Math.exp(mm))) b = mm; else a = mm; }
  return { D: Math.exp(0.5 * (a + b)) };
}

module.exports = { simulate, growth, measuredPosc, measuredDosc, makeRng };
