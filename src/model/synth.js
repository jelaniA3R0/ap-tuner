// Gain synthesis by loop shaping against the identified plant, using ArduPilot's exact controller
// and filter structure. Also closed-loop prediction (frequency domain + time-domain simulation).
'use strict';
const { C, acPid, gyroChain } = require('./controller');
const { evalPlant } = require('./plant');

function logspace(f1, f2, n) { const o = new Float64Array(n); for (let i = 0; i < n; i++) o[i] = f1 * Math.pow(f2 / f1, i / (n - 1)); return o; }

/** Stability margins of an open loop L(jw) on grid freqs. Returns {gmDb, pmDeg, wcHz, w180Hz, ms} */
function margins(freqs, L) {
  const mag = C.abs(L), ph = C.phaseDeg(L);
  // unwrap phase
  const up = new Float64Array(ph.length); up[0] = ph[0];
  for (let i = 1; i < ph.length; i++) { let d = ph[i] - ph[i - 1]; while (d > 180) d -= 360; while (d < -180) d += 360; up[i] = up[i - 1] + d; }
  let wc = NaN, pm = NaN, w180 = NaN, gm = NaN;
  for (let i = 1; i < mag.length; i++) {
    if (isNaN(wc) && mag[i - 1] >= 1 && mag[i] < 1) { // gain crossover (last one matters for PM: take the first from low freq where |L| drops below 1)
      const t = (mag[i - 1] - 1) / (mag[i - 1] - mag[i]);
      wc = freqs[i - 1] + t * (freqs[i] - freqs[i - 1]);
      const p = up[i - 1] + t * (up[i] - up[i - 1]);
      pm = 180 + p; while (pm > 360) pm -= 360;
    }
    if (isNaN(w180) && up[i - 1] > -180 && up[i] <= -180) {
      const t = (up[i - 1] + 180) / (up[i - 1] - up[i]);
      w180 = freqs[i - 1] + t * (freqs[i] - freqs[i - 1]);
      const m = mag[i - 1] + t * (mag[i] - mag[i - 1]);
      gm = -20 * Math.log10(m);
    }
  }
  // also catch a loop that is always below 1 (very low gain): pm = 180
  if (isNaN(wc)) { pm = 180; wc = 0; }
  // sensitivity peak Ms = max |1/(1+L)|
  let ms = 0;
  for (let i = 0; i < mag.length; i++) { const r = 1 + L.re[i], im = L.im[i]; const s = 1 / (Math.hypot(r, im) || 1e-9); if (s > ms) ms = s; }
  return { gmDb: gm, pmDeg: pm, wcHz: wc, w180Hz: w180, ms };
}

/**
 * Build the open loop L = G_bare * F_gyro * Cfb for gains g.  ctx = {fsLoop, gyro:{backendHz,gyroFilterHz,notches}, plant}
 */
function openLoop(freqs, ctx, g) {
  const G = evalPlant(freqs, ctx.plant);
  const F = gyroChain(freqs, ctx.gyro);
  const pid = acPid(freqs, ctx.fsLoop, g);
  // one loop of computation/output latency beyond what the identified Td carries is NOT added here:
  // Td was identified from loop-synchronous data so it already includes it.
  const L = C.mul(C.mul(G, F), pid.Cfb);
  return { L, G, F, pid };
}

/** Closed-loop rate response y/r and sensitivity for gains g */
function closedLoop(freqs, ctx, g) {
  const { L, G, F, pid } = openLoop(freqs, ctx, g);
  const onePlusL = C.addReal(L, 1);
  // y = G (Cfb (T r - F y) + Cff T r)  ->  y/r = G T (Cfb + Cff) / (1 + G F Cfb)
  const num = C.mul(C.mul(G, pid.T), C.add(pid.Cfb, pid.Cff));
  const Tcl = C.div(num, onePlusL);
  const S = C.div(C.one(freqs.length), onePlusL);
  return { L, Tcl, S, G, F, pid };
}

/** Control-band sensitivity energy: integral of |S|^2 over 0..fMax Hz (lower = better disturbance rejection) */
function sensitivityEnergy(freqs, S, fMax) {
  let e = 0;
  for (let i = 1; i < freqs.length; i++) { if (freqs[i] > fMax) break; const m = Math.hypot(S.re[i], S.im[i]); e += m * m * (freqs[i] - freqs[i - 1]); }
  return e;
}

/** Disturbance-rejection bandwidth: highest frequency below which |S| stays <= -3 dB */
function sensitivityBandwidth(freqs, S) {
  const mag = C.abs(S);
  for (let i = 0; i < mag.length; i++) if (mag[i] > Math.SQRT1_2) return freqs[i];
  return freqs[freqs.length - 1];
}

/** -3 dB bandwidth (Hz) of a closed-loop response (relative to its low-frequency gain) */
function bandwidth3db(freqs, T) {
  const mag = C.abs(T); const ref = mag[0] || 1;
  for (let i = 0; i < mag.length; i++) if (mag[i] < ref * Math.SQRT1_2) return freqs[i];
  return freqs[freqs.length - 1];
}

/**
 * Time-domain simulation of the rate loop with ArduPilot's AC_PID (as coded in AC_PID::update_all)
 * driving the continuous plant (integrated with sub-steps), returning the step response to a rate
 * demand. Includes the gyro LPF (2-pole) and transport delay. Used for prediction and validation.
 */
function simulateRateStep(ctx, g, { stepDegs = 30, durationS = 1.0, uMax = Infinity } = {}) {
  const n = Math.round(durationS * ctx.fsLoop);
  const ref = new Float64Array(n).fill(stepDegs);
  return simulateRate(ctx, g, ref, { uMax });
}

/** General simulation with an arbitrary reference array (deg/s) at loop rate; optional gyro noise (deg/s RMS, white at backend rate) and output disturbance array */
function simulateRate(ctx, g, ref, { uMax = Infinity, noiseRms = 0, dist = null, seed = 1 } = {}) {
  const fs = ctx.fsLoop, dt = 1 / fs;
  let rnd = seed >>> 0; const rand = () => { rnd = (rnd * 1664525 + 1013904223) >>> 0; return rnd / 4294967296; };
  const gauss = () => { const u1 = rand() || 1e-9, u2 = rand(); return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2); };
  const sub = Math.max(1, Math.ceil(ctx.gyro.backendHz / fs)); const dts = dt / sub;
  const p = ctx.plant;
  // plant state: x1 = rate (deg/s) with pole a ; x2 = motor state (lag tau)
  let rate = 0, motor = 0;
  const delayN = Math.max(0, Math.round(p.Td / dts)); const dq = new Float64Array(delayN + 1);
  // gyro 2-pole LPF at backend rate (same coefficients as DigitalBiquad)
  const fc = ctx.gyro.gyroFilterHz, bfs = ctx.gyro.backendHz;
  let b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  if (fc > 0 && fc < bfs / 2) {
    const fr = bfs / fc, ohm = Math.tan(Math.PI / fr), c = 1 + 2 * Math.cos(Math.PI / 4) * ohm + ohm * ohm;
    b0 = ohm * ohm / c; b1 = 2 * b0; b2 = b0; a1 = 2 * (ohm * ohm - 1) / c; a2 = (1 - 2 * Math.cos(Math.PI / 4) * ohm + ohm * ohm) / c;
  }
  let d1 = 0, d2 = 0; let gyro = 0;
  const aLag = p.tau > 0 ? 1 - Math.exp(-dts / p.tau) : 1;
  const eA = Math.exp(-p.a * dts), gA = p.a > 1e-9 ? (1 - eA) / p.a : dts;
  // PID state
  let tgt = 0, err = 0, der = 0, integ = 0; let first = true;
  const aT = g.FLTT > 0 ? dt / (dt + 1 / (2 * Math.PI * g.FLTT)) : 1;
  const aE = g.FLTE > 0 ? dt / (dt + 1 / (2 * Math.PI * g.FLTE)) : 1;
  const aD = g.FLTD > 0 ? dt / (dt + 1 / (2 * Math.PI * g.FLTD)) : 1;
  const n = ref.length;
  const t = new Float64Array(n), y = new Float64Array(n), u = new Float64Array(n), ytrue = new Float64Array(n);
  let di = 0;
  for (let k = 0; k < n; k++) {
    const target = ref[k];
    y[k] = gyro; ytrue[k] = rate;
    // AC_PID::update_all
    if (first) { tgt = target; err = tgt - gyro; der = 0; first = false; }
    else {
      tgt += aT * (target - tgt);
      const e = tgt - gyro; const eLast = err;
      err += aE * (e - err);
      der += aD * ((err - eLast) / dt - der);
    }
    integ += err * g.I * dt;
    let out = g.P * err + g.D * der + integ + g.FF * tgt;
    if (out > uMax) out = uMax; else if (out < -uMax) out = -uMax;
    u[k] = out; t[k] = k * dt;
    // plant + sensor at backend rate
    for (let s = 0; s < sub; s++) {
      dq[di] = out; di = (di + 1) % (delayN + 1); const ud = dq[di]; // delayed input
      motor += (ud - motor) * aLag;                                   // first-order lag (tau), exact step
      const acc = p.K * motor + (dist ? dist[k] : 0);                 // (s + a), plus torque disturbance
      rate = rate * eA + acc * gA;
      const xin = rate + (noiseRms ? gauss() * noiseRms : 0);
      const yv = b0 * xin + d1; d1 = b1 * xin - a1 * yv + d2; d2 = b2 * xin - a2 * yv; gyro = (fc > 0 && fc < bfs / 2) ? yv : xin;
    }
  }
  return { t: Array.from(t), y: Array.from(y), u: Array.from(u), ytrue: Array.from(ytrue) };
}

/**
 * Loop-shaping synthesis.
 * ctx: { fsLoop, gyro, plant, axis: 'roll'|'pitch'|'yaw', vehicle: 'copter'|'plane'|'vtol'|'heli' }
 * opts: { gmMinDb, pmMinDeg, msMax, dNoiseCap, pRange:[min,max], dRange:[min,max], iRatio, ffEnabled,
 *         filters:{FLTT,FLTE,FLTD}, noisePsd:{f,psd} (gyro noise, deg/s^2 per Hz) }
 * Returns { gains:{P,I,D,FF,...}, margins, bandwidthHz, candidates }
 */
function synthesize(ctx, opts) {
  const freqs = logspace(0.1, Math.min(ctx.fsLoop / 2, 400), 400);
  const F = opts.filters;
  const base = { FLTT: F.FLTT, FLTE: F.FLTE, FLTD: F.FLTD, FF: 0, DFF: 0 };
  const iRatio = opts.iRatio;           // I = P * iRatio (0 -> I handled separately)
  const gm = opts.gmMinDb, pm = opts.pmMinDeg, msMax = opts.msMax;
  const [pMin0, pMax] = opts.pRange, [dMin, dMax] = opts.dRange;
  const pMin = Math.min(pMin0, 1e-4);   // search below the vehicle's nominal minimum; limits are applied by the caller
  const ok = m => isFinite(m.pmDeg) && m.pmDeg >= pm && (isNaN(m.gmDb) || m.gmDb >= gm) && m.ms <= msMax;
  // noise-driven D cap: RMS of D-term output from gyro noise must stay below cap
  const dNoiseRms = (D) => {
    if (!opts.noisePsd) return 0;
    const g = Object.assign({}, base, { P: 0, I: 0, D });
    const pid = acPid(opts.noisePsd.f, ctx.fsLoop, g);
    const H = opts.noisePsdIsFiltered ? C.abs(pid.parts.D) : C.abs(C.mul(pid.parts.D, gyroChain(opts.noisePsd.f, ctx.gyro)));
    let e = 0; const df = opts.noisePsd.f[1] - opts.noisePsd.f[0];
    for (let i = 0; i < H.length; i++) e += H[i] * H[i] * opts.noisePsd.psd[i] * df;
    return Math.sqrt(e);
  };
  // candidate derivative-time ratios D/P (seconds): 0 and a geometric sweep
  const ratios = [0];
  const rMax = opts.rMax ?? 0.08;      // D/P lead time constant cap (s): keeps tunes in the shape ArduPilot vehicles use
  for (let r = 0.004; r <= rMax; r *= 1.3) ratios.push(r);
  const candidates = [];
  for (const r of ratios) {
    // bisection on P (log scale) for the largest P meeting the constraints
    let lo = Math.log(pMin), hi = Math.log(pMax);
    const osMax = opts.osMaxPct ?? 10; const stepMag = opts.stepMag ?? 30;
    const test = (P) => {
      const D = Math.min(dMax, P * r); const g = Object.assign({}, base, { P, I: P * iRatio, D });
      const { L } = openLoop(freqs, ctx, g); const m = margins(freqs, L);
      let os = NaN;
      if (ok(m)) { // overshoot check by time simulation on the identified plant (rate step, actuator limited)
        const st = simulateRateStep(ctx, g, { stepDegs: stepMag, durationS: opts.stepDurS ?? 1.0, uMax: opts.uMax ?? Infinity });
        let peak = 0; for (const v of st.y) if (v > peak) peak = v;
        os = Math.max(0, (peak / stepMag - 1) * 100);
      }
      return { m, g, os, pass: ok(m) && os <= osMax };
    };
    let best = null;
    const t0 = test(Math.exp(lo)); if (!t0.pass) { candidates.push({ r, fail: true }); continue; }
    for (let it = 0; it < 24; it++) {
      const mid = 0.5 * (lo + hi); const tm = test(Math.exp(mid));
      if (tm.pass) { lo = mid; best = tm; } else hi = mid;
    }
    if (!best) best = t0;
    const D = best.g.D;
    const dn = dNoiseRms(D);
    if (opts.dNoiseCap && dn > opts.dNoiseCap) { candidates.push({ r, P: best.g.P, D, noise: dn, fail: 'noise' }); continue; }
    if (D < dMin && r > 0) { candidates.push({ r, P: best.g.P, D, fail: 'dmin' }); continue; }
    const cl = closedLoop(freqs, ctx, best.g);
    const bw = bandwidth3db(freqs, cl.Tcl);
    const sbw = sensitivityBandwidth(freqs, cl.S);
    const sE = sensitivityEnergy(freqs, cl.S, Math.min(10, ctx.fsLoop / 20));
    candidates.push({ r, P: best.g.P, D, I: best.g.I, margins: best.m, bwHz: bw, sBwHz: sbw, sEnergy: sE, noise: dn, overshootPct: best.os });
  }
  const good = candidates.filter(c => !c.fail);
  if (!good.length) return null;
  // choose the candidate with the largest P that meets the margins (P sets the low-frequency stiffness and,
  // with I tied to P, the tracking; this is what QuikTune/AutoTune maximise). D is then just the lead that
  // lets P be largest. Ties go to the lower sensitivity peak.
  good.sort((a, b) => (b.P - a.P) || (a.margins.ms - b.margins.ms));
  const pick = good[0];
  const gains = Object.assign({}, base, { P: pick.P, I: pick.I, D: pick.D });
  return { gains, margins: pick.margins, bandwidthHz: pick.bwHz, sBwHz: pick.sBwHz, overshootPct: pick.overshootPct, dNoise: pick.noise, candidates, freqs };
}

/**
 * QuikTune-style synthesis on the model: raise D (with the current P) until the loop would oscillate,
 * back off by the preset's factor; then raise P (with that D) until it would oscillate, back off the
 * same way; then make sure phase margin, sensitivity peak and overshoot are within the preset, lowering
 * P if needed. This is the procedure ArduPilot's QuikTune applet and the manual tuning guide use, with
 * the oscillation point found on the identified model instead of in the air.
 * opts as synthesize() plus currentP, currentD, backoff (0.4 = QuikTune's 60% reduction).
 */
function synthesizeQuik(ctx, opts) {
  const freqs = logspace(0.1, Math.min(ctx.fsLoop / 2, 400), 400);
  const F = opts.filters;
  const base = { FLTT: F.FLTT, FLTE: F.FLTE, FLTD: F.FLTD, FF: 0, DFF: 0 };
  const iRatio = opts.iRatio, backoff = opts.backoff ?? 0.4;
  const [pMin0, pMax] = opts.pRange, [dMin, dMax] = opts.dRange;
  const pMin = Math.min(pMin0, 1e-4);
  const stepMag = opts.stepMag ?? 30, osMax = opts.osMaxPct ?? 10;
  const gainsOf = (P, D) => Object.assign({}, base, { P, I: P * iRatio, D });
  const stable = (P, D) => { const m = margins(freqs, openLoop(freqs, ctx, gainsOf(P, D)).L); return (isNaN(m.gmDb) || m.gmDb > 0) && (isNaN(m.pmDeg) || m.pmDeg > 0); };
  // largest value of x in [lo, hi] (log scale) for which pred(x) holds; assumes pred true at lo
  const bisect = (lo, hi, pred) => { if (!pred(lo)) return null; if (pred(hi)) return hi; let a = Math.log(lo), b = Math.log(hi); for (let i = 0; i < 26; i++) { const m = 0.5 * (a + b); if (pred(Math.exp(m))) a = m; else b = m; } return Math.exp(a); };
  const dNoise = (D) => { if (!opts.noisePsd) return 0; const g = gainsOf(0, D); const pid = acPid(opts.noisePsd.f, ctx.fsLoop, g); const H = opts.noisePsdIsFiltered ? C.abs(pid.parts.D) : C.abs(C.mul(pid.parts.D, gyroChain(opts.noisePsd.f, ctx.gyro))); let e = 0; const df = opts.noisePsd.f[1] - opts.noisePsd.f[0]; for (let i = 0; i < H.length; i++) e += H[i] * H[i] * opts.noisePsd.psd[i] * df; return Math.sqrt(e); };
  let P0 = Math.min(pMax, Math.max(pMin, opts.currentP || 0.1));
  let D0 = Math.min(dMax, Math.max(0, opts.currentD || 0));
  // as in the manual guide: if the current gains would oscillate, halve P (and D) until they do not
  for (let i = 0; i < 12 && !stable(P0, D0) && P0 > pMin; i++) { P0 = Math.max(pMin, P0 * 0.5); D0 = D0 * 0.5 < 1e-6 ? 0 : D0 * 0.5; }
  // 1) D up to the oscillation point with that P (D may legitimately be 0 on some airframes)
  const dLo = 1e-6;
  const dOsc = stable(P0, dLo) ? bisect(dLo, dMax, D => stable(P0, D)) : null;
  let D = dOsc === null ? 0 : (dOsc >= dMax ? dMax : dOsc * backoff);
  if (opts.dNoiseCap && D > 0) { const n = dNoise(D); if (n > opts.dNoiseCap) { const dCap = bisect(dLo, D, x => dNoise(x) <= opts.dNoiseCap); D = dCap !== null ? dCap : 0; } }
  if (D > 0 && D < dMin) D = stable(P0, dMin) ? dMin : 0;
  D = Math.min(dMax, D);
  // 2) P up to the oscillation point with that D
  const pOsc = bisect(pMin, pMax, P => stable(P, D));
  let P = pOsc === null ? pMin : (pOsc >= pMax ? pMax : pOsc * backoff);
  // 3) phase margin / sensitivity / overshoot limits (lower P until they hold)
  const okAll = (Pv) => { const g = gainsOf(Pv, D); const m = margins(freqs, openLoop(freqs, ctx, g).L); if (!(isFinite(m.pmDeg) && m.pmDeg >= opts.pmMinDeg && (isNaN(m.gmDb) || m.gmDb >= opts.gmMinDb) && m.ms <= opts.msMax)) return false; const st = simulateRateStep(ctx, g, { stepDegs: stepMag, durationS: opts.stepDurS ?? 1.0, uMax: opts.uMax ?? Infinity }); let peak = 0; for (const v of st.y) if (v > peak) peak = v; return (peak / stepMag - 1) * 100 <= osMax; };
  if (!okAll(P)) { const Pc = bisect(pMin, P, okAll); if (Pc !== null) P = Pc; }
  const g = gainsOf(P, D);
  const m = margins(freqs, openLoop(freqs, ctx, g).L);
  const cl = closedLoop(freqs, ctx, g);
  const st = simulateRateStep(ctx, g, { stepDegs: stepMag, durationS: opts.stepDurS ?? 1.0, uMax: opts.uMax ?? Infinity }); let peak = 0; for (const v of st.y) if (v > peak) peak = v;
  return { gains: g, margins: m, bandwidthHz: bandwidth3db(freqs, cl.Tcl), sBwHz: sensitivityBandwidth(freqs, cl.S), overshootPct: Math.max(0, (peak / stepMag - 1) * 100), dNoise: dNoise(D), oscillationPoints: { D: dOsc, P: pOsc, backoff, startP: P0 }, candidates: [], freqs };
}

module.exports = { logspace, margins, openLoop, closedLoop, bandwidth3db, sensitivityBandwidth, sensitivityEnergy, simulateRateStep, simulateRate, synthesize, synthesizeQuik };
