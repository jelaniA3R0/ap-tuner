// Writes golden results of the JavaScript reference implementation for the C++ port's tests:
//   node tools/make-golden.js            -> cpp/tests/golden/*.json
// Inputs are deterministic (seeded generator), so the files only change when the JavaScript code changes.
'use strict';
const fs = require('fs');
const path = require('path');
const { FFT, hann, nextPow2 } = require('../src/dsp/fft');
const { freqResponse, psd, bandRms } = require('../src/dsp/spectra');
const { stepResponse, stepMetrics } = require('../src/dsp/stepresp');
const { C, lpf1, biquadLpf, notch, harmonicNotch, notchBaseFreq, gyroChain, acPid } = require('../src/model/controller');
const { evalPlant, nelderMead, fitPlant } = require('../src/model/plant');
const synth = require('../src/model/synth');
const rules = require('../src/model/rules');
const { identifyTimeDomain, lpf2, lag1 } = require('../src/model/timeid');

const OUT = path.join(__dirname, '..', 'cpp', 'tests', 'golden');
fs.mkdirSync(OUT, { recursive: true });
const arr = a => Array.from(a);
const cplx = z => ({ re: arr(z.re), im: arr(z.im) });
const write = (name, obj) => { fs.writeFileSync(path.join(OUT, name + '.json'), JSON.stringify(obj)); console.log('wrote', name + '.json', (fs.statSync(path.join(OUT, name + '.json')).size / 1024).toFixed(0) + ' KB'); };

// deterministic noise (same LCG as synth.simulateRate)
function rng(seed) { let s = seed >>> 0; return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; }; }
function gaussGen(seed) { const r = rng(seed); return () => { const u1 = r() || 1e-9, u2 = r(); return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2); }; }

// ---------------------------------------------------------------- fft
{
  const n = 64, g = gaussGen(7);
  const re = new Float64Array(n), im = new Float64Array(n);
  for (let i = 0; i < n; i++) { re[i] = g(); im[i] = g(); }
  const inRe = arr(re), inIm = arr(im);
  const f = new FFT(n); f.transform(re, im);
  const fwd = { re: arr(re), im: arr(im) };
  f.transform(re, im, true);
  write('fft', { n, inRe, inIm, fwd, back: { re: arr(re), im: arr(im) }, hann16: arr(hann(16)), nextPow2: [1, 3, 64, 64.5, 1000, 512.0001].map(v => [v, nextPow2(v)]) });
}

// ---------------------------------------------------------------- spectra + step response
{
  const fs = 400, n = 4096, g = gaussGen(11);
  const x = new Float64Array(n), y = new Float64Array(n);
  // x: low-passed noise plus a 3 Hz tone; y: x through a 2-pole system plus measurement noise
  let lp = 0; for (let i = 0; i < n; i++) { lp += 0.2 * (g() * 10 - lp); x[i] = lp + 5 * Math.sin(2 * Math.PI * 3 * i / fs); }
  let s1 = 0, s2 = 0; for (let i = 0; i < n; i++) { s1 += 0.15 * (x[i] - s1); s2 += 0.3 * (s1 - s2); y[i] = s2 + 0.3 * g(); }
  const fr = freqResponse(x, y, fs, 512, 0.75, 0);
  const frSkip = freqResponse(x, y, fs, 256, 0.5, 6);
  const sp = psd(x, fs, 256, 0.5);
  write('spectra', { fs, x: arr(x), y: arr(y),
    fr: { f: arr(fr.f), H: cplx(fr.H), coh: arr(fr.coh), Sxx: arr(fr.Sxx), Syy: arr(fr.Syy), nWin: fr.nWin },
    frSkip: { nWin: frSkip.nWin, coh: arr(frSkip.coh) },
    psd: { f: arr(sp.f), psd: arr(sp.psd), amp: arr(sp.amp), nWin: sp.nWin }, bandRms: bandRms(sp, 5, 40) });

  // closed-loop data for the step response: square-wave target through a simulated rate loop
  const ctx = { fsLoop: fs, gyro: { backendHz: 1000, gyroFilterHz: 40, notches: [] }, plant: { K: 2000 / 57.2957795, a: 1, tau: 0.05, Td: 0.012 } };
  const gains = { P: 0.1, I: 0.1, D: 0.003, FF: 0, DFF: 0, FLTT: 20, FLTE: 0, FLTD: 20 };
  const N = 3000, ref = new Float64Array(N);
  for (let i = 0; i < N; i++) ref[i] = (Math.floor(i / 150) % 2 ? 60 : -40) + (Math.floor(i / 400) % 3) * 10;
  const sim = synth.simulateRate(ctx, gains, ref, { noiseRms: 0.5, seed: 3 });
  const sr = stepResponse(ref, sim.y, fs, { windowSize: 512, durationS: 0.5, minInput: 15 });
  const met = stepMetrics(sr.t, sr.mean);
  const tm = Array.from({ length: 50 }, (_, i) => i * 0.01);
  const ym = tm.map(t => 1 - Math.exp(-t / 0.05) * Math.cos(t * 40));
  write('stepresp', { fs, target: arr(ref), actual: arr(sim.y), mean: sr.mean, count: sr.count, t: sr.t, metrics: met, handT: tm, handY: ym, handMetrics: stepMetrics(tm, ym) });
}

// ---------------------------------------------------------------- controller
{
  const freqs = arr(synth.logspace(0.1, 200, 60));
  const notchSets = [
    { enable: 1, freq: 80, bw: 40, att: 40, hmncs: 3, opts: 0 },
    { enable: 1, freq: 55, bw: 25, att: 30, hmncs: 7, opts: 1 },
    { enable: 1, freq: 120, bw: 60, att: 40, hmncs: 1, opts: 16, chained: 2 },
    { enable: 1, freq: 300, bw: 100, att: 40, hmncs: 3, opts: 0 },
  ];
  const gyro = { backendHz: 1000, gyroFilterHz: 20, notches: [notchSets[0], notchSets[1]] };
  const g = { P: 0.135, I: 0.135, D: 0.0036, FF: 0.02, DFF: 0.001, FLTT: 20, FLTE: 2.5, FLTD: 20 };
  const pid = acPid(freqs, 400, g);
  const nb = [
    [{ mode: 0, freq: 80, ref: 0, fm_rat: 1 }, {}],
    [{ mode: 1, freq: 80, ref: 0.3, fm_rat: 1 }, { throttle: 0.45 }],
    [{ mode: 1, freq: 80, ref: 0.3, fm_rat: 1.2 }, { throttle: 0.2 }],
    [{ mode: 1, freq: 80, ref: 0, fm_rat: 1 }, {}],
    [{ mode: 3, freq: 40, ref: 2, fm_rat: 1 }, { escRpm: 3000 }],
    [{ mode: 2, freq: 40, ref: 0, fm_rat: 1 }, { rpm: 1200 }],
    [{ mode: 4, freq: 50, ref: 1, fm_rat: 1 }, { fftHz: 0 }],
    [{ mode: 4, freq: 50, ref: 1, fm_rat: 1 }, { fftHz: 77 }],
  ].map(([p, op]) => ({ p, op, f: notchBaseFreq(p, op) }));
  write('controller', { freqs,
    lpf1: cplx(lpf1(freqs, 400, 20)), lpf1Off: cplx(lpf1(freqs, 400, 0)), biquadLpf: cplx(biquadLpf(freqs, 1000, 20)),
    notch: cplx(notch(freqs, 1000, 80, 40, 40)),
    harmonic: notchSets.map(n => ({ n, H: cplx(harmonicNotch(freqs, 1000, n)) })),
    gyro, gyroChain: cplx(gyroChain(freqs, gyro)), gains: g,
    pid: { T: cplx(pid.T), Cfb: cplx(pid.Cfb), Cff: cplx(pid.Cff), P: cplx(pid.parts.P), I: cplx(pid.parts.I), D: cplx(pid.parts.D) },
    notchBase: nb });
}

// ---------------------------------------------------------------- plant
{
  const freqs = arr(synth.logspace(0.3, 40, 80));
  const truth = { K: 2000, a: 1, tau: 0.05, Td: 0.012 };
  const G = evalPlant(freqs, truth);
  const g = gaussGen(5);
  const H = { re: G.re.map(v => v * (1 + 0.03 * g())), im: G.im.map(v => v * (1 + 0.03 * g())) };
  const coh = freqs.map((f, i) => Math.min(0.99, 0.4 + 0.6 * Math.exp(-f / 30) + 0.05 * Math.sin(i)));
  const fit = fitPlant(freqs, H, coh, { kind: 'copter', nWin: 60, fMin: 0.3, fMax: 30 });
  const ros = nelderMead(x => 100 * (x[1] - x[0] * x[0]) ** 2 + (1 - x[0]) ** 2, [-1.2, 1], { step: 0.3, maxIter: 2000, tol: 1e-12 });
  write('plant', { freqs, truth, G: cplx(G), H: { re: arr(H.re), im: arr(H.im) }, coh, fit, rosenbrock: ros });
}

// ---------------------------------------------------------------- synth
{
  const us = 57.2957795;
  const ctx = { fsLoop: 400, gyro: { backendHz: 1000, gyroFilterHz: 40, notches: [] }, plant: { K: 2000 / us, a: 1, tau: 0.05, Td: 0.012 } };
  const ctxNotch = Object.assign({}, ctx, { gyro: { backendHz: 2000, gyroFilterHz: 60, notches: [{ enable: 1, freq: 90, bw: 45, att: 40, hmncs: 3, opts: 0 }] } });
  const g = { P: 0.135, I: 0.135, D: 0.0036, FF: 0, DFF: 0, FLTT: 20, FLTE: 0, FLTD: 20 };
  const freqs = arr(synth.logspace(0.2, 200, 300));
  const ol = synth.openLoop(freqs, ctx, g);
  const cl = synth.closedLoop(freqs, ctx, g);
  const m = synth.margins(freqs, ol.L);
  const step = synth.simulateRateStep(ctx, g, { stepDegs: 30 / us, durationS: 1.0, uMax: 1 });
  const refs = Float64Array.from({ length: 800 }, (_, i) => (i % 200 < 100 ? 0.5 : -0.3));
  const dist = Float64Array.from({ length: 800 }, (_, i) => 0.2 * Math.sin(i / 30));
  const noisy = synth.simulateRate(ctxNotch, g, refs, { noiseRms: 0.02, seed: 9, dist, uMax: 0.8 });
  const noisePsd = { f: Array.from({ length: 129 }, (_, i) => i * 400 / 256), psd: Array.from({ length: 129 }, (_, i) => 1e-6 * (1 + i / 20)) };
  const base = rules.PRESETS.normal;
  const opts = Object.assign({}, base, { pRange: [0.01, 2.0], dRange: [0.0005, 0.2], iRatio: 1, filters: { FLTT: 20, FLTE: 0, FLTD: 20 }, uMax: 1, stepMag: 30 / us, currentP: 0.135, currentD: 0.0036 });
  const optsNoise = Object.assign({}, opts, { dNoiseCap: 0.04, noisePsd, noisePsdIsFiltered: true });
  const optsCons = Object.assign({}, rules.PRESETS.conservative, opts, { gmMinDb: 6, pmMinDeg: 30, msMax: 3.0, osMaxPct: 12, backoff: 0.3, currentP: 0, currentD: 0 });
  const yawOpts = Object.assign({}, opts, { pRange: [0.02, 20], dRange: [0, 0.01], iRatio: 0.1, filters: { FLTT: 20, FLTE: 2.5, FLTD: 20 }, stepDurS: 6, uMax: 0.48, currentP: 0.18, currentD: 0 });
  const ctxYaw = Object.assign({}, ctx, { plant: { K: 11.1 / us, a: 0.05, tau: 0.05, Td: 0.08 } });
  const pick = r => r && { gains: r.gains, margins: r.margins, bandwidthHz: r.bandwidthHz, sBwHz: r.sBwHz, overshootPct: r.overshootPct, dNoise: r.dNoise, osc: r.oscillationPoints || null, nCand: r.candidates.length };
  write('synth', { ctx, ctxNotch, ctxYaw, g, freqs, logspace: arr(synth.logspace(0.1, 200, 7)),
    L: cplx(ol.L), Tcl: cplx(cl.Tcl), S: cplx(cl.S), margins: m,
    bw3db: synth.bandwidth3db(freqs, cl.Tcl), sBw: synth.sensitivityBandwidth(freqs, cl.S), sEnergy: synth.sensitivityEnergy(freqs, cl.S, 10),
    step: { y: step.y, u: step.u }, refs: arr(refs), dist: arr(dist), noisy: { y: noisy.y, u: noisy.u, ytrue: noisy.ytrue },
    opts, optsNoise, optsCons, yawOpts, noisePsd,
    quik: pick(synth.synthesizeQuik(ctx, opts)), quikNoise: pick(synth.synthesizeQuik(ctxNotch, optsNoise)), quikCons: pick(synth.synthesizeQuik(ctx, optsCons)),
    quikYaw: pick(synth.synthesizeQuik(ctxYaw, yawOpts)),
    shaped: pick(synth.synthesize(ctx, opts)), shapedNoise: pick(synth.synthesize(ctxNotch, optsNoise)) });
}

// ---------------------------------------------------------------- rules
{
  const fams = ['copter', 'vtol', 'plane'], axes = ['roll', 'pitch', 'yaw'];
  const curs = [{ FLTT: 20, FLTE: 0, FLTD: 20 }, { FLTT: 0, FLTE: 2.5, FLTD: 50 }, { FLTT: 15, FLTE: 12, FLTD: 0 }, { FLTT: 30, FLTE: 7, FLTD: 10 }];
  const filters = [];
  for (const f of fams) for (const a of axes) for (const c of curs) for (const gh of [20, 25, 41]) for (const tau of [0, 0.3, 0.07]) filters.push({ f, a, c, gh, tau, r: rules.filterRules(f, a, c, gh, tau) });
  const irule = [];
  for (const f of fams) for (const a of axes) for (const [P, FF, hasFF] of [[0.1, 0, false], [0.3, 0.2, true], [0.5, 0, true], [1.2, 0.9, false]]) irule.push({ f, a, P, FF, hasFF, r: rules.iRule(f, a, P, FF, hasFF) });
  const angle = [];
  for (const f of fams) for (const a of axes) for (const wc of [0.3, 4.4, 20, 61.7, 200, 1e-5]) for (const yawMin of [2, 0.5]) angle.push({ f, a, wc, yawMin, r: rules.angleRule(f, a, wc, { yawMin }) });
  const limits = []; for (const f of fams) for (const a of axes) limits.push({ f, a, r: rules.gainLimits(f, a) });
  write('rules', { filters, irule, angle, limits, needed: Object.fromEntries(fams.map(f => [f, rules.neededParams(f)])), osc: [0, 5, 15, 49, 50, 99, 100, 500].map(m => [m, rules.oscSmaxFor(m)]), presets: rules.PRESETS });
}

// ---------------------------------------------------------------- timeid
{
  const cases = [];
  for (const [axis, seed, b, a, trimBase, L, n, gap] of [['yaw', 21, 11.5, 0.165, 0.38, 0.6, 9000, true], ['roll', 22, 380, 0.4, 0.04, 0, 9000, false], ['pitch', 23, 300, 0.5, -0.1, 0, 2000, false], ['yaw', 24, 11.5, 0.165, 0.3, 0, 9000, false]]) {
    // simulated airframe r_dot = b (sat(u_lag) - trim(t)) - a r + gust, under a P+I controller with a wandering target
    const fs = 100, g = gaussGen(seed), dt = 1 / fs;
    const actual = new Float64Array(n), u = new Float64Array(n), airborne = new Uint8Array(n);
    let r = 0, ul = 0, integ = 0, gust = 0, tgt = 0;
    const P = axis === 'yaw' ? 1.5 : 0.25 / 57.3 * 57.3, I = axis === 'yaw' ? 0.11 : 0.25;
    for (let i = 0; i < n; i++) {
      if (i % 300 === 0) tgt = (g() * (axis === 'yaw' ? 10 : 20));
      gust += 0.05 * (g() * (axis === 'yaw' ? 4 : 120) - gust);
      const e = (tgt - r) / 57.2957795;
      integ += I * e * dt;
      let out = P * e + integ; out = Math.max(-1, Math.min(1, out));
      u[i] = out;
      ul += (1 - Math.exp(-dt / 0.05)) * (out - ul);
      const us = L ? Math.max(-L, Math.min(L, ul)) : ul;
      const trim = trimBase + 0.05 * Math.sin(i / 2000);
      r += dt * (b * (us - trim) - a * r + gust);
      actual[i] = r + 0.3 * g();
      airborne[i] = gap && i > n * 0.45 && i < n * 0.55 ? 0 : 1;
    }
    const data = { fs, actual, u, airborne };
    const res = identifyTimeDomain(data, { axis });
    cases.push({ axis, fs, actual: arr(actual), u: arr(u), airborne: arr(airborne), ok: res.ok, reason: res.reason || null, fit: res.fit || null });
  }
  const sig = Float64Array.from({ length: 500 }, (_, i) => Math.sin(i / 7) + 0.3 * Math.cos(i / 2));
  write('timeid', { cases, lpf2: arr(lpf2(sig, 100, 2)), lag1: arr(lag1(sig, 100, 0.05)), sig: arr(sig) });
}

// ---------------------------------------------------------------- JavaScript number formatting
{
  const nums = [0, -0, 1, -1, 0.1, 0.1 + 0.2, 1 / 3, 2 / 3, 123.456, 1e21, 1e20, 123456789012345680000, 1e-6, 1e-7, 1.5e-7, 5e-324, 1.7976931348623157e308, 0.000001234, 12.5, 0.125, 0.375, 2.5, -2.5, 1.005, 1.45, 8.345, 99.995, 0.0036, 0.135, 57.2957795, 1234.5678, -0.0005, 4.35, 1e-10, 3.14159265358979];
  const toFixed = [], toPrecision = [];
  for (const x of nums) for (const d of [0, 1, 2, 3, 6]) toFixed.push([x, d, x.toFixed(d)]);
  for (const x of nums) if (x !== 0) for (const p of [1, 2, 3, 4, 6, 10]) toPrecision.push([x, p, x.toPrecision(p)]);
  toPrecision.push([0, 3, (0).toPrecision(3)]);
  write('jsnum', { toString: nums.map(x => [x, String(x)]), toFixed, toPrecision, json: JSON.stringify({ a: [1, 0.5, -3e-9, 1e21, 'x"y\\z\n\u0001', true, null], b: { c: 'é€' } }) });
}
