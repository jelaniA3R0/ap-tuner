// End-to-end check of the maths on a virtual multirotor:
//  simulate a flight with ArduPilot's PID on a known plant -> identify the plant from the data ->
//  synthesise gains -> simulate again and check the predicted and achieved responses.
'use strict';
const assert = require('assert');
const { freqResponse } = require('../src/dsp/spectra');
const { C, gyroChain } = require('../src/model/controller');
const { fitPlant } = require('../src/model/plant');
const { synthesize, synthesizeQuik, simulateRate, simulateRateStep, margins, openLoop, closedLoop, logspace } = require('../src/model/synth');
const { stepMetrics } = require('../src/dsp/stepresp');
const { PRESETS, filterRules, iRule, gainLimits } = require('../src/model/rules');

function pilotReference(n, fs, seed = 7) {
  // random stick taps: hold a rate demand 0.2-0.6 s then zero 0.3-1 s, amplitude +-20..60 deg/s
  let rnd = seed; const rand = () => { rnd = (rnd * 1664525 + 1013904223) >>> 0; return rnd / 4294967296; };
  const r = new Float64Array(n); let k = 0;
  while (k < n) {
    const hold = Math.round((0.2 + 0.4 * rand()) * fs), gap = Math.round((0.3 + 0.7 * rand()) * fs);
    const amp = (rand() < 0.5 ? -1 : 1) * (20 + 40 * rand());
    for (let i = 0; i < hold && k < n; i++, k++) r[k] = amp;
    k += gap;
  }
  return r;
}

function identify(ref, u, y, fs, gyro, kind) {
  // closed-loop safe estimate: H_plant = (S_ry / S_rr) / (S_ru / S_rr)
  const win = fs >= 300 ? 1024 : 256;
  const ry = freqResponse(ref, y, fs, win, 0.75, 1);
  const ru = freqResponse(ref, u, fs, win, 0.75, 1);
  const Hmeas = C.div(ry.H, ru.H);
  const coh = new Float64Array(ry.f.length); for (let i = 0; i < coh.length; i++) coh[i] = Math.min(ry.coh[i], ru.coh[i]);
  // divide out the known gyro filter chain -> bare plant
  const F = gyroChain(ry.f, gyro);
  const Hbare = C.div(Hmeas, F);
  const fit = fitPlant(ry.f, Hbare, coh, { kind, nWin: ry.nWin, fMin: 0.5, fMax: Math.min(60, fs / 4) });
  return { fit, f: ry.f, Hbare, coh, nWin: ry.nWin };
}

function runCase(name, truePlant, startGains, fs, gyroHz, kind, opts = {}) {
  const gyro = { backendHz: 1000, gyroFilterHz: gyroHz, notches: [] };
  const ctxTrue = { fsLoop: fs, gyro, plant: truePlant };
  const n = Math.round(60 * fs);
  const ref = pilotReference(n, fs);
  const sim = simulateRate(ctxTrue, startGains, ref, { noiseRms: opts.noise ?? 1.5, uMax: opts.uMax ?? 1 });
  const id = identify(ref, sim.u, sim.y, fs, gyro, kind);
  assert(id.fit, name + ': fit failed');
  const fit = id.fit;
  // in-band model error (what matters for synthesis): 0.5..10 Hz
  const { evalPlant } = require('../src/model/plant');
  const fb = logspace(0.5, 10, 30); const Gf = evalPlant(fb, fit), Gt = evalPlant(fb, truePlant);
  let maxMag = 0, maxPh = 0;
  for (let i = 0; i < fb.length; i++) { const m = Math.abs(Math.log(Math.hypot(Gf.re[i], Gf.im[i]) / Math.hypot(Gt.re[i], Gt.im[i]))); let ph = (Math.atan2(Gf.im[i], Gf.re[i]) - Math.atan2(Gt.im[i], Gt.re[i])) * 180 / Math.PI; ph = Math.abs(((ph + 180) % 360 + 360) % 360 - 180); if (m > maxMag) maxMag = m; if (ph > maxPh) maxPh = ph; }
  console.log(`${name}: fit K=${fit.K.toFixed(1)} (true ${truePlant.K}) a=${fit.a.toFixed(2)} (${truePlant.a}) tau=${fit.tau.toFixed(3)} (${truePlant.tau}) Td=${fit.Td.toFixed(3)} (${truePlant.Td}) quality=${fit.fitQuality.toFixed(2)} bins=${fit.nBins} windows=${id.nWin}`);
  console.log(`  in-band model error: magnitude ${(100 * (Math.exp(maxMag) - 1)).toFixed(0)}%  phase ${maxPh.toFixed(0)} deg`);
  assert(maxMag < 0.3, name + ': in-band magnitude error too large');
  assert(maxPh < 20, name + ': in-band phase error too large');
  // synthesise with the identified plant
  const ctxId = { fsLoop: fs, gyro, plant: fit };
  const lim0 = gainLimits(kind === 'plane' ? 'plane' : 'copter', 'roll'); const lim = { P: lim0.P.map(v => v / 57.3), D: lim0.D.map(v => v / 57.3), FF: lim0.FF };
  const filters = filterRules(kind === 'plane' ? 'plane' : 'copter', 'roll', startGains, gyroHz, 0.5);
  // gyro noise PSD as the app would measure it (white noise at the backend rate here)
  const nf = logspace(0.5, 500, 200); const sigma = opts.noise ?? 1.5; const npsd = Float64Array.from(nf, () => sigma * sigma / (gyro.backendHz / 2));
  // uniform grid needed by the integrator in synth: resample to linear spacing
  const lf = Float64Array.from({ length: 400 }, (_, i) => 0.5 + i * 1.0); const lpsd = Float64Array.from(lf, () => sigma * sigma / (gyro.backendHz / 2));
  const sopts = Object.assign({}, PRESETS.normal, { pRange: lim.P, dRange: lim.D, iRatio: 1.0, filters, dNoiseCap: 0.05 * (opts.uMax ?? 1), noisePsd: { f: lf, psd: lpsd }, uMax: opts.uMax ?? 1, currentP: startGains.P, currentD: startGains.D });
  let res = synthesizeQuik(ctxId, sopts); if (!res || !(res.gains.P > 0)) res = synthesize(ctxId, sopts);
  assert(res, name + ': synthesis failed');
  const g = res.gains;
  console.log(`  proposed P=${g.P.toFixed(4)} I=${g.I.toFixed(4)} D=${g.D.toFixed(5)}  GM=${res.margins.gmDb.toFixed(1)}dB PM=${res.margins.pmDeg.toFixed(0)}deg wc=${res.margins.wcHz.toFixed(1)}Hz Ms=${res.margins.ms.toFixed(2)} bw=${res.bandwidthHz.toFixed(1)}Hz`);
  // true margins with the TRUE plant (how good is the design really?)
  const freqs = logspace(0.1, fs / 2, 400);
  const mTrue = margins(freqs, openLoop(freqs, ctxTrue, g).L);
  console.log(`  true-plant margins: GM=${mTrue.gmDb.toFixed(1)}dB PM=${mTrue.pmDeg.toFixed(0)}deg wc=${mTrue.wcHz.toFixed(1)}Hz Ms=${mTrue.ms.toFixed(2)}`);
  // the design keeps QuikTune's 40% of the oscillation gain; the preset's safety net is GM 4 dB / PM 25 deg on the
  // identified model, so on the TRUE plant the margins must stay above that net minus a small model-error tolerance
  assert(mTrue.gmDb > 3.5 && mTrue.pmDeg > 22, name + ': true margins too small');
  // step response with new gains on the true plant
  const st = simulateRateStep(ctxTrue, g, { stepDegs: 30, durationS: 1.0, uMax: 1 });
  const met = stepMetrics(st.t, st.y.map(v => v / 30));
  const st0 = simulateRateStep(ctxTrue, startGains, { stepDegs: 30, durationS: 1.0, uMax: 1 });
  const met0 = stepMetrics(st0.t, st0.y.map(v => v / 30));
  console.log(`  step before: rise=${(met0.riseTime * 1000).toFixed(0)}ms overshoot=${met0.overshootPct.toFixed(0)}% settle=${(met0.settlingTime * 1000).toFixed(0)}ms | after: rise=${(met.riseTime * 1000).toFixed(0)}ms overshoot=${met.overshootPct.toFixed(0)}% settle=${(met.settlingTime * 1000).toFixed(0)}ms`);
  assert(met.overshootPct < 25, name + ': overshoot too large');
  assert(met.riseTime < 0.25, name + ': too slow');
  // frequency-domain prediction vs time simulation consistency (closed-loop DC gain ~ 1 and bandwidth sane)
  const cl = closedLoop(freqs, ctxTrue, g);
  const dc = Math.hypot(cl.Tcl.re[0], cl.Tcl.im[0]);
  assert(Math.abs(dc - 1) < 0.05, name + ': closed-loop DC gain off: ' + dc);
  return { fit, g, res };
}

// Case 1: 10" quad, 400 Hz loop, gyro 40 Hz. Plant: K=6000 deg/s^2 per unit mixer, a=1, tau=0.05, Td=0.012
runCase('quad10', { K: 2000, a: 1.0, tau: 0.05, Td: 0.012 }, { P: 0.05, I: 0.05, D: 0.001, FF: 0, FLTT: 20, FLTE: 0, FLTD: 20 }, 400, 40, 'copter');
// Case 2: big 30" hexa, 400 Hz, gyro 20 Hz, slow motors
runCase('heavy30', { K: 500, a: 0.5, tau: 0.12, Td: 0.02 }, { P: 0.08, I: 0.08, D: 0.002, FF: 0, FLTT: 10, FLTE: 0, FLTD: 10 }, 400, 20, 'copter', { noise: 1.0 });
// Case 3: 5" racer, 400 Hz, gyro 80 Hz, fast
runCase('racer5', { K: 8000, a: 2.0, tau: 0.015, Td: 0.006 }, { P: 0.03, I: 0.03, D: 0.0005, FF: 0, FLTT: 40, FLTE: 0, FLTD: 40 }, 400, 80, 'copter', { noise: 3 });
// Case 4: same quad but starting from a sluggish tune (half the gains) and gentle flying
runCase('quad10-sluggish', { K: 2000, a: 1.0, tau: 0.05, Td: 0.012 }, { P: 0.02, I: 0.02, D: 0.0005, FF: 0, FLTT: 20, FLTE: 0, FLTD: 20 }, 400, 40, 'copter');
console.log('synthetic pipeline OK');
