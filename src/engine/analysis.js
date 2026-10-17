// Shared per-axis analysis used by both the live tuner and the log analyser:
//   uniformly sampled data -> identification -> synthesis -> prediction -> diagnostics.
'use strict';
const { freqResponse, psd, bandRms } = require('../dsp/spectra');
const { stepResponse, stepMetrics } = require('../dsp/stepresp');
const { C, gyroChain, notchBaseFreq } = require('../model/controller');
const { fitPlant, evalPlant } = require('../model/plant');
const { synthesize, synthesizeQuik, simulateRateStep, margins, openLoop, closedLoop, logspace } = require('../model/synth');
const { PRESETS, filterRules, iRule, gainLimits, angleRule } = require('../model/rules');
const { nextPow2 } = require('../dsp/fft');
const { identifyTimeDomain } = require('../model/timeid');

/** compact copy of a time-domain fit for the result (no per-chunk arrays beyond the trims) */
function slimTd(f) {
  const r = v => (typeof v === 'number' ? +v.toPrecision(4) : v);
  return { bDegs2: r(f.K), bRads2: r(f.K / 57.2957795), a: r(f.a), tau: r(f.tau), Td: r(f.Td), uLimit: f.uLimit, limitPct: r(f.limitPct || 0), trimMean: r(f.trimMean), trimMin: r(f.trimMin), trimMax: r(f.trimMax), r2: r(f.r2), vaf: r(f.vaf), bRelErr: r(f.bRelErr), method: f.method, airborneS: r(f.airborneS), segments: f.segments, fitQuality: r(f.fitQuality) };
}

/** Build the gyro filter chain description from parameters (first notch set resolved at hover throttle) */
function gyroChainFromParams(params, op = {}) {
  const rate = params.INS_GYRO_RATE | 0;
  const backendHz = 1000 * Math.pow(2, Math.max(0, rate));
  const notches = [];
  for (const pre of ['INS_HNTCH_', 'INS_HNTC2_']) {
    if (!(params[pre + 'ENABLE'] > 0)) continue;
    const p = { mode: params[pre + 'MODE'] | 0, freq: params[pre + 'FREQ'] || 0, ref: params[pre + 'REF'] ?? 0, fm_rat: params[pre + 'FM_RAT'] ?? 1 };
    notches.push({ enable: 1, freq: notchBaseFreq(p, op), bw: params[pre + 'BW'] || 0, att: params[pre + 'ATT'] ?? 40, hmncs: params[pre + 'HMNCS'] ?? 3, opts: params[pre + 'OPTS'] | 0 });
  }
  return { backendHz, gyroFilterHz: params.INS_GYRO_FILTER ?? 20, notches };
}

/**
 * Resample irregular samples (t ascending, seconds) onto a uniform grid at fs.
 * Returns {fs, t0, n, cols:{name: Float64Array}}. Gaps longer than maxGap are left NaN-free by holding.
 */
function resample(t, cols, fs) {
  const n = t.length; if (n < 4) return null;
  const t0 = t[0], t1 = t[n - 1];
  const m = Math.floor((t1 - t0) * fs) + 1;
  const out = {}; for (const k of Object.keys(cols)) out[k] = new Float64Array(m);
  let j = 0;
  for (let i = 0; i < m; i++) {
    const tt = t0 + i / fs;
    while (j < n - 2 && t[j + 1] < tt) j++;
    const dt = t[j + 1] - t[j]; const a = dt > 0 ? Math.min(1, Math.max(0, (tt - t[j]) / dt)) : 0;
    for (const k of Object.keys(cols)) { const c = cols[k]; out[k][i] = c[j] + a * (c[j + 1] - c[j]); }
  }
  return { fs, t0, n: m, cols: out };
}

/** average sample rate over the samples given (robust to bursty telemetry) */
function estimateRate(t) {
  const n = t.length; if (n < 3) return 0;
  const span = t[n - 1] - t[0];
  return span > 0 ? (n - 1) / span : 0;
}

/**
 * Identify the bare plant for one axis.
 * data: { fs, target, actual, u } uniform arrays (u = P+I+D+FF+DFF: the actuator demand);
 * gyro: chain description; family for initial guesses; scaler2: mean (speed scaler)^2 (plane) else 1
 */
function identify(data, gyro, family, { scaler2 = 1, windowS = null } = {}) {
  // planes respond slowly (roll mode ~1-2 Hz): long windows give the frequency resolution needed
  if (!windowS) windowS = family === 'plane' ? 10.24 : 2.56;
  const fs = data.fs;
  const n = Math.min(data.target.length, data.actual.length, data.u.length);
  if (n < fs * 8) return { ok: false, reason: 'not enough data (' + (n / fs).toFixed(1) + ' s)' };
  let win = nextPow2(Math.round(windowS * fs)); win = Math.max(32, Math.min(win, nextPow2(n / 4)));
  const minIn = 2; // deg/s
  const ry = freqResponse(data.target, data.actual, fs, win, 0.75, minIn);
  const ru = freqResponse(data.target, data.u, fs, win, 0.75, minIn);
  if (ry.nWin < 8) return { ok: false, reason: 'not enough pilot/autopilot excitation (' + ry.nWin + ' windows)' };
  const Hmeas = C.div(ry.H, ru.H);
  const coh = new Float64Array(ry.f.length); for (let i = 0; i < coh.length; i++) coh[i] = Math.min(ry.coh[i], ru.coh[i]);
  const F = gyroChain(ry.f, gyro);
  const Hbare = C.scale(C.div(Hmeas, F), scaler2);
  const kind = family === 'plane' ? 'plane' : 'copter';
  const fMax0 = Math.min(60, fs * 0.4, (gyro.gyroFilterHz > 0 ? gyro.gyroFilterHz * 0.9 : 60));
  // excitation band: highest frequency of the contiguous band (from the lowest bin) with coherence >= 0.3
  let fCoh = 0; for (let i = 1; i < coh.length; i++) { if (ry.f[i] > fMax0) break; if (coh[i] >= 0.3) fCoh = ry.f[i]; else if (ry.f[i] > 1.0) break; }
  const fMax = Math.min(fMax0, Math.max(fCoh * 1.1, ry.f[1] * 6));
  const fit = fitPlant(ry.f, Hbare, coh, { kind, nWin: ry.nWin, fMin: Math.max(family === 'plane' ? 0.1 : 0.3, ry.f[1]), fMax });
  if (!fit) return { ok: false, reason: 'coherence too low to fit a model: fly some sharp stick inputs', f: ry.f, coh, Hbare };
  return { ok: true, fit, f: ry.f, coh, Hmeas, Hbare, nWin: ry.nWin, fCoh, win, sxx: ry.Sxx, Hcl: ry.H, cohCl: ry.coh };
}

/** Gyro noise PSD measured from the (post-filter) actual-rate signal: use the high band where the loop is quiet */
function noiseFromActual(actual, fs) {
  const win = Math.max(64, Math.min(1024, nextPow2(fs * 1)));
  const sp = psd(actual, fs, win, 0.5);
  // uniform grid as psd gives; return as {f, psd}
  return { f: sp.f, psd: sp.psd, rmsHigh: bandRms(sp, Math.min(20, fs * 0.25), fs * 0.49) };
}

/**
 * Full analysis of one axis.
 * args: { family, axis, data:{fs,target,actual,u, P,I,D,FF}, params, preset, gyroOp:{throttle,...}, scaler2 }
 */
function analyzeAxis({ family, axis, data, params, preset = 'normal', gyroOp = {}, scaler2 = 1, fsLoop = null, designFrom = null, timeDomain = false }) {
  const fam = family === 'heli' ? 'copter' : family;
  const pre = PRESETS[preset] || PRESETS.normal;
  const loopHz = fsLoop || params.SCHED_LOOP_RATE || (fam === 'plane' ? 50 : 400);
  const gyro = gyroChainFromParams(params, gyroOp);
  const cur = currentGains(family, axis, params);
  const res = { family, axis, preset, loopHz, gyro, current: cur, warnings: [] };
  // measured closed-loop step response (data-driven, no model)
  const sr = stepResponse(data.target, data.actual, data.fs, { windowSize: Math.max(64, Math.min(1024, nextPow2(data.fs * (fam === 'plane' ? 5.12 : 1.28)))), durationS: fam === 'plane' ? 2.0 : 0.5, minInput: 15 });
  if (sr.count > 0) { const norm = sr.mean; res.measuredStep = { t: sr.t, y: norm, count: sr.count, metrics: stepMetrics(sr.t, norm) }; }
  // diagnostics on the raw data
  res.diagnostics = diagnostics(data, cur, fam, axis);
  if (fam === 'plane') res.ffEvents = planeFFEvents(data);
  // identification
  const id = identify(data, gyro, fam, { scaler2 });
  res.identification = id.ok ? { fit: id.fit, nWin: id.nWin, fCoh: id.fCoh, win: id.win } : { ok: false, reason: id.reason };
  if (id.ok) {
    const f = id.f, keep = [];
    for (let i = 1; i < f.length; i++) if (f[i] <= 100) keep.push(i);
    const Gfit = evalPlant(Float64Array.from(keep.map(i => f[i])), id.fit);
    res.identification.bode = { f: keep.map(i => +f[i].toFixed(3)), magMeas: keep.map(i => +Math.hypot(id.Hbare.re[i], id.Hbare.im[i]).toPrecision(4)), phMeas: keep.map(i => +(Math.atan2(id.Hbare.im[i], id.Hbare.re[i]) * 180 / Math.PI).toFixed(1)), coh: keep.map(i => +id.coh[i].toFixed(3)), magFit: Array.from(Gfit.re, (v, k) => +Math.hypot(v, Gfit.im[k]).toPrecision(4)), phFit: Array.from(Gfit.re, (v, k) => +(Math.atan2(Gfit.im[k], v) * 180 / Math.PI).toFixed(1)) };
    res.identification.ok = id.fit.fitQuality >= 0.4 && id.fCoh >= (fam === 'plane' ? 0.8 : 1.5);
    if (id.fit.Td >= 0.075) { res.identification.ok = false; res.warnings.push('The delay estimate hit its limit (' + (id.fit.Td * 1000).toFixed(0) + ' ms): the data rate (' + data.fs + ' Hz) is too low to resolve the loop dynamics. Log/stream the PID data faster (copter: LOG_BITMASK fast attitude; live: a faster link) before trusting P and D.'); }
    if (!res.identification.ok) res.warnings.push('Model fit is weak (quality ' + id.fit.fitQuality.toFixed(2) + ', trusted to ' + id.fCoh.toFixed(1) + ' Hz). Fly sharper stick inputs or use SystemID mode.');
  }
  // ArduPilot copter/VTOL/heli rate PIDs work in rad/s, plane in deg/s. The data and the identified
  // model are in deg/s, so for the gain search the plant is expressed per rad/s (gains then come out
  // directly in ArduPilot units).
  const us = fam === 'plane' ? 1 : 57.2957795;
  let plant = null, lowConf = false, tdFit = null;
  if (res.identification.ok) {
    // Telemetry/log data rarely resolve the actuator lag and transport delay (they sit above the
    // excited band). Low estimates are replaced by conservative floors so the design keeps its margins
    // on the real vehicle: copter 1 loop + 3 ms delay, 5 ms motor lag; plane 1 loop + 15 ms, 30 ms servo lag.
    const tdFloor = 1 / loopHz + (fam === 'plane' ? 0.015 : 0.003), tauFloor = fam === 'plane' ? 0.03 : 0.005;
    // The closed loop with the CURRENT gains is a sharp probe of the loop phase near crossover, so the
    // delay and lag are calibrated by matching the model's closed-loop response (target -> actual) to the
    // measured one; the floors are the starting point and the fit values the lower bound.
    const cal = calibrateLag(id, { fsLoop: loopHz, gyro }, cur, us, { tdFloor, tauFloor });
    plant = Object.assign({}, id.fit, { K: id.fit.K / us, Td: cal.Td, tau: cal.tau });
    res.identification.method = 'frequency response';
    res.identification.assumed = { Td: plant.Td, tau: plant.tau, floors: id.fit.Td < tdFloor || id.fit.tau < tauFloor, closedLoopMismatch: cal.mismatch, calibrated: cal.calibrated };
    if (cal.mismatch && cal.mismatch.magPct > 40) res.warnings.push('Model vs measured closed-loop mismatch ' + cal.mismatch.magPct.toFixed(0) + '% / ' + cal.mismatch.phaseDeg.toFixed(0) + ' deg: treat the proposal with care.');
  } else if (timeDomain && fam !== 'plane' && data.u) {
    // Fallback for ordinary hovers with little stick input: fit the airframe in the time domain (see
    // model/timeid.js). Lower confidence, so the design keeps a larger distance from the oscillation point.
    const td = identifyTimeDomain(data, { axis });
    res.timeDomain = td.fit ? Object.assign({ ok: td.ok, reason: td.reason }, slimTd(td.fit)) : { ok: false, reason: td.reason };
    if (td.ok) {
      tdFit = td.fit; lowConf = true;
      // a delay at either end of the search grid was not resolved: use a cautious value instead
      const tdUnresolved = tdFit.Td <= 0.005 || tdFit.Td >= 0.095;
      plant = { K: tdFit.K / us, a: tdFit.a, tau: Math.max(tdFit.tau, 0.02), Td: tdUnresolved ? 0.08 : Math.max(tdFit.Td, 1 / loopHz + 0.003) };
      res.identification = { ok: true, method: 'time domain (hover data)', confidence: 'lower', reasonFreq: id.reason || res.identification.reason || null,
        fit: { K: tdFit.K, a: tdFit.a, tau: tdFit.tau, Td: tdFit.Td, fitQuality: tdFit.fitQuality, uLimit: tdFit.uLimit, trimMean: tdFit.trimMean },
        assumed: { Td: plant.Td, tau: plant.tau, floors: tdUnresolved, calibrated: false } };
      res.warnings.push(`Lower-confidence model: the flight had too little stick input for the normal measurement, so the airframe was fitted from the ordinary hover data (it replays ${(Math.max(tdFit.vaf, tdFit.r2) * 100).toFixed(0)}% of the measured rate, control power +-${(tdFit.bRelErr * 100).toFixed(0)}%). The proposal keeps 30% of the oscillation gain. Confirm with a flight with sharp stick taps.`);
    } else res.identification.reason = (res.identification.reason ? res.identification.reason + '; ' : '') + 'hover-data fit: ' + td.reason;
  }
  if (plant) {
    const stepMag = 30 / us;
    const ctx = { fsLoop: loopHz, gyro, plant };
    const lim = gainLimits(fam, axis);
    const filters = filterRules(fam, axis, cur, gyro.gyroFilterHz, cur.tconst || 0.5);
    const noise = noiseFromActual(data.actual, data.fs);
    noise.psd = Float64Array.from(noise.psd, v => v / (us * us));
    // a yaw axis that runs near its mixer limit only has the headroom above the trim left for control
    const uMax = fam === 'plane' ? 45 : (tdFit && tdFit.uLimit ? Math.max(0.1, tdFit.uLimit - Math.abs(tdFit.trimMean || 0)) : 1);
    const hasFF = fam === 'plane' || family === 'heli' || (cur.FF > 0);
    // FF from the identified DC gain (plane: FF = 1/(K_dc * scaler), handled by scaler2 normalisation)
    let FF = 0;
    if (hasFF) FF = Math.min(lim.FF[1], Math.max(lim.FF[0], Math.sqrt(scaler2) / Math.max(plant.K / plant.a, 1e-6)));
    if (fam === 'plane' && res.ffEvents && res.ffEvents.ff) { res.ffModel = FF; FF = Math.min(lim.FF[1], Math.max(lim.FF[0], res.ffEvents.ff)); }
    const iRatio = fam === 'plane' ? 0 : (hasFF ? 0 : (axis === 'yaw' ? 0.1 : 1.0));
    // lower-confidence model: conservative margins and 30% of the oscillation gain; the gain search may go
    // past the vehicle's normal limit so the full model design can be shown (the proposal is clamped)
    const stepDurS = lowConf && plant.a < 1 ? 6 : 1.0;   // a weak, slow axis needs a longer step to judge overshoot
    const preUse = lowConf ? Object.assign({}, PRESETS.conservative, { backoff: Math.min(pre.backoff, PRESETS.conservative.backoff) }) : pre;
    const synOpts = Object.assign({}, preUse, { pRange: lowConf ? [lim.P[0], lim.P[1] * 40] : lim.P, stepDurS, dRange: lim.D, iRatio: iRatio || 1e-6, filters, uMax, dNoiseCap: fam === 'plane' ? 1.5 : 0.04, noisePsd: noise, noisePsdIsFiltered: true, rMax: fam === 'plane' ? 0.12 : 0.08, stepMag, currentP: (designFrom && designFrom.P > 0) ? designFrom.P : cur.P, currentD: designFrom ? designFrom.D : cur.D });
    let syn = synthesizeQuik(ctx, synOpts);
    if (!syn || !(syn.gains.P > 0)) syn = synthesize(ctx, synOpts);
    if (!syn) res.warnings.push('No stable gain set found within limits; check the model fit.');
    else {
      const g = syn.gains;
      let modelP = g.P, modelD = g.D, headroomNote = null, designMargins = syn.margins;
      // A weak axis (yaw on big props) can be stable with a huge P on the model; then the limit is the
      // mixer range: size P so a 10 deg/s rate error uses the whole range the motors can give (uLimit).
      if (lowConf) {
        const uRange = tdFit.uLimit || Math.min(1, tdFit.uMaxAbs || 1);   // no limit seen: the largest output used
        const pHead = uRange / (10 / 57.2957795);
        if (modelP > pHead) {
          headroomNote = `limited by the mixer range (${+uRange.toFixed(2)} of output for a 10 deg/s error), the model alone would allow ${modelP >= synOpts.pRange[1] * 0.999 ? 'more than ' : ''}${modelP.toPrecision(3)}`;
          modelP = pHead; modelD = Math.min(modelD, lim.D[1]);
          const fq = logspace(0.005, Math.min(loopHz / 2, 400), 500);
          designMargins = margins(fq, openLoop(fq, ctx, Object.assign({}, g, { P: modelP, D: modelD, I: iRule(fam, axis, modelP, FF, hasFF) })).L);
        }
        else { const fq = logspace(0.005, Math.min(loopHz / 2, 400), 500); designMargins = margins(fq, openLoop(fq, ctx, Object.assign({}, g, { I: iRule(fam, axis, g.P, FF, hasFF) })).L); }
        g.P = modelP; g.D = modelD;
      }
      g.P = clamp(g.P, lim.P); g.D = clamp(g.D, lim.D);
      g.FF = FF;
      g.I = iRule(fam, axis, g.P, FF, hasFF);
      if (fam === 'plane') { g.IMAX = Math.min(0.9, Math.max(0.4, cur.IMAX || 0.666)); }
      const wc = 2 * Math.PI * designMargins.wcHz;
      const ang = angleRule(fam, axis, wc, { yawMin: lowConf ? 0.5 : 2 });
      const proposed = Object.assign({}, g, ang);
      if (fam !== 'plane') proposed.SMAX = cur.SMAX > 0 ? cur.SMAX : 50;
      // predicted responses: current vs proposed on the identified plant
      const stepCur = simulateRateStep(ctx, Object.assign({ FF: cur.FF || 0 }, cur, { DFF: cur.DFF || 0 }), { stepDegs: stepMag, durationS: stepDurS, uMax });
      const stepNew = simulateRateStep(ctx, Object.assign({}, g, { DFF: 0 }), { stepDegs: stepMag, durationS: stepDurS, uMax });
      const freqs = logspace(lowConf ? 0.005 : 0.2, loopHz / 2, lowConf ? 500 : 300);   // a weak yaw axis crosses over far below 0.2 Hz
      const mCur = margins(freqs, openLoop(freqs, ctx, Object.assign({}, cur, { DFF: cur.DFF || 0 })).L);
      res.proposal = { gains: proposed, margins: syn.margins, bandwidthHz: syn.bandwidthHz, overshootPct: syn.overshootPct, dNoise: syn.dNoise, oscillationPoints: syn.oscillationPoints || null,
        predicted: { t: stepCur.t.filter((_, i) => i % 2 === 0), current: stepCur.y.filter((_, i) => i % 2 === 0).map(v => +(v / stepMag).toFixed(4)), proposed: stepNew.y.filter((_, i) => i % 2 === 0).map(v => +(v / stepMag).toFixed(4)),
          currentMetrics: stepMetrics(stepCur.t, stepCur.y.map(v => v / stepMag)), proposedMetrics: stepMetrics(stepNew.t, stepNew.y.map(v => v / stepMag)) },
        currentMargins: mCur, candidates: syn.candidates.map(c => ({ r: c.r, P: c.P, D: c.D, fail: c.fail || null, wc: c.margins?.wcHz, os: c.overshootPct })) };
      // safety: one analysis round never moves a gain more than x2 (up) or /2 (down); the next round continues from there
      const capNote = [];
      const capTo = (k, v) => { proposed[k] = v; capNote.push(k); };
      // P (and I tied to it) and FF: at most x2 / /2 per round; D keeps the designed D/P ratio when P is capped
      const pFrom = cur.P || 0, pTo = proposed.P; const dRatio = proposed.P > 0 ? proposed.D / proposed.P : 0;
      if (pFrom > 0 && pTo > 2 * pFrom) { capTo('P', 2 * pFrom); proposed.D = Math.min(proposed.D, dRatio * proposed.P); proposed.I = iRule(fam, axis, proposed.P, FF, hasFF); }
      if (pFrom > 0 && pTo < 0.5 * pFrom) { capTo('P', 0.5 * pFrom); proposed.D = Math.max(proposed.D, dRatio * proposed.P); proposed.I = iRule(fam, axis, proposed.P, FF, hasFF); }
      const dFrom = cur.D || 0; if (dFrom > 0 && proposed.D > 3 * dFrom) capTo('D', 3 * dFrom); if (dFrom > 0 && proposed.D < 0.33 * dFrom) capTo('D', 0.33 * dFrom);
      const fFrom = cur.FF || 0; if (fFrom > 0 && proposed.FF > 2 * fFrom) { capTo('FF', 2 * fFrom); proposed.I = iRule(fam, axis, proposed.P, proposed.FF, hasFF); } if (fFrom > 0 && proposed.FF < 0.5 * fFrom) { capTo('FF', 0.5 * fFrom); proposed.I = iRule(fam, axis, proposed.P, proposed.FF, hasFF); }
      // lower-confidence model: D moves at most 1.5x per round and is not introduced where it is 0 today
      if (lowConf) { if (!(dFrom > 0)) { if (proposed.D > 0) { proposed.D = 0; capNote.push('D'); } } else if (proposed.D > 1.5 * dFrom) capTo('D', 1.5 * dFrom); else if (proposed.D < dFrom / 1.5) capTo('D', dFrom / 1.5); }
      // angle gain / time constant: at most x1.5 per round
      if (proposed.angleP !== undefined && cur.angleP > 0) proposed.angleP = +Math.min(cur.angleP * 1.5, Math.max(cur.angleP / 1.5, proposed.angleP)).toFixed(2);
      if (proposed.tconst !== undefined && cur.tconst > 0) proposed.tconst = +Math.min(cur.tconst * 1.5, Math.max(cur.tconst / 1.5, proposed.tconst)).toFixed(2);
      // I term winding against a steady torque (trim): give it room (IMAX ~1.5x the trim it has to hold)
      const sat = res.diagnostics.saturation;
      if (fam !== 'plane' && sat && sat.iNearMax && cur.IMAX > 0) { const need = Math.min(0.9, Math.ceil(Math.max(sat.iP95 || 0, Math.abs(tdFit ? tdFit.trimMean : 0)) * 1.5 / 0.05) * 0.05); if (need > cur.IMAX + 1e-6) proposed.IMAX = +need.toFixed(2); }
      if (lowConf) {
        res.proposal.lowConfidence = true;
        const angModel = ang.angleP;
        res.proposal.modelDesign = { P: modelP, I: iRule(fam, axis, modelP, FF, hasFF), D: modelD, angleP: angModel, limitedTo: modelP > lim.P[1] ? lim.P[1] : null, crossoverHz: designMargins.wcHz, margins: designMargins, headroomNote };
        if (modelP > lim.P[1]) res.warnings.push(`The model asks for rate P ${modelP.toFixed(2)}${headroomNote ? ' (' + headroomNote + ')' : ''}, above the usual ${axis} limit ${lim.P[1]} (QuikTune maximum): this airframe has very little ${axis} authority. The proposal stops at ${lim.P[1]}; go further only in small steps after the mechanical cause is fixed.`);
      }
      if (capNote.length) { res.proposal.capped = capNote; res.warnings.push('Change limited for ' + capNote.join(', ') + ' (one round moves P/I at most 2x' + (lowConf ? ', D at most 1.5x and never from 0 on a lower-confidence model' : '') + '): fly again with the new gains and re-analyse to continue.'); }
      res.proposal.changes = describeChanges(cur, proposed);
      if (capNote.length || lowConf) {
        const gp = Object.assign({}, proposed, { DFF: 0 });
        const stepP = simulateRateStep(ctx, gp, { stepDegs: stepMag, durationS: stepDurS, uMax });
        res.proposal.designMargins = res.proposal.margins;
        res.proposal.margins = margins(freqs, openLoop(freqs, ctx, gp).L);
        res.proposal.predicted.proposed = stepP.y.filter((_, i) => i % 2 === 0).map(v => +(v / stepMag).toFixed(4));
        res.proposal.predicted.proposedMetrics = stepMetrics(stepP.t, stepP.y.map(v => v / stepMag));
      }
    }
  }
  res.diagnostics.grade = gradeAxis(res.diagnostics, { idOk: !!res.identification.ok || !!(res.proposal && res.proposal.partial), lowConf });
  // the measured step response of an ungradable / saturated axis is noise: keep it out of the report
  if (res.measuredStep && (res.diagnostics.grade.gradable === false || (res.diagnostics.saturation && res.diagnostics.saturation.saturated))) { res.measuredStepUnreliable = res.measuredStep.metrics; delete res.measuredStep; }
  if (!res.proposal && fam === 'plane' && res.ffEvents && res.ffEvents.ff) {
    // no trustworthy model: still propose the feed-forward and I (ArduPilot rule I = FF), keep P/D
    const lim = gainLimits(fam, axis);
    let FF = Math.min(lim.FF[1], Math.max(lim.FF[0], res.ffEvents.ff));
    if (cur.FF > 0) FF = Math.min(2 * cur.FF, Math.max(0.5 * cur.FF, FF));   // x2 per round
    const g = Object.assign({}, cur, { FF, I: iRule(fam, axis, cur.P, FF, true) });
    res.proposal = { gains: g, partial: true, note: 'Feed-forward from ' + res.ffEvents.events + ' stick events (ArduPilot AUTOTUNE method). P and D unchanged: the rate-loop model could not be identified from this flight.', changes: describeChanges(cur, g) };
  }
  return res;
}

/** grid-search Td and tau so that the model closed loop (current gains) matches the measured target->actual response */
function calibrateLag(id, ctx0, cur, us, { tdFloor, tauFloor }) {
  const f = id.f, H = id.Hcl, coh = id.cohCl;
  const idx = []; for (let i = 1; i < f.length; i++) if (f[i] <= id.fCoh && coh[i] >= 0.3) idx.push(i);
  const g = Object.assign({}, cur, { DFF: cur.DFF || 0, FF: cur.FF || 0 });
  if (idx.length < 4 || !(g.P > 0)) return { Td: Math.max(id.fit.Td, tdFloor), tau: Math.max(id.fit.tau, tauFloor), calibrated: false, mismatch: null };
  const fsub = Float64Array.from(idx.map(i => f[i]));
  const cost = (Td, tau) => {
    const plant = Object.assign({}, id.fit, { K: id.fit.K / us, Td, tau });
    const cl = closedLoop(fsub, Object.assign({}, ctx0, { plant }), g);
    let c = 0, cm = 0, cp = 0, w = 0;
    for (let k = 0; k < idx.length; k++) {
      const i = idx[k]; const mm = Math.hypot(H.re[i], H.im[i]), mp = Math.atan2(H.im[i], H.re[i]);
      const tm = Math.hypot(cl.Tcl.re[k], cl.Tcl.im[k]), tp = Math.atan2(cl.Tcl.im[k], cl.Tcl.re[k]);
      const em = Math.log((tm + 1e-9) / (mm + 1e-9)); let ep = tp - mp; ep = Math.atan2(Math.sin(ep), Math.cos(ep));
      c += coh[i] * (em * em + 2 * ep * ep); cm += coh[i] * Math.abs(em); cp += coh[i] * Math.abs(ep); w += coh[i];
    }
    return { c: c / w, magPct: 100 * (Math.exp(cm / w) - 1), phaseDeg: cp / w * 180 / Math.PI };
  };
  const tds = [id.fit.Td]; for (let v = 0.002; v <= Math.max(tdFloor * 2, 0.03); v += 0.002) if (v > id.fit.Td) tds.push(v);
  const taus = [id.fit.tau]; for (let v = 0.003; v <= Math.max(tauFloor * 3, 0.06); v *= 1.4) if (v > id.fit.tau) taus.push(v);
  let best = null;
  for (const Td of tds) for (const tau of taus) { const r = cost(Td, tau); if (!best || r.c < best.c) best = Object.assign({ Td, tau }, r); }
  // The best-matching lag is used as measured. (An earlier rule preferred the conservative floors whenever
  // they fitted within 10%; a direct oscillation test on the simulator showed that made the design far too
  // pessimistic. The preset back-off provides the robustness instead.)
  // Oscillation sweeps on the simulated quad and quadplane (2026-10-12) showed the open-loop fit's own lag
  // predicts the real oscillation gain within ~10%, while the lag that best matches the closed-loop response
  // is 15-30% pessimistic (the closed-loop mismatch has other causes than plant lag). So the fit's lag is
  // used for the design; the closed-loop match is kept as a diagnostic.
  const fitCost = cost(id.fit.Td, id.fit.tau);
  return { Td: id.fit.Td, tau: id.fit.tau, calibrated: true, mismatch: { magPct: fitCost.magPct, phaseDeg: fitCost.phaseDeg }, closedLoopBest: { Td: best.Td, tau: best.tau, magPct: best.magPct, phaseDeg: best.phaseDeg } };
}

function clamp(v, [lo, hi]) { return Math.min(hi, Math.max(lo, v)); }

function currentGains(family, axis, params) {
  const { axisGains } = require('./vehicle');
  const g = axisGains(family, axis, params);
  const fam = family === 'heli' ? 'copter' : family;
  const { FAMILIES } = require('../model/rules');
  const f = FAMILIES[fam];
  if (f.angleP) g.angleP = params[f.angleP[axis]];
  if (f.accMax) g.accMax = params[f.accMax[axis]];
  if (f.tconst && f.tconst[axis]) g.tconst = params[f.tconst[axis]];
  return g;
}

function describeChanges(cur, nw) {
  const out = [];
  for (const k of ['P', 'I', 'D', 'FF', 'FLTT', 'FLTE', 'FLTD', 'SMAX', 'IMAX', 'angleP', 'tconst']) {
    if (nw[k] === undefined || cur[k] === undefined) continue;
    const a = cur[k], b = nw[k];
    if (Math.abs(a - b) <= 1e-9) continue;
    const pct = a !== 0 ? ((b / a - 1) * 100) : null;
    out.push({ key: k, from: a, to: b, pct });
  }
  return out;
}

/**
 * Flight-data diagnostics for an axis: tracking error, oscillation energy, saturation, noise,
 * D-term share, I-term offset (trim), slew-rate activity.
 */
function diagnostics(data, cur, family, axis) {
  const fs = data.fs; const n = data.target.length;
  const err = new Float64Array(n); let eSq = 0, tSq = 0;
  for (let i = 0; i < n; i++) { err[i] = data.target[i] - data.actual[i]; eSq += err[i] * err[i]; tSq += data.target[i] * data.target[i]; }
  const rmsErr = Math.sqrt(eSq / n), rmsTarget = Math.sqrt(tSq / n);
  const out = { rmsErrorDegs: rmsErr, rmsTargetDegs: rmsTarget, trackingPct: rmsTarget > 1 ? 100 * Math.max(0, 1 - rmsErr / rmsTarget) : null };
  // spectral content of the actual rate: oscillation band (2 - 0.4*fs or 30 Hz) vs control band
  const win = Math.max(64, Math.min(1024, nextPow2(fs)));
  if (n >= win * 2) {
    const spA = psd(data.actual, fs, win, 0.5), spE = psd(err, fs, win, 0.5), spT = psd(data.target, fs, win, 0.5);
    const fHi = Math.min(30, fs * 0.45);
    out.oscRmsDegs = bandRms(spE, 2, fHi);          // error energy in the oscillation band
    out.lowRmsDegs = bandRms(spA, 0.05, 2);
    // dominant peak in the error spectrum above 1 Hz
    let pk = 0, pf = 0, pi = 0; for (let i = 1; i < spE.f.length; i++) if (spE.f[i] > 1 && spE.f[i] <= fHi && spE.psd[i] > pk) { pk = spE.psd[i]; pf = spE.f[i]; pi = i; }
    const med = Float64Array.from(spE.psd.slice(1)).sort()[Math.floor(spE.psd.length / 2)] || 1e-12;
    out.peakHz = pf; out.peakProminence = pk / med;
    const targetAtPeak = spT.psd[pi] || 0;
    out.peakVsTarget = targetAtPeak > 0 ? pk / targetAtPeak : Infinity;
    out.oscillating = out.peakProminence > 40 && out.oscRmsDegs > 3 && out.peakVsTarget > 4;
    out.noiseRmsDegs = bandRms(spA, Math.min(20, fs * 0.3), fs * 0.49);
  }
  // PID term shares and saturation
  if (data.P && data.D && data.I) {
    let p2 = 0, d2 = 0, iSum = 0, sat = 0, uMax = family === 'plane' ? 44 : 0.98;
    for (let i = 0; i < n; i++) { p2 += data.P[i] * data.P[i]; d2 += data.D[i] * data.D[i]; iSum += data.I[i]; if (Math.abs(data.u[i]) >= uMax) sat++; }
    out.dOverP = p2 > 0 ? Math.sqrt(d2 / p2) : 0;
    out.iMean = iSum / n;
    out.saturationPct = 100 * sat / n;
  }
  if (data.srate) { let m = 0, c = 0; for (let i = 0; i < n; i++) { if (data.srate[i] > m) m = data.srate[i]; if (data.srate[i] > 3) c++; } out.srateMax = m; out.sratePctAbove3 = 100 * c / n; }
  if (data.dmod) { let m = 1; for (let i = 0; i < n; i++) if (data.dmod[i] < m) m = data.dmod[i]; out.dmodMin = m; }
  out.saturation = saturationCheck(data, cur, family, out);
  out.grade = gradeAxis(out);
  return out;
}

/**
 * Saturation / trim detector: a rate demand the airframe persistently does not follow, an I term sitting
 * near its limit (IMAX), or motors pinned at their limit. Together they mean a steady torque the motors can
 * hardly overcome (motor tilt, prop/motor mismatch, CG): a mechanical problem that gain changes cannot fix.
 */
function saturationCheck(data, cur, family, d) {
  if (family === 'plane') return null;
  const n = data.target.length, air = data.airborne;
  let sT = 0, sA = 0, sU = 0, c = 0;
  for (let i = 0; i < n; i++) { if (air && !air[i]) continue; sT += data.target[i]; sA += data.actual[i]; sU += data.u[i]; c++; }
  if (c < 10) return null;
  const r = { meanDemandDegs: sT / c, meanActualDegs: sA / c, steadyErrorDegs: (sT - sA) / c, uMean: sU / c, imax: cur.IMAX || null };
  // I term: from the PID columns when present, else from the (slower) PID log summary
  if (data.I) {
    const v = []; let m = 0; for (let i = 0; i < n; i++) { if (air && !air[i]) continue; v.push(Math.abs(data.I[i])); m += data.I[i]; }
    v.sort((a, b) => a - b); r.iMean = m / v.length; r.iP95 = v[Math.floor(v.length * 0.95)] || 0;
  } else if (data.iStats) { r.iMean = data.iStats.mean; r.iP95 = data.iStats.p95; }
  r.persistent = Math.abs(r.steadyErrorDegs) > 8 && Math.abs(r.steadyErrorDegs) > 0.4 * (d.rmsTargetDegs || 0);
  r.iNearMax = !!(r.imax > 0 && r.iMean !== undefined && (Math.abs(r.iMean) > 0.7 * r.imax || r.iP95 > 0.85 * r.imax));
  const m = data.motors || null;
  r.pinnedPct = m ? m.pinnedTopPct : null;
  r.pinned = !!(m && m.pinnedTopPct > 1);
  if (m && m.ccwMean) { r.ccwMean = m.ccwMean; r.cwMean = m.cwMean; r.maxPwm = m.maxPwm; }
  r.saturated = r.persistent && (r.iNearMax || r.pinned) || (r.iNearMax && r.pinned);
  r.trimHeavy = !r.saturated && (r.iNearMax || Math.abs(r.iMean || 0) > 0.6 * (r.imax || 1));
  return r;
}

/**
 * Grade from the flight data. ctx.idOk: an airframe model was measured; ctx.lowConf: only the
 * lower-confidence hover-data model. A grade of A/B is only given when the tracking can be judged.
 */
function gradeAxis(d, ctx = {}) {
  const notes = [];
  let score = 100;
  if (d.oscillating) { score -= 40; notes.push(d.peakHz > 15 ? `Strong ${d.peakHz.toFixed(1)} Hz content in the rate error (${d.oscRmsDegs.toFixed(1)} deg/s RMS): at this frequency it is almost certainly motor/vibration noise reaching the controller, not a tuning oscillation. Set up the harmonic notch (Log tab, noise section) before tuning.` : `Oscillation at ${d.peakHz.toFixed(1)} Hz (${d.oscRmsDegs.toFixed(1)} deg/s RMS)`); }
  else if (d.oscRmsDegs > 4) { score -= 15; notes.push('Noticeable high-frequency error energy'); }
  if (d.trackingPct !== null) { if (d.trackingPct < 50) { score -= 25; notes.push(`Poor rate tracking (${d.trackingPct.toFixed(0)}%)`); } else if (d.trackingPct < 70) { score -= 10; notes.push(`Soft rate tracking (${d.trackingPct.toFixed(0)}%)`); } }
  if (d.saturationPct > 2) { score -= 15; notes.push(`Actuator saturated ${d.saturationPct.toFixed(1)}% of the time`); }
  if (d.dOverP > 1.5) { score -= 10; notes.push('D term dominates P (noisy D / too much D)'); }
  if (d.srateMax > 5) { score -= 10; notes.push(`Slew-rate limiter active (max ${d.srateMax.toFixed(1)})`); }
  if (d.dmodMin !== undefined && d.dmodMin < 0.9) { score -= 10; notes.push('Gains were reduced by the slew limiter (Dmod < 0.9)'); }
  if (Math.abs(d.iMean || 0) > (d.uMaxHint || 0.15)) notes.push('Large steady I term: trim / CG / motor imbalance');
  const sat = d.saturation;
  const dropTracking = () => { for (let i = notes.length - 1; i >= 0; i--) if (/rate tracking/.test(notes[i])) notes.splice(i, 1); };
  if (sat && sat.saturated) {
    dropTracking();
    score = Math.min(score, 25);
    notes.unshift(`Saturated: the airframe does not follow a steady ${sat.steadyErrorDegs.toFixed(0)} deg/s demand` + (sat.iNearMax ? `, the I term sits at ${Math.abs(sat.iMean).toFixed(2)} of its ${sat.imax} limit` : '') + (sat.pinned ? `, motors at their limit ${sat.pinnedPct.toFixed(1)}% of the time` : '') + '. Mechanical / torque imbalance: tuning cannot fix this.');
  } else if (ctx.idOk !== undefined && (d.trackingPct === null || d.trackingPct === undefined || d.trackingPct < 50)) {
    // little stick input: the demanded rate is small next to what gusts/trim add, so the tracking says nothing
    if (!ctx.idOk || (d.rmsTargetDegs || 0) < 20) {
      dropTracking();
      notes.unshift(`Not gradable: too little stick input. The demanded rate (${(d.rmsTargetDegs || 0).toFixed(1)} deg/s RMS) is smaller than what gusts and trim add (${(d.rmsErrorDegs || 0).toFixed(1)} deg/s RMS error), so the tracking cannot be judged. Fly sharp stick taps on this axis.`);
      return { score: null, letter: '–', gradable: false, notes };
    }
    score = Math.min(score, 50);
  }
  if (ctx.idOk === false && score >= 70) { score = 69; notes.push('No airframe model could be measured, so the grade is capped at C'); }
  if (ctx.lowConf && score >= 70) { score = 69; notes.push('Only a lower-confidence model was measured, so the grade is capped at C'); }
  const letter = score >= 85 ? 'A' : score >= 70 ? 'B' : score >= 55 ? 'C' : score >= 40 ? 'D' : 'F';
  return { score: Math.max(0, score), letter, gradable: true, notes };
}

/**
 * Plane feed-forward the way AP_AutoTune does it: for each stick-demand event (filtered target above
 * 40% of the usual demand) FF_single = max_actuator / (max_rate * scaler), actuator = FF+P+D (no I),
 * all filtered at 0.75 Hz; the estimate is the median over events.
 */
function planeFFEvents(data) {
  if (!data.P || !data.FF) return null;
  const n = data.target.length, fs = data.fs;
  const lp = (fc) => { const al = 1 / (1 + fs / (2 * Math.PI * fc)); let s = 0; return v => { s += al * (v - s); return s; }; };
  const fa = lp(0.75), fr = lp(0.75), ftg = lp(4);
  const absT = Array.from(data.target).map(Math.abs).sort((x, y) => x - y); const p95 = absT[Math.floor(absT.length * 0.95)] || 0;
  if (p95 < 10) return { ff: null, events: 0, reason: 'too little rate demand' };
  const thr1 = 0.4 * p95, thr2 = 0.25 * thr1;
  let state = 0, maxA = 0, minA = 0, maxR = 0, minR = 0; const ffs = [];
  for (let i = 0; i < n; i++) {
    const u = data.P[i] + (data.D ? data.D[i] : 0) + data.FF[i] + (data.I ? data.I[i] : 0);
    const clipped = Math.max(-45, Math.min(45, u)) - (data.I ? data.I[i] : 0);
    const act = fa(clipped), rat = fr(data.actual[i]), r = ftg(data.target[i]);
    const ss = data.ss ? data.ss[i] : Math.sqrt(data.scaler2 || 1);
    if (state === 0) { if (r > thr1) { state = 1; maxA = minA = maxR = minR = 0; } else if (r < -thr1) { state = -1; maxA = minA = maxR = minR = 0; } }
    else {
      maxA = Math.max(maxA, act); minA = Math.min(minA, act); maxR = Math.max(maxR, rat); minR = Math.min(minR, rat);
      if ((state === 1 && r < thr2) || (state === -1 && r > -thr2)) { const ff = state === 1 ? maxA / (maxR * ss) : minA / (minR * ss); if (isFinite(ff) && ff > 0 && ff < 5) ffs.push(ff); state = 0; }
    }
  }
  ffs.sort((x, y) => x - y);
  if (ffs.length < 6) return { ff: null, events: ffs.length, reason: 'fewer than 6 usable stick events' };
  return { ff: ffs[Math.floor(ffs.length / 2)], p25: ffs[Math.floor(ffs.length * 0.25)], p75: ffs[Math.floor(ffs.length * 0.75)], events: ffs.length };
}

module.exports = { gradeAxis, saturationCheck, planeFFEvents, analyzeAxis, identify, resample, estimateRate, gyroChainFromParams, diagnostics, noiseFromActual, currentGains };
