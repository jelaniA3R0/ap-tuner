// Plain-language issues for the flight check-up: what is wrong with an axis (from the flight data) and what
// the proposal does about it. Each issue: { level: 'bad' | 'warn' | 'ok', text, fix? }.
'use strict';

const fmt = v => (typeof v === 'number' ? +v.toPrecision(3) : v);
const pctTxt = p => (p > 0 ? '+' : '') + Math.round(p) + '%';

function axisIssues(r, family) {
  const out = [];
  if (!r || r.error) { out.push({ level: 'warn', text: 'No data for this axis' + (r && r.error ? ': ' + r.error : '') }); return out; }
  const d = r.diagnostics || {}, p = r.proposal, idn = r.identification || {}, ms = r.measuredStep && r.measuredStep.metrics;
  const sat = d.saturation;
  if (sat && sat.saturated) {
    const parts = [`asks for a steady ${Math.abs(sat.steadyErrorDegs).toFixed(0)} deg/s ${r.axis === 'yaw' ? (sat.steadyErrorDegs > 0 ? 'nose-right ' : 'nose-left ') : ''}turn that the aircraft does not make`];
    if (sat.iNearMax) parts.push(`the I term holds ${Math.abs(sat.iMean).toFixed(2)} of its ${fmt(sat.imax)} limit (IMAX)`);
    if (sat.pinned) parts.push(`a motor sits at its upper limit ${sat.pinnedPct.toFixed(1)}% of the time (up to ${sat.maxPwm} us)`);
    if (r.axis === 'yaw' && sat.ccwMean && Math.abs(sat.ccwMean - sat.cwMean) > 100) parts.push(`the counter-clockwise motors run at ${sat.ccwMean.toFixed(0)} us against ${sat.cwMean.toFixed(0)} us for the clockwise ones`);
    out.push({ level: 'bad', text: `Saturated ${r.axis}: the controller ${parts.join('; ')}. This is a mechanical / torque imbalance: tuning can't fix this.`,
      fix: r.axis === 'yaw' ? 'check that all motors are vertical (inclinometer, unloaded and loaded), the props are the same type and pitch and on the right motors, then tilt the motors 1-2 deg to give yaw authority; tune yaw only after that' : 'check the centre of gravity, motor mounts and props on this axis before tuning' });
  } else if (sat && sat.trimHeavy) {
    out.push({ level: 'warn', text: `The I term holds a large steady correction on ${r.axis} (${Math.abs(sat.iMean || 0).toFixed(2)} of ${fmt(sat.imax)}): trim, centre of gravity or motor tilt`, fix: 'fix the cause mechanically; a larger IMAX only hides it' });
  }
  if (d.grade && d.grade.gradable === false) out.push({ level: 'warn', text: 'Not gradable: too little stick input. Gusts and trim moved the aircraft more than the pilot did, so how well it follows the sticks cannot be judged.', fix: 'fly 3-4 sharp stick taps each way on this axis' });
  if (p && p.lowConfidence) {
    const md = p.modelDesign || {};
    out.push({ level: 'warn', text: `Lower-confidence proposal: measured from ordinary hover data (no stick taps), so it keeps a bigger safety distance. Full model design: rate P ${fmt(md.P)}, I ${fmt(md.I)}${md.angleP ? ', angle P ' + fmt(md.angleP) : ''}; one round moves at most 2x towards it.`, fix: 'fly the new values gently, then fly sharp stick taps and re-analyse' });
  }
  if (d.oscillating) {
    if (d.peakHz > 15) out.push({ level: 'bad', text: `Noise at ${d.peakHz.toFixed(0)} Hz reaches the controller (${d.oscRmsDegs.toFixed(1)} deg/s): motor vibration rather than a tuning problem`, fix: 'the filter step deals with this' });
    else out.push({ level: 'bad', text: `Oscillating at ${d.peakHz.toFixed(1)} Hz (${d.oscRmsDegs.toFixed(1)} deg/s)`, fix: 'the gains are lowered first' });
  }
  const judged = !(d.grade && d.grade.gradable === false) && !(sat && sat.saturated);   // tracking/step numbers mean nothing otherwise
  if (judged && d.trackingPct !== null && d.trackingPct !== undefined) {
    if (d.trackingPct < 50) out.push({ level: 'bad', text: `Sluggish: follows only ${d.trackingPct.toFixed(0)}% of the demanded rate` });
    else if (d.trackingPct < 70) out.push({ level: 'warn', text: `Soft: follows ${d.trackingPct.toFixed(0)}% of the demanded rate` });
    else out.push({ level: 'ok', text: `Follows ${d.trackingPct.toFixed(0)}% of the demanded rate` });
  }
  if (judged && ms) {
    if (ms.overshootPct > 25) out.push({ level: 'warn', text: `Bouncy: overshoots ${ms.overshootPct.toFixed(0)}% on a step` });
    if (ms.riseTime > (family === 'plane' ? 0.6 : 0.15)) out.push({ level: 'warn', text: `Slow: ${(ms.riseTime * 1000).toFixed(0)} ms to answer a step` });
  }
  if (d.saturationPct > 2) out.push({ level: 'warn', text: `Motors or servos at their limit ${d.saturationPct.toFixed(1)}% of the time` });
  if (d.dOverP > 1.5) out.push({ level: 'warn', text: 'D term works harder than P (noisy gyro or too much D)' });
  if (d.srateMax > 5) out.push({ level: 'warn', text: `Slew limiter active (max ${d.srateMax.toFixed(1)}): the gains are on the edge` });
  if (!sat && Math.abs(d.iMean || 0) > 0.15) out.push({ level: 'warn', text: 'Large steady I term: trim, centre of gravity or a weak motor' });
  if (!p && !idn.ok) out.push({ level: 'warn', text: 'Could not measure the airframe: ' + (idn.reason || 'too little stick input') + '. Tap the sticks harder and more often.' });
  if (p && p.changes) {
    const c = Object.fromEntries(p.changes.map(x => [x.key, x]));
    const lowHigh = (k, name, thr) => {
      const x = c[k]; if (!x || x.pct === null || x.pct === undefined) return;
      if (x.pct > thr) out.push({ level: 'warn', text: `${name} is low for this airframe: ${pctTxt(x.pct)} proposed (${fmt(x.from)} to ${fmt(x.to)})` });
      else if (x.pct < -thr) out.push({ level: 'warn', text: `${name} is high for this airframe: ${pctTxt(x.pct)} proposed (${fmt(x.from)} to ${fmt(x.to)})` });
    };
    lowHigh('P', 'Rate P', 15); lowHigh('D', 'Rate D', 20); lowHigh('FF', 'Feed-forward', 15); lowHigh('angleP', 'Angle P', 15); lowHigh('tconst', 'Time constant', 15);
    for (const k of ['FLTT', 'FLTD', 'FLTE']) if (c[k]) out.push({ level: 'warn', text: `${k} filter ${fmt(c[k].from)} to ${fmt(c[k].to)} Hz (ArduPilot filter rule)` });
    if (!p.changes.length) out.push({ level: 'ok', text: 'The gains already match what the measurement suggests' });
  }
  if (!out.length) out.push({ level: 'ok', text: 'Nothing wrong found' });
  return out;
}

/** vehicle-level issues from the setup checks (prepare) */
function vehicleIssues(checks) {
  return (checks || []).filter(c => !c.ok).map(c => ({ level: 'warn', text: c.title, fix: c.detail }));
}

module.exports = { axisIssues, vehicleIssues };
