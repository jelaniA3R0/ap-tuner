// CLI: node tools/analyze-log.js <file.bin> [preset] [family]
'use strict';
const { analyzeLog } = require('../src/engine/loganalysis');
const file = process.argv[2]; const preset = process.argv[3] || 'normal'; const family = process.argv[4] || null;
const t0 = Date.now();
const r = analyzeLog(file, { preset, family });
console.log(`${r.file}\n${r.firmware} | family ${r.family} | flight window ${r.flightWindow.tStart?.toFixed(0)}-${r.flightWindow.tEnd?.toFixed(0)} s | param changes in log: ${r.paramChanges} | ${Date.now() - t0} ms`);
for (const [axis, a] of Object.entries(r.axes)) {
  if (a.error) { console.log(`\n== ${axis}: ${a.error}`); continue; }
  const d = a.diagnostics, c = a.current;
  console.log(`\n== ${axis}  data ${a.dataInfo.seconds.toFixed(0)} s @ ${a.dataInfo.fs} Hz (raw ${a.dataInfo.rawRateHz.toFixed(1)} Hz) scaler2=${a.dataInfo.scaler2.toFixed(2)} thr=${(a.dataInfo.throttle ?? 0).toFixed(2)}`);
  console.log(`   current: P=${c.P} I=${c.I} D=${c.D} FF=${c.FF} FLTT=${c.FLTT} FLTE=${c.FLTE} FLTD=${c.FLTD} SMAX=${c.SMAX} angleP=${c.angleP ?? '-'} tconst=${c.tconst ?? '-'}`);
  console.log(`   diagnostics: grade ${d.grade.letter} (${d.grade.score ?? 'not gradable'}) tracking ${d.trackingPct?.toFixed(0)}% rmsErr ${d.rmsErrorDegs.toFixed(2)} osc ${d.oscRmsDegs?.toFixed(2)} peak ${d.peakHz?.toFixed(1)}Hz x${d.peakProminence?.toFixed(0)} noise ${d.noiseRmsDegs?.toFixed(2)} D/P ${d.dOverP?.toFixed(2)} sat ${d.saturationPct?.toFixed(1)}% srateMax ${d.srateMax?.toFixed(1)} dmodMin ${d.dmodMin?.toFixed(2)}`);
  for (const n of d.grade.notes) console.log('     - ' + n);
  if (a.measuredStep) { const m = a.measuredStep.metrics; console.log(`   measured step (${a.measuredStep.count} windows): rise ${(m.riseTime * 1000).toFixed(0)} ms, overshoot ${m.overshootPct.toFixed(0)}%, settle ${(m.settlingTime * 1000).toFixed(0)} ms, final ${m.finalValue.toFixed(2)}`); }
  const id = a.identification;
  if (a.timeDomain) { const t = a.timeDomain; console.log(`   hover-data fit (${t.ok ? 'OK' : 'FAILED: ' + t.reason}): b=${t.bRads2} rad/s2 per unit (${t.bDegs2} deg/s2) a=${t.a} tau=${(t.tau * 1000).toFixed(0)} ms Td=${(t.Td * 1000).toFixed(0)} ms mixer limit ${t.uLimit ?? '-'} (${t.limitPct}% of time) trim ${t.trimMin}..${t.trimMax} (mean ${t.trimMean}) r2 ${t.r2} replay ${t.vaf} b +-${(t.bRelErr * 100).toFixed(0)}% [${t.method}, ${t.airborneS} s airborne in ${t.segments} part(s)]`); }
  const sat = d.saturation; if (sat) console.log(`   saturation: ${sat.saturated ? 'SATURATED' : sat.trimHeavy ? 'trim-heavy' : 'no'} | steady error ${sat.steadyErrorDegs.toFixed(1)} deg/s (demand ${sat.meanDemandDegs.toFixed(1)}, actual ${sat.meanActualDegs.toFixed(1)}) | I mean ${sat.iMean?.toFixed(3)} p95 ${sat.iP95?.toFixed(3)} / IMAX ${sat.imax} | motors pinned ${sat.pinnedPct?.toFixed(1)}%${sat.ccwMean ? ' | CCW ' + sat.ccwMean.toFixed(0) + ' us vs CW ' + sat.cwMean.toFixed(0) + ' us' : ''}`);
  if (!id.ok) console.log(`   identification: FAILED - ${id.reason || 'weak fit'}`);
  else if (id.method && id.method.startsWith('time')) console.log(`   model (${id.method}, LOWER CONFIDENCE): K=${id.fit.K.toFixed(1)} a=${id.fit.a.toFixed(3)} design lag ${(id.assumed.tau * 1000).toFixed(0)} ms delay ${(id.assumed.Td * 1000).toFixed(0)} ms quality ${id.fit.fitQuality.toFixed(2)}`);
  else console.log(`   model: K=${id.fit.K.toFixed(1)} a=${id.fit.a.toFixed(2)} tau=${(id.fit.tau * 1000).toFixed(0)} ms Td=${(id.fit.Td * 1000).toFixed(0)} ms quality ${id.fit.fitQuality.toFixed(2)} trusted to ${id.fCoh.toFixed(1)} Hz (${id.nWin} windows)`);
  for (const w of a.warnings) console.log('   ! ' + w);
  if (a.proposal) {
    const p = a.proposal; const g = p.gains;
    console.log(`   PROPOSED: P=${g.P.toFixed(4)} I=${g.I.toFixed(4)} D=${g.D.toFixed(5)} FF=${g.FF.toFixed(3)} FLTT=${g.FLTT} FLTE=${g.FLTE} FLTD=${g.FLTD} ${g.angleP ? 'angleP=' + g.angleP : ''}${g.tconst ? 'tconst=' + g.tconst : ''}`);
    console.log(`   margins: GM ${p.margins.gmDb.toFixed(1)} dB, PM ${p.margins.pmDeg.toFixed(0)} deg, crossover ${p.margins.wcHz.toFixed(1)} Hz, Ms ${p.margins.ms.toFixed(2)} | current: GM ${p.currentMargins.gmDb?.toFixed(1)} PM ${p.currentMargins.pmDeg?.toFixed(0)} wc ${p.currentMargins.wcHz?.toFixed(1)}`);
    const pm = p.predicted.proposedMetrics, cm = p.predicted.currentMetrics;
    console.log(`   predicted step: current rise ${(cm.riseTime * 1000).toFixed(0)} ms / overshoot ${cm.overshootPct.toFixed(0)}%  ->  proposed rise ${(pm.riseTime * 1000).toFixed(0)} ms / overshoot ${pm.overshootPct.toFixed(0)}%`);
    if (p.modelDesign) console.log(`   full model design (uncapped): P=${p.modelDesign.P.toFixed(3)} I=${p.modelDesign.I.toFixed(3)} D=${p.modelDesign.D.toFixed(4)} angleP=${p.modelDesign.angleP} crossover ${p.modelDesign.crossoverHz.toFixed(2)} Hz`);
    console.log('   changes: ' + p.changes.map(c => `${c.key} ${fmt(c.from)} -> ${fmt(c.to)}${c.pct !== null ? ' (' + (c.pct > 0 ? '+' : '') + c.pct.toFixed(0) + '%)' : ''}`).join(', '));
  }
}
const nz = r.noise;
console.log('\n== gyro noise / notch');
if (!nz.available) console.log('   ' + nz.reason);
else {
  console.log(`   source: ${nz.source}; peaks: ${nz.peaks.map(p => p.f + 'Hz(x' + p.prominence + ')').join(' ')}`);
  if (nz.harmonics) console.log(`   harmonic family f0=${nz.harmonics.f0} Hz: ` + nz.harmonics.members.map(m => m.k + 'x@' + m.f).join(' '));
  if (nz.note) console.log('   ' + nz.note);
  console.log(`   control-band (5-40 Hz) noise RMS: pre ${nz.rmsBand.pre} deg/s, with current filters ${nz.rmsBand.postCurrent ?? '-'}`);
  if (nz.recommended) { console.log('   RECOMMENDED: ' + JSON.stringify(nz.recommended.params) + ` INS_GYRO_FILTER=${nz.recommended.gyroFilterHz}`); console.log('   ' + nz.recommended.why + `; predicted control-band noise ${nz.recommended.rmsBandPost} deg/s; phase lag @10 Hz current ${nz.recommended.phaseLag10Hz.current} deg -> proposed ${nz.recommended.phaseLag10Hz.proposed} deg`); for (const n of nz.recommended.notes) console.log('     - ' + n); }
  if (nz.consistency && nz.consistency.length) console.log('   peak repeatability: ' + nz.consistency.map(c => c.f.toFixed(1) + ' Hz ' + c.hits + '/' + c.windows).join(', '));
  if (nz.recommendation) console.log('   ' + nz.recommendation);
}
for (const [axis, a] of Object.entries(r.axes)) if (a.issues && a.issues.length) { console.log('\n== issues: ' + axis); for (const i of a.issues) console.log('   [' + i.level + '] ' + i.text + (i.fix ? '  -> ' + i.fix : '')); }
if (r.motors) console.log('\n== motors (flying): mean PWM ' + JSON.stringify(r.motors.meanPwm) + ', at upper limit (' + r.motors.topPwm + ' us) ' + r.motors.pinnedTopPct.toFixed(1) + '% of the time, max ' + r.motors.maxPwm);
function fmt(v) { return typeof v === 'number' ? +v.toPrecision(4) : v; }
