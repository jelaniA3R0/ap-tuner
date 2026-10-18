// What the app knows about the ArduPilot parameters it writes or exports (2026-10-17 audit F-43, F-35, Q12): the
// storage type (whole number or float), the range ArduPilot documents (@Range: advisory, ArduPilot does not enforce
// it, so outside it is a warning), the limits the app accepts at all (outside them the value is refused),
// enumerations, the highest documented bit of a bitmask and @RebootRequired (the value takes effect only after a
// reboot). From the ArduPilot parameter documentation (Copter / Plane 4.5-4.6); entries marked [AP-assumed] could not
// be checked against version-matched metadata here. The same table is in cpp/core/src/parammeta.cpp.
'use strict';

const RATE = '(?:ATC_RAT|Q_A_RAT)';
// [pattern, rule]; the first match wins
const RULES = [
  [`^${RATE}_(?:RLL|PIT)_P$`, { doc: [0.01, 0.5], hard: [0, 5] }],
  [`^${RATE}_(?:RLL|PIT)_I$`, { doc: [0.01, 2], hard: [0, 10] }],
  [`^${RATE}_(?:RLL|PIT)_D$`, { doc: [0, 0.05], hard: [0, 1] }],
  [`^${RATE}_YAW_P$`, { doc: [0.1, 2.5], hard: [0, 5] }],
  [`^${RATE}_YAW_I$`, { doc: [0.01, 1], hard: [0, 10] }],
  [`^${RATE}_YAW_D$`, { doc: [0, 0.02], hard: [0, 1] }],
  [`^${RATE}_(?:RLL|PIT|YAW)_FF$`, { doc: [0, 0.5], hard: [0, 5] }],
  [`^${RATE}_(?:RLL|PIT|YAW)_IMAX$`, { doc: [0, 1], hard: [0, 1] }],
  [`^${RATE}_(?:RLL|PIT|YAW)_FLT[TED]$`, { doc: [0, 100], hard: [0, 1000] }],           // [AP-assumed] range
  [`^${RATE}_(?:RLL|PIT|YAW)_SMAX$`, { doc: [0, 200], hard: [0, 1000] }],
  [`^${RATE}_(?:RLL|PIT|YAW)_D_FF$`, { doc: [0, 0.02], hard: [0, 1] }],
  ['^(?:ATC_ANG|Q_A_ANG)_(?:RLL|PIT|YAW)_P$', { doc: [3, 12], hard: [0.1, 30] }],
  ['^(?:RLL|PTCH)_RATE_P$', { doc: [0.08, 0.35], hard: [0, 5] }],
  ['^(?:RLL|PTCH)_RATE_I$', { doc: [0.01, 0.6], hard: [0, 5] }],
  ['^(?:RLL|PTCH)_RATE_D$', { doc: [0.001, 0.03], hard: [0, 1] }],
  ['^(?:RLL|PTCH)_RATE_FF$', { doc: [0.05, 2], hard: [0, 5] }],
  ['^YAW_RATE_(?:P|I|FF)$', { hard: [0, 5] }],                                            // [AP-assumed] no documented range used
  ['^YAW_RATE_D$', { hard: [0, 1] }],
  ['^(?:RLL|PTCH|YAW)_RATE_IMAX$', { doc: [0, 1], hard: [0, 1] }],
  ['^(?:RLL|PTCH|YAW)_RATE_FLT[TED]$', { doc: [0, 100], hard: [0, 1000] }],              // [AP-assumed] range
  ['^(?:RLL|PTCH|YAW)_RATE_SMAX$', { doc: [0, 200], hard: [0, 1000] }],
  ['^(?:RLL2SRV|PTCH2SRV)_TCONST$', { doc: [0.4, 1], hard: [0.05, 5] }],
  ['^INS_GYRO_FILTER$', { int: true, doc: [0, 256], hard: [0, 1000] }],
  ['^INS_HNTC[H2]_ENABLE$', { int: true, values: [0, 1] }],
  ['^INS_HNTC[H2]_MODE$', { int: true, values: [0, 1, 2, 3, 4, 5] }],
  ['^INS_HNTC[H2]_FREQ$', { doc: [10, 495], hard: [0, 2000] }],
  ['^INS_HNTC[H2]_BW$', { doc: [5, 250], hard: [0, 1000] }],
  ['^INS_HNTC[H2]_ATT$', { doc: [5, 50], hard: [0, 100] }],
  ['^INS_HNTC[H2]_REF$', { doc: [0, 1], hard: [0, 10], reboot: true }],
  ['^INS_HNTC[H2]_HMNCS$', { int: true, bits: 15, reboot: true }],                         // [AP-assumed] up to 16 harmonics
  ['^INS_HNTC[H2]_OPTS$', { int: true, bits: 5, reboot: true }],                           // [AP-assumed] bits 0-5 (4 = triple notch)
  ['^INS_HNTC[H2]_FM_RAT$', { doc: [0.1, 1], hard: [0, 2] }],
  ['^FFT_ENABLE$', { int: true, values: [0, 1], reboot: true }],
  ['^GCS_PID_MASK$', { int: true, bits: 4 }],
].map(([re, rule]) => [new RegExp(re), rule]);

function paramRule(name) { for (const [re, rule] of RULES) if (re.test(name)) return rule; return null; }
/** the value takes effect only after the autopilot reboots (ArduPilot @RebootRequired) */
function isRebootRequired(name) { const r = paramRule(name); return !!(r && r.reboot); }

/** {ok, errors: [text], warnings: [text], reboot, int} for a value about to be written or exported */
function validateParam(name, value) {
  const rule = paramRule(name);
  const errors = [], warnings = [];
  if (typeof value !== 'number' || !Number.isFinite(value)) errors.push('is not a number');
  else if (!rule) errors.push('is not a parameter AP Tuner can check');
  else {
    if (rule.int && !Number.isInteger(value)) errors.push('must be a whole number');
    if (rule.values && !rule.values.includes(value)) errors.push(`must be one of ${rule.values.join(', ')}`);
    if (rule.bits !== undefined && Number.isInteger(value) && (value < 0 || value >= Math.pow(2, rule.bits + 1))) errors.push(`sets bits ArduPilot does not define (bits 0 to ${rule.bits} only)`);
    if (rule.hard && (value < rule.hard[0] || value > rule.hard[1])) errors.push(`is outside ${rule.hard[0]} to ${rule.hard[1]}`);
    if (!errors.length && rule.doc && (value < rule.doc[0] || value > rule.doc[1])) warnings.push(`is outside ArduPilot's documented range ${rule.doc[0]} to ${rule.doc[1]}`);
  }
  return { ok: !errors.length, errors, warnings, reboot: !!(rule && rule.reboot), int: !!(rule && rule.int) };
}

/** decimal exponent of |x| > 0 by exact steps (the same in both editions; Math.log10 may differ in the last bit) */
function decExp(a) { let e = 0; while (a >= 10) { a /= 10; e++; } while (a < 1) { a *= 10; e--; } return e; }
function stripZeros(s) { if (s.indexOf('.') < 0) return s; s = s.replace(/0+$/, ''); return s.endsWith('.') ? s.slice(0, -1) : s; }
/**
 * A value for a .param file: fixed point, never exponent notation; whole numbers for integer parameters; floats with
 * the fewest digits that give back the same float32 (what the autopilot stores).
 */
function formatParamValue(name, value) {
  const rule = paramRule(name);
  if (rule && rule.int && Number.isInteger(value)) return value.toFixed(0);
  const f = Math.fround(value);
  if (f === 0) return '0';
  const e = decExp(Math.abs(f));
  for (let p = 1; p <= 9; p++) {
    const s = f.toFixed(Math.max(0, Math.min(20, p - 1 - e)));
    if (Math.fround(parseFloat(s)) === f) return stripZeros(s);
  }
  return stripZeros(f.toFixed(Math.max(0, Math.min(20, 9 - e))));
}

module.exports = { paramRule, isRebootRequired, validateParam, formatParamValue };
