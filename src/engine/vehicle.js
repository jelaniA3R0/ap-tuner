// Vehicle family detection and parameter access helpers.
'use strict';
const { FAMILIES, SUFFIXES } = require('../model/rules');

const COPTER_TYPES = new Set([2, 13, 14, 15, 29, 3]);   // quad, hexa, octo, tri, dodeca, coaxial
const HELI_TYPES = new Set([4]);
const PLANE_TYPES = new Set([1, 19, 20, 21, 22, 23, 24, 25]);  // fixed wing + VTOL types

/**
 * Decide the family from heartbeat type and (if known) parameters.
 * Plane with Q_ENABLE=1 and currently in a Q mode -> 'vtol'; otherwise 'plane'.
 */
function detectFamily(vehicle, params) {
  if (!vehicle) return null;
  const t = vehicle.type;
  if (HELI_TYPES.has(t)) return 'heli';
  if (COPTER_TYPES.has(t)) return 'copter';
  if (PLANE_TYPES.has(t)) {
    const q = params?.Q_ENABLE;
    if (q > 0 && isVtolMode(vehicle.modeName)) return 'vtol';
    return 'plane';
  }
  return null;
}

function isVtolMode(modeName) { return /^Q|^QAUTOTUNE|^LOITER_ALT_QLAND/.test(modeName || ''); }

function familyDef(family) { return FAMILIES[family === 'heli' ? 'copter' : family]; }

/** read the full gain set for an axis from a params object */
function axisGains(family, axis, params) {
  const f = familyDef(family); const pre = f.axes[axis];
  const g = {};
  for (const s of SUFFIXES) g[s] = params[pre + s];
  // ArduPilot FLTE/FLTT/FLTD may be undefined for old firmware -> treat as 0 (no filter)
  for (const k of ['FLTT', 'FLTE', 'FLTD', 'FF', 'D_FF', 'SMAX']) if (g[k] === undefined) g[k] = 0;
  g.DFF = g.D_FF;
  return g;
}

/** names of the params the tuner may write for an axis */
function axisParamNames(family, axis) {
  const f = familyDef(family); const pre = f.axes[axis];
  const names = { P: pre + 'P', I: pre + 'I', D: pre + 'D', FF: pre + 'FF', FLTT: pre + 'FLTT', FLTE: pre + 'FLTE', FLTD: pre + 'FLTD', SMAX: pre + 'SMAX', IMAX: pre + 'IMAX' };
  if (f.angleP) names.angleP = f.angleP[axis];
  if (f.accMax) names.accMax = f.accMax[axis];
  if (f.tconst && f.tconst[axis]) names.tconst = f.tconst[axis];
  return names;
}

/** ArduPlane speed scaler (Plane::calc_speed_scaler) */
function planeSpeedScaler(params, airspeed, throttlePct) {
  const ss = params.SCALING_SPEED || 15;
  if (airspeed > 0.1) {
    let s = ss / airspeed;
    const smin = (params.AIRSPEED_MIN || 9) * 0.5, smax = (params.AIRSPEED_MAX || 22) * 2;
    s = Math.min(Math.max(s, ss / smax), ss / smin);
    return Math.min(Math.max(s, 0.5), 2.0);
  }
  // no airspeed: throttle based estimate
  let s = 1 - ((throttlePct || 50) - 50) / 100 * 0.5;
  return Math.min(Math.max(s, 0.6), 1.67);
}

module.exports = { detectFamily, familyDef, axisGains, axisParamNames, planeSpeedScaler, isVtolMode };
