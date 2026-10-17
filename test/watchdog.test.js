// The step watchdog judges only the samples from after a gain change (LiveAxis `since`):
// an oscillation that was there before a change must not be blamed on the change.
'use strict';
const assert = require('assert');
const { LiveAxis } = require('../src/engine/axis');

const ax = new LiveAxis('roll');
// 10 s at 50 Hz: slew rate 7 (above the limit of 5) until t = 1006 s, calm (1) after
for (let i = 0; i < 500; i++) {
  const t = 1000 + i / 50;
  ax.push(t, { desired: 0, achieved: 0, P: 0, I: 0, D: 0, FF: 0, SRate: t < 1006 ? 7 : 1, PDmod: 1 });
}
const all = ax.watchdog(5, { oscSmax: 5 });
assert(all.oscillating && all.srateMax === 7, 'the oscillation before the change is seen without `since`');
const after = ax.watchdog(5, { oscSmax: 5, since: 1006 });
assert(!after.oscillating && after.srateMax === 1, 'samples before `since` are ignored');
const none = ax.watchdog(2, { oscSmax: 5, since: 1100 });
assert(!none.oscillating && none.samples === 0, 'nothing after `since` yet: no verdict');
const w = ax.window(null, 1008);
assert(w && Math.abs(w.n / w.fs - 2) < 0.1, 'window() starts at `since`');

// spectral check: 10 s at 100 Hz with an 8 Hz rate-error oscillation of 20 deg/s
const osc = new LiveAxis('pitch');
for (let i = 0; i < 1000; i++) {
  const t = 1000 + i / 100;
  osc.push(t, { desired: 0, achieved: 20 * Math.sin(2 * Math.PI * 8 * t), P: 0, I: 0, D: 0, FF: 0, SRate: 1, PDmod: 1 });
}
const full = osc.watchdog(2, { oscSmax: 5 });
assert(full.oscillating && full.peakHz > 7 && full.peakHz < 9, 'the spectral check sees the 8 Hz oscillation');
assert(!osc.watchdog(2, { oscSmax: 5, since: osc.lastTs - 1 }).oscillating, 'no spectral verdict on 1 s after `since`');
assert(osc.watchdog(2, { oscSmax: 5, since: osc.lastTs - 3.5 }).oscillating, 'spectral verdict once a full window came in');
console.log('watchdog test passed');
