// Bare-airframe rate plant model and its identification from measured frequency responses.
//   G(s) = K * exp(-s Td) / ((s + a) (tau s + 1))
// a ~ 0 for multirotors (torque -> rate is nearly an integrator), a = -Lp (roll damping) for planes,
// tau = motor/ESC or servo lag, Td = pure transport delay (sensor, loop, output).
// Units: input u is the rate-PID output (copter: mixer units -1..1; plane: surface degrees),
// output y is body rate in deg/s. K then carries those units (deg/s^2 per unit u).
'use strict';
const { C } = require('./controller');

function evalPlant(freqs, p) {
  const n = freqs.length, o = C.of(n);
  for (let i = 0; i < n; i++) {
    const w = 2 * Math.PI * freqs[i];
    // 1/((jw + a)(j w tau + 1))
    const d1r = p.a, d1i = w, d2r = 1, d2i = w * p.tau;
    const dr = d1r * d2r - d1i * d2i, di = d1r * d2i + d1i * d2r;
    const dd = dr * dr + di * di || 1e-300;
    const gr = p.K * dr / dd, gi = -p.K * di / dd;
    const th = -w * p.Td, c = Math.cos(th), s = Math.sin(th);
    o.re[i] = gr * c - gi * s; o.im[i] = gr * s + gi * c;
  }
  return o;
}

// Nelder-Mead simplex minimiser (small problems)
function nelderMead(f, x0, { step = 0.3, maxIter = 600, tol = 1e-7 } = {}) {
  const n = x0.length;
  let simplex = [x0.slice()];
  for (let i = 0; i < n; i++) { const x = x0.slice(); x[i] = x[i] + (x[i] !== 0 ? step * Math.abs(x[i]) : step); simplex.push(x); }
  let vals = simplex.map(f);
  for (let it = 0; it < maxIter; it++) {
    const idx = vals.map((v, i) => i).sort((a, b) => vals[a] - vals[b]);
    simplex = idx.map(i => simplex[i]); vals = idx.map(i => vals[i]);
    if (Math.abs(vals[n] - vals[0]) < tol * (Math.abs(vals[0]) + 1e-12)) break;
    const cen = new Array(n).fill(0);
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) cen[j] += simplex[i][j] / n;
    const refl = cen.map((c, j) => c + (c - simplex[n][j]));
    const fr = f(refl);
    if (fr < vals[0]) {
      const exp = cen.map((c, j) => c + 2 * (c - simplex[n][j])); const fe = f(exp);
      if (fe < fr) { simplex[n] = exp; vals[n] = fe; } else { simplex[n] = refl; vals[n] = fr; }
    } else if (fr < vals[n - 1]) { simplex[n] = refl; vals[n] = fr; }
    else {
      const con = cen.map((c, j) => c + 0.5 * (simplex[n][j] - c)); const fc = f(con);
      if (fc < vals[n]) { simplex[n] = con; vals[n] = fc; }
      else { for (let i = 1; i <= n; i++) { simplex[i] = simplex[i].map((v, j) => simplex[0][j] + 0.5 * (v - simplex[0][j])); vals[i] = f(simplex[i]); } }
    }
  }
  let best = 0; for (let i = 1; i <= n; i++) if (vals[i] < vals[best]) best = i;
  return { x: simplex[best], f: vals[best] };
}

/**
 * Fit the plant model to a measured response H (complex) at freqs, weighted by coherence.
 * Only bins with coh >= cohMin and fMin <= f <= fMax are used.
 * kind: 'copter' (a small, fixed near 0 allowed) or 'plane' (a free, larger) — only affects initial guesses.
 * Returns { K, a, tau, Td, cost, nBins, fitBand:[f1,f2], fitQuality (0..1) }
 */
function fitPlant(freqs, H, coh, { cohMin = null, nWin = 0, fMin = 0.3, fMax = 40, kind = 'copter', fixedTdMax = 0.08 } = {}) {
  // with many averaged windows, bins of modest coherence still carry useful information
  if (cohMin === null) cohMin = nWin >= 40 ? 0.3 : 0.5;
  const idx = [];
  for (let i = 0; i < freqs.length; i++) if (freqs[i] >= fMin && freqs[i] <= fMax && coh[i] >= cohMin && isFinite(H.re[i]) && isFinite(H.im[i])) idx.push(i);
  if (idx.length < 6) return null;
  const fsub = Float64Array.from(idx.map(i => freqs[i]));
  const mag = idx.map(i => Math.hypot(H.re[i], H.im[i]));
  const ph = idx.map(i => Math.atan2(H.im[i], H.re[i]));
  const wgt = idx.map(i => coh[i] * coh[i]);   // emphasise the well-measured bins
  // cost: weighted log-magnitude error + phase error (radians), phase wrapped
  const cost = x => {
    const p = { K: Math.exp(x[0]), a: Math.exp(x[1]), tau: Math.exp(x[2]), Td: Math.min(fixedTdMax, Math.max(0, x[3])) };
    const G = evalPlant(fsub, p);
    let c = 0;
    for (let k = 0; k < idx.length; k++) {
      const gm = Math.hypot(G.re[k], G.im[k]), gp = Math.atan2(G.im[k], G.re[k]);
      const em = Math.log(gm / (mag[k] || 1e-9));
      let ep = gp - ph[k]; ep = Math.atan2(Math.sin(ep), Math.cos(ep));
      c += wgt[k] * (em * em + 2 * ep * ep);
    }
    // mild regularisation keeps a, tau, Td in physically sensible ranges
    c += 1e-3 * (x[1] * x[1] * 0.01 + x[2] * x[2] * 0.01);
    return c / idx.length;
  };
  // initial K from low-frequency magnitude of H*(jw+a)
  const guesses = kind === 'plane'
    ? [[0, Math.log(6), Math.log(0.08), 0.02], [0, Math.log(15), Math.log(0.05), 0.03], [0, Math.log(2), Math.log(0.15), 0.02]]
    : [[0, Math.log(0.3), Math.log(0.04), 0.012], [0, Math.log(1.5), Math.log(0.08), 0.02], [0, Math.log(0.05), Math.log(0.02), 0.008]];
  let best = null;
  for (const g0 of guesses) {
    // scale K guess so that mid-band magnitude matches
    const mid = Math.floor(idx.length / 3);
    const p0 = { K: 1, a: Math.exp(g0[1]), tau: Math.exp(g0[2]), Td: g0[3] };
    const G0 = evalPlant(Float64Array.from([fsub[mid]]), p0);
    const k0 = mag[mid] / (Math.hypot(G0.re[0], G0.im[0]) || 1);
    g0[0] = Math.log(k0 > 0 ? k0 : 1);
    const r = nelderMead(cost, g0, { step: 0.5, maxIter: 800 });
    if (!best || r.f < best.f) best = r;
  }
  const x = best.x;
  const p = { K: Math.exp(x[0]), a: Math.exp(x[1]), tau: Math.exp(x[2]), Td: Math.min(fixedTdMax, Math.max(0, x[3])) };
  p.cost = best.f; p.nBins = idx.length; p.fitBand = [fsub[0], fsub[fsub.length - 1]];
  // fit quality from the weighted mean error (divided by the sum of the weights, so the same misfit gives the
  // same quality whatever the coherence; dividing by the number of bins made low-coherence fits look better):
  // 1 at zero error, ~0.5 at 0.3 (≈ 30% magnitude / 20° phase errors)
  const G = evalPlant(fsub, p);
  let e = 0, w = 0;
  for (let k = 0; k < idx.length; k++) {
    const em = Math.log(Math.hypot(G.re[k], G.im[k]) / (mag[k] || 1e-9));
    let ep = Math.atan2(G.im[k], G.re[k]) - ph[k]; ep = Math.atan2(Math.sin(ep), Math.cos(ep));
    e += wgt[k] * (em * em + 2 * ep * ep); w += wgt[k];
  }
  p.misfit = w > 0 ? e / w : Infinity;
  p.fitQuality = 1 / (1 + p.misfit / 0.3);
  p.dcGain = p.K / p.a;              // deg/s per unit u in steady state
  p.bandwidthHz = Math.min(p.a, 1 / p.tau) / (2 * Math.PI);
  return p;
}

module.exports = { evalPlant, fitPlant, nelderMead };
