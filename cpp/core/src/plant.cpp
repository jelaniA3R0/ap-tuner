#include "aptuner/core/plant.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace aptuner::core {

CVec evalPlant(const Vec& freqs, const Plant& p) {
  const std::size_t n = freqs.size();
  CVec o(n);
  for (std::size_t i = 0; i < n; i++) {
    const double w = 2 * kPi * freqs[i];
    // 1/((jw + a)(j w tau + 1))
    const double d1r = p.a, d1i = w, d2r = 1, d2i = w * p.tau;
    const double dr = d1r * d2r - d1i * d2i, di = d1r * d2i + d1i * d2r;
    double dd = dr * dr + di * di;
    if (dd == 0) dd = 1e-300;
    const double gr = p.K * dr / dd, gi = -p.K * di / dd;
    const double th = -w * p.Td, c = std::cos(th), s = std::sin(th);
    o.re[i] = gr * c - gi * s;
    o.im[i] = gr * s + gi * c;
  }
  return o;
}

NelderMeadResult nelderMead(const std::function<double(const Vec&)>& f, const Vec& x0, const NelderMeadOptions& opt) {
  const std::size_t n = x0.size();
  std::vector<Vec> simplex{x0};
  for (std::size_t i = 0; i < n; i++) {
    Vec x = x0;
    x[i] = x[i] + (x[i] != 0 ? opt.step * std::fabs(x[i]) : opt.step);
    simplex.push_back(x);
  }
  Vec vals(simplex.size());
  for (std::size_t i = 0; i < simplex.size(); i++) vals[i] = f(simplex[i]);
  std::vector<std::size_t> idx(n + 1);
  for (int it = 0; it < opt.maxIter; it++) {
    std::iota(idx.begin(), idx.end(), std::size_t{0});
    // JS Array.prototype.sort with (a, b) => vals[a] - vals[b] is stable; NaN compares as "equal"
    std::stable_sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) { return vals[a] - vals[b] < 0; });
    std::vector<Vec> s2(n + 1);
    Vec v2(n + 1);
    for (std::size_t k = 0; k <= n; k++) {
      s2[k] = simplex[idx[k]];
      v2[k] = vals[idx[k]];
    }
    simplex.swap(s2);
    vals.swap(v2);
    if (std::fabs(vals[n] - vals[0]) < opt.tol * (std::fabs(vals[0]) + 1e-12)) break;
    Vec cen(n, 0.0);
    for (std::size_t i = 0; i < n; i++)
      for (std::size_t j = 0; j < n; j++) cen[j] += simplex[i][j] / static_cast<double>(n);
    Vec refl(n);
    for (std::size_t j = 0; j < n; j++) refl[j] = cen[j] + (cen[j] - simplex[n][j]);
    const double fr = f(refl);
    if (fr < vals[0]) {
      Vec ex(n);
      for (std::size_t j = 0; j < n; j++) ex[j] = cen[j] + 2 * (cen[j] - simplex[n][j]);
      const double fe = f(ex);
      if (fe < fr) {
        simplex[n] = ex;
        vals[n] = fe;
      } else {
        simplex[n] = refl;
        vals[n] = fr;
      }
    } else if (fr < vals[n - 1]) {
      simplex[n] = refl;
      vals[n] = fr;
    } else {
      Vec con(n);
      for (std::size_t j = 0; j < n; j++) con[j] = cen[j] + 0.5 * (simplex[n][j] - cen[j]);
      const double fc = f(con);
      if (fc < vals[n]) {
        simplex[n] = con;
        vals[n] = fc;
      } else {
        for (std::size_t i = 1; i <= n; i++) {
          for (std::size_t j = 0; j < n; j++) simplex[i][j] = simplex[0][j] + 0.5 * (simplex[i][j] - simplex[0][j]);
          vals[i] = f(simplex[i]);
        }
      }
    }
  }
  std::size_t best = 0;
  for (std::size_t i = 1; i <= n; i++)
    if (vals[i] < vals[best]) best = i;
  return {simplex[best], vals[best]};
}

std::optional<PlantFit> fitPlant(const Vec& freqs, const CVec& H, const Vec& coh, const FitPlantOptions& opt) {
  // with many averaged windows, bins of modest coherence still carry useful information
  const double cohMin = opt.cohMin.has_value() ? *opt.cohMin : (opt.nWin >= 40 ? 0.3 : 0.5);
  std::vector<std::size_t> idx;
  for (std::size_t i = 0; i < freqs.size(); i++)
    if (freqs[i] >= opt.fMin && freqs[i] <= opt.fMax && coh[i] >= cohMin && std::isfinite(H.re[i]) &&
        std::isfinite(H.im[i]))
      idx.push_back(i);
  if (idx.size() < 6) return std::nullopt;
  const std::size_t m = idx.size();
  Vec fsub(m), mag(m), ph(m), wgt(m);
  for (std::size_t k = 0; k < m; k++) {
    const std::size_t i = idx[k];
    fsub[k] = freqs[i];
    mag[k] = std::hypot(H.re[i], H.im[i]);
    ph[k] = std::atan2(H.im[i], H.re[i]);
    wgt[k] = coh[i] * coh[i];   // emphasise the well-measured bins
  }
  const double tdMax = opt.fixedTdMax;
  auto toPlant = [&](const Vec& x) {
    return Plant{std::exp(x[0]), std::exp(x[1]), std::exp(x[2]), std::min(tdMax, std::max(0.0, x[3]))};
  };
  // cost: weighted log-magnitude error + phase error (radians), phase wrapped
  auto cost = [&](const Vec& x) {
    const CVec G = evalPlant(fsub, toPlant(x));
    double c = 0;
    for (std::size_t k = 0; k < m; k++) {
      const double gm = std::hypot(G.re[k], G.im[k]), gp = std::atan2(G.im[k], G.re[k]);
      const double em = std::log(gm / (mag[k] != 0 ? mag[k] : 1e-9));
      double ep = gp - ph[k];
      ep = std::atan2(std::sin(ep), std::cos(ep));
      c += wgt[k] * (em * em + 2 * ep * ep);
    }
    // mild regularisation keeps a, tau, Td in physically sensible ranges
    c += 1e-3 * (x[1] * x[1] * 0.01 + x[2] * x[2] * 0.01);
    return c / static_cast<double>(m);
  };
  std::vector<Vec> guesses;
  if (opt.kind == "plane")
    guesses = {{0, std::log(6.0), std::log(0.08), 0.02}, {0, std::log(15.0), std::log(0.05), 0.03},
               {0, std::log(2.0), std::log(0.15), 0.02}};
  else
    guesses = {{0, std::log(0.3), std::log(0.04), 0.012}, {0, std::log(1.5), std::log(0.08), 0.02},
               {0, std::log(0.05), std::log(0.02), 0.008}};
  std::optional<NelderMeadResult> best;
  for (auto& g0 : guesses) {
    // scale the K guess so that the mid-band magnitude matches
    const std::size_t mid = m / 3;
    const Plant p0{1, std::exp(g0[1]), std::exp(g0[2]), g0[3]};
    const CVec G0 = evalPlant(Vec{fsub[mid]}, p0);
    const double h0 = std::hypot(G0.re[0], G0.im[0]);
    const double k0 = mag[mid] / (h0 != 0 ? h0 : 1);
    g0[0] = std::log(k0 > 0 ? k0 : 1);
    NelderMeadResult r = nelderMead(cost, g0, {0.5, 800, 1e-7});
    if (!best || r.f < best->f) best = r;
  }
  const Vec& x = best->x;
  PlantFit p;
  static_cast<Plant&>(p) = toPlant(x);
  p.cost = best->f;
  p.nBins = static_cast<int>(m);
  p.fitBandLo = fsub.front();
  p.fitBandHi = fsub.back();
  // fit quality from the weighted mean error (divided by the sum of the weights, so the same misfit gives the
  // same quality whatever the coherence): 1 at zero error, ~0.5 at 0.3 (~30% magnitude / 20 deg phase errors)
  {
    const CVec G = evalPlant(fsub, p);
    double e = 0, w = 0;
    for (std::size_t k = 0; k < m; k++) {
      const double em = std::log(std::hypot(G.re[k], G.im[k]) / (mag[k] != 0 ? mag[k] : 1e-9));
      double ep = std::atan2(G.im[k], G.re[k]) - ph[k];
      ep = std::atan2(std::sin(ep), std::cos(ep));
      e += wgt[k] * (em * em + 2 * ep * ep);
      w += wgt[k];
    }
    p.misfit = w > 0 ? e / w : std::numeric_limits<double>::infinity();
  }
  p.fitQuality = 1 / (1 + p.misfit / 0.3);
  p.dcGain = p.K / p.a;   // deg/s per unit u in steady state
  p.bandwidthHz = std::min(p.a, 1 / p.tau) / (2 * kPi);
  return p;
}

}  // namespace aptuner::core
