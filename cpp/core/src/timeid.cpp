#include "aptuner/core/timeid.hpp"

#include <algorithm>
#include <cmath>

#include "aptuner/core/jsnum.hpp"
#include "aptuner/core/plant.hpp"

namespace aptuner::core {

Vec lpf2(const Vec& x, double fs, double fc) {
  const std::size_t n = x.size();
  Vec y(n);
  if (!(fc > 0) || fc >= fs / 2) {
    y = x;
    return y;
  }
  const double K = std::tan(kPi * fc / fs), q = std::sqrt(2.0), norm = 1 / (1 + K * q + K * K);
  const double b0 = K * K * norm, b1 = 2 * b0, b2 = b0, a1 = 2 * (K * K - 1) * norm, a2 = (1 - K * q + K * K) * norm;
  double x1 = n ? x[0] : 0, x2 = x1, y1 = x1, y2 = x1;
  for (std::size_t i = 0; i < n; i++) {
    const double v = b0 * x[i] + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = x[i];
    y2 = y1;
    y1 = v;
    y[i] = v;
  }
  return y;
}

Vec lag1(const Vec& x, double fs, double tau) {
  const std::size_t n = x.size();
  Vec y(n);
  if (!(tau > 0)) {
    y = x;
    return y;
  }
  const double al = 1 - std::exp(-1 / (fs * tau));
  double s = n ? x[0] : 0;
  for (std::size_t i = 0; i < n; i++) {
    s += al * (x[i] - s);
    y[i] = s;
  }
  return y;
}

namespace {

using Range = std::pair<std::size_t, std::size_t>;   // [i0, i1)

Vec clip(const Vec& u, double L) {
  if (!(L != 0)) return u;
  Vec o(u.size());
  for (std::size_t i = 0; i < u.size(); i++) o[i] = u[i] > L ? L : u[i] < -L ? -L : u[i];
  return o;
}

std::vector<Range> segmentsOf(const std::vector<std::uint8_t>& valid, std::size_t n, std::size_t minN) {
  if (valid.empty()) return {{0, n}};
  std::vector<Range> out;
  std::size_t i = 0;
  while (i < n) {
    if (!valid[i]) {
      i++;
      continue;
    }
    std::size_t j = i;
    while (j < n && valid[j]) j++;
    if (j - i >= minN) out.emplace_back(i, j);
    i = j;
  }
  return out;
}

struct Fixed {
  double b = 0, a = 0, sse = 0, r2 = 0;
  double m = 0, invUU = 0, uRms = 0;
  Vec trims;
  double tau = 0;
  std::size_t d = 0;
  double L = 0;
};

/** least squares for a fixed lag / delay with per-chunk intercepts (removed by demeaning inside each chunk) */
std::optional<Fixed> fitFixed(const Vec& rd, const Vec& uf, const Vec& rf, std::size_t d,
                              const std::vector<Range>& chunks) {
  double suu = 0, srr = 0, sur = 0, syu = 0, syr = 0, syy = 0;
  double m = 0;
  struct Means {
    double my, mu, mr;
  };
  std::vector<Means> means;
  for (const auto& [i0, i1] : chunks) {
    const double k = static_cast<double>(i1 - i0);
    if (k < 20) continue;
    double my = 0, mu = 0, mr = 0;
    for (std::size_t i = i0; i < i1; i++) {
      my += rd[i];
      mu += uf[i - d];
      mr += rf[i];
    }
    my /= k;
    mu /= k;
    mr /= k;
    means.push_back({my, mu, mr});
    for (std::size_t i = i0; i < i1; i++) {
      const double y = rd[i] - my, x1 = uf[i - d] - mu, x2 = -(rf[i] - mr);
      suu += x1 * x1;
      srr += x2 * x2;
      sur += x1 * x2;
      syu += y * x1;
      syr += y * x2;
      syy += y * y;
      m++;
    }
  }
  const double det = suu * srr - sur * sur;
  if (!(det > 0) || m < 100) return std::nullopt;
  Fixed f;
  f.b = (syu * srr - syr * sur) / det;
  f.a = (syr * suu - syu * sur) / det;
  f.sse = std::max(0.0, syy - f.b * syu - f.a * syr);
  // chunk mean: my = b (mu - trim) - a mr  ->  trim = mu - (my + a mr) / b
  for (const auto& q : means) f.trims.push_back(q.mu - (q.my + f.a * q.mr) / f.b);
  f.r2 = syy > 0 ? 1 - f.sse / syy : 0;
  f.m = m;
  f.invUU = srr / det;
  f.uRms = std::sqrt(suu / m);
  return f;
}

std::size_t pieceLen(std::size_t seg, std::size_t c0, std::size_t c1, std::size_t dN) {
  const std::size_t start = std::max(c0, dN + 1);
  const long long avail = static_cast<long long>(c1) - static_cast<long long>(start);
  return static_cast<std::size_t>(std::max<long long>(1, std::min<long long>(static_cast<long long>(seg), avail)));
}

/** variance accounted for of the rate, simulating segS-second pieces of each chunk from the measured start rate */
double replay(const Vec& r, const Vec& u, double fs, const TimeIdFit& fit, const std::vector<Range>& chunks,
              double segS) {
  const std::size_t seg = static_cast<std::size_t>(jsRound(segS * fs));
  const std::size_t dN = static_cast<std::size_t>(jsRound(fit.Td * fs));
  const double L = fit.uLimit.value_or(0);
  const double al = fit.tau > 0 ? 1 - std::exp(-1 / (fs * fit.tau)) : 1, eA = std::exp(-fit.a / fs),
               gA = (1 - eA) / fit.a;
  auto sat = [&](double v) { return L != 0 ? std::max(-L, std::min(L, v)) : v; };
  double se = 0, sv = 0;
  for (std::size_t ci = 0; ci < chunks.size(); ci++) {
    const auto [c0, c1] = chunks[ci];
    const double trim = fit.trims[std::min(ci, fit.trims.size() - 1)];
    const std::size_t sg = pieceLen(seg, c0, c1, dN);
    for (std::size_t s = std::max(c0, dN + 1); s + sg <= c1; s += sg) {
      double x = r[s], ml = sat(u[s - dN]), mr = 0;
      for (std::size_t i = s; i < s + sg; i++) mr += r[i];
      mr /= static_cast<double>(sg);
      for (std::size_t i = s; i < s + sg; i++) {
        const double e = r[i] - x;
        se += e * e;
        sv += (r[i] - mr) * (r[i] - mr);
        ml += al * (sat(u[i - dN]) - ml);
        x = x * eA + fit.K * (ml - trim) * gA;
      }
    }
  }
  return sv > 0 ? 1 - se / sv : 0;
}

struct OutputError {
  double K = 0, a = 0;
  std::optional<double> uLimit;
  Vec trims;
  double vaf = 0;
};

/** output-error fit of b, a and the mixer limit (lag and delay from the equation fit); trims solved exactly */
std::optional<OutputError> outputErrorFit(const Vec& r, const Vec& u, double fs, const std::vector<Range>& chunks,
                                          double segS, const TimeIdFit& init) {
  const std::size_t seg = static_cast<std::size_t>(jsRound(segS * fs));
  const std::size_t dN = static_cast<std::size_t>(jsRound(init.Td * fs));
  const double al = init.tau > 0 ? 1 - std::exp(-1 / (fs * init.tau)) : 1;
  double uMaxAbs = 0;
  for (const auto& [c0, c1] : chunks)
    for (std::size_t i = c0; i < c1; i++) uMaxAbs = std::max(uMaxAbs, std::fabs(u[i]));
  struct Eval {
    double vaf;
    Vec trims;
  };
  auto evalAt = [&](double K, double a, double L) {
    const double eA = std::exp(-a / fs), gA = (1 - eA) / a;
    auto sat = [&](double v) { return L != 0 ? std::max(-L, std::min(L, v)) : v; };
    double se = 0, sv = 0;
    Eval ev;
    Vec res, gs;
    for (const auto& [c0, c1] : chunks) {
      // free response (trim 0) and the unit-trim response g for each piece
      res.clear();
      gs.clear();
      double sRg = 0, sGg = 0;
      const std::size_t sg = pieceLen(seg, c0, c1, dN);
      for (std::size_t s = std::max(c0, dN + 1); s + sg <= c1; s += sg) {
        double x = r[s], g = 0, ml = sat(u[s - dN]);
        for (std::size_t i = s; i < s + sg; i++) {
          res.push_back(r[i] - x);
          gs.push_back(g);
          ml += al * (sat(u[i - dN]) - ml);
          x = x * eA + K * ml * gA;
          g = g * eA - K * gA;
        }
      }
      for (std::size_t k = 0; k < res.size(); k++) {
        sRg += res[k] * gs[k];
        sGg += gs[k] * gs[k];
      }
      const double trim = sGg > 0 ? sRg / sGg : 0;
      ev.trims.push_back(trim);
      for (std::size_t k = 0; k < res.size(); k++) {
        const double e = res[k] - trim * gs[k];
        se += e * e;
      }
      // variance of the measured rate inside each piece
      for (std::size_t s = std::max(c0, dN + 1); s + sg <= c1; s += sg) {
        double mr = 0;
        for (std::size_t i = s; i < s + sg; i++) mr += r[i];
        mr /= static_cast<double>(sg);
        for (std::size_t i = s; i < s + sg; i++) sv += (r[i] - mr) * (r[i] - mr);
      }
    }
    ev.vaf = sv > 0 ? 1 - se / sv : 0;
    return ev;
  };
  Vec Ls;
  for (double L : {0.0, 1.0, 0.9, 0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5, 0.45, 0.4})
    if (L == 0 || L < uMaxAbs * 0.98) Ls.push_back(L);
  struct Best {
    double f, K, a, L;
  };
  std::optional<Best> best;
  for (const double L : Ls) {
    auto cost = [&](const Vec& x) { return 1 - evalAt(std::exp(x[0]), std::exp(x[1]), L).vaf; };
    const NelderMeadResult res =
        nelderMead(cost, {std::log(std::max(init.K, 1e-3)), std::log(std::max(init.a, 0.02))}, {0.4, 120, 1e-5});
    if (!best || res.f < best->f) best = Best{res.f, std::exp(res.x[0]), std::exp(res.x[1]), L};
  }
  if (!best) return std::nullopt;
  Eval e = evalAt(best->K, best->a, best->L);
  OutputError o;
  o.K = best->K;
  o.a = best->a;
  if (best->L != 0) o.uLimit = best->L;
  o.trims = std::move(e.trims);
  o.vaf = e.vaf;
  return o;
}

void trimStats(TimeIdFit& fit) {
  double s = 0;
  for (double v : fit.trims) s += v;
  fit.trimMean = s / static_cast<double>(fit.trims.size());
  fit.trimMin = *std::min_element(fit.trims.begin(), fit.trims.end());
  fit.trimMax = *std::max_element(fit.trims.begin(), fit.trims.end());
}

}  // namespace

TimeIdResult identifyTimeDomain(const TimeIdInput& data, const TimeIdOptions& opt) {
  TimeIdResult out;
  const double fs = data.fs;
  const std::size_t n = std::min(data.actual.size(), data.u.size());
  double fc = opt.fc.has_value() && *opt.fc != 0 ? *opt.fc : (opt.axis == Axis::Yaw ? 1.0 : 2.0);
  fc = std::min(fc, fs * 0.2);
  const std::size_t skip = static_cast<std::size_t>(std::ceil(std::max(opt.tdMax * fs, 3 * fs / fc))) + 2;
  const std::vector<Range> segs =
      segmentsOf(data.airborne, n, skip + static_cast<std::size_t>(jsRound(5 * fs)));
  double used = 0;
  for (const auto& [a, b] : segs) used += static_cast<double>(b) - static_cast<double>(a) - static_cast<double>(skip);
  if (used < fs * 8) {
    out.reason = "not enough steady airborne data (" + jsToFixed(std::max(0.0, used) / fs, 1) + " s)";
    return out;
  }
  Vec r(data.actual.begin(), data.actual.begin() + static_cast<std::ptrdiff_t>(n));
  Vec u(data.u.begin(), data.u.begin() + static_cast<std::ptrdiff_t>(n));
  // the aircraft has to move on this axis for anything to be measured (RMS of the rate about its local mean)
  double sr = 0, cnt = 0;
  for (const auto& [a, b] : segs) {
    double m = 0;
    for (std::size_t i = a + skip; i < b; i++) m += r[i];
    m /= std::max(1.0, static_cast<double>(b) - static_cast<double>(a) - static_cast<double>(skip));
    for (std::size_t i = a + skip; i < b; i++) {
      sr += (r[i] - m) * (r[i] - m);
      cnt++;
    }
  }
  const double rRms = std::sqrt(sr / std::max(1.0, cnt));
  if (rRms < 2) {
    out.reason = "the aircraft barely moved on this axis (" + jsToFixed(rRms, 1) + " deg/s RMS): nothing to measure";
    return out;
  }
  const Vec rf = lpf2(r, fs, fc);
  Vec rd(n, 0.0);
  for (std::size_t i = 1; i + 1 < n; i++) rd[i] = (rf[i + 1] - rf[i - 1]) * fs / 2;
  // chunks for the slowly varying trim: inside each segment, ~chunkS long
  std::vector<Range> chunks;
  for (const auto& [a, b] : segs) {
    const std::size_t s0 = a + skip;
    const double len = static_cast<double>(b) - 1 - static_cast<double>(s0);
    const int k = static_cast<int>(std::max(1.0, jsRound(len / (opt.chunkS * fs))));
    for (int c = 0; c < k; c++)
      chunks.emplace_back(s0 + static_cast<std::size_t>(std::floor(c * len / k)),
                          s0 + static_cast<std::size_t>(std::floor((c + 1) * len / k)));
  }
  const std::size_t tdStep = static_cast<std::size_t>(std::max(1.0, jsRound(fs * 0.005)));
  const std::size_t dMax = static_cast<std::size_t>(jsRound(opt.tdMax * fs));
  double uMaxAbs = 0;
  for (const auto& [a, b] : segs)
    for (std::size_t i = a; i < b; i++) uMaxAbs = std::max(uMaxAbs, std::fabs(u[i]));
  Vec limits;
  for (double L : {1.0, 0.9, 0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5, 0.45, 0.4})
    if (L < uMaxAbs * 0.98) limits.push_back(L);
  auto fitAt = [&](double tau, std::size_t d, double L) {
    return fitFixed(rd, lpf2(lag1(clip(u, L), fs, tau), fs, fc), rf, d, chunks);
  };
  auto searchTauTd = [&](double L) {
    std::optional<Fixed> best;
    for (const double tau : opt.taus) {
      const Vec uf = lpf2(lag1(clip(u, L), fs, tau), fs, fc);
      for (std::size_t d = 0; d <= dMax; d += tdStep) {
        auto q = fitFixed(rd, uf, rf, d, chunks);
        if (q && (!best || q->sse < best->sse)) {
          q->tau = tau;
          q->d = d;
          q->L = L;
          best = std::move(q);
        }
      }
    }
    return best;
  };
  // coordinate search: lag / delay without a limit, then the mixer limit, then lag / delay again
  std::optional<Fixed> best = searchTauTd(0);
  if (!best) {
    out.reason = "least-squares fit failed (no usable data)";
    return out;
  }
  for (const double L : limits) {
    auto q = fitAt(best->tau, best->d, L);
    if (q && q->sse < best->sse * 0.995) {
      q->tau = best->tau;
      q->d = best->d;
      q->L = L;
      best = std::move(q);
    }
  }
  if (best->L != 0) {
    auto q = searchTauTd(best->L);
    if (q && q->sse <= best->sse) best = std::move(q);
  }
  TimeIdFit fit;
  fit.K = best->b;
  fit.a = best->a;
  fit.tau = best->tau;
  fit.Td = static_cast<double>(best->d) / fs;
  if (best->L != 0) fit.uLimit = best->L;
  fit.uMaxAbs = uMaxAbs;
  fit.trims = best->trims;
  fit.r2 = best->r2;
  fit.fc = fc;
  fit.chunkS = opt.chunkS;
  fit.airborneS = used / fs;
  fit.segments = static_cast<int>(segs.size());
  trimStats(fit);
  // effective number of independent samples after the low-pass filter, for the uncertainty of b
  fit.nEff = std::max(10.0, (best->m / fs) * 2 * fc);
  fit.bRelErr = std::sqrt(best->sse / std::max(1.0, fit.nEff - 2 - static_cast<double>(chunks.size())) * best->invUU) /
                std::fabs(best->b);
  fit.uRms = best->uRms;   // controller activity after removing the trims (mixer units RMS)
  fit.a = std::max(1e-3, fit.a);
  const double horizon = opt.axis == Axis::Yaw ? opt.chunkS : 1.5;
  fit.vaf = replay(r, u, fs, fit, chunks, horizon);
  // output-error refinement: weighs the slow, large motions that set the tune (on a weak yaw axis: heading swings)
  const auto oe = outputErrorFit(r, u, fs, chunks, horizon, fit);
  fit.equationError = {fit.K, fit.a, fit.uLimit, fit.r2, fit.vaf};
  const bool oeSane =
      oe && oe->K > 0 && oe->K / fit.K < 2 && oe->K / fit.K > 0.5 && oe->a < std::max(1.0, 3 * fit.a + 0.5);
  if (oeSane && oe->vaf > fit.vaf + 0.02) {
    fit.K = oe->K;
    fit.a = oe->a;
    fit.uLimit = oe->uLimit;
    fit.trims = oe->trims;
    fit.vaf = oe->vaf;
    fit.method = "output error";
    trimStats(fit);
  } else {
    fit.method = "equation error";
  }
  if (fit.uLimit) {
    double c = 0, m = 0;
    for (const auto& [a, b] : segs)
      for (std::size_t i = a; i < b; i++) {
        m++;
        if (std::fabs(u[i]) >= *fit.uLimit) c++;
      }
    fit.limitPct = 100 * c / m;
  } else {
    fit.limitPct = 0;
  }
  std::string reason;
  if (!(fit.K > 0))
    reason = "fitted control power is not positive (the data do not show the airframe answering the controller)";
  else if (best->a < -0.5)
    reason = "fitted damping is negative (unstable model): the data are not informative";
  else if (fit.bRelErr > 0.5)
    reason = "control power too uncertain (+-" + jsToFixed(fit.bRelErr * 100, 0) + "%)";
  else if (fit.vaf < 0)
    reason = "the model does not replay the measured rate (replay " + jsToFixed(fit.vaf * 100, 0) +
             "%): gusts dominate this axis";
  else if (std::max(fit.r2, fit.vaf) < 0.3)
    reason = "the model explains only " + jsToFixed(std::max(fit.r2, fit.vaf) * 100, 0) + "% of the measured rate";
  // 0..1 quality for display: how well the model replays the flight, and how certain the control power is
  fit.fitQuality = std::max(0.0, std::min(1.0, std::max(fit.r2, fit.vaf))) * std::max(0.0, 1 - std::min(1.0, fit.bRelErr));
  out.ok = reason.empty();
  out.reason = reason;
  out.fit = std::move(fit);
  return out;
}

}  // namespace aptuner::core
