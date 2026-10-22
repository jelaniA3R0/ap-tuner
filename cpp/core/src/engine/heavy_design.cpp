// Model-based tuner: robust design over the plant set (margins, noise, time simulations, stress tests, limits from
// physics, one gyro filter for every axis) and the predictions for a set of gains.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <set>

#include "heavy_internal.hpp"

namespace aptuner::engine::heavy {

namespace {

constexpr double kR2D = 57.29577951308232;

double roundTo(double v, double step) { return std::round(v / step) * step; }
double r3(double v) { return std::round(v * 1000) / 1000; }
double r2(double v) { return std::round(v * 100) / 100; }

/** a gain on a tidy grid: 2 significant figures */
double tidy(double v) {
  if (!(v > 0)) return 0;
  const double e = std::pow(10.0, std::floor(std::log10(v)) - 1);
  return std::round(v / e) * e;
}

// ---- yaw time metrics (grid2.js): lift-off with the trim, a yaw gust, two pilot turns
struct YawMetrics {
  double takeoff = 0, osc1 = 0, osc2 = 0, gust = 0, ovR = 0, ovL = 0, lim = 0, score = 0;
};
YawMetrics yawMetrics(const cc::Gains& g, const cc::Plant& p) {
  const double right = g.rcTrim + 0.4 * (g.rcMax - g.rcTrim), left = g.rcTrim - 0.4 * (g.rcTrim - g.rcMin);
  cc::SimInput in;
  in.seconds = 160;
  in.recordHz = 30;
  in.stickPwm = [&](double t) { return (t >= 100 && t < 104) ? right : (t >= 130 && t < 134) ? left : g.rcTrim; };
  in.dist = [](double t) { return (t >= 60 && t < 70) ? 0.08 : 0.0; };
  const cc::SimOutput s = cc::simulate(g, p, in);
  YawMetrics m;
  auto range = [&](double a, double b, double& mn, double& mx, double& pk) {
    mn = 1e9;
    mx = -1e9;
    pk = 0;
    for (std::size_t k = 0; k < s.t.size(); ++k)
      if (s.t[k] >= a && s.t[k] < b) {
        mn = std::min(mn, s.att[k]);
        mx = std::max(mx, s.att[k]);
        pk = std::max(pk, std::fabs(s.att[k]));
      }
  };
  double mn, mx, pk;
  range(0, 30, mn, mx, m.takeoff);
  m.takeoff = pk = m.takeoff;
  range(30, 45, mn, mx, pk);
  m.osc1 = mx - mn;
  range(45, 60, mn, mx, pk);
  m.osc2 = mx - mn;
  range(60, 100, mn, mx, m.gust);
  auto over = [&](double rel, double end, double dir) {
    std::size_t k0 = 0;
    while (k0 < s.t.size() && s.t[k0] < rel + 1) ++k0;
    if (k0 >= s.t.size()) return 0.0;
    const double tgt = s.target[k0];
    double o = 0;
    for (std::size_t k = k0; k < s.t.size() && s.t[k] < end; ++k) o = std::max(o, dir * cc::wrap180(s.att[k] - tgt));
    return o;
  };
  m.ovR = over(104, 130, 1);
  m.ovL = over(134, 160, -1);
  m.lim = s.limitedFrac;
  m.score = m.takeoff + 4 * m.osc2 + 2 * m.osc1 + m.gust + 0.5 * (m.ovR + m.ovL) + 20 * m.lim;
  return m;
}

// ---- roll / pitch time metrics (pdesign4.js / final.js): lift-off with the trim and gusts, gust wander, a 5 deg step
struct RpMetrics {
  double takeoff = 0, gustRms = 0, overshoot = 0, rise = 99, lim = 0, lateP2p = 0, score = 0;
};
RpMetrics rpMetrics(const cc::Gains& g, const cc::Plant& p) {
  RpMetrics m;
  const cc::GustSequence gust(41);
  cc::SimInput a;
  a.seconds = 40;
  a.shaped = true;
  a.angleDeg = [](double) { return 0.0; };
  a.dist = [&](double t) { return -gust.at(t); };
  a.recordHz = 50;
  const cc::SimOutput s = cc::simulate(g, p, a);
  double ss = 0, mn = 1e9, mx = -1e9;
  std::size_t n = 0;
  for (std::size_t k = 0; k < s.t.size(); ++k) {
    if (s.t[k] < 10) m.takeoff = std::max(m.takeoff, std::fabs(s.att[k]));
    if (s.t[k] > 15) {
      ss += s.att[k] * s.att[k];
      ++n;
    }
    if (s.t[k] > 30) {
      mn = std::min(mn, s.att[k]);
      mx = std::max(mx, s.att[k]);
    }
  }
  m.gustRms = n ? std::sqrt(ss / static_cast<double>(n)) : 0;
  m.lateP2p = mx > mn ? mx - mn : 0;
  m.lim = s.limitedFrac;
  cc::SimInput b;
  b.seconds = 12;
  b.angleDeg = [](double t) { return t > 1 ? 5.0 : 0.0; };
  b.recordHz = 100;
  cc::Plant q = p;
  q.trim = 0;
  const cc::SimOutput s2 = cc::simulate(g, q, b);
  for (std::size_t k = 0; k < s2.t.size(); ++k) {
    m.overshoot = std::max(m.overshoot, s2.att[k] - 5);
    if (m.rise > 98 && s2.att[k] >= 4.5) m.rise = s2.t[k] - 1;
  }
  m.score = m.takeoff + 2 * m.gustRms + m.overshoot + 2 * m.rise;
  return m;
}

Json rpMetricsJson(const RpMetrics& n, const RpMetrics& w) {
  return Json::obj({{"takeoffTiltDeg", Json(Json::Array{r2(n.takeoff), r2(w.takeoff)})},
                    {"gustRmsDeg", Json(Json::Array{r2(n.gustRms), r2(w.gustRms)})},
                    {"stepOvershootDeg", Json(Json::Array{r2(n.overshoot), r2(w.overshoot)})},
                    {"stepRiseS", Json(Json::Array{r2(n.rise), r2(w.rise)})},
                    {"lateOscillationDeg", Json(Json::Array{r2(n.lateP2p), r2(w.lateP2p)})},
                    {"limitedPct", Json(Json::Array{r2(100 * n.lim), r2(100 * w.lim)})}});
}

double noiseRatio(const cc::Gains& g, const cc::Gains& cur, const Vec& freqs) {
  double r = 0;
  for (double f : freqs) {
    const double c = cc::noiseGain(cur, f);
    if (c > 0) r = std::max(r, cc::noiseGain(g, f) / c);
  }
  return r;
}

// ---- roll / pitch: the best candidate for one gyro-filter route
AxisDesign designRp(const cc::Gains& cur, const AxisModel& m, double gyroHz, double acc, double inputTC, const Vec& nf, int axis) {
  AxisDesign d;
  const cc::Plant& nom = m.set[0];
  const cc::Plant& slow = m.set.size() > 1 ? m.set[1] : m.set[0];
  // rate P needed for a bandwidth the motor lag and delay allow: P ~ bandwidth / a
  const double Pc = 1 / (3 * (nom.tau + nom.T) * nom.a);
  std::set<double> Ps;
  for (double k : {0.4, 0.5, 0.6, 0.7, 0.85, 1.0, 1.2, 1.4, 1.7}) {
    const double P = tidy(k * Pc);
    if (P > 0.02 && P <= 1.5) Ps.insert(P);
  }
  const double D0 = cur.D > 0 ? cur.D : 0.0036;
  const std::vector<double> Ds{tidy(D0), tidy(1.5 * D0), tidy(2.5 * D0)};
  std::vector<std::pair<Candidate, double>> ok;
  for (double P : Ps)
    for (double ir : {0.1, 0.2, 0.3, 0.4, 0.5})
      for (double D : Ds)
        for (double K : {0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0}) {
          cc::Gains g = cur;
          g.P = P;
          g.I = r3(P * ir);
          g.D = std::min(D, 0.03);
          g.angleP = K;
          g.gyroHz = gyroHz;
          if (gyroHz != cur.gyroHz) g.FLTT = g.FLTD = gyroHz / 2;
          g.accMax = acc;
          g.inputTC = inputTC;
          ++d.candidates;
          const double nr = noiseRatio(g, cur, nf);
          if (nr > 1.15) continue;
          const cc::WorstMargins wm = cc::worstMargins(g, m.set, 40);
          if (wm.pmDeg < 40 || wm.gmDb < 10 || wm.ms > 1.8) continue;
          Candidate c;
          c.g = g;
          c.wm = wm;
          c.noiseRatio = nr;
          ok.push_back({c, 0});
        }
  d.accepted = static_cast<int>(ok.size());
  if (ok.empty()) {
    d.reason = "no gains reach phase margin 40 deg, gain margin 10 dB and Ms 1.8 on every plant of the set without more noise";
    return d;
  }
  // rank by the time simulations on the nominal and the slow plant
  double best = 1e18;
  for (auto& [c, sc] : ok) {
    const RpMetrics a = rpMetrics(c.g, nom), b = rpMetrics(c.g, slow);
    c.score = std::max(a.takeoff, b.takeoff) + 2 * std::max(a.gustRms, b.gustRms) + std::max(a.overshoot, b.overshoot) +
              2 * std::max(a.rise, b.rise) + 2 * std::max(a.lateP2p, b.lateP2p) + 20 * std::max(a.lim, b.lim);
    c.metrics = rpMetricsJson(a, b);
    c.metrics.set("score", r2(c.score));
    best = std::min(best, c.score);
  }
  // within 10% of the best time-domain score, take the one with the most phase margin (then the lower rate P)
  const Candidate* pick = nullptr;
  for (const auto& [c, sc] : ok) {
    if (c.score > 1.1 * best) continue;
    if (!pick || c.wm.pmDeg > pick->wm.pmDeg + 0.5 || (std::fabs(c.wm.pmDeg - pick->wm.pmDeg) <= 0.5 && c.g.P < pick->g.P)) pick = &c;
  }
  d.g = pick->g;
  d.wm = pick->wm;
  d.noiseRatio = pick->noiseRatio;
  d.metrics = pick->metrics;
  d.ok = true;
  (void)axis;
  return d;
}

void stressRp(AxisDesign& d, const AxisModel& m) {
  cc::Plant p = m.set[0];
  p.T += 0.05;
  const cc::Margins pm = cc::margins(d.g, p);
  cc::Plant slow = m.set.size() > 1 ? m.set[1] : m.set[0];
  slow.T += 0.05;
  const GustResult g1 = gustResponse(d.g, slow, 40);
  d.stress = Json::obj({{"pmWith50msMoreDelayDeg", pm.pmDeg ? Json(std::round(*pm.pmDeg)) : Json()},
                        {"gustAmpSlowPlant50msDelayDeg", r2(g1.ampDeg)}});
}

}  // namespace

// ------------------------------------------------------------------------------------------------- predictions

Json predictYaw(const cc::Gains& g, const AxisModel& m) {
  double lo = 1e9, hi = 0, nomSwing = 0, lateHi = 0, limHi = 0;
  for (std::size_t i = 0; i < m.set.size(); ++i) {
    cc::SimInput in;
    in.seconds = 60;
    in.recordHz = 30;
    const cc::SimOutput s = cc::simulate(g, m.set[i], in);
    double pk = 0, mn = 1e9, mx = -1e9;
    for (std::size_t k = 0; k < s.t.size(); ++k) {
      if (s.t[k] < 40) pk = std::max(pk, std::fabs(s.att[k]));
      else {
        mn = std::min(mn, s.att[k]);
        mx = std::max(mx, s.att[k]);
      }
    }
    if (i == 0) nomSwing = pk;
    lo = std::min(lo, pk);
    hi = std::max(hi, pk);
    lateHi = std::max(lateHi, mx - mn);
    limHi = std::max(limHi, s.limitedFrac);
  }
  const cc::Plant& n = m.set[0];
  const cc::Poles po = cc::slowAxisPoles(n.a, n.c, g.P, g.I, g.angleP);
  const cc::WorstMargins wm = cc::worstMargins(g, m.set);
  return Json::obj({{"swingDeg", Json(Json::Array{std::round(lo), std::round(hi)})},
                    {"swingNominalDeg", std::round(nomSwing)},
                    {"lateOscillationDeg", r2(lateHi)},
                    {"limitedPct", r2(100 * limHi)},
                    {"zeta", r2(po.zeta)},
                    {"periodS", std::round(po.periodS * 10) / 10},
                    {"pmWorstDeg", std::round(wm.pmDeg)},
                    {"ms", r2(wm.ms)},
                    {"headingErrorLimitDeg", std::round(cc::yawHeadingErrorMax(g) * kR2D)}});
}

Json predictRp(const cc::Gains& g, const AxisModel& m) {
  const cc::WorstMargins wm = cc::worstMargins(g, m.set);
  const RpMetrics a = rpMetrics(g, m.set[0]), b = rpMetrics(g, m.set.size() > 1 ? m.set[1] : m.set[0]);
  Json j = rpMetricsJson(a, b);
  j.set("pmWorstDeg", std::round(wm.pmDeg));
  j.set("gmWorstDb", wm.gmDb >= 98 ? Json() : Json(std::round(wm.gmDb * 10) / 10));
  j.set("ms", r2(wm.ms));
  j.set("worstCrossoverHz", r2(wm.wcAtWorstPmRad / (2 * kPi)));
  j.set("oscillationExpected", wm.pmDeg < 15);
  return j;
}

// ------------------------------------------------------------------------------------------------- the design

Design designAll(const DesignInputs& in) {
  Design out;
  const cc::Gains& yawCur = in.flown[2];
  out.gyroHz = in.flown[0].gyroHz;
  // ---- roll / pitch, designed together because the gyro filter is shared
  const AxisModel &mr = in.models[0], &mp = in.models[1];
  Vec nf = in.noiseFreqs;
  if (nf.empty()) nf = {1.25 * out.gyroHz, 1.85 * out.gyroHz};
  // acceleration limit from what the vehicle can do: 70% of a (u_max - |trim|), the same for both axes
  double accRp = 0;
  std::vector<std::string> accWhy;
  for (int ax = 0; ax < 2; ++ax) {
    const AxisModel& m = in.models[static_cast<std::size_t>(ax)];
    if (!m.ok) continue;
    const double cap = m.set[0].a * std::max(0.05, m.umax - std::fabs(m.trim)) * kR2D;
    const double a = std::floor(0.7 * cap / 10) * 10;
    accRp = accRp > 0 ? std::min(accRp, a) : a;
    accWhy.push_back(std::string(kAxisNames[ax]) + " can accelerate about " + std::to_string(static_cast<int>(std::round(cap))) + " deg/s/s");
  }
  const double accCur = std::min(in.flown[0].accMax > 0 ? in.flown[0].accMax : 1e9, in.flown[1].accMax > 0 ? in.flown[1].accMax : 1e9);
  if (accRp > 0) accRp = std::clamp(accRp, 40.0, accCur < 1e8 ? accCur : 720.0);
  const double tc = std::max(in.flown[0].inputTC, 0.2);
  std::vector<double> routes{out.gyroHz};
  if (in.allowFilterChange && out.gyroHz / 2 >= 8) routes.push_back(std::round(out.gyroHz / 2));
  double bestSum = 1e18;
  for (double gyro : routes) {
    std::array<AxisDesign, 2> ds;
    bool all = true;
    double sum = 0;
    for (int ax = 0; ax < 2; ++ax) {
      const AxisModel& m = in.models[static_cast<std::size_t>(ax)];
      if (!m.ok) continue;
      ds[static_cast<std::size_t>(ax)] = designRp(in.flown[static_cast<std::size_t>(ax)], m, gyro, accRp > 0 ? accRp : in.flown[static_cast<std::size_t>(ax)].accMax, tc, nf, ax);
      if (!ds[static_cast<std::size_t>(ax)].ok) all = false;
      else sum += ds[static_cast<std::size_t>(ax)].metrics["score"].numOr(0);
    }
    // a filter change also changes an axis without a model: allowed only when every modelled axis is designed for it
    if (gyro != out.gyroHz && (!mr.ok || !mp.ok)) all = false;
    if (all && sum < bestSum) {
      bestSum = sum;
      for (int ax = 0; ax < 2; ++ax) out.axes[static_cast<std::size_t>(ax)] = ds[static_cast<std::size_t>(ax)];
      out.gyroChanged = gyro != out.gyroHz;
      if (out.gyroChanged) out.gyroHz = gyro;
    } else if (bestSum > 1e17) {
      for (int ax = 0; ax < 2; ++ax)
        if (!ds[static_cast<std::size_t>(ax)].ok) out.axes[static_cast<std::size_t>(ax)] = ds[static_cast<std::size_t>(ax)];
    }
  }
  for (int ax = 0; ax < 2; ++ax) {
    AxisDesign& d = out.axes[static_cast<std::size_t>(ax)];
    const AxisModel& m = in.models[static_cast<std::size_t>(ax)];
    if (!m.ok) {
      d.reason = m.reason.empty() ? "no model of this axis" : m.reason;
      continue;
    }
    if (!d.ok) continue;
    d.lim.acc = d.g.accMax;
    d.lim.inputTC = tc;
    d.lim.imax = std::max(in.flown[static_cast<std::size_t>(ax)].IMAX, std::ceil((std::fabs(m.trimMax) + 0.2) * 10) / 10);
    d.g.IMAX = d.lim.imax;
    d.lim.why = accWhy;
    stressRp(d, m);
  }
  if (out.gyroChanged) out.notes.push_back("INS_GYRO_FILTER " + std::to_string(static_cast<int>(in.flown[0].gyroHz)) + " -> " + std::to_string(static_cast<int>(out.gyroHz)) + " Hz (target and D filters at half of it) for roll and pitch together; it needs a reboot");

  // ---- yaw
  AxisDesign& dy = out.axes[2];
  const AxisModel& my = in.models[2];
  if (!my.ok) {
    dy.reason = my.reason.empty() ? "no model of this axis" : my.reason;
    return out;
  }
  const cc::Plant& n = my.set[0];
  const double tr = std::fabs(my.trim);
  const double accWeak = n.a * std::max(0.0, my.umax - tr) * kR2D, accStrong = n.a * (my.umax + tr) * kR2D;
  Limits lim;
  lim.acc = std::clamp(roundTo(1.25 * accStrong, 5), 10.0, yawCur.accMax > 0 ? std::max(10.0, yawCur.accMax) : 120.0);
  lim.why.push_back("yaw can accelerate " + num(accWeak, 1) + " deg/s/s against the torque imbalance and " + num(accStrong, 0) +
                    " deg/s/s with it");
  lim.rateMax = 10;
  for (double v : {10.0, 15.0, 20.0, 25.0, 30.0, 45.0, 60.0, 75.0, 90.0})
    if (v * v / (2 * lim.acc) <= 15 && (yawCur.rateMax <= 0 || v <= yawCur.rateMax)) lim.rateMax = v;
  lim.why.push_back("a " + std::to_string(static_cast<int>(lim.rateMax)) + " deg/s turn stops within " +
                    std::to_string(static_cast<int>(std::round(lim.rateMax * lim.rateMax / (2 * lim.acc)))) + " deg at " +
                    std::to_string(static_cast<int>(lim.acc)) + " deg/s/s");
  lim.pltRate = lim.rateMax;
  lim.imax = std::clamp(std::ceil((std::fabs(my.trimMax) + 0.2) * 10) / 10, std::max(0.1, yawCur.IMAX > 0 ? std::min(yawCur.IMAX, 1.0) : 0.5), 1.0);
  lim.why.push_back("IMAX " + num(lim.imax, 2) + " holds the trim " + num(std::fabs(my.trimMax), 2) + " with 0.2 to spare");
  // the rate-loop bandwidth the slowest motors allow (1 / (3 (tau + T)) on the strong, slow corner) caps the rate P
  double pMax = 6;
  for (const auto& p : my.set) pMax = std::min(pMax, 1 / (3 * (p.tau + p.T) * p.a));
  lim.why.push_back("rate P at most " + num(pMax, 1) + ": the bandwidth the slowest motor response allows");
  std::set<double> Ps;
  for (double k : {0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0}) {
    const double P = std::max(0.1, roundTo(k / n.a, 0.1));
    if (P <= pMax + 1e-9) Ps.insert(P);
  }
  if (Ps.empty()) Ps.insert(std::max(0.1, roundTo(pMax, 0.1)));
  std::vector<Candidate> ok, all;
  for (double P : Ps)
    for (double ir : {0.05, 0.075, 0.1})
      for (double K : {1.0, 1.25, 1.5, 2.0, 2.5, 3.0}) {
        cc::Gains g = yawCur;
        g.P = P;
        g.I = r3(P * ir);
        g.angleP = K;
        g.IMAX = lim.imax;
        g.accMax = lim.acc;
        g.rateMax = lim.rateMax;
        g.pltRate = lim.pltRate;
        ++dy.candidates;
        Candidate c;
        c.g = g;
        c.wm = cc::worstMargins(g, my.set);
        // a slow axis is judged as the hand tune judged it: the slow pair well damped (zeta >= 0.35) on every plant of
        // the set, then the time simulations. Its loop phase margin (dominated by the angle loop and the mixer limit) is
        // reported, not used as a gate: the flown and confirmed Step 2 (P 2.5, angle P 1) has a low one.
        double zMin = 1;
        bool stable = true;
        for (const auto& p : my.set) {
          const cc::Poles po = cc::slowAxisPoles(p.a, p.c, g.P, g.I, g.angleP);
          zMin = std::min(zMin, po.zeta);
          stable = stable && po.stable;
        }
        const double zNom = cc::slowAxisPoles(n.a, n.c, g.P, g.I, g.angleP).zeta;
        c.metrics.set("zeta", zMin);
        // zeta >= 0.35 on the nominal plant, >= 0.2 on the pessimistic corners (no aerodynamic damping, weak)
        if (stable && zNom >= 0.35 && zMin >= 0.2 && c.wm.gmDb >= 6) ok.push_back(c);
        all.push_back(c);
      }
  dy.accepted = static_cast<int>(ok.size());
  if (ok.empty()) {
    // keep the most robust ones and say so
    std::sort(all.begin(), all.end(), [](const Candidate& a, const Candidate& b) { return a.wm.pmDeg > b.wm.pmDeg; });
    all.resize(std::min<std::size_t>(all.size(), 20));
    ok = all;
    out.notes.push_back("yaw: no gains reach every margin target on every plant; the most robust ones were ranked");
  }
  double best = 1e18;
  for (auto& c : ok) {
    double worst = 0;
    YawMetrics wn;
    for (std::size_t i = 0; i < my.set.size(); ++i) {
      const YawMetrics m = yawMetrics(c.g, my.set[i]);
      if (i == 0) wn = m;
      worst = std::max(worst, m.score);
    }
    c.score = worst;
    if (std::getenv("APTUNER_DEBUG"))
      std::fprintf(stderr, "yaw P %.2f I %.3f K %.2f pm %.0f gm %.1f ms %.2f zeta %.2f score %.1f\n", c.g.P, c.g.I, c.g.angleP, c.wm.pmDeg, c.wm.gmDb, c.wm.ms,
                   c.metrics["zeta"].numOr(0), worst);
    const double z = c.metrics["zeta"].numOr(0);
    c.metrics = Json::obj({{"takeoffSwingDeg", r2(wn.takeoff)}, {"oscillationDeg", r2(wn.osc2)}, {"gustDeg", r2(wn.gust)},
                           {"turnOvershootDeg", r2(std::max(wn.ovR, wn.ovL))}, {"limitedPct", r2(100 * wn.lim)}, {"worstScore", r2(worst)},
                           {"zetaWorst", r2(z)}});
    best = std::min(best, worst);
  }
  // within 10% of the best score, the best-damped one (then the lower rate P)
  const Candidate* pick = nullptr;
  for (const auto& c : ok) {
    if (c.score > 1.1 * best) continue;
    const double z = c.metrics["zetaWorst"].numOr(0), zp = pick ? pick->metrics["zetaWorst"].numOr(0) : -1;
    if (!pick || z > zp + 0.01 || (std::fabs(z - zp) <= 0.01 && c.g.P < pick->g.P)) pick = &c;
  }
  dy.g = pick->g;
  dy.wm = pick->wm;
  dy.metrics = pick->metrics;
  dy.ok = true;
  dy.lim = lim;
  // stress tests of the winner (stages.js): +50 ms delay, rotor kick x2 / x4, gyro noise
  Json st = Json::array();
  struct S {
    const char* name;
    double delay, lead, noise;
  };
  for (const S& s : {S{"+50 ms delay", 0.05, 0, 0}, S{"rotor kick x2", 0, 0.2, 0}, S{"delay + kick x2 + gyro noise 2 deg/s", 0.05, 0.2, 2},
                     S{"rotor kick x4 + delay", 0.05, 0.6, 0}}) {
    double pk = 0, late = 0;
    for (std::size_t i = 0; i < my.set.size(); ++i) {
      cc::SimInput sin;
      sin.seconds = 90;
      sin.extraDelay = s.delay;
      sin.lead = s.lead;
      sin.gyroNoiseDegs = s.noise;
      const cc::SimOutput so = cc::simulate(dy.g, my.set[i], sin);
      double mn = 1e9, mx = -1e9;
      for (std::size_t k = 0; k < so.t.size(); ++k) {
        if (so.t[k] < 30) pk = std::max(pk, std::fabs(so.att[k]));
        if (so.t[k] > 60) {
          mn = std::min(mn, so.att[k]);
          mx = std::max(mx, so.att[k]);
        }
      }
      late = std::max(late, mx - mn);
    }
    st.push(Json::obj({{"case", s.name}, {"swingDeg", std::round(pk)}, {"lateOscillationDeg", r2(late)}, {"ok", late < 3}}));
  }
  dy.stress = Json::obj({{"cases", st}});
  return out;
}

}  // namespace aptuner::engine::heavy
