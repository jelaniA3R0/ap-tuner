// Model-based tuner: identification (yaw windowed integral regression, roll / pitch marginal mode), the plant sets,
// and the validation of the model against the flights (closed-loop replay).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>

#include "heavy_internal.hpp"

namespace aptuner::engine::heavy {

namespace {

constexpr double kD2R = 0.017453292519943295;

double median(Vec v) {
  if (v.empty()) return std::nan("");
  std::nth_element(v.begin(), v.begin() + static_cast<long>(v.size() / 2), v.end());
  return v[v.size() / 2];
}

/** least squares by the normal equations (Gauss-Jordan with partial pivoting); empty when singular */
Vec solveNormal(std::vector<Vec> A, Vec b) {
  const std::size_t n = b.size();
  for (std::size_t c = 0; c < n; ++c) {
    std::size_t piv = c;
    for (std::size_t r = c + 1; r < n; ++r)
      if (std::fabs(A[r][c]) > std::fabs(A[piv][c])) piv = r;
    if (std::fabs(A[piv][c]) < 1e-14) return {};
    std::swap(A[c], A[piv]);
    std::swap(b[c], b[piv]);
    for (std::size_t r = 0; r < n; ++r) {
      if (r == c) continue;
      const double f = A[r][c] / A[c][c];
      for (std::size_t k = c; k < n; ++k) A[r][k] -= f * A[c][k];
      b[r] -= f * b[c];
    }
  }
  Vec x(n);
  for (std::size_t i = 0; i < n; ++i) x[i] = b[i] / A[i][i];
  return x;
}

/** one regression row: dr = a int(u) - c int(r) + d_k dt */
struct Row {
  double iu, ir, dt, y;
  std::size_t h;   // hover slot
};

struct HoverSeries {
  Vec t, r, u;   // s, rad/s, output
};

double avgAround(const HoverSeries& s, std::size_t i, std::size_t e) {
  const std::size_t a = i > e ? i - e : 0, b = std::min(s.r.size(), i + e + 1);
  double sum = 0;
  for (std::size_t k = a; k < b; ++k) sum += s.r[k];
  return b > a ? sum / static_cast<double>(b - a) : 0;
}

std::vector<Row> buildRows(const std::vector<HoverSeries>& hs, double tau) {
  std::vector<Row> rows;
  for (std::size_t h = 0; h < hs.size(); ++h) {
    const HoverSeries& s = hs[h];
    if (s.t.size() < 10) continue;
    // motor lag on the output
    Vec u(s.u.size());
    double y = s.u[0];
    for (std::size_t k = 0; k < s.u.size(); ++k) {
      const double dt = k ? s.t[k] - s.t[k - 1] : 0;
      if (tau > 0 && dt > 0) y += (1 - std::exp(-dt / tau)) * (s.u[k] - y);
      else y = s.u[k];
      u[k] = y;
    }
    const double fs = (static_cast<double>(s.t.size()) - 1) / (s.t.back() - s.t.front());
    const std::size_t e = std::max<std::size_t>(1, static_cast<std::size_t>(std::lround(0.05 * fs)));
    const double tStart = s.t.front() + 1.5, tStop = s.t.back() - 0.5;
    std::size_t i = 0;
    while (i < s.t.size() && s.t[i] < tStart) ++i;
    while (i < s.t.size()) {
      std::size_t j = i;
      double iu = 0, ir = 0;
      while (j + 1 < s.t.size() && s.t[j + 1] <= s.t[i] + 1.0) {
        const double dt = s.t[j + 1] - s.t[j];
        iu += u[j] * dt;
        ir += s.r[j] * dt;
        ++j;
      }
      if (s.t[j] > tStop || j == i) break;
      const double dur = s.t[j] - s.t[i];
      if (dur > 0.8) rows.push_back({iu, ir, dur, avgAround(s, j, e) - avgAround(s, i, e), h});
      i = j;
    }
  }
  return rows;
}

struct Fit {
  bool ok = false;
  double a = 0, c = 0, r2 = 0;
  Vec d;   // per hover slot
};

Fit fitRows(const std::vector<Row>& rows, std::size_t nh, std::size_t skip = static_cast<std::size_t>(-1)) {
  const std::size_t n = 2 + nh;
  std::vector<Vec> A(n, Vec(n, 0));
  Vec b(n, 0);
  std::size_t used = 0;
  std::vector<char> seen(nh, 0);
  for (const Row& r : rows) {
    if (r.h == skip) continue;
    Vec x(n, 0);
    x[0] = r.iu;
    x[1] = -r.ir;
    x[2 + r.h] = r.dt;
    seen[r.h] = 1;
    for (std::size_t i = 0; i < n; ++i) {
      b[i] += x[i] * r.y;
      for (std::size_t j = 0; j < n; ++j) A[i][j] += x[i] * x[j];
    }
    ++used;
  }
  // a hover left out has no rows: pin its trim to 0
  for (std::size_t h = 0; h < nh; ++h)
    if (!seen[h]) A[2 + h][2 + h] = 1;
  Fit f;
  if (used < 8) return f;
  const Vec th = solveNormal(A, b);
  if (th.empty()) return f;
  double ss = 0, st = 0;
  for (const Row& r : rows) {
    if (r.h == skip) continue;
    const double p = th[0] * r.iu - th[1] * r.ir + th[2 + r.h] * r.dt;
    ss += (r.y - p) * (r.y - p);
    st += r.y * r.y;
  }
  f.ok = true;
  f.a = th[0];
  f.c = th[1];
  f.d.assign(th.begin() + 2, th.end());
  f.r2 = st > 0 ? 1 - ss / st : 0;
  return f;
}

}  // namespace

// ------------------------------------------------------------------------------------------------- yaw

YawIdent identifyYaw(const std::vector<FlightRecord>& flights, const std::vector<Hover>& hovers, Family) {
  YawIdent id;
  std::vector<HoverSeries> hs;
  for (std::size_t hi = 0; hi < hovers.size(); ++hi) {
    const Hover& h = hovers[hi];
    const Window w = axisWindow(flights[h.flight].axis[2], h.tLift, h.t1);
    if (w.size() < 30 || !(w.fs >= 4)) continue;
    HoverSeries s;
    s.t = w.t;
    s.u = w.out;
    s.r.resize(w.act.size());
    for (std::size_t k = 0; k < w.act.size(); ++k) s.r[k] = w.act[k] * kD2R;
    hs.push_back(std::move(s));
    id.hoverIdx.push_back(hi);
  }
  if (hs.empty()) {
    id.reason = "no hover with yaw-rate data at 4 Hz or more";
    return id;
  }
  Fit best;
  double bestTau = 0;
  std::vector<Row> bestRows;
  for (double tau : {0.0, 0.1, 0.2, 0.3}) {
    const auto rows = buildRows(hs, tau);
    const Fit f = fitRows(rows, hs.size());
    if (f.ok && (!best.ok || f.r2 > best.r2)) {
      best = f;
      bestTau = tau;
      bestRows = rows;
    }
  }
  id.windows = static_cast<int>(bestRows.size());
  if (!best.ok) {
    id.reason = "too few 1-second windows to fit the yaw model";
    return id;
  }
  id.a = best.a;
  id.c = best.c;
  id.tau = bestTau;
  id.r2 = best.r2;
  for (double d : best.d) id.trims.push_back(best.a != 0 ? -d / best.a : 0);
  // the spread when one hover at a time is left out (jackknife), at least x0.8 .. x1.25
  id.aLo = 0.8 * id.a;
  id.aHi = 1.25 * id.a;
  id.cLo = std::max(0.0, 0.5 * id.c);
  id.cHi = id.c;
  if (hs.size() >= 3)
    for (std::size_t k = 0; k < hs.size(); ++k) {
      const Fit f = fitRows(bestRows, hs.size(), k);
      if (!f.ok || f.a <= 0) continue;
      id.aLo = std::min(id.aLo, f.a);
      id.aHi = std::max(id.aHi, f.a);
      id.cLo = std::min(id.cLo, std::max(0.0, f.c));
      id.cHi = std::max(id.cHi, f.c);
    }
  if (!(id.a > 0)) id.reason = "the fitted control power is not positive";
  else if (id.r2 < 0.3) id.reason = "the yaw model explains only " + std::to_string(static_cast<int>(id.r2 * 100)) + "% of the yaw-rate changes";
  else if (id.windows < 15) id.reason = "too few 1-second windows (" + std::to_string(id.windows) + ")";
  else id.ok = true;
  return id;
}

AxisModel yawModel(const YawIdent& id, double umax, bool umaxMeasured) {
  AxisModel m;
  m.method = "yaw regression";
  m.ok = id.ok;
  m.reason = id.reason;
  if (!id.ok) return m;
  m.umax = umax;
  m.umaxMeasured = umaxMeasured;
  m.trim = id.trims.empty() ? 0 : std::accumulate(id.trims.begin(), id.trims.end(), 0.0) / static_cast<double>(id.trims.size());
  m.trimMax = m.trim;
  for (double t : id.trims)
    if (std::fabs(t) > std::fabs(m.trimMax)) m.trimMax = t;
  const double sgn = m.trimMax < 0 ? -1 : 1;
  auto P = [&](const char* name, double a, double c, double tau, double trim) {
    cc::Plant p;
    p.name = name;
    p.a = a;
    p.c = c;
    p.tau = tau;
    p.T = 0.01;
    p.trim = trim;
    p.umax = umax;
    return p;
  };
  // nominal, the slow corner, and the corners the hand analysis used (grid2.js / stages.js)
  m.set = {P("nominal", id.a, id.c, 0.2, m.trim),
           P("weak and slow", id.aLo, 0, 0.3, m.trimMax),
           P("strong, slow motors", id.aHi, id.cHi, 0.4, m.trim),
           P("strong, fast motors", id.aHi, 0, 0.1, m.trim),
           P("large trim", id.a, id.cLo, 0.25, m.trimMax + sgn * 0.05)};
  return m;
}

// ------------------------------------------------------------------------------------------------- roll / pitch

RpIdent identifyMarginal(const Oscillation& osc, const std::vector<Hover>& hovers, Family fam, int axis) {
  RpIdent id;
  id.method = "oscillation";
  if (!osc.found) {
    id.reason = "no persistent low-frequency oscillation to measure the vehicle from";
    return id;
  }
  // the cluster's centre and every distinct window frequency in it: the spread of the frequency is the spread of a
  Vec freqs{osc.fHz};
  for (double f : osc.freqs) {
    const double r = std::round(f / 0.02) * 0.02;
    if (std::none_of(freqs.begin(), freqs.end(), [&](double x) { return std::fabs(x - r) < 0.01; })) freqs.push_back(r);
  }
  Vec as, taus, centre;
  for (std::size_t hi : osc.hovers) {
    const cc::Gains g = gainsFromParams(hovers[hi].params, fam, axis);
    for (std::size_t k = 0; k < freqs.size(); ++k)
      for (double T : {0.01, 0.025}) {
        const auto f = cc::marginalModeFit(g, 2 * kPi * freqs[k], T);
        if (!f || !(f->a > 0) || f->tau > 0.5) continue;
        as.push_back(f->a);
        if (k == 0) {
          id.fits.push_back(*f);
          centre.push_back(f->a);
          taus.push_back(f->tau);
        }
      }
  }
  if (!centre.empty()) {
    id.aLo = *std::min_element(as.begin(), as.end());
    id.aHi = *std::max_element(as.begin(), as.end());
    as = centre;
  }
  // a second wobble frequency: the loop could be the one at either; the range covers both
  if (osc.f2Hz > 0) {
    Vec a2;
    for (std::size_t hi : osc.hovers)
      for (double T : {0.01, 0.025}) {
        const auto f = cc::marginalModeFit(gainsFromParams(hovers[hi].params, fam, axis), 2 * kPi * osc.f2Hz, T);
        if (f && f->a > 0 && f->tau <= 0.5) a2.push_back(f->a);
      }
    if (!a2.empty()) {
      id.a2 = median(a2);
      id.aLo = std::min(id.aLo > 0 ? id.aLo : id.a2, *std::min_element(a2.begin(), a2.end()));
      id.aHi = std::max(id.aHi, *std::max_element(a2.begin(), a2.end()));
    }
  }
  if (id.fits.empty()) {
    id.reason = "the oscillation does not fit the model form (phase lag outside 0-90 deg beyond the integrator)";
    return id;
  }
  id.a = median(as);
  id.tauLo = *std::min_element(taus.begin(), taus.end());
  id.tauHi = *std::max_element(taus.begin(), taus.end());
  id.ok = true;
  return id;
}

AxisModel rpModel(const RpIdent& id, double trim, double umax, bool umaxMeasured) {
  AxisModel m;
  m.method = id.method == "plan" ? "saved model" : "marginal mode";
  m.ok = id.ok;
  m.reason = id.reason;
  if (!id.ok) return m;
  m.trim = trim;
  m.trimMax = trim;
  m.umax = umax;
  m.umaxMeasured = umaxMeasured;
  const double tLo = std::max(0.02, id.tauLo - 0.01), tHi = id.tauHi + 0.02, tMid = 0.5 * (tLo + tHi);
  auto P = [&](const std::string& name, double a, double tau, double T) {
    cc::Plant p;
    p.name = name;
    p.a = a;
    p.tau = tau;
    p.T = T;
    p.trim = trim;
    p.umax = umax;
    return p;
  };
  // control power: at least x0.82 .. x1.2, wider when the oscillation frequency wandered between windows
  const double aLo = id.aLo > 0 ? std::min(0.82 * id.a, id.aLo) : 0.82 * id.a, aHi = id.aHi > 0 ? std::max(1.2 * id.a, id.aHi) : 1.2 * id.a;
  m.set.push_back(P("nominal", id.a, tMid, 0.0175));
  m.set.push_back(P("weak and slow", aLo, tHi, 0.025));
  Vec as{aLo, id.a, aHi};
  if (aHi > 1.6 * id.a) as.push_back(std::sqrt(id.a * aHi));   // a wide range: one more point in between
  for (double a : as)
    for (double tau : {tLo, tMid, tHi})
      for (double T : {0.01, 0.025}) m.set.push_back(P("corner", a, tau, T));
  return m;
}

Json AxisModel::toJson() const {
  Json s = Json::array();
  for (const auto& p : set)
    s.push(Json::obj({{"name", p.name}, {"a", p.a}, {"c", p.c}, {"tau", p.tau}, {"T", p.T}, {"trim", p.trim}, {"umax", p.umax}}));
  return Json::obj({{"ok", ok}, {"method", method}, {"reason", reason}, {"trim", trim}, {"trimMax", trimMax}, {"umax", umax},
                    {"umaxMeasured", umaxMeasured}, {"set", s}});
}

AxisModel AxisModel::fromJson(const Json& j) {
  AxisModel m;
  m.ok = j["ok"].isBool() && j["ok"].asBool();
  m.method = j["method"].strOr("");
  m.reason = j["reason"].strOr("");
  m.trim = j["trim"].numOr(0);
  m.trimMax = j["trimMax"].numOr(m.trim);
  m.umax = j["umax"].numOr(0.5);
  m.umaxMeasured = j["umaxMeasured"].isBool() && j["umaxMeasured"].asBool();
  if (j["set"].isArray())
    for (const auto& x : j["set"].arr()) {
      cc::Plant p;
      p.name = x["name"].strOr("");
      p.a = x["a"].numOr(1);
      p.c = x["c"].numOr(0);
      p.tau = x["tau"].numOr(0.05);
      p.T = x["T"].numOr(0.01);
      p.trim = x["trim"].numOr(0);
      p.umax = x["umax"].numOr(0.5);
      m.set.push_back(p);
    }
  if (m.set.empty()) m.ok = false;
  return m;
}

// ------------------------------------------------------------------------------------------------- validation

double logYawSwing(const FlightRecord& f, const Hover& h) {
  const Series& hd = f.axis[2].ang;
  if (hd.empty()) return std::nan("");
  const double ts = h.tLift - 0.5, h0 = hd.at(ts);
  double mx = 0;
  for (std::size_t i = hd.indexAt(ts); i < hd.size() && hd.t[i] <= std::min(h.t1, ts + 40); ++i)
    if (hd.t[i] >= ts) mx = std::max(mx, std::fabs(cc::wrap180(hd.v[i] - h0)));
  return mx;
}

YawReplay replayYaw(const FlightRecord& f, const Hover& h, std::size_t hoverIdx, const cc::Gains& g, const cc::Plant& p) {
  YawReplay r;
  r.hover = hoverIdx;
  const double ts = h.tLift - 0.5;
  cc::SimInput in;
  in.seconds = std::max(1.0, h.t1 - ts);
  in.recordHz = 10;
  if (!f.stickYaw.empty()) in.stickPwm = [&](double t) { return f.stickYaw.at(ts + t); };
  const cc::SimOutput s = cc::simulate(g, p, in);
  const Series& hd = f.axis[2].ang;
  // the logged output at 1 Hz and below: what the model represents (motor vibration rides on top of it)
  Series out;
  {
    const Series& raw = f.axis[2].out;
    double y = raw.empty() ? 0 : raw.at(ts);
    for (std::size_t i = raw.indexAt(ts); i < raw.size() && raw.t[i] <= h.t1 + 1; ++i) {
      const double dt = i ? raw.t[i] - raw.t[i - 1] : 0;
      y += (1 - std::exp(-2 * kPi * 1.0 * std::max(0.0, dt))) * (raw.v[i] - y);
      out.push(raw.t[i], y);
    }
  }
  const double h0 = hd.at(ts);
  double eH = 0, eU = 0;
  std::size_t n = 0;
  Json ct = Json::array(), cl = Json::array(), cm = Json::array();
  for (std::size_t k = 0; k < s.t.size(); ++k) {
    const double t = s.t[k];
    const double logH = cc::wrap180(hd.at(ts + t) - h0), simH = s.att[k];
    r.swingLog = std::max(r.swingLog, std::fabs(logH));
    r.swingModel = std::max(r.swingModel, std::fabs(simH));
    eH += (logH - simH) * (logH - simH);
    const double lu = out.at(ts + t);
    if (std::isfinite(lu)) eU += (lu - s.u[k]) * (lu - s.u[k]);
    ++n;
    if (std::getenv("APTUNER_DEBUG") && k % 30 == 0)
      std::fprintf(stderr, "replay yaw t %.0f out log %.3f model %.3f heading log %.1f model %.1f\n", t, lu, s.u[k], logH, simH);
    if (k % 5 == 0) {
      ct.push(std::round(t * 10) / 10);
      cl.push(std::round(logH * 10) / 10);
      cm.push(std::round(simH * 10) / 10);
    }
  }
  r.headingRmsDeg = n ? std::sqrt(eH / static_cast<double>(n)) : 0;
  r.outRms = n ? std::sqrt(eU / static_cast<double>(n)) : 0;
  // the output check allows for gusts the model does not contain, which a higher rate P passes on to the output
  r.pass = std::fabs(r.swingLog - r.swingModel) <= std::max(5.0, 0.12 * r.swingLog) && r.headingRmsDeg <= 30 && r.outRms <= 0.07 + 0.03 * g.P;
  r.chart = Json::obj({{"t", ct}, {"log", cl}, {"model", cm}});
  return r;
}

GustResult gustResponse(const cc::Gains& g, const cc::Plant& p, double seconds) {
  const cc::GustSequence gust(seconds + 1);
  cc::SimInput in;
  in.seconds = seconds;
  in.shaped = true;
  in.angleDeg = [](double) { return 0.0; };
  // the gust torque scaled to this vehicle as the hand analysis did (gust x a / 5.8 in rad/s/s = gust / 5.8 output units)
  in.dist = [&](double t) { return -gust.at(t); };
  in.recordHz = 50;
  cc::Plant q = p;
  q.trim = 0;
  const cc::SimOutput s = cc::simulate(g, q, in);
  GustResult r;
  Vec late;
  for (std::size_t k = 0; k < s.t.size(); ++k)
    if (s.t[k] > seconds / 3) late.push_back(s.att[k]);
  if (late.size() < 10) return r;
  const double mu = std::accumulate(late.begin(), late.end(), 0.0) / static_cast<double>(late.size());
  double ss = 0;
  int zc = 0;
  for (std::size_t k = 0; k < late.size(); ++k) {
    ss += (late[k] - mu) * (late[k] - mu);
    if (k && ((late[k] - mu > 0) != (late[k - 1] - mu > 0))) ++zc;
  }
  r.rmsDeg = std::sqrt(ss / static_cast<double>(late.size()));
  r.ampDeg = (*std::max_element(late.begin(), late.end()) - *std::min_element(late.begin(), late.end())) / 2;
  r.fHz = zc / 2.0 / (seconds * 2 / 3);
  r.limited = s.limitedFrac;
  return r;
}

RpCheck checkRpOscillation(const FlightRecord& f, const Hover& h, const cc::Gains& g, const AxisModel& m, const Oscillation& osc) {
  RpCheck c;
  c.fLogHz = osc.fHz;
  c.logAmp = osc.angleAmpDeg;
  const cc::WorstMargins wm = cc::worstMargins(g, m.set);
  c.pmWorst = wm.pmDeg;
  const cc::Margins nm = cc::margins(g, m.set[0]);
  c.fModelHz = nm.wcRad / (2 * kPi);
  // the gust-driven simulation over the motor-lag range at the nominal control power
  double fSum = 0;
  int nF = 0;
  c.simAmpLo = 1e9;
  for (std::size_t i = 0; i < m.set.size(); ++i) {
    const cc::Plant& p = m.set[i];
    if (std::fabs(p.a - m.set[0].a) > 1e-9 || std::fabs(p.T - 0.025) > 1e-9) continue;
    const GustResult gr = gustResponse(g, p);
    c.simAmpLo = std::min(c.simAmpLo, gr.ampDeg);
    c.simAmpHi = std::max(c.simAmpHi, gr.ampDeg);
    if (gr.fHz > 0.1) {
      fSum += gr.fHz;
      ++nF;
    }
  }
  if (c.simAmpLo > 1e8) c.simAmpLo = 0;
  const double fSim = nF ? fSum / nF : c.fModelHz;
  const bool fOk = osc.fHz > 0 && std::fabs(fSim - osc.fHz) <= 0.15 * osc.fHz;
  const bool pmOk = c.pmWorst >= -20 && c.pmWorst <= 10;
  const bool regime = c.simAmpHi >= 0.5 * c.logAmp;   // the model can be as lively as the flight
  c.ok = fOk && pmOk && regime;
  char buf[400];
  std::snprintf(buf, sizeof buf,
                "with the flown gains the model oscillates at %.2f Hz (flight: %.2f Hz) and its worst phase margin is %.0f deg; "
                "gusts drive it to +-%.1f..%.1f deg (flight: +-%.1f deg)",
                fSim, osc.fHz, c.pmWorst, c.simAmpLo, c.simAmpHi, c.logAmp);
  c.text = buf;
  (void)f;
  (void)h;
  return c;
}

}  // namespace aptuner::engine::heavy
