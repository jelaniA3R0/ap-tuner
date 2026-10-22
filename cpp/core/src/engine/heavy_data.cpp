// Model-based tuner: parameter names, hover detection, axis windows, spectra and oscillation detection.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>

#include "aptuner/engine/loganalysis.hpp"
#include "heavy_internal.hpp"

namespace aptuner::engine::heavy {

namespace {

const char* rateCode(int axis) { return axis == 0 ? "RLL" : axis == 1 ? "PIT" : "YAW"; }
const char* oneCode(int axis) { return axis == 0 ? "R" : axis == 1 ? "P" : "Y"; }

double median(Vec v) {
  if (v.empty()) return std::nan("");
  std::nth_element(v.begin(), v.begin() + static_cast<long>(v.size() / 2), v.end());
  return v[v.size() / 2];
}

}  // namespace

std::string Hover::id() const { return "flight " + std::to_string(flight + 1) + " hover " + std::to_string(index); }

// ------------------------------------------------------------------------------------------------- parameters

std::string paramName(Family f, int axis, const std::string& key) {
  const bool q = f == Family::Vtol;
  const std::string A = q ? "Q_A_" : "ATC_";
  if (key == "P" || key == "I" || key == "D" || key == "FF" || key == "IMAX" || key == "FLTT" || key == "FLTE" || key == "FLTD")
    return A + "RAT_" + rateCode(axis) + "_" + key;
  if (key == "ANG") return A + "ANG_" + rateCode(axis) + "_P";
  if (key == "ACC") return A + "ACC_" + oneCode(axis) + "_MAX";
  if (key == "ACC_OLD") return A + "ACCEL_" + oneCode(axis) + "_MAX";
  if (key == "RMAX") return A + "RATE_" + oneCode(axis) + "_MAX";
  if (key == "INPUT_TC") return A + "INPUT_TC";
  if (key == "WPY") return A + "RATE_WPY_MAX";
  if (key == "GYRO") return "INS_GYRO_FILTER";
  if (key == "LOOP") return "SCHED_LOOP_RATE";
  if (key == "PLT_RATE") return q ? "Q_PLT_Y_RATE" : "PILOT_Y_RATE";
  if (key == "PLT_EXPO") return q ? "Q_PLT_Y_EXPO" : "PILOT_Y_EXPO";
  if (key == "PLT_TC") return q ? "Q_PLT_Y_RATE_TC" : "PILOT_Y_RATE_TC";
  if (key == "HOVER") return q ? "Q_M_THST_HOVER" : "MOT_THST_HOVER";
  if (key == "HOVER_LEARN") return q ? "Q_M_HOVER_LEARN" : "MOT_HOVER_LEARN";
  return "";
}

cc::Gains gainsFromParams(const log::Params& p, Family f, int axis) {
  cc::Gains g;
  g.kind = axis == 2 ? cc::Kind::Yaw : cc::Kind::RollPitch;
  auto v = [&](const std::string& key, double d) { return p.getOr(paramName(f, axis, key), d); };
  g.P = v("P", 0);
  g.I = v("I", 0);
  g.D = v("D", 0);
  g.FF = v("FF", 0);
  g.IMAX = v("IMAX", 0.5);
  g.FLTT = v("FLTT", 20);
  g.FLTE = v("FLTE", axis == 2 ? 2.5 : 0);
  g.FLTD = v("FLTD", 20);
  g.angleP = v("ANG", 4.5);
  if (p.has(paramName(f, axis, "ACC"))) g.accMax = v("ACC", 0);
  else g.accMax = p.getOr(paramName(f, axis, "ACC_OLD"), 0) / 100;   // older firmware: centi-degrees
  g.rateMax = v("RMAX", 0);
  g.inputTC = v("INPUT_TC", 0.15);
  g.gyroHz = v("GYRO", 20);
  g.loopHz = p.getOr("SCHED_LOOP_RATE", f == Family::Vtol ? 300 : 400);
  if (!(g.loopHz > 0)) g.loopHz = 400;
  g.pltRate = v("PLT_RATE", f == Family::Vtol ? 90 : 202.5);
  g.pltExpo = v("PLT_EXPO", 0);
  g.pltRateTC = v("PLT_TC", 0);
  const int ch = static_cast<int>(p.getOr("RCMAP_YAW", 4));
  const std::string rc = "RC" + std::to_string(ch) + "_";
  g.rcMin = p.getOr(rc + "MIN", 1100);
  g.rcTrim = p.getOr(rc + "TRIM", 1500);
  g.rcMax = p.getOr(rc + "MAX", 1900);
  g.rcDz = p.getOr(rc + "DZ", 20);
  return g;
}

double paramValueFor(const log::Params& p, Family f, int axis, const std::string& key, double value) {
  if (key == "ACC" && !p.has(paramName(f, axis, "ACC")) && p.has(paramName(f, axis, "ACC_OLD"))) return value * 100;
  return value;
}

// ------------------------------------------------------------------------------------------------- hovers

std::vector<Hover> detectHovers(const std::vector<FlightRecord>& flights, std::vector<std::string>& notes) {
  std::vector<Hover> out;
  for (std::size_t fi = 0; fi < flights.size(); ++fi) {
    const FlightRecord& r = flights[fi];
    const AxisRec& yaw = r.axis[2];
    // the time base: the PID log (25 Hz, with RESET flags) or the throttle samples
    const Series& base = !yaw.flags.empty() ? yaw.flags : !r.throttle.empty() ? r.throttle : yaw.actual;
    if (base.size() < 20 || r.throttle.empty()) {
      notes.push_back(r.name + ": no throttle or controller data, no hovers found");
      continue;
    }
    auto live = [&](double t) {
      if (!yaw.flags.empty() && (static_cast<int>(yaw.flags.at(t)) & 4)) return false;   // RESET: on the ground
      if (!r.armed.empty() && r.armed.at(t) < 0.5) return false;
      if (r.family == Family::Vtol && !r.mode.empty() && !isQMode(r.mode.at(t))) return false;
      return true;
    };
    Vec thr;
    for (std::size_t i = 0; i < base.size(); ++i)
      if (live(base.t[i])) {
        const double x = r.throttle.at(base.t[i]);
        if (x > 0.15) thr.push_back(x);
      }
    if (thr.size() < 20) {
      notes.push_back(r.name + ": the controller was never flying");
      continue;
    }
    const double lim = std::max(0.15, 0.6 * median(thr));
    // spans where the controller flies (gaps under 1 s bridged), at least 8 s
    std::vector<std::pair<double, double>> spans;
    double a = -1, last = -1;
    for (std::size_t i = 0; i <= base.size(); ++i) {
      const bool ok = i < base.size() && live(base.t[i]) && r.throttle.at(base.t[i]) > lim;
      const double t = i < base.size() ? base.t[i] : 1e18;
      if (ok && a < 0) a = t;
      if (ok) {
        if (last >= 0 && t - last > 1 && a < last) {
          spans.push_back({a, last});
          a = t;
        }
        last = t;
      }
      if ((!ok || i == base.size()) && a >= 0 && last >= a) {
        spans.push_back({a, last});
        a = -1;
      }
    }
    // never across a tuning-parameter change
    std::vector<std::pair<double, double>> pieces;
    for (auto [s0, s1] : spans) {
      double c0 = s0;
      for (const auto& c : r.changes)
        if (isTuningParam(c.name) && c.t > c0 && c.t < s1) {
          pieces.push_back({c0, c.t});
          c0 = c.t;
        }
      pieces.push_back({c0, s1});
    }
    int idx = 0;
    for (auto [s0, s1] : pieces) {
      if (s1 - s0 < 8) continue;
      Hover h;
      h.flight = fi;
      h.index = ++idx;
      h.t0 = s0;
      h.t1 = s1;
      // lift-off: the throttle reaches 90% of this hover's median (thrust carries the weight; before that the ground
      // holds the vehicle and no torque imbalance can turn it)
      Vec ht;
      for (std::size_t i = r.throttle.indexAt(s0); i < r.throttle.size() && r.throttle.t[i] <= s1; ++i)
        if (r.throttle.t[i] >= s0) ht.push_back(r.throttle.v[i]);
      const double hoverThr = median(ht);
      h.tLift = s0;
      for (std::size_t i = r.throttle.indexAt(s0 - 6); i < r.throttle.size() && r.throttle.t[i] < s1; ++i) {
        const double t = r.throttle.t[i];
        if (t < s0 - 6 || !live(t)) continue;
        if (r.throttle.v[i] >= 0.9 * hoverThr) {
          h.tLift = t;
          break;
        }
      }
      h.params = r.paramsAt(h.t0 + 0.01);
      double sum = 0;
      int n = 0;
      for (std::size_t i = r.throttle.indexAt(h.tLift + 3); i < r.throttle.size() && r.throttle.t[i] < h.t1; ++i) {
        sum += r.throttle.v[i];
        ++n;
      }
      h.meanThrottle = n ? sum / n : 0;
      h.modeNum = r.mode.empty() ? -1 : r.mode.at(h.tLift + 1);
      out.push_back(h);
    }
    if (!idx) notes.push_back(r.name + ": no hover of 8 s or more");
  }
  return out;
}

Window axisWindow(const AxisRec& a, double ta, double tb) {
  Window w;
  if (a.actual.empty()) return w;
  const bool same = a.target.size() == a.actual.size() && a.out.size() == a.actual.size();
  for (std::size_t i = a.actual.indexAt(ta); i < a.actual.size(); ++i) {
    const double t = a.actual.t[i];
    if (t < ta) continue;
    if (t > tb) break;
    w.t.push_back(t);
    w.act.push_back(a.actual.v[i]);
    w.tgt.push_back(same ? a.target.v[i] : a.target.at(t));
    w.out.push_back(same ? a.out.v[i] : a.out.at(t));
  }
  if (w.t.size() > 2) w.fs = (static_cast<double>(w.t.size()) - 1) / (w.t.back() - w.t.front());
  return w;
}

// ------------------------------------------------------------------------------------------------- spectra

Spectrum dftGrid(const Vec& x, double fs, double fmin, double fmax, double df) {
  Spectrum s;
  const std::size_t N = x.size();
  if (N < 8 || !(fs > 0)) return s;
  const double mu = std::accumulate(x.begin(), x.end(), 0.0) / static_cast<double>(N);
  Vec w(N);
  for (std::size_t i = 0; i < N; ++i) w[i] = (x[i] - mu) * (0.5 - 0.5 * std::cos(2 * kPi * static_cast<double>(i) / static_cast<double>(N - 1)));
  for (double f = fmin; f <= fmax + 1e-9; f += df) {
    // rotate a phasor instead of calling cos / sin per sample
    const double ph = 2 * kPi * f / fs;
    const cc::Cx step(std::cos(ph), -std::sin(ph));
    cc::Cx z(1, 0), acc(0, 0);
    for (std::size_t i = 0; i < N; ++i) {
      acc += w[i] * z;
      z *= step;
      if ((i & 255) == 255) z /= std::abs(z);
    }
    s.f.push_back(f);
    s.X.push_back(acc);
    s.amp.push_back(2 * std::abs(acc) / (static_cast<double>(N) * 0.5));
  }
  return s;
}

Vec decimate(const Vec& x, double fs, double targetHz, double& fsOut) {
  const std::size_t k = std::max<std::size_t>(1, static_cast<std::size_t>(std::floor(fs / targetHz)));
  fsOut = fs / static_cast<double>(k);
  Vec y;
  y.reserve(x.size() / k + 1);
  for (std::size_t i = 0; i + k <= x.size(); i += k) {
    double s = 0;
    for (std::size_t j = 0; j < k; ++j) s += x[i + j];
    y.push_back(s / static_cast<double>(k));
  }
  return y;
}

double highPassRms(const Vec& x, double fs, double fc) {
  if (x.size() < 10 || !(fs > 2 * fc)) return std::nan("");
  const double al = 1 - std::exp(-2 * kPi * fc / fs);
  double s = x[0], sum = 0;
  for (double v : x) {
    s += al * (v - s);
    sum += (v - s) * (v - s);
  }
  return std::sqrt(sum / static_cast<double>(x.size()));
}

Oscillation findOscillation(const std::vector<FlightRecord>& flights, const std::vector<Hover>& hovers, int axis) {
  Oscillation o;
  struct Win {
    double f, amp, prom;
    cc::Cx T, A;
    double iAmp;
    std::size_t hover;
    double t0;
  };
  std::vector<Win> wins;
  std::vector<std::size_t> perHoverCount(hovers.size(), 0);
  for (std::size_t hi = 0; hi < hovers.size(); ++hi) {
    const Hover& h = hovers[hi];
    const AxisRec& ax = flights[h.flight].axis[static_cast<std::size_t>(axis)];
    const Window w = axisWindow(ax, h.tLift + 3, h.t1 - 0.5);
    if (w.size() < 20 || !(w.fs > 6)) continue;
    double fsD = 0;
    const Vec act = decimate(w.act, w.fs, 50, fsD), tgt = decimate(w.tgt, w.fs, 50, fsD);
    const double dur = static_cast<double>(act.size()) / fsD;
    const double W = dur < 10 ? dur : 10, step = 5;
    if (W < 6) continue;
    for (double s0 = 0; s0 + W <= dur + 1e-9; s0 += step) {
      const auto i0 = static_cast<std::size_t>(s0 * fsD), n = static_cast<std::size_t>(W * fsD);
      if (i0 + n > act.size()) break;
      const Vec a(act.begin() + static_cast<long>(i0), act.begin() + static_cast<long>(i0 + n));
      const Spectrum sp = dftGrid(a, fsD, 0.2, 3.0, 0.02);
      if (sp.f.size() < 5) continue;
      std::size_t k = 0;
      for (std::size_t j = 1; j + 1 < sp.f.size(); ++j)
        if (sp.amp[j] > sp.amp[k] && sp.amp[j] >= sp.amp[j - 1] && sp.amp[j] >= sp.amp[j + 1]) k = j;
      if (k == 0 || k + 1 >= sp.f.size()) continue;
      const double med = median(sp.amp);
      Win wi;
      wi.f = sp.f[k];
      wi.amp = sp.amp[k];
      wi.prom = med > 0 ? sp.amp[k] / med : 0;
      wi.A = sp.X[k];
      const Vec tg(tgt.begin() + static_cast<long>(i0), tgt.begin() + static_cast<long>(i0 + n));
      const Spectrum st = dftGrid(tg, fsD, sp.f[k], sp.f[k], 0.02);
      wi.T = st.X.empty() ? cc::Cx{} : st.X[0];
      // I-term swing at the peak (PID log rate)
      wi.iAmp = 0;
      if (!ax.I.empty()) {
        Vec iv;
        const double ta = w.t.front() + s0, tb = ta + W;
        for (std::size_t j = ax.I.indexAt(ta); j < ax.I.size() && ax.I.t[j] <= tb; ++j)
          if (ax.I.t[j] >= ta) iv.push_back(ax.I.v[j]);
        const double fsI = iv.size() > 2 ? static_cast<double>(iv.size()) / W : 0;
        if (fsI > 2 * sp.f[k] + 1) {
          const Spectrum si = dftGrid(iv, fsI, sp.f[k], sp.f[k], 0.02);
          if (!si.amp.empty()) wi.iAmp = si.amp[0];
        }
      }
      wi.hover = hi;
      wi.t0 = s0;
      wins.push_back(wi);
    }
  }
  o.windows = static_cast<int>(wins.size());
  if (std::getenv("APTUNER_DEBUG"))
    for (const auto& w : wins)
      std::fprintf(stderr, "osc axis %d hover %zu t0 %.0f: f %.2f amp %.2f prom %.1f\n", axis, w.hover, w.t0, w.f, w.amp, w.prom);
  if (wins.size() < 2) return o;
  Vec fs;
  for (const auto& w : wins)
    if (w.prom >= 3) fs.push_back(w.f);
  if (fs.size() < 2) return o;
  // the largest cluster of peak frequencies (+-15%): a second, weaker mode does not pull the frequency away
  double fc = 0;
  std::size_t bestN = 0;
  for (double c : fs) {
    std::size_t n = 0;
    for (double f : fs)
      if (std::fabs(f - c) <= 0.15 * c) ++n;
    if (n > bestN || (n == bestN && c < fc)) {
      bestN = n;
      fc = c;
    }
  }
  Vec cl;
  for (double f : fs)
    if (std::fabs(f - fc) <= 0.15 * fc) cl.push_back(f);
  const double f0 = median(cl);
  std::vector<const Win*> same;
  for (const auto& w : wins)
    if (w.prom >= 3 && std::fabs(w.f - f0) <= 0.15 * f0) same.push_back(&w);
  o.persistence = static_cast<double>(same.size()) / static_cast<double>(wins.size());
  if (same.size() < 2 || o.persistence < 0.4) return o;
  o.intermittent = o.persistence < 0.6;
  // a second cluster (another mode, or the same loop wandering): kept so the identification can widen its range
  {
    double f2 = 0;
    std::size_t n2 = 0;
    for (double c : fs) {
      if (std::fabs(c - f0) <= 0.15 * f0) continue;
      std::size_t n = 0;
      for (double f : fs)
        if (std::fabs(f - c) <= 0.15 * c && std::fabs(f - f0) > 0.15 * f0) ++n;
      if (n > n2) {
        n2 = n;
        f2 = c;
      }
    }
    if (n2 >= 2 && static_cast<double>(n2) >= 0.25 * static_cast<double>(wins.size())) {
      Vec c2;
      for (double f : fs)
        if (std::fabs(f - f2) <= 0.15 * f2 && std::fabs(f - f0) > 0.15 * f0) c2.push_back(f);
      o.f2Hz = median(c2);
      o.persistence2 = static_cast<double>(n2) / static_cast<double>(wins.size());
    }
  }
  cc::Cx Sta(0, 0);
  double Stt = 0, Saa = 0, amp = 0, prom = 0, iAmp = 0;
  for (const Win* w : same) {
    Sta += w->T * std::conj(w->A);
    Stt += std::norm(w->T);
    Saa += std::norm(w->A);
    amp += w->amp;
    prom += w->prom;
    iAmp += w->iAmp;
    o.freqs.push_back(w->f);
    if (std::find(o.hovers.begin(), o.hovers.end(), w->hover) == o.hovers.end()) o.hovers.push_back(w->hover);
  }
  const double n = static_cast<double>(same.size());
  o.found = true;
  o.fHz = f0;
  o.rateAmpDegs = amp / n;
  o.angleAmpDeg = o.rateAmpDegs / (2 * kPi * f0);
  o.prominence = prom / n;
  o.iSwing = iAmp / n;
  o.phaseDeg = std::arg(Sta) * 180 / kPi;
  o.coherence = Stt > 0 && Saa > 0 ? std::norm(Sta) / (Stt * Saa) : 0;
  // growth within a hover: the last window against the first
  for (std::size_t hi = 0; hi < hovers.size(); ++hi) {
    const Win *first = nullptr, *lastW = nullptr;
    for (const Win* w : same)
      if (w->hover == hi) {
        if (!first || w->t0 < first->t0) first = w;
        if (!lastW || w->t0 > lastW->t0) lastW = w;
      }
    if (!first || first == lastW) continue;
    const double e = first->amp / (2 * kPi * first->f), l = lastW->amp / (2 * kPi * lastW->f);
    if (l > o.ampLateDeg) {
      o.ampLateDeg = l;
      o.ampEarlyDeg = e;
    }
    if (l > 1.8 * e && l > 4) o.growing = true;
  }
  return o;
}

}  // namespace aptuner::engine::heavy
