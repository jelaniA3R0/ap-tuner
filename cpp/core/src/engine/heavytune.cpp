// Model-based tuning for heavy and slow vehicles: the pipeline (see heavytune.hpp).
#include "aptuner/engine/heavytune.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <map>
#include <numeric>
#include <regex>

#include "aptuner/core/parammeta.hpp"
#include "aptuner/engine/loganalysis.hpp"
#include "aptuner/engine/report.hpp"
#include "heavy_internal.hpp"

namespace aptuner::engine::heavy {

std::string num(double v, int dec) {
  if (!std::isfinite(v)) return "-";
  char b[64];
  std::snprintf(b, sizeof b, "%.*f", dec, v);
  return b;
}

namespace {

constexpr double kR2D = 57.29577951308232;

double median(Vec v) {
  if (v.empty()) return std::nan("");
  std::nth_element(v.begin(), v.begin() + static_cast<long>(v.size() / 2), v.end());
  return v[v.size() / 2];
}
double mean(const Vec& v) { return v.empty() ? std::nan("") : std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size()); }
double r2(double v) { return std::round(v * 100) / 100; }
double r3(double v) { return std::round(v * 1000) / 1000; }

Json issue(const char* level, const std::string& axis, const std::string& title, const std::string& text, const std::string& fix = "",
           bool cantTune = false) {
  return Json::obj({{"level", level}, {"axis", axis}, {"title", title}, {"text", text}, {"fix", fix}, {"cantTune", cantTune}});
}

/** samples of a series inside the steady part of the hovers (lift-off + skip .. end - 0.5 s) */
Vec hoverValues(const std::vector<FlightRecord>& fl, const std::vector<Hover>& hv, const Series FlightRecord::*s, double skip) {
  Vec out;
  for (const Hover& h : hv) {
    const Series& x = fl[h.flight].*s;
    for (std::size_t i = x.indexAt(h.tLift + skip); i < x.size() && x.t[i] <= h.t1 - 0.5; ++i)
      if (x.t[i] >= h.tLift + skip) out.push_back(x.v[i]);
  }
  return out;
}
Vec hoverAxisValues(const std::vector<FlightRecord>& fl, const std::vector<Hover>& hv, int axis, const Series AxisRec::*s, double skip) {
  Vec out;
  for (const Hover& h : hv) {
    const Series& x = fl[h.flight].axis[static_cast<std::size_t>(axis)].*s;
    for (std::size_t i = x.indexAt(h.tLift + skip); i < x.size() && x.t[i] <= h.t1 - 0.5; ++i)
      if (x.t[i] >= h.tLift + skip) out.push_back(x.v[i]);
  }
  return out;
}

/** lift motor thrust (0..1 of full) from PWM with the motor parameters (spin min / max, thrust expo) */
double thrustOf(double pwm, const log::Params& p, Family f) {
  const bool q = f == Family::Vtol;
  auto g = [&](const char* qn, const char* cn, double d) { return p.getOr(q ? qn : cn, d); };
  double lo = g("Q_M_PWM_MIN", "MOT_PWM_MIN", 0), hi = g("Q_M_PWM_MAX", "MOT_PWM_MAX", 0);
  if (!(lo > 0) || !(hi > lo)) {
    lo = 1000;
    hi = 2000;
  }
  const double smin = g("Q_M_SPIN_MIN", "MOT_SPIN_MIN", 0.15), smax = g("Q_M_SPIN_MAX", "MOT_SPIN_MAX", 0.95), e = g("Q_M_THST_EXPO", "MOT_THST_EXPO", 0.65);
  const double x = (pwm - lo) / (hi - lo);
  const double act = std::clamp((x - smin) / (smax - smin), 0.0, 1.0);
  return (1 - e) * act + e * act * act;
}
double pwmTop(const log::Params& p, Family f) {
  const bool q = f == Family::Vtol;
  double lo = p.getOr(q ? "Q_M_PWM_MIN" : "MOT_PWM_MIN", 0), hi = p.getOr(q ? "Q_M_PWM_MAX" : "MOT_PWM_MAX", 0);
  if (!(lo > 0) || !(hi > lo)) {
    lo = 1000;
    hi = 2000;
  }
  return lo + p.getOr(q ? "Q_M_SPIN_MAX" : "MOT_SPIN_MAX", 0.95) * (hi - lo);
}

/** mixer limit of an axis from the LIMIT flags (log) while flying: median |output| when limited */
struct MixLimit {
  double umax = 0, limitedPct = 0;
  bool measured = false;
};
MixLimit mixerLimit(const std::vector<FlightRecord>& fl, const std::vector<Hover>& hv, int axis) {
  MixLimit m;
  Vec atLim, all;
  std::size_t n = 0, nl = 0;
  for (const Hover& h : hv) {
    const AxisRec& a = fl[h.flight].axis[static_cast<std::size_t>(axis)];
    if (a.flags.empty()) {
      // telemetry has no LIMIT flags: the largest outputs delivered bound the limit from below
      for (std::size_t i = a.out.indexAt(h.tLift); i < a.out.size() && a.out.t[i] <= h.t1; ++i)
        if (a.out.t[i] >= h.tLift) all.push_back(std::fabs(a.out.v[i]));
      continue;
    }
    for (std::size_t i = a.flags.indexAt(h.tLift); i < a.flags.size() && a.flags.t[i] <= h.t1; ++i) {
      const double t = a.flags.t[i];
      if (t < h.tLift) continue;
      const int fl_ = static_cast<int>(a.flags.v[i]);
      if (fl_ & 4) continue;
      const double u = std::fabs(a.P.at(t) + a.I.at(t) + a.D.at(t) + (a.FF.empty() ? 0 : a.FF.at(t)));
      all.push_back(u);
      ++n;
      if (fl_ & 1) {
        atLim.push_back(u);
        ++nl;
      }
    }
  }
  if (n) m.limitedPct = 100.0 * static_cast<double>(nl) / static_cast<double>(n);
  // the mixer also raises LIMIT when the other axes or the throttle use up the motors: the limit is at least as large
  // as the largest output it actually delivered (99th percentile)
  double p99 = 0;
  if (!all.empty()) {
    std::sort(all.begin(), all.end());
    p99 = all[static_cast<std::size_t>(0.99 * static_cast<double>(all.size() - 1))];
  }
  if (atLim.size() >= 10) {
    m.umax = std::max(median(atLim), p99);
    m.measured = true;
  } else if (!all.empty()) {
    m.umax = std::max(0.3, p99 + 0.02);
  }
  return m;
}

Json gainsJson(const cc::Gains& g) {
  Json j = Json::obj({{"P", r3(g.P)}, {"I", r3(g.I)}, {"D", std::round(g.D * 1e5) / 1e5}, {"IMAX", r2(g.IMAX)}, {"angleP", r2(g.angleP)},
                      {"FLTT", g.FLTT}, {"FLTD", g.FLTD}, {"FLTE", g.FLTE}, {"gyroHz", g.gyroHz}, {"accMax", g.accMax}, {"rateMax", g.rateMax}});
  if (g.kind == cc::Kind::Yaw) j.set("pltRate", g.pltRate);
  else j.set("inputTC", g.inputTC);
  return j;
}

/** move `from` towards `to` by at most a factor k (the per-step limit without a validated model) */
double towards(double from, double to, double k) {
  if (!(from > 0) || !(to > 0)) return to;
  return to > from ? std::min(to, from * k) : std::max(to, from / k);
}

struct Row {
  std::string group, name, label, reason;
  double from = std::nan(""), to = 0;
};

Json rowsJson(const std::vector<Row>& rows, bool undo) {
  Json a = Json::array();
  for (const Row& r : rows) {
    Json j = Json::obj({{"group", r.group}, {"name", r.name}, {"value", undo ? r.from : r.to}, {"label", r.label}, {"reason", undo ? std::string("value as flown") : r.reason}});
    if (!undo && std::isfinite(r.from)) j.set("from", r.from);
    a.push(j);
  }
  return a;
}

}  // namespace

Json analyzeFlights(const std::vector<FlightRecord>& flights, const Options& opts) {
  Json res = Json::object();
  Json issues = Json::array(), summary = Json::array(), notesJ = Json::array();
  if (flights.empty()) {
    res.set("error", "no flights");
    return res;
  }
  const Family fam = flights[0].family;
  const FlightRecord& last = flights.back();
  const Json sysid = last.identity["sysid"];
  res.set("vehicle", Json::obj({{"sysid", sysid}, {"family", core::familyName(fam)}, {"firmware", last.firmware}, {"frameClass", last.identity["frameClass"]},
                                {"frameType", last.identity["frameType"]}}));
  Json fj = Json::array();
  for (const auto& f : flights) {
    fj.push(Json::obj({{"name", f.name}, {"source", f.source}, {"sysid", f.identity["sysid"]}, {"rateHz", r2(f.axis[1].actual.rateHz())}}));
    if (!(f.identity["sysid"] == sysid)) notesJ.push(f.name + " comes from another system id: it is analysed with the others only if it is the same airframe");
  }
  res.set("flights", fj);
  if (fam != Family::Vtol && fam != Family::Copter) {
    res.set("error", "model tuning is for multirotors and QuadPlane VTOL flight");
    return res;
  }

  // ---------------------------------------------------------------------------------- 1. hovers
  std::vector<std::string> hn;
  const std::vector<Hover> hovers = detectHovers(flights, hn);
  for (const auto& s : hn) notesJ.push(s);
  Json hj = Json::array();
  for (const Hover& h : hovers)
    hj.push(Json::obj({{"flight", flights[h.flight].name}, {"index", h.index}, {"t0", r2(h.t0)}, {"tLift", r2(h.tLift)}, {"t1", r2(h.t1)},
                       {"seconds", std::round(h.t1 - h.tLift)}, {"throttle", r2(h.meanThrottle)}, {"mode", h.modeNum}}));
  res.set("hovers", hj);
  if (hovers.empty()) {
    res.set("error", "no hover of 8 s or more was found (the controller must be flying, throttle above 60% of hover)");
    res.set("notes", notesJ);
    return res;
  }
  const log::Params flownP = hovers.back().params;   // the latest settings
  {
    // the tuning settings in effect (what "check my values" starts from)
    Json fp = Json::object();
    const std::string rc = "RC" + std::to_string(static_cast<int>(flownP.getOr("RCMAP_YAW", 4))) + "_";
    for (const auto& [k, v] : flownP.items())
      if (k.rfind("Q_A_", 0) == 0 || k.rfind("ATC_", 0) == 0 || k.rfind("Q_PLT_", 0) == 0 || k.rfind("PILOT_Y_", 0) == 0 || k.rfind(rc, 0) == 0 ||
          k == "INS_GYRO_FILTER" || k == "SCHED_LOOP_RATE" || k == "RCMAP_YAW" || k == "Q_ENABLE")
        fp.set(k, v);
    res.set("flownParams", fp);
  }
  std::array<cc::Gains, 3> flown;
  for (int a = 0; a < 3; ++a) flown[static_cast<std::size_t>(a)] = gainsFromParams(flownP, fam, a);

  // ---------------------------------------------------------------------------------- 2. diagnostics
  // vibration: gyro and controller output above 15 Hz (fast data only)
  Json vib = Json::object();
  std::array<double, 3> gyroHf{std::nan(""), std::nan(""), std::nan("")};
  for (int a = 0; a < 3; ++a) {
    Vec g, o;
    for (const Hover& h : hovers) {
      const Window w = axisWindow(flights[h.flight].axis[static_cast<std::size_t>(a)], h.tLift + 3, h.t1 - 0.5);
      if (w.size() < 100 || w.fs < 60) continue;
      g.push_back(highPassRms(w.act, w.fs, 15));
      o.push_back(highPassRms(w.out, w.fs, 15));
    }
    if (!g.empty()) {
      gyroHf[static_cast<std::size_t>(a)] = median(g);
      vib.set(kAxisNames[a], Json::obj({{"gyroAbove15HzDegs", r2(median(g))}, {"outputAbove15Hz", r3(median(o))}}));
    }
  }
  res.set("vibration", vib);
  // compare with the saved plan's vibration (same airframe)
  if (opts.plan.isObject() && opts.plan["vibration"].isObject())
    for (int a = 0; a < 2; ++a) {
      const double before = opts.plan["vibration"][kAxisNames[a]]["gyroAbove15HzDegs"].numOr(0);
      const double now = gyroHf[static_cast<std::size_t>(a)];
      if (before > 0 && std::isfinite(now) && now > 2 * before)
        issues.push(issue("bad", kAxisNames[a], "Vibration up", "The " + std::string(kAxisNames[a]) + " gyro above 15 Hz is " + num(now, 1) + " deg/s, against " +
                                                                  num(before, 1) + " deg/s in the flights the plan was made from (x" + num(now / before, 1) + ").",
                          "Mechanical: check props (balance, tracking, damage), motor mounts and the flight controller mount before flying the next step.", true));
    }
  {
    // the same within this batch: one flight much noisier than another
    for (int a = 0; a < 2; ++a) {
      Vec per;
      std::vector<std::string> names;
      for (std::size_t fi = 0; fi < flights.size(); ++fi) {
        Vec g;
        for (const Hover& h : hovers)
          if (h.flight == fi) {
            const Window w = axisWindow(flights[fi].axis[static_cast<std::size_t>(a)], h.tLift + 3, h.t1 - 0.5);
            if (w.size() > 100 && w.fs >= 60) g.push_back(highPassRms(w.act, w.fs, 15));
          }
        if (!g.empty()) {
          per.push_back(median(g));
          names.push_back(flights[fi].name);
        }
      }
      if (per.size() >= 2) {
        const auto mn = std::min_element(per.begin(), per.end()), mx = std::max_element(per.begin(), per.end());
        if (*mx > 2.5 * *mn)
          issues.push(issue("warn", kAxisNames[a], "Vibration differs between flights",
                            names[static_cast<std::size_t>(mx - per.begin())] + ": " + num(*mx, 1) + " deg/s above 15 Hz on " + kAxisNames[a] + ", against " + num(*mn, 1) +
                                " deg/s in " + names[static_cast<std::size_t>(mn - per.begin())] + ". Something mechanical changed between them.",
                            "Props, motor mounts, flight-controller mount.", true));
      }
    }
  }
  // mixer limits
  std::array<MixLimit, 3> mix;
  for (int a = 0; a < 3; ++a) mix[static_cast<std::size_t>(a)] = mixerLimit(flights, hovers, a);
  // yaw: trim, I term, motor split
  const Vec yawOut = hoverAxisValues(flights, hovers, 2, &AxisRec::out, 5), yawI = hoverAxisValues(flights, hovers, 2, &AxisRec::I, 5);
  const double yawOutMean = mean(yawOut), yawIMean = mean(yawI);
  double ccw = std::nan(""), cw = std::nan("");
  {
    Vec m12, m34;
    for (const Hover& h : hovers) {
      const auto& M = flights[h.flight].motors;
      if (!M.count(1) || !M.count(2) || !M.count(3) || !M.count(4)) continue;
      const Series& s1 = M.at(1);
      for (std::size_t i = s1.indexAt(h.tLift + 5); i < s1.size() && s1.t[i] <= h.t1 - 0.5; ++i) {
        const double t = s1.t[i];
        m12.push_back((s1.v[i] + M.at(2).at(t)) / 2);
        m34.push_back((M.at(3).at(t) + M.at(4).at(t)) / 2);
      }
    }
    ccw = mean(m12);
    cw = mean(m34);
  }
  const bool quadX = (fam == Family::Vtol ? flownP.getOr("Q_FRAME_CLASS", 1) : flownP.getOr("FRAME_CLASS", 1)) == 1 &&
                     (fam == Family::Vtol ? flownP.getOr("Q_FRAME_TYPE", 1) : flownP.getOr("FRAME_TYPE", 1)) == 1;
  // motor saturation
  {
    const double top = pwmTop(flownP, fam);
    std::size_t n = 0, sat = 0;
    for (const Hover& h : hovers) {
      const auto& M = flights[h.flight].motors;
      if (M.empty()) continue;
      const Series& s1 = M.begin()->second;
      for (std::size_t i = s1.indexAt(h.tLift + 3); i < s1.size() && s1.t[i] <= h.t1 - 0.5; ++i) {
        double mx = 0;
        for (const auto& [k, s] : M) mx = std::max(mx, s.at(s1.t[i]));
        ++n;
        if (mx >= top - 10) ++sat;
      }
    }
    const double pct = n ? 100.0 * static_cast<double>(sat) / static_cast<double>(n) : 0;
    res.set("motorsAtTopPct", r2(pct));
    if (pct > 2)
      issues.push(issue(pct > 8 ? "bad" : "warn", "", "Motors at their limit",
                        "A lift motor sits at its top output (" + num(top, 0) + " us) " + num(pct, 1) + "% of the hover time. Then the mixer cannot add any correction in that direction.",
                        "Fix the reason it needs that much (torque imbalance, CG, weight) rather than tuning around it."));
  }

  // ---------------------------------------------------------------------------------- 3. oscillations (roll / pitch)
  std::array<Oscillation, 3> osc;
  for (int a = 0; a < 2; ++a) osc[static_cast<std::size_t>(a)] = findOscillation(flights, hovers, a);
  // ---------------------------------------------------------------------------------- 4. identification
  std::array<AxisModel, 3> models;
  std::vector<std::string> modelNotes;
  auto aRangeOf = [](const AxisModel& m) {
    if (!m.ok || m.set.empty()) return Json();
    double lo = 1e18, hi = 0;
    for (const auto& p : m.set) {
      lo = std::min(lo, p.a);
      hi = std::max(hi, p.a);
    }
    return Json(Json::Array{r3(lo), r3(hi)});
  };
  auto planModel = [&](int a) {
    if (!opts.plan.isObject() || !opts.plan["models"][kAxisNames[a]].isObject()) return AxisModel();
    return AxisModel::fromJson(opts.plan["models"][kAxisNames[a]]);
  };
  // the steady yaw output of every hover (this flight's torque trim, also when the yaw fit has too little to work on)
  std::vector<std::pair<std::size_t, double>> yawTrims;   // {hover, trim}
  const YawIdent yid = identifyYaw(flights, hovers, fam);
  const MixLimit& my = mix[2];
  models[2] = yawModel(yid, my.measured ? my.umax : std::max(0.5, my.umax), my.measured);
  if (models[2].ok) {
    for (std::size_t k = 0; k < yid.hoverIdx.size(); ++k) yawTrims.push_back({yid.hoverIdx[k], yid.trims[k]});
    // a saved yaw model that disagrees with this fit: the design must hold for both
    AxisModel pm = planModel(2);
    if (pm.ok) {
      const double ratio = models[2].set[0].a / pm.set[0].a;
      if (ratio > 1.3 || ratio < 1 / 1.3) {
        const double shift = models[2].trim - pm.trim;
        for (auto p : pm.set) {
          p.trim += shift;
          p.umax = models[2].umax;
          models[2].set.push_back(p);
        }
        models[2].method += " + saved model (they disagree)";
        modelNotes.push_back("yaw: this flight gives control power " + num(models[2].set[0].a, 2) + ", the saved model " + num(pm.set[0].a, 2) +
                             ": the yaw design must hold for both");
      }
    }
  } else {
    AxisModel pm = planModel(2);
    for (std::size_t hi = 0; hi < hovers.size(); ++hi) {
      const Vec v = hoverAxisValues(flights, {hovers[hi]}, 2, &AxisRec::out, 5);
      if (!v.empty()) yawTrims.push_back({hi, mean(v)});
    }
    if (pm.ok && !yawTrims.empty()) {
      double t = 0;
      for (const auto& [h, v] : yawTrims) t += v;
      t /= static_cast<double>(yawTrims.size());
      const double shift = t - pm.trim;
      for (auto& p : pm.set) p.trim += shift;
      pm.trimMax += shift;
      pm.trim = t;
      if (my.measured) {
        pm.umax = my.umax;
        for (auto& p : pm.set) p.umax = my.umax;
      }
      pm.method = "saved model (" + pm.method + "), trim from this flight";
      modelNotes.push_back("yaw: " + yid.reason + "; the saved yaw model is used with this flight's trim (" + num(t, 2) + ")");
      models[2] = pm;
    }
  }
  Json identJ = Json::object();
  {
    Json tr = Json::array();
    for (const auto& [h, v] : yawTrims) tr.push(r2(v));
    identJ.set("yaw", Json::obj({{"ok", models[2].ok}, {"fitOk", yid.ok}, {"reason", yid.reason}, {"method", models[2].ok ? models[2].method : std::string("yaw regression")},
                                 {"a", models[2].ok ? Json(r3(models[2].set[0].a)) : Json(r3(yid.a))}, {"c", models[2].ok ? Json(r3(models[2].set[0].c)) : Json(r3(yid.c))},
                                 {"r2", r3(yid.r2)}, {"windows", yid.windows}, {"aRange", aRangeOf(models[2])}, {"trims", tr}, {"umax", r2(models[2].umax)},
                                 {"umaxMeasured", my.measured}, {"limitedPct", r2(my.limitedPct)}}));
  }
  for (int a = 0; a < 2; ++a) {
    const auto A = static_cast<std::size_t>(a);
    const RpIdent rid = identifyMarginal(osc[A], hovers, fam, a);
    const Vec iv = hoverAxisValues(flights, hovers, a, &AxisRec::I, 5);
    const double trim = iv.empty() ? 0 : mean(iv);
    const MixLimit& ml = mix[A];
    const double umax = ml.measured ? ml.umax : 0.5;
    AxisModel pm = planModel(a);
    if (pm.ok) {
      for (auto& p : pm.set) {
        p.trim = trim;
        if (ml.measured) p.umax = umax;
      }
      pm.trim = pm.trimMax = trim;
    }
    if (rid.ok) {
      models[A] = rpModel(rid, trim, umax, ml.measured);
      if (rid.a2 > 0)
        modelNotes.push_back(std::string(kAxisNames[a]) + " wobbles at two frequencies (" + num(osc[A].fHz, 2) + " and " + num(osc[A].f2Hz, 2) +
                             " Hz): the control power is somewhere in " + num(rid.aLo, 1) + "-" + num(rid.aHi, 1) + " rad/s/s per unit; the design covers all of it");
      // a saved model that disagrees: both are kept, the design must hold on all of them
      if (pm.ok) {
        const double ratio = models[A].set[0].a / pm.set[0].a;
        if (ratio > 1.5 || ratio < 1 / 1.5) {
          for (const auto& p : pm.set) models[A].set.push_back(p);
          models[A].method += " + saved model (they disagree)";
          modelNotes.push_back(std::string(kAxisNames[a]) + ": this flight gives control power " + num(models[A].set[0].a, 1) + ", the saved model " +
                               num(pm.set[0].a, 1) + ": the " + kAxisNames[a] + " model is uncertain, so the design must hold for both");
        }
      }
    } else if (pm.ok) {
      models[A] = pm;
      models[A].method = "saved model (" + models[A].method + ")";
    } else {
      models[A].ok = false;
      models[A].reason = rid.reason;
    }
    Json fits = Json::array();
    for (const auto& f : rid.fits) fits.push(Json::obj({{"a", r2(f.a)}, {"tau", r3(f.tau)}, {"T", f.T}}));
    identJ.set(kAxisNames[a], Json::obj({{"ok", models[A].ok}, {"method", models[A].method}, {"reason", models[A].ok ? std::string() : models[A].reason},
                                         {"a", models[A].ok ? Json(r2(models[A].set[0].a)) : Json()},
                                         {"tauRange", rid.ok ? Json(Json::Array{r3(rid.tauLo), r3(rid.tauHi)}) : Json()}, {"fits", fits},
                                         {"aRange", aRangeOf(models[A])}, {"frequencyHz", osc[A].found ? Json(r2(osc[A].fHz)) : Json()},
                                         {"secondFrequencyHz", osc[A].f2Hz > 0 ? Json(r2(osc[A].f2Hz)) : Json()}, {"trim", r3(trim)}, {"umax", r2(umax)},
                                         {"umaxMeasured", ml.measured}}));
  }
  res.set("identification", identJ);
  for (const auto& s : modelNotes) notesJ.push(s);

  // ---------------------------------------------------------------------------------- 5. validation
  Json axesJ = Json::object();
  std::array<bool, 3> valid{false, false, false};
  std::array<bool, 3> cascade{false, false, false};
  Json replayJ = Json::object();
  // yaw: replay every hover with its own gains and trim
  if (models[2].ok) {
    Json hr = Json::array();
    int pass = 0, n = 0;
    for (const auto& [hIdx, trim] : yawTrims) {
      const Hover& h = hovers[hIdx];
      cc::Plant p = models[2].set[0];
      p.trim = trim;
      const YawReplay rp = replayYaw(flights[h.flight], h, hIdx, gainsFromParams(h.params, fam, 2), p);
      ++n;
      if (rp.pass) ++pass;
      hr.push(Json::obj({{"hover", h.id()}, {"flight", flights[h.flight].name}, {"swingLogDeg", std::round(rp.swingLog)}, {"swingModelDeg", std::round(rp.swingModel)},
                         {"headingRmsDeg", std::round(rp.headingRmsDeg)}, {"outputRms", r3(rp.outRms)}, {"pass", rp.pass}, {"chart", rp.chart}}));
    }
    valid[2] = n > 0 && pass * 10 >= 6 * n;
    const std::string txt = std::to_string(pass) + " of " + std::to_string(n) + " hovers replayed within 5 deg (or 12%) of the logged heading swing";
    replayJ.set("yaw", Json::obj({{"ok", valid[2]}, {"text", txt}, {"hovers", hr}}));
  }

  for (int a = 0; a < 2; ++a) {
    const auto A = static_cast<std::size_t>(a);
    if (!models[A].ok) continue;
    const Oscillation& o = osc[A];
    if (o.found) {
      const Hover& h = hovers[o.hovers.front()];
      const cc::Gains g = gainsFromParams(h.params, fam, a);
      const RpCheck c = checkRpOscillation(flights[h.flight], h, g, models[A], o);
      valid[A] = c.ok;
      cascade[A] = c.ok && c.pmWorst <= 10;
      // chart: 30 s of the logged angle (mean removed) and the model's gust response with the same gains
      Json ct = Json::array(), cl = Json::array(), cm = Json::array();
      const Series& ang = flights[h.flight].axis[A].ang;
      Vec lv;
      for (std::size_t i = ang.indexAt(h.tLift + 3); i < ang.size() && ang.t[i] < std::min(h.t1, h.tLift + 33); ++i) lv.push_back(ang.v[i]);
      const double mu = mean(lv);
      cc::Plant pn = models[A].set[0];
      pn.trim = 0;
      const cc::GustSequence gust(31);
      cc::SimInput si;
      si.seconds = 60;
      si.shaped = true;
      si.angleDeg = [](double) { return 0.0; };
      si.dist = [&](double t) { return -gust.at(t); };
      si.recordHz = 5;
      const cc::SimOutput so = cc::simulate(g, pn, si);
      for (std::size_t k = 0; k < so.t.size(); ++k) {
        if (so.t[k] < 30) continue;
        const double t = so.t[k] - 30;
        ct.push(std::round(t * 10) / 10);
        cl.push(std::round((ang.at(h.tLift + 3 + t) - mu) * 10) / 10);
        cm.push(std::round(so.att[k] * 10) / 10);
      }
      replayJ.set(kAxisNames[a], Json::obj({{"ok", c.ok}, {"text", c.text}, {"pmWorstDeg", std::round(c.pmWorst)}, {"fLogHz", r2(c.fLogHz)},
                                            {"simAmpDeg", Json(Json::Array{r2(c.simAmpLo), r2(c.simAmpHi)})},
                                            {"chart", Json::obj({{"t", ct}, {"log", cl}, {"model", cm}, {"hover", h.id()}})}}));
    } else {
      // no oscillation: the model (saved) must agree that the flown gains are well damped
      const Json pr = predictRp(flown[A], models[A]);
      valid[A] = !pr["oscillationExpected"].asBool();
      replayJ.set(kAxisNames[a], Json::obj({{"ok", valid[A]},
                                            {"text", std::string("no low-frequency oscillation in this flight; the model gives the flown gains a worst phase margin of ") +
                                                         num(pr["pmWorstDeg"].numOr(0), 0) + " deg" + (valid[A] ? " (well damped: consistent)" : " (it expects an oscillation: inconsistent)")},
                                            {"pmWorstDeg", pr["pmWorstDeg"]}}));
    }
  }
  res.set("replay", replayJ);

  // ---------------------------------------------------------------------------------- diagnostics texts
  // torque imbalance
  if (std::isfinite(yawOutMean) && (std::fabs(yawOutMean) > 0.15 || (std::isfinite(yawIMean) && std::fabs(yawIMean) > 0.6 * flown[2].IMAX) ||
                                    (std::isfinite(ccw) && std::fabs(ccw - cw) > 100))) {
    std::string t = "The yaw controller holds a steady " + num(yawOutMean, 2) + " of output to stop the vehicle turning";
    if (std::isfinite(yawIMean)) t += "; its I term sits at " + num(yawIMean, 2) + " (limit " + num(flown[2].IMAX, 2) + ")";
    if (std::isfinite(ccw) && quadX) t += "; the counter-clockwise motors (1 + 2) run at " + num(ccw, 0) + " us against " + num(cw, 0) + " us for the clockwise ones (3 + 4)";
    if (models[2].ok) t += ". The model measures this torque trim at " + num(models[2].trim, 2) + " of the output";
    t += ". Tuning can't remove a torque imbalance: it only decides how quickly the controller catches it.";
    issues.push(issue("bad", "yaw", "Torque imbalance (yaw trim)", t,
                      "Causes: motor tilt, the wing bending under thrust (on a tandem the more heavily loaded diagonal adds yaw in the same direction), or props with different torque. "
                      "Check every motor mount with an inclinometer, unloaded and loaded (weights on the booms), and that all props are the same model and pitch.",
                      true));
  }
  if (models[2].ok) {
    const double head = models[2].umax - std::fabs(models[2].trim);
    const double a = models[2].set[0].a;
    const double weak = a * std::max(0.0, head) * kR2D, strong = a * (models[2].umax + std::fabs(models[2].trim)) * kR2D;
    const double v = 14;
    issues.push(issue(head < 0.3 ? "warn" : "ok", "yaw", "Yaw authority",
                      std::string("The mixer gives yaw at most ") + num(models[2].umax, 2) + (my.measured ? " (measured from the LIMIT flags)" : " (estimated)") +
                          "; the trim uses " + num(std::fabs(models[2].trim), 2) + ", leaving " + num(head, 2) + " for corrections. The vehicle can accelerate in yaw " +
                          num(weak, 1) + " deg/s/s against the imbalance and " + num(strong, 1) + " deg/s/s with it: stopping a " + num(v, 0) + " deg/s drift takes " +
                          num(weak > 0 ? v * v / (2 * weak) : 999, 0) + " deg.",
                      head < 0.3 ? "More yaw authority needs a mechanical fix (1-2 deg of motor tilt in the right direction), not gains." : "", head < 0.3));
  }
  // CG from the front / rear thrust split (quad X: motors 1 + 3 front, 2 + 4 rear; 2 + 3 left, 1 + 4 right)
  if (quadX) {
    double Ff = 0, Fr = 0, Fl = 0, Fri = 0;
    std::size_t n = 0;
    for (const Hover& h : hovers) {
      const auto& M = flights[h.flight].motors;
      if (!M.count(1) || !M.count(2) || !M.count(3) || !M.count(4)) continue;
      const Series& s1 = M.at(1);
      for (std::size_t i = s1.indexAt(h.tLift + 5); i < s1.size() && s1.t[i] <= h.t1 - 0.5; ++i) {
        const double t = s1.t[i];
        const double m1 = thrustOf(s1.v[i], h.params, fam), m2 = thrustOf(M.at(2).at(t), h.params, fam), m3 = thrustOf(M.at(3).at(t), h.params, fam),
                     m4 = thrustOf(M.at(4).at(t), h.params, fam);
        Ff += m1 + m3;
        Fr += m2 + m4;
        Fl += m2 + m3;
        Fri += m1 + m4;
        ++n;
      }
    }
    if (n > 50 && Ff + Fr > 0) {
      const double off = (Fr - Ff) / (2 * (Ff + Fr)), offL = (Fl - Fri) / (2 * (Fl + Fri));
      const double pitchI = models[1].trim;
      std::string t = "Front / rear thrust " + num(100 * Ff / (Ff + Fr), 0) + "% / " + num(100 * Fr / (Ff + Fr), 0) + "%: the centre of gravity is " +
                      num(std::fabs(off) * 100, 1) + "% of the front-rear motor spacing " + (off > 0 ? "aft" : "forward") + " of the motors' midpoint";
      if (std::fabs(offL) > 0.01) t += ", and " + num(std::fabs(offL) * 100, 1) + "% of the left-right spacing to the " + (offL > 0 ? "left" : "right");
      t += ". The pitch I term (" + num(pitchI, 2) + ") " + ((off > 0) == (pitchI < 0) ? "agrees" : "does not clearly agree") + ".";
      std::string fix;
      if (opts.motorSpacingM > 0) fix = "Move " + std::string(off > 0 ? "the battery or payload forward" : "weight aft") + " so the CG shifts " + num(std::fabs(off) * opts.motorSpacingM * 100, 1) + " cm.";
      else fix = "Enter the front-rear motor spacing to get the shift in cm. A centred CG lets all motors share the load and frees control margin.";
      if (std::fabs(off) > 0.01)
        issues.push(issue(std::fabs(off) > 0.04 ? "warn" : "ok", "pitch", "Centre of gravity", t, fix, true));
      res.set("cg", Json::obj({{"aftFractionOfSpacing", r3(off)}, {"leftFractionOfSpacing", r3(offL)}, {"frontPct", r2(100 * Ff / (Ff + Fr))}}));
    }
  }
  // oscillations
  for (int a = 0; a < 2; ++a) {
    const auto A = static_cast<std::size_t>(a);
    const Oscillation& o = osc[A];
    if (!o.found) {
      issues.push(issue("ok", kAxisNames[a], "No low-frequency wobble", "No persistent 0.2-3 Hz oscillation on " + std::string(kAxisNames[a]) + " in " +
                                                                             std::to_string(std::max(0, o.windows)) + " ten-second windows."));
      continue;
    }
    std::string t = std::string(kAxisNames[a]) + " wobbles at " + num(o.fHz, 2) + " Hz (+-" + num(o.angleAmpDeg, 1) + " deg" +
                    (o.growing ? ", growing to +-" + num(o.ampLateDeg, 1) + " deg" : "") + ") in " + num(100 * o.persistence, 0) + "% of the windows; the rate target leads the actual rate by " +
                    num(o.phaseDeg, 0) + " deg and the I term swings +-" + num(o.iSwing, 3) + " every cycle.";
    if (cascade[A])
      t += " The model confirms a cascade: the angle loop is too fast for the rate loop (worst phase margin " + num(replayJ[kAxisNames[a]]["pmWorstDeg"].numOr(0), 0) +
           " deg with the flown gains).";
    issues.push(issue("bad", kAxisNames[a], cascade[A] ? "Cascade oscillation" : "Low-frequency oscillation", t,
                      "Don't chase it with the stick. Lower the angle P below the rate-loop speed (the proposal does this with the rate gains together)."));
  }
  // slew limiter
  for (int a = 0; a < 3; ++a) {
    const Vec dm = hoverAxisValues(flights, hovers, a, &AxisRec::dmod, 3);
    if (!dm.empty() && *std::min_element(dm.begin(), dm.end()) < 0.9)
      issues.push(issue("warn", kAxisNames[a], "Slew limiter active",
                        "The D/P slew limiter cut the gains to " + num(*std::min_element(dm.begin(), dm.end()), 2) + " at times: the output was changing faster than SMAX allows (noise or the edge of stability)."));
  }
  // hover throttle learning
  {
    const Vec thr = hoverValues(flights, hovers, &FlightRecord::throttle, 5);
    const double learned = flownP.getOr(paramName(fam, 0, "HOVER"), 0);
    if (!thr.empty() && learned > 0) {
      const double real = median(thr);
      const bool off = std::fabs(real - learned) > 0.05;
      issues.push(issue(off ? "warn" : "ok", "", "Hover throttle",
                        "The vehicle hovers at " + num(real, 2) + " throttle; " + paramName(fam, 0, "HOVER") + " is " + num(learned, 2) + "." +
                            (off ? " The altitude controller in QHOVER / QLOITER (or AltHold / Loiter) uses this value, so climbs and descents are uneven until it is learned." : ""),
                        off ? "Keep " + paramName(fam, 0, "HOVER_LEARN") + " at 2 (learn and save) and hover 30 s in QHOVER / AltHold, or set it to " + num(real, 2) + "." : ""));
      res.set("hoverThrottle", Json::obj({{"measured", r3(real)}, {"param", r3(learned)}}));
    }
  }
  // altitude hold in the altitude-holding modes
  {
    Vec err;
    for (const Hover& h : hovers) {
      const FlightRecord& f = flights[h.flight];
      if (f.altDes.empty() || f.alt.empty()) continue;
      for (std::size_t i = f.alt.indexAt(h.tLift + 5); i < f.alt.size() && f.alt.t[i] <= h.t1 - 1; ++i) {
        const double t = f.alt.t[i], m = f.mode.empty() ? -1 : f.mode.at(t);
        const bool hold = fam == Family::Vtol ? (m == 18 || m == 19 || m == 20 || m == 21) : (m == 2 || m == 5 || m == 16);
        if (hold) err.push_back(f.alt.v[i] - f.altDes.at(t));
      }
    }
    if (err.size() > 50) {
      double ss = 0;
      for (double e : err) ss += e * e;
      const double rms = std::sqrt(ss / static_cast<double>(err.size()));
      issues.push(issue(rms > 0.7 ? "warn" : "ok", "", "Altitude hold", "In the altitude-holding modes the height stayed within " + num(rms, 2) + " m RMS of its target.",
                        rms > 0.7 ? "Learn the hover throttle first; then the vertical velocity / acceleration gains." : ""));
    }
  }
  // compass
  {
    std::vector<std::string> msgs;
    for (const auto& f : flights)
      for (const auto& m : f.text)
        if (std::regex_search(m.msg, std::regex("mag|compass", std::regex::icase))) msgs.push_back(m.msg);
    const Vec fld = hoverValues(flights, hovers, &FlightRecord::magField, 3);
    const Vec thr = hoverValues(flights, hovers, &FlightRecord::throttle, 3);
    std::string t;
    bool bad = !msgs.empty();
    if (!msgs.empty()) t = "Messages: \"" + msgs.front() + "\"" + (msgs.size() > 1 ? " and " + std::to_string(msgs.size() - 1) + " more" : "") + ". ";
    if (fld.size() > 50) {
      const double mu = mean(fld);
      double ss = 0;
      for (double v : fld) ss += (v - mu) * (v - mu);
      const double cv = std::sqrt(ss / static_cast<double>(fld.size())) / mu;
      t += "In the hovers the field strength varied by " + num(100 * cv, 0) + "%.";
      if (cv > 0.1) bad = true;
    }
    if (!t.empty())
      issues.push(issue(bad ? "warn" : "ok", "", "Compass", t,
                        bad ? "Move the compass away from power wires and the lift battery, run compass calibration and, with current logging, COMPASS_MOT; check COMPASS_ORIENT." : "", bad));
  }

  // ---------------------------------------------------------------------------------- 6. vehicle class
  Json why = Json::array();
  if (models[2].ok && models[2].set[0].a < 1) why.push("yaw control power " + num(models[2].set[0].a, 2) + " rad/s/s per unit (< 1)");
  for (int a = 0; a < 2; ++a)
    if (models[static_cast<std::size_t>(a)].ok && models[static_cast<std::size_t>(a)].set[0].a < 20)
      why.push(std::string(kAxisNames[a]) + " control power " + num(models[static_cast<std::size_t>(a)].set[0].a, 1) + " rad/s/s per unit (< 20)");
  for (int a = 0; a < 2; ++a)
    if (cascade[static_cast<std::size_t>(a)]) why.push(std::string("cascade oscillation on ") + kAxisNames[a] + " confirmed by the model");
  const bool heavy = why.size() > 0;
  res.set("heavyClass", Json::obj({{"heavy", heavy}, {"why", why}}));

  // ---------------------------------------------------------------------------------- 7. design
  Design design;
  bool designed = false;
  if (heavy) {
    DesignInputs din;
    din.fam = fam;
    din.models = models;
    din.flown = flown;
    // measured vibration peaks of the pitch gyro (fast data)
    for (const Hover& h : hovers) {
      const Window w = axisWindow(flights[h.flight].axis[1], h.tLift + 3, h.t1 - 0.5);
      if (w.size() < 600 || w.fs < 100) continue;
      double fsD = 0;
      const Vec x = decimate(w.act, w.fs, 150, fsD);
      const Spectrum sp = dftGrid(Vec(x.begin(), x.begin() + static_cast<long>(std::min<std::size_t>(x.size(), static_cast<std::size_t>(fsD * 20)))), fsD, 15,
                                  std::min(70.0, fsD / 2 - 1), 0.5);
      std::vector<std::pair<double, double>> pk;
      const double med = median(sp.amp);
      for (std::size_t j = 1; j + 1 < sp.f.size(); ++j)
        if (sp.amp[j] > sp.amp[j - 1] && sp.amp[j] >= sp.amp[j + 1] && sp.amp[j] > 3 * med) pk.push_back({sp.amp[j], sp.f[j]});
      std::sort(pk.rbegin(), pk.rend());
      // the two strongest peaks at least 4 Hz apart
      for (const auto& [amp, f] : pk) {
        if (din.noiseFreqs.size() >= 2) break;
        if (din.noiseFreqs.empty() || std::fabs(din.noiseFreqs[0] - f) >= 4) din.noiseFreqs.push_back(f);
      }
      break;
    }
    design = designAll(din);
    designed = true;
    Json nf = Json::array();
    for (double f : din.noiseFreqs) nf.push(r2(f));
    res.set("noisePeaksHz", nf);
    for (const auto& s : design.notes) notesJ.push(s);
  }

  // ---------------------------------------------------------------------------------- 8. per-axis report
  for (int a = 0; a < 3; ++a) {
    const auto A = static_cast<std::size_t>(a);
    Json ax = Json::obj({{"model", models[A].toJson()}, {"validated", valid[A]}, {"flown", gainsJson(flown[A])}});
    if (models[A].ok) ax.set("predictedFlown", a == 2 ? predictYaw(flown[A], models[A]) : predictRp(flown[A], models[A]));
    if (a == 2 && models[2].ok) {
      const cc::Poles po = cc::slowAxisPoles(models[2].set[0].a, models[2].set[0].c, flown[2].P, flown[2].I, flown[2].angleP);
      ax.set("polesFlown", Json::obj({{"zeta", r2(po.zeta)}, {"periodS", std::round(po.periodS * 10) / 10}, {"stable", po.stable},
                                      {"angleToRateSpeed", r2(flown[2].angleP / (models[2].set[0].a * flown[2].P + models[2].set[0].c))}}));
    } else if (models[A].ok) {
      const double bw = models[A].set[0].a * flown[A].P;
      ax.set("angleToRateSpeed", r2(flown[A].angleP / std::max(1e-6, bw)));
    }
    if (designed) {
      const AxisDesign& d = design.axes[A];
      Json dj = Json::obj({{"ok", d.ok}, {"reason", d.reason}, {"candidates", d.candidates}, {"accepted", d.accepted}});
      if (d.ok) {
        dj.set("gains", gainsJson(d.g));
        dj.set("pmWorstDeg", std::round(d.wm.pmDeg));
        dj.set("gmWorstDb", d.wm.gmDb >= 98 ? Json() : Json(std::round(d.wm.gmDb * 10) / 10));
        dj.set("ms", r2(d.wm.ms));
        dj.set("noiseVsNow", r2(d.noiseRatio));
        dj.set("metrics", d.metrics);
        dj.set("stress", d.stress);
        Json lw = Json::array();
        for (const auto& s : d.lim.why) lw.push(s);
        dj.set("limitsWhy", lw);
        dj.set("predicted", a == 2 ? predictYaw(d.g, models[A]) : predictRp(d.g, models[A]));
      }
      ax.set("design", dj);
    }
    axesJ.set(kAxisNames[a], ax);
  }
  res.set("axes", axesJ);

  // ---------------------------------------------------------------------------------- 9. steps
  Json steps = Json::array();
  struct Group {
    std::string title;
    std::vector<int> axes;
  };
  std::vector<std::vector<Row>> stepRows;
  std::vector<std::string> stepTitles;
  std::vector<std::array<cc::Gains, 3>> stepGains;
  if (designed) {
    std::array<cc::Gains, 3> cur = flown;
    auto changed = [](double a, double b) { return std::fabs(a - b) > 1e-6 * std::max(1.0, std::fabs(a)) && std::fabs(a - b) > 0.02 * std::fabs(a); };
    for (const Group& G : {Group{"yaw", {2}}, Group{"roll and pitch", {0, 1}}}) {
      bool any = false;
      for (int a : G.axes) any = any || design.axes[static_cast<std::size_t>(a)].ok;
      if (!any) continue;
      // what is flown already passes every test and the design is not clearly better (15%): keep it
      bool keep = true;
      for (int a : G.axes) {
        const AxisDesign& d = design.axes[static_cast<std::size_t>(a)];
        if (d.ok && !(d.flownAcceptable && d.score >= 0.85 * d.flownScore)) keep = false;
      }
      if (keep) {
        notesJ.push("Keep the " + G.title + " settings you fly: they pass every test on the model and the design would not be clearly better.");
        for (int a : G.axes)
          if (design.axes[static_cast<std::size_t>(a)].ok) res.at("axes").at(kAxisNames[a]).at("design").set("kept", true);
        continue;
      }
      for (int stepNo = 0; stepNo < 4; ++stepNo) {
        std::vector<Row> rows;
        std::array<cc::Gains, 3> nx = cur;
        bool limited = false;
        for (int a : G.axes) {
          const auto A = static_cast<std::size_t>(a);
          const AxisDesign& d = design.axes[A];
          if (!d.ok) continue;
          const cc::Gains& tg = d.g;
          cc::Gains& g = nx[A];
          const bool jump = valid[A];
          auto mv = [&](double from, double to, double k) {
            double v = jump ? to : towards(from, to, k);
            // an intermediate step: two significant figures
            if (std::fabs(v - to) > 1e-12 && v > 0) {
              const double e = std::pow(10.0, std::floor(std::log10(v)) - 1);
              v = std::round(v / e) * e;
            }
            if (std::fabs(v - to) > 1e-9) limited = true;
            return v;
          };
          g.P = mv(cur[A].P, tg.P, 2);
          g.I = mv(cur[A].I, tg.I, 2);
          g.D = mv(cur[A].D, tg.D, 1.5);
          g.angleP = mv(cur[A].angleP, tg.angleP, 1.5);
          g.IMAX = tg.IMAX;
          g.accMax = tg.accMax;
          g.rateMax = a == 2 ? tg.rateMax : g.rateMax;
          g.pltRate = tg.pltRate;
          g.inputTC = tg.inputTC;
          g.gyroHz = tg.gyroHz;
          g.FLTT = tg.FLTT;
          g.FLTD = tg.FLTD;
          const std::string grp = kAxisNames[a];
          const std::string reasonTxt = valid[A] ? "robust design on the validated model" : "towards the model design, at most x2 per step (model not validated by a replay)";
          auto add = [&](const std::string& key, double from, double to, const std::string& label, const std::string& reason) {
            if (!changed(from, to)) return;
            const std::string name = paramName(fam, a, key);
            if (name.empty()) return;
            rows.push_back({grp, name, label, reason, paramValueFor(flownP, fam, a, key, from), paramValueFor(flownP, fam, a, key, to)});
          };
          add("P", cur[A].P, g.P, "rate P", reasonTxt);
          add("I", cur[A].I, g.I, "rate I", reasonTxt + "; I/P " + num(g.P > 0 ? g.I / g.P : 0, 2) + " from the design");
          if (a != 2) add("D", cur[A].D, g.D, "rate D", reasonTxt);
          add("IMAX", cur[A].IMAX, g.IMAX, "I limit", d.lim.why.size() > 2 && a == 2 ? d.lim.why[2] : "holds the measured trim with 0.2 to spare");
          add("ANG", cur[A].angleP, g.angleP, "angle P", "the angle loop must stay slower than the rate loop (cascade rule)");
          add("ACC", cur[A].accMax, g.accMax, "acceleration limit (deg/s/s)", d.lim.why.empty() ? "what the vehicle can do" : a == 2 ? d.lim.why[0] : "70% of what the vehicle can do, the same for roll and pitch: " + [&] { std::string s; for (const auto& w : d.lim.why) s += (s.empty() ? "" : "; ") + w; return s; }());
          if (a == 2) {
            add("RMAX", cur[A].rateMax, g.rateMax, "rate limit (deg/s)", d.lim.why.size() > 1 ? d.lim.why[1] : "stopping distance");
            add("PLT_RATE", cur[A].pltRate, g.pltRate, "pilot yaw rate (deg/s)", "the same as the rate limit");
            const auto wpy = flownP.get(paramName(fam, 2, "WPY"));
            if (wpy && changed(*wpy, g.rateMax) && stepNo == 0) rows.push_back({grp, paramName(fam, 2, "WPY"), "waypoint yaw rate (deg/s)", "the same as the rate limit", *wpy, g.rateMax});
          } else {
            add("FLTT", cur[A].FLTT, g.FLTT, "target filter (Hz)", "half the gyro filter");
            add("FLTD", cur[A].FLTD, g.FLTD, "D filter (Hz)", "half the gyro filter");
          }
        }
        if (G.axes[0] == 0) {
          if (std::fabs(cur[0].gyroHz - nx[0].gyroHz) > 0.01)
            rows.push_back({"filters", "INS_GYRO_FILTER", "gyro low-pass (Hz)", "one filter for every axis: roll and pitch are designed for it together (reboot)", cur[0].gyroHz, nx[0].gyroHz});
          if (std::fabs(cur[0].inputTC - nx[0].inputTC) > 0.005)
            rows.push_back({"feel", paramName(fam, 0, "INPUT_TC"), "stick smoothing (s)", "softer stick shaping for a slow vehicle", cur[0].inputTC, nx[0].inputTC});
          nx[2].gyroHz = nx[0].gyroHz;
        }
        if (rows.empty()) break;
        // the values as the .param file writes them (no float32 / binary-fraction noise)
        for (Row& r : rows) {
          if (std::isfinite(r.from)) r.from = std::stod(core::formatParamValue(r.name, r.from));
          r.to = std::stod(core::formatParamValue(r.name, r.to));
        }
        stepRows.push_back(rows);
        stepTitles.push_back(G.title);
        stepGains.push_back(nx);
        cur = nx;
        if (!limited) break;
      }
    }
    for (std::size_t s = 0; s < stepRows.size(); ++s) {
      const auto& rows = stepRows[s];
      const auto& g = stepGains[s];
      Json pred = Json::object();
      for (int a = 0; a < 3; ++a)
        if (models[static_cast<std::size_t>(a)].ok)
          for (const Row& r : rows)
            if (r.group == kAxisNames[a] || (a < 2 && (r.group == "filters" || r.group == "feel"))) {
              pred.set(kAxisNames[a], a == 2 ? predictYaw(g[2], models[2]) : predictRp(g[static_cast<std::size_t>(a)], models[static_cast<std::size_t>(a)]));
              break;
            }
      bool reboot = false;
      for (const Row& r : rows) reboot = reboot || r.name == "INS_GYRO_FILTER" || core::isRebootRequired(r.name);
      const std::string title = "Step " + std::to_string(s + 1) + ": " + stepTitles[s];
      Json card = Json::array();
      card.push("Load the file in Mission Planner: CONFIG > Full Parameter List > Load from file > Write Params. (On a QuadPlane the Extended Tuning tab shows the fixed-wing settings: the Q_ values are only in the Full Parameter List.)");
      if (reboot) card.push("Reboot the autopilot after writing (INS_GYRO_FILTER takes effect only after a reboot).");
      card.push("Keep the UNDO file at hand: it holds the values you flew before.");
      card.push("Take off in the mode you normally use, hover low (2-3 m) and leave the sticks centred for 30 s.");
      card.push("Then give gentle 1/3-stick inputs, a few seconds each way, on each axis.");
      card.push("Abort (land, load the UNDO file) if it wobbles faster than about once per second, the motors pulse, or the heading swings more than 45 deg.");
      card.push("Don't chase an oscillation with the stick: centre the sticks and land.");
      card.push("Bring the log back and use \"Check my next flight\".");
      const std::string head = title + " - " + std::string(models[2].ok || models[0].ok || models[1].ok ? "model-based proposal" : "proposal");
      const Json ident = Json(logIdentityText(Json::obj({{"file", last.name}, {"params", Json::obj({{"SYSID_THISMAV", sysid}})}, {"firmware", last.firmware}})));
      const ParamFileOut pf = paramFileChecked(rowsJson(rows, false), {head, "Proposals only: check the values before writing."}, ident, true);
      const ParamFileOut uf = paramFileChecked(rowsJson(rows, true), {"UNDO for " + title + ": the values as flown before this step."}, ident, false);
      Json rj = Json::array();
      for (const Row& r : rows)
        rj.push(Json::obj({{"group", r.group}, {"name", r.name}, {"label", r.label}, {"from", r.from}, {"to", r.to}, {"reason", r.reason},
                           {"text", core::formatParamValue(r.name, r.to)}, {"fromText", std::isfinite(r.from) ? Json(core::formatParamValue(r.name, r.from)) : Json()}}));
      steps.push(Json::obj({{"n", static_cast<int>(s + 1)}, {"title", title}, {"rows", rj}, {"predicted", pred}, {"reboot", reboot}, {"testCard", card},
                            {"paramText", pf.text}, {"undoText", uf.text}, {"paramErrors", pf.errors}, {"paramWarnings", pf.warnings},
                            {"jump", std::all_of(rows.begin(), rows.end(), [&](const Row& r) {
                               for (int a = 0; a < 3; ++a)
                                 if (r.group == kAxisNames[a]) return static_cast<bool>(valid[static_cast<std::size_t>(a)]);
                               return true;
                             })}}));
    }
  }
  res.set("steps", steps);

  // ---------------------------------------------------------------------------------- 10. next-flight check
  if (opts.plan.isObject()) {
    Json ck = Json::object();
    const Json& plan = opts.plan;
    if (!(plan["vehicle"]["sysid"] == sysid)) ck.set("warning", "The plan was made for system id " + plan["vehicle"]["sysid"].dump() + ", this flight is system id " + sysid.dump() + ".");
    // which step was flown: the latest step whose values are in effect, or whose parameters moved away from the values
    // the plan started from (applied with other values)
    int stepFound = 0;
    bool asProposed = false;
    Json diffs = Json::array();
    std::map<std::string, double> baseline;   // the value each parameter had when the plan was made
    if (plan["steps"].isArray())
      for (const auto& st : plan["steps"].arr())
        for (const auto& r : st["rows"].arr())
          if (!baseline.count(r["name"].strOr("")) && r["from"].isNumber()) baseline[r["name"].strOr("")] = r["from"].num();
    if (plan["steps"].isArray())
      for (const auto& st : plan["steps"].arr()) {
        int hit = 0, moved = 0, n = 0;
        Json d = Json::array();
        for (const auto& r : st["rows"].arr()) {
          const auto v = flownP.get(r["name"].strOr(""));
          if (!v) continue;
          ++n;
          const double to = r["to"].numOr(0);
          const double from = baseline.count(r["name"].strOr("")) ? baseline[r["name"].strOr("")] : std::nan("");
          const bool same = std::fabs(*v - to) <= 0.02 * std::max(std::fabs(to), 1e-3) + 1e-6;
          const bool away = std::isfinite(from) && std::fabs(*v - from) > 0.02 * std::max(std::fabs(from), 1e-3) + 1e-6;
          if (same) ++hit;
          else d.push(Json::obj({{"name", r["name"]}, {"proposed", to}, {"flown", *v}}));
          if (same || away) ++moved;
        }
        if (n && (hit * 10 >= 6 * n || moved * 10 >= 6 * n)) {
          stepFound = static_cast<int>(st["n"].numOr(0));
          asProposed = hit == n;
          diffs = d;
        }
      }
    ck.set("stepFlown", stepFound);
    ck.set("asProposed", asProposed);
    ck.set("differences", diffs);
    // predicted (saved model, the gains actually flown) against what the flight shows
    Json axc = Json::object();
    AxisModel pyaw = AxisModel::fromJson(plan["models"]["yaw"]);
    if (pyaw.ok) {
      // the saved model with this flight's torque trim (the trim is a condition of the day, not of the gains)
      double thisTrim = 0;
      for (const auto& [h, v] : yawTrims) thisTrim += v;
      if (!yawTrims.empty()) {
        thisTrim /= static_cast<double>(yawTrims.size());
        const double shift = thisTrim - pyaw.trim;
        for (auto& p : pyaw.set) p.trim += shift;
      }
      const Json pr = predictYaw(flown[2], pyaw);
      Vec sw;
      for (const Hover& h : hovers) {
        const double s = logYawSwing(flights[h.flight], h);
        if (std::isfinite(s)) sw.push_back(s);
      }
      const double lo = pr["swingDeg"][0].numOr(0), hi = pr["swingDeg"][1].numOr(0);
      const double flownSwing = sw.empty() ? std::nan("") : median(sw);
      const double tol = std::max(3.0, 0.2 * lo);
      const bool inRange = std::isfinite(flownSwing) && flownSwing >= lo - tol && flownSwing <= hi + std::max(3.0, 0.2 * hi);
      const bool better = std::isfinite(flownSwing) && flownSwing < lo - tol;
      Json swj = Json::array();
      for (double s : sw) swj.push(std::round(s));
      axc.set("yaw", Json::obj({{"predictedSwingDeg", pr["swingDeg"]}, {"flownSwingDeg", std::isfinite(flownSwing) ? Json(std::round(flownSwing)) : Json()},
                                {"perHover", swj}, {"confirmed", inRange}, {"betterThanPredicted", better}, {"trimUsed", r2(yawTrims.empty() ? pyaw.trim : thisTrim)},
                                {"text", "Heading swing after lift-off: predicted " + num(lo, 0) + "-" + num(hi, 0) + " deg for the gains flown (with this flight's torque trim " +
                                             num(yawTrims.empty() ? pyaw.trim : thisTrim, 2) + "), flown " + num(flownSwing, 0) + " deg: " +
                                             (inRange ? "prediction confirmed." : better ? "better than predicted (the model is on the safe side)." : "worse than predicted: the model is re-fitted from this flight.")}}));
    }
    for (int a = 0; a < 2; ++a) {
      const AxisModel pm = AxisModel::fromJson(plan["models"][kAxisNames[a]]);
      if (!pm.ok) continue;
      const Json pr = predictRp(flown[static_cast<std::size_t>(a)], pm);
      const bool expect = pr["oscillationExpected"].asBool();
      const bool seen = osc[static_cast<std::size_t>(a)].found;
      axc.set(kAxisNames[a], Json::obj({{"predictedPmDeg", pr["pmWorstDeg"]}, {"oscillationExpected", expect}, {"oscillationSeen", seen}, {"confirmed", expect == seen},
                                        {"text", std::string(kAxisNames[a]) + ": the model " + (expect ? "expected" : "did not expect") + " a low-frequency wobble with the gains flown (worst phase margin " +
                                                     num(pr["pmWorstDeg"].numOr(0), 0) + " deg); the flight shows " +
                                                     (seen ? "one at " + num(osc[static_cast<std::size_t>(a)].fHz, 2) + " Hz (+-" + num(osc[static_cast<std::size_t>(a)].angleAmpDeg, 1) + " deg)" : "none (well damped)") +
                                                     (expect == seen ? ": confirmed." : ": not as predicted.")}}));
    }
    ck.set("axes", axc);
    ck.set("next", steps.size() ? "The next step is below (re-fitted on this flight)." : "Stop here: the model has nothing better to propose for these axes.");
    res.set("check", ck);
  }

  // ---------------------------------------------------------------------------------- 11. plan to save
  {
    Json mj = Json::object();
    for (int a = 0; a < 3; ++a) {
      const auto A = static_cast<std::size_t>(a);
      // a validated new model replaces the saved one; otherwise the saved one is kept
      if (models[A].ok && valid[A]) mj.set(kAxisNames[a], models[A].toJson());
      else if (opts.plan.isObject() && opts.plan["models"][kAxisNames[a]].isObject()) mj.set(kAxisNames[a], opts.plan["models"][kAxisNames[a]]);
      else if (models[A].ok) mj.set(kAxisNames[a], models[A].toJson());
    }
    Json ps = Json::array();
    for (const auto& s : steps.arr()) ps.push(Json::obj({{"n", s["n"]}, {"title", s["title"]}, {"rows", s["rows"]}, {"predicted", s["predicted"]}}));
    char date[32] = "";
    const std::time_t now = std::time(nullptr);
    std::strftime(date, sizeof date, "%Y-%m-%d %H:%M", std::localtime(&now));
    Json src = Json::array();
    for (const auto& f : flights) src.push(f.name);
    res.set("plan", Json::obj({{"version", 1}, {"created", date}, {"vehicle", res["vehicle"]}, {"sources", src}, {"models", mj}, {"steps", ps},
                               {"vibration", vib.size() ? vib : (opts.plan.isObject() ? opts.plan["vibration"] : Json::object())}}));
  }

  // ---------------------------------------------------------------------------------- 12. summary in plain words
  summary.push(std::to_string(hovers.size()) + " hover(s) found in " + std::to_string(flights.size()) + " flight(s).");
  if (!heavy) summary.push("The vehicle does not look heavy or slow on any axis: the normal analysis applies; no model-based proposal.");
  for (int a = 2; a >= 0; --a) {
    const auto A = static_cast<std::size_t>(a);
    std::string s = std::string(kAxisNames[a]) + ": ";
    if (!models[A].ok) s += "no model (" + models[A].reason + ")";
    else {
      s += "model from " + models[A].method + " (control power " + num(models[A].set[0].a, a == 2 ? 2 : 1) + " rad/s/s per unit), ";
      s += valid[A] ? "the replay matches the flights" : "NOT confirmed by the replay: steps limited to x2";
      if (designed && design.axes[A].ok)
        s += "; design: rate P " + num(design.axes[A].g.P, 3) + ", I " + num(design.axes[A].g.I, 3) + (a < 2 ? ", D " + num(design.axes[A].g.D, 4) : std::string()) + ", angle P " +
             num(design.axes[A].g.angleP, 2) + " (worst phase margin " + num(design.axes[A].wm.pmDeg, 0) + " deg)";
      else if (designed) s += "; no design: " + design.axes[A].reason;
    }
    summary.push(s);
  }
  if (!steps.size() && heavy) summary.push("No step: the flown values already match the design.");
  for (const auto& n : opts.userNotes) notesJ.push(n);
  res.set("issues", issues);
  res.set("summary", summary);
  res.set("notes", notesJ);
  return res;
}

// ------------------------------------------------------------------------------------------------- check my values

double exactValue(const std::string& name, double value) { return std::stod(core::formatParamValue(name, value)); }

Json evaluateSettings(const Json& result, const Json& rows) {
  const Family fam = core::parseFamily(result["vehicle"]["family"].strOr("vtol")).value_or(Family::Vtol);
  log::Params base;
  if (result["flownParams"].isObject())
    for (const auto& [k, v] : result["flownParams"].items())
      if (v.isNumber()) base.set(k, v.num());
  log::Params p = base;
  Json rowsOut = Json::array();
  bool allOk = true;
  std::vector<std::string> names;
  for (const auto& r : rows.isArray() ? rows.arr() : Json::Array{}) {
    const std::string name = r["name"].strOr("");
    double v = r["value"].isNumber() ? r["value"].num() : std::nan("");
    if (r["value"].isString()) {
      try {
        std::size_t used = 0;
        const std::string s = r["value"].str();
        v = std::stod(s, &used);
        if (used != s.size()) v = std::nan("");
      } catch (const std::exception&) {
        v = std::nan("");
      }
    }
    Json errors = Json::array(), warnings = Json::array();
    std::string text;
    if (name.empty() || !std::isfinite(v)) {
      errors.push("is not a number");
    } else {
      v = exactValue(name, v);
      text = core::formatParamValue(name, v);
      const core::ParamCheck c = core::validateParam(name, Json(v));
      for (const auto& e : c.errors) errors.push(e);
      for (const auto& w : c.warnings) warnings.push(w);
      if (!base.has(name)) warnings.push("is not among the settings of the analysed flights: the model cannot judge it");
      p.set(name, v);
      names.push_back(name);
    }
    const bool ok = errors.size() == 0;
    allOk = allOk && ok;
    rowsOut.push(Json::obj({{"name", name}, {"value", std::isfinite(v) ? Json(v) : Json()}, {"text", text}, {"ok", ok}, {"errors", errors}, {"warnings", warnings}}));
  }
  Json axes = Json::object();
  for (int a = 0; a < 3; ++a) {
    // which rows act on this axis (the gyro filter and stick shaping are shared by roll and pitch)
    bool touched = false;
    for (const auto& n : names) {
      for (const char* k : {"P", "I", "D", "FF", "IMAX", "FLTT", "FLTE", "FLTD", "ANG", "ACC", "ACC_OLD", "RMAX"})
        if (n == paramName(fam, a, k)) touched = true;
      if (a < 2 && (n == "INS_GYRO_FILTER" || n == paramName(fam, a, "INPUT_TC"))) touched = true;
      if (a == 2 && (n == paramName(fam, 2, "PLT_RATE") || n == paramName(fam, 2, "WPY"))) touched = true;
    }
    const AxisModel m = AxisModel::fromJson(result["plan"]["models"][kAxisNames[a]]);
    Json ax = Json::obj({{"touched", touched}});
    if (!m.ok) {
      ax.set("pass", Json());
      ax.set("why", Json(Json::Array{Json(std::string("no model of this axis: the values can only be range-checked"))}));
      axes.set(kAxisNames[a], ax);
      continue;
    }
    const cc::Gains g = gainsFromParams(p, fam, a), cur = gainsFromParams(base, fam, a);
    Json tests = Json::array();
    bool pass = true;
    auto test = [&](bool ok, bool hard, const std::string& text) {
      tests.push(Json::obj({{"ok", ok}, {"hard", hard}, {"text", text}}));
      if (hard && !ok) pass = false;
    };
    if (a == 2) {
      ax.set("predicted", predictYaw(g, m));
      const cc::Plant& n = m.set[0];
      double zMin = 1;
      bool stable = true;
      for (const auto& pl : m.set) {
        const cc::Poles po = cc::slowAxisPoles(pl.a, pl.c, g.P, g.I, g.angleP);
        zMin = std::min(zMin, po.zeta);
        stable = stable && po.stable;
      }
      const double zNom = cc::slowAxisPoles(n.a, n.c, g.P, g.I, g.angleP).zeta;
      test(stable, true, stable ? "stable on every plant of the model" : "UNSTABLE on at least one plant of the model");
      test(zNom >= 0.35, true, "slow-mode damping on the nominal plant " + num(zNom, 2) + " (needs 0.35 or more)");
      test(zMin >= 0.2, true, "slow-mode damping on the worst plant " + num(zMin, 2) + " (needs 0.2 or more)");
      test(g.IMAX >= std::fabs(m.trimMax) + 0.15, true, "IMAX " + num(g.IMAX, 2) + " against the torque trim " + num(std::fabs(m.trimMax), 2) + " (needs 0.15 to spare)");
      const double strong = n.a * (m.umax + std::fabs(m.trim)) * 57.29577951308232;
      if (g.accMax > 0) test(g.accMax <= 1.3 * strong, false, "acceleration limit " + num(g.accMax, 0) + " deg/s/s; the vehicle can do about " + num(strong, 0));
      if (g.rateMax > 0 && g.accMax > 0)
        test(g.rateMax * g.rateMax / (2 * g.accMax) <= 20, false, "a " + num(g.rateMax, 0) + " deg/s turn stops within " + num(g.rateMax * g.rateMax / (2 * g.accMax), 0) + " deg");
    } else {
      const Json pr = predictRp(g, m);
      ax.set("predicted", pr);
      const double pm = pr["pmWorstDeg"].numOr(-99), gm = pr["gmWorstDb"].isNumber() ? pr["gmWorstDb"].num() : 99, ms = pr["ms"].numOr(99);
      test(pm >= 40, true, "worst phase margin " + num(pm, 0) + " deg (needs 40 or more)");
      test(gm >= 10, true, "worst gain margin " + (gm >= 98 ? std::string("large") : num(gm, 1) + " dB") + " (needs 10 dB or more)");
      test(ms <= 1.8, true, "sensitivity peak " + num(ms, 2) + " (needs 1.8 or less)");
      Vec nf;
      if (result["noisePeaksHz"].isArray())
        for (const auto& f : result["noisePeaksHz"].arr()) nf.push_back(f.numOr(0));
      if (nf.empty()) nf = {1.25 * cur.gyroHz, 1.85 * cur.gyroHz};
      double nr = 0;
      for (double f : nf) {
        const double c = cc::noiseGain(cur, f);
        if (c > 0) nr = std::max(nr, cc::noiseGain(g, f) / c);
      }
      test(nr <= 1.15, true, "motor noise from the gyro x" + num(nr, 2) + " of what is flown (needs 1.15 or less)");
      const double cap = m.set[0].a * std::max(0.05, m.umax - std::fabs(m.trim)) * 57.29577951308232;
      if (g.accMax > 0) test(g.accMax <= cap, false, "acceleration limit " + num(g.accMax, 0) + " deg/s/s; the vehicle can do about " + num(cap, 0));
    }
    ax.set("pass", pass);
    ax.set("tests", tests);
    axes.set(kAxisNames[a], ax);
  }
  return Json::obj({{"ok", allOk}, {"rows", rowsOut}, {"axes", axes}});
}

}  // namespace aptuner::engine::heavy
