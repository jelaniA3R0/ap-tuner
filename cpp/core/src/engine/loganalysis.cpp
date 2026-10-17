#include "aptuner/engine/loganalysis.hpp"

#include <algorithm>
#include <cmath>
#include <regex>

#include "aptuner/core/jsnum.hpp"
#include "aptuner/engine/issues.hpp"
#include "aptuner/engine/notch.hpp"
#include "aptuner/engine/vehicle.hpp"

namespace aptuner::engine {

using namespace aptuner::core;

namespace {

constexpr double kDegPerRad = 57.2957795;

/** nearest-sample lookup on a time-indexed table (queries must be non-decreasing) */
struct Sampler {
  const Vec& tUs;
  const Vec& vals;
  std::size_t j = 0;
  double operator()(double tSec) {
    const double t = tSec * 1e6;
    while (j + 1 < tUs.size() && tUs[j + 1] <= t) j++;
    return vals[j];
  }
};

const log::MsgTable* thrTable(const log::LogData& log, Family family) {
  const log::MsgTable* q = log.msg("QTUN");
  if (family == Family::Vtol && q) return q;
  return log.msg("CTUN");
}

bool isQMode(double m) { return m >= 17 && m <= 26 && m == std::floor(m); }

/**
 * Airborne mask on the data grid (copter / VTOL): throttle out above 60% of the typical hover throttle,
 * without the first 3 s after lift-off and the last 2 s before touch-down (ground contact, ground effect).
 */
std::vector<std::uint8_t> airborneMask(const log::MsgTable& ct, const AxisData& d) {
  const Vec& tUs = *ct.col("TimeUS");
  const Vec& thO = *ct.col("ThO");
  const double tA = d.t0, tB = d.t0 + static_cast<double>(d.n - 1) / d.fs;
  Vec thr;
  for (std::size_t i = 0; i < ct.n; i++) {
    const double ts = tUs[i] / 1e6;
    if (ts >= tA && ts <= tB && thO[i] > 0.15) thr.push_back(thO[i]);
  }
  if (thr.size() < 10) return {};
  std::sort(thr.begin(), thr.end());
  const double lim = std::max(0.15, 0.6 * thr[thr.size() / 2]);
  Sampler get{tUs, thO};
  std::vector<std::uint8_t> m(d.n);
  for (std::size_t i = 0; i < d.n; i++) m[i] = get(d.t0 + static_cast<double>(i) / d.fs) > lim ? 1 : 0;
  std::vector<std::uint8_t> out(d.n, 0);
  const long long a = static_cast<long long>(jsRound(3 * d.fs)), b = static_cast<long long>(jsRound(2 * d.fs));
  std::size_t i = 0;
  while (i < d.n) {
    if (!m[i]) {
      i++;
      continue;
    }
    std::size_t j = i;
    while (j < d.n && m[j]) j++;
    for (long long k = static_cast<long long>(i) + a; k < static_cast<long long>(j) - b; k++) out[static_cast<std::size_t>(k)] = 1;
    i = j;
  }
  return out;
}

struct ModeSeg {
  double t0, t1, mode;
};

std::vector<ModeSeg> modeSegments(const log::LogData& log, double tEnd) {
  const log::MsgTable* m = log.msg("MODE");
  std::vector<ModeSeg> out;
  if (!m || !m->n || !m->col("TimeUS") || !m->col("Mode")) return out;
  const Vec& t = *m->col("TimeUS");
  const Vec& mode = *m->col("Mode");
  for (std::size_t i = 0; i < m->n; i++)
    out.push_back({t[i] / 1e6, i + 1 < m->n ? t[i + 1] / 1e6 : tEnd, mode[i]});
  return out;
}

Vec secondsOf(const Vec& tUs, std::size_t i0, std::size_t i1) {
  Vec t(i1 - i0);
  for (std::size_t i = i0; i < i1; i++) t[i - i0] = tUs[i] / 1e6;
  return t;
}

}  // namespace

const std::set<std::string>& logWant() {
  static const std::set<std::string> w{"ATT", "RATE", "PIDR", "PIDP", "PIDY", "PIQR", "PIQP", "PIQY", "IMU", "ISBH",
                                       "ISBD", "GYR",  "CTUN", "QTUN", "MODE", "AETR", "VIBE", "RCOU", "ESC", "FTN1",
                                       "FTN2", "ARSP", "ATUN", "ATRP", "QUIK", "ANG",  "PARM", "MSG"};
  return w;
}

Family detectLogFamily(const log::LogData& log) {
  std::string txt;
  for (std::size_t i = 0; i < log.text.size(); i++) {
    if (i) txt += ' ';
    txt += log.text[i].msg;
  }
  const auto& p = log.params;
  if (txt.find("ArduCopter") != std::string::npos) {
    std::string low = txt;
    std::transform(low.begin(), low.end(), low.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const bool heli = p.has("H_SV_MAN") || (low.find("heli") != std::string::npos && p.has("H_COL_MAX"));
    return heli ? Family::Heli : Family::Copter;
  }
  const log::MsgTable* piqr = log.msg("PIQR");
  if (txt.find("ArduPlane") != std::string::npos) return piqr && piqr->n > 500 ? Family::Vtol : Family::Plane;
  if (piqr) return Family::Vtol;
  return log.msg("RATE") ? Family::Copter : Family::Plane;
}

std::optional<AxisData> axisData(const log::LogData& log, Family family, Axis axis, const Window& win) {
  const FamilyDef& fam = familyDef(family);
  const int a = static_cast<int>(axis);
  const log::MsgTable* pid = log.msg(fam.logPid[a]);
  const log::MsgTable* rateMsg = log.msg("RATE") && fam.hasLogRate ? log.msg("RATE") : nullptr;
  auto rateOf = [](const log::MsgTable* m) {
    if (!m || m->n <= 200 || !m->col("TimeUS")) return 0.0;
    const Vec t = secondsOf(*m->col("TimeUS"), 0, std::min<std::size_t>(m->n, 2000));
    return estimateRate(t);
  };
  const double pidRate = rateOf(pid), rateRate = rateOf(rateMsg);
  Vec t;
  std::vector<std::pair<std::string, Vec>> cols;
  const bool usePid = pid && pid->n > 200 && pidRate >= rateRate * 0.5;
  if (usePid) {
    const auto need = [&](const char* k) -> const Vec& {
      static const Vec empty;
      const Vec* c = pid->col(k);
      return c ? *c : empty;
    };
    t = secondsOf(*pid->col("TimeUS"), 0, pid->n);
    const Vec &P = need("P"), &I = need("I"), &D = need("D"), &FF = need("FF");
    const Vec* DFF = pid->col("DFF");
    Vec u(pid->n);
    for (std::size_t i = 0; i < pid->n; i++) u[i] = P[i] + I[i] + D[i] + FF[i] + (DFF ? (*DFF)[i] : 0);
    // copter / VTOL PIDx logs are in rad/s, plane PIDx in deg/s: keep everything in deg/s
    const double k = family == Family::Plane ? 1 : kDegPerRad;
    Vec tar = need("Tar"), act = need("Act");
    if (k != 1) {
      for (auto& v : tar) v *= k;
      for (auto& v : act) v *= k;
    }
    cols = {{"target", tar}, {"actual", act}, {"u", u}, {"P", P}, {"I", I}, {"D", D}, {"FF", FF}};
    cols.emplace_back("srate", pid->col("SRate") ? *pid->col("SRate") : Vec(pid->n, 0.0));
    cols.emplace_back("dmod", pid->col("Dmod") ? *pid->col("Dmod") : Vec(pid->n, 1.0));
    const log::MsgTable* aetr = log.msg("AETR");
    if (family == Family::Plane && aetr && aetr->col("SS")) {
      Sampler get{*aetr->col("TimeUS"), *aetr->col("SS")};
      Vec ss(pid->n);
      for (std::size_t i = 0; i < pid->n; i++) {
        const double v = get(t[i]);
        ss[i] = (v != 0 && !std::isnan(v)) ? v : 1;
      }
      cols.emplace_back("ss", std::move(ss));
    }
  } else if (rateMsg) {
    const auto& names = fam.logRate[static_cast<std::size_t>(a)];
    t = secondsOf(*rateMsg->col("TimeUS"), 0, rateMsg->n);
    for (int k = 0; k < 3; k++) {
      const Vec* c = rateMsg->col(names[static_cast<std::size_t>(k)]);
      if (!c) return std::nullopt;
      cols.emplace_back(k == 0 ? "target" : k == 1 ? "actual" : "u", *c);
    }
  } else {
    return std::nullopt;
  }
  // time window
  std::size_t i0 = 0, i1 = t.size();
  if (win.tStart)
    while (i0 < t.size() && t[i0] < *win.tStart) i0++;
  if (win.tEnd)
    while (i1 > i0 && t[i1 - 1] > *win.tEnd) i1--;
  if (i1 - i0 < 200) return std::nullopt;
  const Vec tt(t.begin() + static_cast<std::ptrdiff_t>(i0), t.begin() + static_cast<std::ptrdiff_t>(i1));
  std::vector<Vec> subs;
  for (auto& c : cols) subs.emplace_back(c.second.begin() + static_cast<std::ptrdiff_t>(i0), c.second.begin() + static_cast<std::ptrdiff_t>(i1));
  const double fsRaw = estimateRate(tt);
  const double fs = fsRaw >= 300 ? 400 : fsRaw >= 150 ? 200 : fsRaw >= 75 ? 100 : fsRaw >= 40 ? 50 : fsRaw >= 20 ? 25 : 10;
  std::vector<const Vec*> ptrs;
  for (auto& s : subs) ptrs.push_back(&s);
  const auto rs = resample(tt, ptrs, fs);
  if (!rs) return std::nullopt;
  AxisData d;
  d.fs = fs;
  d.t0 = rs->t0;
  d.n = rs->n;
  d.rawRate = fsRaw;
  for (std::size_t k = 0; k < cols.size(); k++) {
    const std::string& name = cols[k].first;
    Vec v = rs->cols[k];
    if (name == "target") d.target = std::move(v);
    else if (name == "actual") d.actual = std::move(v);
    else if (name == "u") d.u = std::move(v);
    else if (name == "P") d.P = std::move(v);
    else if (name == "I") d.I = std::move(v);
    else if (name == "D") d.D = std::move(v);
    else if (name == "FF") d.FF = std::move(v);
    else if (name == "srate") d.srate = std::move(v);
    else if (name == "dmod") d.dmod = std::move(v);
    else if (name == "ss") d.ss = std::move(v);
  }
  // plane: speed scaler from AETR.SS (mean of scaler^2 over the window)
  d.scaler2 = 1;
  const log::MsgTable* aetr = log.msg("AETR");
  if (family == Family::Plane && aetr && aetr->col("SS")) {
    const Vec& at = *aetr->col("TimeUS");
    const Vec& ss = *aetr->col("SS");
    double s = 0, c = 0;
    for (std::size_t i = 0; i < aetr->n; i++) {
      const double ts = at[i] / 1e6;
      if (ts >= tt.front() && ts <= tt.back() && ss[i] > 0) {
        s += ss[i] * ss[i];
        c++;
      }
    }
    if (c != 0) d.scaler2 = s / c;
  }
  // throttle for the notch operating point
  const log::MsgTable* ct = thrTable(log, family);
  if (ct && ct->col("ThO")) {
    const Vec& ctT = *ct->col("TimeUS");
    const Vec& thO = *ct->col("ThO");
    double s = 0, c = 0;
    for (std::size_t i = 0; i < ct->n; i++) {
      const double ts = ctT[i] / 1e6;
      if (ts >= tt.front() && ts <= tt.back()) {
        s += thO[i];
        c++;
      }
    }
    d.throttle = c != 0 ? s / c : 0;
    if (family != Family::Plane) d.airborne = airborneMask(*ct, d);
  }
  return d;
}

std::optional<std::pair<double, double>> flightWindow(const log::LogData& log, Family family) {
  const log::MsgTable* ct = thrTable(log, family);
  if (!ct || !ct->col("ThO") || !ct->n) return std::nullopt;
  const Vec& tUs = *ct->col("TimeUS");
  const Vec& thO = *ct->col("ThO");
  const double thr = family == Family::Plane ? 5 : 0.15;   // plane CTUN.ThO is in percent
  const double tEnd = tUs[ct->n - 1] / 1e6;
  std::vector<ModeSeg> segs;
  for (const auto& s : modeSegments(log, tEnd)) {
    const bool keep = family == Family::Vtol ? isQMode(s.mode) : family == Family::Plane ? !isQMode(s.mode) : true;
    if (keep) segs.push_back(s);
  }
  if (segs.empty()) segs.push_back({0, tEnd, 0});
  std::optional<std::pair<double, double>> best;
  for (const auto& r : segs) {
    std::optional<double> t0, t1;
    for (std::size_t i = 0; i < ct->n; i++) {
      const double ts = tUs[i] / 1e6;
      if (ts < r.t0 || ts > r.t1) continue;
      if (thO[i] > thr) {
        if (!t0) t0 = ts;
        t1 = ts;
      }
    }
    if (t0 && (!best || *t1 - *t0 > best->second - best->first)) best = std::make_pair(*t0, *t1);
  }
  return best;
}

Json iStats(const log::LogData& log, Family family, Axis axis, const AxisData& d) {
  const FamilyDef& fam = familyDef(family);
  const log::MsgTable* pid = log.msg(fam.logPid[static_cast<int>(axis)]);
  if (!pid || !pid->col("I")) return Json();
  const Vec& tUs = *pid->col("TimeUS");
  const Vec& I = *pid->col("I");
  Vec v;
  double m = 0;
  for (std::size_t i = 0; i < pid->n; i++) {
    const double kf = jsRound((tUs[i] / 1e6 - d.t0) * d.fs);
    if (kf < 0 || kf >= static_cast<double>(d.n)) continue;
    const std::size_t k = static_cast<std::size_t>(kf);
    if (!d.airborne.empty() && !d.airborne[k]) continue;
    v.push_back(std::fabs(I[i]));
    m += I[i];
  }
  if (v.size() < 20) return Json();
  std::sort(v.begin(), v.end());
  const double n = static_cast<double>(v.size());
  return Json::obj({{"mean", m / n},
                    {"p95", v[static_cast<std::size_t>(std::floor(n * 0.95))]},
                    {"max", v.back()},
                    {"n", v.size()}});
}

Json motorStats(const log::LogData& log, const log::Params& params, Family family, const Window& win) {
  const log::MsgTable* rc = log.msg("RCOU");
  if (!rc) return Json();
  const std::string q = family == Family::Vtol ? "Q_M_" : "MOT_";
  struct Chan {
    const Vec* col = nullptr;
    int n = 0;
  };
  std::vector<Chan> chans(9);
  for (int n = 1; n <= 14; n++) {
    const double fn = params.getOr("SERVO" + std::to_string(n) + "_FUNCTION", std::nan(""));
    const Vec* c = rc->col("C" + std::to_string(n));
    if (fn >= 33 && fn <= 40 && c) chans[static_cast<std::size_t>(fn - 32)] = {c, n};
  }
  std::vector<int> motors;
  for (int k = 1; k <= 8; k++)
    if (chans[static_cast<std::size_t>(k)].col) motors.push_back(k);
  if (motors.empty()) return Json();
  const std::string s0 = "SERVO" + std::to_string(chans[static_cast<std::size_t>(motors[0])].n);
  const double pmin = params.truthyOr(q + "PWM_MIN", params.truthyOr(s0 + "_MIN", 1000));
  const double pmax = params.truthyOr(q + "PWM_MAX", params.truthyOr(s0 + "_MAX", 2000));
  const double top = pmin + (pmax - pmin) * params.getOr(q + "SPIN_MAX", 0.95) - 5;
  const log::MsgTable* ct = thrTable(log, family);
  std::optional<double> hov;
  if (ct && ct->col("ThO")) {
    Vec v;
    for (double x : *ct->col("ThO"))
      if (x > 0.15) v.push_back(x);
    std::sort(v.begin(), v.end());
    if (!v.empty()) hov = v[v.size() / 2];
  }
  std::optional<Sampler> thr;
  if (ct && ct->col("ThO")) thr.emplace(Sampler{*ct->col("TimeUS"), *ct->col("ThO")});
  std::vector<double> sum(9, 0.0);
  double c = 0, pinned = 0, maxPwm = 0;
  const Vec& rt = *rc->col("TimeUS");
  for (std::size_t i = 0; i < rc->n; i++) {
    const double ts = rt[i] / 1e6;
    if (win.tStart && (ts < *win.tStart || !(ts <= win.tEnd.value_or(std::nan(""))))) continue;
    if (thr && hov && *hov != 0 && (*thr)(ts) < 0.6 * *hov) continue;
    bool any = false;
    for (int k : motors) {
      const double v = (*chans[static_cast<std::size_t>(k)].col)[i];
      sum[static_cast<std::size_t>(k)] += v;
      if (v >= top) any = true;
      if (v > maxPwm) maxPwm = v;
    }
    if (any) pinned++;
    c++;
  }
  if (c < 20) return Json();
  Json mean = Json::object();
  Json motorsJ = Json::array();
  for (int k : motors) {
    mean.set(std::to_string(k), jsRoundFixed(sum[static_cast<std::size_t>(k)] / c, 0));
    motorsJ.push(k);
  }
  Json out = Json::obj({{"motors", motorsJ},
                        {"meanPwm", mean},
                        {"pinnedTopPct", 100 * pinned / c},
                        {"topPwm", jsRoundFixed(top, 0)},
                        {"maxPwm", maxPwm},
                        {"pwmMin", pmin},
                        {"pwmMax", pmax}});
  // ArduPilot quad X / +: motors 1 and 2 spin counter-clockwise, 3 and 4 clockwise
  const auto cls = params.get(family == Family::Vtol ? "Q_FRAME_CLASS" : "FRAME_CLASS");
  const auto typ = params.get(family == Family::Vtol ? "Q_FRAME_TYPE" : "FRAME_TYPE");
  if (cls && *cls == 1 && typ && (*typ == 0 || *typ == 1) && mean.has("1") && mean.has("2") && mean.has("3") &&
      mean.has("4")) {
    out.set("ccwMean", (mean["1"].num() + mean["2"].num()) / 2);
    out.set("cwMean", (mean["3"].num() + mean["4"].num()) / 2);
  }
  return out;
}

Json analyzeLog(const log::LogData& log, const std::string& file, const AnalyzeLogOptions& opt) {
  const Family fam = opt.family ? *opt.family : detectLogFamily(log);
  const log::Params& params = log.params;
  const auto fw = flightWindow(log, fam);
  Window win;
  win.tStart = opt.tStart ? opt.tStart : (fw ? std::optional<double>(fw->first) : std::nullopt);
  win.tEnd = opt.tEnd ? opt.tEnd : (fw ? std::optional<double>(fw->second) : std::nullopt);
  // firmware: the first "Ardu..." text message, else the first text message
  Json firmware;
  static const std::regex fwRe("Ardu(Copter|Plane|Rover|Sub)|^Ardu", std::regex::icase);
  for (const auto& t : log.text)
    if (std::regex_search(t.msg, fwRe)) {
      firmware = t.msg;
      break;
    }
  if (firmware.isNull() && !log.text.empty()) firmware = log.text[0].msg;
  Json paramsJ = Json::object();
  for (const auto& [k, v] : params.items()) paramsJ.items().emplace_back(k, Json(v));
  Json counts = Json::object();
  for (const auto& [k, v] : log.counts) counts.items().emplace_back(k, Json(v));
  Json out = Json::obj({{"file", file}, {"family", familyName(fam)}});
  if (!firmware.isNull()) out.set("firmware", firmware);
  out.set("params", paramsJ);
  out.set("paramChanges", log.changes.size());
  out.set("flightWindow", Json::obj({{"tStart", win.tStart ? Json(*win.tStart) : Json()}, {"tEnd", win.tEnd ? Json(*win.tEnd) : Json()}}));
  out.set("axes", Json::object());
  out.set("counts", counts);
  const Json motors = fam != Family::Plane ? motorStats(log, params, fam, win) : Json();
  if (motors.isObject()) out.set("motors", motors);
  for (const Axis axis : kAxes) {
    auto d = axisData(log, fam, axis, win);
    if (!d) {
      out.at("axes").set(axisName(axis), Json::obj({{"error", "no rate-loop data for this axis in the log"}}));
      continue;
    }
    if (fam != Family::Plane) {
      d->motors = motors;
      d->iStats = iStats(log, fam, axis, *d);
    }
    AnalyzeArgs a;
    a.family = fam;
    a.axis = axis;
    a.data = &*d;
    a.params = &params;
    a.preset = opt.preset;
    a.gyroOp.throttle = d->throttle;
    a.scaler2 = d->scaler2;
    a.timeDomain = true;   // the hover-data fit is used only for log analysis: its result is reviewed before writing
    Json res = analyzeAxis(a);
    res.set("dataInfo", Json::obj({{"fs", d->fs},
                                   {"rawRateHz", d->rawRate},
                                   {"seconds", static_cast<double>(d->n) / d->fs},
                                   {"scaler2", d->scaler2},
                                   {"throttle", d->throttle ? Json(*d->throttle) : Json()}}));
    // time-series preview (decimated) for the UI
    const std::size_t step = std::max<std::size_t>(1, d->n / 4000);
    Json st = Json::array(), sTar = Json::array(), sAct = Json::array(), su = Json::array();
    for (std::size_t i = 0; i < d->n; i += step) {
      st.push(jsRoundFixed(d->t0 + static_cast<double>(i) / d->fs, 3));
      sTar.push(jsRoundFixed(d->target[i], 2));
      sAct.push(jsRoundFixed(d->actual[i], 2));
      su.push(jsRoundFixed(d->u[i], 4));
    }
    res.set("series", Json::obj({{"t", st}, {"target", sTar}, {"actual", sAct}, {"u", su}}));
    res.set("issues", axisIssues(res, fam));
    out.at("axes").set(axisName(axis), res);
  }
  out.set("noise", analyzeNoise(log, params, fam, win));
  return out;
}

Json analyzeLogFile(const std::string& path, const AnalyzeLogOptions& opt) {
  const log::LogData log = log::parseBinFile(path, logWant());
  return analyzeLog(log, path, opt);
}

}  // namespace aptuner::engine
