#include "aptuner/engine/loganalysis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <regex>
#include <stdexcept>

#include "aptuner/core/jsnum.hpp"
#include "aptuner/engine/issues.hpp"
#include "aptuner/engine/notch.hpp"
#include "aptuner/log/integrity.hpp"

namespace aptuner::engine {

using namespace aptuner::core;

namespace {

constexpr double kDegPerRad = 57.2957795;
/** values no real aircraft logs (corrupt records): rates in deg/s, controller terms in mixer units / degrees */
constexpr double kRateLimitDegs = 5000, kOutputLimit = 1000;
/** a gap longer than this is shortened on the analysis grid (the samples keep their logged times) */
constexpr double kMaxGapOnGridS = 2;

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

struct ModeSeg {
  double t0, t1, mode;
  std::optional<double> prev;
};

/** mode segments from MODE messages */
std::vector<ModeSeg> modeSegments(const log::LogData& log, double tEnd) {
  const log::MsgTable* m = log.msg("MODE");
  std::vector<ModeSeg> out;
  if (!m || !m->n || !m->col("TimeUS") || !m->col("Mode")) return out;
  const Vec& t = *m->col("TimeUS");
  const Vec& mode = *m->col("Mode");
  for (std::size_t i = 0; i < m->n; i++)
    out.push_back({t[i] / 1e6, i + 1 < m->n ? t[i + 1] / 1e6 : tEnd, mode[i],
                   i > 0 ? std::optional<double>(mode[i - 1]) : std::nullopt});
  return out;
}

const char* planeModeName(double m) {
  static const std::map<int, const char*> names{
      {0, "MANUAL"},       {1, "CIRCLE"},     {2, "STABILIZE"},  {3, "TRAINING"},    {4, "ACRO"},
      {5, "FBWA"},         {6, "FBWB"},       {7, "CRUISE"},     {8, "AUTOTUNE"},    {10, "AUTO"},
      {11, "RTL"},         {12, "LOITER"},    {13, "TAKEOFF"},   {14, "AVOID_ADSB"}, {15, "GUIDED"},
      {16, "INITIALISING"}, {17, "QSTABILIZE"}, {18, "QHOVER"},  {19, "QLOITER"},    {20, "QLAND"},
      {21, "QRTL"},        {22, "QAUTOTUNE"}, {23, "QACRO"},     {24, "THERMAL"},    {25, "LOITER_ALT_QLAND"},
      {26, "AUTOLAND"}};
  if (m != std::floor(m) || m < 0 || m > 100) return nullptr;
  const auto it = names.find(static_cast<int>(m));
  return it == names.end() ? nullptr : it->second;
}

// ------------------------------------------------------------------------------------------- reporting helpers

std::string f1(double v) { return jsToFixed(v, 1); }
std::string num(double v) { return jsToString(v); }
std::string plural(double n, const std::string& one, const std::string& many) { return num(n) + " " + (n == 1 ? one : many); }
std::string bytesText(double n) {
  return n >= 1048576 ? jsToFixed(n / 1048576, 1) + " MB" : n >= 1024 ? jsToFixed(n / 1024, 0) + " kB" : num(n) + " bytes";
}
std::string modeName(double m) {
  const char* s = planeModeName(m);
  return s ? std::string(s) : "mode " + num(m);
}
/** +(+x).toPrecision(4) as a template literal prints it (no value: NaN) */
std::string prec4(const std::optional<double>& x) {
  if (!x) return "NaN";
  if (!std::isfinite(*x)) return num(*x);
  return num(jsRoundPrecision(*x, 4));
}
std::string join(const std::vector<std::string>& parts, const std::string& sep) {
  std::string s;
  for (std::size_t i = 0; i < parts.size(); i++) s += (i ? sep : "") + parts[i];
  return s;
}

/** plain-language notes about the file and its time stamps (for every axis) */
std::vector<std::string> logNotes(const log::Integrity& integ, double loggerDropped,
                                  const std::map<std::string, log::TimeIssue>& timeIssues,
                                  const std::vector<std::string>& tables) {
  std::vector<std::string> out;
  if (loggerDropped > 0)
    out.push_back("The flight controller's logger reports " + plural(loggerDropped, "message", "messages") +
                  " it could not write (DSF.Dp): the log is incomplete (SD card too slow, too much logging, or a "
                  "simulator running too fast).");
  const double skipped = static_cast<double>(integ.skippedBytes);
  if (skipped > 0)
    out.push_back(bytesText(skipped) + " of the file could not be read (damaged or blank data) and " +
                  (skipped == 1 ? "was" : "were") + " skipped; reading picked up again after " +
                  plural(static_cast<double>(integ.resyncs), "damaged stretch", "damaged stretches") +
                  ". Any time missing there is left out.");
  if (integ.truncatedBytes > 0)
    out.push_back("The file ends in the middle of a record (the last " + bytesText(static_cast<double>(integ.truncatedBytes)) +
                  "): the log or its download was cut short.");
  if (integ.badFmtCount > 0) {
    const double n = static_cast<double>(integ.badFmtCount);
    std::vector<std::string> list;
    for (std::size_t i = 0; i < integ.badFmts.size() && i < 3; i++)
      list.push_back((integ.badFmts[i].name.empty() ? std::string("?") : integ.badFmts[i].name) + ": " + integ.badFmts[i].reason);
    out.push_back(plural(n, "message definition", "message definitions") + " in the file " + (n == 1 ? "is" : "are") +
                  " invalid and " + (n == 1 ? "was" : "were") + " ignored (" + join(list, "; ") + ").");
  }
  if (integ.fmtConflicts > 0)
    out.push_back(plural(static_cast<double>(integ.fmtConflicts), "message definition appears", "message definitions appear") +
                  " a second time with a different layout; the first one was used.");
  for (const std::string& name : tables) {
    const auto it = timeIssues.find(name);
    if (it == timeIssues.end()) continue;
    const log::TimeIssue& r = it->second;
    if (r.outOfOrder || r.duplicates) {
      std::vector<std::string> parts;
      if (r.outOfOrder) parts.push_back(plural(static_cast<double>(r.outOfOrder), "record", "records") + " with time stamps out of order");
      if (r.duplicates) parts.push_back(plural(static_cast<double>(r.duplicates), "repeated time stamp", "repeated time stamps"));
      out.push_back(name + ": " + join(parts, " and ") + " left out.");
    }
    if (r.reset)
      out.push_back(name + ": the log's clock jumps back from " + f1(r.reset->first) + " s to " + f1(r.reset->second) +
                    " s (two logs joined, or a reboot): only the part before the jump is used (" +
                    plural(static_cast<double>(r.droppedAfterReset), "record", "records") + " after it ignored).");
  }
  return out;
}

/** plain-language notes about one axis dataset: gaps, masked samples, ground time */
std::vector<std::string> axisNotes(const AxisData& d, const Window& win, std::optional<double> airS) {
  std::vector<std::string> out;
  const double winS = win.tStart ? win.tEnd.value_or(0) - *win.tStart : (gridTime(d, d.n - 1) - d.t0);
  std::vector<std::pair<double, double>> big, small;
  for (const auto& g : d.gaps) (g.second - g.first >= 1 ? big : small).push_back(g);
  for (std::size_t k = 0; k < big.size() && k < 3; k++) {
    const auto& g = big[k];
    out.push_back("Logging stopped for " + f1(g.second - g.first) + " s (" + f1(g.first) + "-" + f1(g.second) +
                  " s) inside the analysed window: that stretch is left out (nothing is filled in).");
  }
  if (big.size() > 3) out.push_back(num(static_cast<double>(big.size() - 3)) + " more logging gaps of 1 s or longer are left out as well.");
  if (!small.empty()) {
    double s = 0;
    for (const auto& g : small) s = s + g.second - g.first;
    std::size_t longestN = 0;
    for (const auto& x : d.segs) longestN = std::max(longestN, x.g1 - x.g0);
    const double longest = static_cast<double>(longestN) / d.fs;
    out.push_back("The " + d.source + " data have " + plural(static_cast<double>(small.size()), "short dropout", "short dropouts") +
                  " (" + f1(s) + " s in total, " + jsToFixed(100 * s / std::max(winS, 1e-9), 0) +
                  "% of the window) where samples are missing: the log could not keep up. They are left out; spectra and "
                  "models use only unbroken stretches (the longest is " + jsToFixed(longest, 1) + " s).");
  }
  if (d.nonFinite || d.implausible) {
    std::vector<std::string> parts;
    if (d.nonFinite) parts.push_back(plural(static_cast<double>(d.nonFinite), "sample was", "samples were") + " not a number (NaN / infinity)");
    if (d.implausible) parts.push_back(plural(static_cast<double>(d.implausible), "sample had", "samples had") + " impossible values (corrupt records)");
    out.push_back(join(parts, " and ") + ": left out.");
  }
  if (airS && winS > 0 && *airS < 0.9 * winS)
    out.push_back("Only " + f1(*airS) + " s of the " + f1(winS) + " s window " + (*airS == 1 ? "was" : "were") +
                  " flown (the rest was on the ground" + (d.gaps.empty() ? "" : " or not logged") +
                  "): every number here uses the flown part only.");
  return out;
}

/** plain-language notes about the window choice (tuning changes, transition) */
std::vector<std::string> windowNotes(const ChosenWindow* w) {
  std::vector<std::string> out;
  if (!w) return out;
  if (!w->cutBy.empty()) {
    std::vector<std::string> items;
    for (std::size_t i = 0; i < w->cutBy.size() && i < 4; i++) {
      const ParamChangeRec& c = w->cutBy[i];
      items.push_back(c.name + " " + prec4(c.from) + " -> " + prec4(c.to) + " at " + f1(c.t) + " s");
    }
    out.push_back("Tuning parameters changed during the flight (" + join(items, ", ") + (w->cutBy.size() > 4 ? ", ..." : "") +
                  "): only " + f1(w->tStart) + "-" + f1(w->tEnd) +
                  " s, flown with one set of values, is analysed, and the values in effect then are used.");
  }
  if (w->transition)
    out.push_back("The first " + num(kTransitionS) + " s after the switch from " + modeName(w->transition->first) + " to " +
                  modeName(w->transition->second) + " (the transition) are left out.");
  return out;
}

Json integrityJson(const log::Integrity& g) {
  Json bad = Json::array();
  for (const auto& b : g.badFmts) bad.push(Json::obj({{"type", b.type}, {"name", b.name}, {"reason", b.reason}}));
  return Json::obj({{"format", g.format.kind},
                    {"supported", g.format.supported},
                    {"message", g.format.supported ? Json() : Json(g.format.message)},
                    {"bytes", g.bytes},
                    {"skippedBytes", g.skippedBytes},
                    {"resyncs", g.resyncs},
                    {"truncatedBytes", g.truncatedBytes},
                    {"paddingBytes", g.paddingBytes},
                    {"badFmts", bad},
                    {"badFmtCount", g.badFmtCount},
                    {"fmtConflicts", g.fmtConflicts}});
}

Json countsJson(const log::LogData& log) {
  Json counts = Json::object();
  for (const auto& [k, v] : log.counts) counts.items().emplace_back(k, Json(v));
  return counts;
}

/** a result that refuses the file (not a DataFlash log, or empty), shaped like a normal result */
Json refusal(const std::string& file, Family fam, const log::LogData& log) {
  const std::string& msg = log.integrity.format.message;
  Json axes = Json::object();
  for (const Axis a : kAxes) axes.set(axisName(a), Json::obj({{"error", msg}}));
  Json integ = integrityJson(log.integrity);
  integ.set("timeIssues", Json::object());
  integ.set("notes", Json(Json::Array{Json(msg)}));
  return Json::obj({{"file", file},
                    {"family", familyName(fam)},
                    {"error", msg},
                    {"format", Json::obj({{"kind", log.integrity.format.kind}, {"supported", false}})},
                    {"params", Json::object()},
                    {"paramChanges", 0},
                    {"paramChangesAll", 0},
                    {"tuningChanges", Json::array()},
                    {"flightWindow", Json::obj({{"tStart", Json()}, {"tEnd", Json()}})},
                    {"integrity", integ},
                    {"axes", axes},
                    {"counts", countsJson(log)},
                    {"noise", Json::obj({{"available", false}, {"reason", msg}})}});
}

}  // namespace

const std::set<std::string>& logWant() {
  static const std::set<std::string> w{"ATT",  "RATE", "PIDR", "PIDP", "PIDY", "PIQR", "PIQP", "PIQY", "IMU",  "ISBH",
                                       "ISBD", "GYR",  "CTUN", "QTUN", "MODE", "AETR", "VIBE", "RCOU", "ESC",  "RPM",
                                       "FTN1", "FTN2", "ARSP", "ATUN", "ATRP", "QUIK", "ANG",  "DSF",  "PARM", "MSG"};
  return w;
}

const std::vector<std::string>& logTimed() {
  static const std::vector<std::string> t{"RATE", "PIDR", "PIDP", "PIDY", "PIQR", "PIQP", "PIQY", "IMU", "GYR",
                                          "CTUN", "QTUN", "MODE", "AETR", "RCOU", "ESC",  "RPM",  "FTN1"};
  return t;
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

// ------------------------------------------------------------------------------------------- parameters

bool isTuningParam(const std::string& name) {
  static const std::vector<std::string> prefixes{"ATC_RAT_",   "ATC_ANG_",  "ATC_ACC_",   "Q_A_RAT_",   "Q_A_ANG_",
                                                 "Q_A_ACC_",   "RLL_RATE_", "PTCH_RATE_", "YAW_RATE_",  "RLL2SRV_",
                                                 "PTCH2SRV_",  "INS_HNTCH_", "INS_HNTC2_", "MOT_SPIN_", "MOT_PWM_",
                                                 "Q_M_SPIN_",  "Q_M_PWM_"};
  static const std::set<std::string> names{"ATC_RATE_FF_ENAB", "ATC_INPUT_TC",    "Q_A_RATE_FF_ENAB", "Q_A_INPUT_TC",
                                           "INS_GYRO_FILTER",  "INS_GYRO_RATE",   "INS_FAST_SAMPLE",  "INS_RAW_LOG_OPT",
                                           "INS_LOG_BAT_OPT",  "SCHED_LOOP_RATE", "MOT_THST_EXPO",    "Q_M_THST_EXPO",
                                           "FRAME_CLASS",      "FRAME_TYPE",      "Q_FRAME_CLASS",    "Q_FRAME_TYPE",
                                           "SCALING_SPEED"};
  if (names.count(name)) return true;
  for (const auto& p : prefixes)
    if (name.compare(0, p.size(), p) == 0) return true;
  // SERVOn_FUNCTION
  const std::string a = "SERVO", b = "_FUNCTION";
  if (name.size() <= a.size() + b.size() || name.compare(0, a.size(), a) != 0 ||
      name.compare(name.size() - b.size(), b.size(), b) != 0)
    return false;
  for (std::size_t i = a.size(); i < name.size() - b.size(); i++)
    if (name[i] < '0' || name[i] > '9') return false;
  return true;
}

ParamChanges parameterChanges(const log::LogData& log) {
  std::map<std::string, double> last;
  for (const auto& [k, v] : log.params.items()) last[k] = v;
  ParamChanges out;
  for (const auto& c : log.changes) {
    const auto it = last.find(c.name);
    out.all.push_back({c.t, c.name, it == last.end() ? std::nullopt : std::optional<double>(it->second), c.val});
    last[c.name] = c.val;
  }
  for (const auto& c : out.all)
    if (isTuningParam(c.name)) out.tuning.push_back(c);
  return out;
}

log::Params paramsAt(const log::LogData& log, std::optional<double> t) {
  log::Params p;
  for (const auto& [k, v] : log.params.items()) p.set(k, v);
  if (!t) return p;
  for (const auto& c : log.changes)
    if (c.t <= *t) p.set(c.name, c.val);
  return p;
}

// ------------------------------------------------------------------------------------------- window

bool isQMode(double m) { return (m >= 17 && m <= 23 && m == std::floor(m)) || m == 25; }

std::optional<ChosenWindow> chooseWindow(const log::LogData& log, Family family, const std::optional<ThrottleStream>& thr,
                                         const std::vector<ParamChangeRec>& tuning, std::optional<double> tStart,
                                         std::optional<double> tEnd) {
  const bool expl = tStart || tEnd;
  if (!expl && !thr) return std::nullopt;
  const double idle = family == Family::Plane ? 0.05 : 0.15;
  const std::optional<double> tLast = thr ? std::optional<double>(thr->t.back()) : tEnd;
  struct Range {
    double t0, t1;
    std::optional<double> mode;
    std::optional<std::pair<double, double>> transition;
  };
  std::vector<Range> ranges;
  if (expl) {
    ranges.push_back({tStart.value_or(0), tEnd ? *tEnd : tLast ? *tLast : std::numeric_limits<double>::infinity(),
                      std::nullopt, std::nullopt});
  } else {
    const bool kinds = family == Family::Vtol || family == Family::Plane;
    const auto kindChange = [&](const ModeSeg& s) { return kinds && s.prev && isQMode(*s.prev) != isQMode(s.mode); };
    const std::vector<ModeSeg> all = modeSegments(log, *tLast);
    std::vector<ModeSeg> segs;
    for (const auto& s : all)
      if (family == Family::Vtol ? isQMode(s.mode) : family == Family::Plane ? !isQMode(s.mode) : true) segs.push_back(s);
    // modes are logged but none of them uses the analysed controller: there is nothing to analyse
    if (!all.empty() && segs.empty()) {
      ChosenWindow c;
      c.none = true;
      c.reason = family == Family::Vtol ? "the log has no flight in a VTOL (Q) mode" : "the log has no flight in a fixed-wing mode";
      return c;
    }
    if (!segs.empty()) {
      for (const auto& s : segs) {
        const bool kc = kindChange(s);
        ranges.push_back({kc ? s.t0 + kTransitionS : s.t0, s.t1, s.mode,
                          kc ? std::optional<std::pair<double, double>>(std::make_pair(*s.prev, s.mode)) : std::nullopt});
      }
    } else {
      ranges.push_back({0, *tLast, std::nullopt, std::nullopt});
    }
  }
  std::optional<ChosenWindow> best;
  for (const Range& r : ranges) {
    // cut where a tuning parameter changed: each piece was flown with one set of values
    std::vector<double> edges{r.t0};
    for (const auto& c : tuning)
      if (c.t > r.t0 && c.t < r.t1) edges.push_back(c.t);
    edges.push_back(r.t1);
    for (std::size_t k = 0; k + 1 < edges.size(); k++) {
      double a = edges[k], b = edges[k + 1];
      if (!expl) {
        std::optional<double> t0, t1;
        for (std::size_t i = 0; i < thr->t.size(); i++) {
          const double ts = thr->t[i];
          if (ts < a || ts > b) continue;
          if (thr->v[i] > idle) {
            if (!t0) t0 = ts;
            t1 = ts;
          }
        }
        if (!t0) continue;
        a = *t0;
        b = *t1;
      }
      if (!best || b - a > best->tEnd - best->tStart) {
        best = ChosenWindow{};
        best->tStart = a;
        best->tEnd = b;
        best->mode = r.mode;
        best->transition = r.transition;
        best->rangeT0 = r.t0;
        best->rangeT1 = r.t1;
        best->explicitRange = expl;
      }
    }
  }
  if (!best) return std::nullopt;
  for (const auto& c : tuning)
    if (c.t > best->rangeT0 && c.t < best->rangeT1) best->cutBy.push_back(c);
  return best;
}

std::optional<std::pair<double, double>> flightWindow(const log::LogData& log, Family family) {
  const auto w = chooseWindow(log, family, throttleStream(log, family), parameterChanges(log).tuning);
  if (!w || w->none) return std::nullopt;
  return std::make_pair(w->tStart, w->tEnd);
}

// ------------------------------------------------------------------------------------------- axis data

double gridTime(const AxisData& d, std::size_t g) {
  if (d.segs.empty()) return d.t0 + static_cast<double>(g) / d.fs;
  std::size_t lo = 0, hi = d.segs.size() - 1;
  while (lo < hi) {
    const std::size_t m = (lo + hi + 1) >> 1;
    if (d.segs[m].g0 <= g)
      lo = m;
    else
      hi = m - 1;
  }
  return d.segs[lo].t0 + (static_cast<double>(g) - static_cast<double>(d.segs[lo].g0)) / d.fs;
}

long long gridIndex(const AxisData& d, double t) {
  if (d.segs.empty()) {
    const double k = jsRound((t - d.t0) * d.fs);
    return k >= 0 && k < static_cast<double>(d.n) ? static_cast<long long>(k) : -1;
  }
  std::size_t lo = 0, hi = d.segs.size() - 1;
  while (lo < hi) {
    const std::size_t m = (lo + hi + 1) >> 1;
    if (d.segs[m].t0 <= t)
      lo = m;
    else
      hi = m - 1;
  }
  const AxisSeg& s = d.segs[lo];
  const double g = static_cast<double>(s.g0) + jsRound((t - s.t0) * d.fs);
  return g >= static_cast<double>(s.g0) && g < static_cast<double>(s.g1) ? static_cast<long long>(g) : -1;
}

std::optional<AxisData> axisData(const log::LogData& log, Family family, Axis axis, const Window& win, const AxisCtx* ctxIn) {
  const FamilyDef& fam = familyDef(family);
  AxisCtx own;
  if (!ctxIn) {
    own.thr = throttleStream(log, family);
    if (family != Family::Plane && own.thr && win.tStart) own.air = airborneIntervals(*own.thr, *win.tStart, win.tEnd.value_or(0));
  }
  const AxisCtx& ctx = ctxIn ? *ctxIn : own;
  const std::size_t ax = static_cast<std::size_t>(axis);
  const std::string pidName = fam.logPid[ax];
  const log::MsgTable* pid = log.msg(pidName);
  const log::MsgTable* rateMsg = log.msg("RATE") && fam.hasLogRate ? log.msg("RATE") : nullptr;
  // native rate of each stream from its typical step (robust against stamp jitter and gaps)
  const auto rateOf = [](const log::MsgTable* m) {
    if (!m || m->n <= 200 || !m->col("TimeUS")) return 0.0;
    const double s = log::medianStep(*m->col("TimeUS"));
    return s > 0 ? 1e6 / s : 0.0;
  };
  const double pidRate = rateOf(pid), rateRate = rateOf(rateMsg);
  const Vec* tUs = nullptr;
  std::string source;
  std::vector<std::pair<std::string, Vec>> cols;   // in the JavaScript order: target, actual, u, P, I, D, FF, srate, dmod, ss
  if (pid && pid->n > 200 && pidRate >= rateRate * 0.5 && pid->col("TimeUS")) {
    tUs = pid->col("TimeUS");
    source = pidName;
    const auto colOr = [&](const char* k, double fill) {
      const Vec* c = pid->col(k);
      return c ? *c : Vec(pid->n, fill);
    };
    const Vec P = colOr("P", 0), I = colOr("I", 0), D = colOr("D", 0), FF = colOr("FF", 0);
    const Vec* DFF = pid->col("DFF");
    Vec u(pid->n);
    for (std::size_t i = 0; i < pid->n; i++) u[i] = P[i] + I[i] + D[i] + FF[i] + (DFF ? (*DFF)[i] : 0);
    // copter / VTOL PIDx logs are in rad/s, plane PIDx in deg/s: keep everything in deg/s
    const double k = family == Family::Plane ? 1 : kDegPerRad;
    Vec tar = colOr("Tar", 0), act = colOr("Act", 0);
    if (k != 1) {
      for (auto& v : tar) v *= k;
      for (auto& v : act) v *= k;
    }
    cols = {{"target", std::move(tar)}, {"actual", std::move(act)}, {"u", std::move(u)}, {"P", P}, {"I", I}, {"D", D}, {"FF", FF}};
    cols.emplace_back("srate", colOr("SRate", 0));
    cols.emplace_back("dmod", colOr("Dmod", 1));
    const log::MsgTable* aetr = log.msg("AETR");
    if (family == Family::Plane && aetr && aetr->col("SS") && aetr->col("TimeUS")) {
      Sampler get{*aetr->col("TimeUS"), *aetr->col("SS")};
      Vec ss(pid->n);
      for (std::size_t i = 0; i < pid->n; i++) {
        const double v = get((*tUs)[i] / 1e6);
        ss[i] = (v != 0 && !std::isnan(v)) ? v : 1;
      }
      cols.emplace_back("ss", std::move(ss));
    }
  } else if (rateMsg && rateMsg->col("TimeUS")) {
    const auto& names = fam.logRate[ax];
    tUs = rateMsg->col("TimeUS");
    source = "RATE";
    for (std::size_t k = 0; k < 3; k++) {
      const Vec* c = rateMsg->col(names[k]);
      if (!c) return std::nullopt;
      cols.emplace_back(k == 0 ? "target" : k == 1 ? "actual" : "u", *c);
    }
  } else {
    return std::nullopt;
  }
  // time window
  std::size_t i0 = 0, i1 = tUs->size();
  if (win.tStart)
    while (i0 < i1 && (*tUs)[i0] / 1e6 < *win.tStart) i0++;
  if (win.tEnd)
    while (i1 > i0 && (*tUs)[i1 - 1] / 1e6 > *win.tEnd) i1--;
  if (i1 - i0 < 200) return std::nullopt;
  const std::size_t m = i1 - i0;
  Vec tt(m);
  for (std::size_t k = 0; k < m; k++) tt[k] = (*tUs)[i0 + k] / 1e6;
  // samples that are not numbers or that no aircraft can log (corrupt records) are masked
  static const std::pair<const char*, double> kChecks[] = {{"target", kRateLimitDegs}, {"actual", kRateLimitDegs},
                                                           {"u", kOutputLimit},        {"P", kOutputLimit},
                                                           {"I", kOutputLimit},        {"D", kOutputLimit},
                                                           {"FF", kOutputLimit}};
  std::vector<std::pair<const Vec*, double>> checks;
  for (const auto& [name, lim] : kChecks)
    for (const auto& c : cols)
      if (c.first == name) checks.emplace_back(&c.second, lim);
  std::vector<std::uint8_t> good(m, 0);
  std::size_t nonFinite = 0, implausible = 0;
  for (std::size_t k = 0; k < m; k++) {
    int bad = 0;
    for (const auto& [col, lim] : checks) {
      const double v = (*col)[i0 + k];
      if (!std::isfinite(v)) {
        bad = 1;
        break;
      }
      if (std::fabs(v) > lim) bad = 2;
    }
    if (bad == 1)
      nonFinite++;
    else if (bad == 2)
      implausible++;
    else
      good[k] = 1;
  }
  // unbroken segments (a step longer than 2.5 typical intervals is a logging gap) and the native rate inside them
  const log::GapSplit sp = log::splitAtGaps(tt);
  double numS = 0, den = 0;
  for (const auto& [a, b] : sp.segments)
    if (b - a >= 2) {
      numS += static_cast<double>(b - a - 1);
      den += tt[b - 1] - tt[a];
    }
  if (!(den > 0)) return std::nullopt;
  const double fs = jsRoundFixed(numS / den, 3);
  // grid: segments one after the other, at least one empty slot between them; long gaps are shortened
  const double maxGapSlots = std::max(1.0, jsRound(kMaxGapOnGridS * fs));
  struct Seg {
    std::size_t g0, g1;
    double t0;
    std::size_t k0;
  };
  std::vector<Seg> segs;
  for (const auto& [a, b] : sp.segments) {
    std::size_t g0 = 0;
    if (!segs.empty()) {
      const Seg& p = segs.back();
      const double slots = jsRound((tt[a] - (p.t0 + static_cast<double>(p.g1 - p.g0) / fs)) * fs);
      g0 = p.g1 + static_cast<std::size_t>(std::max(1.0, std::min(maxGapSlots, slots)));
    }
    segs.push_back({g0, g0 + (b - a), tt[a], a});
  }
  if (segs.empty()) return std::nullopt;
  const std::size_t N = segs.back().g1;
  AxisData d;
  d.valid.assign(N, 0);
  for (const auto& s : segs)
    for (std::size_t g = s.g0; g < s.g1; g++) d.valid[g] = good[s.k0 + g - s.g0];
  d.fs = fs;
  d.t0 = tt[0];
  d.n = N;
  d.rawRate = sp.medianDt > 0 ? 1 / sp.medianDt : fs;
  d.source = source;
  for (const auto& s : segs) d.segs.push_back({s.g0, s.g1, s.t0});
  d.gaps = sp.gaps;
  d.nonFinite = nonFinite;
  d.implausible = implausible;
  for (const auto& [name, src] : cols) {
    Vec a(N, 0.0);
    for (const auto& s : segs)
      for (std::size_t g = s.g0; g < s.g1; g++) a[g] = src[i0 + s.k0 + g - s.g0];
    // slots without a usable sample hold the last usable value (they are never used: valid = 0, they only keep
    // running filters calm)
    std::optional<double> last;
    for (std::size_t g = 0; g < N; g++) {
      if (d.valid[g])
        last = a[g];
      else
        a[g] = last ? *last : std::nan("");
    }
    std::optional<double> first;
    for (std::size_t g = 0; g < N && !first; g++)
      if (d.valid[g]) first = a[g];
    for (std::size_t g = 0; g < N && !d.valid[g]; g++) a[g] = first ? *first : 0;
    if (name == "target") d.target = std::move(a);
    else if (name == "actual") d.actual = std::move(a);
    else if (name == "u") d.u = std::move(a);
    else if (name == "P") d.P = std::move(a);
    else if (name == "I") d.I = std::move(a);
    else if (name == "D") d.D = std::move(a);
    else if (name == "FF") d.FF = std::move(a);
    else if (name == "srate") d.srate = std::move(a);
    else if (name == "dmod") d.dmod = std::move(a);
    else if (name == "ss") d.ss = std::move(a);
  }
  // airborne (copter / VTOL): on the ground the controller output says nothing about the airframe
  if (family != Family::Plane && ctx.air) {
    d.airborne.assign(N, 0);
    for (const auto& s : segs)
      for (std::size_t g = s.g0; g < s.g1; g++) d.airborne[g] = inIntervals(ctx.air->intervals, tt[s.k0 + g - s.g0]) ? 1 : 0;
  }
  // plane: speed scaler from AETR.SS (mean of scaler^2 over the window)
  d.scaler2 = 1;
  const log::MsgTable* aetr = log.msg("AETR");
  if (family == Family::Plane && aetr && aetr->col("SS") && aetr->col("TimeUS")) {
    const Vec& at = *aetr->col("TimeUS");
    const Vec& ss = *aetr->col("SS");
    double s = 0, c = 0;
    for (std::size_t i = 0; i < aetr->n; i++) {
      const double ts = at[i] / 1e6;
      if (ts >= tt[0] && ts <= tt[m - 1] && ss[i] > 0) {
        s += ss[i] * ss[i];
        c++;
      }
    }
    if (c != 0) d.scaler2 = s / c;
  }
  // throttle (0..1) for the notch operating point: the hover throttle while airborne
  if (ctx.op && ctx.op->throttle) {
    d.throttle = *ctx.op->throttle;
  } else if (ctx.thr) {
    double s = 0, c = 0;
    for (std::size_t i = 0; i < ctx.thr->t.size(); i++) {
      const double ts = ctx.thr->t[i];
      if (ts >= tt[0] && ts <= tt[m - 1]) {
        s += ctx.thr->v[i];
        c++;
      }
    }
    d.throttle = c != 0 ? s / c : 0;
  }
  return d;
}

Json iStats(const log::LogData& log, Family family, Axis axis, const AxisData& d) {
  const FamilyDef& fam = familyDef(family);
  const log::MsgTable* pid = log.msg(fam.logPid[static_cast<std::size_t>(axis)]);
  if (!pid || !pid->col("I") || !pid->col("TimeUS")) return Json();
  const Vec& tUs = *pid->col("TimeUS");
  const Vec& I = *pid->col("I");
  Vec v;
  double m = 0;
  for (std::size_t i = 0; i < pid->n; i++) {
    const long long g = gridIndex(d, tUs[i] / 1e6);
    if (g < 0) continue;
    const std::size_t k = static_cast<std::size_t>(g);
    if (!d.valid.empty() && !d.valid[k]) continue;
    if (!d.airborne.empty() && !d.airborne[k]) continue;
    if (!std::isfinite(I[i])) continue;
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

Json motorStats(const log::LogData& log, const log::Params& params, Family family, const Window& win,
                const std::optional<Airborne>& air) {
  const log::MsgTable* rc = log.msg("RCOU");
  if (!rc || !rc->col("TimeUS")) return Json();
  const std::string q = family == Family::Vtol ? "Q_M_" : "MOT_";
  struct Chan {
    const Vec* col = nullptr;
    int n = 0;
  };
  std::vector<Chan> chans(9);   // motor k (1-based) -> RCOU column
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
  std::vector<double> sum(9, 0.0);
  double c = 0, pinned = 0, maxPwm = 0;
  const Vec& rt = *rc->col("TimeUS");
  for (std::size_t i = 0; i < rc->n; i++) {
    const double ts = rt[i] / 1e6;
    if (win.tStart && (ts < *win.tStart || ts > win.tEnd.value_or(0))) continue;
    if (air && !inIntervals(air->intervals, ts)) continue;
    bool ok = true;
    for (int k : motors)
      if (!std::isfinite((*chans[static_cast<std::size_t>(k)].col)[i])) ok = false;
    if (!ok) continue;
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
  // ArduPilot quad X / +: motors 1 and 2 spin counter-clockwise, 3 and 4 clockwise (a positive yaw output speeds
  // up the CCW pair). Other frames: no split.
  const auto cls = params.get(family == Family::Vtol ? "Q_FRAME_CLASS" : "FRAME_CLASS");
  const auto typ = params.get(family == Family::Vtol ? "Q_FRAME_TYPE" : "FRAME_TYPE");
  if (cls && *cls == 1 && typ && (*typ == 0 || *typ == 1) && mean.has("1") && mean.has("2") && mean.has("3") &&
      mean.has("4")) {
    out.set("ccwMean", (mean["1"].num() + mean["2"].num()) / 2);
    out.set("cwMean", (mean["3"].num() + mean["4"].num()) / 2);
  }
  return out;
}

// ------------------------------------------------------------------------------------------- analysis

Json analyzeLog(log::LogData& log, const std::string& file, const AnalyzeLogOptions& opt) {
  const Family fam = opt.family ? *opt.family : detectLogFamily(log);
  if (!log.integrity.format.supported) return refusal(file, fam, log);
  const std::map<std::string, log::TimeIssue> timeIssues = log::cleanLogTimes(log, logTimed());
  const ParamChanges changes = parameterChanges(log);
  const std::optional<ThrottleStream> thr = throttleStream(log, fam);
  const std::optional<ChosenWindow> found = chooseWindow(log, fam, thr, changes.tuning, opt.tStart, opt.tEnd);
  const std::optional<std::string> none = found && found->none ? std::optional<std::string>(found->reason) : std::nullopt;
  const ChosenWindow* w = found && !found->none ? &*found : nullptr;
  Window win;
  win.tStart = w ? std::optional<double>(w->tStart) : opt.tStart;
  win.tEnd = w ? std::optional<double>(w->tEnd) : opt.tEnd;
  const log::Params params = paramsAt(log, win.tStart);
  std::optional<Airborne> air;
  if (fam != Family::Plane && thr && win.tStart) air = airborneIntervals(*thr, *win.tStart, win.tEnd.value_or(0));
  const NotchOp op = notchOperatingPoint(log, fam, win, air);
  AxisCtx ctx;
  ctx.thr = thr;
  ctx.air = air;
  ctx.op = op;
  // firmware: the first "Ardu..." text message, else the first text message
  Json firmware;
  static const std::regex fwRe("Ardu(Copter|Plane|Rover|Sub)|^Ardu", std::regex::icase);
  for (const auto& t : log.text)
    if (std::regex_search(t.msg, fwRe)) {
      firmware = t.msg;
      break;
    }
  if (firmware.isNull() && !log.text.empty()) firmware = log.text[0].msg;
  // the logger counts the messages it had to drop (DSF.Dp, cumulative)
  double loggerDropped = 0;
  if (const log::MsgTable* dsf = log.msg("DSF"); dsf && dsf->col("Dp"))
    for (double v : *dsf->col("Dp"))
      if (v > loggerDropped) loggerDropped = v;
  Json timeJ = Json::object();
  for (const std::string& name : logTimed()) {
    const auto it = timeIssues.find(name);
    if (it == timeIssues.end()) continue;
    const log::TimeIssue& r = it->second;
    timeJ.set(name, Json::obj({{"outOfOrder", r.outOfOrder},
                               {"duplicates", r.duplicates},
                               {"reset", r.reset ? Json::obj({{"fromS", r.reset->first}, {"toS", r.reset->second}}) : Json()},
                               {"droppedAfterReset", r.droppedAfterReset}}));
  }
  std::vector<std::string> notes = logNotes(log.integrity, loggerDropped, timeIssues,
                                            {familyDef(fam).logPid[0], "RATE", thr ? thr->name : "CTUN", "MODE"});
  for (auto& s : windowNotes(w)) notes.push_back(std::move(s));
  Json integrity = integrityJson(log.integrity);
  integrity.set("timeIssues", timeJ);
  integrity.set("loggerDropped", loggerDropped);
  Json notesJ = Json::array();
  for (const auto& s : notes) notesJ.push(s);
  integrity.set("notes", notesJ);
  Json paramsJ = Json::object();
  for (const auto& [k, v] : params.items()) paramsJ.items().emplace_back(k, Json(v));
  Json tuningJ = Json::array();
  for (std::size_t i = 0; i < changes.tuning.size() && i < 50; i++) {
    const ParamChangeRec& c = changes.tuning[i];
    Json cj = Json::obj({{"t", c.t}, {"name", c.name}});
    if (c.from) cj.set("from", *c.from);
    cj.set("to", c.to);
    tuningJ.push(cj);
  }
  Json out = Json::obj({{"file", file}, {"family", familyName(fam)}});
  if (!firmware.isNull()) out.set("firmware", firmware);
  out.set("format", Json::obj({{"kind", log.integrity.format.kind}, {"supported", true}}));
  out.set("params", paramsJ);
  out.set("paramChanges", changes.tuning.size());
  out.set("paramChangesAll", changes.all.size());
  out.set("tuningChanges", tuningJ);
  out.set("flightWindow", Json::obj({{"tStart", win.tStart ? Json(*win.tStart) : Json()}, {"tEnd", win.tEnd ? Json(*win.tEnd) : Json()}}));
  out.set("integrity", integrity);
  out.set("axes", Json::object());
  out.set("counts", countsJson(log));
  if (none) {
    // the log's modes never used the analysed controller: refuse instead of analysing the wrong flight
    const std::string msg = "Nothing to analyse for the " + std::string(familyName(fam)) + " controller: " + *none + ".";
    for (const Axis axis : kAxes) out.at("axes").set(axisName(axis), Json::obj({{"error", msg}}));
    out.set("noise", Json::obj({{"available", false}, {"reason", msg}}));
    out.at("integrity").at("notes").push(msg);
    return out;
  }
  const Json motors = fam != Family::Plane ? motorStats(log, params, fam, win, air) : Json();
  if (motors.isObject()) out.set("motors", motors);
  for (const Axis axis : kAxes) {
    auto d = axisData(log, fam, axis, win, &ctx);
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
    a.gyroOp = op;
    a.gyroOp.throttle = d->throttle;
    a.scaler2 = d->scaler2;
    a.timeDomain = true;   // the hover-data fit is used only for log analysis: its result is reviewed before writing
    Json res = analyzeAxis(a);
    std::size_t nValid = 0, nUse = 0;
    for (std::size_t g = 0; g < d->n; g++)
      if (d->valid[g]) {
        nValid++;
        if (d->airborne.empty() || d->airborne[g]) nUse++;
      }
    const std::optional<double> airS = !d->airborne.empty() ? std::optional<double>(static_cast<double>(nUse) / d->fs) : std::nullopt;
    std::size_t longestN = 0;
    for (const auto& s : d->segs) longestN = std::max(longestN, s.g1 - s.g0);
    Json gapsJ = Json::array();
    for (std::size_t k = 0; k < d->gaps.size() && k < 20; k++) {
      const auto& g = d->gaps[k];
      gapsJ.push(Json::obj({{"t0", g.first}, {"t1", g.second}, {"s", g.second - g.first}}));
    }
    Json info = Json::obj({{"fs", d->fs},
                           {"rawRateHz", d->rawRate},
                           {"source", d->source},
                           {"seconds", static_cast<double>(nValid) / d->fs},
                           {"usableSeconds", static_cast<double>(nUse) / d->fs},
                           {"windowSeconds", win.tStart ? Json(win.tEnd.value_or(0) - *win.tStart) : Json()},
                           {"segments", d->segs.size()},
                           {"longestSegmentS", static_cast<double>(longestN) / d->fs},
                           {"gaps", gapsJ},
                           {"gapCount", d->gaps.size()},
                           {"excluded", Json::obj({{"nonFinite", d->nonFinite}, {"implausible", d->implausible}})},
                           {"scaler2", d->scaler2}});
    if (d->throttle) info.set("throttle", *d->throttle);
    res.set("dataInfo", info);
    {
      Json::Array front;
      for (const auto& s : notes) front.emplace_back(s);
      for (auto& s : axisNotes(*d, win, airS)) front.emplace_back(std::move(s));
      Json& wj = res.at("warnings");
      if (!wj.isArray()) wj = Json::array();
      Json::Array& wa = wj.arr();
      wa.insert(wa.begin(), front.begin(), front.end());
    }
    // time-series preview (decimated, real samples only) for the UI
    const std::size_t step = std::max<std::size_t>(1, d->n / 4000);
    Json st = Json::array(), sTar = Json::array(), sAct = Json::array(), su = Json::array();
    for (std::size_t g = 0; g < d->n; g += step) {
      if (!d->valid[g]) continue;
      st.push(jsRoundFixed(gridTime(*d, g), 3));
      sTar.push(jsRoundFixed(d->target[g], 2));
      sAct.push(jsRoundFixed(d->actual[g], 2));
      su.push(jsRoundFixed(d->u[g], 4));
    }
    res.set("series", Json::obj({{"t", st}, {"target", sTar}, {"actual", sAct}, {"u", su}}));
    try {
      res.set("issues", axisIssues(res, fam));
    } catch (const std::exception&) {
      res.set("issues", Json::array());
    }
    out.at("axes").set(axisName(axis), res);
  }
  out.set("noise", analyzeNoise(log, params, fam, win));
  return out;
}

Json analyzeLogFile(const std::string& path, const AnalyzeLogOptions& opt) {
  log::LogData log = log::parseBinFile(path, logWant());
  return analyzeLog(log, path, opt);
}

}  // namespace aptuner::engine
