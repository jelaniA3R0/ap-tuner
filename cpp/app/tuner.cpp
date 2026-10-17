#include "tuner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include "aptuner/core/jsnum.hpp"
#include "aptuner/engine/analysis.hpp"
#include "aptuner/engine/vehicle.hpp"
#include "common.hpp"

namespace aptuner::app {

using core::jsToFixed;
using core::jsToString;
using rt::Loop;
using engine::jor;
using engine::jv;

const std::vector<std::string> kAxisNames{"roll", "pitch", "yaw"};
const std::string kInFlightOff =
    "changes in flight are turned off: while the vehicle is armed the app only measures. Land to write the changes, or turn on "
    "\"Allow changes in flight\" (experimental, tested only in the simulator)";

namespace {
constexpr int kPidTuningId = 194, kAttitudeId = 30;
// gain keys in the order they are written (filters first, D before P)
const std::vector<std::string> kWriteOrder{"FLTT", "FLTE", "FLTD", "SMAX", "D", "P", "I", "FF", "angleP", "tconst"};
// per-axis values a corrective write moves in steps of at most stepMax; kLinearKeys also from / to zero (linearly)
const std::vector<std::string> kSteppedKeys{"P", "I", "D", "FF", "angleP", "tconst", "FLTT", "FLTE", "FLTD"};
const std::vector<std::string> kLinearKeys{"P", "I", "D", "FF", "angleP", "tconst"};
// vehicle-wide values: a change affects every axis (notch, gyro filter, FFT)
const std::vector<std::string> kSharedParams{"INS_HNTCH_ENABLE", "INS_HNTCH_MODE", "INS_HNTCH_FREQ", "INS_HNTCH_BW", "INS_HNTCH_ATT", "INS_HNTCH_REF",
                                             "INS_HNTCH_HMNCS",  "INS_HNTCH_OPTS", "INS_HNTCH_FM_RAT", "INS_GYRO_FILTER", "FFT_ENABLE"};
// share of an observation window with slew-rate / Dmod trips that counts as "oscillating" when no model fits
constexpr double kOscSeenFraction = 0.05;
const std::vector<std::string> kRunningStates{"observing", "analyzing", "applying", "proposed", "ramping"};

core::Axis axisOf(const std::string& a) { return *core::parseAxis(a); }

std::string str(const Json& v) {
  if (v.isNumber()) return jsToString(v.num());
  if (v.isString()) return v.str();
  if (v.isNull()) return "undefined";
  return v.dump();
}

bool hasKey(const Json& o, const std::string& k) { return o.isObject() && o.has(k) && !o[k].isNull(); }
bool inList(const std::vector<std::string>& v, const std::string& s) { return std::find(v.begin(), v.end(), s) != v.end(); }

std::string join(const std::vector<std::string>& v, const std::string& sep) {
  std::string out;
  for (std::size_t i = 0; i < v.size(); i++) out += (i ? sep : "") + v[i];
  return out;
}

double wrap180(double d) { return std::fmod(d + 540, 360) - 180; }

/** steps so that no multiplicative change exceeds stepMax (pairs with a zero end do not count) */
int stepsFor(const std::vector<std::pair<double, double>>& pairs, double stepMax) {
  int steps = 1;
  for (const auto& [a, b] : pairs)
    if (a > 0 && b > 0) steps = std::max(steps, static_cast<int>(std::ceil(std::fabs(std::log(b / a)) / std::log(stepMax) - 1e-9)));
  return steps;
}
/** value at step s of steps: geometric between positive ends, linear otherwise; the end value at the last step */
double interp(double a, double b, int s, int steps) {
  if (s == steps) return b;
  return (a > 0 && b > 0) ? a * std::pow(b / a, static_cast<double>(s) / steps) : a + (b - a) * s / steps;
}

/** runs a function when it goes out of scope (the JavaScript `finally`; also in coroutines) */
struct ScopeExit {
  std::function<void()> f;
  ~ScopeExit() {
    if (f) f();
  }
};
}  // namespace

TripError::TripError(Trip t) : rt::Error("oscillation: " + join(t.reasons, "; "), "trip"), trip(std::move(t)) {}

Tuner::Tuner(io::Link& lk, const std::string& sessionDir) : link(lk), sessionDir_(sessionDir) {
  opts = Json::obj({{"preset", "normal"}, {"autoApply", false}, {"axes", Json(Json::Array{"roll", "pitch"})}, {"observeSeconds", 30},
                    {"stepMax", 1.5}, {"watchSeconds", 4}, {"oscSmax", 5}, {"massKg", 0}, {"allowRamp", false},
                    {"rampDoubleTime", 10}, {"rampMargin", 60}, {"maxReducePct", 20}, {"inFlightWrites", false}, {"watchGrace", 0.5},
                    {"filterWatchSeconds", 10}, {"monitorSeconds", 2}, {"monitorFraction", 0.5}, {"revertStepSeconds", 1},
                    {"calmWaitSeconds", 4}, {"angleAbort", 10}, {"angleAbortSeconds", 0.8}, {"sessionDir", sessionDir}});
  for (const auto& a : kAxisNames) axes.emplace(a, LiveAxis(a));
  attitude = Json::obj({{"roll", 0}, {"pitch", 0}, {"yaw", 0}, {"p", 0}, {"q", 0}, {"r", 0}, {"ts", 0}});
  rc = Json::obj({{"roll", 0}, {"pitch", 0}, {"yaw", 0}, {"thr", 0}, {"ts", 0}});
  hud = Json::obj({{"airspeed", 0}, {"throttle", 0}, {"alt", 0}});
  monitor = Json::obj({{"active", false}, {"axes", Json::object()}, {"attitude", Json::obj({{"active", false}, {"roll", Json()}, {"pitch", Json()}})}, {"lastTrip", Json()}});
  stateSince = Loop::now();
  // ---- link bindings
  link.on("PID_TUNING", [this](const Json& f) {
    // the units depend on the controller family: nothing is stored before it is known (F-91)
    if (!family) return;
    const double ax = f["axis"].numOr(0);
    const char* name = ax == 1 ? "roll" : ax == 2 ? "pitch" : ax == 3 ? "yaw" : nullptr;
    if (!name) return;
    // copter / VTOL rate PIDs run in rad/s (plane reports deg/s): keep everything in deg/s
    const double k = family == Family::Plane ? 1 : 57.2957795;
    Json g = f;
    g.set("desired", f["desired"].numOr(0) * k);
    g.set("achieved", f["achieved"].numOr(0) * k);
    axes[name].push(Loop::now() / 1000, g);
  });
  link.on("ATTITUDE", [this](const Json& f) {
    auto d = [&](const char* k) { return f[k].numOr(0) * 57.2958; };
    const double t = Loop::now() / 1000;
    attitude = Json::obj({{"roll", d("roll")}, {"pitch", d("pitch")}, {"yaw", d("yaw")}, {"p", d("rollspeed")}, {"q", d("pitchspeed")},
                          {"r", d("yawspeed")}, {"ts", t}});
    attHist_.push_back({t, d("roll"), d("pitch")});
    while (!attHist_.empty() && attHist_.front().t < t - 3) attHist_.pop_front();
  });
  // attitude target (Copter: the attitude controller's target; Plane: nav_roll / nav_pitch) for the attitude-error abort
  link.on("NAV_CONTROLLER_OUTPUT", [this](const Json& f) {
    const double t = Loop::now() / 1000;
    const AttSample* best = nullptr;
    for (const auto& a : attHist_)
      if (std::fabs(a.t - t) <= 0.1 && (!best || std::fabs(a.t - t) < std::fabs(best->t - t))) best = &a;
    if (best) attErr_.push_back({t, wrap180(best->roll - f["nav_roll"].numOr(0)), best->pitch - f["nav_pitch"].numOr(0)});
    while (!attErr_.empty() && attErr_.front().t < t - 5) attErr_.pop_front();
  });
  link.on("RC_CHANNELS", [this](const Json& f) {
    auto n = [&](const char* k) { return (f[k].numOr(0) - 1500) / 500; };
    rc = Json::obj({{"roll", n("chan1_raw")}, {"pitch", n("chan2_raw")}, {"yaw", n("chan4_raw")},
                    {"thr", (f["chan3_raw"].numOr(0) - 1000) / 1000}, {"ts", Loop::now() / 1000}});
  });
  link.on("VFR_HUD", [this](const Json& f) {
    hud = Json::obj({{"airspeed", f["airspeed"]}, {"throttle", f["throttle"]}, {"alt", f["alt"]}, {"groundspeed", f["groundspeed"]}});
  });
  link.on("STATUSTEXT", [this](const Json& f) { log("vehicle", f["text"].strOr("")); });
  link.on("VIBRATION", [this](const Json& f) {
    auto i = [&](const char* k) { return static_cast<double>(static_cast<long long>(f[k].numOr(0))); };
    vibe = Json::obj({{"x", f["vibration_x"]}, {"y", f["vibration_y"]}, {"z", f["vibration_z"]},
                      {"clip", i("clipping_0") + i("clipping_1") + i("clipping_2")}, {"ts", Loop::now() / 1000}});
  });
  link.on("heartbeat", [this](const Json& v) {
    if (!family && params.size())
      family = engine::detectFamily(static_cast<int>(v["type"].numOr(0)), v["modeName"].strOr(""), toParams(params));
    if (linkGen_ != link.generation()) {
      linkGen_ = link.generation();
      if (family) {
        requestStreams();
        log("link", "new connection: streams re-requested");
      }
    }
    // a disarm pauses a running session (once: a paused session keeps its message)
    if (v["armed"].asBool())
      armedSeen_ = true;
    else if (armedSeen_ && inList(kRunningStates, state))
      onDisarm();
  });
}

Tuner::~Tuner() {
  stopMonitor();
  if (timer_) Loop::main().clear(timer_);
}

void Tuner::log(const std::string& kind, const std::string& text) {
  const Json e = Json::obj({{"t", Loop::now()}, {"kind", kind}, {"text", text}});
  messages.push(e);
  if (messages.size() > 500) messages.arr().erase(messages.arr().begin());
  emit("log", e);
}

void Tuner::setState(const std::string& s, const std::string& info) {
  state = s;
  stateSince = Loop::now();
  stateInfo = info;
  emit("state", Json::obj({{"state", s}, {"info", info}}));
  log("tuner", s + (info.empty() ? "" : ": " + info));
}

Json Tuner::gainsOf(const std::string& axis) const { return engine::currentGains(*family, axisOf(axis), toParams(params)); }

rt::Task<Json> Tuner::prepare(bool confirm) {
  if (!link.alive()) throw rt::Error("no vehicle heartbeat: connect first");
  if (job_) throw rt::Error("a change is being written (" + job_->kind + "): wait until it has finished");
  Json r;
  std::exception_ptr ep;
  std::string what;
  try {
    r = co_await prepareInner(confirm);
  } catch (const std::exception& e) {
    ep = std::current_exception();
    what = e.what();
  }
  if (ep) {
    if (state == "preparing") setState("idle", "reading the vehicle failed: " + what);
    std::rethrow_exception(ep);
  }
  co_return r;
}

rt::Task<Json> Tuner::prepareInner(bool confirm) {
  setState("preparing", "reading parameters");
  // a new connection may be a different vehicle: drop everything cached from the previous one
  if (preparedGen_ != link.generation()) {
    preparedGen_ = link.generation();
    params = Json::object();
    originals = Json::object();
    previous = Json::object();
    results = Json::object();
    lastGood = Json::object();
    prepared = false;
    endSession();
  }
  // samples buffered so far may be from another vehicle or in other units: none are kept until the family is known (F-91)
  family.reset();
  for (const auto& a : kAxisNames) axes[a].clear();
  const Json v = link.vehicle();
  const int type = static_cast<int>(v["type"].numOr(0));
  std::optional<Family> fam = engine::detectFamily(type, v["modeName"].strOr(""), log::Params());
  if (fam == Family::Plane) {
    double q = 0;
    try {
      q = co_await link.requestParam("Q_ENABLE");
    } catch (const std::exception&) {
      q = 0;
    }
    params.set("Q_ENABLE", q);
    fam = engine::detectFamily(type, v["modeName"].strOr(""), toParams(params));
  }
  if (!fam) throw rt::Error("unsupported vehicle type " + v["typeName"].strOr("?"));
  family = fam;
  const std::vector<std::string> names = core::neededParams(*fam == Family::Heli ? Family::Copter : *fam);
  const Json got = co_await link.requestParams(names, 3, [this](int d, int n) { emit("progress", Json::obj({{"done", d}, {"total", n}})); });
  for (const auto& [k, val] : got.items()) params.set(k, val);
  loopHz = jor(params, "SCHED_LOOP_RATE", *fam == Family::Plane ? 50 : 400);
  // PID_TUNING streaming needs GCS_PID_MASK (bits: roll 1, pitch 2, yaw 4) and SMAX > 0 for the slew-rate signal
  Json writes = Json::array();
  if (static_cast<int>(params["GCS_PID_MASK"].numOr(0)) != 7)
    writes.push(Json::obj({{"name", "GCS_PID_MASK"}, {"from", params["GCS_PID_MASK"]}, {"to", 7},
                           {"why", "turns on the PID telemetry the app listens to (roll, pitch, yaw). Harmless; you can set it back to 0 later."}}));
  for (const auto& ax : kAxisNames) {
    const Json n = engine::axisParamNames(*fam, axisOf(ax));
    const std::string smax = n["SMAX"].str();
    if (!(jv(params, smax.c_str()) > 0))
      writes.push(Json::obj({{"name", smax}, {"from", params[smax]}, {"to", core::familyDef(*fam).defaultSmax},
                             {"why", "slew-rate limit; ArduPilot only reports the oscillation signal (SRate) when it is above zero. 50 is the QuikTune default and does not change how the vehicle flies."}}));
  }
  pendingWrites = writes;
  for (const auto& ax : kAxisNames)
    if (!hasKey(originals, ax)) originals.set(ax, gainsOf(ax));
  auto gainsAll = [this]() {
    Json g = Json::object();
    for (const auto& a : kAxisNames) g.set(a, gainsOf(a));
    return g;
  };
  const Json checks = safetyChecks();
  if (writes.size() && !confirm) {
    setState("ready", std::string(core::familyName(*fam)) + " detected; " + std::to_string(writes.size()) +
                          " housekeeping setting(s) need your OK");
    co_return Json::obj({{"family", core::familyName(*fam)}, {"vehicle", v}, {"needsConfirm", true}, {"writes", writes},
                         {"checks", checks}, {"gains", gainsAll()}, {"loopHz", loopHz}});
  }
  if (writes.size()) {
    // housekeeping only (telemetry on, slew rate reported): not watched, but while armed it needs the opt-in too
    auto job = beginJob("prepare");
    ScopeExit guard{[this, job] { endJob(job); }};
    for (const auto& w : writes.arr()) {
      const std::string why = w["why"].str();
      co_await setParam(w["name"].str(), w["to"].num(), why.substr(0, why.find('.')), job);
    }
  }
  pendingWrites = Json::array();
  requestStreams();
  lastGood = originals;
  if (sessionFile_.empty()) sessionFile_ = pathStr(fsPath(sessionDir_) / (isoStampForFile() + ".json"));
  saveSession();
  prepared = true;
  setState("ready", std::string(core::familyName(*fam)) + " detected, loop " + jsToString(loopHz) + " Hz, gyro filter " +
                        str(params["INS_GYRO_FILTER"]) + " Hz");
  co_return Json::obj({{"family", core::familyName(*fam)}, {"vehicle", v}, {"needsConfirm", false}, {"writes", Json::array()},
                       {"checks", safetyChecks()}, {"gains", gainsAll()}, {"loopHz", loopHz}});
}

rt::Task<void> Tuner::switchFamily(Family fam) {
  if (family == fam) co_return;
  if (!prepared) throw rt::Error("read the vehicle first");
  if (job_) throw rt::Error("a change is being written (" + job_->kind + "): wait until it has finished");
  // read every parameter of the new family afresh (the vehicle may have been changed or replaced)
  const std::vector<std::string> names = core::neededParams(fam == Family::Heli ? Family::Copter : fam);
  const Json got = co_await link.requestParams(names, 3, [this](int d, int n) { emit("progress", Json::obj({{"done", d}, {"total", n}})); });
  for (const auto& [k, val] : got.items()) params.set(k, val);
  family = fam;
  loopHz = jor(params, "SCHED_LOOP_RATE", fam == Family::Plane ? 50 : 400);
  for (const auto& ax : kAxisNames) {
    originals.set(ax, gainsOf(ax));
    axes[ax].clear();
  }
  lastGood = originals;
  previous = Json::object();
  results = Json::object();
  {
    auto job = beginJob("switch family");
    ScopeExit guard{[this, job] { endJob(job); }};
    for (const auto& ax : kAxisNames) {
      const Json n = engine::axisParamNames(fam, axisOf(ax));
      const std::string smax = n["SMAX"].strOr("");
      if (!smax.empty() && !(jv(params, smax.c_str()) > 0))
        co_await setParam(smax, core::familyDef(fam).defaultSmax, "slew-rate limit so the oscillation signal is reported", job);
    }
  }
  requestStreams();
  log("tuner", std::string("controller family switched to ") + core::familyName(fam) + " (loop " + jsToString(loopHz) + " Hz)");
  saveSession();
}

Json Tuner::safetyChecks() const {
  const Json& p = params;
  Json out = Json::array();
  auto add = [&](bool ok, const std::string& title, const std::string& detail) {
    out.push(Json::obj({{"ok", ok}, {"title", title}, {"detail", detail}}));
  };
  if (family == Family::Copter || family == Family::Vtol || family == Family::Heli) {
    const std::string pre = family == Family::Vtol ? "Q_M_" : "MOT_";
    const Json& hov = p[pre + "THST_HOVER"];
    if (hov.isNumber()) {
      const double h = hov.num();
      const bool good = h > 0.2 && h < 0.7;
      add(good, "Hover throttle " + jsToFixed(h, 2),
          good ? "learned value in the normal range" : "outside 0.2-0.7: check MOT_HOVER_LEARN and that the vehicle has hovered long enough");
    }
    const Json& expo = p[pre + "THST_EXPO"];
    if (expo.isNumber())
      add(expo.num() >= 0.4 && expo.num() <= 0.8, "Thrust expo " + jsToFixed(expo.num(), 2),
          "ArduPilot guide: 0.55 for 5\", 0.65 for 10\", 0.75 for 20\" props (thrust-stand data is better)");
    const Json& gf = p["INS_GYRO_FILTER"];
    if (gf.isNumber())
      add(gf.num() >= 10 && gf.num() <= 100, "Gyro filter " + jsToString(gf.num()) + " Hz",
          "guide: 80 Hz for 5\", 40 Hz for 10\", 20 Hz for 20\" and larger");
    const Json& ffe = p[family == Family::Vtol ? "Q_A_RATE_FF_ENAB" : "ATC_RATE_FF_ENAB"];
    if (ffe.isNumber())
      add(ffe.num() > 0, std::string("Rate feed-forward ") + (ffe.num() > 0 ? "on" : "off"),
          ffe.num() > 0 ? "normal" : "ATC_RATE_FF_ENAB should be 1 for normal flying");
    const bool notchOn = jv(p, "INS_HNTCH_ENABLE") > 0;
    add(notchOn, std::string("Harmonic notch ") + (notchOn ? "enabled" : "not set up"),
        notchOn ? "mode " + str(p["INS_HNTCH_MODE"]) + ", " + str(p["INS_HNTCH_FREQ"]) + " Hz"
                : "a notch filter usually allows a much tighter tune; see step 4");
  }
  if (family == Family::Plane) {
    add(jv(p, "SCALING_SPEED") > 0, "Scaling speed " + str(p["SCALING_SPEED"]) + " m/s",
        "should be close to the normal cruise speed; changing it later invalidates the tune");
    add(jv(p, "RLL_RATE_FF") > 0 && jv(p, "PTCH_RATE_FF") > 0, "Feed-forward set", "plane rate loops are driven mostly by FF");
  }
  add(true, std::string("PID telemetry ") + (static_cast<int>(p["GCS_PID_MASK"].numOr(0)) == 7 ? "on" : "will be turned on"),
      "GCS_PID_MASK = 7");
  return out;
}

void Tuner::requestStreams() {
  // legacy stream request first (EXTRA1 carries ATTITUDE + PID_TUNING); it would otherwise override the intervals
  link.requestDataStream(10, 20);
  link.requestDataStream(3, 5);   // RC_CHANNELS
  link.requestDataStream(2, 5);   // EXTENDED_STATUS: NAV_CONTROLLER_OUTPUT (attitude target)
  Loop::main().setTimeout(1500, [this] {
    link.setMessageRate(kPidTuningId, family == Family::Plane ? 25 : 50);
    link.setMessageRate(kAttitudeId, 25);
    link.setMessageRate(241, 2);   // VIBRATION, for the final check
  });
}

// ---------------- writing: one write lock, the in-flight gate, cancellation

bool Tuner::isArmed() const {
  const Json& v = link.vehicle();
  return v.isObject() && v["armed"].asBool();
}

std::shared_ptr<Job> Tuner::beginJob(const std::string& kind, bool corrective, bool handlesTrips) {
  if (job_) throw rt::Error("a change is being written (" + job_->kind + "): wait until it has finished");
  auto j = std::make_shared<Job>();
  j->id = ++jobSeq_;
  j->kind = kind;
  j->corrective = corrective;
  j->handlesTrips = handlesTrips;
  job_ = j;
  return j;
}

void Tuner::endJob(const std::shared_ptr<Job>& job) {
  if (job_ == job) job_.reset();
}

void Tuner::cancelJob(const std::string& reason) {
  if (job_ && !job_->corrective && job_->cancelled.empty()) job_->cancelled = reason.empty() ? "cancelled" : reason;
}

rt::Task<bool> Tuner::waitNoJob(double ms) {
  const double t0 = Loop::now();
  while (job_ && Loop::now() - t0 < ms) co_await rt::sleep(50);
  co_return !job_;
}

void Tuner::checkpoint(Job& job) {
  if (job.corrective) return;
  if (pendingTrip_) {
    if (!job.handlesTrips) throw rt::Error("not written: the vehicle is oscillating", "blocked");
    Trip t = *pendingTrip_;
    pendingTrip_.reset();
    throw TripError(std::move(t));
  }
  if (!job.cancelled.empty()) throw rt::Error("stopped: " + job.cancelled, "cancelled");
}

void Tuner::assertForward(const Job& job) const {
  if (!link.alive()) throw rt::Error("no vehicle heartbeat: connect first");
  if (job.kind != "prepare" && (!prepared || preparedGen_ != link.generation())) throw rt::Error("read the vehicle first (the connection changed)");
  if (isArmed() && !opts["inFlightWrites"].asBool()) throw rt::Error(kInFlightOff, "inflight-off");
}

std::string Tuner::watchBlind(const std::string& ax) const {
  const Json n = engine::axisParamNames(*family, axisOf(ax));
  const std::string smax = n["SMAX"].strOr("");
  if (!smax.empty() && !(jv(params, smax.c_str()) > 0)) return smax + " is 0, so the vehicle does not report its slew rate";
  const LiveAxis& la = axes.at(ax);
  const double now = Loop::now() / 1000;
  if (la.t.empty() || now - la.lastTs > 1) return "no live PID data";
  const SlewStats s = la.slewStats(2, 5, 0.9, std::nullopt, now);
  if (s.n < 10) return "too little live PID data";
  if (s.noSlew == s.n) return "the PID data carry no slew-rate signal (SRate / PDmod)";
  return "";
}

rt::Task<double> Tuner::setParam(const std::string& name, double value, const std::string& reason, std::shared_ptr<Job> job) {
  if (!job) throw rt::Error("internal: write without the write lock");
  if (job->corrective) {
    if (!link.connected()) throw rt::Error(name + " not written: no connection to the vehicle");
  } else {
    checkpoint(*job);
    assertForward(*job);
  }
  const Json from = params[name];
  const bool armed = isArmed();
  double v = 0;
  std::string cancelCode;
  try {
    v = co_await link.setParam(name, value, 1500, 3, [this, job] { return job->corrective || (job->cancelled.empty() && !pendingTrip_); });
  } catch (const rt::Error& e) {
    if (e.code() != "cancelled" && e.code() != "cancelled-sent") throw;
    cancelCode = e.code();
  }
  if (!cancelCode.empty()) {
    if (cancelCode == "cancelled-sent") {
      // an earlier PARAM_SET may have reached the vehicle: ask what it holds now
      Json rb;
      try {
        rb = co_await link.requestParam(name, 1000, 1);
      } catch (const std::exception&) {
        rb = Json();
      }
      if (rb.isNumber()) params.set(name, rb);
      journal.push(Json::obj({{"t", Loop::now()}, {"name", name}, {"from", from}, {"to", rb}, {"requested", value}, {"reason", reason},
                              {"armed", armed}, {"job", job->kind},
                              {"status", rb.isNumber() ? "cancelled, read back" : "uncertain: cancelled, no read-back"}}));
      saveSession();
    }
    checkpoint(*job);   // report the trip / cancel that stopped it
    throw rt::Error(name + " not written: cancelled", "cancelled");
  }
  params.set(name, v);
  journal.push(Json::obj({{"t", Loop::now()}, {"name", name}, {"from", from}, {"to", v}, {"requested", value}, {"reason", reason}, {"armed", armed}, {"job", job->kind}}));
  emit("param", Json::obj({{"name", name}, {"from", from}, {"to", v}, {"reason", reason}}));
  log("param", name + ": " + fmt4(from) + " -> " + fmt4(Json(v)) + " (" + reason + ")");
  saveSession();
  co_return v;
}

void Tuner::saveSession() {
  if (sessionFile_.empty()) return;
  writeText(sessionFile_, Json::obj({{"started", sessionFile_}, {"family", familyJson(family)}, {"originals", originals},
                                     {"lastGood", lastGood}, {"journal", journal}, {"opts", opts}})
                              .dump());
}

rt::Task<void> Tuner::writeGains(const std::string& axis, const Json& gains, const std::string& reason, std::shared_ptr<Job> job) {
  const Json n = engine::axisParamNames(*family, axisOf(axis));
  for (const auto& k : kWriteOrder) {
    if (!gains[k].isNumber() || !n[k].isString()) continue;
    const std::string pname = n[k].str();
    const Json& cur = params[pname];
    if (cur.isNumber() && std::fabs(cur.num() - gains[k].num()) < 1e-9) continue;
    co_await setParam(pname, gains[k].num(), reason, job);
  }
}

Json Tuner::analyze(const std::string& axis, double seconds) {
  if (!family) return Json::obj({{"axis", axis}, {"error", "no data"}});
  const auto data = axes[axis].window(seconds != 0 ? seconds : opts["observeSeconds"].numOr(30));
  if (!data) return Json::obj({{"axis", axis}, {"error", "no data"}});
  const log::Params p = toParams(params);
  const double scaler2 =
      family == Family::Plane ? std::pow(engine::planeSpeedScaler(p, hud["airspeed"].numOr(0), hud["throttle"].numOr(0)), 2) : 1;
  engine::AnalyzeArgs a;
  a.family = *family;
  a.axis = axisOf(axis);
  a.data = &*data;
  a.params = &p;
  a.preset = opts["preset"].strOr("normal");
  a.gyroOp.throttle = hud["throttle"].numOr(0) / 100;
  a.scaler2 = scaler2;
  a.fsLoop = loopHz;
  a.designFrom = hasKey(originals, axis) ? originals[axis] : Json();
  Json res = engine::analyzeAxis(a);
  res.set("dataInfo", Json::obj({{"fs", data->fs}, {"rawRateHz", data->rawRate}, {"seconds", static_cast<double>(data->n) / data->fs}}));
  res.set("at", Loop::now());
  const std::size_t step = std::max<std::size_t>(1, data->n / 1500);
  Json st = Json::array(), tar = Json::array(), act = Json::array(), u = Json::array();
  for (std::size_t i = 0; i < data->n; i += step) {
    st.push(core::jsRoundFixed(data->t0 + static_cast<double>(i) / data->fs, 3));
    tar.push(core::jsRoundFixed(data->target[i], 2));
    act.push(core::jsRoundFixed(data->actual[i], 2));
    u.push(core::jsRoundFixed(data->u[i], 4));
  }
  res.set("series", Json::obj({{"t", st}, {"target", tar}, {"actual", act}, {"u", u}}));
  results.set(axis, res);
  emit("analysis", res);
  return res;
}

// ---------------- session control

void Tuner::beginSession() {
  Json values = Json::object();
  if (family)
    for (const auto& ax : kAxisNames) {
      const Json n = engine::axisParamNames(*family, axisOf(ax));
      for (const auto& k : kWriteOrder)
        if (n[k].isString() && params[n[k].str()].isNumber()) values.set(n[k].str(), params[n[k].str()]);
    }
  for (const auto& k : kSharedParams)
    if (params[k].isNumber()) values.set(k, params[k]);
  sessionBase_ = Json::obj({{"t", Loop::now()}, {"values", values}});
  changes.clear();
  tripLatch_.clear();
  pendingTrip_.reset();
  sessionActive = true;
}

void Tuner::endSession() {
  sessionActive = false;
  pendingTrip_.reset();
}

void Tuner::start() {
  if (job_) throw rt::Error("a change is being written (" + job_->kind + "): wait until it has finished");
  if (state == "idle" || state == "ready" || state == "done" || state == "paused") {
    if (!family || !prepared) throw rt::Error("read the vehicle first (step 2)");
    if (opts["autoApply"].asBool() && isArmed() && !opts["inFlightWrites"].asBool()) throw rt::Error(kInFlightOff);
    if (!sessionActive) beginSession();
    requestStreams();
    axisIdx = 0;
    round = 0;
    for (const auto& a : kAxisNames) axes[a].setBaseline();
    setState("observing", currentAxis() + ": fly normally and tap the sticks (round 1)");
    tick();
  }
}

void Tuner::pause(const std::string& reason) {
  const std::string r = reason.empty() ? "paused by user" : reason;
  cancelJob(r);
  setState("paused", r);
  ramp_.reset();
}

std::string Tuner::currentAxis() const {
  const auto& a = opts["axes"];
  if (!a.isArray() || a.arr().empty()) return "roll";
  return a[static_cast<std::size_t>(axisIdx) % a.size()].strOr("roll");
}

rt::Task<void> Tuner::stop() {
  cancelJob("stopped");
  ramp_.reset();
  loopGen_++;
  if (timer_) Loop::main().clear(timer_);
  timer_ = 0;
  endSession();
  setState("idle", "stopped");
  co_return;
}

rt::Task<void> Tuner::restoreOriginals() {
  pause("restoring original gains");
  co_await takeOver();
  std::vector<Target> targets;
  for (const auto& ax : kAxisNames) {
    if (!hasKey(originals, ax)) continue;
    const Json n = engine::axisParamNames(*family, axisOf(ax));
    const Json& o = originals[ax];
    for (const auto& k : kWriteOrder)
      if (n[k].isString() && o[k].isNumber() && !(k == "SMAX" && !(o[k].num() > 0))) targets.push_back({n[k].str(), o[k].num()});
  }
  co_await userCorrective("restore", targets, "restore original");
  lastGood = originals;
  log("tuner", "original gains restored");
}

rt::Task<void> Tuner::undoAxis(const std::string& axis) {
  if (!hasKey(previous, axis)) throw rt::Error("nothing to undo for " + axis);
  const Json g = previous[axis];
  if (state == "observing" || state == "applying" || state == "analyzing") pause("undo requested");
  co_await takeOver();
  const Json n = engine::axisParamNames(*family, axisOf(axis));
  std::vector<Target> targets;
  for (const auto& k : kWriteOrder)
    if (n[k].isString() && g[k].isNumber() && !(k == "SMAX" && !(g[k].num() > 0))) targets.push_back({n[k].str(), g[k].num()});
  co_await userCorrective("undo", targets, "undo last apply");
  lastGood.set(axis, g);
  saveSession();
}

rt::Task<void> Tuner::takeOver() {
  if (!job_) co_return;
  if (!job_->corrective) {
    cancelJob("taken over by the user");
    job_->abortWatch = true;
  }
  if (!(co_await waitNoJob(15000))) throw rt::Error("the app is still busy writing: try again in a moment");
}

rt::Task<void> Tuner::userCorrective(std::string kind, std::vector<Target> targets, std::string reason) {
  if (!link.alive()) throw rt::Error("no vehicle heartbeat: connect first");
  if (isArmed() && !opts["inFlightWrites"].asBool()) throw rt::Error("the vehicle is armed and " + kInFlightOff);
  auto job = beginJob(kind, true);
  ScopeExit guard{[this, job] { endJob(job); }};
  const CorrectiveResult r = co_await corrective(job, std::move(targets), reason);
  if (!r.failed.empty()) throw rt::Error("not written: " + join(r.failed, ", "));
}

void Tuner::onDisarm() {
  if (state != "idle") pause("vehicle disarmed");
}

rt::Task<bool> Tuner::applyProposal(const std::string& axis) {
  const Json& r = results[axis];
  if (!r.isObject() || !r["proposal"].isObject()) throw rt::Error("no proposal for " + axis);
  if (state == "applying" || job_) throw rt::Error("another change is being written: wait until it has finished");
  if (isArmed() && !opts["inFlightWrites"].asBool()) throw rt::Error(kInFlightOff);
  const Json target = r["proposal"]["gains"];
  auto job = beginJob("apply " + axis, false, true);
  ScopeExit guard{[this, job] { endJob(job); }};
  bool ok = false;
  std::exception_ptr ep;
  std::string code, what;
  try {
    setState("applying", axis);
    ok = co_await applyStaged(axis, target, job);
  } catch (const rt::Error& e) {
    ep = std::current_exception();
    code = e.code();
    what = e.what();
  } catch (const std::exception& e) {
    ep = std::current_exception();
    what = e.what();
  }
  if (ep) {
    if (code == "cancelled") co_return false;
    if (state == "applying") pause(axis + ": apply stopped: " + what);
    std::rethrow_exception(ep);
  }
  co_return ok;
}

rt::Task<bool> Tuner::applyStaged(const std::string& axis, Json target, std::shared_ptr<Job> job) {
  const Json cur = gainsOf(axis);
  previous.set(axis, cur);
  std::vector<std::string> keys;
  for (const char* k : {"P", "I", "D", "FF"})
    if (target[k].isNumber() && cur[k].isNumber()) keys.push_back(k);
  // number of steps so that no multiplicative change exceeds stepMax
  int steps = 1;
  const double stepMax = opts["stepMax"].numOr(1.5);
  for (const auto& k : keys) {
    const double a = jor(cur, k.c_str(), 1e-6), b = jor(target, k.c_str(), 1e-6);
    if (a > 0 && b > 0) steps = std::max(steps, static_cast<int>(std::ceil(std::fabs(std::log(b / a)) / std::log(stepMax) - 1e-9)));
  }
  steps = std::min(steps, 6);
  // filters / smax first (small effect), then the gains stepwise, the angle gain last
  Json pre = Json::object();
  for (const char* k : {"FLTT", "FLTE", "FLTD", "SMAX"})
    if (target[k].isNumber()) pre.set(k, target[k]);
  if (!(co_await change(job, axis, pre, "apply filters", ""))) co_return false;
  for (int s = 1; s <= steps; s++) {
    Json g = Json::object();
    for (const auto& k : keys) {
      const double a = cur[k].num(), b = target[k].num();
      g.set(k, (a > 0 && b > 0) ? a * std::pow(b / a, static_cast<double>(s) / steps) : (s == steps ? b : a + (b - a) * s / steps));
    }
    if (!(co_await change(job, axis, g, "apply step " + std::to_string(s) + "/" + std::to_string(steps), ""))) co_return false;
    lastGood.set(axis, gainsOf(axis));
    saveSession();
  }
  std::vector<std::string> akeys;
  std::vector<std::pair<double, double>> apairs;
  for (const char* k : {"angleP", "tconst"})
    if (target[k].isNumber() && cur[k].isNumber()) {
      akeys.push_back(k);
      apairs.push_back({cur[k].num(), target[k].num()});
    }
  const int asteps = stepsFor(apairs, stepMax);
  for (int s = 1; s <= asteps && !akeys.empty(); s++) {
    Json g = Json::object();
    for (const auto& k : akeys) g.set(k, interp(cur[k].num(), target[k].num(), s, asteps));
    if (!(co_await change(job, axis, g, asteps > 1 ? "apply angle gain " + std::to_string(s) + "/" + std::to_string(asteps) : "apply angle gain", "")))
      co_return false;
    lastGood.set(axis, gainsOf(axis));
    saveSession();
  }
  log("tuner", isArmed() ? axis + ": new gains applied, each change held " + jsToString(opts["watchSeconds"].numOr(4)) + " s without oscillation"
                         : axis + ": new gains written (vehicle disarmed)");
  co_return true;
}

std::vector<std::string> Tuner::sharedAxes() const {
  const double now = Loop::now() / 1000;
  std::vector<std::string> out;
  for (const auto& ax : kAxisNames) {
    bool tuned = false;
    for (const auto& a : opts["axes"].arr())
      if (a.strOr("") == ax) tuned = true;
    const LiveAxis& la = axes.at(ax);
    if (tuned || (!la.t.empty() && now - la.lastTs < 1)) out.push_back(ax);
  }
  return out;
}

rt::Task<bool> Tuner::change(std::shared_ptr<Job> job, std::optional<std::string> axis, Json values, std::string reason, std::string kind) {
  const bool shared = !axis.has_value();
  std::vector<Target> entries;
  if (shared) {
    for (const auto& [name, to] : values.items()) entries.push_back({name, to.num()});
  } else {
    const Json n = engine::axisParamNames(*family, axisOf(*axis));
    for (const auto& k : kWriteOrder)
      if (values[k].isNumber() && n[k].isString()) entries.push_back({n[k].str(), values[k].num()});
  }
  std::vector<Target> todo;
  for (const auto& e : entries) {
    const Json& c = params[e.name];
    if (!(c.isNumber() && std::fabs(c.num() - e.to) < 1e-9)) todo.push_back(e);
  }
  if (todo.empty()) co_return true;
  const std::vector<std::string> watchAxes = shared ? sharedAxes() : std::vector<std::string>{*axis};
  const bool armed = isArmed();
  std::shared_ptr<Change> ch;
  std::exception_ptr err;
  std::string code;
  std::optional<Trip> trip;
  try {
    checkpoint(*job);
    assertForward(*job);
    std::optional<Calm> calm;
    if (armed) {
      for (const auto& ax : watchAxes) {
        const std::string why = watchBlind(ax);
        if (!why.empty()) throw rt::Error(ax + ": " + why + ": the change could not be watched, so it was not written");
      }
      if (!sessionActive) beginSession();
      // nothing is changed while the vehicle shows oscillation: wait a moment for calm, otherwise stop here
      calm = calmBefore();
      const double t0 = Loop::now();
      while (!calm->calm && Loop::now() - t0 < opts["calmWaitSeconds"].numOr(4) * 1000) {
        co_await rt::sleep(250);
        checkpoint(*job);
        calm = calmBefore();
      }
      if (!calm->calm) {
        pause("not calm before \"" + reason + "\" (" + join(calm->reasons, "; ") + "): nothing more is written. Land if it keeps shaking");
        co_return false;
      }
    }
    ch = std::make_shared<Change>();
    ch->id = ++changeSeq_;
    ch->t = Loop::now();
    ch->kind = !kind.empty() ? kind : shared ? "filters" : "gains";
    ch->axis = axis;
    ch->axes = watchAxes;
    ch->shared = shared;
    ch->reason = reason;
    ch->armed = armed;
    for (const auto& e : todo) ch->params.push_back({e.name, params[e.name], e.to});
    ch->calmBefore = calm;
    ch->status = armed ? "writing" : "written";
    changes.push_back(ch);
    if (changes.size() > 200) changes.erase(changes.begin());
    for (const auto& e : todo) {
      co_await setParam(e.name, e.to, reason, job);
      ch->written.push_back(e.name);
    }
  } catch (const TripError& e) {
    trip = e.trip;
    code = "trip";
  } catch (const rt::Error& e) {
    err = std::current_exception();
    code = e.code();
  } catch (const std::exception&) {
    err = std::current_exception();
  }
  if (code == "trip") {
    if (ch) ch->status = "partial";
    co_await handleTrip(job, *trip, ch);
    co_return false;
  }
  if (err) {
    // stopped or failed midway in flight: the part that was written is watched like any change
    if (!ch || !armed || ch->written.empty()) {
      if (code == "cancelled") co_return false;
      std::rethrow_exception(err);
    }
    ch->status = "partial";
    const WatchResult res = co_await watchChange(job, ch);
    if (!res.ok) {
      co_await handleTrip(job, *res.trip, ch);
      co_return false;
    }
    if (code == "cancelled") co_return false;
    std::rethrow_exception(err);
  }
  if (!armed) co_return true;
  const WatchResult res = co_await watchChange(job, ch);
  if (res.ok) {
    ch->status = res.aborted ? "watch ended early" : "held";
    co_return true;
  }
  co_await handleTrip(job, *res.trip, ch);
  co_return false;
}

rt::Task<bool> Tuner::applyFilterChange(Json values, std::string reason) {
  if (job_) throw rt::Error("a change is being written (" + job_->kind + "): wait until it has finished");
  auto job = beginJob("filters", false, true);
  ScopeExit guard{[this, job] { endJob(job); }};
  co_return co_await change(job, std::nullopt, values, reason, "notch");
}

rt::Task<bool> Tuner::watchExternalChange(Json changed, std::string reason) {
  std::vector<std::string> names;
  for (const auto& [k, c] : changed.items()) names.push_back(k);
  if (!isArmed() || names.empty()) co_return true;
  if (job_ && !(co_await waitNoJob(15000))) throw rt::Error("the app is still busy writing");
  auto job = beginJob("watch", false, true);
  ScopeExit guard{[this, job] { endJob(job); }};
  if (!sessionActive) beginSession();
  auto ch = std::make_shared<Change>();
  ch->id = ++changeSeq_;
  ch->t = Loop::now();
  ch->kind = "autopilot";
  ch->axes = sharedAxes();
  ch->shared = true;
  ch->reason = reason;
  ch->armed = true;
  for (const auto& [k, c] : changed.items()) ch->params.push_back({k, c["from"], c["to"].numOr(0)});
  ch->written = names;
  ch->calmBefore = calmBefore();
  ch->status = "watching";
  changes.push_back(ch);
  if (changes.size() > 200) changes.erase(changes.begin());
  const WatchResult res = co_await watchChange(job, ch);
  if (res.ok) {
    ch->status = "held";
    co_return true;
  }
  co_await handleTrip(job, *res.trip, ch);
  co_return false;
}

Calm Tuner::calmBefore() const {
  const double now = Loop::now() / 1000;
  Calm c;
  c.t = Loop::now();
  const double oscSmax = opts["oscSmax"].numOr(5);
  for (const auto& ax : kAxisNames) {
    const LiveAxis& la = axes.at(ax);
    if (la.t.empty() || now - la.lastTs > 1) continue;
    const SlewStats s = la.slewStats(2, oscSmax, 0.9, std::nullopt, now);
    if (s.bad) c.reasons.push_back(ax + ": slew rate " + jsToFixed(s.srateMax, 1) + (s.dmodMin < 0.9 ? ", Dmod " + jsToFixed(s.dmodMin, 2) : ""));
  }
  c.calm = c.reasons.empty();
  return c;
}

rt::Future<Tuner::WatchResult> Tuner::watchChange(std::shared_ptr<Job> job, std::shared_ptr<Change> ch) {
  rt::Promise<WatchResult> p;
  const double dur = ch->shared ? std::max(opts["watchSeconds"].numOr(4), opts["filterWatchSeconds"].numOr(10)) : opts["watchSeconds"].numOr(4);
  std::map<std::string, double> rate0;
  for (const auto& ax : ch->axes) rate0[ax] = axes.at(ax).rateHz();
  const double since = Loop::now() / 1000 + opts["watchGrace"].numOr(0.5);
  ch->status = "watching";
  auto iv = std::make_shared<std::uint64_t>(0);
  *iv = Loop::main().setInterval(250, [this, p, iv, job, ch, dur, rate0, since] {
    auto done = [&](bool ok, std::vector<std::string> reasons = {}, const std::string& ax = "", bool failClosed = false, bool aborted = false) {
      Loop::main().clear(*iv);
      WatchResult r;
      r.ok = ok;
      r.aborted = aborted;
      if (!ok) r.trip = Trip{"watch", ax, std::move(reasons), failClosed, Loop::now()};
      p.resolve(r);
    };
    const double now = Loop::now() / 1000;
    if (job->abortWatch) return done(true, {}, "", false, true);
    if (pendingTrip_) {
      Trip t = *pendingTrip_;
      pendingTrip_.reset();
      return done(false, t.reasons, t.axis);
    }
    if (!link.alive()) return done(false, {"the link to the vehicle was lost during the watch"}, ch->axes.empty() ? "" : ch->axes[0], true);
    const double oscSmax = opts["oscSmax"].numOr(5);
    for (const auto& ax : ch->axes) {
      const LiveAxis& la = axes.at(ax);
      if (now >= since + 1 && now - la.lastTs > 1)
        return done(false, {ax + ": no PID data for " + jsToFixed(now - la.lastTs, 1) + " s after the change"}, ax, true);
      const Watchdog w = la.watchdog(2, oscSmax, 8, 0.9, since);
      if (w.oscillating) {
        std::vector<std::string> rs;
        for (const auto& r : w.reasons) rs.push_back(ax + ": " + r);
        return done(false, rs, ax);
      }
    }
    if (auto att = attitudeTrip(ch->axes, since)) return done(false, att->reasons, att->axis);
    if (now >= since + dur) {
      // fail closed: the watch only counts if enough samples came in after the change
      for (const auto& ax : ch->axes) {
        const int n = axes.at(ax).slewStats(dur + 1, 5, 0.9, since, now).n;
        const auto it = rate0.find(ax);
        const double r0 = it != rate0.end() ? it->second : 0;
        const double need = std::max(10.0, std::floor(0.5 * r0 * dur));
        if (n < need)
          return done(false,
                      {n ? ax + ": only " + jsToString(n) + " PID samples after the change (" + jsToString(need) + " needed to judge it)"
                         : ax + ": no PID data after the change"},
                      ax, true);
      }
      return done(true);
    }
  });
  return p.future();
}

rt::Task<Tuner::CheckResult> Tuner::watchCorrective(std::vector<std::string> watchAxes) {
  std::map<std::string, double> rate0;
  for (const auto& ax : watchAxes) rate0[ax] = axes.at(ax).rateHz();
  const double grace = opts["watchGrace"].numOr(0.5), watchS = opts["watchSeconds"].numOr(4), monS = opts["monitorSeconds"].numOr(2);
  const double since = Loop::now() / 1000 + grace;
  co_await rt::sleep((grace + watchS) * 1000);
  const double now = Loop::now() / 1000;
  if (!link.alive()) co_return CheckResult{false, {"the link to the vehicle is lost: the result is not known"}};
  for (const auto& ax : watchAxes) {
    const SlewStats s = axes.at(ax).slewStats(monS, opts["oscSmax"].numOr(5), 0.9, since, now);
    const double need = std::max(10.0, std::floor(0.5 * rate0[ax] * std::min(monS, watchS)));
    if (s.n < need) co_return CheckResult{false, {ax + ": only " + jsToString(s.n) + " PID samples after the correction: the result is not known"}};
    if (s.frac >= opts["monitorFraction"].numOr(0.5))
      co_return CheckResult{false, {ax + ": slew rate " + jsToFixed(s.srateMax, 1) + " in " + jsToString(core::jsRound(100 * s.frac)) + " % of the last " +
                                    jsToString(monS) + " s"}};
  }
  co_return CheckResult{true, {}};
}

std::optional<Trip> Tuner::sustained(const std::string& ax) const {
  const double monS = opts["monitorSeconds"].numOr(2), oscSmax = opts["oscSmax"].numOr(5);
  const SlewStats s = axes.at(ax).slewStats(monS, oscSmax, 0.9, std::nullopt, Loop::now() / 1000);
  if (s.n >= 10 && s.frac >= opts["monitorFraction"].numOr(0.5) && s.badLast - s.badFirst >= 0.4 * monS)
    return Trip{"monitor", ax,
                {ax + " oscillating: slew rate " + jsToFixed(s.srateMax, 1) + " (limit " + jsToString(oscSmax) + ")" +
                 (s.dmodMin < 0.9 ? ", Dmod " + jsToFixed(s.dmodMin, 2) : "") + " in " + jsToString(core::jsRound(100 * s.frac)) + " % of the last " +
                 jsToString(monS) + " s"},
                false, Loop::now()};
  return std::nullopt;
}

std::optional<Trip> Tuner::attitudeTrip(const std::vector<std::string>& watchAxes, double since) const {
  const double now = Loop::now() / 1000, lim = opts["angleAbort"].numOr(10);
  if (attErr_.empty() || now - attErr_.back().t > 1) return std::nullopt;   // no attitude target streaming: inactive
  const AttSample& last = attErr_.back();
  for (const auto& ax : watchAxes) {
    if (ax != "roll" && ax != "pitch") continue;
    int n = 0;
    double oldest = std::nan(""), worst = 0;
    for (auto it = attErr_.rbegin(); it != attErr_.rend() && it->t >= since; ++it) {
      const double e = ax == "roll" ? it->roll : it->pitch;
      if (!(std::fabs(e) > lim)) break;
      n++;
      oldest = it->t;
      worst = std::max(worst, std::fabs(e));
    }
    if (n >= 3 && last.t - oldest >= opts["angleAbortSeconds"].numOr(0.8))
      return Trip{"watch", ax,
                  {ax + ": attitude error " + jsToFixed(worst, 1) + " deg for " + jsToFixed(last.t - oldest, 1) + " s (limit " + jsToString(lim) + " deg)"},
                  false, Loop::now()};
  }
  return std::nullopt;
}

// ---------------- trips: the evidence-based revert

void Tuner::raiseTrip(Trip trip) {
  if (job_ && job_->corrective) return;   // already reverting
  if (!pendingTrip_) pendingTrip_ = std::move(trip);
  if (!job_) startCorrective();
}

void Tuner::startCorrective() {
  if (!pendingTrip_) return;
  Trip trip = *pendingTrip_;
  pendingTrip_.reset();
  auto job = beginJob("revert", true);
  rt::Future<void> f = handleTrip(job, trip, nullptr);
  f.then([this, f, job] {
    if (f.error()) {
      try {
        std::rethrow_exception(f.error());
      } catch (const std::exception& e) {
        log("error", std::string("revert failed: ") + e.what());
      }
    }
    endJob(job);
  });
}

bool Tuner::correctiveAllowed() const {
  if (!isArmed() || opts["inFlightWrites"].asBool()) return true;
  for (const auto& c : changes)
    if (c->armed) return true;
  return false;
}

std::vector<Tuner::Target> Tuner::sessionDiff() const {
  std::vector<Target> out;
  if (!sessionBase_.isObject()) return out;
  for (const auto& [name, to] : sessionBase_["values"].items()) {
    const Json& cur = params[name];
    if (!cur.isNumber() || std::fabs(cur.num() - to.num()) <= 1e-9 * std::max(1.0, std::fabs(to.num()))) continue;
    if (name.size() >= 5 && name.compare(name.size() - 5, 5, "_SMAX") == 0 && !(to.num() > 0)) continue;   // never blind the detector
    out.push_back({name, to.num()});
  }
  return out;
}

rt::Task<void> Tuner::handleTrip(std::shared_ptr<Job> job, Trip trip, std::shared_ptr<Change> ch) {
  job->corrective = true;
  const std::string what = join(trip.reasons, "; ");
  const std::string after = ch ? " after \"" + ch->reason + "\"" : "";
  Json rs = Json::array();
  for (const auto& r : trip.reasons) rs.push(r);
  lastTrip = Json::obj({{"source", trip.source}, {"axis", trip.axis}, {"reasons", rs}, {"failClosed", trip.failClosed}, {"t", trip.t},
                        {"change", ch ? Json(ch->id) : Json()}, {"at", Loop::now()}});
  for (const auto& ax : kAxisNames) tripLatch_[ax] = true;
  if (ch) ch->status = "tripped";
  pause(what + after + ": oscillation detected, reverting");
  log("watchdog", what + after);
  emit("trip", lastTrip);
  if (!correctiveAllowed()) {
    pause(what + ". Changes in flight are off, so the app changed nothing in the air: land and check the gains");
    co_return;
  }
  auto listOf = [](const std::vector<Target>& w) {
    std::string s;
    for (const auto& e : w) s += (s.empty() ? "" : ", ") + e.name + " " + fmt4(Json(e.to));
    return s;
  };
  std::optional<std::pair<std::string, std::string>> reverted;   // what the revert of the change wrote, and why it was not enough
  if (ch && ch->calmBefore && ch->calmBefore->calm) {
    std::vector<Target> targets;
    for (const auto& p : ch->params)
      if (p.from.isNumber()) targets.push_back({p.name, p.from.num()});
    const CorrectiveResult r = co_await corrective(job, targets, "revert: oscillation" + after);
    std::vector<std::string> wa;
    for (const auto& ax : kAxisNames)
      if (inList(ch->axes, ax) || ax == trip.axis) wa.push_back(ax);
    const CheckResult w = co_await watchCorrective(wa);
    std::string list = listOf(r.written);
    if (list.empty()) list = "nothing to write";
    if (w.ok && r.failed.empty()) {
      pause(what + after + ": the change was reverted (" + list + ") and the vehicle is calm again. Press Resume to continue, or land");
      co_return;
    }
    reverted = std::make_pair(list, !r.failed.empty() ? "not written: " + join(r.failed, ", ") : join(w.reasons, "; "));
    log("watchdog", "after the revert (" + list + "): " + reverted->second + ": restoring the values from the start of the session");
  }
  const std::vector<Target> targets = sessionDiff();
  if (targets.empty()) {
    pause(reverted ? what + after + ": the change was reverted (" + reverted->first + "), but " + reverted->second + ": land now"
                   : what + after + ": the vehicle has the values it started the session with, so there is nothing to go back to. Land now and lower the gains");
    co_return;
  }
  const CorrectiveResult r = co_await corrective(job, targets, "restore the session start values: oscillation");
  std::vector<std::string> wa;
  for (const auto& ax : kAxisNames)
    if (!axes.at(ax).t.empty()) wa.push_back(ax);
  const CheckResult w = co_await watchCorrective(wa);
  const std::string list = listOf(r.written);
  if (!r.failed.empty())
    pause(what + after + ": restoring the values from the start of the session failed for " + join(r.failed, ", ") + ": land now");
  else if (w.ok)
    pause(what + after + ": the values from the start of the session were restored (" + list + ") and the vehicle is calm again. Land and check, or press Resume");
  else
    pause(what + after + ": still oscillating with the values from the start of the session (" + join(w.reasons, "; ") + "): land now");
}

rt::Task<Tuner::CorrectiveResult> Tuner::corrective(std::shared_ptr<Job> job, std::vector<Target> targets, std::string reason) {
  std::map<std::string, std::string> gainKey;
  if (family)
    for (const auto& ax : kAxisNames) {
      const Json n = engine::axisParamNames(*family, axisOf(ax));
      for (const auto& k : kSteppedKeys)
        if (n[k].isString()) gainKey[n[k].str()] = k;
    }
  struct Step {
    std::string name;
    double from, to;
  };
  std::vector<Step> stepped;
  std::vector<Target> direct;
  for (const auto& t : targets) {
    const Json& cur = params[t.name];
    if (cur.isNumber() && std::fabs(cur.num() - t.to) <= 1e-9 * std::max(1.0, std::fabs(t.to))) continue;
    const auto it = gainKey.find(t.name);
    if (it != gainKey.end() && cur.isNumber() && ((cur.num() > 0 && t.to > 0) || inList(kLinearKeys, it->second)))
      stepped.push_back({t.name, cur.num(), t.to});
    else
      direct.push_back(t);
  }
  std::stable_sort(direct.begin(), direct.end(),
                   [](const Target& a, const Target& b) { return a.name == "INS_HNTCH_ENABLE" && b.name != "INS_HNTCH_ENABLE"; });
  CorrectiveResult out;
  auto record = [&](const std::string& name, double v) {
    for (auto& e : out.written)
      if (e.name == name) {
        e.to = v;
        return;
      }
    out.written.push_back({name, v});
  };
  for (const auto& t : direct) {
    std::string msg;
    bool ok = true;
    try {
      co_await setParam(t.name, t.to, reason, job);
    } catch (const std::exception& e) {
      ok = false;
      msg = e.what();
    }
    if (ok) {
      record(t.name, t.to);
    } else {
      if (!inList(out.failed, t.name)) out.failed.push_back(t.name);
      log("error", t.name + ": " + msg);
    }
  }
  std::vector<std::pair<double, double>> pairs;
  for (const auto& s : stepped) pairs.push_back({s.from, s.to});
  const int steps = stepsFor(pairs, opts["stepMax"].numOr(1.5));
  for (int k = 1; k <= steps && !stepped.empty(); k++) {
    if (k > 1) co_await rt::sleep(opts["revertStepSeconds"].numOr(1) * 1000);
    for (const auto& s : stepped) {
      if (inList(out.failed, s.name)) continue;
      const double v = interp(s.from, s.to, k, steps);
      std::string msg;
      bool ok = true;
      try {
        co_await setParam(s.name, v, reason, job);
      } catch (const std::exception& e) {
        ok = false;
        msg = e.what();
      }
      if (ok) {
        record(s.name, v);
      } else {
        if (!inList(out.failed, s.name)) out.failed.push_back(s.name);
        log("error", s.name + ": " + msg);
      }
    }
  }
  co_return out;
}

// ---------------- the oscillation monitor (every axis, every 250 ms)

void Tuner::startMonitor() {
  if (monitorTimer_) return;
  monitorTimer_ = Loop::main().setInterval(250, [this] {
    try {
      monitorTick();
    } catch (const std::exception& e) {
      log("error", std::string("monitor: ") + e.what());
    }
  });
}

void Tuner::stopMonitor() {
  if (monitorTimer_) Loop::main().clear(monitorTimer_);
  monitorTimer_ = 0;
}

void Tuner::monitorTick() {
  const double now = Loop::now() / 1000;
  const double monS = opts["monitorSeconds"].numOr(2), oscSmax = opts["oscSmax"].numOr(5);
  Json ax = Json::object();
  std::map<std::string, std::string> states;
  for (const auto& a : kAxisNames) {
    const LiveAxis& la = axes.at(a);
    const SlewStats s = la.slewStats(monS, oscSmax, 0.9, std::nullopt, now);
    std::string st;
    if (!family) {
      st = "no vehicle";
    } else {
      const Json n = engine::axisParamNames(*family, axisOf(a));
      const std::string smax = n["SMAX"].strOr("");
      if (la.t.empty() || now - la.lastTs > 1 || s.n < 10)
        st = "no data";
      else if (!smax.empty() && !(jv(params, smax.c_str()) > 0))
        st = "blind: SMAX is 0";
      else if (s.noSlew == s.n)
        st = "blind: no slew-rate data";
      else if (sustained(a))
        st = "oscillating";
      else
        st = s.bad ? "unsettled" : "calm";
    }
    if (st == "calm") tripLatch_[a] = false;
    states[a] = st;
    ax.set(a, Json::obj({{"state", st}, {"srateMax", core::jsRoundFixed(s.srateMax, 2)}, {"dmodMin", core::jsRoundFixed(s.dmodMin, 3)},
                         {"frac", core::jsRoundFixed(s.frac, 2)}, {"samples", s.n}}));
  }
  const bool attActive = !attErr_.empty() && now - attErr_.back().t < 1;
  monitor = Json::obj({{"active", sessionActive},
                       {"axes", ax},
                       {"attitude", Json::obj({{"active", attActive},
                                               {"roll", attErr_.empty() ? Json() : Json(core::jsRoundFixed(attErr_.back().roll, 1))},
                                               {"pitch", attErr_.empty() ? Json() : Json(core::jsRoundFixed(attErr_.back().pitch, 1))}})},
                       {"lastTrip", lastTrip}});
  if (!sessionActive) return;
  for (const auto& a : kAxisNames)
    if (states[a] == "oscillating" && !tripLatch_[a]) {
      if (auto t = sustained(a)) raiseTrip(*t);
      return;
    }
  if (pendingTrip_ && !job_) startCorrective();
}

// ---------------- the observe / apply loop (one loop only: an older loop ends at its next turn)

void Tuner::tick() {
  if (timer_) Loop::main().clear(timer_);
  const int gen = ++loopGen_;
  timer_ = Loop::main().setTimeout(500, [this, gen] { tickLoop(gen); });
}

void Tuner::tickLoop(int gen) {
  if (gen != loopGen_) return;
  timer_ = 0;
  rt::Future<void> f = tickStep();
  f.then([this, gen] {
    if (gen == loopGen_ && (state == "observing" || state == "ramping"))
      timer_ = Loop::main().setTimeout(state == "ramping" ? 250 : 1000, [this, gen] { tickLoop(gen); });
  });
}

rt::Task<void> Tuner::tickStep() {
  std::string code, what;
  bool failed = false;
  try {
    if (state == "observing")
      co_await observeStep();
    else if (state == "ramping")
      co_await rampStep();
  } catch (const rt::Error& e) {
    code = e.code();
    what = e.what();
    failed = true;
  } catch (const std::exception& e) {
    what = e.what();
    failed = true;
  }
  if (failed && code != "cancelled") {
    log("error", what);
    pause("error: " + what);
  }
}

rt::Task<void> Tuner::observeStep() {
  const std::string axis = currentAxis();
  LiveAxis& la = axes[axis];
  const double elapsed = (Loop::now() - stateSince) / 1000;
  const double exc = la.excitation(20);
  // an oscillation is caught by the monitor (every axis, every 250 ms) while a session is active
  emit("observe", Json::obj({{"axis", axis}, {"seconds", la.seconds()}, {"excitation", exc}, {"rateHz", la.rateHz()}, {"elapsed", elapsed}}));
  const double observeS = opts["observeSeconds"].numOr(30);
  // a fresh analysis (from the flight check-up, < 90 s old) is used directly in the first round
  const Json& prev = results[axis];
  const bool fresh = opts["useFreshResult"].asBool() && round == 0 && prev.isObject() && !prev["used"].asBool() &&
                     Loop::now() - prev["at"].numOr(0) < 90000;
  if (!fresh) {
    if (elapsed < observeS) co_return;
    if (la.seconds() < observeS * 0.8) {
      stateInfo = axis + ": waiting for telemetry (" + jsToFixed(la.seconds(), 0) + " s buffered at " + jsToFixed(la.rateHz(), 0) + " Hz)";
      co_return;
    }
  }
  setState("analyzing", axis);
  Json res;
  if (fresh) {
    results.at(axis).set("used", true);
    res = results[axis];
  } else {
    res = analyze(axis);
  }
  if (!res["proposal"].isObject()) {
    // no model: when the slew rate / Dmod show oscillation, say so and stop (never "fly sharper" while it shakes, F-03)
    const double oscSmax = opts["oscSmax"].numOr(5);
    const SlewStats os = la.slewStats(observeS, oscSmax, 0.9, std::nullopt, Loop::now() / 1000);
    if (os.n >= 10 && os.frac >= kOscSeenFraction) {
      const std::string why = axis + " oscillating: slew rate up to " + jsToFixed(os.srateMax, 1) + " (limit " + jsToString(oscSmax) + ")" +
                              (os.dmodMin < 0.9 ? ", Dmod down to " + jsToFixed(os.dmodMin, 2) : "") + " in " +
                              jsToString(core::jsRound(100 * os.frac)) + " % of the last " + jsToString(observeS) + " s, so no gains were worked out";
      raiseTrip(Trip{"observe", axis, {why}, false, Loop::now()});
      if (state == "analyzing") pause(why + ". Land if it keeps shaking");
      co_return;
    }
    round++;
    if (opts["allowRamp"].asBool() && round >= 2) {
      startRamp(axis);
      co_return;
    }
    std::string why = res["identification"]["reason"].strOr("");
    if (why.empty() && res["warnings"].isArray() && res["warnings"].size()) why = res["warnings"][std::size_t{0}].strOr("");
    if (why.empty()) why = res["error"].strOr("");
    if (why.empty()) why = "no proposal";
    setState("observing", axis + ": " + why + " (round " + std::to_string(round + 1) + ") - fly sharper stick inputs");
    co_return;
  }
  // converged when the rate gains move < 10% AND the angle gain / time constant (capped at x1.5) < 15%
  bool big = false;
  for (const auto& c : res["proposal"]["changes"].arr()) {
    const std::string k = c["key"].strOr("");
    const double pct = std::fabs(c["pct"].numOr(0));
    if (((k == "P" || k == "D" || k == "FF") && pct > 10) || ((k == "angleP" || k == "tconst") && pct > 15)) big = true;
  }
  if (!big) {
    log("tuner", axis + ": converged (rate gains < 10%, angle gain < 15%)");
    nextAxis();
    co_return;
  }
  if (opts["autoApply"].asBool()) {
    if (isArmed() && !opts["inFlightWrites"].asBool()) {
      pause(axis + ": " + kInFlightOff);
      co_return;
    }
    if (job_) {
      setState("observing", axis + ": waiting for another change to finish (" + job_->kind + ")");
      co_return;
    }
    std::string list;
    for (const auto& c : res["proposal"]["changes"].arr())
      list += (list.empty() ? "" : ", ") + c["key"].strOr("") + " " + fmt4(c["from"]) + "->" + fmt4(c["to"]);
    setState("applying", axis + ": " + list);
    auto job = beginJob("apply " + axis, false, true);
    bool ok = false;
    {
      ScopeExit guard{[this, job] { endJob(job); }};
      ok = co_await applyStaged(axis, res["proposal"]["gains"], job);
    }
    if (ok && state == "applying") {
      round++;
      if (round >= 4)
        nextAxis();
      else
        setState("observing", axis + ": re-checking with the new gains (round " + std::to_string(round + 1) + ")");
    }
  } else {
    setState("proposed", axis + ": review the proposal and press Apply");
  }
}

void Tuner::nextAxis() {
  axisIdx++;
  round = 0;
  if (static_cast<std::size_t>(axisIdx) >= opts["axes"].size()) {
    setState("done", "all axes tuned");
    return;
  }
  setState("observing", currentAxis() + ": fly normally and tap the sticks (round 1)");
}

void Tuner::resume() {
  if (job_) throw rt::Error("a change is still being written (" + job_->kind + "): wait until it has finished");
  if (state == "proposed" || state == "paused" || state == "applying") {
    setState("observing", currentAxis() + ": observing (round " + std::to_string(round + 1) + ")");
    tick();
  }
}

// ---------------- QuikTune-style ramp (VTOL-quicktune.lua logic)

void Tuner::startRamp(const std::string& axis) {
  const Family fam = *family == Family::Heli ? Family::Copter : *family;
  const Json cur = engine::currentGains(fam, axisOf(axis), toParams(params));
  const core::Filters f = core::filterRules(fam, axisOf(axis), core::AxisGainsIn{jv(cur, "FLTT"), jv(cur, "FLTE"), jv(cur, "FLTD")},
                                            jor(params, "INS_GYRO_FILTER", 20), 0.5);
  Ramp r;
  r.axis = axis;
  r.stage = "D";
  r.names = engine::axisParamNames(*family, axisOf(axis));
  r.saved = cur;
  r.lastChange = Loop::now();
  r.filters = Json::obj({{"FLTT", f.FLTT}, {"FLTE", f.FLTE}, {"FLTD", f.FLTD}});
  ramp_ = r;
  setState("ramping", axis + ": QuikTune-style ramp, raising D (doubling every " + jsToString(opts["rampDoubleTime"].numOr(10)) + " s)");
}

rt::Task<void> Tuner::rampStep() {
  if (job_) co_return;
  auto job = beginJob("ramp", false, true);
  ScopeExit guard{[this, job] { endJob(job); }};
  std::optional<Trip> trip;
  try {
    co_await rampWrite(job);
  } catch (const TripError& e) {
    trip = e.trip;
  }
  if (trip) co_await handleTrip(job, *trip, nullptr);
}

rt::Task<void> Tuner::rampWrite(std::shared_ptr<Job> job) {
  if (!ramp_) {
    setState("observing", "ramp ended");
    co_return;
  }
  Ramp& r = *ramp_;
  LiveAxis& la = axes[r.axis];
  const double now = Loop::now();
  if (!r.filtersSet) {
    const std::string ax = r.axis;
    const Json fl = r.filters;
    co_await writeGains(ax, fl, "QuikTune filter rule", job);
    if (ramp_) ramp_->filtersSet = true;
    co_return;
  }
  // pilot input pauses the ramp (QuikTune PILOT_INPUT_DELAY = 4 s)
  if (std::fabs(rc["roll"].numOr(0)) > 0.05 || std::fabs(rc["pitch"].numOr(0)) > 0.05 || std::fabs(rc["yaw"].numOr(0)) > 0.05) r.lastPilot = now;
  if (r.lastPilot != 0 && now - r.lastPilot < 4000) co_return;
  if (now - r.lastChange < 4000) co_return;   // STAGE_DELAY after a stage change
  const std::string key = r.stage;
  const std::string pname = r.names[key].str();
  const double cur = params[pname].numOr(0);
  const Watchdog w = la.watchdog(1.5, opts["oscSmax"].numOr(5));
  const core::GainLimits gl = core::gainLimits(*family == Family::Heli ? Family::Copter : *family, axisOf(r.axis));
  const double* lim = key == "D" ? gl.D : gl.P;
  const bool limited = cur >= lim[1];
  if (w.oscillating || limited) {
    const double reduction = w.oscillating ? (100 - opts["rampMargin"].numOr(60)) / 100 : 1;
    const double nv = std::min(cur * reduction, lim[1]);
    const double saved = r.saved[key].numOr(0);
    const std::string axis = r.axis;
    const Json names = r.names;
    if (nv < saved && key == "D" && axis != "yaw") {
      // lowering D below the original: also lower P (QuikTune rule, not below a factor of 2)
      const double ratio = std::max(nv / saved, 0.5);
      co_await setParam(names["P"].str(), params[names["P"].str()].numOr(0) * ratio, "QuikTune: P lowered with D", job);
    }
    co_await setParam(pname, nv, std::string("QuikTune: ") + (w.oscillating ? "oscillation at " + fmt4(Json(cur)) : "limit reached") + ", back off", job);
    if (key == "P") co_await coupleI(axis, nv, job);
    lastGood.set(axis, gainsOf(axis));
    saveSession();
    if (!ramp_) co_return;
    ramp_->lastChange = now;
    if (ramp_->stage == "D") {
      ramp_->stage = "P";
      stateInfo = axis + ": D done (" + fmt4(Json(nv)) + "), now raising P";
    } else {
      log("tuner", axis + ": ramp done P=" + fmt4(params[names["P"].str()]) + " D=" + fmt4(params[names["D"].str()]));
      ramp_.reset();
      nextAxis();
    }
    co_return;
  }
  // raise the gain: double every rampDoubleTime seconds, writing twice a second
  if (now < r.nextWrite) co_return;
  r.nextWrite = now + 500;
  const double mul = std::exp(std::log(2.0) / (2 * opts["rampDoubleTime"].numOr(10)));
  double nv = std::max(cur * mul, 0.001);
  // never reduce below the original by more than maxReducePct (QUIK_MAX_REDUCE)
  const double minAllowed = r.saved[key].numOr(0) * (100 - opts["maxReducePct"].numOr(20)) / 100;
  if (nv < minAllowed) nv = minAllowed;
  const std::string axis = r.axis;
  co_await setParam(pname, nv, "QuikTune ramp " + key, job);
  if (key == "P") co_await coupleI(axis, nv, job);
}

rt::Task<void> Tuner::coupleI(const std::string& axis, double P, std::shared_ptr<Job> job) {
  const Json n = engine::axisParamNames(*family, axisOf(axis));
  const double ff = jor(params, n["FF"].str().c_str(), 0);
  if (ff > 0) co_return;   // with FF the I gain is tied to FF, not P
  const double I = core::iRule(*family == Family::Heli ? Family::Copter : *family, axisOf(axis), P, ff, false);
  co_await setParam(n["I"].str(), I, "I follows P", job);
}

Json Tuner::changeJson(const Change& c) const {
  Json ps = Json::array();
  for (const auto& p : c.params) ps.push(Json::obj({{"name", p.name}, {"from", p.from}, {"to", p.to}}));
  return Json::obj({{"id", c.id}, {"t", c.t}, {"kind", c.kind}, {"axis", c.axis ? Json(*c.axis) : Json()}, {"reason", c.reason}, {"armed", c.armed},
                    {"status", c.status}, {"params", ps}});
}

Json Tuner::snapshot() const {
  Json rates = Json::object();
  for (const auto& a : kAxisNames) {
    const LiveAxis& la = axes.at(a);
    rates.set(a, Json::obj({{"hz", core::jsRoundFixed(la.rateHz(), 1)},
                            {"seconds", core::jsRoundFixed(la.seconds(), 0)},
                            {"excitation", core::jsRoundFixed(la.excitation(20), 1)},
                            {"watchdog", la.watchdog(2, opts["oscSmax"].numOr(5)).toJson()}}));
  }
  Json gains;
  if (family) {
    gains = Json::object();
    for (const auto& a : kAxisNames) gains.set(a, gainsOf(a));
  }
  const auto& ls = link.stats();
  const auto& ps = link.parserStats();
  Json j = journal, m = messages;
  if (j.size() > 50) j.arr().erase(j.arr().begin(), j.arr().end() - 50);
  if (m.size() > 40) m.arr().erase(m.arr().begin(), m.arr().end() - 40);
  Json ch = Json::array();
  for (std::size_t i = changes.size() > 10 ? changes.size() - 10 : 0; i < changes.size(); i++) ch.push(changeJson(*changes[i]));
  return Json::obj({{"state", state},
                    {"info", stateInfo},
                    {"family", familyJson(family)},
                    {"loopHz", loopHz},
                    {"opts", opts},
                    {"axis", currentAxis()},
                    {"round", round},
                    {"attitude", attitude},
                    {"rc", rc},
                    {"hud", hud},
                    {"vehicle", link.vehicle()},
                    {"linkStats", Json::obj({{"rx", ls.rx}, {"tx", ls.tx}, {"bytes", ls.bytes}})},
                    {"parserStats", Json::obj({{"frames", ps.frames}, {"badCrc", ps.badCrc}, {"unknown", ps.unknown}, {"dropped", ps.dropped}})},
                    {"msgRates", link.msgRates()},
                    {"rates", rates},
                    {"gains", gains},
                    {"originals", originals},
                    {"previous", previous},
                    {"prepared", prepared},
                    {"pendingWrites", pendingWrites},
                    {"vibe", vibe},
                    {"monitor", monitor},
                    {"sessionActive", sessionActive},
                    {"writing", job_ ? Json(job_->kind) : Json()},
                    {"changes", ch},
                    {"journal", j},
                    {"messages", m}});
}

}  // namespace aptuner::app
