#include "tuner.hpp"

#include <algorithm>
#include <cmath>

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

namespace {
constexpr int kPidTuningId = 194, kAttitudeId = 30;
// after a gain change the watchdog ignores this many seconds: ArduPilot's slew-rate signal looks back ~0.3 s
// and telemetry arrives late, so those samples still describe the gains before the change
constexpr double kWatchGraceS = 0.5;

/** JS `a > b * f` for values that may be missing (a missing value compares false) */
bool above(const Json& a, const Json& b, double f) { return a.isNumber() && b.isNumber() && a.num() > b.num() * f; }
bool below(const Json& a, const Json& b, double f) { return a.isNumber() && b.isNumber() && a.num() < b.num() * f; }

core::Axis axisOf(const std::string& a) { return *core::parseAxis(a); }

std::string str(const Json& v) {
  if (v.isNumber()) return jsToString(v.num());
  if (v.isString()) return v.str();
  if (v.isNull()) return "undefined";
  return v.dump();
}

bool hasKey(const Json& o, const std::string& k) { return o.isObject() && o.has(k) && !o[k].isNull(); }
}  // namespace

Tuner::Tuner(io::Link& lk, const std::string& sessionDir) : link(lk), sessionDir_(sessionDir) {
  opts = Json::obj({{"preset", "normal"}, {"autoApply", false}, {"axes", Json(Json::Array{"roll", "pitch"})}, {"observeSeconds", 30},
                    {"stepMax", 1.5}, {"watchSeconds", 4}, {"oscSmax", 5}, {"massKg", 0}, {"allowRamp", false},
                    {"rampDoubleTime", 10}, {"rampMargin", 60}, {"maxReducePct", 20}, {"angleAbort", 10}, {"sessionDir", sessionDir}});
  for (const auto& a : kAxisNames) axes.emplace(a, LiveAxis(a));
  attitude = Json::obj({{"roll", 0}, {"pitch", 0}, {"yaw", 0}, {"p", 0}, {"q", 0}, {"r", 0}, {"ts", 0}});
  rc = Json::obj({{"roll", 0}, {"pitch", 0}, {"yaw", 0}, {"thr", 0}, {"ts", 0}});
  hud = Json::obj({{"airspeed", 0}, {"throttle", 0}, {"alt", 0}});
  stateSince = Loop::now();
  // ---- link bindings
  link.on("PID_TUNING", [this](const Json& f) {
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
    attitude = Json::obj({{"roll", d("roll")}, {"pitch", d("pitch")}, {"yaw", d("yaw")}, {"p", d("rollspeed")}, {"q", d("pitchspeed")},
                          {"r", d("yawspeed")}, {"ts", Loop::now() / 1000}});
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
    if (state != "idle" && !v["armed"].asBool() && armedSeen_) onDisarm();
    if (v["armed"].asBool()) armedSeen_ = true;
  });
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
    family.reset();
  }
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
  family = fam;
  if (!fam) throw rt::Error("unsupported vehicle type " + v["typeName"].strOr("?"));
  const std::vector<std::string> names = core::neededParams(*fam == Family::Heli ? Family::Copter : *fam);
  const Json got = co_await link.requestParams(names, 3, [this](int d, int n) { emit("progress", Json::obj({{"done", d}, {"total", n}})); });
  for (const auto& [k, val] : got.items()) params.set(k, val);
  loopHz = jor(params, "SCHED_LOOP_RATE", *fam == Family::Plane ? 50 : 400);
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
  for (const auto& w : writes.arr()) {
    const std::string why = w["why"].str();
    co_await setParam(w["name"].str(), w["to"].num(), why.substr(0, why.find('.')));
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
  for (const auto& ax : kAxisNames) {
    const Json n = engine::axisParamNames(fam, axisOf(ax));
    const std::string smax = n["SMAX"].strOr("");
    if (!smax.empty() && !(jv(params, smax.c_str()) > 0))
      co_await setParam(smax, core::familyDef(fam).defaultSmax, "slew-rate limit so the oscillation signal is reported");
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
  link.requestDataStream(2, 5);   // VFR_HUD
  Loop::main().setTimeout(1500, [this] {
    link.setMessageRate(kPidTuningId, family == Family::Plane ? 25 : 50);
    link.setMessageRate(kAttitudeId, 25);
    link.setMessageRate(241, 2);   // VIBRATION, for the final check
  });
}

rt::Task<double> Tuner::setParam(const std::string& name, double value, const std::string& reason) {
  const Json from = params[name];
  const double v = co_await link.setParam(name, value);
  params.set(name, v);
  journal.push(Json::obj({{"t", Loop::now()}, {"name", name}, {"from", from}, {"to", v}, {"reason", reason}}));
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

rt::Task<void> Tuner::writeGains(const std::string& axis, const Json& gains, const std::string& reason) {
  const Json n = engine::axisParamNames(*family, axisOf(axis));
  for (const char* k : {"FLTT", "FLTE", "FLTD", "SMAX", "D", "P", "I", "FF", "angleP", "tconst"}) {
    if (!gains[k].isNumber() || !n[k].isString()) continue;
    const std::string pname = n[k].str();
    const Json& cur = params[pname];
    if (cur.isNumber() && std::fabs(cur.num() - gains[k].num()) < 1e-9) continue;
    co_await setParam(pname, gains[k].num(), reason);
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

void Tuner::start() {
  if (state == "idle" || state == "ready" || state == "done" || state == "paused") {
    if (!family || !prepared) throw rt::Error("read the vehicle first (step 2)");
    requestStreams();
    axisIdx = 0;
    round = 0;
    for (const auto& a : kAxisNames) axes[a].setBaseline();
    setState("observing", currentAxis() + ": fly normally and tap the sticks (round 1)");
    tick();
  }
}

void Tuner::pause(const std::string& reason) {
  setState("paused", reason.empty() ? "paused by user" : reason);
  ramp_.reset();
}

std::string Tuner::currentAxis() const {
  const auto& a = opts["axes"];
  if (!a.isArray() || a.arr().empty()) return "roll";
  return a[static_cast<std::size_t>(axisIdx) % a.size()].strOr("roll");
}

rt::Task<void> Tuner::stop() {
  ramp_.reset();
  setState("idle", "stopped");
  co_return;
}

rt::Task<void> Tuner::restoreOriginals() {
  pause("restoring original gains");
  for (const auto& ax : kAxisNames)
    if (hasKey(originals, ax)) co_await writeGains(ax, originals[ax], "restore original");
  lastGood = originals;
  log("tuner", "original gains restored");
}

rt::Task<void> Tuner::revertAxis(const std::string& axis, const std::string& reason) {
  if (!hasKey(lastGood, axis)) co_return;
  co_await writeGains(axis, lastGood[axis], "revert: " + reason);
}

rt::Task<void> Tuner::undoAxis(const std::string& axis) {
  if (!hasKey(previous, axis)) throw rt::Error("nothing to undo for " + axis);
  const Json g = previous[axis];
  if (state == "observing" || state == "applying") pause("undo requested");
  co_await writeGains(axis, g, "undo last apply");
  lastGood.set(axis, g);
  saveSession();
}

void Tuner::onDisarm() {
  if (state != "idle") pause("vehicle disarmed");
}

rt::Task<bool> Tuner::applyProposal(const std::string& axis) {
  const Json& r = results[axis];
  if (!r.isObject() || !r["proposal"].isObject()) throw rt::Error("no proposal for " + axis);
  const Json target = r["proposal"]["gains"];
  setState("applying", axis);
  co_return co_await applyStaged(axis, target);
}

rt::Task<bool> Tuner::applyStaged(const std::string& axis, Json target) {
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
    if (a > 0 && b > 0) steps = std::max(steps, static_cast<int>(std::ceil(std::fabs(std::log(b / a)) / std::log(stepMax))));
  }
  steps = std::min(steps, 6);
  // filters / smax first (small effect), then the gains stepwise
  Json pre = Json::object();
  for (const char* k : {"FLTT", "FLTE", "FLTD", "SMAX"})
    if (target[k].isNumber()) pre.set(k, target[k]);
  co_await writeGains(axis, pre, "apply filters");
  const double watchS = opts["watchSeconds"].numOr(4);
  for (int s = 1; s <= steps; s++) {
    if (state == "paused" || state == "idle") co_return false;
    const Json before = gainsOf(axis);
    Json g = Json::object();
    for (const auto& k : keys) {
      const double a = cur[k].num(), b = target[k].num();
      g.set(k, (a > 0 && b > 0) ? a * std::pow(b / a, static_cast<double>(s) / steps) : (s == steps ? b : a + (b - a) * s / steps));
    }
    co_await writeGains(axis, g, "apply step " + std::to_string(s) + "/" + std::to_string(steps));
    const bool ok = co_await watch(axis, watchS);
    if (!ok) {
      // a step that only lowered gains did not cause the oscillation: going back up would make it worse
      if (std::none_of(keys.begin(), keys.end(), [&](const std::string& k) { return above(g[k], before[k], 1 + 1e-6); })) {
        Json low = Json::object();
        std::string list;
        for (const auto& k : keys) {
          low.set(k, target[k]);
          list += (list.empty() ? "" : ", ") + k + " " + fmt4(target[k]);
        }
        co_await writeGains(axis, low, "oscillation continued after lowering the gains (step " + std::to_string(s) + "): lowered to the target");
        lastGood.set(axis, gainsOf(axis));
        saveSession();
        pause(axis + ": still oscillating after lowering the gains, so the lower gains were kept (" + list +
              "). Land if it keeps shaking, otherwise press Resume");
        co_return false;
      }
      co_await revertAxis(axis, "oscillation detected after step " + std::to_string(s));
      pause(axis + ": oscillation after applying step " + std::to_string(s) + "/" + std::to_string(steps) + "; reverted to last good gains");
      co_return false;
    }
    lastGood.set(axis, gainsOf(axis));
    saveSession();
  }
  if (target["angleP"].isNumber() || target["tconst"].isNumber()) {
    const Json before = gainsOf(axis);
    Json ag = Json::object();
    if (target["angleP"].isNumber()) ag.set("angleP", target["angleP"]);
    if (target["tconst"].isNumber()) ag.set("tconst", target["tconst"]);
    co_await writeGains(axis, ag, "apply angle gain");
    const bool ok = co_await watch(axis, watchS);
    if (!ok) {
      // a softer angle loop (lower angle P, longer time constant) did not cause it either: keep it
      if (!above(target["angleP"], before["angleP"], 1 + 1e-6) && !below(target["tconst"], before["tconst"], 1 - 1e-6)) {
        lastGood.set(axis, gainsOf(axis));
        saveSession();
        pause(axis + ": still oscillating after softening the angle gain, so it was kept. Land if it keeps shaking, otherwise press Resume");
        co_return false;
      }
      co_await revertAxis(axis, "oscillation after angle gain");
      pause(axis + ": oscillation after the angle gain; reverted");
      co_return false;
    }
    lastGood.set(axis, gainsOf(axis));
    saveSession();
  }
  log("tuner", axis + ": new gains applied and held for " + jsToString(watchS) + " s without oscillation");
  co_return true;
}

rt::Future<bool> Tuner::watch(const std::string& axis, double seconds) {
  rt::Promise<bool> p;
  const double t0 = Loop::now();
  const double since = t0 / 1000 + kWatchGraceS;   // samples before this still describe the old gains
  auto iv = std::make_shared<std::uint64_t>(0);
  *iv = Loop::main().setInterval(250, [this, p, iv, axis, seconds, t0, since] {
    const Watchdog w = axes[axis].watchdog(2, opts["oscSmax"].numOr(5), 8, 0.9, since);
    const double attErr = 0;   // only meaningful with centred sticks (not used, as in the JavaScript version)
    if (w.oscillating || attErr > opts["angleAbort"].numOr(10)) {
      Loop::main().clear(*iv);
      std::string rs;
      for (std::size_t i = 0; i < w.reasons.size(); i++) rs += (i ? "; " : "") + w.reasons[i];
      log("watchdog", axis + ": " + (rs.empty() ? "attitude error " + jsToFixed(attErr, 1) + " deg" : rs));
      p.resolve(false);
      return;
    }
    if (Loop::now() - t0 > (seconds + kWatchGraceS) * 1000) {
      Loop::main().clear(*iv);
      p.resolve(true);
    }
  });
  return p.future();
}

void Tuner::tick() {
  if (timer_) Loop::main().clear(timer_);
  timer_ = Loop::main().setTimeout(500, [this] { tickLoop(); });
}

void Tuner::tickLoop() {
  timer_ = 0;
  rt::Future<void> f = tickStep();
  f.then([this] {
    if (state == "observing" || state == "ramping")
      timer_ = Loop::main().setTimeout(state == "ramping" ? 250 : 1000, [this] { tickLoop(); });
  });
}

rt::Task<void> Tuner::tickStep() {
  try {
    if (state == "observing")
      co_await observeStep();
    else if (state == "ramping")
      co_await rampStep();
  } catch (const std::exception& e) {
    log("error", e.what());
    pause(std::string("error: ") + e.what());
  }
}

rt::Task<void> Tuner::observeStep() {
  const std::string axis = currentAxis();
  LiveAxis& la = axes[axis];
  const double elapsed = (Loop::now() - stateSince) / 1000;
  const double exc = la.excitation(20);
  // watchdog during observation too (the vehicle may be oscillating with its current gains)
  const Watchdog w = la.watchdog(2, opts["oscSmax"].numOr(5));
  if (w.oscillating && w.bandRms > 15) {
    std::string rs;
    for (std::size_t i = 0; i < w.reasons.size(); i++) rs += (i ? "; " : "") + w.reasons[i];
    log("watchdog", axis + ": " + rs);
    if (journal.size()) co_await revertAxis(axis, "oscillation while observing");
    pause(axis + " is oscillating (" + w.reasons[0] + "); reduce gains before continuing");
    co_return;
  }
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
    round++;
    if (opts["allowRamp"].asBool() && round >= 2) {
      startRamp(axis);
      co_return;
    }
    std::string why = res["identification"]["reason"].strOr("");
    if (why.empty() && res["warnings"].isArray() && res["warnings"].size()) why = res["warnings"][std::size_t{0}].strOr("");
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
    std::string list;
    for (const auto& c : res["proposal"]["changes"].arr())
      list += (list.empty() ? "" : ", ") + c["key"].strOr("") + " " + fmt4(c["from"]) + "->" + fmt4(c["to"]);
    setState("applying", axis + ": " + list);
    const bool ok = co_await applyStaged(axis, res["proposal"]["gains"]);
    if (ok) {
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
    r.filtersSet = true;
    co_await writeGains(ax, fl, "QuikTune filter rule");
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
      co_await setParam(names["P"].str(), params[names["P"].str()].numOr(0) * ratio, "QuikTune: P lowered with D");
    }
    co_await setParam(pname, nv,
                      std::string("QuikTune: ") + (w.oscillating ? "oscillation at " + fmt4(Json(cur)) : "limit reached") + ", back off");
    if (key == "P") co_await coupleI(axis, nv);
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
  co_await setParam(pname, nv, "QuikTune ramp " + key);
  if (key == "P") co_await coupleI(axis, nv);
}

rt::Task<void> Tuner::coupleI(const std::string& axis, double P) {
  const Json n = engine::axisParamNames(*family, axisOf(axis));
  const double ff = jor(params, n["FF"].str().c_str(), 0);
  if (ff > 0) co_return;   // with FF the I gain is tied to FF, not P
  const double I = core::iRule(*family == Family::Heli ? Family::Copter : *family, axisOf(axis), P, ff, false);
  co_await setParam(n["I"].str(), I, "I follows P");
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
                    {"journal", j},
                    {"messages", m}});
}

}  // namespace aptuner::app
