// Model tuning from live telemetry, read-only (see heavylive.hpp), and the saved plans per vehicle.
#include "heavylive.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <thread>

#include "aptuner/core/rules.hpp"
#include "aptuner/engine/heavytune.hpp"
#include "aptuner/engine/vehicle.hpp"
#include "aptuner/rt/loop.hpp"
#include "common.hpp"

namespace aptuner::app {

using core::Json;
using rt::Loop;
namespace fs = std::filesystem;
namespace heavy = engine::heavy;

namespace {

constexpr int kPidTuning = 194, kAttitude = 30, kServoOut = 36, kRcChannels = 65, kNavOut = 62, kVfrHud = 74, kVibration = 241;

std::string stamp() {
  const std::time_t now = std::time(nullptr);
  char b[32];
  std::strftime(b, sizeof b, "%Y-%m-%d %H-%M-%S", std::localtime(&now));
  return b;
}

std::string planFolder(const std::string& sessionsDir, const Json& sysid) {
  return pathStr(fsPath(sessionsDir) / "plans" / ("sysid-" + (sysid.isNumber() ? std::to_string(static_cast<int>(sysid.num())) : std::string("unknown"))));
}

}  // namespace

std::vector<std::string> heavyParamNames(bool q, int yawChannel) {
  std::vector<std::string> n;
  const std::string A = q ? "Q_A_" : "ATC_";
  for (const char* ax : {"RLL", "PIT", "YAW"}) {
    for (const char* k : {"P", "I", "D", "FF", "IMAX", "FLTT", "FLTE", "FLTD"}) n.push_back(A + "RAT_" + ax + "_" + k);
    n.push_back(A + "ANG_" + ax + "_P");
  }
  for (const char* ax : {"R", "P", "Y"}) {
    n.push_back(A + "ACC_" + ax + "_MAX");
    n.push_back(A + "RATE_" + ax + "_MAX");
  }
  n.push_back(A + "INPUT_TC");
  n.push_back(A + "RATE_WPY_MAX");
  const std::string M = q ? "Q_M_" : "MOT_";
  for (const char* k : {"PWM_MIN", "PWM_MAX", "SPIN_MIN", "SPIN_MAX", "THST_EXPO", "THST_HOVER", "HOVER_LEARN"}) n.push_back(M + k);
  if (q) {
    for (const char* k : {"Q_ENABLE", "Q_FRAME_CLASS", "Q_FRAME_TYPE", "Q_PLT_Y_RATE", "Q_PLT_Y_EXPO", "Q_PLT_Y_RATE_TC"}) n.push_back(k);
  } else {
    for (const char* k : {"FRAME_CLASS", "FRAME_TYPE", "PILOT_Y_RATE", "PILOT_Y_EXPO", "PILOT_Y_RATE_TC"}) n.push_back(k);
  }
  for (const char* k : {"INS_GYRO_FILTER", "SCHED_LOOP_RATE", "SYSID_THISMAV", "GCS_PID_MASK", "RCMAP_YAW"}) n.push_back(k);
  for (int i = 1; i <= 16; ++i) n.push_back("SERVO" + std::to_string(i) + "_FUNCTION");
  for (const char* k : {"MIN", "TRIM", "MAX", "DZ"}) n.push_back("RC" + std::to_string(yawChannel) + "_" + k);
  return n;
}

// ------------------------------------------------------------------------------------------------- plans

Json loadPlan(const std::string& sessionsDir, const Json& sysid, const std::vector<std::string>& sources) {
  const fs::path dir = fsPath(planFolder(sessionsDir, sysid));
  std::error_code ec;
  if (!fs::is_directory(dir, ec)) return Json();
  std::vector<fs::path> files;
  for (const auto& e : fs::directory_iterator(dir, ec))
    if (e.path().extension() == ".json") files.push_back(e.path());
  std::sort(files.rbegin(), files.rend());   // newest first (time-stamped names)
  for (const auto& f : files) {
    try {
      const Json p = Json::parse(readText(pathStr(f)));
      bool overlap = false;
      for (const auto& s : p["sources"].isArray() ? p["sources"].arr() : Json::Array{})
        if (std::find(sources.begin(), sources.end(), s.strOr("")) != sources.end()) overlap = true;
      if (!overlap) return p;
    } catch (const std::exception&) {
    }
  }
  return Json();
}

std::string savePlan(const std::string& sessionsDir, const Json& plan) {
  const std::string dir = planFolder(sessionsDir, plan["vehicle"]["sysid"]);
  std::error_code ec;
  fs::create_directories(fsPath(dir), ec);
  const std::string file = pathStr(fsPath(dir) / (stamp() + ".json"));
  writeText(file, plan.dump());
  return file;
}

Json analyseAndSave(const std::vector<heavy::FlightRecord>& flights, const std::string& sessionsDir, double motorSpacingM) {
  heavy::Options o;
  o.motorSpacingM = motorSpacingM;
  std::vector<std::string> names;
  for (const auto& f : flights) names.push_back(f.name);
  if (!flights.empty()) o.plan = loadPlan(sessionsDir, flights.back().identity["sysid"], names);
  Json r = heavy::analyzeFlights(flights, o);
  if (o.plan.isObject()) r.set("planUsed", Json::obj({{"created", o.plan["created"]}, {"sources", o.plan["sources"]}}));
  if (r["plan"].isObject() && !r.has("error")) r.set("planFile", savePlan(sessionsDir, r["plan"]));
  return r;
}

// ------------------------------------------------------------------------------------------------- live

HeavyLive::HeavyLive(io::Link& link, std::string sessionsDir) : link_(link), sessionsDir_(std::move(sessionsDir)) {
  // always subscribed: the live readout (message rates, attitude) works before recording; recording needs start()
  for (const char* m : {"PID_TUNING", "ATTITUDE", "NAV_CONTROLLER_OUTPUT", "RC_CHANNELS", "SERVO_OUTPUT_RAW", "VFR_HUD", "HEARTBEAT", "STATUSTEXT", "VIBRATION"}) {
    const std::string name = m;
    handlers_.push_back(link_.on(name, [this, name](const Json& f) { onMsg(name, f); }));
  }
  handlers_.push_back(link_.on("reboot", [this](const Json&) {
    if (listening_) requestStreams();
  }));
}

void HeavyLive::setState(const std::string& s, const std::string& info) {
  state_ = s;
  info_ = info;
  emit("state", state());
}

Json HeavyLive::state() const {
  Json pid = Json::object();
  for (int a = 0; a < 3; ++a) pid.set(core::axisName(static_cast<core::Axis>(a)), std::round(rec_.pidSeconds(a)));
  Json rates = Json::object();
  const double now = Loop::now() / 1000;
  for (const auto& [k, q] : seen_) {
    std::size_t n = 0;
    for (double x : q)
      if (x >= now - 3) ++n;
    rates.set(k, std::round(static_cast<double>(n) / 3.0 * 10) / 10);
  }
  return Json::obj({{"state", state_}, {"info", info_}, {"listening", listening_}, {"armed", rec_.armed()}, {"flights", flights_},
                    {"recordedS", rec_.startS() >= 0 ? std::round(rec_.lastS() - rec_.startS()) : 0.0}, {"pidSeconds", pid}, {"warnings", warnings_}, {"rates", rates}, {"attitude", att_}, {"mode", customMode_}, {"link", link_.description()}});
}

void HeavyLive::requestStreams() {
  // the streams the analysis needs (a request to the autopilot, not a parameter write; Mission Planner does the same)
  link_.requestDataStream(10, 20);   // EXTRA1: ATTITUDE, PID_TUNING
  link_.requestDataStream(3, 10);    // RC_CHANNELS, SERVO_OUTPUT_RAW
  link_.requestDataStream(2, 5);     // EXTENDED_STATUS: NAV_CONTROLLER_OUTPUT
  link_.requestDataStream(11, 5);    // EXTRA2: VFR_HUD
  Loop::main().setTimeout(1500, [this] {
    link_.setMessageRate(kPidTuning, 25);
    link_.setMessageRate(kAttitude, 25);
    link_.setMessageRate(kServoOut, 10);
    link_.setMessageRate(kRcChannels, 10);
    link_.setMessageRate(kNavOut, 10);
    link_.setMessageRate(kVfrHud, 5);
    link_.setMessageRate(kVibration, 2);
  });
}

rt::Task<Json> HeavyLive::start() {
  if (!link_.alive()) throw rt::Error("no vehicle heartbeat: connect first (the app finds Mission Planner by itself: press Connect)");
  setState("reading", "reading the vehicle's settings");
  warnings_ = Json::array();
  const Json v = link_.vehicle();
  const int type = static_cast<int>(v["type"].numOr(0));
  bool q = false;
  try {
    q = (co_await link_.requestParam("Q_ENABLE")) > 0;
  } catch (const std::exception&) {
    q = false;
  }
  double yawCh = 4;
  try {
    yawCh = co_await link_.requestParam("RCMAP_YAW");
  } catch (const std::exception&) {
  }
  const auto names = heavyParamNames(q, static_cast<int>(yawCh));
  const Json got = co_await link_.requestParams(names, 6, [this](int d, int n) { emit("progress", Json::obj({{"done", d}, {"total", n}})); });
  params_ = Json::object();
  for (const auto& [k, val] : got.items())
    if (val.isNumber()) params_.set(k, val);
  // older firmware: acceleration limits under their old names (centi-degrees)
  if (!params_.has(std::string(q ? "Q_A_" : "ATC_") + "ACC_R_MAX")) {
    const std::string A = q ? "Q_A_" : "ATC_";
    const std::vector<std::string> oldNames{A + "ACCEL_R_MAX", A + "ACCEL_P_MAX", A + "ACCEL_Y_MAX"};
    const Json old = co_await link_.requestParams(oldNames, 3);
    for (const auto& [k, val] : old.items())
      if (val.isNumber()) params_.set(k, val);
  }
  log::Params lp;
  for (const auto& [k, val] : params_.items()) lp.set(k, val.num());
  const auto fam = q ? std::optional<core::Family>(core::Family::Vtol) : engine::detectFamily(type, v["modeName"].strOr(""), lp);
  if (!fam || (*fam != core::Family::Vtol && *fam != core::Family::Copter)) {
    setState("idle", "model tuning is for multirotors and QuadPlane VTOL flight");
    throw rt::Error("model tuning is for multirotors and QuadPlane VTOL flight");
  }
  rec_ = heavy::TelemetryRecorder();
  rec_.setFamily(*fam);
  rec_.setParams(params_);
  if (static_cast<int>(params_["GCS_PID_MASK"].numOr(0)) != 7)
    warnings_.push("GCS_PID_MASK is " + std::to_string(static_cast<int>(params_["GCS_PID_MASK"].numOr(0))) +
                   ": the autopilot does not send the PID telemetry (rate target, actual, P / I / D). In Mission Planner set GCS_PID_MASK = 7 "
                   "(Full Parameter List, Write Params). The app does not change it for you. Without it only the heading and motors can be judged.");
  listening_ = true;
  requestStreams();
  setState("waiting", "listening: arm and fly a hover; the recording starts when the vehicle arms and is analysed after it lands");
  co_return state();
}

void HeavyLive::stop() {
  listening_ = false;
  setState("idle", "stopped");
}

void HeavyLive::onMsg(const std::string& type, const Json& f) {
  const double t = Loop::now() / 1000;
  {
    auto& q = seen_[type];
    q.push_back(t);
    while (!q.empty() && q.front() < t - 3) q.pop_front();
  }
  if (type == "ATTITUDE")
    att_ = Json::obj({{"roll", std::round(f["roll"].numOr(0) * 573) / 10}, {"pitch", std::round(f["pitch"].numOr(0) * 573) / 10},
                      {"yaw", std::round(std::fmod(f["yaw"].numOr(0) * 57.3 + 360, 360) * 10) / 10}});
  if (type == "HEARTBEAT" && f["type"].numOr(0) != 6) customMode_ = f["custom_mode"].numOr(-1);
  if (!listening_) return;
  const bool before = rec_.armed();
  if (type == "HEARTBEAT") {
    // the autopilot's heartbeat only (a ground station's is type 6)
    if (f["type"].numOr(0) == 6 || f["autopilot"].numOr(3) == 8) return;
    rec_.onMessage(type, f, t);
    const bool now = rec_.armed();
    if (now && !before && state_ != "analysing") {
      // a new flight: start a clean recording with the settings as they are now
      rec_.clear();
      rec_.setParams(params_);
      rec_.onMessage(type, f, t);
      armedAt_ = t;
      setState("recording", "recording the flight");
    } else if (!now && before && state_ == "recording") {
      finishFlight("landed and disarmed");
    }
    return;
  }
  if (state_ == "recording") rec_.onMessage(type, f, t);
}

void HeavyLive::analyseNow() {
  if (busy_) return;
  finishFlight("analysis asked for");
}

void HeavyLive::finishFlight(const std::string& why) {
  if (busy_) return;
  if (rec_.lastS() - rec_.startS() < 15) {
    setState("waiting", "the flight was too short to analyse (" + why + ")");
    return;
  }
  busy_ = true;
  ++flights_;
  setState("analysing", why + ": building the model (this takes up to a minute)");
  const std::string name = "telemetry " + stamp();
  auto rec = std::make_shared<heavy::FlightRecord>(rec_.finish(name));
  const std::string dir = sessionsDir_;
  std::thread([this, rec, dir] {
    Json r;
    try {
      r = analyseAndSave({*rec}, dir, 0);
    } catch (const std::exception& e) {
      r = Json::obj({{"error", std::string("analysis failed: ") + e.what()}});
    }
    Loop::main().post([this, r] {
      busy_ = false;
      setState("done", r.has("error") ? r["error"].strOr("") : "analysed: see the result below");
      if (onResult) onResult(r);
    });
  }).detach();
}

}  // namespace aptuner::app
