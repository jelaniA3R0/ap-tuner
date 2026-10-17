#include "groundtune.hpp"

#include <algorithm>
#include <cmath>
#include <regex>

#include "aptuner/core/jsnum.hpp"
#include "aptuner/engine/issues.hpp"
#include "aptuner/engine/loganalysis.hpp"
#include "aptuner/engine/report.hpp"
#include "aptuner/io/sysinfo.hpp"
#include "common.hpp"
#include "flight.hpp"

namespace aptuner::app {

using core::jsToFixed;
using rt::Loop;

namespace {
const std::set<std::string> kRunning{"flight", "capture", "captured", "usb", "download"};

std::string join(const Json& a, const char* sep) {
  std::string s;
  for (const auto& x : a.arr()) s += (s.empty() ? "" : sep) + x.strOr("");
  return s;
}
std::string mb(double bytes) { return jsToFixed(bytes / 1048576, 1); }
}  // namespace

GroundTune::GroundTune(io::Link& link, const std::string& sessionsDir)
    : link_(link), dl_(link), dir_(pathStr(fsPath(sessionsDir) / "ground")) {
  link_.on("message", [this](const Json& m) {
    const Json& f = m["fields"];
    if (f.isObject() && f.has("time_boot_ms") && m["compid"].numOr(0) == 1 && f["time_boot_ms"].numOr(0) > 0) {
      bootMs_ = f["time_boot_ms"].num();
      bootAt_ = Loop::now();
    }
  });
  reset();
}

void GroundTune::reset() {
  stopTimer();
  phase_ = "idle";
  info_.clear();
  since_ = Loop::now();
  opts = Json::obj({{"axes", Json(Json::Array{"roll", "pitch"})}, {"preset", "normal"}, {"seconds", 30}, {"usb", Json()}});
  mark_ = captureStart_ = usb_ = progress_ = result = error_ = Json();
  tFlying_ = 0;
  rows_ = Json::array();
  writes_ = Json::array();
  files = Json::object();
  notes_ = Json::array();
  sysid_.reset();
  knownPorts_.clear();
  busy_.clear();
  tried_.clear();
  restoreMark();
}

// the marked window survives an app restart (the pilot may close the app before plugging in USB)
std::string GroundTune::markFile() const { return pathStr(fsPath(dir_) / "last-mark.json"); }
void GroundTune::saveMark() { writeText(markFile(), mark_.dump()); }
void GroundTune::restoreMark() {
  savedMark_ = Json();
  try {
    const std::string t = readText(markFile());
    if (t.empty()) return;
    const Json m = Json::parse(t);
    if (m.isObject() && Loop::now() - m["savedAt"].numOr(0) < 6 * 3600e3) savedMark_ = m;
  } catch (const std::exception&) {
  }
}

std::optional<double> GroundTune::nowBootMs() const {
  if (bootMs_ < 0 || Loop::now() - bootAt_ > 5000) return std::nullopt;
  return bootMs_ + (Loop::now() - bootAt_);
}

std::string GroundTune::familyOf(const Json& v) const {
  if (!v.isObject()) return "";
  const std::string m = v["modeName"].strOr("");
  if (!m.empty() && m[0] == 'Q') return "vtol";
  const auto& pm = goodModes("plane");
  return std::find(pm.begin(), pm.end(), m) != pm.end() || v["type"].numOr(0) == 1 ? "plane" : "copter";
}

bool GroundTune::modeOk(const Json& v) const {
  const std::string fam = familyOf(v);
  if (fam.empty()) return false;
  const auto& gm = goodModes(fam);
  return std::find(gm.begin(), gm.end(), v["modeName"].strOr("")) != gm.end();
}

void GroundTune::setPhase(const std::string& p, const std::string& info) {
  phase_ = p;
  since_ = Loop::now();
  info_ = info;
  emit("phase", Json::obj({{"phase", p}, {"info", info}}));
}

void GroundTune::note(const std::string& level, const std::string& text) {
  notes_.push(Json::obj({{"t", Loop::now()}, {"level", level}, {"text", text}}));
}

void GroundTune::fail(const std::string& msg) {
  error_ = msg;
  note("bad", msg);
  setPhase("error", msg);
}

void GroundTune::start(const Json& o) {
  if (kRunning.count(phase_)) throw rt::Error("a session is already running");
  reset();
  for (const auto& [k, v] : o.items())
    if (!v.isNull()) opts.set(k, v);
  if (link_.alive()) {
    sysid_ = link_.targetSys();
    // a timestamp source for marking: most mirrors carry ATTITUDE; ask for SYSTEM_TIME at 1 Hz as well
    link_.setMessageRate(2, 1);
    setPhase("flight", "take off and hover; the 30 s capture starts by itself");
  } else {
    setPhase("flight", "no telemetry: connect first, or press \"No telemetry: go to USB\" after the flight");
  }
  tick();
}

void GroundTune::stop() {
  stopTimer();
  dl_.end();
  setPhase("stopped", "stopped by you");
}

void GroundTune::captureAgain() {
  if (phase_ != "captured" && phase_ != "flight") throw rt::Error("not in flight");
  mark_ = Json();
  tFlying_ = 0;
  setPhase("flight", "hover again; the capture starts by itself");
}

rt::Task<void> GroundTune::goUsb(bool useSaved) {
  if (phase_ == "capture") throw rt::Error("wait for the capture to finish");
  if (link_.vehicle()["armed"].asBool() && link_.alive()) throw rt::Error("land and disarm first");
  if (useSaved && savedMark_.isObject() && !mark_.isObject()) {
    mark_ = savedMark_;
    note("ok", "using the 30 s marked at " + localTime(mark_["savedAt"].numOr(0)));
  }
  co_await enterUsb();
}

void GroundTune::stopTimer() {
  if (timer_) Loop::main().clear(timer_);
  timer_ = 0;
}

void GroundTune::tick() {
  stopTimer();
  timer_ = Loop::main().setTimeout(200, [this] { loopOnce(); });
}

void GroundTune::loopOnce() {
  timer_ = 0;
  rt::Future<void> f = step();
  f.then([this, f] {
    if (f.error()) {
      try {
        std::rethrow_exception(f.error());
      } catch (const std::exception& e) {
        fail(e.what());
      }
    }
    if (kRunning.count(phase_) && !timer_) timer_ = Loop::main().setTimeout(phase_ == "usb" ? 1500 : 500, [this] { loopOnce(); });
  });
}

rt::Task<void> GroundTune::step() {
  const Json v = link_.vehicle();
  const bool armed = v.isObject() && v["armed"].asBool() && link_.alive();
  const double el = (Loop::now() - since_) / 1000;
  const std::string modeName = v["modeName"].strOr("");
  const std::string modesText = "QStabilize / QHover / QLoiter (QuadPlane), Stabilize / AltHold / Loiter (copter) or FBWA (plane)";
  if (phase_ == "flight") {
    if (!link_.alive()) {
      info_ = "waiting for telemetry (Mission Planner mirror or radio)";
      co_return;
    }
    if (!armed) {
      tFlying_ = 0;
      info_ = "arm, take off and hover in " + modesText;
      co_return;
    }
    if (!modeOk(v)) {
      tFlying_ = 0;
      info_ = "flying in " + modeName + ": switch to " + modesText;
      co_return;
    }
    if (!nowBootMs()) {
      info_ = "no time stamps over the telemetry yet (waiting for ATTITUDE / SYSTEM_TIME)";
      link_.setMessageRate(2, 1);
      co_return;
    }
    if (tFlying_ == 0) tFlying_ = Loop::now();
    if (Loop::now() - tFlying_ < 4000) {
      info_ = "hovering in " + modeName + ": the capture starts in a moment";
      co_return;
    }
    captureStart_ = Json::obj({{"ms", *nowBootMs()}, {"mode", modeName}, {"family", familyOf(v)}});
    setPhase("capture", "capturing: short, sharp stick taps on " + join(opts["axes"], ", ") + ", hover in between");
    co_return;
  }
  if (phase_ == "capture") {
    const double w = opts["seconds"].numOr(30);
    if (!armed) {
      setPhase("flight", "the vehicle disarmed during the capture: take off again");
      co_return;
    }
    if (!modeOk(v)) {
      setPhase("flight", "left " + captureStart_["mode"].strOr("") + " during the capture: switch back, it starts again");
      tFlying_ = 0;
      co_return;
    }
    if (el < w) {
      info_ = "capturing: " + core::jsToString(std::ceil(w - el)) + " s left, keep tapping " + join(opts["axes"], ", ");
      co_return;
    }
    const double t1 = nowBootMs() ? *nowBootMs() : captureStart_["ms"].num() + w * 1000;
    mark_ = Json::obj({{"tStart", captureStart_["ms"].num() / 1000}, {"tEnd", t1 / 1000}, {"mode", captureStart_["mode"]},
                       {"family", captureStart_["family"]}, {"sysid", sysid_ ? Json(*sysid_) : Json()}, {"axes", opts["axes"]},
                       {"savedAt", Loop::now()}});
    saveMark();
    note("ok", "captured " + core::jsToString(w) + " s in " + mark_["mode"].strOr("") + " (autopilot time " + jsToFixed(mark_["tStart"].num(), 0) +
                   "-" + jsToFixed(mark_["tEnd"].num(), 0) + " s)");
    setPhase("captured", "captured: land and disarm, then plug the USB cable into the autopilot");
    co_return;
  }
  if (phase_ == "captured") {
    if (!armed && link_.alive())
      co_await enterUsb();
    else if (!link_.alive())
      info_ = "captured; telemetry lost. After landing press \"Landed: go to USB\"";
    co_return;
  }
  if (phase_ == "usb") co_await scanUsb();
}

rt::Task<void> GroundTune::enterUsb() {
  knownPorts_.clear();
  for (const auto& p : io::serialMap()) knownPorts_.insert(p.port);
  usbSince_ = Loop::now();
  setPhase("usb", opts["usb"].isObject() ? "connecting to the test port" : "plug the USB cable into the autopilot (the battery may stay connected)");
  if (!timer_) tick();
  co_return;
}

rt::Task<void> GroundTune::scanUsb() {
  std::vector<Json> cands;
  if (opts["usb"].isObject()) {
    Json c = Json::obj({{"label", "test port"}});
    for (const auto& [k, v] : opts["usb"].items()) c.set(k, v);
    cands.push_back(c);
  } else {
    const auto ports = io::serialMap();
    static const std::regex usbser("USBSER", std::regex::icase), bth("Bth", std::regex::icase);
    std::vector<io::SerialMapEntry> list;
    for (const auto& p : ports)
      if (std::regex_search(p.dev, usbser)) list.push_back(p);
    for (const auto& p : ports)
      if (!knownPorts_.count(p.port) && !std::regex_search(p.dev, bth) &&
          std::none_of(list.begin(), list.end(), [&](const io::SerialMapEntry& e) { return e.port == p.port; }))
        list.push_back(p);
    for (const auto& p : list)
      if (!busy_.count(p.port) && !tried_.count(p.port))
        cands.push_back(Json::obj({{"kind", "serial"}, {"port", p.port}, {"baud", 115200}, {"label", p.port}}));
    if (cands.empty()) {
      // nothing new: after a while try the remaining ports once more (a port seen at the start may be the autopilot)
      if (Loop::now() - usbSince_ > 20000 && !tried_.empty()) tried_.clear();
      std::string b;
      for (const auto& x : busy_) b += (b.empty() ? "" : ", ") + x;
      info_ = "plug the USB cable into the autopilot" + (busy_.empty() ? std::string() : " (in use by another program: " + b + ")");
      co_return;
    }
  }
  for (const auto& c : cands) {
    info_ = "trying " + c["label"].strOr("") + "…";
    auto status = std::make_shared<std::string>();
    const std::uint64_t sub = link_.on("status", [status](const Json& s) { *status = s.strOr(""); });
    link_.connect(c);
    const double t0 = Loop::now();
    bool ok = false;
    while (Loop::now() - t0 < 4500) {
      co_await rt::sleep(200);
      if (link_.alive()) {
        ok = true;
        break;
      }
      if (status->find("cannot open") != std::string::npos) break;
    }
    link_.off(sub);
    if (ok) {
      usb_ = Json::obj({{"label", c["label"]}, {"kind", c["kind"]}});
      if (sysid_ && link_.targetSys() != *sysid_)
        note("warn", "the USB vehicle has system id " + std::to_string(link_.targetSys()) + ", the flown one had " + std::to_string(*sysid_));
      note("ok", "connected over USB (" + c["label"].strOr("") + ")");
      stopTimer();
      setPhase("download", "reading the log list");
      rt::Future<void> f = download();
      f.then([this, f] {
        if (!f.error()) return;
        try {
          std::rethrow_exception(f.error());
        } catch (const std::exception& e) {
          fail(e.what());
        }
      });
      co_return;
    }
    const std::string port = c["port"].isString() ? c["port"].str() : std::string();
    if (status->find("cannot open") != std::string::npos)
      busy_.insert(port);
    else if (!port.empty())
      tried_.insert(port);
    link_.disconnect();
    co_await rt::sleep(500);
  }
}

rt::Task<void> GroundTune::download() {
  if (link_.vehicle()["armed"].asBool()) throw rt::Error("the vehicle is armed: disarm first");
  std::vector<io::LogEntry> logs;
  for (const auto& l : co_await dl_.list())
    if (l.size > 0) logs.push_back(l);
  if (logs.empty()) throw rt::Error("the autopilot reports no logs (is an SD card in it, and LOG_BACKEND_TYPE on?)");
  const Json m = mark_;
  std::optional<io::FetchResult> got;
  std::optional<io::LogEntry> used;
  // newest first; a reboot after landing does not start a new log, but check the ones before
  std::vector<io::LogEntry> order(logs.rbegin(), logs.rend());
  if (order.size() > 4) order.resize(4);
  for (const auto& log : order) {
    info_ = "log " + std::to_string(log.id) + " (" + mb(log.size) + " MB)";
    std::string code, msg;
    try {
      std::optional<double> tStart;
      double tEnd = 0;
      if (m.isObject()) {
        tStart = m["tStart"].num();
        tEnd = m["tEnd"].num();
      }
      got = co_await dl_.fetchWindow(log, tStart, tEnd, [this](const Json& p) {
        progress_ = p;
        const double total = p["total"].numOr(0), done = p["done"].numOr(0);
        info_ = p["stage"].strOr("") + ": " + (total > 1000 ? mb(done) + " / " + mb(total) + " MB" : core::jsToString(done) + " / " + core::jsToString(total));
      });
      used = log;
    } catch (const rt::Error& e) {
      code = e.code();
      msg = e.what();
      if (code != "after" && code != "before") throw;
    }
    if (used) break;
    note("warn", "log " + std::to_string(log.id) + ": " + msg + ", trying the one before");
  }
  dl_.end();
  if (!got) throw rt::Error("none of the newest logs covers the marked 30 s");
  const std::string stamp = isoStampForFile().substr(0, 19);
  const std::string file =
      pathStr(fsPath(dir_) / (stamp + " log" + std::to_string(used->id) + (got->whole ? "" : " (marked window)") + ".bin"));
  writeText(file, std::string(got->buf.begin(), got->buf.end()));
  files.set("bin", file);
  note("ok", "downloaded " + mb(got->bytes) + " MB of log " + std::to_string(used->id) + " (" + mb(used->size) + " MB in total)");
  setPhase("download", "analysing");
  co_await rt::sleep(0);
  engine::AnalyzeLogOptions ao;
  ao.preset = opts["preset"].strOr("normal");
  if (m.isObject()) {
    if (m["family"].isString()) ao.family = core::parseFamily(m["family"].str());
    ao.tStart = m["tStart"].num();
    ao.tEnd = m["tEnd"].num();
  }
  Json r = engine::analyzeLogFile(file, ao);
  const auto fam = core::parseFamily(r["family"].str());
  for (const char* ax : {"roll", "pitch", "yaw"})
    if (r["axes"][ax].isObject() && !r["axes"][ax].has("error")) r.at("axes").at(ax).set("issues", engine::axisIssues(r["axes"][ax], *fam));
  const Json chosenList = m.isObject() ? m["axes"] : opts["axes"];
  std::set<std::string> chosen;
  for (const auto& a : chosenList.arr()) chosen.insert(a.str());
  Json axes = Json::object();
  for (const auto& [ax, a] : r["axes"].items())
    if (chosen.count(ax)) axes.set(ax, a);
  rows_ = engine::proposalRows(*fam, axes, r["noise"]);
  Json chosenJ = Json::array();
  for (const auto& a : chosen) chosenJ.push(a);
  r.set("chosenAxes", chosenJ);
  result = r;
  // current values on the vehicle (the log holds the values at the time of the flight)
  for (auto& row : rows_.arr()) {
    try {
      const double v = co_await link_.requestParam(row["name"].str(), 1200, 2);
      row.set("vehicle", v);
    } catch (const std::exception&) {
      row.erase("vehicle");
    }
  }
  setPhase("review", rows_.size() ? core::jsToString(static_cast<double>(rows_.size())) + " change(s) proposed: check them, then press Write to the autopilot"
                                  : "no change proposed: see the issues per axis");
}

rt::Task<Json> GroundTune::write(const std::vector<std::string>& names) {
  if (phase_ != "review" && phase_ != "done") throw rt::Error("nothing to write yet");
  if (!link_.alive()) throw rt::Error("the autopilot is not connected");
  if (link_.vehicle()["armed"].asBool()) throw rt::Error("the vehicle is armed: disarm first");
  std::vector<std::size_t> idx;
  for (std::size_t i = 0; i < rows_.size(); i++)
    if (std::find(names.begin(), names.end(), rows_[i]["name"].strOr("")) != names.end()) idx.push_back(i);
  if (idx.empty()) throw rt::Error("choose at least one change");
  setPhase("writing", "writing");
  Json before = Json::array();
  for (std::size_t i : idx) {
    Json& r = rows_.arr()[i];
    std::optional<double> cur;
    try {
      cur = co_await link_.requestParam(r["name"].str(), 1200, 2);
    } catch (const std::exception&) {
    }
    if (cur) {
      r.set("vehicle", *cur);
      before.push(Json::obj({{"group", r["group"]}, {"name", r["name"]}, {"value", core::jsRoundPrecision(*cur, 6)}}));
    } else {
      r.erase("vehicle");
    }
  }
  const std::string stamp = isoStampForFile().substr(0, 19);
  files.set("undo", pathStr(fsPath(dir_) / (stamp + " UNDO (values before AP Tuner).param")));
  writeText(files["undo"].str(), engine::paramFile(before, {"Values on the autopilot before AP Tuner wrote, " + localNow(), "Load this file to go back."}));
  Json results = Json::array();
  // notch: enable first; the other notch parameters only exist after a reboot when it was off
  std::stable_sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) {
    return (rows_[a]["name"].strOr("") == "INS_HNTCH_ENABLE" ? -1 : 0) < (rows_[b]["name"].strOr("") == "INS_HNTCH_ENABLE" ? -1 : 0);
  });
  for (std::size_t i : idx) {
    const Json r = rows_[i];
    const double value = r["value"].num();
    if (r["vehicle"].isNumber() && std::fabs(r["vehicle"].num() - value) <= 1e-6 * std::max(1.0, std::fabs(value))) {
      results.push(Json::obj({{"name", r["name"]}, {"from", r["vehicle"]}, {"to", value}, {"ok", true}, {"same", true}}));
      continue;
    }
    bool failed = false;
    double echo = 0;
    try {
      echo = co_await link_.setParam(r["name"].str(), value, 1500, 3);
    } catch (const std::exception&) {
      failed = true;
    }
    if (!failed) {
      const bool ok = std::fabs(echo - value) <= 1e-4 * std::max(1.0, std::fabs(value));
      results.push(Json::obj({{"name", r["name"]}, {"from", r["vehicle"]}, {"to", echo}, {"ok", ok}}));
    } else {
      const std::string n = r["name"].str();
      const bool reboot = n.rfind("INS_HNTCH_", 0) == 0 && n != "INS_HNTCH_ENABLE";
      results.push(Json::obj({{"name", r["name"]}, {"from", r["vehicle"]}, {"ok", false},
                              {"error", reboot ? "needs a reboot first (the notch was just switched on)" : "no answer from the autopilot"}}));
    }
  }
  writes_ = results;
  Json after = Json::array();
  for (std::size_t i : idx) {
    const Json& r = rows_[i];
    Json row = Json::obj({{"group", r["group"]}, {"name", r["name"]}, {"value", r["value"]}});
    if (r["vehicle"].isNumber()) row.set("from", r["vehicle"]);
    after.push(row);
  }
  files.set("param", pathStr(fsPath(dir_) / (stamp + " written by AP Tuner.param")));
  writeText(files["param"].str(), engine::paramFile(after, {"Written to the autopilot by AP Tuner (Telemetry + USB), " + localNow()}));
  std::size_t bad = 0;
  for (const auto& x : results.arr())
    if (!x["ok"].asBool()) bad++;
  const std::size_t n = results.size();
  setPhase("done", bad ? std::to_string(n - bad) + " of " + std::to_string(n) + " written; " + std::to_string(bad) + " need attention (see the list)"
                       : "all " + std::to_string(n) + " written and read back. Unplug the USB and fly to check");
  co_return results;
}

rt::Task<void> GroundTune::undo() {
  if (!writes_.size()) throw rt::Error("nothing written yet");
  if (link_.vehicle()["armed"].asBool()) throw rt::Error("the vehicle is armed: disarm first");
  for (auto& w : writes_.arr()) {
    if (!w["ok"].asBool() || w["same"].asBool() || !w["from"].isNumber()) continue;
    bool failed = false;
    try {
      co_await link_.setParam(w["name"].str(), w["from"].num(), 1500, 3);
    } catch (const std::exception&) {
      failed = true;
    }
    w.set(failed ? "undoError" : "undone", true);
  }
  setPhase("done", "previous values written back");
}

Json GroundTune::snapshot() const {
  const Json& v = link_.vehicle();
  auto slimAxis = [](const Json& a) -> Json {
    if (!a.isObject()) return Json();
    const Json& p = a["proposal"];
    return Json::obj({{"error", a["error"]},
                      {"issues", a["issues"].isArray() ? a["issues"] : Json::array()},
                      {"grade", a["diagnostics"]["grade"].isObject() ? a["diagnostics"]["grade"] : Json()},
                      {"warnings", a["warnings"].isArray() ? a["warnings"] : Json::array()},
                      {"dataInfo", a["dataInfo"]},
                      {"proposal", p.isObject() ? Json::obj({{"changes", p["changes"]}, {"margins", p["margins"]}, {"overshootPct", p["overshootPct"]}}) : Json()}});
  };
  Json res;
  if (result.isObject()) {
    Json axes = Json::object();
    for (const auto& [k, a] : result["axes"].items()) axes.set(k, slimAxis(a));
    const Json& nz = result["noise"];
    Json noise;
    if (nz.isObject())
      noise = Json::obj({{"available", nz["available"]}, {"reason", nz["reason"]},
                         {"recommended", nz["recommended"].isObject() ? Json::obj({{"params", nz["recommended"]["params"]}, {"gyroFilterHz", nz["recommended"]["gyroFilterHz"]},
                                                                                   {"why", nz["recommended"]["why"]}})
                                                                      : Json()}});
    res = Json::obj({{"family", result["family"]}, {"firmware", result["firmware"]}, {"window", result["flightWindow"]},
                     {"chosenAxes", result["chosenAxes"]}, {"axes", axes}, {"noise", noise}});
  }
  Json notes = notes_;
  if (notes.size() > 20) notes.arr().erase(notes.arr().begin(), notes.arr().end() - 20);
  return Json::obj({{"phase", phase_},
                    {"info", info_},
                    {"since", since_},
                    {"opts", Json::obj({{"axes", opts["axes"]}, {"preset", opts["preset"]}, {"seconds", opts["seconds"]}})},
                    {"mark", mark_},
                    {"savedMark", savedMark_},
                    {"mode", v.isObject() ? v["modeName"] : Json()},
                    {"armed", v.isObject() && v["armed"].asBool()},
                    {"modeOk", modeOk(v)},
                    {"alive", link_.alive()},
                    {"linkDesc", link_.description()},
                    {"usb", usb_},
                    {"progress", progress_},
                    {"result", res},
                    {"rows", rows_},
                    {"writes", writes_},
                    {"files", files},
                    {"notes", notes},
                    {"error", error_}});
}

}  // namespace aptuner::app
