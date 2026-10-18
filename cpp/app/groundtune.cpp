#include "groundtune.hpp"

#include <algorithm>
#include <cmath>
#include <regex>

#include "aptuner/core/jsnum.hpp"
#include "aptuner/core/parammeta.hpp"
#include "aptuner/engine/identity.hpp"
#include "aptuner/engine/issues.hpp"
#include "aptuner/engine/loganalysis.hpp"
#include "aptuner/engine/report.hpp"
#include "aptuner/io/sysinfo.hpp"
#include "common.hpp"
#include "flight.hpp"
#include "paramwrite.hpp"

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
/** fmt(v) of src/engine/groundtune.js: +v.toPrecision(6) */
std::string fmt6(const Json& v) { return v.isNumber() ? core::jsToString(core::jsRoundPrecision(v.num(), 6)) : engine::jsText(v); }
std::string joinV(const std::vector<std::string>& v, const std::string& sep) {
  std::string s;
  for (std::size_t i = 0; i < v.size(); i++) s += (i ? sep : "") + v[i];
  return s;
}
Json strings(const std::vector<std::string>& v) {
  Json a = Json::array();
  for (const auto& x : v) a.push(x);
  return a;
}
bool contains(const std::vector<std::string>& v, const std::string& s) { return std::find(v.begin(), v.end(), s) != v.end(); }
}  // namespace

GroundTune::GroundTune(io::Link& link, const std::string& sessionsDir, Journal* journal)
    : link_(link), dl_(link), dir_(pathStr(fsPath(sessionsDir) / "ground")) {
  if (journal) {
    audit_ = journal;
  } else {
    ownJournal_ = std::make_unique<Journal>(pathStr(fsPath(sessionsDir) / "journal"));
    audit_ = ownJournal_.get();
  }
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
  identity_ = proposal_ = logCheck_ = reboot_ = lastTxn_ = Json();
  rebootPending_ = Json::array();
  afterReboot_ = Json::array();
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
  if (phase_ == "writing" || phase_ == "rebooting") throw rt::Error("the app is writing to the autopilot: wait until it has finished");
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
  rows_ = engine::proposalRows(*fam, axes, r["noise"], r["params"]);
  Json chosenJ = Json::array();
  for (const auto& a : chosen) chosenJ.push(a);
  r.set("chosenAxes", chosenJ);
  result = r;
  co_await review();
  setPhase("review", rows_.size() ? core::jsToString(static_cast<double>(rows_.size())) + " change(s) proposed: check them, then press Write to the autopilot"
                                  : "no change proposed: see the issues per axis");
}

rt::Task<void> GroundTune::reviewReader(std::shared_ptr<std::size_t> next) {
  while (*next < rows_.size()) {
    const std::size_t i = (*next)++;
    const std::string name = rows_[i]["name"].str();
    std::optional<double> v;
    try {
      v = co_await link_.requestParam(name, 1200, 2);
    } catch (const std::exception&) {
    }
    rows_.arr()[i].set("vehicle", v ? Json(*v) : Json());
  }
}

rt::Task<void> GroundTune::review() {
  // the value on the autopilot now for every row, the checks, whether the log is from this vehicle; one proposal
  // (id + hash over this connection and the exact values) that Write has to name
  co_await link_.waitAutopilot(3000);
  identity_ = engine::sessionIdentity(link_.identity(), Json());
  logCheck_ = engine::logVehicleCheck(result, identity_);
  for (const auto& n : logCheck_["notes"].arr()) note("warn", n.str());
  bool enablesNotch = false;
  for (const auto& r : rows_.arr())
    if (r["name"].strOr("") == "INS_HNTCH_ENABLE" && r["value"].numOr(0) > 0) enablesNotch = true;
  // the values on the autopilot now, read a few at a time (a missing one costs its timeouts only once)
  auto next = std::make_shared<std::size_t>(0);
  std::vector<rt::Future<void>> readers;
  for (int k = 0; k < 4; k++) readers.push_back(reviewReader(next));
  co_await rt::whenAll(readers);
  std::vector<std::string> problems;
  for (const auto& p : logCheck_["problems"].arr()) problems.push_back(p.str());
  for (auto& row : rows_.arr()) {
    const std::string name = row["name"].str();
    const core::ParamCheck v = core::validateParam(name, row["value"]);
    row.set("reboot", v.reboot);
    row.set("warnings", strings(v.warnings));
    row.set("errors", strings(v.errors));
    row.set("same", false);
    row.set("deferred", false);
    row.set("blocked", Json());
    if (!v.ok) {
      row.set("blocked", name + " " + joinV(v.errors, ", "));
    } else if (!row["vehicle"].isNumber()) {
      // the notch values may only appear once the notch is switched on (after the reboot): they are written then
      if (enablesNotch && name.rfind("INS_HNTCH_", 0) == 0 && name != "INS_HNTCH_ENABLE")
        row.set("deferred", true);
      else
        row.set("blocked", "not found on the autopilot");
    } else if (engine::sameF32(row["vehicle"], row["value"])) {
      row.set("same", true);
    } else if (row["from"].isNumber() && !engine::sameF32(row["vehicle"], row["from"])) {
      row.set("blocked", "the autopilot holds " + fmt6(row["vehicle"]) + ", the log had " + fmt6(row["from"]) +
                             ": the change was worked out for other values (fly again with the values now on the autopilot)");
    }
    if (row["blocked"].isNull() && !problems.empty()) row.set("blocked", joinV(problems, "; "));
  }
  Json prow = Json::array(), logRows = Json::array();
  for (const auto& x : rows_.arr()) {
    prow.push(Json::obj({{"name", x["name"]}, {"from", x["vehicle"]}, {"to", x["value"]}}));
    logRows.push(Json::obj({{"name", x["name"]}, {"log", x["from"]}, {"vehicle", x["vehicle"]}, {"to", x["value"]}, {"reboot", x["reboot"]}, {"blocked", x["blocked"]},
                            {"deferred", x["deferred"]}, {"same", x["same"]}, {"warnings", x["warnings"]}}));
  }
  proposal_ = engine::makeProposal("log", "", identity_, prow);
  audit_->append("identity", Json::obj({{"identity", identity_}, {"via", "ground review"}, {"log", logCheck_["log"]}, {"logProblems", logCheck_["problems"]}}));
  audit_->append("proposal", Json::obj({{"proposal", proposal_["id"]}, {"hash", proposal_["hash"]}, {"source", "log"}, {"file", files["bin"]},
                                        {"identity", identity_["key"]}, {"rows", logRows}}));
}

rt::Task<void> GroundTune::recheck() {
  if (!result.isObject() || (phase_ != "review" && phase_ != "done")) throw rt::Error("nothing to check yet");
  if (!link_.alive()) throw rt::Error("the autopilot is not connected");
  co_await review();
  writes_ = Json::array();
  setPhase("review", "values read again: check them, then press Write to the autopilot");
}

rt::Task<Json> GroundTune::write(std::vector<std::string> names, Json approval) {
  if (phase_ != "review" && phase_ != "done") throw rt::Error("nothing to write yet");
  const Json p = proposal_;
  if (!p.isObject() || !approval.isObject() || !approval["id"].isString() || approval["id"].str() != p["id"].str() || !approval["hash"].isString() ||
      approval["hash"].str() != p["hash"].str()) {
    audit_->append("refused", Json::obj({{"what", "ground write"}, {"proposal", p.isObject() ? p["id"] : Json()}, {"approval", approval}, {"why", "proposal id / hash mismatch"}}));
    throw rt::Error("the list changed since it was shown: look at it again, then press Write");
  }
  if (!link_.alive()) throw rt::Error("the autopilot is not connected");
  const Json id = link_.identity();
  if (p["gen"].numOr(-1) != link_.generation() || !(p["sysid"] == id["sysid"]) || !(p["compid"] == id["compid"])) {
    audit_->append("refused", Json::obj({{"what", "ground write"}, {"proposal", p["id"]}, {"why", "another connection"}, {"now", id}}));
    throw rt::Error("the connection changed since the review: press \"Read the values again\", check them, then Write");
  }
  if (link_.vehicle()["armed"].asBool()) throw rt::Error("the vehicle is armed: disarm first");
  if (!audit_->ok()) throw rt::Error("the write journal cannot be written (" + audit_->error() + "): nothing is written. Free some disk space and restart AP Tuner");
  Json rows = Json::array();
  for (const auto& r : rows_.arr())
    if (contains(names, r["name"].strOr(""))) rows.push(r);
  if (!rows.size()) throw rt::Error("choose at least one change");
  std::string blocked;
  for (const auto& r : rows.arr())
    if (!r["blocked"].isNull()) blocked += (blocked.empty() ? "" : "; ") + r["name"].str() + " (" + r["blocked"].str() + ")";
  if (!blocked.empty()) throw rt::Error("cannot be written: " + blocked);
  Json namesJ = Json::array();
  for (const auto& n : names) namesJ.push(n);
  audit_->append("approval", Json::obj({{"proposal", p["id"]}, {"hash", p["hash"]}, {"by", "user"}, {"via", "ground write"}, {"names", namesJ}, {"identity", identity_["key"]}}));
  setPhase("writing", "writing");
  const std::string stamp = isoStampForFile().substr(0, 19);
  // the notch switch first; values that only exist once the notch is on wait for the reboot
  Json now = Json::array(), later = Json::array();
  for (const auto& r : rows.arr()) (r["deferred"].asBool() ? later : now).push(r);
  std::stable_sort(now.arr().begin(), now.arr().end(), [](const Json& a, const Json& b) { return a["name"].strOr("") == "INS_HNTCH_ENABLE" && b["name"].strOr("") != "INS_HNTCH_ENABLE"; });
  Json items = Json::array();
  for (const auto& r : now.arr()) items.push(Json::obj({{"name", r["name"]}, {"to", r["value"]}, {"expect", r["vehicle"]}, {"reason", r["label"]}}));
  TxnOpts to;
  to.journal = [this](const std::string& k, const Json& d) { audit_->append(k, d); };
  to.ctx = Json::obj({{"proposal", p["id"]}, {"hash", p["hash"]}, {"identity", identity_["key"]}, {"via", "ground write"}});
  // the UNDO file is on disk before the first value is written (otherwise nothing is written)
  to.afterPreRead = [this, stamp, rows](const Json& before) {
    files.set("undo", pathStr(fsPath(dir_) / (stamp + " UNDO (values before AP Tuner).param")));
    Json undoRows = Json::array();
    for (const auto& b : before.arr()) {
      Json group;
      for (const auto& r : rows.arr())
        if (r["name"].strOr("") == b["name"].strOr("")) {
          group = r["group"];
          break;
        }
      undoRows.push(Json::obj({{"group", group}, {"name", b["name"]}, {"value", b["before"]}}));
    }
    if (!writeText(files["undo"].str(), engine::paramFile(undoRows, {"Values on the autopilot before AP Tuner wrote, " + localNow(), "Load this file to go back."}, identity_, false)))
      throw std::runtime_error("the UNDO file could not be saved (" + files["undo"].str() + ")");
  };
  const Json txn = co_await runTransaction(link_, items, to);
  lastTxn_ = txn;
  Json writes = Json::array();
  for (const auto& r : rows.arr()) writes.push(rowResult(r, txn));
  writes_ = writes;
  // what acts only after a reboot (written and read back) and what can only be written after it
  rebootPending_ = Json::array();
  for (const auto& w : writes_.arr())
    if (w["reboot"].asBool() && w["status"].str() == "verified") rebootPending_.push(Json::obj({{"name", w["name"]}, {"value", w["to"]}}));
  afterReboot_ = Json::array();
  if (txn["ok"].asBool())
    for (const auto& r : later.arr())
      afterReboot_.push(Json::obj({{"name", r["name"]}, {"value", r["value"]}, {"group", r["group"]}, {"label", r["label"]}, {"reboot", r["reboot"].asBool()}}));
  Json written = Json::array();
  for (const auto& w : writes_.arr())
    if (w["status"].str() == "verified") written.push(Json::obj({{"group", w["group"]}, {"name", w["name"]}, {"value", w["to"]}, {"from", w["from"]}}));
  if (written.size()) {
    files.set("param", pathStr(fsPath(dir_) / (stamp + " written by AP Tuner.param")));
    if (!writeText(files["param"].str(), engine::paramFile(written, {"Written to the autopilot by AP Tuner (Telemetry + USB) and read back, " + localNow()}, identity_))) {
      files.erase("param");
      note("warn", "the file of the written values could not be saved");
    }
  }
  std::string tail = " Unplug the USB and fly to check";
  if (rebootPending_.size() || afterReboot_.size()) {
    std::string list;
    for (const auto& x : rebootPending_.arr()) list += (list.empty() ? "" : ", ") + x["name"].str();
    for (const auto& x : afterReboot_.arr()) list += (list.empty() ? "" : ", ") + x["name"].str();
    tail = " " + list + (rebootPending_.size() + afterReboot_.size() == 1 ? " takes" : " take") + " effect only after a reboot: press Reboot (disarmed)";
  }
  setPhase("done", txn["ok"].asBool() ? "all " + core::jsToString(static_cast<double>(rows.size() - later.size())) + " written and read back." + tail : txnText(txn));
  co_return writes_;
}

Json GroundTune::rowResult(const Json& r, const Json& txn) const {
  // one row's outcome of a transaction, in plain words
  Json out = Json::obj({{"group", r["group"]}, {"name", r["name"]}, {"label", r["label"]}, {"requested", r["value"]}, {"reboot", r["reboot"].asBool()}, {"from", Json()},
                        {"to", Json()}, {"status", "not written"}, {"ok", false}, {"text", ""}, {"changed", false}, {"undo", Json()}});
  const std::string name = r["name"].str();
  if (r["deferred"].asBool()) {
    out.set("status", "after reboot");
    out.set("text", name + ": written after the reboot (it appears once the notch is on)");
    return out;
  }
  Json res;
  for (const auto& x : txn["rows"].arr())
    if (x["name"].strOr("") == name) res = x;
  if (!res.isObject()) {
    out.set("text", txn["aborted"].isString() ? "not written: " + txn["aborted"].str() : name + ": not written (an earlier value failed)");
    return out;
  }
  Json rb;
  for (const auto& x : txn["rollback"].arr())
    if (x["name"].strOr("") == name) rb = x;
  out.set("from", res["before"]);
  out.set("to", res["holds"]);
  out.set("status", res["status"]);
  out.set("ok", res["ok"]);
  out.set("text", writeText(res));
  if (!txn["ok"].asBool() && rb.isObject()) {
    out.set("ok", false);
    out.set("to", rb["holds"]);
    if (rb["ok"].asBool()) {
      out.set("status", "rolled back");
      out.set("text", writeText(res) + "; written back to " + fmt6(rb["holds"]) + " (read back)");
    } else {
      out.set("status", "uncertain");
      out.set("text", writeText(res) + "; writing it back failed: " + writeText(rb));
    }
  } else if (!txn["ok"].asBool() && !rb.isObject() && res["ok"].asBool()) {
    for (const auto& u : txn["uncertain"].arr())
      if (u["name"].strOr("") == name) {
        out.set("status", "uncertain");
        out.set("ok", false);
        out.set("text", u["why"]);
      }
  }
  const std::string st = out["status"].str();
  out.set("changed", changedStatus(st) && !(st == "verified" && engine::sameF32(out["to"], out["from"])));
  return out;
}

rt::Task<Json> GroundTune::rebootNow() {
  if (phase_ != "done") throw rt::Error("nothing written yet");
  if (!rebootPending_.size() && !afterReboot_.size()) throw rt::Error("nothing written here needs a reboot");
  // what the vehicle holds now: the value written, or after an Undo the value from before
  Json expect = Json::array();
  for (const auto& w : writes_.arr())
    if (w["status"].str() == "verified")
      expect.push(Json::obj({{"name", w["name"]}, {"value", w["undo"].isString() && w["undoHolds"].isNumber() ? w["undoHolds"] : w["to"]}}));
  setPhase("rebooting", "rebooting the autopilot: it disconnects for a few seconds");
  RebootOpts ro;
  ro.journal = [this](const std::string& k, const Json& d) { audit_->append(k, d); };
  ro.ctx = Json::obj({{"via", "ground write"}, {"identity", identity_.isObject() ? identity_["key"] : Json()}});
  ro.onInfo = [this](const std::string& t) { info_ = t; };
  const Json res = co_await rebootAndVerify(link_, expect, ro);
  reboot_ = Json::obj({{"at", Loop::now()}, {"ok", res["ok"]}, {"reason", res["reason"]}, {"rows", res["rows"]}});
  for (auto& w : writes_.arr())
    for (const auto& x : res["rows"].arr())
      if (x["name"].strOr("") == w["name"].strOr(""))
        w.set("afterReboot", x["ok"].asBool() ? "read back after the reboot" : (x["holds"].isNull() ? "no answer after the reboot" : "holds " + fmt6(x["holds"]) + " after the reboot"));
  if (!res["ok"].asBool()) {
    setPhase("done", "reboot: " + res["reason"].strOr(""));
    co_return reboot_;
  }
  rebootPending_ = Json::array();
  // the identity of the new connection (the link may have reopened): the same vehicle was checked by rebootAndVerify
  identity_ = engine::sessionIdentity(link_.identity(), Json());
  if (afterReboot_.size()) {
    const Json later = afterReboot_;
    afterReboot_ = Json::array();
    Json items = Json::array();
    for (const auto& r : later.arr()) items.push(Json::obj({{"name", r["name"]}, {"to", r["value"]}, {"reason", r["label"]}}));
    TxnOpts to;
    to.journal = [this](const std::string& k, const Json& d) { audit_->append(k, d); };
    to.ctx = Json::obj({{"via", "ground write after the reboot"}, {"identity", identity_["key"]}});
    const Json txn = co_await runTransaction(link_, items, to);
    for (const auto& r : later.arr()) {
      Json row = r;
      row.set("deferred", false);
      const Json nw = rowResult(row, txn);
      for (auto& w : writes_.arr())
        if (w["name"].strOr("") == r["name"].strOr(""))
          for (const auto& [k, v] : nw.items()) w.set(k, v);
    }
    Json pend = Json::array();
    for (const auto& w : writes_.arr()) {
      bool isLater = false;
      for (const auto& l : later.arr())
        if (l["name"].strOr("") == w["name"].strOr("")) isLater = true;
      if (w["reboot"].asBool() && w["status"].str() == "verified" && isLater) pend.push(Json::obj({{"name", w["name"]}, {"value", w["to"]}}));
    }
    rebootPending_ = pend;
    std::string pendNames;
    for (const auto& x : pend.arr()) pendNames += (pendNames.empty() ? "" : ", ") + x["name"].str();
    setPhase("done", txn["ok"].asBool() ? "rebooted and read back; then " + core::jsToString(static_cast<double>(later.size())) + " notch value(s) written and read back" +
                                              (pend.size() ? ": " + pendNames + " need one more reboot (press Reboot)" : "")
                                        : "after the reboot: " + txnText(txn));
    co_return reboot_;
  }
  setPhase("done", "rebooted: every value written here was read back after the reboot. Unplug the USB and fly to check");
  co_return reboot_;
}

rt::Task<void> GroundTune::undo() {
  // write the values from before back, each verified on its own (an undo goes as far back as it can)
  std::vector<std::size_t> todo;
  for (std::size_t i = 0; i < writes_.size(); i++) {
    const Json& w = writes_[i];
    if (w["changed"].asBool() && w["from"].isNumber() && w["undo"].strOr("") != "undone") todo.push_back(i);
  }
  if (todo.empty()) throw rt::Error("nothing to undo: no value written here is different from before");
  if (!link_.alive()) throw rt::Error("the autopilot is not connected");
  if (link_.vehicle()["armed"].asBool()) throw rt::Error("the vehicle is armed: disarm first");
  if (identity_.isObject() && !engine::sameVehicle(identity_, link_.identity()))
    throw rt::Error("another vehicle is connected now (" + engine::identityText(link_.identity()) + "): the undo values are for " + engine::identityText(identity_));
  setPhase("writing", "writing the previous values back");
  Json namesJ = Json::array();
  for (const auto i : todo) namesJ.push(writes_[i]["name"]);
  audit_->append("approval", Json::obj({{"by", "user"}, {"via", "ground undo"}, {"names", namesJ}, {"identity", identity_.isObject() ? identity_["key"] : Json()}}));
  for (const auto i : todo) {
    const std::string name = writes_[i]["name"].str();
    const double from = writes_[i]["from"].num();
    std::optional<double> cur;
    try {
      cur = co_await link_.requestParam(name, 1200, 2);
    } catch (const std::exception&) {
    }
    Json r;
    if (cur && engine::sameF32(*cur, from)) {
      r = Json::obj({{"name", name}, {"status", "unchanged"}, {"ok", true}, {"holds", *cur}});
    } else {
      VerifyOpts vo;
      vo.before = cur ? Json(*cur) : Json();
      r = co_await verifiedSet(link_, name, from, vo);
    }
    Json ev = Json::obj({{"via", "ground undo"}, {"identity", identity_.isObject() ? identity_["key"] : Json()}});
    for (const auto& [k, v] : r.items()) ev.set(k, v);
    audit_->append("write", ev);
    Json& w = writes_.arr()[i];
    w.set("undo", r["ok"].asBool() ? Json("undone") : r["status"]);
    w.set("undoText", r["ok"].asBool() ? "back to " + fmt6(r["holds"]) + " (read back)" : writeText(r));
    w.set("undoHolds", r["holds"]);
    w.set("afterReboot", Json());   // the last reboot's check was about the value written, not this one
    if (r["ok"].asBool() && w["reboot"].asBool() && r["status"].str() == "verified") {
      Json next = Json::array();
      for (const auto& x : rebootPending_.arr())
        if (x["name"].strOr("") != name) next.push(x);
      next.push(Json::obj({{"name", name}, {"value", r["holds"]}}));
      rebootPending_ = next;
    }
  }
  std::string bad;
  for (const auto i : todo)
    if (writes_[i]["undo"].strOr("") != "undone") bad += (bad.empty() ? "" : "; ") + writes_[i]["undoText"].strOr("");
  std::string pend;
  for (const auto& x : rebootPending_.arr()) pend += (pend.empty() ? "" : ", ") + x["name"].str();
  setPhase("done", !bad.empty() ? "undo incomplete: " + bad + ". Load the UNDO file in Mission Planner to go back"
                                : "previous values written back and read back" +
                                      (pend.empty() ? "" : "; " + pend + (rebootPending_.size() == 1 ? " takes" : " take") + " effect after a reboot (press Reboot)"));
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
                    {"error", error_},
                    {"proposal", proposal_.isObject() ? Json::obj({{"id", proposal_["id"]}, {"hash", proposal_["hash"]}}) : Json()},
                    {"identity", identity_},
                    {"identityText", identity_.isObject() ? Json(engine::identityText(identity_)) : Json()},
                    {"logCheck", logCheck_},
                    {"rebootPending", rebootPending_},
                    {"afterReboot", afterReboot_},
                    {"reboot", reboot_},
                    {"audit", audit_->status()}});
}

}  // namespace aptuner::app
