#include "flight.hpp"

#include <algorithm>
#include <cmath>
#include <set>

#include "aptuner/core/jsnum.hpp"
#include "aptuner/engine/issues.hpp"
#include "aptuner/engine/vehicle.hpp"
#include "common.hpp"

namespace aptuner::app {

using core::jsToFixed;
using engine::jv;
using rt::Loop;

const std::vector<std::string> kFlightPhases{"preflight", "checkup", "review", "tuning", "filters", "recheck", "overall", "land", "finished"};

namespace {
const std::set<std::string> kRunning{"preflight", "checkup", "review", "tuning", "filters", "recheck", "overall", "land", "paused"};
const std::vector<std::string> kCopterModes{"STABILIZE", "ALT_HOLD", "POSHOLD", "LOITER"};
const std::vector<std::string> kVtolModes{"QSTABILIZE", "QHOVER", "QLOITER"};
const std::vector<std::string> kPlaneModes{"FLY_BY_WIRE_A", "FLY_BY_WIRE_B", "CRUISE", "STABILIZE"};

std::string replaceFirst(std::string s, const std::string& a, const std::string& b) {
  const auto p = s.find(a);
  if (p != std::string::npos) s.replace(p, a.size(), b);
  return s;
}

std::string pretty(const std::vector<std::string>& list) {
  std::string out;
  for (std::size_t i = 0; i < list.size(); i++) {
    std::string m = list[i];
    for (const auto& [a, b] : std::vector<std::pair<std::string, std::string>>{{"FLY_BY_WIRE_", "FBW"}, {"ALT_HOLD", "AltHold"}, {"POSHOLD", "PosHold"},
                                                                                {"STABILIZE", "Stabilize"}, {"LOITER", "Loiter"}, {"QHOVER", "QHover"},
                                                                                {"QLOITER", "QLoiter"}})
      m = replaceFirst(m, a, b);
    out += (i ? ", " : "") + m;
  }
  return out;
}

bool contains(const std::vector<std::string>& v, const std::string& s) { return std::find(v.begin(), v.end(), s) != v.end(); }
}  // namespace

const std::vector<std::string>& goodModes(const std::string& family) {
  if (family == "vtol") return kVtolModes;
  if (family == "plane") return kPlaneModes;
  return kCopterModes;   // copter, heli and unknown
}

FlightSession::FlightSession(io::Link& link, Tuner& tuner, NotchLive& notch) : link_(link), tuner_(tuner), notch_(notch) {
  reset();
  tuner_.on("trip", [this](const Json& t) { onTrip(t); });
}

void FlightSession::reset() {
  phase_ = "idle";
  since_ = Loop::now();
  info_.clear();
  opts_ = Json::obj({{"axes", Json(Json::Array{"roll", "pitch"})}, {"preset", "normal"}, {"auto", false}, {"observeSeconds", 30}});
  checkup_ = filters_ = overall_ = summary_ = Json();
  notes_ = Json::array();
  pausedInfo_ = Json();
  resumePhase_.clear();
  gainsStart_ = Json();
  journalStart_ = 0;
  tFlying_ = 0;
  tripPause_ = false;
  if (timer_) Loop::main().clear(timer_);
  timer_ = 0;
}

bool FlightSession::inFlight() const { return tuner_.opts["inFlightWrites"].asBool(); }

void FlightSession::onTrip(const Json& t) {
  if (!kRunning.count(phase_)) return;
  std::string text;
  for (const auto& r : t["reasons"].arr()) text += (text.empty() ? "" : "; ") + r.strOr("");
  if (phase_ == "land") {
    note("bad", "oscillation while landing: " + text);
    return;
  }
  note("bad", "oscillation: " + text);
  tripPause_ = true;
  pauseFlight("oscillation: " + text);
}

std::string FlightSession::family() const { return tuner_.family ? core::familyName(*tuner_.family) : ""; }
bool FlightSession::quadplane() const { return jv(tuner_.params, "Q_ENABLE") > 0; }
const std::vector<std::string>& FlightSession::modes() const { return goodModes(family()); }
std::string FlightSession::goodModesText() const {
  return quadplane() && phase_ == "preflight" ? pretty(kVtolModes) + " (hover tune) or " + pretty(kPlaneModes) + " (fixed-wing tune)" : pretty(modes());
}
bool FlightSession::modeOk() const {
  const Json& v = link_.vehicle();
  return v.isObject() && contains(modes(), v["modeName"].strOr(""));
}

void FlightSession::setPhase(const std::string& p, const std::string& info) {
  phase_ = p;
  since_ = Loop::now();
  info_ = info;
  emit("phase", Json::obj({{"phase", p}, {"info", info}}));
  tuner_.log("flight", p + (info.empty() ? "" : ": " + info));
}

void FlightSession::note(const std::string& level, const std::string& text) {
  notes_.push(Json::obj({{"t", Loop::now()}, {"phase", phase_}, {"level", level}, {"text", text}}));
  tuner_.log("flight", text);
}

Json FlightSession::gainsNow() const { return tuner_.snapshot()["gains"]; }

void FlightSession::start(const Json& opts) {
  if (!tuner_.prepared) throw rt::Error("read the vehicle first");
  if (kRunning.count(phase_)) throw rt::Error("a flight session is already running");
  if (opts["auto"].asBool() && !inFlight())
    throw rt::Error("\"Apply by itself\" writes in flight: turn on \"Allow changes in flight\" first (experimental, tested only in the simulator)");
  reset();
  for (const auto& [k, v] : opts.items()) opts_.set(k, v);
  tuner_.opts.set("axes", opts_["axes"]);
  tuner_.opts.set("preset", opts_["preset"]);
  tuner_.opts.set("autoApply", true);
  tuner_.beginSession();
  gainsStart_ = gainsNow();
  journalStart_ = tuner_.journal.size();
  setPhase("preflight", "arm, take off and hover at a safe height");
  tick();
}

void FlightSession::stop() {
  if (timer_) Loop::main().clear(timer_);
  timer_ = 0;
  rt::detach(tuner_.stop().future());
  setPhase("stopped", "stopped by you");
}

void FlightSession::cont() {
  if (phase_ == "review") {
    if (!checkup_.isObject() || !checkup_["summary"]["anyProposal"].asBool())
      throw rt::Error("nothing to apply yet: repeat the check-up with sharper stick taps");
    if (!inFlight()) {
      // nothing may be written in flight: the check-up is the result (apply it on the ground), or the final check
      if (checkup_["summary"]["changes"].numOr(0) != 0) {
        note("warn", "changes in flight are off, so nothing was written: land, then apply the proposals on the ground (Advanced view) or use the Telemetry + USB tab");
        finish("check-up only: changes in flight are off");
        return;
      }
      filters_ = Json::obj({{"done", true}, {"result", "off"}});
      note("warn", "changes in flight are off: the filter step is skipped");
      startOverall();
      return;
    }
    startTuning();
    return;
  }
  if (phase_ == "land" || phase_ == "paused") {
    finish(phase_ == "land" ? "finished before landing" : "finished early");
    return;
  }
  throw rt::Error("nothing to continue in phase " + phase_);
}

void FlightSession::repeatCheckup() {
  if (phase_ != "review" && phase_ != "paused") throw rt::Error("not at the check-up");
  for (const auto& ax : opts_["axes"].arr()) tuner_.axes[ax.str()].setBaseline();
  setPhase("checkup", "measuring again: short sharp stick taps on roll and pitch");
  if (!timer_) tick();
}

void FlightSession::skipFilters() {
  if (phase_ != "filters") throw rt::Error("not at the filter step");
  if (!filters_.isObject()) filters_ = Json::object();
  filters_.set("done", true);
  filters_.set("result", "skipped");
  note("warn", "filter step skipped");
}

void FlightSession::resume() {
  if (phase_ != "paused") {
    if (tuner_.state == "paused") tuner_.resume();
    return;
  }
  if (!tuner_.writing().empty()) throw rt::Error("the app is still reverting (" + tuner_.writing() + "): wait until it has finished");
  const std::string p = resumePhase_.empty() ? "checkup" : resumePhase_;
  pausedInfo_ = Json();
  tripPause_ = false;
  if (p == "tuning" || p == "recheck") {
    setPhase(p, "continuing");
    tuner_.resume();
  } else if (p == "checkup" || p == "overall") {
    for (const auto& ax : opts_["axes"].arr()) tuner_.axes[ax.str()].setBaseline();
    setPhase(p, "continuing: keep tapping the sticks");
  } else {
    setPhase(p, "continuing");
  }
  if (!timer_) tick();
}

void FlightSession::pauseFlight(const std::string& reason) {
  if (phase_ != "paused") {
    resumePhase_ = phase_;
    pausedInfo_ = reason;
    setPhase("paused", reason);
  }
}

void FlightSession::tick() {
  if (timer_) Loop::main().clear(timer_);
  timer_ = Loop::main().setTimeout(300, [this] { loopOnce(); });
}

void FlightSession::loopOnce() {
  timer_ = 0;
  rt::Future<void> f = step();
  f.then([this, f] {
    if (f.error()) {
      try {
        std::rethrow_exception(f.error());
      } catch (const std::exception& e) {
        note("bad", std::string("error: ") + e.what());
      }
    }
    timer_ = kRunning.count(phase_) ? Loop::main().setTimeout(1000, [this] { loopOnce(); }) : 0;
  });
}

rt::Task<void> FlightSession::step() {
  const Json v = link_.vehicle();
  const bool armed = v.isObject() && v["armed"].asBool();
  const double elapsed = (Loop::now() - since_) / 1000;
  if (!link_.alive()) {
    info_ = "link lost: waiting for the vehicle";
    co_return;
  }
  const std::string modeName = v["modeName"].strOr("");
  if (phase_ == "preflight") {
    if (!armed) {
      tFlying_ = 0;
      info_ = "arm and take off, then hover at a safe height in " + goodModesText();
      co_return;
    }
    if (quadplane()) {   // a QuadPlane: tune the controller of the mode being flown
      const std::string want = (!modeName.empty() && modeName[0] == 'Q') ? "vtol" : "plane";
      if (want != family() && contains(goodModes(want), modeName)) {
        info_ = std::string("switching to the ") + (want == "vtol" ? "hover (VTOL)" : "fixed-wing") + " controller";
        tFlying_ = 0;
        co_await tuner_.switchFamily(*core::parseFamily(want));
        tuner_.opts.set("axes", opts_["axes"]);
        tuner_.opts.set("preset", opts_["preset"]);
        tuner_.opts.set("autoApply", true);
        tuner_.beginSession();   // the values to go back to are those of the controller now flown
        gainsStart_ = gainsNow();
        note("ok", "QuadPlane in " + modeName + ": tuning the " + (want == "vtol" ? "hover (Q_A_*)" : "fixed-wing") + " gains");
        co_return;
      }
    }
    if (!modeOk()) {
      tFlying_ = 0;
      info_ = "flying in " + modeName + ": switch to " + goodModesText();
      co_return;
    }
    if (tFlying_ == 0) tFlying_ = Loop::now();
    if (Loop::now() - tFlying_ < 5000) {
      info_ = "flying in " + modeName + ": the check-up starts in a moment";
      co_return;
    }
    for (const auto& ax : opts_["axes"].arr()) tuner_.axes[ax.str()].setBaseline();
    tuner_.requestStreams();
    setPhase("checkup", "fly normally and tap the sticks: short sharp inputs on roll and pitch");
    co_return;
  }
  if (phase_ == "checkup" || phase_ == "overall") {
    if (!armed) {
      pauseFlight("the vehicle disarmed: take off again and press Resume, or press Finish");
      co_return;
    }
    if (!modeOk()) {
      info_ = "switch back to " + goodModesText() + " (now " + modeName + ")";
      since_ = Loop::now();
      co_return;
    }
    const double w = opts_["observeSeconds"].numOr(30);
    if (elapsed < w) {
      info_ = std::string(phase_ == "checkup" ? "measuring how it flies now" : "final check with the new gains") + ": " +
              core::jsToString(std::ceil(w - elapsed)) + " s left, keep tapping the sticks";
      co_return;
    }
    Json res = Json::object();
    const auto fam = core::parseFamily(family());
    for (const auto& axJ : opts_["axes"].arr()) {
      const std::string ax = axJ.str();
      Json r = tuner_.analyze(ax);
      const Json issues = engine::axisIssues(r, fam ? *fam : core::Family::Copter);
      r.set("issues", issues);
      if (tuner_.results[ax].isObject()) tuner_.results.at(ax).set("issues", issues);
      res.set(ax, r);
    }
    const Json summary = summarise(res);
    if (phase_ == "checkup") {
      Json checks = tuner_.safetyChecks();
      checkup_ = Json::obj({{"results", res}, {"summary", summary}, {"gains", gainsNow()}, {"checks", checks}, {"t", Loop::now()}});
      checkup_.set("vehicleIssues", engine::vehicleIssues(checks));
      if (!summary["anyProposal"].asBool()) {
        setPhase("review", "nothing could be proposed yet (too little stick input): fly sharper taps and press Repeat");
        co_return;
      }
      if (summary["changes"].numOr(0) == 0) {
        note("ok", "the gains already match the measurement: nothing to apply");
        setPhase("review", "the gains are already right; press Continue for the filter and final check, or Repeat");
        co_return;
      }
      if (opts_["auto"].asBool()) {
        note("ok", "auto mode: applying the changes now");
        startTuning();
        co_return;
      }
      setPhase("review", core::jsToString(summary["changes"].num()) + " change(s) proposed: read the issues, then press Apply in flight");
      co_return;
    }
    const std::string hoverName = family() == "vtol" ? "Q_M_THST_HOVER" : "MOT_THST_HOVER";
    Json hover;
    if (family() != "plane") {
      try {
        hover = co_await link_.requestParam(hoverName);
      } catch (const std::exception&) {
        hover = tuner_.params[hoverName];
      }
    }
    overall_ = Json::obj({{"results", res}, {"summary", summary}, {"gains", gainsNow()}, {"vibe", tuner_.vibe}, {"hover", hover},
                          {"notch", jv(tuner_.params, "INS_HNTCH_ENABLE") > 0}, {"t", Loop::now()}});
    overall_.set("verdict", verdict(overall_));
    setPhase("land", overall_["verdict"]["ok"].asBool() ? "all good: land when you are ready"
                                                         : "land when you are ready; the list shows what still needs attention");
    co_return;
  }
  if (phase_ == "tuning" || phase_ == "recheck") {
    const std::string st = tuner_.state;
    if (st == "done") {
      if (phase_ == "tuning")
        co_await startFilters();
      else
        startOverall();
      co_return;
    }
    if (st == "paused") {
      if (!armed) {
        pauseFlight("the vehicle disarmed during tuning: take off again and press Resume, or press Finish");
        co_return;
      }
      pausedInfo_ = tuner_.stateInfo;
      info_ = "paused: " + tuner_.stateInfo + " (press Resume when it flies normally)";
      co_return;
    }
    if (st == "idle") {
      pauseFlight("the tuner stopped: press Resume or Finish");
      co_return;
    }
    pausedInfo_ = Json();
    info_ = tuner_.stateInfo;
    co_return;
  }
  if (phase_ == "filters") {
    // after the filter step the gains are always checked again (all chosen axes), filter changed or not;
    // without in-flight changes there is nothing to re-tune: straight to the final check
    if (filters_.isObject() && filters_["done"].asBool()) {
      if (!inFlight())
        startOverall();
      else
        startRecheck();
    }
    co_return;
  }
  if (phase_ == "land") {
    if (!armed) finish("landed");
    co_return;
  }
  if (phase_ == "paused") {
    // after a trip the tuner's message says what was reverted and what to do next
    if (tripPause_ && tuner_.state == "paused") {
      pausedInfo_ = tuner_.stateInfo;
      info_ = tuner_.stateInfo;
    }
    co_return;
  }
}

void FlightSession::startTuning() {
  tuner_.opts.set("useFreshResult", true);   // the check-up analysis is applied right away, then the normal rounds
  tuner_.start();
  setPhase("tuning", "applying the changes in flight: small steps, each watched for oscillation");
}

rt::Task<void> FlightSession::startFilters() {
  setPhase("filters", "checking the notch filter options");
  filters_ = Json::obj({{"started", Loop::now()}, {"done", false}});
  std::string failed, failedCode;
  try {
    const Json st = co_await notch_.status();
    filters_.set("status", Json::obj({{"notchActive", st["notchActive"]}, {"mode", st["mode"]}, {"esc", st["escTelemetry"]}, {"fft", st["fftCapable"]},
                                      {"fftEnabled", st["fftEnabled"]}, {"freq", st["current"]["INS_HNTCH_FREQ"]}}));
    if (st["notchActive"].asBool()) {
      filters_.set("result", "kept");
      note("ok", "notch filter already configured (mode " + core::jsToString(st["mode"].numOr(0)) + ", " + fmt4(st["current"]["INS_HNTCH_FREQ"]) + " Hz): kept");
    } else if (!inFlight()) {
      filters_.set("result", "off");
      note("warn", "changes in flight are off: the notch filter was not changed (set it on the ground, from a log)");
    } else if (st["escProposal"].isObject()) {
      info_ = "setting the notch to the ESC RPM; every axis is watched for oscillation for a few seconds";
      const Json a = co_await notch_.applyEsc();
      filters_.set("result", "esc");
      filters_.set("applied", a);
      std::string list;
      for (const auto& [k, v] : a.items()) {
        std::string kk = k;
        if (kk.rfind("INS_HNTCH_", 0) == 0) kk = kk.substr(10);
        list += (list.empty() ? "" : ", ") + kk + " " + core::jsToString(v.numOr(0));
      }
      note("ok", "notch filter set to track the ESC RPM: " + list);
    } else if (st["fftCapable"].asBool() && st["fftEnabled"].asBool() && link_.vehicle()["armed"].asBool()) {
      info_ = "measuring the motor noise with the autopilot FFT: hold a steady hover for 30 s, sticks centred";
      co_await notch_.startFftTune();
      // 30 s of measuring; a pause, stop or trip ends it early (the tune is stopped either way)
      for (int i = 0; i < 120 && phase_ == "filters" && tuner_.state != "paused"; i++) co_await rt::sleep(250);
      const Json r = co_await notch_.stopFftTune();
      const bool ok = r["ok"].asBool();
      const bool reverted = r["held"].isBool() && !r["held"].asBool();
      filters_.set("result", ok ? "fft" : reverted ? "reverted" : "none");
      filters_.set("applied", r["changed"]);
      note(ok ? "ok" : "warn", ok ? "the autopilot measured its motor noise and wrote the notch filter"
                               : reverted ? "the notch the autopilot wrote made the vehicle oscillate and was reverted"
                                          : "the FFT measurement wrote nothing (it needs a steadier hover): set the notch from a log after landing");
    } else {
      filters_.set("result", "none");
      note("warn", st["fftCapable"].asBool()
                       ? "no ESC RPM telemetry, and the autopilot FFT is off (enable it and reboot on the ground): set the notch from a log after landing"
                       : "no in-flight route for the notch on this autopilot (no ESC RPM, no FFT): set it from a log after landing");
    }
  } catch (const rt::Error& e) {
    failed = e.what();
    failedCode = e.code();
  } catch (const std::exception& e) {
    failed = e.what();
  }
  if (!failed.empty()) {
    const bool reverted = failedCode == "reverted";
    filters_.set("result", reverted ? "reverted" : "none");
    note(reverted ? "bad" : "warn", (reverted ? "" : "filter step failed: ") + failed);
  }
  filters_.set("done", true);
}

void FlightSession::startRecheck() {
  tuner_.opts.set("useFreshResult", false);
  tuner_.start();
  setPhase("recheck", "checking the gains again with the filter active");
}

void FlightSession::startOverall() {
  for (const auto& ax : opts_["axes"].arr()) tuner_.axes[ax.str()].setBaseline();
  setPhase("overall", "final check: fly normally for 30 s with a few stick taps");
}

Json FlightSession::summarise(const Json& res) const {
  int changes = 0, bad = 0, warn = 0;
  bool anyProposal = false;
  std::string worst = "A";
  const std::string order = "ABCDF";
  for (const auto& [ax, r] : res.items()) {
    if (r["proposal"].isObject()) {
      anyProposal = true;
      bool big = false;
      for (const auto& c : r["proposal"]["changes"].arr()) {
        const std::string k = c["key"].strOr("");
        if ((k == "P" || k == "D" || k == "FF" || k == "angleP" || k == "tconst") && std::fabs(c["pct"].numOr(0)) > 10) big = true;
      }
      if (big) changes++;
    }
    for (const auto& i : r["issues"].arr()) {
      if (i["level"].strOr("") == "bad")
        bad++;
      else if (i["level"].strOr("") == "warn")
        warn++;
    }
    const std::string g = r["diagnostics"]["grade"]["letter"].strOr("");
    if (!g.empty() && order.find(g) != std::string::npos && order.find(g) > order.find(worst)) worst = g;
  }
  return Json::obj({{"changes", changes}, {"anyProposal", anyProposal}, {"bad", bad}, {"warn", warn}, {"worst", worst}});
}

Json FlightSession::verdict(const Json& ov) const {
  Json items = Json::array();
  bool ok = true;
  for (const auto& [ax, r] : ov["results"].items()) {
    const Json& g = r["diagnostics"]["grade"];
    const Json& d = r["diagnostics"];
    if (!g.isObject()) {
      items.push(Json::obj({{"ok", false}, {"text", ax + ": no measurement (too little stick input)"}}));
      ok = false;
      continue;
    }
    if (g["gradable"].isBool() && !g["gradable"].asBool()) {
      items.push(Json::obj({{"ok", false}, {"text", ax + ": not gradable (too little stick input on this axis): fly a few sharp stick taps to check it"}}));
      ok = false;
      continue;
    }
    const std::string letter = g["letter"].strOr("");
    const bool good = (letter == "A" || letter == "B") && !d["oscillating"].asBool();
    if (!good) ok = false;
    const std::string tracking = d["trackingPct"].isNumber() ? jsToFixed(d["trackingPct"].num(), 0) + "%" : "n/a";
    items.push(Json::obj({{"ok", good},
                          {"text", ax + ": grade " + letter + ", follows " + tracking + " of the demand, error " +
                                       (d["rmsErrorDegs"].isNumber() ? jsToFixed(d["rmsErrorDegs"].num(), 1) : std::string("undefined")) +
                                       " deg/s" + (d["oscillating"].asBool() ? ", oscillating" : "")}}));
    if (r["proposal"].isObject()) {
      bool big = false;
      std::string list;
      for (const auto& c : r["proposal"]["changes"].arr()) {
        const std::string k = c["key"].strOr("");
        if (k != "P" && k != "D") continue;
        if (std::fabs(c["pct"].numOr(0)) > 20) big = true;
        list += (list.empty() ? "" : ", ") + k + " " + core::jsToString(core::jsRound(c["pct"].numOr(0))) + "%";
      }
      if (big) items.push(Json::obj({{"ok", false}, {"text", ax + ": the gains could still move more than 20% (" + list + "): run another session later"}}));
    }
  }
  if (ov["vibe"].isObject()) {
    const Json& v = ov["vibe"];
    const double vib = std::max({v["x"].numOr(0), v["y"].numOr(0), v["z"].numOr(0)});
    const double clip = v["clip"].numOr(0);
    const bool good = vib < 30 && clip == 0;
    if (!good) ok = false;
    items.push(Json::obj({{"ok", good},
                          {"text", "vibration " + jsToFixed(vib, 0) + " (ArduPilot: below 30 is good)" +
                                       (clip != 0 ? ", " + core::jsToString(clip) + " accelerometer clips" : "")}}));
  }
  if (ov["hover"].isNumber()) {
    const double h = ov["hover"].num();
    const bool good = h > 0.2 && h < 0.7;
    items.push(Json::obj({{"ok", good},
                          {"text", "hover throttle " + jsToFixed(h, 2) + (good ? " (normal range)" : " (outside 0.2 to 0.7: check the thrust-to-weight ratio)")}}));
  }
  const bool notch = ov["notch"].asBool();
  items.push(Json::obj({{"ok", notch}, {"text", notch ? "harmonic notch filter active" : "no harmonic notch filter yet: set it from a log after landing"}}));
  return Json::obj({{"ok", ok}, {"items", items}});
}

void FlightSession::finish(const std::string& reason) {
  if (timer_) Loop::main().clear(timer_);
  timer_ = 0;
  if (tuner_.state != "idle") rt::detach(tuner_.stop().future());
  const Json gainsEnd = gainsNow();
  auto pick = [](const Json& res) -> Json {
    if (!res.isObject()) return Json();
    Json o = Json::object();
    for (const auto& [ax, r] : res.items()) {
      const Json& d = r["diagnostics"];
      o.set(ax, Json::obj({{"grade", d["grade"].isObject() ? d["grade"]["letter"] : Json()},
                           {"score", d["grade"].isObject() ? d["grade"]["score"] : Json()},
                           {"trackingPct", d.isObject() ? d["trackingPct"] : Json()},
                           {"rmsErrorDegs", d.isObject() ? d["rmsErrorDegs"] : Json()},
                           {"oscRmsDegs", d.isObject() ? d["oscRmsDegs"] : Json()},
                           {"step", r["measuredStep"].isObject() ? r["measuredStep"]["metrics"] : Json()}}));
    }
    return o;
  };
  summary_ = Json::obj({{"reason", reason},
                        {"before", checkup_.isObject() ? pick(checkup_["results"]) : Json()},
                        {"after", overall_.isObject() ? pick(overall_["results"]) : Json()},
                        {"gainsStart", gainsStart_},
                        {"gainsEnd", gainsEnd},
                        {"writes", static_cast<double>(tuner_.journal.size()) - static_cast<double>(journalStart_)},
                        {"filters", filters_},
                        {"verdict", overall_.isObject() ? overall_["verdict"] : Json()},
                        {"notes", notes_},
                        {"axes", opts_["axes"]}});
  setPhase("finished", reason);
}

Json FlightSession::snapshot() const {
  const Json& v = link_.vehicle();
  auto slim = [](const Json& r) -> Json {
    if (!r.isObject()) return Json();
    const Json& d = r["diagnostics"];
    const Json& p = r["proposal"];
    Json proposal;
    if (p.isObject())
      proposal = Json::obj({{"changes", p["changes"]},
                            {"margins", p["margins"]},
                            {"overshootPct", p["overshootPct"]},
                            {"predicted", p["predicted"].isObject() ? Json::obj({{"current", p["predicted"]["currentMetrics"]}, {"proposed", p["predicted"]["proposedMetrics"]}}) : Json()}});
    return Json::obj({{"issues", r["issues"].isArray() ? r["issues"] : Json::array()},
                      {"grade", d["grade"].isObject() ? d["grade"] : Json()},
                      {"diagnostics", d.isObject() ? Json::obj({{"trackingPct", d["trackingPct"]}, {"rmsErrorDegs", d["rmsErrorDegs"]}, {"oscRmsDegs", d["oscRmsDegs"]},
                                                               {"saturationPct", d["saturationPct"]}, {"oscillating", d["oscillating"]}})
                                                   : Json()},
                      {"proposal", proposal},
                      {"measuredStep", r["measuredStep"].isObject() ? r["measuredStep"]["metrics"] : Json()},
                      {"error", r["error"]},
                      {"current", r["current"]},
                      {"warnings", r["warnings"].isArray() ? r["warnings"] : Json::array()}});
  };
  auto block = [&](const Json& b) -> Json {
    if (!b.isObject()) return Json();
    Json axes = Json::object();
    for (const auto& [a, r] : b["results"].items()) axes.set(a, slim(r));
    return Json::obj({{"summary", b["summary"]}, {"axes", axes}, {"gains", b["gains"]}, {"vehicleIssues", b["vehicleIssues"].isArray() ? b["vehicleIssues"] : Json::array()},
                      {"verdict", b["verdict"]}, {"vibe", b["vibe"]}, {"hover", b["hover"]}, {"notch", b["notch"]}, {"t", b["t"]}});
  };
  Json phases = Json::array();
  for (const auto& p : kFlightPhases) phases.push(p);
  Json notes = notes_;
  if (notes.size() > 30) notes.arr().erase(notes.arr().begin(), notes.arr().end() - 30);
  return Json::obj({{"phase", phase_},
                    {"phases", phases},
                    {"since", since_},
                    {"info", info_},
                    {"opts", opts_},
                    {"mode", v.isObject() ? v["modeName"] : Json()},
                    {"armed", v.isObject() && v["armed"].asBool()},
                    {"modeOk", modeOk()},
                    {"goodModes", goodModesText()},
                    {"inFlight", inFlight()},
                    {"pausedInfo", pausedInfo_},
                    {"resumePhase", resumePhase_.empty() ? Json() : Json(resumePhase_)},
                    {"checkup", block(checkup_)},
                    {"filters", filters_},
                    {"overall", block(overall_)},
                    {"summary", summary_},
                    {"notes", notes},
                    {"tunerState", tuner_.state},
                    {"tunerInfo", tuner_.stateInfo},
                    {"tunerRound", tuner_.round},
                    {"tunerAxis", tuner_.currentAxis()},
                    {"writes", static_cast<double>(tuner_.journal.size()) - static_cast<double>(journalStart_)}});
}

}  // namespace aptuner::app
