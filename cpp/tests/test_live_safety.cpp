// In-flight safety of the live tuner (2026-10-17 audit), on a fake vehicle behind the real MAVLink link. The same
// scenarios as test/live-safety.test.js:
//   R0 / F-05 in-flight writing needs the opt-in        F-91 no samples before the family is known
//   F-02 continuous all-axis monitor                     F-01 notch change watched and reverted
//   F-26 the watch fails closed                          F-44 evidence-based revert, never above stepMax
//   F-31 / F-92 one write lock, cancel before every PARAM_SET (retries too), no Resume while writing
//   F-03 "oscillating", not "fly sharper"                F-06 attitude-error abort
// The objects of a test are left alive on purpose (timers of the event loop may still point at them).
#include <filesystem>
#include <memory>

#include "aptuner/engine/analysis.hpp"
#include "check.hpp"
#include "common.hpp"
#include "fakevehicle.hpp"
#include "flight.hpp"
#include "liveaxis.hpp"
#include "notchlive.hpp"
#include "tuner.hpp"

namespace aptuner::app {
struct TunerTestAccess {
  static void startRamp(Tuner& t, const std::string& axis) { t.startRamp(axis); }
  static rt::Future<void> rampStep(Tuner& t) { return t.rampStep(); }
};
}  // namespace aptuner::app

namespace {
using namespace aptuner;
using app::Tuner;
using core::Json;
using test::FakeVehicle;

rt::Loop& loop() { return rt::Loop::main(); }
void runFor(double ms) { loop().runUntil([] { return false; }, ms); }
bool waitFor(const std::function<bool()>& pred, double ms) { return loop().runUntil(pred, ms); }
template <class T>
bool settle(const rt::Future<T>& f, double ms = 30000) {
  return loop().runUntil([&] { return f.ready(); }, ms);
}
/** message / code of a settled future's error ("" when it succeeded) */
template <class T>
std::string errorOf(const rt::Future<T>& f) {
  if (!f.ready() || !f.error()) return "";
  try {
    std::rethrow_exception(f.error());
  } catch (const std::exception& e) {
    return e.what();
  }
  return "?";
}
template <class T>
std::string codeOf(const rt::Future<T>& f) {
  if (!f.ready() || !f.error()) return "";
  try {
    std::rethrow_exception(f.error());
  } catch (const rt::Error& e) {
    return e.code();
  } catch (const std::exception&) {
    return "";
  }
  return "";
}
bool has(const std::string& s, const std::string& part) { return s.find(part) != std::string::npos; }
bool threwWith(const std::function<void()>& fn, const std::string& part) {
  try {
    fn();
  } catch (const std::exception& e) {
    return has(e.what(), part);
  }
  return false;
}
std::string sessDir() { return app::pathStr(std::filesystem::temp_directory_path() / "aptuner-cpp-live-tests"); }

struct Rig {
  io::Link* link;
  FakeVehicle* fv;
  Tuner* tuner;
};
// short watch times so the suite runs in about a minute (the defaults are 4 s / 10 s / 1 s)
Rig setup(bool armed, bool inFlight, const Json& opts = Json::object(), std::map<std::string, double> params = test::copterParams(), int type = 2,
          bool prepare = true) {
  auto* link = new io::Link();
  auto* fv = new FakeVehicle(std::move(params), armed, type);
  fv->attach(*link);
  auto* tuner = new Tuner(*link, sessDir());
  tuner->opts.set("inFlightWrites", inFlight);
  const Json fast = Json::obj({{"watchSeconds", 0.6}, {"watchGrace", 0.2}, {"filterWatchSeconds", 1.0}, {"revertStepSeconds", 0.1}, {"calmWaitSeconds", 0.5}});
  for (const auto& [k, v] : fast.items())
    tuner->opts.set(k, v);
  for (const auto& [k, v] : opts.items()) tuner->opts.set(k, v);
  tuner->startMonitor();
  CHECK(waitFor([link] { return link->alive(); }, 3000));
  if (prepare) {
    auto f = tuner->prepare(true).future();
    CHECK(settle(f));
    CHECK(errorOf(f).empty());
    runFor(1100);   // fill the buffers (about 55 samples per axis)
  }
  return {link, fv, tuner};
}
void done(const Rig& r) {
  r.tuner->stopMonitor();
  rt::detach(r.tuner->stop().future());
  r.fv->stop();
  runFor(30);
}
/** a proposal for an axis: the current gains with some keys changed */
void propose(Tuner& t, const std::string& axis, const Json& changes) {
  const Json cur = engine::currentGains(core::Family::Copter, *core::parseAxis(axis), app::toParams(t.params));
  Json gains = cur, list = Json::array();
  for (const auto& [k, v] : changes.items()) {
    gains.set(k, v);
    list.push(Json::obj({{"key", k}, {"from", cur[k]}, {"to", v}, {"pct", 100 * (v.num() / cur[k].num() - 1)}}));
  }
  t.results.set(axis, Json::obj({{"axis", axis}, {"at", rt::Loop::now()}, {"proposal", Json::obj({{"gains", gains}, {"changes", list}})}}));
  t.bindProposal(axis, t.results.at(axis).at("proposal"));   // id + hash, as an analysis does
}
/** the approval the page sends back: the id and hash of the proposal it shows */
Json approve(Tuner& t, const std::string& axis) {
  const Json& p = t.results[axis]["proposal"];
  return Json::obj({{"id", p["id"]}, {"hash", p["hash"]}});
}
/** largest ratio between consecutive values written to the same parameter (and from its value before) */
double maxWriteRatio(const FakeVehicle& fv, std::map<std::string, double> last) {
  double worst = 1;
  for (const auto& s : fv.sets) {
    const auto it = last.find(s.name);
    if (it != last.end() && it->second > 0 && s.value > 0) worst = std::max(worst, it->second > s.value ? it->second / s.value : s.value / it->second);
    last[s.name] = s.value;
  }
  return worst;
}
bool apply(Rig& r, const std::string& axis) {
  auto f = r.tuner->applyProposal(axis, approve(*r.tuner, axis)).future();
  CHECK(settle(f));
  CHECK(errorOf(f).empty());
  return f.ready() && !f.error() && f.get();
}
bool messageHas(const Tuner& t, const std::string& part) {
  for (const auto& m : t.messages.arr())
    if (has(m["text"].strOr(""), part)) return true;
  return false;
}
}  // namespace

// ---------------- R0: in-flight writing is off by default

TEST(live_r0_default_off) {
  auto* link = new io::Link();
  auto* t = new Tuner(*link, sessDir());
  CHECK(!t->opts["inFlightWrites"].asBool());
}

TEST(live_r0_armed_without_optin_writes_nothing) {
  Rig r = setup(true, false);
  const std::size_t n0 = r.fv->sets.size();
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  auto a = r.tuner->applyProposal("roll", approve(*r.tuner, "roll")).future();
  settle(a);
  CHECK(errorOf(a) == app::kInFlightOff);   // Advanced Apply while armed
  auto b = r.tuner->applyFilterChange(Json::obj({{"INS_HNTCH_ENABLE", 1}}), "test").future();
  settle(b);
  CHECK(has(errorOf(b), "changes in flight are turned off"));   // notch / filter write while armed
  auto* notch = new app::NotchLive(*r.link, *r.tuner);
  r.fv->escRpm = 6000;
  runFor(600);
  auto c = notch->applyEsc().future();
  settle(c);
  CHECK(has(errorOf(c), "changes in flight are turned off"));   // live notch write while armed
  auto d = r.tuner->restoreOriginals().future();
  settle(d);
  CHECK(has(errorOf(d), "changes in flight are turned off"));   // restore while armed
  r.tuner->opts.set("autoApply", true);
  CHECK(threwWith([&] { r.tuner->start(); }, "changes in flight are turned off"));   // auto-apply session while armed
  r.tuner->opts.set("autoApply", false);
  // the guided flight: "Apply by itself" is refused, and at the review step nothing is written
  auto* flight = new app::FlightSession(*r.link, *r.tuner, *notch);
  CHECK(threwWith([&] { flight->start(Json::obj({{"auto", true}})); }, "Allow changes in flight"));
  flight->setPhaseForTest("review", Json::obj({{"summary", Json::obj({{"anyProposal", true}, {"changes", 1}})}, {"results", Json::object()}}));
  flight->cont();
  CHECK(flight->phase() == "finished");
  runFor(300);
  CHECK(r.fv->sets.size() == n0);   // no PARAM_SET was sent
  done(r);
}

TEST(live_r0_housekeeping_while_armed_needs_optin) {
  auto p = test::copterParams();
  p["GCS_PID_MASK"] = 0;
  Rig r = setup(true, false, Json::object(), p, 2, false);
  auto f = r.tuner->prepare(true).future();
  settle(f);
  CHECK(has(errorOf(f), "changes in flight are turned off"));
  CHECK(r.fv->sets.empty());
  CHECK(r.tuner->state != "preparing");
  done(r);
}

TEST(live_r0_on_the_ground_no_optin_needed) {
  Rig r = setup(false, false);
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  CHECK(apply(r, "roll"));
  CHECK_NEAR(r.fv->params["ATC_RAT_RLL_P"], 0.16, 1e-6);
  done(r);
}

TEST(live_r0_optin_allows_inflight_apply) {
  Rig r = setup(true, true);
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  CHECK(apply(r, "roll"));
  CHECK_NEAR(r.fv->params["ATC_RAT_RLL_P"], 0.16, 1e-6);
  CHECK(!r.tuner->changes.empty() && r.tuner->changes.back()->status == "held" && r.tuner->changes.back()->armed);
  done(r);
}

// ---------------- F-91: nothing stored before the family is known

TEST(live_f91_plane_samples_before_prepare_not_stored) {
  Rig r = setup(true, true, Json::object(), test::copterParams(), 1, false);
  r.fv->desiredAmp = 30;   // a plane reports deg/s
  runFor(600);
  CHECK(r.tuner->axes["roll"].t.empty());   // PID_TUNING before prepare() is dropped
  auto f = r.tuner->prepare(true).future();
  settle(f);
  CHECK(errorOf(f).empty());
  CHECK(r.tuner->family == core::Family::Plane);
  const double t0 = rt::Loop::now() / 1000;
  runFor(600);
  const app::LiveAxis& ax = r.tuner->axes["roll"];
  CHECK(ax.t.size() > 10);
  CHECK(!ax.t.empty() && ax.t[0] >= t0 - 1.5);   // only samples from after the family was known
  double peak = 0;
  for (double v : ax.target) peak = std::max(peak, std::fabs(v));
  CHECK(peak <= 31 && peak > 5);   // plane rates kept in deg/s, not x57.3
  done(r);
}

TEST(live_f91_prepare_clears_buffers) {
  Rig r = setup(false, true);
  r.tuner->axes["roll"].push(rt::Loop::now() / 1000 - 100, Json::obj({{"desired", 1e4}, {"achieved", 1e4}, {"P", 0}, {"I", 0}, {"D", 0}, {"FF", 0}, {"SRate", 0.5}, {"PDmod", 1}}));
  auto f = r.tuner->prepare(true).future();
  settle(f);
  bool old = false;
  for (double v : r.tuner->axes["roll"].target) old = old || v == 1e4;
  CHECK(!old);
  done(r);
}

// ---------------- F-02: the continuous monitor

TEST(live_f02_monitor_catches_non_tuned_axis_within_3s) {
  Rig r = setup(true, true);
  r.tuner->opts.set("axes", Json(Json::Array{"roll"}));
  r.tuner->start();
  runFor(300);
  const double t0 = rt::Loop::now();
  r.fv->slew["yaw"] = 20;   // yaw is not being tuned
  CHECK(waitFor([&] { return r.tuner->state == "paused"; }, 3000));
  CHECK(rt::Loop::now() - t0 < 3000);
  CHECK(has(r.tuner->stateInfo, "yaw oscillating"));
  CHECK(waitFor([&] { return r.tuner->writing().empty(); }, 3000));
  CHECK(has(r.tuner->stateInfo, "nothing to go back to. Land now"));
  CHECK(r.fv->sets.empty());   // nothing written: the app had changed nothing
  done(r);
}

TEST(live_f02_monitor_without_optin_only_tells_to_land) {
  Rig r = setup(true, false);
  r.tuner->start();
  runFor(300);
  r.fv->slew["pitch"] = 20;
  CHECK(waitFor([&] { return r.tuner->state == "paused" && r.tuner->writing().empty() && has(r.tuner->stateInfo, "changed nothing in the air"); }, 3500));
  CHECK(r.fv->sets.empty());
  done(r);
}

TEST(live_f02_no_trip_on_calm_or_short_bursts) {
  Rig r = setup(true, true);
  r.tuner->start();
  runFor(200);
  // a 0.3 s burst (a sharp stick tap) is not a sustained oscillation
  r.fv->slew["roll"] = 8;
  runFor(300);
  r.fv->slew["roll"] = 0.5;
  runFor(2500);
  CHECK(r.tuner->state == "observing");
  done(r);
}

// ---------------- F-01: the in-flight notch change

TEST(live_f01_notch_change_reverted_and_paused_within_5s) {
  // INS_HNTCH_REF already 1 (set earlier on the ground): in flight only values that act at once are written (F-35)
  auto p = test::copterParams();
  p["INS_HNTCH_REF"] = 1;
  Rig r = setup(true, true, Json::object(), p);
  auto* notch = new app::NotchLive(*r.link, *r.tuner);
  r.fv->escRpm = 6000;
  runFor(500);
  r.tuner->start();
  runFor(200);
  auto onset = std::make_shared<double>(0);
  FakeVehicle* fv = r.fv;
  // the airframe: 0.3 s after the notch comes on, pitch starts to oscillate; it stops when the notch is off again
  fv->onSet = [fv, onset](const std::string& name, double v) {
    if (name != "INS_HNTCH_ENABLE") return;
    if (v > 0)
      rt::Loop::main().setTimeout(300, [fv, onset] {
        if (fv->params["INS_HNTCH_ENABLE"] > 0) {
          fv->slew["pitch"] = 25;
          fv->dmod["pitch"] = 0.75;
          *onset = rt::Loop::now();
        }
      });
    else {
      fv->slew["pitch"] = 0.5;
      fv->dmod["pitch"] = 1;
    }
  };
  const auto before = fv->params;
  auto f = notch->applyEsc().future();
  CHECK(settle(f));
  CHECK(codeOf(f) == "reverted");   // applyEsc reports the revert
  const auto on = fv->setsOf("INS_HNTCH_ENABLE");
  CHECK(on.size() >= 2);
  if (on.size() >= 2) {
    CHECK(on[0].value == 1 && on[1].value == 0);   // the notch was switched off again
    CHECK(*onset > 0 && on[1].t - *onset < 5000);
    for (const auto& s : fv->sets)
      if (s.t >= on[1].t) {
        CHECK(s.name == "INS_HNTCH_ENABLE");   // the notch switch is the first corrective write
        break;
      }
  }
  for (const char* k : {"INS_HNTCH_ENABLE", "INS_HNTCH_MODE", "INS_HNTCH_FREQ", "INS_HNTCH_BW", "INS_HNTCH_REF"}) CHECK(fv->params[k] == before.at(k));
  CHECK(r.tuner->state == "paused");
  CHECK(has(r.tuner->stateInfo, "the change was reverted") && has(r.tuner->stateInfo, "calm again"));
  bool pausedSoon = false;
  for (const auto& m : r.tuner->messages.arr())
    if (m["kind"].strOr("") == "tuner" && m["text"].strOr("").rfind("paused: ", 0) == 0 && has(m["text"].strOr(""), "oscillation detected, reverting") &&
        m["t"].numOr(0) - *onset < 5000)
      pausedSoon = true;
  CHECK(pausedSoon);   // paused within 5 s of the onset
  done(r);
}

// ---------------- F-26: the watch fails closed

TEST(live_f26_no_data_after_change_reverts) {
  Rig r = setup(true, true);
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  FakeVehicle* fv = r.fv;
  fv->onSet = [fv](const std::string& name, double) {
    if (name == "ATC_RAT_RLL_I" && fv->params["ATC_RAT_RLL_I"] > 0.15) {
      fv->streaming = false;
      rt::Loop::main().setTimeout(1600, [fv] { fv->streaming = true; });
    }
  };
  CHECK(!apply(r, "roll"));   // the change is not kept without data
  CHECK_NEAR(fv->params["ATC_RAT_RLL_P"], 0.135, 1e-6);
  CHECK_NEAR(fv->params["ATC_RAT_RLL_I"], 0.135, 1e-6);
  CHECK(has(r.tuner->stateInfo, "no PID data"));
  done(r);
}

TEST(live_f26_sparse_data_after_change_reverts) {
  Rig r = setup(true, true);
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  FakeVehicle* fv = r.fv;
  fv->onSet = [fv](const std::string& name, double) {
    if (name == "ATC_RAT_RLL_I" && fv->params["ATC_RAT_RLL_I"] > 0.15) fv->rateHz = 4;
  };
  CHECK(!apply(r, "roll"));
  CHECK_NEAR(fv->params["ATC_RAT_RLL_P"], 0.135, 1e-6);
  CHECK(has(r.tuner->stateInfo, "PID samples after the change"));
  done(r);
}

TEST(live_f26_link_lost_during_watch_fails_closed) {
  Rig r = setup(true, true);
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  FakeVehicle* fv = r.fv;
  io::Link* link = r.link;
  fv->onSet = [fv, link](const std::string& name, double) {
    if (name == "ATC_RAT_RLL_I" && fv->params["ATC_RAT_RLL_I"] > 0.15) rt::Loop::main().setTimeout(300, [link] { link->disconnect(); });
  };
  CHECK(!apply(r, "roll"));
  CHECK(r.tuner->state == "paused");
  CHECK(messageHas(*r.tuner, "link to the vehicle was lost during the watch"));
  CHECK(has(r.tuner->stateInfo, "land now"));
  done(r);
}

// ---------------- F-44: evidence-based revert

TEST(live_f44_calm_before_step_reverts_to_pre_step_set) {
  Rig r = setup(true, true);
  propose(*r.tuner, "pitch", Json::obj({{"P", 0.17}, {"I", 0.17}}));
  FakeVehicle* fv = r.fv;
  fv->onSet = [fv](const std::string& name, double v) {
    if (name == "ATC_RAT_PIT_P") fv->slew["pitch"] = v > 0.15 ? 9 : 0.5;
  };
  CHECK(!apply(r, "pitch"));
  CHECK_NEAR(fv->params["ATC_RAT_PIT_P"], 0.135, 1e-6);   // back to the set before the step
  CHECK(has(r.tuner->stateInfo, "the change was reverted") && has(r.tuner->stateInfo, "calm again"));
  done(r);
}

TEST(live_f44_lowering_step_reverted_upward_on_evidence) {
  // the cd07d42 rule kept lowering after a trip; with the 2 s before the step calm, the step itself is undone
  Rig r = setup(true, true);
  propose(*r.tuner, "roll", Json::obj({{"D", 0.0025}}));
  FakeVehicle* fv = r.fv;
  fv->onSet = [fv](const std::string& name, double v) {
    if (name == "ATC_RAT_RLL_D") fv->slew["roll"] = v < 0.003 ? 7 : 0.5;
  };
  CHECK(!apply(r, "roll"));
  CHECK_NEAR(fv->params["ATC_RAT_RLL_D"], 0.0036, 1e-7);   // D back up (the calm set before the step)
  CHECK(maxWriteRatio(*fv, test::copterParams()) <= r.tuner->opts["stepMax"].num() * (1 + 1e-6));
  done(r);
}

TEST(live_f44_still_oscillating_restores_session_start_in_steps) {
  Rig r = setup(true, true);
  r.tuner->beginSession();   // session start: D 0.0036
  // an earlier in-flight apply raised roll D to 0.0072 (held), then the vehicle starts oscillating and keeps on
  propose(*r.tuner, "roll", Json::obj({{"D", 0.0072}}));
  CHECK(apply(r, "roll"));
  const std::size_t n0 = r.fv->sets.size();
  r.tuner->resume();
  runFor(100);
  r.fv->slew["roll"] = 7;   // persistent oscillation, not caused by any write the monitor can blame
  CHECK(waitFor([&] { return r.tuner->state == "paused" && r.tuner->writing().empty(); }, 8000));
  int writes = 0;
  for (std::size_t i = n0; i < r.fv->sets.size(); i++)
    if (r.fv->sets[i].name == "ATC_RAT_RLL_D") writes++;
  CHECK(writes >= 2);   // restored in steps
  CHECK_NEAR(r.fv->params["ATC_RAT_RLL_D"], 0.0036, 1e-7);   // session start value restored
  CHECK(maxWriteRatio(*r.fv, test::copterParams()) <= r.tuner->opts["stepMax"].num() * (1 + 1e-6));
  CHECK(has(r.tuner->stateInfo, "still oscillating with the values from the start of the session"));
  done(r);
}

TEST(live_f44_revert_fails_then_session_start_restored) {
  Rig r = setup(true, true);
  r.tuner->beginSession();
  propose(*r.tuner, "pitch", Json::obj({{"P", 0.17}, {"I", 0.17}}));
  CHECK(apply(r, "pitch"));
  r.tuner->resume();
  propose(*r.tuner, "pitch", Json::obj({{"P", 0.21}, {"I", 0.21}}));
  // this step starts an oscillation that a revert of the step alone does not stop (only P <= 0.15 is calm)
  FakeVehicle* fv = r.fv;
  fv->onSet = [fv](const std::string& name, double v) {
    if (name == "ATC_RAT_PIT_P") fv->slew["pitch"] = v > 0.15 ? 9 : 0.5;
  };
  CHECK(!apply(r, "pitch"));
  CHECK_NEAR(fv->params["ATC_RAT_PIT_P"], 0.135, 1e-6);   // escalated to the session start values
  CHECK(maxWriteRatio(*fv, test::copterParams()) <= r.tuner->opts["stepMax"].num() * (1 + 1e-6));
  CHECK(has(r.tuner->stateInfo, "values from the start of the session were restored") && has(r.tuner->stateInfo, "calm again"));
  done(r);
}

// ---------------- F-31 / F-92: one write lock, cancellation

TEST(live_f92_second_apply_refused_and_resume_refused) {
  Rig r = setup(true, true);
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  propose(*r.tuner, "pitch", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  auto a = r.tuner->applyProposal("roll", approve(*r.tuner, "roll")).future();
  runFor(50);
  auto b = r.tuner->applyProposal("pitch", approve(*r.tuner, "pitch")).future();
  settle(b);
  CHECK(has(errorOf(b), "another change is being written"));
  CHECK(threwWith([&] { r.tuner->resume(); }, "still being written"));
  CHECK(threwWith([&] { r.tuner->start(); }, "being written"));
  CHECK(settle(a) && errorOf(a).empty() && a.get());
  CHECK(r.fv->setsOf("ATC_RAT_PIT_P").empty());   // the second apply wrote nothing
  done(r);
}

TEST(live_f31_pause_stops_before_the_next_write) {
  Rig r = setup(true, true);
  propose(*r.tuner, "roll", Json::obj({{"P", 0.27}, {"I", 0.27}, {"D", 0.0072}}));   // x2: two steps
  auto paused = std::make_shared<double>(0);
  Tuner* t = r.tuner;
  r.fv->onSet = [t, paused](const std::string& name, double) {
    if (name == "ATC_RAT_RLL_I" && *paused == 0)
      rt::Loop::main().setTimeout(250, [t, paused] {
        if (*paused != 0) return;
        t->pause("paused by user");
        *paused = rt::Loop::now();
      });
  };
  CHECK(!apply(r, "roll"));   // the apply stops
  CHECK(*paused > 0);         // paused during the watch of step 1
  for (const auto& s : r.fv->sets) CHECK(s.t <= *paused);   // no PARAM_SET after the pause
  CHECK(r.tuner->state == "paused" && r.tuner->stateInfo == "paused by user");
  CHECK(std::fabs(r.fv->params["ATC_RAT_RLL_P"] - 0.27) > 0.05);   // the last step was not written
  done(r);
}

TEST(live_f31_pause_cancels_param_set_retries) {
  Rig r = setup(true, true);
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  r.fv->ignoreSets = true;   // the vehicle does not answer: the link would retry every 1.5 s
  auto a = r.tuner->applyProposal("roll", approve(*r.tuner, "roll")).future();
  runFor(300);
  r.tuner->pause("paused by user");
  CHECK(settle(a) && errorOf(a).empty() && !a.get());
  runFor(2000);
  CHECK(r.fv->setsOf("ATC_RAT_RLL_P").size() == 1);   // one PARAM_SET, no retry after the pause
  bool journaled = false;
  for (const auto& j : r.tuner->journal.arr())
    if (j["name"].strOr("") == "ATC_RAT_RLL_P" && has(j["status"].strOr(""), "cancelled")) journaled = true;
  CHECK(journaled);   // the journal says the write was cancelled (and what was read back)
  done(r);
}

TEST(live_f31_single_loop_after_pause_resume) {
  Rig r = setup(true, true, Json::obj({{"observeSeconds", 600}}));
  auto calls = std::make_shared<int>(0);
  r.tuner->on("observe", [calls](const Json&) { (*calls)++; });
  r.tuner->start();
  runFor(1200);
  r.tuner->pause("");
  r.tuner->resume();
  r.tuner->pause("");
  r.tuner->resume();
  const int c0 = *calls;
  runFor(3200);
  CHECK(*calls - c0 <= 4);   // one observe loop
  done(r);
}

// ---------------- F-03: the message names the oscillation

TEST(live_f03_low_coherence_with_high_srate_says_oscillating) {
  Rig r = setup(true, true, Json::obj({{"observeSeconds", 3}}));
  r.fv->desiredAmp = 0;   // no stick input: no model can be fitted
  // every 8th sample above the limit: not sustained (the monitor stays quiet), but clearly not calm
  auto k = std::make_shared<int>(0);
  FakeVehicle* fv = r.fv;
  const auto iv = loop().setInterval(20, [fv, k] { (*k)++; fv->slew["roll"] = *k % 8 == 0 ? 9 : 0.5; });
  r.tuner->opts.set("axes", Json(Json::Array{"roll"}));
  r.tuner->start();
  CHECK(waitFor([&] { return r.tuner->state == "paused"; }, 8000));
  loop().clear(iv);
  CHECK(has(r.tuner->stateInfo, "roll oscillating"));
  CHECK(!has(r.tuner->stateInfo, "fly sharper"));   // never "fly sharper" while it oscillates
  done(r);
}

// ---------------- F-06: attitude-error abort

TEST(live_f06_attitude_error_during_watch_reverts) {
  Rig r = setup(true, true, Json::obj({{"watchSeconds", 1.5}}));
  r.fv->attitude = FakeVehicle::Att{0, 0};
  r.fv->navTarget = FakeVehicle::Att{0, 0};
  runFor(500);
  propose(*r.tuner, "pitch", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  FakeVehicle* fv = r.fv;
  fv->onSet = [fv](const std::string& name, double v) {
    if (name == "ATC_RAT_PIT_P") fv->attitude->pitch = v > 0.15 ? 15 : 0;   // 15 deg off target, slew rate calm
  };
  CHECK(!apply(r, "pitch"));
  CHECK(messageHas(*r.tuner, "attitude error 15.0 deg"));   // the attitude error tripped the watch
  CHECK_NEAR(fv->params["ATC_RAT_PIT_P"], 0.135, 1e-6);    // reverted
  done(r);
}

// ---------------- the QuikTune-style ramp writes in flight too

TEST(live_ramp_needs_optin_while_armed) {
  Rig r = setup(true, false);
  r.tuner->params.set("ATC_RAT_RLL_FLTD", 0);   // an unset D filter: the QuikTune filter rule has to write it first
  app::TunerTestAccess::startRamp(*r.tuner, "roll");
  auto f = app::TunerTestAccess::rampStep(*r.tuner);
  settle(f);
  CHECK(has(errorOf(f), "changes in flight are turned off"));
  CHECK(r.fv->sets.empty());
  done(r);
}

TEST(live_ramp_fails_closed_without_data) {
  Rig r = setup(true, true);
  r.tuner->params.set("ATC_RAT_RLL_FLTD", 0);
  r.fv->streaming = false;   // no PID data: the ramp could not see an oscillation
  runFor(1200);
  app::TunerTestAccess::startRamp(*r.tuner, "roll");
  auto f = app::TunerTestAccess::rampStep(*r.tuner);
  settle(f);
  CHECK(errorOf(f).empty());
  CHECK(r.tuner->state == "paused");
  CHECK(has(r.tuner->stateInfo, "the ramp stopped"));
  CHECK(r.fv->sets.empty());   // nothing written blind
  done(r);
}

// ---------------- LiveAxis evidence

TEST(live_slewstats_missing_extension_fields_not_bad) {
  app::LiveAxis ax("roll");
  for (int i = 0; i < 100; i++)
    ax.push(1000 + i / 50.0, Json::obj({{"desired", 0}, {"achieved", 0}, {"P", 0}, {"I", 0}, {"D", 0}, {"FF", 0}, {"SRate", 0}, {"PDmod", 0}}));
  const app::SlewStats s = ax.slewStats(2);
  CHECK(s.bad == 0);   // Dmod 0 = fields missing, not a trip
  CHECK(s.noSlew == s.n);
  CHECK(!ax.watchdog(2).oscillating);   // the step watchdog ignores missing Dmod too
}
