// Verified parameter writes bound to the right aircraft (2026-10-17 audit, roadmap R3), on fake vehicles behind the real
// MAVLink link. The same scenarios as test/write-verify.test.js:
//   F-25 session identity, a reconnect / another vehicle drops proposals    F-28 the target is pinned, ACKs bound
//   F-29 / F-30 exact verification by type                                  F-32 transactions, verified rollback
//   F-33 approval bound to the proposal id + hash, IMAX written             F-35 reboot-only values, reboot workflow
//   F-39 the append-only journal                                            F-43 .param export checks
// wv_sitl_ground_write runs the ground write on ArduCopter SITL (tools/sitl-ground-write.js --cpp); skipped otherwise.
// The objects of a test are left alive on purpose (timers of the event loop may still point at them).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>

#include "aptuner/core/jsnum.hpp"
#include "aptuner/core/parammeta.hpp"
#include "aptuner/engine/analysis.hpp"
#include "aptuner/engine/identity.hpp"
#include "aptuner/engine/report.hpp"
#include "check.hpp"
#include "common.hpp"
#include "fakevehicle.hpp"
#include "groundtune.hpp"
#include "journal.hpp"
#include "notchlive.hpp"
#include "paramwrite.hpp"
#include "tuner.hpp"

namespace aptuner::app {
struct GroundTestAccess {
  static rt::Future<void> review(GroundTune& g) { return g.review(); }
  static void setPhase(GroundTune& g, const std::string& p, const std::string& info) { g.setPhase(p, info); }
  static void setRows(GroundTune& g, Json rows) { g.rows_ = std::move(rows); }
};
}  // namespace aptuner::app

namespace {
using namespace aptuner;
using app::GroundTestAccess;
using app::Tuner;
using core::Json;
using test::Bus;
using test::f32;
using test::FakeVehicle;

rt::Loop& loop() { return rt::Loop::main(); }
void runFor(double ms) { loop().runUntil([] { return false; }, ms); }
bool waitFor(const std::function<bool()>& pred, double ms) { return loop().runUntil(pred, ms); }
template <class T>
bool settle(const rt::Future<T>& f, double ms = 40000) {
  return loop().runUntil([&] { return f.ready(); }, ms);
}
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
template <class T>
T valueOf(const rt::Future<T>& f) {
  CHECK(settle(f));
  CHECK(errorOf(f).empty());
  return f.ready() && !f.error() ? f.get() : T();
}
void valueOf(const rt::Future<void>& f) {
  CHECK(settle(f));
  CHECK(errorOf(f).empty());
}
bool has(const std::string& s, const std::string& part) { return s.find(part) != std::string::npos; }
bool matches(const std::string& s, const std::string& re) { return std::regex_search(s, std::regex(re)); }
std::string sessDir() { return app::pathStr(std::filesystem::temp_directory_path() / "aptuner-cpp-wv-tests"); }
int nJournal = 0;
app::Journal* newJournal() { return new app::Journal(sessDir() + "/journal-" + std::to_string(++nJournal) + "-" + std::to_string(static_cast<long long>(rt::Loop::now()))); }
/** the journal file as events */
std::vector<Json> events(const app::Journal& j) {
  std::vector<Json> out;
  if (j.file().empty()) return out;
  std::istringstream in(app::readText(j.file()));
  std::string line;
  while (std::getline(in, line))
    if (!line.empty()) out.push_back(Json::parse(line));
  return out;
}
std::vector<Json> eventsOf(const app::Journal& j, const std::string& kind) {
  std::vector<Json> out;
  for (const auto& e : events(j))
    if (e["kind"].strOr("") == kind) out.push_back(e);
  return out;
}

struct Rig {
  io::Link* link;
  FakeVehicle* fv;
  Tuner* tuner;
  app::Journal* journal;
};
Rig setup(bool armed = false, bool inFlight = false, std::map<std::string, double> params = test::copterParams(), bool prepare = true) {
  auto* link = new io::Link();
  auto* fv = new FakeVehicle(std::move(params), armed);
  fv->attach(*link);
  auto* journal = newJournal();
  auto* tuner = new Tuner(*link, sessDir(), journal);
  tuner->opts.set("inFlightWrites", inFlight);
  const Json fast = Json::obj({{"watchSeconds", 0.6}, {"watchGrace", 0.2}, {"filterWatchSeconds", 1.0}, {"revertStepSeconds", 0.1}, {"calmWaitSeconds", 0.5}});
  for (const auto& [k, v] : fast.items())
    tuner->opts.set(k, v);
  tuner->startMonitor();
  CHECK(waitFor([link] { return link->alive(); }, 3000));
  if (prepare) {
    auto f = tuner->prepare(true).future();
    CHECK(settle(f));
    CHECK(errorOf(f).empty());
    runFor(armed ? 1100 : 50);
  }
  return {link, fv, tuner, journal};
}
void done(const Rig& r) {
  r.tuner->stopMonitor();
  rt::detach(r.tuner->stop().future());
  r.fv->stop();
  runFor(30);
}
/** a proposal for an axis: the current gains with some keys changed, bound like an analysis binds it */
Json& propose(Tuner& t, const std::string& axis, const Json& changes) {
  const Json cur = engine::currentGains(core::Family::Copter, *core::parseAxis(axis), app::toParams(t.params));
  Json gains = cur, list = Json::array();
  for (const auto& [k, v] : changes.items()) {
    gains.set(k, v);
    list.push(Json::obj({{"key", k}, {"from", cur[k]}, {"to", v}, {"pct", 100 * (v.num() / cur[k].num() - 1)}}));
  }
  t.results.set(axis, Json::obj({{"axis", axis}, {"at", rt::Loop::now()}, {"proposal", Json::obj({{"gains", gains}, {"changes", list}})}}));
  return t.bindProposal(axis, t.results.at(axis).at("proposal"));
}
Json approve(Tuner& t, const std::string& axis) {
  const Json& p = t.results[axis]["proposal"];
  return Json::obj({{"id", p["id"]}, {"hash", p["hash"]}});
}
rt::Future<bool> applyF(Rig& r, const std::string& axis, const Json& approval) { return r.tuner->applyProposal(axis, approval).future(); }
double maxWriteRatio(const std::vector<FakeVehicle::Set>& sets, std::map<std::string, double> last) {
  double worst = 1;
  for (const auto& s : sets) {
    const auto it = last.find(s.name);
    if (it != last.end() && it->second > 0 && s.value > 0) worst = std::max(worst, it->second > s.value ? it->second / s.value : s.value / it->second);
    last[s.name] = s.value;
  }
  return worst;
}
app::VerifyOpts quickV() {
  app::VerifyOpts o;
  o.timeoutMs = 300;
  o.retries = 1;
  o.readRetries = 1;
  return o;
}
app::TxnOpts quickT() {
  app::TxnOpts o;
  o.timeoutMs = 300;
  o.retries = 1;
  o.readRetries = 1;
  return o;
}
Json items(std::initializer_list<std::pair<const char*, double>> l) {
  Json a = Json::array();
  for (const auto& [n, v] : l) a.push(Json::obj({{"name", n}, {"to", v}}));
  return a;
}
std::string statuses(const Json& rows) {
  std::string s;
  for (const auto& r : rows.arr()) s += (s.empty() ? "" : ",") + r["status"].strOr("");
  return s;
}

struct GRig {
  io::Link* link;
  FakeVehicle* fv;
  app::GroundTune* g;
  app::Journal* journal;
};
GRig groundSetup(std::map<std::string, double> params = test::copterParams()) {
  auto* link = new io::Link();
  auto* fv = new FakeVehicle(std::move(params), false);
  fv->attach(*link);
  auto* journal = newJournal();
  auto* g = new app::GroundTune(*link, sessDir() + "/g" + std::to_string(nJournal) + "-" + std::to_string(static_cast<long long>(rt::Loop::now())), journal);
  CHECK(waitFor([link] { return link->alive(); }, 3000));
  auto w = link->waitAutopilot(2000).future();
  settle(w);
  return {link, fv, g, journal};
}
/** the Telemetry + USB review, as after a log download: the log's values and the proposed rows */
Json review(app::GroundTune& g, const Json& rows, const Json& logParams = Json::object(), const std::string& firmware = "ArduCopter V4.5.7 (abcdef12)") {
  Json params = Json::obj({{"SYSID_THISMAV", 1}});
  for (const auto& [k, v] : logParams.items()) {
    if (v.isNull())
      params.erase(k);
    else
      params.set(k, v);
  }
  g.result = Json::obj({{"family", "copter"}, {"firmware", firmware}, {"params", params}, {"axes", Json::object()}, {"chosenAxes", Json::array()}});
  Json rs = Json::array();
  for (const auto& r : rows.arr()) {
    Json x = Json::obj({{"group", "roll"}, {"label", r["name"]}});
    for (const auto& [k, v] : r.items()) x.set(k, v);
    rs.push(x);
  }
  GroundTestAccess::setRows(g, rs);
  auto f = GroundTestAccess::review(g);
  CHECK(settle(f));
  CHECK(errorOf(f).empty());
  GroundTestAccess::setPhase(g, "review", "test review");
  return Json::obj({{"id", g.proposal()["id"]}, {"hash", g.proposal()["hash"]}});
}
Json rollRows() {
  return Json(Json::Array{Json::obj({{"name", "ATC_RAT_RLL_P"}, {"value", 0.16}, {"from", f32(0.135)}}), Json::obj({{"name", "ATC_RAT_RLL_I"}, {"value", 0.16}, {"from", f32(0.135)}}),
                          Json::obj({{"name", "ATC_RAT_RLL_D"}, {"value", 0.0045}, {"from", f32(0.0036)}})});
}
std::vector<std::string> rollNames() { return {"ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D"}; }
std::string fieldList(const Json& rows, const char* key) {
  std::string s;
  for (const auto& r : rows.arr()) s += (s.empty() ? "" : ",") + engine::jsText(r[key]);
  return s;
}
}  // namespace

// ---------------- F-28: the target is pinned to one ArduPilot vehicle

TEST(wv_pin_ignores_tracker_gcs_and_companion) {
  auto* link = new io::Link();
  auto* bus = new Bus();
  auto* tracker = new FakeVehicle(test::copterParams(), false, 5);   // ArduPilot AntennaTracker
  tracker->sysid = 2;
  tracker->attach(*link, bus);
  mavlink::Encoder gcs(255, 190), comp(1, 191);
  link->feed(gcs.pack("HEARTBEAT", Json::obj({{"type", 6}, {"autopilot", 8}, {"base_mode", 0}, {"custom_mode", 0}, {"system_status", 4}, {"mavlink_version", 3}})));
  link->feed(comp.pack("HEARTBEAT", Json::obj({{"type", 18}, {"autopilot", 8}, {"base_mode", 0}, {"custom_mode", 0}, {"system_status", 4}, {"mavlink_version", 3}})));
  CHECK(!link->pinned());
  auto* fv = new FakeVehicle(test::copterParams(), false);
  fv->sysid = 3;
  fv->attach(*link, bus);
  CHECK(waitFor([link] { return link->alive(); }, 2000));
  CHECK(link->targetSys() == 3 && link->targetComp() == 1);
  CHECK(link->vehicle()["type"].numOr(0) == 2);
  runFor(300);
  CHECK(link->stats().foreign > 0);
  CHECK(valueOf(link->requestParam("ATC_RAT_RLL_P")) == f32(0.135));
  CHECK(tracker->reads.empty());   // requests go to system id 3 only
  tracker->silence();
  fv->stop();
  runFor(30);
}

TEST(wv_second_vehicle_reported_not_followed) {
  auto* link = new io::Link();
  auto* bus = new Bus();
  auto* a = new FakeVehicle(test::copterParams(), false);
  a->attach(*link, bus);
  CHECK(waitFor([link] { return link->alive(); }, 2000));
  auto statuses = std::make_shared<std::vector<std::string>>();
  link->on("status", [statuses](const Json& s) { statuses->push_back(s.strOr("")); });
  auto* b = new FakeVehicle(test::copterParams(), false);
  b->sysid = 2;
  b->uid = "ffffffffffffffffffffffff";
  b->attach(*link, bus);
  b->desiredAmp = 5;
  b->slew["roll"] = 9;
  auto* tuner = new Tuner(*link, sessDir(), newJournal());
  valueOf(tuner->prepare(true).future());
  runFor(1200);
  CHECK(link->targetSys() == 1);
  bool seen = false;
  const Json others = link->othersSeen();
  for (const auto& o : others.arr())
    if (o["sysid"].numOr(0) == 2) seen = true;
  CHECK(seen);
  bool told = false;
  for (const auto& s : *statuses)
    if (has(s, "another vehicle is on this link (system id 2, MAV_TYPE_QUADROTOR): the app stays with system id 1")) told = true;
  CHECK(told);
  auto p = link->requestParam("ATC_RAT_RLL_P");
  b->foreignSet("ATC_RAT_RLL_P", 0.9);
  CHECK(valueOf(p) == f32(0.135));
  double peak = 0;
  for (const double v : tuner->axes["roll"].target) peak = std::max(peak, std::fabs(v));
  CHECK(peak < 12);
  CHECK(tuner->axes["roll"].slewStats(1).srateMax <= 0.5);
  CHECK(tuner->identity["uid"].strOr("") == a->uid + "000000000000");
  tuner->stopMonitor();
  rt::detach(tuner->stop().future());
  a->silence();
  b->stop();
  runFor(30);
}

TEST(wv_target_silent_other_vehicle_is_a_new_connection) {
  auto* link = new io::Link();
  auto* bus = new Bus();
  auto* a = new FakeVehicle(test::copterParams(), false);
  a->attach(*link, bus);
  CHECK(waitFor([link] { return link->alive(); }, 2000));
  auto* journal = newJournal();
  auto* tuner = new Tuner(*link, sessDir(), journal);
  auto* b = new FakeVehicle(test::copterParams(), false);
  b->sysid = 2;
  b->uid = "eeeeeeeeeeeeeeeeeeeeeeee";
  b->attach(*link, bus);
  valueOf(tuner->prepare(true).future());
  propose(*tuner, "roll", Json::obj({{"P", 0.16}}));
  const Json ap = approve(*tuner, "roll");
  const int gen0 = link->generation();
  auto changed = std::make_shared<bool>(false);
  link->on("vehicle-change", [changed](const Json&) { *changed = true; });
  a->heartbeats = false;
  a->streaming = false;
  a->sysTimeHz = 0;
  CHECK(waitFor([link] { return link->targetSys() == 2; }, 8000));
  CHECK(*changed && link->generation() > gen0);
  runFor(100);
  CHECK(!tuner->prepared);
  auto f = tuner->applyProposal("roll", ap).future();
  settle(f);
  CHECK(matches(errorOf(f), "no proposal|another connection"));
  CHECK(b->sets.empty());
  CHECK(!eventsOf(*journal, "connection").empty());
  tuner->stopMonitor();
  rt::detach(tuner->stop().future());
  a->silence();
  b->stop();
  runFor(30);
}

TEST(wv_type_change_same_sysid_is_another_vehicle) {
  Rig r = setup();
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}}));
  const Json ap = approve(*r.tuner, "roll");
  const int gen0 = r.link->generation();
  r.fv->type = 13;   // the mirror now shows a hexacopter with the same system id
  CHECK(waitFor([&r, gen0] { return r.link->generation() > gen0; }, 2500));
  runFor(50);
  auto f = applyF(r, "roll", ap);
  settle(f);
  CHECK(matches(errorOf(f), "no proposal|another connection"));
  CHECK(r.fv->sets.empty());
  done(r);
}

TEST(wv_reconnect_rejects_pending_write_no_retry_to_new_vehicle) {
  Rig r = setup();
  r.fv->echoDelayMs = 2000;   // the old vehicle's answer would come late
  auto w = app::verifiedSet(*r.link, "ATC_RAT_RLL_P", 0.16).future();
  runFor(300);
  auto* fv2 = new FakeVehicle(test::copterParams(), false);   // reconnect: another vehicle, same system id
  fv2->uid = "ffffffffffffffffffffffff";
  fv2->attach(*r.link);
  const Json res = valueOf(w);
  CHECK(res["status"].strOr("") == "uncertain");
  CHECK(matches(res["error"].strOr(""), "connection (changed|was closed)"));
  runFor(2500);
  CHECK(fv2->setsOf("ATC_RAT_RLL_P").empty());   // no retry went to the new vehicle
  CHECK(r.fv->setsOf("ATC_RAT_RLL_P").size() == 1);
  const auto it = r.link->params().find("ATC_RAT_RLL_P");
  CHECK(it == r.link->params().end() || it->second.value != f32(0.16));   // the late echo of the old connection is not taken
  fv2->stop();
  r.fv->silence();
  done(r);
}

TEST(wv_different_vehicle_after_reconnect) {
  Rig r = setup();
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  const Json ap = approve(*r.tuner, "roll");
  const std::string key0 = r.tuner->identity["key"].strOr("");
  r.fv->silence();
  auto* fv2 = new FakeVehicle(test::copterParams(), false);
  fv2->uid = "abababababababababababab";
  fv2->attach(*r.link);
  CHECK(waitFor([&r] { return r.link->alive() && r.link->autopilot().isObject(); }, 2000));
  auto f = applyF(r, "roll", ap);
  settle(f);
  CHECK(matches(errorOf(f), "no proposal|another connection"));
  CHECK(fv2->sets.empty());
  valueOf(r.tuner->prepare(true).future());
  CHECK(r.tuner->identity["uid"].strOr("") == fv2->uid + "000000000000");
  CHECK(r.tuner->identity["key"].strOr("") != key0);
  fv2->stop();
  done(r);
}

TEST(wv_ack_for_another_ground_station_ignored) {
  Rig r = setup(false, false, test::copterParams(), false);
  r.fv->ackForOther = true;
  auto a = r.link->commandAck(511, {30, 40000}, 600);
  settle(a);
  CHECK(has(errorOf(a), "no ack"));
  bool sent = false;
  for (const auto& c : r.fv->commands)
    if (c.command == 511) sent = true;
  CHECK(sent);
  r.fv->ackForOther = false;
  CHECK(valueOf(r.link->commandAck(511, {30, 40000}, 600))["result"].numOr(-1) == 0);
  done(r);
}

TEST(wv_identity_from_autopilot_version) {
  Rig r = setup();
  const Json& id = r.tuner->identity;
  CHECK(id["fw"].strOr("") == "4.5.7");
  CHECK(id["gitHash"].strOr("") == "abcdef12");
  CHECK(id["uid"].strOr("") == "0123456789abcdef01234567000000000000");
  CHECK(id["gen"].numOr(-1) == r.link->generation());
  CHECK(id["sysid"].numOr(0) == 1 && id["autopilot"].numOr(0) == 3);
  CHECK(id["key"].strOr("") == "g" + std::to_string(r.link->generation()) + ":1/1");
  CHECK(id["paramHash"].strOr("") == engine::paramHash(r.tuner->paramsAtPrepare));
  CHECK(matches(engine::identityText(id), "^system id 1 \\(quadrotor\\), ArduPilot 4\\.5\\.7 \\(abcdef12\\), board UID 0123456789abcdef01234567, connection #\\d+$"));
  done(r);
}

// ---------------- F-29 / F-30: exact verification

TEST(wv_exact_float32_verified_and_journaled) {
  Rig r = setup();
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  CHECK(valueOf(applyF(r, "roll", approve(*r.tuner, "roll"))));
  CHECK(r.fv->params["ATC_RAT_RLL_P"] == f32(0.16));
  std::vector<Json> w;
  for (const auto& e : eventsOf(*r.journal, "write"))
    if (e["name"].strOr("") == "ATC_RAT_RLL_P") w.push_back(e);
  CHECK(w.size() == 1);
  if (w.size() == 1) {
    CHECK(w[0]["requested"].numOr(0) == 0.16);
    CHECK(w[0]["sent"].numOr(0) == f32(0.16));
    CHECK(w[0]["echo"].numOr(0) == f32(0.16));
    CHECK(w[0]["status"].strOr("") == "verified");
    CHECK(w[0]["identity"].strOr("") == r.tuner->identity["key"].strOr("?"));
    CHECK(w[0]["type"].numOr(0) == 9);
  }
  for (const char* k : {"identity", "proposal", "approval", "txn", "write"}) CHECK(!eventsOf(*r.journal, k).empty());
  done(r);
}

TEST(wv_refused_write_rolls_back_whole_apply) {
  Rig r = setup();
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}, {"D", 0.005}}));
  r.fv->readOnly.insert("ATC_RAT_RLL_I");
  auto f = applyF(r, "roll", approve(*r.tuner, "roll"));
  settle(f);
  CHECK(codeOf(f) == "apply-failed");
  CHECK(has(errorOf(f), "refused 0.16 and kept 0.135") && has(errorOf(f), "Rolled back"));
  CHECK(r.fv->params["ATC_RAT_RLL_P"] == f32(0.135) && r.fv->params["ATC_RAT_RLL_D"] == f32(0.0036) && r.fv->params["ATC_RAT_RLL_I"] == f32(0.135));
  CHECK(r.tuner->state == "paused");
  const auto ends = eventsOf(*r.journal, "txn");
  const Json end = ends.empty() ? Json() : ends.back();
  CHECK(end["phase"].strOr("") == "end" && end["rolledBack"].asBool() && end["failed"].strOr("") == "ATC_RAT_RLL_I");
  done(r);
}

TEST(wv_clamped_write_detected) {
  Rig r = setup();
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  r.fv->clamp["ATC_RAT_RLL_P"] = {0, 0.15};
  auto f = applyF(r, "roll", approve(*r.tuner, "roll"));
  settle(f);
  CHECK(has(errorOf(f), "asked for 0.16, the autopilot holds 0.15"));
  CHECK(r.fv->params["ATC_RAT_RLL_P"] == f32(0.135));
  bool found = false;
  for (const auto& e : eventsOf(*r.journal, "write"))
    if (e["name"].strOr("") == "ATC_RAT_RLL_P" && e["status"].strOr("") == "different" && e["readBack"].numOr(0) == f32(0.15)) found = true;
  CHECK(found);
  done(r);
}

TEST(wv_integer_parameters_cast_by_type) {
  Rig r = setup();
  const Json r1 = valueOf(app::verifiedSet(*r.link, "INS_HNTCH_MODE", 2.7).future());
  CHECK(r1["status"].strOr("") == "verified" && r1["sent"].numOr(0) == 3 && r1["type"].numOr(0) == 2 && has(r1["note"].strOr(""), "rounded to 3"));
  const Json r2 = valueOf(app::verifiedSet(*r.link, "INS_HNTCH_ENABLE", 300).future());
  CHECK(r2["status"].strOr("") == "invalid");
  CHECK(r.fv->setsOf("INS_HNTCH_ENABLE").empty());
  // what the cast avoids: ArduPilot truncates a float sent to a whole-number parameter (2.7 -> 2)
  CHECK(valueOf(r.link->setParam("INS_HNTCH_MODE", 2.7)) == 2);
  const Json r3 = valueOf(app::verifiedSet(*r.link, "ATC_RAT_RLL_D", 0.0047).future());
  CHECK(r3["status"].strOr("") == "verified" && r3["sent"].numOr(0) == f32(0.0047));
  done(r);
}

TEST(wv_dropped_echoes_proven_by_read_back) {
  Rig r = setup();
  r.fv->dropEchoes = 2;
  const Json res = valueOf(app::verifiedSet(*r.link, "ATC_RAT_RLL_P", 0.16, quickV()).future());
  CHECK(res["status"].strOr("") == "verified" && res["echo"].isNull() && res["readBack"].numOr(0) == f32(0.16));
  CHECK(r.fv->setsOf("ATC_RAT_RLL_P").size() == 2);
  done(r);
}

TEST(wv_lost_writes_not_applied_and_set_rolled_back) {
  Rig r = setup();
  FakeVehicle* fv = r.fv;
  fv->onSet = [fv](const std::string& name, double) {
    if (name == "ATC_RAT_RLL_P") fv->dropSets = 2;
  };
  const Json t = valueOf(app::runTransaction(*r.link, items({{"ATC_RAT_RLL_P", 0.16}, {"ATC_RAT_RLL_I", 0.16}}), quickT()).future());
  CHECK(!t["ok"].asBool() && t["rows"][1]["status"].strOr("") == "not-applied" && t["rolledBack"].asBool());
  CHECK(r.fv->params["ATC_RAT_RLL_P"] == f32(0.135) && r.fv->params["ATC_RAT_RLL_I"] == f32(0.135));
  CHECK(has(app::txnText(t), "no answer to the write; the autopilot still holds 0.135. Everything else written in this set went back"));
  done(r);
}

TEST(wv_stale_echo_then_read_back) {
  Rig r = setup();
  r.fv->staleEchoes = 1;   // a late answer to an earlier read arrives first, with the old value
  const Json res = valueOf(app::verifiedSet(*r.link, "ATC_RAT_RLL_P", 0.16).future());
  CHECK(res["status"].strOr("") == "verified" && res["echo"].numOr(0) == f32(0.135) && res["readBack"].numOr(0) == f32(0.16));
  done(r);
}

TEST(wv_late_echo_verified) {
  Rig r = setup();
  r.fv->echoDelayMs = 2000;   // longer than the 1.5 s timeout: the link retries, the late echo resolves the write
  const Json res = valueOf(app::verifiedSet(*r.link, "ATC_RAT_RLL_P", 0.16).future());
  CHECK(res["status"].strOr("") == "verified");
  CHECK(r.fv->setsOf("ATC_RAT_RLL_P").size() == 2);
  done(r);
}

TEST(wv_no_answer_uncertain_and_reported) {
  Rig r = setup();
  FakeVehicle* fv = r.fv;
  fv->onSet = [fv](const std::string& name, double) {
    if (name == "ATC_RAT_RLL_P") {   // the vehicle goes quiet after the first write
      fv->ignoreSets = true;
      fv->ignoreReads = true;
    }
  };
  const Json t = valueOf(app::runTransaction(*r.link, items({{"ATC_RAT_RLL_P", 0.16}, {"ATC_RAT_RLL_I", 0.16}}), quickT()).future());
  CHECK(!t["ok"].asBool() && t["rows"][1]["status"].strOr("") == "uncertain" && !t["rolledBack"].asBool());
  std::vector<std::string> names;
  for (const auto& u : t["uncertain"].arr()) names.push_back(u["name"].strOr(""));
  std::sort(names.begin(), names.end());
  CHECK(names == std::vector<std::string>({"ATC_RAT_RLL_I", "ATC_RAT_RLL_P"}));
  CHECK(matches(app::txnText(t), "Uncertain parameters: .*check them before flying"));
  done(r);
}

TEST(wv_missing_parameter_stops_before_writing) {
  Rig r = setup();
  r.fv->missing.insert("ATC_RAT_RLL_IMAX");
  const Json t = valueOf(app::runTransaction(*r.link, items({{"ATC_RAT_RLL_P", 0.16}, {"ATC_RAT_RLL_IMAX", 0.6}}), quickT()).future());
  CHECK(!t["ok"].asBool() && has(t["aborted"].strOr(""), "ATC_RAT_RLL_IMAX does not exist on this autopilot"));
  CHECK(r.fv->sets.empty());
  done(r);
}

// ---------------- F-32: transactions in flight and failed rollbacks

TEST(wv_inflight_failure_rolls_back_within_stepmax_and_pauses) {
  Rig r = setup(true, true);
  propose(*r.tuner, "pitch", Json::obj({{"P", 0.27}, {"I", 0.27}}));   // x2: two steps
  r.fv->clamp["ATC_RAT_PIT_I"] = {0, 0.2};                            // step 2: I 0.27 is stored as 0.2
  auto stored = std::make_shared<std::vector<FakeVehicle::Set>>();
  r.fv->onSet = [stored](const std::string& name, double v) { stored->push_back({0, name, v}); };
  auto f = applyF(r, "pitch", approve(*r.tuner, "pitch"));
  settle(f);
  CHECK(codeOf(f) == "apply-failed" && has(errorOf(f), "Rolled back"));
  CHECK(r.fv->params["ATC_RAT_PIT_P"] == f32(0.135) && r.fv->params["ATC_RAT_PIT_I"] == f32(0.135));
  CHECK(maxWriteRatio(*stored, test::copterParams()) <= r.tuner->opts["stepMax"].num() * (1 + 1e-6));   // no change of a stored value above stepMax
  CHECK(r.tuner->state == "paused");
  done(r);
}

TEST(wv_failed_rollback_lists_uncertain_parameters) {
  Rig r = setup();
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}, {"D", 0.005}}));
  FakeVehicle* fv = r.fv;
  fv->onSet = [fv](const std::string& name, double) {
    if (name == "ATC_RAT_RLL_D") fv->readOnly.insert("ATC_RAT_RLL_D");   // D cannot be written back
  };
  fv->clamp["ATC_RAT_RLL_I"] = {0, 0.15};
  auto f = applyF(r, "roll", approve(*r.tuner, "roll"));
  settle(f);
  CHECK(has(errorOf(f), "uncertain parameters: ATC_RAT_RLL_D: the autopilot refused 0.0036 and kept 0.005") && has(errorOf(f), "check them before flying"));
  CHECK(r.fv->params["ATC_RAT_RLL_P"] == f32(0.135) && r.fv->params["ATC_RAT_RLL_I"] == f32(0.135));
  const auto ends = eventsOf(*r.journal, "txn");
  const Json end = ends.empty() ? Json() : ends.back();
  CHECK(!end["rolledBack"].asBool() && end["uncertain"].size() == 1);
  done(r);
}

// ---------------- F-33: the approval names the proposal

TEST(wv_approval_must_match_proposal) {
  Rig r = setup();
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}}));
  const Json old = approve(*r.tuner, "roll");
  propose(*r.tuner, "roll", Json::obj({{"P", 0.17}}));   // a newer analysis replaced it
  auto a = applyF(r, "roll", old);
  settle(a);
  CHECK(codeOf(a) == "approval");
  auto b = applyF(r, "roll", Json::obj({{"id", r.tuner->results["roll"]["proposal"]["id"]}, {"hash", "0000000000000000"}}));
  settle(b);
  CHECK(codeOf(b) == "approval");
  auto c = r.tuner->applyProposal("roll").future();
  settle(c);
  CHECK(codeOf(c) == "approval");
  CHECK(r.fv->sets.empty());
  CHECK(eventsOf(*r.journal, "refused").size() == 3);
  CHECK(valueOf(applyF(r, "roll", approve(*r.tuner, "roll"))));
  CHECK(r.fv->params["ATC_RAT_RLL_P"] == f32(0.17));
  done(r);
}

TEST(wv_rows_full_names_reasons_and_hash) {
  Rig r = setup();
  const Json p = propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}, {"IMAX", 0.7}}));
  CHECK(fieldList(p["rows"], "name") == "ATC_RAT_RLL_IMAX,ATC_RAT_RLL_P,ATC_RAT_RLL_I");
  bool reasons = true;
  for (const auto& row : p["rows"].arr())
    if (row["reason"].strOr("").empty() || row["reboot"].asBool()) reasons = false;
  CHECK(reasons);
  CHECK(matches(p["hash"].strOr(""), "^[0-9a-f]{16}$"));
  const Json again = engine::makeProposal("live", "roll", r.tuner->identity, p["rows"]);
  CHECK(again["hash"] == p["hash"]);
  Json rows2 = p["rows"];
  rows2.arr()[0].set("to", 0.71);
  CHECK(!(engine::makeProposal("live", "roll", r.tuner->identity, rows2)["hash"] == p["hash"]));
  CHECK(valueOf(applyF(r, "roll", approve(*r.tuner, "roll"))));
  CHECK(r.fv->params["ATC_RAT_RLL_IMAX"] == f32(0.7));   // IMAX is written like every value shown
  done(r);
}

TEST(wv_change_by_another_station_makes_proposal_stale) {
  Rig r = setup();
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  r.fv->foreignSet("ATC_RAT_RLL_I", 0.2);   // Mission Planner writes I after the analysis
  runFor(50);
  bool reported = false;
  for (const auto& m : r.tuner->messages.arr())
    if (has(m["text"].strOr(""), "ATC_RAT_RLL_I changed on the vehicle: 0.135 -> 0.2 (not by AP Tuner")) reported = true;
  CHECK(reported);
  auto f = applyF(r, "roll", approve(*r.tuner, "roll"));
  settle(f);
  CHECK(codeOf(f) == "stale" && has(errorOf(f), "ATC_RAT_RLL_I is 0.2 on the vehicle, the proposal was made from 0.135"));
  CHECK(r.fv->sets.empty());
  bool conflict = false;
  for (const auto& e : eventsOf(*r.journal, "conflict"))
    if (e["name"].strOr("") == "ATC_RAT_RLL_I") conflict = true;
  CHECK(conflict);
  done(r);
}

TEST(wv_old_proposal_refused) {
  Rig r = setup();
  Json& p = propose(*r.tuner, "roll", Json::obj({{"P", 0.16}}));
  p.set("at", p["at"].num() - 16 * 60000);
  auto f = applyF(r, "roll", approve(*r.tuner, "roll"));
  settle(f);
  CHECK(codeOf(f) == "stale" && has(errorOf(f), "16 min old"));
  CHECK(r.fv->sets.empty());
  done(r);
}

TEST(wv_undo_reads_fresh_and_reports) {
  Rig r = setup();
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}, {"I", 0.16}}));
  CHECK(valueOf(applyF(r, "roll", approve(*r.tuner, "roll"))));
  r.fv->readOnly.insert("ATC_RAT_RLL_I");
  const std::size_t n0 = r.fv->sets.size();
  auto f = r.tuner->undoAxis("roll").future();
  settle(f);
  CHECK(has(errorOf(f), "not written back: ATC_RAT_RLL_I: the autopilot refused 0.135 and kept 0.16"));
  CHECK(r.fv->params["ATC_RAT_RLL_P"] == f32(0.135));
  bool only = true;
  for (std::size_t i = n0; i < r.fv->sets.size(); i++)
    if (r.fv->sets[i].name != "ATC_RAT_RLL_P" && r.fv->sets[i].name != "ATC_RAT_RLL_I") only = false;
  CHECK(only);   // only the values that differ are written
  done(r);
}

// ---------------- F-35: values that act only after a reboot

TEST(wv_reboot_values_never_in_flight) {
  Rig r = setup(true, true);
  auto a = r.tuner->applyFilterChange(Json::obj({{"INS_HNTCH_REF", 1}}), "test").future();
  settle(a);
  CHECK(codeOf(a) == "reboot-required");
  r.fv->escRpm = 6000;
  auto* notch = new app::NotchLive(*r.link, *r.tuner);
  runFor(600);
  auto b = notch->applyEsc().future();
  settle(b);
  CHECK(codeOf(b) == "reboot-required" && has(errorOf(b), "INS_HNTCH_REF"));
  auto c = notch->enableFft().future();
  settle(c);
  CHECK(codeOf(c) == "reboot-required");
  CHECK(r.fv->sets.empty());
  done(r);
}

TEST(wv_reboot_workflow_on_the_ground_verified_after) {
  Rig r = setup();
  r.fv->escRpm = 6000;
  r.fv->rebootMs = 1500;
  auto* notch = new app::NotchLive(*r.link, *r.tuner);
  runFor(600);
  const Json applied = valueOf(notch->applyEsc().future());
  CHECK(applied["INS_HNTCH_MODE"].numOr(0) == 3);
  CHECK(fieldList(r.tuner->rebootPending, "name") == "INS_HNTCH_REF");
  const Json st = valueOf(notch->status().future());
  CHECK(!st["notchActive"].asBool());
  CHECK(st["rebootPending"].size() == 1 && st["rebootPending"][0].strOr("") == "INS_HNTCH_REF");
  const int gen0 = r.link->generation();
  const Json res = valueOf(notch->reboot().future());
  CHECK(res["ok"].asBool());
  CHECK(r.fv->reboots == 1);
  CHECK(res["rows"].size() == 1 && res["rows"][0]["name"].strOr("") == "INS_HNTCH_REF" && res["rows"][0]["ok"].asBool());
  CHECK(r.tuner->rebootPending.size() == 0);
  CHECK(r.link->generation() == gen0);   // the same connection (the fake keeps its transport)
  CHECK(r.link->bootCount() >= 1);
  CHECK(valueOf(notch->status().future())["notchActive"].asBool());
  std::string phases;
  for (const auto& e : eventsOf(*r.journal, "reboot")) phases += (phases.empty() ? "" : ",") + e["phase"].strOr("");
  CHECK(phases == "request,verified");
  done(r);
}

TEST(wv_reboot_refused_while_armed) {
  Rig r = setup(true, true);
  auto* notch = new app::NotchLive(*r.link, *r.tuner);
  auto f = notch->reboot().future();
  settle(f);
  CHECK(has(errorOf(f), "refusing to reboot an armed vehicle"));
  bool sent = false;
  for (const auto& c : r.fv->commands)
    if (c.command == 246) sent = true;
  CHECK(!sent);
  done(r);
}

TEST(wv_other_board_after_reboot_detected) {
  Rig r = setup();
  FakeVehicle* fv = r.fv;
  fv->rebootMs = 1500;
  fv->onReboot = [fv] { fv->uid = "555555555555555555555555"; };
  const int gen0 = r.link->generation();
  app::RebootOpts ro;
  ro.timeoutMs = 15000;
  const Json res = valueOf(app::rebootAndVerify(*r.link, Json(Json::Array{Json::obj({{"name", "ATC_RAT_RLL_P"}, {"value", 0.135}})}), ro).future());
  CHECK(!res["ok"].asBool() && has(res["reason"].strOr(""), "a different vehicle answered after the reboot"));
  CHECK(r.link->generation() > gen0);
  runFor(1100);
  CHECK(!r.tuner->prepared);
  done(r);
}

// ---------------- Telemetry + USB: the ground write

TEST(wv_ground_write_verified_undo_file_and_undo) {
  GRig G = groundSetup();
  const Json ap = review(*G.g, rollRows(), Json::obj({{"ATC_RAT_RLL_P", f32(0.135)}}));
  bool clean = true;
  for (const auto& row : G.g->rows().arr())
    if (!row["blocked"].isNull() || !row["vehicle"].isNumber()) clean = false;
  CHECK(clean);
  const Json res = valueOf(G.g->write(rollNames(), ap).future());
  CHECK(statuses(res) == "verified,verified,verified");
  CHECK(G.fv->params["ATC_RAT_RLL_D"] == f32(0.0045));
  const std::string undo = app::readText(G.g->files["undo"].strOr(""));
  CHECK(matches(undo, "# For: system id 1 \\(quadrotor\\), ArduPilot 4\\.5\\.7 \\(abcdef12\\), board UID 0123456789abcdef01234567, connection #\\d+\\. Load it only on this vehicle\\."));
  CHECK(has(undo, "\r\nATC_RAT_RLL_P,0.135\r\n") && has(undo, "\r\nATC_RAT_RLL_D,0.0036\r\n"));
  CHECK(has(app::readText(G.g->files["param"].strOr("")), "ATC_RAT_RLL_D,0.0045"));
  CHECK(has(G.g->info(), "all 3 written and read back"));
  valueOf(G.g->undo().future());
  CHECK(fieldList(G.g->writes(), "undo") == "undone,undone,undone");
  CHECK(G.fv->params["ATC_RAT_RLL_P"] == f32(0.135));
  CHECK(has(G.g->info(), "previous values written back and read back"));
  G.fv->stop();
  runFor(30);
}

TEST(wv_ground_refused_row_others_rolled_back_honest_status) {
  GRig G = groundSetup();
  const Json ap = review(*G.g, rollRows());
  G.fv->readOnly.insert("ATC_RAT_RLL_I");
  const Json res = valueOf(G.g->write(rollNames(), ap).future());
  CHECK(statuses(res) == "rolled back,refused,not written");
  bool anyOk = false;
  for (const auto& w : res.arr())
    if (w["ok"].asBool()) anyOk = true;
  CHECK(!anyOk);   // no row claims success
  CHECK(has(res[1]["text"].strOr(""), "refused 0.16 and kept 0.135"));
  CHECK(matches(G.g->info(), "refused .*went back to its value from before"));
  CHECK(G.fv->params["ATC_RAT_RLL_P"] == f32(0.135) && G.fv->params["ATC_RAT_RLL_D"] == f32(0.0036));
  CHECK(!G.g->files.has("param"));   // no "written" file when nothing stayed written
  G.fv->stop();
  runFor(30);
}

TEST(wv_ground_approval_and_connection_bound) {
  GRig G = groundSetup();
  const Json ap = review(*G.g, rollRows());
  auto a = G.g->write({"ATC_RAT_RLL_P"}, Json::obj({{"id", ap["id"]}, {"hash", "x"}})).future();
  settle(a);
  CHECK(has(errorOf(a), "the list changed since it was shown"));
  G.fv->silence();
  auto* fv2 = new FakeVehicle(test::copterParams(), false);   // the USB cable was plugged again
  fv2->attach(*G.link);
  CHECK(waitFor([&G] { return G.link->alive(); }, 2000));
  auto b = G.g->write({"ATC_RAT_RLL_P"}, ap).future();
  settle(b);
  CHECK(has(errorOf(b), "connection changed since the review"));
  CHECK(fv2->sets.size() + G.fv->sets.size() == 0);
  valueOf(G.g->recheck().future());
  const Json ap2 = Json::obj({{"id", G.g->proposal()["id"]}, {"hash", G.g->proposal()["hash"]}});
  CHECK(!(ap2["id"] == ap["id"]));
  const Json res = valueOf(G.g->write({"ATC_RAT_RLL_P"}, ap2).future());
  CHECK(res[0]["status"].strOr("") == "verified" && fv2->params["ATC_RAT_RLL_P"] == f32(0.16));
  fv2->stop();
  runFor(30);
}

TEST(wv_ground_value_changed_since_review_writes_nothing) {
  GRig G = groundSetup();
  const Json ap = review(*G.g, rollRows());
  G.fv->foreignSet("ATC_RAT_RLL_I", 0.14);
  const Json res = valueOf(G.g->write(rollNames(), ap).future());
  CHECK(statuses(res) == "not written,not written,not written");
  CHECK(has(G.g->info(), "nothing written: changed on the autopilot since the values were shown: ATC_RAT_RLL_I is 0.14, was 0.135"));
  CHECK(G.fv->sets.empty() && !G.g->files.has("undo"));   // not even the UNDO file
  G.fv->stop();
  runFor(30);
}

TEST(wv_ground_log_of_another_vehicle_or_firmware_blocked) {
  GRig G = groundSetup();
  Json ap = review(*G.g, rollRows(), Json::obj({{"SYSID_THISMAV", Json()}, {"MAV_SYSID", 7}}));   // ArduPilot 4.7 name
  bool all = true;
  for (const auto& row : G.g->rows().arr())
    if (!has(row["blocked"].strOr(""), "the log is from system id 7, the autopilot on this connection is system id 1")) all = false;
  CHECK(all);
  auto f = G.g->write({"ATC_RAT_RLL_P"}, ap).future();
  settle(f);
  CHECK(has(errorOf(f), "cannot be written"));
  ap = review(*G.g, rollRows(), Json::object(), "ArduCopter V4.5.6 (abcdef12)");
  all = true;
  for (const auto& row : G.g->rows().arr())
    if (!has(row["blocked"].strOr(""), "recorded with ArduPilot 4.5.6, the autopilot now runs 4.5.7")) all = false;
  CHECK(all);
  ap = review(*G.g, Json(Json::Array{Json::obj({{"name", "ATC_RAT_RLL_P"}, {"value", 0.16}, {"from", 0.12}})}));
  CHECK(has(G.g->rows()[0]["blocked"].strOr(""), "the autopilot holds 0.135, the log had 0.12"));
  CHECK(G.fv->sets.empty());
  G.fv->stop();
  runFor(30);
}

TEST(wv_ground_undo_failure_reported) {
  GRig G = groundSetup();
  const Json ap = review(*G.g, rollRows());
  valueOf(G.g->write(rollNames(), ap).future());
  G.fv->readOnly.insert("ATC_RAT_RLL_P");
  valueOf(G.g->undo().future());
  CHECK(fieldList(G.g->writes(), "undo") == "refused,undone,undone");
  CHECK(has(G.g->info(), "undo incomplete: ATC_RAT_RLL_P: the autopilot refused 0.135 and kept 0.16. Load the UNDO file in Mission Planner"));
  G.fv->stop();
  runFor(30);
}

TEST(wv_ground_notch_reboot_workflow_with_values_after_reboot) {
  GRig G = groundSetup();
  G.fv->rebootMs = 1500;
  for (const char* k : {"INS_HNTCH_FREQ", "INS_HNTCH_BW", "INS_HNTCH_REF", "INS_HNTCH_HMNCS"}) G.fv->hidden.insert(k);   // appear once the notch is on
  const Json rows(Json::Array{Json::obj({{"group", "notch"}, {"name", "INS_HNTCH_ENABLE"}, {"value", 1}, {"from", 0}}),
                              Json::obj({{"group", "notch"}, {"name", "INS_HNTCH_FREQ"}, {"value", 55}}), Json::obj({{"group", "notch"}, {"name", "INS_HNTCH_BW"}, {"value", 27.5}}),
                              Json::obj({{"group", "notch"}, {"name", "INS_HNTCH_REF"}, {"value", 0.35}}), Json::obj({{"group", "notch"}, {"name", "INS_HNTCH_HMNCS"}, {"value", 3}})});
  const Json ap = review(*G.g, rows);
  CHECK(fieldList(G.g->rows(), "deferred") == "false,true,true,true,true");
  valueOf(G.g->write({"INS_HNTCH_ENABLE", "INS_HNTCH_FREQ", "INS_HNTCH_BW", "INS_HNTCH_REF", "INS_HNTCH_HMNCS"}, ap).future());
  CHECK(statuses(G.g->writes()) == "verified,after reboot,after reboot,after reboot,after reboot");
  CHECK(has(G.g->info(), "take effect only after a reboot: press Reboot"));
  valueOf(G.g->rebootNow().future());
  CHECK(G.fv->reboots == 1);
  CHECK(statuses(G.g->writes()) == "verified,verified,verified,verified,unchanged");   // HMNCS was 3 already
  CHECK(G.fv->params["INS_HNTCH_FREQ"] == f32(55) && G.fv->params["INS_HNTCH_REF"] == f32(0.35));
  CHECK(fieldList(G.g->rebootPending(), "name") == "INS_HNTCH_REF");   // REF changed and needs one more reboot
  CHECK(has(G.g->info(), "need one more reboot"));
  valueOf(G.g->rebootNow().future());
  CHECK(G.fv->reboots == 2 && G.g->rebootPending().size() == 0);
  CHECK(has(G.g->info(), "rebooted: every value written here was read back after the reboot"));
  bool allBack = true;
  for (const auto& w : G.g->writes().arr())
    if (w["afterReboot"].isString() && !has(w["afterReboot"].str(), "read back after the reboot")) allBack = false;
  CHECK(allBack);
  // Undo: REF waits for a reboot again; the reboot checks the values from before (not the values written)
  std::map<std::string, Json> before;
  for (const auto& w : G.g->writes().arr()) before[w["name"].str()] = w["from"];
  valueOf(G.g->undo().future());
  CHECK(fieldList(G.g->writes(), "undo") == "undone,undone,undone,undone,null");   // HMNCS did not change
  CHECK(fieldList(G.g->rebootPending(), "name") == "INS_HNTCH_REF");
  valueOf(G.g->rebootNow().future());
  CHECK(G.fv->reboots == 3 && G.g->rebootPending().size() == 0);
  CHECK(has(G.g->info(), "rebooted: every value written here was read back after the reboot"));
  const Json rb = G.g->snapshot()["reboot"];
  bool same = rb["ok"].asBool() && rb["rows"].size() == 4;
  for (const auto& x : rb["rows"].arr())
    if (!x["ok"].asBool() || !x["holds"].isNumber() || x["holds"].num() != before[x["name"].str()].num()) same = false;
  CHECK(same);
  allBack = true;
  for (const auto& w : G.g->writes().arr())
    if (w["afterReboot"].isString() && !has(w["afterReboot"].str(), "read back after the reboot")) allBack = false;
  CHECK(allBack);
  G.fv->stop();
  runFor(30);
}

// ---------------- F-43: the exported file

TEST(wv_param_file_checks_format_and_identity) {
  const Json id = Json::obj({{"sysid", 1}, {"compid", 1}, {"typeName", "MAV_TYPE_QUADROTOR"}, {"fw", "4.5.7"}, {"gitHash", "abcdef12"},
                             {"uid", "0123456789abcdef01234567000000000000"}, {"gen", 3}});
  const Json rows(Json::Array{Json::obj({{"group", "roll"}, {"name", "ATC_RAT_RLL_P"}, {"value", 0.16}, {"from", f32(0.135)}}), Json::obj({{"group", "roll"}, {"name", "ATC_RAT_RLL_D"}, {"value", 4.2e-7}}),
                              Json::obj({{"group", "roll"}, {"name", "ATC_RAT_RLL_I"}, {"value", NAN}}), Json::obj({{"group", "notch"}, {"name", "INS_HNTCH_OPTS"}, {"value", 64}}),
                              Json::obj({{"group", "notch"}, {"name", "INS_HNTCH_MODE"}, {"value", 2.5}}), Json::obj({{"group", "notch"}, {"name", "INS_HNTCH_REF"}, {"value", 2}}),
                              Json::obj({{"group", "notch"}, {"name", "INS_HNTCH_HMNCS"}, {"value", 7}}), Json::obj({{"group", "pitch"}, {"name", "PTCH2SRV_TCONST"}, {"value", 0.15}}),
                              Json::obj({{"group", "x"}, {"name", "SOMETHING_ELSE"}, {"value", 1}})});
  const engine::ParamFileOut out = engine::paramFileChecked(rows, {"test"}, id);
  std::vector<std::string> lines;
  std::size_t at = 0;
  while (true) {
    const auto e = out.text.find("\r\n", at);
    if (e == std::string::npos) break;
    lines.push_back(out.text.substr(at, e - at));
    at = e + 2;
  }
  CHECK(at == out.text.size());   // CRLF line ends, nothing after the last one
  CHECK(lines.size() > 1 && lines[1] == "# For: system id 1 (quadrotor), ArduPilot 4.5.7 (abcdef12), board UID 0123456789abcdef01234567, connection #3. Load it only on this vehicle.");
  std::string values;
  for (const auto& l : lines)
    if (!l.empty() && l[0] >= 'A' && l[0] <= 'Z') values += (values.empty() ? "" : "|") + l;
  CHECK(values == "ATC_RAT_RLL_P,0.16|ATC_RAT_RLL_D,0.00000042|INS_HNTCH_REF,2|INS_HNTCH_HMNCS,7|PTCH2SRV_TCONST,0.15");
  CHECK(fieldList(out.errors, "name") == "ATC_RAT_RLL_I,INS_HNTCH_OPTS,INS_HNTCH_MODE,SOMETHING_ELSE");
  CHECK(fieldList(out.warnings, "name") == "INS_HNTCH_REF,PTCH2SRV_TCONST");
  CHECK(std::find(lines.begin(), lines.end(), "# INS_HNTCH_REF takes effect only after a reboot of the autopilot") != lines.end());
  CHECK(std::find(lines.begin(), lines.end(), "# ATC_RAT_RLL_P was 0.135") != lines.end());
}

TEST(wv_formats_and_hashes_are_fixed) {
  // the same fixed values as test/write-verify.test.js (the two editions agree to the bit)
  std::string fmt;
  for (const double v : {0.16, 0.0036, 4.2e-7, 12345.678, 0.1, 1.0, 0.0, 1e-12, 123456789.0, 0.000123456})
    fmt += (fmt.empty() ? "" : "|") + core::formatParamValue("ATC_RAT_RLL_D", v);
  CHECK(fmt == "0.16|0.0036|0.00000042|12345.678|0.1|1|0|0.000000000001|123456792|0.000123456");
  CHECK(core::formatParamValue("INS_HNTCH_MODE", 3) == "3");
  CHECK(engine::f32hex(0.135) == "3e0a3d71");
  CHECK(engine::fnv1a64("AP Tuner") == "ff9b411acb97fff8");
  CHECK(engine::fnv1a64("") == "cbf29ce484222325");
  CHECK(engine::fnv1a64("a") == "af63dc4c8601ec8c");
  CHECK(engine::paramHash(Json::obj({{"ATC_RAT_RLL_P", 0.135}, {"ATC_RAT_RLL_I", 0.135}, {"X", "text"}})) == "0d6e8b666e1b99bc");
  const Json id = engine::sessionIdentity(Json::obj({{"gen", 3}, {"sysid", 1}, {"compid", 1}, {"autopilot", 3}, {"type", 2}, {"typeName", "MAV_TYPE_QUADROTOR"}, {"uid", "aa"},
                                                     {"fw", "4.5.7"}, {"gitHash", "abcdef12"}}),
                                          Json::obj({{"A", 1}}));
  const Json p = engine::makeProposal("live", "roll", id,
                                      Json(Json::Array{Json::obj({{"name", "ATC_RAT_RLL_P"}, {"from", 0.135}, {"to", 0.16}}), Json::obj({{"name", "ATC_RAT_RLL_IMAX"}, {"to", 0.7}})}));
  CHECK(p["hash"].strOr("") == "51b120c7020a222e");
  CHECK(core::validateParam("INS_HNTCH_OPTS", 16).toJson().dump() == "{\"ok\":true,\"errors\":[],\"warnings\":[],\"reboot\":true,\"int\":true}");
}

// ---------------- F-39: the journal

TEST(wv_journal_is_append_only) {
  Rig r = setup();
  propose(*r.tuner, "roll", Json::obj({{"P", 0.16}}));
  valueOf(applyF(r, "roll", approve(*r.tuner, "roll")));
  r.tuner->setState("ready");
  const std::string first = app::readText(r.journal->file());
  propose(*r.tuner, "roll", Json::obj({{"P", 0.17}}));
  valueOf(applyF(r, "roll", approve(*r.tuner, "roll")));
  const std::string second = app::readText(r.journal->file());
  CHECK(second.rfind(first, 0) == 0 && second.size() > first.size());   // only appended
  bool shape = true;
  for (const auto& e : events(*r.journal))
    if (!e["t"].isNumber() || !e["kind"].isString()) shape = false;
  CHECK(shape);
  bool writes = true;
  for (const auto& e : eventsOf(*r.journal, "write"))
    if (!e.has("requested") || !e.has("sent") || !e.has("echo") || !e.has("status") || !e.has("identity") || !e.has("gen")) writes = false;
  CHECK(writes);
  const auto appr = eventsOf(*r.journal, "approval");
  CHECK(appr.size() == 2);
  for (const auto& a : appr) CHECK(a["by"].strOr("") == "user" && a["proposal"].isString() && a["hash"].isString());
  done(r);
}

TEST(wv_unwritable_journal_blocks_forward_writes) {
  const std::string blocker = sessDir() + "/not-a-dir-" + std::to_string(static_cast<long long>(rt::Loop::now()));
  app::writeText(blocker, "x");   // the journal folder cannot be created: a file is in the way
  auto* link = new io::Link();
  auto* fv = new FakeVehicle(test::copterParams(), false);
  fv->attach(*link);
  auto* tuner = new Tuner(*link, sessDir(), new app::Journal(blocker + "/journal"));
  CHECK(waitFor([link] { return link->alive(); }, 2000));
  valueOf(tuner->prepare(true).future());
  CHECK(!tuner->audit().ok());
  propose(*tuner, "roll", Json::obj({{"P", 0.16}}));
  auto f = tuner->applyProposal("roll", approve(*tuner, "roll")).future();
  settle(f);
  CHECK(has(errorOf(f), "the write journal cannot be written"));
  CHECK(fv->sets.empty());
  CHECK(tuner->snapshot()["audit"]["lost"].numOr(0) > 0);
  rt::detach(tuner->stop().future());
  fv->stop();
  runFor(30);
}

// ---------------- SITL: the ground write on ArduCopter SITL (tools/sitl-ground-write.js --cpp starts the simulator and
// sets APT_SITL_TCP; without it this test is skipped). The same phases as the JavaScript edition of the script.

namespace {
double readP(io::Link& link, const std::string& name) {
  auto f = link.requestParam(name, 1500, 2);
  settle(f, 8000);
  return f.ready() && !f.error() ? f.get() : NAN;
}
std::string fmtv(double v) { return std::isfinite(v) ? core::jsToString(core::jsRoundPrecision(v, 6)) : "undefined"; }
void step(bool ok, const std::string& what) {
  std::printf("    %s %s\n", ok ? "PASS" : "FAIL", what.c_str());
  std::fflush(stdout);
  CHECK(ok);
}
/** the review as after a log download from this vehicle: the log holds the values it has now */
Json sitlReview(app::GroundTune& g, io::Link& link, const Json& rows) {
  const Json id = link.identity();
  std::string fw = id["fw"].strOr("");
  const auto dash = fw.find('-');
  if (dash != std::string::npos) fw = fw.substr(0, dash);
  g.result = Json::obj({{"family", "copter"},
                        {"firmware", "ArduCopter V" + fw + " (" + id["gitHash"].strOr("") + ")"},
                        {"params", Json::obj({{"MAV_SYSID", id["sysid"]}})},
                        {"axes", Json::object()},
                        {"chosenAxes", Json::array()}});
  Json rs = Json::array();
  for (const auto& r : rows.arr()) {
    Json x = Json::obj({{"group", "sitl"}, {"label", r["name"]}});
    for (const auto& [k, v] : r.items()) x.set(k, v);
    rs.push(x);
  }
  GroundTestAccess::setRows(g, rs);
  auto f = GroundTestAccess::review(g);
  CHECK(settle(f));
  CHECK(errorOf(f).empty());
  GroundTestAccess::setPhase(g, "review", "SITL review");
  return Json::obj({{"id", g.proposal()["id"]}, {"hash", g.proposal()["hash"]}});
}
Json rebootF(app::GroundTune& g) {
  auto f = g.rebootNow().future();
  settle(f, 90000);
  const std::string err = errorOf(f);
  if (!f.ready() || !err.empty()) step(false, "the reboot did not finish" + (err.empty() ? std::string() : ": " + err));
  return f.ready() && !f.error() ? f.get() : Json();
}
}  // namespace

TEST(wv_sitl_ground_write) {
  const char* port = std::getenv("APT_SITL_TCP");
  if (!port || !*port) {
    std::printf("    skipped: no simulator (tools/sitl-ground-write.js --cpp runs this test against ArduCopter SITL)\n");
    return;
  }
  const char* dirEnv = std::getenv("APT_SITL_DIR");
  const std::string dir = dirEnv && *dirEnv ? std::string(dirEnv) : sessDir() + "/sitl";
  auto* link = new io::Link();
  // the simulator waits for one client on its TCP port: connect again every 2 s until it answers
  const double t0 = rt::Loop::now();
  double last = -1e12;
  while (!link->alive() && rt::Loop::now() - t0 < 60000) {
    if (!link->connected() && rt::Loop::now() - last > 2000) {
      last = rt::Loop::now();
      link->connect(Json::obj({{"kind", "tcp"}, {"host", "127.0.0.1"}, {"port", std::atoi(port)}}));
    }
    runFor(200);
  }
  step(link->alive(), "connected to the simulator (TCP " + std::string(port) + ")");
  if (!link->alive()) return;
  auto* g = new app::GroundTune(*link, dir, new app::Journal(dir + "/journal"));

  // 1 identity
  {
    auto w = link->waitAutopilot(5000).future();
    settle(w, 8000);
  }
  const Json id = link->identity();
  std::printf("    vehicle: %s\n", engine::identityText(id).c_str());
  step(!id["fw"].strOr("").empty() && !id["gitHash"].strOr("").empty() && !id["uid"].strOr("").empty(),
       "AUTOPILOT_VERSION decoded: ArduPilot " + id["fw"].strOr("?") + " (" + id["gitHash"].strOr("?") + "), board UID " + id["uid"].strOr("?"));
  const bool disarmed = link->vehicle().isObject() && !link->vehicle()["armed"].asBool();
  step(disarmed, "the simulator is disarmed");
  if (!disarmed) return;

  // 2 the ground write of four gains, read back, the UNDO file, Undo
  const std::vector<std::string> names{"ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D", "ATC_ANG_RLL_P"};
  std::map<std::string, double> before;
  for (const auto& n : names) before[n] = readP(*link, n);
  Json rows = Json::array();
  for (const auto& n : names) rows.push(Json::obj({{"name", n}, {"from", before[n]}, {"value", core::jsRoundPrecision(before[n] * 1.1, 6)}}));
  Json ap = sitlReview(*g, *link, rows);
  bool writable = true;
  std::string list;
  for (const auto& r : g->rows().arr()) {
    if (!r["blocked"].strOr("").empty() || r["same"].asBool()) writable = false;
    list += (list.empty() ? "" : ", ") + r["name"].strOr("") + " " + fmtv(r["vehicle"].numOr(NAN)) + " -> " + engine::jsText(r["value"]);
  }
  step(writable, "review: every row can be written (" + list + ")");
  Json res = valueOf(g->write(names, ap).future());
  step(statuses(res) == "verified,verified,verified,verified", "write: every value verified (" + statuses(res) + ")");
  bool holds = true;
  for (const auto& r : rows.arr())
    if (!engine::sameF32(readP(*link, r["name"].str()), r["value"].num())) holds = false;
  step(holds, "read back fresh: the vehicle holds every new value");
  std::string undoText;
  try {
    undoText = app::readText(g->files["undo"].strOr(""));
  } catch (const std::exception&) {
  }
  bool undoOk = has(undoText, "# For: system id 1 (quadrotor), ArduPilot ");
  for (const auto& n : names)
    if (!has(undoText, "\r\n" + n + ",")) undoOk = false;
  step(undoOk, "UNDO file saved with the vehicle in its header and every value from before");
  valueOf(g->undo().future());
  step(fieldList(g->writes(), "undo") == "undone,undone,undone,undone", "Undo: every value written back and read back (" + fieldList(g->writes(), "undo") + ")");
  holds = true;
  for (const auto& n : names)
    if (!engine::sameF32(readP(*link, n), before[n])) holds = false;
  step(holds, "read back fresh: the vehicle holds the values from before again");

  // 3 a value ArduPilot refuses: reported as refused, the rest of the set rolled back
  const double pitBefore = readP(*link, "ATC_RAT_PIT_P"), flte = readP(*link, "ATC_RAT_YAW_FLTE");
  ap = sitlReview(*g, *link,
                  Json(Json::Array{Json::obj({{"name", "ATC_RAT_PIT_P"}, {"from", pitBefore}, {"value", core::jsRoundPrecision(pitBefore * 1.1, 6)}}),
                                   Json::obj({{"name", "ATC_RAT_YAW_FLTE"}, {"from", flte}, {"value", 3}})}));
  res = valueOf(g->write({"ATC_RAT_PIT_P", "ATC_RAT_YAW_FLTE"}, ap).future());
  step(statuses(res) == "rolled back,refused",
       "refused write: ATC_RAT_PIT_P " + res[0]["status"].strOr("") + ", ATC_RAT_YAW_FLTE " + res[1]["status"].strOr("") + " (" + res[1]["text"].strOr("") + ")");
  step(engine::sameF32(readP(*link, "ATC_RAT_PIT_P"), pitBefore) && engine::sameF32(readP(*link, "ATC_RAT_YAW_FLTE"), flte),
       "after the refusal the vehicle holds the values from before (read fresh)");
  step(matches(g->info(), "refused .*went back to its value from before"), "the summary says so: " + g->info());

  // 4 a whole-number parameter: the value is cast as ArduPilot stores it, then verified
  const double gf = readP(*link, "INS_GYRO_FILTER");
  const Json c1 = valueOf(app::verifiedSet(*link, "INS_GYRO_FILTER", gf + 0.6).future());
  step(c1["status"].strOr("") == "verified" && c1["sent"].numOr(NAN) == gf + 1 && has(c1["note"].strOr(""), "rounded"),
       "INS_GYRO_FILTER " + core::jsToString(gf + 0.6) + " sent as " + engine::jsText(c1["sent"]) + " (type " + engine::jsText(c1["type"]) + ") and verified");
  const Json c2 = valueOf(app::verifiedSet(*link, "INS_GYRO_FILTER", gf).future());
  step(c2["status"].strOr("") == "verified" && engine::sameF32(readP(*link, "INS_GYRO_FILTER"), gf), "INS_GYRO_FILTER back to " + core::jsToString(gf) + " (verified, read back)");

  // 5 the notch, a value that acts only after a reboot, the reboot workflow; Undo and the second reboot
  const std::vector<std::string> nn{"INS_HNTCH_ENABLE", "INS_HNTCH_FREQ", "INS_HNTCH_BW", "INS_HNTCH_REF"};
  const std::map<std::string, double> target{{"INS_HNTCH_ENABLE", 1}, {"INS_HNTCH_FREQ", 60}, {"INS_HNTCH_BW", 30}, {"INS_HNTCH_REF", 0.3}};
  std::map<std::string, double> nb;
  for (const auto& n : nn) nb[n] = readP(*link, n);
  Json nrows = Json::array();
  for (const auto& n : nn) nrows.push(Json::obj({{"name", n}, {"from", nb[n]}, {"value", target.at(n)}, {"group", "notch"}}));
  ap = sitlReview(*g, *link, nrows);
  res = valueOf(g->write(nn, ap).future());
  std::string st;
  for (const auto& w : res.arr()) st += (st.empty() ? "" : ", ") + w["name"].strOr("") + " " + w["status"].strOr("");
  step(statuses(res) == "verified,verified,verified,verified", "notch written and read back (" + st + ")");
  step(fieldList(g->rebootPending(), "name") == "INS_HNTCH_REF", "INS_HNTCH_REF waits for a reboot: " + g->info());
  const int gen0 = link->generation();
  const double tr = rt::Loop::now();
  const Json rb = rebootF(*g);
  step(rb["ok"].asBool() && g->rebootPending().size() == 0, "rebooted in " + fmtv((rt::Loop::now() - tr) / 1000) + " s, every value read back after the reboot (connection #" +
                                                                std::to_string(gen0) + " -> #" + std::to_string(link->generation()) + "): " + g->info());
  std::map<std::string, double> fresh;
  auto freshText = [&]() {
    std::string s;
    for (const auto& n : nn) {
      fresh[n] = readP(*link, n);
      s += (s.empty() ? "" : ", ") + n + " " + fmtv(fresh[n]);
    }
    return s;
  };
  std::string ft = freshText();
  bool allTarget = true;
  for (const auto& n : nn)
    if (!engine::sameF32(fresh[n], target.at(n))) allTarget = false;
  step(allTarget, "read fresh after the reboot: " + ft);
  valueOf(g->undo().future());
  step(fieldList(g->writes(), "undo") == "undone,undone,undone,undone" && fieldList(g->rebootPending(), "name") == "INS_HNTCH_REF",
       "notch Undo: written back and read back; REF waits for a reboot: " + g->info());
  const Json rb2 = rebootF(*g);
  ft = freshText();
  bool allBefore = rb2["ok"].asBool();
  for (const auto& n : nn)
    if (!engine::sameF32(fresh[n], nb[n])) allBefore = false;
  step(allBefore, "second reboot: the notch values from before read back (" + ft + ")");
  std::set<std::string> kinds;
  for (const auto& e : events(g->audit())) kinds.insert(e["kind"].strOr(""));
  bool allKinds = true;
  std::string kl;
  for (const char* k : {"identity", "proposal", "approval", "txn", "write", "rollback", "reboot"})
    if (!kinds.count(k)) allKinds = false;
  for (const auto& k : kinds) kl += (kl.empty() ? "" : ", ") + k;
  step(allKinds, "journal " + app::pathStr(std::filesystem::path(g->audit().file()).filename()) + ": " + kl);
  link->disconnect();
  runFor(100);
}
