#include "paramwrite.hpp"

#include <cmath>
#include <map>
#include <optional>
#include <vector>

#include "aptuner/core/jsnum.hpp"
#include "aptuner/engine/identity.hpp"

namespace aptuner::app {

using engine::sameF32;
using rt::Loop;

namespace {

const std::map<int, std::pair<double, double>>& intRange() {
  static const std::map<int, std::pair<double, double>> r{{1, {0, 255}}, {2, {-128, 127}}, {3, {0, 65535}}, {4, {-32768, 32767}}, {5, {0, 4294967295.0}}, {6, {-2147483648.0, 2147483647.0}}};
  return r;
}
const char* typeName(int t) {
  switch (t) {
    case 1: return "UINT8";
    case 2: return "INT8";
    case 3: return "UINT16";
    case 4: return "INT16";
    case 5: return "UINT32";
    case 6: return "INT32";
    case 7: return "UINT64";
    case 8: return "INT64";
    case 9: return "REAL32";
    case 10: return "REAL64";
    default: return "?";
  }
}
double roundHalfAway(double v) { return v < 0 ? -std::floor(-v + 0.5) : std::floor(v + 0.5); }
/** +v.toPrecision(6) as text (JavaScript prints a missing value as "undefined") */
std::string num(const Json& v) {
  if (v.isNumber()) return std::isfinite(v.num()) ? core::jsToString(core::jsRoundPrecision(v.num(), 6)) : core::jsToString(v.num());
  if (v.isNull()) return "undefined";
  return engine::jsText(v);
}
std::string num(double v) { return num(Json(v)); }
std::string str(const Json& v) { return v.isString() ? v.str() : ""; }

int txnSeq = 0;

/** {txn, ...ctx, ...data} as the JavaScript Object.assign builds it */
Json event(const Json& head, const Json& ctx, const Json& data) {
  Json e = head;
  if (ctx.isObject())
    for (const auto& [k, v] : ctx.items()) e.set(k, v);
  if (data.isObject())
    for (const auto& [k, v] : data.items()) e.set(k, v);
  return e;
}

}  // namespace

Cast castForType(double value, int type) {
  Cast c;
  if (!std::isfinite(value)) {
    c.error = "the value is not a number";
    return c;
  }
  const auto it = intRange().find(type);
  if (it != intRange().end()) {
    const double r = roundHalfAway(value);
    if (r < it->second.first || r > it->second.second) {
      c.error = num(value) + " does not fit a " + typeName(type) + " parameter (" + core::jsToString(it->second.first) + " to " + core::jsToString(it->second.second) + ")";
      return c;
    }
    c.ok = true;
    c.value = r;
    if (r != value) c.note = "rounded to " + core::jsToString(r) + " (a whole-number parameter)";
    return c;
  }
  if (type == 7 || type == 8) {
    c.error = "64-bit integer parameters are not written by AP Tuner";
    return c;
  }
  const double f = static_cast<double>(static_cast<float>(value));
  if (!std::isfinite(f)) {
    c.error = core::jsToString(value) + " does not fit a float parameter";
    return c;
  }
  c.ok = true;
  c.value = f;
  return c;
}

bool changedStatus(const std::string& s) { return s == "verified" || s == "different" || s == "uncertain"; }

std::string writeText(const Json& r) {
  const std::string name = str(r["name"]), s = str(r["status"]);
  if (s == "verified") return name + " = " + num(r["holds"]) + " (read back)";
  if (s == "unchanged") return name + " already " + num(r["holds"]);
  if (s == "refused") return name + ": the autopilot refused " + num(r["sent"]) + " and kept " + num(r["holds"]);
  if (s == "different") return name + ": asked for " + num(r["sent"]) + ", the autopilot holds " + num(r["holds"]);
  if (s == "not-applied") return name + ": no answer to the write; the autopilot still holds " + num(r["holds"]);
  if (s == "uncertain") return name + ": the value on the autopilot is not known (" + (r["error"].isString() && !r["error"].str().empty() ? r["error"].str() : "no answer") + ")";
  if (s == "missing") return name + " does not exist on this autopilot";
  if (s == "invalid") return name + ": " + str(r["error"]);
  if (s == "cancelled") return name + ": not written (stopped)";
  if (s == "not-sent") return name + ": not written (" + str(r["error"]) + ")";
  return name + ": " + s;
}

std::string txnText(const Json& t) {
  if (t["ok"].asBool()) return core::jsToString(static_cast<double>(t["rows"].size())) + " value(s) written and read back";
  if (t["aborted"].isString()) return "nothing written: " + t["aborted"].str();
  if (t["cancelled"].asBool()) {
    std::string names;
    for (const auto& r : t["rows"].arr())
      if (str(r["status"]) == "verified") names += (names.empty() ? "" : ", ") + str(r["name"]);
    return "stopped: " + names + " written before the stop";
  }
  const Json& failed = t["rows"][t["rows"].size() - 1];
  const std::string head = writeText(failed);
  if (t["rolledBack"].asBool()) return head + ". Everything else written in this set went back to its value from before (read back)";
  std::string list;
  for (const auto& u : t["uncertain"].arr())
    list += (list.empty() ? "" : ", ") + str(u["name"]) + " (" + (u["holds"].isNumber() ? "holds " + num(u["holds"]) : std::string("value not known")) + ")";
  return head + ". Uncertain parameters: " + list + ": check them before flying";
}

rt::Task<Json> verifiedSet(io::Link& link, std::string name, double value, VerifyOpts o) {
  const double t0 = Loop::now();
  Json res = Json::obj({{"name", name}, {"requested", value}, {"sent", Json()}, {"type", Json()}, {"before", o.before}, {"echo", Json()}, {"readBack", Json()},
                        {"holds", Json()}, {"status", "uncertain"}, {"ok", false}, {"error", Json()}, {"note", Json()}, {"cancelled", false}, {"attempts", 0}, {"ms", 0}});
  auto finish = [&res, t0] {
    res.set("ms", Loop::now() - t0);
    res.set("ok", res["status"].str() == "verified");
    return res;
  };
  // the type comes from the vehicle's own answer: read the parameter first when the app has not seen it
  if (!link.params().count(name)) {
    std::string code;
    bool failed = false;
    try {
      co_await link.requestParam(name, o.timeoutMs, o.readRetries);
    } catch (const io::LinkError& e) {
      failed = true;
      code = e.code();
      res.set("error", std::string(e.what()));
    } catch (const std::exception& e) {
      failed = true;
      res.set("error", std::string(e.what()));
    }
    if (failed) {
      if (code == "link-changed" || code == "rebooted") {
        res.set("status", "not-sent");
      } else {
        res.set("status", "missing");
        res.set("error", "no answer when it was read");
      }
      co_return finish();
    }
  }
  const auto known = link.params().find(name);
  if (known == link.params().end()) {
    res.set("status", "not-sent");
    res.set("error", "the connection changed");
    co_return finish();
  }
  const int type = known->second.type;
  res.set("type", type);
  if (res["before"].isNull()) res.set("before", known->second.value);
  const Json before = res["before"];
  const Cast c = castForType(value, type);
  if (!c.ok) {
    res.set("status", "invalid");
    res.set("error", c.error);
    co_return finish();
  }
  res.set("sent", c.value);
  if (!c.note.empty()) res.set("note", c.note);
  bool err = false;
  std::string errCode, errMsg;
  int errAttempts = 0;
  std::optional<double> echo;
  try {
    echo = co_await link.setParam(name, c.value, o.timeoutMs, o.retries, o.keepGoing);
    res.set("echo", *echo);
    res.set("attempts", 1);
  } catch (const io::LinkError& e) {
    err = true;
    errCode = e.code();
    errMsg = e.what();
    errAttempts = e.attempts;
  } catch (const std::exception& e) {
    err = true;
    errMsg = e.what();
  }
  if (err) res.set("attempts", errAttempts);
  if (err && errCode == "cancelled" && !errAttempts) {
    res.set("status", "cancelled");
    res.set("cancelled", true);
    res.set("error", "stopped before it was sent");
    co_return finish();
  }
  if (err && (errCode == "link-changed" || errCode == "rebooted")) {
    // it may have reached the vehicle, but it cannot be read back on another connection or during a reboot
    res.set("status", errAttempts ? "uncertain" : "not-sent");
    res.set("error", errMsg);
    co_return finish();
  }
  if (err && errCode == "cancelled") res.set("cancelled", true);
  if (!err && sameF32(*echo, c.value)) {
    res.set("status", "verified");
    res.set("holds", *echo);
    co_return finish();
  }
  // no echo, or an echo with another value (a refusal, clamping, a late answer to something else): ask once more
  std::optional<double> rb;
  std::string readCode;
  try {
    rb = co_await link.requestParam(name, o.timeoutMs, o.readRetries);
  } catch (const io::LinkError& e) {
    readCode = e.code();
  } catch (const std::exception&) {
  }
  if (rb) {
    res.set("readBack", *rb);
    res.set("holds", *rb);
    if (sameF32(*rb, c.value))
      res.set("status", "verified");
    else if (!before.isNull() && sameF32(*rb, before.num()))
      res.set("status", err ? "not-applied" : "refused");
    else
      res.set("status", "different");
    if (res["status"].str() != "verified" && res["error"].isNull()) res.set("error", err ? Json(errMsg) : Json());
  } else {
    res.set("status", "uncertain");
    res.set("error", (err ? errMsg : "the echo showed " + num(res["echo"])) + "; reading it back got no answer" + (readCode == "link-changed" ? " (the connection changed)" : ""));
  }
  co_return finish();
}

rt::Task<Json> runTransaction(io::Link& link, Json items, TxnOpts o) {
  const int txn = o.ctx["txn"].isNumber() ? static_cast<int>(o.ctx["txn"].num()) : ++txnSeq;
  Json out = Json::obj({{"txn", txn}, {"ok", false}, {"aborted", Json()}, {"stale", Json::array()}, {"cancelled", false}, {"rows", Json::array()},
                        {"rollback", Json::array()}, {"rolledBack", false}, {"uncertain", Json::array()}});
  auto J = [&o, txn](const std::string& kind, const Json& data) {
    if (o.journal) o.journal(kind, event(Json::obj({{"txn", txn}}), o.ctx, data));
  };
  const int gen0 = link.generation();
  // 1. a fresh read of every value
  struct Pre {
    Json it;
    std::optional<double> before;
    bool failed = false;
    std::string code, msg;
  };
  std::vector<Pre> pre;
  for (const auto& it : items.arr()) {
    Pre p;
    p.it = it;
    try {
      p.before = co_await link.requestParam(it["name"].str(), o.timeoutMs, o.readRetries);
    } catch (const io::LinkError& e) {
      p.failed = true;
      p.code = e.code();
      p.msg = e.what();
    } catch (const std::exception& e) {
      p.failed = true;
      p.msg = e.what();
    }
    const bool stop = p.failed;
    pre.push_back(std::move(p));
    if (stop) break;
  }
  for (const auto& p : pre)
    if (!p.before) {
      const std::string n = p.it["name"].str();
      out.set("aborted", p.code == "timeout" ? n + " does not exist on this autopilot (no answer when it was read)" : "reading " + n + " failed: " + (p.failed ? p.msg : "no answer"));
      out.set("missing", p.code == "timeout" ? Json(n) : Json());
      J("txn", Json::obj({{"phase", "aborted"}, {"reason", out["aborted"]}}));
      co_return out;
    }
  Json stale = Json::array();
  for (const auto& p : pre)
    if (p.it["expect"].isNumber() && !sameF32(*p.before, p.it["expect"].num()))
      stale.push(Json::obj({{"name", p.it["name"]}, {"expect", p.it["expect"]}, {"holds", *p.before}}));
  out.set("stale", stale);
  if (stale.size()) {
    std::string list;
    for (const auto& s : stale.arr()) list += (list.empty() ? "" : "; ") + s["name"].str() + " is " + num(s["holds"]) + ", was " + num(s["expect"]);
    out.set("aborted", "changed on the autopilot since the values were shown: " + list);
    J("txn", Json::obj({{"phase", "aborted"}, {"reason", out["aborted"]}, {"stale", stale}}));
    co_return out;
  }
  if (o.afterPreRead) {
    Json before = Json::array();
    for (const auto& p : pre) before.push(Json::obj({{"name", p.it["name"]}, {"before", *p.before}}));
    std::string stop;
    try {
      o.afterPreRead(before);
    } catch (const std::exception& e) {
      stop = e.what();
    }
    if (!stop.empty()) {
      out.set("aborted", stop);
      J("txn", Json::obj({{"phase", "aborted"}, {"reason", stop}}));
      co_return out;
    }
  }
  {
    Json list = Json::array();
    for (std::size_t i = 0; i < items.size(); i++) {
      const Json& it = items[i];
      list.push(Json::obj({{"name", it["name"]}, {"to", it["to"]}, {"before", *pre[i].before}, {"expect", it["expect"]}, {"reason", it["reason"]}}));
    }
    J("txn", Json::obj({{"phase", "begin"}, {"items", list}}));
  }
  // 2. write in order, each verified; a value the vehicle already holds is not written again
  Json rows = Json::array();
  std::optional<Json> failed;
  bool cancelled = false;
  for (std::size_t i = 0; i < items.size(); i++) {
    const Json& it = items[i];
    const std::string name = it["name"].str();
    const double before = *pre[i].before;
    const auto known = link.params().find(name);
    const Cast c = known != link.params().end() ? castForType(it["to"].numOr(NAN), known->second.type) : Cast{};
    if (c.ok && sameF32(before, c.value)) {
      const Json r = Json::obj({{"name", name}, {"requested", it["to"]}, {"sent", Json()}, {"type", known->second.type}, {"before", before}, {"holds", before},
                                {"status", "unchanged"}, {"ok", true}, {"error", Json()}, {"note", Json()}, {"reason", it["reason"]}});
      rows.push(r);
      J("write", r);
      continue;
    }
    VerifyOpts vo;
    vo.before = before;
    vo.keepGoing = o.keepGoing;
    vo.timeoutMs = o.timeoutMs;
    vo.retries = o.retries;
    vo.readRetries = o.readRetries;
    Json r = co_await verifiedSet(link, name, it["to"].numOr(NAN), vo);
    r.set("reason", it["reason"]);
    rows.push(r);
    J("write", r);
    if (r["cancelled"].asBool() && !changedStatus(r["status"].str())) {
      cancelled = true;
      break;
    }
    if (!r["ok"].asBool()) {
      failed = r;
      break;
    }
    if (r["cancelled"].asBool()) {
      cancelled = true;
      break;
    }
  }
  out.set("rows", rows);
  if (cancelled) {
    out.set("cancelled", true);
    J("txn", Json::obj({{"phase", "end"}, {"ok", false}, {"cancelled", true}}));
    co_return out;
  }
  if (!failed) {
    out.set("ok", true);
    J("txn", Json::obj({{"phase", "end"}, {"ok", true}}));
    co_return out;
  }
  // 3. roll back what may have changed, newest first, each verified; never across another connection
  Json rollback = Json::array(), uncertain = Json::array();
  for (std::size_t k = rows.size(); k-- > 0;) {
    const Json r = rows[k];
    const std::string st = r["status"].str();
    if (!changedStatus(st) || (st == "verified" && sameF32(r["holds"], r["before"]))) continue;
    if (link.generation() != gen0) {
      uncertain.push(Json::obj({{"name", r["name"]}, {"holds", r["holds"]}, {"why", "the connection changed during the write: check it after reconnecting"}}));
      continue;
    }
    VerifyOpts vo;
    vo.before = r["holds"];
    vo.timeoutMs = o.timeoutMs;
    vo.retries = o.retries;
    vo.readRetries = o.readRetries;
    Json rb = co_await verifiedSet(link, r["name"].str(), r["before"].num(), vo);
    rb.set("rollbackOf", st);
    rollback.push(rb);
    J("rollback", rb);
    if (!rb["ok"].asBool()) uncertain.push(Json::obj({{"name", r["name"]}, {"holds", rb["holds"]}, {"why", writeText(rb)}}));
  }
  out.set("rollback", rollback);
  out.set("uncertain", uncertain);
  out.set("rolledBack", uncertain.size() == 0);
  J("txn", Json::obj({{"phase", "end"}, {"ok", false}, {"failed", (*failed)["name"]}, {"rolledBack", uncertain.size() == 0}, {"uncertain", uncertain}}));
  co_return out;
}

rt::Task<Json> rebootAndVerify(io::Link& link, Json expect, RebootOpts o) {
  auto J = [&o](const std::string& kind, const Json& data) {
    if (o.journal) o.journal(kind, event(Json::object(), o.ctx, data));
  };
  auto info = [&o](const std::string& s) {
    if (o.onInfo) o.onInfo(s);
  };
  auto fail = [&J](const std::string& reason, const Json& extra = Json::object()) {
    J("reboot", event(Json::obj({{"phase", "failed"}, {"reason", reason}}), Json::object(), extra));
    return event(Json::obj({{"ok", false}, {"reason", reason}, {"rebooted", false}, {"rows", Json::array()}}), Json::object(), extra);
  };
  if (!link.alive()) co_return fail("the autopilot is not connected");
  if (link.vehicle().isObject() && link.vehicle()["armed"].asBool()) co_return fail("refusing to reboot an armed vehicle");
  const Json id0 = link.identity();
  const Json opts0 = link.opts();
  const int boot0 = link.bootCount(), gen0 = link.generation();
  std::map<std::string, int> types;
  for (const auto& e : expect.arr()) {
    const auto it = link.params().find(e["name"].str());
    if (it != link.params().end()) types[e["name"].str()] = it->second.type;
  }
  J("reboot", Json::obj({{"phase", "request"}, {"identity", id0}, {"expect", expect}}));
  const double tCmd = Loop::now();
  Json ack;
  std::string ackErr;
  try {
    ack = co_await link.commandAck(246, {1}, 3000);
  } catch (const std::exception& e) {
    ackErr = e.what();
  }
  if (!ackErr.empty()) co_return fail("no answer to the reboot command: " + ackErr);
  if (ack["result"].numOr(-1) != 0) co_return fail("the autopilot refused to reboot (result " + engine::jsText(ack["result"]) + ")");
  info("rebooting: the autopilot restarts, this takes a few seconds");
  const double t0 = Loop::now();
  bool gone = false;
  double lastTry = 0;
  while (Loop::now() - t0 < o.timeoutMs) {
    co_await rt::sleep(200);
    if (!gone && (link.generation() != gen0 || link.bootCount() > boot0 || !link.connected() || Loop::now() - link.lastHeartbeatAt() > 1500)) {
      gone = true;
      info("the autopilot is restarting");
    }
    // USB and TCP close on a reboot: open the same connection again until it answers
    if (gone && !link.connected() && opts0.isObject() && opts0["kind"].strOr("") != "test" && Loop::now() - lastTry > 2500) {
      lastTry = Loop::now();
      link.connect(opts0);
    }
    if (gone && link.alive() && Loop::now() - link.lastHeartbeatAt() < 1500) break;
  }
  if (!gone) co_return fail("the autopilot did not restart (its heartbeat never stopped)");
  if (!link.alive()) co_return fail("the autopilot did not come back within " + core::jsToString(std::round(o.timeoutMs / 1000)) + " s: reconnect it, then check the values");
  // proof of the restart: the boot clock went back, or the time since boot is shorter than the time since the command
  const double tBack = Loop::now();
  link.setMessageRate(2, 2);   // SYSTEM_TIME: the boot clock (the rates asked for before the reboot are gone)
  while (Loop::now() - tBack < 3000 && !(link.bootCount() > boot0 && link.generation() == gen0) && !(link.bootMs() && link.bootAt() >= tBack)) co_await rt::sleep(100);
  std::optional<double> uptime;
  if (link.bootMs() && link.bootAt() >= tBack) uptime = *link.bootMs() + (Loop::now() - link.bootAt());
  const bool proven = (link.generation() == gen0 && link.bootCount() > boot0) || (uptime && *uptime < Loop::now() - tCmd + 2000);
  if (!proven)
    co_return fail(!uptime ? "the restart could not be confirmed (the autopilot sends no time since boot): power it off and on, then check the values"
                           : "the autopilot did not restart (it has been running for " + core::jsToString(std::round(*uptime / 1000)) + " s)");
  // who answers now: AUTOPILOT_VERSION again (a board with another UID is another vehicle)
  link.refreshAutopilot();
  co_await link.waitAutopilot(3000);
  const Json id1 = link.identity();
  if (!engine::sameVehicle(id0, id1)) co_return fail("a different vehicle answered after the reboot: " + engine::identityText(id1), Json::obj({{"identity", id1}}));
  info("back after the reboot: reading the values again");
  Json rows = Json::array();
  bool allOk = true;
  std::string bad;
  for (const auto& e : expect.arr()) {
    const std::string name = e["name"].str();
    std::optional<double> holds;
    try {
      holds = co_await link.requestParam(name, 1500, 3);
    } catch (const std::exception&) {
    }
    const auto known = link.params().find(name);
    const int type = known != link.params().end() ? known->second.type : (types.count(name) ? types[name] : 9);
    const Cast c = castForType(e["value"].numOr(NAN), type);
    const bool ok = c.ok && holds && sameF32(*holds, c.value);
    rows.push(Json::obj({{"name", name}, {"expect", c.ok ? Json(c.value) : e["value"]}, {"holds", holds ? Json(*holds) : Json()}, {"ok", ok}}));
    if (!ok) {
      allOk = false;
      bad += (bad.empty() ? "" : "; ") + (holds ? name + " holds " + num(*holds) + ", expected " + num(rows[rows.size() - 1]["expect"]) : name + " did not answer");
    }
  }
  J("reboot", Json::obj({{"phase", "verified"}, {"ok", allOk}, {"rows", rows}, {"identity", id1}}));
  co_return Json::obj({{"ok", allOk}, {"reason", allOk ? Json() : Json("after the reboot: " + bad)}, {"rebooted", true}, {"rows", rows}, {"identity", id1}});
}

}  // namespace aptuner::app
