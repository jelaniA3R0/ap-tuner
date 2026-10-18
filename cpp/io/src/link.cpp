#include "aptuner/io/link.hpp"

#include <cmath>
#include <cstdio>

#include "aptuner/core/cx.hpp"
#include "aptuner/core/jsnum.hpp"

namespace aptuner::io {

using rt::Loop;

namespace {
constexpr int kArduPilot = 3;   // MAV_AUTOPILOT_ARDUPILOTMEGA
// heartbeat types that are never the vehicle: antenna tracker, ground station, onboard computer, gimbal, ADS-B, camera
const std::set<int> kNotAVehicle{5, 6, 18, 26, 27, 30};

std::string typeName(int type) {
  const char* n = mavlink::enumName("MAV_TYPE", type);
  return n ? std::string(n) : std::to_string(type);
}
std::string fwTypeName(unsigned type) {
  switch (type) {
    case 0: return "dev";
    case 64: return "alpha";
    case 128: return "beta";
    case 192: return "rc";
    case 255: return "official";
    default: return std::to_string(type);
  }
}
std::exception_ptr linkError(const std::string& msg, const std::string& code, int attempts) {
  return std::make_exception_ptr(LinkError(msg, code, attempts));
}
}  // namespace

struct Link::ParamReq {
  std::string name;
  bool isSet = false;
  double value = 0;
  int type = 9;
  double timeoutMs = 1500;
  int retries = 4;
  int gen = 0;
  std::function<bool()> keepGoing;
  rt::Promise<double> p;
};

Json parseAutopilotVersion(const Json& f) {
  const auto v = static_cast<std::uint32_t>(static_cast<long long>(f["flight_sw_version"].numOr(0)));
  const unsigned type = v & 0xff;
  std::string fw;
  if (v) {
    fw = std::to_string((v >> 24) & 0xff) + "." + std::to_string((v >> 16) & 0xff) + "." + std::to_string((v >> 8) & 0xff);
    if (type != 255) fw += "-" + fwTypeName(type);
  }
  // uid2 (18 bytes) supersedes uid; both 0 when the board has no hardware id (SITL)
  std::string uid;
  bool any = false;
  for (const auto& b : f["uid2"].isArray() ? f["uid2"].arr() : Json::Array{}) {
    const unsigned x = static_cast<unsigned>(static_cast<long long>(b.numOr(0))) & 0xff;
    char h[3];
    std::snprintf(h, sizeof h, "%02x", x);
    uid += h;
    if (x) any = true;
  }
  if (!any) {
    const std::string h = f["uid_hex"].strOr("");
    uid = h.find_first_of("123456789abcdef") != std::string::npos ? h : "";
  }
  std::string git;
  for (const auto& b : f["flight_custom_version"].isArray() ? f["flight_custom_version"].arr() : Json::Array{}) {
    const double x = b.numOr(0);
    if (x > 32 && x < 127) git.push_back(static_cast<char>(static_cast<int>(x)));
  }
  return Json::obj({{"fw", fw.empty() ? Json() : Json(fw)}, {"fwRaw", static_cast<double>(v)}, {"gitHash", git.empty() ? Json() : Json(git)},
                    {"board", static_cast<double>(static_cast<std::uint32_t>(static_cast<long long>(f["board_version"].numOr(0))))},
                    {"vendor", f["vendor_id"].numOr(0)}, {"product", f["product_id"].numOr(0)}, {"uid", uid.empty() ? Json() : Json(uid)}});
}

Link::Link() { rateWinT_ = Loop::now(); }

Link::~Link() {
  disconnect();
  if (apTimer_) Loop::main().clear(apTimer_);
}

std::string modeName(int type, long long mode) {
  const bool copter = type == 2 || type == 13 || type == 14 || type == 15 || type == 29 || type == 4 || type == 3;
  const char* n = mavlink::enumName(copter ? "COPTER_MODE" : "PLANE_MODE", mode);
  if (!n) return std::to_string(mode);
  std::string s = n;
  for (const char* pre : {"COPTER_MODE_", "PLANE_MODE_"})
    if (s.rfind(pre, 0) == 0) return s.substr(std::string(pre).size());
  return s;
}

void Link::connect(const Json& opts) {
  disconnect();
  newGeneration("the connection changed");
  opts_ = opts;
  serialOpened_ = false;
  parser_ = mavlink::Parser();
  const std::string kind = opts["kind"].strOr("tcp");
  // a closed transport delivers nothing more (transport.cpp), so late events of an old one never reach this connection
  TransportCallbacks cb;
  cb.onData = [this](const std::vector<std::uint8_t>& d) { rx(d); };
  cb.onStatus = [this](const std::string& s) { emit("status", Json(s)); };
  cb.onClose = [this](int) {
    connected_ = false;
    stopHeartbeat();
  };
  if (kind == "tcp") {
    const std::string host = opts["host"].strOr("127.0.0.1");
    const int port = static_cast<int>(opts["port"].numOr(5760));
    description_ = "TCP " + host + ":" + std::to_string(port);
    cb.onOpen = [this] {
      connected_ = true;
      startHeartbeat();
    };
    transport_ = openTcp(host, port, std::move(cb));
  } else if (kind == "serial") {
    const std::string port = opts["port"].strOr("");
    const int baud = static_cast<int>(opts["baud"].numOr(115200));
    description_ = port + " @ " + std::to_string(baud);
    cb.onOpen = [this] {
      serialOpened_ = true;
      connected_ = true;
      emit("status", Json("serial open " + description_));
      startHeartbeat();
    };
    transport_ = openSerial(port, baud, std::move(cb));
  } else {
    const bool listen = kind == "udpin";
    const int port = static_cast<int>(opts["port"].numOr(14550));
    const std::string host = opts["host"].strOr(listen ? "0.0.0.0" : "127.0.0.1");
    description_ = listen ? "UDP listen " + std::to_string(port) : "UDP " + host + ":" + std::to_string(port);
    cb.onOpen = [this] {
      connected_ = true;
      startHeartbeat();
    };
    transport_ = openUdp(host, port, listen, std::move(cb));
  }
}

void Link::disconnect() {
  stopHeartbeat();
  if (transport_) {
    transport_->close();
    transport_.reset();
  }
  connected_ = false;
  opts_ = Json();
  rejectPending("the connection was closed");
}

void Link::attachTransport(std::unique_ptr<Transport> t, const std::string& description) {
  disconnect();
  newGeneration("the connection changed");
  opts_ = Json::obj({{"kind", "test"}});
  serialOpened_ = false;
  parser_ = mavlink::Parser();
  description_ = description;
  transport_ = std::move(t);
  connected_ = true;
  startHeartbeat();
}

void Link::newGeneration(const std::string& why) {
  generation_++;
  rejectPending(why);
  params_.clear();
  pinned_ = false;
  targetSys_ = 1;
  targetComp_ = 1;
  lastHeartbeatAt_ = 0;
  heartbeat_ = Json();
  vehicle_ = Json();
  autopilot_ = Json();
  prevAutopilot_ = Json();
  if (apTimer_) Loop::main().clear(apTimer_);
  apTimer_ = 0;
  bootMs_.reset();
  bootAt_ = 0;
  bootCount_ = 0;
  others_.clear();
  warned_.clear();
  ignored_ = Json();
}

void Link::vehicleChanged(const std::string& why) {
  generation_++;
  rejectPending("another vehicle answered (" + why + ")");
  params_.clear();
  autopilot_ = Json();
  prevAutopilot_ = Json();
  bootMs_.reset();
  bootAt_ = 0;
  bootCount_ = 0;
  emit("status", Json("another vehicle: " + why));
  changedWhy_ = why;
}

void Link::rejectPending(const std::string& why, const std::string& code) {
  auto pend = std::move(pending_);
  pending_.clear();
  for (auto& [name, ws] : pend)
    for (auto& w : ws) {
      Loop::main().clear(w.timer);
      w.p.reject(linkError(name + ": " + why, code, w.attempt + 1));
    }
  const auto cmds = pendingCmds_;
  for (const auto& [id, cancel] : cmds) cancel(why, code);
}

void Link::startHeartbeat() {
  stopHeartbeat();
  const Json hb = Json::obj({{"type", 6}, {"autopilot", 8}, {"base_mode", 0}, {"custom_mode", 0}, {"system_status", 4}, {"mavlink_version", 3}});
  hbTimer_ = Loop::main().setInterval(1000, [this, hb] { send("HEARTBEAT", hb); });
  send("HEARTBEAT", hb);
}

void Link::stopHeartbeat() {
  if (hbTimer_) Loop::main().clear(hbTimer_);
  hbTimer_ = 0;
}

void Link::rx(const std::vector<std::uint8_t>& data) {
  stats_.bytes += data.size();
  std::vector<mavlink::Message> msgs = parser_.feed(data.data(), data.size());
  const double now = Loop::now();
  if (now - rateWinT_ >= 2000) {
    const double dt = (now - rateWinT_) / 1000;
    msgRates_ = Json::object();
    for (const auto& [k, v] : rateCounts_) msgRates_.set(k, core::jsRoundFixed(v / dt, 1));
    rateCounts_.clear();
    rateWinT_ = now;
  }
  for (auto& m : msgs) {
    stats_.rx++;
    rateCounts_[m.name]++;
    const std::string name = m.name;
    if (name == "HEARTBEAT") {
      onHeartbeat(m, now);
      continue;
    }
    // only the target speaks: other systems and components (another vehicle, a gimbal, a ground station) are dropped
    if (!pinned_ || m.sysid != targetSys_ || m.compid != targetComp_) {
      stats_.foreign++;
      continue;
    }
    const Json& f = m.fields;
    if (f.has("time_boot_ms")) bootClock(f["time_boot_ms"].numOr(0), now);
    if (name == "PARAM_VALUE") {
      const std::string id = f["param_id"].strOr("");
      const double value = f["param_value"].numOr(0);
      ParamInfo pi;
      pi.value = value;
      pi.type = static_cast<int>(f["param_type"].numOr(9));
      pi.index = static_cast<int>(f["param_index"].numOr(-1));
      pi.count = static_cast<int>(f["param_count"].numOr(0));
      pi.ts = now;
      params_[id] = pi;
      auto it = pending_.find(id);
      if (it != pending_.end()) {
        auto waiters = std::move(it->second);
        pending_.erase(it);
        for (auto& w : waiters) {
          Loop::main().clear(w.timer);
          w.p.resolve(value);
        }
      }
      emit("param", Json::obj({{"name", id}, {"value", value}, {"type", pi.type}}));
    } else if (name == "AUTOPILOT_VERSION") {
      onAutopilotVersion(f);
    }
    if (has("message"))
      emit("message", Json::obj({{"id", m.id}, {"name", name}, {"sysid", m.sysid}, {"compid", m.compid}, {"seq", m.seq}, {"fields", m.fields}}));
    emit(name, m.fields);
  }
}

void Link::onHeartbeat(const mavlink::Message& m, double now) {
  const Json& f = m.fields;
  const int type = static_cast<int>(f["type"].numOr(0));
  const int ap = static_cast<int>(f["autopilot"].numOr(0));
  if (ap != kArduPilot || m.compid != 1 || kNotAVehicle.count(type)) {
    // a ground station, companion, tracker, gimbal or a non-ArduPilot autopilot: never the target
    if (type != 6 && m.compid == 1 && (ignored_.isNull() || static_cast<int>(ignored_["sysid"].numOr(-1)) != m.sysid)) {
      ignored_ = Json::obj({{"sysid", m.sysid}, {"compid", m.compid}, {"autopilot", ap}, {"type", type}});
      if (ap != kArduPilot && ap != 8)
        emit("status", Json("system id " + std::to_string(m.sysid) + " is not an ArduPilot autopilot (autopilot " + std::to_string(ap) +
                            "): AP Tuner only works with ArduPilot"));
    }
    return;
  }
  const bool isTarget = pinned_ && m.sysid == targetSys_ && m.compid == targetComp_;
  bool changed = false;
  if (!isTarget) {
    if (pinned_ && now - lastHeartbeatAt_ < 5000) {   // ours is alive: only report the other
      noteOther(m, now);
      return;
    }
    if (pinned_) {
      vehicleChanged("system id " + std::to_string(targetSys_) + " went silent and system id " + std::to_string(m.sysid) + " answered");
      changed = true;
    }
    pinned_ = true;
    targetSys_ = m.sysid;
    targetComp_ = m.compid;
    others_.erase(m.sysid);
    vehicle_ = Json();
    requestAutopilotVersion();
  } else if (vehicle_.isObject() && static_cast<int>(vehicle_["type"].numOr(-1)) != type) {
    // the same system id with another airframe type: a different vehicle behind the same link (e.g. the mirror)
    vehicleChanged("the vehicle type changed from " + vehicle_["typeName"].strOr("?") + " to " + typeName(type));
    changed = true;
    requestAutopilotVersion();
  }
  lastHeartbeatAt_ = now;
  heartbeat_ = f;
  const long long baseMode = static_cast<long long>(f["base_mode"].numOr(0));
  const long long mode = static_cast<long long>(f["custom_mode"].numOr(0));
  vehicle_ = Json::obj({{"type", type}, {"autopilot", ap}, {"mode", static_cast<double>(mode)}, {"armed", (baseMode & 128) != 0},
                        {"modeName", modeName(type, mode)}, {"typeName", typeName(type)}});
  if (changed) emit("vehicle-change", Json::obj({{"reason", changedWhy_}, {"generation", generation_}}));
  emit("heartbeat", vehicle_);
  if (has("message"))
    emit("message", Json::obj({{"id", m.id}, {"name", "HEARTBEAT"}, {"sysid", m.sysid}, {"compid", m.compid}, {"seq", m.seq}, {"fields", m.fields}}));
  emit("HEARTBEAT", f);
}

void Link::noteOther(const mavlink::Message& m, double now) {
  Other o;
  o.sysid = m.sysid;
  o.compid = m.compid;
  o.type = static_cast<int>(m.fields["type"].numOr(0));
  o.lastSeen = now;
  others_[m.sysid] = o;
  if (warned_.count(m.sysid)) return;
  warned_.insert(m.sysid);
  emit("status", Json("another vehicle is on this link (system id " + std::to_string(m.sysid) + ", " + typeName(o.type) +
                      "): the app stays with system id " + std::to_string(targetSys_)));
}

Json Link::othersSeen() const {
  const double now = Loop::now();
  Json out = Json::array();
  for (const auto& [id, o] : others_)
    if (now - o.lastSeen < 10000) out.push(Json::obj({{"sysid", o.sysid}, {"compid", o.compid}, {"type", o.type}}));
  return out;
}

void Link::bootClock(double ms, double now) {
  // the autopilot's time since boot going back (by more than 0.5 s: it only ever counts up while it runs) is a reboot
  if (bootMs_ && ms + 500 < *bootMs_) {
    bootCount_++;
    rejectPending("the autopilot rebooted", "rebooted");
    if (!autopilot_.isNull()) prevAutopilot_ = autopilot_;
    autopilot_ = Json();
    emit("status", Json("the autopilot rebooted"));
    emit("reboot", Json::obj({{"count", bootCount_}}));
    requestAutopilotVersion();
  }
  bootMs_ = ms;
  bootAt_ = now;
}

void Link::onAutopilotVersion(const Json& f) {
  const Json info = parseAutopilotVersion(f);
  const Json prev = !autopilot_.isNull() ? autopilot_ : prevAutopilot_;
  if (apTimer_) Loop::main().clear(apTimer_);
  apTimer_ = 0;
  if (prev.isObject() && prev["uid"].isString() && info["uid"].isString() && prev["uid"].str() != info["uid"].str()) {
    vehicleChanged("another board answered (UID " + info["uid"].str().substr(0, 24) + ")");
    autopilot_ = info;
    emit("vehicle-change", Json::obj({{"reason", changedWhy_}, {"generation", generation_}}));
  } else {
    autopilot_ = info;
  }
  prevAutopilot_ = Json();
  emit("autopilot", info);
}

void Link::requestAutopilotVersion() {
  if (apTimer_) Loop::main().clear(apTimer_);
  apTimer_ = 0;
  const int gen = generation_;
  auto n = std::make_shared<int>(0);
  // ask now and every second (REQUEST_MESSAGE 512, older firmware: REQUEST_AUTOPILOT_CAPABILITIES 520), six times at most
  auto ask = std::make_shared<std::function<void()>>();
  std::weak_ptr<std::function<void()>> weak = ask;
  *ask = [this, gen, n, weak] {
    apTimer_ = 0;
    if (generation_ != gen || !autopilot_.isNull() || !transport_ || !pinned_) return;
    if (*n % 2 == 0)
      commandLong(512, {148});
    else
      commandLong(520, {1});
    if (++*n < 6)
      if (auto again = weak.lock()) apTimer_ = Loop::main().setTimeout(1000, [again] { (*again)(); });
  };
  (*ask)();
}

void Link::refreshAutopilot() {
  if (!autopilot_.isNull()) prevAutopilot_ = autopilot_;
  autopilot_ = Json();
  requestAutopilotVersion();
}

rt::Task<Json> Link::waitAutopilot(double ms) {
  const double t0 = Loop::now();
  if (autopilot_.isNull() && pinned_ && !apTimer_) requestAutopilotVersion();
  while (autopilot_.isNull() && Loop::now() - t0 < ms) co_await rt::sleep(50);
  co_return autopilot_;
}

Json Link::identity() const {
  const bool v = vehicle_.isObject(), a = autopilot_.isObject();
  return Json::obj({{"gen", generation_}, {"sysid", pinned_ ? Json(targetSys_) : Json()}, {"compid", pinned_ ? Json(targetComp_) : Json()},
                    {"autopilot", v ? vehicle_["autopilot"] : Json()}, {"type", v ? vehicle_["type"] : Json()}, {"typeName", v ? vehicle_["typeName"] : Json()},
                    {"uid", a ? autopilot_["uid"] : Json()}, {"fw", a ? autopilot_["fw"] : Json()}, {"gitHash", a ? autopilot_["gitHash"] : Json()},
                    {"board", a ? autopilot_["board"] : Json()}, {"link", description_}});
}

void Link::send(const std::string& name, const Json& fields) {
  if (!transport_) return;
  transport_->send(enc_.pack(name, fields));
  stats_.tx++;
}

void Link::paramAttempt(const std::shared_ptr<ParamReq>& req, int n) {
  if (generation_ != req->gen) {
    req->p.reject(linkError(req->name + (req->isSet ? " not written: the connection changed" : ": the connection changed"), "link-changed", n));
    return;
  }
  if (req->isSet && req->keepGoing && !req->keepGoing()) {
    req->p.reject(linkError(req->name + " not written: cancelled", "cancelled", n));
    return;
  }
  const std::uint64_t wid = nextWaiter_++;
  const std::uint64_t timer = Loop::main().setTimeout(req->timeoutMs, [this, req, wid, n] {
    auto it = pending_.find(req->name);
    if (it != pending_.end()) {
      auto& ws = it->second;
      for (auto w = ws.begin(); w != ws.end(); ++w)
        if (w->id == wid) {
          ws.erase(w);
          break;
        }
      if (ws.empty()) pending_.erase(it);
    }
    if (n < req->retries)
      paramAttempt(req, n + 1);
    else
      req->p.reject(linkError((req->isSet ? "param set timeout " : "param timeout ") + req->name, "timeout", n + 1));
  });
  pending_[req->name].push_back({wid, req->p, timer, n});
  if (req->isSet)
    send("PARAM_SET", Json::obj({{"target_system", targetSys_}, {"target_component", targetComp_}, {"param_id", req->name},
                                 {"param_value", req->value}, {"param_type", req->type}}));
  else
    send("PARAM_REQUEST_READ", Json::obj({{"target_system", targetSys_}, {"target_component", targetComp_},
                                          {"param_id", req->name}, {"param_index", -1}}));
}

rt::Future<double> Link::requestParam(const std::string& name, double timeoutMs, int retries) {
  auto req = std::make_shared<ParamReq>();
  req->name = name;
  req->timeoutMs = timeoutMs;
  req->retries = retries;
  req->gen = generation_;
  paramAttempt(req, 0);
  return req->p.future();
}

rt::Future<double> Link::setParam(const std::string& name, double value, double timeoutMs, int retries, std::function<bool()> keepGoing) {
  auto req = std::make_shared<ParamReq>();
  req->name = name;
  req->isSet = true;
  req->value = value;
  auto known = params_.find(name);
  req->type = known != params_.end() ? known->second.type : 9;
  req->timeoutMs = timeoutMs;
  req->retries = retries;
  req->gen = generation_;
  req->keepGoing = std::move(keepGoing);
  paramAttempt(req, 0);
  return req->p.future();
}

namespace {
struct BatchState {
  std::vector<std::string> names;
  std::size_t next = 0;
  int done = 0;
  Json out = Json::object();
  std::function<void(int, int)> onProgress;
};

rt::Task<void> paramWorker(Link* link, std::shared_ptr<BatchState> st) {
  while (st->next < st->names.size()) {
    const std::string n = st->names[st->next++];
    try {
      const double v = co_await link->requestParam(n);
      st->out.set(n, v);
    } catch (const std::exception&) {
      st->out.set(n, Json());
    }
    st->done++;
    if (st->onProgress) st->onProgress(st->done, static_cast<int>(st->names.size()));
  }
}

rt::Task<Json> paramBatch(Link* link, std::shared_ptr<BatchState> st, int concurrency) {
  std::vector<rt::Future<void>> ws;
  for (int i = 0; i < concurrency; i++) ws.push_back(paramWorker(link, st));
  co_await rt::whenAll(ws);
  co_return st->out;
}
}  // namespace

rt::Future<Json> Link::requestParams(const std::vector<std::string>& names, int concurrency,
                                     std::function<void(int, int)> onProgress) {
  auto st = std::make_shared<BatchState>();
  st->names = names;
  st->onProgress = std::move(onProgress);
  return paramBatch(this, st, concurrency);
}

void Link::setMessageRate(int msgId, double hz) {
  const double interval = hz > 0 ? core::jsRound(1e6 / hz) : -1;
  commandLong(511, {static_cast<double>(msgId), interval});
}

void Link::requestDataStream(int streamId, double hz) {
  send("REQUEST_DATA_STREAM", Json::obj({{"target_system", targetSys_}, {"target_component", targetComp_}, {"req_stream_id", streamId},
                                         {"req_message_rate", hz}, {"start_stop", hz > 0 ? 1 : 0}}));
}

void Link::commandLong(int command, const std::vector<double>& p) {
  auto at = [&](std::size_t i) { return i < p.size() && !std::isnan(p[i]) ? p[i] : 0.0; };
  send("COMMAND_LONG", Json::obj({{"target_system", targetSys_}, {"target_component", targetComp_}, {"command", command},
                                  {"confirmation", 0}, {"param1", at(0)}, {"param2", at(1)}, {"param3", at(2)}, {"param4", at(3)},
                                  {"param5", at(4)}, {"param6", at(5)}, {"param7", at(6)}}));
}

rt::Future<Json> Link::commandAck(int command, const std::vector<double>& p, double timeoutMs) {
  rt::Promise<Json> pr;
  struct Ids {
    std::uint64_t handler = 0, timer = 0, cmd = 0;
  };
  auto ids = std::make_shared<Ids>();
  ids->cmd = nextWaiter_++;
  auto done = [this, ids] {
    Loop::main().clear(ids->timer);
    off(ids->handler);
    pendingCmds_.erase(ids->cmd);
  };
  ids->handler = on("COMMAND_ACK", [this, pr, command, done](const Json& f) {
    if (f["command"].numOr(-1) != command) return;
    const double ts = f["target_system"].numOr(0), tc = f["target_component"].numOr(0);
    if ((ts != 0 && ts != encoderSys()) || (tc != 0 && tc != encoderComp())) return;   // an ACK for another requester
    done();
    pr.resolve(Json::obj({{"result", f["result"]}}));
  });
  pendingCmds_[ids->cmd] = [pr, done, command](const std::string& why, const std::string& code) {
    done();
    pr.reject(linkError("command " + std::to_string(command) + ": " + why, code, 1));
  };
  ids->timer = Loop::main().setTimeout(timeoutMs, [pr, done, command] {
    done();
    pr.reject(linkError("no ack for command " + std::to_string(command), "timeout", 1));
  });
  commandLong(command, p);
  return pr.future();
}

bool Link::alive() const { return connected_ && pinned_ && (Loop::now() - lastHeartbeatAt_) < 5000; }

}  // namespace aptuner::io
