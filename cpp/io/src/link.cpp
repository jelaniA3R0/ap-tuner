#include "aptuner/io/link.hpp"

#include <cmath>

#include "aptuner/core/cx.hpp"
#include "aptuner/core/jsnum.hpp"

namespace aptuner::io {

using rt::Loop;

struct Link::ParamReq {
  std::string name;
  bool isSet = false;
  double value = 0;
  int type = 9;
  double timeoutMs = 1500;
  int retries = 4;
  rt::Promise<double> p;
};

Link::Link() { rateWinT_ = Loop::now(); }

Link::~Link() { disconnect(); }

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
  opts_ = opts;
  generation_++;
  serialOpened_ = false;
  parser_ = mavlink::Parser();
  lastHeartbeatAt_ = 0;
  heartbeat_ = Json();
  vehicle_ = Json();
  const std::string kind = opts["kind"].strOr("tcp");
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
      const double type = m.fields["type"].numOr(0);
      if (type == 6 || m.compid != 1) continue;   // ignore GCS / companion heartbeats
      targetSys_ = m.sysid;
      targetComp_ = m.compid;
      lastHeartbeatAt_ = now;
      heartbeat_ = m.fields;
      const long long baseMode = static_cast<long long>(m.fields["base_mode"].numOr(0));
      const long long mode = static_cast<long long>(m.fields["custom_mode"].numOr(0));
      const int t = static_cast<int>(type);
      const char* tn = mavlink::enumName("MAV_TYPE", t);
      vehicle_ = Json::obj({{"type", t},
                            {"autopilot", m.fields["autopilot"]},
                            {"mode", static_cast<double>(mode)},
                            {"armed", (baseMode & 128) != 0},
                            {"modeName", modeName(t, mode)},
                            {"typeName", tn ? std::string(tn) : std::to_string(t)}});
      emit("heartbeat", vehicle_);
    } else if (name == "PARAM_VALUE") {
      const std::string id = m.fields["param_id"].strOr("");
      const double value = m.fields["param_value"].numOr(0);
      ParamInfo pi;
      pi.value = value;
      pi.type = static_cast<int>(m.fields["param_type"].numOr(9));
      pi.index = static_cast<int>(m.fields["param_index"].numOr(-1));
      pi.count = static_cast<int>(m.fields["param_count"].numOr(0));
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
      emit("param", Json::obj({{"name", id}, {"value", value}}));
    }
    if (has("message"))
      emit("message", Json::obj({{"id", m.id}, {"name", name}, {"sysid", m.sysid}, {"compid", m.compid}, {"seq", m.seq}, {"fields", m.fields}}));
    emit(name, m.fields);
  }
}

void Link::send(const std::string& name, const Json& fields) {
  if (!transport_) return;
  transport_->send(enc_.pack(name, fields));
  stats_.tx++;
}

void Link::paramAttempt(const std::shared_ptr<ParamReq>& req, int n) {
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
    }
    if (n < req->retries) {
      paramAttempt(req, n + 1);
    } else {
      pending_.erase(req->name);
      req->p.reject((req->isSet ? "param set timeout " : "param timeout ") + req->name);
    }
  });
  pending_[req->name].push_back({wid, req->p, timer});
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
  paramAttempt(req, 0);
  return req->p.future();
}

rt::Future<double> Link::setParam(const std::string& name, double value, double timeoutMs, int retries) {
  auto req = std::make_shared<ParamReq>();
  req->name = name;
  req->isSet = true;
  req->value = value;
  auto known = params_.find(name);
  req->type = known != params_.end() ? known->second.type : 9;
  req->timeoutMs = timeoutMs;
  req->retries = retries;
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
  auto ids = std::make_shared<std::pair<std::uint64_t, std::uint64_t>>(0, 0);   // handler, timer
  ids->first = on("COMMAND_ACK", [this, pr, ids, command](const Json& f) {
    if (f["command"].numOr(-1) != command) return;
    Loop::main().clear(ids->second);
    off(ids->first);
    pr.resolve(Json::obj({{"result", f["result"]}}));
  });
  ids->second = Loop::main().setTimeout(timeoutMs, [this, pr, ids, command] {
    off(ids->first);
    pr.reject("no ack for command " + std::to_string(command));
  });
  commandLong(command, p);
  return pr.future();
}

bool Link::alive() const { return connected_ && (Loop::now() - lastHeartbeatAt_) < 5000; }

}  // namespace aptuner::io
