// MAVLink link: TCP client / UDP client / UDP listener / serial, GCS heartbeat, parameter protocol, message
// intervals and commands. Emits decoded messages. Port of src/mavlink/link.js (loop thread only).
//   events: "status" (string), "heartbeat" (vehicle), "param" ({name, value}), "message" ({id, name, sysid,
//           compid, seq, fields}) and one event per message name with its fields.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "aptuner/core/json.hpp"
#include "aptuner/io/transport.hpp"
#include "aptuner/mavlink/mavlink.hpp"
#include "aptuner/rt/async.hpp"
#include "aptuner/rt/emitter.hpp"

namespace aptuner::io {

using core::Json;

struct ParamInfo {
  double value = 0;
  int type = 9;
  int index = -1, count = 0;
  double ts = 0;
};

class Link : public rt::Emitter {
 public:
  Link();
  ~Link();

  /** opts: {kind: "tcp" | "udp" | "udpin" | "serial", host, port, baud} (serial: port is the COM name) */
  void connect(const Json& opts);
  void disconnect();

  void send(const std::string& name, const Json& fields);
  /** request one parameter; resolves to its value (rejects on timeout) */
  rt::Future<double> requestParam(const std::string& name, double timeoutMs = 1500, int retries = 4);
  /** many parameters with a small concurrency; result: {name: value | null} */
  rt::Future<Json> requestParams(const std::vector<std::string>& names, int concurrency = 3,
                                 std::function<void(int, int)> onProgress = nullptr);
  /** set a parameter and wait for the echo; resolves to the value the vehicle reports */
  rt::Future<double> setParam(const std::string& name, double value, double timeoutMs = 1500, int retries = 3);
  /** MAV_CMD_SET_MESSAGE_INTERVAL (511) */
  void setMessageRate(int msgId, double hz);
  /** legacy REQUEST_DATA_STREAM (works through older firmware and the Mission Planner mirror) */
  void requestDataStream(int streamId, double hz);
  void commandLong(int command, const std::vector<double>& p = {});
  /** send a command and wait for its COMMAND_ACK; resolves {result} */
  rt::Future<Json> commandAck(int command, const std::vector<double>& p = {}, double timeoutMs = 3000);

  bool alive() const;
  bool connected() const { return connected_; }
  bool serialOpened() const { return serialOpened_; }
  int generation() const { return generation_; }
  const std::string& description() const { return description_; }
  const Json& opts() const { return opts_; }
  const Json& vehicle() const { return vehicle_; }       // null until a heartbeat
  const Json& heartbeat() const { return heartbeat_; }
  double lastHeartbeatAt() const { return lastHeartbeatAt_; }
  int targetSys() const { return targetSys_; }
  int targetComp() const { return targetComp_; }
  const std::map<std::string, ParamInfo>& params() const { return params_; }
  std::map<std::string, ParamInfo>& params() { return params_; }
  const Json& msgRates() const { return msgRates_; }
  struct Stats {
    std::uint64_t rx = 0, tx = 0, bytes = 0;
  };
  const Stats& stats() const { return stats_; }
  const mavlink::Parser::Stats& parserStats() const { return parser_.stats(); }

 private:
  struct ParamReq;
  void paramAttempt(const std::shared_ptr<ParamReq>& req, int n);
  void rx(const std::vector<std::uint8_t>& data);
  void startHeartbeat();
  void stopHeartbeat();

  std::unique_ptr<Transport> transport_;
  mavlink::Parser parser_;
  mavlink::Encoder enc_{255, 190};   // GCS system id 255 = SYSID_MYGCS (RC overrides are only accepted from it)
  int targetSys_ = 1, targetComp_ = 1;
  bool connected_ = false, serialOpened_ = false;
  std::map<std::string, ParamInfo> params_;
  struct Waiter {
    std::uint64_t id;
    rt::Promise<double> p;
    std::uint64_t timer;
  };
  std::map<std::string, std::vector<Waiter>> pending_;
  std::uint64_t nextWaiter_ = 1;
  Json heartbeat_, vehicle_, opts_, msgRates_ = Json::object();
  double lastHeartbeatAt_ = 0;
  Stats stats_;
  std::map<std::string, int> rateCounts_;
  double rateWinT_ = 0;
  std::uint64_t hbTimer_ = 0;
  int generation_ = 0;
  std::string description_;
};

/** flight mode name for a vehicle type and custom_mode ("QSTABILIZE", "LOITER", ...) */
std::string modeName(int type, long long mode);

}  // namespace aptuner::io
