// MAVLink link: TCP client / UDP client / UDP listener / serial, GCS heartbeat, parameter protocol, message
// intervals and commands. Emits decoded messages. Port of src/mavlink/link.js (loop thread only).
//   events: "status" (string), "heartbeat" (vehicle), "param" ({name, value, type}), "message" ({id, name, sysid,
//           compid, seq, fields}), "vehicle-change" ({reason, generation}), "reboot" ({count}), "autopilot" (info)
//           and one event per message name with its fields.
// Bound to one vehicle (2026-10-17 audit F-25, F-28): the first ArduPilot autopilot heard (autopilot 3, component 1,
// not a ground station, antenna tracker or other peripheral) becomes the target and stays it; messages from every
// other system or component are dropped, another vehicle on the link is only reported. A different vehicle (the
// target silent for 5 s while another one answers, a heartbeat type change, another board UID) counts like a new
// connection: the generation goes up, every pending request is rejected and the cached values are dropped.
// Parameter echoes and command acknowledgements only resolve requests of the current generation (an ACK addressed to
// another ground station does not count); AUTOPILOT_VERSION (board UID, firmware) is asked for on every new target
// and after a reboot (the autopilot's boot clock going back).
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
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

/** a failed request: code "timeout", "cancelled", "link-changed" or "rebooted"; attempts = PARAM_SETs / reads sent */
class LinkError : public rt::Error {
 public:
  LinkError(const std::string& msg, const std::string& code, int nAttempts) : rt::Error(msg, code), attempts(nAttempts) {}
  int attempts;
};

/** AUTOPILOT_VERSION fields -> {fw: "4.5.7" (+ "-beta" etc.), fwRaw, gitHash, board, vendor, product, uid (hex)} */
Json parseAutopilotVersion(const Json& f);

class Link : public rt::Emitter {
 public:
  Link();
  ~Link();

  /** opts: {kind: "tcp" | "udp" | "udpin" | "serial", host, port, baud} (serial: port is the COM name) */
  void connect(const Json& opts);
  void disconnect();
  /** a connection on a given transport (tests: a fake vehicle); bytes come in through feed() */
  void attachTransport(std::unique_ptr<Transport> t, const std::string& description = "test transport");
  void feed(const std::vector<std::uint8_t>& data) { rx(data); }

  void send(const std::string& name, const Json& fields);
  /** request one parameter; resolves to its value (rejects with a LinkError) */
  rt::Future<double> requestParam(const std::string& name, double timeoutMs = 1500, int retries = 4);
  /** many parameters with a small concurrency; result: {name: value | null} */
  rt::Future<Json> requestParams(const std::vector<std::string>& names, int concurrency = 3,
                                 std::function<void(int, int)> onProgress = nullptr);
  /** set a parameter and wait for the echo; resolves to the value the vehicle reports (compare it: paramwrite).
      keepGoing (optional): checked before every PARAM_SET, retries included; when it returns false nothing more is sent
      and the future rejects with code "cancelled" (attempts > 0: an earlier attempt may have reached the vehicle). A new
      connection or another vehicle rejects it with code "link-changed". */
  rt::Future<double> setParam(const std::string& name, double value, double timeoutMs = 1500, int retries = 3,
                              std::function<bool()> keepGoing = nullptr);
  /** MAV_CMD_SET_MESSAGE_INTERVAL (511) */
  void setMessageRate(int msgId, double hz);
  /** legacy REQUEST_DATA_STREAM (works through older firmware and the Mission Planner mirror) */
  void requestDataStream(int streamId, double hz);
  void commandLong(int command, const std::vector<double>& p = {});
  /** send a command and wait for its COMMAND_ACK from the target; resolves {result}; rejects on timeout or when the
      connection changes. An ACK naming another requester (ArduPilot fills target_system / component) is not ours. */
  rt::Future<Json> commandAck(int command, const std::vector<double>& p = {}, double timeoutMs = 3000);

  /** forget AUTOPILOT_VERSION and ask again (after a reboot: another board UID then counts as another vehicle) */
  void refreshAutopilot();
  /** resolves with AUTOPILOT_VERSION once known, or null after ms */
  rt::Task<Json> waitAutopilot(double ms = 3000);
  /** who the app talks to on this connection (gen = generation: a reconnect or another vehicle changes it) */
  Json identity() const;
  /** other vehicles heard in the last 10 s: [{sysid, compid, type}] */
  Json othersSeen() const;

  bool alive() const;
  bool connected() const { return connected_; }
  bool pinned() const { return pinned_; }
  bool serialOpened() const { return serialOpened_; }
  int generation() const { return generation_; }
  const std::string& description() const { return description_; }
  const Json& opts() const { return opts_; }
  const Json& vehicle() const { return vehicle_; }       // null until a heartbeat
  const Json& heartbeat() const { return heartbeat_; }
  const Json& autopilot() const { return autopilot_; }   // null until AUTOPILOT_VERSION arrived
  double lastHeartbeatAt() const { return lastHeartbeatAt_; }
  int targetSys() const { return targetSys_; }
  int targetComp() const { return targetComp_; }
  int encoderSys() const { return 255; }
  int encoderComp() const { return 190; }
  /** the autopilot's boot clock: count goes up on a reboot; ms (none until a message with time_boot_ms), at */
  int bootCount() const { return bootCount_; }
  std::optional<double> bootMs() const { return bootMs_; }
  double bootAt() const { return bootAt_; }
  const std::map<std::string, ParamInfo>& params() const { return params_; }
  std::map<std::string, ParamInfo>& params() { return params_; }
  const Json& msgRates() const { return msgRates_; }
  struct Stats {
    std::uint64_t rx = 0, tx = 0, bytes = 0, foreign = 0;
  };
  const Stats& stats() const { return stats_; }
  const mavlink::Parser::Stats& parserStats() const { return parser_.stats(); }

 private:
  struct ParamReq;
  void paramAttempt(const std::shared_ptr<ParamReq>& req, int n);
  void rx(const std::vector<std::uint8_t>& data);
  void onHeartbeat(const mavlink::Message& m, double now);
  void noteOther(const mavlink::Message& m, double now);
  void bootClock(double ms, double now);
  void onAutopilotVersion(const Json& f);
  void requestAutopilotVersion();
  void newGeneration(const std::string& why);
  void vehicleChanged(const std::string& why);
  void rejectPending(const std::string& why, const std::string& code = "link-changed");
  void startHeartbeat();
  void stopHeartbeat();

  std::unique_ptr<Transport> transport_;
  mavlink::Parser parser_;
  mavlink::Encoder enc_{255, 190};   // GCS system id 255 = SYSID_MYGCS (RC overrides are only accepted from it)
  int targetSys_ = 1, targetComp_ = 1;
  bool pinned_ = false;
  bool connected_ = false, serialOpened_ = false;
  std::map<std::string, ParamInfo> params_;
  struct Waiter {
    std::uint64_t id;
    rt::Promise<double> p;
    std::uint64_t timer;
    int attempt;
  };
  std::map<std::string, std::vector<Waiter>> pending_;
  std::map<std::uint64_t, std::function<void(const std::string&, const std::string&)>> pendingCmds_;   // commands waiting for their ACK
  std::uint64_t nextWaiter_ = 1;
  Json heartbeat_, vehicle_, opts_, msgRates_ = Json::object();
  Json autopilot_, prevAutopilot_;
  struct Other {
    int sysid = 0, compid = 0, type = 0;
    double lastSeen = 0;
  };
  std::map<int, Other> others_;
  std::set<int> warned_;
  Json ignored_;
  std::optional<double> bootMs_;
  double bootAt_ = 0;
  int bootCount_ = 0;
  std::string changedWhy_;
  double lastHeartbeatAt_ = 0;
  Stats stats_;
  std::map<std::string, int> rateCounts_;
  double rateWinT_ = 0;
  std::uint64_t hbTimer_ = 0, apTimer_ = 0;
  int generation_ = 0;
  std::string description_;
};

/** flight mode name for a vehicle type and custom_mode ("QSTABILIZE", "LOITER", ...) */
std::string modeName(int type, long long mode);

}  // namespace aptuner::io
