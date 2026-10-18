// A scriptable fake ArduPilot copter on the app's real MAVLink Link (no sockets, no simulator). It answers
// PARAM_REQUEST_READ / PARAM_SET / COMMAND_LONG the way ArduPilot does (float32 values, whole-number parameters stored as
// ArduPilot casts them, echo of the stored value, AUTOPILOT_VERSION, reboot) and streams HEARTBEAT, PID_TUNING (three
// axes), ATTITUDE, NAV_CONTROLLER_OUTPUT, SYSTEM_TIME and ESC telemetry. A test scripts it: oscillation on an axis
// (SRate), telemetry dropouts, sparse data, attitude errors, ignored / dropped / refused / clamped writes, late or stale
// echoes, missing parameters, another ground station writing, another vehicle on the same link, a reboot.
// The C++ twin of test/fakevehicle.js (same behaviour, same parameters).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "aptuner/core/json.hpp"
#include "aptuner/io/link.hpp"
#include "aptuner/io/transport.hpp"
#include "aptuner/mavlink/mavlink.hpp"
#include "aptuner/rt/loop.hpp"

namespace aptuner::test {

using core::Json;

inline double f32(double v) { return static_cast<double>(static_cast<float>(v)); }

inline std::map<std::string, double> copterParams() {
  std::map<std::string, double> p{{"SCHED_LOOP_RATE", 400}, {"INS_GYRO_FILTER", 20}, {"GCS_PID_MASK", 7}, {"ATC_RATE_FF_ENAB", 1}, {"MOT_THST_HOVER", 0.39},
                                  {"MOT_THST_EXPO", 0.65}, {"INS_HNTCH_ENABLE", 0}, {"INS_HNTCH_MODE", 1}, {"INS_HNTCH_FREQ", 80}, {"INS_HNTCH_BW", 40},
                                  {"INS_HNTCH_ATT", 40}, {"INS_HNTCH_REF", 0}, {"INS_HNTCH_HMNCS", 3}, {"INS_HNTCH_OPTS", 0}, {"INS_HNTCH_FM_RAT", 1},
                                  {"FFT_ENABLE", 0}};
  struct AxisRow {
    const char* ax;
    double P, I, D, E;
  };
  for (const AxisRow& r : {AxisRow{"RLL", 0.135, 0.135, 0.0036, 0}, AxisRow{"PIT", 0.135, 0.135, 0.0036, 0}, AxisRow{"YAW", 0.18, 0.018, 0, 2.5}}) {
    const std::string pre = std::string("ATC_RAT_") + r.ax + "_";
    p[pre + "P"] = r.P;
    p[pre + "I"] = r.I;
    p[pre + "D"] = r.D;
    p[pre + "FF"] = 0;
    p[pre + "FLTT"] = 20;
    p[pre + "FLTE"] = r.E;
    p[pre + "FLTD"] = 20;
    p[pre + "SMAX"] = 50;
    p[pre + "IMAX"] = 0.5;
    p[pre + "D_FF"] = 0;
  }
  p["ATC_ANG_RLL_P"] = 4.5;
  p["ATC_ANG_PIT_P"] = 4.5;
  p["ATC_ANG_YAW_P"] = 4.5;
  for (auto& [k, v] : p) v = f32(v);
  return p;
}

/** how ArduPilot stores a float sent for a whole-number parameter (AP_Param::set_float: +-0.01, clamp, truncate) */
inline double apStore(int type, double v) {
  double lo, hi;
  if (type == 2) {
    lo = -128;
    hi = 127;
  } else if (type == 4) {
    lo = -32768;
    hi = 32767;
  } else if (type == 6) {
    lo = -2147483648.0;
    hi = 2147483647.0;
  } else {
    return f32(v);
  }
  return std::trunc(std::min(hi, std::max(lo, v + (v < 0 ? -0.01 : 0.01))));
}

class FakeVehicle;
/** several fake vehicles on one link (one transport, every vehicle hears everything) */
struct Bus {
  std::vector<FakeVehicle*> vehicles;
  std::shared_ptr<bool> closed = std::make_shared<bool>(false);
};

class FakeVehicle {
 public:
  struct Set {
    double t;
    std::string name;
    double value;
  };
  struct Cmd {
    double t;
    int command;
    double p1;
  };
  struct Att {
    double roll = 0, pitch = 0;
  };

  explicit FakeVehicle(std::map<std::string, double> p = copterParams(), bool isArmed = true, int vehicleType = 2, int customMode = 2, double rate = 50)
      : params(std::move(p)), armed(isArmed), type(vehicleType), mode(customMode), rateHz(rate) {}

  /** attach to the link as its only vehicle, or (bus) share one link with other fake vehicles */
  FakeVehicle& attach(io::Link& link, Bus* bus = nullptr) {
    link_ = &link;
    enc_.emplace(sysid, 1);
    // a closed transport (the link reconnected) delivers nothing more, like a closed socket
    if (bus) {
      bus->vehicles.push_back(this);
      closed_ = bus->closed;
      if (bus->vehicles.size() == 1) link.attachTransport(std::make_unique<BusTransport>(bus), "fake vehicles");
    } else {
      closed_ = std::make_shared<bool>(false);
      link.attachTransport(std::make_unique<Transport>(this, closed_), "fake vehicle");
    }
    auto& loop = rt::Loop::main();
    t0_ = rt::Loop::now();
    nextPid_ = nextAtt_ = nextSlow_ = nextSys_ = t0_;
    hb();
    timers_.push_back(loop.setInterval(1000, [this] { hb(); }));
    timers_.push_back(loop.setInterval(5, [this] { tick(); }));
    return *this;
  }
  void stop() {
    silence();
    if (link_ && closed_ && !*closed_) link_->disconnect();
  }
  /** stop this vehicle's timers only (the link stays: other vehicles on the bus go on) */
  void silence() {
    for (auto id : timers_) rt::Loop::main().clear(id);
    timers_.clear();
  }
  void feed(const std::string& name, const Json& fields) {
    if (link_ && enc_ && closed_ && !*closed_ && !down_) link_->feed(enc_->pack(name, fields));
  }
  std::vector<Set> setsOf(const std::string& name) const {
    std::vector<Set> out;
    for (const auto& s : sets)
      if (s.name == name) out.push_back(s);
    return out;
  }
  int typeOf(const std::string& name) const {
    const auto it = types.find(name);
    return it != types.end() ? it->second : 9;
  }
  /** another ground station writes a value (ArduPilot announces the change to every channel) */
  void foreignSet(const std::string& name, double value) {
    params[name] = apStore(typeOf(name), value);
    valueMsg(name, -1);
  }

  // the script
  std::map<std::string, double> params;
  // MAV_PARAM_TYPE of the whole-number parameters (everything else is REAL32 = 9)
  std::map<std::string, int> types{{"INS_HNTCH_ENABLE", 2}, {"INS_HNTCH_MODE", 2}, {"INS_HNTCH_HMNCS", 2}, {"INS_HNTCH_OPTS", 2}, {"FFT_ENABLE", 2},
                                   {"ATC_RATE_FF_ENAB", 2}, {"GCS_PID_MASK", 4},   {"SCHED_LOOP_RATE", 4}, {"INS_GYRO_FILTER", 4}};
  bool armed;
  int type, mode;
  double rateHz;
  int sysid = 1;                                  // set before attach()
  std::string uid = "0123456789abcdef01234567";   // board UID (hex), sent as uid2
  std::vector<int> fw{4, 5, 7};
  std::string gitHash = "abcdef12";
  bool streaming = true, heartbeats = true, ignoreSets = false, ignoreReads = false;
  double replyDelayMs = 5;
  double echoDelayMs = -1;   // delay of the echo of a PARAM_SET (-1: replyDelayMs): late acknowledgements
  int dropEchoes = 0;        // the next N writes are stored but their echo is lost
  int dropSets = 0;          // the next N PARAM_SETs are lost on the way (not stored, no echo)
  int staleEchoes = 0;       // the next N writes: first a PARAM_VALUE with the old value (a late answer), then the echo
  std::set<std::string> readOnly;                            // refused: ArduPilot keeps the value and echoes it
  std::map<std::string, std::pair<double, double>> clamp;    // stored clamped (a vehicle-side limit)
  std::set<std::string> missing;                             // no such parameter: no answer at all
  std::set<std::string> hidden;                              // exist only after the next reboot
  bool ackForOther = false;                                  // COMMAND_ACKs name another ground station as the requester
  double rebootMs = 2000;                                    // silent for this long after MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
  std::function<void()> onReboot;                            // what the reboot changes
  int reboots = 0;
  std::map<std::string, double> slew{{"roll", 0.5}, {"pitch", 0.5}, {"yaw", 0.5}};   // SRate (above OSC_SMAX 5 = oscillating)
  std::map<std::string, double> dmod{{"roll", 1}, {"pitch", 1}, {"yaw", 1}};
  double desiredAmp = 0.2;   // rad/s (deg/s on a plane): a slow sine as the rate target
  std::function<void(const std::string&, double)> onSet;   // the scripted reaction of the airframe to a write
  std::optional<Att> attitude, navTarget;                 // ATTITUDE at 25 Hz / NAV_CONTROLLER_OUTPUT at 5 Hz
  double escRpm = 0;                                       // > 0: ESC_TELEMETRY_1_TO_4 at 5 Hz
  double sysTimeHz = 2;                                    // SYSTEM_TIME (time_boot_ms: the boot clock)
  std::vector<Set> sets;                                   // every PARAM_SET received
  std::vector<Set> reads;                                  // every PARAM_REQUEST_READ received (value unused)
  std::vector<Cmd> commands;                               // every COMMAND_LONG received

  /** bytes the app sent (through the link's transport) */
  void received(const std::vector<std::uint8_t>& b) {
    for (const auto& m : parser_.feed(b.data(), b.size())) {
      const std::string name = m.name;
      const Json& f = m.fields;
      if (f.has("target_system") && f["target_system"].numOr(0) != 0 && f["target_system"].numOr(0) != sysid) continue;   // for another vehicle
      if (down_) continue;   // rebooting: nothing is heard
      if (name == "PARAM_REQUEST_READ") {
        const std::string id = f["param_id"].strOr("");
        reads.push_back({rt::Loop::now(), id, 0});
        if (exists(id) && !ignoreReads) reply(id);
      } else if (name == "PARAM_SET") {
        set(f["param_id"].strOr(""), f["param_value"].numOr(0));
      } else if (name == "COMMAND_LONG") {
        command(m);
      }
    }
  }

 private:
  class Transport : public io::Transport {
   public:
    Transport(FakeVehicle* v, std::shared_ptr<bool> closed) : v_(v), closed_(std::move(closed)) {}
    void send(const std::vector<std::uint8_t>& bytes) override {
      if (v_ && !*closed_) v_->received(bytes);
    }
    void close() override {
      *closed_ = true;
      v_ = nullptr;
    }

   private:
    FakeVehicle* v_;
    std::shared_ptr<bool> closed_;
  };
  class BusTransport : public io::Transport {
   public:
    explicit BusTransport(Bus* bus) : bus_(bus) {}
    void send(const std::vector<std::uint8_t>& bytes) override {
      if (*bus_->closed) return;
      const auto vs = bus_->vehicles;
      for (auto* v : vs) v->received(bytes);
    }
    void close() override { *bus_->closed = true; }

   private:
    Bus* bus_;
  };

  bool exists(const std::string& name) const { return !missing.count(name) && !hidden.count(name); }
  void hb() {
    if (heartbeats)
      feed("HEARTBEAT", Json::obj({{"type", type}, {"autopilot", 3}, {"base_mode", armed ? 129 : 1}, {"custom_mode", mode}, {"system_status", armed ? 4 : 3},
                                   {"mavlink_version", 3}}));
  }
  void valueMsg(const std::string& name, int index, std::optional<double> value = std::nullopt) {
    const auto it = params.find(name);
    const double v = value ? *value : (it != params.end() ? it->second : 0);   // an unknown name reads as 0 (keeps prepare quick)
    feed("PARAM_VALUE", Json::obj({{"param_id", name}, {"param_value", v}, {"param_type", typeOf(name)}, {"param_count", 900},
                                   {"param_index", index < 0 ? 65535 : index}}));   // ArduPilot: -1 for a change it announces
  }
  void reply(const std::string& name) {
    rt::Loop::main().setTimeout(replyDelayMs, [this, name] { valueMsg(name, 1); });
  }
  void set(const std::string& name, double value) {
    sets.push_back({rt::Loop::now(), name, value});
    if (ignoreSets || !exists(name)) return;
    if (dropSets > 0) {
      dropSets--;
      return;
    }
    const auto it = params.find(name);
    const double old = it != params.end() ? it->second : 0;
    const double delay = echoDelayMs >= 0 ? echoDelayMs : replyDelayMs;
    if (readOnly.count(name)) {
      feed("STATUSTEXT", Json::obj({{"severity", 4}, {"text", "Param write denied (" + name + ")"}}));
      rt::Loop::main().setTimeout(delay, [this, name] { valueMsg(name, -1); });
      return;
    }
    double v = apStore(typeOf(name), value);
    const auto c = clamp.find(name);
    if (c != clamp.end()) v = f32(std::min(c->second.second, std::max(c->second.first, v)));
    params[name] = v;
    if (staleEchoes > 0) {
      staleEchoes--;
      rt::Loop::main().setTimeout(1, [this, name, old] { valueMsg(name, 1, old); });
    }
    if (dropEchoes > 0)
      dropEchoes--;
    else
      rt::Loop::main().setTimeout(delay, [this, name] { valueMsg(name, -1); });
    if (onSet) onSet(name, params[name]);
  }
  void command(const mavlink::Message& m) {
    const Json& f = m.fields;
    const int cmd = static_cast<int>(f["command"].numOr(0));
    const double p1 = f["param1"].numOr(0);
    commands.push_back({rt::Loop::now(), cmd, p1});
    int result = 0;
    if (cmd == 246) result = armed ? 4 : 0;   // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: refused while armed (MAV_RESULT_FAILED)
    const Json ack = Json::obj({{"command", cmd}, {"result", result}, {"progress", 0}, {"result_param2", 0}, {"target_system", ackForOther ? 254 : m.sysid},
                                {"target_component", ackForOther ? 1 : m.compid}});
    rt::Loop::main().setTimeout(replyDelayMs, [this, ack] { feed("COMMAND_ACK", ack); });
    if ((cmd == 512 && p1 == 148) || cmd == 520) rt::Loop::main().setTimeout(replyDelayMs, [this] { autopilotVersion(); });
    if (cmd == 246 && result == 0) rt::Loop::main().setTimeout(replyDelayMs + 20, [this] { reboot(); });
  }
  void autopilotVersion() {
    Json uid2 = Json::array(), git = Json::array(), zeros = Json::array();
    for (std::size_t i = 0; i < 18; i++) uid2.push(i * 2 + 1 < uid.size() ? std::stoi(uid.substr(i * 2, 2), nullptr, 16) : 0);
    for (std::size_t i = 0; i < 8; i++) {
      git.push(i < gitHash.size() ? static_cast<int>(static_cast<unsigned char>(gitHash[i])) : 0);
      zeros.push(0);
    }
    const double sw = static_cast<double>((static_cast<std::uint32_t>(fw[0]) << 24) | (static_cast<std::uint32_t>(fw[1]) << 16) | (static_cast<std::uint32_t>(fw[2]) << 8) | 255u);
    feed("AUTOPILOT_VERSION", Json::obj({{"capabilities", 0}, {"flight_sw_version", sw}, {"middleware_sw_version", 0}, {"os_sw_version", 0}, {"board_version", 0x8c0000},
                                         {"flight_custom_version", git}, {"middleware_custom_version", zeros}, {"os_custom_version", zeros}, {"vendor_id", 0x2dae},
                                         {"product_id", 0x1016}, {"uid", 0}, {"uid2", uid2}}));
  }
  void reboot() {
    down_ = true;
    reboots++;
    rt::Loop::main().setTimeout(rebootMs, [this] {
      hidden.clear();
      if (onReboot) onReboot();
      t0_ = rt::Loop::now();
      down_ = false;
      hb();
    });
  }
  void tick() {
    const double now = rt::Loop::now();
    if (down_) {
      nextPid_ = now;
      return;
    }
    if (!streaming) nextPid_ = now;
    // PID_TUNING at rateHz on average (timers are coarse on Windows: catch up in bursts)
    for (int n = 0; streaming && nextPid_ <= now && n < 20; n++) {
      nextPid_ += 1000 / rateHz;
      const double t = (now - t0_) / 1000;
      int id = 1;
      for (const char* ax : {"roll", "pitch", "yaw"}) {
        const double des = desiredAmp * std::sin(2 * std::numbers::pi * 0.5 * t + id);
        feed("PID_TUNING", Json::obj({{"axis", id}, {"desired", des}, {"achieved", 0.9 * des}, {"FF", 0}, {"P", 0.01 * des}, {"I", 0}, {"D", 0},
                                      {"SRate", slew[ax]}, {"PDmod", dmod[ax]}}));
        id++;
      }
    }
    if (attitude && nextAtt_ <= now) {
      nextAtt_ = now + 40;
      const double d = std::numbers::pi / 180;
      feed("ATTITUDE", Json::obj({{"time_boot_ms", now - t0_}, {"roll", attitude->roll * d}, {"pitch", attitude->pitch * d}, {"yaw", 0}, {"rollspeed", 0},
                                  {"pitchspeed", 0}, {"yawspeed", 0}}));
    }
    if (sysTimeHz > 0 && nextSys_ <= now) {
      nextSys_ = now + 1000 / sysTimeHz;
      feed("SYSTEM_TIME", Json::obj({{"time_unix_usec", now * 1000}, {"time_boot_ms", now - t0_}}));
    }
    if (nextSlow_ <= now) {
      nextSlow_ = now + 200;
      if (navTarget)
        feed("NAV_CONTROLLER_OUTPUT", Json::obj({{"nav_roll", navTarget->roll}, {"nav_pitch", navTarget->pitch}, {"nav_bearing", 0}, {"target_bearing", 0},
                                                 {"wp_dist", 0}, {"alt_error", 0}, {"aspd_error", 0}, {"xtrack_error", 0}}));
      if (escRpm > 0) {
        auto four = [](double v) { return Json(Json::Array{v, v, v, v}); };
        feed("ESC_TELEMETRY_1_TO_4", Json::obj({{"temperature", four(30)}, {"voltage", four(1600)}, {"current", four(500)}, {"totalcurrent", four(0)},
                                                {"rpm", four(escRpm)}, {"count", four(1)}}));
      }
    }
  }

  io::Link* link_ = nullptr;
  std::optional<mavlink::Encoder> enc_;
  mavlink::Parser parser_;
  std::shared_ptr<bool> closed_;
  std::vector<std::uint64_t> timers_;
  bool down_ = false;
  double t0_ = 0, nextPid_ = 0, nextAtt_ = 0, nextSlow_ = 0, nextSys_ = 0;
};

}  // namespace aptuner::test
