// A scriptable fake ArduPilot copter on the app's real MAVLink Link (no sockets, no simulator). It answers
// PARAM_REQUEST_READ / PARAM_SET / COMMAND_LONG the way ArduPilot does (float32 values, echo of the stored value) and
// streams HEARTBEAT, PID_TUNING (three axes), ATTITUDE, NAV_CONTROLLER_OUTPUT and ESC telemetry. A test scripts it:
// oscillation on an axis (SRate), telemetry dropouts, sparse data, attitude errors, ignored writes.
// The C++ twin of test/fakevehicle.js (same behaviour, same parameters).
#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
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

class FakeVehicle {
 public:
  struct Set {
    double t;
    std::string name;
    double value;
  };
  struct Att {
    double roll = 0, pitch = 0;
  };

  explicit FakeVehicle(std::map<std::string, double> p = copterParams(), bool isArmed = true, int vehicleType = 2, int customMode = 2, double rate = 50)
      : params(std::move(p)), armed(isArmed), type(vehicleType), mode(customMode), rateHz(rate) {}

  FakeVehicle& attach(io::Link& link) {
    link_ = &link;
    link.attachTransport(std::make_unique<Transport>(this), "fake vehicle");
    auto& loop = rt::Loop::main();
    t0_ = rt::Loop::now();
    nextPid_ = nextAtt_ = nextSlow_ = t0_;
    hb();
    timers_.push_back(loop.setInterval(1000, [this] { hb(); }));
    timers_.push_back(loop.setInterval(5, [this] { tick(); }));
    return *this;
  }
  void stop() {
    for (auto id : timers_) rt::Loop::main().clear(id);
    timers_.clear();
    if (link_) link_->disconnect();
  }
  void feed(const std::string& name, const Json& fields) {
    if (link_ && link_->connected()) link_->feed(enc_.pack(name, fields));
  }
  std::vector<Set> setsOf(const std::string& name) const {
    std::vector<Set> out;
    for (const auto& s : sets)
      if (s.name == name) out.push_back(s);
    return out;
  }

  // the script
  std::map<std::string, double> params;
  bool armed;
  int type, mode;
  double rateHz;
  bool streaming = true, heartbeats = true, ignoreSets = false;
  double replyDelayMs = 5;
  std::map<std::string, double> slew{{"roll", 0.5}, {"pitch", 0.5}, {"yaw", 0.5}};   // SRate (above OSC_SMAX 5 = oscillating)
  std::map<std::string, double> dmod{{"roll", 1}, {"pitch", 1}, {"yaw", 1}};
  double desiredAmp = 0.2;   // rad/s (deg/s on a plane): a slow sine as the rate target
  std::function<void(const std::string&, double)> onSet;   // the scripted reaction of the airframe to a write
  std::optional<Att> attitude, navTarget;                 // ATTITUDE at 25 Hz / NAV_CONTROLLER_OUTPUT at 5 Hz
  double escRpm = 0;                                       // > 0: ESC_TELEMETRY_1_TO_4 at 5 Hz
  std::vector<Set> sets;                                   // every PARAM_SET received

  /** bytes the app sent (through the link's transport) */
  void received(const std::vector<std::uint8_t>& b) {
    for (const auto& m : parser_.feed(b.data(), b.size())) {
      const std::string name = m.name;
      const Json& f = m.fields;
      if (name == "PARAM_REQUEST_READ") {
        reply(f["param_id"].strOr(""));
      } else if (name == "PARAM_SET") {
        const std::string id = f["param_id"].strOr("");
        sets.push_back({rt::Loop::now(), id, f["param_value"].numOr(0)});
        if (ignoreSets) continue;
        params[id] = f32(f["param_value"].numOr(0));
        reply(id);
        if (onSet) onSet(id, params[id]);
      } else if (name == "COMMAND_LONG") {
        const double cmd = f["command"].numOr(0);
        rt::Loop::main().setTimeout(replyDelayMs, [this, cmd] { feed("COMMAND_ACK", Json::obj({{"command", cmd}, {"result", 0}})); });
      }
    }
  }

 private:
  class Transport : public io::Transport {
   public:
    explicit Transport(FakeVehicle* v) : v_(v) {}
    void send(const std::vector<std::uint8_t>& bytes) override {
      if (v_) v_->received(bytes);
    }
    void close() override { v_ = nullptr; }

   private:
    FakeVehicle* v_;
  };

  void hb() {
    if (heartbeats)
      feed("HEARTBEAT", Json::obj({{"type", type}, {"autopilot", 3}, {"base_mode", armed ? 129 : 1}, {"custom_mode", mode}, {"system_status", armed ? 4 : 3},
                                   {"mavlink_version", 3}}));
  }
  void reply(const std::string& name) {
    const auto it = params.find(name);
    const double v = it != params.end() ? it->second : 0;   // an unknown name reads as 0 (keeps prepare quick)
    rt::Loop::main().setTimeout(replyDelayMs, [this, name, v] {
      feed("PARAM_VALUE", Json::obj({{"param_id", name}, {"param_value", v}, {"param_type", 9}, {"param_count", 900}, {"param_index", 1}}));
    });
  }
  void tick() {
    const double now = rt::Loop::now();
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
  mavlink::Encoder enc_{1, 1};
  mavlink::Parser parser_;
  std::vector<std::uint64_t> timers_;
  double t0_ = 0, nextPid_ = 0, nextAtt_ = 0, nextSlow_ = 0;
};

}  // namespace aptuner::test
