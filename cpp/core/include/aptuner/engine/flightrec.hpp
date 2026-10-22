// One flight as the model-based (heavy / slow axis) tuner needs it, from either source:
//  - a DataFlash .bin log: RATE (rate target / actual / output at the loop or logging rate), PIQx / PIDx (P, I, D,
//    FF, LIMIT / RESET flags, slew limiter), ATT, RCIN, RCOU, QTUN / CTUN, MODE, MAG, messages, parameters;
//  - live telemetry through Mission Planner: PID_TUNING (needs GCS_PID_MASK 7), ATTITUDE, NAV_CONTROLLER_OUTPUT,
//    RC_CHANNELS, SERVO_OUTPUT_RAW, VFR_HUD, HEARTBEAT, STATUSTEXT, VIBRATION and the parameter list.
// Samples keep their own time stamps (seconds); nothing is interpolated or resampled here.
#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "aptuner/core/json.hpp"
#include "aptuner/core/rules.hpp"
#include "aptuner/engine/loganalysis.hpp"
#include "aptuner/log/dfreader.hpp"

namespace aptuner::engine::heavy {

using core::Json;
using Vec = std::vector<double>;

/** a time series; at(t) holds the last sample at or before t (the first sample before the start) */
struct Series {
  Vec t, v;
  bool empty() const { return t.empty(); }
  std::size_t size() const { return t.size(); }
  double at(double tq) const;
  /** index of the last sample at or before tq (0 before the start) */
  std::size_t indexAt(double tq) const;
  void push(double tt, double vv) {
    t.push_back(tt);
    v.push_back(vv);
  }
  /** median sample rate (Hz), 0 with fewer than 3 samples */
  double rateHz() const;
};

struct AxisRec {
  Series target, actual, out;                     // deg/s, deg/s, controller output (P + I + D + FF)
  Series P, I, D, FF, flags, dmod, srate;         // PID terms (flags: 1 LIMIT, 4 RESET, 8 I_TERM_SET; logs only)
  Series angDes, ang;                             // deg (yaw: heading 0..360 as logged)
};

struct FlightRecord {
  std::string source;    // "log" | "telemetry"
  std::string name;      // file name or "telemetry <date time>"
  core::Family family = core::Family::Vtol;
  std::string firmware;
  log::Params params;                       // values at the start
  std::vector<ParamChangeRec> changes;      // tuning-parameter changes during the flight
  std::array<AxisRec, 3> axis;              // roll, pitch, yaw
  Series stickYaw;                          // yaw stick PWM (RCMAP_YAW channel)
  std::map<int, Series> motors;             // lift motor number (1..) -> PWM
  Series throttle;                          // lift throttle 0..1
  Series altDes, alt;                       // m (QTUN / CTUN), when logged
  Series mode;                              // flight mode number
  Series armed;                             // 1 armed (telemetry); empty for logs
  Series magField;                          // compass field length (mGauss), first compass
  Series vibe;                              // vibration level (m/s/s, the largest axis)
  std::vector<log::TextMsg> text;
  Json identity = Json::object();           // {sysid, fw, gitHash, frame}
  /** parameters in effect at time t (start values with the logged changes applied) */
  log::Params paramsAt(double t) const;
};

/** the motor outputs used for lift: SERVOn_FUNCTION 33..40 -> motor 1..8 (ArduPilot numbering); {motor: servo channel} */
std::map<int, int> liftMotorChannels(const log::Params& p);

/** read a parsed .bin log into a flight record (the time stamps are cleaned in place) */
FlightRecord recordFromLog(log::LogData& log, const std::string& name, std::optional<core::Family> family = std::nullopt);
FlightRecord recordFromLogFile(const std::string& path, std::optional<core::Family> family = std::nullopt);

/**
 * Live telemetry to a flight record: feed it the MAVLink messages (as the link decodes them, a JSON object of fields)
 * with their arrival time; finish() gives the record. Read-only: it never sends anything.
 */
class TelemetryRecorder {
 public:
  void setParams(const Json& params);   // {NAME: value}
  void setFamily(core::Family f) { rec_.family = f; }
  void onMessage(const std::string& type, const Json& f, double tS);
  /** seconds of PID_TUNING data per axis (0 when GCS_PID_MASK is off) */
  double pidSeconds(int axis) const;
  bool armed() const { return armed_; }
  double startS() const { return t0_; }
  double lastS() const { return tLast_; }
  void clear();
  FlightRecord finish(const std::string& name) const;
 private:
  FlightRecord rec_;
  bool armed_ = false;
  double t0_ = -1, tLast_ = 0;
  Json params_ = Json::object();
};

}  // namespace aptuner::engine::heavy
