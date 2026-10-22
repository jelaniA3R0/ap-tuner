// Model tuning from live telemetry (heavy / slow vehicles), read-only: with the vehicle connected (directly or through
// Mission Planner's MAVLink mirror) the app reads the settings it needs, records every flight from arming to disarming
// (PID_TUNING, ATTITUDE, RC_CHANNELS, SERVO_OUTPUT_RAW, ...) and, after landing, runs the same analysis as for a log:
// model, replay, robust design, step files. It never writes a parameter; it only asks the autopilot for the telemetry
// streams (as Mission Planner itself does).
#pragma once

#include <deque>
#include <map>
#include <memory>
#include <string>

#include "aptuner/core/json.hpp"
#include "aptuner/engine/flightrec.hpp"
#include "aptuner/io/link.hpp"
#include "aptuner/rt/emitter.hpp"
#include "aptuner/rt/async.hpp"

namespace aptuner::app {

class HeavyLive : public rt::Emitter {
 public:
  HeavyLive(io::Link& link, std::string sessionsDir);
  /** read the vehicle's settings and start listening (state: reading -> waiting -> recording -> analysing -> done) */
  rt::Task<core::Json> start();
  void stop();
  /** analyse what has been recorded so far (also while flying) */
  void analyseNow();
  core::Json state() const;
  /** called with each finished analysis (on the loop thread) */
  std::function<void(const core::Json&)> onResult;

 private:
  void onMsg(const std::string& type, const core::Json& f);
  void setState(const std::string& s, const std::string& info);
  void finishFlight(const std::string& why);
  void requestStreams();

  io::Link& link_;
  std::string sessionsDir_;
  engine::heavy::TelemetryRecorder rec_;
  std::string state_ = "idle", info_;
  bool listening_ = false, busy_ = false;
  double armedAt_ = 0;
  core::Json params_ = core::Json::object();
  core::Json warnings_ = core::Json::array();
  std::vector<std::uint64_t> handlers_;
  int flights_ = 0;
  std::map<std::string, std::deque<double>> seen_;   // arrival times of each message type (last 3 s)
  core::Json att_ = core::Json::object();
  double customMode_ = -1;
};

/** the parameters the model tuner reads from a live vehicle */
std::vector<std::string> heavyParamNames(bool quadplane, int yawChannel);

/** saved plans per vehicle (sessions/plans/sysid-N/<time>.json): the latest one whose flights are not among `sources` */
core::Json loadPlan(const std::string& sessionsDir, const core::Json& sysid, const std::vector<std::string>& sources);
/** save the plan of an analysis (returns the file) */
std::string savePlan(const std::string& sessionsDir, const core::Json& plan);
/** run the analysis on flights (with the saved plan of the vehicle) and save its plan */
core::Json analyseAndSave(const std::vector<engine::heavy::FlightRecord>& flights, const std::string& sessionsDir, double motorSpacingM);

}  // namespace aptuner::app
