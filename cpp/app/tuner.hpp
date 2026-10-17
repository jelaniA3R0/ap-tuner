// Live tuning session: observe -> identify -> propose -> (apply in bounded steps with a watchdog) -> observe again,
// per axis; the QuikTune-style ramp as a fallback; the parameter journal with restore / undo.
// Port of src/engine/tuner.js (runs on the event loop).
#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "aptuner/core/json.hpp"
#include "aptuner/core/rules.hpp"
#include "aptuner/io/link.hpp"
#include "aptuner/rt/async.hpp"
#include "aptuner/rt/emitter.hpp"
#include "liveaxis.hpp"

namespace aptuner::app {

using core::Family;

extern const std::vector<std::string> kAxisNames;   // roll, pitch, yaw

class Tuner : public rt::Emitter {
 public:
  Tuner(io::Link& link, const std::string& sessionDir);

  // ---- session setup
  rt::Task<Json> prepare(bool confirm);
  rt::Task<void> switchFamily(Family fam);
  Json safetyChecks() const;
  void requestStreams();
  rt::Task<double> setParam(const std::string& name, double value, const std::string& reason);
  void saveSession();
  rt::Task<void> writeGains(const std::string& axis, const Json& gains, const std::string& reason);
  /** analysis of one axis over the last observe window (or `seconds`) */
  Json analyze(const std::string& axis, double seconds = 0);

  // ---- session control
  void start();
  void pause(const std::string& reason);
  std::string currentAxis() const;
  rt::Task<void> stop();
  rt::Task<void> restoreOriginals();
  rt::Task<void> revertAxis(const std::string& axis, const std::string& reason);
  rt::Task<void> undoAxis(const std::string& axis);
  rt::Task<bool> applyProposal(const std::string& axis);
  void resume();
  Json snapshot() const;

  void log(const std::string& kind, const std::string& text);
  void setState(const std::string& s, const std::string& info = "");

  // ---- state (public: the flight / ground sessions and the server read it, as in the JavaScript)
  io::Link& link;
  Json opts;
  std::map<std::string, LiveAxis> axes;
  Json attitude, rc, hud, vibe;
  Json params = Json::object();
  std::optional<Family> family;
  std::string state = "idle", stateInfo;
  double stateSince = 0;
  int axisIdx = 0, round = 0;
  Json journal = Json::array(), originals = Json::object(), lastGood = Json::object(), previous = Json::object();
  Json results = Json::object();
  Json messages = Json::array();
  Json pendingWrites = Json::array();
  double loopHz = 400;
  bool prepared = false;

 private:
  struct Ramp {
    std::string axis, stage;
    Json names, saved, filters;
    double lastChange = 0, nextWrite = 0, lastPilot = 0;
    bool filtersSet = false;
  };
  rt::Task<bool> applyStaged(const std::string& axis, Json target);
  rt::Future<bool> watch(const std::string& axis, double seconds);
  void tick();
  void tickLoop();
  rt::Task<void> tickStep();
  rt::Task<void> observeStep();
  void nextAxis();
  void startRamp(const std::string& axis);
  rt::Task<void> rampStep();
  rt::Task<void> coupleI(const std::string& axis, double P);
  void onDisarm();
  Json gainsOf(const std::string& axis) const;

  std::optional<Ramp> ramp_;
  std::uint64_t timer_ = 0;
  std::string sessionDir_, sessionFile_;
  int linkGen_ = -1, preparedGen_ = -1;
  bool armedSeen_ = false;
};

}  // namespace aptuner::app
