// Live tuning session: observe -> identify -> propose -> (apply in bounded steps, each change watched) -> observe
// again, per axis; the QuikTune-style ramp as a fallback; the parameter journal with restore / undo; and the in-flight
// safety layer (opt-in, one write lock with cancellation, all-axis oscillation monitor, watch after every in-flight
// change that fails closed, evidence-based revert). Port of src/engine/tuner.js (runs on the event loop).
#pragma once

#include <deque>
#include <map>
#include <memory>
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
/** the message given when a write while armed is refused because the in-flight opt-in is off */
extern const std::string kInFlightOff;

/** an oscillation (or a fail-closed condition) found by the monitor, a post-change watch or an observation round */
struct Trip {
  std::string source, axis;
  std::vector<std::string> reasons;
  bool failClosed = false;
  double t = 0;
};

/** thrown at a write checkpoint when a trip is pending (code "trip"); the job that catches it reverts */
class TripError : public rt::Error {
 public:
  explicit TripError(Trip t);
  Trip trip;
};

/** the holder of the single write lock */
struct Job {
  int id = 0;
  std::string kind;
  bool corrective = false, handlesTrips = false, abortWatch = false;
  std::string cancelled;   // non-empty: pause / stop asked it to write nothing more
};

struct Calm {
  bool calm = true;
  std::vector<std::string> reasons;
  double t = 0;
};

/** one change written (or found) in flight, with what it is checked against */
struct Change {
  struct Param {
    std::string name;
    Json from;   // number, or null when the value before was not known
    double to = 0;
  };
  int id = 0;
  double t = 0;
  std::string kind;
  std::optional<std::string> axis;   // none: vehicle-wide (notch, gyro filter)
  std::vector<std::string> axes;     // the axes it is watched on
  bool shared = false, armed = false;
  std::string reason, status;
  std::vector<Param> params;
  std::vector<std::string> written;
  std::optional<Calm> calmBefore;
};

class Tuner : public rt::Emitter {
 public:
  Tuner(io::Link& link, const std::string& sessionDir);
  ~Tuner();

  // ---- session setup
  rt::Task<Json> prepare(bool confirm);
  rt::Task<void> switchFamily(Family fam);
  Json safetyChecks() const;
  void requestStreams();
  /** the one place every parameter write goes through (job = the write-lock holder) */
  rt::Task<double> setParam(const std::string& name, double value, const std::string& reason, std::shared_ptr<Job> job);
  void saveSession();
  rt::Task<void> writeGains(const std::string& axis, const Json& gains, const std::string& reason, std::shared_ptr<Job> job);
  /** analysis of one axis over the last observe window (or `seconds`) */
  Json analyze(const std::string& axis, double seconds = 0);

  // ---- session control
  void beginSession();
  void endSession();
  void start();
  void pause(const std::string& reason);
  std::string currentAxis() const;
  rt::Task<void> stop();
  rt::Task<void> restoreOriginals();
  rt::Task<void> undoAxis(const std::string& axis);
  rt::Task<bool> applyProposal(const std::string& axis);
  /** a vehicle-wide change (notch, gyro filter, FFT) through the write lock, watched in flight; true if it held */
  rt::Task<bool> applyFilterChange(Json values, std::string reason);
  /** values the vehicle changed by itself ({name: {from, to}}): watched like the app's own changes */
  rt::Task<bool> watchExternalChange(Json changed, std::string reason);
  void resume();
  void startMonitor();
  void stopMonitor();
  Json snapshot() const;
  /** the kind of the job holding the write lock ("" when nothing is being written) */
  std::string writing() const { return job_ ? job_->kind : ""; }

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
  bool sessionActive = false;
  std::vector<std::shared_ptr<Change>> changes;
  Json monitor;
  Json lastTrip;

 private:
  friend struct TunerTestAccess;   // cpp/tests: the private ramp steps, as test/live-safety.test.js calls them
  struct Ramp {
    std::string axis, stage;
    Json names, saved, filters;
    double lastChange = 0, nextWrite = 0, lastPilot = 0;
    bool filtersSet = false;
  };
  struct WatchResult {
    bool ok = true, aborted = false;
    std::optional<Trip> trip;
  };
  struct CheckResult {
    bool ok = true;
    std::vector<std::string> reasons;
  };
  struct Target {
    std::string name;
    double to = 0;
  };
  struct CorrectiveResult {
    std::vector<Target> written;
    std::vector<std::string> failed;
  };
  rt::Task<Json> prepareInner(bool confirm);
  rt::Task<bool> applyStaged(const std::string& axis, Json target, std::shared_ptr<Job> job);
  rt::Task<bool> change(std::shared_ptr<Job> job, std::optional<std::string> axis, Json values, std::string reason, std::string kind);
  rt::Future<WatchResult> watchChange(std::shared_ptr<Job> job, std::shared_ptr<Change> ch);
  rt::Task<CheckResult> watchCorrective(std::vector<std::string> watchAxes);
  rt::Task<void> handleTrip(std::shared_ptr<Job> job, Trip trip, std::shared_ptr<Change> ch);
  rt::Task<CorrectiveResult> corrective(std::shared_ptr<Job> job, std::vector<Target> targets, std::string reason);
  rt::Task<void> userCorrective(std::string kind, std::vector<Target> targets, std::string reason);
  rt::Task<void> takeOver();
  rt::Task<bool> waitNoJob(double ms);
  std::shared_ptr<Job> beginJob(const std::string& kind, bool corrective = false, bool handlesTrips = false);
  void endJob(const std::shared_ptr<Job>& job);
  void cancelJob(const std::string& reason);
  void checkpoint(Job& job);
  void assertForward(const Job& job) const;
  bool isArmed() const;
  std::string watchBlind(const std::string& ax) const;
  std::vector<std::string> sharedAxes() const;
  Calm calmBefore() const;
  std::optional<Trip> sustained(const std::string& ax) const;
  std::optional<Trip> attitudeTrip(const std::vector<std::string>& watchAxes, double since) const;
  void raiseTrip(Trip trip);
  void startCorrective();
  bool correctiveAllowed() const;
  std::vector<Target> sessionDiff() const;
  void monitorTick();
  void tick();
  void tickLoop(int gen);
  rt::Task<void> tickStep();
  rt::Task<void> observeStep();
  void nextAxis();
  void startRamp(const std::string& axis);
  rt::Task<void> rampStep();
  rt::Task<void> rampWrite(std::shared_ptr<Job> job);
  rt::Task<void> coupleI(const std::string& axis, double P, std::shared_ptr<Job> job);
  void onDisarm();
  Json gainsOf(const std::string& axis) const;
  Json changeJson(const Change& c) const;

  std::optional<Ramp> ramp_;
  std::uint64_t timer_ = 0, monitorTimer_ = 0;
  int loopGen_ = 0;
  std::string sessionDir_, sessionFile_;
  int linkGen_ = -1, preparedGen_ = -1;
  bool armedSeen_ = false;
  std::shared_ptr<Job> job_;
  int jobSeq_ = 0, changeSeq_ = 0;
  std::optional<Trip> pendingTrip_;
  std::map<std::string, bool> tripLatch_;
  Json sessionBase_;   // {t, values: {name: value}} or null
  struct AttSample {
    double t, roll, pitch;
  };
  std::deque<AttSample> attHist_, attErr_;
};

}  // namespace aptuner::app
