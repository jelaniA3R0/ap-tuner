// Guided flight session, one flight led by the app: preflight -> checkup -> review -> tuning -> filters ->
// recheck -> overall -> land -> finished. Port of src/engine/flight.js.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "aptuner/core/json.hpp"
#include "aptuner/io/link.hpp"
#include "aptuner/rt/async.hpp"
#include "aptuner/rt/emitter.hpp"
#include "notchlive.hpp"
#include "tuner.hpp"

namespace aptuner::app {

/** flight modes that use the analysed controller, per family */
const std::vector<std::string>& goodModes(const std::string& family);
extern const std::vector<std::string> kFlightPhases;

class FlightSession : public rt::Emitter {
 public:
  FlightSession(io::Link& link, Tuner& tuner, NotchLive& notch);
  void reset();
  void start(const Json& opts);
  void stop();
  void cont();   // "continue"
  void repeatCheckup();
  void skipFilters();
  void resume();
  Json snapshot() const;

 private:
  std::string family() const;
  bool quadplane() const;
  const std::vector<std::string>& modes() const;
  std::string goodModesText() const;
  bool modeOk() const;
  void setPhase(const std::string& p, const std::string& info = "");
  void note(const std::string& level, const std::string& text);
  void pauseFlight(const std::string& reason);
  void tick();
  void loopOnce();
  rt::Task<void> step();
  void startTuning();
  rt::Task<void> startFilters();
  void startRecheck();
  void startOverall();
  Json summarise(const Json& res) const;
  Json verdict(const Json& ov) const;
  void finish(const std::string& reason);
  Json gainsNow() const;

  io::Link& link_;
  Tuner& tuner_;
  NotchLive& notch_;
  std::string phase_ = "idle", info_, resumePhase_;
  double since_ = 0, tFlying_ = 0;
  Json opts_, checkup_, filters_, overall_, summary_, notes_ = Json::array(), pausedInfo_, gainsStart_;
  std::size_t journalStart_ = 0;
  std::uint64_t timer_ = 0;
};

}  // namespace aptuner::app
