// "Telemetry + USB" session for vehicles flown on a slow telemetry radio: flight (telemetry only marks the time)
// -> capture -> land -> usb (the app finds the autopilot's USB port) -> download (log head + the marked window)
// -> review -> write (on the ground, disarmed, each value read back) -> done (undo file kept).
// Port of src/engine/groundtune.js.
#pragma once

#include <optional>
#include <set>
#include <string>

#include "aptuner/core/json.hpp"
#include "aptuner/io/link.hpp"
#include "aptuner/io/logdl.hpp"
#include "aptuner/rt/async.hpp"
#include "aptuner/rt/emitter.hpp"

namespace aptuner::app {

using core::Json;

class GroundTune : public rt::Emitter {
 public:
  GroundTune(io::Link& link, const std::string& sessionsDir);
  void reset();
  void start(const Json& opts);
  void stop();
  void captureAgain();
  rt::Task<void> goUsb(bool useSaved);
  rt::Task<Json> write(const std::vector<std::string>& names);
  rt::Task<void> undo();
  Json snapshot() const;

  Json opts;
  Json result;   // last analysis (null until downloaded)
  Json files = Json::object();

 private:
  std::string markFile() const;
  void saveMark();
  void restoreMark();
  std::optional<double> nowBootMs() const;
  std::string familyOf(const Json& v) const;
  bool modeOk(const Json& v) const;
  void setPhase(const std::string& p, const std::string& info = "");
  void note(const std::string& level, const std::string& text);
  void stopTimer();
  void tick();
  void loopOnce();
  rt::Task<void> step();
  rt::Task<void> enterUsb();
  rt::Task<void> scanUsb();
  rt::Task<void> download();
  void fail(const std::string& msg);

  io::Link& link_;
  io::LogDownloader dl_;
  std::string dir_;
  double bootMs_ = -1, bootAt_ = 0;
  std::string phase_ = "idle", info_;
  double since_ = 0, tFlying_ = 0, usbSince_ = 0;
  Json mark_, savedMark_, captureStart_, usb_, progress_, rows_ = Json::array(), writes_ = Json::array(), notes_ = Json::array(), error_;
  std::optional<int> sysid_;
  std::set<std::string> knownPorts_, busy_, tried_;
  std::uint64_t timer_ = 0;
};

}  // namespace aptuner::app
