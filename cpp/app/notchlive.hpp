// Live harmonic-notch setup through the vehicle: the autopilot's in-flight FFT notch tune (RC option 162 over
// MAVLink), ESC RPM telemetry tracking (mode 3), or the log route. Port of src/engine/notchlive.js.
#pragma once

#include <string>
#include <vector>

#include "aptuner/core/json.hpp"
#include "aptuner/io/link.hpp"
#include "aptuner/rt/async.hpp"
#include "tuner.hpp"

namespace aptuner::app {

extern const std::vector<std::string> kNotchParams;

class NotchLive {
 public:
  NotchLive(io::Link& link, Tuner& tuner);
  rt::Task<Json> status();
  rt::Task<Json> applyEsc();
  rt::Task<Json> enableFft();
  rt::Task<Json> reboot();
  rt::Task<Json> startFftTune();
  rt::Task<Json> stopFftTune();

  bool escSeen() const { return escSeen_; }
  double escLastTs() const { return escLastTs_; }

 private:
  void onEsc(const Json& f);
  io::Link& link_;
  Tuner& tuner_;
  bool escSeen_ = false;
  double escLastTs_ = 0;
  struct RpmSample {
    double t, mean;
    int n;
  };
  std::vector<RpmSample> rpm_;
  Json fftMessages_ = Json::array();
  Json tune_;
};

}  // namespace aptuner::app
