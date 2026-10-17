// Try the usual places for a vehicle: Mission Planner mirror (TCP 14550), SITL ports, UDP 14550, then every
// serial port at the common telemetry speeds. Port of autoconnect() in src/server.js, including the
// "wait until the port is really open" fix and the busy / silent port bookkeeping.
#pragma once

#include <functional>

#include "aptuner/core/json.hpp"
#include "aptuner/io/link.hpp"
#include "aptuner/rt/async.hpp"

namespace aptuner::io {

struct AutoConnectState {
  bool running = false;
  core::Json tried = core::Json::array();   // [{label, result, busy}]
  core::Json found;                         // the candidate that answered (null if none)
  bool missionPlanner = false;
  core::Json busyPorts = core::Json::array();
  std::string current;
};

/** resolves the candidate that answered (null when nothing did); progress({trying} | {found}) */
rt::Task<core::Json> autoconnect(Link& link, AutoConnectState& state, std::function<void(const core::Json&)> onProgress);

}  // namespace aptuner::io
