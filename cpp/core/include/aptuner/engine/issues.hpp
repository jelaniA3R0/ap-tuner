// Plain-language issues for an axis result: what is wrong (from the flight data) and what the proposal does about
// it. Port of src/engine/issues.js. Each issue: {level: "bad" | "warn" | "ok", text, fix?}.
#pragma once

#include "aptuner/core/json.hpp"
#include "aptuner/core/rules.hpp"

namespace aptuner::engine {

core::Json axisIssues(const core::Json& r, core::Family family);
/** vehicle-level issues from the setup checks: [{ok, title, detail}] -> issues for the failed ones */
core::Json vehicleIssues(const core::Json& checks);

}  // namespace aptuner::engine
