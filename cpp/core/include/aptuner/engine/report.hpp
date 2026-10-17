// Parameter-file builder (Mission Planner / QGC .param format: NAME,VALUE per line; # comments).
// Port of src/engine/report.js.
#pragma once

#include <string>
#include <vector>

#include "aptuner/core/json.hpp"
#include "aptuner/core/rules.hpp"

namespace aptuner::engine {

/** rows [{group, name, value, from?, label, pct?}] from the axes of an analysis result and the noise result */
core::Json proposalRows(core::Family family, const core::Json& axes, const core::Json& noise);
/** the .param text (CRLF line ends) */
std::string paramFile(const core::Json& rows, const std::vector<std::string>& header = {});

}  // namespace aptuner::engine
