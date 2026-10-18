// Parameter-file builder (Mission Planner / QGC .param format: NAME,VALUE per line; # comments). Every value is checked
// before it goes into a file (2026-10-17 audit F-43) and written in fixed point; the file names the vehicle it is for.
// Port of src/engine/report.js.
#pragma once

#include <string>
#include <vector>

#include "aptuner/core/json.hpp"
#include "aptuner/core/rules.hpp"

namespace aptuner::engine {

/** rows [{group, name, value, from?, label, pct?, reason}] from the axes of an analysis result and the noise result;
    params: the values in the log (the notch rows' "from"), null when not known */
core::Json proposalRows(core::Family family, const core::Json& axes, const core::Json& noise, const core::Json& params = core::Json());

struct ParamFileOut {
  std::string text;
  core::Json errors = core::Json::array(), warnings = core::Json::array();   // [{name, value, why}] / [{name, why}]
};
/** the .param text (CRLF line ends) and what the checks found. strict: a row that fails a check is left out (a comment
    says why); not strict (an UNDO file of values read from the autopilot): every row is kept, findings become comments.
    identity: the vehicle the file is for (an identity object, a sentence, or null) */
ParamFileOut paramFileChecked(const core::Json& rows, const std::vector<std::string>& header = {}, const core::Json& identity = core::Json(), bool strict = true);
std::string paramFile(const core::Json& rows, const std::vector<std::string>& header = {}, const core::Json& identity = core::Json(), bool strict = true);

/** the vehicle a log comes from: {sysid, fw, gitHash, firmwareText} (SYSID_THISMAV or, from ArduPilot 4.7 on, MAV_SYSID; the version and hash
    of its MSG text) */
core::Json logIdentity(const core::Json& result);
/** the sentence naming the vehicle of a log (the header of a .param file made from it) */
std::string logIdentityText(const core::Json& result);
/** does the log come from the vehicle on this connection? {log, problems: [text] (writing is refused), notes: [text]} */
core::Json logVehicleCheck(const core::Json& result, const core::Json& identity);

}  // namespace aptuner::engine
