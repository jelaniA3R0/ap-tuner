// What the app knows about the ArduPilot parameters it writes or exports (2026-10-17 audit F-43, F-35, Q12): the
// storage type (whole number or float), the range ArduPilot documents (@Range: advisory, outside it is a warning), the
// limits the app accepts at all (outside them the value is refused), enumerations, the highest documented bit of a
// bitmask and @RebootRequired. Port of src/model/parammeta.js (the same table).
#pragma once

#include <string>
#include <vector>

#include "aptuner/core/json.hpp"

namespace aptuner::core {

struct ParamCheck {
  bool ok = true;
  std::vector<std::string> errors, warnings;
  bool reboot = false, isInt = false;
  /** {ok, errors, warnings, reboot, int} as the JavaScript validateParam returns it */
  Json toJson() const;
};

/** the value takes effect only after the autopilot reboots (ArduPilot @RebootRequired) */
bool isRebootRequired(const std::string& name);
/** the checks of a value about to be written or exported (value: a number; anything else "is not a number") */
ParamCheck validateParam(const std::string& name, const Json& value);
/** a value for a .param file: fixed point, never exponent notation; whole numbers for integer parameters; floats with
    the fewest digits that give back the same float32 */
std::string formatParamValue(const std::string& name, double value);

}  // namespace aptuner::core
