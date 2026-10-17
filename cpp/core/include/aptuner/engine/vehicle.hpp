// Vehicle family detection and parameter access helpers. Port of src/engine/vehicle.js.
#pragma once

#include <cmath>
#include <optional>
#include <string>

#include "aptuner/core/json.hpp"
#include "aptuner/core/rules.hpp"
#include "aptuner/log/dfreader.hpp"

namespace aptuner::engine {

using core::Axis;
using core::Family;
using core::Json;

/** family from the MAVLink heartbeat type and parameters (plane with Q_ENABLE in a Q mode -> vtol) */
std::optional<Family> detectFamily(int mavType, const std::string& modeName, const log::Params& params);
bool isVtolMode(const std::string& modeName);

/**
 * Gains of one axis as a JSON object with the JavaScript keys (P I D FF FLTT FLTE FLTD SMAX IMAX D_FF DFF);
 * a missing parameter leaves its key out (FLTT / FLTE / FLTD / FF / D_FF / SMAX default to 0).
 */
Json axisGains(Family f, Axis axis, const log::Params& params);

/** parameter names the tuner may write for an axis: {P: "...", I: ..., angleP: ..., ...} */
Json axisParamNames(Family f, Axis axis);

/** ArduPlane speed scaler (Plane::calc_speed_scaler) */
double planeSpeedScaler(const log::Params& params, double airspeed, double throttlePct);

/** JavaScript-style access on a JSON object of numbers: missing / non-number -> NaN (like undefined) */
inline double jv(const Json& o, const char* key) {
  const Json& v = o[key];
  return v.isNumber() ? v.num() : std::nan("");
}
/** o[key] || d */
inline double jor(const Json& o, const char* key, double d) {
  const double v = jv(o, key);
  return (v != 0 && !std::isnan(v)) ? v : d;
}
/** o[key] ?? d */
inline double jnn(const Json& o, const char* key, double d) {
  const Json& v = o[key];
  return v.isNumber() ? v.num() : d;
}

}  // namespace aptuner::engine
