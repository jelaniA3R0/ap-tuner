// JavaScript number formatting, so the C++ version writes exactly what the JavaScript version wrote
// (parameter files, reports, API values): Number.prototype.toString, toFixed and toPrecision.
#pragma once

#include <string>

namespace aptuner::core {

/** String(x) / x.toString(): shortest round-trip digits, JS exponent rules ("1e-7", "123.456", "1e+21") */
std::string jsToString(double x);

/** x.toFixed(digits) (0..20 digits, exact decimal rounding with ties away from zero, as the spec says) */
std::string jsToFixed(double x, int digits);

/** x.toPrecision(precision) (1..21 significant digits) */
std::string jsToPrecision(double x, int precision);

/** +x.toFixed(d): the number toFixed would print */
double jsRoundFixed(double x, int digits);
/** +x.toPrecision(p) */
double jsRoundPrecision(double x, int precision);

}  // namespace aptuner::core
