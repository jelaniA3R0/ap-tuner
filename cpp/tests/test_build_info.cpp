// Phase 0 smoke tests: the library links, reports its version, and the numeric environment behaves as the
// ported math will assume (IEEE doubles, std::sin/cos accuracy used by the filter models).
#include <cmath>
#include <limits>
#include <numbers>

#include "aptuner/core/build_info.hpp"
#include "aptuner/version.hpp"
#include "check.hpp"

TEST(build_info_reports_version) {
  const auto b = aptuner::core::buildInfo();
  CHECK(b.version == aptuner::kVersion);
  CHECK(!b.compiler.empty());
  CHECK(!b.platform.empty());
  CHECK(b.cppStandard >= 202002);
  CHECK(aptuner::core::buildInfoLine().find(aptuner::kVersion) != std::string::npos);
}

TEST(numeric_environment) {
  CHECK(std::numeric_limits<double>::is_iec559);
  CHECK_NEAR(std::sin(std::numbers::pi / 6), 0.5, 1e-15);
  // a 2-pole Butterworth coefficient as the gyro filter model computes it (fc 20 Hz at 1 kHz)
  const double ohm = std::tan(std::numbers::pi * 20.0 / 1000.0);
  const double c = 1.0 + 2.0 * std::cos(std::numbers::pi / 4) * ohm + ohm * ohm;
  CHECK_REL(ohm * ohm / c, 0.0036216815149286421, 1e-12);   // value from the JavaScript version
}
