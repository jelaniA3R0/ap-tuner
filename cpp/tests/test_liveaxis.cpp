// The step watchdog judges only the samples from after a gain change (LiveAxis `since`): an oscillation that was
// there before a change must not be blamed on the change. Same cases as test/watchdog.test.js.
#include <cmath>
#include <numbers>

#include "check.hpp"
#include "liveaxis.hpp"

using aptuner::core::Json;
using aptuner::app::LiveAxis;

TEST(liveaxis_watchdog_since) {
  LiveAxis ax("roll");
  // 10 s at 50 Hz: slew rate 7 (above the limit of 5) until t = 1006 s, calm (1) after
  for (int i = 0; i < 500; i++) {
    const double t = 1000 + i / 50.0;
    ax.push(t, Json::obj({{"desired", 0}, {"achieved", 0}, {"P", 0}, {"I", 0}, {"D", 0}, {"FF", 0},
                          {"SRate", t < 1006 ? 7 : 1}, {"PDmod", 1}}));
  }
  const auto all = ax.watchdog(5, 5);
  CHECK(all.oscillating);
  CHECK_NEAR(all.srateMax, 7, 0);
  const auto after = ax.watchdog(5, 5, 8, 0.9, 1006.0);
  CHECK(!after.oscillating);
  CHECK_NEAR(after.srateMax, 1, 0);
  const auto none = ax.watchdog(2, 5, 8, 0.9, 1100.0);
  CHECK(!none.oscillating);
  CHECK(none.samples == 0);
  const auto w = ax.window(0, 1008.0);
  CHECK(w.has_value());
  if (w) CHECK_NEAR(static_cast<double>(w->n) / w->fs, 2, 0.1);
}

TEST(liveaxis_watchdog_spectral) {
  // 10 s at 100 Hz with an 8 Hz rate-error oscillation of 20 deg/s
  LiveAxis osc("pitch");
  for (int i = 0; i < 1000; i++) {
    const double t = 1000 + i / 100.0;
    osc.push(t, Json::obj({{"desired", 0}, {"achieved", 20 * std::sin(2 * std::numbers::pi * 8 * t)}, {"P", 0}, {"I", 0}, {"D", 0},
                           {"FF", 0}, {"SRate", 1}, {"PDmod", 1}}));
  }
  const auto full = osc.watchdog(2, 5);
  CHECK(full.oscillating);
  CHECK(full.peakHz > 7 && full.peakHz < 9);
  CHECK(!osc.watchdog(2, 5, 8, 0.9, osc.lastTs - 1).oscillating);    // no spectral verdict on 1 s after `since`
  CHECK(osc.watchdog(2, 5, 8, 0.9, osc.lastTs - 3.5).oscillating);   // a full window came in after it
}
