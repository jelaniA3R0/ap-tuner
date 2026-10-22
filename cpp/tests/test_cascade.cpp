// The heavy-vehicle axis model (core/cascade): margins, the sqrt controller, the slow-axis poles, marginal-mode
// identification and the nonlinear simulation, checked against the hand analysis of the tandem QuadPlane
// (research/tandem-2026-10: appcheck.js, roots.js, pvalid.js).
#include <algorithm>
#include <cmath>
#include <vector>

#include "aptuner/core/cascade.hpp"
#include "check.hpp"

using namespace aptuner::core::cascade;

namespace {

Gains rp(double P, double I, double D, double K, double gyro, double fltt, double fltd, double acc = 400, double tc = 0.15) {
  Gains g;
  g.kind = Kind::RollPitch;
  g.P = P;
  g.I = I;
  g.D = D;
  g.angleP = K;
  g.gyroHz = gyro;
  g.FLTT = fltt;
  g.FLTD = fltd;
  g.FLTE = 0;
  g.accMax = acc;
  g.inputTC = tc;
  g.rateMax = 75;
  g.IMAX = 0.5;
  return g;
}

std::vector<Plant> set(std::vector<double> as) {
  std::vector<Plant> s;
  for (double a : as)
    for (double tau : {0.05, 0.08, 0.11})
      for (double T : {0.01, 0.025}) {
        Plant p;
        p.a = a;
        p.tau = tau;
        p.T = T;
        s.push_back(p);
      }
  return s;
}

Gains yawGains(double P, double I, double imax, double K, double rmax, double acc, double plt) {
  Gains g;
  g.kind = Kind::Yaw;
  g.P = P;
  g.I = I;
  g.IMAX = imax;
  g.angleP = K;
  g.FLTT = 20;
  g.FLTE = 2.5;
  g.FLTD = 20;
  g.gyroHz = 20;
  g.rateMax = rmax;
  g.accMax = acc;
  g.pltRate = plt;
  g.pltExpo = 0.25;
  g.pltRateTC = 0.25;
  g.rcMin = 992;
  g.rcMax = 2005;
  return g;
}

double peakAbs(const SimOutput& s, double tMax) {
  double pk = 0;
  for (std::size_t k = 0; k < s.t.size(); ++k)
    if (s.t[k] < tMax) pk = std::max(pk, std::fabs(s.att[k]));
  return pk;
}

}  // namespace

TEST(cascade_pitch_margins_match_hand_analysis) {
  // appcheck.js: pitch a 4.8-7: stock -11 deg, the app's earlier proposal 6 deg, the flown set 42 deg
  const auto pitch = set({4.8, 5.8, 7.0});
  CHECK_NEAR(worstMargins(rp(0.25, 0.25, 0.0036, 4.5, 20, 20, 10), pitch).pmDeg, -11, 2);
  CHECK_NEAR(worstMargins(rp(0.5, 0.5, 0.0054, 3, 20, 20, 10), pitch).pmDeg, 6, 2);
  const WorstMargins flown = worstMargins(rp(0.5, 0.15, 0.009, 1, 10, 10, 5, 100, 0.2), pitch);
  CHECK_NEAR(flown.pmDeg, 42, 2);
  CHECK(flown.ms < 1.8);
  // roll a 7-13: stock -6, flown 40
  const auto roll = set({7, 9.5, 13});
  CHECK_NEAR(worstMargins(rp(0.25, 0.25, 0.0036, 4.5, 20, 20, 10), roll).pmDeg, -6, 2);
  CHECK_NEAR(worstMargins(rp(0.4, 0.1, 0.009, 1, 10, 10, 5, 100, 0.2), roll).pmDeg, 40, 2);
}

TEST(cascade_gain_margin_ignores_low_frequency_crossing) {
  // the rate-I + angle-P cascade passes -180 deg far below the crossover with |L| >> 1: conditionally stable, harmless.
  // The upper gain margin must come from a crossing above the crossover.
  const Gains g = rp(0.5, 0.15, 0.009, 1, 10, 10, 5, 100, 0.2);
  Plant p;
  p.a = 5.8;
  p.tau = 0.08;
  p.T = 0.02;
  const Margins m = margins(g, p);
  CHECK(m.pmDeg.has_value());
  CHECK(m.w180Rad > m.wcRad);
  CHECK(m.gmDb >= 10);
  // there is a -180 crossing below the crossover with |L| > 1 (the case the first hand draft counted by mistake)
  bool lowCrossing = false;
  double prev = std::arg(loopAt(g, p, 0.03));
  for (double w = 0.03; w < m.wcRad; w *= 1.003) {
    const auto L = loopAt(g, p, w);
    const double a = std::arg(L);
    if (std::fabs(a - prev) > 3 && std::abs(L) > 1) lowCrossing = true;   // the principal value jumps at +-180 deg
    prev = a;
  }
  CHECK(lowCrossing);
}

TEST(cascade_sqrt_controller) {
  const double dt = 1.0 / 300;
  // linear region: error * P
  CHECK_NEAR(sqrtController(0.01, 4.5, 3.49, dt), 0.045, 1e-12);
  // beyond acc / P^2: sqrt(2 acc (e - acc / 2P^2))
  const double acc = 3.49, p = 4.5, e = 0.5;
  CHECK_NEAR(sqrtController(e, p, acc, dt), std::sqrt(2 * acc * (e - acc / (2 * p * p))), 1e-12);
  CHECK_NEAR(sqrtController(-e, p, acc, dt), -std::sqrt(2 * acc * (e - acc / (2 * p * p))), 1e-12);
  // the inverse gives back the error
  for (double err : {0.02, 0.3, 1.2}) CHECK_NEAR(invSqrtController(sqrtController(err, p, acc, 0), p, acc), err, 1e-9);
  // never overshoots in one step
  CHECK_NEAR(sqrtController(1e-4, 4.5, 0, 1e-6), 4.5e-4, 1e-12);
  // yaw heading error limit (AC_AttitudeControl::thrust_heading_rotation_angles): 45 deg at stock gains, less at high rate P
  CHECK_NEAR(yawHeadingErrorMax(yawGains(0.18, 0.018, 0.5, 4.5, 75, 100, 100)) / kD2R, 45, 1e-9);
  const double step2 = yawHeadingErrorMax(yawGains(2.5, 0.19, 0.7, 1.0, 20, 15, 20)) / kD2R;
  CHECK(step2 > 28 && step2 < 34);
}

TEST(cascade_slow_axis_poles) {
  // roots.js: stock yaw gains on a 0.20, c 0.165: zeta 0.12, period 16 s (the swing seen in every hover)
  const Poles stock = slowAxisPoles(0.20, 0.165, 0.18, 0.018, 4.5);
  CHECK(stock.stable);
  CHECK_NEAR(stock.zeta, 0.12, 0.01);
  CHECK_NEAR(stock.periodS, 16.1, 0.3);
  const Poles step2 = slowAxisPoles(0.20, 0.165, 2.5, 0.19, 1.0);
  CHECK_NEAR(step2.zeta, 0.42, 0.01);
  CHECK_NEAR(step2.periodS, 9.9, 0.2);
}

TEST(cascade_marginal_mode_identification) {
  // a loop made exactly marginal at its phase crossover: the fit must give back the plant
  const Gains flown = rp(0.25, 0.25, 0.0036, 4.5, 20, 20, 10);
  for (double tau : {0.06, 0.09})
    for (double T : {0.01, 0.025}) {
      Plant p;
      p.a = 5.8;
      p.tau = tau;
      p.T = T;
      // the phase crossover near the oscillation (1-8 rad/s), above the low-frequency cascade crossing
      double w0 = 0;
      for (double w = 1; w < 8 && w0 == 0; w *= 1.001) {
        const auto a = loopAt(flown, p, w), b = loopAt(flown, p, w * 1.001);
        if (a.real() < 0 && b.real() < 0 && (a.imag() > 0) != (b.imag() > 0)) w0 = w;
      }
      CHECK(w0 > 0);
      const double k = 1 / std::abs(loopAt(flown, p, w0));
      Plant marginal = p;
      marginal.a = p.a * k;
      CHECK_NEAR(std::abs(1.0 + loopAt(flown, marginal, w0)), 0, 0.02);
      const auto fit = marginalModeFit(flown, w0, T);
      CHECK(fit.has_value());
      if (fit) {
        CHECK_REL(fit->a, marginal.a, 0.03);
        CHECK_NEAR(fit->tau, tau, 0.004);
      }
    }
}

TEST(cascade_yaw_takeoff_swing_matches_hand_analysis) {
  // appcheck.js (validated yaw model a 0.2, c 0.165, tau 0.2, trim 0.42, u_max 0.63): swing 107 / 41 / 11 deg
  Plant p;
  p.a = 0.2;
  p.c = 0.165;
  p.tau = 0.2;
  p.T = 0;
  p.trim = 0.42;
  p.umax = 0.63;
  SimInput in;
  in.seconds = 60;
  const double stock = peakAbs(simulate(yawGains(0.18, 0.018, 0.5, 4.5, 75, 100, 100), p, in), 40);
  const double app = peakAbs(simulate(yawGains(0.36, 0.036, 0.75, 3, 75, 100, 100), p, in), 40);
  const double step2 = peakAbs(simulate(yawGains(2.5, 0.19, 0.7, 1, 20, 15, 20), p, in), 40);
  CHECK_REL(stock, 107, 0.08);
  CHECK_REL(app, 41, 0.1);
  CHECK_NEAR(step2, 11, 1.5);
}

TEST(cascade_pitch_oscillation_in_simulation) {
  // pvalid.js: stock pitch gains + gusts sit in a ~0.4-0.45 Hz oscillation
  const Gains g = rp(0.25, 0.25, 0.0036, 4.5, 20, 20, 10);
  Plant p;
  p.a = 5.8;
  p.tau = 0.08;
  p.T = 0.02;
  p.umax = 0.45;
  const GustSequence gust(60);
  SimInput in;
  in.seconds = 60;
  in.shaped = true;
  in.angleDeg = [](double) { return 0.0; };
  in.dist = [&](double t) { return -gust.at(t); };
  in.recordHz = 100;
  const SimOutput s = simulate(g, p, in);
  std::vector<double> late;
  for (std::size_t k = 0; k < s.t.size(); ++k)
    if (s.t[k] > 20) late.push_back(s.att[k]);
  int zc = 0;
  for (std::size_t k = 1; k < late.size(); ++k)
    if ((late[k] > 0) != (late[k - 1] > 0)) ++zc;
  const double f = zc / 2.0 / 40.0;
  CHECK(f > 0.36 && f < 0.5);
  const double amp = (*std::max_element(late.begin(), late.end()) - *std::min_element(late.begin(), late.end())) / 2;
  CHECK(amp > 5);
  // the flown set on the same plant and gusts: calm
  const SimOutput s2 = simulate(rp(0.5, 0.15, 0.009, 1, 10, 10, 5, 100, 0.2), p, in);
  double pk = 0;
  for (std::size_t k = 0; k < s2.t.size(); ++k)
    if (s2.t[k] > 20) pk = std::max(pk, std::fabs(s2.att[k]));
  CHECK(pk < amp / 2);
}
