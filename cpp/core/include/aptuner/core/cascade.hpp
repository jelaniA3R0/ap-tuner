// Model of one ArduPilot attitude axis (multirotor / QuadPlane VTOL, ArduCopter / ArduPlane 4.7) from the pilot's
// stick to the vehicle, for heavy and slow vehicles whose angle loop and rate loop interact (the "cascade"):
//
//   pilot input shaping (INPUT_TC / PLT_Y_RATE_TC, ACC_*_MAX)  ->  angle controller (sqrt controller with angle P and
//   clamp(ACC/2), correction limited to RATE_*_MAX, feed-forward added; yaw heading error limited as
//   AC_AttitudeControl::thrust_heading_rotation_angles does)  ->  rate PID (AC_PID::update_all: FLTT, FLTE, D on the
//   error filtered by FLTD, integrator frozen in the saturated direction, IMAX)  ->  mixer limit u_max  ->
//   transport delay T  ->  motor lag tau  ->  vehicle  rate' = a (u - trim) - c rate  (+ gusts)
//   gyro: 2-pole Butterworth INS_GYRO_FILTER.
//
// Linear analysis breaks the loop at the plant input:  L(s) = G(s) FE(s) Cr(s) (FT(s) K / s + Fg(s)),
// G = a e^(-sT) / ((s + c)(1 + tau s)). Phase margin at the last gain crossover (unwrapped phase); gain margin only at
// the first -180 deg crossing above it (the low-frequency crossing of the rate-I + angle cascade is harmless).
#pragma once

#include <complex>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace aptuner::core::cascade {

using Cx = std::complex<double>;
using Vec = std::vector<double>;

inline constexpr double kD2R = 0.017453292519943295;

enum class Kind { RollPitch, Yaw };

/** the parameters of one axis as ArduPilot names them (rate gains per rad/s; limits in deg, deg/s, deg/s/s) */
struct Gains {
  Kind kind = Kind::RollPitch;
  double P = 0, I = 0, D = 0, FF = 0, IMAX = 0.5;
  double FLTT = 20, FLTE = 0, FLTD = 20;   // Hz, 0 = off
  double angleP = 4.5;                     // ANG_*_P
  double accMax = 0;                       // ACC_*_MAX deg/s/s (0 = no sqrt controller / no shaping limit)
  double rateMax = 0;                      // RATE_*_MAX deg/s (0 = none)
  double inputTC = 0.15;                   // INPUT_TC (roll / pitch) s
  double gyroHz = 20;                      // INS_GYRO_FILTER
  double loopHz = 300;                     // SCHED_LOOP_RATE
  // yaw pilot input (QuadPlane Q_PLT_Y_* / Copter PILOT_Y_*), stick channel (RCn_MIN / TRIM / MAX / DZ)
  double pltRate = 100, pltExpo = 0, pltRateTC = 0.25;
  double rcMin = 1000, rcTrim = 1500, rcMax = 2000, rcDz = 30;
};

/** the vehicle on one axis: rate' = a (u_lagged_delayed - trim) - c rate */
struct Plant {
  double a = 1;       // rad/s/s per unit of controller output
  double c = 0;       // aerodynamic / rotor damping 1/s
  double tau = 0.05;  // motor (+ ESC) lag s
  double T = 0.01;    // transport delay s
  double trim = 0;    // steady output needed (torque imbalance, CG offset) in output units
  double umax = 1;    // mixer limit of this axis' output
  std::string name;
};

// ------------------------------------------------------------------------------------------------- linear analysis

/** loop transfer function broken at the plant input, at w rad/s */
Cx loopAt(const Gains& g, const Plant& p, double w);
/** the controller part C(jw) = FE Cr (FT K / s + Fg) (output per rad of attitude / rad/s of rate feedback) */
Cx controllerAt(const Gains& g, double w);
/** gyro noise to controller output: |FE Cr Fg| at f Hz */
double noiseGain(const Gains& g, double fHz);

struct Margins {
  std::optional<double> pmDeg;   // phase margin at the last gain crossover (none: |L| never crosses 1)
  double gmDb = 99;              // upper gain margin (99: no -180 crossing above the crossover)
  double ms = 0;                 // max |1 / (1 + L)|
  double wcRad = 0;              // last gain crossover (rad/s)
  double w180Rad = 0;            // the -180 crossing used for the gain margin (0: none)
};
Margins margins(const Gains& g, const Plant& p);

struct WorstMargins {
  double pmDeg = 1e9, gmDb = 1e9, ms = 0;
  double wcAtWorstPmRad = 0, slowestWcRad = 1e9;
  std::size_t worstPlant = 0;
};
/** worst case over a plant set; stopBelowPm: stop early once the worst PM falls below it (screening) */
WorstMargins worstMargins(const Gains& g, const std::vector<Plant>& set, double stopBelowPm = -1e9);

/** closed-loop poles of a slow axis (rate PI + angle P on rate' = a u - c rate):
    s^3 + (aP + c) s^2 + a (P K + I) s + a I K = 0 */
struct Poles {
  std::vector<Cx> roots;
  double zeta = 1, periodS = 0, wnRad = 0;   // of the least damped complex pair (zeta 1, period 0: none)
  bool stable = true;
};
Poles slowAxisPoles(double a, double c, double P, double I, double K);

/**
 * Marginal-mode identification: when the loop sits on the edge of stability at w0 (a persistent oscillation), L(jw0) ~ -1,
 * so G(jw0) = -1 / C(jw0). With G = a e^(-sT) / ((s + c)(1 + tau s)): |G| gives a, the phase gives tau + T.
 * Returns a and tau for a given transport delay T (nullopt when the phase does not fit this plant form).
 */
struct MarginalFit {
  double a = 0, tau = 0, T = 0, lagDeg = 0;
};
std::optional<MarginalFit> marginalModeFit(const Gains& flown, double w0, double T, double c = 0);

// ------------------------------------------------------------------------------------------------- time domain

/** ArduPilot sqrt_controller (AP_Math/control.cpp) */
double sqrtController(double error, double p, double secondOrdLim, double dt);
/** ArduPilot inv_sqrt_controller */
double invSqrtController(double output, double p, double dMax);
/** the largest yaw heading error the attitude controller keeps (rad): min(inv_sqrt_controller(1/rate P, angle P,
    clamp(ACC_Y/2, 10..120 deg/s/s)), 45 deg) */
double yawHeadingErrorMax(const Gains& g);
/** the acceleration limit of the angle controller (rad/s/s): clamp(ACC/2, 40..720) roll/pitch, (10..120) yaw; 0 = linear */
double angleControllerAccel(const Gains& g);
/** yaw stick PWM to the pilot's yaw-rate demand (rad/s): dead zone, expo, PLT_Y_RATE */
double yawStickToRate(double pwm, const Gains& g);

/** reproducible gust torque (output units): the filtered random sequence the hand analysis used (seed 7, ~1 Hz band) */
class GustSequence {
 public:
  explicit GustSequence(double seconds, double hz = 300, std::uint32_t seed = 7);
  /** gust in output units at time t (scaled so that a = 5.8 rad/s/s per unit gives the reference torque) */
  double at(double t) const;
 private:
  Vec v_;
  double hz_;
};

struct SimInput {
  double seconds = 30;
  /** yaw: stick PWM over time (empty: centred) */
  std::function<double(double)> stickPwm;
  /** roll / pitch: desired angle (deg) over time; shaped = already shaped by the autopilot (ATT.Des*) */
  std::function<double(double)> angleDeg;
  bool shaped = false;
  /** extra disturbance in output units (gusts, a yaw gust step) */
  std::function<double(double)> dist;
  double extraDelay = 0;      // stress: + transport delay s
  double lead = 0;            // stress: rotor-inertia kick (output-rate lead, s)
  double gyroNoiseDegs = 0;   // stress: white gyro noise (deg/s RMS)
  std::uint32_t seed = 12345;
  double recordHz = 30;
  double initialAngleDeg = 0, initialI = 0;
};
struct SimOutput {
  Vec t, att, rate, u, I, target;   // s, deg, deg/s, output, I term, attitude target deg (wrapped for yaw)
  double limitedFrac = 0;           // share of time the output was at the mixer limit
  double outputJitter = 0;          // RMS output change per second over the last half (noise reaching the motors)
};
SimOutput simulate(const Gains& g, const Plant& p, const SimInput& in);

/** wrap an angle in degrees to (-180, 180] */
double wrap180(double deg);

}  // namespace aptuner::core::cascade
