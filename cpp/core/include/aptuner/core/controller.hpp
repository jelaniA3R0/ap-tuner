// Frequency-domain models of the ArduPilot control chain on an arbitrary frequency grid (Hz).
// Port of src/model/controller.js:
//  - AC_PID (copter / plane rate PID): target LPF (FLTT), error LPF (FLTE), D on the filtered error with its own
//    LPF (FLTD), integrator, FF on the filtered target; discrete at the loop rate (AC_PID::update_all).
//  - Gyro chain: 2-pole low-pass INS_GYRO_FILTER (DigitalBiquadFilter) + harmonic notch sets (INS_HNTCH_*,
//    INS_HNTC2_*), discrete at the gyro backend rate.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "aptuner/core/cx.hpp"

namespace aptuner::core {

/** ArduPilot first-order LPF y += alpha (x - y); fc <= 0 -> pass-through */
CVec lpf1(const Vec& freqs, double fs, double fc);
/** 2-pole Butterworth low-pass (LowPassFilter2p / DigitalBiquadFilter) */
CVec biquadLpf(const Vec& freqs, double fs, double fc);
CVec biquad(const Vec& freqs, double fs, double b0, double b1, double b2, double a0, double a1, double a2);
/** ArduPilot NotchFilter: centre, bandwidth (Hz), attenuation (dB) */
CVec notch(const Vec& freqs, double fs, double center, double bw, double attDb);
/** quality factor of an ArduPilot notch from its centre and bandwidth (NotchFilter::calculate_A_and_Q) */
double notchQ(double center, double bw);
/** NotchFilter with a given Q (NotchFilter::init_with_A_and_Q): how a moving harmonic notch is updated */
CVec notchAQ(const Vec& freqs, double fs, double center, double Q, double attDb);

/** one harmonic-notch set, base frequency already resolved for the operating point */
struct NotchDef {
  bool enable = true;
  double freq = 0;
  double bw = 0;
  double att = 40;
  int hmncs = 3;   // harmonics bitmask
  int opts = 0;    // bit0 double notch, bit4 triple notch
  int chained = 1;
  // configured INS_HNTCH_FREQ / BW: the firmware fixes A, Q and the composite spread from them and only moves the
  // centres with the operating point; 0 = the notch sits at its configured frequency (freq / bw)
  double refFreq = 0, refBw = 0;
};
CVec harmonicNotch(const Vec& freqs, double fs, const NotchDef& n);

/** notch parameters as logged (INS_HNTCH_MODE / FREQ / REF / FM_RAT) */
struct NotchParams {
  int mode = 0;
  double freq = 0;
  double ref = 0;
  std::optional<double> fmRat;   // INS_HNTCH_FM_RAT (default 1)
};
/** operating point used to resolve the notch frequency (0 = not measured) */
struct NotchOp {
  std::optional<double> throttle;   // 0..1
  double rpm = 0, rpm2 = 0;         // RPM sensors 1 and 2
  double escRpm = 0;                // average ESC motor rpm
  double fftHz = 0;                 // in-flight FFT noise frequency
  double backendHz = 0;             // rate the gyro filters run at (IMU.GHz)
};
/**
 * base notch frequency at an operating point, as AP_Vehicle::update_dynamic_notch sets it: FREQ (mode 0, or REF 0);
 * throttle FREQ * max(FM_RAT, sqrt(thr / REF)); RPM / ESC max(FREQ * FM_RAT, rpm / 60 * REF); FFT
 * max(FREQ * FM_RAT, fftHz); without a measurement the floor FREQ * FM_RAT
 */
double notchBaseFreq(const NotchParams& p, const NotchOp& op);

struct GyroChain {
  double backendHz = 1000;
  std::string backendSource = "INS_GYRO_RATE";   // or "IMU.GHz" (the logged gyro rate)
  double gyroFilterHz = 20;
  std::vector<NotchDef> notches;
};
CVec gyroChain(const Vec& freqs, const GyroChain& g);

/** rate-PID gains (ArduPilot units); filters in Hz, 0 = off */
struct PidGains {
  double P = 0, I = 0, D = 0, FF = 0, DFF = 0;
  double FLTT = 0, FLTE = 0, FLTD = 0;
};

struct PidResponse {
  CVec T;     // target filter
  CVec Cfb;   // error -> output (P + I + D)
  CVec Cff;   // filtered target -> output (FF + D_FF)
  CVec P, I, D;
};
PidResponse acPid(const Vec& freqs, double fs, const PidGains& g);

}  // namespace aptuner::core
