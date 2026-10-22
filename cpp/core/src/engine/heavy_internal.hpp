// Internal parts of the model-based tuner (heavytune.cpp, heavy_data.cpp, heavy_ident.cpp, heavy_design.cpp).
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "aptuner/core/cascade.hpp"
#include "aptuner/core/json.hpp"
#include "aptuner/engine/flightrec.hpp"

namespace aptuner::engine::heavy {

namespace cc = aptuner::core::cascade;
using core::Axis;
using core::Family;

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr const char* kAxisNames[3] = {"roll", "pitch", "yaw"};

/** a number as text with dec decimals */
std::string num(double v, int dec);

// ------------------------------------------------------------------------------------------------- parameters

/** the ArduPilot parameter for a key of an axis: P I D FF IMAX FLTT FLTE FLTD ANG ACC RMAX (axis); INPUT_TC GYRO LOOP
    PLT_RATE PLT_EXPO PLT_TC WPY HOVER (vehicle); empty when the family has none */
std::string paramName(Family f, int axis, const std::string& key);
/** the model gains of an axis from a parameter set (ACC in deg/s/s also when the vehicle stores centi-degrees) */
cc::Gains gainsFromParams(const log::Params& p, Family f, int axis);
/** the value to write for a key: deg/s/s ACC values converted back to centi-degrees for old firmware names */
double paramValueFor(const log::Params& p, Family f, int axis, const std::string& key, double value);

// ------------------------------------------------------------------------------------------------- hovers

struct Hover {
  std::size_t flight = 0;   // index into the flights
  int index = 0;            // 1.. within the flight
  double t0 = 0, tLift = 0, t1 = 0;   // span start, lift-off (controller output rising), end (s)
  log::Params params;       // in effect during the hover
  double meanThrottle = 0;
  double modeNum = -1;
  std::string id() const;   // "flight 1 hover 2"
};
std::vector<Hover> detectHovers(const std::vector<FlightRecord>& flights, std::vector<std::string>& notes);

/** fast samples of one axis inside [ta, tb] */
struct Window {
  Vec t, tgt, act, out;
  double fs = 0;
  std::size_t size() const { return t.size(); }
};
Window axisWindow(const AxisRec& a, double ta, double tb);

// ------------------------------------------------------------------------------------------------- spectra

/** amplitude spectrum on a grid (Hann, mean removed): amp[k] at f[k], complex value kept for phases */
struct Spectrum {
  Vec f, amp;
  std::vector<cc::Cx> X;
};
Spectrum dftGrid(const Vec& x, double fs, double fmin, double fmax, double df);
/** block-average a signal down to about targetHz (keeps the low band, limits aliasing) */
Vec decimate(const Vec& x, double fs, double targetHz, double& fsOut);
/** RMS of x after a first-order high-pass at fc (vibration above fc) */
double highPassRms(const Vec& x, double fs, double fc);

/** a low-frequency (0.2-3 Hz) oscillation of one axis over the hovers */
struct Oscillation {
  bool found = false;
  double fHz = 0, rateAmpDegs = 0, angleAmpDeg = 0;
  double prominence = 0, persistence = 0;   // peak / band median; share of windows with the same peak
  double phaseDeg = 0;                      // rate target leads actual by (deg)
  double coherence = 0;
  double iSwing = 0;                        // I-term amplitude at the peak (output units)
  bool growing = false;
  bool intermittent = false;                // in 40-60% of the windows only
  double ampEarlyDeg = 0, ampLateDeg = 0;
  int windows = 0;
  std::vector<std::size_t> hovers;          // hovers that show it
  Vec freqs;                                // peak frequency of every window in the cluster
};
Oscillation findOscillation(const std::vector<FlightRecord>& flights, const std::vector<Hover>& hovers, int axis);

// ------------------------------------------------------------------------------------------------- identification

struct YawIdent {
  bool ok = false;
  std::string reason;
  double a = 0, c = 0, tau = 0, r2 = 0;
  std::vector<double> trims;          // per hover (same order as hoverIdx)
  std::vector<std::size_t> hoverIdx;
  double aLo = 0, aHi = 0, cLo = 0, cHi = 0;
  int windows = 0;
};
YawIdent identifyYaw(const std::vector<FlightRecord>& flights, const std::vector<Hover>& hovers, Family fam);

struct RpIdent {
  bool ok = false;
  std::string reason, method;   // method: "oscillation" | "plan"
  double a = 0, tauLo = 0, tauHi = 0;
  double aLo = 0, aHi = 0;   // spread of the per-window fits
  std::vector<cc::MarginalFit> fits;
};
RpIdent identifyMarginal(const Oscillation& osc, const std::vector<Hover>& hovers, Family fam, int axis);

/** the model of an axis: a plant set (index 0 nominal, 1 the slow corner) */
struct AxisModel {
  bool ok = false;
  std::string method, reason;
  std::vector<cc::Plant> set;
  double trim = 0, trimMax = 0, umax = 0.5;
  bool umaxMeasured = false;
  Json toJson() const;
  static AxisModel fromJson(const Json& j);
};

/** the yaw model from the identification (+ mixer limit); the trim of each hover stays in YawIdent */
AxisModel yawModel(const YawIdent& id, double umax, bool umaxMeasured);
/** the roll / pitch model from a marginal-mode identification; trim from the I term, mixer limit as measured */
AxisModel rpModel(const RpIdent& id, double trim, double umax, bool umaxMeasured);

// ------------------------------------------------------------------------------------------------- validation

/** closed-loop replay of one hover from lift-off with the logged yaw stick and the flown gains */
struct YawReplay {
  std::size_t hover = 0;
  double swingLog = 0, swingModel = 0, headingRmsDeg = 0, outRms = 0;
  bool pass = false;
  Json chart = Json::object();   // {t, log, model} heading (deg), 2 Hz
};
YawReplay replayYaw(const FlightRecord& f, const Hover& h, std::size_t hoverIdx, const cc::Gains& g, const cc::Plant& p);
/** largest heading change from lift-off over the first 40 s of a hover (deg) */
double logYawSwing(const FlightRecord& f, const Hover& h);

/** roll / pitch: does the model reproduce the oscillation (frequency +-15 %, regime) with the flown gains? */
struct RpCheck {
  bool ok = false;
  double pmWorst = 0, fModelHz = 0, fLogHz = 0, simAmpLo = 0, simAmpHi = 0, logAmp = 0;
  std::string text;
  Json chart = Json::object();
};
RpCheck checkRpOscillation(const FlightRecord& f, const Hover& h, const cc::Gains& g, const AxisModel& m, const Oscillation& osc);

/** gust-driven simulation of an axis: late oscillation frequency (Hz, 0 if none) and amplitude (deg) */
struct GustResult {
  double fHz = 0, ampDeg = 0, rmsDeg = 0, limited = 0;
};
GustResult gustResponse(const cc::Gains& g, const cc::Plant& p, double seconds = 60);

// ------------------------------------------------------------------------------------------------- design

struct Candidate {
  cc::Gains g;
  cc::WorstMargins wm;
  double noiseRatio = 1;
  double score = 1e9;
  Json metrics = Json::object();
};
/** physics limits of an axis (ACC_*_MAX, RATE_*_MAX, PLT_Y_RATE, IMAX) and the reasons */
struct Limits {
  double acc = 0, rateMax = 0, pltRate = 0, imax = 0.5, inputTC = 0;
  std::vector<std::string> why;
};

struct AxisDesign {
  bool ok = false;
  std::string reason;
  cc::Gains g;
  cc::WorstMargins wm;
  double noiseRatio = 1;
  int candidates = 0, accepted = 0;
  Limits lim;
  Json metrics = Json::object(), stress = Json::object();
};

struct DesignInputs {
  Family fam = Family::Vtol;
  std::array<AxisModel, 3> models;
  std::array<cc::Gains, 3> flown;
  Vec noiseFreqs;             // measured vibration peaks (Hz); empty: 1.25 and 1.85 x the gyro filter
  bool allowFilterChange = true;
};
struct Design {
  std::array<AxisDesign, 3> axes;
  double gyroHz = 0;
  bool gyroChanged = false;
  std::vector<std::string> notes;
};
Design designAll(const DesignInputs& in);

/** what the model predicts for a set of gains (also the flown ones): yaw swing range, poles; roll / pitch margins,
    gust wander, step, oscillation expected */
Json predictYaw(const cc::Gains& g, const AxisModel& m);
Json predictRp(const cc::Gains& g, const AxisModel& m);

}  // namespace aptuner::engine::heavy
