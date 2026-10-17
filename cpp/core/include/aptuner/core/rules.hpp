// ArduPilot tuning rules and parameter naming per vehicle family. Port of src/model/rules.js: QuikTune
// (VTOL-quicktune.lua), AC_AutoTune_Multi, AP_AutoTune (plane) and the tuning-process wiki pages.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace aptuner::core {

enum class Family { Copter, Vtol, Plane, Heli };
enum class Axis { Roll = 0, Pitch = 1, Yaw = 2 };

const char* familyName(Family f);                 // "copter" | "vtol" | "plane" | "heli"
std::optional<Family> parseFamily(const std::string& s);
const char* axisName(Axis a);                     // "roll" | "pitch" | "yaw"
std::optional<Axis> parseAxis(const std::string& s);
inline constexpr std::array<Axis, 3> kAxes{Axis::Roll, Axis::Pitch, Axis::Yaw};

/** rate-loop parameter naming and limits of one family (heli uses the copter definition) */
struct FamilyDef {
  std::array<const char*, 3> axes;        // rate-PID prefix per axis, e.g. "Q_A_RAT_RLL_"
  std::array<const char*, 3> angleP;      // angle P per axis (nullptr: none, plane)
  std::array<const char*, 3> accMax;      // accel limit per axis (nullptr: none)
  std::array<const char*, 3> tconst;      // plane time constants (nullptr where absent)
  const char* loopRate;
  const char* gyroFilter;
  const char* hoverThr;                   // nullptr for plane
  const char* scalingSpeed;               // plane only
  std::array<const char*, 3> logPid;      // DataFlash PID message per axis, e.g. "PIQR"
  bool hasLogRate;                        // RATE message usable (copter / vtol)
  std::array<std::array<const char*, 3>, 3> logRate;   // {des, actual, out} RATE fields per axis
  double defaultSmax;
  double uMax;                            // actuator range: mixer 1, plane 45 deg
};
const FamilyDef& familyDef(Family f);

extern const char* const kSuffixes[10];   // P I D FF FLTT FLTE FLTD SMAX IMAX D_FF

/** all parameter names the tuner reads for a family */
std::vector<std::string> neededParams(Family f);

struct Preset {
  const char* name;
  double gmMinDb, pmMinDeg, msMax, osMaxPct, backoff;
  const char* label;
};
const Preset& preset(const std::string& name);   // unknown -> "normal"
extern const Preset kPresets[3];                  // conservative, normal, aggressive

/** current axis gains as far as the rules need them */
struct AxisGainsIn {
  double FLTT = 0, FLTE = 0, FLTD = 0;
};
struct Filters {
  double FLTT = 0, FLTE = 0, FLTD = 0;
};
Filters filterRules(Family f, Axis axis, const AxisGainsIn& cur, double gyroHz, double tau);

double iRule(Family f, Axis axis, double P, double FF, bool hasFF);

struct GainLimits {
  double P[2], D[2], FF[2];
};
GainLimits gainLimits(Family f, Axis axis);

/** angle P (copter / vtol) or TCONST (plane) from the rate-loop crossover in rad/s */
struct AngleRule {
  std::optional<double> angleP, tconst;
};
AngleRule angleRule(Family f, Axis axis, double wcRadS, double yawMin = 2);

double oscSmaxFor(double massKg);

}  // namespace aptuner::core
