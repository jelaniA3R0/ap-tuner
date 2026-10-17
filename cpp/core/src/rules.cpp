#include "aptuner/core/rules.hpp"

#include <algorithm>
#include <cmath>

#include "aptuner/core/cx.hpp"
#include "aptuner/core/jsnum.hpp"

namespace aptuner::core {

const char* familyName(Family f) {
  switch (f) {
    case Family::Copter: return "copter";
    case Family::Vtol: return "vtol";
    case Family::Plane: return "plane";
    case Family::Heli: return "heli";
  }
  return "copter";
}

std::optional<Family> parseFamily(const std::string& s) {
  if (s == "copter") return Family::Copter;
  if (s == "vtol") return Family::Vtol;
  if (s == "plane") return Family::Plane;
  if (s == "heli") return Family::Heli;
  return std::nullopt;
}

const char* axisName(Axis a) {
  switch (a) {
    case Axis::Roll: return "roll";
    case Axis::Pitch: return "pitch";
    case Axis::Yaw: return "yaw";
  }
  return "roll";
}

std::optional<Axis> parseAxis(const std::string& s) {
  if (s == "roll") return Axis::Roll;
  if (s == "pitch") return Axis::Pitch;
  if (s == "yaw") return Axis::Yaw;
  return std::nullopt;
}

namespace {

const FamilyDef kCopter{
    {"ATC_RAT_RLL_", "ATC_RAT_PIT_", "ATC_RAT_YAW_"},
    {"ATC_ANG_RLL_P", "ATC_ANG_PIT_P", "ATC_ANG_YAW_P"},
    {"ATC_ACC_R_MAX", "ATC_ACC_P_MAX", "ATC_ACC_Y_MAX"},
    {nullptr, nullptr, nullptr},
    "SCHED_LOOP_RATE",
    "INS_GYRO_FILTER",
    "MOT_THST_HOVER",
    nullptr,
    {"PIDR", "PIDP", "PIDY"},
    true,
    {{{"RDes", "R", "ROut"}, {"PDes", "P", "POut"}, {"YDes", "Y", "YOut"}}},
    50,
    1,
};

const FamilyDef kVtol{
    {"Q_A_RAT_RLL_", "Q_A_RAT_PIT_", "Q_A_RAT_YAW_"},
    {"Q_A_ANG_RLL_P", "Q_A_ANG_PIT_P", "Q_A_ANG_YAW_P"},
    {"Q_A_ACC_R_MAX", "Q_A_ACC_P_MAX", "Q_A_ACC_Y_MAX"},
    {nullptr, nullptr, nullptr},
    "SCHED_LOOP_RATE",
    "INS_GYRO_FILTER",
    "Q_M_THST_HOVER",
    nullptr,
    {"PIQR", "PIQP", "PIQY"},
    true,
    {{{"RDes", "R", "ROut"}, {"PDes", "P", "POut"}, {"YDes", "Y", "YOut"}}},
    50,
    1,
};

const FamilyDef kPlane{
    {"RLL_RATE_", "PTCH_RATE_", "YAW_RATE_"},
    {nullptr, nullptr, nullptr},
    {nullptr, nullptr, nullptr},
    {"RLL2SRV_TCONST", "PTCH2SRV_TCONST", nullptr},
    "SCHED_LOOP_RATE",
    "INS_GYRO_FILTER",
    nullptr,
    "SCALING_SPEED",
    {"PIDR", "PIDP", "PIDY"},
    false,
    {{{nullptr, nullptr, nullptr}, {nullptr, nullptr, nullptr}, {nullptr, nullptr, nullptr}}},
    150,
    45,
};

}  // namespace

const FamilyDef& familyDef(Family f) {
  switch (f) {
    case Family::Vtol: return kVtol;
    case Family::Plane: return kPlane;
    case Family::Copter:
    case Family::Heli: return kCopter;
  }
  return kCopter;
}

const char* const kSuffixes[10] = {"P", "I", "D", "FF", "FLTT", "FLTE", "FLTD", "SMAX", "IMAX", "D_FF"};

std::vector<std::string> neededParams(Family family) {
  const FamilyDef& f = familyDef(family);
  std::vector<std::string> names{f.loopRate, f.gyroFilter, "GCS_PID_MASK", "INS_GYRO_RATE",
                                 "INS_HNTCH_ENABLE", "INS_HNTCH_MODE", "INS_HNTCH_FREQ", "INS_HNTCH_BW",
                                 "INS_HNTCH_ATT", "INS_HNTCH_REF", "INS_HNTCH_HMNCS", "INS_HNTCH_OPTS",
                                 "INS_HNTCH_FM_RAT", "INS_HNTC2_ENABLE", "INS_HNTC2_MODE", "INS_HNTC2_FREQ",
                                 "INS_HNTC2_BW", "INS_HNTC2_ATT", "INS_HNTC2_REF", "INS_HNTC2_HMNCS",
                                 "INS_HNTC2_OPTS", "INS_HNTC2_FM_RAT"};
  for (int ax = 0; ax < 3; ax++)
    for (const char* s : kSuffixes) names.push_back(std::string(f.axes[ax]) + s);
  for (int ax = 0; ax < 3; ax++)
    if (f.angleP[ax]) names.push_back(f.angleP[ax]);
  for (int ax = 0; ax < 3; ax++)
    if (f.accMax[ax]) names.push_back(f.accMax[ax]);
  for (int ax = 0; ax < 3; ax++)
    if (f.tconst[ax]) names.push_back(f.tconst[ax]);
  if (family == Family::Plane) {
    names.push_back("RLL2SRV_RMAX");
    names.push_back("PTCH2SRV_RMAX_UP");
    names.push_back("PTCH2SRV_RMAX_DN");
  }
  if (f.hoverThr) names.push_back(f.hoverThr);
  if (f.scalingSpeed) {
    for (const char* n : {f.scalingSpeed, "AIRSPEED_MIN", "AIRSPEED_MAX", "ARSPD_USE", "RLL_RATE_FF", "PTCH_RATE_FF"})
      names.push_back(n);
  }
  if (family == Family::Copter || family == Family::Heli)
    for (const char* n : {"ATC_RATE_FF_ENAB", "ATC_INPUT_TC", "FRAME_CLASS", "FRAME_TYPE", "MOT_THST_EXPO",
                          "MOT_SPIN_MIN", "MOT_PWM_MIN", "MOT_PWM_MAX", "MOT_BAT_VOLT_MAX", "MOT_BAT_VOLT_MIN",
                          "ATC_THR_MIX_MAN", "PILOT_Y_RATE_TC", "FFT_ENABLE", "LOG_BITMASK", "INS_LOG_BAT_MASK",
                          "INS_RAW_LOG_OPT"})
      names.push_back(n);
  if (family == Family::Vtol)
    for (const char* n : {"Q_ENABLE", "Q_FRAME_CLASS", "Q_FRAME_TYPE", "Q_M_THST_EXPO", "Q_M_SPIN_MIN",
                          "Q_A_RATE_FF_ENAB", "Q_TAILSIT_ENABLE", "Q_TILT_ENABLE", "LOG_BITMASK", "INS_LOG_BAT_MASK",
                          "INS_RAW_LOG_OPT"})
      names.push_back(n);
  // de-duplicate, keeping the first occurrence (JS: Array.from(new Set(names)))
  std::vector<std::string> out;
  for (auto& n : names)
    if (std::find(out.begin(), out.end(), n) == out.end()) out.push_back(n);
  return out;
}

// Kept fraction of the model's oscillation gain (QuikTune keeps 40%), validated by oscillation sweeps on
// ArduPilot's simulated quad and quadplane (docs/how-it-works.md). The margin limits are a safety net only.
const Preset kPresets[3] = {
    {"conservative", 6, 30, 3.0, 12, 0.3, "Conservative (large / flexible / payload aircraft: 30% of the oscillation gain)"},
    {"normal", 4, 25, 3.5, 20, 0.4, "Normal (QuikTune rule: 40% of the oscillation gain)"},
    {"aggressive", 3, 20, 4.5, 30, 0.5, "Aggressive (small agile aircraft: 50% of the oscillation gain)"},
};

const Preset& preset(const std::string& name) {
  for (const auto& p : kPresets)
    if (name == p.name) return p;
  return kPresets[1];
}

Filters filterRules(Family f, Axis axis, const AxisGainsIn& cur, double gyroHz, double tau) {
  if (f == Family::Plane) {
    const double t = (tau != 0 && !std::isnan(tau)) ? tau : 0.5;   // tau || 0.5
    return {jsRoundFixed(10 / (2 * kPi * std::max(t, 0.1)), 2), 0, jsRoundFixed(gyroHz * 0.5, 1)};
  }
  // keep the vehicle's own target / D filters when they are sane (0 < f <= INS_GYRO_FILTER)
  auto sane = [&](double v) { return v > 0 && v <= gyroHz; };
  Filters r;
  r.FLTT = sane(cur.FLTT) ? cur.FLTT : jsRoundFixed(gyroHz * 0.5, 1);
  r.FLTD = sane(cur.FLTD) ? cur.FLTD : jsRoundFixed(gyroHz * 0.5, 1);
  r.FLTE = 0;
  if (axis == Axis::Yaw)
    r.FLTE = (cur.FLTE > 0 && cur.FLTE <= 8) ? cur.FLTE : 2.5;
  else if (cur.FLTE > 0 && cur.FLTE <= gyroHz)
    r.FLTE = cur.FLTE;
  return r;
}

double iRule(Family f, Axis axis, double P, double FF, bool hasFF) {
  if (f == Family::Plane) {
    // AP_AutoTune: roll I = min(P, FF/1s); pitch/yaw I = max(0.75 P, FF)
    return axis == Axis::Roll ? std::min(P, FF) : std::max(0.75 * P, FF);
  }
  if (hasFF && FF > 0) return FF;   // heli / vectored axes: I = FF
  return axis == Axis::Yaw ? 0.1 * P : P;   // QuikTune: RP_PI_RATIO 1, Y_PI_RATIO 10
}

GainLimits gainLimits(Family f, Axis axis) {
  if (f == Family::Plane) return {{0.01, 3.0}, {0.0005, 0.3}, {0.01, 3.0}};
  if (axis == Axis::Yaw) return {{0.02, 0.5}, {0, 0.01}, {0, 1}};   // QUIK_YAW_P_MAX 0.5, QUIK_YAW_D_MAX 0.01
  return {{0.01, 2.0}, {0.0005, 0.2}, {0, 1}};                       // AUTOTUNE_RP_MIN/MAX, RD_MAX
}

AngleRule angleRule(Family f, Axis axis, double wcRadS, double yawMin) {
  AngleRule r;
  if (f == Family::Plane) {
    // TCONST = 1 / angle gain; keep >= 3x the rate-loop time constant, within [0.15, 1.0]
    const double tau = std::min(1.0, std::max(0.15, 3 / std::max(wcRadS, 1e-3)));
    r.tconst = jsRoundFixed(tau, 2);
    return r;
  }
  const double p =
      std::min(axis == Axis::Yaw ? 8.0 : 12.0, std::max(axis == Axis::Yaw ? yawMin : 3.0, wcRadS / 4));
  r.angleP = jsRoundFixed(p, 2);
  return r;
}

double oscSmaxFor(double massKg) {
  if (!(massKg != 0) || std::isnan(massKg)) return 5;
  if (massKg >= 100) return 1.5;
  if (massKg >= 50) return 3;
  if (massKg >= 15) return 4;
  return 5;
}

}  // namespace aptuner::core
