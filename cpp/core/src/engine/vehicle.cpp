#include "aptuner/engine/vehicle.hpp"

#include <algorithm>
#include <cmath>

namespace aptuner::engine {

std::optional<Family> detectFamily(int t, const std::string& modeName, const log::Params& params) {
  if (t == 4) return Family::Heli;
  if (t == 2 || t == 13 || t == 14 || t == 15 || t == 29 || t == 3) return Family::Copter;
  if (t == 1 || (t >= 19 && t <= 25)) {
    const double q = params.getOr("Q_ENABLE", std::nan(""));
    if (q > 0 && isVtolMode(modeName)) return Family::Vtol;
    return Family::Plane;
  }
  return std::nullopt;
}

bool isVtolMode(const std::string& m) {
  // /^Q|^QAUTOTUNE|^LOITER_ALT_QLAND/
  return (!m.empty() && m[0] == 'Q') || m.rfind("LOITER_ALT_QLAND", 0) == 0;
}

Json axisGains(Family f, Axis axis, const log::Params& params) {
  const core::FamilyDef& d = core::familyDef(f);
  const std::string pre = d.axes[static_cast<int>(axis)];
  Json g = Json::object();
  for (const char* s : core::kSuffixes) {
    auto v = params.get(pre + s);
    const std::string k = s;
    const bool zeroDefault = k == "FLTT" || k == "FLTE" || k == "FLTD" || k == "FF" || k == "D_FF" || k == "SMAX";
    if (v)
      g.set(k, *v);
    else if (zeroDefault)
      g.set(k, 0.0);
    else
      g.set(k, Json());   // undefined: keeps the JavaScript key order, dropped when written
  }
  g.set("DFF", g["D_FF"]);
  // drop undefined members (JSON.stringify leaves them out)
  Json out = Json::object();
  for (const auto& [k, v] : g.items())
    if (!v.isNull()) out.set(k, v);
  return out;
}

Json axisParamNames(Family f, Axis axis) {
  const core::FamilyDef& d = core::familyDef(f);
  const int a = static_cast<int>(axis);
  const std::string pre = d.axes[a];
  Json n = Json::object();
  for (const char* k : {"P", "I", "D", "FF", "FLTT", "FLTE", "FLTD", "SMAX", "IMAX"}) n.set(k, pre + k);
  if (d.angleP[a]) n.set("angleP", d.angleP[a]);
  if (d.accMax[a]) n.set("accMax", d.accMax[a]);
  if (d.tconst[a]) n.set("tconst", d.tconst[a]);
  return n;
}

double planeSpeedScaler(const log::Params& params, double airspeed, double throttlePct) {
  const double ss = params.truthyOr("SCALING_SPEED", 15);
  if (airspeed > 0.1) {
    double s = ss / airspeed;
    const double smin = params.truthyOr("AIRSPEED_MIN", 9) * 0.5, smax = params.truthyOr("AIRSPEED_MAX", 22) * 2;
    s = std::min(std::max(s, ss / smax), ss / smin);
    return std::min(std::max(s, 0.5), 2.0);
  }
  // no airspeed: throttle-based estimate
  const double thr = (throttlePct != 0 && !std::isnan(throttlePct)) ? throttlePct : 50;
  const double s = 1 - (thr - 50) / 100 * 0.5;
  return std::min(std::max(s, 0.6), 1.67);
}

}  // namespace aptuner::engine
