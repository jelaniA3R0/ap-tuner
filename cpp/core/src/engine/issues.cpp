#include "aptuner/engine/issues.hpp"

#include <cmath>
#include <map>
#include <string>

#include "aptuner/core/cx.hpp"
#include "aptuner/core/jsnum.hpp"

namespace aptuner::engine {

using core::Json;
using core::jsToFixed;
using core::jsToString;

namespace {

/** JS template value of a field: number -> String(n), null -> "null", missing -> "undefined" */
std::string str(const Json& v) {
  if (v.isNumber()) return jsToString(v.num());
  if (v.isString()) return v.str();
  if (v.isNull()) return "null";
  if (v.isBool()) return v.asBool() ? "true" : "false";
  return "[object]";
}
/** fmt(v): +v.toPrecision(3) for numbers */
std::string fmt(const Json& v) {
  if (v.isNumber()) return jsToString(core::jsRoundPrecision(v.num(), 3));
  return str(v);
}
double num(const Json& v) { return v.isNumber() ? v.num() : (v.isNull() ? 0 : std::nan("")); }   // JS ToNumber
std::string pctTxt(double p) { return std::string(p > 0 ? "+" : "") + jsToString(core::jsRound(p)) + "%"; }

Json issue(const char* level, const std::string& text, const std::string& fix = "") {
  Json o = Json::obj({{"level", level}, {"text", text}});
  if (!fix.empty()) o.set("fix", fix);
  return o;
}

}  // namespace

Json axisIssues(const Json& r, core::Family family) {
  Json out = Json::array();
  if (!r.isObject() || r.has("error")) {
    out.push(issue("warn", "No data for this axis" + (r.isObject() && r["error"].isString() ? ": " + r["error"].str() : std::string())));
    return out;
  }
  const Json& d = r["diagnostics"];
  const Json& p = r["proposal"];
  const Json& idn = r["identification"];
  const Json& ms = r["measuredStep"]["metrics"];
  const Json& sat = d["saturation"];
  const std::string axis = r["axis"].strOr("");
  const bool hasSat = sat.isObject();
  if (hasSat && sat["saturated"].asBool()) {
    const double steady = num(sat["steadyErrorDegs"]);
    std::vector<std::string> parts{"asks for a steady " + jsToFixed(std::fabs(steady), 0) + " deg/s " +
                                   (axis == "yaw" ? (steady > 0 ? "nose-right " : "nose-left ") : "") +
                                   "turn that the aircraft does not make"};
    if (sat["iNearMax"].asBool())
      parts.push_back("the I term holds " + jsToFixed(std::fabs(num(sat["iMean"])), 2) + " of its " + fmt(sat["imax"]) +
                      " limit (IMAX)");
    if (sat["pinned"].asBool())
      parts.push_back("a motor sits at its upper limit " + jsToFixed(num(sat["pinnedPct"]), 1) + "% of the time (up to " +
                      str(sat["maxPwm"]) + " us)");
    const double ccw = num(sat["ccwMean"]), cw = num(sat["cwMean"]);
    if (axis == "yaw" && ccw != 0 && !std::isnan(ccw) && std::fabs(ccw - cw) > 100)
      parts.push_back("the counter-clockwise motors run at " + jsToFixed(ccw, 0) + " us against " + jsToFixed(cw, 0) +
                      " us for the clockwise ones");
    std::string joined;
    for (std::size_t i = 0; i < parts.size(); i++) joined += (i ? "; " : "") + parts[i];
    out.push(issue("bad",
                   "Saturated " + axis + ": the controller " + joined +
                       ". This is a mechanical / torque imbalance: tuning can't fix this.",
                   axis == "yaw" ? "check that all motors are vertical (inclinometer, unloaded and loaded), the props are "
                                   "the same type and pitch and on the right motors, then tilt the motors 1-2 deg to give "
                                   "yaw authority; tune yaw only after that"
                                 : "check the centre of gravity, motor mounts and props on this axis before tuning"));
  } else if (hasSat && sat["trimHeavy"].asBool()) {
    const double im = num(sat["iMean"]);
    out.push(issue("warn",
                   "The I term holds a large steady correction on " + axis + " (" +
                       jsToFixed(std::fabs(im != 0 && !std::isnan(im) ? im : 0), 2) + " of " + fmt(sat["imax"]) +
                       "): trim, centre of gravity or motor tilt",
                   "fix the cause mechanically; a larger IMAX only hides it"));
  }
  const Json& grade = d["grade"];
  const bool notGradable = grade.isObject() && grade["gradable"].isBool() && !grade["gradable"].asBool();
  if (notGradable)
    out.push(issue("warn",
                   "Not gradable: too little stick input. Gusts and trim moved the aircraft more than the pilot did, so "
                   "how well it follows the sticks cannot be judged.",
                   "fly 3-4 sharp stick taps each way on this axis"));
  if (p.isObject() && p["lowConfidence"].asBool()) {
    const Json& md = p["modelDesign"];
    const double ap = num(md["angleP"]);
    out.push(issue("warn",
                   "Lower-confidence proposal: measured from ordinary hover data (no stick taps), so it keeps a bigger "
                   "safety distance. Full model design: rate P " +
                       fmt(md["P"]) + ", I " + fmt(md["I"]) +
                       (ap != 0 && !std::isnan(ap) ? ", angle P " + fmt(md["angleP"]) : std::string()) +
                       "; one round moves at most 2x towards it.",
                   "fly the new values gently, then fly sharp stick taps and re-analyse"));
  }
  if (d["oscillating"].asBool()) {
    const double pk = num(d["peakHz"]), osc = num(d["oscRmsDegs"]);
    if (pk > 15)
      out.push(issue("bad",
                     "Noise at " + jsToFixed(pk, 0) + " Hz reaches the controller (" + jsToFixed(osc, 1) +
                         " deg/s): motor vibration rather than a tuning problem",
                     "the filter step deals with this"));
    else
      out.push(issue("bad", "Oscillating at " + jsToFixed(pk, 1) + " Hz (" + jsToFixed(osc, 1) + " deg/s)",
                     "the gains are lowered first"));
  }
  const bool judged = !notGradable && !(hasSat && sat["saturated"].asBool());
  if (judged && d["trackingPct"].isNumber()) {
    const double t = d["trackingPct"].num();
    if (t < 50)
      out.push(issue("bad", "Sluggish: follows only " + jsToFixed(t, 0) + "% of the demanded rate"));
    else if (t < 70)
      out.push(issue("warn", "Soft: follows " + jsToFixed(t, 0) + "% of the demanded rate"));
    else
      out.push(issue("ok", "Follows " + jsToFixed(t, 0) + "% of the demanded rate"));
  }
  if (judged && ms.isObject()) {
    if (ms["settled"].isBool() && !ms["settled"].asBool()) {
      const Json& st = r["measuredStep"]["t"];
      const double win = st.isArray() && st.size() ? num(st[st.size() - 1]) : 0;
      out.push(issue("warn", "The measured step response has not settled by the end of its " + jsToFixed(win, 1) +
                                 " s window (a slow axis, or one that hardly answers), so rise time and overshoot are not "
                                 "judged from it"));
    } else {
      const double os = num(ms["overshootPct"]), rise = num(ms["riseTime"]);
      if (os > 25) out.push(issue("warn", "Bouncy: overshoots " + jsToFixed(os, 0) + "% on a step"));
      if (rise > (family == core::Family::Plane ? 0.6 : 0.15))
        out.push(issue("warn", "Slow: " + jsToFixed(rise * 1000, 0) + " ms to answer a step"));
    }
  }
  if (num(d["saturationPct"]) > 2)
    out.push(issue("warn", "Motors or servos at their limit " + jsToFixed(num(d["saturationPct"]), 1) + "% of the time"));
  if (num(d["dOverP"]) > 1.5) out.push(issue("warn", "D term works harder than P (noisy gyro or too much D)"));
  if (num(d["srateMax"]) > 5)
    out.push(issue("warn", "Slew limiter active (max " + jsToFixed(num(d["srateMax"]), 1) + "): the gains are on the edge"));
  const double iMean = num(d["iMean"]);
  if (!hasSat && std::fabs(iMean != 0 && !std::isnan(iMean) ? iMean : 0) > 0.15)
    out.push(issue("warn", "Large steady I term: trim, centre of gravity or a weak motor"));
  if (!p.isObject() && !idn["ok"].asBool())
    out.push(issue("warn", "Could not measure the airframe: " +
                               (idn["reason"].isString() && !idn["reason"].str().empty() ? idn["reason"].str()
                                                                                          : std::string("too little stick input")) +
                               ". Tap the sticks harder and more often."));
  if (p.isObject() && p["changes"].isArray()) {
    std::map<std::string, const Json*> c;
    for (const auto& x : p["changes"].arr()) c[x["key"].strOr("")] = &x;
    auto lowHigh = [&](const char* k, const char* name, double thr) {
      auto it = c.find(k);
      if (it == c.end() || !(*it->second)["pct"].isNumber()) return;
      const Json& x = *it->second;
      const double pct = x["pct"].num();
      if (pct > thr)
        out.push(issue("warn", std::string(name) + " is low for this airframe: " + pctTxt(pct) + " proposed (" +
                                   fmt(x["from"]) + " to " + fmt(x["to"]) + ")"));
      else if (pct < -thr)
        out.push(issue("warn", std::string(name) + " is high for this airframe: " + pctTxt(pct) + " proposed (" +
                                   fmt(x["from"]) + " to " + fmt(x["to"]) + ")"));
    };
    lowHigh("P", "Rate P", 15);
    lowHigh("D", "Rate D", 20);
    lowHigh("FF", "Feed-forward", 15);
    lowHigh("angleP", "Angle P", 15);
    lowHigh("tconst", "Time constant", 15);
    for (const char* k : {"FLTT", "FLTD", "FLTE"}) {
      auto it = c.find(k);
      if (it != c.end())
        out.push(issue("warn", std::string(k) + " filter " + fmt((*it->second)["from"]) + " to " + fmt((*it->second)["to"]) +
                                   " Hz (ArduPilot filter rule)"));
    }
    if (p["changes"].arr().empty()) out.push(issue("ok", "The gains already match what the measurement suggests"));
  }
  if (out.arr().empty()) out.push(issue("ok", "Nothing wrong found"));
  return out;
}

Json vehicleIssues(const Json& checks) {
  Json out = Json::array();
  if (!checks.isArray()) return out;
  for (const auto& c : checks.arr())
    if (!c["ok"].asBool()) {
      Json o = Json::obj({{"level", "warn"}, {"text", c["title"]}, {"fix", c["detail"]}});
      out.push(o);
    }
  return out;
}

}  // namespace aptuner::engine
