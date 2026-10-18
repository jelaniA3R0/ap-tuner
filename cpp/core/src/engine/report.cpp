#include "aptuner/engine/report.hpp"

#include <algorithm>
#include <cmath>
#include <regex>

#include "aptuner/core/jsnum.hpp"
#include "aptuner/core/parammeta.hpp"
#include "aptuner/engine/identity.hpp"
#include "aptuner/engine/vehicle.hpp"

namespace aptuner::engine {

using core::Json;

namespace {

const char* prettyOf(const std::string& k) {
  if (k == "P") return "rate P";
  if (k == "I") return "rate I";
  if (k == "D") return "rate D";
  if (k == "FF") return "feed-forward";
  if (k == "FLTT") return "target filter (Hz)";
  if (k == "FLTE") return "error filter (Hz)";
  if (k == "FLTD") return "D filter (Hz)";
  if (k == "SMAX") return "slew limit";
  if (k == "IMAX") return "I limit";
  if (k == "angleP") return "angle P";
  if (k == "tconst") return "time constant";
  return nullptr;
}
std::string whyOf(const std::string& k) {
  if (k == "P") return "from the identified rate-loop model (preset margins)";
  if (k == "I") return "follows P / FF (ArduPilot rule)";
  if (k == "D") return "from the identified rate-loop model";
  if (k == "FF") return "from the stick events (feed-forward)";
  if (k == "FLTT" || k == "FLTE" || k == "FLTD") return "filter rule";
  if (k == "SMAX") return "slew-rate limit (needed for the oscillation signal)";
  if (k == "IMAX") return "the I term holds a steady correction near its limit";
  if (k == "angleP") return "from the rate-loop crossover (at most x1.5 per round)";
  if (k == "tconst") return "from the rate-loop crossover";
  return "";
}
std::string join(const std::vector<std::string>& v, const std::string& sep) {
  std::string out;
  for (std::size_t i = 0; i < v.size(); i++) out += (i ? sep : "") + v[i];
  return out;
}
bool finiteNum(const Json& v) { return v.isNumber() && std::isfinite(v.num()); }

}  // namespace

Json proposalRows(core::Family family, const Json& axes, const Json& noise, const Json& params) {
  Json rows = Json::array();
  if (axes.isObject())
    for (const auto& [axisN, a] : axes.items()) {
      if (!a.isObject() || !a["proposal"].isObject()) continue;
      const auto axis = core::parseAxis(axisN);
      if (!axis) continue;
      const Json names = axisParamNames(family, *axis);
      for (const auto& c : a["proposal"]["changes"].arr()) {
        const std::string key = c["key"].strOr("");
        const Json& pn = names[key];
        if (!pn.isString()) continue;
        const char* pretty = prettyOf(key);
        Json row = Json::obj({{"group", axisN},
                              {"name", pn},
                              {"value", core::jsRoundPrecision(c["to"].num(), 6)},
                              {"from", c["from"]},
                              {"label", pretty ? Json(pretty) : Json(key)},
                              {"pct", c["pct"]},
                              {"reason", whyOf(key)}});
        rows.push(row);
      }
    }
  if (noise.isObject() && noise["recommended"].isObject()) {
    const Json& rec = noise["recommended"];
    const std::string why = rec["why"].isString() && !rec["why"].str().empty() ? rec["why"].str() : "notch from the gyro noise in the log";
    for (const auto& [k, v] : rec["params"].items()) {
      std::string label = k;
      const std::string pre = "INS_HNTCH_";
      if (label.rfind(pre, 0) == 0) label = "notch " + label.substr(pre.size());
      rows.push(Json::obj({{"group", "notch"}, {"name", k}, {"value", v}, {"from", params.isObject() ? params[k] : Json()}, {"label", label}, {"reason", why}}));
    }
    rows.push(Json::obj({{"group", "notch"},
                         {"name", "INS_GYRO_FILTER"},
                         {"value", rec["gyroFilterHz"]},
                         {"from", params.isObject() ? params["INS_GYRO_FILTER"] : Json()},
                         {"label", "gyro low-pass (Hz)"},
                         {"reason", "gyro low-pass that the notch allows"}}));
  }
  return rows;
}

ParamFileOut paramFileChecked(const Json& rows, const std::vector<std::string>& header, const Json& identity, bool strict) {
  ParamFileOut out;
  std::vector<std::string> lines{"# AP Tuner parameter file. Load it in Mission Planner: CONFIG > Full Parameter List > Load from file, then Write Params."};
  if (identity.isString())
    lines.push_back("# For: " + identity.str() + ". Load it only on this vehicle.");
  else if (identity.isObject())
    lines.push_back("# For: " + identityText(identity) + ". Load it only on this vehicle.");
  for (const auto& h : header) lines.push_back("# " + h);
  std::string group;
  bool first = true;
  for (const auto& r : rows.arr()) {
    const std::string g = r["group"].isString() ? r["group"].str() : "undefined";
    if (first || g != group) {
      group = g;
      first = false;
      lines.push_back("# --- " + group + " ---");
    }
    const std::string name = r["name"].strOr("");
    const Json& v = r["value"];
    const core::ParamCheck c = core::validateParam(name, v);
    if (!c.ok && strict) {
      out.errors.push(Json::obj({{"name", name}, {"value", v}, {"why", join(c.errors, ", ")}}));
      lines.push_back("# NOT IN THIS FILE: " + name + " = " + jsText(v) + " " + join(c.errors, ", "));
      continue;
    }
    for (const auto& w : c.ok ? c.warnings : c.errors) {
      out.warnings.push(Json::obj({{"name", name}, {"why", w}}));
      lines.push_back("# NOTE: " + name + " " + w);
    }
    if (c.reboot) lines.push_back("# " + name + " takes effect only after a reboot of the autopilot");
    // Mission Planner reads NAME,VALUE lines; keep the previous value on a comment line of its own
    if (finiteNum(r["from"])) lines.push_back("# " + name + " was " + core::formatParamValue(name, r["from"].num()));
    lines.push_back(name + "," + core::formatParamValue(name, v.isNumber() ? v.num() : NAN));
  }
  for (std::size_t i = 0; i < lines.size(); i++) out.text += (i ? "\r\n" : "") + lines[i];
  out.text += "\r\n";
  return out;
}

std::string paramFile(const Json& rows, const std::vector<std::string>& header, const Json& identity, bool strict) {
  return paramFileChecked(rows, header, identity, strict).text;
}

Json logIdentity(const Json& result) {
  const Json& p = result["params"];
  const std::string fwText = result["firmware"].isString() ? result["firmware"].str() : "";
  std::smatch m, h;
  static const std::regex verRe("V(\\d+)\\.(\\d+)\\.(\\d+)");
  static const std::regex hashRe("\\(([0-9a-f]{6,})\\)", std::regex::ECMAScript | std::regex::icase);
  const bool hasVer = std::regex_search(fwText, m, verRe);
  const bool hasHash = std::regex_search(fwText, h, hashRe);
  std::string hash = hasHash ? h[1].str() : "";
  std::transform(hash.begin(), hash.end(), hash.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return Json::obj({{"sysid", p["SYSID_THISMAV"].isNumber() ? p["SYSID_THISMAV"] : Json()},
                    {"fw", hasVer ? Json(m[1].str() + "." + m[2].str() + "." + m[3].str()) : Json()},
                    {"gitHash", hasHash ? Json(hash) : Json()},
                    {"firmwareText", fwText.empty() ? Json() : Json(fwText)}});
}

Json logVehicleCheck(const Json& result, const Json& identity) {
  const Json L = logIdentity(result);
  Json problems = Json::array(), notes = Json::array();
  const bool hasId = identity.isObject();
  if (L["sysid"].isNull())
    notes.push("the log has no SYSID_THISMAV: the system id could not be compared");
  else if (hasId && identity["sysid"].isNumber() && L["sysid"].num() != identity["sysid"].num())
    problems.push("the log is from system id " + jsText(L["sysid"]) + ", the autopilot on this connection is system id " + jsText(identity["sysid"]));
  std::string fw;
  if (hasId && identity["fw"].isString() && !identity["fw"].str().empty()) {
    fw = identity["fw"].str();
    const auto dash = fw.find('-');
    if (dash != std::string::npos) fw = fw.substr(0, dash);
  }
  const std::string lfw = L["fw"].isString() ? L["fw"].str() : "";
  if (lfw.empty() || fw.empty()) {
    notes.push(std::string("the firmware version could not be compared (") + (lfw.empty() ? "not in the log" : "the autopilot did not report it") + ")");
  } else if (lfw != fw) {
    problems.push("the log was recorded with ArduPilot " + lfw + ", the autopilot now runs " + identity["fw"].str() + ": fly again with this firmware");
  } else if (L["gitHash"].isString() && identity["gitHash"].isString() && !identity["gitHash"].str().empty()) {
    const std::string lh = L["gitHash"].str();
    std::string ih = identity["gitHash"].str();
    std::transform(ih.begin(), ih.end(), ih.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (lh.rfind(ih, 0) != 0 && ih.rfind(lh, 0) != 0)
      problems.push("the log was recorded with another build of ArduPilot " + lfw + " (" + lh + ", now " + identity["gitHash"].str() + "): fly again with this firmware");
  }
  return Json::obj({{"log", L}, {"problems", problems}, {"notes", notes}});
}

}  // namespace aptuner::engine
