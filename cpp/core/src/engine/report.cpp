#include "aptuner/engine/report.hpp"

#include "aptuner/core/jsnum.hpp"
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

}  // namespace

Json proposalRows(core::Family family, const Json& axes, const Json& noise) {
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
                              {"pct", c["pct"]}});
        rows.push(row);
      }
    }
  if (noise.isObject() && noise["recommended"].isObject()) {
    for (const auto& [k, v] : noise["recommended"]["params"].items()) {
      std::string label = k;
      const std::string pre = "INS_HNTCH_";
      if (label.rfind(pre, 0) == 0) label = "notch " + label.substr(pre.size());
      rows.push(Json::obj({{"group", "notch"}, {"name", k}, {"value", v}, {"label", label}}));
    }
    rows.push(Json::obj({{"group", "notch"},
                         {"name", "INS_GYRO_FILTER"},
                         {"value", noise["recommended"]["gyroFilterHz"]},
                         {"label", "gyro low-pass (Hz)"}}));
  }
  return rows;
}

std::string paramFile(const Json& rows, const std::vector<std::string>& header) {
  std::vector<std::string> lines{
      "# AP Tuner parameter file. Load it in Mission Planner: CONFIG > Full Parameter List > Load from file, then Write Params."};
  for (const auto& h : header) lines.push_back("# " + h);
  std::string group;
  bool first = true;
  for (const auto& r : rows.arr()) {
    const std::string g = r["group"].strOr("");
    if (first || g != group) {
      group = g;
      first = false;
      lines.push_back("# --- " + group + " ---");
    }
    // Mission Planner reads NAME,VALUE lines; keep the previous value on a comment line of its own
    if (r.has("from") && !r["from"].isNull())
      lines.push_back("# " + r["name"].strOr("") + " was " + core::jsToString(core::jsRoundPrecision(r["from"].numOr(0), 6)));
    const Json& v = r["value"];
    lines.push_back(r["name"].strOr("") + "," + (v.isNumber() ? core::jsToString(v.num()) : v.strOr("")));
  }
  std::string out;
  for (std::size_t i = 0; i < lines.size(); i++) {
    if (i) out += "\r\n";
    out += lines[i];
  }
  return out + "\r\n";
}

}  // namespace aptuner::engine
