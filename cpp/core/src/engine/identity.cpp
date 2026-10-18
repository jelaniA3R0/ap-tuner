#include "aptuner/engine/identity.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "aptuner/core/jsnum.hpp"

namespace aptuner::engine {

using core::Json;

namespace {
double nowMs() {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}
bool isFiniteNumber(const Json& v) { return v.isNumber() && std::isfinite(v.num()); }
int seq = 0;
}  // namespace

std::string jsText(const Json& v) {
  if (v.isNumber()) return core::jsToString(v.num());
  if (v.isString()) return v.str();
  if (v.isNull()) return "null";
  if (v.isBool()) return v.asBool() ? "true" : "false";
  return v.dump();
}

std::string f32hex(double v) {
  const auto bits = std::bit_cast<std::uint32_t>(static_cast<float>(v));
  char buf[9];
  std::snprintf(buf, sizeof buf, "%08x", static_cast<unsigned>(bits));
  return buf;
}

std::string fnv1a64(const std::string& text) {
  std::uint64_t h = 0xcbf29ce484222325ULL;
  for (const char c : text) {
    h ^= static_cast<std::uint8_t>(c);
    h *= 0x100000001b3ULL;
  }
  char buf[17];
  std::snprintf(buf, sizeof buf, "%016llx", static_cast<unsigned long long>(h));
  return buf;
}

bool sameF32(double a, double b) { return std::isfinite(a) && std::isfinite(b) && static_cast<float>(a) == static_cast<float>(b); }
bool sameF32(const Json& a, const Json& b) { return a.isNumber() && b.isNumber() && sameF32(a.num(), b.num()); }

std::string paramHash(const Json& params) {
  std::vector<std::string> names;
  if (params.isObject())
    for (const auto& [k, v] : params.items())
      if (isFiniteNumber(v)) names.push_back(k);
  std::sort(names.begin(), names.end());
  std::string canon;
  for (std::size_t i = 0; i < names.size(); i++) canon += (i ? "\n" : "") + names[i] + "=" + f32hex(params[names[i]].num());
  return fnv1a64(canon);
}

Json sessionIdentity(const Json& linkIdentity, const Json& params) {
  Json id = linkIdentity;
  id.set("key", "g" + jsText(linkIdentity["gen"]) + ":" + jsText(linkIdentity["sysid"]) + "/" + jsText(linkIdentity["compid"]));
  id.set("paramHash", params.isObject() ? Json(paramHash(params)) : Json());
  return id;
}

std::string identityText(const Json& id) {
  if (!id.isObject() || id["sysid"].isNull()) return "no vehicle";
  std::string type;
  if (id["typeName"].isString() && !id["typeName"].str().empty()) {
    type = id["typeName"].str();
    if (type.rfind("MAV_TYPE_", 0) == 0) type = type.substr(9);
    std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  } else {
    type = "type " + jsText(id["type"]);
  }
  std::string uid;
  if (id["uid"].isString() && !id["uid"].str().empty()) {
    uid = id["uid"].str();
    std::size_t zeros = 0;
    while (zeros < uid.size() && uid[uid.size() - 1 - zeros] == '0') zeros++;
    uid = uid.substr(0, uid.size() - zeros / 2 * 2).substr(0, 24);   // like .replace(/(00)+$/, '').slice(0, 24)
  }
  const std::string fw = id["fw"].isString() && !id["fw"].str().empty() ? id["fw"].str() : "firmware not reported";
  const std::string git = id["gitHash"].isString() && !id["gitHash"].str().empty() ? " (" + id["gitHash"].str() + ")" : "";
  return "system id " + jsText(id["sysid"]) + " (" + type + "), ArduPilot " + fw + git + ", board " + (id["uid"].isString() && !id["uid"].str().empty() ? "UID " + uid : "UID not reported") +
         ", connection #" + jsText(id["gen"]);
}

bool sameVehicle(const Json& a, const Json& b) {
  if (!a.isObject() || !b.isObject()) return false;
  auto eqNum = [&](const char* k) { return a[k].isNumber() && b[k].isNumber() ? a[k].num() == b[k].num() : (a[k].isNull() && b[k].isNull()); };
  auto known = [](const Json& v) { return v.isString() && !v.str().empty(); };
  if (!eqNum("sysid") || !eqNum("compid") || !eqNum("type")) return false;
  if (known(a["uid"]) && known(b["uid"]) && a["uid"].str() != b["uid"].str()) return false;
  if (known(a["fw"]) && known(b["fw"]) && a["fw"].str() != b["fw"].str()) return false;
  return true;
}

Json makeProposal(const std::string& source, const std::string& axis, const Json& identity, const Json& rows) {
  const std::string id = std::string(source == "log" ? "G" : "L") + std::to_string(++seq) + (axis.empty() ? "" : "-" + axis);
  auto hx = [](const Json& v) { return isFiniteNumber(v) ? f32hex(v.num()) : std::string("-"); };
  const bool has = identity.isObject();
  auto orDash = [](const Json& v) { return v.isString() && !v.str().empty() ? v.str() : std::string("-"); };
  std::string canon = "APT-PROPOSAL 1\nidentity " + (has ? jsText(identity["key"]) : "-") + " uid " + (has ? orDash(identity["uid"]) : "-") + " fw " +
                      (has ? orDash(identity["fw"]) : "-");
  for (const auto& r : rows.arr()) canon += "\n" + r["name"].strOr("") + "=" + hx(r["to"]) + " from " + hx(r["from"]);
  return Json::obj({{"id", id}, {"hash", fnv1a64(canon)}, {"source", source}, {"axis", axis.empty() ? Json() : Json(axis)},
                    {"identityKey", has ? identity["key"] : Json()}, {"gen", has ? identity["gen"] : Json()}, {"sysid", has ? identity["sysid"] : Json()},
                    {"compid", has ? identity["compid"] : Json()}, {"at", nowMs()}, {"rows", rows}});
}

}  // namespace aptuner::engine
