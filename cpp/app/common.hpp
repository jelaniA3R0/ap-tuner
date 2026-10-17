// Small helpers shared by the app modules.
#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "aptuner/core/json.hpp"
#include "aptuner/core/rules.hpp"
#include "aptuner/log/dfreader.hpp"

namespace aptuner::app {

using core::Json;

/** numeric members of a JSON parameter object as log::Params (null / missing values left out) */
log::Params toParams(const Json& p);

/** fmt(v) of the JavaScript version: +v.toPrecision(4) for numbers, "undefined" / "null" otherwise */
std::string fmt4(const Json& v);

/** new Date().toISOString() and the file-name stamp the JavaScript version used (":" and "." -> "-") */
std::string isoNow();
std::string isoStampForFile();
/** new Date().toLocaleString() / toLocaleTimeString() approximations */
std::string localNow();
std::string localTime(double epochMs);

/** "copter" / "vtol" / "plane" / "heli" or null */
inline Json familyJson(const std::optional<core::Family>& f) { return f ? Json(core::familyName(*f)) : Json(); }

/** deep copy (JSON documents are values already; kept for readability where the JavaScript cloned) */
inline Json clone(const Json& j) { return j; }

/** UTF-8 path string -> filesystem path (and back) */
std::filesystem::path fsPath(const std::string& s);
inline std::string pathStr(const std::filesystem::path& p) { const auto u = p.u8string(); return std::string(u.begin(), u.end()); }

/** read a whole text / binary file; write one (creating the directory) */
std::string readText(const std::string& path);
bool writeText(const std::string& path, const std::string& data);

/** `bytes` random bytes from the operating system's cryptographic generator, as lower-case hex */
std::string randomHex(std::size_t bytes);

}  // namespace aptuner::app
