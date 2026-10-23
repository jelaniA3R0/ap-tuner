#include "common.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "aptuner/core/jsnum.hpp"

#ifdef _WIN32
#include <windows.h>
#include <bcrypt.h>
#endif

namespace aptuner::app {

log::Params toParams(const Json& p) {
  log::Params out;
  if (!p.isObject()) return out;
  for (const auto& [k, v] : p.items())
    if (v.isNumber()) out.set(k, v.num());
  return out;
}

std::string fmt4(const Json& v) {
  if (v.isNumber()) return core::jsToString(core::jsRoundPrecision(v.num(), 4));
  if (v.isNull()) return "undefined";
  if (v.isString()) return v.str();
  return v.dump();
}

namespace {
std::tm utcNow(long long& ms) {
  const auto now = std::chrono::system_clock::now();
  ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  const std::time_t t = static_cast<std::time_t>(ms / 1000);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &t);
#else
  gmtime_r(&t, &tm);
#endif
  return tm;
}
std::tm localOf(double epochMs) {
  const std::time_t t = static_cast<std::time_t>(epochMs / 1000);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  return tm;
}
}  // namespace

std::string isoNow() {
  long long ms = 0;
  const std::tm tm = utcNow(ms);
  char buf[96];
  std::snprintf(buf, sizeof buf, "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                tm.tm_min, tm.tm_sec, static_cast<int>(ms % 1000));
  return buf;
}

std::string isoStampForFile() {
  std::string s = isoNow();
  for (char& c : s)
    if (c == ':' || c == '.') c = '-';
  return s;
}

std::string localNow() {
  const double now = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                             std::chrono::system_clock::now().time_since_epoch())
                                             .count());
  const std::tm tm = localOf(now);
  char buf[96];
  std::snprintf(buf, sizeof buf, "%d/%d/%04d, %d:%02d:%02d %s", tm.tm_mon + 1, tm.tm_mday, tm.tm_year + 1900,
                (tm.tm_hour % 12) ? tm.tm_hour % 12 : 12, tm.tm_min, tm.tm_sec, tm.tm_hour < 12 ? "AM" : "PM");
  return buf;
}

std::string localTime(double epochMs) {
  const std::tm tm = localOf(epochMs);
  char buf[96];
  std::snprintf(buf, sizeof buf, "%d:%02d:%02d %s", (tm.tm_hour % 12) ? tm.tm_hour % 12 : 12, tm.tm_min, tm.tm_sec,
                tm.tm_hour < 12 ? "AM" : "PM");
  return buf;
}

std::filesystem::path fsPath(const std::string& s) {
  return std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(s.data()), s.size()));
}

std::string readText(const std::string& path) {
  std::ifstream f(fsPath(path), std::ios::binary);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

bool writeText(const std::string& path, const std::string& data) {
  std::error_code ec;
  const auto p = fsPath(path);
  if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path(), ec);
  std::ofstream f(p, std::ios::binary);
  if (!f) return false;
  f << data;
  return static_cast<bool>(f);
}

std::string randomHex(std::size_t bytes) {
  std::string raw(bytes, 0);
#ifdef _WIN32
  if (BCryptGenRandom(nullptr, reinterpret_cast<PUCHAR>(raw.data()), static_cast<ULONG>(raw.size()), BCRYPT_USE_SYSTEM_PREFERRED_RNG) != 0)
    throw std::runtime_error("no random numbers from the system");
#else
  std::ifstream f("/dev/urandom", std::ios::binary);
  if (!f.read(raw.data(), static_cast<std::streamsize>(raw.size()))) throw std::runtime_error("no random numbers from the system");
#endif
  static const char* hex = "0123456789abcdef";
  std::string out;
  for (const char c : raw) {
    const auto b = static_cast<unsigned char>(c);
    out += hex[b >> 4];
    out += hex[b & 15];
  }
  return out;
}

}  // namespace aptuner::app
