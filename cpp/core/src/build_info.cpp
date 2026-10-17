#include "aptuner/core/build_info.hpp"

#include "aptuner/version.hpp"

namespace aptuner::core {

namespace {

std::string compilerName() {
#if defined(__clang__)
  return "clang " + std::to_string(__clang_major__) + "." + std::to_string(__clang_minor__) + "." +
         std::to_string(__clang_patchlevel__);
#elif defined(__GNUC__)
  return "gcc " + std::to_string(__GNUC__) + "." + std::to_string(__GNUC_MINOR__) + "." +
         std::to_string(__GNUC_PATCHLEVEL__);
#elif defined(_MSC_VER)
  return "msvc " + std::to_string(_MSC_VER);
#else
  return "unknown compiler";
#endif
}

std::string platformName() {
#if defined(_WIN32)
  std::string os = "windows";
#elif defined(__APPLE__)
  std::string os = "macos";
#elif defined(__linux__)
  std::string os = "linux";
#else
  std::string os = "unknown";
#endif
#if defined(__x86_64__) || defined(_M_X64)
  return os + "-x86_64";
#elif defined(__aarch64__) || defined(_M_ARM64)
  return os + "-arm64";
#else
  return os;
#endif
}

}  // namespace

BuildInfo buildInfo() {
  BuildInfo b;
  b.version = kVersion;
  b.compiler = compilerName();
  b.platform = platformName();
  b.cppStandard = static_cast<int>(__cplusplus);
  return b;
}

std::string buildInfoLine() {
  const BuildInfo b = buildInfo();
  return "AP Tuner " + b.version + " (" + b.compiler + ", " + b.platform + ", C++" + std::to_string(b.cppStandard) +
         ")";
}

}  // namespace aptuner::core
