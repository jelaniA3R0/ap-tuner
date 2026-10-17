// Build information: version, compiler and platform, printed by every program and kept in session files
// so a result can always be traced back to the exact build that produced it.
#pragma once

#include <string>

namespace aptuner::core {

struct BuildInfo {
  std::string version;   // e.g. "0.1.0"
  std::string compiler;  // e.g. "clang 21.1.2"
  std::string platform;  // e.g. "windows-x86_64"
  int cppStandard = 0;   // value of __cplusplus, e.g. 202002
};

BuildInfo buildInfo();

/** one line for logs and --version: "AP Tuner 0.1.0 (clang 21.1.2, windows-x86_64, C++202002)" */
std::string buildInfoLine();

}  // namespace aptuner::core
