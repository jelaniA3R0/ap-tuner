// aptuner-analyze: command-line log analysis (C++ counterpart of tools/analyze-log.js).
// Phase 0: the program exists and builds everywhere; the analysis itself arrives in phase 2.
#include <cstdio>
#include <string>

#include "aptuner/core/build_info.hpp"

namespace {

void usage() {
  std::puts("usage: aptuner-analyze <log.bin> [conservative|normal|aggressive] [copter|vtol|plane]");
  std::puts("       aptuner-analyze --version");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc >= 2 && (std::string(argv[1]) == "--version" || std::string(argv[1]) == "-v")) {
    std::puts(aptuner::core::buildInfoLine().c_str());
    return 0;
  }
  if (argc < 2 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") {
    usage();
    return argc < 2 ? 2 : 0;
  }
  std::fprintf(stderr, "%s\nlog analysis is not ported yet (phase 2); use: node tools/analyze-log.js %s\n",
               aptuner::core::buildInfoLine().c_str(), argv[1]);
  return 3;
}
