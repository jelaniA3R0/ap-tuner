// aptuner: the application (HTTP + SSE server serving the web/ user interface, live tuning, guided flight).
// Phase 0: placeholder that builds everywhere; the server arrives in phase 4.
#include <cstdio>
#include <string>

#include "aptuner/core/build_info.hpp"

int main(int argc, char** argv) {
  std::puts(aptuner::core::buildInfoLine().c_str());
  if (argc >= 2 && std::string(argv[1]) == "--version") return 0;
  std::puts("The C++ application is being ported (server: phase 4). Use \"start AP Tuner.cmd\" meanwhile.");
  return 0;
}
