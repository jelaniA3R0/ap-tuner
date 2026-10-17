// Test runner: runs every registered TEST, only those whose name contains argv[1], or (argv[1] = -text) all but those.
#include <cstdio>
#include <cstring>

#include "aptuner/core/build_info.hpp"
#include "check.hpp"

int main(int argc, char** argv) {
  using namespace aptuner::test;
  const char* filter = argc >= 2 ? argv[1] : nullptr;
  const bool exclude = filter && filter[0] == '-';
  if (exclude) filter++;
  std::printf("%s\n", aptuner::core::buildInfoLine().c_str());
  int run = 0, failedTests = 0;
  for (const auto& c : registry()) {
    if (filter && (std::strstr(c.name.c_str(), filter) != nullptr) == exclude) continue;
    const int before = failures();
    c.fn();
    ++run;
    const bool ok = failures() == before;
    if (!ok) ++failedTests;
    std::printf("%s %s\n", ok ? "  ok  " : "  FAIL", c.name.c_str());
  }
  std::printf("%d test(s), %d failed\n", run, failedTests);
  return failedTests == 0 && run > 0 ? 0 : 1;
}
