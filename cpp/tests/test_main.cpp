// Test runner: runs every registered TEST, or only those whose name contains one of the arguments; an argument
// "-text" leaves out the tests whose name contains text (e.g. "-live_ -wv_": everything but the real-time suites).
#include <cstdio>
#include <string>
#include <vector>

#include "aptuner/core/build_info.hpp"
#include "check.hpp"

int main(int argc, char** argv) {
  using namespace aptuner::test;
  std::vector<std::string> include, exclude;
  for (int i = 1; i < argc; i++) {
    if (argv[i][0] == '-')
      exclude.push_back(argv[i] + 1);
    else
      include.push_back(argv[i]);
  }
  std::printf("%s\n", aptuner::core::buildInfoLine().c_str());
  int run = 0, failedTests = 0;
  for (const auto& c : registry()) {
    bool take = include.empty();
    for (const auto& f : include)
      if (c.name.find(f) != std::string::npos) take = true;
    for (const auto& f : exclude)
      if (c.name.find(f) != std::string::npos) take = false;
    if (!take) continue;
    const int before = failures();
    c.fn();
    ++run;
    const bool ok = failures() == before;
    if (!ok) ++failedTests;
    std::printf("%s %s\n", ok ? "  ok  " : "  FAIL", c.name.c_str());
    std::fflush(stdout);
  }
  std::printf("%d test(s), %d failed\n", run, failedTests);
  return failedTests == 0 && run > 0 ? 0 : 1;
}
