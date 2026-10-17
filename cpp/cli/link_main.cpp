// aptuner-link: link check against a vehicle or the simulator.
//   aptuner-link auto                       auto-connect and report what answered
//   aptuner-link tcp 127.0.0.1 5760         connect, show the vehicle, read parameters
//   aptuner-link serial COM53 57600
//   ... [--set NAME VALUE] [--logs] [--download ID BYTES]
#include <cstdio>
#include <string>
#include <vector>

#include "aptuner/core/build_info.hpp"
#include "aptuner/io/autoconnect.hpp"
#include "aptuner/io/link.hpp"
#include "aptuner/io/logdl.hpp"
#include "aptuner/io/sysinfo.hpp"
#include "aptuner/rt/loop.hpp"

using aptuner::core::Json;
using namespace aptuner;

namespace {

struct Args {
  std::vector<std::string> pos;
  std::string setName;
  double setValue = 0;
  bool logs = false;
  int dlId = -1;
  double dlBytes = 0;
};

rt::Task<int> run(io::Link& link, Args a) {
  if (!a.pos.empty() && a.pos[0] == "auto") {
    io::AutoConnectState st;
    const Json found = co_await io::autoconnect(link, st, [](const Json& p) {
      if (p.has("trying")) std::printf("  trying %s\n", p["trying"].str().c_str());
    });
    std::printf("Mission Planner running: %s\n", st.missionPlanner ? "yes" : "no");
    for (const auto& t : st.tried.arr())
      std::printf("  tried %s: %s\n", t["label"].str().c_str(), t["result"].str().c_str());
    if (found.isNull()) {
      std::printf("no vehicle found\n");
      co_return 1;
    }
    std::printf("connected: %s\n", found["label"].str().c_str());
  } else {
    Json opts;
    if (a.pos.size() >= 3 && a.pos[0] == "serial")
      opts = Json::obj({{"kind", "serial"}, {"port", a.pos[1]}, {"baud", std::stod(a.pos[2])}});
    else if (a.pos.size() >= 3)
      opts = Json::obj({{"kind", a.pos[0]}, {"host", a.pos[1]}, {"port", std::stod(a.pos[2])}});
    else
      opts = Json::obj({{"kind", "tcp"}, {"host", "127.0.0.1"}, {"port", 5760}});
    link.connect(opts);
    const double t0 = rt::Loop::now();
    while (!link.alive() && rt::Loop::now() - t0 < 15000) co_await rt::sleep(100);
    if (!link.alive()) {
      std::printf("no heartbeat from %s\n", link.description().c_str());
      co_return 1;
    }
  }
  std::printf("vehicle: %s\n", link.vehicle().dump().c_str());
  const Json ps = co_await link.requestParams({"SCHED_LOOP_RATE", "INS_GYRO_FILTER", "ATC_RAT_RLL_P", "Q_A_RAT_RLL_P", "NOT_A_PARAM"}, 3);
  std::printf("params: %s\n", ps.dump().c_str());
  if (!a.setName.empty()) {
    const double before = co_await link.requestParam(a.setName);
    const double echo = co_await link.setParam(a.setName, a.setValue);
    const double after = co_await link.requestParam(a.setName);
    std::printf("set %s: was %g, echo %g, read back %g\n", a.setName.c_str(), before, echo, after);
  }
  if (a.logs || a.dlId >= 0) {
    io::LogDownloader dl(link);
    const auto logs = co_await dl.list();
    std::printf("%zu log(s) on the vehicle\n", logs.size());
    for (const auto& l : logs) std::printf("  log %d: %.0f bytes\n", l.id, l.size);
    if (a.dlId >= 0) {
      const double t0 = rt::Loop::now();
      const auto bytes = co_await dl.readRange(a.dlId, 0, a.dlBytes);
      const double s = (rt::Loop::now() - t0) / 1000;
      const auto fmts = log::readFormats(bytes.data(), bytes.size());
      std::printf("downloaded %zu bytes of log %d in %.1f s (%.0f kB/s), %zu formats in the head\n", bytes.size(), a.dlId, s,
                  static_cast<double>(bytes.size()) / 1024 / (s > 0 ? s : 1), fmts.size());
      dl.end();
    }
  }
  std::printf("link: %s, rx %llu msgs, %llu bytes; rates %s\n", link.description().c_str(),
              static_cast<unsigned long long>(link.stats().rx), static_cast<unsigned long long>(link.stats().bytes),
              link.msgRates().dump().c_str());
  co_return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; i++) {
    const std::string s = argv[i];
    if (s == "--version") {
      std::puts(core::buildInfoLine().c_str());
      return 0;
    }
    if (s == "--ports") {
      for (const auto& p : io::listSerialPorts()) std::printf("%s  %s\n", p.port.c_str(), p.name.c_str());
      return 0;
    }
    if (s == "--set" && i + 2 < argc) {
      a.setName = argv[++i];
      a.setValue = std::stod(argv[++i]);
    } else if (s == "--logs") {
      a.logs = true;
    } else if (s == "--download" && i + 2 < argc) {
      a.dlId = std::stoi(argv[++i]);
      a.dlBytes = std::stod(argv[++i]);
    } else {
      a.pos.push_back(s);
    }
  }
  auto& loop = rt::Loop::main();
  io::Link link;
  link.on("status", [](const Json& s) { std::printf("  [link] %s\n", s.strOr("").c_str()); });
  int code = -1;
  loop.post([&] {
    rt::Future<int> f = run(link, a);
    f.then([&, f] {
      try {
        code = f.get();
      } catch (const std::exception& e) {
        std::printf("error: %s\n", e.what());
        code = 2;
      }
      loop.stop();
    });
  });
  loop.runUntil([&] { return code >= 0; }, 600000);
  link.disconnect();
  return code < 0 ? 3 : code;
}
