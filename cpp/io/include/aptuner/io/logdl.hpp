// Download DataFlash logs from the autopilot over MAVLink (LOG_REQUEST_LIST / LOG_REQUEST_DATA). Port of
// src/mavlink/logdl.js: fetchWindow() pulls the log head (formats, parameters) plus the bytes around a time
// window found by bisection on small probes, so a 30 s piece of a long log downloads in seconds.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <vector>

#include "aptuner/core/json.hpp"
#include "aptuner/io/link.hpp"
#include "aptuner/log/dfreader.hpp"
#include "aptuner/rt/async.hpp"

namespace aptuner::io {

struct LogEntry {
  int id = 0;
  double size = 0, timeUtc = 0, last = 0;
};

struct FetchResult {
  std::vector<std::uint8_t> buf;   // a parseable (possibly partial) log
  std::map<int, log::Fmt> fmts;
  double start = 0, end = 0, bytes = 0;
  std::optional<double> firstT, lastT;
  bool whole = false;
};

class LogDownloader {
 public:
  explicit LogDownloader(Link& link) : link_(link) {}
  /** logs on the autopilot, sorted by id */
  rt::Future<std::vector<LogEntry>> list(double timeoutMs = 4000);
  /** bytes [ofs, ofs + len) of a log (shorter at the end of the log); re-requests gaps */
  rt::Future<std::vector<std::uint8_t>> readRange(int id, double ofs, double len,
                                                  std::function<void(double, double)> onProgress = nullptr,
                                                  double idleMs = 1200, int maxRounds = 12);
  void end();
  /** head + the part covering [tStart, tEnd] (TimeUS seconds); tStart unset = the whole log.
   *  onProgress({stage, done, total}); rejects with code "after" / "before" when the window is outside the log */
  rt::Task<FetchResult> fetchWindow(LogEntry log, std::optional<double> tStart, double tEnd,
                                    std::function<void(const core::Json&)> onProgress = nullptr,
                                    double marginBefore = 3, double marginAfter = 1);

 private:
  Link& link_;
};

}  // namespace aptuner::io
