#include "aptuner/io/logdl.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace aptuner::io {

using core::Json;
using rt::Loop;

namespace {
constexpr int kBlock = 90;   // LOG_DATA payload size
}

rt::Future<std::vector<LogEntry>> LogDownloader::list(double timeoutMs) {
  rt::Promise<std::vector<LogEntry>> p;
  struct St {
    std::map<int, LogEntry> logs;
    std::optional<double> total;
    std::uint64_t handler = 0, hard = 0, quiet = 0;
    bool finished = false;
  };
  auto st = std::make_shared<St>();
  Link* link = &link_;
  auto done = [link, st, p]() {
    if (st->finished) return;
    st->finished = true;
    link->off(st->handler);
    Loop::main().clear(st->hard);
    Loop::main().clear(st->quiet);
    std::vector<LogEntry> v;
    for (auto& [id, e] : st->logs) v.push_back(e);
    p.resolve(v);
  };
  st->handler = link_.on("LOG_ENTRY", [st, done](const Json& f) {
    st->total = f["num_logs"].numOr(0);
    if (*st->total == 0) return done();
    LogEntry e;
    e.id = static_cast<int>(f["id"].numOr(0));
    e.size = f["size"].numOr(0);
    e.timeUtc = f["time_utc"].numOr(0);
    e.last = f["last_log_num"].numOr(0);
    st->logs[e.id] = e;
    Loop::main().clear(st->quiet);
    st->quiet = Loop::main().setTimeout(700, done);
    if (st->total && static_cast<double>(st->logs.size()) >= *st->total) done();
  });
  st->hard = Loop::main().setTimeout(timeoutMs, done);
  link_.send("LOG_REQUEST_LIST", Json::obj({{"target_system", link_.targetSys()}, {"target_component", link_.targetComp()},
                                            {"start", 0}, {"end", 0xffff}}));
  return p.future();
}

rt::Future<std::vector<std::uint8_t>> LogDownloader::readRange(int id, double ofs, double len,
                                                               std::function<void(double, double)> onProgress,
                                                               double idleMs, int maxRounds) {
  rt::Promise<std::vector<std::uint8_t>> p;
  struct St {
    std::size_t len = 0, nBlocks = 0, gotN = 0;
    std::vector<std::uint8_t> got, buf;
    std::optional<std::size_t> eofAt;
    double lastRx = 0;
    int rounds = 0;
    std::uint64_t handler = 0, timer = 0;
    bool finished = false;
  };
  auto st = std::make_shared<St>();
  st->len = static_cast<std::size_t>(len);
  st->nBlocks = (st->len + kBlock - 1) / kBlock;
  st->got.assign(st->nBlocks, 0);
  st->buf.assign(st->len, 0);
  st->lastRx = Loop::now();
  Link* link = &link_;
  auto finish = [link, st, p](const std::string& err) {
    if (st->finished) return;
    st->finished = true;
    link->off(st->handler);
    Loop::main().clear(st->timer);
    if (!err.empty()) {
      p.reject(err);
      return;
    }
    std::vector<std::uint8_t> out = st->buf;
    out.resize(st->eofAt ? *st->eofAt : st->len);
    p.resolve(out);
  };
  auto needBlocks = [st]() { return st->eofAt ? (*st->eofAt + kBlock - 1) / kBlock : st->nBlocks; };
  auto complete = [st, needBlocks]() {
    const std::size_t nb = needBlocks();
    for (std::size_t k = 0; k < nb; k++)
      if (!st->got[k]) return false;
    return true;
  };
  auto request = [link, st, needBlocks, finish, id, ofs, maxRounds]() {
    // first missing run of blocks
    const std::size_t nb = needBlocks();
    std::size_t a = 0;
    while (a < nb && st->got[a]) a++;
    if (a >= nb) return finish("");
    std::size_t b = a;
    while (b < nb && !st->got[b]) b++;
    st->rounds++;
    if (st->rounds > maxRounds)
      return finish("log download stalled at " + std::to_string(static_cast<long long>(ofs) + static_cast<long long>(a * kBlock)) + " bytes");
    st->lastRx = Loop::now();
    link->send("LOG_REQUEST_DATA", Json::obj({{"target_system", link->targetSys()}, {"target_component", link->targetComp()},
                                              {"id", id}, {"ofs", ofs + static_cast<double>(a * kBlock)},
                                              {"count", static_cast<double>(std::min(st->len, b * kBlock) - a * kBlock)}}));
  };
  st->handler = link_.on("LOG_DATA", [st, id, ofs, onProgress, complete, finish](const Json& f) {
    if (f["id"].numOr(-1) != id) return;
    st->lastRx = Loop::now();
    const double relD = f["ofs"].numOr(0) - ofs;
    const double count = f["count"].numOr(0);
    if (count < kBlock && relD >= 0 && relD <= static_cast<double>(st->len)) {
      const std::size_t e = std::min(st->len, static_cast<std::size_t>(relD + count));
      st->eofAt = st->eofAt ? std::min(*st->eofAt, e) : e;
    }
    if (relD < 0 || relD >= static_cast<double>(st->len)) return;
    const std::size_t rel = static_cast<std::size_t>(relD);
    if (rel % kBlock) return;
    const std::size_t k = rel / kBlock;
    if (!st->got[k]) {
      st->got[k] = 1;
      st->gotN++;
    }
    const std::size_t n = std::min(static_cast<std::size_t>(count), st->len - rel);
    const Json& data = f["data"];
    for (std::size_t j = 0; j < n; j++) st->buf[rel + j] = static_cast<std::uint8_t>(data[j].numOr(0));
    if (onProgress && st->gotN % 64 == 0) onProgress(static_cast<double>(std::min(st->len, st->gotN * kBlock)), static_cast<double>(st->len));
    if (complete()) finish("");
  });
  st->timer = Loop::main().setInterval(200, [st, request, idleMs] {
    if (Loop::now() - st->lastRx > idleMs) request();
  });
  request();
  return p.future();
}

void LogDownloader::end() {
  link_.send("LOG_REQUEST_END", Json::obj({{"target_system", link_.targetSys()}, {"target_component", link_.targetComp()}}));
}

rt::Task<FetchResult> LogDownloader::fetchWindow(LogEntry log, std::optional<double> tStart, double tEnd,
                                                 std::function<void(const Json&)> onProgress, double marginBefore,
                                                 double marginAfter) {
  auto progress = [onProgress](const std::string& stage, double done, double total) {
    if (onProgress) onProgress(Json::obj({{"stage", stage}, {"done", done}, {"total", total}}));
  };
  const int id = log.id;
  const double size = log.size;
  const double HEAD = std::min(size, 1048576.0);
  progress("head", 0, HEAD);
  std::vector<std::uint8_t> head =
      co_await readRange(id, 0, HEAD, [progress](double d, double t) { progress("head", d, t); });
  FetchResult r;
  r.fmts = log::readFormats(head.data(), head.size());
  std::set<std::string> names;
  for (const auto& [t, f] : r.fmts) names.insert(f.name);
  if (!names.count("PIDR") && !names.count("PIQR") && !names.count("RATE"))
    throw rt::Error("the log head has no rate-loop formats (is this a flight log?)");
  if (!tStart || size <= HEAD) {
    std::vector<std::uint8_t> rest;
    if (size > HEAD)
      rest = co_await readRange(id, HEAD, size - HEAD, [progress, HEAD, size](double d, double) { progress("whole log", HEAD + d, size); });
    r.buf = head;
    r.buf.insert(r.buf.end(), rest.begin(), rest.end());
    r.start = 0;
    r.end = size;
    r.bytes = size;
    r.whole = true;
    co_return r;
  }
  const std::size_t headEnd = log::completeLength(head.data(), head.size(), r.fmts);
  auto probe = [this, id, size](double o) { return readRange(id, o, std::min(4096.0, size - o), nullptr, 800, 6); };
  // log time span: last timestamp near the end
  const std::vector<std::uint8_t> tail = co_await probe(std::max(static_cast<double>(headEnd), size - 4096));
  r.lastT = log::lastTime(tail.data(), tail.size(), r.fmts);
  const std::size_t from = headEnd > 8192 ? headEnd - 8192 : 0;
  r.firstT = log::firstTime(head.data() + from, head.size() - from, r.fmts);
  if (r.lastT && tEnd > *r.lastT + 1) throw rt::Error("the marked window is after the end of this log", "after");
  if (r.firstT && *tStart < *r.firstT - 30) throw rt::Error("the marked window is before the start of this log", "before");
  // bisection: smallest offset whose first timestamp >= t
  auto locate = [&](double t) -> rt::Task<double> {
    double lo = static_cast<double>(headEnd), hi = size;
    int n = 0;
    while (hi - lo > 8192 && n < 40) {
      const double mid = std::floor((lo + hi) / 2);
      n++;
      progress("finding the marked 30 s", n, 22);
      const std::vector<std::uint8_t> b = co_await probe(mid);
      const auto tm = log::firstTime(b.data(), b.size(), r.fmts);
      if (!tm || *tm < t)
        lo = mid;
      else
        hi = mid;
    }
    co_return lo;
  };
  const double start = co_await locate(*tStart - marginBefore);
  const double end = std::min(size, (co_await locate(tEnd + marginAfter)) + 8192);
  const std::vector<std::uint8_t> mid =
      co_await readRange(id, start, end - start, [progress](double d, double t) { progress("downloading the marked 30 s", d, t); });
  const long long s0 = log::firstSync(mid.data(), mid.size(), r.fmts, 0);
  r.buf.assign(head.begin(), head.begin() + static_cast<std::ptrdiff_t>(headEnd));
  r.buf.insert(r.buf.end(), mid.begin() + (s0 > 0 ? s0 : 0), mid.end());
  r.start = start;
  r.end = end;
  r.bytes = HEAD + (end - start);
  r.whole = false;
  co_return r;
}

}  // namespace aptuner::io
