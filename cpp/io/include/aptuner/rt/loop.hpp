// Single-threaded event loop with timers and a microtask queue: the JavaScript execution model in C++.
// All application logic runs on the loop thread; I/O threads hand their results over with post().
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <thread>

namespace aptuner::rt {

class Loop {
 public:
  using Fn = std::function<void()>;

  /** the process-wide loop the application uses */
  static Loop& main();

  /** run fn on the loop thread (thread-safe; may be called from any thread) */
  void post(Fn fn);
  /** run fn after the current task, before timers and other posted work (loop thread only) */
  void queueMicrotask(Fn fn);
  /** call fn after ms milliseconds / every ms milliseconds (loop thread only); returns an id for clear() */
  std::uint64_t setTimeout(double ms, Fn fn);
  std::uint64_t setInterval(double ms, Fn fn);
  void clear(std::uint64_t id);

  /** process work until stop() (or until idle when untilIdle) */
  void run();
  /** run until pred() is true or timeoutMs passes; returns pred() (for tests and command-line tools) */
  bool runUntil(const std::function<bool()>& pred, double timeoutMs);
  void stop();
  bool inLoopThread() const { return std::this_thread::get_id() == owner_.load(); }

  /** wall-clock milliseconds since the epoch (Date.now()) */
  static double now();

 private:
  using Clock = std::chrono::steady_clock;
  struct Timer {
    Clock::time_point due;
    double intervalMs = 0;   // 0 = one shot
    Fn fn;
  };
  bool step(Clock::time_point waitUntil);
  void drainMicrotasks();

  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<Fn> posted_;
  std::deque<Fn> micro_;
  std::map<std::uint64_t, Timer> timers_;
  std::multimap<Clock::time_point, std::uint64_t> due_;
  std::uint64_t nextId_ = 1;
  std::atomic<bool> stop_{false};
  std::atomic<std::thread::id> owner_{};
};

}  // namespace aptuner::rt
