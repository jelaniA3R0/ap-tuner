#include "aptuner/rt/loop.hpp"

#include <cstdio>
#include <exception>

namespace aptuner::rt {

Loop& Loop::main() {
  static Loop loop;
  return loop;
}

void Loop::post(Fn fn) {
  {
    std::lock_guard<std::mutex> lk(mu_);
    posted_.push_back(std::move(fn));
  }
  cv_.notify_one();
}

void Loop::queueMicrotask(Fn fn) { micro_.push_back(std::move(fn)); }

std::uint64_t Loop::setTimeout(double ms, Fn fn) {
  const std::uint64_t id = nextId_++;
  const auto due = Clock::now() + std::chrono::microseconds(static_cast<long long>((ms > 0 ? ms : 0) * 1000));
  timers_[id] = Timer{due, 0, std::move(fn)};
  due_.emplace(due, id);
  return id;
}

std::uint64_t Loop::setInterval(double ms, Fn fn) {
  const std::uint64_t id = nextId_++;
  const double iv = ms > 1 ? ms : 1;
  const auto due = Clock::now() + std::chrono::microseconds(static_cast<long long>(iv * 1000));
  timers_[id] = Timer{due, iv, std::move(fn)};
  due_.emplace(due, id);
  return id;
}

void Loop::clear(std::uint64_t id) { timers_.erase(id); }   // its due_ entry is skipped when it comes up

double Loop::now() {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count());
}

void Loop::drainMicrotasks() {
  while (!micro_.empty()) {
    Fn f = std::move(micro_.front());
    micro_.pop_front();
    try {
      f();
    } catch (const std::exception& e) {
      std::fprintf(stderr, "[loop] uncaught exception in microtask: %s\n", e.what());
    }
  }
}

bool Loop::step(Clock::time_point waitUntil) {
  // 1) due timers
  const auto now = Clock::now();
  while (!due_.empty() && due_.begin()->first <= now) {
    const std::uint64_t id = due_.begin()->second;
    const auto at = due_.begin()->first;
    due_.erase(due_.begin());
    auto it = timers_.find(id);
    if (it == timers_.end() || it->second.due != at) continue;   // cleared or rescheduled
    Fn fn = it->second.fn;
    if (it->second.intervalMs > 0) {
      it->second.due = now + std::chrono::microseconds(static_cast<long long>(it->second.intervalMs * 1000));
      due_.emplace(it->second.due, id);
    } else {
      timers_.erase(it);
    }
    try {
      fn();
    } catch (const std::exception& e) {
      std::fprintf(stderr, "[loop] uncaught exception in timer: %s\n", e.what());
    }
    drainMicrotasks();
  }
  // 2) posted work (or wait for it / the next timer)
  std::deque<Fn> work;
  {
    std::unique_lock<std::mutex> lk(mu_);
    auto until = waitUntil;
    if (!due_.empty() && due_.begin()->first < until) until = due_.begin()->first;
    if (posted_.empty() && !stop_) cv_.wait_until(lk, until, [&] { return !posted_.empty() || stop_.load(); });
    work.swap(posted_);
  }
  for (auto& fn : work) {
    try {
      fn();
    } catch (const std::exception& e) {
      std::fprintf(stderr, "[loop] uncaught exception in task: %s\n", e.what());
    }
    drainMicrotasks();
  }
  return !stop_;
}

void Loop::run() {
  owner_ = std::this_thread::get_id();
  stop_ = false;
  while (step(Clock::now() + std::chrono::hours(1))) {
  }
}

bool Loop::runUntil(const std::function<bool()>& pred, double timeoutMs) {
  owner_ = std::this_thread::get_id();
  stop_ = false;
  const auto end = Clock::now() + std::chrono::microseconds(static_cast<long long>(timeoutMs * 1000));
  drainMicrotasks();
  while (!pred() && Clock::now() < end) {
    if (!step(std::min(end, Clock::now() + std::chrono::milliseconds(50)))) break;
  }
  return pred();
}

void Loop::stop() {
  stop_ = true;
  cv_.notify_all();
}

}  // namespace aptuner::rt
