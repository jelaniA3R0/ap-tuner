// Promises and coroutine tasks on the event loop: the JavaScript async / await model.
//   Task<int> f() { co_await sleep(100); int v = co_await other(); co_return v + 1; }
// Tasks start eagerly (like an async function), continuations run as loop microtasks, exceptions propagate
// to whoever awaits (like a rejected promise).
#pragma once

#include <coroutine>
#include <cstdio>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "aptuner/rt/loop.hpp"

namespace aptuner::rt {

struct Unit {};

/** error with an optional machine-readable code (JavaScript: Object.assign(new Error(msg), {code})) */
class Error : public std::runtime_error {
 public:
  explicit Error(const std::string& msg, std::string code = "") : std::runtime_error(msg), code_(std::move(code)) {}
  const std::string& code() const { return code_; }

 private:
  std::string code_;
};

template <class T>
using Stored = std::conditional_t<std::is_void_v<T>, Unit, T>;

template <class T>
struct SharedState {
  std::optional<Stored<T>> value;
  std::exception_ptr error;
  bool done = false;
  std::vector<std::function<void()>> conts;
};

template <class T>
class Future;

template <class T>
class Promise {
 public:
  Promise() : s_(std::make_shared<SharedState<T>>()) {}
  Future<T> future() const;
  bool done() const { return s_->done; }

  template <class U = T, std::enable_if_t<!std::is_void_v<U>, int> = 0>
  void resolve(U v) const {
    if (s_->done) return;
    s_->value.emplace(std::move(v));
    finish();
  }
  template <class U = T, std::enable_if_t<std::is_void_v<U>, int> = 0>
  void resolve() const {
    if (s_->done) return;
    s_->value.emplace();
    finish();
  }
  void reject(std::exception_ptr e) const {
    if (s_->done) return;
    s_->error = std::move(e);
    finish();
  }
  void reject(const std::string& msg, const std::string& code = "") const {
    reject(std::make_exception_ptr(Error(msg, code)));
  }

 private:
  void finish() const {
    s_->done = true;
    auto conts = std::move(s_->conts);
    s_->conts.clear();
    for (auto& c : conts) Loop::main().queueMicrotask(std::move(c));
  }
  std::shared_ptr<SharedState<T>> s_;
  friend class Future<T>;
};

template <class T>
class Future {
 public:
  Future() = default;
  explicit Future(std::shared_ptr<SharedState<T>> s) : s_(std::move(s)) {}
  bool valid() const { return s_ != nullptr; }
  bool ready() const { return s_ && s_->done; }
  bool failed() const { return s_ && s_->done && s_->error; }
  /** run fn (on the loop) once settled */
  void then(std::function<void()> fn) const {
    if (s_->done)
      Loop::main().queueMicrotask(std::move(fn));
    else
      s_->conts.push_back(std::move(fn));
  }
  /** value of a settled future (rethrows its error) */
  Stored<T> get() const {
    if (s_->error) std::rethrow_exception(s_->error);
    return *s_->value;
  }
  std::exception_ptr error() const { return s_->error; }

  // awaitable
  bool await_ready() const noexcept { return s_->done; }
  void await_suspend(std::coroutine_handle<> h) const { s_->conts.push_back([h] { h.resume(); }); }
  T await_resume() const {
    if (s_->error) std::rethrow_exception(s_->error);
    if constexpr (!std::is_void_v<T>) return *s_->value;
  }

 private:
  std::shared_ptr<SharedState<T>> s_;
};

template <class T>
Future<T> Promise<T>::future() const {
  return Future<T>(s_);
}

template <class T>
Future<T> resolved(Stored<T> v) {
  Promise<T> p;
  if constexpr (std::is_void_v<T>)
    p.resolve();
  else
    p.resolve(std::move(v));
  return p.future();
}

// ---------------------------------------------------------------- coroutine task

template <class T>
class Task;

namespace detail {
template <class T>
struct TaskPromiseBase {
  Promise<T> p;
  std::suspend_never initial_suspend() noexcept { return {}; }
  std::suspend_never final_suspend() noexcept { return {}; }
  void unhandled_exception() { p.reject(std::current_exception()); }
  template <class U>
  void return_value(U&& v) {
    p.resolve(T(std::forward<U>(v)));
  }
};
template <>
struct TaskPromiseBase<void> {
  Promise<void> p;
  std::suspend_never initial_suspend() noexcept { return {}; }
  std::suspend_never final_suspend() noexcept { return {}; }
  void unhandled_exception() { p.reject(std::current_exception()); }
  void return_void() { p.resolve(); }
};
}  // namespace detail

/** coroutine return type; awaitable; convertible to its Future */
template <class T = void>
class Task {
 public:
  struct promise_type : detail::TaskPromiseBase<T> {
    Task get_return_object() { return Task(this->p.future()); }
  };
  explicit Task(Future<T> f) : f_(std::move(f)) {}
  const Future<T>& future() const { return f_; }
  operator Future<T>() const { return f_; }
  bool await_ready() const noexcept { return f_.await_ready(); }
  void await_suspend(std::coroutine_handle<> h) const { f_.await_suspend(h); }
  T await_resume() const { return f_.await_resume(); }

 private:
  Future<T> f_;
};

// ---------------------------------------------------------------- helpers

/** a future resolved after ms milliseconds (await sleep(ms)) */
inline Future<void> sleep(double ms) {
  Promise<void> p;
  Loop::main().setTimeout(ms, [p] { p.resolve(); });
  return p.future();
}

/** resolves when all futures have settled; rethrows the first error (Promise.all) */
template <class T>
Future<std::vector<Stored<T>>> whenAll(std::vector<Future<T>> fs) {
  Promise<std::vector<Stored<T>>> p;
  auto left = std::make_shared<std::size_t>(fs.size());
  if (fs.empty()) {
    p.resolve({});
    return p.future();
  }
  for (const auto& f : fs)
    f.then([p, left, fs] {
      if (--*left) return;
      std::vector<Stored<T>> out;
      for (const auto& g : fs) {
        if (g.error()) {
          p.reject(g.error());
          return;
        }
        out.push_back(g.get());
      }
      p.resolve(std::move(out));
    });
  return p.future();
}

/** run a task and report an unexpected failure instead of losing it (fire and forget) */
template <class T>
void detach(const Future<T>& f, std::function<void(const std::string&)> onError = nullptr) {
  f.then([f, onError] {
    if (!f.error()) return;
    try {
      std::rethrow_exception(f.error());
    } catch (const std::exception& e) {
      if (onError)
        onError(e.what());
      else
        std::fprintf(stderr, "[task] %s\n", e.what());
    }
  });
}

}  // namespace aptuner::rt
