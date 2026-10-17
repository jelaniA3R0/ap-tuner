// Minimal EventEmitter: named events with JSON payloads, handlers called on the loop thread in the order they
// were added. Removing a handler from inside an emit is safe (the list is copied first, as in Node).
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "aptuner/core/json.hpp"

namespace aptuner::rt {

class Emitter {
 public:
  using Handler = std::function<void(const core::Json&)>;

  std::uint64_t on(const std::string& event, Handler h) {
    const std::uint64_t id = next_++;
    handlers_[event].emplace_back(id, std::move(h));
    return id;
  }
  /** remove one handler (by the id on() returned) */
  void off(std::uint64_t id) {
    for (auto& [ev, list] : handlers_)
      for (auto it = list.begin(); it != list.end(); ++it)
        if (it->first == id) {
          list.erase(it);
          return;
        }
  }
  void emit(const std::string& event, const core::Json& payload) {
    auto it = handlers_.find(event);
    if (it == handlers_.end() || it->second.empty()) return;
    const auto copy = it->second;
    for (const auto& [id, h] : copy)
      if (alive(event, id)) h(payload);
  }
  bool has(const std::string& event) const {
    auto it = handlers_.find(event);
    return it != handlers_.end() && !it->second.empty();
  }

 private:
  bool alive(const std::string& event, std::uint64_t id) const {
    auto it = handlers_.find(event);
    if (it == handlers_.end()) return false;
    for (const auto& p : it->second)
      if (p.first == id) return true;
    return false;
  }
  std::map<std::string, std::vector<std::pair<std::uint64_t, Handler>>> handlers_;
  std::uint64_t next_ = 1;
};

/** removes its handler when it goes out of scope */
class Subscription {
 public:
  Subscription() = default;
  Subscription(Emitter& e, std::uint64_t id) : e_(&e), id_(id) {}
  Subscription(Subscription&& o) noexcept : e_(o.e_), id_(o.id_) { o.e_ = nullptr; }
  Subscription& operator=(Subscription&& o) noexcept {
    reset();
    e_ = o.e_;
    id_ = o.id_;
    o.e_ = nullptr;
    return *this;
  }
  Subscription(const Subscription&) = delete;
  Subscription& operator=(const Subscription&) = delete;
  ~Subscription() { reset(); }
  void reset() {
    if (e_) e_->off(id_);
    e_ = nullptr;
  }

 private:
  Emitter* e_ = nullptr;
  std::uint64_t id_ = 0;
};

}  // namespace aptuner::rt
