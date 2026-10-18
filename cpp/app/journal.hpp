// The write journal (2026-10-17 audit F-39): one append-only JSON-lines file per app run in sessions/journal/, one line
// per event, written to disk before the app goes on: identities, proposals, approvals, every write (requested, sent,
// type, value before, echo, read-back, status, error, time), rollbacks, reboots and exports. Nothing in it is ever
// rewritten. A forward write that cannot be journaled is not made (a correction still is). Port of src/engine/journal.js.
#pragma once

#include <deque>
#include <string>

#include "aptuner/core/json.hpp"

namespace aptuner::app {

using core::Json;

class Journal {
 public:
  explicit Journal(std::string dir) : dir_(std::move(dir)) {}
  /** append one event {t, kind, ...data}; false when it could not be written (the error stays: the journal has a gap) */
  bool append(const std::string& kind, const Json& data = Json::object());
  /** false once an event could not be written: forward writes stop (restart the app after freeing disk space) */
  bool ok() const { return lost_ == 0; }
  const std::string& error() const { return error_; }
  const std::string& file() const { return file_; }
  /** {file, count, lost, error} */
  Json status() const;
  const std::deque<Json>& recent() const { return recent_; }

 private:
  std::string dir_, file_, error_;
  int count_ = 0, lost_ = 0;
  std::deque<Json> recent_;
};

}  // namespace aptuner::app
