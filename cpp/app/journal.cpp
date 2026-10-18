#include "journal.hpp"

#include <filesystem>
#include <fstream>
#include <system_error>

#include "aptuner/rt/loop.hpp"
#include "common.hpp"

namespace aptuner::app {

bool Journal::append(const std::string& kind, const Json& data) {
  Json e = Json::obj({{"t", rt::Loop::now()}, {"kind", kind}});
  if (data.isObject())
    for (const auto& [k, v] : data.items()) e.set(k, v);
  if (file_.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(fsPath(dir_), ec);
    if (ec) {
      error_ = ec.message();
      lost_++;
      return false;
    }
    file_ = pathStr(fsPath(dir_) / (isoStampForFile() + ".jsonl"));
  }
  std::ofstream f(fsPath(file_), std::ios::binary | std::ios::app);
  if (f) {
    f << e.dump() << '\n';
    f.flush();
  }
  if (!f) {
    error_ = "the journal file " + file_ + " could not be written";
    lost_++;
    return false;
  }
  count_++;
  recent_.push_back(e);
  if (recent_.size() > 300) recent_.pop_front();
  return true;
}

Json Journal::status() const {
  return Json::obj({{"file", file_.empty() ? Json() : Json(file_)}, {"count", count_}, {"lost", lost_}, {"error", error_.empty() ? Json() : Json(error_)}});
}

}  // namespace aptuner::app
