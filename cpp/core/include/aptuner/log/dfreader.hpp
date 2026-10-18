// ArduPilot DataFlash (.bin) log reader, self-describing via FMT messages. Port of src/log/dfreader.js:
// one column (std::vector<double>) per message field, parameters (boot values + in-flight changes), text
// messages, and helpers for partial logs (a head plus a slice from the middle, e.g. downloaded over MAVLink).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace aptuner::log {

using Vec = std::vector<double>;

struct Fmt {
  int type = 0;
  int len = 0;
  std::string name, format;
  std::vector<std::string> labels;
};

/** one message type as columns */
struct MsgTable {
  std::size_t n = 0;
  std::vector<std::string> labels;                                  // field order as in the FMT
  std::unordered_map<std::string, Vec> num;                         // numeric fields
  std::unordered_map<std::string, std::vector<std::string>> str;    // string fields
  std::unordered_map<std::string, std::vector<std::array<std::int16_t, 32>>> i16;   // int16[32] arrays (ISBD)

  /** numeric column or nullptr */
  const Vec* col(const std::string& label) const;
  bool has(const std::string& label) const { return col(label) != nullptr; }
};

/** ordered parameter set with JavaScript-like access */
class Params {
 public:
  bool has(const std::string& name) const { return index_.count(name) != 0; }
  std::optional<double> get(const std::string& name) const;
  /** params[name] ?? d */
  double getOr(const std::string& name, double d) const;
  /** params[name] || d  (undefined, 0 and NaN -> d) */
  double truthyOr(const std::string& name, double d) const;
  /** params[name] | 0 */
  int intOf(const std::string& name) const;
  void set(const std::string& name, double v);   // insert or replace
  const std::vector<std::pair<std::string, double>>& items() const { return items_; }
  std::size_t size() const { return items_.size(); }

 private:
  std::vector<std::pair<std::string, double>> items_;
  std::unordered_map<std::string, std::size_t> index_;
};

struct ParamChange {
  double t = 0;
  std::string name;
  double val = 0;
};

struct TextMsg {
  double t = 0;
  std::string msg;
};

/** what kind of file a buffer holds (sniffFormat) */
struct FileFormat {
  std::string kind;       // "dataflash" | "tlog" | "dataflash-text" | "text" | "empty" | "unknown"
  bool supported = false;
  std::string message;    // why it cannot be analysed (empty when supported)
};
FileFormat sniffFormat(const std::uint8_t* buf, std::size_t size);

struct BadFmt {
  int type = 0;
  std::string name, reason;
};

/** how well the file could be read */
struct Integrity {
  FileFormat format;
  std::size_t bytes = 0;
  std::size_t skippedBytes = 0;    // bytes that were not part of a verified record
  std::size_t resyncs = 0;         // times reading picked up again after skipped bytes
  std::size_t truncatedBytes = 0;  // an incomplete record at the end
  std::size_t paddingBytes = 0;    // zero / 0xFF bytes after the last record
  std::vector<BadFmt> badFmts;     // the first 20 invalid FMT definitions
  std::size_t badFmtCount = 0;
  std::size_t fmtConflicts = 0;    // a type defined again with a different layout (the first one is kept)
};

struct LogData {
  std::map<int, Fmt> fmts;
  std::map<std::string, std::size_t> counts;
  std::map<std::string, MsgTable> msgs;
  Params params;                       // first value of each parameter (boot value)
  std::vector<ParamChange> changes;    // later in-flight changes (every new value, also a change back)
  std::vector<TextMsg> text;
  Integrity integrity;

  const MsgTable* msg(const std::string& name) const;
  MsgTable* msg(const std::string& name);
};

/** IEEE 754 half precision (DataFlash format character 'g') */
double f16(std::uint16_t u);

/**
 * parse a whole .bin image; want = message names to keep as columns (empty = all). Files that are not
 * DataFlash logs are not parsed (integrity.format says why); FMT definitions are validated; every record must
 * be followed by the next record header (or the end of the file).
 */
LogData parseBin(const std::uint8_t* buf, std::size_t size, const std::set<std::string>& want = {});
LogData parseBinFile(const std::string& path, const std::set<std::string>& want = {});   // throws on read errors
std::vector<std::uint8_t> readFile(const std::string& path);

/** split a table by an instance field (e.g. IMU.I) -> {instance: table} (no field: {0: whole table}) */
std::map<int, MsgTable> splitByInstance(const MsgTable& m, const std::string& field = "I");

// ---- partial logs ----
/** FMT table from a log head */
std::map<int, Fmt> readFormats(const std::uint8_t* buf, std::size_t size);
/** offset of the first well-chained message at or after `from` (-1 if none) */
long long firstSync(const std::uint8_t* buf, std::size_t size, const std::map<int, Fmt>& fmts, std::size_t from = 0);
/** byte length of the complete, chained messages from the start of buf */
std::size_t completeLength(const std::uint8_t* buf, std::size_t size, const std::map<int, Fmt>& fmts);
/** first / last TimeUS (seconds) in a chunk */
std::optional<double> firstTime(const std::uint8_t* buf, std::size_t size, const std::map<int, Fmt>& fmts);
std::optional<double> lastTime(const std::uint8_t* buf, std::size_t size, const std::map<int, Fmt>& fmts);

}  // namespace aptuner::log
