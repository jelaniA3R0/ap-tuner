#include "aptuner/log/dfreader.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstring>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <unordered_map>

namespace aptuner::log {

namespace {

constexpr std::uint8_t kHead1 = 0xa3, kHead2 = 0x95;
constexpr int kFmtType = 128, kFmtLen = 89;

enum class Kind { None, I8, U8, I16, U16, F16, I32, U32, F32, F64, Str, I16Arr, I16x100, U16x100, I32x100, U32x100, I32x1e7, I64, U64 };

struct FC {
  int size;
  Kind kind;
};

FC formatChar(char ch) {
  switch (ch) {
    case 'a': return {64, Kind::I16Arr};
    case 'b': return {1, Kind::I8};
    case 'B': return {1, Kind::U8};
    case 'g': return {2, Kind::F16};
    case 'h': return {2, Kind::I16};
    case 'H': return {2, Kind::U16};
    case 'i': return {4, Kind::I32};
    case 'I': return {4, Kind::U32};
    case 'f': return {4, Kind::F32};
    case 'd': return {8, Kind::F64};
    case 'n': return {4, Kind::Str};
    case 'N': return {16, Kind::Str};
    case 'Z': return {64, Kind::Str};
    case 'c': return {2, Kind::I16x100};
    case 'C': return {2, Kind::U16x100};
    case 'e': return {4, Kind::I32x100};
    case 'E': return {4, Kind::U32x100};
    case 'L': return {4, Kind::I32x1e7};
    case 'M': return {1, Kind::U8};
    case 'q': return {8, Kind::I64};
    case 'Q': return {8, Kind::U64};
    default: return {0, Kind::None};
  }
}

inline std::uint16_t u16(const std::uint8_t* p) { return static_cast<std::uint16_t>(p[0] | (p[1] << 8)); }
inline std::uint32_t u32(const std::uint8_t* p) {
  return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
         (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}
inline std::uint64_t u64(const std::uint8_t* p) {
  return static_cast<std::uint64_t>(u32(p)) | (static_cast<std::uint64_t>(u32(p + 4)) << 32);
}
inline float f32(const std::uint8_t* p) {
  const std::uint32_t v = u32(p);
  float f;
  std::memcpy(&f, &v, 4);
  return f;
}
inline double f64(const std::uint8_t* p) {
  const std::uint64_t v = u64(p);
  double d;
  std::memcpy(&d, &v, 8);
  return d;
}
inline std::uint64_t u64be(const std::uint8_t* p) {
  std::uint64_t v = 0;
  for (int k = 0; k < 8; k++) v = (v << 8) | p[k];
  return v;
}

}  // namespace

double f16(std::uint16_t u) {
  const double s = (u & 0x8000) ? -1 : 1;
  const int e = (u >> 10) & 0x1f, m = u & 0x3ff;
  if (e == 0) return s * m * std::pow(2.0, -24);
  if (e == 31) return m ? std::numeric_limits<double>::quiet_NaN() : s * std::numeric_limits<double>::infinity();
  return s * std::pow(2.0, e - 15) * (1 + m / 1024.0);
}

namespace {

double readField(const std::uint8_t* p, Kind k) {
  switch (k) {
    case Kind::I8: return static_cast<std::int8_t>(p[0]);
    case Kind::U8: return p[0];
    case Kind::I16: return static_cast<std::int16_t>(u16(p));
    case Kind::U16: return u16(p);
    case Kind::F16: return f16(u16(p));
    case Kind::I32: return static_cast<std::int32_t>(u32(p));
    case Kind::U32: return u32(p);
    case Kind::F32: return f32(p);
    case Kind::F64: return f64(p);
    case Kind::I16x100: return static_cast<std::int16_t>(u16(p)) / 100.0;
    case Kind::U16x100: return u16(p) / 100.0;
    case Kind::I32x100: return static_cast<std::int32_t>(u32(p)) / 100.0;
    case Kind::U32x100: return u32(p) / 100.0;
    case Kind::I32x1e7: return static_cast<std::int32_t>(u32(p)) / 1e7;
    case Kind::I64: return static_cast<double>(static_cast<std::int64_t>(u64(p)));
    case Kind::U64: return static_cast<double>(u64(p));
    default: return std::nan("");
  }
}

/** latin1 string up to the first NUL, returned as UTF-8 (JavaScript strings hold the same characters) */
std::string readStr(const std::uint8_t* p, std::size_t n) {
  std::string s;
  for (std::size_t i = 0; i < n && p[i]; i++) {
    const std::uint8_t c = p[i];
    if (c < 0x80) {
      s.push_back(static_cast<char>(c));
    } else {
      s.push_back(static_cast<char>(0xC0 | (c >> 6)));
      s.push_back(static_cast<char>(0x80 | (c & 0x3F)));
    }
  }
  return s;
}

std::vector<std::string> splitComma(const std::string& s) {
  std::vector<std::string> out;
  std::size_t a = 0;
  while (true) {
    const std::size_t b = s.find(',', a);
    out.push_back(s.substr(a, b == std::string::npos ? std::string::npos : b - a));
    if (b == std::string::npos) break;
    a = b + 1;
  }
  return out;
}

struct ColBuilder {
  std::vector<std::string> labels;
  std::vector<Kind> kinds;
  std::vector<int> sizes;
  std::vector<Vec> num;
  std::vector<std::vector<std::string>> str;
  std::vector<std::vector<std::array<std::int16_t, 32>>> arr;
  std::size_t n = 0;
};

}  // namespace

const Vec* MsgTable::col(const std::string& label) const {
  auto it = num.find(label);
  return it == num.end() ? nullptr : &it->second;
}

std::optional<double> Params::get(const std::string& name) const {
  auto it = index_.find(name);
  if (it == index_.end()) return std::nullopt;
  return items_[it->second].second;
}

double Params::getOr(const std::string& name, double d) const {
  auto v = get(name);
  return v ? *v : d;
}

double Params::truthyOr(const std::string& name, double d) const {
  auto v = get(name);
  return (v && *v != 0 && !std::isnan(*v)) ? *v : d;
}

int Params::intOf(const std::string& name) const {
  auto v = get(name);
  if (!v || !std::isfinite(*v)) return 0;
  // JS ToInt32: truncate, wrap modulo 2^32
  const double t = std::trunc(*v);
  const double m = std::fmod(t, 4294967296.0);
  long long w = static_cast<long long>(m < 0 ? m + 4294967296.0 : m);
  if (w >= 2147483648LL) w -= 4294967296LL;
  return static_cast<int>(w);
}

void Params::set(const std::string& name, double v) {
  auto it = index_.find(name);
  if (it != index_.end()) {
    items_[it->second].second = v;
    return;
  }
  index_[name] = items_.size();
  items_.emplace_back(name, v);
}

const MsgTable* LogData::msg(const std::string& name) const {
  auto it = msgs.find(name);
  return it == msgs.end() ? nullptr : &it->second;
}

MsgTable* LogData::msg(const std::string& name) {
  auto it = msgs.find(name);
  return it == msgs.end() ? nullptr : &it->second;
}

namespace {

/** latin1 bytes up to the first NUL (format strings are checked per byte, as the JavaScript reader sees them) */
std::string readRaw(const std::uint8_t* p, std::size_t n) {
  std::string s;
  for (std::size_t i = 0; i < n && p[i]; i++) s.push_back(static_cast<char>(p[i]));
  return s;
}

/** one latin1 character as UTF-8 */
std::string latin1Char(unsigned char c) {
  std::string s;
  if (c < 0x80) {
    s.push_back(static_cast<char>(c));
  } else {
    s.push_back(static_cast<char>(0xC0 | (c >> 6)));
    s.push_back(static_cast<char>(0x80 | (c & 0x3F)));
  }
  return s;
}

/** empty when an FMT definition is consistent, otherwise why it is not */
std::string fmtProblem(int type, int len, const std::string& name, const std::string& rawFormat, std::size_t nLabels) {
  bool nameOk = !name.empty() && name.size() <= 4;
  for (const char ch : name)
    if (!((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '_')) nameOk = false;
  if (!nameOk) return "invalid name";
  if (type == kFmtType) return name == "FMT" && len == kFmtLen ? std::string() : "redefines the FMT record";
  int size = 3;
  for (const char ch : rawFormat) {
    const FC fc = formatChar(ch);
    if (fc.kind == Kind::None) return "unknown format character '" + latin1Char(static_cast<unsigned char>(ch)) + "'";
    size += fc.size;
  }
  if (rawFormat.empty()) return "empty format";
  if (size != len) return "length " + std::to_string(len) + " does not match its format (" + std::to_string(size) + " bytes)";
  if (nLabels != rawFormat.size()) return std::to_string(nLabels) + " labels for " + std::to_string(rawFormat.size()) + " fields";
  return {};
}

}  // namespace

FileFormat sniffFormat(const std::uint8_t* buf, std::size_t N) {
  if (!N) return {"empty", false, "The file is empty."};
  // a DataFlash log starts with its FMT records; allow a little junk in front
  const std::size_t lim = std::min<std::size_t>(N, 4096);
  for (std::size_t i = 0; i + 3 <= lim; i++)
    if (buf[i] == kHead1 && buf[i + 1] == kHead2 && buf[i + 2] == kFmtType) return {"dataflash", true, ""};
  // Mission Planner telemetry log: 8-byte time stamp (microseconds since 1970, big-endian) + a MAVLink frame
  if (N >= 10 && (buf[8] == 0xfe || buf[8] == 0xfd)) {
    const double us = static_cast<double>(u64be(buf));
    if (us > 9.4e14 && us < 4.2e15)
      return {"tlog", false,
              "This is a MAVLink telemetry log (.tlog), not a DataFlash log (.bin). Telemetry logs do not hold the "
              "rate-loop data at a usable rate. Use the .bin log from the flight controller (Mission Planner: DataFlash "
              "Logs > Download DataFlash Log Via Mavlink, or copy it from the SD card)."};
  }
  const std::size_t m = std::min<std::size_t>(N, 512);
  std::size_t printable = 0;
  for (std::size_t i = 0; i < m; i++) {
    const std::uint8_t c = buf[i];
    if (c == 9 || c == 10 || c == 13 || (c >= 32 && c < 127)) printable++;
  }
  if (printable == m) {
    // /^\s*FMT\s*,/
    auto space = [](std::uint8_t c) { return c == 9 || c == 10 || c == 13 || c == 32; };
    std::size_t k = 0;
    while (k < m && space(buf[k])) k++;
    bool fmt = k + 3 <= m && buf[k] == 'F' && buf[k + 1] == 'M' && buf[k + 2] == 'T';
    if (fmt) {
      k += 3;
      while (k < m && space(buf[k])) k++;
      fmt = k < m && buf[k] == ',';
    }
    if (fmt)
      return {"dataflash-text", false,
              "This is a DataFlash log converted to text (.log). AP Tuner reads the binary .bin log: use the .bin file "
              "the text was made from."};
    return {"text", false, "This is a text file (for example CSV), not an ArduPilot DataFlash log (.bin)."};
  }
  return {"unknown", false,
          "This is not an ArduPilot DataFlash log (.bin): it does not start with the log format records. If it is a "
          ".bin log, the start of the file is missing."};
}

LogData parseBin(const std::uint8_t* buf, std::size_t N, const std::set<std::string>& want) {
  LogData out;
  Fmt fmtFmt;
  fmtFmt.type = kFmtType;
  fmtFmt.len = kFmtLen;
  fmtFmt.name = "FMT";
  fmtFmt.format = "BBnNZ";
  fmtFmt.labels = {"Type", "Length", "Name", "Format", "Columns"};
  std::array<Fmt*, 256> fmts{};
  std::array<std::unique_ptr<Fmt>, 256> fmtStore{};
  fmtStore[kFmtType] = std::make_unique<Fmt>(fmtFmt);
  fmts[kFmtType] = fmtStore[kFmtType].get();
  out.fmts[kFmtType] = fmtFmt;
  Integrity& integ = out.integrity;
  integ.format = sniffFormat(buf, N);
  integ.bytes = N;
  if (!integ.format.supported) return out;
  // zero / 0xFF bytes at the end of the file (unused flash or SD blocks) are padding, not data; the last record
  // itself may end with such bytes, so records are still read up to the end of the file
  std::size_t padStart = N;
  while (padStart > 0 && (buf[padStart - 1] == 0 || buf[padStart - 1] == 0xff)) padStart--;
  std::map<std::string, int> nameOwner{{"FMT", kFmtType}};   // message name -> type that defined it
  std::unordered_map<std::string, double> lastParam;           // latest logged value of each parameter
  std::array<ColBuilder*, 256> colOf{};
  std::array<int, 256> msgOff{};
  msgOff.fill(-1);
  std::map<std::string, ColBuilder> cols;
  // a record header at j whose type is defined (or j is at the end of the data)
  auto headerAt = [&](std::size_t j) {
    return j + 3 > N || j >= padStart || (buf[j] == kHead1 && buf[j + 1] == kHead2 && fmts[buf[j + 2]] != nullptr);
  };
  std::size_t i = 0;
  bool skipping = false, truncated = false;
  while (i < padStart) {
    if (i + 3 > N) {
      integ.truncatedBytes = N - i;
      truncated = true;
      break;
    }
    const int t = buf[i + 2];
    Fmt* fm = buf[i] == kHead1 && buf[i + 1] == kHead2 ? fmts[static_cast<std::size_t>(t)] : nullptr;
    if (!fm) {
      i++;
      integ.skippedBytes++;
      skipping = true;
      continue;
    }
    if (i + static_cast<std::size_t>(fm->len) > N) {
      integ.truncatedBytes = N - i;
      truncated = true;
      break;
    }
    // the next record must start right after this one: a corrupted header or a false sync inside data fails this
    // (an FMT record may be followed by the first record of the type it defines)
    const std::size_t next = i + static_cast<std::size_t>(fm->len);
    const bool chained = headerAt(next) || (t == kFmtType && next + 3 <= N && buf[next] == kHead1 &&
                                            buf[next + 1] == kHead2 && buf[next + 2] == buf[i + 3]);
    if (!chained) {
      i++;
      integ.skippedBytes++;
      skipping = true;
      continue;
    }
    if (skipping) {
      integ.resyncs++;
      skipping = false;
    }
    const std::uint8_t* body = buf + i + 3;
    if (t == kFmtType) {
      const int type = body[0], len = body[1];
      const std::string name = readStr(body + 2, 4), format = readStr(body + 6, 16), rawFormat = readRaw(body + 6, 16);
      const std::vector<std::string> labels = splitComma(readStr(body + 22, 64));
      const Fmt* old = fmts[static_cast<std::size_t>(type)];
      if (old) {
        // the same definition again is harmless; a different one would change how existing records are read
        if (type != kFmtType && (old->len != len || old->name != name || old->format != format || old->labels != labels))
          integ.fmtConflicts++;
      } else {
        std::string why = fmtProblem(type, len, name, rawFormat, labels.size());
        if (why.empty() && nameOwner.count(name)) why = "name already used by type " + std::to_string(nameOwner[name]);
        if (!why.empty()) {
          integ.badFmtCount++;
          if (integ.badFmts.size() < 20) integ.badFmts.push_back({type, name, why});
        } else {
          auto f = std::make_unique<Fmt>();
          f->type = type;
          f->len = len;
          f->name = name;
          f->format = format;
          f->labels = labels;
          fmtStore[static_cast<std::size_t>(type)] = std::move(f);
          fmts[static_cast<std::size_t>(type)] = fmtStore[static_cast<std::size_t>(type)].get();
          out.fmts[type] = *fmts[static_cast<std::size_t>(type)];
          nameOwner[name] = type;
        }
      }
      out.counts["FMT"]++;
      i += static_cast<std::size_t>(fm->len);
      continue;
    }
    out.counts[fm->name]++;
    if (fm->name == "PARM" && fm->format.rfind("QNf", 0) == 0) {
      // Q N f (TimeUS, Name, Value) [+ Default]; first value = boot value, every later new value is a change
      const std::string name = readStr(body + 8, 16);
      const double val = f32(body + 24);
      if (!out.params.has(name)) {
        out.params.set(name, val);
        lastParam[name] = val;
      } else {
        const double last = lastParam[name];
        if (last != val && !(std::isnan(val) && std::isnan(last))) {
          out.changes.push_back({static_cast<double>(u64(body)) / 1e6, name, val});
          lastParam[name] = val;
        }
      }
      i += static_cast<std::size_t>(fm->len);
      continue;
    }
    if (fm->name == "MSG") {
      int& off = msgOff[static_cast<std::size_t>(t)];
      if (off < 0) {
        off = 8;
        int o = 0;
        for (const char ch : fm->format) {
          if (ch == 'Z') {
            off = o;
            break;
          }
          o += formatChar(ch).size;
        }
      }
      out.text.push_back({static_cast<double>(u64(body)) / 1e6, readStr(body + off, 64)});
      i += static_cast<std::size_t>(fm->len);
      continue;
    }
    if (!want.empty() && !want.count(fm->name)) {
      i += static_cast<std::size_t>(fm->len);
      continue;
    }
    ColBuilder* c = colOf[static_cast<std::size_t>(t)];
    if (!c) {
      auto it = cols.find(fm->name);
      if (it == cols.end()) {
        ColBuilder nb;
        nb.labels = fm->labels;
        for (const char ch : fm->format) {
          const FC fc = formatChar(ch);
          nb.sizes.push_back(fc.size);
          nb.kinds.push_back(fc.kind);
        }
        const std::size_t k = nb.kinds.size();
        nb.num.resize(k);
        nb.str.resize(k);
        nb.arr.resize(k);
        it = cols.emplace(fm->name, std::move(nb)).first;
      }
      c = &it->second;
      colOf[static_cast<std::size_t>(t)] = c;
    }
    const std::uint8_t* p = body;
    for (std::size_t k = 0; k < c->kinds.size(); k++) {
      const Kind kind = c->kinds[k];
      if (kind == Kind::Str) {
        c->str[k].push_back(readStr(p, static_cast<std::size_t>(c->sizes[k])));
      } else if (kind == Kind::I16Arr) {
        std::array<std::int16_t, 32> a{};
        for (int j = 0; j < 32; j++) a[static_cast<std::size_t>(j)] = static_cast<std::int16_t>(u16(p + 2 * j));
        c->arr[k].push_back(a);
      } else if (kind != Kind::None) {
        c->num[k].push_back(readField(p, kind));
      }
      p += c->sizes[k];
    }
    c->n++;
    i += static_cast<std::size_t>(fm->len);
  }
  if (!truncated) integ.paddingBytes = N > i ? N - i : 0;
  for (auto& [name, c] : cols) {
    MsgTable m;
    m.n = c.n;
    m.labels = c.labels;
    for (std::size_t k = 0; k < c.labels.size() && k < c.kinds.size(); k++) {
      const Kind kind = c.kinds[k];
      if (kind == Kind::Str)
        m.str[c.labels[k]] = std::move(c.str[k]);
      else if (kind == Kind::I16Arr)
        m.i16[c.labels[k]] = std::move(c.arr[k]);
      else if (kind != Kind::None)
        m.num[c.labels[k]] = std::move(c.num[k]);
    }
    out.msgs[name] = std::move(m);
  }
  return out;
}

std::vector<std::uint8_t> readFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  f.seekg(0, std::ios::end);
  const std::streamoff size = f.tellg();
  f.seekg(0, std::ios::beg);
  std::vector<std::uint8_t> data(static_cast<std::size_t>(size));
  if (size > 0) f.read(reinterpret_cast<char*>(data.data()), size);
  if (!f) throw std::runtime_error("cannot read " + path);
  return data;
}

LogData parseBinFile(const std::string& path, const std::set<std::string>& want) {
  const std::vector<std::uint8_t> data = readFile(path);
  return parseBin(data.data(), data.size(), want);
}

std::map<int, MsgTable> splitByInstance(const MsgTable& m, const std::string& field) {
  std::map<int, MsgTable> out;
  const Vec* inst = m.col(field);
  if (!inst) {
    out[0] = m;
    return out;
  }
  std::map<int, std::vector<std::size_t>> idx;
  for (std::size_t k = 0; k < m.n && k < inst->size(); k++) idx[static_cast<int>((*inst)[k])].push_back(k);
  for (const auto& [v, ks] : idx) {
    MsgTable o;
    o.n = ks.size();
    o.labels = m.labels;
    for (const auto& [lab, arr] : m.num) {
      Vec a(ks.size());
      for (std::size_t j = 0; j < ks.size(); j++) a[j] = arr[ks[j]];
      o.num[lab] = std::move(a);
    }
    for (const auto& [lab, arr] : m.str) {
      std::vector<std::string> a;
      for (std::size_t j : ks) a.push_back(arr[j]);
      o.str[lab] = std::move(a);
    }
    for (const auto& [lab, arr] : m.i16) {
      std::vector<std::array<std::int16_t, 32>> a;
      for (std::size_t j : ks) a.push_back(arr[j]);
      o.i16[lab] = std::move(a);
    }
    out[v] = std::move(o);
  }
  return out;
}

std::map<int, Fmt> readFormats(const std::uint8_t* buf, std::size_t size) {
  std::map<int, Fmt> fmts;
  Fmt f;
  f.type = kFmtType;
  f.len = kFmtLen;
  f.name = "FMT";
  f.labels = {"Type", "Length", "Name", "Format", "Columns"};
  fmts[kFmtType] = f;
  for (std::size_t i = 0; i + kFmtLen <= size; i++) {
    if (buf[i] != kHead1 || buf[i + 1] != kHead2 || buf[i + 2] != kFmtType) continue;
    const std::uint8_t* b = buf + i + 3;
    const int type = b[0];
    if (type != kFmtType) {
      Fmt x;
      x.type = type;
      x.len = b[1];
      x.name = readStr(b + 2, 4);
      x.format = readStr(b + 6, 16);
      x.labels = splitComma(readStr(b + 22, 64));
      fmts[type] = std::move(x);
    }
    i += kFmtLen - 1;
  }
  return fmts;
}

namespace {

const Fmt* fmtAt(const std::map<int, Fmt>& fmts, int t) {
  auto it = fmts.find(t);
  return it == fmts.end() ? nullptr : &it->second;
}

/** a message starts at i and the next messages chain after it (guards against false syncs inside data) */
bool chainAt(const std::uint8_t* buf, std::size_t size, std::size_t i, const std::map<int, Fmt>& fmts, int depth = 3) {
  std::size_t j = i;
  for (int k = 0; k < depth; k++) {
    if (j + 3 > size) return k > 0;
    if (buf[j] != kHead1 || buf[j + 1] != kHead2) return false;
    const Fmt* fm = fmtAt(fmts, buf[j + 2]);
    if (!fm || !fm->len) return false;
    j += static_cast<std::size_t>(fm->len);
  }
  return true;
}

bool isTimeUsQ(const Fmt& fm) { return !fm.labels.empty() && fm.labels[0] == "TimeUS" && !fm.format.empty() && fm.format[0] == 'Q'; }

}  // namespace

long long firstSync(const std::uint8_t* buf, std::size_t size, const std::map<int, Fmt>& fmts, std::size_t from) {
  for (std::size_t i = from; i + 3 <= size; i++)
    if (chainAt(buf, size, i, fmts)) return static_cast<long long>(i);
  return -1;
}

std::size_t completeLength(const std::uint8_t* buf, std::size_t size, const std::map<int, Fmt>& fmts) {
  long long s = firstSync(buf, size, fmts, 0);
  if (s < 0) return 0;
  std::size_t i = static_cast<std::size_t>(s), last = i;
  while (i + 3 <= size) {
    const Fmt* fm = (buf[i] == kHead1 && buf[i + 1] == kHead2) ? fmtAt(fmts, buf[i + 2]) : nullptr;
    if (!fm || !fm->len) {
      const long long nx = firstSync(buf, size, fmts, i + 1);
      if (nx < 0) break;
      i = static_cast<std::size_t>(nx);
      continue;
    }
    if (i + static_cast<std::size_t>(fm->len) > size) break;
    i += static_cast<std::size_t>(fm->len);
    last = i;
  }
  return last;
}

std::optional<double> firstTime(const std::uint8_t* buf, std::size_t size, const std::map<int, Fmt>& fmts) {
  long long s = firstSync(buf, size, fmts, 0);
  while (s >= 0 && static_cast<std::size_t>(s) + 3 <= size) {
    const std::size_t i = static_cast<std::size_t>(s);
    const Fmt* fm = fmtAt(fmts, buf[i + 2]);
    if (fm && isTimeUsQ(*fm) && i + 11 <= size) return static_cast<double>(u64(buf + i + 3)) / 1e6;
    if (!fm || !fm->len) {
      s = firstSync(buf, size, fmts, i + 1);
      continue;
    }
    s = static_cast<long long>(i) + fm->len;
  }
  return std::nullopt;
}

std::optional<double> lastTime(const std::uint8_t* buf, std::size_t size, const std::map<int, Fmt>& fmts) {
  long long s = firstSync(buf, size, fmts, 0);
  std::optional<double> t;
  while (s >= 0 && static_cast<std::size_t>(s) + 3 <= size) {
    const std::size_t i = static_cast<std::size_t>(s);
    const Fmt* fm = fmtAt(fmts, buf[i + 2]);
    if (!fm || !fm->len) {
      s = firstSync(buf, size, fmts, i + 1);
      continue;
    }
    if (i + static_cast<std::size_t>(fm->len) > size) break;
    if (isTimeUsQ(*fm)) {
      const double v = static_cast<double>(u64(buf + i + 3)) / 1e6;
      if (v > 0 && v < 1e7) t = v;
    }
    s = static_cast<long long>(i) + fm->len;
  }
  return t;
}

}  // namespace aptuner::log
