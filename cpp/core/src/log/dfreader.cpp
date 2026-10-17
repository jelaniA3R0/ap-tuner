#include "aptuner/log/dfreader.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <memory>
#include <stdexcept>

namespace aptuner::log {

namespace {

constexpr std::uint8_t kHead1 = 0xa3, kHead2 = 0x95;
constexpr int kFmtType = 128, kFmtLen = 89;

enum class Kind { None, I8, U8, I16, U16, I32, U32, F32, F64, Str, I16Arr, I16x100, U16x100, I32x100, U32x100, I32x1e7, I64, U64 };

struct FC {
  int size;
  Kind kind;
};

FC formatChar(char ch) {
  switch (ch) {
    case 'a': return {64, Kind::I16Arr};
    case 'b': return {1, Kind::I8};
    case 'B': return {1, Kind::U8};
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

double readField(const std::uint8_t* p, Kind k) {
  switch (k) {
    case Kind::I8: return static_cast<std::int8_t>(p[0]);
    case Kind::U8: return p[0];
    case Kind::I16: return static_cast<std::int16_t>(u16(p));
    case Kind::U16: return u16(p);
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
  std::array<ColBuilder*, 256> colOf{};
  std::array<int, 256> msgOff{};
  msgOff.fill(-1);
  std::map<std::string, ColBuilder> cols;
  std::size_t i = 0;
  while (i + 3 <= N) {
    if (buf[i] != kHead1 || buf[i + 1] != kHead2) {
      i++;
      continue;
    }
    const int t = buf[i + 2];
    Fmt* fm = fmts[static_cast<std::size_t>(t)];
    if (!fm) {
      i++;
      continue;
    }
    if (i + static_cast<std::size_t>(fm->len) > N) break;
    const std::uint8_t* body = buf + i + 3;
    if (t == kFmtType) {
      const int type = body[0], len = body[1];
      if (!fmts[static_cast<std::size_t>(type)] || type != kFmtType) {
        auto f = std::make_unique<Fmt>();
        f->type = type;
        f->len = len;
        f->name = readStr(body + 2, 4);
        f->format = readStr(body + 6, 16);
        f->labels = splitComma(readStr(body + 22, 64));
        fmtStore[static_cast<std::size_t>(type)] = std::move(f);
        fmts[static_cast<std::size_t>(type)] = fmtStore[static_cast<std::size_t>(type)].get();
        colOf[static_cast<std::size_t>(type)] = nullptr;
        msgOff[static_cast<std::size_t>(type)] = -1;
        out.fmts[type] = *fmts[static_cast<std::size_t>(type)];
      }
      out.counts["FMT"]++;
      i += static_cast<std::size_t>(fm->len);
      continue;
    }
    out.counts[fm->name]++;
    if (fm->name == "PARM") {
      // Q N f (TimeUS, Name, Value) [+ Default]
      const std::string name = readStr(body + 8, 16);
      const double val = f32(body + 24);
      if (!out.params.has(name)) out.params.set(name, val);   // first value = boot value
      const double first = *out.params.get(name);
      if (first != val) out.changes.push_back({static_cast<double>(u64(body)) / 1e6, name, val});
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
