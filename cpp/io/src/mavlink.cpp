#include "aptuner/mavlink/mavlink.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <unordered_map>

namespace aptuner::mavlink {

namespace {

struct EnumEntry {
  long long value;
  const char* name;
};

#include "mavlink_defs.inc"

const std::unordered_map<int, const MsgDef*>& byId() {
  static const std::unordered_map<int, const MsgDef*> m = [] {
    std::unordered_map<int, const MsgDef*> r;
    for (const auto& d : kMessages) r[d.id] = &d;
    return r;
  }();
  return m;
}

const std::unordered_map<std::string, const MsgDef*>& byName() {
  static const std::unordered_map<std::string, const MsgDef*> m = [] {
    std::unordered_map<std::string, const MsgDef*> r;
    for (const auto& d : kMessages) r[d.name] = &d;
    return r;
  }();
  return m;
}

int sizeOf(FieldType t) {
  switch (t) {
    case kChar:
    case kI8:
    case kU8: return 1;
    case kI16:
    case kU16: return 2;
    case kI32:
    case kU32:
    case kF32: return 4;
    case kF64:
    case kI64:
    case kU64: return 8;
  }
  return 1;
}

std::uint64_t le(const std::uint8_t* p, int n) {
  std::uint64_t v = 0;
  for (int i = n - 1; i >= 0; i--) v = (v << 8) | p[i];
  return v;
}

double readOne(const std::uint8_t* p, FieldType t) {
  switch (t) {
    case kChar:
    case kU8: return p[0];
    case kI8: return static_cast<std::int8_t>(p[0]);
    case kI16: return static_cast<std::int16_t>(le(p, 2));
    case kU16: return static_cast<double>(le(p, 2));
    case kI32: return static_cast<std::int32_t>(le(p, 4));
    case kU32: return static_cast<double>(le(p, 4));
    case kF32: {
      const std::uint32_t v = static_cast<std::uint32_t>(le(p, 4));
      float f;
      std::memcpy(&f, &v, 4);
      return f;
    }
    case kF64: {
      const std::uint64_t v = le(p, 8);
      double d;
      std::memcpy(&d, &v, 8);
      return d;
    }
    case kI64: return static_cast<double>(static_cast<std::int64_t>(le(p, 8)));
    case kU64: return static_cast<double>(le(p, 8));
  }
  return 0;
}

void putLe(std::uint8_t* p, std::uint64_t v, int n) {
  for (int i = 0; i < n; i++) p[i] = static_cast<std::uint8_t>(v >> (8 * i));
}

/** JavaScript-like integer conversion: truncate, wrap to the field width */
std::uint64_t toBits(double v) {
  if (!std::isfinite(v)) return 0;
  const double t = std::trunc(v);
  if (t >= 0) return t >= 18446744073709551615.0 ? ~0ULL : static_cast<std::uint64_t>(t);
  return static_cast<std::uint64_t>(static_cast<std::int64_t>(t <= -9223372036854775808.0 ? -9223372036854775807.0 : t));
}

void writeOne(std::uint8_t* p, FieldType t, double v) {
  switch (t) {
    case kChar:
    case kI8:
    case kU8: p[0] = static_cast<std::uint8_t>(toBits(v) & 0xff); break;
    case kI16:
    case kU16: putLe(p, toBits(v), 2); break;
    case kI32:
    case kU32: putLe(p, toBits(v), 4); break;
    case kF32: {
      const float f = static_cast<float>(v);
      std::uint32_t b;
      std::memcpy(&b, &f, 4);
      putLe(p, b, 4);
      break;
    }
    case kF64: {
      std::uint64_t b;
      std::memcpy(&b, &v, 8);
      putLe(p, b, 8);
      break;
    }
    case kI64:
    case kU64: putLe(p, toBits(v), 8); break;
  }
}

/** latin1 bytes -> UTF-8 (JavaScript decodes char arrays as latin1) */
std::string latin1(const std::uint8_t* p, std::size_t n) {
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

/** UTF-8 -> latin1 bytes (characters above 0xFF become '?') */
std::vector<std::uint8_t> toLatin1(const std::string& s) {
  std::vector<std::uint8_t> out;
  for (std::size_t i = 0; i < s.size();) {
    const auto c = static_cast<std::uint8_t>(s[i]);
    if (c < 0x80) {
      out.push_back(c);
      i++;
    } else if ((c & 0xE0) == 0xC0 && i + 1 < s.size()) {
      const unsigned cp = ((c & 0x1Fu) << 6) | (static_cast<std::uint8_t>(s[i + 1]) & 0x3Fu);
      out.push_back(cp <= 0xFF ? static_cast<std::uint8_t>(cp) : '?');
      i += 2;
    } else {
      out.push_back('?');
      i++;
      while (i < s.size() && (static_cast<std::uint8_t>(s[i]) & 0xC0) == 0x80) i++;
    }
  }
  return out;
}

}  // namespace

const MsgDef* msgById(int id) {
  auto it = byId().find(id);
  return it == byId().end() ? nullptr : it->second;
}

const MsgDef* msgByName(const std::string& name) {
  auto it = byName().find(name);
  return it == byName().end() ? nullptr : it->second;
}

const char* enumName(const std::string& e, long long value) {
  auto find = [&](const EnumEntry* arr, std::size_t n) -> const char* {
    for (std::size_t i = 0; i < n; i++)
      if (arr[i].value == value) return arr[i].name;
    return nullptr;
  };
  if (e == "MAV_TYPE") return find(kEnum_MAV_TYPE, sizeof kEnum_MAV_TYPE / sizeof kEnum_MAV_TYPE[0]);
  if (e == "COPTER_MODE") return find(kEnum_COPTER_MODE, sizeof kEnum_COPTER_MODE / sizeof kEnum_COPTER_MODE[0]);
  if (e == "PLANE_MODE") return find(kEnum_PLANE_MODE, sizeof kEnum_PLANE_MODE / sizeof kEnum_PLANE_MODE[0]);
  return nullptr;
}

std::uint16_t crcX25(std::uint16_t crc, std::uint8_t byte) {
  unsigned tmp = byte ^ (crc & 0xff);
  tmp = (tmp ^ (tmp << 4)) & 0xff;
  return static_cast<std::uint16_t>(((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff);
}

core::Json decodePayload(const MsgDef& def, const std::uint8_t* payload, std::size_t len) {
  // v2 payloads may be truncated (trailing zeros removed): zero-extend
  std::vector<std::uint8_t> p(static_cast<std::size_t>(def.length), 0);
  std::memcpy(p.data(), payload, std::min(len, p.size()));
  core::Json out = core::Json::object();
  for (int k = 0; k < def.nFields; k++) {
    const FieldDef& f = def.fields[k];
    const std::uint8_t* at = p.data() + f.offset;
    if (f.len) {
      if (f.type == kChar) {
        out.items().emplace_back(f.name, core::Json(latin1(at, static_cast<std::size_t>(f.len))));
      } else {
        core::Json::Array a;
        a.reserve(static_cast<std::size_t>(f.len));
        const int sz = sizeOf(f.type);
        for (int i = 0; i < f.len; i++) a.emplace_back(readOne(at + i * sz, f.type));
        out.items().emplace_back(f.name, core::Json(std::move(a)));
      }
    } else {
      out.items().emplace_back(f.name, core::Json(readOne(at, f.type)));
    }
  }
  return out;
}

std::vector<std::uint8_t> encodePayload(const MsgDef& def, const core::Json& obj) {
  std::vector<std::uint8_t> p(static_cast<std::size_t>(def.length), 0);
  for (int k = 0; k < def.nFields; k++) {
    const FieldDef& f = def.fields[k];
    const core::Json& v = obj[f.name];
    if (v.isNull()) continue;
    std::uint8_t* at = p.data() + f.offset;
    if (f.len) {
      if (f.type == kChar) {
        const auto bytes = toLatin1(v.isString() ? v.str() : v.dump());
        for (std::size_t i = 0; i < bytes.size() && i < static_cast<std::size_t>(f.len); i++) at[i] = bytes[i];
      } else if (v.isArray()) {
        const int sz = sizeOf(f.type);
        for (int i = 0; i < f.len && static_cast<std::size_t>(i) < v.size(); i++) writeOne(at + i * sz, f.type, v[static_cast<std::size_t>(i)].numOr(0));
      }
    } else {
      writeOne(at, f.type, v.numOr(0));
    }
  }
  return p;
}

std::vector<std::uint8_t> Encoder::pack(const std::string& name, const core::Json& fields) {
  const MsgDef* def = msgByName(name);
  if (!def) throw std::invalid_argument("unknown message " + name);
  std::vector<std::uint8_t> payload = encodePayload(*def, fields);
  // v2 trailing-zero truncation (keep at least 1 byte)
  std::size_t n = payload.size();
  while (n > 1 && payload[n - 1] == 0) n--;
  std::vector<std::uint8_t> frame(12 + n);
  frame[0] = 0xfd;
  frame[1] = static_cast<std::uint8_t>(n);
  frame[2] = 0;
  frame[3] = 0;
  frame[4] = seq_;
  frame[5] = static_cast<std::uint8_t>(sysid_);
  frame[6] = static_cast<std::uint8_t>(compid_);
  frame[7] = static_cast<std::uint8_t>(def->id & 0xff);
  frame[8] = static_cast<std::uint8_t>((def->id >> 8) & 0xff);
  frame[9] = static_cast<std::uint8_t>((def->id >> 16) & 0xff);
  std::memcpy(frame.data() + 10, payload.data(), n);
  std::uint16_t crc = 0xffff;
  for (std::size_t i = 1; i < 10 + n; i++) crc = crcX25(crc, frame[i]);
  crc = crcX25(crc, static_cast<std::uint8_t>(def->crcExtra));
  frame[10 + n] = static_cast<std::uint8_t>(crc & 0xff);
  frame[11 + n] = static_cast<std::uint8_t>(crc >> 8);
  seq_ = static_cast<std::uint8_t>(seq_ + 1);
  return frame;
}

std::vector<Message> Parser::feed(const std::uint8_t* data, std::size_t len) {
  buf_.insert(buf_.end(), data, data + len);
  std::vector<Message> out;
  std::size_t i = 0;
  const std::vector<std::uint8_t>& b = buf_;
  while (i < b.size()) {
    const std::uint8_t m = b[i];
    if (m != 0xfd && m != 0xfe) {
      i++;
      stats_.dropped++;
      continue;
    }
    const bool v2 = m == 0xfd;
    const std::size_t hdr = v2 ? 10 : 6;
    if (i + hdr > b.size()) break;
    const std::size_t plen = b[i + 1];
    const std::uint8_t incompat = v2 ? b[i + 2] : 0;
    const std::size_t sigLen = (v2 && (incompat & 1)) ? 13 : 0;
    const std::size_t total = hdr + plen + 2 + sigLen;
    if (i + total > b.size()) break;
    const int id = v2 ? (b[i + 7] | (b[i + 8] << 8) | (b[i + 9] << 16)) : b[i + 5];
    const MsgDef* def = msgById(id);
    if (!def) {
      stats_.unknown++;
      i++;
      continue;
    }
    std::uint16_t crc = 0xffff;
    for (std::size_t k = i + 1; k < i + hdr + plen; k++) crc = crcX25(crc, b[k]);
    crc = crcX25(crc, static_cast<std::uint8_t>(def->crcExtra));
    const std::uint16_t got = static_cast<std::uint16_t>(b[i + hdr + plen] | (b[i + hdr + plen + 1] << 8));
    if (crc != got) {
      stats_.badCrc++;
      i++;
      continue;
    }
    Message msg;
    msg.id = id;
    msg.name = def->name;
    msg.seq = v2 ? b[i + 4] : b[i + 2];
    msg.sysid = v2 ? b[i + 5] : b[i + 3];
    msg.compid = v2 ? b[i + 6] : b[i + 4];
    msg.payload.assign(static_cast<std::size_t>(def->length), 0);
    std::memcpy(msg.payload.data(), b.data() + i + hdr, std::min(plen, msg.payload.size()));
    msg.fields = decodePayload(*def, b.data() + i + hdr, plen);
    out.push_back(std::move(msg));
    stats_.frames++;
    i += total;
  }
  if (i >= buf_.size())
    buf_.clear();
  else
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(i));
  if (buf_.size() > (1u << 20)) buf_.clear();   // runaway garbage guard
  return out;
}

}  // namespace aptuner::mavlink
