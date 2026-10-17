// MAVLink v1 / v2 frame parser and v2 encoder, driven by the message table generated from the same definitions
// as the JavaScript version (tools/gen-mavlink-cpp.js). Port of src/mavlink/mavlink.js. Message fields are
// JSON objects with the MAVLink field names.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "aptuner/core/json.hpp"

namespace aptuner::mavlink {

enum FieldType : std::uint8_t { kChar, kI8, kU8, kI16, kU16, kI32, kU32, kF32, kF64, kI64, kU64 };

struct FieldDef {
  const char* name;
  FieldType type;
  int len;      // array length (0 = scalar)
  int offset;   // byte offset in the payload (wire order)
};

struct MsgDef {
  int id;
  const char* name;
  int crcExtra;
  int length;   // full payload length
  const FieldDef* fields;
  int nFields;
};

const MsgDef* msgById(int id);
const MsgDef* msgByName(const std::string& name);

/** enum name for a value: "MAV_TYPE_QUADROTOR" etc. (MAV_TYPE, COPTER_MODE, PLANE_MODE); nullptr if unknown */
const char* enumName(const std::string& enumName, long long value);

struct Message {
  int id = 0;
  const char* name = "";
  int sysid = 0, compid = 0, seq = 0;
  core::Json fields;                  // decoded fields
  std::vector<std::uint8_t> payload;  // zero-extended to the full length
};

core::Json decodePayload(const MsgDef& def, const std::uint8_t* payload, std::size_t len);
std::vector<std::uint8_t> encodePayload(const MsgDef& def, const core::Json& fields);

std::uint16_t crcX25(std::uint16_t crc, std::uint8_t byte);

class Encoder {
 public:
  Encoder(int sysid = 254, int compid = 190) : sysid_(sysid), compid_(compid) {}
  /** a MAVLink v2 frame; throws std::invalid_argument for an unknown message */
  std::vector<std::uint8_t> pack(const std::string& name, const core::Json& fields);

 private:
  int sysid_, compid_;
  std::uint8_t seq_ = 0;
};

class Parser {
 public:
  struct Stats {
    std::uint64_t frames = 0, badCrc = 0, unknown = 0, dropped = 0;
  };
  /** feed bytes, get the complete messages decoded so far */
  std::vector<Message> feed(const std::uint8_t* data, std::size_t n);
  const Stats& stats() const { return stats_; }

 private:
  std::vector<std::uint8_t> buf_;
  Stats stats_;
};

}  // namespace aptuner::mavlink
