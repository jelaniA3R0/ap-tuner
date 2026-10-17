// Phase 3 unit tests: MAVLink codec (round trip, v1 frames, truncation, resync, CRC), the async runtime
// (tasks, sleep, exceptions, whenAll, timers) and the event emitter.
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "aptuner/mavlink/mavlink.hpp"
#include "aptuner/rt/async.hpp"
#include "aptuner/rt/emitter.hpp"
#include "aptuner/rt/loop.hpp"
#include "check.hpp"

using aptuner::core::Json;
using namespace aptuner;

TEST(mavlink_round_trip) {
  mavlink::Encoder enc(255, 190);
  const Json hb = Json::obj({{"type", 6}, {"autopilot", 8}, {"base_mode", 0}, {"custom_mode", 0}, {"system_status", 4}, {"mavlink_version", 3}});
  const Json ps = Json::obj({{"target_system", 1}, {"target_component", 1}, {"param_id", "ATC_RAT_RLL_P"}, {"param_value", 0.135}, {"param_type", 9}});
  std::vector<std::uint8_t> bytes;
  for (const auto& f : {enc.pack("HEARTBEAT", hb), enc.pack("PARAM_SET", ps)}) bytes.insert(bytes.end(), f.begin(), f.end());
  // garbage in front and a split delivery: the parser must resync and buffer
  std::vector<std::uint8_t> stream{0x00, 0x13, 0xfd, 0x02};
  stream.insert(stream.end(), bytes.begin(), bytes.end());
  mavlink::Parser p;
  auto first = p.feed(stream.data(), 9);
  auto rest = p.feed(stream.data() + 9, stream.size() - 9);
  first.insert(first.end(), rest.begin(), rest.end());
  CHECK(first.size() == 2);
  if (first.size() == 2) {
    CHECK(std::string(first[0].name) == "HEARTBEAT" && first[0].sysid == 255 && first[0].compid == 190);
    CHECK(first[0].fields["system_status"].num() == 4 && first[0].fields["mavlink_version"].num() == 3);
    CHECK(std::string(first[1].name) == "PARAM_SET" && first[1].fields["param_id"].str() == "ATC_RAT_RLL_P");
    CHECK_NEAR(first[1].fields["param_value"].num(), 0.135, 1e-7);
    CHECK(first[1].seq == 1);
  }
  // a v2 frame with trailing zeros truncated: HEARTBEAT with type 0 only keeps 1 payload byte... still decodes
  const auto z = enc.pack("HEARTBEAT", Json::obj({{"custom_mode", 0}}));
  CHECK(z[1] == 1);
  mavlink::Parser p2;
  const auto zm = p2.feed(z.data(), z.size());
  CHECK(zm.size() == 1 && zm[0].fields["type"].num() == 0);
  // a corrupted CRC is rejected
  auto bad = enc.pack("HEARTBEAT", hb);
  bad[bad.size() - 1] ^= 0xff;
  mavlink::Parser p3;
  CHECK(p3.feed(bad.data(), bad.size()).empty() && p3.stats().badCrc == 1);
}

TEST(mavlink_v1_frame) {
  // MAVLink v1 HEARTBEAT: fe len seq sys comp id payload crc
  const mavlink::MsgDef* def = mavlink::msgByName("HEARTBEAT");
  CHECK(def && def->crcExtra == 50 && def->length == 9);
  std::vector<std::uint8_t> payload = mavlink::encodePayload(*def, Json::obj({{"type", 2}, {"autopilot", 3}, {"custom_mode", 5}}));
  std::vector<std::uint8_t> f{0xfe, 9, 7, 1, 1, 0};
  f.insert(f.end(), payload.begin(), payload.end());
  std::uint16_t crc = 0xffff;
  for (std::size_t i = 1; i < f.size(); i++) crc = mavlink::crcX25(crc, f[i]);
  crc = mavlink::crcX25(crc, 50);
  f.push_back(static_cast<std::uint8_t>(crc & 0xff));
  f.push_back(static_cast<std::uint8_t>(crc >> 8));
  mavlink::Parser p;
  const auto m = p.feed(f.data(), f.size());
  CHECK(m.size() == 1 && m[0].seq == 7 && m[0].fields["custom_mode"].num() == 5 && m[0].fields["type"].num() == 2);
  CHECK(std::string(mavlink::enumName("MAV_TYPE", 2)) == "MAV_TYPE_QUADROTOR");
  CHECK(std::string(mavlink::enumName("PLANE_MODE", 17)) == "PLANE_MODE_QSTABILIZE");
}

namespace {
rt::Task<int> addLater(int a, int b) {
  co_await rt::sleep(20);
  co_return a + b;
}
rt::Task<int> failing() {
  co_await rt::sleep(5);
  throw rt::Error("boom", "code1");
}
rt::Task<std::string> chain() {
  int x = co_await addLater(1, 2);
  std::string err;
  try {
    co_await failing();
  } catch (const rt::Error& e) {
    err = std::string(e.what()) + "/" + e.code();
  }
  auto all = co_await rt::whenAll(std::vector<rt::Future<int>>{addLater(1, 1), addLater(2, 2), addLater(3, 3)});
  co_return std::to_string(x) + " " + err + " " + std::to_string(all[0] + all[1] + all[2]);
}
}  // namespace

TEST(async_runtime) {
  auto& loop = rt::Loop::main();
  std::string out;
  bool done = false;
  loop.post([&] {
    rt::Future<std::string> f = chain();
    f.then([&, f] {
      out = f.get();
      done = true;
    });
  });
  int ticks = 0;
  std::uint64_t iv = 0;
  loop.post([&] { iv = loop.setInterval(10, [&] { ticks++; }); });
  CHECK(loop.runUntil([&] { return done; }, 5000));
  loop.clear(iv);
  CHECK(out == "3 boom/code1 12");
  CHECK(ticks >= 2);
  // emitter: a handler that removes itself during emit
  rt::Emitter em;
  int calls = 0;
  std::uint64_t id1 = 0;
  id1 = em.on("x", [&](const Json&) {
    calls++;
    em.off(id1);
  });
  em.on("x", [&](const Json& v) { calls += static_cast<int>(v.num()); });
  em.emit("x", Json(10));
  em.emit("x", Json(100));
  CHECK(calls == 111);
}
