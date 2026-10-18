// Session identity and proposal binding (2026-10-17 audit F-25, F-33). The identity of a connection is the link
// generation, the vehicle's system / component id, autopilot and frame type, AUTOPILOT_VERSION (board UID, firmware)
// and a hash of the parameter snapshot read from it. A proposal has an id and a hash over its identity and the exact
// list of (parameter, value) it writes. FNV-1a 64 over a canonical text with float32 values as their 8 hex digits, so
// both editions agree to the bit. Port of src/engine/identity.js.
#pragma once

#include <string>

#include "aptuner/core/json.hpp"

namespace aptuner::engine {

/** the 8 hex digits of a value as a float32 (what the autopilot stores) */
std::string f32hex(double v);
/** FNV-1a 64-bit hash of a text (UTF-8 bytes), 16 hex digits */
std::string fnv1a64(const std::string& text);
/** the same value once stored as a float32 (the exact write check; NaN / missing is never the same) */
bool sameF32(const core::Json& a, const core::Json& b);
bool sameF32(double a, double b);
/** hash of a parameter snapshot: (name, float32 value) of every number, sorted by name */
std::string paramHash(const core::Json& params);
/** the identity of a session: the link's identity() plus key and the hash of the parameters read (null: none) */
core::Json sessionIdentity(const core::Json& linkIdentity, const core::Json& params);
/** a sentence for the UI and file headers */
std::string identityText(const core::Json& id);
/** the same vehicle (across connections): ids, frame type and, where both are known, board UID and firmware */
bool sameVehicle(const core::Json& a, const core::Json& b);
/** {id, hash, source, axis, identityKey, gen, sysid, compid, at, rows}; axis "" = none; rows [{name, to, from, ...}] */
core::Json makeProposal(const std::string& source, const std::string& axis, const core::Json& identity, const core::Json& rows);
/** a number / null / string as JavaScript's template literals print it ("null" for null) */
std::string jsText(const core::Json& v);

}  // namespace aptuner::engine
