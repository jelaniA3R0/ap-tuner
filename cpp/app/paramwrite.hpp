// Verified parameter writes (2026-10-17 audit F-28, F-29, F-30, F-32, F-35, F-51). Every value is cast to the
// parameter's type (whole numbers for integer parameters, float32 otherwise), sent, and the echo compared exactly; a
// missing or different echo is followed by one more read, so the result says what the vehicle holds. A set of writes is
// a transaction: read fresh, write in order, and on the first failure write everything back (newest first, verified);
// what cannot be proven is reported as uncertain. Values that take effect only after a reboot get a reboot on the
// ground, after which every value is read again. Port of src/engine/paramwrite.js (results are the same JSON objects).
#pragma once

#include <functional>
#include <string>

#include "aptuner/core/json.hpp"
#include "aptuner/io/link.hpp"
#include "aptuner/rt/async.hpp"

namespace aptuner::app {

using core::Json;

struct Cast {
  bool ok = false;
  double value = 0;
  std::string note, error;
};
/** the value to send for a parameter of this MAV_PARAM_TYPE */
Cast castForType(double value, int type);

/** results after which the vehicle may hold something else than before (verified, different, uncertain) */
bool changedStatus(const std::string& status);
/** one result in plain words */
std::string writeText(const Json& r);
/** one sentence on a transaction: what failed, and whether the rest went back */
std::string txnText(const Json& t);

using JournalFn = std::function<void(const std::string& kind, const Json& data)>;

struct VerifyOpts {
  Json before;   // the value before (null: the link's cached value)
  double timeoutMs = 1500;
  int retries = 3, readRetries = 2;
  std::function<bool()> keepGoing;
};
/** write one parameter and prove what the vehicle holds afterwards; never throws. Resolves {name, requested, sent, type,
    before, echo, readBack, holds, status, ok, error, note, cancelled, attempts, ms}; status: verified, refused, different,
    not-applied, uncertain, missing, invalid, cancelled, not-sent */
rt::Task<Json> verifiedSet(io::Link& link, std::string name, double value, VerifyOpts opts = {});

struct TxnOpts {
  JournalFn journal;
  std::function<bool()> keepGoing;
  std::function<void(const Json& before)> afterPreRead;   // [{name, before}]; may throw to stop before the first write
  Json ctx = Json::object();                              // added to every journal event
  double timeoutMs = 1500;
  int retries = 3, readRetries = 2;
};
/** a set of writes that either all hold or are written back; items [{name, to, expect?, reason?}]; never throws.
    Resolves {txn, ok, aborted, stale, cancelled, rows, rollback, rolledBack, uncertain [{name, holds, why}]} */
rt::Task<Json> runTransaction(io::Link& link, Json items, TxnOpts opts = {});

struct RebootOpts {
  JournalFn journal;
  Json ctx = Json::object();
  double timeoutMs = 60000;
  std::function<void(const std::string&)> onInfo;
};
/** the reboot workflow: only disarmed, ACK from the target, gone and back, the restart proven, the same vehicle, every
    expected value [{name, value}] read fresh. Never throws; resolves {ok, reason, rebooted, rows [{name, expect, holds, ok}], identity} */
rt::Task<Json> rebootAndVerify(io::Link& link, Json expect, RebootOpts opts = {});

}  // namespace aptuner::app
