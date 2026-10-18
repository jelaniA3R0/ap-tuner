// "Telemetry + USB" session for vehicles flown on a slow telemetry radio: flight (telemetry only marks the time)
// -> capture -> land -> usb (the app finds the autopilot's USB port) -> download (log head + the marked window)
// -> review -> write (on the ground, disarmed, each value read back) -> done (undo file kept).
// The write (2026-10-17 audit R3): the review checks every row and binds the list to the connection (proposal id +
// hash); Write is one transaction (fresh read, UNDO file, exact verification, rollback) with a truthful status per row;
// Undo is verified value by value; values that act only after a reboot get the reboot workflow.
// Port of src/engine/groundtune.js.
#pragma once

#include <memory>
#include <optional>
#include <set>
#include <string>

#include "aptuner/core/json.hpp"
#include "aptuner/io/link.hpp"
#include "aptuner/io/logdl.hpp"
#include "aptuner/rt/async.hpp"
#include "aptuner/rt/emitter.hpp"
#include "journal.hpp"

namespace aptuner::app {

using core::Json;

class GroundTune : public rt::Emitter {
 public:
  /** journal: the write journal of the app run (null: one of its own in sessionsDir/journal) */
  GroundTune(io::Link& link, const std::string& sessionsDir, Journal* journal = nullptr);
  void reset();
  void start(const Json& opts);
  void stop();
  void captureAgain();
  rt::Task<void> goUsb(bool useSaved);
  /** write the chosen rows: approval = {id, hash} of the reviewed list; resolves the rows with their status */
  rt::Task<Json> write(std::vector<std::string> names, Json approval = Json());
  /** write the values from before back, each verified on its own */
  rt::Task<void> undo();
  /** read the values on the autopilot again: a new proposal to review */
  rt::Task<void> recheck();
  /** reboot (disarmed) after values that act only after a reboot; every value written here is read again after it */
  rt::Task<Json> rebootNow();
  Json snapshot() const;

  const std::string& phase() const { return phase_; }
  const std::string& info() const { return info_; }
  const Json& rows() const { return rows_; }
  const Json& writes() const { return writes_; }
  const Json& proposal() const { return proposal_; }
  const Json& rebootPending() const { return rebootPending_; }
  Journal& audit() { return *audit_; }

  Json opts;
  Json result;   // last analysis (null until downloaded)
  Json files = Json::object();

 private:
  friend struct GroundTestAccess;   // cpp/tests: a review without a log download, as test/write-verify.test.js does
  std::string markFile() const;
  void saveMark();
  void restoreMark();
  std::optional<double> nowBootMs() const;
  std::string familyOf(const Json& v) const;
  bool modeOk(const Json& v) const;
  void setPhase(const std::string& p, const std::string& info = "");
  void note(const std::string& level, const std::string& text);
  void stopTimer();
  void tick();
  void loopOnce();
  rt::Task<void> step();
  rt::Task<void> enterUsb();
  rt::Task<void> scanUsb();
  rt::Task<void> download();
  rt::Task<void> review();
  rt::Task<void> reviewReader(std::shared_ptr<std::size_t> next);
  Json rowResult(const Json& r, const Json& txn) const;
  void fail(const std::string& msg);

  io::Link& link_;
  io::LogDownloader dl_;
  std::string dir_;
  std::unique_ptr<Journal> ownJournal_;
  Journal* audit_ = nullptr;
  double bootMs_ = -1, bootAt_ = 0;
  std::string phase_ = "idle", info_;
  double since_ = 0, tFlying_ = 0, usbSince_ = 0;
  Json mark_, savedMark_, captureStart_, usb_, progress_, rows_ = Json::array(), writes_ = Json::array(), notes_ = Json::array(), error_;
  Json identity_, proposal_, logCheck_, rebootPending_ = Json::array(), afterReboot_ = Json::array(), reboot_, lastTxn_;
  std::optional<int> sysid_;
  std::set<std::string> knownPorts_, busy_, tried_;
  std::uint64_t timer_ = 0;
};

}  // namespace aptuner::app
