#ifndef CONTROL_G6_COORD_POLICY_HPP_
#define CONTROL_G6_COORD_POLICY_HPP_

/**
 * G6a correction↔control coordination policy (header-only, unit-tested).
 *
 * States: idle → paused → (release|abort) → idle
 * At most one owned transaction. Conflicting prepare rejected (not overwritten).
 * Idempotent prepare/release/abort/query for the same (owner, transaction).
 *
 * Pause acknowledgement (caller responsibility after policy says paused):
 * - Plan progression held (run_control_plan cleared if it was active).
 * - Command suppress armed; zero twist published.
 * - in_plan_command cleared (go_point/explore exited or never running).
 * Does NOT prove vehicle velocity is zero or physical hover.
 *
 * Release restores plan only if plan_was_active && resume_requested &&
 * !operator_superseded && session/epoch still match the pause-time identity.
 */

#include <cstdint>
#include <string>

#include "../../common/slam_identity.hpp"

namespace control
{
namespace g6_coord
{

enum class State : uint8_t
{
  Idle = 0,
  Paused = 1,
};

inline const char * state_name(State s)
{
  switch (s) {
    case State::Idle: return "idle";
    case State::Paused: return "paused";
  }
  return "unknown";
}

struct TxnIdentity
{
  slam_identity::UuidBytes session_id{slam_identity::nil_uuid()};
  uint64_t epoch_id{0};
  uint64_t transaction_id{0};
  uint64_t owner_generation{0};
};

struct CoordSnapshot
{
  State state{State::Idle};
  TxnIdentity active{};
  bool plan_was_active{false};
  bool operator_superseded{false};
};

struct CoordResult
{
  bool success{false};
  State state{State::Idle};
  std::string reason;
  bool plan_was_active{false};
  bool plan_resumed{false};
  bool cmd_source_quiescent{false};
  uint64_t active_transaction_id{0};
  uint64_t active_owner_generation{0};
};

inline bool same_txn(const TxnIdentity & a, const TxnIdentity & b)
{
  return a.transaction_id == b.transaction_id &&
    a.owner_generation == b.owner_generation &&
    a.epoch_id == b.epoch_id &&
    slam_identity::equal(a.session_id, b.session_id);
}

inline CoordResult fill_from(const CoordSnapshot & snap, bool success, const std::string & reason)
{
  CoordResult r;
  r.success = success;
  r.state = snap.state;
  r.reason = reason;
  r.plan_was_active = snap.plan_was_active;
  r.active_transaction_id = snap.active.transaction_id;
  r.active_owner_generation = snap.active.owner_generation;
  return r;
}

/**
 * @param plan_currently_active current run_control_plan
 * @param cmd_quiescent true once in_plan_command is false and zero published
 */
inline CoordResult prepare_pause(
  CoordSnapshot & snap,
  const TxnIdentity & req,
  bool plan_currently_active,
  bool cmd_quiescent)
{
  if (req.transaction_id == 0 || req.owner_generation == 0) {
    return fill_from(snap, false, "missing_transaction_or_owner");
  }
  if (slam_identity::is_nil(req.session_id) || req.epoch_id == 0) {
    return fill_from(snap, false, "missing_session_or_epoch");
  }
  if (snap.state == State::Paused) {
    if (same_txn(snap.active, req)) {
      // Idempotent re-prepare.
      auto r = fill_from(snap, cmd_quiescent, cmd_quiescent ? "" : "awaiting_cmd_quiescence");
      r.cmd_source_quiescent = cmd_quiescent;
      r.success = cmd_quiescent;
      return r;
    }
    return fill_from(snap, false, "conflicting_transaction");
  }
  // Idle → take ownership.
  snap.state = State::Paused;
  snap.active = req;
  snap.plan_was_active = plan_currently_active;
  snap.operator_superseded = false;
  auto r = fill_from(snap, cmd_quiescent, cmd_quiescent ? "" : "awaiting_cmd_quiescence");
  r.cmd_source_quiescent = cmd_quiescent;
  r.success = cmd_quiescent;
  r.plan_was_active = snap.plan_was_active;
  return r;
}

inline CoordResult query(const CoordSnapshot & snap, const TxnIdentity & req)
{
  if (snap.state == State::Idle) {
    if (req.transaction_id != 0 && snap.active.transaction_id == req.transaction_id) {
      // Cleared; report idle for that id.
    }
    return fill_from(snap, true, "idle");
  }
  if (req.transaction_id != 0 && !same_txn(snap.active, req) &&
    req.transaction_id != snap.active.transaction_id)
  {
    auto r = fill_from(snap, false, "other_transaction_active");
    return r;
  }
  return fill_from(snap, true, "");
}

inline CoordResult release(
  CoordSnapshot & snap,
  const TxnIdentity & req,
  bool resume_plan_if_owned,
  bool identity_still_valid)
{
  if (snap.state != State::Paused) {
    // Idempotent release when already idle for same id.
    if (req.transaction_id != 0 && req.transaction_id == snap.active.transaction_id &&
      req.owner_generation == snap.active.owner_generation)
    {
      return fill_from(snap, true, "already_idle");
    }
    return fill_from(snap, false, "no_active_pause");
  }
  if (!same_txn(snap.active, req)) {
    return fill_from(snap, false, "transaction_mismatch");
  }
  const bool may_resume = resume_plan_if_owned && snap.plan_was_active &&
    !snap.operator_superseded && identity_still_valid;
  CoordResult r = fill_from(snap, true, "");
  r.plan_was_active = snap.plan_was_active;
  r.plan_resumed = may_resume;
  r.cmd_source_quiescent = true;
  snap.state = State::Idle;
  // Keep last active ids for idempotent query/release; clear ownership.
  snap.plan_was_active = false;
  return r;
}

/**
 * Abort the owned pause.
 * @param resume_requested caller asserts outcome permits resume (definite
 *   non-application). Applied-but-unresolved feedback must pass false —
 *   timeout is not evidence of non-application (G6b.1).
 */
inline CoordResult abort_txn(
  CoordSnapshot & snap, const TxnIdentity & req, bool resume_requested = false)
{
  if (snap.state != State::Paused) {
    return fill_from(snap, true, "already_idle");
  }
  if (!same_txn(snap.active, req)) {
    return fill_from(snap, false, "transaction_mismatch");
  }
  const bool was_active = snap.plan_was_active;
  const bool may_resume =
    resume_requested && was_active && !snap.operator_superseded;
  CoordResult r = fill_from(snap, true, "aborted");
  r.plan_was_active = was_active;
  r.plan_resumed = may_resume;
  snap.state = State::Idle;
  snap.plan_was_active = false;
  return r;
}

/** Operator stop/reset/land while paused: keep pause owned but block resume. */
inline void mark_operator_superseded(CoordSnapshot & snap)
{
  if (snap.state == State::Paused) {
    snap.operator_superseded = true;
    snap.plan_was_active = false;
  }
}

}  // namespace g6_coord
}  // namespace control

#endif  // CONTROL_G6_COORD_POLICY_HPP_
