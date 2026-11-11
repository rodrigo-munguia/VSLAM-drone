#ifndef GLOBALSLAM_G6B_FEEDBACK_COMPLETION_HPP_
#define GLOBALSLAM_G6B_FEEDBACK_COMPLETION_HPP_

/**
 * G6b.1: order-independent coordinated-correction completion.
 *
 * Defect (corr-on Webots): Wait_correction_feedback_barrier blocked inside the
 * LSposUpdate *response callback* on the component SingleThreadedExecutor.
 * CorrectionFeedbackAccepted was published (and often already in-flight) but
 * could not be delivered until the wait timed out — every applied txn aborted.
 *
 * Policy:
 * - Register interest *before* LS dispatch so early FeedbackAccepted is latched.
 * - Never block a ROS callback waiting for another callback on the same executor.
 * - Wakeups alone are not completion; predicates check explicit state.
 * - Duplicate FeedbackAccepted for the same correction_id is idempotent.
 * - Foreign/stale IDs do not complete the active txn.
 * - Reset / generation change invalidates pending work.
 */

#include <cstdint>
#include <deque>
#include <string>

namespace globalslam
{
namespace g6b_fb
{

enum class TxnPhase : uint8_t
{
  Idle = 0,
  /** Registered before LS dispatch; may latch early feedback. */
  Dispatched = 1,
  /** LS applied; waiting for matching FeedbackAccepted (non-blocking). */
  AwaitingFeedback = 2,
  /** Ready for coordinated release(resume). */
  ReadyToRelease = 3,
  /** Definite non-application → abort may resume plan. */
  NotApplied = 4,
  /** Applied but feedback unresolved at timeout → abort must NOT resume. */
  UnresolvedHold = 5,
};

inline const char * phase_name(TxnPhase p)
{
  switch (p) {
    case TxnPhase::Idle: return "idle";
    case TxnPhase::Dispatched: return "dispatched";
    case TxnPhase::AwaitingFeedback: return "awaiting_feedback";
    case TxnPhase::ReadyToRelease: return "ready_to_release";
    case TxnPhase::NotApplied: return "not_applied";
    case TxnPhase::UnresolvedHold: return "unresolved_hold";
  }
  return "unknown";
}

constexpr std::size_t kMaxEarlyFeedbackIds = 16;

struct PendingTxn
{
  TxnPhase phase{TxnPhase::Idle};
  uint64_t correction_id{0};
  uint64_t dispatch_generation{0};
  uint64_t epoch_id{0};
  std::string type;
  bool used_coordination{false};
  bool notify_control_on_apply{false};
  bool early_feedback_latched{false};
  uint64_t feedback_snapshot_seq{0};
  std::string feedback_mode;
};

inline bool is_terminal(TxnPhase p)
{
  return p == TxnPhase::ReadyToRelease || p == TxnPhase::NotApplied ||
    p == TxnPhase::UnresolvedHold || p == TxnPhase::Idle;
}

inline bool may_auto_resume_plan(TxnPhase p)
{
  // A: not applied → resume OK. B: ready_to_release uses release(resume).
  // C/D unresolved_hold → never auto-resume.
  return p == TxnPhase::NotApplied || p == TxnPhase::ReadyToRelease;
}

inline void clear_txn(PendingTxn & t) { t = PendingTxn{}; }

inline void push_early_id(std::deque<uint64_t> & q, uint64_t id)
{
  if (id == 0) {
    return;
  }
  for (uint64_t x : q) {
    if (x == id) {
      return;  // idempotent
    }
  }
  q.push_back(id);
  while (q.size() > kMaxEarlyFeedbackIds) {
    q.pop_front();
  }
}

inline bool consume_early_id(std::deque<uint64_t> & q, uint64_t id)
{
  for (auto it = q.begin(); it != q.end(); ++it) {
    if (*it == id) {
      q.erase(it);
      return true;
    }
  }
  return false;
}

/** Before LS async_send_request. */
inline void register_dispatch(
  PendingTxn & t,
  std::deque<uint64_t> & early_ids,
  uint64_t correction_id,
  uint64_t generation,
  uint64_t epoch_id,
  const std::string & type,
  bool used_coordination,
  bool notify_control_on_apply)
{
  t = PendingTxn{};
  t.phase = TxnPhase::Dispatched;
  t.correction_id = correction_id;
  t.dispatch_generation = generation;
  t.epoch_id = epoch_id;
  t.type = type;
  t.used_coordination = used_coordination;
  t.notify_control_on_apply = notify_control_on_apply;
  if (consume_early_id(early_ids, correction_id)) {
    t.early_feedback_latched = true;
  }
}

/**
 * FeedbackAccepted arrived. Returns true if the active txn is now
 * ReadyToRelease (caller should release).
 */
inline bool on_feedback_accepted(
  PendingTxn & t,
  std::deque<uint64_t> & early_ids,
  uint64_t correction_id,
  uint64_t snapshot_seq,
  const std::string & mode)
{
  if (correction_id == 0) {
    return false;
  }
  if (t.phase == TxnPhase::Idle || t.correction_id != correction_id) {
    // Early or foreign — latch bounded early id for a future dispatch.
    push_early_id(early_ids, correction_id);
    return false;
  }
  t.feedback_snapshot_seq = snapshot_seq;
  t.feedback_mode = mode;
  t.early_feedback_latched = true;
  if (t.phase == TxnPhase::Dispatched) {
    // Applied response not yet seen — stay Dispatched with latch.
    return false;
  }
  if (t.phase == TxnPhase::AwaitingFeedback) {
    t.phase = TxnPhase::ReadyToRelease;
    return true;
  }
  // Duplicate after ReadyToRelease / terminal: idempotent no-op.
  return false;
}

/**
 * LS response observed. Returns true if ReadyToRelease (caller should release).
 * If applied but no feedback yet → AwaitingFeedback (caller arms timeout timer).
 * If !applied → NotApplied (caller abort+resume).
 */
inline bool on_ls_outcome(PendingTxn & t, bool applied, bool unresolved)
{
  if (t.phase != TxnPhase::Dispatched && t.phase != TxnPhase::AwaitingFeedback) {
    return false;
  }
  if (unresolved) {
    t.phase = TxnPhase::UnresolvedHold;
    return false;
  }
  if (!applied) {
    t.phase = TxnPhase::NotApplied;
    return false;
  }
  if (t.early_feedback_latched) {
    t.phase = TxnPhase::ReadyToRelease;
    return true;
  }
  t.phase = TxnPhase::AwaitingFeedback;
  return false;
}

/** Barrier timer fired while AwaitingFeedback. */
inline void on_feedback_timeout(PendingTxn & t)
{
  if (t.phase == TxnPhase::AwaitingFeedback) {
    t.phase = TxnPhase::UnresolvedHold;
  }
}

inline void invalidate_generation(PendingTxn & t, uint64_t live_generation)
{
  if (t.phase == TxnPhase::Idle) {
    return;
  }
  if (t.dispatch_generation != live_generation) {
    clear_txn(t);
  }
}

}  // namespace g6b_fb
}  // namespace globalslam

#endif  // GLOBALSLAM_G6B_FEEDBACK_COMPLETION_HPP_
