#ifndef CONTROL_G6_FEEDBACK_BARRIER_HPP_
#define CONTROL_G6_FEEDBACK_BARRIER_HPP_

/**
 * G6b: associate CorrectionApplied with a unique post-correction odometry
 * snapshot that Control has actually accepted.
 *
 * CorrectionApplied alone does not prove Control consumed a corrected pose —
 * separate topics have no cross-topic ordering. Association key is
 * snapshot_seq (not stamp alone).
 *
 * GT feedback (Slam_feedback=false): corrected odometry is not the plan pose
 * source; barrier may clear on a matching CorrectionApplied while paused.
 *
 * Odometry feedback: require an accepted (odom+meta) with matching
 * session/epoch, last_correction_id, and snapshot_seq >= required.
 */

#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <utility>

#include "../../common/slam_identity.hpp"

namespace control
{
namespace g6_feedback
{

constexpr size_t kMaxBufferedMetas = 32;
constexpr size_t kMaxBufferedOdomStamps = 32;
constexpr size_t kMaxAcceptedHistory = 32;

struct PendingBarrier
{
  bool armed{false};
  slam_identity::UuidBytes session_id{slam_identity::nil_uuid()};
  uint64_t epoch_id{0};
  uint64_t correction_id{0};
  uint64_t required_snapshot_seq{0};
  bool accepted{false};
  uint64_t accepted_snapshot_seq{0};
  std::string feedback_mode;
};

struct MetaSample
{
  slam_identity::UuidBytes session_id{slam_identity::nil_uuid()};
  uint64_t epoch_id{0};
  uint64_t snapshot_seq{0};
  uint64_t last_correction_id{0};
  int32_t stamp_sec{0};
  uint32_t stamp_nanosec{0};
};

struct AcceptedSample
{
  slam_identity::UuidBytes session_id{slam_identity::nil_uuid()};
  uint64_t epoch_id{0};
  uint64_t snapshot_seq{0};
  uint64_t last_correction_id{0};
};

inline bool same_stamp(const MetaSample & a, int32_t sec, uint32_t nsec)
{
  return a.stamp_sec == sec && a.stamp_nanosec == nsec;
}

inline bool meta_satisfies(
  const PendingBarrier & b, const MetaSample & m)
{
  if (!b.armed || b.accepted) {
    return false;
  }
  if (b.correction_id == 0 || m.snapshot_seq == 0) {
    return false;
  }
  if (slam_identity::is_nil(b.session_id) || slam_identity::is_nil(m.session_id)) {
    return false;
  }
  if (!slam_identity::equal(b.session_id, m.session_id) || b.epoch_id != m.epoch_id) {
    return false;
  }
  if (m.last_correction_id != b.correction_id) {
    return false;
  }
  return m.snapshot_seq >= b.required_snapshot_seq;
}

inline bool accepted_satisfies(
  const PendingBarrier & b, const AcceptedSample & a)
{
  MetaSample m;
  m.session_id = a.session_id;
  m.epoch_id = a.epoch_id;
  m.snapshot_seq = a.snapshot_seq;
  m.last_correction_id = a.last_correction_id;
  return meta_satisfies(b, m);
}

/** Arm from CorrectionApplied. Rejects nil/zero identity. */
inline bool arm_from_applied(
  PendingBarrier & b,
  const slam_identity::UuidBytes & session_id,
  uint64_t epoch_id,
  uint64_t correction_id,
  uint64_t snapshot_seq)
{
  if (correction_id == 0 || snapshot_seq == 0 ||
    slam_identity::is_nil(session_id) || epoch_id == 0)
  {
    return false;
  }
  b.armed = true;
  b.session_id = session_id;
  b.epoch_id = epoch_id;
  b.correction_id = correction_id;
  b.required_snapshot_seq = snapshot_seq;
  b.accepted = false;
  b.accepted_snapshot_seq = 0;
  b.feedback_mode.clear();
  return true;
}

/** GT path: CorrectionApplied is enough while coordinating (pose is GPS). */
inline bool accept_gt(PendingBarrier & b)
{
  if (!b.armed || b.accepted) {
    return false;
  }
  b.accepted = true;
  b.accepted_snapshot_seq = b.required_snapshot_seq;
  b.feedback_mode = "gt";
  return true;
}

inline bool accept_odometry(PendingBarrier & b, uint64_t snapshot_seq)
{
  if (!b.armed || b.accepted) {
    return false;
  }
  b.accepted = true;
  b.accepted_snapshot_seq = snapshot_seq;
  b.feedback_mode = "odometry";
  return true;
}

inline void clear_barrier(PendingBarrier & b)
{
  b = PendingBarrier{};
}

/** Drop pending / history on estimator reset (new session or epoch). */
inline void on_estimator_reset(
  PendingBarrier & b,
  std::deque<MetaSample> & meta_buf,
  std::deque<std::pair<int32_t, uint32_t>> & odom_stamp_buf,
  std::deque<AcceptedSample> & accepted_hist)
{
  clear_barrier(b);
  meta_buf.clear();
  odom_stamp_buf.clear();
  accepted_hist.clear();
}

inline void push_bounded_meta(std::deque<MetaSample> & buf, const MetaSample & m)
{
  buf.push_back(m);
  while (buf.size() > kMaxBufferedMetas) {
    buf.pop_front();
  }
}

inline void push_bounded_stamp(
  std::deque<std::pair<int32_t, uint32_t>> & buf, int32_t sec, uint32_t nsec)
{
  buf.emplace_back(sec, nsec);
  while (buf.size() > kMaxBufferedOdomStamps) {
    buf.pop_front();
  }
}

inline void push_accepted(std::deque<AcceptedSample> & hist, const AcceptedSample & a)
{
  hist.push_back(a);
  while (hist.size() > kMaxAcceptedHistory) {
    hist.pop_front();
  }
}

/**
 * Find meta matching odom stamp. If several share the stamp, prefer the
 * highest snapshot_seq (unique association remains snapshot_seq).
 */
inline std::optional<MetaSample> find_meta_for_stamp(
  const std::deque<MetaSample> & buf, int32_t sec, uint32_t nsec)
{
  std::optional<MetaSample> best;
  for (const auto & m : buf) {
    if (!same_stamp(m, sec, nsec)) {
      continue;
    }
    if (!best || m.snapshot_seq >= best->snapshot_seq) {
      best = m;
    }
  }
  return best;
}

inline bool stamp_seen(
  const std::deque<std::pair<int32_t, uint32_t>> & buf, int32_t sec, uint32_t nsec)
{
  for (const auto & s : buf) {
    if (s.first == sec && s.second == nsec) {
      return true;
    }
  }
  return false;
}

/**
 * Try to satisfy an armed barrier from recent accepted history (odom-before-event).
 */
inline bool try_accept_from_history(
  PendingBarrier & b, const std::deque<AcceptedSample> & hist)
{
  if (!b.armed || b.accepted) {
    return false;
  }
  for (auto it = hist.rbegin(); it != hist.rend(); ++it) {
    if (accepted_satisfies(b, *it)) {
      return accept_odometry(b, it->snapshot_seq);
    }
  }
  return false;
}

/** Reject stale session/epoch relative to a live reference. */
inline bool identity_matches_live(
  const slam_identity::UuidBytes & live_session,
  uint64_t live_epoch,
  const slam_identity::UuidBytes & msg_session,
  uint64_t msg_epoch)
{
  if (slam_identity::is_nil(live_session) || live_epoch == 0) {
    // No live latch yet — accept message identity as-is for association.
    return !slam_identity::is_nil(msg_session) && msg_epoch != 0;
  }
  return slam_identity::equal(live_session, msg_session) && live_epoch == msg_epoch;
}

}  // namespace g6_feedback
}  // namespace control

#endif  // CONTROL_G6_FEEDBACK_BARRIER_HPP_
