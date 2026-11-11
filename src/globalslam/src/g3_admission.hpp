#ifndef GLOBALSLAM_G3_ADMISSION_HPP_
#define GLOBALSLAM_G3_ADMISSION_HPP_

/**
 * G3 Global SLAM EstimatorReference + keyframe admission (header-only).
 *
 * Policies:
 * - Never adopt identity from a keyframe; wait for EstimatorReference.
 * - Keyframes arriving before a valid reference are dropped (no silent buffer
 *   that could adopt identity later).
 * - Same-session altitude/metadata updates (same epoch) do not reset GS.
 * - Session or epoch change / invalid reference → clear GS state (caller).
 * - Delayed old-session data after a newer session was adopted is ignored.
 * - Duplicate (session, epoch, keyframe_id, stream) is dropped.
 * - Missing source stamp rejected unless accept_missing_source_stamp=true.
 * - Image size must match configured Mono_cam rows/cols.
 */

#include <cstdint>
#include <string>
#include <unordered_set>

#include "../../common/slam_identity.hpp"

namespace globalslam
{

struct ActiveReference
{
  bool have_seen_any{false};
  bool valid{false};
  slam_identity::UuidBytes session_id{slam_identity::nil_uuid()};
  uint64_t epoch_id{0};
  /** Sessions previously replaced by a newer Local SLAM instance. */
  std::unordered_set<std::string> retired_sessions;
};

enum class RefUpdateAction
{
  None,
  /** valid=false for the active session/epoch (or first invalid). */
  Invalidate,
  /** New epoch same session, or first valid, or new session. */
  IdentityChange,
  /** Same (session, epoch) metadata-only update (e.g. altitude). */
  MetadataOnly,
  /** Delayed old session after a newer one was adopted — ignore. */
  RejectStaleSession,
};

inline RefUpdateAction classify_reference_update(
  const ActiveReference & cur,
  bool msg_valid,
  const slam_identity::UuidBytes & msg_session,
  uint64_t msg_epoch)
{
  if (slam_identity::is_nil(msg_session)) {
    return RefUpdateAction::None;
  }
  const std::string sid = slam_identity::to_string(msg_session);

  if (!msg_valid) {
    if (!cur.have_seen_any) {
      return RefUpdateAction::Invalidate;
    }
    if (slam_identity::equal(msg_session, cur.session_id)) {
      return RefUpdateAction::Invalidate;
    }
    return RefUpdateAction::None;
  }

  // msg_valid — one authoritative Local SLAM publisher assumed.
  if (cur.retired_sessions.count(sid) != 0) {
    return RefUpdateAction::RejectStaleSession;
  }
  if (!cur.have_seen_any || !cur.valid) {
    return RefUpdateAction::IdentityChange;
  }
  if (!slam_identity::equal(msg_session, cur.session_id)) {
    // New component instance: adopt, and retire the previous session so a
    // delayed old latched message cannot switch us back.
    return RefUpdateAction::IdentityChange;
  }
  if (msg_epoch != cur.epoch_id) {
    return RefUpdateAction::IdentityChange;
  }
  (void)msg_epoch;
  return RefUpdateAction::MetadataOnly;
}

inline void apply_reference_update(
  ActiveReference & cur,
  RefUpdateAction action,
  bool msg_valid,
  const slam_identity::UuidBytes & msg_session,
  uint64_t msg_epoch)
{
  cur.have_seen_any = true;
  switch (action) {
    case RefUpdateAction::Invalidate:
      cur.valid = false;
      cur.session_id = msg_session;
      cur.epoch_id = msg_epoch;
      break;
    case RefUpdateAction::IdentityChange: {
      if (!slam_identity::is_nil(cur.session_id) &&
        !slam_identity::equal(cur.session_id, msg_session))
      {
        cur.retired_sessions.insert(slam_identity::to_string(cur.session_id));
      }
      cur.valid = true;
      cur.session_id = msg_session;
      cur.epoch_id = msg_epoch;
      break;
    }
    case RefUpdateAction::MetadataOnly:
      cur.valid = msg_valid;
      break;
    default:
      break;
  }
}

struct KeyframeAdmitResult
{
  bool accepted{false};
  std::string reason;
};

inline uint64_t kf_dedupe_key(uint64_t keyframe_id, uint8_t stream)
{
  return (keyframe_id << 8) | static_cast<uint64_t>(stream);
}

inline KeyframeAdmitResult admit_keyframe(
  const ActiveReference & ref,
  const slam_identity::UuidBytes & kf_session,
  uint64_t kf_epoch,
  uint64_t keyframe_id,
  uint8_t stream,
  bool source_stamp_valid,
  bool accept_missing_source_stamp,
  int img_rows,
  int img_cols,
  int expected_rows,
  int expected_cols,
  std::unordered_set<uint64_t> & seen_ids)
{
  if (!ref.valid) {
    return {false, "no_valid_reference"};
  }
  if (slam_identity::is_nil(kf_session) || slam_identity::is_nil(ref.session_id)) {
    return {false, "missing_session_id"};
  }
  if (!slam_identity::equal(kf_session, ref.session_id)) {
    return {false, "session_mismatch"};
  }
  if (kf_epoch != ref.epoch_id) {
    return {false, "epoch_mismatch"};
  }
  if (keyframe_id == 0) {
    return {false, "missing_keyframe_id"};
  }
  if (!source_stamp_valid && !accept_missing_source_stamp) {
    return {false, "missing_source_stamp"};
  }
  if (img_rows != expected_rows || img_cols != expected_cols) {
    return {false, "calibration_resolution_mismatch"};
  }
  const uint64_t key = kf_dedupe_key(keyframe_id, stream);
  if (seen_ids.count(key) != 0) {
    return {false, "duplicate_keyframe"};
  }
  seen_ids.insert(key);
  return {true, ""};
}

}  // namespace globalslam

#endif  // GLOBALSLAM_G3_ADMISSION_HPP_
