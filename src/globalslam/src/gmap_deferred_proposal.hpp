/**
 * GSLAM-MAP-ARCH-M7.4.1: deferred closing-KF proposal lifecycle policy.
 *
 * Bound: active GMAP processing opportunities while the closing KF is neither
 * admitted nor present in the ordinary KF buffer, excluding Suspend cycles.
 */
#ifndef GMAP_DEFERRED_PROPOSAL_HPP
#define GMAP_DEFERRED_PROPOSAL_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "../../common/keyframe_types.hpp"
#include "gmap_visual_sync.hpp"
#include "globalslam_types.hpp"

namespace gmap_deferred_proposal
{

/** Active (non-Suspend) process opportunities before terminal reject. */
inline constexpr int kMaxActiveDeferCycles = 64;

enum class ClosingStatus : uint8_t
{
  Admitted = 0,
  PendingInBuffer = 1,
  WatermarkInvalid = 2,
  ObsoleteLifetime = 3,
  Unresolved = 4,
};

inline const char * status_name(ClosingStatus s)
{
  switch (s) {
    case ClosingStatus::Admitted: return "admitted";
    case ClosingStatus::PendingInBuffer: return "pending_in_buffer";
    case ClosingStatus::WatermarkInvalid: return "watermark_invalid";
    case ClosingStatus::ObsoleteLifetime: return "obsolete_lifetime";
    case ClosingStatus::Unresolved: return "unresolved";
  }
  return "unknown";
}

struct DeferDecision
{
  enum class Action : uint8_t { Process, Defer, Reject } action{Action::Defer};
  ClosingStatus status{ClosingStatus::Unresolved};
  std::string reason;
  int active_cycles_after{0};
};

inline bool buffer_contains_kf_id(
  const std::vector<KEYFRAME> & buffer, uint64_t keyframe_id)
{
  for (const auto & kf : buffer) {
    if (kf.keyframe_id == keyframe_id) {
      return true;
    }
  }
  return false;
}

inline bool map_contains_kf_id(const GLOBAL_MAP & map, uint64_t keyframe_id)
{
  for (const auto & kf : map.KeyFDATA) {
    if (kf.keyframe_id == keyframe_id) {
      return true;
    }
  }
  return false;
}

inline uint64_t max_keyframe_id_in_map(const GLOBAL_MAP & map)
{
  uint64_t m = 0;
  for (const auto & kf : map.KeyFDATA) {
    if (kf.keyframe_id > m) {
      m = kf.keyframe_id;
    }
  }
  return m;
}

/**
 * Decide defer/process/reject for a proposal whose closing KF may be missing.
 *
 * @param active_cycles  Prior count of active unresolved deferrals for this proposal.
 * @param suspend        True if admit mode is Suspend (do not increment cycles).
 */
inline DeferDecision evaluate_closing_endpoint(
  uint64_t closing_keyframe_id,
  uint64_t proposal_lifetime,
  uint64_t current_lifetime,
  const GLOBAL_MAP & auth_map,
  const std::vector<KEYFRAME> & kf_buffer,
  gmap_visual_sync::AdmitMode admit_mode,
  uint64_t sync_watermark,
  int active_cycles,
  bool suspend)
{
  DeferDecision d;
  d.active_cycles_after = active_cycles;

  if (proposal_lifetime != current_lifetime) {
    d.action = DeferDecision::Action::Reject;
    d.status = ClosingStatus::ObsoleteLifetime;
    d.reason = "obsolete_map_lifetime";
    return d;
  }

  if (map_contains_kf_id(auth_map, closing_keyframe_id)) {
    d.action = DeferDecision::Action::Process;
    d.status = ClosingStatus::Admitted;
    d.reason = "closing_admitted";
    d.active_cycles_after = 0;
    return d;
  }

  if (admit_mode == gmap_visual_sync::AdmitMode::Bound &&
    closing_keyframe_id <= sync_watermark)
  {
    d.action = DeferDecision::Action::Reject;
    d.status = ClosingStatus::WatermarkInvalid;
    d.reason = "closing_kf_obsolete_under_watermark";
    return d;
  }

  if (buffer_contains_kf_id(kf_buffer, closing_keyframe_id)) {
    d.action = DeferDecision::Action::Defer;
    d.status = ClosingStatus::PendingInBuffer;
    d.reason = "closing_kf_pending_in_buffer";
    // Legitimately pending — do not advance starvation counter.
    return d;
  }

  // Unresolved: not admitted, not in buffer.
  if (suspend) {
    d.action = DeferDecision::Action::Defer;
    d.status = ClosingStatus::Unresolved;
    d.reason = "defer_while_sync_suspend";
    return d;
  }

  d.active_cycles_after = active_cycles + 1;
  if (d.active_cycles_after > kMaxActiveDeferCycles) {
    d.action = DeferDecision::Action::Reject;
    d.status = ClosingStatus::Unresolved;
    d.reason = "deferred_closing_kf_timeout_active_cycles";
    return d;
  }

  // Higher admitted IDs without buffer presence strongly suggests the closing
  // KF will never arrive on the map stream; still require the bound above so
  // short reordering windows do not reject prematurely.
  d.action = DeferDecision::Action::Defer;
  d.status = ClosingStatus::Unresolved;
  d.reason = "closing_kf_absent_defer";
  return d;
}

}  // namespace gmap_deferred_proposal

#endif
