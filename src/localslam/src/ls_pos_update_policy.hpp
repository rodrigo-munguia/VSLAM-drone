#ifndef LOCALSLAM_LS_POS_UPDATE_POLICY_HPP_
#define LOCALSLAM_LS_POS_UPDATE_POLICY_HPP_

/**
 * G3/G5 Local SLAM LSposUpdate admission policy (header-only, unit-tested).
 *
 * Validates identity/type/finiteness before queuing. Duplicate correction_id
 * within the same (session, epoch) is rejected so retries cannot apply twice.
 * correction_id is consumed at admission (terminal for this epoch even if
 * gated_noop). Re-validate under the same mutex that clears the queue on reset.
 *
 * At most one pending request: caller rejects with outcome=busy if the slot is
 * occupied (do not overwrite — that would drop a pending correction silently).
 *
 * Bookkeeping: applied_ids grows for the active (session, epoch) only and is
 * cleared on reset/epoch bump — never mid-epoch eviction (eviction would allow
 * a silent re-apply of an old correction_id). If the set hits
 * kMaxAppliedCorrectionIds, new admissions are rejected.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>

#include "../../common/slam_identity.hpp"

namespace localslam
{

/** Soft cap per Local SLAM epoch; reject rather than evict. */
inline constexpr std::size_t kMaxAppliedCorrectionIds = 4096;

struct PosUpdateRequestView
{
  double delta_x{0.0};
  double delta_y{0.0};
  double delta_z{0.0};
  std::string type;
  /** G6d origin; empty is accepted (inferred from type at consumers). */
  std::string source;
  slam_identity::UuidBytes session_id{slam_identity::nil_uuid()};
  uint64_t epoch_id{0};
  uint64_t keyframe_id{0};
  uint64_t correction_id{0};
};

inline bool is_finite_delta(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

inline bool is_known_correction_type(const std::string & type)
{
  return type == "bundle_adjustment" || type == "close_loop";
}

struct PosUpdateAdmitResult
{
  bool accepted{false};
  std::string reason;
};

/**
 * @param ref_valid current EstimatorReference.valid
 * @param active_session / active_epoch current Local SLAM identity
 * @param applied_ids correction_ids already accepted for this epoch (mutated on accept)
 */
inline PosUpdateAdmitResult admit_pos_update(
  const PosUpdateRequestView & req,
  bool ref_valid,
  const slam_identity::UuidBytes & active_session,
  uint64_t active_epoch,
  std::unordered_set<uint64_t> & applied_ids)
{
  if (!ref_valid) {
    return {false, "invalid_reference"};
  }
  if (slam_identity::is_nil(req.session_id) || slam_identity::is_nil(active_session)) {
    return {false, "missing_session_id"};
  }
  if (!slam_identity::equal(req.session_id, active_session)) {
    return {false, "session_mismatch"};
  }
  if (req.epoch_id != active_epoch) {
    return {false, "epoch_mismatch"};
  }
  if (!is_known_correction_type(req.type)) {
    return {false, "unknown_type"};
  }
  if (!is_finite_delta(req.delta_x, req.delta_y, req.delta_z)) {
    return {false, "non_finite_delta"};
  }
  if (req.delta_x == 0.0 && req.delta_y == 0.0 && req.delta_z == 0.0) {
    return {false, "zero_delta"};
  }
  if (req.correction_id == 0) {
    return {false, "missing_correction_id"};
  }
  if (applied_ids.count(req.correction_id) != 0) {
    return {false, "duplicate_correction_id"};
  }
  // Warn before hard capacity; never evict (eviction would re-enable duplicates).
  if (applied_ids.size() + 1 == kMaxAppliedCorrectionIds) {
    // Caller should log; reason empty on accept. Soft signal via size check.
  }
  if (applied_ids.size() >= kMaxAppliedCorrectionIds) {
    return {false, "correction_bookkeeping_full"};
  }
  applied_ids.insert(req.correction_id);
  return {true, ""};
}

/** True when the next accept would fill the epoch bookkeeping set. */
inline bool bookkeeping_near_capacity(const std::unordered_set<uint64_t> & applied_ids)
{
  return applied_ids.size() + 1 >= kMaxAppliedCorrectionIds;
}

}  // namespace localslam

#endif  // LOCALSLAM_LS_POS_UPDATE_POLICY_HPP_
