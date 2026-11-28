/**
 * M7.6 test-only structural map invariants (extends M6 identity checks).
 * Does not change estimator behavior.
 */
#ifndef GMAP_STRUCTURAL_INVARIANTS_M7_6_HPP
#define GMAP_STRUCTURAL_INVARIANTS_M7_6_HPP

#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_set>

#include "gmap_landmark_identity.hpp"
#include "globalslam_types.hpp"

namespace gmap_struct_m76
{

struct Report
{
  bool ok{true};
  std::string reason;
};

inline Report check(const GLOBAL_MAP & map)
{
  Report r;
  const auto id = gmap_landmark_id::validate_landmark_identity_invariants(map);
  if (!id.ok) {
    r.ok = false;
    r.reason = id.reason;
    return r;
  }

  // KF identity uniqueness.
  std::unordered_set<uint64_t> kf_ids;
  for (const auto & kf : map.KeyFDATA) {
    if (!kf_ids.insert(kf.keyframe_id).second) {
      r.ok = false;
      r.reason = "duplicate_keyframe_id";
      return r;
    }
  }

  // Visibility graph dimensions match KF storage (when non-empty).
  if (!map.Vgraph.empty()) {
    if (map.Vgraph.n_rows != map.KeyFDATA.size() ||
      map.Vgraph.n_cols != map.KeyFDATA.size())
    {
      r.ok = false;
      r.reason = "vgraph_dim_mismatch";
      return r;
    }
  } else if (!map.KeyFDATA.empty() && map.KeyFDATA.size() > 1) {
    // Empty Vgraph with multi-KF is allowed only at init edge cases; treat as soft.
  }

  // No NaN/Inf in KF or landmark geometry.
  for (const auto & kf : map.KeyFDATA) {
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(kf.t_c2n[i])) {
        r.ok = false;
        r.reason = "nonfinite_kf_t_c2n";
        return r;
      }
    }
  }
  for (const auto & a : map.AnchorsDATA) {
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(a.AnchorState[i])) {
        r.ok = false;
        r.reason = "nonfinite_landmark_xyz";
        return r;
      }
    }
  }

  // idx_ref_pose_slam in range when KeyFDATA non-empty.
  if (!map.KeyFDATA.empty()) {
    if (map.idx_ref_pose_slam < 0 ||
      static_cast<std::size_t>(map.idx_ref_pose_slam) >= map.KeyFDATA.size())
    {
      r.ok = false;
      r.reason = "idx_ref_pose_slam_oob";
      return r;
    }
  }

  return r;
}

/** Exact LandmarkId set comparison (order-independent). */
inline std::unordered_set<uint64_t> landmark_id_set(const GLOBAL_MAP & map)
{
  std::unordered_set<uint64_t> s;
  for (const auto & a : map.AnchorsDATA) {
    s.insert(a.id.value);
  }
  return s;
}

inline std::unordered_set<uint64_t> keyframe_id_set(const GLOBAL_MAP & map)
{
  std::unordered_set<uint64_t> s;
  for (const auto & kf : map.KeyFDATA) {
    s.insert(kf.keyframe_id);
  }
  return s;
}

}  // namespace gmap_struct_m76

#endif
