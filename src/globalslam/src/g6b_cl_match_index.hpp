#ifndef GLOBALSLAM_G6B_CL_MATCH_INDEX_HPP_
#define GLOBALSLAM_G6B_CL_MATCH_INDEX_HPP_

/**
 * G6b / G6b.2: resolve the "matched" keyframe index used after Pose_SLAM when
 * updating the global map with new visual matches / local BA.
 *
 * Home-tag Step2 can close a loop without Get_matches ever assigning
 * idx_kf_matched. Using that uninitialized index after Pose_SLAM prints
 * is a demonstrated SIGSEGV (exit -11) in Update_gmap_with_new_matches.
 *
 * Policy (G6b.2):
 * - If visual match index is in [0, n_keyframes), use it — it is a
 *   corresponding observation of the same map region.
 * - Do NOT fall back to pose-graph gauge (idx_ref_pose_slam): that index is
 *   only a translation prior anchor, not a corresponding visual reference.
 * - Else no valid index (caller must skip match/BA refinement; Pose_SLAM
 *   results already applied).
 */

#include <cstddef>
#include <cstdint>

namespace globalslam
{
namespace g6b_cl
{

struct MatchIndexDecision
{
  bool valid{false};
  int index{-1};
  const char * reason{"unset"};
};

inline MatchIndexDecision resolve_matched_kf_index(
  int idx_kf_matched_from_visual,
  int /*idx_ref_pose_slam*/,
  std::size_t n_keyframes)
{
  MatchIndexDecision d;
  if (n_keyframes == 0) {
    d.reason = "empty_map";
    return d;
  }
  const int n = static_cast<int>(n_keyframes);
  // Matched KF must be an existing map KF (allow any [0,n) ).
  if (idx_kf_matched_from_visual >= 0 && idx_kf_matched_from_visual < n) {
    d.valid = true;
    d.index = idx_kf_matched_from_visual;
    d.reason = "visual_match";
    return d;
  }
  d.reason = "no_corresponding_visual_match";
  return d;
}

}  // namespace g6b_cl
}  // namespace globalslam

#endif  // GLOBALSLAM_G6B_CL_MATCH_INDEX_HPP_
