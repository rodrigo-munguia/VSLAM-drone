/**
 * GSLAM-LC-CONSISTENCY-A3: discrete visual-loop XY synchronization helpers.
 *
 * Alignment feedback (exported to LocalSLAM):
 *   δ_L.xy = p_G_final.xy − p_L_original.xy
 *   δ_L.z  = 0
 *
 * p_L_original is the Local EKF camera centre on the closing KEYFRAME before
 * Pose_SLAM mutates KeyFDATA (CLOOP does not run relative/hybrid rewrite on
 * kf_cl). p_G_final is tip t_c2n after Pose_SLAM + optional post-loop BA.
 *
 * One-time prev_ekf XY rebase after confirmed Local apply; admit watermark
 * uses last Local-generated keyframe_id (not last Global-admitted id).
 */
#ifndef GMAP_VISUAL_SYNC_HPP_
#define GMAP_VISUAL_SYNC_HPP_

#include <cstdint>

#include <armadillo>

#include "gmap_relative_kf_init.hpp"

namespace gmap_visual_sync
{

/** XY alignment translation; Z forced to 0. */
inline arma::vec::fixed<3> align_delta_xy(
  const arma::vec::fixed<3> & p_g_final,
  const arma::vec::fixed<3> & p_l_original)
{
  arma::vec::fixed<3> d;
  d[0] = p_g_final[0] - p_l_original[0];
  d[1] = p_g_final[1] - p_l_original[1];
  d[2] = 0.0;
  return d;
}

/** Legacy incremental tip correction (characterization / contrast). */
inline arma::vec::fixed<3> incremental_delta_xy(
  const arma::vec::fixed<3> & p_g_after,
  const arma::vec::fixed<3> & p_g_before)
{
  arma::vec::fixed<3> d;
  d[0] = p_g_after[0] - p_g_before[0];
  d[1] = p_g_after[1] - p_g_before[1];
  d[2] = 0.0;
  return d;
}

inline void rebase_cam_pose_xy(
  gmap_relative_kf_init::CamPose & pose, double dx, double dy)
{
  pose.t_c2n[0] += dx;
  pose.t_c2n[1] += dy;
}

enum class AdmitMode : uint8_t
{
  Open = 0,
  /** LC feedback in flight — drop all map admits until apply outcome. */
  Suspend = 1,
  /** After successful apply — drop keyframe_id <= watermark. */
  Bound = 2,
};

inline bool should_admit_keyframe(
  AdmitMode mode, uint64_t watermark, uint64_t keyframe_id)
{
  switch (mode) {
    case AdmitMode::Open:
      return true;
    case AdmitMode::Suspend:
      return false;
    case AdmitMode::Bound:
      return keyframe_id > watermark;
  }
  return true;
}

}  // namespace gmap_visual_sync

#endif
