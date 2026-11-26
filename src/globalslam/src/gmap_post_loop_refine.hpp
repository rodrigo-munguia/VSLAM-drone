/**
 * GSLAM-MAP-ARCH-M7.4.1: post-loop Match_list + visibility updates on an
 * explicit GLOBAL_MAP workspace (behavior from CLOOP::Update_gmap_with_new_matches
 * / Match_list_of_points_into_KF at fcb40e4).
 */
#ifndef GMAP_POST_LOOP_REFINE_HPP
#define GMAP_POST_LOOP_REFINE_HPP

#include <cstdint>
#include <vector>

#include "../../common/Vision/vision.hpp"
#include "globalslam_types.hpp"
#include "parameters.hpp"

namespace gmap_post_loop_refine
{

struct MatchConfig
{
  int vm_max_inov_error{20};
  int img_cols{640};
  int img_rows{480};
  /** Vgraph strength threshold (production Get_n_visually_linked_kf arg). */
  int min_vgraph_strength{10};
};

inline MatchConfig match_config_from_par(const parameters & PAR)
{
  MatchConfig c;
  c.vm_max_inov_error = PAR.VM_max_inov_error;
  c.img_cols = PAR.Mono_cam_img_cols;
  c.img_rows = PAR.Mono_cam_img_rows;
  return c;
}

std::vector<int> get_visually_linked_kf(
  const GLOBAL_MAP & map, int idx_kf, int min_strength);

/**
 * Descriptor+innovation match of landmark storage indices into KF.
 * Inserts Matched_landmark_ids/UV and increments n_kf_matched (production).
 */
void match_list_of_points_into_kf(
  GLOBAL_MAP & map,
  const CAM & cam,
  const MatchConfig & config,
  const std::vector<int64_t> & idx_points,
  int idx_KF,
  std::vector<int64_t> & idx_matched_points);

void update_visibility_graph_with_matched_points(
  GLOBAL_MAP & map,
  const std::vector<int64_t> & idx_points,
  int idx_KF);

/**
 * Bidirectional cross-match between closing neighborhood and matched-historical
 * neighborhood (production Update_gmap_with_new_matches).
 *
 * idx_closing is the original closing KF (not necessarily tip).
 * Fills idx_vl_new_kf / idx_vl_old_kf for subsequent post-loop BA.
 */
void run_post_loop_matching(
  GLOBAL_MAP & map,
  int idx_closing,
  int idx_matched_historical,
  const CAM & cam,
  const MatchConfig & config,
  std::vector<int> & idx_vl_new_kf,
  std::vector<int> & idx_vl_old_kf);

}  // namespace gmap_post_loop_refine

#endif
