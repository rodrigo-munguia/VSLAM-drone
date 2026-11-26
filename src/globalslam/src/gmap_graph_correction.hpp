/**
 * GSLAM-MAP-ARCH-M7.3: reusable Pose_SLAM + landmark δ + post-loop BA.
 *
 * Operates on an explicitly supplied GLOBAL_MAP. No CLOOP/GMAP worker ownership.
 * Numerical algorithms are behavior-preserving extractions of the prior CLOOP
 * implementations (cloop_pose_slam.cpp / cloop_bundle_adjustment.cpp).
 */
#ifndef GMAP_GRAPH_CORRECTION_HPP
#define GMAP_GRAPH_CORRECTION_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <armadillo>

#include "../../common/Vision/vision.hpp"
#include "globalslam_types.hpp"
#include "parameters.hpp"

namespace gmap_graph_correction
{

struct PoseSlamConfig
{
  double clo_sigma_kf_att{0.00001};
  double clo_sigma_kf_xy{0.005};
  double clo_sigma_kf_z{0.005};
  double odo_sigma_kf_att{0.0001};
  double odo_sigma_kf_xy{0.05};
  double odo_sigma_kf_z{0.05};
  bool update_z{true};
};

inline PoseSlamConfig pose_slam_config_from_par(const parameters & PAR)
{
  PoseSlamConfig c;
  c.clo_sigma_kf_att = PAR.CL_clo_sigma_kf_att;
  c.clo_sigma_kf_xy = PAR.CL_clo_sigma_kf_xy;
  c.clo_sigma_kf_z = PAR.CL_clo_sigma_kf_z;
  c.odo_sigma_kf_att = PAR.CL_odo_sigma_kf_att;
  c.odo_sigma_kf_xy = PAR.CL_odo_sigma_kf_xy;
  c.odo_sigma_kf_z = PAR.CL_odo_sigma_kf_z;
  c.update_z = PAR.CL_update_z;
  return c;
}

struct PoseSlamDiagnostics
{
  bool ok{false};
  int idx_kf_ref{-1};
  int idx_closing{-1};
  int idx_tip{-1};
  double initial_error{0};
  double final_error{0};
  arma::vec::fixed<3> t_loop{};
  arma::vec::fixed<3> pnp_C_nav{};
  std::vector<int> variable_indices;
  /** Factor summary lines: "prior x<i>" / "odo x<a>-x<b> tx ty tz" / "loop x_c-x_r tx ty tz" */
  std::vector<std::string> factor_lines;
};

/**
 * Explicit Pose_SLAM inputs (M7.4).
 * Graph variables: [idx_ref … idx_tip] inclusive (tip may be after closing).
 * Loop factor: Between(x_closing, x_ref, Pose3(I, t_loop)) — original endpoints.
 */
struct PoseSlamInputs
{
  int idx_ref{-1};
  int idx_closing{-1};
  int idx_tip{-1};
  /** Coherent loop translation (do not rebuild from mixed-time map state). */
  arma::vec::fixed<3> t_loop{};
  /** Optional provenance only (diagnostics); not used to rebuild t_loop. */
  arma::vec::fixed<3> pnp_C_nav{};
};

/**
 * Build R≡I pose graph with explicit endpoints and coherent t_loop.
 * Returns KF deltas for [idx_ref … idx_tip]; does not mutate map.
 */
bool run_pose_slam(
  const GLOBAL_MAP & map,
  const PoseSlamInputs & inputs,
  const PoseSlamConfig & config,
  std::vector<arma::vec::fixed<3>> & delta_kf_pos,
  std::vector<int> & idx_kf_opt,
  PoseSlamDiagnostics * diag = nullptr);

/**
 * M7.3 compatibility: closing = tip, t_loop = t_ref − pnp_C_nav,
 * window = [idx_ref_pose_slam … tip].
 */
bool run_pose_slam(
  const GLOBAL_MAP & map,
  const arma::vec::fixed<3> & pnp_C_nav,
  const PoseSlamConfig & config,
  std::vector<arma::vec::fixed<3>> & delta_kf_pos,
  std::vector<int> & idx_kf_opt,
  PoseSlamDiagnostics * diag = nullptr);

/** Apply Pose_SLAM KF deltas and init_KF landmark translation (production rule). */
void apply_pose_slam_geometry(
  GLOBAL_MAP & map,
  const std::vector<arma::vec::fixed<3>> & delta_kf_pos,
  const std::vector<int> & idx_kf_opt);

struct PostLoopBaConfig
{
  double sigma_uv{1.0};
  double sigma_kf_att{0.01};
  double sigma_kf_xy{0.05};
  double sigma_kf_z{0.05};
  double sigma_kf_pt{0.1};
  double max_delta_kf_pos{1.75};
  bool update_kf_pos{true};
  bool update_kf_att{true};
  bool update_map{true};
};

inline PostLoopBaConfig post_loop_ba_config_from_par(const parameters & PAR)
{
  PostLoopBaConfig c;
  c.sigma_uv = PAR.BA_sigma_uv;
  c.sigma_kf_att = PAR.BA_sigma_kf_att;
  c.sigma_kf_xy = PAR.BA_sigma_kf_xy;
  c.sigma_kf_z = PAR.BA_sigma_kf_z;
  c.sigma_kf_pt = PAR.BA_sigma_kf_pt;
  c.max_delta_kf_pos = PAR.BA_max_delta_kf_pos;
  c.update_kf_pos = PAR.BA_update_kf_pos;
  c.update_kf_att = PAR.BA_update_kf_att;
  c.update_map = PAR.BA_update_map;
  return c;
}

struct PostLoopBaDiagnostics
{
  bool accepted{false};
  double initial_error{0};
  double final_error{0};
  std::vector<int> idx_kf_optimized;
  std::vector<int64_t> idx_points;
  int n_projection_factors{0};
};

/**
 * Post-loop local BA (former CLOOP::Update_gmap_with_BA body).
 * Mutates map on acceptance; returns whether geometry was written back.
 */
bool run_post_loop_ba(
  GLOBAL_MAP & map,
  const std::vector<int> & idx_vl_new_kf,
  const std::vector<int> & idx_vl_old_kf,
  const CAM & cam,
  const PostLoopBaConfig & config,
  PostLoopBaDiagnostics * diag = nullptr);

}  // namespace gmap_graph_correction

#endif
