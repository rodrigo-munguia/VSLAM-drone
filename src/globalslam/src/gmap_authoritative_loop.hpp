/**
 * GSLAM-MAP-ARCH-M7.4: authoritative proposal validation + correction transaction.
 * Operates on an explicitly supplied GLOBAL_MAP (GMAP-owned workspace/commit).
 */
#ifndef GMAP_AUTHORITATIVE_LOOP_HPP
#define GMAP_AUTHORITATIVE_LOOP_HPP

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gmap_graph_correction.hpp"
#include "gmap_loop_proposal.hpp"
#include "gmap_post_loop_refine.hpp"
#include "globalslam_types.hpp"
#include "parameters.hpp"
#include "../../common/Vision/vision.hpp"

namespace gmap_authoritative_loop
{

struct ProcessConfig
{
  gmap_graph_correction::PoseSlamConfig pose;
  gmap_graph_correction::PostLoopBaConfig ba;
  gmap_post_loop_refine::MatchConfig match;
  gmap_loop::RevalidationTolerances reval;
  bool enable_post_loop_ba{true};
  double xy_gate_m{0.5};
  double reproj_thresh_px{25.0};
  double significant_hist_path_m{0.0};  // 0 = skip re-eval (use proposal as verified)
};

inline ProcessConfig config_from_par(const parameters & PAR)
{
  ProcessConfig c;
  c.pose = gmap_graph_correction::pose_slam_config_from_par(PAR);
  c.ba = gmap_graph_correction::post_loop_ba_config_from_par(PAR);
  c.match = gmap_post_loop_refine::match_config_from_par(PAR);
  c.enable_post_loop_ba = PAR.Enable_global_ba;
  c.xy_gate_m = PAR.CL_xy_update_max_delta;
  c.reproj_thresh_px = PAR.CL_min_mean_reprojection_error;
  c.significant_hist_path_m = PAR.CL_min_historical_travel_m;
  return c;
}

/** Resolve keyframe_id → storage index; -1 if missing. */
int find_kf_index_by_id(const GLOBAL_MAP & map, uint64_t keyframe_id);

struct GeometryCompareResult
{
  gmap_loop::RevalidationDecision decision{
    gmap_loop::RevalidationDecision::Reject};
  std::string reason;
  arma::vec::fixed<3> common_delta{};
};

GeometryCompareResult compare_relevant_geometry(
  const GLOBAL_MAP & map,
  const gmap_loop::LoopProposal & prop,
  int idx_ref,
  const gmap_loop::RevalidationTolerances & tol);

/**
 * Attempt authoritative re-PnP using retained LandmarkId+UV against current map.
 * On success fills new_pnp_C_nav and new_t_loop = t_ref_now − new_pnp.
 */
bool authoritative_repnp(
  const GLOBAL_MAP & map,
  const gmap_loop::LoopProposal & prop,
  int idx_ref,
  const CAM & cam,
  double radius_uncertainty,
  double xy_gate_m,
  double reproj_thresh_px,
  arma::vec::fixed<3> & new_pnp_C_nav,
  arma::vec::fixed<3> & new_t_loop,
  std::string & reason);

struct CorrectionResult
{
  bool committed{false};
  gmap_loop::ProposalState state{gmap_loop::ProposalState::Rejected};
  std::string reason;
  arma::vec::fixed<3> delta_L{};
  arma::vec::fixed<3> p_G_final_closing{};
  bool match_ba_refine_ran{false};
  gmap_loop::RevalidationDecision reval{
    gmap_loop::RevalidationDecision::Reject};
  /** Populated on success: corrected workspace ready to commit. */
  std::optional<GLOBAL_MAP> workspace;
};

/**
 * Validate + correct on a deep-cloned workspace of `map`.
 * Does NOT mutate `map`. On success, result.workspace holds corrected geometry.
 */
CorrectionResult process_proposal_on_workspace(
  const GLOBAL_MAP & map,
  const gmap_loop::LoopProposal & prop,
  const CAM & cam,
  const ProcessConfig & config);

}  // namespace gmap_authoritative_loop

#endif
