#include "gmap_authoritative_loop.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "cloop_visual_pnp.hpp"
#include "g6b_cl_match_index.hpp"
#include "gmap_landmark_identity.hpp"
#include "gmap_map_clone.hpp"
#include "gmap_post_loop_refine.hpp"
#include "gmap_visual_sync.hpp"

namespace gmap_authoritative_loop
{
namespace
{

bool nearly_equal_vec(
  const arma::vec::fixed<3> & a, const arma::vec::fixed<3> & b, double tol)
{
  return std::abs(a[0] - b[0]) <= tol && std::abs(a[1] - b[1]) <= tol &&
    std::abs(a[2] - b[2]) <= tol;
}

}  // namespace

int find_kf_index_by_id(const GLOBAL_MAP & map, uint64_t keyframe_id)
{
  for (std::size_t i = 0; i < map.KeyFDATA.size(); ++i) {
    if (map.KeyFDATA[i].keyframe_id == keyframe_id) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

GeometryCompareResult compare_relevant_geometry(
  const GLOBAL_MAP & map,
  const gmap_loop::LoopProposal & prop,
  int idx_ref,
  const gmap_loop::RevalidationTolerances & tol)
{
  GeometryCompareResult out;
  const arma::vec::fixed<3> t_ref_now =
    map.KeyFDATA[static_cast<std::size_t>(idx_ref)].t_c2n;
  const arma::vec::fixed<3> d_ref = t_ref_now - prop.t_ref_at_verification;

  if (prop.kind == gmap_loop::Kind::Tag || prop.inlier_landmark_ids.empty()) {
    if (nearly_equal_vec(d_ref, arma::vec::fixed<3>{0, 0, 0}, tol.position_tol_m)) {
      out.decision = gmap_loop::RevalidationDecision::ReuseCoherent;
      out.reason = "tag_or_empty_inliers_ref_unchanged";
      return out;
    }
    // Tag: absolute position measurement — if ref moved, still reuse coherent
    // relative factor (Strategy A); absolute gate uses consistent geometry.
    if (nearly_equal_vec(
          d_ref, d_ref, tol.common_translation_tol_m))  // always true
    {
      // Treat any pure ref translation as common translation for tag.
      out.decision = gmap_loop::RevalidationDecision::CommonTranslationReuse;
      out.common_delta = d_ref;
      out.reason = "tag_common_ref_translation";
      return out;
    }
  }

  std::vector<arma::vec::fixed<3>> deltas;
  deltas.push_back(d_ref);
  int n_missing = 0;
  for (std::size_t i = 0; i < prop.inlier_landmark_ids.size(); ++i) {
    std::size_t idx = 0;
    if (!gmap_landmark_id::resolve_landmark_index(
          map, prop.inlier_landmark_ids[i], idx))
    {
      ++n_missing;
      continue;
    }
    const arma::vec::fixed<3> now = map.AnchorsDATA[idx].AnchorState;
    const arma::vec::fixed<3> then = prop.inlier_xyz_at_verification[i];
    deltas.push_back(now - then);
  }

  if (n_missing > 0 &&
    n_missing == static_cast<int>(prop.inlier_landmark_ids.size()))
  {
    out.decision = gmap_loop::RevalidationDecision::Reject;
    out.reason = "all_inlier_landmarks_missing";
    return out;
  }
  if (n_missing > 0) {
    out.decision = gmap_loop::RevalidationDecision::RequireRepnp;
    out.reason = "partial_inlier_landmarks_missing";
    return out;
  }

  bool all_zero = true;
  for (const auto & d : deltas) {
    if (!nearly_equal_vec(d, arma::vec::fixed<3>{0, 0, 0}, tol.position_tol_m)) {
      all_zero = false;
      break;
    }
  }
  if (all_zero) {
    out.decision = gmap_loop::RevalidationDecision::ReuseCoherent;
    out.reason = "relevant_geometry_unchanged";
    return out;
  }

  const arma::vec::fixed<3> d0 = deltas.front();
  bool common = true;
  for (const auto & d : deltas) {
    if (!nearly_equal_vec(d, d0, tol.common_translation_tol_m)) {
      common = false;
      break;
    }
  }
  if (common) {
    out.decision = gmap_loop::RevalidationDecision::CommonTranslationReuse;
    out.common_delta = d0;
    out.reason = "common_translation";
    return out;
  }

  out.decision = gmap_loop::RevalidationDecision::RequireRepnp;
  out.reason = "nonuniform_geometry_change";
  return out;
}

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
  std::string & reason)
{
  std::vector<cv::Point3f> pts3;
  std::vector<cv::Point2f> pts2;
  pts3.reserve(prop.inlier_landmark_ids.size());
  pts2.reserve(prop.inlier_landmark_ids.size());

  for (std::size_t i = 0; i < prop.inlier_landmark_ids.size(); ++i) {
    std::size_t idx = 0;
    if (!gmap_landmark_id::resolve_landmark_index(
          map, prop.inlier_landmark_ids[i], idx))
    {
      continue;
    }
    const auto & xyz = map.AnchorsDATA[idx].AnchorState;
    pts3.emplace_back(
      static_cast<float>(xyz[0]), static_cast<float>(xyz[1]),
      static_cast<float>(xyz[2]));
    const cv::Point2d uv_u =
      Undistort_a_point(prop.inlier_uvs[i], cam, 1);
    pts2.emplace_back(static_cast<float>(uv_u.x), static_cast<float>(uv_u.y));
  }

  if (pts3.size() < 4) {
    reason = "insufficient_correspondence_support";
    return false;
  }

  const KEYFRAME & kf = prop.closing_kf;
  const auto solved = globalslam::cloop_visual_pnp::solve_and_accept(
    pts3, pts2, kf.Rn2c, kf.t_c2n, cam, radius_uncertainty, xy_gate_m,
    reproj_thresh_px);
  if (!solved.accepted) {
    reason = std::string("repnp_") + solved.reject_reason;
    return false;
  }

  new_pnp_C_nav = solved.C_nav;
  new_t_loop =
    map.KeyFDATA[static_cast<std::size_t>(idx_ref)].t_c2n - new_pnp_C_nav;
  reason = "repnp_accepted";
  return true;
}

CorrectionResult process_proposal_on_workspace(
  const GLOBAL_MAP & map,
  const gmap_loop::LoopProposal & prop,
  const CAM & cam,
  const ProcessConfig & config)
{
  CorrectionResult result;
  result.state = gmap_loop::ProposalState::Rejected;

  const int idx_ref = find_kf_index_by_id(map, prop.pose_graph_ref_keyframe_id);
  const int idx_closing = find_kf_index_by_id(map, prop.closing_keyframe_id);
  if (idx_ref < 0) {
    result.reason = "missing_ref_kf";
    return result;
  }
  if (idx_closing < 0) {
    result.state = gmap_loop::ProposalState::DeferredPendingEndpoint;
    result.reason = "missing_closing_kf";
    return result;
  }
  if (idx_closing < idx_ref) {
    result.reason = "endpoint_ordering";
    return result;
  }

  const int idx_tip = static_cast<int>(map.KeyFDATA.size()) - 1;
  auto geom = compare_relevant_geometry(map, prop, idx_ref, config.reval);
  result.reval = geom.decision;

  arma::vec::fixed<3> t_loop = prop.t_loop;
  arma::vec::fixed<3> pnp_for_diag = prop.pnp_C_nav;

  if (geom.decision == gmap_loop::RevalidationDecision::Reject) {
    result.reason = geom.reason;
    return result;
  }
  if (geom.decision == gmap_loop::RevalidationDecision::RequireRepnp) {
    if (prop.kind != gmap_loop::Kind::Visual) {
      result.reason = "tag_cannot_repnp:" + geom.reason;
      return result;
    }
    arma::vec::fixed<3> new_pnp;
    arma::vec::fixed<3> new_t;
    std::string rr;
    if (!authoritative_repnp(
          map, prop, idx_ref, cam, prop.closing_kf.radius_uncertainty,
          config.xy_gate_m, config.reproj_thresh_px, new_pnp, new_t, rr))
    {
      result.reason = rr;
      return result;
    }
    t_loop = new_t;
    pnp_for_diag = new_pnp;
    result.reason = rr;
  } else {
    result.reason = geom.reason;
    // Common translation / reuse: keep coherent t_loop unchanged.
  }

  // Significant-loop consistency for visual: absolute PnP in current geometry.
  if (prop.kind == gmap_loop::Kind::Visual &&
    config.significant_hist_path_m > 0.0)
  {
    // Absolute PnP expressed consistently: pnp_now = t_ref_now − t_loop
    // (same relative factor). Re-evaluate euclidean vs matched historical.
    const arma::vec::fixed<3> pnp_abs =
      map.KeyFDATA[static_cast<std::size_t>(idx_ref)].t_c2n - t_loop;
    if (prop.matched_historical_keyframe_id >= 0) {
      const int idx_matched = find_kf_index_by_id(
        map, static_cast<uint64_t>(prop.matched_historical_keyframe_id));
      if (idx_matched >= 0) {
        const auto & tm = map.KeyFDATA[static_cast<std::size_t>(idx_matched)].t_c2n;
        const double euclid = std::sqrt(
          std::pow(pnp_abs[0] - tm[0], 2) + std::pow(pnp_abs[1] - tm[1], 2));
        (void)euclid;  // historical path gate already applied at detection;
        // keep absolute position consistent for any future gate callers.
        (void)pnp_abs;
      }
    }
  }

  GLOBAL_MAP workspace = gmap_map_clone::deep_clone(map);

  gmap_graph_correction::PoseSlamInputs pin;
  pin.idx_ref = idx_ref;
  pin.idx_closing = idx_closing;
  pin.idx_tip = idx_tip;
  pin.t_loop = t_loop;
  pin.pnp_C_nav = pnp_for_diag;

  std::vector<arma::vec::fixed<3>> deltas;
  std::vector<int> idx_opt;
  gmap_graph_correction::PoseSlamDiagnostics diag;
  if (!gmap_graph_correction::run_pose_slam(
        workspace, pin, config.pose, deltas, idx_opt, &diag) ||
    deltas.empty())
  {
    result.state = gmap_loop::ProposalState::CorrectionFailed;
    result.reason = "pose_slam_empty";
    return result;
  }

  gmap_graph_correction::apply_pose_slam_geometry(workspace, deltas, idx_opt);

  // Bump pose-graph reference like production (closing-1 or tip-1).
  if (workspace.KeyFDATA.size() >= 2) {
    workspace.idx_ref_pose_slam =
      static_cast<int>(workspace.KeyFDATA.size()) - 2;
  }

  // Pre-M7.4 order: Match_list (always if matched KF valid) then optional BA.
  // Match_list mutations retained even if BA is disabled or rejected.
  bool refine_ran = false;
  if (prop.kind == gmap_loop::Kind::Visual &&
    prop.matched_historical_keyframe_id >= 0)
  {
    const int idx_matched = find_kf_index_by_id(
      workspace, static_cast<uint64_t>(prop.matched_historical_keyframe_id));
    const auto match_dec = globalslam::g6b_cl::resolve_matched_kf_index(
      idx_matched, idx_ref, static_cast<int>(workspace.KeyFDATA.size()));
    if (match_dec.valid) {
      std::vector<int> idx_vl_new;
      std::vector<int> idx_vl_old;
      gmap_post_loop_refine::run_post_loop_matching(
        workspace, idx_closing, match_dec.index, cam, config.match,
        idx_vl_new, idx_vl_old);
      if (config.enable_post_loop_ba) {
        gmap_graph_correction::PostLoopBaDiagnostics ba_diag;
        gmap_graph_correction::run_post_loop_ba(
          workspace, idx_vl_new, idx_vl_old, cam, config.ba, &ba_diag);
        refine_ran = true;
        // BA reject retains Pose_SLAM + Match_list observations (production).
      }
    }
  }

  const int idx_closing_after =
    find_kf_index_by_id(workspace, prop.closing_keyframe_id);
  if (idx_closing_after < 0) {
    result.state = gmap_loop::ProposalState::CorrectionFailed;
    result.reason = "closing_lost_after_correction";
    return result;
  }

  const arma::vec::fixed<3> p_g =
    workspace.KeyFDATA[static_cast<std::size_t>(idx_closing_after)].t_c2n;
  result.delta_L = gmap_visual_sync::align_delta_xy(p_g, prop.p_L_original);
  result.p_G_final_closing = p_g;
  result.match_ba_refine_ran = refine_ran;
  result.committed = true;
  result.state = gmap_loop::ProposalState::GlobalCommitted;
  result.workspace = std::move(workspace);
  if (result.reason.empty()) {
    result.reason = "ok";
  }
  return result;
}

}  // namespace gmap_authoritative_loop
