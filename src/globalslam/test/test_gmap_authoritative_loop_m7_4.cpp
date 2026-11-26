/**
 * GSLAM-MAP-ARCH-M7.4: authoritative loop proposal / extended graph / content.
 */
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "gmap_authoritative_loop.hpp"
#include "gmap_graph_correction.hpp"
#include "gmap_landmark_identity.hpp"
#include "gmap_loop_proposal.hpp"
#include "gmap_map_clone.hpp"
#include "gmap_visual_sync.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/keyframe_types.hpp"

using gmap_landmark_id::insert_landmark;
using gmap_landmark_id::rebuild_id_to_index;
using gmap_landmark_id::erase_landmark_at_storage_index;
using gmap_landmark_id::validate_landmark_identity_invariants;

namespace
{

constexpr double kTol = 1e-9;
constexpr double kObjTol = 1e-8;

KEYFRAME make_kf(double x, double y, double z, uint64_t id)
{
  KEYFRAME kf;
  kf.t_c2n = {x, y, z};
  kf.Rn2c = arma::eye<arma::mat>(3, 3);
  kf.Rr2c = arma::eye<arma::mat>(3, 3);
  kf.Rn2r = arma::eye<arma::mat>(3, 3);
  kf.keyframe_id = id;
  kf.radius_uncertainty = 1.0;
  return kf;
}

G_ANCHOR make_anchor(double x, double y, double z, int init_kf)
{
  G_ANCHOR a;
  a.feat_type = "g_anchor";
  a.AnchorState = {x, y, z};
  a.init_KF = init_kf;
  a.n_kf_matched = 3;
  a.Descriptor = cv::Mat::zeros(1, 32, CV_8U);
  a.Descriptor.at<unsigned char>(0, 0) = 7;
  return a;
}

CAM make_cam(std::vector<double> & dist)
{
  dist = {0, 0, 0, 0, 0};
  CAM cam;
  cam.distortions = dist.data();
  cam.cc[0] = 320;
  cam.cc[1] = 240;
  cam.fc[0] = 500;
  cam.fc[1] = 500;
  cam.alpha_c = 0;
  return cam;
}

/** Closing equals tip — M7.3-compatible topology. */
GLOBAL_MAP make_tip_equals_closing_map()
{
  GLOBAL_MAP map;
  map.idx_ref_pose_slam = 1;
  map.KeyFDATA.push_back(make_kf(0.0, 0.0, -1.0, 100));
  map.KeyFDATA.push_back(make_kf(1.0, 0.0, -1.0, 101));
  map.KeyFDATA.push_back(make_kf(2.0, 0.1, -1.0, 102));
  map.KeyFDATA.push_back(make_kf(3.0, 0.0, -1.0, 103));
  map.KeyFDATA.push_back(make_kf(0.2, 0.05, -1.0, 104));
  map.next_landmark_id = 100;
  insert_landmark(map, make_anchor(0.5, 0.0, 2.0, 0));
  insert_landmark(map, make_anchor(1.5, 0.0, 2.0, 2));
  insert_landmark(map, make_anchor(2.5, 0.1, 2.0, 3));
  rebuild_id_to_index(map);
  return map;
}

/** Closing older than tip (extended graph). */
GLOBAL_MAP make_extended_graph_map()
{
  GLOBAL_MAP map = make_tip_equals_closing_map();
  // Tip KFs after closing 104
  map.KeyFDATA.push_back(make_kf(0.4, 0.1, -1.0, 105));
  map.KeyFDATA.push_back(make_kf(0.6, 0.15, -1.0, 106));
  const auto id_new = insert_landmark(map, make_anchor(0.5, 0.1, 2.0, 5));
  map.KeyFDATA[5].Matched_landmark_ids = {id_new};
  map.KeyFDATA[5].UV_Matched_points = {{330, 250}};
  map.KeyFDATA[6].Matched_landmark_ids = {id_new};
  map.KeyFDATA[6].UV_Matched_points = {{335, 252}};
  rebuild_id_to_index(map);
  return map;
}

gmap_loop::LoopProposal make_prop_from_map(
  const GLOBAL_MAP & map, uint64_t closing_id, const arma::vec::fixed<3> & pnp)
{
  gmap_loop::LoopProposal p;
  p.proposal_id = 1;
  p.map_lifetime = 0;
  p.correction_epoch = 0;
  p.closing_keyframe_id = closing_id;
  p.pose_graph_ref_keyframe_id =
    map.KeyFDATA[static_cast<std::size_t>(map.idx_ref_pose_slam)].keyframe_id;
  p.pnp_C_nav = pnp;
  const int idx_c =
    gmap_authoritative_loop::find_kf_index_by_id(map, closing_id);
  p.p_L_original = map.KeyFDATA[static_cast<std::size_t>(idx_c)].t_c2n;
  p.t_ref_at_verification =
    map.KeyFDATA[static_cast<std::size_t>(map.idx_ref_pose_slam)].t_c2n;
  p.t_loop = p.t_ref_at_verification - p.pnp_C_nav;
  p.kind = gmap_loop::Kind::Tag;
  p.source_label = "tag_loop";
  p.closing_kf = map.KeyFDATA[static_cast<std::size_t>(idx_c)];
  return p;
}

gmap_graph_correction::PoseSlamConfig pose_cfg()
{
  return gmap_graph_correction::PoseSlamConfig{};
}

}  // namespace

TEST(AuthoritativeLoopM74, CoherentMeasurementMatchesM73WhenTipEqualsClosing)
{
  GLOBAL_MAP map = make_tip_equals_closing_map();
  const arma::vec::fixed<3> pnp = {0.15, 0.02, -1.0};
  const arma::vec::fixed<3> t_loop =
    map.KeyFDATA[static_cast<std::size_t>(map.idx_ref_pose_slam)].t_c2n - pnp;

  std::vector<arma::vec::fixed<3>> d_legacy, d_ex;
  std::vector<int> i_legacy, i_ex;
  gmap_graph_correction::PoseSlamDiagnostics a, b;
  ASSERT_TRUE(gmap_graph_correction::run_pose_slam(
    map, pnp, pose_cfg(), d_legacy, i_legacy, &a));

  gmap_graph_correction::PoseSlamInputs in;
  in.idx_ref = map.idx_ref_pose_slam;
  in.idx_closing = static_cast<int>(map.KeyFDATA.size()) - 1;
  in.idx_tip = in.idx_closing;
  in.t_loop = t_loop;
  in.pnp_C_nav = pnp;
  ASSERT_TRUE(gmap_graph_correction::run_pose_slam(
    map, in, pose_cfg(), d_ex, i_ex, &b));

  EXPECT_NEAR(a.initial_error, b.initial_error, kObjTol);
  EXPECT_NEAR(a.final_error, b.final_error, kObjTol);
  ASSERT_EQ(d_legacy.size(), d_ex.size());
  for (std::size_t i = 0; i < d_legacy.size(); ++i) {
    EXPECT_NEAR(d_legacy[i][0], d_ex[i][0], kTol);
    EXPECT_NEAR(d_legacy[i][1], d_ex[i][1], kTol);
    EXPECT_NEAR(d_legacy[i][2], d_ex[i][2], kTol);
  }
  EXPECT_EQ(b.idx_closing, in.idx_closing);
}

TEST(AuthoritativeLoopM74, CommonTranslationReusesCoherentMeasurement)
{
  GLOBAL_MAP map = make_tip_equals_closing_map();
  auto prop = make_prop_from_map(map, 104, {0.15, 0.02, -1.0});
  // Translate ref + all landmarks by same delta.
  const arma::vec::fixed<3> Delta = {3.0, -2.0, 0.5};
  map.KeyFDATA[static_cast<std::size_t>(map.idx_ref_pose_slam)].t_c2n += Delta;
  for (auto & a : map.AnchorsDATA) {
    a.AnchorState += Delta;
  }
  // Add inlier evidence matching translated landmarks.
  prop.inlier_landmark_ids.push_back(map.AnchorsDATA[0].id);
  prop.inlier_xyz_at_verification.push_back(
    map.AnchorsDATA[0].AnchorState - Delta);
  prop.inlier_uvs.push_back({320, 240});
  prop.kind = gmap_loop::Kind::Visual;

  const int idx_ref = map.idx_ref_pose_slam;
  auto cmp = gmap_authoritative_loop::compare_relevant_geometry(
    map, prop, idx_ref, {});
  EXPECT_EQ(cmp.decision,
    gmap_loop::RevalidationDecision::CommonTranslationReuse);
  EXPECT_NEAR(cmp.common_delta[0], Delta[0], 1e-6);
}

TEST(AuthoritativeLoopM74, ExtendedGraphKeepsOriginalEndpoints)
{
  GLOBAL_MAP map = make_extended_graph_map();
  const arma::vec::fixed<3> pnp = {0.15, 0.02, -1.0};
  const int idx_ref = map.idx_ref_pose_slam;
  const int idx_closing =
    gmap_authoritative_loop::find_kf_index_by_id(map, 104);
  ASSERT_EQ(idx_closing, 4);
  ASSERT_EQ(static_cast<int>(map.KeyFDATA.size()) - 1, 6);

  gmap_graph_correction::PoseSlamInputs in;
  in.idx_ref = idx_ref;
  in.idx_closing = idx_closing;
  in.idx_tip = static_cast<int>(map.KeyFDATA.size()) - 1;
  in.t_loop = map.KeyFDATA[static_cast<std::size_t>(idx_ref)].t_c2n - pnp;
  in.pnp_C_nav = pnp;

  std::vector<arma::vec::fixed<3>> deltas;
  std::vector<int> idx_opt;
  gmap_graph_correction::PoseSlamDiagnostics diag;
  ASSERT_TRUE(gmap_graph_correction::run_pose_slam(
    map, in, pose_cfg(), deltas, idx_opt, &diag));
  EXPECT_EQ(diag.idx_closing, idx_closing);
  EXPECT_EQ(diag.idx_tip, 6);
  EXPECT_EQ(diag.variable_indices.front(), idx_ref);
  EXPECT_EQ(diag.variable_indices.back(), 6);
  // Loop factor line references original endpoints.
  ASSERT_FALSE(diag.factor_lines.empty());
  EXPECT_NE(diag.factor_lines.back().find("loop x4-x1"), std::string::npos);

  GLOBAL_MAP before = gmap_map_clone::deep_clone(map);
  gmap_graph_correction::apply_pose_slam_geometry(map, deltas, idx_opt);
  // Newer landmark (init_KF=5) must move with KF5 delta.
  const auto it5 = std::find(idx_opt.begin(), idx_opt.end(), 5);
  ASSERT_NE(it5, idx_opt.end());
  std::size_t li = 0;
  bool found = false;
  for (std::size_t i = 0; i < map.AnchorsDATA.size(); ++i) {
    if (map.AnchorsDATA[i].init_KF == 5) {
      li = i;
      found = true;
      break;
    }
  }
  ASSERT_TRUE(found);
  const int di = static_cast<int>(it5 - idx_opt.begin());
  EXPECT_NEAR(
    map.AnchorsDATA[li].AnchorState[0],
    before.AnchorsDATA[li].AnchorState[0] + deltas[static_cast<std::size_t>(di)][0],
    kTol);
}

TEST(AuthoritativeLoopM74, UnrelatedGrowthDoesNotInvalidate)
{
  GLOBAL_MAP map = make_tip_equals_closing_map();
  auto prop = make_prop_from_map(map, 104, {0.15, 0.02, -1.0});
  map.KeyFDATA.push_back(make_kf(9, 9, -1, 999));
  insert_landmark(map, make_anchor(9, 9, 9, 0));
  rebuild_id_to_index(map);
  auto cmp = gmap_authoritative_loop::compare_relevant_geometry(
    map, prop, map.idx_ref_pose_slam, {});
  EXPECT_EQ(cmp.decision, gmap_loop::RevalidationDecision::ReuseCoherent);
}

TEST(AuthoritativeLoopM74, MissingClosingDefers)
{
  GLOBAL_MAP map = make_tip_equals_closing_map();
  auto prop = make_prop_from_map(map, 104, {0.15, 0.02, -1.0});
  prop.closing_keyframe_id = 777;  // not present
  std::vector<double> dist;
  CAM cam = make_cam(dist);
  auto cfg = gmap_authoritative_loop::ProcessConfig{};
  cfg.enable_post_loop_ba = false;
  auto r = gmap_authoritative_loop::process_proposal_on_workspace(
    map, prop, cam, cfg);
  EXPECT_EQ(r.state, gmap_loop::ProposalState::DeferredPendingEndpoint);
  EXPECT_FALSE(r.committed);
  EXPECT_FALSE(r.workspace.has_value());
}

TEST(AuthoritativeLoopM74, TransactionPreservesContentAndIsolatesWorkspace)
{
  GLOBAL_MAP map = make_extended_graph_map();
  const auto ids_before = [&]() {
    std::vector<uint64_t> v;
    for (const auto & a : map.AnchorsDATA) {
      v.push_back(a.id.value);
    }
    return v;
  }();
  const auto watermark = map.next_landmark_id;
  // Delete one landmark on authoritative map before correction.
  erase_landmark_at_storage_index(map, 0, nullptr);
  rebuild_id_to_index(map);
  const auto deleted_gone = map.AnchorsDATA[0].id.value;

  auto prop = make_prop_from_map(map, 104, {0.15, 0.02, -1.0});
  GLOBAL_MAP untouched = gmap_map_clone::deep_clone(map);

  std::vector<double> dist;
  CAM cam = make_cam(dist);
  auto cfg = gmap_authoritative_loop::ProcessConfig{};
  cfg.enable_post_loop_ba = false;
  auto r = gmap_authoritative_loop::process_proposal_on_workspace(
    map, prop, cam, cfg);
  ASSERT_TRUE(r.committed);
  ASSERT_TRUE(r.workspace.has_value());

  // Authoritative input unchanged (workspace isolation).
  for (std::size_t i = 0; i < map.KeyFDATA.size(); ++i) {
    EXPECT_NEAR(map.KeyFDATA[i].t_c2n[0], untouched.KeyFDATA[i].t_c2n[0], kTol);
  }
  // Workspace preserves KF count (intervening tips).
  EXPECT_EQ(r.workspace->KeyFDATA.size(), map.KeyFDATA.size());
  EXPECT_EQ(r.workspace->next_landmark_id, watermark);
  // Deleted identity not restored.
  bool found_deleted = false;
  for (const auto & a : r.workspace->AnchorsDATA) {
    if (a.id.value == ids_before[0]) {
      found_deleted = true;
    }
  }
  EXPECT_FALSE(found_deleted);
  (void)deleted_gone;

  // Same-KF alignment uses closing identity, not tip.
  const int idx_c =
    gmap_authoritative_loop::find_kf_index_by_id(*r.workspace, 104);
  ASSERT_GE(idx_c, 0);
  EXPECT_NEAR(r.p_G_final_closing[0],
    r.workspace->KeyFDATA[static_cast<std::size_t>(idx_c)].t_c2n[0], kTol);
  EXPECT_NEAR(r.delta_L[2], 0.0, kTol);
  const auto tip = r.workspace->KeyFDATA.back().t_c2n;
  // Tip may differ from closing after extended correction.
  EXPECT_EQ(r.workspace->KeyFDATA.back().keyframe_id, 106u);

  const auto inv = validate_landmark_identity_invariants(*r.workspace);
  EXPECT_TRUE(inv.ok) << inv.reason;
  (void)tip;
}

TEST(AuthoritativeLoopM74, PoseSlamFailureLeavesNoWorkspace)
{
  GLOBAL_MAP map;
  map.idx_ref_pose_slam = 0;
  map.KeyFDATA.push_back(make_kf(0, 0, -1, 1));
  // Invalid: closing == ref with empty odometry chain is ok, but tip < closing
  // forced by bad proposal ids.
  auto prop = make_prop_from_map(map, 1, {0, 0, -1});
  prop.pose_graph_ref_keyframe_id = 999;  // missing ref
  std::vector<double> dist;
  CAM cam = make_cam(dist);
  auto r = gmap_authoritative_loop::process_proposal_on_workspace(
    map, prop, cam, {});
  EXPECT_FALSE(r.committed);
  EXPECT_FALSE(r.workspace.has_value());
}

TEST(AuthoritativeLoopM74, AlignSameKfNotTip)
{
  const arma::vec::fixed<3> p_l = {0.2, 0.05, -1.0};
  const arma::vec::fixed<3> p_g_closing = {0.15, 0.02, -0.9};
  const arma::vec::fixed<3> p_g_tip = {0.6, 0.15, -0.9};
  const auto d_ok = gmap_visual_sync::align_delta_xy(p_g_closing, p_l);
  const auto d_wrong = gmap_visual_sync::align_delta_xy(p_g_tip, p_l);
  EXPECT_NEAR(d_ok[0], -0.05, 1e-12);
  EXPECT_NEAR(d_ok[2], 0.0, 1e-12);
  EXPECT_GT(std::abs(d_wrong[0] - d_ok[0]), 0.1);
}
