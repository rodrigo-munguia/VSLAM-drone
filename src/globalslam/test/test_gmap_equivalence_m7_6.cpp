/**
 * GSLAM-MAP-ARCH-M7.6: detection / proposal / GTSAM correction equivalence.
 * Detection parity: owned deep extract vs authoritative source (M7.5 contract
 * vs M7.4.1 snapshot semantics for geometry/IDs/descriptor bytes).
 */
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cloop_visual_retrieve.hpp"
#include "gmap_authoritative_loop.hpp"
#include "gmap_authoritative_read.hpp"
#include "gmap_graph_correction.hpp"
#include "gmap_landmark_identity.hpp"
#include "gmap_loop_proposal.hpp"
#include "gmap_map_clone.hpp"
#include "gmap_post_loop_refine.hpp"
#include "gmap_structural_invariants_m7_6.hpp"
#include "gmap_visual_sync.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/keyframe_types.hpp"

using gmap_landmark_id::insert_landmark;
using gmap_landmark_id::rebuild_id_to_index;

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
  kf.Descriptors_all = cv::Mat::zeros(3, 32, CV_8U);
  for (int i = 0; i < 3; ++i) {
    kf.Descriptors_all.at<unsigned char>(i, 0) =
      static_cast<unsigned char>(20 + i);
    kf.keyPoints_all.emplace_back(100.f + i, 200.f, 1.f);
  }
  return kf;
}

G_ANCHOR make_anchor(double x, double y, double z, int init_kf, unsigned char d0)
{
  G_ANCHOR a;
  a.feat_type = "g_anchor";
  a.AnchorState = {x, y, z};
  a.init_KF = init_kf;
  a.n_kf_matched = 3;
  a.Descriptor = cv::Mat::zeros(1, 32, CV_8U);
  a.Descriptor.at<unsigned char>(0, 0) = d0;
  a.Keypoint = cv::KeyPoint(320.f, 240.f, 1.f);
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
  insert_landmark(map, make_anchor(0.5, 0.0, 2.0, 0, 7));
  insert_landmark(map, make_anchor(1.5, 0.0, 2.0, 2, 8));
  insert_landmark(map, make_anchor(2.5, 0.1, 2.0, 3, 9));
  map.Vgraph = arma::zeros(5, 5);
  for (int i = 0; i < 4; ++i) {
    map.Vgraph(i, i + 1) = 1;
    map.Vgraph(i + 1, i) = 1;
  }
  rebuild_id_to_index(map);
  return map;
}

GLOBAL_MAP make_extended_graph_map()
{
  GLOBAL_MAP map = make_tip_equals_closing_map();
  map.KeyFDATA.push_back(make_kf(0.4, 0.1, -1.0, 105));
  map.KeyFDATA.push_back(make_kf(0.6, 0.15, -1.0, 106));
  const auto id_new = insert_landmark(map, make_anchor(0.5, 0.1, 2.0, 5, 10));
  map.KeyFDATA[5].Matched_landmark_ids = {id_new};
  map.KeyFDATA[5].UV_Matched_points = {{330, 250}};
  map.KeyFDATA[6].Matched_landmark_ids = {id_new};
  map.KeyFDATA[6].UV_Matched_points = {{335, 252}};
  map.Vgraph = arma::zeros(7, 7);
  for (int i = 0; i < 6; ++i) {
    map.Vgraph(i, i + 1) = 2;
    map.Vgraph(i + 1, i) = 2;
  }
  rebuild_id_to_index(map);
  return map;
}

gmap_loop::LoopProposal make_prop(
  const GLOBAL_MAP & map, uint64_t closing_id, const arma::vec::fixed<3> & pnp)
{
  gmap_loop::LoopProposal p;
  p.proposal_id = 1;
  p.map_lifetime = 0;
  p.closing_keyframe_id = closing_id;
  p.pose_graph_ref_keyframe_id =
    map.KeyFDATA[static_cast<std::size_t>(map.idx_ref_pose_slam)].keyframe_id;
  p.matched_historical_keyframe_id =
    static_cast<int64_t>(p.pose_graph_ref_keyframe_id);
  p.pnp_C_nav = pnp;
  const int idx_c = gmap_authoritative_loop::find_kf_index_by_id(map, closing_id);
  p.p_L_original = map.KeyFDATA[static_cast<std::size_t>(idx_c)].t_c2n;
  p.t_ref_at_verification =
    map.KeyFDATA[static_cast<std::size_t>(map.idx_ref_pose_slam)].t_c2n;
  p.t_loop = p.t_ref_at_verification - p.pnp_C_nav;
  p.kind = gmap_loop::Kind::Tag;
  p.closing_kf = map.KeyFDATA[static_cast<std::size_t>(idx_c)];
  return p;
}

}  // namespace

TEST(EquivalenceM76, DetectionExtractGeometryIdDescriptorParity)
{
  GLOBAL_MAP auth = make_tip_equals_closing_map();
  // M7.4.1 snapshot semantics for logical content = full assign of values;
  // M7.5 extract = deep_clone. Geometry/IDs/bytes must match.
  auto extract = gmap_auth_read::make_detection_extract(auth, 3, 1, 9);
  ASSERT_EQ(extract.map.KeyFDATA.size(), auth.KeyFDATA.size());
  ASSERT_EQ(extract.map.AnchorsDATA.size(), auth.AnchorsDATA.size());
  EXPECT_EQ(extract.map.idx_ref_pose_slam, auth.idx_ref_pose_slam);
  for (std::size_t i = 0; i < auth.KeyFDATA.size(); ++i) {
    EXPECT_EQ(extract.map.KeyFDATA[i].keyframe_id, auth.KeyFDATA[i].keyframe_id);
    EXPECT_NEAR(extract.map.KeyFDATA[i].t_c2n[0], auth.KeyFDATA[i].t_c2n[0], kTol);
    EXPECT_NEAR(extract.map.KeyFDATA[i].t_c2n[1], auth.KeyFDATA[i].t_c2n[1], kTol);
    EXPECT_EQ(
      cv::norm(extract.map.KeyFDATA[i].Descriptors_all,
        auth.KeyFDATA[i].Descriptors_all, cv::NORM_INF),
      0);
  }
  for (std::size_t i = 0; i < auth.AnchorsDATA.size(); ++i) {
    EXPECT_EQ(extract.map.AnchorsDATA[i].id, auth.AnchorsDATA[i].id);
    EXPECT_NEAR(
      extract.map.AnchorsDATA[i].AnchorState[0],
      auth.AnchorsDATA[i].AnchorState[0], kTol);
    EXPECT_EQ(
      cv::norm(extract.map.AnchorsDATA[i].Descriptor,
        auth.AnchorsDATA[i].Descriptor, cv::NORM_INF),
      0);
  }
}

TEST(EquivalenceM76, CandidateSelectionOrderingParity)
{
  GLOBAL_MAP auth = make_tip_equals_closing_map();
  // Unlink tip from early KFs.
  const int tip = static_cast<int>(auth.KeyFDATA.size()) - 1;
  auth.Vgraph.row(tip).zeros();
  auth.Vgraph.col(tip).zeros();
  auto extract = gmap_auth_read::make_detection_extract(auth, 0, 0, 1);
  const int ba_limit = tip;
  auto e_auth = globalslam::cloop_visual_retrieve::enumerate_unlinked_historical(
    auth.Vgraph, tip, ba_limit);
  auto e_ext = globalslam::cloop_visual_retrieve::enumerate_unlinked_historical(
    extract.map.Vgraph, tip, ba_limit);
  ASSERT_EQ(e_auth, e_ext);

  std::vector<globalslam::cloop_visual_retrieve::RankedCandidate> ca, cb;
  for (int idx : e_auth) {
    globalslam::cloop_visual_retrieve::RankedCandidate c;
    c.idx_kf = idx;
    c.n_matches = 10 - idx;
    ca.push_back(c);
    cb.push_back(c);
  }
  globalslam::cloop_visual_retrieve::rank_candidates_inplace(ca);
  globalslam::cloop_visual_retrieve::rank_candidates_inplace(cb);
  ASSERT_EQ(ca.size(), cb.size());
  for (std::size_t i = 0; i < ca.size(); ++i) {
    EXPECT_EQ(ca[i].idx_kf, cb[i].idx_kf);
    EXPECT_EQ(ca[i].n_matches, cb[i].n_matches);
  }
}

TEST(EquivalenceM76, CoherentLoopProposalFromExtract)
{
  GLOBAL_MAP auth = make_tip_equals_closing_map();
  auto extract = gmap_auth_read::make_detection_extract(auth, 5, 2, 1);
  const arma::vec::fixed<3> pnp = {0.15, 0.02, -1.0};
  auto prop_a = make_prop(auth, 104, pnp);
  auto prop_e = make_prop(extract.map, 104, pnp);
  prop_e.map_lifetime = extract.map_lifetime;
  prop_e.correction_epoch = extract.correction_epoch;

  EXPECT_EQ(prop_a.closing_keyframe_id, prop_e.closing_keyframe_id);
  EXPECT_EQ(prop_a.pose_graph_ref_keyframe_id, prop_e.pose_graph_ref_keyframe_id);
  EXPECT_NEAR(prop_a.t_loop[0], prop_e.t_loop[0], kTol);
  EXPECT_NEAR(prop_a.t_loop[1], prop_e.t_loop[1], kTol);
  EXPECT_NEAR(prop_a.t_loop[2], prop_e.t_loop[2], kTol);
  EXPECT_EQ(prop_e.map_lifetime, 5u);
  EXPECT_EQ(prop_e.correction_epoch, 2u);
}

TEST(EquivalenceM76, IdenticalInputGtsamPoseSlamParity)
{
  GLOBAL_MAP map = make_tip_equals_closing_map();
  const arma::vec::fixed<3> pnp = {0.15, 0.02, -1.0};
  const arma::vec::fixed<3> t_loop =
    map.KeyFDATA[static_cast<std::size_t>(map.idx_ref_pose_slam)].t_c2n - pnp;

  std::vector<arma::vec::fixed<3>> d_a, d_b;
  std::vector<int> i_a, i_b;
  gmap_graph_correction::PoseSlamDiagnostics da, db;
  gmap_graph_correction::PoseSlamConfig cfg;

  ASSERT_TRUE(gmap_graph_correction::run_pose_slam(map, pnp, cfg, d_a, i_a, &da));

  gmap_graph_correction::PoseSlamInputs in;
  in.idx_ref = map.idx_ref_pose_slam;
  in.idx_closing = static_cast<int>(map.KeyFDATA.size()) - 1;
  in.idx_tip = in.idx_closing;
  in.t_loop = t_loop;
  in.pnp_C_nav = pnp;
  ASSERT_TRUE(gmap_graph_correction::run_pose_slam(map, in, cfg, d_b, i_b, &db));

  EXPECT_NEAR(da.initial_error, db.initial_error, kObjTol);
  EXPECT_NEAR(da.final_error, db.final_error, kObjTol);
  ASSERT_EQ(d_a.size(), d_b.size());
  for (std::size_t i = 0; i < d_a.size(); ++i) {
    EXPECT_NEAR(d_a[i][0], d_b[i][0], kTol);
    EXPECT_NEAR(d_a[i][1], d_b[i][1], kTol);
    EXPECT_NEAR(d_a[i][2], d_b[i][2], kTol);
  }
}

TEST(EquivalenceM76, ExtendedGraphPreservesEndpointsAndNewerGeometry)
{
  GLOBAL_MAP map = make_extended_graph_map();
  const auto ids_before = gmap_struct_m76::keyframe_id_set(map);
  const auto lm_before = gmap_struct_m76::landmark_id_set(map);
  const arma::vec::fixed<3> tip_before = map.KeyFDATA.back().t_c2n;
  const arma::vec::fixed<3> pnp = {0.15, 0.02, -1.0};

  gmap_graph_correction::PoseSlamInputs in;
  in.idx_ref = map.idx_ref_pose_slam;
  in.idx_closing = gmap_authoritative_loop::find_kf_index_by_id(map, 104);
  in.idx_tip = static_cast<int>(map.KeyFDATA.size()) - 1;
  in.t_loop = map.KeyFDATA[static_cast<std::size_t>(in.idx_ref)].t_c2n - pnp;
  in.pnp_C_nav = pnp;

  std::vector<arma::vec::fixed<3>> deltas;
  std::vector<int> idx_opt;
  gmap_graph_correction::PoseSlamDiagnostics diag;
  ASSERT_TRUE(gmap_graph_correction::run_pose_slam(
    map, in, gmap_graph_correction::PoseSlamConfig{}, deltas, idx_opt, &diag));
  EXPECT_EQ(diag.idx_closing, in.idx_closing);
  EXPECT_EQ(diag.idx_tip, in.idx_tip);
  EXPECT_EQ(diag.variable_indices.front(), in.idx_ref);
  EXPECT_EQ(diag.variable_indices.back(), in.idx_tip);

  gmap_graph_correction::apply_pose_slam_geometry(map, deltas, idx_opt);
  EXPECT_EQ(gmap_struct_m76::keyframe_id_set(map), ids_before);
  EXPECT_EQ(gmap_struct_m76::landmark_id_set(map), lm_before);
  EXPECT_EQ(map.KeyFDATA.back().keyframe_id, 106u);
  (void)tip_before;
  EXPECT_TRUE(gmap_struct_m76::check(map).ok);
}

TEST(EquivalenceM76, PostLoopMatchingWorkspaceIsolation)
{
  GLOBAL_MAP auth = make_tip_equals_closing_map();
  auth.KeyFDATA[0].Matched_landmark_ids = {auth.AnchorsDATA[0].id};
  auth.KeyFDATA[0].UV_Matched_points = {{320, 240}};
  auth.KeyFDATA[4].Matched_landmark_ids.clear();
  auth.Vgraph(0, 4) = 20;
  auth.Vgraph(4, 0) = 20;
  GLOBAL_MAP ws = gmap_map_clone::deep_clone(auth);
  std::vector<double> dist;
  CAM cam = make_cam(dist);
  gmap_post_loop_refine::MatchConfig cfg;
  cfg.vm_max_inov_error = 200;
  cfg.min_vgraph_strength = 10;
  std::vector<int> a, b;
  gmap_post_loop_refine::run_post_loop_matching(ws, 4, 0, cam, cfg, a, b);
  EXPECT_EQ(auth.KeyFDATA[4].Matched_landmark_ids.size(), 0u);
  EXPECT_TRUE(gmap_struct_m76::check(auth).ok);
  EXPECT_TRUE(gmap_struct_m76::check(ws).ok);
}

TEST(EquivalenceM76, AlignmentDeltaLFromFinalClosing)
{
  GLOBAL_MAP map = make_tip_equals_closing_map();
  auto prop = make_prop(map, 104, {0.15, 0.02, -1.0});
  std::vector<double> dist;
  CAM cam = make_cam(dist);
  gmap_authoritative_loop::ProcessConfig cfg;
  cfg.enable_post_loop_ba = false;
  auto r = gmap_authoritative_loop::process_proposal_on_workspace(
    map, prop, cam, cfg);
  ASSERT_TRUE(r.committed);
  ASSERT_TRUE(r.workspace.has_value());
  const int idx_c = gmap_authoritative_loop::find_kf_index_by_id(*r.workspace, 104);
  ASSERT_GE(idx_c, 0);
  const arma::vec::fixed<3> p_g =
    r.workspace->KeyFDATA[static_cast<std::size_t>(idx_c)].t_c2n;
  const arma::vec::fixed<3> expect =
    gmap_visual_sync::align_delta_xy(p_g, prop.p_L_original);
  EXPECT_NEAR(r.delta_L[0], expect[0], kTol);
  EXPECT_NEAR(r.delta_L[1], expect[1], kTol);
  EXPECT_NEAR(r.delta_L[2], 0.0, kTol);
}
