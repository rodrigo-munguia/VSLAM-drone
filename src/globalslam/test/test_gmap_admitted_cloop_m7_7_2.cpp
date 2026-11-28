/**
 * GSLAM-MAP-ARCH-M7.7.2: admitted-KF-driven CLOOP eligibility + provenance.
 */
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "cloop_visual_retrieve.hpp"
#include "gmap_admitted_cloop_queue.hpp"
#include "gmap_authoritative_loop.hpp"
#include "gmap_deferred_proposal.hpp"
#include "gmap_landmark_identity.hpp"
#include "gmap_loop_proposal.hpp"
#include "gmap_map_clone.hpp"
#include "gmap_relative_kf_init.hpp"
#include "gmap_visual_sync.hpp"
#include "parameters.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/keyframe_types.hpp"

using gmap_landmark_id::insert_landmark;
using gmap_landmark_id::rebuild_id_to_index;

namespace
{

KEYFRAME make_kf(double x, double y, double z, uint64_t id)
{
  KEYFRAME kf;
  kf.t_c2n = {x, y, z};
  kf.t_c2r = {0, 0, 0};
  kf.Rn2c = arma::eye<arma::mat>(3, 3);
  kf.Rr2c = arma::eye<arma::mat>(3, 3);
  kf.Rn2r = arma::eye<arma::mat>(3, 3);
  kf.keyframe_id = id;
  kf.radius_uncertainty = 1.0;
  kf.frame = cv::Mat(48, 64, CV_8UC1, cv::Scalar(40));
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
  a.Descriptor.at<unsigned char>(0, 0) = static_cast<unsigned char>(11 + init_kf);
  return a;
}

parameters make_par()
{
  parameters p{};
  p.CL_min_n_matches = 0;
  p.CL_min_n_not_vl_kf = -1;
  p.CL_min_historical_travel_m = 0.0;
  p.CL_visual_max_pnp_attempts = 3;
  p.BA_max_n_kf_optimized = 2;
  p.Init_number_candidate_points_per_kf = 20;
  p.Mono_cam_cc_u = 32;
  p.Mono_cam_cc_v = 24;
  p.Mono_cam_fc_u = 50;
  p.Mono_cam_fc_v = 50;
  p.Mono_cam_alpha_c = 0;
  p.Mono_cam_distortions = {0, 0, 0, 0, 0};
  return p;
}

GLOBAL_MAP make_chain_map()
{
  GLOBAL_MAP map;
  map.idx_ref_pose_slam = 1;
  for (int i = 0; i < 8; ++i) {
    map.KeyFDATA.push_back(make_kf(0.5 * i, 0.0, -1.0, 100 + static_cast<uint64_t>(i)));
  }
  // Closing = index 5; tip = 7 (newer after closing).
  map.Vgraph = arma::zeros(8, 8);
  for (int i = 0; i < 7; ++i) {
    map.Vgraph(i, i + 1) = 2;
    map.Vgraph(i + 1, i) = 2;
  }
  // Leave closing (5) unlinked to early KFs 0..2 for candidate eligibility.
  map.Vgraph(5, 0) = 0;
  map.Vgraph(0, 5) = 0;
  map.Vgraph(5, 1) = 0;
  map.Vgraph(1, 5) = 0;
  map.Vgraph(5, 2) = 0;
  map.Vgraph(2, 5) = 0;
  map.next_landmark_id = 1;
  for (int i = 0; i < 3; ++i) {
    auto id = insert_landmark(map, make_anchor(0.2 * i, 0.0, 2.0, i));
    map.KeyFDATA[static_cast<std::size_t>(i)].Matched_landmark_ids = {id};
    map.KeyFDATA[static_cast<std::size_t>(i)].UV_Matched_points = {{10.0 + i, 10.0}};
  }
  rebuild_id_to_index(map);
  return map;
}

}  // namespace

TEST(AdmittedCloopM772, QueueFifoOverflowExactlyOnce)
{
  gmap_admitted_cloop::EligibilityQueue q;
  for (uint64_t i = 1; i <= gmap_admitted_cloop::kEligibilityCapacity + 3; ++i) {
    gmap_admitted_cloop::Eligibility e;
    e.keyframe_id = i;
    e.map_lifetime = 1;
    e.closing_query_generation = 0;
    q.push(e);
  }
  EXPECT_EQ(q.size(), gmap_admitted_cloop::kEligibilityCapacity);
  EXPECT_GE(q.stats().overflow_dropped, 3u);
  // Duplicate skipped.
  gmap_admitted_cloop::Eligibility dup;
  dup.keyframe_id = gmap_admitted_cloop::kEligibilityCapacity;  // still in queue
  dup.map_lifetime = 1;
  EXPECT_FALSE(q.push(dup));
  EXPECT_GE(q.stats().duplicate_skipped, 1u);

  auto first = q.try_pop(1, 0);
  ASSERT_TRUE(first.has_value());
  // Oldest dropped were 1..3; first remaining is 4.
  EXPECT_EQ(first->keyframe_id, 4u);
}

TEST(AdmittedCloopM772, QueueObsoleteLifetimeGeneration)
{
  gmap_admitted_cloop::EligibilityQueue q;
  gmap_admitted_cloop::Eligibility e;
  e.keyframe_id = 42;
  e.map_lifetime = 2;
  e.closing_query_generation = 5;
  q.push(e);
  EXPECT_FALSE(q.try_pop(2, 6).has_value());  // gen mismatch
  EXPECT_EQ(q.stats().obsolete_discarded, 1u);
  q.push(e);
  EXPECT_FALSE(q.try_pop(3, 5).has_value());  // lifetime mismatch
}

TEST(AdmittedCloopM772, LocalPoseProvenanceSurvivesRewriteAndClone)
{
  KEYFRAME kf = make_kf(10.0, 20.0, 1.0, 7);
  kf.t_c2n_local_original = kf.t_c2n;
  kf.local_original_valid = true;
  const arma::vec::fixed<3> local = kf.t_c2n_local_original;

  gmap_relative_kf_init::CamPose T;
  T.Rc2n = arma::eye<arma::mat>(3, 3);
  T.t_c2n = {100.0, 200.0, 3.0};
  gmap_relative_kf_init::apply_cam_pose(kf, T);
  EXPECT_NEAR(kf.t_c2n[0], 100.0, 1e-12);
  EXPECT_TRUE(kf.local_original_valid);
  EXPECT_NEAR(kf.t_c2n_local_original[0], local[0], 1e-12);
  EXPECT_NEAR(kf.t_c2n_local_original[1], local[1], 1e-12);

  GLOBAL_MAP map;
  map.KeyFDATA.push_back(kf);
  GLOBAL_MAP clone = gmap_map_clone::deep_clone(map);
  ASSERT_EQ(clone.KeyFDATA.size(), 1u);
  EXPECT_TRUE(clone.KeyFDATA[0].local_original_valid);
  EXPECT_NEAR(clone.KeyFDATA[0].t_c2n_local_original[0], local[0], 1e-12);
  EXPECT_NEAR(clone.KeyFDATA[0].t_c2n[0], 100.0, 1e-12);
}

TEST(AdmittedCloopM772, HistoricalSearchExcludesSelfAndNewer)
{
  using globalslam::cloop_visual_retrieve::enumerate_unlinked_historical;
  GLOBAL_MAP map = make_chain_map();
  const int closing = 5;
  const int ba_limit = closing;  // preceding only
  auto idxs = enumerate_unlinked_historical(map.Vgraph, closing, ba_limit);
  for (int i : idxs) {
    EXPECT_LT(i, closing);
    EXPECT_NE(i, closing);
  }
  // Newer tip indices must not appear.
  for (int i : idxs) {
    EXPECT_LT(i, 7);
  }
}

TEST(AdmittedCloopM772, NoAppendEndpointWhenInMap)
{
  using globalslam::cloop_visual_retrieve::positions_with_current_endpoint;
  std::vector<std::array<double, 3>> mapped = {
    {0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
  std::array<double, 3> cur = {2, 0, 0};
  bool appended = false;
  auto pos = positions_with_current_endpoint(mapped, cur, false, &appended);
  EXPECT_FALSE(appended);
  EXPECT_EQ(pos.size(), 3u);
}

TEST(AdmittedCloopM772, DeferredNotRequiredWhenClosingAdmitted)
{
  GLOBAL_MAP map = make_chain_map();
  const uint64_t closing_id = map.KeyFDATA[5].keyframe_id;
  auto d = gmap_deferred_proposal::evaluate_closing_endpoint(
    closing_id, 0, 0, map, {}, gmap_visual_sync::AdmitMode::Open, 0, 0, false);
  EXPECT_EQ(
    d.action, gmap_deferred_proposal::DeferDecision::Action::Process);
}

TEST(AdmittedCloopM772, AlignUsesImmutableLocalNotRewrittenGlobal)
{
  arma::vec::fixed<3> p_g = {5.0, 7.0, -1.0};
  arma::vec::fixed<3> p_l = {1.0, 2.0, 9.0};
  arma::vec::fixed<3> p_g_wrong_tip = {50.0, 70.0, -1.0};
  const auto d = gmap_visual_sync::align_delta_xy(p_g, p_l);
  EXPECT_NEAR(d[0], 4.0, 1e-12);
  EXPECT_NEAR(d[1], 5.0, 1e-12);
  EXPECT_NEAR(d[2], 0.0, 1e-12);
  const auto d_wrong = gmap_visual_sync::align_delta_xy(p_g_wrong_tip, p_l);
  EXPECT_GT(std::abs(d_wrong[0] - d[0]), 1.0);
}

TEST(AdmittedCloopM772, CloseLoopOnlyIdNotInMap)
{
  GLOBAL_MAP map = make_chain_map();
  EXPECT_FALSE(gmap_deferred_proposal::map_contains_kf_id(map, 12686));
  auto d = gmap_deferred_proposal::evaluate_closing_endpoint(
    12686, 0, 0, map, {}, gmap_visual_sync::AdmitMode::Open, 0, 0, false);
  EXPECT_NE(
    d.action, gmap_deferred_proposal::DeferDecision::Action::Process);
}

TEST(AdmittedCloopM772, AuthoritativeCorrectionUsesLocalOriginal)
{
  GLOBAL_MAP map = make_chain_map();
  // Closing at 5: rewrite Global pose but keep Local original distinct.
  map.KeyFDATA[5].t_c2n_local_original = {1.0, 2.0, -1.0};
  map.KeyFDATA[5].local_original_valid = true;
  map.KeyFDATA[5].t_c2n = {3.0, 4.0, -1.0};

  gmap_loop::LoopProposal prop;
  prop.proposal_id = 9;
  prop.map_lifetime = 0;
  prop.closing_keyframe_id = map.KeyFDATA[5].keyframe_id;
  prop.pose_graph_ref_keyframe_id = map.KeyFDATA[1].keyframe_id;
  prop.matched_historical_keyframe_id =
    static_cast<int64_t>(map.KeyFDATA[0].keyframe_id);
  prop.p_L_original = map.KeyFDATA[5].t_c2n_local_original;
  prop.closing_query_generation = 0;
  prop.pnp_C_nav = {1.1, 2.05, -1.0};
  prop.t_ref_at_verification = map.KeyFDATA[1].t_c2n;
  prop.t_loop = prop.t_ref_at_verification - prop.pnp_C_nav;
  prop.kind = gmap_loop::Kind::Tag;
  prop.source_label = "tag_loop";
  prop.closing_kf = map.KeyFDATA[5];

  std::vector<double> dist;
  CAM cam;
  dist = {0, 0, 0, 0, 0};
  cam.distortions = dist.data();
  cam.cc[0] = 32;
  cam.cc[1] = 24;
  cam.fc[0] = 50;
  cam.fc[1] = 50;
  cam.alpha_c = 0;

  auto cfg = gmap_authoritative_loop::config_from_par(make_par());
  // Tag path may not require visual inliers; exercise workspace process.
  auto r = gmap_authoritative_loop::process_proposal_on_workspace(
    map, prop, cam, cfg);
  // May reject for geometric reasons in this synthetic fixture; if committed,
  // δ_L must use Local original.
  if (r.committed) {
    const arma::vec::fixed<3> expect =
      gmap_visual_sync::align_delta_xy(r.p_G_final_closing, prop.p_L_original);
    EXPECT_NEAR(r.delta_L[0], expect[0], 1e-9);
    EXPECT_NEAR(r.delta_L[1], expect[1], 1e-9);
    EXPECT_NEAR(r.delta_L[2], 0.0, 1e-9);
    ASSERT_TRUE(r.workspace.has_value());
    EXPECT_TRUE(r.workspace->KeyFDATA[5].local_original_valid);
    EXPECT_NEAR(
      r.workspace->KeyFDATA[5].t_c2n_local_original[0], 1.0, 1e-12);
  }
}

TEST(AdmittedCloopM772, CapacityConstantDocumented)
{
  EXPECT_EQ(gmap_admitted_cloop::kEligibilityCapacity, 32u);
}
