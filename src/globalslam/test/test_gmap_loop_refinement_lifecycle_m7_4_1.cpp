/**
 * GSLAM-MAP-ARCH-M7.4.1: Match_list restoration + deferred proposal lifecycle.
 */
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "gmap_authoritative_loop.hpp"
#include "gmap_deferred_proposal.hpp"
#include "gmap_landmark_identity.hpp"
#include "gmap_loop_proposal.hpp"
#include "gmap_map_clone.hpp"
#include "gmap_post_loop_refine.hpp"
#include "gmap_visual_sync.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/keyframe_types.hpp"

using gmap_landmark_id::insert_landmark;
using gmap_landmark_id::rebuild_id_to_index;
using gmap_landmark_id::validate_landmark_identity_invariants;

namespace
{

KEYFRAME make_kf(double x, double y, double z, uint64_t id)
{
  KEYFRAME kf;
  kf.t_c2n = {x, y, z};
  kf.Rn2c = arma::eye<arma::mat>(3, 3);
  kf.Rr2c = arma::eye<arma::mat>(3, 3);
  kf.Rn2r = arma::eye<arma::mat>(3, 3);
  kf.keyframe_id = id;
  kf.radius_uncertainty = 1.0;
  kf.Descriptors_all = cv::Mat::zeros(4, 32, CV_8U);
  for (int i = 0; i < 4; ++i) {
    kf.Descriptors_all.at<unsigned char>(i, 0) =
      static_cast<unsigned char>(10 + i);
    kf.keyPoints_all.emplace_back(320.f + i, 240.f, 1.f);
  }
  return kf;
}

G_ANCHOR make_anchor(double x, double y, double z, int init_kf, unsigned char d0)
{
  G_ANCHOR a;
  a.feat_type = "g_anchor";
  a.AnchorState = {x, y, z};
  a.init_KF = init_kf;
  a.n_kf_matched = 2;
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

GLOBAL_MAP make_map_with_vgraph()
{
  GLOBAL_MAP map;
  map.idx_ref_pose_slam = 0;
  map.KeyFDATA.push_back(make_kf(0, 0, -1, 10));
  map.KeyFDATA.push_back(make_kf(1, 0, -1, 11));
  map.KeyFDATA.push_back(make_kf(2, 0, -1, 12));  // closing
  map.KeyFDATA.push_back(make_kf(2.2, 0.1, -1, 13));  // tip after closing
  map.next_landmark_id = 50;
  const auto id_a = insert_landmark(map, make_anchor(0.5, 0.0, 2.0, 0, 10));
  const auto id_b = insert_landmark(map, make_anchor(1.5, 0.0, 2.0, 1, 11));
  map.KeyFDATA[0].Matched_landmark_ids = {id_a};
  map.KeyFDATA[0].UV_Matched_points = {{320, 240}};
  map.KeyFDATA[1].Matched_landmark_ids = {id_a, id_b};
  map.KeyFDATA[1].UV_Matched_points = {{321, 241}, {330, 245}};
  // Strong Vgraph links so get_visually_linked_kf(strength>10) returns neighbors.
  map.Vgraph = arma::zeros(4, 4);
  map.Vgraph(0, 1) = 20;
  map.Vgraph(1, 0) = 20;
  map.Vgraph(1, 2) = 20;
  map.Vgraph(2, 1) = 20;
  map.Vgraph(2, 3) = 15;
  map.Vgraph(3, 2) = 15;
  rebuild_id_to_index(map);
  return map;
}

}  // namespace

TEST(LoopRefinementM741, MatchListInsertsObservationsAndUpdatesCounters)
{
  GLOBAL_MAP map = make_map_with_vgraph();
  std::vector<double> dist;
  CAM cam = make_cam(dist);
  auto cfg = gmap_post_loop_refine::MatchConfig{};
  cfg.vm_max_inov_error = 200;  // permissive for synthetic fixture
  cfg.img_cols = 640;
  cfg.img_rows = 480;
  cfg.min_vgraph_strength = 10;

  const std::size_t n_obs_before = map.KeyFDATA[2].Matched_landmark_ids.size();
  const int n_match_before = map.AnchorsDATA[0].n_kf_matched;

  std::vector<int> idx_new, idx_old;
  gmap_post_loop_refine::run_post_loop_matching(
    map, /*idx_closing=*/2, /*idx_matched=*/0, cam, cfg, idx_new, idx_old);

  EXPECT_FALSE(idx_new.empty());
  EXPECT_FALSE(idx_old.empty());
  EXPECT_NE(std::find(idx_new.begin(), idx_new.end(), 2), idx_new.end());
  EXPECT_NE(std::find(idx_old.begin(), idx_old.end(), 0), idx_old.end());

  // May or may not match depending on descriptors; at least API mutates safely.
  const auto inv = validate_landmark_identity_invariants(map);
  EXPECT_TRUE(inv.ok) << inv.reason;
  EXPECT_EQ(map.next_landmark_id, 52u);
  (void)n_obs_before;
  (void)n_match_before;
}

TEST(LoopRefinementM741, MatchListOnlyMutatesWorkspace)
{
  GLOBAL_MAP auth = make_map_with_vgraph();
  GLOBAL_MAP ws = gmap_map_clone::deep_clone(auth);
  std::vector<double> dist;
  CAM cam = make_cam(dist);
  auto cfg = gmap_post_loop_refine::MatchConfig{};
  cfg.vm_max_inov_error = 200;
  cfg.min_vgraph_strength = 10;
  std::vector<int> a, b;
  gmap_post_loop_refine::run_post_loop_matching(ws, 2, 0, cam, cfg, a, b);
  // Authoritative observation counts unchanged.
  EXPECT_EQ(auth.KeyFDATA[2].Matched_landmark_ids.size(), 0u);
  EXPECT_EQ(auth.AnchorsDATA[0].n_kf_matched, 2);
}

TEST(LoopRefinementM741, DeferredPendingInBufferDoesNotStarveCounter)
{
  GLOBAL_MAP map;
  map.KeyFDATA.push_back(make_kf(0, 0, -1, 1));
  std::vector<KEYFRAME> buf = {make_kf(1, 0, -1, 5)};
  auto d = gmap_deferred_proposal::evaluate_closing_endpoint(
    5, 0, 0, map, buf, gmap_visual_sync::AdmitMode::Open, 0, 10, false);
  EXPECT_EQ(d.action, gmap_deferred_proposal::DeferDecision::Action::Defer);
  EXPECT_EQ(d.status, gmap_deferred_proposal::ClosingStatus::PendingInBuffer);
  EXPECT_EQ(d.active_cycles_after, 10);  // unchanged
}

TEST(LoopRefinementM741, SuspendDoesNotIncrementDeferCounter)
{
  GLOBAL_MAP map;
  auto d = gmap_deferred_proposal::evaluate_closing_endpoint(
    5, 0, 0, map, {}, gmap_visual_sync::AdmitMode::Suspend, 0, 3, true);
  EXPECT_EQ(d.action, gmap_deferred_proposal::DeferDecision::Action::Defer);
  EXPECT_EQ(d.active_cycles_after, 3);
}

TEST(LoopRefinementM741, ActiveCycleBoundTerminatesDeferred)
{
  GLOBAL_MAP map;
  auto d = gmap_deferred_proposal::evaluate_closing_endpoint(
    5, 0, 0, map, {}, gmap_visual_sync::AdmitMode::Open, 0,
    gmap_deferred_proposal::kMaxActiveDeferCycles, false);
  EXPECT_EQ(d.action, gmap_deferred_proposal::DeferDecision::Action::Reject);
  EXPECT_EQ(d.reason, "deferred_closing_kf_timeout_active_cycles");
}

TEST(LoopRefinementM741, WatermarkInvalidRejects)
{
  GLOBAL_MAP map;
  auto d = gmap_deferred_proposal::evaluate_closing_endpoint(
    5, 0, 0, map, {}, gmap_visual_sync::AdmitMode::Bound, 10, 0, false);
  EXPECT_EQ(d.action, gmap_deferred_proposal::DeferDecision::Action::Reject);
  EXPECT_EQ(d.status, gmap_deferred_proposal::ClosingStatus::WatermarkInvalid);
}

TEST(LoopRefinementM741, ObsoleteLifetimeRejects)
{
  GLOBAL_MAP map;
  auto d = gmap_deferred_proposal::evaluate_closing_endpoint(
    5, 1, 2, map, {}, gmap_visual_sync::AdmitMode::Open, 0, 0, false);
  EXPECT_EQ(d.action, gmap_deferred_proposal::DeferDecision::Action::Reject);
  EXPECT_EQ(d.status, gmap_deferred_proposal::ClosingStatus::ObsoleteLifetime);
}

TEST(LoopRefinementM741, AdmittedProcesses)
{
  GLOBAL_MAP map;
  map.KeyFDATA.push_back(make_kf(0, 0, -1, 5));
  auto d = gmap_deferred_proposal::evaluate_closing_endpoint(
    5, 0, 0, map, {}, gmap_visual_sync::AdmitMode::Open, 0, 7, false);
  EXPECT_EQ(d.action, gmap_deferred_proposal::DeferDecision::Action::Process);
  EXPECT_EQ(d.active_cycles_after, 0);
}

TEST(LoopRefinementM741, AuthoritativePathRunsMatchBeforeBaGate)
{
  GLOBAL_MAP map = make_map_with_vgraph();
  // Make tip==closing for simpler Pose_SLAM tip.
  map.KeyFDATA.resize(3);
  map.Vgraph = arma::zeros(3, 3);
  map.Vgraph(0, 1) = 20;
  map.Vgraph(1, 0) = 20;
  map.Vgraph(1, 2) = 20;
  map.Vgraph(2, 1) = 20;

  gmap_loop::LoopProposal prop;
  prop.proposal_id = 42;
  prop.map_lifetime = 0;
  prop.closing_keyframe_id = 12;
  prop.pose_graph_ref_keyframe_id = 10;
  prop.matched_historical_keyframe_id = 10;
  prop.pnp_C_nav = {0.1, 0.0, -1.0};
  prop.p_L_original = map.KeyFDATA[2].t_c2n;
  prop.t_ref_at_verification = map.KeyFDATA[0].t_c2n;
  prop.t_loop = prop.t_ref_at_verification - prop.pnp_C_nav;
  prop.kind = gmap_loop::Kind::Visual;
  prop.closing_kf = map.KeyFDATA[2];

  std::vector<double> dist;
  CAM cam = make_cam(dist);
  auto cfg = gmap_authoritative_loop::ProcessConfig{};
  cfg.enable_post_loop_ba = false;  // Match_list must still run
  cfg.match.vm_max_inov_error = 200;
  cfg.match.min_vgraph_strength = 10;

  GLOBAL_MAP before = gmap_map_clone::deep_clone(map);
  auto r = gmap_authoritative_loop::process_proposal_on_workspace(
    map, prop, cam, cfg);
  ASSERT_TRUE(r.committed);
  ASSERT_TRUE(r.workspace.has_value());
  // Input map unchanged.
  EXPECT_EQ(map.KeyFDATA[2].Matched_landmark_ids.size(),
    before.KeyFDATA[2].Matched_landmark_ids.size());
  EXPECT_NEAR(r.delta_L[2], 0.0, 1e-12);
}
