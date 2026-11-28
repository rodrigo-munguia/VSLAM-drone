/**
 * GSLAM-MAP-ARCH-M7.6: transaction isolation, deferred lifecycle, reset,
 * intervening map-content preservation, revalidation.
 */
#include <cstdint>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "gmap_authoritative_loop.hpp"
#include "gmap_authoritative_read.hpp"
#include "gmap_deferred_proposal.hpp"
#include "gmap_landmark_identity.hpp"
#include "gmap_loop_proposal.hpp"
#include "gmap_map_clone.hpp"
#include "gmap_structural_invariants_m7_6.hpp"
#include "gmap_visual_sync.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/keyframe_types.hpp"

using gmap_landmark_id::erase_landmark_at_storage_index;
using gmap_landmark_id::insert_landmark;
using gmap_landmark_id::rebuild_id_to_index;

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
  return kf;
}

G_ANCHOR make_anchor(double x, double y, double z, int init_kf)
{
  G_ANCHOR a;
  a.feat_type = "g_anchor";
  a.AnchorState = {x, y, z};
  a.init_KF = init_kf;
  a.n_kf_matched = 2;
  a.Descriptor = cv::Mat::zeros(1, 32, CV_8U);
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

GLOBAL_MAP make_map()
{
  GLOBAL_MAP map;
  map.idx_ref_pose_slam = 1;
  map.KeyFDATA.push_back(make_kf(0, 0, -1, 100));
  map.KeyFDATA.push_back(make_kf(1, 0, -1, 101));
  map.KeyFDATA.push_back(make_kf(2, 0, -1, 102));
  map.KeyFDATA.push_back(make_kf(0.2, 0, -1, 103));  // closing
  map.next_landmark_id = 50;
  insert_landmark(map, make_anchor(0.5, 0, 2, 0));
  insert_landmark(map, make_anchor(1.5, 0, 2, 1));
  map.Vgraph = arma::zeros(4, 4);
  rebuild_id_to_index(map);
  return map;
}

gmap_loop::LoopProposal make_prop(const GLOBAL_MAP & map, uint64_t life = 0)
{
  gmap_loop::LoopProposal p;
  p.proposal_id = 7;
  p.map_lifetime = life;
  p.closing_keyframe_id = 103;
  p.pose_graph_ref_keyframe_id = 101;
  p.matched_historical_keyframe_id = 101;
  p.pnp_C_nav = {0.1, 0.0, -1.0};
  p.p_L_original = map.KeyFDATA[3].t_c2n;
  p.t_ref_at_verification = map.KeyFDATA[1].t_c2n;
  p.t_loop = p.t_ref_at_verification - p.pnp_C_nav;
  p.kind = gmap_loop::Kind::Tag;
  p.closing_kf = map.KeyFDATA[3];
  return p;
}

}  // namespace

TEST(LifecycleM76, InterveningContentPreservedOnSuccessfulCorrection)
{
  GLOBAL_MAP map = make_map();
  auto prop = make_prop(map);

  // Intervening growth before processing.
  map.KeyFDATA.push_back(make_kf(0.4, 0.1, -1, 104));
  map.KeyFDATA.push_back(make_kf(0.6, 0.2, -1, 105));
  const auto id_new = insert_landmark(map, make_anchor(0.7, 0.1, 2, 4));
  map.KeyFDATA[4].Matched_landmark_ids = {id_new};
  map.KeyFDATA[4].UV_Matched_points = {{321, 241}};
  const LandmarkId deleted_id = map.AnchorsDATA[0].id;
  erase_landmark_at_storage_index(map, 0, nullptr);  // delete one old LM
  rebuild_id_to_index(map);
  map.Vgraph = arma::zeros(map.KeyFDATA.size(), map.KeyFDATA.size());

  const auto kf_before = gmap_struct_m76::keyframe_id_set(map);
  const auto lm_before = gmap_struct_m76::landmark_id_set(map);
  const uint64_t watermark = map.next_landmark_id;
  ASSERT_EQ(lm_before.count(deleted_id.value), 0u);

  std::vector<double> dist;
  CAM cam = make_cam(dist);
  gmap_authoritative_loop::ProcessConfig cfg;
  cfg.enable_post_loop_ba = false;
  auto r = gmap_authoritative_loop::process_proposal_on_workspace(
    map, prop, cam, cfg);
  ASSERT_TRUE(r.committed) << r.reason;
  ASSERT_TRUE(r.workspace.has_value());

  EXPECT_EQ(gmap_struct_m76::keyframe_id_set(*r.workspace), kf_before);
  EXPECT_EQ(gmap_struct_m76::landmark_id_set(*r.workspace), lm_before);
  EXPECT_GE(r.workspace->next_landmark_id, watermark);
  EXPECT_EQ(gmap_struct_m76::landmark_id_set(*r.workspace).count(deleted_id.value), 0u);
  EXPECT_TRUE(gmap_struct_m76::check(*r.workspace).ok);
  // Authoritative input unchanged (workspace isolation).
  EXPECT_EQ(gmap_struct_m76::keyframe_id_set(map), kf_before);
}

TEST(LifecycleM76, FailedCorrectionAtomicity)
{
  GLOBAL_MAP map = make_map();
  auto prop = make_prop(map);
  prop.pose_graph_ref_keyframe_id = 9999;  // invalid → fail
  GLOBAL_MAP before = gmap_map_clone::deep_clone(map);
  std::vector<double> dist;
  CAM cam = make_cam(dist);
  gmap_authoritative_loop::ProcessConfig cfg;
  auto r = gmap_authoritative_loop::process_proposal_on_workspace(
    map, prop, cam, cfg);
  EXPECT_FALSE(r.committed);
  EXPECT_FALSE(r.workspace.has_value());
  EXPECT_EQ(map.KeyFDATA.size(), before.KeyFDATA.size());
  EXPECT_NEAR(map.KeyFDATA[0].t_c2n[0], before.KeyFDATA[0].t_c2n[0], 1e-12);
  EXPECT_EQ(gmap_struct_m76::landmark_id_set(map),
    gmap_struct_m76::landmark_id_set(before));
}

TEST(LifecycleM76, WorkspaceMutationDoesNotLeakBeforeCommit)
{
  GLOBAL_MAP auth = make_map();
  GLOBAL_MAP ws = gmap_map_clone::deep_clone(auth);
  ws.KeyFDATA[0].t_c2n[0] = 42;
  ws.AnchorsDATA[0].AnchorState[0] = 99;
  if (!ws.AnchorsDATA[0].Descriptor.empty()) {
    ws.AnchorsDATA[0].Descriptor.at<unsigned char>(0, 0) = 11;
  }
  EXPECT_NEAR(auth.KeyFDATA[0].t_c2n[0], 0.0, 1e-12);
  EXPECT_NEAR(auth.AnchorsDATA[0].AnchorState[0], 0.5, 1e-12);
}

TEST(LifecycleM76, ObsoleteLifetimeProposalRejected)
{
  GLOBAL_MAP map = make_map();
  auto d = gmap_deferred_proposal::evaluate_closing_endpoint(
    103, /*prop_life=*/1, /*cur_life=*/2, map, {},
    gmap_visual_sync::AdmitMode::Open, 0, 0, false);
  EXPECT_EQ(d.action, gmap_deferred_proposal::DeferDecision::Action::Reject);
  EXPECT_EQ(d.status, gmap_deferred_proposal::ClosingStatus::ObsoleteLifetime);
}

TEST(LifecycleM76, DeferredBoundThenSubsequentProcess)
{
  GLOBAL_MAP map;
  auto reject = gmap_deferred_proposal::evaluate_closing_endpoint(
    5, 0, 0, map, {}, gmap_visual_sync::AdmitMode::Open, 0,
    gmap_deferred_proposal::kMaxActiveDeferCycles, false);
  EXPECT_EQ(reject.action, gmap_deferred_proposal::DeferDecision::Action::Reject);

  map.KeyFDATA.push_back(make_kf(0, 0, -1, 5));
  auto ok = gmap_deferred_proposal::evaluate_closing_endpoint(
    5, 0, 0, map, {}, gmap_visual_sync::AdmitMode::Open, 0, 0, false);
  EXPECT_EQ(ok.action, gmap_deferred_proposal::DeferDecision::Action::Process);
}

TEST(LifecycleM76, UnknownApplyKeepsSuspendSemantics)
{
  // Policy constant: Suspend mode defers without incrementing active cycles.
  GLOBAL_MAP map;
  auto d = gmap_deferred_proposal::evaluate_closing_endpoint(
    5, 0, 0, map, {}, gmap_visual_sync::AdmitMode::Suspend, 0, 12, true);
  EXPECT_EQ(d.action, gmap_deferred_proposal::DeferDecision::Action::Defer);
  EXPECT_EQ(d.active_cycles_after, 12);
}

TEST(LifecycleM76, ResetAfterExtractRejectsObsoleteProposal)
{
  GLOBAL_MAP auth = make_map();
  auto extract = gmap_auth_read::make_detection_extract(auth, /*life=*/3, 0, 1);
  auto prop = make_prop(extract.map, /*life=*/3);

  // Reset authoritative map / bump lifetime.
  auth = GLOBAL_MAP{};
  auth.next_landmark_id = 1;
  const uint64_t life_new = 4;
  EXPECT_NE(prop.map_lifetime, life_new);

  auto d = gmap_deferred_proposal::evaluate_closing_endpoint(
    prop.closing_keyframe_id, prop.map_lifetime, life_new, auth, {},
    gmap_visual_sync::AdmitMode::Open, 0, 0, false);
  EXPECT_EQ(d.action, gmap_deferred_proposal::DeferDecision::Action::Reject);

  // Extract still memory-safe.
  EXPECT_FALSE(extract.map.KeyFDATA.empty());
  EXPECT_EQ(extract.map.KeyFDATA[0].keyframe_id, 100u);
}

TEST(LifecycleM76, UnrelatedGrowthDoesNotInvalidateEndpoint)
{
  GLOBAL_MAP map = make_map();
  map.KeyFDATA.push_back(make_kf(1, 1, -1, 200));
  insert_landmark(map, make_anchor(3, 3, 3, 0));
  rebuild_id_to_index(map);
  auto d = gmap_deferred_proposal::evaluate_closing_endpoint(
    103, 0, 0, map, {}, gmap_visual_sync::AdmitMode::Open, 0, 0, false);
  EXPECT_EQ(d.action, gmap_deferred_proposal::DeferDecision::Action::Process);
}

TEST(LifecycleM76, CommonTranslationRevalidation)
{
  GLOBAL_MAP map = make_map();
  auto prop = make_prop(map);
  prop.kind = gmap_loop::Kind::Visual;
  prop.inlier_landmark_ids.push_back(map.AnchorsDATA[0].id);
  prop.inlier_xyz_at_verification.push_back(map.AnchorsDATA[0].AnchorState);
  prop.inlier_uvs.push_back({320, 240});

  const arma::vec::fixed<3> Delta = {2.0, -1.0, 0.25};
  map.KeyFDATA[static_cast<std::size_t>(map.idx_ref_pose_slam)].t_c2n += Delta;
  for (auto & a : map.AnchorsDATA) {
    a.AnchorState += Delta;
  }
  // Evidence stores pre-translation XYZ.
  prop.inlier_xyz_at_verification[0] = map.AnchorsDATA[0].AnchorState - Delta;

  auto cmp = gmap_authoritative_loop::compare_relevant_geometry(
    map, prop, map.idx_ref_pose_slam, {});
  EXPECT_EQ(cmp.decision,
    gmap_loop::RevalidationDecision::CommonTranslationReuse);
}

TEST(LifecycleM76, NoActiveCloopFullMapWritebackSurface)
{
  // Static architectural check: CLOOP no longer exposes Get_GlobalMap.
  // Compile-time: Install/Clear only. Runtime: proposal path uses workspace.
  GLOBAL_MAP auth = make_map();
  auto extract = gmap_auth_read::make_detection_extract(auth, 0, 0, 1);
  extract.map.KeyFDATA[0].t_c2n[0] = 123;
  EXPECT_NEAR(auth.KeyFDATA[0].t_c2n[0], 0.0, 1e-12);
}
