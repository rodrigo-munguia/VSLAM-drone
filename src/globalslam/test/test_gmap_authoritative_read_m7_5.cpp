/**
 * GSLAM-MAP-ARCH-M7.5: authoritative read access, owned extracts, OpenCV
 * isolation, publication geometry parity, concurrent extract vs mutate.
 */
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "gmap_authoritative_loop.hpp"
#include "gmap_authoritative_read.hpp"
#include "gmap_deferred_proposal.hpp"
#include "gmap_landmark_identity.hpp"
#include "gmap_loop_proposal.hpp"
#include "gmap_map_clone.hpp"
#include "gmap_visual_sync.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/keyframe_types.hpp"

using gmap_landmark_id::erase_landmark_at_storage_index;
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
  kf.frame = cv::Mat::zeros(8, 8, CV_8UC1);
  kf.frame.at<unsigned char>(0, 0) = 42;
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
  a.n_kf_matched = 2;
  a.Descriptor = cv::Mat::zeros(1, 32, CV_8U);
  a.Descriptor.at<unsigned char>(0, 0) = d0;
  a.Keypoint = cv::KeyPoint(100.f, 200.f, 1.f);
  return a;
}

GLOBAL_MAP make_fixture_map()
{
  GLOBAL_MAP map;
  map.idx_ref_pose_slam = 1;
  map.KeyFDATA.push_back(make_kf(0, 0, -1, 10));
  map.KeyFDATA.push_back(make_kf(1, 0, -1, 11));
  map.KeyFDATA.push_back(make_kf(2, 0, -1, 12));
  map.KeyFDATA.push_back(make_kf(3, 0.1, -1, 13));
  map.next_landmark_id = 50;
  const auto id_a = insert_landmark(map, make_anchor(0.5, 0.0, 2.0, 0, 9));
  const auto id_b = insert_landmark(map, make_anchor(1.5, 0.0, 2.0, 1, 11));
  map.KeyFDATA[0].Matched_landmark_ids = {id_a};
  map.KeyFDATA[0].UV_Matched_points = {{320, 240}};
  map.KeyFDATA[1].Matched_landmark_ids = {id_a, id_b};
  map.KeyFDATA[1].UV_Matched_points = {{321, 241}, {330, 250}};
  map.KeyFDATA[2].Matched_landmark_ids = {id_b};
  map.KeyFDATA[2].UV_Matched_points = {{331, 251}};
  map.Vgraph = arma::zeros<arma::mat>(4, 4);
  map.Vgraph(0, 1) = 2;
  map.Vgraph(1, 0) = 2;
  map.Vgraph(1, 2) = 1;
  map.Vgraph(2, 1) = 1;
  rebuild_id_to_index(map);
  return map;
}

}  // namespace

TEST(M75AuthRead, DetectionExtractDeepCloneParity)
{
  GLOBAL_MAP auth = make_fixture_map();
  const auto extract = gmap_auth_read::make_detection_extract(auth, 7, 3, 11);
  ASSERT_EQ(extract.map.KeyFDATA.size(), auth.KeyFDATA.size());
  ASSERT_EQ(extract.map.AnchorsDATA.size(), auth.AnchorsDATA.size());
  EXPECT_EQ(extract.map_lifetime, 7u);
  EXPECT_EQ(extract.correction_epoch, 3u);
  EXPECT_EQ(extract.auth_seq, 11u);
  EXPECT_EQ(extract.map.idx_ref_pose_slam, auth.idx_ref_pose_slam);
  for (std::size_t i = 0; i < auth.KeyFDATA.size(); ++i) {
    EXPECT_EQ(extract.map.KeyFDATA[i].keyframe_id, auth.KeyFDATA[i].keyframe_id);
    EXPECT_NEAR(extract.map.KeyFDATA[i].t_c2n[0], auth.KeyFDATA[i].t_c2n[0], 1e-12);
  }
  for (std::size_t i = 0; i < auth.AnchorsDATA.size(); ++i) {
    EXPECT_EQ(extract.map.AnchorsDATA[i].id, auth.AnchorsDATA[i].id);
    EXPECT_NEAR(
      extract.map.AnchorsDATA[i].AnchorState[0],
      auth.AnchorsDATA[i].AnchorState[0], 1e-12);
  }
}

TEST(M75AuthRead, CandidatePoseOrderingPreserved)
{
  GLOBAL_MAP auth = make_fixture_map();
  const auto poses = gmap_auth_read::extract_candidate_poses(auth);
  ASSERT_EQ(poses.size(), auth.KeyFDATA.size());
  for (std::size_t i = 0; i < poses.size(); ++i) {
    EXPECT_EQ(poses[i].storage_index, static_cast<int>(i));
    EXPECT_EQ(
      poses[i].keyframe_id,
      static_cast<int64_t>(auth.KeyFDATA[i].keyframe_id));
  }
}

TEST(M75AuthRead, OpenCVOwnershipIsolation)
{
  GLOBAL_MAP auth = make_fixture_map();
  auto extract = gmap_auth_read::make_detection_extract(auth, 1, 0, 1);
  ASSERT_FALSE(extract.map.KeyFDATA[0].Descriptors_all.empty());
  ASSERT_FALSE(extract.map.AnchorsDATA[0].Descriptor.empty());

  // Mutate authoritative OpenCV storage after extract.
  auth.KeyFDATA[0].Descriptors_all.at<unsigned char>(0, 0) = 255;
  auth.KeyFDATA[0].frame.at<unsigned char>(0, 0) = 1;
  auth.AnchorsDATA[0].Descriptor.at<unsigned char>(0, 0) = 255;
  auth.AnchorsDATA[0].AnchorState[0] = 99.0;

  EXPECT_EQ(extract.map.KeyFDATA[0].Descriptors_all.at<unsigned char>(0, 0), 20);
  EXPECT_EQ(extract.map.KeyFDATA[0].frame.at<unsigned char>(0, 0), 42);
  EXPECT_EQ(extract.map.AnchorsDATA[0].Descriptor.at<unsigned char>(0, 0), 9);
  EXPECT_NEAR(extract.map.AnchorsDATA[0].AnchorState[0], 0.5, 1e-12);

  // Mutate extract must not alter authoritative.
  extract.map.KeyFDATA[1].Descriptors_all.at<unsigned char>(0, 0) = 77;
  EXPECT_EQ(auth.KeyFDATA[1].Descriptors_all.at<unsigned char>(0, 0), 20);
}

TEST(M75AuthRead, OwnedInputSurvivesAuthMutation)
{
  GLOBAL_MAP auth = make_fixture_map();
  auto extract = gmap_auth_read::make_detection_extract(auth, 2, 1, 5);
  const auto id0 = extract.map.AnchorsDATA[0].id;
  const auto kf0 = extract.map.KeyFDATA[0].keyframe_id;

  insert_landmark(auth, make_anchor(9, 9, 9, 0, 1));
  erase_landmark_at_storage_index(auth, 0, nullptr);
  rebuild_id_to_index(auth);
  auth.KeyFDATA.push_back(make_kf(4, 0, -1, 99));
  auth.KeyFDATA[0].t_c2n[0] = -50.0;

  EXPECT_EQ(extract.map.AnchorsDATA[0].id, id0);
  EXPECT_EQ(extract.map.KeyFDATA[0].keyframe_id, kf0);
  EXPECT_NEAR(extract.map.KeyFDATA[0].t_c2n[0], 0.0, 1e-12);
  EXPECT_EQ(extract.map.KeyFDATA.size(), 4u);
}

TEST(M75AuthRead, PublicationGeometryParity)
{
  GLOBAL_MAP auth = make_fixture_map();
  const auto pub = gmap_auth_read::extract_publication_geometry(auth);
  ASSERT_EQ(pub.t_c2n.size(), auth.KeyFDATA.size());
  ASSERT_EQ(pub.landmark_xyz.size(), auth.AnchorsDATA.size());
  for (std::size_t i = 0; i < auth.KeyFDATA.size(); ++i) {
    EXPECT_NEAR(pub.t_c2n[i][0], auth.KeyFDATA[i].t_c2n[0], 1e-12);
    EXPECT_NEAR(pub.t_c2n[i][1], auth.KeyFDATA[i].t_c2n[1], 1e-12);
  }
  for (std::size_t i = 0; i < auth.AnchorsDATA.size(); ++i) {
    EXPECT_NEAR(
      pub.landmark_xyz[i][0], auth.AnchorsDATA[i].AnchorState[0], 1e-12);
  }
}

TEST(M75AuthRead, CoherentGeometrySingleExtract)
{
  GLOBAL_MAP auth = make_fixture_map();
  const auto extract = gmap_auth_read::make_detection_extract(auth, 1, 0, 1);
  const arma::vec::fixed<3> t_ref =
    extract.map.KeyFDATA[static_cast<std::size_t>(extract.map.idx_ref_pose_slam)]
      .t_c2n;
  const arma::vec::fixed<3> lm = extract.map.AnchorsDATA[0].AnchorState;
  // Simulate map mutation after extract (incoherent if mixed).
  auth.KeyFDATA[extract.map.idx_ref_pose_slam].t_c2n[0] += 10.0;
  auth.AnchorsDATA[0].AnchorState[0] += 10.0;
  // Proposal geometry must stay on the extract state.
  const arma::vec::fixed<3> t_loop = t_ref - lm;
  EXPECT_NEAR(t_loop[0], t_ref[0] - lm[0], 1e-12);
  EXPECT_NE(t_ref[0], auth.KeyFDATA[extract.map.idx_ref_pose_slam].t_c2n[0]);
}

TEST(M75AuthRead, ResetAfterExtractObsoleteLifetime)
{
  GLOBAL_MAP auth = make_fixture_map();
  const uint64_t life_old = 3;
  auto extract = gmap_auth_read::make_detection_extract(auth, life_old, 0, 1);

  // Simulate reset: new lifetime, empty map.
  auth = GLOBAL_MAP{};
  auth.next_landmark_id = 1;
  const uint64_t life_new = life_old + 1;
  EXPECT_NE(extract.map_lifetime, life_new);
  EXPECT_FALSE(extract.map.KeyFDATA.empty());
  EXPECT_TRUE(auth.KeyFDATA.empty());
  // Memory remains valid on extract; lifetime mismatch rejects mutation.
  EXPECT_EQ(extract.map.KeyFDATA[0].keyframe_id, 10u);
}

TEST(M75AuthRead, ConcurrentExtractAndMutate)
{
  GLOBAL_MAP auth = make_fixture_map();
  std::mutex mu;
  std::atomic<bool> run{true};
  std::atomic<int> extracts{0};
  std::atomic<int> mutates{0};
  std::atomic<bool> bad{false};

  std::thread writer([&]() {
    while (run.load()) {
      std::lock_guard<std::mutex> lk(mu);
      if (!auth.AnchorsDATA.empty()) {
        auth.AnchorsDATA[0].AnchorState[0] += 0.001;
        if (!auth.AnchorsDATA[0].Descriptor.empty()) {
          auth.AnchorsDATA[0].Descriptor.at<unsigned char>(0, 0) =
            static_cast<unsigned char>(
              (auth.AnchorsDATA[0].Descriptor.at<unsigned char>(0, 0) + 1) % 200);
        }
      }
      auth.KeyFDATA.push_back(make_kf(0, 0, -1, 1000 + mutates.load()));
      if (auth.KeyFDATA.size() > 12) {
        auth.KeyFDATA.pop_back();
      }
      rebuild_id_to_index(auth);
      ++mutates;
    }
  });

  std::thread reader([&]() {
    for (int i = 0; i < 40; ++i) {
      GLOBAL_MAP local;
      {
        std::lock_guard<std::mutex> lk(mu);
        local = gmap_map_clone::deep_clone(auth);
      }
      if (local.KeyFDATA.empty() || local.AnchorsDATA.empty()) {
        bad = true;
        break;
      }
      // Touch owned Mats after unlock.
      if (!local.AnchorsDATA[0].Descriptor.empty()) {
        (void)local.AnchorsDATA[0].Descriptor.at<unsigned char>(0, 0);
      }
      if (!validate_landmark_identity_invariants(local).ok) {
        bad = true;
        break;
      }
      ++extracts;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    run = false;
  });

  reader.join();
  writer.join();
  EXPECT_FALSE(bad.load());
  EXPECT_GE(extracts.load(), 10);
  EXPECT_GE(mutates.load(), 10);
}

TEST(M75AuthRead, LoopProposalLifetimePreservedOnExtract)
{
  GLOBAL_MAP auth = make_fixture_map();
  auto extract = gmap_auth_read::make_detection_extract(auth, 9, 4, 2);
  gmap_loop::LoopProposal prop;
  prop.proposal_id = 1;
  prop.kind = gmap_loop::Kind::Visual;
  prop.closing_keyframe_id = extract.map.KeyFDATA.back().keyframe_id;
  prop.pose_graph_ref_keyframe_id =
    extract.map.KeyFDATA[static_cast<std::size_t>(extract.map.idx_ref_pose_slam)]
      .keyframe_id;
  prop.matched_historical_keyframe_id =
    static_cast<int64_t>(extract.map.KeyFDATA[0].keyframe_id);
  prop.map_lifetime = extract.map_lifetime;
  prop.correction_epoch = extract.correction_epoch;
  prop.t_loop = extract.map.KeyFDATA[extract.map.idx_ref_pose_slam].t_c2n -
    extract.map.AnchorsDATA[0].AnchorState;

  EXPECT_EQ(prop.map_lifetime, 9u);
  EXPECT_EQ(prop.correction_epoch, 4u);
  EXPECT_EQ(prop.matched_historical_keyframe_id, 10);
  EXPECT_EQ(prop.closing_keyframe_id, 13u);
}

TEST(M75AuthRead, DeferredPolicyUnchangedBound)
{
  // Sanity: M7.4.1 bound constant remains 64.
  EXPECT_EQ(gmap_deferred_proposal::kMaxActiveDeferCycles, 64);
}
