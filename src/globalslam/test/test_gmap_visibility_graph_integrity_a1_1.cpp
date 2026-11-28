/**
 * GSLAM-LC-POLICY-A1.1: exact visibility graph integrity regressions.
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <set>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "cloop_visual_retrieve.hpp"
#include "gmap_landmark_identity.hpp"
#include "gmap_map_clone.hpp"
#include "gmap_post_loop_refine.hpp"
#include "gmap_visibility_graph_exact.hpp"

using gmap_landmark_id::erase_landmark_at_storage_index;
using gmap_landmark_id::insert_landmark;
using gmap_visibility_graph_exact::InsertObservationResult;
using gmap_visibility_graph_exact::RebuildDiagnostics;
using gmap_visibility_graph_exact::build_reference_vgraph;
using gmap_visibility_graph_exact::rebuild_exact_visibility_graph;
using gmap_visibility_graph_exact::try_insert_kf_observation;
using gmap_visibility_graph_exact::validate_map_against_reference;

namespace
{

G_ANCHOR make_anchor(double x, int init_kf)
{
  G_ANCHOR a;
  a.feat_type = "g_anchor";
  a.AnchorState = {x, 0.0, 1.0};
  a.init_KF = init_kf;
  a.n_kf_matched = 0;
  a.n_tries_matchs = 0;
  a.Descriptor = cv::Mat::zeros(1, 32, CV_8U);
  a.Keypoint = cv::KeyPoint(0.f, 0.f, 1.f);
  return a;
}

KEYFRAME make_kf(uint64_t id)
{
  KEYFRAME kf;
  kf.keyframe_id = id;
  kf.t_c2n = {0, 0, 0};
  kf.Rn2c.eye();
  return kf;
}

void expect_exact(const GLOBAL_MAP & map)
{
  const auto c = validate_map_against_reference(map);
  EXPECT_TRUE(c.ok) << c.reason << " max_err=" << c.max_abs_error
                    << " mismatches=" << c.weight_mismatches;
  if (!map.KeyFDATA.empty()) {
    EXPECT_EQ(map.Vgraph.n_rows, map.KeyFDATA.size());
    EXPECT_EQ(map.Vgraph.n_cols, map.KeyFDATA.size());
    for (std::size_t i = 0; i < map.KeyFDATA.size(); ++i) {
      EXPECT_NEAR(map.Vgraph(i, i), 0.0, 1e-12);
    }
  }
}

LandmarkId add_obs(
  GLOBAL_MAP & map, int kf, LandmarkId lid, double u = 10.0, double v = 10.0)
{
  const auto r = try_insert_kf_observation(
    map, kf, lid, cv::Point2d(u, v), true);
  EXPECT_EQ(r, InsertObservationResult::Inserted);
  return lid;
}

}  // namespace

TEST(VisibilityGraphA11, EmptyAndReset)
{
  GLOBAL_MAP map;
  auto d = rebuild_exact_visibility_graph(map);
  EXPECT_TRUE(map.Vgraph.empty());
  EXPECT_EQ(d.n_kf, 0);
  expect_exact(map);
}

TEST(VisibilityGraphA11, FirstKfAdmission)
{
  GLOBAL_MAP map;
  map.KeyFDATA.push_back(make_kf(1));
  const LandmarkId lid = insert_landmark(map, make_anchor(1.0, 0));
  add_obs(map, 0, lid);
  rebuild_exact_visibility_graph(map);
  ASSERT_EQ(map.Vgraph.n_rows, 1u);
  EXPECT_NEAR(map.Vgraph(0, 0), 0.0, 1e-12);
  expect_exact(map);
}

TEST(VisibilityGraphA11, ExpansionPreservesExisting)
{
  GLOBAL_MAP map;
  map.KeyFDATA.push_back(make_kf(1));
  map.KeyFDATA.push_back(make_kf(2));
  const LandmarkId a = insert_landmark(map, make_anchor(1.0, 0));
  add_obs(map, 0, a);
  add_obs(map, 1, a);
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(0, 1), 1.0, 1e-12);

  map.KeyFDATA.push_back(make_kf(3));
  const LandmarkId b = insert_landmark(map, make_anchor(2.0, 2));
  add_obs(map, 2, b);
  add_obs(map, 1, b);
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(0, 1), 1.0, 1e-12);
  EXPECT_NEAR(map.Vgraph(1, 2), 1.0, 1e-12);
  EXPECT_NEAR(map.Vgraph(0, 2), 0.0, 1e-12);
  expect_exact(map);
}

TEST(VisibilityGraphA11, LandmarkBirthTwoObservers)
{
  GLOBAL_MAP map;
  map.KeyFDATA.push_back(make_kf(1));
  map.KeyFDATA.push_back(make_kf(2));
  const LandmarkId lid = insert_landmark(map, make_anchor(1.0, 1));
  add_obs(map, 0, lid);
  add_obs(map, 1, lid);
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(0, 1), 1.0, 1e-12);
  expect_exact(map);
}

TEST(VisibilityGraphA11, ThirdObserverCompletesPairs)
{
  GLOBAL_MAP map;
  for (int i = 0; i < 3; ++i) {
    map.KeyFDATA.push_back(make_kf(static_cast<uint64_t>(i + 1)));
  }
  const LandmarkId lid = insert_landmark(map, make_anchor(1.0, 0));
  add_obs(map, 0, lid);
  add_obs(map, 1, lid);
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(0, 1), 1.0, 1e-12);
  EXPECT_NEAR(map.Vgraph(0, 2), 0.0, 1e-12);

  add_obs(map, 2, lid);
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(0, 1), 1.0, 1e-12);
  EXPECT_NEAR(map.Vgraph(0, 2), 1.0, 1e-12);
  EXPECT_NEAR(map.Vgraph(1, 2), 1.0, 1e-12);
  expect_exact(map);
}

TEST(VisibilityGraphA11, MultipleSharedLandmarks)
{
  GLOBAL_MAP map;
  map.KeyFDATA.push_back(make_kf(1));
  map.KeyFDATA.push_back(make_kf(2));
  for (int i = 0; i < 5; ++i) {
    const LandmarkId lid = insert_landmark(map, make_anchor(i, 0));
    add_obs(map, 0, lid, 10 + i, 10);
    add_obs(map, 1, lid, 20 + i, 10);
  }
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(0, 1), 5.0, 1e-12);
  expect_exact(map);
}

TEST(VisibilityGraphA11, DuplicateInsertionIdempotent)
{
  GLOBAL_MAP map;
  map.KeyFDATA.push_back(make_kf(1));
  map.KeyFDATA.push_back(make_kf(2));
  const LandmarkId lid = insert_landmark(map, make_anchor(1.0, 0));
  add_obs(map, 0, lid, 5, 5);
  add_obs(map, 1, lid, 6, 6);
  rebuild_exact_visibility_graph(map);
  const int n_before = map.AnchorsDATA[0].n_kf_matched;
  const auto r = try_insert_kf_observation(
    map, 0, lid, cv::Point2d(5.1, 5.1), true);
  EXPECT_EQ(r, InsertObservationResult::AlreadyPresent);
  EXPECT_EQ(map.KeyFDATA[0].Matched_landmark_ids.size(), 1u);
  EXPECT_EQ(map.AnchorsDATA[0].n_kf_matched, n_before);
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(0, 1), 1.0, 1e-12);
  expect_exact(map);
}

TEST(VisibilityGraphA11, ConflictingSameKfUvPreservesExisting)
{
  GLOBAL_MAP map;
  map.KeyFDATA.push_back(make_kf(1));
  const LandmarkId lid = insert_landmark(map, make_anchor(1.0, 0));
  add_obs(map, 0, lid, 10, 10);
  const auto r = try_insert_kf_observation(
    map, 0, lid, cv::Point2d(50, 50), true);
  EXPECT_EQ(r, InsertObservationResult::ConflictUv);
  ASSERT_EQ(map.KeyFDATA[0].UV_Matched_points.size(), 1u);
  EXPECT_NEAR(map.KeyFDATA[0].UV_Matched_points[0].x, 10.0, 1e-9);
}

TEST(VisibilityGraphA11, LandmarkDeletionRemovesAllPairs)
{
  GLOBAL_MAP map;
  for (int i = 0; i < 4; ++i) {
    map.KeyFDATA.push_back(make_kf(static_cast<uint64_t>(i + 1)));
  }
  const LandmarkId keep = insert_landmark(map, make_anchor(0.0, 0));
  const LandmarkId del = insert_landmark(map, make_anchor(1.0, 0));
  for (int i = 0; i < 4; ++i) {
    add_obs(map, i, del, 1, i);
  }
  add_obs(map, 0, keep, 2, 0);
  add_obs(map, 1, keep, 2, 1);
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(0, 1), 2.0, 1e-12);
  EXPECT_NEAR(map.Vgraph(2, 3), 1.0, 1e-12);

  std::size_t del_idx = 0;
  ASSERT_TRUE(gmap_landmark_id::resolve_landmark_index(map, del, del_idx));
  std::vector<int> kf_vl;
  erase_landmark_at_storage_index(map, del_idx, &kf_vl);
  gmap_landmark_id::rebuild_id_to_index(map);
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(0, 1), 1.0, 1e-12);
  EXPECT_NEAR(map.Vgraph(2, 3), 0.0, 1e-12);
  expect_exact(map);
}

TEST(VisibilityGraphA11, BatchDeletion)
{
  GLOBAL_MAP map;
  for (int i = 0; i < 3; ++i) {
    map.KeyFDATA.push_back(make_kf(static_cast<uint64_t>(i + 1)));
  }
  std::vector<LandmarkId> lids;
  for (int k = 0; k < 3; ++k) {
    lids.push_back(insert_landmark(map, make_anchor(k, 0)));
    for (int i = 0; i < 3; ++i) {
      add_obs(map, i, lids.back(), 10 + k, i);
    }
  }
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(0, 1), 3.0, 1e-12);

  // Delete first two landmarks (reverse storage order).
  for (int k = 1; k >= 0; --k) {
    const LandmarkId lid = lids[static_cast<std::size_t>(k)];
    std::size_t idx = 0;
    ASSERT_TRUE(gmap_landmark_id::resolve_landmark_index(map, lid, idx));
    erase_landmark_at_storage_index(map, idx, nullptr);
  }
  gmap_landmark_id::rebuild_id_to_index(map);
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(0, 1), 1.0, 1e-12);
  expect_exact(map);
}

TEST(VisibilityGraphA11, PostLoopAndRepeatNoInflation)
{
  GLOBAL_MAP map;
  for (int i = 0; i < 4; ++i) {
    map.KeyFDATA.push_back(make_kf(static_cast<uint64_t>(i + 1)));
  }
  const LandmarkId lid = insert_landmark(map, make_anchor(1.0, 0));
  add_obs(map, 0, lid);
  add_obs(map, 1, lid);
  rebuild_exact_visibility_graph(map);

  // Simulate post-loop insert into KF 2 then rebuild.
  ASSERT_EQ(
    try_insert_kf_observation(map, 2, lid, cv::Point2d(3, 3), true),
    InsertObservationResult::Inserted);
  gmap_post_loop_refine::update_visibility_graph_with_matched_points(map, {}, 2);
  EXPECT_NEAR(map.Vgraph(0, 2), 1.0, 1e-12);
  EXPECT_NEAR(map.Vgraph(1, 2), 1.0, 1e-12);
  expect_exact(map);

  // Repeat same observation — no inflation.
  ASSERT_EQ(
    try_insert_kf_observation(map, 2, lid, cv::Point2d(3.2, 3.1), true),
    InsertObservationResult::AlreadyPresent);
  gmap_post_loop_refine::update_visibility_graph_with_matched_points(map, {}, 2);
  EXPECT_NEAR(map.Vgraph(0, 2), 1.0, 1e-12);
  expect_exact(map);
}

TEST(VisibilityGraphA11, PureGeometryPreservesGraph)
{
  GLOBAL_MAP map;
  map.KeyFDATA.push_back(make_kf(1));
  map.KeyFDATA.push_back(make_kf(2));
  const LandmarkId lid = insert_landmark(map, make_anchor(1.0, 0));
  add_obs(map, 0, lid);
  add_obs(map, 1, lid);
  rebuild_exact_visibility_graph(map);
  const arma::mat before = map.Vgraph;
  map.KeyFDATA[0].t_c2n = {1.0, 2.0, 3.0};
  map.KeyFDATA[1].t_c2n = {4.0, 5.0, 6.0};
  map.AnchorsDATA[0].AnchorState = {9.0, 8.0, 7.0};
  EXPECT_TRUE(arma::approx_equal(map.Vgraph, before, "absdiff", 1e-12));
}

TEST(VisibilityGraphA11, BaConsumerSeesCurrentGraph)
{
  GLOBAL_MAP map;
  for (int i = 0; i < 5; ++i) {
    map.KeyFDATA.push_back(make_kf(static_cast<uint64_t>(i + 1)));
  }
  const LandmarkId lid = insert_landmark(map, make_anchor(1.0, 0));
  add_obs(map, 0, lid);
  add_obs(map, 4, lid);
  rebuild_exact_visibility_graph(map);
  // strength > 2 would exclude; strength > 0 includes.
  EXPECT_GT(map.Vgraph(4, 0), 0.0);
  EXPECT_LE(map.Vgraph(4, 0), 2.0);
  expect_exact(map);
}

TEST(VisibilityGraphA11, AuthoritativeCommitCloneCoherent)
{
  GLOBAL_MAP map;
  map.KeyFDATA.push_back(make_kf(1));
  map.KeyFDATA.push_back(make_kf(2));
  const LandmarkId lid = insert_landmark(map, make_anchor(1.0, 0));
  add_obs(map, 0, lid);
  add_obs(map, 1, lid);
  rebuild_exact_visibility_graph(map);
  GLOBAL_MAP clone = gmap_map_clone::deep_clone(map);
  expect_exact(clone);
  EXPECT_TRUE(arma::approx_equal(clone.Vgraph, map.Vgraph, "absdiff", 1e-12));
}

TEST(VisibilityGraphA11, FailedTxnWorkspaceDoesNotLeak)
{
  GLOBAL_MAP auth;
  auth.KeyFDATA.push_back(make_kf(1));
  auth.KeyFDATA.push_back(make_kf(2));
  const LandmarkId lid = insert_landmark(auth, make_anchor(1.0, 0));
  add_obs(auth, 0, lid);
  add_obs(auth, 1, lid);
  rebuild_exact_visibility_graph(auth);
  const arma::mat auth_v = auth.Vgraph;

  GLOBAL_MAP workspace = gmap_map_clone::deep_clone(auth);
  const LandmarkId extra = insert_landmark(workspace, make_anchor(9.0, 1));
  add_obs(workspace, 0, extra);
  add_obs(workspace, 1, extra);
  rebuild_exact_visibility_graph(workspace);
  // Discard workspace (failed txn) — auth unchanged.
  EXPECT_TRUE(arma::approx_equal(auth.Vgraph, auth_v, "absdiff", 1e-12));
  expect_exact(auth);
}

TEST(VisibilityGraphA11, OwnedDetectionExtractCoherent)
{
  GLOBAL_MAP auth;
  for (int i = 0; i < 3; ++i) {
    auth.KeyFDATA.push_back(make_kf(static_cast<uint64_t>(i + 1)));
  }
  const LandmarkId lid = insert_landmark(auth, make_anchor(1.0, 0));
  add_obs(auth, 0, lid);
  add_obs(auth, 2, lid);
  rebuild_exact_visibility_graph(auth);
  GLOBAL_MAP extract = gmap_map_clone::deep_clone(auth);
  expect_exact(extract);
  auto a = globalslam::cloop_visual_retrieve::enumerate_unlinked_historical(
    auth.Vgraph, 2, 2);
  auto b = globalslam::cloop_visual_retrieve::enumerate_unlinked_historical(
    extract.Vgraph, 2, 2);
  EXPECT_EQ(a, b);
}

TEST(VisibilityGraphA11, FusionDisjointObservers)
{
  GLOBAL_MAP map;
  for (int i = 0; i < 4; ++i) {
    map.KeyFDATA.push_back(make_kf(static_cast<uint64_t>(i + 1)));
  }
  const LandmarkId A = insert_landmark(map, make_anchor(1.0, 0));
  const LandmarkId B = insert_landmark(map, make_anchor(2.0, 2));
  add_obs(map, 0, A);
  add_obs(map, 1, A);
  add_obs(map, 2, B);
  add_obs(map, 3, B);
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(0, 1), 1.0, 1e-12);
  EXPECT_NEAR(map.Vgraph(2, 3), 1.0, 1e-12);
  EXPECT_NEAR(map.Vgraph(0, 2), 0.0, 1e-12);

  // Simulate fusion: canonical A on {1,2,3,4}, remove B observations.
  for (int i = 2; i < 4; ++i) {
    // replace B with A
    for (std::size_t j = 0; j < map.KeyFDATA[static_cast<std::size_t>(i)].Matched_landmark_ids.size(); ++j) {
      if (map.KeyFDATA[static_cast<std::size_t>(i)].Matched_landmark_ids[j] == B) {
        map.KeyFDATA[static_cast<std::size_t>(i)].Matched_landmark_ids[j] = A;
      }
    }
  }
  std::size_t bidx = 0;
  ASSERT_TRUE(gmap_landmark_id::resolve_landmark_index(map, B, bidx));
  erase_landmark_at_storage_index(map, bidx, nullptr);
  gmap_landmark_id::rebuild_id_to_index(map);
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(0, 2), 1.0, 1e-12);
  EXPECT_NEAR(map.Vgraph(0, 3), 1.0, 1e-12);
  EXPECT_NEAR(map.Vgraph(1, 2), 1.0, 1e-12);
  EXPECT_NEAR(map.Vgraph(1, 3), 1.0, 1e-12);
  expect_exact(map);
}

TEST(VisibilityGraphA11, FusionOverlappingCanDecreaseWeight)
{
  GLOBAL_MAP map;
  for (int i = 0; i < 4; ++i) {
    map.KeyFDATA.push_back(make_kf(static_cast<uint64_t>(i + 1)));
  }
  const LandmarkId A = insert_landmark(map, make_anchor(1.0, 0));
  const LandmarkId B = insert_landmark(map, make_anchor(2.0, 1));
  // A: {0,1,2}, B: {1,2,3}
  for (int i : {0, 1, 2}) {
    add_obs(map, i, A, 1, i);
  }
  for (int i : {1, 2, 3}) {
    add_obs(map, i, B, 2, i);
  }
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(1, 2), 2.0, 1e-12);  // both A and B

  // Consolidate to one observation per KF of canonical A; remove B.
  for (int i = 0; i < 4; ++i) {
    auto & kf = map.KeyFDATA[static_cast<std::size_t>(i)];
    kf.Matched_landmark_ids.clear();
    kf.UV_Matched_points.clear();
  }
  for (int i = 0; i < 4; ++i) {
    add_obs(map, i, A, 3, i);
  }
  std::size_t bidx = 0;
  ASSERT_TRUE(gmap_landmark_id::resolve_landmark_index(map, B, bidx));
  erase_landmark_at_storage_index(map, bidx, nullptr);
  gmap_landmark_id::rebuild_id_to_index(map);
  rebuild_exact_visibility_graph(map);
  EXPECT_NEAR(map.Vgraph(1, 2), 1.0, 1e-12);  // decreased
  EXPECT_NEAR(map.Vgraph(0, 3), 1.0, 1e-12);  // new cross
  expect_exact(map);
}

TEST(VisibilityGraphA11, ConsumerSensitivityExactVsInflated)
{
  GLOBAL_MAP exact;
  for (int i = 0; i < 6; ++i) {
    exact.KeyFDATA.push_back(make_kf(static_cast<uint64_t>(i + 1)));
  }
  const LandmarkId lid = insert_landmark(exact, make_anchor(1.0, 0));
  add_obs(exact, 0, lid);
  add_obs(exact, 5, lid);
  rebuild_exact_visibility_graph(exact);

  GLOBAL_MAP inflated = gmap_map_clone::deep_clone(exact);
  // Legacy-style inflation: bump (5,0) as if duplicate increments.
  inflated.Vgraph(5, 0) = 5.0;
  inflated.Vgraph(0, 5) = 5.0;

  auto e_exact = globalslam::cloop_visual_retrieve::enumerate_unlinked_historical(
    exact.Vgraph, 5, 5);
  auto e_infl = globalslam::cloop_visual_retrieve::enumerate_unlinked_historical(
    inflated.Vgraph, 5, 5);
  // Exact: V(5,0)=1 → KF0 not unlinked. Inflated also linked.
  EXPECT_EQ(
    std::find(e_exact.begin(), e_exact.end(), 0), e_exact.end());
  EXPECT_EQ(
    std::find(e_infl.begin(), e_infl.end(), 0), e_infl.end());

  // Stale zero (legacy incomplete assoc): pretend V(5,0)=0 while obs share lid.
  GLOBAL_MAP stale = gmap_map_clone::deep_clone(exact);
  stale.Vgraph(5, 0) = 0.0;
  stale.Vgraph(0, 5) = 0.0;
  auto e_stale = globalslam::cloop_visual_retrieve::enumerate_unlinked_historical(
    stale.Vgraph, 5, 5);
  EXPECT_NE(std::find(e_stale.begin(), e_stale.end(), 0), e_stale.end());
  // Exact semantics exclude that false candidate.
  EXPECT_EQ(std::find(e_exact.begin(), e_exact.end(), 0), e_exact.end());
}

TEST(VisibilityGraphA11, RepeatedMutationStress)
{
  GLOBAL_MAP map;
  std::mt19937 rng(42);
  for (int step = 0; step < 40; ++step) {
    map.KeyFDATA.push_back(make_kf(static_cast<uint64_t>(step + 1)));
    const int n_new = 3 + static_cast<int>(rng() % 4);
    for (int k = 0; k < n_new; ++k) {
      const LandmarkId lid =
        insert_landmark(map, make_anchor(step * 10 + k, step));
      add_obs(map, step, lid, 10 + k, step);
      if (step > 0 && (rng() % 2) == 0) {
        (void)try_insert_kf_observation(
          map, step - 1, lid, cv::Point2d(11 + k, step - 1), true);
      }
      // Duplicate attempt
      (void)try_insert_kf_observation(
        map, step, lid, cv::Point2d(10 + k, step), true);
    }
    if (step > 5 && (rng() % 3) == 0 && !map.AnchorsDATA.empty()) {
      const std::size_t idx = static_cast<std::size_t>(rng() % map.AnchorsDATA.size());
      erase_landmark_at_storage_index(map, idx, nullptr);
      gmap_landmark_id::rebuild_id_to_index(map);
    }
    rebuild_exact_visibility_graph(map);
    expect_exact(map);
  }
}

TEST(VisibilityGraphA11, RebuildBenchmarkScales)
{
  struct Scale
  {
    int n_kf;
    int n_lm;
    const char * name;
  };
  const Scale scales[] = {
    {50, 3000, "small"},
    {150, 8000, "mission"},
    {300, 15000, "extended"},
    {500, 30000, "stress"},
  };
  std::mt19937 rng(7);
  for (const auto & s : scales) {
    GLOBAL_MAP map;
    map.KeyFDATA.reserve(static_cast<std::size_t>(s.n_kf));
    for (int i = 0; i < s.n_kf; ++i) {
      map.KeyFDATA.push_back(make_kf(static_cast<uint64_t>(i + 1)));
    }
    // Assign each landmark to 2–6 random KFs (approx revisit degree).
    for (int l = 0; l < s.n_lm; ++l) {
      const LandmarkId lid = insert_landmark(map, make_anchor(l, 0));
      const int degree = 2 + static_cast<int>(rng() % 5);
      std::set<int> kfs;
      while (static_cast<int>(kfs.size()) < degree) {
        kfs.insert(static_cast<int>(rng() % s.n_kf));
      }
      int t = 0;
      for (int kf : kfs) {
        add_obs(map, kf, lid, 10 + t, 10 + (l % 50));
        ++t;
      }
    }
    std::vector<double> times;
    RebuildDiagnostics last{};
    for (int rep = 0; rep < 5; ++rep) {
      last = rebuild_exact_visibility_graph(map);
      times.push_back(last.duration_ms);
    }
    std::sort(times.begin(), times.end());
    const double med = times[times.size() / 2];
    const double p95 = times[static_cast<std::size_t>(0.95 * (times.size() - 1))];
    expect_exact(map);
    std::cout << "[benchmark] " << s.name << " kf=" << s.n_kf << " lm=" << s.n_lm
              << " obs_entries=" << last.n_obs_entries
              << " nonzero_edges=" << last.n_nonzero_edges
              << " med_ms=" << med << " p95_ms=" << p95
              << " max_ms=" << times.back() << std::endl;
    // Soft bound: complete rebuild should stay well under typical GMAP update.
    EXPECT_LT(med, 500.0) << s.name;
  }
}
