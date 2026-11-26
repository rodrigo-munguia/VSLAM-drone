/**
 * GSLAM-MAP-ARCH-M6: deterministic logical-map / BA-factor / writeback equivalence.
 *
 * Closes M3–M5 gaps that do not require Webots. Production BA still uses
 * Symbol('l', storage_index) after LandmarkId resolve; this test mirrors that
 * construction on a fixture where IDs differ from slots.
 */
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "gmap_landmark_identity.hpp"
#include "../../common/keyframe_types.hpp"

using gmap_landmark_id::assign_global_map_preserving_id_watermark;
using gmap_landmark_id::cull_underobserved_landmarks;
using gmap_landmark_id::erase_landmark_at_storage_index;
using gmap_landmark_id::insert_landmark;
using gmap_landmark_id::rebuild_id_to_index;
using gmap_landmark_id::resolve_landmark_index;
using gmap_landmark_id::validate_landmark_identity_invariants;

namespace
{

G_ANCHOR make_anchor(double x, int n_matched = 2, int n_tries = 0)
{
  G_ANCHOR a;
  a.feat_type = "g_anchor";
  a.AnchorState = {x, 0.0, 1.0};
  a.init_KF = 0;
  a.n_kf_matched = n_matched;
  a.n_tries_matchs = n_tries;
  a.Descriptor = cv::Mat::zeros(1, 32, CV_8U);
  a.Descriptor.at<unsigned char>(0, 0) = static_cast<unsigned char>(x);
  return a;
}

/** Same logical fixture as pre_migration_logical_fixture.json after middle delete. */
GLOBAL_MAP make_m6_fixture()
{
  GLOBAL_MAP map;
  map.KeyFDATA.resize(2);
  const LandmarkId a = insert_landmark(map, make_anchor(0));
  const LandmarkId b = insert_landmark(map, make_anchor(1));
  const LandmarkId c = insert_landmark(map, make_anchor(2));
  const LandmarkId d = insert_landmark(map, make_anchor(3));
  map.KeyFDATA[0].Matched_landmark_ids = {a, b, c, d};
  map.KeyFDATA[0].UV_Matched_points = {{1, 1}, {2, 2}, {3, 3}, {4, 4}};
  map.KeyFDATA[0].keyframe_id = 10;
  map.KeyFDATA[1].Matched_landmark_ids = {b, d};
  map.KeyFDATA[1].UV_Matched_points = {{5, 5}, {6, 6}};
  map.KeyFDATA[1].keyframe_id = 20;
  erase_landmark_at_storage_index(map, 1, nullptr);  // delete id b
  rebuild_id_to_index(map);
  return map;
}

struct BaFactor
{
  int idx_kf{0};
  std::size_t idx_pt{0};
  std::uint64_t landmark_id{0};
  double uv_x{0};
  double uv_y{0};
};

/** Mirror production BA observation→Symbol('l', slot) construction (no undistort). */
std::vector<BaFactor> build_ba_factors(
  const GLOBAL_MAP & map, const std::vector<int> & idx_kf_to_optimize)
{
  std::vector<BaFactor> factors;
  for (int idx_kf : idx_kf_to_optimize) {
    const auto & kf = map.KeyFDATA[static_cast<std::size_t>(idx_kf)];
    for (std::size_t j = 0; j < kf.Matched_landmark_ids.size(); ++j) {
      std::size_t idx_pt = 0;
      if (!resolve_landmark_index(map, kf.Matched_landmark_ids[j], idx_pt)) {
        continue;
      }
      BaFactor f;
      f.idx_kf = idx_kf;
      f.idx_pt = idx_pt;
      f.landmark_id = kf.Matched_landmark_ids[j].value;
      f.uv_x = kf.UV_Matched_points[j].x;
      f.uv_y = kf.UV_Matched_points[j].y;
      factors.push_back(f);
    }
  }
  return factors;
}

}  // namespace

TEST(LandmarkIdentityM6, LogicalMapMatchesPreMigrationFixture)
{
  GLOBAL_MAP map = make_m6_fixture();
  ASSERT_EQ(map.AnchorsDATA.size(), 3u);
  EXPECT_EQ(map.AnchorsDATA[0].id.value, 1u);
  EXPECT_EQ(map.AnchorsDATA[1].id.value, 3u);
  EXPECT_EQ(map.AnchorsDATA[2].id.value, 4u);
  ASSERT_EQ(map.KeyFDATA[0].Matched_landmark_ids.size(), 3u);
  EXPECT_EQ(map.KeyFDATA[0].Matched_landmark_ids[0].value, 1u);
  EXPECT_EQ(map.KeyFDATA[0].Matched_landmark_ids[1].value, 3u);
  EXPECT_EQ(map.KeyFDATA[0].Matched_landmark_ids[2].value, 4u);
  EXPECT_EQ(map.KeyFDATA[0].UV_Matched_points[0], cv::Point2d(1, 1));
  EXPECT_EQ(map.KeyFDATA[0].UV_Matched_points[1], cv::Point2d(3, 3));
  EXPECT_EQ(map.KeyFDATA[0].UV_Matched_points[2], cv::Point2d(4, 4));
  ASSERT_EQ(map.KeyFDATA[1].Matched_landmark_ids.size(), 1u);
  EXPECT_EQ(map.KeyFDATA[1].Matched_landmark_ids[0].value, 4u);
  EXPECT_EQ(map.KeyFDATA[1].UV_Matched_points[0], cv::Point2d(6, 6));
  const auto r = validate_landmark_identity_invariants(map);
  EXPECT_TRUE(r.ok) << r.reason;
}

TEST(LandmarkIdentityM6, BaFactorsUseStorageIndexKeysAfterResolve)
{
  GLOBAL_MAP map = make_m6_fixture();
  const std::vector<int> kfs = {0, 1};
  const auto factors = build_ba_factors(map, kfs);
  // Expected: KF0 observes ids 1,3,4 at slots 0,1,2; KF1 observes id 4 at slot 2
  ASSERT_EQ(factors.size(), 4u);
  EXPECT_EQ(factors[0].landmark_id, 1u);
  EXPECT_EQ(factors[0].idx_pt, 0u);
  EXPECT_EQ(factors[1].landmark_id, 3u);
  EXPECT_EQ(factors[1].idx_pt, 1u);
  EXPECT_EQ(factors[2].landmark_id, 4u);
  EXPECT_EQ(factors[2].idx_pt, 2u);
  EXPECT_EQ(factors[3].landmark_id, 4u);
  EXPECT_EQ(factors[3].idx_pt, 2u);
  EXPECT_EQ(factors[3].idx_kf, 1);
  // Symbol('l', idx_pt) keys equal storage indices (policy preserved).
  for (const auto & f : factors) {
    EXPECT_EQ(map.AnchorsDATA[f.idx_pt].id.value, f.landmark_id);
  }
}

TEST(LandmarkIdentityM6, BaWritebackTargetsLogicalLandmark)
{
  GLOBAL_MAP map = make_m6_fixture();
  // Simulate optimized result for Symbol('l', 1) → write back slot 1 (id 3).
  const std::size_t slot = 1;
  const LandmarkId intended = map.AnchorsDATA[slot].id;
  EXPECT_EQ(intended.value, 3u);
  map.AnchorsDATA[slot].AnchorState[0] = 42.0;
  map.AnchorsDATA[slot].AnchorState[1] = 43.0;
  map.AnchorsDATA[slot].AnchorState[2] = 44.0;
  std::size_t resolved = 999;
  ASSERT_TRUE(resolve_landmark_index(map, intended, resolved));
  EXPECT_EQ(resolved, slot);
  EXPECT_DOUBLE_EQ(map.AnchorsDATA[resolved].AnchorState[0], 42.0);
  // Neighbor IDs unchanged.
  EXPECT_EQ(map.AnchorsDATA[0].id.value, 1u);
  EXPECT_EQ(map.AnchorsDATA[2].id.value, 4u);
  EXPECT_DOUBLE_EQ(map.AnchorsDATA[0].AnchorState[0], 0.0);
}

TEST(LandmarkIdentityM6, PnPCorrespondenceOrderFromPersistentIds)
{
  GLOBAL_MAP map = make_m6_fixture();
  // Historical KF0 observations → ordered 3D–2D rows (production Collect pattern).
  std::vector<cv::Point3d> obj;
  std::vector<cv::Point2d> img;
  std::vector<std::uint64_t> ids;
  for (std::size_t j = 0; j < map.KeyFDATA[0].Matched_landmark_ids.size(); ++j) {
    std::size_t slot = 0;
    ASSERT_TRUE(resolve_landmark_index(
      map, map.KeyFDATA[0].Matched_landmark_ids[j], slot));
    const auto & A = map.AnchorsDATA[slot];
    obj.emplace_back(A.AnchorState[0], A.AnchorState[1], A.AnchorState[2]);
    img.push_back(map.KeyFDATA[0].UV_Matched_points[j]);
    ids.push_back(A.id.value);
  }
  ASSERT_EQ(obj.size(), 3u);
  EXPECT_EQ(ids[0], 1u);
  EXPECT_EQ(ids[1], 3u);
  EXPECT_EQ(ids[2], 4u);
  EXPECT_DOUBLE_EQ(obj[1].x, 2.0);  // id 3 was born at x=2
  EXPECT_EQ(img[1], cv::Point2d(3, 3));
}

TEST(LandmarkIdentityM6, CullSeamPreservesSurvivingObservationIds)
{
  GLOBAL_MAP map;
  map.KeyFDATA.resize(1);
  const LandmarkId a = insert_landmark(map, make_anchor(0, 5, 0));
  const LandmarkId b = insert_landmark(map, make_anchor(1, 1, 10));  // cullable
  const LandmarkId c = insert_landmark(map, make_anchor(2, 5, 0));
  map.KeyFDATA[0].Matched_landmark_ids = {a, b, c};
  map.KeyFDATA[0].UV_Matched_points = {{1, 1}, {2, 2}, {3, 3}};
  std::vector<std::vector<int>> vg;
  int n_del = 0;
  cull_underobserved_landmarks(map, 3, vg, &n_del);
  EXPECT_EQ(n_del, 1);
  ASSERT_EQ(map.KeyFDATA[0].Matched_landmark_ids.size(), 2u);
  EXPECT_EQ(map.KeyFDATA[0].Matched_landmark_ids[0], a);
  EXPECT_EQ(map.KeyFDATA[0].Matched_landmark_ids[1], c);
  EXPECT_EQ(map.KeyFDATA[0].UV_Matched_points[0], cv::Point2d(1, 1));
  EXPECT_EQ(map.KeyFDATA[0].UV_Matched_points[1], cv::Point2d(3, 3));
  const auto r = validate_landmark_identity_invariants(map);
  EXPECT_TRUE(r.ok) << r.reason;
}

TEST(LandmarkIdentityM6, PoseSlamDenseScanPreservesIds)
{
  // Pose_SLAM corrects by init_KF / dense AnchorsDATA; IDs must not change.
  GLOBAL_MAP map = make_m6_fixture();
  const std::vector<std::uint64_t> before = {
    map.AnchorsDATA[0].id.value, map.AnchorsDATA[1].id.value,
    map.AnchorsDATA[2].id.value};
  for (auto & a : map.AnchorsDATA) {
    a.AnchorState[0] += 0.5;  // geometric correction only
  }
  EXPECT_EQ(map.AnchorsDATA[0].id.value, before[0]);
  EXPECT_EQ(map.AnchorsDATA[1].id.value, before[1]);
  EXPECT_EQ(map.AnchorsDATA[2].id.value, before[2]);
}

TEST(LandmarkIdentityM6, MapCopyPreservesObservationsAndWatermark)
{
  GLOBAL_MAP map = make_m6_fixture();
  const auto next_before = map.next_landmark_id;
  GLOBAL_MAP copy;
  copy.next_landmark_id = next_before + 10;
  assign_global_map_preserving_id_watermark(copy, map);
  EXPECT_EQ(copy.next_landmark_id, next_before + 10);
  EXPECT_EQ(copy.KeyFDATA[0].Matched_landmark_ids, map.KeyFDATA[0].Matched_landmark_ids);
  const auto r = validate_landmark_identity_invariants(copy);
  EXPECT_TRUE(r.ok) << r.reason;
}
