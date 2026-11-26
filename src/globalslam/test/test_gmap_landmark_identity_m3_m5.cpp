/**
 * GSLAM-MAP-ARCH-M3/M4/M5: persistent LandmarkId observations + no remapping.
 */
#include <set>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "gmap_landmark_identity.hpp"
#include "../../common/keyframe_types.hpp"

using gmap_landmark_id::assign_global_map_preserving_id_watermark;
using gmap_landmark_id::clear_landmark_storage;
using gmap_landmark_id::cull_underobserved_landmarks;
using gmap_landmark_id::erase_landmark_at_storage_index;
using gmap_landmark_id::insert_landmark;
using gmap_landmark_id::rebuild_id_to_index;
using gmap_landmark_id::resolve_landmark_index;
using gmap_landmark_id::validate_landmark_identity_invariants;

namespace
{

G_ANCHOR make_anchor(double x, int init_kf = 0)
{
  G_ANCHOR a;
  a.feat_type = "g_anchor";
  a.AnchorState = {x, 0.0, 1.0};
  a.init_KF = init_kf;
  a.n_kf_matched = 2;
  a.n_tries_matchs = 0;
  a.Descriptor = cv::Mat::zeros(1, 32, CV_8U);
  return a;
}

void expect_ok(const GLOBAL_MAP & map)
{
  const auto r = validate_landmark_identity_invariants(map);
  EXPECT_TRUE(r.ok) << r.reason;
}

/** Build map where IDs differ from slots after a deletion. */
GLOBAL_MAP make_fixture_ids_differ_from_slots()
{
  GLOBAL_MAP map;
  map.KeyFDATA.resize(2);
  const LandmarkId a = insert_landmark(map, make_anchor(0));
  const LandmarkId b = insert_landmark(map, make_anchor(1));
  const LandmarkId c = insert_landmark(map, make_anchor(2));
  const LandmarkId d = insert_landmark(map, make_anchor(3));
  map.KeyFDATA[0].Matched_landmark_ids = {a, b, c, d};
  map.KeyFDATA[0].UV_Matched_points = {{1, 1}, {2, 2}, {3, 3}, {4, 4}};
  map.KeyFDATA[1].Matched_landmark_ids = {b, d};
  map.KeyFDATA[1].UV_Matched_points = {{5, 5}, {6, 6}};
  // Delete slot 1 (id b) so survivors: a,c,d at slots 0,1,2 with ids 1,3,4
  erase_landmark_at_storage_index(map, 1, nullptr);
  rebuild_id_to_index(map);
  EXPECT_NE(map.AnchorsDATA[1].id.value, 1u);  // slot 1 is not id 1
  return map;
}

}  // namespace

TEST(LandmarkIdentityM3M5, StrongTypingObservationField)
{
  static_assert(
    std::is_same<decltype(KEYFRAME{}.Matched_landmark_ids),
      std::vector<LandmarkId>>::value,
    "");
  static_assert(!std::is_convertible<std::size_t, LandmarkId>::value, "");
}

TEST(LandmarkIdentityM3M5, ObservationResolvesWhenIdsDifferFromSlots)
{
  GLOBAL_MAP map = make_fixture_ids_differ_from_slots();
  expect_ok(map);
  for (const LandmarkId lid : map.KeyFDATA[0].Matched_landmark_ids) {
    std::size_t slot = 0;
    ASSERT_TRUE(resolve_landmark_index(map, lid, slot));
    EXPECT_EQ(map.AnchorsDATA[slot].id, lid);
    EXPECT_DOUBLE_EQ(map.AnchorsDATA[slot].AnchorState[2], 1.0);
  }
}

TEST(LandmarkIdentityM3M5, ObservationUvPairingAfterCopy)
{
  GLOBAL_MAP map = make_fixture_ids_differ_from_slots();
  GLOBAL_MAP copy;
  assign_global_map_preserving_id_watermark(copy, map);
  ASSERT_EQ(copy.KeyFDATA[0].Matched_landmark_ids.size(),
    copy.KeyFDATA[0].UV_Matched_points.size());
  EXPECT_EQ(copy.KeyFDATA[0].UV_Matched_points[0], cv::Point2d(1, 1));
  expect_ok(copy);
}

TEST(LandmarkIdentityM3M5, SingleMiddleDeletionNoRenumber)
{
  GLOBAL_MAP map;
  map.KeyFDATA.resize(1);
  const LandmarkId a = insert_landmark(map, make_anchor(0));
  const LandmarkId b = insert_landmark(map, make_anchor(1));
  const LandmarkId c = insert_landmark(map, make_anchor(2));
  map.KeyFDATA[0].Matched_landmark_ids = {a, b, c};
  map.KeyFDATA[0].UV_Matched_points = {{1, 1}, {2, 2}, {3, 3}};
  erase_landmark_at_storage_index(map, 1, nullptr);
  rebuild_id_to_index(map);
  ASSERT_EQ(map.KeyFDATA[0].Matched_landmark_ids.size(), 2u);
  EXPECT_EQ(map.KeyFDATA[0].Matched_landmark_ids[0], a);
  EXPECT_EQ(map.KeyFDATA[0].Matched_landmark_ids[1], c);
  EXPECT_FALSE(gmap_landmark_id::contains_landmark(map, b));
  expect_ok(map);
}

TEST(LandmarkIdentityM3M5, BatchDeletionFirstMiddleLast)
{
  GLOBAL_MAP map;
  map.KeyFDATA.resize(1);
  std::vector<LandmarkId> ids;
  for (int i = 0; i < 5; ++i) {
    ids.push_back(insert_landmark(map, make_anchor(i)));
  }
  map.KeyFDATA[0].Matched_landmark_ids = ids;
  map.KeyFDATA[0].UV_Matched_points.assign(5, cv::Point2d(0, 0));
  erase_landmark_at_storage_index(map, 4, nullptr);
  erase_landmark_at_storage_index(map, 2, nullptr);
  erase_landmark_at_storage_index(map, 0, nullptr);
  rebuild_id_to_index(map);
  ASSERT_EQ(map.KeyFDATA[0].Matched_landmark_ids.size(), 2u);
  EXPECT_EQ(map.KeyFDATA[0].Matched_landmark_ids[0], ids[1]);
  EXPECT_EQ(map.KeyFDATA[0].Matched_landmark_ids[1], ids[3]);
  expect_ok(map);
}

TEST(LandmarkIdentityM3M5, ProductionCullSeam)
{
  GLOBAL_MAP map;
  map.KeyFDATA.resize(1);
  const LandmarkId keep = insert_landmark(map, make_anchor(0));
  const LandmarkId drop = insert_landmark(map, make_anchor(1));
  map.AnchorsDATA[0].n_kf_matched = 5;
  map.AnchorsDATA[0].n_tries_matchs = 5;
  map.AnchorsDATA[1].n_kf_matched = 1;  // < 3
  map.AnchorsDATA[1].n_tries_matchs = 5;  // > 0
  map.KeyFDATA[0].Matched_landmark_ids = {keep, drop};
  map.KeyFDATA[0].UV_Matched_points = {{1, 1}, {2, 2}};
  std::vector<std::vector<int>> vg;
  int n_del = 0;
  cull_underobserved_landmarks(map, /*min_keep=*/3, vg, &n_del);
  EXPECT_EQ(n_del, 1);
  EXPECT_FALSE(gmap_landmark_id::contains_landmark(map, drop));
  ASSERT_EQ(map.KeyFDATA[0].Matched_landmark_ids.size(), 1u);
  EXPECT_EQ(map.KeyFDATA[0].Matched_landmark_ids[0], keep);
  expect_ok(map);
}

TEST(LandmarkIdentityM3M5, InvalidPersistentReferenceDetected)
{
  GLOBAL_MAP map = make_fixture_ids_differ_from_slots();
  map.KeyFDATA[0].Matched_landmark_ids.push_back(LandmarkId{99999});
  map.KeyFDATA[0].UV_Matched_points.push_back({9, 9});
  const auto r = validate_landmark_identity_invariants(map);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason, "obs_unresolved_id");
}

TEST(LandmarkIdentityM3M5, BaSymbolUsesStorageIndexAfterResolve)
{
  GLOBAL_MAP map = make_fixture_ids_differ_from_slots();
  std::vector<std::size_t> keys;
  for (const LandmarkId lid : map.KeyFDATA[0].Matched_landmark_ids) {
    std::size_t slot = 0;
    ASSERT_TRUE(resolve_landmark_index(map, lid, slot));
    keys.push_back(slot);
  }
  // Keys are dense storage indices (0..n-1), not necessarily equal to ID values.
  for (std::size_t i = 0; i < keys.size(); ++i) {
    EXPECT_EQ(keys[i], i);
    EXPECT_NE(map.AnchorsDATA[keys[i]].id.value, 0u);
  }
}

TEST(LandmarkIdentityM3M5, VisibilitySharedCountById)
{
  GLOBAL_MAP map;
  map.KeyFDATA.resize(2);
  const LandmarkId a = insert_landmark(map, make_anchor(0));
  const LandmarkId b = insert_landmark(map, make_anchor(1));
  map.KeyFDATA[0].Matched_landmark_ids = {a, b};
  map.KeyFDATA[0].UV_Matched_points = {{1, 1}, {2, 2}};
  map.KeyFDATA[1].Matched_landmark_ids = {b};
  map.KeyFDATA[1].UV_Matched_points = {{3, 3}};
  int shared = 0;
  for (LandmarkId lid : map.KeyFDATA[0].Matched_landmark_ids) {
    for (LandmarkId other : map.KeyFDATA[1].Matched_landmark_ids) {
      if (lid == other) {
        ++shared;
      }
    }
  }
  EXPECT_EQ(shared, 1);
}

TEST(LandmarkIdentityM3M5, MixedLifecycleNoReuse)
{
  GLOBAL_MAP map;
  map.KeyFDATA.resize(1);
  std::set<std::uint64_t> ever;
  for (int step = 0; step < 100; ++step) {
    const LandmarkId id = insert_landmark(map, make_anchor(step));
    ASSERT_TRUE(ever.insert(id.value).second);
    map.KeyFDATA[0].Matched_landmark_ids.push_back(id);
    map.KeyFDATA[0].UV_Matched_points.push_back(
      cv::Point2d(step, step));
    if (step % 11 == 10 && map.AnchorsDATA.size() > 2) {
      erase_landmark_at_storage_index(map, 1, nullptr);
      rebuild_id_to_index(map);
    }
    expect_ok(map);
  }
}

TEST(LandmarkIdentityM3M5, ClearRemovesObservations)
{
  GLOBAL_MAP map = make_fixture_ids_differ_from_slots();
  const auto next = map.next_landmark_id;
  clear_landmark_storage(map);
  map.KeyFDATA.clear();
  EXPECT_TRUE(map.AnchorsDATA.empty());
  EXPECT_EQ(map.next_landmark_id, next);
}
