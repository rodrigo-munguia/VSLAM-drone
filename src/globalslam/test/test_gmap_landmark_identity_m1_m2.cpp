/**
 * GSLAM-MAP-ARCH-M1/M2: stable LandmarkId + id_to_index lifecycle regressions.
 */
#include <cstdint>
#include <set>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "gmap_landmark_identity.hpp"

using gmap_landmark_id::allocate_landmark_id;
using gmap_landmark_id::assign_global_map_preserving_id_watermark;
using gmap_landmark_id::clear_landmark_storage;
using gmap_landmark_id::contains_landmark;
using gmap_landmark_id::erase_storage_index_preserving_order;
using gmap_landmark_id::insert_landmark;
using gmap_landmark_id::kInvalidId;
using gmap_landmark_id::rebuild_id_to_index;
using gmap_landmark_id::resolve_landmark_index;
using gmap_landmark_id::validate_landmark_identity_invariants;

namespace
{

G_ANCHOR make_anchor(double x)
{
  G_ANCHOR a;
  a.feat_type = "g_anchor";
  a.AnchorState = {x, 0.0, 1.0};
  a.init_KF = 0;
  a.n_kf_matched = 2;
  a.n_tries_matchs = 0;
  return a;
}

void expect_invariants(const GLOBAL_MAP & map)
{
  const auto r = validate_landmark_identity_invariants(map);
  EXPECT_TRUE(r.ok) << r.reason;
}

}  // namespace

TEST(LandmarkIdentityM1M2, StrongTypingNoImplicitConversion)
{
  static_assert(!std::is_convertible<std::size_t, LandmarkId>::value, "");
  static_assert(!std::is_convertible<int, LandmarkId>::value, "");
  static_assert(!std::is_convertible<LandmarkId, std::size_t>::value, "");
  static_assert(!std::is_convertible<LandmarkId, int>::value, "");
  LandmarkId a{3};
  LandmarkId b{3};
  LandmarkId c{4};
  EXPECT_EQ(a, b);
  EXPECT_NE(a, c);
  LandmarkIdHash h;
  EXPECT_EQ(h(a), h(b));
}

TEST(LandmarkIdentityM1M2, EmptyMap)
{
  GLOBAL_MAP map;
  EXPECT_TRUE(map.AnchorsDATA.empty());
  EXPECT_TRUE(map.id_to_index.empty());
  EXPECT_EQ(map.next_landmark_id, 1u);
  std::size_t idx = 99;
  EXPECT_FALSE(resolve_landmark_index(map, kInvalidId, idx));
  EXPECT_EQ(idx, 99u);
  EXPECT_FALSE(contains_landmark(map, LandmarkId{1}));
  expect_invariants(map);
}

TEST(LandmarkIdentityM1M2, SequentialInsertion)
{
  GLOBAL_MAP map;
  const LandmarkId id0 = insert_landmark(map, make_anchor(0));
  const LandmarkId id1 = insert_landmark(map, make_anchor(1));
  const LandmarkId id2 = insert_landmark(map, make_anchor(2));
  EXPECT_TRUE(gmap_landmark_id::is_valid(id0));
  EXPECT_EQ(id0.value, 1u);
  EXPECT_EQ(id1.value, 2u);
  EXPECT_EQ(id2.value, 3u);
  EXPECT_EQ(map.next_landmark_id, 4u);
  EXPECT_EQ(map.AnchorsDATA.size(), 3u);
  EXPECT_EQ(map.AnchorsDATA[0].id, id0);
  EXPECT_EQ(map.AnchorsDATA[1].id, id1);
  EXPECT_EQ(map.AnchorsDATA[2].id, id2);
  std::size_t i = 0;
  ASSERT_TRUE(resolve_landmark_index(map, id1, i));
  EXPECT_EQ(i, 1u);
  expect_invariants(map);
}

TEST(LandmarkIdentityM1M2, VectorReallocationPreservesIds)
{
  GLOBAL_MAP map;
  map.AnchorsDATA.reserve(2);
  std::vector<LandmarkId> ids;
  for (int n = 0; n < 64; ++n) {
    ids.push_back(insert_landmark(map, make_anchor(static_cast<double>(n))));
  }
  for (std::size_t i = 0; i < ids.size(); ++i) {
    EXPECT_EQ(map.AnchorsDATA[i].id, ids[i]);
    std::size_t slot = 0;
    ASSERT_TRUE(resolve_landmark_index(map, ids[i], slot));
    EXPECT_EQ(slot, i);
  }
  expect_invariants(map);
}

TEST(LandmarkIdentityM1M2, SingleMiddleDeletion)
{
  GLOBAL_MAP map;
  const LandmarkId a = insert_landmark(map, make_anchor(0));
  const LandmarkId b = insert_landmark(map, make_anchor(1));
  const LandmarkId c = insert_landmark(map, make_anchor(2));
  const LandmarkId d = insert_landmark(map, make_anchor(3));
  erase_storage_index_preserving_order(map, 1, nullptr);
  rebuild_id_to_index(map);
  EXPECT_FALSE(contains_landmark(map, b));
  EXPECT_EQ(map.AnchorsDATA.size(), 3u);
  EXPECT_EQ(map.AnchorsDATA[0].id, a);
  EXPECT_EQ(map.AnchorsDATA[1].id, c);
  EXPECT_EQ(map.AnchorsDATA[2].id, d);
  std::size_t slot = 0;
  ASSERT_TRUE(resolve_landmark_index(map, c, slot));
  EXPECT_EQ(slot, 1u);
  expect_invariants(map);
}

TEST(LandmarkIdentityM1M2, BatchDeletionFirstMiddleLast)
{
  GLOBAL_MAP map;
  std::vector<LandmarkId> ids;
  for (int i = 0; i < 5; ++i) {
    ids.push_back(insert_landmark(map, make_anchor(i)));
  }
  // Delete last, middle, first (high indices first like Delete_points).
  erase_storage_index_preserving_order(map, 4, nullptr);
  erase_storage_index_preserving_order(map, 2, nullptr);
  erase_storage_index_preserving_order(map, 0, nullptr);
  rebuild_id_to_index(map);
  EXPECT_EQ(map.AnchorsDATA.size(), 2u);
  EXPECT_EQ(map.AnchorsDATA[0].id, ids[1]);
  EXPECT_EQ(map.AnchorsDATA[1].id, ids[3]);
  EXPECT_FALSE(contains_landmark(map, ids[0]));
  EXPECT_FALSE(contains_landmark(map, ids[2]));
  EXPECT_FALSE(contains_landmark(map, ids[4]));
  expect_invariants(map);
}

TEST(LandmarkIdentityM1M2, DeleteAndReinsertNoReuse)
{
  GLOBAL_MAP map;
  const LandmarkId a = insert_landmark(map, make_anchor(0));
  const LandmarkId b = insert_landmark(map, make_anchor(1));
  erase_storage_index_preserving_order(map, 0, nullptr);
  rebuild_id_to_index(map);
  const LandmarkId c = insert_landmark(map, make_anchor(9));
  EXPECT_NE(c, a);
  EXPECT_GT(c.value, b.value);
  EXPECT_FALSE(contains_landmark(map, a));
  expect_invariants(map);
}

TEST(LandmarkIdentityM1M2, ObservationIdsSurviveDeletionWithoutRenumber)
{
  // Updated for M3–M5: persistent refs are LandmarkIds; survivors not renumbered.
  GLOBAL_MAP map;
  KEYFRAME kf0;
  KEYFRAME kf1;
  map.KeyFDATA.push_back(kf0);
  map.KeyFDATA.push_back(kf1);
  const LandmarkId id0 = insert_landmark(map, make_anchor(0));
  const LandmarkId id1 = insert_landmark(map, make_anchor(1));
  const LandmarkId id2 = insert_landmark(map, make_anchor(2));
  map.KeyFDATA[0].Matched_landmark_ids = {id0, id1, id2};
  map.KeyFDATA[0].UV_Matched_points = {{1, 1}, {2, 2}, {3, 3}};
  map.KeyFDATA[1].Matched_landmark_ids = {id1, id2};
  map.KeyFDATA[1].UV_Matched_points = {{4, 4}, {5, 5}};

  erase_storage_index_preserving_order(map, 1, nullptr);
  rebuild_id_to_index(map);

  ASSERT_EQ(map.KeyFDATA[0].Matched_landmark_ids.size(), 2u);
  EXPECT_EQ(map.KeyFDATA[0].Matched_landmark_ids[0], id0);
  EXPECT_EQ(map.KeyFDATA[0].Matched_landmark_ids[1], id2);
  EXPECT_EQ(map.KeyFDATA[0].UV_Matched_points[0], cv::Point2d(1, 1));
  EXPECT_EQ(map.KeyFDATA[0].UV_Matched_points[1], cv::Point2d(3, 3));
  ASSERT_EQ(map.KeyFDATA[1].Matched_landmark_ids.size(), 1u);
  EXPECT_EQ(map.KeyFDATA[1].Matched_landmark_ids[0], id2);
  EXPECT_EQ(map.KeyFDATA[1].UV_Matched_points[0], cv::Point2d(5, 5));
  EXPECT_EQ(map.AnchorsDATA[0].id, id0);
  EXPECT_EQ(map.AnchorsDATA[1].id, id2);
  expect_invariants(map);
}

TEST(LandmarkIdentityM1M2, MapCopyIndependentMetadata)
{
  GLOBAL_MAP a;
  insert_landmark(a, make_anchor(0));
  insert_landmark(a, make_anchor(1));
  GLOBAL_MAP b = a;
  rebuild_id_to_index(b);
  EXPECT_EQ(b.AnchorsDATA[0].id, a.AnchorsDATA[0].id);
  EXPECT_EQ(b.next_landmark_id, a.next_landmark_id);
  erase_storage_index_preserving_order(b, 0, nullptr);
  rebuild_id_to_index(b);
  EXPECT_EQ(a.AnchorsDATA.size(), 2u);
  EXPECT_EQ(b.AnchorsDATA.size(), 1u);
  EXPECT_TRUE(contains_landmark(a, a.AnchorsDATA[0].id));
  expect_invariants(a);
  expect_invariants(b);
}

TEST(LandmarkIdentityM1M2, MapReplacementAllocatorRollbackPrevention)
{
  GLOBAL_MAP authoritative;
  insert_landmark(authoritative, make_anchor(0));  // id 1
  GLOBAL_MAP snapshot = authoritative;
  // Authoritative allocates further while snapshot is held.
  insert_landmark(authoritative, make_anchor(1));  // id 2
  insert_landmark(authoritative, make_anchor(2));  // id 3
  EXPECT_EQ(authoritative.next_landmark_id, 4u);
  EXPECT_EQ(snapshot.next_landmark_id, 2u);

  // Older snapshot committed into authoritative.
  assign_global_map_preserving_id_watermark(authoritative, snapshot);
  EXPECT_EQ(authoritative.AnchorsDATA.size(), 1u);
  EXPECT_EQ(authoritative.next_landmark_id, 4u);  // watermark preserved
  const LandmarkId next = insert_landmark(authoritative, make_anchor(9));
  EXPECT_EQ(next.value, 4u);
  EXPECT_NE(next.value, 2u);
  EXPECT_NE(next.value, 3u);
  expect_invariants(authoritative);
}

TEST(LandmarkIdentityM1M2, MapResetPreservesWatermark)
{
  GLOBAL_MAP map;
  insert_landmark(map, make_anchor(0));
  insert_landmark(map, make_anchor(1));
  const auto next_before = map.next_landmark_id;
  clear_landmark_storage(map);
  map.KeyFDATA.clear();
  EXPECT_TRUE(map.AnchorsDATA.empty());
  EXPECT_TRUE(map.id_to_index.empty());
  EXPECT_EQ(map.next_landmark_id, next_before);
  const LandmarkId id = insert_landmark(map, make_anchor(5));
  EXPECT_EQ(id.value, next_before);
  expect_invariants(map);
}

TEST(LandmarkIdentityM1M2, InvalidLookupDoesNotInsert)
{
  GLOBAL_MAP map;
  insert_landmark(map, make_anchor(0));
  const auto size_before = map.id_to_index.size();
  std::size_t idx = 0;
  EXPECT_FALSE(resolve_landmark_index(map, kInvalidId, idx));
  EXPECT_FALSE(resolve_landmark_index(map, LandmarkId{999}, idx));
  EXPECT_EQ(map.id_to_index.size(), size_before);
}

TEST(LandmarkIdentityM1M2, GeometricMutationKeepsIdentity)
{
  GLOBAL_MAP map;
  const LandmarkId id = insert_landmark(map, make_anchor(0));
  map.AnchorsDATA[0].AnchorState = {9.0, 8.0, 7.0};
  EXPECT_EQ(map.AnchorsDATA[0].id, id);
  std::size_t slot = 0;
  ASSERT_TRUE(resolve_landmark_index(map, id, slot));
  EXPECT_EQ(slot, 0u);
  expect_invariants(map);
}

TEST(LandmarkIdentityM1M2, MixedLifecycleSequence)
{
  GLOBAL_MAP map;
  std::set<std::uint64_t> ever_allocated;
  std::vector<LandmarkId> active;
  for (int step = 0; step < 200; ++step) {
    if (step % 7 != 0 || active.empty()) {
      const LandmarkId id = insert_landmark(map, make_anchor(step));
      ASSERT_TRUE(ever_allocated.insert(id.value).second) << "reuse " << id.value;
      active.push_back(id);
    } else {
      // Delete a middle-ish active by storage index of first active id.
      std::size_t slot = 0;
      ASSERT_TRUE(resolve_landmark_index(map, active[active.size() / 2], slot));
      erase_storage_index_preserving_order(map, slot, nullptr);
      rebuild_id_to_index(map);
      active.erase(active.begin() + static_cast<std::ptrdiff_t>(active.size() / 2));
      // Rebuild active list from map to stay consistent with order.
      active.clear();
      for (const auto & a : map.AnchorsDATA) {
        active.push_back(a.id);
      }
    }
    expect_invariants(map);
  }
  EXPECT_GE(ever_allocated.size(), 100u);
}
