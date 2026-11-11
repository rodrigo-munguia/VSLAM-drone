#include <gtest/gtest.h>

#include "g3_admission.hpp"

using namespace globalslam;

static slam_identity::UuidBytes sid(uint8_t fill)
{
  slam_identity::UuidBytes u{};
  u.fill(fill);
  u[6] = 0x40;
  u[8] = 0x80;
  return u;
}

TEST(G3Admission, SameSessionNewEpochClears)
{
  ActiveReference cur;
  auto s = sid(1);
  apply_reference_update(
    cur, RefUpdateAction::IdentityChange, true, s, 1);
  EXPECT_TRUE(cur.valid);
  auto action = classify_reference_update(cur, true, s, 2);
  EXPECT_EQ(action, RefUpdateAction::IdentityChange);
}

TEST(G3Admission, NewSessionReusingEpochOne)
{
  ActiveReference cur;
  auto s1 = sid(1);
  auto s2 = sid(2);
  apply_reference_update(cur, RefUpdateAction::IdentityChange, true, s1, 1);
  auto action = classify_reference_update(cur, true, s2, 1);
  EXPECT_EQ(action, RefUpdateAction::IdentityChange);
  apply_reference_update(cur, action, true, s2, 1);
  // Delayed old session cannot switch back.
  EXPECT_EQ(
    classify_reference_update(cur, true, s1, 1),
    RefUpdateAction::RejectStaleSession);
}

TEST(G3Admission, MetadataOnlySameEpoch)
{
  ActiveReference cur;
  auto s = sid(4);
  apply_reference_update(cur, RefUpdateAction::IdentityChange, true, s, 3);
  EXPECT_EQ(classify_reference_update(cur, true, s, 3), RefUpdateAction::MetadataOnly);
}

TEST(G3Admission, InvalidateThenKeyframeDropped)
{
  ActiveReference cur;
  auto s = sid(5);
  apply_reference_update(cur, RefUpdateAction::IdentityChange, true, s, 1);
  apply_reference_update(cur, RefUpdateAction::Invalidate, false, s, 1);
  EXPECT_FALSE(cur.valid);
  std::unordered_set<uint64_t> seen;
  auto r = admit_keyframe(
    cur, s, 1, 1, 0, true, false, 240, 400, 240, 400, seen);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.reason, "no_valid_reference");
}

TEST(G3Admission, KeyframePolicies)
{
  ActiveReference cur;
  auto s = sid(6);
  apply_reference_update(cur, RefUpdateAction::IdentityChange, true, s, 2);
  std::unordered_set<uint64_t> seen;
  auto ok = admit_keyframe(
    cur, s, 2, 10, 0, true, false, 240, 400, 240, 400, seen);
  EXPECT_TRUE(ok.accepted);
  auto dup = admit_keyframe(
    cur, s, 2, 10, 0, true, false, 240, 400, 240, 400, seen);
  EXPECT_EQ(dup.reason, "duplicate_keyframe");
  auto miss = admit_keyframe(
    cur, s, 2, 11, 0, false, false, 240, 400, 240, 400, seen);
  EXPECT_EQ(miss.reason, "missing_source_stamp");
  auto cal = admit_keyframe(
    cur, s, 2, 12, 0, true, false, 240, 320, 240, 400, seen);
  EXPECT_EQ(cal.reason, "calibration_resolution_mismatch");
  auto stale = admit_keyframe(
    cur, s, 1, 13, 0, true, false, 240, 400, 240, 400, seen);
  EXPECT_EQ(stale.reason, "epoch_mismatch");
}
