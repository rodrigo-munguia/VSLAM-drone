#include <gtest/gtest.h>

#include "g6_feedback_barrier.hpp"

using control::g6_feedback::AcceptedSample;
using control::g6_feedback::MetaSample;
using control::g6_feedback::PendingBarrier;
using control::g6_feedback::accept_gt;
using control::g6_feedback::accept_odometry;
using control::g6_feedback::accepted_satisfies;
using control::g6_feedback::arm_from_applied;
using control::g6_feedback::find_meta_for_stamp;
using control::g6_feedback::meta_satisfies;
using control::g6_feedback::on_estimator_reset;
using control::g6_feedback::push_accepted;
using control::g6_feedback::push_bounded_meta;
using control::g6_feedback::try_accept_from_history;

static slam_identity::UuidBytes sid(uint8_t fill = 7)
{
  slam_identity::UuidBytes id;
  id.fill(fill);
  id[6] = 0x40;
  id[8] = 0x80;
  return id;
}

static MetaSample meta(
  uint64_t seq, uint64_t corr, int32_t sec = 1, uint32_t nsec = 0,
  uint64_t epoch = 1)
{
  MetaSample m;
  m.session_id = sid();
  m.epoch_id = epoch;
  m.snapshot_seq = seq;
  m.last_correction_id = corr;
  m.stamp_sec = sec;
  m.stamp_nanosec = nsec;
  return m;
}

TEST(G6FeedbackBarrier, ArmAndMetaSatisfiesExactOrNewer)
{
  PendingBarrier b;
  ASSERT_TRUE(arm_from_applied(b, sid(), 1, 42, 10));
  EXPECT_TRUE(meta_satisfies(b, meta(10, 42)));
  EXPECT_TRUE(meta_satisfies(b, meta(11, 42)));  // newer compatible
  EXPECT_FALSE(meta_satisfies(b, meta(9, 42)));   // older
  EXPECT_FALSE(meta_satisfies(b, meta(10, 99)));  // wrong correction
  auto stale = meta(10, 42);
  stale.epoch_id = 2;
  EXPECT_FALSE(meta_satisfies(b, stale));
}

TEST(G6FeedbackBarrier, GtAcceptWithoutOdometry)
{
  PendingBarrier b;
  ASSERT_TRUE(arm_from_applied(b, sid(), 1, 7, 3));
  EXPECT_TRUE(accept_gt(b));
  EXPECT_TRUE(b.accepted);
  EXPECT_EQ(b.feedback_mode, "gt");
  EXPECT_FALSE(accept_gt(b));  // idempotent no-op
}

TEST(G6FeedbackBarrier, OdomBeforeEventFromHistory)
{
  PendingBarrier b;
  std::deque<AcceptedSample> hist;
  AcceptedSample a;
  a.session_id = sid();
  a.epoch_id = 1;
  a.snapshot_seq = 5;
  a.last_correction_id = 100;
  push_accepted(hist, a);

  ASSERT_TRUE(arm_from_applied(b, sid(), 1, 100, 5));
  EXPECT_TRUE(try_accept_from_history(b, hist));
  EXPECT_EQ(b.feedback_mode, "odometry");
  EXPECT_EQ(b.accepted_snapshot_seq, 5u);
}

TEST(G6FeedbackBarrier, EventBeforeOdomThenAccept)
{
  PendingBarrier b;
  ASSERT_TRUE(arm_from_applied(b, sid(), 1, 55, 8));
  EXPECT_FALSE(b.accepted);
  EXPECT_TRUE(accept_odometry(b, 8));
  EXPECT_EQ(b.accepted_snapshot_seq, 8u);
}

TEST(G6FeedbackBarrier, DelayedOldOdomRejected)
{
  PendingBarrier b;
  ASSERT_TRUE(arm_from_applied(b, sid(), 1, 3, 20));
  // Old meta with wrong / older correction association.
  EXPECT_FALSE(meta_satisfies(b, meta(15, 2)));
  EXPECT_FALSE(meta_satisfies(b, meta(15, 3)));  // seq too old
}

TEST(G6FeedbackBarrier, DuplicateEventsIdempotentArm)
{
  PendingBarrier b;
  ASSERT_TRUE(arm_from_applied(b, sid(), 1, 9, 4));
  ASSERT_TRUE(accept_odometry(b, 4));
  // Re-arm same id (duplicate CorrectionApplied).
  ASSERT_TRUE(arm_from_applied(b, sid(), 1, 9, 4));
  EXPECT_FALSE(b.accepted);
  EXPECT_TRUE(try_accept_from_history(b, {AcceptedSample{sid(), 1, 4, 9}}));
}

TEST(G6FeedbackBarrier, ResetClearsBarrierAndBuffers)
{
  PendingBarrier b;
  std::deque<MetaSample> metas;
  std::deque<std::pair<int32_t, uint32_t>> stamps;
  std::deque<AcceptedSample> hist;
  ASSERT_TRUE(arm_from_applied(b, sid(), 1, 1, 1));
  push_bounded_meta(metas, meta(1, 1));
  push_accepted(hist, AcceptedSample{sid(), 1, 1, 1});
  on_estimator_reset(b, metas, stamps, hist);
  EXPECT_FALSE(b.armed);
  EXPECT_TRUE(metas.empty());
  EXPECT_TRUE(hist.empty());
}

TEST(G6FeedbackBarrier, SharedStampPrefersHighestSnapshotSeq)
{
  std::deque<MetaSample> buf;
  push_bounded_meta(buf, meta(1, 0, 5, 100));
  push_bounded_meta(buf, meta(2, 0, 5, 100));  // same stamp, higher seq
  auto found = find_meta_for_stamp(buf, 5, 100);
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(found->snapshot_seq, 2u);
}

TEST(G6FeedbackBarrier, RejectNilArm)
{
  PendingBarrier b;
  EXPECT_FALSE(arm_from_applied(b, slam_identity::nil_uuid(), 1, 1, 1));
  EXPECT_FALSE(arm_from_applied(b, sid(), 0, 1, 1));
  EXPECT_FALSE(arm_from_applied(b, sid(), 1, 0, 1));
  EXPECT_FALSE(arm_from_applied(b, sid(), 1, 1, 0));
}
