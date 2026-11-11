#include <gtest/gtest.h>

#include "g6b_feedback_completion.hpp"

using namespace globalslam::g6b_fb;

TEST(G6bFeedbackCompletion, EarlyFeedbackBeforeLsResponse)
{
  PendingTxn t;
  std::deque<uint64_t> early;
  // Feedback arrives before register (and before LS response).
  EXPECT_FALSE(on_feedback_accepted(t, early, 42, 7, "gt"));
  EXPECT_EQ(early.size(), 1u);

  register_dispatch(t, early, 42, 1, 9, "bundle_adjustment", true, false);
  EXPECT_EQ(t.phase, TxnPhase::Dispatched);
  EXPECT_TRUE(t.early_feedback_latched);
  EXPECT_TRUE(early.empty());

  EXPECT_TRUE(on_ls_outcome(t, /*applied=*/true, /*unresolved=*/false));
  EXPECT_EQ(t.phase, TxnPhase::ReadyToRelease);
  EXPECT_TRUE(may_auto_resume_plan(t.phase));
}

TEST(G6bFeedbackCompletion, FeedbackAfterAwaiting)
{
  PendingTxn t;
  std::deque<uint64_t> early;
  register_dispatch(t, early, 99, 2, 1, "bundle_adjustment", true, false);
  EXPECT_FALSE(on_ls_outcome(t, true, false));
  EXPECT_EQ(t.phase, TxnPhase::AwaitingFeedback);

  EXPECT_TRUE(on_feedback_accepted(t, early, 99, 11, "gt"));
  EXPECT_EQ(t.phase, TxnPhase::ReadyToRelease);
}

TEST(G6bFeedbackCompletion, DuplicateFeedbackIdempotent)
{
  PendingTxn t;
  std::deque<uint64_t> early;
  register_dispatch(t, early, 5, 1, 1, "bundle_adjustment", true, false);
  ASSERT_FALSE(on_ls_outcome(t, true, false));
  ASSERT_TRUE(on_feedback_accepted(t, early, 5, 1, "gt"));
  EXPECT_FALSE(on_feedback_accepted(t, early, 5, 1, "gt"));
  EXPECT_EQ(t.phase, TxnPhase::ReadyToRelease);
}

TEST(G6bFeedbackCompletion, ForeignFeedbackDoesNotComplete)
{
  PendingTxn t;
  std::deque<uint64_t> early;
  register_dispatch(t, early, 10, 1, 1, "bundle_adjustment", true, false);
  ASSERT_FALSE(on_ls_outcome(t, true, false));
  EXPECT_FALSE(on_feedback_accepted(t, early, 11, 1, "gt"));
  EXPECT_EQ(t.phase, TxnPhase::AwaitingFeedback);
  EXPECT_EQ(early.back(), 11u);
}

TEST(G6bFeedbackCompletion, NotAppliedMayResume)
{
  PendingTxn t;
  std::deque<uint64_t> early;
  register_dispatch(t, early, 3, 1, 1, "bundle_adjustment", true, false);
  EXPECT_FALSE(on_ls_outcome(t, false, false));
  EXPECT_EQ(t.phase, TxnPhase::NotApplied);
  EXPECT_TRUE(may_auto_resume_plan(t.phase));
}

TEST(G6bFeedbackCompletion, TimeoutHoldDoesNotResume)
{
  PendingTxn t;
  std::deque<uint64_t> early;
  register_dispatch(t, early, 3, 1, 1, "bundle_adjustment", true, false);
  ASSERT_FALSE(on_ls_outcome(t, true, false));
  on_feedback_timeout(t);
  EXPECT_EQ(t.phase, TxnPhase::UnresolvedHold);
  EXPECT_FALSE(may_auto_resume_plan(t.phase));
}

TEST(G6bFeedbackCompletion, UnresolvedOutcomeHolds)
{
  PendingTxn t;
  std::deque<uint64_t> early;
  register_dispatch(t, early, 3, 1, 1, "bundle_adjustment", true, false);
  EXPECT_FALSE(on_ls_outcome(t, false, true));
  EXPECT_EQ(t.phase, TxnPhase::UnresolvedHold);
  EXPECT_FALSE(may_auto_resume_plan(t.phase));
}

TEST(G6bFeedbackCompletion, GenerationInvalidate)
{
  PendingTxn t;
  std::deque<uint64_t> early;
  register_dispatch(t, early, 1, 5, 1, "bundle_adjustment", true, false);
  invalidate_generation(t, 6);
  EXPECT_EQ(t.phase, TxnPhase::Idle);
}
