#include <gtest/gtest.h>

#include "g6_coord_policy.hpp"

using control::g6_coord::CoordSnapshot;
using control::g6_coord::State;
using control::g6_coord::TxnIdentity;
using control::g6_coord::abort_txn;
using control::g6_coord::mark_operator_superseded;
using control::g6_coord::prepare_pause;
using control::g6_coord::query;
using control::g6_coord::release;

static TxnIdentity make_txn(uint64_t tid, uint64_t owner, uint64_t epoch = 1)
{
  TxnIdentity t;
  t.session_id.fill(7);
  t.session_id[6] = 0x40;
  t.session_id[8] = 0x80;
  t.epoch_id = epoch;
  t.transaction_id = tid;
  t.owner_generation = owner;
  return t;
}

TEST(G6CoordPolicy, PreparePauseIdempotentAndConflict)
{
  CoordSnapshot snap;
  auto a = make_txn(10, 100);
  auto r = prepare_pause(snap, a, true, true);
  EXPECT_TRUE(r.success);
  EXPECT_EQ(snap.state, State::Paused);
  EXPECT_TRUE(r.plan_was_active);

  // Idempotent
  r = prepare_pause(snap, a, true, true);
  EXPECT_TRUE(r.success);

  // Conflict
  auto b = make_txn(11, 100);
  r = prepare_pause(snap, b, true, true);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.reason, "conflicting_transaction");
}

TEST(G6CoordPolicy, ReleaseResumesOnlyWhenOwnedAndNotSuperseded)
{
  CoordSnapshot snap;
  auto a = make_txn(10, 100);
  ASSERT_TRUE(prepare_pause(snap, a, true, true).success);

  auto r = release(snap, a, true, true);
  EXPECT_TRUE(r.success);
  EXPECT_TRUE(r.plan_resumed);
  EXPECT_EQ(snap.state, State::Idle);

  // Second release: idempotent already_idle for same txn
  r = release(snap, a, true, true);
  EXPECT_TRUE(r.success);
  EXPECT_EQ(r.reason, "already_idle");
  // Different txn while idle → no_active_pause
  r = release(snap, make_txn(99, 100), true, true);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.reason, "no_active_pause");
}

TEST(G6CoordPolicy, OperatorSupersedeBlocksResume)
{
  CoordSnapshot snap;
  auto a = make_txn(10, 100);
  ASSERT_TRUE(prepare_pause(snap, a, true, true).success);
  mark_operator_superseded(snap);
  auto r = release(snap, a, true, true);
  EXPECT_TRUE(r.success);
  EXPECT_FALSE(r.plan_resumed);
}

TEST(G6CoordPolicy, AbortRestoresActivePlanWhenRequested)
{
  // Definite non-application: GS passes resume_requested=true.
  CoordSnapshot snap;
  auto a = make_txn(10, 100);
  ASSERT_TRUE(prepare_pause(snap, a, true, true).success);
  auto r = abort_txn(snap, a, /*resume_requested=*/true);
  EXPECT_TRUE(r.success);
  EXPECT_TRUE(r.plan_was_active);
  EXPECT_TRUE(r.plan_resumed);
  EXPECT_EQ(snap.state, State::Idle);
}

TEST(G6CoordPolicy, AbortDoesNotResumeWhenPlanWasIdle)
{
  CoordSnapshot snap;
  auto a = make_txn(10, 100);
  ASSERT_TRUE(prepare_pause(snap, a, false, true).success);
  auto r = abort_txn(snap, a, /*resume_requested=*/true);
  EXPECT_TRUE(r.success);
  EXPECT_FALSE(r.plan_was_active);
  EXPECT_FALSE(r.plan_resumed);
}

TEST(G6CoordPolicy, AbortAppliedUnresolvedDoesNotResume)
{
  // G6b.1: applied but feedback missing — resume_requested=false.
  CoordSnapshot snap;
  auto a = make_txn(10, 100);
  ASSERT_TRUE(prepare_pause(snap, a, true, true).success);
  auto r = abort_txn(snap, a, /*resume_requested=*/false);
  EXPECT_TRUE(r.success);
  EXPECT_TRUE(r.plan_was_active);
  EXPECT_FALSE(r.plan_resumed);
  EXPECT_EQ(snap.state, State::Idle);
}

TEST(G6CoordPolicy, AbortRespectsOperatorSupersede)
{
  CoordSnapshot snap;
  auto a = make_txn(10, 100);
  ASSERT_TRUE(prepare_pause(snap, a, true, true).success);
  mark_operator_superseded(snap);
  auto r = abort_txn(snap, a, /*resume_requested=*/true);
  EXPECT_TRUE(r.success);
  EXPECT_FALSE(r.plan_resumed);
}

TEST(G6CoordPolicy, AwaitingQuiescence)
{
  CoordSnapshot snap;
  auto a = make_txn(10, 100);
  auto r = prepare_pause(snap, a, false, false);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.reason, "awaiting_cmd_quiescence");
  EXPECT_EQ(snap.state, State::Paused);
}
