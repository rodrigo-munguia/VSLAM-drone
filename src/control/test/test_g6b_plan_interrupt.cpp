#include <gtest/gtest.h>

#include "g6b_plan_interrupt.hpp"

using control::g6b_plan::InterruptKind;
using control::g6b_plan::classify_run_clear;
using control::g6b_plan::should_resume_same_target;

TEST(G6bPlanInterrupt, CoordPauseVsOperatorStop)
{
  EXPECT_EQ(classify_run_clear(true, false), InterruptKind::None);
  EXPECT_EQ(classify_run_clear(false, true), InterruptKind::CoordPause);
  EXPECT_EQ(classify_run_clear(false, false), InterruptKind::OperatorStop);
  // Suppress while plan still marked active: treat as none for classify_run_clear
  // (run still true) — park path only engages when run is cleared.
  EXPECT_EQ(classify_run_clear(true, true), InterruptKind::None);
}

TEST(G6bPlanInterrupt, ResumeSameTargetOnlyWhenPlanLiveAndSuppressClear)
{
  EXPECT_TRUE(should_resume_same_target(true, false));
  EXPECT_FALSE(should_resume_same_target(true, true));
  EXPECT_FALSE(should_resume_same_target(false, false));
  EXPECT_FALSE(should_resume_same_target(false, true));
}

TEST(G6bPlanInterrupt, UnresolvedHoldKeepsParkUntilOperatorClearsSuppress)
{
  // Abort-without-resume leaves suppress armed + run false → still CoordPause.
  EXPECT_EQ(classify_run_clear(false, true), InterruptKind::CoordPause);
  EXPECT_FALSE(should_resume_same_target(false, true));
  // Operator 'r': run true, suppress cleared → same-target resume.
  EXPECT_TRUE(should_resume_same_target(true, false));
  // Clearing run without suppress (true operator stop) exits the command.
  EXPECT_EQ(classify_run_clear(false, false), InterruptKind::OperatorStop);
}
