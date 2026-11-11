#include <gtest/gtest.h>

#include "g6_coord_dispatch.hpp"

using globalslam::g6_coord::CoordMode;
using globalslam::g6_coord::decide_dispatch;
using globalslam::g6_coord::parse_coord_mode;
using globalslam::g6_coord::use_legacy_control_c;

TEST(G6CoordDispatch, RequiredNeedsPause)
{
  auto d = decide_dispatch(true, CoordMode::Required, true, false);
  EXPECT_FALSE(d.may_dispatch);
  EXPECT_TRUE(d.need_pause);
  d = decide_dispatch(true, CoordMode::Required, true, true);
  EXPECT_TRUE(d.may_dispatch);
}

TEST(G6CoordDispatch, RequiredMissingControlSkips)
{
  auto d = decide_dispatch(true, CoordMode::Required, false, false);
  EXPECT_FALSE(d.may_dispatch);
  EXPECT_TRUE(d.skip_because_no_control);
}

TEST(G6CoordDispatch, AbsentOkWithoutControl)
{
  auto d = decide_dispatch(true, CoordMode::AbsentOk, false, false);
  EXPECT_TRUE(d.may_dispatch);
  EXPECT_FALSE(d.need_pause);
}

TEST(G6CoordDispatch, OfflineBypass)
{
  auto d = decide_dispatch(true, CoordMode::OfflineBypass, false, false);
  EXPECT_TRUE(d.may_dispatch);
  EXPECT_FALSE(d.need_pause);
}

TEST(G6CoordDispatch, CorrectionsDisabled)
{
  auto d = decide_dispatch(false, CoordMode::Required, true, true);
  EXPECT_FALSE(d.may_dispatch);
}

TEST(G6CoordDispatch, LegacyCOnlyWhenNotCoordinated)
{
  EXPECT_TRUE(use_legacy_control_c(false, true, true));
  EXPECT_FALSE(use_legacy_control_c(true, true, true));
  EXPECT_FALSE(use_legacy_control_c(false, true, false));
  EXPECT_FALSE(use_legacy_control_c(false, false, true));
}

TEST(G6CoordDispatch, ParseMode)
{
  EXPECT_EQ(parse_coord_mode("absent_ok"), CoordMode::AbsentOk);
  EXPECT_EQ(parse_coord_mode("offline_bypass"), CoordMode::OfflineBypass);
  EXPECT_EQ(parse_coord_mode("required"), CoordMode::Required);
  EXPECT_EQ(parse_coord_mode("nope"), CoordMode::Required);
}
