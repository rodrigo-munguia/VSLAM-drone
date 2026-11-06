#include <cmath>
#include <gtest/gtest.h>

#include "plot_frame_conversions.hpp"

using plot2::frames::expected_public_from_webots_z_up_via_legacy_gt;
using plot2::frames::legacy_gt_position_to_public;
using plot2::frames::public_xy_to_screen;
using plot2::frames::public_xz_to_screen;
using plot2::frames::public_zy_to_screen;
using plot2::frames::screen_to_public_xy;
using plot2::frames::screen_to_public_xz;
using plot2::frames::screen_to_public_zy;
using plot2::frames::top_view_screen_basis_public_xy;

TEST(Plot2Frames, LegacyGtMapsByDOnEachAxis)
{
  double xp = 0, yp = 0, zp = 0;
  legacy_gt_position_to_public(1.25, -7.44, 0.04, xp, yp, zp);
  EXPECT_DOUBLE_EQ(xp, 1.25);
  EXPECT_DOUBLE_EQ(yp, 7.44);
  EXPECT_DOUBLE_EQ(zp, -0.04);
}

TEST(Plot2Frames, Screenshot1SignsAgreeAfterConversion)
{
  const double est_y = -7.44;
  const double est_z = 0.04;

  double gx = 0, gy = 0, gz = 0;
  legacy_gt_position_to_public(-0.03, 9.62, -0.05, gx, gy, gz);

  EXPECT_GT(est_y * gy, 0.0);
  EXPECT_GT(est_z * gz, 0.0);
  EXPECT_DOUBLE_EQ(gy, -9.62);
  EXPECT_DOUBLE_EQ(gz, 0.05);
}

TEST(Plot2Frames, Screenshot2HorizontalAndNearZeroZ)
{
  const double est_x = 2.83;
  const double est_y = 10.50;
  const double est_z = -0.05;

  double gx = 0, gy = 0, gz = 0;
  legacy_gt_position_to_public(3.69, -10.26, 0.04, gx, gy, gz);

  EXPECT_GT(est_x * gx, 0.0);
  EXPECT_GT(est_y * gy, 0.0);
  EXPECT_GT(est_z * gz, 0.0);
  EXPECT_DOUBLE_EQ(gy, 10.26);
  EXPECT_DOUBLE_EQ(gz, -0.04);
}

TEST(Plot2Frames, ClimbIncreasesPublicZFromLegacyGt)
{
  double home_x = 0, home_y = 0, home_z = 0;
  legacy_gt_position_to_public(0.0, 0.0, -0.1, home_x, home_y, home_z);

  double climb_x = 0, climb_y = 0, climb_z = 0;
  legacy_gt_position_to_public(0.0, 0.0, -2.1, climb_x, climb_y, climb_z);

  const double dz_pub = climb_z - home_z;
  EXPECT_DOUBLE_EQ(dz_pub, 2.0);
  EXPECT_GT(dz_pub, 0.0);
}

TEST(Plot2Frames, DescentDecreasesPublicZFromLegacyGt)
{
  double high_x = 0, high_y = 0, high_z = 0;
  legacy_gt_position_to_public(1.0, -4.0, -3.0, high_x, high_y, high_z);
  double low_x = 0, low_y = 0, low_z = 0;
  legacy_gt_position_to_public(1.0, -4.0, -1.0, low_x, low_y, low_z);
  EXPECT_LT(low_z - high_z, 0.0);
  EXPECT_DOUBLE_EQ(low_z - high_z, -2.0);
}

TEST(Plot2Frames, NonzeroAllAxesIndependentOfOdometryHelpers)
{
  double xp = 0, yp = 0, zp = 0;
  expected_public_from_webots_z_up_via_legacy_gt(3.0, 10.0, 2.0, xp, yp, zp);
  EXPECT_DOUBLE_EQ(xp, 3.0);
  EXPECT_DOUBLE_EQ(yp, 10.0);
  EXPECT_DOUBLE_EQ(zp, 2.0);
}

TEST(Plot2Frames, MatchesPublicEstimateWhenInternalEqualsLegacyGt)
{
  const double p_int[3] = {2.5, -8.0, -1.5};
  const double p_pub_est[3] = {
    1.0 * p_int[0], -1.0 * p_int[1], -1.0 * p_int[2]};

  double gt_pub[3] = {0, 0, 0};
  legacy_gt_position_to_public(
    p_int[0], p_int[1], p_int[2], gt_pub[0], gt_pub[1], gt_pub[2]);

  EXPECT_DOUBLE_EQ(gt_pub[0], p_pub_est[0]);
  EXPECT_DOUBLE_EQ(gt_pub[1], p_pub_est[1]);
  EXPECT_DOUBLE_EQ(gt_pub[2], p_pub_est[2]);
}

TEST(Plot2Frames, CombinedDisplacementOriginSubtractIsLinear)
{
  double o0x = 0, o0y = 0, o0z = 0;
  legacy_gt_position_to_public(1.0, 2.0, 3.0, o0x, o0y, o0z);
  double px = 0, py = 0, pz = 0;
  legacy_gt_position_to_public(4.0, -5.0, 6.0, px, py, pz);
  EXPECT_DOUBLE_EQ(px - o0x, 3.0);
  EXPECT_DOUBLE_EQ(py - o0y, 7.0);
  EXPECT_DOUBLE_EQ(pz - o0z, -3.0);
}

TEST(Plot2Projection, GroundLandmarkBelowRobotInBothVerticalViews)
{
  const double robot[3] = {1.0, -2.0, 2.0};
  const double landmark[3] = {1.2, -1.8, 0.5};
  const double delta_z = landmark[2] - robot[2];
  ASSERT_LT(delta_z, 0.0);

  const double cx = 200.0, cy = 100.0, scale = 10.0;
  double rsx = 0, rsy = 0, lsx = 0, lsy = 0;

  public_xz_to_screen(robot[0], robot[2], cx, cy, scale, rsx, rsy);
  public_xz_to_screen(landmark[0], landmark[2], cx, cy, scale, lsx, lsy);
  EXPECT_GT(lsy, rsy);

  public_zy_to_screen(robot[1], robot[2], cx, cy, scale, rsx, rsy);
  public_zy_to_screen(landmark[1], landmark[2], cx, cy, scale, lsx, lsy);
  EXPECT_GT(lsy, rsy);
}

TEST(Plot2Projection, AboveLandmarkAboveRobotInBothVerticalViews)
{
  const double robot[3] = {0.5, 1.0, 1.0};
  const double above[3] = {0.5, 1.0, 3.0};
  ASSERT_GT(above[2] - robot[2], 0.0);

  const double cx = 50.0, cy = 80.0, scale = 5.0;
  double rsx = 0, rsy = 0, asx = 0, asy = 0;

  public_xz_to_screen(robot[0], robot[2], cx, cy, scale, rsx, rsy);
  public_xz_to_screen(above[0], above[2], cx, cy, scale, asx, asy);
  EXPECT_LT(asy, rsy);

  public_zy_to_screen(robot[1], robot[2], cx, cy, scale, rsx, rsy);
  public_zy_to_screen(above[1], above[2], cx, cy, scale, asx, asy);
  EXPECT_LT(asy, rsy);
}

TEST(Plot2Projection, EqualHeightSameScreenY)
{
  const double cx = 10.0, cy = 20.0, scale = 8.0;
  double s1x = 0, s1y = 0, s2x = 0, s2y = 0;
  public_xz_to_screen(1.0, 2.5, cx, cy, scale, s1x, s1y);
  public_xz_to_screen(-3.0, 2.5, cx, cy, scale, s2x, s2y);
  EXPECT_DOUBLE_EQ(s1y, s2y);
  public_zy_to_screen(4.0, 2.5, cx, cy, scale, s1x, s1y);
  public_zy_to_screen(-1.0, 2.5, cx, cy, scale, s2x, s2y);
  EXPECT_DOUBLE_EQ(s1y, s2y);
}

TEST(Plot2Projection, PositiveZArrowPointsUpOnScreen)
{
  const double cx = 0.0, cy = 0.0, scale = 1.0;
  double ox = 0, oy = 0, zx = 0, zy = 0;
  public_xz_to_screen(0.0, 0.0, cx, cy, scale, ox, oy);
  public_xz_to_screen(0.0, 2.0, cx, cy, scale, zx, zy);
  EXPECT_LT(zy, oy);

  public_zy_to_screen(0.0, 0.0, cx, cy, scale, ox, oy);
  public_zy_to_screen(0.0, 2.0, cx, cy, scale, zx, zy);
  EXPECT_LT(zy, oy);
}

TEST(Plot2Projection, PositiveAxisEndpointsIndependent)
{
  const double cx = 100.0, cy = 100.0, scale = 10.0;
  double sx = 0, sy = 0;

  public_xz_to_screen(2.0, 0.0, cx, cy, scale, sx, sy);
  EXPECT_DOUBLE_EQ(sx, 120.0);
  EXPECT_DOUBLE_EQ(sy, 100.0);

  public_xz_to_screen(0.0, 2.0, cx, cy, scale, sx, sy);
  EXPECT_DOUBLE_EQ(sx, 100.0);
  EXPECT_DOUBLE_EQ(sy, 80.0);

  public_zy_to_screen(2.0, 0.0, cx, cy, scale, sx, sy);
  EXPECT_DOUBLE_EQ(sx, 120.0);
  EXPECT_DOUBLE_EQ(sy, 100.0);

  public_zy_to_screen(0.0, 2.0, cx, cy, scale, sx, sy);
  EXPECT_DOUBLE_EQ(sx, 100.0);
  EXPECT_DOUBLE_EQ(sy, 80.0);
}

TEST(Plot2Projection, SharedProjectionRobotGtPathLandmark)
{
  const double robot[3] = {0.0, 0.0, 2.0};
  const double gt[3] = {0.1, -0.1, 1.9};
  const double path[3] = {-0.2, 0.2, 2.1};
  const double lm[3] = {0.3, 0.0, 0.2};
  const double cx = 0.0, cy = 0.0, scale = 1.0;

  double rsy, gsy, psy, lsy, ignore;
  public_xz_to_screen(robot[0], robot[2], cx, cy, scale, ignore, rsy);
  public_xz_to_screen(gt[0], gt[2], cx, cy, scale, ignore, gsy);
  public_xz_to_screen(path[0], path[2], cx, cy, scale, ignore, psy);
  public_xz_to_screen(lm[0], lm[2], cx, cy, scale, ignore, lsy);

  EXPECT_GT(gsy, rsy);
  EXPECT_LT(psy, rsy);
  EXPECT_GT(lsy, rsy);
}

TEST(Plot2Projection, PanOffsetDoesNotReverseVerticalOrder)
{
  const double scale = 10.0;
  const double c0x = 200.0, c0y = 100.0;
  const double c1x = 250.0, c1y = 40.0;

  double r0 = 0, l0 = 0, r1 = 0, l1 = 0, ign = 0;
  public_xz_to_screen(1.0, 2.0, c0x, c0y, scale, ign, r0);
  public_xz_to_screen(1.0, 0.5, c0x, c0y, scale, ign, l0);
  public_xz_to_screen(1.0, 2.0, c1x, c1y, scale, ign, r1);
  public_xz_to_screen(1.0, 0.5, c1x, c1y, scale, ign, l1);

  EXPECT_GT(l0, r0);
  EXPECT_GT(l1, r1);
  EXPECT_DOUBLE_EQ(l0 - r0, l1 - r1);
}

TEST(Plot2Projection, ScreenInverseRoundTripNotUsedAsSoleEvidence)
{
  const double cx = 11.0, cy = 22.0, scale = 4.0;
  double sx = 0, sy = 0, xr = 0, zr = 0;
  public_xz_to_screen(1.5, -2.5, cx, cy, scale, sx, sy);
  EXPECT_DOUBLE_EQ(sx, 11.0 + 1.5 * 4.0);
  EXPECT_DOUBLE_EQ(sy, 22.0 - (-2.5) * 4.0);
  screen_to_public_xz(sx, sy, cx, cy, scale, xr, zr);
  EXPECT_DOUBLE_EQ(xr, 1.5);
  EXPECT_DOUBLE_EQ(zr, -2.5);

  double yr = 0;
  public_zy_to_screen(-3.0, 4.0, cx, cy, scale, sx, sy);
  EXPECT_DOUBLE_EQ(sx, 11.0 + (-3.0) * 4.0);
  EXPECT_DOUBLE_EQ(sy, 22.0 - 4.0 * 4.0);
  screen_to_public_zy(sx, sy, cx, cy, scale, yr, zr);
  EXPECT_DOUBLE_EQ(yr, -3.0);
  EXPECT_DOUBLE_EQ(zr, 4.0);
}

TEST(Plot2Projection, LegacyBrokenProjectionWouldPutGroundAbove)
{
  const double robot_z = 2.0, ground_z = 0.5;
  const double cy = 100.0, scale = 10.0;
  const double legacy_robot_sy = cy + robot_z * scale;
  const double legacy_ground_sy = cy + ground_z * scale;
  EXPECT_LT(legacy_ground_sy, legacy_robot_sy);

  double rsx = 0, rsy = 0, gsx = 0, gsy = 0;
  public_xz_to_screen(0.0, robot_z, 0.0, cy, scale, rsx, rsy);
  public_xz_to_screen(0.0, ground_z, 0.0, cy, scale, gsx, gsy);
  EXPECT_GT(gsy, rsy);
}

TEST(Plot2Projection, VerticalGroundBelowRemainsValidWithTopViewFix)
{
  double rsx = 0, rsy = 0, lsx = 0, lsy = 0;
  public_xz_to_screen(0.0, 2.0, 0.0, 100.0, 10.0, rsx, rsy);
  public_xz_to_screen(0.0, 0.5, 0.0, 100.0, 10.0, lsx, lsy);
  EXPECT_GT(lsy, rsy);
}

// ---------------------------------------------------------------------------
// Top XY from above: +X right, +Y up, look toward -Z.
// ---------------------------------------------------------------------------

TEST(Plot2TopView, PositiveXGoesScreenRightPositiveYGoesScreenUp)
{
  const double cx = 100.0, cy = 100.0, scale = 10.0;
  double ox = 0, oy = 0, sx = 0, sy = 0;
  public_xy_to_screen(0.0, 0.0, cx, cy, scale, ox, oy);

  public_xy_to_screen(2.0, 0.0, cx, cy, scale, sx, sy);
  EXPECT_GT(sx, ox);  // +X → right
  EXPECT_DOUBLE_EQ(sy, oy);

  public_xy_to_screen(0.0, 2.0, cx, cy, scale, sx, sy);
  EXPECT_LT(sy, oy);  // +Y → up
  EXPECT_DOUBLE_EQ(sx, ox);
}

TEST(Plot2TopView, AsymmetricLabelledReferenceProjectsFromAbove)
{
  // Independently specified points (not inverse of the helper alone).
  const double cx = 0.0, cy = 0.0, scale = 1.0;
  double sx = 0, sy = 0;

  public_xy_to_screen(0.0, 0.0, cx, cy, scale, sx, sy);
  EXPECT_DOUBLE_EQ(sx, 0.0);
  EXPECT_DOUBLE_EQ(sy, 0.0);

  public_xy_to_screen(1.0, 0.0, cx, cy, scale, sx, sy);
  EXPECT_DOUBLE_EQ(sx, 1.0);
  EXPECT_DOUBLE_EQ(sy, 0.0);

  public_xy_to_screen(0.0, 1.0, cx, cy, scale, sx, sy);
  EXPECT_DOUBLE_EQ(sx, 0.0);
  EXPECT_DOUBLE_EQ(sy, -1.0);

  // +Z is unused in XY; off-axis (1,2) → (1, -2)
  public_xy_to_screen(1.0, 2.0, cx, cy, scale, sx, sy);
  EXPECT_DOUBLE_EQ(sx, 1.0);
  EXPECT_DOUBLE_EQ(sy, -2.0);
}

TEST(Plot2TopView, ScreenBasisLooksTowardMinusZ)
{
  double right[2] = {0, 0}, up[2] = {0, 0};
  top_view_screen_basis_public_xy(right, up);
  EXPECT_DOUBLE_EQ(right[0], 1.0);
  EXPECT_DOUBLE_EQ(right[1], 0.0);
  EXPECT_DOUBLE_EQ(up[0], 0.0);
  EXPECT_DOUBLE_EQ(up[1], 1.0);

  // look = -(right × up) with z=0 components → -Z
  // right×up = (0,0, right_x*up_y - right_y*up_x) = (0,0,1)
  const double cross_z = right[0] * up[1] - right[1] * up[0];
  EXPECT_DOUBLE_EQ(cross_z, 1.0);
  const double look_z = -cross_z;
  EXPECT_DOUBLE_EQ(look_z, -1.0);

  // Map (x,y)->(sr,su)=(x,y) has det +1 (proper).
  const double det = right[0] * up[1] - right[1] * up[0];
  EXPECT_DOUBLE_EQ(det, 1.0);
}

TEST(Plot2TopView, DefectiveOldMappingLookedFromBelow)
{
  // Old: sx = cx + y*s, sy = cy - x*s → right=+Y, up=+X → look=+Z.
  const double old_right[2] = {0.0, 1.0};
  const double old_up[2] = {1.0, 0.0};
  const double old_cross_z = old_right[0] * old_up[1] - old_right[1] * old_up[0];
  EXPECT_DOUBLE_EQ(old_cross_z, -1.0);
  EXPECT_DOUBLE_EQ(-old_cross_z, 1.0);  // looked from below
}

TEST(Plot2TopView, PoseGtPathLandmarkShareSameProjection)
{
  const double cx = 50.0, cy = 60.0, scale = 5.0;
  const double x = 1.0;
  double r_sx, r_sy, g_sx, g_sy, p_sx, p_sy, l_sx, l_sy;
  public_xy_to_screen(x, 3.0, cx, cy, scale, r_sx, r_sy);
  public_xy_to_screen(x, 3.1, cx, cy, scale, g_sx, g_sy);
  public_xy_to_screen(x, 2.9, cx, cy, scale, p_sx, p_sy);
  public_xy_to_screen(x, 3.0, cx, cy, scale, l_sx, l_sy);
  // Same x → same screen_x (arrows / pose / path / GT / landmarks).
  EXPECT_DOUBLE_EQ(r_sx, g_sx);
  EXPECT_DOUBLE_EQ(r_sx, p_sx);
  EXPECT_DOUBLE_EQ(r_sx, l_sx);
  EXPECT_DOUBLE_EQ(r_sx, cx + x * scale);
}

TEST(Plot2TopView, PanAndPositiveZoomPreserveDirections)
{
  double ox = 0, oy = 0, sx = 0, sy = 0;
  public_xy_to_screen(0.0, 0.0, 10.0, 20.0, 5.0, ox, oy);
  public_xy_to_screen(1.0, 0.0, 10.0, 20.0, 5.0, sx, sy);
  EXPECT_GT(sx - ox, 0.0);
  public_xy_to_screen(0.0, 1.0, 10.0, 20.0, 5.0, sx, sy);
  EXPECT_LT(sy - oy, 0.0);

  // Pan changes centre equally; direction of +X remains right.
  public_xy_to_screen(0.0, 0.0, 40.0, -5.0, 5.0, ox, oy);
  public_xy_to_screen(1.0, 0.0, 40.0, -5.0, 5.0, sx, sy);
  EXPECT_DOUBLE_EQ(sx - ox, 5.0);

  // Larger positive zoom increases displacement magnitude, same sign.
  public_xy_to_screen(0.0, 0.0, 0.0, 0.0, 2.0, ox, oy);
  public_xy_to_screen(1.0, 0.0, 0.0, 0.0, 2.0, sx, sy);
  EXPECT_DOUBLE_EQ(sx - ox, 2.0);
  public_xy_to_screen(1.0, 0.0, 0.0, 0.0, 8.0, sx, sy);
  EXPECT_DOUBLE_EQ(sx - ox, 8.0);
}

TEST(Plot2TopView, ScreenInverseMatchesDocumentedConvention)
{
  const double cx = 11.0, cy = 22.0, scale = 4.0;
  double sx = 0, sy = 0, xr = 0, yr = 0;
  public_xy_to_screen(1.5, -2.5, cx, cy, scale, sx, sy);
  EXPECT_DOUBLE_EQ(sx, 11.0 + 1.5 * 4.0);
  EXPECT_DOUBLE_EQ(sy, 22.0 - (-2.5) * 4.0);
  screen_to_public_xy(sx, sy, cx, cy, scale, xr, yr);
  EXPECT_DOUBLE_EQ(xr, 1.5);
  EXPECT_DOUBLE_EQ(yr, -2.5);
}

TEST(Plot2TopView, PublicMatchesWebotsGpsAfterLegacyGtChain)
{
  double xp = 0, yp = 0, zp = 0;
  expected_public_from_webots_z_up_via_legacy_gt(3.0, -4.0, 1.5, xp, yp, zp);
  EXPECT_DOUBLE_EQ(xp, 3.0);
  EXPECT_DOUBLE_EQ(yp, -4.0);
  EXPECT_DOUBLE_EQ(zp, 1.5);
}

TEST(Plot2TopView, VerticalUpBehaviorPreservedAlongsideTopView)
{
  double rsx = 0, rsy = 0, lsx = 0, lsy = 0;
  public_xz_to_screen(0.0, 2.0, 0.0, 100.0, 10.0, rsx, rsy);
  public_xz_to_screen(0.0, 0.5, 0.0, 100.0, 10.0, lsx, lsy);
  EXPECT_GT(lsy, rsy);
  double ox = 0, oy = 0, zx = 0, zy = 0;
  public_xz_to_screen(0.0, 0.0, 0.0, 0.0, 1.0, ox, oy);
  public_xz_to_screen(0.0, 1.0, 0.0, 0.0, 1.0, zx, zy);
  EXPECT_LT(zy, oy);
}
