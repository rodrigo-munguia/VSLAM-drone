#include <gtest/gtest.h>
#include "mission_enu.hpp"

TEST(MissionEnu, IdentityAtOrigin)
{
  double gx, gy, gz;
  mission_args_to_gps_enu(10.0, 20.0, 15.0, 0.0, 0.0, 0.0, gx, gy, gz);
  EXPECT_DOUBLE_EQ(gx, 10.0);
  EXPECT_DOUBLE_EQ(gy, 20.0);
  EXPECT_DOUBLE_EQ(gz, 15.0);
}

TEST(MissionEnu, EastPositiveNoYFlipOnX)
{
  double gx, gy, gz;
  mission_args_to_gps_enu(0.0, 0.0, 10.0, 5.0, 0.0, 5.0, gx, gy, gz);
  EXPECT_DOUBLE_EQ(gx, 5.0);
  EXPECT_DOUBLE_EQ(gy, 0.0);
  EXPECT_DOUBLE_EQ(gz, 15.0);
}

TEST(MissionEnu, LegacyYIsNegatedNorth)
{
  // mission_y = +3 means GPS Y must decrease by 3 (undo legacy negation).
  double gx, gy, gz;
  mission_args_to_gps_enu(100.0, 200.0, 15.0, 0.0, 3.0, 0.0, gx, gy, gz);
  EXPECT_DOUBLE_EQ(gx, 100.0);
  EXPECT_DOUBLE_EQ(gy, 197.0);
  EXPECT_DOUBLE_EQ(gz, 15.0);
}

TEST(MissionEnu, RoundTripFromLegacyRelative)
{
  const double sx = 1.5, sy = -2.5, sz = 15.7;
  const double gps_x = sx + 4.0;
  const double gps_y = sy + 7.0;
  const double gps_z = sz + 5.0;
  // Legacy relative: x=ΔE, y=-(ΔN), z=ΔU
  const double mx = gps_x - sx;
  const double my = -(gps_y - sy);
  const double mz = gps_z - sz;
  double ox, oy, oz;
  mission_args_to_gps_enu(sx, sy, sz, mx, my, mz, ox, oy, oz);
  EXPECT_NEAR(ox, gps_x, 1e-12);
  EXPECT_NEAR(oy, gps_y, 1e-12);
  EXPECT_NEAR(oz, gps_z, 1e-12);
}
