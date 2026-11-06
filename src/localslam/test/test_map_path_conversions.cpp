#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "map_path_conversions.hpp"

using localslam::map_path::append_path_pose;
using localslam::map_path::clear_path;
using localslam::map_path::internal_landmark_to_public;
using localslam::map_path::LandmarkXYZ;
using localslam::map_path::make_empty_landmark_cloud;
using localslam::map_path::make_landmark_cloud;

TEST(MapPath, LandmarkUsesA2AxisMapD)
{
  double xp = 0, yp = 0, zp = 0;
  ASSERT_TRUE(internal_landmark_to_public(1.5, -2.5, 3.25, xp, yp, zp));
  EXPECT_DOUBLE_EQ(xp, 1.5);
  EXPECT_DOUBLE_EQ(yp, 2.5);
  EXPECT_DOUBLE_EQ(zp, -3.25);
}

TEST(MapPath, LandmarkRejectsNonFinite)
{
  double xp = 0, yp = 0, zp = 0;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(internal_landmark_to_public(nan, 0, 0, xp, yp, zp));
  EXPECT_FALSE(internal_landmark_to_public(0, inf, 0, xp, yp, zp));
  EXPECT_FALSE(internal_landmark_to_public(0, 0, -inf, xp, yp, zp));
}

TEST(MapPath, EmptyCloudAndInvalidSkip)
{
  std_msgs::msg::Header h;
  h.frame_id = "odom";
  auto empty = make_empty_landmark_cloud(h);
  EXPECT_EQ(empty.width, 0u);
  EXPECT_EQ(empty.height, 1u);
  EXPECT_EQ(empty.header.frame_id, "odom");
  EXPECT_EQ(empty.fields.size(), 3u);
  EXPECT_TRUE(empty.data.empty());

  std::vector<LandmarkXYZ> pts = {
    {1.0, 2.0, 3.0},
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
    {4.0, 5.0, 6.0},
  };
  auto cloud = make_landmark_cloud(h, pts);
  EXPECT_EQ(cloud.width, 2u);
  EXPECT_EQ(cloud.point_step, 12u);
  EXPECT_EQ(cloud.data.size(), 24u);
  // First point: D*(1,2,3) = (1,-2,-3)
  float x0, y0, z0;
  std::memcpy(&x0, cloud.data.data() + 0, 4);
  std::memcpy(&y0, cloud.data.data() + 4, 4);
  std::memcpy(&z0, cloud.data.data() + 8, 4);
  EXPECT_FLOAT_EQ(x0, 1.0f);
  EXPECT_FLOAT_EQ(y0, -2.0f);
  EXPECT_FLOAT_EQ(z0, -3.0f);
}

TEST(MapPath, PathBoundsFramePauseAndReset)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";

  geometry_msgs::msg::PoseStamped a;
  a.header.frame_id = "odom";
  a.pose.position.x = 1.0;
  a.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped b = a;
  b.pose.position.x = 2.0;
  geometry_msgs::msg::PoseStamped c = a;
  c.pose.position.x = 3.0;
  geometry_msgs::msg::PoseStamped d = a;
  d.pose.position.x = 4.0;

  append_path_pose(path, a, 3);
  append_path_pose(path, b, 3);
  append_path_pose(path, c, 3);
  EXPECT_EQ(path.poses.size(), 3u);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 3.0);

  append_path_pose(path, d, 3);
  EXPECT_EQ(path.poses.size(), 3u);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 4.0);

  for (const auto & p : path.poses) {
    EXPECT_EQ(p.header.frame_id, "odom");
  }
  EXPECT_EQ(path.header.frame_id, "odom");

  // Disable / pause: no append when not calling append_path_pose.
  const auto size_paused = path.poses.size();
  EXPECT_EQ(size_paused, 3u);

  clear_path(path);
  EXPECT_TRUE(path.poses.empty());
  // Cleared path remains publishable (empty poses).
  EXPECT_EQ(path.header.frame_id, "odom");
}

TEST(MapPath, PathMaxPosesNonPositiveTreatedAsOne)
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped a;
  a.pose.orientation.w = 1.0;
  a.pose.position.x = 1.0;
  geometry_msgs::msg::PoseStamped b = a;
  b.pose.position.x = 2.0;
  append_path_pose(path, a, 0);
  append_path_pose(path, b, 0);
  EXPECT_EQ(path.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 2.0);
}
