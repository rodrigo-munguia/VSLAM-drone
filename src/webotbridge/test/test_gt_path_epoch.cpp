#include <gtest/gtest.h>
#include <deque>
#include <string>

#include "gt_frame_conversions.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using webotbridge::gt::GpsSample;
using webotbridge::gt::GtReference;
using webotbridge::gt::clear_reference;
using webotbridge::gt::interpolate_gps;
using webotbridge::gt::webots_gps_to_public_gt;

namespace
{

struct PathEpochState
{
  uint64_t path_epoch_id{0};
  GtReference ref;
  nav_msgs::msg::Path path;
  int max_poses{5};
  std::string frame_id{"odom"};
};

void publish_empty(PathEpochState & s, int32_t sec)
{
  s.path.poses.clear();
  s.path.header.frame_id = s.frame_id;
  s.path.header.stamp.sec = sec;
  s.path.header.stamp.nanosec = 0;
}

void invalidate(PathEpochState & s, int32_t sec)
{
  clear_reference(s.ref);
  s.path_epoch_id = 0;
  publish_empty(s, sec);
}

bool append_if_current(
  PathEpochState & s,
  uint64_t sample_epoch,
  double t_sample,
  double t_min,
  double xw, double yw, double zw)
{
  if (!s.ref.resolved || sample_epoch != s.ref.epoch_id) {
    return false;
  }
  if (t_sample + 1e-9 < t_min) {
    return false;
  }
  double xp, yp, zp;
  webots_gps_to_public_gt(xw, yw, zw, s.ref, xp, yp, zp);
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = s.frame_id;
  ps.header.stamp.sec = static_cast<int32_t>(t_sample);
  ps.pose.position.x = xp;
  ps.pose.position.y = yp;
  ps.pose.position.z = zp;
  ps.pose.orientation.w = 1.0;
  s.path.poses.push_back(ps);
  s.path.header = ps.header;
  while (static_cast<int>(s.path.poses.size()) > s.max_poses) {
    s.path.poses.erase(s.path.poses.begin());
  }
  return true;
}

}  // namespace

TEST(GtPathEpoch, EmptyOnInvalidate)
{
  PathEpochState s;
  s.ref.resolved = true;
  s.ref.epoch_id = 3;
  s.path_epoch_id = 3;
  geometry_msgs::msg::PoseStamped ps;
  ps.pose.orientation.w = 1.0;
  s.path.poses.push_back(ps);
  invalidate(s, 10);
  EXPECT_FALSE(s.ref.resolved);
  EXPECT_TRUE(s.path.poses.empty());
  EXPECT_EQ(s.path.header.frame_id, "odom");
}

TEST(GtPathEpoch, HistoryLimitClamps)
{
  PathEpochState s;
  s.max_poses = 3;
  s.ref.resolved = true;
  s.ref.epoch_id = 1;
  s.path_epoch_id = 1;
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(append_if_current(s, 1, static_cast<double>(i), 0.0, i, 0, 0));
  }
  EXPECT_EQ(s.path.poses.size(), 3u);
  EXPECT_DOUBLE_EQ(s.path.poses.front().pose.position.x, 2.0);
}

TEST(GtPathEpoch, StaleSampleRejected)
{
  PathEpochState s;
  s.ref.resolved = true;
  s.ref.epoch_id = 2;
  EXPECT_FALSE(append_if_current(s, 2, 1.0, 2.0, 1, 0, 0));
  EXPECT_TRUE(s.path.poses.empty());
  EXPECT_TRUE(append_if_current(s, 2, 2.5, 2.0, 1, 0, 0));
  EXPECT_EQ(s.path.poses.size(), 1u);
}

TEST(GtPathEpoch, WrongEpochRejected)
{
  PathEpochState s;
  s.ref.resolved = true;
  s.ref.epoch_id = 5;
  EXPECT_FALSE(append_if_current(s, 4, 3.0, 0.0, 0, 0, 0));
}

TEST(GtPathEpoch, UnresolvedHistoryNoSilentLatest)
{
  std::deque<GpsSample> hist;
  hist.push_back({0.0, 1, 2, 3});
  hist.push_back({1.0, 4, 5, 6});
  GpsSample out;
  std::string reason;
  EXPECT_FALSE(interpolate_gps(hist, 10.0, 0.1, out, reason));
  EXPECT_NE(reason.find("after"), std::string::npos);
}

TEST(GtPathEpoch, IdentityQuaternionOnPathPose)
{
  PathEpochState s;
  s.ref.resolved = true;
  s.ref.epoch_id = 1;
  append_if_current(s, 1, 1.0, 0.0, 0, 0, 0);
  ASSERT_EQ(s.path.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(s.path.poses[0].pose.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(s.path.poses[0].pose.orientation.x, 0.0);
}
