#ifndef LOCALSLAM_MAP_PATH_CONVERSIONS_HPP_
#define LOCALSLAM_MAP_PATH_CONVERSIONS_HPP_

/**
 * A4: landmark and path helpers for public ROS outputs.
 *
 * Landmark positions use the same A2 axis map D=diag(1,-1,-1) into the
 * odometry parent frame. The cloud is a changing local sparse EKF map
 * (features + anchors currently in the estimator), not a persistent
 * global reconstruction. State indices are not stable IDs and are not
 * published.
 *
 * Path poses reuse accepted A2 odometry snapshots. Stamps are publication
 * time (same meaning as ~/odometry).
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "odometry_conversions.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"

namespace localslam
{
namespace map_path
{

inline bool finite3(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

/** Internal nav landmark → public odom parent. Rejects non-finite inputs. */
inline bool internal_landmark_to_public(
  double x_int, double y_int, double z_int,
  double & x_pub, double & y_pub, double & z_pub)
{
  if (!finite3(x_int, y_int, z_int)) {
    return false;
  }
  const double vin[3] = {x_int, y_int, z_int};
  double vout[3];
  odometry::apply_D(vin, vout);
  if (!finite3(vout[0], vout[1], vout[2])) {
    return false;
  }
  x_pub = vout[0];
  y_pub = vout[1];
  z_pub = vout[2];
  return true;
}

struct LandmarkXYZ
{
  double x{0};
  double y{0};
  double z{0};
};

/**
 * Build XYZ PointCloud2 in parent_frame from internal-nav landmarks.
 * Invalid points are skipped. Empty input → empty cloud (width=0) so
 * consumers can clear a previous map.
 */
inline sensor_msgs::msg::PointCloud2 make_landmark_cloud(
  const std_msgs::msg::Header & header,
  const std::vector<LandmarkXYZ> & internal_points)
{
  std::vector<float> xyz;
  xyz.reserve(internal_points.size() * 3);
  for (const auto & p : internal_points) {
    double xp = 0, yp = 0, zp = 0;
    if (!internal_landmark_to_public(p.x, p.y, p.z, xp, yp, zp)) {
      continue;
    }
    xyz.push_back(static_cast<float>(xp));
    xyz.push_back(static_cast<float>(yp));
    xyz.push_back(static_cast<float>(zp));
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(xyz.size() / 3);
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;

  sensor_msgs::msg::PointField fx;
  fx.name = "x";
  fx.offset = 0;
  fx.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fx.count = 1;
  sensor_msgs::msg::PointField fy = fx;
  fy.name = "y";
  fy.offset = 4;
  sensor_msgs::msg::PointField fz = fx;
  fz.name = "z";
  fz.offset = 8;
  cloud.fields = {fx, fy, fz};

  cloud.data.resize(cloud.row_step);
  if (!xyz.empty()) {
    std::memcpy(cloud.data.data(), xyz.data(), xyz.size() * sizeof(float));
  }
  return cloud;
}

/** Empty cloud with the same frame/stamp contract (clears consumers). */
inline sensor_msgs::msg::PointCloud2 make_empty_landmark_cloud(
  const std_msgs::msg::Header & header)
{
  return make_landmark_cloud(header, {});
}

/**
 * Append one accepted odometry pose. Drops oldest when size would exceed
 * max_poses (max_poses <= 0 → treat as 1).
 */
inline void append_path_pose(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose,
  int max_poses)
{
  const int cap = max_poses > 0 ? max_poses : 1;
  path.poses.push_back(pose);
  while (static_cast<int>(path.poses.size()) > cap) {
    path.poses.erase(path.poses.begin());
  }
}

inline void clear_path(nav_msgs::msg::Path & path)
{
  path.poses.clear();
}

}  // namespace map_path
}  // namespace localslam

#endif  // LOCALSLAM_MAP_PATH_CONVERSIONS_HPP_
