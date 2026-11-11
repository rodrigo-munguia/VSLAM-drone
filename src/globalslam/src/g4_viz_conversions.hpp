#ifndef GLOBALSLAM_G4_VIZ_CONVERSIONS_HPP_
#define GLOBALSLAM_G4_VIZ_CONVERSIONS_HPP_

/**
 * G4: public visualization helpers for Global SLAM.
 *
 * Internal KEYFRAME poses remain in the estimator's nav/camera conventions.
 * Public ROS outputs use the same A2 axis map as Local SLAM:
 *   D = diag(1, -1, -1)
 * applied once at the publish boundary.
 *
 * Path reference point: robot body origin (KEYFRAME.r_N / Rn2r), not camera.
 *   p_public = D * r_N
 *   R_odom←body = D * Rn2r^T * D
 * Do not apply D to quaternion components; convert the DCM after congruence.
 *
 * Path header.stamp = snapshot/publication time (not "all poses observed now").
 * Each PoseStamped keeps the keyframe's original source stamp when valid.
 *
 * PointCloud2: finite XYZ landmarks only; same frame_id and header.stamp as Path.
 * Visualization limits truncate published copies only — never the estimator map.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"

namespace globalslam
{
namespace g4_viz
{

inline constexpr double D_axis[3] = {1.0, -1.0, -1.0};

inline void apply_D(const double v_in[3], double v_out[3])
{
  v_out[0] = D_axis[0] * v_in[0];
  v_out[1] = D_axis[1] * v_in[1];
  v_out[2] = D_axis[2] * v_in[2];
}

/** Column-major 3x3: R_out = D * R_in * D. */
inline void congruence_D(const double R_in_cm[9], double R_out_cm[9])
{
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R_out_cm[c * 3 + r] =
        D_axis[r] * R_in_cm[c * 3 + r] * D_axis[c];
    }
  }
}

inline void transpose3_cm(const double R_in[9], double R_out[9])
{
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R_out[c * 3 + r] = R_in[r * 3 + c];
    }
  }
}

inline bool finite3(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

/** Robust DCM (column-major) → unit quaternion (wxyz), w >= 0. */
inline bool dcm_to_quat_wxyz(const double R_cm[9], double q_wxyz[4])
{
  const double r00 = R_cm[0], r01 = R_cm[3], r02 = R_cm[6];
  const double r10 = R_cm[1], r11 = R_cm[4], r12 = R_cm[7];
  const double r20 = R_cm[2], r21 = R_cm[5], r22 = R_cm[8];
  const double trace = r00 + r11 + r22;

  double w, x, y, z;
  if (trace > 0.0) {
    const double s = 0.5 / std::sqrt(trace + 1.0);
    w = 0.25 / s;
    x = (r21 - r12) * s;
    y = (r02 - r20) * s;
    z = (r10 - r01) * s;
  } else if (r00 > r11 && r00 > r22) {
    const double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
    w = (r21 - r12) / s;
    x = 0.25 * s;
    y = (r01 + r10) / s;
    z = (r02 + r20) / s;
  } else if (r11 > r22) {
    const double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
    w = (r02 - r20) / s;
    x = (r01 + r10) / s;
    y = 0.25 * s;
    z = (r12 + r21) / s;
  } else {
    const double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
    w = (r10 - r01) / s;
    x = (r02 + r20) / s;
    y = (r12 + r21) / s;
    z = 0.25 * s;
  }

  const double n = std::sqrt(w * w + x * x + y * y + z * z);
  if (!std::isfinite(n) || n < 1e-12) {
    return false;
  }
  w /= n;
  x /= n;
  y /= n;
  z /= n;
  if (w < 0.0) {
    w = -w;
    x = -x;
    y = -y;
    z = -z;
  }
  q_wxyz[0] = w;
  q_wxyz[1] = x;
  q_wxyz[2] = y;
  q_wxyz[3] = z;
  return true;
}

/**
 * Internal body pose (r_N, Rn2r = R_n←r column-major) → public odom pose.
 * Orientation is R_odom←body, not a direct D·quat.
 */
inline bool internal_body_to_public_pose(
  const double r_N[3],
  const double Rn2r_cm[9],
  double p_pub[3],
  double q_wxyz[4])
{
  if (!finite3(r_N[0], r_N[1], r_N[2])) {
    return false;
  }
  for (int i = 0; i < 9; ++i) {
    if (!std::isfinite(Rn2r_cm[i])) {
      return false;
    }
  }
  apply_D(r_N, p_pub);
  if (!finite3(p_pub[0], p_pub[1], p_pub[2])) {
    return false;
  }
  double Rr2n_cm[9];
  transpose3_cm(Rn2r_cm, Rr2n_cm);
  double R_pub_cm[9];
  congruence_D(Rr2n_cm, R_pub_cm);
  return dcm_to_quat_wxyz(R_pub_cm, q_wxyz);
}

/** Landmark XYZ in internal nav → public odom parent. */
inline bool internal_landmark_to_public(
  double x_int, double y_int, double z_int,
  double & x_pub, double & y_pub, double & z_pub)
{
  if (!finite3(x_int, y_int, z_int)) {
    return false;
  }
  const double vin[3] = {x_int, y_int, z_int};
  double vout[3];
  apply_D(vin, vout);
  if (!finite3(vout[0], vout[1], vout[2])) {
    return false;
  }
  x_pub = vout[0];
  y_pub = vout[1];
  z_pub = vout[2];
  return true;
}

/** Lightweight committed-map copy for unlocked serialization. */
struct SnapshotKeyframe
{
  double r_N[3]{0, 0, 0};
  double Rn2r_cm[9]{
    1, 0, 0,
    0, 1, 0,
    0, 0, 1};
  bool source_stamp_valid{false};
  int32_t stamp_sec{0};
  uint32_t stamp_nanosec{0};
  uint64_t keyframe_id{0};
};

struct SnapshotLandmark
{
  double x{0};
  double y{0};
  double z{0};
};

struct MapSnapshot
{
  uint64_t generation{0};
  uint64_t epoch_id{0};
  std::vector<SnapshotKeyframe> keyframes;
  std::vector<SnapshotLandmark> landmarks;
};

/**
 * Build Path from committed body poses in keyframe order.
 * max_poses <= 0 → no truncation (publish full snapshot).
 * Truncation drops oldest published poses only.
 */
inline nav_msgs::msg::Path make_keyframe_path(
  const std_msgs::msg::Header & header,
  const std::vector<SnapshotKeyframe> & keyframes,
  int max_poses)
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(keyframes.size());

  for (const auto & kf : keyframes) {
    double p[3];
    double q[4];
    if (!internal_body_to_public_pose(kf.r_N, kf.Rn2r_cm, p, q)) {
      continue;
    }
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = header.frame_id;
    if (kf.source_stamp_valid) {
      ps.header.stamp.sec = kf.stamp_sec;
      ps.header.stamp.nanosec = kf.stamp_nanosec;
    } else {
      ps.header.stamp = header.stamp;
    }
    ps.pose.position.x = p[0];
    ps.pose.position.y = p[1];
    ps.pose.position.z = p[2];
    ps.pose.orientation.w = q[0];
    ps.pose.orientation.x = q[1];
    ps.pose.orientation.y = q[2];
    ps.pose.orientation.z = q[3];
    path.poses.push_back(ps);
  }

  if (max_poses > 0) {
    while (static_cast<int>(path.poses.size()) > max_poses) {
      path.poses.erase(path.poses.begin());
    }
  }
  return path;
}

/**
 * Build XYZ PointCloud2. Non-finite landmarks skipped.
 * max_points <= 0 → no truncation. Truncation keeps earliest landmarks.
 * is_dense=true; only finite points are serialized.
 */
inline sensor_msgs::msg::PointCloud2 make_landmark_cloud(
  const std_msgs::msg::Header & header,
  const std::vector<SnapshotLandmark> & landmarks,
  int max_points)
{
  std::vector<float> xyz;
  xyz.reserve(landmarks.size() * 3);
  for (const auto & lm : landmarks) {
    double xp = 0, yp = 0, zp = 0;
    if (!internal_landmark_to_public(lm.x, lm.y, lm.z, xp, yp, zp)) {
      continue;
    }
    xyz.push_back(static_cast<float>(xp));
    xyz.push_back(static_cast<float>(yp));
    xyz.push_back(static_cast<float>(zp));
    if (max_points > 0 &&
      static_cast<int>(xyz.size() / 3) >= max_points)
    {
      break;
    }
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

inline nav_msgs::msg::Path make_empty_path(const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path;
  path.header = header;
  return path;
}

inline sensor_msgs::msg::PointCloud2 make_empty_cloud(
  const std_msgs::msg::Header & header)
{
  return make_landmark_cloud(header, {}, 0);
}

/**
 * Reconstruct internal camera position from body pose + extrinsics:
 *   t_c2n = r_N + Rn2r^T * t_c2r
 * Used only in tests that verify body vs camera reference points.
 */
inline void camera_position_from_body(
  const double r_N[3],
  const double Rn2r_cm[9],
  const double t_c2r[3],
  double t_c2n[3])
{
  double Rr2n[9];
  transpose3_cm(Rn2r_cm, Rr2n);
  t_c2n[0] = r_N[0] +
    Rr2n[0] * t_c2r[0] + Rr2n[3] * t_c2r[1] + Rr2n[6] * t_c2r[2];
  t_c2n[1] = r_N[1] +
    Rr2n[1] * t_c2r[0] + Rr2n[4] * t_c2r[1] + Rr2n[7] * t_c2r[2];
  t_c2n[2] = r_N[2] +
    Rr2n[2] * t_c2r[0] + Rr2n[5] * t_c2r[1] + Rr2n[8] * t_c2r[2];
}

}  // namespace g4_viz
}  // namespace globalslam

#endif  // GLOBALSLAM_G4_VIZ_CONVERSIONS_HPP_
