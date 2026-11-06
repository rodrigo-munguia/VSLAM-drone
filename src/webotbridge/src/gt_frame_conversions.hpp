#ifndef WEBOTBRIDGE_GT_FRAME_CONVERSIONS_HPP_
#define WEBOTBRIDGE_GT_FRAME_CONVERSIONS_HPP_

/**
 * Fixed ground-truth map: Webots GPS (ENU, z-up) → public odom parent (+up).
 *
 * Public odometry (A2): p_pub = D * p_int with D = diag(1,-1,-1).
 *
 * Reference epoch comes from Local SLAM EstimatorReference (not first
 * /odometry / empty Path). Heading and altitude may latch at different times.
 *
 *   yaw_enu = EstimatorReference.yaw_enu_equivalent  (= -yaw_at_home_internal)
 *   p_xy0   = GPS interpolated at heading_stamp
 *   p_z0    = GPS interpolated at altitude_stamp (when altitude_valid)
 *
 *   p_pub = R_z(-yaw_enu) * (p_w - p_ref) + D * Robot_init_internal
 *
 * where p_ref.xy = p_xy0.xy and p_ref.z = p_z0.z (or p_xy0.z if altitude
 * updates are disabled).
 *
 * R_z(-yaw) = [[ cos(yaw), sin(yaw), 0],
 *              [-sin(yaw), cos(yaw), 0],
 *              [ 0,        0,       1]]  (det +1)
 *
 * Heading uses the noisy attitude measurement accepted by SLAM (via
 * yaw_enu_equivalent), not a noise-free simulator heading.
 */

#include <cmath>
#include <cstdint>
#include <deque>
#include <string>

#include "builtin_interfaces/msg/time.hpp"

namespace webotbridge
{
namespace gt
{

inline constexpr double kD_axis[3] = {1.0, -1.0, -1.0};

inline double stamp_to_sec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

inline builtin_interfaces::msg::Time sec_to_stamp(double sec)
{
  builtin_interfaces::msg::Time t;
  if (!std::isfinite(sec) || sec < 0.0) {
    t.sec = 0;
    t.nanosec = 0;
    return t;
  }
  const auto s = static_cast<int32_t>(std::floor(sec));
  t.sec = s;
  t.nanosec = static_cast<uint32_t>(std::llround((sec - static_cast<double>(s)) * 1e9));
  if (t.nanosec >= 1000000000u) {
    t.sec += 1;
    t.nanosec -= 1000000000u;
  }
  return t;
}

inline double wrap_angle(double a)
{
  constexpr double pi = 3.14159265358979323846;
  while (a > pi) {
    a -= 2.0 * pi;
  }
  while (a < -pi) {
    a += 2.0 * pi;
  }
  return a;
}

inline double lerp_angle(double a0, double a1, double u)
{
  const double d = wrap_angle(a1 - a0);
  return wrap_angle(a0 + u * d);
}

struct GpsSample
{
  double t_sec{0.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ImuYawSample
{
  double t_sec{0.0};
  double yaw_enu{0.0};
};

struct GtReference
{
  bool resolved{false};
  uint64_t epoch_id{0};
  double yaw_enu{0.0};
  double origin_xy[3]{0.0, 0.0, 0.0};
  double origin_z{0.0};
  double robot_init_internal[3]{0.0, 0.0, 0.0};
  std::string unresolved_reason;
};

inline void internal_robot_init_to_public(
  double xi, double yi, double zi,
  double & xp, double & yp, double & zp)
{
  xp = kD_axis[0] * xi;
  yp = kD_axis[1] * yi;
  zp = kD_axis[2] * zi;
}

inline void rotate_world_to_heading(
  double dx_w, double dy_w, double dz_w,
  double yaw_enu,
  double & dx_h, double & dy_h, double & dz_h)
{
  const double c = std::cos(yaw_enu);
  const double s = std::sin(yaw_enu);
  dx_h = c * dx_w + s * dy_w;
  dy_h = -s * dx_w + c * dy_w;
  dz_h = dz_w;
}

inline void webots_gps_to_public_gt(
  double x_w, double y_w, double z_w,
  double yaw_enu,
  double origin_x, double origin_y, double origin_z,
  double robot_init_internal_x,
  double robot_init_internal_y,
  double robot_init_internal_z,
  double & x_pub, double & y_pub, double & z_pub)
{
  double hx = 0.0, hy = 0.0, hz = 0.0;
  rotate_world_to_heading(
    x_w - origin_x, y_w - origin_y, z_w - origin_z,
    yaw_enu, hx, hy, hz);
  double ox = 0.0, oy = 0.0, oz = 0.0;
  internal_robot_init_to_public(
    robot_init_internal_x, robot_init_internal_y, robot_init_internal_z,
    ox, oy, oz);
  x_pub = hx + ox;
  y_pub = hy + oy;
  z_pub = hz + oz;
}

inline void webots_gps_to_public_gt(
  double x_w, double y_w, double z_w,
  const GtReference & ref,
  double & x_pub, double & y_pub, double & z_pub)
{
  webots_gps_to_public_gt(
    x_w, y_w, z_w, ref.yaw_enu,
    ref.origin_xy[0], ref.origin_xy[1], ref.origin_z,
    ref.robot_init_internal[0], ref.robot_init_internal[1],
    ref.robot_init_internal[2],
    x_pub, y_pub, z_pub);
}

/**
 * Interpolate a GPS sample at t_query.
 * Clock domain: ROS Time seconds from message header.stamp (sim time when
 * use_sim_time is set). Linear interpolation; max |dt| to nearest sample
 * must be <= max_dt_sec. No silent latest-sample fallback.
 */
inline bool interpolate_gps(
  const std::deque<GpsSample> & hist,
  double t_query,
  double max_dt_sec,
  GpsSample & out,
  std::string & reason)
{
  if (hist.empty()) {
    reason = "empty GPS history";
    return false;
  }
  if (hist.size() == 1) {
    if (std::abs(hist.front().t_sec - t_query) > max_dt_sec) {
      reason = "single GPS sample outside max_dt";
      return false;
    }
    out = hist.front();
    return true;
  }
  // Find bracketing pair
  if (t_query < hist.front().t_sec) {
    if (hist.front().t_sec - t_query > max_dt_sec) {
      reason = "query before GPS history window";
      return false;
    }
    out = hist.front();
    return true;
  }
  if (t_query > hist.back().t_sec) {
    if (t_query - hist.back().t_sec > max_dt_sec) {
      reason = "query after GPS history window";
      return false;
    }
    out = hist.back();
    return true;
  }
  for (size_t i = 1; i < hist.size(); ++i) {
    const auto & a = hist[i - 1];
    const auto & b = hist[i];
    if (t_query >= a.t_sec && t_query <= b.t_sec) {
      // Interior bracket: interpolate. max_dt applies to extrapolation
      // outside the history window (handled above), not to the distance
      // from query to the nearer sample when both sides exist.
      const double span = b.t_sec - a.t_sec;
      const double u = (span > 1e-12) ? ((t_query - a.t_sec) / span) : 0.0;
      out.t_sec = t_query;
      out.x = a.x + u * (b.x - a.x);
      out.y = a.y + u * (b.y - a.y);
      out.z = a.z + u * (b.z - a.z);
      return true;
    }
  }
  reason = "GPS interpolate failed";
  return false;
}

inline bool interpolate_yaw(
  const std::deque<ImuYawSample> & hist,
  double t_query,
  double max_dt_sec,
  double & yaw_out,
  std::string & reason)
{
  if (hist.empty()) {
    reason = "empty IMU yaw history";
    return false;
  }
  if (hist.size() == 1) {
    if (std::abs(hist.front().t_sec - t_query) > max_dt_sec) {
      reason = "single IMU sample outside max_dt";
      return false;
    }
    yaw_out = hist.front().yaw_enu;
    return true;
  }
  if (t_query < hist.front().t_sec) {
    if (hist.front().t_sec - t_query > max_dt_sec) {
      reason = "query before IMU history";
      return false;
    }
    yaw_out = hist.front().yaw_enu;
    return true;
  }
  if (t_query > hist.back().t_sec) {
    if (t_query - hist.back().t_sec > max_dt_sec) {
      reason = "query after IMU history";
      return false;
    }
    yaw_out = hist.back().yaw_enu;
    return true;
  }
  for (size_t i = 1; i < hist.size(); ++i) {
    const auto & a = hist[i - 1];
    const auto & b = hist[i];
    if (t_query >= a.t_sec && t_query <= b.t_sec) {
      const double span = b.t_sec - a.t_sec;
      const double u = (span > 1e-12) ? ((t_query - a.t_sec) / span) : 0.0;
      yaw_out = lerp_angle(a.yaw_enu, b.yaw_enu, u);
      return true;
    }
  }
  reason = "IMU yaw interpolate failed";
  return false;
}

/** Trim history older than (newest - duration). */
template<typename Sample>
inline void trim_history(std::deque<Sample> & hist, double duration_sec)
{
  if (hist.empty() || duration_sec <= 0.0) {
    return;
  }
  const double newest = hist.back().t_sec;
  while (!hist.empty() && (newest - hist.front().t_sec) > duration_sec) {
    hist.pop_front();
  }
}

inline void clear_reference(GtReference & ref)
{
  ref.resolved = false;
  ref.epoch_id = 0;
  ref.unresolved_reason = "cleared";
}

}  // namespace gt
}  // namespace webotbridge

#endif  // WEBOTBRIDGE_GT_FRAME_CONVERSIONS_HPP_
