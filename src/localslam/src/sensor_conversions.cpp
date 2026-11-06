#include "sensor_conversions.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace localslam
{
namespace conversions
{

bool quaternion_usable(
  double x, double y, double z, double w, double norm_tol)
{
  if (!is_finite(x) || !is_finite(y) || !is_finite(z) || !is_finite(w)) {
    return false;
  }
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (!is_finite(n) || n < 1e-6) {
    return false;
  }
  return std::abs(n - 1.0) <= norm_tol;
}

long int stamp_to_ms(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<long int>(stamp.sec) * 1000L +
         static_cast<long int>(stamp.nanosec) / 1000000L;
}

bool imu_to_internal_att(const sensor_msgs::msg::Imu & msg, AttObs & att_out)
{
  if (imu_orientation_unavailable(msg)) {
    return false;
  }
  const auto & q = msg.orientation;
  if (!quaternion_usable(q.x, q.y, q.z, q.w)) {
    return false;
  }

  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 mat(quat);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  mat.getRPY(roll, pitch, yaw);

  double i_roll = 0.0;
  double i_pitch = 0.0;
  double i_yaw = 0.0;
  ros_rpy_to_internal(roll, pitch, yaw, i_roll, i_pitch, i_yaw);

  att_out.roll = static_cast<float>(i_roll);
  att_out.pitch = static_cast<float>(i_pitch);
  att_out.yaw = static_cast<float>(i_yaw);
  att_out.time = stamp_to_ms(msg.header.stamp);
  return true;
}

bool body_twist_to_internal_spd(
  const geometry_msgs::msg::TwistWithCovarianceStamped & msg, SpdObs & spd_out)
{
  const auto & v = msg.twist.twist.linear;
  if (!is_finite(v.x) || !is_finite(v.y) || !is_finite(v.z)) {
    return false;
  }
  double ix = 0.0;
  double iy = 0.0;
  double iz = 0.0;
  ros_body_linear_to_internal(v.x, v.y, v.z, ix, iy, iz);
  spd_out.speedX = static_cast<float>(ix);
  spd_out.speedY = static_cast<float>(iy);
  spd_out.speedZ = static_cast<float>(iz);
  spd_out.time = stamp_to_ms(msg.header.stamp);
  return true;
}

bool mavros_relative_to_internal_alt(
  const mavros_msgs::msg::Altitude & msg, AltObs & alt_out)
{
  const double relative = static_cast<double>(msg.relative);
  if (!is_finite(relative)) {
    return false;
  }
  alt_out.altitude = relative;
  alt_out.time = stamp_to_ms(msg.header.stamp);
  return true;
}

bool range_msg_to_internal_range(
  const sensor_msgs::msg::Range & msg,
  RangeObs & range_out,
  double gate_min,
  double gate_max)
{
  const double r = static_cast<double>(msg.range);
  if (!is_finite(r)) {
    return false;
  }
  if (msg.min_range < msg.max_range) {
    if (r < msg.min_range || r > msg.max_range) {
      return false;
    }
  }
  if (r <= gate_min || r >= gate_max) {
    return false;
  }
  range_out.range = r;
  range_out.volt = 0.0;
  range_out.time = stamp_to_ms(msg.header.stamp);
  return true;
}

}  // namespace conversions
}  // namespace localslam
