#ifndef LOCALSLAM_SENSOR_CONVERSIONS_HPP_
#define LOCALSLAM_SENSOR_CONVERSIONS_HPP_

/**
 * Boundary conversions between ROS-facing standard messages and the
 * estimator's existing internal observation fields.
 *
 * Frame documentation (A1 — no wholesale ENU/NED rewrite):
 *
 * Attitude
 *   Source (public): sensor_msgs/Imu.orientation (REP-145 body→fixed, z-up),
 *     interpreted with tf2 getRPY. frame_id as published by the driver.
 *   Destination: estimator ATT RPY for Euler_to_Ra2b after axis map
 *     D=diag(1,-1,-1) on fixed and body bases (x same; y,z reversed —
 *     matches webotbridge GT). Equivalent RPY witness:
 *       roll_int  =  roll_ros
 *       pitch_int = -pitch_ros
 *       yaw_int   = -yaw_ros
 *     Matrix form: Euler_to_Ra2b(att) == D * R_F←B^T * D.
 *
 * Body velocity
 *   Source (public): TwistWithCovarianceStamped.twist.linear in the robot
 *     body frame after Rn2r * GPS-speed (webotbridge), *before* Spd flips.
 *   Internal SPD:
 *       speedX =  v_ros.x
 *       speedY = -v_ros.y
 *       speedZ = -v_ros.z
 *
 * Altitude
 *   Source (public), exclusive by Altitude_input_mode:
 *     - derived_altitude: mavros_msgs/Altitude.relative [m], +up
 *       (Webots experiment: noisy GPS z).
 *     - fluid_pressure: sensor_msgs/FluidPressure.fluid_pressure [Pa]
 *       converted at the ROS boundary to relative altitude [m, +up] via
 *       ISA troposphere (see pressure_altitude.hpp). variance IGNORED.
 *   Internal altitude stores +up; Alt_h_2_d applied once in Altitude_update.
 *   Noise: Sigma_alt_update [m] only — never Pa² as m².
 *
 * Downward range
 *   Slant range [m] along ranger axis; no axis flip. Not range-to-base.
 */

#include <cmath>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "mavros_msgs/msg/altitude.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace localslam
{
namespace conversions
{

struct AttObs
{
  float roll{0.f};
  float pitch{0.f};
  float yaw{0.f};
  long int time{0};
};

struct SpdObs
{
  float speedX{0.f};
  float speedY{0.f};
  float speedZ{0.f};
  long int time{0};
};

struct AltObs
{
  double altitude{0.0};
  long int time{0};
};

struct RangeObs
{
  double range{0.0};
  double volt{0.0};
  long int time{0};
};

inline bool is_finite(double v)
{
  return std::isfinite(v);
}

inline bool imu_orientation_unavailable(const sensor_msgs::msg::Imu & msg)
{
  return msg.orientation_covariance.size() > 0 &&
         msg.orientation_covariance[0] == -1.0;
}

bool quaternion_usable(
  double x, double y, double z, double w, double norm_tol = 1e-3);

bool imu_to_internal_att(const sensor_msgs::msg::Imu & msg, AttObs & att_out);

bool body_twist_to_internal_spd(
  const geometry_msgs::msg::TwistWithCovarianceStamped & msg, SpdObs & spd_out);

bool mavros_relative_to_internal_alt(
  const mavros_msgs::msg::Altitude & msg, AltObs & alt_out);

bool range_msg_to_internal_range(
  const sensor_msgs::msg::Range & msg,
  RangeObs & range_out,
  double gate_min,
  double gate_max);

long int stamp_to_ms(const builtin_interfaces::msg::Time & stamp);

inline void ros_body_linear_to_internal(
  double vx, double vy, double vz, double & ix, double & iy, double & iz)
{
  ix = vx;
  iy = -vy;
  iz = -vz;
}

inline void ros_rpy_to_internal(
  double roll, double pitch, double yaw,
  double & i_roll, double & i_pitch, double & i_yaw)
{
  i_roll = roll;
  i_pitch = -pitch;
  i_yaw = -yaw;
}

}  // namespace conversions
}  // namespace localslam

#endif  // LOCALSLAM_SENSOR_CONVERSIONS_HPP_
