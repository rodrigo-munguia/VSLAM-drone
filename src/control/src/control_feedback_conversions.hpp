#ifndef CONTROL_FEEDBACK_CONVERSIONS_HPP_
#define CONTROL_FEEDBACK_CONVERSIONS_HPP_

/**
 * A3: map Local SLAM public nav_msgs/Odometry (A2 contract) into the
 * controller's existing r_c / r_s conventions (formerly filled from
 * interfaces/Robotstate).
 *
 * Controllers consume (after legacy Slam_callback transforms):
 *   r_c.x = x_int
 *   r_c.y = y_int
 *   r_c.z = -z_int + init_z
 *   r_c.roll = roll_int
 *   r_c.pitch = -pitch_int
 *   r_c.yaw = -wrap(yaw_int - init_yaw_int)
 *
 * With A2 D=diag(1,-1,-1) and R_odom←body ≡ ROS body→parent:
 *   (x,y,z)_int = (x_pub, -y_pub, -z_pub)
 *   (roll,pitch,yaw)_int = (roll_pub, -pitch_pub, -yaw_pub)
 * which simplifies to control-frame:
 *   r_c.x = x_pub
 *   r_c.y = -y_pub
 *   r_c.z = z_pub + init_z
 *   r_c.roll = roll_pub
 *   r_c.pitch = pitch_pub
 *   r_c.yaw = wrap(yaw_pub - init_yaw_pub)
 *
 * radius_uncertainty (r_u) is NOT present in Odometry and cannot be
 * reconstructed from pose covariance (docs/A5_cl_radius_contract.md).
 * A5b: with Slam_feedback_interface=odometry, pose uses this mapping while
 * r_u is taken from Robotstate.radius_uncertainty only (radius-only callback).
 */

#include <cmath>
#include <string>

namespace control
{
namespace feedback
{

struct ControlPoseFeedback
{
  double x{0};
  double y{0};
  double z{0};
  double roll{0};
  double pitch{0};
  double yaw{0};
};

/** Internal-nav pose reconstructed from public odometry (for r_s logging). */
struct InternalSlamPose
{
  double x{0};
  double y{0};
  double z{0};
  double roll{0};
  double pitch{0};
  double yaw{0};
};

inline double wrap_pi(double angle)
{
  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  } else if (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  return angle;
}

/**
 * Convert public odometry pose (parent = odom, R_odom←body as RPY) into
 * controller feedback pose. init_yaw_pub / init_z match legacy latches
 * (first sample yaw in public convention; altitude latch from GT).
 */
inline ControlPoseFeedback odometry_pose_to_control(
  double x_pub, double y_pub, double z_pub,
  double roll_pub, double pitch_pub, double yaw_pub,
  double init_yaw_pub, double init_z)
{
  ControlPoseFeedback c;
  c.x = x_pub;
  c.y = -y_pub;
  c.z = z_pub + init_z;
  c.roll = roll_pub;
  c.pitch = pitch_pub;
  c.yaw = wrap_pi(yaw_pub - init_yaw_pub);
  return c;
}

/** Public odometry → internal SLAM indices used historically in r_s. */
inline InternalSlamPose odometry_pose_to_internal_slam(
  double x_pub, double y_pub, double z_pub,
  double roll_pub, double pitch_pub, double yaw_pub)
{
  InternalSlamPose s;
  s.x = x_pub;
  s.y = -y_pub;
  s.z = -z_pub;
  s.roll = roll_pub;
  s.pitch = -pitch_pub;
  s.yaw = -yaw_pub;
  return s;
}

/**
 * Legacy Robotstate (internal) → control feedback — same formulas as
 * Slam_callback, for equivalence tests.
 */
inline ControlPoseFeedback legacy_internal_to_control(
  double x_int, double y_int, double z_int,
  double roll_int, double pitch_int, double yaw_int,
  double init_yaw_int, double init_z)
{
  ControlPoseFeedback c;
  c.x = x_int;
  c.y = y_int;
  c.z = -z_int + init_z;
  c.roll = roll_int;
  c.pitch = -pitch_int;
  c.yaw = -wrap_pi(yaw_int - init_yaw_int);
  return c;
}

inline bool expected_odometry_frames(
  const std::string & frame_id,
  const std::string & child_frame_id,
  const std::string & expect_parent,
  const std::string & expect_child)
{
  return frame_id == expect_parent && child_frame_id == expect_child;
}

inline bool finite6(double a, double b, double c, double d, double e, double f)
{
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c) &&
         std::isfinite(d) && std::isfinite(e) && std::isfinite(f);
}

}  // namespace feedback
}  // namespace control

#endif  // CONTROL_FEEDBACK_CONVERSIONS_HPP_
