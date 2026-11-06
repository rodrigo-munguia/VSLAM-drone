#ifndef CONTROL_ODOMETRY_RADIUS_FEEDBACK_HPP_
#define CONTROL_ODOMETRY_RADIUS_FEEDBACK_HPP_

/**
 * A5b: in Slam_feedback_interface=odometry, pose comes only from Odometry
 * while radius_uncertainty is taken from the existing Robotstate topic.
 *
 * Robotstate has no header/timestamp, so radius is treated as the latest
 * independent sample — not paired to a specific Odometry stamp.
 */

#include <cmath>
#include <limits>

namespace control
{
namespace odometry_radius
{

struct Pose6
{
  double x{0};
  double y{0};
  double z{0};
  double roll{0};
  double pitch{0};
  double yaw{0};
};

struct FeedbackState
{
  Pose6 pose;
  double r_u{std::numeric_limits<double>::quiet_NaN()};
};

/** Apply Odometry-derived pose without modifying r_u. */
inline void apply_odometry_pose(FeedbackState & s, const Pose6 & pose)
{
  s.pose = pose;
  // intentionally leave s.r_u unchanged
}

/**
 * Apply Robotstate.radius_uncertainty only (ignore conflicting pose fields).
 * Availability is independent of pose-feedback readiness.
 */
inline void apply_robotstate_radius_only(FeedbackState & s, double radius_uncertainty)
{
  s.r_u = radius_uncertainty;
}

/** Clear radius on control re-init / after preventing stale pre-reset use. */
inline void clear_radius(FeedbackState & s)
{
  s.r_u = std::numeric_limits<double>::quiet_NaN();
}

/**
 * A6 coordinated-reset gate: while !armed, radius callbacks must drop.
 * clear_and_disarm → NaN + armed=false; arm_after_odometry → armed=true.
 */
inline void clear_and_disarm(FeedbackState & s, bool & armed)
{
  clear_radius(s);
  armed = false;
}

inline bool accept_radius_if_armed(
  FeedbackState & s, double radius_uncertainty, bool armed)
{
  if (!armed) {
    return false;
  }
  apply_robotstate_radius_only(s, radius_uncertainty);
  return true;
}

inline void arm_after_odometry(bool & armed)
{
  armed = true;
}

}  // namespace odometry_radius
}  // namespace control

#endif  // CONTROL_ODOMETRY_RADIUS_FEEDBACK_HPP_
