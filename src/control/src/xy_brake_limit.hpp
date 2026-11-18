#ifndef CONTROL_XY_BRAKE_LIMIT_HPP_
#define CONTROL_XY_BRAKE_LIMIT_HPP_

#include <algorithm>
#include <cmath>

namespace control
{

/**
 * Cap horizontal velocity command by cruise vmax and stopping distance.
 * v_cap = min(vmax, sqrt(2 * a_brake * dist_xy)).
 *
 * a_brake is the *command* envelope (conservative vs plant). Pair with
 * velocity-aware override using measured closing speed.
 */
inline void apply_xy_brake_limit(
  double & ux, double & uy, double dist_xy, double vmax, double a_brake = 0.5)
{
  const double umag = std::hypot(ux, uy);
  if (umag < 1e-9) {
    return;
  }
  double v_cap = std::sqrt(std::max(0.0, 2.0 * a_brake * std::max(dist_xy, 0.0)));
  v_cap = std::min(v_cap, std::abs(vmax));
  if (umag > v_cap) {
    const double s = v_cap / umag;
    ux *= s;
    uy *= s;
  }
}

/**
 * Closing speed toward waypoint [m/s]: positive when approaching.
 * e = (target - pos) in nav frame; v = nav-frame velocity.
 */
inline double closing_speed_xy(
  double e_x, double e_y, double v_x, double v_y)
{
  const double dist = std::hypot(e_x, e_y);
  if (dist < 1e-9) {
    return 0.0;
  }
  return (v_x * e_x + v_y * e_y) / dist;
}

/**
 * True when measured approach speed exceeds the distance-based stop envelope.
 * closing^2 > 2 * a_brake * dist  ⇒  cannot stop in remaining distance.
 */
inline bool closing_too_fast(
  double closing, double dist_xy, double a_brake)
{
  if (closing <= 0.0) {
    return false;  // departing or stationary — P law handles
  }
  const double d = std::max(dist_xy, 0.0);
  return closing * closing > 2.0 * a_brake * d;
}

/**
 * Rate-limit a 2D velocity reference toward a target.
 * v ← v + limitNorm(target - v, a_cmd_max * dt)
 */
inline void rate_limit_xy(
  double & vx, double & vy, double target_x, double target_y,
  double a_cmd_max, double dt)
{
  const double max_du = std::abs(a_cmd_max) * std::max(dt, 1e-6);
  double dx = target_x - vx;
  double dy = target_y - vy;
  const double du = std::hypot(dx, dy);
  if (du > max_du && du > 1e-12) {
    const double s = max_du / du;
    dx *= s;
    dy *= s;
  }
  vx += dx;
  vy += dy;
}

/**
 * Asymmetric rate limit: allow faster reduction of |v| than increase.
 * Prevents the accel slew from delaying braking into an overshoot.
 */
inline void rate_limit_xy_asymmetric(
  double & vx, double & vy, double target_x, double target_y,
  double a_acc, double a_dec, double dt)
{
  const double cur = std::hypot(vx, vy);
  const double tgt = std::hypot(target_x, target_y);
  const double a = (tgt < cur - 1e-9) ? a_dec : a_acc;
  rate_limit_xy(vx, vy, target_x, target_y, a, dt);
}

/**
 * Scale a unit-direction (or any) XY command to a creep speed.
 */
inline void scale_xy_to_speed(double & ux, double & uy, double speed)
{
  const double umag = std::hypot(ux, uy);
  if (umag < 1e-9) {
    ux = 0.0;
    uy = 0.0;
    return;
  }
  const double s = speed / umag;
  ux *= s;
  uy *= s;
}

}  // namespace control

#endif  // CONTROL_XY_BRAKE_LIMIT_HPP_
