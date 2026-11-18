#ifndef CONTROL_XY_TRAJ_REF_HPP_
#define CONTROL_XY_TRAJ_REF_HPP_

#include <algorithm>
#include <cmath>

namespace control
{

/**
 * CTRL-R2: single continuous horizontal position→velocity reference.
 *
 * Nav-frame law (e = target − pos, v = measured nav velocity):
 *   d = ||e||
 *   closing = (v · e) / d   (positive when approaching)
 *   d_eff = max(0, d − max(0, closing) · t_lag)
 *   v_pd = Kp·e − Kd·v
 *   v_cap = min(vmax, sqrt(2·a_brake·d_eff))
 *   v_des = scale(v_pd, v_cap)     (zero when d≈0)
 *
 * Kd damps the outer loop (prevents near-hover lateral limit cycles).
 * t_lag budgets S3/S2/S1 response delay. a_brake is a command envelope
 * below observed plant brake (~1–1.5 m/s²) so taper starts early.
 *
 * No settle latch, creep mode, near-field dual vmax, or closing override.
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

/** Desired nav-frame horizontal velocity [m/s]. */
inline void xy_traj_v_target_nav(
  double e_x, double e_y,
  double v_x, double v_y,
  double vmax, double kp, double kd, double a_brake, double t_lag,
  double & out_vx, double & out_vy)
{
  const double d = std::hypot(e_x, e_y);
  if (!(d > 1e-9) || !std::isfinite(d)) {
    out_vx = 0.0;
    out_vy = 0.0;
    return;
  }
  if (!std::isfinite(e_x) || !std::isfinite(e_y) ||
      !std::isfinite(v_x) || !std::isfinite(v_y))
  {
    out_vx = 0.0;
    out_vy = 0.0;
    return;
  }

  const double closing = (v_x * e_x + v_y * e_y) / d;
  const double d_eff = std::max(
    0.0, d - std::max(0.0, closing) * std::max(0.0, t_lag));
  const double a = std::abs(a_brake);
  double v_cap = std::min(std::abs(vmax), std::sqrt(2.0 * a * d_eff));
  if (!std::isfinite(v_cap) || v_cap < 0.0) {
    v_cap = 0.0;
  }

  // PD velocity request, then magnitude envelope (one continuous law).
  double vx = std::abs(kp) * e_x - kd * v_x;
  double vy = std::abs(kp) * e_y - kd * v_y;
  if (!std::isfinite(vx) || !std::isfinite(vy)) {
    out_vx = 0.0;
    out_vy = 0.0;
    return;
  }
  const double spd = std::hypot(vx, vy);
  if (spd > v_cap && spd > 1e-12) {
    const double s = v_cap / spd;
    vx *= s;
    vy *= s;
  }
  out_vx = vx;
  out_vy = vy;
}

/**
 * Rotate nav XY vector into body using the same Rn2b as go_point:
 *   Rn2b = {{cosψ, sinψ}, {-sinψ, cosψ}}
 */
inline void nav_xy_to_body(
  double n_x, double n_y, double yaw,
  double & b_x, double & b_y)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  b_x = c * n_x + s * n_y;
  b_y = -s * n_x + c * n_y;
}

/** Rate-limit a 2D velocity reference toward a target. */
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
 * Continuous asymmetric rate limit: brake reference faster than accelerate.
 * Part of one tracking law — not a terminal mode switch.
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
 * One step of the body-frame reference used by go_point:
 * nav target → body → rate-limit into (v_ref_x, v_ref_y).
 */
inline void xy_traj_step_body(
  double e_x, double e_y,
  double v_x_nav, double v_y_nav,
  double yaw,
  double vmax, double kp, double kd, double a_brake, double t_lag,
  double a_acc, double a_dec, double dt,
  double & v_ref_x, double & v_ref_y)
{
  double des_n_x = 0.0;
  double des_n_y = 0.0;
  xy_traj_v_target_nav(
    e_x, e_y, v_x_nav, v_y_nav, vmax, kp, kd, a_brake, t_lag,
    des_n_x, des_n_y);

  double des_b_x = 0.0;
  double des_b_y = 0.0;
  nav_xy_to_body(des_n_x, des_n_y, yaw, des_b_x, des_b_y);

  rate_limit_xy_asymmetric(
    v_ref_x, v_ref_y, des_b_x, des_b_y, a_acc, a_dec, dt);

  if (!std::isfinite(v_ref_x) || !std::isfinite(v_ref_y)) {
    v_ref_x = 0.0;
    v_ref_y = 0.0;
  }
}

}  // namespace control

#endif  // CONTROL_XY_TRAJ_REF_HPP_
