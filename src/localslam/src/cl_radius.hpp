#ifndef LOCALSLAM_CL_RADIUS_HPP_
#define LOCALSLAM_CL_RADIUS_HPP_

/**
 * A5: pure helpers documenting the legacy Cl_radius_uncertainty definition
 * from EKF::prediction_compute_CL_cov (ekf_prediction.cpp).
 *
 * This quantity is NOT a function of the EKF pose covariance P and cannot be
 * reconstructed from nav_msgs/Odometry pose covariance (A2). See
 * docs/A5_cl_radius_contract.md.
 */

#include <algorithm>
#include <cmath>

namespace localslam
{
namespace cl_radius
{

/**
 * Legacy radius: r_u = sqrt(max(Pc_00, Pc_11)).
 * No confidence multiplier. Horizontal only. Units: metres.
 */
inline double radius_from_Pc(double Pc_00, double Pc_11)
{
  return std::sqrt(std::max(Pc_00, Pc_11));
}

/**
 * One prediction step of Pc (diagonal accumulation; Q is diagonal).
 * Gate: max(|vx|,|vy|) > v_min (body velocities, m/s).
 * Pc ← Pc + diag(σ_x², σ_y²) Δt
 */
inline void accumulate_Pc_step(
  double & Pc_00, double & Pc_11,
  double vx, double vy, double delta_t,
  double sigma_x, double sigma_y, double v_min)
{
  if (std::max(std::abs(vx), std::abs(vy)) > v_min) {
    Pc_00 += (sigma_x * sigma_x) * delta_t;
    Pc_11 += (sigma_y * sigma_y) * delta_t;
  }
}

/**
 * Illustrates that EKF pose-position variances are a different quantity.
 * Returns true only if numerically equal within tol — expected false in
 * general for independent Pc vs P.
 */
inline bool pose_xy_variance_equals_Pc(
  double P_xx, double P_yy,
  double Pc_00, double Pc_11,
  double tol = 1e-12)
{
  return std::abs(P_xx - Pc_00) <= tol && std::abs(P_yy - Pc_11) <= tol;
}

}  // namespace cl_radius
}  // namespace localslam

#endif  // LOCALSLAM_CL_RADIUS_HPP_
