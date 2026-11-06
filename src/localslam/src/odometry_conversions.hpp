#ifndef LOCALSLAM_ODOMETRY_CONVERSIONS_HPP_
#define LOCALSLAM_ODOMETRY_CONVERSIONS_HPP_

/**
 * A2: map the Euler EKF robot state / covariance to a public ROS odometry
 * snapshot. See docs/A2_odometry_contract.md.
 *
 * Internal Euler state (indices):
 *   0 unused, 1:3 Euler (φ,θ,ψ), 4:6 body ω, 7:9 nav position,
 *   10:12 body linear velocity.
 * Euler_to_Ra2b(φ,θ,ψ) = R_n←r (nav → robot / body).
 * ω, v are physical body rates/velocities in the internal body basis.
 *
 * Public axes (same as validated A1): D = diag(1,-1,-1) on nav and body
 * (x unchanged; y,z reversed). Not geographic ENU.
 */

#include <array>
#include <cmath>
#include <string>

namespace localslam
{
namespace odometry
{

/** Axis map between internal nav/body bases and public odom/base_link. */
inline constexpr double D_axis[3] = {1.0, -1.0, -1.0};

struct PublicOdometry
{
  double position[3]{0, 0, 0};
  /** Unit quaternion of R_odom←body (w, x, y, z), w >= 0. */
  double quat_wxyz[4]{1, 0, 0, 0};
  double twist_linear[3]{0, 0, 0};
  double twist_angular[3]{0, 0, 0};
  /** Row-major 6x6: [x y z rot_x rot_y rot_z] in parent (odom). */
  std::array<double, 36> pose_covariance{};
  /** Row-major 6x6: [vx vy vz wx wy wz] in child (body). */
  std::array<double, 36> twist_covariance{};
  bool valid{false};
  std::string reject_reason;
};

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

/** Transpose of column-major 3x3. */
inline void transpose3_cm(const double R_in[9], double R_out[9])
{
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R_out[c * 3 + r] = R_in[r * 3 + c];
    }
  }
}

/**
 * Map Euler-angle rates to internal-nav angular velocity:
 *   ω_n = R_r←n * R_e←b * ė
 * with R_b←e from the estimator prediction (ė = R_b←e ω_b).
 * Returns false near pitch singularity (|cos θ| too small).
 */
bool euler_rate_to_nav_angvel_jacobian(
  double phi, double theta, double psi,
  const double Rn2r_cm[9],
  double G_n_row_major[9],
  double cos_theta_eps = 1e-3);

/**
 * Jacobian of public odom-frame rotation-vector error w.r.t. Euler angles:
 *   δθ_pub ≈ J δe,  J = D * G_n(φ,θ,ψ)
 */
bool public_rotvec_euler_jacobian(
  double phi, double theta, double psi,
  double J_row_major[9],
  double cos_theta_eps = 1e-3);

/** Robust DCM (column-major, R_odom←body) → unit quaternion (wxyz), w>=0. */
bool dcm_to_quat_wxyz(const double R_cm[9], double q_wxyz[4]);

/** Build R_odom←body = D * R_n←r^T * D from Euler angles. */
bool public_world_from_body_dcm(
  double phi, double theta, double psi, double R_pub_cm[9]);

/**
 * Convert one consistent Euler robot state + 13x13 P block into public
 * odometry quantities. Does not modify the estimator.
 *
 * P_row_major: 13x13 row-major covariance of x[0..12].
 * Rejects non-finite state/P, pitch singularity, or non-PSD (within tol).
 */
bool convert_euler_state_to_public_odometry(
  const double x[13],
  const double P_row_major[13 * 13],
  PublicOdometry & out,
  double cos_theta_eps = 1e-3,
  double psd_eig_tol = -1e-9);

/** Symmetry check: max |A - A^T| for row-major n x n. */
double matrix_asymmetry(const double * A_rm, int n);

/** Smallest eigenvalue of symmetric 6x6 row-major (Jacobi), for PSD checks. */
double min_eig_sym6(const double A_rm[36]);

}  // namespace odometry
}  // namespace localslam

#endif  // LOCALSLAM_ODOMETRY_CONVERSIONS_HPP_
