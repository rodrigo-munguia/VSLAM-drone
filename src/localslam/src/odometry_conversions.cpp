#include "odometry_conversions.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

#include "../../common/Transforms/Euler_to_Ra2b.hpp"

namespace localslam
{
namespace odometry
{
namespace
{

bool all_finite(const double * v, int n)
{
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(v[i])) {
      return false;
    }
  }
  return true;
}

void mat3_mul(const double A_rm[9], const double B_rm[9], double C_rm[9])
{
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      C_rm[r * 3 + c] =
        A_rm[r * 3 + 0] * B_rm[0 * 3 + c] +
        A_rm[r * 3 + 1] * B_rm[1 * 3 + c] +
        A_rm[r * 3 + 2] * B_rm[2 * 3 + c];
    }
  }
}

void cm_to_rm(const double R_cm[9], double R_rm[9])
{
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R_rm[r * 3 + c] = R_cm[c * 3 + r];
    }
  }
}

void extract3x3(
  const double P_rm[13 * 13], int i0, int j0, double M_rm[9])
{
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      M_rm[r * 3 + c] = P_rm[(i0 + r) * 13 + (j0 + c)];
    }
  }
}

void congruence_D_sym(const double Sin_rm[9], double Sout_rm[9])
{
  // Sout = D * Sin * D  (D diagonal ±1)
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      Sout_rm[r * 3 + c] =
        D_axis[r] * Sin_rm[r * 3 + c] * D_axis[c];
    }
  }
}

void symmetrize6(double A_rm[36])
{
  for (int r = 0; r < 6; ++r) {
    for (int c = r + 1; c < 6; ++c) {
      const double v = 0.5 * (A_rm[r * 6 + c] + A_rm[c * 6 + r]);
      A_rm[r * 6 + c] = v;
      A_rm[c * 6 + r] = v;
    }
  }
}

}  // namespace

bool euler_rate_to_nav_angvel_jacobian(
  double phi, double theta, double /*psi*/,
  const double Rn2r_cm[9],
  double G_n_row_major[9],
  double cos_theta_eps)
{
  const double ct = std::cos(theta);
  if (std::abs(ct) < cos_theta_eps) {
    return false;
  }
  const double sp = std::sin(phi);
  const double cp = std::cos(phi);
  const double st = std::sin(theta);

  // R_e2b = inv(R_b2e): maps Euler-angle rates → body angular velocity
  // R_b2e from ekf_prediction (ė = R_b2e ω_b):
  // [[1, sp*tanθ, cp*tanθ], [0, cp, -sp], [0, sp/ct, cp/ct]]
  double Re2b_rm[9] = {
    1.0, 0.0, -st,
    0.0, cp, sp * ct,
    0.0, -sp, cp * ct
  };

  double Rr2n_cm[9];
  transpose3_cm(Rn2r_cm, Rr2n_cm);
  double Rr2n_rm[9];
  cm_to_rm(Rr2n_cm, Rr2n_rm);

  // G_n = Rr2n * Re2b : ė → ω_n
  mat3_mul(Rr2n_rm, Re2b_rm, G_n_row_major);
  return all_finite(G_n_row_major, 9);
}

bool public_rotvec_euler_jacobian(
  double phi, double theta, double psi,
  double J_row_major[9],
  double cos_theta_eps)
{
  double Rn2r_cm[9];
  Euler_to_Ra2b_colum_major(phi, theta, psi, Rn2r_cm);
  double G_n[9];
  if (!euler_rate_to_nav_angvel_jacobian(
      phi, theta, psi, Rn2r_cm, G_n, cos_theta_eps))
  {
    return false;
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      J_row_major[r * 3 + c] = D_axis[r] * G_n[r * 3 + c];
    }
  }
  return true;
}

bool dcm_to_quat_wxyz(const double R_cm[9], double q_wxyz[4])
{
  // Shepperd from column-major R (body axes in world = columns)
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
  w /= n; x /= n; y /= n; z /= n;
  if (w < 0.0) {
    w = -w; x = -x; y = -y; z = -z;
  }
  q_wxyz[0] = w;
  q_wxyz[1] = x;
  q_wxyz[2] = y;
  q_wxyz[3] = z;
  return true;
}

bool public_world_from_body_dcm(
  double phi, double theta, double psi, double R_pub_cm[9])
{
  double Rn2r_cm[9];
  Euler_to_Ra2b_colum_major(phi, theta, psi, Rn2r_cm);
  double Rr2n_cm[9];
  transpose3_cm(Rn2r_cm, Rr2n_cm);
  congruence_D(Rr2n_cm, R_pub_cm);
  return all_finite(R_pub_cm, 9);
}

double matrix_asymmetry(const double * A_rm, int n)
{
  double m = 0.0;
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      m = std::max(m, std::abs(A_rm[r * n + c] - A_rm[c * n + r]));
    }
  }
  return m;
}

double min_eig_sym6(const double A_rm[36])
{
  // Jacobi eigenvalue iteration on a copy
  double a[6][6];
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      a[r][c] = 0.5 * (A_rm[r * 6 + c] + A_rm[c * 6 + r]);
    }
  }
  for (int iter = 0; iter < 40; ++iter) {
    int p = 0, q = 1;
    double maxv = 0.0;
    for (int i = 0; i < 6; ++i) {
      for (int j = i + 1; j < 6; ++j) {
        const double v = std::abs(a[i][j]);
        if (v > maxv) {
          maxv = v; p = i; q = j;
        }
      }
    }
    if (maxv < 1e-15) {
      break;
    }
    const double app = a[p][p];
    const double aqq = a[q][q];
    const double apq = a[p][q];
    const double phi = 0.5 * std::atan2(2.0 * apq, aqq - app);
    const double c = std::cos(phi);
    const double s = std::sin(phi);
    for (int k = 0; k < 6; ++k) {
      if (k == p || k == q) {
        continue;
      }
      const double aik = a[k][p];
      const double aiq = a[k][q];
      a[k][p] = a[p][k] = c * aik - s * aiq;
      a[k][q] = a[q][k] = s * aik + c * aiq;
    }
    a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
    a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
    a[p][q] = a[q][p] = 0.0;
  }
  double mine = a[0][0];
  for (int i = 1; i < 6; ++i) {
    mine = std::min(mine, a[i][i]);
  }
  return mine;
}

bool convert_euler_state_to_public_odometry(
  const double x[13],
  const double P_row_major[13 * 13],
  PublicOdometry & out,
  double cos_theta_eps,
  double psd_eig_tol)
{
  out = PublicOdometry{};
  if (!all_finite(x, 13) || !all_finite(P_row_major, 13 * 13)) {
    out.reject_reason = "non-finite state or covariance";
    return false;
  }

  const double phi = x[1];
  const double theta = x[2];
  const double psi = x[3];

  double R_pub_cm[9];
  if (!public_world_from_body_dcm(phi, theta, psi, R_pub_cm)) {
    out.reject_reason = "failed to form public DCM";
    return false;
  }
  if (!dcm_to_quat_wxyz(R_pub_cm, out.quat_wxyz)) {
    out.reject_reason = "quaternion conversion failed";
    return false;
  }

  const double p_int[3] = {x[7], x[8], x[9]};
  const double v_int[3] = {x[10], x[11], x[12]};
  const double w_int[3] = {x[4], x[5], x[6]};
  apply_D(p_int, out.position);
  apply_D(v_int, out.twist_linear);
  apply_D(w_int, out.twist_angular);

  double J_rm[9];
  if (!public_rotvec_euler_jacobian(phi, theta, psi, J_rm, cos_theta_eps)) {
    out.reject_reason = "pitch singularity (cos(theta) too small)";
    return false;
  }

  // Blocks from P
  double Pp_rm[9], Pe_rm[9], Pw_rm[9], Pv_rm[9];
  double Ppe_rm[9], Ppv_rm[9], Ppw_rm[9];
  double Pew_rm[9], Pev_rm[9], Pwv_rm[9];
  extract3x3(P_row_major, 7, 7, Pp_rm);
  extract3x3(P_row_major, 1, 1, Pe_rm);
  extract3x3(P_row_major, 4, 4, Pw_rm);
  extract3x3(P_row_major, 10, 10, Pv_rm);
  extract3x3(P_row_major, 7, 1, Ppe_rm);   // pos × euler
  extract3x3(P_row_major, 7, 10, Ppv_rm);  // pos × v (unused in pose)
  extract3x3(P_row_major, 7, 4, Ppw_rm);   // pos × w (unused in pose)
  extract3x3(P_row_major, 1, 4, Pew_rm);   // euler × w
  extract3x3(P_row_major, 1, 10, Pev_rm);  // euler × v
  extract3x3(P_row_major, 4, 10, Pwv_rm);  // w × v

  (void)Ppv_rm;
  (void)Ppw_rm;

  // Position public: Σp' = D Pp D
  double Pp_pub[9];
  congruence_D_sym(Pp_rm, Pp_pub);

  // Rotation public: Σr = J Pe J^T
  double PeJt[9];
  // PeJt = Pe * J^T
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      PeJt[r * 3 + c] =
        Pe_rm[r * 3 + 0] * J_rm[c * 3 + 0] +
        Pe_rm[r * 3 + 1] * J_rm[c * 3 + 1] +
        Pe_rm[r * 3 + 2] * J_rm[c * 3 + 2];
    }
  }
  double Sr_rm[9];
  mat3_mul(J_rm, PeJt, Sr_rm);

  // Cross Σ_pr = D * Ppe * J^T
  double PpeJt[9];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      PpeJt[r * 3 + c] =
        Ppe_rm[r * 3 + 0] * J_rm[c * 3 + 0] +
        Ppe_rm[r * 3 + 1] * J_rm[c * 3 + 1] +
        Ppe_rm[r * 3 + 2] * J_rm[c * 3 + 2];
    }
  }
  double Spr_rm[9];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      Spr_rm[r * 3 + c] = D_axis[r] * PpeJt[r * 3 + c];
    }
  }

  // Assemble pose 6x6
  std::array<double, 36> pose{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      pose[r * 6 + c] = Pp_pub[r * 3 + c];
      pose[r * 6 + (c + 3)] = Spr_rm[r * 3 + c];
      pose[(r + 3) * 6 + c] = Spr_rm[c * 3 + r];  // Σrp = Σpr^T
      pose[(r + 3) * 6 + (c + 3)] = Sr_rm[r * 3 + c];
    }
  }
  symmetrize6(pose.data());

  // Twist layout [vx vy vz wx wy wz] in public body; Σ' = D Σ D.
  // Pwv_rm is cov(w, v) at indices (4:6, 10:12).
  double Pw_pub[9], Pv_pub[9];
  congruence_D_sym(Pw_rm, Pw_pub);
  congruence_D_sym(Pv_rm, Pv_pub);

  std::array<double, 36> twist{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      twist[r * 6 + c] = Pv_pub[r * 3 + c];
      twist[(r + 3) * 6 + (c + 3)] = Pw_pub[r * 3 + c];
      twist[r * 6 + (c + 3)] =
        D_axis[r] * Pwv_rm[c * 3 + r] * D_axis[c];  // cov(v,w)
      twist[(r + 3) * 6 + c] =
        D_axis[r] * Pwv_rm[r * 3 + c] * D_axis[c];  // cov(w,v)
    }
  }
  symmetrize6(twist.data());

  if (!all_finite(pose.data(), 36) || !all_finite(twist.data(), 36)) {
    out.reject_reason = "non-finite mapped covariance";
    return false;
  }
  if (min_eig_sym6(pose.data()) < psd_eig_tol ||
      min_eig_sym6(twist.data()) < psd_eig_tol)
  {
    out.reject_reason = "mapped covariance not positive semidefinite";
    return false;
  }

  // nav_msgs has no pose–twist cross; Pe×v / Pe×w unused here.
  (void)Pev_rm;
  (void)Pew_rm;

  out.pose_covariance = pose;
  out.twist_covariance = twist;
  out.valid = true;
  return true;
}

}  // namespace odometry
}  // namespace localslam
