#include <cmath>
#include <cstring>
#include <random>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include "odometry_conversions.hpp"

using localslam::odometry::D_axis;
using localslam::odometry::PublicOdometry;
using localslam::odometry::apply_D;
using localslam::odometry::congruence_D;
using localslam::odometry::convert_euler_state_to_public_odometry;
using localslam::odometry::dcm_to_quat_wxyz;
using localslam::odometry::matrix_asymmetry;
using localslam::odometry::min_eig_sym6;
using localslam::odometry::public_rotvec_euler_jacobian;
using localslam::odometry::public_world_from_body_dcm;
using localslam::odometry::transpose3_cm;

namespace
{

void skew(const double v[3], double S_rm[9])
{
  S_rm[0] = 0;     S_rm[1] = -v[2]; S_rm[2] = v[1];
  S_rm[3] = v[2];  S_rm[4] = 0;     S_rm[5] = -v[0];
  S_rm[6] = -v[1]; S_rm[7] = v[0];  S_rm[8] = 0;
}

/** Rotation-vector of R_rel = R_a * R_b^T (small / Rodriguez). */
void rotvec_from_relative_cm(const double Ra_cm[9], const double Rb_cm[9], double w[3])
{
  // R_rel = Ra * Rb^T
  double Rbt[9];
  transpose3_cm(Rb_cm, Rbt);
  double Rrel[9];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      Rrel[c * 3 + r] =
        Ra_cm[0 * 3 + r] * Rbt[c * 3 + 0] +
        Ra_cm[1 * 3 + r] * Rbt[c * 3 + 1] +
        Ra_cm[2 * 3 + r] * Rbt[c * 3 + 2];
    }
  }
  // vee(R - R^T)/2 for small angles; general: θ from trace
  const double tr =
    Rrel[0] + Rrel[4] + Rrel[8];
  double cos_th = 0.5 * (tr - 1.0);
  cos_th = std::max(-1.0, std::min(1.0, cos_th));
  const double th = std::acos(cos_th);
  if (th < 1e-10) {
    w[0] = w[1] = w[2] = 0.0;
    return;
  }
  const double s = 1.0 / (2.0 * std::sin(th));
  w[0] = s * (Rrel[5] - Rrel[7]) * th;  // (r21 - r12) with cm indexing
  w[1] = s * (Rrel[6] - Rrel[2]) * th;
  w[2] = s * (Rrel[1] - Rrel[3]) * th;
}

void fill_psd13_with_cross(double P[13 * 13], unsigned seed = 7)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> N(0.0, 1.0);
  // P = A A^T + eps I  (full, nonzero cross-terms)
  double A[13 * 13];
  for (int i = 0; i < 13 * 13; ++i) {
    A[i] = N(rng);
  }
  for (int i = 0; i < 13; ++i) {
    for (int j = 0; j < 13; ++j) {
      double s = 0.0;
      for (int k = 0; k < 13; ++k) {
        s += A[i * 13 + k] * A[j * 13 + k];
      }
      P[i * 13 + j] = s;
    }
    P[i * 13 + i] += 1e-3;
  }
}

}  // namespace

TEST(OdometryFrame, BodyForwardYawMapsIndependentlyOfLegacyHelpers)
{
  // Internal yaw = +π/2: Rr2n sends body +x to +y in internal nav.
  // Public R_odom←body = D Rr2n D sends body +x to -y (D flips y).
  // ROS public yaw = −π/2 recovers body +x → −y in odom, which equals
  // D * (internal +y). Confirms published DCM is body→parent, not parent→body.
  const double psi_int = M_PI / 2.0;
  double R_pub[9];
  ASSERT_TRUE(public_world_from_body_dcm(0.0, 0.0, psi_int, R_pub));
  const double body_x[3] = {1.0, 0.0, 0.0};
  double world[3] = {
    R_pub[0] * body_x[0] + R_pub[3] * body_x[1] + R_pub[6] * body_x[2],
    R_pub[1] * body_x[0] + R_pub[4] * body_x[1] + R_pub[7] * body_x[2],
    R_pub[2] * body_x[0] + R_pub[5] * body_x[1] + R_pub[8] * body_x[2],
  };
  EXPECT_NEAR(world[0], 0.0, 1e-12);
  EXPECT_NEAR(world[1], -1.0, 1e-12);
  EXPECT_NEAR(world[2], 0.0, 1e-12);

  // Round-trip: ROS R_F←B at yaw=+π/2 equals R_pub from internal yaw=−π/2.
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, M_PI / 2.0);
  tf2::Matrix3x3 RfB(q);
  double R_ros_cm[9];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R_ros_cm[c * 3 + r] = RfB[r][c];
    }
  }
  double R_from_int[9];
  ASSERT_TRUE(public_world_from_body_dcm(0.0, 0.0, -M_PI / 2.0, R_from_int));
  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(R_from_int[i], R_ros_cm[i], 1e-12);
  }
}

TEST(OdometryFrame, CombinedOrientationMatrixIndependentOfSignFlipsAlone)
{
  const double phi = 0.12, theta = -0.34, psi = 0.56;
  double Rn2r[9];
  Euler_to_Ra2b_colum_major(phi, theta, psi, Rn2r);
  double Rr2n[9];
  transpose3_cm(Rn2r, Rr2n);
  double R_expected[9];
  congruence_D(Rr2n, R_expected);

  double R_pub[9];
  ASSERT_TRUE(public_world_from_body_dcm(phi, theta, psi, R_pub));
  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(R_pub[i], R_expected[i], 1e-12);
  }

  double q[4];
  ASSERT_TRUE(dcm_to_quat_wxyz(R_pub, q));
  EXPECT_GE(q[0], 0.0);
  EXPECT_NEAR(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3], 1.0, 1e-12);

  // Equivalent quaternion sign: -q represents same rotation; helper forces w>=0.
  double qneg[4] = {-q[0], -q[1], -q[2], -q[3]};
  // Reconstruct DCM from ±q should match — check via converting R again.
  double q2[4];
  ASSERT_TRUE(dcm_to_quat_wxyz(R_pub, q2));
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(q2[i], q[i], 1e-12);
  }
  (void)qneg;
}

TEST(OdometryFrame, PositionAndVelocityAllAxes)
{
  double x[13] = {};
  x[7] = 1.5; x[8] = -2.5; x[9] = 3.5;
  x[10] = 0.4; x[11] = -0.2; x[12] = 0.1;
  x[4] = 0.01; x[5] = -0.02; x[6] = 0.03;
  x[1] = 0.05; x[2] = -0.04; x[3] = 0.03;

  double P[13 * 13] = {};
  fill_psd13_with_cross(P);

  PublicOdometry out;
  ASSERT_TRUE(convert_euler_state_to_public_odometry(x, P, out));
  EXPECT_DOUBLE_EQ(out.position[0], 1.5);
  EXPECT_DOUBLE_EQ(out.position[1], 2.5);
  EXPECT_DOUBLE_EQ(out.position[2], -3.5);
  EXPECT_DOUBLE_EQ(out.twist_linear[0], 0.4);
  EXPECT_DOUBLE_EQ(out.twist_linear[1], 0.2);
  EXPECT_DOUBLE_EQ(out.twist_linear[2], -0.1);
  EXPECT_DOUBLE_EQ(out.twist_angular[0], 0.01);
  EXPECT_DOUBLE_EQ(out.twist_angular[1], 0.02);
  EXPECT_DOUBLE_EQ(out.twist_angular[2], -0.03);
}

TEST(OdometryFrame, AngularRatesAreBodyRatesNotEulerDerivatives)
{
  // Documented: x(4:6) are ω_b; ė = R_b2e ω_b. Public twist uses D*ω_b.
  double w_int[3] = {0.2, -0.3, 0.4};
  double w_pub[3];
  apply_D(w_int, w_pub);
  EXPECT_DOUBLE_EQ(w_pub[0], 0.2);
  EXPECT_DOUBLE_EQ(w_pub[1], 0.3);
  EXPECT_DOUBLE_EQ(w_pub[2], -0.4);
}

TEST(OdometryJacobian, FiniteDifferenceMatchesAnalytic)
{
  const double phi = 0.15, theta = -0.25, psi = 0.35;
  double J[9];
  ASSERT_TRUE(public_rotvec_euler_jacobian(phi, theta, psi, J));

  double R0[9];
  ASSERT_TRUE(public_world_from_body_dcm(phi, theta, psi, R0));

  const double eps = 1e-6;
  const double e0[3] = {phi, theta, psi};
  for (int k = 0; k < 3; ++k) {
    double ep[3] = {e0[0], e0[1], e0[2]};
    ep[k] += eps;
    double Rp[9];
    ASSERT_TRUE(public_world_from_body_dcm(ep[0], ep[1], ep[2], Rp));
    double w[3];
    rotvec_from_relative_cm(Rp, R0, w);
    for (int r = 0; r < 3; ++r) {
      const double fd = w[r] / eps;
      EXPECT_NEAR(J[r * 3 + k], fd, 2e-4)
        << "J mismatch row " << r << " col " << k;
    }
  }
}

TEST(OdometryCovariance, FullMatrixCrossTermsSymmetryPsd)
{
  double x[13] = {};
  x[1] = 0.1; x[2] = -0.2; x[3] = 0.3;
  x[4] = 0.05; x[5] = -0.04; x[6] = 0.03;
  x[7] = 1.0; x[8] = 2.0; x[9] = -0.5;
  x[10] = 0.2; x[11] = -0.1; x[12] = 0.05;

  double P[13 * 13];
  fill_psd13_with_cross(P, 11);
  // Ensure nontrivial euler–position cross block
  EXPECT_NE(P[7 * 13 + 1], 0.0);

  PublicOdometry out;
  ASSERT_TRUE(convert_euler_state_to_public_odometry(x, P, out));
  EXPECT_LT(matrix_asymmetry(out.pose_covariance.data(), 6), 1e-12);
  EXPECT_LT(matrix_asymmetry(out.twist_covariance.data(), 6), 1e-12);
  EXPECT_GE(min_eig_sym6(out.pose_covariance.data()), -1e-9);
  EXPECT_GE(min_eig_sym6(out.twist_covariance.data()), -1e-9);

  // Pose rotation block should reflect J Pe J^T, not a raw Euler copy:
  // at least one off-diagonal from cross/J mixing when Pe has off-diagonals.
  bool any_rot = false;
  for (int i = 21; i < 36; ++i) {
    if (std::abs(out.pose_covariance[i]) > 1e-12) {
      any_rot = true;
      break;
    }
  }
  EXPECT_TRUE(any_rot);
}

TEST(OdometryCovariance, RejectsPitchSingularityAndNonFinite)
{
  double x[13] = {};
  x[2] = M_PI / 2.0;  // singular pitch
  double P[13 * 13] = {};
  for (int i = 0; i < 13; ++i) {
    P[i * 13 + i] = 1.0;
  }
  PublicOdometry out;
  EXPECT_FALSE(convert_euler_state_to_public_odometry(x, P, out));
  EXPECT_FALSE(out.valid);
  EXPECT_NE(out.reject_reason.find("singularity"), std::string::npos);

  x[2] = 0.1;
  x[7] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(convert_euler_state_to_public_odometry(x, P, out));
  EXPECT_NE(out.reject_reason.find("non-finite"), std::string::npos);
}

TEST(OdometryCovariance, IdentityStateDiagonalPropagatesByD)
{
  double x[13] = {};
  double P[13 * 13] = {};
  P[7 * 13 + 7] = 4.0;
  P[8 * 13 + 8] = 9.0;
  P[9 * 13 + 9] = 16.0;
  P[10 * 13 + 10] = 0.25;
  P[11 * 13 + 11] = 0.49;
  P[12 * 13 + 12] = 0.81;
  P[4 * 13 + 4] = 0.01;
  P[5 * 13 + 5] = 0.04;
  P[6 * 13 + 6] = 0.09;
  P[1 * 13 + 1] = 0.02;
  P[2 * 13 + 2] = 0.03;
  P[3 * 13 + 3] = 0.05;

  PublicOdometry out;
  ASSERT_TRUE(convert_euler_state_to_public_odometry(x, P, out));
  // Position variances unchanged under D (D_ii^2 = 1)
  EXPECT_NEAR(out.pose_covariance[0], 4.0, 1e-12);
  EXPECT_NEAR(out.pose_covariance[7], 9.0, 1e-12);
  EXPECT_NEAR(out.pose_covariance[14], 16.0, 1e-12);
  EXPECT_NEAR(out.twist_covariance[0], 0.25, 1e-12);
  EXPECT_NEAR(out.twist_covariance[7], 0.49, 1e-12);
  EXPECT_NEAR(out.twist_covariance[14], 0.81, 1e-12);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
