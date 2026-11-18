/**
 * E5e-A3a: Euler covariance initialization must assign P = diagmat(p_ini).
 * Deterministic unit test — no Webots, no tuning.
 */

#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <armadillo>

#include "ekf.hpp"
#include "localslam_types.hpp"
#include "parameters.hpp"

namespace
{

parameters make_test_par()
{
  parameters par{};
  par.Robot_state_size = 13;
  par.Attitude_update = true;
  par.Altitude_update = false;
  par.Speed_update = false;
  par.Visual_update = false;
  par.GPS_update = false;
  par.Rangetobase_update = false;
  par.CL_Compute_Cp_from_start = false;
  par.Prediction_model = "noise_driven";
  par.Max_freq_output = 100;
  par.Mono_cam_frame_period = 0.0333;
  par.Data_origin = "test";
  par.Robot_init_x = 0.0;
  par.Robot_init_y = 0.0;
  par.Robot_init_z = 0.0;
  par.Robot_init_roll = 0.0;
  par.Robot_init_pitch = 0.0;
  par.Robot_init_yaw = 0.0;
  par.Mono_cam_cc_u = 320.0;
  par.Mono_cam_cc_v = 240.0;
  par.Mono_cam_fc_u = 500.0;
  par.Mono_cam_fc_v = 500.0;
  par.Mono_cam_alpha_c = 0.0;
  par.Mono_cam_2_robot_pos_x = 0.0;
  par.Mono_cam_2_robot_pos_y = 0.0;
  par.Mono_cam_2_robot_pos_z = 0.0;
  par.Mono_cam_2_robot_axis_x = 0.0;
  par.Mono_cam_2_robot_axis_y = 0.0;
  par.Mono_cam_2_robot_axis_z = 0.0;
  par.Mono_cam_distortions = {0.0, 0.0, 0.0, 0.0, 0.0};
  par.Sigma_att_update = 0.001;
  par.Sigma_w = 0.1;
  par.Sigma_a = 4.0;
  par.Tau_r = 1.0;
  par.Tau_p = 1.0e17;
  par.Stats = false;
  return par;
}

DATA make_att(double roll, double pitch, double yaw, long int time_ms)
{
  DATA dat;
  dat.data_type = "attitude";
  dat.att.roll = static_cast<float>(roll);
  dat.att.pitch = static_cast<float>(pitch);
  dat.att.yaw = static_cast<float>(yaw);
  dat.att.time = time_ms;
  dat.time = time_ms;
  return dat;
}

arma::vec::fixed<13> expected_p_ini()
{
  const double eps = std::numeric_limits<double>::epsilon();
  return {0, .01, .01, .01, .01, .01, .01, eps, eps, eps, .0001, .0001, .0001};
}

}  // namespace

TEST(EulerCovInit, AssignsDiagmatPini)
{
  parameters par = make_test_par();
  EKF ekf(par);
  ASSERT_FALSE(ekf.get_ekf_initialized_state());

  DATA dat = make_att(0.05, -0.02, 1.2, 1000);
  ekf.system_init(dat);
  ASSERT_TRUE(ekf.get_ekf_initialized_state());

  arma::vec::fixed<13> x;
  arma::mat::fixed<13, 13> P;
  ekf.copy_robot_state_and_cov(x, P);

  EXPECT_EQ(P.n_rows, 13u);
  EXPECT_EQ(P.n_cols, 13u);

  const auto p_ini = expected_p_ini();
  for (arma::uword i = 0; i < 13; ++i) {
    EXPECT_NEAR(P(i, i), p_ini(i), 1e-18) << "diag i=" << i;
  }
  // Off-diagonals must be zero (diagonal initialization).
  for (arma::uword r = 0; r < 13; ++r) {
    for (arma::uword c = 0; c < 13; ++c) {
      if (r == c) {
        continue;
      }
      EXPECT_DOUBLE_EQ(P(r, c), 0.0) << "off diag (" << r << "," << c << ")";
    }
  }

  EXPECT_TRUE(P.is_finite());
  EXPECT_TRUE(arma::approx_equal(P, P.t(), "absdiff", 0.0));
  // Diagonal with non-negative entries → PSD.
  arma::vec eig = arma::eig_sym(P);
  EXPECT_GE(eig.min(), -1e-18);

  // Previous defect: P was all zeros. Guard against regression.
  EXPECT_GT(P(1, 1), 0.0);
  EXPECT_GT(P(2, 2), 0.0);
  EXPECT_GT(P(3, 3), 0.0);
}

TEST(EulerCovInit, FirstAttitudeGainNotZeroSuppressed)
{
  parameters par = make_test_par();
  const double sigma = par.Sigma_att_update;
  const double R = sigma * sigma;

  EKF ekf(par);
  DATA dat0 = make_att(0.0, 0.0, 0.0, 0);
  ekf.system_init(dat0);

  arma::vec::fixed<13> x0;
  arma::mat::fixed<13, 13> P0;
  ekf.copy_robot_state_and_cov(x0, P0);

  // Scalar attitude-channel gain with H selecting that state: K = P/(P+R).
  // With corrected P_φ=0.01 and σ=0.001 → K ≈ 0.9999 (not ~1e-10).
  const double P_phi = P0(1, 1);
  ASSERT_GT(P_phi, 1e-6);
  const double K_phi_expected = P_phi / (P_phi + R);
  EXPECT_GT(K_phi_expected, 0.9);
  EXPECT_LT(K_phi_expected, 1.0 + 1e-9);

  // Drive one attitude measurement through the public EKF step.
  // Init state roll/pitch/yaw are 0; measurement has nonzero roll.
  const double z_roll = 0.08;
  DATA z = make_att(z_roll, 0.0, 0.0, 50);
  // Warm the static timers so prediction/update path runs.
  ekf.ekf_step(z);
  // Second call after a tiny wait is awkward in unit tests; call again with
  // later timestamp-bearing attitude — wall clock advances between calls.
  DATA z2 = make_att(z_roll, 0.0, 0.0, 100);
  ekf.ekf_step(z2);

  arma::vec::fixed<13> x1;
  arma::mat::fixed<13, 13> P1;
  ekf.copy_robot_state_and_cov(x1, P1);

  // Posterior roll must move toward the measurement (not remain ~0 from K≈0).
  EXPECT_GT(std::abs(x1(1)), 0.01) << "roll posterior still near zero; gain suppressed?";
  EXPECT_NEAR(x1(1), z_roll, 0.02);
  // Attitude variance should drop after a strong update.
  EXPECT_LT(P1(1, 1), P0(1, 1));
}
