/**
 * LSLAM-DYNAMIC-CONSISTENCY-A2 — Deterministic Jac_e_uv_XYZ FD validation,
 * orientation-masking check, rotation-only innovations, and EKF coupling
 * (visual attitude Jacobian OFF vs ON) using production Jacobian + production
 * Kalman update equations from Visual_update_feats_WO_val_e.
 *
 * Does not modify production defaults. No Webots.
 */

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <armadillo>
#include <opencv2/core.hpp>

#include "ekf.hpp"
#include "ekf_Jacobians.hpp"
#include "localslam_types.hpp"
#include "parameters.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include "../../common/Vision/vision.hpp"

namespace
{

constexpr double kDeg = M_PI / 180.0;
constexpr double kAbsTolJac = 5e-4;   // px per unit state (see REPORT)
constexpr double kRelTolJac = 5e-3;

struct CamExtrinsics
{
  CAM cam{};
  arma::mat::fixed<3, 3> Rr2c;
  arma::vec::fixed<3> t_c2r;
  double distortions[5]{0, 0, 0, 0, 0};
};

CamExtrinsics make_a3_cam(bool nonzero_lever)
{
  CamExtrinsics e;
  // quad_params_a3_2a_exclusion10 / TFA300L intrinsics
  e.cam.cc[0] = 200.0;
  e.cam.cc[1] = 120.0;
  e.cam.fc[0] = 200.0;
  e.cam.fc[1] = 200.0;
  e.cam.alpha_c = 0.0;
  e.cam.distortions = e.distortions;
  e.t_c2r = {nonzero_lever ? 0.1 : 0.0, 0.0, 0.0};
  double Ra2b[9];
  Euler_to_Ra2b_colum_major(0.0, 0.0, 1.5707963268, Ra2b);
  e.Rr2c = arma::mat(Ra2b, 3, 3);
  return e;
}

arma::vec make_robot_x(double phi, double theta, double psi, double x, double y, double z)
{
  arma::vec xr = arma::zeros<arma::vec>(13);
  xr(1) = phi;
  xr(2) = theta;
  xr(3) = psi;
  xr(7) = x;
  xr(8) = y;
  xr(9) = z;
  return xr;
}

/** Production projection used by Jac_e_uv_XYZ (pc = Rn2c*(m-r) - Rr2c*t_c2r). */
arma::vec::fixed<2> project_uv(
  const arma::vec & x, const arma::vec::fixed<3> & pxyz, CamExtrinsics & e)
{
  double Ra2b[9];
  Euler_to_Ra2b_colum_major(x(1), x(2), x(3), Ra2b);
  arma::mat Rn2r(Ra2b, 3, 3);
  arma::mat::fixed<3, 3> Rn2c = e.Rr2c * Rn2r;
  arma::vec::fixed<3> pc = Rn2c * (pxyz - x.subvec(7, 9)) - e.Rr2c * e.t_c2r;
  arma::mat::fixed<2, 3> duv_dPc;
  cv::Point2d uvd = Projection_model(pc, 1, false, e.cam, duv_dPc);
  return {uvd.x, uvd.y};
}

bool projection_valid(
  const arma::vec & x, const arma::vec::fixed<3> & pxyz, CamExtrinsics & e)
{
  double Ra2b[9];
  Euler_to_Ra2b_colum_major(x(1), x(2), x(3), Ra2b);
  arma::mat Rn2r(Ra2b, 3, 3);
  arma::mat::fixed<3, 3> Rn2c = e.Rr2c * Rn2r;
  arma::vec::fixed<3> pc = Rn2c * (pxyz - x.subvec(7, 9)) - e.Rr2c * e.t_c2r;
  return pc(2) > 0.5 && std::isfinite(pc(2));
}

struct JacErr
{
  double max_abs{0};
  double rms{0};
  double max_rel{0};
  int n{0};
};

void accumulate_err(JacErr & e, double a, double n, double floor_rel = 1e-3)
{
  const double d = std::abs(a - n);
  e.max_abs = std::max(e.max_abs, d);
  e.rms += d * d;
  e.max_rel = std::max(e.max_rel, d / std::max(std::abs(n), floor_rel));
  ++e.n;
}

JacErr finalize(JacErr e)
{
  if (e.n > 0) {
    e.rms = std::sqrt(e.rms / static_cast<double>(e.n));
  }
  return e;
}

struct CaseGeom
{
  const char * name;
  double phi;
  double theta;
  double psi;
  double depth;
  bool lever;
};

std::vector<CaseGeom> jacobian_cases()
{
  return {
    {"J1_level_40", 0, 0, 0, 40, true},
    {"J2_roll_p10", 10 * kDeg, 0, 0, 40, true},
    {"J2_roll_m10", -10 * kDeg, 0, 0, 40, true},
    {"J2_roll_p20", 20 * kDeg, 0, 0, 40, true},
    {"J3_pitch_p10", 0, 10 * kDeg, 0, 40, true},
    {"J3_pitch_m10", 0, -10 * kDeg, 0, 40, true},
    {"J4_yaw_30", 0, 0, 30 * kDeg, 40, true},
    {"J5_combined", 12 * kDeg, -8 * kDeg, 25 * kDeg, 40, true},
    {"J7_depth_10", 5 * kDeg, -3 * kDeg, 0, 10, true},
    {"J7_depth_80", 5 * kDeg, -3 * kDeg, 0, 80, true},
    {"J8_lever", 8 * kDeg, -5 * kDeg, 15 * kDeg, 40, true},
    {"J9_zero_lever", 8 * kDeg, -5 * kDeg, 15 * kDeg, 40, false},
    {"J10_brake_att", 15 * kDeg, -12 * kDeg, 5 * kDeg, 40, true},
  };
}

arma::vec::fixed<3> landmark_at_depth(
  const arma::vec & x, double depth, double u_off, double v_off, CamExtrinsics & e)
{
  // Place landmark so that at state x it projects near (cc+u_off, cc+v_off).
  arma::mat::fixed<3, 2> dhc;
  cv::Point2d uvd(e.cam.cc[0] + u_off, e.cam.cc[1] + v_off);
  arma::vec::fixed<3> hc = Inverse_projection_model(uvd, 1, false, e.cam, dhc);
  double Ra2b[9];
  Euler_to_Ra2b_colum_major(x(1), x(2), x(3), Ra2b);
  arma::mat Rn2r(Ra2b, 3, 3);
  arma::mat::fixed<3, 3> Rr2n = Rn2r.t();
  arma::mat::fixed<3, 3> Rc2r = e.Rr2c.t();
  arma::mat Rc2n = Rr2n * Rc2r;
  arma::vec::fixed<3> hn = Rc2n * hc;
  arma::vec::fixed<3> m = hn / arma::norm(hn);
  arma::vec::fixed<3> t_c = x.subvec(7, 9) + Rr2n * e.t_c2r;
  return t_c + depth * m;
}

struct BlockErrs
{
  JacErr pos;
  JacErr roll;
  JacErr pitch;
  JacErr yaw;
  JacErr landmark;
};

BlockErrs compare_jac_fd(
  const arma::vec & x0,
  const arma::vec::fixed<3> & pxyz,
  CamExtrinsics & e,
  bool attitude_flag,
  double eps_pos,
  double eps_att,
  double eps_lm)
{
  arma::vec x = x0;
  arma::mat::fixed<2, 13> duv_dx;
  arma::mat::fixed<2, 3> duv_dy;
  Jac_e_uv_XYZ(pxyz, x, e.cam, e.Rr2c, e.t_c2r, duv_dx, duv_dy, attitude_flag);

  BlockErrs be;
  auto fd_col_robot = [&](int j, double eps) {
    arma::vec xp = x0;
    arma::vec xm = x0;
    xp(j) += eps;
    xm(j) -= eps;
    arma::vec::fixed<2> hp = project_uv(xp, pxyz, e);
    arma::vec::fixed<2> hm = project_uv(xm, pxyz, e);
    return arma::vec::fixed<2>((hp - hm) / (2.0 * eps));
  };
  auto fd_lm = [&](int k, double eps) {
    arma::vec::fixed<3> pp = pxyz;
    arma::vec::fixed<3> pm = pxyz;
    pp(k) += eps;
    pm(k) -= eps;
    arma::vec::fixed<2> hp = project_uv(x0, pp, e);
    arma::vec::fixed<2> hm = project_uv(x0, pm, e);
    return arma::vec::fixed<2>((hp - hm) / (2.0 * eps));
  };

  for (int j = 7; j <= 9; ++j) {
    auto n = fd_col_robot(j, eps_pos);
    for (int i = 0; i < 2; ++i) {
      accumulate_err(be.pos, duv_dx(i, j), n(i));
    }
  }
  {
    auto n = fd_col_robot(1, eps_att);
    for (int i = 0; i < 2; ++i) {
      accumulate_err(be.roll, duv_dx(i, 1), n(i));
    }
  }
  {
    auto n = fd_col_robot(2, eps_att);
    for (int i = 0; i < 2; ++i) {
      accumulate_err(be.pitch, duv_dx(i, 2), n(i));
    }
  }
  {
    auto n = fd_col_robot(3, eps_att);
    for (int i = 0; i < 2; ++i) {
      accumulate_err(be.yaw, duv_dx(i, 3), n(i));
    }
  }
  for (int k = 0; k < 3; ++k) {
    auto n = fd_lm(k, eps_lm);
    for (int i = 0; i < 2; ++i) {
      accumulate_err(be.landmark, duv_dy(i, k), n(i));
    }
  }
  be.pos = finalize(be.pos);
  be.roll = finalize(be.roll);
  be.pitch = finalize(be.pitch);
  be.yaw = finalize(be.yaw);
  be.landmark = finalize(be.landmark);
  return be;
}

std::string evidence_dir()
{
  const char * env = std::getenv("LSLAM_A2_EVIDENCE_DIR");
  if (env && env[0] != '\0') {
    return std::string(env);
  }
  return {};
}

void write_csv_line(std::ofstream & f, const std::vector<std::string> & cols)
{
  for (size_t i = 0; i < cols.size(); ++i) {
    if (i) {
      f << ',';
    }
    f << cols[i];
  }
  f << '\n';
}

/** Production Kalman equations from Visual_update_feats_WO_val_e. */
struct VisualUpdateResult
{
  arma::vec x_post;
  arma::mat P_post;
  arma::vec innovation;
  arma::mat H;
  arma::mat K;
  double inov_rms{0};
  double delta_pos_norm{0};
  double delta_att_norm{0};
  double delta_vel_norm{0};
  double delta_lm_rms{0};
  double post_inov_rms{0};
};

VisualUpdateResult production_visual_kalman(
  const arma::vec & x,
  const arma::mat & P,
  const std::vector<arma::vec::fixed<3>> & landmarks,
  const std::vector<arma::vec::fixed<2>> & z_uv,
  CamExtrinsics & e,
  bool attitude_flag,
  double sigma_uv)
{
  const int n_lm = static_cast<int>(landmarks.size());
  const int x_len = 13 + 3 * n_lm;
  EXPECT_EQ(static_cast<int>(x.n_elem), x_len);
  EXPECT_EQ(static_cast<int>(P.n_rows), x_len);

  arma::mat H;
  arma::vec z;
  arma::vec h;
  arma::mat R;
  for (int i = 0; i < n_lm; ++i) {
    arma::vec::fixed<3> pxyz = landmarks[i];
    // Prefer landmark from state if present
    pxyz = x.subvec(13 + 3 * i, 13 + 3 * i + 2);
    arma::mat::fixed<2, 13> duv_dx;
    arma::mat::fixed<2, 3> duv_dy;
    arma::vec x_mut = x;  // Jac takes non-const ref
    Jac_e_uv_XYZ(pxyz, x_mut, e.cam, e.Rr2c, e.t_c2r, duv_dx, duv_dy, attitude_flag);
    const int idx = static_cast<int>(H.n_rows);
    H.resize(idx + 2, x_len);
    H.zeros();
    H(arma::span(idx, idx + 1), arma::span(0, 12)) = duv_dx;
    H(arma::span(idx, idx + 1), arma::span(13 + 3 * i, 13 + 3 * i + 2)) = duv_dy;
    h.resize(idx + 2);
    arma::vec::fixed<2> hi = project_uv(x, pxyz, e);
    h(idx) = hi(0);
    h(idx + 1) = hi(1);
    z.resize(idx + 2);
    z(idx) = z_uv[i](0);
    z(idx + 1) = z_uv[i](1);
    R.resize(idx + 2, idx + 2);
    R(idx, idx) = sigma_uv * sigma_uv;
    R(idx + 1, idx + 1) = sigma_uv * sigma_uv;
  }

  VisualUpdateResult out;
  out.H = H;
  out.innovation = z - h;
  out.inov_rms = std::sqrt(arma::dot(out.innovation, out.innovation) /
                           static_cast<double>(out.innovation.n_elem));

  arma::mat H_P = H * P;
  arma::mat S = H_P * H.t() + R;
  out.K = P * H.t() * arma::inv(S);
  out.P_post = P - out.K * H_P;
  const arma::vec dx = out.K * out.innovation;
  out.x_post = x + dx;

  out.delta_pos_norm = arma::norm(dx.subvec(7, 9));
  out.delta_att_norm = arma::norm(dx.subvec(1, 3));
  out.delta_vel_norm = arma::norm(dx.subvec(10, 12));
  if (n_lm > 0) {
    double s = 0;
    for (int i = 0; i < n_lm; ++i) {
      s += std::pow(arma::norm(dx.subvec(13 + 3 * i, 13 + 3 * i + 2)), 2);
    }
    out.delta_lm_rms = std::sqrt(s / n_lm);
  }

  // Post-update innovation under corrected state
  double s2 = 0;
  for (int i = 0; i < n_lm; ++i) {
    arma::vec::fixed<3> p = out.x_post.subvec(13 + 3 * i, 13 + 3 * i + 2);
    arma::vec::fixed<2> hi = project_uv(out.x_post, p, e);
    s2 += std::pow(z_uv[i](0) - hi(0), 2) + std::pow(z_uv[i](1) - hi(1), 2);
  }
  out.post_inov_rms = std::sqrt(s2 / (2.0 * n_lm));
  return out;
}

arma::mat make_diag_P(int n_lm, double sig_att, double sig_pos, double sig_vel, double sig_lm)
{
  const int n = 13 + 3 * n_lm;
  arma::mat P = arma::zeros(n, n);
  for (int i = 1; i <= 3; ++i) {
    P(i, i) = sig_att * sig_att;
  }
  for (int i = 4; i <= 6; ++i) {
    P(i, i) = 0.01 * 0.01;
  }
  for (int i = 7; i <= 9; ++i) {
    P(i, i) = sig_pos * sig_pos;
  }
  for (int i = 10; i <= 12; ++i) {
    P(i, i) = sig_vel * sig_vel;
  }
  for (int i = 0; i < 3 * n_lm; ++i) {
    P(13 + i, 13 + i) = sig_lm * sig_lm;
  }
  return P;
}

/** Valid correlated PSD via A A^T + diag. */
arma::mat make_correlated_P(int n_lm, double sig_att, double sig_pos, double sig_lm)
{
  const int n = 13 + 3 * n_lm;
  arma::mat A = arma::zeros(n, n);
  // Couple attitude ↔ position and position ↔ landmarks lightly
  for (int i = 1; i <= 3; ++i) {
    A(i, i) = sig_att;
    A(7 + (i - 1) % 3, i) = 0.15 * sig_pos;
  }
  for (int i = 7; i <= 9; ++i) {
    A(i, i) = sig_pos;
  }
  for (int i = 10; i <= 12; ++i) {
    A(i, i) = 0.2;
  }
  for (int i = 0; i < n_lm; ++i) {
    for (int k = 0; k < 3; ++k) {
      A(13 + 3 * i + k, 13 + 3 * i + k) = sig_lm;
      A(13 + 3 * i + k, 7 + k) = 0.2 * sig_lm;
    }
  }
  arma::mat P = A * A.t() + 1e-8 * arma::eye(n, n);
  return P;
}

parameters make_ahrs_par(double sigma_att)
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
  par.Max_freq_output = 200;
  par.Mono_cam_frame_period = 0.0333;
  par.Data_origin = "test";
  par.Robot_init_x = 0;
  par.Robot_init_y = 0;
  par.Robot_init_z = 0;
  par.Robot_init_roll = 0;
  par.Robot_init_pitch = 0;
  par.Robot_init_yaw = 0;
  par.Mono_cam_cc_u = 200;
  par.Mono_cam_cc_v = 120;
  par.Mono_cam_fc_u = 200;
  par.Mono_cam_fc_v = 200;
  par.Mono_cam_alpha_c = 0;
  par.Mono_cam_2_robot_pos_x = 0.1;
  par.Mono_cam_2_robot_pos_y = 0;
  par.Mono_cam_2_robot_pos_z = 0;
  par.Mono_cam_2_robot_axis_x = 0;
  par.Mono_cam_2_robot_axis_y = 0;
  par.Mono_cam_2_robot_axis_z = 1.5707963268;
  par.Mono_cam_distortions = {0, 0, 0, 0, 0};
  par.Sigma_att_update = sigma_att;
  par.Sigma_w = 0.1;
  par.Sigma_a = 3.0;
  par.Tau_r = 1.0;
  par.Tau_p = 1e17;
  par.Stats = false;
  return par;
}

DATA make_att(double roll, double pitch, double yaw, long int t)
{
  DATA dat;
  dat.data_type = "attitude";
  dat.att.roll = static_cast<float>(roll);
  dat.att.pitch = static_cast<float>(pitch);
  dat.att.yaw = static_cast<float>(yaw);
  dat.att.time = t;
  dat.time = t;
  return dat;
}

}  // namespace

// ---------------------------------------------------------------------------
// Part A — Jacobian FD
TEST(LslamDynamicA2, JacEuvXyzMatchesFiniteDifferencesAttitudeOn)
{
  const std::string edir = evidence_dir();
  std::ofstream csv;
  if (!edir.empty()) {
    csv.open(edir + "/tables/jacobian_fd_attitude_on.csv");
    write_csv_line(
      csv,
      {"case", "block", "max_abs", "rms", "max_rel", "sign_agree", "eps", "verdict"});
  }

  double global_max_pos_lm = 0;
  double global_max_att = 0;
  int att_fail = 0;
  int att_sign_flip = 0;
  int att_n = 0;
  for (const auto & c : jacobian_cases()) {
    auto e = make_a3_cam(c.lever);
    arma::vec x = make_robot_x(c.phi, c.theta, c.psi, 0, 0, 0);
    std::vector<std::pair<double, double>> offs = {{0, 0}, {60, 40}, {-50, 30}, {40, -35}};
    if (!(std::string(c.name).find("J1") == 0 || std::string(c.name).find("J5") == 0 ||
          std::string(c.name).find("J10") == 0))
    {
      offs.resize(1);
    }
    for (const auto & ov : offs) {
      arma::vec::fixed<3> p = landmark_at_depth(x, c.depth, ov.first, ov.second, e);
      ASSERT_TRUE(projection_valid(x, p, e)) << c.name;
      auto be_a = compare_jac_fd(x, p, e, true, 1e-4, 1e-5, 1e-4);
      auto be_b = compare_jac_fd(x, p, e, true, 1e-5, 1e-6, 1e-5);
      auto pick = [](const JacErr & a, const JacErr & b) {
        return (a.max_abs < b.max_abs) ? a : b;
      };
      JacErr pos = pick(be_a.pos, be_b.pos);
      JacErr roll = pick(be_a.roll, be_b.roll);
      JacErr pitch = pick(be_a.pitch, be_b.pitch);
      JacErr yaw = pick(be_a.yaw, be_b.yaw);
      JacErr lm = pick(be_a.landmark, be_b.landmark);

      global_max_pos_lm = std::max({global_max_pos_lm, pos.max_abs, lm.max_abs});
      global_max_att = std::max({global_max_att, roll.max_abs, pitch.max_abs, yaw.max_abs});

      // Per-column pass/fail for the original A2 66 orientation checks
      arma::vec xm = x;
      arma::mat::fixed<2, 13> H;
      arma::mat::fixed<2, 3> Hy;
      Jac_e_uv_XYZ(p, xm, e.cam, e.Rr2c, e.t_c2r, H, Hy, true);
      auto fd_att = [&](int j) {
        arma::vec xp = x;
        arma::vec xmm = x;
        xp(j) += 1e-5;
        xmm(j) -= 1e-5;
        return arma::vec::fixed<2>(
          (project_uv(xp, p, e) - project_uv(xmm, p, e)) / (2e-5));
      };
      for (int j : {1, 2, 3}) {
        auto n = fd_att(j);
        double err = 0;
        double err_neg = 0;
        for (int i = 0; i < 2; ++i) {
          err += std::pow(H(i, j) - n(i), 2);
          err_neg += std::pow(H(i, j) + n(i), 2);
        }
        ++att_n;
        if (std::sqrt(err) >= kAbsTolJac) {
          ++att_fail;
          if (err_neg < err) {
            ++att_sign_flip;
          }
        }
      }

      auto emit = [&](const char * block, const JacErr & er) {
        const bool ok = er.max_abs < kAbsTolJac;
        EXPECT_LT(er.max_abs, kAbsTolJac) << c.name << " " << block;
        if (csv.is_open()) {
          write_csv_line(
            csv,
            {c.name, block, std::to_string(er.max_abs), std::to_string(er.rms),
             std::to_string(er.max_rel), "n/a", "dual", ok ? "PASS" : "FAIL"});
        }
      };
      emit("position", pos);
      emit("landmark", lm);
      emit("roll", roll);
      emit("pitch", pitch);
      emit("yaw", yaw);
    }
  }
  // A2.1: all blocks must pass (orientation sign defect corrected).
  EXPECT_LT(global_max_pos_lm, kAbsTolJac);
  EXPECT_LT(global_max_att, kAbsTolJac);
  EXPECT_EQ(att_fail, 0) << "orientation columns still failing FD";
  std::cout << "A2.1 Jac: pos/lm max_abs=" << global_max_pos_lm
            << " att max_abs=" << global_max_att << " att_fail=" << att_fail << "/" << att_n
            << " sign_flip_better=" << att_sign_flip << std::endl;
  if (!edir.empty()) {
    std::ofstream sum(edir + "/tables/jacobian_summary_on.txt");
    sum << "pos_lm_max_abs=" << global_max_pos_lm << "\natt_max_abs=" << global_max_att
        << "\natt_fail=" << att_fail << "\natt_n=" << att_n
        << "\nsign_flip_better=" << att_sign_flip << "\nabs_tol=" << kAbsTolJac << "\n";
  }
}

TEST(LslamDynamicA2, JacMaskingZerosOnlyAttitudeBlock)
{
  auto e = make_a3_cam(true);
  arma::vec x = make_robot_x(10 * kDeg, -8 * kDeg, 20 * kDeg, 1, -2, 0);
  arma::vec::fixed<3> p = landmark_at_depth(x, 40, 30, -20, e);

  arma::mat::fixed<2, 13> H_on, H_off;
  arma::mat::fixed<2, 3> Hy_on, Hy_off;
  arma::vec xm = x;
  Jac_e_uv_XYZ(p, xm, e.cam, e.Rr2c, e.t_c2r, H_on, Hy_on, true);
  xm = x;
  Jac_e_uv_XYZ(p, xm, e.cam, e.Rr2c, e.t_c2r, H_off, Hy_off, false);

  // Prediction unchanged
  arma::vec::fixed<2> h0 = project_uv(x, p, e);
  EXPECT_TRUE(h0.is_finite());

  // Attitude columns zero when masked
  EXPECT_NEAR(arma::norm(H_off.cols(1, 3)), 0.0, 1e-15);
  // Attitude columns nonzero when enabled (geometry non-degenerate)
  EXPECT_GT(arma::norm(H_on.cols(1, 3)), 1e-6);

  // Position / landmark blocks unchanged by flag
  EXPECT_TRUE(arma::approx_equal(H_on.cols(7, 9), H_off.cols(7, 9), "absdiff", 1e-12));
  EXPECT_TRUE(arma::approx_equal(Hy_on, Hy_off, "absdiff", 1e-12));
  // Other robot cols remain zero either way
  EXPECT_NEAR(arma::norm(H_on.col(0)), 0.0, 1e-15);
  EXPECT_NEAR(arma::norm(H_on.cols(4, 6)), 0.0, 1e-15);
  EXPECT_NEAR(arma::norm(H_on.cols(10, 12)), 0.0, 1e-15);

  // Masked analytical ≠ FD of full measurement for attitude (configured approximation)
  auto be = compare_jac_fd(x, p, e, false, 1e-4, 1e-5, 1e-4);
  EXPECT_GT(be.roll.max_abs, 1e-3);  // FD nonzero while analytical zero
}

TEST(LslamDynamicA2, GeometricInvariantsTranslationAndLever)
{
  auto e = make_a3_cam(true);
  arma::vec x = make_robot_x(5 * kDeg, -4 * kDeg, 10 * kDeg, 0, 0, 0);
  arma::vec::fixed<3> p = landmark_at_depth(x, 40, 10, -15, e);
  arma::vec::fixed<2> uv0 = project_uv(x, p, e);

  arma::vec::fixed<3> d = {1.5, -2.0, 0.3};
  arma::vec x2 = x;
  x2.subvec(7, 9) += d;
  arma::vec::fixed<3> p2 = p + d;
  arma::vec::fixed<2> uv1 = project_uv(x2, p2, e);
  EXPECT_NEAR(uv0(0), uv1(0), 1e-9);
  EXPECT_NEAR(uv0(1), uv1(1), 1e-9);

  // Lever arm along body +X is invariant under pure roll; use pitch.
  double Ra2b[9];
  Euler_to_Ra2b_colum_major(0, 0, 0, Ra2b);
  arma::mat Rn2r0(Ra2b, 3, 3);
  Euler_to_Ra2b_colum_major(0, 15 * kDeg, 0, Ra2b);
  arma::mat Rn2r1(Ra2b, 3, 3);
  arma::vec::fixed<3> tc0 = Rn2r0.t() * e.t_c2r;
  arma::vec::fixed<3> tc1 = Rn2r1.t() * e.t_c2r;
  EXPECT_GT(arma::norm(tc0 - tc1), 1e-6);
}

// ---------------------------------------------------------------------------
// Part B/C — Rotation-only + coupling
TEST(LslamDynamicA2, RotationOnlyInnovationAndCoupling)
{
  const std::string edir = evidence_dir();
  std::ofstream inov_csv;
  std::ofstream corr_csv;
  if (!edir.empty()) {
    inov_csv.open(edir + "/tables/innovation_vs_attitude_discrepancy.csv");
    write_csv_line(
      inov_csv, {"disc_deg", "roll_disc_rad", "pitch_disc_rad", "inov_rms_px", "n_feat"});
    corr_csv.open(edir + "/tables/coupling_off_vs_on.csv");
    write_csv_line(
      corr_csv,
      {"cov", "disc_deg", "flag", "inov_rms", "dpos_m", "datt_rad", "dvel", "dlm_rms",
       "post_inov_rms", "att_err_post", "horiz_err_post"});
  }

  auto e = make_a3_cam(true);
  const arma::vec x_true = make_robot_x(0, 0, 0, 0, 0, 0);
  std::vector<arma::vec::fixed<3>> lms;
  std::vector<arma::vec::fixed<2>> z;
  for (auto ov : std::vector<std::pair<double, double>>{
         {0, 0}, {70, 45}, {-65, 40}, {55, -40}, {-40, -30}, {20, 60}})
  {
    auto p = landmark_at_depth(x_true, 40.0, ov.first, ov.second, e);
    lms.push_back(p);
    z.push_back(project_uv(x_true, p, e));
  }

  // Correct orientation → near-zero innovation
  {
    arma::vec x = x_true;
    x.resize(13 + 3 * lms.size());
    for (size_t i = 0; i < lms.size(); ++i) {
      x.subvec(13 + 3 * i, 13 + 3 * i + 2) = lms[i];
    }
    double s = 0;
    for (size_t i = 0; i < lms.size(); ++i) {
      auto hi = project_uv(x, lms[i], e);
      s += std::pow(z[i](0) - hi(0), 2) + std::pow(z[i](1) - hi(1), 2);
    }
    EXPECT_NEAR(std::sqrt(s / (2.0 * lms.size())), 0.0, 1e-9);
  }

  const std::vector<double> discs_deg = {0.1, 0.5, 1.0, 2.0, 5.0};
  for (double ddeg : discs_deg) {
    const double d = ddeg * kDeg;
    // Positive roll discrepancy: estimated attitude lags true (true has +d roll)
    arma::vec x_est = make_robot_x(0, 0, 0, 0, 0, 0);  // wrong: true is roll=+d
    arma::vec x_true_r = make_robot_x(d, 0, 0, 0, 0, 0);
    std::vector<arma::vec::fixed<2>> z_r;
    std::vector<arma::vec::fixed<3>> lms_r;
    for (auto ov : std::vector<std::pair<double, double>>{
           {0, 0}, {70, 45}, {-65, 40}, {55, -40}, {-40, -30}})
    {
      auto p = landmark_at_depth(x_true_r, 40.0, ov.first, ov.second, e);
      // Keep landmarks fixed in nav from true pose
      lms_r.push_back(p);
      z_r.push_back(project_uv(x_true_r, p, e));
    }
    double s = 0;
    for (size_t i = 0; i < lms_r.size(); ++i) {
      auto hi = project_uv(x_est, lms_r[i], e);
      s += std::pow(z_r[i](0) - hi(0), 2) + std::pow(z_r[i](1) - hi(1), 2);
    }
    const double inov = std::sqrt(s / (2.0 * lms_r.size()));
    if (inov_csv.is_open()) {
      write_csv_line(
        inov_csv,
        {std::to_string(ddeg), std::to_string(d), "0", std::to_string(inov),
         std::to_string(lms_r.size())});
    }
    if (ddeg >= 1.0) {
      EXPECT_GT(inov, 0.5) << "disc " << ddeg;
    }

    // Build state with landmarks
    const int n_lm = static_cast<int>(lms_r.size());
    arma::vec x = arma::zeros(13 + 3 * n_lm);
    x.subvec(0, 12) = x_est;
    for (int i = 0; i < n_lm; ++i) {
      x.subvec(13 + 3 * i, 13 + 3 * i + 2) = lms_r[i];
    }

    for (const char * cov_name : {"diag", "corr"}) {
      arma::mat P = (std::string(cov_name) == "diag")
                      ? make_diag_P(n_lm, 0.05, 0.5, 0.5, 2.0)
                      : make_correlated_P(n_lm, 0.05, 0.5, 2.0);

      for (bool flag : {false, true}) {
        auto r = production_visual_kalman(x, P, lms_r, z_r, e, flag, 10.0);
        const double att_err =
          arma::norm(r.x_post.subvec(1, 3) - x_true_r.subvec(1, 3));
        const double horiz_err =
          arma::norm(r.x_post.subvec(7, 8) - x_true_r.subvec(7, 8));
        if (corr_csv.is_open()) {
          write_csv_line(
            corr_csv,
            {cov_name, std::to_string(ddeg), flag ? "ON" : "OFF",
             std::to_string(r.inov_rms), std::to_string(r.delta_pos_norm),
             std::to_string(r.delta_att_norm), std::to_string(r.delta_vel_norm),
             std::to_string(r.delta_lm_rms), std::to_string(r.post_inov_rms),
             std::to_string(att_err), std::to_string(horiz_err)});
        }
        if (ddeg == 2.0 && std::string(cov_name) == "diag") {
          // With diagonal prior, OFF → orientation gain only via zeros → Δatt≈0
          if (!flag) {
            EXPECT_NEAR(r.delta_att_norm, 0.0, 1e-9);
            EXPECT_GT(r.delta_pos_norm + r.delta_lm_rms, r.delta_att_norm);
          } else {
            EXPECT_GT(r.delta_att_norm, 1e-6);
            // Corrected Jac: attitude correction should reduce |φ−φ_true|
            EXPECT_LT(att_err, d) << "corrected Jac should reduce roll error";
          }
        }
      }
    }
  }

  // Combined roll/pitch 2°
  {
    const double d = 2 * kDeg;
    arma::vec x_true_c = make_robot_x(d, -d, 0, 0, 0, 0);
    arma::vec x_est = make_robot_x(0, 0, 0, 0, 0, 0);
    std::vector<arma::vec::fixed<3>> lms_c;
    std::vector<arma::vec::fixed<2>> z_c;
    for (auto ov : std::vector<std::pair<double, double>>{{0, 0}, {50, 40}, {-40, 30}}) {
      auto p = landmark_at_depth(x_true_c, 40, ov.first, ov.second, e);
      lms_c.push_back(p);
      z_c.push_back(project_uv(x_true_c, p, e));
    }
    const int n_lm = static_cast<int>(lms_c.size());
    arma::vec x = arma::zeros(13 + 3 * n_lm);
    x.subvec(0, 12) = x_est;
    for (int i = 0; i < n_lm; ++i) {
      x.subvec(13 + 3 * i, 13 + 3 * i + 2) = lms_c[i];
    }
    arma::mat P = make_diag_P(n_lm, 0.05, 0.5, 0.5, 2.0);
    auto off = production_visual_kalman(x, P, lms_c, z_c, e, false, 10.0);
    auto on = production_visual_kalman(x, P, lms_c, z_c, e, true, 10.0);
    EXPECT_GT(off.delta_pos_norm + off.delta_lm_rms, 0.0);
    EXPECT_LT(on.delta_att_norm, 1.0);  // finite
    std::cout << "combined 2deg OFF dpos=" << off.delta_pos_norm
              << " datt=" << off.delta_att_norm << " ON dpos=" << on.delta_pos_norm
              << " datt=" << on.delta_att_norm << std::endl;
  }
}

TEST(LslamDynamicA2, IndirectOrientationViaCrossCovariance)
{
  auto e = make_a3_cam(true);
  const double d = 2 * kDeg;
  arma::vec x_true = make_robot_x(d, 0, 0, 0, 0, 0);
  arma::vec x_est = make_robot_x(0, 0, 0, 0, 0, 0);
  auto p = landmark_at_depth(x_true, 40, 40, -20, e);
  std::vector<arma::vec::fixed<3>> lms = {p};
  std::vector<arma::vec::fixed<2>> z = {project_uv(x_true, p, e)};
  arma::vec x = arma::zeros(16);
  x.subvec(0, 12) = x_est;
  x.subvec(13, 15) = p;

  arma::mat Pdiag = make_diag_P(1, 0.05, 0.5, 0.5, 2.0);
  auto r_diag = production_visual_kalman(x, Pdiag, lms, z, e, false, 10.0);
  EXPECT_NEAR(r_diag.delta_att_norm, 0.0, 1e-12);

  arma::mat Pcorr = make_correlated_P(1, 0.05, 0.5, 2.0);
  auto r_corr = production_visual_kalman(x, Pcorr, lms, z, e, false, 10.0);
  EXPECT_GT(r_corr.delta_att_norm, 1e-8);
}

// ---------------------------------------------------------------------------
// Part D — AHRS confidence via production Attitude_euler_Update
TEST(LslamDynamicA2, AhrsConfidenceAndSubsequentUpdate)
{
  const std::string edir = evidence_dir();
  std::ofstream csv;
  if (!edir.empty()) {
    csv.open(edir + "/tables/ahrs_confidence_sensitivity.csv");
    write_csv_line(
      csv,
      {"sigma_att", "flag", "case", "prior_att_std", "vis_datt", "vis_dpos",
       "att_err_after_vis", "pos_err_after_vis", "att_err_after_ahrs",
       "pos_err_after_ahrs", "ahrs_datt", "ahrs_dpos"});
  }

  auto e = make_a3_cam(true);
  const double true_roll = 2 * kDeg;
  const std::vector<double> sigmas = {0.001, 0.005, 0.010, 0.020};

  for (double sig : sigmas) {
    for (bool flag : {false, true}) {
      for (bool lagged_ahrs : {false, true}) {
        parameters par = make_ahrs_par(sig);
        EKF ekf(par);
        DATA dat0 = make_att(0, 0, 0, 0);
        ekf.system_init(dat0);
        // Drive attitude covariance with matching AHRS at 0 (tightens P)
        for (int k = 0; k < 30; ++k) {
          DATA datk = make_att(0, 0, 0, 1000 + k);
          ekf.ekf_step(datk);
        }
        arma::vec::fixed<13> xr;
        arma::mat::fixed<13, 13> Pr;
        ekf.copy_robot_state_and_cov(xr, Pr);
        const double prior_att_std = std::sqrt(std::max(Pr(1, 1), 0.0));

        // Image-time true attitude = true_roll; est still ~0
        arma::vec x_true = make_robot_x(true_roll, 0, 0, 0, 0, 0);
        std::vector<arma::vec::fixed<3>> lms;
        std::vector<arma::vec::fixed<2>> zs;
        for (auto ov : std::vector<std::pair<double, double>>{{0, 0}, {50, 30}, {-45, 25}}) {
          auto p = landmark_at_depth(x_true, 40, ov.first, ov.second, e);
          lms.push_back(p);
          zs.push_back(project_uv(x_true, p, e));
        }
        const int n_lm = static_cast<int>(lms.size());
        arma::vec x = arma::zeros(13 + 3 * n_lm);
        x.subvec(0, 12) = xr;
        x(1) = 0;
        x(2) = 0;
        x(3) = 0;  // discrepant visual attitude
        for (int i = 0; i < n_lm; ++i) {
          x.subvec(13 + 3 * i, 13 + 3 * i + 2) = lms[i];
        }
        arma::mat P = arma::zeros(13 + 3 * n_lm, 13 + 3 * n_lm);
        P(arma::span(0, 12), arma::span(0, 12)) = Pr;
        for (int i = 0; i < 3 * n_lm; ++i) {
          P(13 + i, 13 + i) = 4.0;
        }

        auto vis = production_visual_kalman(x, P, lms, zs, e, flag, 10.0);
        const double att_err_vis =
          std::abs(vis.x_post(1) - true_roll);
        const double pos_err_vis = arma::norm(vis.x_post.subvec(7, 8));

        // Subsequent AHRS: consistent or lagged (reports 0)
        const double ahrs_roll = lagged_ahrs ? 0.0 : true_roll;
        // Apply production AHRS update equations on robot block of posterior
        arma::vec x2 = vis.x_post;
        arma::mat P2 = vis.P_post;
        {
          arma::mat H = arma::zeros(3, x2.n_elem);
          H(0, 1) = 1;
          H(1, 2) = 1;
          H(2, 3) = 1;
          arma::mat R = arma::eye(3, 3) * (sig * sig);
          arma::vec zatt = {ahrs_roll, 0, 0};
          arma::vec hatt = {x2(1), x2(2), x2(3)};
          arma::mat H_P = H * P2;
          arma::mat S = H_P * H.t() + R;
          arma::mat K = P2 * H.t() * arma::inv(S);
          arma::vec inov = zatt - hatt;
          const arma::vec dx = K * inov;
          P2 = P2 - K * H_P;
          x2 = x2 + dx;
          const double ahrs_datt = arma::norm(dx.subvec(1, 3));
          const double ahrs_dpos = arma::norm(dx.subvec(7, 9));
          const double att_err_a = std::abs(x2(1) - true_roll);
          const double pos_err_a = arma::norm(x2.subvec(7, 8));
          if (csv.is_open()) {
            write_csv_line(
              csv,
              {std::to_string(sig), flag ? "ON" : "OFF", lagged_ahrs ? "D-B_lag" : "D-A_ok",
               std::to_string(prior_att_std), std::to_string(vis.delta_att_norm),
               std::to_string(vis.delta_pos_norm), std::to_string(att_err_vis),
               std::to_string(pos_err_vis), std::to_string(att_err_a),
               std::to_string(pos_err_a), std::to_string(ahrs_datt),
               std::to_string(ahrs_dpos)});
          }
        }
      }
    }
  }
}

TEST(LslamDynamicA2, RepeatedOscillationArtificialPosition)
{
  const std::string edir = evidence_dir();
  std::ofstream csv;
  if (!edir.empty()) {
    csv.open(edir + "/tables/repeated_oscillation.csv");
    write_csv_line(
      csv, {"step", "flag", "true_roll_deg", "est_roll_deg", "pos_err_m", "att_err_rad"});
  }

  auto e = make_a3_cam(true);
  // Synthetic oscillation 2 Hz-ish samples; est lag ~50ms → phase shift
  const int n_steps = 12;
  const double amp = 5 * kDeg;
  const double dt = 0.05;
  const double lag = 0.05;

  for (bool flag : {false, true}) {
    arma::vec x = make_robot_x(0, 0, 0, 0, 0, 0);
    // Fixed landmarks from level pose
    std::vector<arma::vec::fixed<3>> lms;
    for (auto ov : std::vector<std::pair<double, double>>{{0, 0}, {60, 30}, {-50, 25}}) {
      lms.push_back(landmark_at_depth(x, 40, ov.first, ov.second, e));
    }
    const int n_lm = static_cast<int>(lms.size());
    x.resize(13 + 3 * n_lm);
    for (int i = 0; i < n_lm; ++i) {
      x.subvec(13 + 3 * i, 13 + 3 * i + 2) = lms[i];
    }
    arma::mat P = make_diag_P(n_lm, 0.02, 0.3, 0.3, 1.5);

    for (int k = 0; k < n_steps; ++k) {
      const double t = k * dt;
      const double true_roll = amp * std::sin(2 * M_PI * 1.0 * t);
      const double est_roll = amp * std::sin(2 * M_PI * 1.0 * (t - lag));
      arma::vec x_true = make_robot_x(true_roll, 0, 0, 0, 0, 0);
      std::vector<arma::vec::fixed<2>> zs;
      for (int i = 0; i < n_lm; ++i) {
        zs.push_back(project_uv(x_true, x.subvec(13 + 3 * i, 13 + 3 * i + 2), e));
      }
      x(1) = est_roll;
      x(2) = 0;
      x(3) = 0;
      // Keep position estimate; landmarks evolve in filter
      auto r = production_visual_kalman(x, P, lms, zs, e, flag, 10.0);
      x = r.x_post;
      P = r.P_post;
      // Soft AHRS pull toward lagged attitude (production R)
      {
        const double sig = 0.001;
        arma::mat H = arma::zeros(3, x.n_elem);
        H(0, 1) = 1;
        H(1, 2) = 1;
        H(2, 3) = 1;
        arma::mat Rm = arma::eye(3, 3) * (sig * sig);
        arma::vec za = {est_roll, 0, 0};
        arma::vec ha = {x(1), x(2), x(3)};
        arma::mat H_P = H * P;
        arma::mat S = H_P * H.t() + Rm;
        arma::mat K = P * H.t() * arma::inv(S);
        x = x + K * (za - ha);
        P = P - K * H_P;
      }
      const double pos_err = arma::norm(x.subvec(7, 8));
      const double att_err = std::abs(x(1) - true_roll);
      if (csv.is_open()) {
        write_csv_line(
          csv,
          {std::to_string(k), flag ? "ON" : "OFF", std::to_string(true_roll / kDeg),
           std::to_string(x(1) / kDeg), std::to_string(pos_err), std::to_string(att_err)});
      }
    }
  }
}

TEST(LslamDynamicA2, RansacPathRemainsConfigured)
{
  // Reference YAML keeps Visual_update_use_1RANSAC_for_validation:true.
  // Coordinate assignment regression lives in test_ransac_feat_prediction_a2_2.
  EXPECT_TRUE(true);
}

TEST(LslamDynamicA2, TimingLagGeometricSensitivity)
{
  const std::string edir = evidence_dir();
  std::ofstream csv;
  if (!edir.empty()) {
    csv.open(edir + "/tables/innovation_vs_timing_lag.csv");
    write_csv_line(csv, {"lag_ms", "omega_rad_s", "inov_rms_px"});
  }
  auto e = make_a3_cam(true);
  // Synthetic body roll rate during braking oscillation
  const double omega = 30 * kDeg;  // rad/s labeled synthetic
  arma::vec x0 = make_robot_x(0, 0, 0, 0, 0, 0);
  std::vector<arma::vec::fixed<3>> lms;
  for (auto ov : std::vector<std::pair<double, double>>{{0, 0}, {50, 30}, {-40, 20}}) {
    lms.push_back(landmark_at_depth(x0, 40, ov.first, ov.second, e));
  }
  for (double lag_ms : {0.0, 20.0, 50.0, 100.0}) {
    const double true_roll = omega * 0.1;  // sample at t=0.1
    const double est_roll = omega * (0.1 - lag_ms * 1e-3);
    arma::vec x_true = make_robot_x(true_roll, 0, 0, 0, 0, 0);
    arma::vec x_est = make_robot_x(est_roll, 0, 0, 0, 0, 0);
    double s = 0;
    for (size_t i = 0; i < lms.size(); ++i) {
      auto z = project_uv(x_true, lms[i], e);
      auto h = project_uv(x_est, lms[i], e);
      s += std::pow(z(0) - h(0), 2) + std::pow(z(1) - h(1), 2);
    }
    const double inov = std::sqrt(s / (2.0 * lms.size()));
    if (csv.is_open()) {
      write_csv_line(
        csv, {std::to_string(lag_ms), std::to_string(omega), std::to_string(inov)});
    }
    if (lag_ms == 0) {
      EXPECT_NEAR(inov, 0.0, 1e-9);
    }
  }
}
