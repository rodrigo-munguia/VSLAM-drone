/**
 * E5e-G2: horizontal ground-plane range-assisted landmark init geometry + Jacobians.
 * Deterministic — no Webots / Isaac.
 */

#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <armadillo>
#include <opencv2/core.hpp>

#include "ekf_Jacobians.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include "../../common/Vision/vision.hpp"

namespace
{

constexpr double kDeg = M_PI / 180.0;

arma::mat::fixed<3, 3> euler_Rr2n(double roll, double pitch, double yaw)
{
  double Ra2b[9];
  Euler_to_Ra2b_colum_major(roll, pitch, yaw, Ra2b);
  arma::mat Rn2r(Ra2b, 3, 3);
  return Rn2r.t();
}

CAM make_cam()
{
  static double kDistortions[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  CAM c{};
  c.cc[0] = 640.0;
  c.cc[1] = 360.0;
  c.fc[0] = 910.1;
  c.fc[1] = 910.1;
  c.alpha_c = 0.0;
  c.distortions = kDistortions;
  return c;
}

}  // namespace

TEST(RangeHorizontalPlaneInit, LambdaRejectsParallelAndBehind)
{
  arma::vec::fixed<3> optical = {0.0, 0.0, 1.0};
  arma::vec::fixed<3> m_horiz = {1.0, 0.0, 0.0};
  double lambda = 0.0;
  EXPECT_FALSE(range_horizontal_plane_lambda(m_horiz, optical, 50.0, &lambda));

  arma::vec::fixed<3> m_up = {0.0, 0.0, -1.0};  // opposite hemisphere → λ < 0
  EXPECT_FALSE(range_horizontal_plane_lambda(m_up, optical, 50.0, &lambda));

  EXPECT_FALSE(range_horizontal_plane_lambda(optical, optical, -1.0, &lambda));
  EXPECT_FALSE(range_horizontal_plane_lambda(optical, optical, NAN, &lambda));
}

TEST(RangeHorizontalPlaneInit, NadirAgreesWithOldCameraNormal)
{
  // Level: λ = r / (m·v) = r * vz/mz with vz=mz=1 for center → same as r for center.
  arma::vec::fixed<3> optical = {0.0, 0.0, 1.0};
  arma::vec::fixed<3> m = {0.1, -0.2, 1.0};
  m = m / arma::norm(m);
  const double r = 50.0;
  double lambda_h = 0.0;
  ASSERT_TRUE(range_horizontal_plane_lambda(m, optical, r, &lambda_h));
  const double lambda_old = r / arma::dot(m, optical);
  EXPECT_NEAR(lambda_h, lambda_old, 1e-12);
}

TEST(RangeHorizontalPlaneInit, HitDefinesConstantZ)
{
  // Camera at origin, optical tilted; hit z shared by all features.
  const double roll = 10.0 * kDeg;
  const arma::mat::fixed<3, 3> Rr2n = euler_Rr2n(roll, 0.0, 0.0);
  const arma::mat::fixed<3, 3> Rc2r = arma::eye(3, 3);  // optical = body +Z
  const arma::mat Rc2n = Rr2n * Rc2r;
  arma::vec::fixed<3> vn = Rc2n * arma::vec::fixed<3>{0.0, 0.0, 1.0};
  const double r = 50.0;
  const arma::vec::fixed<3> p_C = {0.0, 0.0, 0.0};
  const arma::vec::fixed<3> p_hit = p_C + r * vn;
  const double z_plane = p_hit(2);

  CAM cam = make_cam();
  arma::mat::fixed<3, 2> dhc;
  for (double du : {-100.0, 0.0, 150.0}) {
    for (double dv : {-80.0, 0.0, 120.0}) {
      cv::Point2d uvd(cam.cc[0] + du, cam.cc[1] + dv);
      arma::vec::fixed<3> hc = Inverse_projection_model(uvd, 1, false, cam, dhc);
      arma::vec::fixed<3> hn = Rc2n * hc;
      arma::vec::fixed<3> m = hn / arma::norm(hn);
      double lambda = 0.0;
      if (!range_horizontal_plane_lambda(m, vn, r, &lambda)) {
        continue;
      }
      arma::vec::fixed<3> p = p_C + lambda * m;
      EXPECT_NEAR(p(2), z_plane, 1e-9) << "u=" << uvd.x << " v=" << uvd.y;
    }
  }
}

TEST(RangeHorizontalPlaneInit, JacobianMatchesFiniteDifferences)
{
  CAM cam = make_cam();
  arma::mat::fixed<3, 3> Rc2r = arma::eye(3, 3);
  arma::vec::fixed<3> t_c2r = {0.0, 0.0, 0.0};
  arma::vec x = arma::zeros<arma::vec>(13);
  x(1) = 8.0 * kDeg;   // roll
  x(2) = -5.0 * kDeg;  // pitch
  x(3) = 20.0 * kDeg;  // yaw
  x(7) = 1.0;
  x(8) = -2.0;
  x(9) = 3.0;
  const double range_eff = 48.5;
  cv::Point2d uvd(700.0, 300.0);

  arma::mat::fixed<3, 13> dy_dx;
  arma::mat::fixed<3, 3> dy_duvr;
  Jac_e_init_feat_wr_horizontal(uvd, x, cam, Rc2r, t_c2r, range_eff, dy_dx, dy_duvr);

  auto eval_p = [&](const arma::vec & xs, double u, double v, double r) {
    arma::mat::fixed<3, 2> dhc;
    arma::vec::fixed<3> hc =
      Inverse_projection_model(cv::Point2d(u, v), 1, true, cam, dhc);
    double Ra2b[9];
    Euler_to_Ra2b_colum_major(xs(1), xs(2), xs(3), Ra2b);
    arma::mat Rn2r(Ra2b, 3, 3);
    arma::mat::fixed<3, 3> Rr2n = Rn2r.t();
    arma::mat Rc2n = Rr2n * Rc2r;
    arma::vec::fixed<3> hn = Rc2n * hc;
    arma::vec::fixed<3> m = hn / arma::norm(hn);
    arma::vec::fixed<3> vn = Rc2n * arma::vec::fixed<3>{0.0, 0.0, 1.0};
    double lambda = 0.0;
    EXPECT_TRUE(range_horizontal_plane_lambda(m, vn, r, &lambda));
    return arma::vec(Rr2n * t_c2r + xs.subvec(7, 9) + lambda * m);
  };

  const arma::vec p0 = eval_p(x, uvd.x, uvd.y, range_eff);
  const double eps_a = 1e-7;
  for (int k = 1; k <= 3; ++k) {
    arma::vec xp = x;
    arma::vec xm = x;
    xp(k) += eps_a;
    xm(k) -= eps_a;
    arma::vec fd = (eval_p(xp, uvd.x, uvd.y, range_eff) - eval_p(xm, uvd.x, uvd.y, range_eff)) /
                   (2.0 * eps_a);
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(dy_dx(i, k), fd(i), 5e-5) << "attitude col " << k << " row " << i;
    }
  }
  for (int k = 7; k <= 9; ++k) {
    arma::vec xp = x;
    arma::vec xm = x;
    xp(k) += eps_a;
    xm(k) -= eps_a;
    arma::vec fd = (eval_p(xp, uvd.x, uvd.y, range_eff) - eval_p(xm, uvd.x, uvd.y, range_eff)) /
                   (2.0 * eps_a);
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(dy_dx(i, k), fd(i), 1e-6) << "pos col " << k << " row " << i;
    }
  }

  const double eps_uv = 1e-4;
  {
    arma::vec fd_u =
      (eval_p(x, uvd.x + eps_uv, uvd.y, range_eff) - eval_p(x, uvd.x - eps_uv, uvd.y, range_eff)) /
      (2.0 * eps_uv);
    arma::vec fd_v =
      (eval_p(x, uvd.x, uvd.y + eps_uv, range_eff) - eval_p(x, uvd.x, uvd.y - eps_uv, range_eff)) /
      (2.0 * eps_uv);
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(dy_duvr(i, 0), fd_u(i), 5e-4) << "du row " << i;
      EXPECT_NEAR(dy_duvr(i, 1), fd_v(i), 5e-4) << "dv row " << i;
    }
  }

  const double eps_r = 1e-5;
  arma::vec fd_r =
    (eval_p(x, uvd.x, uvd.y, range_eff + eps_r) - eval_p(x, uvd.x, uvd.y, range_eff - eps_r)) /
    (2.0 * eps_r);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(dy_duvr(i, 2), fd_r(i), 1e-6) << "dr row " << i;
  }

  // Covariance assembly smoke: Ry finite, Py PSD-ish (sym + nonneg eig)
  arma::mat::fixed<3, 3> Ry = arma::diagmat(arma::vec{1.0, 1.0, 4.0});
  arma::mat P13 = 1e-4 * arma::eye(13, 13);
  arma::mat Py = dy_dx * P13 * dy_dx.t() + dy_duvr * Ry * dy_duvr.t();
  EXPECT_TRUE(arma::approx_equal(Py, Py.t(), "absdiff", 1e-12));
  arma::vec eig = arma::eig_sym(Py);
  EXPECT_GE(eig.min(), -1e-9);
  EXPECT_TRUE(Py.is_finite());
  (void)p0;
}
