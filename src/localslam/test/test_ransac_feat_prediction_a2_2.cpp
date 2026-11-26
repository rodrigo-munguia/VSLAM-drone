/**
 * LSLAM-DYNAMIC-CONSISTENCY-A2.2 — FeatPrediction hi(1) must use PredictedPoint.y.
 * Exercises the production free function linked from ekf_visual_update_With_validation.cpp.
 */

#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <armadillo>
#include <opencv2/core.hpp>

#include "ekf_types.hpp"
#include "ekf_Jacobians.hpp"
#include "parameters.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include "../../common/Vision/vision.hpp"

// Production symbol (external linkage) in ekf_visual_update_With_validation.cpp.
void FeatPrediction(
  arma::vec & x, std::vector<FEAT> & FeatsDATA, std::vector<FEAT> & AnchorsDATA,
  int type, int idx, CAM & camera_parameters, parameters & par,
  arma::mat & Rr2c, arma::vec::fixed<3> & t_c2r,
  arma::vec::fixed<2> & hi, arma::vec::fixed<2> & zi,
  arma::mat::fixed<2, 2> & Ri, arma::mat & Hi);

namespace
{

constexpr double kDeg = M_PI / 180.0;

parameters make_par()
{
  parameters par{};
  par.Visual_update_attitude_update = false;
  par.Sigma_cam_uv = 10.0;
  par.Sigma_cam_uv_anchors = 10.0;
  return par;
}

void make_cam_extrinsics(CAM & cam, arma::mat::fixed<3, 3> & Rr2c, arma::vec::fixed<3> & t_c2r,
  double * distortions)
{
  cam.cc[0] = 200.0;
  cam.cc[1] = 120.0;
  cam.fc[0] = 200.0;
  cam.fc[1] = 200.0;
  cam.alpha_c = 0.0;
  cam.distortions = distortions;
  t_c2r = {0.1, 0.0, 0.0};
  double Ra2b[9];
  Euler_to_Ra2b_colum_major(0.0, 0.0, 1.5707963268, Ra2b);
  Rr2c = arma::mat(Ra2b, 3, 3);
}

arma::vec::fixed<2> project_uv(
  const arma::vec & x, const arma::vec::fixed<3> & pxyz,
  CAM & cam, const arma::mat::fixed<3, 3> & Rr2c, const arma::vec::fixed<3> & t_c2r)
{
  double Ra2b[9];
  Euler_to_Ra2b_colum_major(x(1), x(2), x(3), Ra2b);
  arma::mat Rn2r(Ra2b, 3, 3);
  arma::mat::fixed<3, 3> Rn2c = Rr2c * Rn2r;
  arma::vec::fixed<3> pc = Rn2c * (pxyz - x.subvec(7, 9)) - Rr2c * t_c2r;
  arma::mat::fixed<2, 3> duv_dPc;
  cv::Point2d uvd = Projection_model(pc, 1, false, cam, duv_dPc);
  return {uvd.x, uvd.y};
}

arma::vec::fixed<3> landmark_at(
  const arma::vec & x, double depth, double u_off, double v_off,
  CAM & cam, const arma::mat::fixed<3, 3> & Rr2c, const arma::vec::fixed<3> & t_c2r)
{
  arma::mat::fixed<3, 2> dhc;
  cv::Point2d uvd(cam.cc[0] + u_off, cam.cc[1] + v_off);
  arma::vec::fixed<3> hc = Inverse_projection_model(uvd, 1, false, cam, dhc);
  double Ra2b[9];
  Euler_to_Ra2b_colum_major(x(1), x(2), x(3), Ra2b);
  arma::mat Rn2r(Ra2b, 3, 3);
  arma::mat::fixed<3, 3> Rr2n = Rn2r.t();
  arma::mat::fixed<3, 3> Rc2r = Rr2c.t();
  arma::mat Rc2n = Rr2n * Rc2r;
  arma::vec::fixed<3> hn = Rc2n * hc;
  arma::vec::fixed<3> m = hn / arma::norm(hn);
  return arma::vec::fixed<3>(x.subvec(7, 9) + Rr2n * t_c2r + depth * m);
}

}  // namespace

TEST(RansacFeatPredictionA22, PredictedImageCoordinatesDistinctXY)
{
  double distortions[5] = {0, 0, 0, 0, 0};
  CAM cam{};
  arma::mat::fixed<3, 3> Rr2c;
  arma::vec::fixed<3> t_c2r;
  make_cam_extrinsics(cam, Rr2c, t_c2r, distortions);
  parameters par = make_par();

  arma::vec x = arma::zeros<arma::vec>(16);
  // Non-level attitude (Test C geometry folded into this fixture)
  x(1) = 8.0 * kDeg;
  x(2) = -5.0 * kDeg;
  x(3) = 15.0 * kDeg;

  arma::vec::fixed<3> p = landmark_at(x, 40.0, 55.0, -40.0, cam, Rr2c, t_c2r);
  x.subvec(13, 15) = p;
  arma::vec::fixed<2> uv = project_uv(x, p, cam, Rr2c, t_c2r);
  ASSERT_NE(uv(0), uv(1)) << "fixture requires distinct u,v";
  ASSERT_GT(std::abs(uv(0) - uv(1)), 1.0);

  FEAT feat{};
  feat.idx_i_state = 13;
  feat.idx_f_state = 15;
  feat.PredictedPoint = cv::Point2f(static_cast<float>(uv(0)), static_cast<float>(uv(1)));
  feat.MatchedPoint = feat.PredictedPoint;
  std::vector<FEAT> feats = {feat};
  std::vector<FEAT> anchors;

  arma::vec::fixed<2> hi, zi;
  arma::mat::fixed<2, 2> Ri;
  arma::mat Hi;
  arma::mat Rr2c_m = Rr2c;
  FeatPrediction(x, feats, anchors, /*type=*/0, /*idx=*/0, cam, par, Rr2c_m, t_c2r, hi, zi, Ri, Hi);

  // Correct: hi = [PredictedPoint.x, PredictedPoint.y]
  EXPECT_NEAR(hi(0), uv(0), 1e-9);
  EXPECT_NEAR(hi(1), uv(1), 1e-9);
  // Defective assignment hi(1)=PredictedPoint.x would fail this:
  EXPECT_NE(hi(1), hi(0));
  EXPECT_NEAR(hi(1), static_cast<double>(feat.PredictedPoint.y), 1e-9);

  // Hypothesis innovation when observation matches prediction
  EXPECT_NEAR(zi(0) - hi(0), 0.0, 1e-9);
  EXPECT_NEAR(zi(1) - hi(1), 0.0, 1e-9);
}

TEST(RansacFeatPredictionA22, AnchorPredictedCoordinatesUseY)
{
  double distortions[5] = {0, 0, 0, 0, 0};
  CAM cam{};
  arma::mat::fixed<3, 3> Rr2c;
  arma::vec::fixed<3> t_c2r;
  make_cam_extrinsics(cam, Rr2c, t_c2r, distortions);
  parameters par = make_par();

  arma::vec x = arma::zeros<arma::vec>(13);
  x(1) = -10.0 * kDeg;
  x(2) = 6.0 * kDeg;

  arma::vec::fixed<3> p = landmark_at(x, 40.0, -30.0, 45.0, cam, Rr2c, t_c2r);
  arma::vec::fixed<2> uv = project_uv(x, p, cam, Rr2c, t_c2r);
  ASSERT_GT(std::abs(uv(0) - uv(1)), 1.0);

  FEAT anchor{};
  anchor.AnchorState = p;
  anchor.PredictedPoint = cv::Point2f(static_cast<float>(uv(0)), static_cast<float>(uv(1)));
  anchor.MatchedPoint = anchor.PredictedPoint;
  std::vector<FEAT> feats;
  std::vector<FEAT> anchors = {anchor};

  arma::vec::fixed<2> hi, zi;
  arma::mat::fixed<2, 2> Ri;
  arma::mat Hi;
  arma::mat Rr2c_m = Rr2c;
  FeatPrediction(x, feats, anchors, /*type=*/1, /*idx=*/0, cam, par, Rr2c_m, t_c2r, hi, zi, Ri, Hi);

  EXPECT_NEAR(hi(0), uv(0), 1e-9);
  EXPECT_NEAR(hi(1), uv(1), 1e-9);
  EXPECT_NEAR(zi(1) - hi(1), 0.0, 1e-9);
}
