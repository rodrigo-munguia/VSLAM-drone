/**
 * GSLAM-VISUAL-LOOP-A2b: visual-loop OpenCV PnP ↔ nav-frame helpers.
 *
 * Repository camera model:
 *   p_c = Rn2c * (p_n - t_c2n)     with t_c2n = camera centre C in nav
 *
 * OpenCV solvePnP:
 *   p_c = R_cv * p_w + t_cv
 *
 * Consistency:
 *   R_cv = Rn2c
 *   t_cv = -Rn2c * C
 *   C    = -R_cv^T * t_cv
 *
 * Used only by CLOOP::Get_pos_of_current_kf (visual path). Tag/home-tag
 * geometry is separate (g6b_home_tag_geometry).
 */

#ifndef GLOBALSLAM_CLOOP_VISUAL_PNP_HPP_
#define GLOBALSLAM_CLOOP_VISUAL_PNP_HPP_

#include <cmath>
#include <cstdint>
#include <vector>

#include <armadillo>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "../../common/Vision/vision.hpp"

namespace globalslam
{
namespace cloop_visual_pnp
{

/** Explicit Armadillo (column-major) → OpenCV row-major 3×3 copy. */
inline cv::Mat arma_Rn2c_to_cv_R(const arma::mat::fixed<3, 3> & Rn2c)
{
  cv::Mat R(3, 3, CV_64FC1);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R.at<double>(r, c) = Rn2c(r, c);
    }
  }
  return R;
}

inline arma::mat::fixed<3, 3> cv_R_to_arma(const cv::Mat & R)
{
  arma::mat::fixed<3, 3> out;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out(r, c) = R.at<double>(r, c);
    }
  }
  return out;
}

/** OpenCV translation seed: t_cv = -Rn2c * C. */
inline arma::vec::fixed<3> t_cv_from_nav(
  const arma::mat::fixed<3, 3> & Rn2c, const arma::vec::fixed<3> & t_c2n)
{
  return -Rn2c * t_c2n;
}

/** Nav camera centre from OpenCV extrinsics: C = -R_cv^T * t_cv. */
inline arma::vec::fixed<3> camera_centre_from_cv(
  const arma::mat::fixed<3, 3> & R_cv, const arma::vec::fixed<3> & t_cv)
{
  return -R_cv.t() * t_cv;
}

inline arma::vec::fixed<3> camera_centre_from_cv_mats(
  const cv::Mat & R_cv, const cv::Mat & tvec)
{
  return camera_centre_from_cv(
    cv_R_to_arma(R_cv),
    arma::vec::fixed<3>{
      tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)});
}

/**
 * Reproject world point with OpenCV extrinsics into the undistorted pixel
 * domain (observations for PnP are Undistort_a_point'd).
 */
inline cv::Point2d project_undistorted_from_cv(
  const arma::vec::fixed<3> & Pw,
  const arma::mat::fixed<3, 3> & R_cv,
  const arma::vec::fixed<3> & t_cv,
  const CAM & cam)
{
  const arma::vec::fixed<3> pc = R_cv * Pw + t_cv;
  arma::mat::fixed<2, 3> J;
  // Projection_model returns distorted pixels; undo to match PnP 2D domain.
  const cv::Point2d uvd = Projection_model(pc, 1, false, cam, J);
  if (uvd.x == -1 || uvd.y == -1) {
    return uvd;
  }
  return Undistort_a_point(uvd, cam, 1);
}

struct SolveAcceptResult
{
  bool accepted{false};
  arma::vec::fixed<3> C_nav;
  arma::mat::fixed<3, 3> R_cv;
  arma::vec::fixed<3> t_cv;
  double mean_reprojection_error{0};
  double d_xy{0};
  int n_inliers{0};
  const char * reject_reason{"unset"};
};

/**
 * Production visual PnP solve + acceptance (A2b).
 *
 * @param Points3D  Nav-frame landmarks
 * @param Points2D  Undistorted pixel observations (same domain as PnP input)
 * @param Rn2c_seed Local KF Rn2c (warm start only; not used for gate)
 * @param C_seed    Local KF t_c2n camera centre (warm start + XY gate)
 */
inline SolveAcceptResult solve_and_accept(
  const std::vector<cv::Point3f> & Points3D,
  const std::vector<cv::Point2f> & Points2D,
  const arma::mat::fixed<3, 3> & Rn2c_seed,
  const arma::vec::fixed<3> & C_seed,
  const CAM & cam,
  double radius_uncertainty,
  double xy_gate_m,
  double reproj_thresh_px)
{
  SolveAcceptResult out;
  out.C_nav.zeros();
  out.R_cv.eye();
  out.t_cv.zeros();

  if (Points3D.size() < 4 || Points3D.size() != Points2D.size()) {
    out.reject_reason = "insufficient_correspondences";
    return out;
  }

  cv::Mat R = arma_Rn2c_to_cv_R(Rn2c_seed);
  cv::Mat rvec;
  cv::Rodrigues(R, rvec);

  const arma::vec::fixed<3> t_seed = t_cv_from_nav(Rn2c_seed, C_seed);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << t_seed[0], t_seed[1], t_seed[2]);

  cv::Mat A = cv::Mat::zeros(3, 3, CV_64FC1);
  A.at<double>(0, 0) = cam.fc[0];
  A.at<double>(1, 1) = cam.fc[1];
  A.at<double>(0, 2) = cam.cc[0];
  A.at<double>(1, 2) = cam.cc[1];
  A.at<double>(2, 2) = 1.0;
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
  cv::Mat inliers;

  cv::solvePnPRansac(
    Points3D, Points2D, A, distCoeffs, rvec, tvec,
    /*useExtrinsicGuess=*/true, 500, 1.0, 0.99, inliers,
    cv::SOLVEPNP_ITERATIVE);

  if (inliers.empty()) {
    out.reject_reason = "empty_inliers";
    return out;
  }

  cv::Mat R_cv_mat;
  cv::Rodrigues(rvec, R_cv_mat);
  out.R_cv = cv_R_to_arma(R_cv_mat);
  out.t_cv = {
    tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};
  out.C_nav = camera_centre_from_cv(out.R_cv, out.t_cv);

  // XY gate vs Local KF camera centre (nav), not OpenCV t vectors.
  out.d_xy = std::sqrt(
    std::pow(out.C_nav[0] - C_seed[0], 2) +
    std::pow(out.C_nav[1] - C_seed[1], 2));

  double sum_e = 0;
  int n_in = 0;
  const int n_pts = static_cast<int>(Points3D.size());

  // OpenCV 4: inliers are correspondence indices (CV_32S Nx1).
  if (inliers.type() != CV_32S && inliers.type() != CV_32SC1) {
    out.reject_reason = "unexpected_inliers_type";
    return out;
  }
  for (int k = 0; k < inliers.rows; ++k) {
    const int i = inliers.at<int>(k);
    if (i < 0 || i >= n_pts) {
      continue;
    }
    arma::vec::fixed<3> Pw = {
      Points3D[static_cast<size_t>(i)].x,
      Points3D[static_cast<size_t>(i)].y,
      Points3D[static_cast<size_t>(i)].z};
    const cv::Point2d uv_pred =
      project_undistorted_from_cv(Pw, out.R_cv, out.t_cv, cam);
    const cv::Point2d uv_obs(
      Points2D[static_cast<size_t>(i)].x,
      Points2D[static_cast<size_t>(i)].y);
    if (uv_pred.x == -1 || uv_pred.y == -1) {
      out.reject_reason = "reproj_behind_camera";
      out.mean_reprojection_error = 1e8;
      return out;
    }
    sum_e += std::sqrt(
      std::pow(uv_pred.x - uv_obs.x, 2) + std::pow(uv_pred.y - uv_obs.y, 2));
    ++n_in;
  }
  if (n_in < 1) {
    out.reject_reason = "no_valid_inliers";
    return out;
  }
  out.n_inliers = n_in;
  out.mean_reprojection_error = sum_e / static_cast<double>(n_in);

  if (out.mean_reprojection_error < reproj_thresh_px &&
    out.d_xy < radius_uncertainty + xy_gate_m)
  {
    out.accepted = true;
    out.reject_reason = "ok";
  } else {
    out.reject_reason = "gate_failed";
  }
  return out;
}

}  // namespace cloop_visual_pnp
}  // namespace globalslam

#endif  // GLOBALSLAM_CLOOP_VISUAL_PNP_HPP_
