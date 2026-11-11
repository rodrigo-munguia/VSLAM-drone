/**
 * Landmark birth two-view reprojection (GSLAM-INIT-D3/D4 convention).
 *
 * RMS = sqrt(0.5 * (e_prev^2 + e_curr^2)) where each e is the Euclidean
 * pixel error between Projection_model(pt in camera) and Undistort_a_point(uv).
 * Camera pose uses KEYFRAME::Rn2c and t_c2n (nav frame), matching D3 audit logs.
 *
 * Used by the Init_birth_reproj gate (production default ON) and by audit logging.
 */
#ifndef GMAP_BIRTH_REPROJ_HPP
#define GMAP_BIRTH_REPROJ_HPP

#include <cmath>
#include <limits>

#include <armadillo>
#include <opencv2/core.hpp>

#include "../../common/Vision/vision.hpp"
#include "../../common/keyframe_types.hpp"

namespace gmap_birth_reproj
{

inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

inline double reproj_err_px(
  const arma::vec::fixed<3> & pt_n,
  const KEYFRAME & kf,
  const cv::Point2f & uv_distorted,
  const CAM & cam)
{
  arma::vec::fixed<3> pc = kf.Rn2c * (pt_n - kf.t_c2n);
  if (!(pc[2] > 1e-9) || !std::isfinite(pc[2])) {
    return kNaN;
  }
  arma::mat::fixed<2, 3> duv_dPc;
  cv::Point2d uv_hat = Projection_model(pc, 0, false, cam, duv_dPc);
  if (!std::isfinite(uv_hat.x) || !std::isfinite(uv_hat.y)) {
    return kNaN;
  }
  cv::Point2d uv_u = Undistort_a_point(cv::Point2d(uv_distorted.x, uv_distorted.y), cam, 1);
  if (!std::isfinite(uv_u.x) || !std::isfinite(uv_u.y)) {
    return kNaN;
  }
  const double dx = uv_hat.x - uv_u.x;
  const double dy = uv_hat.y - uv_u.y;
  return std::sqrt(dx * dx + dy * dy);
}

/** Two-view RMS matching D3 `reproj_rms_px`. Returns NaN if either view invalid. */
inline double two_view_reproj_rms_px(
  const arma::vec::fixed<3> & pt_n,
  const KEYFRAME & kf_prev,
  const KEYFRAME & kf_curr,
  const cv::Point2f & uv_prev,
  const cv::Point2f & uv_curr,
  const CAM & cam)
{
  const double e_prev = reproj_err_px(pt_n, kf_prev, uv_prev, cam);
  const double e_curr = reproj_err_px(pt_n, kf_curr, uv_curr, cam);
  if (!std::isfinite(e_prev) || !std::isfinite(e_curr)) {
    return kNaN;
  }
  return std::sqrt(0.5 * (e_prev * e_prev + e_curr * e_curr));
}

/**
 * Gate decision. When enable=false, always accept (caller skips compute).
 * When enable=true: reject if RMS is nonfinite or RMS > max_rms_px.
 */
inline bool passes_gate(bool enable, double rms_px, double max_rms_px)
{
  if (!enable) {
    return true;
  }
  if (!std::isfinite(rms_px)) {
    return false;
  }
  return rms_px <= max_rms_px;
}

}  // namespace gmap_birth_reproj

#endif
