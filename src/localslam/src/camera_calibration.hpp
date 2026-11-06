#ifndef LOCALSLAM_CAMERA_CALIBRATION_HPP_
#define LOCALSLAM_CAMERA_CALIBRATION_HPP_

/**
 * A8: Camera calibration contracts for Local SLAM.
 *
 * Estimator camera model (existing, unchanged):
 * - Intrinsics: fc_u, fc_v (focal px), cc_u, cc_v (principal point),
 *   alpha_c (skew; used as KK = [[fc1, alpha_c*fc2, cc1],[0,fc2,cc2],[0,0,1]]).
 * - Distortion: Bouguet radial only — distortions[0]=k1, [1]=k2.
 *   Tangential (p1,p2) and k3 are NOT applied by projection/Jacobians even if
 *   stored in the 5-vector. Images are treated as RAW (distorted); undistort
 *   runs inside Inverse_projection_model.
 * - Resolution: Mono_cam_img_cols × Mono_cam_img_rows (feature border checks).
 *
 * CameraInfo mapping:
 * - image_path=raw: use K (fx=K[0], fy=K[4], cx=K[2], cy=K[5],
 *   alpha_c = K[1]/fy if fy!=0 else 0) and D with plumb_bob (or empty/"")
 *   requiring |p1|,|p2|,|k3| <= eps; only k1,k2 used.
 * - image_path=rectified: use P (fx=P[0], fy=P[5], cx=P[2], cy=P[6],
 *   alpha_c = P[1]/fy) and force D = {0,0,0,0,0}. Non-zero D is rejected
 *   (rectification is an upstream responsibility).
 *
 * Rejected: equidistant / rational_polynomial / other models; nonzero
 * unsupported D coeffs; binning_x/y not in {0,1}; ROI not full-frame;
 * zero/non-finite intrinsics; height/width < 1.
 */

#include <cmath>
#include <string>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"

namespace localslam
{
namespace camera_cal
{

struct Intrinsics
{
  int width{0};
  int height{0};
  double fc_u{0.0};
  double fc_v{0.0};
  double cc_u{0.0};
  double cc_v{0.0};
  double alpha_c{0.0};
  std::vector<double> distortions{0.0, 0.0, 0.0, 0.0, 0.0};
  std::string optical_frame;
  std::string image_path;  // "raw" or "rectified"
};

inline bool nearly_equal(double a, double b, double eps = 1e-9)
{
  return std::fabs(a - b) <= eps;
}

inline bool intrinsics_equal(const Intrinsics & a, const Intrinsics & b, double eps = 1e-9)
{
  if (a.width != b.width || a.height != b.height) {
    return false;
  }
  if (a.image_path != b.image_path) {
    return false;
  }
  if (!nearly_equal(a.fc_u, b.fc_u, eps) || !nearly_equal(a.fc_v, b.fc_v, eps) ||
    !nearly_equal(a.cc_u, b.cc_u, eps) || !nearly_equal(a.cc_v, b.cc_v, eps) ||
    !nearly_equal(a.alpha_c, b.alpha_c, eps))
  {
    return false;
  }
  if (a.distortions.size() != b.distortions.size()) {
    return false;
  }
  for (size_t i = 0; i < a.distortions.size(); ++i) {
    if (!nearly_equal(a.distortions[i], b.distortions[i], eps)) {
      return false;
    }
  }
  return true;
}

struct MapResult
{
  bool ok{false};
  std::string error;
  Intrinsics intrinsics;
};

inline bool is_supported_distortion_model(const std::string & model)
{
  return model.empty() || model == "plumb_bob" || model == "plumb_bob_k3";
}

/**
 * Map sensor_msgs/CameraInfo into estimator Intrinsics for the declared path.
 * Does not mutate the message.
 */
inline MapResult map_camera_info(
  const sensor_msgs::msg::CameraInfo & msg,
  const std::string & image_path,
  double unsupported_d_eps = 1e-9)
{
  MapResult out;
  out.intrinsics.image_path = image_path;
  out.intrinsics.optical_frame = msg.header.frame_id;

  if (image_path != "raw" && image_path != "rectified") {
    out.error = "image_path must be 'raw' or 'rectified'";
    return out;
  }
  if (msg.width < 1 || msg.height < 1) {
    out.error = "CameraInfo width/height must be >= 1";
    return out;
  }
  // binning: 0 means "none" in many drivers; only 0 or 1 accepted (no subsample).
  if ((msg.binning_x != 0 && msg.binning_x != 1) ||
    (msg.binning_y != 0 && msg.binning_y != 1))
  {
    out.error = "unsupported binning_x/y (only 0 or 1 allowed)";
    return out;
  }
  // ROI must be full frame (x/y offset 0 and size 0=full or equals width/height).
  const auto & roi = msg.roi;
  const bool roi_full =
    (roi.x_offset == 0 && roi.y_offset == 0) &&
    ((roi.width == 0 && roi.height == 0) ||
     (roi.width == msg.width && roi.height == msg.height));
  if (!roi_full) {
    out.error = "unsupported CameraInfo ROI (must be full-frame)";
    return out;
  }

  if (!is_supported_distortion_model(msg.distortion_model)) {
    out.error =
      "unsupported distortion_model='" + msg.distortion_model +
      "' (accepted: empty, plumb_bob)";
    return out;
  }

  out.intrinsics.width = static_cast<int>(msg.width);
  out.intrinsics.height = static_cast<int>(msg.height);
  out.intrinsics.distortions.assign(5, 0.0);

  if (image_path == "raw") {
    if (msg.k.size() < 9) {
      out.error = "CameraInfo.K incomplete for raw path";
      return out;
    }
    const double fx = msg.k[0];
    const double fy = msg.k[4];
    const double cx = msg.k[2];
    const double cy = msg.k[5];
    const double skew = msg.k[1];
    if (!(std::isfinite(fx) && std::isfinite(fy) && std::isfinite(cx) &&
        std::isfinite(cy) && std::isfinite(skew)) ||
      fx == 0.0 || fy == 0.0)
    {
      out.error = "invalid/uncalibrated K for raw path (fx,fy must be nonzero finite)";
      return out;
    }
    out.intrinsics.fc_u = fx;
    out.intrinsics.fc_v = fy;
    out.intrinsics.cc_u = cx;
    out.intrinsics.cc_v = cy;
    out.intrinsics.alpha_c = skew / fy;

    // D: plumb_bob [k1,k2,p1,p2,k3] (may be shorter)
    std::vector<double> d = msg.d;
    while (d.size() < 5) {
      d.push_back(0.0);
    }
    if (d.size() > 5) {
      // Extra coeffs must be ~0
      for (size_t i = 5; i < d.size(); ++i) {
        if (!nearly_equal(d[i], 0.0, unsupported_d_eps)) {
          out.error = "unsupported CameraInfo.D length/coeffs beyond k1..k3";
          return out;
        }
      }
    }
    const double k1 = d[0];
    const double k2 = d[1];
    const double p1 = d[2];
    const double p2 = d[3];
    const double k3 = d[4];
    if (!std::isfinite(k1) || !std::isfinite(k2) || !std::isfinite(p1) ||
      !std::isfinite(p2) || !std::isfinite(k3))
    {
      out.error = "non-finite CameraInfo.D";
      return out;
    }
    if (!nearly_equal(p1, 0.0, unsupported_d_eps) ||
      !nearly_equal(p2, 0.0, unsupported_d_eps) ||
      !nearly_equal(k3, 0.0, unsupported_d_eps))
    {
      out.error =
        "unsupported distortion: estimator uses Bouguet k1,k2 only "
        "(p1,p2,k3 must be ~0)";
      return out;
    }
    out.intrinsics.distortions = {k1, k2, 0.0, 0.0, 0.0};
  } else {
    // rectified: prefer P; D must be zero / unused
    if (msg.p.size() < 12) {
      out.error = "CameraInfo.P incomplete for rectified path";
      return out;
    }
    const double fx = msg.p[0];
    const double fy = msg.p[5];
    const double cx = msg.p[2];
    const double cy = msg.p[6];
    const double skew = msg.p[1];
    if (!(std::isfinite(fx) && std::isfinite(fy) && std::isfinite(cx) &&
        std::isfinite(cy) && std::isfinite(skew)) ||
      fx == 0.0 || fy == 0.0)
    {
      out.error = "invalid P for rectified path (fx,fy must be nonzero finite)";
      return out;
    }
    for (double v : msg.d) {
      if (!nearly_equal(v, 0.0, unsupported_d_eps)) {
        out.error =
          "rectified path requires D≈0 (rectify upstream; do not pass raw D)";
        return out;
      }
    }
    out.intrinsics.fc_u = fx;
    out.intrinsics.fc_v = fy;
    out.intrinsics.cc_u = cx;
    out.intrinsics.cc_v = cy;
    out.intrinsics.alpha_c = skew / fy;
    out.intrinsics.distortions = {0.0, 0.0, 0.0, 0.0, 0.0};
  }

  out.ok = true;
  return out;
}

inline bool image_size_matches(
  const Intrinsics & cal, int img_cols, int img_rows)
{
  return cal.width == img_cols && cal.height == img_rows;
}

}  // namespace camera_cal
}  // namespace localslam

#endif  // LOCALSLAM_CAMERA_CALIBRATION_HPP_
