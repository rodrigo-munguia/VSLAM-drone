#ifndef GLOBALSLAM_G4_CALIBRATION_HPP_
#define GLOBALSLAM_G4_CALIBRATION_HPP_

/**
 * G4 calibration compatibility helpers.
 *
 * Supported integrated configuration: Local SLAM Calibration_source=yaml with
 * Mono_cam_* (rows/cols, fc, cc, alpha/skew, distortions) matching Global SLAM
 * YAML. Resolution matching alone is insufficient.
 *
 * Unsupported in G4: Local SLAM CameraInfo-driven calibration paired with a
 * different Global SLAM YAML (no runtime CameraInfo transport on Kf yet).
 * Full calibration transport remains a later slice.
 */

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace globalslam
{
namespace g4_cal
{

struct MonoCamFingerprint
{
  int rows{0};
  int cols{0};
  double cc_u{0};
  double cc_v{0};
  double fc_u{0};
  double fc_v{0};
  double alpha_c{0};
  std::vector<double> distortions;
};

inline bool nearly_equal(double a, double b, double eps = 1e-9)
{
  return std::abs(a - b) <= eps;
}

inline bool fingerprints_match(
  const MonoCamFingerprint & a,
  const MonoCamFingerprint & b,
  std::string * reason)
{
  auto fail = [&](const std::string & msg) {
      if (reason) {
        *reason = msg;
      }
      return false;
    };

  if (a.rows != b.rows || a.cols != b.cols) {
    return fail(
      "image dimensions differ (" + std::to_string(a.rows) + "x" +
      std::to_string(a.cols) + " vs " + std::to_string(b.rows) + "x" +
      std::to_string(b.cols) + ")");
  }
  if (!nearly_equal(a.fc_u, b.fc_u) || !nearly_equal(a.fc_v, b.fc_v)) {
    return fail("focal lengths differ");
  }
  if (!nearly_equal(a.cc_u, b.cc_u) || !nearly_equal(a.cc_v, b.cc_v)) {
    return fail("principal point differs");
  }
  if (!nearly_equal(a.alpha_c, b.alpha_c)) {
    return fail("skew (alpha_c) differs");
  }
  if (a.distortions.size() != b.distortions.size()) {
    return fail("distortion coefficient count differs");
  }
  for (size_t i = 0; i < a.distortions.size(); ++i) {
    if (!nearly_equal(a.distortions[i], b.distortions[i])) {
      return fail("distortion coefficients differ at index " + std::to_string(i));
    }
  }
  return true;
}

inline std::string fingerprint_summary(const MonoCamFingerprint & fp)
{
  std::ostringstream oss;
  oss << fp.rows << "x" << fp.cols
      << " fc=(" << fp.fc_u << "," << fp.fc_v << ")"
      << " cc=(" << fp.cc_u << "," << fp.cc_v << ")"
      << " alpha=" << fp.alpha_c
      << " ndist=" << fp.distortions.size();
  return oss.str();
}

/** True when the operator-selected Local SLAM source is the G4-supported one. */
inline bool localslam_source_supported(const std::string & calibration_source)
{
  return calibration_source == "yaml" || calibration_source.empty();
}

}  // namespace g4_cal
}  // namespace globalslam

#endif  // GLOBALSLAM_G4_CALIBRATION_HPP_
