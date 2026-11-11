/**
 * INIT-D4: birth reprojection gate decision + D3 RMS convention (D=0 cameras).
 */
#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include <vector>

#include "gmap_birth_reproj.hpp"
#include "../../common/Vision/vision.hpp"

namespace
{

CAM make_cam_d0()
{
  static std::vector<double> d{0, 0, 0, 0, 0};
  CAM cam;
  cam.distortions = d.data();
  cam.fc[0] = 200.0;
  cam.fc[1] = 200.0;
  cam.cc[0] = 200.0;
  cam.cc[1] = 120.0;
  cam.alpha_c = 0.0;
  return cam;
}

KEYFRAME make_kf(double tx, double ty, double tz)
{
  KEYFRAME kf;
  kf.Rn2c.eye();
  kf.t_c2n[0] = tx;
  kf.t_c2n[1] = ty;
  kf.t_c2n[2] = tz;
  return kf;
}

cv::Point2f project_distorted(const arma::vec::fixed<3> & pt_n, const KEYFRAME & kf, const CAM & cam)
{
  arma::vec::fixed<3> pc = kf.Rn2c * (pt_n - kf.t_c2n);
  arma::mat::fixed<2, 3> J;
  cv::Point2d uv = Projection_model(pc, 0, false, cam, J);
  return cv::Point2f(static_cast<float>(uv.x), static_cast<float>(uv.y));
}

}  // namespace

TEST(BirthReprojGate, DisabledAlwaysPasses)
{
  EXPECT_TRUE(gmap_birth_reproj::passes_gate(false, 1e9, 20.0));
  EXPECT_TRUE(gmap_birth_reproj::passes_gate(false, gmap_birth_reproj::kNaN, 20.0));
}

TEST(BirthReprojGate, EnabledRejectsNonfinite)
{
  EXPECT_FALSE(gmap_birth_reproj::passes_gate(true, gmap_birth_reproj::kNaN, 20.0));
  EXPECT_FALSE(gmap_birth_reproj::passes_gate(true, std::numeric_limits<double>::infinity(), 20.0));
}

TEST(BirthReprojGate, EnabledBoundary)
{
  EXPECT_TRUE(gmap_birth_reproj::passes_gate(true, 20.0, 20.0));
  EXPECT_FALSE(gmap_birth_reproj::passes_gate(true, 20.0001, 20.0));
  EXPECT_TRUE(gmap_birth_reproj::passes_gate(true, 19.999, 20.0));
}

TEST(BirthReprojGate, ConsistentObservationsNearZeroRms)
{
  CAM cam = make_cam_d0();
  KEYFRAME kf0 = make_kf(0, 0, 0);
  KEYFRAME kf1 = make_kf(0.5, 0, 0);
  arma::vec::fixed<3> pt_n = {0.25, 0.1, 4.0};
  cv::Point2f uv0 = project_distorted(pt_n, kf0, cam);
  cv::Point2f uv1 = project_distorted(pt_n, kf1, cam);

  const double rms = gmap_birth_reproj::two_view_reproj_rms_px(
    pt_n, kf0, kf1, uv0, uv1, cam);
  ASSERT_TRUE(std::isfinite(rms));
  EXPECT_LT(rms, 1e-6);
  EXPECT_TRUE(gmap_birth_reproj::passes_gate(true, rms, 20.0));
}

TEST(BirthReprojGate, PerturbedObservationExceedsThreshold)
{
  CAM cam = make_cam_d0();
  KEYFRAME kf0 = make_kf(0, 0, 0);
  KEYFRAME kf1 = make_kf(0.5, 0, 0);
  arma::vec::fixed<3> pt_n = {0.25, 0.1, 4.0};
  cv::Point2f uv0 = project_distorted(pt_n, kf0, cam);
  cv::Point2f uv1 = project_distorted(pt_n, kf1, cam);
  uv1.x += 40.0f;  // large pixel error in one view

  const double rms = gmap_birth_reproj::two_view_reproj_rms_px(
    pt_n, kf0, kf1, uv0, uv1, cam);
  ASSERT_TRUE(std::isfinite(rms));
  EXPECT_GT(rms, 20.0);
  EXPECT_FALSE(gmap_birth_reproj::passes_gate(true, rms, 20.0));
}

TEST(BirthReprojGate, BehindCameraYieldsNonfinite)
{
  CAM cam = make_cam_d0();
  KEYFRAME kf0 = make_kf(0, 0, 0);
  KEYFRAME kf1 = make_kf(0.5, 0, 0);
  arma::vec::fixed<3> pt_n = {0.0, 0.0, -1.0};  // behind both cameras
  cv::Point2f uv0(200, 120);
  cv::Point2f uv1(200, 120);
  const double rms = gmap_birth_reproj::two_view_reproj_rms_px(
    pt_n, kf0, kf1, uv0, uv1, cam);
  EXPECT_FALSE(std::isfinite(rms));
  EXPECT_FALSE(gmap_birth_reproj::passes_gate(true, rms, 20.0));
}

TEST(BirthReprojGate, RejectLeavesNoPartialStateContract)
{
  // Documentary: Init_new_map_points must `continue` before AnchorsDATA /
  // Idx_Matched_points mutation when the gate rejects. Enforced by code review
  // + integration A/B; this unit checks decision-only reject path.
  EXPECT_FALSE(gmap_birth_reproj::passes_gate(true, 25.0, 20.0));
}
