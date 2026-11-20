/**
 * GSLAM-VISUAL-LOOP-A2 / A2b — production visual PnP geometry regressions.
 *
 * Exercises globalslam::cloop_visual_pnp::solve_and_accept (used by
 * CLOOP::Get_pos_of_current_kf). Get_matches / ORB retrieval not exercised.
 */

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include <armadillo>

#include "../../common/Vision/vision.hpp"
#include "cloop_visual_pnp.hpp"

using globalslam::cloop_visual_pnp::camera_centre_from_cv;
using globalslam::cloop_visual_pnp::solve_and_accept;
using globalslam::cloop_visual_pnp::t_cv_from_nav;

namespace
{

constexpr double kFx = 200.0;
constexpr double kFy = 200.0;
constexpr double kCx = 200.0;
constexpr double kCy = 120.0;
constexpr double kReprojAcceptPx = 70.0;
constexpr double kXyGateM = 6.0;
constexpr double kRadiusUncM = 2.0;

CAM make_cam(const std::vector<double> & d)
{
  // distortions pointer must outlive CAM uses — static per unique set via leak
  // of one vector copy kept in a static list.
  static std::vector<std::vector<double>> pool;
  pool.push_back(d);
  CAM cam;
  cam.distortions = pool.back().data();
  cam.fc[0] = kFx;
  cam.fc[1] = kFy;
  cam.cc[0] = kCx;
  cam.cc[1] = kCy;
  cam.alpha_c = 0.0;
  return cam;
}

arma::mat::fixed<3, 3> rot_zyx(double yaw, double pitch, double roll)
{
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cr = std::cos(roll), sr = std::sin(roll);
  arma::mat::fixed<3, 3> Rz = {{cy, -sy, 0}, {sy, cy, 0}, {0, 0, 1}};
  arma::mat::fixed<3, 3> Ry = {{cp, 0, sp}, {0, 1, 0}, {-sp, 0, cp}};
  arma::mat::fixed<3, 3> Rx = {{1, 0, 0}, {0, cr, -sr}, {0, sr, cr}};
  return Rz * Ry * Rx;
}

struct PoseNav
{
  arma::mat::fixed<3, 3> Rn2c;
  arma::vec::fixed<3> t_c2n;
};

PoseNav make_gt_pose()
{
  PoseNav p;
  p.Rn2c = rot_zyx(0.26, 0.14, -0.09).t();
  p.t_c2n = {1.2, -0.7, 0.3};
  return p;
}

std::vector<arma::vec::fixed<3>> make_landmarks()
{
  return {
    {1.0, -0.5, 4.0}, {2.5, 0.8, 5.5}, {-1.2, 1.5, 3.5}, {0.3, -1.8, 6.0},
    {3.0, 1.2, 7.0}, {-2.0, -1.0, 4.5}, {1.5, 2.0, 5.0}, {0.0, 0.5, 8.0},
    {2.0, -2.0, 3.8}, {-0.8, 0.2, 6.5}, {1.8, 0.0, 4.2}, {-1.5, -0.7, 5.8},
  };
}

struct Corr
{
  std::vector<cv::Point3f> pts3d;
  std::vector<cv::Point2f> pts2d_undist;
};

Corr project_landmarks(
  const std::vector<arma::vec::fixed<3>> & landmarks_n,
  const PoseNav & pose,
  const CAM & cam)
{
  Corr out;
  arma::mat::fixed<2, 3> J;
  for (const auto & pw : landmarks_n) {
    const arma::vec::fixed<3> pc = pose.Rn2c * (pw - pose.t_c2n);
    const cv::Point2d uvd = Projection_model(pc, 0, false, cam, J);
    if (uvd.x < 5 || uvd.x > 395 || uvd.y < 5 || uvd.y > 235) {
      continue;
    }
    out.pts3d.emplace_back(
      static_cast<float>(pw(0)), static_cast<float>(pw(1)),
      static_cast<float>(pw(2)));
    const cv::Point2d uv = Undistort_a_point(uvd, cam, 1);
    out.pts2d_undist.emplace_back(
      static_cast<float>(uv.x), static_cast<float>(uv.y));
  }
  return out;
}

double rot_error_deg(
  const arma::mat::fixed<3, 3> & Ra, const arma::mat::fixed<3, 3> & Rb)
{
  arma::mat::fixed<3, 3> dR = Ra.t() * Rb;
  double c = 0.5 * (arma::trace(dR) - 1.0);
  c = std::max(-1.0, std::min(1.0, c));
  return std::acos(c) * 180.0 / std::acos(-1.0);
}

void print_result(
  const char * name, const PoseNav & gt,
  const globalslam::cloop_visual_pnp::SolveAcceptResult & r)
{
  std::cout << "\n=== " << name << " ===\n"
            << "  accepted=" << r.accepted
            << " reason=" << r.reject_reason
            << " n_inliers=" << r.n_inliers
            << " mean_reproj=" << r.mean_reprojection_error
            << " d_xy=" << r.d_xy << "\n"
            << "  GT C=[" << gt.t_c2n.t() << "]  C_nav=[" << r.C_nav.t()
            << "]  |ΔC|=" << arma::norm(r.C_nav - gt.t_c2n)
            << "  rot_err_deg=" << rot_error_deg(r.R_cv, gt.Rn2c) << "\n";
}

}  // namespace

TEST(VisualLoopA2b, ConventionHelpers)
{
  const PoseNav gt = make_gt_pose();
  const auto t_cv = t_cv_from_nav(gt.Rn2c, gt.t_c2n);
  const auto C = camera_centre_from_cv(gt.Rn2c, t_cv);
  EXPECT_NEAR(arma::norm(C - gt.t_c2n), 0.0, 1e-12);
  EXPECT_GT(arma::norm(t_cv - gt.t_c2n), 0.5);
}

TEST(VisualLoopA2b, CaseA_CorrectGeometry)
{
  CAM cam = make_cam({0, 0, 0, 0, 0});
  const PoseNav gt = make_gt_pose();
  const Corr corr = project_landmarks(make_landmarks(), gt, cam);
  ASSERT_GE(corr.pts3d.size(), 8u);

  const auto r = solve_and_accept(
    corr.pts3d, corr.pts2d_undist, gt.Rn2c, gt.t_c2n, cam,
    kRadiusUncM, kXyGateM, kReprojAcceptPx);
  print_result("CaseA_CorrectGeometry", gt, r);

  EXPECT_TRUE(r.accepted);
  EXPECT_GE(r.n_inliers, 8);
  EXPECT_LT(arma::norm(r.C_nav - gt.t_c2n), 0.05);
  EXPECT_LT(rot_error_deg(r.R_cv, gt.Rn2c), 1.0);
  EXPECT_LT(r.mean_reprojection_error, 1.0);
  EXPECT_LT(r.d_xy, 0.05);
}

TEST(VisualLoopA2b, CaseB_KfOrientationDiscrepancy)
{
  CAM cam = make_cam({0, 0, 0, 0, 0});
  const PoseNav gt = make_gt_pose();
  const Corr corr = project_landmarks(make_landmarks(), gt, cam);
  ASSERT_GE(corr.pts3d.size(), 8u);

  PoseNav kf_wrong = gt;
  kf_wrong.Rn2c = rot_zyx(0.26 + 0.44, 0.14, -0.09).t();
  kf_wrong.t_c2n = gt.t_c2n + arma::vec::fixed<3>({0.1, -0.05, 0.0});

  const auto r = solve_and_accept(
    corr.pts3d, corr.pts2d_undist, kf_wrong.Rn2c, kf_wrong.t_c2n, cam,
    kRadiusUncM, kXyGateM, kReprojAcceptPx);
  print_result("CaseB_KfOrientationDiscrepancy", gt, r);

  // Gate uses PnP R,t — wrong KF attitude must not poison acceptance.
  EXPECT_TRUE(r.accepted);
  EXPECT_LT(arma::norm(r.C_nav - gt.t_c2n), 0.05);
  EXPECT_LT(rot_error_deg(r.R_cv, gt.Rn2c), 1.0);
  EXPECT_LT(r.mean_reprojection_error, 1.0);
}

TEST(VisualLoopA2b, CaseC_IncorrectCorrespondences)
{
  CAM cam = make_cam({0, 0, 0, 0, 0});
  const PoseNav gt = make_gt_pose();
  Corr corr = project_landmarks(make_landmarks(), gt, cam);
  ASSERT_GE(corr.pts3d.size(), 8u);

  // Derange 2D observations (half-range reverse) then offset every point so
  // no correspondence remains consistent with GT.
  const size_t n = corr.pts2d_undist.size();
  for (size_t i = 0; i < n / 2; ++i) {
    std::swap(corr.pts2d_undist[i], corr.pts2d_undist[n - 1 - i]);
  }
  for (size_t i = 0; i < n; ++i) {
    corr.pts2d_undist[i].x += 80.f + 15.f * static_cast<float>(i % 3);
    corr.pts2d_undist[i].y -= 55.f + 10.f * static_cast<float>(i % 2);
  }

  const auto r = solve_and_accept(
    corr.pts3d, corr.pts2d_undist, gt.Rn2c, gt.t_c2n, cam,
    kRadiusUncM, kXyGateM, kReprojAcceptPx);
  print_result("CaseC_IncorrectCorrespondences", gt, r);

  // Correct geometry must not accept a pose from destroyed correspondences,
  // or if RANSAC still returns something under the warm seed, it must fail
  // the 70 px gate / not land on GT.
  if (r.accepted) {
    EXPECT_GT(arma::norm(r.C_nav - gt.t_c2n), 0.2)
      << "Accepted pose with destroyed matches must not equal GT";
    EXPECT_GT(r.mean_reprojection_error, kReprojAcceptPx);
  } else {
    EXPECT_TRUE(
      std::string(r.reject_reason) == "gate_failed" ||
      std::string(r.reject_reason) == "empty_inliers" ||
      std::string(r.reject_reason) == "no_valid_inliers" ||
      std::string(r.reject_reason) == "reproj_behind_camera");
  }
}

TEST(VisualLoopA2b, CaseD_TvecVsCameraCentre)
{
  CAM cam = make_cam({0, 0, 0, 0, 0});
  const PoseNav gt = make_gt_pose();
  const auto t_cv = t_cv_from_nav(gt.Rn2c, gt.t_c2n);
  EXPECT_GT(arma::norm(t_cv - gt.t_c2n), 0.5);

  const Corr corr = project_landmarks(make_landmarks(), gt, cam);
  ASSERT_GE(corr.pts3d.size(), 8u);

  PoseNav far_seed = gt;
  far_seed.t_c2n = {5.0, 5.0, 5.0};
  const auto r = solve_and_accept(
    corr.pts3d, corr.pts2d_undist, far_seed.Rn2c, far_seed.t_c2n, cam,
    /*radius*/20.0, /*xy*/20.0, kReprojAcceptPx);
  print_result("CaseD_FarSeed", gt, r);

  EXPECT_TRUE(r.accepted);
  EXPECT_LT(arma::norm(r.C_nav - gt.t_c2n), 0.05);
  // Recovered centre must not equal OpenCV t_cv of GT.
  EXPECT_GT(arma::norm(r.C_nav - t_cv), 0.5);
}

TEST(VisualLoopA2b, CaseE_NonzeroDistortion)
{
  // Mild radial distortion supported by Undistort/Projection_model (Bouguet).
  CAM cam = make_cam({-0.12, 0.04, 0.0, 0.0, 0.0});
  const PoseNav gt = make_gt_pose();
  const Corr corr = project_landmarks(make_landmarks(), gt, cam);
  ASSERT_GE(corr.pts3d.size(), 8u);

  const auto r = solve_and_accept(
    corr.pts3d, corr.pts2d_undist, gt.Rn2c, gt.t_c2n, cam,
    kRadiusUncM, kXyGateM, kReprojAcceptPx);
  print_result("CaseE_NonzeroDistortion", gt, r);

  EXPECT_TRUE(r.accepted);
  EXPECT_LT(arma::norm(r.C_nav - gt.t_c2n), 0.08);
  EXPECT_LT(rot_error_deg(r.R_cv, gt.Rn2c), 2.0);
  EXPECT_LT(r.mean_reprojection_error, 5.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
