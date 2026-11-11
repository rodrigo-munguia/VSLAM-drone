#include <cmath>
#include <gtest/gtest.h>

#include "gmap_relative_kf_init.hpp"

using gmap_relative_kf_init::CamPose;
using gmap_relative_kf_init::compose;
using gmap_relative_kf_init::invert;
using gmap_relative_kf_init::propagate;
using gmap_relative_kf_init::apply_cam_pose;
using gmap_relative_kf_init::from_keyframe;

static CamPose make_pose(double yaw, double x, double y, double z)
{
  CamPose T;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  T.Rc2n.zeros();
  T.Rc2n(0, 0) = c;
  T.Rc2n(0, 1) = -s;
  T.Rc2n(1, 0) = s;
  T.Rc2n(1, 1) = c;
  T.Rc2n(2, 2) = 1.0;
  T.t_c2n(0) = x;
  T.t_c2n(1) = y;
  T.t_c2n(2) = z;
  return T;
}

TEST(RelativeKfInit, InvertComposeIdentity)
{
  CamPose T = make_pose(0.3, 1.0, -2.0, 0.5);
  CamPose I = compose(T, invert(T));
  EXPECT_NEAR(arma::norm(I.t_c2n), 0.0, 1e-9);
  EXPECT_NEAR(arma::norm(I.Rc2n - arma::eye(3, 3)), 0.0, 1e-9);
}

TEST(RelativeKfInit, PropagateMatchesComposition)
{
  // GS prev corrected; EKF absolute drifted.
  CamPose T_gs_prev = make_pose(0.1, 0.0, 0.0, 1.0);
  CamPose T_ekf_prev = make_pose(0.1, 0.2, 0.0, 1.0);
  CamPose T_ekf_curr = make_pose(0.25, 1.2, 0.5, 1.1);
  CamPose T_gs_curr = propagate(T_gs_prev, T_ekf_prev, T_ekf_curr);

  CamPose T_rel = compose(invert(T_ekf_prev), T_ekf_curr);
  CamPose expect = compose(T_gs_prev, T_rel);
  EXPECT_NEAR(arma::norm(T_gs_curr.t_c2n - expect.t_c2n), 0.0, 1e-9);
  EXPECT_NEAR(arma::norm(T_gs_curr.Rc2n - expect.Rc2n), 0.0, 1e-9);
}

TEST(RelativeKfInit, ApplyDerivesBodyFromExtrinsics)
{
  KEYFRAME kf;
  kf.Rr2c = arma::eye(3, 3);
  kf.t_c2r.zeros();
  kf.t_c2r(2) = 0.1;  // camera below body along +z_cam / check formula
  CamPose T = make_pose(0.0, 3.0, 4.0, 5.0);
  apply_cam_pose(kf, T);
  EXPECT_NEAR(kf.t_c2n(0), 3.0, 1e-12);
  EXPECT_NEAR(kf.t_c2n(1), 4.0, 1e-12);
  EXPECT_NEAR(kf.t_c2n(2), 5.0, 1e-12);
  // Rn2r = Rr2c^T * Rn2c = Rn2c when Rr2c=I
  EXPECT_NEAR(arma::norm(kf.Rn2r - kf.Rn2c), 0.0, 1e-12);
  // r_N = t_c2n - Rn2r^T * t_c2r
  arma::vec::fixed<3> r_expect = kf.t_c2n - kf.Rn2r.t() * kf.t_c2r;
  EXPECT_NEAR(arma::norm(kf.r_N - r_expect), 0.0, 1e-12);
  CamPose back = from_keyframe(kf);
  EXPECT_NEAR(arma::norm(back.t_c2n - T.t_c2n), 0.0, 1e-12);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
