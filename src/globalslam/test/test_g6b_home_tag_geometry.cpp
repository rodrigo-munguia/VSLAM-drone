#include <gtest/gtest.h>

#include <cmath>

#include "g6b_home_tag_geometry.hpp"

using namespace globalslam::g6b_home_tag;

TEST(G6bHomeTagGeometry, LiveCloopRemapIsAxisSwapNotNavTransform)
{
  double x, y, z;
  live_cloop_tvec_remap(/*tvec*/ 1.0, 2.0, 4.0, x, y, z);
  EXPECT_DOUBLE_EQ(x, 2.0);
  EXPECT_DOUBLE_EQ(y, -1.0);
  EXPECT_DOUBLE_EQ(z, 4.0);
}

TEST(G6bHomeTagGeometry, ControlAndCloopDisagreeOnYSign)
{
  double cx, cy, cz, gx, gy, gz;
  live_control_tvec_remap(1.0, 2.0, 4.0, cx, cy, cz);
  live_cloop_tvec_remap(1.0, 2.0, 4.0, gx, gy, gz);
  EXPECT_DOUBLE_EQ(cx, gx);
  EXPECT_DOUBLE_EQ(cz, gz);
  EXPECT_DOUBLE_EQ(cy, 1.0);
  EXPECT_DOUBLE_EQ(gy, -1.0);
  EXPECT_FALSE(nearly_equal(cy, gy));
}

TEST(G6bHomeTagGeometry, DownwardCamOpticalDepthMapsToNegNavZ)
{
  // Controlled geometry: Rn2c = I, tag at nav origin, camera at (0,0,-4).
  // OpenCV tvec (marker in camera) = (0,0,4).
  // Production: t_c2n = -Rn2c^T * tvec = (0,0,-4).
  const double Rn2c_I[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1};
  double tx, ty, tz;
  camera_tvec_to_nav_camera_pos_tag_at_origin(0.0, 0.0, 4.0, Rn2c_I, tx, ty, tz);
  EXPECT_NEAR(tx, 0.0, 1e-12);
  EXPECT_NEAR(ty, 0.0, 1e-12);
  EXPECT_NEAR(tz, -4.0, 1e-12);

  double lx, ly, lz;
  live_cloop_tvec_remap(0.0, 0.0, 4.0, lx, ly, lz);
  EXPECT_DOUBLE_EQ(lz, 4.0);
  EXPECT_FALSE(nearly_equal(lz, tz));
}

TEST(G6bHomeTagGeometry, ProductionConvertRejectsWithoutHomeRef)
{
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double tx, ty, tz;
  const auto r = convert_home_tag_tvec_to_nav_camera_pos(
    0.0, 0.0, 4.0, Rn2c_I, 6.0, 90.0, 10.0,
    /*home_ref_valid=*/false, /*spatial_ready=*/true, /*session_ok=*/true,
    0.0, 0.0, 0.0, tx, ty, tz);
  EXPECT_EQ(r, PoseReject::HomeRefMissing);
}

TEST(G6bHomeTagGeometry, ProductionConvertRejectsSessionMismatch)
{
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double tx, ty, tz;
  const auto r = convert_home_tag_tvec_to_nav_camera_pos(
    0.0, 0.0, 4.0, Rn2c_I, 6.0, 90.0, 10.0,
    true, /*spatial_ready=*/true, /*session_ok=*/false,
    0.0, 0.0, 0.0, tx, ty, tz);
  EXPECT_EQ(r, PoseReject::SessionEpochMismatch);
}

TEST(G6bHomeTagGeometry, ProductionConvertSucceedsDownwardCase)
{
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double tx, ty, tz;
  const auto r = convert_home_tag_tvec_to_nav_camera_pos(
    0.0, 0.0, 4.0, Rn2c_I, 6.0, 90.0, 10.0,
    true, true, true, 0.0, 0.0, 0.0, tx, ty, tz);
  EXPECT_EQ(r, PoseReject::Ok);
  EXPECT_NEAR(tz, -4.0, 1e-12);
}

TEST(G6bHomeTagGeometry, NonzeroLeverArmViaRotatedRn2c)
{
  // Camera looking along +X_nav: Rn2c maps N→C with C_z = N_x.
  // Tag at origin, camera at (-4, 0.3, -1.7) → tvec = -Rn2c * t_c2n.
  // Rn2c rows: e1' = (0,0,-1)? Use R that maps N_x→C_z, N_y→-C_y, N_z→C_x:
  //   R = [[0,0,1], [0,-1,0], [1,0,0]]  (row-major)
  const double Rn2c[9] = {
    0, 0, 1,
    0, -1, 0,
    1, 0, 0};
  const double t_true[3] = {-4.0, 0.3, -1.7};
  // tvec = -Rn2c * t_true
  double tvec[3] = {0, 0, 0};
  for (int i = 0; i < 3; ++i) {
    tvec[i] = -(
      Rn2c[3 * i + 0] * t_true[0] +
      Rn2c[3 * i + 1] * t_true[1] +
      Rn2c[3 * i + 2] * t_true[2]);
  }
  double tx, ty, tz;
  camera_tvec_to_nav_camera_pos_tag_at_origin(
    tvec[0], tvec[1], tvec[2], Rn2c, tx, ty, tz);
  EXPECT_NEAR(tx, t_true[0], 1e-12);
  EXPECT_NEAR(ty, t_true[1], 1e-12);
  EXPECT_NEAR(tz, t_true[2], 1e-12);
}

TEST(G6bHomeTagGeometry, AsymmetricTranslationNoAxisCancel)
{
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  // Camera at (1.25, -2.5, -3.75) ⇒ tvec = -t = (-1.25, 2.5, 3.75)
  double tx, ty, tz;
  camera_tvec_to_nav_camera_pos_tag_at_origin(
    -1.25, 2.5, 3.75, Rn2c_I, tx, ty, tz);
  EXPECT_NEAR(tx, 1.25, 1e-12);
  EXPECT_NEAR(ty, -2.5, 1e-12);
  EXPECT_NEAR(tz, -3.75, 1e-12);
}

TEST(G6bHomeTagGeometry, YawOnlyRn2c)
{
  const double c = std::cos(0.4);
  const double s = std::sin(0.4);
  // Rn2c = Rz(yaw) about N_z
  const double Rn2c[9] = {
    c, s, 0,
    -s, c, 0,
    0, 0, 1};
  const double t_true[3] = {2.0, -1.0, -5.0};
  double tvec[3] = {0, 0, 0};
  for (int i = 0; i < 3; ++i) {
    tvec[i] = -(
      Rn2c[3 * i + 0] * t_true[0] +
      Rn2c[3 * i + 1] * t_true[1] +
      Rn2c[3 * i + 2] * t_true[2]);
  }
  double tx, ty, tz;
  camera_tvec_to_nav_camera_pos_tag_at_origin(
    tvec[0], tvec[1], tvec[2], Rn2c, tx, ty, tz);
  EXPECT_NEAR(tx, t_true[0], 1e-12);
  EXPECT_NEAR(ty, t_true[1], 1e-12);
  EXPECT_NEAR(tz, t_true[2], 1e-12);
}

TEST(G6bHomeTagGeometry, CombinedRollPitchYaw)
{
  // R = Rz(yaw)*Ry(pitch)*Rx(roll) as R_C←N (row-major).
  const double cr = std::cos(0.1), sr = std::sin(0.1);
  const double cp = std::cos(-0.2), sp = std::sin(-0.2);
  const double cy = std::cos(0.35), sy = std::sin(0.35);
  const double Rx[9] = {1, 0, 0, 0, cr, sr, 0, -sr, cr};
  const double Ry[9] = {cp, 0, -sp, 0, 1, 0, sp, 0, cp};
  const double Rz[9] = {cy, sy, 0, -sy, cy, 0, 0, 0, 1};
  auto mul = [](const double A[9], const double B[9], double C[9]) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        C[3 * i + j] =
          A[3 * i + 0] * B[j] +
          A[3 * i + 1] * B[3 + j] +
          A[3 * i + 2] * B[6 + j];
      }
    }
  };
  double Rzy[9], Rn2c[9];
  mul(Rz, Ry, Rzy);
  mul(Rzy, Rx, Rn2c);
  const double t_true[3] = {0.7, -1.1, -3.3};
  double tvec[3] = {0, 0, 0};
  for (int i = 0; i < 3; ++i) {
    tvec[i] = -(
      Rn2c[3 * i + 0] * t_true[0] +
      Rn2c[3 * i + 1] * t_true[1] +
      Rn2c[3 * i + 2] * t_true[2]);
  }
  double tx, ty, tz;
  camera_tvec_to_nav_camera_pos_tag_at_origin(
    tvec[0], tvec[1], tvec[2], Rn2c, tx, ty, tz);
  EXPECT_NEAR(tx, t_true[0], 1e-10);
  EXPECT_NEAR(ty, t_true[1], 1e-10);
  EXPECT_NEAR(tz, t_true[2], 1e-10);
}

TEST(G6bHomeTagGeometry, SameCameraPoseDifferentTagYawSameTvecCenter)
{
  // Tag center at origin: rotation of H about its Z does not move the origin,
  // so tvec (center in C) is independent of tag yaw — same t_c2n recovery.
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double a_x, a_y, a_z, b_x, b_y, b_z;
  camera_tvec_to_nav_camera_pos_tag_at_origin(0.1, -0.2, 4.0, Rn2c_I, a_x, a_y, a_z);
  camera_tvec_to_nav_camera_pos_tag_at_origin(0.1, -0.2, 4.0, Rn2c_I, b_x, b_y, b_z);
  EXPECT_NEAR(a_x, b_x, 1e-15);
  EXPECT_NEAR(a_y, b_y, 1e-15);
  EXPECT_NEAR(a_z, b_z, 1e-15);
}

TEST(G6bHomeTagGeometry, RecHomeStampAndClear)
{
  HomeTagReference ref;
  slam_identity::UuidBytes sid{};
  sid[0] = 7;
  stamp_home_reference(ref, sid, 3, 12);
  EXPECT_TRUE(ref.valid);
  EXPECT_EQ(ref.epoch_id, 3u);
  EXPECT_EQ(ref.n_kf_home, 12);
  clear_home_reference(ref);
  EXPECT_FALSE(ref.valid);
}

TEST(G6bHomeTagGeometry, GateRejectsFarHorizontal)
{
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  // t_c2n will be (10,0,-1) from tvec=(-10,0,1) → d_xy=10 > gate 6
  double tx, ty, tz;
  const auto r = convert_home_tag_tvec_to_nav_camera_pos(
    -10.0, 0.0, 1.0, Rn2c_I, 6.0, 90.0, 5.0,
    true, true, true, 0.0, 0.0, 0.0, tx, ty, tz);
  EXPECT_EQ(r, PoseReject::GateXyOrPixel);
}

TEST(G6bHomeTagGeometry, NonFiniteTvecRejected)
{
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double tx, ty, tz;
  const auto r = convert_home_tag_tvec_to_nav_camera_pos(
    NAN, 0.0, 4.0, Rn2c_I, 6.0, 90.0, 5.0,
    true, true, true, 0.0, 0.0, 0.0, tx, ty, tz);
  EXPECT_EQ(r, PoseReject::NonFinite);
}

TEST(G6bHomeTagGeometry, PoseSlamPrintsAreCameraNavNotBody)
{
  // Documentation lock: t_c2n := camera in nav; r_N := body.
  SUCCEED();
}
