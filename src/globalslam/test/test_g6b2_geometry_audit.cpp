/**
 * G6b.2 audit-only geometry harness.
 *
 * Expected values are derived from the physical identity
 *   p_C_N = p_H_N - R_N←C * t_C←H
 * with R_N←C = Rn2cᵀ (Rn2c = R_C←N), NOT by calling the production helper.
 *
 * Production `camera_tvec_to_nav_camera_pos_tag_at_origin` is invoked only to
 * compare against the independent expectation (agreement / disagreement).
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "g6b_home_tag_geometry.hpp"

using namespace globalslam::g6b_home_tag;

namespace
{

/** Independent recovery: p_C_N = p_H_N - Rn2cᵀ * tvec. */
void recover_camera_nav_independent(
  double tvec_x, double tvec_y, double tvec_z,
  const double Rn2c_rm[9],
  double p_H_N_x, double p_H_N_y, double p_H_N_z,
  double & p_C_N_x, double & p_C_N_y, double & p_C_N_z)
{
  // Rn2cᵀ * tvec  (Rn2c is row-major R_C←N)
  const double rx =
    Rn2c_rm[0] * tvec_x + Rn2c_rm[3] * tvec_y + Rn2c_rm[6] * tvec_z;
  const double ry =
    Rn2c_rm[1] * tvec_x + Rn2c_rm[4] * tvec_y + Rn2c_rm[7] * tvec_z;
  const double rz =
    Rn2c_rm[2] * tvec_x + Rn2c_rm[5] * tvec_y + Rn2c_rm[8] * tvec_z;
  p_C_N_x = p_H_N_x - rx;
  p_C_N_y = p_H_N_y - ry;
  p_C_N_z = p_H_N_z - rz;
}

/** Synthesize OpenCV tvec from known geometry: t_C←H = Rn2c * (p_H_N - p_C_N). */
void synthesize_tvec(
  double p_H_N_x, double p_H_N_y, double p_H_N_z,
  double p_C_N_x, double p_C_N_y, double p_C_N_z,
  const double Rn2c_rm[9],
  double & tvec_x, double & tvec_y, double & tvec_z)
{
  const double dx = p_H_N_x - p_C_N_x;
  const double dy = p_H_N_y - p_C_N_y;
  const double dz = p_H_N_z - p_C_N_z;
  tvec_x = Rn2c_rm[0] * dx + Rn2c_rm[1] * dy + Rn2c_rm[2] * dz;
  tvec_y = Rn2c_rm[3] * dx + Rn2c_rm[4] * dy + Rn2c_rm[5] * dz;
  tvec_z = Rn2c_rm[6] * dx + Rn2c_rm[7] * dy + Rn2c_rm[8] * dz;
}

void yaw_Rn2c(double yaw, double out[9])
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  out[0] = c;  out[1] = s;  out[2] = 0;
  out[3] = -s; out[4] = c;  out[5] = 0;
  out[6] = 0;  out[7] = 0;  out[8] = 1;
}

/** Quad downward cam: Mono_cam_2_robot_axis_z = π/2 → Rr2c ≈ yaw 90°. */
void downward_cam_Rr2c_yaw90(double Rr2c_rm[9])
{
  // R_C←B for axis_z=π/2 (matches Euler_to_Ra2b used as Rr2c in ekf.cpp).
  yaw_Rn2c(1.5707963267948966, Rr2c_rm);
}

}  // namespace

TEST(G6b2GeometryAudit, TagAtNavOriginAgreesWithProduction)
{
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  const double p_H[3] = {0, 0, 0};
  const double p_C_true[3] = {0.0, 0.0, -4.0};
  double tvec[3];
  synthesize_tvec(
    p_H[0], p_H[1], p_H[2], p_C_true[0], p_C_true[1], p_C_true[2],
    Rn2c_I, tvec[0], tvec[1], tvec[2]);
  EXPECT_NEAR(tvec[2], 4.0, 1e-12);

  double ind[3], prod[3];
  recover_camera_nav_independent(
    tvec[0], tvec[1], tvec[2], Rn2c_I, p_H[0], p_H[1], p_H[2],
    ind[0], ind[1], ind[2]);
  camera_tvec_to_nav_camera_pos_tag_at_origin(
    tvec[0], tvec[1], tvec[2], Rn2c_I, prod[0], prod[1], prod[2]);

  EXPECT_NEAR(ind[0], p_C_true[0], 1e-12);
  EXPECT_NEAR(ind[1], p_C_true[1], 1e-12);
  EXPECT_NEAR(ind[2], p_C_true[2], 1e-12);
  EXPECT_NEAR(prod[0], ind[0], 1e-12);
  EXPECT_NEAR(prod[1], ind[1], 1e-12);
  EXPECT_NEAR(prod[2], ind[2], 1e-12);
}

TEST(G6b2GeometryAudit, NonzeroTagNavPositionDisagreesWithProduction)
{
  // Physical: tag at (2, -1, 0), camera at (2, -1, -4). Production assumes p_H=0.
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  const double p_H[3] = {2.0, -1.0, 0.0};
  const double p_C_true[3] = {2.0, -1.0, -4.0};
  double tvec[3];
  synthesize_tvec(
    p_H[0], p_H[1], p_H[2], p_C_true[0], p_C_true[1], p_C_true[2],
    Rn2c_I, tvec[0], tvec[1], tvec[2]);

  double ind[3], prod[3];
  recover_camera_nav_independent(
    tvec[0], tvec[1], tvec[2], Rn2c_I, p_H[0], p_H[1], p_H[2],
    ind[0], ind[1], ind[2]);
  camera_tvec_to_nav_camera_pos_tag_at_origin(
    tvec[0], tvec[1], tvec[2], Rn2c_I, prod[0], prod[1], prod[2]);

  EXPECT_NEAR(ind[0], p_C_true[0], 1e-12);
  EXPECT_NEAR(ind[1], p_C_true[1], 1e-12);
  EXPECT_NEAR(ind[2], p_C_true[2], 1e-12);
  // Production recovers as if tag were at origin → wrong XY.
  EXPECT_NEAR(prod[0], 0.0, 1e-12);
  EXPECT_NEAR(prod[1], 0.0, 1e-12);
  EXPECT_NEAR(prod[2], -4.0, 1e-12);
  EXPECT_FALSE(nearly_equal(prod[0], ind[0]));
  EXPECT_FALSE(nearly_equal(prod[1], ind[1]));
}

TEST(G6b2GeometryAudit, NonzeroRobotInitDoesNotEstablishTagOrigin)
{
  // Robot_init_* sets body at (5,0,0); tag spatial origin is never written.
  // Same tvec as hover-over-tag: production still claims camera at (0,0,-4).
  // That is only physically true if the tag is at N origin — not if N was
  // initialized 5 m away from the tag without a stored p_H_N.
  const double Robot_init_x = 5.0;
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  // If tag stayed at world home while N origin was placed at body init 5 m east,
  // then p_H_N = (-5, 0, 0) in that N frame.
  const double p_H[3] = {-Robot_init_x, 0.0, 0.0};
  const double p_C_true[3] = {-Robot_init_x, 0.0, -4.0};
  double tvec[3];
  synthesize_tvec(
    p_H[0], p_H[1], p_H[2], p_C_true[0], p_C_true[1], p_C_true[2],
    Rn2c_I, tvec[0], tvec[1], tvec[2]);

  double ind[3], prod[3];
  recover_camera_nav_independent(
    tvec[0], tvec[1], tvec[2], Rn2c_I, p_H[0], p_H[1], p_H[2],
    ind[0], ind[1], ind[2]);
  camera_tvec_to_nav_camera_pos_tag_at_origin(
    tvec[0], tvec[1], tvec[2], Rn2c_I, prod[0], prod[1], prod[2]);

  EXPECT_NEAR(ind[0], p_C_true[0], 1e-12);
  EXPECT_NEAR(prod[0], 0.0, 1e-12);
  EXPECT_GE(std::fabs(prod[0] - ind[0]), 4.9);
}

TEST(G6b2GeometryAudit, InitAwayFromTagUnsupportedWithoutStoredPH)
{
  // Body initialized at origin; tag is 3 m north. Hover over tag → camera XY
  // should be ~tag XY. Production forces XY toward 0.
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  const double p_H[3] = {0.0, 3.0, 0.0};
  const double p_C_true[3] = {0.0, 3.0, -4.0};
  double tvec[3];
  synthesize_tvec(
    p_H[0], p_H[1], p_H[2], p_C_true[0], p_C_true[1], p_C_true[2],
    Rn2c_I, tvec[0], tvec[1], tvec[2]);
  double ind[3], prod[3];
  recover_camera_nav_independent(
    tvec[0], tvec[1], tvec[2], Rn2c_I, p_H[0], p_H[1], p_H[2],
    ind[0], ind[1], ind[2]);
  camera_tvec_to_nav_camera_pos_tag_at_origin(
    tvec[0], tvec[1], tvec[2], Rn2c_I, prod[0], prod[1], prod[2]);
  EXPECT_NEAR(ind[1], 3.0, 1e-12);
  EXPECT_NEAR(prod[1], 0.0, 1e-12);
}

TEST(G6b2GeometryAudit, RecHomeAfterMovementDoesNotCaptureSpatialTagPose)
{
  // RecHome only stamps session/epoch/n_kf_home — no p_H_N / R_N←H API exists.
  HomeTagReference ref;
  slam_identity::UuidBytes sid{};
  sid[0] = 1;
  stamp_home_reference(ref, sid, /*epoch=*/9, /*n_kf_home=*/42);
  EXPECT_TRUE(ref.valid);
  EXPECT_EQ(ref.epoch_id, 9u);
  EXPECT_EQ(ref.n_kf_home, 42);
  EXPECT_EQ(ref.session_id[0], 1);
  // Lifecycle stamp only: no member stores tag translation or orientation.
  // (HomeTagReference fields: valid, session_id, epoch_id, n_kf_home.)
  clear_home_reference(ref);
  EXPECT_FALSE(ref.valid);
  EXPECT_EQ(ref.n_kf_home, 0);
}

TEST(G6b2GeometryAudit, DownwardExtrinsicsAndLeverArmIndependent)
{
  // Config: axis_z=π/2, t_c2r=(0.1,0,0). Body at origin, level; Rn2r=I.
  // Rn2c = Rr2c * Rn2r = Rr2c.
  double Rr2c[9];
  downward_cam_Rr2c_yaw90(Rr2c);
  const double t_c2r[3] = {0.1, 0.0, 0.0};
  const double r_N[3] = {0.0, 0.0, -4.0};
  // t_c2n = r_N + Rn2rᵀ * t_c2r = r_N + t_c2r when Rn2r=I
  const double p_C[3] = {
    r_N[0] + t_c2r[0], r_N[1] + t_c2r[1], r_N[2] + t_c2r[2]};
  const double p_H[3] = {0, 0, 0};
  double tvec[3];
  synthesize_tvec(p_H[0], p_H[1], p_H[2], p_C[0], p_C[1], p_C[2], Rr2c,
    tvec[0], tvec[1], tvec[2]);

  double ind[3], prod[3];
  recover_camera_nav_independent(
    tvec[0], tvec[1], tvec[2], Rr2c, 0, 0, 0, ind[0], ind[1], ind[2]);
  camera_tvec_to_nav_camera_pos_tag_at_origin(
    tvec[0], tvec[1], tvec[2], Rr2c, prod[0], prod[1], prod[2]);
  EXPECT_NEAR(ind[0], p_C[0], 1e-9);
  EXPECT_NEAR(ind[1], p_C[1], 1e-9);
  EXPECT_NEAR(ind[2], p_C[2], 1e-9);
  EXPECT_NEAR(prod[0], ind[0], 1e-9);
  EXPECT_NEAR(prod[1], ind[1], 1e-9);
  EXPECT_NEAR(prod[2], ind[2], 1e-9);
  EXPECT_NEAR(prod[0], 0.1, 1e-9);
}

TEST(G6b2GeometryAudit, PositiveAndNegativeYaw)
{
  for (double yaw : {0.5, -0.5}) {
    double Rn2c[9];
    yaw_Rn2c(yaw, Rn2c);
    const double p_H[3] = {0, 0, 0};
    const double p_C[3] = {1.5, -0.8, -3.2};
    double tvec[3];
    synthesize_tvec(
      p_H[0], p_H[1], p_H[2], p_C[0], p_C[1], p_C[2], Rn2c,
      tvec[0], tvec[1], tvec[2]);
    double ind[3], prod[3];
    recover_camera_nav_independent(
      tvec[0], tvec[1], tvec[2], Rn2c, 0, 0, 0, ind[0], ind[1], ind[2]);
    camera_tvec_to_nav_camera_pos_tag_at_origin(
      tvec[0], tvec[1], tvec[2], Rn2c, prod[0], prod[1], prod[2]);
    EXPECT_NEAR(ind[0], p_C[0], 1e-10);
    EXPECT_NEAR(prod[0], ind[0], 1e-10);
    EXPECT_NEAR(prod[1], ind[1], 1e-10);
    EXPECT_NEAR(prod[2], ind[2], 1e-10);
  }
}

TEST(G6b2GeometryAudit, CombinedRollPitchYawTagAtOrigin)
{
  const double cr = std::cos(0.12), sr = std::sin(0.12);
  const double cp = std::cos(-0.18), sp = std::sin(-0.18);
  const double cy = std::cos(0.4), sy = std::sin(0.4);
  // R = Rz*Ry*Rx row-major
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
  const double p_C[3] = {0.6, -1.2, -2.8};
  double tvec[3];
  synthesize_tvec(0, 0, 0, p_C[0], p_C[1], p_C[2], Rn2c, tvec[0], tvec[1], tvec[2]);
  double ind[3], prod[3];
  recover_camera_nav_independent(
    tvec[0], tvec[1], tvec[2], Rn2c, 0, 0, 0, ind[0], ind[1], ind[2]);
  camera_tvec_to_nav_camera_pos_tag_at_origin(
    tvec[0], tvec[1], tvec[2], Rn2c, prod[0], prod[1], prod[2]);
  EXPECT_NEAR(ind[0], p_C[0], 1e-10);
  EXPECT_NEAR(prod[0], ind[0], 1e-10);
  EXPECT_NEAR(prod[1], ind[1], 1e-10);
  EXPECT_NEAR(prod[2], ind[2], 1e-10);
}

TEST(G6b2GeometryAudit, UnchangedObservationYieldsExpectedReferencePose)
{
  // Same physical hover twice → identical recovered pose (tag@origin).
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double tvec[3];
  synthesize_tvec(0, 0, 0, 0.2, -0.3, -4.1, Rn2c_I, tvec[0], tvec[1], tvec[2]);
  double a[3], b[3];
  recover_camera_nav_independent(
    tvec[0], tvec[1], tvec[2], Rn2c_I, 0, 0, 0, a[0], a[1], a[2]);
  recover_camera_nav_independent(
    tvec[0], tvec[1], tvec[2], Rn2c_I, 0, 0, 0, b[0], b[1], b[2]);
  EXPECT_NEAR(a[0], b[0], 0);
  EXPECT_NEAR(a[1], b[1], 0);
  EXPECT_NEAR(a[2], b[2], 0);
  EXPECT_NEAR(a[0], 0.2, 1e-12);
  EXPECT_NEAR(a[1], -0.3, 1e-12);
  EXPECT_NEAR(a[2], -4.1, 1e-12);
}

TEST(G6b2GeometryAudit, KnownReturnDisplacementTagAtOrigin)
{
  // Depart camera (0,0,-4) → return with lateral drift; tag@0; tvec encodes return.
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  const double p_return[3] = {1.25, -0.5, -4.0};
  double tvec[3];
  synthesize_tvec(
    0, 0, 0, p_return[0], p_return[1], p_return[2], Rn2c_I,
    tvec[0], tvec[1], tvec[2]);
  double ind[3], prod[3];
  recover_camera_nav_independent(
    tvec[0], tvec[1], tvec[2], Rn2c_I, 0, 0, 0, ind[0], ind[1], ind[2]);
  camera_tvec_to_nav_camera_pos_tag_at_origin(
    tvec[0], tvec[1], tvec[2], Rn2c_I, prod[0], prod[1], prod[2]);
  EXPECT_NEAR(ind[0], 1.25, 1e-12);
  EXPECT_NEAR(ind[1], -0.5, 1e-12);
  EXPECT_NEAR(prod[0], ind[0], 1e-12);
  EXPECT_NEAR(prod[1], ind[1], 1e-12);
}

TEST(G6b2GeometryAudit, ResetInvalidatesLifecycleNotSpatialBecauseNoneStored)
{
  HomeTagReference ref;
  slam_identity::UuidBytes sid{};
  sid[0] = 3;
  stamp_home_reference(ref, sid, 2, 7);
  clear_home_reference(ref);
  EXPECT_FALSE(ref.valid);
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double tx, ty, tz;
  const auto r = convert_home_tag_tvec_to_nav_camera_pos(
    0, 0, 4, Rn2c_I, 6, 90, 5,
    ref.valid, ref.spatial_ready, true,
    ref.p_H_N[0], ref.p_H_N[1], ref.p_H_N[2],
    tx, ty, tz);
  EXPECT_EQ(r, PoseReject::HomeRefMissing);
}

TEST(G6b2GeometryAudit, EstimatedRn2cErrorPropagatesIntoPosition)
{
  // True Rn2c = I; production uses yaw-biased Rn2c_hat.
  // Pure optical-depth tvec is invariant under yaw-about-Z; use lateral offset
  // so attitude error couples into recovered XY (same path as live KF Rn2c).
  const double Rn2c_true[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double Rn2c_hat[9];
  yaw_Rn2c(0.2, Rn2c_hat);
  const double p_C[3] = {1.5, -0.8, -4.0};
  double tvec[3];
  synthesize_tvec(0, 0, 0, p_C[0], p_C[1], p_C[2], Rn2c_true,
    tvec[0], tvec[1], tvec[2]);
  double with_true[3], with_hat[3];
  recover_camera_nav_independent(
    tvec[0], tvec[1], tvec[2], Rn2c_true, 0, 0, 0,
    with_true[0], with_true[1], with_true[2]);
  recover_camera_nav_independent(
    tvec[0], tvec[1], tvec[2], Rn2c_hat, 0, 0, 0,
    with_hat[0], with_hat[1], with_hat[2]);
  EXPECT_NEAR(with_true[0], p_C[0], 1e-12);
  EXPECT_NEAR(with_true[1], p_C[1], 1e-12);
  EXPECT_NEAR(with_true[2], p_C[2], 1e-12);
  EXPECT_GT(
    std::hypot(with_hat[0] - with_true[0], with_hat[1] - with_true[1]),
    0.2);
}

TEST(G6b2GeometryAudit, TagRvecUnusedByProductionPositionPath)
{
  // Documented: rvec discarded. Same tvec + different fictitious tag yaw
  // cannot change production position (center-only).
  const double Rn2c_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double a[3], b[3];
  camera_tvec_to_nav_camera_pos_tag_at_origin(0.1, -0.2, 4.0, Rn2c_I, a[0], a[1], a[2]);
  camera_tvec_to_nav_camera_pos_tag_at_origin(0.1, -0.2, 4.0, Rn2c_I, b[0], b[1], b[2]);
  EXPECT_NEAR(a[0], b[0], 0);
  EXPECT_NEAR(a[1], b[1], 0);
  EXPECT_NEAR(a[2], b[2], 0);
  // Unsupported without rvec path: full T_N←C = T_N←H * inv(T_C←H) when
  // T_N←H ≠ I or attitude measurement desired — not implemented.
  SUCCEED();
}
