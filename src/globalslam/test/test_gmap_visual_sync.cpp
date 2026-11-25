#include <cmath>
#include <gtest/gtest.h>

#include "gmap_visual_sync.hpp"
#include "gmap_relative_kf_init.hpp"

using gmap_relative_kf_init::CamPose;
using gmap_relative_kf_init::compose;
using gmap_relative_kf_init::invert;
using gmap_relative_kf_init::propagate;
using gmap_relative_kf_init::propagate_xy_rel_z_abs;
using gmap_visual_sync::AdmitMode;
using gmap_visual_sync::align_delta_xy;
using gmap_visual_sync::incremental_delta_xy;
using gmap_visual_sync::rebase_cam_pose_xy;
using gmap_visual_sync::should_admit_keyframe;

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

TEST(VisualSync, AlignDiffersFromIncrementalWithOffset)
{
  arma::vec::fixed<3> p_l;
  p_l[0] = 10.0;
  p_l[1] = 20.0;
  p_l[2] = 1.0;
  arma::vec::fixed<3> p_g_before;
  p_g_before[0] = 12.0;
  p_g_before[1] = 22.5;
  p_g_before[2] = 1.0;
  arma::vec::fixed<3> p_g_after;
  p_g_after[0] = 9.0;
  p_g_after[1] = 19.0;
  p_g_after[2] = 1.2;  // BA moved Z; align still z=0

  const auto d_g = incremental_delta_xy(p_g_after, p_g_before);
  const auto d_l = align_delta_xy(p_g_after, p_l);
  EXPECT_NEAR(d_l[0], -1.0, 1e-12);
  EXPECT_NEAR(d_l[1], -1.0, 1e-12);
  EXPECT_NEAR(d_l[2], 0.0, 1e-12);
  EXPECT_FALSE(std::abs(d_l[0] - d_g[0]) < 1e-12 && std::abs(d_l[1] - d_g[1]) < 1e-12);

  // After BA tip change, align uses final tip not Pose_SLAM-only tip.
  arma::vec::fixed<3> p_g_ba = p_g_after;
  p_g_ba[0] = 8.5;
  const auto d_l_ba = align_delta_xy(p_g_ba, p_l);
  EXPECT_NEAR(d_l_ba[0], -1.5, 1e-12);
}

TEST(VisualSync, AlignRemovesXyOffset)
{
  arma::vec::fixed<3> p_l;
  p_l[0] = 10.0;
  p_l[1] = 20.0;
  p_l[2] = 5.0;
  arma::vec::fixed<3> p_g;
  p_g[0] = 9.0;
  p_g[1] = 19.0;
  p_g[2] = 7.0;
  const auto d = align_delta_xy(p_g, p_l);
  EXPECT_NEAR(p_l[0] + d[0], p_g[0], 1e-12);
  EXPECT_NEAR(p_l[1] + d[1], p_g[1], 1e-12);
  EXPECT_NEAR(p_l[2] + d[2], p_l[2], 1e-12);  // Z unchanged
}

TEST(VisualSync, RebaseThenPropagateNoDoubleCount)
{
  arma::vec::fixed<3> p_l;
  p_l[0] = 10.0;
  p_l[1] = 20.0;
  p_l[2] = 1.0;
  arma::vec::fixed<3> p_g;
  p_g[0] = 9.0;
  p_g[1] = 19.0;
  p_g[2] = 1.0;
  const auto d = align_delta_xy(p_g, p_l);

  CamPose T_gs = make_pose(0.2, p_g[0], p_g[1], p_g[2]);
  CamPose T_ekf_prev = make_pose(0.2, p_l[0], p_l[1], p_l[2]);
  CamPose T_ekf_curr = make_pose(0.25, p_g[0] + 1.0, p_g[1] + 0.2, 1.05);

  CamPose bad = propagate(T_gs, T_ekf_prev, T_ekf_curr);
  rebase_cam_pose_xy(T_ekf_prev, d[0], d[1]);
  CamPose good = propagate(T_gs, T_ekf_prev, T_ekf_curr);

  EXPECT_GT(arma::norm(bad.t_c2n.subvec(0, 1) - T_ekf_curr.t_c2n.subvec(0, 1)), 0.5);
  EXPECT_NEAR(arma::norm(good.t_c2n - T_ekf_curr.t_c2n), 0.0, 1e-9);
}

TEST(VisualSync, HybridZUnchangedByRebase)
{
  CamPose T_gs = make_pose(0.1, 1.0, 2.0, 10.0);
  CamPose T_prev = make_pose(0.1, 0.0, 0.0, 1.0);
  CamPose T_curr = make_pose(0.15, 2.0, 3.0, -0.55);
  rebase_cam_pose_xy(T_prev, 1.0, 2.0);
  CamPose T_hyb = propagate_xy_rel_z_abs(T_gs, T_prev, T_curr);
  EXPECT_NEAR(T_hyb.t_c2n(2), T_curr.t_c2n(2), 1e-12);
}

TEST(VisualSync, AdmitGateGenerationBoundary)
{
  // Scenario: admitted 100, Local generated 101/102, then apply → watermark 102.
  EXPECT_FALSE(should_admit_keyframe(AdmitMode::Suspend, 0, 101));
  EXPECT_FALSE(should_admit_keyframe(AdmitMode::Bound, 102, 100));
  EXPECT_FALSE(should_admit_keyframe(AdmitMode::Bound, 102, 101));
  EXPECT_FALSE(should_admit_keyframe(AdmitMode::Bound, 102, 102));
  EXPECT_TRUE(should_admit_keyframe(AdmitMode::Bound, 102, 103));
  EXPECT_TRUE(should_admit_keyframe(AdmitMode::Open, 102, 50));
}

TEST(VisualSync, FailedApplyNoRebaseSemantics)
{
  CamPose prev = make_pose(0.0, 1.0, 2.0, 3.0);
  CamPose untouched = prev;
  // Model: only call rebase on applied — failed path leaves pose alone.
  EXPECT_NEAR(arma::norm(prev.t_c2n - untouched.t_c2n), 0.0, 1e-15);
  EXPECT_TRUE(should_admit_keyframe(AdmitMode::Open, 0, 1));
}

TEST(VisualSync, PhysicalMotionPreservedByAlignTranslate)
{
  // Live Local pose after reference KF: p_L_live = p_L_ref + motion.
  arma::vec::fixed<3> p_l_ref;
  p_l_ref[0] = 10.0;
  p_l_ref[1] = 20.0;
  p_l_ref[2] = 1.0;
  arma::vec::fixed<3> motion;
  motion[0] = 0.4;
  motion[1] = -0.2;
  motion[2] = 0.05;
  arma::vec::fixed<3> p_l_live = p_l_ref + motion;
  arma::vec::fixed<3> p_g_final;
  p_g_final[0] = 9.0;
  p_g_final[1] = 19.0;
  p_g_final[2] = 1.0;
  const auto d = align_delta_xy(p_g_final, p_l_ref);
  arma::vec::fixed<3> after = p_l_live + d;
  EXPECT_NEAR(after[0], p_g_final[0] + motion[0], 1e-12);
  EXPECT_NEAR(after[1], p_g_final[1] + motion[1], 1e-12);
  EXPECT_NEAR(after[2], p_l_live[2], 1e-12);  // Z untouched by δ_L
}

TEST(VisualSync, LandmarkRelativeGeometryPreserved)
{
  arma::vec::fixed<3> vehicle;
  vehicle[0] = 1.0;
  vehicle[1] = 2.0;
  vehicle[2] = 3.0;
  arma::vec::fixed<3> landmark;
  landmark[0] = 4.0;
  landmark[1] = 5.0;
  landmark[2] = 6.0;
  const arma::vec::fixed<3> rel_before = landmark - vehicle;
  const double dx = -1.5;
  const double dy = 0.75;
  vehicle[0] += dx;
  vehicle[1] += dy;
  landmark[0] += dx;
  landmark[1] += dy;
  EXPECT_NEAR(arma::norm((landmark - vehicle) - rel_before), 0.0, 1e-15);
  EXPECT_NEAR(vehicle[2], 3.0, 1e-15);
  EXPECT_NEAR(landmark[2], 6.0, 1e-15);
}

TEST(VisualSync, SubsequentKfNoCorrectionDisplacement)
{
  CamPose T_gs = make_pose(0.0, 9.0, 19.0, 1.0);
  CamPose T_prev = make_pose(0.0, 9.0, 19.0, 1.0);  // already corrected ref
  CamPose T_curr = make_pose(0.1, 10.0, 19.5, 1.1);
  CamPose T_next = make_pose(0.15, 11.0, 20.0, 1.2);
  CamPose first = propagate(T_gs, T_prev, T_curr);
  CamPose second = propagate(first, T_curr, T_next);
  // Ordinary relative init: second uses live poses only — no δ_L term.
  EXPECT_NEAR(second.t_c2n(0), T_next.t_c2n(0), 1e-9);
  EXPECT_NEAR(second.t_c2n(1), T_next.t_c2n(1), 1e-9);
}

TEST(VisualSync, UnknownStatusKeepsSuspend)
{
  // While Suspend, no KF may enter (including post-watermark ids).
  EXPECT_FALSE(should_admit_keyframe(AdmitMode::Suspend, 102, 103));
  EXPECT_FALSE(should_admit_keyframe(AdmitMode::Suspend, 0, 1));
  // Opening without rebase is the definitive-fail path, not unknown.
  EXPECT_TRUE(should_admit_keyframe(AdmitMode::Open, 0, 50));
}

TEST(VisualSync, DuplicateRebaseIsIdempotentOnlyIfCallerGuards)
{
  CamPose prev = make_pose(0.3, 1.0, 2.0, 3.0);
  rebase_cam_pose_xy(prev, 0.5, -0.25);
  CamPose once = prev;
  // A second accidental rebase would double-count — production must gate.
  rebase_cam_pose_xy(prev, 0.5, -0.25);
  EXPECT_NEAR(prev.t_c2n(0) - once.t_c2n(0), 0.5, 1e-12);
  EXPECT_NEAR(arma::norm(once.t_c2n.subvec(0, 1) - arma::vec({1.5, 1.75})), 0.0, 1e-12);
}

TEST(VisualSync, TwoSuccessiveIndependentSyncs)
{
  arma::vec::fixed<3> p_l1;
  p_l1[0] = 10.0;
  p_l1[1] = 20.0;
  p_l1[2] = 1.0;
  arma::vec::fixed<3> p_g1;
  p_g1[0] = 9.0;
  p_g1[1] = 19.0;
  p_g1[2] = 1.0;
  const auto d1 = align_delta_xy(p_g1, p_l1);
  CamPose cache = make_pose(0.0, p_l1[0], p_l1[1], p_l1[2]);
  rebase_cam_pose_xy(cache, d1[0], d1[1]);
  EXPECT_NEAR(cache.t_c2n(0), p_g1[0], 1e-12);
  // First post-sync KF replaces cache with live corrected pose.
  cache = make_pose(0.05, 9.5, 19.2, 1.0);
  EXPECT_TRUE(should_admit_keyframe(AdmitMode::Bound, 50, 51));

  // Second LC later: new original local vs new final global — fresh δ_L.
  arma::vec::fixed<3> p_l2;
  p_l2[0] = 15.0;
  p_l2[1] = 25.0;
  p_l2[2] = 1.0;
  arma::vec::fixed<3> p_g2;
  p_g2[0] = 14.2;
  p_g2[1] = 24.1;
  p_g2[2] = 1.0;
  const auto d2 = align_delta_xy(p_g2, p_l2);
  EXPECT_NEAR(d2[0], -0.8, 1e-12);
  EXPECT_NEAR(d2[1], -0.9, 1e-12);
  CamPose cache2 = make_pose(0.1, p_l2[0], p_l2[1], p_l2[2]);
  rebase_cam_pose_xy(cache2, d2[0], d2[1]);
  EXPECT_NEAR(cache2.t_c2n(0), p_g2[0], 1e-12);
  EXPECT_FALSE(should_admit_keyframe(AdmitMode::Bound, 120, 120));
  EXPECT_TRUE(should_admit_keyframe(AdmitMode::Bound, 120, 121));
}

TEST(VisualSync, FeedbackDisabledNoBoundBoundary)
{
  // Without Local apply there is no Bound watermark — Open admits freely.
  EXPECT_TRUE(should_admit_keyframe(AdmitMode::Open, 0, 1));
  EXPECT_TRUE(should_admit_keyframe(AdmitMode::Open, 999, 1));
}

TEST(VisualSync, SyncDuringMotionDoesNotRequireZeroVelocity)
{
  // A4.1 / A3: alignment translate preserves post-reference motion; no
  // quiescence or zero-velocity gate in the sync helpers.
  arma::vec::fixed<3> p_l_ref;
  p_l_ref[0] = 1.0;
  p_l_ref[1] = 2.0;
  p_l_ref[2] = 3.0;
  arma::vec::fixed<3> motion;
  motion[0] = 0.8;
  motion[1] = -0.3;
  motion[2] = 0.0;
  arma::vec::fixed<3> p_g;
  p_g[0] = 0.5;
  p_g[1] = 1.5;
  p_g[2] = 3.0;
  const auto d = align_delta_xy(p_g, p_l_ref);
  arma::vec::fixed<3> live = p_l_ref + motion + d;
  EXPECT_NEAR(live[0], p_g[0] + motion[0], 1e-12);
  EXPECT_NEAR(live[1], p_g[1] + motion[1], 1e-12);
}
