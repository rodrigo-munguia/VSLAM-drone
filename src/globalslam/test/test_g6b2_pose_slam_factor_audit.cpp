/**
 * G6b.2 audit-only: Pose_SLAM loop-closure factor semantics (GTSAM 4.2).
 *
 * Mirrors production construction in cloop_pose_slam.cpp:
 *   BetweenFactor<Pose3>(x_last, x_ref, Pose3(I, t_ref - pos), Diagonal σ)
 * with σ_att = CL_clo_sigma_kf_att (default 1e-5), σ_xy/z from YAML.
 *
 * Does not modify production optimizer factors.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

using gtsam::symbol_shorthand::X;

namespace
{

gtsam::SharedNoiseModel make_lc_noise(
  double sigma_att, double sigma_xy, double sigma_z)
{
  return gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << sigma_att, sigma_att, sigma_att,
      sigma_xy, sigma_xy, sigma_z)
      .finished());
}

/** Production-shaped home loop factor: measured relative pose R≡I. */
gtsam::BetweenFactor<gtsam::Pose3> make_home_between(
  gtsam::Key k_last, gtsam::Key k_ref,
  const gtsam::Point3 & t_ref, const gtsam::Point3 & pos_obs,
  const gtsam::SharedNoiseModel & noise)
{
  const gtsam::Point3 clo = t_ref - pos_obs;
  const gtsam::Pose3 measured(gtsam::Rot3::Identity(), clo);
  return gtsam::BetweenFactor<gtsam::Pose3>(k_last, k_ref, measured, noise);
}

}  // namespace

TEST(G6b2PoseSlamFactorAudit, ResidualIsSixDimensional)
{
  const auto noise = make_lc_noise(1e-5, 5e-4, 5e-3);
  const gtsam::Point3 t_ref(0, 0, -0.3);
  const gtsam::Point3 pos(1.0, -0.5, -4.0);
  auto factor = make_home_between(X(1), X(0), t_ref, pos, noise);

  const gtsam::Pose3 p_last(gtsam::Rot3::Identity(), pos);
  const gtsam::Pose3 p_ref(gtsam::Rot3::Identity(), t_ref);
  const gtsam::Vector e = factor.evaluateError(p_last, p_ref);
  ASSERT_EQ(e.size(), 6);
  EXPECT_NEAR(e.norm(), 0.0, 1e-12);
}

TEST(G6b2PoseSlamFactorAudit, RotationPerturbationChangesFactorCost)
{
  // Hold translation fixed; perturb last-node rotation only.
  const double sigma_att = 1e-5;  // production CL_clo_sigma_kf_att
  const double sigma_xy = 5e-4;
  const double sigma_z = 5e-3;
  const auto noise = make_lc_noise(sigma_att, sigma_xy, sigma_z);
  const gtsam::Point3 t_ref(0, 0, -0.3);
  const gtsam::Point3 pos(1.0, -0.5, -4.0);
  auto factor = make_home_between(X(1), X(0), t_ref, pos, noise);

  const gtsam::Pose3 p_ref(gtsam::Rot3::Identity(), t_ref);
  const gtsam::Pose3 p_last_I(gtsam::Rot3::Identity(), pos);
  gtsam::Values vals_I;
  vals_I.insert(X(1), p_last_I);
  vals_I.insert(X(0), p_ref);
  const double cost_I = factor.error(vals_I);

  const gtsam::Rot3 R_pert = gtsam::Rot3::RzRyRx(0.05, -0.03, 0.02);
  const gtsam::Pose3 p_last_R(R_pert, pos);
  gtsam::Values vals_R;
  vals_R.insert(X(1), p_last_R);
  vals_R.insert(X(0), p_ref);
  const double cost_R = factor.error(vals_R);

  const gtsam::Vector e_R = factor.evaluateError(p_last_R, p_ref);
  // Rotational residual components (first 3 of Pose3 tangent) must be nonzero.
  EXPECT_GT(e_R.head<3>().norm(), 1e-3);
  EXPECT_NEAR(cost_I, 0.0, 1e-18);
  EXPECT_GT(cost_R, 1e3);  // (δθ/σ_att)^2 with σ_att=1e-5 → huge
  EXPECT_GT(cost_R, cost_I + 1e3);

  std::cout << "G6b2 factor audit: cost_I=" << cost_I
            << " cost_R_pert=" << cost_R
            << " |e_rot|=" << e_R.head<3>().norm()
            << " |e_trans|=" << e_R.tail<3>().norm() << std::endl;
}

TEST(G6b2PoseSlamFactorAudit, TranslationPerturbationChangesCostIndependently)
{
  const auto noise = make_lc_noise(1e-5, 5e-4, 5e-3);
  const gtsam::Point3 t_ref(0, 0, -0.3);
  const gtsam::Point3 pos(0, 0, -4.0);
  auto factor = make_home_between(X(1), X(0), t_ref, pos, noise);
  const gtsam::Pose3 p_ref(gtsam::Rot3::Identity(), t_ref);

  const gtsam::Pose3 p_ok(gtsam::Rot3::Identity(), pos);
  const gtsam::Pose3 p_shift(gtsam::Rot3::Identity(), gtsam::Point3(0.2, 0, -4.0));
  gtsam::Values v0;
  v0.insert(X(1), p_ok);
  v0.insert(X(0), p_ref);
  gtsam::Values v1;
  v1.insert(X(1), p_shift);
  v1.insert(X(0), p_ref);
  const double c0 = factor.error(v0);
  const double c1 = factor.error(v1);
  EXPECT_NEAR(c0, 0.0, 1e-18);
  EXPECT_GT(c1, c0);
  const gtsam::Vector e = factor.evaluateError(p_shift, p_ref);
  EXPECT_NEAR(e.head<3>().norm(), 0.0, 1e-12);
  EXPECT_GT(e.tail<3>().norm(), 0.1);
}

TEST(G6b2PoseSlamFactorAudit, IdentityRPullsConnectedPoseRotationInMiniGraph)
{
  // Mini graph: prior on ref (tight), odometry Between(I, Δt), home Between(I, …).
  // Initialize last node with wrong rotation; optimize; rotation is pulled to I.
  const double sigma_att = 1e-5;
  const auto lc_noise = make_lc_noise(sigma_att, 5e-4, 5e-3);
  const auto odo_noise = make_lc_noise(0.01, 0.05, 0.05);
  const auto prior_noise = make_lc_noise(
    sigma_att / 10000.0, 5e-4 / 10000.0, 5e-3 / 10000.0);

  const gtsam::Point3 t_ref(0, 0, -0.3);
  const gtsam::Point3 pos_true(2.0, 0.0, -4.0);
  const gtsam::Point3 odo = pos_true - t_ref;

  gtsam::NonlinearFactorGraph graph;
  graph.addPrior(X(0), gtsam::Pose3(gtsam::Rot3::Identity(), t_ref), prior_noise);
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
    X(0), X(1), gtsam::Pose3(gtsam::Rot3::Identity(), odo), odo_noise);
  graph.add(make_home_between(X(1), X(0), t_ref, pos_true, lc_noise));

  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3(gtsam::Rot3::Identity(), t_ref));
  // Wrong attitude on last KF; correct translation.
  initial.insert(
    X(1),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.4, 0.0, 0.0), pos_true));

  const double err0 = graph.error(initial);
  gtsam::Values result =
    gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  const double err1 = graph.error(result);

  const gtsam::Rot3 R1 = result.at<gtsam::Pose3>(X(1)).rotation();
  const double rot_err =
    gtsam::Rot3::Logmap(R1.between(gtsam::Rot3::Identity())).norm();

  EXPECT_LT(err1, err0);
  EXPECT_LT(rot_err, 0.05);  // attitude pulled toward identity by Between R≡I
  std::cout << "G6b2 mini-graph: err0=" << err0 << " err1=" << err1
            << " |Log(R_last)|=" << rot_err << std::endl;
}

TEST(G6b2PoseSlamFactorAudit, ClassifyNotTranslationOnlyFactor)
{
  // Classification lock for the audit document:
  // Residual dim=6 + finite σ_att + cost rises under pure rotation perturbation
  // ⇒ NOT class A (translation-only). Closest: B (full-pose) with intentional
  // identity measurement, equivalently C if one only looks at default init R=I.
  const auto noise = make_lc_noise(1e-5, 5e-4, 5e-3);
  auto factor = make_home_between(
    X(1), X(0), gtsam::Point3(0, 0, 0), gtsam::Point3(1, 0, 0), noise);
  gtsam::Matrix H1, H2;
  const gtsam::Vector e = factor.evaluateError(
    gtsam::Pose3(gtsam::Rot3::Rz(0.01), gtsam::Point3(1, 0, 0)),
    gtsam::Pose3(),
    H1, H2);
  ASSERT_EQ(e.size(), 6);
  ASSERT_EQ(H1.rows(), 6);
  ASSERT_EQ(H1.cols(), 6);
  // Jacobian w.r.t. rotational coords of p1 is nonzero → rotational influence.
  const double Hrot = H1.block(0, 0, 3, 3).norm();
  EXPECT_GT(Hrot, 0.0);
}
