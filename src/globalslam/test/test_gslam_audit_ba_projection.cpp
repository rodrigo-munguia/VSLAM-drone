/**
 * Audit harness (offline): GTSAM GenericProjectionFactor + Cal3_S2 consistency
 * and Delta_norm XY-gate semantics used by GMAP::Bundle_adjustment.
 *
 * Does not modify production BA. Expected projections are generated with an
 * independent pinhole model (not by reading BA helper internals).
 */

#include <cmath>
#include <gtest/gtest.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/inference/Symbol.h>
#include <vector>

using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

namespace
{

/** Independent pinhole projection (pixel), matching Cal3_S2 (no distortion). */
gtsam::Point2 independent_project(
  const gtsam::Pose3 & pose_w_c,
  const gtsam::Point3 & Xw,
  double fx, double fy, double cx, double cy)
{
  const gtsam::Point3 Xc = pose_w_c.transformTo(Xw);
  EXPECT_GT(Xc.z(), 1e-6);
  return gtsam::Point2(fx * Xc.x() / Xc.z() + cx, fy * Xc.y() / Xc.z() + cy);
}

/** Production BA gate: XY displacement only (metres). */
bool ba_delta_norm_accepts(
  double dx, double dy, double /*dz*/, double max_delta_xy)
{
  const double delta_norm = std::sqrt(dx * dx + dy * dy);
  return delta_norm <= max_delta_xy;
}

}  // namespace

TEST(GslamAuditBaProjection, ExactCalibratedObservationsRecoverLandmark)
{
  const double fx = 200.0, fy = 200.0, cx = 200.0, cy = 120.0;
  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(fx, fy, 0.0, cx, cy));
  auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  // Two cameras looking at a known landmark (observable parallax).
  const gtsam::Point3 X_true(1.0, 0.5, 4.0);
  const gtsam::Pose3 pose0(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 0));
  const gtsam::Pose3 pose1(gtsam::Rot3::Identity(), gtsam::Point3(0.8, 0, 0));

  const gtsam::Point2 z0 = independent_project(pose0, X_true, fx, fy, cx, cy);
  const gtsam::Point2 z1 = independent_project(pose1, X_true, fx, fy, cx, cy);

  gtsam::NonlinearFactorGraph graph;
  graph.addPrior(X(0), pose0, gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished()));
  graph.addPrior(X(1), pose1, gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished()));
  graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
    z0, noise, X(0), L(0), K);
  graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
    z1, noise, X(1), L(0), K);

  gtsam::Values init;
  init.insert(X(0), pose0);
  init.insert(X(1), pose1);
  init.insert(L(0), gtsam::Point3(1.2, 0.4, 3.5));  // perturbed

  const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, init).optimize();
  const gtsam::Point3 X_hat = result.at<gtsam::Point3>(L(0));
  EXPECT_NEAR(X_hat.x(), X_true.x(), 1e-3);
  EXPECT_NEAR(X_hat.y(), X_true.y(), 1e-3);
  EXPECT_NEAR(X_hat.z(), X_true.z(), 1e-3);
  EXPECT_LT(graph.error(result), graph.error(init));
}

TEST(GslamAuditBaProjection, DeltaNormGateIsHorizontalOnly)
{
  // Production: Delta_norm = sqrt(dx^2+dy^2); z ignored.
  EXPECT_TRUE(ba_delta_norm_accepts(0.0, 0.0, 100.0, 1.75));
  EXPECT_TRUE(ba_delta_norm_accepts(1.0, 1.0, 0.0, 1.75));  // ~1.41
  EXPECT_FALSE(ba_delta_norm_accepts(2.0, 0.0, 0.0, 1.75));
  EXPECT_FALSE(ba_delta_norm_accepts(1.3, 1.3, 0.0, 1.75));  // ~1.84
}

TEST(GslamAuditBaProjection, OutlierAssociationInflatesError)
{
  const double fx = 200.0, fy = 200.0, cx = 200.0, cy = 120.0;
  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(fx, fy, 0.0, cx, cy));
  auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
  const gtsam::Point3 X_true(0.0, 0.0, 5.0);
  const gtsam::Pose3 pose(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 0));
  const gtsam::Point2 z_good = independent_project(pose, X_true, fx, fy, cx, cy);
  const gtsam::Point2 z_bad(z_good.x() + 40.0, z_good.y() - 25.0);

  gtsam::NonlinearFactorGraph g_good, g_bad;
  g_good.addPrior(X(0), pose, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
  g_bad.addPrior(X(0), pose, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
  g_good.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
    z_good, noise, X(0), L(0), K);
  g_bad.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
    z_bad, noise, X(0), L(0), K);

  gtsam::Values v;
  v.insert(X(0), pose);
  v.insert(L(0), X_true);
  EXPECT_LT(g_good.error(v), 1.0);
  EXPECT_GT(g_bad.error(v), 100.0);
}

/**
 * Production-like gauge: strong pose prior on first KF, one landmark prior,
 * free remaining landmarks — recover nonplanar scene with observable baseline.
 */
TEST(GslamAuditBaProjection, ProductionLikeGaugeRecoversNonplanarScene)
{
  const double fx = 320.0, fy = 320.0, cx = 320.0, cy = 240.0;
  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(fx, fy, 0.0, cx, cy));
  auto meas = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  const std::vector<gtsam::Point3> X_true = {
    {0.5, 0.2, 5.0}, {-0.4, 0.6, 4.5}, {0.1, -0.5, 6.0}, {1.0, 0.0, 5.5}};
  const gtsam::Pose3 pose0(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 0));
  const gtsam::Pose3 pose1(gtsam::Rot3::Identity(), gtsam::Point3(1.0, 0.05, 0));
  const gtsam::Pose3 pose2(gtsam::Rot3::Identity(), gtsam::Point3(2.0, -0.05, 0));

  gtsam::NonlinearFactorGraph graph;
  // Strong first-KF prior (production /10000 style).
  graph.addPrior(X(0), pose0, gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4).finished()));
  graph.addPrior(X(1), pose1, gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()));
  graph.addPrior(X(2), pose2, gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished()));
  // Single landmark prior (production: first landmark only).
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(
    L(0), X_true[0], gtsam::noiseModel::Isotropic::Sigma(3, 0.1));

  gtsam::Values init;
  init.insert(X(0), pose0);
  init.insert(X(1), gtsam::Pose3(
      gtsam::Rot3::Identity(), gtsam::Point3(1.15, 0.15, 0.05)));
  init.insert(X(2), gtsam::Pose3(
      gtsam::Rot3::Identity(), gtsam::Point3(1.8, 0.1, -0.05)));

  for (size_t i = 0; i < X_true.size(); ++i) {
    const auto z0 = independent_project(pose0, X_true[i], fx, fy, cx, cy);
    const auto z1 = independent_project(pose1, X_true[i], fx, fy, cx, cy);
    const auto z2 = independent_project(pose2, X_true[i], fx, fy, cx, cy);
    graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
      z0, meas, X(0), L(i), K);
    graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
      z1, meas, X(1), L(i), K);
    graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
      z2, meas, X(2), L(i), K);
    const gtsam::Point3 l_pert(
      X_true[i].x() + 0.2, X_true[i].y() - 0.15, X_true[i].z() + 0.3);
    init.insert(L(i), l_pert);
  }

  const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, init).optimize();
  EXPECT_LT(graph.error(result), graph.error(init));
  for (size_t i = 0; i < X_true.size(); ++i) {
    const auto Xh = result.at<gtsam::Point3>(L(i));
    EXPECT_NEAR(Xh.x(), X_true[i].x(), 5e-2);
    EXPECT_NEAR(Xh.y(), X_true[i].y(), 5e-2);
    EXPECT_NEAR(Xh.z(), X_true[i].z(), 5e-2);
  }
  // Reference pose remains near prior.
  const auto t0 = result.at<gtsam::Pose3>(X(0)).translation();
  EXPECT_NEAR(t0.x(), 0.0, 1e-3);
  EXPECT_NEAR(t0.y(), 0.0, 1e-3);
}

/**
 * Noise-free exact observations with fixed poses: even a 2 cm baseline is
 * algebraically sufficient for LM to recover depth. Production low-parallax
 * failures therefore require noise/association/pose uncertainty — not the
 * projection factor alone. Documented by recovering depth here.
 */
TEST(GslamAuditBaProjection, NoiseFreeTinyBaselineStillRecoversDepth)
{
  const double fx = 200.0, fy = 200.0, cx = 200.0, cy = 120.0;
  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(fx, fy, 0.0, cx, cy));
  auto meas = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
  const gtsam::Point3 X_true(0.0, 0.0, 8.0);
  const gtsam::Pose3 pose0(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 0));
  const gtsam::Pose3 pose1(gtsam::Rot3::Identity(), gtsam::Point3(0.02, 0, 0));

  gtsam::NonlinearFactorGraph graph;
  graph.addPrior(X(0), pose0, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
  graph.addPrior(X(1), pose1, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
  graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
    independent_project(pose0, X_true, fx, fy, cx, cy), meas, X(0), L(0), K);
  graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
    independent_project(pose1, X_true, fx, fy, cx, cy), meas, X(1), L(0), K);

  gtsam::Values init;
  init.insert(X(0), pose0);
  init.insert(X(1), pose1);
  init.insert(L(0), gtsam::Point3(0.0, 0.0, 3.0));

  const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, init).optimize();
  EXPECT_NEAR(result.at<gtsam::Point3>(L(0)).z(), X_true.z(), 1e-3);
}

/**
 * Sequential windows: after accepting a first window, a second window with
 * exact observations should not invent large XY jumps that trip the gate.
 */
TEST(GslamAuditBaProjection, SequentialWindowsPreserveAcceptedGeometry)
{
  const double fx = 250.0, fy = 250.0, cx = 320.0, cy = 240.0;
  const double max_delta = 1.75;
  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(fx, fy, 0.0, cx, cy));
  auto meas = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
  const gtsam::Point3 Lm(0.3, -0.2, 5.0);

  auto run_window = [&](const gtsam::Pose3 & p0, const gtsam::Pose3 & p1,
                        const gtsam::Point3 & l_init) {
    gtsam::NonlinearFactorGraph g;
    g.addPrior(X(0), p0, gtsam::noiseModel::Isotropic::Sigma(6, 1e-5));
    g.addPrior(X(1), p1, gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05).finished()));
    g.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
      independent_project(p0, Lm, fx, fy, cx, cy), meas, X(0), L(0), K);
    g.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
      independent_project(p1, Lm, fx, fy, cx, cy), meas, X(1), L(0), K);
    gtsam::Values init;
    init.insert(X(0), p0);
    init.insert(X(1), p1);
    init.insert(L(0), l_init);
    return gtsam::LevenbergMarquardtOptimizer(g, init).optimize();
  };

  const gtsam::Pose3 a0(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 0));
  const gtsam::Pose3 a1(gtsam::Rot3::Identity(), gtsam::Point3(0.7, 0, 0));
  const gtsam::Point3 l_pert(Lm.x() + 0.1, Lm.y() + 0.1, Lm.z() + 0.2);
  const auto r1 = run_window(a0, a1, l_pert);
  const auto t1 = r1.at<gtsam::Pose3>(X(1)).translation();
  EXPECT_TRUE(ba_delta_norm_accepts(t1.x() - 0.7, t1.y(), t1.z(), max_delta));

  // Second window advances along same trajectory; init near truth.
  const gtsam::Pose3 b0 = a1;
  const gtsam::Pose3 b1(gtsam::Rot3::Identity(), gtsam::Point3(1.4, 0, 0));
  const gtsam::Point3 l_from_r1 = r1.at<gtsam::Point3>(L(0));
  const auto r2 = run_window(b0, b1, l_from_r1);
  const auto t2 = r2.at<gtsam::Pose3>(X(1)).translation();
  EXPECT_TRUE(ba_delta_norm_accepts(t2.x() - 1.4, t2.y(), t2.z(), max_delta));
  EXPECT_NEAR(t2.x(), 1.4, 5e-2);
}

/**
 * Demonstrated gate limitation: a cross-wired association can move the free
 * pose by tens of centimetres while still passing BA_max_delta_kf_pos=1.75 m.
 * Lower objective is therefore not proof of geometric correctness.
 */
TEST(GslamAuditBaProjection, WrongAssociationCanPassXyGateYetCorruptPose)
{
  const double fx = 200.0, fy = 200.0, cx = 200.0, cy = 120.0;
  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(fx, fy, 0.0, cx, cy));
  auto meas = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
  const gtsam::Point3 Xa(0.0, 0.0, 4.0);
  const gtsam::Point3 Xb(2.0, 0.0, 4.0);
  const gtsam::Pose3 pose0(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 0));
  const gtsam::Pose3 pose1_true(gtsam::Rot3::Identity(), gtsam::Point3(0.5, 0, 0));

  gtsam::NonlinearFactorGraph graph;
  graph.addPrior(X(0), pose0, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
  graph.addPrior(X(1), pose1_true, gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.2, 0.2, 0.2, 0.5, 0.5, 0.5).finished()));
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(
    L(0), Xa, gtsam::noiseModel::Isotropic::Sigma(3, 0.05));
  graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
    independent_project(pose0, Xa, fx, fy, cx, cy), meas, X(0), L(0), K);
  graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
    independent_project(pose1_true, Xb, fx, fy, cx, cy), meas, X(1), L(0), K);

  gtsam::Values init;
  init.insert(X(0), pose0);
  init.insert(X(1), pose1_true);
  init.insert(L(0), Xa);
  const auto result = gtsam::LevenbergMarquardtOptimizer(graph, init).optimize();
  const auto t1 = result.at<gtsam::Pose3>(X(1)).translation();
  const double dx = t1.x() - pose1_true.x();
  const double dy = t1.y() - pose1_true.y();
  EXPECT_GT(std::abs(dx), 0.3) << "expected measurable pose corruption, dx=" << dx;
  EXPECT_TRUE(ba_delta_norm_accepts(dx, dy, 0.0, 1.75))
    << "gate limitation not demonstrated; dx=" << dx << " dy=" << dy;
}
