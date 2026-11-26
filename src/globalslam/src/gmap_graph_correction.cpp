/**
 * GSLAM-MAP-ARCH-M7.3: behavior-preserving Pose_SLAM / landmark δ / post-loop BA.
 * Extracted from cloop_pose_slam.cpp and cloop_bundle_adjustment.cpp.
 */

#include "gmap_graph_correction.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include "gmap_landmark_identity.hpp"

namespace gmap_graph_correction
{
namespace
{

gtsam::Rot3 identity_rot()
{
  return gtsam::Rot3(1, 0, 0, 0, 1, 0, 0, 0, 1);
}

}  // namespace

bool run_pose_slam(
  const GLOBAL_MAP & map,
  const arma::vec::fixed<3> & pnp_C_nav,
  const PoseSlamConfig & config,
  std::vector<arma::vec::fixed<3>> & delta_kf_pos,
  std::vector<int> & idx_kf_opt,
  PoseSlamDiagnostics * diag)
{
  delta_kf_pos.clear();
  idx_kf_opt.clear();

  if (diag) {
    *diag = PoseSlamDiagnostics{};
  }

  if (map.KeyFDATA.empty()) {
    return false;
  }

  const int idx_kf_ref = map.idx_ref_pose_slam;
  if (idx_kf_ref < 0 ||
    static_cast<std::size_t>(idx_kf_ref) >= map.KeyFDATA.size())
  {
    return false;
  }

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;

  for (int i = idx_kf_ref; i < static_cast<int>(map.KeyFDATA.size()); ++i) {
    const gtsam::Rot3 R = identity_rot();
    gtsam::Point3 t;
    t(0) = map.KeyFDATA[static_cast<std::size_t>(i)].t_c2n[0];
    t(1) = map.KeyFDATA[static_cast<std::size_t>(i)].t_c2n[1];
    t(2) = map.KeyFDATA[static_cast<std::size_t>(i)].t_c2n[2];
    const gtsam::Pose3 pose(R, t);
    initial.insert(gtsam::Symbol('x', i), pose);

    if (diag) {
      diag->variable_indices.push_back(i);
    }

    if (i == idx_kf_ref) {
      const double ref_sigma_att = config.clo_sigma_kf_att / 10000.0;
      const double ref_sigma_xy = config.clo_sigma_kf_xy / 10000.0;
      const double ref_sigma_z = config.clo_sigma_kf_z / 10000.0;
      auto ref_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << ref_sigma_att, ref_sigma_att, ref_sigma_att,
          ref_sigma_xy, ref_sigma_xy, ref_sigma_z)
          .finished());
      graph.addPrior(gtsam::Symbol('x', i), pose, ref_noise);
      if (diag) {
        diag->factor_lines.push_back("prior x" + std::to_string(i));
      }
    }
  }

  auto odo_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << config.odo_sigma_kf_att, config.odo_sigma_kf_att,
      config.odo_sigma_kf_att, config.odo_sigma_kf_xy, config.odo_sigma_kf_xy,
      config.odo_sigma_kf_z)
      .finished());

  for (int i = idx_kf_ref + 1; i < static_cast<int>(map.KeyFDATA.size()); ++i) {
    const arma::vec::fixed<3> odo_i =
      map.KeyFDATA[static_cast<std::size_t>(i)].t_c2n -
      map.KeyFDATA[static_cast<std::size_t>(i - 1)].t_c2n;
    const gtsam::Rot3 R = identity_rot();
    gtsam::Point3 t;
    t(0) = odo_i[0];
    t(1) = odo_i[1];
    t(2) = odo_i[2];
    const gtsam::Pose3 pose_odo(R, t);
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      gtsam::Symbol('x', i - 1), gtsam::Symbol('x', i), pose_odo, odo_noise);
    if (diag) {
      diag->factor_lines.push_back(
        "odo x" + std::to_string(i - 1) + "-x" + std::to_string(i) + " " +
        std::to_string(odo_i[0]) + " " + std::to_string(odo_i[1]) + " " +
        std::to_string(odo_i[2]));
    }
  }

  auto lc_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << config.clo_sigma_kf_att, config.clo_sigma_kf_att,
      config.clo_sigma_kf_att, config.clo_sigma_kf_xy, config.clo_sigma_kf_xy,
      config.clo_sigma_kf_z)
      .finished());
  const arma::vec::fixed<3> clo_c =
    map.KeyFDATA[static_cast<std::size_t>(idx_kf_ref)].t_c2n - pnp_C_nav;
  const gtsam::Rot3 R = identity_rot();
  gtsam::Point3 t;
  t(0) = clo_c[0];
  t(1) = clo_c[1];
  t(2) = clo_c[2];
  const gtsam::Pose3 pose_cl(R, t);
  const int idx_c_kf = static_cast<int>(map.KeyFDATA.size()) - 1;
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
    gtsam::Symbol('x', idx_c_kf), gtsam::Symbol('x', idx_kf_ref), pose_cl,
    lc_noise);
  if (diag) {
    diag->factor_lines.push_back(
      "loop x" + std::to_string(idx_c_kf) + "-x" + std::to_string(idx_kf_ref) +
      " " + std::to_string(clo_c[0]) + " " + std::to_string(clo_c[1]) + " " +
      std::to_string(clo_c[2]));
    diag->t_loop = clo_c;
    diag->pnp_C_nav = pnp_C_nav;
    diag->idx_kf_ref = idx_kf_ref;
    diag->idx_closing = idx_c_kf;
  }

  const double initial_error = graph.error(initial);
  const gtsam::Values result =
    gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  const double final_error = graph.error(result);

  std::cout << "cloop-> ---- Close loop results ---------------- " << std::endl;
  std::cout << "cloop-> initial error = " << initial_error << std::endl;
  std::cout << "cloop-> final error = " << final_error << std::endl;
  std::cout << "cloop-> number of kf: " << map.KeyFDATA.size() << std::endl;
  std::cout << "cloop-> last kf pose:           "
            << map.KeyFDATA.back().t_c2n.t() << std::endl;
  std::cout << "cloop-> last kf optimized pose: "
            << result.at<gtsam::Pose3>(gtsam::Symbol('x', idx_c_kf))
                 .translation()
                 .transpose()
            << std::endl;

  for (int i = idx_kf_ref; i < static_cast<int>(map.KeyFDATA.size()); ++i) {
    const Eigen::Matrix<double, 3, 1> t_o =
      result.at<gtsam::Pose3>(gtsam::Symbol('x', i)).translation();
    arma::vec::fixed<3> dt_kf;
    dt_kf[0] = t_o[0] - map.KeyFDATA[static_cast<std::size_t>(i)].t_c2n[0];
    dt_kf[1] = t_o[1] - map.KeyFDATA[static_cast<std::size_t>(i)].t_c2n[1];
    if (config.update_z) {
      dt_kf[2] = t_o[2] - map.KeyFDATA[static_cast<std::size_t>(i)].t_c2n[2];
    } else {
      dt_kf[2] = 0;
    }
    idx_kf_opt.push_back(i);
    delta_kf_pos.push_back(dt_kf);
  }

  if (diag) {
    diag->ok = !delta_kf_pos.empty();
    diag->initial_error = initial_error;
    diag->final_error = final_error;
  }
  return !delta_kf_pos.empty();
}

void apply_pose_slam_geometry(
  GLOBAL_MAP & map,
  const std::vector<arma::vec::fixed<3>> & delta_kf_pos,
  const std::vector<int> & idx_kf_opt)
{
  for (std::size_t i = 0; i < idx_kf_opt.size(); ++i) {
    const int idx_kf = idx_kf_opt[i];
    map.KeyFDATA[static_cast<std::size_t>(idx_kf)].t_c2n += delta_kf_pos[i];
  }

  for (int i = static_cast<int>(map.AnchorsDATA.size()) - 1; i >= 0; --i) {
    const int idx_init_kf = map.AnchorsDATA[static_cast<std::size_t>(i)].init_KF;
    const auto it = std::find(idx_kf_opt.begin(), idx_kf_opt.end(), idx_init_kf);
    if (it != idx_kf_opt.end()) {
      const int index = static_cast<int>(it - idx_kf_opt.begin());
      map.AnchorsDATA[static_cast<std::size_t>(i)].AnchorState +=
        delta_kf_pos[static_cast<std::size_t>(index)];
    }
  }
}

bool run_post_loop_ba(
  GLOBAL_MAP & map,
  const std::vector<int> & idx_vl_new_kf,
  const std::vector<int> & idx_vl_old_kf_in,
  const CAM & cam,
  const PostLoopBaConfig & config,
  PostLoopBaDiagnostics * diag)
{
  if (diag) {
    *diag = PostLoopBaDiagnostics{};
  }

  std::vector<int> idx_vl_old_kf = idx_vl_old_kf_in;

  const double fc1 = cam.fc[0];
  const double fc2 = cam.fc[1];
  const double cc1 = cam.cc[0];
  const double cc2 = cam.cc[1];
  const double alpha_c = cam.alpha_c;
  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(fc1, fc2, alpha_c, cc1, cc2));
  auto measurement_noise =
    gtsam::noiseModel::Isotropic::Sigma(2, config.sigma_uv);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  std::vector<int64_t> idx_points;

  std::vector<int> idx_kf_to_optimize;
  const int n_old_kf = static_cast<int>(idx_vl_old_kf.size());
  for (int i = 0; i < n_old_kf; ++i) {
    const int idx_kf_t = idx_vl_old_kf[static_cast<std::size_t>(i)];
    if (std::find(idx_kf_to_optimize.begin(), idx_kf_to_optimize.end(),
          idx_kf_t) == idx_kf_to_optimize.end())
    {
      idx_kf_to_optimize.push_back(idx_kf_t);
    }
  }
  std::sort(idx_vl_old_kf.begin(), idx_vl_old_kf.end());

  constexpr int max_n_new_kf_2_opt = 7;
  for (int i = static_cast<int>(map.KeyFDATA.size()) - 1;
       i >= static_cast<int>(map.KeyFDATA.size()) - max_n_new_kf_2_opt; --i)
  {
    if (i < 0) {
      break;
    }
    if (std::find(idx_kf_to_optimize.begin(), idx_kf_to_optimize.end(), i) ==
      idx_kf_to_optimize.end())
    {
      idx_kf_to_optimize.push_back(i);
    }
  }
  std::sort(idx_kf_to_optimize.begin(), idx_kf_to_optimize.end());

  (void)idx_vl_new_kf;

  std::cout << "cloop-> Local bundle adjustment for loop close refinement"
            << std::endl;

  for (std::size_t k = 0; k < idx_kf_to_optimize.size(); ++k) {
    const int idx_kf = idx_kf_to_optimize[k];
    const arma::mat::fixed<3, 3> Rc2n = map.KeyFDATA[static_cast<std::size_t>(idx_kf)].Rn2c.t();
    gtsam::Rot3 R(
      Rc2n.at(0, 0), Rc2n.at(0, 1), Rc2n.at(0, 2), Rc2n.at(1, 0), Rc2n.at(1, 1),
      Rc2n.at(1, 2), Rc2n.at(2, 0), Rc2n.at(2, 1), Rc2n.at(2, 2));
    gtsam::Point3 t;
    t(0) = map.KeyFDATA[static_cast<std::size_t>(idx_kf)].t_c2n[0];
    t(1) = map.KeyFDATA[static_cast<std::size_t>(idx_kf)].t_c2n[1];
    t(2) = map.KeyFDATA[static_cast<std::size_t>(idx_kf)].t_c2n[2];
    const gtsam::Pose3 pose(R, t);

    double sigma_att;
    double sigma_xy;
    double sigma_z;
    if (k == 0) {
      sigma_att = config.sigma_kf_att / 10000.0;
      sigma_xy = config.sigma_kf_xy / 10000.0;
      sigma_z = config.sigma_kf_z / 10000.0;
    } else {
      sigma_att = config.sigma_kf_att;
      sigma_xy = config.sigma_kf_xy;
      sigma_z = config.sigma_kf_z;
    }
    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << sigma_att, sigma_att, sigma_att, sigma_xy, sigma_xy,
        sigma_z)
        .finished());
    graph.addPrior(gtsam::Symbol('x', idx_kf), pose, pose_noise);
    initial.insert(gtsam::Symbol('x', idx_kf), pose);

    for (std::size_t j = 0;
         j < map.KeyFDATA[static_cast<std::size_t>(idx_kf)]
               .Matched_landmark_ids.size();
         ++j)
    {
      const LandmarkId lid =
        map.KeyFDATA[static_cast<std::size_t>(idx_kf)].Matched_landmark_ids[j];
      std::size_t idx_pt_sz = 0;
      if (!gmap_landmark_id::resolve_landmark_index(map, lid, idx_pt_sz)) {
        std::cout << "cloop BA: unresolved LandmarkId=" << lid.value
                  << " kf=" << idx_kf << " — skipping factor" << std::endl;
        continue;
      }
      const int64_t idx_pt = static_cast<int64_t>(idx_pt_sz);
      const cv::Point2d ud_d =
        map.KeyFDATA[static_cast<std::size_t>(idx_kf)].UV_Matched_points[j];
      const cv::Point2d uv_u = Undistort_a_point(ud_d, cam, 1);
      gtsam::Point2 pt;
      pt(0) = uv_u.x;
      pt(1) = uv_u.y;
      graph.emplace_shared<
        gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
        pt, measurement_noise, gtsam::Symbol('x', idx_kf),
        gtsam::Symbol('l', idx_pt), K);
      if (std::find(idx_points.begin(), idx_points.end(), idx_pt) ==
        idx_points.end())
      {
        idx_points.push_back(idx_pt);
      }
    }
    std::cout << "cloop-> Kf: " << idx_kf << " n matches: "
              << map.KeyFDATA[static_cast<std::size_t>(idx_kf)]
                   .Matched_landmark_ids.size()
              << std::endl;
  }
  std::cout << std::endl;

  if (!idx_vl_old_kf.empty()) {
    const int last_old = idx_vl_old_kf.back();
    for (std::size_t i = 0; i < idx_points.size(); ++i) {
      const int64_t idx_pt = idx_points[i];
      gtsam::Point3 Pt;
      Pt(0) = map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].AnchorState[0];
      Pt(1) = map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].AnchorState[1];
      Pt(2) = map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].AnchorState[2];
      if (map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].init_KF <= last_old) {
        auto point_noise =
          gtsam::noiseModel::Isotropic::Sigma(3, config.sigma_kf_pt);
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(
          gtsam::Symbol('l', idx_pt), Pt, point_noise);
      }
      initial.insert<gtsam::Point3>(gtsam::Symbol('l', idx_pt), Pt);
    }
  } else {
    for (std::size_t i = 0; i < idx_points.size(); ++i) {
      const int64_t idx_pt = idx_points[i];
      gtsam::Point3 Pt;
      Pt(0) = map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].AnchorState[0];
      Pt(1) = map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].AnchorState[1];
      Pt(2) = map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].AnchorState[2];
      initial.insert<gtsam::Point3>(gtsam::Symbol('l', idx_pt), Pt);
    }
  }

  const double initial_error = graph.error(initial);
  const gtsam::Values result =
    gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  const double final_error = graph.error(result);

  bool f_valid_update = true;
  for (std::size_t k = 0; k < idx_kf_to_optimize.size(); ++k) {
    const int idx_kf = idx_kf_to_optimize[k];
    const Eigen::Matrix<double, 3, 1> t_o =
      result.at<gtsam::Pose3>(gtsam::Symbol('x', idx_kf)).translation();
    arma::vec::fixed<3> D_kf_n;
    D_kf_n[0] = t_o[0] - map.KeyFDATA[static_cast<std::size_t>(idx_kf)].t_c2n[0];
    D_kf_n[1] = t_o[1] - map.KeyFDATA[static_cast<std::size_t>(idx_kf)].t_c2n[1];
    D_kf_n[2] = t_o[2] - map.KeyFDATA[static_cast<std::size_t>(idx_kf)].t_c2n[2];
    const double Delta_norm =
      std::sqrt(std::pow(D_kf_n[0], 2) + std::pow(D_kf_n[1], 2));
    if (Delta_norm > config.max_delta_kf_pos) {
      f_valid_update = false;
      std::cout << "Delta_norm for n-kf optimized too large!! " << std::endl;
    }
  }

  for (std::size_t k = 0; k < idx_kf_to_optimize.size(); ++k) {
    const int idx_kf = idx_kf_to_optimize[k];
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 3, 1> t_o;
    R = result.at<gtsam::Pose3>(gtsam::Symbol('x', idx_kf)).rotation().matrix();
    t_o = result.at<gtsam::Pose3>(gtsam::Symbol('x', idx_kf)).translation();
    arma::mat R_arma_t = arma::mat(R.data(), R.rows(), R.cols(), false, false);
    arma::mat::fixed<3, 3> R_o = R_arma_t.t();

    if (config.update_kf_att && f_valid_update) {
      map.KeyFDATA[static_cast<std::size_t>(idx_kf)].Rn2c = R_o;
    }
    if (config.update_kf_pos && f_valid_update) {
      map.KeyFDATA[static_cast<std::size_t>(idx_kf)].t_c2n[0] = t_o[0];
      map.KeyFDATA[static_cast<std::size_t>(idx_kf)].t_c2n[1] = t_o[1];
      map.KeyFDATA[static_cast<std::size_t>(idx_kf)].t_c2n[2] = t_o[2];
    }
  }

  if (config.update_map && f_valid_update) {
    for (std::size_t i = 0; i < idx_points.size(); ++i) {
      const int64_t idx_pt = idx_points[i];
      const Eigen::Vector3d pt =
        result.at<gtsam::Point3>(gtsam::Symbol('l', idx_pt));
      map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].AnchorState[0] = pt[0];
      map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].AnchorState[1] = pt[1];
      map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].AnchorState[2] = pt[2];
    }
  }

  if (diag) {
    diag->accepted = f_valid_update;
    diag->initial_error = initial_error;
    diag->final_error = final_error;
    diag->idx_kf_optimized = idx_kf_to_optimize;
    diag->idx_points = idx_points;
    diag->n_projection_factors = 0;
    for (int idx_kf : idx_kf_to_optimize) {
      diag->n_projection_factors += static_cast<int>(
        map.KeyFDATA[static_cast<std::size_t>(idx_kf)]
          .Matched_landmark_ids.size());
    }
  }
  return f_valid_update;
}

}  // namespace gmap_graph_correction
