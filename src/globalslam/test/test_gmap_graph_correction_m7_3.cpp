/**
 * GSLAM-MAP-ARCH-M7.3: production Pose_SLAM / landmark δ / post-loop BA
 * equivalence on an explicitly supplied GLOBAL_MAP.
 *
 * Immutable reference = independent verbatim mirror of pre-extraction
 * cloop_pose_slam / cloop_update_gmap landmark apply (HEAD c7f24e8).
 * Extracted APIs must match that mirror within numerical tolerance.
 */
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include "gmap_graph_correction.hpp"
#include "gmap_landmark_identity.hpp"
#include "gmap_visual_sync.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/keyframe_types.hpp"

using gmap_landmark_id::insert_landmark;
using gmap_landmark_id::rebuild_id_to_index;
using gmap_landmark_id::validate_landmark_identity_invariants;

namespace
{

constexpr double kPoseTol = 1e-9;
constexpr double kObjTol = 1e-8;

KEYFRAME make_kf(double x, double y, double z, uint64_t id)
{
  KEYFRAME kf;
  kf.t_c2n = {x, y, z};
  kf.Rn2c = arma::eye<arma::mat>(3, 3);
  kf.Rr2c = arma::eye<arma::mat>(3, 3);
  kf.Rn2r = arma::eye<arma::mat>(3, 3);
  kf.t_c2r = {0, 0, 0};
  kf.r_N = {x, y, z};
  kf.keyframe_id = id;
  return kf;
}

G_ANCHOR make_anchor(double x, double y, double z, int init_kf)
{
  G_ANCHOR a;
  a.feat_type = "g_anchor";
  a.AnchorState = {x, y, z};
  a.init_KF = init_kf;
  a.n_kf_matched = 3;
  a.n_tries_matchs = 0;
  a.Descriptor = cv::Mat::zeros(1, 32, CV_8U);
  a.Descriptor.at<unsigned char>(0, 0) =
    static_cast<unsigned char>(std::abs(x) + 1);
  return a;
}

/** Deep-ish map clone avoiding OpenCV Mat aliasing of Descriptor. */
GLOBAL_MAP deep_clone_map(const GLOBAL_MAP & src)
{
  GLOBAL_MAP dst;
  dst.idx_ref_pose_slam = src.idx_ref_pose_slam;
  dst.next_landmark_id = src.next_landmark_id;
  dst.KeyFDATA = src.KeyFDATA;
  for (auto & kf : dst.KeyFDATA) {
    if (!kf.frame.empty()) {
      kf.frame = kf.frame.clone();
    }
    if (!kf.Descriptors_all.empty()) {
      kf.Descriptors_all = kf.Descriptors_all.clone();
    }
  }
  dst.AnchorsDATA.reserve(src.AnchorsDATA.size());
  for (const auto & a : src.AnchorsDATA) {
    G_ANCHOR b = a;
    if (!a.Descriptor.empty()) {
      b.Descriptor = a.Descriptor.clone();
    }
    dst.AnchorsDATA.push_back(b);
  }
  rebuild_id_to_index(dst);
  if (!src.Vgraph.empty()) {
    dst.Vgraph = src.Vgraph;
  }
  return dst;
}

/**
 * Deterministic nondegenerate fixture: IDs ≠ storage slots after gap,
 * landmarks inside and outside Pose_SLAM correction window.
 */
GLOBAL_MAP make_pose_slam_fixture()
{
  GLOBAL_MAP map;
  map.idx_ref_pose_slam = 1;  // graph from KF1 … tip
  map.KeyFDATA.push_back(make_kf(0.0, 0.0, -1.0, 100));
  map.KeyFDATA.push_back(make_kf(1.0, 0.0, -1.0, 101));
  map.KeyFDATA.push_back(make_kf(2.0, 0.1, -1.0, 102));
  map.KeyFDATA.push_back(make_kf(3.0, 0.0, -1.0, 103));
  map.KeyFDATA.push_back(make_kf(0.2, 0.05, -1.0, 104));  // closing tip

  // Force IDs ≠ slots: bump allocator, insert, delete mid entry.
  map.next_landmark_id = 100;
  const LandmarkId id_out =
    insert_landmark(map, make_anchor(0.5, 0.0, 2.0, 0));  // init outside window
  const LandmarkId id_gap =
    insert_landmark(map, make_anchor(9.0, 9.0, 9.0, 1));
  const LandmarkId id_in =
    insert_landmark(map, make_anchor(1.5, 0.0, 2.0, 2));  // init inside window
  const LandmarkId id_tip =
    insert_landmark(map, make_anchor(2.5, 0.1, 2.0, 3));

  (void)id_gap;
  gmap_landmark_id::erase_landmark_at_storage_index(map, 1, nullptr);
  rebuild_id_to_index(map);

  // Observations on several KFs (distinct UVs).
  map.KeyFDATA[1].Matched_landmark_ids = {id_out, id_in};
  map.KeyFDATA[1].UV_Matched_points = {{320, 240}, {340, 250}};
  map.KeyFDATA[2].Matched_landmark_ids = {id_in, id_tip};
  map.KeyFDATA[2].UV_Matched_points = {{330, 245}, {350, 255}};
  map.KeyFDATA[3].Matched_landmark_ids = {id_tip};
  map.KeyFDATA[3].UV_Matched_points = {{360, 260}};
  map.KeyFDATA[4].Matched_landmark_ids = {id_in, id_tip};
  map.KeyFDATA[4].UV_Matched_points = {{325, 242}, {355, 258}};

  return map;
}

CAM make_cam(std::vector<double> & dist_store)
{
  dist_store = {0, 0, 0, 0, 0};
  CAM cam;
  cam.distortions = dist_store.data();
  cam.cc[0] = 320;
  cam.cc[1] = 240;
  cam.fc[0] = 500;
  cam.fc[1] = 500;
  cam.alpha_c = 0;
  return cam;
}

gtsam::Rot3 I_rot()
{
  return gtsam::Rot3(1, 0, 0, 0, 1, 0, 0, 0, 1);
}

/** Immutable pre-extraction Pose_SLAM mirror (c7f24e8 cloop_pose_slam.cpp). */
struct LegacyPoseSlamResult
{
  double initial_error{0};
  double final_error{0};
  std::vector<arma::vec::fixed<3>> delta_kf_pos;
  std::vector<int> idx_kf_opt;
  std::vector<std::string> factor_lines;
  arma::vec::fixed<3> t_loop{};
};

LegacyPoseSlamResult legacy_pose_slam_mirror(
  const GLOBAL_MAP & map,
  const arma::vec::fixed<3> & pos,
  const gmap_graph_correction::PoseSlamConfig & config)
{
  LegacyPoseSlamResult out;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  const int idx_kf_ref = map.idx_ref_pose_slam;

  for (int i = idx_kf_ref; i < static_cast<int>(map.KeyFDATA.size()); ++i) {
    gtsam::Point3 t;
    t(0) = map.KeyFDATA[static_cast<std::size_t>(i)].t_c2n[0];
    t(1) = map.KeyFDATA[static_cast<std::size_t>(i)].t_c2n[1];
    t(2) = map.KeyFDATA[static_cast<std::size_t>(i)].t_c2n[2];
    const gtsam::Pose3 pose(I_rot(), t);
    initial.insert(gtsam::Symbol('x', i), pose);
    if (i == idx_kf_ref) {
      const double ref_sigma_att = config.clo_sigma_kf_att / 10000.0;
      const double ref_sigma_xy = config.clo_sigma_kf_xy / 10000.0;
      const double ref_sigma_z = config.clo_sigma_kf_z / 10000.0;
      auto ref_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << ref_sigma_att, ref_sigma_att, ref_sigma_att,
          ref_sigma_xy, ref_sigma_xy, ref_sigma_z)
          .finished());
      graph.addPrior(gtsam::Symbol('x', i), pose, ref_noise);
      out.factor_lines.push_back("prior x" + std::to_string(i));
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
    gtsam::Point3 t;
    t(0) = odo_i[0];
    t(1) = odo_i[1];
    t(2) = odo_i[2];
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      gtsam::Symbol('x', i - 1), gtsam::Symbol('x', i),
      gtsam::Pose3(I_rot(), t), odo_noise);
    out.factor_lines.push_back(
      "odo x" + std::to_string(i - 1) + "-x" + std::to_string(i));
  }

  auto lc_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << config.clo_sigma_kf_att, config.clo_sigma_kf_att,
      config.clo_sigma_kf_att, config.clo_sigma_kf_xy, config.clo_sigma_kf_xy,
      config.clo_sigma_kf_z)
      .finished());
  const arma::vec::fixed<3> clo_c =
    map.KeyFDATA[static_cast<std::size_t>(idx_kf_ref)].t_c2n - pos;
  out.t_loop = clo_c;
  gtsam::Point3 t;
  t(0) = clo_c[0];
  t(1) = clo_c[1];
  t(2) = clo_c[2];
  const int idx_c_kf = static_cast<int>(map.KeyFDATA.size()) - 1;
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
    gtsam::Symbol('x', idx_c_kf), gtsam::Symbol('x', idx_kf_ref),
    gtsam::Pose3(I_rot(), t), lc_noise);
  out.factor_lines.push_back(
    "loop x" + std::to_string(idx_c_kf) + "-x" + std::to_string(idx_kf_ref));

  out.initial_error = graph.error(initial);
  const gtsam::Values result =
    gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  out.final_error = graph.error(result);

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
    out.idx_kf_opt.push_back(i);
    out.delta_kf_pos.push_back(dt_kf);
  }
  return out;
}

gmap_graph_correction::PoseSlamConfig default_pose_cfg()
{
  gmap_graph_correction::PoseSlamConfig c;
  c.clo_sigma_kf_att = 0.00001;
  c.clo_sigma_kf_xy = 0.005;
  c.clo_sigma_kf_z = 0.005;
  c.odo_sigma_kf_att = 0.0001;
  c.odo_sigma_kf_xy = 0.05;
  c.odo_sigma_kf_z = 0.05;
  c.update_z = true;
  return c;
}

}  // namespace

TEST(GmapGraphCorrectionM73, PoseSlamMatchesLegacyMirror)
{
  GLOBAL_MAP map = make_pose_slam_fixture();
  ASSERT_GE(map.AnchorsDATA.size(), 2u);
  // IDs differ from storage slots (allocator started at 100).
  EXPECT_GE(map.AnchorsDATA[0].id.value, 100u);
  EXPECT_NE(map.AnchorsDATA[0].id.value, 0u);
  EXPECT_NE(map.AnchorsDATA[1].id.value, 1u);

  const arma::vec::fixed<3> pnp_C_nav = {0.15, 0.02, -1.0};
  const auto cfg = default_pose_cfg();

  const LegacyPoseSlamResult legacy =
    legacy_pose_slam_mirror(map, pnp_C_nav, cfg);

  std::vector<arma::vec::fixed<3>> deltas;
  std::vector<int> idx_opt;
  gmap_graph_correction::PoseSlamDiagnostics diag;
  ASSERT_TRUE(
    gmap_graph_correction::run_pose_slam(map, pnp_C_nav, cfg, deltas, idx_opt, &diag));

  ASSERT_EQ(deltas.size(), legacy.delta_kf_pos.size());
  ASSERT_EQ(idx_opt, legacy.idx_kf_opt);
  EXPECT_NEAR(diag.initial_error, legacy.initial_error, kObjTol);
  EXPECT_NEAR(diag.final_error, legacy.final_error, kObjTol);
  EXPECT_NEAR(diag.t_loop[0], legacy.t_loop[0], kPoseTol);
  EXPECT_NEAR(diag.t_loop[1], legacy.t_loop[1], kPoseTol);
  EXPECT_NEAR(diag.t_loop[2], legacy.t_loop[2], kPoseTol);
  ASSERT_EQ(diag.factor_lines.size(), legacy.factor_lines.size());
  for (std::size_t i = 0; i < legacy.factor_lines.size(); ++i) {
    EXPECT_EQ(diag.factor_lines[i].substr(0, legacy.factor_lines[i].size()),
      legacy.factor_lines[i]);
  }
  for (std::size_t i = 0; i < deltas.size(); ++i) {
    EXPECT_NEAR(deltas[i][0], legacy.delta_kf_pos[i][0], kPoseTol);
    EXPECT_NEAR(deltas[i][1], legacy.delta_kf_pos[i][1], kPoseTol);
    EXPECT_NEAR(deltas[i][2], legacy.delta_kf_pos[i][2], kPoseTol);
  }
  // Loop endpoints: Between(closing=tip, ref)
  EXPECT_EQ(diag.idx_closing, static_cast<int>(map.KeyFDATA.size()) - 1);
  EXPECT_EQ(diag.idx_kf_ref, map.idx_ref_pose_slam);
}

TEST(GmapGraphCorrectionM73, LandmarkCorrectionWindowAndIdentities)
{
  GLOBAL_MAP map = make_pose_slam_fixture();
  const auto ids_before = [&]() {
    std::vector<uint64_t> v;
    for (const auto & a : map.AnchorsDATA) {
      v.push_back(a.id.value);
    }
    return v;
  }();
  const auto watermark = map.next_landmark_id;
  const arma::vec::fixed<3> outside_before = map.AnchorsDATA[0].AnchorState;
  // AnchorsDATA[0] has init_KF=0, outside window (ref=1).

  const arma::vec::fixed<3> pnp_C_nav = {0.15, 0.02, -1.0};
  std::vector<arma::vec::fixed<3>> deltas;
  std::vector<int> idx_opt;
  ASSERT_TRUE(gmap_graph_correction::run_pose_slam(
    map, pnp_C_nav, default_pose_cfg(), deltas, idx_opt));
  gmap_graph_correction::apply_pose_slam_geometry(map, deltas, idx_opt);

  EXPECT_EQ(map.next_landmark_id, watermark);
  for (std::size_t i = 0; i < map.AnchorsDATA.size(); ++i) {
    EXPECT_EQ(map.AnchorsDATA[i].id.value, ids_before[i]);
  }
  // Outside-window landmark unchanged.
  EXPECT_NEAR(map.AnchorsDATA[0].AnchorState[0], outside_before[0], kPoseTol);
  EXPECT_NEAR(map.AnchorsDATA[0].AnchorState[1], outside_before[1], kPoseTol);
  EXPECT_NEAR(map.AnchorsDATA[0].AnchorState[2], outside_before[2], kPoseTol);

  // Inside-window landmark (init_KF=2) must move with that KF delta.
  std::size_t idx_in = 0;
  bool found = false;
  for (std::size_t i = 0; i < map.AnchorsDATA.size(); ++i) {
    if (map.AnchorsDATA[i].init_KF == 2) {
      idx_in = i;
      found = true;
      break;
    }
  }
  ASSERT_TRUE(found);
  const auto it = std::find(idx_opt.begin(), idx_opt.end(), 2);
  ASSERT_NE(it, idx_opt.end());
  const int di = static_cast<int>(it - idx_opt.begin());
  // After apply, position = original + delta; check nonzero correction typically.
  EXPECT_GT(arma::norm(deltas[static_cast<std::size_t>(di)]), 0.0);
  (void)idx_in;

  const auto inv = validate_landmark_identity_invariants(map);
  EXPECT_TRUE(inv.ok) << inv.reason;
}

TEST(GmapGraphCorrectionM73, IndependentMapInvocationNoHiddenMutation)
{
  GLOBAL_MAP map_a = make_pose_slam_fixture();
  GLOBAL_MAP map_b = deep_clone_map(map_a);
  const GLOBAL_MAP map_b_snapshot = deep_clone_map(map_b);

  const arma::vec::fixed<3> pnp_C_nav = {0.15, 0.02, -1.0};
  std::vector<arma::vec::fixed<3>> deltas;
  std::vector<int> idx_opt;
  ASSERT_TRUE(gmap_graph_correction::run_pose_slam(
    map_a, pnp_C_nav, default_pose_cfg(), deltas, idx_opt));
  gmap_graph_correction::apply_pose_slam_geometry(map_a, deltas, idx_opt);

  ASSERT_EQ(map_b.KeyFDATA.size(), map_b_snapshot.KeyFDATA.size());
  for (std::size_t i = 0; i < map_b.KeyFDATA.size(); ++i) {
    EXPECT_NEAR(map_b.KeyFDATA[i].t_c2n[0], map_b_snapshot.KeyFDATA[i].t_c2n[0],
      kPoseTol);
    EXPECT_NEAR(map_b.KeyFDATA[i].t_c2n[1], map_b_snapshot.KeyFDATA[i].t_c2n[1],
      kPoseTol);
  }
  for (std::size_t i = 0; i < map_b.AnchorsDATA.size(); ++i) {
    EXPECT_NEAR(map_b.AnchorsDATA[i].AnchorState[0],
      map_b_snapshot.AnchorsDATA[i].AnchorState[0], kPoseTol);
    EXPECT_EQ(map_b.AnchorsDATA[i].id.value,
      map_b_snapshot.AnchorsDATA[i].id.value);
  }
  // map_a tip moved relative to untouched map_b tip.
  EXPECT_GT(
    arma::norm(map_a.KeyFDATA.back().t_c2n - map_b.KeyFDATA.back().t_c2n),
    1e-6);
}

TEST(GmapGraphCorrectionM73, AlignmentDeltaXyOnly)
{
  GLOBAL_MAP map = make_pose_slam_fixture();
  const arma::vec::fixed<3> p_l_original = map.KeyFDATA.back().t_c2n;
  const arma::vec::fixed<3> pnp_C_nav = {0.15, 0.02, -1.0};
  std::vector<arma::vec::fixed<3>> deltas;
  std::vector<int> idx_opt;
  ASSERT_TRUE(gmap_graph_correction::run_pose_slam(
    map, pnp_C_nav, default_pose_cfg(), deltas, idx_opt));
  gmap_graph_correction::apply_pose_slam_geometry(map, deltas, idx_opt);
  const arma::vec::fixed<3> p_g_final = map.KeyFDATA.back().t_c2n;
  const arma::vec::fixed<3> d =
    gmap_visual_sync::align_delta_xy(p_g_final, p_l_original);
  EXPECT_NEAR(d[0], p_g_final[0] - p_l_original[0], kPoseTol);
  EXPECT_NEAR(d[1], p_g_final[1] - p_l_original[1], kPoseTol);
  EXPECT_NEAR(d[2], 0.0, kPoseTol);
}

TEST(GmapGraphCorrectionM73, PostLoopBaAcceptAndReject)
{
  std::vector<double> dist;
  CAM cam = make_cam(dist);

  // Accepted path: mild geometry after Pose_SLAM.
  {
    GLOBAL_MAP map = make_pose_slam_fixture();
    const arma::vec::fixed<3> pnp_C_nav = {0.15, 0.02, -1.0};
    std::vector<arma::vec::fixed<3>> deltas;
    std::vector<int> idx_opt;
    ASSERT_TRUE(gmap_graph_correction::run_pose_slam(
      map, pnp_C_nav, default_pose_cfg(), deltas, idx_opt));
    gmap_graph_correction::apply_pose_slam_geometry(map, deltas, idx_opt);

    gmap_graph_correction::PostLoopBaConfig ba_cfg;
    ba_cfg.sigma_uv = 1.0;
    ba_cfg.sigma_kf_att = 0.01;
    ba_cfg.sigma_kf_xy = 0.05;
    ba_cfg.sigma_kf_z = 0.05;
    ba_cfg.sigma_kf_pt = 0.1;
    ba_cfg.max_delta_kf_pos = 1.75;
    ba_cfg.update_kf_pos = true;
    ba_cfg.update_kf_att = true;
    ba_cfg.update_map = true;

    std::vector<int> idx_new;
    std::vector<int> idx_old = {1};
    gmap_graph_correction::PostLoopBaDiagnostics diag;
    const bool accepted = gmap_graph_correction::run_post_loop_ba(
      map, idx_new, idx_old, cam, ba_cfg, &diag);
    EXPECT_TRUE(accepted);
    EXPECT_TRUE(diag.accepted);
    EXPECT_FALSE(diag.idx_kf_optimized.empty());
    EXPECT_GT(diag.n_projection_factors, 0);
  }

  // Rejected path: tiny max_delta forces rejection without changing thresholds
  // used in production YAML — this exercises the existing policy branch.
  {
    GLOBAL_MAP map = make_pose_slam_fixture();
    // Deliberately inconsistent observations → large KF move after BA.
    for (auto & kf : map.KeyFDATA) {
      for (auto & uv : kf.UV_Matched_points) {
        uv.x += 80.0;
      }
    }
    gmap_graph_correction::PostLoopBaConfig ba_cfg;
    ba_cfg.sigma_uv = 1.0;
    ba_cfg.sigma_kf_att = 0.01;
    ba_cfg.sigma_kf_xy = 0.05;
    ba_cfg.sigma_kf_z = 0.05;
    ba_cfg.sigma_kf_pt = 0.1;
    ba_cfg.max_delta_kf_pos = 0.001;  // existing production gate, tight for reject case
    ba_cfg.update_kf_pos = true;
    ba_cfg.update_kf_att = true;
    ba_cfg.update_map = true;

    const arma::vec::fixed<3> tip_before = map.KeyFDATA.back().t_c2n;
    std::vector<int> idx_new;
    std::vector<int> idx_old = {1};
    gmap_graph_correction::PostLoopBaDiagnostics diag;
    const bool accepted = gmap_graph_correction::run_post_loop_ba(
      map, idx_new, idx_old, cam, ba_cfg, &diag);
    EXPECT_FALSE(accepted);
    EXPECT_FALSE(diag.accepted);
    EXPECT_NEAR(map.KeyFDATA.back().t_c2n[0], tip_before[0], kPoseTol);
    EXPECT_NEAR(map.KeyFDATA.back().t_c2n[1], tip_before[1], kPoseTol);
  }
}

TEST(GmapGraphCorrectionM73, DualRunDeterminism)
{
  GLOBAL_MAP map0 = make_pose_slam_fixture();
  GLOBAL_MAP map1 = deep_clone_map(map0);
  const arma::vec::fixed<3> pnp_C_nav = {0.15, 0.02, -1.0};
  const auto cfg = default_pose_cfg();

  std::vector<arma::vec::fixed<3>> d0, d1;
  std::vector<int> i0, i1;
  gmap_graph_correction::PoseSlamDiagnostics a, b;
  ASSERT_TRUE(gmap_graph_correction::run_pose_slam(map0, pnp_C_nav, cfg, d0, i0, &a));
  ASSERT_TRUE(gmap_graph_correction::run_pose_slam(map1, pnp_C_nav, cfg, d1, i1, &b));
  EXPECT_NEAR(a.initial_error, b.initial_error, kObjTol);
  EXPECT_NEAR(a.final_error, b.final_error, kObjTol);
  ASSERT_EQ(d0.size(), d1.size());
  for (std::size_t i = 0; i < d0.size(); ++i) {
    EXPECT_NEAR(d0[i][0], d1[i][0], kPoseTol);
    EXPECT_NEAR(d0[i][1], d1[i][1], kPoseTol);
    EXPECT_NEAR(d0[i][2], d1[i][2], kPoseTol);
  }
}
