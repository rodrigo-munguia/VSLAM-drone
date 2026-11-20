/**
 * GSLAM-VISUAL-LOOP-A2d — retrieval / ranking / fallback orchestration tests.
 * Exercises production helpers in cloop_visual_retrieve.hpp (+ A2b PnP).
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <armadillo>

#include "../../common/Vision/vision.hpp"
#include "cloop_visual_pnp.hpp"
#include "cloop_visual_retrieve.hpp"

using globalslam::cloop_visual_retrieve::RankedCandidate;
using globalslam::cloop_visual_retrieve::compute_significance_metrics;
using globalslam::cloop_visual_retrieve::enumerate_unlinked_historical;
using globalslam::cloop_visual_retrieve::euclidean_kf_separation_m;
using globalslam::cloop_visual_retrieve::filter_min_historical_travel;
using globalslam::cloop_visual_retrieve::historical_path_distance_m;
using globalslam::cloop_visual_retrieve::kf_index_separation;
using globalslam::cloop_visual_retrieve::max_attempts_clamped;
using globalslam::cloop_visual_retrieve::positions_with_current_endpoint;
using globalslam::cloop_visual_retrieve::rank_candidates_inplace;
using globalslam::cloop_visual_pnp::solve_and_accept;

namespace
{

arma::mat make_vgraph(int n)
{
  arma::mat V(n, n, arma::fill::zeros);
  return V;
}

}  // namespace

TEST(VisualLoopA2d, EnumerateIncludesMidRouteDespiteEarlierLink)
{
  // KF0..KF40, current=40, BA window=10 → limit=31
  constexpr int n = 41;
  arma::mat V = make_vgraph(n);
  V(40, 8) = 2;   // earlier link must NOT stop search
  V(40, 30) = 5;  // inside search limit
  const auto idxs = enumerate_unlinked_historical(V, 40, 31);
  EXPECT_TRUE(std::find(idxs.begin(), idxs.end(), 20) != idxs.end());
  EXPECT_TRUE(std::find(idxs.begin(), idxs.end(), 8) == idxs.end());
  EXPECT_TRUE(std::find(idxs.begin(), idxs.end(), 7) != idxs.end());
  // Contiguous prefix break is gone: 9..29 with V==0 are included
  EXPECT_TRUE(std::find(idxs.begin(), idxs.end(), 9) != idxs.end());
  EXPECT_TRUE(std::find(idxs.begin(), idxs.end(), 29) != idxs.end());
  EXPECT_TRUE(std::find(idxs.begin(), idxs.end(), 30) == idxs.end());
}

TEST(VisualLoopA2d, EnumerateHomeNotRequired)
{
  arma::mat V = make_vgraph(41);
  V(40, 30) = 1;
  const auto idxs = enumerate_unlinked_historical(V, 40, 31);
  EXPECT_TRUE(std::find(idxs.begin(), idxs.end(), 20) != idxs.end());
  EXPECT_GT(idxs.size(), 5u);
}

TEST(VisualLoopA2d, RankStrongerLaterCandidateFirst)
{
  std::vector<RankedCandidate> cands(2);
  cands[0].idx_kf = 5;
  cands[0].n_matches = 45;  // weaker earlier
  cands[1].idx_kf = 20;
  cands[1].n_matches = 80;  // stronger mid-route
  rank_candidates_inplace(cands);
  EXPECT_EQ(cands[0].idx_kf, 20);
  EXPECT_EQ(cands[1].idx_kf, 5);
}

TEST(VisualLoopA2d, RankTieBreakLowerIndex)
{
  std::vector<RankedCandidate> cands(2);
  cands[0].idx_kf = 20;
  cands[0].n_matches = 50;
  cands[1].idx_kf = 10;
  cands[1].n_matches = 50;
  rank_candidates_inplace(cands);
  EXPECT_EQ(cands[0].idx_kf, 10);
}

TEST(VisualLoopA2d, MaxPnpAttemptsBounded)
{
  EXPECT_EQ(max_attempts_clamped(3, 10), 3);
  EXPECT_EQ(max_attempts_clamped(3, 2), 2);
  EXPECT_EQ(max_attempts_clamped(0, 5), 1);
}

TEST(VisualLoopA2d, PnpFallbackSkipsFailingTopCandidate)
{
  // Top-ranked has too few correspondences (PnP reject); second is valid.
  // Mirrors Step's ranked fallback loop over solve_and_accept.
  CAM cam;
  static double D[5] = {0, 0, 0, 0, 0};
  cam.distortions = D;
  cam.fc[0] = 200;
  cam.fc[1] = 200;
  cam.cc[0] = 200;
  cam.cc[1] = 120;
  cam.alpha_c = 0;

  arma::mat::fixed<3, 3> Rn2c = arma::eye<arma::mat>(3, 3);
  arma::vec::fixed<3> C = {0.5, -0.2, 0.1};

  std::vector<cv::Point3f> pts3d = {
    {0.f, 0.f, 4.f}, {1.f, 0.5f, 5.f}, {-1.f, 0.3f, 4.5f},
    {0.5f, -0.8f, 6.f}, {1.2f, 1.f, 5.5f}, {-0.7f, -0.4f, 4.2f},
    {0.2f, 0.9f, 5.2f}, {0.8f, -0.2f, 4.8f}};
  std::vector<cv::Point2f> good2d;
  arma::mat::fixed<2, 3> J;
  for (const auto & p : pts3d) {
    arma::vec::fixed<3> Pw = {p.x, p.y, p.z};
    arma::vec::fixed<3> pc = Rn2c * (Pw - C);
    cv::Point2d uv = Projection_model(pc, 0, false, cam, J);
    good2d.emplace_back(static_cast<float>(uv.x), static_cast<float>(uv.y));
  }

  struct FakeCand {
    int idx_kf;
    std::vector<cv::Point3f> p3;
    std::vector<cv::Point2f> p2;
  };
  // Ranked order: weak/insufficient first, strong second.
  std::vector<FakeCand> ordered = {
    {5, {pts3d[0], pts3d[1], pts3d[2]}, {good2d[0], good2d[1], good2d[2]}},
    {20, pts3d, good2d},
  };

  int accepted_kf = -1;
  int attempts = 0;
  const int n_try = max_attempts_clamped(3, static_cast<int>(ordered.size()));
  for (int i = 0; i < n_try; ++i) {
    ++attempts;
    const auto r = solve_and_accept(
      ordered[i].p3, ordered[i].p2, Rn2c, C, cam, 2.0, 6.0, 70.0);
    if (r.accepted) {
      accepted_kf = ordered[i].idx_kf;
      break;
    }
    EXPECT_FALSE(r.accepted);
  }
  EXPECT_EQ(attempts, 2);
  EXPECT_EQ(accepted_kf, 20);
}

TEST(VisualLoopA2d, NoValidCandidateNoAccept)
{
  CAM cam;
  static double D[5] = {0, 0, 0, 0, 0};
  cam.distortions = D;
  cam.fc[0] = 200;
  cam.fc[1] = 200;
  cam.cc[0] = 200;
  cam.cc[1] = 120;
  cam.alpha_c = 0;
  arma::mat::fixed<3, 3> Rn2c = arma::eye<arma::mat>(3, 3);
  arma::vec::fixed<3> C = {0, 0, 0};
  std::vector<cv::Point3f> pts3d = {
    {0, 0, 4}, {1, 0, 5}, {-1, 0, 4}, {0, 1, 5}, {1, 1, 6}, {-1, -1, 5},
    {0.5, 0.5, 4.5}, {-0.5, 0.5, 5.5}};
  std::vector<cv::Point2f> bad;
  for (size_t i = 0; i < pts3d.size(); ++i) {
    bad.emplace_back(10.f + 20.f * i, 200.f);
  }
  const auto r = solve_and_accept(pts3d, bad, Rn2c, C, cam, 2.0, 6.0, 70.0);
  EXPECT_FALSE(r.accepted);
}

TEST(VisualLoopA2d, MultipleClosuresNoReactivateSemantics)
{
  // Documented contract: travel_dis re-arm + autonomous commit (no activate_CL).
  // Helper-level: two independent successful PnP accepts are independent of a
  // sticky arm flag (flag not part of solve_and_accept).
  CAM cam;
  static double D[5] = {0, 0, 0, 0, 0};
  cam.distortions = D;
  cam.fc[0] = 200;
  cam.fc[1] = 200;
  cam.cc[0] = 200;
  cam.cc[1] = 120;
  cam.alpha_c = 0;
  arma::mat::fixed<3, 3> Rn2c = arma::eye<arma::mat>(3, 3);
  arma::vec::fixed<3> C1 = {0.4, 0.0, 0.0};
  arma::vec::fixed<3> C2 = {1.0, -0.3, 0.2};
  std::vector<cv::Point3f> pts3d = {
    {0, 0, 4}, {1, 0, 5}, {-1, 0, 4}, {0, 1, 5}, {1, 1, 6}, {-1, -1, 5},
    {0.5, 0.5, 4.5}, {-0.5, 0.5, 5.5}};

  auto project = [&](const arma::vec::fixed<3> & C) {
    std::vector<cv::Point2f> uvs;
    arma::mat::fixed<2, 3> J;
    for (const auto & p : pts3d) {
      arma::vec::fixed<3> Pw = {p.x, p.y, p.z};
      arma::vec::fixed<3> pc = Rn2c * (Pw - C);
      cv::Point2d uv = Projection_model(pc, 0, false, cam, J);
      uvs.emplace_back(static_cast<float>(uv.x), static_cast<float>(uv.y));
    }
    return uvs;
  };

  const auto r1 = solve_and_accept(
    pts3d, project(C1), Rn2c, C1, cam, 2.0, 6.0, 70.0);
  const auto r2 = solve_and_accept(
    pts3d, project(C2), Rn2c, C2, cam, 2.0, 6.0, 70.0);
  EXPECT_TRUE(r1.accepted);
  EXPECT_TRUE(r2.accepted);
  EXPECT_LT(arma::norm(r1.C_nav - C1), 0.05);
  EXPECT_LT(arma::norm(r2.C_nav - C2), 0.05);
}

TEST(VisualLoopA2d, SignificancePathNotEuclid)
{
  // A3f: historical path ≠ Euclidean proximity (L-shaped chain).
  std::vector<std::array<double, 3>> pos = {
    {0, 0, 0}, {10, 0, 0}, {20, 0, 0}, {20, 10, 0}, {20, 20, 0},
  };
  EXPECT_EQ(kf_index_separation(4, 0), 4);
  EXPECT_DOUBLE_EQ(historical_path_distance_m(pos, 0, 4), 40.0);
  EXPECT_NEAR(euclidean_kf_separation_m(pos, 0, 4), std::sqrt(800.0), 1e-9);
  const auto m = compute_significance_metrics(pos, 4, 0);
  EXPECT_EQ(m.idx_sep, 4);
  EXPECT_DOUBLE_EQ(m.hist_path_m, 40.0);
  EXPECT_NEAR(m.euclid_m, std::sqrt(800.0), 1e-9);
  EXPECT_LT(0.0, historical_path_distance_m(pos, 1, 2));
  EXPECT_DOUBLE_EQ(historical_path_distance_m(pos, 2, 2), 0.0);
  EXPECT_DOUBLE_EQ(historical_path_distance_m(pos, 3, 1), 20.0);  // swapped 1↔3
  EXPECT_LT(historical_path_distance_m(pos, 0, 99), 0.0);  // invalid
}

namespace
{

/** Uniform spacing along +X; positions[i] = (i * step, 0, 0). */
std::vector<std::array<double, 3>> make_linear_path(int n, double step)
{
  std::vector<std::array<double, 3>> pos;
  pos.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    pos.push_back({static_cast<double>(i) * step, 0.0, 0.0});
  }
  return pos;
}

}  // namespace

TEST(VisualLoopA3g, TrivialA3eKf76Vs66Excluded)
{
  // A3e: KF66→76 ≈ 106.4 m along chain; gate 200 m must exclude.
  constexpr double step = 10.63786028562046;  // 10 * step = 106.3786
  auto mapped = make_linear_path(77, step);   // indices 0..76
  const std::array<double, 3> cur =
    {mapped[76][0] + 5.0, mapped[76][1], mapped[76][2]};  // unappended CL
  bool appended = false;
  const auto pos = positions_with_current_endpoint(mapped, cur, true, &appended);
  EXPECT_TRUE(appended);
  const int endpoint = static_cast<int>(pos.size()) - 1;
  const double s = historical_path_distance_m(pos, 66, endpoint);
  EXPECT_NEAR(s, 106.3786 + 5.0, 0.05);
  int n_rej = 0;
  const auto kept = filter_min_historical_travel(
    {66}, pos, endpoint, 200.0, &n_rej);
  EXPECT_TRUE(kept.empty());
  EXPECT_EQ(n_rej, 1);
}

TEST(VisualLoopA3g, GenuineReturnToStartEligible)
{
  // Out-and-back: KF5 and KF140 spatially close, hist-path ≈ 1.5 km.
  std::vector<std::array<double, 3>> mapped(141, {0.0, 0.0, 0.0});
  mapped[5] = {0.0, 0.0, 0.0};
  const double seg = 1500.0 / 135.0;  // 135 segments from 5→140
  double x = 0.0;
  for (int i = 6; i <= 140; ++i) {
    if (i <= 72) {
      x += seg;  // outbound
    } else {
      x -= seg;  // return toward KF5
    }
    mapped[static_cast<size_t>(i)] = {x, 0.0, 0.0};
  }
  EXPECT_NEAR(mapped[5][0], mapped[140][0], 15.0);
  const double path = historical_path_distance_m(mapped, 5, 140);
  EXPECT_NEAR(path, 1500.0, 1e-6);
  int n_rej = 0;
  const auto kept = filter_min_historical_travel(
    {5}, mapped, 140, 200.0, &n_rej);
  EXPECT_EQ(kept.size(), 1u);
  EXPECT_EQ(kept[0], 5);
  EXPECT_EQ(n_rej, 0);
  EXPECT_LT(euclidean_kf_separation_m(mapped, 5, 140), 20.0);
}

TEST(VisualLoopA3g, MidRouteRevisitEligible)
{
  auto mapped = make_linear_path(80, 10.0);  // 0..79
  // Revisit KF20 after traveling to KF50 (300 m) — mid-route.
  const double s = historical_path_distance_m(mapped, 20, 50);
  EXPECT_DOUBLE_EQ(s, 300.0);
  int n_rej = 0;
  const auto kept = filter_min_historical_travel(
    {20, 45}, mapped, 50, 200.0, &n_rej);
  EXPECT_EQ(kept.size(), 1u);
  EXPECT_EQ(kept[0], 20);  // 45→50 is only 50 m → rejected
  EXPECT_EQ(n_rej, 1);
}

TEST(VisualLoopA3g, BoundaryExactly200Eligible)
{
  // 20 steps × 10 m = 200 m exactly.
  auto pos = make_linear_path(21, 10.0);
  EXPECT_DOUBLE_EQ(historical_path_distance_m(pos, 0, 20), 200.0);
  int n_rej = 0;
  auto kept = filter_min_historical_travel({0}, pos, 20, 200.0, &n_rej);
  EXPECT_EQ(kept.size(), 1u);
  EXPECT_EQ(n_rej, 0);
  // Just below: 19×10 = 190 → excluded
  kept = filter_min_historical_travel({0}, pos, 19, 200.0, &n_rej);
  EXPECT_TRUE(kept.empty());
  EXPECT_EQ(n_rej, 1);
  // Above: 21×10 with extra point
  pos.push_back({210.0, 0.0, 0.0});
  kept = filter_min_historical_travel({0}, pos, 21, 200.0, &n_rej);
  EXPECT_EQ(kept.size(), 1u);
}

TEST(VisualLoopA3g, PathNotEuclidReturnHome)
{
  // Out-and-back to origin: Euclid≈0, hist-path=200.
  std::vector<std::array<double, 3>> pos = {
    {0, 0, 0}, {50, 0, 0}, {100, 0, 0}, {50, 0, 0}, {0, 0, 0},
  };
  EXPECT_DOUBLE_EQ(historical_path_distance_m(pos, 0, 4), 200.0);
  EXPECT_NEAR(euclidean_kf_separation_m(pos, 0, 4), 0.0, 1e-12);
  int n_rej = 0;
  const auto kept = filter_min_historical_travel({0}, pos, 4, 200.0, &n_rej);
  EXPECT_EQ(kept.size(), 1u);
  EXPECT_EQ(n_rej, 0);
}

TEST(VisualLoopA3g, CurrentKfNotAppendedUsesEndpoint)
{
  auto mapped = make_linear_path(10, 10.0);  // 0..9 → 90 m to last
  const std::array<double, 3> cur = {200.0, 0.0, 0.0};  // +110 m beyond last
  bool appended = false;
  const auto pos =
    positions_with_current_endpoint(mapped, cur, true, &appended);
  EXPECT_TRUE(appended);
  EXPECT_EQ(pos.size(), 11u);
  const double s = historical_path_distance_m(pos, 0, 10);
  EXPECT_DOUBLE_EQ(s, 200.0);  // 90 + 110
  // Without append, path to last mapped is only 90 m → would exclude.
  EXPECT_DOUBLE_EQ(historical_path_distance_m(mapped, 0, 9), 90.0);
  int n_rej = 0;
  EXPECT_EQ(
    filter_min_historical_travel({0}, pos, 10, 200.0, &n_rej).size(), 1u);
  EXPECT_TRUE(
    filter_min_historical_travel({0}, mapped, 9, 200.0, &n_rej).empty());
}

TEST(VisualLoopA3g, InvalidInputsSafe)
{
  std::vector<std::array<double, 3>> empty;
  EXPECT_LT(historical_path_distance_m(empty, 0, 0), 0.0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<std::array<double, 3>> bad = {
    {0, 0, 0}, {nan, 0, 0}, {20, 0, 0},
  };
  EXPECT_LT(historical_path_distance_m(bad, 0, 2), 0.0);
  int n_rej = 0;
  const auto kept =
    filter_min_historical_travel({0, 1}, bad, 2, 200.0, &n_rej);
  EXPECT_TRUE(kept.empty());
  EXPECT_EQ(n_rej, 2);
  bool appended = true;
  const auto no_append = positions_with_current_endpoint(
    {{0, 0, 0}}, {nan, 0, 0}, true, &appended);
  EXPECT_FALSE(appended);
  EXPECT_EQ(no_append.size(), 1u);
  // Reset / empty map: filter yields empty
  EXPECT_TRUE(
    filter_min_historical_travel({}, empty, -1, 200.0, &n_rej).empty());
}

TEST(VisualLoopA3g, VisibilityRankingPnpUnchanged)
{
  // H: preserve enum / rank / PnP contracts alongside the gate.
  constexpr int n = 41;
  arma::mat V = make_vgraph(n);
  V(40, 8) = 2;
  const auto idxs = enumerate_unlinked_historical(V, 40, 31);
  EXPECT_TRUE(std::find(idxs.begin(), idxs.end(), 8) == idxs.end());
  EXPECT_TRUE(std::find(idxs.begin(), idxs.end(), 20) != idxs.end());

  std::vector<RankedCandidate> cands(2);
  cands[0].idx_kf = 5;
  cands[0].n_matches = 45;
  cands[1].idx_kf = 20;
  cands[1].n_matches = 80;
  rank_candidates_inplace(cands);
  EXPECT_EQ(cands[0].idx_kf, 20);
  EXPECT_EQ(max_attempts_clamped(3, 10), 3);
}

TEST(VisualLoopA2d, PerfEnumerateScales)
{
  for (int n : {50, 200, 500}) {
    arma::mat V = make_vgraph(n);
    // Sparse links every 17th KF
    for (int i = 0; i < n - 10; i += 17) {
      V(n - 1, i) = 1;
    }
    const auto t0 = std::chrono::high_resolution_clock::now();
    const auto idxs = enumerate_unlinked_historical(V, n - 1, n - 10);
    const auto t1 = std::chrono::high_resolution_clock::now();
    const double ms =
      std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() /
      1000.0;
    std::cout << "A2d enum n=" << n << " eligible=" << idxs.size()
              << " time_ms=" << ms << std::endl;
    EXPECT_LT(ms, 50.0);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
