/**
 * GSLAM-VISUAL-LOOP-A2d: visual historical retrieval / ranking helpers.
 *
 * Enumeration: all Vgraph(current,i)==0 for i in [0, ba_limit), no early break.
 * Ranking: descending match count, ascending KF index on ties.
 * Does not apply home / uncertainty filters (visual-specific policy).
 */

#ifndef GLOBALSLAM_CLOOP_VISUAL_RETRIEVE_HPP_
#define GLOBALSLAM_CLOOP_VISUAL_RETRIEVE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <armadillo>
#include <opencv2/core.hpp>

namespace globalslam
{
namespace cloop_visual_retrieve
{

struct RankedCandidate
{
  int idx_kf{-1};
  int n_matches{0};
  std::vector<int> idx_pt_matches;
  std::vector<cv::Point2d> image_points;
};

struct AttemptDiag
{
  int current_kf_idx{-1};
  int n_eligible{0};
  int n_hist_rejected{0};
  int n_above_threshold{0};
  int selected_kf_idx{-1};
  int selected_n_matches{0};
  int pnp_attempts{0};
  int last_n_inliers{0};
  double last_reproj_error{0};
  double last_hist_path_m{-1.0};
  const char * last_pnp_reason{"none"};
  bool accepted{false};
  bool committed{false};
  double t_search_s{0};
  double t_pnp_s{0};
};

/** All historically unlinked KFs before BA window; does not stop at first link. */
inline std::vector<int> enumerate_unlinked_historical(
  const arma::mat & Vgraph, int idx_current_kf, int idx_kf_sup_limit)
{
  std::vector<int> out;
  if (idx_kf_sup_limit < 0) {
    return out;
  }
  const int limit = std::min(idx_kf_sup_limit, static_cast<int>(Vgraph.n_cols));
  const int row = idx_current_kf;
  if (row < 0 || row >= static_cast<int>(Vgraph.n_rows)) {
    return out;
  }
  out.reserve(static_cast<size_t>(limit));
  for (int i = 0; i < limit; ++i) {
    if (static_cast<int>(Vgraph.at(row, i)) == 0) {
      out.push_back(i);
    }
  }
  return out;
}

/** Rank by n_matches desc, then idx_kf asc (deterministic). */
inline void rank_candidates_inplace(std::vector<RankedCandidate> & cands)
{
  std::stable_sort(
    cands.begin(), cands.end(),
    [](const RankedCandidate & a, const RankedCandidate & b) {
      if (a.n_matches != b.n_matches) {
        return a.n_matches > b.n_matches;
      }
      return a.idx_kf < b.idx_kf;
    });
}

inline int max_attempts_clamped(int max_pnp_attempts, int n_ranked)
{
  if (max_pnp_attempts < 1) {
    max_pnp_attempts = 1;
  }
  return std::min(max_pnp_attempts, n_ranked);
}

/** |current − matched| KF-index separation (A ≠ historical path length). */
inline int kf_index_separation(int idx_current_kf, int idx_matched_kf)
{
  return std::abs(idx_current_kf - idx_matched_kf);
}

inline bool is_finite3(const std::array<double, 3> & p)
{
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

/**
 * Chain length along consecutive KF centres from idx_lo→idx_hi.
 * positions[i] = camera centre (t_c2n) of KF i. Returns −1 if indices invalid
 * or any segment endpoint is non-finite. This is historical traveled distance
 * along the KF chain — not Euclidean current↔matched proximity, and not
 * travel_dis since last commit.
 */
inline double historical_path_distance_m(
  const std::vector<std::array<double, 3>> & positions,
  int idx_lo, int idx_hi)
{
  if (idx_lo < 0 || idx_hi < 0) {
    return -1.0;
  }
  if (idx_lo > idx_hi) {
    std::swap(idx_lo, idx_hi);
  }
  if (idx_hi >= static_cast<int>(positions.size())) {
    return -1.0;
  }
  if (idx_lo == idx_hi) {
    return is_finite3(positions[static_cast<size_t>(idx_lo)]) ? 0.0 : -1.0;
  }
  double sum = 0.0;
  for (int i = idx_lo; i < idx_hi; ++i) {
    const auto & a = positions[static_cast<size_t>(i)];
    const auto & b = positions[static_cast<size_t>(i + 1)];
    if (!is_finite3(a) || !is_finite3(b)) {
      return -1.0;
    }
    const double dx = b[0] - a[0];
    const double dy = b[1] - a[1];
    const double dz = b[2] - a[2];
    sum += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return sum;
}

/**
 * Build path centres for hist-travel: mapped KF t_c2n, optionally appending the
 * current CL query pose when it is not yet in GLOBAL_MAP (production path).
 */
inline std::vector<std::array<double, 3>> positions_with_current_endpoint(
  const std::vector<std::array<double, 3>> & mapped_centres,
  const std::array<double, 3> & current_t_c2n,
  bool append_current_endpoint,
  bool * appended_out = nullptr)
{
  std::vector<std::array<double, 3>> out = mapped_centres;
  if (appended_out) {
    *appended_out = false;
  }
  if (!append_current_endpoint) {
    return out;
  }
  if (!is_finite3(current_t_c2n)) {
    return out;
  }
  out.push_back(current_t_c2n);
  if (appended_out) {
    *appended_out = true;
  }
  return out;
}

/**
 * A3g: keep candidates with hist-path ≥ min_travel_m (endpoint = current).
 * Invalid / non-finite path (−1) is treated as rejected.
 */
inline std::vector<int> filter_min_historical_travel(
  const std::vector<int> & candidates,
  const std::vector<std::array<double, 3>> & positions,
  int idx_endpoint,
  double min_travel_m,
  int * n_rejected_out = nullptr)
{
  std::vector<int> kept;
  kept.reserve(candidates.size());
  int rejected = 0;
  for (int j : candidates) {
    const double s =
      historical_path_distance_m(positions, j, idx_endpoint);
    if (!(s >= min_travel_m)) {
      ++rejected;
      continue;
    }
    kept.push_back(j);
  }
  if (n_rejected_out) {
    *n_rejected_out = rejected;
  }
  return kept;
}

inline double euclidean_kf_separation_m(
  const std::vector<std::array<double, 3>> & positions,
  int idx_a, int idx_b)
{
  if (idx_a < 0 || idx_b < 0 ||
    idx_a >= static_cast<int>(positions.size()) ||
    idx_b >= static_cast<int>(positions.size()))
  {
    return -1.0;
  }
  const double dx =
    positions[static_cast<size_t>(idx_a)][0] -
    positions[static_cast<size_t>(idx_b)][0];
  const double dy =
    positions[static_cast<size_t>(idx_a)][1] -
    positions[static_cast<size_t>(idx_b)][1];
  const double dz =
    positions[static_cast<size_t>(idx_a)][2] -
    positions[static_cast<size_t>(idx_b)][2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

struct SignificanceMetrics
{
  int idx_sep{-1};
  double hist_path_m{-1.0};
  double euclid_m{-1.0};
};

inline SignificanceMetrics compute_significance_metrics(
  const std::vector<std::array<double, 3>> & positions,
  int idx_current_kf, int idx_matched_kf)
{
  SignificanceMetrics m;
  m.idx_sep = kf_index_separation(idx_current_kf, idx_matched_kf);
  m.hist_path_m =
    historical_path_distance_m(positions, idx_matched_kf, idx_current_kf);
  m.euclid_m =
    euclidean_kf_separation_m(positions, idx_matched_kf, idx_current_kf);
  return m;
}

}  // namespace cloop_visual_retrieve
}  // namespace globalslam

#endif  // GLOBALSLAM_CLOOP_VISUAL_RETRIEVE_HPP_
