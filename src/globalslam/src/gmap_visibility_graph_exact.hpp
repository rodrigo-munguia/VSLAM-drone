/**
 * GSLAM-LC-POLICY-A1.1 — Exact visibility graph contract and reconstruction.
 *
 * Contract (derived state):
 *   L_i = set of distinct active LandmarkIds in KeyFDATA[i].Matched_landmark_ids
 *   For i != j:  Vgraph(i,j) = |L_i ∩ L_j|
 *   Diagonal: 0 (compatible with existing consumers using strength > 0 / > 2 / > 10)
 *
 * Authoritative source: persistent KF observations (Matched_landmark_ids + active
 * AnchorsDATA via id_to_index). Deleted / unresolvable / duplicate lids do not
 * contribute beyond the distinct-set definition.
 *
 * Vgraph MUST NOT accumulate historical co-visibility unsupported by current
 * observation relationships. Production maintenance uses complete reconstruction
 * from observations (Strategy A).
 */
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <armadillo>
#include <opencv2/core.hpp>

#include "globalslam_types.hpp"
#include "gmap_landmark_identity.hpp"
#include "../../common/keyframe_types.hpp"

namespace gmap_visibility_graph_exact
{

/** Selected diagonal convention: zero. */
inline constexpr double kDiagonalValue = 0.0;

struct RebuildDiagnostics
{
  int n_kf{0};
  int n_landmarks{0};
  int n_obs_entries{0};
  int n_distinct_obs{0};
  int n_unresolvable_obs{0};
  int n_duplicate_obs_entries{0};
  int n_nonzero_edges{0};  // counted once per unordered pair
  double duration_ms{0.0};
  bool ok{true};
  std::string reason;
};

struct CompareDiagnostics
{
  bool ok{true};
  bool dim_mismatch{false};
  int asymmetric_pairs{0};
  int missing_edges{0};
  int stale_edges{0};
  int weight_mismatches{0};
  double max_abs_error{0.0};
  int affected_pairs{0};
  std::string reason;
};

struct ObservationIntegrity
{
  int n_kf{0};
  int n_obs_entries{0};
  int n_duplicate_same_kf_lid{0};
  int n_unresolvable{0};
  int n_uv_length_mismatch{0};
  bool ok{true};
  std::string reason;
};

inline ObservationIntegrity audit_observation_integrity(const GLOBAL_MAP & map)
{
  ObservationIntegrity out;
  out.n_kf = static_cast<int>(map.KeyFDATA.size());
  for (std::size_t i = 0; i < map.KeyFDATA.size(); ++i) {
    const auto & kf = map.KeyFDATA[i];
    if (kf.Matched_landmark_ids.size() != kf.UV_Matched_points.size()) {
      out.n_uv_length_mismatch++;
      out.ok = false;
      out.reason = "uv_length_mismatch";
    }
    std::unordered_set<LandmarkId, LandmarkIdHash> seen;
    for (const LandmarkId lid : kf.Matched_landmark_ids) {
      out.n_obs_entries++;
      if (!seen.insert(lid).second) {
        out.n_duplicate_same_kf_lid++;
      }
      std::size_t idx = 0;
      if (!gmap_landmark_id::resolve_landmark_index(map, lid, idx)) {
        out.n_unresolvable++;
      }
    }
  }
  if (out.n_duplicate_same_kf_lid > 0 && out.reason.empty()) {
    out.ok = false;
    out.reason = "duplicate_same_kf_landmark";
  }
  if (out.n_unresolvable > 0 && out.reason.empty()) {
    out.ok = false;
    out.reason = "unresolvable_landmark_id";
  }
  return out;
}

/**
 * Independent reference: set-intersection (or B B^T) with zero diagonal.
 * Does not call production rebuild.
 */
inline arma::mat build_reference_vgraph(const GLOBAL_MAP & map)
{
  const int n = static_cast<int>(map.KeyFDATA.size());
  arma::mat V(n, n, arma::fill::zeros);
  if (n == 0) {
    return V;
  }

  std::vector<std::unordered_set<LandmarkId, LandmarkIdHash>> sets(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    for (const LandmarkId lid : map.KeyFDATA[static_cast<std::size_t>(i)].Matched_landmark_ids) {
      std::size_t idx = 0;
      if (!gmap_landmark_id::resolve_landmark_index(map, lid, idx)) {
        continue;  // integrity issue reported separately; exclude from weight
      }
      sets[static_cast<std::size_t>(i)].insert(lid);
    }
  }

  for (int i = 0; i < n; ++i) {
    V(i, i) = kDiagonalValue;
    for (int j = i + 1; j < n; ++j) {
      int shared = 0;
      const auto & a = sets[static_cast<std::size_t>(i)];
      const auto & b = sets[static_cast<std::size_t>(j)];
      if (a.size() < b.size()) {
        for (const LandmarkId lid : a) {
          if (b.count(lid) != 0) {
            ++shared;
          }
        }
      } else {
        for (const LandmarkId lid : b) {
          if (a.count(lid) != 0) {
            ++shared;
          }
        }
      }
      V(i, j) = static_cast<double>(shared);
      V(j, i) = static_cast<double>(shared);
    }
  }
  return V;
}

/**
 * Efficient production reconstruction via landmark → observer sets.
 * Must match build_reference_vgraph exactly for valid observations.
 */
inline RebuildDiagnostics rebuild_exact_visibility_graph(GLOBAL_MAP & map)
{
  RebuildDiagnostics d;
  const auto t0 = std::chrono::steady_clock::now();
  const int n = static_cast<int>(map.KeyFDATA.size());
  d.n_kf = n;
  d.n_landmarks = static_cast<int>(map.AnchorsDATA.size());

  if (n == 0) {
    map.Vgraph.clear();
    d.duration_ms = 0.0;
    return d;
  }

  std::unordered_map<LandmarkId, std::vector<int>, LandmarkIdHash> observers;
  observers.reserve(static_cast<std::size_t>(map.AnchorsDATA.size()) + 8);

  for (int i = 0; i < n; ++i) {
    const auto & kf = map.KeyFDATA[static_cast<std::size_t>(i)];
    std::unordered_set<LandmarkId, LandmarkIdHash> seen;
    seen.reserve(kf.Matched_landmark_ids.size() + 1);
    for (const LandmarkId lid : kf.Matched_landmark_ids) {
      d.n_obs_entries++;
      if (!seen.insert(lid).second) {
        d.n_duplicate_obs_entries++;
        continue;
      }
      std::size_t idx = 0;
      if (!gmap_landmark_id::resolve_landmark_index(map, lid, idx)) {
        d.n_unresolvable_obs++;
        continue;
      }
      d.n_distinct_obs++;
      observers[lid].push_back(i);
    }
  }

  arma::mat V(n, n, arma::fill::zeros);
  for (int i = 0; i < n; ++i) {
    V(i, i) = kDiagonalValue;
  }

  for (auto & kv : observers) {
    auto & obs = kv.second;
    std::sort(obs.begin(), obs.end());
    obs.erase(std::unique(obs.begin(), obs.end()), obs.end());
    const int m = static_cast<int>(obs.size());
    for (int a = 0; a < m; ++a) {
      for (int b = a + 1; b < m; ++b) {
        const int i = obs[static_cast<std::size_t>(a)];
        const int j = obs[static_cast<std::size_t>(b)];
        V(i, j) += 1.0;
        V(j, i) += 1.0;
      }
    }
  }

  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (V(i, j) > 0.0) {
        d.n_nonzero_edges++;
      }
    }
  }

  map.Vgraph = std::move(V);
  const auto t1 = std::chrono::steady_clock::now();
  d.duration_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();
  return d;
}

inline CompareDiagnostics compare_to_reference(
  const arma::mat & produced, const arma::mat & reference, double tol = 1e-9)
{
  CompareDiagnostics c;
  if (produced.n_rows != reference.n_rows || produced.n_cols != reference.n_cols) {
    c.ok = false;
    c.dim_mismatch = true;
    c.reason = "dimension_mismatch";
    return c;
  }
  const int n = static_cast<int>(produced.n_rows);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const double a = produced(i, j);
      const double b = reference(i, j);
      if (i != j) {
        const double sym = std::abs(produced(i, j) - produced(j, i));
        if (sym > tol) {
          c.asymmetric_pairs++;
        }
      }
      const double err = std::abs(a - b);
      if (err > tol) {
        c.weight_mismatches++;
        c.affected_pairs++;
        c.max_abs_error = std::max(c.max_abs_error, err);
        if (a == 0.0 && b > 0.0) {
          c.missing_edges++;
        } else if (a > 0.0 && b == 0.0) {
          c.stale_edges++;
        }
      }
    }
  }
  // asymmetric counted twice per unordered pair in loop; normalize later if needed
  if (c.asymmetric_pairs > 0 || c.weight_mismatches > 0) {
    c.ok = false;
    if (c.reason.empty()) {
      c.reason = "weight_or_symmetry_mismatch";
    }
  }
  return c;
}

inline CompareDiagnostics validate_map_against_reference(const GLOBAL_MAP & map)
{
  const arma::mat ref = build_reference_vgraph(map);
  if (map.KeyFDATA.empty()) {
    CompareDiagnostics c;
    c.ok = map.Vgraph.empty() || (map.Vgraph.n_rows == 0 && map.Vgraph.n_cols == 0);
    if (!c.ok) {
      c.dim_mismatch = true;
      c.reason = "empty_map_nonempty_vgraph";
    }
    return c;
  }
  return compare_to_reference(map.Vgraph, ref);
}

enum class InsertObservationResult
{
  Inserted = 0,
  AlreadyPresent = 1,
  ConflictUv = 2,
  Invalid = 3,
};

inline bool kf_observes_landmark(
  const KEYFRAME & kf, LandmarkId lid, int * first_index = nullptr)
{
  for (std::size_t i = 0; i < kf.Matched_landmark_ids.size(); ++i) {
    if (kf.Matched_landmark_ids[i] == lid) {
      if (first_index != nullptr) {
        *first_index = static_cast<int>(i);
      }
      return true;
    }
  }
  return false;
}

/**
 * Idempotent monocular observation insert: at most one (KF, LandmarkId).
 * Same lid + similar UV → AlreadyPresent (no counter/weight side effects).
 * Same lid + conflicting UV → ConflictUv; existing observation preserved.
 * n_kf_matched is treated as distinct-KF count: incremented only on Inserted.
 */
inline InsertObservationResult try_insert_kf_observation(
  GLOBAL_MAP & map,
  int idx_kf,
  LandmarkId lid,
  const cv::Point2d & uv,
  bool increment_n_kf_matched,
  double uv_conflict_tol_px = 1.0)
{
  if (idx_kf < 0 || static_cast<std::size_t>(idx_kf) >= map.KeyFDATA.size()) {
    return InsertObservationResult::Invalid;
  }
  std::size_t lm_idx = 0;
  if (!gmap_landmark_id::resolve_landmark_index(map, lid, lm_idx)) {
    return InsertObservationResult::Invalid;
  }
  auto & kf = map.KeyFDATA[static_cast<std::size_t>(idx_kf)];
  if (kf.Matched_landmark_ids.size() != kf.UV_Matched_points.size()) {
    return InsertObservationResult::Invalid;
  }
  int existing = -1;
  if (kf_observes_landmark(kf, lid, &existing)) {
    const cv::Point2d & uv0 =
      kf.UV_Matched_points[static_cast<std::size_t>(existing)];
    const double d =
      std::hypot(uv0.x - uv.x, uv0.y - uv.y);
    if (d <= uv_conflict_tol_px) {
      return InsertObservationResult::AlreadyPresent;
    }
    return InsertObservationResult::ConflictUv;
  }
  kf.Matched_landmark_ids.push_back(lid);
  kf.UV_Matched_points.push_back(uv);
  if (increment_n_kf_matched) {
    map.AnchorsDATA[lm_idx].n_kf_matched++;
  }
  return InsertObservationResult::Inserted;
}

/** Conceptual fusion readiness: rebuild after observation-state mutation. */
inline void after_observation_mutation(GLOBAL_MAP & map)
{
  (void)rebuild_exact_visibility_graph(map);
}

}  // namespace gmap_visibility_graph_exact
