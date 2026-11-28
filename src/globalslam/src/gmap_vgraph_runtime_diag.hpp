/**
 * GSLAM-LC-POLICY-A1.2 — opt-in exact Vgraph runtime diagnostics.
 * Enabled only when GSLAM_VGRAPH_RUNTIME_DIR is set. Observational only.
 */
#ifndef GMAP_VGRAPH_RUNTIME_DIAG_HPP
#define GMAP_VGRAPH_RUNTIME_DIAG_HPP

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <armadillo>

#include "globalslam_types.hpp"
#include "gmap_landmark_identity.hpp"

namespace gmap_vgraph_runtime_diag
{

inline const char * runtime_dir()
{
  return std::getenv("GSLAM_VGRAPH_RUNTIME_DIR");
}

inline bool enabled()
{
  const char * d = runtime_dir();
  return d != nullptr && d[0] != '\0';
}

inline std::mutex & io_mutex()
{
  static std::mutex m;
  return m;
}

inline uint64_t now_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch())
      .count());
}

inline void ensure_header(const std::string & path, const char * header)
{
  std::ifstream in(path);
  if (in.good()) {
    return;
  }
  std::ofstream out(path, std::ios::app);
  if (out) {
    out << header << '\n';
  }
}

/** Thread-local rebuild attribution for aggregate GMAP-update accounting. */
inline std::string & tls_reason()
{
  thread_local std::string reason{"unspecified"};
  return reason;
}

inline int & tls_update_rebuild_count()
{
  thread_local int c{0};
  return c;
}

inline double & tls_update_rebuild_ms()
{
  thread_local double ms{0.0};
  return ms;
}

inline void begin_gmap_update()
{
  if (!enabled()) {
    return;
  }
  tls_update_rebuild_count() = 0;
  tls_update_rebuild_ms() = 0.0;
}

inline void set_reason(const char * reason)
{
  if (!enabled()) {
    return;
  }
  tls_reason() = reason ? reason : "unspecified";
}

inline void log_rebuild(
  const char * reason,
  int n_kf,
  int n_landmarks,
  int n_obs_entries,
  int n_distinct_obs,
  int n_duplicate_obs_entries,
  int n_unresolvable_obs,
  int n_nonzero_edges,
  double duration_ms)
{
  if (!enabled()) {
    return;
  }
  tls_update_rebuild_count() += 1;
  tls_update_rebuild_ms() += duration_ms;
  const std::string path = std::string(runtime_dir()) + "/vgraph_rebuild_timing.csv";
  std::lock_guard<std::mutex> lk(io_mutex());
  ensure_header(
    path,
    "t_ns,reason,n_kf,n_landmarks,n_obs_entries,n_distinct_obs,"
    "n_duplicate_obs_entries,n_unresolvable_obs,n_nonzero_edges,duration_ms");
  std::ofstream out(path, std::ios::app);
  if (!out) {
    return;
  }
  out << now_ns() << ',' << (reason ? reason : tls_reason().c_str()) << ','
      << n_kf << ',' << n_landmarks << ',' << n_obs_entries << ','
      << n_distinct_obs << ',' << n_duplicate_obs_entries << ','
      << n_unresolvable_obs << ',' << n_nonzero_edges << ','
      << duration_ms << '\n';
}

template <typename RebuildDiagnosticsT>
inline void log_rebuild_diag(const char * reason, const RebuildDiagnosticsT & d)
{
  log_rebuild(
    reason, d.n_kf, d.n_landmarks, d.n_obs_entries, d.n_distinct_obs,
    d.n_duplicate_obs_entries, d.n_unresolvable_obs, d.n_nonzero_edges,
    d.duration_ms);
}

inline void log_gmap_update_overhead(
  double gmap_update_ms, double ba_ms, int n_kf_admitted, int n_kf_total,
  int n_lm, int n_obs)
{
  if (!enabled()) {
    return;
  }
  const std::string path =
    std::string(runtime_dir()) + "/gmap_update_rebuild_overhead.csv";
  std::lock_guard<std::mutex> lk(io_mutex());
  ensure_header(
    path,
    "t_ns,n_rebuilds,rebuild_ms_total,gmap_update_ms,ba_ms,"
    "rebuild_frac,n_kf_admitted,n_kf_total,n_lm,n_obs");
  std::ofstream out(path, std::ios::app);
  if (!out) {
    return;
  }
  const int n_reb = tls_update_rebuild_count();
  const double reb_ms = tls_update_rebuild_ms();
  const double frac =
    gmap_update_ms > 0.0 ? (reb_ms / gmap_update_ms) : 0.0;
  out << now_ns() << ',' << n_reb << ',' << reb_ms << ',' << gmap_update_ms
      << ',' << ba_ms << ',' << frac << ',' << n_kf_admitted << ','
      << n_kf_total << ',' << n_lm << ',' << n_obs << '\n';
}

struct InsertCounters
{
  uint64_t inserted{0};
  uint64_t already_present{0};
  uint64_t conflict_uv{0};
  uint64_t invalid{0};
};

inline InsertCounters & insert_counters()
{
  static InsertCounters c;
  return c;
}

/** r: 0=Inserted 1=AlreadyPresent 2=ConflictUv 3=Invalid */
inline void note_insert_result(int r)
{
  if (!enabled()) {
    return;
  }
  auto & c = insert_counters();
  switch (r) {
    case 0: ++c.inserted; break;
    case 1: ++c.already_present; break;
    case 2: ++c.conflict_uv; break;
    default: ++c.invalid; break;
  }
}

template <typename EnumT>
inline void note_insert_result_enum(EnumT r)
{
  note_insert_result(static_cast<int>(r));
}

inline void flush_insert_counters(const char * site)
{
  if (!enabled()) {
    return;
  }
  auto & c = insert_counters();
  const std::string path =
    std::string(runtime_dir()) + "/observation_insert_results.csv";
  std::lock_guard<std::mutex> lk(io_mutex());
  ensure_header(
    path,
    "t_ns,site,inserted,already_present,conflict_uv,invalid");
  std::ofstream out(path, std::ios::app);
  if (!out) {
    return;
  }
  out << now_ns() << ',' << (site ? site : "") << ',' << c.inserted << ','
      << c.already_present << ',' << c.conflict_uv << ',' << c.invalid << '\n';
}

inline void log_ba_window(
  int tip_kf_idx,
  const std::vector<int> & idx_optimize,
  const std::vector<int> & idx_visibility_added,
  int n_factors,
  double ba_ms,
  bool accepted)
{
  if (!enabled()) {
    return;
  }
  std::ostringstream opt;
  for (std::size_t i = 0; i < idx_optimize.size(); ++i) {
    if (i) {
      opt << ';';
    }
    opt << idx_optimize[i];
  }
  std::ostringstream vis;
  for (std::size_t i = 0; i < idx_visibility_added.size(); ++i) {
    if (i) {
      vis << ';';
    }
    vis << idx_visibility_added[i];
  }
  const std::string path =
    std::string(runtime_dir()) + "/ba_window_composition.csv";
  std::lock_guard<std::mutex> lk(io_mutex());
  ensure_header(
    path,
    "t_ns,tip_kf_idx,n_optimize,optimize_idxs,visibility_added_idxs,"
    "n_factors,ba_ms,accepted");
  std::ofstream out(path, std::ios::app);
  if (!out) {
    return;
  }
  out << now_ns() << ',' << tip_kf_idx << ',' << idx_optimize.size() << ','
      << opt.str() << ',' << vis.str() << ',' << n_factors << ',' << ba_ms
      << ',' << (accepted ? 1 : 0) << '\n';
}

/**
 * Visibility exclusion characterization for one closing query.
 * Counts historical KFs in [0, ba_limit) by exact V weight bins.
 */
inline void log_visibility_exclusions(
  uint64_t closing_kf_id,
  int closing_idx,
  int ba_limit,
  const GLOBAL_MAP & map)
{
  if (!enabled()) {
    return;
  }
  if (closing_idx < 0 || map.Vgraph.empty() ||
    closing_idx >= static_cast<int>(map.Vgraph.n_rows))
  {
    return;
  }
  const int limit =
    std::min(ba_limit, static_cast<int>(map.Vgraph.n_cols));
  int n_v0 = 0;
  int n_v1 = 0;
  int n_v2_5 = 0;
  int n_v6_20 = 0;
  int n_v21p = 0;
  double max_w = 0.0;
  for (int i = 0; i < limit; ++i) {
    if (i == closing_idx) {
      continue;
    }
    const double w = map.Vgraph.at(closing_idx, i);
    if (w > max_w) {
      max_w = w;
    }
    if (w <= 0.0) {
      ++n_v0;
    } else if (w < 1.5) {
      ++n_v1;
    } else if (w < 5.5) {
      ++n_v2_5;
    } else if (w < 20.5) {
      ++n_v6_20;
    } else {
      ++n_v21p;
    }
  }
  const std::string path =
    std::string(runtime_dir()) + "/visibility_candidate_exclusions.csv";
  std::lock_guard<std::mutex> lk(io_mutex());
  ensure_header(
    path,
    "t_ns,closing_kf_id,closing_idx,ba_limit,n_eligible_v0,"
    "n_excluded_v1,n_excluded_v2_5,n_excluded_v6_20,n_excluded_v21p,max_weight");
  std::ofstream out(path, std::ios::app);
  if (!out) {
    return;
  }
  out << now_ns() << ',' << closing_kf_id << ',' << closing_idx << ','
      << ba_limit << ',' << n_v0 << ',' << n_v1 << ',' << n_v2_5 << ','
      << n_v6_20 << ',' << n_v21p << ',' << max_w << '\n';
}

inline int & snap_counter()
{
  static int c{0};
  return c;
}

inline bool should_snapshot(const char * boundary, int n_kf)
{
  if (!enabled()) {
    return false;
  }
  static int last_periodic = -1;
  const std::string b = boundary ? boundary : "";
  if (b == "after_delete" || b == "after_ba" || b == "pre_loop_commit" ||
    b == "post_loop_refine" || b == "authoritative_commit" ||
    b == "mission_end" || b == "initial_mapping")
  {
    return true;
  }
  // Ordinary mapping: every 15 KF or first few
  if (b == "ordinary_update") {
    if (n_kf <= 5 || n_kf == 20 || n_kf == 50 || n_kf == 80 ||
      n_kf == 100 || (n_kf % 15 == 0 && n_kf != last_periodic))
    {
      last_periodic = n_kf;
      return true;
    }
  }
  return false;
}

/**
 * Write owned diagnostic snapshot for offline independent reference check.
 * Does not run expensive reference reconstruction online.
 */
inline void write_snapshot(const char * boundary, const GLOBAL_MAP & map)
{
  if (!enabled()) {
    return;
  }
  const int n_kf = static_cast<int>(map.KeyFDATA.size());
  if (!should_snapshot(boundary, n_kf)) {
    return;
  }
  const int id = ++snap_counter();
  std::ostringstream dir;
  dir << runtime_dir() << "/snapshots";
  const std::string snap_dir = dir.str();
  // Directory created by runner; best-effort mkdir via ofstream parent.
  std::ostringstream base;
  base << snap_dir << "/snap_" << id << "_" << (boundary ? boundary : "x");
  const std::string prefix = base.str();

  std::lock_guard<std::mutex> lk(io_mutex());

  {
    std::ofstream meta(prefix + "_meta.csv");
    if (!meta) {
      return;
    }
    meta << "boundary,n_kf,n_lm,n_obs,v_rows,v_cols,t_ns\n";
    std::size_t n_obs = 0;
    for (const auto & kf : map.KeyFDATA) {
      n_obs += kf.Matched_landmark_ids.size();
    }
    meta << (boundary ? boundary : "") << ',' << n_kf << ','
         << map.AnchorsDATA.size() << ',' << n_obs << ','
         << map.Vgraph.n_rows << ',' << map.Vgraph.n_cols << ','
         << now_ns() << '\n';
  }

  {
    std::ofstream kf(prefix + "_kf_ids.csv");
    kf << "kf_idx,keyframe_id\n";
    for (int i = 0; i < n_kf; ++i) {
      kf << i << ',' << map.KeyFDATA[static_cast<std::size_t>(i)].keyframe_id
         << '\n';
    }
  }

  {
    std::ofstream obs(prefix + "_observations.csv");
    obs << "kf_idx,landmark_id,u,v\n";
    for (int i = 0; i < n_kf; ++i) {
      const auto & kfr = map.KeyFDATA[static_cast<std::size_t>(i)];
      for (std::size_t j = 0; j < kfr.Matched_landmark_ids.size(); ++j) {
        const auto lid = kfr.Matched_landmark_ids[j];
        double u = 0.0;
        double v = 0.0;
        if (j < kfr.UV_Matched_points.size()) {
          u = kfr.UV_Matched_points[j].x;
          v = kfr.UV_Matched_points[j].y;
        }
        obs << i << ',' << lid.value << ',' << u << ',' << v << '\n';
      }
    }
  }

  {
    std::ofstream lm(prefix + "_landmarks.csv");
    lm << "storage_idx,landmark_id,n_kf_matched,init_kf\n";
    for (std::size_t i = 0; i < map.AnchorsDATA.size(); ++i) {
      const auto & a = map.AnchorsDATA[i];
      lm << i << ',' << a.id.value << ',' << a.n_kf_matched << ','
         << a.init_KF << '\n';
    }
  }

  {
    std::ofstream vg(prefix + "_vgraph_edges.csv");
    vg << "i,j,weight\n";
    const int nr = static_cast<int>(map.Vgraph.n_rows);
    const int nc = static_cast<int>(map.Vgraph.n_cols);
    for (int i = 0; i < nr; ++i) {
      for (int j = i + 1; j < nc; ++j) {
        const double w = map.Vgraph(i, j);
        if (w != 0.0) {
          vg << i << ',' << j << ',' << w << '\n';
        }
      }
      // Diagonal check
      if (i < nc) {
        const double d = map.Vgraph(i, i);
        if (d != 0.0) {
          vg << i << ',' << i << ',' << d << '\n';
        }
      }
    }
  }

  {
    const std::string idx_path =
      std::string(runtime_dir()) + "/snapshot_index.csv";
    ensure_header(
      idx_path, "snap_id,boundary,n_kf,n_lm,prefix");
    std::ofstream idx(idx_path, std::ios::app);
    if (idx) {
      idx << id << ',' << (boundary ? boundary : "") << ',' << n_kf << ','
          << map.AnchorsDATA.size() << ',' << prefix << '\n';
    }
  }
}

inline void log_loop_txn_overhead(
  uint64_t proposal_id,
  int n_rebuilds,
  double rebuild_ms,
  double match_ms,
  double ba_ms,
  double total_ms)
{
  if (!enabled()) {
    return;
  }
  const std::string path =
    std::string(runtime_dir()) + "/loop_transaction_rebuild_overhead.csv";
  std::lock_guard<std::mutex> lk(io_mutex());
  ensure_header(
    path,
    "t_ns,proposal_id,n_rebuilds,rebuild_ms,match_ms,ba_ms,total_ms");
  std::ofstream out(path, std::ios::app);
  if (!out) {
    return;
  }
  out << now_ns() << ',' << proposal_id << ',' << n_rebuilds << ','
      << rebuild_ms << ',' << match_ms << ',' << ba_ms << ',' << total_ms
      << '\n';
}

/**
 * Production rebuild wrapper with optional timing attribution.
 * Include gmap_visibility_graph_exact.hpp before calling.
 */
template <typename MapT, typename RebuildFn>
inline auto rebuild_with_log(MapT & map, const char * reason, RebuildFn && fn)
  -> decltype(fn(map))
{
  set_reason(reason);
  auto d = fn(map);
  log_rebuild_diag(reason, d);
  return d;
}

}  // namespace gmap_vgraph_runtime_diag

#endif
