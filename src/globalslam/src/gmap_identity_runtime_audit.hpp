/**
 * GSLAM-MAP-ARCH-M6: opt-in identity / observation / snapshot audit.
 *
 * Enabled only when GSLAM_IDENTITY_INVARIANT_DIR is set to a writable directory.
 * Lightweight boundary sampling — not a per-landmark full scan on every birth.
 */
#ifndef GMAP_IDENTITY_RUNTIME_AUDIT_HPP
#define GMAP_IDENTITY_RUNTIME_AUDIT_HPP

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

#include "gmap_landmark_identity.hpp"

namespace gmap_identity_runtime_audit
{

inline const char * audit_dir()
{
  const char * d = std::getenv("GSLAM_IDENTITY_INVARIANT_DIR");
  return (d && d[0] != '\0') ? d : nullptr;
}

inline bool enabled() { return audit_dir() != nullptr; }

inline std::mutex & io_mutex()
{
  static std::mutex m;
  return m;
}

inline double wall_s()
{
  using clock = std::chrono::steady_clock;
  static const auto t0 = clock::now();
  return std::chrono::duration<double>(clock::now() - t0).count();
}

inline void append_line(const std::string & filename, const std::string & line)
{
  const char * dir = audit_dir();
  if (!dir) {
    return;
  }
  std::lock_guard<std::mutex> lock(io_mutex());
  std::ofstream f(std::string(dir) + "/" + filename, std::ios::app);
  if (f) {
    f << line << '\n';
  }
}

inline void ensure_headers()
{
  if (!enabled()) {
    return;
  }
  static bool once = false;
  if (once) {
    return;
  }
  once = true;
  std::lock_guard<std::mutex> lock(io_mutex());
  const char * dir = audit_dir();
  {
    std::ofstream f(std::string(dir) + "/invariant_samples.csv");
    f << "t_s,boundary,ok,n_kf,n_lm,n_lookup,next_id,reason\n";
  }
  {
    std::ofstream f(std::string(dir) + "/invariant_violations.csv");
    f << "t_s,boundary,ok,n_kf,n_lm,n_lookup,next_id,reason\n";
  }
  {
    std::ofstream f(std::string(dir) + "/snapshot_events.csv");
    f << "t_s,event,peer,n_kf,n_lm,next_id,n_obs,last_kf_id,id_min,id_max\n";
  }
}

/** Full structural check + CSV violation / OK sample at a named boundary. */
inline void check_boundary(const char * boundary, const GLOBAL_MAP & map)
{
  if (!enabled()) {
    return;
  }
  ensure_headers();
  const auto r = gmap_landmark_id::validate_landmark_identity_invariants(map);
  std::ostringstream oss;
  oss << wall_s() << ',' << boundary << ',' << (r.ok ? 1 : 0) << ','
      << map.KeyFDATA.size() << ',' << map.AnchorsDATA.size() << ','
      << map.id_to_index.size() << ',' << map.next_landmark_id << ','
      << (r.ok ? "ok" : r.reason);
  append_line("invariant_samples.csv", oss.str());
  if (!r.ok) {
    append_line("invariant_violations.csv", oss.str());
  }
}

/**
 * Snapshot / commit characterization (E1–E3).
 * event: gmap_get | gmap_set | cloop_set | cloop_get
 */
inline void log_snapshot_event(
  const char * event, const GLOBAL_MAP & map, const char * peer_role = "")
{
  if (!enabled()) {
    return;
  }
  ensure_headers();
  std::size_t n_obs = 0;
  for (const auto & kf : map.KeyFDATA) {
    n_obs += kf.Matched_landmark_ids.size();
  }
  std::uint64_t id_min = 0;
  std::uint64_t id_max = 0;
  if (!map.AnchorsDATA.empty()) {
    id_min = map.AnchorsDATA.front().id.value;
    id_max = map.AnchorsDATA.front().id.value;
    for (const auto & a : map.AnchorsDATA) {
      if (a.id.value < id_min) {
        id_min = a.id.value;
      }
      if (a.id.value > id_max) {
        id_max = a.id.value;
      }
    }
  }

  std::ostringstream oss;
  oss << wall_s() << ',' << event << ',' << (peer_role ? peer_role : "") << ','
      << map.KeyFDATA.size() << ',' << map.AnchorsDATA.size() << ','
      << map.next_landmark_id << ',' << n_obs << ','
      << (map.KeyFDATA.empty() ? 0 : map.KeyFDATA.back().keyframe_id) << ','
      << id_min << ',' << id_max;
  append_line("snapshot_events.csv", oss.str());
}

}  // namespace gmap_identity_runtime_audit

#endif
