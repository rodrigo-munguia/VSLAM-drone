/**
 * M7.7 opt-in runtime diagnostics (disabled unless GSLAM_M7_RUNTIME_DIR is set).
 * Observational only — no estimator / sync semantic changes.
 */
#ifndef GMAP_M7_RUNTIME_DIAG_HPP
#define GMAP_M7_RUNTIME_DIAG_HPP

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "globalslam_types.hpp"

namespace gmap_m7_runtime_diag
{

inline const char * runtime_dir()
{
  return std::getenv("GSLAM_M7_RUNTIME_DIR");
}

inline bool enabled()
{
  const char * d = runtime_dir();
  return d != nullptr && d[0] != '\0';
}

inline uint64_t now_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch())
      .count());
}

inline std::mutex & io_mutex()
{
  static std::mutex m;
  return m;
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

inline std::string kf_ids_csv(const GLOBAL_MAP & map)
{
  std::ostringstream os;
  for (std::size_t i = 0; i < map.KeyFDATA.size(); ++i) {
    if (i) {
      os << ';';
    }
    os << map.KeyFDATA[i].keyframe_id;
  }
  return os.str();
}

inline std::string lm_ids_csv(const GLOBAL_MAP & map)
{
  std::ostringstream os;
  for (std::size_t i = 0; i < map.AnchorsDATA.size(); ++i) {
    if (i) {
      os << ';';
    }
    os << map.AnchorsDATA[i].id.value;
  }
  return os.str();
}

inline void log_extract(
  const char * site,
  uint64_t wait_ns,
  uint64_t clone_ns,
  std::size_t n_kf,
  std::size_t n_lm,
  std::size_t n_obs,
  uint64_t auth_seq,
  uint64_t map_lifetime)
{
  if (!enabled()) {
    return;
  }
  const std::string path = std::string(runtime_dir()) + "/detection_extract.csv";
  std::lock_guard<std::mutex> lk(io_mutex());
  ensure_header(
    path,
    "t_ns,site,wait_ns,clone_ns,n_kf,n_lm,n_obs,auth_seq,map_lifetime");
  std::ofstream out(path, std::ios::app);
  if (!out) {
    return;
  }
  out << now_ns() << ',' << site << ',' << wait_ns << ',' << clone_ns << ','
      << n_kf << ',' << n_lm << ',' << n_obs << ',' << auth_seq << ','
      << map_lifetime << '\n';
}

inline void log_identity_snapshot(
  const char * stage,
  uint64_t proposal_id,
  uint64_t closing_kf,
  uint64_t map_lifetime,
  uint64_t correction_epoch,
  const GLOBAL_MAP & map)
{
  if (!enabled()) {
    return;
  }
  std::size_t n_obs = 0;
  for (const auto & kf : map.KeyFDATA) {
    n_obs += kf.Matched_landmark_ids.size();
  }
  const std::string path = std::string(runtime_dir()) + "/identity_snapshots.csv";
  std::lock_guard<std::mutex> lk(io_mutex());
  ensure_header(
    path,
    "t_ns,stage,proposal_id,closing_kf,map_lifetime,correction_epoch,"
    "n_kf,n_lm,n_obs,next_landmark_id,kf_ids,lm_ids");
  std::ofstream out(path, std::ios::app);
  if (!out) {
    return;
  }
  out << now_ns() << ',' << stage << ',' << proposal_id << ',' << closing_kf
      << ',' << map_lifetime << ',' << correction_epoch << ','
      << map.KeyFDATA.size() << ',' << map.AnchorsDATA.size() << ',' << n_obs
      << ',' << map.next_landmark_id << ',' << kf_ids_csv(map) << ','
      << lm_ids_csv(map) << '\n';
}

inline void log_event(
  const char * kind,
  uint64_t proposal_id,
  uint64_t a,
  uint64_t b,
  const std::string & detail)
{
  if (!enabled()) {
    return;
  }
  const std::string path = std::string(runtime_dir()) + "/runtime_events.csv";
  std::lock_guard<std::mutex> lk(io_mutex());
  ensure_header(path, "t_ns,kind,proposal_id,a,b,detail");
  std::ofstream out(path, std::ios::app);
  if (!out) {
    return;
  }
  out << now_ns() << ',' << kind << ',' << proposal_id << ',' << a << ',' << b
      << ",\"" << detail << "\"\n";
}

/** M7.7.3 detection-funnel counters (opt-in). */
inline void log_funnel(
  const char * stage,
  uint64_t keyframe_id,
  uint64_t a,
  uint64_t b,
  const std::string & detail)
{
  if (!enabled()) {
    return;
  }
  const std::string path = std::string(runtime_dir()) + "/detection_funnel.csv";
  std::lock_guard<std::mutex> lk(io_mutex());
  ensure_header(path, "t_ns,stage,keyframe_id,a,b,detail");
  std::ofstream out(path, std::ios::app);
  if (!out) {
    return;
  }
  out << now_ns() << ',' << stage << ',' << keyframe_id << ',' << a << ',' << b
      << ",\"" << detail << "\"\n";
}

}  // namespace gmap_m7_runtime_diag

#endif
