/**
 * GSLAM-BA-D5 audit-only trajectory / keyframe stream dump.
 * Enabled when GSLAM_BA_D5_DIR is set. Default: no-op.
 */
#ifndef GMAP_BA_D5_LOG_HPP
#define GMAP_BA_D5_LOG_HPP

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "globalslam_types.hpp"

namespace gmap_ba_d5
{

inline const char * audit_dir()
{
  const char * d = std::getenv("GSLAM_BA_D5_DIR");
  return (d && d[0] != '\0') ? d : nullptr;
}

inline bool enabled() { return audit_dir() != nullptr; }

inline std::mutex & io_mutex()
{
  static std::mutex m;
  return m;
}

inline void append_line(const char * filename, const std::string & line)
{
  const char * dir = audit_dir();
  if (!dir) {
    return;
  }
  std::ostringstream path;
  path << dir << '/' << filename;
  std::lock_guard<std::mutex> lock(io_mutex());
  std::ofstream out(path.str(), std::ios::app);
  if (out) {
    out << line << '\n';
  }
}

inline void dump_kf_pose(
  const char * event,
  int idx_kf,
  const KEYFRAME & kf,
  int ba_id = -1)
{
  if (!enabled()) {
    return;
  }
  std::ostringstream out;
  out << std::setprecision(17);
  // Body origin consistent with G4 viz (after BA, KeyFDATA.r_N may be stale).
  const arma::mat::fixed<3, 3> Rn2r_der = kf.Rr2c.t() * kf.Rn2c;
  const arma::vec::fixed<3> r_N_der = kf.t_c2n - Rn2r_der.t() * kf.t_c2r;

  out << "{\"event\":\"" << event << "\""
      << ",\"idx_kf\":" << idx_kf
      << ",\"keyframe_id\":" << kf.keyframe_id
      << ",\"source_time_ms\":" << kf.source_time_ms
      << ",\"ba_id\":" << ba_id
      << ",\"t_c2n\":[" << kf.t_c2n[0] << ',' << kf.t_c2n[1] << ',' << kf.t_c2n[2] << ']'
      << ",\"Rn2c\":[";
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (r || c) {
        out << ',';
      }
      out << kf.Rn2c.at(r, c);
    }
  }
  out << "],\"t_c2r\":[" << kf.t_c2r[0] << ',' << kf.t_c2r[1] << ',' << kf.t_c2r[2] << ']'
      << ",\"Rr2c\":[";
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (r || c) {
        out << ',';
      }
      out << kf.Rr2c.at(r, c);
    }
  }
  out << "],\"r_N\":[" << kf.r_N[0] << ',' << kf.r_N[1] << ',' << kf.r_N[2] << ']'
      << ",\"r_N_der\":[" << r_N_der[0] << ',' << r_N_der[1] << ',' << r_N_der[2] << "]}";
  append_line("kf_pose_stream.jsonl", out.str());
}

inline void dump_ba_commit(
  int ba_id,
  bool accepted,
  const GLOBAL_MAP & Gmap,
  const std::vector<int> & idx_kf_to_optimize)
{
  if (!enabled()) {
    return;
  }
  for (int idx_kf : idx_kf_to_optimize) {
    dump_kf_pose(accepted ? "ba_commit" : "ba_reject_keep", idx_kf, Gmap.KeyFDATA[idx_kf], ba_id);
  }
}

}  // namespace gmap_ba_d5

#endif
