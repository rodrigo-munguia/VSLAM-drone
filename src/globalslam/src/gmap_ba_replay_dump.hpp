/**
 * GSLAM-INIT-D3 audit-only full BA window dump for offline replay.
 *
 * Enabled only when GSLAM_BA_REPLAY_DIR is set. Default: no-op.
 * Does not alter BA, gates, or map commit.
 */
#ifndef GMAP_BA_REPLAY_DUMP_HPP
#define GMAP_BA_REPLAY_DUMP_HPP

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gtsam/geometry/Cal3_S2.h>

#include "globalslam_types.hpp"
#include "gmap_landmark_identity.hpp"
#include "../../common/Vision/vision.hpp"

namespace gmap_ba_replay
{

inline const char * audit_dir()
{
  const char * d = std::getenv("GSLAM_BA_REPLAY_DIR");
  return (d && d[0] != '\0') ? d : nullptr;
}

inline bool enabled() { return audit_dir() != nullptr; }

inline std::mutex & io_mutex()
{
  static std::mutex m;
  return m;
}

inline void dump_window(
  int ba_id,
  const GLOBAL_MAP & Gmap,
  const std::vector<int> & idx_kf_to_optimize,
  const std::vector<int64> & idx_points,
  const CAM & cam,
  double sigma_uv,
  double sigma_kf_att,
  double sigma_kf_xy,
  double sigma_kf_z,
  double sigma_pt,
  double max_delta_kf_pos)
{
  const char * dir = audit_dir();
  if (!dir) {
    return;
  }

  std::ostringstream out;
  out << std::setprecision(17);
  out << "{\"ba_id\":" << ba_id
      << ",\"K\":[" << cam.fc[0] << ',' << cam.fc[1] << ',' << cam.alpha_c << ','
      << cam.cc[0] << ',' << cam.cc[1] << ']'
      << ",\"sigma_uv\":" << sigma_uv
      << ",\"sigma_kf_att\":" << sigma_kf_att
      << ",\"sigma_kf_xy\":" << sigma_kf_xy
      << ",\"sigma_kf_z\":" << sigma_kf_z
      << ",\"sigma_pt\":" << sigma_pt
      << ",\"max_delta_kf_pos\":" << max_delta_kf_pos
      << ",\"kf\":[";

  for (size_t k = 0; k < idx_kf_to_optimize.size(); ++k) {
    if (k) {
      out << ',';
    }
    const int idx_kf = idx_kf_to_optimize[k];
    const auto & kf = Gmap.KeyFDATA[idx_kf];
    // Store camera-from-nav Rn2c and t_c2n (same as production Pose3 build uses Rc2n=Rn2c^T).
    out << "{\"idx\":" << idx_kf
        << ",\"keyframe_id\":" << kf.keyframe_id
        << ",\"Rn2c\":[";
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        if (r || c) {
          out << ',';
        }
        out << kf.Rn2c.at(r, c);
      }
    }
    out << "],\"t_c2n\":[" << kf.t_c2n[0] << ',' << kf.t_c2n[1] << ',' << kf.t_c2n[2] << "]}";
  }

  out << "],\"landmarks\":[";
  for (size_t i = 0; i < idx_points.size(); ++i) {
    if (i) {
      out << ',';
    }
    const int64_t idx_pt = idx_points[i];
    const auto & A = Gmap.AnchorsDATA[idx_pt];
    out << "{\"idx_pt\":" << idx_pt
        << ",\"audit_id\":" << A.audit_id
        << ",\"xyz\":[" << A.AnchorState[0] << ',' << A.AnchorState[1] << ','
        << A.AnchorState[2] << "]}";
  }

  out << "],\"obs\":[";
  bool first = true;
  for (int idx_kf : idx_kf_to_optimize) {
    const auto & kf = Gmap.KeyFDATA[idx_kf];
    for (size_t j = 0; j < kf.Matched_landmark_ids.size(); ++j) {
      std::size_t idx_pt = 0;
      if (!gmap_landmark_id::resolve_landmark_index(
            Gmap, kf.Matched_landmark_ids[j], idx_pt)) {
        continue;
      }
      // Match production: factors use undistorted UV.
      const cv::Point2f ud = Undistort_a_point(kf.UV_Matched_points[j], cam, 1);
      if (!first) {
        out << ',';
      }
      first = false;
      out << "{\"idx_kf\":" << idx_kf
          << ",\"idx_pt\":" << idx_pt
          << ",\"landmark_id\":" << kf.Matched_landmark_ids[j].value
          << ",\"audit_id\":" << Gmap.AnchorsDATA[idx_pt].audit_id
          << ",\"uv_u\":[" << ud.x << ',' << ud.y << "]}";
    }
  }
  out << "]}\n";

  std::ostringstream path;
  path << dir << "/ba_replay_windows.jsonl";
  std::lock_guard<std::mutex> lock(io_mutex());
  std::ofstream f(path.str(), std::ios::app);
  if (f) {
    f << out.str();
  }
}

}  // namespace gmap_ba_replay

#endif
