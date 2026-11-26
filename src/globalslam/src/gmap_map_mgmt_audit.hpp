/**
 * GSLAM-MAP-MGMT-A2 — observational map-management audit.
 *
 * Enabled when GSLAM_MAP_MGMT_AUDIT_DIR is set to a writable directory,
 * or when GSLAM_MAP_MGMT_AUDIT_ENABLE=1 (writes under /tmp/gslam_map_mgmt_audit).
 * Default (unset): all calls are no-ops; production decisions unchanged.
 *
 * Does not alter matching, initialization, culling, BA, or loop closure.
 */
#ifndef GMAP_MAP_MGMT_AUDIT_HPP
#define GMAP_MAP_MGMT_AUDIT_HPP

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <vector>

#include <armadillo>
#include <opencv2/opencv.hpp>

#include "globalslam_types.hpp"
#include "../../common/Vision/vision.hpp"

namespace gmap_map_mgmt_audit
{

inline const char * audit_dir()
{
  const char * d = std::getenv("GSLAM_MAP_MGMT_AUDIT_DIR");
  if (d && d[0] != '\0') {
    return d;
  }
  const char * en = std::getenv("GSLAM_MAP_MGMT_AUDIT_ENABLE");
  if (en && en[0] != '\0' && en[0] != '0') {
    return "/tmp/gslam_map_mgmt_audit";
  }
  return nullptr;
}

inline bool enabled() { return audit_dir() != nullptr; }

inline std::mutex & io_mutex()
{
  static std::mutex m;
  return m;
}

inline double wall_s_now()
{
  return std::chrono::duration<double>(
           std::chrono::system_clock::now().time_since_epoch())
    .count();
}

inline int64_t next_stable_id()
{
  static std::atomic<int64_t> seq{0};
  return ++seq;
}

inline void ensure_headers()
{
  static std::atomic<bool> done{false};
  if (done.load()) {
    return;
  }
  const char * dir = audit_dir();
  if (!dir) {
    return;
  }
  std::lock_guard<std::mutex> lock(io_mutex());
  if (done.load()) {
    return;
  }
  ::mkdir(dir, 0755);
  {
    std::ofstream f(std::string(dir) + "/per_keyframe.csv", std::ios::app);
    if (f.tellp() == 0) {
      f << "wall_s,kf_idx,keyframe_id,source_time_ms,est_x,est_y,est_z,"
           "n_features,n_desc,img_cols,img_rows,"
           "n_anchors_before,n_eligible_age,n_excluded_age,"
           "n_eligible_visible,n_matched_existing,n_hist_visible,"
           "n_hist_visible_matched_ge3,"
           "n_ssc_current,n_raw_matches,n_fmat_inliers,"
           "n_rejected_min_distance,n_triang_submitted,"
           "n_rejected_depth,n_rejected_birth_gate,n_births,"
           "n_deleted,n_anchors_after,n_obs_total,n_keyframes,"
           "grid_assoc_occ,grid_birth_occ,grid_combined_occ,"
           "n_birth_in_supported_cell,mean_nn_px_birth_to_assoc,"
           "ba_ran,ba_n_kf,ba_n_pts,ba_time_s,ba_accepted,"
           "match_time_s,init_time_s,update_time_s,"
           "horizon_kf,min_distance_px\n";
    }
  }
  {
    std::ofstream f(std::string(dir) + "/per_landmark_birth.csv", std::ios::app);
    if (f.tellp() == 0) {
      f << "wall_s,stable_id,anchor_idx,audit_id,init_kf_idx,keyframe_id,"
           "x,y,z,uv_u,uv_v,birth_rms_px,n_kf_matched\n";
    }
  }
  {
    std::ofstream f(std::string(dir) + "/per_landmark_delete.csv", std::ios::app);
    if (f.tellp() == 0) {
      f << "wall_s,stable_id,anchor_idx,audit_id,init_kf_idx,n_kf_matched,"
           "n_tries,delete_kf_idx\n";
    }
  }
  {
    std::ofstream f(std::string(dir) + "/ba_events.csv", std::ios::app);
    if (f.tellp() == 0) {
      f << "wall_s,kf_idx,n_kf,n_pts,n_obs,ba_time_s,accepted,n_anchors_total\n";
    }
  }
  {
    std::ofstream f(std::string(dir) + "/anchor_snapshot.jsonl", std::ios::app);
    (void)f;
  }
  done.store(true);
}

constexpr int kGridW = 8;
constexpr int kGridH = 6;

struct KfTick
{
  bool active{false};
  int kf_idx{-1};
  uint64_t keyframe_id{0};
  long int source_time_ms{0};
  double est_x{0}, est_y{0}, est_z{0};
  int n_features{0};
  int n_desc{0};
  int img_cols{0};
  int img_rows{0};

  int n_anchors_before{0};
  int n_eligible_age{0};
  int n_excluded_age{0};
  int n_eligible_visible{0};
  int n_matched_existing{0};
  int n_hist_visible{0};
  int n_hist_visible_matched_ge3{0};

  int n_ssc_current{0};
  int n_raw_matches{0};
  int n_fmat_inliers{0};
  int n_rejected_min_distance{0};
  int n_triang_submitted{0};
  int n_rejected_depth{0};
  int n_rejected_birth_gate{0};
  int n_births{0};
  int n_deleted{0};

  int ba_ran{0};
  int ba_n_kf{0};
  int ba_n_pts{0};
  double ba_time_s{0};
  int ba_accepted{0};
  int ba_n_obs{0};

  double match_time_s{0};
  double init_time_s{0};

  int horizon_kf{0};
  double min_distance_px{0};

  // Image-space UV samples (associated before init; births).
  std::vector<cv::Point2f> uv_assoc;
  std::vector<cv::Point2f> uv_birth;
};

inline KfTick & tick()
{
  static KfTick t;
  return t;
}

// Stable IDs keyed by current vector index; compacted on delete.
inline std::vector<int64_t> & stable_ids()
{
  static std::vector<int64_t> ids;
  return ids;
}

inline void begin_kf(
  int kf_idx,
  const KEYFRAME & kf,
  int n_anchors_before,
  int horizon_kf,
  double min_distance_px)
{
  if (!enabled()) {
    return;
  }
  ensure_headers();
  auto & t = tick();
  t = KfTick{};
  t.active = true;
  t.kf_idx = kf_idx;
  t.keyframe_id = kf.keyframe_id;
  t.source_time_ms = kf.source_time_ms;
  t.est_x = kf.t_c2n[0];
  t.est_y = kf.t_c2n[1];
  t.est_z = kf.t_c2n[2];
  t.n_anchors_before = n_anchors_before;
  t.horizon_kf = horizon_kf;
  t.min_distance_px = min_distance_px;
  if (!kf.frame.empty()) {
    t.img_cols = kf.frame.cols;
    t.img_rows = kf.frame.rows;
  }
}

inline void note_features(int n_kp, int n_desc)
{
  if (!tick().active) {
    return;
  }
  tick().n_features = n_kp;
  tick().n_desc = n_desc;
}

inline bool project_visible(
  const G_ANCHOR & A,
  const KEYFRAME & kf,
  const CAM & cam,
  int img_cols,
  int img_rows,
  int margin)
{
  arma::mat::fixed<3, 3> Rn2c = kf.Rn2c;
  arma::vec::fixed<3> t_c2n = kf.t_c2n;
  arma::vec::fixed<3> pc = Rn2c * (A.AnchorState - t_c2n);
  if (pc[2] <= 1e-6) {
    return false;
  }
  arma::mat::fixed<2, 3> duv_dPc;
  cv::Point2d uvd = Projection_model(pc, 0, false, cam, duv_dPc);
  return (uvd.x > -margin) && (uvd.x < img_cols + margin) &&
         (uvd.y > -margin) && (uvd.y < img_rows + margin);
}

/**
 * Read-only association-horizon / visibility census after production Match.
 * Does not call the matcher or mutate the map.
 */
inline void census_association(
  const GLOBAL_MAP & Gmap,
  int idx_KF,
  int idx_kf_min_ref,
  const CAM & cam,
  int margin,
  int n_matched_existing,
  const std::vector<cv::Point2f> & uv_matched)
{
  if (!tick().active) {
    return;
  }
  auto & t = tick();
  t.n_matched_existing = n_matched_existing;
  t.uv_assoc = uv_matched;
  const KEYFRAME & kf = Gmap.KeyFDATA[idx_KF];
  const int img_c = t.img_cols > 0 ? t.img_cols : 400;
  const int img_r = t.img_rows > 0 ? t.img_rows : 300;

  for (size_t i = 0; i < Gmap.AnchorsDATA.size(); ++i) {
    const auto & A = Gmap.AnchorsDATA[i];
    const bool eligible = A.init_KF > idx_kf_min_ref;
    if (eligible) {
      ++t.n_eligible_age;
      if (project_visible(A, kf, cam, img_c, img_r, margin)) {
        ++t.n_eligible_visible;
      }
    } else {
      ++t.n_excluded_age;
      if (project_visible(A, kf, cam, img_c, img_r, margin)) {
        ++t.n_hist_visible;
        if (A.n_kf_matched >= 3) {
          ++t.n_hist_visible_matched_ge3;
        }
      }
    }
  }
}

inline void note_match_time(double s)
{
  if (tick().active) {
    tick().match_time_s = s;
  }
}

inline void note_init_stats(
  int n_ssc,
  int n_raw,
  int n_inliers,
  int n_rej_min_dis,
  int n_triang,
  int n_rej_depth,
  int n_rej_birth,
  int n_births,
  double init_time_s,
  const std::vector<cv::Point2f> & uv_births)
{
  if (!tick().active) {
    return;
  }
  auto & t = tick();
  t.n_ssc_current = n_ssc;
  t.n_raw_matches = n_raw;
  t.n_fmat_inliers = n_inliers;
  t.n_rejected_min_distance = n_rej_min_dis;
  t.n_triang_submitted = n_triang;
  t.n_rejected_depth = n_rej_depth;
  t.n_rejected_birth_gate = n_rej_birth;
  t.n_births = n_births;
  t.init_time_s = init_time_s;
  t.uv_birth = uv_births;
}

inline void note_birth(
  int64_t anchor_idx,
  int64_t audit_id,
  int init_kf,
  uint64_t keyframe_id,
  const arma::vec::fixed<3> & xyz,
  const cv::Point2f & uv,
  double birth_rms)
{
  if (!enabled()) {
    return;
  }
  ensure_headers();
  auto & ids = stable_ids();
  while (static_cast<int64_t>(ids.size()) <= anchor_idx) {
    ids.push_back(-1);
  }
  const int64_t sid = next_stable_id();
  ids[static_cast<size_t>(anchor_idx)] = sid;

  std::ostringstream line;
  line << std::setprecision(9) << wall_s_now() << ',' << sid << ',' << anchor_idx
       << ',' << audit_id << ',' << init_kf << ',' << keyframe_id << ','
       << xyz[0] << ',' << xyz[1] << ',' << xyz[2] << ',' << uv.x << ',' << uv.y
       << ',' << (std::isfinite(birth_rms) ? birth_rms : -1.0) << ",2";
  std::lock_guard<std::mutex> lock(io_mutex());
  std::ofstream f(std::string(audit_dir()) + "/per_landmark_birth.csv", std::ios::app);
  if (f) {
    f << line.str() << '\n';
  }
}

inline void note_delete(
  int64_t anchor_idx,
  int64_t audit_id,
  int init_kf,
  int n_matched,
  int n_tries,
  int delete_kf_idx)
{
  if (!enabled()) {
    return;
  }
  ensure_headers();
  if (tick().active) {
    ++tick().n_deleted;
  }
  int64_t sid = -1;
  auto & ids = stable_ids();
  if (anchor_idx >= 0 && static_cast<size_t>(anchor_idx) < ids.size()) {
    sid = ids[static_cast<size_t>(anchor_idx)];
    ids.erase(ids.begin() + static_cast<std::ptrdiff_t>(anchor_idx));
  }
  std::ostringstream line;
  line << std::setprecision(9) << wall_s_now() << ',' << sid << ',' << anchor_idx
       << ',' << audit_id << ',' << init_kf << ',' << n_matched << ',' << n_tries
       << ',' << delete_kf_idx;
  std::lock_guard<std::mutex> lock(io_mutex());
  std::ofstream f(std::string(audit_dir()) + "/per_landmark_delete.csv", std::ios::app);
  if (f) {
    f << line.str() << '\n';
  }
}

inline void note_ba(
  int n_kf,
  int n_pts,
  int n_obs,
  double ba_time_s,
  bool accepted,
  int n_anchors_total)
{
  if (!enabled()) {
    return;
  }
  ensure_headers();
  if (tick().active) {
    tick().ba_ran = 1;
    tick().ba_n_kf = n_kf;
    tick().ba_n_pts = n_pts;
    tick().ba_time_s = ba_time_s;
    tick().ba_accepted = accepted ? 1 : 0;
    tick().ba_n_obs = n_obs;
  }
  std::ostringstream line;
  line << std::setprecision(9) << wall_s_now() << ','
       << (tick().active ? tick().kf_idx : -1) << ',' << n_kf << ',' << n_pts
       << ',' << n_obs << ',' << ba_time_s << ',' << (accepted ? 1 : 0) << ','
       << n_anchors_total;
  std::lock_guard<std::mutex> lock(io_mutex());
  std::ofstream f(std::string(audit_dir()) + "/ba_events.csv", std::ios::app);
  if (f) {
    f << line.str() << '\n';
  }
}

inline void grid_stats(
  const std::vector<cv::Point2f> & uv,
  int cols,
  int rows,
  int & occupied,
  std::vector<int> & counts)
{
  counts.assign(kGridW * kGridH, 0);
  occupied = 0;
  if (cols <= 0 || rows <= 0) {
    return;
  }
  for (const auto & p : uv) {
    int gx = static_cast<int>(std::floor(p.x / cols * kGridW));
    int gy = static_cast<int>(std::floor(p.y / rows * kGridH));
    if (gx < 0) {
      gx = 0;
    }
    if (gy < 0) {
      gy = 0;
    }
    if (gx >= kGridW) {
      gx = kGridW - 1;
    }
    if (gy >= kGridH) {
      gy = kGridH - 1;
    }
    ++counts[gy * kGridW + gx];
  }
  for (int c : counts) {
    if (c > 0) {
      ++occupied;
    }
  }
}

inline void flush_kf(const GLOBAL_MAP & Gmap, double update_time_s)
{
  if (!tick().active || !enabled()) {
    tick().active = false;
    return;
  }
  ensure_headers();
  auto & t = tick();
  const int cols = t.img_cols > 0 ? t.img_cols : 400;
  const int rows = t.img_rows > 0 ? t.img_rows : 300;

  std::vector<int> assoc_c, birth_c, comb_c;
  int assoc_occ = 0, birth_occ = 0, comb_occ = 0;
  grid_stats(t.uv_assoc, cols, rows, assoc_occ, assoc_c);
  grid_stats(t.uv_birth, cols, rows, birth_occ, birth_c);
  std::vector<cv::Point2f> comb = t.uv_assoc;
  comb.insert(comb.end(), t.uv_birth.begin(), t.uv_birth.end());
  grid_stats(comb, cols, rows, comb_occ, comb_c);

  int birth_in_supported = 0;
  for (size_t i = 0; i < birth_c.size(); ++i) {
    if (birth_c[i] > 0 && assoc_c[i] > 0) {
      birth_in_supported += birth_c[i];
    }
  }

  double mean_nn = -1.0;
  if (!t.uv_birth.empty() && !t.uv_assoc.empty()) {
    double sum = 0;
    for (const auto & b : t.uv_birth) {
      double best = 1e18;
      for (const auto & a : t.uv_assoc) {
        const double d =
          std::hypot(static_cast<double>(b.x - a.x), static_cast<double>(b.y - a.y));
        if (d < best) {
          best = d;
        }
      }
      sum += best;
    }
    mean_nn = sum / static_cast<double>(t.uv_birth.size());
  }

  int64_t n_obs = 0;
  for (const auto & kf : Gmap.KeyFDATA) {
    n_obs += static_cast<int64_t>(kf.Matched_landmark_ids.size());
  }

  const int n_after = static_cast<int>(Gmap.AnchorsDATA.size());

  std::ostringstream line;
  line << std::setprecision(9) << wall_s_now() << ',' << t.kf_idx << ','
       << t.keyframe_id << ',' << t.source_time_ms << ',' << t.est_x << ','
       << t.est_y << ',' << t.est_z << ',' << t.n_features << ',' << t.n_desc
       << ',' << cols << ',' << rows << ',' << t.n_anchors_before << ','
       << t.n_eligible_age << ',' << t.n_excluded_age << ','
       << t.n_eligible_visible << ',' << t.n_matched_existing << ','
       << t.n_hist_visible << ',' << t.n_hist_visible_matched_ge3 << ','
       << t.n_ssc_current << ',' << t.n_raw_matches << ',' << t.n_fmat_inliers
       << ',' << t.n_rejected_min_distance << ',' << t.n_triang_submitted << ','
       << t.n_rejected_depth << ',' << t.n_rejected_birth_gate << ','
       << t.n_births << ',' << t.n_deleted << ',' << n_after << ',' << n_obs
       << ',' << Gmap.KeyFDATA.size() << ',' << assoc_occ << ',' << birth_occ
       << ',' << comb_occ << ',' << birth_in_supported << ',' << mean_nn << ','
       << t.ba_ran << ',' << t.ba_n_kf << ',' << t.ba_n_pts << ',' << t.ba_time_s
       << ',' << t.ba_accepted << ',' << t.match_time_s << ',' << t.init_time_s
       << ',' << update_time_s << ',' << t.horizon_kf << ','
       << t.min_distance_px;

  {
    std::lock_guard<std::mutex> lock(io_mutex());
    std::ofstream f(std::string(audit_dir()) + "/per_keyframe.csv", std::ios::app);
    if (f) {
      f << line.str() << '\n';
    }
  }

  // Periodic thin snapshot (every 10 KF) for density / lifecycle offline.
  if (t.kf_idx >= 0 && (t.kf_idx % 10 == 0 || t.kf_idx < 3)) {
    std::ostringstream snap;
    snap << std::setprecision(6) << "{\"wall_s\":" << wall_s_now()
         << ",\"kf_idx\":" << t.kf_idx << ",\"n_anchors\":" << n_after
         << ",\"anchors\":[";
    bool first = true;
    for (size_t i = 0; i < Gmap.AnchorsDATA.size(); ++i) {
      const auto & A = Gmap.AnchorsDATA[i];
      if (!first) {
        snap << ',';
      }
      first = false;
      int64_t sid = -1;
      if (i < stable_ids().size()) {
        sid = stable_ids()[i];
      }
      snap << "{\"i\":" << i << ",\"sid\":" << sid << ",\"ikf\":" << A.init_KF
           << ",\"nm\":" << A.n_kf_matched << ",\"nt\":" << A.n_tries_matchs
           << ",\"x\":" << A.AnchorState[0] << ",\"y\":" << A.AnchorState[1]
           << ",\"z\":" << A.AnchorState[2] << '}';
    }
    snap << "]}";
    std::lock_guard<std::mutex> lock(io_mutex());
    std::ofstream f(std::string(audit_dir()) + "/anchor_snapshot.jsonl", std::ios::app);
    if (f) {
      f << snap.str() << '\n';
    }
  }

  t.active = false;
}

}  // namespace gmap_map_mgmt_audit

#endif
