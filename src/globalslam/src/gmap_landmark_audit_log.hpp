/**
 * GSLAM-INIT-D2 audit-only landmark birth / association dump.
 *
 * Enabled only when environment variable GSLAM_LANDMARK_AUDIT_DIR is set to a
 * writable directory. Default (unset/empty): all calls are no-ops.
 *
 * Does not alter triangulation, matching, gates, BA, or map commit logic.
 * Assigns monotonic audit_id on G_ANCHOR when enabled (production ignores it).
 */
#ifndef GMAP_LANDMARK_AUDIT_LOG_HPP
#define GMAP_LANDMARK_AUDIT_LOG_HPP

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <armadillo>

#include "globalslam_types.hpp"
#include "gmap_birth_reproj.hpp"
#include "../../common/Vision/vision.hpp"

namespace gmap_landmark_audit
{

inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

inline const char * audit_dir()
{
  const char * d = std::getenv("GSLAM_LANDMARK_AUDIT_DIR");
  return (d && d[0] != '\0') ? d : nullptr;
}

inline bool enabled() { return audit_dir() != nullptr; }

inline std::mutex & io_mutex()
{
  static std::mutex m;
  return m;
}

inline int64_t next_audit_id()
{
  static std::atomic<int64_t> seq{0};
  return ++seq;
}

inline double wall_s_now()
{
  return std::chrono::duration<double>(
           std::chrono::system_clock::now().time_since_epoch())
    .count();
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
  if (!out) {
    return;
  }
  out << line << '\n';
}

inline double json_num(double v)
{
  if (!std::isfinite(v)) {
    return kNaN;
  }
  return v;
}

inline void write_num(std::ostream & out, const char * key, double v, bool first = false)
{
  if (!first) {
    out << ',';
  }
  out << '"' << key << "\":";
  if (!std::isfinite(v)) {
    out << "null";
  } else {
    out << v;
  }
}

struct BirthGeom
{
  double baseline_m{kNaN};
  double parallax_deg{kNaN};
  double depth_prev{kNaN};
  double depth_curr{kNaN};
  double baseline_depth_ratio{kNaN};
  double reproj_prev_px{kNaN};
  double reproj_curr_px{kNaN};
  double reproj_rms_px{kNaN};
  bool cheirality_ok{false};
};

inline double reproj_err_px(
  const arma::vec::fixed<3> & pt_n,
  const KEYFRAME & kf,
  const cv::Point2f & uv_distorted,
  const CAM & cam)
{
  // Same convention as production Init_birth_reproj gate (D3/D4).
  // Map nonfinite (behind-camera / invalid) to a large finite value for logging.
  const double e = gmap_birth_reproj::reproj_err_px(pt_n, kf, uv_distorted, cam);
  return std::isfinite(e) ? e : 1.0e6;
}

inline BirthGeom compute_birth_geom(
  const arma::vec::fixed<3> & pt_n,
  const KEYFRAME & kf_prev,
  const KEYFRAME & kf_curr,
  const cv::Point2f & uv_prev,
  const cv::Point2f & uv_curr,
  const CAM & cam)
{
  BirthGeom g;
  const arma::vec::fixed<3> c_prev = kf_prev.t_c2n;
  const arma::vec::fixed<3> c_curr = kf_curr.t_c2n;
  g.baseline_m = arma::norm(c_curr - c_prev);

  arma::vec::fixed<3> pc_prev = kf_prev.Rn2c * (pt_n - c_prev);
  arma::vec::fixed<3> pc_curr = kf_curr.Rn2c * (pt_n - c_curr);
  g.depth_prev = pc_prev[2];
  g.depth_curr = pc_curr[2];
  g.cheirality_ok = (pc_prev[2] > 0.0 && pc_curr[2] > 0.0);

  const arma::vec::fixed<3> r1 = pt_n - c_prev;
  const arma::vec::fixed<3> r2 = pt_n - c_curr;
  const double n1 = arma::norm(r1);
  const double n2 = arma::norm(r2);
  if (n1 > 1e-9 && n2 > 1e-9) {
    double c = arma::as_scalar(r1.t() * r2) / (n1 * n2);
    if (c > 1.0) {
      c = 1.0;
    }
    if (c < -1.0) {
      c = -1.0;
    }
    g.parallax_deg = std::acos(c) * 180.0 / M_PI;
  }
  const double depth_mean =
    (std::isfinite(g.depth_prev) && std::isfinite(g.depth_curr))
      ? 0.5 * (std::abs(g.depth_prev) + std::abs(g.depth_curr))
      : kNaN;
  if (std::isfinite(depth_mean) && depth_mean > 1e-9 && std::isfinite(g.baseline_m)) {
    g.baseline_depth_ratio = g.baseline_m / depth_mean;
  }

  g.reproj_prev_px = reproj_err_px(pt_n, kf_prev, uv_prev, cam);
  g.reproj_curr_px = reproj_err_px(pt_n, kf_curr, uv_curr, cam);
  if (std::isfinite(g.reproj_prev_px) && std::isfinite(g.reproj_curr_px)) {
    g.reproj_rms_px =
      std::sqrt(0.5 * (g.reproj_prev_px * g.reproj_prev_px + g.reproj_curr_px * g.reproj_curr_px));
  }
  return g;
}

inline int64_t assign_audit_id(G_ANCHOR & anchor)
{
  if (!enabled()) {
    return -1;
  }
  if (anchor.audit_id > 0) {
    return anchor.audit_id;
  }
  anchor.audit_id = next_audit_id();
  return anchor.audit_id;
}

inline void log_birth_accepted(
  G_ANCHOR & anchor,
  int64_t idx_pt,
  int idx_kf_prev,
  int idx_kf_curr,
  int64_t keyframe_id_prev,
  int64_t keyframe_id_curr,
  int feat_idx_prev,
  int feat_idx_curr,
  const cv::Point2f & uv_prev,
  const cv::Point2f & uv_curr,
  float match_distance,
  bool ratio_passed,
  bool fransac_inlier,
  const char * triang_method,
  const BirthGeom & geom,
  const char * measured_not_gated)
{
  if (!enabled()) {
    return;
  }
  const int64_t aid = assign_audit_id(anchor);
  std::ostringstream out;
  out << std::setprecision(9);
  out << "{\"event\":\"birth_accepted\""
      << ",\"wall_s\":" << wall_s_now()
      << ",\"audit_id\":" << aid
      << ",\"idx_pt\":" << idx_pt
      << ",\"idx_kf_prev\":" << idx_kf_prev
      << ",\"idx_kf_curr\":" << idx_kf_curr
      << ",\"keyframe_id_prev\":" << keyframe_id_prev
      << ",\"keyframe_id_curr\":" << keyframe_id_curr
      << ",\"feat_idx_prev\":" << feat_idx_prev
      << ",\"feat_idx_curr\":" << feat_idx_curr
      << ",\"uv_prev\":[" << uv_prev.x << ',' << uv_prev.y << ']'
      << ",\"uv_curr\":[" << uv_curr.x << ',' << uv_curr.y << ']'
      << ",\"pt_n\":[" << anchor.AnchorState[0] << ',' << anchor.AnchorState[1] << ','
      << anchor.AnchorState[2] << ']'
      << ",\"match_distance\":" << match_distance
      << ",\"ratio_passed\":" << (ratio_passed ? "true" : "false")
      << ",\"fransac_inlier\":" << (fransac_inlier ? "true" : "false")
      << ",\"triang_method\":\"" << triang_method << '"';
  write_num(out, "baseline_m", geom.baseline_m);
  write_num(out, "parallax_deg", geom.parallax_deg);
  write_num(out, "depth_prev", geom.depth_prev);
  write_num(out, "depth_curr", geom.depth_curr);
  write_num(out, "baseline_depth_ratio", geom.baseline_depth_ratio);
  write_num(out, "reproj_prev_px", geom.reproj_prev_px);
  write_num(out, "reproj_curr_px", geom.reproj_curr_px);
  write_num(out, "reproj_rms_px", geom.reproj_rms_px);
  out << ",\"cheirality_ok\":" << (geom.cheirality_ok ? "true" : "false")
      << ",\"measured_not_gated\":\"" << measured_not_gated << "\"}";
  append_line("landmark_births.jsonl", out.str());
}

inline void log_birth_rejected(
  int idx_kf_prev,
  int idx_kf_curr,
  int64_t keyframe_id_prev,
  int64_t keyframe_id_curr,
  int feat_idx_prev,
  int feat_idx_curr,
  const cv::Point2f & uv_prev,
  const cv::Point2f & uv_curr,
  float match_distance,
  const char * reason,
  const char * triang_method,
  const BirthGeom * geom_or_null)
{
  if (!enabled()) {
    return;
  }
  std::ostringstream out;
  out << std::setprecision(9);
  out << "{\"event\":\"birth_rejected\""
      << ",\"wall_s\":" << wall_s_now()
      << ",\"idx_kf_prev\":" << idx_kf_prev
      << ",\"idx_kf_curr\":" << idx_kf_curr
      << ",\"keyframe_id_prev\":" << keyframe_id_prev
      << ",\"keyframe_id_curr\":" << keyframe_id_curr
      << ",\"feat_idx_prev\":" << feat_idx_prev
      << ",\"feat_idx_curr\":" << feat_idx_curr
      << ",\"uv_prev\":[" << uv_prev.x << ',' << uv_prev.y << ']'
      << ",\"uv_curr\":[" << uv_curr.x << ',' << uv_curr.y << ']'
      << ",\"match_distance\":" << match_distance
      << ",\"reason\":\"" << reason << '"'
      << ",\"triang_method\":\"" << triang_method << '"';
  if (geom_or_null) {
    write_num(out, "baseline_m", geom_or_null->baseline_m);
    write_num(out, "parallax_deg", geom_or_null->parallax_deg);
    write_num(out, "depth_prev", geom_or_null->depth_prev);
    write_num(out, "depth_curr", geom_or_null->depth_curr);
    write_num(out, "reproj_rms_px", geom_or_null->reproj_rms_px);
    out << ",\"cheirality_ok\":" << (geom_or_null->cheirality_ok ? "true" : "false");
  } else {
    out << ",\"baseline_m\":null,\"parallax_deg\":null,\"depth_prev\":null,"
           "\"depth_curr\":null,\"reproj_rms_px\":null,\"cheirality_ok\":null";
  }
  out << '}';
  append_line("landmark_births.jsonl", out.str());
}

inline void log_association(
  int64_t audit_id,
  int64_t idx_pt,
  int idx_kf,
  int64_t keyframe_id,
  int feat_idx,
  double uv_x,
  double uv_y,
  double inov_px,
  float match_distance,
  const char * method,  // "match_existing_into_new_kf" | "match_new_into_previous_kf" | "birth"
  bool is_birth_observation)
{
  if (!enabled() || audit_id <= 0) {
    return;
  }
  std::ostringstream out;
  out << std::setprecision(9);
  out << "{\"event\":\"assoc\""
      << ",\"wall_s\":" << wall_s_now()
      << ",\"audit_id\":" << audit_id
      << ",\"idx_pt\":" << idx_pt
      << ",\"idx_kf\":" << idx_kf
      << ",\"keyframe_id\":" << keyframe_id
      << ",\"feat_idx\":" << feat_idx
      << ",\"uv\":[" << uv_x << ',' << uv_y << ']';
  write_num(out, "inov_px", inov_px);
  out << ",\"match_distance\":" << match_distance
      << ",\"method\":\"" << method << '"'
      << ",\"is_birth_observation\":" << (is_birth_observation ? "true" : "false")
      << '}';
  append_line("landmark_assoc.jsonl", out.str());
}

inline void log_delete(int64_t audit_id, int64_t idx_pt, int n_kf_matched, int n_tries)
{
  if (!enabled() || audit_id <= 0) {
    return;
  }
  std::ostringstream out;
  out << "{\"event\":\"delete\""
      << ",\"wall_s\":" << wall_s_now()
      << ",\"audit_id\":" << audit_id
      << ",\"idx_pt\":" << idx_pt
      << ",\"n_kf_matched\":" << n_kf_matched
      << ",\"n_tries_matchs\":" << n_tries
      << '}';
  append_line("landmark_deletes.jsonl", out.str());
}

/** Per-observation pre-BA residuals using current map state (no optimized Values). */
inline void append_ba_obs_pre_jsonl(
  int ba_id,
  bool accepted,
  const GLOBAL_MAP & Gmap,
  const std::vector<int> & idx_kf_to_optimize,
  const CAM & cam)
{
  if (!enabled()) {
    return;
  }
  std::ostringstream out;
  out << std::setprecision(9);
  out << "{\"event\":\"ba_obs\",\"ba_id\":" << ba_id
      << ",\"accepted\":" << (accepted ? "true" : "false")
      << ",\"wall_s\":" << wall_s_now()
      << ",\"obs\":[";
  bool first = true;
  for (int idx_kf : idx_kf_to_optimize) {
    const auto & kf = Gmap.KeyFDATA[idx_kf];
    for (size_t j = 0; j < kf.Idx_Matched_points.size(); ++j) {
      const int64_t idx_pt = kf.Idx_Matched_points[j];
      if (idx_pt < 0 || static_cast<size_t>(idx_pt) >= Gmap.AnchorsDATA.size()) {
        continue;
      }
      const auto & A = Gmap.AnchorsDATA[idx_pt];
      const double e = reproj_err_px(A.AnchorState, kf, kf.UV_Matched_points[j], cam);
      // Birth pair: init_KF is current KF at triangulation; previous is init_KF-1.
      const bool is_birth =
        (idx_kf == A.init_KF) || (A.init_KF > 0 && idx_kf == A.init_KF - 1);
      if (!first) {
        out << ',';
      }
      first = false;
      out << "{\"audit_id\":" << A.audit_id
          << ",\"idx_pt\":" << idx_pt
          << ",\"idx_kf\":" << idx_kf
          << ",\"keyframe_id\":" << kf.keyframe_id
          << ",\"init_kf\":" << A.init_KF
          << ",\"n_kf_matched\":" << A.n_kf_matched
          << ",\"is_birth_kf_pair\":" << (is_birth ? "true" : "false");
      write_num(out, "pre_reproj_px", e);
      out << '}';
    }
  }
  out << "]}";
  append_line("ba_window_obs.jsonl", out.str());
}

}  // namespace gmap_landmark_audit

#endif
