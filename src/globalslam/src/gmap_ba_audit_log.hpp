/**
 * GSLAM-BA-D1 audit-only BA window dump.
 *
 * Enabled only when environment variable GSLAM_BA_AUDIT_DIR is set to an
 * existing/writable directory. Default (unset/empty): all calls are no-ops and
 * production BA behavior is unchanged.
 *
 * Does not alter thresholds, noise models, priors, or commit logic.
 */
#ifndef GMAP_BA_AUDIT_LOG_HPP
#define GMAP_BA_AUDIT_LOG_HPP

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include "globalslam_types.hpp"
#include "../../common/Vision/vision.hpp"

namespace gmap_ba_audit
{

inline const char * audit_dir()
{
  const char * d = std::getenv("GSLAM_BA_AUDIT_DIR");
  return (d && d[0] != '\0') ? d : nullptr;
}

inline bool enabled() { return audit_dir() != nullptr; }

inline gtsam::Pose3 pose_from_kf(const KEYFRAME & kf)
{
  arma::mat::fixed<3, 3> Rc2n = kf.Rn2c.t();
  gtsam::Rot3 R(
    Rc2n.at(0, 0), Rc2n.at(0, 1), Rc2n.at(0, 2),
    Rc2n.at(1, 0), Rc2n.at(1, 1), Rc2n.at(1, 2),
    Rc2n.at(2, 0), Rc2n.at(2, 1), Rc2n.at(2, 2));
  return gtsam::Pose3(R, gtsam::Point3(kf.t_c2n[0], kf.t_c2n[1], kf.t_c2n[2]));
}

struct ResidualStats
{
  int n{0};
  double sum{0};
  double sum_sq{0};
  double max_abs{0};
  int n_above_5px{0};
  int n_above_10px{0};
  int n_above_20px{0};
  void add(double e)
  {
    ++n;
    sum += e;
    sum_sq += e * e;
    if (e > max_abs) {
      max_abs = e;
    }
    if (e > 5.0) {
      ++n_above_5px;
    }
    if (e > 10.0) {
      ++n_above_10px;
    }
    if (e > 20.0) {
      ++n_above_20px;
    }
  }
  double mean() const { return n ? sum / n : 0.0; }
  double rms() const { return n ? std::sqrt(sum_sq / n) : 0.0; }
};

inline ResidualStats compute_residual_stats(
  const GLOBAL_MAP & Gmap,
  const std::vector<int> & idx_kf_to_optimize,
  const CAM & cam,
  gtsam::Cal3_S2::shared_ptr K,
  const gtsam::Values * values_opt)  // nullptr → use map poses/landmarks
{
  ResidualStats st;
  for (int idx_kf : idx_kf_to_optimize) {
    gtsam::Pose3 pose;
    if (values_opt && values_opt->exists(gtsam::Symbol('x', idx_kf))) {
      pose = values_opt->at<gtsam::Pose3>(gtsam::Symbol('x', idx_kf));
    } else {
      pose = pose_from_kf(Gmap.KeyFDATA[idx_kf]);
    }
    gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pose, *K);
    const auto & kf = Gmap.KeyFDATA[idx_kf];
    for (size_t j = 0; j < kf.Idx_Matched_points.size(); ++j) {
      const int64 idx_pt = kf.Idx_Matched_points[j];
      if (idx_pt < 0 || static_cast<size_t>(idx_pt) >= Gmap.AnchorsDATA.size()) {
        continue;
      }
      gtsam::Point3 X;
      if (values_opt && values_opt->exists(gtsam::Symbol('l', idx_pt))) {
        X = values_opt->at<gtsam::Point3>(gtsam::Symbol('l', idx_pt));
      } else {
        const auto & a = Gmap.AnchorsDATA[idx_pt].AnchorState;
        X = gtsam::Point3(a[0], a[1], a[2]);
      }
      try {
        const gtsam::Point2 zhat = camera.project(X);
        const cv::Point2f ud = Undistort_a_point(kf.UV_Matched_points[j], cam, 1);
        const double dx = zhat.x() - ud.x;
        const double dy = zhat.y() - ud.y;
        st.add(std::sqrt(dx * dx + dy * dy));
      } catch (...) {
        st.add(1.0e6);
      }
    }
  }
  (void)cam;
  return st;
}

struct LandmarkGeo
{
  int64 idx_pt{-1};
  int init_kf{-1};
  int n_obs_in_window{0};
  int n_kf_matched{0};
  double depth_mean{0};
  double baseline_max{0};
  double parallax_max_deg{0};
  bool behind_camera{false};
};

inline std::vector<LandmarkGeo> landmark_geometry(
  const GLOBAL_MAP & Gmap,
  const std::vector<int> & idx_kf_to_optimize,
  const std::vector<int64> & idx_points)
{
  std::vector<LandmarkGeo> out;
  out.reserve(idx_points.size());
  for (int64 idx_pt : idx_points) {
    LandmarkGeo g;
    g.idx_pt = idx_pt;
    if (idx_pt < 0 || static_cast<size_t>(idx_pt) >= Gmap.AnchorsDATA.size()) {
      out.push_back(g);
      continue;
    }
    const auto & A = Gmap.AnchorsDATA[idx_pt];
    g.init_kf = A.init_KF;
    g.n_kf_matched = A.n_kf_matched;
    const gtsam::Point3 X(A.AnchorState[0], A.AnchorState[1], A.AnchorState[2]);

    std::vector<int> observers;
    std::vector<gtsam::Point3> cams;
    std::vector<double> depths;
    for (int idx_kf : idx_kf_to_optimize) {
      const auto & kf = Gmap.KeyFDATA[idx_kf];
      bool sees = false;
      for (int64 m : kf.Idx_Matched_points) {
        if (m == idx_pt) {
          sees = true;
          break;
        }
      }
      if (!sees) {
        continue;
      }
      ++g.n_obs_in_window;
      observers.push_back(idx_kf);
      const gtsam::Point3 t(kf.t_c2n[0], kf.t_c2n[1], kf.t_c2n[2]);
      cams.push_back(t);
      arma::vec::fixed<3> pc = kf.Rn2c * (A.AnchorState - kf.t_c2n);
      depths.push_back(pc[2]);
      if (pc[2] <= 0.0) {
        g.behind_camera = true;
      }
    }
    if (!depths.empty()) {
      double s = 0;
      for (double d : depths) {
        s += d;
      }
      g.depth_mean = s / depths.size();
    }
    for (size_t i = 0; i < cams.size(); ++i) {
      for (size_t j = i + 1; j < cams.size(); ++j) {
        const double base = (cams[i] - cams[j]).norm();
        if (base > g.baseline_max) {
          g.baseline_max = base;
        }
        const gtsam::Point3 r1 = (X - cams[i]);
        const gtsam::Point3 r2 = (X - cams[j]);
        const double n1 = r1.norm();
        const double n2 = r2.norm();
        if (n1 > 1e-9 && n2 > 1e-9) {
          double c = r1.dot(r2) / (n1 * n2);
          if (c > 1.0) {
            c = 1.0;
          }
          if (c < -1.0) {
            c = -1.0;
          }
          const double deg = std::acos(c) * 180.0 / M_PI;
          if (deg > g.parallax_max_deg) {
            g.parallax_max_deg = deg;
          }
        }
      }
    }
    out.push_back(g);
  }
  return out;
}

inline void append_window_jsonl(
  int ba_id,
  double wall_s,
  const GLOBAL_MAP & Gmap_before_commit,
  const std::vector<int> & idx_kf_to_optimize,
  const std::vector<int64> & idx_points,
  const CAM & cam,
  gtsam::Cal3_S2::shared_ptr K,
  const gtsam::NonlinearFactorGraph & graph,
  const gtsam::Values & initial,
  const gtsam::Values & result,
  bool accepted,
  const ResidualStats & pre,
  const ResidualStats & post,
  double elapsed_s,
  const std::vector<std::array<double, 6>> & kf_deltas,  // dx dy dz droll dpitch dyaw approx
  bool map_committed)
{
  const char * dir = audit_dir();
  if (!dir) {
    return;
  }
  std::ostringstream path;
  path << dir << "/ba_windows.jsonl";
  std::ofstream out(path.str(), std::ios::app);
  if (!out) {
    return;
  }

  auto geo = landmark_geometry(Gmap_before_commit, idx_kf_to_optimize, idx_points);
  int n_obs = 0;
  for (int idx_kf : idx_kf_to_optimize) {
    n_obs += static_cast<int>(Gmap_before_commit.KeyFDATA[idx_kf].Idx_Matched_points.size());
  }

  double parallax_min = 1e9, parallax_med = 0, depth_min = 1e9, depth_med = 0;
  int n_behind = 0, n_low_par = 0, n_single_obs = 0;
  std::vector<double> pars, deps;
  for (const auto & g : geo) {
    if (g.behind_camera) {
      ++n_behind;
    }
    if (g.n_obs_in_window <= 1) {
      ++n_single_obs;
    }
    if (g.parallax_max_deg > 0) {
      pars.push_back(g.parallax_max_deg);
      if (g.parallax_max_deg < 1.0) {
        ++n_low_par;
      }
    }
    if (g.depth_mean > 0) {
      deps.push_back(g.depth_mean);
    }
  }
  auto median = [](std::vector<double> v) {
    if (v.empty()) {
      return 0.0;
    }
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
  };
  if (!pars.empty()) {
    parallax_min = *std::min_element(pars.begin(), pars.end());
    parallax_med = median(pars);
  } else {
    parallax_min = 0;
  }
  if (!deps.empty()) {
    depth_min = *std::min_element(deps.begin(), deps.end());
    depth_med = median(deps);
  } else {
    depth_min = 0;
  }

  double max_xy = 0, max_z = 0;
  for (const auto & d : kf_deltas) {
    const double xy = std::sqrt(d[0] * d[0] + d[1] * d[1]);
    if (xy > max_xy) {
      max_xy = xy;
    }
    if (std::abs(d[2]) > max_z) {
      max_z = std::abs(d[2]);
    }
  }

  out << std::setprecision(9);
  out << "{\"ba_id\":" << ba_id
      << ",\"wall_s\":" << wall_s
      << ",\"elapsed_s\":" << elapsed_s
      << ",\"accepted\":" << (accepted ? "true" : "false")
      << ",\"map_committed\":" << (map_committed ? "true" : "false")
      << ",\"reject_reason\":\"" << (accepted ? "" : "Delta_norm_xy") << "\""
      << ",\"n_kf_map\":" << Gmap_before_commit.KeyFDATA.size()
      << ",\"n_anchors_map\":" << Gmap_before_commit.AnchorsDATA.size()
      << ",\"n_kf_window\":" << idx_kf_to_optimize.size()
      << ",\"n_landmarks_window\":" << idx_points.size()
      << ",\"n_observations\":" << n_obs
      << ",\"obj_initial\":" << graph.error(initial)
      << ",\"obj_final\":" << graph.error(result)
      << ",\"pre_reproj_mean\":" << pre.mean()
      << ",\"pre_reproj_rms\":" << pre.rms()
      << ",\"pre_reproj_max\":" << pre.max_abs
      << ",\"pre_n_gt5\":" << pre.n_above_5px
      << ",\"pre_n_gt10\":" << pre.n_above_10px
      << ",\"pre_n_gt20\":" << pre.n_above_20px
      << ",\"post_reproj_mean\":" << post.mean()
      << ",\"post_reproj_rms\":" << post.rms()
      << ",\"post_reproj_max\":" << post.max_abs
      << ",\"post_n_gt5\":" << post.n_above_5px
      << ",\"post_n_gt10\":" << post.n_above_10px
      << ",\"post_n_gt20\":" << post.n_above_20px
      << ",\"max_kf_xy_delta\":" << max_xy
      << ",\"max_kf_abs_dz\":" << max_z
      << ",\"parallax_min_deg\":" << parallax_min
      << ",\"parallax_med_deg\":" << parallax_med
      << ",\"depth_min\":" << depth_min
      << ",\"depth_med\":" << depth_med
      << ",\"n_behind_camera\":" << n_behind
      << ",\"n_low_parallax_lt1deg\":" << n_low_par
      << ",\"n_single_obs_in_window\":" << n_single_obs
      << ",\"kf_idx\":[";
  for (size_t i = 0; i < idx_kf_to_optimize.size(); ++i) {
    if (i) {
      out << ',';
    }
    out << idx_kf_to_optimize[i];
  }
  out << "],\"kf_keyframe_id\":[";
  for (size_t i = 0; i < idx_kf_to_optimize.size(); ++i) {
    if (i) {
      out << ',';
    }
    out << Gmap_before_commit.KeyFDATA[idx_kf_to_optimize[i]].keyframe_id;
  }
  out << "],\"kf_pose_before\":[";
  for (size_t i = 0; i < idx_kf_to_optimize.size(); ++i) {
    if (i) {
      out << ',';
    }
    const auto & t = Gmap_before_commit.KeyFDATA[idx_kf_to_optimize[i]].t_c2n;
    out << '[' << t[0] << ',' << t[1] << ',' << t[2] << ']';
  }
  out << "],\"kf_delta_xyz\":[";
  for (size_t i = 0; i < kf_deltas.size(); ++i) {
    if (i) {
      out << ',';
    }
    out << '[' << kf_deltas[i][0] << ',' << kf_deltas[i][1] << ',' << kf_deltas[i][2] << ']';
  }
  out << "],\"landmark_idx_sample\":[";
  const size_t n_sample = std::min<size_t>(idx_points.size(), 32);
  for (size_t i = 0; i < n_sample; ++i) {
    if (i) {
      out << ',';
    }
    out << idx_points[i];
  }
  out << "],\"landmark_geo_sample\":[";
  for (size_t i = 0; i < std::min(geo.size(), n_sample); ++i) {
    if (i) {
      out << ',';
    }
    const auto & g = geo[i];
    const int64_t audit_id =
      (g.idx_pt >= 0 &&
       static_cast<size_t>(g.idx_pt) < Gmap_before_commit.AnchorsDATA.size())
        ? Gmap_before_commit.AnchorsDATA[g.idx_pt].audit_id
        : static_cast<int64_t>(-1);
    out << "{\"pt\":" << g.idx_pt
        << ",\"audit_id\":" << audit_id
        << ",\"init_kf\":" << g.init_kf
        << ",\"n_obs\":" << g.n_obs_in_window
        << ",\"n_matched\":" << g.n_kf_matched
        << ",\"depth\":" << g.depth_mean
        << ",\"base\":" << g.baseline_max
        << ",\"par_deg\":" << g.parallax_max_deg
        << ",\"behind\":" << (g.behind_camera ? "true" : "false")
        << '}';
  }
  out << "]}\n";
}

}  // namespace gmap_ba_audit

#endif
