#pragma once
// LSLAM-PERF-W1: disabled-by-default visual-stage profiling.
// Env: LSLAM_VISUAL_PROFILE=1 (also accepts UAV_SLAM_VISUAL_PROFILE=1).
// Rows are buffered under a mutex and flushed periodically — no per-frame
// fstream on the EKF worker. No EKF math changes. Reversible diagnostic.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace localslam {
namespace visual_profile {

inline bool env_truthy(const char * v)
{
  return v != nullptr &&
         (std::strcmp(v, "1") == 0 || std::strcmp(v, "true") == 0 ||
          std::strcmp(v, "yes") == 0);
}

inline bool enabled()
{
  static int cached = -1;
  if (cached < 0) {
    cached = (env_truthy(std::getenv("LSLAM_VISUAL_PROFILE")) ||
              env_truthy(std::getenv("UAV_SLAM_VISUAL_PROFILE")))
               ? 1
               : 0;
  }
  return cached == 1;
}

inline const char * path()
{
  static std::string p;
  if (p.empty()) {
    const char * v = std::getenv("LSLAM_VISUAL_PROFILE_PATH");
    if (v == nullptr || v[0] == '\0') {
      v = std::getenv("UAV_SLAM_VISUAL_PROFILE_PATH");
    }
    p = (v != nullptr && v[0] != '\0') ? v : "/tmp/lslam_visual_profile.csv";
  }
  return p.c_str();
}

inline const char * counters_path()
{
  static std::string p;
  if (p.empty()) {
    const char * v = std::getenv("LSLAM_DATABUFFER_PROFILE_PATH");
    if (v == nullptr || v[0] == '\0') {
      v = std::getenv("UAV_SLAM_DATABUFFER_PROFILE_PATH");
    }
    p = (v != nullptr && v[0] != '\0') ? v : "/tmp/lslam_databuffer_profile.csv";
  }
  return p.c_str();
}

inline const char * convert_path()
{
  static std::string p;
  if (p.empty()) {
    const char * v = std::getenv("LSLAM_CONVERT_PROFILE_PATH");
    p = (v != nullptr && v[0] != '\0') ? v : "/tmp/lslam_convert_profile.csv";
  }
  return p.c_str();
}

inline double ms_since(std::chrono::steady_clock::time_point t0)
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double, std::milli>(clock::now() - t0).count();
}

inline double mono_ms_now()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double, std::milli>(clock::now().time_since_epoch())
    .count();
}

struct StageRow {
  double mono_ms{0};
  double delete_ms{0};
  double detect_init_ms{0};
  double init_ms{0};
  double detect_match_ms{0};
  double match_ms{0};
  double correct_ms{0};
  double total_ms{0};
  int n_matches{0};
  int accepted{0};
  int n_feats{0};
  int n_anchors{0};
  int x_len{0};
  int img_w{0};
  int img_h{0};
};

struct ConvertRow {
  double mono_ms{0};
  double convert_ms{0};
  int img_w{0};
  int img_h{0};
};

inline std::mutex & buf_mu()
{
  static std::mutex m;
  return m;
}

inline std::vector<StageRow> & stage_buf()
{
  static std::vector<StageRow> v;
  return v;
}

inline std::vector<ConvertRow> & convert_buf()
{
  static std::vector<ConvertRow> v;
  return v;
}

// Nested timings filled by helpers before write_stage_row.
inline thread_local double tls_detect_init_ms = 0.0;
inline thread_local double tls_detect_match_ms = 0.0;

inline void note_detect_init_ms(double ms)
{
  tls_detect_init_ms = ms;
}

inline void note_detect_match_ms(double ms)
{
  tls_detect_match_ms = ms;
}

inline void write_stage_row(
  double delete_ms, double init_ms, double match_ms, double correct_ms, double total_ms,
  int n_matches, int accepted, int n_feats, int n_anchors, int x_len, int img_w, int img_h)
{
  if (!enabled()) {
    return;
  }
  StageRow r;
  r.mono_ms = mono_ms_now();
  r.delete_ms = delete_ms;
  r.detect_init_ms = tls_detect_init_ms;
  r.init_ms = init_ms;
  r.detect_match_ms = tls_detect_match_ms;
  r.match_ms = match_ms;
  r.correct_ms = correct_ms;
  r.total_ms = total_ms;
  r.n_matches = n_matches;
  r.accepted = accepted;
  r.n_feats = n_feats;
  r.n_anchors = n_anchors;
  r.x_len = x_len;
  r.img_w = img_w;
  r.img_h = img_h;
  tls_detect_init_ms = 0.0;
  tls_detect_match_ms = 0.0;
  std::lock_guard<std::mutex> lock(buf_mu());
  stage_buf().push_back(r);
}

inline void write_convert_row(double convert_ms, int img_w, int img_h)
{
  if (!enabled()) {
    return;
  }
  ConvertRow r;
  r.mono_ms = mono_ms_now();
  r.convert_ms = convert_ms;
  r.img_w = img_w;
  r.img_h = img_h;
  std::lock_guard<std::mutex> lock(buf_mu());
  convert_buf().push_back(r);
}

inline void write_databuffer_row(
  uint64_t enqueued, uint64_t overwritten, uint64_t consumed, uint64_t callback_rx)
{
  if (!enabled()) {
    return;
  }
  static std::mutex mu;
  static bool header_done = false;
  std::lock_guard<std::mutex> lock(mu);
  std::ofstream f(counters_path(), std::ios::app);
  if (!f.is_open()) {
    return;
  }
  if (!header_done) {
    f.seekp(0, std::ios::end);
    if (f.tellp() == 0) {
      f << "mono_ms,enqueued,overwritten,consumed,callback_rx\n";
    }
    header_done = true;
  }
  f << mono_ms_now() << ',' << enqueued << ',' << overwritten << ',' << consumed << ','
    << callback_rx << '\n';
}

inline void flush_pending()
{
  if (!enabled()) {
    return;
  }
  std::vector<StageRow> stages;
  std::vector<ConvertRow> converts;
  {
    std::lock_guard<std::mutex> lock(buf_mu());
    stages.swap(stage_buf());
    converts.swap(convert_buf());
  }
  if (!stages.empty()) {
    static bool stage_header = false;
    std::ofstream f(path(), std::ios::app);
    if (f.is_open()) {
      if (!stage_header) {
        f.seekp(0, std::ios::end);
        if (f.tellp() == 0) {
          f << "mono_ms,delete_ms,detect_init_ms,init_ms,detect_match_ms,match_ms,"
               "correct_ms,total_ms,n_matches,accepted,n_feats,n_anchors,x_len,img_w,img_h\n";
        }
        stage_header = true;
      }
      for (const auto & r : stages) {
        f << r.mono_ms << ',' << r.delete_ms << ',' << r.detect_init_ms << ',' << r.init_ms
          << ',' << r.detect_match_ms << ',' << r.match_ms << ',' << r.correct_ms << ','
          << r.total_ms << ',' << r.n_matches << ',' << r.accepted << ',' << r.n_feats << ','
          << r.n_anchors << ',' << r.x_len << ',' << r.img_w << ',' << r.img_h << '\n';
      }
    }
  }
  if (!converts.empty()) {
    static bool convert_header = false;
    std::ofstream f(convert_path(), std::ios::app);
    if (f.is_open()) {
      if (!convert_header) {
        f.seekp(0, std::ios::end);
        if (f.tellp() == 0) {
          f << "mono_ms,convert_ms,img_w,img_h\n";
        }
        convert_header = true;
      }
      for (const auto & r : converts) {
        f << r.mono_ms << ',' << r.convert_ms << ',' << r.img_w << ',' << r.img_h << '\n';
      }
    }
  }
}

}  // namespace visual_profile
}  // namespace localslam
