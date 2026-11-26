#pragma once
// LSLAM-DYNAMIC-CONSISTENCY-A3: opt-in stage-tagged EKF state diagnostics.
// Env: LSLAM_DYNAMIC_DIAG=1
// Path: LSLAM_DYNAMIC_DIAG_PATH (default /tmp/lslam_dynamic_diag.csv)
// Buffered under mutex; flush periodically. No estimator math changes.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>
#include <cstdio>

#include <armadillo>

namespace localslam {
namespace dynamic_diag {

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
    cached = env_truthy(std::getenv("LSLAM_DYNAMIC_DIAG")) ? 1 : 0;
  }
  return cached == 1;
}

inline const char * path()
{
  static std::string p;
  if (p.empty()) {
    const char * v = std::getenv("LSLAM_DYNAMIC_DIAG_PATH");
    p = (v != nullptr && v[0] != '\0') ? v : "/tmp/lslam_dynamic_diag.csv";
  }
  return p.c_str();
}

inline double mono_ms_now()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double, std::milli>(clock::now().time_since_epoch())
    .count();
}

inline double wrap_pi(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

struct Row {
  double mono_ms{0};
  const char * stage{""};  // PRED / AHRS / VIS / ALT / RANGE
  const char * phase{""};  // before / after
  long int meas_time_ms{0};
  double pred_dt{0};
  double x{0}, y{0}, z{0};
  double phi{0}, theta{0}, psi{0};
  double vx{0}, vy{0}, vz{0};
  double wx{0}, wy{0}, wz{0};
  double Pxx{0}, Pyy{0}, Pzz{0};
  double Ppp{0}, Ptt{0}, Pss{0};
  double dx{0}, dy{0}, dz{0};
  double dphi{0}, dtheta{0}, dpsi{0};
  double dvx{0}, dvy{0}, dvz{0};
  int n_matches{-1};
  double inov_rms{-1};
  double comp_ms{0};
};

inline std::mutex & mu()
{
  static std::mutex m;
  return m;
}

inline std::vector<Row> & buf()
{
  static std::vector<Row> b;
  return b;
}

inline std::atomic<uint64_t> & seq()
{
  static std::atomic<uint64_t> s{0};
  return s;
}

inline void snapshot_state(
  Row & r, const arma::vec & x, const arma::mat & P)
{
  if (x.n_elem < 13 || P.n_rows < 13) {
    return;
  }
  r.phi = x(1);
  r.theta = x(2);
  r.psi = x(3);
  r.wx = x(4);
  r.wy = x(5);
  r.wz = x(6);
  r.x = x(7);
  r.y = x(8);
  r.z = x(9);
  r.vx = x(10);
  r.vy = x(11);
  r.vz = x(12);
  r.Ppp = P(1, 1);
  r.Ptt = P(2, 2);
  r.Pss = P(3, 3);
  r.Pxx = P(7, 7);
  r.Pyy = P(8, 8);
  r.Pzz = P(9, 9);
}

inline void fill_deltas(Row & after, const Row & before)
{
  after.dx = after.x - before.x;
  after.dy = after.y - before.y;
  after.dz = after.z - before.z;
  after.dphi = wrap_pi(after.phi - before.phi);
  after.dtheta = wrap_pi(after.theta - before.theta);
  after.dpsi = wrap_pi(after.psi - before.psi);
  after.dvx = after.vx - before.vx;
  after.dvy = after.vy - before.vy;
  after.dvz = after.vz - before.vz;
}

inline void write_rows_unlocked(const std::vector<Row> & rows)
{
  if (rows.empty()) {
    return;
  }
  const bool need_hdr = [&]() {
    std::ifstream in(path());
    return !in.good() || in.peek() == std::ifstream::traits_type::eof();
  }();
  std::ofstream f(path(), std::ios::app);
  if (!f) {
    return;
  }
  if (need_hdr) {
    f << "seq,mono_ms,stage,phase,meas_time_ms,pred_dt,"
         "x,y,z,phi,theta,psi,vx,vy,vz,wx,wy,wz,"
         "Pxx,Pyy,Pzz,Ppp,Ptt,Pss,"
         "dx,dy,dz,dphi,dtheta,dpsi,dvx,dvy,dvz,"
         "n_matches,inov_rms,comp_ms\n";
  }
  for (const auto & row : rows) {
    f << seq().fetch_add(1) << ',' << row.mono_ms << ',' << row.stage << ','
      << row.phase << ',' << row.meas_time_ms << ',' << row.pred_dt << ','
      << row.x << ',' << row.y << ',' << row.z << ',' << row.phi << ','
      << row.theta << ',' << row.psi << ',' << row.vx << ',' << row.vy << ','
      << row.vz << ',' << row.wx << ',' << row.wy << ',' << row.wz << ','
      << row.Pxx << ',' << row.Pyy << ',' << row.Pzz << ',' << row.Ppp << ','
      << row.Ptt << ',' << row.Pss << ',' << row.dx << ',' << row.dy << ','
      << row.dz << ',' << row.dphi << ',' << row.dtheta << ',' << row.dpsi
      << ',' << row.dvx << ',' << row.dvy << ',' << row.dvz << ','
      << row.n_matches << ',' << row.inov_rms << ',' << row.comp_ms << '\n';
  }
}

inline void push(Row r)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mu());
  buf().push_back(r);
  if (buf().size() >= 256) {
    write_rows_unlocked(buf());
    buf().clear();
  }
}

inline void flush()
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mu());
  write_rows_unlocked(buf());
  buf().clear();
}

}  // namespace dynamic_diag
}  // namespace localslam
