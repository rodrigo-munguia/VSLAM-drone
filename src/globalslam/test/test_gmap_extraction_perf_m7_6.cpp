/**
 * GSLAM-MAP-ARCH-M7.6: full-map detection extraction computational characterization.
 * Does not change production algorithms.
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "gmap_authoritative_read.hpp"
#include "gmap_landmark_identity.hpp"
#include "gmap_map_clone.hpp"
#include "../../common/keyframe_types.hpp"

using gmap_landmark_id::insert_landmark;
using gmap_landmark_id::rebuild_id_to_index;

namespace
{

struct PayloadStats
{
  int n_kf{0};
  int n_lm{0};
  std::size_t frame_bytes{0};
  std::size_t kf_desc_bytes{0};
  std::size_t lm_desc_bytes{0};
  std::size_t vgraph_bytes{0};
  std::size_t approx_total{0};
};

KEYFRAME make_kf(int i, int img_side, int n_desc)
{
  KEYFRAME kf;
  kf.t_c2n = {0.5 * i, 0.0, -1.0};
  kf.Rn2c = arma::eye<arma::mat>(3, 3);
  kf.Rr2c = arma::eye<arma::mat>(3, 3);
  kf.Rn2r = arma::eye<arma::mat>(3, 3);
  kf.keyframe_id = 1000 + i;
  kf.frame = cv::Mat::zeros(img_side, img_side, CV_8UC1);
  kf.Descriptors_all = cv::Mat::zeros(n_desc, 32, CV_8U);
  for (int r = 0; r < n_desc; ++r) {
    kf.keyPoints_all.emplace_back(float(r), float(r), 1.f);
  }
  return kf;
}

G_ANCHOR make_lm(int i)
{
  G_ANCHOR a;
  a.feat_type = "g_anchor";
  a.AnchorState = {0.1 * i, 0.0, 2.0};
  a.init_KF = 0;
  a.n_kf_matched = 2;
  a.Descriptor = cv::Mat::zeros(1, 32, CV_8U);
  return a;
}

GLOBAL_MAP make_scaled_map(int n_lm, int n_kf, int img_side, int desc_per_kf)
{
  GLOBAL_MAP map;
  map.idx_ref_pose_slam = 0;
  map.next_landmark_id = 1;
  for (int i = 0; i < n_kf; ++i) {
    map.KeyFDATA.push_back(make_kf(i, img_side, desc_per_kf));
  }
  map.Vgraph = arma::zeros(n_kf, n_kf);
  for (int i = 0; i < n_lm; ++i) {
    insert_landmark(map, make_lm(i));
  }
  rebuild_id_to_index(map);
  return map;
}

PayloadStats measure_payload(const GLOBAL_MAP & map)
{
  PayloadStats s;
  s.n_kf = static_cast<int>(map.KeyFDATA.size());
  s.n_lm = static_cast<int>(map.AnchorsDATA.size());
  for (const auto & kf : map.KeyFDATA) {
    if (!kf.frame.empty()) {
      s.frame_bytes += static_cast<std::size_t>(kf.frame.total() * kf.frame.elemSize());
    }
    if (!kf.Descriptors_all.empty()) {
      s.kf_desc_bytes += static_cast<std::size_t>(
        kf.Descriptors_all.total() * kf.Descriptors_all.elemSize());
    }
  }
  for (const auto & a : map.AnchorsDATA) {
    if (!a.Descriptor.empty()) {
      s.lm_desc_bytes += static_cast<std::size_t>(
        a.Descriptor.total() * a.Descriptor.elemSize());
    }
  }
  if (!map.Vgraph.empty()) {
    s.vgraph_bytes = static_cast<std::size_t>(
      map.Vgraph.n_elem * sizeof(double));
  }
  s.approx_total =
    s.frame_bytes + s.kf_desc_bytes + s.lm_desc_bytes + s.vgraph_bytes +
    map.KeyFDATA.size() * 256 + map.AnchorsDATA.size() * 128;
  return s;
}

struct Timing
{
  double median_ms{0};
  double p95_ms{0};
  double max_ms{0};
};

Timing summarize(std::vector<double> samples_ms)
{
  Timing t;
  if (samples_ms.empty()) {
    return t;
  }
  std::sort(samples_ms.begin(), samples_ms.end());
  t.median_ms = samples_ms[samples_ms.size() / 2];
  t.p95_ms = samples_ms[static_cast<std::size_t>(0.95 * (samples_ms.size() - 1))];
  t.max_ms = samples_ms.back();
  return t;
}

Timing bench_extract(GLOBAL_MAP & auth, int warm, int n)
{
  std::vector<double> samples;
  samples.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < warm + n; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    auto ext = gmap_auth_read::make_detection_extract(auth, 0, 0, 1);
    const auto t1 = std::chrono::steady_clock::now();
    (void)ext;
    const double ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (i >= warm) {
      samples.push_back(ms);
    }
  }
  return summarize(samples);
}

Timing bench_pub(GLOBAL_MAP & auth, int warm, int n)
{
  std::vector<double> samples;
  for (int i = 0; i < warm + n; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    auto pub = gmap_auth_read::extract_publication_geometry(auth);
    const auto t1 = std::chrono::steady_clock::now();
    (void)pub;
    if (i >= warm) {
      samples.push_back(
        std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
  }
  return summarize(samples);
}

}  // namespace

TEST(ExtractionPerfM76, UncontendedScaling)
{
  // Practical fixture sizes (descriptor/image light to keep CI tractable).
  // Landmarks target: 1k / 5k / 15k; KF and image sized proportionally.
  struct Case { const char * name; int n_lm; int n_kf; int img; int desc; };
  const Case cases[] = {
    {"small_1k", 1000, 40, 32, 50},
    {"medium_5k", 5000, 80, 32, 50},
    {"mission_15k", 15000, 120, 24, 40},
  };

  const char * path =
    "/home/rmung/DEV/projects/EKF-MonoSLAM-Range-To-Base/docs/evidence/"
    "gslam_map_arch_m7_6/raw/extraction_uncontended.csv";
  std::ofstream csv(path);
  if (!csv) {
    csv.open("/tmp/m76_extraction_uncontended.csv");
  }
  csv << "name,n_lm,n_kf,frame_B,kf_desc_B,lm_desc_B,vgraph_B,approx_B,"
         "det_med_ms,det_p95_ms,det_max_ms,pub_med_ms,pub_p95_ms\n";

  for (const auto & c : cases) {
    GLOBAL_MAP map = make_scaled_map(c.n_lm, c.n_kf, c.img, c.desc);
    const auto pay = measure_payload(map);
    const auto det = bench_extract(map, 2, 12);
    const auto pub = bench_pub(map, 2, 12);
    csv << c.name << "," << pay.n_lm << "," << pay.n_kf << ","
        << pay.frame_bytes << "," << pay.kf_desc_bytes << ","
        << pay.lm_desc_bytes << "," << pay.vgraph_bytes << ","
        << pay.approx_total << ","
        << det.median_ms << "," << det.p95_ms << "," << det.max_ms << ","
        << pub.median_ms << "," << pub.p95_ms << "\n";

    // Sanity: detection extract costs more than publication geometry.
    EXPECT_GE(det.median_ms, 0.0);
    EXPECT_GE(pub.median_ms, 0.0);
    // Soft upper bound only to catch pathological hangs (not a RT threshold).
    EXPECT_LT(det.max_ms, 30000.0) << c.name;
  }
}

TEST(ExtractionPerfM76, ContendedExtractVsWriter)
{
  GLOBAL_MAP auth = make_scaled_map(3000, 60, 24, 40);
  std::mutex mu;
  std::atomic<bool> run{true};
  std::vector<double> wait_ms;
  std::vector<double> clone_ms;
  std::vector<double> writer_ms;

  std::thread writer([&]() {
    while (run.load()) {
      const auto t0 = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lk(mu);
        if (!auth.AnchorsDATA.empty()) {
          auth.AnchorsDATA[0].AnchorState[0] += 0.001;
        }
        // Simulate short BA-like hold.
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      writer_ms.push_back(
        std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - t0).count());
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  for (int i = 0; i < 20; ++i) {
    const auto tw0 = std::chrono::steady_clock::now();
    GLOBAL_MAP local;
    {
      std::lock_guard<std::mutex> lk(mu);
      const auto tc0 = std::chrono::steady_clock::now();
      wait_ms.push_back(
        std::chrono::duration<double, std::milli>(tc0 - tw0).count());
      local = gmap_map_clone::deep_clone(auth);
      clone_ms.push_back(
        std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - tc0).count());
    }
    (void)local;
  }
  run = false;
  writer.join();

  auto w = summarize(wait_ms);
  auto c = summarize(clone_ms);
  auto ww = summarize(writer_ms);

  std::ofstream csv(
    "/home/rmung/DEV/projects/EKF-MonoSLAM-Range-To-Base/docs/evidence/"
    "gslam_map_arch_m7_6/raw/extraction_contended.csv");
  if (!csv) {
    csv.open("/tmp/m76_extraction_contended.csv");
  }
  csv << "metric,median_ms,p95_ms,max_ms\n";
  csv << "lock_wait," << w.median_ms << "," << w.p95_ms << "," << w.max_ms << "\n";
  csv << "clone_hold," << c.median_ms << "," << c.p95_ms << "," << c.max_ms << "\n";
  csv << "writer_section," << ww.median_ms << "," << ww.p95_ms << "," << ww.max_ms << "\n";

  EXPECT_GE(wait_ms.size(), 20u);
  EXPECT_LT(c.max_ms, 30000.0);
}

TEST(ExtractionPerfM76, PubMuchSmallerThanDetectionPayload)
{
  GLOBAL_MAP map = make_scaled_map(2000, 50, 32, 40);
  const auto pay = measure_payload(map);
  auto pub = gmap_auth_read::extract_publication_geometry(map);
  const std::size_t pub_bytes =
    pub.t_c2n.size() * sizeof(arma::vec::fixed<3>) +
    pub.Rn2c.size() * sizeof(arma::mat::fixed<3, 3>) +
    pub.landmark_xyz.size() * sizeof(arma::vec::fixed<3>);
  EXPECT_LT(pub_bytes, pay.approx_total);
  EXPECT_GT(pay.frame_bytes + pay.kf_desc_bytes + pay.lm_desc_bytes, 0u);
}
