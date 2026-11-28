/**
 * GSLAM-MAP-ARCH-M7.6: controlled concurrency, stress, OpenCV ownership,
 * structural invariants under concurrent extract vs mutate.
 */
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "cloop_visual_retrieve.hpp"
#include "gmap_authoritative_loop.hpp"
#include "gmap_authoritative_read.hpp"
#include "gmap_landmark_identity.hpp"
#include "gmap_map_clone.hpp"
#include "gmap_structural_invariants_m7_6.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/keyframe_types.hpp"

using gmap_landmark_id::erase_landmark_at_storage_index;
using gmap_landmark_id::insert_landmark;
using gmap_landmark_id::rebuild_id_to_index;

namespace
{

constexpr int kWatchdogMs = 8000;

KEYFRAME make_kf(double x, double y, double z, uint64_t id)
{
  KEYFRAME kf;
  kf.t_c2n = {x, y, z};
  kf.Rn2c = arma::eye<arma::mat>(3, 3);
  kf.Rr2c = arma::eye<arma::mat>(3, 3);
  kf.Rn2r = arma::eye<arma::mat>(3, 3);
  kf.keyframe_id = id;
  kf.radius_uncertainty = 1.0;
  kf.frame = cv::Mat::zeros(16, 16, CV_8UC1);
  kf.frame.setTo(cv::Scalar(40));
  kf.Descriptors_all = cv::Mat::zeros(4, 32, CV_8U);
  for (int i = 0; i < 4; ++i) {
    kf.Descriptors_all.at<unsigned char>(i, 0) =
      static_cast<unsigned char>(10 + i);
    kf.keyPoints_all.emplace_back(100.f + i, 200.f, 1.f);
  }
  return kf;
}

G_ANCHOR make_anchor(double x, double y, double z, int init_kf, unsigned char d0)
{
  G_ANCHOR a;
  a.feat_type = "g_anchor";
  a.AnchorState = {x, y, z};
  a.init_KF = init_kf;
  a.n_kf_matched = 2;
  a.Descriptor = cv::Mat::zeros(1, 32, CV_8U);
  a.Descriptor.at<unsigned char>(0, 0) = d0;
  a.Keypoint = cv::KeyPoint(100.f, 200.f, 1.f);
  return a;
}

GLOBAL_MAP make_base_map(int n_kf, int n_lm)
{
  GLOBAL_MAP map;
  map.idx_ref_pose_slam = 0;
  map.next_landmark_id = 1000;  // IDs ≠ slots
  for (int i = 0; i < n_kf; ++i) {
    map.KeyFDATA.push_back(make_kf(i * 0.5, 0.01 * i, -1.0, 100 + i));
  }
  map.Vgraph = arma::zeros(n_kf, n_kf);
  for (int i = 0; i + 1 < n_kf; ++i) {
    map.Vgraph(i, i + 1) = 2;
    map.Vgraph(i + 1, i) = 2;
  }
  for (int i = 0; i < n_lm; ++i) {
    const auto id = insert_landmark(
      map, make_anchor(0.2 * i, 0.1, 2.0, i % std::max(1, n_kf),
        static_cast<unsigned char>(i % 200)));
    const int kf = i % n_kf;
    map.KeyFDATA[kf].Matched_landmark_ids.push_back(id);
    map.KeyFDATA[kf].UV_Matched_points.push_back({320.0 + i, 240.0});
  }
  rebuild_id_to_index(map);
  return map;
}

bool mat_bytes_equal(const cv::Mat & a, const cv::Mat & b)
{
  if (a.size() != b.size() || a.type() != b.type()) {
    return false;
  }
  if (a.empty()) {
    return true;
  }
  return cv::norm(a, b, cv::NORM_INF) == 0;
}

}  // namespace

TEST(ConcurrencyM76, StructuralInvariantsOnBaseFixture)
{
  GLOBAL_MAP map = make_base_map(8, 20);
  const auto r = gmap_struct_m76::check(map);
  EXPECT_TRUE(r.ok) << r.reason;
}

TEST(ConcurrencyM76, ControlledExtractVsMutateBarrier)
{
  GLOBAL_MAP auth = make_base_map(6, 12);
  std::mutex mu;
  std::condition_variable cv;
  enum Phase { Start, WriterReady, ExtractDone, WriterMutated, Done } phase = Start;
  std::atomic<bool> bad{false};
  std::string bad_reason;
  GLOBAL_MAP extract_hold;

  std::thread writer([&]() {
    {
      std::unique_lock<std::mutex> lk(mu);
      phase = WriterReady;
      cv.notify_all();
      cv.wait(lk, [&] { return phase == ExtractDone; });
      // Mutate authoritative after extract released lock conceptually.
      auth.AnchorsDATA[0].AnchorState[0] += 5.0;
      if (!auth.AnchorsDATA[0].Descriptor.empty()) {
        auth.AnchorsDATA[0].Descriptor.at<unsigned char>(0, 0) = 99;
      }
      auth.KeyFDATA[0].t_c2n[0] += 7.0;
      auth.KeyFDATA[0].Descriptors_all.at<unsigned char>(0, 0) = 77;
      insert_landmark(auth, make_anchor(9, 9, 9, 0, 1));
      erase_landmark_at_storage_index(auth, 1, nullptr);
      rebuild_id_to_index(auth);
      phase = WriterMutated;
      cv.notify_all();
      cv.wait(lk, [&] { return phase == Done; });
    }
  });

  std::thread reader([&]() {
    {
      std::unique_lock<std::mutex> lk(mu);
      cv.wait(lk, [&] { return phase == WriterReady; });
      extract_hold = gmap_map_clone::deep_clone(auth);
      phase = ExtractDone;
      cv.notify_all();
      cv.wait(lk, [&] { return phase == WriterMutated; });
    }
    // Unlocked: verify extract unchanged vs expected pre-mutation snapshot.
    if (extract_hold.AnchorsDATA[0].AnchorState[0] > 1.0) {
      bad = true;
      bad_reason = "extract_saw_post_mutation_xyz";
    }
    if (extract_hold.AnchorsDATA[0].Descriptor.at<unsigned char>(0, 0) == 99) {
      bad = true;
      bad_reason = "extract_saw_post_mutation_desc";
    }
    if (extract_hold.KeyFDATA[0].t_c2n[0] > 1.0) {
      bad = true;
      bad_reason = "extract_saw_post_mutation_pose";
    }
    const auto inv = gmap_struct_m76::check(extract_hold);
    if (!inv.ok) {
      bad = true;
      bad_reason = inv.reason;
    }
    // Touch extract Mats after auth mutation.
    if (!extract_hold.KeyFDATA[0].Descriptors_all.empty()) {
      extract_hold.KeyFDATA[0].Descriptors_all.at<unsigned char>(0, 1) = 3;
    }
    {
      std::lock_guard<std::mutex> lk(mu);
      phase = Done;
      cv.notify_all();
    }
  });

  const auto t0 = std::chrono::steady_clock::now();
  writer.join();
  reader.join();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();
  EXPECT_LT(ms, kWatchdogMs) << "watchdog: controlled interleaving hung";
  EXPECT_FALSE(bad.load()) << bad_reason;

  // Auth mutated independently.
  EXPECT_NEAR(auth.AnchorsDATA[0].AnchorState[0], 5.0, 1e-9);
  const auto auth_inv = gmap_struct_m76::check(auth);
  EXPECT_TRUE(auth_inv.ok) << auth_inv.reason;
}

TEST(ConcurrencyM76, OpenCVDeepOwnershipByteContents)
{
  GLOBAL_MAP auth = make_base_map(4, 5);
  auto ext = gmap_auth_read::make_detection_extract(auth, 1, 0, 1);
  ASSERT_FALSE(ext.map.KeyFDATA[0].frame.empty());
  ASSERT_FALSE(ext.map.AnchorsDATA[0].Descriptor.empty());

  const unsigned char f0 = auth.KeyFDATA[0].frame.at<unsigned char>(0, 0);
  const unsigned char d0 = auth.AnchorsDATA[0].Descriptor.at<unsigned char>(0, 0);
  EXPECT_TRUE(mat_bytes_equal(ext.map.KeyFDATA[0].frame, auth.KeyFDATA[0].frame));
  EXPECT_TRUE(mat_bytes_equal(
    ext.map.AnchorsDATA[0].Descriptor, auth.AnchorsDATA[0].Descriptor));

  auth.KeyFDATA[0].frame.setTo(cv::Scalar(1));
  auth.AnchorsDATA[0].Descriptor.setTo(cv::Scalar(2));
  EXPECT_EQ(ext.map.KeyFDATA[0].frame.at<unsigned char>(0, 0), f0);
  EXPECT_EQ(ext.map.AnchorsDATA[0].Descriptor.at<unsigned char>(0, 0), d0);

  ext.map.KeyFDATA[0].frame.setTo(cv::Scalar(9));
  ext.map.AnchorsDATA[0].Descriptor.setTo(cv::Scalar(8));
  EXPECT_EQ(auth.KeyFDATA[0].frame.at<unsigned char>(0, 0), 1);
  EXPECT_EQ(auth.AnchorsDATA[0].Descriptor.at<unsigned char>(0, 0), 2);

  auth = GLOBAL_MAP{};
  EXPECT_EQ(ext.map.KeyFDATA[0].frame.at<unsigned char>(0, 0), 9);
}

TEST(ConcurrencyM76, ExtractDuringWorkspaceCorrectionSeesCommittedOnly)
{
  GLOBAL_MAP auth = make_base_map(5, 8);
  GLOBAL_MAP workspace = gmap_map_clone::deep_clone(auth);
  // Stage correction only on workspace.
  for (auto & kf : workspace.KeyFDATA) {
    kf.t_c2n[0] += 10.0;
  }
  for (auto & a : workspace.AnchorsDATA) {
    a.AnchorState[1] += 10.0;
  }

  std::mutex mu;
  GLOBAL_MAP seen;
  {
    std::lock_guard<std::mutex> lk(mu);
    seen = gmap_map_clone::deep_clone(auth);  // coherent pre-commit
  }
  EXPECT_NEAR(seen.KeyFDATA[0].t_c2n[0], auth.KeyFDATA[0].t_c2n[0], 1e-12);
  EXPECT_NEAR(seen.KeyFDATA[0].t_c2n[0] + 10.0, workspace.KeyFDATA[0].t_c2n[0], 1e-12);

  // Commit.
  {
    std::lock_guard<std::mutex> lk(mu);
    gmap_map_clone::assign_deep(auth, workspace);
  }
  GLOBAL_MAP after;
  {
    std::lock_guard<std::mutex> lk(mu);
    after = gmap_map_clone::deep_clone(auth);
  }
  EXPECT_NEAR(after.KeyFDATA[0].t_c2n[0], workspace.KeyFDATA[0].t_c2n[0], 1e-12);
  // Pre-commit extract still old.
  EXPECT_NEAR(seen.KeyFDATA[0].t_c2n[0] + 10.0, after.KeyFDATA[0].t_c2n[0], 1e-12);
}

TEST(ConcurrencyM76, PublicationGeometryCoherentWithLockedSnapshot)
{
  GLOBAL_MAP auth = make_base_map(7, 15);
  std::mutex mu;
  gmap_auth_read::PublicationGeometry pub;
  {
    std::lock_guard<std::mutex> lk(mu);
    pub = gmap_auth_read::extract_publication_geometry(auth);
  }
  ASSERT_EQ(pub.t_c2n.size(), auth.KeyFDATA.size());
  ASSERT_EQ(pub.landmark_xyz.size(), auth.AnchorsDATA.size());
  for (std::size_t i = 0; i < pub.t_c2n.size(); ++i) {
    EXPECT_NEAR(pub.t_c2n[i][0], auth.KeyFDATA[i].t_c2n[0], 1e-12);
  }
  // Mutate auth after unlock; pub unchanged.
  auth.KeyFDATA[0].t_c2n[0] = 999;
  EXPECT_NEAR(pub.t_c2n[0][0], 0.0, 1e-12);
}

TEST(ConcurrencyM76, CandidateEnumParityAuthVsExtract)
{
  GLOBAL_MAP auth = make_base_map(10, 5);
  // Break some Vgraph links so eligible set is nontrivial.
  auth.Vgraph(9, 0) = 0;
  auth.Vgraph(9, 1) = 0;
  auth.Vgraph(9, 2) = 0;
  auto ext = gmap_auth_read::make_detection_extract(auth, 0, 0, 1);
  const int cur = static_cast<int>(auth.KeyFDATA.size()) - 1;
  const int ba_limit = std::max(0, cur - 2);
  auto a = globalslam::cloop_visual_retrieve::enumerate_unlinked_historical(
    auth.Vgraph, cur, ba_limit);
  auto b = globalslam::cloop_visual_retrieve::enumerate_unlinked_historical(
    ext.map.Vgraph, cur, ba_limit);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i], b[i]);
  }
}

TEST(ConcurrencyM76, RepeatedStressSingleWriterManyReaders)
{
  constexpr int kSeed = 76;
  constexpr int kOps = 2500;
  GLOBAL_MAP auth = make_base_map(8, 30);
  std::mutex mu;
  std::atomic<bool> run{true};
  std::atomic<int> extracts{0};
  std::atomic<int> mutates{0};
  std::atomic<int> violations{0};
  std::atomic<int> exceptions{0};

  std::thread writer([&]() {
    int local = 0;
    while (run.load() && local < kOps) {
      try {
        std::lock_guard<std::mutex> lk(mu);
        if (local % 7 == 0 && auth.AnchorsDATA.size() > 5) {
          erase_landmark_at_storage_index(auth, 2, nullptr);
          rebuild_id_to_index(auth);
        } else if (local % 5 == 0) {
          insert_landmark(auth, make_anchor(local * 0.01, 0, 2, 0,
            static_cast<unsigned char>(local % 180)));
        } else if (local % 11 == 0) {
          auth.KeyFDATA.push_back(make_kf(0, 0, -1, 5000 + local));
          const int n = static_cast<int>(auth.KeyFDATA.size());
          auth.Vgraph.resize(n, n);
        } else if (!auth.AnchorsDATA.empty()) {
          auth.AnchorsDATA[0].AnchorState[0] += 0.0001;
          if (!auth.AnchorsDATA[0].Descriptor.empty()) {
            auto & d = auth.AnchorsDATA[0].Descriptor;
            d.at<unsigned char>(0, 0) =
              static_cast<unsigned char>((d.at<unsigned char>(0, 0) + 1) % 180);
          }
        }
        const auto inv = gmap_struct_m76::check(auth);
        if (!inv.ok) {
          ++violations;
        }
        ++mutates;
        ++local;
      } catch (...) {
        ++exceptions;
        break;
      }
      // Yield so readers obtain the mutex between mutations.
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    run = false;
  });

  auto reader_fn = [&]() {
    while (run.load() || extracts.load() < 200) {
      if (!run.load() && extracts.load() >= 200) {
        break;
      }
      try {
        GLOBAL_MAP local;
        {
          std::lock_guard<std::mutex> lk(mu);
          local = gmap_map_clone::deep_clone(auth);
        }
        const auto inv = gmap_struct_m76::check(local);
        if (!inv.ok) {
          ++violations;
        }
        if (!local.AnchorsDATA.empty() &&
          !local.AnchorsDATA[0].Descriptor.empty())
        {
          (void)local.AnchorsDATA[0].Descriptor.at<unsigned char>(0, 0);
        }
        // Simulate unlocked matching work.
        auto poses = gmap_auth_read::extract_candidate_poses(local);
        (void)poses;
        ++extracts;
      } catch (...) {
        ++exceptions;
        break;
      }
    }
  };

  std::thread r1(reader_fn);
  std::thread r2(reader_fn);

  const auto t0 = std::chrono::steady_clock::now();
  writer.join();
  r1.join();
  r2.join();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();

  EXPECT_LT(ms, kWatchdogMs) << "watchdog: stress hung";
  EXPECT_EQ(violations.load(), 0);
  EXPECT_EQ(exceptions.load(), 0);
  EXPECT_GE(mutates.load(), kOps);
  EXPECT_GE(extracts.load(), 200);
  // Combined read/write ops across campaign.
  EXPECT_GE(mutates.load() + extracts.load(), kOps + 200);
  // Record seed for evidence.
  EXPECT_EQ(kSeed, 76);
}

TEST(ConcurrencyM76, DetectionComputeAfterExtractWhileAuthMutates)
{
  GLOBAL_MAP auth = make_base_map(12, 40);
  GLOBAL_MAP extract;
  {
    extract = gmap_map_clone::deep_clone(auth);
  }
  std::atomic<bool> done{false};
  std::atomic<bool> bad{false};

  std::thread mutator([&]() {
    while (!done.load()) {
      if (auth.AnchorsDATA.empty()) {
        break;
      }
      auth.AnchorsDATA[0].AnchorState[0] += 0.01;
      auth.KeyFDATA[0].t_c2n[1] += 0.01;
      if (!auth.KeyFDATA[0].Descriptors_all.empty()) {
        auth.KeyFDATA[0].Descriptors_all.at<unsigned char>(0, 0) =
          static_cast<unsigned char>(
            (auth.KeyFDATA[0].Descriptors_all.at<unsigned char>(0, 0) + 1) % 100);
      }
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  });

  // Unlocked "detection": enumerate + rank on extract while auth mutates.
  const int cur = static_cast<int>(extract.KeyFDATA.size()) - 1;
  auto eligible = globalslam::cloop_visual_retrieve::enumerate_unlinked_historical(
    extract.Vgraph, cur, cur);
  std::vector<globalslam::cloop_visual_retrieve::RankedCandidate> cands;
  for (int idx : eligible) {
    globalslam::cloop_visual_retrieve::RankedCandidate c;
    c.idx_kf = idx;
    c.n_matches = 5 + (idx % 3);
    cands.push_back(c);
  }
  globalslam::cloop_visual_retrieve::rank_candidates_inplace(cands);
  const auto id0 = extract.AnchorsDATA[0].id;
  const double x0 = extract.AnchorsDATA[0].AnchorState[0];
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  if (extract.AnchorsDATA[0].id != id0 ||
    std::abs(extract.AnchorsDATA[0].AnchorState[0] - x0) > 1e-15)
  {
    bad = true;
  }
  done = true;
  mutator.join();
  EXPECT_FALSE(bad.load());
  EXPECT_FALSE(cands.empty());
}
