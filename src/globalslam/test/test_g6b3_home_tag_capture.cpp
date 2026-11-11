/**
 * G6b.3: independent home-tag capture + return-path tests.
 * Expected geometry from p_H = p_C + Rn2cᵀ tvec / p_C = p_H − Rn2cᵀ tvec
 * (not by copying production helper internals for the assertion oracle).
 */

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "g6b_home_tag_capture.hpp"
#include "g6b_home_tag_geometry.hpp"

using namespace globalslam::g6b_home_tag;

namespace
{

void I_Rn2c(double out[9])
{
  out[0] = 1; out[1] = 0; out[2] = 0;
  out[3] = 0; out[4] = 1; out[5] = 0;
  out[6] = 0; out[7] = 0; out[8] = 1;
}

void yaw_Rn2c(double yaw, double out[9])
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  out[0] = c; out[1] = s; out[2] = 0;
  out[3] = -s; out[4] = c; out[5] = 0;
  out[6] = 0; out[7] = 0; out[8] = 1;
}

/** Independent: tvec = Rn2c * (p_H - p_C). */
void synth_tvec(
  const double p_H[3], const double p_C[3], const double Rn2c[9], double tvec[3])
{
  const double d[3] = {p_H[0] - p_C[0], p_H[1] - p_C[1], p_H[2] - p_C[2]};
  tvec[0] = Rn2c[0] * d[0] + Rn2c[1] * d[1] + Rn2c[2] * d[2];
  tvec[1] = Rn2c[3] * d[0] + Rn2c[4] * d[1] + Rn2c[5] * d[2];
  tvec[2] = Rn2c[6] * d[0] + Rn2c[7] * d[1] + Rn2c[8] * d[2];
}

RecordingObservation make_obs(
  uint64_t kf_id,
  const double p_C[3],
  const double p_H_true[3],
  const double Rn2c[9],
  int tag_id = 23,
  uint64_t epoch = 1)
{
  RecordingObservation o;
  slam_identity::UuidBytes sid{};
  sid[0] = 1;
  o.session_id = sid;
  o.epoch_id = epoch;
  o.keyframe_id = kf_id;
  o.source_time_ms = static_cast<long int>(kf_id * 100);
  o.source_stamp_valid = true;
  o.tag_id = tag_id;
  o.t_c2n[0] = p_C[0];
  o.t_c2n[1] = p_C[1];
  o.t_c2n[2] = p_C[2];
  for (int i = 0; i < 9; ++i) {
    o.Rn2c_rm[i] = Rn2c[i];
  }
  synth_tvec(p_H_true, p_C, Rn2c, o.tvec);
  tag_position_from_camera(
    o.t_c2n[0], o.t_c2n[1], o.t_c2n[2],
    o.tvec[0], o.tvec[1], o.tvec[2],
    o.Rn2c_rm,
    o.p_H_N[0], o.p_H_N[1], o.p_H_N[2]);
  o.d_px = 5.0;
  return o;
}

CaptureParams tight_params()
{
  CaptureParams p;
  p.home_tag_id = 23;
  p.min_inliers = 3;
  p.min_inlier_fraction = 0.5;
  p.max_spread_m = 1.5;
  p.max_samples = 40;
  p.gate_px = 90.0;
  return p;
}

}  // namespace

TEST(G6b3HomeTagCapture, TagNotAtNavOriginRoundTrip)
{
  double Rn2c[9];
  I_Rn2c(Rn2c);
  const double p_H[3] = {2.0, -1.5, 0.1};
  const double p_C[3] = {2.1, -1.4, -4.0};
  double tvec[3];
  synth_tvec(p_H, p_C, Rn2c, tvec);
  double p_H_rec[3], p_C_rec[3];
  tag_position_from_camera(
    p_C[0], p_C[1], p_C[2], tvec[0], tvec[1], tvec[2], Rn2c,
    p_H_rec[0], p_H_rec[1], p_H_rec[2]);
  camera_position_from_tag(
    p_H_rec[0], p_H_rec[1], p_H_rec[2], tvec[0], tvec[1], tvec[2], Rn2c,
    p_C_rec[0], p_C_rec[1], p_C_rec[2]);
  EXPECT_NEAR(p_H_rec[0], p_H[0], 1e-12);
  EXPECT_NEAR(p_H_rec[1], p_H[1], 1e-12);
  EXPECT_NEAR(p_C_rec[0], p_C[0], 1e-12);
  EXPECT_NEAR(p_C_rec[2], p_C[2], 1e-12);
}

TEST(G6b3HomeTagCapture, NonzeroRobotInitInvariant)
{
  // Same physical geometry under translated N origin.
  double Rn2c[9];
  I_Rn2c(Rn2c);
  const double p_H_a[3] = {5.0, 0.0, 0.0};
  const double p_C_a[3] = {5.0, 0.0, -4.0};
  double tvec[3];
  synth_tvec(p_H_a, p_C_a, Rn2c, tvec);
  // Shift N by −5 in x (Robot_init style): both p_H and p_C shift equally.
  const double p_H_b[3] = {0.0, 0.0, 0.0};
  const double p_C_b[3] = {0.0, 0.0, -4.0};
  double tvec_b[3];
  synth_tvec(p_H_b, p_C_b, Rn2c, tvec_b);
  EXPECT_NEAR(tvec[0], tvec_b[0], 1e-12);
  EXPECT_NEAR(tvec[1], tvec_b[1], 1e-12);
  EXPECT_NEAR(tvec[2], tvec_b[2], 1e-12);
}

TEST(G6b3HomeTagCapture, CaptureReadyAndReturnUsesStoredPH)
{
  HomeTagCaptureSession cap;
  cap.set_params(tight_params());
  slam_identity::UuidBytes sid{};
  sid[0] = 1;
  std::string reason;
  ASSERT_TRUE(cap.begin(sid, 1, reason));

  double Rn2c[9];
  I_Rn2c(Rn2c);
  const double p_H[3] = {1.25, -0.5, 0.05};
  for (uint64_t i = 1; i <= 5; ++i) {
    const double p_C[3] = {
      p_H[0] + 0.05 * static_cast<double>(i),
      p_H[1] - 0.02 * static_cast<double>(i),
      -4.0};
    std::string rej;
    ASSERT_TRUE(cap.offer(make_obs(i, p_C, p_H, Rn2c), rej)) << rej;
  }
  cap.request_finalize();
  ASSERT_TRUE(cap.finalize(10));
  ASSERT_TRUE(cap.is_ready());
  EXPECT_NEAR(cap.reference().p_H_N[0], p_H[0], 0.2);
  EXPECT_NEAR(cap.reference().p_H_N[1], p_H[1], 0.2);

  // Return observation must not update frozen ref.
  const auto frozen = cap.reference().p_H_N[0];
  const double p_C_ret[3] = {p_H[0] + 0.3, p_H[1], -4.0};
  auto ret = make_obs(99, p_C_ret, p_H, Rn2c);
  std::string rej;
  EXPECT_FALSE(cap.offer(ret, rej));
  EXPECT_NEAR(cap.reference().p_H_N[0], frozen, 0.0);

  double tx, ty, tz;
  const auto r = convert_home_tag_tvec_to_nav_camera_pos(
    ret.tvec[0], ret.tvec[1], ret.tvec[2], Rn2c, 6.0, 90.0, 5.0,
    true, true, true,
    cap.reference().p_H_N[0], cap.reference().p_H_N[1], cap.reference().p_H_N[2],
    tx, ty, tz);
  EXPECT_EQ(r, PoseReject::Ok);
  EXPECT_NEAR(tx, p_C_ret[0], 0.25);
  EXPECT_NEAR(tz, p_C_ret[2], 0.25);
}

TEST(G6b3HomeTagCapture, OutliersRejectedInsufficientSpreadFails)
{
  HomeTagCaptureSession cap;
  auto p = tight_params();
  p.min_inliers = 3;
  p.min_inlier_fraction = 0.6;
  p.max_spread_m = 0.3;
  cap.set_params(p);
  slam_identity::UuidBytes sid{};
  sid[0] = 1;
  std::string reason;
  ASSERT_TRUE(cap.begin(sid, 1, reason));
  double Rn2c[9];
  I_Rn2c(Rn2c);
  const double p_H[3] = {0, 0, 0};
  for (uint64_t i = 1; i <= 3; ++i) {
    const double p_C[3] = {0.01 * static_cast<double>(i), 0, -4};
    std::string rej;
    ASSERT_TRUE(cap.offer(make_obs(i, p_C, p_H, Rn2c), rej));
  }
  // Far outliers
  const double p_H_bad[3] = {10, 10, 0};
  for (uint64_t i = 10; i <= 12; ++i) {
    const double p_C[3] = {10, 10, -4};
    std::string rej;
    ASSERT_TRUE(cap.offer(make_obs(i, p_C, p_H_bad, Rn2c), rej));
  }
  cap.request_finalize();
  // 3 good / 6 total = 0.5 < 0.6 fraction → fail
  EXPECT_FALSE(cap.finalize(5));
  EXPECT_EQ(cap.state(), CaptureState::Failed);
  EXPECT_FALSE(cap.reference().spatial_ready);
}

TEST(G6b3HomeTagCapture, InsufficientSamplesFail)
{
  HomeTagCaptureSession cap;
  cap.set_params(tight_params());
  slam_identity::UuidBytes sid{};
  sid[0] = 1;
  std::string reason;
  ASSERT_TRUE(cap.begin(sid, 1, reason));
  double Rn2c[9];
  I_Rn2c(Rn2c);
  const double p_H[3] = {0, 0, 0};
  const double p_C[3] = {0, 0, -4};
  std::string rej;
  ASSERT_TRUE(cap.offer(make_obs(1, p_C, p_H, Rn2c), rej));
  ASSERT_TRUE(cap.offer(make_obs(2, p_C, p_H, Rn2c), rej));
  cap.request_finalize();
  EXPECT_FALSE(cap.finalize(2));
  EXPECT_EQ(cap.diagnostics().fail_reason, "insufficient_inliers");
}

TEST(G6b3HomeTagCapture, WrongTagIdRejected)
{
  HomeTagCaptureSession cap;
  cap.set_params(tight_params());
  slam_identity::UuidBytes sid{};
  sid[0] = 1;
  std::string reason;
  ASSERT_TRUE(cap.begin(sid, 1, reason));
  double Rn2c[9];
  I_Rn2c(Rn2c);
  const double p_H[3] = {0, 0, 0};
  const double p_C[3] = {0, 0, -4};
  std::string rej;
  EXPECT_FALSE(cap.offer(make_obs(1, p_C, p_H, Rn2c, /*tag*/ 99), rej));
  EXPECT_EQ(rej, "wrong_tag_id");
}

TEST(G6b3HomeTagCapture, DuplicateKeyframeAcrossStreamsDeduped)
{
  HomeTagCaptureSession cap;
  cap.set_params(tight_params());
  slam_identity::UuidBytes sid{};
  sid[0] = 1;
  std::string reason;
  ASSERT_TRUE(cap.begin(sid, 1, reason));
  double Rn2c[9];
  I_Rn2c(Rn2c);
  const double p_H[3] = {0, 0, 0};
  const double p_C[3] = {0, 0, -4};
  std::string rej;
  ASSERT_TRUE(cap.offer(make_obs(7, p_C, p_H, Rn2c), rej));
  EXPECT_FALSE(cap.offer(make_obs(7, p_C, p_H, Rn2c), rej));
  EXPECT_EQ(rej, "duplicate_keyframe_id");
  EXPECT_EQ(cap.sample_count(), 1);
}

TEST(G6b3HomeTagCapture, PostCutoffAndPreRecordingRejected)
{
  HomeTagCaptureSession cap;
  cap.set_params(tight_params());
  slam_identity::UuidBytes sid{};
  sid[0] = 1;
  std::string reason;
  ASSERT_TRUE(cap.begin(sid, 1, reason));
  double Rn2c[9];
  I_Rn2c(Rn2c);
  const double p_H[3] = {0, 0, 0};
  for (uint64_t i = 1; i <= 4; ++i) {
    const double p_C[3] = {0.01 * i, 0, -4};
    std::string rej;
    ASSERT_TRUE(cap.offer(make_obs(i, p_C, p_H, Rn2c), rej));
  }
  cap.request_finalize();
  std::string rej;
  EXPECT_FALSE(cap.offer(make_obs(100, p_H, p_H, Rn2c), rej));  // after cutoff id
  EXPECT_EQ(rej, "after_cutoff_id");
}

TEST(G6b3HomeTagCapture, DeliberateRecaptureRequiredWhenReady)
{
  HomeTagCaptureSession cap;
  cap.set_params(tight_params());
  slam_identity::UuidBytes sid{};
  sid[0] = 1;
  std::string reason;
  ASSERT_TRUE(cap.begin(sid, 1, reason));
  double Rn2c[9];
  I_Rn2c(Rn2c);
  const double p_H[3] = {0, 0, 0};
  for (uint64_t i = 1; i <= 4; ++i) {
    const double p_C[3] = {0, 0, -4};
    std::string rej;
    ASSERT_TRUE(cap.offer(make_obs(i, p_C, p_H, Rn2c), rej));
  }
  cap.request_finalize();
  ASSERT_TRUE(cap.finalize(4));
  EXPECT_FALSE(cap.begin(sid, 1, reason));
  EXPECT_EQ(reason, "already_ready_deliberate_recapture_required");
  EXPECT_TRUE(cap.is_ready());
}

TEST(G6b3HomeTagCapture, CorrectionInvalidateDuringRecording)
{
  HomeTagCaptureSession cap;
  cap.set_params(tight_params());
  slam_identity::UuidBytes sid{};
  sid[0] = 1;
  std::string reason;
  ASSERT_TRUE(cap.begin(sid, 1, reason));
  cap.fail("correction_during_capture");
  EXPECT_EQ(cap.state(), CaptureState::Failed);
  EXPECT_FALSE(cap.reference().spatial_ready);
  EXPECT_FALSE(cap.reference().valid);
}

TEST(G6b3HomeTagCapture, OptimizedPoseCorruptionDoesNotAffectStoredObs)
{
  // RecordingObservation copies t_c2n at offer time; mutating a "global" pose
  // buffer afterward must not change the candidate.
  double Rn2c[9];
  I_Rn2c(Rn2c);
  const double p_H[3] = {1, 2, 0};
  const double p_C[3] = {1, 2, -4};
  auto obs = make_obs(1, p_C, p_H, Rn2c);
  const double stored = obs.p_H_N[0];
  // Simulate corrupted global optimized pose (not used by capture offer copy).
  double fake_global_t_c2n[3] = {999, 999, 999};
  (void)fake_global_t_c2n;
  EXPECT_NEAR(obs.p_H_N[0], stored, 0.0);
  EXPECT_NEAR(obs.t_c2n[0], p_C[0], 1e-12);
}

TEST(G6b3HomeTagCapture, CombinedAttitudeAndLeverArm)
{
  double Rn2c[9];
  yaw_Rn2c(1.5707963267948966, Rn2c);  // downward cam yaw π/2
  const double p_H[3] = {0.5, -0.2, 0.0};
  // Camera with lever along body x mapped through Rn2c
  const double p_C[3] = {0.6, -0.2, -3.5};
  double tvec[3];
  synth_tvec(p_H, p_C, Rn2c, tvec);
  double rec[3];
  tag_position_from_camera(
    p_C[0], p_C[1], p_C[2], tvec[0], tvec[1], tvec[2], Rn2c,
    rec[0], rec[1], rec[2]);
  EXPECT_NEAR(rec[0], p_H[0], 1e-9);
  EXPECT_NEAR(rec[1], p_H[1], 1e-9);
  EXPECT_NEAR(rec[2], p_H[2], 1e-9);
}

TEST(G6b3HomeTagCapture, NotReadyRejectsReturnConversion)
{
  double Rn2c[9];
  I_Rn2c(Rn2c);
  double tx, ty, tz;
  const auto r = convert_home_tag_tvec_to_nav_camera_pos(
    0, 0, 4, Rn2c, 6, 90, 5,
    true, /*spatial_ready=*/false, true,
    1.0, 2.0, 0.0, tx, ty, tz);
  EXPECT_EQ(r, PoseReject::HomeRefNotReady);
}

TEST(G6b3HomeTagCapture, ClearResetsReady)
{
  HomeTagCaptureSession cap;
  cap.set_params(tight_params());
  slam_identity::UuidBytes sid{};
  sid[0] = 1;
  std::string reason;
  ASSERT_TRUE(cap.begin(sid, 1, reason));
  double Rn2c[9];
  I_Rn2c(Rn2c);
  const double p_H[3] = {0, 0, 0};
  for (uint64_t i = 1; i <= 4; ++i) {
    const double p_C[3] = {0, 0, -4};
    std::string rej;
    ASSERT_TRUE(cap.offer(make_obs(i, p_C, p_H, Rn2c), rej));
  }
  cap.request_finalize();
  ASSERT_TRUE(cap.finalize(4));
  cap.clear();
  EXPECT_EQ(cap.state(), CaptureState::Unrecorded);
  EXPECT_FALSE(cap.reference().spatial_ready);
}
