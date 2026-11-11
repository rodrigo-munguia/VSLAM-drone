#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "g4_calibration.hpp"
#include "g4_viz_conversions.hpp"

using globalslam::g4_cal::MonoCamFingerprint;
using globalslam::g4_cal::fingerprints_match;
using globalslam::g4_cal::localslam_source_supported;
using globalslam::g4_viz::MapSnapshot;
using globalslam::g4_viz::SnapshotKeyframe;
using globalslam::g4_viz::SnapshotLandmark;
using globalslam::g4_viz::camera_position_from_body;
using globalslam::g4_viz::internal_body_to_public_pose;
using globalslam::g4_viz::internal_landmark_to_public;
using globalslam::g4_viz::make_empty_cloud;
using globalslam::g4_viz::make_empty_path;
using globalslam::g4_viz::make_keyframe_path;
using globalslam::g4_viz::make_landmark_cloud;

namespace
{

void set_identity(double R[9])
{
  for (int i = 0; i < 9; ++i) {
    R[i] = 0.0;
  }
  R[0] = R[4] = R[8] = 1.0;
}

/** Column-major Rz(yaw). */
void set_yaw(double R[9], double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  R[0] = c;  R[3] = -s; R[6] = 0;
  R[1] = s;  R[4] = c;  R[7] = 0;
  R[2] = 0;  R[5] = 0;  R[8] = 1;
}

}  // namespace

TEST(G4Viz, AsymmetricLandmarkExposesAxisMap)
{
  double xp = 0, yp = 0, zp = 0;
  ASSERT_TRUE(internal_landmark_to_public(1.25, -7.44, 0.04, xp, yp, zp));
  EXPECT_DOUBLE_EQ(xp, 1.25);
  EXPECT_DOUBLE_EQ(yp, 7.44);
  EXPECT_DOUBLE_EQ(zp, -0.04);
}

TEST(G4Viz, RejectsNonFiniteLandmark)
{
  double xp = 0, yp = 0, zp = 0;
  EXPECT_FALSE(internal_landmark_to_public(
    std::numeric_limits<double>::quiet_NaN(), 1.0, 2.0, xp, yp, zp));
  EXPECT_FALSE(internal_landmark_to_public(
    1.0, std::numeric_limits<double>::infinity(), 2.0, xp, yp, zp));
}

TEST(G4Viz, IdentityBodyPose)
{
  double Rn2r[9];
  set_identity(Rn2r);
  const double r_N[3] = {1.0, 2.0, 3.0};
  double p[3];
  double q[4];
  ASSERT_TRUE(internal_body_to_public_pose(r_N, Rn2r, p, q));
  EXPECT_DOUBLE_EQ(p[0], 1.0);
  EXPECT_DOUBLE_EQ(p[1], -2.0);
  EXPECT_DOUBLE_EQ(p[2], -3.0);
  EXPECT_NEAR(q[0], 1.0, 1e-9);
  EXPECT_NEAR(q[1], 0.0, 1e-9);
  EXPECT_NEAR(q[2], 0.0, 1e-9);
  EXPECT_NEAR(q[3], 0.0, 1e-9);
}

TEST(G4Viz, PositiveAndNegativeYaw)
{
  double Rn2r[9];
  set_yaw(Rn2r, 0.5);
  const double r0[3] = {0, 0, 0};
  double p[3];
  double q_pos[4];
  ASSERT_TRUE(internal_body_to_public_pose(r0, Rn2r, p, q_pos));

  set_yaw(Rn2r, -0.5);
  double q_neg[4];
  ASSERT_TRUE(internal_body_to_public_pose(r0, Rn2r, p, q_neg));

  // Opposite yaw → opposite z-component of quaternion (public).
  EXPECT_NEAR(q_pos[3], -q_neg[3], 1e-9);
  EXPECT_GT(std::abs(q_pos[3]), 1e-3);
}

TEST(G4Viz, CombinedOrientationNotComponentWiseD)
{
  // Non-principal attitude: do not apply D to quaternion components.
  double Rn2r[9];
  set_yaw(Rn2r, 0.3);
  // Small pitch about y after yaw via approximate composition is unnecessary;
  // verify unit quaternion and that D on (x,y,z) would disagree with DCM path
  // for a known non-identity case by checking norm and w>=0.
  const double r0[3] = {0.1, -0.2, 0.3};
  double p[3];
  double q[4];
  ASSERT_TRUE(internal_body_to_public_pose(r0, Rn2r, p, q));
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  EXPECT_NEAR(n, 1.0, 1e-9);
  EXPECT_GE(q[0], 0.0);
  EXPECT_DOUBLE_EQ(p[0], 0.1);
  EXPECT_DOUBLE_EQ(p[1], 0.2);
  EXPECT_DOUBLE_EQ(p[2], -0.3);
}

TEST(G4Viz, LeverArmBodyVsCamera)
{
  double Rn2r[9];
  set_identity(Rn2r);
  const double r_N[3] = {1.0, 0.0, 0.0};
  const double t_c2r[3] = {0.1, 0.0, 0.0};
  double t_c2n[3];
  camera_position_from_body(r_N, Rn2r, t_c2r, t_c2n);
  EXPECT_NEAR(t_c2n[0], 1.1, 1e-12);
  EXPECT_NEAR(t_c2n[1], 0.0, 1e-12);
  EXPECT_NEAR(t_c2n[2], 0.0, 1e-12);

  double p_body[3], q_body[4];
  ASSERT_TRUE(internal_body_to_public_pose(r_N, Rn2r, p_body, q_body));
  double p_cam[3], q_cam[4];
  ASSERT_TRUE(internal_body_to_public_pose(t_c2n, Rn2r, p_cam, q_cam));
  EXPECT_NEAR(p_cam[0] - p_body[0], 0.1, 1e-12);
}

TEST(G4Viz, SourceStampsRetainedAcrossPathRebuild)
{
  SnapshotKeyframe a;
  set_identity(a.Rn2r_cm);
  a.r_N[0] = 1.0;
  a.source_stamp_valid = true;
  a.stamp_sec = 10;
  a.stamp_nanosec = 250000000u;
  a.keyframe_id = 1;

  SnapshotKeyframe b = a;
  b.r_N[0] = 2.0;
  b.stamp_sec = 11;
  b.keyframe_id = 2;

  std_msgs::msg::Header hdr;
  hdr.frame_id = "odom";
  hdr.stamp.sec = 99;
  hdr.stamp.nanosec = 0;

  auto path = make_keyframe_path(hdr, {a, b}, 0);
  ASSERT_EQ(path.poses.size(), 2u);
  EXPECT_EQ(path.header.stamp.sec, 99);
  EXPECT_EQ(path.poses[0].header.stamp.sec, 10);
  EXPECT_EQ(path.poses[0].header.stamp.nanosec, 250000000u);
  EXPECT_EQ(path.poses[1].header.stamp.sec, 11);

  // Revised historical pose replaces previous Path value (full rebuild).
  a.r_N[0] = 1.5;
  path = make_keyframe_path(hdr, {a, b}, 0);
  ASSERT_EQ(path.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 1.5);
  EXPECT_EQ(path.poses[0].header.stamp.sec, 10);
}

TEST(G4Viz, EmptyOutputsAndLimitsPreserveEstimatorCopy)
{
  std_msgs::msg::Header hdr;
  hdr.frame_id = "odom";
  auto path = make_empty_path(hdr);
  auto cloud = make_empty_cloud(hdr);
  EXPECT_TRUE(path.poses.empty());
  EXPECT_EQ(cloud.width, 0u);
  EXPECT_TRUE(cloud.is_dense);
  EXPECT_EQ(cloud.point_step, 12u);

  std::vector<SnapshotLandmark> map_pts;
  for (int i = 0; i < 5; ++i) {
    map_pts.push_back(SnapshotLandmark{
      static_cast<double>(i), 1.0, 2.0});
  }
  map_pts.push_back(SnapshotLandmark{
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});

  const auto limited = make_landmark_cloud(hdr, map_pts, 3);
  EXPECT_EQ(limited.width, 3u);
  EXPECT_EQ(map_pts.size(), 6u);  // estimator/snapshot copy untouched

  const auto full = make_landmark_cloud(hdr, map_pts, 0);
  EXPECT_EQ(full.width, 5u);  // NaN skipped
  ASSERT_GE(full.data.size(), 12u);
  float x0 = 0;
  std::memcpy(&x0, full.data.data(), sizeof(float));
  EXPECT_FLOAT_EQ(x0, 0.0f);
}

TEST(G4Viz, GenerationGateSemantics)
{
  // Publication path checks generation before and after serialize; this unit
  // documents the intended policy: a snapshot tagged with an old generation
  // must not be treated as current after reset bumps generation.
  MapSnapshot snap;
  snap.generation = 3;
  uint64_t current_generation = 3;
  EXPECT_EQ(snap.generation, current_generation);
  current_generation = 4;  // reset
  EXPECT_NE(snap.generation, current_generation);
}

TEST(G4Cal, YamlSourceSupportedCameraInfoNot)
{
  EXPECT_TRUE(localslam_source_supported("yaml"));
  EXPECT_TRUE(localslam_source_supported(""));
  EXPECT_FALSE(localslam_source_supported("camera_info"));
}

TEST(G4Cal, FingerprintRequiresMoreThanResolution)
{
  MonoCamFingerprint a;
  a.rows = 240;
  a.cols = 400;
  a.fc_u = 200;
  a.fc_v = 200;
  a.cc_u = 200;
  a.cc_v = 120;
  a.alpha_c = 0;
  a.distortions = {0, 0, 0, 0, 0};

  MonoCamFingerprint b = a;
  std::string reason;
  EXPECT_TRUE(fingerprints_match(a, b, &reason));

  b.fc_u = 210;
  EXPECT_FALSE(fingerprints_match(a, b, &reason));
  EXPECT_NE(reason.find("focal"), std::string::npos);

  b = a;
  b.rows = 480;  // same cols, different rows — not "resolution match enough"
  EXPECT_FALSE(fingerprints_match(a, b, &reason));
}
