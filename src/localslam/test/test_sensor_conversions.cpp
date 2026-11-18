#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include "ekf_run_control.hpp"
#include "range_frame_association.hpp"
#include "sensor_conversions.hpp"
#include "sensor_input_plan.hpp"

using localslam::EKFRunControl;
using localslam::SensorInputPlan;
using localslam::associate_depth_to_frame;
using localslam::feature_in_range_confidence_region;
using localslam::initial_depth_sigma_m;
using localslam::kAltFallbackDtMs;
using localslam::kFirstLandmarkDepthSigmaScale;
using localslam::kRangeGateMaxM;
using localslam::kRangeGateMinM;
using localslam::kRangeInitMinM;
using localslam::make_sensor_input_plan;
using localslam::note_altitude_for_fallback;
using localslam::note_downward_range;
using localslam::range_passes_assoc_gate;
using localslam::range_permits_undelayed_init;
using localslam::validate_altitude_input_mode;
using localslam::validate_range_acceptance_params;
using localslam::conversions::body_twist_to_internal_spd;
using localslam::conversions::imu_orientation_unavailable;
using localslam::conversions::imu_to_internal_att;
using localslam::conversions::mavros_relative_to_internal_alt;
using localslam::conversions::quaternion_usable;
using localslam::conversions::range_msg_to_internal_range;

// ---------------------------------------------------------------------------
// Attitude frame contract via rotation matrices (not RPY sign-flip alone).
//
// Source (ROS / Webots IMU, REP-145): orientation quaternion → R_F←B =
//   body axes expressed in a right-handed z-up fixed frame (tf2 Matrix3x3).
// Destination (estimator ATT + Euler_to_Ra2b): same physical attitude after
//   the axis map D = diag(1,-1,-1) applied to both fixed and body bases
//   (x unchanged; y and z reversed — matches webotbridge GT y/z pattern).
// Under that map, the world←body DCM stored by Euler_to_Ra2b must equal
//   D * R_F←B^T * D. Converter RPY is only checked as the matrix witness.
// ---------------------------------------------------------------------------
static void expected_internal_dcm_from_ros_quat(
  const tf2::Quaternion & q_ros, double R_expected_col_major[9])
{
  const tf2::Matrix3x3 R_F_from_B(q_ros);  // source: fixed ← body
  constexpr double D[3] = {1.0, -1.0, -1.0};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      // (D * R_F_from_B^T * D)[r,c] = D[r] * R_F_from_B[c][r] * D[c]
      R_expected_col_major[c * 3 + r] =
        D[r] * R_F_from_B[c][r] * D[c];
    }
  }
}

TEST(FrameContract, CombinedNonzeroOrientationMatrixMap)
{
  const double roll_ros = 0.12;
  const double pitch_ros = -0.34;
  const double yaw_ros = 0.56;

  tf2::Quaternion q;
  q.setRPY(roll_ros, pitch_ros, yaw_ros);

  double R_expected[9];
  expected_internal_dcm_from_ros_quat(q, R_expected);

  sensor_msgs::msg::Imu msg;
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  msg.orientation_covariance[0] = 0.0;

  localslam::conversions::AttObs att;
  ASSERT_TRUE(imu_to_internal_att(msg, att));

  double R_from_converter[9];
  Euler_to_Ra2b_colum_major(
    static_cast<double>(att.roll),
    static_cast<double>(att.pitch),
    static_cast<double>(att.yaw),
    R_from_converter);

  // AttObs stores RPY as float; tolerate float32 round-trip into Euler_to_Ra2b.
  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(R_from_converter[i], R_expected[i], 1e-5)
      << "DCM mismatch at col-major index " << i
      << " (source z-up Imu vs D=diag(1,-1,-1) destination)";
  }
}

TEST(FrameContract, BodyVelocityAllAxesAndIgnoreCov)
{
  // Public ROS body (after Rn2r * gps_speed, before Spd flips):
  // internal = (vx, -vy, -vz)
  geometry_msgs::msg::TwistWithCovarianceStamped msg;
  msg.header.frame_id = "base_link";
  msg.header.stamp.sec = 1;
  msg.header.stamp.nanosec = 0;
  msg.twist.twist.linear.x = 1.5;
  msg.twist.twist.linear.y = -2.5;
  msg.twist.twist.linear.z = 3.5;
  msg.twist.twist.angular.z = 99.0;
  msg.twist.covariance[0] = 123.0;

  localslam::conversions::SpdObs spd;
  ASSERT_TRUE(body_twist_to_internal_spd(msg, spd));
  EXPECT_FLOAT_EQ(spd.speedX, 1.5f);
  EXPECT_FLOAT_EQ(spd.speedY, 2.5f);
  EXPECT_FLOAT_EQ(spd.speedZ, -3.5f);
}

TEST(FrameContract, AltitudePositiveUpStoredOnce)
{
  mavros_msgs::msg::Altitude msg;
  msg.relative = 4.25f;
  msg.amsl = 999.0f;
  msg.local = -1.0f;
  msg.terrain = 0.5f;
  localslam::conversions::AltObs alt;
  ASSERT_TRUE(mavros_relative_to_internal_alt(msg, alt));
  EXPECT_DOUBLE_EQ(alt.altitude, 4.25);
  // Estimator applies Alt_h_2_d (-1) once inside Altitude_update:
  EXPECT_DOUBLE_EQ(alt.altitude * (-1), -4.25);
}

TEST(SensorConversions, RejectUnavailableAndBadImu)
{
  sensor_msgs::msg::Imu msg;
  msg.orientation.w = 1.0;
  msg.orientation_covariance[0] = -1.0;
  localslam::conversions::AttObs att;
  EXPECT_TRUE(imu_orientation_unavailable(msg));
  EXPECT_FALSE(imu_to_internal_att(msg, att));

  msg.orientation_covariance[0] = 0.0;
  msg.orientation.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(imu_to_internal_att(msg, att));
  EXPECT_FALSE(quaternion_usable(0, 0, 0, 0));
}

TEST(SensorConversions, RangeGates)
{
  sensor_msgs::msg::Range msg;
  msg.range = 1.5f;
  msg.min_range = 0.1f;
  msg.max_range = 5.0f;
  msg.variance = 0.5f;
  localslam::conversions::RangeObs range;
  EXPECT_TRUE(range_msg_to_internal_range(msg, range, 0.4, 6.0));
  msg.range = 0.2f;
  EXPECT_FALSE(range_msg_to_internal_range(msg, range, 0.4, 6.0));
  msg.range = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(range_msg_to_internal_range(msg, range, 0.4, 6.0));
}

// ---------------------------------------------------------------------------
// Input selection plan
// ---------------------------------------------------------------------------
TEST(InputPlan, StandardExcludesLegacyMigrated)
{
  SensorInputPlan p = make_sensor_input_plan("standard");
  EXPECT_TRUE(p.use_standard_migrated);
  EXPECT_TRUE(p.sub_image_std);
  EXPECT_TRUE(p.sub_imu_std);
  EXPECT_TRUE(p.sub_body_velocity_std);
  EXPECT_TRUE(p.sub_downward_range_std);
  EXPECT_TRUE(p.sub_altitude_std);
  EXPECT_FALSE(p.sub_frame_legacy);
  EXPECT_FALSE(p.sub_att_legacy);
  EXPECT_FALSE(p.sub_spd_legacy);
  EXPECT_FALSE(p.sub_range_legacy);
  EXPECT_FALSE(p.sub_alt_legacy);
  EXPECT_TRUE(p.sub_rangetobase);
}

TEST(InputPlan, LegacyKeepsCustomMigrated)
{
  SensorInputPlan p = make_sensor_input_plan("legacy");
  EXPECT_FALSE(p.use_standard_migrated);
  EXPECT_FALSE(p.sub_image_std);
  EXPECT_TRUE(p.sub_frame_legacy);
  EXPECT_TRUE(p.sub_att_legacy);
  EXPECT_TRUE(p.sub_spd_legacy);
  EXPECT_TRUE(p.sub_range_legacy);
  EXPECT_TRUE(p.sub_alt_legacy);
  EXPECT_TRUE(p.sub_rangetobase);
}

TEST(InputPlan, UnsupportedModesThrow)
{
  EXPECT_THROW(make_sensor_input_plan("mixed"), std::runtime_error);
  EXPECT_NO_THROW(validate_altitude_input_mode("derived_altitude"));
  EXPECT_NO_THROW(validate_altitude_input_mode("fluid_pressure"));
  EXPECT_THROW(validate_altitude_input_mode("bogus"), std::runtime_error);
}

TEST(InputPlan, NoDualFusionFlags)
{
  auto s = make_sensor_input_plan("standard");
  // Cannot have both standard and legacy for the same modality
  EXPECT_FALSE(s.sub_image_std && s.sub_frame_legacy);
  EXPECT_FALSE(s.sub_imu_std && s.sub_att_legacy);
  EXPECT_FALSE(s.sub_body_velocity_std && s.sub_spd_legacy);
  EXPECT_FALSE(s.sub_downward_range_std && s.sub_range_legacy);
  EXPECT_FALSE(s.sub_altitude_std && s.sub_alt_legacy);
}

TEST(InputPlan, ExclusiveAltitudeSources)
{
  using localslam::apply_altitude_input_mode;
  {
    auto s = make_sensor_input_plan("standard");
    apply_altitude_input_mode(s, "derived_altitude");
    EXPECT_TRUE(s.sub_altitude_std);
    EXPECT_FALSE(s.sub_pressure_std);
  }
  {
    auto s = make_sensor_input_plan("standard");
    apply_altitude_input_mode(s, "fluid_pressure");
    EXPECT_FALSE(s.sub_altitude_std);
    EXPECT_TRUE(s.sub_pressure_std);
  }
  {
    auto s = make_sensor_input_plan("legacy");
    EXPECT_THROW(apply_altitude_input_mode(s, "fluid_pressure"), std::runtime_error);
  }
}

// ---------------------------------------------------------------------------
// Services / run control
// ---------------------------------------------------------------------------
TEST(RunControl, EnableDisableReenableReset)
{
  EKFRunControl c;
  EXPECT_FALSE(c.steps_running);
  c.set_running(true);   // enable before init
  EXPECT_TRUE(c.steps_running);
  c.set_running(false);  // disable
  EXPECT_FALSE(c.steps_running);
  c.set_running(true);   // re-enable without implicit reinit
  EXPECT_TRUE(c.steps_running);
  EXPECT_FALSE(c.reinit_requested);

  c.request_reinit();    // reset while enabled
  EXPECT_TRUE(c.consume_reinit());
  EXPECT_FALSE(c.consume_reinit());
  EXPECT_TRUE(c.steps_running);  // reset does not clear enable

  c.set_running(false);
  c.request_reinit();    // reset while disabled
  EXPECT_TRUE(c.consume_reinit());
  EXPECT_FALSE(c.steps_running);
}

// ---------------------------------------------------------------------------
// Timestamp / range-frame association (A1 behavior change vs zero stamps)
// ---------------------------------------------------------------------------
TEST(RangeAssoc, SonarWithinThresholdNoAltFallback)
{
  localslam::RangeFrameAssociationState s;
  ASSERT_TRUE(note_downward_range(s, 1.2, 1000));
  note_altitude_for_fallback(s, 3.0);
  double fr = -1.0;
  // dt = 50 <= 100 → sonar kept; alt fallback must not replace.
  std::string t = associate_depth_to_frame(s, fr, 1050);
  EXPECT_EQ(t, "sonar");
  EXPECT_DOUBLE_EQ(fr, 1.2);
}

TEST(RangeAssoc, SonarPendingButDtOutsideThresholdStillAltOverwrite)
{
  // Legacy getData order: apply sonar, then if dt>100 and alt pending, overwrite.
  localslam::RangeFrameAssociationState s;
  ASSERT_TRUE(note_downward_range(s, 1.2, 1000));
  note_altitude_for_fallback(s, 3.0);
  double fr = -1.0;
  std::string t = associate_depth_to_frame(s, fr, 1000 + kAltFallbackDtMs + 1);
  EXPECT_EQ(t, "alt");
  EXPECT_DOUBLE_EQ(fr, 3.0);
}

TEST(RangeAssoc, OutsideThresholdUsesAltitudeFallback)
{
  localslam::RangeFrameAssociationState s;
  ASSERT_TRUE(note_downward_range(s, 1.2, 1000));
  // Consume sonar on an earlier frame first
  double fr1 = -1.0;
  associate_depth_to_frame(s, fr1, 1010);
  note_altitude_for_fallback(s, 2.5);
  double fr2 = -1.0;
  // No pending sonar; dt from last_range_time=1000 → frame 1200 is 200 > 100
  std::string t = associate_depth_to_frame(s, fr2, 1200);
  EXPECT_EQ(t, "alt");
  EXPECT_DOUBLE_EQ(fr2, 2.5);
}

TEST(RangeAssoc, LegacyZeroTimestampsSuppressAltFallback)
{
  // Pre-A1 Webots: times often 0 → dt=0 → alt fallback condition false.
  localslam::RangeFrameAssociationState s;
  note_altitude_for_fallback(s, 2.0);
  // No sonar pending
  double fr = -1.0;
  std::string t = associate_depth_to_frame(s, fr, 0);
  // last_range_time default 0 → dt=0 → not > 100
  EXPECT_EQ(t, "");
  EXPECT_DOUBLE_EQ(fr, -1.0);
  EXPECT_TRUE(s.range_available_alt);
}

TEST(RangeAssoc, InvalidRangeRejectedByGate)
{
  localslam::RangeFrameAssociationState s;
  EXPECT_FALSE(note_downward_range(s, 0.1, 500));
  EXPECT_FALSE(s.range_available);
}

// ---------------------------------------------------------------------------
// B2: configurable association / undelayed-init gates
// ---------------------------------------------------------------------------
TEST(RangeAssoc, SonarDefaultExclusiveBounds)
{
  EXPECT_FALSE(range_passes_assoc_gate(0.4, kRangeGateMinM, kRangeGateMaxM));
  EXPECT_TRUE(range_passes_assoc_gate(0.4000001, kRangeGateMinM, kRangeGateMaxM));
  EXPECT_TRUE(range_passes_assoc_gate(1.0, kRangeGateMinM, kRangeGateMaxM));
  EXPECT_TRUE(range_passes_assoc_gate(5.999, kRangeGateMinM, kRangeGateMaxM));
  EXPECT_FALSE(range_passes_assoc_gate(6.0, kRangeGateMinM, kRangeGateMaxM));

  localslam::RangeFrameAssociationState s;
  EXPECT_FALSE(note_downward_range(s, 0.4, 1));
  EXPECT_FALSE(note_downward_range(s, 6.0, 2));
  EXPECT_TRUE(note_downward_range(s, 1.5, 3));
  EXPECT_TRUE(s.range_available);
  EXPECT_DOUBLE_EQ(s.last_range, 1.5);
}

TEST(RangeAssoc, CustomLongRangeBoundsAcceptAboveSix)
{
  constexpr double lo = 0.1;
  constexpr double hi = 150.0;
  EXPECT_FALSE(range_passes_assoc_gate(0.1, lo, hi));
  EXPECT_TRUE(range_passes_assoc_gate(0.11, lo, hi));
  EXPECT_TRUE(range_passes_assoc_gate(50.0, lo, hi));
  EXPECT_TRUE(range_passes_assoc_gate(149.9, lo, hi));
  EXPECT_FALSE(range_passes_assoc_gate(150.0, lo, hi));

  localslam::RangeFrameAssociationState s;
  EXPECT_FALSE(note_downward_range(s, 10.0, 10));  // default sonar gate
  EXPECT_TRUE(note_downward_range(s, 10.0, 11, lo, hi));
  EXPECT_DOUBLE_EQ(s.last_range, 10.0);
}

TEST(RangeAssoc, UndelayedInitThresholdExclusive)
{
  EXPECT_FALSE(range_permits_undelayed_init(0.5, kRangeInitMinM));
  EXPECT_TRUE(range_permits_undelayed_init(0.5000001, kRangeInitMinM));
  // Assoc can accept (0.4, 0.5] but undelayed init still refuses at == 0.5
  EXPECT_TRUE(range_passes_assoc_gate(0.45, kRangeGateMinM, kRangeGateMaxM));
  EXPECT_FALSE(range_permits_undelayed_init(0.45, kRangeInitMinM));
}

TEST(RangeAssoc, NonFiniteNeverPassesAssocGate)
{
  EXPECT_FALSE(range_passes_assoc_gate(
    std::numeric_limits<double>::quiet_NaN(), kRangeGateMinM, kRangeGateMaxM));
  EXPECT_FALSE(range_passes_assoc_gate(
    std::numeric_limits<double>::infinity(), kRangeGateMinM, kRangeGateMaxM));
  localslam::RangeFrameAssociationState s;
  EXPECT_FALSE(note_downward_range(
    s, std::numeric_limits<double>::quiet_NaN(), 1));
  EXPECT_FALSE(s.range_available);
}

TEST(RangeAssoc, ValidateRangeAcceptanceParams)
{
  EXPECT_NO_THROW(validate_range_acceptance_params(0.4, 6.0, 0.5));
  EXPECT_NO_THROW(validate_range_acceptance_params(0.1, 150.0, 0.5));
  // init may be <= assoc_min (intersection decides eligibility)
  EXPECT_NO_THROW(validate_range_acceptance_params(0.4, 6.0, 0.2));
  EXPECT_THROW(validate_range_acceptance_params(-0.1, 6.0, 0.5), std::runtime_error);
  EXPECT_THROW(validate_range_acceptance_params(0.4, 0.4, 0.2), std::runtime_error);
  EXPECT_THROW(validate_range_acceptance_params(0.4, 6.0, 6.0), std::runtime_error);
  EXPECT_THROW(validate_range_acceptance_params(0.4, 6.0, -1.0), std::runtime_error);
  EXPECT_THROW(
    validate_range_acceptance_params(
      std::numeric_limits<double>::quiet_NaN(), 6.0, 0.5),
    std::runtime_error);
}

TEST(RangeAssoc, MissingRangeKeepsFrameUnsetAndConsumeOnce)
{
  localslam::RangeFrameAssociationState s;
  double fr = -1.0;
  EXPECT_EQ(associate_depth_to_frame(s, fr, 100), "");
  EXPECT_DOUBLE_EQ(fr, -1.0);
  EXPECT_FALSE(range_permits_undelayed_init(fr, kRangeInitMinM));

  ASSERT_TRUE(note_downward_range(s, 2.0, 200));
  fr = -1.0;
  EXPECT_EQ(associate_depth_to_frame(s, fr, 210), "sonar");
  EXPECT_DOUBLE_EQ(fr, 2.0);
  EXPECT_FALSE(s.range_available);
  double fr2 = -1.0;
  EXPECT_EQ(associate_depth_to_frame(s, fr2, 220), "");
  EXPECT_DOUBLE_EQ(fr2, -1.0);
}

TEST(RangeAssoc, HeaderlessNodeTimeIsMeaningfulDt)
{
  // Simulates headerless GPS-speed / now() stamping: times are nonzero and
  // differ, so A1 can trigger alt fallback unlike all-zero legacy stamps.
  localslam::RangeFrameAssociationState s;
  ASSERT_TRUE(note_downward_range(s, 1.0, 10'000));
  double fr = -1;
  associate_depth_to_frame(s, fr, 10'010);  // consume sonar
  note_altitude_for_fallback(s, 4.0);
  fr = -1;
  std::string t = associate_depth_to_frame(s, fr, 10'000 + kAltFallbackDtMs + 1);
  EXPECT_EQ(t, "alt");
  EXPECT_DOUBLE_EQ(fr, 4.0);
}

TEST(Timestamp, StampToMsZeroVsNonzero)
{
  builtin_interfaces::msg::Time z;
  z.sec = 0;
  z.nanosec = 0;
  EXPECT_EQ(localslam::conversions::stamp_to_ms(z), 0);

  builtin_interfaces::msg::Time t;
  t.sec = 12;
  t.nanosec = 345'000'000;
  EXPECT_EQ(localslam::conversions::stamp_to_ms(t), 12'345);

  // Exact A1 vs pre-A1 difference for range↔frame:
  // Pre-A1 custom msgs often carried time=0 → frame.time - last_range_time = 0
  //   → (dt > 100) false → altitude-as-depth fallback never/rarely fires.
  // A1 stores stamp_to_ms(header.stamp) (or bridge now() for headerless) →
  //   nonzero dt can exceed 100 ms and activate the fallback.
}

// ---------------------------------------------------------------------------
// B4: altitude→FRAME.range fallback flag + confidence σ classification
// ---------------------------------------------------------------------------
TEST(RangeAssoc, DefaultTruePreservesAltOverwrite)
{
  localslam::RangeFrameAssociationState s;
  ASSERT_TRUE(note_downward_range(s, 1.2, 1000));
  note_altitude_for_fallback(s, 3.0);
  double fr = -1.0;
  std::string t = associate_depth_to_frame(
    s, fr, 1000 + kAltFallbackDtMs + 1, /*allow_altitude_fallback=*/true);
  EXPECT_EQ(t, "alt");
  EXPECT_DOUBLE_EQ(fr, 3.0);
}

TEST(RangeAssoc, FallbackDisabledKeepsLaserAndClearsAltLatch)
{
  localslam::RangeFrameAssociationState s;
  ASSERT_TRUE(note_downward_range(s, 8.5, 1000, 0.1, 150.0));
  note_altitude_for_fallback(s, 12.0);
  double fr = -1.0;
  // Would overwrite under legacy true; false must keep laser and clear latch.
  std::string t = associate_depth_to_frame(
    s, fr, 1000 + kAltFallbackDtMs + 1, /*allow_altitude_fallback=*/false);
  EXPECT_EQ(t, "sonar");
  EXPECT_DOUBLE_EQ(fr, 8.5);
  EXPECT_FALSE(s.range_available_alt);
}

TEST(RangeAssoc, FallbackDisabledLeavesFrameUnsetWithoutRange)
{
  localslam::RangeFrameAssociationState s;
  note_altitude_for_fallback(s, 9.0);
  double fr = -1.0;
  std::string t = associate_depth_to_frame(
    s, fr, 500 + kAltFallbackDtMs + 1, false);
  EXPECT_EQ(t, "");
  EXPECT_DOUBLE_EQ(fr, -1.0);
  EXPECT_FALSE(s.range_available_alt);
  EXPECT_FALSE(range_permits_undelayed_init(fr, kRangeInitMinM));
}

TEST(RangeConfidence, InVsOutRegionAndFirstLandmarkScale)
{
  // Production: only σ_ini changes; eligibility is independent.
  EXPECT_TRUE(feature_in_range_confidence_region(10.0, 40.0, 8.0, 120.0));
  EXPECT_FALSE(feature_in_range_confidence_region(50.0, 40.0, 8.0, 120.0));
  EXPECT_FALSE(feature_in_range_confidence_region(10.0, 40.0, 120.0, 120.0));

  EXPECT_DOUBLE_EQ(
    initial_depth_sigma_m(true, true, 1.0, 5.0), 1.0);
  EXPECT_DOUBLE_EQ(
    initial_depth_sigma_m(false, true, 1.0, 5.0), 5.0);
  EXPECT_DOUBLE_EQ(
    initial_depth_sigma_m(true, false, 1.0, 5.0),
    1.0 * kFirstLandmarkDepthSigmaScale);
  EXPECT_DOUBLE_EQ(
    initial_depth_sigma_m(false, false, 1.0, 5.0),
    5.0 * kFirstLandmarkDepthSigmaScale);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
