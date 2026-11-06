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
using localslam::kAltFallbackDtMs;
using localslam::make_sensor_input_plan;
using localslam::note_altitude_for_fallback;
using localslam::note_downward_range;
using localslam::validate_altitude_input_mode;
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
