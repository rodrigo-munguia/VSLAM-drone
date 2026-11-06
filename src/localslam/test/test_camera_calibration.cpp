#include <gtest/gtest.h>

#include "camera_calibration.hpp"

using localslam::camera_cal::Intrinsics;
using localslam::camera_cal::intrinsics_equal;
using localslam::camera_cal::map_camera_info;

static sensor_msgs::msg::CameraInfo make_valid_raw_info()
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header.frame_id = "camera_optical_frame";
  msg.width = 320;
  msg.height = 240;
  msg.distortion_model = "plumb_bob";
  msg.d = {0.01, -0.02, 0.0, 0.0, 0.0};
  msg.k = {
    200.0, 0.0, 160.0,
    0.0, 210.0, 120.0,
    0.0, 0.0, 1.0};
  msg.p = {
    200.0, 0.0, 160.0, 0.0,
    0.0, 210.0, 120.0, 0.0,
    0.0, 0.0, 1.0, 0.0};
  msg.binning_x = 0;
  msg.binning_y = 0;
  return msg;
}

TEST(CameraCalibration, RawPathMapsKAndD)
{
  const auto msg = make_valid_raw_info();
  const auto mapped = map_camera_info(msg, "raw");
  ASSERT_TRUE(mapped.ok) << mapped.error;
  EXPECT_EQ(mapped.intrinsics.width, 320);
  EXPECT_EQ(mapped.intrinsics.height, 240);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.fc_u, 200.0);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.fc_v, 210.0);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.cc_u, 160.0);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.cc_v, 120.0);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.alpha_c, 0.0);
  ASSERT_EQ(mapped.intrinsics.distortions.size(), 5u);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.distortions[0], 0.01);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.distortions[1], -0.02);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.distortions[2], 0.0);
}

TEST(CameraCalibration, RawPathRejectsTangential)
{
  auto msg = make_valid_raw_info();
  msg.d = {0.0, 0.0, 0.001, 0.0, 0.0};
  const auto mapped = map_camera_info(msg, "raw");
  EXPECT_FALSE(mapped.ok);
  EXPECT_NE(mapped.error.find("unsupported distortion"), std::string::npos);
}

TEST(CameraCalibration, RawPathRejectsEquidistant)
{
  auto msg = make_valid_raw_info();
  msg.distortion_model = "equidistant";
  const auto mapped = map_camera_info(msg, "raw");
  EXPECT_FALSE(mapped.ok);
}

TEST(CameraCalibration, RawPathRejectsBinningAndRoi)
{
  auto msg = make_valid_raw_info();
  msg.binning_x = 2;
  EXPECT_FALSE(map_camera_info(msg, "raw").ok);

  msg = make_valid_raw_info();
  msg.roi.x_offset = 10;
  msg.roi.width = 100;
  msg.roi.height = 100;
  EXPECT_FALSE(map_camera_info(msg, "raw").ok);
}

TEST(CameraCalibration, RectifiedUsesPAndForcesD0)
{
  auto msg = make_valid_raw_info();
  msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  msg.p = {
    190.0, 0.0, 155.0, 0.0,
    0.0, 205.0, 118.0, 0.0,
    0.0, 0.0, 1.0, 0.0};
  const auto mapped = map_camera_info(msg, "rectified");
  ASSERT_TRUE(mapped.ok) << mapped.error;
  EXPECT_DOUBLE_EQ(mapped.intrinsics.fc_u, 190.0);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.fc_v, 205.0);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.cc_u, 155.0);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.cc_v, 118.0);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.distortions[0], 0.0);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.distortions[1], 0.0);
}

TEST(CameraCalibration, RectifiedRejectsNonzeroD)
{
  auto msg = make_valid_raw_info();
  msg.d = {0.01, 0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(map_camera_info(msg, "rectified").ok);
}

TEST(CameraCalibration, RejectsUncalibratedK)
{
  auto msg = make_valid_raw_info();
  msg.k[0] = 0.0;
  EXPECT_FALSE(map_camera_info(msg, "raw").ok);
}

TEST(CameraCalibration, IntrinsicsEqualIgnoresOpticalFrame)
{
  Intrinsics a;
  a.width = 320;
  a.height = 240;
  a.fc_u = 1.0;
  a.fc_v = 2.0;
  a.cc_u = 3.0;
  a.cc_v = 4.0;
  a.alpha_c = 0.0;
  a.distortions = {0, 0, 0, 0, 0};
  a.image_path = "raw";
  a.optical_frame = "a";
  Intrinsics b = a;
  b.optical_frame = "b";
  EXPECT_TRUE(intrinsics_equal(a, b));
  b.fc_u = 9.0;
  EXPECT_FALSE(intrinsics_equal(a, b));
}

TEST(CameraCalibration, YamlEquivalentCameraInfoMatchesMarsDefaults)
{
  // Equivalent to A6 mars YAML Mono_cam_* (400x240, fc=200, D=0).
  sensor_msgs::msg::CameraInfo msg;
  msg.header.frame_id = "camera_optical_frame";
  msg.width = 400;
  msg.height = 240;
  msg.distortion_model = "plumb_bob";
  msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  msg.k = {
    200.0, 0.0, 200.0,
    0.0, 200.0, 120.0,
    0.0, 0.0, 1.0};
  const auto mapped = map_camera_info(msg, "raw");
  ASSERT_TRUE(mapped.ok) << mapped.error;
  EXPECT_EQ(mapped.intrinsics.width, 400);
  EXPECT_EQ(mapped.intrinsics.height, 240);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.fc_u, 200.0);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.fc_v, 200.0);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.cc_u, 200.0);
  EXPECT_DOUBLE_EQ(mapped.intrinsics.cc_v, 120.0);
}
