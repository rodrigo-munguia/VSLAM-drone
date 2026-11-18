#include <cmath>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "range_frame_association.hpp"

namespace
{

/** Mirror localslam declare/get defaults for B2/B4 range params. */
void declare_range_acceptance_params(rclcpp::Node & node)
{
  node.declare_parameter<double>("Range_assoc_min_m", 0.4);
  node.declare_parameter<double>("Range_assoc_max_m", 6.0);
  node.declare_parameter<double>("Range_init_min_m", 0.5);
  node.declare_parameter<bool>("Range_allow_altitude_fallback", true);
}

void get_range_acceptance_params(
  rclcpp::Node & node, double & lo, double & hi, double & init_min)
{
  node.get_parameter("Range_assoc_min_m", lo);
  node.get_parameter("Range_assoc_max_m", hi);
  node.get_parameter("Range_init_min_m", init_min);
}

}  // namespace

class RangeAcceptanceParamsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    if (!rclcpp::ok()) {
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(RangeAcceptanceParamsTest, DefaultDeclaresMatchSonarGates)
{
  auto node = std::make_shared<rclcpp::Node>("b2_range_defaults");
  declare_range_acceptance_params(*node);
  double lo = 0, hi = 0, init_min = 0;
  get_range_acceptance_params(*node, lo, hi, init_min);
  EXPECT_DOUBLE_EQ(lo, localslam::kRangeGateMinM);
  EXPECT_DOUBLE_EQ(hi, localslam::kRangeGateMaxM);
  EXPECT_DOUBLE_EQ(init_min, localslam::kRangeInitMinM);
  EXPECT_NO_THROW(
    localslam::validate_range_acceptance_params(lo, hi, init_min));
}

TEST_F(RangeAcceptanceParamsTest, OverridesLoadAndValidateLongRange)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("Range_assoc_min_m", 0.1),
    rclcpp::Parameter("Range_assoc_max_m", 150.0),
    rclcpp::Parameter("Range_init_min_m", 0.5),
  });
  auto node = std::make_shared<rclcpp::Node>("b2_range_long", opts);
  declare_range_acceptance_params(*node);
  double lo = 0, hi = 0, init_min = 0;
  get_range_acceptance_params(*node, lo, hi, init_min);
  EXPECT_DOUBLE_EQ(lo, 0.1);
  EXPECT_DOUBLE_EQ(hi, 150.0);
  EXPECT_DOUBLE_EQ(init_min, 0.5);
  EXPECT_NO_THROW(
    localslam::validate_range_acceptance_params(lo, hi, init_min));
  EXPECT_TRUE(localslam::range_passes_assoc_gate(100.0, lo, hi));
}

TEST_F(RangeAcceptanceParamsTest, InvalidOverrideFailsValidation)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("Range_assoc_min_m", 10.0),
    rclcpp::Parameter("Range_assoc_max_m", 5.0),
    rclcpp::Parameter("Range_init_min_m", 0.5),
  });
  auto node = std::make_shared<rclcpp::Node>("b2_range_bad", opts);
  declare_range_acceptance_params(*node);
  double lo = 0, hi = 0, init_min = 0;
  get_range_acceptance_params(*node, lo, hi, init_min);
  EXPECT_THROW(
    localslam::validate_range_acceptance_params(lo, hi, init_min),
    std::runtime_error);
}

TEST_F(RangeAcceptanceParamsTest, AltitudeFallbackDefaultTrue)
{
  auto node = std::make_shared<rclcpp::Node>("b4_alt_fb_default");
  declare_range_acceptance_params(*node);
  bool allow = false;
  ASSERT_TRUE(node->get_parameter("Range_allow_altitude_fallback", allow));
  EXPECT_TRUE(allow);
}

TEST_F(RangeAcceptanceParamsTest, LaserProfileLoadsFallbackFalseAndGates)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("Range_assoc_min_m", 0.1),
    rclcpp::Parameter("Range_assoc_max_m", 150.0),
    rclcpp::Parameter("Range_init_min_m", 0.5),
    rclcpp::Parameter("Range_allow_altitude_fallback", false),
    rclcpp::Parameter("Range_sensor_max_range_pattern", 50.0),
    rclcpp::Parameter("Range_sensor_max_r_max", 5.0),
    rclcpp::Parameter("Range_sensor_max_range_operation", 120.0),
    rclcpp::Parameter("Sigma_cam_initial_depth_range", 1.0),
    rclcpp::Parameter("Sigma_cam_initial_depth_WOrange", 5.0),
  });
  auto node = std::make_shared<rclcpp::Node>("b4_laser_profile", opts);
  node->declare_parameter<double>("Range_assoc_min_m", 0.4);
  node->declare_parameter<double>("Range_assoc_max_m", 6.0);
  node->declare_parameter<double>("Range_init_min_m", 0.5);
  node->declare_parameter<bool>("Range_allow_altitude_fallback", true);
  node->declare_parameter<double>("Range_sensor_max_range_pattern", 6.0);
  node->declare_parameter<double>("Range_sensor_max_r_max", 1.75);
  node->declare_parameter<double>("Range_sensor_max_range_operation", 6.0);
  node->declare_parameter<double>("Sigma_cam_initial_depth_range", 0.1);
  node->declare_parameter<double>("Sigma_cam_initial_depth_WOrange", 0.2);

  double lo = 0, hi = 0, init_min = 0;
  get_range_acceptance_params(*node, lo, hi, init_min);
  bool allow = true;
  double pattern = 0, r_max = 0, op = 0, sig_in = 0, sig_wo = 0;
  node->get_parameter("Range_allow_altitude_fallback", allow);
  node->get_parameter("Range_sensor_max_range_pattern", pattern);
  node->get_parameter("Range_sensor_max_r_max", r_max);
  node->get_parameter("Range_sensor_max_range_operation", op);
  node->get_parameter("Sigma_cam_initial_depth_range", sig_in);
  node->get_parameter("Sigma_cam_initial_depth_WOrange", sig_wo);

  EXPECT_DOUBLE_EQ(lo, 0.1);
  EXPECT_DOUBLE_EQ(hi, 150.0);
  EXPECT_DOUBLE_EQ(init_min, 0.5);
  EXPECT_FALSE(allow);
  EXPECT_DOUBLE_EQ(pattern, 50.0);
  EXPECT_DOUBLE_EQ(r_max, 5.0);
  EXPECT_DOUBLE_EQ(op, 120.0);
  EXPECT_DOUBLE_EQ(sig_in, 1.0);
  EXPECT_DOUBLE_EQ(sig_wo, 5.0);
  EXPECT_TRUE(localslam::range_passes_assoc_gate(8.0, lo, hi));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
