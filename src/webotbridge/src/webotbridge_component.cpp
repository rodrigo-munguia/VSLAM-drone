// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "webotbridge_component.hpp"
#include "gt_frame_conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <random>
#include <cmath>
#include <limits>
#include <thread>
#include "../../common/Transforms/Euler_to_Ra2b.hpp"

using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

namespace webotbridge
{

WEBOTb::WEBOTb(const rclcpp::NodeOptions & options)
: Node("webotb", options)
{
  setParameters();

  sub_Imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10, std::bind(&WEBOTb::Imu_callback, this, _1));
  sub_Camera_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera", 10, std::bind(&WEBOTb::Camera_callback, this, _1));
  sub_Range_ = this->create_subscription<sensor_msgs::msg::Range>(
    "ds0", 10, std::bind(&WEBOTb::Range_callback, this, _1));
  sub_Gps_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "webot_robot/gps", 10, std::bind(&WEBOTb::Gps_callback, this, _1));
  sub_GpsSpeed_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "webot_robot/gps/speed_vector", 10,
    std::bind(&WEBOTb::GpsSpeed_callback, this, _1));
  {
    rclcpp::QoS qos_ref(rclcpp::KeepLast(1));
    qos_ref.reliable();
    qos_ref.transient_local();
    sub_EstimatorRef_ =
      this->create_subscription<interfaces::msg::EstimatorReference>(
        "EKFslam/estimator_reference", qos_ref,
        std::bind(&WEBOTb::EstimatorReference_callback, this, _1));
  }

  pub_Alt_ = create_publisher<interfaces::msg::Alt>("Altitude_topic", 10);
  pub_Att_ = create_publisher<interfaces::msg::Att>("Attitude_topic", 10);
  pub_Gps_ = create_publisher<interfaces::msg::Gps>("Gps_topic", 10);
  pub_Range_ = create_publisher<interfaces::msg::Range>("Range_topic", 10);
  pub_Frame_ = create_publisher<interfaces::msg::Frame>("Frame_topic", 10);
  pub_Spd_ = create_publisher<interfaces::msg::Spd>("Speed_topic", 10);
  pub_OdoD_ = create_publisher<interfaces::msg::Ododiff>("OdometryD_topic", 10);
  pub_OdoV_ = create_publisher<interfaces::msg::Odovw>("OdometryV_topic", 10);
  pub_RangeToBase_ =
    create_publisher<interfaces::msg::Rangetobase>("Rangetobase_topic", 10);
  pub_GT_ = create_publisher<geometry_msgs::msg::PointStamped>("GT_topic", 10);
  {
    rclcpp::QoS qos_path(rclcpp::KeepLast(1));
    qos_path.reliable();
    qos_path.transient_local();
    pub_GT_path_ = create_publisher<nav_msgs::msg::Path>("ground_truth/path", qos_path);
  }
  gt_path_.header.frame_id = gt_path_frame_id_;

  // Standard outputs (parallel). Launch remaps these onto EKFslam ~/topics.
  pub_Image_std_ = create_publisher<sensor_msgs::msg::Image>("image", 10);
  {
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.reliable();
    pub_CameraInfo_std_ =
      create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
  }
  pub_Imu_std_ = create_publisher<sensor_msgs::msg::Imu>("imu_out", 10);
  pub_BodyVel_std_ =
    create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "body_velocity", 10);
  pub_DownwardRange_std_ =
    create_publisher<sensor_msgs::msg::Range>("downward_range", 10);
  pub_Altitude_std_ =
    create_publisher<mavros_msgs::msg::Altitude>("altitude", 10);

  Range_available = false;
  Rn2rG.eye();

  RCLCPP_INFO(
    this->get_logger(),
    "WEBOTb publishing legacy topics and standard image/camera_info/imu_out/"
    "body_velocity/downward_range/altitude (see launch remaps). "
    "CameraInfo %dx%d fc=(%.1f,%.1f) (Publish_camera_info=%d). "
    "mavros Altitude.relative = GPS-height emulation [m], +up; "
    "other altitude fields set to NaN (unavailable).",
    Camera_width_, Camera_height_, Camera_fc_u_, Camera_fc_v_,
    Publish_camera_info_ ? 1 : 0);
}

WEBOTb::~WEBOTb() {}

void WEBOTb::setParameters()
{
  this->declare_parameter<double>("Sigma_roll", 0.017453);
  this->declare_parameter<double>("Sigma_pitch", 0.017453);
  this->declare_parameter<double>("Sigma_yaw", 0.032);
  this->declare_parameter<double>("Sigma_Alt", 0.1);
  this->declare_parameter("Base_location", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter<double>("Sigma_rangetobase", 0.25);
  this->declare_parameter<double>("Sigma_Spd", 0.05);
  // A8: defaults match A6 mars / localslam Mono_cam_* (400x240, fc=200, D=0).
  this->declare_parameter<bool>("Publish_camera_info", true);
  this->declare_parameter<int>("Camera_width", 400);
  this->declare_parameter<int>("Camera_height", 240);
  this->declare_parameter<double>("Camera_fc_u", 200.0);
  this->declare_parameter<double>("Camera_fc_v", 200.0);
  this->declare_parameter<double>("Camera_cc_u", 200.0);
  this->declare_parameter<double>("Camera_cc_v", 120.0);
  this->declare_parameter<double>("Camera_alpha_c", 0.0);
  this->declare_parameter<double>("Camera_k1", 0.0);
  this->declare_parameter<double>("Camera_k2", 0.0);
  this->declare_parameter<std::string>("Camera_optical_frame", "camera_optical_frame");
  // Internal-frame Robot_init_* (same as Local SLAM); converted via D=diag(1,-1,-1).
  // Prefer values from EstimatorReference when available.
  this->declare_parameter<double>("GT_robot_init_x", 0.0);
  this->declare_parameter<double>("GT_robot_init_y", 0.0);
  this->declare_parameter<double>("GT_robot_init_z", 0.0);
  // GPS/IMU history window for matching EstimatorReference stamps [s].
  this->declare_parameter<double>("GT_history_duration_sec", 5.0);
  // Max |dt| from query stamp to nearest GPS sample [s].
  this->declare_parameter<double>("GT_match_max_dt_sec", 0.15);
  // Max poses retained on ground_truth/path. Values < 1 → clamped to 1.
  this->declare_parameter<int>("GT_path_max_poses", 2000);
  this->declare_parameter<std::string>("GT_path_frame_id", "odom");

  this->get_parameter("Sigma_roll", PAR.Sigma_roll);
  this->get_parameter("Sigma_pitch", PAR.Sigma_pitch);
  this->get_parameter("Sigma_yaw", PAR.Sigma_yaw);
  this->get_parameter("Sigma_Alt", PAR.Sigma_Alt);
  rclcpp::Parameter Base_location_par = this->get_parameter("Base_location");
  PAR.Base_location = Base_location_par.as_double_array();
  this->get_parameter("Sigma_rangetobase", PAR.Sigma_rangetobase);
  this->get_parameter("Sigma_Spd", PAR.Sigma_Spd);
  this->get_parameter("Publish_camera_info", Publish_camera_info_);
  this->get_parameter("Camera_width", Camera_width_);
  this->get_parameter("Camera_height", Camera_height_);
  this->get_parameter("Camera_fc_u", Camera_fc_u_);
  this->get_parameter("Camera_fc_v", Camera_fc_v_);
  this->get_parameter("Camera_cc_u", Camera_cc_u_);
  this->get_parameter("Camera_cc_v", Camera_cc_v_);
  this->get_parameter("Camera_alpha_c", Camera_alpha_c_);
  this->get_parameter("Camera_k1", Camera_k1_);
  this->get_parameter("Camera_k2", Camera_k2_);
  this->get_parameter("Camera_optical_frame", Camera_optical_frame_);
  this->get_parameter("GT_robot_init_x", gt_ref_.robot_init_internal[0]);
  this->get_parameter("GT_robot_init_y", gt_ref_.robot_init_internal[1]);
  this->get_parameter("GT_robot_init_z", gt_ref_.robot_init_internal[2]);
  this->get_parameter("GT_history_duration_sec", gt_history_duration_sec_);
  this->get_parameter("GT_match_max_dt_sec", gt_match_max_dt_sec_);
  this->get_parameter("GT_path_max_poses", gt_path_max_poses_);
  this->get_parameter("GT_path_frame_id", gt_path_frame_id_);
  if (gt_path_max_poses_ < 1) {
    gt_path_max_poses_ = 1;
  }
  PAR.Publish_camera_info = Publish_camera_info_;
  PAR.Camera_width = Camera_width_;
  PAR.Camera_height = Camera_height_;
  PAR.Camera_fc_u = Camera_fc_u_;
  PAR.Camera_fc_v = Camera_fc_v_;
  PAR.Camera_cc_u = Camera_cc_u_;
  PAR.Camera_cc_v = Camera_cc_v_;
  PAR.Camera_alpha_c = Camera_alpha_c_;
  PAR.Camera_k1 = Camera_k1_;
  PAR.Camera_k2 = Camera_k2_;
  PAR.Camera_optical_frame = Camera_optical_frame_;
}

void WEBOTb::loop()
{
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

double WEBOTb::addGaussianNoise(double value, double stdDev)
{
  std::random_device rd;
  std::mt19937 generator(rd());
  std::normal_distribution<double> distribution(0.0, stdDev);
  return value + distribution(generator);
}

builtin_interfaces::msg::Time WEBOTb::now_stamp() const
{
  return this->get_clock()->now();
}

builtin_interfaces::msg::Time WEBOTb::stamp_from_msg_or_now(
  const std_msgs::msg::Header & header) const
{
  if (header.stamp.sec != 0 || header.stamp.nanosec != 0) {
    return header.stamp;
  }
  return now_stamp();
}

void WEBOTb::Imu_callback(const sensor_msgs::msg::Imu & msg)
{
  tf2::Quaternion quat;
  quat.setX(msg.orientation.x);
  quat.setY(msg.orientation.y);
  quat.setZ(msg.orientation.z);
  quat.setW(msg.orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  const auto stamp = stamp_from_msg_or_now(msg.header);

  // Legacy Att: internal convention (pitch/yaw negated) already applied here.
  auto message = interfaces::msg::Att();
  message.roll = addGaussianNoise(roll, PAR.Sigma_roll);
  message.pitch = addGaussianNoise(-pitch, PAR.Sigma_pitch);
  message.yaw = addGaussianNoise(-yaw + 0.00, PAR.Sigma_yaw);
  message.time =
    static_cast<int32_t>(stamp.sec) * 1000 + static_cast<int32_t>(stamp.nanosec / 1000000);
  pub_Att_->publish(message);

  // Standard Imu: pass-through orientation (and rest of message); Local SLAM
  // applies the same pitch/yaw flips at its boundary in standard mode.
  // Apply the same attitude noise in the quaternion domain approximately by
  // rebuilding from the noisy Euler that matches legacy Att magnitudes.
  // For parity of noise samples with Att, rebuild from the Att RPY *before*
  // the historical flips: roll_n, -pitch_n_internal, -yaw_n_internal.
  {
    sensor_msgs::msg::Imu imu_out = msg;
    imu_out.header.stamp = stamp;
    // Public frame: driver IMU frame (Webots inertial unit). Coordinates in
    // orientation are that frame's attitude vs Webots world (z-up).
    if (imu_out.header.frame_id.empty()) {
      imu_out.header.frame_id = "imu_link";
    }
    // Noise: rebuild quaternion from the same noisy RPY used for legacy Att
    // (undo internal flips to stay in ROS/Webots RPY).
    const double roll_n = message.roll;
    const double pitch_n = -message.pitch;
    const double yaw_n = -message.yaw;
    tf2::Quaternion qn;
    qn.setRPY(roll_n, pitch_n, yaw_n);
    imu_out.orientation.x = qn.x();
    imu_out.orientation.y = qn.y();
    imu_out.orientation.z = qn.z();
    imu_out.orientation.w = qn.w();
    pub_Imu_std_->publish(imu_out);
  }

  // Bounded IMU yaw history for optional diagnostics (GT yaw uses SLAM's
  // yaw_enu_equivalent from EstimatorReference, not live IMU).
  {
    std::lock_guard<std::mutex> lock(gt_mutex_);
    gt::ImuYawSample s;
    s.t_sec = gt::stamp_to_sec(stamp);
    s.yaw_enu = yaw;
    if (!imu_yaw_hist_.empty() && s.t_sec + 1e-9 < imu_yaw_hist_.back().t_sec) {
      // Backward clock jump / sim restart: drop history.
      imu_yaw_hist_.clear();
      gps_hist_.clear();
      if (gt_ref_.resolved) {
        invalidate_gt_epoch("IMU stamp went backwards");
      }
    }
    imu_yaw_hist_.push_back(s);
    gt::trim_history(imu_yaw_hist_, gt_history_duration_sec_);
  }

  // Live Rn2rG for GPS-speed → body only (must track current attitude).
  {
    double Ra2b[9];
    const double yaw_live = restrictAngle(yaw);
    Euler_to_Ra2b_colum_major(roll, pitch, yaw_live, Ra2b);
    arma::mat Rn2r(Ra2b, 3, 3);
    Rn2rG = Rn2r;
  }
}

sensor_msgs::msg::CameraInfo WEBOTb::make_camera_info(
  const std_msgs::msg::Header & header) const
{
  sensor_msgs::msg::CameraInfo info;
  info.header = header;
  if (info.header.frame_id.empty()) {
    info.header.frame_id = Camera_optical_frame_;
  }
  info.height = static_cast<uint32_t>(Camera_height_);
  info.width = static_cast<uint32_t>(Camera_width_);
  info.distortion_model = "plumb_bob";
  info.d = {Camera_k1_, Camera_k2_, 0.0, 0.0, 0.0};
  // Row-major K: [fx, s, cx; 0, fy, cy; 0, 0, 1]; s = alpha_c * fy
  const double skew = Camera_alpha_c_ * Camera_fc_v_;
  info.k = {
    Camera_fc_u_, skew, Camera_cc_u_,
    0.0, Camera_fc_v_, Camera_cc_v_,
    0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  // P matches K for unrectified monocular (Tx=0).
  info.p = {
    Camera_fc_u_, skew, Camera_cc_u_, 0.0,
    0.0, Camera_fc_v_, Camera_cc_v_, 0.0,
    0.0, 0.0, 1.0, 0.0};
  info.binning_x = 0;
  info.binning_y = 0;
  info.roi.x_offset = 0;
  info.roi.y_offset = 0;
  info.roi.height = 0;
  info.roi.width = 0;
  info.roi.do_rectify = false;
  return info;
}

void WEBOTb::Camera_callback(const sensor_msgs::msg::Image & msg)
{
  auto stamp = stamp_from_msg_or_now(msg.header);

  auto message = interfaces::msg::Frame();
  if (Range_available == true) {
    message.range = Range;
    Range_available = false;
  } else {
    message.range = -1;
  }
  message.img = msg;
  message.img.header.stamp = stamp;
  message.time =
    static_cast<int64_t>(stamp.sec) * 1000 + static_cast<int64_t>(stamp.nanosec / 1000000);
  pub_Frame_->publish(message);

  sensor_msgs::msg::Image img_out = msg;
  img_out.header.stamp = stamp;
  // Optical frame as published by Webots camera driver / bridge params.
  if (img_out.header.frame_id.empty()) {
    img_out.header.frame_id = Camera_optical_frame_;
  }
  pub_Image_std_->publish(img_out);

  if (Publish_camera_info_) {
    // Dimensions/intrinsics come from bridge params (match localslam Mono_cam_*);
    // if the driver image size differs, localslam camera_info mode rejects it.
    pub_CameraInfo_std_->publish(make_camera_info(img_out.header));
  }
}

void WEBOTb::Range_callback(const sensor_msgs::msg::Range & msg)
{
  Range = msg.range;
  Range_available = true;

  auto stamp = stamp_from_msg_or_now(msg.header);

  auto message = interfaces::msg::Range();
  message.range = msg.range;
  message.time =
    static_cast<int32_t>(stamp.sec) * 1000 + static_cast<int32_t>(stamp.nanosec / 1000000);
  pub_Range_->publish(message);

  sensor_msgs::msg::Range range_out = msg;
  range_out.header.stamp = stamp;
  // Ranger frame: Webots ds0 (+X along beam / sensor axis).
  if (range_out.header.frame_id.empty()) {
    range_out.header.frame_id = "downward_ranger_link";
  }
  pub_DownwardRange_std_->publish(range_out);
}

void WEBOTb::invalidate_gt_epoch(const std::string & reason)
{
  gt::clear_reference(gt_ref_);
  gt_ref_.unresolved_reason = reason;
  gt_path_.poses.clear();
  gt_path_epoch_id_ = 0;
  publish_empty_gt_path(now_stamp());
  RCLCPP_WARN(this->get_logger(), "GT epoch invalidated: %s", reason.c_str());
}

void WEBOTb::publish_empty_gt_path(const builtin_interfaces::msg::Time & stamp)
{
  gt_path_.header.stamp = stamp;
  gt_path_.header.frame_id = gt_path_frame_id_;
  gt_path_.poses.clear();
  if (pub_GT_path_) {
    pub_GT_path_->publish(gt_path_);
  }
}

void WEBOTb::append_gt_path_pose(
  const builtin_interfaces::msg::Time & stamp,
  double x_pub, double y_pub, double z_pub)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = stamp;
  ps.header.frame_id = gt_path_frame_id_;
  ps.pose.position.x = x_pub;
  ps.pose.position.y = y_pub;
  ps.pose.position.z = z_pub;
  // Orientation intentionally unspecified: identity quaternion.
  ps.pose.orientation.w = 1.0;
  ps.pose.orientation.x = 0.0;
  ps.pose.orientation.y = 0.0;
  ps.pose.orientation.z = 0.0;
  gt_path_.header.stamp = stamp;
  gt_path_.header.frame_id = gt_path_frame_id_;
  gt_path_.poses.push_back(ps);
  while (static_cast<int>(gt_path_.poses.size()) > gt_path_max_poses_) {
    gt_path_.poses.erase(gt_path_.poses.begin());
  }
  pub_GT_path_->publish(gt_path_);
}

void WEBOTb::try_resolve_gt_reference()
{
  if (!have_ref_msg_ || !last_ref_msg_.valid || !last_ref_msg_.heading_valid) {
    return;
  }
  if (last_ref_msg_.altitude_update_enabled && !last_ref_msg_.altitude_valid) {
    gt_ref_.resolved = false;
    gt_ref_.unresolved_reason = "awaiting altitude home latch";
    return;
  }

  std::string reason;
  const double t_heading = gt::stamp_to_sec(last_ref_msg_.heading_stamp);
  gt::GpsSample gps_h;
  if (!gt::interpolate_gps(gps_hist_, t_heading, gt_match_max_dt_sec_, gps_h, reason)) {
    gt_ref_.resolved = false;
    gt_ref_.unresolved_reason = "heading GPS: " + reason;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "GT unresolved: %s", gt_ref_.unresolved_reason.c_str());
    return;
  }

  double origin_z = gps_h.z;
  if (last_ref_msg_.altitude_update_enabled && last_ref_msg_.altitude_valid) {
    const double t_alt = gt::stamp_to_sec(last_ref_msg_.altitude_stamp);
    gt::GpsSample gps_a;
    if (!gt::interpolate_gps(gps_hist_, t_alt, gt_match_max_dt_sec_, gps_a, reason)) {
      gt_ref_.resolved = false;
      gt_ref_.unresolved_reason = "altitude GPS: " + reason;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "GT unresolved: %s", gt_ref_.unresolved_reason.c_str());
      return;
    }
    origin_z = gps_a.z;
  }

  gt_ref_.resolved = true;
  gt_ref_.epoch_id = last_ref_msg_.epoch_id;
  gt_ref_.yaw_enu = last_ref_msg_.yaw_enu_equivalent;
  gt_ref_.origin_xy[0] = gps_h.x;
  gt_ref_.origin_xy[1] = gps_h.y;
  gt_ref_.origin_xy[2] = gps_h.z;
  gt_ref_.origin_z = origin_z;
  gt_ref_.robot_init_internal[0] = last_ref_msg_.robot_init_x;
  gt_ref_.robot_init_internal[1] = last_ref_msg_.robot_init_y;
  gt_ref_.robot_init_internal[2] = last_ref_msg_.robot_init_z;
  gt_ref_.unresolved_reason.clear();

  if (gt_path_epoch_id_ != gt_ref_.epoch_id) {
    gt_path_.poses.clear();
    gt_path_epoch_id_ = gt_ref_.epoch_id;
    publish_empty_gt_path(now_stamp());
  }

  RCLCPP_INFO(
    this->get_logger(),
    "GT resolved epoch_id=%llu yaw_enu=%.4f origin_xy=(%.4f,%.4f) origin_z=%.4f "
    "robot_init=(%.3f,%.3f,%.3f)",
    static_cast<unsigned long long>(gt_ref_.epoch_id),
    gt_ref_.yaw_enu,
    gt_ref_.origin_xy[0], gt_ref_.origin_xy[1], gt_ref_.origin_z,
    gt_ref_.robot_init_internal[0], gt_ref_.robot_init_internal[1],
    gt_ref_.robot_init_internal[2]);
}

void WEBOTb::EstimatorReference_callback(
  const interfaces::msg::EstimatorReference & msg)
{
  std::lock_guard<std::mutex> lock(gt_mutex_);
  if (!msg.valid) {
    have_ref_msg_ = true;
    last_ref_msg_ = msg;
    invalidate_gt_epoch("EstimatorReference valid=false");
    return;
  }
  // New session (component recreation), new epoch, or metadata update
  // (e.g. altitude became valid within the same session/epoch).
  if (have_ref_msg_ && last_ref_msg_.valid) {
    const bool session_changed =
      last_ref_msg_.session_id.uuid != msg.session_id.uuid;
    if (session_changed) {
      invalidate_gt_epoch("new EstimatorReference session_id");
    } else if (last_ref_msg_.epoch_id != msg.epoch_id) {
      invalidate_gt_epoch("new EstimatorReference epoch_id");
    }
  }
  have_ref_msg_ = true;
  last_ref_msg_ = msg;
  try_resolve_gt_reference();
}

void WEBOTb::Gps_callback(const geometry_msgs::msg::PointStamped & msg)
{
  double x = msg.point.x;
  double y = msg.point.y;
  double z = msg.point.z;

  auto stamp = stamp_from_msg_or_now(msg.header);
  const double t_sec = gt::stamp_to_sec(stamp);

  {
    std::lock_guard<std::mutex> lock(gt_mutex_);
    if (!gps_hist_.empty() && t_sec + 1e-9 < gps_hist_.back().t_sec) {
      gps_hist_.clear();
      imu_yaw_hist_.clear();
      if (gt_ref_.resolved) {
        invalidate_gt_epoch("GPS stamp went backwards");
      }
    }
    gt::GpsSample s;
    s.t_sec = t_sec;
    s.x = x;
    s.y = y;
    s.z = z;
    gps_hist_.push_back(s);
    gt::trim_history(gps_hist_, gt_history_duration_sec_);

    if (!gt_ref_.resolved) {
      try_resolve_gt_reference();
    }

    if (gt_ref_.resolved) {
      // Drop samples that predate the altitude reference (older epoch leak).
      const double t_alt = last_ref_msg_.altitude_valid
        ? gt::stamp_to_sec(last_ref_msg_.altitude_stamp)
        : gt::stamp_to_sec(last_ref_msg_.heading_stamp);
      if (t_sec + 1e-9 >= t_alt) {
        double xp = 0.0, yp = 0.0, zp = 0.0;
        gt::webots_gps_to_public_gt(x, y, z, gt_ref_, xp, yp, zp);
        auto message_g = geometry_msgs::msg::PointStamped();
        message_g.header.stamp = stamp;
        message_g.header.frame_id = gt_path_frame_id_;
        message_g.point.x = xp;
        message_g.point.y = yp;
        message_g.point.z = zp;
        pub_GT_->publish(message_g);
        append_gt_path_pose(stamp, xp, yp, zp);
      }
    }
  }

  // Emulate altitude: GPS height + noise (unchanged). Positive up.
  const double alt_meas = addGaussianNoise(z, PAR.Sigma_Alt);

  auto message = interfaces::msg::Alt();
  message.altitude = alt_meas;
  message.time =
    static_cast<int32_t>(stamp.sec) * 1000 + static_cast<int32_t>(stamp.nanosec / 1000000);
  pub_Alt_->publish(message);

  // mavros Altitude: relative carries the same derived-height observation.
  // Datum: Webots GPS z (+up) with Sigma_Alt noise. Home latch remains in EKF.
  // Unavailable MAVLink fields → NaN (do not invent AMSL/terrain).
  const float nan_f = std::numeric_limits<float>::quiet_NaN();
  mavros_msgs::msg::Altitude alt_std;
  alt_std.header.stamp = stamp;
  // relative is vertical height [m], +up, from GPS z (+ noise). Not AGL.
  alt_std.header.frame_id =
    msg.header.frame_id.empty() ? "gps_link" : msg.header.frame_id;
  alt_std.monotonic = nan_f;
  alt_std.amsl = nan_f;
  alt_std.local = nan_f;
  alt_std.relative = static_cast<float>(alt_meas);
  alt_std.terrain = nan_f;
  alt_std.bottom_clearance = nan_f;
  pub_Altitude_std_->publish(alt_std);

  // Horizontal range-to-base (custom, temporary in A1)
  double range = std::sqrt(
    std::pow(x - PAR.Base_location[0], 2) + std::pow(y - PAR.Base_location[1], 2));
  auto message_r = interfaces::msg::Rangetobase();
  message_r.range = addGaussianNoise(range, PAR.Sigma_rangetobase);
  message_r.time = message.time;
  pub_RangeToBase_->publish(message_r);
}

void WEBOTb::GpsSpeed_callback(const geometry_msgs::msg::Vector3 & msg)
{
  // Headerless input: stamp with node clock (sim time if use_sim_time).
  const auto stamp = now_stamp();
  const int32_t time_ms =
    static_cast<int32_t>(stamp.sec) * 1000 + static_cast<int32_t>(stamp.nanosec / 1000000);

  arma::vec sp_N = {msg.x, msg.y, msg.z};
  arma::vec sp_R = Rn2rG * sp_N;

  // Shared noise samples: ROS body = sp_R; legacy Spd applies y/z flips.
  const double vx = addGaussianNoise(sp_R[0], PAR.Sigma_Spd);
  const double vy_ros = addGaussianNoise(sp_R[1], PAR.Sigma_Spd);
  const double vz_ros = addGaussianNoise(sp_R[2], PAR.Sigma_Spd);

  auto message = interfaces::msg::Spd();
  message.speed_x = vx;
  message.speed_y = -vy_ros;
  message.speed_z = -vz_ros;
  message.time = time_ms;
  pub_Spd_->publish(message);

  geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
  twist_msg.header.stamp = stamp;
  // Body frame after Rn2r * GPS speed (Webots body aligned with IMU),
  // BEFORE legacy Spd (y,z) sign flips. Local SLAM applies those flips.
  twist_msg.header.frame_id = "base_link";
  twist_msg.twist.twist.linear.x = vx;
  twist_msg.twist.twist.linear.y = vy_ros;
  twist_msg.twist.twist.linear.z = vz_ros;
  // Covariance unknown → leave zeros (Local SLAM ignores msg cov; uses YAML).
  pub_BodyVel_std_->publish(twist_msg);
}

double WEBOTb::restrictAngle(double angle)
{
  angle = fmod(angle, 2 * M_PI);
  if (angle <= -M_PI) {
    angle += 2 * M_PI;
  } else if (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  return angle;
}

}  // namespace webotbridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(webotbridge::WEBOTb)
