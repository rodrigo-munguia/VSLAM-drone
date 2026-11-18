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

#ifndef COMPOSITION__WEBOTb_COMPONENT_HPP_
#define COMPOSITION__WEBOTb_COMPONENT_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/srv/simple_serv.hpp"
#include "interfaces/msg/alt.hpp"
#include "interfaces/msg/att.hpp"
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/range.hpp"
#include "interfaces/msg/gps.hpp"
#include "interfaces/msg/spd.hpp"
#include "interfaces/msg/ododiff.hpp"
#include "interfaces/msg/odovw.hpp"
#include "interfaces/msg/rangetobase.hpp"
#include "interfaces/msg/estimator_reference.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "mavros_msgs/msg/altitude.hpp"

#include "parameters.hpp"
#include "gt_frame_conversions.hpp"
#include <armadillo>
#include <deque>
#include <mutex>

namespace webotbridge
{

class WEBOTb : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit WEBOTb(const rclcpp::NodeOptions & options);

  ~WEBOTb();

private:
  parameters PAR;
  void loop();
  void setParameters();
  double addGaussianNoise(double value, double stdDev);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_Imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_Camera_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_Range_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_LaserRange_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_Gps_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_GpsSpeed_;
  rclcpp::Subscription<interfaces::msg::EstimatorReference>::SharedPtr sub_EstimatorRef_;

  rclcpp::Publisher<interfaces::msg::Alt>::SharedPtr pub_Alt_;
  rclcpp::Publisher<interfaces::msg::Att>::SharedPtr pub_Att_;
  rclcpp::Publisher<interfaces::msg::Gps>::SharedPtr pub_Gps_;
  rclcpp::Publisher<interfaces::msg::Range>::SharedPtr pub_Range_;
  rclcpp::Publisher<interfaces::msg::Frame>::SharedPtr pub_Frame_;
  rclcpp::Publisher<interfaces::msg::Spd>::SharedPtr pub_Spd_;
  rclcpp::Publisher<interfaces::msg::Ododiff>::SharedPtr pub_OdoD_;
  rclcpp::Publisher<interfaces::msg::Odovw>::SharedPtr pub_OdoV_;
  rclcpp::Publisher<interfaces::msg::Rangetobase>::SharedPtr pub_RangeToBase_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_GT_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_GT_path_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_Image_std_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_CameraInfo_std_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_Imu_std_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_BodyVel_std_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_DownwardRange_std_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_LaserRange_std_;
  rclcpp::Publisher<mavros_msgs::msg::Altitude>::SharedPtr pub_Altitude_std_;

  sensor_msgs::msg::Image::SharedPtr img_msg;
  rclcpp::TimerBase::SharedPtr timer_;

  void Imu_callback(const sensor_msgs::msg::Imu & msg);
  void Camera_callback(const sensor_msgs::msg::Image & msg);
  void Range_callback(const sensor_msgs::msg::Range & msg);
  void LaserRange_callback(const sensor_msgs::msg::Range & msg);
  void Gps_callback(const geometry_msgs::msg::PointStamped & msg);
  void GpsSpeed_callback(const geometry_msgs::msg::Vector3 & msg);
  void EstimatorReference_callback(const interfaces::msg::EstimatorReference & msg);
  sensor_msgs::msg::CameraInfo make_camera_info(
    const std_msgs::msg::Header & header) const;

  void try_resolve_gt_reference();
  void invalidate_gt_epoch(const std::string & reason);
  void publish_empty_gt_path(const builtin_interfaces::msg::Time & stamp);
  void append_gt_path_pose(
    const builtin_interfaces::msg::Time & stamp,
    double x_pub, double y_pub, double z_pub);

  bool Range_available;
  double Range;
  bool Publish_camera_info_{true};
  int Camera_width_{400};
  int Camera_height_{240};
  double Camera_fc_u_{200.0};
  double Camera_fc_v_{200.0};
  double Camera_cc_u_{200.0};
  double Camera_cc_v_{120.0};
  double Camera_alpha_c_{0.0};
  double Camera_k1_{0.0};
  double Camera_k2_{0.0};
  std::string Camera_optical_frame_{"camera_optical_frame"};

  double restrictAngle(double angle);
  arma::mat Rn2rG;

  std::mutex gt_mutex_;
  std::deque<gt::GpsSample> gps_hist_;
  std::deque<gt::ImuYawSample> imu_yaw_hist_;
  interfaces::msg::EstimatorReference last_ref_msg_;
  bool have_ref_msg_{false};
  gt::GtReference gt_ref_;
  nav_msgs::msg::Path gt_path_;
  uint64_t gt_path_epoch_id_{0};

  double gt_history_duration_sec_{5.0};
  double gt_match_max_dt_sec_{0.15};
  int gt_path_max_poses_{2000};
  std::string gt_path_frame_id_{"odom"};

  builtin_interfaces::msg::Time stamp_from_msg_or_now(
    const std_msgs::msg::Header & header) const;
  builtin_interfaces::msg::Time now_stamp() const;
};

}  // namespace webotbridge

#endif  // COMPOSITION__WEBOTb_COMPONENT_HPP_
