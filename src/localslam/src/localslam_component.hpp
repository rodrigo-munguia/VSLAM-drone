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

#ifndef COMPOSITION__LOCALSLAM_COMPONENT_HPP_
#define COMPOSITION__LOCALSLAM_COMPONENT_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "interfaces/msg/alt.hpp"
#include "interfaces/msg/att.hpp"
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/range.hpp"
#include "interfaces/msg/gps.hpp"
#include "interfaces/msg/spd.hpp"
#include "interfaces/msg/ododiff.hpp"
#include "interfaces/msg/odovw.hpp"
#include "interfaces/msg/kf.hpp"
#include "interfaces/msg/robotstate.hpp"
#include "interfaces/msg/rangetobase.hpp"
#include "interfaces/msg/correction_applied.hpp"
#include "interfaces/msg/odometry_meta.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "camera_calibration.hpp"
#include "pressure_altitude.hpp"
#include <deque>
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "mavros_msgs/msg/altitude.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "localslam_types.hpp"
#include "ekf_types.hpp"
#include <condition_variable>
#include <mutex>
#include "parameters.hpp"
#include "interfaces/srv/simple_serv.hpp"
#include "interfaces/srv/l_spos_update.hpp"
#include "interfaces/msg/lslam.hpp"
#include "interfaces/msg/estimator_reference.hpp"
#include "interfaces/msg/correction_applied.hpp"
#include "../../common/keyframe_types.hpp"
#include "../../common/slam_identity.hpp"
#include <unordered_set>


using namespace std;

class EKF;  // defined in ekf.hpp (global); not localslam::EKF

namespace localslam
{

class EKFslam : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit EKFslam(const rclcpp::NodeOptions & options);

  ~EKFslam();

private:
  // declare subscribers (legacy)
  rclcpp::Subscription<interfaces::msg::Alt>::SharedPtr sub_Alt_;
  rclcpp::Subscription<interfaces::msg::Att>::SharedPtr sub_Att_;
  rclcpp::Subscription<interfaces::msg::Gps>::SharedPtr sub_Gps_;
  rclcpp::Subscription<interfaces::msg::Range>::SharedPtr sub_Range_;
  rclcpp::Subscription<interfaces::msg::Frame>::SharedPtr sub_Frame_;
  rclcpp::Subscription<interfaces::msg::Spd>::SharedPtr sub_Spd_;
  rclcpp::Subscription<interfaces::msg::Ododiff>::SharedPtr sub_OdoD_;
  rclcpp::Subscription<interfaces::msg::Odovw>::SharedPtr sub_OdoV_;
  rclcpp::Subscription<interfaces::msg::Rangetobase>::SharedPtr sub_Rangetobase_;

  // declare subscribers (standard / A1)
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_Image_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_Imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_BodyVel_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_DownwardRange_;
  rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr sub_MavrosAlt_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr sub_Pressure_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_CameraInfo_;

  // declare callbacks (legacy)
  void Alt_callback(const interfaces::msg::Alt & msg);
  void Att_callback(const interfaces::msg::Att & msg);
  void Gps_callback(const interfaces::msg::Gps & msg);
  void Range_callback(const interfaces::msg::Range & msg);
  void Frame_callback(const interfaces::msg::Frame & msg);
  void Speed_callback(const interfaces::msg::Spd & msg);
  void OdoDiff_callback(const interfaces::msg::Ododiff &msg);
  void OdoVW_callback(const interfaces::msg::Odovw &msg);
  void Rangetobase_callback(const interfaces::msg::Rangetobase &msg);

  // declare callbacks (standard)
  void Image_std_callback(const sensor_msgs::msg::Image & msg);
  void Imu_std_callback(const sensor_msgs::msg::Imu & msg);
  void BodyVelocity_std_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped & msg);
  void DownwardRange_std_callback(const sensor_msgs::msg::Range & msg);
  void Altitude_std_callback(const mavros_msgs::msg::Altitude & msg);
  void FluidPressure_std_callback(const sensor_msgs::msg::FluidPressure & msg);
  void CameraInfo_callback(const sensor_msgs::msg::CameraInfo & msg);
  void reset_pressure_altitude_converter();
  void clear_pending_altitude_observations();

  // Shared enqueue into the existing observation buffer / EKF path
  void enqueue_observation(const DATA & dat);
  /** Convert Image → DATA and enqueue, or bound-wait if CameraInfo not ready. */
  void handle_visual_image(const sensor_msgs::msg::Image & msg);
  void flush_waiting_images();
  bool visual_calibration_ready() const;
  void apply_intrinsics_to_parameters(const camera_cal::Intrinsics & intr);

  // declare Services
  rclcpp::Service<interfaces::srv::SimpleServ>::SharedPtr srv_ekf_run_;
  void Handle_ekf_run_service(
    const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,
    std::shared_ptr<interfaces::srv::SimpleServ::Response> response);
  rclcpp::Service<interfaces::srv::LSposUpdate>::SharedPtr srv_pos_update_;
  void Handle_pos_update_service(
    const std::shared_ptr<interfaces::srv::LSposUpdate::Request> request,
    std::shared_ptr<interfaces::srv::LSposUpdate::Response> response);

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
  void Handle_enable_service(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void Handle_reset_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Shared control used by legacy SimpleServ and standard services
  void set_ekf_steps_running(bool running);
  void request_ekf_reinit();
  void request_log_stats();
  void toggle_cl_uncertainty();
  void request_activate_visual_update();

  // declare publishers
  rclcpp::Publisher<interfaces::msg::Lslam>::SharedPtr pub_lslam_data_;
  rclcpp::Publisher<interfaces::msg::Kf>::SharedPtr pub_kf_;
  rclcpp::Publisher<interfaces::msg::Kf>::SharedPtr pub_kf_cl_;
  rclcpp::Publisher<interfaces::msg::Robotstate>::SharedPtr pub_robot_state_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_landmarks_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  /** Latched estimator reference epoch for GT adapters (TRANSIENT_LOCAL). */
  rclcpp::Publisher<interfaces::msg::EstimatorReference>::SharedPtr pub_estimator_ref_;
  /** G6a: latched notice after applied mutation + odom publish (feedback barrier). */
  rclcpp::Publisher<interfaces::msg::CorrectionApplied>::SharedPtr pub_correction_applied_;
  /** G6b: unique snapshot association companion to ~/odometry. */
  rclcpp::Publisher<interfaces::msg::OdometryMeta>::SharedPtr pub_odometry_meta_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Bounded trajectory from accepted A2 odometry snapshots (EKF worker only).
  nav_msgs::msg::Path path_msg_;

  DATA getData();
  vector<DATA> DataBuffer;
  void ManageBuffer(DATA &dat);
  mutable std::mutex mutex_dat;

  // LSLAM-PERF-W1: DataBuffer / image callback counters (env-gated profiling).
  uint64_t diag_frames_callback_rx_{0};
  uint64_t diag_frames_enqueued_{0};
  uint64_t diag_frames_overwritten_{0};
  uint64_t diag_frames_consumed_{0};

  thread ekf_loop_;
  void EKF_LOOP();
  bool ekf_thread_loop_run;
  bool ekf_steps_loop_run;
  std::mutex mutex_ekf_run;
  std::mutex mutex_get_state;
  std::mutex mutex_init_system;
  std::mutex mutex_pos_update;
  std::condition_variable cv_pos_update_;
  std::mutex mutex_log;
  std::mutex mutex_ru;
  std::mutex mutex_vu;
  std::mutex mutex_cal_;

  bool re_init_sys;
  bool new_slam_state_flag;
  bool get_log_flag;
  bool activate_CL_uncertainty;
  bool activate_CL_uncertainty_flag;
  bool activate_ekf_visual_update_flag;

  // A8 calibration state (camera_info mode)
  bool calibration_ready_{false};
  bool camera_cal_pending_apply_{false};
  bool allow_new_geometry_after_reset_{true};
  bool map_has_visual_state_{false};  // set true after feats exist; cleared on reinit
  camera_cal::Intrinsics applied_intrinsics_;
  std::deque<sensor_msgs::msg::Image> waiting_images_;
  size_t waiting_images_dropped_{0};

  // A9 FluidPressure → relative altitude (ROS boundary only)
  pressure_altitude::PressureToRelativeAltitude pressure_converter_;
  std::mutex mutex_pressure_wait_;
  std::deque<sensor_msgs::msg::FluidPressure> waiting_pressures_;
  size_t waiting_pressures_dropped_{0};
  uint64_t pressure_callback_generation_{0};

  POS_UPDATE pos_update;

  parameters PAR;
  void setParameters();
  void setup_subscriptions();

  void pub_kf(KEYFRAME &KF);
  void pub_kf_cl(KEYFRAME &KF);
  void pub_lslam_Data(LOCALSLAM_DATA &lslam_data);
  void pub_robot_state(arma::vec::fixed<13> &x_r,double &radius_uncertainty);
  /**
   * Publish public odometry + OdometryMeta with a unique snapshot_seq.
   * Returns the snapshot_seq on success, 0 if skipped.
   */
  uint64_t pub_odometry(
    const arma::vec::fixed<13> & x_r,
    const arma::mat::fixed<13, 13> & P_r);
  void pub_landmarks_from_lslam(LOCALSLAM_DATA &lslam_data);
  void pub_empty_landmarks();
  void clear_and_publish_path();

  void publish_estimator_reference_invalid();
  void publish_estimator_reference_from_ekf(
    ::EKF & filter, const DATA & heading_dat);
  void fill_estimator_reference_identity(interfaces::msg::EstimatorReference & msg) const;
  void fill_kf_message(interfaces::msg::Kf & message, KEYFRAME & KF) const;
  void clear_pending_pos_update_locked();
  static builtin_interfaces::msg::Time stamp_from_ms(long int time_ms);

  /** G3: one UUID per component instance; epoch bumps on successful re-init. */
  slam_identity::UuidBytes estimator_session_id_{slam_identity::nil_uuid()};
  uint64_t estimator_epoch_id_{0};
  bool estimator_ref_published_valid_{false};
  long int epoch_heading_time_ms_{0};
  double epoch_yaw_at_home_{0.0};
  std::unordered_set<uint64_t> applied_correction_ids_;
  /** G6b: monotonically increasing; never reused after reset. */
  uint64_t next_odometry_snapshot_seq_{1};
  uint64_t last_odometry_snapshot_seq_{0};
  /** Carried on OdometryMeta until next applied correction or reset. */
  uint64_t last_published_correction_id_{0};

  void log_data(STORE &data);
  void mean_std(std::vector<double> &data, double &mean, double &std, double &sum);
  void log_data_to_file(string file_name,std::pair<std::vector<double>,std::vector<double>> &data);

};

}  // namespace localslam

#endif  // COMPOSITION__LOCALSLAM_COMPONENT_HPP_
