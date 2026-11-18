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

#ifndef COMPOSITION__CONTROL_COMPONENT_HPP_
#define COMPOSITION__CONTROL_COMPONENT_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "interfaces/srv/simple_serv.hpp"
#include "interfaces/srv/unified_trajectory.hpp"
#include "interfaces/srv/correction_coordinate.hpp"
#include "interfaces/msg/unified_trajectory_status.hpp"
#include "g6_coord_policy.hpp"
#include "g6_feedback_barrier.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include "interfaces/msg/alt.hpp"
#include "interfaces/msg/att.hpp"
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/range.hpp"
#include "interfaces/msg/gps.hpp"
#include "interfaces/msg/spd.hpp"
#include "interfaces/msg/ododiff.hpp"
#include "interfaces/msg/odovw.hpp"
#include "interfaces/msg/robotstate.hpp"
#include "interfaces/msg/correction_applied.hpp"
#include "interfaces/msg/global_loop_committed.hpp"
#include "interfaces/msg/odometry_meta.hpp"
#include "interfaces/msg/correction_feedback_accepted.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "parameters.hpp"
#include "../../localslam/src/localslam_types.hpp"
#include "../../common/Vision/vision.hpp"
#include <opencv2/aruco.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include "control_types.hpp"
#include "logger.hpp"

namespace control
{

  class ControlQuad : public rclcpp::Node
  {
  public:
    COMPOSITION_PUBLIC
    explicit ControlQuad(const rclcpp::NodeOptions & options);

    ~ControlQuad();

  protected:
    //void on_timer();

  private:
    
    parameters PAR;
    
    bool actual_z_flag;
    void reinit_control();

    thread control_loop_;
    void Control_Loop();
    bool control_thread_loop_run;

    

    std::mutex mutex_webots_state;
    std::mutex mutex_slam_state;
    std::mutex mutex_robot_state;
    std::mutex mutex_cl;
    std::mutex mutex_frame;
    cv::Mat frame;
    bool cl_flag;
    /** G6d: last GlobalLoopCommitted (informational; not a re-nav trigger). */
    bool global_loop_committed_seen_{false};
    std::string global_loop_committed_source_;
    uint64_t global_loop_committed_revision_{0};

    ROBOT_STATE r_c;  // robot feedback state
    ROBOT_STATE r_a;  // robot actual state (from webots)
    ROBOT_STATE r_s;  // robot estimated state (from SLAM)
    
    
    // declare subscribers
    
    // webot buscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_Imu_;    
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_Gps_;

    rclcpp::Subscription<interfaces::msg::Robotstate>::SharedPtr sub_Slam_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_Odom_;
    rclcpp::Subscription<interfaces::msg::OdometryMeta>::SharedPtr sub_OdomMeta_;
    rclcpp::Subscription<interfaces::msg::CorrectionApplied>::SharedPtr sub_CorrectionApplied_;
    /** G6d informational: global map loop commit (never sets cl_flag). */
    rclcpp::Subscription<interfaces::msg::GlobalLoopCommitted>::SharedPtr
      sub_GlobalLoopCommitted_;

    rclcpp::Subscription<interfaces::msg::Frame>::SharedPtr sub_Frame_;

    // declare publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<interfaces::msg::CorrectionFeedbackAccepted>::SharedPtr
      pub_correction_feedback_accepted_;

    // declare Services
      rclcpp::Service<interfaces::srv::SimpleServ>::SharedPtr srv_control_run_;
      void Handle_control_run_service(const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,std::shared_ptr<interfaces::srv::SimpleServ::Response> response);
      rclcpp::Service<interfaces::srv::CorrectionCoordinate>::SharedPtr srv_correction_coord_;
      void Handle_correction_coordinate(
        const std::shared_ptr<interfaces::srv::CorrectionCoordinate::Request> request,
        std::shared_ptr<interfaces::srv::CorrectionCoordinate::Response> response);
      void publish_zero_twist();
      bool wait_cmd_quiescent(std::chrono::milliseconds timeout);

    // declare clients
      rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_ekf_run_;
      rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_ekf_enable_;
      rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_gslam_run_;
      rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_webot_run_;
      rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_plot_;
      rclcpp::Client<interfaces::srv::UnifiedTrajectory>::SharedPtr
        client_unified_traj_;
      rclcpp::Subscription<interfaces::msg::UnifiedTrajectoryStatus>::SharedPtr
        sub_unified_traj_status_;

    //------   
    void setParameters();
    void setSubscribers();
    std::mutex mutex_run_plan;
    bool run_control_plan;
    // G6a coordination (separate from legacy SimpleServ 'c' / cl_flag).
    std::mutex mutex_coord_;
    g6_coord::CoordSnapshot coord_snap_;
    std::atomic<bool> coord_suppress_cmd_{false};
    std::atomic<bool> in_plan_command_{false};
    std::condition_variable cv_coord_quiet_;
    
    // control plan parser
    int cmd_idx; // command (instruction index)
    void load_control_plan();
    
    vector<CMD> Control_plan;
    bool execute_cmd();

    bool go_point(ROBOT_STATE &r_d);
    /** R4d: request acknowledged unified traj and wait for matching settle. */
    bool go_point_unified(ROBOT_STATE &r_d);
    void UnifiedTrajStatus_callback(
      const interfaces::msg::UnifiedTrajectoryStatus & msg);
    bool cancel_unified_trajectory();
    bool go_point_using_visual_mark(ROBOT_STATE &r_d);
    double compute_ys(double lambda_y);
    bool recognize_home();
    bool explore_area(double x_a,double y_a);
    bool go_home();

    void Imu_callback(const sensor_msgs::msg::Imu & msg);
    void Gps_callback(const geometry_msgs::msg::PointStamped & msg);
    void Slam_callback(const interfaces::msg::Robotstate & msg);
    /** A5b odometry mode: Robotstate.radius_uncertainty only (no pose). */
    void Slam_radius_only_callback(const interfaces::msg::Robotstate & msg);
    void Odometry_callback(const nav_msgs::msg::Odometry & msg);
    void OdometryMeta_callback(const interfaces::msg::OdometryMeta & msg);
    void CorrectionApplied_feedback_callback(
      const interfaces::msg::CorrectionApplied & msg);
    void GlobalLoopCommitted_callback(
      const interfaces::msg::GlobalLoopCommitted & msg);
    void Frame_callback(const interfaces::msg::Frame & msg);
    void clear_feedback_barrier_locked();
    void publish_feedback_accepted_locked();
    void try_pair_odometry_meta(int32_t stamp_sec, uint32_t stamp_nanosec);
    void note_accepted_snapshot_locked(const g6_feedback::MetaSample & meta);

    // Odometry / legacy SLAM yaw latch (public or internal per interface)
    bool slam_yaw_init_flag_{false};
    double slam_init_yaw_{0.0};
    double slam_init_z_{0.0};

    // A6: radius accept gate for coordinated reset (odometry-mode radius-only).
    // While false, Slam_radius_only_callback drops messages (including queued
    // pre-reset callbacks). Re-armed on the next accepted Odometry pose update
    // whose header.stamp is not older than radius_reset_stamp_ (rejects queued
    // pre-reset Odometry that would otherwise re-arm and admit stale radius).
    bool radius_feedback_armed_{false};
    bool radius_reset_stamp_valid_{false};
    rclcpp::Time radius_reset_stamp_{0, 0, RCL_ROS_TIME};
    void invalidate_cached_radius(bool request_ekf_reinit);

    // G6b feedback barrier (mutex_coord_ guards barrier state with pause).
    g6_feedback::PendingBarrier feedback_barrier_;
    std::deque<g6_feedback::MetaSample> odom_meta_buf_;
    std::deque<std::pair<int32_t, uint32_t>> odom_stamp_buf_;
    std::deque<g6_feedback::AcceptedSample> accepted_snap_hist_;
    /** True after Odometry_callback accepted a pose for this stamp (pre-meta). */
    std::deque<std::pair<int32_t, uint32_t>> accepted_odom_stamps_;

    void SaveScreenShot();
    
    double restrictAngle(double angle);
    
    CAM cam_parameters;
    arma::vec::fixed<3> t_c2r;  // vector defining the position of the camara respect to the robot frame 
    arma::mat::fixed<3,3> Rr2c; // Robot to camera rotation matrix
   
    LoggerGoPoint *logger_gp; // for loggind go-point command data

    LoggerGoPoint *logger_vp; // for loggind go-point command data

    arma::mat Rn2rG;

    // CTRL-R2: continuous acceleration-limited XY velocity reference (body).
    double v_ref_x_{0.0};
    double v_ref_y_{0.0};

    // CTRL-R4d: absolute GPS ENU spawn (raw, no Y negation) for mission→driver.
    bool gps_spawn_valid_{false};
    double gps_spawn_x_{0.0};
    double gps_spawn_y_{0.0};
    double gps_spawn_z_{0.0};
    uint32_t unified_traj_req_seq_{0};
    std::mutex mutex_traj_status_;
    interfaces::msg::UnifiedTrajectoryStatus traj_status_{};
    bool traj_status_fresh_{false};
  };

}  // namespace composition

#endif  // COMPOSITION__TALKER_COMPONENT_HPP_