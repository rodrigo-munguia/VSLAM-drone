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

#include "localslam_component.hpp"
#include "visual_profile.hpp"
#include "dynamic_diag.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <cinttypes>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/alt.hpp"
#include "interfaces/msg/att.hpp"
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/range.hpp"
#include "interfaces/msg/gps.hpp"
#include "interfaces/msg/spd.hpp"
#include "interfaces/msg/ododiff.hpp"
#include "interfaces/msg/odovw.hpp"
#include "interfaces/msg/lslam.hpp"
#include "interfaces/msg/kf.hpp"
#include "interfaces/srv/l_spos_update.hpp"
#include "geometry_msgs/msg/point.h"
#include "ekf.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "../../common/Transforms/Ra2b_TO_Quat_a2b.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <stdexcept>
#include <limits>
#include "sensor_input_plan.hpp"
#include "range_frame_association.hpp"
#include "odometry_conversions.hpp"
#include "map_path_conversions.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "camera_calibration.hpp"
#include "ls_pos_update_policy.hpp"
#include "../../common/ros_slam_identity.hpp"

//#include <Matrix3x3.h>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

namespace localslam
{


// Create a Listener "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
EKFslam::EKFslam(const rclcpp::NodeOptions & options)
: Node("lslam", options)
{
  setParameters();

  validate_range_acceptance_params(
    PAR.Range_assoc_min_m, PAR.Range_assoc_max_m, PAR.Range_init_min_m);
  validate_altitude_input_mode(PAR.Altitude_input_mode);
  // Throws on unsupported Sensor_input_mode / altitude combo
  {
    SensorInputPlan plan = make_sensor_input_plan(PAR.Sensor_input_mode);
    apply_altitude_input_mode(plan, PAR.Altitude_input_mode);
  }
  if (PAR.Pressure_reference_policy != "first_valid") {
    throw std::runtime_error(
      "Unsupported Pressure_reference_policy='" + PAR.Pressure_reference_policy +
      "'. Use 'first_valid'.");
  }
  if (PAR.Camera_info_qos_reliability != "reliable" &&
    PAR.Camera_info_qos_reliability != "best_effort")
  {
    throw std::runtime_error(
      "Unsupported Camera_info_qos_reliability='" +
      PAR.Camera_info_qos_reliability + "'. Use 'reliable' or 'best_effort'.");
  }
  {
    pressure_altitude::ConvertConfig cfg;
    cfg.t0_K = PAR.Pressure_t0_K;
    cfg.p_min_Pa = PAR.Pressure_min_Pa;
    cfg.p_max_Pa = PAR.Pressure_max_Pa;
    cfg.z_lever_arm_m = PAR.Pressure_z_lever_arm_m;
    cfg.expected_frame_id = PAR.Pressure_expected_frame_id;
    pressure_converter_.configure(cfg);
  }

  setup_subscriptions();

  // Legacy + standard control services share set_ekf_steps_running / request_ekf_reinit
  srv_ekf_run_ = this->create_service<interfaces::srv::SimpleServ>(
    "ekf_run_service",
    std::bind(&EKFslam::Handle_ekf_run_service, this, _1, _2));
  srv_pos_update_ = this->create_service<interfaces::srv::LSposUpdate>(
    "local_slam_pos_update",
    std::bind(&EKFslam::Handle_pos_update_service, this, _1, _2));
  srv_enable_ = this->create_service<std_srvs::srv::SetBool>(
    "~/enable",
    std::bind(&EKFslam::Handle_enable_service, this, _1, _2));
  srv_reset_ = this->create_service<std_srvs::srv::Trigger>(
    "~/reset",
    std::bind(&EKFslam::Handle_reset_service, this, _1, _2));

  pub_lslam_data_ = create_publisher<interfaces::msg::Lslam>("lslam_data_topic", 10);
  pub_kf_ = create_publisher<interfaces::msg::Kf>("Kf_topic", 10);
  pub_kf_cl_ = create_publisher<interfaces::msg::Kf>("Kf_cl_topic", 10);
  pub_robot_state_ = create_publisher<interfaces::msg::Robotstate>("Robotstate_topic", 10);
  pub_odometry_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10);
  // A4: changing local sparse map (XYZ) and bounded trajectory in odom parent.
  pub_landmarks_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/landmarks", 10);
  pub_path_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
  {
    rclcpp::QoS qos_corr(rclcpp::KeepLast(1));
    qos_corr.reliable();
    qos_corr.transient_local();
    pub_correction_applied_ =
      create_publisher<interfaces::msg::CorrectionApplied>("~/correction_applied", qos_corr);
  }
  pub_odometry_meta_ = create_publisher<interfaces::msg::OdometryMeta>("~/odometry_meta", 10);
  path_msg_.header.frame_id = PAR.Odometry_frame_id;
  {
    rclcpp::QoS qos_ref(rclcpp::KeepLast(1));
    qos_ref.reliable();
    qos_ref.transient_local();
    pub_estimator_ref_ =
      create_publisher<interfaces::msg::EstimatorReference>("~/estimator_reference", qos_ref);
  }

  // G3: session UUID once per component instance; initial invalid reference
  // available before the first successful init (late-subscriber friendly).
  estimator_session_id_ = slam_identity::random_uuid_v4();
  estimator_epoch_id_ = 0;
  publish_estimator_reference_invalid();
  RCLCPP_INFO(
    this->get_logger(),
    "EstimatorReference session_id=%s (new component instance)",
    slam_identity::to_string(estimator_session_id_).c_str());

  // Initialize worker flags before starting EKF_LOOP (avoid publishing on
  // null publishers if ekf_steps_loop_run were left uninitialized).
  new_slam_state_flag = false;
  re_init_sys = false;
  ekf_steps_loop_run = false;
  get_log_flag = false;
  activate_CL_uncertainty = false;
  activate_CL_uncertainty_flag = false;
  activate_ekf_visual_update_flag = false;

  ekf_thread_loop_run = true;
  ekf_loop_ = thread(&EKFslam::EKF_LOOP, this);

  if (PAR.Calibration_source != "yaml" && PAR.Calibration_source != "camera_info") {
    throw std::runtime_error(
      "Unsupported Calibration_source='" + PAR.Calibration_source +
      "'. Use 'yaml' or 'camera_info'.");
  }
  if (PAR.Camera_info_image_path != "raw" && PAR.Camera_info_image_path != "rectified") {
    throw std::runtime_error(
      "Unsupported Camera_info_image_path='" + PAR.Camera_info_image_path +
      "'. Use 'raw' or 'rectified'.");
  }
  // yaml mode: calibration is ready from parameters at startup.
  calibration_ready_ = (PAR.Calibration_source == "yaml");
  allow_new_geometry_after_reset_ = true;

  RCLCPP_INFO(
    this->get_logger(),
    "EKFslam Sensor_input_mode=%s Altitude_input_mode=%s Calibration_source=%s "
    "Camera_info_image_path=%s",
    PAR.Sensor_input_mode.c_str(), PAR.Altitude_input_mode.c_str(),
    PAR.Calibration_source.c_str(), PAR.Camera_info_image_path.c_str());

  std::cout << cv::getBuildInformation() << std::endl;
  arma::arma_version ver;
  std::cout << "ARMA version: " << ver.as_string() << std::endl;
}

void EKFslam::setup_subscriptions()
{
  SensorInputPlan plan = make_sensor_input_plan(PAR.Sensor_input_mode);
  apply_altitude_input_mode(plan, PAR.Altitude_input_mode);

  if (plan.sub_rangetobase) {
    sub_Rangetobase_ = this->create_subscription<interfaces::msg::Rangetobase>(
      "Rangetobase_topic", 10,
      std::bind(&EKFslam::Rangetobase_callback, this, _1));
  }

  if (plan.sub_image_std) {
    sub_Image_ = this->create_subscription<sensor_msgs::msg::Image>(
      "~/image", 10, std::bind(&EKFslam::Image_std_callback, this, _1));
  }
  if (PAR.Calibration_source == "camera_info") {
    // VOLATILE durability (not transient_local). Reliability is configurable.
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.durability_volatile();
    if (PAR.Camera_info_qos_reliability == "best_effort") {
      qos.best_effort();
    } else {
      qos.reliable();
    }
    sub_CameraInfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "~/camera_info", qos,
      std::bind(&EKFslam::CameraInfo_callback, this, _1));
    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed ~/camera_info (Calibration_source=camera_info, "
      "reliability=%s, durability=volatile); "
      "visual frames wait for first valid calibration",
      PAR.Camera_info_qos_reliability.c_str());
  }
  if (plan.sub_imu_std) {
    sub_Imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "~/imu", 10, std::bind(&EKFslam::Imu_std_callback, this, _1));
  }
  if (plan.sub_body_velocity_std) {
    sub_BodyVel_ =
      this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "~/body_velocity", 10,
        std::bind(&EKFslam::BodyVelocity_std_callback, this, _1));
  }
  if (plan.sub_downward_range_std) {
    sub_DownwardRange_ = this->create_subscription<sensor_msgs::msg::Range>(
      "~/downward_range", 10,
      std::bind(&EKFslam::DownwardRange_std_callback, this, _1));
  }
  // Exclusive altitude sources (A9).
  if (plan.sub_altitude_std) {
    sub_MavrosAlt_ = this->create_subscription<mavros_msgs::msg::Altitude>(
      "~/altitude", 10, std::bind(&EKFslam::Altitude_std_callback, this, _1));
    RCLCPP_INFO(
      this->get_logger(),
      "Altitude source: mavros_msgs/Altitude on ~/altitude "
      "(Altitude_input_mode=derived_altitude)");
  }
  if (plan.sub_pressure_std) {
    sub_Pressure_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      "~/pressure", 10,
      std::bind(&EKFslam::FluidPressure_std_callback, this, _1));
    RCLCPP_INFO(
      this->get_logger(),
      "Altitude source: sensor_msgs/FluidPressure on ~/pressure "
      "(Altitude_input_mode=fluid_pressure); "
      "FluidPressure.variance ignored — Sigma_alt_update [m] is authoritative");
  }

  if (plan.sub_alt_legacy) {
    sub_Alt_ = this->create_subscription<interfaces::msg::Alt>(
      "Altitude_topic", 10, std::bind(&EKFslam::Alt_callback, this, _1));
  }
  if (plan.sub_att_legacy) {
    sub_Att_ = this->create_subscription<interfaces::msg::Att>(
      "Attitude_topic", 10, std::bind(&EKFslam::Att_callback, this, _1));
  }
  if (plan.sub_range_legacy) {
    sub_Range_ = this->create_subscription<interfaces::msg::Range>(
      "Range_topic", 10, std::bind(&EKFslam::Range_callback, this, _1));
  }
  if (plan.sub_frame_legacy) {
    sub_Frame_ = this->create_subscription<interfaces::msg::Frame>(
      "Frame_topic", 10, std::bind(&EKFslam::Frame_callback, this, _1));
  }
  if (plan.sub_spd_legacy) {
    sub_Spd_ = this->create_subscription<interfaces::msg::Spd>(
      "Speed_topic", 10, std::bind(&EKFslam::Speed_callback, this, _1));
  }
  if (plan.sub_gps_legacy) {
    sub_Gps_ = this->create_subscription<interfaces::msg::Gps>(
      "Gps_topic", 10, std::bind(&EKFslam::Gps_callback, this, _1));
  }
  if (plan.sub_odo_d_legacy) {
    sub_OdoD_ = this->create_subscription<interfaces::msg::Ododiff>(
      "OdometryD_topic", 10, std::bind(&EKFslam::OdoDiff_callback, this, _1));
  }
  if (plan.sub_odo_v_legacy) {
    sub_OdoV_ = this->create_subscription<interfaces::msg::Odovw>(
      "OdometryV_topic", 10, std::bind(&EKFslam::OdoVW_callback, this, _1));
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Subscription plan: standard_migrated=%d rangetobase=%d "
    "(legacy frame/att/spd/range/alt = %d/%d/%d/%d/%d)",
    plan.use_standard_migrated ? 1 : 0,
    plan.sub_rangetobase ? 1 : 0,
    plan.sub_frame_legacy ? 1 : 0,
    plan.sub_att_legacy ? 1 : 0,
    plan.sub_spd_legacy ? 1 : 0,
    plan.sub_range_legacy ? 1 : 0,
    plan.sub_alt_legacy ? 1 : 0);
}
//--------------------------------------------------------------------------
EKFslam::~EKFslam()
{ 
  ekf_thread_loop_run = false; // for leaving the main ekf loop
  ekf_loop_.join();
}
//-----------------------------------------------------------------------------


//------------------------------------------------------------------------
//  Main EKF slam loop (this function runs in a separate thread)
void EKFslam::EKF_LOOP()
{
  
  EKF ekf(PAR); // create EKF object
  ekf.set_reference_identity(estimator_session_id_, estimator_epoch_id_);
  cout << "-> EKF-SLAM thread running... " << endl;   

  DATA dat;
  bool ekf_s_l_r = false;  
  
  while(ekf_thread_loop_run == true)  // thread main loop
  {
    
    mutex_ekf_run.lock();
      ekf_s_l_r = ekf_steps_loop_run;
    mutex_ekf_run.unlock();      


    while(ekf_s_l_r == true)  // if ekf_steps_loop_run flag is true
    {  
        
        mutex_ekf_run.lock();
        // if a signal to stop/start the loop of steps is received
          ekf_s_l_r = ekf_steps_loop_run; // this flag can be changed by service "srv_ekf_run_"
        mutex_ekf_run.unlock(); 
       
        mutex_init_system.lock();
          if (re_init_sys == true) // this flag can be changed by service "srv_ekf_run_"
          { 
            // tell the ekf object that the system must be initialized again
            ekf.set_ekf_initialized_statet(false);         
            re_init_sys = false;
            // Drop queued corrections under the same lock used at apply time.
            {
              std::lock_guard<std::mutex> lock_pos(mutex_pos_update);
              clear_pending_pos_update_locked();
              applied_correction_ids_.clear();
            }
            last_published_correction_id_ = 0;
            // Keep EKF identity at current epoch until next successful init
            // bumps epoch_id; clear any pre-reset KF still sitting in EKF.
            ekf.set_reference_identity(estimator_session_id_, estimator_epoch_id_);
            // A4: clear trajectory and publish empty landmark cloud so consumers
            // drop the previous local map (custom Lslam still follows re-init).
            clear_and_publish_path();
            pub_empty_landmarks();
            map_has_visual_state_ = false;
          }      
        mutex_init_system.unlock();

        // A8: apply CameraInfo intrinsics to the EKF object (PAR is by-value copy).
        {
          std::lock_guard<std::mutex> lock(mutex_cal_);
          if (camera_cal_pending_apply_ && calibration_ready_) {
            ekf.set_camera_intrinsics(
              applied_intrinsics_.width,
              applied_intrinsics_.height,
              applied_intrinsics_.fc_u,
              applied_intrinsics_.fc_v,
              applied_intrinsics_.cc_u,
              applied_intrinsics_.cc_v,
              applied_intrinsics_.alpha_c,
              applied_intrinsics_.distortions);
            camera_cal_pending_apply_ = false;
          }
        }

      if (ekf.get_ekf_initialized_state() == false)
        {   
            // Initialize system state
            dat = getData();
            const bool was_init = ekf.get_ekf_initialized_state();
            ekf.system_init(dat);
            if (!was_init && ekf.get_ekf_initialized_state()) {
              {
                std::lock_guard<std::mutex> lock_pos(mutex_pos_update);
                ++estimator_epoch_id_;
                applied_correction_ids_.clear();
                clear_pending_pos_update_locked();
              }
              last_published_correction_id_ = 0;
              ekf.set_reference_identity(estimator_session_id_, estimator_epoch_id_);
              epoch_heading_time_ms_ =
                (dat.att.time != 0) ? dat.att.time : dat.time;
              epoch_yaw_at_home_ = ekf.get_yaw_at_home();
              publish_estimator_reference_from_ekf(ekf, dat);
            }
        }
      else
        {          
            //--------------------            
            dat = getData();  // get data from dataset/actual robot           
            
            ekf.ekf_step(dat); // perform a filter step
            if (ekf.consume_altitude_home_latched_event()) {
              publish_estimator_reference_from_ekf(ekf, dat);
            }
            
            //------- after ekf step publish stuff!!! --------------------------------------
                // Consistent snapshot: custom Robotstate + standard ~/odometry
                arma::vec::fixed<13> x_r;
                arma::mat::fixed<13, 13> P_r;
                double radius_uncertainty;
                if (ekf.get_RobotStateSnapshot(x_r, P_r, radius_uncertainty) == true)
                {
                  pub_robot_state(x_r, radius_uncertainty);
                  pub_odometry(x_r, P_r);
                }

                if (dat.data_type == "frame")
                {
                  // for each frame check if there is a new Key Frame available
                  KEYFRAME KF;
                  if(ekf.get_KeyFrame(KF) == true)
                  {
                    // publish keyframe for global mapping process
                    pub_kf(KF);
                  } 
                  KEYFRAME KFCL;
                  if(ekf.get_KeyFrameCL(KFCL) == true)
                  {
                    // publish "intermediate" keyframe for closing-loop process
                    pub_kf_cl(KFCL);
                  }
                    // publish local slam data (custom + A4 PointCloud2)
                    LOCALSLAM_DATA lslam_data;
                    lslam_data = ekf.get_lslam_data();
                    if (!lslam_data.map_state.empty() ||
                      !lslam_data.map_anchors.empty())
                    {
                      map_has_visual_state_ = true;
                    }
                    pub_lslam_Data(lslam_data);
                    pub_landmarks_from_lslam(lslam_data);
                }
                //LOCALSLAM_DATA lslam_data;
                //    lslam_data = ekf.get_lslam_data();
                //pub_lslam_Data(lslam_data);
             //-------------end publish data ----------------------------------------
             
             // -- apply queued Global SLAM Δ (re-validate identity under lock)
             bool publish_correction_notice = false;
             interfaces::msg::CorrectionApplied corr_notice;
             {
               std::lock_guard<std::mutex> lock_pos(mutex_pos_update);
               if (pos_update.pos_update_available) {
                 PosUpdateRequestView view;
                 view.delta_x = pos_update.delta_pos_update[0];
                 view.delta_y = pos_update.delta_pos_update[1];
                 view.delta_z = pos_update.delta_pos_update[2];
                 view.type = pos_update.type;
                 view.source = pos_update.source;
                 view.session_id = {};
                 for (size_t i = 0; i < 16; ++i) {
                   view.session_id[i] = pos_update.session_id[i];
                 }
                 view.epoch_id = pos_update.epoch_id;
                 view.keyframe_id = pos_update.keyframe_id;
                 view.correction_id = pos_update.correction_id;

                 // Identity must still match at apply time. correction_id was
                 // consumed at admission; do not re-insert into applied_ids.
                 pos_update.pos_update_available = false;
                 if (pos_update.cancel_requested) {
                   pos_update.result_applied = false;
                   pos_update.applied_delta.zeros();
                   pos_update.outcome = "cancelled";
                   pos_update.result_last_generated_kf_id =
                     ekf.last_generated_keyframe_id();
                   pos_update.result_ready = true;
                   cv_pos_update_.notify_all();
                 } else {
                 bool identity_ok =
                   estimator_ref_published_valid_ &&
                   slam_identity::equal(view.session_id, estimator_session_id_) &&
                   view.epoch_id == estimator_epoch_id_;
                 if (!identity_ok) {
                   pos_update.result_applied = false;
                   pos_update.applied_delta.zeros();
                   pos_update.outcome = "reset_before_apply";
                   pos_update.result_last_generated_kf_id =
                     ekf.last_generated_keyframe_id();
                   pos_update.result_ready = true;
                   cv_pos_update_.notify_all();
                   RCLCPP_WARN(
                     this->get_logger(),
                     "LSposUpdate dropped at EKF apply: identity/reset "
                     "(correction_id=%llu)",
                     static_cast<unsigned long long>(view.correction_id));
                 } else {
                   const auto applied = ekf.Update_pos_with_delta(
                     pos_update.delta_pos_update, pos_update.type);
                   pos_update.result_applied = applied.applied;
                   pos_update.applied_delta[0] = applied.applied_dx;
                   pos_update.applied_delta[1] = applied.applied_dy;
                   pos_update.applied_delta[2] = applied.applied_dz;
                   pos_update.outcome = applied.applied ? "applied" : "gated_noop";
                   // Watermark after mutation: includes any KF stamped this cycle.
                   pos_update.result_last_generated_kf_id =
                     ekf.last_generated_keyframe_id();
                   pos_update.result_ready = true;
                   cv_pos_update_.notify_all();
                   if (applied.applied) {
                     RCLCPP_INFO(
                       this->get_logger(),
                       "LOCAL_APPLIED correction_id=%llu d=[%.4f %.4f %.4f] "
                       "last_gen_kf=%llu",
                       static_cast<unsigned long long>(view.correction_id),
                       applied.applied_dx, applied.applied_dy, applied.applied_dz,
                       static_cast<unsigned long long>(
                         pos_update.result_last_generated_kf_id));
                   }
                   // Publish corrected snapshot in this cycle (path keeps history;
                   // past poses are not retroactively optimized — jump is expected).
                   if (applied.applied) {
                     last_published_correction_id_ = view.correction_id;
                     arma::vec::fixed<13> x_corr;
                     arma::mat::fixed<13, 13> P_corr;
                     double ru_corr = 0.0;
                     uint64_t snap_seq = 0;
                     if (ekf.get_RobotStateSnapshot(x_corr, P_corr, ru_corr)) {
                       pub_robot_state(x_corr, ru_corr);
                       snap_seq = pub_odometry(x_corr, P_corr);
                     }
                     corr_notice.header.stamp = this->get_clock()->now();
                     corr_notice.header.frame_id = PAR.Odometry_frame_id;
                     corr_notice.session_id = slam_identity::to_msg(view.session_id);
                     corr_notice.epoch_id = view.epoch_id;
                     corr_notice.correction_id = view.correction_id;
                     corr_notice.snapshot_seq = snap_seq;
                     corr_notice.type = view.type;
                     corr_notice.source = view.source;
                     corr_notice.applied = true;
                     corr_notice.applied_delta_x = applied.applied_dx;
                     corr_notice.applied_delta_y = applied.applied_dy;
                     corr_notice.applied_delta_z = applied.applied_dz;
                     corr_notice.outcome = "applied";
                     // Only advertise the barrier when a unique snapshot was published.
                     publish_correction_notice = (snap_seq != 0);
                     if (snap_seq != 0) {
                       RCLCPP_INFO(
                         this->get_logger(),
                         "[G6b-TXN] CorrectionApplied correction_id=%llu "
                         "snapshot_seq=%llu type=%s delta=[%.4f %.4f %.4f]",
                         static_cast<unsigned long long>(view.correction_id),
                         static_cast<unsigned long long>(snap_seq),
                         view.type.c_str(),
                         applied.applied_dx, applied.applied_dy, applied.applied_dz);
                     }
                   }
                 }
                 }
               }
             }
             if (publish_correction_notice && pub_correction_applied_) {
               pub_correction_applied_->publish(corr_notice);
             }

             //-- check if log date is required
             mutex_log.lock();
               if(get_log_flag == true)
               {
                 get_log_flag = false;
                 log_data(ekf.store);
               } 
             mutex_log.unlock();

             mutex_ru.lock();
               if(activate_CL_uncertainty_flag == true)
               {
                 activate_CL_uncertainty_flag = false;
                 ekf.SetCompute_Pc_flag(activate_CL_uncertainty);
               }
            mutex_ru.unlock();

            mutex_vu.lock();
            if(activate_ekf_visual_update_flag  == true)
               {
                ekf.set_ekf_visual_update(true);
                activate_ekf_visual_update_flag = false;
               }
            mutex_vu.unlock();


        
        } // if ekf is initilized

        // LSLAM-PERF-W1: flush buffered stage/convert rows + DataBuffer counters.
        if (localslam::visual_profile::enabled()) {
          static auto next_db = std::chrono::steady_clock::now();
          const auto now_db = std::chrono::steady_clock::now();
          if (now_db >= next_db) {
            next_db = now_db + std::chrono::seconds(1);
            localslam::visual_profile::flush_pending();
            localslam::visual_profile::write_databuffer_row(
              diag_frames_enqueued_, diag_frames_overwritten_, diag_frames_consumed_,
              diag_frames_callback_rx_);
          }
        }
        // LSLAM-DYNAMIC-CONSISTENCY-A3: opt-in stage CSV flush (disabled unless env set).
        if (localslam::dynamic_diag::enabled()) {
          static auto next_dd = std::chrono::steady_clock::now();
          const auto now_dd = std::chrono::steady_clock::now();
          if (now_dd >= next_dd) {
            next_dd = now_dd + std::chrono::seconds(1);
            localslam::dynamic_diag::flush();
          }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // sleep a short period of time to save proccesor use

    } // while(ekf.run == true)    

  }
 

}
//--------------------------------------------------------------------------------------------------------------
// Publish robot state data to topic
// Rodrigo M. 2022
void EKFslam::pub_robot_state(arma::vec::fixed<13> &x_r, double &radius_uncertainty)
{
  auto message = interfaces::msg::Robotstate();

  message.robot_state = arma::conv_to<std::vector<double>>::from(x_r);

  double phi = x_r(1);
  double theta = x_r(2);
  double psi = x_r(3);
  double Ra2b[9];
  Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
  arma::mat Rn2r_a(Ra2b,3,3);
  arma::mat::fixed<3,3> Rn2r = Rn2r_a;
  arma::vec::fixed<4> q_n2r; 
  Ra2b_TO_Quat_a2b_arma(Rn2r, q_n2r);

  geometry_msgs::msg::Transform t_n2r;
    t_n2r.translation.x = x_r(7);
    t_n2r.translation.y = x_r(8);
    t_n2r.translation.z = x_r(9);

    t_n2r.rotation.w =  q_n2r(0);
    t_n2r.rotation.x =  q_n2r(1);
    t_n2r.rotation.y =  q_n2r(2);
    t_n2r.rotation.z =  q_n2r(3);

  message.n2r = t_n2r;
  
  geometry_msgs::msg::Twist vels;
  // linear velocity of the robot expressed in the body frame
  vels.linear.x = x_r(10);
  vels.linear.y = x_r(11);
  vels.linear.z = x_r(12);
  // angular velocity of the robot expressed in the body frame
  vels.angular.x = x_r(4);
  vels.angular.y = x_r(5);
  vels.angular.z = x_r(6);

  message.vels = vels;
  message.radius_uncertainty = radius_uncertainty;  

  pub_robot_state_->publish(message);  

  //cout << "slam  : " << x_r(7)<< " " << x_r(8) << "  " << x_r(9) << " " << x_r(3)  << endl;

}

uint64_t EKFslam::pub_odometry(
  const arma::vec::fixed<13> & x_r,
  const arma::mat::fixed<13, 13> & P_r)
{
  double x[13];
  double Prm[13 * 13];
  for (int i = 0; i < 13; ++i) {
    x[i] = x_r(i);
    for (int j = 0; j < 13; ++j) {
      Prm[i * 13 + j] = P_r(i, j);
    }
  }

  odometry::PublicOdometry pub;
  if (!odometry::convert_euler_state_to_public_odometry(x, Prm, pub)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Skipping ~/odometry: %s", pub.reject_reason.c_str());
    return 0;
  }

  const uint64_t snapshot_seq = next_odometry_snapshot_seq_++;
  last_odometry_snapshot_seq_ = snapshot_seq;
  // One stamp shared by Odometry and OdometryMeta (association uses snapshot_seq).
  const builtin_interfaces::msg::Time stamp = this->get_clock()->now();

  nav_msgs::msg::Odometry msg;
  // Publication-time stamp on the configured ROS clock (sim or system).
  // Prediction remains wall-clock-driven; this is not a measurement epoch.
  msg.header.stamp = stamp;
  msg.header.frame_id = PAR.Odometry_frame_id;
  msg.child_frame_id = PAR.Odometry_child_frame_id;

  msg.pose.pose.position.x = pub.position[0];
  msg.pose.pose.position.y = pub.position[1];
  msg.pose.pose.position.z = pub.position[2];
  msg.pose.pose.orientation.w = pub.quat_wxyz[0];
  msg.pose.pose.orientation.x = pub.quat_wxyz[1];
  msg.pose.pose.orientation.y = pub.quat_wxyz[2];
  msg.pose.pose.orientation.z = pub.quat_wxyz[3];
  for (int i = 0; i < 36; ++i) {
    msg.pose.covariance[i] = pub.pose_covariance[i];
  }

  msg.twist.twist.linear.x = pub.twist_linear[0];
  msg.twist.twist.linear.y = pub.twist_linear[1];
  msg.twist.twist.linear.z = pub.twist_linear[2];
  msg.twist.twist.angular.x = pub.twist_angular[0];
  msg.twist.twist.angular.y = pub.twist_angular[1];
  msg.twist.twist.angular.z = pub.twist_angular[2];
  for (int i = 0; i < 36; ++i) {
    msg.twist.covariance[i] = pub.twist_covariance[i];
  }

  pub_odometry_->publish(msg);

  if (pub_odometry_meta_) {
    interfaces::msg::OdometryMeta meta;
    meta.header.stamp = stamp;
    meta.header.frame_id = PAR.Odometry_frame_id;
    meta.snapshot_seq = snapshot_seq;
    meta.session_id = slam_identity::to_msg(estimator_session_id_);
    meta.epoch_id = estimator_epoch_id_;
    meta.last_correction_id = last_published_correction_id_;
    pub_odometry_meta_->publish(meta);
  }

  // A4 path: same accepted snapshot, same stamp/frame/orientation as odometry.
  geometry_msgs::msg::PoseStamped ps;
  ps.header = msg.header;
  ps.pose = msg.pose.pose;
  path_msg_.header = msg.header;
  map_path::append_path_pose(path_msg_, ps, PAR.Path_max_poses);
  pub_path_->publish(path_msg_);
  return snapshot_seq;
}

void EKFslam::pub_landmarks_from_lslam(LOCALSLAM_DATA &lslam_data)
{
  std::vector<map_path::LandmarkXYZ> pts;
  pts.reserve(lslam_data.map_state.size() + lslam_data.map_anchors.size());
  for (const auto & p : lslam_data.map_state) {
    pts.push_back({p.x, p.y, p.z});
  }
  for (const auto & p : lslam_data.map_anchors) {
    pts.push_back({p.x, p.y, p.z});
  }

  std_msgs::msg::Header header;
  header.stamp = this->get_clock()->now();
  header.frame_id = PAR.Odometry_frame_id;
  auto cloud = map_path::make_landmark_cloud(header, pts);
  pub_landmarks_->publish(cloud);
}

void EKFslam::pub_empty_landmarks()
{
  std_msgs::msg::Header header;
  header.stamp = this->get_clock()->now();
  header.frame_id = PAR.Odometry_frame_id;
  pub_landmarks_->publish(map_path::make_empty_landmark_cloud(header));
}

void EKFslam::clear_and_publish_path()
{
  map_path::clear_path(path_msg_);
  path_msg_.header.stamp = this->get_clock()->now();
  path_msg_.header.frame_id = PAR.Odometry_frame_id;
  pub_path_->publish(path_msg_);
}

builtin_interfaces::msg::Time EKFslam::stamp_from_ms(long int time_ms)
{
  builtin_interfaces::msg::Time t;
  if (time_ms < 0) {
    t.sec = 0;
    t.nanosec = 0;
    return t;
  }
  t.sec = static_cast<int32_t>(time_ms / 1000);
  t.nanosec = static_cast<uint32_t>((time_ms % 1000) * 1000000L);
  return t;
}

void EKFslam::fill_estimator_reference_identity(
  interfaces::msg::EstimatorReference & msg) const
{
  msg.session_id = slam_identity::to_msg(estimator_session_id_);
  msg.epoch_id = estimator_epoch_id_;
}

void EKFslam::clear_pending_pos_update_locked()
{
  const bool had_pending = pos_update.pos_update_available;
  pos_update.pos_update_available = false;
  pos_update.cancel_requested = true;
  pos_update.delta_pos_update.zeros();
  pos_update.type.clear();
  pos_update.source.clear();
  pos_update.session_id = {};
  pos_update.epoch_id = 0;
  pos_update.keyframe_id = 0;
  pos_update.correction_id = 0;
  if (had_pending && !pos_update.result_ready) {
    pos_update.result_ready = true;
    pos_update.result_applied = false;
    pos_update.applied_delta.zeros();
    pos_update.outcome = "reset_before_apply";
    cv_pos_update_.notify_all();
  }
}

void EKFslam::publish_estimator_reference_invalid()
{
  if (!pub_estimator_ref_) {
    return;
  }
  interfaces::msg::EstimatorReference msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = PAR.Odometry_frame_id;
  fill_estimator_reference_identity(msg);
  msg.valid = false;
  msg.heading_valid = false;
  msg.altitude_update_enabled = PAR.Altitude_update;
  msg.altitude_valid = false;
  msg.robot_init_x = PAR.Robot_init_x;
  msg.robot_init_y = PAR.Robot_init_y;
  msg.robot_init_z = PAR.Robot_init_z;
  {
    std::lock_guard<std::mutex> lock_pos(mutex_pos_update);
    estimator_ref_published_valid_ = false;
  }
  pub_estimator_ref_->publish(msg);
  RCLCPP_INFO(
    this->get_logger(),
    "EstimatorReference invalidated (session=%s epoch_id=%llu)",
    slam_identity::to_string(estimator_session_id_).c_str(),
    static_cast<unsigned long long>(estimator_epoch_id_));
}

void EKFslam::publish_estimator_reference_from_ekf(
  ::EKF & filter, const DATA & heading_dat)
{
  if (!pub_estimator_ref_) {
    return;
  }
  interfaces::msg::EstimatorReference msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = PAR.Odometry_frame_id;
  fill_estimator_reference_identity(msg);
  msg.valid = true;
  msg.robot_init_x = PAR.Robot_init_x;
  msg.robot_init_y = PAR.Robot_init_y;
  msg.robot_init_z = PAR.Robot_init_z;

  msg.heading_valid = true;
  // Use the frozen init-epoch heading (not the current step's observation).
  (void)heading_dat;
  msg.heading_stamp = stamp_from_ms(epoch_heading_time_ms_);
  msg.yaw_at_home_internal = epoch_yaw_at_home_;
  // Att.yaw ≈ -yaw_ENU (+ noise). Public ENU equivalent for R_z(-yaw_enu).
  msg.yaw_enu_equivalent = -msg.yaw_at_home_internal;

  msg.altitude_update_enabled = PAR.Altitude_update;
  if (!PAR.Altitude_update) {
    msg.altitude_valid = true;
    msg.altitude_stamp = msg.heading_stamp;
    msg.altitude_home_up = std::numeric_limits<double>::quiet_NaN();
  } else if (filter.altitude_home_ready()) {
    msg.altitude_valid = true;
    msg.altitude_stamp = stamp_from_ms(filter.altitude_home_time_ms());
    // alt_at_home_ is after Alt_h_2_d; invert to +up raw height domain.
    msg.altitude_home_up = filter.altitude_home_internal() *
      (PAR.Alt_h_2_d == 0 ? 1.0 : (1.0 / static_cast<double>(PAR.Alt_h_2_d)));
  } else {
    msg.altitude_valid = false;
    msg.altitude_home_up = std::numeric_limits<double>::quiet_NaN();
  }

  {
    std::lock_guard<std::mutex> lock_pos(mutex_pos_update);
    estimator_ref_published_valid_ = true;
  }
  pub_estimator_ref_->publish(msg);
  RCLCPP_INFO(
    this->get_logger(),
    "EstimatorReference session=%s epoch_id=%llu heading_valid=%d altitude_valid=%d "
    "yaw_enu=%.4f robot_init=(%.3f,%.3f,%.3f)",
    slam_identity::to_string(estimator_session_id_).c_str(),
    static_cast<unsigned long long>(msg.epoch_id),
    msg.heading_valid ? 1 : 0,
    msg.altitude_valid ? 1 : 0,
    msg.yaw_enu_equivalent,
    msg.robot_init_x, msg.robot_init_y, msg.robot_init_z);
}

//--------------------------------------------------------------------------------------------------------------
// Publish LocalSLAM data to topic
// Rodrigo M. 2022
void EKFslam::pub_lslam_Data(LOCALSLAM_DATA &lslam_data)
{
  auto message = interfaces::msg::Lslam();      
           
      int size_map = lslam_data.map_state.size();
     
      message.map_state.clear();      
      message.robot_state = lslam_data.robot_state;

      arma::vec::fixed<4> q_r2c; 
     Ra2b_TO_Quat_a2b_arma(lslam_data.Rr2c, q_r2c); 
  
     geometry_msgs::msg::Transform t_r2c;
    t_r2c.translation.x = lslam_data.t_c2r(0);
    t_r2c.translation.y = lslam_data.t_c2r(1);
    t_r2c.translation.z = lslam_data.t_c2r(2);

    t_r2c.rotation.w =  q_r2c(0);
    t_r2c.rotation.x =  q_r2c(1);
    t_r2c.rotation.y =  q_r2c(2);
    t_r2c.rotation.z =  q_r2c(3);

    arma::vec::fixed<4> q_n2r; 
    Ra2b_TO_Quat_a2b_arma(lslam_data.Rn2r, q_n2r);

    geometry_msgs::msg::Transform t_n2r;
    t_n2r.translation.x = lslam_data.r_N(0);
    t_n2r.translation.y = lslam_data.r_N(1);
    t_n2r.translation.z = lslam_data.r_N(2);

    t_n2r.rotation.w =  q_n2r(0);
    t_n2r.rotation.x =  q_n2r(1);
    t_n2r.rotation.y =  q_n2r(2);
    t_n2r.rotation.z =  q_n2r(3);

    message.r2c = t_r2c;
    message.n2r = t_n2r;         
         
         for(int i = 0; i < size_map ; i++)
         { 
            geometry_msgs::msg::Point p;        
            p.x = lslam_data.map_state[i].x;
            p.y = lslam_data.map_state[i].y;
            p.z = lslam_data.map_state[i].z;
            message.map_state.push_back(p);
         }
         
         for(int i = 0; i < lslam_data.matched_img_feats.size() ; i++)
         {
            geometry_msgs::msg::Point pm;
            pm.x = lslam_data.matched_img_feats[i].x;
            pm.y = lslam_data.matched_img_feats[i].y;
            message.matched_img_feats.push_back(pm);
         }
           
         for(int i = 0; i < lslam_data.unmatched_img_feats.size() ; i++)
         {
            geometry_msgs::msg::Point pu;
            pu.x = lslam_data.unmatched_img_feats[i].x;
            pu.y = lslam_data.unmatched_img_feats[i].y;
            message.unmatched_img_feats.push_back(pu);
         }

         for(int i = 0; i < lslam_data.map_anchors.size() ; i++)
         { 
            geometry_msgs::msg::Point p;        
            p.x = lslam_data.map_anchors[i].x;
            p.y = lslam_data.map_anchors[i].y;
            p.z = lslam_data.map_anchors[i].z;
            message.map_anchors.push_back(p);
         }

         for(int i = 0; i < lslam_data.matched_img_anchors.size() ; i++)
         {
            geometry_msgs::msg::Point pm;
            pm.x = lslam_data.matched_img_anchors[i].x;
            pm.y = lslam_data.matched_img_anchors[i].y;
            message.matched_img_anchors.push_back(pm);
         }
           
         for(int i = 0; i < lslam_data.unmatched_img_anchors.size() ; i++)
         {
            geometry_msgs::msg::Point pu;
            pu.x = lslam_data.unmatched_img_anchors[i].x;
            pu.y = lslam_data.unmatched_img_anchors[i].y;
            message.unmatched_img_anchors.push_back(pu);
         } 

  message.radius_uncertainty = lslam_data.radius_uncertainty;             
     
  pub_lslam_data_->publish(message);    
    //cout << "xx" << endl;


}
void EKFslam::fill_kf_message(interfaces::msg::Kf & message, KEYFRAME & KF) const
{
  arma::vec::fixed<4> q_r2c;
  Ra2b_TO_Quat_a2b_arma(KF.Rr2c, q_r2c);

  geometry_msgs::msg::Transform t_r2c;
  t_r2c.translation.x = KF.t_c2r(0);
  t_r2c.translation.y = KF.t_c2r(1);
  t_r2c.translation.z = KF.t_c2r(2);
  t_r2c.rotation.w = q_r2c(0);
  t_r2c.rotation.x = q_r2c(1);
  t_r2c.rotation.y = q_r2c(2);
  t_r2c.rotation.z = q_r2c(3);

  arma::vec::fixed<4> q_n2r;
  Ra2b_TO_Quat_a2b_arma(KF.Rn2r, q_n2r);

  geometry_msgs::msg::Transform t_n2r;
  t_n2r.translation.x = KF.r_N(0);
  t_n2r.translation.y = KF.r_N(1);
  t_n2r.translation.z = KF.r_N(2);
  t_n2r.rotation.w = q_n2r(0);
  t_n2r.rotation.x = q_n2r(1);
  t_n2r.rotation.y = q_n2r(2);
  t_n2r.rotation.z = q_n2r(3);

  // Identity captured at selection — do not substitute the live epoch here.
  message.session_id = slam_identity::to_msg(KF.session_id);
  message.epoch_id = KF.epoch_id;
  message.keyframe_id = KF.keyframe_id;
  message.source_stamp_valid = KF.source_stamp_valid;
  message.stream = KF.stream;
  message.r2c = t_r2c;
  message.n2r = t_n2r;
  message.radius_uncertainty = KF.radius_uncertainty;

  std_msgs::msg::Header img_header;
  // frame_id documents internal pose packet semantics (not public odom).
  img_header.frame_id = "internal_nav_cam";
  if (KF.source_stamp_valid) {
    img_header.stamp = stamp_from_ms(KF.source_time_ms);
  } else {
    // Deliberate: do not fabricate publish-time. Zero stamp + flag=false.
    img_header.stamp.sec = 0;
    img_header.stamp.nanosec = 0;
  }
  message.header = img_header;

  sensor_msgs::msg::Image::SharedPtr img_msg =
    cv_bridge::CvImage(img_header, sensor_msgs::image_encodings::MONO8, KF.frame)
      .toImageMsg();
  message.img = *img_msg;
}

//----------------------------------------------------------------------------------------------------------------
// Publish "intermediate"KeyFrames to topic
// Rodrigo M. 2022
void EKFslam::pub_kf_cl(KEYFRAME &KF)
{
  auto message = interfaces::msg::Kf();
  fill_kf_message(message, KF);
  pub_kf_cl_->publish(message);
}

//--------------------------------------------------------------------------------------------------------------
// Publish KeyFrames to topic
// Rodrigo M. 2022
void EKFslam::pub_kf(KEYFRAME &KF)
{
  auto message = interfaces::msg::Kf();
  fill_kf_message(message, KF);
  pub_kf_->publish(message);
}
//------------------------------------------------------------------------------------------------------------------------------
void EKFslam::Handle_pos_update_service(
  const std::shared_ptr<interfaces::srv::LSposUpdate::Request> request,
  std::shared_ptr<interfaces::srv::LSposUpdate::Response> response)
{
  response->accepted = false;
  response->applied = false;
  response->applied_delta_x = 0.0;
  response->applied_delta_y = 0.0;
  response->applied_delta_z = 0.0;
  response->outcome = "rejected";
  response->reason.clear();
  response->last_generated_keyframe_id = 0;

  PosUpdateRequestView view;
  view.delta_x = request->delta_x;
  view.delta_y = request->delta_y;
  view.delta_z = request->delta_z;
  view.type = request->type;
  view.source = request->source;
  view.session_id = slam_identity::from_msg(request->session_id);
  view.epoch_id = request->epoch_id;
  view.keyframe_id = request->keyframe_id;
  view.correction_id = request->correction_id;

  bool steps_running = false;
  {
    std::lock_guard<std::mutex> lock_run(mutex_ekf_run);
    steps_running = ekf_steps_loop_run;
  }
  if (!steps_running) {
    response->reason = "localslam_disabled";
    response->outcome = "localslam_disabled";
    return;
  }
  // EKF object lives on the worker thread; valid EstimatorReference is the
  // cross-thread signal that system_init succeeded for this epoch.
  if (!estimator_ref_published_valid_) {
    response->reason = "not_initialized";
    response->outcome = "not_initialized";
    return;
  }

  std::unique_lock<std::mutex> lock_pos(mutex_pos_update);
  if (pos_update.pos_update_available) {
    response->reason = "busy";
    response->outcome = "busy";
    return;
  }

  if (bookkeeping_near_capacity(applied_correction_ids_)) {
    RCLCPP_WARN(
      this->get_logger(),
      "LSposUpdate bookkeeping near capacity (%zu/%zu) for this epoch; "
      "further distinct correction_ids will be rejected until epoch reset "
      "(no eviction — duplicates stay blocked)",
      applied_correction_ids_.size(),
      localslam::kMaxAppliedCorrectionIds);
  }
  const auto admit = admit_pos_update(
    view,
    estimator_ref_published_valid_,
    estimator_session_id_,
    estimator_epoch_id_,
    applied_correction_ids_);
  if (!admit.accepted) {
    response->reason = admit.reason;
    response->outcome = "rejected";
    RCLCPP_WARN(
      this->get_logger(),
      "LSposUpdate rejected: %s (correction_id=%llu epoch=%llu)",
      admit.reason.c_str(),
      static_cast<unsigned long long>(view.correction_id),
      static_cast<unsigned long long>(view.epoch_id));
    return;
  }

  pos_update.delta_pos_update[0] = view.delta_x;
  pos_update.delta_pos_update[1] = view.delta_y;
  pos_update.delta_pos_update[2] = view.delta_z;
  pos_update.type = view.type;
  pos_update.source = view.source;
  pos_update.session_id = view.session_id;
  pos_update.epoch_id = view.epoch_id;
  pos_update.keyframe_id = view.keyframe_id;
  pos_update.correction_id = view.correction_id;
  pos_update.result_ready = false;
  pos_update.result_applied = false;
  pos_update.applied_delta.zeros();
  pos_update.outcome.clear();
  pos_update.cancel_requested = false;
  pos_update.pos_update_available = true;
  response->accepted = true;

  // Wait for EKF worker (separate thread) — do not hold other locks.
  // Timeout: wait_for reacquires the mutex before returning. The worker holds
  // this same mutex for the entire take→mutate→result path, so a timeout
  // return either sees result_ready or a still-pending request we can cancel
  // atomically (no unreported later mutation).
  const bool done = cv_pos_update_.wait_for(
    lock_pos, std::chrono::milliseconds(2000),
    [this]() { return pos_update.result_ready; });
  if (!done) {
    if (pos_update.result_ready) {
      // Applied while reacquiring the lock after the wait clock expired.
    } else if (pos_update.pos_update_available) {
      pos_update.pos_update_available = false;
      pos_update.cancel_requested = true;
      pos_update.result_applied = false;
      pos_update.applied_delta.zeros();
      pos_update.outcome = "cancelled";
      pos_update.result_ready = true;
      response->reason = "apply_timeout_cancelled";
      response->outcome = "cancelled";
      response->applied = false;
      RCLCPP_WARN(
        this->get_logger(),
        "LSposUpdate cancelled on timeout before EKF mutation "
        "(correction_id=%llu)",
        static_cast<unsigned long long>(view.correction_id));
      return;
    } else {
      // Worker cleared available but has not published result yet — should be
      // unreachable while worker holds this mutex through mutation. Treat as
      // unresolved: do not claim non-application.
      response->reason = "apply_pending_unresolved";
      response->outcome = "apply_pending";
      response->applied = false;
      RCLCPP_ERROR(
        this->get_logger(),
        "LSposUpdate timeout with in-flight apply (correction_id=%llu); "
        "outcome unresolved — do not auto-resume coordination",
        static_cast<unsigned long long>(view.correction_id));
      return;
    }
  }

  response->applied = pos_update.result_applied;
  response->applied_delta_x = pos_update.applied_delta[0];
  response->applied_delta_y = pos_update.applied_delta[1];
  response->applied_delta_z = pos_update.applied_delta[2];
  response->outcome = pos_update.outcome;
  response->reason = pos_update.outcome == "applied" ? "" : pos_update.outcome;
  response->last_generated_keyframe_id = pos_update.result_last_generated_kf_id;
}

//-----------------------------------------------------------------------------------------------------------------------------
void EKFslam::set_ekf_steps_running(bool running)
{
  mutex_ekf_run.lock();
  ekf_steps_loop_run = running;
  mutex_ekf_run.unlock();
}

void EKFslam::request_ekf_reinit()
{
  // Same semantics as legacy SimpleServ 'i': mark uninitialized on the
  // worker thread. Altitude home is instance-owned and cleared with
  // set_ekf_initialized_statet(false) / system_init.
  // Invalidate reference immediately; clear queued corrections so a
  // prepare→reset→send race cannot mutate the next epoch.
  {
    std::lock_guard<std::mutex> lock_pos(mutex_pos_update);
    clear_pending_pos_update_locked();
    applied_correction_ids_.clear();
  }
  // G6b: clear correction association; keep snapshot_seq monotonic (never reuse).
  last_published_correction_id_ = 0;
  publish_estimator_reference_invalid();  // sets estimator_ref_published_valid_=false
  mutex_init_system.lock();
  re_init_sys = true;
  mutex_init_system.unlock();
  // A8: geometry changes are accepted only after explicit re-init.
  {
    std::lock_guard<std::mutex> lock(mutex_cal_);
    allow_new_geometry_after_reset_ = true;
    map_has_visual_state_ = false;
  }
  // A9: clear pressure reference so the next valid FluidPressure re-latches.
  reset_pressure_altitude_converter();
  clear_pending_altitude_observations();
}

void EKFslam::reset_pressure_altitude_converter()
{
  pressure_converter_.reset_reference();
  pressure_callback_generation_ = pressure_converter_.generation();
  std::lock_guard<std::mutex> lock(mutex_pressure_wait_);
  waiting_pressures_.clear();
  waiting_pressures_dropped_ = 0;
}

void EKFslam::clear_pending_altitude_observations()
{
  // Drop buffered alt observations so a pre-reset sample cannot become the
  // first post-reset altitude (ManageBuffer keeps one-per-type).
  std::lock_guard<std::mutex> lock(mutex_dat);
  for (int i = 0; i < static_cast<int>(DataBuffer.size()); ++i) {
    if (DataBuffer[i].data_type == "alt") {
      DataBuffer.erase(DataBuffer.begin() + i);
      --i;
    }
  }
}

void EKFslam::request_log_stats()
{
  mutex_log.lock();
  get_log_flag = true;
  mutex_log.unlock();
}

void EKFslam::toggle_cl_uncertainty()
{
  mutex_ru.lock();
  activate_CL_uncertainty = !activate_CL_uncertainty;
  activate_CL_uncertainty_flag = true;
  mutex_ru.unlock();
}

void EKFslam::request_activate_visual_update()
{
  mutex_vu.lock();
  activate_ekf_visual_update_flag = true;
  mutex_vu.unlock();
}

void EKFslam::Handle_enable_service(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  // Maps to SimpleServ 'r' / 's'. Enable before init is valid: the worker
  // waits for attitude (or proceeds without) once running. Disable freezes
  // steps; re-enable resumes without implicit reinit.
  // G5: disable clears any pending correction (waiting service gets
  // reset_before_apply) — do not leave work that cannot run.
  if (!request->data) {
    std::lock_guard<std::mutex> lock_pos(mutex_pos_update);
    clear_pending_pos_update_locked();
  }
  set_ekf_steps_running(request->data);
  response->success = true;
  response->message = request->data ? "ekf loop enabled" : "ekf loop disabled";
  RCLCPP_INFO(this->get_logger(), "localslam-> ~/enable %s",
    request->data ? "true" : "false");
}

void EKFslam::Handle_reset_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // Maps to SimpleServ 'i'. Works while enabled or disabled; does not clear
  // DataBuffer (legacy 'i' did not either).
  request_ekf_reinit();
  response->success = true;
  response->message = "ekf re-init requested";
  RCLCPP_INFO(this->get_logger(), "localslam-> ~/reset");
}

void EKFslam::Handle_ekf_run_service(
  const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,
  std::shared_ptr<interfaces::srv::SimpleServ::Response> response)
{
  if (request->cmd == 'r') {
    set_ekf_steps_running(true);
    cout << "localslam-> request received: run ekf loop" << endl;
  }
  if (request->cmd == 's') {
    set_ekf_steps_running(false);
    cout << "localslam-> request received: stop ekf loop" << endl;
  }
  if (request->cmd == 'v') {
    request_activate_visual_update();
    cout << "localslam-> request received: activate visual updates" << endl;
  }
  if (request->cmd == 'i') {
    request_ekf_reinit();
    cout << "localslam-> request received: re-initialize ekf " << endl;
  }
  if (request->cmd == '0') {
    request_log_stats();
  }
  if (request->cmd == 'u') {
    toggle_cl_uncertainty();
  }
  response->response = true;
}

//--------------------------------------------------------------------
 void EKFslam::log_data(STORE &data)
 {

    cout << "----> Local EKF-SLAM stats: <----" << endl;    
    std::printf("Total execution time: %.3f seconds.\n", data.total_exec_time);
    std::printf("Total computation time: %.3f seconds.\n", data.total_comp_time);
    //std::printf("Total real time: %.3f seconds.\n", total_time);
    cout << "Total number of initialized feats: " << data.n_init_feats << endl;
    cout << "Total number of deleted feats: " << data.n_delete_feats << endl;
    cout << "Total number of initialized Anchors: " << data.n_init_anchors << endl;
    cout << "Total number of deleted Anchors: " << data.n_delete_anchors << endl; 

    double mean_tf,std_tf, sum_tf;
    mean_std(data.time_per_frame.second,mean_tf,std_tf,sum_tf);
    cout << "Mean time per frame: " << mean_tf << "s  Std. time per frame: " << std_tf << "s" << endl;
    double mean_feats,std_feats,sum_feats;
    mean_std(data.n_feats_per_frame.second,mean_feats,std_feats,sum_feats);
    cout << "Mean feats per frame: " << mean_feats << "  Std. Feats per frame: " << std_feats  << endl;
    double mean_anchors,std_anchors,sum_anchors;
    mean_std(data.n_anchors_per_frame.second,mean_anchors,std_anchors,sum_anchors);
    cout << "Mean Anchors per frame: " << mean_anchors << "  Std. Anchors per frame: " << std_anchors  << endl;

    std::ofstream myfile("log_ekf");
    if (myfile.is_open()) {       
        myfile << "Total execution time (s): " << data.total_exec_time <<  endl;
        myfile << "Total computation time (s): " << data.total_comp_time <<  endl;
        myfile << "Total number of initialized feats: " << data.n_init_feats << endl;
        myfile << "Total number of deleted feats: " << data.n_delete_feats << endl;
        myfile << "Total number of initialized Anchors: " << data.n_init_anchors << endl;
        myfile << "Total number of deleted Anchors: " << data.n_delete_anchors << endl; 
        myfile << "Mean time per frame: " << mean_tf << "s  Std. time per frame: " << std_tf << "s" << endl;
        myfile << "Mean feats per frame: " << mean_feats << "  Std. Feats per frame: " << std_feats  << endl;
        myfile << "Mean Anchors per frame: " << mean_anchors << "  Std. Anchors per frame: " << std_anchors  << endl;       
        myfile.close();
        //std::cout << "File created or overwritten successfully.\n";
    }
    else {
        std::cout << "Unable to create or open the file.\n";
    }    
    
    
    
    log_data_to_file("log_ekf_time_per_frame",data.time_per_frame);
    log_data_to_file("log_ekf_feats_per_frame",data.n_feats_per_frame);
    log_data_to_file("log_ekf_anchors_per_frame",data.n_anchors_per_frame);

 }

 void EKFslam::log_data_to_file(string file_name,std::pair<std::vector<double>,std::vector<double>> &data)
 {
    std::ofstream myfile(file_name);
    if (myfile.is_open()) {
        
        for (int i = 0; i < data.first.size(); i++)
        {
          myfile << data.first[i] << "," << data.second[i] << endl;
        }       
        
        myfile.close();
        //std::cout << "File created or overwritten successfully.\n";
    }
    else {
        std::cout << "Unable to create or open the file.\n";
    }

 }

 void EKFslam::mean_std(std::vector<double> &data, double &mean, double &std, double &sum)
{
    sum = 0;
    for (int i = 0; i < data.size() ; i++ )
    {     
        sum = sum + data[i];
    }    
    mean = sum/data.size();

    double sum_sq = 0;
    for (int i = 0; i < data.size() ; i++ )
    { 
        sum_sq = sum_sq + pow(data[i] - mean,2 );
    }
    std = sqrt(sum_sq/data.size());
}




}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(localslam::EKFslam)