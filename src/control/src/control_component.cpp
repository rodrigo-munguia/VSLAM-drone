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

#include "control_component.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <limits>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include "../../common/ros_slam_identity.hpp"



using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;

namespace control
{

// Create a Talker "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.

  

  ControlQuad::ControlQuad(const rclcpp::NodeOptions & options)
  : Node("control_quad", options)
  { 
    setParameters();
    setSubscribers();
    reinit_control();
    
    
    // Set subscribers
    //sub_Imu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&WEBOTb::Imu_callback, this, _1)); 
    
    // Set publisher       
    //pub_Alt_ = create_publisher<interfaces::msg::Alt>("Altitude_topic",10);
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
    {
      rclcpp::QoS qos_fb(rclcpp::KeepLast(1));
      qos_fb.reliable();
      qos_fb.transient_local();
      pub_correction_feedback_accepted_ =
        create_publisher<interfaces::msg::CorrectionFeedbackAccepted>(
          "correction_feedback_accepted", qos_fb);
    }

    // Set Services
    srv_control_run_ = this->create_service<interfaces::srv::SimpleServ>("control_run_service",std::bind(&ControlQuad::Handle_control_run_service, this,_1,_2));
    srv_correction_coord_ = this->create_service<interfaces::srv::CorrectionCoordinate>(
      "control_correction_coordinate",
      std::bind(&ControlQuad::Handle_correction_coordinate, this, _1, _2));
    
    // Set client
    client_ekf_run_ = create_client<interfaces::srv::SimpleServ>("ekf_run_service");
    client_ekf_enable_ = create_client<std_srvs::srv::SetBool>(PAR.Ekf_enable_service);
    client_gslam_run_ = create_client<interfaces::srv::SimpleServ>("globalslam_run_service");
    client_webot_run_ = create_client<interfaces::srv::SimpleServ>("webot_service");
    client_plot_ = create_client<interfaces::srv::SimpleServ>("plot_service");
    client_unified_traj_ =
      create_client<interfaces::srv::UnifiedTrajectory>("unified_trajectory");

    RCLCPP_INFO(
      this->get_logger(),
      "Control Slam_feedback=%s Slam_feedback_interface=%s odom_topic=%s Unified_mission=%s",
      PAR.Slam_feedback ? "true" : "false",
      PAR.Slam_feedback_interface.c_str(),
      PAR.Odometry_topic.c_str(),
      PAR.Unified_mission ? "true" : "false");

    run_control_plan = false;
    control_thread_loop_run = true;
    cl_flag = false;
    // GT pose path (Slam_feedback=false) multiplies GPS by Rn2rG before the
    // first IMU sample can populate it. Default-constructed arma::mat is 0x0
    // and throws on GPS. Match WEBOTb: identity until attitude is available.
    Rn2rG.eye(3, 3);

    

    control_loop_ = thread(&ControlQuad::Control_Loop,this);

    logger_gp = new LoggerGoPoint("logfile_gp.csv"); // initialize logger for go to point

    logger_vp = new LoggerGoPoint("logfile_vp.csv"); // initialize logger for go to (visual) point
  }

  ControlQuad::~ControlQuad()
  { 
    
  }
   
  //---------------------------------------------------
  // Set paramters 
  void ControlQuad::setParameters()
  {  
    //  Declare node parameters (and default values)
     // Empty → resolve installed ekf_monoslam_bringup/control_plan.txt (cwd-independent).
     // Absolute existing path is preserved as an explicit override.
     this->declare_parameter<string>("Control_plan_path", "");
     this->declare_parameter<bool>("Slam_feedback",false);
     this->declare_parameter<std::string>("Slam_feedback_interface", "odometry");
     this->declare_parameter<std::string>("Odometry_topic", "odometry");
     this->declare_parameter<std::string>("Odometry_expect_frame_id", "odom");
     this->declare_parameter<std::string>(
       "Odometry_expect_child_frame_id", "base_link");
     this->declare_parameter<std::string>(
       "Ekf_enable_service", "/EKFslam/enable");
     this->declare_parameter<double>("kp_x", 0.5);
     this->declare_parameter<double>("kp_y", 0.5);
     this->declare_parameter<double>("kp_z", 0.7);
     this->declare_parameter<double>("kp_yaw", 0.7);
     this->declare_parameter<double>("maxControlSignal_xy", 0.5);
     this->declare_parameter<double>("minControlSignal_xy", -0.5);
     this->declare_parameter<double>("maxControlSignal_z", 0.25);
     this->declare_parameter<double>("minControlSignal_z", -0.25);
     this->declare_parameter<double>("maxControlSignal_yaw", 0.2);
     this->declare_parameter<double>("minControlSignal_yaw", -0.2);
     this->declare_parameter<double>("cp_x", 0.05);
     this->declare_parameter<double>("cp_y", 0.05);
     this->declare_parameter<double>("cp_z", 0.05);
     this->declare_parameter<double>("cp_yaw", 0.087);
     this->declare_parameter<int>("Mono_cam_img_rows",240);
     this->declare_parameter<int>("Mono_cam_img_cols",320);
     this->declare_parameter("Mono_cam_distortions", std::vector<double>{0.02921, -0.00504, 0.00297, -0.00843, 0.00000});
     this->declare_parameter<double>("Mono_cam_cc_u",156.24435);
     this->declare_parameter<double>("Mono_cam_cc_v",117.04562);
     this->declare_parameter<double>("Mono_cam_fc_u",206.34225);
     this->declare_parameter<double>("Mono_cam_fc_v",268.65192);
     this->declare_parameter<double>("Mono_cam_alpha_c",0.0);
     this->declare_parameter<double>("Mono_cam_2_robot_axis_x",0.10);
     this->declare_parameter<double>("Mono_cam_2_robot_axis_y",0.0);
     this->declare_parameter<double>("Mono_cam_2_robot_axis_z",1.5707963268);
     this->declare_parameter<double>("Mono_cam_2_robot_pos_x",0.1);
     this->declare_parameter<double>("Mono_cam_2_robot_pos_y",0.0);
     this->declare_parameter<double>("Mono_cam_2_robot_pos_z",0.0);
     this->declare_parameter<double>("Explore_area_lambda_y",-0.25);  
     this->declare_parameter<double>("Rec_Home_lambda_y",-0.5 );
     this->declare_parameter<bool>("Unified_mission", true);
     this->declare_parameter<double>("Unified_traj_timeout_s", 120.0);
      
    
    // Set parameter struct
     this->get_parameter("Control_plan_path",PAR.Control_plan_path);
     // Empty / relative → installed ekf_monoslam_bringup resource.
     // Absolute existing path is preserved (explicit override).
     if (PAR.Control_plan_path.empty() || PAR.Control_plan_path[0] != '/') {
       try {
         const std::string share =
           ament_index_cpp::get_package_share_directory("ekf_monoslam_bringup");
         const std::string installed = share + "/control_plan.txt";
         if (PAR.Control_plan_path.empty()) {
           PAR.Control_plan_path = installed;
         } else {
           // relative: prefer installed default for cwd independence
           PAR.Control_plan_path = installed;
         }
         RCLCPP_INFO(this->get_logger(),
           "Control_plan_path resolved to %s", PAR.Control_plan_path.c_str());
       } catch (const std::exception & e) {
         RCLCPP_WARN(this->get_logger(),
           "Could not resolve ekf_monoslam_bringup share (%s); "
           "using Control_plan_path='%s'", e.what(), PAR.Control_plan_path.c_str());
       }
     }
     this->get_parameter("Slam_feedback",PAR.Slam_feedback);
     this->get_parameter("Slam_feedback_interface", PAR.Slam_feedback_interface);
     this->get_parameter("Odometry_topic", PAR.Odometry_topic);
     this->get_parameter(
       "Odometry_expect_frame_id", PAR.Odometry_expect_frame_id);
     this->get_parameter(
       "Odometry_expect_child_frame_id", PAR.Odometry_expect_child_frame_id);
     this->get_parameter("Ekf_enable_service", PAR.Ekf_enable_service);
     this->get_parameter("kp_x",PAR.kp_x); 
     this->get_parameter("kp_y",PAR.kp_y); 
     this->get_parameter("kp_z",PAR.kp_z); 
     this->get_parameter("kp_yaw",PAR.kp_yaw);
     this->get_parameter("maxControlSignal_xy",PAR.maxControlSignal_xy); 
     this->get_parameter("minControlSignal_xy",PAR.minControlSignal_xy);
     this->get_parameter("maxControlSignal_z",PAR.maxControlSignal_z); 
     this->get_parameter("minControlSignal_z",PAR.minControlSignal_z);
     this->get_parameter("maxControlSignal_yaw",PAR.maxControlSignal_yaw); 
     this->get_parameter("minControlSignal_yaw",PAR.minControlSignal_yaw); 
     this->get_parameter("cp_x",PAR.cp_x);
     this->get_parameter("cp_y",PAR.cp_y);  
     this->get_parameter("cp_z",PAR.cp_z);  
     this->get_parameter("cp_yaw",PAR.cp_yaw); 
     this->get_parameter("Mono_cam_img_rows",PAR.Mono_cam_img_rows);
     this->get_parameter("Mono_cam_img_cols",PAR.Mono_cam_img_cols); 
     rclcpp::Parameter Mono_cam_distortions_par = this->get_parameter("Mono_cam_distortions");
     PAR.Mono_cam_distortions = Mono_cam_distortions_par.as_double_array();
     this->get_parameter("Mono_cam_cc_u",PAR.Mono_cam_cc_u);
     this->get_parameter("Mono_cam_cc_v",PAR.Mono_cam_cc_v);
     this->get_parameter("Mono_cam_fc_u",PAR.Mono_cam_fc_u);
     this->get_parameter("Mono_cam_fc_v",PAR.Mono_cam_fc_v);
     this->get_parameter("Mono_cam_alpha_c",PAR.Mono_cam_alpha_c);
     this->get_parameter("Mono_cam_2_robot_axis_x",PAR.Mono_cam_2_robot_axis_x);
     this->get_parameter("Mono_cam_2_robot_axis_y",PAR.Mono_cam_2_robot_axis_y);
     this->get_parameter("Mono_cam_2_robot_axis_z",PAR.Mono_cam_2_robot_axis_z);
     this->get_parameter("Mono_cam_2_robot_pos_x",PAR.Mono_cam_2_robot_pos_x);
     this->get_parameter("Mono_cam_2_robot_pos_y",PAR.Mono_cam_2_robot_pos_y);
     this->get_parameter("Mono_cam_2_robot_pos_z",PAR.Mono_cam_2_robot_pos_z);
     this->get_parameter("Explore_area_lambda_y",PAR.Explore_area_lambda_y);
     this->get_parameter("Rec_Home_lambda_y",PAR.Rec_Home_lambda_y);
     this->get_parameter("Unified_mission", PAR.Unified_mission);
     this->get_parameter("Unified_traj_timeout_s", PAR.Unified_traj_timeout_s);

  }
  //-----------------------------------------------------------
  void ControlQuad::setSubscribers()
  {
    sub_Imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&ControlQuad::Imu_callback, this, _1));
    sub_Gps_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "webot_robot/gps", 10, std::bind(&ControlQuad::Gps_callback, this, _1));
    sub_unified_traj_status_ =
      this->create_subscription<interfaces::msg::UnifiedTrajectoryStatus>(
        "unified_trajectory_status", 10,
        std::bind(&ControlQuad::UnifiedTrajStatus_callback, this, _1));
    sub_Frame_ = this->create_subscription<interfaces::msg::Frame>(
      "Frame_topic", 10, std::bind(&ControlQuad::Frame_callback, this, _1));

    // Pose feedback source is exclusive (legacy XOR odometry).
    // A5b: odometry mode additionally subscribes to Robotstate for
    // radius_uncertainty only (intentional custom-interface exception).
    if (PAR.Slam_feedback_interface == "odometry") {
      sub_Odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        PAR.Odometry_topic, 10,
        std::bind(&ControlQuad::Odometry_callback, this, _1));
      sub_Slam_ = this->create_subscription<interfaces::msg::Robotstate>(
        "Robotstate_topic", 10,
        std::bind(&ControlQuad::Slam_radius_only_callback, this, _1));
      RCLCPP_INFO(
        this->get_logger(),
        "SLAM pose from Odometry '%s' (expect frames %s / %s); "
        "radius_uncertainty from Robotstate_topic (A5b)",
        PAR.Odometry_topic.c_str(),
        PAR.Odometry_expect_frame_id.c_str(),
        PAR.Odometry_expect_child_frame_id.c_str());
    } else if (PAR.Slam_feedback_interface == "legacy") {
      sub_Slam_ = this->create_subscription<interfaces::msg::Robotstate>(
        "Robotstate_topic", 10,
        std::bind(&ControlQuad::Slam_callback, this, _1));
      RCLCPP_INFO(this->get_logger(), "SLAM feedback from legacy Robotstate_topic");
    } else {
      throw std::runtime_error(
        "Unsupported Slam_feedback_interface='" + PAR.Slam_feedback_interface +
        "'. Use 'legacy' or 'odometry'.");
    }

    // G6b: always listen for correction association (GT and odometry barriers).
    {
      rclcpp::QoS qos_corr(rclcpp::KeepLast(1));
      qos_corr.reliable();
      qos_corr.transient_local();
      sub_CorrectionApplied_ =
        this->create_subscription<interfaces::msg::CorrectionApplied>(
          "correction_applied", qos_corr,
          std::bind(&ControlQuad::CorrectionApplied_feedback_callback, this, _1));
      sub_GlobalLoopCommitted_ =
        this->create_subscription<interfaces::msg::GlobalLoopCommitted>(
          "global_loop_committed", qos_corr,
          std::bind(&ControlQuad::GlobalLoopCommitted_callback, this, _1));
    }
    sub_OdomMeta_ = this->create_subscription<interfaces::msg::OdometryMeta>(
      "odometry_meta", 10,
      std::bind(&ControlQuad::OdometryMeta_callback, this, _1));
  }
  //-----------------------------------------------------------
  void ControlQuad::invalidate_cached_radius(bool request_ekf_reinit)
  {
    // Disarm first so any queued Robotstate callback cannot restore r_u.
    radius_feedback_armed_ = false;
    // Gate later Odometry re-arm on stamp: queued pre-reset Odometry messages
    // keep older header.stamp and must not re-arm radius acceptance.
    radius_reset_stamp_ = this->get_clock()->now();
    radius_reset_stamp_valid_ = true;
    mutex_robot_state.lock();
    r_c.r_u = std::numeric_limits<double>::quiet_NaN();
    mutex_robot_state.unlock();
    mutex_slam_state.lock();
    r_s.r_u = std::numeric_limits<double>::quiet_NaN();
    mutex_slam_state.unlock();
    {
      std::lock_guard<std::mutex> lock(mutex_coord_);
      clear_feedback_barrier_locked();
    }
    if (request_ekf_reinit && client_ekf_run_) {
      auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
      request->cmd = 'i';
      (void)client_ekf_run_->async_send_request(request);
      RCLCPP_INFO(
        this->get_logger(),
        "Coordinated reset: radius disarmed/cleared; requested EKF re-init ('i')");
    }
  }

  void ControlQuad::clear_feedback_barrier_locked()
  {
    g6_feedback::on_estimator_reset(
      feedback_barrier_, odom_meta_buf_, odom_stamp_buf_, accepted_snap_hist_);
    accepted_odom_stamps_.clear();
  }

  void ControlQuad::publish_feedback_accepted_locked()
  {
    if (!feedback_barrier_.accepted || !pub_correction_feedback_accepted_) {
      return;
    }
    interfaces::msg::CorrectionFeedbackAccepted msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = PAR.Odometry_expect_frame_id;
    msg.session_id = slam_identity::to_msg(feedback_barrier_.session_id);
    msg.epoch_id = feedback_barrier_.epoch_id;
    msg.correction_id = feedback_barrier_.correction_id;
    msg.snapshot_seq = feedback_barrier_.accepted_snapshot_seq;
    msg.feedback_mode = feedback_barrier_.feedback_mode;
    pub_correction_feedback_accepted_->publish(msg);
    RCLCPP_INFO(
      this->get_logger(),
      "[G6b-TXN] CorrectionFeedbackAccepted correction_id=%llu "
      "snapshot_seq=%llu mode=%s",
      static_cast<unsigned long long>(msg.correction_id),
      static_cast<unsigned long long>(msg.snapshot_seq),
      msg.feedback_mode.c_str());
  }

  void ControlQuad::note_accepted_snapshot_locked(const g6_feedback::MetaSample & meta)
  {
    g6_feedback::AcceptedSample a;
    a.session_id = meta.session_id;
    a.epoch_id = meta.epoch_id;
    a.snapshot_seq = meta.snapshot_seq;
    a.last_correction_id = meta.last_correction_id;
    g6_feedback::push_accepted(accepted_snap_hist_, a);
    if (feedback_barrier_.armed && !feedback_barrier_.accepted &&
      g6_feedback::meta_satisfies(feedback_barrier_, meta))
    {
      if (g6_feedback::accept_odometry(feedback_barrier_, meta.snapshot_seq)) {
        publish_feedback_accepted_locked();
      }
    }
  }

  void ControlQuad::try_pair_odometry_meta(int32_t stamp_sec, uint32_t stamp_nanosec)
  {
    std::lock_guard<std::mutex> lock(mutex_coord_);
    auto meta = g6_feedback::find_meta_for_stamp(
      odom_meta_buf_, stamp_sec, stamp_nanosec);
    if (!meta) {
      return;
    }
    // Pose must already have been accepted for this stamp (or will be soon).
    bool pose_accepted = false;
    for (const auto & s : accepted_odom_stamps_) {
      if (s.first == stamp_sec && s.second == stamp_nanosec) {
        pose_accepted = true;
        break;
      }
    }
    if (!pose_accepted) {
      return;
    }
    note_accepted_snapshot_locked(*meta);
  }

  void ControlQuad::reinit_control()
  {
    cmd_idx = 0;
    actual_z_flag = false;
    slam_yaw_init_flag_ = false;
    slam_init_yaw_ = 0.0;
    slam_init_z_ = 0.0;
    // Constructor and plan reload: clear radius without requiring EKF up yet.
    invalidate_cached_radius(false);
    double cam_axis_x = PAR.Mono_cam_2_robot_axis_x;
    double cam_axis_y = PAR.Mono_cam_2_robot_axis_y;
    double cam_axis_z = PAR.Mono_cam_2_robot_axis_z;
    

    cam_parameters.distortions = &PAR.Mono_cam_distortions[0];
    cam_parameters.cc[0] = PAR.Mono_cam_cc_u;
    cam_parameters.cc[1] = PAR.Mono_cam_cc_v;
    cam_parameters.fc[0] = PAR.Mono_cam_fc_u;
    cam_parameters.fc[1] = PAR.Mono_cam_fc_v;
    cam_parameters.alpha_c = PAR.Mono_cam_alpha_c;
    t_c2r(0) = PAR.Mono_cam_2_robot_pos_x;
    t_c2r(1) = PAR.Mono_cam_2_robot_pos_y;
    t_c2r(2) = PAR.Mono_cam_2_robot_pos_z;  
    double Ra2b_c[9];
    Euler_to_Ra2b_colum_major(cam_axis_x, cam_axis_y, cam_axis_z, Ra2b_c);
    arma::mat Rr2c_t(Ra2b_c,3,3); // camera to robot rotation matrix 
    Rr2c = Rr2c_t;
    load_control_plan();

  }
  //-----------------------------------------------------------
  void ControlQuad::load_control_plan()
  {     
    //cout << PAR.Control_plan_path << endl;   

    ifstream file_Control_plan;
    file_Control_plan.open(PAR.Control_plan_path);   // open file for reading    

    string cmd_line;
     while ( getline(file_Control_plan,cmd_line))
        {
            stringstream ss(cmd_line);
            vector<string> cmd_args;
            while( ss.good() ) // parse each comma-separated element into a vector of strings
            {
                string substr;
                getline( ss, substr, ',' );
                cmd_args.push_back( substr );
            }
            //string cmd = cmd_args[0];            
            CMD cmd;
            cmd.name = cmd_args[0];
            for(int i = 1; i < cmd_args.size();i++)
            {
              cmd.args.push_back(stod(cmd_args[i]));              
            }
            Control_plan.push_back(cmd);
        }
        /*
        for(auto c:Control_plan)
        {
          cout << c.name << endl;
        }
        */        
        file_Control_plan.close();
  }
  //----------------------------------------------------------------------
  //  Main Plot loop (this function runs in a separate thread)
  void ControlQuad::Control_Loop()
  {

    cout << "-> Control thread running... " << endl; 
    while(control_thread_loop_run == true)  // thread main loop
    {      

      //cout << "actual: " << r_a.x << " " << r_a.y << "  " << r_a.z << " att: " << r_a.roll << " " << r_a.pitch << " " << r_a.yaw   << endl;
      //cout << "slam  : " << r_s.x << " " << r_s.y << "  " << r_s.z << " att: " << r_s.roll  << " " << r_s.pitch << " " << r_s.yaw  << endl; 
      
      mutex_run_plan.lock();
        bool run = run_control_plan; 
      mutex_run_plan.unlock();
      
          if(run == true)
          {
            bool result = execute_cmd();
            if(result == false)run_control_plan = false;
          }
      

      std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep a short period of time to save proccesor use
    }

  }
  //----------------------------------------------------------------------------
  bool ControlQuad::execute_cmd()
  {

      //cout << "cmd_idx: " <<  cmd_idx <<  " Control_plan.size(): " << Control_plan.size() << endl;

      if (cmd_idx == Control_plan.size())
      {
          cout << "control-> no more commands to execute" << endl;
          return false;
      }


      string cmd = Control_plan[cmd_idx].name;
      bool result = false;

      //cout << "cmd: " << cmd << endl;

      if(cmd == "TakeOff")
      {
        cout << "control-> Take off" << endl;
        // Wait for webot_service — async fire-and-forget left K_VERTICAL_THRUST=0
        // on some S5a launches (vehicle sat on ground with cmd_z=1.8).
        if (!client_webot_run_->wait_for_service(std::chrono::seconds(10))) {
          cout << "control-> TakeOff: webot_service unavailable" << endl;
          result = false;
        } else {
          auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
          request->cmd = 't';
          auto fut = client_webot_run_->async_send_request(request);
          if (fut.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            result = true;
          } else {
            cout << "control-> TakeOff: webot_service call timed out" << endl;
            result = false;
          }
          // R4d: after arming, latch unified ownership once (no legacy climb).
          if (result && PAR.Unified_mission) {
            auto ureq = std::make_shared<interfaces::srv::SimpleServ::Request>();
            ureq->cmd = 'u';
            auto ufut = client_webot_run_->async_send_request(ureq);
            if (ufut.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
              cout << "control-> TakeOff: unified latch timed out" << endl;
              result = false;
            } else {
              cout << "control-> TakeOff: unified mode latched (arm only)" << endl;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
      }
      if(cmd == "Land")
      {
        cout << "control-> Landing" << endl;
        auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
        request->cmd = 'l';              
        auto res = client_webot_run_->async_send_request(request);
        result = true;
      }
      if(cmd == "pointV")
      {
        ROBOT_STATE r_d;  // desired robot state
        r_d.x = Control_plan[cmd_idx].args[0];
        r_d.y = Control_plan[cmd_idx].args[1];
        r_d.z = Control_plan[cmd_idx].args[2];
        r_d.yaw = Control_plan[cmd_idx].args[3];
        result = go_point_using_visual_mark(r_d);            
      } 
      if(cmd == "pointa")
      {
        ROBOT_STATE r_d;  // desired robot state
        r_d.x = Control_plan[cmd_idx].args[0];
        r_d.y = Control_plan[cmd_idx].args[1];
        r_d.z = Control_plan[cmd_idx].args[2];
        r_d.yaw = Control_plan[cmd_idx].args[3];
        if (PAR.Unified_mission) {
          result = go_point_unified(r_d);
        } else {
          result = go_point(r_d);
        }
      }
      if(cmd == "StartSLAM")
      {
        cout << "control-> start SLAM (SetBool enable=true)" << endl;
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        (void)client_ekf_enable_->async_send_request(request);
        result = true;
      }
      if(cmd == "ActivateVisualUpdate")
      {
        cout << "control-> Activate SLAM visual updates" << endl;
        auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
        request->cmd = 'v';
        auto res = client_ekf_run_->async_send_request(request);
        result = true;
      }

      if(cmd == "StopSLAM")
      {
        cout << "control-> Stop SLAM (SetBool enable=false)" << endl;
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;
        (void)client_ekf_enable_->async_send_request(request);
        result = true;
      }
      if(cmd == "StartUncertainty")
      {
        cout << "control-> start SLAM/CL uncertanty" << endl;
        auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
        request->cmd = 'u';              
        auto res = client_ekf_run_->async_send_request(request);
        result = true;

      }
      if(cmd == "ExploreArea")
      {
        double x_a = Control_plan[cmd_idx].args[0];
        double y_a = Control_plan[cmd_idx].args[1];
        result = explore_area(x_a,y_a);
      }
      if(cmd == "RecHome")
      { 
        cout << "control-> start home area recognition " << endl;
        // Begin capture before exploration (GS 'b'); finalize after ('u').
        {
          auto req_b = std::make_shared<interfaces::srv::SimpleServ::Request>();
          req_b->cmd = 'b';
          (void)client_gslam_run_->async_send_request(req_b);
        }
        result = recognize_home();
        if(result == true)
        {
          auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
          request->cmd = 'u';
          // Wait for finalize so plan can stop if capture failed (no zero-origin fallback).
          auto future = client_gslam_run_->async_send_request(request);
          const auto wait_st = future.wait_for(std::chrono::seconds(3));
          if (wait_st == std::future_status::ready) {
            const auto resp = future.get();
            if (!resp || !resp->response) {
              cout << "control-> RecHome capture FAILED (no ready home-tag reference); "
                   << "withholding home recognition success" << endl;
              result = false;
            } else {
              cout << "control-> RecHome capture READY" << endl;
            }
          } else {
            cout << "control-> RecHome finalize timed out; treating as failure" << endl;
            result = false;
          }
        }
      }
      if(cmd=="GoHome")
      { 
        cout << "Control-> going to home " << endl; 
        // first activate Close Loop in gslam component
        auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
        request->cmd = 'c';              
        auto res = client_gslam_run_->async_send_request(request);
        // 
        result = go_home();

      }

      if(result == true)
      {
        cmd_idx++; // if command result was succes continue increment command index.
        return true;
      }

      return false; // 
  }

  //----------------------------------------------------------------------------
  void ControlQuad::Handle_control_run_service(const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,std::shared_ptr<interfaces::srv::SimpleServ::Response> response) 
    { 
      // Match EKFslam SimpleServ: acknowledge handled cmds. Leaving the bool
      // unset left response=false while the plan still started (B2/B3 false
      // negative on ros2 service call).
      bool handled = false;
      if (request->cmd == 'r')
      { 
        // Plan start/stop toggle. Keyboard uses 'a' → this cmd (not teleop 'r').
        mutex_run_plan.lock();
        if(run_control_plan == false)
        {
          cout << "control-> request received: Run control plan" << endl;
          
            run_control_plan = true;
          coord_suppress_cmd_ = false;
          
        }
        else
        {
          cout << "control-> request received: Stop control plan" << endl;
          
          run_control_plan = false;
          // Operator stop while a G6a pause is owned: block auto-resume.
          {
            std::lock_guard<std::mutex> lock(mutex_coord_);
            g6_coord::mark_operator_superseded(coord_snap_);
          }
          
        } 
        mutex_run_plan.unlock();
        handled = true;
      }
      if (request->cmd == 'h')
      {
        // G6b.2: explicit coordination-hold recovery (nonconflicting with
        // keyboard teleop 'r'=forward). Clears suppress and resumes plan.
        mutex_run_plan.lock();
        coord_suppress_cmd_ = false;
        run_control_plan = true;
        mutex_run_plan.unlock();
        cout << "control-> request received: Recover coordination hold "
                "(plan resume)" << endl;
        RCLCPP_INFO(
          this->get_logger(),
          "[G6b-TXN] operator hold recovery via SimpleServ 'h'");
        handled = true;
      }
      if (request->cmd == 'i')
      { 
        cout << "control-> request received: Re-init control plan (coordinated)" << endl;
        mutex_run_plan.lock();
            run_control_plan = false;
        mutex_run_plan.unlock();
        {
          std::lock_guard<std::mutex> lock(mutex_coord_);
          g6_coord::mark_operator_superseded(coord_snap_);
          coord_suppress_cmd_ = false;
        }
        // A6: invalidate radius before/with EKF re-init; re-arm on next odometry.
        invalidate_cached_radius(true);
        reinit_control();
        handled = true;
      }
      if (request->cmd == 'c')
      {
        // Legacy close-loop interrupt (go_home re-nav). G6a coordinated path
        // prefers CorrectionCoordinate release with signal_close_loop instead
        // of also sending this command (avoid double completion actions).
        cout << "control-> request received: Close loop detected" << endl;
        mutex_cl.lock();
          cl_flag = true;
        mutex_cl.unlock();
        handled = true;
      }
      response->response = handled;
   }

  void ControlQuad::publish_zero_twist()
  {
    auto twist_msg = geometry_msgs::msg::Twist();
    pub_cmd_vel_->publish(twist_msg);
  }

  bool ControlQuad::wait_cmd_quiescent(std::chrono::milliseconds timeout)
  {
    // Plan command routines clear in_plan_command_ on exit; suppress forces
    // zero twist while they wind down. Acknowledgement ≠ vehicle speed zero.
    std::unique_lock<std::mutex> lock(mutex_coord_);
    return cv_coord_quiet_.wait_for(lock, timeout, [this]() {
      return !in_plan_command_.load();
    });
  }

  void ControlQuad::Handle_correction_coordinate(
    const std::shared_ptr<interfaces::srv::CorrectionCoordinate::Request> request,
    std::shared_ptr<interfaces::srv::CorrectionCoordinate::Response> response)
  {
    using slam_identity::from_msg;
    g6_coord::TxnIdentity req;
    req.session_id = from_msg(request->session_id);
    req.epoch_id = request->epoch_id;
    req.transaction_id = request->transaction_id;
    req.owner_generation = request->owner_generation;

    response->success = false;
    response->state = "idle";
    response->reason.clear();
    response->plan_was_active = false;
    response->plan_resumed = false;
    response->cmd_source_quiescent = false;
    response->active_transaction_id = 0;
    response->active_owner_generation = 0;

    const std::string op = request->operation;
    g6_coord::CoordResult result;

    if (op == "prepare_pause") {
      bool plan_active = false;
      {
        std::lock_guard<std::mutex> lock_run(mutex_run_plan);
        plan_active = run_control_plan;
        // Hold plan progression for this transaction (not physical hover).
        run_control_plan = false;
      }
      coord_suppress_cmd_ = true;
      publish_zero_twist();
      {
        std::lock_guard<std::mutex> lock(mutex_coord_);
        result = g6_coord::prepare_pause(
          coord_snap_, req, plan_active, !in_plan_command_.load());
      }
      if (!result.cmd_source_quiescent) {
        const bool quiet = wait_cmd_quiescent(std::chrono::milliseconds(1500));
        publish_zero_twist();
        std::lock_guard<std::mutex> lock(mutex_coord_);
        result = g6_coord::prepare_pause(
          coord_snap_, req, coord_snap_.plan_was_active, quiet);
        result.cmd_source_quiescent = quiet;
        result.success = quiet;
        if (!quiet) {
          result.reason = "cmd_quiescence_timeout";
        }
      }
    } else if (op == "release") {
      bool resume = false;
      {
        std::lock_guard<std::mutex> lock(mutex_coord_);
        // Resume only if this release matches the pause identity (GS must not
        // release across Local SLAM session/epoch changes).
        const bool id_ok = g6_coord::same_txn(coord_snap_.active, req);
        // Odometry feedback: refuse resume-capable release until Control has
        // accepted the required corrected snapshot (or newer compatible).
        // GT feedback does not require corrected odometry for resumption.
        const bool odom_fb =
          PAR.Slam_feedback && PAR.Slam_feedback_interface == "odometry";
        if (request->resume_plan_if_owned && odom_fb) {
          const bool barrier_ok =
            feedback_barrier_.accepted &&
            feedback_barrier_.correction_id == req.transaction_id &&
            slam_identity::equal(feedback_barrier_.session_id, req.session_id) &&
            feedback_barrier_.epoch_id == req.epoch_id;
          if (!barrier_ok) {
            result = g6_coord::fill_from(
              coord_snap_, false, "feedback_barrier_not_satisfied");
            result.cmd_source_quiescent = !in_plan_command_.load();
            response->success = result.success;
            response->state = g6_coord::state_name(result.state);
            response->reason = result.reason;
            response->plan_was_active = result.plan_was_active;
            response->plan_resumed = false;
            response->cmd_source_quiescent = result.cmd_source_quiescent;
            response->active_transaction_id = result.active_transaction_id;
            response->active_owner_generation = result.active_owner_generation;
            RCLCPP_WARN(
              this->get_logger(),
              "CorrectionCoordinate release blocked: feedback barrier "
              "not satisfied for txn=%llu",
              static_cast<unsigned long long>(req.transaction_id));
            return;
          }
        }
        result = g6_coord::release(
          coord_snap_, req, request->resume_plan_if_owned, id_ok);
        resume = result.plan_resumed;
        if (result.success) {
          coord_suppress_cmd_ = false;
          // Barrier is consumed with a successful release of this txn.
          if (feedback_barrier_.correction_id == req.transaction_id) {
            g6_feedback::clear_barrier(feedback_barrier_);
          }
        }
      }
      if (result.success && request->signal_close_loop) {
        std::lock_guard<std::mutex> lock_cl(mutex_cl);
        cl_flag = true;
      }
      if (resume) {
        std::lock_guard<std::mutex> lock_run(mutex_run_plan);
        run_control_plan = true;
        RCLCPP_INFO(
          this->get_logger(),
          "[G6b-TXN] plan resumed after feedback barrier txn=%llu",
          static_cast<unsigned long long>(req.transaction_id));
      }
    } else if (op == "abort") {
      bool resume = false;
      {
        std::lock_guard<std::mutex> lock(mutex_coord_);
        // resume_plan_if_owned: only definite non-application (GS). Applied
        // but unresolved feedback must not resume odometry control.
        result = g6_coord::abort_txn(
          coord_snap_, req, request->resume_plan_if_owned);
        if (result.success) {
          if (feedback_barrier_.correction_id == req.transaction_id) {
            g6_feedback::clear_barrier(feedback_barrier_);
          }
          resume = result.plan_resumed;
          if (resume) {
            // Definite non-application: allow parked commands to continue.
            coord_suppress_cmd_ = false;
          } else {
            // Unresolved/applied hold: keep suppress armed so go_point park
            // does not treat this as OperatorStop (would return false and
            // advance/stop the plan). Operator 'r' clears suppress.
            coord_suppress_cmd_ = true;
          }
        }
      }
      if (resume) {
        std::lock_guard<std::mutex> lock_run(mutex_run_plan);
        run_control_plan = true;
        RCLCPP_INFO(
          this->get_logger(),
          "[G6b-TXN] plan resumed after abort (non-applied) txn=%llu",
          static_cast<unsigned long long>(req.transaction_id));
      } else if (result.success) {
        RCLCPP_WARN(
          this->get_logger(),
          "[G6b-TXN] abort without plan resume txn=%llu "
          "(unresolved/applied hold or idle plan) — recover with "
          "SimpleServ 'h' (keyboard: not teleop 'r'; use operator hold-recover)",
          static_cast<unsigned long long>(req.transaction_id));
      }
    } else if (op == "query") {
      std::lock_guard<std::mutex> lock(mutex_coord_);
      result = g6_coord::query(coord_snap_, req);
      result.cmd_source_quiescent = !in_plan_command_.load() && !coord_suppress_cmd_.load();
    } else {
      result.success = false;
      result.reason = "unknown_operation";
      result.state = g6_coord::State::Idle;
    }

    response->success = result.success;
    response->state = g6_coord::state_name(result.state);
    response->reason = result.reason;
    response->plan_was_active = result.plan_was_active;
    response->plan_resumed = result.plan_resumed;
    response->cmd_source_quiescent = result.cmd_source_quiescent;
    response->active_transaction_id = result.active_transaction_id;
    response->active_owner_generation = result.active_owner_generation;
    RCLCPP_INFO(
      this->get_logger(),
      "CorrectionCoordinate op=%s success=%d state=%s reason=%s txn=%llu",
      op.c_str(), result.success ? 1 : 0, response->state.c_str(),
      result.reason.c_str(),
      static_cast<unsigned long long>(request->transaction_id));
  }

  void ControlQuad::SaveScreenShot()
  {
    auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
    request->cmd = '9';
    auto result = client_plot_->async_send_request(request); 
  }        
    


}// namespace control


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(control::ControlQuad)
