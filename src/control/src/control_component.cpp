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
    

    // Set Services
    srv_control_run_ = this->create_service<interfaces::srv::SimpleServ>("control_run_service",std::bind(&ControlQuad::Handle_control_run_service, this,_1,_2));
    
    // Set client
    client_ekf_run_ = create_client<interfaces::srv::SimpleServ>("ekf_run_service");
    client_ekf_enable_ = create_client<std_srvs::srv::SetBool>(PAR.Ekf_enable_service);
    client_gslam_run_ = create_client<interfaces::srv::SimpleServ>("globalslam_run_service");
    client_webot_run_ = create_client<interfaces::srv::SimpleServ>("webot_service");
    client_plot_ = create_client<interfaces::srv::SimpleServ>("plot_service");

    RCLCPP_INFO(
      this->get_logger(),
      "Control Slam_feedback=%s Slam_feedback_interface=%s odom_topic=%s",
      PAR.Slam_feedback ? "true" : "false",
      PAR.Slam_feedback_interface.c_str(),
      PAR.Odometry_topic.c_str());

    run_control_plan = false;
    control_thread_loop_run = true;
    cl_flag = false;

    

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

  }
  //-----------------------------------------------------------
  void ControlQuad::setSubscribers()
  {
    sub_Imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&ControlQuad::Imu_callback, this, _1));
    sub_Gps_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "webot_robot/gps", 10, std::bind(&ControlQuad::Gps_callback, this, _1));
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
    if (request_ekf_reinit && client_ekf_run_) {
      auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
      request->cmd = 'i';
      (void)client_ekf_run_->async_send_request(request);
      RCLCPP_INFO(
        this->get_logger(),
        "Coordinated reset: radius disarmed/cleared; requested EKF re-init ('i')");
    }
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
        auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
        request->cmd = 't';              
        auto res = client_webot_run_->async_send_request(request);
        result = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // 
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
        result = go_point(r_d);      
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
        result = recognize_home();
        if(result == true)
        {
          auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
          request->cmd = 'u';              
          auto res = client_gslam_run_->async_send_request(request);
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
      if (request->cmd == 'r')
      { 
        mutex_run_plan.lock();
        if(run_control_plan == false)
        {
          cout << "control-> request received: Run control plan" << endl;
          
            run_control_plan = true;
          
        }
        else
        {
          cout << "control-> request received: Stop control plan" << endl;
          
          run_control_plan = false;
          
        } 
        mutex_run_plan.unlock();         
      }
      if (request->cmd == 'i')
      { 
        cout << "control-> request received: Re-init control plan (coordinated)" << endl;
        mutex_run_plan.lock();
            run_control_plan = false;
        mutex_run_plan.unlock();
        // A6: invalidate radius before/with EKF re-init; re-arm on next odometry.
        invalidate_cached_radius(true);
        reinit_control();
      }
      if (request->cmd == 'c')
      {
        cout << "control-> request received: Close loop detected" << endl;
        mutex_cl.lock();
          cl_flag = true;
        mutex_cl.unlock();
      }

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
