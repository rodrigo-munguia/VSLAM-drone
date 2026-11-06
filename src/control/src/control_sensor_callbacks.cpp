#include "control_component.hpp"
#include "control_feedback_conversions.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
using namespace std;


namespace control
{
  
  double ControlQuad::restrictAngle(double angle) {
    // Normalize the angle to the range -2π to +2π
    angle = fmod(angle, 2 * M_PI);

    // Adjust the angle to the range -π to +π
    if (angle <= -M_PI)
        angle += 2 * M_PI;
    else if (angle > M_PI)
        angle -= 2 * M_PI;

    return angle;
}


  void ControlQuad::Imu_callback(const sensor_msgs::msg::Imu & msg) 
  { 
    static double init_yaw;   
    static bool init_yaw_flag = false;
    //cout << msg.orientation.x << " " << msg.orientation.y << " "<<  msg.orientation.z << " " << msg.orientation.w << endl;
    tf2::Quaternion quat;
    quat.setX(msg.orientation.x);
    quat.setY(msg.orientation.y);
    quat.setZ(msg.orientation.z);
    quat.setW(msg.orientation.w);
    //tf2::fromMsg(msg.orientation, quat);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    
    if(init_yaw_flag== false)
    {
        init_yaw = yaw;
        init_yaw_flag = true;
    }
    else
    {   
      mutex_webots_state.lock();  
        r_a.yaw = restrictAngle(yaw-init_yaw);
        r_a.roll = roll;
        r_a.pitch = pitch;
      mutex_webots_state.unlock(); 

        if(PAR.Slam_feedback == false)
        { 
          mutex_robot_state.lock();          
            r_c.yaw = restrictAngle(yaw-init_yaw);
            double Ra2b[9];          
            yaw = restrictAngle(yaw);
            Euler_to_Ra2b_colum_major(roll, pitch, yaw, Ra2b);
            arma::mat Rn2r(Ra2b,3,3);
            Rn2rG = Rn2r;
          mutex_robot_state.unlock();           
        }
       
    }
  
  }
    
  //-----------------------------------------------------------------------
  void ControlQuad::Gps_callback(const geometry_msgs::msg::PointStamped & msg)
  { 
    

    static double x_0;
    static double y_0;
    static double z_0;
    static bool init_flag = false;

    if(!init_flag)
    {
      x_0 = msg.point.x;
      y_0 = -msg.point.y;
      z_0 = msg.point.z;
      init_flag = true;
      return;
    }
    
    double x = msg.point.x;;
    double y = -msg.point.y;
    double z = msg.point.z;

    mutex_webots_state.lock();
      r_a.x = x - x_0;
      r_a.y = y - y_0;
      r_a.z = z - z_0;
      actual_z_flag = true;
    mutex_webots_state.unlock();
    
    if(PAR.Slam_feedback == false)
        {
            mutex_robot_state.lock();
                
                arma::vec gt_N = {x - x_0, y - y_0, z - z_0 };
                arma::vec gt_R = Rn2rG.t()*gt_N;
                
                r_c.x = gt_R[0];
                r_c.y = gt_R[1];
                r_c.z = gt_R[2];
            mutex_robot_state.unlock();
            //cout << "-> GPS  " << r_c.x << " " << r_c.y << "  " << r_c.z << endl;  
        }

      

  }

  //-----------------------------------------------------------------------
  void ControlQuad::Slam_callback(const interfaces::msg::Robotstate & msg)
  {
    if (msg.robot_state.size() < 13) {
      return;
    }
    const double x = msg.robot_state[7];
    const double y = msg.robot_state[8];
    const double z = msg.robot_state[9];
    const double roll = msg.robot_state[1];
    const double pitch = msg.robot_state[2];
    const double yaw = msg.robot_state[3];
    const double r_u = msg.radius_uncertainty;

    if (!slam_yaw_init_flag_) {
      slam_init_yaw_ = yaw;
      slam_yaw_init_flag_ = true;
      slam_init_z_ = r_a.z;
      return;
    }

    mutex_slam_state.lock();
    r_s.x = x;
    r_s.y = y;
    r_s.z = z;
    r_s.yaw = restrictAngle(yaw - slam_init_yaw_);
    r_s.roll = roll;
    r_s.pitch = pitch;
    r_s.r_u = r_u;
    mutex_slam_state.unlock();

    if (PAR.Slam_feedback == true && actual_z_flag == true) {
      const auto c = feedback::legacy_internal_to_control(
        x, y, z, roll, pitch, yaw, slam_init_yaw_, slam_init_z_);
      mutex_robot_state.lock();
      r_c.x = c.x;
      r_c.y = c.y;
      r_c.z = c.z;
      r_c.roll = c.roll;
      r_c.pitch = c.pitch;
      r_c.yaw = c.yaw;
      r_c.r_u = r_u;  // available only on legacy Robotstate
      mutex_robot_state.unlock();
    }
  }

  void ControlQuad::Slam_radius_only_callback(
    const interfaces::msg::Robotstate & msg)
  {
    // A5b/A6: odometry mode — consume radius_uncertainty only.
    // Pose / n2r / vels must not overwrite Odometry-derived feedback.
    // Drop while disarmed so queued pre-reset callbacks cannot restore r_u.
    if (PAR.Slam_feedback != true) {
      return;
    }
    if (!radius_feedback_armed_) {
      return;
    }
    const double r_u = msg.radius_uncertainty;
    mutex_slam_state.lock();
    r_s.r_u = r_u;
    mutex_slam_state.unlock();
    mutex_robot_state.lock();
    r_c.r_u = r_u;
    mutex_robot_state.unlock();
  }

  void ControlQuad::Odometry_callback(const nav_msgs::msg::Odometry & msg)
  {
    if (!feedback::expected_odometry_frames(
          msg.header.frame_id, msg.child_frame_id,
          PAR.Odometry_expect_frame_id, PAR.Odometry_expect_child_frame_id))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Rejecting Odometry with frames '%s'/'%s' (expected '%s'/'%s')",
        msg.header.frame_id.c_str(), msg.child_frame_id.c_str(),
        PAR.Odometry_expect_frame_id.c_str(),
        PAR.Odometry_expect_child_frame_id.c_str());
      return;
    }

    // A6: ignore Odometry stamped before the last coordinated radius invalidate.
    // Prevents queued pre-reset poses from latching yaw / re-arming radius.
    if (radius_reset_stamp_valid_) {
      const rclcpp::Time stamp(msg.header.stamp);
      if (stamp < radius_reset_stamp_) {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Ignoring pre-reset Odometry (stamp older than radius_reset_stamp_)");
        return;
      }
    }

    const auto & p = msg.pose.pose.position;
    tf2::Quaternion quat(
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w);
    tf2::Matrix3x3 mat(quat);
    double roll_pub = 0.0, pitch_pub = 0.0, yaw_pub = 0.0;
    mat.getRPY(roll_pub, pitch_pub, yaw_pub);

    if (!feedback::finite6(
          p.x, p.y, p.z, roll_pub, pitch_pub, yaw_pub))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Rejecting non-finite Odometry pose");
      return;
    }

    if (!slam_yaw_init_flag_) {
      slam_init_yaw_ = yaw_pub;  // public yaw latch
      slam_yaw_init_flag_ = true;
      slam_init_z_ = r_a.z;
      return;
    }

    const auto s = feedback::odometry_pose_to_internal_slam(
      p.x, p.y, p.z, roll_pub, pitch_pub, yaw_pub);
    mutex_slam_state.lock();
    r_s.x = s.x;
    r_s.y = s.y;
    r_s.z = s.z;
    r_s.roll = s.roll;
    r_s.pitch = s.pitch;
    // Match legacy r_s.yaw = wrap(yaw_int - init_yaw_int)
    r_s.yaw = feedback::wrap_pi(s.yaw - (-slam_init_yaw_));
    // A5b: do not touch r_s.r_u — radius arrives separately from Robotstate.
    mutex_slam_state.unlock();

    if (PAR.Slam_feedback == true && actual_z_flag == true) {
      const auto c = feedback::odometry_pose_to_control(
        p.x, p.y, p.z, roll_pub, pitch_pub, yaw_pub,
        slam_init_yaw_, slam_init_z_);
      mutex_robot_state.lock();
      r_c.x = c.x;
      r_c.y = c.y;
      r_c.z = c.z;
      r_c.roll = c.roll;
      r_c.pitch = c.pitch;
      r_c.yaw = c.yaw;
      // A5b: leave r_c.r_u unchanged (Slam_radius_only_callback owns it).
      mutex_robot_state.unlock();
      // A6: re-arm radius acceptance after coordinated reset / cold start.
      // Only reached for Odometry with stamp >= radius_reset_stamp_ (if set).
      if (!radius_feedback_armed_) {
        radius_feedback_armed_ = true;
        RCLCPP_INFO(
          this->get_logger(),
          "Radius feedback re-armed after accepted post-reset Odometry");
      }
    }
  }
  //----------------------------------------------------
  void ControlQuad::Frame_callback(const interfaces::msg::Frame & msg) 
{
  
  
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg.img,sensor_msgs::image_encodings::MONO8 );
  
   
  mutex_frame.lock(); 
    frame = cv_ptr->image;
  mutex_frame.unlock();    

  


}




}    