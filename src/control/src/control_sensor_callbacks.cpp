#include "control_component.hpp"
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
    static double init_yaw;   
    static bool init_yaw_flag = false;
    static double init_z;

    double x = msg.robot_state[7];
    double y = msg.robot_state[8];
    double z = msg.robot_state[9];
    double roll = msg.robot_state[1];
    double pitch = msg.robot_state[2];
    double yaw = msg.robot_state[3];

    double r_u = msg.radius_uncertainty;
    
    if(init_yaw_flag== false)
    {
        init_yaw = yaw;
        init_yaw_flag = true;
        init_z = r_a.z;
    }
    else
    {  
      mutex_slam_state.lock();
        r_s.x = x;
        r_s.y = y;
        r_s.z = z;
        r_s.yaw = restrictAngle(yaw-init_yaw);
        r_s.roll = roll;
        r_s.pitch = pitch;
        r_s.r_u = r_u;
      mutex_slam_state.unlock();  


      if(PAR.Slam_feedback == true && actual_z_flag == true)
      {
          mutex_robot_state.lock();
                  r_c.x = x;
                  r_c.y = y;
                  r_c.z = -z + init_z;
                  r_c.roll = roll;
                  r_c.pitch = -pitch;
                  r_c.yaw = -restrictAngle(yaw-init_yaw);
                  r_c.r_u = r_u;
          mutex_robot_state.unlock();
      }

      //cout << "-> SLAM " << x << " " << y << "  " << z << endl;    
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