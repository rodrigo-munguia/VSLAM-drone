#ifndef COMPOSITION__PLOT_COMPONENT2_HPP_
#define COMPOSITION__PLOT_COMPONENT2_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/simple_serv.hpp"
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/lslam.hpp"
#include "interfaces/msg/gmap.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
//#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <armadillo>
#include <opencv2/core.hpp>
#include "logger.hpp"
#include <atomic>


using namespace std;

struct RobotState {
    arma::vec3 position;     // X, Y, Z position
    arma::vec3 orientation;  // Roll, Pitch, Yaw (or could be quaternion if you prefer)
    arma::vec3 velocity;     // Optional: linear velocity

    RobotState()
        : position(arma::vec3({0, 0, 0})),
          orientation(arma::vec3({0, 0, 0})),
          velocity(arma::vec3({0, 0, 0})) {}
};  


namespace plot2
{



class PLOTscene2 : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit PLOTscene2(const rclcpp::NodeOptions & options);

  ~PLOTscene2();

protected:
  

private:
  thread plot_loop_;
  void PLOT_LOOP();

  // declare subscribers
  rclcpp::Subscription<interfaces::msg::Frame>::SharedPtr sub_Frame_;
  rclcpp::Subscription<interfaces::msg::Lslam>::SharedPtr sub_lslam_data_;
  //rclcpp::Subscription<interfaces::msg::Gmap>::SharedPtr sub_gmap_data_;  
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_Gps_; // webots ground truth
   
    //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
  std::vector<arma::vec3> positions_;
  std::vector<arma::vec3> gt_;
  std::vector<arma::vec3> mapFeat_;
  std::vector<arma::vec3> mapAnchor_;

  std::vector<arma::vec2> img_matched_feats_;
  std::vector<arma::vec2> img_unmatched_feats_;
  std::vector<arma::vec2> img_matched_anchors_;
  std::vector<arma::vec2> img_unmatched_anchors_;    


  std::mutex data_mutex_;
  std::thread ui_thread_;
  std::atomic<bool> running_;
  std::atomic<bool> lslamstarted_;

  RobotState RobotState_;
  RobotState RobotStateGT_;

  mutable std::mutex frame_mutex_;
  mutable cv::Mat latest_frame_;
  mutable bool new_frame_available_ = false;

  mutable GLuint latest_texture_id_ = 0;
  mutable int latest_img_width_ = 0;
  mutable int latest_img_height_ = 0;

  bool capture_frames_ = false;

  double distance_traveled_xy;
    
  
   // declare callbacks
  void Frame_callback(const interfaces::msg::Frame & msg) const;
  void lslam_data_callback(const interfaces::msg::Lslam & msg);
  // void gmap_data_callback(const interfaces::msg::Gmap & msg);
  void GT_callback(const geometry_msgs::msg::PointStamped & msg);  // for webots ground truth

  void draw_plot(const std::vector<arma::vec3>& positions_, const std::vector<arma::vec3>& gt_, const std::vector<arma::vec3>& feats_, const std::vector<arma::vec3>& anchors_);
  void draw_plot_xz(const std::vector<arma::vec3>& positions_, const std::vector<arma::vec3>& gt_, const std::vector<arma::vec3>& feats_, const std::vector<arma::vec3>& anchors_);
  void draw_plot_zy(const std::vector<arma::vec3>& positions_, const std::vector<arma::vec3>& gt_, const std::vector<arma::vec3>& feats_, const std::vector<arma::vec3>& anchors_);
  void draw_quad(const RobotState &robotState_, const RobotState &robotStateGT_);
  void draw_frame_window(const std::vector<arma::vec2>& matchedFeats_,const std::vector<arma::vec2>& unmatchedFeats_,const std::vector<arma::vec2>& matchedAnchors_,const std::vector<arma::vec2>& unmatchedAnchors_);
  
  void get_positions(std::vector<arma::vec3>& out);
  void get_gt(std::vector<arma::vec3>& out);
  void get_localMap(std::vector<arma::vec3>& out1, std::vector<arma::vec3>& out2);
  bool get_robot_state(RobotState &out);
  void get_robot_stateGT(RobotState &out);
  void get_FramePoints(std::vector<arma::vec2>& mf, std::vector<arma::vec2>& uf,std::vector<arma::vec2>& ma, std::vector<arma::vec2>& ua);
  
  void save_screen_shoot();

  // parameters
  void setParameters();
  string label_logs;

};

}  // namespace composition

#endif  // COMPOSITION__LISTENER_COMPONENT_HPP_