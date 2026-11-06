#ifndef COMPOSITION__PLOT_COMPONENT2_HPP_
#define COMPOSITION__PLOT_COMPONENT2_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/simple_serv.hpp"
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/lslam.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>
#include <armadillo>
#include <opencv2/core.hpp>
#include "logger.hpp"
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <string>

using namespace std;

struct RobotState {
    arma::vec3 position;
    arma::vec3 orientation;  // roll, pitch, yaw (radians)
    arma::vec3 velocity;

    RobotState()
        : position(arma::vec3({0, 0, 0})),
          orientation(arma::vec3({0, 0, 0})),
          velocity(arma::vec3({0, 0, 0})) {}
};

namespace plot2
{

/**
 * Data ownership (A9 workflow / Jazzy):
 *   nav_msgs/Odometry     → estimated pose / twist (public odom parent)
 *   nav_msgs/Path         → estimated trajectory (same snapshots as odometry)
 *   sensor_msgs/PointCloud2 → current local landmarks (feats+anchors merged)
 *   interfaces/Lslam      → image feature overlays + radius_uncertainty only
 *   interfaces/Frame      → camera image
 *   geometry_msgs/PointStamped GT_topic → same public-odom GT as /ground_truth/path
 *     parent (webotbridge fixed yaw0 map). Viewer latches an origin when SLAM
 *     starts; no extra D / attitude transform.
 *
 * Common display frame: odometry parent (odom), +up public convention.
 * Do not also accumulate trajectory from Odometry when Path is consumed.
 */

class PLOTscene2 : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit PLOTscene2(const rclcpp::NodeOptions & options);

  ~PLOTscene2();

private:
  thread plot_loop_;
  void PLOT_LOOP();

  // Standard (A2/A4)
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_Odom_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_Path_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_Landmarks_;
  // Custom (narrow): image + feature overlays + radius; GT
  rclcpp::Subscription<interfaces::msg::Frame>::SharedPtr sub_Frame_;
  rclcpp::Subscription<interfaces::msg::Lslam>::SharedPtr sub_lslam_data_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_Gps_;

  rclcpp::Service<interfaces::srv::SimpleServ>::SharedPtr srv_plot_;
  void Handle_plot_service(
    const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,
    std::shared_ptr<interfaces::srv::SimpleServ::Response> response);

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
  double radius_uncertainty_{0.0};

  mutable std::mutex frame_mutex_;
  mutable cv::Mat latest_frame_;
  mutable bool new_frame_available_ = false;

  mutable GLuint latest_texture_id_ = 0;
  mutable int latest_img_width_ = 0;
  mutable int latest_img_height_ = 0;

  bool capture_frames_ = false;

  // Keyboard/service-driven view state (shared across plot panes)
  std::mutex view_mutex_;
  float view_scale_{10.0f};
  ImVec2 pan_xy_{0, 0};
  ImVec2 pan_xz_{0, 0};
  ImVec2 pan_zy_{0, 0};
  int active_view_{1};  // 1=xy, 2=xz, 3=yz
  bool clear_request_{false};
  double distance_traveled_xy;

  void Frame_callback(const interfaces::msg::Frame & msg) const;
  void lslam_data_callback(const interfaces::msg::Lslam & msg);
  void GT_callback(const geometry_msgs::msg::PointStamped & msg);
  void Odometry_callback(const nav_msgs::msg::Odometry & msg);
  void Path_callback(const nav_msgs::msg::Path & msg);
  void Landmarks_callback(const sensor_msgs::msg::PointCloud2 & msg);

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
  void apply_clear_if_requested();

  void setParameters();
  string label_logs;
};

}  // namespace plot2

#endif  // COMPOSITION__PLOT_COMPONENT2_HPP_
