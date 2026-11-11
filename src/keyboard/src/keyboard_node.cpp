#include <cstdio>
#include <iostream>
#include <string>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

#include "vpKeyboard.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/simple_serv.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

static std::string viewer_mode_from_env()
{
  const char * v = std::getenv("EKF_VIEWER");
  if (v == nullptr || v[0] == '\0') {
    return "plot2";
  }
  const std::string s(v);
  if (s == "plot2" || s == "rviz2" || s == "none") {
    return s;
  }
  return "plot2";
}

static void printcommands(const std::string & viewer)
{
  std::cout
    << "\n| Operator commands (integrated experiment):\n"
    << "|   viewer mode: " << viewer << "\n"
    << "|   q  quit this operator terminal\n"
    << "|   s  Start/Stop Local SLAM (/EKFslam/enable; visual on start)\n"
    << "|   w  Coordinated reset (/EKFslam/reset + control)\n"
    << "|   a  Start/Stop control plan\n"
    << "|   h  Recover coordination hold (resume plan; not teleop)\n"
    << "|   t  Takeoff / land (Webots webot_service)\n"
    << "|   0  Request Local SLAM statistics log\n";
  if (viewer == "plot2") {
    std::cout
      << "| Plot2 (plot_service):\n"
      << "|   -/+ zoom out/in   1/2/3 select XY / XZ / YZ view\n"
      << "|   8/5/4/6 pan up/down/left/right (active view)\n"
      << "|   7 clear plot   9 screenshot (also P in Plot2 window)\n";
  } else if (viewer == "rviz2") {
    std::cout
      << "| RViz2: use the RViz window for navigation (orbit/pan/zoom).\n"
      << "| Plot2 keys (-/+ 1-9) are unavailable in this mode.\n"
      << "| GT_topic / image overlays are not shown in the shipped RViz layout.\n";
  } else {
    std::cout
      << "| No viewer: Plot2 keys (-/+ 1-9) are unavailable.\n";
  }
  std::cout
    << "| Manual flight (cmd_vel):\n"
    << "|         r forward                 i up\n"
    << "|   d left       g right    j turnL   l turnR\n"
    << "|         f back                    k down\n"
    << "|   space stop\n"
    << "| Note: Global SLAM is not in this launch; globalslam_run_service\n"
    << "|       calls on reset are best-effort and may be unavailable.\n"
    << std::endl;
}

int main(int argc, char ** argv)
{
  vpKeyboard keyboard;
  int k = 0;

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("keyboard_dataset");

  const std::string viewer = viewer_mode_from_env();
  const bool plot2_keys = (viewer == "plot2");

  auto client_ekf = node->create_client<interfaces::srv::SimpleServ>("ekf_run_service");
  auto client_globalslam =
    node->create_client<interfaces::srv::SimpleServ>("globalslam_run_service");
  auto client_control =
    node->create_client<interfaces::srv::SimpleServ>("control_run_service");
  auto client_webot =
    node->create_client<interfaces::srv::SimpleServ>("webot_service");
  auto client_plot =
    node->create_client<interfaces::srv::SimpleServ>("plot_service");
  auto client_enable =
    node->create_client<std_srvs::srv::SetBool>("/EKFslam/enable");
  auto client_reset =
    node->create_client<std_srvs::srv::Trigger>("/EKFslam/reset");

  auto pub_cmd_vel =
    node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  bool takeoff = false;
  bool slam_running = false;
  printcommands(viewer);

  auto send_simple = [&](
    const rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr & client,
    int64_t cmd)
  {
    if (!client->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<interfaces::srv::SimpleServ::Request>();
    req->cmd = cmd;
    (void)client->async_send_request(req);
  };

  while (rclcpp::ok() && k != 'q') {
    k = 0;
    if (keyboard.kbhit()) {
      k = keyboard.getchar();

      if (k == 's') {
        if (!slam_running) {
          slam_running = true;
          // Single enable path: standard SetBool (not also legacy 'r').
          if (client_enable->service_is_ready()) {
            auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            req->data = true;
            (void)client_enable->async_send_request(req);
          }
          // Visual update is a separate SimpleServ command ('v').
          send_simple(client_ekf, 'v');
          std::cout << "operator-> SLAM enable requested (/EKFslam/enable + visual)\n";
        } else {
          slam_running = false;
          if (client_enable->service_is_ready()) {
            auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            req->data = false;
            (void)client_enable->async_send_request(req);
          }
          std::cout << "operator-> SLAM disable requested (/EKFslam/enable)\n";
        }
      }

      if (k == 'a') {
        send_simple(client_control, 'r');
        std::cout << "operator-> control plan toggle requested\n";
      }

      if (k == 'h') {
        // Explicit hold recovery — must not reuse teleop 'r' (forward).
        send_simple(client_control, 'h');
        std::cout << "operator-> coordination-hold recovery requested\n";
      }

      if (k == 'w') {
        // Single reset path: standard Trigger (not also legacy 'i' on EKF).
        if (client_reset->service_is_ready()) {
          auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
          (void)client_reset->async_send_request(req);
        }
        send_simple(client_control, 'i');
        send_simple(client_globalslam, 'i');  // best-effort; may be absent
        if (plot2_keys) {
          send_simple(client_plot, 'c');
        }
        slam_running = false;
        std::cout << "operator-> coordinated reset requested\n";
      }

      if (k == '0') {
        send_simple(client_ekf, '0');
        send_simple(client_globalslam, '0');
      }

      if (plot2_keys &&
        (k == 45 || k == 43 || k == '1' || k == '2' || k == '3' ||
          k == '8' || k == '5' || k == '4' || k == '6' || k == '7' || k == '9'))
      {
        send_simple(client_plot, k);
      } else if (!plot2_keys &&
        (k == 45 || k == 43 || k == '1' || k == '2' || k == '3' ||
          k == '8' || k == '5' || k == '4' || k == '6' || k == '7' || k == '9'))
      {
        std::cout << "operator-> Plot2 key ignored (viewer=" << viewer << ")\n";
      }

      if (k == ' ') {
        geometry_msgs::msg::Twist twist;
        pub_cmd_vel->publish(twist);
        std::cout << "stop\n";
      }
      if (k == 'i') {
        geometry_msgs::msg::Twist twist;
        twist.linear.z = 0.25;
        pub_cmd_vel->publish(twist);
      }
      if (k == 'k') {
        geometry_msgs::msg::Twist twist;
        twist.linear.z = -0.25;
        pub_cmd_vel->publish(twist);
      }
      if (k == 'l') {
        geometry_msgs::msg::Twist twist;
        twist.angular.z = -0.5;
        pub_cmd_vel->publish(twist);
      }
      if (k == 'j') {
        geometry_msgs::msg::Twist twist;
        twist.angular.z = 0.5;
        pub_cmd_vel->publish(twist);
      }
      if (k == 'r') {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.5;
        pub_cmd_vel->publish(twist);
      }
      if (k == 'f') {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = -0.5;
        pub_cmd_vel->publish(twist);
      }
      if (k == 'd') {
        geometry_msgs::msg::Twist twist;
        twist.linear.y = 0.5;
        pub_cmd_vel->publish(twist);
      }
      if (k == 'g') {
        geometry_msgs::msg::Twist twist;
        twist.linear.y = -0.5;
        pub_cmd_vel->publish(twist);
      }

      if (k == 't') {
        if (!takeoff) {
          send_simple(client_webot, 't');
          takeoff = true;
          std::cout << "operator-> takeoff\n";
        } else {
          send_simple(client_webot, 'l');
          takeoff = false;
          std::cout << "operator-> land\n";
        }
      }

      std::this_thread::sleep_for(10ms);
      printcommands(viewer);
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  rclcpp::shutdown();
  return 0;
}
