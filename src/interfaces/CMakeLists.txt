cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Alt.msg"
  "msg/Att.msg"
  "msg/Bar.msg"
  "msg/Frame.msg"
  "msg/Gps.msg"
  "msg/Range.msg"
  "msg/Spd.msg"
  "msg/Lslam.msg"
  "msg/Kf.msg"
  "msg/Robotstate.msg"
  "msg/Gmap.msg"
  "msg/Ododiff.msg"
  "msg/Odovw.msg"
  "msg/Rangetobase.msg"
  "msg/EstimatorReference.msg"
  "msg/CorrectionApplied.msg"
  "msg/OdometryMeta.msg"
  "msg/CorrectionFeedbackAccepted.msg"
  "msg/GlobalLoopCommitted.msg"
  "msg/UnifiedTrajectoryStatus.msg"
  "srv/SimpleServ.srv"
  "srv/LSposUpdate.srv"
  "srv/CorrectionCoordinate.srv"
  "srv/UnifiedTrajectory.srv"
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs builtin_interfaces
    unique_identifier_msgs
)

#rosidl_target_interfaces(intercafes
#      ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

# Lint tests optional; skip when ament_lint packages are not installed.
if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()
