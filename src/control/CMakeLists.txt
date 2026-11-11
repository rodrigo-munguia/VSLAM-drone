cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED aruco)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)
ament_export_include_directories(include)

set(SOURCE_FILES
  src/control_component.cpp
  src/control_commands.cpp
  src/control_sensor_callbacks.cpp
  ../common/Transforms/Euler_to_Ra2b.cpp
  ../common/Vision/vision.cpp)

add_library(control_component SHARED ${SOURCE_FILES})

target_compile_definitions(control_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(control_component
  "interfaces"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "OpenCV"
  "cv_bridge"
  "tf2"
  "tf2_ros"
  "ament_index_cpp")

rclcpp_components_register_nodes(control_component "control::ControlQuad")
set(node_plugins "${node_plugins}control::ControlQuad;$<TARGET_FILE:control_component>\n")

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
  control_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_control_feedback_conversions
      test/test_control_feedback_conversions.cpp)
    if(TARGET test_control_feedback_conversions)
      target_include_directories(test_control_feedback_conversions PRIVATE src)
    endif()

    ament_add_gtest(test_go_home_ru test/test_go_home_ru.cpp)
    if(TARGET test_go_home_ru)
      target_include_directories(test_go_home_ru PRIVATE src)
    endif()

    ament_add_gtest(test_odometry_radius_feedback
      test/test_odometry_radius_feedback.cpp)
    if(TARGET test_odometry_radius_feedback)
      target_include_directories(test_odometry_radius_feedback PRIVATE src)
    endif()

    ament_add_gtest(test_g6_coord_policy test/test_g6_coord_policy.cpp)
    if(TARGET test_g6_coord_policy)
      target_include_directories(test_g6_coord_policy PRIVATE src ../common)
    endif()

    ament_add_gtest(test_g6_feedback_barrier test/test_g6_feedback_barrier.cpp)
    if(TARGET test_g6_feedback_barrier)
      target_include_directories(test_g6_feedback_barrier PRIVATE src ../common)
    endif()

    ament_add_gtest(test_g6b_plan_interrupt test/test_g6b_plan_interrupt.cpp)
    if(TARGET test_g6b_plan_interrupt)
      target_include_directories(test_g6b_plan_interrupt PRIVATE src)
    endif()
  endif()
endif()

ament_package()
