cmake_minimum_required(VERSION 3.8)
project(keyboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

set(SOURCE_FILES src/keyboard_node.cpp src/vpKeyboard.cpp)

add_executable(keyboard_node ${SOURCE_FILES})

ament_target_dependencies(keyboard_node
  rclcpp interfaces geometry_msgs std_srvs)


target_include_directories(keyboard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(keyboard_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS keyboard_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()
