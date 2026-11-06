cmake_minimum_required(VERSION 3.8)
project(webotbridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -fpermissive -DARMA_DONT_USE_WRAPPER -DARMA_USE_BLAS -DARMA_USE_LAPACK)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)


include_directories(include)
ament_export_include_directories(include)
#include_directories( ${OpenCV_INCLUDE_DIRS} )

# create ament index resource which references the libraries in the binary dir
#set(node_plugins "")

#add_executable(dataset_node src/dataset_node.cpp)
set(SOURCE_FILES 
src/webotbridge_component.cpp
../common/Transforms/Euler_to_Ra2b.cpp)

add_library(webotbridge_component SHARED ${SOURCE_FILES})


#target_include_directories(dataset_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_features(dataset_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#target_link_libraries( dataset_component ${OpenCV_LIBS} )

#message(WARNING "libraries: ${OpenCV_LIBS}")


target_compile_definitions(webotbridge_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

#install(TARGETS dataset_node
#  DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies(webotbridge_component
  "interfaces"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "mavros_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "BLAS"
  "LAPACK")


rclcpp_components_register_nodes(webotbridge_component "webotbridge::WEBOTb")
set(node_plugins "${node_plugins}webotbridge::WEBOTb;$<TARGET_FILE:webotbridge_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


install(TARGETS
  webotbridge_component  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
  endif()
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    find_package(builtin_interfaces REQUIRED)
    ament_add_gtest(test_gt_frame_conversions test/test_gt_frame_conversions.cpp)
    if(TARGET test_gt_frame_conversions)
      target_include_directories(test_gt_frame_conversions PRIVATE src)
      ament_target_dependencies(test_gt_frame_conversions builtin_interfaces)
    endif()
    ament_add_gtest(test_gt_path_epoch test/test_gt_path_epoch.cpp)
    if(TARGET test_gt_path_epoch)
      target_include_directories(test_gt_path_epoch PRIVATE src)
      ament_target_dependencies(test_gt_path_epoch nav_msgs geometry_msgs builtin_interfaces)
    endif()
  endif()
endif()



ament_package()
