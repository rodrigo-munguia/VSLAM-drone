cmake_minimum_required(VERSION 3.8)
project(plot2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -fpermissive)
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
find_package(tf2 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)
if(NOT GLFW_FOUND)
  message(FATAL_ERROR
    "glfw3 not found. Install: sudo apt-get install libglfw3-dev")
endif()

# uncomment the following section in order to fill in
# further dependencies manually.

include_directories(include)
ament_export_include_directories(include)
#include_directories( ${OpenCV_INCLUDE_DIRS} )

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# ImGui-based GUI plotting node
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/imgui)

set(IMGUI_SOURCES
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

#add_executable(dataset_node src/dataset_node.cpp)
set(SOURCE_FILES 
src/plot_component2.cpp 
../common/Transforms/quat2R.cpp
../common/Transforms/Euler_to_Ra2b.cpp
${IMGUI_SOURCES})

add_library(plot_component2 SHARED ${SOURCE_FILES})

target_include_directories(plot_component2 PRIVATE
  ${IMGUI_DIR}
  ${IMGUI_DIR}/backends
)

target_compile_definitions(plot_component2
  PRIVATE "COMPOSITION_BUILDING_DLL")

#install(TARGETS dataset_node
#  DESTINATION lib/${PROJECT_NAME})
target_link_libraries(plot_component2
  ${GLFW_LIBRARIES}
  GL
  dl
  pthread  
)

ament_target_dependencies(plot_component2
  "interfaces"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "OpenCV"
  "cv_bridge")
target_include_directories(plot_component2 PRIVATE ${GLFW_INCLUDE_DIRS})


rclcpp_components_register_nodes(plot_component2 "plot2::PLOTscene2")
set(node_plugins "${node_plugins}plot2::PLOTscene2;$<TARGET_FILE:plot_component2>\n")



# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


install(TARGETS
  plot_component2  
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
    ament_add_gtest(test_plot_frame_conversions
      test/test_plot_frame_conversions.cpp)
    if(TARGET test_plot_frame_conversions)
      target_include_directories(test_plot_frame_conversions PRIVATE src)
    endif()
  endif()
endif()



ament_package()
