cmake_minimum_required(VERSION 3.8)
project(localslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -fpermissive -DARMA_DONT_USE_WRAPPER -DARMA_USE_BLAS -DARMA_USE_LAPACK)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

include_directories(include)

set(SOURCE_FILES
  src/localslam_component.cpp
  src/sensors_callbacks.cpp
  src/sensor_conversions.cpp
  src/odometry_conversions.cpp
  src/ekf.cpp
  src/ekf_prediction.cpp
  ../common/Transforms/Euler_to_Ra2b.cpp
  ../common/Transforms/Ra2b_TO_Quat_a2b.cpp
  src/ekf_attitude_update.cpp
  ../common/Transforms/AngleWrap.cpp
  src/ekf_speed_update.cpp
  ../common/Transforms/quat2R.cpp
  src/ekf_altitude_update.cpp
  src/ekf_rangetobase_update.cpp
  src/ekf_visual_update.cpp
  src/ekf_visual_aux_methods.cpp
  ../common/anms/anms.cpp
  ../common/Vision/vision.cpp
  src/ekf_Jacobians.cpp
  src/ekf_visual_init_feats.cpp
  src/ekf_visual_match_feats.cpp
  src/ekf_visual_delete_feats.cpp
  src/ekf_visual_update_WO_validation.cpp
  src/ekf_visual_update_With_validation.cpp
  src/parameters.cpp)

add_library(localslam_component SHARED ${SOURCE_FILES})

target_compile_definitions(localslam_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(localslam_component
  "interfaces"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "std_srvs"
  "OpenCV"
  "cv_bridge"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "mavros_msgs"
  "Threads"
  "tf2"
  "tf2_ros"
  "BLAS"
  "LAPACK")

rclcpp_components_register_nodes(localslam_component "localslam::EKFslam")

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
  localslam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Standalone launch + config (no ekf_monoslam_bringup required).
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_sensor_conversions test/test_sensor_conversions.cpp
      src/sensor_conversions.cpp
      ../common/Transforms/Euler_to_Ra2b.cpp)
    if(TARGET test_sensor_conversions)
      ament_target_dependencies(test_sensor_conversions
        sensor_msgs
        geometry_msgs
        mavros_msgs
        tf2
        std_msgs)
      target_include_directories(test_sensor_conversions PRIVATE src)
    endif()

    ament_add_gtest(test_range_acceptance_params
      test/test_range_acceptance_params.cpp)
    if(TARGET test_range_acceptance_params)
      ament_target_dependencies(test_range_acceptance_params rclcpp)
      target_include_directories(test_range_acceptance_params PRIVATE src)
    endif()

    ament_add_gtest(test_odometry_conversions test/test_odometry_conversions.cpp
      src/odometry_conversions.cpp
      ../common/Transforms/Euler_to_Ra2b.cpp)
    if(TARGET test_odometry_conversions)
      ament_target_dependencies(test_odometry_conversions tf2)
      target_include_directories(test_odometry_conversions PRIVATE src)
    endif()

    ament_add_gtest(test_map_path_conversions test/test_map_path_conversions.cpp)
    if(TARGET test_map_path_conversions)
      ament_target_dependencies(test_map_path_conversions
        sensor_msgs
        nav_msgs
        geometry_msgs
        std_msgs)
      target_include_directories(test_map_path_conversions PRIVATE src)
    endif()

    ament_add_gtest(test_cl_radius test/test_cl_radius.cpp)
    if(TARGET test_cl_radius)
      target_include_directories(test_cl_radius PRIVATE src)
    endif()

    ament_add_gtest(test_camera_calibration test/test_camera_calibration.cpp)
    if(TARGET test_camera_calibration)
      ament_target_dependencies(test_camera_calibration sensor_msgs std_msgs)
      target_include_directories(test_camera_calibration PRIVATE src)
    endif()

    ament_add_gtest(test_pressure_altitude test/test_pressure_altitude.cpp)
    if(TARGET test_pressure_altitude)
      ament_target_dependencies(test_pressure_altitude sensor_msgs std_msgs)
      target_include_directories(test_pressure_altitude PRIVATE src)
    endif()

    ament_add_gtest(test_ls_pos_update_policy test/test_ls_pos_update_policy.cpp)
    if(TARGET test_ls_pos_update_policy)
      target_include_directories(test_ls_pos_update_policy PRIVATE src ../common)
    endif()

    ament_add_gtest(test_ls_pos_apply_policy test/test_ls_pos_apply_policy.cpp)
    if(TARGET test_ls_pos_apply_policy)
      target_include_directories(test_ls_pos_apply_policy PRIVATE src)
    endif()

    # E5e-A3a: Euler P = diagmat(p_ini) regression (links estimator library).
    ament_add_gtest(test_euler_cov_init test/test_euler_cov_init.cpp)
    if(TARGET test_euler_cov_init)
      target_link_libraries(test_euler_cov_init localslam_component)
      target_include_directories(test_euler_cov_init PRIVATE src ../common)
    endif()

    # E5e-G2: horizontal-plane range init geometry + Jacobian finite differences.
    ament_add_gtest(test_range_horizontal_plane_init
      test/test_range_horizontal_plane_init.cpp)
    if(TARGET test_range_horizontal_plane_init)
      target_link_libraries(test_range_horizontal_plane_init localslam_component)
      target_include_directories(test_range_horizontal_plane_init PRIVATE src ../common)
    endif()

    # LSLAM-DYNAMIC-CONSISTENCY-A2: Jac_e_uv_XYZ FD + attitude coupling.
    ament_add_gtest(test_visual_jacobian_attitude_coupling_a2
      test/test_visual_jacobian_attitude_coupling_a2.cpp)
    if(TARGET test_visual_jacobian_attitude_coupling_a2)
      target_link_libraries(test_visual_jacobian_attitude_coupling_a2 localslam_component)
      target_include_directories(test_visual_jacobian_attitude_coupling_a2 PRIVATE src ../common)
    endif()

    # LSLAM-DYNAMIC-CONSISTENCY-A2.2: FeatPrediction hi(1) = PredictedPoint.y.
    ament_add_gtest(test_ransac_feat_prediction_a2_2
      test/test_ransac_feat_prediction_a2_2.cpp)
    if(TARGET test_ransac_feat_prediction_a2_2)
      target_link_libraries(test_ransac_feat_prediction_a2_2 localslam_component)
      target_include_directories(test_ransac_feat_prediction_a2_2 PRIVATE src ../common)
    endif()
  endif()
endif()

ament_package()
