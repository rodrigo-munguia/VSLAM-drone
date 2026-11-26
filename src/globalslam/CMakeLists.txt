cmake_minimum_required(VERSION 3.8)
project(globalslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -fpermissive
    -DARMA_DONT_USE_WRAPPER -DARMA_USE_BLAS -DARMA_USE_LAPACK)
endif()

# Opt-in: dependency-free policy unit tests only. Disabled by default.
# Does NOT build or install the runtime component / launch assets.
option(GLOBALSLAM_POLICY_TESTS_ONLY
  "Build only G2 policy unit tests; skip GTSAM runtime component (default OFF)"
  OFF)

find_package(ament_cmake REQUIRED)

if(GLOBALSLAM_POLICY_TESTS_ONLY)
  message(STATUS
    "globalslam: GLOBALSLAM_POLICY_TESTS_ONLY=ON — building policy tests only. "
    "Runtime component globalslam::Gslam is NOT built; launch/config are NOT installed.")

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    ament_add_gtest(test_g2_run_control test/test_g2_run_control.cpp)
    if(TARGET test_g2_run_control)
      target_include_directories(test_g2_run_control PRIVATE src)
    endif()
    ament_add_gtest(test_g3_admission test/test_g3_admission.cpp)
    if(TARGET test_g3_admission)
      target_include_directories(test_g3_admission PRIVATE src ../common)
    endif()
    ament_add_gtest(test_g4_viz_conversions test/test_g4_viz_conversions.cpp)
    if(TARGET test_g4_viz_conversions)
      target_include_directories(test_g4_viz_conversions PRIVATE src)
      ament_target_dependencies(test_g4_viz_conversions
        nav_msgs sensor_msgs geometry_msgs std_msgs)
    endif()
    ament_add_gtest(test_g6_coord_dispatch test/test_g6_coord_dispatch.cpp)
    if(TARGET test_g6_coord_dispatch)
      target_include_directories(test_g6_coord_dispatch PRIVATE src)
    endif()
    ament_add_gtest(test_g6b_cl_match_index test/test_g6b_cl_match_index.cpp)
    if(TARGET test_g6b_cl_match_index)
      target_include_directories(test_g6b_cl_match_index PRIVATE src)
    endif()
    ament_add_gtest(test_g6b_home_tag_geometry test/test_g6b_home_tag_geometry.cpp)
    if(TARGET test_g6b_home_tag_geometry)
      target_include_directories(test_g6b_home_tag_geometry PRIVATE src)
    endif()
    ament_add_gtest(test_g6b2_geometry_audit test/test_g6b2_geometry_audit.cpp)
    if(TARGET test_g6b2_geometry_audit)
      target_include_directories(test_g6b2_geometry_audit PRIVATE src ../common)
    endif()
    ament_add_gtest(test_g6b3_home_tag_capture test/test_g6b3_home_tag_capture.cpp)
    if(TARGET test_g6b3_home_tag_capture)
      target_include_directories(test_g6b3_home_tag_capture PRIVATE src ../common)
    endif()
    ament_add_gtest(test_g6c_opt_feedback_audit test/test_g6c_opt_feedback_audit.cpp)
    if(TARGET test_g6c_opt_feedback_audit)
      target_include_directories(test_g6c_opt_feedback_audit PRIVATE src)
    endif()
    ament_add_gtest(test_g6d_source_policy test/test_g6d_source_policy.cpp)
    if(TARGET test_g6d_source_policy)
      target_include_directories(test_g6d_source_policy PRIVATE src)
    endif()
    ament_add_gtest(test_g6b_feedback_completion test/test_g6b_feedback_completion.cpp)
    if(TARGET test_g6b_feedback_completion)
      target_include_directories(test_g6b_feedback_completion PRIVATE src)
    endif()
  endif()

  ament_package()
  return()
endif()

# --- Normal build: GTSAM is required for the runtime component ---
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
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d aruco
  OPTIONAL_COMPONENTS opencv_sfm)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

find_package(GTSAM QUIET)
if(NOT GTSAM_FOUND)
  message(FATAL_ERROR [[
globalslam: GTSAM is required to build the runtime component
(globalslam::Gslam), but GTSAMConfig.cmake was not found.
  Fix: install a compatible GTSAM (this repo was validated with 4.2.0)
  to a user prefix and expose it via CMAKE_PREFIX_PATH or GTSAM_DIR, e.g.
    export GTSAM_PREFIX=/path/to/gtsam-4.2.0
    export CMAKE_PREFIX_PATH="${GTSAM_PREFIX}:${CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="${GTSAM_PREFIX}/lib:${LD_LIBRARY_PATH}"
  See docs/history/G1_globalslam_interface_contract.md (G2 close-out).
  For policy-unit-tests only (no component): -DGLOBALSLAM_POLICY_TESTS_ONLY=ON
]])
endif()

if(NOT OpenCV_opencv_sfm_FOUND)
  add_definitions("-DNOTHAVE_SFM")
endif()

include_directories(include)
if(Eigen3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIRS})
else()
  include_directories(/usr/include/eigen3)
endif()

set(GLOBALSLAM_COMPONENT_SOURCES
  src/globalslam_component.cpp
  src/parameters.cpp
  ../common/Transforms/quat2R.cpp
  ../common/Transforms/Ra2b_TO_Quat_a2b.cpp
  src/gmap.cpp
  src/gmap_init_new_map_points.cpp
  ../common/Vision/vision.cpp
  ../common/anms/anms.cpp
  src/gmap_visibility_graph_functions.cpp
  src/gmap_visual_match.cpp
  src/gmap_delete_points.cpp
  src/gmap_bundle_adjustment.cpp
  src/cloop.cpp
  src/cloop_visibility_graph_functions.cpp
  src/cloop_visual_match.cpp
  src/cloop_get_pos.cpp
  ../common/Transforms/Euler_to_Ra2b.cpp
  src/cloop_update_gmap.cpp
  src/cloop_pose_slam.cpp
  src/cloop_fuse_points.cpp
  src/cloop_update_gmap_with_new_matches.cpp
  src/cloop_bundle_adjustment.cpp)

add_library(globalslam_component SHARED ${GLOBALSLAM_COMPONENT_SOURCES})
target_compile_definitions(globalslam_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(globalslam_component
  interfaces
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  OpenCV
  cv_bridge
  sensor_msgs
  geometry_msgs
  nav_msgs
  BLAS
  LAPACK
  Eigen3
  Boost)

if(TARGET gtsam)
  target_link_libraries(globalslam_component gtsam)
elseif(TARGET GTSAM::gtsam)
  target_link_libraries(globalslam_component GTSAM::gtsam)
else()
  message(FATAL_ERROR
    "globalslam: GTSAM was found but neither target 'gtsam' nor 'GTSAM::gtsam' "
    "is available. Check GTSAM_DIR / CMAKE_PREFIX_PATH.")
endif()

rclcpp_components_register_nodes(globalslam_component "globalslam::Gslam")

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS globalslam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Package-owned config + standalone launch (only when the component is built).
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py")

message(STATUS "globalslam: GTSAM found — building and registering globalslam::Gslam")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_g2_run_control test/test_g2_run_control.cpp)
    if(TARGET test_g2_run_control)
      target_include_directories(test_g2_run_control PRIVATE src)
    endif()
    ament_add_gtest(test_g3_admission test/test_g3_admission.cpp)
    if(TARGET test_g3_admission)
      target_include_directories(test_g3_admission PRIVATE src ../common)
    endif()
    ament_add_gtest(test_g4_viz_conversions test/test_g4_viz_conversions.cpp)
    if(TARGET test_g4_viz_conversions)
      target_include_directories(test_g4_viz_conversions PRIVATE src)
      ament_target_dependencies(test_g4_viz_conversions
        nav_msgs sensor_msgs geometry_msgs std_msgs)
    endif()
    ament_add_gtest(test_g6_coord_dispatch test/test_g6_coord_dispatch.cpp)
    if(TARGET test_g6_coord_dispatch)
      target_include_directories(test_g6_coord_dispatch PRIVATE src)
    endif()
    ament_add_gtest(test_g6b_cl_match_index test/test_g6b_cl_match_index.cpp)
    if(TARGET test_g6b_cl_match_index)
      target_include_directories(test_g6b_cl_match_index PRIVATE src)
    endif()
    ament_add_gtest(test_g6b_home_tag_geometry test/test_g6b_home_tag_geometry.cpp)
    if(TARGET test_g6b_home_tag_geometry)
      target_include_directories(test_g6b_home_tag_geometry PRIVATE src)
    endif()
    ament_add_gtest(test_g6b2_geometry_audit test/test_g6b2_geometry_audit.cpp)
    if(TARGET test_g6b2_geometry_audit)
      target_include_directories(test_g6b2_geometry_audit PRIVATE src ../common)
    endif()
    ament_add_gtest(test_g6b3_home_tag_capture test/test_g6b3_home_tag_capture.cpp)
    if(TARGET test_g6b3_home_tag_capture)
      target_include_directories(test_g6b3_home_tag_capture PRIVATE src ../common)
    endif()
    ament_add_gtest(test_g6c_opt_feedback_audit test/test_g6c_opt_feedback_audit.cpp)
    if(TARGET test_g6c_opt_feedback_audit)
      target_include_directories(test_g6c_opt_feedback_audit PRIVATE src)
    endif()
    ament_add_gtest(test_g6d_source_policy test/test_g6d_source_policy.cpp)
    if(TARGET test_g6d_source_policy)
      target_include_directories(test_g6d_source_policy PRIVATE src)
    endif()
    ament_add_gtest(test_gslam_audit_ba_projection
      test/test_gslam_audit_ba_projection.cpp)
    if(TARGET test_gslam_audit_ba_projection)
      target_include_directories(test_gslam_audit_ba_projection PRIVATE src)
      if(TARGET gtsam)
        target_link_libraries(test_gslam_audit_ba_projection gtsam)
      elseif(TARGET GTSAM::gtsam)
        target_link_libraries(test_gslam_audit_ba_projection GTSAM::gtsam)
      endif()
    endif()
    ament_add_gtest(test_g6b2_pose_slam_factor_audit
      test/test_g6b2_pose_slam_factor_audit.cpp)
    if(TARGET test_g6b2_pose_slam_factor_audit)
      target_include_directories(test_g6b2_pose_slam_factor_audit PRIVATE src)
      if(TARGET gtsam)
        target_link_libraries(test_g6b2_pose_slam_factor_audit gtsam)
      elseif(TARGET GTSAM::gtsam)
        target_link_libraries(test_g6b2_pose_slam_factor_audit GTSAM::gtsam)
      endif()
    endif()
    ament_add_gtest(test_g6b_feedback_completion test/test_g6b_feedback_completion.cpp)
    if(TARGET test_g6b_feedback_completion)
      target_include_directories(test_g6b_feedback_completion PRIVATE src)
    endif()
    ament_add_gtest(test_gmap_birth_reproj_gate
      test/test_gmap_birth_reproj_gate.cpp
      ../common/Vision/vision.cpp)
    if(TARGET test_gmap_birth_reproj_gate)
      target_include_directories(test_gmap_birth_reproj_gate PRIVATE src ../common)
      ament_target_dependencies(test_gmap_birth_reproj_gate OpenCV)
      target_link_libraries(test_gmap_birth_reproj_gate ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
    endif()
    # GSLAM-MAP-ARCH-M1/M2: stable LandmarkId + id_to_index lifecycle.
    ament_add_gtest(test_gmap_landmark_identity_m1_m2
      test/test_gmap_landmark_identity_m1_m2.cpp)
    if(TARGET test_gmap_landmark_identity_m1_m2)
      target_include_directories(test_gmap_landmark_identity_m1_m2 PRIVATE src ../common)
      ament_target_dependencies(test_gmap_landmark_identity_m1_m2 OpenCV)
      target_link_libraries(test_gmap_landmark_identity_m1_m2
        ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
    endif()
    ament_add_gtest(test_gmap_relative_kf_init
      test/test_gmap_relative_kf_init.cpp)
    if(TARGET test_gmap_relative_kf_init)
      target_include_directories(test_gmap_relative_kf_init PRIVATE src ../common)
      ament_target_dependencies(test_gmap_relative_kf_init OpenCV)
      target_link_libraries(test_gmap_relative_kf_init ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
    endif()
    # GSLAM-LC-CONSISTENCY-A3: alignment δ_L + rebase + admit watermark.
    ament_add_gtest(test_gmap_visual_sync
      test/test_gmap_visual_sync.cpp)
    if(TARGET test_gmap_visual_sync)
      target_include_directories(test_gmap_visual_sync PRIVATE src ../common)
      ament_target_dependencies(test_gmap_visual_sync OpenCV)
      target_link_libraries(test_gmap_visual_sync ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
    endif()
    # GSLAM-VISUAL-LOOP-A2b: production cloop_visual_pnp::solve_and_accept.
    ament_add_gtest(test_visual_loop_a2_pnp_geometry
      test/test_visual_loop_a2_pnp_geometry.cpp
      ../common/Vision/vision.cpp)
    if(TARGET test_visual_loop_a2_pnp_geometry)
      target_include_directories(test_visual_loop_a2_pnp_geometry PRIVATE src ../common)
      ament_target_dependencies(test_visual_loop_a2_pnp_geometry OpenCV)
      target_link_libraries(test_visual_loop_a2_pnp_geometry
        ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} ${OpenCV_LIBS})
    endif()
    # GSLAM-VISUAL-LOOP-A2d: retrieval / ranking / PnP fallback helpers.
    ament_add_gtest(test_visual_loop_a2d_retrieve
      test/test_visual_loop_a2d_retrieve.cpp
      ../common/Vision/vision.cpp)
    if(TARGET test_visual_loop_a2d_retrieve)
      target_include_directories(test_visual_loop_a2d_retrieve PRIVATE src ../common)
      ament_target_dependencies(test_visual_loop_a2d_retrieve OpenCV)
      target_link_libraries(test_visual_loop_a2d_retrieve
        ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} ${OpenCV_LIBS})
    endif()
  endif()
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()
