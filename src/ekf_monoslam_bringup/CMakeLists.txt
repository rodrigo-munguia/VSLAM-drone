cmake_minimum_required(VERSION 3.8)
project(ekf_monoslam_bringup)

find_package(ament_cmake REQUIRED)

# Experiment resources (resolved via ament index; cwd-independent)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../control_plan.txt
  ${CMAKE_CURRENT_SOURCE_DIR}/../../control_plan_g6b.txt
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Optional RViz configs (rviz2 is not a hard package dependency).
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/config/a4_localslam.rviz
  ${CMAKE_CURRENT_SOURCE_DIR}/config/quad_experiment.rviz
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
