cmake_minimum_required(VERSION 3.0.2)
project(ctf_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Vision library: ArUco/AprilTag flag detection.
add_library(ctf_vision
  src/vision/aruco_flag_detector.cpp
)
target_link_libraries(ctf_vision ${OpenCV_LIBS})

add_executable(flag_detector_node src/nodes/flag_detector_node.cpp)
target_link_libraries(flag_detector_node ctf_vision ${catkin_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS
  ctf_vision
  flag_detector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY launch params rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Real robot SLAM / CTF Python nodes.
catkin_install_python(PROGRAMS
  scripts/clock_drift_check.py
  scripts/odom_tf_broadcaster.py
  scripts/map_merge_debug.py
  scripts/tf_map_merge.py
  scripts/launch_config_echo.py
  scripts/laser_obstacle_filter.py
  scripts/robot_coordinator.py
  scripts/slam_frontier_explorer_ctf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
