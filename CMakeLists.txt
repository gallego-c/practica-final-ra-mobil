cmake_minimum_required(VERSION 3.0.2)
project(ctf_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  cv_bridge
  geometry_msgs
  move_base_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    costmap_2d
    cv_bridge
    geometry_msgs
    move_base_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ── Librería de visión (detección ArUco/AprilTag) ─────────────────────────────
add_library(ctf_vision
  src/vision/aruco_flag_detector.cpp
)
target_link_libraries(ctf_vision ${OpenCV_LIBS})

# ── Librería de juego (exploración + agente) ──────────────────────────────────
add_library(ctf_game
  src/game/exploration.cpp
  src/game/robot_agent.cpp
)
target_link_libraries(ctf_game ${catkin_LIBRARIES})

# ── Nodos ejecutables ─────────────────────────────────────────────────────────
add_executable(ctf_demo_node src/nodes/ctf_demo_node.cpp)
target_link_libraries(ctf_demo_node ${catkin_LIBRARIES})

add_executable(ctf_coordinator_node src/nodes/ctf_coordinator_node.cpp)
target_link_libraries(ctf_coordinator_node ctf_game ${catkin_LIBRARIES})

add_executable(flag_detector_node src/nodes/flag_detector_node.cpp)
target_link_libraries(flag_detector_node ctf_vision ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(robot_obstacle_publisher_node src/nodes/robot_obstacle_publisher_node.cpp)
target_link_libraries(robot_obstacle_publisher_node ${catkin_LIBRARIES})

# ── Instalación ───────────────────────────────────────────────────────────────
install(TARGETS
  ctf_vision
  ctf_game
  ctf_demo_node
  ctf_coordinator_node
  flag_detector_node
  robot_obstacle_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY launch params rviz worlds models maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# SLAM / CTF Python nodes
catkin_install_python(PROGRAMS
  scripts/clock_drift_check.py
  scripts/odom_tf_broadcaster.py
  scripts/map_merge_debug.py
  scripts/tf_map_merge.py
  scripts/launch_config_echo.py
  scripts/laser_obstacle_filter.py
  scripts/robot_coordinator.py
  scripts/robot_obstacle_publisher.py
  scripts/slam_frontier_explorer.py
  scripts/slam_frontier_explorer_ctf.py
  scripts/slam_frontier_explorer_single.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
