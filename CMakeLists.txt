cmake_minimum_required(VERSION 3.0.2)
project(ctf_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  move_base_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ctf_navigation
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    costmap_2d
    geometry_msgs
    move_base_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ctf_navigation src/planner_5d.cpp)
target_link_libraries(ctf_navigation ${catkin_LIBRARIES})

add_executable(ctf_demo_node src/ctf_demo_node.cpp)
target_link_libraries(ctf_demo_node ${catkin_LIBRARIES})

install(TARGETS ctf_navigation ctf_demo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES planner_5d_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/robot_obstacle_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
