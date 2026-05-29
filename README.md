# practica-final-ra-mobil

ROS Noetic catkin package for a capture-the-flag project with two TurtleBot3 robots in Gazebo.

Catkin package name: **`ctf_navigation`**

Quick start:
1) Install dependencies (Gazebo + navigation stack):
```bash
sudo apt update
sudo apt install \
  ros-noetic-turtlebot3-gazebo \
  ros-noetic-turtlebot3-msgs \
  ros-noetic-turtlebot3-description \
  ros-noetic-turtlebot3-navigation \
  ros-noetic-costmap-2d \
  ros-noetic-multirobot-map-merge \
  ros-noetic-nav-core \
  ros-noetic-navigation
```

2) Build:
```bash
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
```

3) Run the full CTF demo (Gazebo + navigation + autonomous demo node):
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ctf_navigation ctf_demo.launch
```

Optional launches:
- Gazebo only (two robots, no navigation): `roslaunch ctf_navigation simulation.launch`
- Navigation on the CTF map: `roslaunch ctf_navigation navigation.launch`
- SLAM with gmapping: `roslaunch ctf_navigation gmapping.launch`
- Map merge only: `roslaunch ctf_navigation map_merge.launch`
- Shared SLAM with known initial poses: `roslaunch ctf_navigation slam_demo.launch run_demo:=false`

Demo arguments (example):
```bash
roslaunch ctf_navigation ctf_demo.launch local_planner:=5d localization:=static run_demo:=true
```
