# practica-final-ra-mobil

ROS Noetic catkin package for a capture-the-flag project with two robots.

This repo includes a C++ keyboard teleop for TurtleBot3 Waffle.

Quick start:
1) Install TurtleBot3 Gazebo packages:
	sudo apt update
	sudo apt install ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3-description
2) Build:
	cd ~/catkin_ws && catkin_make
	source ~/catkin_ws/devel/setup.bash
3) Run Gazebo:
	export TURTLEBOT3_MODEL=waffle
	roslaunch turtlebot3_gazebo turtlebot3_world.launch
4) Run teleop (new terminal):
	source ~/catkin_ws/devel/setup.bash
	roslaunch practica_final_ra_mobil teleop_keyboard.launch