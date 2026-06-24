# ctf_navigation - robot_real

Rama limpia para ejecutar Capture The Flag con dos TurtleBot3 Waffle Pi reales.
Se ha recortado el stack de simulacion y demos antiguos para dejar solo lo que
usa el despliegue fisico: SLAM por robot, fusion de mapas por TF, navegacion,
coordinacion entre robots y deteccion de bandera con ArUco/AprilTag.

## Launch principales

```bash
# Juego completo con robots reales
roslaunch ctf_navigation real_multi_robot_ctf.launch

# Solo vision de bandera para ambos robots
roslaunch ctf_navigation real_robot_flag_vision.launch
```

Antes de lanzar `real_multi_robot_ctf.launch`, los bringups de los dos robots
deben estar activos y publicando bajo los namespaces `robot1` y `robot2`.

## Estructura

```text
ctf_navigation/
|-- launch/
|   |-- real_multi_robot_ctf.launch
|   |-- real_robot_flag_vision.launch
|   |-- real_robot_tf.launch
|   |-- shared_slam.launch
|   |-- slam_navigation.launch
|   |-- map_merge_tf.launch
|   `-- slam_world_tf.launch
|-- scripts/
|   |-- slam_frontier_explorer_ctf.py
|   |-- robot_coordinator.py
|   |-- laser_obstacle_filter.py
|   |-- tf_map_merge.py
|   |-- odom_tf_broadcaster.py
|   |-- map_merge_debug.py
|   |-- launch_config_echo.py
|   `-- clock_drift_check.py
|-- src/
|   |-- nodes/flag_detector_node.cpp
|   `-- vision/aruco_flag_detector.cpp
|-- include/ctf_navigation/
|   |-- common/
|   `-- vision/
|-- params/
`-- rviz/
```

## Dependencias ROS

```bash
sudo apt update
sudo apt install \
  ros-noetic-turtlebot3-bringup \
  ros-noetic-turtlebot3-navigation \
  ros-noetic-navigation \
  ros-noetic-gmapping \
  ros-noetic-global-planner \
  ros-noetic-dwa-local-planner \
  ros-noetic-costmap-2d \
  ros-noetic-cv-bridge \
  ros-noetic-rqt-image-view \
  libopencv-dev
```

## Compilacion

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Notas de ejecucion real

- Sincroniza relojes en todos los PCs antes de arrancar ROS.
- Verifica que los topics existan como `/robot1/scan`, `/robot2/scan`,
  `/robot1/odom`, `/robot2/odom` y las camaras `cv_camera`.
- Si el bringup ya publica la cadena `base_footprint -> base_link -> base_scan`,
  lanza con `publish_kinematic_tf:=false`.
- La fusion de mapas se hace con `tf_map_merge.py`, no con
  `multirobot_map_merge`.
