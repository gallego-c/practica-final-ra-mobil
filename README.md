# ctf_navigation

Juego **Capture The Flag** con dos TurtleBot3 Waffle Pi en Gazebo (ROS Noetic).

Incluye:
- **Visión** (detección de bandera con cámara + LIDAR) — branch `visio`
- **SLAM multi-robot** (gmapping + map_merge + exploración por fronteras) — branch `slam`

---

## Requisitos

```bash
sudo apt install \
  ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-description \
  ros-noetic-turtlebot3-navigation ros-noetic-move-base \
  ros-noetic-dwa-local-planner ros-noetic-global-planner \
  ros-noetic-map-server ros-noetic-gmapping ros-noetic-multirobot-map-merge \
  ros-noetic-cv-bridge libopencv-dev ros-noetic-gazebo-ros

export TURTLEBOT3_MODEL=waffle_pi
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
```

## Compilación

```bash
cd ~/catkin_ws && catkin_make
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rospack find ctf_navigation
ls $(rospack find ctf_navigation)/launch/slam_demo.launch
```

---

## Uso — Juego con visión (`visio`)

```bash
roslaunch ctf_navigation ctf_game.launch
roslaunch ctf_navigation ctf_game.launch local_planner:=5d
```

Modo oráculo (sin visión, prueba de planificadores):

```bash
roslaunch ctf_navigation ctf_demo.launch local_planner:=5d
```

---

## Uso — SLAM multi-robot (`slam`)

Solo exploración (maximizar mapa desconocido):

```bash
roslaunch ctf_navigation slam_demo.launch run_ctf:=false local_planner:=5d
```

Exploración + CTF oráculo después:

```bash
roslaunch ctf_navigation slam_demo.launch local_planner:=5d run_demo:=true
```

Otros launches SLAM:
- `roslaunch ctf_navigation shared_slam.launch` — gmapping + map merge
- `roslaunch ctf_navigation slam_navigation.launch` — move_base sobre `/merged_map`
- `roslaunch ctf_navigation simulation.launch` — solo Gazebo

---

## Estructura

```
include/ctf_navigation/
  common/     geometry, markers, move_base client, slam_wait
  vision/     detección bandera roja
  game/       exploración y agente
  planner_5d.h

src/nodes/
  ctf_coordinator_node.cpp   # juego realista
  ctf_demo_node.cpp          # oráculo (+ esperas SLAM)
  flag_detector_node.cpp
  robot_obstacle_publisher_node.cpp

scripts/
  slam_frontier_explorer.py  # exploración por fronteras (SLAM)
  robot_obstacle_publisher.py
```

---

## Nodos

| Nodo | Función |
|------|---------|
| `ctf_coordinator_node` | Exploración + visión + captura + persecución |
| `ctf_demo_node` | Oráculo (coordenadas conocidas) |
| `flag_detector_node` | Visión HSV + LIDAR |
| `slam_frontier_explorer` | Exploración SLAM por fronteras |
| `robot_obstacle_publisher` | Evita colisión entre robots (SLAM) |

---

## Visualización

```bash
rosrun rviz rviz
rqt_image_view /robot1/flag_detector/debug_image
```
