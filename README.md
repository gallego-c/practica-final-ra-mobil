# ctf_navigation

Juego **Capture The Flag** con dos TurtleBot3 Waffle Pi en Gazebo (ROS Noetic).
Todo el código de ejecución está en **C++** (sin nodos Python).

Los robots exploran el mapa, detectan la bandera con la **cámara**, planifican con
`move_base`, y el primero que la captura vuelve a su base mientras el otro lo persigue.

---

## Estructura del paquete

```
ctf_navigation/
├── include/ctf_navigation/
│   ├── common/          # Utilidades compartidas
│   │   ├── geometry.hpp
│   │   ├── tf_helper.hpp
│   │   ├── move_base_client.hpp
│   │   └── markers.hpp
│   ├── vision/          # Detección de bandera (OpenCV HSV)
│   │   ├── red_flag_detector.hpp
│   │   └── laser_utils.hpp
│   ├── game/            # Lógica del juego
│   │   ├── types.hpp
│   │   ├── exploration.hpp
│   │   └── robot_agent.hpp
│   └── planner_5d.h     # Plugin local move_base
├── src/
│   ├── nodes/           # Ejecutables ROS
│   │   ├── ctf_coordinator_node.cpp   # Juego realista
│   │   ├── ctf_demo_node.cpp          # Modo oráculo
│   │   ├── flag_detector_node.cpp     # Visión + LIDAR
│   │   └── robot_obstacle_publisher_node.cpp
│   ├── game/
│   ├── vision/
│   └── planner_5d.cpp
├── launch/
│   ├── ctf_game.launch      # ← Juego completo
│   ├── ctf_demo.launch      # Modo oráculo (pruebas planner)
│   ├── simulation.launch
│   └── navigation.launch
├── params/
├── maps/
├── models/ctf_flag.urdf
└── worlds/ctf_world.world
```

---

## Requisitos

Ubuntu 20.04 + ROS Noetic:

```bash
sudo apt install ros-noetic-turtlebot3-description ros-noetic-turtlebot3-gazebo \
     ros-noetic-move-base ros-noetic-dwa-local-planner ros-noetic-global-planner \
     ros-noetic-map-server ros-noetic-cv-bridge \
     libopencv-dev ros-noetic-gazebo-ros

export TURTLEBOT3_MODEL=waffle_pi
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
```

---

## Compilación

```bash
cd ~/catkin_ws/src
# copiar o enlazar el paquete ctf_navigation aquí
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Uso

### Juego CTF realista (exploración + visión)

```bash
roslaunch ctf_navigation ctf_game.launch
```

**Cambiar posición de la bandera:**

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
roslaunch ctf_navigation ctf_game.launch local_planner:=5d
```

### Modo oráculo (solo probar planning motion)

Los robots van directos a la bandera (sin exploración ni visión):

```bash
roslaunch ctf_navigation ctf_demo.launch
```

Optional launches:
- Gazebo only (two robots, no navigation): `roslaunch ctf_navigation simulation.launch`
- Navigation on the CTF map: `roslaunch ctf_navigation navigation.launch`
- SLAM with gmapping: `roslaunch ctf_navigation gmapping.launch`
- Map merge only: `roslaunch ctf_navigation map_merge.launch`
- Shared SLAM with known initial poses: `roslaunch ctf_navigation slam_demo.launch run_demo:=false`

SLAM demo — maximize map coverage, then optional CTF:
```bash
# Exploration only (no flag/chase):
roslaunch ctf_navigation slam_demo.launch run_ctf:=false

# Exploration + CTF after ~72% map coverage:
roslaunch ctf_navigation slam_demo.launch local_planner:=5d run_demo:=true
```

Exploration stops when coverage ≥ 72% or no frontiers remain (up to 5 min).
Each robot picks its own frontiers (Voronoi split) so they spread across the arena.

Demo arguments (example):
```bash
python3 $(rospack find ctf_navigation)/scripts/generate_ctf_map.py
```

(Solo utilidad offline; los nodos en ejecución son 100 % C++.)
