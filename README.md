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
# ctf_navigation

Juego **Capture The Flag** con dos TurtleBot3 Waffle Pi en Gazebo (ROS Noetic).
El núcleo del juego está en C++, con utilidades Python para SLAM y soporte multi-robot.

Los robots pueden jugar al CTF clásico, explorar con SLAM en paralelo y compartir un mapa fusionado para navegar hacia la bandera.

---

## Estructura del paquete

```text
ctf_navigation/
├── include/ctf_navigation/
│   ├── common/
│   ├── vision/
│   ├── game/
│   └── planner_5d.h
├── src/
│   ├── nodes/
│   ├── game/
│   ├── vision/
│   └── planner_5d.cpp
├── scripts/
│   ├── generate_ctf_map.py
│   ├── robot_obstacle_publisher.py
│   └── slam_frontier_explorer.py
├── launch/
│   ├── ctf_game.launch
│   ├── ctf_demo.launch
│   ├── simulation.launch
│   ├── navigation.launch
│   ├── gmapping.launch
│   ├── map_merge.launch
│   ├── shared_slam.launch
│   ├── slam_navigation.launch
│   └── slam_demo.launch
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
  ros-noetic-map-server ros-noetic-cv-bridge ros-noetic-gmapping \
  ros-noetic-multirobot-map-merge ros-noetic-rospy ros-noetic-tf \
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

### CTF clásico

```bash
roslaunch ctf_navigation ctf_game.launch
```

### Modo oráculo

Ambos robots van directos a la bandera, sin exploración ni visión:

```bash
roslaunch ctf_navigation ctf_demo.launch
```

### SLAM multi-robot

Exploración en paralelo con mapa fusionado:

```bash
roslaunch ctf_navigation slam_demo.launch run_ctf:=false
```

Exploración + CTF después de cubrir el mapa:

```bash
roslaunch ctf_navigation slam_demo.launch local_planner:=5d run_demo:=true
```

### Launches útiles

- Gazebo solo: `roslaunch ctf_navigation simulation.launch`
- Navegación sobre un mapa ya conocido: `roslaunch ctf_navigation navigation.launch`
- GMapping por robot: `roslaunch ctf_navigation gmapping.launch`
- Fusión de mapas: `roslaunch ctf_navigation map_merge.launch`

### Utilidad offline

```bash
python3 $(rospack find ctf_navigation)/scripts/generate_ctf_map.py
```

(Los nodos en ejecución mezclan C++ y Python según el modo.)
