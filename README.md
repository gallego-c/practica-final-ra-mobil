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
roslaunch ctf_navigation ctf_game.launch flag_x:=2.0 flag_y:=-3.0
```

**Probar planificador local 5D:**

```bash
roslaunch ctf_navigation ctf_game.launch local_planner:=5d
```

### Modo oráculo (solo probar planning motion)

Los robots van directos a la bandera (sin exploración ni visión):

```bash
roslaunch ctf_navigation ctf_demo.launch
roslaunch ctf_navigation ctf_demo.launch local_planner:=5d flag_x:=1.0 flag_y:=2.0
```

### Visualización

```bash
rosrun rviz rviz
rqt_image_view /robot1/flag_detector/debug_image
```

| Topic | Descripción |
|---|---|
| `/robot1/flag_detector/flag_found` | ¿Ve la bandera? |
| `/robot1/flag_detector/flag_estimate` | Posición estimada (map) |
| `/ctf/markers` | Bases y estimaciones |
| `/robot1/move_base/local_costmap/costmap` | Costmap local |

---

## Nodos C++

| Nodo | Función |
|---|---|
| `ctf_coordinator_node` | Exploración, captura, persecución |
| `flag_detector_node` | Visión HSV + rango LIDAR + TF |
| `robot_obstacle_publisher_node` | Evita colisión entre robots |
| `ctf_demo_node` | Modo oráculo (coordenadas conocidas) |

---

## Flujo del juego (`ctf_game.launch`)

```
1. SEARCH
   - Genera waypoints de cobertura desde /map
   - Cada robot explora con move_base
   - flag_detector publica posición al ver rojo + LIDAR
   - Primer robot que llega → CAPTURA

2. CHASE
   - Portador → su base
   - Perseguidor → posición del portador (TF)
   - Gana portador si llega a base
   - Gana perseguidor si distancia ≤ catch_distance
```

---

## Parámetros útiles

**Visión** — `params/vision.yaml`:
- `min_blob_area`, `h_low1`/`h_high1`, `s_min`, `v_min`

**Juego** — `launch/ctf_game.launch`:
- `flag_capture_distance`, `flag_standoff_distance`, `capture_pause_sec`, `catch_distance`, `explore_step`
- Topic de evento: `/ctf/flag_captured` (`std_msgs/Bool`), tambien publicado como alias `/flag_captured`

**Navegación** — `params/costmap_*.yaml`, `params/planner_local_dwa.yaml`

---

## Migración a robots reales

| Simulación | Hardware |
|---|---|
| `flag_detector` (HSV rojo) | Sustituir detección en `src/vision/red_flag_detector.cpp` |
| Mapa estático | `gmapping` + `map_saver` |
| TF del otro robot | Localización compartida / comunicación entre robots |

---

## Regenerar mapa

```bash
python3 $(rospack find ctf_navigation)/scripts/generate_ctf_map.py
```

(Solo utilidad offline; los nodos en ejecución son 100 % C++.)
