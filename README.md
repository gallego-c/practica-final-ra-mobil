# ctf_navigation

Juego de **Capture The Flag (CTF)** multi-robot con dos TurtleBot3 Waffle Pi en Gazebo (ROS Noetic).

Este paquete implementa una simulación autónoma y distribuida donde dos robots exploran un mapa desconocido, localizan una bandera visualmente, e inician dinámicamente un juego de atrapa la bandera con roles de portador, perseguidor e interceptor.

---

## 🛠️ Arquitectura del Sistema

El sistema es un híbrido de nodos en **C++** y scripts en **Python** coordinados mediante tópicos de ROS:

### 🐍 Nodos de Control y Exploración (Python)
- **[slam_frontier_explorer_ctf.py](file:///home/arros/catkin_ws/src/practica-final-ra-mobil/scripts/slam_frontier_explorer_ctf.py)**: El coordinador principal del juego. Implementa la máquina de estados de cada robot (`EXPLORING`, `SEEKING_FLAG`, `CHASING_CARRIER`, `RETURNING_HOME`) y el selector de fronteras basado en diagramas de Voronoi y penalizaciones por territorio para evitar redundancia en la exploración.
- **[robot_coordinator.py](file:///home/arros/catkin_ws/src/practica-final-ra-mobil/scripts/robot_coordinator.py)**: Gestiona la prevención de colisiones inter-robot. Publica las huellas de los robots como obstáculos virtuales en una nube de puntos (`PointCloud2`) para que `move_base` los esquive en tiempo real, resolviendo bloqueos mutuos mediante prioridades temporales.
- **[laser_obstacle_filter.py](file:///home/arros/catkin_ws/src/practica-final-ra-mobil/scripts/laser_obstacle_filter.py)**: Filtra los datos de los sensores láser (LIDAR) para eliminar reflejos internos de los propios chasis de los robots.

### ⚡ Nodos de Alto Rendimiento (C++)
- **[flag_detector_node.cpp](file:///home/arros/catkin_ws/src/practica-final-ra-mobil/src/nodes/flag_detector_node.cpp)**: Procesa el flujo de la cámara RGB de cada robot utilizando OpenCV (filtros HSV) para detectar la bandera roja en tiempo real. Cruza los píxeles de la bandera detectada con el escaneo láser más cercano para estimar la posición exacta $(x, y)$ en el mapa global.

---

## 📂 Estructura del Proyecto

```
ctf_navigation/
├── launch/
│   ├── slam_multi_robot_ctf.launch  # ← Launch principal de la simulación
│   ├── simulation.launch            # Inicialización de Gazebo con robots y bandera
│   ├── shared_slam.launch           # GMapping individual y fusión de mapas (map_merge)
│   ├── slam_navigation.launch       # Move_base con DWA planner y costmaps
│   └── rviz/                        # Visualización y debug
├── scripts/
│   ├── slam_frontier_explorer_ctf.py # Lógica de juego, fronteras y estados
│   ├── robot_coordinator.py          # Evasión inter-robot y prioridades
│   └── laser_obstacle_filter.py      # Filtro de ruido del láser
├── src/
│   └── nodes/
│       └── flag_detector_node.cpp    # Procesamiento visual HSV de la bandera
├── params/                           # Configuración de Costmaps, DWA Planner y Visión
├── worlds/                           # Entornos virtuales (.world)
└── package.xml                       # Metadatos del paquete (Nombre oficial: ctf_navigation)
```

---

## ⚙️ Requisitos e Instalación

Asegúrate de instalar todas las dependencias necesarias de ROS Noetic:

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
  ros-noetic-navigation \
  ros-noetic-gmapping \
  ros-noetic-cv-bridge \
  libopencv-dev
```

Configura tu variable de entorno del robot en tu `.bashrc`:
```bash
export TURTLEBOT3_MODEL=waffle_pi
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
```

---

## 🚀 Compilación y Ejecución

### 1. Compilar el Workspace
Asegúrate de cargar primero el entorno global de ROS:
```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Lanzar la Simulación de CTF
Para lanzar la simulación en el mapa de las **9 habitaciones** (por defecto recomendado):
```bash
roslaunch ctf_navigation slam_multi_robot_ctf.launch world:=nine_rooms_world
```

Para lanzarla en otros mundos disponibles (`hallway_world` o `hallway_obstacles_world`):
```bash
roslaunch ctf_navigation slam_multi_robot_ctf.launch world:=hallway_world
```

### 3. Parámetros útiles al lanzar
Puedes reposicionar la bandera roja dinámicamente desde el propio comando usando `flag_x` y `flag_y`:
```bash
roslaunch ctf_navigation slam_multi_robot_ctf.launch world:=nine_rooms_world flag_x:=1.5 flag_y:=-2.5
```

---

## 💡 Consejos de Rendimiento / Solución de Problemas

La simulación ejecuta múltiples nodos complejos en paralelo (2 SLAM Gmapping, 2 costmaps locales/globales, 2 detectores visuales, etc.). Si notas lag o tirones:

1. **Limpiar procesos colgados (Zombies)**:
   Gazebo a veces no se cierra limpiamente al pulsar `Ctrl+C`. Si la simulación va muy lenta al reiniciar, ejecuta este comando para limpiar la memoria:
   ```bash
   killall -9 gzserver gzclient rosmaster rviz
   ```
2. **Cerrar la interfaz de Gazebo**:
   La ventana física 3D de Gazebo (`gzclient`) consume una cantidad enorme de recursos de GPU/CPU. Puedes cerrarla directamente una vez iniciada la simulación y controlar todo el progreso de la exploración y el juego desde **RViz**, que es mucho más ligero y muestra toda la telemetría e intenciones de los robots.
