# ctf_navigation

Juego **Capture The Flag (CTF)** con dos TurtleBot3 Waffle Pi en un entorno de simulación de Gazebo (ROS Noetic).

Este paquete implementa un sistema competitivo y cooperativo multi-robot que combina:
- **Mapeado Colaborativo (SLAM)** en tiempo real (GMapping + `multirobot_map_merge`).
- **Exploración Autónoma** de fronteras por regiones Voronoi.
- **Navegación y Planificación** local (DWA y un planificador personalizado A* 5D).
- **Visión por Computador** para la localización tridimensional de la bandera roja (HSV + LIDAR).
- **Lógica del Juego CTF** (fases de búsqueda, captura, huida a base y persecución).
- **Coordinación y Seguridad** para evitar colisiones inter-robot y evitar auto-mapeado de los chasis.

---

## Estructura del Proyecto

El código está estructurado en módulos para facilitar la modularidad de C++ y Python:

```
ctf_navigation/
├── include/ctf_navigation/
│   ├── common/           # Utilidades de geometría, marcadores, cliente move_base, TF
│   ├── game/             # Definiciones de agentes de juego y planeador de exploración
│   ├── vision/           # Detección de bandera y procesamiento LIDAR
│   └── planner_5d.h      # Cabecera del planificador local personalizado 5D
├── src/
│   ├── game/             # Implementación de exploración y control de agentes
│   ├── vision/           # Implementación de segmentación HSV y posicionamiento de bandera
│   ├── nodes/            # Nodos ROS C++ (ctf_coordinator, flag_detector, etc.)
│   └── planner_5d.cpp    # Plugin de move_base (Planner5D) basado en búsqueda A* 5D
├── scripts/              # Scripts de Python para exploración SLAM y coordinación
│   ├── slam_frontier_explorer_ctf.py     # Exploración SLAM + lógica completa de CTF en Python
│   ├── slam_frontier_explorer.py         # Explorador SLAM puro (sin juego) de dos robots
│   ├── slam_frontier_explorer_single.py  # Explorador SLAM monorrobot optimizado (clearance fallbacks)
│   ├── robot_coordinator.py              # Coordinador de prioridades (robot2 cede el paso)
│   ├── robot_obstacle_publisher.py       # Publicador de nube de puntos del footprint del otro robot
│   └── laser_obstacle_filter.py          # Filtro LIDAR para evitar el auto-mapeado del otro robot
├── launch/               # Ficheros de lanzamiento para múltiples configuraciones de juego
├── params/               # Configuración de costmaps, planificadores y visión
├── maps/                 # Mapas pre-guardados para localización estática
├── worlds/               # Entorno Gazebo para la simulación
└── planner_5d_plugin.xml # Registro del plugin de planificación local para move_base
```

---

## Requisitos

Instala las dependencias necesarias de ROS Noetic y librerías externas:

```bash
sudo apt update
sudo apt install \
  ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-description \
  ros-noetic-turtlebot3-navigation ros-noetic-move-base \
  ros-noetic-dwa-local-planner ros-noetic-global-planner \
  ros-noetic-map-server ros-noetic-gmapping ros-noetic-multirobot-map-merge \
  ros-noetic-cv-bridge libopencv-dev ros-noetic-gazebo-ros \
  ros-noetic-explore-lite

export TURTLEBOT3_MODEL=waffle_pi
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
```

## Compilación

Crea tu workspace y compila el paquete usando `catkin_make`:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Comprobar que ROS encuentra el paquete correctamente
rospack find ctf_navigation
```

---

## Modos de Uso y Ejecución

El repositorio soporta tanto entornos dinámicos con SLAM cooperativo como entornos con mapa estático pre-construido.

### 1. Juego CTF Completo sobre SLAM Dinámico (Python)
Arranca una simulación cooperativa donde los robots construyen un mapa común utilizando GMapping y `map_merge`. Paralelamente, exploran autónomamente por fronteras Voronoi. El robot que detecta la bandera mediante visión la captura, y el juego transiciona a la fase de huida y persecución:

```bash
roslaunch ctf_navigation slam_multi_robot_ctf.launch local_planner:=dwa
```
*Parámetros disponibles:*
- `local_planner`: `dwa` (por defecto) o `5d` (planificador A* 5D personalizado).
- `flag_x` / `flag_y`: Posición de la bandera (por defecto `-3.2` y `3.2`).
- `robot1_x` / `robot1_y` / `robot2_x` / `robot2_y`: Poses de inicio de los chasis.

---

### 2. Exploración Cooperativa SLAM Pura (Sin Juego)
Si solo deseas evaluar la maximización del área mapeada utilizando exploración de fronteras Voronoi en un mapa dinámico fusionado:

- **Con el explorador de fronteras Voronoi personalizado (Python):**
  ```bash
  roslaunch ctf_navigation slam_demo.launch run_ctf:=false
  ```
- **Con el explorador estándar `explore_lite` de ROS:**
  ```bash
  roslaunch ctf_navigation slam_multi_robot_explore_lite.launch
  ```

---

### 3. Exploración Monorrobot con SLAM
Lanza un único robot en Gazebo mapeando su entorno de forma autónoma. Cuenta con niveles de seguridad (`clearance`) decrementales para adentrarse por zonas muy estrechas con seguridad:

- **Con explorador de fronteras personalizado (Python):**
  ```bash
  roslaunch ctf_navigation slam_single_robot.launch local_planner:=dwa
  ```
- **Con `explore_lite` (ROS):**
  ```bash
  roslaunch ctf_navigation slam_single_robot_explore_lite.launch
  ```

---

### 4. Juego CTF Realista con Mapa Estático (C++)
Ejecuta el juego utilizando un mapa pre-construido cargado a través de `map_server`. La coordinación y lógica de juego están implementadas en C++ (`ctf_coordinator_node`):

```bash
roslaunch ctf_navigation ctf_game.launch local_planner:=dwa rviz:=true
```

---

### 5. Juego Oráculo con Mapa Estático (C++)
Para pruebas puras de planificación y persecución sin el módulo de visión activa (los robots conocen en todo momento las coordenadas exactas de la bandera a nivel de software):

```bash
roslaunch ctf_navigation ctf_demo.launch local_planner:=5d
```

---

## Nodos y Scripts Principales

| Nodo / Script | Lenguaje | Descripción |
| :--- | :---: | :--- |
| `flag_detector_node` | C++ | Procesa la cámara RGB (HSV rojo) e intersecta con LIDAR para estimar la posición 3D de la bandera. |
| `ctf_coordinator_node` | C++ | Coordinador del juego con mapa estático (controla la búsqueda, captura y persecución). |
| `ctf_demo_node` | C++ | Versión oráculo (sin visión) para coordinar el juego con coordenadas de bandera conocidas. |
| `robot_obstacle_publisher_node` | C++ | Transforma el footprint de un robot en un obstáculo LIDAR / PointCloud2 para el otro robot. |
| `slam_frontier_explorer_ctf.py` | Python | Exploración por fronteras cooperativa Voronoi + lógica completa de captura/retorno/chase sobre SLAM. |
| `slam_frontier_explorer.py` | Python | Exploración cooperativa pura por fronteras (Voronoi) en SLAM. |
| `slam_frontier_explorer_single.py`| Python | Exploración monorrobot por fronteras con replanificación y holguras adaptativas (clearance fallback). |
| `robot_coordinator.py` | Python | Publica obstáculos tridimensionales inter-robot y prioriza el paso (robot2 se para si robot1 está cerca). |
| `laser_obstacle_filter.py` | Python | Remueve del escaneo LIDAR del robot A los puntos correspondientes al chasis del robot B. |

---

## Arquitectura de Control y Coordinación

1. **Auto-Mapeado Evitado (`laser_obstacle_filter.py`)**:
   Cuando dos robots mapean de forma dinámica, la presencia de uno frente al LIDAR del otro provoca que se dibujen "paredes ficticias" en la rejilla de ocupación de GMapping. Este script elimina de forma dinámica los puntos que caen dentro del footprint del otro robot antes de enviar el scan a SLAM.
   
2. **Evitación e Interferencia de Costmaps (`robot_coordinator.py`)**:
   Los robots leen la pose del contrario mediante transformadas de TF (`base_footprint`) y publican una nube de puntos sintética (`PointCloud2`) en `/robotX/other_robot_cloud`. Esta nube es alimentada al `obstacle_layer` del costmap local de move_base de forma que se evitan en tiempo real.
   
3. **Ceder Paso por Proximidad**:
   Si la distancia entre ambos robots desciende de `1.5` metros, el coordinador intercepta y sobreescribe los comandos cmd_vel del robot 2 enviándole velocidad cero (`Twist()`), cediéndole la prioridad de paso al robot 1. La marcha se reanuda una vez se separan más de `1.8` metros.

4. **Planificador Local 5D A* (`Planner5D`)**:
   Este plugin local para move_base evalúa la cinemática del robot en 5 dimensiones (x, y, orientacion, velocidad lineal, velocidad angular). Permite giros controlados en espacios reducidos calculando la ruta óptima mediante A*.

---

## Herramientas de Visualización y Diagnóstico

### RViz (Visualización 3D)
Se proporciona una configuración preestablecida de Rviz para monitorizar las odometrías, costmaps, el mapa unificado `/merged_map` y la detección de la bandera mediante marcadores:

```bash
rosrun rviz rviz -d $(rospack find ctf_navigation)/rviz/ctf_game.rviz
```

### Diagnóstico de Imagen (Cámara)
Para observar en tiempo real la máscara HSV y el contorno de detección de la bandera procesado por OpenCV:

```bash
rosrun rqt_image_view rqt_image_view
# Selecciona el topic: /robot1/flag_detector/debug_image
```
