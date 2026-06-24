# CTF Navigation (Capture The Flag)

Benvingut al repositori de **ctf_navigation**! Aquest projecte implementa un emocionant joc autònom i distribuït de *Capture The Flag* (Atrapa la Bandera) utilitzant robòtica mòbil multiagent a ROS Noetic i Gazebo.

Dos robots (TurtleBot3 Waffle Pi) cooperen i competeixen per explorar un entorn desconegut, localitzar visualment una bandera vermella i executar tàctiques de captura i intercepció dinàmica.

---

## Fases del Joc

El joc es divideix en diverses fases d'estat gestionades pel coordinador principal:

1. **Fase d'Exploració (`EXPLORING`)**:
   - Ambdós robots inicien sense coneixement del mapa.
   - Utilitzen **SLAM distribuït** (Gmapping + map_merge) per crear un mapa global.
   - Navegació basada en fronteres utilitzant **Diagrames de Voronoi** i penalitzacions territorials per evitar que els robots explorin les mateixes zones.

2. **Cerca i Aproximació (`PURSUING_FLAG` / `APPROACHING_SHARED`)**:
   - Els robots processen el feed de les seves càmeres RGB per detectar el color vermell (la bandera).
   - Si un robot detecta la bandera, calcula la seva posició combinant la visió amb l'**escàner làser (LIDAR)** i comparteix la ubicació amb el seu company.
   - El robot que la veu inicia l'aproximació, mentre el company es posiciona estratègicament per interceptar més endavant.

3. **Captura i Persecució (`CAPTURED` / `CHASE`)**:
   - El primer robot a assolir la bandera (a una distància segura per no col·lidir) es converteix en el **Portador** (Carrier).
   - L'objectiu del Portador és tornar a la seva base (punt d'inici) utilitzant `move_base`.
   - L'altre robot assumeix el rol de **Perseguidor** (Pursuer / Interceptor) i calcula trajectòries dinàmiques per caçar o interceptar el Portador abans que arribi a casa.

---

## Arquitectura del Sistema

El paquet combina nodes d'alt rendiment en C++ per al processament sensorial i scripts flexibles en Python per a la lògica d'estats:

### Nodes de Control i Lògica (Python)
- **`slam_frontier_explorer_ctf.py`**: El "cervell" del joc. Gestiona la màquina d'estats, l'algorisme d'exploració de fronteres i orquestra els moviments de persecució i fugida.
- **`robot_coordinator.py`**: Sistema intel·ligent de prevenció de col·lisions inter-robot. Converteix temporalment els companys en "obstacles" per als costmaps locals, evitant bloquejos mutus mitjançant un sistema de prioritats temporals.
- **`laser_obstacle_filter.py`**: Neteja el soroll del LIDAR, filtrant reflexos fantasmes del propi xassís del TurtleBot3.

### Nodes de Processament (C++)
- **`flag_detector_node.cpp`**: Processa les imatges de la càmera en temps real mitjançant OpenCV. Detecta píxels vermells (HSV), estima l'àrea i el centroide de la bandera, i projecta aquest vector sobre les lectures del làser per obtenir una coordenada global precisa `(x, y)` al mapa.

---

## Estructura del Projecte

```text
ctf_navigation/
├── launch/
│   ├── slam_multi_robot_ctf.launch  # Launch principal del joc
│   ├── simulation.launch            # Gazebo, models de robots i bandera
│   ├── shared_slam.launch           # Nodes de Gmapping i map_merge
│   ├── slam_navigation.launch       # move_base (DWA i Costmaps)
│   └── rviz/                        # Configuracions de visualització
├── scripts/
│   ├── slam_frontier_explorer_ctf.py
│   ├── robot_coordinator.py
│   └── laser_obstacle_filter.py
├── src/
│   └── nodes/
│       └── flag_detector_node.cpp   # Node C++ de visió artificial
├── params/                          # Paràmetres de navegació (yaml)
├── worlds/                          # Entorns de Gazebo (.world)
└── package.xml
```

---

## Requisits i Instal·lació

El projecte està dissenyat per a **Ubuntu 20.04** i **ROS Noetic**.

### 1. Dependències del sistema
Assegura't d'instal·lar els paquets necessaris:

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

### 2. Variables d'Entorn
El sistema utilitza el model **Waffle Pi** per defecte, ja que compta amb la càmera RGB necessària per a la visió:

```bash
export TURTLEBOT3_MODEL=waffle_pi
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
```

---

## Compilació i Execució

### Compilar el Workspace

Clona aquest repositori dins de la carpeta `src` del teu workspace de Catkin (`~/catkin_ws/src`) i compila:

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Llançar la Simulació

Inicia l'ecosistema complet (Gazebo, RViz, Controladors, Nodes de Visió i Lògica). Et recomanem l'entorn de les **9 habitacions** per a una experiència completa d'exploració:

```bash
roslaunch ctf_navigation slam_multi_robot_ctf.launch world:=nine_rooms_world
```

També disposes d'altres mons com `hallway_world` o `hallway_obstacles_world`:
```bash
roslaunch ctf_navigation slam_multi_robot_ctf.launch world:=hallway_world
```

**Truc**: Pots canviar la posició inicial de la bandera al vol:
```bash
roslaunch ctf_navigation slam_multi_robot_ctf.launch world:=nine_rooms_world flag_x:=1.5 flag_y:=-2.5
```

---

## Consells i Solució de Problemes

A causa de la quantitat de processos simultanis (2 instàncies de Gazebo controllers, 2 SLAMs, 2 processadors de visió, map_merge, move_base locals/globals), el sistema és intensiu en CPU/RAM.

1. **Tancar el client de Gazebo**: Per guanyar moltíssims FPS i reduir càrrega a la CPU, tanca la finestra 3D de Gazebo un cop llançada la simulació. Pots veure el mapa i els robots perfectament des de **RViz**.
2. **Matar processos "Zombies"**: Si `Ctrl+C` no tanca tot netament i ROS es queda travat, neteja l'entorn amb:
   ```bash
   killall -9 gzserver gzclient rosmaster rviz
   ```
3. **Paciència a l'inici**: Els nodes de navegació (`move_base`) tenen un temps d'espera d'uns 30 segons (configurable) a l'inici per donar temps a que Gazebo carregui els controladors físics.

---
**Autor**: Paessa
**Llicència**: MIT
