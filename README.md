# ctf_navigation - Branca: robot_real

Aquesta branca conté l'adaptació completa del projecte **Capture The Flag (CTF)** per ser executat amb **dos robots TurtleBot3 Waffle Pi reals**.

L'objectiu principal d'aquesta implementació és traslladar tota la complexitat algorísmica de la simulació (SLAM col·laboratiu, exploració per fronteres de Voronoi, visió per computador i evasió mútua d'obstacles) a un entorn físic real. Això implica gestionar desafiaments com la deriva de l'odometria, la sincronització de rellotges, les limitacions d'ample de banda a la xarxa i el processament sensorial en maquinari limitat.

El flux de la màquina d'estats (Exploració -> Cerca -> Captura -> Persecució) es manté idèntic a la versió de simulació, però els paràmetres de navegació, visió i transformades han estat rigorosament ajustats per al món físic.

---

## Canvis i Adaptacions per al Món Real

Per fer possible l'execució en robots reals, s'han introduït els següents ajustaments crítics a la pila de programari:

1. **Gestió de la Xarxa i Sincronització (NTP)**
   S'ha afegit l'script `clock_drift_check.py` per garantir que els rellotges interns del PC principal i les Raspberry Pi dels robots estiguin perfectament sincronitzats. Les transformades de TF fallen estrepitosament en entorns multiagent si hi ha més de 50ms de desfasament temporal.

2. **Publicació de Transformades Cinemàtiques**
   A diferència de Gazebo, on la jerarquia de TF es pot injectar fàcilment, als robots reals utilitzem `odom_tf_broadcaster.py` per garantir una emissió neta i contínua de les transformades `odom -> base_footprint -> base_link -> base_scan`. 

3. **Visió i Processament (ArUco + HSV)**
   A la branca real, la detecció per color pot patir molt per la il·luminació variable de l'entorn. Es proveeix suport per a la visió millorada (`aruco_flag_detector.cpp` i els paràmetres adaptats a `vision_real.yaml`), permetent una millor resiliència utilitzant tant segmentació HSV per al vermell com reconeixement de marcadors fiducials.

4. **Filtres de Soroll per al LIDAR**
   Quan els robots físics operen en proximitat, l'escàner de l'un detecta l'altre, provocant falsos positius permanents al mapa SLAM (`laser_obstacle_filter.py`). S'han ajustat els paràmetres de filtre de l'empremta física per ignorar el volum del company sense introduir latència.

---

## Arquitectura i Fitxers Conservats

Tota l'estructura de la lògica autònoma i els nodes principals s'ha preservat de la versió de simulació:

- **Launchers d'Arrencada:**
  - `launch/real_multi_robot_ctf.launch`: Llançador principal del joc per a dos robots reals.
  - `launch/real_robot_flag_vision.launch`: Mode de depuració només per testejar les càmeres i l'espai de color en el lloc abans de jugar.
  - `launch/real_robot_tf.launch`: Injecció de coordenades i transformades estàtiques.

- **Configuracions i Paràmetres (Ajustats per hardware físic):**
  - `params/*_slam.yaml`
  - `params/gmapping_robot1.yaml` i `params/gmapping_robot2.yaml`
  - `params/vision_real.yaml`

- **Scripts de Control, Debug i Coordinació:**
  - `scripts/slam_frontier_explorer_ctf.py`: El cervell principal del joc.
  - `scripts/robot_coordinator.py`: Lògica per a la concessió de pas a nivell de Costmap.
  - `scripts/map_merge_debug.py` i `scripts/tf_map_merge.py`: Diagnòstics per analitzar si el SLAM cooperatiu està alineant correctament els mapes de tots dos robots.
  - `scripts/launch_config_echo.py`: Utilitat per abocar configuracions per pantalla en temps d'execució.

- **Nodes de Visió Avançada:**
  - `src/nodes/flag_detector_node.cpp`
  - `src/vision/aruco_flag_detector.cpp`

---

## Guia d'Execució i Notes Importants

Executar un sistema multiagent complex en maquinari real requereix una preparació acurada de l'entorn abans de llançar els scripts principals.

### Pas 1: Preparació dels Robots

1. Encén els dos TurtleBot3 Waffle Pi i connecta'ls a la mateixa xarxa Wi-Fi que el PC Mestre (ROS_MASTER_URI).
2. Assegura't de sincronitzar els rellotges (mitjançant *chrony* o *ntpdate*):
   ```bash
   sudo ntpdate -u <IP_DEL_PC_MESTRE>
   ```
3. Executa el "bringup" bàsic a cadascun dels robots (connexió amb l'OpenCR, motors i LIDAR):
   ```bash
   # Al robot 1:
   ROS_NAMESPACE=robot1 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=robot1
   
   # Al robot 2:
   ROS_NAMESPACE=robot2 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=robot2
   ```
4. Inicia els nodes de les càmeres (`cv_camera` o similar) a cada robot per tenir el flux de vídeo actiu a `/robot1/camera/image_raw` i `/robot2/camera/image_raw`.

### Pas 2: Verificació Prèvia (Molt Recomanat)

Abans de llançar tota la lògica d'exploració autònoma, assegura't que la visió funciona correctament a l'entorn de llum actual:

```bash
roslaunch ctf_navigation real_robot_flag_vision.launch
```
*Comprova els tòpics de debug a rqt_image_view per assegurar que la màscara HSV detecta el color de la bandera nítidament.*

### Pas 3: Llançar el Joc CTF

Un cop verificada la telemetria (`/scan`, `/odom` i càmeres) de tots dos robots, ja pots llançar l'orquestrador principal des del PC:

```bash
roslaunch ctf_navigation real_multi_robot_ctf.launch
```

**Paràmetre Extra:**
Si el bringup del robot ja està configurat per publicar de forma estàtica la cadena cinemàtica bàsica (`base_footprint -> base_link -> base_scan`), pots evitar conflictes a l'arbre de TF llançant amb l'opció:
```bash
roslaunch ctf_navigation real_multi_robot_ctf.launch publish_kinematic_tf:=false
```

---

## Solució de Problemes Freqüents a la Branca Real

- **Map_merge no acobla els mapes:** Això gairebé sempre passa perquè les estimacions d'odometria inicials (`/robot1/odom` vs `/robot2/odom`) han derivat molt des que es van encendre, o no se'ls ha donat una estimació d'inici `initial_pose` correcta. Si l'error és constant, atura els robots, alinea'ls al terra i torna a reiniciar el bringup.
- **Tironades i Pèrdua de Senyal:** Si els robots es mouen a batzegades, és probable que l'ample de banda de la xarxa Wi-Fi estigui saturat enviant dades completes de càmera i núvols de punts no comprimits. Fes ús de `image_transport/compressed` sempre que sigui possible.
- **Els robots topen l'un amb l'altre de front:** Assegura't que `robot_coordinator.py` està publicant correctament al tòpic `other_robot_cloud` i que els paràmetres del DWA `obstacle_layer` estan llegint aquest tòpic amb una prioritat alta.
