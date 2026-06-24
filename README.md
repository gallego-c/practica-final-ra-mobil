# ctf_navigation - robot_real

Rama para ejecutar Capture The Flag con dos TurtleBot3 Waffle Pi reales.

El flujo principal se conserva sin cambios funcionales:

```bash
roslaunch ctf_navigation real_multi_robot_ctf.launch
```

Para probar solo la vision de bandera en los dos robots:

```bash
roslaunch ctf_navigation real_robot_flag_vision.launch
```

## Stack real conservado

- `launch/real_multi_robot_ctf.launch`
- `launch/real_robot_flag_vision.launch`
- `launch/real_robot_tf.launch`
- `launch/shared_slam.launch`
- `launch/slam_navigation.launch`
- `launch/map_merge_tf.launch`
- `launch/slam_world_tf.launch`
- `params/*_slam.yaml`
- `params/gmapping_robot1.yaml`
- `params/gmapping_robot2.yaml`
- `params/vision_real.yaml`
- `scripts/clock_drift_check.py`
- `scripts/odom_tf_broadcaster.py`
- `scripts/map_merge_debug.py`
- `scripts/tf_map_merge.py`
- `scripts/launch_config_echo.py`
- `scripts/laser_obstacle_filter.py`
- `scripts/robot_coordinator.py`
- `scripts/slam_frontier_explorer_ctf.py`
- `src/nodes/flag_detector_node.cpp`
- `src/vision/aruco_flag_detector.cpp`

## Notas de ejecucion

- Arranca antes los bringups reales de `robot1` y `robot2`.
- Sincroniza relojes entre PCs antes de lanzar ROS.
- Comprueba que existan `/robot1/scan`, `/robot2/scan`, `/robot1/odom`,
  `/robot2/odom` y las camaras `cv_camera`.
- Si el bringup ya publica `base_footprint -> base_link -> base_scan`, lanza con
  `publish_kinematic_tf:=false`.
