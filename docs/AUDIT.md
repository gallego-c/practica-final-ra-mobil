# Auditoría de integración — ctf_navigation

Documento generado tras revisión exhaustiva del repositorio (launch, scripts, nodos C++, params, TF, topics).

---

## Resumen ejecutivo

El paquete contiene **tres stacks distintos** que no deben mezclarse:

| Stack | Launch principal | Coordinador | Mapa de planificación |
|-------|------------------|-------------|------------------------|
| **SLAM + CTF (recomendado)** | `slam_multi_robot_ctf.launch` | Python `slam_frontier_explorer_ctf.py` | `/merged_map` + validación local |
| **SLAM demo** | `slam_demo.launch` | C++ o solo exploración | Depende de `run_ctf` |
| **Mapa estático** | `ctf_game.launch` | C++ `ctf_coordinator_node` | `/map` |

La causa más frecuente de robots parados, metas imposibles o captura sin acercarse suele ser **desalineación entre el mapa donde se eligen metas y el mapa donde planifica `move_base`**, o **TF map↔odom incorrecto** en el stack estático.

---

## P0 — Errores críticos (rompen o invalidan la ejecución)

### 1. TF estático map→odom sin pose de spawn *(corregido)*

**Archivos:** `launch/navigation.launch`, `launch/ctf_game.launch`, `launch/ctf_demo.launch`

Con `localization:=static`, antes se publicaba identidad `map→robotN/odom` (0,0,0). Gazebo coloca el robot en `(-3,-3)` pero el planificador creía que estaba en `(0,0)` → **~3 m de error** en metas, bandera y bases.

**Corrección:** TF estático usa `robotN_x/y/yaw` de spawn. Los launches padre pasan esos argumentos.

### 2. Exploración en `/merged_map` vs costmap en `/robotN/map` *(parcialmente corregido)*

**Archivos:** `launch/slam_navigation.launch`, `scripts/slam_frontier_explorer_ctf.py`

El explorador elige fronteras en `/merged_map` (frame `map`). Si `move_base` planifica solo en `/robot1/map`, metas “libres” en el mapa fusionado pueden ser bloqueadas en el costmap local → **ABORTED**, giros, pararse delante de obstáculos.

**Corrección:**
- Nuevo arg `plan_on_merged_map` (default `true`) en `slam_navigation.launch`.
- Validación dual en `_safe_goal_near_frontier` (merged + mapa local del robot).
- Si sigue fallando: probar `plan_on_merged_map:=false` o revisar calidad del map merge.

### 3. `gmapping.launch` huérfano

**Archivo:** `launch/gmapping.launch`

Dos gmapping con `map_frame: map` → **conflicto TF**. Marcado DEPRECATED; usar `shared_slam.launch`.

---

## P1 — Problemas de comportamiento de alto impacto

### 4. Dos coordinadores CTF incompatibles

| Aspecto | C++ `ctf_coordinator_node` | Python `slam_frontier_explorer_ctf.py` |
|---------|---------------------------|----------------------------------------|
| Exploración | Grid waypoints | Fronteras Voronoi |
| `flag_capture_distance` | 0.40 m | 0.45 m |
| Captura | Solo distancia | Aproximación física + stall |
| `/ctf/pursuing_flag` | No publica | Sí (coordinator deja de hacer yield) |
| Uso | `ctf_game`, `slam_demo run_ctf:=true` | `slam_multi_robot_ctf` |

**No lanzar ambos a la vez.** Para SLAM+visión+CTF usar solo `slam_multi_robot_ctf.launch`.

### 5. Pipeline `cmd_vel` distinto por stack

| Stack | move_base → | Multiplexor |
|-------|-------------|-------------|
| SLAM multi-robot | `/robotN/cmd_vel_raw` | `robot_coordinator.py` → `/robotN/cmd_vel` |
| Estático (`ctf_game`) | `/robotN/cmd_vel` directo | Ninguno |

Si `slam_navigation` está activo pero falta `robot_coordinator`, **los robots no se mueven**.

### 6. Radio del otro robot inconsistente *(corregido en C++)*

| Componente | Radio |
|------------|-------|
| `robot_coordinator.py` | 0.28 m |
| `robot_obstacle_publisher_node.cpp` | ~~0.22~~ → **0.28 m** |
| `laser_obstacle_filter.py` | 0.32 m (filtro LIDAR, distinto propósito) |

### 7. `slam_demo.launch` confuso por defecto *(corregido)*

Antes: `run_ctf:=true` por defecto → C++ grid en SLAM, RViz de mapa estático.

Ahora: `run_ctf:=false` (solo exploración), RViz `slam_multi_robot_ctf.rviz`, `scan_filtered` en visión.

### 8. AMCL con poses fijas

Si usas `localization:=amcl`, las poses iniciales siguen los args de spawn (corregido en `navigation.launch`). Cambiar `world:=hallway_world` requiere coherencia spawn + AMCL + bases CTF.

---

## P2 — Deuda técnica / mantenimiento

- **`/ctf_navigation/spawns`** publicado en `simulation.launch` pero ningún nodo lo lee (las bases van por params de launch).
- **`robot_obstacle_publisher.py`** instalado pero no usado; el stack estático usa el nodo C++.
- **`costmap_global_slam.yaml`** vs overrides en launch — documentado en YAML.
- **`ctf_game.rviz`** muestra `/merged_map` aunque el stack estático no lo publica.
- **`explore_lite`** — tercer camino de exploración independiente.
- **Scripts offline** (`generate_ctf_map.py`) no instalados en catkin.
- **Cambiar `world:=`** sin mapa estático acorde rompe `ctf_game.launch` (siempre carga `ctf_map.yaml`).

---

## Matriz de topics / frames (referencia)

| Topic / frame | SLAM CTF | Estático CTF |
|---------------|----------|--------------|
| Mapa exploración | `/merged_map` | `/map` |
| Costmap global | `/merged_map` o `/robotN/map`* | `/map` |
| Metas move_base | frame `map` | frame `map` |
| Flag estimate | `/robotN/flag_detector/flag_estimate` | igual |
| cmd_vel Gazebo | `/robotN/cmd_vel` (vía coordinator) | directo |
| Estado juego | `/ctf/game_state`, `/ctf/flag_captured` | + `/flag_captured` (C++) |

\* Controlado por `plan_on_merged_map`.

---

## Qué launch usar

```bash
# Juego completo SLAM + visión + CTF (RECOMENDADO)
roslaunch ctf_navigation slam_multi_robot_ctf.launch

# Solo exploración SLAM
roslaunch ctf_navigation slam_demo.launch

# CTF con mapa estático pregenerado
roslaunch ctf_navigation ctf_game.launch

# Oráculo (planning sin visión)
roslaunch ctf_navigation ctf_demo.launch
```

---

## Prioridad de fixes pendientes

1. Unificar `flag_capture_distance` y lógica de captura C++/Python.
2. Consolidar un solo coordinador CTF para SLAM (eliminar duplicación).
3. RViz por stack (static vs SLAM).
4. Leer `/ctf_navigation/spawns` en coordinadores o eliminar el rosparam.
5. Mapa estático parametrizable por `world` en `ctf_game.launch`.
