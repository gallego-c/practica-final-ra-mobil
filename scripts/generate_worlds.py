#!/usr/bin/env python3
"""Generate Gazebo SDF world files for the nine-rooms and hallway arenas."""

from pathlib import Path

WALL_HEIGHT = 1.0
WALL_THICK = 0.1
WALL_Z = WALL_HEIGHT / 2.0


def wall_model(name, cx, cy, sx, sy):
    return f"""    <model name="{name}">
      <static>true</static><pose>{cx} {cy} {WALL_Z} 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>{sx} {sy} {WALL_HEIGHT}</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>{sx} {sy} {WALL_HEIGHT}</size></box></geometry></visual>
      </link>
    </model>"""


def world_header(name):
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{name}">

    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
"""


def world_footer():
    return """
  </world>
</sdf>
"""


def wall_with_door_segments(prefix, axis, fixed_coord, span_start, span_end, door_center):
    """Split a wall span into two segments around a centered 1 m door."""
    door_half = 0.5
    segments = []
    left_start, left_end = span_start, door_center - door_half
    right_start, right_end = door_center + door_half, span_end

    if left_end > left_start:
        length = left_end - left_start
        center = (left_start + left_end) / 2.0
        if axis == "h":
            segments.append((f"{prefix}_left", center, fixed_coord, length, WALL_THICK))
        else:
            segments.append((f"{prefix}_left", fixed_coord, center, WALL_THICK, length))

    if right_end > right_start:
        length = right_end - right_start
        center = (right_start + right_end) / 2.0
        if axis == "h":
            segments.append((f"{prefix}_right", center, fixed_coord, length, WALL_THICK))
        else:
            segments.append((f"{prefix}_right", fixed_coord, center, WALL_THICK, length))

    return segments


def generate_nine_rooms_world():
    """3x3 grid of 3 m rooms (9 m total), 1 m doors between adjacent rooms."""
    room_size = 3.0
    grid = 3
    total = room_size * grid  # 9 m
    half = total / 2.0  # 4.5
    walls = []

    # Exterior walls
    walls.append(("ext_north", 0.0, half, total, WALL_THICK))
    walls.append(("ext_south", 0.0, -half, total, WALL_THICK))
    walls.append(("ext_east", half, 0.0, WALL_THICK, total))
    walls.append(("ext_west", -half, 0.0, WALL_THICK, total))

    # Internal horizontal dividers between rows
    for row in range(1, grid):
        y = -half + row * room_size
        for col in range(grid):
            x0 = -half + col * room_size
            x1 = x0 + room_size
            door_x = x0 + room_size / 2.0
            prefix = f"h_div_{row}_{col}"
            walls.extend(
                wall_with_door_segments(prefix, "h", y, x0, x1, door_x)
            )

    # Internal vertical dividers between columns
    for col in range(1, grid):
        x = -half + col * room_size
        for row in range(grid):
            y0 = -half + row * room_size
            y1 = y0 + room_size
            door_y = y0 + room_size / 2.0
            prefix = f"v_div_{col}_{row}"
            walls.extend(
                wall_with_door_segments(prefix, "v", x, y0, y1, door_y)
            )

    body = [world_header("nine_rooms_world")]
    for name, cx, cy, sx, sy in walls:
        body.append(wall_model(name, cx, cy, sx, sy))
    body.append(world_footer())
    return "\n".join(body)


HALLWAY_LENGTH = 9.0
HALLWAY_WIDTH = 3.0
HALLWAY_HALF_L = HALLWAY_LENGTH / 2.0
HALLWAY_HALF_W = HALLWAY_WIDTH / 2.0


def hallway_shell():
    """Exterior walls for a 3 m x 9 m corridor centered at the origin."""
    return [
        ("ext_north", 0.0, HALLWAY_HALF_W, HALLWAY_LENGTH, WALL_THICK),
        ("ext_south", 0.0, -HALLWAY_HALF_W, HALLWAY_LENGTH, WALL_THICK),
        ("ext_west", -HALLWAY_HALF_L, 0.0, WALL_THICK, HALLWAY_WIDTH),
        ("ext_east", HALLWAY_HALF_L, 0.0, WALL_THICK, HALLWAY_WIDTH),
    ]


def build_world(name, walls):
    body = [world_header(name)]
    for wall in walls:
        body.append(wall_model(*wall))
    body.append(world_footer())
    return "\n".join(body)


def hallway_center_partitions():
    """1 m center walls at 3 m and 6 m from the west end."""
    walls = []
    for idx, dist in enumerate((3.0, 6.0), start=1):
        x = -HALLWAY_HALF_L + dist
        walls.append((f"partition_{idx}", x, 0.0, WALL_THICK, 1.0))
    return walls


def hallway_side_obstacles(marks):
    """1 m protrusions from north/south walls at the given distances from the west end."""
    walls = []
    inset = 0.5  # 1 m segment, 0.5 m inward from each side wall
    for idx, dist in enumerate(marks, start=1):
        x = -HALLWAY_HALF_L + dist
        walls.append((f"obs_{idx}_north", x, HALLWAY_HALF_W - inset, WALL_THICK, 1.0))
        walls.append((f"obs_{idx}_south", x, -HALLWAY_HALF_W + inset, WALL_THICK, 1.0))
    return walls


def generate_hallway_world():
    """3 m wide x 9 m long hallway with 1 m center walls at 3 m and 6 m."""
    walls = list(hallway_shell()) + hallway_center_partitions()
    return build_world("hallway_world", walls)


def generate_hallway_obstacles_world():
    """Hallway with center partitions plus side obstacles at 1.5 m and 7.5 m."""
    walls = (
        list(hallway_shell())
        + hallway_center_partitions()
        + hallway_side_obstacles((1.5, 7.5))
    )
    return build_world("hallway_obstacles_world", walls)


def main():
    out_dir = Path(__file__).resolve().parents[1] / "worlds"
    out_dir.mkdir(parents=True, exist_ok=True)

    worlds = {
        "nine_rooms_world.world": generate_nine_rooms_world(),
        "hallway_world.world": generate_hallway_world(),
        "hallway_obstacles_world.world": generate_hallway_obstacles_world(),
    }
    for filename, content in worlds.items():
        path = out_dir / filename
        path.write_text(content, encoding="utf-8")
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
