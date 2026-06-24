#!/usr/bin/env python3
"""Generate the static occupancy map used by the CTF Gazebo world."""

from pathlib import Path


RESOLUTION = 0.05
WIDTH = 200
HEIGHT = 200
ORIGIN_X = -5.0
ORIGIN_Y = -5.0


def world_to_pixel(x, y):
    col = int(round((x - ORIGIN_X) / RESOLUTION))
    row_from_bottom = int(round((y - ORIGIN_Y) / RESOLUTION))
    row = HEIGHT - 1 - row_from_bottom
    return col, row


def fill_rect(grid, xmin, xmax, ymin, ymax, value=0):
    c0, r0 = world_to_pixel(xmin, ymax)
    c1, r1 = world_to_pixel(xmax, ymin)
    c0 = max(0, min(WIDTH - 1, c0))
    c1 = max(0, min(WIDTH - 1, c1))
    r0 = max(0, min(HEIGHT - 1, r0))
    r1 = max(0, min(HEIGHT - 1, r1))
    for row in range(min(r0, r1), max(r0, r1) + 1):
        for col in range(min(c0, c1), max(c0, c1) + 1):
            grid[row][col] = value


def add_box(grid, cx, cy, sx, sy):
    fill_rect(grid, cx - sx / 2.0, cx + sx / 2.0, cy - sy / 2.0, cy + sy / 2.0)


def main():
    out_dir = Path(__file__).resolve().parents[1] / "maps"
    out_dir.mkdir(parents=True, exist_ok=True)

    grid = [[254 for _ in range(WIDTH)] for _ in range(HEIGHT)]

    add_box(grid, 0.0, 5.0, 10.0, 0.1)
    add_box(grid, 0.0, -5.0, 10.0, 0.1)
    add_box(grid, 5.0, 0.0, 0.1, 10.0)
    add_box(grid, -5.0, 0.0, 0.1, 10.0)

    add_box(grid, -2.75, 0.0, 3.5, 0.2)
    add_box(grid, 2.75, 0.0, 3.5, 0.2)

    add_box(grid, -3.0, -2.0, 1.5, 0.2)
    add_box(grid, -1.5, -3.5, 0.2, 1.5)
    add_box(grid, 3.0, 2.0, 1.5, 0.2)
    add_box(grid, 1.5, 3.5, 0.2, 1.5)

    add_box(grid, -1.0, 1.5, 0.3, 0.3)
    add_box(grid, 1.0, -1.5, 0.3, 0.3)

    pgm_path = out_dir / "ctf_map.pgm"
    with pgm_path.open("w", encoding="ascii") as f:
        f.write(f"P2\n{WIDTH} {HEIGHT}\n255\n")
        for row in grid:
            f.write(" ".join(str(v) for v in row))
            f.write("\n")

    yaml_path = out_dir / "ctf_map.yaml"
    yaml_path.write_text(
        "\n".join(
            [
                "image: ctf_map.pgm",
                f"resolution: {RESOLUTION}",
                f"origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="ascii",
    )

    print(f"Wrote {pgm_path}")
    print(f"Wrote {yaml_path}")


if __name__ == "__main__":
    main()
