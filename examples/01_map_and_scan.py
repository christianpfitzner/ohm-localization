#!/usr/bin/env python3
"""A hall, an occupancy grid and one LIDAR scan — numpy only, no ROS, no simulator.

    python3 examples/01_map_and_scan.py

`#` is a wall, `.` is floor. `parse_grid` turns that text into wall rectangles, `GridMap` into a
0.25 m occupancy grid with a distance field, and `synth.scan_dict` rays one scan through the same
rectangles the grid was built from — which is why the two cannot disagree.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                            # noqa: E402

from ohm_localization import synth                            # noqa: E402
from ohm_localization.gridmap import FREE, GridMap, parse_grid        # noqa: E402

HALL = """\
########################
#......................#
#......................#
#....####..............#
#....####......###.....#
#......................#
#..........###.........#
#......................#
########################
"""

POSE = (5.0, 3.0, 0.0)         # x, y, theta of the robot that does the looking: m and rad
CELL = 0.5                     # m, the size of one character of the map text


def endpoints(pose, scan):
    """Where every beam that echoed hit something, in hall coordinates."""
    beams = np.arange(len(scan["ranges"]))
    a = scan["angle_min"] + beams * scan["angle_increment"]
    r = np.asarray(scan["ranges"], dtype=float)
    hit = np.isfinite(r)
    return pose[0] + r[hit] * np.cos(a[hit]), pose[1] + r[hit] * np.sin(a[hit])


hall = parse_grid(HALL, cell=CELL, name="example")
grid = GridMap(hall)
scan = synth.scan_dict(POSE, hall, t=0.0, beams=180, range_max=20.0)

print(hall)
print(grid)
ranges = np.asarray(scan["ranges"], dtype=float)
echoed = int(np.count_nonzero(np.isfinite(ranges)))
print(f"scan: {len(ranges)} beams, {echoed} echoed, nearest echo "
      f"{float(np.min(ranges[np.isfinite(ranges)])):.2f} m")

hx, hy = endpoints(POSE, scan)
picture = [["." if free == FREE else "#" for free in row] for row in
           grid.data[::-2, ::2]]                   # row 0 of the print is the far wall; every other
                                                   # row and column, so it fits a terminal
for x, y in zip(hx, hy):
    picture[int((grid.height - 1 - y / grid.resolution) / 2)][int(x / grid.resolution / 2)] = "*"
picture[int((grid.height - 1 - POSE[1] / grid.resolution) / 2)][int(POSE[0] / grid.resolution / 2)] = "o"
print("\n".join("".join(row) for row in picture))
print(f"the robot stands {float(grid.distance_at(*POSE[:2])):.2f} m from the nearest wall, "
      f"{grid.hall.size[0]:.1f} × {grid.hall.size[1]:.1f} m hall")
