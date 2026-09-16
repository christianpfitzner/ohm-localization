"""Shared fixtures.  The suite runs with NumPy alone; the simulator only unlocks the cross-checks."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from ohm_localization import gridmap, synth                     # noqa: E402
from ohm_localization.gridmap import GridMap, load_hall, parse_grid   # noqa: E402

MECANUM_LAB = gridmap.mecanum_lab_dir()
needs_sim = pytest.mark.skipif(not MECANUM_LAB,
                               reason="mecanum-lab checkout not found (see install.sh --check)")


@pytest.fixture(scope="session")
def hall():
    return load_hall("rooms")


@pytest.fixture(scope="session")
def grid(hall):
    from ohm_localization.gridmap import GridMap
    return GridMap(hall)


@pytest.fixture(scope="session")
def drive():
    """A drive through the hall with its drifting odometry and one scan per pose.

    `synth.free_path` picks the route, which means the route is chosen for having something in it to
    see: a drive parallel to one long wall tests a localiser on a direction that a LIDAR does not
    measure, and the numbers then look broken when only the drive is.
    """
    grid = GridMap(load_hall("rooms"))
    true = synth.free_path(grid, vx=0.55, seconds=30.0, sine=1.1, frequency=0.09, seed=3)
    if true is None:                                             # a hall this fixture cannot drive
        true = synth.drive_path(vx=0.6, seconds=20.0, start=(6.0, 8.0, 0.0), sine=0.5, frequency=0.12)
    odom = synth.fake_odometry(true, dt=0.05, scale=1.04, gyro_bias=0.012)
    return true, odom


def corridor_text(cols: int = 60, rows: int = 10, pillars: int = 0) -> str:
    """A hall that is long, straight and empty, as text: 31 × 6 m with `pillars` every 4 m.

    Two of the lessons in this exercise cannot be demonstrated in the simulator's halls because they
    are not degenerate enough: a robot in `rooms` sees three walls and a doorway, so a scan match has
    enough to lock on and behaves.  What ICP is actually weak at needs geometry with nothing in it —
    a corridor whose ends are beyond the 8 m range, and a corridor whose features repeat.

    Line 0 of the returned text is the top edge of the map (`parse_grid` mirrors it), so the long
    walls are the first and last lines.  Writing them as the first and last *columns* instead — which
    is what I did first, and which `parse_grid` then faithfully rendered — gives a hall open at both
    ends with four stubs, and a distance query at its centre answers 12 m instead of 1.5 m.  That is
    how this fixture came to check the map's own clearance in a test.
    """
    rows = max(rows, 6)
    grid = [["#"] * (cols + 2)]
    grid += [["#"] + ["."] * cols + ["#"] for _ in range(rows - 2)]
    grid.append(["#"] * (cols + 2))
    if pillars:                                                  # a post every 8 cells = every 4 m
        for c in range(4, cols - 1, 8):
            grid[2][c] = grid[rows - 3][c] = "#"
    return "\n".join("".join(r) for r in grid)


@pytest.fixture(scope="session")
def bare_corridor():
    """31 × 6 m of nothing: the slide along it is not measured by anything in the scan."""
    return parse_grid(corridor_text(), cell=0.5, name="bare corridor")


@pytest.fixture(scope="session")
def pillar_corridor():
    """The same corridor with a feature every 4 m — the same scan, twice, four metres apart."""
    return parse_grid(corridor_text(pillars=1), cell=0.5, name="pillar corridor")


def free_pose(hall, rng, margin=0.8):
    """A pose that is inside the hall and not inside a wall — random poses otherwise sit in walls."""
    from ohm_localization.gridmap import GridMap
    g = GridMap(hall, resolution=0.25)
    pick = rng.integers(0, len(g.free_xy), size=200)
    for xy in g.free_xy[pick]:
        if (margin < xy[0] < hall.size[0] - margin and margin < xy[1] < hall.size[1] - margin):
            return (float(xy[0]), float(xy[1]), float(rng.uniform(-np.pi, np.pi)))
    return None
