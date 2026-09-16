"""Shared fixtures.  The suite runs with NumPy alone; the simulator only unlocks the cross-checks."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from ohm_localization import gridmap, synth                     # noqa: E402
from ohm_localization.gridmap import (GridMap, corridor_text, load_hall,   # noqa: E402
                           parse_grid)

MECANUM_LAB = gridmap.mecanum_lab_dir()
if MECANUM_LAB:
    # On the path at *collection* time, not inside the tests that need it.  Without this line the
    # cross-check tests pass or fail depending on which file pytest happened to run first, which is
    # the worst kind of green: the one that failed here (`synth.cast` against the simulator's own ray
    # caster) is the test that says whether every offline number in this repository means anything,
    # and it was passing by borrowing a `sys.path` entry from an unrelated test's import.
    sys.path.insert(0, MECANUM_LAB)
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


@pytest.fixture(scope="session")
def bare_corridor():
    """31 × 6 m of nothing: the slide along it is not measured by anything in the scan."""
    return parse_grid(corridor_text(), cell=0.5, name="bare corridor")


@pytest.fixture(scope="session")
def pillar_corridor():
    """The same corridor with a feature every 4 m — the same scan, twice, four metres apart."""
    return parse_grid(corridor_text(pillars=1), cell=0.5, name="pillar corridor")


def free_pose(hall, rng, margin=0.8):
    """A pose on free floor — random poses otherwise sit in walls.  See `synth.random_pose`."""
    return synth.random_pose(GridMap(hall), rng, margin=margin)   # clearance 0: `maze` has 1 m
                                                   # corridors, no pose there is 0.8 m off a wall
