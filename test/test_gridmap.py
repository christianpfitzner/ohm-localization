"""The map: two ways of reading one hall, and they have to agree."""
import numpy as np
import pytest

from conftest import MECANUM_LAB, needs_sim, free_pose            # noqa: F401
from ohm_localization.gridmap import GridMap, load_hall, parse_grid


def _raster(hall, resolution=0.25):
    return GridMap(hall, resolution=resolution).data


def test_simulator_and_text_file_describe_the_same_hall(hall):
    """The hall the LIDAR is cast against and the hall the map is built from, cell for cell.

    The one difference between `load_hall` and `parse_grid` is that the simulator merges wall cells
    into as few rectangles as it can and this file emits one rectangle per wall cell.  The union is
    the same, so the rasterised map must be identical — and if a merge ever eats a cell, that shows
    here rather than as a filter that will not converge.
    """
    import os
    path = os.path.join(MECANUM_LAB or ".", "worlds", f"{hall.name}.txt")
    if not os.path.isfile(path):
        pytest.skip("no worlds/ directory to read")
    with open(path, encoding="utf-8") as fh:
        text = parse_grid(fh.read(), cell=hall.cell, name=hall.name)
    assert np.array_equal(_raster(hall), _raster(text)), \
        f"{hall.name}: simulator and text parser disagree on " \
        f"{int((_raster(hall) != _raster(text)).sum())} cells"


@needs_sim
def test_maze_is_not_half_a_metre_wide():
    """`maze` is built on a 1 m grid while every other hall is on 0.5 m — from the simulator's config.

    A map that assumed the default cell size here would be exactly half the size of the hall the
    beams are flying through, which is the kind of mistake that reads as a badly tuned filter.
    """
    assert load_hall("maze").cell == pytest.approx(1.0)
    assert load_hall("rooms").cell == pytest.approx(0.5)


def test_distance_field_is_the_exact_distance_to_the_walls(hall):
    """The fine raster the hot loop gathers from is within half a field cell of the rectangles."""
    g = GridMap(hall)
    rng = np.random.default_rng(11)
    pts = np.asarray([free_pose(hall, rng) for _ in range(50)])
    x, y = pts[:, 0], pts[:, 1]
    exact, field = g.distance_at(x, y), g.field_at(x, y)
    assert np.max(np.abs(exact - field)) <= g.fres * 0.75 + 1e-6
    inside = g.occupied_at(x, y)
    assert np.all(g.distance_at(x[~inside], y[~inside]) > 0.0)


def test_off_the_map_is_not_free_space(hall):
    """Outside the hall nothing is known, so it must not read as a floor a particle can sit on."""
    g = GridMap(hall)
    assert bool(g.occupied_at(-1.0, 4.0)) and bool(g.occupied_at(g.hall.size[0] + 5.0, 4.0))
    assert float(g.field_at(-1.0, 4.0)) == g.fmax


def test_free_cells_are_really_free(hall):
    g = GridMap(hall)
    assert len(g.free_xy) > 100
    assert not np.any(g.occupied_at(g.free_xy[:, 0], g.free_xy[:, 1]))
