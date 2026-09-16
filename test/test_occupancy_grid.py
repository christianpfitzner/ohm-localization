"""The map as an ROS message: geometry through the round trip, glue through stubs.

`nav_msgs` is not installed on the machine these numbers were produced on, and the tests below are written so
that this costs nothing: the conversion is a dict in the middle (`gridmap.occupancy_grid`,
`gridmap.hall_from_occupancy_grid`), so the parts that can be wrong — cell order, origin, which value means
wall — are all tested without ROS, and the eleven lines of message glue are tested against stub classes with
the same attribute names. One test at the end uses the real message when it exists and says why it skipped
when it does not.

What a map conversion is actually vulnerable to is a mistake that *looks fine*: a transpose, or a row order
flipped, gives a picture of the hall that a nearly symmetric hall renders identically, and a localiser on top
of the mirrored map converges — on the mirror image, tens of centimetres from where the robot is. That is a
worse failure than a crash, and it is the reason the first test below checks the y direction with an
asymmetric hall rather than trusting the shape of the picture.
"""
import math
import sys

import numpy as np
import pytest

from ohm_localization import gridmap as G
from ohm_localization import map_server_node as MS


# ------------------------------------------------------------------ the geometry of the conversion

@pytest.mark.parametrize("world", ["production", "rooms", "maze", "open"])
def test_a_map_that_goes_through_the_message_comes_back_as_the_same_walls(world):
    """Round trip on every hall of this exercise, judged where the geometry is defined."""
    g = G.load_map(world, resolution=0.25)
    back = G.hall_from_occupancy_grid(G.occupancy_grid(g))
    assert back.size == pytest.approx(g.hall.size)
    # 20 000 points, because `maze` has 1 m corridors and 0.5 m walls and only a sixth of its area is more
    # than 30 cm from a wall; a 4 000-point sample leaves too few free points there to say anything.
    rng = np.random.default_rng(1)
    xs = rng.uniform(0, g.hall.size[0], 20000)
    ys = rng.uniform(0, g.hall.size[1], 20000)
    a = G.distance_to_walls(xs, ys, g.hall.rects)
    b = G.distance_to_walls(xs, ys, back.rects)
    free = a > 0.3                              # outside the walls, away from a face
    assert free.sum() > 2000
    # Exact, not approximately: every wall of every hall in this exercise sits on the 0.25 m grid, so the
    # cells of the raster tile the rectangles with nothing left over.
    assert np.abs(a[free] - b[free]).max() < 1e-9
    # What must survive is the *shape* of the map: a point that is well outside a wall is well outside a wall.
    # A transpose or a flipped row order leaves the numbers above looking plausible in a symmetric hall and
    # destroys this test in any hall.
    assert not ((a > 0.3) & (b < -0.3)).any() and not ((a < -0.3) & (b > 0.3)).any()
    # Inside a wall the two do not agree, and the reason is worth writing down rather than asserting away:
    # `distance_to_walls` reports how deep a point sits *into the box that contains it*, which in `production`
    # is up to 1.0 m for its 3.5 x 2 m block and 12 cm for the 0.25 m cells that tile the same block. In a
    # hall whose walls are one grid cell thick (`rooms`, `maze`, `open`) the difference is 0.25 m at most.
    # Free space is the same map either way; inside a wall it is a different question, and a localiser has no
    # business answering it — which is why the filter's likelihood field saturates instead of trusting depth.
    deepest_hall, deepest_back = a.min(), b.min()
    assert deepest_back >= -0.5 * g.resolution - 1e-9        # measured: -0.123 … -0.125 m at 0.25 m cells
    assert deepest_hall < deepest_back                       # the hall's own walls are thicker than a cell


def test_the_row_order_survives_a_hall_that_is_not_symmetric():
    """A flipped row order is invisible in a symmetric hall and fatal in an asymmetric one."""
    text = G.corridor_text(cols=40, rows=12)
    hall = G.parse_grid(text, cell=0.5, name="corridor")
    g = G.GridMap(hall, resolution=0.5, field_resolution=0.25)
    fields = G.occupancy_grid(g)
    back = G.hall_from_occupancy_grid(fields)
    xs = np.linspace(0.5, 19.5, 60)
    ys = np.linspace(0.5, 5.5, 20)
    x, y = np.meshgrid(xs, ys)
    a = G.distance_to_walls(x, y, hall.rects)
    b = G.distance_to_walls(x, y, back.rects)
    both = (a > 0.25) & (b > 0.25)
    assert both.sum() > 200
    assert np.abs(a[both] - b[both]).max() < 1e-9
    # And the specific asymmetry: a ceiling beam in the top rows must not come back as a floor beam.
    top = [r for r in back.rects if r[3] > 5.0 and 4.0 < r[0] < 16.0]
    truth_top = [r for r in hall.rects if r[3] > 5.0 and 4.0 < r[0] < 16.0]
    assert len(top) == len(truth_top) > 0


def test_a_map_from_elsewhere_is_moved_not_reinterpreted():
    """`info.origin` is where the data starts. Ignoring it is a 3-metre-off map that looks plausible."""
    g = G.load_map("production", resolution=0.5)
    fields = G.occupancy_grid(g)
    fields["info"]["origin"]["position"]["x"] = -2.0
    fields["info"]["origin"]["position"]["y"] = 1.5
    back = G.hall_from_occupancy_grid(fields, name="shifted")
    assert back.rects[:, [0, 2]].min() == pytest.approx(-2.0)
    assert back.rects[:, [1, 3]].min() == pytest.approx(1.5)
    assert back.size == pytest.approx(g.hall.size)          # size is size; the origin shifts, the hall does not


def test_unknown_is_not_a_wall_and_probabilities_have_a_threshold():
    g = G.load_map("production", resolution=0.5)
    fields = G.occupancy_grid(g)
    n = len(fields["data"])
    assert all(v in (0, 100) for v in fields["data"])       # a hall this exercise builds knows everything
    fields["data"] = [-1] * n                                # a map that knows nothing
    assert len(G.hall_from_occupancy_grid(fields).rects) == 0
    fields["data"] = [60] * n                                # a probabilistic map from somewhere else
    assert len(G.hall_from_occupancy_grid(fields).rects) == n
    fields["data"] = [40] * n                                # below `map_server`'s 50: not a wall
    assert len(G.hall_from_occupancy_grid(fields).rects) == 0


def test_a_truncated_or_headless_grid_is_refused_not_guessed():
    g = G.load_map("production", resolution=0.5)
    fields = G.occupancy_grid(g)
    fields["data"] = fields["data"][:10]
    with pytest.raises(ValueError, match="[Rr]ow-major"):
        G.hall_from_occupancy_grid(fields)
    with pytest.raises(ValueError, match="resolution"):
        G.hall_from_occupancy_grid({"info": {"resolution": 0, "width": 4, "height": 4}, "data": [0, 0, 0, 0]})
    with pytest.raises(ValueError, match="info"):
        G.hall_from_occupancy_grid({"data": [0]})


# ------------------------------------------------------------------- the message glue, on stubs

class _Bag:
    """An object that takes attributes, which is all a ROS message is from here."""

    def __setattr__(self, k, v):
        object.__setattr__(self, k, v)


def _stub_module():
    from types import SimpleNamespace
    return SimpleNamespace(OccupancyGrid=_Bag, MapMetaData=_Bag, Header=_Bag, Pose=_Bag, Vector3=_Bag,
                           Quaternion=_Bag, Time=object)


def test_the_glue_puts_every_field_where_the_message_wants_it():
    g = G.load_map("production", resolution=0.25)
    fields = G.occupancy_grid(g, frame_id="map")
    fields["header"] = dict(fields["header"], stamp="STAMP")
    m = MS.build_message(fields, _stub_module())
    assert m.header.frame_id == "map" and m.header.stamp == "STAMP"
    assert (m.info.width, m.info.height) == (g.width, g.height)
    assert m.info.resolution == pytest.approx(0.25)
    assert m.info.layer == "static"
    assert (m.info.origin.position.x, m.info.origin.position.y) == (0.0, 0.0)
    assert m.info.origin.orientation.w == 1.0
    assert len(m.data) == g.width * g.height
    assert m.data.count(100) == int((g.data == G.OCCUPIED).sum())
    # And the message is a round trip away from being the hall again — the glue did not lose a cell.
    back = G.hall_from_occupancy_grid({"header": fields["header"], "info": fields["info"], "data": m.data})
    assert len(back.rects) == len(m.data) - m.data.count(0) - m.data.count(-1)


def test_describe_says_what_would_be_published():
    g = G.load_map("production", resolution=0.25)
    line = MS.describe(G.occupancy_grid(g))
    assert "80x48 cells at 0.25 m" in line
    assert "20.00x12.00 m" in line
    assert f"{int((g.data == G.OCCUPIED).sum())} occupied" in line
    assert "0 unknown" in line


def test_dry_run_needs_no_ros_and_names_the_hall(capsys):
    assert MS.main(["--world", "production", "--resolution", "0.5", "--dry-run"]) == 0
    err = capsys.readouterr().err
    assert "production" in err and "40x24 cells at 0.5 m" in err


def test_without_the_message_packages_the_node_says_so_instead_of_tracebacking(capsys):
    """On a machine with no ROS: exit 3 and a sentence, not an ImportError from a launch file."""
    if "nav_msgs" in sys.modules:
        pytest.skip("this machine has nav_msgs; the guarded path is the one that is not taken here")
    assert MS.main(["--world", "production", "--topic", "/map"]) == 3
    err = capsys.readouterr().err
    assert "nav_msgs" in err and "--dry-run" in err


def test_a_real_message_carries_the_same_map():
    """The only test here that needs a sourced workspace; it skips, with a reason, when there is none."""
    pytest.importorskip("nav_msgs", reason="nav_msgs is not installed in this sandbox")
    from nav_msgs.msg import MapMetaData, OccupancyGrid                      # noqa: F401
    from geometry_msgs.msg import Pose, Quaternion, Vector3                  # noqa: F401
    from std_msgs.msg import Header                                          # noqa: F401
    from types import SimpleNamespace
    g = G.load_map("production", resolution=0.25)
    fields = G.occupancy_grid(g)
    m = MS.build_message(fields, SimpleNamespace(OccupancyGrid=OccupancyGrid, MapMetaData=MapMetaData,
                                                 Header=Header, Pose=Pose, Vector3=Vector3,
                                                 Quaternion=Quaternion))
    back = G.hall_from_occupancy_grid(m)                    # the message itself, not the dict, reads back
    assert back.size == pytest.approx(g.hall.size)
    xs = np.array([4.0, 10.0, 15.0])
    ys = np.array([9.0, 6.0, 1.75])
    assert np.abs(G.distance_to_walls(xs, ys, back.rects)
                  - G.distance_to_walls(xs, ys, g.hall.rects)).max() < 0.3
