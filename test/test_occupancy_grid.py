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
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from ohm_localization import gridmap as G
from ohm_localization import map_server_node as MS


# ------------------------------------------------------------------ the geometry of the conversion

@pytest.mark.parametrize("world", ["production", "rooms", "maze", "open"])
def _tiles_the_cells(g, hall_back) -> bool:
    """Whether `hall_back`'s rectangles cover the occupied cells of `g` and nothing else.

    Cell centres are enough: the boxes are made of cells, so a box that covers a centre covers that cell.
    """
    r = hall_back.rects
    cx = (np.arange(g.width) + 0.5) * g.resolution
    cy = (np.arange(g.height) + 0.5) * g.resolution
    inside = np.zeros((g.height, g.width), dtype=bool)
    for x0, y0, x1, y1 in r:
        inside |= ((cx >= x0) & (cx < x1)).reshape(1, -1) & ((cy >= y0) & (cy < y1)).reshape(-1, 1)
    return bool((inside == (g.data == G.OCCUPIED)).all())


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
    # Inside a wall too. `distance_to_walls` reports how deep a point sits into the box that contains it, so
    # this holds only because the read-back merges its cells into whole walls: measured over 20 000 points,
    # the deepest interior agrees to 1e-9. It is the number a likelihood field is made of — with one box per
    # cell, the same comparison gives -0.07 m where the hall says -0.93 m, and endpoints inside masonry stop
    # being a verdict. `test_merged_rectangles_tile_the_walls_exactly` is the test for that.
    inside = a < -0.3
    if inside.sum() >= 500:                   # walls thick enough to have an interior: production, maze
        assert np.abs(a[inside] - b[inside]).max() < 1e-9
    else:                                     # `rooms` and `open` are 0.5 m walls: deepest is -0.25 m
        assert a.min() < -0.2, "these halls stopped having walls to measure"
        assert np.abs(a - b).max() < 1e-9


def test_the_row_order_survives_a_hall_that_is_not_symmetric():
    """A flipped row order is invisible in a symmetric hall and fatal in an asymmetric one."""
    text = ("############\n"      # 12 columns, 8 rows, 0.5 m cells: 6.0 x 4.0 m
            "#..........#\n"
            "#..#.......#\n"      # a stub hanging from the ceiling at x 1.0 … 1.5
            "#..#.......#\n"
            "#.......#..#\n"      # and one standing at x 4.0 … 4.5, so top and bottom differ
            "#..........#\n"
            "#..........#\n"
            "############")
    hall = G.parse_grid(text, cell=0.5, name="asymmetric")
    g = G.GridMap(hall, resolution=0.5, field_resolution=0.25)
    back = G.hall_from_occupancy_grid(G.occupancy_grid(g))
    xs = np.linspace(0.5, 5.5, 40)
    ys = np.linspace(0.5, 3.5, 30)
    x, y = np.meshgrid(xs, ys)
    a = G.distance_to_walls(x, y, hall.rects)
    b = G.distance_to_walls(x, y, back.rects)
    # Exact wherever the robot can be. Measured over this 1 200-point lattice the free space agrees to
    # 1e-9 at both 0.5 m and 0.25 m raster; inside a wall the two may differ — 0.18 m at the corner of a
    # stub one cell wide — because `parse_grid` and the rastered read-back tile the same wall differently.
    outside = a > 0.05
    assert outside.sum() > 400
    assert np.abs(a[outside] - b[outside]).max() < 1e-9, "the read-back is not the same map"
    assert not ((a > 0.05) & (b < -0.05)).any() and not ((a < -0.05) & (b > 0.05)).any()

    # The asymmetry itself, at four points whose distances are read off the read-back: inside the upper
    # stub, on the floor below it, and either side of the lower stub three metres along. A row order
    # flipped on the way through the message puts a wall where the second is floor and moves the third
    # pair, so these four numbers cannot come out of a transposed map.
    d = [float(G.distance_to_walls(x, y, back.rects)) for x, y in
         ((1.75, 2.6), (1.75, 1.4), (4.25, 1.2), (4.25, 2.8))]
    assert d[0] < 0.0 < 0.3 < d[1], d
    assert d[2] < d[3] - 0.3, d
    # And the boxes tile the wall cells exactly: same cells, no gap, no overlap, merged where it was
    # mergeable — the ceiling beam is one box, not one per cell.
    assert _tiles_the_cells(g, back), "the rectangles do not tile the occupied cells"
    ceiling = [r for r in back.rects if r[1] >= 3.0]
    assert len(ceiling) == 1 and ceiling[0][2] - ceiling[0][0] == pytest.approx(6.0)


def _grid(mask, res=0.5, ox=0.0, oy=0.0):
    """A boolean wall mask in the shape `occupancy_grid()` produces, without a `GridMap` behind it."""
    height, width = mask.shape
    return {"header": {"frame_id": "map", "stamp": None},
            "info": {"resolution": res, "width": width, "height": height,
                     "origin": {"position": {"x": ox, "y": oy, "z": 0.0},
                                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}},
            "data": [100 if v else 0 for v in mask.ravel().tolist()]}


@pytest.mark.parametrize("world", ["production", "rooms", "maze", "open"])
def test_merged_rectangles_tile_the_walls_and_nothing_else(world):
    """Fewer boxes, same walls: every occupied cell covered, no cell covered twice.

    `production` at 0.25 m: 1104 occupied cells, 10 rectangles — the 10 the hall text is made of. The area
    and the exact tiling are the contract; the count is what the contract buys.
    """
    g = G.load_map(world, resolution=0.25)
    back = G.hall_from_occupancy_grid(G.occupancy_grid(g))
    cells = int((g.data == G.OCCUPIED).sum())
    r = back.rects
    assert len(r) and _tiles_the_cells(g, back)                       # no gap, nothing outside the walls
    area = float(((r[:, 2] - r[:, 0]) * (r[:, 3] - r[:, 1])).sum())
    assert area == pytest.approx(cells * 0.25 ** 2)                   # … and so no overlap: same area
    assert len(r) <= cells // 4, f"{world}: {len(r)} boxes for {cells} cells is not a merge"
    assert back.cell == pytest.approx(0.25)                           # still reports the cell it was given


def test_merging_stops_where_merging_is_impossible():
    """A staircase has no pair of cells to join; a straight wall is one box; an empty map is none."""
    staircase = np.eye(6, dtype=bool)
    assert len(G.hall_from_occupancy_grid(_grid(staircase)).rects) == 6

    straight = np.zeros((6, 9), dtype=bool)
    straight[2, 1:8] = True
    rects = G.hall_from_occupancy_grid(_grid(straight)).rects
    assert len(rects) == 1 and rects[0].tolist() == [0.5, 1.0, 4.0, 1.5]   # cells 1..7

    assert len(G.hall_from_occupancy_grid(_grid(np.zeros((5, 5), dtype=bool))).rects) == 0

    # The origin moves the merged boxes as a whole, exactly as it moved the cells before.
    shifted = G.hall_from_occupancy_grid(_grid(straight, ox=-2.0, oy=1.5)).rects
    assert shifted.tolist() == [[-1.5, 2.5, 2.0, 3.0]]


def test_a_thick_wall_reports_its_depth_after_the_round_trip():
    """The reason the merge exists: a wall tiled one box per cell has no interior to punish a bad particle.

    Point (4.13, 9.07) in `production` is 0.93 m inside a 2 m thick block. Measured against the boxes the
    read-back yields now, it is −0.93 m, the same answer the hall text gives; against one box per cell it is
    −0.07 m, which a likelihood field can barely tell from standing on the surface.
    """
    g = G.load_map("production", resolution=0.25)
    back = G.hall_from_occupancy_grid(G.occupancy_grid(g))
    x, y = np.array([4.13]), np.array([9.07])
    truth = G.distance_to_walls(x, y, g.hall.rects)[0]
    assert truth == pytest.approx(-0.93, abs=0.01)
    assert G.distance_to_walls(x, y, back.rects)[0] == pytest.approx(truth, abs=1e-9)

    cells = np.array([(col * 0.25, row * 0.25, (col + 1) * 0.25, (row + 1) * 0.25)
                      for row, col in zip(*np.nonzero(g.data == G.OCCUPIED))])
    assert G.distance_to_walls(x, y, cells)[0] == pytest.approx(-0.07, abs=0.01)


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
    assert len(G.hall_from_occupancy_grid(fields).rects) == 1     # every cell is a wall: that is one box
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

# The fields of the real messages, as of nav_msgs in ROS 2 Jazzy and later. A stub that accepted any
# attribute is how `info.layer` — a field `MapMetaData` has never had — survived here while `map_server`
# died on publish with `AttributeError` on a real machine.
MESSAGE_FIELDS = {
    "OccupancyGrid": ("header", "info", "data"),
    "MapMetaData": ("map_load_time", "resolution", "width", "height", "origin"),
    "Header": ("stamp", "frame_id"),
    "Pose": ("position", "orientation"),
    "Point": ("x", "y", "z"),
    "Quaternion": ("x", "y", "z", "w"),
}


def _stub_module(fields=None):
    """Message stubs that refuse a field the real message does not have."""
    from types import SimpleNamespace
    fields = fields or MESSAGE_FIELDS

    def make(names):
        class _Msg:
            def __setattr__(self, key, value):
                if key not in names:
                    raise AttributeError(f"no field {key!r}; this message has {names}")
                object.__setattr__(self, key, value)
        _Msg.__slots__ = tuple(names)
        return _Msg

    return SimpleNamespace(**{name: make(names) for name, names in fields.items()})


def test_the_glue_builds_a_real_nav_msgs_message():
    """The stubs above only know the field names they were handed; this one asks `nav_msgs` itself.

    Runs when a ROS 2 is sourced, skips when it is not. `map_server` published nothing at all while this
    test did not exist: it wrote `info.layer`, a field `MapMetaData` has never had, and the stub accepted it.
    """
    nav_msgs = pytest.importorskip("nav_msgs.msg", reason="needs a sourced ROS 2")
    import geometry_msgs.msg
    import std_msgs.msg
    g = G.load_map("production", resolution=0.5)
    Types = SimpleNamespace(OccupancyGrid=nav_msgs.OccupancyGrid, MapMetaData=nav_msgs.MapMetaData,
                            Header=std_msgs.msg.Header, Pose=geometry_msgs.msg.Pose,
                            Point=geometry_msgs.msg.Point, Quaternion=geometry_msgs.msg.Quaternion)
    msg = MS.build_message(G.occupancy_grid(g, frame_id="map"), Types)
    from rclpy.serialization import serialize_message
    assert len(serialize_message(msg)) > g.width * g.height, "the message did not serialize"
    assert (msg.info.width, msg.info.height) == (g.width, g.height)
    assert msg.info.resolution == pytest.approx(0.5)
    assert msg.header.frame_id == "map"
    assert len(msg.data) == g.width * g.height
    assert sum(1 for v in msg.data if v >= 50) == int((g.data == G.OCCUPIED).sum())
    assert msg.info.origin.orientation.w == 1.0


def test_the_glue_puts_every_field_where_the_message_wants_it():
    g = G.load_map("production", resolution=0.25)
    fields = G.occupancy_grid(g, frame_id="map")
    fields["header"] = dict(fields["header"], stamp="STAMP")
    m = MS.build_message(fields, _stub_module())
    assert m.header.frame_id == "map" and m.header.stamp == "STAMP"
    assert (m.info.width, m.info.height) == (g.width, g.height)
    assert m.info.resolution == pytest.approx(0.25)
    assert (m.info.origin.position.x, m.info.origin.position.y) == (0.0, 0.0)
    assert m.info.origin.orientation.w == 1.0
    assert len(m.data) == g.width * g.height
    assert m.data.count(100) == int((g.data == G.OCCUPIED).sum())
    # And the message is a round trip away from being the same walls again: the glue lost no cell, and
    # the read-back raster is the raster that went in, cell for cell.
    back = G.hall_from_occupancy_grid({"header": fields["header"], "info": fields["info"], "data": m.data})
    assert np.array_equal(G.GridMap(back).data, g.data)


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
    from geometry_msgs.msg import Point, Pose, Quaternion                    # noqa: F401
    from std_msgs.msg import Header                                          # noqa: F401
    from types import SimpleNamespace
    g = G.load_map("production", resolution=0.25)
    fields = G.occupancy_grid(g)
    m = MS.build_message(fields, SimpleNamespace(OccupancyGrid=OccupancyGrid, MapMetaData=MapMetaData,
                                                 Header=Header, Pose=Pose, Point=Point,
                                                 Quaternion=Quaternion))
    from rclpy.serialization import serialize_message
    # Writing it is the test. Assigning a wrong message type — a `Vector3` where a `Point` belongs — is
    # accepted by Python and aborts the C serializer, which is how `/map` stayed unpublished for a day.
    raw = serialize_message(m)
    assert len(raw) > len(fields["data"]), "the message did not serialize"
    back = G.hall_from_occupancy_grid(m)                    # the message itself, not the dict, reads back
    assert back.size == pytest.approx(g.hall.size)
    assert len(back.rects) == len(g.hall.rects) == 10       # merged: the hall's own rectangles, again

    # Free space, a wall face and four points inside a wall. Exact rather than within a cell, because every
    # wall of `production` sits on the 0.25 m lattice the grid quantises to — the measured maximum over these
    # twelve points is 0.0, including -0.93 m at (4.13, 9.07), which one-box-per-cell reported as -0.07 m.
    # A map from somewhere whose walls are not on the lattice cannot be exact, and is judged in free space in
    # `test_a_map_that_goes_through_the_message_comes_back_as_the_same_walls` instead.
    xs = np.array([2.0, 10.0, 15.0, 18.0, 5.6, 9.9, 12.0, 3.0, 4.13, 4.5, 14.2, 1.0])
    ys = np.array([2.0, 6.0, 1.75, 6.0, 9.0, 4.1, 11.0, 11.0, 9.07, 9.5, 9.2, 11.8])
    truth = G.distance_to_walls(xs, ys, g.hall.rects)
    assert (truth < -0.2).sum() >= 4, "these sample points stopped going inside a wall"
    assert np.abs(G.distance_to_walls(xs, ys, back.rects) - truth).max() < 1e-9
