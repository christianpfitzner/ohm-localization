"""The known map: a mecanum-lab hall as an occupancy grid, with an exact distance field.

Why this file exists at all: a localiser is only as good as its map, and a map that is half a cell
off does not fail with an error — it fails with a filter that looks badly tuned.  So the hall is
never copied into this repository.  It is read from the simulator's own parser, which means the cell
size of the hall (`worlds.cell_by_world`, and `maze` is not 0.5 m like the others) cannot drift out
of sync with the geometry the LIDAR is ray-cast against.

Two sources, in this order:

1.  `mecanum_lab.worlds.load_world(name, cfg)` — the exact rectangles the LIDAR uses.  Available
    whenever the simulator is importable, which is every run through `./lab` and every ROS run with
    the simulator installed.
2.  `worlds/<name>.txt` read by `parse_grid()` below — for a laptop with NumPy and no simulator.
    It reproduces the simulator's conventions on purpose (origin bottom left, cell centres
    half a cell inside, row 0 of the text is the top edge) and `test/test_gridmap.py` pins the two
    sources against each other, because an unnoticed difference between them is a broken exercise
    that every group discovers at a different time.

The distance field is the exact distance from a point to the nearest wall rectangle, not a chamfer
transform of the cells.  The rectangles are known here — recomputing an approximation of them from
a rasterised grid would add a half-cell bias to every beam, and that bias is then blamed on the
particle count, which is the wrong lesson.

That exact form costs one pass over the rectangles per point, which is far too slow for a thousand
particles times a hundred beams twenty times a second.  So the field is evaluated exactly **once** on
a fine grid (`field_resolution`, 10 cm by default — a quarter of the occupancy resolution, so the
most a beam can lose to the raster is 7 cm against a sensor model that is already 15 cm wide) and
the hot loop gathers from that grid.  `distance_at()` stays the exact form, `field_at()` the fast
one, and `test/test_gridmap.py` pins the two against each other instead of trusting the word "fine".
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

import numpy as np

from ohm_localization import paths

DEFAULT_CELL = 0.5                      # m, the simulator's worlds.CELL
FREE, OCCUPIED, UNKNOWN = 0, 100, -1    # nav_msgs/msg/OccupancyGrid values


# ------------------------------------------------------------------ the hall as rectangles

@dataclass
class Hall:
    """Wall rectangles in world metres — the form the simulator's LIDAR sees the hall in."""

    name: str = "?"
    cell: float = DEFAULT_CELL
    size: tuple = (0.0, 0.0)                        # (width, height) in metres
    rects: np.ndarray = field(default_factory=lambda: np.zeros((0, 4)))   # x0, y0, x1, y1

    def __str__(self) -> str:
        return (f"Hall('{self.name}' {self.size[0]:g}x{self.size[1]:g} m, "
                f"{len(self.rects)} rectangles, cell {self.cell:g} m)")


def mecanum_lab_dir(explicit: str | None = None) -> str:
    """Where the simulator lives: argument, environment, or the sibling checkout.

    Same search order as `install.sh --check`, so the answer a student gets from `--check` is the
    answer the code uses.  Nothing here imports ROS: `mecanum_lab` is a plain Python package.
    """
    for cand in (explicit, os.environ.get("MECANUM_LAB"), os.environ.get("MECANUM_LAB_DIR")):
        if cand and os.path.isdir(os.path.join(cand, "worlds")):
            return os.path.abspath(cand)
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for cand in (os.path.join(os.path.dirname(here), "mecanum-lab"),
                 os.path.join(os.path.expanduser("~"), "git", "mecanum-lab"),
                 os.path.join(os.path.expanduser("~"), "ros2_ws", "src", "mecanum-lab")):
        if os.path.isdir(os.path.join(cand, "worlds")):
            return cand
    return ""


def mecanum_lab_root(explicit: str | None = None) -> str:
    """Where the simulator keeps its `worlds/`: the checkout if there is one, the install prefix if not.

    This is the question a colcon workspace changes.  `mecanum_lab_dir()` above answers "where is the
    checkout", which is what `install.sh --check` and `tools/drive_check.py` print; but after
    `colcon build` and `source install/setup.bash` there is no checkout on the path, `mecanum_lab` is
    importable anyway, and its hall text lives in `<prefix>/share/mecanum_lab/worlds/`.  A localiser that
    can only answer the first question reports "no such world 'production'" in exactly the setup the
    simulator's own install script recommends — which is how a path bug becomes a mystery about a hall.
    """
    return paths.mecanum_lab_root(explicit or mecanum_lab_dir(explicit))


def load_hall(name: str, cell: float | None = None, mec_dir: str | None = None) -> Hall:
    """The hall named `name`, from the simulator if it is there and from its text file if not.

    `cell` overrides the grid edge on the text-file path only.  On the simulator path the cell size
    comes from the simulator's config, because that is the size the walls were built from.

    The simulator is tried **imported as it already stands** — an installed package that the shell has
    sourced — and a checkout's directory is only added afterwards, with `paths.add_to_path`, which
    *appends*.  A `PYTHONPATH` that shadows the installed `mecanum_lab` with an older checkout turns a path
    bug into what reads like a simulator bug, and the simulator's own launch files make the same choice for
    the same reason.
    """
    directory = mecanum_lab_dir(mec_dir)
    if directory:
        paths.add_to_path(directory)
    try:
        from mecanum_lab import types, worlds            # installed already, or the checkout just added
    except Exception:
        pass
    else:
        cfg = {"worlds": types.cfg_get(types.load_config(), "worlds", {})}
        if cell is not None:
            cfg["worlds"]["cell"] = float(cell)
        w = worlds.load_world(name, cfg=cfg)
        rects = [[r.x0, r.y0, r.x1, r.y1] for r in w.walls]
        return Hall(name=w.name, cell=float(w.cell), size=tuple(w.size),
                    rects=np.asarray(rects, dtype=float).reshape(-1, 4))
    # No simulator importable: the text file, parsed under the same rules.  `mecanum_lab_root()` belongs in
    # this list because after a colcon build that `share/mecanum_lab` is the only place the hall text is;
    # `paths.data_root()` is there so that a hall dropped into this package's own `worlds/` also works, in
    # either layout.
    bases = [directory, mecanum_lab_root(mec_dir), paths.data_root(), os.getcwd(),
             os.path.dirname(os.path.dirname(os.path.abspath(__file__)))]
    for base in bases:
        path = os.path.join(base or "", "worlds", f"{name}.txt")
        if os.path.isfile(path):
            with open(path, encoding="utf-8") as fh:
                return parse_grid(fh.read(), cell or DEFAULT_CELL, name)
    raise FileNotFoundError(
        f"World '{name}': no importable mecanum_lab and no worlds/{name}.txt in "
        + ", ".join(b for b in bases if b) + ".  Set MECANUM_LAB, or build the workspace and source it "
        "(install.sh --build); the hall comes from the simulator either way — this repository keeps no copy.")


def corridor_text(cols: int = 60, rows: int = 10, pillars: int = 0) -> str:
    """A hall that is long, straight and empty, as map text: 31 × 6 m, posts every 4 m if asked.

    The simulator's halls are all too *interesting* to demonstrate what ICP is weak at.  A robot in
    `rooms` sees three walls and a doorway, so a scan match has plenty to lock on and behaves; the two
    failure modes worth teaching need geometry with nothing in them — a corridor whose ends are beyond
    the 8 m range, so the slide along it is invisible, and a corridor whose features repeat, so the
    slide is visible four times over and the match cannot tell which.  Hence a generator rather than a
    fifth hall file in somebody else's repository.

    Line 0 of the text is the top edge (`parse_grid` mirrors it), which makes the long walls the first
    and last *lines*.  Writing them as the first and last columns instead — my first attempt — gives a
    hall open at both ends with four stubs, and a clearance query at its centre then answers 12 m
    instead of 1.5 m: `test/test_icp.py` checks the fixture's own clearance for that reason.
    """
    rows = max(rows, 6)
    grid = [["#"] * (cols + 2)]
    grid += [["#"] + ["."] * cols + ["#"] for _ in range(rows - 2)]
    grid.append(["#"] * (cols + 2))
    if pillars:                                                 # a post every 8 cells = every 4 m
        for c in range(4, cols - 1, 8):
            grid[2][c] = grid[rows - 3][c] = "#"
    return "\n".join("".join(r) for r in grid)


def parse_grid(text: str, cell: float = DEFAULT_CELL, name: str = "?") -> Hall:
    """ASCII hall -> wall rectangles, following the simulator's convention exactly.

    `#` is a wall, everything else is floor (`-` and `|` are painted lines, free to drive over).
    The world origin is bottom left while line 0 of the text is the top edge, so a wall on text row
    `r` of `nrows` sits at `y = (nrows - 1 - r) * cell`.  Getting that one line wrong mirrors the
    map, and a mirrored map in a hall that is nearly symmetric looks like a tuning problem.
    """
    rows = [line for line in text.split("\n")]
    while rows and not rows[-1].strip():
        rows.pop()
    if not rows:
        raise ValueError(f"World '{name}' is empty.")
    rows = [r.ljust(max(len(x) for x in rows)) for r in rows]
    nrows, ncols = len(rows), max(len(r) for r in rows)
    walls = np.zeros((nrows, ncols), dtype=bool)
    for r, row in enumerate(rows):
        for c, ch in enumerate(row):
            if ch == "#":
                walls[r, c] = True
    y = (nrows - 1 - np.arange(nrows)) * cell          # y of the bottom edge of text row r
    x = np.arange(ncols) * cell
    rects = []
    for r in range(nrows):                             # one rectangle per wall cell, unmerged
        for c in np.flatnonzero(walls[r]):
            rects.append((x[c], y[r], x[c] + cell, y[r] + cell))
    return Hall(name=name, cell=cell, size=(ncols * cell, nrows * cell),
                rects=np.asarray(rects, dtype=float).reshape(-1, 4))


# ------------------------------------------------------------------ the grid a filter reads

class GridMap:
    """Occupancy grid plus the exact distance to the nearest wall, in world metres.

    Indexing is `data[row, col]` with `row` along +y and `col` along +x, origin at (0, 0), so
    `row = int(y / resolution)` — the layout an `OccupancyGrid` message uses (row-major, starting at
    the origin corner), which keeps the RViz picture and the array in the file the same shape.
    """

    def __init__(self, hall: Hall, resolution: float = 0.25, field_resolution: float = 0.10):
        self.hall, self.resolution = hall, float(resolution)
        self.fres = float(field_resolution)
        self.width = max(int(np.ceil(hall.size[0] / self.resolution)), 1)
        self.height = max(int(np.ceil(hall.size[1] / self.resolution)), 1)
        cx = (np.arange(self.width) + 0.5) * self.resolution
        cy = (np.arange(self.height) + 0.5) * self.resolution
        gx, gy = np.meshgrid(cx, cy)                   # (height, width), the drawn layout
        inside = np.zeros(gx.shape, dtype=bool)
        for x0, y0, x1, y1 in hall.rects:
            inside |= (gx >= x0) & (gx < x1) & (gy >= y0) & (gy < y1)
        self.data = np.where(inside, OCCUPIED, FREE).astype(np.int8)
        self.free_xy = np.column_stack([gx[~inside].ravel(), gy[~inside].ravel()])

        fw = max(int(np.ceil(hall.size[0] / self.fres)), 1)
        fh = max(int(np.ceil(hall.size[1] / self.fres)), 1)
        fx, fy = np.meshgrid((np.arange(fw) + 0.5) * self.fres,
                             (np.arange(fh) + 0.5) * self.fres)
        self.fw, self.fh = fw, fh
        self.fdist = distance_to_walls(fx, fy, hall.rects).astype(np.float32)
        self.fmax = float(max(self.fdist.max(), 1.0))

    # -- lookups used by the localiser: every one of them tolerates a point off the grid --------
    def distance_at(self, x, y):
        """Signed distance in metres from (x, y) to the nearest wall: negative inside one.

        The truth of the matter, evaluated from the rectangles, so it cannot go out of bounds and
cannot be off by half a cell.  Used by the tests and by anything that asks about a handful of
points; a hundred thousand beams an update takes `field_at()` instead.
        """
        return distance_to_walls(x, y, self.hall.rects)

    def field_at(self, x, y):
        """`distance_at()` gathered from the fine raster — the hot loop's version of the same query.

        A point off the grid gets `fmax` instead of a crash and instead of a lie: outside the hall
        nothing is known, and a particle that has drifted that far is not being asked for a
        recommendation, it is being dropped.  Nearest cell, no interpolation — interpolation costs
        about as much as the gather and the field is already four times finer than the map.
        """
        x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
        finite = np.isfinite(x) & np.isfinite(y)               # a nan in here is a bug upstream,
        x, y = np.where(finite, x, 0.0), np.where(finite, y, 0.0)   # and it is not this array's
        col = (x / self.fres).astype(np.int64)                 # business to turn into a huge index
        row = (y / self.fres).astype(np.int64)
        out = (col < 0) | (row < 0) | (col >= self.fw) | (row >= self.fh) | ~finite
        idx = np.clip(row, 0, self.fh - 1) * self.fw + np.clip(col, 0, self.fw - 1)
        return np.where(out, self.fmax, self.fdist.ravel()[idx])

    def occupied_at(self, x, y):
        """True inside a wall and off the grid — the hall is closed, so outside is not free."""
        col = (np.asarray(x) / self.resolution).astype(np.int64)
        row = (np.asarray(y) / self.resolution).astype(np.int64)
        out = (col < 0) | (row < 0) | (col >= self.width) | (row >= self.height)
        idx = np.where(out, 0, np.clip(row, 0, self.height - 1) * self.width
                       + np.clip(col, 0, self.width - 1))
        return np.where(out, True, self.data.ravel()[idx] == OCCUPIED)

    def sample_free(self, n: int, rng: np.random.Generator) -> np.ndarray:
        """`n` poses (n, 3) at free cell centres with a random heading: the global prior."""
        pick = rng.integers(0, len(self.free_xy), size=n)
        xy = self.free_xy[pick]
        return np.column_stack([xy, rng.uniform(-np.pi, np.pi, size=n)])

    @property
    def center(self) -> tuple:
        return (self.hall.size[0] / 2.0, self.hall.size[1] / 2.0)

    def __str__(self) -> str:
        free = int(np.count_nonzero(self.data == FREE))
        return (f"GridMap({self.hall.name} {self.width}x{self.height} @ {self.resolution:g} m "
                f"field @{self.fres:g} m, {free} free cells)")


def distance_to_walls(x, y, rects: np.ndarray) -> np.ndarray:
    """Signed distance from each point to the nearest rectangle: positive outside, negative inside.

    The sign is not a refinement, it is the whole sensor model.  A beam stops at the first surface it
    meets, so an endpoint 30 cm inside a wall is not evidence of "a wall somewhere near" — it is
    evidence that whoever predicted it is wrong.  Measured as a plain distance to the nearest surface,
    a point inside a rectangle is at distance 0, and with walls half a metre thick a particle could
    then stand up to half a metre *inside* masonry and score as well as the true pose: the likelihood
    went flat in exactly the direction the exercise is about, and the filter drove along the odometry
    while reporting full confidence.  Depth inside is measured to the nearest face, so it saturates at
    half the wall thickness — enough to be a verdict, not enough to be a cliff.

    The vectorised form of "closest point on an axis-aligned box" over every rectangle at once; the
    minimum over rectangles picks the deepest penetration when a point is inside several.
    """
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    if len(rects) == 0:
        return np.full(x.shape, 1e3, dtype=float)
    x0, y0, x1, y1 = (rects[:, i][((None,) * x.ndim) + (slice(None),)] for i in range(4))
    dx = np.maximum(np.maximum(x0 - x[..., None], 0.0), x[..., None] - x1)
    dy = np.maximum(np.maximum(y0 - y[..., None], 0.0), y[..., None] - y1)
    inside = (x[..., None] >= x0) & (x[..., None] < x1) & (y[..., None] >= y0) & (y[..., None] < y1)
    depth = np.minimum.reduce([x[..., None] - x0, x1 - x[..., None], y[..., None] - y0, y1 - y[..., None]])
    return np.min(np.where(inside, -depth, np.hypot(dx, dy)), axis=-1)


def load_map(world: str, resolution: float = 0.25, field_resolution: float = 0.10,
             cell: float | None = None, mec_dir: str | None = None) -> GridMap:
    """`load_hall` and `GridMap` in one call — what a node writes on startup."""
    return GridMap(load_hall(world, cell=cell, mec_dir=mec_dir), resolution=resolution,
                   field_resolution=field_resolution)


# ------------------------------------------------------------- the same map as an ROS message

def occupancy_grid(g: "GridMap", frame_id: str = "map", stamp=None) -> dict:
    """A `GridMap` as the fields of a `nav_msgs/msg/OccupancyGrid`, without importing nav_msgs.

    The reason for the plain dict in the middle is testability, and the reason for the message at all is
    RViz: the simulator publishes the walls as markers and nothing else, so a group that wants to *see* the
    map their filter is using — which is the first thing to want when a localiser disagrees with a drive —
    needs an `OccupancyGrid` on `/map`. Writing the conversion here rather than in the node means the
    round-trip test below runs on a machine with NumPy alone, and the node that wraps this dict in a real
    message is eleven lines of glue that cannot get the geometry wrong.

    The layout is the message's: `data` is row-major with row 0 at the origin corner (`info.origin`), rows
    along +x?? no — along +y, which is what `GridMap.data[row, col]` already is, so the flattening is a
    `ravel()` and not a transpose. A transpose here would be invisible in a hall that is nearly symmetric
    about its middle and catastrophic in one that is not.
    """
    data = np.ascontiguousarray(g.data, dtype=np.int8)
    return {
        "header": {"frame_id": frame_id, "stamp": stamp},
        "info": {
            "map_load_time": None,
            "resolution": float(g.resolution),
            "layer": "static",
            # origin: the corner the data starts at, which for this class is always (0, 0, 0) — the hall's
            # own world frame. Reading a map back from elsewhere shifts the other way, in `hall_from_...`.
            "origin": {"position": {"x": 0.0, "y": 0.0, "z": 0.0},
                       "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
            "width": int(g.width), "height": int(g.height),
        },
        "data": data.ravel().tolist(),
    }


def hall_from_occupancy_grid(grid, name: str = "occupancy_grid") -> "Hall":
    """The inverse: an `OccupancyGrid` (message or the dict above) back to wall rectangles.

    Accepts a real message as well, because `nav_msgs` exposes the same attribute names this dict uses as
    keys — so the only difference between the two arguments is `msg.info` versus `grid["info"]`, handled
    once here rather than in every caller.

    One rectangle per occupied cell, unmerged, which is what `parse_grid` produces too: merging runs of
    cells into longer walls would change nothing about the geometry (the union is the same), and `Hall` is
    read as a *set of boxes to measure distance to*, where fewer boxes is a speed win and nothing else.
    `-1` (unknown) is treated as *not a wall*: a map that does not know is not a map full of obstacles.
    Cells that are neither 0, 100 nor −1 (a probabilistic map from somewhere else) are walls from 50 up,
    the threshold `map_server` itself uses.

    The origin is applied, which is the one place a map from another frame can quietly arrive 3 m off:
    a map whose `origin.x` is −1.5 describes walls 1.5 m to the right of where an identity-origin map puts
    them, and a localiser on top of it will happily localise the robot 1.5 m outside the hall.
    """
    def get(obj, key, default=None):
        if isinstance(obj, dict):
            return obj.get(key, default)
        return getattr(obj, key, default)

    info = get(grid, "info")
    if info is None:
        raise ValueError("not an OccupancyGrid: no .info / 'info' to read resolution and origin from")
    res = float(get(info, "resolution", 0.0) or 0.0)
    width, height = int(get(info, "width", 0)), int(get(info, "height", 0))
    if res <= 0.0 or width <= 0 or height <= 0:
        raise ValueError(f"OccupancyGrid is unusable: resolution {res}, size {width}x{height}")
    origin = get(info, "origin") or {}
    pos = get(origin, "position") or {}
    ox, oy = float(get(pos, "x", 0.0) or 0.0), float(get(pos, "y", 0.0) or 0.0)

    flat = list(get(grid, "data") or [])
    if len(flat) != width * height:
        raise ValueError(f"OccupancyGrid data holds {len(flat)} cells, the header promises "
                         f"{width}x{height} = {width * height}. Row-major, starting at the origin "
                         "corner — a column-major map or a truncated list lands here.")
    data = np.asarray(flat, dtype=np.int16).reshape(height, width)
    occupied = data >= 50                                   # 100 is a wall, -1 is unknown, 0 is free
    rects = []
    for row, col in zip(*np.nonzero(occupied)):
        rects.append((ox + col * res, oy + row * res, ox + (col + 1) * res, oy + (row + 1) * res))
    return Hall(name=name, cell=res, size=(width * res, height * res),
                rects=np.asarray(rects, dtype=float).reshape(-1, 4))
