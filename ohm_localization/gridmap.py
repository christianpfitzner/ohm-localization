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
    half a cell inside, row 0 of the text is the top edge) and `tests/test_gridmap.py` pins the two
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
one, and `tests/test_gridmap.py` pins the two against each other instead of trusting the word "fine".
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field

import numpy as np

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


def load_hall(name: str, cell: float | None = None, mec_dir: str | None = None) -> Hall:
    """The hall named `name`, from the simulator if it is there and from its text file if not.

    `cell` overrides the grid edge on the text-file path only.  On the simulator path the cell size
    comes from the simulator's config, because that is the size the walls were built from.
    """
    directory = mecanum_lab_dir(mec_dir)
    if directory and directory not in sys.path:
        sys.path.insert(0, directory)
    if directory:
        try:
            from mecanum_lab import types, worlds            # after the path is in place
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
    # No simulator: the text file, parsed under the same rules.
    for base in (directory, os.getcwd(), os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "worlds")):
        path = os.path.join(base or "", "worlds", f"{name}.txt")
        if os.path.isfile(path):
            with open(path, encoding="utf-8") as fh:
                return parse_grid(fh.read(), cell or DEFAULT_CELL, name)
    raise FileNotFoundError(
        f"World '{name}': no simulator checkout found and no worlds/{name}.txt here.  "
        "Pass --sim DIR or set MECANUM_LAB (see install.sh --check).")


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
    instead of 1.5 m: `tests/test_icp.py` checks the fixture's own clearance for that reason.
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
