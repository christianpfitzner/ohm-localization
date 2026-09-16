"""A synthetic 2D LIDAR and a synthetic drive, so the algorithms can be tested without a simulator.

`tests/` and `tools/mcl_offline.py` need scans.  Asking the simulator for them means a simulator, a
clock and a robot; asking for them here means NumPy and a fixed seed, which is what lets the same
test run on a laptop, in CI, and 200 times a second.  The ray cast is therefore a real slab test
against the wall rectangles of `gridmap.py`, not a toy: the same rectangles the simulator casts its
beams against.

That is a second implementation of something that already exists, and a second implementation of a
sensor is exactly the kind of thing that quietly disagrees with the first one.  So
`tests/test_synth.py` compares this caster against `mecanum_lab.sensors.Lidar` beam by beam on every
world in the simulator, with the noise switched off on both sides, and skips nothing when the
simulator is present.  If the two ever part company, the offline tests stop being evidence.

Ranges follow the simulator's convention: a beam that hits nothing is `inf` and never a wall at
`range_max` (`Lidar.scan()` does the same, see CONTRACT §6.4).
"""
from __future__ import annotations

import math

import numpy as np

from .gridmap import Hall


def cast(pose, hall: Hall, beams: int = 360, range_max: float = 8.0,
         angle_min: float = 0.0, range_min: float = 0.05) -> np.ndarray:
    """One scan from `pose` = (x, y, theta): beam 0 forward, then counter-clockwise.

    Returns `ranges` of length `beams`, `inf` where nothing was hit within `range_max`, and a reading
    never below `range_min`.  The clamp is not decoration: `Lidar.scan()` publishes
    `max(self.range_min, hit)`, so a beam that hits something 2 cm away is reported as 5 cm — which
    the simulator only ever shows a robot that is inside a wall, but the tests sample such poses on
    purpose, and "the same measurement in both simulators" has to hold there too or the difference
    turns up as a bad beam in the middle of an unrelated test.
    """
    x, y, theta = (float(v) for v in pose3(pose))
    step = 2.0 * math.pi / beams
    a = angle_min + step * np.arange(beams) + theta
    dx, dy = np.cos(a), np.sin(a)
    t = np.full(beams, np.inf)
    for x0, y0, x1, y1 in hall.rects:
        with np.errstate(divide="ignore", invalid="ignore"):
            tx_a = (x0 - x) / dx
            tx_b = (x1 - x) / dx
            ty_a = (y0 - y) / dy
            ty_b = (y1 - y) / dy
        t_enter = np.maximum(np.minimum(tx_a, tx_b), np.minimum(ty_a, ty_b))
        t_exit = np.minimum(np.maximum(tx_a, tx_b), np.maximum(ty_a, ty_b))
        hit = (t_exit >= np.maximum(t_enter, 0.0)) & (t_exit > 0.0)
        d = np.where(t_enter > 0.0, t_enter, 0.0)     # origin inside the rectangle: it is a hit now
        t = np.where(hit & (d < t), d, t)
    return np.where(t < range_max, np.maximum(t, range_min), np.inf)


def pose3(p) -> tuple:
    """`.x/.y/.theta` or the first three of a sequence — the same tolerance the filter uses."""
    if hasattr(p, "x") and hasattr(p, "theta"):
        return p.x, p.y, p.theta
    seq = tuple(p)
    return float(seq[0]), float(seq[1]), float(seq[2] if len(seq) > 2 else 0.0)


def scan_dict(pose, hall: Hall, t: float = 0.0, beams: int = 360, range_max: float = 8.0,
              sigma: float = 0.0, rng: np.random.Generator | None = None,
              range_min: float = 0.05) -> dict:
    """A scan in the shape `Scan` has, noise included — for a test or for a log file.

    `sigma` is added per beam, and `inf` stays `inf`: a beam that found nothing is not made into a
    measurement by adding noise to it, which is the mistake the `z_short` term of a real sensor model
    exists to absorb.
    """
    ranges = cast(pose, hall, beams=beams, range_max=range_max, range_min=range_min)
    if sigma:
        rng = rng or np.random.default_rng(1)
        finite = np.isfinite(ranges)
        ranges = ranges.astype(float)
        ranges[finite] += rng.normal(0.0, sigma, size=int(finite.sum()))
    return {"t": float(t), "angle_min": 0.0, "angle_increment": 2.0 * math.pi / beams,
            "range_min": 0.05, "range_max": float(range_max), "ranges": ranges.tolist(),
            "missing": int(np.count_nonzero(~np.isfinite(ranges)))}


def drive_path(vx: float = 0.5, omega: float = 0.0, dt: float = 0.05, seconds: float = 30.0,
               start=(0.0, 0.0, 0.0), sine: float = 0.0, frequency: float = 0.1) -> np.ndarray:
    """Poses of a dead-reckoned drive as an (n, 3) array — the shape a `drive` list produces.

    A straight leg, a turn and a sine weave, which is what the graded tasks drive: a corridor of
    range readings along a wall, a rotation in the middle of a hall, and a heading that changes while
    the robot moves so that a scan pair has a rotation *and* a translation in it.
    """
    n = int(round(seconds / dt))
    out = np.zeros((n, 3))
    x, y, th = (float(v) for v in start)
    for i in range(n):
        out[i] = (x, y, th)
        w = omega + (sine * math.sin(2 * math.pi * frequency * i * dt) if sine else 0.0)
        x, y = x + vx * dt * math.cos(th), y + vx * dt * math.sin(th)
        th = (th + w * dt + math.pi) % (2 * math.pi) - math.pi
    return out


def free_path(grid, vx: float = 0.6, seconds: float = 20.0, sine: float = 0.5,
              frequency: float = 0.12, margin: float = 0.35, seed: int = 0,
              dt: float = 0.05, observable: bool = True) -> np.ndarray | None:
    """A drive that stays on the floor, and — with `observable` — one a localiser can be tested on.

    Two failure modes of a synthetic drive, both found the hard way:

    *  It walks through a wall.  Every beam then comes back at `range_min`, the point cloud is empty
       and the filter is being tested on a robot inside masonry.
    *  It runs parallel to one long wall.  A 360° LIDAR cannot tell where along a straight wall it is
       — that direction is simply not measured — so a filter driving like that scores no better than
       odometry however correct it is, and the exercise appears broken when the drive is.

    So candidates are scored by how much the distance to the walls varies along them, which is a
    cheap proxy for "the scan says something different at every point of this drive".

    A 30 s drive at 0.55 m/s is 16 m of floor with a 1.1 rad/s wiggle through it, and `rooms` is
    22 × 16 m with walls through the middle: the requested drive simply does not exist there.  So the
    length is tried as asked, then at 0.6 and 0.35 of it, and the longest that fits wins — a shorter
    drive in a small hall is a working exercise, a `None` is a broken test.  `len(path) * dt` tells the
    caller what it got.  `observable=False` takes the first route that fits, walls only excluded.
    """
    for scale in (1.0, 0.6, 0.35):
        path = _free_path_at_length(grid, vx, seconds * scale, sine, frequency, margin, seed, dt,
                                    observable)
        if path is not None:
            return path
    return None


def _free_path_at_length(grid, vx, seconds, sine, frequency, margin, seed, dt, observable):
    rng = np.random.default_rng(seed)
    best = None
    for pick in rng.permutation(len(grid.free_xy))[::13]:        # a couple of thousand starts
        x, y = grid.free_xy[pick]
        if not (margin < x < grid.hall.size[0] - margin and margin < y < grid.hall.size[1] - margin):
            continue
        for theta in rng.permutation(np.arange(0.0, 2 * math.pi, math.pi / 4)):
            path = drive_path(vx=vx, dt=dt, seconds=seconds, sine=sine, frequency=frequency,
                              start=(float(x), float(y), float(theta)))
            if grid.occupied_at(path[:, 0], path[:, 1]).any():
                continue
            if not observable:
                return path
            d = grid.distance_at(path[:, 0], path[:, 1])
            score = float(d.max() - d.min()) + 0.3 * float(np.std(d))
            if best is None or score > best[0]:
                best = (score, path)
            if score > 4.0:                                      # plenty of structure: stop looking
                return path
    return best[1] if best else None


def fake_odometry(true_path: np.ndarray, dt: float, scale: float = 1.03,
                  gyro_bias: float = 0.0, rng: np.random.Generator | None = None,
                  sigma_xy: float = 0.002, sigma_theta: float = 0.0015) -> np.ndarray:
    """Odometry built on wheels that are slightly too large and a gyro that is slightly off.

    Dead reckoning means the pose is integrated from the *sensor's own* heading, so a gyro bias
    curves the path.  The first version of this function took each true step and walked it at the
    true step's direction with a scale error on top, which produced a 4 % wheel error and absolutely
    no consequence from `gyro_bias` — 0.36 rad of heading drift over 30 s and a position error of
    13 cm, an odometry so good that no localiser on earth could beat it, and an exercise whose every
    number was therefore meaningless.  So the body velocity the wheels would have measured is taken
    out of the true path, and the integration from there on never looks at the truth again:

        v_body = R(theta_true)ᵀ · Δp / dt          what the encoders see, in the body frame
        x     += R(theta_odom) · (v_body · scale) · dt      own heading, own scale, plus noise
        theta += omega · dt + gyro_bias · dt

    `scale` and `gyro_bias` are the two knobs the exercises turn: at 1.04 and 12 mrad/s a 30 s drive
    ends more than a metre away from where the odometry claims, which is more than enough for a map
    to stop matching and no collision at all.
    """
    rng = rng or np.random.default_rng(3)
    out = np.zeros_like(true_path)
    x, y, th = (float(v) for v in true_path[0])                 # the robot starts where it is told
    out[0] = (x, y, th)
    for i in range(len(true_path) - 1):
        d = true_path[i + 1, :2] - true_path[i, :2]
        th_true = float(true_path[i, 2])
        c, s = math.cos(th_true), math.sin(th_true)
        vx = (c * d[0] + s * d[1]) / dt                          # encoders: body frame, m/s
        vy = (-s * d[0] + c * d[1]) / dt
        dth = _wrap(true_path[i + 1, 2] - true_path[i, 2])
        c2, s2 = math.cos(th), math.sin(th)                     # the ODOMETRY'S heading, not the truth
        x += scale * dt * (vx * c2 - vy * s2) + rng.normal(0.0, sigma_xy)
        y += scale * dt * (vx * s2 + vy * c2) + rng.normal(0.0, sigma_xy)
        th = _wrap(th + dth + gyro_bias * dt + rng.normal(0.0, sigma_theta))
        out[i + 1] = (x, y, th)
    return out


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi
