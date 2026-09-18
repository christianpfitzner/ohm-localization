"""E4 solved: the particle lifecycle in the five functions the criteria measure.

    python3 tools/lab_check.py e4_particles --module solution/particles_solution.py
    python3 tools/lab_check.py e4_particles --module student/particles_template.py     # five TODOs, seven FAILs

The shortest implementation that is honest about being a sampler, with the three places a filter dies
commented where they are: the `√dt` in `move_particles`, the one uniform draw in `resample`, and the circular
mean in `weighted_estimate`. Every number the criteria quote as measured comes from this file running under
`tools/lab_check.py --verbose`, so a threshold can be re-checked by running it rather than by believing the
sentence in the JSON.
"""
from __future__ import annotations

import math

import numpy as np

SIGMA_FLOOR = 1e-4      # metres and radians: a cloud of one duplicate particle has σ 0, which is not a claim


def sample_uniform(grid, n: int, rng: np.random.Generator) -> np.ndarray:
    """Rejection sampling on the distance field: uniform over the hall, and nothing inside a wall.

    Rejection rather than `GridMap.sample_free()`'s cell list because it is the same statement in one line —
    a pose is kept if the field says it is clear — and because the acceptance rate is a number worth knowing:
    in `production` a little over half of the hall's bounding box is free, so this loop is short. The heading
    is drawn over the whole circle, which for a uniform prior is the only statement that does not prefer a
    direction; a prior that prefers one is a prior that has already seen a scan.
    """
    sx, sy = grid.hall.size
    out = np.empty((0, 3))
    while len(out) < n:
        want = max(4 * (n - len(out) + 1), 64)
        cand = np.column_stack([rng.uniform(0.0, sx, want), rng.uniform(0.0, sy, want),
                                rng.uniform(-math.pi, math.pi, want)])
        clear = grid.field_at(cand[:, 0], cand[:, 1]) > 0.20        # 200 mm from any wall: a robot is a disc
        out = np.vstack([out, cand[clear]])
    return out[:n]


def sample_gaussian(center, sigma, n: int, rng: np.random.Generator) -> np.ndarray:
    """`center + 𝒩(0, sigma)`, with the heading wrapped back into (−π, π]."""
    c, s = np.asarray(center, float), np.asarray(sigma, float)
    x = c + rng.normal(0.0, 1.0, size=(n, 3)) * s
    x[:, 2] = ((x[:, 2] + math.pi) % (2.0 * math.pi)) - math.pi     # not a clip: 3.2 rad is −3.08 rad
    return x


def move_particles(x: np.ndarray, rot1: float, trans: float, rot2: float, dt: float, p,
                   rng: np.random.Generator) -> np.ndarray:
    """The three-part odometry step, with every particle drawing its own version of it.

    The three σ are the same three the filter uses (`mcl.py:predict()`), so this function drops into the localiser
    instead of being a toy version of it: `s1` and `s3` for the two rotations, `s2` for the distance, each one an
    alpha part proportional to the motion in the step and a floor proportional to √dt. Two things to notice in it,
    both of which the criteria measure:

    The floor is a rate — `noise_rate_xy·√dt`, `noise_rate_theta·√dt` — because it is what a *stationary* robot's
    wheels still get wrong, and a per-step σ for it would silently mean "per message". The alphas need no such
    factor: they are proportional to the motion itself, so a step that moves 1 cm contributes 1 cm of spread
    whether it arrives every 20 ms or every 200 ms. `noise-floors-are-rates` measures the first statement with the
    alphas off, and `the-cloud-spreads` measures that both are doing something.

    `s2` carries `α₁·(|rot1| + |rot2|)`: turning makes the *distance* uncertain, because the two rotations are
    estimated from the same wheel ticks as the translation and a mecanum wheel that slips in heading slips in
    distance. It looks like a typo in somebody's Jacobian and it is the production model's coupling.
    """
    x = np.asarray(x, dtype=float)
    root = math.sqrt(max(float(dt), 0.0))
    floor_t, floor_xy = float(p.noise_rate_theta) * root, float(p.noise_rate_xy) * root
    n = len(x)
    s1 = abs(float(p.alpha1) * rot1 + float(p.alpha2) * trans) + floor_t
    s2 = (float(p.alpha2) * abs(trans) + float(p.alpha1) * (abs(rot1) + abs(rot2)) + floor_xy)
    s3 = abs(float(p.alpha3) * rot2 + float(p.alpha2) * trans) + floor_t
    r1 = rot1 + rng.normal(0.0, s1, n)                                 # the turn first ...
    d = trans + rng.normal(0.0, s2, n)                                 # ... then the straight line ...
    r2 = rot2 + rng.normal(0.0, s3, n)                                 # ... then the rest of the turn
    out = np.empty_like(x)
    out[:, 0] = x[:, 0] + d * np.cos(x[:, 2] + r1)
    out[:, 1] = x[:, 1] + d * np.sin(x[:, 2] + r1)
    out[:, 2] = ((x[:, 2] + r1 + r2 + math.pi) % (2.0 * math.pi)) - math.pi
    return out


def resample(x: np.ndarray, w: np.ndarray, n: int, rng: np.random.Generator) -> np.ndarray:
    """Systematic resampling: one uniform draw, `n` evenly spaced pointers, the cumulative weights once.

    The single draw is the low variance: `rng.choice` would draw n times independently and put twice as much
    noise into which modes survive, which at 250 particles is the difference between a filter and a lottery.
    Zero-weight particles are unreachable by construction, because the cumulative sum never advances at them.
    """
    w = np.asarray(w, dtype=float)
    w = np.where(np.isfinite(w) & (w > 0.0), w, 0.0)
    total = float(w.sum())
    if total <= 0.0:
        pick = rng.integers(0, len(x), size=n)                      # a flat-likelihood cloud: draw uniformly
        return np.array(x, dtype=float, copy=True)[pick]
    cum = np.cumsum(w) / total
    u0 = float(rng.random()) / n
    slots = (u0 + np.arange(n, dtype=float)) / n
    return np.array(x, dtype=float, copy=True)[np.searchsorted(cum, slots, side="left").clip(0, len(x) - 1)]


def weighted_estimate(x: np.ndarray, w: np.ndarray) -> dict:
    """The weighted mean of the cloud and its spread, with the heading averaged as a direction.

    `Σw·θ` of a cloud straddling ±π means the opposite of what the cloud means; the direction average below is
    the only version that survives a filter wrapping itself around the hall. The circular standard deviation of
    the same directions is the σ of the heading — and it is 0 for a cloud of one pose, which is what the floor
    in the module header is for: a σ of exactly 0 is a claim no sensor supports, and a grader divides by it.
    """
    x = np.asarray(x, dtype=float)
    w = np.asarray(w, dtype=float)
    w = np.where(np.isfinite(w) & (w > 0.0), w, 0.0)
    total = float(w.sum())
    if total <= 0.0:
        w, total = np.full(len(x), 1.0 / max(len(x), 1)), float(len(x))
    p = w / total
    mx, my = float((p * x[:, 0]).sum()), float((p * x[:, 1]).sum())
    c, s = float((p * np.cos(x[:, 2])).sum()), float((p * np.sin(x[:, 2])).sum())
    theta = math.atan2(s, c)
    R = math.hypot(c, s)                                    # the mean resultant length: 1 is one direction
    sx = math.sqrt(max(float((p * (x[:, 0] - mx) ** 2).sum()), 0.0))
    sy = math.sqrt(max(float((p * (x[:, 1] - my) ** 2).sum()), 0.0))
    sth = math.sqrt(max(-2.0 * math.log(max(R, 1e-12)), 0.0))
    return {"x": mx, "y": my, "theta": theta,
            "sx": max(sx, SIGMA_FLOOR), "sy": max(sy, SIGMA_FLOOR), "sth": max(sth, SIGMA_FLOOR)}
