"""Monte Carlo Localisation: a particle filter that estimates where the robot is.

The recursion, in the order the slides write it:

    prior      x^[i] ~ prior                      uniform over the hall, or a Gaussian around a guess
    motion     x'^[i] ~ sample_motion_model_odometry(x^[i], u)      (Thrun 5.1)
    sensor     w^[i]  = p(z | x'^[i], map)                          likelihood field  (Thrun 5.4.3)
    resample   x~[i] ~ low_variance_resample(x', w)
    estimate   (x, y, theta) and its sigma from the weighted cloud

Everything is one array of shape `(n_particles, 3)` and one array of weights, and every loop over
particles is a NumPy expression.  `weights_with_a_for_loop()` at the end of this file is the same
computation written the way a first draft is written, kept on purpose: the exercise measures one
against the other and reports the factor, which is the difference between running a thousand
particles at the LIDAR's 20 Hz and running them at a third of that.

The sensor model is the **likelihood field**: a beam's endpoint is dropped into a distance map and
the closer it lands to a wall, the more the particle that predicted it is worth.  The full beam-end
model of the textbook additionally needs an expected range per beam, which needs a ray cast per
beam per particle — a second, much larger cost, for a term that matters when the map and the sensor
disagree about how far away a wall is.  Here they do not disagree, so the field is the honest choice
and not the cheap one; `z_rand` is kept because a real scan does contain beams that fit nothing.

One thing in `update()` is not a detail, and it is the thing that most often makes a working-look-
ing filter diverge: a beam that came back at the sensor's maximum range did not measure a wall.  It
says "nothing within `range_max` metres", which the likelihood field reads as "a wall far away" — so
weighting it is weighting the opposite of the truth.  The simulator hands out those beams as `inf`
(`Lidar.scan()`) or as exactly `range_max`, depending on the `lidar_no_echo` switch, so a filter that
cannot tell is being tested on a dialect, not on a map.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict

import numpy as np

TWO_PI = 2.0 * math.pi


def wrap(a):
    """Angle to (-pi, pi] — works on a scalar and on an array."""
    return (np.asarray(a) + math.pi) % TWO_PI - math.pi


@dataclass
class MclParams:
    """Every number a group is expected to tune, in one place, so a sweep is a dict and not a diff."""

    particles: int = 1200
    sigma_z: float = 0.15            # m, width of the likelihood field (Thrun's z_hit term)
    z_rand: float = 0.04             # weight of "this beam fits nothing", keeps a mode alive
    beam_stride: int = 3             # every n-th beam: 360 beams / 3 = 120, the cost knob
    drop_uninformative: bool = True  # skip beams at range_max — see the module docstring
    max_range_eps: float = 0.06      # m, how close to range_max counts as "no echo"
    min_range: float = 0.05          # m, readings below this are the robot's own housing
    # Motion model: the odometry's own error, sampled per step (Thrun's alphas, plus a floor).
    alpha1: float = 0.05             # per rad, error growing with the two rotations
    alpha2: float = 0.05             # per m,   error growing with the distance driven
    alpha3: float = 0.05             # per rad, error growing with the closing rotation
    noise_floor_xy: float = 0.01     # m,   scatter even at a standstill
    noise_floor_theta: float = 0.02   # rad, the same for the heading
    resample_below: float = 0.5      # fraction of N: N_eff under this triggers a resampling
    inject_below: float = 0.0        # fraction of N: random particles on a collapse (0 = off)
    sigma_floor: float = 0.01        # m/rad: a filter may not claim to know better than this
    seed: int = 7


class MonteCarloLocaliser:
    """One particle filter for one robot, in the world frame of the map it was built with."""

    def __init__(self, grid, params: MclParams | None = None, rng: np.random.Generator | None = None,
                 pose=(0.0, 0.0, 0.0), sigma=(0.5, 0.5, 0.35), prior: str = "gaussian"):
        """`prior`: where the robot is believed to be before the first beam.

        `"gaussian"` is localisation with a known start pose — the easy case, and the one that hides
        a broken sensor model, because the true pose is inside the initial cloud no matter how
        badly the weights are computed.  `"uniform"` is global localisation: the filter is told only
        that the robot is somewhere on the floor, which is the same computation a kidnapped robot
        needs, and it fails loudly and immediately if the weights carry no information.
        """
        self.grid = grid
        self.p = params or MclParams()
        self.rng = rng or np.random.default_rng(self.p.seed)
        self.n = int(self.p.particles)
        if prior == "uniform":
            self.x = grid.sample_free(self.n, self.rng)
        elif prior == "odom":
            self.x = np.tile(np.asarray(pose, dtype=float), (self.n, 1))
        else:
            self.x = np.column_stack([
                np.asarray(pose[:2], dtype=float) + self.rng.normal(0, sigma[:2], size=(self.n, 2)),
                wrap(np.asarray(pose[2], dtype=float)
                     + self.rng.normal(0, sigma[2], size=self.n))])
        self.w = np.full(self.n, 1.0 / self.n)
        self.logw = np.zeros(self.n)
        self.steps = self.resamples = self.injected = self.degenerate = 0
        self.last_neff = 1.0 / self.n * self.n          # = N before any weighting
        self.last_beams = 0
        self.last_odom = None

    # -------------------------------------------------------------------------------- motion model
    def delta_from_odometry(self, prev, pose) -> tuple:
        """(rot1, trans, rot2) between two odometry poses — the input of the motion model.

        Both arguments are anything with `.x`, `.y`, `.theta` (the simulator's `Odom`, a recorded
        dict, or a triple in that order), so the same filter runs against a live topic and a log.
        """
        px, py, pt = _pose(prev)
        x, y, t = _pose(pose)
        rot1 = wrap(math.atan2(y - py, x - px) - pt)
        trans = math.hypot(x - px, y - py)
        rot2 = wrap(t - pt - rot1)
        return rot1, trans, rot2

    def predict(self, rot1: float, trans: float, rot2: float) -> None:
        """One motion step for all particles: the same odometry reading, sampled differently.

        Sampling — rather than applying one clean transform to every particle — is what lets the
        cloud spread again between two scans.  With the noise set to zero the particles move as one
        rigid body, the cloud can only shrink at every resampling, and the filter walks into a
        confident wrong pose after a few seconds.  That is the failure a group has to be able to
        produce on request, which is why the noise is a parameter and not a constant.
        """
        p = self.p
        s1 = math.hypot(p.alpha1 * abs(rot1) + p.alpha2 * trans, 0.0) + p.noise_floor_theta
        s2 = p.alpha2 * trans + p.alpha1 * (abs(rot1) + abs(rot2)) + p.noise_floor_xy
        s3 = math.hypot(p.alpha3 * abs(rot2) + p.alpha2 * trans, 0.0) + p.noise_floor_theta
        r1 = wrap(rot1 - self.rng.normal(0.0, s1, size=self.n))
        d = max(trans, 0.0) - self.rng.normal(0.0, s2, size=self.n)
        r2 = wrap(rot2 - self.rng.normal(0.0, s3, size=self.n))
        self.x[:, 0] += d * np.cos(self.x[:, 2] + r1)
        self.x[:, 1] += d * np.sin(self.x[:, 2] + r1)
        self.x[:, 2] = wrap(self.x[:, 2] + r1 + r2)
        self.steps += 1

    def predict_odometry(self, pose) -> bool:
        """Move by the odometry delta since the last call.  False on the first pose it ever sees."""
        if self.last_odom is None:
            self.last_odom = _pose(pose)
            return False
        prev = self.last_odom
        self.last_odom = _pose(pose)
        self.predict(*self.delta_from_odometry(prev, pose))
        return True

    # -------------------------------------------------------------------------------- sensor model
    def neff(self) -> float:
        """Effective sample size, 1 / Σ wᵢ² — the number computed by hand twice in the lecture.

        Reported *before* resampling, which is the moment it means something: after a systematic
        resample every weight is 1/N again and the number is N by construction, so a filter that
        samples it from its own output tells you nothing about how well the scan discriminated.
        """
        return 1.0 / float(np.sum(self.w ** 2)) if len(self.w) else 0.0

    def update(self, scan) -> float:
        """Weight the cloud with one scan; returns the effective sample size N_eff.

        `scan` is the simulator's `Scan` or a recorded dict with the same four fields
        (`ranges`, `angle_increment`, `range_max`, optionally `angle_min` and `range_min`).
        """
        p = self.p
        ranges, angles, range_max, angle_min = _scan_arrays(scan)
        if ranges.size == 0:
            self.degenerate += 1
            return self.last_neff
        if not p.drop_uninformative:
            # The other dialect: `ros_bridge` publishes a missing echo as `range_max`, so a filter
            # configured for that dialect has to see the number the ROS topic would have carried —
            # not `inf`, which would send the beam endpoint to infinity and the weights with it.
            ranges = np.where(np.isfinite(ranges), ranges, range_max)
        lo = max(p.min_range, float(getattr(scan, "range_min", 0.0) or
                                    (scan.get("range_min", 0.0) if isinstance(scan, dict) else 0.0)))
        idx = np.arange(0, len(ranges), max(int(p.beam_stride), 1))
        keep = ranges[idx] > lo
        if p.drop_uninformative:
            keep &= ranges[idx] < range_max - p.max_range_eps       # the beam that saw nothing
        idx = idx[keep]
        if idx.size == 0:
            self.degenerate += 1
            return self.last_neff
        rng_, ang = ranges[idx], angle_min + idx * angles
        self.last_beams = int(idx.size)

        # Beam endpoints for every particle: (n_particles, n_beams).  One expression, no loop.
        theta = self.x[:, 2][:, None]
        ex = self.x[:, 0][:, None] + rng_[None, :] * np.cos(theta + ang[None, :])
        ey = self.x[:, 1][:, None] + rng_[None, :] * np.sin(theta + ang[None, :])
        d = self.grid.field_at(ex, ey)
        log_b = np.log(p.z_rand + (1.0 - p.z_rand) * np.exp(
            -0.5 * (d / p.sigma_z) ** 2))
        # One sum over beams, with no fudge factor on it — and the absence is a measured result.  An
        # earlier version divided this sum by a "temperature", because the filter was collapsing to
        # N_eff = 1 with a sigma forty times smaller than its error, and dividing by twelve made that
        # go away.  It was treating the symptom: the collapse came from a motion model that admitted
        # far less error than the odometry actually carries.  With `noise_floor_*` and the alphas
        # above set to what the drifting odometry really does, the cloud stays wide on its own, N_eff
        # stays in the hundreds, NEES lands near 1, and the sharpening argument about correlated beams
        # (117 beams hitting one flat wall is one measurement of that wall, repeated) no longer binds
        # anywhere in these halls.  It is worth understanding; it is not worth a knob that is switched
        # on for no measured reason, and docs/mcl.md keeps the argument.
        self.logw += log_b.sum(axis=1)
        self.logw -= self.logw.max()                  # a thousand beams, each under 1, underflow
        w = np.exp(self.logw)
        total = w.sum()
        if not np.isfinite(total) or total <= 0.0:
            self.degenerate += 1
            self.logw[:] = 0.0
            return self.last_neff
        self.w = w / total
        self.last_neff = self.neff()
        if self.p.inject_below and self.last_neff < self.p.inject_below * self.n:
            self.inject(int(0.1 * self.n))
        if self.p.resample_below and self.last_neff < self.p.resample_below * self.n:
            self.resample()
        return self.last_neff

    def weights_with_a_for_loop(self, scan) -> np.ndarray:
        """The same weights, written as two Python loops.  Only ever called by the timing tool.

        Kept in the module and not in the tool so that the comparison is against this file's own
        model: same beams, same field, same constants, one written vectorised and one not.  A
        difference of more than rounding between the two is a bug in the vectorised version, which is
        the second reason it exists.
        """
        p = self.p
        ranges, angles, range_max, angle_min = _scan_arrays(scan)
        out = np.zeros(self.n)
        for i in range(self.n):
            acc, px, py, pt = 0.0, self.x[i, 0], self.x[i, 1], self.x[i, 2]
            for j in range(0, len(ranges), max(int(p.beam_stride), 1)):
                r = ranges[j]
                if p.drop_uninformative and r >= range_max - p.max_range_eps:
                    continue
                if r <= p.min_range:
                    continue
                a = angle_min + j * angles
                d = float(self.grid.field_at(px + r * math.cos(pt + a),
                                             py + r * math.sin(pt + a)))
                acc += math.log(p.z_rand + (1.0 - p.z_rand) * math.exp(-0.5 * (d / p.sigma_z) ** 2))
            out[i] = acc
        return out

    # -------------------------------------------------------------------------------------- resample
    def resample(self, n: int | None = None) -> int:
        """Low-variance (systematic) resampling: copy the good particles, drop the dead ones.

        Systematic and not multinomial because a multinomial draw loses most of the cloud in one
        step — with 1200 particles it keeps about 63 % distinct ones — and then the filter has to
        rebuild that spread from the motion model alone.  The difference is visible as N_eff after
        one resampling and is the cheapest thing in this file to measure.
        """
        n = int(n or self.n)
        cum = np.cumsum(self.w)
        u = (self.rng.random() + np.arange(n)) / n
        idx = np.searchsorted(cum, u)
        idx = np.clip(idx, 0, self.n - 1)
        self.x = self.x[idx]
        self.w = np.full(n, 1.0 / n)
        self.logw = np.zeros(n)
        self.n = n
        self.resamples += 1
        self.last_neff = float(n)
        return n

    def inject(self, k: int) -> int:
        """Replace `k` particles by uniform draws: the recovery of a robot that was lost.

        Random injection is the answer to the question the slides ask under "recovering a lost
        robot": a filter that has lost the true pose cannot find it again by weighting, because
        nothing in the cloud is near it.  Off by default (`inject_below = 0`), and the measurement
        says that was right: on a drive where the filter already knows where it is, switching
        injection on made the RMSE in `arena` go from 0.036 m to 0.350 m — 0.1·N fresh particles every
        single update is a claim that the filter is lost, and it behaves like one.  It is a recovery
        mechanism, not a performance knob, and `docs/mcl.md` keeps the table.
        """
        k = min(int(k), self.n)
        if k <= 0:
            return 0
        self.x[-k:] = self.grid.sample_free(k, self.rng)
        self.injected += k
        return k

    # ------------------------------------------------------------------------------------- estimate
    def estimate(self) -> dict:
        """Weighted mean and spread of the cloud — the numbers that go on `/<robot>/kf/pose`.

        `theta` is a circular mean, not an arithmetic one: the average of +170 deg and -170 deg is
        0 deg, which is the opposite direction, and the error is invisible in a hall where the robot
        drives straight most of the time.  `sx`, `sy` and `sth` are the standard deviations of the
        cloud — the 1 sigma the grader measures against the actual error (NEES), which is what stops
        a wide, useless cloud from passing as a good estimate.
        """
        c, s = float(np.sum(self.w * np.cos(self.x[:, 2]))), float(np.sum(self.w * np.sin(self.x[:, 2])))
        theta = math.atan2(s, c)
        mx, my = float(np.sum(self.w * self.x[:, 0])), float(np.sum(self.w * self.x[:, 1]))
        vx = float(np.sum(self.w * (self.x[:, 0] - mx) ** 2))
        vy = float(np.sum(self.w * (self.x[:, 1] - my) ** 2))
        vth = float(np.sum(self.w * (wrap(self.x[:, 2] - theta)) ** 2))
        f = self.p.sigma_floor
        return {"x": mx, "y": my, "theta": theta,
                "sx": max(math.sqrt(vx), f), "sy": max(math.sqrt(vy), f),
                "sth": max(math.sqrt(vth), f),
                "neff": self.last_neff, "beams": self.last_beams,
                "particles": self.n, "steps": self.steps, "resamples": self.resamples,
                "injected": self.injected, "degenerate": self.degenerate}

    def spread(self) -> float:
        """Largest distance from the mean, in metres — the picture of "the cloud has not converged".

        `sx` is a standard deviation and so is small and comfortable even while the cloud still holds
        two modes, one of them wrong.  The maximum radius cannot hide that.
        """
        return float(np.max(np.hypot(self.x[:, 0] - np.sum(self.w * self.x[:, 0]),
                                     self.x[:, 1] - np.sum(self.w * self.x[:, 1]))))

    def as_dict(self) -> dict:
        return {"params": asdict(self.p), "steps": self.steps, "resamples": self.resamples,
                "injected": self.injected, "neff": self.last_neff, "estimate": self.estimate()}


# -------------------------------------------------------------------------------------------- helpers
def _pose(p) -> tuple:
    """`.x/.y/.theta` of an Odom, or the first three of a sequence, as plain floats."""
    if hasattr(p, "x") and hasattr(p, "theta"):
        return float(p.x), float(p.y), float(p.theta)
    seq = tuple(p)
    return float(seq[0]), float(seq[1]), float(seq[2] if len(seq) > 2 else 0.0)


def _scan_arrays(scan) -> tuple:
    """(ranges, angle_increment, range_max, angle_min) from a `Scan` or from a recorded dict."""
    if isinstance(scan, dict):
        get = scan.get
    else:
        get = lambda key, default=None: getattr(scan, key, default)   # noqa: E731
    ranges = np.asarray(get("ranges", []), dtype=float)
    return (ranges, float(get("angle_increment", TWO_PI / max(len(ranges), 1))),
            float(get("range_max", 8.0)), float(get("angle_min", 0.0)))
