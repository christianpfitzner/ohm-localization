"""L1–L4 template: a Monte-Carlo localiser you finish.  Two of its five methods are yours.

    ./tools/run_lab.sh grade --task mcl_production --controller student/mcl_template.py --headless
    ./tools/run_lab.sh run    --world production --task mcl_production \\
                                 --controller student/mcl_template.py --truth      # drive it yourself

**What is already here, and why.** The node lifecycle (`serve()` at the bottom, one mission per task), the
map (`load_map`, from the simulator's own hall text, with the exact clearance field), the initial cloud, the
motion model with its `rot1 / trans / rot2` decomposition and its noise **rates**, the N_eff trigger, the
systematic resampling, the estimate with its circular mean of the heading, and everything RViz reads — the two
trails, the cloud and the `TF` edge that puts the LIDAR fan on the walls instead of on the odometry
(`ros2 launch ohm_localization mcl.launch.py controller:=student/mcl_template.py` shows *your* cloud
collapsing and *your* scan where you believe the robot is, because `ohm_localization/view.py` publishes it).
Those are the parts where a mistake costs an
afternoon and teaches nothing: the rate semantics of the motion noise, for instance, cost a whole day of this
repository's development and are documented in `docs/mcl.md` — a group should not have to reinvent them
inside 180 minutes to be allowed to learn about sensor models.

**What is yours — `TODO(L1)` and nothing else.** The weight of a particle given a scan. As shipped, every
particle is equally likely, which is not a wrong filter, it is a filter that has decided the LIDAR has
nothing to say — and that is exactly the state a group should start from, because the run it produces is the
baseline of the whole exercise:

    shipped, graded by the laboratory's own grader (student/FAILURE.md; ./tools/template_check.py --record):
              mcl_production  estimate 0.68 m RMSE, odometry 0.19 m, improvement 0.27x   →  FAILS
              mcl_wide        0.68 m,  0.19 m,  0.27x                                    →  FAILS
              mcl_budget      1.26 m,  0.44 m,  0.35x                                    →  FAILS
              mcl_dirty       0.68 m,  0.19 m,  0.27x, NEES 0.12                         →  FAILS
              kf/pose at 5.3 Hz, 0 wall contacts, no crash: the plumbing works, the model does not

Three times worse than the raw odometry, not as good as it — and the reason is the first thing this exercise
should teach. A cloud that is never weighted keeps every particle, each particle's heading random-walks by
the motion model's own floor, individual paths curl, and the mean of curled paths is a shortened line. On a
10.2 m straight line, 680 steps, 0.02 rad of heading noise per step (the floor of the model given below), the
mean of 1200 such clouds ends 0.63 m short of the wall: the estimate *lags*. `N_eff` sits at 1200, resampling
never happens, nothing reports an error, and the node publishes a quietly useless pose at 5 Hz for 34 seconds.

Everything after that first weight is measurement. `tools/mcl_report.py` on a recording
(`OHM_RECORD=…`, see the README) turns a parameter question into a three-second answer, so spend the
afternoon on the four σ_z values of L4 rather than on four more minutes of staring at a loop.

Graded criteria per task, and what the reference solution measures, are in `docs/exercises.md`. Do not read
`ohm_localization/mcl.py` before your filter works — it is the reference, it is in this repository for the
viva and for the numbers in the documentation, and copying a sensor model you have not derived will not help
you answer "why does σ_z ≈ 0.5 beat σ_z = 0.25 when the sensor measures 0.25?".
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanum_lab import robot_io                                        # noqa: E402
from ohm_localization.gridmap import load_map                           # noqa: E402
from ohm_localization.hall import hall_name                             # noqa: E402
from ohm_localization.view import RosView                               # noqa: E402

# --------------------------------------------------------------------------- parameters (L4 changes these)
PARTICLES = 1200            # N
BEAM_STRIDE = 3             # use every n-th beam: the work of an update is particles × beams
SIGMA_Z = 0.15             # m, how much you believe one range measurement  ← this is L4's knob
Z_RAND = 0.04               # "the sensor is confused": a floor under the likelihood, keeps weights finite
PRIOR_SIGMA = 0.5           # m, how well the robot knows where it started (L2 says 2.0)
NOISE_RATE_XY = 0.045       # m/√s — a rate, not a per-message σ, see docs/mcl.md
NOISE_RATE_THETA = 0.09     # rad/√s
SIGMA_FLOOR = 0.01          # m/rad: no filter may claim to know better than this
RESAMPLE_BELOW = 0.5        # fraction of N: resample when N_eff drops under this
SIGMA_THETA_PRIOR = 0.6     # rad, heading uncertainty at the start
SLEEP = 5.0                 # s, longer than this between two odometry messages is not a prediction
REPORT_DT = 0.2             # s, how often kf/pose goes out (the grader wants ≥ 5 Hz)


def _pose(p) -> tuple:
    """(x, y, theta) out of whatever the bus handed over: an `Odom`, a triple, a recorded dict.

    `rob.odom()` gives an object with `.x/.y/.theta/.t`; a replay of a recording gives a dict; a bare triple
    is what a test types. Accepting all three here costs six lines and stops a type mismatch from looking
    like a filter that has diverged, which is the same reason the reference implementation has this helper.
    """
    if hasattr(p, "x"):
        return (float(p.x), float(p.y), float(p.theta))
    if isinstance(p, dict):
        return (float(p["x"]), float(p["y"]), float(p["theta"]))
    return (float(p[0]), float(p[1]), float(p[2]))


class ParticleFilter:
    """N samples of the pose, moved by the wheels and judged by the LIDAR."""

    def __init__(self, grid, n=PARTICLES, rng=None, pose=(0.0, 0.0, 0.0),
                 sigma=(PRIOR_SIGMA, PRIOR_SIGMA, SIGMA_THETA_PRIOR)):
        self.grid, self.n = grid, int(n)
        self.rng = rng or np.random.default_rng(7)
        self.x = np.column_stack([
            np.asarray(pose[:2], float) + self.rng.normal(0, sigma[:2], size=(self.n, 2)),
            self._wrap(np.asarray(pose[2], float) + self.rng.normal(0, sigma[2], size=self.n))])
        self.w = np.full(self.n, 1.0 / self.n)
        self.last_odom, self.last_neff, self.resamples = None, float(self.n), 0

    @staticmethod
    def _wrap(a):
        return (np.asarray(a, float) + np.pi) % (2.0 * np.pi) - np.pi

    # ------------------------------------------------------------------------------------ motion model
    def delta_from_odometry(self, prev, pose):
        """(rot1, trans, rot2) between two odometry poses, as on the slides."""
        px, py, pt = prev
        x, y, t = (float(pose[0]), float(pose[1]), float(pose[2]))
        rot1 = math.atan2(y - py, x - px) - pt
        trans = math.hypot(x - px, y - py)
        return self._wrap([rot1])[0], trans, self._wrap([t - pt])[0] - self._wrap([rot1])[0]

    def predict(self, rot1, trans, rot2, dt=0.05, turn=None):
        """One motion step for all particles, each with its own draw. Given, and deliberately not a constant.

        `dt` is here because the floors are *rates*: the same σ per message would make the cloud diffuse
        √(rate) faster just because a topic publishes more often (docs/mcl.md has the measurement).
        `turn` is here because rot1 comes from atan2 of the step, and a step that did not translate has no
        bearing — a robot turning on the spot would otherwise be assigned a ±π rotation every step and paid
        α·π of heading noise for it. Without `turn` the cap is off, which is right for a drive program whose
        rotations are commanded rather than inferred.
        """
        floor_t = NOISE_RATE_THETA * math.sqrt(max(dt, 0.0))
        floor_xy = NOISE_RATE_XY * math.sqrt(max(dt, 0.0))
        cap = math.inf if turn is None else abs(turn) + floor_t
        a1, a3 = min(abs(rot1), cap), min(abs(rot2), cap)
        s1 = floor_t + 0.05 * a1 + 0.05 * trans
        s2 = floor_xy + 0.05 * trans + 0.05 * (a1 + a3)
        s3 = floor_t + 0.05 * a3 + 0.05 * trans
        r1 = self._wrap(rot1 - self.rng.normal(0.0, s1, size=self.n))
        d = max(trans, 0.0) - self.rng.normal(0.0, s2, size=self.n)
        r2 = self._wrap(rot2 - self.rng.normal(0.0, s3, size=self.n))
        self.x[:, 0] += d * np.cos(self.x[:, 2] + r1)
        self.x[:, 1] += d * np.sin(self.x[:, 2] + r1)
        self.x[:, 2] = self._wrap(self.x[:, 2] + r1 + r2)

    def predict_odometry(self, pose, dt=None):
        """Move by the delta since the previous call. False for the first pose it ever sees."""
        triple = _pose(pose)
        if self.last_odom is None:
            self.last_odom = triple
            return False
        if dt is None:                                  # a message carries its own stamp; a triple does not
            t, prev_t = getattr(pose, "t", None), getattr(self, "last_odom_t", None)
            dt = (t - prev_t) if (t is not None and prev_t is not None and t > prev_t) else 0.05
        self.last_odom_t = getattr(pose, "t", getattr(self, "last_odom_t", None))
        prev = self.last_odom
        self.last_odom = triple
        self.predict(*self.delta_from_odometry(prev, triple), dt=dt,
                     turn=self._wrap([triple[2] - prev[2]])[0])
        return True

    # ------------------------------------------------------------------------------------ sensor model
    def log_weights(self, scan):
        """TODO(L1): the weight of every particle given one scan. Returns log-weights, shape (N,).

        What to compute, per particle and per beam you keep:

        * where the beam leaves the robot: bearing `angle_min + i·angle_increment + theta` **of the
          particle**, origin at the particle. A beam that keeps the robot's own heading weights every
          hypothesis as if it were the odometry, which is a filter that agrees with itself.
        * how far the wall is along that beam, from the map. You do not have to trace: `grid` answers
          `grid.field_at(x, y)` for any array of points and returns the distance to the nearest wall in
          metres (negative inside a wall, exact at the query point — it is a rasterised version of
          `grid.distance_at`, which is the same number computed from the wall rectangles). Drop a beam's
          endpoint in there and you know how well that endpoint agrees with the map.
          `ohm_localization.synth.cast()`, 20 lines, is the other school: trace the beam until it stops.
        * a Gaussian in the difference with `SIGMA_Z`, plus the `Z_RAND` floor, summed over beams — in log
          form, because with 100 beams the products underflow to zero and then one particle with a lucky
          weight decides the estimate.
        * the beams that returned `range_max` found nothing. Dropping them is defensible and is what the
          reference does; weighting them as an 8 m wall invents one. Write down which you chose, L1 asks.

        Start from the loop in the comment below if that helps you get the geometry right, then make it
        fast — `TODO(L3)` is exactly the difference between the two.

        #   idx    = np.arange(0, len(scan.ranges), BEAM_STRIDE)      # `scan` is a `Scan`: use attributes
        #   ranges = np.asarray(scan.ranges, float)[idx]              # inf = this beam found nothing
        #   angles = scan.angle_min + idx * scan.angle_increment      # body-frame bearing of each beam
        #   for k in range(self.n):                                  # 1200 × 107 of these is 1.5 s per scan
        #       x, y, th = self.x[k]
        #       … predicted range for each (x, y, th + angle) …
        #       logw[k] = sum of the per-beam log-likelihoods
        """
        return np.zeros(self.n)                       # ← the line TODO(L1) replaces

    # -------------------------------------------------------------------------------------- bookkeeping
    def update(self, scan):
        """Weights, N_eff, and a resampling only when the weights have stopped being spread out."""
        logw = self.log_weights(scan)
        logw = logw - np.max(logw)
        w = np.exp(logw)
        total = w.sum()
        if not np.isfinite(total) or total <= 0.0:
            return 0.0                                # a scan nothing explains: keep the belief as it was
        self.w = w / total
        self.last_neff = 1.0 / np.sum(self.w ** 2)    # the number from the slides, before resampling
        if self.last_neff < RESAMPLE_BELOW * self.n:
            self.resample()
        return self.last_neff

    def resample(self):
        """Systematic resampling: one uniform offset, then a deterministic sweep. Given, and worth 2×."""
        pos = (self.rng.random() + np.arange(self.n)) / self.n
        cum = np.cumsum(self.w)
        idx = np.searchsorted(cum, pos)
        self.x = self.x[np.minimum(idx, self.n - 1)]
        self.w = np.full(self.n, 1.0 / self.n)
        self.resamples += 1

    def estimate(self):
        """Weighted mean position, circular mean heading, and the 1σ the grader will check (NEES)."""
        mean = np.sum(self.x * self.w[:, None], axis=0)
        c, s = np.cos(self.x[:, 2]), np.sin(self.x[:, 2])
        theta = math.atan2(float(np.sum(self.w * s)), float(np.sum(self.w * c)))
        d = self._wrap(self.x[:, 2] - theta)
        return dict(x=float(mean[0]), y=float(mean[1]), theta=theta,
                    sx=max(math.sqrt(float(np.sum(self.w * (self.x[:, 0] - mean[0]) ** 2))), SIGMA_FLOOR),
                    sy=max(math.sqrt(float(np.sum(self.w * (self.x[:, 1] - mean[1]) ** 2))), SIGMA_FLOOR),
                    sth=max(math.sqrt(float(np.sum(self.w * d * d))), SIGMA_FLOOR),
                    neff=float(self.last_neff), resamples=self.resamples, particles=self.n)


def mission(rob, task):
    """Localise for as long as this task runs. Given, including the stamp discipline; read the comments."""
    # `hall_name` waits for /sim/world rather than asking once: the simulator and this node start in the same
    # instant, and the hall the local config defaults to is `maze` — a node that answers first localises the
    # whole drive against walls that are not in this hall, and nothing about the estimate looks wrong.
    grid = load_map(hall_name(rob, task))
    n = int(os.environ.get("OHM_MCL_PARTICLES", PARTICLES))
    sigma_prior = float(os.environ.get("OHM_MCL_PRIOR", PRIOR_SIGMA))
    print(f"mcl_template: {grid}, {n} particles, stride {BEAM_STRIDE}, sigma_z {SIGMA_Z} m, "
          f"prior ±{sigma_prior} m", file=sys.stderr)
    f, last_odom_t, last_scan_t, letzter = None, 0.0, 0.0, -1e9
    view = RosView(rob)             # /particles and /kf/path over ROS; nothing on the door that grades
    while rob.running() and rob.task() == task:
        rob.spin(0.005)
        o, scan = rob.odom(), rob.scan()
        if o is None:
            continue
        if f is None:
            f = ParticleFilter(grid, n, pose=(o.x, o.y, o.theta),
                               sigma=(sigma_prior, sigma_prior, SIGMA_THETA_PRIOR))
            f.last_odom_t, last_odom_t = o.t, o.t
        if o.t > last_odom_t:
            if o.t - last_odom_t > SLEEP:             # five seconds of dead reckoning is not a prediction
                f = ParticleFilter(grid, n, pose=(o.x, o.y, o.theta),
                                   sigma=(sigma_prior, sigma_prior, SIGMA_THETA_PRIOR))
                f.last_odom, f.last_odom_t = None, o.t
            f.predict_odometry(o)
            last_odom_t = o.t
        if scan is not None and scan.t > last_scan_t:
            last_scan_t = scan.t
            f.update(scan)
        e = f.estimate()
        view.publish(f.x, e, o.t)               # the cloud you are running, for RViz (`rviz:=true`)
        view.odometry_tf(o, e, o.t)             # and the transform that keeps your scan on the walls
        if o.t - letzter >= REPORT_DT:
            letzter = o.t
            rob.send_kf(e["x"], e["y"], e["theta"], e["sx"], e["sy"], e["sth"],
                        info={"t": round(o.t, 2), "particles": e["particles"],
                              "neff": round(e["neff"], 1), "resamples": e["resamples"],
                              "sigma_z": SIGMA_Z, "prior_sigma": sigma_prior})


if __name__ == "__main__":
    # The runner owns the bus and the lifecycle and calls mission() once per task exactly.
    robot_io.serve(sys.modules[__name__])
