"""E3 — scan matching as the odometry of the real robot.  `scan_step()` is yours; the drive is the simulator's.

    ./tools/run_lab.sh grade --only icp_odom_production --controller student/icp_odom_template.py
    python3 -m student.icp_odom_template --help                  # the offline path: a recorded drive, no robot

**What this exercise is for.** E2 registered one pair of scans on a desk. This one registers two every fifth
of a second while a robot drives, on a machine whose wheels are lying by 12 % and whose heading drifts, and it
is graded on the drive rather than on the pair. The matcher does not change — `scan_step()` is E2's `register()`
called in a loop — and everything that is new here is what a *driving* robot demands: an anchor scan to match
against, a guess to start from, an answer to reject when the fit is bad, and a σ that describes an integrated
pose instead of the last fit.

**The robot's wheels are deliberately wrong.** The task sets `odom.geometry.scale_xy = 1.12` and
`odom.bias_omega = 0.03`: every wheel tick is 12 % too long and the heading drifts while the robot drives
straight. That is not decoration and it is not a bug in the task — it is the situation in which scan matching is
worth doing at all, and the reason the grader scores your estimate *against* the raw odometry of the same drive.
Do not fix the wheels, do not calibrate them, do not fuse them in. The exercise is what the scans alone are
worth, and the number that answers that question is the improvement column.

**What is yours — `TODO(E3)`, one function.**

    scan_step(prev_scan, scan, guess, cfg) -> (T, (sx, sy, sth))
`T` carries points from the frame of `prev_scan` into the frame of `scan` — `icp.relative(pose_prev, pose_scan)`
in that order, the same convention as E2 and for the same reason. `guess` is the wheel step between the two
instants as a 3×3 transform: use it or ignore it, and be able to say which you chose and what it cost you,
because `tools/icp_eval.py --basin` measures that the basin is real and E2 measured that a wrong guess can be
worse than none. The σ is the pairwise one, from the fit; `IcpOdom.sigmas()` below is where it becomes a claim
about the pose, and that is the part the viva asks about.

**What is already here, and why each piece is not yours to write.** The bookkeeping: the anchor scan, the
period gate that keeps the published rate at 5 Hz, the `√pairs` widening of the σ, and the `kf/pose` call. Those
are the parts of a localising node that are not the algorithm and are the parts that break first on a real
robot — a node that publishes a fit from a scan pair 0.1 s apart at 20 Hz is publishing the same measurement
twice — so they are given, and they are the parts to read before you touch the loop.

**The fallback, and what it measures.** Until `scan_step()` is written, the node integrates the *wheel* step and
reports the odometry's own σ. That is not a stub: it is the baseline this exercise is graded against, and the
grader's improvement column will read 1.00 — you will pass the rate criterion and fail the accuracy ones, which
is the honest shape of a node that has plumbing and no sensor. `docs/FAILURE.md` has the measured numbers.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

# Run it from anywhere, including `python3 student/icp_odom_template.py` on its own: the module-level import
# below needs the repository root on the path, and `tools/run_lab.sh` only sets it for the graded path.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ohm_localization import icp                                            # noqa: E402

STRIDE = 2                   # one point in every two of a 360-beam scan. Not a taste: this is the value every
                               # measured number in `docs/handout/lab-e3_icp_odom.md` was produced with, and the
                               # accuracy/cost knee in docs/icp.md §6 sits in the same place
SIGMA_Z = 0.02               # the simulator's beam noise, m: the σ of one range reading
MODE = "line"                # point-to-line, for the reason in docs/icp.md §6
REPORT_DT = 0.15             # s, the shortest period between two kf/pose messages: the task asks for 5 Hz, and
                               # the gate runs on the scan clock (20 Hz), so 0.2 s lands on 4.8 Hz and fails it
MAX_CORR = 0.5               # m, the correspondence gate: the basin of attraction, see docs/icp.md §10. Given
                               # because it is a measured number and not a knob — three graded drives of the
                               # reference with only this constant changed: 1.281 m at 0.25 m, 1.065 m at 0.5 m,
                               # 1.046 m at 1.5 m. Tightening a gate costs accuracy and widening one buys nothing,
                               # because a gate is the door the correction comes through rather than a quality
                               # knob. Tuning it by feel costs one 40-second graded run per guess, so the value is
                               # here and the hour is yours for `scan_step`.


def scan_step(prev_scan, scan, guess: np.ndarray, cfg: dict) -> tuple:
    """The transform from `prev_scan`'s frame into `scan`'s, and its σ. **TODO(E3): write this.**

    Returns `(T, (sx, sy, sth))`: `T` is 3×3 and carries *points* from the previous scan's body frame into this
    scan's; the σ is the pairwise one in metres, metres and radians. `cfg` is what this node was built with —
    `mode`, `stride`, `sigma_z`, `max_corr` — the same four keys the task's `icp` block carries in
    `config/tasks_localization.json`, where `mcl_node.task_options()` is how a node reads that block if it wants
    its parameters from the task rather than from here.

    The shipped version returns the wheel step — `guess` — and the odometry's σ, which makes this node exactly
    as good as the wheels and exactly as honest: it will report a 30 mm σ for an estimate that is metres away
    from the truth, and the NEES the grader computes for that is roughly 10⁴. Replacing it is the whole
    exercise; everything else in this file already works.
    """
    return guess, (0.03, 0.03, 0.017)               # the odometry σ of one 50 ms period, for the fallback


class IcpOdom:
    """The bookkeeping around `scan_step()`: one anchor scan, one integrated pose, one honest σ."""

    def __init__(self, mode: str = MODE, stride: int = STRIDE, sigma_z: float = SIGMA_Z, step_fn=None,
                 max_corr: float = MAX_CORR):
        self.mode, self.stride, self.sigma_z = mode, stride, sigma_z
        self.cfg = {"mode": mode, "stride": stride, "sigma_z": sigma_z, "max_corr": max_corr}
        # The matcher is an argument, not a call: the node has to be testable with a step function that returns
        # a known transform (test/test_exercises.py drives it with the exact truth of a drive, which is the only
        # way to tell an integration bug from a matching bug), and solution/icp_odom_solution.py passes its own
        # guarded one. Default: this file's `scan_step`, which is the exercise.
        self.scan_step = step_fn or scan_step
        self.T, self.prev, self.prev_odom, self.prev_t = np.eye(3), None, None, None
        self.steps, self.pairs, self.rejected = [], 0, 0

    def guess_from_odometry(self, odom) -> np.ndarray:
        """What the wheels claim about the step between the two scan instants, as a 3×3 in the scan convention.

        The three-part decomposition the wheels actually report (`rot1`, `trans`, `rot2`) is rebuilt as a matrix
        product in the body frame and then inverted, because the guess has to be in the same convention as the
        answer: it is a claim about the transform between two *scans*, not about the robot's pose. This node
        never lets an odometry reading be a pose — except once, at the anchor in `step()`, and that exception is
        the whole difference between an odometry and a localiser.
        """
        if odom is None or self.prev_odom is None:
            return np.eye(3)
        a, b = self.prev_odom, odom
        rot1 = math.atan2(b.y - a.y, b.x - a.x) - a.theta
        trans = math.hypot(b.x - a.x, b.y - a.y)
        rot2 = (b.theta - a.theta) - rot1
        return icp.inverse(icp.se2(0.0, 0.0, rot1) @ icp.se2(trans, 0.0, 0.0) @ icp.se2(0.0, 0.0, rot2))

    def step(self, scan, odom=None) -> bool:
        """Match this scan against the previous one and integrate the result. True when a pair was used.

        The first scan anchors the whole run: `self.T` starts at the *odometry* pose rather than at the origin.
        Without an anchor, the answer is expressed in the frame of the first scan, which is a perfectly good
        frame and not the one `/truth` is stated in — and the grader, which is honest, grades it as the error it
        measures. Measured: the library's own ICP odometry node, run unanchored on this task, reports 5.26 m
        RMSE for a matcher whose pairwise error is 7 mm, and 3.5 m of that is this line not being here. That is
        not a bug in the node — it is what its documentation says it publishes, and the number in
        `docs/icp.md` §9 is a relative-pose number — but it is the reason a scan-matching exercise has to decide
        what it believes about t = 0, and the reason E5 needs a map.
        """
        t = float(getattr(scan, "t", 0.0) or (scan or {}).get("t", 0.0))
        if self.prev is not None and t <= self.prev_t:
            return False
        if self.prev is None and odom is not None:
            self.T = icp.pose_to_T((odom.x, odom.y, odom.theta))
        T, sigmas = self.scan_step(self.prev if self.prev is not None else scan, scan,
                                   self.guess_from_odometry(odom), self.cfg)
        self.advance(np.asarray(T, dtype=float))
        self.steps.append(tuple(sigmas))
        self.pairs += 1
        self.prev, self.prev_t, self.prev_odom = scan, t, odom
        return True

    def advance(self, T_src_to_dst: np.ndarray) -> None:
        """Integrate one pairwise transform. Separate so that a test can drive it directly.

        `T_src_to_dst` carries points from the previous scan's frame into this scan's frame, so the robot's own
        increment is its inverse. Getting that backwards does not raise: integrating the *exact* per-step
        transforms of a 36 s drive wanders 4.9 m away from the truth those transforms were made of, and the
        output still looks like a localiser (`test/test_icp_odometry.py` integrates exact truth deltas through
        this method as a control before it believes one word about ICP).
        """
        self.T = self.T @ icp.inverse(T_src_to_dst)

    def pose(self) -> tuple:
        return icp.triple_of(self.T)

    def sigmas(self) -> tuple:
        """The median pairwise σ, widened by √pairs: the only honest way to state an integrated error.

        One ICP step reports a covariance from that step's residuals, and integrating N steps multiplies the
        variance by N if the steps are independent — generous here, because the residual of a scan pair is
        largely bias rather than noise, and bias grows linearly with N, so √N still under-claims. What the
        alternative — publishing a pairwise σ as the pose σ — comes to: 3 mm for a pose that is 1.2 m away, a
        NEES of about 160 000, and a robot that will drive into a corner on the strength of it.

        The median over the pairs, not the last one: the last pair is chosen by the clock and not by the
        geometry, and a single degenerate step in a hall of flat walls reports σ_θ = 11.7° for itself while the
        run it belongs to has a median of 0.13°.
        """
        if not self.steps:
            return (1.0, 1.0, 0.5)
        s = np.median(np.array(self.steps, dtype=float), axis=0) if len(self.steps) > 1 \
            else np.array(self.steps[0], dtype=float)
        n = max(self.pairs, 1)
        return tuple(float(v) * math.sqrt(n) for v in s)


def mission(rob, task):
    """Publish the stitched pose on `kf/pose` for as long as this task runs. Given — read it, do not rewrite it."""

    from ohm_localization.mcl_node import spin_or_stop           # function-local, as in mcl_node.py

    print(f"icp_odom (student): mode {MODE}, stride {STRIDE}, σ_z {SIGMA_Z} m, gate {MAX_CORR} m — "
          f"no map is read by this node", file=sys.stderr)
    f, letzter, last_t = IcpOdom(), -1e9, None
    while rob.running() and rob.task() == task and spin_or_stop(rob, 0.005):
        o, scan = rob.odom(), rob.scan()
        if scan is None or last_t is not None and scan.t <= last_t:
            continue
        last_t = scan.t
        f.step(scan, o)
        if scan.t - letzter >= REPORT_DT:
            letzter = scan.t
            x, y, th = f.pose()
            sx, sy, sth = f.sigmas()
            rob.send_kf(x, y, th, sx, sy, sth,
                        info={"t": round(scan.t, 2), "pairs": f.pairs, "mode": f.mode, "stride": f.stride,
                              "integrated": True})


if __name__ == "__main__":
    from mecanum_lab import robot_io                             # the controller door; see mcl_node.py

    try:
        robot_io.serve(sys.modules[__name__])
    except KeyboardInterrupt:
        print("icp_odom: stopped", file=sys.stderr)
