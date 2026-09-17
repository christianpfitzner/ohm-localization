"""Localise by stitching scans together (ICP odometry) — no map, no prior, no particles.

    ros2 run ohm_localization icp_odom_node --robot alice
    python3 -m ohm_localization.icp_odom_node --robot alice
    ./tools/run_lab.sh grade --task mcl_production --controller <this file as a controller>   # see below

This is the other half of the lesson, and the one that cannot be taught by the map-based filter: MCL is told
where it is because the map says so, ICP is told only how it moved since the last scan. Run both against the
same drive and the difference is visible without any theory — the particle filter's error stays bounded
because every scan is checked against something that does not move, and this node's error grows, because
every scan is checked against one that has moved.

What it costs, measured twice — on a recording of the graded L1 drive (727 scans, 36 s) and on synthetic
scans of the same drive shape (400 steps, 20 s, in `test/test_icp_odometry.py`):

    one pairwise step                    7 mm and 0.04° of heading   (better than the wheel odometry)
    integrated, point-to-line            1.76 m (recording) · 0.91 m (synthetic)
    integrated, point-to-point           1.0–1.2 m (recording) · 0.78 m (synthetic)
    the same drive, wheel odometry       0.18 m (recording) · 0.07 m (synthetic)
    the same drive, the map-based L1 node   15 mm

So a method that beats the wheels on every single step loses to them over a drive, by a factor of ten, and
loses to a particle filter with a map by two orders of magnitude. The reason is in `docs/icp.md` §7: the
residual is not noise, it is a bias that belongs to the point-to-line objective — −0.051°/step on the
recording, −0.099°/step on the synthetic drive, summed to −37° and −39° of heading — and a bias added N times
grows linearly where noise would have grown as √N. "Our ICP achieves 6 mm" is a sentence that has to be asked
"over what horizon?" before it means anything; this node is how to answer it about your own drive.

Parameters are the same names the library uses (`OHM_ICP_MODE`, `OHM_ICP_STRIDE`, `OHM_ICP_SIGMA_Z`,
`OHM_ICP_GUESS`), and the guess comes from the wheel odometry unless `OHM_ICP_GUESS=identity`: ICP has a
basin of attraction and the wheel odometry is a free, if imperfect, starting guess. `tools/icp_eval.py
--basin` is where the size of that basin is measured; using identity here would be a node that fails for a
reason it does not mention.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

from ohm_localization import icp
from ohm_localization.icp import register_scans

SLEEP = 5.0                 # s, longer than this between two scans is not a step to integrate
REPORT_DT = 0.2             # s, how often the pose goes out
MODE = os.environ.get("OHM_ICP_MODE", "line")          # "line" | "point", see docs/icp.md
STRIDE = int(os.environ.get("OHM_ICP_STRIDE", "4"))    # every n-th beam becomes a point
SIGMA_Z = float(os.environ.get("OHM_ICP_SIGMA_Z", "0.05"))
USE_ODOM_GUESS = os.environ.get("OHM_ICP_GUESS", "odom") != "identity"


class IcpOdom:
    """The accumulated transform, updated once per scan pair."""

    def __init__(self, mode=MODE, stride=STRIDE, sigma_z=SIGMA_Z, guess=USE_ODOM_GUESS):
        self.T = icp.se2()                    # scan 0 defines the frame everything is expressed in
        self.prev, self.prev_odom, self.prev_t = None, None, None
        self.pairs, self.rejected, self.steps = 0, 0, []
        self.mode, self.stride, self.sigma_z, self.guess = mode, stride, sigma_z, guess

    def step(self, scan, odom=None):
        """One scan. False for the first, for a stale one, and for a gap no pairwise match can bridge.

        `scan` is the simulator's `Scan` or a recorded dict (see `tools/record_scans.py`), which is why the
        stamp is read through `_stamp()` rather than as an attribute: the offline measurement and the live
        node must run the same integrator, or the number in the documentation describes neither.
        """
        t = _stamp(scan)
        """Integrate one scan. False for the first one, and for a gap that no pairwise match can bridge."""
        if self.prev is not None and self.prev_t is not None and t - self.prev_t > SLEEP:
            self.rejected += 1                # an 8 m jump is not a step; integrate nothing across it
            self.prev, self.prev_odom = None, None
        if self.prev is None:
            self.prev, self.prev_t, self.prev_odom = scan, t, odom
            return False
        if t <= self.prev_t:
            return False                      # an old or repeated scan is not a step forward
        T_guess = icp.relative(self.prev_odom, odom) if (self.guess and odom is not None
                                                         and self.prev_odom is not None) else icp.se2()
        try:
            res = register_scans(self.prev, scan, T_guess=T_guess, mode=self.mode,
                                 stride=self.stride, sigma_z=self.sigma_z)
        except Exception:                     # noqa: BLE001 - a failed pair is a rejected pair
            self.rejected += 1
            self.prev, self.prev_t, self.prev_odom = scan, t, odom
            return False
        if not np.isfinite(res.T).all() or not res.converged:
            self.rejected += 1
            self.prev, self.prev_t, self.prev_odom = scan, t, odom
            return False
        self.advance(res.T)
        self.pairs += 1
        self.steps.append(res)
        self.prev, self.prev_t, self.prev_odom = scan, t, odom
        return True

    def advance(self, T_src_to_dst: np.ndarray) -> None:
        """Integrate one pairwise transform. Separate, so that the control test can drive it directly.

        `T_src_to_dst` carries points from the previous scan's frame into this scan's frame — that is
        what `icp.relative(a, b)` means and what the pairwise evaluation measures against. The robot's own
        increment is the inverse of that. Getting this wrong does not fail: integrating the *exact* per-step
        transforms of a 36 s drive wanders 4.9 m away from the truth those transforms were made of, and the
        report still looks like a localiser. `test/test_icp_odometry.py` integrates exact truth deltas
        through this method as a control before it believes one word about ICP.
        """
        self.T = self.T @ icp.inverse(T_src_to_dst)

    def pose(self) -> tuple:
        return icp.triple_of(self.T)

    def sigmas(self) -> tuple:
        """The median pairwise σ, widened by √(pairs): the only honest way to state an integrated error.

        A single ICP step reports its covariance from the residuals of that step, and integrating N steps
        multiplies the variance by N if the steps are independent — which is a generous assumption here, see
        the bias measurement in `test/test_icp_odometry.py`, and a generous one in the direction of caution:
        the true growth is linear in N when the residual is bias rather than noise, so √N still under-claims.
        Reporting a pairwise σ as though it were the pose σ would claim a few millimetres for a 36-second
        drive, which is the mistake this method exists to prevent and the first thing to ask about in a viva.

        The median over the pairs, not the last pair: the last one is chosen by the clock and not by the
        geometry, and over the graded drive the pairwise σ_θ runs from 0.13° at the median to 0.66° at its
        worst — a run described by its final step would be described by an accident.
        """
        if not self.steps:
            return (1.0, 1.0, 0.5)
        # Median of the pairs, not the last one: one degenerate step (a spin in a hall of flat walls reports
        # σ_θ = 11.7° for itself, measured) would otherwise describe a run of 700 steps whose median σ_θ is
        # 0.13°. The distinction is not cosmetic — the last pair is chosen by the clock, not by the geometry.
        sx, sy, sth = np.median(np.array([r.sigmas for r in self.steps]), axis=0)
        n = max(self.pairs, 1)
        return (sx * math.sqrt(n), sy * math.sqrt(n), sth * math.sqrt(n))


def _stamp(scan):
    """`.t` of a `Scan`, `["t"]` of a recording row, 0.0 of a bare range array nobody stamped."""
    if isinstance(scan, dict):
        return float(scan.get("t", 0.0))
    return float(getattr(scan, "t", 0.0))


def integrate(scans, odom=None, mode=MODE, stride=STRIDE, sigma_z=SIGMA_Z, guess=USE_ODOM_GUESS) -> list:
    """All of `scans` in order, as `(t, pose, sigmas)`. The offline half of this node.

    `odom` is the wheel odometry at the same instants, used only as ICP's starting guess; `None` means the
    identity guess, which is a different (worse) experiment and is asked for explicitly by
    `OHM_ICP_GUESS=identity`.
    """
    f = IcpOdom(mode=mode, stride=stride, sigma_z=sigma_z, guess=guess)
    out = []
    for k, scan in enumerate(scans):
        o = odom[k] if odom is not None else None
        if f.step(scan, o) or k == 0:
            out.append((_stamp(scan), f.pose(), f.sigmas()))
    return out


def mission(rob, task):
    """Publish the stitched pose on `kf/pose` for as long as this task runs."""

    # Function-local: this node reaches ROS through `robot_io`, and its entry point must stay
    # importable on a machine with no ROS.
    from ohm_localization.mcl_node import spin_or_stop

    print(f"icp_odom: mode {MODE}, stride {STRIDE}, σ_z {SIGMA_Z} m, guess "
          f"{'odometry' if USE_ODOM_GUESS else 'identity'} — no map is read by this node", file=sys.stderr)
    f, letzter, last_t = IcpOdom(), -1e9, None
    while rob.running() and rob.task() == task and spin_or_stop(rob, 0.005):
        o, scan = rob.odom(), rob.scan()
        if scan is None or last_t is not None and scan.t <= last_t:
            continue
        last_t = scan.t
        f.step(scan, o)
        x, y, th = f.pose()
        sx, sy, sth = f.sigmas()
        stamp = o.t if o is not None else scan.t
        if stamp - letzter >= REPORT_DT:
            letzter = stamp
            rob.send_kf(x, y, th, sx, sy, sth,
                        info={"t": round(stamp, 2), "pairs": f.pairs, "rejected": f.rejected,
                              "mode": f.mode, "stride": f.stride, "integrated": True})


def main() -> None:
    """`ros2 run ohm_localization icp_odom_node --robot alice`, and `python3 -m …`.

    Imports the simulator's runner here rather than at module level for the reason in `mcl_node.py`: the rest
    of this package imports numpy and nothing else, and a node that cannot be imported without ROS cannot be
    tested without it either.
    """
    from mecanum_lab import robot_io                 # the controller door; see mcl_node.py for the other two
    try:
        robot_io.serve(sys.modules[__name__])
    except KeyboardInterrupt:
        print("icp_odom_node: stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
