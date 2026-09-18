"""E3 solved: scan matching as odometry, with the one guard that makes the integration survivable.

    ./tools/run_lab.sh grade --task icp_odom_production --controller solution/icp_odom_solution.py
    python3 -m solution.icp_odom_solution                        # the same matcher on desk pairs, no robot

The function is E2's registration and the interesting part of this file is what happens when it does not
believe the answer. The numbers behind the two guards are measured on a recording of this exact drive
(`OHM_RECORD=drive.jsonl ./tools/run_lab.sh grade --task icp_odom_production --controller
tools/record_scans.py`, then replayed — the loop is in `docs/exercises.md` and it turns a 40 s run into a
second-and-a-half experiment):

* **The rejection is the guard that works.** The median pair on this drive fits to 23.8 mm; 14 of the 726 do not
  converge at all. Refusing a fit whose mean correspondence distance is over 60 mm — 2.5× the beam noise, 2.5×
  the median — changes this drive from 1.20 m to 0.99 m RMSE, because one poisoned pair held for the remaining
  30 s is worth more than every other refinement in this file.
* **The gate is not a quality meter, and tightening it is not the fix.** Two attempts, both measured, both worth
  knowing: a 0.25 m gate ("consecutive scans are 5 cm apart, so a true correspondence is nearby") makes the drive
  *worse* — 1.18 m against 0.99 m on the replay, and 1.281 m against 1.065 m when the same change is graded (widening
  to 1.5 m grades 1.046 m, i.e. nothing outside the run-to-run spread) — because at 8 m range a 1° rotation moves a
  point 14 cm and the next wall along is a metre away, so a tight gate drops the good correspondences with the bad
  ones. And a guard on the *fraction* of points surviving the gate, which looks like an obvious sanity check, costs
  the drive its sensor: 2.57 m on the replay, 2.812 m at 1.42× when graded, both FAILs. The median pair keeps 48 %
  of its points, always, so a `keep_min` of 0.55 rejects two pairs in three and the node spends the drive integrating
  wheels it stopped trusting. `KEEP_MIN = 0.15` is a floor for the case where the matcher found almost nothing, and that is
  all it is for.

Everything else is `student/icp_odom_template.py`, unchanged, and the difference between the two files is one
function: the same convention (`T` carries points from the previous scan into this one, the robot's increment is
its inverse), the same anchor, the same 5 Hz report gate, the same √pairs σ. Run both and compare the improvement
column — that comparison is the exercise, and `docs/verification.md` §19 is where the measured version of it lives.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

from ohm_localization import icp

STRIDE = 2                    # every second beam: 180 points, the point where the accuracy/cost curve is flat
SIGMA_Z = 0.02                # m, the simulator's beam noise: what s² of the fit is scaled against
MODE = "line"                 # point-to-line, docs/icp.md §6
MAX_CORR = 0.5                # m, the gate — and see the docstring: 0.25 m grades worse (1.281 m), 1.5 m no better
                               # (1.046 m)
MAX_ITERATIONS = 20           # consecutive scans are 5 cm apart; a fit that needs 20 is not converging
FIT_MAX = 0.06                # m, the mean correspondence distance still believed: 2.5× the beam noise, and
                              # 2.4× the median of this drive's pairs, which is 23.8 mm
KEEP_MIN = 0.15               # of the points that must survive the gate — a sanity floor and nothing more
REPORT_DT = 0.15              # s, the shortest period between two kf/pose messages


def scan_step(prev_scan, scan, guess: np.ndarray, cfg: dict) -> tuple:
    """Register the previous scan onto this one; `(T, (sx, sy, sth))`, or the identity when the fit is refused."""
    r = icp.register_scans(prev_scan, scan, T_guess=guess, stride=cfg.get("stride", STRIDE),
                           mode=cfg.get("mode", MODE), sigma_z=cfg.get("sigma_z", SIGMA_Z),
                           max_iterations=MAX_ITERATIONS, max_corr=cfg.get("max_corr", MAX_CORR))
    kept = r.correspondences / max(r.correspondences + r.rejected, 1)
    if not r.converged or r.fitness > FIT_MAX or kept < KEEP_MIN:
        # The pose does not move, and the σ is the one for "we have not seen anything lately": the range the
        # matcher could not resolve rather than the millimetres of a fit it threw away.
        return np.eye(3), (MAX_CORR, MAX_CORR, 0.35)
    return r.T, r.sigmas


def mission(rob, task):
    """Publish the stitched pose on `kf/pose`. Same loop as the template, because none of this is the exercise.

    The bookkeeping is imported from the template rather than repeated here, on purpose: `student/` is the
    scaffolding the exercise hands out, and the statement this file is supposed to make is that the difference
    between a failing node and a passing one is one function.
    """
    from ohm_localization.mcl_node import spin_or_stop           # function-local, as in mcl_node.py
    from ohm_localization.exercises import ROOT, load_module

    IcpOdom = load_module(os.path.join(ROOT, "student", "icp_odom_template.py"),
                          "ohm_e3_scaffold").IcpOdom
    f, letzter, last_t = IcpOdom(mode=MODE, stride=STRIDE, sigma_z=SIGMA_Z, max_corr=MAX_CORR,
                                 step_fn=scan_step), -1e9, None
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
                              "gate": MAX_CORR, "integrated": True})


if __name__ == "__main__":
    from ohm_localization.exercises import scan_pairs
    err = []
    for p in scan_pairs(n=6):
        T, sig = scan_step(p["model"], p["scene"], np.eye(3),
                           {"stride": STRIDE, "mode": MODE, "sigma_z": SIGMA_Z, "max_corr": MAX_CORR})
        d, th = icp.error_between(T, p["truth"])
        err.append(d)
        print(f"pair: error {d * 1000:6.1f} mm {th:5.2f}°, σ ({sig[0] * 1000:4.1f}, {sig[1] * 1000:4.1f} mm, "
              f"{math.degrees(sig[2]):.3f}°)")
    print(f"median {np.median(err) * 1000:.1f} mm — with the 0.25 m gate the desk pairs still register; "
          f"the gate only removes the alias basins, which a 1.2 m pair never needed")
