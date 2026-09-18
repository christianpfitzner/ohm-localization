"""E2 solved: the library's fit, the σ it reports, and the one number that says when the hall said nothing.

    python3 -m solution.icp_pair_solution                                   # twelve pairs and a corridor
    python3 tools/lab_check.py e2_icp_pair --module solution/icp_pair_solution.py
    python3 tools/lab_check.py e2_icp_pair --module student/icp_pair_template.py   # six criteria fail

The transform and the σ both come from the library here, and that is the floor of the exercise rather than its
ceiling: what is graded is that the pose lands, that the σ came off each fit instead of out of a constant, and
that the degeneracy is in the numbers you report. `python3 -m solution.icp_pair_solution` prints all of it, and
the two lines worth staring at are the last two:

    median 7.1 mm 0.09°, worst 36.0 mm 0.54°, median cond 9.0e+00
    corridor: σ (21, 1.2 mm, …), cond 2.6e+03

**Why the corridor's σ is 21 mm and not a metre.** In a bare corridor slid along its own axis the data
constrain the pose across the walls and not along them: every locally fitted line segment lies along a wall, so
its normal is across, and the x-column of `J` should be identically zero. It is not, because the 20 mm beam
noise displaces the points *along* the grazing beams, which tilts the segments, and a near-null direction then
collects a σ from the noise rather than from the geometry. `test/test_icp.py` has the same case at 1.2 mm and
is titled "the covariance only just notices" for exactly this reason. **`cond` is the number that shouts**: 9
in a hall with corners, 2600 in a corridor, a factor of 290.

**And why "just add a prior" is the wrong reflex here**, which is the question the viva asks: information adds,
so a prior can only ever *shrink* a covariance — `C = (JᵀJ/s² + P)⁻¹` with a 300 mm prior leaves a 21 mm σ at
21 mm. A prior is what stops a *singular* fit from answering confidently (`icp.py:_cov()` does the equivalent
by handing back `diag(99, 99, 9)` when `cond > 10¹⁰`); it is not what makes a merely *near*-singular fit
honest. For that there are three defensible moves and the protocol asks you to pick one and say why: report
the meter and widen the σ on the strength of it (what `icp_odom_node.py:sigmas()` does for an integrated pose,
growing it by √pairs), refuse the pair when `cond` is over a threshold you measured, or publish the σ you have
and never use it as a control signal in the direction the meter says is free.
"""
from __future__ import annotations

import math

import numpy as np

from ohm_localization import icp

MODE = "line"           # point-to-line: the mode that survives a rotated start, see docs/icp.md §6
MAX_ITERATIONS = 40     # the library's 30 converges everywhere here; 40 costs nothing on a straggler
SIGMA_Z = 0.02          # the simulator's beam noise in metres — one reading's σ, so s² is in units of it


def register(model, scene, T_guess: np.ndarray | None = None) -> dict:
    """Register `model` onto `scene`; the pose, the σ, and the numbers the protocol table asks for."""
    r = icp.register_scans(model, scene, T_guess=T_guess, mode=MODE, sigma_z=SIGMA_Z,
                           max_iterations=MAX_ITERATIONS)
    sx, sy, sth = r.sigmas                    # sqrt of the diagonal of s²(JᵀJ)⁻¹ of the *last* iteration
    return {"x": r.pose[0], "y": r.pose[1], "theta": r.pose[2], "T": r.T,
            "sx": sx, "sy": sy, "sth": sth,
            "iterations": r.iterations, "converged": r.converged, "fitness": r.fitness,
            "correspondences": r.correspondences, "rejected": r.rejected,
            "cond": r.cond}                   # the degeneracy meter: 9 in the hall, 2600 in the corridor


if __name__ == "__main__":
    from ohm_localization.exercises import corridor_pair, scan_pairs
    err_t, err_r, cond = [], [], []
    for p in scan_pairs(n=12):
        a = register(p["model"], p["scene"])
        d, th = icp.error_between(np.asarray(a["T"]), p["truth"])
        err_t.append(d)
        err_r.append(th)
        cond.append(a["cond"])
        print(f"pair at ({p['poses'][0][0]:6.2f}, {p['poses'][0][1]:6.2f}): {a['iterations']} iterations, "
              f"error {d * 1000:6.1f} mm {th:5.2f}°, σ ({a['sx'] * 1000:5.1f}, {a['sy'] * 1000:5.1f} mm, "
              f"{math.degrees(a['sth']):.3f}°), cond {a['cond']:.1e}, {a['correspondences']} correspondences")
    print(f"median {np.median(err_t) * 1000:.1f} mm {np.median(err_r):.2f}°, "
          f"worst {max(err_t) * 1000:.1f} mm {max(err_r):.2f}°, median cond {np.median(cond):.1e}")
    c = corridor_pair()
    a = register(c["model"], c["scene"])
    print(f"corridor: σ ({a['sx'] * 1000:.0f}, {a['sy'] * 1000:.1f} mm, {math.degrees(a['sth']):.4f}°), "
          f"cond {a['cond']:.1e} — the σ whispers, the condition number shouts")
