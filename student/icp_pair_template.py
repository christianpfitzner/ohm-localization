"""E2 — one model scan, one scene scan, one pose and the σ that belongs to it.  `register()` is yours.

    python3 tools/lab_check.py e2_icp_pair                    # the five criteria the sheet lists
    python3 student/icp_pair_template.py                      # one pair, printed: iterations and σ

**What this exercise is for.** E1 produced correspondences; this exercise turns them into a pose and — the
part that is graded and that most groups leave out — into a statement about how much that pose is worth. A
pose without a σ is an assertion, and the viva's question about it has exactly two honest answers: the number
came out of `JᵀJ` of the last iteration, or it did not.

**The convention, in one sentence, because reversing it is the one-character bug that costs an hour.** Your
answer is the transform that carries points from the frame of the **model** scan into the frame of the
**scene** scan:

    transform(T, points(model)) ≈ points(scene)              T = icp.relative(model_pose, scene_pose)

`icp.relative()`'s docstring is the one-line proof of that order, and `--verbose` prints the truth of every
pair so you can check yours against it in one line of code. The reversed order scores a large error and no
error message.

**What is yours — `TODO(E2)`.** Two things, and only the second one is new material:

 1. the loop: correspondences → solve the weighted least-squares step → apply → repeat, until the step or
    the fitness stops improving;
 2. `sx, sy, sth` from the *last* iteration's `JᵀJ` and residual variance.

You may use `icp.icp()` for (1) — the fit is not the exercise, the σ is, and the library's implementation is
there so that a 180-minute session is spent on the part that is graded. You may not use its covariance for
(2): derive it, and be able to say what each of the three numbers means at the pair you are standing at. The
criterion that catches a σ that was invented rather than derived is the corridor pair in `--verbose`: in a
bare corridor the fit along the corridor is free, and a σ that stays at a few millimetres there is not
measuring the fit. (`docs/icp-degeneracy.md` is the picture of exactly this, and `docs/verification.md` §7
has the numbers.)

**What is already here.** `pair_fixture()` hands you the same twelve pairs the grader scores — a fixed
relative motion, both poses free, and a pair rejected unless the identity explains it *badly* and the truth
explains it *well*, because a pair taken in a corner registers from the identity and measures nothing.
`demo()` prints one pair's iterations and its σ, which is the output you want for your protocol table.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

# Run it from anywhere: `python3 student/icp_pair_template.py`. The import below is a module-level one, so the
# path has to be fixed here rather than in the `__main__` block at the bottom, where it arrives too late to help
# and the demo dies with a ModuleNotFoundError about a package that is sitting one directory up.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ohm_localization import icp, synth                         # numpy only: no simulator, no ROS  # noqa: E402
from ohm_localization.exercises import corridor_pair, scan_pairs  # noqa: E402

SIGMA_Z = 0.02                    # the simulator's beam noise in metres: the σ of one range reading


# ------------------------------------------------------------------------------------ given: the pairs
def pair_fixture(n: int = 12, **kw) -> list:
    """The twelve model/scene pairs the grader scores, with their truths — the same fixture, same seed.

    Each entry is `{"model": scan, "scene": scan, "truth": 3×3, "poses": (a, b), "points": (n, m)}`. The
    rejection that keeps them honest is in `ohm_localization/exercises.py:scan_pairs()`; read it once, it is
    eight lines and it is the difference between a mark scheme and a lottery.
    """
    return scan_pairs(n=n, **kw)


def points(scan, stride: int = 1):
    """The (n, 2) cloud of one scan, blind beams dropped — the library's rule, so the count matches."""
    return icp.points_from_scan(scan, stride=stride)[0]


# --------------------------------------------------------------------------------------- yours: TODO
def register(model, scene, T_guess: np.ndarray | None = None) -> dict:
    """Register `model` onto `scene`; return the pose and its three standard deviations. **TODO(E2).**

    Returns a dict with at least

        x, y, theta     the pose (m, m, rad) of the transform that carries model points into scene points
        sx, sy, sth     its 1σ in (m, m, rad) — from the fit, not from a feeling

    A 3×3 array under the key `T` is accepted in place of x/y/theta while you develop it; the σ is asked for
    from the first submission, because a σ added afterwards is a σ fitted to the answer.

    `T_guess` is where the wheel odometry would put the scene if the wheels were the truth. It is `None` in
    most of the criteria and 0.5 m and 8° wrong in one of them, which is the criterion that asks how big
    your basin of attraction is. Two things about it decide whether the robot version of this exercise works:
    the correspondence gate (`max_corr`) is the basin, and point-to-line is the mode that survives a rotated
    start — `tools/icp_eval.py --basin` measures both statements on the real pairs.

    Report the σ from the last iteration only if it converged: `converged=False` with a covariance of a few
    millimetres is the most dangerous line a localiser can print.
    """
    raise NotImplementedError("TODO(E2): register() — the loop, and sx/sy/sth from JᵀJ. "
                              "See tools/lab_check.py e2_icp_pair.")


# ------------------------------------------------------------------------------------------------- demo
def demo(n_pairs: int = 1, verbose: bool = True) -> None:
    """One pair registered by the library, printed the way the protocol table wants it."""
    pairs = pair_fixture(n=n_pairs)
    for p in pairs:
        pa, pb = points(p["model"]), points(p["scene"])
        tx, ty, tt = icp.triple_of(p["truth"])
        r = icp.register_scans(p["model"], p["scene"], stride=1, max_iterations=40)
        ex, ey, et = r.pose
        dx, dth = icp.error_between(r.T, p["truth"])
        sx, sy, sth = r.sigmas
        print(f"{len(pa)} model points × {len(pb)} scene points, truth ({tx:+.2f}, {ty:+.2f}, "
              f"{math.degrees(tt):+.1f}°), converged {r.converged} after {r.iterations} iterations")
        print(f"  answer           ({ex:+.3f}, {ey:+.3f}, {math.degrees(et):+.2f}°)  "
              f"error {dx * 1000:5.1f} mm {dth:5.2f}°")
        print(f"  σ from JᵀJ       ({sx * 1000:5.1f}, {sy * 1000:5.1f} mm, {math.degrees(sth):.2f}°)  "
              f"condition of JᵀJ {r.cond:.1e}, fitness {r.fitness * 1000:.1f} mm "
              f"({r.correspondences} kept, {r.rejected} gated out)")
        if verbose:
            for it, mean_d, step, rot in r.history[:6]:
                print(f"    iter {it:2d}  mean distance {mean_d * 1000:7.1f} mm  step {step:.4f} m "
                      f"{rot:.4f} rad")
    c = corridor_pair()
    r = icp.register_scans(c["model"], c["scene"], stride=1, max_iterations=40)
    sx, sy, sth = r.sigmas
    print(f"the same matcher in a bare corridor, slid {c['along'][0] * 1.5:.1f} m along it: "
          f"σ along {sx * 1000:.0f} mm, across {sy * 1000:.1f} mm, condition {r.cond:.1e}")
    print("  that is the degeneracy the fifth criterion looks for — a σ that stays small there is a σ that "
          "is not read off the fit. docs/icp-degeneracy.md is the picture of it.")


if __name__ == "__main__":
    demo()
