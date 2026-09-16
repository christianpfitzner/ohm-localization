#!/usr/bin/env python3
"""ICP between two LIDAR scans — numpy only, no ROS, no simulator.

    python3 examples/03_icp_scan_matching.py

Two scans of the same walls from two poses: ICP answers with the transform that carries one onto the
other, its σ, and how much better its answer is than the guess it started from. A hall with posts gives
it corners to lock on; a 121 × 5 m corridor gives it two long walls and nothing else.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                            # noqa: E402

from ohm_localization import icp, synth                       # noqa: E402
from ohm_localization.gridmap import corridor_text, parse_grid          # noqa: E402

BEAMS, RANGE_MAX = 360, 12.0
FROM = (5.0, 3.0, 0.0)                       # the pose the first scan was taken from
STEP = (0.6, 0.0, 0.08)                      # what the robot really did between them: m, m, rad


def scan_at(pose, hall, t=0.0):
    return synth.scan_dict(pose, hall, t=t, beams=BEAMS, range_max=RANGE_MAX, sigma=0.02,
                           rng=np.random.default_rng(5))


def match(name, hall, start=FROM, step=STEP):
    """One registration: its error, its σ, and how flat the valley around the answer is."""
    to = (start[0] + step[0], start[1] + step[1], start[2] + step[2])
    a, b = scan_at(start, hall), scan_at(to, hall, t=1.0)
    result = icp.register_scans(a, b, mode="line", stride=4, sigma_z=0.05)
    distance, degrees = icp.error_between(result.T, icp.relative(start, to))
    sx, sy, sth = result.sigmas
    # The same pair from a start 0.5 m further along the hall: a hall with nothing in the direction of
    # travel is nearly as pleased there as it is at the answer.
    slid = icp.register_scans(a, b, T_guess=icp.se2(-0.5, 0.0, 0.0), mode="line", stride=4,
                              sigma_z=0.05)
    print(f"  {name:16s} {distance * 1000:5.0f} mm off, {degrees:5.2f}°, {result.iterations} "
          f"iterations, {result.correspondences:3d} pairs")
    print(f"  {'':16s} σ = ({sx:.2f}, {sy:.2f}) m along/across the hall — along is "
          f"{sx / max(sy, 1e-9):.0f}× across, cond(JᵀJ) {result.cond:.1e}")
    print(f"  {'':16s} fitness at the answer {result.fitness * 1000:5.1f} mm, from a guess 0.5 m off "
          f"{slid.fitness * 1000:5.1f} mm — "
          + ("flat, the hall cannot tell the two apart" if slid.fitness < 2 * result.fitness
             else "a valley: the wrong guess is punished"))


for name, text in (("hall with posts", corridor_text(cols=60, rows=10, pillars=2)),
                   ("empty corridor", corridor_text(cols=240, rows=10))):
    hall = parse_grid(text, cell=0.5, name=name)
    print(f"\n{name}: {hall.size[0]:.0f} × {hall.size[1]:.0f} m, the robot moved "
          f"{STEP[0]:.1f} m and {np.degrees(STEP[2]):.0f}°, scans of {RANGE_MAX:.0f} m range")
    match(name, hall)

print("\nTwo long walls fix the sideways offset and the heading; almost nothing fixes a shift along them.")
print("That is what the wide along-hall σ and the fitness of a 0.5 m wrong guess are saying, and it is")
print("why ICP is a good correction to odometry and a poor substitute for it — see docs/icp.md §7.")
