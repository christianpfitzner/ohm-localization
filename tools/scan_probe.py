#!/usr/bin/env python3
"""Diagnostic node: is this repo's map and beam model the simulator's, or its mirror image?

Run it as a controller and it answers in a few lines:

    tools/run_lab.sh grade --task mcl_production --controller tools/scan_probe.py

Why this exists rather than a look at the code: the offline tests in `tests/` generate their scans
with `ohm_localization.synth.cast()` from the rectangles `gridmap.parse_grid()` read out of
`worlds/*.txt`.  If either of the two conventions a LIDAR model has — which way the map's text rows
run, and which way the beams sweep — is the opposite of the simulator's, the offline suite cannot
notice, because both sides of every comparison come from the same file.  The scans and the map then
agree with each other and disagree with the robot, and the only symptom is a particle filter that
converges beautifully onto a pose a couple of metres from the one the simulator has.  That is what
the first integration run of `solution/mcl_node.py` looked like: 2.08 m RMSE, every offline test
green.

So this file takes the one thing the tests cannot have — a real `/scan` beside a real `/truth` — and
compares it with the synthetic scan of that same truth pose, for each convention.  The mean absolute
beam difference is a few millimetres for the one that is right and metres for the ones that are not.

It also prints the **message clock** of the three topics a stamp-driven node reads.  That is not a
second question bolted on: the filter ticks on message stamps because `tools/fastgrade.py` runs the
simulator 25× faster than real time, and if a topic's stamp did not advance, such a node would simply
never look at that sensor — the same 2.08 m symptom, from the opposite direction, and worth ruling
out in the same run.

Conventions checked, all of them defensible readings of the same hardware:

    identity    beam i at angle_min + i·step + theta, counter-clockwise, map as parsed
    mirror_y    the map's text rows run top-to-bottom and the y-flip in parse_grid is backwards
    mirror_x    the same about the other axis
    clockwise   the beams sweep the other way: beam i at theta − i·step
    reverse     beam order reversed about the array centre (a different way to be clockwise)
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from ohm_localization.gridmap import (GridMap, load_hall,   # noqa: E402
                                      mecanum_lab_dir)

# The bootstrap is here rather than left to `tools/run_lab.sh` so that the import works whether this is
# loaded as a controller by the node loader or imported directly by a test.  It still needs a session
# to talk to: run it as a controller, because its whole subject is a real `/scan` beside a real
# `/truth`, which nothing else in this repository can produce.
sys.path.insert(0, mecanum_lab_dir() or ".")
from mecanum_lab import robot_io                                # noqa: E402

from ohm_localization import synth                              # noqa: E402

CANDIDATES = ("identity", "mirror_y", "mirror_x", "clockwise", "reverse")
FRAMES = 120                       # sample this many odometry messages, then report and stop
FIRST = 20                         # ignore the start-up frames: the robot has barely moved


def mirrored(hall, axis):
    """The same hall, reflected: the walls move, the size does not."""
    w, h = hall.size
    r = np.asarray(hall.rects).copy()
    if axis == "y":
        r[:, [1, 3]] = h - r[:, [3, 1]]
    else:
        r[:, [0, 2]] = w - r[:, [2, 0]]
    return type(hall)(name=hall.name + " mirrored in " + axis, cell=hall.cell, size=hall.size, rects=r)


def pose_in_mirror(truth, axis, size):
    """The same robot, seen in the mirrored map — its heading mirrors with it."""
    if axis == "y":
        return (truth.x, size[1] - truth.y, math.pi - truth.theta)
    return (size[0] - truth.x, truth.y, -truth.theta)


def compare(truth, scan, hall):
    """Mean |real − synthetic| per convention, and how many beams that mean was taken over.

    The count is not decoration; it is the half of this check that nearly fooled its author.  A wrong
    convention that blinds half the beams is graded on an easier question: on the spawn pose of
    `arena` the map mirrored in x explains 115 beams at 0.0120 m while the correct one explains all
    180 echoing beams at 0.0125 m, so ranking by the bare mean picks the mirror — and the mirror is
    not a better model, it is a model that got to skip 65 beams.  A convention is therefore only
    eligible when it uses nearly as many beams as the real scan has echoes.  A metric whose
    denominator moves under you is not a metric.
    """
    got = np.asarray(scan.ranges, dtype=float)
    echoed = int(np.count_nonzero(np.isfinite(got)))
    out = {}
    for conv in CANDIDATES:
        h = mirrored(hall, conv[-1]) if conv.startswith("mirror") else hall
        pose = pose_in_mirror(truth, conv[-1], hall.size) if conv.startswith("mirror") \
            else (truth.x, truth.y, truth.theta)
        want = synth.cast(pose, h, beams=len(got), range_max=scan.range_max)
        if conv == "clockwise":
            want = np.r_[want[0], want[:0:-1]]                # beam 0 stays forward, the rest unwind
        elif conv == "reverse":
            want = want[::-1]
        both = np.isfinite(want) & np.isfinite(got)
        out[conv] = (float(np.mean(np.abs(want[both] - got[both]))) if both.any() else float("inf"),
                     int(both.sum()), echoed)
    eligible = {k: v for k, v in out.items() if v[1] >= 0.9 * echoed}
    return out, min(eligible or out, key=lambda k: out[k][0]), echoed


def stamp(*measure) -> float:
    return max([m.t for m in measure if m is not None] + [0.0])


def _record(series, value):
    """Append a stamp if it is newer than the last one; return whether the topic ever appeared."""
    if value is None:
        return False
    if not series or value.t > series[-1]:
        series.append(value.t)
    return True


def mission(rob, task):
    """Sample a stretch of the drive: the convention verdict and the message clock."""
    name = str((rob.world() or {}).get("name") or rob.config("world", "arena"))
    hall = load_hall(name)
    GridMap(hall)                                              # built here so a bad map fails now
    baseline = stamp(rob.odom(), rob.scan(), rob.truth())
    seen = {kind: [] for kind in ("odom", "scan", "truth")}
    verdict = None
    while rob.running() and rob.task() == task and len(seen["odom"]) < FRAMES:
        rob.spin(0.005)
        o, scan, truth = rob.odom(), rob.scan(), rob.truth()
        if o is None or stamp(o, scan, truth) <= baseline:
            continue                                   # the bus still remembers the previous run
        for kind, msg in (("odom", o), ("scan", scan), ("truth", truth)):
            _record(seen[kind], msg)
        if truth is None and not seen["truth"]:
            print("scan_probe: no /truth — run under the grader (debug_truth) or `run --truth`",
                  file=sys.stderr)
            return
        if scan is not None and len(seen["odom"]) > FIRST and verdict is None:
            verdict = compare(truth, scan, hall)

    print(f"\nscan_probe: {hall.name} — {len(seen['odom'])} frames of one drive")
    for kind in ("odom", "scan", "truth"):
        v = seen[kind]
        if not v:
            print(f"   {kind:6s} never arrived on this bus")
            continue
        mono = all(b > a for a, b in zip(v, v[1:]))
        span = v[-1] - v[0]
        hz = (len(v) - 1) / span if span > 0 else float("inf")
        print(f"   {kind:6s} {len(v):4d} distinct stamps, {v[0]:.3f} … {v[-1]:.3f} s, "
              f"strictly increasing {mono}, about {hz:.1f} Hz")
        if not mono:
            print(f"   → {kind}'s stamp does not advance, so a node that ticks on stamps — which is "
                  f"what every filter in this exercise does, on purpose — would never use it.")
    if verdict is None:
        print("scan_probe: no scan beside a truth inside the sampling window", file=sys.stderr)
        return
    res, best, echoed = verdict
    print(f"\n   map and beam convention, at one pose, {echoed} of 360 beams echoing:")
    for conv in CANDIDATES:
        d, used, _ = res[conv]
        mark = "  <== the simulator" if conv == best else \
            ("  (not eligible: uses too few of the echoing beams to be compared)"
             if used < 0.9 * echoed else "")
        print(f"      {conv:10s} mean |real − synthetic| = {d:7.4f} m over {used:3d} beams{mark}")
    if best != "identity":
        print("   → this repo's map or beam convention is NOT the simulator's.  Nothing that "
              "inherits it — mcl_node, the ICP pair fixtures, every offline number — can be trusted "
              "until this line says identity.", file=sys.stderr)
    else:
        print(f"   → identity.  The synthetic scan of the true pose differs from the real /scan by "
              f"{res['identity'][0] * 1000:.1f} mm, which is the beam noise (σ = 15 mm) and nothing "
              f"else: `synth.cast()` is the simulator's ray cast, not an approximation of it.")


if __name__ == "__main__":
    robot_io.serve(sys.modules[__name__])
