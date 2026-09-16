#!/usr/bin/env python3
"""Record a real drive — odometry, truth and every scan — to a JSONL file for offline work.

    OHM_RECORD=drive.jsonl tools/run_lab.sh grade --task mcl_arena --controller tools/record_scans.py
    python3 tools/mcl_report.py drive.jsonl                 # the same filter, offline, with numbers

Two things this is for, and they are the two things a filter exercise needs that a live run cannot
give you.

**Replay.**  Every number in `docs/verification.md` is reproducible from a file like this: the filter
sees the identical scans and the identical odometry, so a change to the sensor model is blamed on the
change and not on the wind.  It also makes the offline suite and the laboratory's grader comparable —
the offline tests use `synth.cast()`, `tools/scan_probe.py` shows that is the simulator's own ray cast
to within the beam noise, and this closes the circle by running the filter over the real thing.

**The truth column.**  `debug_truth` is on under the grader, so the file carries the exact pose next to
the odometry that claims to be it.  The distance between those two *is* the quality of the
laboratory's odometry, and the improvement a localiser is supposed to earn is measured against nothing
else.  If a hall's odometry drifts 2 cm over a drive, no filter can beat it by a factor of two and a
threshold that asks for it is grading the simulator.  That is how the odometry profile in
`config/tasks_localization.json` came to be what it is — and it is printed when the recording ends.

Format: one JSON object per line.  The first is the header (`world`, `cell`, `size`, the hall as grid
text, `task`), then one per scan: `t`, `truth`, `odom`, the 360 ranges and the four angle fields.
Line-oriented and flushed per line because this runs as a node *thread* of the grading process, which
is killed rather than unwound when the last task finishes — the first version had the write in a
`finally` and produced no file at all, no error, nothing: a daemon thread does not run its `finally`
clauses at interpreter shutdown.
"""
import json
import math
import os
import sys

from mecanum_lab import robot_io

OUT = os.environ.get("OHM_RECORD", "drive.jsonl")


def stamp(*measure) -> float:
    return max([m.t for m in measure if m is not None] + [0.0])


def mission(rob, task):
    """Log every scan until the task ends, one line each, flushed as it goes."""
    from ohm_localization.gridmap import load_hall

    name = str((rob.world() or {}).get("name") or rob.config("world", "arena"))
    hall = load_hall(name)
    baseline = stamp(rob.odom(), rob.scan())
    last_scan, n, frames = 0.0, 0, []
    with open(OUT, "w", encoding="utf-8") as fh:
        fh.write(json.dumps({"header": True, "world": hall.name, "cell": hall.cell,
                             "size": list(hall.size), "task": task, "grid_map_text": _grid_of(hall),
                             "note": "tools/record_scans.py; ranges in metres, null = no echo within "
                                     "range_max; angles radians; one scan per line"}) + "\n")
        fh.flush()
        while rob.running() and rob.task() == task:
            rob.spin(0.005)
            o, scan, truth = rob.odom(), rob.scan(), rob.truth()
            if o is None or stamp(o, scan, truth) <= baseline:
                continue                                   # the bus still remembers the previous run
            if scan is None or scan.t <= last_scan:
                continue                                   # one record per scan, none twice
            last_scan = scan.t
            frame = {"t": float(scan.t), "truth": None if truth is None
                     else [truth.x, truth.y, truth.theta],
                     "odom": [o.x, o.y, o.theta, o.vx, o.vy, o.omega],
                     "ranges": [None if math.isinf(r) else float(r) for r in scan.ranges],
                     "angle_min": float(scan.angle_min), "angle_increment": float(scan.angle_increment),
                     "range_min": float(scan.range_min), "range_max": float(scan.range_max)}
            fh.write(json.dumps(frame) + "\n")
            fh.flush()
            n += 1
            if truth is not None:
                frames.append(frame)
    drift = _drift(frames)
    print(f"record_scans: {n} scans of '{task}' in {hall.name} -> {OUT}", file=sys.stderr)
    if drift is None:
        print("record_scans: no /truth in this run — record under the grader or with `run --truth`, "
              "or the file has no odometry error to report", file=sys.stderr)
    else:
        print(f"record_scans: the odometry of this drive was {drift[0]:.3f} m RMSE from the truth, "
              f"worst {drift[1]:.3f} m, {drift[2]:.1f} deg off in heading at the end. That is the "
              f"number a localiser has to beat.", file=sys.stderr)


def _drift(frames):
    """How wrong the odometry was, in the units the grader uses for `sensor: "odom"`."""
    e, eth = [], []
    for f in frames:
        e.append(math.hypot(f["odom"][0] - f["truth"][0], f["odom"][1] - f["truth"][1]))
        d = f["odom"][2] - f["truth"][2]
        eth.append(abs(math.degrees(math.atan2(math.sin(d), math.cos(d)))))
    if not e:
        return None
    return sum(e) / len(e), max(e), eth[-1]


def _grid_of(hall):
    """The hall as `parse_grid` would read it back, so a replay needs no simulator checkout.

    Joined with an empty separator, which is worth stating because the first version joined with
    "#": the recording then carried a map twice as wide as the hall, 95 characters to a row instead of
    48, and a replay on it produced a filter that was 300 mm wrong with its estimate inside a wall 45 %
    of the time — all of it from a separator.  `tools/scan_probe.py` is the check that catches a wrong
    map on the live side; the row count printed by `tools/mcl_report.py` is the check that catches it
    here, which is why the grid size is in the first line of its output.
    """
    from ohm_localization.gridmap import GridMap
    g = GridMap(hall, resolution=hall.cell)
    return "\n".join("".join("#" if v else "." for v in row) for row in (g.data == 100)[::-1])


if __name__ == "__main__":
    robot_io.serve(sys.modules[__name__])
