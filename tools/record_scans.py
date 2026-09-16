#!/usr/bin/env python3
"""Record one drive — odometry, truth and every scan — to JSONL for offline work.

    OHM_RECORD=drive.jsonl ./tools/run_lab.sh grade --task mcl_production --controller tools/record_scans.py
    python3 tools/mcl_report.py drive.jsonl          # the same filter offline, with numbers

**Replay.** The filter sees the identical scans and the identical odometry twice, so a parameter change is
compared against a fixed measurement series.

**The truth column.** `debug_truth` is on under the grader, so the file carries the exact pose next to the
odometry that claims to be it. The distance between those two is the quality of the simulator's odometry,
and it is what the `improvement_min` of a task is measured against; it is printed when the recording ends.

Format: one JSON object per line. First the header (`world`, `cell`, `size`, the hall as grid text, `task`),
then one per scan: `t`, `truth`, `odom`, the ranges and the four angle fields. Line-oriented and flushed per
line: this runs as a node *thread* of the grading process, which is killed rather than unwound when the last
task finishes, so a `finally` would never run and no file would appear.
"""
import json
import math
import os
import sys

from mecanum_lab import robot_io

OUT = os.environ.get("OHM_RECORD")
if not OUT:
    # No default: an implicit `drive.jsonl` in the current directory would be overwritten on every run.
    print("record_scans: no output file — run it as  OHM_RECORD=drive.jsonl  …  (nothing was written)",
          file=sys.stderr)
    raise SystemExit(2)


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
    """The hall as `parse_grid` reads it back, so a replay needs no simulator checkout.

    Rows are joined with an empty separator: with a `#` between them the recorded map is twice as wide as
    the hall. `tools/scan_probe.py` checks the map convention on the live side; the grid size in the first
    line of `tools/mcl_report.py` output checks it here.
    """
    from ohm_localization.gridmap import GridMap
    g = GridMap(hall, resolution=hall.cell)
    return "\n".join("".join("#" if v else "." for v in row) for row in (g.data == 100)[::-1])


if __name__ == "__main__":
    robot_io.serve(sys.modules[__name__])
