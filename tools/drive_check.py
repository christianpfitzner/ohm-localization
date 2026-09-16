#!/usr/bin/env python3
"""Is a task's commanded drive driveable, and is the hall worth localising in? Offline, in a second.

    python3 tools/drive_check.py                        # every task in our own task file
    python3 tools/drive_check.py mcl_wide --clearance 0.4

A `kind: "kf"` task states the drive the grader will command, and two things can be wrong with it in a
way no unit test can see.  It can hit a wall — which ends the run on `contacts_max` with the filter
innocent, after 48 s of grading.  Or it can be driveable in a hall where the LIDAR has nothing to say:
`arena` is the world the Kalman-filter experiment uses, and it is a *bad* world for this one, because
from its middle the walls are 8 to 17 m away and only about a quarter of the 360 beams find anything
at all.  Both are properties of the task file and the world text, both are checkable without driving,
and the second one is the difference between an exercise that converges to a centimetre and one that
wanders at 400 mm wondering why.

So this prints, per task: the minimum clearance along the commanded path (with the robot's own radius
as the limit), whether the path leaves the hall, and — per scan along it — how many beams would echo
within `range_max` and what that implies for a filter that uses one beam in three.

It reads the world file through the simulator (`mecanum_lab.worlds`), so the spawn pose and the walls
are the ones the run will use, and the map through this repo's `parse_grid`, so the geometry is the one
the filter will use.  If the two ever disagree, `tools/scan_probe.py` is the arbiter.
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from ohm_localization import synth                                          # noqa: E402
from ohm_localization.gridmap import load_map, mecanum_lab_dir   # noqa: E402

TASK_FILE = os.path.join(ROOT, "config", "tasks_localization.json")
ROBOT_RADIUS = 0.30        # m, chassis corner to the LIDAR at the centre: what must stay free
RANGE_MAX = 8.0            # m, the simulator's LIDAR


def spawn_of(world: str, index: int = 0) -> tuple:
    """The pose the grader will release the robot at — from the simulator's own world loader."""
    sim = mecanum_lab_dir()
    if not sim:
        raise SystemExit("drive_check needs the simulator checkout for the spawn poses "
                         "(set MECANUM_LAB=…)")
    sys.path.insert(0, sim)
    from mecanum_lab import worlds
    w = worlds.load_world(world)
    p = w.spawns[min(index, len(w.spawns) - 1)]
    return (float(p.x), float(p.y), float(p.theta))


def informativeness(grid, path, beams: int = 360, stride: int = 3):
    """Echoing beams per scan along the path — how much the hall actually says about the robot."""
    hall = grid.hall
    echo, used = [], []
    for pose in path[::max(len(path) // 60, 1)]:
        ranges = synth.cast(pose, hall, beams=beams, range_max=RANGE_MAX)
        e = int(np.count_nonzero(np.isfinite(ranges) & (ranges < RANGE_MAX - 0.06)))
        idx = np.arange(0, beams, max(stride, 1))
        keep = np.isfinite(ranges[idx]) & (ranges[idx] < RANGE_MAX - 0.06)
        echo.append(e)
        used.append(int(keep.sum()))
    return float(np.mean(echo)), float(np.mean(used)), float(np.min(echo))


def check(task: dict, clearance: float, stride: int):
    world = str(task.get("world", "arena"))
    grid = load_map(world)
    start = spawn_of(world)
    drive = task.get("drive") or []
    path = synth.path_from_drive(drive, start=start)
    d = grid.distance_at(path[:, 0], path[:, 1])
    inside = ~grid.occupied_at(path[:, 0], path[:, 1])
    length = float(np.sum(np.hypot(np.diff(path[:, 0]), np.diff(path[:, 1]))))
    echo, used, worst = informativeness(grid, path, stride=stride)
    dur = sum(float(b.get("duration", 0.0)) for b in drive)
    print(f"{task['id']:12s} {world:11s} spawn {tuple(round(v, 2) for v in start)}  "
          f"{dur:4.0f} s, {length:5.1f} m commanded")
    print(f"             clearance min {d.min():5.2f} m (robot needs {ROBOT_RADIUS:.2f}), "
          f"{'STAYS' if inside.all() else 'LEAVES THE HALL'}"
          f"{'' if inside.all() else ' at ' + str(np.argmin(~inside) * 0.05) + ' s'}")
    fits = d.min() > ROBOT_RADIUS + clearance and inside.all()
    note = "" if echo >= 0.35 * 360 else (
        f"  — an open hall: {echo:.0f} echoing beams is what L4 is *about*, and what an accidental "
        f"choice here would make mysterious")
    print(f"             beams echoing: mean {echo:4.0f}/360, worst {worst:3.0f}, "
          f"one in {stride} → {used:4.0f} per update."
          f"{'' if fits else '   DRIVE DOES NOT FIT'}{note}")
    #  Only the drive can fail the check.  How informative a hall is is a fact the author should see
    #  and may well have chosen on purpose — a task may sit in an open hall precisely so that the
    #  difference is visible — so it is printed, not enforced.
    return fits


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("task", nargs="?", help="task id (default: all in the task file)")
    ap.add_argument("--tasks", default=TASK_FILE)
    ap.add_argument("--clearance", type=float, default=0.10,
                    help="extra margin beyond the robot radius, m — slip is not in this model")
    ap.add_argument("--stride", type=int, default=3)
    args = ap.parse_args(argv)
    with open(args.tasks, encoding="utf-8") as fh:
        cfg = json.load(fh)
    todo = [t for t in cfg["tasks"] if args.task is None or t["id"] == args.task]
    if not todo:
        print(f"no task '{args.task}' in {args.tasks}", file=sys.stderr)
        return 2
    ok = all([check(t, args.clearance, args.stride) for t in todo])
    print("\nall task drives fit and their halls say something" if ok
          else "\nat least one task is not driveable as written, or its hall is too open for a LIDAR")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
