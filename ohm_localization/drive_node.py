#!/usr/bin/env python3
"""Drive the commanded path of a task, so a ROS run has something to localise.

    ros2 run ohm_localization drive_node --task mcl_production --robot alice

The `kind: "kf"` tasks are driven by the grader, so a `ros2 launch` without `grade:=` would show a
robot standing at its spawn pose for the whole run. This node publishes the same segments — read from
the same `config/tasks_localization.json` — onto `/<robot>/cmd_vel`, which is the drive the thresholds
were calibrated on. `launch/mcl.launch.py` starts it unless a grader is already driving.
"""
import json
import os
import sys
import time

from ohm_localization import paths

DT = 0.05          # s between two commands: 20 Hz, the rate the drive was measured at


def segments(task_id: str) -> tuple:
    """The task's command segments and how often to repeat them: ([{vx, vy, omega, duration}], n)."""
    try:
        with open(paths.task_file(), encoding="utf-8") as fh:
            cfg = json.load(fh)
    except OSError:
        return [], 1
    task = next((t for t in cfg.get("tasks", []) if t.get("id") == task_id), {})
    return list(task.get("drive") or []), max(int(task.get("repeat", 1)), 1)


def mission(rob, task: str) -> None:
    """One segment at a time, for its own duration, then stand still."""
    from ohm_localization.mcl_node import spin_or_stop
    drive, times = segments(task or os.environ.get("OHM_DRIVE_TASK", ""))
    if not drive:
        print(f"drive_node: task {task!r} has no drive to play", file=sys.stderr)
        return
    seconds = sum(float(s["duration"]) for s in drive) * times
    print(f"drive_node: {task} — {len(drive)} segments, {seconds:.0f} s of driving", file=sys.stderr)
    for _ in range(times):
        for seg in drive:
            until = time.monotonic() + float(seg["duration"])
            while rob.running() and time.monotonic() < until:
                try:
                    rob.publish_cmd_vel(float(seg.get("vx", 0.0)), float(seg.get("vy", 0.0)),
                                        float(seg.get("omega", 0.0)))
                except Exception:               # `ros2 launch` shut the graph down under us
                    return
                if not spin_or_stop(rob, DT):
                    return
            if not rob.running():
                return
    try:
        rob.publish_cmd_vel(0.0, 0.0, 0.0)
    except Exception:
        pass
    print("drive_node: drive played — standing still", file=sys.stderr)


def main() -> None:
    """`ros2 run ohm_localization drive_node`, and the controller door of a graded run."""
    from mecanum_lab import robot_io                 # here, not at import: see mcl_node.py
    try:
        robot_io.serve(sys.modules[__name__])
    except KeyboardInterrupt:
        print("drive_node: stopped", file=sys.stderr)
    except Exception as exc:                          # noqa: BLE001 - and only for a graph that is really gone
        # The same shutdown as in mcl_node.main(): the runner's last `mission_state` publish lands in a
        # context the launch has already closed. Saying "over then" instead of raising is what keeps the
        # launch's last lines free of `exit code 1` for a drive that was driven perfectly.
        rclpy = sys.modules.get("rclpy")
        if rclpy is None or rclpy.ok():
            raise
        print(f"drive_node: the bus closed before the last status message ({type(exc).__name__})",
              file=sys.stderr)


if __name__ == "__main__":
    main()
