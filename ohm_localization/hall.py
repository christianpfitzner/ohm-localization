"""Which hall a run is in — asked of the simulator, and waited for.

    load_map(hall_name(rob, task))

Three nodes need this answer (`mcl_node` and `student/mcl_template.py`) and there is one rule for getting it, which is why it is here rather than in each of them: the wrong answer does not fail, it
localises beautifully against the wrong walls.

`rob.world()` — the simulator's own accessor — answers from the local config default whenever `/sim/world`
has not arrived yet, and the default is a different hall than the one a launch started (`maze`, where every
task of this exercise drives in `production`). The launch files start the simulator and the node in the same
instant, and the simulator only republishes that topic about once a second, so the node that asks first gets
`maze`, loads a 52 × 44 grid for a 20 × 12 m hall, and reports an estimate that looks plausible while being
matched against walls that are not there. Measured here: `mcl_template: GridMap(maze 52x44 …)` on a
`ros2 launch … controller:=student/mcl_template.py`, with the robot driving `production` (docs/verification.md
§16). Hence: wait for the topic, and only then fall back.

Order, most trustworthy first: `/sim/world`, then the hall the *task file* names for this task, then the
simulator's config — the last one labelled for what it is, a guess. Nothing here needs ROS: `rob` is anything
with `running()`, `spin()`, `bus.last("world")` and `config()`, which is both of the simulator's buses, so
the graded door and the ROS door run this same code and can disagree about nothing.
"""
from __future__ import annotations

import json
import time

from ohm_localization import paths

WORLD_WAIT = 6.0          # s: how long to wait for /sim/world before falling back
POLL = 0.05               # s between looks, the same cadence the node loops use


def task_world(task_id: str) -> str:
    """The hall the task file names for this task — empty if the task file or the task is not there."""
    try:
        with open(paths.task_file(), encoding="utf-8") as fh:
            cfg = json.load(fh)
        return str(next((t.get("world") or "" for t in cfg.get("tasks", [])
                         if t.get("id") == task_id), ""))
    except (OSError, ValueError, StopIteration):
        return ""


def _world_payload(rob) -> str:
    """The `name` from `/sim/world`, or "" while that message has not arrived."""
    try:
        payload = str(rob.bus.last("world")[0] or "")
    except Exception:                                 # noqa: BLE001 - no bus: nothing to ask
        return ""
    try:
        return str(json.loads(payload).get("name") or "")
    except ValueError:
        return ""                                     # the topic carries something that is not our JSON


def _spin(rob, dt: float) -> bool:
    """Spin, and whether to keep waiting. A graph that died mid-wait ends the wait, not the run.

    The same rule `mcl_node.spin_or_stop` exists for: a `ros2 launch` shutdown lands inside the wait set and
    rclpy raises rather than answering cleanly.
    """
    try:
        rob.spin(dt)
    except Exception:                                 # noqa: BLE001 - the run is over
        return False
    return True


def hall_name(rob, task_id: str = "", wait: float = WORLD_WAIT) -> str:
    """The hall this robot is driving in, or the best guess left — never an empty name.

    `wait` bounds the wait for `/sim/world`, and the run is not lost when it expires: the task file knows
    which hall its task was calibrated on, which is a better answer than the simulator's config default and
    needs no topic. The default is still named in the returned string's source, so a caller that wants to
    complain about it can compare against `rob.config("world")`.
    """
    deadline = time.monotonic() + float(wait)
    while rob.running() and time.monotonic() < deadline:
        name = _world_payload(rob)
        if name:
            return name
        if not _spin(rob, POLL):
            break
    return task_world(task_id) or str(rob.config("world", "arena") or "arena")
