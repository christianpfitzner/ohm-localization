"""The simulator's own launcher, on this exercise's task file. Same commands as `./lab`.

    python3 -m ohm_localization.lab grade --task mcl_production --controller solution/mcl_node.py --headless
    python3 -m ohm_localization.lab run   --world production --task mcl_wide --controller student/mcl_template.py --truth
    python3 -m ohm_localization.lab sim   --world production --headless        # drive it yourself

`mecanum_lab.grade.Grader` takes its task dictionary as an argument, so a new exercise needs nothing from the
simulator. `node.py` — the file that owns the window, the physics clock, the controller threads, the sensor
profiles and the report — does not: it calls `tasks.load_tasks()` with no argument, and `tasks.TASKS_PATH` is
a module constant pointing at the simulator's own `config/tasks.json`. So this module changes exactly one
thing about that run and says so out loud:

    tasks.TASKS_PATH = <this package's data root>/config/tasks_localization.json

Everything else — engine, physics, grader state machine, thresholds applied, report printed — is the
laboratory's code, unmodified, on its own clock. The alternative was to add four tasks to the simulator's
`config/tasks.json`, which is another repository's file and the one its own LaTeX handouts read for the same
numbers; a task that is not theirs to grade does not belong in it, and a fork of the simulator to hold my
tasks would be a simulator that stops receiving their fixes.

Two more things this file does that a `python -m mecanum_lab.node` line does not:

* **finds the simulator**, as a checkout or as an installed package (`paths.mecanum_lab_root()`), and puts it
  on the path *behind* whatever is already there;
* **resolves `--controller` against this repository**: the laboratory resolves a relative path against its
  own checkout, so `solution/mcl_node.py` would otherwise name a file in the wrong project and the failure
  would arrive as a traceback about an import, in a process whose stderr the launch file interleaves with
  the simulator's.

`tools/lab_grade.py` is the shell-facing shim for this module (it is what `tools/run_lab.sh` calls, because
that script also sets `PYTHONPATH` and swallows the Pygame banner); `ros2 launch ohm_localization
mcl.launch.py` starts the two halves of a ROS run through here.
"""
from __future__ import annotations

import os
import sys


def _simulator() -> str:
    from ohm_localization import paths
    return paths.mecanum_lab_root()


def _resolve_controllers(argv: list) -> list:
    """Relative `--controller` paths belong to this repo, not to the simulator's."""
    from ohm_localization import paths
    root = paths.data_root()
    out = []
    for i, arg in enumerate(argv):
        if argv[i - 1] == "--controller" and not os.path.isabs(arg):
            arg = arg if os.path.isfile(arg) else os.path.join(root, arg)
        out.append(arg)
    return out


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__.split("Two more things")[0].strip())
        print("\nCommands: grade | run | sim | controller | spawn "
              "(see the simulator's ./lab -h for the options)")
        return 0

    sim = _simulator()
    if sim:
        from ohm_localization import paths
        paths.add_to_path(sim)                      # appended: never shadow an installed mecanum_lab
    argv = _resolve_controllers(argv)

    from ohm_localization import paths
    try:
        from mecanum_lab import tasks as T
        from mecanum_lab import node
    except Exception as exc:                         # noqa: BLE001 - the message is the whole point
        print(f"ohm_localization.lab: cannot import the simulator ({type(exc).__name__}: {exc}).\n"
              "It is a separate repository: either build it in this workspace (install.sh --workspace)\n"
              "and source the result, or point MECANUM_LAB at a checkout "
              "(install.sh --check says what it can see).", file=sys.stderr)
        return 2

    task_file = paths.task_file()
    if os.path.abspath(T.TASKS_PATH) != os.path.abspath(task_file):
        print(f"lab: tasks from {task_file}\nlab: simulator {sim} (its own config/tasks.json is not read)",
              file=sys.stderr)
    T.TASKS_PATH = task_file                        # the one change, see the module docstring
    return node.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
