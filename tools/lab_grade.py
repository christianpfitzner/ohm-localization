#!/usr/bin/env python3
"""Run the simulator's own grader against **this** repo's task file.

Why a launcher at all: `mecanum_lab.grade.Grader` takes its task dictionary as an argument, so a new
exercise needs nothing from the simulator — but `node.py`, which owns the window, the controller
thread, the sensor profile and the report, calls `tasks.load_tasks()` with no argument, and
`tasks.TASKS_PATH` is a module constant pointing at the simulator's own `config/tasks.json`.  So this
file changes exactly one thing about that run, and says so out loud:

    tasks.TASKS_PATH = <this repo>/config/tasks_localization.json

Everything else — engine, physics, grader state machine, thresholds applied, report printed — is the
laboratory's code, unmodified, running on its own clock.  The alternative was adding three tasks to
the simulator's `config/tasks.json`, which is another repository's file and the one its LaTeX handouts
read for the same numbers; a task that is not theirs to grade does not belong in it.

Usage (through tools/run_lab.sh, which sets PYTHONPATH and the bus):

    tools/run_lab.sh grade --task mcl_arena --controller solution/mcl_node.py
    tools/run_lab.sh grade --task mcl_wide,mcl_budget --controller solution/mcl_node.py --json b.json
    tools/run_lab.sh run --world arena --task mcl_arena --robot alice \\
                             --controller solution/mcl_node.py --truth
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TASK_FILE = os.path.join(ROOT, "config", "tasks_localization.json")
sys.path.insert(0, ROOT)

from ohm_localization.gridmap import mecanum_lab_dir              # noqa: E402

SIM = mecanum_lab_dir()


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__.split("Usage (through")[0].strip())
        print("\nCommands: grade | run | sim   (see the simulator's ./lab -h for the options)")
        return 0
    if not SIM:
        print("mecanum-lab not found.  Set MECANUM_LAB=/path/to/mecanum-lab or run install.sh --check",
              file=sys.stderr)
        return 2
    sys.path.insert(0, SIM)

    # A relative --controller is relative to *this* repo, not to the simulator's: the laboratory's
    # own launcher resolves it against its own checkout, and `solution/mcl_node.py` would otherwise
    # mean a file that does not exist in the other repository.
    for i, arg in enumerate(argv):
        if arg == "--controller" and i + 1 < len(argv) and not os.path.isabs(argv[i + 1]):
            argv[i + 1] = os.path.join(ROOT, argv[i + 1])

    from mecanum_lab import tasks as T
    print(f"lab_grade: tasks from {TASK_FILE}\nlab_grade: simulator {SIM} "
          f"(its own config/tasks.json is not read)", file=sys.stderr)
    T.TASKS_PATH = TASK_FILE                      # the one change, see the module docstring
    from mecanum_lab import node
    return node.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
