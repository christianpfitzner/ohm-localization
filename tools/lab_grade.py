#!/usr/bin/env python3
"""Shell-facing shim for `ohm_localization.lab` — the simulator's launcher on this repo's task file.

    tools/run_lab.sh grade --task mcl_production --controller solution/mcl_node.py
    tools/run_lab.sh run --world production --task mcl_wide --controller student/mcl_template.py --truth

The implementation is `ohm_localization/lab.py`, which is also what `ros2 launch ohm_localization
mcl.launch.py` runs, so the graded path and the ROS path go through the same three lines of task-file
surgery. This file exists because `tools/run_lab.sh` needs something to exec that works before this package
is installed or even importable-by-name, and because the laboratory's `./lab` spelling — a script in the
project root that knows where its own repository is — is what the students already type in the other labs.
See `ohm_localization/lab.py` for why one module constant is the only thing that had to change.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ohm_localization import lab                                        # noqa: E402

if __name__ == "__main__":
    raise SystemExit(lab.main())
