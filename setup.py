"""The localisation exercise as a ROS 2 package: `colcon build --paths . --symlink-install`.

Two ways to use this repository and the build has to be right for both.

*In a workspace* — `src/ohm-localization` beside `src/mecanum-lab`, `colcon build --symlink-install`, then
`source install/setup.bash` — the package is what makes `ros2 launch ohm_localization mcl.launch.py` and
`ros2 run ohm_localization mcl_node` work from any directory, and `install.sh --workspace` builds the three
repositories in the order they need.

*From the checkout* — `./tools/run_lab.sh grade …` — no build at all, no ROS at all: the graded path runs the
simulator and the controller in one process on the in-process bus, which is how the thresholds in
`config/tasks_localization.json` were calibrated. That is also why `solution/` and `student/` are installed as
**data files rather than as modules**: they are not imported by name, they are handed to the simulator's node
loader as a *file path* (`--controller student/mcl_template.py`), and installing them under
`share/ohm_localization/` is what lets a launch file find a controller after a colcon build, when the
checkout may not be on the path at all.

`data_files` is therefore the part to read. An installed package has no `config/` next to its module — the
task file, the launch files, the solution and the student template live in
`<prefix>/share/ohm_localization/` — and `ohm_localization/paths.py` is what asks AMENT_PREFIX_PATH where
things went instead of guessing from `__file__`. `test/test_packaging.py` builds a fake install prefix and
checks that branch, because a fallback nobody can reach is a fallback nobody sees break.

The `tools/` directory is deliberately not installed. Those are the measurement instruments of the exercise
(`mcl_report.py`, `icp_eval.py`, `drive_check.py`, `scan_probe.py`) and they are run from a checkout by
everybody who uses them, including the documentation; an installed copy would be a second copy to keep in
sync and would change which files a student's protocol quotes.
"""
from glob import glob
import os

from setuptools import setup

PACKAGE = "ohm_localization"

setup(
    name=PACKAGE,
    version="0.1.0",
    description="Monte-Carlo localization and ICP scan matching for the mecanum-lab simulator",
    author="Praktikum Intelligente Robotik",
    license="MIT",
    python_requires=">=3.10",
    packages=[PACKAGE],
    # The ament resource index marker first, as in the two sibling packages of this workspace: `ros2 pkg
    # list` reads that index, and without it a built package is invisible to `ros2 launch <pkg> …`.
    data_files=[
        (os.path.join("share", "ament_index", "resource_index", "packages"), ["resource/" + PACKAGE]),
        (os.path.join("share", PACKAGE), ["package.xml"]),
        (os.path.join("share", PACKAGE, "launch"), glob("launch/*.py")),
        (os.path.join("share", PACKAGE, "config"), glob("config/*.json")),
        # Controllers by path, not by module: see the docstring. `student/*.py` is what a group edits,
        # `solution/*.py` is the reference that the thresholds were measured against.
        # The RViz view of the exercise (`launch/mcl.rviz`) is data too: `rviz_config.render()` reads it back
        # out of the install prefix, so a `ros2 launch` from an installed workspace shows the same window a
        # checkout does. `ohm_localization/rviz_config.py` names the file, this line is why it exists there.
        (os.path.join("share", PACKAGE, "launch"), glob("launch/*.rviz")),
        (os.path.join("share", PACKAGE, "solution"), glob("solution/*.py")),
        (os.path.join("share", PACKAGE, "student"), glob("student/*.py")),
        (os.path.join("share", PACKAGE, "docs"), glob("docs/*.md")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    # Every name here is a name a launch file can call with `executable=`. test/test_packaging.py compares
    # the two lists, because a launch file that names an executable nobody installs fails at launch time
    # with an error that mentions a missing file, three layers away from this list being wrong.
    entry_points={"console_scripts": [
        "ohm-lab = ohm_localization.lab:main",                # ./lab, with this repo's task file
        "mcl_node = ohm_localization.mcl_node:main",
        "drive_node = ohm_localization.drive_node:main",      # the commanded drive of a task
        "map_server = ohm_localization.map_server_node:main",
        "icp_odom_node = ohm_localization.icp_odom_node:main",
    ]},
    zip_safe=False,
)
