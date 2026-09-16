"""Where the data of this package lives — asked, never guessed.

Two layouts share one package name, exactly as they do in the simulator, and the failure mode of
guessing is identical: after `colcon build` the module sits in
`<prefix>/lib/python3.12/site-packages/ohm_localization/` and its `config/tasks_localization.json` sits in
`<prefix>/share/ohm_localization/config/`, which is not above the module at all. A source-tree path
resolved in the installed layout does not raise where the mistake is: it raises `FileNotFoundError` for a
task file three directories away, or "no such world", which reads like a broken exercise rather than like a
path. So the question "which layout am I in" is answered once, here, in the order ROS itself uses:

1. next to the source tree, when running from a checkout (that is what `./tools/run_lab.sh` and pytest do);
2. `share/ohm_localization` under each entry of `AMENT_PREFIX_PATH`, which is where an installed ROS 2 says
   it put itself.

`mecanum_lab_root()` is the same question for the *other* repository, because this one cannot run without
it: the halls and their cell sizes come from the simulator's parser, not from a copy kept here.
"""
from __future__ import annotations

import os
import sys

PACKAGE = "ohm_localization"


def _module_root(module_file: str | None = None) -> str:
    """The directory the package module sits in, one level up: the checkout, or the build tree."""
    return os.path.dirname(os.path.dirname(os.path.abspath(module_file or __file__)))


def _is_checkout(directory: str) -> bool:
    """A repository, not a build product.

    `setup.py`, `package.xml` and `config/` are not enough to tell them apart, because a colcon build tree
    under `--symlink-install` holds symlinks to precisely those three — measured here, where the build
    directory was preferred over the install prefix until this line looked for something a build never
    produces. `tools/` and `test/` are the two directories that exist in a repository and are not in
    `setup.py`'s `data_files`, so their presence is what "I am standing in the checkout" actually means.
    """
    if not os.path.isdir(os.path.join(directory, "config")):
        return False
    if not (os.path.isfile(os.path.join(directory, "setup.py"))
            or os.path.isfile(os.path.join(directory, "package.xml"))):
        return False
    return any(os.path.isdir(os.path.join(directory, d)) for d in ("tools", "test"))


def data_root_traces(module_file: str | None = None) -> list:
    """Every candidate for `data_root()`, in the order it is tried, with why it was or was not taken.

    Returned as `(directory, accepted, why)` rows rather than printed, so `install.sh --check` can show a
    student exactly which layout they are in and `test/test_packaging.py` can assert the order — a fallback
    that nobody can see is a fallback that will silently be wrong on the day it is first needed.
    """
    here = _module_root(module_file)
    rows = [(here, _is_checkout(here), "source tree (config/ plus a directory a build never copies)")]
    for var in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        for prefix in [p for p in os.environ.get(var, "").split(os.pathsep) if p]:
            # AMENT_PREFIX_PATH holds package prefixes; colcon's own COLCON_PREFIX_PATH holds the *install
            # root*, which is one level up and needs the package name back. This sandbox has a partial ROS
            # install where sourcing sets COLCON_PREFIX_PATH and leaves AMENT_PREFIX_PATH empty, which is
            # exactly the case a check that only reads the first variable loses.
            for share in (os.path.join(prefix, "share", PACKAGE),
                          os.path.join(prefix, PACKAGE, "share", PACKAGE)):
                ok = os.path.isdir(os.path.join(share, "config"))
                rows.append((share, ok, f"{var} prefix {prefix}"))
                if ok:
                    break
    rows.append((here, os.path.isdir(os.path.join(here, "config")),
                 "module directory has a config/ (a colcon build tree under --symlink-install)"))
    return rows


def data_root(module_file: str | None = None) -> str:
    """Directory holding this package's `config/`, `launch/`, `solution/` and `student/`.

    Order: the checkout if we are in one (that is what `./tools/run_lab.sh` and pytest need); then an
    installed `share/ohm_localization` named by `AMENT_PREFIX_PATH` or by colcon's own `COLCON_PREFIX_PATH`;
    and only then the module's own directory, which under `--symlink-install` is a build tree whose `config/`
    is a symlink back to the source — the right files, reached the long way, which is why it is last.
    """
    for directory, accepted, _why in data_root_traces(module_file):
        if accepted:
            return directory
    return _module_root(module_file)      # nothing found: name the source tree, where the message reads well


def task_file(explicit: str | None = None) -> str:
    """`config/tasks_localization.json`, in whichever layout this run is in.

    Overridable by `OHM_TASKS`, because the exercise is also graded from a copied task file (a group's own
    variant, or a supervisor's tighter thresholds) and that should be an environment variable rather than an
    edit to a launcher.
    """
    return explicit or os.environ.get("OHM_TASKS") or os.path.join(data_root(), "config",
                                                                   "tasks_localization.json")


def mecanum_lab_root(explicit: str | None = None) -> str:
    """The simulator's data root: its checkout if there is one, its install prefix if there is not.

    The order is deliberately *not* "import it and ask", because in the layout everyone builds —
    `colcon build` over both repositories, then `source install/setup.bash` — `mecanum_lab` is importable
    and `types.data_root()` answers correctly, while in the layout everyone *develops* in, the checkout is on
    `PYTHONPATH` via `tools/run_lab.sh` and `worlds/` sits next to the module. Both answer; the checkout wins
    only because its message is the one a student can act on. Returns "" when the simulator is neither, and
    the caller decides whether that is fatal (`load_hall` is not, `tools/drive_check.py` is).
    """
    if explicit:
        return explicit
    given = os.environ.get("MECANUM_LAB")
    if given and os.path.isdir(os.path.join(given, "worlds")):
        return given
    here = _module_root()                                         # the sibling of this repository
    for candidate in (here, os.path.join(os.path.expanduser("~"), "git", "mecanum-lab")):
        if os.path.isdir(os.path.join(candidate, "worlds")):
            return candidate
    try:                                                          # installed: ask the package itself
        from mecanum_lab import types
    except Exception:
        return ""                                                 # not importable either: the caller decides
    root = types.data_root()          # it knows its own two layouts; see its docstring
    return root if os.path.isdir(os.path.join(root, "worlds")) else ""


def add_to_path(*directories: str) -> None:
    """Put the simulator checkout on `sys.path` if it is a checkout and not already there.

    Appended, never prepended, whenever ROS is already providing the package: a `PYTHONPATH` that shadows an
    installed `mecanum_lab` with an older checkout is the kind of failure that shows up as a simulator bug.
    `tools/run_lab.sh` puts the checkout in front on purpose (it *is* the checkout workflow); this function
    is for the library, which has no opinion and should not break the other package.
    """
    for d in directories:
        if d and os.path.isdir(d) and d not in sys.path:
            sys.path.append(d)
