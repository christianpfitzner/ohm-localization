#!/usr/bin/env bash
# What this machine needs before `./tools/run_lab.sh grade …` does anything.

# Checking is the interesting half, because on the four machines that matter this exercise fails in one of
# exactly two ways, and neither is "ROS is missing". Either the simulator checkout is not where this repo
# looks for it (and then the map cannot be loaded and every tool says so in its own dialect, which is
# annoying), or pygame is absent from a system-python install (and then the simulator refuses to start, in
# a message about surfaces, three layers away from the cause). Everything else is numpy and a task file.
#
# Nothing installs by itself: --pip prints one command and only --yes runs it. sudo is never called on a
# check run. The default mode is --check.
#
# Exit codes, so a wrapper can tell the classes apart; the most fundamental problem wins, because that is
# the only one worth fixing first:
#   0  ready               3  ROS 2 asked for and not usable (optional here: the stub bus needs no ROS)
#   2  bad option          4  the task file is not readable / has no tasks
#   5  python side (numpy, pytest, pygame)
#   6  the simulator checkout missing, or too old to grade a `kind: "kf"` task
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$here"
export PYGAME_HIDE_SUPPORT_PROMPT=1        # pygame otherwise thanks you loudly on every import

usage() {
  cat <<'TEXT'
usage: ./install.sh [option]...

  --check         look and explain, install nothing (also the default)
  --pip           print the pip command for requirements.txt; with --yes, run it
  --yes           actually run what --pip asks for
  --test          after checking, run tools/check.sh (offline suite, ~40 s)
  --sim-dir=PATH  where the mecanum-lab checkout is (same as MECANUM_LAB=…, default: sibling or ~/git)
  --build         colcon build this package in place (--symlink-install), into ./build ./install ./log
  --workspace     colcon build the whole practicum: the simulator, its interfaces, this package,
                  and ohm_frontier if it is next door. Same three directories, one workspace.
  --ros           with --check: also report whether a ROS 2 install and a colcon are usable at all

  Building is optional. `./tools/run_lab.sh grade …` needs no colcon and no ROS at all; `ros2 launch
  ohm_localization mcl.launch.py` needs both. After a build: source install/setup.bash.

  Exit: 0 ready · 2 bad option · 3 ROS asked for and unusable · 4 task file · 5 python side · 6 simulator
TEXT
}

mode=check; want_ros=0; yes=0; run_tests=0; sim_dir="${MECANUM_LAB:-}"
for a in "$@"; do
  case "$a" in
    --check) mode=check ;;
    --pip) mode=pip ;;
    --yes) yes=1 ;;
    --test) run_tests=1 ;;
    --with-ros|--ros) want_ros=1 ;;
    --build) mode=build ;;
    --workspace) mode=workspace ;;
    --sim-dir=*) sim_dir="${a#*=}" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $a" >&2; usage >&2; exit 2 ;;
  esac
done

# --------------------------------------------------------------------------------- the colcon part
# Two ways this repository is used, and only one of them needs a build: the graded path runs everything in
# one process on the simulator's stub bus (`./tools/run_lab.sh`), and the ROS path (`ros2 launch
# ohm_localization mcl.launch.py`) needs an installed package. Both must work, because the lecture's own
# install script installs ROS and the laboratory's `./lab` is a colcon-built package — a localisation
# exercise that cannot be built is an exercise that cannot be run the way the rest of the practicum is run.
#
# `--workspace` builds four things in one directory, in the order they depend on:
#   mecanum_lab_interfaces   the messages (a second path on purpose: it is built first and its absence must
#                            not stop the pure-python simulator from building on a machine without rosidl)
#   mecanum-lab              the simulator
#   ohm-localization         this package
#   ohm_frontier             the navigation exercise, when it is checked out next door
colcon_paths() {                    # $1 = self | workspace ; $2 = ifaces | python
  local list=("$here")
  if [ "$1" = workspace ]; then
    [ -n "$sim_dir" ] || sim_dir="$(PYTHONPATH="$here" python3 -c 'from ohm_localization.gridmap import mecanum_lab_dir; print(mecanum_lab_dir())' 2>/dev/null)"
    if [ -n "$sim_dir" ]; then
      list=("$sim_dir" "${list[@]}")
      if [ "${2:-python}" = ifaces ] && [ -d "$sim_dir/interfaces/mecanum_lab_interfaces" ]; then
        list=("$sim_dir/interfaces/mecanum_lab_interfaces")
      fi
    fi
    if [ "${2:-python}" != ifaces ]; then
      for neighbour in "$here/../ohm-nav-exploration/ohm_frontier" "$HOME/git/ohm-nav-exploration/ohm_frontier"; do
        # resolved and de-duplicated: when the repo lives under ~/git the two candidates are the same
        # directory through different spellings, and colcon listing a package twice is a warning a
        # student has to read every time for no information at all.
        [ -f "$neighbour/package.xml" ] || continue
        neighbour="$(readlink -f "$neighbour")"
        local seen=0 other
        for other in "${list[@]}"; do [ "$(readlink -f "$other")" = "$neighbour" ] && seen=1; done
        [ "$seen" = 0 ] && list=("${list[@]}" "$neighbour")
      done
    fi
  fi
  printf '%s\n' "${list[@]}"
}

colcon_pass() {                     # $1 = self | workspace | ifaces
  local paths=(); while IFS= read -r line; do paths+=("$line"); done < <(colcon_paths "$2" "$1")
  echo "  colcon build --paths ${paths[*]}"
  colcon build --symlink-install --paths "${paths[@]}"
}

do_build() {
  local setup=""
  command -v colcon >/dev/null 2>&1 || command -v colcon >/dev/null 2>&1 || {
    bad "colcon not on PATH (sudo apt install python3-colcon-common-extensions, or ~/.local/bin/colcon)"
    last 3; return 1; }
  set +u
  for f in "${ROS_SETUP:-}" "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" /opt/ros/jazzy/setup.bash \
           /opt/ros/kilted/setup.bash /opt/ros/humble/setup.bash; do
    if [ -n "$f" ] && [ -f "$f" ]; then setup="$f"; break; fi
  done
  set -u
  if [ -z "$setup" ]; then
    bad "no ROS 2 setup.bash to source (looked in \$ROS_SETUP, /opt/ros/\$ROS_DISTRO, jazzy, kilted, humble)"
    last 3; return 1
  fi
  echo "building with $setup"
  echo "------------------------------------------------------------------"
  set +u
  # shellcheck disable=SC1090
  source "$setup" || { bad "sourcing $setup failed"; last 3; return 1; }
  set -u

  # The message package goes through a build of its own, the way the simulator's own install.sh does it.
  # One colcon invocation over all four paths is wrong twice over: `mecanum_lab_interfaces` needs
  # `rosidl_default_generators` (absent from a ros-base install), and when it fails in a shared invocation
  # colcon aborts the packages behind it — measured here, where a sandbox with a partial /opt/ros built
  # *nothing* until the interfaces went into a pass of their own. The simulator itself is pure python and
  # does not need the messages to be built to run its stub-bus mode, so the second pass must happen either way.
  local status=0
  if [ "$1" = workspace ]; then
    echo "pass 1/2: the message package (may fail on a partial ROS install; the lab does not need it)"
    if colcon_pass ifaces workspace; then
      ok "mecanum_lab_interfaces built"
    else
      warn "interfaces not built (needs rosidl_default_generators, absent from a ros-base install). That is"
      echo "           survivable: every graded task in this repository runs on the stub bus without any"
      echo "           message package at all — only the RViz/nav2 side of a ROS run cares about it."
    fi
    echo "pass 2/2: the python packages"
    colcon_pass self workspace || status=1
  else
    colcon_pass self self || status=1
  fi
  if [ "$status" = 0 ]; then
    ok "colcon build finished — source $here/install/setup.bash, then: ros2 launch ohm_localization mcl.launch.py"
    return 0
  fi
  bad "colcon build failed — the report is in ./log/latest_log; a package that cannot find rosidl or"
  echo "           nav_msgs is usually a partial ROS install (ros-base instead of desktop), not this repo"
  last 3; return 1
}

problems=0
ok()   { printf '  \033[32mok\033[0m      %s\n' "$1"; }
warn() { printf '  \033[33mnote\033[0m     %s\n' "$1"; }
bad()  { printf '  \033[31mMISSING\033[0m  %s\n' "$1"; problems=1; }
last() { [ "$1" -gt "$problems" ] && problems=$1; return 0; }

echo "ohm-localization — environment"
echo "------------------------------------------------------------------"

# ---------------------------------------------------------------------------------------- python side
if python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3,10) else 1)' 2>/dev/null; then
  ok "python3 $(python3 -c 'import platform;print(platform.python_version())')"
else
  bad "python3 ≥ 3.10 (the code uses `X | None` annotations and match-free but modern dict syntax)"; last 5
fi
if python3 -c 'import numpy' 2>/dev/null; then
  ok "numpy $(python3 -c 'import numpy;print(numpy.__version__)')"
else
  bad "numpy — the whole library is numpy: nothing works without it"; last 5
fi
if python3 -c 'import pytest' 2>/dev/null; then
  ok "pytest $(python3 -c 'import pytest;print(pytest.__version__)')"
else
  warn "pytest missing — only ./tools/check.sh and the tests need it (sudo apt install python3-pytest)"
fi
if python3 -c 'import pygame' 2>/dev/null; then
  ok "pygame $(python3 -c 'import pygame;print(pygame.version.ver)' 2>/dev/null | head -1)"
else
  bad "pygame — the simulator's engine imports it at start-up even for --headless (sudo apt install python3-pygame)"; last 5
fi
if python3 -c 'import scipy' 2>/dev/null; then
  ok "scipy present — icp.py will use a cKDTree and say that it did"
else
  warn "scipy absent: ICP nearest neighbours run brute force (130k distances per pair, nothing), and one test skips"
fi
if [ "$mode" = pip ]; then
  cmd=(python3 -m pip install --user -r "$here/requirements.txt")
  echo; echo "  ${cmd[*]}"
  if [ "$yes" = 1 ]; then "${cmd[@]}"; else echo "  (not run: add --yes)"; fi
fi

# ------------------------------------------------------------------------ the simulator (needed, not built)
if [ -z "$sim_dir" ]; then
  sim_dir="$(python3 -c 'import sys;sys.path.insert(0,"'"$here"'")
from ohm_localization.gridmap import mecanum_lab_dir;print(mecanum_lab_dir() or "")')"
fi
if [ -z "$sim_dir" ] || [ ! -d "$sim_dir/worlds" ]; then
  bad "mecanum-lab checkout not found (looked at MECANUM_LAB, the sibling of this repo, ~/git).
         It is not a pip package and not an ROS package to build — it is a checkout:
             git clone <course-url>/mecanum-lab.git ~/git/mecanum-lab
         or point us at it:  MECANUM_LAB=/path/to/mecanum-lab ./install.sh --check"; last 6
else
  ok "simulator checkout $sim_dir"
  for w in production arena; do
    [ -f "$sim_dir/worlds/$w.txt" ] && ok "world '$w' (the graded hall, and the open one L4 was almost in)" \
      || { bad "worlds/$w.txt is not in that checkout"; last 6; }
  done
  if grep -q '"kf"' "$sim_dir/mecanum_lab/grade.py" 2>/dev/null || \
     grep -rq "kind.*kf\|_check_kf" "$sim_dir/mecanum_lab/" 2>/dev/null; then
    ok "grade.py knows the 'kf' task family (rmse, improvement, NEES, rate, contacts)"
  else
    bad "that checkout is too old: no 'kf' estimator family in mecanum_lab/grade.py, which is what these
         four tasks are graded with. Update the simulator checkout."; last 6
  fi
  if python3 -c "import sys;sys.path.insert(0,'$sim_dir');import mecanum_lab.node,mecanum_lab.tasks" 2>/dev/null; then
    ok "mecanum_lab imports (node, tasks) under PYTHONPATH from tools/run_lab.sh"
  else
    bad "mecanum_lab is on disk but does not import — look at the traceback above, it is usually a missing
         dependency of the simulator itself rather than of this repo"; last 6
  fi
fi

# ------------------------------------------------------------------------------------- the task file
if python3 - "$here/config/tasks_localization.json" <<'PY'
import json, sys
cfg = json.load(open(sys.argv[1], encoding="utf-8"))
tasks = {t["id"]: t for t in cfg["tasks"]}
order = cfg.get("order") or list(tasks)
assert order == list(tasks), f"order {order} does not match the tasks {list(tasks)}"
for t in tasks.values():
    for key in ("world", "drive", "rmse_max", "improvement_min", "max_error_max",
                "rate_min", "contacts_max", "nees", "text", "checks", "mcl", "sim"):
        assert key in t, f"{t['id']}: no {key!r}"
    assert 0.1 <= t["nees"][0] < t["nees"][1], f"{t['id']}: NEES band {t['nees']}"
    assert t["kind"] == "kf" and t["sensor"] == "odom", f"{t['id']}: must be a kf task graded against odom"
    assert t["sim"].get("debug_truth") is True, f"{t['id']}: the grader needs debug_truth"
print(f"  ok      task file: {len(tasks)} tasks, {sum(t['points'] for t in tasks.values())} points, "
      f"all kind=kf against sensor=odom")
PY
then :; else bad "config/tasks_localization.json is not what the launcher expects"; last 4; fi

# ----------------------------------------------------------------------------------- ROS (optional here)
if [ "$want_ros" = 1 ] || [ -n "${ROS_DISTRO:-}" ] || ls /opt/ros >/dev/null 2>&1; then
  if [ -n "${ROS_DISTRO:-}" ]; then ok "ROS 2 '$ROS_DISTRO' sourced"; elif ls /opt/ros/jazzy >/dev/null 2>&1; then
    ok "ROS 2 jazzy under /opt/ros (LAB-CONCEPT's pin) — not sourced; MECANUM_ROS=0 will source it"
  elif ls /opt/ros >/dev/null 2>&1; then warn "ROS under /opt/ros: $(ls /opt/ros | tr '\n' ' ')"
  else warn "no ROS 2 found. Everything in the default path runs on the in-process stub bus (MECANUM_ROS=stub),
         which is what the exercise is designed around; real DDS is a bonus, not a requirement"; fi
fi

# ---------------------------------------------------------------------------- build, when it was asked for
if [ "$mode" = build ] || [ "$mode" = workspace ]; then
  echo "------------------------------------------------------------------"
  build_status=0
  do_build "$mode" || build_status=$problems
  [ "$run_tests" = 1 ] && [ "$problems" = 0 ] && ./tools/check.sh
  exit "$problems"
fi

echo "------------------------------------------------------------------"
if [ "$problems" = 0 ]; then
  echo "ready. The three commands worth running first:"
  echo "    ./tools/check.sh                                    # suite + task check + ICP claims (~40 s)"
  echo "    ./tools/run_lab.sh grade --task mcl_production \\
    --controller solution/mcl_node.py --headless   # the real grader, ~36 s, expect PASS 30/30"
else
  echo "not ready — see MISSING above. Exit code $problems tells you which class of thing is wrong."
fi
[ "$run_tests" = 1 ] && [ "$problems" = 0 ] && ./tools/check.sh
exit "$problems"
