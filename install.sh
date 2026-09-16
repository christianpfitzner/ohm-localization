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
    --with-ros) want_ros=1 ;;
    --sim-dir=*) sim_dir="${a#*=}" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $a" >&2; usage >&2; exit 2 ;;
  esac
done

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

echo "------------------------------------------------------------------"
if [ "$problems" = 0 ]; then
  echo "ready. The two commands worth running first:"
  echo "    ./tools/check.sh                                    # suite + task check + ICP claims (~40 s)"
  echo "    ./tools/run_lab.sh grade --task mcl_production \\
    --controller solution/mcl_node.py --headless   # the real grader, ~36 s, expect PASS 30/30"
else
  echo "not ready — see MISSING above. Exit code $problems tells you which class of thing is wrong."
fi
[ "$run_tests" = 1 ] && [ "$problems" = 0 ] && ./tools/check.sh
exit "$problems"
