#!/usr/bin/env bash
# The laboratory's simulator, graded against this repo's task file.
#
# Same three things ./lab does — PYTHONPATH, the pygame banner, an optional ROS sourcing — and then
# this repo's launcher instead of `python3 -m mecanum_lab.node`, so the task dictionary comes from
# config/tasks_localization.json.  See tools/lab_grade.py for why one module constant is the only
# thing that had to change.
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

sim="${MECANUM_LAB:-}"
if [[ -z "$sim" ]]; then
  sim="$(PYTHONPATH="$here" python3 -c 'from ohm_localization.gridmap import mecanum_lab_dir; print(mecanum_lab_dir())')"
fi
if [[ -z "$sim" ]]; then
  echo "mecanum-lab not found — set MECANUM_LAB=/path/to/mecanum-lab (see install.sh --check)" >&2
  exit 2
fi
if ! python3 -c "import pygame" >/dev/null 2>&1; then
  echo "pygame is missing (the simulator's window needs it):  sudo apt install python3-pygame" >&2
  exit 3
fi

export PYTHONPATH="$here:$sim${PYTHONPATH:+:$PYTHONPATH}"
export PYGAME_HIDE_SUPPORT_PROMPT=1
# In-process bus unless the group asks for ROS: one process, one clock, no daemon, and the same
# grader.  MECANUM_ROS=0 is the laboratory's own switch for the real thing.
: "${MECANUM_ROS:=stub}"
export MECANUM_ROS

if [[ "$MECANUM_ROS" != "stub" ]]; then
  set +u
  for f in "${ROS_SETUP:-}" "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" /opt/ros/jazzy/setup.bash \
           /opt/ros/kilted/setup.bash /opt/ros/humble/setup.bash; do
    if [[ -n "$f" && -f "$f" ]]; then
      # shellcheck disable=SC1090
      source "$f" 2>/dev/null && break
    fi
  done
  set -u
fi

exec python3 "$here/tools/lab_grade.py" "$@"
