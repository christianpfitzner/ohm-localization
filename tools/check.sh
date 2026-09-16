#!/usr/bin/env bash
# The offline half of "does this repository still do what its documentation says".
#
#   ./tools/check.sh              suite + task/drive check + ICP + sheet drift            (~40 s)
#   ./tools/check.sh --live       and the four graded runs against the real grader      (~3 min)
#   ./tools/check.sh --quick      just the test suite
#
# Everything except --live runs without a simulator session, so this is what to run on a laptop on a train.
# What it covers, and why each piece is here rather than being left to the tests: the pytest suite is
# self-contained, which is both its virtue and its hole — the scans it checks against come from the same
# `synth.cast` and the same parsed map text that it is testing, so a mirrored convention or a corrupted map
# cannot be caught there (that is `--live`'s `scan_probe`, and it happened for real: see docs/verification.md
# §8 item 3). The ICP claims are here because a document that quotes hero numbers from one lucky pair rots
# silently, and `--claims` makes it rot loudly instead.
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$here"
export PYGAME_HIDE_SUPPORT_PROMPT=1
export MECANUM_ROS="${MECANUM_ROS:-stub}"

live=0; quick=0
for a in "$@"; do case "$a" in
  --live) live=1 ;; --quick) quick=1 ;; *) echo "unknown option: $a (--live, --quick)" >&2; exit 2 ;;
esac; done

fail=0
step() { printf '\n\033[1m▶ %s\033[0m\n' "$1"; }
run() { if "$@"; then printf '  \033[32mok\033[0m   %s\n' "$*"; else printf '  \033[31mFAILED\033[0m %s\n' "$*"; fail=1; fi; }

step "test suite (numpy alone; one test skips without SciPy)"
run python3 -m pytest test -q
[ "$quick" = 1 ] && { [ "$fail" = 0 ] && echo -e "\nalright — the suite is green" || echo -e "\nthe suite is not green"; exit "$fail"; }

step "task file: are the four drives driveable, and are their halls worth localising in"
run python3 tools/drive_check.py

step "ICP: the numbers docs/icp.md quotes, recomputed"
run python3 tools/icp_eval.py --claims

step "student sheets: do docs/handout/* still say what config/tasks_localization.json says"
# Same shape as the ICP step: the thresholds on the paper are generated, so a re-tuned task file that never
# reached the printed sheet is a drift failure here rather than a cohort chasing an old target.
run python3 tools/make_handout.py --check

step "library imports without the simulator on the path at all"
run env PYTHONPATH="$here" python3 -c 'import ohm_localization, ohm_localization.gridmap as g
assert g.mecanum_lab_dir.__doc__, "the map layer should work with no simulator in sight"
from ohm_localization import icp, mcl, synth; print("  (gridmap/mcl/synth/icp import clean)")'

if [ "$live" = 1 ]; then
  step "the four tasks against the simulator's own grader"
  for t in mcl_production mcl_wide mcl_budget mcl_dirty; do
    printf '\n── %s\n' "$t"
    ./tools/run_lab.sh grade --task "$t" --controller solution/mcl_node.py --headless || fail=1
  done
  step "and the template: does the shipped student file still fail, and fail where the docs say"
  # Four graded runs of `student/mcl_template.py`, compared against student/FAILURE.md. The point is not that
  # it fails — a broken file fails too — it is that it fails on the *sensor model* while the plumbing (rate,
  # contacts, no crash) still works, which is the difference between an exercise and a debugging trap.
  run python3 tools/template_check.py --check

  step "and the package: does it build, and does the installed layout resolve?"
  # The graded path never needs a build, and a launch file that cannot find its own `config/` after
  # `colcon build` is the failure this step exists for. Skips (loudly) with no colcon on PATH.
  if command -v colcon >/dev/null 2>&1 || [ -x "$HOME/.local/bin/colcon" ]; then
    run ./install.sh --build
    run bash -c 'source "$PWD/install/setup.bash" && cd /tmp && python3 -c "
from ohm_localization import paths
import os
root = paths.data_root()
assert os.path.join(\"share\", \"ohm_localization\") in root, f\"not the install prefix: {root}\"
assert os.path.isfile(paths.task_file()), paths.task_file()
from ohm_localization.gridmap import load_hall
print(\"  (installed layout: \" + root + \", hall \" + load_hall(\"production\").name + \")\")"'
  else
    printf '  \033[33mskip\033[0m   no colcon on PATH — the ROS/colcon side is unverified on this machine\n'
  fi

  step "and the convention probe: is our map and beam model the simulator's? (a controller, so it grades 0/30)"
  # The probe publishes no pose, so its own grade is 0/30 by design and the launcher exits non-zero on
  # it: what this step checks is the convention line it prints, and a probe that cannot reach a session
  # prints nothing at all. Hence `|| true` *inside* the capture — with `pipefail` set, piping straight
  # into grep reports the launcher's failing grade as this step's failure, which is a story about
  # `pipefail` and not about the map.
  probe_out="$(./tools/run_lab.sh grade --task mcl_production --controller tools/scan_probe.py \
                   --headless 2>&1 || true)"
  echo "$probe_out" | grep -E "identity|mirror|clockwise|reverse|Hz|→" || true
  if ! grep -q "the simulator" <<<"$probe_out"; then
    printf '  \033[31mFAILED\033[0m the convention probe produced no verdict — no session, or no /truth\n'
    fail=1
  fi
fi

printf '\n%s\n' "------------------------------------------------------------"
[ "$fail" = 0 ] && echo "alright — everything that can be checked without a robot is checked and holds" \
                || echo "not all of it held; the FAILED lines above say which"
exit "$fail"
