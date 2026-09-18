#!/usr/bin/env python3
"""Prove that the student template fails — and fail in the way the documentation says it fails.

A template that passes is not a template. A template that fails for a reason nobody measured is a trap: the
group spends the first hour of a 180-minute session debugging a node that was broken on the inside, and the
difference between "the sensor model is missing, so you are exactly as good as your odometry" and "the
template crashes on the second spin" is the difference between a task and a mystery. Both of those states
look the same from the outside — 0 points — which is exactly why this has to be measured rather than
intended.

So: grade every shipped template on the tasks it is profiled on, and record which *criteria* it misses and by
how much.

    ./tools/template_check.py                  # 4 MCL tasks, 1 ICP-odometry task, 3 offline exercises
    ./tools/template_check.py --record         # also write student/FAILURE.md
    ./tools/template_check.py --check          # exit 1 if the profile moved (for tools/check.sh --live)
    ./tools/template_check.py --controller student/icp_odom_template.py      # one template only
    ./tools/template_check.py --tasks mcl_wide --no-offline                  # one graded run, for a quick look

The MCL and E3 tasks are the simulator's grader and take about a minute each; the three offline templates go
through `ohm_localization.exercises.run()` and take a few seconds, so they run on every pass.

`student/FAILURE.md` is the recorded profile. The comparison is on the *criterion names* exactly and on the
numbers with a 40 % tolerance: the graded numbers repeat to a few per cent across runs of the same code, and
a template whose failing criterion changes identity has changed behaviour, not precision.

Run time is the simulator's: about a minute per graded task, so this is a tool you run when a template or a
threshold changes, not something `tools/check.sh` does on every save (`--check` is in check.sh's `--live`
tier, where the graded runs live anyway).
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from ohm_localization import paths                                        # noqa: E402

TEMPLATE = os.path.join(ROOT, "student", "mcl_template.py")
PROFILE = os.path.join(ROOT, "student", "FAILURE.md")

# Every shipped template and the tasks it is profiled on, in one place. `--controller` narrows the run to one
# group (that is what it has always meant); without it, a `--record` pass covers all of them, because a
# documentation file that describes two of the four templates is the kind of drift a checker exists to catch.
GROUPS = (("student/mcl_template.py", ("mcl_production", "mcl_wide", "mcl_budget", "mcl_dirty")),
          ("student/icp_odom_template.py", ("icp_odom_production",)))
# The offline templates, profiled by the criteria engine instead of the simulator's grader. Cheap enough to run
# on every pass — a few seconds for all three, no simulator, no ROS.
OFFLINE = ("e1_nn", "e2_icp_pair", "e4_particles")

# The one thing the shipped template must always get right, on every task: it publishes, it does not crash,
# it does not hit a wall, and the estimate it publishes is not better than the odometry it was given.
MUST_FAIL = ("rmse", "improvement")            # the criteria the exercise is about: missing by design
MUST_NOT_BREAK = ("rate", "contacts")          # the plumbing the template gives you: must already work

# The grader's own arithmetic, spelled out so that the profile is derived from the same two things it is:
# the numbers it measured and the thresholds in config/tasks_localization.json. (Its `criteria` rows are
# printed whether a check passed or failed — a passing run also shows what it had to meet — so the rows
# alone cannot say which check was missed.)
CHECKS = (("rmse", "rmse_max", "max"), ("max_error", "max_error_max", "max"),
          ("improvement", "improvement_min", "min"), ("rate_hz", "rate_min", "min"),
          ("contacts", "contacts_max", "max"), ("nees", "nees", "range"))


def grade(task: str, controller: str, sim: str | None, keep: str) -> dict:
    """One graded run of `controller`, as the laboratory's own grader reports it."""
    out = os.path.join(keep, f"{task}.json")
    cmd = [os.path.join(ROOT, "tools", "run_lab.sh"), "grade", "--task", task,
           "--controller", controller, "--headless", "--json", out]
    env = dict(os.environ, MECANUM_ROS="stub", PYGAME_HIDE_SUPPORT_PROMPT="1")
    if sim:
        env["MECANUM_LAB"] = sim
    res = subprocess.run(cmd, env=env, capture_output=True, text=True, cwd=ROOT)
    if not os.path.isfile(out):
        raise SystemExit(f"no report for {task} (exit {res.returncode}):\n{res.stdout[-1200:]}\n{res.stderr[-1200:]}")
    with open(out, encoding="utf-8") as fh:
        return json.load(fh)["tasks"][0]


def missed_checks(measured: dict, limits: dict) -> list:
    """Which of the task's own thresholds the numbers miss, in the task file's order."""
    out = []
    for key, limit_key, sense in CHECKS:
        bound, value = limits.get(limit_key), measured.get(key)
        if bound is None or value is None:
            continue
        if (sense == "max" and value > bound) or (sense == "min" and value < bound) \
                or (sense == "range" and not (bound[0] <= value <= bound[1])):
            out.append(key)
    return out


def profile(task: str, r: dict, limits: dict, controller: str) -> dict:
    """What a shipped template did on one task, in the units the mark scheme uses."""
    m = r["measured"]
    return {"kind": "graded", "task": task, "controller": controller,
            "passed": bool(r["passed"]), "points": r.get("points", 0),
            "max_points": r.get("max_points", 0), "rmse": m.get("rmse"),
            "rmse_odom": m.get("rmse_odom"), "improvement": m.get("improvement"),
            "max_error": m.get("max_error"), "rate_hz": m.get("rate_hz"),
            "contacts": m.get("contacts"), "nees": m.get("nees"),
            "missed": missed_checks(m, limits), "reason": r.get("reason", "")}


def offline_profiles(ids: tuple) -> list:
    """What each offline template scores, and which criteria sent it there.

    Same purpose as a graded profile, ten seconds instead of ten minutes: `student/FAILURE.md` is the description
    of the starting state of every exercise, and "the template scores nothing" has two very different causes —
    an empty TODO and a criterion nobody can meet. Only the first one is a template.
    """
    from ohm_localization import exercises as ex

    book = ex.exercise_file()
    rows = []
    for ex_id in ids:
        e = ex.exercise(book, ex_id)
        res = ex.run(ex_id, os.path.join(ROOT, e["file"]), exercises=book)
        rows.append({"kind": "offline", "task": ex_id, "controller": e["file"],
                     "passed": res["passed"],
                     "points": res["points"], "max_points": res["max_points"],
                     "missed": [r["criterion"] for r in res["rows"] if not r["ok"]],
                     "reason": "; ".join(r["measured"] for r in res["rows"] if not r["ok"])[:400]})
    return rows


def render(rows: list[dict]) -> str:
    graded = [r for r in rows if r["kind"] == "graded"]
    mcl = [r for r in graded if r["controller"].endswith("mcl_template.py")]
    icp = [r for r in graded if not r["controller"].endswith("mcl_template.py")]
    offline = [r for r in rows if r["kind"] == "offline"]
    head = (
        "# What the shipped templates score\n\n"
        "Written by `./tools/template_check.py --record`; do not edit. The graded rows are the simulator's\n"
        "grader on the templates as they sit in this repository, `--headless`, in-process bus; the offline rows\n"
        "are `ohm_localization.exercises.run()` on the same files.\n\n"
        "## `student/mcl_template.py` — the localiser with its sensor model missing\n\n"
        "The template's sensor model is one line that returns zero for every particle. That is not a broken\n"
        "filter, it is a filter that has decided the LIDAR has nothing to say, and the table below is what\n"
        "that decision costs. It is *more* than the odometry's own error, not equal to it: an unweighted\n"
        "cloud keeps every particle, each heading random-walks at the motion model's floor, the paths curl,\n"
        "and the mean of curled paths is a shortened line — so the estimate lags the drive (see the 0.63 m\n"
        "measurement in `student/mcl_template.py`). `N_eff` sits at N, resampling never happens, and the node\n"
        "publishes at 5 Hz the whole time: nothing in the plumbing complains, which is the point of the\n"
        "exercise. The checks it misses are the ones the exercise is about; the ones it passes are the\n"
        "plumbing the template gives you, so that the first hour goes into the model.\n\n"
        "| task | verdict | estimate RMSE | raw odometry | improvement | max error | kf/pose | NEES |\n"
        "|---|---|---|---|---|---|---|---|\n")
    body = "".join(
        f"| `{r['task']}` | {r['points']:.0f}/{r['max_points']:.0f} "
        f"{'PASS' if r['passed'] else 'FAIL'} | {r['rmse']:.2f} m | {r['rmse_odom']:.2f} m | "
        f"{r['improvement']:.2f}× | {r['max_error']:.2f} m | {r['rate_hz']:.1f} Hz | {r['nees']:.2f} |\n"
        for r in mcl)
    icp_head = ("\n## `student/icp_odom_template.py` — the odometry, with a place for a matcher\n\n"
                "`scan_step()` is the TODO, so the node integrates the wheel step it was handed as its own guess. "
                "Its improvement over the odometry is therefore **exactly 1.00**, and its RMSE is the odometry's: "
                "the template *is* the baseline this exercise is marked against. That is the failure profile of a "
                "program that is completely correct and measures nothing — no crash, no contact, a publish rate "
                "already at the threshold — and it is why E3 is graded on *improvement* rather than on absolute "
                "error: a threshold that punished the world's miscalibration would teach a group to fake the pose "
                "instead of the match.\n\n"
                "| task | verdict | estimate RMSE | raw odometry | improvement | max error | kf/pose | contacts |\n"
                "|---|---|---|---|---|---|---|---|\n")
    icp_body = "".join(
        f"| `{r['task']}` | {r['points']:.0f}/{r['max_points']:.0f} "
        f"{'PASS' if r['passed'] else 'FAIL'} | {r['rmse']:.2f} m | {r['rmse_odom']:.2f} m | "
        f"{r['improvement']:.2f}× | {r['max_error']:.2f} m | {r['rate_hz']:.1f} Hz | {r['contacts']} |\n"
        for r in icp)
    off_head = ("\n## The offline templates — 0 points, on purpose\n\n"
                "Every criterion of E1, E2 and E4 sits behind a function that raises, so all three score nothing. "
                "Stated with the criterion names, because the alternative state — a template that earns points by "
                "accident — makes a mark scheme unrecoverable: a group cannot tell a criterion they failed from one "
                "that was never reachable, and the next revision of the sheet quietly moves. "
                "`test/test_exercises.py` asserts this table on every commit; the graded rows above need `--live`.\n\n"
                "| exercise | template | points | criteria not met |\n|---|---|---|---|\n")
    off_body = "".join(
        f"| `{r['task']}` | `{os.path.basename(r['controller'])}` | {r['points']:.0f}/{r['max_points']:.0f} | "
        + ", ".join(f"`{c}`" for c in r["missed"]) + " |\n" for r in offline)
    missed = ("\nMissing checks per graded task — the fix list, in the order the mark scheme asks for it:\n\n"
              + "".join(f"- `{r['task']}`: " + (", ".join(f"`{c}`" for c in r["missed"]) or "*none*")
                        + f" — {r['reason']}\n" for r in graded))
    machine = ("\nMachine-readable profile, written by the same run and read back by `--check` (do not edit "
               "the table above without this block; `--check` compares this, because parsing a Markdown table "
               "for a number that has a unit next to it is how a checker starts agreeing with a typo):\n\n"
               "```json\n" + json.dumps(rows, indent=1, sort_keys=True) + "\n```\n")
    return (head + body + icp_head + icp_body + off_head + off_body + missed
            + "\nA template that passes a task is a template that teaches nothing; a template that fails by\n"
              "crashing teaches debugging. Neither of those is what is recorded above, and `--check` is what\n"
              "keeps this table honest when a threshold, a noise model or a template moves.\n"
              + machine)


def compare(recorded: list[dict], fresh: list[dict]) -> list[str]:
    """Names exactly, numbers to 40 %: see the module docstring."""
    problems = []
    by_task = {r["task"]: r for r in recorded}
    for row in fresh:
        old = by_task.get(row["task"])
        if old is None:
            problems.append(f"{row['task']}: not in {os.path.basename(PROFILE)} — regenerate")
            continue
        if old.get("kind", "graded") != row["kind"]:
            problems.append(f"{row['task']}: kind {old.get('kind', 'graded')} → {row['kind']}")
        if old["missed"] != row["missed"]:
            problems.append(f"{row['task']}: missed criteria {old['missed']} → {row['missed']}")
        if old["passed"] != row["passed"]:
            problems.append(f"{row['task']}: passed {old['passed']} → {row['passed']}")
        if row["kind"] == "offline":
            if row["points"]:
                problems.append(f"{row['task']}: the offline template earns {row['points']}/"
                                f"{row['max_points']} — the answer is not behind its TODO")
            continue
        for key in ("rmse", "improvement", "nees", "max_error"):
            a, b = old.get(key), row.get(key)
            if a is None or b is None:
                continue
            if abs(a - b) > 0.4 * max(abs(a), abs(b), 1e-9):
                problems.append(f"{row['task']}.{key}: {a} → {b}")
    for row in fresh:
        if row["kind"] != "graded":
            continue
        for key in MUST_FAIL:
            if row.get(key) is None or (key == "improvement" and row["improvement"] > 1.2):
                problems.append(f"{row['task']}: the template is too good on {key} "
                                f"({row.get(key)}) — it should still be missing this")
        if row["passed"]:
            problems.append(f"{row['task']}: the template PASSES — that is not a template")
    return problems


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 epilog="See the module docstring for what a profile is for.")
    ap.add_argument("--tasks", default=None, help="comma-separated task ids (default: every task in the file)")
    ap.add_argument("--controller", default=None,
                    help="profile only this controller (default: every template in GROUPS)")
    ap.add_argument("--no-offline", action="store_true", help="skip the three offline templates")
    ap.add_argument("--sim", default=None, help="path to the mecanum-lab checkout (default: found)")
    ap.add_argument("--record", action="store_true", help=f"write {os.path.relpath(PROFILE, ROOT)}")
    ap.add_argument("--check", action="store_true", help="compare against the recorded profile, exit 1 on drift")
    a = ap.parse_args(argv)

    with open(paths.task_file(), encoding="utf-8") as fh:
        want = {t["id"]: t for t in json.load(fh)["tasks"]}
    if a.tasks:
        groups = [(a.controller or GROUPS[0][0], a.tasks.split(","))]
    elif a.controller:
        groups = [(a.controller, [t for c, ids in GROUPS if c == a.controller for t in ids])]
        if not groups[0][1]:
            raise SystemExit(f"no profiled tasks for controller {a.controller}; see GROUPS")
    else:
        groups = [(c, [t for t in ids if t in want]) for c, ids in GROUPS]
    for _, ids in groups:
        for tid in ids:
            if tid not in want:
                raise SystemExit(f"unknown task '{tid}'; the file has {', '.join(want)}")

    rows = []
    with tempfile.TemporaryDirectory(prefix="ohm-template-") as keep:
        for controller, ids in groups:
            for tid in ids:
                print(f"grading {controller} on {tid} …", flush=True)
                rows.append(profile(tid, grade(tid, os.path.join(ROOT, controller) if not os.path.isabs(controller) else controller,
                                      a.sim, keep), want[tid], controller))
    if not a.no_offline:
        print("grading the offline templates …", flush=True)
        rows += offline_profiles(OFFLINE)

    graded = [r for r in rows if r["kind"] == "graded"]
    print(f"\n{'task':>18} {'verdict':>8} {'rmse':>7} {'odom':>7} {'impr':>6} {'max':>7} {'Hz':>6} "
          f"{'NEES':>6}  missed")
    for r in graded:
        print(f"{r['task']:>18} {str(r['points']) + '/' + str(r['max_points']):>8} {r['rmse']:7.3f} "
              f"{r['rmse_odom']:7.3f} {r['improvement']:5.2f}x {r['max_error']:7.3f} {r['rate_hz']:6.1f} "
              f"{r['nees']:6.2f}  {', '.join(r['missed']) or '(nothing)'}")
    for r in [r for r in rows if r["kind"] == "offline"]:
        print(f"{r['task']:>18} {str(r['points']) + '/' + str(r['max_points']):>8} "
              f"{'-':>7} {'-':>7} {'-':>6} {'-':>7} {'-':>6} {'-':>6}  {', '.join(r['missed'])}")

    if a.record:
        with open(PROFILE, "w", encoding="utf-8") as fh:
            fh.write(render(rows))
        print(f"\nwrote {os.path.relpath(PROFILE, ROOT)}")
    if a.check:
        problems = compare(read_recorded(), rows) if os.path.isfile(PROFILE) else [
            f"--check: {os.path.relpath(PROFILE, ROOT)} does not exist — run --record first"]
        for msg in problems:
            print(f"  DRIFT {msg}", file=sys.stderr)
        print(f"template_check --check: {'OK' if not problems else str(len(problems)) + ' problem(s)'}")
        return 1 if problems else 0
    return 0


def read_recorded() -> list:
    """The fenced json block at the bottom of student/FAILURE.md."""
    with open(PROFILE, encoding="utf-8") as fh:
        text = fh.read()
    try:
        block = text.split("```json", 1)[1].split("```", 1)[0]
        return json.loads(block)
    except (IndexError, json.JSONDecodeError) as exc:
        raise SystemExit(f"{PROFILE}: no readable json profile ({exc}) — run --record")


if __name__ == "__main__":
    raise SystemExit(main())
