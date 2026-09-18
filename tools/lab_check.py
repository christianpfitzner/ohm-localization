#!/usr/bin/env python3
"""The grader of the offline exercises — the same PASS/FAIL report as the simulator's grader, no robot needed.

    python3 tools/lab_check.py                              # every offline exercise, on your files
    python3 tools/lab_check.py e1_nn e4_particles --verbose # two of them, with the measured notes
    python3 tools/lab_check.py e2_icp_pair --module solution/icp_pair_solution.py
    python3 tools/lab_check.py --list                       # what there is, and what each one is worth
    python3 tools/lab_check.py --check                      # CI: the solutions pass, the templates do not
    python3 tools/lab_check.py --json out.json              # the same report, machine-readable

The report is deliberately the same shape as `./tools/run_lab.sh grade` produces, for the same reason twice: a
group running both in one session should see one format, and the line that decides a mark should be the line the
documentation quotes its numbers from. The criteria themselves — what is measured, with what fixture, and why —
are in `ohm_localization/exercises.py`, and the points and thresholds are in
`config/exercises_localization.json`, which is also what the sheets and the GitHub task descriptions are
generated from.

Two of the five exercises (`e3_icp_odom`, `e5_mcl`) are graded by the simulator on a driving robot and are not
here: this tool prints the command that grades them and stops, because a mark scheme that quietly redefines a
graded exercise as an offline one is how a laboratory stops measuring what it thinks it measures.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ohm_localization import exercises as ex                                      # noqa: E402

BAR = "-" * 60


def report_one(result: dict, verbose: bool = False) -> str:
    """One exercise, in the grader's own shape: verdict, points, the criteria, the reason."""
    out = [BAR, "%-6s %5.1f/%4.1f pts  %s — %s" % ("PASS" if result["passed"] else "FAIL", result["points"],
                                                   result["max_points"], result["id"].split("_")[0].upper(),
                                                   result["title"])]
    for row in result["rows"]:
        out.append("        %s %s: %s" % ("required:" if not row["ok"] else "        ok",
                                          row["criterion"], row["measured"]))
    for note in result.get("notes", []):
        out.append("        measured: " + note)
    out.append("        verdict: " + result["reason"])
    out.append(BAR)
    if not verbose:
        out = [line for line in out if not line.startswith("        measured:")]
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("exercises", nargs="*", help="exercise ids (default: every offline one)")
    ap.add_argument("--module", metavar="FILE", help="grade this file instead of the exercise's own template "
                                                    "(only with exactly one exercise id)")
    ap.add_argument("--check", action="store_true", help="CI mode: every solution must pass every criterion and "
                                                         "every shipped template must earn nothing")
    ap.add_argument("--json", metavar="FILE", help="also write the whole report here")
    ap.add_argument("--verbose", action="store_true", help="with the notes each criterion measured")
    ap.add_argument("--list", action="store_true", help="list the exercises and their points and stop")
    a = ap.parse_args()

    try:
        book = ex.exercise_file()
    except FileNotFoundError:
        print(f"lab_check: no exercise file at {ex.EXERCISE_FILE}", file=sys.stderr)
        return 2
    offline = [e for e in book["exercises"] if e.get("kind") == "offline"]
    graded = [e for e in book["exercises"] if e.get("kind") != "offline"]

    if a.list:
        for e in offline + graded:
            print("%-14s %-3s %-5s %-3s  %s" % (e["id"], e["code"], e["visit"],
                                                "offl" if e.get("kind") == "offline" else "sim", e["title"]))
            print("%16s%s" % ("", "    " + e["run"]))
        return 0

    if a.module and len(a.exercises) != 1:
        print("lab_check: --module needs exactly one exercise id", file=sys.stderr)
        return 2
    wanted = a.exercises or [e["id"] for e in offline]
    results, scored, fail = [], [], 0
    for ex_id in wanted:
        try:
            e = ex.exercise(book, ex_id)
        except KeyError as exc:
            print(f"lab_check: {exc.args[0]}", file=sys.stderr)
            return 2
        if e.get("kind") != "offline":
            print(f"\n{e['code']} ({e['id']}) is graded by the simulator on the robot, not here:\n"
                  f"    {e['run']}\n")
            continue
        module = a.module or os.path.join(ex.ROOT, e["file"])
        try:
            result = ex.run(ex_id, module, verbose=a.verbose, exercises=book)
        except FileNotFoundError as exc:
            print(f"lab_check: {exc}", file=sys.stderr)
            fail = 1
            continue
        results.append(result)
        scored.append(result)
        print(report_one(result, a.verbose))
        if a.check and not a.module:
            scored.pop()                            # the templates are asserted, not totalled
            # In CI the interesting assertion is the pair: the shipped template earns nothing and the reference
            # earns everything. A template that scores points is a template that teaches a group to write the
            # answer somewhere else; a reference that does not pass its own sheet is a sheet nobody has met. Both
            # are silent rot, and both are caught here rather than in a lab session.
            if result["points"] > 0:
                print(f"lab_check: the shipped template earns {result['points']:g} points on {ex_id}; it should "
                      f"earn none — every criterion of an offline exercise is behind its TODO\n", file=sys.stderr)
                fail = 1
            if not os.path.isfile(os.path.join(ex.ROOT, e.get("solution", ""))):
                print(f"lab_check: {e['solution']} is missing, so nothing proves these thresholds have been met\n",
                      file=sys.stderr)
                fail = 1
        elif not result["passed"]:
            fail = 1

    if a.check and not a.module:
        for ex_id in wanted:
            e = ex.exercise(book, ex_id)
            res = ex.run(ex_id, os.path.join(ex.ROOT, e["solution"]), exercises=book)
            print(report_one(res, a.verbose))
            if not res["passed"]:
                print(f"lab_check: {e['solution']} does not meet its own sheet ({res['points']:g}/"
                      f"{res['max_points']:g})\n", file=sys.stderr)
                fail = 1
            scored.append(res)

    offline_max = sum(r["max_points"] for r in scored)
    offline_got = sum(r["points"] for r in scored)
    if scored:
        head = ("reference solutions" if a.check and not a.module else
                "your files" if not a.check else "the checked file")
        print(f"\nPoints reached ({head}): %.1f / %.1f  ->  %s"
              % (offline_got, offline_max,
                 "every offline criterion met" if not fail else "see the required: lines above"))
        if a.check and not a.module:
            print("(In --check both sides are asserted: the shipped templates must earn nothing on every "
                  "exercise, the reference solutions full marks.)")
    if a.json:
        with open(a.json, "w", encoding="utf-8") as handle:
            json.dump({"exercises": results, "points": round(offline_got, 1),
                       "max_points": offline_max, "passed": not fail}, handle, indent=2)
    return fail


if __name__ == "__main__":
    raise SystemExit(main())
