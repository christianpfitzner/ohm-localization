# todo

What is left, in the order that makes the exercise teachable rather than the order that makes it look
finished. Items 1 and 2 block offering this as a laboratory; 3 onward are growth.

## 1. `student/` templates — and the proof that they do **not** pass

Nothing here has been shown to be *doable* in 180 minutes, only that it is solvable: `solution/mcl_node.py`
is a reference, not a handout. The templates must be generated from it (`tools/make_template.py`, a
`STUDENT`/`SOLUTION` split of the same file, so the docstrings and the reasoning stay and the bodies
disappear) and then **graded**: a template that passes L1 is a template that teaches nothing, and L1's
thresholds are loose enough to pass a shipped-working `update()` unchanged.

The check to add alongside it, once the templates exist:

```
./tools/run_lab.sh grade --task mcl_production --controller student/mcl_template.py --headless   # must FAIL
```

with the failure recorded per task (which criterion, and how far off) — because "the template fails" is
useless unless it fails on *the thing the task is about*: L1 should fail on accuracy with an empty
likelihood, L3 on `rate` or accuracy at 250 particles, L4 on NEES. A template that fails on a crash is a
template with a syntax error, not an exercise.

## 2. The ROS 2 deployment (the empty `launch/` and `runs/` directories are the sign of it)

Everything measured so far ran on the in-process stub bus, which is by design — but the lecture is a ROS
lecture, and the Jazzy question below cannot be settled on a stub.

* `package.xml` + `setup.py` (ament_python), so `ohm_localization`, `solution/` and `launch/` are
  installable and `ros2 launch` can find them; `rosdep`-clean with `numpy` only.
* `launch/mcl.launch.py` — the simulator's own node plus this one, `robot:=alice`, `task:=mcl_production`,
  `no_echo:=` exposed (it is a lesson, not a flag), and `gui:=true` for the `run` form.
* **Re-grade the four tasks over real DDS** (`MECANUM_ROS=0`). The `rate_min: 5` thresholds against a
  measured 30–40 Hz have plenty of margin for it, but "plenty of margin" is not a measurement: this is the
  first thing to do on a machine where the ROS environment is the point of the afternoon.
* `runs/` is where the `--log`/`--json` output of a session should land, named `<task>-<date>.json`, so the
  protocol citations point at a file. Then `.gitignore` it.

## 3. L5 (extra credit): ICP as the sensor, not as the exercise

The ICP material is complete as a measurement suite (`tools/icp_eval.py`, `docs/icp.md`) and absent as a
task. The task that would be worth building is **scan-matching odometry on a chassis whose wheels are
worse than L1's** (`odom.geometry.scale_xy` ≈ 1.10, which is a wheel radius that is wrong, and
measurable: L1's 1.03 already gives 187 mm over the graded drive). The student fuses ICP's relative pose
between successive scans into the odometry and the improvement column does the rest of the grading.

**Measure the ceiling before writing the threshold** — that is the rule this repository earned twice
(`verification.md` §8 item 11): the first L4 was unearnable against a 6 mm odometry, and the first arena
task had a ceiling of 1.12×. Expect the ICP odometry to be limited by the ~50 ms between scans at
0.3 m/s (1.5 cm of motion per pair, against a 6 mm median ICP error) and by the 10/12 convergence rate, and
set `improvement_min` from a measured run rather than from optimism.

Second thing worth a task, and cheap to build on what exists: **the degeneracy hunt.** Hand two halls —
`production` and a `corridor_text` hall dropped into the simulator's `worlds/` — and grade on the σ-to-error
ratio rather than on accuracy. It is the one ICP lesson that a grader can check without an opinion.

## 4. Three things the simulator would have to offer (ask, don't fork)

All three would improve this exercise and none is ours to implement in someone else's repository:

* **An `OccupancyGrid` on `/map`.** Today `GridMap` parses `worlds/<name>.txt` through
  `mecanum_lab.worlds`, which is fine and honest, but it means the students never meet the ROS map message
  that every real localisation stack is built around, and `worlds/*.txt` is the only "map" the simulator
  publishes anything about (`/sim/world` carries a wall *count*). With a `/map`, `GridMap.from_occupancy_grid`
  becomes an exercise in itself — and the resolution/reflection questions in `gridmap.py` get to be
  discovered by students rather than answered for them.
* **A teleport/kidnap service.** A genuine global-localisation task needs the robot to be moved; today the
  nearest thing is L2's 2 m prior, which is the same *computation* but not the same failure, and the
  measurement that shows it (uniform prior, 1200 particles, 3.2 m and never better) is done on a synthetic
  drive rather than on a live kidnapping.
* **Jazzy or Kilted.** `intelligent-robotics/praktikum/LAB-CONCEPT.md` pins the course to Jazzy; the
  simulator's docs say Kilted; both are installed here and the stub path uses neither. Somebody teaching
  the course has to decide, and the decision belongs in one of the two documents rather than in a footnote
  of mine.

## 5. Holes in the checking, that I can see and have not closed

* **A test that a recording is a recording of what it claims.** `verification.md` §8 item 3 is a map that was
  written with `"#"` between the rows and replayed as a hall twice as wide as reality; the tools now print the
  grid's cell count, and the right fix is an assertion in the loader (`len(row) == hall.size[0]/cell` and the
  row count likewise) plus a `needs_sim` test that records a couple of scans and compares the embedded grid to
  `load_hall(...)`. Cheap, and it is the bug class that produced two wrong conclusions in one day.
* **`MclParams.default_dt`** (0.05 s) is used by every bare-triple `predict_odometry` call, i.e. by all of the
  tests and by any node that forgets the stamp. A test asserting the *node's* effective dt against the
  recording's would catch a future refactor that drops the stamps — the failure it protects against is
  §8 item 5, worth 6× in error.
* **The ICP covariance is checked for consistency, not against a Monte-Carlo ensemble.** `tests/test_icp.py`
  checks the σ against the error in a couple of geometries, which is the right shape of check; a proper
  coverage test (does 1σ contain the truth 68 % of the time over 200 noise draws) is a 30-line script and the
  only thing that would let `docs/icp.md` say "the σ is right" instead of "the σ is honest in these
  geometries".
* **Run-to-run spread on L3 is not characterised.** One graded run measures 55 mm against a 60 mm ceiling; the
  spread across repeats looked like the third digit, but that was on one machine at one load. On the
  slowest laptop in the room the filter may be starved of updates rather than wrong, and if a cohort misses
  L3 that way, the fix is `timeout`/`warmup` or `rate_min`, not the accuracy limit — see
  `verification.md` §7 and §9.
* **`tools/mcl_report.py` has no way to compare two runs side by side** (`--compare a.json b.json`, one line
  each: what changed, what it cost). Every question in the protocol is a comparison, so the tool should speak
  the language.
* **The ICP basin is swept along x only.** A 2-D grid over (Δx, Δθ) would show the rotational basin, which is
  the one that matters for a scan-matching odometry; ASCII heat map, no matplotlib (there is none here).

## 6. Courseware

The sheets in `docs/exercises.md` are the handout, generated by hand from `config/tasks_localization.json`.
Two things to fix before printing: the simulator's LaTeX handout machinery reads *its* `config/tasks.json`,
so our file needs either its own include or a small pandoc step (the text/checks/hints are already structured
JSON, so the LaTeX is a template away); and the L3/L4 protocol tables want to be actual empty tables with
column headings, because students fill in numbers under headings rather than under prose.
