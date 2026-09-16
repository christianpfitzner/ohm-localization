# ohm-localization — ASY2 laboratory: localising a mecanum robot with a LIDAR and a map

Exercises for the `intelligent-robotics` (ASY2) laboratory, built on the [`mecanum-lab`](../mecanum-lab)
simulator: **Monte-Carlo localisation** (a particle filter against a known map) and **ICP scan
matching**, in ROS 2 idioms but runnable with no ROS at all.

Nothing in this README is a target. Every number is a measurement taken in this checkout, and
[`docs/verification.md`](docs/verification.md) gives the command that produced each one — along with the
eleven bugs those measurements caught, several of which were invisible to the test suite.

```
                                        ./tools/run_lab.sh grade --task mcl_production \
  map text from worlds/*.txt                --controller solution/mcl_node.py --headless
        │                                        │
        ▼                                        ▼
  GridMap (exact clearance field)      MonteCarloLocaliser  ──kf/pose──▶  the simulator's own grader
        │                                        │                          rmse 27 mm against 187 mm
        ▼                                        ▼                          of wheel odometry, 6.96×
  tools/mcl_report.py  (offline replay, 4 ms per update)      tools/icp_eval.py (scan matching)
```

## The four tasks, and what the solution achieves

| | task | hall | what changes | graded on | solution measures |
|---|---|---|---|---|---|
| L1 | `mcl_production` | `production` | — | rmse ≤ 50 mm, ≥ 5× odometry, NEES | **27 mm, 6.96×**, NEES 0.35 — 30/30 |
| L2 | `mcl_wide` | `production` | prior σ 2 m (parked elsewhere) | rmse ≤ 60 mm, ≥ 3× | **27 mm, 6.99×**, NEES 0.30 — 35/35 |
| L3 | `mcl_budget` | `production` | 250 particles, every beam | rmse ≤ 70 mm, ≥ 3× | **56 mm, 7.9×**, NEES 5.4 — 35/35 |
| L4 | `mcl_dirty` | `production` | LIDAR σ 15 mm → **250 mm** | rmse ≤ 70 mm, ≥ 2.5×, NEES in [0.3, 5] | **56 mm, 3.32×**, NEES 0.58 — 30/30 |

130/130 points, all four `PASS`, on the graded drive, in the simulator's own grader. The sheets and the
protocol questions are in [`docs/exercises.md`](docs/exercises.md).

L1–L3 are one filter with one parameter block changed. L4 is the one that cannot be solved by tuning
blindly: the sensor has changed under the student, and σ_z is not a knob on the filter but a claim
about the sensor. Carrying L1's σ_z = 0.15 m into L4 costs a factor of two in accuracy **and** pushes
NEES from 0.58 to somewhere between 15 and 38 — a filter reporting 5 mm of uncertainty around a 96 mm
error.

## Thirty seconds to see it work

```bash
./install.sh --check                     # python, numpy, the simulator checkout, ROS if any
./tools/check.sh                          # offline suite + task-file check + ICP claims (~40 s)

./tools/run_lab.sh grade --task mcl_production --controller solution/mcl_node.py --headless
                                          # the real grader, in-process, ~36 s wall
```

The interesting thing to run second is the offline replay, because a localisation exercise that takes
36 s per data point teaches patience rather than filtering:

```bash
OHM_RECORD=/tmp/prod.jsonl ./tools/run_lab.sh grade --task mcl_production \
        --controller tools/record_scans.py --headless        # one drive, all its messages, 36 s
./tools/mcl_report.py /tmp/prod.jsonl --prior 2 --particles 250 --stride 1 --timing
                                          # 727 scans replayed in 3 s, with the numbers above
```

Every parameter is a flag there and an environment variable in the live node (`OHM_MCL_SIGMA_Z=0.5`),
and both come from the same `mcl` block in `config/tasks_localization.json` — the simulator's own
idiom of keeping thresholds and handout text in one JSON that is read by both the grader and the node.

## What is here

```
ohm_localization/     the library, importable with numpy alone
    gridmap.py        worlds/*.txt → walls, and an exact clearance field (signed distance to rects)
    mcl.py            MonteCarloLocaliser: motion model, sensor model, N_eff, systematic resampling
    synth.py          a ray cast that is the simulator's, fake odometry, drive paths, random poses
    icp.py            point-to-point and point-to-line ICP, with covariance and cond(JᵀJ)
config/tasks_localization.json   the four task sheets: thresholds, drive, sim profile, and the mcl block
solution/mcl_node.py  the localising node (130/130).  This is the reference, not what a student is given
tests/                39 tests; 1 skips without SciPy.  Every threshold in them is a measured number
tools/                run_lab.sh, lab_grade.py, mcl_report.py, icp_eval.py, drive_check.py,
                      scan_probe.py, record_scans.py, check.sh
docs/                 mcl.md · icp.md · exercises.md · verification.md
```

## The two facts that decide whether this works

**Which hall you localise in.** `production` is 20 × 12 m with obstacles inside; from the poses the
drive visits, **323 of its 360 beams find a wall**. `arena` — the hall the Kalman-filter exercise uses,
and a good one there — has **99**. The same filter and parameters that reach 13 mm in `production` land
at 1.1 m in `arena`, and the best this filter achieves in `arena` over any drive we tried is **1.12×**
the odometry, obtained by setting σ_z to 1.0 m, i.e. by telling the filter not to believe the LIDAR.
That is why the graded tasks are in `production` and the open-hall result is a measurement in
[`docs/verification.md`](docs/verification.md) rather than a threshold: a task you can only pass at
1.1× grades the hall, not the work. `tools/drive_check.py` prints the echoing-beam count for every task
in the file, so this cannot be discovered by accident again.

**How honest the filter's σ is.** Accuracy is easy to plot and hard to deserve. The graded NEES column
(mean squared error over the reported variance) is what stops a submission that got lucky: zero motion
noise produces a filter that is confidently wrong (0.274 m error, NEES 749, in `tests/test_mcl.py`), and
a random-injection "rescue" that a third group will reach for measurably *hurts* a working filter —
530 mm and never converged, against 13 mm for the same code with injection off.

## Against the lecture

`05b1_localization.tex` derives the particle filter with `N_eff = 1/Σwᵢ²` as a practice problem; that
formula is a graded column here and `estimate()["neff"]` is measured *before* resampling, where it still
means something. Because it counts particles rather than hypotheses, a converged filter reports
`N_eff = 1200` while its cloud stands in **14** distinct 5 cm boxes: `diversity()` reports the boxes,
and the pair is the discussion. The vectorisation the lecture's performance section asks for is
measurable here at **3.55 ms vs 1508 ms** per update — 425× — which is the difference between a filter
that tracks a 20 Hz LIDAR and one that is 30 minutes behind it.

## Notes on the environment

* **ROS 2 Jazzy** is what `LAB-CONCEPT.md` pins the laboratory to; the simulator's own docs say Kilted.
  Both are installed here and both work. Everything in the default path runs on the in-process stub bus
  (`MECANUM_ROS=stub`) with no ROS at all, which is what makes this usable on the students' laptops;
  `MECANUM_ROS=0 ./tools/run_lab.sh …` uses the real DDS.
* `mecanum-lab` is not modified by this repository, and its `config/tasks.json` is not read: our four
  tasks live in our own file, which `tools/lab_grade.py` points the grader at. One module constant, no
  patch, no fork.
* Python 3.12, numpy 1.26. No SciPy and no matplotlib here, so the ray cast, the ICP nearest
  neighbours and every plot-free table are numpy; `icp.py` uses a KD-tree only if SciPy happens to be
  installed, and says which one ran.

## Not done yet

`todo.md`: `student/` templates (the solutions here must not be what is handed out, and the template
must measurably *not* pass), the ament package skeleton + launch files for the ROS 2 deployment, an
ICP-as-a-node task, and three things the simulator would have to offer for the next exercise
(an `OccupancyGrid` publisher, a kidnapped-robot teleport, and a Jazzy/Kilted decision).
