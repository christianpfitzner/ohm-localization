# Verification — every number in this repository, and the command that produced it

Measured on 2026-09-16 in this checkout: Python 3.12.3, numpy 1.26.4, no SciPy, no matplotlib;
`mecanum-lab` at `../mecanum-lab` on its in-process stub bus (`MECANUM_ROS=stub`); ROS 2 `jazzy` and
`kilted` both installed, neither needed for anything below.

Two rules were kept while writing this. A threshold is quoted with the run that produced it, never as a
target. And when a number and a mechanism disagreed, the *mechanism* won: several of the entries below
are claims that were deleted, not adjusted.

Everything is reproducible with `./tools/check.sh --live` (≈ 6 minutes, including the four graded runs).

---

## 1. The graded runs — the simulator's own grader, our task file

```
for T in mcl_production mcl_wide mcl_budget mcl_dirty; do
  ./tools/run_lab.sh grade --task $T --controller solution/mcl_node.py --headless; done
```

| task | points | rmse | raw odometry | improvement | max error | kf/pose | contacts | NEES | verdict |
|---|---|---|---|---|---|---|---|---|---|
| `mcl_production` | 30/30 | **0.027 m** | 0.187 m | **6.96×** | 0.110 m | 34.1 Hz | 0 | 0.35 | PASS |
| `mcl_wide` | 35/35 | **0.027 m** | 0.187 m | **6.99×** | 0.074 m | 34.1 Hz | 0 | 0.30 | PASS |
| `mcl_budget` | 35/35 | **0.055 m** | 0.437 m | **7.88×** | 0.183 m | 40.3 Hz | 0 | 5.39 | PASS |
| `mcl_dirty` | 30/30 | **0.056 m** | 0.187 m | **3.32×** | 0.147 m | 29.9 Hz | 0 | 0.58 | PASS |

130/130. `measured` and `criteria` come out of `mecanum_lab.grade` unchanged — the improvement column
is the grader's own `_probe` baseline against `sensor: "odom"`, not a number this repository computed.
Wall contacts 0 means the drives fit their hall, which `tools/drive_check.py` had established
*before* the first 36 s run (see §4).

Successive runs of the same task differ in the third digit, and how much is measured rather than assumed:
three repeats of L3 gave rmse **0.055 / 0.055 / 0.058 m**, improvement 7.58–8.13, NEES 5.31–5.46, max error
0.165–0.213 m; three of L4 gave **0.056 / 0.057 / 0.056 m**, improvement 3.31–3.34, NEES 0.52–0.57. L1 sat at
0.027 m on every run (once at 0.029). Those repeats are inside the thresholds by design, and one threshold
was widened because of them — see §7.

## 2. The same drives replayed offline

`tools/record_scans.py` records one graded drive as JSONL; `tools/mcl_report.py` replays it. Seven
hundred scanned-second of drive become three seconds of analysis, which is what makes the sweeps in
§3 possible at all.

| replay | particles · beams | rmse | late | max | odometry | improvement | NEES | N_eff min | 5 cm boxes | ms/update |
|---|---|---|---|---|---|---|---|---|---|---|
| L1 drive | 1200 · 107 | **0.013 m** | 0.012 | 0.111 | 0.170 | **13.2×** | 0.22 | 1200 | 14 | 3.9 |
| L2 (prior 2 m) | 1200 · 107 | 0.015 m | 0.012 | 0.152 | 0.170 | 11.1× | 0.76 | 1200 | 14 | 3.9 |
| L3 drive, budget | 250 · 322 | 0.025 m | 0.025 | 0.120 | 0.388 | 15.5× | 2.07 | 250 | 11 | 2.3 |
| L4 drive, dirty | 1200 · 320, σ_z 0.5 | **0.046 m** | 0.041 | 0.130 | 0.170 | 3.67× | 2.10 | 1200 | 13 | ~4 |

The replay is systematically **optimistic** against the grader — 13 mm vs 27 mm on L1, 46 mm vs 56 mm on
L4 — and the reason is known rather than waved away: the graded window starts after `warmup` and
includes the first seconds while the cloud is still narrowing, and the replay compares against the
recorded `/truth` at the scan stamps. Direction and size are stable across the four tasks (1.2×–2.1×),
which is why the thresholds are set from the **grader** and the replay is used only for *comparisons
between configurations of one drive*. A claim that a replay predicted a graded number would be the
kind this repository has already been warned about by §8 item 3.

## 3. Sensitivity, on recorded drives

`σ_z` with a clean sensor (L1 recording, `--sigma-z`, everything else at defaults):

| σ_z | 0.05 | 0.15 | 0.30 | 0.40 | 0.80 |
|---|---|---|---|---|---|
| rmse | 14 mm | 13 mm | 12 mm | 13 mm | 14 mm |
| improvement | 12.2× | 13.2× | 14.2× | 12.9× | 11.9× |
| NEES | 0.39 | 0.22 | 0.15 | 0.07 | 0.06 |

In `production` σ_z is **almost irrelevant between 0.05 m and 0.8 m** — 320 beams that each see a wall
at 1–4 m overwhelm a mis-set width. That is the control condition for the next table, and the reason L4
is graded on the *dirty* sensor rather than on σ_z in general.

The same sweep with the beam noise raised (`lidar.sigma`, i.e. task L4; each cell is the best of the σ_z
shown, with L1's σ_z = 0.15 for comparison):

| `lidar.sigma` | rmse at σ_z 0.15 | NEES at σ_z 0.15 | best rmse | at σ_z | improvement |
|---|---|---|---|---|---|
| 0.015 m (default) | 13 mm | 0.18 | 10 mm | 0.35 | 17.1× |
| 0.10 m | 25 mm | 15.7 | 19 mm | 0.35 | 9.0× |
| **0.25 m (L4)** | **96 mm** | **15–38** | **49 mm** | **0.50** | **3.5×** |
| 0.40 m | 122 mm | 24.6 | 69 mm | 0.30 | 2.5× |

Two things a student needs to be told explicitly, both visible only in this table. The optimum sits near
**2 × σ_sensor**, not at it — with 300 beams a likelihood that is honest per beam is still sharper than
the posterior a filter can resolve. And σ_z = 0.15 in the fourth row does not merely lose accuracy:
NEES 15–38 means the filter is reporting a σ **four to six times too small**, which is the failure mode
the NEES column exists to catch.

Two further knobs, one recording each:

| change | rmse | what happened |
|---|---|---|
| `inject_below = 0.5` (random particles on low N_eff) | **0.530 m**, never converged | 87 240 particles injected, 0.1 % of the estimate in a wall. A rescue mechanism used as a performance knob: it *hurts* a filter that was working (0.013 m with it off). `tests/test_mcl.py` asserts this. |
| `drop_uninformative = False` (weight beams that saw nothing as 8 m walls) | 0.022 m vs 0.013 m | 1.6× worse in `production`, where only ~37 of 360 beams are empty. In `arena` the same change costs a factor of 17 (§4 of `docs/mcl.md`) — the penalty is proportional to how much of the hall is out of range, which is why it is not one number. |
| uniform prior over the whole hall, 1200 particles | 3.2 m, never better | `rooms` has 4432 free cells at 25 cm: a third of a particle per cell. 4000 particles converges after 38 s; `arena` (5520 cells, mostly open) does not converge at 4000. Asserted in `tests/test_mcl.py`. |
| zero motion noise | 0.274 m, **NEES 749** | Confidently wrong, and it looks like success: N_eff stays high, reported σ falls, estimate follows the odometry. Asserted (`> 5× worse`, `NEES > 50`). |

## 4. Which hall is worth localising in

`python3 tools/drive_check.py`, all six shipped worlds, spawn poses from `mecanum_lab.worlds`, 7.2 m
spider drive (`synth.path_from_drive`, dead reckoning from the commanded blocks):

| world | size | min clearance | echoes mean/worst of 360 | at stride 3 |
|---|---|---|---|---|
| `arena` | 24 × 16 m | 4.25 m | **102 / 66** | 34 |
| `rooms` | 22 × 16 m | −0.24 m (drive hits a wall) | 347 / 334 | 116 |
| `production` | 20 × 12 m | **1.75 m** | **323 / 297** | **108** |
| `track` | 18 × 11 m | −0.05 m | 341 / 315 | 114 |
| `maze` | 13 × 11 m | −0.50 m | 360 / 360 | 120 |
| `open` | 30 × 20 m | 1.65 m | 183 / 156 | 61 |

The same filter, the same parameters (1200 particles, 1 beam in 3, σ_z 0.15), one recording each:

| hall | rmse | improvement | note |
|---|---|---|---|
| `production` | 0.013 m | 13.2× | the graded choice |
| `arena`, spider drive | 0.512 m | **0.34×** | **worse than the odometry** (0.174 m) |
| `arena`, 53 s drive | 1.140 m | 0.34× | odometry 0.392 m; NEES 30.2 |
| `arena`, best of 10 configurations | **0.348 m** | **1.12×** | σ_z = 1.0 m: the filter reaching its best result by not believing the LIDAR |

That last line is why `arena` is not a task. To make an exercise out of "get 1.12×" would grade the
hall; the open-hall effect is instead a protocol question and a column in `drive_check.py`. The first
version of this file had `mcl_arena` as a graded task with `rmse_max 0.25` — a threshold no submission
could earn, which the grading run reported as `accuracy 1.175 … violates rmse_max=0.25` with a correct
filter. Deleting the threshold and keeping the measurement is the fix.

`maze`'s 1.0 m corridors are worth noting twice: nothing can be 0.8 m from a wall in there, which is why
`synth.random_pose` separates `margin` from `clearance` (§8 item 7).

## 5. Message conventions, against the live simulator

`./tools/run_lab.sh grade --task mcl_production --controller tools/scan_probe.py`:

```
   odom  120 distinct stamps, 0.040 … 3.980 s, strictly increasing True, ~30.2 Hz
   scan   79 distinct stamps, 0.040 … 3.940 s, strictly increasing True, ~20.0 Hz
   truth  79 distinct stamps, 0.040 … 3.940 s, strictly increasing True, ~20.0 Hz
   map and beam convention, at one pose, 298 of 360 beams echoing:
      identity   mean |real − synthetic| =  0.0123 m over 298 beams   <== the simulator
      mirror_y                              1.8487 m   (not eligible: too few echoing beams)
      mirror_x                              1.8487 m   (not eligible)
      clockwise                             1.4845 m   (not eligible)
      reverse                               1.4947 m   (not eligible)
```

A repeat of the same probe measured 11.6 mm; both runs used `--headless`, which matters because the simulator
tries to open a window without it and a display-less machine then produces no probe output at all.

11.6–12.3 mm against a sensor whose σ is 15 mm: `synth.cast()` **is** the simulator's ray cast, and the map
text parsed by `parse_grid()` is the map the beams are cast against — so the offline numbers above
describe the same robot as the live ones. The four alternatives are rejected on the way for covering too
few of the echoing beams to be a comparison at all, which is printed rather than hidden. The clock
report is what ruled out the stamp hypothesis in §8 item 3: at 20 Hz with strictly increasing stamps, a
node that ticks on stamps is fine, and the 2.08 m failure was elsewhere.

## 6. ICP

`python3 tools/icp_eval.py --claims` recomputes all of this and exits non-zero if the document drifts
from it (it did, twice — see §8 item 10). Twelve synthetic pairs in `production` at σ_beam = 20 mm, stride
4, truth = `icp.relative()` of the two poses that generated them:

| variant | median |Δt| | worst | median |Δθ| | fit | reported σ | cond(JᵀJ) | iterations | converged |
|---|---|---|---|---|---|---|---|---|
| point-to-point | 34.6 mm | 112.9 | 0.264° | 55.1 mm | 6.45 mm | 24.6 | 18.0 | 12/12 |
| point-to-line | **8.6 mm** | 85.2 | **0.076°** | 49.7 mm | 5.42 mm | 17.4 | **5.5** | 11/12 |

The claims as `--claims` states them (10 pairs, seed 17): **6.6×** better in translation, **2.9×** in
rotation, 5 mm median absolute, cond 17.7. On the single pair in `tests/test_icp.py` — a 1.4 m motion with
0.4 rad of turn in `rooms`, deterministic fixture — the same comparison is **6.2×** and **263×**
(line 5.3 mm / 0.00264°, point 32.5 mm / 0.693°, cond 3.9 vs 8.8). The rotation advantage is real and
pair-dependent: it needs a lever arm against a long wall. Quoted at the median it is 2.9×, and that is
the number the claims list checks, because a hero pair is not a claim about a method.

On **real beams** (`--recorded /tmp/prod.jsonl`, σ_sensor 15 mm, truth from `/truth`): point 16.0 mm /
0.272°, point-to-line **6.0 mm / 0.040°**, 10/12 converged for line against 12/12 for point — the better
variant is also the one that sometimes fails to converge inside 30 iterations, which is worth knowing
before anyone builds an odometry on it.

Beam noise (12 pairs, median |Δt|): σ_beam 5 mm → line 9 mm; 20 mm → 9 mm; 50 mm → point 45 mm / line
10 mm; 150 mm → point 96 mm / line 40 mm. Both degrade near-linearly in σ_beam; the ratio narrows from
4× to 2.4×.

Basin of attraction (guess displaced along x, median |Δt| of the result):

| σ_beam | mode | 0 | 0.1 | 0.25 | 0.5 | 1.0 | 2.0 | 3.0 | 5.0 m |
|---|---|---|---|---|---|---|---|---|---|
| 20 mm | point | 35 mm | 36 | 36 | 38 | 67 | **1840** | 3087 | 5088 |
| 20 mm | line | 9 mm | 9 | 9 | 9 | 15 | **2468** | 3614 | 5135 |
| 150 mm | point | 95 mm | 95 | 94 | 92 | **102 → 1190** | | | |
| 150 mm | line | 40 mm | 40 | 40 | 40 | 43 | 54 | **2861** | 4957 |

So: a clean sensor tolerates a 1 m wrong guess in a structured hall and nothing at 2 m; with a dirty
window point-to-point gives up at 1 m while point-to-line still recovers from 2 m. Not "ICP converges":
ICP converges *from here*.

The two geometries the simulator's halls cannot provide (`--degenerate`, `gridmap.corridor_text`,
31 × 6 m of nothing with 0.5 m cells, 3 m and 4 m slides along the axis):

| corridor | variant | result | reported σ | σ understates by | cond | fitness at truth vs at the estimate |
|---|---|---|---|---|---|---|
| bare, 3 m slide | point | 2.995 m wrong, converged in 3 it | 2.55 mm | **1175×** | **12.0** | 158.2 mm vs 22.2 mm |
| bare, 3 m slide | line | 2.962 m wrong, converged in 3 it | 2.30 mm | **1289×** | **2406** | 158.2 mm vs 28.4 mm |
| posts every 4 m, 4 m slide | point | 4.002 m wrong, 2 it | 2.24 mm | 1790× | 10.5 | 474.9 mm vs 18.7 mm |
| posts every 4 m, 4 m slide | line | **3.999 m wrong**, 3 it | 3.48 mm | 1148× | 23.5 | **474.9 mm vs 18.6 mm** |

Two lessons, both measured rather than asserted in a paper. A slide along a bare corridor is not
measurable by a 2D LIDAR at all: the filter converges in three iterations, reports a millimetre σ, and is
3 m wrong. And with posts every 4 m the aliased pose fits the scan **25× better than the truth does** —
so ICP did nothing wrong, and nothing inside ICP can tell you. Note also that the degeneracy meter moves
**only for point-to-line** (2406 vs 12): the variant that models the walls is the one that notices the
walls say nothing about the direction along themselves. Point-to-point reports a healthy cond while
being 3 m off. A warning you do not print is a warning you do not have.

## 7. Thresholds, and the margin inside them

| task | graded quantity | threshold | solution measured | margin |
|---|---|---|---|---|
| L1 | rmse | ≤ 0.05 | 0.027 (three runs identical) | 1.9× |
| L1 | improvement | ≥ 5.0 | 6.96 | 1.4× |
| L2 | rmse | ≤ 0.06 | 0.027 | 2.2× |
| L3 | rmse | ≤ **0.07** | 0.055–0.058 | 1.2× |
| L3 | improvement | ≥ 3.0 | 7.58–8.13 | 2.5× |
| L3 | NEES | ≤ 12 | 5.31–5.46 | 2.2× |
| L4 | rmse | ≤ 0.07 | 0.056–0.057 | 1.2× |
| L4 | improvement | ≥ 2.5 | 3.31–3.34 | 1.3× |
| L4 | NEES | [0.3, 5] | 0.52–0.57 | 1.7× above the lower edge |

Two thresholds started life tighter than this table, and both were changed by a repeat run of the *correct*
solution rather than by a student's complaint. L3 was set at `rmse_max 0.06` when its only measurement was
0.055 — a repeat came out at 0.058, **3 % from failing a filter that had done everything right**, which is
not a threshold but a coin. It is now 0.07, and the lesson it was meant to deliver (a quarter of the cloud
costs real accuracy: 56 mm against L1's 27 mm, NEES 5.4 against 0.35) is carried by the comparison in the
task text and in `docs/mcl.md`, where it is a number rather than a trap. The lower edge of L4's NEES band was
0.5 until a correct run measured 0.58 — 16 % from an unearned failure — and is now 0.3.

Where a threshold can fail a correct answer without teaching anything, it is loosened; where it only fails
wrong ones it stays tight, and those are checked rather than wished: σ_z = 0.15 in L4 measures NEES **15–38**
against a band that ends at 5, zero motion noise measures **NEES 749**, and injection on measures
**0.530 m** against a 70 mm limit.

## 8. The bug ledger

These are the reasons the rest of this file is worth reading. Each one produced a plausible number or a
green test suite, and none was visible from the code that contained it.

1. **ICP never wrote its result.** The loop updated a local `T` and returned `res.T`, which stayed the
   identity. Every accuracy figure measured before this was fixed was measuring a transform of zero —
   and they looked *good*, because the identity is a fine answer for a pair generated 1.4 m apart when
   the error metric is the correspondence distance. Caught by `error_between(res.T, truth)` on a pair
   where the identity is a 1.4 m error. `tests/test_icp.py` now asserts both variants against the truth.
2. **The point-to-line Jacobian's rotation column was the lecture's cross product `n × p`** rather than
   the derivative (`n_y q_x − n_x q_y`) of the rotated point. With frozen correspondences the numeric
   correlation of my column against the analytic one was **+1.000** where it should be **−0.978**: the
   sign and scale were entangled, so the iteration still converged and only the covariance and the last
   few millimetres were wrong. Fixed by differentiating rather than transcribing.
3. **The offline replay ran on a map twice as wide as the hall.** `tools/record_scans.py` wrote its grid
   with `"#".join(...)` instead of `"".join(...)`, so 96 characters became 190. On that recording the
   filter reported RMSE 0.298 m and put its estimate **inside a wall 45 % of the time** — and that was
   the *second* explanation I nearly accepted for a live 2.08 m failure, after "the beam convention is
   mirrored". Two tools now exist because of it: `scan_probe.py` (§5) compares a real `/scan` against a
   real `/truth`, and `mcl_report.py` prints the grid's cell count on its first line, because 190 × 64 is
   obviously wrong the moment you are made to look.
4. **`str.replace` is silent in both directions.** It replaces every occurrence — and nothing at all when
   the text does not match. A diagnostic I believed I had installed had never been written into the file;
   the trace file stayed empty, which is the only reason the lie surfaced. Every scripted edit in this
   session after that point asserts its own match count, and the guideline is in the tool docstrings.
5. **The motion-noise floor was per prediction step, not per second.** `noise_floor_xy = 0.01` meant "per
   message", so a node predicting on each odometry sample (50 Hz) diffused √2.5 faster per second than a
   replay predicting per scan (20 Hz). Same recording, same parameters: **0.20 m offline, 1.26 m live**.
   This is what finally explained the arena failure, and it had already cost two wrong conclusions.
   Real chassis do not know how often their encoders are read, so the parameter is now
   `noise_rate_xy` / `noise_rate_theta` in m·s^−½ and rad·s^−½, applied as `rate·√dt`, with `dt` taken
   from the message stamp when the pose carries one. The two regression tests are
   `test_ten_seconds_of_standing_still_spreads_the_cloud_the_same_at_any_topic_rate` (20 Hz vs 200 Hz,
   spread = rate·√10 within 25 %) and `test_a_pose_that_carries_its_own_stamp_needs_no_told_dt`.
6. **The fix for 5 shipped broken**: `predict_odometry` computed `dt` and then called
   `predict(...)` without passing it. The two new tests failed within a minute of writing them, which is
   the only reason it did not become a seventh entry. `predict(..., dt=dt)`.
7. **`synth.random_pose` asked for geometry that `maze` does not contain.** After moving the helper out of
   `tests/conftest.py` (item 9) it required 0.8 m clearance from any wall; `maze` has 1.0 m corridors, so
   the whole hall fails that test and the caller got `None`. `margin` (inside the extents) and `clearance`
   (off the walls) are separate arguments now, and the docstring says which halls need which.
8. **The cross-check tests were green by borrowing an import.** `needs_sim` tests import `mecanum_lab`, but
   only another test file's import had put the checkout on `sys.path`: run in isolation they all failed
   with `ModuleNotFoundError`. The one that failed was the test that says whether `synth.cast` is the
   simulator's ray cast — the test that licenses every offline number in §2–§4 — and it had been passing
   by accident of file order. Fixed in `tests/conftest.py` at collection time.
9. **Fixtures that tools need must not live in the test suite.** `corridor_text` and `free_pose` moved to
   `gridmap.corridor_text` and `synth.random_pose`, because a tool that imports its fixtures from `tests/`
   cannot be run by a student — and the first version of `free_pose` had already been copied into a tool,
   which is how a copy drifts.
10. **Two ICP claims were hero numbers, not claims.** "`≥ 20× in rotation`" and "`cond < 12` on structured
    pairs" came from one pair each; the medians over ten pairs are **2.9×** and **17.7**. `--claims` now
    checks the medians and documents the pair-level numbers with their provenance. The tool's own wording
    is the rule: *fix the document or the code, in that order, and never the threshold*.
11. **Two thresholds were unearnable.** `improvement_min: 2.0` against the simulator's *default* odometry,
    which drifts 6 mm over the graded drive — nothing beats that by two, and the task would have graded
    the choice of sensor profile. And `mcl_arena` at `rmse_max 0.25` where the ceiling is 1.12× (§4). The
    fixes were different and worth naming: L1–L4 now declare an `odom` block with the model error a real
    chassis has (`geometry.scale_xy 1.03`, `bias_omega 0.004`), which makes the baseline 187 mm and the
    improvement meaningful; and the arena task became the dirty-window task, whose ceiling (49 mm offline,
    56 mm graded) was measured before its threshold was written.

## 9. Soft spots, stated plainly

* **1.2× margins** on L3 and L4 rmse are tighter than I would like, and they are measured rather than
  chosen: three repeats each, spread in the third decimal (§1, §7). A laptop materially slower than this one
  would push L3 the wrong way, because its filter has 250 particles × 322 beams to fit between two scans. If
  a cohort misses L3 on speed rather than on understanding, the fix is `timeout`/`warmup` (sim time) or
  `rate_min` — not the accuracy limit, which is the thing being taught.
* All graded runs here used the **stub bus**. The ROS 2 path shares `grade.py` and `robot_io`, and
  `MECANUM_ROS=0` was verified to bring the node up, but the four tasks have not been re-graded over
  real DDS here — the rate checks (≥ 5 Hz against a measured 30–40 Hz) have the margin for it, and
  `todo.md` keeps it as the first thing to do on a machine with ROS.
* **No student template exists yet**, so nothing here has been proven to be *doable* in the 180-minute
  format — only that it is solvable. The template must be measured to fail (`tools/make_template.py`,
  `todo.md` item 1) before the exercise is offered, and L1's thresholds would pass unchanged if a
  template shipped a working `update()`: that is a property of the template, not of the thresholds.
* `diversity()` counts 5 cm boxes; `MclParams.default_dt` (0.05 s) is used for any prediction whose pose
  carries no stamp, which includes every bare-triple call in the tests. Both are documented where they
  are used; neither is measured as a *task* quantity.
* The ICP covariance is checked for *consistency* (NEES-like ratios in `tests/test_icp.py`) and for the
  degeneracy contrast in §6, not against a Monte-Carlo ensemble over many noise draws. That is the
  difference between "the σ is honest in these geometries" and "the σ is right".
