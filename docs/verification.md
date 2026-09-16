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
> **Superseded for the four tasks by §12.** The table below was measured before the rotation-noise fix
> described there, and it stayed because it is the record of the code it describes: the same tasks, the same
> thresholds and the same solution now reach 15–35 mm. Where a number below is quoted as *current*, §12 has
> the replacement; where it is quoted as a *finding* (which halls grade, what a replay costs, what a knob
> does), the finding is unchanged.

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
| `inject_below = 0.5` (random particles on low N_eff) | **0.530 m**, never converged | 87 240 particles injected, 0.1 % of the estimate in a wall. A rescue mechanism used as a performance knob: it *hurts* a filter that was working (0.013 m with it off). `test/test_mcl.py` asserts this. |
| `drop_uninformative = False` (weight beams that saw nothing as 8 m walls) | 0.022 m vs 0.013 m | 1.6× worse in `production`, where only ~37 of 360 beams are empty. In `arena` the same change costs a factor of 17 (§4 of `docs/mcl.md`) — the penalty is proportional to how much of the hall is out of range, which is why it is not one number. |
| uniform prior over the whole hall, 1200 particles | 3.2 m, never better | `rooms` has 4432 free cells at 25 cm: a third of a particle per cell. 4000 particles converges after 38 s; `arena` (5520 cells, mostly open) does not converge at 4000. Asserted in `test/test_mcl.py`. |
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
rotation, 5 mm median absolute, cond 17.7. On the single pair in `test/test_icp.py` — a 1.4 m motion with
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

**Which cloud the stride is allowed to thin.** `production`, the fixture pair of
`test/test_icp_coverage.py` (point-to-line, σ_beam 20 mm), median |Δt|, σ and mean squared Mahalanobis
over 120 noise draws; mean signed yaw error over the 399 consecutive pairs of the 20 s graded-shape drive
of `test/test_icp_odometry.py`:

| `stride` (source) | `stride_dst` (target) | median \|Δt\| | reported σ | mean Mahalanobis² | yaw error |
|---|---|---|---|---|---|
| 4 | 1 — what the code does by default | 6.0 mm | 3.6 mm | 3.38 | −0.099°/step |
| 4 | 4 | 132.9 mm | 39.3 mm | 9.54 | +1.307°/step |
| 2 | 2 | 23.4 mm | 6.1 mm | 14.88 | +0.054°/step |
| 1 | 1 | 6.5 mm | 2.1 mm | 10.59 | −0.035°/step |

Thinning the target moves the normal that point-to-line measures its residual along, which enters every
residual with the same sign: the yaw column is a bias, not noise.

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
   where the identity is a 1.4 m error. `test/test_icp.py` now asserts both variants against the truth.
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
   `test/conftest.py` (item 9) it required 0.8 m clearance from any wall; `maze` has 1.0 m corridors, so
   the whole hall fails that test and the caller got `None`. `margin` (inside the extents) and `clearance`
   (off the walls) are separate arguments now, and the docstring says which halls need which.
8. **The cross-check tests were green by borrowing an import.** `needs_sim` tests import `mecanum_lab`, but
   only another test file's import had put the checkout on `sys.path`: run in isolation they all failed
   with `ModuleNotFoundError`. The one that failed was the test that says whether `synth.cast` is the
   simulator's ray cast — the test that licenses every offline number in §2–§4 — and it had been passing
   by accident of file order. Fixed in `test/conftest.py` at collection time.
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
12. **`stride` was popped out of `**kwargs` twice, so only the first cloud was ever thinned.**
    `register_scans` asked each cloud for `kwargs.pop("stride", 1)`; the second ask found nothing left and
    used the default. Every number in §6 was therefore measured with a dense target, which is the good
    case — but the name promised something else, and honouring the promise is not a fix: point-to-line fits
    the wall through target points, and a thinned target took the pair from **6.0 mm** to **133 mm** and
    added +1.3°/step of yaw (§6's stride table). Caught by four coverage and odometry tests failing on the
    one-line change; `stride_dst` and `test_the_target_cloud_is_not_free_to_thin` keep it from being tidied
    back.

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
* The ICP covariance is checked for *consistency* (NEES-like ratios in `test/test_icp.py`) and for the
  degeneracy contrast in §6, not against a Monte-Carlo ensemble over many noise draws. That is the
  difference between "the σ is honest in these geometries" and "the σ is right".

## 10. The printed sheets

`tools/make_handout.py` renders `docs/handout/exercises.tex` and `.md` from `config/tasks_localization.json`
and nothing else, so the mark scheme on paper cannot silently disagree with the file the grader reads.
Measured here:

```
$ python3 tools/make_handout.py --check
make_handout --check: 4 sheets match the task file (130 pts, sum of the checks: 17 protocol tables)
$ cd docs/handout && latexmk -pdf -interaction=nonstopmode exercises.tex && pdfinfo exercises.pdf | grep Pages
Pages:           8
$ grep -c Overfull exercises.log
0
```

* **8 A4 pages for 4 tasks.** 26 tables: 17 protocol forms (one per `checks` item), 4 mark schemes, 4
  checkpoint rows and the name/date/group/seed box, with 356 empty cells sized for a pen. No overfull box,
  and the `.tex` holds no byte above 0x7F — every non-ASCII character in the task file goes through a
  mapping table, and one that is not in it aborts the generator instead of printing a sheet with `??` where
  a symbol of the mark scheme used to be.
* The generated thresholds are **parsed back out of the .tex** and compared whole-dictionary against the
  JSON (`test/test_handout.py`: 22 tests; the suite here is 63 passed, 1 skipped). Grepping for one number
  would still pass on a sheet that had lost a row, which is the failure this has to catch.
* Drift is checked in both directions. `--check` fails when the sheets are not what the task file generates
  — measured: setting `rmse_max` to 0.04 without regenerating gives exit 1 and names `exercises.tex` — and a
  test fails when the three commands in the sheet's scaffold block are no longer the commands
  `docs/exercises.md` documents, since the generator keeps them as constants and cannot see the docs.
* Two LaTeX faults found on the way, both invisible in the generator's text output and obvious on paper,
  which is why the PDF is compiled by a test rather than eyeballed. A protocol table that followed its
  question with a newline instead of a blank line was set **inline**, so the first sheet hung 13 cm into the
  margin (33 overfull boxes); and `°` → `\textdegree` swallowed the following space while an unescaped
  `mcl_production` inside `\texttt` aborted the compile with *Missing $ inserted* — the heading printed
  `+170°and − 170°points` and the task line did not compile at all.
* **Not measured:** whether a group can fill a sheet inside the 180-minute format. Each table asks for
  numbers the tools produce in about 40 s of replay, which is the argument for believing they are fillable;
  believing is the wrong verb here, for the same reason as the missing-template bullet in §9.


## 11. The package: two layouts, one colcon build

Everything above is measurable from a checkout. This section is about the layout the lecture's own install
script produces, which is the one that breaks a path bug, and it was built here:

```
$ ./install.sh --build                      # colcon build --paths . --symlink-install
Finished <<< ohm_localization [1.00s]
$ ./install.sh --workspace                  # interfaces, then the three python packages
Failed   <<< mecanum_lab_interfaces [0.52s, exited with code 1]      # needs rosidl_default_generators
Summary: 2 packages finished [0.60s]
  colcon build --paths …/mecanum-lab …/ohm-localization …/ohm_frontier
Summary: 3 packages finished [1.51s]                                 # mecanum_lab, ohm_localization, ohm_frontier
```

* **The interfaces go through a build of their own, and that failure must not be fatal.** Measured with one
  colcon invocation over all four paths: `mecanum_lab_interfaces` failed (this sandbox's `/opt/ros/jazzy` is a
  partial install without `rosidl_default_generators`) and colcon **aborted the two exercises behind it** —
  a workspace with a partial ROS install built *nothing*. Two passes later, both exercises build on the same
  machine. The simulator's own `install.sh` splits for the same reason; this is that split, copied.
* **`data_root()` prefers a checkout, then a prefix, then the build tree — in that order, and the middle one
  mattered.** Sourcing this workspace here sets `COLCON_PREFIX_PATH` and leaves `AMENT_PREFIX_PATH` *empty*,
  so a resolver that reads only the ROS variable returned `build/ohm_localization` — right files, wrong
  directory, because `--symlink-install` fills the build tree with symlinks named `setup.py`, `package.xml`
  and `config`. The build tree is now told apart by `tools/` and `test/`, the two directories `setup.py` never
  installs. Both branches are tested against a **fake prefix** built in a temp directory
  (`test/test_packaging.py`), because a fallback that only runs on someone else's machine is a fallback that
  will be wrong the first day it is needed.
* **The hall text comes from the simulator in either layout.** With `HOME` moved away so that no checkout is
  findable and the workspace sourced, all six halls load through `mecanum_lab.types.data_root()`:
  `production 20×12 m/10 rectangles · rooms 22×16/31 · arena 24×16/4 · maze 13×11/17 @ 1 m · open 30×20/4 ·
  track 18×11/5`. Before this, an *installed* simulator gave "no such world 'production'" — a path bug that
  reads like a broken exercise, which is the worst kind.
* `ros2 launch` itself cannot be run in this sandbox (no `ros2` CLI, no `launch_ros`, no `nav_msgs`), so the
  launch file is verified by parsing: `test/test_packaging.py` asserts that every argument the file *reads* is
  declared, that every `…:=` printed in its own docstring is declared, that every `--set` path is a real
  setting of the simulator's config **or** of a task's `sim` block (an unknown `--set` path is silently
  ignored by the simulator, so a typo there is a no-op argument forever), that every entry point resolves to a
  callable, and that `resource/ohm_localization` exists. It also builds the real `LaunchDescription` **when
  `launch_ros` is importable** — here it skips, with that reason printed.
* **`/map` without `nav_msgs`.** The `OccupancyGrid` conversion is a dict in the middle of `gridmap.py`, so
  the round trip is tested here: free-space geometry through the message is *exact* in all four halls
  (`max |Δ| = 0.00 mm` over 20 000 sample points, because every wall of every hall sits on the 0.25 m grid),
  the row order is pinned with an asymmetric corridor, `info.origin` is applied (`-2.0, 1.5` shifts every
  rectangle and nothing else), `-1` (unknown) is not a wall, a truncated `data` is refused naming its layout,
  and the eleven lines of message glue are checked against stub classes. Inside a wall the round trip cannot
  agree and does not try to: `distance_to_walls` reports depth *into the box that contains the point*, 1.0 m
  for `production`'s 3.5 × 2 m block and 0.125 m for the cells that tile it.

## 12. A step that does not translate has no bearing (and the numbers moved)

`delta_from_odometry` takes `rot1` from `atan2(dy, dx)`. For a robot that is standing still or turning on the
spot, the per-step displacement *is* the odometry's jitter — 1 cm here — whose bearing is a uniform random
angle. So `rot1` came out near ±π on those steps, `rot2 = turn − rot1` near ∓π, and because the rotation noise
is proportional to `|rot1|`, the model charged itself 0.05 · π ≈ 0.08 rad of heading noise **per stationary
step**. Measured over 10 s of standing (200 steps, 1200 particles, 1 cm odom jitter):

| | σ_x | σ_θ | this model's own 10 s floor |
|---|---|---|---|
| uncapped (as shipped until now) | **1.93 m** | **1.69 rad** | 0.14 m, 0.28 rad |
| capped (`predict(..., turn=…)`) | 0.52 m | 0.57 rad | 0.14 m, 0.28 rad |

The median `|rot1|` of a standing step is 1.61 rad and it does not depend on which way the robot faces, so this
was not an artefact of one heading. Even capped, 10 s of standing spreads the position 3.7× further than
`rate·√10`: the heading random walk rotates each subsequent step, so a floor stated as a rate is a statement
about *topic-rate invariance*, not an upper bound. `test/test_mcl.py` pins both rows and the rate-invariance
property; `test_a_step_that_really_turns_is_not_capped` pins that a straight leg and an arc are bit-identical
with and without the cap, which is what keeps the fix from being a new fudge factor.

The graded consequence, same task file, same seeds, after the fix:

| task | points | RMSE before | **RMSE now** | odometry | improvement before | **now** | max now | NEES before | **NEES now** |
|---|---|---|---|---|---|---|---|---|---|
| `mcl_production` | 30/30 | 0.027 | **0.015** | 0.187 | 6.96 | **12.47** | 0.037 | 0.35 | **0.19** |
| `mcl_wide` | 35/35 | 0.027 | **0.015** | 0.187 | 6.99 | **12.62** | 0.037 | 0.30 | **0.18** |
| `mcl_budget` | 35/35 | 0.055 | **0.017** | 0.437 | 7.88 | **26.35** | 0.053 | 5.56 | **0.27** |
| `mcl_dirty` | 30/30 | 0.057 | **0.035** | 0.190 | 3.35 | **5.38** | 0.090 | 0.57 | **1.66** |

Two of the *thresholds* were wrong in the new light and were changed, with the direction that rule requires:
only the **NEES lower floors** moved (0.1 → 0.05 for L1–L3). The filter got better, its σ came closer to its
error, and a floor of 0.1 would have sat 1.8× from the measured 0.18 — a correct answer failing on luck. The
upper bounds and every `rmse_max`/`improvement_min` are untouched: those are requirements on the robot, not on
the filter's self-confidence. L4's floor stays 0.3 (measured 1.66, 5.5× above it).

## 13. ICP as odometry: what stitching scans costs, and where the cost comes from

`ohm_localization/icp_odom_node.py` matches consecutive scans and integrates them — no map, no prior. Two
controls come first, because without them the rest is a story about a bug:

* integrating the **exact** transforms taken from `/truth` through the same accumulator reproduces the exact
  path (`err < 1e-9`). The first version chained `T @ res.T` instead of `T @ inverse(res.T)` and wandered
  **4.9 m** over 36 s while every pairwise number in the file still looked fine; `icp.relative(a, b)` carries
  *points* from the old frame into the new one, and the robot's increment is its inverse.
* point-to-point vs point-to-line on identical pairs, so that a difference is attributable to the objective.

| over the graded drive shape | one pairwise step | integrated | wheel odometry | MCL (same drive) |
|---|---|---|---|---|
| recording, 727 scans / 36.3 s | 7 mm, 0.04° | **1.76 m** (line), 1.0–1.2 m (point) | 0.18 m | 15 mm |
| synthetic, 400 steps / 20.0 s | 7 mm, 0.04° | **0.91 m** (line), 0.78 m (point) | 0.065 m | 15 mm |

A method that beats the wheels at every single step loses to them by a factor of ten over a drive. The reason
is bias, not noise, and it belongs to one of the two objectives: the **mean signed rotation error per step** is
**−0.051°** on the recording (summing to −36.7° of heading) and **−0.099°** on the synthetic drive (−39.3°
over 399 pairs), against **−0.0009°** and **+0.0080°** for point-to-point. A bias added N times grows linearly
where noise would have grown as √N — noise alone would have given 5.9° here. The bias survives `sigma_z`
(0.02/0.05), the `unit_sincos` scaling and every beam stride (−0.053 … −0.071°/step), so it is a property of
the linearised point-to-line objective in a hall of flat walls, not of my numerics. Two consequences are
pinned in `test/test_icp_odometry.py`: the node reports its σ as the **median pairwise σ widened by √N** (a
pairwise σ published as a pose σ would claim millimetres for a 36 s drive, and even √N under-claims a bias),
and the ordering *ICP-odometry ≫ odometry ≫ MCL* is asserted so that this section goes red if the hall or the
library changes under it.

**Not a graded task, deliberately.** The measured ceiling of ICP odometry in this hall is *worse than the
baseline the grader compares against* (`improvement` < 1), so any threshold on it would grade the choice of
method rather than the quality of an implementation. It is a viva question with a number attached — "your ICP
does 6 mm; over what horizon?" — and a node you can run.

## 14. The shipped template fails, and by how much

`./tools/template_check.py --record` writes `student/FAILURE.md`; `--check` compares the criterion names
exactly and the numbers to 40 %. Grading `student/mcl_template.py` as shipped (sensor model = `TODO(L1)`,
weights all zero):

| task | verdict | RMSE | odometry | improvement | max | kf/pose | NEES | missing checks |
|---|---|---|---|---|---|---|---|---|
| `mcl_production` | 0/30 FAIL | 0.680 | 0.187 | 0.27× | 0.979 | 5.3 Hz | 0.12 | rmse, max_error, improvement |
| `mcl_wide` | 0/35 FAIL | 0.681 | 0.187 | 0.27× | 0.988 | 5.3 Hz | 0.12 | rmse, max_error, improvement |
| `mcl_budget` | 0/35 FAIL | 1.257 | 0.436 | 0.35× | 2.507 | 5.2 Hz | 0.18 | rmse, max_error, improvement |
| `mcl_dirty` | 0/30 FAIL | 0.680 | 0.187 | 0.27× | 0.979 | 5.3 Hz | 0.12 | rmse, max_error, improvement, nees |

Three times *worse* than the odometry, not equal to it, and the mechanism is the exercise's first lesson: an
unweighted cloud keeps every particle, each particle's heading random-walks at the motion model's floor, the
paths curl, and the mean of curled paths is a shortened line — measured directly: a 10.2 m straight line at
680 steps and 0.02 rad of heading noise per step (the floor of the template's own model) ends with the mean of
1200 such clouds **0.63 m short of the wall**. `N_eff` sits at 1200, resampling never happens, the node
publishes at 5.3 Hz with zero wall contacts and no exception for 34 seconds: a quietly useless filter is
indistinguishable from a working one in the plumbing, which is why "0 points" is not a useful description of
a template and the *criterion names* are what this tool records.

**Not measured:** whether a group finishes the sheet in 180 minutes.
