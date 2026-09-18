# What the shipped templates score

Written by `./tools/template_check.py --record`; do not edit. The graded rows are the simulator's
grader on the templates as they sit in this repository, `--headless`, in-process bus; the offline rows
are `ohm_localization.exercises.run()` on the same files.

## `student/mcl_template.py` — the localiser with its sensor model missing

The template's sensor model is one line that returns zero for every particle. That is not a broken
filter, it is a filter that has decided the LIDAR has nothing to say, and the table below is what
that decision costs. It is *more* than the odometry's own error, not equal to it: an unweighted
cloud keeps every particle, each heading random-walks at the motion model's floor, the paths curl,
and the mean of curled paths is a shortened line — so the estimate lags the drive (see the 0.63 m
measurement in `student/mcl_template.py`). `N_eff` sits at N, resampling never happens, and the node
publishes at 5 Hz the whole time: nothing in the plumbing complains, which is the point of the
exercise. The checks it misses are the ones the exercise is about; the ones it passes are the
plumbing the template gives you, so that the first hour goes into the model.

| task | verdict | estimate RMSE | raw odometry | improvement | max error | kf/pose | NEES |
|---|---|---|---|---|---|---|---|
| `mcl_production` | 0/30 FAIL | 0.68 m | 0.19 m | 0.27× | 0.98 m | 5.3 Hz | 0.12 |
| `mcl_wide` | 0/35 FAIL | 0.68 m | 0.19 m | 0.27× | 0.98 m | 5.3 Hz | 0.12 |
| `mcl_budget` | 0/35 FAIL | 1.25 m | 0.45 m | 0.36× | 2.50 m | 5.2 Hz | 0.18 |
| `mcl_dirty` | 0/30 FAIL | 0.68 m | 0.19 m | 0.27× | 0.98 m | 5.3 Hz | 0.12 |

## `student/icp_odom_template.py` — the odometry, with a place for a matcher

`scan_step()` is the TODO, so the node integrates the wheel step it was handed as its own guess. Its improvement over the odometry is therefore **exactly 1.00**, and its RMSE is the odometry's: the template *is* the baseline this exercise is marked against. That is the failure profile of a program that is completely correct and measures nothing — no crash, no contact, a publish rate already at the threshold — and it is why E3 is graded on *improvement* rather than on absolute error: a threshold that punished the world's miscalibration would teach a group to fake the pose instead of the match.

| task | verdict | estimate RMSE | raw odometry | improvement | max error | kf/pose | contacts |
|---|---|---|---|---|---|---|---|
| `icp_odom_production` | 0/30 FAIL | 3.97 m | 3.98 m | 1.00× | 7.35 m | 5.7 Hz | 0 |

## The offline templates — 0 points, on purpose

Every criterion of E1, E2 and E4 sits behind a function that raises, so all three score nothing. Stated with the criterion names, because the alternative state — a template that earns points by accident — makes a mark scheme unrecoverable: a group cannot tell a criterion they failed from one that was never reachable, and the next revision of the sheet quietly moves. `test/test_exercises.py` asserts this table on every commit; the graded rows above need `--live`.

| exercise | template | points | criteria not met |
|---|---|---|---|
| `e1_nn` | `nn_template.py` | 0/20 | `exact-on-every-shape`, `self-match-is-the-identity`, `empty-target-raises`, `measured-speed` |
| `e2_icp_pair` | `icp_pair_template.py` | 0/30 | `fits-as-well-as-the-truth`, `translation-error`, `heading-error`, `sigma-not-invented`, `degeneracy-is-reported`, `survives-a-wrong-guess` |
| `e4_particles` | `particles_template.py` | 0/50 | `uniform-on-the-floor`, `gaussian-prior-with-the-asked-sigma`, `noiseless-motion-is-rigid`, `the-cloud-spreads`, `noise-floors-are-rates`, `resample-follows-weights`, `estimate-averages-headings` |

Missing checks per graded task — the fix list, in the order the mark scheme asks for it:

- `mcl_production`: `rmse`, `max_error`, `improvement` — accuracy 0.68 violates rmse_max=0.05; max error 0.979 violates max_error_max=0.25; improvement over raw sensor 0.27 violates improvement_min=5.0
- `mcl_wide`: `rmse`, `max_error`, `improvement` — accuracy 0.68 violates rmse_max=0.06; max error 0.979 violates max_error_max=0.35; improvement over raw sensor 0.27 violates improvement_min=3.0
- `mcl_budget`: `rmse`, `max_error`, `improvement` — accuracy 1.249 violates rmse_max=0.07; max error 2.498 violates max_error_max=0.3; improvement over raw sensor 0.36 violates improvement_min=3.0
- `mcl_dirty`: `rmse`, `max_error`, `improvement`, `nees` — accuracy 0.68 violates rmse_max=0.07; max error 0.979 violates max_error_max=0.2; improvement over raw sensor 0.27 violates improvement_min=2.5; NEES 0.12 outside [0.3, 5.0] — the stated standard deviation does not match the actual error
- `icp_odom_production`: `rmse`, `max_error`, `improvement` — accuracy 3.968 violates rmse_max=1.6; max error 7.347 violates max_error_max=2.6; improvement over raw sensor 1.0 violates improvement_min=2.0

A template that passes a task is a template that teaches nothing; a template that fails by
crashing teaches debugging. Neither of those is what is recorded above, and `--check` is what
keeps this table honest when a threshold, a noise model or a template moves.

Machine-readable profile, written by the same run and read back by `--check` (do not edit the table above without this block; `--check` compares this, because parsing a Markdown table for a number that has a unit next to it is how a checker starts agreeing with a typo):

```json
[
 {
  "contacts": 0,
  "controller": "student/mcl_template.py",
  "improvement": 0.27,
  "kind": "graded",
  "max_error": 0.979,
  "max_points": 30,
  "missed": [
   "rmse",
   "max_error",
   "improvement"
  ],
  "nees": 0.12,
  "passed": false,
  "points": 0.0,
  "rate_hz": 5.3,
  "reason": "accuracy 0.68 violates rmse_max=0.05; max error 0.979 violates max_error_max=0.25; improvement over raw sensor 0.27 violates improvement_min=5.0",
  "rmse": 0.68,
  "rmse_odom": 0.187,
  "task": "mcl_production"
 },
 {
  "contacts": 0,
  "controller": "student/mcl_template.py",
  "improvement": 0.27,
  "kind": "graded",
  "max_error": 0.979,
  "max_points": 35,
  "missed": [
   "rmse",
   "max_error",
   "improvement"
  ],
  "nees": 0.12,
  "passed": false,
  "points": 0.0,
  "rate_hz": 5.3,
  "reason": "accuracy 0.68 violates rmse_max=0.06; max error 0.979 violates max_error_max=0.35; improvement over raw sensor 0.27 violates improvement_min=3.0",
  "rmse": 0.68,
  "rmse_odom": 0.187,
  "task": "mcl_wide"
 },
 {
  "contacts": 0,
  "controller": "student/mcl_template.py",
  "improvement": 0.36,
  "kind": "graded",
  "max_error": 2.498,
  "max_points": 35,
  "missed": [
   "rmse",
   "max_error",
   "improvement"
  ],
  "nees": 0.18,
  "passed": false,
  "points": 0.0,
  "rate_hz": 5.2,
  "reason": "accuracy 1.249 violates rmse_max=0.07; max error 2.498 violates max_error_max=0.3; improvement over raw sensor 0.36 violates improvement_min=3.0",
  "rmse": 1.249,
  "rmse_odom": 0.447,
  "task": "mcl_budget"
 },
 {
  "contacts": 0,
  "controller": "student/mcl_template.py",
  "improvement": 0.27,
  "kind": "graded",
  "max_error": 0.979,
  "max_points": 30,
  "missed": [
   "rmse",
   "max_error",
   "improvement",
   "nees"
  ],
  "nees": 0.12,
  "passed": false,
  "points": 0.0,
  "rate_hz": 5.3,
  "reason": "accuracy 0.68 violates rmse_max=0.07; max error 0.979 violates max_error_max=0.2; improvement over raw sensor 0.27 violates improvement_min=2.5; NEES 0.12 outside [0.3, 5.0] \u2014 the stated standard deviation does not match the actual error",
  "rmse": 0.68,
  "rmse_odom": 0.187,
  "task": "mcl_dirty"
 },
 {
  "contacts": 0,
  "controller": "student/icp_odom_template.py",
  "improvement": 1.0,
  "kind": "graded",
  "max_error": 7.347,
  "max_points": 30,
  "missed": [
   "rmse",
   "max_error",
   "improvement"
  ],
  "nees": 15.23,
  "passed": false,
  "points": 0.0,
  "rate_hz": 5.7,
  "reason": "accuracy 3.968 violates rmse_max=1.6; max error 7.347 violates max_error_max=2.6; improvement over raw sensor 1.0 violates improvement_min=2.0",
  "rmse": 3.968,
  "rmse_odom": 3.984,
  "task": "icp_odom_production"
 },
 {
  "controller": "student/nn_template.py",
  "kind": "offline",
  "max_points": 20.0,
  "missed": [
   "exact-on-every-shape",
   "self-match-is-the-identity",
   "empty-target-raises",
   "measured-speed"
  ],
  "passed": false,
  "points": 0.0,
  "reason": "TODO(E1): nearest_neighbour() \u2014 see tools/lab_check.py e1_nn; TODO(E1): nearest_neighbour() \u2014 see tools/lab_check.py e1_nn; NotImplementedError where ValueError was asked for; TODO(E1): nearest_neighbour() \u2014 see tools/lab_check.py e1_nn",
  "task": "e1_nn"
 },
 {
  "controller": "student/icp_pair_template.py",
  "kind": "offline",
  "max_points": 30.0,
  "missed": [
   "fits-as-well-as-the-truth",
   "translation-error",
   "heading-error",
   "sigma-not-invented",
   "degeneracy-is-reported",
   "survives-a-wrong-guess"
  ],
  "passed": false,
  "points": 0.0,
  "reason": "TODO(E2): register() \u2014 the loop, and sx/sy/sth from J\u1d40J. See tools/lab_check.py e2_icp_pair.; TODO(E2): register() \u2014 the loop, and sx/sy/sth from J\u1d40J. See tools/lab_check.py e2_icp_pair.; TODO(E2): register() \u2014 the loop, and sx/sy/sth from J\u1d40J. See tools/lab_check.py e2_icp_pair.; TODO(E2): register() \u2014 the loop, and sx/sy/sth from J\u1d40J. See tools/lab_check.py e2_icp_pair.; TODO(E2): register() \u2014 t",
  "task": "e2_icp_pair"
 },
 {
  "controller": "student/particles_template.py",
  "kind": "offline",
  "max_points": 50.0,
  "missed": [
   "uniform-on-the-floor",
   "gaussian-prior-with-the-asked-sigma",
   "noiseless-motion-is-rigid",
   "the-cloud-spreads",
   "noise-floors-are-rates",
   "resample-follows-weights",
   "estimate-averages-headings"
  ],
  "passed": false,
  "points": 0.0,
  "reason": "TODO(E4): sample_uniform() \u2014 see tools/lab_check.py e4_particles; TODO(E4): sample_gaussian() \u2014 remember the wrap; TODO(E4): move_particles() \u2014 three draws, the floors scaling with \u221adt; TODO(E4): move_particles() \u2014 three draws, the floors scaling with \u221adt; TODO(E4): move_particles() \u2014 three draws, the floors scaling with \u221adt; TODO(E4): resample() \u2014 systematic, one uniform draw; TODO(E4): weighted_",
  "task": "e4_particles"
 }
]
```
