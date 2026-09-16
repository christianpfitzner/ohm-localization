# What the shipped template scores

Written by `./tools/template_check.py --record`; do not edit. Every number is the simulator's
grader on `student/mcl_template.py` as it sits in this repository, `--headless`, in-process bus.

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
| `mcl_wide` | 0/35 FAIL | 0.68 m | 0.19 m | 0.27× | 0.99 m | 5.3 Hz | 0.12 |
| `mcl_budget` | 0/35 FAIL | 1.26 m | 0.44 m | 0.35× | 2.51 m | 5.2 Hz | 0.18 |
| `mcl_dirty` | 0/30 FAIL | 0.68 m | 0.19 m | 0.27× | 0.98 m | 5.3 Hz | 0.12 |

Missing checks per task — the fix list, in the order the mark scheme asks for it:

- `mcl_production`: `rmse`, `max_error`, `improvement` — accuracy 0.68 violates rmse_max=0.05; max error 0.979 violates max_error_max=0.25; improvement over raw sensor 0.27 violates improvement_min=5.0
- `mcl_wide`: `rmse`, `max_error`, `improvement` — accuracy 0.681 violates rmse_max=0.06; max error 0.988 violates max_error_max=0.35; improvement over raw sensor 0.27 violates improvement_min=3.0
- `mcl_budget`: `rmse`, `max_error`, `improvement` — accuracy 1.257 violates rmse_max=0.07; max error 2.507 violates max_error_max=0.3; improvement over raw sensor 0.35 violates improvement_min=3.0
- `mcl_dirty`: `rmse`, `max_error`, `improvement`, `nees` — accuracy 0.68 violates rmse_max=0.07; max error 0.979 violates max_error_max=0.2; improvement over raw sensor 0.27 violates improvement_min=2.5; NEES 0.12 outside [0.3, 5.0] — the stated standard deviation does not match the actual error

A template that passes a task is a template that teaches nothing; a template that fails by
crashing teaches debugging. Neither of those is what is recorded above, and `--check` is what
keeps this table honest when a threshold, a noise model or the template itself moves.

Machine-readable profile, written by the same run and read back by `--check` (do not edit the table above without this block; `--check` compares this, because parsing a Markdown table for a number that has a unit next to it is how a checker starts agreeing with a typo):

```json
[
 {
  "contacts": 0,
  "improvement": 0.27,
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
  "improvement": 0.27,
  "max_error": 0.988,
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
  "reason": "accuracy 0.681 violates rmse_max=0.06; max error 0.988 violates max_error_max=0.35; improvement over raw sensor 0.27 violates improvement_min=3.0",
  "rmse": 0.681,
  "rmse_odom": 0.187,
  "task": "mcl_wide"
 },
 {
  "contacts": 0,
  "improvement": 0.35,
  "max_error": 2.507,
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
  "reason": "accuracy 1.257 violates rmse_max=0.07; max error 2.507 violates max_error_max=0.3; improvement over raw sensor 0.35 violates improvement_min=3.0",
  "rmse": 1.257,
  "rmse_odom": 0.436,
  "task": "mcl_budget"
 },
 {
  "contacts": 0,
  "improvement": 0.27,
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
 }
]
```
