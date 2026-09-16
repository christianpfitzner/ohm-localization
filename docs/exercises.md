# The exercise sheets — L1 … L4, and the machinery around them

Four graded tasks, **130 points**, one 180-minute laboratory block, one machine per pair. Every threshold
was measured on this checkout; [`verification.md`](verification.md) has the runs.

| | task | what changes from the one before | graded | solution |
|---|---|---|---|---|
| L1 | `mcl_production` | — | rmse ≤ 50 mm · ≥ 5× odometry · NEES [0.05, 5] | 15 mm · 12.5× · 0.19 — **30/30** |
| L2 | `mcl_wide` | prior σ **2 m** instead of 0.5 m | rmse ≤ 60 mm · ≥ 3× · max err ≤ 350 mm | 15 mm · 12.6× · max 37 mm — **35/35** |
| L3 | `mcl_budget` | **250 particles**, every beam | rmse ≤ 70 mm · ≥ 3× · NEES ≤ 12 | 17 mm · 26.4× · 0.27 — **35/35** |
| L4 | `mcl_dirty` | **LIDAR σ 15 mm → 250 mm** | rmse ≤ 70 mm · ≥ 2.5× · NEES [0.3, 5] | 35 mm · 5.4× · 1.66 — **30/30** |

`pass_from` is 50 %. L4 is only interesting once L1 works.

## How it runs

Grade one task:

```bash
./tools/run_lab.sh grade --task mcl_production --controller student/mcl_template.py --headless
```

`grade` releases the robot, drives the drive, scores it and leaves. It takes about 36 s.

Drive the same hall yourself, with the ground truth drawn next to your estimate:

```bash
./tools/run_lab.sh run --world production --task mcl_production --controller student/mcl_template.py --truth
```

Add `--json out.json` to either for the `measured`/`criteria` blocks.

Both call the simulator's `lab grade`; `tools/lab_grade.py` sets `tasks.TASKS_PATH` to
`config/tasks_localization.json`, so `mecanum-lab/config/tasks.json` is not read. The launcher prints the
task file it loaded.

The tasks are `kind: "kf"`: you publish `/<robot>/kf/pose` with `x, y, theta, sx, sy, sth` and the grader
compares it against `/truth`.

* **The baseline is `sensor: "odom"`.** The improvement column is your estimate against raw wheel odometry
  over the same drive, computed by the grader.
* **The `sim` block is verified.** The task declares `odom.geometry.scale_xy = 1.03`, `odom.bias_omega`, the
  odom σs and, in L4, `lidar.sigma = 0.25`; the grader fails the run if the session used different numbers.
  On the simulator's default odometry this drive drifts **6 mm**
  ([`verification.md` §8 item 11](verification.md)).

The `mcl` block of each task is what the reference node reads for its parameters; your node may read them
from anywhere. Environment overrides work without editing a file: `OHM_MCL_SIGMA_Z=0.5`,
`OHM_MCL_PARTICLES=3000`.

**Timing.** `timeout` and `warmup` are simulator time, and the simulator runs as fast as your CPU allows
(`tools/fastgrade.py` in the simulator reaches ~25× real time), so `rate_min: 5` is a liveness check, not a
compute budget. `tools/mcl_report.py --timing` reports milliseconds per update on your machine: 1200
particles × 107 beams is **3.6 ms** vectorised and **1509 ms** as Python loops. At 20 Hz that is the
difference between tracking a robot and being half an hour behind it.

## L1 — Where am I? Monte-Carlo localisation against the map

*30 pts · `production` · 34 s drive · solution 15 mm against 187 mm of odometry (12.5×), NEES 0.19 · the
shipped template reaches 0.68 m (0.27×) and fails — see `student/FAILURE.md`*

The grader drives; you estimate. One scan every twentieth of a second, 360 beams, and a map you did not
build: draw N samples from the belief you were given, move them by the odometry, weight each by how well its
predicted beams fit the walls, and resample when the weights stop being spread out. Report the mean and the
1σ on `kf/pose`; the σ is graded.

**What the grade is made of.** rmse ≤ 50 mm; ≥ 5× better than raw odometry; worst instantaneous error ≤
250 mm; ≥ 5 Hz; **zero wall contacts**; NEES in [0.1, 5]. A drive that touches a wall fails regardless of
accuracy (`contacts_max: 0` counts the robot's contacts, not your estimate's), and NEES outside the band
means the σ is wrong even where the position is good.

**Protocol.**
1. Particles moved by the odometry's own delta `(rot1, trans, rot2)`, not teleported onto the odometry pose.
2. Weights from the map: predicted beam end points against the distance field.
3. `N_eff = 1/Σw²` as the resampling trigger — and show that resampling does **not** happen on every scan.
4. `sx, sy` from the spread of the cloud, not a constant.
5. The two numbers of the run (rmse, improvement) plus N_eff's median and your filter's `diversity()`.

**Common mistakes.** Beam angles are `angle_min + i·angle_increment + θ` of the *particle*; a beam that
keeps the robot's heading weights every hypothesis as if it were the odometry. Sum the weights in log form.
σ of the heading comes from the circular mean (+170° and −170° do not average to 0). The scatter you add in
the motion step is load-bearing: with zero motion noise this filter measures **0.274 m error and NEES 749**.

## L2 — Parked somewhere else: localisation from a 2 m prior

*35 pts · same hall, same drive · prior σ = 2 m · solution 15 mm, 12.6×, max error 37 mm*

Two metres of 1σ around the start pose puts the true pose in the cloud surrounded by a thousand wrong
neighbours, and the first scans have to throw the neighbours away. `max_error_max` is 350 mm: this task
grades the beginning of the run, because a filter that takes eight seconds to find the robot has a pleasant
RMSE and a useless first eight seconds.

**Protocol.** The initial cloud comes from the prior, not from the odometry with zero spread. Watch N_eff
while it converges and put the shape in your write-up: falling and then rising is a filter that found the
robot; sitting at 1 is a filter that found one lucky particle. Report how many scans the first 25 cm took
(the counters on `kf/info`, not a stopwatch). No `/truth`, no GPS.

**No random particles.** Injection on a working filter measured **0.530 m, never converged, 87 240
particles injected**, against 13 mm for the same code with it off. It recovers a lost robot; it does not
improve one that is not lost.

## L3 — Same accuracy, a quarter of the particles: where the budget belongs

*35 pts · longer drive (53 s) · 250 particles, every beam · solution 55–58 mm, 7.6–8.1×, NEES 5.3–5.5*

The task file says `particles: 250` and `beam_stride: 1`. One update still costs `particles × beams`, so the
computation is the same and the budget is moved: 360 beams constrain a pose more sharply than 1200 particles
cover a hall. Measured on one recording: 13 mm at 1200 × 107, 25 mm at 250 × 322, NEES 0.22 → 2.07. Graded: 15
mm for L1, 17 mm here ([`verification.md` §12](verification.md)).

**Protocol.** One update must be vectorised NumPy — all particles, all beams, no Python loop over particles.
Hand in the measured cost table (`mcl_report.py --timing` gives milliseconds per update for any setting), and
say which of N_eff, rmse and max error broke first when the particles were taken away. Then answer, in one or
two sentences: why are 360 beams not 360 independent measurements of a hall? What does the beam 2° from this
one see at 4 m?

If you keep L1's thresholds and pass them at 250 particles, write that down and say what you changed.

## L4 — The window is dirty: make the model match the sensor

*30 pts · same hall, same drive · `lidar.sigma = 0.25 m` · solution 35 mm, 5.4×, NEES 1.66 · with L1's σ_z =
0.15 on this sensor the same filter reports 96 mm at NEES 15–38*

One change you will not see in the topic: the LIDAR's σ is 250 mm instead of 15 mm. **σ_z is not a knob on
the filter, it is a claim about the sensor.** Weight a 250 mm measurement as though it were 150 mm and the
cloud collapses onto whatever the first dirty beams happened to like.

Measured on a recording of this drive, all with the rest of L1 untouched:

| σ_z | 0.15 (L1's) | 0.30 | **0.50** | 0.80 |
|---|---|---|---|---|
| rmse | 96 mm | 62 mm | **49 mm** | 62 mm |
| NEES | 15–38 | 7.7 | **2.2** | 2.3 |

Find your own numbers first, then compare. The minimum is near **2 × σ_sensor**, not at it.

**Protocol.** rmse and improvement with the parameters that got you there; NEES before and after you changed
σ_z; a σ_z table of at least four values; why σ_z = 0.5 beats σ_z = 0.25, which is what the sensor actually
measures; and what `z_rand` costs you with a sensor this noisy.

The threshold is NEES ∈ [0.3, 5] as well as rmse ≤ 70 mm. At σ_z = 0.15 you will probably still pass the
accuracy and you will fail the honesty.

## What to hand in

One PDF per group, per task: the parameters you used, the `measured` block from `--json`, the protocol items
above answered in sentences, and for anything you claim rather than measured, the command that would measure
it. A number that is not reproducible from a command you wrote down is not a result. The viva asks about two
of them.

## The development loop

A graded run takes ~36 s and gives one number. A recording takes 36 s once, and after that a parameter study
is three seconds of replay.

Record one drive:

```bash
OHM_RECORD=/tmp/l1.jsonl ./tools/run_lab.sh grade --task mcl_production \
        --controller tools/record_scans.py --headless            # one drive, everything it saw
```

Replay it with any parameters:

```bash
./tools/mcl_report.py /tmp/l1.jsonl --particles 250 --stride 1 --sigma-z 0.5 --timing
```

Before you blame your filter:

* `tools/drive_check.py` — does the task's drive fit its hall, and how many of the 360 beams find a wall
  along it. `production` gives 323, `arena` gives 99, and the same filter differs by a factor of 40 between
  the two ([`verification.md` §4](verification.md)).
* `tools/scan_probe.py` (run as a controller) — is your map/beam convention the simulator's? 12 mm is beam
  noise, metres is a mirrored map. Offline scans and offline map come from the same text file, so the test
  suite cannot answer this. A mirrored convention looks like a filter converging beautifully onto a pose two
  metres from the real one.
* `tools/icp_eval.py` — the scan-matching half of the laboratory, including the two geometries where it fails.

## Rules

* **`/truth` is off limits.** It exists on the graded bus (`debug_truth`) for the grader; a node that reads it
  is not a localiser. For debugging, use `run --truth` and the visualiser.
* **Do not read the thresholds.** The `mcl` block in the task JSON is parameters; `rmse_max`, `nees` and
  friends are the mark scheme.
* **Numbers in your protocol come from runs, not from files.** If you copy a threshold from this document,
  say that you copied it.
