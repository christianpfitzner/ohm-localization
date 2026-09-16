# The exercise sheets — L1 … L4, and the machinery around them

Four graded tasks, **130 points**, one 180-minute laboratory block with a partner or two at one machine.
The thresholds are not aspirational: each was measured on this checkout, the reference solution's numbers
are printed next to them, and [`verification.md`](verification.md) has the run.

| | task | what changes from the one before | graded | solution |
|---|---|---|---|---|
| L1 | `mcl_production` | — | rmse ≤ 50 mm · ≥ 5× odometry · NEES [0.1, 5] | 27 mm · 6.96× · 0.35 — **30/30** |
| L2 | `mcl_wide` | prior σ **2 m** instead of 0.5 m | rmse ≤ 60 mm · ≥ 3× · max err ≤ 350 mm | 27 mm · 6.99× · max 74 mm — **35/35** |
| L3 | `mcl_budget` | **250 particles**, every beam | rmse ≤ 70 mm · ≥ 3× · NEES ≤ 12 | 56 mm · 7.9× · 5.4 — **35/35** |
| L4 | `mcl_dirty` | **LIDAR σ 15 mm → 250 mm** | rmse ≤ 70 mm · ≥ 2.5× · NEES [0.3, 5] | 56 mm · 3.32× · 0.58 — **30/30** |

`pass_from` is 50 %, but treat anything under "all four" as unfinished: they are one filter seen from four
directions, and L4 is only interesting once L1 works.

## How it runs

```bash
./tools/run_lab.sh grade --task mcl_production --controller student/mcl_template.py --headless
./tools/run_lab.sh run    --world production --task mcl_production --controller student/mcl_template.py --truth
```

`run` is the sandbox: the same hall, your own drive with the keyboard, `--truth` drawing the ground truth
next to your estimate. `grade` releases the robot, drives the drive, scores it and leaves. Add
`--json out.json` for the `measured`/`criteria` blocks — every number in the table above came out of one of
those files.

Mechanically this is the simulator's own `lab grade`, with one change: `tools/lab_grade.py` sets
`tasks.TASKS_PATH` to this repository's `config/tasks_localization.json`, so our four tasks are used and
`mecanum-lab/config/tasks.json` is not read at all. One module constant, no fork of the simulator, and the
launcher prints which file it loaded and from where — because a run graded against someone else's thresholds
is a confusing way to spend an afternoon.

The tasks are `kind: "kf"`, i.e. the grader's estimator family: you publish `/<robot>/kf/pose` with
`x, y, theta, sx, sy, sth` and it compares against `/truth`. Two properties of that family matter here.

* **The baseline is `sensor: "odom"`.** The improvement column is your estimate against *raw wheel
  odometry over the same drive*, computed by the grader itself (`_probe` takes the last sample of the named
  sensor). Not against GPS, and not against a number you measured.
* **The `sim` block is verified.** The task declares `odom.geometry.scale_xy = 1.03`, `odom.bias_omega`,
  the odom σs and — in L4 — `lidar.sigma = 0.25`, and the grader fails the run if the session it drove used
  different numbers. That is what makes the improvement column mean something: with the simulator's *default*
  odometry this drive drifts **6 mm**, and nothing beats that by a factor of two, so a threshold on
  improvement would have graded the sensor profile rather than your filter ([`verification.md` §8
  item 11](verification.md)).

The `mcl` block at the bottom of each task is what the reference node reads for its parameters — the
simulator's habit of keeping thresholds and handout text in one JSON, so a task that asks for a 2 m prior
does not need a second place that knows about it. Your node may read its parameters from anywhere, but
overriding ours by environment (`OHM_MCL_SIGMA_Z=0.5`, `OHM_MCL_PARTICLES=3000`, …) works without editing a
file, which is the fastest way through L4's sweep.

**Timing.** `timeout` and `warmup` are in *simulator* time; the simulator runs as fast as your CPU allows
(`tools/fastgrade.py` in the simulator reaches ~25× real time), so `rate_min: 5` is a liveness check, not a
compute budget. Do not plan around wall-clock: measure your own cost with
`tools/mcl_report.py --timing`, which reports milliseconds per update on your machine. On this one, 1200
particles × 107 beams is **3.6 ms** vectorised and **1509 ms** written as Python loops — 425×, and at 20 Hz
that is the difference between tracking a robot and being half an hour behind it.

## L1 — Where am I? Monte-Carlo localisation against the map

*30 pts · `production` · 34 s drive · solution 27 mm against 187 mm of odometry (6.96×), NEES 0.35*

The grader drives; you estimate. One scan every twentieth of a second, 360 beams, and a map you did not
build: draw N samples from the belief you were given, move them by the odometry, weight each by how well its
predicted beams fit the walls, and resample when the weights stop being spread out. Report the mean and the
1σ on `kf/pose` — the σ is graded, so a cloud wide enough to hide in is not a passing answer.

**What the grade is made of.** rmse ≤ 50 mm; ≥ 5× better than raw odometry; worst instantaneous error ≤
250 mm; ≥ 5 Hz; **zero wall contacts**; NEES in [0.1, 5]. The last two have killed more submissions than the
first: a drive that touches a wall fails regardless of accuracy (`contacts_max: 0` is the robot's contacts,
not your estimate's), and NEES outside the band means your σ is fiction even if your position is good.

**Protocol.**
1. Particles moved by the odometry's own delta `(rot1, trans, rot2)`, not teleported onto the odometry pose.
2. Weights from the map: predicted beam end points against the distance field.
3. `N_eff = 1/Σw²` as the resampling trigger — and show that resampling does **not** happen on every scan.
4. `sx, sy` from the spread of the cloud, not a constant.
5. The two numbers of the run (rmse, improvement) plus N_eff's median and your filter's `diversity()`.

**Traps that cost the last group their session.** Beam angles are `angle_min + i·angle_increment + θ` of the
*particle*; a beam that keeps the robot's heading weights every hypothesis as if it were the odometry. Sum
the weights in log form. σ of the heading comes from the circular mean (+170° and −170° do not average to 0).
And the scatter you add in the motion step is load-bearing: with zero motion noise this filter measures
**0.274 m error and NEES 749** while looking healthier than it ever will again.

## L2 — Parked somewhere else: localisation from a 2 m prior

*35 pts · same hall, same drive · prior σ = 2 m · solution 27 mm, 6.99×, max error 74 mm*

The robot was not where the odometry claims. Two metres of 1σ around the start pose means the true pose is in
the cloud *surrounded by a thousand wrong neighbours*, and the first scans have to throw the neighbours away.
`max_error_max` is 350 mm here — this task grades the beginning of the run rather than its average, because a
filter that takes eight seconds to find the robot has a pleasant RMSE and a useless first eight seconds.

**Protocol.** The initial cloud comes from the prior, not from the odometry with zero spread. Watch N_eff
while it converges and put the shape in your write-up: falling and then rising is a filter that found the
robot; sitting at 1 is a filter that found one lucky particle. Report how many scans the first 25 cm took
(the counters on `kf/info`, not a stopwatch). No `/truth`, no GPS — this one is decided by the LIDAR and the
map.

**And no random particles.** If you are tempted: injection on a working filter measured **0.530 m, never
converged, 87 240 particles injected**, against 13 mm for the same code with it off. It recovers a lost
robot; it does not improve one that is not lost. Measure that once yourself, it is four lines with a
recording, and then decide which story you want to tell in the protocol.

## L3 — Same accuracy, a quarter of the particles: where the budget belongs

*35 pts · longer drive (53 s) · 250 particles, every beam · solution 55–58 mm, 7.6–8.1×, NEES 5.3–5.5*

The task file now says `particles: 250` and `beam_stride: 1` — a quarter of the cloud for nearly the same
accuracy and 3× the beams per particle. The cost of one update is `particles × beams`, which is unchanged, so
this is the *same computation spent differently*: 360 beams constrain a pose far more sharply than 1200
particles cover a hall. Measured on one recording: 13 mm at 1200 × 107, 25 mm at 250 × 322, and NEES 0.22 →
2.07. Accuracy is not the only thing that degrades, and the graded numbers say the same thing: 27 mm
against L1 and 56 mm here, in the same hall on the same wheels.

**Protocol.** One update must be vectorised NumPy — all particles, all beams, no Python loop over particles.
Hand in the measured cost table (`mcl_report.py --timing` gives you milliseconds per update for any setting),
and say which of N_eff, rmse and max error broke first when the particles were taken away. Then answer, in
one or two sentences: why are 360 beams not 360 independent measurements of a hall? (Hint: what does the beam
2° from this one see at 4 m?)

If you keep L1's thresholds and pass them at 250 particles, you have beaten the reference — write that down
too, and say what you changed.

## L4 — The window is dirty: make the model match the sensor

*30 pts · same hall, same drive · `lidar.sigma = 0.25 m` · solution 56–57 mm, 3.3×, NEES 0.52–0.57*

One change you will not see in the topic: the LIDAR's σ is 250 mm instead of 15 mm — a reflective, partly
covered window, which on a real robot is a Tuesday. The parameters that won L1 will not survive this, not
because the filter is broken but because **σ_z is not a knob on the filter, it is a claim about the sensor**.
Weight a 250 mm measurement as though it were 150 mm and the cloud collapses onto whatever the first dirty
beams happened to like.

Measured on a recording of this drive, all with the rest of L1 untouched:

| σ_z | 0.15 (L1's) | 0.30 | **0.50** | 0.80 |
|---|---|---|---|---|
| rmse | 96 mm | 62 mm | **49 mm** | 62 mm |
| NEES | 15–38 | 7.7 | **2.2** | 2.3 |

Find your own numbers first, then compare. Note where the minimum is: near **2 × σ_sensor**, not at it.

**Protocol.** rmse and improvement with the parameters that got you there; NEES before and after you changed
σ_z; a σ_z table of at least four values (with a recording this is a loop over `mcl_report.py`, not four lab
sessions); the answer to "why does σ_z = 0.5 beat σ_z = 0.25, which is what the sensor actually measures?";
and what `z_rand` costs you with a sensor this noisy.

The threshold is NEES ∈ [0.3, 5] as well as rmse ≤ 70 mm, which is the point: at σ_z = 0.15 you will probably
still pass the accuracy and you will certainly fail the honesty.

## What to hand in

One PDF per group, per task: the parameters you used, the `measured` block from `--json`, the protocol items
above answered in sentences, and — for anything you claim rather than measured — the command that would
measure it. If a number in your protocol is not reproducible from a command you wrote down, it is not a
result. We will ask about two of them in the viva, and about which one you expected to be better.

## The development loop (this is the part that saves the afternoon)

A graded run takes ~36 s and gives you one number. A recording takes 36 s **once** and then every parameter
study is three seconds of replay:

```bash
OHM_RECORD=/tmp/l1.jsonl ./tools/run_lab.sh grade --task mcl_production \
        --controller tools/record_scans.py --headless            # one drive, everything it saw
./tools/mcl_report.py /tmp/l1.jsonl --particles 250 --stride 1 --sigma-z 0.5 --timing
```

Useful before you blame your filter:

* `tools/drive_check.py` — does the task's drive even fit its hall, and how many of the 360 beams find a
  wall along it. `production` gives 323, `arena` gives 99; the same filter differs by a factor of 40 between
  those two halls ([`verification.md` §4](verification.md)), and no amount of tuning fixes a hall.
* `tools/scan_probe.py` (run as a controller) — is *your* map/beam convention the simulator's? 12 mm is beam
  noise; metres is a mirrored map. Both offline scans and offline map come from the same text file here, so
  the test suite cannot answer this question and a mirrored convention looks like a filter that converges
  beautifully onto a pose two metres from the real one.
* `tools/icp_eval.py` — the scan-matching half of the laboratory, including the two geometries where it fails.

## Rules, and why they are the rules

* **`/truth` is off limits.** It exists on the graded bus (`debug_truth`) for the *grader*; a node that reads
  it is not a localiser, it is a report. If you want it for debugging, use `run --truth` and the visualiser.
* **Do not read the thresholds.** The `mcl` block in the task JSON is parameters; `rmse_max`, `nees` and
  friends are the mark scheme, and a node that optimises against them is measuring the wrong thing while
  scoring well on it.
* **Numbers in your protocol come from runs, not from files.** If you copy a threshold from this document,
  say that you copied it.
