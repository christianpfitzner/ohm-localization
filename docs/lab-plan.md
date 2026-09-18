# The plan a tutor reads — four 180-minute visits, five exercises

Everything on a student sheet comes from `config/exercises_localization.json`; this file is what does **not** fit
on a sheet: what to have ready, where groups get stuck, which answers to accept, and how the marks are actually
awarded. It is written for the person running the room, not for the group.

```
I1  E1 nearest neighbour (20)      +  E2 one pair, and its σ (30)      offline · 50 pts · no robot
I2  E3 scan matching as odometry   (30)                               on the robot · simulator-graded
M1  E4 the particles of MCL        (50)                               offline · 50 pts · no robot
M2  E5 the complete localiser      (130)                              on the robot · four graded drives
                                                                            260 points
```

## The evening before

```bash
cd ~/git/ohm-localization
./tools/check.sh                    # suite + documentation self-consistency, ~1 min
./tools/check.sh --live             # when a robot task or a threshold changed: grades E3 and E5, ~5 min
ros2 launch mecanum_lab demo.launch.py  seconds.env:=20    # the simulator is alive and the hall loads
python3 tools/lab_check.py          # the three offline exercises, on the reference: 100/100 in ~8 s
```

Then, once, in a terminal the groups will not see:

```bash
python3 student/nn_template.py         # the four demos, with what is missing from each
python3 student/icp_pair_template.py
python3 student/particles_template.py
```

Things that have bitten before and are worth ten minutes:

* **`ohm-lab` must be importable** (`./install.sh` in `~/git/mecanum-lab`), and the file that decides it is
  read once per login shell. If a group says "the grader cannot find my file", it is nearly always this and not
  their code.
* **The world hash is the map.** A task records the hall it was driven in; a rebuilt `hall.yaml` fails the grade
  with a different hash, and no localisation code can fix that.
* **`--live` takes minutes, not seconds**, because every measurement is a real drive. Say so before a group asks
  you to re-run the mark scheme in front of them.
* Have `tools/mcl_report.py` and `tools/icp_eval.py` open, and a recording of the drive in question
  (`OHM_RECORD=drive.jsonl ./tools/run_lab.sh grade --task icp_odom_production --controller tools/record_scans.py
  --headless`). Every good question in these exercises ends
  with a number, and the fastest way to end an argument is to print it.

## I1 · 180 min · E1 (20) + E2 (30) · offline

**Target:** a matcher is a correspondence, a transform and a covariance, and the covariance is the part that
tells you whether to believe the other two.

| min | what happens |
|---|---|
| 0–15 | the demo: `python3 student/icp_pair_template.py` prints one pair, and a matcher that does not exist yet |
| 15–60 | **E1**. Two-loop first, then the broadcast, then the row blocks. The number in the table is the *ratio*. |
| 60–75 | the shape trap, on the board: `(n,2) − (2,m)` is `(n,n,2)`; one `None` and one `newaxis` is 150× |
| 75–160 | **E2**. One pair end to end, then all twelve, then the corridor |
| 160–180 | the viva: the four questions below, and the corridor pair's `cond` on the projector |

**Ask, and listen for:**

* *"What does your code do with a beam that hit nothing?"* — `inf` and a range equal to `range_max` are not
  points. A matcher that registers against an empty cloud reports a pose with a covariance of zero.
* *"You have a transform and a σ. Which one is the measurement?"* — neither: the transform is the estimate, the
  σ is its standard error, and both come out of the same fit. `T` carries no uncertainty of its own.
* *"The corridor pair fits to 2.5 mm and its σ along the corridor is 21 mm. Where is the 30 cm of freedom?"* —
  the answer this exercise is built to make them find. The fit's σ measures how well the *fit* is pinned by
  points 20 mm apart, and the null direction collects its σ from the beam noise tilting wall segments, not from
  the geometry. `cond` is 2644 there and 9 on a hall pair. Accept the criterion either way — a large `cond`, or a
  σ that is honestly large — but make them say which they produced, and why.
* *"Your twelve pairs have σ from 1.6 mm to 9.2 mm. Did twelve identical matches run twelve times?"* — no:
  a pair looking at a corner is pinned harder than a pair looking at one long wall, and σ is the number that
  knows. A matcher reporting the same σ for every pair is inventing it.

**Where the marks go:** E1 is 20 (10 correctness, 3 self-match, 3 empty-input, **4 speed**); E2 is 30 (7 fit
quality, 4 + 3 accuracy, **6 σ**, **6 degeneracy**, 4 robustness). Twelve of E2's thirty points are the
covariance. If a group has spent 100 of its 180 minutes on the transform and none on the σ, redirect them at
minute 100, out loud.

## I2 · 180 min · E3 (30) · on the robot

**Target:** the same matcher, running as a sensor: what it is worth over a drive, what an anchor is worth, and
why the numbers that look like quality meters (`kept correspondences`, `NEES`) are not one.

| min | what happens |
|---|---|
| 0–20 | the drive exists: `./tools/run_lab.sh drive` in the simulator; the wheel odometry walks 4 m off |
| 20–45 | read the scaffold together — the anchor, the guess, `advance()`, and why the given code is given |
| 45–150 | `scan_step()`. Their E2 matcher, with the guards. Re-grade freely |
| 150–180 | the σ question, and the two tables below on the projector |

**The one bug worth planning for** is the integration direction: `advance()` already inverts, and a student who
"fixes" it gets a drive that looks fine and ends 3 m away. Feed it exact truth steps and it comes out exactly
right — `test/test_exercises.py` does exactly that, and it is the ten-minute exercise that saves an hour.

**Numbers to put on the board** (one graded drive each, `production`, one 34 s course, wheels 12 % long and
0.10 rad/s yaw-biased — deliberately worse than the real robot's calibration residuals):

| | RMSE | vs the wheels | worst | rate | contacts |
|---|---|---|---|---|---|
| shipped template (the wheel step) | 3.97 m | **1.00×** | 7.35 m | 5.7 Hz | 0 |
| reference, guarded ICP | **1.07 m** | **3.74×** | 1.65 m | 5.7 Hz | 0 |
| threshold | ≤ 1.60 m | ≥ 2.0× | ≤ 2.60 m | ≥ 5 Hz | 0 |

**Ask, and listen for:**

* *"How many correspondences did your last pair keep?"* — the reference keeps 46 % and the task passes; a run that
  reports 74 % kept scores 5.26 m and fails. The keep count describes the match, not the pose. Write both on the
  board, in that order.
* *"What happens if you gate at 25 cm instead of 50?"* — let them run it, then put the three graded drives on the
  board: **1.281 m at 0.25 m, 1.065 m at 0.5 m, 1.046 m at 1.5 m**. Tightening the gate costs a fifth of a metre and
  widening it threefold buys nothing outside the run-to-run spread. A gate is not a quality knob; it is the door the
  correction comes through. Then ask which of the two they were about to tune, and on what evidence.
* *"Your NEES is 111. Is your σ a lie?"* — half of it. The error of an *integrated* pose is bias, not noise, and
  a per-pair σ widened by √pairs is honest about a fit, not about an integral — which is why NEES is deliberately
  **not** a criterion here, and why the fallback scores a *better* NEES (15.2) than the reference (111.3) while
  being four metres worse. The difference between a σ about a fit and a σ about a pose is the reason
  `kf/pose` is the graded interface of E5 and not of this exercise.
* *"Where would the pose be without the anchor?"* — in the frame of the first scan, offset from the hall origin by
  the robot's starting pose, about 3.5 m of it. That is not a detail: the same matcher scores 1.07 m and 5.26 m
  depending on it, and it is the one number the library node gets wrong for this task.

## M1 · 180 min · E4 (50) · offline

**Target:** the filter's bookkeeping — sampling, the motion model's five noise terms, systematic resampling, and
the estimate — separated from the sensor model, which is the only function E5 leaves open.

| min | what happens |
|---|---|
| 0–15 | the picture: `python3 student/particles_template.py` shows the cloud it should generate and does not |
| 15–55 | `sample_uniform`, `sample_gaussian`. **Hold the θ wrap discussion here** — it costs more marks than the rest |
| 55–100 | `move_particles`, then the two motion criteria. Both edges of the band, out loud |
| 100–145 | `resample`, then the 80/20 run, and the mode that dies |
| 145–180 | `weighted_estimate` and the wrap, then the four questions below |

**Ask, and listen for:**

* *"Your θ spread is 1.15 rad on a cloud that should be 0.5 rad wide. Where?"* — arithmetic mean of angles: a
  cloud spanning ±π averages to something near 0, and 0 is not in the cloud. Measured on the graded fixture, the
  naive mean puts the heading at **0.77 rad** where the circle mean says **3.10** and the cloud is centred on
  3.10. The fix is `atan2(Σ wᵢ sin θᵢ, Σ wᵢ cos θᵢ)`, and the criterion fails in that one number.
* *"You took 1200 uniform samples and 70 % landed on a wall. Is that wrong?"* — no. `resample` is never asked to
  avoid walls, and at 60 % wall coverage the reference lands 68 % of its samples there and still hits 81 % of the
  free 0.5 m cells. What is **not** acceptable is a sampler whose busiest quarter holds 6× its quietest.
* *"One second of driving, delivered as 20 updates or as 40: how much should the cloud spread?"* — the same
  amount, in the noise floors. Reference 0.419 / 0.423 / 0.416 m at 20 / 10 / 40 updates. A σ that does not carry
  √dt is a σ that changes when the LIDAR rate changes, and the MCL that passes on a 20 Hz bench fails on the
  robot's 5 Hz one. Note that this invariance is claimed for the **floors only**: the α terms are motion-proportional,
  and a group who demands rate invariance of the whole model has misread the lecture — say so before they build it.
* *"After resampling, how many particles of the dead mode survived?"* — the reference: 960 / 240 / 0. A
  systematic resampler that copies 5 % of a mode with 80 % of the weight is the failure that ends a localisation,
  and there is no criterion in E5 that catches it as directly as this one.

**Where the marks go:** 12 + 8 sampling, **18 the motion model**, **8 resampling**, 4 estimate. The motion model
is worth as much as everything else put together, because that is where the σs live and the σs are what E5 grades.

## M2 · 180 min · E5 (130) · on the robot, four graded drives

This is the existing task set and the existing handout — `docs/exercises.md` §"The four tasks", the PDF in
`docs/handout/`, and the per-task failure table in `student/FAILURE.md`. The three things a tutor adds to it:

* **They will arrive having done E1–E4.** Say which part of E5 each of those was: E1 is the inner loop of nothing
  in E5 (the sensor model is ray casting, not correspondence — worth saying out loud, so nobody "optimises" the
  wrong function), E2 is the σ they must not fudge, E3 is what the odometry they are comparing against is worth,
  E4 is every line of the template except the sensor model.
* **The two-minute viva is per group and decides the borderline marks**: what is `mean(x)` over a multimodal
  cloud, why is σ the spread of the cloud rather than the error, and what does a NEES of 0.19 mean for the next
  drive.
* **Do not let a group widen a σ to pass.** The shipped template's measured profile — an estimate 0.68 m out
  with a NEES of **0.12**, missing L4's band [0.3, 5] from *below* — is in `student/FAILURE.md`. A σ widened until
  NEES falls into 0.3 … 5 is the same trick as a hard-coded pose, with more steps, and the viva question that
  follows it is short.

## Marking

```bash
python3 tools/lab_check.py                                    # E1, E2, E4 — the file itself, in ~8 s
./tools/run_lab.sh grade --task icp_odom_production --controller student/icp_odom_template.py --headless
./tools/run_lab.sh grade --task mcl_production,mcl_wide,mcl_budget,mcl_dirty \
                         --controller student/mcl_template.py --headless
```

`tools/lab_check.py` is the mark scheme for the offline exercises and prints the measured value of every
criterion, not only pass/fail; the robot marks come from the simulator's own grader and are the marks that count.
Nothing is averaged across the two: a criterion says `required:` and `measured:`, and a group that disagrees with
one has a number to argue with, not a vibe.

The mark scheme is tested. `python3 tools/lab_check.py --check` and `python3 -m pytest test/test_exercises.py`
assert that the reference solutions meet every criterion and that the shipped templates earn **nothing** — so a
threshold nobody has ever met cannot survive a commit, and a template whose answer is accidentally already
written gets caught.
