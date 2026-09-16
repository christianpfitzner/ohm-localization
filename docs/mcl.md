# Monte-Carlo localisation: the model, the parameters, and what each one is worth

The filter is `ohm_localization.mcl.MonteCarloLocaliser`; the node that runs it against the simulator is
`solution/mcl_node.py`; the measured numbers are in [`verification.md`](verification.md) §1–§4. This file
is about *why* the pieces are the shape they are, and which parameter is worth your attention.

```
  odometry (30–50 Hz)                       LIDAR (20 Hz), 360 beams, range 8 m
        │                                            │
   predict_odometry ── motion model ─▶  N particles ─┤  weights: log-likelihood of the scan
        │                                            ▼
        │                                      N_eff = 1/Σwᵢ²  ── below N/2 ─▶ systematic resampling
        ▼                                            │
   estimate (x̂, ŷ, θ̂, σx, σy, σθ) ◀────────────────┘   →  kf/pose  at ≥ 5 Hz, graded against /truth
```

One scan is one update; one odometry message is one prediction. Both are keyed on the **message stamp**,
because `robot_io` hands out the *last* message it received, which is the same object until a new one
arrives — weighting a scan twice would double-count the world, and the failure looks like a filter that
is very sure of itself. (`tools/scan_probe.py` measures the clocks rather than assuming them: scan and truth at a
clean 20.0 Hz, odometry at 30–50 Hz depending on the session, all three strictly increasing. `tools/fastgrade.py` in the simulator runs the same sessions 25× faster
than real time, which a wall-clock-driven node would not survive.)

## The motion model, and why its noise is a rate

The lecture's odometry model, in the `(rot1, trans, rot2)` decomposition:

```
s = [ α1·|rot1| + α2·trans ,  α2·trans + α1·(|rot1|+|rot2|) ,  α3·|rot2| + α2·trans ]
particle pose += the reading sampled with  N(0, s + rate·√dt)
```

Each particle gets its **own** draw. That sampling is the whole reason the cloud can widen again between
two scans, and it is where the exercise's most instructive failure is produced: set the noise to zero and
the particles move as one rigid body, so resampling can only shrink the cloud. Measured (`tests/test_mcl.py`
asserts it): **0.274 m RMSE and NEES 749**, against 0.012 m and 0.2 for the same filter with noise — and
it does not look broken. N_eff stays high, the reported σ falls, and the estimate simply follows the
odometry. A filter that has convinced itself it already knows stops listening.

The `α` terms are proportional to the motion, so they are the same physical statement whatever the message
rate: half the variance of a 1 cm step and a 1 cm-per-step sequence is the same over a metre of driving.
The **floor** is different in kind — it is what the wheels get wrong while the robot is standing still —
and it therefore has to be a rate:

```
floor = noise_rate_xy · √dt         (0.045 m·s^−½  ≈  10 mm per 20 Hz step)
```

Getting this wrong is [`verification.md` §8 item 5](verification.md): with a per-*step* floor, the same
filter on the same data predicted at the odometry rate (50 Hz) drifted √2.5 faster per second than the
offline replay that predicted per scan (20 Hz) — **0.20 m vs 1.26 m** on one recording of one drive, and
for a while the honest conclusion was that the filter could not localise in that hall. Two tests now pin
it: ten seconds of standing still must spread the cloud by `rate·√10` whether the messages arrive at 20 Hz
or at 200 Hz.

## The sensor model

For each kept beam *i*, at a particle's pose *p*:

```
ẑ_i = distance from the predicted beam origin to the nearest wall along the beam      (exact: the field)
w_i ∝ exp(−(z_i − ẑ_i)² / 2σ_z²)     mixed with  z_rand  (a uniform "sensor is confused" term)
```

The beam count is a **budget**, not a detail: `beam_stride = 3` uses 107 of 360 beams and costs nothing
measurable in `production` (13 mm either way, because the beams are correlated: 15° apart at 3 m is a
different patch of wall, at 0.5 m it is the same one). Which beams you *keep* matters more than how many:

* **A beam that found nothing is a statement of the form `range > 8 m`.** Weighting it as though it read
  exactly 8 m invents a wall — and since that wall moves with the robot, it constrains almost nothing, so
  the mistake is nearly free in a hall where few beams are empty (1.6× worse in `production`) and very
  expensive in one where many are (**17× worse in `arena`**). `drop_uninformative` defaults to True; the
  simulator's `lidar_no_echo` launch arg decides whether such a beam arrives as `inf` or as `range_max`,
  and the node handles both, which is a 3-line `keep` mask rather than a topic parameter.
* **σ_z is a claim about the sensor, not a knob.** With a clean LIDAR in `production` it is almost
  irrelevant — 0.05 m to 0.8 m spans 12 mm to 14 mm RMSE, because 320 beams that each see a wall at 1–4 m
  overwhelm the width. Change the sensor (`lidar.sigma = 0.25`, task L4) and the same parameter dominates
  everything: σ_z 0.15 → 96 mm and NEES 15–38; σ_z 0.50 → 49 mm and NEES 2.2. The optimum lands near
  **2 × σ_sensor** rather than at it, which is worth an exam question: per-beam sharpness and posterior
  resolution are not the same quantity, and a 300-beam product is sharper than any single beam.

## N_eff is about the weights; the cloud's coverage is a different number

`N_eff = 1/Σwᵢ²` — the lecture's practice problem, and a graded column here. `update()` measures it
**before** resampling; after a resampling the weights are uniform again and the number is meaningless.

It counts *particles*, and that is its limitation. After resampling, many particles are copies; copies
predict identical beams, so they carry identical weights, so N_eff climbs back toward N. Measured on a
converged run in `production`: **N_eff 1200 while the whole cloud stands in 14 boxes of 5 cm**, in a hall
of 240 m². `MonteCarloLocaliser.diversity(cell=0.05)` reports the boxes, and the node puts both in the
`info` of every published pose. A filter whose N_eff is high because it holds 1200 copies of one pose is
not the filter the formula was written to describe, and a report that shows only N_eff can show a healthy
filter that stopped exploring.

Resampling is **systematic** (one uniform random offset, then a deterministic sweep), which is the
low-variance option and measurably so: with equal weights it keeps all N particles, where a multinomial
draw keeps about 63 % of them and calls that a sample (`tests/test_mcl.py`).

Injection — spawning random particles when N_eff collapses — is a *recovery* mechanism, and on a
recording of a healthy drive it is a disaster: **0.530 m RMSE, never converged, 87 240 particles
injected**, against 0.013 m with it off. It is off by default (`inject_below = 0.0`) and the test asserts
the harm, because the temptation to "fix a stuck filter" with it is strong and the evidence is the
opposite.

## σ is the thing you have to deserve

Accuracy is easy to plot. The graded quantity that is hard to earn is **NEES** — mean squared error
divided by the reported variance, which should be ≈ 1 for an honest filter:

| configuration | NEES | reading |
|---|---|---|
| L1 solution, graded | 0.35 | σ somewhat conservative (cloud σ ≈ 34 mm against 27 mm error) |
| 250 particles, 322 beams (L3) | 5.39 | a quarter of the particles is a quarter of the honesty budget |
| zero motion noise | **749** | confidently wrong |
| σ_z 0.15 with σ_sensor 0.25 (L4 done wrong) | **15–38** | reporting 5 mm around a 96 mm error |
| σ_z 1.0 in `arena` | 0.21 | honest *because* the cloud is enormous |

Three things change this number and students should have to say which: the motion noise (`predict()`
samples), `sigma_floor` (a filter may not claim to know better than 1 cm / 0.01 rad, which caps the
arrogance but does not create information), and the resampling rate.

## Global localisation, and the budget it needs

A uniform prior over the hall is the honest version of "the robot was carried here". At 1200 particles it
does not work: `rooms` has **4432 free cells** at 25 cm, so that is a third of a particle per cell, and no
weighting scheme can find a mode the cloud does not contain. Measured: 3.2 m and never better; 4000
particles converges after 38 s; `arena` (5520 cells, mostly open) does not converge at 4000 either. The
assertion is in `tests/test_mcl.py`, and the number is in the task text, because "increase the particle
count" is a different lesson from "improve the likelihood" and the lecture's kidnapped-robot section is
about exactly this.

## What to measure, in the lab

1. **N_eff over a drive**, and the same for `diversity()`. Where does N_eff drop, and what is the robot
   looking at there? Where do the two disagree?
2. **rmse against odometry**, i.e. the improvement ratio, not the rmse alone: the ratio is what makes the
   number comparable between halls and groups, and it is the column the grader computes.
3. **NEES as a function of σ_z and of N.** Four σ_z values and two particle counts is eight numbers and one
   table — that is the whole of L4's grade and it takes three minutes with a recording.
4. **Convergence time** (first time the error stays under 25 cm): `mcl_report.py` reports it, and the
   difference between 0.0 s and 3 s tells you whether your prior or your likelihood did the work.
5. **Wall contacts of the estimate** (`occupied_at` on the estimate, not the robot): 0.0 % for a good
   filter, 45 % for one running on a corrupted map ([`verification.md` §8 item 3](verification.md)).

## Questions for the protocol

* The task's `sim` block sets `odom.geometry.scale_xy = 1.03`. Why does that make the *improvement* column
  meaningful, and what would L1 be graded against without it?
* Your σ is 5 mm and your error is 96 mm. Name three parameters you could change and which of the three
  would make the σ *honest* rather than the error smaller.
* A beam that returned `range_max` is weighted out in this implementation. Write the likelihood term you
  would use instead, and predict its effect in `production` and in `arena` before you measure either.
* N_eff = 1200, `diversity()` = 14. Is the filter healthy? What would you have to change if the answer is
  no, and what number would tell you it worked?
* With 250 particles the same hall gives 55 mm instead of 27 mm. What does the extra 28 mm consist of —
  fewer hypotheses, coarser resampling, or a worse posterior mean? Design one measurement that decides.
