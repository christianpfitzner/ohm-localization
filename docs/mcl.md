# Monte-Carlo localisation: the model, the parameters, and what each one is worth

The filter is `ohm_localization.mcl.MonteCarloLocaliser`; the node that runs it against the simulator is
`solution/mcl_node.py`; the measured numbers are in [`verification.md`](verification.md) §1–§4.

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
because `robot_io` hands out the last message it received, which is the same object until a new one arrives.
Weighting a scan twice double-counts the world, and the failure looks like a filter that is very sure of
itself. `tools/scan_probe.py` measures the clocks: scan and truth at 20.0 Hz, odometry at 30–50 Hz depending
on the session, all three strictly increasing. A node driven by the wall clock does not survive
`tools/fastgrade.py` in the simulator, which runs the same sessions 25× faster than real time.

## The motion model and its noise rates

The lecture's odometry model, in the `(rot1, trans, rot2)` decomposition:

```
s = [ α1·|rot1| + α2·trans ,  α2·trans + α1·(|rot1|+|rot2|) ,  α3·|rot2| + α2·trans ]
particle pose += the reading sampled with  N(0, s + rate·√dt)
```

Each particle gets its own draw, which is what lets the cloud widen again between two scans. With the noise
at zero the particles move as one rigid body and resampling can only shrink the cloud: measured, and asserted
by `test/test_mcl.py`, **0.274 m RMSE and NEES 749** against 0.012 m and 0.2 for the same filter with noise.
N_eff stays high, the reported σ falls, and the estimate follows the odometry.

The `α` terms are proportional to the motion, so they mean the same thing at any message rate. The floor is
what the wheels get wrong while the robot is standing still, so it is a rate:

```
floor = noise_rate_xy · √dt         (0.045 m·s^−½  ≈  10 mm per 20 Hz step)
```

A per-*step* floor instead of a per-*second* one is [`verification.md` §8 item 5](verification.md): the same
filter on the same data, predicted at the odometry rate (50 Hz), drifted √2.5 faster per second than the
offline replay that predicted per scan (20 Hz) — **0.20 m vs 1.26 m** on one recording. Two tests pin it: ten
seconds of standing still must spread the cloud by `rate·√10` whether the messages arrive at 20 Hz or 200 Hz.

## The sensor model

For each kept beam *i*, at a particle's pose *p*:

```
ẑ_i = distance from the predicted beam origin to the nearest wall along the beam      (exact: the field)
w_i ∝ exp(−(z_i − ẑ_i)² / 2σ_z²)     mixed with  z_rand  (a uniform "sensor is confused" term)
```

The beam count is a budget: `beam_stride = 3` uses 107 of 360 beams and costs nothing measurable in
`production` (13 mm either way), because beams 15° apart at 3 m see different walls and at 0.5 m see the same
one. Which beams you keep matters more than how many:

* **A beam that found nothing is a statement of the form `range > 8 m`.** Weighting it as though it read
  exactly 8 m invents a wall. That wall moves with the robot, so it constrains almost nothing: the mistake
  costs 1.6× in `production`, where few beams are empty, and **17× in `arena`**, where many are.
  `drop_uninformative` defaults to True. The simulator's `lidar_no_echo` launch arg decides whether such a
  beam arrives as `inf` or as `range_max`; the node handles both with a `keep` mask.
* **σ_z is a claim about the sensor, not a knob.** With a clean LIDAR in `production` it is almost
  irrelevant — 0.05 m to 0.8 m spans 12 mm to 14 mm RMSE, because 320 beams that each see a wall at 1–4 m
  overwhelm the width. With `lidar.sigma = 0.25` (task L4) the same parameter dominates: σ_z 0.15 → 96 mm and
  NEES 15–38; σ_z 0.50 → 49 mm and NEES 2.2. The optimum lands near **2 × σ_sensor** rather than at it:
  per-beam sharpness and posterior resolution are not the same quantity, and a 300-beam product is sharper
  than any single beam.

## N_eff counts particles; coverage is a different number

`N_eff = 1/Σwᵢ²` — a graded column here. `update()` measures it **before** resampling; after a resampling the
weights are uniform again and the number means nothing.

After resampling many particles are copies; copies predict identical beams and carry identical weights, so
N_eff climbs back toward N. Measured on a converged run in `production`: **N_eff 1200 while the whole cloud
stands in 14 boxes of 5 cm**, in a hall of 240 m². `MonteCarloLocaliser.diversity(cell=0.05)` reports the
boxes, and the node publishes both numbers in the `info` of every pose. A report that shows only N_eff can
show a filter that stopped exploring.

Resampling is **systematic** (one uniform random offset, then a deterministic sweep): with equal weights it
keeps all N particles, where a multinomial draw keeps about 63 % (`test/test_mcl.py`).

Injection — spawning random particles when N_eff collapses — is a recovery mechanism. On a recording of a
healthy drive: **0.530 m RMSE, never converged, 87 240 particles injected**, against 0.013 m with it off. It
is off by default (`inject_below = 0.0`) and `test/test_mcl.py` asserts the harm.

## σ is the thing you have to deserve

The graded quantity that is hard to earn is **NEES**: mean squared error divided by the reported variance,
which should be ≈ 1 for an honest filter.

| configuration | NEES | reading |
|---|---|---|
| L1 solution, graded | 0.19 | σ conservative (cloud σ ≈ 34 mm against 15 mm error, both after `docs/verification.md` §12) |
| 250 particles, 322 beams (L3) | 5.39 | a quarter of the particles is a quarter of the honesty budget |
| zero motion noise | **749** | confidently wrong |
| σ_z 0.15 with σ_sensor 0.25 (L4 done wrong) | **15–38** | reporting 5 mm around a 96 mm error |
| σ_z 1.0 in `arena` | 0.21 | honest because the cloud is enormous |

Three parameters move this number: the motion noise (`predict()` samples), `sigma_floor` (a filter may not
claim to know better than 1 cm / 0.01 rad, which caps the arrogance but does not create information), and the
resampling rate.

## Global localisation, and the budget it needs

A uniform prior over the hall is the honest version of "the robot was carried here". At 1200 particles it
does not work: `rooms` has **4432 free cells** at 25 cm, a third of a particle per cell, and no weighting
scheme can find a mode the cloud does not contain. Measured: 3.2 m and never better; 4000 particles converges
after 38 s; `arena` (5520 cells, mostly open) does not converge at 4000 either. Asserted in
`test/test_mcl.py`.

## What to measure, in the lab

1. **N_eff over a drive**, and the same for `diversity()`. Where does N_eff drop, and what is the robot
   looking at there? Where do the two disagree?
2. **rmse against odometry**, i.e. the improvement ratio, not the rmse alone: the ratio is what makes the
   number comparable between halls and groups, and it is the column the grader computes.
3. **NEES as a function of σ_z and of N.** Four σ_z values and two particle counts is eight numbers and one
   table — that is the whole of L4's grade and it takes three minutes with a recording.
4. **Convergence time** (first time the error stays under 25 cm): `mcl_report.py` reports it, and the
   difference between 0.0 s and 3 s tells you whether your prior or your likelihood did the work.
5. **Wall contacts of the estimate** (`occupied_at` on the estimate, not the robot): 0.0 % for a good filter,
   45 % for one running on a corrupted map ([`verification.md` §8 item 3](verification.md)).

## Questions for the protocol

* The task's `sim` block sets `odom.geometry.scale_xy = 1.03`. Why does that make the improvement column
  meaningful, and what would L1 be graded against without it?
* Your σ is 5 mm and your error is 96 mm. Name three parameters you could change and which of the three
  would make the σ honest rather than the error smaller.
* A beam that returned `range_max` is weighted out in this implementation. Write the likelihood term you
  would use instead, and predict its effect in `production` and in `arena` before you measure either.
* N_eff = 1200, `diversity()` = 14. Is the filter healthy? What would you have to change if the answer is
  no, and what number would tell you it worked?
* L3 runs 250 particles on a 53 s drive and grades 17 mm against L1's 15 mm on 34 s with 1200; the two runs
  differ in drive and in budget, so the difference is not a measurement of the particle count. Halve your own
  N on a recording (`mcl_report.py`) and ask what the extra millimetres consist of: fewer hypotheses, coarser
  resampling, or a worse posterior mean. Design one measurement that decides.
