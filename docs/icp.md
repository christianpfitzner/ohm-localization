# ICP: registering two scans, and what the result is worth

The library is `ohm_localization/icp.py`; `tools/icp_eval.py` measures everything below and exits non-zero
if the document drifts from it (`--claims`). Where MCL gives you a posterior and a map, ICP gives you a
*relative pose between two scans* and a number telling you how well they fit — which is why it is in this
exercise at all: it is the sensor model inside every laser odometry and every pose-graph localiser, and it
never once tells you that it is wrong.

```
  scan A (360 beams) ──points──▶ src ──┐                       T maps src onto dst
  scan B (360 beams) ──points──▶ dst ──┤  repeat:               truth = icp.relative(a, b)
                                       ▼                       error = icp.error_between(res.T, truth)
        1. nearest neighbour of each src point in dst          (brute force; cKDTree if SciPy is present)
        2. residual per pair          point:  q − T·p        (2 components)
                                      line:   n·(q − T·p)    (1 component, n from the k nearest of q)
        3. Gauss–Newton step on SE(2), with max_corr gating the outlier pairs
        4. until |Δξ| < tolerance or 30 iterations
        result: T, fitness, correspondences, covariance (JᵀJ)⁻¹·s², cond(JᵀJ)
```

Point-to-point minimises the distance *between stored points*; point-to-line minimises the distance to the
*surface* those points came from. That distinction is the whole of the accuracy difference measured below,
and both are correct estimators in the limit.

## What it is worth, measured

Twelve pairs in `production` at σ_beam = 20 mm (`tools/icp_eval.py`), truth being the transform between the
two poses that generated the scans:

| variant | median |Δt| | median |Δθ| | fit | reported σ | cond | iterations | converged |
|---|---|---|---|---|---|---|---|
| point-to-point | 34.6 mm | 0.264° | 55.1 mm | 6.45 mm | 24.6 | 18.0 | 12/12 |
| point-to-line | **8.6 mm** | **0.076°** | 49.7 mm | 5.42 mm | 17.4 | **5.5** | 11/12 |

The `--claims` run (10 pairs, seed 17) quotes **6.6×** in translation and **2.9×** in rotation; on the
deterministic fixture pair in `test/test_icp.py` (a 1.4 m move with 0.4 rad of turn in `rooms`) it is
**6.2×** and **263×**. Both sentences are true. The rotation advantage needs a lever arm against a long
wall, so it is large on a pair that has one and small on a median over random pairs — which is why the
executable claim checks the median and quotes the pair-level number with its provenance attached.

Why the difference is so big in the first place: nearest-neighbour matching between two *fan-shaped*
clouds is biased. Every source point snaps to the closest stored point, which is systematically **along**
the wall it came from — the residual you minimise is not the one you think you are minimising. The
point-to-line residual measures distance to the surface, where that snap is worth nothing. It also
converges in a third of the iterations (5.5 against 18.0), because a line constraint couples the three DOF
instead of pulling each point at right angles to itself.

On **real beams** from a graded drive (`--recorded /tmp/prod.jsonl`, σ_sensor 15 mm, truth from the
simulator's `/truth`): point 16.0 mm / 0.272°, point-to-line **6.0 mm / 0.040°** — so the synthetic
geometry is not flattering the method. But note the converged column: 10/12 for line against 12/12 for
point. The accurate variant is also the one that occasionally fails to settle inside 30 iterations, which
matters the moment something downstream trusts it blindly.

Raising the beam noise (12 pairs, median |Δt|): 5 mm → 9 mm; 20 mm → 9 mm; 50 mm → point 45 mm vs line
10 mm; 150 mm → point 96 mm vs line 40 mm. Accuracy degrades close to linearly in σ_beam, and the advantage
narrows from ~4× to ~2.4× — a dirty sensor flattens the differences between estimators, exactly as it did
for the particle filter's σ_z in task L4.

## The basin of attraction is a size, not a property

`--basin` starts the match from a deliberately wrong guess (displaced along x) and reports the median error
of what comes out:

| σ_beam | mode | 0 | 0.25 | 1.0 | 2.0 | 3.0 | 5.0 m of wrong guess |
|---|---|---|---|---|---|---|---|
| 20 mm | point | 35 mm | 36 | 67 | **1840** | 3087 | 5088 |
| 20 mm | line | 9 mm | 9 | 15 | **2468** | 3614 | 5135 |
| 150 mm | point | 95 mm | 94 | 102 → **1190** | | | |
| 150 mm | line | 40 mm | 40 | 43 | 54 | **2861** | 4957 |

A clean sensor in a structured hall tolerates about a metre of wrong guess and nothing at two; with a dirty
window point-to-point gives up at 1 m while point-to-line still recovers from 2 m. So "ICP converges" is
not a statement — "ICP converges from under 1 m of error with this sensor in this hall" is. This is also why
the pairs used for the accuracy table are chosen with a real motion between them: the identity is often a
plausible answer for two scans taken in the same corner, and a matcher that slides from the truth to the
identity has done correct arithmetic on a question with two answers.

## The two geometries that break it, on purpose

The simulator's halls are all too interesting to demonstrate this, so `gridmap.corridor_text` generates
31 × 6 m of nothing (0.5 m cells; posts every 4 m in the second case) and the robot is put in the middle
with its 8 m range reaching neither end. `--degenerate`:

| corridor | slide | variant | result | reported σ | cond | fit at truth vs at estimate |
|---|---|---|---|---|---|---|
| bare | 3.0 m | point | 2.995 m wrong, converged in 3 it | 2.55 mm | **12.0** | 158.2 vs 22.2 mm |
| bare | 3.0 m | line | 2.962 m wrong, converged in 3 it | 2.30 mm | **2406** | 158.2 vs 28.4 mm |
| posts / 4 m | 4.0 m | point | 4.002 m wrong, 2 it | 2.24 mm | 10.5 | 474.9 vs 18.7 mm |
| posts / 4 m | 4.0 m | line | **3.999 m wrong**, 3 it | 3.48 mm | 23.5 | **474.9 vs 18.6 mm** |

**A slide along a bare corridor is not measurable by a 2D LIDAR.** Two infinite walls give you the distance
between them and the heading, and nothing along the corridor. The solver converges in three iterations,
reports a σ of 2.5 mm, and is 3 m wrong: **the σ understates the error by 1175×**. Nothing in the result is
inconsistent — the covariance is correctly derived from a Jacobian that correctly describes a problem with a
flat direction. What is missing is the *knowledge that the problem is flat*, and that is what `cond(JᵀJ)` is
for:

* point-to-line: **cond 2406**. It models the walls, so it notices that the walls say nothing about the
  direction along themselves.
* point-to-point: **cond 12.0** — healthy. It never models a wall, so it has nothing to be suspicious about.
  A warning you do not print is a warning you do not have; this pair of numbers is the reason `cond` is in
  `IcpResult` at all.

**Aliasing is worse, because there the fit itself lies.** Put a post every 4 m and slide the robot exactly
one period: the match lands 3.999 m away, converges in three iterations, and its fitness is **18.6 mm
against 474.9 mm at the true pose** — the aliased pose fits the scan **25× better than the truth does**.
ICP did not fail. It minimised the objective it was given, and the objective has a better minimum at the
wrong pose. No threshold on fitness, no iteration limit and no covariance tells you this; the only defences
are a starting guess within the basin (odometry, a previous scan, a coarse global search) or geometry that
does not repeat. That is the argument for why real localisation stacks are pose graphs and not ICP calls.

## The covariance, and what it is a covariance of

`s²` is the residual variance scaled by `max_corr`-gated correspondences, `C = (JᵀJ)⁻¹ s²`, three diagonal
entries (x, y, θ). `res.sigmas` unpacks them. It is a *local* linearised uncertainty about the fit — not a
bound on the error, and least of all in the two geometries above, where the estimate is a different mode
rather than a noisy version of the right one. `unit_sincos` keeps the rotation small inside the linear
system instead of carrying an unconstrained 4th parameter; measured against the free form on the same pair
it differs by about **1 mm** and nothing else, so it is not a knob to tune — the only thing that moves
results here is `mode`, `max_corr` and the geometry.

## Two bugs this file's numbers cost

Both are in [`verification.md` §8](verification.md) items 1–2; they belong here too because they are the
reason the accuracy tables are phrased as measurements.

1. The solver updated its transform and never wrote it to the result: `res.T` stayed the identity, and the
   fitness numbers were **excellent** because the fitness is a correspondence distance and the identity is a
   respectable answer for two scans taken in the same hall. Only checking `error_between(res.T, truth)` on a
   pair where the identity is a 1.4 m error exposed it. If you take one engineering lesson from this
   repository: an accuracy metric that does not move when the answer is wrong is not a metric.
2. The rotation column of the point-to-line Jacobian was transcribed from the lecture's cross-product form
   `n × p` instead of differentiated. The numeric-vs-analytic correlation was **+1.000** where the correct
   column gives **−0.978**: sign and scale were entangled, so Gauss–Newton still converged and only the
   covariance and the last millimetres were wrong. A solver that converges is not a solver that is right.

## To use it

```python
from ohm_localization import icp
res = icp.register_scans(scan_a, scan_b, T_guess=icp.se2(0.2, -0.1, 0.1), mode="line", max_corr=1.0)
res.pose            # (x, y, theta)  — T that carries A's cloud onto B's frame
res.sigmas          # (sx, sy, sth)
res.fitness, res.cond, res.iterations, res.converged
icp.error_between(res.T, icp.relative(pose_a, pose_b))          # (m, deg) against the truth
icp.mean_match(src, dst, icp.inverse(icp.relative(a, b)), max_corr=1.0)   # fitness *at* a given transform
```

`T_guess` is not optional in a system that works: it is the odometry, the previous scan's result, or a coarse
search. Passing the identity because it is convenient is how you get the bare-corridor number above as your
"result".

## Stitching scans into odometry — and what it costs

`ohm_localization/icp_odom_node.py` runs ICP between *consecutive* scans and integrates them: no map, no
prior, no particles. It is the control experiment for the whole exercise, and the numbers are in
`docs/verification.md` §13. The short version, on the graded drive:

| | one pairwise step | integrated over the drive |
|---|---|---|
| point-to-line | 7 mm, 0.04° | **0.91–1.76 m** |
| point-to-point | 7 mm, 0.04° | 0.78–1.2 m |
| the wheel odometry | ~30 mm | 0.07–0.18 m |
| the map-based filter of L1 | — | **0.015 m** |

Two things have to be true before that table means anything, and `test/test_icp_odometry.py` asserts both.
The first is a **control**: integrating the *exact* per-step transforms taken from `/truth` through the same
accumulator must reproduce the exact path (`< 1e-9`). The version shipped for an hour chained `T @ res.T`
where the robot's increment is `T @ inverse(res.T)` — `icp.relative(a, b)` carries *points* from the old frame
into the new one — and drifted 4.9 m on a drive whose true transforms were the input. A test that feeds an
integrator only its own algorithm's output cannot tell an integration bug from a bad algorithm.

The second is that the drift is **bias, not noise**, and that it belongs to one objective: mean signed
rotation error per step is −0.051° (line) against −0.0009° (point) on the recording, summing to −37° of
heading over 726 steps, where independent noise would have given 5.9°. A bias added N times grows linearly.
It survives `sigma_z`, the linearisation's `unit_sincos` scaling and every beam stride, so it is a property of
the linearised point-to-line objective in a hall of flat walls. Which is the honest answer to "why is your ICP
not working on the real robot": a 6 mm pairwise accuracy quoted without a horizon is not a claim, and
rotation in a corridor is the degenerate direction — the same degeneracy as `--degenerate` above, arriving by
itself one step at a time.

The node therefore reports σ as the **median** pairwise σ widened by √N — median, because the last pair is
chosen by the clock and not by the geometry (its σ_θ runs 0.13° at the median to 0.66° at its worst on the
graded drive), and widened, because a pairwise covariance published as a pose covariance claims millimetres for
a 36 s drive. Even √N under-claims a bias.

## Questions for the protocol

* Point-to-line is 6.6× better in translation and 2.9× better in rotation *on a median over pairs*, and
  263× better in rotation on one particular pair. What property of that pair makes the difference, and how
  would you choose test pairs if you had to report one number for a hall?
* In the posted corridor the estimate fits 25× better than the truth. Write down two things you could
  change — one in the algorithm, one in the mission — that would prevent this, and say what each costs.
* `cond` is 12 for point-to-point and 2406 for point-to-line on the same scan pair, and both are 3 m wrong.
  Which one do you trust, and what would you add to the point-to-point result to make it as honest?
* The basin ends near 1 m with a clean sensor and 2 m with a 150 mm one for point-to-line. Why does a
  noisier sensor *widen* the range of starting guesses that work for one variant?
* A colleague reports "ICP gave 6 mm and σ 5 mm, so it works". Which of the columns in this file would you
  ask for before believing them, and in what order?
