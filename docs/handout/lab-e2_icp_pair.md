# E2 — One model, one scene — and the σ that goes with them

*I1 · 85 minutes of a 180-minute session · 30 points · file `student/icp_pair_template.py`*

Command that decides it: `python3 tools/lab_check.py e2_icp_pair`

Your function: `register(model, scene, T_guess=None) -> dict(x, y, theta, sx, sy, sth)`

One model scan and one scene scan of the same place, taken 1.2 m and 20 degrees apart, plus the truth of how they are related — and a matcher that returns a pose and says how sure it is. This is the exercise where the σ is the point, not the decoration: a pose without a σ is an assertion, and the viva asks where the number came from.

The convention, because it is the one-character bug that costs an hour: your answer is the transform that carries points from the frame of the **model** scan into the frame of the **scene** scan, which is `icp.relative(model_pose, scene_pose)` in that order. `ohm_localization/icp.py:relative()`'s docstring is the one-line proof, and getting it backwards scores a large error rather than an error message.

Iterate: correspondences from your E1 function, solve the weighted least-squares step, apply it, repeat until the step or the fitness stops improving. Point-to-line is the mode that survives a 20-degree start, and the library's implementation is available so that the fit is not the exercise — but write down, from your own iteration printout, what changed between iteration 0 and the last one.

## Do this, in this order

1. Get one pair right first: `--verbose` prints the twelve pairs, their sizes and their truths. Print the fitness at the identity guess, at the truth, and at your answer, and be sure you understand the ordering before you touch a threshold.
2. The loop with no σ. `python3 tools/lab_check.py e2_icp_pair --verbose` tells you the median and worst error over the twelve pairs — that is the accuracy part of the mark, and it is the same quantity the paper's table is.
3. `sx, sy, sth` from `JᵀJ` of the last iteration: invert it, scale by the residual variance, take the square root of the diagonal. Then run the same matcher on the corridor pair the grader hands you and look at what your numbers say there: in a bare corridor the fit along it is free, and the σ is **not** where that shows up — measured here, the σ along the corridor is 21 mm while the condition number of `JᵀJ` is 2644 against 9.0 in a hall with corners. One of those two numbers is telling the truth, and the criterion asks for it by name.
4. Then the guess, because the robot will not hand you the identity: `T_guess` 100 mm and 2° off is the criterion, and `tools/icp_eval.py --basin` is the sweep. Measured on these twelve pairs with the reference matcher: the 100 mm guess recovers 11 of the 12 to within 60 mm, while a 250 mm one leaves nine of them under 10 mm and puts the other three at 1.91 m, 4.89 m and 5.30 m. What makes those three recoverable is in the fitness column, not the error column: at those answers the mean correspondence distance is 342 … 565 mm against 28 … 30 mm at the truth — twenty times worse, and invisible if you print the step size and nothing else. That is why the first criterion here asks your fit to be as good as the truth's rather than merely converged, and why E3 refuses a fit on exactly this number.

## Protocol — what goes in the write-up

* Per-pair table for all twelve: points used, correspondences kept, iterations, final mean correspondence distance, error in mm and degrees, and the three σ. This table is the deliverable, and it is the same quantity the robot graded in I2 measures.
* The basin sweep of `tools/icp_eval.py`: the largest translation and rotation error of `T_guess` from which the pair still converges, in point-to-point and point-to-line mode, and one sentence on what the failures 1.9 m, 4.9 m and 5.3 m away have in common with each other.
* The corridor pair: your σ along it and across it, your condition number there and in the hall, and one sentence on which of the two numbers you would publish. Say why adding a prior cannot rescue a 21 mm σ — information adds, so a prior can only shrink one.
* The residual histogram at the last iteration against the simulator's 20 mm beam noise, and whether your reported σ is larger or smaller than it. Both answers are defensible; no number is not.

## Hand in

* `register()` passing all six of its criteria: `python3 tools/lab_check.py e2_icp_pair` prints PASS.
* The per-pair table above with all twelve rows, and the corridor row separated — with the σ and the condition number filled from your own fit, not from a constant.

## What is measured

| criterion | pts | what it measures | the ask | measured here |
|---|---|---|---|---|
| `fits-as-well-as-the-truth` | 4 | Mean correspondence distance at your answer against the mean at the truth, on every pair | never worse than 1.3× the truth's fit plus 5 mm | every pair at or under 1.00× |
| `translation-error` | 9 | |error| of the returned pose against the truth, over twelve non-degenerate pairs in `production` | median ≤ 15 mm and worst ≤ 60 mm | median 7.1 mm, worst 36.0 mm |
| `heading-error` | 5 | The same twelve pairs, in degrees | median ≤ 0.5° and worst ≤ 2.0° | median 0.09°, worst 0.54° |
| `sigma-not-invented` | 3 | The reported σ from pair to pair: its spread divided by its mean, on both position axes | at least 0.15 on both | 0.61 and 0.64, with σ running 1.6 … 9.2 mm over the twelve pairs |
| `degeneracy-is-reported` | 5 | The bare-corridor pair: the condition number reported, or a σ inflated on the strength of it | cond ≥ 100, or σ along the corridor ≥ 50 mm and ≥ 3× the across σ | cond 2644 against a median of 9.0 in the hall; the σ there is 21 mm, which is the lesson and not the answer |
| `survives-a-wrong-guess` | 4 | The same twelve pairs from a `T_guess` 100 mm and 2° wrong, and from the identity | at least 10 of 12 within 60 mm both times | 11 of 12 and 12 of 12 |

*The `measured here` column is the reference solution (`solution/icp_pair_solution.py`) run through the command above; the ask is the threshold in `config/exercises_localization.json`. Both are reproduced by `python3 tools/lab_check.py --check`. `docs/verification.md` §19 (and §14 for the four MCL drives) has the runs.*

**What this is for.** The loop, the σ from J^T J, and the two ways a matcher lies: the constant σ, and the correspondences that were gated but still counted.
