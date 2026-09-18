# E1 — The nearest neighbour inside ICP

*I1 · 85 minutes of a 180-minute session · 20 points · file `student/nn_template.py`*

Command that decides it: `python3 tools/lab_check.py e1_nn`

Your function: `nearest_neighbour(src, dst) -> (n,) int array of indices into dst`

ICP spends its entire life answering one question — for every point of one scan, which point of the other scan is nearest — and it answers it once per iteration, so registering the twelve pairs of E2 is a few million nearest-neighbour queries and nothing else. Two decisions decide whether that is usable: what you do with a beam that found nothing, and how you put the question to numpy.

The point cloud is given to you (`scan_points()`), including the rule that a beam which hit nothing is `inf` and a beam at `range_max` means "nothing in range", and neither of those is a point. Write `nearest_neighbour()` for `src` of shape (n, 2) and `dst` of shape (m, 2), returning the (n,) integer array of indices into `dst`. An empty `dst` is a `ValueError`, not an empty answer: a scan in which nothing echoed has no correspondences, and a matcher that registers against an empty cloud reports a pose with a covariance of zero.

Write the two-loop version first and print its time. Then write the version you would actually run and print its time. The number that goes into the protocol table is the ratio, not the smaller one: measured on the cloud of one real scan here, the same algorithm costs a third of a millisecond or fifty, depending only on how the question is put to numpy — and a matcher that is 150× too slow is not a matcher you can run at 20 Hz.

## Do this, in this order

1. `python3 student/nn_template.py` prints a demo of two clouds and says which criteria are missing. `python3 tools/lab_check.py e1_nn --verbose` is the grader, with the numbers.
2. The two-loop version. Note what its answer looks like when two points are equally near — you do not have to decide, the grader accepts either — and how long it takes on the 353 × 351 cloud of one real scan: 52 ms measured, which is 20 periods of 2.6 ms each and so already too slow for one scan period on its own.
3. One numpy expression for the same answer: `src[:, None, :] - dst[None, :, :]` is (n, m, 2) and its norm is the whole distance matrix; `argmin` along axis 1 is the answer. Measure it.
4. Same arithmetic in row blocks, a few hundred rows at a time, and with the squares written out (`dx*dx + dy*dy`) instead of `np.linalg.norm(..., axis=2)`. Measure it again and be careful which of the two changes you credit: on this fixture the blocked version ran 0.36 ms against 3.1 ms for the whole-(n, m, 2) `norm` form, and with 353 source rows the block never even splits — so the speed is the arithmetic staying in cache, and the blocking is about memory and nothing else. Do that arithmetic for three sizes and put all three in the table: one scan pair (353 × 351) is 1.9 MiB and blocking is pointless there; a scan against a 40 000-point map is 216 MiB per call; 40 000 × 40 000 is 24 GiB, which is not a slow answer, it is no answer.
5. Optional, five minutes with scipy installed: `cKDTree(dst).query(src)`. Write down at which cloud size the tree stops being a loss.

## Protocol — what goes in the write-up

* The milliseconds of all four implementations on the same cloud, from one `--verbose` run. The ratio of loops to broadcast, and the ratio of broadcast to KD-tree.
* The shape and size of the largest intermediate array each version makes, and what it costs at 40 000 x 40 000 points.
* What your code does with a beam whose range is `inf`, whose range is exactly `range_max`, and with a `dst` that is empty. One sentence each, with the line number that implements it.
* The mean and median correspondence distance at the identity guess from the demo output, and which of the two is the number you would trust. Say why.

## Hand in

* `nearest_neighbour()` passing all four of its criteria: `python3 tools/lab_check.py e1_nn` prints PASS.
* The protocol above, and the two timings in the same units as the grader prints them.

## What is measured

| criterion | pts | what it measures | the ask | measured here |
|---|---|---|---|---|
| `exact-on-every-shape` | 10 | Every point of `src` gets its nearest point of `dst`, on eight fixture clouds including 1×1, 1×500, 500×1, a 4×4 grid against itself where everything is an exact tie, and a real scan pair | all eight exact, the distances checked against an independent two-loop implementation | 8/8; a wrong `argmin` axis, a transposed cloud or a distance returned instead of an index cannot agree with the reference |
| `self-match-is-the-identity` | 3 | `nearest_neighbour(a, a)` is `arange(n)` | identity on 400 points | identity |
| `empty-target-raises` | 3 | An empty `dst` raises `ValueError` | ValueError, not an empty answer and not another exception type | ValueError |
| `measured-speed` | 4 | Wall-clock milliseconds on the cloud of one real scan (353 × 351 points), best of 5 | at or under 20 ms | 0.28 … 0.36 ms for the reference across runs on this machine (best of 5); the two-loop version takes 52 ms, the whole-(n, m, 2) `np.linalg.norm` form 3.1 ms, and the fastest thing that is still a Python loop over the rows takes 1.3 ms |

*The `measured here` column is the reference solution (`solution/nn_solution.py`) run through the command above; the ask is the threshold in `config/exercises_localization.json`. Both are reproduced by `python3 tools/lab_check.py --check`. `docs/verification.md` §19 (and §14 for the four MCL drives) has the runs.*

**What this is for.** The inner loop of both ICP modes, and the fact that a correspondence is an index and not a distance — point-to-line ICP fits a line through the neighbours of the corresponding point, and can only do that with an index.
