"""E1 — the nearest-neighbour search of ICP, written by hand.  One function is yours: `nearest_neighbour`.

    python3 tools/lab_check.py e1_nn                              # the check the sheet names
    python3 student/nn_template.py                                # a small demo, no grader needed

**Why this is the first exercise and not a warm-up.** ICP spends its whole life answering one question —
for every point of one scan, which point of the other scan is closest — and answers it once per iteration,
so an ICP run over 12 model/scene pairs is a few million nearest-neighbour queries and nothing else. The
two decisions that make or break it are both in this one function: **what you do with a beam that found
nothing** and **how you ask numpy the question**. Everything else in E2 and E3 is arithmetic on top of the
answer you write here.

**What is already here.** `scan_points()` turns a raw scan into the point cloud ICP works on, dropping the
beams that carry no information, and `demo()` prints a picture of what a correspondence is. The check
itself (`tools/lab_check.py`) is in the repository and is honest about what it measures — run it once
before you write anything and read the lines it prints: three of its criteria fail, and they fail with a
`NotImplementedError` that says which one is yours.

**What is yours — `TODO(E1)` and nothing else.**

    nearest_neighbour(src, dst) -> indices
        src: (n, 2) float array, the points to ask about      dst: (m, 2) float array, the points to answer from
        returns: (n,) int array, `out[i]` = index into `dst` of the point nearest to `src[i]`

Two conventions that are not ours to invent and are the two ways to fail this quietly:

* The answer is an **index into `dst`**, not a distance and not a point. E2 needs the index because
  point-to-line ICP fits a line through the *neighbourhood* of the corresponding point, which it can only
  find from an index.
* An empty `dst` is a `ValueError`, not an empty array: a scan in which nothing echoed has no correspondences
  to be found, and an ICP that silently registers against an empty cloud reports a pose with a covariance of
  zero. `ohm_localization/icp.py` raises for the same reason.

Three ways to write it, and the check measures all three. A double Python loop is correct and slow (it is the
right first answer, and the sheet asks for its time). A numpy broadcast `src[:, None, :] - dst[None, :, :]`
builds the whole n×m×2 difference and is ~100× faster on the 360×360 cloud of one real scan. The chunked
version of the same thing — the one in the library — is what you want when n and m are in the tens of
thousands, because the full matrix is then a gigabyte and numpy's speed is not the problem any more. The
measured numbers of all three are what the protocol table is for; the lecture's animation of the same
comparison is the picture of them.

Do not read `ohm_localization/icp.py:nearest_neighbour()` before yours works: it is four lines shorter than
yours will be, and the sentence "I vectorised it" is worth nothing in the viva if you cannot say what the
intermediate array costs.
"""
from __future__ import annotations

import math
import sys
import time

import numpy as np

BEAMS = 360               # beams of one simulated scan, the cloud size the protocol table measures on


# --------------------------------------------------------------------- the point cloud of one scan
def scan_points(scan, stride: int = 1) -> np.ndarray:
    """(n, 2) body-frame points of one scan. Given — this is the part that is not the exercise.

    A beam that hit nothing is `inf`, and a beam at `range_max` means "nothing within range"; both are a
    statement about what is *not* there and neither is a point. Feeding either into a centroid drags the
    whole registration towards the sensor, which is the single most common way to ruin an ICP silently —
    see `ohm_localization/icp.py:points_from_scan()`, which does exactly this and counts what it dropped.
    """
    ranges = np.asarray(scan["ranges"], dtype=float)
    step = float(scan.get("angle_increment", 2.0 * math.pi / max(len(ranges), 1)))
    a0 = float(scan.get("angle_min", 0.0))
    idx = np.arange(0, len(ranges), max(int(stride), 1))
    ok = (ranges[idx] > float(scan.get("range_min", 0.05))) & np.isfinite(ranges[idx]) \
        & (ranges[idx] < float(scan.get("range_max", 8.0)) - 0.06)
    idx, r = idx[ok], ranges[idx][ok]
    return np.column_stack([r * np.cos(a0 + idx * step), r * np.sin(a0 + idx * step)])


# -------------------------------------------------------------------------------------- yours: TODO
def nearest_neighbour(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Index in `dst` of the point nearest to each point of `src`. **TODO(E1): write this.**

    `src` is (n, 2), `dst` is (m, 2), the answer is an (n,) integer array. `dst` empty is a `ValueError`.

    Start with the two loops, print its time, then write the vectorised version and print that. The number
    that goes into the protocol is the ratio of the two, not the smaller one: the point of the exercise is
    that the same algorithm costs 1 ms or 200 ms depending on how the question is put to numpy, and that a
    matcher which is 100× too slow is not a matcher you can run at 20 Hz.
    """
    raise NotImplementedError("TODO(E1): nearest_neighbour() — see tools/lab_check.py e1_nn")


# ------------------------------------------------------------------------------------ the slow path
def nearest_neighbour_loop(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """The same answer, written the way you would write it in C: two loops, no numpy in the inner one.

    Given, so the speed criterion has something to compare against that is *yours* in algorithm and not in
    code: if your vectorised version is not faster than this, it is not vectorised.
    """
    if len(dst) == 0:
        raise ValueError("nearest_neighbour: the target cloud is empty")
    out = np.empty(len(src), dtype=np.int64)
    for i in range(len(src)):
        best, best_d = 0, float("inf")
        for j in range(len(dst)):
            d = (dst[j, 0] - src[i, 0]) ** 2 + (dst[j, 1] - src[i, 1]) ** 2
            if d < best_d:
                best, best_d = j, d
        out[i] = best
    return out


def timed(fn, src: np.ndarray, dst: np.ndarray, repeats: int = 3) -> float:
    """Best of `repeats` wall-clock milliseconds of `fn(src, dst)` — the number for the protocol table."""
    best = float("inf")
    for _ in range(max(int(repeats), 1)):
        t0 = time.perf_counter()
        fn(src, dst)
        best = min(best, (time.perf_counter() - t0) * 1000.0)
    return float(best)


# ------------------------------------------------------------------------------------------------- demo
def demo(beams: int = BEAMS) -> None:
    """Two clouds, one moved, and the correspondences printed as distances — what E2 will iterate on."""
    from ohm_localization import synth                                   # numpy only, no simulator
    from ohm_localization.gridmap import load_hall

    hall = load_hall("rooms")
    a = synth.scan_dict((4.0, 3.0, 0.2), hall, t=0.0, beams=beams, sigma=0.01)
    b = synth.scan_dict((4.35, 3.1, 0.28), hall, t=0.05, beams=beams, sigma=0.01)
    pa, pb = scan_points(a), scan_points(b)
    print(f"rooms: {len(pa)} points from the model scan, {len(pb)} from the scene scan "
          f"({beams} beams each, the beams that found nothing dropped)")
    try:
        idx = nearest_neighbour(pa, pb)
    except NotImplementedError as exc:
        print(f"  {exc}\n  the loop version, which is given, says:")
        idx = nearest_neighbour_loop(pa, pb)
    d = np.linalg.norm(pa - pb[idx], axis=1)
    print(f"  mean correspondence distance at the identity guess: {d.mean() * 1000:5.1f} mm "
          f"(median {np.median(d) * 1000:5.1f} mm, worst {d.max() * 1000:5.1f} mm)")
    print("  that is the number ICP drives down; E2 is the loop that does it, E3 is the robot that asks "
          "for it 20 times a second")


if __name__ == "__main__":
    sys.path.insert(0, __file__.rsplit("/", 2)[0])          # run it from anywhere: python3 student/…
    demo()
