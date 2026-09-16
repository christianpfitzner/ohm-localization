"""Is the σ that ICP reports the σ the error has? A Monte-Carlo answer, because nothing else asks.

`tests/test_icp.py` checks the covariance's *shape* — that it shrinks with more correspondences, that the
degenerate geometry makes it big, that the numbers are consistent between two variants on one pair. None
of that says whether the σ describes the error. The only way to ask is to run the same registration many
times with fresh noise and count how often the truth lands where the reported σ says it should: a
coverage test, which is what a real sensor-fusion course would demand of a Kalman filter (NEES) and what
nobody thinks to demand of ICP because ICP prints a fitness value that looks like evidence.

What the counts say on this checkout, and why the numbers are phrased as they are below:

* **1σ is not a 68 % event in 2 DOF.** Per axis, |error| < σ happens 68.3 % of the time; the *box* (both
  axes, independent) is 0.683² = **46.6 %**; the 1σ *ellipse* — Mahalanobis distance ≤ 1 — is
  1 − e^(−½) = **39.3 %**; the 3-DOF (x, y, θ) 1σ ellipsoid is **19.9 %**. A test that asserted "68 %
  inside 1σ" would fail a perfectly calibrated filter, which is exactly the mistake this file is here to
  make impossible.
* On a structured pair in `production` (120 noise draws, σ_beam 20 mm, 85 points per scan, starting from
  the truth so the sweep measures noise and not convergence): point-to-line gives box **26.7 %**, ellipse
  **24.2 %**, 3-DOF **12.5 %**, and mean squared Mahalanobis distance **3.20** where a calibrated 2-DOF
  fit gives 2.00 — i.e. the reported σ is about **1.27× too small**. Mildly overconfident, and
  explainable: the nearest-neighbour correspondences are chosen *after* the noise is realised, so each
  residual is biased toward the pair the solver picked for it, `max_corr` then truncates the tail that
  would have revealed it, and the yaw term is linearised. Nothing here pretends 1.27× is a bug: a
  filter that is 27 % optimistic on σ is a filter worth using.
* Point-to-point on the same 120 realisations: mean squared Mahalanobis **28.5**, σ short by **3.8×**,
  median translation error 34.7 mm against a reported σ of 6.6 mm, and the 1σ box caught **0 of 120**
  runs. This is the number to quote when someone argues that the accuracy difference between the two
  variants does not matter — it is not 10 % of a σ, it is no coverage at all.
* The target cloud is not free to thin. At `stride` 4 on 60 draws: **6.5 mm** median error with
  `stride_dst` 1, **133 mm** with `stride_dst` 4, mean yaw error −0.005° against −0.404°. The wall normal
  that point-to-line measures its residual along is fitted through target points; see
  `test_the_target_cloud_is_not_free_to_thin`.

Runtime: the structured block is ~1.1 s per mode, the `stride_dst` block ~2.3 s at 60 draws and the
corridor block ~6 s for both modes at 120 draws.
"""
import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

from ohm_localization import icp, synth                                     # noqa: E402
from ohm_localization.gridmap import (GridMap, corridor_text, load_hall,    # noqa: E402
                                      parse_grid)

# The pair is built by the tool that reports the accuracy numbers, not by a copy of it here.  Duplicating
# the rejection rules (0.6 m off the walls, at least 120 echoing beams, 0.4–1.2 m of motion) would give
# this file a geometry that is *nearly* the one documented in docs/icp.md, and "nearly" is how a coverage
# percentage stops being the one in the documentation.
import icp_eval                                                             # noqa: E402

DRAWS = 120
SIGMA_BEAM = 0.02
IDEAL = {"box": 0.6827 ** 2, "ellipse": 1.0 - math.exp(-0.5), "solid3": 0.199, "maha2": 2.0}


@pytest.fixture(scope="module")
def structured_hall():
    """`production`, named here rather than taken from the shared `hall` fixture (`rooms`).

    The coverage percentages in this file's header are quoted for the hall docs/icp.md measures ICP in,
    and the two halls are not interchangeable here: point-to-point's 1σ box covers 0 of 120 runs in
    `production` and 48 of 120 in `rooms`, because a 22 × 16 m hall of small rooms constrains the
    nearest-neighbour snapping that biases the point-to-point residual in the first place. Asking for the
    free-standing fixture and getting somebody else's default hall is how a measured test turns into a
    failing one.
    """
    return load_hall("production")


def _pair(hall, seed=17):
    """One structured pair: poses, scans-with-noise-free-beams and the truth between them."""
    return icp_eval.synthetic_triples(hall, 1, noise=SIGMA_BEAM, seed=seed)[0]


def _draws(pose_a, pose_b, hall, T_truth, mode, draws=DRAWS, seed0=1000):
    """(Mahalanobis², inside-box, inside-ellipse, |error|) over independent beam-noise realisations.

    One rng per realisation per scan, so a run is reproducible from its index and the whole block can be
    re-run after a refactor without wondering which of the changes moved the numbers.
    """
    maha2, box, ellipse, errors = [], [], [], []
    for k in range(draws):
        sa = synth.scan_dict(pose_a, hall, sigma=SIGMA_BEAM, rng=np.random.default_rng(seed0 + k))
        sb = synth.scan_dict(pose_b, hall, sigma=SIGMA_BEAM, rng=np.random.default_rng(4 * seed0 + k))
        res = icp.register_scans(sa, sb, T_guess=T_truth, stride=4, mode=mode, sigma_z=SIGMA_BEAM)
        e = icp.inverse(T_truth) @ res.T                 # the error, expressed in the truth's frame
        sx, sy, sth = res.sigmas
        dx, dy = float(e[0, 2]), float(e[1, 2])
        maha2.append((dx / sx) ** 2 + (dy / sy) ** 2)
        box.append(abs(dx) < sx and abs(dy) < sy)
        ellipse.append((dx / sx) ** 2 + (dy / sy) ** 2 <= 1.0)
        errors.append(math.hypot(dx, dy))
    return (np.array(maha2), np.array(box, dtype=bool), np.array(ellipse, dtype=bool),
            np.array(errors))


def test_the_sigma_of_a_structured_pair_is_a_little_optimistic_and_says_so(structured_hall):
    """1.27× optimistic, measured 120 ways: the number a student should leave in the protocol.

    Asserting a *band* rather than a floor, because both directions are informative. If the mean squared
    Mahalanobis distance drifts toward 2.00 the covariance has become honest — worth knowing, since the
    reason it is above 2 (correspondences selected after the noise, `max_corr` truncating the residual
    tail) is a property of the algorithm, not of this hall. If it drifts toward 10 the σ has become a
    decoration, which is what the next test says about the other variant.
    """
    _, _, T_truth, (pose_a, pose_b) = _pair(structured_hall)
    m2, box, ell, err = _draws(pose_a, pose_b, structured_hall, T_truth, "line")
    assert m2.mean() == pytest.approx(3.20, abs=0.9), \
        f"mean Mahalanobis² {m2.mean():.2f} (2.00 would be calibrated, 3.20 was measured)"
    assert box.mean() / IDEAL["box"] > 0.4, \
        f"1σ box caught {box.mean():.3f} of {DRAWS}; the ideal is {IDEAL['box']:.3f}"
    assert 0.10 <= ell.mean() <= 0.55, \
        f"1σ ellipse held {ell.mean():.3f} against the {IDEAL['ellipse']:.3f} a calibrated 2-DOF fit gives"
    assert np.median(err) < 0.010, f"median error {np.median(err):.4f} m — the accuracy claim moved"


def test_point_to_point_reports_a_sigma_that_covers_nothing(structured_hall):
    """0 of 120 runs inside its own 1σ box: the accuracy difference between the variants is a lie of σ.

    Point-to-point on the identical realisations: median error 34.7 mm against a reported σ of 6.6 mm,
    mean squared Mahalanobis 28.5, so its σ is short by a factor of 3.8. It is a fine *estimate* — 35 mm
    in a hall is not a disaster — but everything downstream that reads its covariance (a pose graph's
    information matrix, a fusion filter's gating, a "is this match trustworthy?" threshold) would be
    reading a number that has no coverage behind it.
    """
    _, _, T_truth, (pose_a, pose_b) = _pair(structured_hall)
    m2_line, _, _, _ = _draws(pose_a, pose_b, structured_hall, T_truth, "line")
    m2, box, _, err = _draws(pose_a, pose_b, structured_hall, T_truth, "point")
    assert box.mean() == 0.0, f"point-to-point covered {box.mean() * DRAWS:.0f} of {DRAWS} runs — " \
                              f"the measured 0 makes the point and a non-zero value needs re-reading"
    assert m2.mean() > 10.0, \
        f"mean Mahalanobis² {m2.mean():.1f} is no longer grossly optimistic (2.00 calibrated, " \
        f"28.5 measured); did the correspondence bias go away?"
    assert m2.mean() / m2_line.mean() > 4.0, \
        f"point-to-point is {m2.mean() / m2_line.mean():.1f}× worse calibrated than point-to-line " \
        f"(measured 8.9×)"
    assert np.median(err) > 0.020, \
        f"median error {np.median(err) * 1000:.1f} mm — the 5× -of-σ gap over its reported 6.6 mm σ " \
        f"(measured 34.7 mm) is what this test is about"


def _stride_draws(hall, pose_a, pose_b, T_truth, stride_dst, draws=60, seed0=1000):
    """(|error|, signed yaw error) of one structured pair at `stride` 4 with a `stride_dst` target."""
    err, yaw = [], []
    for k in range(draws):
        sa = synth.scan_dict(pose_a, hall, sigma=SIGMA_BEAM, rng=np.random.default_rng(seed0 + k))
        sb = synth.scan_dict(pose_b, hall, sigma=SIGMA_BEAM, rng=np.random.default_rng(4 * seed0 + k))
        res = icp.register_scans(sa, sb, T_guess=T_truth, stride=4, stride_dst=stride_dst,
                                 mode="line", sigma_z=SIGMA_BEAM)
        e = icp.inverse(T_truth) @ res.T
        err.append(math.hypot(float(e[0, 2]), float(e[1, 2])))
        yaw.append(math.degrees(icp.triple_of(e)[2]))
    return np.array(err), np.array(yaw)


def test_the_target_cloud_is_not_free_to_thin(structured_hall):
    """6.5 mm with a dense target, 133 mm with a thinned one: `stride_dst` is the expensive parameter.

    Point-to-line fits each wall through the nearest *target* points and measures the residual along the
    normal of that fit. Removing source beams only removes measurements. Removing target beams widens
    the span the line is fitted through, so the fitted normal is tilted by the along-wall discretisation
    — and a tilt enters the residual as a systematic error, the same sign on every draw, which is a bias
    rather than noise. `register_scans` therefore thins the source with `stride` and leaves the target at
    `stride_dst=1`.

    Measured on 60 draws of the structured pair, both clouds at `stride` 4, σ_beam 20 mm: median error
    6.5 mm against 133 mm, mean yaw error −0.005° against −0.404°. The failure mode this pins is someone
    tidying the two parameters into one so a single `stride` thins both clouds.
    """
    _, _, T_truth, (pose_a, pose_b) = _pair(structured_hall)
    err_dense, yaw_dense = _stride_draws(structured_hall, pose_a, pose_b, T_truth, 1)
    err_thin, yaw_thin = _stride_draws(structured_hall, pose_a, pose_b, T_truth, 4)
    assert np.median(err_dense) < 0.010, \
        f"dense target: median error {np.median(err_dense) * 1000:.1f} mm — the accuracy of the pair moved"
    assert np.median(err_thin) > 10 * np.median(err_dense), \
        f"thinning the target cost {np.median(err_thin) / np.median(err_dense):.1f}× " \
        f"(measured 20×): the wall fit stopped depending on the target cloud"
    assert abs(yaw_dense.mean()) < 0.05, \
        f"a dense target carries {yaw_dense.mean():+.3f}°/pair of yaw — the unbiased case is not unbiased"
    assert abs(yaw_thin.mean()) > 0.2 and np.sign(yaw_thin.mean()) == np.sign(np.median(yaw_thin)), \
        f"thinned target: mean {yaw_thin.mean():+.3f}°, median {np.median(yaw_thin):+.3f}° — " \
        f"the systematic bias (measured −0.40°) went random or away"


def _corridor_draws(grid, mode, slide=3.0, draws=DRAWS, seed0=1000):
    """Coverage per axis for a slide *along* a bare corridor: the direction nothing measures.

    The corridor's own axes are used rather than the world's, because the corridor is placed along x and
    the point of the test is which axis is which: along = the flat direction, across = the one the two
    walls do constrain.
    """
    cells = grid.free_xy
    mid = (float(np.mean(cells[:, 0])), float(np.mean(cells[:, 1])), 0.0)
    pose_a, pose_b = mid, (mid[0] + slide, mid[1], mid[2])
    T_truth = icp.relative(pose_a, pose_b)
    along, across, sig_along, sig_across = [], [], [], []
    for k in range(draws):
        sa = synth.scan_dict(pose_a, grid.hall, sigma=SIGMA_BEAM, rng=np.random.default_rng(seed0 + k))
        sb = synth.scan_dict(pose_b, grid.hall, sigma=SIGMA_BEAM, rng=np.random.default_rng(4 * seed0 + k))
        res = icp.register_scans(sa, sb, stride=4, mode=mode, sigma_z=SIGMA_BEAM)
        e = icp.inverse(T_truth) @ res.T
        sx, sy, _sth = res.sigmas
        along.append(abs(float(e[0, 2])) < sx)
        across.append(abs(float(e[1, 2])) < sy)
        sig_along.append(sx)
        sig_across.append(sy)
    return (np.array(along, dtype=bool), np.array(across, dtype=bool),
            np.array(sig_along), np.array(sig_across))


@pytest.fixture(scope="module")
def bare_corridor_grid():
    """31 × 6 m of nothing: 8 m range reaches neither end, so the slide along it is unmeasurable."""
    return GridMap(parse_grid(corridor_text(), cell=0.5, name="bare corridor"))


def test_in_a_bare_corridor_only_the_direction_that_was_measured_is_covered(bare_corridor_grid):
    """Along the corridor 0 %, across it 75 % — and the covariance of point-to-line knows the difference.

    A 3 m slide along a corridor with nothing in it lands 3.004 m wrong (measured median over 120 draws)
    while reporting σ 42.6 mm along and 2.3 mm across. So the along-axis coverage is 0.000 and the
    across-axis coverage 0.750 — honest-to-optimistic in the direction the walls constrain, and nothing at
    all in the one they do not. The two axes having *different* σ is the redeeming feature: σ is 18× wider
    along the corridor than across it, so the covariance does carry a degeneracy warning, for the variant
    that models the walls.

    Point-to-point on the same realisations reports σ 2.5 mm along and 2.5 mm across — an aspect ratio of
    1.0 — and is 3.000 m wrong. It has no warning to give because it never formed an opinion about the
    wall, which is the same lesson `--claims` prints for `cond` (2406 vs 12), now in the units a caller
    actually reads.
    """
    along_l, across_l, s_along_l, s_across_l = _corridor_draws(bare_corridor_grid, "line")
    assert along_l.mean() < 0.05, f"the flat direction was covered {along_l.mean():.2f} of the time"
    assert across_l.mean() > 0.50, f"the constrained direction stopped being plausible: {across_l.mean():.2f}"
    aspect_l = float(np.median(s_along_l) / np.median(s_across_l))
    assert aspect_l > 5.0, f"point-to-line's σ is only {aspect_l:.1f}× wider along the corridor " \
                           f"(measured 18.5×): the degeneracy warning faded"

    along_p, across_p, s_along_p, s_across_p = _corridor_draws(bare_corridor_grid, "point")
    assert along_p.mean() < 0.05, "point-to-point covered the flat direction, which it cannot measure"
    assert float(np.median(s_along_p) / np.median(s_across_p)) < 2.0, \
        "point-to-point started reporting the flat direction — it has no business knowing about it"
