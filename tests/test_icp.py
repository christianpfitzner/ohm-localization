"""ICP: does a scan pair register, how well, and what does it believe while doing it?

Every threshold in here is a measured number from this checkout, not an aspiration; the runs behind
them are in `docs/verification.md`.  Four of the tests assert that the algorithm *fails*, which is
the point of an ICP exercise: the interesting behaviour of a scan matcher is where it stops being
right, and it stops being right in ways that its own output does not always admit.
"""
import math

import numpy as np
import pytest

from ohm_localization.gridmap import corridor_text                # noqa: F401
from ohm_localization import icp, synth
from ohm_localization.gridmap import GridMap, load_hall, parse_grid


def _scan_pair(hall, a, b, sigma=0.02, seed=21):
    """Two scans plus the truth of their relative pose, from two poses that are known to be free."""
    rng = np.random.default_rng(seed)
    return (synth.scan_dict(a, hall, t=0.0, sigma=sigma, rng=rng),
            synth.scan_dict(b, hall, t=1.0, sigma=sigma, rng=rng),
            icp.relative(a, b), (a, b))


_mean_match = icp.mean_match      # the library's, so the fixture and the tools ask the same question


def _pair(hall):
    """A pair with structure to register against: the identity must be a bad answer.

    The first honest version of this fixture took a free pose 1.35 m from another free pose and got
    an ICP that slid from the truth to the identity and reported a mean correspondence distance of
    3 cm doing it — not a bug in the registration but a bug in the pair: the robot was in a corner,
    both scans saw the same two walls, and in a corner a translation is nearly free.  So a candidate
    pair here is rejected unless the identity explains it badly (over 40 cm on average) while the
    truth explains it well.  A registration exercise needs a hall with something in it, and the
    fixture has to know that.
    """
    g = GridMap(hall)
    for theta in (0.0, math.pi / 2, math.pi, -math.pi / 2, 0.6, -0.6, 2.4, 3.7):
        for x in np.arange(1.5, hall.size[0] - 4.5, 0.5):
            for y in np.arange(1.5, hall.size[1] - 4.5, 0.5):
                a = (float(x), float(y), float(theta))
                b = (a[0] + 1.4 * math.cos(theta + 0.2), a[1] + 1.4 * math.sin(theta + 0.2),
                     a[2] + 0.4)
                if any(bool(g.occupied_at(px, py)) for px, py in
                       ((a[0], a[1]), (b[0], b[1]), ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2))):
                    continue
                sa, sb = synth.scan_dict(a, hall), synth.scan_dict(b, hall)
                pa, pb = icp.points_from_scan(sa)[0], icp.points_from_scan(sb)[0]
                if len(pa) < 200 or len(pb) < 200:
                    continue
                if _mean_match(pa, pb, np.eye(3)) < 0.4:          # the identity already fits: useless
                    continue
                if _mean_match(pa, pb, icp.relative(a, b)) > 0.08:  # the truth does not: bad pair
                    continue
                return _scan_pair(hall, a, b)
    raise AssertionError(f"{hall.name}: no non-degenerate 1.4 m scan pair found")


def _corridor_pair(hall, slide):
    """A pair in the middle of a long empty corridor, `slide` metres apart along it.

    Middle, deliberately: the ends of a 30 m corridor are 15 m away and the LIDAR stops reporting at
    8 m, so nothing in either scan knows where along the corridor the robot is.  Near an end, the
    end is visible and the pair stops being degenerate — which is a fact about the geometry, and the
    reason the first version of this helper, which searched the simulator's halls for a long run of
    free cells, kept finding pairs that were not degenerate at all.
    """
    cx, cy = hall.size[0] / 2.0, hall.size[1] / 2.0
    a, b = (cx, cy, 0.0), (cx + slide, cy, 0.0)
    return _scan_pair(hall, a, b)[:3]


# ---------------------------------------------------------------------------- the working cases
def test_a_pair_with_a_known_answer_gives_the_known_answer(hall):
    """Point-to-line, started 0.2 m off a 1.4 m / 23° pair: 5 mm and 0.003° in five iterations."""
    a, b, truth, _ = _pair(hall)
    res = icp.register_scans(a, b, T_guess=truth @ icp.se2(0.2, -0.1, 0.1), mode="line",
                             max_corr=1.0)
    dist, angle = icp.error_between(res.T, truth)
    assert res.converged, "the loop never stopped moving"
    assert dist < 0.01, f"translation error {dist * 100:.1f} mm"
    assert angle < 0.1, f"rotation error {angle:.4f} deg"
    assert res.iterations <= 10, f"{res.iterations} iterations for a pair this easy"
    assert res.correspondences > 250, f"only {res.correspondences} pairs kept"


def test_the_iteration_history_shows_the_descent(hall):
    """Every step on record, and the sequence falls: 20 cm of correspondence distance down to 3.7 cm.

    Not strictly monotone, and the test must not pretend otherwise — once the Newton step is below the
    tolerance the correspondence set flips between two almost identical assignments and the mean moves
    in the fifth decimal.  What is asserted is the shape that matters for the heat map: a large first
    descent and a final value at the floor the loop reached.
    """
    a, b, truth, _ = _pair(hall)
    res = icp.register_scans(a, b, T_guess=truth @ icp.se2(0.3, 0.1, 0.1), mode="line")
    assert len(res.history) == res.iterations
    fits = [h[1] for h in res.history]
    best = min(fits)
    assert fits[0] > 5 * best, f"barely moved: {fits[0]:.3f} -> {best:.3f} m"
    assert fits[-1] < 1.05 * best, f"still wandering at the end: {fits}"
    assert all(h[2] >= 0.0 and h[3] >= 0.0 for h in res.history), "step sizes are not magnitudes"


def test_point_to_line_beats_point_to_point_on_the_same_pair(hall):
    """Why the lecture prefers point-to-line, measured on one pair: 6 mm against 33 mm.

    Both are correct estimators in the limit, and on a LIDAR pair they are not close.  Nearest-
    neighbour matching between two fan-shaped clouds biases the answer — every point snaps to the
    closest stored point, which is systematically *along* the wall it came from — while the point-to-
    line residual only measures the distance to the surface, where the same snap is worth nothing.
    Measured on this pair: **6.2×** in translation and **263×** in rotation on identical input.
    `tools/icp_eval.py --claims` also reports the median over ten random pairs — 6.6× in translation
    but only 2.9× in rotation, because the rotation advantage needs a lever arm and a long wall, and
    a claim about a method should be quoted at the number that reproduces when the pairs change.
    """
    a, b, truth, _ = _pair(hall)
    guess = truth @ icp.se2(0.2, -0.1, 0.1)
    line = icp.register_scans(a, b, T_guess=guess, mode="line", max_corr=1.0)
    point = icp.register_scans(a, b, T_guess=guess, mode="point", max_corr=1.0)
    el, ep = icp.error_between(line.T, truth), icp.error_between(point.T, truth)
    assert el[0] < 0.01 and ep[0] < 0.05, f"line {el[0]:.4f} m, point {ep[0]:.4f} m"
    assert ep[0] > 3.0 * el[0], f"point-to-point only {ep[0] / el[0]:.1f}x worse: fixture too easy"
    assert ep[1] > 3.0 * el[1], f"rotation {ep[1]:.3f} vs {el[1]:.3f} deg"


def test_the_range_max_dialect_is_not_a_wall():
    """Publishing `range_max` where the sensor meant `inf` builds a wall out of nothing.

    In `rooms` this proves nothing — every beam finds something inside 8 m, so there is no `inf` to
    mistranslate (checked: the pair there has none).  In the open `arena` a third of the beams are at
    max range: 105 phantom points, and the registration of a pair that was 3.8 mm right is 40 mm right,
    with the mean correspondence distance doubled from 25 mm to 60 mm.  Same in `track` (46 phantom
    beams, 13 mm → 39 mm) and `production` (61, 8 mm → 39 mm).

    The reason it is not *worse* than that is worth knowing, and the bare corridor of the next test
    shows it: the locus of points at exactly `range_max` from the sensor is a circle around the sensor,
    so a phantom wall at the horizon pins almost nothing about the position — it is a lie the geometry
    can absorb.  It still enters the cost, it still gets counted as a correspondence, and in a filter
    that compares hypotheses over metres of hall (`tests/test_mcl.py`) the same one line costs a factor
    of seventeen.
    """
    a, b, truth, _ = _pair(load_hall("arena"))
    honest, _ = icp.points_from_scan(a)
    liar = dict(a)
    liar["ranges"] = [r if math.isfinite(r) else a["range_max"] for r in a["ranges"]]
    cheated, _ = icp.points_from_scan(liar, drop_uninformative=False)
    assert len(cheated) > len(honest), "no beam was at range_max in this hall: nothing to teach"
    dst = icp.points_from_scan(b)[0]
    good = icp.icp(honest, dst, T0=truth, mode="line", max_corr=1.0)
    bad = icp.icp(cheated, dst, T0=truth, mode="line", max_corr=1.0)
    eg, eb = icp.error_between(good.T, truth), icp.error_between(bad.T, truth)
    assert eg[0] < 0.01, f"reference registration in arena: {eg[0]:.4f} m"
    assert eb[0] > eg[0] + 0.02, f"phantom walls cost only {eb[0] - eg[0]:.4f} m ({eg[0]:.4f} → {eb[0]:.4f})"
    assert bad.fitness > good.fitness, "the fit did not even get worse: the phantom points were free"
    assert bad.correspondences > good.correspondences


def test_relative_carries_a_point_from_one_frame_into_the_other(hall):
    """`relative(a, b)` is the transform an algorithm has to produce; checked on one floor point."""
    a, b = (6.0, 8.0, 0.3), (6.9, 8.4, 0.75)
    floor = np.array([[9.0, 8.0]])                                # one point on the floor, world frame
    rot_a, rot_b = icp.se2(*a)[:2, :2], icp.se2(*b)[:2, :2]
    body_a = (floor - np.array(a[:2])) @ rot_a                    # world -> body of pose a
    in_b = icp.transform(icp.relative(a, b), body_a)               # body of a -> body of b
    world_again = in_b @ rot_b.T + np.array(b[:2])                # body of b -> world
    assert np.allclose(world_again, floor, atol=1e-9)


def test_both_nearest_neighbour_searches_answer_the_same():
    src = np.random.default_rng(1).uniform(0, 10, size=(200, 2))
    dst = np.random.default_rng(2).uniform(0, 10, size=(300, 2))
    brute = icp.nearest_neighbour(src, dst, method="brute")
    try:
        tree = icp.nearest_neighbour(src, dst, method="kdtree")
    except RuntimeError:
        pytest.skip("SciPy is not installed: brute force only, and the tool says so")
    assert np.array_equal(brute, tree)


def test_the_constrained_and_the_free_linearisation_agree_here(hall):
    """The 4-parameter form everyone reaches for is not the disaster the notes imply — on these pairs.

    It is kept switchable because the difference is worth measuring by hand rather than being told: on
    the structured pair in `rooms` the free form lands at 4.2 mm against the constrained form's 5.3 mm,
    in the same number of iterations.  What it does change is `dof`, and with it the meaning of the
    covariance: a 4-parameter fit describes a transform that is not a rigid motion, which is a
    sufficient reason on its own for the default.  A student who can show a pair where the free form
    *does* drift has found something this file has not, and should hand it in.
    """
    a, b, truth, _ = _pair(hall)
    src, dst = icp.points_from_scan(a)[0], icp.points_from_scan(b)[0]
    guess = truth @ icp.se2(0.2, -0.1, 0.1)
    free = icp.icp(src, dst, T0=guess, mode="line", unit_sincos=False)
    rigid = icp.icp(src, dst, T0=guess, mode="line")
    assert (free.dof, rigid.dof) == (4, 3)
    ef, er = icp.error_between(free.T, truth), icp.error_between(rigid.T, truth)
    assert ef[0] < 0.02 and er[0] < 0.02, f"free {ef[0]:.4f} m, constrained {er[0]:.4f} m"
    assert abs(ef[0] - er[0]) < 0.02, f"they disagree by {abs(ef[0] - er[0]):.3f} m: recheck this"


# --------------------------------------------------------------------- the derivative, measured
def _numeric_residual_jacobian(src, dst, T, gate=1.0, k=8, eps=1e-6):
    """d(residual)/d(x, y, theta) with the correspondences and the lines held where they are.

    Perturbing the pose and re-running the whole loop would compare the Jacobian against a moving
    target: the correspondence set changes discontinuously with the pose, and the finite difference
    then measures the fence, not the slope.  So the pairs and their line fits are frozen here and only
    the points move.
    """
    moved = icp.transform(T, src)
    idx = icp.nearest_neighbour(moved, dst)
    d = np.linalg.norm(moved - dst[idx], axis=1)
    keep = d < gate
    p = moved[keep]
    lines = icp._line_params(p, dst, idx[keep], k)
    c, nx_, ny_ = lines[:, :2], lines[:, 2], lines[:, 3]

    def res(dx, dy, dth):
        ca, sa = math.cos(dth), math.sin(dth)
        p0 = p.mean(axis=0)
        R = np.array([[ca, -sa], [sa, ca]])
        q = R @ (p - p0).T
        pp = (q.T + p0 + np.array([dx, dy]))
        return nx_ * (pp[:, 0] - c[:, 0]) + ny_ * (pp[:, 1] - c[:, 1])

    num = np.column_stack([(res(eps, 0, 0) - res(-eps, 0, 0)) / (2 * eps),
                           (res(0, eps, 0) - res(0, -eps, 0)) / (2 * eps),
                           (res(0, 0, eps) - res(0, 0, -eps)) / (2 * eps)])
    return p, lines, num, keep.sum()


def test_the_rotation_column_is_the_derivative_not_the_cross_product(hall):
    """The one line of point-to-line ICP that is wrong in most implementations, checked numerically.

    The residual of a pair is `r = n·(R p + t - c)`, and the lecture writes the derivative with respect
    to the angle as the cross product `n × p = n_x p_y - n_y p_x`.  The actual derivative is its
    **negative**, `n_y p_x - n_x p_y`, because `ξ×p = ξ(-p_y, p_x)`: copying the cross product into the
    third column of `J` mirrors the rotation half of the Jacobian while leaving the translation half
    untouched, so the loop still walks downhill and the bug is invisible for an iteration.

    Measured on a real pair, correspondences frozen: the column this file uses correlates **+1.000**
    with the finite difference, the cross-product spelling **−0.978**.  With that sign wrong the same
    code converged to a pose 53° away and a correspondence distance that had grown from 0.12 m to
    0.34 m — while point-to-point, which does not use this column, worked, which is the only reason
    the error was noticed.
    """
    a, b, truth, _ = _pair(hall)
    src, dst = icp.points_from_scan(a)[0], icp.points_from_scan(b)[0]
    p, lines, num, kept = _numeric_residual_jacobian(src, dst, truth @ icp.se2(0.2, -0.1, 0.1))
    assert kept > 100, f"only {kept} pairs: nothing to check"
    c, nx_, ny_ = lines[:, :2], lines[:, 2], lines[:, 3]
    q = p - p.mean(axis=0)
    correct = ny_ * q[:, 0] - nx_ * q[:, 1]
    cross_product = nx_ * q[:, 1] - ny_ * q[:, 0]
    assert np.corrcoef(correct, num[:, 2])[0, 1] > 0.999, "the column in icp.py is not the derivative"
    assert np.corrcoef(cross_product, num[:, 2])[0, 1] < -0.9, "the two spellings are not opposite?"
    assert np.max(np.abs(correct - num[:, 2])) < 1e-4
    assert np.allclose(num[:, 0], nx_, atol=1e-6)               # the easy columns, while we are here
    assert np.allclose(num[:, 1], ny_, atol=1e-6)


# ------------------------------------------------------------------------- where it stops working
def test_the_basin_of_attraction_has_an_edge(hall):
    """ICP is a local method: 0.4 m of initial error is nothing, 4 m and 1.4 rad is hopeless.

    Sweeping the size of the mistake the guess is allowed to contain is the content of the extra-credit
    heat map; the assertion is only that the edge is there.  A method that converged from any start
    would be a different, better and far more expensive algorithm — global registration by branch and
    bound or by feature triplets, which is not what a 180-minute lab is for.
    """
    a, b, truth, _ = _pair(hall)
    near = icp.register_scans(a, b, T_guess=truth @ icp.se2(0.4, 0.2, 0.12), mode="line",
                              max_corr=1.0)
    far = icp.register_scans(a, b, T_guess=truth @ icp.se2(4.0, 3.0, 1.4), mode="line",
                             max_corr=1.0)
    assert icp.error_between(near.T, truth)[0] < 0.01
    assert icp.error_between(far.T, truth)[0] > 1.0, \
        f"it recovered from 4 m and 80°: {icp.error_between(far.T, truth)[0]:.2f} m"


def test_a_bare_corridor_leaves_the_slide_free_and_the_covariance_only_just_notices(bare_corridor):
    """Degeneracy, and the trap inside it: a perfect fit that is 3 m wrong in an unconstrained scene.

    In 31 × 6 m of empty corridor whose ends are beyond the range, two poses 3 m apart see exactly the
    same thing: two parallel walls, forever.  The along-corridor direction is not measured, so the
    truth is not determined by the data — and point-to-point ICP, started at the identity, answers in
    **three iterations** with a mean correspondence distance of **20 mm**, a covariance of **1.2 mm**,
    and a pose **2.998 m** from the truth.

    The covariance does not shout, which is the second half of the lesson.  `JᵀJ` is rank-deficient in
    the free direction in exact arithmetic, but the beams graze the far geometry enough to leave a
    nonzero `n_x`, and the variance is floored at the sensor's own figure — so the reported σ stays at
    1.2 mm for an answer that is 3 m wrong, a factor of 2500.  Only `cond` (2379 for point-to-line
    against 6 on a structured pair) and the anisotropy of the ellipse (20 mm along the corridor against
    1.2 mm across it) say anything at all.  A covariance from a fit is a statement about the fit's
    curvature under an assumed noise model; it is not a statement about whether the scene can be
    localised, and nothing in the output says which of the two you are reading.

    Point-to-line does not even finish: it reaches 28 mm in three iterations and then keeps stepping
    along the free direction at a millimetre a go until the loop gives up at 30.  `converged` is a
    statement about the step size, and in an unconstrained direction the step size never becomes zero —
    a group that treats it as a correctness flag will be waiting for a number that never arrives.
    """
    a, b, truth = _corridor_pair(bare_corridor, 3.0)
    src, dst = icp.points_from_scan(a)[0], icp.points_from_scan(b)[0]
    point = icp.icp(src, dst, T0=np.eye(3), mode="point", max_corr=1.0)
    line = icp.icp(src, dst, T0=np.eye(3), mode="line", max_corr=1.0)
    assert point.converged and point.iterations <= 3, f"point: {point.iterations} iterations"
    assert point.fitness < 0.03, f"point: fit {point.fitness:.4f} m is not the noise floor"
    assert abs(icp.error_between(point.T, truth)[0] - 3.0) < 0.05, \
        f"point: error {icp.error_between(point.T, truth)[0]:.2f} m, expected the whole slide"
    assert max(point.sigmas[:2]) < 0.05, f"point: sigma {point.sigmas} — it admitted the doubt"
    #  Point-to-line does not finish at all: it reaches the noise floor in three iterations and then
    #  keeps stepping along the free direction at a millimetre a go, so the loop runs to its iteration
    #  limit.  `converged` is a statement about the step size, and in an unconstrained direction the
    #  step size never becomes zero — a group that treats it as a correctness flag will be waiting.
    assert not line.converged and line.iterations == 30, f"line stopped anyway: {line.history[-3:]}"
    assert line.fitness < 0.03, f"line: never got near a fit ({line.fitness:.3f} m)"
    assert line.cond > 100, f"condition {line.cond:.0f}: expected the free direction to show up"
    ev = np.linalg.eigvalsh(line.covariance[:2, :2])
    assert math.sqrt(ev[1] / max(ev[0], 1e-18)) > 5.0, f"anisotropy {ev}"


def test_a_repeating_hall_fits_perfectly_one_period_away(pillar_corridor):
    """Perceptual aliasing, made exact: the same scan four metres from itself.

    The corridor of the test above has nothing in it; this one has a post every 4 m, and the two poses
    are 4 m apart — exactly one period.  The scans are then the same measurement, so the identity is a
    perfect registration: **fit 0.021 m after two iterations, σ = 9.8 mm, pose 4.005 m wrong**.  Worse,
    the truth is the *worse* explanation of the pair — 0.43 m of correspondence distance against
    0.021 m — because the pillars cast shadows on each other from one pose and not from the other.  A
    20 m corridor is worth six of these aliases, and there is no threshold on the fitness and no
    covariance that can find the mistake, because the data genuinely do not distinguish the two poses.

    This is why a scan matcher is a local method and why it is paired with something that keeps a
    distribution over poses: MCL does not care how ambiguous the world is, it carries the ambiguity
    until the world stops being ambiguous.  It is also why a corridor with regularly spaced pillars is
    the most dangerous floor plan in this building.
    """
    a, b, truth = _corridor_pair(pillar_corridor, 4.0)
    src, dst = icp.points_from_scan(a)[0], icp.points_from_scan(b)[0]
    alias = icp.icp(src, dst, T0=np.eye(3), mode="line", max_corr=1.0)
    dist, angle = icp.error_between(alias.T, truth)
    assert alias.converged and alias.iterations <= 3
    assert dist > 3.5, f"it found the truth after all ({dist:.2f} m): the alias is not exact"
    assert alias.fitness < 0.05, f"the alias is not a good fit ({alias.fitness:.3f} m)"
    assert max(alias.sigmas[:2]) < 0.05, f"it admitted the doubt: {alias.sigmas}"
    assert _mean_match(src, dst, truth) > _mean_match(src, dst, np.eye(3)), \
        "the truth should explain the pair worse than the alias does — that is the whole point"
    near = icp.icp(src, dst, T0=truth @ icp.se2(0.3, 0.0, 0.0), mode="line", max_corr=1.0)
    assert icp.error_between(near.T, truth)[0] < 0.25, \
        "started near the truth it should stay near the truth, not fall into the alias"


def test_a_wrong_guess_that_is_near_the_truth_uses_the_gate_to_stay_away_from_the_opposite_wall(hall):
    """The correspondence gate is not a speed control, it is what keeps a match honest.

    Dropping pairs over a metre away is what stops the loop from registering a wall onto the wall
    opposite it — the classic way to end up with a good fit in the wrong place, which is available
    whenever the hall has two similar walls facing each other.
    """
    a, b, truth, _ = _pair(hall)
    src, dst = icp.points_from_scan(a)[0], icp.points_from_scan(b)[0]
    guess = truth @ icp.se2(0.5, 0.4, 0.2)
    tight = icp.icp(src, dst, T0=guess, mode="line", max_corr=1.0)
    loose = icp.icp(src, dst, T0=guess, mode="line", max_corr=6.0)
    et, el = icp.error_between(tight.T, truth), icp.error_between(loose.T, truth)
    assert et[0] < 0.01, f"with the gate on: {et[0]:.3f} m"
    assert loose.correspondences >= tight.correspondences
