"""The offline criteria of the lab exercises: are they met by the reference and missed by the template?

Two assertions carry the weight here, and both are about the mark scheme rather than about numpy.

**A reference must pass and a shipped template must earn nothing.** A threshold nobody has met is a rumour, and
a template that scores points is a template that teaches a group to write their answer somewhere else. The pair
is checked for every offline exercise, through the same code path `tools/lab_check.py` runs — one grader, one set
of numbers, no second implementation of the mark scheme in the test suite. It costs a few seconds, and the
alternative is a sheet that a cohort chases.

**The scaffolding around the TODO has to be right, or the exercise grades the wrong thing.** `E3`'s node
integrates pairwise transforms; `test_icp_odometry.py`'s control for the library's node is reused here on the
exercise's own scaffold — feed the *exact truth* of a drive through `advance()`/`step()` and the result must be
that drive. An integration sign or an inverted transform does not fail visibly: integrating the exact per-step
transforms of a 34 s drive wanders metres while the output still looks like a localiser.

The statistical criteria are seeded (`ohm_localization.exercises.SEED`), so the numbers in
`config/exercises_localization.json` and on the sheets are reproducible from this file rather than merely
usually true.
"""
import importlib.util
import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ohm_localization import exercises as ex                                      # noqa: E402
from ohm_localization import icp, synth                                           # noqa: E402
from ohm_localization.gridmap import GridMap, load_hall                               # noqa: E402

BOOK = ex.exercise_file()
OFFLINE = [e for e in BOOK["exercises"] if e.get("kind") == "offline"]


def module_of(relpath: str, name: str):
    return ex.load_module(os.path.join(ex.ROOT, relpath), name=name)


# -------------------------------------------------------------------------------------------- the contract
@pytest.mark.parametrize("e", OFFLINE, ids=lambda e: e["id"])
def test_the_criteria_in_the_code_are_the_criteria_on_the_sheet(e):
    """A criterion renamed in one place and not the other is a mark that cannot be awarded."""
    assert {c["id"] for c in e["criteria"]} == {name for name, _p, _f in ex.CRITERIA[e["id"]]}


@pytest.mark.parametrize("e", OFFLINE, ids=lambda e: e["id"])
def test_the_reference_solution_meets_every_criterion(e):
    """Every threshold on the sheet has been met, on this machine, in this run."""
    res = ex.run(e["id"], os.path.join(ex.ROOT, e["solution"]), exercises=BOOK)
    missed = [r["criterion"] + ": " + r["measured"] for r in res["rows"] if not r["ok"]]
    assert not missed, f"{e['solution']} fails its own sheet: {missed}"
    assert res["points"] == res["max_points"]


@pytest.mark.parametrize("e", OFFLINE, ids=lambda e: e["id"])
def test_the_shipped_template_earns_nothing(e):
    """The template fails, and fails because its TODO is empty rather than because it crashes.

    Zero points is the assertion; *which* criteria fail is the template's documented failure profile and belongs
    in `student/FAILURE.md`, where it is kept honest by `tools/check.sh --live` for the robot exercises.
    """
    res = ex.run(e["id"], os.path.join(ex.ROOT, e["file"]), exercises=BOOK)
    assert res["points"] == 0.0, (f"{e['file']} already earns {res['points']:g}/"
                                  f"{res['max_points']:g} — the exercise's answer is not behind its TODO")
    assert all("TODO" in r["measured"] or "NotImplemented" in r["measured"]
               for r in res["rows"] if not r["ok"]), \
        f"{e['id']}: a criterion failed for a reason other than the missing function: " \
        f"{[r['measured'] for r in res['rows'] if not r['ok'] and 'TODO' not in r['measured']]}"


def test_a_missing_module_says_so():
    with pytest.raises(FileNotFoundError):
        ex.load_module(os.path.join(ex.ROOT, "student", "no_such_template.py"))


def test_an_unknown_exercise_names_the_ones_that_exist():
    with pytest.raises(KeyError) as e:
        ex.exercise(BOOK, "e9_laser_catcher")
    assert "e1_nn" in e.value.args[0]


# ---------------------------------------------------------------------------------------- the fixtures
def test_the_pairs_are_pairs_worth_registering():
    """The rejection rule of `scan_pairs()`, asserted on the pairs the mark is made of.

    The identity must explain a pair badly and the truth must explain it well; a pair taken in a corner satisfies
    neither and would score a matcher that did nothing.
    """
    for p in ex.scan_pairs(n=4):
        assert set(p) >= {"model", "scene", "truth", "poses", "points"}
        pa, pb = icp.points_from_scan(p["model"])[0], icp.points_from_scan(p["scene"])[0]
        assert icp.mean_match(pa, pb, np.eye(3)) > 0.4, "the identity already fits: the pair measures nothing"
        assert icp.mean_match(pa, pb, p["truth"]) < 0.03, "the truth does not fit: a wrong pose would fit too"
    a, b = ex.scan_pairs(n=2)[0]["poses"], ex.scan_pairs(n=2)[1]["poses"]
    assert a != b, "the fixture is seeded: two calls must still give the same pairs, in the same order"


def test_the_corridor_pair_is_the_degeneracy_the_criterion_claims():
    """Both ends out of range, the free direction along x, and `cond` the number that notices."""
    pair = ex.corridor_pair()
    cx = pair["hall"].size[0]
    a, b = pair["poses"]
    assert abs(a[1] - b[1]) < 1e-9 and abs(a[2]) < 1e-9, "the corridor pair must slide along its own axis"
    assert a[0] > 8.0 and b[0] < cx - 8.0, "an end wall in range would constrain the slide and hide the lesson"
    r = icp.register_scans(pair["model"], pair["scene"], mode="line", sigma_z=0.02, max_iterations=40)
    assert r.cond > 100.0, f"condition {r.cond:.0f}: the fixture is not degenerate, so the criterion tests nothing"
    assert max(r.sigmas[:2]) < 0.05, "if the σ admitted the freedom on its own, the criterion would be trivial"


# ------------------------------------------------------------------------------------ E3's scaffolding
def test_integrating_exact_truth_steps_reproduces_the_drive():
    """The control before believing anything about the matcher: exact steps in, exact drive out.

    A pair's transform is fed in as the answer of `scan_step`, from the same synthetic drive the ICP tests use, so
    the only thing under test is `IcpOdom` — the anchor, the inversion in `advance()` and the accumulation. An
    inverted transform here does not raise; it produces a plausible-looking pose walk that ends metres away.
    """
    mod = module_of("student/icp_odom_template.py", "ohm_e3_scaffold_test")
    hall = load_hall("production")
    grid = GridMap(hall)
    path = synth.free_path(grid, vx=0.4, seconds=6.0, sine=0.4, seed=7)
    assert path is not None, "no drive fits this hall: the fixture, not the exercise, is broken"
    scans = [synth.scan_dict(pose, hall, t=i * 0.05) for i, pose in enumerate(path)]

    state = {}

    def exact(prev_scan, scan, guess, cfg):
        i = state["i"]
        state["i"] = i + 1
        return icp.relative(path[max(i - 1, 0)], path[i]), (0.01, 0.01, 0.01)

    state["i"] = 0
    f = mod.IcpOdom(step_fn=exact)

    class _Odom:
        def __init__(self, p):
            self.x, self.y, self.theta, self.t = p[0], p[1], p[2], 0.0

    for i, scan in enumerate(scans):
        f.step(scan, _Odom(path[i]))
    got = np.array(f.pose())
    want = np.array(path[-1])
    assert np.allclose(got[:2], want[:2], atol=1e-6), f"{got} after {len(scans)} exact steps, want {want}"
    assert abs(((got[2] - want[2] + math.pi) % (2 * math.pi)) - math.pi) < 1e-6


def test_the_e3_fallback_is_the_odometry_and_says_so():
    """The shipped node's `scan_step` returns the wheel step: improvement 1.00, not a crash.

    `docs/FAILURE.md` quotes improvement = 1.00 for the unmodified template; that number is this line, and a
    fallback that instead returned the identity would give a *failing* baseline that teaches nothing about what the
    wheels are worth.
    """
    mod = module_of("student/icp_odom_template.py", "ohm_e3_fallback_test")
    guess = icp.se2(0.0, 0.0, 0.2) @ icp.se2(0.4, 0.0, 0.0)
    T, sig = mod.scan_step(None, None, guess, {"mode": "line", "stride": 2, "sigma_z": 0.02, "max_corr": 0.5})
    assert np.allclose(T, guess)
    assert len(sig) == 3 and all(0.0 < float(s) < 1.0 for s in sig)


def test_the_e3_sigmas_grow_with_the_number_of_pairs():
    """A pairwise σ widened by √pairs — the difference between a σ about a fit and a σ about a pose."""
    mod = module_of("student/icp_odom_template.py", "ohm_e3_sigma_test")
    f = mod.IcpOdom(step_fn=lambda p, s, g, cfg: (np.eye(3), (0.01, 0.01, 0.01)))
    scan = {"ranges": [1.0] * 20, "angle_min": 0.0, "angle_increment": 0.1, "range_min": 0.05,
            "range_max": 8.0, "t": 0.0}
    starts = None
    for k in range(400):
        scan["t"] = k * 0.05
        f.step(scan, None)
        if k == 0:
            starts = f.sigmas()
    late = f.sigmas()
    assert f.pairs >= 300
    for a, b in zip(starts, late):
        assert b > 4.0 * a, f"σ {a} → {b} over 400 pairs: the widening is not happening"


# --------------------------------------------------------------------------------- the criteria measure
def test_the_motion_criterion_would_notice_a_transform_and_a_sampler():
    """`the-cloud-spreads` has two edges, and both of them bite.

    The reference spreads the cloud; a motion model that is a rigid transform spreads it not at all and one with
    double the noise spreads it twice as far. Both are plausible code and both are wrong in the way a filter dies —
    the first is a dead-reckoning node with particles in it, the second is a σ that is a lie in the other
    direction, and a band with one edge would let one of them through.
    """
    good = module_of("solution/particles_solution.py", "ohm_e4_good")
    lim = ex.exercise(BOOK, "e4_particles")["limits"]["the-cloud-spreads"]

    def still(x, rot1, trans, rot2, dt, p, rng):
        out = np.array(x, float, copy=True)
        out[:, 0] = x[:, 0] + trans * np.cos(x[:, 2] + rot1)
        out[:, 1] = x[:, 1] + trans * np.sin(x[:, 2] + rot1)
        out[:, 2] = ((x[:, 2] + rot1 + rot2 + math.pi) % (2 * math.pi)) - math.pi
        return out

    doubled = {"alpha1": 0.10, "alpha2": 0.10, "alpha3": 0.10, "noise_rate_xy": 0.09, "noise_rate_theta": 0.18}
    for name, mod, limits in (("a rigid transform", SimpleNamespace(move_particles=still), lim),
                              ("double the noise", good, dict(lim, noise=doubled))):
        ok, measured = ex.criterion_particles_motion_spread(mod, limits, {"ex": {}, "notes": []})
        assert not ok, f"{name} passed the spread criterion: {measured}"
    ok_ref, measured_ref = ex.criterion_particles_motion_spread(good, lim, {"ex": {}, "notes": []})
    assert ok_ref, f"the reference itself left the band: {measured_ref}"
def test_the_rate_criterion_would_notice_a_per_step_sigma():
    """`noise-floors-are-rates` separates 0.045 m at every rate from a factor 2.8 across the same rates."""
    good = module_of("solution/particles_solution.py", "ohm_e4_good_rate")
    lim = ex.exercise(BOOK, "e4_particles")["limits"]["noise-floors-are-rates"]

    def per_step(x, rot1, trans, rot2, dt, p, rng):
        x = np.asarray(x, float)
        n = len(x)
        r1 = rot1 + rng.normal(0, 1, n) * (p.alpha1 * abs(rot1) + p.alpha2 * abs(trans))
        r2 = rot2 + rng.normal(0, 1, n) * (p.alpha1 * abs(rot2) + p.alpha2 * abs(trans))
        d = trans + rng.normal(0, 1, n) * (p.alpha3 * abs(trans) + p.noise_rate_xy)     # no √dt: the bug
        out = np.empty_like(x)
        out[:, 0] = x[:, 0] + d * np.cos(x[:, 2] + r1)
        out[:, 1] = x[:, 1] + d * np.sin(x[:, 2] + r1)
        out[:, 2] = ((x[:, 2] + r1 + r2 + math.pi) % (2 * math.pi)) - math.pi
        return out

    ctx, notes = {"ex": {}, "notes": []}, []
    ok, measured = ex.criterion_particles_motion_rate(type("m", (), {"move_particles": per_step}), lim, ctx)
    assert not ok, f"a per-step σ passed the rate criterion: {measured}"
    ok_ref, measured_ref = ex.criterion_particles_motion_rate(good, lim, {"ex": {}, "notes": []})
    assert ok_ref, measured_ref


def test_the_nearest_neighbour_reference_is_independent_of_what_it_grades():
    """The two-loop reference against the library's own matcher, on clouds of every shape."""
    rng = np.random.default_rng(3)
    for n, m in ((1, 1), (5, 1), (1, 5), (60, 90), (200, 3)):
        src, dst = rng.uniform(-3, 3, size=(n, 2)), rng.uniform(-3, 3, size=(m, 2))
        ref = ex._reference_nn(src, dst)
        lib = icp.nearest_neighbour(src, dst)          # the library's own matcher, brute by default
        assert np.allclose(ex._distance_of(src, dst, ref), ex._distance_of(src, dst, lib), atol=1e-9)
        assert ex._distance_of(src, dst, ref).max() <= ex._distance_of(
            src, dst, rng.integers(0, m, size=n) if m else np.zeros(n, int)).max() + 1e-9
