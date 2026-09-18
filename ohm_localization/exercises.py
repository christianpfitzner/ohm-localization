"""The four offline criteria of the lab: what `tools/lab_check.py` measures, and how it measures it.

    python3 tools/lab_check.py                      # every exercise, every criterion, the student's files
    python3 tools/lab_check.py e1_nn --module solution/nn_solution.py     # one exercise, another file
    python3 tools/lab_check.py --verbose            # with the measured numbers of every criterion

**Why the checks live in the library and not in the tool.** The grader of a 180-minute session has to be the
thing the documentation quotes its numbers from, and a test suite that asserts a threshold is reachable is
the same assertion in a different font. Both of them import this module, so a threshold can only ever be
measured once — `test/test_exercises.py` runs every criterion against `solution/` (must pass) and against
`student/` (must fail on its own TODO and on nothing else), which is the pair of facts that makes the mark
scheme real. `docs/verification.md` quotes the same runs.

**What an offline exercise is graded on and why nothing is asked of a robot.** E1 and E2 (nearest neighbour,
one scan pair) and E4 (generating the particles) need numpy and nothing else — no simulator, no ROS, no
robot, no licence, no hall. That is deliberate: the algorithmic content of both topics is checkable in
milliseconds on seeded synthetic data, so a group can run the same command that produces their mark as often
as they like during the session, and the hour that a hardware lab usually loses to "is it my robot or my
code" is spent on the criterion instead. E3 and E5 — ICP as odometry on a driving robot, and the localiser —
are graded by the simulator's own grader through `./tools/run_lab.sh grade`, because there the question is
what a whole drive does to a method and no synthetic stand-in answers that.

**The conventions, stated once here because they are the two ways to be silently wrong.** A registration
answer is the transform that carries points *from the frame of the model scan into the frame of the scene
scan* — `icp.relative(model_pose, scene_pose)` is the truth of it, in that order (`icp.relative()`'s
docstring is the one-line proof), and the reversed order scores a large error rather than an error message.
A particle is a pose `(x, y, θ)` in the **hall** frame with θ in (−π, π], and an estimate is the weighted
mean of the cloud with the heading averaged as a direction and not as a number.
"""
from __future__ import annotations

import importlib.util
import json
import math
import os
import sys
import time

import numpy as np

from ohm_localization import icp, synth
from ohm_localization.gridmap import GridMap, corridor_text, load_hall, parse_grid
from ohm_localization.mcl import MclParams

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXERCISE_FILE = os.path.join(ROOT, "config", "exercises_localization.json")
SEED = 20260917                      # one seed for every criterion, so a mark is repeatable


# -------------------------------------------------------------------------------- module under test
def load_module(path: str, name: str = "under_test"):
    """Import a student file by path, the way the laboratory's own runner imports controllers.

    Every student file in this repository is a script *and* a module — `python3 student/nn_template.py` has
    to work for the demo, and `import` has to work for the check — so the import must not execute anything
    beyond definitions, which is what `if __name__ == "__main__"` is for.
    """
    if not os.path.isfile(path):
        raise FileNotFoundError(f"no such module: {path}")
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod                       # a dataclass in the file wants its module registered
    spec.loader.exec_module(mod)
    return mod


def exercise_file(path: str | None = None) -> dict:
    """`config/exercises_localization.json` — the one source of points, thresholds and sheet text."""
    with open(path or EXERCISE_FILE, encoding="utf-8") as handle:
        return json.load(handle)


def exercise(exercises: dict, ex_id: str) -> dict:
    for e in exercises["exercises"]:
        if e["id"] == ex_id:
            return e
    known = ", ".join(e["id"] for e in exercises["exercises"])
    raise KeyError(f"no exercise {ex_id!r} — the file has {known}")


# ------------------------------------------------------------------------------- the scan-pair fixture
def scan_pairs(hall_name: str = "production", n: int = 12, step: float = 1.2,
               turn: float = 0.35, sigma: float = 0.02, seed: int = SEED,
               min_points: int = 200, identity_fit_min: float = 0.4,
               truth_fit_max: float = 0.03) -> list:
    """`n` model/scene pairs whose truth is known, each one a pair worth registering.

    The rejection is the content of this function. A pair taken in a corner registers from the identity —
    both scans see the same two walls and a translation is nearly free there — and a matcher scored on such
    a pair looks excellent while the fixture measures nothing. So a candidate is kept only if the identity
    explains it *badly* (mean correspondence distance over `identity_fit_min`) while the truth explains it
    *well* (under `truth_fit_max`), which is `test/test_icp.py`'s fixture rule with the same numbers for the
    same reason. `truth_fit_max` is 30 mm and not the 80 mm of the test, because a pair whose truth still
    leaves 8 cm of mean correspondence distance is a pair on which a wrong pose fits just as well: measuring
    a matcher on those is measuring the hall. The motion between the two poses is a fixed body-frame step
    plus a turn, so the answer is never trivially the identity and never so large that the pair stops being
    one scan period of a real drive.
    """
    hall = load_hall(hall_name)
    grid = GridMap(hall)
    rng = np.random.default_rng(seed)
    out, tries = [], 0
    while len(out) < n and tries < 4000:
        tries += 1
        a = synth.random_pose(grid, rng, margin=0.9, clearance=0.45)
        if a is None:
            continue
        b = (a[0] + step * math.cos(a[2] + turn / 2), a[1] + step * math.sin(a[2] + turn / 2),
             a[2] + turn)
        if bool(grid.occupied_at(b[0], b[1])) or grid.distance_at(b[0], b[1]) < 0.45:
            continue
        sa = synth.scan_dict(a, hall, t=0.0, sigma=sigma, rng=rng)
        sb = synth.scan_dict(b, hall, t=0.05, sigma=sigma, rng=rng)
        pa, pb = icp.points_from_scan(sa)[0], icp.points_from_scan(sb)[0]
        if len(pa) < min_points or len(pb) < min_points:
            continue
        T = icp.relative(a, b)
        if icp.mean_match(pa, pb, np.eye(3)) < identity_fit_min or icp.mean_match(pa, pb, T) > truth_fit_max:
            continue
        out.append({"model": sa, "scene": sb, "truth": T, "poses": (a, b), "points": (len(pa), len(pb))})
    if len(out) < n:
        raise RuntimeError(f"{hall_name}: found {len(out)} of {n} registerable pairs in {tries} tries")
    return out


def corridor_pair(width: float = 1.0, along: float = 1.5, seed: int = SEED) -> dict:
    """One pair taken in a bare corridor, sliding along it: the degeneracy E2 has to report.

    `gridmap.corridor_text()` exists because the simulator's halls are all too interesting to demonstrate
    what ICP is weak at — 31 × 6 m of long straight wall and nothing else, both ends beyond the 8 m range, the
    pair centred so neither of them is in range either. Every line the matcher fits then lies along a wall, so
    every normal points across it and the direction along the corridor is not measured at all.

    What the fit says about that, measured, is the point of the fixture: **σ along the corridor 21 mm**, against
    1.2 mm across it, and a condition number of 2644 where the hall pairs sit at 9. The σ does not shout — the
    beam noise tilts the locally fitted wall segments a little, and a near-null direction collects a σ from that
    noise instead of from the geometry (`test/test_icp.py` has the same case at 1.2 mm). `cond` is the number that
    shouts, and `degeneracy-is-reported` is the criterion that asks for it by name rather than for a σ that the
    fit was never able to produce.
    """
    hall = parse_grid(corridor_text(cols=60, rows=10), name="corridor")
    cx, cy = hall.size[0] / 2.0, hall.size[1] / 2.0
    a = (cx - along / 2.0, cy, 0.0)
    b = (cx + along / 2.0, cy, 0.0)
    rng = np.random.default_rng(seed)
    return {"model": synth.scan_dict(a, hall, sigma=0.02, rng=rng),
            "scene": synth.scan_dict(b, hall, sigma=0.02, rng=rng),
            "truth": icp.relative(a, b), "poses": (a, b), "along": (1.0, 0.0), "hall": hall}


# -------------------------------------------------------------------------------------- E1: nearest N
def _clouds(seed: int = SEED) -> list:
    """(src, dst) point sets of every awkward size: 1×1, 1×many, many×1, ties, and a real scan pair."""
    rng = np.random.default_rng(seed)
    shapes = [(1, 1), (1, 500), (500, 1), (7, 3), (360, 360), (200, 1000)]
    out = [(np.round(rng.uniform(-4, 4, size=(n, 2)), 3), np.round(rng.uniform(-4, 4, size=(m, 2)), 3))
           for n, m in shapes]
    gridpts = np.column_stack([np.repeat(np.arange(4.0), 4), np.tile(np.arange(4.0), 4)])
    out.append((gridpts, gridpts))                       # exact ties everywhere: the answer must be one of them
    hall = load_hall("rooms")
    pa = icp.points_from_scan(synth.scan_dict((4.0, 3.0, 0.2), hall))[0]
    pb = icp.points_from_scan(synth.scan_dict((4.35, 3.1, 0.28), hall))[0]
    out.append((pa, pb))
    return out


def _reference_nn(src, dst):
    """The answer, from a two-loop loop that shares no line of code with any vectorised candidate.

    Deliberately the slowest possible implementation: at these fixture sizes it costs a few tens of
    milliseconds once per check run, and what it buys is that a wrong `argmin` axis, a transposed cloud or a
    distance returned instead of an index cannot agree between the candidate and its reference. If `scipy`
    happens to be installed its tree is used as well, as a second opinion — a courtesy, not a requirement,
    because the laboratory's machines are not required to have it.
    """
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    out = np.empty(len(src), dtype=np.int64)
    for i in range(len(src)):
        best, best_d = 0, float("inf")
        for j in range(len(dst)):
            d = (dst[j, 0] - src[i, 0]) ** 2 + (dst[j, 1] - src[i, 1]) ** 2
            if d < best_d:
                best, best_d = j, d
        out[i] = best
    return out


def _distance_of(src, dst, idx):
    return np.linalg.norm(np.asarray(src, float) - np.asarray(dst, float)[np.asarray(idx, int)], axis=1)


def criterion_nn_correct(mod, lim, ctx):
    """Every point of `src` gets its true nearest neighbour in `dst`, on every shape, ties included."""
    bad, detail = [], []
    for i, (src, dst) in enumerate(_clouds()):
        try:
            got = np.asarray(mod.nearest_neighbour(src.copy(), dst.copy()), dtype=int)
        except NotImplementedError as exc:
            return False, f"{exc}"
        ref = _reference_nn(src, dst)
        ok = (got.shape == (len(src),) and got.min(initial=0) >= 0 and got.max(initial=0) < len(dst)
              and np.allclose(_distance_of(src, dst, got), _distance_of(src, dst, ref), atol=1e-9))
        if not ok:
            bad.append(i)
        detail.append(f"{len(src)}×{len(dst)}")
    return not bad, (f"{len(_clouds()) - len(bad)} of {len(_clouds())} fixture shapes exact "
                     f"({'ok' if not bad else 'wrong on ' + ', '.join(detail[i] for i in bad)})")


def criterion_nn_self(mod, lim, ctx):
    """`nearest_neighbour(a, a)` is the identity: the answer of a cloud matched against itself."""
    src = np.round(np.random.default_rng(SEED).uniform(-2, 2, size=(400, 2)), 4)
    got = np.asarray(mod.nearest_neighbour(src, src), dtype=int)
    return bool(np.array_equal(got, np.arange(len(src)))), f"{len(src)} points, {(got != np.arange(len(src))).sum()} mismatches"


def criterion_nn_empty(mod, lim, ctx):
    """An empty target raises instead of answering: a scan that echoed nothing has no correspondences."""
    src = np.array([[0.0, 0.0], [1.0, 1.0]])
    try:
        mod.nearest_neighbour(src, np.zeros((0, 2)))
    except ValueError:
        return True, "ValueError on an empty target"
    except Exception as exc:                                  # a wrong exception type is still a wrong answer
        return False, f"{type(exc).__name__} where ValueError was asked for"
    return False, "answered without raising — an empty cloud is a registration with a zero covariance"


def criterion_nn_speed(mod, lim, ctx):
    """The measured milliseconds of their function on the cloud of one real scan, against the two loops."""
    given = load_module(os.path.join(ROOT, "student", "nn_template.py"), name="ohm_e1_given")
    pa = icp.points_from_scan(synth.scan_dict((4.0, 3.0, 0.2), load_hall("rooms")))[0]
    pb = icp.points_from_scan(synth.scan_dict((4.35, 3.1, 0.28), load_hall("rooms")))[0]
    ms = _best_of(lambda: mod.nearest_neighbour(pa, pb), int(lim.get("repeats", 5)))
    loops = _best_of(lambda: given.nearest_neighbour_loop(pa, pb), 2)
    ctx["notes"].append(f"{len(pa)}×{len(pb)} points: theirs {ms:.2f} ms, two Python loops {loops:.0f} ms, "
                        f"the library's chunked broadcast {_best_of(lambda: icp.nearest_neighbour(pa, pb), 5):.2f} ms")
    return ms <= float(lim["ms_max"]), f"{ms:.2f} ms ≤ {lim['ms_max']:g} ms"


def _best_of(fn, repeats: int = 5) -> float:
    best = float("inf")
    for _ in range(max(int(repeats), 1)):
        t0 = time.perf_counter()
        fn()
        best = min(best, (time.perf_counter() - t0) * 1000.0)
    return float(best)


# ------------------------------------------------------------------------------ E2: one model, one scene
def _answer_of(mod, model, scene, T_guess=None):
    """Call the student's `register()` and normalise its answer to (T, sx, sy, sth).

    A dict with x/y/theta and sx/sy/sth is what the node has to send on `kf/pose` later, which is why that
    is the shape asked for; a 3×3 array is also accepted, because the first thing a group writes is usually
    the transform and the σ comes later.
    """
    out = mod.register(model, scene, T_guess=T_guess) if T_guess is not None else mod.register(model, scene)
    if isinstance(out, np.ndarray):
        out = {"T": out}
    if "T" in out and isinstance(out["T"], np.ndarray):
        T = np.asarray(out["T"], dtype=float)
    else:
        T = icp.se2(float(out["x"]), float(out["y"]), float(out["theta"]))
    s = out.get("sigmas") or (out.get("sx"), out.get("sy"), out.get("sth"))
    return T, tuple(float(v) for v in s), out


def _pairs(ctx):
    if "pairs" not in ctx:
        ctx["pairs"] = scan_pairs(**ctx["ex"].get("fixture", {}))
    return ctx["pairs"]


def criterion_icp_translation(mod, lim, ctx):
    """How far the registered pose is from the truth, over every pair: the median and the worst case."""
    err = [icp.error_between(_answer_of(mod, p["model"], p["scene"])[0], p["truth"])[0]
           for p in _pairs(ctx)]
    med, worst = float(np.median(err)), float(np.max(err))
    return med <= float(lim["dt_median_max"]) and worst <= float(lim["dt_max_max"]), \
        f"median {med * 1000:.1f} mm ≤ {float(lim['dt_median_max']) * 1000:g} mm, worst {worst * 1000:.1f} mm " \
        f"≤ {float(lim['dt_max_max']) * 1000:g} mm"


def criterion_icp_rotation(mod, lim, ctx):
    """The heading of the answer, in degrees: a scan matcher that is 3° off is a robot that is 5 cm off after 2 m."""
    err = [icp.error_between(_answer_of(mod, p["model"], p["scene"])[0], p["truth"])[1]
           for p in _pairs(ctx)]
    med, worst = float(np.median(err)), float(np.max(err))
    return med <= float(lim["dth_median_max"]) and worst <= float(lim["dth_max_max"]), \
        f"median {med:.2f}° ≤ {lim['dth_median_max']:g}°, worst {worst:.2f}° ≤ {lim['dth_max_max']:g}°"


def criterion_icp_fits_as_well(mod, lim, ctx):
    """Their answer must fit the pair at least as well as the truth does — the fitness is not their opinion.

    This is the criterion that separates "wrong but consistent" from "wrong and a worse fit": in a hall with
    repeating structure a wrong pose can fit *better* than the truth (`docs/icp.md` §5 measures a slide of
    exactly one post period fitting better), and there fitness cannot save you. In a hall with one-of-everything
    geometry like `production`, it can, and a matcher that lost the truth would show up here first.
    """
    worse = []
    for i, p in enumerate(_pairs(ctx)):
        T = _answer_of(mod, p["model"], p["scene"])[0]
        src, dst = (icp.points_from_scan(p["model"])[0], icp.points_from_scan(p["scene"])[0])
        mine, truth = icp.mean_match(src, dst, T), icp.mean_match(src, dst, p["truth"])
        if mine > truth * float(lim["fitness_ratio"]) + float(lim["fitness_slack"]):
            worse.append(i)
    n = len(_pairs(ctx))
    return not worse, (f"every pair fits ≤ {lim['fitness_ratio']:g}× the truth's fit" if not worse else
                       f"{len(worse)} of {n} pairs fit worse than the truth does (pairs {worse[:4]})")


def criterion_icp_sigma_degenerate(mod, lim, ctx):
    """The degeneracy must be in the numbers the matcher reports — the meter, or an honest σ.

    A matcher that is accurate in a hall full of corners is not yet a matcher that knows when it is not
    measuring anything. In a bare corridor slid along its own axis the data constrain the pose across the
    walls and not along them, and what a fit hands back for the free direction is **21 mm** on this fixture
    (1.2 mm in `test/test_icp.py`'s version of it, which is titled "the covariance only just notices"): the
    beam noise tilts the locally fitted wall segments a little, and a near-null direction gets a σ from that
    noise rather than from geometry. So `cond` — the condition number of `JᵀJ`, which is 2.6·10³ here against
    9.0 in the hall — is the number that shouts, and the criterion accepts either that shout or a σ that has
    been inflated by somebody who read it.

    Note what a prior cannot do here, because the question comes up in every viva: adding information can only
    *shrink* a covariance, so "put a prior on it" is not how a 21 mm σ becomes an honest one. Either the meter
    is reported, or the σ is widened on the strength of it, or the pair is refused.
    """
    pair = corridor_pair()
    _T, sig, raw = _answer_of(mod, pair["model"], pair["scene"])
    along, across = abs(float(sig[0])), abs(float(sig[1]))
    cond = raw.get("cond", raw.get("condition", float("nan")))
    shout = float(cond) >= float(lim["cond_min"])
    inflated = along >= float(lim["sigma_along_min"]) and along >= float(lim["ratio_min"]) * max(across, 1e-12)
    return bool(shout or inflated), (f"corridor: cond {float(cond):.1f} (asked for ≥ {lim['cond_min']:g}), "
                                     f"σ along {along * 1000:.0f} mm / across {across * 1000:.1f} mm "
                                     f"(or ≥ {float(lim['sigma_along_min']) * 1000:g} mm and ≥ "
                                     f"{lim['ratio_min']:g}× across)")


def criterion_icp_sigma_not_invented(mod, lim, ctx):
    """The σ changes from pair to pair, because it came off each fit and not out of a constant.

    The cheapest way to pass a σ-shaped criterion is to return the same three numbers every time, and the
    symptom is invisible until a robot drives into a corridor. The spread of the reported σ across the twelve
    pairs is the tell: a fit that is reading its own residuals produces a σ that ranges by a factor of five
    between a corner and an open straight (measured: 1.6 mm to 9.2 mm), and a constant produces none of it.
    """
    sig = np.array([_answer_of(mod, p["model"], p["scene"])[1] for p in _pairs(ctx)], dtype=float)
    if not np.all(np.isfinite(sig)):
        return False, "a σ that is not a number is not a σ"
    cv = [float(np.std(sig[:, i]) / max(np.mean(sig[:, i]), 1e-12)) for i in (0, 1)]
    ok = min(cv) >= float(lim["cv_min"])
    return ok, (f"reported σ across the twelve pairs: x {sig[:, 0].min() * 1000:.1f} … "
                f"{sig[:, 0].max() * 1000:.1f} mm (spread/mean {cv[0]:.2f}), y "
                f"{sig[:, 1].min() * 1000:.1f} … {sig[:, 1].max() * 1000:.1f} mm ({cv[1]:.2f}); asked for ≥ "
                f"{lim['cv_min']:g} on both")


def criterion_icp_from_a_guess(mod, lim, ctx):
    """The same pairs from a wrong starting guess: ICP has a basin of attraction and the guess is part of it.

    `T_guess` is what the wheel odometry will hand the matcher on the robot, and on the robot it is wrong by
    centimetres and degrees. The question is asked twice — once from a guess 100 mm and 2° off, once from the
    identity, which is what a node has when the wheels have not been read yet — because the two ways of failing
    it are different: an implementation that ignores `T_guess` fails the first, and one that merely polishes it
    into something near the guess fails the second.

    The limit sits deliberately below the catastrophe a *large* wrong guess causes here, and that gap is the
    protocol's subject: on these twelve pairs a 250 mm guess puts 3 of them between 1.9 m and 5.3 m away, no
    correspondence gate between 0.3 m and 1.5 m prevents it, and the distances are multiples of the post spacing
    in `production` — the alias basins of `docs/icp.md` §5, measured on a hall that has them. Beating that needs
    a coarse-to-fine search over poses, which is what a particle filter is; E5 is where that argument pays off.
    """
    dx, dy, dth = [float(v) for v in lim["guess"]]
    within, need = float(lim["within_max"]), float(lim["fraction_min"])
    pairs = _pairs(ctx)
    out = []
    for name, guess in ((f"a {dx * 1000:.0f} mm/{math.degrees(dth):.0f}° guess", icp.se2(dx, dy, dth)),
                        ("the identity", np.eye(3))):
        ok = sum(1 for p in pairs
                 if icp.error_between(_answer_of(mod, p["model"], p["scene"], T_guess=guess)[0],
                                      p["truth"])[0] <= within)
        out.append(f"from {name}: {ok} of {len(pairs)} within {within * 1000:g} mm")
        if ok < need * len(pairs):
            return False, "; ".join(out) + f" (asked for ≥ {need * len(pairs):.0f})"
    return True, "; ".join(out) + f" (asked for ≥ {need * len(pairs):.0f} both times)"


# --------------------------------------------------------------------------- E4: generating the particles
#: The noise the criteria drive with is the filter's own parameter object, not a parallel invention:
#: `move_particles(x, rot1, trans, rot2, dt, p, rng)` has the arguments of `mcl.MonteCarloLocaliser.predict`,
#: so a function written for this exercise is the function the real filter would call.
NOISE = MclParams()
QUIET = MclParams(alpha1=0.0, alpha2=0.0, alpha3=0.0, noise_rate_xy=0.0, noise_rate_theta=0.0)
#: The two families of motion noise, separated. The alphas are proportional to the motion in the step, so
#: splitting a step changes their accumulation and that is the model speaking, not a bug. The floors are per
#: √second, so splitting a step must change nothing — which is what `noise-floors-are-rates` measures, and the
#: reason it has to be measured on the floors alone.
FLOORS = MclParams(alpha1=0.0, alpha2=0.0, alpha3=0.0)


def _grid(ctx, name: str = "production"):
    key = f"grid:{name}"
    if key not in ctx:
        ctx[key] = GridMap(load_hall(name))
    return ctx[key]


def _call_cloud(fn, *args, **kw):
    out = np.asarray(fn(*args, **kw), dtype=float)
    if out.ndim != 2 or out.shape[1] != 3:
        raise ValueError(f"{fn.__name__} must return (n, 3); got {out.shape}")
    return out


def criterion_particles_uniform(mod, lim, ctx):
    """Global localisation's prior: n random poses, all of them on the floor, spread over the whole floor.

    Both halves bite. Samples inside a wall are a filter that will never explain a single beam; samples all
    in one corner are a filter that has already decided the answer, and the difference is invisible in the
    mean. The coverage count is the second half, and the block ratio is the uniformity of it — the
    histogram of a generator that samples x and y from the free list but forgets to randomise the order
    passes the first and fails the second.
    """
    grid = _grid(ctx)
    n = int(lim["n"])
    x = _call_cloud(mod.sample_uniform, grid, n, np.random.default_rng(SEED))
    d = np.array([grid.distance_at(px, py) for px, py in x[:, :2]])
    free = float(d.min()) >= float(lim["min_clearance"])
    inside = bool(np.all(x[:, 0] >= 0) and np.all(x[:, 0] <= grid.hall.size[0])
                  and np.all(x[:, 1] >= 0) and np.all(x[:, 1] <= grid.hall.size[1]))
    wrapped = bool(np.all(np.abs(x[:, 2]) <= math.pi + 1e-9))
    hit = {tuple(map(int, c)) for c in zip(np.floor(x[:, 0] / 0.5).astype(int),
                                           np.floor(x[:, 1] / 0.5).astype(int))}
    free_cells = len({tuple(map(int, c)) for c in
                      zip(np.floor(grid.free_xy[:, 0] / 0.5).astype(int),
                          np.floor(grid.free_xy[:, 1] / 0.5).astype(int))})
    cover = len(hit) / max(free_cells, 1)
    blocks = np.zeros((4, 4))
    for px, py in x[:, :2]:
        blocks[min(int(px / (grid.hall.size[0] / 4)), 3), min(int(py / (grid.hall.size[1] / 4)), 3)] += 1
    ratio = float(blocks.max() / max(blocks.min(), 1.0))
    ok = free and inside and wrapped and cover >= float(lim["cover_min"]) and ratio <= float(lim["block_ratio_max"])
    return ok, (f"{n} samples, closest to a wall {float(d.min()) * 1000:.0f} mm, {cover * 100:.0f} % of the free "
                f"0.5 m cells hit (≥ {float(lim['cover_min']) * 100:.0f} %), busiest/quietest quarter "
                f"{ratio:.1f}× (≤ {lim['block_ratio_max']:g}×), θ wrapped {wrapped}")


def criterion_particles_gaussian(mod, lim, ctx):
    """The localised prior: the spread the filter is told about is the spread it gets, in heading too.

    A σ that is silently too small is the most expensive bug in this filter, because everything downstream
    — the weights, N_eff, the resampling — behaves as if the robot were known, and the estimate comes out
    confident and wrong. So the sample's own σ is measured against the σ that was asked for, on 4000 draws
    where the sampling error of the measurement itself is under 2 %. The heading is checked where it is
    checked nowhere else: a prior centred at θ = 3.0 rad with σ = 0.5 has **39 %** of its samples past π, which
    have to come back around at −π, and a generator that adds noise without wrapping leaves them outside the
    range every other part of the filter assumes. The wrap band is there so that a clipped or ignored heading
    cannot pass by being merely plausible.
    """
    center, sigma, n = np.asarray(lim["center"], float), np.asarray(lim["sigma"], float), int(lim["n"])
    rng = np.random.default_rng(SEED)
    x = _call_cloud(mod.sample_gaussian, tuple(center), tuple(sigma), n, rng)
    mean, std = x.mean(axis=0), x.std(axis=0)
    dth = (x[:, 2] - center[2] + math.pi) % (2.0 * math.pi) - math.pi      # distance along the circle
    lo, hi = [float(v) for v in lim["std_band"]]
    ok_mean = bool(np.all(np.abs(mean[:2] - center[:2]) <= 3.0 * sigma[:2] / math.sqrt(n)))
    ok_std = bool(np.all(std[:2] >= lo * sigma[:2]) and np.all(std[:2] <= hi * sigma[:2]))
    wrapped = bool(np.all(np.abs(x[:, 2]) <= math.pi + 1e-9))
    over = float(np.mean(dth > math.pi - center[2]))        # samples that had to come around the wrap
    wlo, whi = [float(v) for v in lim["wrap_band"]]
    ok_wrap = wrapped and wlo <= over <= whi
    return ok_mean and ok_std and ok_wrap, (
        f"mean off by ({mean[0] - center[0]:+.4f}, {mean[1] - center[1]:+.4f}) m, measured σ "
        f"({std[0]:.3f}, {std[1]:.3f}, {float(dth.std()):.3f}) against the σ asked for "
        f"({sigma[0]:.2f}, {sigma[1]:.2f}, {sigma[2]:.2f}), {over * 100:.1f} % past the ±π wrap "
        f"(band {wlo * 100:g} … {whi * 100:g} %)")


def _rigid(rot1: float, trans: float, rot2: float, x: np.ndarray) -> np.ndarray:
    """The noiseless motion model, computed here so the noisy one has something to be compared against.

    The same three-part step `mcl.motion_model()` and the simulator use: turn by `rot1`, drive `trans` in the
    heading that results, turn by `rot2`. Every particle moves by the *same* step, because the step is the
    odometry message and the message is one per period for the whole fleet of hypotheses.
    """
    out = np.array(x, dtype=float, copy=True)
    out[:, 0] = x[:, 0] + trans * np.cos(x[:, 2] + rot1)
    out[:, 1] = x[:, 1] + trans * np.sin(x[:, 2] + rot1)
    out[:, 2] = ((x[:, 2] + rot1 + rot2 + math.pi) % (2 * math.pi)) - math.pi
    return out


def criterion_particles_motion_noiseless(mod, lim, ctx):
    """With the noise switched off the motion model is a rigid transform — and only then.

    This is the criterion that finds the three usual mistakes at once: applying the odometry *pose* instead
    of the odometry *step*, rotating about the world origin instead of about the robot, and forgetting that
    the turn happens in two parts (`rot1`, then the straight line, then `rot2`).
    """
    x = np.tile(np.array([3.0, 4.0, 0.3]), (50, 1)) + np.random.default_rng(SEED).normal(0, 0.05, (50, 3))
    got = _call_cloud(mod.move_particles, x, 0.4, 1.1, -0.25, 0.05, QUIET, np.random.default_rng(SEED))
    want = _rigid(0.4, 1.1, -0.25, x)
    d = np.linalg.norm(got[:, :2] - want[:, :2], axis=1)
    dth = np.abs(((got[:, 2] - want[:, 2] + math.pi) % (2 * math.pi)) - math.pi)
    return bool(d.max() <= float(lim["noiseless_max"]) and dth.max() <= float(lim["noiseless_max"])), \
        f"worst point {d.max() * 1000:.3f} mm and {math.degrees(dth.max()):.3f}° from the rigid transform"

def criterion_particles_motion_spread(mod, lim, ctx):
    """With the noise switched on the cloud has to *spread*: the motion model is a sampler, not a transform.

    The band is the point and it is a band on purpose. Too little spread and the filter is a dead-reckoning
    node with extra steps — the cloud can only shrink at every resampling and the estimate walks into a
    confident wrong pose a few seconds later, which is exactly what `docs/mcl.md` §4 records from the
    development of this repository. Too much and the weights are the only thing holding the answer together,
    and the σ of the estimate is a lie in the other direction. The band's edges are the measured values of
    the reference implementation with the parameters given, and the run-to-run spread of 1200 particles.

    The mean is checked against the same motion applied without noise, which is where a motion model with a
    systematic error in it — the turn applied twice, the step taken before the first rotation — shows up as a
    bias rather than hiding inside the spread. It is a loose limit, 150 mm against a measured 71…93 mm, because
    the centre of a spreading cloud is itself a random walk: at 1200 particles its own standard error is 22 mm
    per axis, and a bound tighter than that grades the seed rather than the model. What it does catch is a
    systematic error, which over 20 steps of this drive is metres.
    """
    x0 = np.tile(np.array([5.0, 5.0, 0.0]), (1200, 1))
    steps, dt = int(lim["steps"]), 0.05
    noise = MclParams(**lim["noise"]) if lim.get("noise") else NOISE
    rng = np.random.default_rng(SEED)
    got, want = x0.copy(), x0.copy()
    for _ in range(steps):
        got = _call_cloud(mod.move_particles, got, 0.05, 0.3, 0.02, dt, noise, rng)
        want = _rigid(0.05, 0.3, 0.02, want)
    spread = float(np.hypot(*got[:, :2].std(axis=0)))
    dth = float(np.std(((got[:, 2] + math.pi) % (2 * math.pi)) - math.pi))
    lo, hi = [float(v) for v in lim["spread_band"]]
    drift = float(np.linalg.norm(got[:, :2].mean(axis=0) - want[:, :2].mean(axis=0)))
    ok = (lo <= spread <= hi and dth <= float(lim["dth_max"])
          and drift <= float(lim["mean_max"]))
    return ok, (f"position spread {spread:.3f} m after {steps} steps of {dt * 1000:.0f} ms (band {lo:.3f} … "
                f"{hi:.3f}), heading spread {dth:.4f} rad (≤ {lim['dth_max']:g}), mean drifted "
                f"{drift * 1000:.1f} mm (≤ {float(lim['mean_max']) * 1000:g} mm)")


def criterion_particles_motion_rate(mod, lim, ctx):
    """The noise **floors** are rates: the same second of motion spreads the cloud the same amount at 20 Hz, 10 Hz and 5 Hz.

    The one criterion this repository earned the hard way, and the reason the floor is a rate: the reference
    filter predicted once per scan (20 Hz) in one run and once per odometry message (50 Hz) in another, and the
    same drive came out at 0.20 m and 1.26 m of RMSE with identical parameters. A per-step σ means *per
    message*, so the second version widened its cloud by √2.5 per second for no reason but the topic rate, and
    in a hall where the LIDAR says little nothing was left to pull it back in (`docs/mcl.py:predict()`'s
    docstring carries the same measurement into the code).

    The criterion runs with the alphas at zero, and that is not a simplification: the alpha terms are
    proportional to the motion in the step, so their accumulation over a fixed drive *does* depend on how the
    drive is chopped up — measured here, the full model spreads 0.26 m at 40 updates per second and 0.72 m at
    5, a factor 2.7 that belongs to the model and not to the implementation. The floor is different in kind. It
    is what a stationary robot's wheels still get wrong, it is stated per square root of second, and it must be
    blind to the message bus. Measured on this fixture with the alphas off: **0.044 m at 40 Hz, 0.045 m at
    20 Hz, 0.045 m at 10 Hz** for a model that divides by √dt, and **0.280 m, 0.202 m, 0.141 m** for one that
    does not — the same second of driving, a factor 2.8 apart, and no measurement of the robot involved.
    """
    def spread_of(steps, total=1.0, seed=SEED):
        got = np.tile(np.array([5.0, 5.0, 0.0]), (1200, 1))
        rng = np.random.default_rng(seed)
        dt = total / steps
        for _ in range(steps):
            got = _call_cloud(mod.move_particles, got, 1.0 / steps, 6.0 / steps, 0.0, dt, FLOORS, rng)
        return float(np.hypot(*got[:, :2].std(axis=0)))

    a, b, c = spread_of(int(lim["steps_a"])), spread_of(int(lim["steps_b"])), spread_of(int(lim["steps_c"]))
    lo, hi = [float(v) for v in lim["ratio_band"]]
    ratios = [a / max(b, 1e-9), a / max(c, 1e-9)]
    ok = all(lo <= r <= hi for r in ratios) and b > float(lim["min_spread"])
    return bool(ok), (f"one second of the same motion as {int(lim['steps_a'])} × {1000 / lim['steps_a']:.0f} ms "
                      f"spreads {a:.3f} m, as {int(lim['steps_b'])} × {1000 / lim['steps_b']:.0f} ms "
                      f"{b:.3f} m and as {int(lim['steps_c'])} × {1000 / lim['steps_c']:.0f} ms {c:.3f} m; "
                      f"ratios {ratios[0]:.2f} and {ratios[1]:.2f}, band {lo:g} … {hi:g}")


def criterion_particles_resample(mod, lim, ctx):
    """Resampling copies particles in the ratio of their weights — no more, and no less.

    Three properties, all of them invisible in the mean of the resulting cloud. The cloud here is three modes:
    one holding 80 % of the weight at x = 10, one holding 20 % at x = 0, and one holding nothing at all at
    x = −5. A resampler that sorts and keeps the best n puts 1200 copies at x = 10 and reports a filter that
    has thrown away the second mode (and the robot is standing in it); a resampler that draws with replacement
    but forgets to renormalise brings the third mode back to life; and a resampler that loses a handful of
    particles each time it runs is a filter that dies of thirst at exactly the moment its weights become
    interesting. The mean of the output is the same statement in one number: 8.0 m, which is what a cloud that
    kept both modes in their proportions says, and 10.0 m for one that did not.
    """
    n, per = int(lim["n"]), int(lim["per_mode"])
    x = np.vstack([np.zeros((per, 3)), np.tile([10.0, 0.0, 0.0], (per, 1)),
                   np.tile([-5.0, 0.0, 0.0], (per, 1))])
    w = np.concatenate([np.full(per, float(lim["weight_a"]) / per),
                        np.full(per, float(lim["weight_b"]) / per), np.zeros(per)])
    out = _call_cloud(mod.resample, x, w, n, np.random.default_rng(SEED))
    at = lambda v: int(np.sum(np.all(np.abs(out - np.array(v)) < 1e-9, axis=1)))   # noqa: E731 - three counts
    a, b, dead = at((0.0, 0.0, 0.0)), at((10.0, 0.0, 0.0)), at((-5.0, 0.0, 0.0))
    want_a, want_b = float(lim["weight_a"]) * n, float(lim["weight_b"]) * n
    lo, hi = [float(v) for v in lim["copy_band"]]
    mean = float(out[:, 0].mean())
    want_mean = float(lim["weight_b"]) * 10.0
    ok = (len(out) == n and want_a * lo <= a <= want_a * hi and want_b * lo <= b <= want_b * hi
          and dead == 0 and abs(mean - want_mean) <= float(lim["mean_max"]))
    return ok, (f"{b} copies of the 80 % mode (expected {want_b:.0f}), {a} of the 20 % mode "
                f"(expected {want_a:.0f}), {dead} of the zero-weight mode, mean {mean:.3f} m (expected "
                f"{want_mean:.3f} ± {float(lim['mean_max']) * 1000:g} mm), {len(out)} particles out")


def criterion_particles_estimate(mod, lim, ctx):
    """The estimate is the weighted mean of the cloud, with the heading averaged as a direction.

    Two clouds, one of them straddling ±π: the arithmetic mean of headings that wrap answers 0 rad for a
    cloud that means 180°, and the failure is invisible in every other number the filter prints — the σ is
    fine, the position is fine, and the robot drives backwards. The second half is the σ: it has to come out
    of the spread of the cloud, because that number is what the grader scores as NEES and what the RViz
    ellipse is drawn from.
    """
    x = np.column_stack([np.full(600, 2.0), np.full(600, 1.0),
                         np.concatenate([np.linspace(2.9, math.pi, 300), np.linspace(-math.pi, -2.9, 300)])])
    w = np.full(len(x), 1.0 / len(x))
    e = mod.weighted_estimate(x, w)
    heading = abs(float(e["theta"]))
    straddle = heading >= float(lim["wrap_heading_min"])
    x2 = np.column_stack([np.random.default_rng(SEED).normal(4.0, 0.2, 4000),
                          np.random.default_rng(SEED + 1).normal(2.0, 0.1, 4000),
                          np.zeros(4000)])
    w2 = np.full(len(x2), 1.0 / len(x2))
    e2 = mod.weighted_estimate(x2, w2)
    pos_ok = abs(float(e2["x"]) - 4.0) <= 0.01 and abs(float(e2["y"]) - 2.0) <= 0.01
    sig_ok = (abs(float(e2["sx"]) - 0.2) <= float(lim["sigma_tol"])
              and abs(float(e2["sy"]) - 0.1) <= float(lim["sigma_tol"]))
    return straddle and pos_ok and sig_ok, (
        f"cloud across ±π estimated at {heading:.2f} rad (≥ {lim['wrap_heading_min']:g}), "
        f"mean ({float(e2['x']):.3f}, {float(e2['y']):.3f}), σ ({float(e2['sx']):.3f}, {float(e2['sy']):.3f}) "
        f"against the cloud's own (0.200, 0.100)")


# ----------------------------------------------------------------------------------- the criteria table
CRITERIA = {
    "e1_nn": [("exact-on-every-shape", 10, criterion_nn_correct),
              ("self-match-is-the-identity", 3, criterion_nn_self),
              ("empty-target-raises", 3, criterion_nn_empty),
              ("measured-speed", 4, criterion_nn_speed)],
    "e2_icp_pair": [("fits-as-well-as-the-truth", 4, criterion_icp_fits_as_well),
                    ("translation-error", 9, criterion_icp_translation),
                    ("heading-error", 5, criterion_icp_rotation),
                    ("sigma-not-invented", 3, criterion_icp_sigma_not_invented),
                    ("degeneracy-is-reported", 5, criterion_icp_sigma_degenerate),
                    ("survives-a-wrong-guess", 4, criterion_icp_from_a_guess)],
    "e4_particles": [("uniform-on-the-floor", 8, criterion_particles_uniform),
                     ("gaussian-prior-with-the-asked-sigma", 8, criterion_particles_gaussian),
                     ("noiseless-motion-is-rigid", 4, criterion_particles_motion_noiseless),
                     ("the-cloud-spreads", 8, criterion_particles_motion_spread),
                     ("noise-floors-are-rates", 6, criterion_particles_motion_rate),
                     ("resample-follows-weights", 8, criterion_particles_resample),
                     ("estimate-averages-headings", 8, criterion_particles_estimate)],
}


def run(ex_id: str, module_path: str, limits: dict | None = None, verbose: bool = False,
        exercises: dict | None = None) -> dict:
    """Every criterion of one exercise against one module: the rows a mark is made of.

    `limits` defaults to the exercise's own `limits` block in `config/exercises_localization.json`, and the
    points of each criterion come from that file too — the code knows which question to ask and the JSON
    knows what the answer is worth and how good it has to be. That split is what keeps the sheet, the mark
    scheme and the grader honest with each other: `test/test_exercises.py` asserts that the criteria in the
    file and the criteria in `CRITERIA` are the same set, in the same exercise.
    """
    exercises = exercises or exercise_file()
    ex = exercise(exercises, ex_id)
    limits = limits or ex.get("limits", {})
    want = {c["id"]: c for c in ex.get("criteria", [])}
    if set(want) != {name for name, _p, _f in CRITERIA[ex_id]}:
        raise KeyError(f"{ex_id}: the criteria in config/exercises_localization.json "
                       f"{sorted(want)} are not the criteria in ohm_localization/exercises.py "
                       f"{sorted(n for n, _p, _f in CRITERIA[ex_id])}")
    mod = load_module(module_path, name=f"ohm_under_test_{ex_id}")
    ctx = {"ex": ex, "notes": []}
    rows, earned, reasons = [], 0.0, []
    for name, _default_points, fn in CRITERIA[ex_id]:
        points = float(want[name]["points"])
        try:
            ok, measured = fn(mod, limits.get(name, {}), ctx)
        except NotImplementedError as exc:
            ok, measured = False, f"{exc}"
        except Exception as exc:                                # a criterion that cannot be measured is failed
            ok, measured = False, f"{type(exc).__name__}: {exc}"
        rows.append({"criterion": name, "points": points, "ok": bool(ok), "measured": str(measured)})
        if ok:
            earned += points
        else:
            reasons.append(f"{name}: {measured}")
    return {"id": ex_id, "title": ex["title"], "module": module_path, "rows": rows,
            "points": round(earned, 1), "max_points": sum(r["points"] for r in rows),
            "passed": not reasons, "reason": "; ".join(reasons) or "meets requirements",
            "notes": ctx["notes"], "verbose": verbose}
