#!/usr/bin/env python3
"""What scan matching is worth here, measured: accuracy, basin of attraction, and the two ways it fails.

    python3 tools/icp_eval.py                      # point-to-point vs point-to-line on real pairs
    python3 tools/icp_eval.py --sweep-noise        # and for four different beam noises
    python3 tools/icp_eval.py --basin              # how wrong the starting guess is allowed to be
    python3 tools/icp_eval.py --degenerate         # a bare corridor, and a corridor with repeating posts
    python3 tools/icp_eval.py --claims             # recompute every number docs/icp.md quotes
    python3 tools/icp_eval.py --recorded /tmp/prod.jsonl

ICP is the other half of this exercise.  It is what turns two LIDAR scans into a relative pose, it is
the heart of every laser odometry and every pose-graph localisation, and — unlike a particle filter —
it never tells you when it is wrong.  That is the subject rather than the accuracy table.  In a hall
like `production` both variants converge and both print a covariance; the point-to-line variant is
some six times more accurate in translation and a couple of hundred times more accurate in rotation.
In a bare corridor both report a σ of about a millimetre around a pose that is three metres from the
truth, and in a corridor whose posts repeat every four metres the aliased pose *fits better than the
truth does*, which leaves nothing inside the algorithm to argue with.

Scan pairs are cast from the same map the localiser uses (`synth.cast`, so the geometry is the
simulator's) and the truth is the transform between the two poses that produced them — never a number
this file computed twice.  With `--recorded` the pairs come off a real drive and the truth comes from
the recorded `/truth`, which turns the whole thing into a cross-check against the simulator instead of
against this repository.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from ohm_localization import icp as I                                          # noqa: E402
from ohm_localization import synth                                             # noqa: E402
from ohm_localization.gridmap import GridMap, corridor_text, load_hall, parse_grid   # noqa: E402

RANGE_MAX = 8.0
MIN_ECHO = 120        # beams.  A pair in a corner where nothing echoes measures the corner, not ICP.


# -------------------------------------------------------------------------------------- scan pairs
def synthetic_triples(hall, n=12, noise=0.02, seed=17, span=(0.4, 1.2),
                      turn=math.radians(22.0)):
    """`n` pairs of scans with a real motion between them, where the LIDAR has something to see.

    The rejection is the point: a pose against a wall, or one at the far end of an open hall, gives a
    pair whose error says something about the clamp or about the empty space and nothing about the
    matcher.
    """
    grid = GridMap(hall)
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < n:
        a = synth.random_pose(grid, rng, margin=0.8, clearance=0.6)
        if a is None:
            raise SystemExit(f"no pose with 0.6 m clearance in {grid} — pick a wider hall")
        b = (a[0] + rng.uniform(*span), a[1] + rng.uniform(*span), a[2] + rng.uniform(-turn, turn))
        if (grid.distance_at(b[0], b[1]) < 0.6 or grid.occupied_at(b[0], b[1])
                or _echoes(hall, a) < MIN_ECHO or _echoes(hall, b) < MIN_ECHO):
            continue
        sa = synth.scan_dict(a, hall, sigma=noise, rng=np.random.default_rng(3))
        sb = synth.scan_dict(b, hall, sigma=noise, rng=np.random.default_rng(4))
        out.append((sa, sb, I.relative(a, b), (a, b)))
    return out


def recorded_triples(scans, truth, n=12, step=40, seed=5):
    """Pairs taken from a drive: scan `i` against scan `i + step`, truth from the recorded poses."""
    rng = np.random.default_rng(seed)
    idx = list(range(0, len(scans) - step, 5))
    rng.shuffle(idx)
    out = []
    for i in idx:
        a, b = truth[i][:3], truth[i + step][:3]
        if math.hypot(a[0] - b[0], a[1] - b[1]) < 0.2:
            continue                                  # a pair 5 cm apart measures the noise, not ICP
        out.append((scans[i], scans[i + step], I.relative(a, b), (a, b)))
        if len(out) >= n:
            break
    return out


def _echoes(hall, pose, beams=360):
    r = synth.cast(pose, hall, beams=beams, range_max=RANGE_MAX)
    return int(np.count_nonzero(np.isfinite(r) & (r < RANGE_MAX - 0.06)))


# ----------------------------------------------------------------------------------------- runs
def evaluate(triples, mode, sigma_z=None, **kw):
    """One ICP variant over a list of pairs: medians a person will quote, and nothing smoothed away."""
    err, sig, cond, its, fits, conv, points = [], [], [], [], [], [], []
    for sa, sb, T_truth, _ in triples:
        res = I.register_scans(sa, sb, mode=mode, sigma_z=sigma_z if sigma_z is not None else 0.02,
                               **kw)
        d, deg = I.error_between(res.T, T_truth)
        err.append((d, deg))
        sig.append(res.sigmas)
        cond.append(res.cond)
        its.append(res.iterations)
        fits.append(res.fitness)
        conv.append(res.converged)
        points.append(res.correspondences)
    e = np.array(err)
    return dict(mode=mode, n=len(e), trans=float(np.median(e[:, 0])), trans_max=float(e[:, 0].max()),
                yaw_deg=float(np.median(e[:, 1])), fit=float(np.median(fits)),
                sx=float(np.median([s[0] for s in sig])), syaw_deg=float(np.median([s[2] for s in sig])),
                cond=float(np.median(cond)), its=float(np.median(its)),
                points=float(np.median(points)), converged=int(sum(conv)))


def report(r, ref=None):
    extra = ""
    if ref:
        extra = (f"\n         → {ref['trans'] / max(r['trans'], 1e-12):4.1f}× less translation, "
                 f"{ref['yaw_deg'] / max(r['yaw_deg'], 1e-12):5.0f}× less rotation than {ref['mode']}")
    print(f"  {r['mode']:6s} ×{r['points']:.0f} pts: median |Δt| {r['trans'] * 1000:6.1f} mm "
          f"(worst {r['trans_max'] * 1000:6.1f}), |Δθ| {r['yaw_deg']:7.4f}°, "
          f"fit {r['fit'] * 1000:5.1f} mm\n"
          f"         σ {r['sx'] * 1000:5.2f} mm / {r['syaw_deg']:6.3f}°, cond {r['cond']:6.1f}, "
          f"{r['its']:4.1f} iterations, converged {r['converged']}/{r['n']}{extra}")


def basin(hall, triples, errors=(0.0, 0.1, 0.25, 0.5, 1.0, 2.0, 3.0, 5.0)):
    """Median error against the error in the *starting guess*: the size of the basin, in metres.

    Reported as a table rather than as one number because it is not one number: point-to-point gives up
    somewhere around a metre in a structured hall and point-to-line survives further, and a task that
    says "ICP converges" without saying from where is describing a different algorithm.
    """
    print(f"basin of attraction: the guess is displaced along x by the column; "
          f"median |Δt| of the result, mm")
    print(f"  {'mode':6s} " + " ".join(f"{e * 1000:6.0f}" for e in errors))
    rows = {}
    for mode in ("point", "line"):
        row = []
        for e in errors:
            errs = []
            for sa, sb, T_truth, _ in triples:
                res = I.register_scans(sa, sb, T_guess=I.se2(-e, 0.0, 0.0), mode=mode, stride=4)
                errs.append(I.error_between(res.T, T_truth)[0])
            row.append(float(np.median(errs)))
        rows[mode] = row
        print(f"  {mode:6s} " + " ".join(f"{v * 1000:6.0f}" for v in row))
    for mode, row in rows.items():
        loose = next((e for e, v in zip(errors, row) if v > 0.1), None)
        held = [e for e, v in zip(errors, row) if v <= 0.1]
        print(f"  {mode}: stays right up to {max(held, default=0):.2f} m of wrong guess, "
              f"first wrong at {loose if loose is not None else 'beyond ' + str(errors[-1])} m")
    return {"errors": list(errors), **rows}


def degenerate(noise=0.02):
    """The two geometries the simulator's halls cannot provide, and what each does to the answer."""
    out = {}
    # `parse_grid` gives a Hall (the walls); the free-cell list and the clearance query live on the
    # GridMap that wraps it, so both are built here rather than assumed.
    specs = (("bare corridor", GridMap(parse_grid(corridor_text(), cell=0.5, name="bare corridor")), 3.0),
             ("posted corridor", GridMap(parse_grid(corridor_text(pillars=1), cell=0.5,
                                                        name="posted corridor")), 4.0))
    for name, grid, slide in specs:
        cells = grid.free_xy
        mid = (float(np.mean(cells[:, 0])), float(np.mean(cells[:, 1])), 0.0)
        a, b = mid, (mid[0] + slide, mid[1], mid[2])
        sa = synth.scan_dict(a, grid.hall, sigma=noise, rng=np.random.default_rng(3))
        sb = synth.scan_dict(b, grid.hall, sigma=noise, rng=np.random.default_rng(4))
        pa = I.points_from_scan(sa, stride=4)[0]        # (cloud, dropped): the cloud
        pb = I.points_from_scan(sb, stride=4)[0]
        T_truth = I.relative(a, b)
        print(f"\n{name} ({grid}): poses {slide:.1f} m apart along it, "
              f"clearance at the mid pose {grid.distance_at(mid[0], mid[1]):.2f} m, "
              f"{len(pb)} points per scan")
        for mode in ("point", "line"):
            res = I.register_scans(sa, sb, mode=mode, stride=4, max_iterations=30)
            d, deg = I.error_between(res.T, T_truth)
            sx, sy, sth = res.sigmas
            at_truth = I.mean_match(pb, pa, I.inverse(T_truth), max_corr=1.5)
            print(f"  {mode:6s} {'converged' if res.converged else 'HUNTED   '} in {res.iterations:2d} "
                  f"iterations: error {d:6.3f} m / {deg:5.2f}°, σ {min(sx, sy) * 1000:5.2f} mm, "
                  f"fit {res.fitness * 1000:5.2f} mm, cond {res.cond:8.1f}")
            print(f"         σ understates the error by {d / max(min(sx, sy), 1e-6):7.0f}×   "
                  f"fitness at the truth {at_truth * 1000:6.2f} mm vs at the estimate "
                  f"{res.fitness * 1000:6.2f} mm "
                  f"({'the truth fits WORSE' if at_truth > res.fitness else 'the truth fits better'})")
            out[(name, mode)] = dict(error=d, sigma=min(sx, sy), cond=res.cond, fit=res.fitness,
                                     fitness_at_truth=at_truth, iterations=res.iterations,
                                     converged=bool(res.converged))
    return out


# ---------------------------------------------------------------------------------------- driver
def claims(args):
    """Every number `docs/icp.md` quotes, recomputed here, with the claim it is quoted to support."""
    hall = load_hall(args.hall)
    triples = synthetic_triples(hall, 10, noise=0.02, seed=args.seed)
    p = evaluate(triples, "point", stride=4)
    l = evaluate(triples, "line", stride=4)
    deg = degenerate()
    bare_p, bare_l = deg[("bare corridor", "point")], deg[("bare corridor", "line")]
    posted = deg[("posted corridor", "line")]
    bare = bare_p
    checks = [
        ("point-to-line is ≥ 4× better in translation", p["trans"] / max(l["trans"], 1e-12),
         lambda v: v >= 4.0),
        # 2× in rotation, not the 263× the single pair in `tests/test_icp.py` shows.  The rotation
        # advantage of point-to-line is real but pair-dependent — it is largest where the motion has a
        # lever arm against a long wall — so a claim printed as a median has to be a median's number.
        ("point-to-line is ≥ 2× better in rotation (median over pairs)",
         p["yaw_deg"] / max(l["yaw_deg"], 1e-12), lambda v: v >= 2.0),
        ("point-to-line median error < 20 mm in a structured hall", l["trans"], lambda v: v < 0.020),
        ("cond < 30 on structured pairs", l["cond"], lambda v: v < 30.0),
        # The pair of lines that follows is the reason `cond` is worth printing at all.  In a bare
        # corridor the *point-to-line* condition number blows up by two orders of magnitude — it is the
        # variant that models the walls, so it is the one that notices there are only two of them and
        # that they say nothing about the direction along themselves.  Point-to-point reports a healthy
        # single-digit condition number while being 3 m wrong: it never models the wall, so it has
        # nothing to be suspicious about.  A warning you do not print is a warning you do not have.
        ("bare corridor: cond of point-to-line ≥ 500 — it knows something is wrong",
         bare_l["cond"], lambda v: v >= 500.0),
        ("bare corridor: cond of point-to-point < 30 — it does not know",
         bare_p["cond"], lambda v: v < 30.0),
        ("a bare corridor: σ understates a 3 m error by ≥ 100×",
         bare["error"] / max(bare["sigma"], 1e-12), lambda v: v >= 100.0),
        ("a posted corridor: the match lands ≥ 3 m away, one post-period wrong",
         posted["error"], lambda v: v >= 3.0),
        ("a posted corridor: the aliased pose fits as well as the truth does",
         posted["fitness_at_truth"] / max(posted["fit"], 1e-12), lambda v: v >= 0.9),
    ]
    ok = True
    for label, value, good in checks:
        met = bool(good(value))
        ok = ok and met
        print(f"  {'meets' if met else 'NOT MET':8s} {label}: {value:.3f}")
    print("\n" + ("the documented numbers hold" if ok else
                  "the documentation quotes numbers this run did not reproduce — fix the document or "
                  "the code, in that order, and never the threshold"))
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--hall", default="production")
    ap.add_argument("--pairs", type=int, default=12)
    ap.add_argument("--noise", type=float, default=0.02, help="beam noise σ in m (synthetic pairs)")
    ap.add_argument("--stride", type=int, default=4, help="every n-th beam becomes a point")
    ap.add_argument("--seed", type=int, default=17)
    ap.add_argument("--sweep-noise", action="store_true", help="5/20/50/150 mm, not one value")
    ap.add_argument("--basin", action="store_true")
    ap.add_argument("--degenerate", action="store_true")
    ap.add_argument("--claims", action="store_true")
    ap.add_argument("--recorded", help="a tools/record_scans.py JSONL instead of synthetic pairs")
    ap.add_argument("--json", metavar="FILE", help="also write the numbers as JSON")
    args = ap.parse_args(argv)

    if args.claims:
        return claims(args)

    out = {}
    if args.recorded:
        sys.path.insert(0, HERE)
        from mcl_report import load as load_rec
        hall, times, truth, odom, scans, header = load_rec(args.recorded)
        triples = recorded_triples(scans, truth, args.pairs)
        print(f"{len(triples)} pairs from {args.recorded} "
              f"({header.get('world', '?')}, real beams, truth from /truth)")
        report(evaluate(triples, "point", stride=args.stride, sigma_z=args.noise))
        report(evaluate(triples, "line", stride=args.stride, sigma_z=args.noise))
    else:
        hall = load_hall(args.hall)
        noises = [args.noise] if not args.sweep_noise else [0.005, 0.02, 0.05, 0.15]
        for noise in noises:
            triples = synthetic_triples(hall, args.pairs, noise=noise, seed=args.seed)
            print(f"{len(triples)} pairs in {GridMap(hall)}, beam σ {noise * 1000:.0f} mm, "
                  f"stride {args.stride}")
            p = evaluate(triples, "point", stride=args.stride, sigma_z=max(noise, 0.02))
            l = evaluate(triples, "line", stride=args.stride, sigma_z=max(noise, 0.02))
            report(p)
            report(l, ref=p)
            out[str(noise)] = {"point": p, "line": l}
            if args.basin:
                out[str(noise)]["basin"] = basin(hall, triples)

    if args.degenerate:
        out["degenerate"] = degenerate(args.noise)
    if args.json:
        with open(args.json, "w") as fh:
            json.dump(out, fh, indent=1, default=str)
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
