#!/usr/bin/env python3
"""Replay a recorded drive through the localiser and report what it did, offline and offline-fast.

    python3 tools/mcl_report.py /tmp/arena.jsonl
    python3 tools/mcl_report.py /tmp/arena.jsonl --particles 250 --stride 1 --sigma-z 0.15
    python3 tools/mcl_report.py /tmp/arena.jsonl --table --timing

Why a replay rather than another live run: `./lab grade` answers one question — did it pass — with the
parameters the task file chose, and it takes 48 s of wall clock to say no.  This takes the recorded
scans and odometry of that same drive and answers the questions a group actually asks while working:
how far off, how fast, with how many particles, using how many beams, how long until it had found the
robot, and how much of that is the sensor model rather than the motion model.  Every threshold in
`config/tasks_localization.json` was read off a run of this file (docs/verification.md lists them).

The numbers are the same *quantities* the grader measures, with two honest differences, both printed:

* the replay sees one odometry message per scan (the recording stores them together), so it predicts
  at 20 Hz where the live node predicts at 50 Hz.  The motion noise has a per-step floor, so a
  coarser step means a slightly tighter cloud — a few millimetres, and the way to see it is `--stride`
  with everything else fixed;
* the grader's `improvement` divides by the RMSE of the raw sensor over the *graded* window, which
  starts after a warm-up; this one covers the whole recording.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ohm_localization.gridmap import GridMap, parse_grid                    # noqa: E402
from ohm_localization.mcl import MclParams, MonteCarloLocaliser             # noqa: E402

WIDTH = dict(rmse=7, maxerr=7, impr=6, nees=6, neff=8, converge=9)


def load(path):
    """A recording as arrays: poses, scans in the shape `Scan` has, and the hall it was driven in."""
    header, frames = None, []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if obj.get("header"):
                header = obj
            else:
                frames.append(obj)
    if header is None or not frames:
        raise SystemExit(f"{path}: no header line or no frames — record it first "
                         f"(tools/record_scans.py)")
    hall = parse_grid(header["grid_map_text"], cell=float(header.get("cell", 0.5)),
                      name=str(header.get("world", "?")))
    scans, truth, odom, t = [], [], [], []
    for f in frames:
        rng = [np.inf if r is None else float(r) for r in f["ranges"]]
        scans.append({"t": float(f["t"]), "ranges": rng,
                      "angle_min": float(f.get("angle_min", 0.0)),
                      "angle_increment": float(f.get("angle_increment", 2 * math.pi / len(rng))),
                      "range_min": float(f.get("range_min", 0.05)),
                      "range_max": float(f.get("range_max", 8.0))})
        truth.append(f["truth"])
        odom.append(f["odom"])
        t.append(float(f["t"]))
    if any(x is None for x in truth):
        print("note: some frames have no /truth (recorded without debug_truth) — those frames "
              "are dropped from the error columns", file=sys.stderr)
    keep = [i for i in range(len(t)) if truth[i] is not None]
    return hall, np.array(t)[keep], [truth[i] for i in keep], [odom[i] for i in keep], \
        [scans[i] for i in keep], header


def replay(grid, truth, odom, scans, params, prior=0.5, sigma_theta=0.6, seed=7, loop_weights=False):
    """The node's loop, without the node: same order of predict/update, same report."""
    f = MonteCarloLocaliser(grid, params, pose=tuple(odom[0][:3]), sigma=(prior, prior, sigma_theta),
                            rng=np.random.default_rng(seed))
    err, neff, beams, sx, sy, wall = [], [], [], [], [], []
    t0 = time.perf_counter()
    t_prev = None
    for tp, op, scan in zip(truth, odom, scans):
        # The interval comes from the recording, not from a default: `predict_odometry` cannot infer
        # it from a bare triple, and the whole point of the rate semantics is that a step means a
        # *time*, so a replay that guessed the time would be a replay of a different filter.
        f.predict_odometry(op[:3], dt=None if t_prev is None else scan["t"] - t_prev)
        t_prev = scan["t"]
        if loop_weights:                       # the same sensor model, written as Python loops
            by_loop = f.weights_with_a_for_loop(scan)
            f.logw = by_loop - by_loop.max()
            w = np.exp(f.logw)
            f.w = w / w.sum()
            f.last_neff = f.neff()
            if f.last_neff < params.resample_below * f.n:
                f.resample()
        else:
            f.update(scan)
        e = f.estimate()
        err.append(math.hypot(e["x"] - tp[0], e["y"] - tp[1]))
        neff.append(e["neff"])
        beams.append(e["beams"])
        sx.append(e["sx"])
        sy.append(e["sy"])
        wall.append(grid.occupied_at(e["x"], e["y"]))
    dt = (time.perf_counter() - t0) / max(len(scans), 1)
    places = [f.diversity() for _ in [0]]        # measured once at the end, see below
    return (np.array(err), np.array(neff), np.array(beams), np.array(sx), np.array(sy),
            np.array(wall, dtype=bool), dt, f, places[0])


def rms(v):
    return float(np.sqrt(np.mean(np.square(v)))) if len(v) else float("nan")


def converged_after(err, times, limit=0.25, hold=10):
    """First time the error stays under `limit` for `hold` scans: how long finding the robot took."""
    for i in range(len(err) - hold):
        if (err[i:i + hold] < limit).all():
            return float(times[i] - times[0])
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("recording", nargs="?", help="JSONL from tools/record_scans.py")
    ap.add_argument("--particles", type=int, default=None)
    ap.add_argument("--stride", type=int, default=None)
    ap.add_argument("--sigma-z", type=float, default=None)
    ap.add_argument("--prior", type=float, default=None, help="m, 1σ of the initial cloud")
    ap.add_argument("--z-rand", type=float, default=None)
    ap.add_argument("--no-drop", action="store_true", help="weight the beams at range_max too")
    ap.add_argument("--inject", type=float, default=None, help="fraction of N: random particles")
    ap.add_argument("--loop-weights", action="store_true", help="the same model as Python loops")
    ap.add_argument("--timing", action="store_true", help="time both implementations, 20 scans each")
    ap.add_argument("--table", action="store_true", help="one line every 5 seconds of the drive")
    ap.add_argument("--json", metavar="FILE", help="also write the series as JSON")
    args = ap.parse_args(argv)
    if not args.recording:
        ap.print_help()
        return 2

    hall, times, truth, odom, scans, header = load(args.recording)
    grid = GridMap(hall)
    p = MclParams()
    for name, value in (("particles", args.particles), ("beam_stride", args.stride),
                        ("sigma_z", args.sigma_z), ("z_rand", args.z_rand),
                        ("inject_below", args.inject)):
        if value is not None:
            setattr(p, name, value)
    if args.no_drop:
        p.drop_uninformative = False
    prior = 0.5 if args.prior is None else args.prior

    odo_err = np.array([math.hypot(o[0] - t[0], o[1] - t[1]) for o, t in zip(odom, truth)])
    err, neff, beams, sx, sy, in_wall, per_update, f, places = replay(
        grid, truth, odom, scans, p, prior=prior, loop_weights=args.loop_weights)
    nees = float(np.mean(err ** 2 / (0.5 * (sx ** 2 + sy ** 2))))
    conv = converged_after(err, times)
    late = err[len(err) // 2:]

    print(f"{hall.name}: {len(scans)} scans over {times[-1] - times[0]:.1f} s, "
          f"grid {grid}, prior ±{prior:.2f} m")
    print(f"  {p.particles:5d} particles, 1 beam in {p.beam_stride} "
          f"({int(np.mean(beams))} of {len(scans[0]['ranges'])} beams used), "
          f"sigma_z {p.sigma_z} m, z_rand {p.z_rand}, "
          f"{'loops' if args.loop_weights else 'vectorised'}")
    print(f"  RMSE {rms(err)*1000:6.0f} mm   late {rms(late)*1000:6.0f} mm   max {err.max()*1000:6.0f} mm"
          f"   (odometry alone: {rms(odo_err)*1000:.0f} mm RMSE, {odo_err.max()*1000:.0f} mm worst)")
    print(f"  improvement over odometry {rms(odo_err)/max(rms(err),1e-9):6.2f}x   "
          f"NEES {nees:6.2f}   N_eff min {neff.min():5.0f} median {np.median(neff):5.0f}   "
          f"{places} distinct 5 cm boxes (N_eff counts particles, this counts poses)   "
          f"in a wall {100*in_wall.mean():.1f} % of the time")
    print(f"  converged (<25 cm, held) after "
          f"{'never' if conv is None else f'%.1f s' % conv}   "
          f"{1000*per_update:5.1f} ms per update   counters: {f.steps} steps, {f.resamples} resamples, "
          f"{f.injected} injected, {f.degenerate} degenerate scans")
    if rms(odo_err) < 0.05:
        print("  note: this drive's odometry is only "
              f"{rms(odo_err)*100:.0f} cm off in RMSE — a filter cannot earn a factor of two against "
              f"that, and a threshold that asks for one is grading the sensor profile, not the filter.")
    if in_wall.any():
        print(f"  note: the estimate sat inside a wall {int(in_wall.sum())} times — a pose that the "
              f"map rules out is not a rounding error, it is the sensor model or the motion model.")

    if args.table:
        step = max(int(round(5.0 / max(times[1] - times[0], 1e-9))), 1)
        print("\n   t [s]   err [mm]  N_eff  beams  sx [mm]  in wall")
        for i in range(0, len(err), step):
            print(f"  {times[i]-times[0]:6.1f} {err[i]*1000:8.0f} {neff[i]:6.0f} {beams[i]:6d} "
                  f"{sx[i]*1000:7.1f} {str(bool(in_wall[i])):>7s}")

    if args.timing:
        n = min(20, len(scans))
        for label, loop in (("vectorised", False), ("python loops", True)):
            g2 = GridMap(hall)
            f2 = MonteCarloLocaliser(g2, p, pose=tuple(odom[0][:3]), sigma=(prior, prior, 0.6),
                                     rng=np.random.default_rng(7))
            t0 = time.perf_counter()
            for op, scan in zip(odom[:n], scans[:n]):
                f2.predict_odometry(op[:3])
                if loop:
                    w = f2.weights_with_a_for_loop(scan)
                    f2.logw = w - w.max()
                    ww = np.exp(f2.logw)
                    f2.w = ww / ww.sum()
                    f2.last_neff = f2.neff()
                else:
                    f2.update(scan)
            el = (time.perf_counter() - t0) / n
            print(f"  {label:13s}: {1000*el:6.2f} ms per update "
                  f"({p.particles} particles × {int(np.mean(beams))} beams, "
                  f"{p.particles*int(np.mean(beams))/el/1e6:.1f} M beam looks/s)")

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump({"world": hall.name, "t": times.tolist(), "error": err.tolist(),
                       "neff": neff.tolist(), "beams": beams.tolist(), "sx": sx.tolist(),
                       "sy": sy.tolist(), "odom_error": odo_err.tolist(),
                       "params": {k: getattr(p, k) for k in
                                  ("particles", "beam_stride", "sigma_z", "z_rand",
                                   "drop_uninformative", "inject_below")},
                       "prior": prior, "rmse": rms(err), "rmse_odom": rms(odo_err),
                       "nees": nees, "converged_after": conv}, fh)
        print(f"  series written to {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
