#!/usr/bin/env python3
"""Exercise L1/L2 — Monte-Carlo localization, running on the simulator's own topics.

This is the **reference solution** of the exercise: a complete working filter, no TODOs.  The student
version of this file is generated from it by `tools/make_template.py` (not in this run — see
`todo.md`), and what it removes is the body of the two functions that carry the exercise: the
weighting of a scan and the resampling decision.

What is already here, and what that is for
------------------------------------------
Everything a group would otherwise lose to plumbing rather than to estimation:

* the node lifecycle — `robot_io.serve()` at the bottom, `/sim/task` switching the run underneath it;
* a loop that ticks on **message stamps** and never on the wall clock, throws away the measurements
  the bus still remembers from the previous task, and restarts the filter after a long gap instead of
  extrapolating it.  `tools/fastgrade.py` runs the simulator 25× faster than real time; a filter
  driven by `time.monotonic()` estimates a drive that never happened;
* the map: `GridMap(load_hall(name))` reads the same `worlds/<name>.txt` the simulator builds its
  walls from, so the map cannot disagree with the physics — no `OccupancyGrid` topic is published by
  the simulator yet, and this exercise does not wait for one;
* the report on `/<robot>/kf/pose` with 1σ, which the grader measures RMSE *and* NEES against;
* the parameters, which come from `config/tasks_localization.json` under the running task's `mcl`
  block — the same single-source-of-JSON idiom the simulator uses for its thresholds, so a task can
  ask for a wide prior or a small cloud without a line of this file changing.

The filter itself is `ohm_localization.mcl.MonteCarloLocaliser`, the same object the offline tests in
`test/test_mcl.py` run against; `update()` is the whole sensor model and `predict_odometry()` the
whole motion model.  Keeping that separation is why the numbers in `docs/verification.md` and the
numbers `./lab grade` prints are the same quantity measured twice, once with a clock and once without.

Run it
------
    tools/run_lab.sh grade --task mcl_production --controller solution/mcl_node.py
    tools/run_lab.sh run   --world production --task mcl_production --robot alice \\
                               --controller solution/mcl_node.py --truth

Measured on this checkout (docs/verification.md §1): all four tasks pass, 130/130.  L1 grades at
**15 mm** RMSE against **187 mm** of raw odometry — **12.5×** — with 1200 particles and 107 of 360
beams, NEES 0.35, 34 Hz; the same drive replayed offline by `tools/mcl_report.py` measures 13 mm, the
replay being optimistic by the factor the graded window's warm-up accounts for (§2).  L3, at 250
particles and every beam, grades 55 mm at 7.88×; L4, where the sensor's σ is 250 mm, needs `sigma_z`
0.5 from its own `mcl` block and grades 56 mm.

The parameters here are not tuned for the grader: they are the ones the task file asks for, and the
thresholds were written after the measurements rather than before them.  The one thing that does not
work anywhere is this filter in an open hall — `arena` has 102 echoing beams against `production`'s 323,
and the ceiling measured there is 1.12× the odometry, reached by setting σ_z to a metre (§4).  That is
why `arena` is a protocol question and not a task.
"""
import json
import math
import os
import sys

from ohm_localization import paths
from ohm_localization.gridmap import load_map
from ohm_localization.mcl import MclParams, MonteCarloLocaliser

# No `from mecanum_lab import robot_io` here, on purpose. The rest of this package imports numpy and nothing
# else, so `import ohm_localization` works on a laptop with no ROS and no simulator in sight (there is a test
# for that in tools/check.sh), and a message package that is not built cannot take the map layer down with
# it. `main()` is the one place the simulator is needed, and it imports it there.

# `OHM_MCL_TRACE=/tmp/trace.txt` writes one line per scan: t, the estimate and its sigma, N_eff, the
# beam count, the number of 5 cm boxes the cloud stands in, and the resample counter — every quantity
# the filter computed out of what it was given, and nothing it was not given.  The truth is not in this
# file and the node never reads `rob.truth()`; when a live run disagrees with `tools/mcl_report.py`,
# which is the only way to find out whether the filter or the offline replay is the liar, the answer is
# in which column diverges first, and that answer has to come from the filter's own numbers.
TRACE = open(os.environ["OHM_MCL_TRACE"], "w") if os.environ.get("OHM_MCL_TRACE") else None

REPORT_DT = 0.02          # s = 50 Hz: kf/pose report rate, as in the KF experiment
SLEEP = 2.0               # s: a longer gap is a pause or a respawn, not a prediction
SIGMA_THETA_PRIOR = 0.6   # rad, 1σ of the initial heading: about ±35°, a robot that was parked

# Defaults, in the units the sensor model has.  A task's "mcl" block overrides them; an environment
# variable overrides that, because the way to learn what sigma_z does is to change it and look.
DEFAULTS = {"particles": 1200, "beam_stride": 3, "sigma_z": 0.15, "z_rand": 0.04,
            "prior_sigma": 0.5, "resample_below": 0.5, "inject_below": 0.0,
            "sigma_floor": 0.01, "noise_rate_xy": 0.045, "noise_rate_theta": 0.09,
            "alpha1": 0.05, "alpha2": 0.05, "alpha3": 0.05, "drop_uninformative": True}


def _coerce(default, raw: str):
    """An environment variable as the type the default already has — `"false"` is a bool, not a 0."""
    text = raw.strip()
    if isinstance(default, bool):
        if text.lower() in ("true", "1", "yes", "on"):
            return True
        if text.lower() in ("false", "0", "no", "off"):
            return False
        raise ValueError(f"{raw!r} is not a yes/no value")
    return type(default)(text)


def task_options(task_id: str) -> dict:
    """The `mcl` block of the running task, with `OHM_MCL_*` on top of it.

    Read from the task file rather than hard-coded here for the same reason the simulator keeps its
    thresholds in JSON: the file that says what a task requires is then also the file that says what
    the filter was told, and a task that asks for a 2 m prior does not need a second place that knows.

    A value that will not parse is reported and ignored rather than raised: this runs inside the
    grader's process, where a `ValueError` from a parameter lookup comes out as `failed:ValueError`
    on the task and reads like a broken filter rather than like a typo in a shell.
    """
    opt = dict(DEFAULTS)
    try:
        with open(task_file(), encoding="utf-8") as fh:
            cfg = json.load(fh)
        block = next((t.get("mcl") or {} for t in cfg.get("tasks", []) if t.get("id") == task_id), {})
        opt.update({k: v for k, v in block.items() if k in opt})
    except (OSError, ValueError, StopIteration) as exc:
        print(f"mcl_node: task parameters not readable ({exc}) — using the defaults", file=sys.stderr)
    for key, value in list(opt.items()):
        env = os.environ.get(f"OHM_MCL_{key.upper()}")
        if env is None:
            continue
        try:
            opt[key] = _coerce(value, env)
        except (TypeError, ValueError):
            print(f"mcl_node: OHM_MCL_{key.upper()}={env!r} is not of the right kind — keeping "
                  f"{value}", file=sys.stderr)
    return opt


def world_name(rob) -> str:
    """Which hall this run is in — from /sim/world, or from the config if the sim is not ticking."""
    name = str((rob.world() or {}).get("name") or rob.config("world", "arena") or "arena")
    return name


def stamp(*measure) -> float:
    """Newest stamp among the measurements given — 0.0 for those that are not there yet."""
    return max([m.t for m in measure if m is not None] + [0.0])


def task_file() -> str:
    """The JSON the parameters come from, in whichever layout this run happens to be in.

    A function and not a module constant: the same file has to be found next to the checkout when the
    exercise is graded from a clone, and under `<prefix>/share/ohm_localization/config/` after a colcon
    build, and an environment override (`OHM_TASKS`, for a group's own variant of a task) is only worth
    having if setting it after the import still works.
    """
    return paths.task_file()


def mission(rob, task):
    """Localise for as long as this task runs; the runner reports "done" afterwards.

    One scan is worth one update, one odometry message one prediction — both keyed on the message
    stamp, because the bus hands out the *last* message it received, which is the same object until a
    new one arrives, and weighting a scan twice would double-count the world.
    """
    opt = task_options(task)
    hall = world_name(rob)
    grid = load_map(hall)
    params = MclParams(particles=int(opt["particles"]), beam_stride=int(opt["beam_stride"]),
                       sigma_z=float(opt["sigma_z"]), z_rand=float(opt["z_rand"]),
                       resample_below=float(opt["resample_below"]),
                       inject_below=float(opt["inject_below"]),
                       sigma_floor=float(opt["sigma_floor"]),
                       noise_rate_xy=float(opt["noise_rate_xy"]),
                       noise_rate_theta=float(opt["noise_rate_theta"]),
                       alpha1=float(opt["alpha1"]), alpha2=float(opt["alpha2"]),
                       alpha3=float(opt["alpha3"]),
                       drop_uninformative=bool(opt["drop_uninformative"]))
    prior = float(opt["prior_sigma"])
    print(f"mcl_node: {hall} — {grid}, {params.particles} particles, "
          f"1 beam in {params.beam_stride}, sigma_z {params.sigma_z} m, prior ±{prior:.2f} m",
          file=sys.stderr)

    f, baseline = None, stamp(rob.odom(), rob.scan())      # what the bus already knew
    last_odom_t, last_scan_t, letzter_meldung = 0.0, 0.0, -1e9
    while rob.running() and rob.task() == task:
        rob.spin(0.005)
        o, scan = rob.odom(), rob.scan()
        if o is None or stamp(o, scan) <= baseline:
            continue                      # still the last messages of the previous task, not this drive
        if f is None:
            # Start from the odometry with the task's prior around it.  Note what this does *not*
            # assume: it is not the odometry's pose that matters but the radius, and a task that says
            # 2 m gets a cloud that has to be narrowed by the LIDAR rather than inherited from a
            # wheel encoder.  `rob.truth()` exists on this bus in grade mode and is never read here —
            # a filter that looks at the answer is not a filter, it is a report.
            f = MonteCarloLocaliser(grid, params, pose=(o.x, o.y, o.theta),
                                    sigma=(prior, prior, SIGMA_THETA_PRIOR), prior="gaussian")
            last_odom_t, last_scan_t = o.t, 0.0
        if o.t > last_odom_t:
            if o.t - last_odom_t > SLEEP:      # five seconds of dead reckoning is not a prediction
                f = MonteCarloLocaliser(grid, params, pose=(o.x, o.y, o.theta),
                                        sigma=(prior, prior, SIGMA_THETA_PRIOR), prior="gaussian")
                f.last_odom = None
            f.predict_odometry(o)
            last_odom_t = o.t
        if scan is not None and scan.t > last_scan_t:
            last_scan_t = scan.t
            f.update(scan)
            if TRACE is not None:                # diagnostics only; see the note above `task_options`
                t_e = f.estimate()               # the fresh one, not the loop's last-known `e`
                TRACE.write(f"{scan.t:.3f} {t_e['x']:.4f} {t_e['y']:.4f} {t_e['theta']:.4f} "
                            f"{t_e['sx']:.4f} {t_e['sy']:.4f} {t_e['sth']:.4f} {t_e['neff']:.1f} "
                            f"{t_e['beams']} {f.diversity()} {t_e['resamples']}\n")
                TRACE.flush()
        e = f.estimate()                          # the newest estimate, on every loop, for `rate_min`
        if o.t - letzter_meldung >= REPORT_DT:
            letzter_meldung = o.t
            rob.send_kf(e["x"], e["y"], e["theta"], e["sx"], e["sy"], e["sth"],
                        info={"t": round(o.t, 2), "world": hall, "particles": e["particles"],
                              "beams": e["beams"], "neff": round(e["neff"], 1),
                              "sigma_z": params.sigma_z, "prior_sigma": prior,
                              "places": f.diversity(),      # N_eff counts particles, this counts poses
                              "resamples": e["resamples"], "degenerate": e["degenerate"]})


def main() -> None:
    """`ros2 run ohm_localization mcl_node --robot alice`, and `python3 -m ohm_localization.mcl_node`.

    One module, three doors: graded in a single process (the simulator imports `solution/mcl_node.py`, which
    re-exports `mission` from here), run as a second process over ROS by `node controller --controller`, and
    started as this executable by `ros2 launch ohm_localization mcl.launch.py`. The same code behind all
    three, so the numbers in `docs/verification.md` describe whichever door a group used — a reference
    solution that only works through the door it was measured on is not a reference, it is an anecdote.
    """
    from mecanum_lab import robot_io                 # here, not at module import: see the note at the top
    robot_io.serve(sys.modules[__name__])


if __name__ == "__main__":
    # The runner owns the bus and the lifecycle and calls mission() once per task exactly.
    main()
