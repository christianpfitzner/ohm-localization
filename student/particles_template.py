"""E4 — generating the particles of a Monte Carlo localiser.  Five functions are yours; `move_particles` is the one that decides the run.

    python3 tools/lab_check.py e4_particles             # the seven criteria of the sheet, with the numbers
    python3 student/particles_template.py               # a demo: a prior, a drive of 20 steps, a resampling

**What this exercise is for.** A particle is not an estimate; a cloud of particles is. These five functions
are the whole lifecycle of that cloud — draw the prior, move it by one odometry step, resample it by the
weights the sensor model will produce, and read one pose and one σ off it — and every one of them has a
plausible-looking implementation that the criteria below catch. The weights themselves are the sensor model,
which is the graded part of the next visit (`student/mcl_template.py`, task L1); here the machinery that has
to be right around it is what is measured.

**Two of the seven criteria are about spread, and they are the two that matter.** The reference filter in
this repository was run twice with the same parameters over the same drive and came out at **0.20 m** and
**1.26 m** of RMSE. The only difference was that one version predicted once per LIDAR scan (20 Hz) and the
other once per wheel message (50 Hz). A per-step σ means *per message*, so the second version spread its
cloud by √2.5 per second for no reason but the topic rate, and in a hall where the LIDAR says little nothing
was left to pull it back in (`ohm_localization/mcl.py:predict()` carries that measurement into the code, and
`docs/verification.md` §13 has the run; the same trap is `docs/icp.md` §10 for ICP's σ).

The motion noise is therefore stated in two different kinds, and knowing which is which is a large part of the
exercise. The three **alphas** are proportional to the motion inside the step — a step that moves 1 cm
contributes 1 cm of spread whether it arrives every 20 ms or every 200 ms — and the two **floors**
(`noise_rate_xy`, `noise_rate_theta`) are per square root of second, because they are what a robot that is
standing still still gets wrong. `noise-floors-are-rates` measures the floors with the alphas switched off: one
second of the same motion spreads the cloud 0.045 m at 20 Hz and 0.045 m at 10 Hz when the floor is divided by
√dt, and 0.202 m against 0.141 m when it is not. The alphas are deliberately *not* held invariant, and
`ohm_localization/exercises.py:criterion_particles_motion_rate()` says why in one paragraph.

In the other direction: a cloud that cannot widen, or a σ you invent instead of measuring, is a filter that
walks into a wrong pose confidently a few seconds later. `the-cloud-spreads` has a band with **two** edges for
that reason, and the lower one is the edge that catches the dead-reckoning implementation of a "motion model".

**The five functions, and the convention each one is graded in.** All poses are `(x, y, θ)` in the **hall**
frame with θ in (−π, π]; `dt` is the period this step covers, in seconds; `rng` is a
`numpy.random.Generator` and no function may seed one itself (a filter that reseeds is a filter that repeats
itself).

    sample_uniform(grid, n, rng) -> (n, 3)        the global localisation prior: n poses on the free floor
    sample_gaussian(center, sigma, n, rng) -> (n, 3)   the localised prior, with the heading wrapped
    move_particles(x, rot1, trans, rot2, dt, p, rng) -> (n, 3)   one odometry step, sampled per particle
    resample(x, w, n, rng) -> (n, 3)              copies in the ratio of the weights
    weighted_estimate(x, w) -> dict               x, y, theta and sx, sy, sth — the σ from the spread

`p` is an `ohm_localization.mcl.MclParams` — the same object the running filter is handed, carrying `alpha1`,
`alpha2`, `alpha3` and `noise_rate_xy`, `noise_rate_theta` — and `move_particles` takes its arguments in the
same order as `MonteCarloLocaliser.predict`, so what you write here is the function that filter would call.
Both noise paths have to work: with all five of those fields at zero the step is a rigid transform, and that is
a criterion.

**Read the criteria before you write.** `ohm_localization/exercises.py` is 300 lines and every criterion is
one function with a docstring that says what it measures and why; the grader is not hidden, it is documented.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

# Run it from anywhere: `python3 student/particles_template.py`. The path has to be fixed before the import
# below, not in the `__main__` block at the bottom, where it arrives too late for a module-level import.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ohm_localization.gridmap import GridMap, load_hall          # noqa: E402

HALL = "production"             # the map the criteria sample against; any hall file works for the demo


# -------------------------------------------------------------------------------------- yours: the prior
def sample_uniform(grid: GridMap, n: int, rng: np.random.Generator) -> np.ndarray:
    """`n` poses spread over the free floor, with a random heading. **TODO(E4).**

    Two failure modes and the criterion measures both. Samples inside a wall are a filter that will never
    explain one beam — and the failure is invisible in the mean, which is why the criterion looks at the
    sample nearest a wall and not at the average. Samples all in one corner are a filter that has already
    decided the answer; the criterion counts the distinct 0.5 m cells your cloud hits and the ratio of the
    busiest to the quietest quarter of the hall, so a cloud that samples the free cells in order and stops
    early does not pass.

    `grid.sample_free(n, rng)` is three lines of the library and passes this criterion; writing your own is
    the exercise, and the lines worth understanding are how the free cells are chosen and why the heading is
    drawn uniformly in (−π, π] rather than in [0, 2π).
    """
    raise NotImplementedError("TODO(E4): sample_uniform() — see tools/lab_check.py e4_particles")


def sample_gaussian(center, sigma, n: int, rng: np.random.Generator) -> np.ndarray:
    """`n` poses around `center` with the spread `sigma`, both (x, y, θ) triples. **TODO(E4).**

    The σ is the whole content of this function. A prior σ that is silently too small is the most expensive
    bug in the filter, because everything downstream — the weights, N_eff, the resampling — then behaves as
    though the robot were known, and the estimate comes out confident and wrong. The criterion measures the
    σ of your sample against the σ you were given, on 4000 draws, where the sampling error of the measurement
    itself is under 2 %.

    The heading is where implementations die: a prior at θ = 3.0 rad with σ = 0.5 has **39 %** of its samples
    past π, which have to come back around at −π, and every other part of the filter assumes θ ∈ (−π, π]. Add
    the noise, then wrap. A generator that clips instead of wrapping is caught by the criterion's wrap band,
    which is not a trick question — 39 % of that prior really is past the wrap.
    """
    raise NotImplementedError("TODO(E4): sample_gaussian() — remember the wrap")


# --------------------------------------------------------------------------------- yours: the motion model
def move_particles(x: np.ndarray, rot1: float, trans: float, rot2: float, dt: float, p,
                   rng: np.random.Generator) -> np.ndarray:
    """One odometry step `(rot1, trans, rot2)`, sampled independently for every particle. **TODO(E4).**

    The step is what the wheels reported since the last period: turn `rot1`, drive `trans` in the heading that
    results, turn `rot2`. It is the same three-part step for all particles, because it is one message about
    one robot; the *noise* is drawn per particle, which is the entire difference between a motion model and a
    dead-reckoning node.

    With every noise field of `p` at zero this is exactly the rigid transform, and `noiseless-motion-is-rigid`
    checks it to the micrometre. Three mistakes surface there and nowhere else: the odometry **pose** used
    instead of the odometry **step** (the filter then teleports its cloud onto the wheel estimate and throws
    away every hypothesis it had), the rotation taken about the world origin instead of about the robot, and
    one rotation where the model has two.

    With the noise on, draw three terms — one per rotation and one for the distance — rather than one draw for
    the whole step, each with a σ of the shape

        s₁ = |α₁·rot1 + α₂·trans| + noise_rate_theta·√dt            the first rotation
        s₂ = α₂·|trans| + α₁·(|rot1| + |rot2|) + noise_rate_xy·√dt   the distance
        s₃ = |α₃·rot2 + α₂·trans| + noise_rate_theta·√dt            the second rotation

    which is what `mcl.py:predict()` uses, term by term, for the reasons given in its docstring. Two of those
    terms are viva questions anyway: why does turning make the *distance* uncertain (the second term of `s₂` —
    the two rotations and the translation are computed from the same wheel ticks, and a wheel that slips in
    heading slips in distance), and why does only the floor carry √dt while the alphas do not (the paragraph at
    the top of this file, and the criterion that measures it).
    """
    raise NotImplementedError("TODO(E4): move_particles() — three draws, the floors scaling with √dt")


# ------------------------------------------------------------------------- yours: resampling and estimate
def resample(x: np.ndarray, w: np.ndarray, n: int, rng: np.random.Generator) -> np.ndarray:
    """`n` particles drawn from `x` in the ratio of `w`. **TODO(E4).**

    Three properties, none of them visible in the mean of the result: a particle carrying 90 % of the weight
    comes back about 90 % of the time (a resampler that sorts and keeps the best n keeps it **once** and
    throws the belief away with the rest), a particle with zero weight never comes back, and the cloud still
    holds `n` members — a shrinking cloud is a filter that runs out of hypotheses quietly, and it shrinks
    exactly when the weights are peaked, which is when the filter most needs the spread.

    Systematic resampling gets all three, has the lowest variance of the common schemes, and is eight lines:
    one uniform draw `u ∈ [0, 1/n)`, `n` pointers at `u + k/n`, and the cumulative weights walked once. The
    naive `rng.choice(n, p=w)` is correct and noisier; if you use it, say why in the protocol.
    """
    raise NotImplementedError("TODO(E4): resample() — systematic, one uniform draw")


def weighted_estimate(x: np.ndarray, w: np.ndarray) -> dict:
    """One pose and one σ from the cloud: `{"x", "y", "theta", "sx", "sy", "sth"}`. **TODO(E4).**

    The position is the weighted mean and its σ is the weighted spread of the cloud — that number is what the
    grader scores as NEES and what RViz draws the covariance ellipse from, so it is a claim about the sensor
    and the map and not a setting.

    The heading is averaged as a **direction**, not as a number: `Σw·θ / Σw` of a cloud straddling ±π answers
    0 rad for a cloud that means 180°, and the failure is invisible in every other number the filter prints —
    the σ is fine, the position is fine, and the robot drives backwards. Average `cos θ` and `sin θ` and take
    `atan2` of the two; the σ of the heading is then the circular standard deviation of the same directions.
    """
    raise NotImplementedError("TODO(E4): weighted_estimate() — and wrap the heading")


# ----------------------------------------------------------------------------------------------- the demo
def demo(n: int = 1200, steps: int = 20) -> None:
    """A prior, twenty odometry steps, one resampling — printed the way the criteria measure it."""
    from ohm_localization.exercises import NOISE                 # the same rates the criteria hand over
    grid = GridMap(load_hall(HALL))
    rng = np.random.default_rng(1)
    try:
        x = sample_uniform(grid, n, rng)
    except NotImplementedError as exc:
        print(f"{exc}\nthe demo continues with the library's own sampler, so you can see the shape of it:")
        x = grid.sample_free(n, rng)
    print(f"{HALL}: {n} particles, x from {x[:, 0].min():.1f} to {x[:, 0].max():.1f} m, "
          f"{len({(int(a / 2), int(b / 2)) for a, b in x[:, :2]})} of the hall's 2 m squares hit")
    for k in range(steps):
        try:
            x = move_particles(x, 0.05, 0.3, 0.02, 0.05, NOISE, rng)
        except NotImplementedError as exc:
            print(f"{exc}\n  stopping there — the rest of the demo needs the motion model.")
            return
    spread = float(np.hypot(*x[:, :2].std(axis=0)))
    print(f"after {steps} steps of 50 ms: spread {spread:.3f} m, mean "
          f"({x[:, 0].mean():.2f}, {x[:, 1].mean():.2f}) m")
    w = np.exp(-((x[:, 0] - x[:, 0].mean()) ** 2 + (x[:, 1] - x[:, 1].mean()) ** 2) / 0.02)
    w /= w.sum()
    print(f"N_eff before resampling: {1.0 / float((w ** 2).sum()):.0f} of {n}")
    try:
        y = resample(x, w, n, rng)
        e = weighted_estimate(y, np.full(n, 1.0 / n))
    except NotImplementedError as exc:
        print(f"{exc}")
        return
    print(f"resampled: spread {float(np.hypot(*y[:, :2].std(axis=0))):.3f} m, estimate "
          f"({e['x']:.3f}, {e['y']:.3f}, {math.degrees(e['theta']):.1f}°) ± "
          f"({e['sx']:.3f}, {e['sy']:.3f} m, {math.degrees(e['sth']):.1f}°)")
    print("tools/lab_check.py e4_particles measures the seven criteria; this demo only shows their shape.")


if __name__ == "__main__":
    demo()
