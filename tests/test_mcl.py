"""The particle filter: does it converge, how well, and does it know how well it has converged?

Every threshold here is a measurement from this checkout — the runs are in `docs/verification.md` —
and four of the tests assert that the filter *fails*.  That is the content of the exercise: MCL is
cheap, accurate and completely unreliable under conditions a student can name, and the conditions are
more interesting than the accuracy.
"""
import math

import numpy as np
import pytest

from conftest import free_pose                                   # noqa: F401
from ohm_localization import synth
from ohm_localization.gridmap import GridMap, load_hall
from ohm_localization.mcl import MclParams, MonteCarloLocaliser


class Run:
    """One offline localisation run, with the numbers a group would write on the results sheet."""

    def __init__(self, error, odo_error, neff, sx, sy, filter):
        self.error, self.odo_error, self.neff = error, odo_error, neff
        self.sx, self.sy, self.filter = sx, sy, filter

    @staticmethod
    def rms(v):
        return float(np.sqrt(np.mean(np.square(v)))) if len(v) else float("nan")

    @property
    def rmse(self):
        return self.rms(self.error)

    @property
    def improvement(self):
        return self.rms(self.odo_error) / self.rmse if self.rmse else float("nan")

    @property
    def nees(self):
        """The grader's own number: mean of e² / σ², in the 2D form it uses."""
        e2 = np.square(self.error) * 0.5                        # isotropic split of the 2D error
        return float(np.mean(e2 / (0.5 * (self.sx ** 2 + self.sy ** 2))))


def _run(hall, prior="gaussian", sigma=(0.5, 0.5, 0.35), seconds=30.0, params=None, seed=7,
         scan_every=2):
    """Scans at the true pose, odometry that drifts, no simulator and no clock: `python3 -m pytest`.

    The drive is searched for on free floor by `synth.free_path`, which also picks one with structure
    in it: a filter driven parallel to one long wall is being tested on a direction a LIDAR does not
    measure, and it scores like a broken filter when it is not one.
    """
    grid = GridMap(hall)
    true = synth.free_path(grid, vx=0.55, seconds=seconds, sine=1.1, frequency=0.09, seed=3)
    assert true is not None, f"{hall.name}: no drive of {seconds:g} s fits on the floor"
    odom = synth.fake_odometry(true, dt=0.05, scale=1.04, gyro_bias=0.012,
                               rng=np.random.default_rng(seed + 1))
    f = MonteCarloLocaliser(grid, params or MclParams(particles=1200, beam_stride=3),
                            rng=np.random.default_rng(seed), pose=tuple(true[0]), sigma=sigma,
                            prior=prior)
    out = {k: [] for k in ("error", "odo_error", "neff", "sx", "sy")}
    noise = np.random.default_rng(99)
    for i, (tp, op) in enumerate(zip(true, odom)):
        if i % scan_every:                                      # one scan every 0.1 s, as in the lab
            f.predict_odometry(op)
            out["neff"].append(f.update(synth.scan_dict(tp, hall, t=i * 0.05, sigma=0.02, rng=noise)))
            e = f.estimate()
            out["error"].append(math.hypot(e["x"] - tp[0], e["y"] - tp[1]))
            out["odo_error"].append(math.hypot(op[0] - tp[0], op[1] - tp[1]))
            out["sx"].append(e["sx"])
            out["sy"].append(e["sy"])
    return Run(*(np.asarray(v) for v in out.values()), f)


# ---------------------------------------------------------------------------- the working case
def test_it_localises_and_is_far_better_than_the_odometry_alone(hall):
    """The claim the exercise rests on: 12 mm RMSE against 372 mm of odometry, 31×.

    Against raw odometry, which is what the simulator's grader compares a submission to.  The
    odometry here is the synthetic one — 4 % scale error and a 0.012 rad/s gyro bias, which is what
    the laboratory's robot actually shows — and it drifts 0.37 m over the 30 s drive, which is the
    number the filter has to beat rather than a millimetre-perfect one.
    """
    r = _run(hall)
    assert r.rmse < 0.05, f"particle filter RMSE {r.rmse:.3f} m"
    assert r.improvement > 8.0, f"only {r.improvement:.1f}x better than the odometry it was given"
    assert r.neff.min() > 100, f"the cloud collapsed to N_eff={r.neff.min():.0f}"
    assert r.filter.last_beams > 80, f"only {r.filter.last_beams} beams were used"


def test_a_two_metre_mistake_in_the_prior_is_gone_in_the_first_scan(hall):
    """Start 2 m and 50° off: the first scan fixes it, and the RMSE over the drive stays at 3 cm.

    This is what a prior is for in this hall, and it is also the limit of what the prior is asked to
    do here — the scan is so informative that a 2 m mistake is worth one update.  The corollary is
    uncomfortable and worth saying in the protocol: in a hall this well mapped, MCL is not much of a
    *global* localiser, it is an excellent scan matcher with a distribution attached.
    """
    r = _run(hall, sigma=(2.0, 2.0, 0.9))
    assert r.rmse < 0.10, f"from a 2 m prior: {r.rmse:.3f} m"
    assert r.improvement > 4.0, f"only {r.improvement:.1f}x better than odometry"


def test_sigma_matches_the_error_it_describes(hall):
    """NEES near 1: the spread the filter reports is the spread it is wrong by.

    Measured 0.1–2.4 over four halls with the shipped parameters, which is on the conservative side —
    the filter slightly overstates its uncertainty, because the motion noise floor (1 cm per step) is
    larger than the error the odometry actually makes at those speeds.  Deliberately: the same filter
    with the noise set to zero reports NEES in the hundreds (see the next test but one).
    """
    r = _run(hall)
    assert 0.05 < r.nees < 5.0, f"NEES {r.nees:.2f}: reported spread and real error disagree"


# --------------------------------------------------------------------------- where it stops working
def test_a_cloud_that_can_only_shrink_walks_into_a_wrong_pose(hall):
    """Motion noise at zero: 274 mm against 12 mm, and a NEES of 749 instead of 0.2.

    The failure every group has to be able to produce on demand, because it is the one that looks like
    success: the cloud tightens, N_eff stays high, the reported sigma falls, and the estimate follows
    the odometry — the filter has stopped listening to the LIDAR because it has convinced itself it
    already knows.  The particles need somewhere to *go* between two scans, and the sampling in
    `predict()` is where they get it.
    """
    live, rigid = _run(hall), _run(hall, params=MclParams(
        particles=1200, beam_stride=3, alpha1=0.0, alpha2=0.0, alpha3=0.0,
        noise_floor_xy=0.0, noise_floor_theta=0.0))
    assert rigid.rmse > live.rmse * 5.0, (f"no motion noise gave {rigid.rmse:.3f} m against "
                                          f"{live.rmse:.3f} m with it — the exercise lost its point")
    assert rigid.nees > 50.0, f"and it admitted the doubt (NEES {rigid.nees:.1f})?"


def test_a_uniform_prior_over_the_whole_hall_is_out_of_reach_at_this_particle_count(hall):
    """Global localisation, measured as the failure it is: 3.2 m and never better, at 1200 particles.

    `rooms` has 4432 free cells at 25 cm, so 1200 particles is a third of a particle per cell: the
    chance that any of them starts near the true pose is small, and a filter cannot weight its way to
    a mode it does not contain — the lecture's "recovering a lost robot" section exists for this, and
    its answer is more particles or random injection, not a better likelihood.  With 4000 particles the
    same run converges after 38 s; `arena`, which is 5520 free cells and mostly open floor, does not
    converge at 4000 either.

    Asserting the failure is not pessimism: it is the one number in this exercise that tells a group
    their particle count is a *budget* and that the budget is set by the size of the building.
    """
    r = _run(hall, prior="uniform")
    late = r.error[len(r.error) // 2:]
    assert Run.rms(late) > 1.0, f"it converged from a uniform prior ({Run.rms(late):.2f} m)"
    assert len(hall.rects) > 0
    cells = len(GridMap(hall).free_xy)
    assert 1200 < cells, f"{cells} free cells: the arithmetic of the docstring needs checking"


def test_weighting_a_beam_that_saw_nothing_bites_in_an_open_hall_and_nowhere_else():
    """The `lidar_no_echo` lesson, and the reason it has to be measured in the right hall.

    A beam at `range_max` did not measure a wall, so weighting it asks the map how well a particle
    agrees with a wall that is not there.  In `rooms` that costs nothing measurable — 15 mm against
    12 mm — because the walls are close and almost no beam reaches 8 m, so there is almost no `inf` to
    mistranslate.  `track` hides it too (30 mm against 23 mm).  The open `arena` does not: a third of
    the beams are at max range and the same one-line mistake costs a factor of seventeen, 1.42 m
    against 84 mm, with a NEES of 3518 instead of 0.19.

    A group that measures this in `rooms` writes up that the parameter is decoration.  The measurement
    is only as good as the hall, which is a general fact about this exercise and not only about this
    line.
    """
    good_arena, bad_arena = _run(load_hall("arena")), _run(
        load_hall("arena"), params=MclParams(particles=1200, beam_stride=3, drop_uninformative=False))
    assert good_arena.rmse < 0.15, f"reference run in arena: {good_arena.rmse:.3f} m"
    assert bad_arena.rmse > 5.0 * good_arena.rmse, (
        f"arena: weighting the empty beams cost only {bad_arena.rmse:.3f} m against "
        f"{good_arena.rmse:.3f} m — has this hall stopped producing max-range beams?")
    assert bad_arena.nees > 10.0 * good_arena.nees, f"honest anyway? (NEES {bad_arena.nees:.0f})"
    good_track, bad_track = _run(load_hall("track")), _run(
        load_hall("track"), params=MclParams(particles=1200, beam_stride=3, drop_uninformative=False))
    assert bad_track.rmse < 2.0 * good_track.rmse, (
        f"track: {bad_track.rmse:.3f} m against {good_track.rmse:.3f} m — the close walls were meant "
        f"to hide this mistake, and a test that fails here means the hall changed")


def test_random_injection_is_a_recovery_mechanism_and_not_a_performance_knob(hall):
    """Injecting 10 % fresh particles on every collapse costs a factor of six when nothing is lost.

    Measured from the same wide prior: 32 mm with injection off, 185 mm with it on, because a filter
    that keeps being told to start again never finishes starting.  `inject_below` is 0 by default and
    the reason is in that ratio — it is the answer to a kidnapped robot, and switching it on to improve
    a working filter is the same mistake as switching off the motion noise in reverse.
    """
    wide = (2.0, 2.0, 0.9)
    calm = _run(hall, sigma=wide)
    jittery = _run(hall, sigma=wide, params=MclParams(particles=1200, beam_stride=3,
                                                      inject_below=0.5))
    assert jittery.rmse > 2.0 * calm.rmse, (f"injection cost {jittery.rmse:.3f} m against "
                                            f"{calm.rmse:.3f} m: the knob stopped mattering")
    assert jittery.filter.injected > 0, "injection never triggered: nothing was measured"


# ----------------------------------------------------------------------------- the internals
def test_the_vectorised_weight_is_the_loop_weight(hall):
    """`weights_with_a_for_loop()` is offered as the same computation, so it must be the numbers.

    This is the test that catches a change to the sensor model in one implementation and not the other
    — the failure mode of every exercise where a vectorised version and a reference version coexist.
    The pose has to be on free floor: at a pose inside a wall every beam is at `range_min`, both
    implementations agree that there is nothing to say, and the comparison silently compares two
    zeros.  (The first version of this test did exactly that, and passed.)  They agree to 1e-6 rather
    than to machine precision because the vectorised version sums 30 beams with numpy's pairwise
    summation and the loop adds them in order, on log-weights of order 10: that is the floating-point
    floor of the comparison, not a modelling difference.
    """
    grid = GridMap(hall)
    pose = free_pose(hall, np.random.default_rng(5))
    scan = synth.scan_dict(pose, hall, sigma=0.0)
    f = MonteCarloLocaliser(grid, MclParams(particles=40, beam_stride=12), pose=pose,
                            sigma=(0.05, 0.05, 0.02), rng=np.random.default_rng(2))
    assert f.update(scan) > 1.0, "the reference path did not use the scan at all"
    by_loop = f.weights_with_a_for_loop(scan)
    assert np.max(np.abs((f.logw - f.logw.max()) - (by_loop - by_loop.max()))) < 1e-5


def test_a_scan_with_nothing_in_it_leaves_the_filter_alone(hall):
    """An empty scan increments a counter instead of multiplying the weights by a constant.

    Weighting an empty scan would leave the posterior unchanged in theory and, in code, would subtract
    a large negative number from every log-weight depending on how many beams were discarded — which
    is how a filter ends up with all weights equal after a drive through a featureless stretch.
    """
    grid = GridMap(hall)
    pose = free_pose(hall, np.random.default_rng(6))
    f = MonteCarloLocaliser(grid, MclParams(particles=100, beam_stride=3), pose=pose,
                            rng=np.random.default_rng(2))
    empty = {"t": 0.0, "angle_min": 0.0, "angle_increment": 2 * math.pi / 360, "range_min": 0.05,
             "range_max": 8.0, "ranges": [math.inf] * 360, "missing": 360}
    before = f.logw.copy()
    f.update(empty)
    assert np.array_equal(f.logw, before), "an empty scan changed the weights"
    assert f.degenerate == 1, "and it was not counted as degenerate"


def test_neff_is_the_number_from_the_slides(hall):
    """N_eff = 1 / Σ wᵢ² : computed by hand twice in the lecture, from this code.

    Weights 0.1, 0.3, 0.6 give 1 / 0.46 = 2.174 of 3 — the effective sample size of a very uneven
    cloud, which is what the grader's floor is checked against.
    """
    f = MonteCarloLocaliser(GridMap(hall), MclParams(particles=3), pose=(0, 0, 0),
                            rng=np.random.default_rng(1))
    f.w = np.array([0.1, 0.3, 0.6])
    assert f.neff() == pytest.approx(1.0 / 0.46, abs=1e-6)
    f.w = np.full(3, 1.0 / 3.0)
    assert f.neff() == pytest.approx(3.0), "an even cloud must count as three particles"
    #  What a node reports is `estimate()["neff"]`, i.e. the value measured at the last update — before
    #  any resampling, after which every weight is 1/N again and the number is N by construction.  A
    #  filter that samples N_eff from its own resampled output reports its own particle count, which is
    #  not a measurement of anything.
    pose = free_pose(hall, np.random.default_rng(8))
    g = MonteCarloLocaliser(GridMap(hall), MclParams(particles=200), pose=pose,
                            rng=np.random.default_rng(4))
    g.update(synth.scan_dict(pose, hall, sigma=0.02, rng=np.random.default_rng(5)))
    assert g.estimate()["neff"] == pytest.approx(g.last_neff)
    assert 1.0 <= g.last_neff <= 200.0


def test_systematic_resampling_loses_nothing_when_the_weights_are_even(hall):
    """Low-variance resampling keeps every particle once when all are equally good; a multinomial
    draw keeps about 63 % of them and calls that a sample."""
    f = MonteCarloLocaliser(GridMap(hall), MclParams(particles=500), pose=(6.0, 8.0, 0.0),
                            rng=np.random.default_rng(3))
    f.resample()
    assert len(np.unique(f.x[:, 0] + 1j * f.x[:, 1])) == 500


def test_the_mean_of_two_opposite_particles_is_not_the_average(hall):
    """+170° and −170° average to 0°, which is the wrong way round."""
    f = MonteCarloLocaliser(GridMap(hall), MclParams(particles=2), pose=(0, 0, 0),
                            rng=np.random.default_rng(1))
    f.x = np.array([[6.0, 8.0, math.radians(170.0)], [6.0, 8.0, math.radians(-170.0)]])
    assert abs(abs(f.estimate()["theta"]) - math.pi) < 0.05
