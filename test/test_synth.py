"""The offline sensor, checked against the real one."""
import math

import numpy as np
import pytest

from conftest import MECANUM_LAB, needs_sim, free_pose           # noqa: F401
from ohm_localization import synth
from ohm_localization.gridmap import load_hall

WORLDS = ("rooms", "arena", "maze", "track", "production", "open")


@needs_sim
@pytest.mark.parametrize("world", WORLDS)
def test_the_synthetic_lidar_is_the_simulators_lidar(world):
    """Beam for beam, `synth.cast()` against `mecanum_lab.sensors.Lidar.scan()`.

    Both with the noise switched off, on twelve random free poses in every hall the simulator ships.
    This is the test that lets every offline number in `docs/verification.md` mean something: without
    it the synthetic drive is a story about a robot that does not exist.
    """
    from mecanum_lab import sensors, types, worlds
    sim_world = worlds.load_world(world, cfg={"worlds": _world_cfg()})
    lidar = sensors.Lidar(sim_world, sensors.Noise(seed=1),
                          {"beams": 360, "range_max": 8.0, "sigma": 0.0, "range_min": 0.05})
    hall, rng = load_hall(world), np.random.default_rng(2024)
    worst, disagree_inf = 0.0, 0
    for _ in range(12):
        pose = free_pose(hall, rng)
        assert pose is not None, f"{world}: no free pose found"
        ref = lidar.scan(types.Pose(*pose)).ranges
        mine = synth.cast(pose, hall, beams=360, range_max=8.0, range_min=0.05)
        assert len(ref) == len(mine)
        disagree_inf += sum(1 for a, b in zip(ref, mine) if math.isinf(a) != math.isinf(b))
        worst = max(worst, max((abs(a - b) for a, b in zip(ref, mine)
                                if not (math.isinf(a) or math.isinf(b))), default=0.0))
    assert disagree_inf == 0, f"{world}: the two disagree about which beams came back at all"
    assert worst < 1e-9, f"{world}: worst beam difference {worst:.3e} m"


def _world_cfg():
    """The simulator's own `worlds` config section — that is where the maze's cell size lives."""
    from mecanum_lab import types
    return types.load_config().get("worlds", {})


def test_odometry_drifts_because_a_wheel_radius_is_wrong(drive):
    """Scale 1.04 means the wheels are half a centimetre too large, and 20 s of driving proves it."""
    true, odom = drive
    end_error = math.hypot(true[-1, 0] - odom[-1, 0], true[-1, 1] - odom[-1, 1])
    length = float(np.sum(np.hypot(np.diff(true[:, 0]), np.diff(true[:, 1]))))
    assert end_error > 0.3, f"a 4 % wheel error over {length:.1f} m drifted only {end_error:.2f} m"
    clean = synth.fake_odometry(true, dt=0.05, scale=1.0, gyro_bias=0.0,
                                rng=np.random.default_rng(5), sigma_xy=0.0, sigma_theta=0.0)
    assert math.hypot(true[-1, 0] - clean[-1, 0], true[-1, 1] - clean[-1, 1]) < 0.05


def test_a_beam_that_saw_nothing_stays_a_beam_that_saw_nothing(hall):
    """Adding noise to `inf` must not turn it into a measurement of a wall."""
    scan = synth.scan_dict(free_pose(hall, np.random.default_rng(4)), hall, sigma=0.25,
                           rng=np.random.default_rng(6))
    assert any(math.isinf(r) for r in scan["ranges"])
    assert scan["missing"] == sum(1 for r in scan["ranges"] if math.isinf(r))
