"""ICP as odometry: stitch the scans together and see what the drive costs.

Two of these tests are controls, and they are the reason the other two mean anything.

The first control integrates the **exact** per-step transforms of a drive — the ones taken from the simulator's
truth by the same `icp.relative()` the node uses — through the same accumulator. If the accumulator's order or
direction is wrong, that test fails with a metre-scale number while every ICP number in the file still looks
plausible; and the wrong direction was in fact shipped for an hour, during which "ICP odometry drifts 4.9 m"
was a finding about my matrix multiplication. A test that only exercises the code with the algorithm's own
output cannot tell an integrator bug from a bad algorithm, which is what a control is for.

The second control is point-to-point against point-to-line on the same pairs, where the only difference is the
objective. That is what turns "the drift is big" into "the drift is a bias, and it belongs to one of the two
objectives" — see `test_point_to_line_carries_a_rotation_bias_that_point_to_point_does_not`.
"""
import json
import math
import os

import numpy as np
import pytest

from ohm_localization import gridmap, icp, icp_odom_node, synth

HALL = "production"
DT = 0.05                     # s, the scan period of the graded tasks
SPANS = 20.0                  # s of the SPIDER drive — the shape of `mcl_production`'s first blocks
NOISE = 0.015                 # m, the simulator's own 1σ on one range reading


@pytest.fixture(scope="module")
def drive():
    """The graded drive shape, its exact path, drifting odometry, and one noisy synthetic scan per step."""
    hall = gridmap.load_hall(HALL)
    blocks = [{"vx": 0.30, "duration": 6.0}, {"vx": 0.0, "omega": 0.5, "duration": 3.2},
              {"vx": 0.30, "duration": 6.0}, {"vx": 0.0, "omega": -0.5, "duration": 3.2},
              {"vx": 0.30, "duration": 1.6}]
    path = synth.path_from_drive(blocks, start=(3.0, 3.0, 0.0), dt=DT)
    # The commanded drive is dead reckoning; keep it inside the hall by starting where it fits (the same
    # question tools/drive_check.py asks before 40 s of grading does).
    keep = path[(path[:, 0] > 1.0) & (path[:, 0] < 19.0) & (path[:, 1] > 1.0) & (path[:, 1] < 11.0)]
    path = keep[:int(SPANS / DT)]
    odom = synth.fake_odometry(path, DT, scale=1.03, rng=np.random.default_rng(11))
    rng = np.random.default_rng(7)
    scans = [synth.scan_dict(pose, hall, t=i * DT, sigma=NOISE, rng=rng) for i, pose in enumerate(path)]
    return dict(path=path, odom=odom, scans=scans)


def _rmse(poses, truth):
    d = np.hypot(np.asarray(poses)[:, 0] - np.asarray(truth)[:, 0],
                 np.asarray(poses)[:, 1] - np.asarray(truth)[:, 1])
    return float(np.sqrt((d ** 2).mean())), float(d.max())


def test_integrating_the_exact_deltas_reproduces_the_exact_path(drive):
    """The control. Wrong order or wrong direction here, and every number below is about the integrator."""
    path = drive["path"]
    f = icp_odom_node.IcpOdom()
    f.T = icp.pose_to_T(path[0])
    for i in range(1, len(path)):
        f.advance(icp.relative(path[i - 1], path[i]))         # exact, taken from the truth itself
    err = np.hypot(f.T[0, 2] - path[-1][0], f.T[1, 2] - path[-1][1])
    assert err < 1e-9, f"exact transforms integrated to {err:.3f} m from where they came from"
    # And the same drive in the wrong direction, to show the control is not vacuous: this is the bug the
    # comment in `advance()` describes, and it is 4.9 m of "the algorithm seems fine".
    g = icp_odom_node.IcpOdom()
    g.T = icp.pose_to_T(path[0])
    for i in range(1, len(path)):
        g.T = g.T @ icp.relative(path[i - 1], path[i])        # no inverse: points, not the robot
    assert _rmse([icp.triple_of(g.T)], [path[-1]])[0] > 1.0


def test_icp_odometry_drifts_where_the_map_based_filter_does_not(drive):
    """Stitching scans is bounded by nothing: the same drive costs metres here and centimetres in L1.

    Measured on 400 synthetic scans of `production` along the graded drive shape, σ_z = 50 mm, every 4th
    beam, starting from the exact first pose: [filled by the run below].
    """
    path, scans, odom = drive["path"], drive["scans"], drive["odom"]

    def run(mode):
        out = icp_odom_node.integrate(scans, odom=list(odom), mode=mode, stride=4, sigma_z=0.05)
        t = np.array([row[0] for row in out])
        poses = np.array([row[1] for row in out])
        idx = [int(np.argmin(np.abs(np.arange(len(path)) * DT - x))) for x in t]
        truth = path[idx]
        # Scan 0 defines the frame the stitch is expressed in, so the comparison starts there. An ICP
        # odometry has no absolute reference to be wrong about — that absence is what the test measures.
        return _rmse(poses - poses[0] + truth[0], truth), len(out), t

    line, n, t = run("line")
    point, _, _ = run("point")
    wheel = _rmse(odom, path)
    mcl = 0.015                                     # the same drive, map-based: L1's graded RMSE
    print(f"\nicp_odometry over {n} steps / {t[-1] - t[0]:.1f} s: point-to-line {line[0]:.3f} m rmse "
          f"(max {line[1]:.2f}), point-to-point {point[0]:.3f} m (max {point[1]:.2f}), "
          f"wheel odometry {wheel[0]:.3f} m (max {wheel[1]:.2f}), MCL graded {mcl*1000:.0f} mm")
    assert line[0] > 0.30, f"ICP odometry measured {line[0]:.3f} m; the drift this exercise teaches has gone"
    assert line[0] > wheel[0] > mcl, "the ordering of the three methods in docs/icp.md is no longer true"
    assert wheel[0] < 0.35, f"the fake odometry drifted {wheel[0]:.2f} m; the comparison has changed"


def test_point_to_line_carries_a_rotation_bias_that_point_to_point_does_not(drive):
    """The drift is bias, not noise — and it belongs to the point-to-line objective.

    Median pairwise error is 0.04° of heading, which over 400 steps would be a random walk of 0.8° if the
    errors were independent and zero-mean. They are not: point-to-line pulls the heading the same way on
    every step in this hall, and a bias added 400 times is not a random walk any more.
    """
    path, scans, odom = drive["path"], drive["scans"], drive["odom"]

    def signed(mode):
        out = []
        for i in range(1, len(scans)):
            res = icp.register_scans(scans[i - 1], scans[i], T_guess=icp.relative(odom[i - 1], odom[i]),
                                      mode=mode, stride=4, sigma_z=0.05)
            est = icp.triple_of(res.T)[2]
            true = icp.triple_of(icp.relative(path[i - 1], path[i]))[2]
            out.append((est - true + np.pi) % (2 * np.pi) - np.pi)
        return np.degrees(np.array(out))

    line, point = signed("line"), signed("point")
    assert line.mean() < -0.02, f"mean signed rotation error {line.mean():+.4f}°/step — the bias disappeared"
    assert abs(point.mean()) < 0.02, f"point-to-point mean {point.mean():+.4f}°/step should sit at zero"
    n = len(line)
    assert abs(line.sum()) > 3 * math.sqrt(n) * line.std() / math.sqrt(n)      # sum ≫ σ of the mean's √N
    print(f"\nbias: line {line.mean():+.4f}°/step (sum {line.sum():+.1f}° over {n} pairs), "
          f"point {point.mean():+.4f}°/step (sum {point.sum():+.1f}°)")


def test_the_reported_uncertainty_widens_with_the_number_of_steps(drive):
    """A pairwise σ published as a pose σ is a claim about the wrong horizon."""
    path, scans, odom = drive["path"], drive["scans"], drive["odom"]
    f = icp_odom_node.IcpOdom(mode="line", stride=4, sigma_z=0.05)
    singles, wide = None, None
    for i, scan in enumerate(scans[:120]):
        f.step(scan, odom[i])
        if f.steps:
            one = np.array(f.steps[-1].sigmas)
            claim = np.array(f.sigmas())
            ratio = claim / one
            if f.pairs == 1:
                singles = ratio
            wide = ratio
    assert np.all(wide > 1.0), "the pose σ did not widen with the number of steps integrated"
    assert abs(wide[0] / math.sqrt(max(f.pairs, 1)) - 1.0) < 0.02
    assert np.all(singles == 1.0), "the first step's σ should be its own, √1 = 1"
