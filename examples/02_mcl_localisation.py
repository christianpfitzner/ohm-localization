#!/usr/bin/env python3
"""Monte-Carlo localisation on a synthetic drive — numpy only, no ROS, no simulator.

    python3 examples/02_mcl_localisation.py

The odometry comes from `synth.fake_odometry`: wheels 3 % too large and a gyro 4 mrad/s off, which is
the drift the graded tasks are calibrated on. The filter is `ohm_localization.mcl.MonteCarloLocaliser`
— predict with the odometry, weight with the scan, resample when `N_eff` falls. It sees the map and the
LIDAR, never the truth.

Run it twice, once with `sigma_z` 0.15 and once with 0.6, and look at what the σ then claims.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                            # noqa: E402

from ohm_localization import synth                            # noqa: E402
from ohm_localization.gridmap import GridMap, corridor_text, parse_grid       # noqa: E402
from ohm_localization.mcl import MclParams, MonteCarloLocaliser       # noqa: E402

DT = 0.05                     # s per step: 20 scans per second, as in the graded tasks
SIGMA_Z = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15        # m, the sensor model's σ

grid = GridMap(parse_grid(corridor_text(cols=60, rows=12, pillars=2), cell=0.5, name="corridor"))
truth = synth.free_path(grid, vx=0.8, seconds=25.0, sine=0.35, seed=1)
odom = synth.fake_odometry(truth, DT, scale=1.03, gyro_bias=0.004, rng=np.random.default_rng(3))

filt = MonteCarloLocaliser(grid, MclParams(particles=400, beam_stride=6, sigma_z=SIGMA_Z),
                           rng=np.random.default_rng(7), pose=odom[0],
                           sigma=(1.0, 1.0, 0.5), prior="gaussian")

err = {"odom": [], "mcl": []}
nees = []
for i, pose in enumerate(truth):
    filt.predict_odometry(odom[i])
    filt.update(synth.scan_dict(pose, grid.hall, t=i * DT, beams=180, range_max=12.0,
                                sigma=0.05, rng=np.random.default_rng(i)))
    e = filt.estimate()
    dx_odom, dy_odom = odom[i, 0] - pose[0], odom[i, 1] - pose[1]
    dx, dy = e["x"] - pose[0], e["y"] - pose[1]
    err["odom"].append(math.hypot(dx_odom, dy_odom))
    err["mcl"].append(math.hypot(dx, dy))
    nees.append(0.5 * (dx ** 2 / e["sx"] ** 2 + dy ** 2 / e["sy"] ** 2))


def rmse(values):
    return float(np.sqrt(np.mean(np.square(values))))


print(f"{len(truth)} steps of {DT * 1000:.0f} ms in {grid}")
for name, values in err.items():
    print(f"  {name:4s} rmse {rmse(values) * 1000:6.0f} mm   worst {max(values) * 1000:6.0f} mm"
          f"   final {values[-1] * 1000:6.0f} mm")
e = filt.estimate()
print(f"  the filter is {rmse(err['odom']) / rmse(err['mcl']):.1f}× better than the wheels")
print(f"  sigma_z {SIGMA_Z} m → final σ ({e['sx']:.2f}, {e['sy']:.2f}) m, N_eff {e['neff']:.0f} of "
      f"{e['particles']}, {e['resamples']} resamples, cloud stands in {filt.diversity()} of 5 cm cells")
print(f"  NEES {np.mean(nees):.2f} — 1.0 means the σ the filter publishes matches the error it makes")
