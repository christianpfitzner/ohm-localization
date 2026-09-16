"""Localisation exercises on the mecanum-lab simulator: a particle filter and ICP.

Two algorithms, one robot, one topic set:

*   `mcl.py` — Monte Carlo Localiser: 1000 particles, likelihood-field weighting, systematic
    resampling.  Chapter "Probabilistic Localization and SLAM", section "Monte Carlo Localization".
*   `icp.py` — Iterative Closest Point, point-to-point and point-to-line, as a laser odometry.
    Chapter "Scan Matching and the ICP Algorithm".

Everything here is NumPy and the standard library only, and nothing here imports `rclpy`: the
algorithm files run in a plain `python3` with no ROS and no simulator running, which is what makes
them testable on a laptop in a train.  The ROS/simulator glue lives in `solution/` and `tools/`, in
the one place it can be switched off.

The map comes from the simulator's own world parser (`gridmap.py`), never from a second copy of a
hall — a map that is a metre off does not fail loudly, it just makes every filter look bad.
"""

__version__ = "0.1.0"
