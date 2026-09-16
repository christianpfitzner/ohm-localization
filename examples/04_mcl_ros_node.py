#!/usr/bin/env python3
"""The smallest MCL node that speaks plain ROS 2: `/map` + `/odom` + `/scan` → `/kf/pose`.

    ros2 run ohm_localization map_server --world production &   # /map, latched
    python3 -m ohm_localization.lab sim --world production --robots alice --headless &
    python3 examples/04_mcl_ros_node.py --robot alice

rclpy, the standard message types and `ohm_localization.mcl` — no simulator imports, no custom
messages. The map arrives as a `nav_msgs/OccupancyGrid`, so this node is as at home with a map from a
mapper as with the one `map_server` publishes. A ROS `LaserScan` writes `range_max` where a beam found
nothing, so the beams are left at the default `drop_uninformative=True`, which throws exactly those
away; keeping them tells the filter there is a wall 8 m out in every direction. `ros2 topic echo /alice/kf/pose` watches it
work.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                            # noqa: E402
import rclpy                                                  # noqa: E402
from rclpy.executors import ExternalShutdownException          # noqa: E402
from rclpy.signals import SignalHandlerOptions                 # noqa: E402
from rclpy.node import Node                                   # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy      # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped       # noqa: E402
from nav_msgs.msg import Odometry, OccupancyGrid              # noqa: E402
from sensor_msgs.msg import LaserScan                         # noqa: E402

from ohm_localization.gridmap import GridMap, hall_from_occupancy_grid      # noqa: E402
from ohm_localization.mcl import MclParams, MonteCarloLocaliser             # noqa: E402

SIGMA_PRIOR = 0.5          # m: how well the start pose is known, ±1σ, from the wheels


class MclNode(Node):
    def __init__(self, robot: str, particles: int = 800, sigma_z: float = 0.15):
        super().__init__("example_mcl")
        self.robot, self.filter, self.grid = robot, None, None
        self.last_odom_t, self.odom = None, None
        self.params = MclParams(particles=particles, beam_stride=3, sigma_z=sigma_z)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(Odometry, f"/{robot}/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, f"/{robot}/scan", self.on_scan, 10)
        self.report = self.create_publisher(PoseWithCovarianceStamped, f"/{robot}/kf/pose", 10)
        self.get_logger().info(f"waiting for /map — {particles} particles, σ_z {sigma_z} m")

    def on_map(self, msg: OccupancyGrid) -> None:
        self.grid = GridMap(hall_from_occupancy_grid(msg, name="ros_map"))
        self.get_logger().info(f"map received: {self.grid}")

    def on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.odom = (pose.position.x, pose.position.y,
                     2.0 * np.arctan2(pose.orientation.z, pose.orientation.w))
        if self.filter is None and self.grid is not None:
            self.filter = MonteCarloLocaliser(self.grid, self.params, pose=self.odom,
                                              sigma=(SIGMA_PRIOR, SIGMA_PRIOR, 0.5))
            self.get_logger().info(f"localising: {self.grid}")
        if self.filter is not None:
            # The triple plus its own dt: a `nav_msgs/Odometry` carries the pose nested two deep and
            # has no `.t`, and a bare triple would fall back on the filter's default step.
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.last_odom_t is not None and t > self.last_odom_t:
                self.filter.predict_odometry(self.odom, dt=t - self.last_odom_t)
            self.last_odom_t = t

    def on_scan(self, msg: LaserScan) -> None:
        if self.filter is None:
            return
        self.filter.update({"ranges": np.asarray(msg.ranges), "angle_min": msg.angle_min,
                            "angle_increment": msg.angle_increment, "range_max": msg.range_max})
        e = self.filter.estimate()
        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.pose.pose.position.x, out.pose.pose.position.y = e["x"], e["y"]
        out.pose.pose.orientation.z = np.sin(e["theta"] / 2.0)
        out.pose.pose.orientation.w = np.cos(e["theta"] / 2.0)
        out.pose.covariance[0], out.pose.covariance[7], out.pose.covariance[35] = \
            e["sx"] ** 2, e["sy"] ** 2, e["sth"] ** 2
        self.report.publish(out)


def main() -> int:
    ap = argparse.ArgumentParser(description="MCL on /map, /odom and /scan: rclpy, no custom messages.")
    ap.add_argument("--robot", default="alice")
    ap.add_argument("--particles", type=int, default=800)
    ap.add_argument("--sigma-z", type=float, default=0.15, help="sensor model σ in m")
    args, ros = ap.parse_known_args()

    # ALL, so Ctrl-C and `kill` both end the spin loop instead of aborting inside the wait set.
    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)
    node = MclNode(args.robot, args.particles, args.sigma_z)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
