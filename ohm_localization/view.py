"""What the localiser is holding, on the ROS graph: the particle cloud and the path it drove.

    ros2 launch ohm_localization mcl.launch.py rviz:=true

Two topics, both beside the graded one and never on the way to it:

| topic | type | what it is |
|---|---|---|
| `/<robot>/particles` | `geometry_msgs/PoseArray` | the cloud — every particle as an arrow |
| `/<robot>/kf/path`   | `nav_msgs/Path`         | the estimate's own trail, in the hall's frame |

RViz can show everything the bus already carries: the scan, the odometry with its trail, the estimate with
its covariance (`launch/mcl.rviz` shows all four). What it cannot show is the particle cloud, because the
cloud never leaves `MonteCarloLocaliser` — `kf/pose` is the answer, and an arrow on the answer is a picture of
the result rather than of the method. The whole point of L2 and L3 is to watch a 2 m belief collapse onto
one wall, and that is a shape that exists only inside the filter. Hence these two topics.

**They exist only on the ROS door.** The graded run and `./lab run` share one process with the simulator on
the in-process bus, which has no node to publish from and no viewer to publish to. `RosView` asks the bus for
its node and, finding none, stays off — the same object with the same methods, so `mission()` has one code
path. The alternative, `if ros:` sprinkled through the loop, is how the door that produces the grade ends up
running different code from the one the documentation measured.

Nothing here is on the way to a number either: `kf/pose` is what the grader measures, these two are next to
it, and a publish that fails — a `ros2 launch` shutdown landing between the loop and the topic — closes the
view instead of ending the run.
"""
from __future__ import annotations

from collections import deque
import math
import sys

VIEW_DT = 0.1          # s between two frames: 10 Hz of picture next to the 50 Hz of kf/pose
CLOUD_MAX = 400        # arrows per frame, see cloud_stride()
PATH_MAX = 2000        # poses on the path before the oldest drops off (200 s at VIEW_DT)


def cloud_stride(n: int) -> int:
    """Take every n-th particle so that one frame stays under CLOUD_MAX arrows.

    1200 arrows is 1200 scene nodes rebuilt ten times a second by a laptop that is also running the filter,
    and RViz answers by dropping frames of the scan. Every third particle costs a third of that and shows the
    same thing — the spread, the multi-modality, the collapse — and the stride is uniform rather than "the
    first 400", because the order of the array is a detail of the last resample.
    """
    return max(1, -(-int(n) // CLOUD_MAX))


def node_of(rob):
    """The live rclpy node behind a robot's bus, or None on the in-process one.

    Deliberately not `rclpy.ok()`: the graded door never initialises rclpy at all, so there the question is
    not "is ROS healthy" but "does this bus have a node" — and asking rclpy answers a different question and
    is one of the ways this exercise has produced a green run on one door and a crash on another.
    """
    return getattr(getattr(rob, "bus", None), "node", None)


def _pose(M, x: float, y: float, yaw: float):
    """A planar pose as a `geometry_msgs/Pose`.

    The half-angle quaternion is the one the simulator uses for every pose of this graph
    (`ros_bridge.yaw_to_quat`), written out here rather than imported: this module imports no ROS and no
    simulator at module level, and four lines of trigonometry is a cheaper price than a module that cannot be
    imported on a laptop with no ROS.
    """
    p = M["Pose"]()
    p.position.x, p.position.y = float(x), float(y)
    p.orientation.z, p.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return p


class RosView:
    """The cloud and the path on two topics — or nowhere, with the same methods.

    `publish()` does nothing while the view is off, throttles both topics to `VIEW_DT` while it is on, and
    switches itself off on a publish that raises. A view is not a reason to stop localising, and it is
    certainly not a reason for a graded run to come back as `failed:RCLError`.
    """

    def __init__(self, rob, dt: float = VIEW_DT):
        self.dt, self.last, self.poses = float(dt), -1e9, deque(maxlen=PATH_MAX)
        self.node, self.on, self.frame = node_of(rob), False, "map"
        if self.node is None:
            return                                    # in-process bus: no node, no viewer, and no complaint
        try:
            from geometry_msgs.msg import PoseArray, PoseStamped
            from mecanum_lab import ros_bridge, tf_bcast
            from nav_msgs.msg import Path
            self.M = ros_bridge.load_msgs()
            if self.M is None:                        # rclpy half-there: the classes are the other half
                raise ImportError("the ROS message classes are not importable")
            self.frame = tf_bcast.frame_for("kf", rob.name, rob.sensor_profile())
            self._PoseArray, self._Path, self._PoseStamped = PoseArray, Path, PoseStamped
            self._cloud = self.node.create_publisher(PoseArray, f"/{rob.name}/particles", 10)
            self._path = self.node.create_publisher(Path, f"/{rob.name}/kf/path", 10)
            self.on = True
        except Exception as exc:                      # noqa: BLE001 - a view nobody can build is not a crash
            print(f"view: /particles and /kf/path stay off ({type(exc).__name__}: {exc})", file=sys.stderr)

    def reset(self) -> None:
        """Forget the trail: the path of a filter that was restarted is two drives drawn as one line."""
        self.poses.clear()

    def header(self, t: float):
        """The measurement's stamp and the hall's frame — never the wall clock.

        The scan and the odometry this loop reads are stamped in simulator seconds, and RViz is started with
        `use_sim_time` to match the clock the TF tree is broadcast in (`rviz_view.command`), so a wall-clock
        stamp here would put the cloud 1.7 billion seconds in the past of the viewer's own "now" and the panel
        would stay empty while the data flows. `kf/pose` is the odd one out — `robot_io.send_kf` stamps it 0.0
        — which is the other way to get this wrong, and not this node's field to fix.
        """
        head = self.M["Header"](frame_id=self.frame)
        head.stamp = self.M["Time"](sec=int(t), nanosec=int((t % 1.0) * 1e9))
        return head

    def publish(self, poses, estimate: dict, t: float) -> None:
        """One frame of both topics: `poses` the (n,3) cloud, `estimate` the filter's `estimate()`."""
        if not self.on or t - self.last < self.dt:
            return
        self.last = t
        try:
            self._publish(poses, estimate, t)
        except Exception as exc:                      # noqa: BLE001 - see the class docstring
            self.on = False
            print(f"view: stopped ({type(exc).__name__}: {exc})", file=sys.stderr)

    def _publish(self, poses, estimate: dict, t: float) -> None:
        cloud = self._PoseArray()
        cloud.header = self.header(t)
        stride = cloud_stride(len(poses))
        cloud.poses = [_pose(self.M, p[0], p[1], p[2]) for p in poses[::stride]]
        self._cloud.publish(cloud)

        track = self._PoseStamped()
        track.header = self.header(t)
        track.pose = _pose(self.M, estimate["x"], estimate["y"], estimate["theta"])
        self.poses.append(track)
        line = self._Path()
        line.header = self.header(t)
        line.poses = list(self.poses)                 # the whole trail, every frame: a Path is not a stream
        self._path.publish(line)
