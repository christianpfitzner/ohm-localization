"""Publish the hall this exercise localises *in* as a `nav_msgs/msg/OccupancyGrid` on `/map`.

    ros2 run ohm_localization map_server --world production
    ros2 run ohm_localization map_server --world rooms --resolution 0.10 --frame map
    python3 -m ohm_localization.map_server_node --world production --dry-run     # no ROS needed

Why: the simulator draws the walls as markers on its own window and publishes no map topic, so a group
looking at RViz sees a robot, a LIDAR fan and no walls — and the first question when a localiser disagrees
with a drive is "what map is it matching against?". One process, one static message, and the picture appears.
The same grid is what `nav2`'s tools would consume, which is the bridge this package deliberately stops at:
the exercise is about writing the localiser, not about running someone else's.

Two things worth knowing before you read the code.

**The conversion is not here.** `gridmap.occupancy_grid()` produces the fields as a plain dict and
`gridmap.hall_from_occupancy_grid()` reads them back, so the round-trip — origin, row order, which value is
a wall — is tested on a machine with NumPy and no message packages (`test/test_occupancy_grid.py`). What is
here is the eleven lines that put that dict into a real message and onto a topic, plus a `--dry-run` that
prints the same numbers without ROS. `build_message()` takes the message classes as an argument for the same
reason: the test hands it stubs and checks the field-by-field assignment that otherwise only runs on a
sourced machine.

**The QoS is `transient_local`.** A static map published once and then forgotten is a map that RViz never
sees, because the viewer starts a second later than the publisher. Latching (durable + transient local, depth
1) is what `nav2_map_server` does for exactly this reason; a `reliable, depth 10` here produces an empty
window and an hour lost.
"""
from __future__ import annotations

import argparse
import sys
from types import SimpleNamespace


def build_message(fields: dict, modules) -> object:
    """The dict from `gridmap.occupancy_grid()` as a message, using the classes it is given.

    `modules` needs `OccupancyGrid`, `MapMetaData`, `Header`, `Vector3`, `Quaternion` and something with a
    `Stamp`-shaped `time` (`builtin_interfaces.msg.Time`). Passing them in — rather than importing them at
    the top of this file — is what lets `test/test_occupancy_grid.py` verify every field of the glue with
    stubs on a machine where `nav_msgs` is not installed, and lets this module be *imported* there.
    """
    info = fields["info"]
    origin = info["origin"]
    m = modules.OccupancyGrid()
    m.header = modules.Header()
    m.header.frame_id = str(fields["header"]["frame_id"])
    if fields["header"].get("stamp") is not None:
        m.header.stamp = fields["header"]["stamp"]
    m.info = modules.MapMetaData()
    m.info.map_load_time = None                       # not loaded from a file; built from the hall
    m.info.resolution = float(info["resolution"])
    m.info.layer = str(info.get("layer", "static"))
    m.info.width, m.info.height = int(info["width"]), int(info["height"])
    m.info.origin = modules.Pose()
    m.info.origin.position = modules.Vector3()
    m.info.origin.position.x = float(origin["position"]["x"])
    m.info.origin.position.y = float(origin["position"]["y"])
    m.info.origin.position.z = float(origin["position"]["z"])
    m.info.origin.orientation = modules.Quaternion()
    for key in ("x", "y", "z", "w"):
        setattr(m.info.origin.orientation, key, float(origin["orientation"][key]))
    m.data = list(fields["data"])                     # int8 in the dict, int8 in the message
    return m


def describe(fields: dict) -> str:
    """What `--dry-run` prints: the same numbers the message would carry, no ROS required."""
    info = fields["info"]
    data = fields["data"]
    walls = sum(1 for v in data if v >= 50)
    unknown = sum(1 for v in data if v < 0)
    return (f"map: {info['width']}x{info['height']} cells at {info['resolution']} m "
            f"= {info['width'] * info['resolution']:.2f}x{info['height'] * info['resolution']:.2f} m, "
            f"origin ({info['origin']['position']['x']:.2f}, {info['origin']['position']['y']:.2f}), "
            f"frame '{fields['header']['frame_id']}' · {walls} occupied, "
            f"{unknown} unknown, {len(data) - walls - unknown} free")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Publish a mecanum-lab hall as /map (nav_msgs/OccupancyGrid).")
    ap.add_argument("--world", default="production", help="hall name, as the simulator spells it")
    ap.add_argument("--resolution", type=float, default=0.25, help="grid step in m (default 0.25)")
    ap.add_argument("--frame", default="map", help="frame_id of the message")
    ap.add_argument("--topic", default="/map")
    ap.add_argument("--dry-run", action="store_true",
                    help="print what would be published and exit (works without ROS installed)")
    ap.add_argument("--ros-args", default=None, help=argparse.SUPPRESS)   # tolerated from a launch file
    a, unknown = ap.parse_known_args(list(sys.argv[1:] if argv is None else argv))
    if unknown:
        print(f"map_server: ignoring unrecognised arguments {' '.join(unknown)}", file=sys.stderr)

    from ohm_localization.gridmap import load_map, occupancy_grid
    grid = load_map(a.world, resolution=a.resolution)
    fields = occupancy_grid(grid, frame_id=a.frame)
    print(f"map_server: {grid.hall} -> {describe(fields)}", file=sys.stderr)
    if a.dry_run:
        return 0

    try:                                              # the message types, imported only when they are needed
        import rclpy
        from geometry_msgs.msg import Pose, Quaternion, Vector3
        from nav_msgs.msg import MapMetaData, OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Header
    except Exception as exc:                          # noqa: BLE001 - the message is the useful part
        print("map_server: this node needs ROS 2 (rclpy, nav_msgs, geometry_msgs, std_msgs) — the rest of\n"
              f"ohm_localization does not. ({type(exc).__name__}: {exc})\n"
              "Source a workspace that has them, or run with --dry-run to see the map itself.",
              file=sys.stderr)
        return 3

    # The classes `build_message` writes its fields on. A namespace rather than a class so that the test can
    # hand over stubs of exactly the same shape and neither side has to know which of the two it got.
    Types = SimpleNamespace(OccupancyGrid=OccupancyGrid, MapMetaData=MapMetaData, Header=Header,
                            Pose=Pose, Vector3=Vector3, Quaternion=Quaternion)
    rclpy.init()
    node = rclpy.create_node("map_server")
    try:
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub = node.create_publisher(OccupancyGrid, a.topic, qos)
        fields = occupancy_grid(grid, frame_id=a.frame)
        fields["header"] = dict(fields["header"], stamp=node.get_clock().now().to_msg())
        pub.publish(build_message(fields, Types))
        node.get_logger().info(f"published {a.topic}: {describe(fields)} "
                               "(latched: transient_local, depth 1)")
        print("map_server: the map is static — published once and latched, Ctrl-C to stop.", file=sys.stderr)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
