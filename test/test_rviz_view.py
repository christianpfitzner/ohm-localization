"""The RViz view: the config on disk, the renderer, and what the window needs that no sensor sends.

These are the parts of a picture that cannot be seen in a picture. An RViz panel pointed at a topic nobody
publishes is an empty panel and a group that believes their filter is broken; a covariance display with the
covariance switched off shows an arrow where the exercise is about the ellipse; a `Map` display on default
QoS never receives the latched `/map` (map_server_node.py) and shows a hall that is not there. None of that
is visible from the launch's exit code, and all of it is visible here.

The second half is `ohm_localization/view.py`, which puts `/<robot>/particles`, both trails and the
`<hall> -> <robot>/odom` transform on the graph — the messages of this exercise that no sensor of the
simulator produces. Two doors, tested
separately and on purpose: on the graded door the view must not exist at all (the in-process bus has no node
to publish from), and on the ROS door the messages must survive `rclpy.serialization`, which is the check that
caught a `Vector3` in a `Pose.position` two doors away (test_occupancy_grid.py).
"""
import ast
import math
import os
from types import SimpleNamespace

import pytest

from ohm_localization import rviz_config, view

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(ROOT, "launch", "mcl.launch.py")

# Every topic in the config, and the file that puts it on the bus. A display for a topic that is in neither
# list is the empty panel this test exists to prevent.
PUBLISHERS = {
    "/{robot}/particles": "ohm_localization/view.py",
    "/{robot}/kf/path": "ohm_localization/view.py",
    "/{robot}/odom/path": "ohm_localization/view.py",
    "/{robot}/kf/pose": "mecanum_lab/robot_io.py (send_kf)",
    "/{robot}/scan": "mecanum_lab/ros_bridge.py (the simulated LIDAR)",
    "/{robot}/truth": "mecanum_lab/ros_bridge.py (with truth:=true)",
    "/map": "ohm_localization/map_server_node.py",
}


def _source(path):
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def _by_class(rows):
    out = {}
    for cls, enabled, name, topic in rows:
        out.setdefault(cls, []).append((enabled, name, topic))
    return out


# ---------------------------------------------------------------------------------- the config on disk

def test_the_config_is_yaml_with_one_placeholder_and_its_displays():
    pytest.importorskip("yaml", reason="RViz reads YAML, and so does rviz_config.displays()")
    body = rviz_config.text()
    assert body.count(rviz_config.PLACEHOLDER) >= 7, "every robot-relative topic needs the placeholder"
    rows = rviz_config.displays()
    assert len(rows) >= 7, "the view is a handful of displays, not an empty window"
    for _cls, _on, name, topic in rows:
        assert "{robot}" not in topic or topic.startswith("/{robot}/"), \
            f"display {name!r} has a placeholder in the middle of a topic, which is not a topic"


def test_every_topic_in_the_view_is_a_topic_something_publishes():
    pytest.importorskip("yaml")
    for cls, _on, name, topic in rviz_config.displays():
        if not topic:
            continue                                   # Grid and TF have no topic: their input is /tf
        assert topic in PUBLISHERS, \
            f"{name!r} ({cls}) shows {topic}, which nothing in {PUBLISHERS.values()} publishes — an empty " \
            f"panel reads like a broken filter, which is the one diagnosis this file must not encourage"


def test_the_five_things_this_view_exists_for_are_on():
    """Particles, the covariance, the scan, the drift, the path.

    Checked one by one, with the display class rather than the name, because a renamed panel is fine and a
    panel that is switched off is not: the user asking for "the particles and the covariance" gets a window
    with two grey checkboxes and no explanation otherwise.
    """
    pytest.importorskip("yaml")
    shown = {cls: on for cls, on, _n, _t in rviz_config.displays()}
    for cls in ("rviz_default_plugins/PoseArray",            # the cloud
                "rviz_default_plugins/PoseWithCovariance",   # the estimate and its sigma
                "rviz_default_plugins/LaserScan",            # the measurement
                "rviz_default_plugins/Path",                 # both trails: the filter's and the wheels'
                "rviz_default_plugins/Map"):                 # and the hall they are all in
        assert shown.get(cls) is True, f"{cls} is missing or switched off"
    trails = {topic: on for _c, on, _n, topic in rviz_config.displays()
              if topic.endswith(("/kf/path", "/odom/path"))}
    assert trails == {"/{robot}/odom/path": True, "/{robot}/kf/path": True}, \
        "both trails, both on: the distance between the two lines is the drift, which is what the localiser " \
        "is measured against"


def test_the_odometry_of_the_view_is_a_trail_and_not_a_marker():
    """No `Odometry` display, now that the localiser owns the top edge of TF.

    An Odometry display transforms each wheel pose into the fixed frame, and correcting the odometry *is* the
    estimate, so that panel would draw the filter's answer where the picture of what the encoders got wrong
    belongs. The drift has to come from the node instead, which is what `/{robot}/odom/path` is.
    """
    pytest.importorskip("yaml")
    shown = {cls: on for cls, on, _n, _t in rviz_config.displays()}
    assert "rviz_default_plugins/Odometry" not in shown, \
        "the odometry marker is drawn through the correction and lands on the estimate — see view.py"


def test_the_covariance_of_the_estimate_is_shown():
    pytest.importorskip("yaml")
    import yaml
    tree = yaml.safe_load(rviz_config.text())
    for node in tree["Visualization Manager"]["Displays"]:
        if node["Class"] == "rviz_default_plugins/PoseWithCovariance":
            assert node["Covariance"]["Value"] is True, \
                "the ellipse is the point: kf/pose carries the sigma the grader checks as NEES, and an " \
                "arrow without it shows the answer instead of the belief"


def test_the_map_is_asked_for_with_the_qos_the_map_server_sends_it_with():
    """A `Map` display on volatile durability never sees a latched map — the panel stays empty forever."""
    pytest.importorskip("yaml")
    import yaml
    tree = yaml.safe_load(rviz_config.text())
    maps = [n for n in tree["Visualization Manager"]["Displays"] if n["Class"] == "rviz_default_plugins/Map"]
    assert maps, "the view has no map panel"
    for node in maps:
        assert node["Topic"]["Value"] == "/map"
        assert node["Topic"]["Durability Policy"] == "Transient Local", \
            "map_server latches (durable + transient local) exactly so that a viewer started one second " \
            "later still gets the hall; a volatile subscription misses it"


def test_the_fixed_frame_is_the_frame_the_transform_is_published_into():
    """`{ground}`, because which frame that is depends on `tf:=` and three processes must agree on it."""
    body = rviz_config.text()
    assert "Fixed Frame: {ground}" in body, \
        "the fixed frame is not a fixed name of this exercise: `hall` when the localiser owns the top edge of " \
        "the tree and `map` when the simulator keeps it — see the docstring of launch/mcl.launch.py"
    with open(rviz_config.render("alice", "hall"), encoding="utf-8") as handle:
        assert "Fixed Frame: hall" in handle.read()
    with open(rviz_config.render("alice"), encoding="utf-8") as handle:
        assert "Fixed Frame: map" in handle.read(), "a config rendered without a launch is the simulator's " \
        "tree, because that is what the simulator does when nobody tells it otherwise"


def test_the_rendered_file_is_the_config_rviz_gets_and_still_parses():
    pytest.importorskip("yaml")
    import yaml
    path = rviz_config.render("muster")
    with open(path, encoding="utf-8") as handle:
        body = handle.read()
    assert "{robot}" not in body, "an unsubstituted placeholder is a topic called /{robot}/scan"
    assert "{ground}" not in body, "and an unsubstituted frame is a window fixed on a frame called {ground}"
    assert "/muster/scan" in body and "/alice" not in body
    assert yaml.safe_load(body), "what RViz reads must still be YAML after the substitution"
    assert [t for _c, _o, _n, t in rviz_config.displays(body) if t] == \
           [t.replace("{robot}", "muster") for _c, _o, _n, t in rviz_config.displays() if t]


@pytest.mark.parametrize("bad", ["", "two names", "a/b"])
def test_a_name_that_would_move_a_topic_somewhere_else_is_refused(bad):
    with pytest.raises(ValueError):
        rviz_config.render(bad)
    with pytest.raises(ValueError):
        rviz_config.render("alice", bad)


# ---------------------------------------------------------------------------- the launch and the view

def test_the_launch_file_uses_this_config_and_offers_the_window_by_default():
    """The launch used to hand RViz the simulator's own config, which shows a robot and no particles."""
    src = _source(LAUNCH)
    assert "rviz_config.render" in src, "the launch must show launch/mcl.rviz, not a generated template"
    assert "rviz_view.render_config" not in src, "the simulator's template has no particle panel"
    declared = {}
    tree = ast.parse(src, filename=LAUNCH)
    for name in ("BASICS", "SETTINGS", "MCL_ARGS"):
        for node in tree.body:
            if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == name for t in node.targets):
                declared.update({row[0]: row[1] for row in ast.literal_eval(node.value)})
    assert declared.get("rviz") == "auto", "the view is part of the exercise, so the common command opens it"
    assert declared.get("map") == "auto", "and the hall it shows the estimate in"
    assert declared.get("tf") == "localizer", \
        "and the laser sits on the walls of that hall, which needs the localiser on the top edge of TF: left " \
        "to the simulator that edge is the identity, so the scan drifts off the map drawn next to it"


def test_the_view_stays_off_on_a_bus_without_a_node():
    """The graded door: one process, the in-process bus, no rclpy anywhere.

    Not a test that `RosView` is empty — a test that `mission()`'s loop can call it a thousand times and
    nothing happens, which is the property that keeps the door producing the grade identical to the door
    the documentation measured — including the transform, the newest of these calls and the one a student
    template inherits.
    """
    rob = SimpleNamespace(bus=object(), name="alice")    # a bus with no node: what the stub is
    assert view.node_of(rob) is None
    v = view.RosView(rob, map_frame="hall")     # even told the frame, there is nothing to publish into
    assert v.on is False
    v.publish([[0.0, 0.0, 0.0]] * 5, {"x": 1.0, "y": 2.0, "theta": 0.5}, 12.0)
    v.odometry_tf(SimpleNamespace(t=12.0, x=0.0, y=0.0, theta=0.0), {"x": 1.0, "y": 2.0, "theta": 0.5}, 12.0)
    v.reset()
    assert len(v.poses) == 0, "a view that is off must not be accumulating a path nobody will read"


def test_the_cloud_is_thinned_for_the_window_and_not_for_the_filter():
    assert view.cloud_stride(view.CLOUD_MAX) == 1
    assert view.cloud_stride(view.CLOUD_MAX + 1) == 2
    assert view.cloud_stride(1200) == 3                  # the task's N: every third particle
    assert view.cloud_stride(1) == 1
    assert 1200 // view.cloud_stride(1200) <= view.CLOUD_MAX


def _pose_as_both(x, y, theta, t=1.0):
    """A pose in whichever shape the callers hand one over. An OdomPose adds the stamp it arrived with, which
    is what makes it a different message from the identical pose beside it."""
    return SimpleNamespace(x=x, y=y, theta=theta, t=t)


def test_the_correction_is_the_estimate_and_not_the_odometry():
    """`hall->odom` composed with `odom->base` must land exactly on the estimate.

    That is the whole claim of the transform, checked as the composition a TF consumer performs rather than as
    the three lines of arithmetic: a sign error in the rotation would show up in the window as the scan sitting
    on the mirror image of the wall it measured, which is a hard thing to see in a picture and easy here.
    """
    for est, odo in [(_pose_as_both(3.0, 2.0, 0.7), _pose_as_both(1.0, 1.0, 0.2)),   # drift in x and heading
                     (_pose_as_both(0.0, 0.0, 0.0), _pose_as_both(0.0, 0.0, 0.0)),   # nothing drifted
                     (_pose_as_both(-4.5, 6.2, -2.4), _pose_as_both(2.0, -3.0, 1.1)),  # three quadrants
                     (_pose_as_both(1.0, 1.0, 3.0), _pose_as_both(1.0, 1.0, -3.0))]:   # the wrap at ±π
        cx, cy, dyaw = view.correction({"x": est.x, "y": est.y, "theta": est.theta}, odo)
        assert cx + math.cos(dyaw) * odo.x - math.sin(dyaw) * odo.y == pytest.approx(est.x, abs=1e-9)
        assert cy + math.sin(dyaw) * odo.x + math.cos(dyaw) * odo.y == pytest.approx(est.y, abs=1e-9), \
            "the laser's position in the hall is this sum: get the sign wrong and the fan lands behind the " \
            "wall the beams were measured against"
        back = math.atan2(math.sin(dyaw + odo.theta), math.cos(dyaw + odo.theta))
        assert back == pytest.approx(math.atan2(math.sin(est.theta), math.cos(est.theta)), abs=1e-9)


def test_the_correction_of_a_robot_that_has_not_drifted_is_nothing():
    cx, cy, dyaw = view.correction({"x": 2.0, "y": 1.5, "theta": 0.4}, _pose_as_both(2.0, 1.5, 0.4))
    assert (cx, cy, dyaw) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12), \
        "estimate equal to odometry must leave the tree as it was, or a good filter's picture moves"


# ---------------------------------------------------------------------- the ROS door, on real messages

def _fake_ros_rob():
    """A robot whose bus has a node — enough of a graph for `RosView`, and no simulator involved."""
    class FakePub:
        def __init__(self): self.sent = []
        def publish(self, msg): self.sent.append(msg)

    class FakeNode:
        def __init__(self): self.pubs, self.topics = {}, []
        def create_publisher(self, cls, topic, qos):
            self.topics.append((cls.__name__, topic))
            return self.pubs.setdefault(topic, FakePub())

    node = FakeNode()
    rob = SimpleNamespace(bus=SimpleNamespace(node=node), name="alice",
                          sensor_profile=lambda: {})
    return rob, node


def _view_on_the_ros_door(map_frame=""):
    """A `RosView` on a bus that has a node — or a skip saying why this environment has no ROS door.

    `importorskip("rclpy")` alone is not enough and used to be wrong: tools/check.sh exports
    `MECANUM_ROS=stub` so that the offline checks cannot reach DDS even on a machine that happens to have ROS
    sourced, and the simulator's message table (`ros_bridge.load_msgs`) then answers None *by design*. The
    view is off on such a run — that is the property the graded door depends on — so there is no message here
    to inspect, and the honest verdict is "skipped". The failure this replaces was real: on a professor's
    sourced shell `./tools/check.sh` went red while the code was fine.

    `map_frame=""` is passed explicitly rather than left to None, so that a stray OHM_MCL_MAP_FRAME in the
    environment of whoever runs the suite cannot turn the plain view into the TF-publishing one.
    """
    pytest.importorskip("rclpy", reason="the ROS door: this checks message fields, not our arithmetic")
    pytest.importorskip("geometry_msgs", reason="PoseArray and PoseStamped come from there")
    pytest.importorskip("nav_msgs", reason="and Path from here")
    pytest.importorskip("mecanum_lab.ros_bridge", reason="the message table is the simulator's to build")
    from mecanum_lab import ros_bridge
    if ros_bridge.load_msgs() is None:
        pytest.skip("no message table in this environment (MECANUM_ROS=stub does that on purpose), so the "
                    "view is off and there is nothing published to look at")
    rob, node = _fake_ros_rob()
    return view.RosView(rob, map_frame=map_frame), node

def test_the_two_topics_are_named_for_this_robot_and_carry_real_messages():
    geometry = pytest.importorskip("geometry_msgs")
    from rclpy.serialization import serialize_message

    v, node = _view_on_the_ros_door()
    assert v.on is True, "a bus with a node is the ROS door, and the view has to be on there"
    assert sorted(node.topics) == [("Path", "/alice/kf/path"), ("Path", "/alice/odom/path"),
                                   ("PoseArray", "/alice/particles")]
    assert v.frame == "map", "the cloud lives in the hall, not in the robot"
    assert v._tf is None, "without a frame from the launch the simulator owns the top edge, and must keep it"

    cloud = [[1.0, 2.0, 0.25], [1.1, 2.0, 0.24], [1.2, 2.1, 0.26]]
    v.publish(cloud, {"x": 1.1, "y": 2.05, "theta": 0.25}, 5.5)
    sent_cloud = node.pubs["/alice/particles"].sent[0]
    assert len(sent_cloud.poses) == 3 and sent_cloud.header.frame_id == "map"
    assert sent_cloud.header.stamp.sec == 5 and sent_cloud.header.stamp.nanosec == 500_000_000, \
        "the measurement's stamp to the millisecond, not the wall clock: RViz runs on use_sim_time like "\
        "every other topic of this graph"
    # A yaw of 0.25 as a quaternion about z, and the position where the particle is.
    assert sent_cloud.poses[0].position.x == pytest.approx(1.0)
    assert sent_cloud.poses[0].orientation.z == pytest.approx(0.12467, abs=1e-4)
    assert sent_cloud.poses[0].orientation.w == pytest.approx(0.99220, abs=1e-4)
    assert sent_cloud.poses[0].position.y != 0.0

    line = node.pubs["/alice/kf/path"].sent[0]
    assert line.poses[0].pose.position.x == pytest.approx(1.1)     # the estimate, not a particle

    # The check that a stub cannot do: the C serializer is the one that aborts on a field of the wrong
    # message type, and it aborts at publish time, in the middle of a run, naming neither line nor file.
    assert serialize_message(sent_cloud) and serialize_message(line)
    assert geometry.msg.PoseArray is type(sent_cloud)

    v.publish(cloud, {"x": 1.2, "y": 2.05, "theta": 0.25}, 5.55)   # inside VIEW_DT: nothing new
    assert len(node.pubs["/alice/particles"].sent) == 1
    v.publish(cloud, {"x": 1.3, "y": 2.05, "theta": 0.25}, 5.8)
    assert len(node.pubs["/alice/kf/path"].sent) == 2, "the trail grows one pose per frame"
    assert len(node.pubs["/alice/kf/path"].sent[1].poses) == 2
    v.reset()
    v.publish(cloud, {"x": 1.4, "y": 2.05, "theta": 0.25}, 6.4)
    assert len(node.pubs["/alice/kf/path"].sent[2].poses) == 1, \
        "a filter that was restarted drove a second drive; the old trail must not be drawn with the new one"


def test_a_publish_that_fails_closes_the_view_and_not_the_run():
    """A `ros2 launch` shutdown can land between the loop and the topic. The grade is not made of these two."""
    v, node = _view_on_the_ros_door()

    class Dead:
        def publish(self, _msg):
            raise RuntimeError("rcl_publish_serialized_message: context is not valid")

    v._cloud = Dead()                       # the publisher the view holds, not the entry in the bus's table
    v.publish([[0.0, 0.0, 0.0]], {"x": 0.0, "y": 0.0, "theta": 0.0}, 1.0)
    assert v.on is False
    v.publish([[0.0, 0.0, 0.0]], {"x": 0.0, "y": 0.0, "theta": 0.0}, 9.0)   # no second exception, no crash


def test_the_localiser_publishes_the_edge_the_laser_needs():
    """`<hall> -> <robot>/odom`, once per odometry message, into the frame the launch named.

    The one message of this file that is not a picture. A `LaserScan` is stamped in `<robot>/laser` and has
    nothing but TF to say where that is, so while the simulator holds the top edge of the tree as the identity
    the fan sits on the wheel encoders' opinion — measured 0.33 m off the walls at the end of a drive, next to
    a map those beams touch. On the filter's opinion it is 0.02 m off. Both numbers: docs/verification.md §17.
    """
    pytest.importorskip("tf2_msgs", reason="tf2_msgs is optional upstream; without it the view says so")
    from rclpy.serialization import serialize_message

    v, node = _view_on_the_ros_door("hall")
    assert v.on and v.frame == "hall", "the launch's frame is also the frame of the cloud and of both trails"
    assert ("TFMessage", "/tf") in node.topics

    est = {"x": 3.1, "y": 2.0, "theta": 0.7}
    v.odometry_tf(_pose_as_both(1.0, 1.0, 0.2), est, 4.0)
    tr = node.pubs["/tf"].sent[0].transforms[0]
    assert tr.header.frame_id == "hall" and tr.child_frame_id == "alice/odom"
    assert tr.header.stamp.sec == 4, "the measurement's stamp, so a consumer composes it with that odometry"
    cx, cy, dyaw = view.correction(est, _pose_as_both(1.0, 1.0, 0.2))
    assert (tr.transform.translation.x, tr.transform.translation.y) == pytest.approx((cx, cy))
    assert (tr.transform.rotation.z, tr.transform.rotation.w) == \
        pytest.approx((math.sin(dyaw / 2.0), math.cos(dyaw / 2.0)), abs=1e-9)
    assert serialize_message(node.pubs["/tf"].sent[0]), \
        "the C serializer has the last word on whether this is a TransformStamped, and it says it at publish"

    v.odometry_tf(_pose_as_both(1.0, 1.0, 0.2, t=1.0), est, 4.02)     # the loop again, no new measurement
    assert len(node.pubs["/tf"].sent) == 1, "one transform per odometry message, not per loop iteration"
    v.odometry_tf(_pose_as_both(1.05, 1.0, 0.21, t=1.02), est, 4.04)
    assert len(node.pubs["/tf"].sent) == 2, "the transform runs at the rate of the odometry, at 50 Hz" 


def test_the_wheels_trail_in_the_halls_frame_so_the_drift_stays_visible():
    """`/<robot>/odom/path`: the odometry's own numbers, drawn where the hall is.

    Stamped in the hall's frame and carrying the wheel pose is not a lie about a frame — it is the odometry's
    claim about the hall written down as a pose, which is the only way the drift survives in a window once the
    transform above cancels it out of everything derived from TF.
    """
    pytest.importorskip("tf2_msgs", reason="no TransformStamped class, no transform to check")

    v, node = _view_on_the_ros_door("hall")
    v.odometry_tf(_pose_as_both(1.0, 1.0, 0.2, t=4.0), {"x": 3.1, "y": 2.0, "theta": 0.7}, 4.0)
    v.odometry_tf(_pose_as_both(1.4, 1.1, 0.25, t=4.2), {"x": 3.2, "y": 2.0, "theta": 0.7}, 4.2)
    line = node.pubs["/alice/odom/path"].sent[-1]
    assert line.header.frame_id == "hall"
    assert [p.pose.position.x for p in line.poses] == pytest.approx([1.0, 1.4]), \
        "the wheel poses themselves, not the corrected ones: the gap to kf/path is the drift"
    assert not node.pubs["/alice/kf/path"].sent, "the estimate's trail is publish()'s to publish"


def test_a_transform_that_fails_closes_the_transform_and_not_the_trail():
    """The two halves of `odometry_tf` fail alone: a dead `/tf` must not stop the picture either."""
    pytest.importorskip("tf2_msgs", reason="the transform is what dies here")

    v, node = _view_on_the_ros_door("hall")

    class Dead:
        def publish(self, _msg):
            raise RuntimeError("rmw_publish: graph is invalid")

    v._tf = Dead()
    v.odometry_tf(_pose_as_both(1.0, 1.0, 0.2, t=4.0), {"x": 3.1, "y": 2.0, "theta": 0.7}, 4.0)
    assert v._tf is None and v.on is True
    v.odometry_tf(_pose_as_both(1.4, 1.1, 0.25, t=4.2), {"x": 3.2, "y": 2.0, "theta": 0.7}, 4.2)
    assert not node.pubs["/tf"].sent
    assert node.pubs["/alice/odom/path"].sent, "the drift keeps being drawn while the graph is going"
