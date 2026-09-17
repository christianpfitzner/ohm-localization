"""The RViz view: the config on disk, the renderer, and the two topics the window needs but no sensor sends.

These are the parts of a picture that cannot be seen in a picture. An RViz panel pointed at a topic nobody
publishes is an empty panel and a group that believes their filter is broken; a covariance display with the
covariance switched off shows an arrow where the exercise is about the ellipse; a `Map` display on default
QoS never receives the latched `/map` (map_server_node.py) and shows a hall that is not there. None of that
is visible from the launch's exit code, and all of it is visible here.

The second half is `ohm_localization/view.py`, the publisher of `/<robot>/particles` and `/<robot>/kf/path`
— the only two messages of this exercise that no sensor of the simulator produces. Two doors, tested
separately and on purpose: on the graded door the view must not exist at all (the in-process bus has no node
to publish from), and on the ROS door the messages must survive `rclpy.serialization`, which is the check that
caught a `Vector3` in a `Pose.position` two doors away (test_occupancy_grid.py).
"""
import ast
import os

import pytest

from ohm_localization import rviz_config, view

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(ROOT, "launch", "mcl.launch.py")

# Every topic in the config, and the file that puts it on the bus. A display for a topic that is in neither
# list is the empty panel this test exists to prevent.
PUBLISHERS = {
    "/{robot}/particles": "ohm_localization/view.py",
    "/{robot}/kf/path": "ohm_localization/view.py",
    "/{robot}/kf/pose": "mecanum_lab/robot_io.py (send_kf)",
    "/{robot}/scan": "mecanum_lab/ros_bridge.py (the simulated LIDAR)",
    "/{robot}/odom": "mecanum_lab/ros_bridge.py (the simulated odometry)",
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
    """Particles, the covariance, the scan, the odometry with its trail, the path.

    Checked one by one, with the display class rather than the name, because a renamed panel is fine and a
    panel that is switched off is not: the user asking for "the particles and the covariance" gets a window
    with two grey checkboxes and no explanation otherwise.
    """
    pytest.importorskip("yaml")
    shown = {cls: on for cls, on, _n, _t in rviz_config.displays()}
    for cls in ("rviz_default_plugins/PoseArray",            # the cloud
                "rviz_default_plugins/PoseWithCovariance",   # the estimate and its sigma
                "rviz_default_plugins/LaserScan",            # the measurement
                "rviz_default_plugins/Odometry",             # the wheel path
                "rviz_default_plugins/Path",                 # the filter's own trail
                "rviz_default_plugins/Map"):                 # and the hall they are all in
        assert shown.get(cls) is True, f"{cls} is missing or switched off"


def test_the_covariance_of_the_estimate_is_shown_and_the_odometry_trail_is_long():
    pytest.importorskip("yaml")
    import yaml
    tree = yaml.safe_load(rviz_config.text())
    for node in tree["Visualization Manager"]["Displays"]:
        if node["Class"] == "rviz_default_plugins/PoseWithCovariance":
            assert node["Covariance"]["Value"] is True, \
                "the ellipse is the point: kf/pose carries the sigma the grader checks as NEES, and an " \
                "arrow without it shows the answer instead of the belief"
        if node["Class"] == "rviz_default_plugins/Odometry":
            assert int(node["Keep"]) >= 100, \
                "the odometry display is the path of the wheel encoder in this view, and 20 poses is a dot"


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


def test_the_fixed_frame_is_the_root_of_the_tree_the_simulator_broadcasts():
    pytest.importorskip("yaml")
    import yaml
    tree = yaml.safe_load(rviz_config.text())
    assert tree["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map", \
        "tf_bcast's ground frame is `map` in the sim tree, and every other header of this graph is written " \
        "against it; a fixed frame that is not in the tree shows a hall with a robot somewhere else"


def test_the_rendered_file_is_the_config_rviz_gets_and_still_parses():
    pytest.importorskip("yaml")
    import yaml
    path = rviz_config.render("muster")
    with open(path, encoding="utf-8") as handle:
        body = handle.read()
    assert "{robot}" not in body, "an unsubstituted placeholder is a topic called /{robot}/scan"
    assert "/muster/scan" in body and "/alice" not in body
    assert yaml.safe_load(body), "what RViz reads must still be YAML after the substitution"
    assert [t for _c, _o, _n, t in rviz_config.displays(body) if t] == \
           [t.replace("{robot}", "muster") for _c, _o, _n, t in rviz_config.displays() if t]


@pytest.mark.parametrize("bad", ["", "two names", "a/b"])
def test_a_name_that_would_move_a_topic_somewhere_else_is_refused(bad):
    with pytest.raises(ValueError):
        rviz_config.render(bad)


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


def test_the_view_stays_off_on_a_bus_without_a_node():
    """The graded door: one process, the in-process bus, no rclpy anywhere.

    Not a test that `RosView` is empty — a test that `mission()`'s loop can call it a thousand times and
    nothing happens, which is the property that keeps the door producing the grade identical to the door
    the documentation measured.
    """
    from types import SimpleNamespace
    rob = SimpleNamespace(bus=object(), name="alice")     # a bus with no node: what the stub is
    assert view.node_of(rob) is None
    v = view.RosView(rob)
    assert v.on is False
    v.publish([[0.0, 0.0, 0.0]] * 5, {"x": 1.0, "y": 2.0, "theta": 0.5}, 12.0)
    v.reset()
    assert len(v.poses) == 0, "a view that is off must not be accumulating a path nobody will read"


def test_the_cloud_is_thinned_for_the_window_and_not_for_the_filter():
    assert view.cloud_stride(view.CLOUD_MAX) == 1
    assert view.cloud_stride(view.CLOUD_MAX + 1) == 2
    assert view.cloud_stride(1200) == 3                  # the task's N: every third particle
    assert view.cloud_stride(1) == 1
    assert 1200 // view.cloud_stride(1200) <= view.CLOUD_MAX


# ---------------------------------------------------------------------- the ROS door, on real messages

def _fake_ros_rob():
    """A robot whose bus has a node — enough of a graph for `RosView`, and no simulator involved."""
    from types import SimpleNamespace

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


def test_the_two_topics_are_named_for_this_robot_and_carry_real_messages():
    pytest.importorskip("rclpy", reason="this is the ROS door: it checks message fields, not our arithmetic")
    geometry = pytest.importorskip("geometry_msgs")
    pytest.importorskip("nav_msgs")
    from rclpy.serialization import serialize_message

    rob, node = _fake_ros_rob()
    v = view.RosView(rob)
    assert v.on is True, "a bus with a node is the ROS door, and the view has to be on there"
    assert sorted(node.topics) == [("Path", "/alice/kf/path"), ("PoseArray", "/alice/particles")]
    assert v.frame == "map", "the cloud lives in the hall, not in the robot"

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
    pytest.importorskip("rclpy")
    rob, node = _fake_ros_rob()
    v = view.RosView(rob)

    class Dead:
        def publish(self, _msg):
            raise RuntimeError("rcl_publish_serialized_message: context is not valid")

    v._cloud = Dead()                       # the publisher the view holds, not the entry in the bus's table
    v.publish([[0.0, 0.0, 0.0]], {"x": 0.0, "y": 0.0, "theta": 0.0}, 1.0)
    assert v.on is False
    v.publish([[0.0, 0.0, 0.0]], {"x": 0.0, "y": 0.0, "theta": 0.0}, 9.0)   # no second exception, no crash
