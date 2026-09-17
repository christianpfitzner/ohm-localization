"""Which hall a node believes it is in — the one question whose wrong answer does not fail.

`ohm_localization/hall.py` exists because a node that asks `rob.world()` before `/sim/world` has arrived gets
the *local config default* — `maze` — while the run drives `production`. Nothing downstream complains: the
filter loads a 52 × 44 grid, the robot drives a 20 × 12 m hall, and the estimate stays plausible. It was seen
on a live `ros2 launch … controller:=student/mcl_template.py`, and the same rule is what makes the reference
solution's graded numbers reproducible over ROS.

These are the three branches of that rule, on a bus fake enough to run with numpy alone: the topic that
arrives late (the case that matters), the topic that never arrives (the task file knows), and a wait that is
interrupted by the graph dying (the end of a `ros2 launch`, not a crash).
"""
import os

from ohm_localization import hall


class FakeBus:
    """Answers `last("world")` with nothing until the run has spun a few times."""

    def __init__(self, payload="", arrives_after=0):
        self.payload, self.left = payload, arrives_after

    def last(self, kind, robot=None):
        return (self.payload if self.left <= 0 else "", 0.0)


class FakeRob:
    def __init__(self, bus, config_world="maze", dies_after=None):
        self.bus, self.spins = bus, 0
        self.config_world, self.dies_after = config_world, dies_after

    def running(self):
        return True

    def spin(self, dt):
        self.spins += 1
        if self.dies_after is not None and self.spins > self.dies_after:
            raise RuntimeError("publisher's context is invalid")
        if self.spins >= self.bus.left and self.bus.payload and self.bus.left > 0:
            self.bus.left = 0                            # the simulator's once-a-second republish

    def config(self, path, standard=None):
        return self.config_world if path == "world" else standard


def test_a_hall_that_arrives_late_is_still_the_hall_of_the_run():
    """The bug, exactly: the first answer available is the wrong hall, so waiting is the whole fix."""
    bus = FakeBus('{"name": "production", "size": [20, 12]}', arrives_after=4)
    rob = FakeRob(bus, config_world="maze")
    assert hall.hall_name(rob, "mcl_production", wait=5.0) == "production", \
        "asking once answers `maze` — the config default — and the drive is then localised against walls " \
        "that are not in this hall"
    assert rob.spins >= 4, "the wait is a wait: it has to spin the bus to get the topic at all"


def test_a_topic_that_never_arrives_falls_back_to_the_hall_the_task_was_calibrated_on():
    rob = FakeRob(FakeBus(""), config_world="maze")
    assert hall.hall_name(rob, "mcl_wide", wait=0.2) == "production", \
        "the task file says which hall its thresholds were measured in, which beats a local default"


def test_an_unknown_task_and_no_topic_leaves_the_guess_and_says_which_one_it_was():
    rob = FakeRob(FakeBus(""), config_world="maze")
    assert hall.hall_name(rob, "not_a_task", wait=0.2) == "maze", "last in the order is the config default"
    assert hall.task_world("not_a_task") == ""


def test_a_graph_that_dies_mid_wait_ends_the_wait_and_not_the_run():
    rob = FakeRob(FakeBus(""), config_world="maze", dies_after=2)
    assert hall.hall_name(rob, "mcl_dirty", wait=5.0) == "production", \
        "a `ros2 launch` shutdown inside the wait is the end of the waiting; the task file still knows"


def test_the_topic_carries_json_and_the_task_file_is_the_one_of_this_run():
    assert hall._world_payload(FakeRob(FakeBus("not json"))) == ""
    assert hall._world_payload(FakeRob(FakeBus('{"size": [1, 1]}'))) == ""
    assert hall._world_payload(FakeRob(FakeBus('{"name": "arena"}'))) == "arena"
    assert os.path.basename(hall.paths.task_file()) == "tasks_localization.json"
    assert hall.task_world("mcl_budget") == "production"
