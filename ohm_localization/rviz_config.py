"""The RViz view of this exercise: a config in `launch/`, rendered per robot.

    ros2 launch ohm_localization mcl.launch.py rviz:=true
    rviz2 --display-config "$(python3 -c 'from ohm_localization import rviz_config; print(rviz_config.render("alice"))')"

`launch/mcl.rviz` is the file to edit, and it is a template: every topic of this project carries the robot's
name (`/alice/scan`, `/bob/particles`), RViz 2 has no way to substitute a name into a display, and a config
that spells `alice` shows nothing for the group that drives `muster`. Hence one `{robot}` placeholder and one
write per launch, which is also how the simulator renders its own view (`mecanum_lab/rviz_view.py`) — the two
files differ in their displays, not in the mechanism.

What is written goes to `/tmp/ohm_mcl_<robot>.rviz` and is a build product: it is regenerated every start, so
"Save Config" in the window is gone on the next launch and the file to change is the one under `launch/`.

Nothing here needs ROS, a display or a running simulator, which is what lets `test/test_rviz_config.py` check
the interesting properties of the file — that the covariance is on, that `/map` is asked for with
`Transient Local`, that every topic in it is a topic something in this repository actually publishes. Each of
those is a silent empty panel when wrong, and an empty panel in a two-hour lab session is diagnosed as a
broken filter.
"""
from __future__ import annotations

import os

from ohm_localization import paths

TEMPLATE = os.path.join("launch", "mcl.rviz")
PLACEHOLDER = "{robot}"
GROUND_PLACEHOLDER = "{ground}"
OUT = "/tmp/ohm_mcl_{robot}.rviz"


def config_file() -> str:
    """`launch/mcl.rviz` in whichever layout this run is in (paths.data_root(), asked not guessed)."""
    return os.path.join(paths.data_root(), TEMPLATE)


def text() -> str:
    """The template as it stands on disk — and a message that names the file when it is not there."""
    source = config_file()
    if not os.path.isfile(source):
        raise FileNotFoundError(f"{TEMPLATE} is missing from {paths.data_root()} — it is part of the "
                                f"package (setup.py installs launch/*.rviz), do not delete it")
    with open(source, encoding="utf-8") as handle:
        return handle.read()


def render(robot: str, ground: str = "map", out: str | None = None) -> str:
    """Fill the template for one robot and one hall frame, write it, return the path to hand to `rviz2`.

    The second placeholder is not decoration: the frame the hall's own coordinates go by depends on who owns
    the top of the TF tree (`view.py`), and it is `hall` in the default `tf:=localizer` run and `map` in a
    `tf:=sim` one. A config whose Fixed Frame does not name the frame the grid and the transform are stamped
    in is an empty window, and an empty window reads as a broken filter.

    A name with a `/` or a space in it is refused rather than cleaned: the simulator sanitises a robot name
    itself (`types.sanitize_name`), so the topics would come out as `/pfitz` while this file still said
    `/pfitz ner/scan`, and the picture would be an empty window pointing at nothing. Two places that spell a
    name differently must disagree loudly, and the launch argument is where the typo is.
    """
    body = text()
    if PLACEHOLDER not in body:
        raise ValueError(f"{TEMPLATE} has no {PLACEHOLDER} placeholder; every topic of a robot is its own")
    name = str(robot or "").strip()
    if not name or any(bad in name for bad in ("/", " ")):
        raise ValueError(f"{robot!r} cannot be a robot name in a topic: no '/', no space, not empty")
    frame = str(ground or "").strip()
    if not frame or any(bad in frame for bad in ("/", " ")):
        raise ValueError(f"{ground!r} cannot be a frame name: no '/', no space, not empty")
    target = out or OUT.format(robot=name)
    os.makedirs(os.path.dirname(os.path.abspath(target)), exist_ok=True)
    with open(target, "w", encoding="utf-8") as handle:
        handle.write(body.replace(PLACEHOLDER, name).replace(GROUND_PLACEHOLDER, frame))
    return target


def displays(body: str | None = None) -> list:
    """The displays of the config as `(class, enabled, name, topic)` rows — for the drift test.

    Parsed with YAML rather than read with a regex, because the file RViz is going to read is YAML and the
    question ("is the covariance on? is /map latched?") is about the parsed value, not about a line that
    happens to contain the word.
    """
    import yaml
    tree = yaml.safe_load(text() if body is None else body)
    rows = []
    for node in tree["Visualization Manager"]["Displays"]:
        topic = (node.get("Topic") or {}).get("Value", "")
        rows.append((node["Class"], bool(node.get("Enabled")), str(node.get("Name", "")), str(topic)))
    return rows
