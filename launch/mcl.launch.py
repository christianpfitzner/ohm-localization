"""Monte-Carlo localisation on the simulator, over ROS 2 — everything important is an argument here.

    ros2 launch ohm_localization mcl.launch.py                       # reference solution, production hall
    ros2 launch ohm_localization mcl.launch.py rviz:=false headless:=true
    ros2 launch ohm_localization mcl.launch.py task:=mcl_wide prior:=2.0
    ros2 launch ohm_localization mcl.launch.py controller:=student/mcl_template.py headless:=true
    ros2 launch ohm_localization mcl.launch.py task:=mcl_dirty lidar_sigma:=0.25 sigma_z:=0.5
    ros2 launch ohm_localization mcl.launch.py map:=false             # no /map topic at all

Four processes at most: the simulator (with our task file), the localiser node, optionally the `map_server`
that publishes the hall as an `OccupancyGrid`, and optionally `rviz2` with this package's own view.

**The window is part of the exercise.** `rviz:=auto` — the default — opens RViz 2 on `launch/mcl.rviz`: the
particle cloud (`/<robot>/particles`), the estimate with its covariance ellipse (`/<robot>/kf/pose`), the
scan, the odometry trail and the filter's own path (`/<robot>/kf/path`), and `map:=auto` starts the map
server beside it, because a localisation without walls is an arrow in the void. The two cases that leave the
window off and say so on the terminal: `headless:=true`, and a terminal with no `DISPLAY` — a viewer that
dies at startup teaches the wrong lesson, and `rviz:=false` never starts one at all.

**This is the ROS door, not the graded one.** The number for a sheet comes from

    ./tools/run_lab.sh grade --task mcl_production --controller solution/mcl_node.py --headless

which runs simulator and controller in one process on the in-process bus. `grade:=` here starts the grader
inside the simulator and your node in a *second* process, and a localisation grade is measured from the
`kf/pose` messages your node publishes over DDS; the same file that reaches 30/30 through the first door is
graded by this one at 0 points with `rate of kf/pose 0.0` in its report, which is the simulator's own
documented behaviour for its KF tasks (`mecanum-lab/launch/kf.launch.py`, `docs/CONTRACT.md` §9.2) and not
something this launch file can fix. Use `grade:=` to check that the wiring works over ROS, never to produce a
number. `docs/verification.md` records whether the effect was measured in this checkout or inherited from
that note.

What you can say, and what it changes:

  rviz:=auto        the viewer. auto opens it when there is a display to open it on and leaves it shut on a
                    headless run, telling you so. `true` asks for it by name and fails if it is not there,
                    `false` never
  map:=auto         the hall as an occupancy grid on /map, which is what the picture is drawn on top of.
                    auto starts it whenever the viewer does; `true`/`false` say so directly
  tf:=localizer     who tells TF where the robot is. `localizer` (default) starts the simulator in its `slam`
                    tree — where it publishes `odom -> base_link` and nothing above — and this node publishes
                    `<hall> -> <robot>/odom` from the estimate, so the laser, which has nothing else to go on,
                    sits on the walls its beams are matched against. `sim` puts the simulator back on that
                    edge as the identity, which is the tree the Kalman lab ships: everything above the
                    encoders drifts, up to 0.33 m, and closing that gap *is* that exercise. `truth` lets the
                    simulator close it with the answer instead — the tutor's view, and not localisation
  task:=<task>      the drive and the hall that goes with it: mcl_production, mcl_wide, mcl_budget,
                    mcl_dirty — empty runs the hall with no commanded drive

The picture is four things: the particle cloud (`/<robot>/particles`), the estimate and its covariance
(`/<robot>/kf/pose`), the filter's own trail (`/<robot>/kf/path`) and the wheel encoders' trail in the same
frame (`/<robot>/odom/path`) — the last two a few centimetres apart is the whole story of the run. The last
three the bus already carries; the cloud and the transform are added by `ohm_localization/view.py`, off the
same filter and beside the graded topic. `view.py` says why they only exist on this door.

Every sensor argument is the same setting you would give without ROS:

    ./lab sim --set lidar.sigma=0.25 --set odom.bias_omega=0.004

and an argument that stays empty is not passed at all, so the task's own test profile in
`config/tasks_localization.json` decides. An argument that is set always wins over the task profile — that
is what makes a single-variable sweep possible from the command line, which is what L4 asks for.

The filter's own parameters (`sigma_z`, `particles`, `beam_stride`, `prior`) go to the node as `OHM_MCL_*`
environment variables, the same names `tools/mcl_report.py` and the task file's `mcl` block use: one set of
names across the three doors the exercise can be run through.
"""
import os
import sys

from launch import LaunchDescription
import launch.actions as L
from launch.event_handlers import OnProcessExit

# The package's data root in either layout: in a checkout this is the repository, after a colcon build it is
# `<prefix>/share/ohm_localization`, and the three directories the launch file needs — `config/`,
# `solution/`, `student/` — are installed to sit next to `launch/` in both.
SHARE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRUE = ("true", "1", "yes", "on")

# What each value of `tf:=` means: the name the hall's frame goes by in this run, and what the simulator is
# told about it. Only the localiser's own value adds anything, because only it takes the edge away from the
# simulator (`tf_bcast.tree()` reads `slam` as "stop publishing map -> odom, a mapping node owns it"; the two
# other spellings leave the simulator where it started, with the frame named `map`).
TF_TREES = {"localizer": ("hall", ("--set", "tf.tree=slam")),
            "sim": ("map", ()),
            "truth": ("map", ("--set", "tf.map_to_odom=truth"))}

BASICS = [
    ("world", "production", "hall to drive in; the tasks of this exercise are calibrated on production"),
    ("robot", "alice", "your robot name (one person, one robot)"),
    ("task", "mcl_production", "task to run: mcl_production | mcl_wide | mcl_budget | mcl_dirty (empty = free driving)"),
    ("controller", "", "a controller *file* instead of the installed node, e.g. student/mcl_template.py "
                       "(empty = run the `mcl_node` executable of this package)"),
    ("grade", "", "task or group to grade inside the sim process (empty = do not grade) — see the note "
                  "in the docstring: this door is for checking the wiring, not for the number on a sheet"),
    ("seconds", "0", "end after N seconds of simulation time (0 = until the task is through or Ctrl-C)"),
    ("headless", "false", "without the Pygame window (sets SDL_VIDEODRIVER=dummy)"),
    ("truth", "true", "publish the exact pose on /<robot>/truth; the grader needs it"),
    ("drive", "true", "drive the task's commanded path with this package's drive_node — a `kf` task is "
                      "otherwise driven only by a grader, so without this the robot stands still "
                      "(empty or false = drive nothing by hand: the keyboard does)"),
    ("seed", "1", "noise seed: same seed, same measurement series"),
    ("log", "", "CSV measurement log, e.g. runs/mcl.csv — tools/mcl_report.py reads its own recording"),
    ("rviz", "auto", "show the cloud, the covariance, the scan, the odometry and the path in rviz2: "
                    "auto | true | false (auto = yes, unless this run is headless or has no DISPLAY)"),
    ("map", "auto", "also run this package's map_server, publishing /map as a nav_msgs/OccupancyGrid: "
                    "auto | true | false (auto = yes when the viewer runs, whose first panel is the hall)"),
    ("tf", "localizer", "who publishes the hall -> odom transform: localizer | sim | truth"),
    ("log_level", "info", "info | debug | warning"),
    ("use_sim_time", "true", "use simulation time (/clock) for timestamps"),
]

# launch argument -> `--set path=in.the.simulator.config`; empty = leave the task's own profile alone
SETTINGS = [
    ("lidar_rate", "lidar.rate", "LIDAR scans per second (the tasks use 20)"),
    ("lidar_sigma", "lidar.sigma", "1σ of one range reading in m — L4 sets this to 0.25"),
    ("lidar_beams", "lidar.beams", "beams per scan"),
    ("lidar_range_max", "lidar.range_max", "the value a missing echo carries in m"),
    ("odom_sigma_xy", "odom.sigma_xy", "noise of the published odometry pose in m"),
    ("odom_sigma_theta", "odom.sigma_theta", "noise of the published odometry heading in rad"),
    ("odom_bias_omega", "odom.bias_omega", "systematic yaw-rate error in rad/s — what makes odometry drift"),
    ("odom_scale_xy", "odom.geometry.scale_xy", "wheelbase scale error, e.g. 1.03 for 3% too long"),
    ("truth_rate", "truth.rate", "rate of the exact pose in Hz (the grader measures against this)"),
    ("sim_rate", "rate", "physics steps per second"),
    ("gui_rate", "gui_rate", "GUI frames per second"),
]

# launch argument -> OHM_MCL_* environment variable of the node (see ohm_localization/mcl_node.py)
MCL_ARGS = [
    ("particles", "OHM_MCL_PARTICLES", "number of particles (L3 asks for a quarter of this)"),
    ("beam_stride", "OHM_MCL_BEAM_STRIDE", "use every n-th beam per update"),
    ("sigma_z", "OHM_MCL_SIGMA_Z", "sensor-model σ in m — the one knob L4 is about"),
    ("prior", "OHM_MCL_PRIOR", "σ of the initial belief in m (L2 starts at 2.0)"),
    ("z_rand", "OHM_MCL_Z_RAND", "clutter floor of the likelihood, 0…1"),
]


def _path(given: str) -> str:
    """Relative to the package's data root, absolute stays absolute — a controller may live anywhere."""
    return given if os.path.isabs(given) else os.path.join(SHARE, given)


def setup(context, *args, **kwargs):
    """Two processes, plus the map server and the viewer when they were asked for."""
    def arg(name):
        return context.launch_configurations.get(name, "")

    headless = arg("headless").lower() in TRUE
    robot = arg("robot")
    # The top edge of the TF tree, decided before anything is started: `tf:=` names who tells TF where the
    # robot is, and that decides what the hall's frame is called, which the viewer, the map server and the
    # node all have to spell the same way. `localizer` (the default) is the AMCL convention — the simulator
    # keeps `odom -> base_link` and `mcl_node` publishes `<hall> -> <robot>/odom` from the estimate, so the
    # laser, which has nothing else, lands on the walls its beams are matched against instead of drifting
    # 0.33 m off them (ohm_localization/view.py, docs/verification.md §17).
    tree = (arg("tf") or "localizer").strip().lower()
    if tree not in TF_TREES:
        raise ValueError(f"tf:='{tree}' is not one of {', '.join(TF_TREES)}")
    ground = TF_TREES[tree][0]
    env = {"MECANUM_LOG": arg("log_level"),
           "MECANUM_USE_SIM_TIME": "1" if arg("use_sim_time").lower() in TRUE else "0",
           "PYTHONPATH": os.pathsep.join([SHARE, os.environ.get("PYTHONPATH", "")])}
    if headless:
        env["SDL_VIDEODRIVER"] = "dummy"
    for name, var, _text in MCL_ARGS:
        if arg(name):
            env[var] = arg(name)

    sim = [sys.executable, "-m", "ohm_localization.lab", "sim",
           "--world", arg("world"), "--robots", robot,
           "--seconds", arg("seconds"), "--seed", arg("seed")]
    if arg("task"):
        sim += ["--task", arg("task")]
    for name, config_path, _comment in SETTINGS:
        if arg(name):
            sim += ["--set", f"{config_path}={arg(name)}"]
    sim += list(TF_TREES[tree][1])        # the tree, which is not a sensor knob and so is not in SETTINGS
    if arg("truth").lower() in TRUE:
        sim.append("--truth")
    if arg("log"):
        sim += ["--log", _path(arg("log")), "--interval", "0.05"]
    if arg("grade"):
        sim += ["--grade", arg("grade"), "--robot", robot]
    if headless:
        sim.append("--headless")

    if arg("controller"):
        # The file door: the simulator imports the path, which is how ./tools/run_lab.sh grades too.
        node_cmd = [sys.executable, "-m", "ohm_localization.lab", "controller", "--robot", robot,
                    "--controller", _path(arg("controller"))]
        node_name = os.path.basename(arg("controller"))
    else:
        # The package door: `ros2 run ohm_localization mcl_node`, the same module's main().
        node_cmd = [sys.executable, "-m", "ohm_localization.mcl_node", "--robot", robot]
        node_name = "mcl_node"

    sim_proc = L.ExecuteProcess(cmd=sim, additional_env=env, output="screen", name="mcl_sim",
                                emulate_tty=True)
    # The frame the node should call the hall, and publish its transform into. Given to the node alone, and
    # left empty when the simulator kept the edge: two publishers on one child frame give it two parents, and
    # a tree with two parents is not a tree but an error message at 20 Hz.
    node_env = {**env, "OHM_MCL_MAP_FRAME": ground if tree == "localizer" else ""}
    parts = [sim_proc,
             L.ExecuteProcess(cmd=node_cmd, additional_env=node_env, output="screen",
                              name=f"node_{robot}", emulate_tty=True)]
    # The run is over when the simulator says so (`seconds:=` or the task's drive): without this the
    # node outlives it and `ros2 launch` sits there until Ctrl-C.
    parts.append(L.RegisterEventHandler(OnProcessExit(
        target_action=sim_proc, on_exit=[L.Shutdown(reason="simulator finished")])))
    def flag(name, default):
        """An on/off argument as `"auto" | "true" | "false"`, with a typo named where it was typed.

        Only the two viewer arguments go through here, and they are the two where a misspelling used to be a
        silent change of picture: `rviz_view.plan()` asks "is there an rviz2" *before* it looks at the value,
        so `rviz:=ture` on a machine that has RViz opened a window instead of saying that `ture` is not one
        of the three values the argument documents.
        """
        spelled = {"1": "true", "yes": "true", "on": "true",
                   "0": "false", "no": "false", "off": "false"}
        value = spelled.get((arg(name) or default).strip().lower(),
                            (arg(name) or default).strip().lower())
        if value not in ("auto", "true", "false"):
            raise ValueError(f"{name}:={arg(name)} is not one of auto, true, false")
        return value

    note = None
    # --- the viewer. The simulator answers "is there an rviz2 on this machine at all" (it is not part of a
    # ROS base install, and a launch that dies on it is a bad first hour); this package decides what the
    # window shows and refuses a value that is not one of the three it documents.
    want = flag("rviz", "auto")
    rviz_started = False
    if want != "false":
        try:
            from mecanum_lab import rviz_view
        except Exception as exc:                     # noqa: BLE001 - a viewer is no reason to stop a run
            rviz_view, start, note = None, False, f"rviz not started (mecanum_lab.rviz_view: {exc})"
        else:
            start, note = rviz_view.plan(want)
        if start and headless and want == "auto":
            start = False
            note = "rviz stays off: this run is headless (`rviz:=true` asks for the window anyway)"
        elif start and not os.environ.get("DISPLAY"):
            start = False
            note = ("no DISPLAY in this terminal, so the window would die at startup — run this where the "
                    "screen is, or start the launch under `xvfb-run`")
        if start and rviz_view is not None:
            if SHARE not in sys.path:
                sys.path.insert(0, SHARE)            # a launch from a checkout with nothing sourced
            from ohm_localization import rviz_config
            # The frame goes into the file rather than onto the command line: `rviz_view.command()` knows only
            # the config and the clock, and the simulator's own view does the same thing by editing the YAML.
            config = rviz_config.render(robot, ground)   # launch/mcl.rviz, this robot in this hall's frame
            parts.append(L.ExecuteProcess(cmd=rviz_view.command(config,
                                                                sim_time=arg("use_sim_time").lower() in TRUE),
                                          additional_env=env, output="screen", name="rviz"))
            rviz_started = True
            note = (f"rviz: {config} on frame '{ground}' ({tree} owns the hall -> odom edge) — particles, "
                    f"kf/pose covariance, scan, odometry, path. The file to edit is launch/mcl.rviz, not the "
                    f"one in /tmp")

    # The map before the viewer, so the latched /map is already on the bus when RViz subscribes; `auto` here
    # means "yes, if there is a window that shows it", which is what the map is for in this exercise.
    if flag("map", "auto") == "true" or (flag("map", "auto") == "auto" and rviz_started):
        parts.append(L.ExecuteProcess(
            cmd=[sys.executable, "-m", "ohm_localization.map_server_node", "--world", arg("world"),
                 "--frame", ground],      # the grid and the transform must name one frame, or the robot
            additional_env=env, output="screen", name="map_server"))   # floats beside its own hall

    # A `kind: "kf"` task is driven by the grader, so a run without `grade:=` needs its own driver or
    # the localiser spends the whole drive watching a robot that never left its spawn pose.
    if arg("drive").lower() in TRUE and not arg("grade"):
        parts.append(L.ExecuteProcess(
            cmd=[sys.executable, "-m", "ohm_localization.drive_node", "--robot", robot],
            additional_env=env, output="screen", name=f"drive_{robot}"))

    sensors = "  ".join(f"{name}={arg(name)}" for name, _, _ in SETTINGS if arg(name))
    knobs = "  ".join(f"{name}={arg(name)}" for name, _, _ in MCL_ARGS if arg(name))
    parts.insert(0, L.LogInfo(msg=f"[mcl] task: {arg('task') or '—'} · world: {arg('world')} · robot: {robot}"
                                 f" · node: {node_name} · tf: {tree} on '{ground}'"
                                 f" · sensors: {sensors or 'task profile'}"
                                 f" · filter: {knobs or 'task file / defaults'}"))
    if note:
        parts.append(L.LogInfo(msg=note))
    if arg("grade"):
        parts.append(L.LogInfo(msg="[mcl] grading over ROS: the report of THIS run is not the number for a "
                                   "sheet — grade with ./tools/run_lab.sh (see the launch file's docstring)"))
    return parts


def generate_launch_description():
    arguments = [L.DeclareLaunchArgument(name, default_value=default, description=text)
                 for name, default, text in BASICS]
    arguments += [L.DeclareLaunchArgument(name, default_value="", description=text)
                  for name, _, text in SETTINGS + MCL_ARGS]
    return LaunchDescription(arguments + [L.OpaqueFunction(function=setup)])
