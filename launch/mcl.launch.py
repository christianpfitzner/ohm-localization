"""Monte-Carlo localisation on the simulator, over ROS 2 — everything important is an argument here.

    ros2 launch ohm_localization mcl.launch.py                       # reference solution, production hall
    ros2 launch ohm_localization mcl.launch.py task:=mcl_wide prior:=2.0
    ros2 launch ohm_localization mcl.launch.py controller:=student/mcl_template.py headless:=true
    ros2 launch ohm_localization mcl.launch.py task:=mcl_dirty lidar_sigma:=0.25 sigma_z:=0.5 rviz:=true
    ros2 launch ohm_localization mcl.launch.py map:=true             # /map as nav_msgs/OccupancyGrid

Three processes at most: the simulator (with our task file), the localiser node, optionally `rviz2` and
optionally the `map_server` that publishes the hall as an `OccupancyGrid` for RViz and for `nav2` tools.

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
    ("rviz", "false", "start rviz2 on this robot's topics: auto | true | false"),
    ("map", "false", "also run this package's map_server, publishing /map as a nav_msgs/OccupancyGrid"),
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
    """Two processes, plus the map server and RViz when they were asked for."""
    def arg(name):
        return context.launch_configurations.get(name, "")

    headless = arg("headless").lower() in TRUE
    robot = arg("robot")
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
    parts = [sim_proc,
             L.ExecuteProcess(cmd=node_cmd, additional_env=env, output="screen",
                              name=f"node_{robot}", emulate_tty=True)]
    # The run is over when the simulator says so (`seconds:=` or the task's drive): without this the
    # node outlives it and `ros2 launch` sits there until Ctrl-C.
    parts.append(L.RegisterEventHandler(OnProcessExit(
        target_action=sim_proc, on_exit=[L.Shutdown(reason="simulator finished")])))
    if arg("map").lower() in TRUE:
        parts.append(L.ExecuteProcess(
            cmd=[sys.executable, "-m", "ohm_localization.map_server_node", "--world", arg("world")],
            additional_env=env, output="screen", name="map_server"))

    # A `kind: "kf"` task is driven by the grader, so a run without `grade:=` needs its own driver or
    # the localiser spends the whole drive watching a robot that never left its spawn pose.
    if arg("drive").lower() in TRUE and not arg("grade"):
        parts.append(L.ExecuteProcess(
            cmd=[sys.executable, "-m", "ohm_localization.drive_node", "--robot", robot],
            additional_env=env, output="screen", name=f"drive_{robot}"))

    note = None
    if arg("rviz").lower() in TRUE or arg("rviz").lower() == "auto":
        try:
            from mecanum_lab import rviz_view                    # same viewer config generator as the sim
            start, note = rviz_view.plan(arg("rviz"))
            if start:
                viewer = rviz_view.command(rviz_view.render_config(SHARE, robot),
                                           sim_time=arg("use_sim_time").lower() in TRUE)
                parts.append(L.ExecuteProcess(cmd=viewer, additional_env=env, output="screen", name="rviz"))
        except Exception as exc:                                 # noqa: BLE001 - a viewer is not a reason to stop
            note = f"rviz not started (mecanum_lab.rviz_view unavailable: {exc})"

    sensors = "  ".join(f"{name}={arg(name)}" for name, _, _ in SETTINGS if arg(name))
    knobs = "  ".join(f"{name}={arg(name)}" for name, _, _ in MCL_ARGS if arg(name))
    parts.insert(0, L.LogInfo(msg=f"[mcl] task: {arg('task') or '—'} · world: {arg('world')} · robot: {robot}"
                                 f" · node: {node_name} · sensors: {sensors or 'task profile'}"
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
