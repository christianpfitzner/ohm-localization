# ohm_localization

Monte-Carlo localization (MCL) and ICP scan matching on the
[`mecanum-lab`](https://github.com/IntelligenteRobotik/mecanum-lab) simulator: the map layer, the
particle filter, four graded tasks (130 points) and the nodes that run them. ASY2 practicum
*Intelligente Robotik*.

The library is numpy. ROS 2 is only needed for `ros2 launch` and `ros2 run`; grading needs neither.

The repository directory is `ohm-localization`, the ROS package is `ohm_localization`. `ros2` always
takes the underscore.

## 1 · Check the machine

```bash
./install.sh --check
```

Checks python, numpy, pytest, pygame, the simulator checkout, the task file and every ROS 2 under
`/opt/ros`, and says which of them can run this package and what a partial one is missing. Exit code 0
means ready.

## 2 · Build (optional — only `ros2 launch` and `ros2 run` need it)

```bash
./install.sh --workspace
```

Builds the simulator, its message package, this package and `ohm_frontier` if it is checked out next
door. For this package alone:

```bash
./install.sh --build
```

Both use `--symlink-install` and leave `build/`, `install/` and `log/` in this directory. Then, in every
terminal:

```bash
source install/setup.bash
```

## 3 · Run

```bash
ros2 launch ohm_localization mcl.launch.py
```

The reference filter in the `production` hall, with RViz 2 open beside it: the particle cloud, the
estimate with its 1σ ellipse, the LIDAR scan, the wheel encoders' trail and the path the filter drove. The
commanded drive of task L1 is driven for you. Ctrl-C ends the run.

**The scan sits on the walls because the localiser says so.** A `LaserScan` carries no position, only the frame
it was measured in (`<robot>/laser`), so where it is drawn is whatever TF says — and TF gets its top edge
from whoever publishes it. By default this launch starts the simulator in the tree where it publishes
`odom -> base_link` and nothing above, and `mcl_node` publishes `<hall> -> <robot>/odom` from the estimate:
the LIDAR then stands where the filter believes it stands, a couple of centimetres off the truth instead of
the 0.2–0.4 m the wheel encoders are off by. `tf:=sim` hands that edge back to the simulator, which fills it
with the identity — the tree the Kalman lab ships, where everything above the encoders drifts and closing
that gap is the exercise. `tf:=truth` lets the simulator close it with the answer, which is a tutor's view and
not a localisation. Both trails are drawn in the hall's frame either way, so the distance between the blue
line (what the wheels thought) and the green one (what the filter thinks) stays visible in all three.

`rviz:=false` runs without the window, `map:=false` without the `/map` topic the hall is drawn from, and
`ros2 launch ohm_localization mcl.launch.py --show-args` lists every argument:

```bash
ros2 launch ohm_localization mcl.launch.py task:=mcl_wide prior:=2.0
```

```bash
ros2 launch ohm_localization mcl.launch.py controller:=student/mcl_template.py headless:=true
```

```bash
ros2 launch ohm_localization mcl.launch.py task:=mcl_dirty sigma_z:=0.5
```

Without a build, on the in-process bus:

```bash
./tools/run_lab.sh run --world production --task mcl_wide --controller student/mcl_template.py --truth
```

```bash
python3 -m ohm_localization.lab sim --world production --headless
```

## 4 · Grade — the door the marks come through

```bash
./tools/run_lab.sh grade --task mcl_production --controller student/mcl_template.py --headless
```

```bash
./tools/run_lab.sh grade --task mcl_production --controller solution/mcl_node.py --headless
```

`grade:=` on the launch file starts the grader in the simulator and your node in a second process; over
that door a localisation grade is measured from `kf/pose` messages crossing DDS and reports
`rate of kf/pose 0.0`. Use it to check wiring, never to produce a number.

## 5 · Examples

| command | what it shows | needs |
|---|---|---|
| `python3 examples/01_map_and_scan.py` | a hall written as text, its grid, one LIDAR scan as ASCII | numpy |
| `python3 examples/02_mcl_localisation.py` | the whole filter on a synthetic drive: 50 mm vs 230 mm of odometry | numpy |
| `python3 examples/02_mcl_localisation.py 0.6` | the same drive with σ_z 0.6 — what the σ then claims | numpy |
| `python3 examples/03_icp_scan_matching.py` | one ICP pair among posts and one in an empty corridor | numpy |
| `python3 examples/04_mcl_ros_node.py` | MCL as a plain rclpy node: `/map` + `/odom` + `/scan` → `/kf/pose` | ROS 2 |

[`examples/README.md`](examples/README.md) has each one with its output and the simulator commands it
needs.

## 6 · The four tasks

All four are graded in the `production` hall (20 × 12 m) against odometry that drifts 0.19–0.44 m over a
drive. `student/mcl_template.py` is the starting point: the node loop, the map, the prior, the motion
model, `N_eff` and the resampling are given, the sensor model is `TODO(L1)`.

| id | pts | the question | what it takes |
|---|---|---|---|
| `mcl_production` | 30 | write the sensor model; localise from a 0.5 m prior | rmse ≤ 0.05 m, ≥ 5× the odometry, NEES ∈ [0.05, 5] |
| `mcl_wide` | 35 | the same from a 2 m prior | rmse ≤ 0.06 m, ≥ 3×, NEES ∈ [0.05, 8] |
| `mcl_budget` | 35 | same accuracy, a quarter of the particles | rmse ≤ 0.07 m, ≥ 3×, NEES ∈ [0.05, 12] |
| `mcl_dirty` | 30 | the window is dirty (σ_lidar = 0.25 m): match the model to the sensor | rmse ≤ 0.07 m, ≥ 2.5×, NEES ∈ [0.3, 5] |

As shipped, the template fails all four — on the sensor model and not on the plumbing
([`student/FAILURE.md`](student/FAILURE.md)).

Printable sheets: [`docs/handout/exercises.pdf`](docs/handout/exercises.pdf), generated from
`config/tasks_localization.json`, so the paper and the grader cannot disagree.

## 7 · Parameters

Three doors, one set of names: the task file's `mcl` block, the `OHM_MCL_*` environment, and the launch
arguments. The environment beats the task file.

| name | launch argument | default | meaning |
|---|---|---|---|
| `OHM_MCL_PARTICLES` | `particles:=` | 1200 | N |
| `OHM_MCL_BEAM_STRIDE` | `beam_stride:=` | 3 | use every n-th beam; one update costs N × beams |
| `OHM_MCL_SIGMA_Z` | `sigma_z:=` | 0.15 m | how much one range is believed — the L4 knob |
| `OHM_MCL_PRIOR` | `prior:=` | 0.5 m | σ of the initial belief (L2 says 2.0) |
| `OHM_MCL_Z_RAND` | `z_rand:=` | 0.04 | clutter floor of the likelihood |
| `OHM_ICP_MODE` `_STRIDE` `_SIGMA_Z` `_GUESS` | — | `line` / 4 / 0.05 / `odom` | `icp_odom_node` |
| `OHM_TASKS` | — | `config/tasks_localization.json` | another task file, e.g. a group's own variant |
| `MECANUM_LAB` | `--sim-dir=` | found next door | where the simulator checkout is |

## 8 · Nodes and topics

| executable | what it does | needs a build? |
|---|---|---|
| `mcl_node` | the MCL localiser: `odom` + `scan` + map → `kf/pose` | no |
| `drive_node` | publishes a task's commanded drive onto `cmd_vel` | no |
| `icp_odom_node` | ICP between consecutive scans → `kf/pose`, no map | no |
| `map_server` | `/map` as `nav_msgs/OccupancyGrid`, latched | yes |
| `ohm-lab` | the simulator's launcher on this task file: `grade` · `run` · `sim` · `controller` | no |

| topic | direction | type |
|---|---|---|
| `/<robot>/odom` | in | `nav_msgs/Odometry` (in-process bus: the simulator's own `Odom`) |
| `/<robot>/scan` | in | `sensor_msgs/LaserScan` — 360 beams at 20 Hz, `range_max` where nothing echoed |
| `/map` | in (`map_server`) | `nav_msgs/OccupancyGrid`, 0.25 m cells, `transient_local` |
| `/<robot>/cmd_vel` | in (`drive_node`) | `geometry_msgs/Twist` |
| `/<robot>/kf/pose` | out | `geometry_msgs/PoseWithCovarianceStamped` — x, y, θ and their σ, which is graded |
| `/<robot>/particles` | out (ROS only) | `geometry_msgs/PoseArray` — the cloud, for the RViz view |
| `/<robot>/kf/path` | out (ROS only) | `nav_msgs/Path` — the estimate's own trail |
| `/<robot>/odom/path` | out (ROS only) | `nav_msgs/Path` — the wheel poses in the same frame, i.e. the drift |
| `/tf` | out (ROS only, `tf:=localizer`) | `<hall> -> <robot>/odom`, from the estimate; what puts the scan on the walls |
| `/sim/world`, `/sim/task` | in | the node asks which hall and which task; it hard-codes neither |

## 9 · When it does not work

| what you see | what it is |
|---|---|
| `package 'ohm_localization' not found` on a machine where the build succeeded | the ROS 2 in this terminal is not the one the workspace was built with. `source install/setup.bash`, or `./install.sh --check` to see which distro can run this package |
| `ros2: command not found` | no ROS 2 sourced in this terminal. A sourced ROS is a property of the terminal, not of the machine |
| `sudo apt install ros-<distro>-desktop` from `--check` | that distro has no `ros2` CLI, no `launch_ros` or no `nav_msgs` — a `ros-base` install. Install the desktop set, or build against the distro `--check` calls complete |
| the launch says rviz stays off | `headless:=true`, or no `DISPLAY` in this terminal. `rviz:=true` asks for the window anyway; `sudo apt install ros-$ROS_DISTRO-rviz2` installs it |
| RViz shows a robot and no walls | `map:=false` — the simulator publishes no map topic of its own, this package's `map_server` does |
| `World 'production': no importable mecanum_lab …` | the simulator is a separate checkout: `./install.sh --workspace`, or `export MECANUM_LAB=…`. This repository keeps no copy of the halls |
| the simulator refuses to start, something about surfaces | pygame is missing (`sudo apt install python3-pygame`), and it is needed for `--headless` too |
| grading says `rate of kf/pose 0.0` | you graded through `ros2 launch`; use `./tools/run_lab.sh grade …` |
| the estimate is good and NEES is out of its band | N_eff counts particles, not places: the cloud can stand in 13 boxes of 5 cm while N_eff is 1200 |
| RViz shows no particles | the localiser is not running, or it is a `./tools/run_lab.sh run` session: `/particles` exists on the ROS door only |
| the LIDAR fan lies off the walls, further away the longer the drive goes on | TF, not the filter: the scan is drawn where the top edge of the tree puts it, and `tf:=sim` makes that edge the identity, so the fan follows the wheel encoders. `tf:=localizer` (the default) publishes it from the estimate. 0.20 m against 0.016 m mean off the truth, in the `production` hall |
| the frame is called `hall` and not `map` | one name for one place: the simulator calls its ground frame `hall` in the tree where the localiser owns the top edge, and the launch spells it identically in RViz, in `/map` and in the transform. `tf:=sim` gives you `map` back |
| RViz fills its terminal with `Message Filter dropping message: frame 'map' at time 0.000 … queue is full` and the estimate arrow never appears | an old `mecanum-lab`, not your filter: `send_kf` used to stamp every `kf/pose` at 0.0 in the frame `map`, and the display's TF filter throws away what it cannot transform — `ros2 topic echo /<robot>/kf/pose --once` shows the header. Pull the simulator (`git -C ../mecanum-lab pull`) and rebuild with `./install.sh --workspace`; since then the header carries the measurement's stamp and the run's frame — docs/verification.md §18 |

## 10 · Layout

```
ohm_localization/   gridmap · mcl · icp · synth · paths · hall · lab · mcl_node · drive_node · icp_odom_node · map_server_node · view · rviz_config
examples/           four small programs, one command each — see examples/README.md
config/             tasks_localization.json: the four tasks — thresholds, drives, hints, protocol items
student/            mcl_template.py (your starting point) and FAILURE.md (how it fails as shipped)
solution/           mcl_node.py, the reference the thresholds were measured against
launch/             mcl.launch.py · mcl.rviz (the RViz view)
docs/               exercises · mcl · icp · handout/exercises.pdf
tools/              run_lab.sh and the instruments the numbers in docs/ were measured with
test/               the suite; ./tools/check.sh runs it and the documents beside it
```

* [`docs/exercises.md`](docs/exercises.md) — the four tasks with their checkpoints
* [`docs/mcl.md`](docs/mcl.md) — the filter: motion model, sensor model, N_eff, what each knob costs
* [`docs/icp.md`](docs/icp.md) — ICP: the two objectives, degeneracy, why stitching scans drifts
* [`examples/README.md`](examples/README.md) — the four examples and what each one prints

Three repositories make the laboratory: [`mecanum-lab`](../mecanum-lab) (the simulator, its physics and
its grader), [`ohm-nav-exploration`](../ohm-nav-exploration) (the navigation exercise on the same robot)
and this one. Nothing here edits the simulator; `ohm_localization/lab.py` injects this task file into its
grader.

MIT licensed.
