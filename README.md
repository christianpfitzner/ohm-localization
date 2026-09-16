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

The reference filter in the `production` hall. The commanded drive of task L1 is driven for you and the
launch exits when it is through. Measured here over DDS: **17 mm** RMSE against **196 mm** of raw
odometry, 11.8×, 16 reports/s.

One argument at a time — `--show-args` lists all of them:

```bash
ros2 launch ohm_localization mcl.launch.py --show-args
```

```bash
ros2 launch ohm_localization mcl.launch.py task:=mcl_wide prior:=2.0
```

```bash
ros2 launch ohm_localization mcl.launch.py controller:=student/mcl_template.py headless:=true
```

```bash
ros2 launch ohm_localization mcl.launch.py task:=mcl_dirty sigma_z:=0.5 rviz:=true map:=true
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

The reference solution, measured in this checkout:

```
PASS   30.0/ 30 pts  L1 — Where am I? Monte-Carlo localization against the map
        required: accuracy 0.015 ≤ 0.05
        required: improvement over raw sensor 12.47 ≥ 5.0
        required: rate of kf/pose 34.1 ≥ 5
        required: NEES 0.19 in [0.05 … 5]
```

`grade:=` on the launch file starts the grader in the simulator and your node in a second process; over
that door a localisation grade is measured from `kf/pose` messages crossing DDS and reports
`rate of kf/pose 0.0`, the effect the simulator documents for its own KF tasks. Use it to check wiring,
never to produce a number.

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

All four are graded in the `production` hall (20 × 12 m, 323 of 360 beams find something) against the
odometry of `sim.odom.geometry.scale_xy: 1.03`, which drifts 0.19–0.44 m over a drive. `reference` and
`template` are `solution/mcl_node.py` and `student/mcl_template.py` as shipped, measured here.

| id | pts | the question | threshold | reference | template |
|---|---|---|---|---|---|
| `mcl_production` | 30 | write the sensor model; localise from a 0.5 m prior | rmse ≤ 0.05, improvement ≥ 5.0, NEES ∈ [0.05, 5] | 15 mm, 12.5×, NEES 0.19 | 0.68 m, 0.27× → fails |
| `mcl_wide` | 35 | the same from a 2 m prior | rmse ≤ 0.06, improvement ≥ 3.0, NEES ∈ [0.05, 8] | 15 mm, 12.6× | 0.68 m, 0.27× → fails |
| `mcl_budget` | 35 | same accuracy, a quarter of the particles, 53 s drive | rmse ≤ 0.07, improvement ≥ 3.0, NEES ∈ [0.05, 12] | 17 mm, 26.4× | 1.26 m, 0.35× → fails |
| `mcl_dirty` | 30 | the window is dirty (σ_lidar = 0.25 m): match the model to the sensor | rmse ≤ 0.07, improvement ≥ 2.5, NEES ∈ [0.3, 5] | 35 mm, 5.4× | 0.68 m, NEES 0.12 → fails |

`student/mcl_template.py` is the starting point: the node loop, the map, the prior, the motion model,
`N_eff` and the resampling are given, the sensor model is `TODO(L1)`. As shipped it publishes at 5.3 Hz,
never touches a wall and is three times worse than the odometry
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
| `/sim/world`, `/sim/task` | in | the node asks which hall and which task; it hard-codes neither |

## 9 · Tests and measurements

```bash
python3 -m pytest test -q
```

99 tests, numpy alone, about 70 s. `tools/` holds the instruments the thresholds were measured with:

| command | what it answers | ~time |
|---|---|---|
| `./tools/check.sh` | suite + drive fit + ICP claims + sheet drift | 60 s |
| `./tools/check.sh --live` | the four graded runs, the template's failure profile, a colcon build | ~7 min |
| `python3 tools/mcl_report.py <recording>` | σ_z sweeps, N_eff, timing, `--compare` two recordings | 3 s |
| `python3 tools/icp_eval.py --claims` | recomputes every ICP number in `docs/icp.md`, non-zero on drift | 10 s |
| `python3 tools/drive_check.py` | does the commanded drive fit the hall, how many beams echo there | 3 s |
| `python3 tools/record_scans.py` | record `scan + odom + truth` to JSONL (needs `OHM_RECORD=`) | per run |
| `python3 tools/scan_probe.py` | is our map/beam convention the simulator's | 40 s |
| `python3 tools/template_check.py --record` | what the shipped template scores, into `student/FAILURE.md` | 3 min |

## 10 · When it does not work

| what you see | what it is |
|---|---|
| `package 'ohm_localization' not found` on a machine where the build succeeded | the ROS 2 in this terminal is not the one the workspace was built with. `source install/setup.bash`, or `./install.sh --check` to see which distro can run this package |
| `ros2: command not found` | no ROS 2 sourced in this terminal. A sourced ROS is a property of the terminal, not of the machine |
| `sudo apt install ros-<distro>-desktop` from `--check` | that distro has no `ros2` CLI, no `launch_ros` or no `nav_msgs` — a `ros-base` install. Install the desktop set, or build against the distro `--check` calls complete |
| `World 'production': no importable mecanum_lab …` | the simulator is a separate checkout: `./install.sh --workspace`, or `export MECANUM_LAB=…`. This repository keeps no copy of the halls |
| the simulator refuses to start, something about surfaces | pygame is missing (`sudo apt install python3-pygame`), and it is needed for `--headless` too |
| grading says `rate of kf/pose 0.0` | you graded through `ros2 launch`; use `./tools/run_lab.sh grade …` |
| the estimate is good and NEES is out of its band | N_eff counts particles, not places: the cloud can stand in 13 boxes of 5 cm while N_eff is 1200. `tools/mcl_report.py` prints both |
| RViz shows a robot and no walls | you did not pass `map:=true`; the simulator publishes no map topic of its own |

## 11 · Layout

```
ohm_localization/   gridmap · mcl · icp · synth · paths · lab · mcl_node · drive_node · icp_odom_node · map_server_node
examples/           four small programs, one command each — see examples/README.md
config/             tasks_localization.json: the four tasks — thresholds, drives, hints, protocol items
student/            mcl_template.py (your starting point) and FAILURE.md (how it fails as shipped)
solution/           mcl_node.py, the reference the thresholds were measured against
launch/             mcl.launch.py
docs/               exercises · mcl · icp · verification · handout/exercises.pdf
tools/              the instruments the numbers were measured with
test/               99 tests, numpy alone
```

* [`docs/exercises.md`](docs/exercises.md) — the four tasks with their checkpoints
* [`docs/mcl.md`](docs/mcl.md) — the filter: motion model, sensor model, N_eff, what each knob costs
* [`docs/icp.md`](docs/icp.md) — ICP: the two objectives, degeneracy, why stitching scans drifts
* [`docs/verification.md`](docs/verification.md) — every number and where it came from
* [`examples/README.md`](examples/README.md) — the four examples and what each one prints

Three repositories make the laboratory: [`mecanum-lab`](../mecanum-lab) (the simulator, its physics and
its grader), [`ohm-nav-exploration`](../ohm-nav-exploration) (the navigation exercise on the same robot)
and this one. Nothing here edits the simulator; `ohm_localization/lab.py` injects this task file into its
grader.

MIT licensed.
