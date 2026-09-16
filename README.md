# ohm_localization

Monte-Carlo localization (MCL) and ICP scan matching, built on the
[`mecanum-lab`](https://github.com/IntelligenteRobotik/mecanum-lab) simulator: a map layer, a particle filter,
four graded tasks and the nodes that run them. Written for the ASY2 practicum *Intelligente Robotik*, and a
package in the same colcon workspace as the simulator and
[`ohm_frontier`](../ohm-nav-exploration) — the three repositories are one laboratory, and this one is the
localization half of it.

Everything here is designed around one question: **how much better than your odometry are you?** The grader
answers it with a number, and every threshold in the task file is a number this checkout reproduces
([`docs/verification.md`](docs/verification.md) has the runs they came from).

```
./install.sh --workspace && source install/setup.bash        # build all three repos, 2 s
ros2 launch ohm_localization mcl.launch.py                   # the reference node, in a window
./tools/run_lab.sh grade --task mcl_production --controller student/mcl_template.py --headless
```

---

## Quickstart

### 1 · Layout

Three repositories, checked out side by side. Nothing is installed into anything else, and no file of the
simulator is edited — the four tasks live here and are injected into the simulator's grader by
`ohm_localization/lab.py`.

```
~/git/
  mecanum-lab/                     the simulator  (package `mecanum_lab`, has its own ./lab)
    interfaces/mecanum_lab_interfaces/     its ROS messages
  ohm-nav-exploration/ohm_frontier/        the navigation exercise (package `ohm_frontier`)
  ohm-localization/                this repository (package `ohm_localization`)
```

Any other layout works too: `MECANUM_LAB=/path/to/mecanum-lab ./install.sh --check` says what it can see, and
`ohm_localization/paths.py` finds the simulator as a checkout *or* as an installed prefix.

### 2 · Build

```bash
cd ~/git/ohm-localization
./install.sh --check            # what is missing, in exit-code classes (no sudo, installs nothing)
./install.sh --workspace        # colcon: interfaces, then mecanum_lab + ohm_localization + ohm_frontier
source install/setup.bash
```

`--workspace` is the three-repository build; `--build` builds only this package in place. Both use
`--symlink-install` and leave `build/`, `install/`, `log/` inside this directory, which is how the simulator
and `ohm_frontier` do it, so one editor and one `git status` cover the whole laboratory.

**Building is optional.** The graded path
([`tools/run_lab.sh`](tools/run_lab.sh)) runs the simulator and your controller in one process on the
simulator's in-process bus: no ROS, no daemon, no build. That is how all 130 points of the mark scheme were
calibrated, and it is what makes `./tools/run_lab.sh grade …` the fastest honest answer to "does my filter
work?". `ros2 launch` is the ROS door, and it needs the build.

### 3 · Run

```bash
ros2 launch ohm_localization mcl.launch.py                                   # reference node, production hall
ros2 launch ohm_localization mcl.launch.py task:=mcl_wide prior:=2.0          # L2: a 2 m prior
ros2 launch ohm_localization mcl.launch.py controller:=student/mcl_template.py headless:=true
ros2 launch ohm_localization mcl.launch.py task:=mcl_dirty lidar_sigma:=0.25 sigma_z:=0.5 rviz:=true map:=true
```

or, without ROS at all:

```bash
./tools/run_lab.sh run --world production --task mcl_wide --controller student/mcl_template.py --truth
ros2 run ohm_localization mcl_node --robot alice          # the same node as an executable, against a running sim
python3 -m ohm_localization.lab sim --world production --headless   # the simulator, on our task file
```

### 4 · Grade — this is the door the marks come through

```bash
./tools/run_lab.sh grade --task mcl_production --controller student/mcl_template.py --headless
./tools/run_lab.sh grade --task mcl_production --controller solution/mcl_node.py --headless   # the reference
```

```
PASS   30.0/ 30 pts  L1 — Where am I? Monte-Carlo localization against the map
        required: accuracy 0.015 ≤ 0.05
        required: improvement over raw sensor 12.47 ≥ 5.0
        required: NEES 0.19 in [0.05 … 5]
```

`grade:=` on the launch file starts the grader too, and **its number is not the number for a sheet**: a
localisation grade is measured from the `kf/pose` messages that cross DDS, and over that door the reference
solution scores 0 with `rate of kf/pose 0.0` — the simulator documents the same effect for its own KF tasks
([`mecanum-lab/launch/kf.launch.py`](../mecanum-lab/launch/kf.launch.py), CONTRACT §9.2). Use it to check
wiring, never to produce a result.

---

## What's in here

```
ohm_localization/         the library — numpy and nothing else, importable with no ROS and no simulator
  gridmap.py              a hall as an occupancy grid + an exact distance field; the OccupancyGrid conversion
  mcl.py                  the particle filter: motion model with rate-based noise, likelihood field, N_eff
  icp.py                  point-to-point and point-to-line ICP, SE(2), covariance, degeneracy
  synth.py                synthetic scans and drifting odometry, for tests and offline measurement
  paths.py                where the data is, in a checkout and in an install prefix
  mcl_node.py             the ROS node (`ros2 run ohm_localization mcl_node`) — the reference solution
  icp_odom_node.py        ICP as odometry: no map, and the drift that costs
  map_server_node.py      /map as a nav_msgs/OccupancyGrid, for RViz
  lab.py                  the simulator's launcher, on this repository's task file
config/tasks_localization.json   the four tasks: thresholds, drives, hints, protocol items — one source for
                                 the grader, the printed sheets and the documentation
solution/mcl_node.py      three lines: re-exports the node above, for `--controller <path>`
student/mcl_template.py   where you start: everything given except the sensor model. It fails, on purpose
student/FAILURE.md        how it fails, measured by tools/template_check.py
launch/mcl.launch.py      every argument of a ROS run, with the sensor overrides
docs/                     mcl.md · icp.md · exercises.md · verification.md · handout/exercises.pdf
tools/                    check.sh · run_lab.sh · mcl_report.py · icp_eval.py · drive_check.py ·
                          record_scans.py · scan_probe.py · template_check.py · make_handout.py
test/                     98 tests, numpy alone; `colcon test` runs them too
```

## Nodes and topics

| executable | what it does | needs ROS? |
|---|---|---|
| `mcl_node` | the MCL localiser: `odom` + `scan` + map → `kf/pose` | no (works on the stub bus) |
| `icp_odom_node` | ICP between consecutive scans → `kf/pose`, no map. For the comparison in `docs/icp.md` §7 | no |
| `map_server` | publishes `/map` (`nav_msgs/OccupancyGrid`, latched) built from the simulator's hall | yes |
| `ohm-lab` | `./lab` with our task file: `grade` · `run` · `sim` · `controller` | no |

| topic | direction | type | note |
|---|---|---|---|
| `/<robot>/odom` | in | `mecanum_lab_interfaces/Odom` (stub: in-process) | the wheels. `sim.odom.geometry.scale_xy` makes it drift |
| `/<robot>/scan` | in | `mecanum_lab_interfaces/LidarScan` | 360 beams, 20 Hz in the tasks, `inf` where nothing echoed |
| `/sim/world`, `/sim/task` | in | | the node asks which hall and which task, and never hard-codes either |
| `/<robot>/kf/pose` | out | `mecanum_lab_interfaces/KfPose` | x, y, θ **and sx, sy, sth** — the σ is graded (NEES) |
| `/map` | out (`map_server`) | `nav_msgs/OccupancyGrid` | 0.25 m cells, `transient_local` so a late RViz still sees it |
| `/<robot>/truth` | in, `truth:=true` only | | never read by the solution; the grader's yardstick |

## Parameters

Three doors, one set of names: the task file's `mcl` block (per task), the `OHM_MCL_*` environment, and the
launch arguments of the same name. The node reads the task file first and the environment over it, so
`OHM_MCL_SIGMA_Z=0.5 ./tools/run_lab.sh grade …` beats `config/tasks_localization.json`.

| name | launch arg | default | meaning |
|---|---|---|---|
| `OHM_MCL_PARTICLES` | `particles:=` | 1200 | N |
| `OHM_MCL_BEAM_STRIDE` | `beam_stride:=` | 3 | use every n-th beam; the cost of an update is N × beams |
| `OHM_MCL_SIGMA_Z` | `sigma_z:=` | 0.15 m | how much you believe one range — **the L4 knob** |
| `OHM_MCL_PRIOR` | `prior:=` | 0.5 m | σ of the initial belief (L2 says 2.0) |
| `OHM_MCL_Z_RAND` | `z_rand:=` | 0.04 | clutter floor of the likelihood |
| `OHM_ICP_MODE` / `_STRIDE` / `_SIGMA_Z` / `_GUESS` | — | `line` / 4 / 0.05 / `odom` | `icp_odom_node` |
| `OHM_TASKS` | — | `config/tasks_localization.json` | a different task file, e.g. a group's own variant |
| `MECANUM_LAB` | `--sim-dir=` | found next door | where the simulator is |
| `MECANUM_ROS` | — | `stub` | `stub` = in-process bus, `0` = real DDS |
| `OHM_MCL_TRACE` | — | off | one line per scan to a file: pose, σ, N_eff, beams, resamples |

## The tasks

All four are graded by the simulator's own `kind: "kf"` grader against `sensor: "odom"`, in the `production`
hall (20 × 12 m, walls inside, 323 of 360 beams find something). `reference` is the reference solution on the
simulator's grader, measured in this checkout; `template` is `student/mcl_template.py` as shipped.

| id | pts | the question | threshold | reference | template |
|---|---|---|---|---|---|
| `mcl_production` | 30 | write the sensor model; localise from a 0.5 m prior | rmse ≤ 0.05, improvement ≥ 5.0, NEES ∈ [0.05, 5] | 15 mm, 12.5×, NEES 0.19 | 0.68 m, 0.27× → FAILS |
| `mcl_wide` | 35 | same, from a 2 m prior | rmse ≤ 0.06, improvement ≥ 3.0, NEES ∈ [0.05, 8] | 15 mm, 12.6× | 0.68 m, 0.27× → FAILS |
| `mcl_budget` | 35 | same accuracy, a quarter of the particles, on a 53 s drive | rmse ≤ 0.07, improvement ≥ 3.0, NEES ∈ [0.05, 12] | 17 mm, 26.4× | 1.26 m, 0.35× → FAILS |
| `mcl_dirty` | 30 | the window is dirty (σ_lidar = 0.25 m): make the model match the sensor | rmse ≤ 0.07, improvement ≥ 2.5, NEES ∈ [0.3, 5] | 35 mm, 5.4× | 0.68 m, NEES 0.12 → FAILS |

130 points, and the odometry they are measured against drifts 0.19–0.44 m over the drive (that is what
`sim.odom.geometry.scale_xy: 1.03` and `bias_omega` are for — in the simulator's default profile the odometry
is 6 mm from the truth and no localiser can earn a factor of five against that).

* Start from [`student/mcl_template.py`](student/mcl_template.py): the node loop, the map, the prior, the
  motion model with its rate-based noise, N_eff and systematic resampling are given, and the sensor model is
  `TODO(L1)`. As shipped it publishes at 5.3 Hz, never hits a wall, and is three times *worse* than the
  odometry — see [`student/FAILURE.md`](student/FAILURE.md) and the reason why in the template's docstring.
* Print the sheets: [`docs/handout/exercises.pdf`](docs/handout/exercises.pdf) (8 pages) is generated from
  `config/tasks_localization.json`, so the mark scheme on paper cannot disagree with the grader.
* `docs/exercises.md` is the four-task walkthrough with the checkpoints; `docs/mcl.md` and `docs/icp.md` are
  the theory with the measurements next to the formulas.

## Tools

| command | what it answers | ~time |
|---|---|---|
| `./tools/check.sh` | suite + drive fit + ICP claims + sheet drift, no simulator session | 60 s |
| `./tools/check.sh --live` | the four graded runs, the template's failure profile, the map-convention probe, a colcon build | ~7 min |
| `./tools/drive_check.py` | does the commanded drive fit the hall, and how many beams echo there | 3 s |
| `./tools/record_scans.py` | record `scan + odom + truth` to JSONL for offline measurement | per run |
| `./tools/mcl_report.py` | σ_z sweeps, N_eff, timing, `--compare` two recordings — the offline answer to a parameter question | 3 s |
| `./tools/icp_eval.py --claims` | recomputes every ICP number quoted in `docs/icp.md`, exits non-zero on drift | 10 s |
| `./tools/icp_eval.py --basin2d` | the basin of attraction as an ASCII map | 10 s |
| `./tools/template_check.py --record` | what the shipped template scores, into `student/FAILURE.md` | 3 min |
| `./tools/scan_probe.py` | is our map/beam convention the simulator's (identity error, mirror, clock direction) | 40 s |

## Troubleshooting

| what you see | what it is |
|---|---|
| `World 'production': no importable mecanum_lab and no worlds/production.txt in …` | the simulator is neither importable nor next door. It is a separate repository: `./install.sh --workspace`, or `export MECANUM_LAB=…`. This repository keeps **no copy** of the halls, on purpose |
| `no such world 'production'` after a colcon build | an old `ohm_localization`: `paths.mecanum_lab_root()` resolves the hall text through the simulator's own two layouts (checkout and `share/mecanum_lab/worlds`) |
| `mecanum-lab not found — set MECANUM_LAB` from `run_lab.sh` | `./install.sh --check` prints what it looked for; exit code 6 means exactly this class |
| the simulator refuses to start, something about surfaces | pygame is missing (`sudo apt install python3-pygame`) — it is needed even for `--headless`, and `install.sh` says so before you find out |
| grading over `ros2 launch` says `rate of kf/pose 0.0` | expected: see step 4. Grade through `tools/run_lab.sh` |
| the estimate is fine and the σ is graded as inconsistent (`NEES …`) | the σ is not a decoration. N_eff counts *particles*; the cloud can stand in 13 boxes of 5 cm while N_eff is 1200 — `mcl_report.py` prints both |
| `ImportError: cannot import name 'OccupancyGrid'` | `map_server` needs `nav_msgs`; nothing else in this package does, and `--dry-run` prints the map without ROS |
| RViz shows a robot and no walls | you did not pass `map:=true`, or you hid the `/map` display: the simulator publishes no map topic of its own |

## The three repositories

| repository | package | what it owns | what this repository does with it |
|---|---|---|---|
| [`mecanum-lab`](../mecanum-lab) | `mecanum_lab` | physics, sensors, worlds, the grader, the RViz config | reads its hall text and its `Scan`/`Odom` shapes; injects our task file into its grader; never edits it. Our `sim` blocks drive it; its own `config/tasks.json` is not read |
| [`ohm-nav-exploration`](../ohm-nav-exploration) | `ohm_frontier` | the frontier-exploration exercise on the same robot | shares the workspace, the launch conventions, the `--controller <file>` idiom and this README's structure. A `map_server` output is what a nav2-style stack would consume; the two exercises stay independent |
| this repository | `ohm_localization` | the map layer, MCL, ICP, four tasks, the nodes | — |

`./install.sh --workspace` builds all three in dependency order, and the messages go through a build of their
own first: when `mecanum_lab_interfaces` fails on a partial ROS install (it needs
`rosidl_default_generators`), colcon aborts the packages behind it — measured here, where a workspace with
one colcon invocation built *nothing* and a sandbox with a partial `/opt/ros/jazzy` still gets both exercises
afterwards.

## Tests

```bash
./tools/check.sh                 # 96 tests + the checks above, no ROS, no simulator session
python3 -m pytest test -q        # just the suite (~60 s)
colcon test --packages-select ohm_localization && colcon test-result --verbose
```

Every threshold in the suite and in the task file is a measured number, and `docs/verification.md` records
where each came from — including the 13 entries of its bug ledger, which is the more useful half: a mirrored
map convention, a `temperature` knob that hid a motion model, a per-step noise floor that made the filter
diffuse √2.5 faster at 50 Hz than at 20 Hz, an ICP that never wrote its own result, and a rotation noise term
driven by the bearing of a step that never moved. When a measurement and a docstring disagree, this repository
fixes the docstring or the code, in that order, and never the threshold.

## Documentation

* [`docs/exercises.md`](docs/exercises.md) — the four tasks, checkpoints, what to write in the protocol
* [`docs/mcl.md`](docs/mcl.md) — the filter: motion model, sensor model, N_eff, and what each knob costs
* [`docs/icp.md`](docs/icp.md) — ICP: the two objectives, degeneracy, and why stitching scans drifts
* [`docs/verification.md`](docs/verification.md) — every number, where it came from, and the bugs it came out of
* [`docs/handout/exercises.pdf`](docs/handout/exercises.pdf) — the printable sheets, generated from the task file

MIT-lizenziert. The handout is built by `pdflatex`/`latexmk`; `pandoc` is not needed and not used.
