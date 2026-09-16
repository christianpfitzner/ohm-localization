# Status of the open items

Everything in the first version of this file is implemented, and the second list is what is genuinely left.
Both lists carry the measurement or the file that settles each item, because a todo list whose entries are
closed by intention rather than by evidence is a list of wishes.

## Done since the last version of this file

### The package: `colcon build` in a workspace — the reason this round exists
* `package.xml`, `setup.py`, `setup.cfg`, `resource/ohm_localization`: an `ament_python` package named
  `ohm_localization`, installed as `share/ohm_localization/{config,launch,solution,student,docs}` with four
  console scripts. `./install.sh --build` builds it in place (1.0 s), `./install.sh --workspace` builds
  interfaces → `mecanum_lab` + `ohm_localization` + `ohm_frontier` (3 packages, 1.5 s), with the messages in a
  pass of their own so that a partial ROS install cannot abort both exercises behind it. Measured in
  `docs/verification.md` §11.
* `ohm_localization/paths.py` answers "where is my data" in both layouts, and `gridmap.load_hall` asks the
  simulator for its own hall text (`mecanum_lab.types.data_root`) instead of guessing a checkout — an
  *installed* simulator used to mean "no such world 'production'". Verified from `/tmp` with a sourced
  workspace and `HOME` moved away: all six halls load.
* `launch/mcl.launch.py`, following the simulator's `kf.launch.py` idiom (sim + controller as two
  `ExecuteProcess`, `headless:=`, `--set` overrides, rviz via `mecanum_lab.rviz_view`), plus the graded-over-ROS
  caveat in its own docstring and a `LogInfo` at runtime when `grade:=` is used anyway.
* `ohm_localization/lab.py` — `./lab` with our task file, reachable as `ros2 run ohm_localization ohm-lab` and
  used by both the shell launcher and the launch file, so the task-file surgery happens in one place.
* `test/test_packaging.py`: the installed branch tested against a **fake prefix**, the build tree refused as a
  "checkout", entry points resolved to callables, launch arguments declared-vs-read, `--set` paths checked
  against the simulator's config, the docstring's `…:=` examples checked against the declarations, and a real
  `LaunchDescription` built where `launch_ros` exists (skips with that reason here).

### Map as a ROS message
* `gridmap.occupancy_grid()` / `gridmap.hall_from_occupancy_grid()` (dict in the middle, no `nav_msgs` needed
  to test the geometry), `map_server_node.py` publishing a latched `/map`. Round trip exact in four halls;
  origin, row order, unknown-vs-wall and truncation all asserted in `test/test_occupancy_grid.py`.

### The student side
* `student/mcl_template.py`: everything given except the sensor model, and it **fails on purpose**.
* `tools/template_check.py --record/--check` → `student/FAILURE.md`: which criteria it misses, per task, with
  the numbers. `--check` is in `tools/check.sh --live`.
* `docs/handout/exercises.pdf` (`tools/make_handout.py`), 8 pages, generated from the task file, drift-checked.

### Two real bugs this round found, and what they cost
* A step that does not translate has no bearing: the rotation noise was driven by `atan2` of the odometry's
  own jitter, so a standing or spinning robot was charged 0.08 rad of heading noise per step. σ_θ after 10 s of
  standing: 1.69 rad → 0.57 rad (floor 0.28). The four graded tasks improved to 15/15/17/35 mm and **two NEES
  floors had to be widened** because a correct answer was 1.8× from failing. `docs/verification.md` §12.
* ICP odometry chained the transform in the wrong direction: exact truth deltas integrated 4.9 m away from the
  truth they came from. Now controlled by a test before any ICP number is believed. §13.

### Also
* `icp_odom_node.py` (ICP as odometry, with the bias measurement that explains its drift), `mcl_report.py
  --compare`, `tools/icp_eval.py --basin2d` (ASCII basins of attraction), 98 tests, README rewritten as
  package documentation with a Quickstart, and `docs/verification.md` up to §14.

## What is left, and why it is not here

* **`ros2 launch … mcl.launch.py` has never been run.** This sandbox has no `ros2` CLI, no `launch_ros` and no
  `nav_msgs`, so the launch file is verified by parsing and by building its description up to the ROS-specific
  parts, and the entry points resolve. On a machine with a desktop ROS: run it, then `ros2 topic hz /alice/kf/pose`
  (expect ≥ 5 Hz) and `ros2 topic echo /map --once`. Anything that fails there is a real finding for
  `docs/verification.md` §11, which is written to receive it.
* **Grading over DDS is documented as broken, not measured as broken here.** The simulator says a KF grade
  taken over ROS scores `rate of kf/pose 0.0`; that number is *theirs*. `MECANUM_ROS=0 ./tools/run_lab.sh
  grade …` on a full install is the one command that turns "the simulator documents this" into "we measured
  it", and it should be recorded rather than quoted from the other repository.
* **`rosidl` interfaces build:** fails in this sandbox (`rosidl_default_generators` absent), tolerated by
  design, and `--workspace` prints why. On a full desktop it should build; nobody has seen it build here.
* **L5 (ICP odometry as a graded task): deliberately not done.** The measured ceiling — 0.91–1.76 m over the
  graded drive against 0.07–0.18 m of wheel odometry — is *worse than the baseline the grader compares
  against*, so every threshold on it would grade the choice of method and not the quality of an implementation.
  It is a node, a viva question with numbers, and `docs/icp.md` §7 instead. If it ever becomes a task, it needs
  a different baseline (a GPS-denied drive, or `improvement` against a *deliberately crippled* odometry), and
  the ceiling has to be measured before the threshold is written — that order, always.
* **The 180-minute format is untested with humans.** Which of the four tasks a group actually finishes, and
  whether the printed sheet is fillable in the time, is the largest unknown left, and no measurement in this
  repository can substitute for one group sitting down with it.
* **ROS 2 distro pin is still a decision nobody has made.** LAB-CONCEPT says Jazzy, the simulator's docs say
  Kilted; `install.sh` sources whatever it finds and reports it. Harmless until a group's RViz behaves
  differently from another's.

## Smaller things, if there is time

* `map_server` publishes no `/map_metadata` and no `map_server` *service*, so `nav2`'s own tools will not
  recognise it as a map server. Deliberate: the topic is for RViz and for a group's own node, and the nav2
  contract is the next exercise's problem.
* `icp_eval.py --integrate` is currently `python3 -c` against a recording; the numbers are pinned by
  `test/test_icp_odometry.py`, so this is convenience only.
* `docs/*` are English with German task titles quoted from the task file; the handout inherits whichever the
  task file holds. A German handout would be one flag in `make_handout.py` and needs the task file to carry both.
