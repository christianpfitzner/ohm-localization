# launch/

One launch file and one view: `mcl.launch.py` starts the simulator (with our task file), `mcl_node`, the
task's commanded drive, the `map_server` and RViz 2, over real DDS. `mcl.rviz` is what the window shows —
the particle cloud, the estimate with its covariance, the scan, the odometry trail and the filter's path —
and it is the file to edit: the launch renders a copy per robot into `/tmp/ohm_mcl_<robot>.rviz`, because
RViz cannot substitute a robot name into a display, and "Save Config" in the window writes that copy.

`ros2 launch ohm_localization mcl.launch.py --show-args` lists every argument. `rviz:=auto` and `map:=auto`
are the defaults — both on unless the run is headless or has no `DISPLAY`, in which case the launch says so
instead of dying.

Grading runs through `./tools/run_lab.sh grade …`, not through a launch file.
