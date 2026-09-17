# launch/

One launch file and one view: `mcl.launch.py` starts the simulator (with our task file), `mcl_node`, the
task's commanded drive, the `map_server` and RViz 2, over real DDS. `mcl.rviz` is what the window shows —
the particle cloud, the estimate with its covariance, the scan, both trails and the hall —
and it is the file to edit: the launch renders a copy per robot into `/tmp/ohm_mcl_<robot>.rviz`, because
RViz cannot substitute a robot name into a display, and "Save Config" in the window writes that copy.

`tf:=localizer` — the default — is why the LIDAR fan is drawn on the walls instead of next to them. TF decides
where a scan is, and the top edge of the tree (`<hall> -> <robot>/odom`) can only have one publisher: this
value starts the simulator in its `slam` tree, where it stops publishing that edge, and `mcl_node` puts it
there from the estimate. `tf:=sim` gives the edge back to the simulator as the identity — the Kalman lab's
tree, where everything above the encoders drifts and that drift is the point — and `tf:=truth` closes it with
the answer, which shows what a localiser is worth by showing what the answer is worth. The frame's name
follows: `hall` in the first case, `map` in the other two, and the launch spells it identically in the viewer,
in `/map` and in the transform, because a Fixed Frame that names a frame nothing transforms into is an empty
window that reads like a broken filter.

`ros2 launch ohm_localization mcl.launch.py --show-args` lists every argument. `rviz:=auto` and `map:=auto`
are the defaults — both on unless the run is headless or has no `DISPLAY`, in which case the launch says so
instead of dying.

Grading runs through `./tools/run_lab.sh grade …`, not through a launch file.
