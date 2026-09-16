# launch/

One launch file: `mcl.launch.py` — the simulator, `mcl_node`, and the task's commanded drive, over real
DDS. `ros2 launch ohm_localization mcl.launch.py --show-args` lists every argument; `rviz:=true` and
`map:=true` add the viewer and the `/map` topic.

Grading runs through `./tools/run_lab.sh grade …`, not through a launch file.
