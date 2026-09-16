# examples

Four programs, one command each. 01–03 need numpy and nothing else — no ROS, no simulator, no build —
and run in a few seconds. 04 is the ROS one. They import `ohm_localization`, so run them from a clone
(`sys.path` is set up in the file) or after `source install/setup.bash`.

## 01 · A hall, a grid, one scan

```bash
python3 examples/01_map_and_scan.py
```

The hall is written as text in the file (`#` wall, `.` floor, 0.5 m per character). The program prints
the grid it becomes, casts one 180-beam scan through the same wall rectangles the grid was built from,
and draws both: `#` wall, `.` floor, `o` the robot, `*` a beam that echoed.

```
Hall('example' 12x4.5 m, 76 rectangles, cell 0.5 m)
GridMap(example 48x18 @ 0.25 m field @0.1 m, 560 free cells)
scan: 180 beams, 179 echoed, nearest echo 0.50 m
```

Change `HALL` or `POSE` at the top and look at what moves.

## 02 · The whole filter, no simulator

```bash
python3 examples/02_mcl_localisation.py
```

A synthetic drive in a corridor, odometry from wheels 3 % too large and a gyro 4 mrad/s off, and the
particle filter from `ohm_localization.mcl` — predict with the odometry, weight with the scan, resample
when `N_eff` falls. The filter never sees the truth it is scored against.

```
  odom rmse    230 mm   worst    472 mm   final    472 mm
  mcl  rmse     50 mm   worst    141 mm   final     19 mm
  the filter is 4.6× better than the wheels
```

The σ_z of the sensor model as an argument, and the σ the filter publishes changes with it:

```bash
python3 examples/02_mcl_localisation.py 0.6
```

The last line is NEES: 1.0 means the σ the filter publishes matches the error it makes. It is the number
the graded tasks check, and it is the one a group can pass by accident — a σ wide enough to cover
anything also covers nothing.

## 03 · ICP between two scans

```bash
python3 examples/03_icp_scan_matching.py
```

One pair of scans in a hall with posts, one in a 121 × 5 m corridor, both from a 0.6 m move with 5° of
turn. For each it prints the error against the truth, the σ along and across the hall, and the
correspondence distance at the answer versus 0.5 m away from it.

```
  hall with posts     19 mm off,   0.03°, 4 iterations,  87 pairs
  empty corridor      28 mm off,   0.98°, 5 iterations,  85 pairs
```

The corridor's two long walls fix the sideways offset and the heading; almost nothing fixes a shift
along them. That is the degeneracy `docs/icp.md` is about, and it is why ICP is a good correction to
odometry and a poor replacement for it.

## 04 · MCL as a plain ROS 2 node

The simulator, a map topic and something to drive the robot; one command per terminal.

```bash
python3 -m ohm_localization.lab sim --world production --robots alice --task mcl_production --truth
```

```bash
ros2 run ohm_localization map_server --world production
```

```bash
ros2 run ohm_localization drive_node --robot alice
```

```bash
python3 examples/04_mcl_ros_node.py --robot alice
```

`04_mcl_ros_node.py` is rclpy, the standard message types and `ohm_localization.mcl`: it subscribes to
`/map` (`nav_msgs/OccupancyGrid`), `/<robot>/odom` and `/<robot>/scan`, and publishes
`/<robot>/kf/pose` (`geometry_msgs/PoseWithCovarianceStamped`, x, y, θ and their σ). No simulator import,
no custom messages, so the map it localises against can come from a mapper instead.

Watch the estimate:

```bash
ros2 topic echo /alice/kf/pose --once
```

`--particles` and `--sigma-z` are its only knobs:

```bash
python3 examples/04_mcl_ros_node.py --robot alice --sigma-z 0.4
```
