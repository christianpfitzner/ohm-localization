# E3 — Scan matching as the odometry of a real robot

*I2 · 180 minutes of a 180-minute session · 30 points · file `student/icp_odom_template.py`*

Command that decides it: `./tools/run_lab.sh grade --task icp_odom_production --controller student/icp_odom_template.py --headless`

The robot drives L1's course with an odometry that is not to be trusted — `odom.geometry.scale_xy = 1.12` and `odom.bias_omega = 0.10`, so every wheel tick is 12 % too long and the reported heading drifts at 0.1 rad/s while the robot drives straight. Your node has to say where it is using nothing but its own scans and one anchor. This is E2 run at 20 Hz on a machine that is moving, with the two things a desk exercise cannot hand you: a drive that integrates, and a hall that runs out of corners.

The matcher does not change and neither does its convention: `scan_step()` returns the transform that carries points from the previous scan into this one, which is `icp.relative(pose_prev, pose_scan)` in that order, and the robot's own increment is its inverse. What is new is everything around it — the anchor at t = 0, what you do with a fit you do not believe, and what a σ means when the thing it describes is an integral rather than a fit.

The miscalibration is not decoration and it is not a bug in the task: it is why the guess cannot be trusted and why the answer has to come from the scans. Do not fix the wheels, do not calibrate them, do not fuse them in. The grader's improvement column is your estimate against the raw odometry of this exact drive, and that column is the measurement this visit is about. Measured on one run of each: the odometry 3.98 m RMSE, the shipped fallback the same 3.97 m and an improvement of exactly 1.00 — because it *is* the odometry — and the reference solution 1.07 m, 3.74×.

## Do this, in this order

1. Get the plumbing visible first: `./tools/run_lab.sh grade --task icp_odom_production --controller student/icp_odom_template.py --headless`. The shipped node passes the rate and the contact criteria and fails the three accuracy ones, with an improvement of 1.00. Read those four lines before you write anything: they are the baseline you are being asked to beat, and they are what your node does the moment it reads the wheels instead of the scans.
2. Write `scan_step()`. Print the fitness, the correspondence count and the condition number of the last iteration for ten consecutive pairs on the straight and ten in the turn — the same code, and the numbers are not the same, and one of the two situations will make you want to refuse a fit.
3. Decide what an anchor is. The reference integrates from the first odometry pose, and that choice is worth 3.5 m on this task: the library's own ICP odometry node publishes in the frame of the first scan, which is a legitimate frame and not the one `/truth` is stated in, and graded on this task it comes to 5.26 m RMSE for a matcher whose pairwise error is 7 mm. Say in the protocol which frame your pose lives in and how you know.
4. Then the guard, and measure it rather than invent it: record one drive (`OHM_RECORD=drive.jsonl ./tools/run_lab.sh grade --task icp_odom_production --controller tools/record_scans.py --headless`) and replay it — a parameter is then 1.5 s and not 40 s. The median pair on this drive fits to 23.8 mm and 14 of 726 do not converge; refusing a fit over 60 mm takes the drive from 1.20 m to 0.99 m.
5. Finally the σ. `IcpOdom.sigmas()` widens the median pairwise σ by √pairs; without any of it the number you publish is 3 mm for a pose that is a metre away. Measure your NEES in the grader's `--json` output and be ready to explain why it is ~10² even after the widening — the answer to that question is the answer to 'what is an odometry'.

## Protocol — what goes in the write-up

* Error against drive time for one drive: your pose, the `/odom` pose and `/truth` in one plot on one axis. The shape of the curve is what to write about — yours and the wheels' should start together and come apart.
* The measured improvement column with the parameters that got it, and the same run with `T_guess` replaced by the identity: on this drive the guess is 1.8 mm and 0.005 rad off per step, and saying what that changed (measured, not predicted) is worth more than a paragraph about basins.
* One table of the guard's cost: RMSE of the same recorded drive with no rejection, with rejection at 60 mm, and with a `keep_min` of 0.55. The third column is the interesting one: the median pair keeps 48 % of its points *always*, so a 55 % floor throws most of the node's own updates away — measured 2.57 m on the replay and 2.812 m at 1.42× when graded, with nothing in the log to say a threshold has just deleted the sensor.
* The gate sweep: 0.25 m, 0.5 m and 1.5 m on the same recording. The ordering will disagree with your intuition about what a gate is for — measured 1.18 m, 0.99 m and 0.97 m on the replay, and 1.281 m, 1.065 m and 1.046 m as three graded drives of the reference.
* The four graded numbers (rmse, improvement, max error, rate) from the grader's own line, plus the NEES and the paragraph that explains its order of magnitude.

## Hand in

* `icp_odom_production` PASSing, with the improvement column quoted and the parameters that got it there.
* The error-against-drive-time plot, the guard table above with all three columns measured, and the NEES paragraph.

## What is measured

Graded by the simulator's own grader on 1 task in `config/tasks_localization.json` (icp_odom_production). The criteria and the points are the task file's own — the same file the grader loads — and the numbers it produced are in `docs/verification.md` §19 (and §14 for the four MCL drives).


### The two runs to compare (measured)

```
{
  "reference": {
    "rmse": 1.065,
    "improvement": 3.74,
    "max_error": 1.645,
    "rate_hz": 5.7,
    "contacts": 0,
    "nees": 111.3
  },
  "template": {
    "rmse": 3.968,
    "improvement": 1.0,
    "max_error": 7.347,
    "rate_hz": 5.7,
    "contacts": 0,
    "nees": 15.2
  },
  "raw_odometry_rmse": 3.984,
  "note": "One graded drive of each at bias_omega 0.10, scale_xy 1.12, on L1's 34 s course. The template's improvement of exactly 1.00 is not a coincidence: until `scan_step()` is written the node integrates the wheel step, so it is the odometry. NEES is deliberately not a criterion here — the error of an integrated pose is bias, not noise, and the reference's 111 against the fallback's 15 is the evidence for that sentence."
}
```


**What this is for.** ICP as a sensor instead of an algorithm: what a scan-matched pose is worth over a drive, what an anchor is worth, why the gate and the correspondence count are not quality meters, and why the σ of a fit and the error of an integral are two different kinds of number.
