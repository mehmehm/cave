# Running experiments from one terminal

`run_experiments.sh` replaces your 13 terminals. For each configuration it:

| Step | What it starts | Was |
|---|---|---|
| 1 | roscore + `roslaunch gazebo_cave_world cave_world.launch` | terminal 1 |
| 2 | `roslaunch cave_sensor_robot spawn_cave_a1.launch`, then waits for `/velodyne_points` and `/a1/trunk_imu` and 15 s for the A1 to stand | terminal 2 |
| 3 | IMU filter, Velodyne time repair (`scan_period 0.1`), LIO-SAM; waits for `/lio_sam/mapping/odometry` | terminals 3-5 |
| 4 | fusion, mode selector, voxel map, OctoMap, frontier detector, metrics logger, rosbag, planner, follower (`/a1/cmd_vel`) and mission manager | terminals 6-13 |

It then waits until the robot is back at its start (or the 40-minute cap),
saves the CSV, summary and bag, shuts everything down and starts the next run.

## One-time setup

The files must be in the workspace you actually build. If you build
`~/a1_ws` but keep the git repo in `~/cave/cave`, copy the edited files into
`~/a1_ws/src/...` (or point `WORKSPACES` in `config/experiment.conf` at the
workspace you build).

```bash
cd ~/a1_ws/src
# Linux line endings + executable bits (needed if files came via Windows)
sed -i 's/\r$//' cave_evaluation/scripts/run_experiments.sh cave_evaluation/config/experiment.conf \
                 cave_exploration/scripts/mission_manager.py
chmod +x cave_evaluation/scripts/run_experiments.sh cave_exploration/scripts/mission_manager.py
cd ~/a1_ws && catkin_make && source devel/setup.bash
```

Check CHAMP's speed limit: the follower asks for up to 0.35 m/s and 0.9 rad/s.
If `a1_config/config/gait/gait.yaml` sets `max_linear_velocity_x` or
`max_angular_velocity_z` lower, CHAMP clips the command; raise them or pass `-s`
with a lower speed.

Close any Gazebo or roscore you have open first; the script stops them.

## Running

```bash
cd ~/a1_ws/src/cave_evaluation/scripts
bash run_experiments.sh -t 600 -c ADAPTIVE    # 10-minute test first
bash run_experiments.sh                       # LIDAR_ONLY, RGB_LIDAR, THERMAL_LIDAR, ADAPTIVE once each
bash run_experiments.sh -n 3                  # 3 repeats of every configuration
bash run_experiments.sh -c ADAPTIVE --rviz    # watch it in RViz
bash run_experiments.sh --no-bag              # skip rosbag recording
```

The terminal prints one status line a minute and a result line per run.
Outputs:

* `~/a1_ws/experiments/results/<config>_mixed_runNN.csv` and `_summary.json`
* `~/a1_ws/experiments/bags/<config>_mixed_runNN.bag`
* node output in `~/a1_ws/experiments/logs/<time>_<config>_runNN/`
  (`world.log`, `a1.log`, `localisation.log`, `pipeline.log`)

Run numbers continue from the files already there, so pilot results are never
overwritten. Ctrl-C stops the current run cleanly; its partial CSV, summary and
bag are still written.

If a run fails at start-up, the script names the stage that timed out; look in
that stage's log.

## What a run does

* **Exploring**: frontier goals go to the A* planner as before.
* **Returning**: the mission manager sends the start pose as the goal when
  either (a) time left is no more than the reserve needed to get home (at
  least 4 min, more when the robot is far away), or (b) no frontiers remain
  for 60 s.
* **Returned / Timeout**: the robot stops and the run ends. A run that hasn't
  got home by 40 min of simulation time is recorded as `TIMEOUT`.

New CSV columns: `elapsed_s`, `robot_x`, `robot_y`, `distance_to_start`,
`mission_phase`. The summary JSON adds `returned_to_start`,
`final_distance_to_start_m` and `mission_events`. Arriving home isn't counted
as an exploration goal.

## Changes that affect comparison with the pilot runs

* Adaptive thresholds are now 0.118 / 0.130 mean brightness (was 0.15 / 0.20,
  which the MIXED cave never reached). `--auto-threshold` instead calibrates
  them from the first 60 s of each run.
* The mode selector is vectorised. Output is bit-identical to before, but
  `selector_processing_ms` drops from ~400 ms to a few ms with thermal, so
  don't compare processing time between pilot and new runs.
* The robot is faster (0.35 m/s, was 0.18) and its goal tolerance is looser,
  so distance and goals per minute aren't comparable with the pilot runs.
* The summary's `duration_s` now starts when simulation time starts, not when
  the logger was launched.
