# Localizing against the saved SLAM map

`app.launch.py slam:=true` starts slam_toolbox in localization mode against a
saved LiDAR map. slam_toolbox then corrects the wheel/gyro drift by publishing
`map -> odom`. Nothing else changes: dock poses stay in `config/docks/<layout>.yaml`
(frame `arena`), Nav2 stays in `odom`, and the final approach still uses the tag.

```
slam:=false   arena --(start pose)--> odom --(EKF)--> base_footprint
slam:=true    arena --(start pose)--> map --(slam_toolbox)--> odom --(EKF)--> base_footprint
```

## The one rule

The map's origin is where the robot was when **mapping** started. `arena -> map`
reuses the start pose (`start_x/start_y/start_yaw`), so:

- Build the map with the robot on the competition start spot, facing the same way.
- Start every run from that same spot. Mark it on the floor with tape.

If the robot cannot start there, tell SLAM where it is with the "2D Pose
Estimate" tool in Foxglove/RViz, or publish `/initialpose` (frame `map`).

## 1. Build the map (once per arena)

Robot on the start spot. Terminal 1, the app without missions (odometry, lidar filter):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
ros2 launch einride_mini_truck_application app.launch.py layout:=arena mission:=false
```

Do not touch the robot until the terminal prints `Gyro bias calibrated: ...`
(about 10 s; driving earlier leaves the gyro bias in, ~0.007 rad/s, which bends
the whole map). Terminal 2, SLAM in mapping mode:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
ros2 launch einride_mini_truck_application slam.launch.py mode:=mapping
```

Check right away that the map starts where the robot is (about zero):

```bash
ros2 run tf2_ros tf2_echo map odom
```

Do not restart the app while mapping: odometry restarts at zero but the map
does not, and the map jumps by metres. If the app must restart, restart SLAM too.

Terminal 3, drive slowly around the whole arena and back to the start:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.10 -p turn:=0.35 -r cmd_vel:=/cmd_vel_nav
```

Keys (the teleop terminal must be the active window):

```
   u    i    o      i forward, , (comma) backward, j/l turn left/right
   j    k    l      u o m . = forward/backward while turning
   m    ,    .      k (or any other key) = STOP, q/z = all speeds +-10 %
```

Drive slowly, turn before reaching walls (ramming spins the wheels and corrupts
odometry), and finish back on the start spot so SLAM can close the loop.
Watch `/map` in Foxglove. When the walls form a clean, closed outline, save:

```bash
MAP=~/ws/src/Einride-Automotive-Hackathon/einride_mini_truck_application/maps/arena
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$MAP'}"
ls ${MAP}.*      # arena.data  arena.posegraph
```

Stop all three terminals with Ctrl+C. Maps are gitignored; they stay on the robot.

## 2. Run with the map

Robot back on the start spot. One terminal:

```bash
ros2 launch einride_mini_truck_application app.launch.py layout:=arena slam:=true \
  map_file:=$HOME/ws/src/Einride-Automotive-Hackathon/einride_mini_truck_application/maps/arena
```

Always pass the full map path (no extension). A wrong path, e.g. from an unset
shell variable, makes slam_toolbox fail to start; check
`ros2 lifecycle get /slam_toolbox` says `active [3]`. Test results:
[test-log.md](test-log.md).

## 3. Check it works

```bash
ros2 run tf2_ros tf2_echo arena map     # the start pose, constant
ros2 run tf2_ros tf2_echo map odom      # from slam_toolbox; changes slowly as drift is corrected
ros2 run tf2_ros tf2_echo arena base_footprint   # robot in the arena
```

- `map -> odom` must exist. If not, slam_toolbox did not load the map: check the
  terminal for the pose graph path and that `arena.posegraph` exists.
- Lift the robot, put it down ~0.3 m to the side, and watch
  `arena -> base_footprint`. Odometry does not see the move; with SLAM the
  position should jump to the true one within a few seconds (it may not for
  large moves: then use `/initialpose`).
- Drive 2-3 laps and park back on the start spot: `map -> base_footprint`
  should be within ~5 cm / 3 deg of the start; `odom -> base_footprint` is
  usually further off.
- Then the full Saga loop, with an obstacle in the way, as in test 6.

## What to tune when it fails

Change one thing at a time, rebuild
(`colcon build --packages-select einride_mini_truck_application`), retest.

| Symptom | Likely cause | Tune |
|---|---|---|
| `map -> odom` metres off right after starting mapping | app restarted or robot moved before calibration | not a parameter: restart both, wait for `Gyro bias calibrated` |
| Doubled / twisted walls in the map | bad odometry (turns) | turn test first: one full circle in place, `odom -> base_footprint` yaw back to ~0 deg (+-10). Then drive slower (`speed:=0.08 turn:=0.25`) |
| `ekf_filter_node: Failed to meet update rate` (constant) | Jetson overloaded | already 20 Hz (`config/localization/ekf.yaml`); turn off heavy Foxglove topics (`/oak/*`) |
| slam_toolbox `unconfigured`, robot does not move | wrong `map_file` path (e.g. an unset `$DIR`) | write the full path; check `ros2 lifecycle get /slam_toolbox` = `active [3]` |
| Scan does not line up with the map walls | robot not where mapping started | start on the tape, or "2D Pose Estimate" in Foxglove |
| Pose jumps while driving | scan matched to the wrong spot (look-alike walls, robots in view) | `slam_localisation.yaml`: `correlation_search_space_dimension` 0.5 -> 0.3 (search less far), `link_match_minimum_response_fine` 0.1 -> 0.2 (accept only good matches) |
| Pose lags / drifts between corrections | too few scans used | `minimum_travel_distance` and `minimum_travel_heading` 0.2 -> 0.1 (more CPU) |
| SLAM slow, `transform_timeout` / old-data warnings | CPU | `max_laser_range` 12.0 -> 6.0 (room-sized), `transform_timeout` 0.3 -> 0.5 |
| Lifted robot never re-found | search window too small | `/initialpose` (2D Pose Estimate); this is expected for big moves |

Mapping uses `config/localization/slam_mapping.yaml`, localization
`slam_localisation.yaml`; keep shared values the same in both.

