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

Keep the robot still ~3 s (gyro calibration). Terminal 2, SLAM in mapping mode:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
ros2 launch einride_mini_truck_application slam.launch.py mode:=mapping
```

Terminal 3, drive slowly around the whole arena and back to the start:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.10 -p turn:=0.35 -r cmd_vel:=/cmd_vel_nav
```

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
ros2 launch einride_mini_truck_application app.launch.py layout:=arena slam:=true
```

Another map: `map_file:=/path/to/map` (no extension).

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
- Then the full Saga loop, with an obstacle in the way, as in test 6.
