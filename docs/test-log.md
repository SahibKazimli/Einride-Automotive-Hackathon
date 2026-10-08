# Robot test log

Every robot test with its numbers, newest last, so results are not lost
between sessions. Add a row after each test. Recommendations for the finals
are at the end.

How to read the pose numbers: `ros2 run tf2_ros tf2_echo <frame> base_footprint`,
position in metres, heading (yaw) in degrees. "Lap error" = reading back on the
start mark minus the reading at the start.

## Before 2026-10-07 (home layout, no SLAM)

| Test | Result |
|---|---|
| 1. Odometry | works; gyro bias calibrated at start |
| 2. Nav2 driving | works |
| 3. Obstacle avoidance (bucket) | works after `limit` instead of `slowdown` in the collision monitor (turning next to the bucket was too weak) |
| 4. Tag detection | works; camera reaches apriltag at only ~0.7 Hz (CPU) |
| 5. Docking | works ("first successful dock, obstacle in the middle", c1c72c4) |
| 6. Full Saga loop | in progress: staging around the bucket worked; the tag was not where home.yaml said, so it was not seen from the staging pose |

## 2026-10-07: SLAM in a room (tags A, B, C on the walls, glass door at one end)

| Test | Setup | Result | Conclusion / action |
|---|---|---|---|
| Mapping, try 1 | full app + mapping | `map -> odom` 4.1 m / -46 deg after the lap | app restarted or robot driven before `Gyro bias calibrated`; map discarded |
| App start-up | full app | gyro calibration took ~10 s (not 3 s); bias +0.007 rad/s; EKF "Failed to meet update rate" at 30 Hz | wait for the log line, not a fixed time; EKF to 20 Hz (2b1e022) |
| Mapping, try 2 | `localization.launch.py` + mapping + teleop only | `map -> odom` 0 at start, 6 cm / -10 deg after one lap; clean walls | saved as `maps/room_1lap`. Glass door invisible to the lidar |
| Localization lap 1 | `slam:=true mission:=false` | lap error: map 14 cm / 4 deg, odom 76 cm / 40 deg; scan slid in turns, "Message Filter dropping message: queue is full" | SLAM works; odometry poor; Jetson overloaded |
| Collision monitor | teleop next to an obstacle | robot froze, no command (not even reverse) worked | stop box all round blocked retreat; fixed with `velocity_polygon` (f976bd5) |
| Localization lap 2 | start exactly on the mark (map at start 0.010, -0.100, 0.07 deg, same as lap 1) | lap error: **map 4.8 cm / 7.8 deg**, **odom 1.6 m / 92 deg** | SLAM good. Odometry lost slow turns: encoders tick per cm, so between ticks the wheels look still and the gyro's turn was zeroed and learned as bias. Fixed (e7c42a9) |

## 2026-10-08: fixes checked (same room)

| Test | Result | Conclusion / action |
|---|---|---|
| Start-up with `map_file:=$DIR/...` and `$DIR` unset | slam_toolbox failed to configure; Nav2 stuck half started (collision monitor `unconfigured`), robot would not move | write the map path out in full. Check `ros2 lifecycle get /slam_toolbox` = `active [3]` |
| Turn test (one full left circle in place) | odom yaw 0.1 -> -15.4 deg; SLAM (map) yaw +2.4 deg | slow-turn fix works (was 92 deg per lap). Gyro reads **~5% low** (~18 deg per circle): add `gyro_scale` (TODO) |
| Collision monitor | obstacle in front: forward blocked, reverse works; behind: reverse blocked, forward works; narrow passages fine | fix works |
| Chair leg | not stopped for | chair base is below the lidar plane (14 cm); the lidar cannot see it. Arena obstacles (robots, buckets) are taller. Depth camera as a second source (TODO) |
| Survey start | tag C (2) seen from the start mark, but every sample dropped: "extrapolation into the past" | camera timestamps **27 min behind** the system clock (`/oak/rgb/image_raw` stamp vs `date +%s`) |
| Camera clock after a reboot | 665 s (11 min) behind | the robot has no clock battery; after boot the clock jumps forward when network time arrives, but the camera driver keeps its old offset. Tag positions then never match a robot pose: **survey and docking fail**. Fix: `sudo systemctl restart einride-mini-truck` after `System clock synchronized: yes` |
| Survey, full run | not done: battery | next |

## How the tests were run (exact commands)

Paths on the robot. Write map paths out in full; an unset `$DIR` broke the
start-up once (2026-10-08).

```bash
PKG=$HOME/ws/src/Einride-Automotive-Hackathon/einride_mini_truck_application
```

**Mapping** (2026-10-07, `room_1lap`): only odometry + mapping + teleop, not
the full app.
```bash
ros2 launch einride_mini_truck_application localization.launch.py   # terminal 1
ros2 launch einride_mini_truck_application slam.launch.py mode:=mapping   # terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.10 -p turn:=0.35   # to /cmd_vel
ros2 run tf2_ros tf2_echo map odom          # ~0 at start; after the lap = drift
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$PKG/maps/room_1lap'}"
```

**Localization, turn, collision and lap tests** (full app, SLAM against the map):
```bash
ros2 launch einride_mini_truck_application app.launch.py slam:=true mission:=false map_file:=$PKG/maps/room_1lap
ros2 lifecycle get /slam_toolbox            # active [3]
ros2 lifecycle get /collision_monitor       # active [3]
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.10 -p turn:=0.35 -r cmd_vel:=/cmd_vel_nav
ros2 run tf2_ros tf2_echo map base_footprint    # SLAM pose
ros2 run tf2_ros tf2_echo odom base_footprint   # odometry-only pose
```
- Turn test: reading, hold `j` for one full circle, `k`, reading again.
- Lap test: readings on the mark, one slow lap, stop on the mark, readings again.
- Collision test: obstacle ~5 cm in front, then behind; try `i` and `,`.

**Motor check** (bypasses Nav2 and the collision monitor):
```bash
timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
ros2 topic hz /wheel_encoders               # ~20 Hz = motor board alive
```

**Camera clock check:**
```bash
timedatectl | grep synchronized
ros2 topic echo /oak/rgb/image_raw --field header.stamp --once; date +%s
```

**Survey:**
```bash
ros2 launch einride_mini_truck_application app.launch.py mission:=false tag_survey:=true slam:=true \
  map_file:=$PKG/maps/room_1lap tag_survey_output:=$PKG/config/docks/room.yaml
ros2 service call /tag_survey/save std_srvs/srv/Trigger "{}"
```

## Raw readings

`tf2_echo` translation (x, y in m) and yaw (deg).

| Test | Frame | Start | End |
|---|---|---|---|
| Localization lap 2 (10-07) | map | (0.010, -0.100), 0.07 | (-0.026, -0.132), 7.9 |
| | odom | (0, 0), 0 | (-1.484, 0.553), 92.5 |
| Turn test (10-08) | odom | (0.000, 0.000), 0.096 | (0.157, -0.087), -15.363 |
| | map | not taken | (0.086, -0.100), 2.414 |

| Camera clock (10-08) | Image stamp | `date +%s` | Behind |
|---|---|---|---|
| Before any restart | 1791414863 | 1791416501 | 1638 s |
| 3 min after a reboot (clock synced) | 1791417338 | 1791418003 | 665 s |

## Still to test

1. Camera clock fix (restart after time sync), then the survey of A, B, C into
   `config/docks/room.yaml`.
2. Rebuild, then the full Saga run: `layout:=room slam:=true map_file:=<full path>`
   with a route that only uses A, B, C.
3. Gyro scale: 3 circles left and 3 right, odom vs map yaw.
4. In the real arena: map, survey (or check `arena.yaml`), full run.

## Recommendations for competition day (SLAM)

Based on the tests above. Provisional until the full run with `slam:=true`
has passed.

**Use `slam:=true`.** After one lap SLAM was within 5 cm / 8 deg; odometry
alone was off by metres. Nav2 still drives in `odom`; SLAM corrects where `odom`
sits on the map.

After every power-up, before anything else:
1. `uptime` (rebooted?), then wait for `timedatectl | grep synchronized` = `yes`.
2. `sudo systemctl restart einride-mini-truck`, wait 30 s.
3. `ros2 topic echo /oak/rgb/image_raw --field header.stamp --once; date +%s`:
   the two numbers must be within ~2 s. If not, restart again. Know the
   `jetson` sudo password before the day.

Building the arena map (once):
1. Tape the start spot with an arrow for the heading. It becomes the map
   origin; every run starts here.
2. Look for glass or shiny surfaces at 10-20 cm height; cover them with paper.
3. Run only odometry + mapping + teleop (not the full app). Wait for
   `Gyro bias calibrated`, check `map -> odom` is ~0, drive one slow lap
   (`speed:=0.10 turn:=0.35`), end on the mark. Never restart anything during
   mapping.
4. Pass: `map -> odom` within ~10 cm / 10 deg after the lap. Save as
   `maps/arena_<date>` and copy it off the robot as a backup.

Docks: survey the tags against that map (`docs/apriltag-map-survey.md`), or
measure that `config/docks/arena.yaml` matches the real arena.

Before every run:
1. Robot on the mark, facing the arrow.
2. Launch with the **full** map path:
   `ros2 launch einride_mini_truck_application app.launch.py layout:=<layout> slam:=true map_file:=$HOME/ws/src/Einride-Automotive-Hackathon/einride_mini_truck_application/maps/<map>`
3. Do not touch the robot until `Gyro bias calibrated`.
4. `ros2 lifecycle get /slam_toolbox` and `/collision_monitor`: both `active [3]`.
5. `tf2_echo map base_footprint`: close to the start values recorded when the
   map was built.
6. Foxglove: heavy topics off (`/oak/*`), they load the Jetson.

Fallback if SLAM misbehaves on the day: `slam:=false layout:=arena`
(odometry only; expect drift on long routes).

Known limits:
- Obstacles lower than 14 cm (chair bases, cables) are invisible to the lidar.
- The camera delivers only a few images per second to apriltag; stand still
  ~10 s per tag when surveying.
