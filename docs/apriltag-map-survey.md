# AprilTag survey against the saved SLAM map

This guide records, for each AprilTag A–H, where the robot must stand to dock
at it, in the coordinate system of the saved LiDAR map. It is a **setup task**:
during the survey you drive the robot manually so its camera can see each tag.

The result is a dock database in the same format as `config/docks/*.yaml`
(frame `map`), so it is used as a layout: `layout:=<file name> slam:=true`.
Each pose is computed exactly as live docking does it (0.326 m in front of the
tag, facing it). Live tag detection still guides the final docking approach.

## Before you start

- The LiDAR map must already be built and saved as a matching
  `.posegraph` + `.data` pair in `maps/` on the Jetson (see
  [slam-localization.md](slam-localization.md)), e.g. `maps/room_1lap`.
- The robot must be in the area covered by that map. Localization can be
  unreliable outside the mapped area.
- The AprilTags must be IDs 0–7 (named A–H), from the `tag36h11` family. The
  black square measured by the detector must be 100 mm across.
- Make sure each tag is mounted where it will stay. The robot records the
  position of the tag itself, so moving a tag later makes its saved position
  wrong.
- The robot needs its camera and LiDAR running. Keep the robot stationary
  until the log prints `Gyro bias calibrated` (about 10 s).
- The camera clock must match the system clock (next section).

## Start the survey

Use separate SSH terminals for the application and for any monitoring or
teleoperation. On the Jetson, use the workspace paths below;
adjust them if this checkout lives elsewhere.

1. Start the robot and connect over SSH. Confirm the hardware service is
   running:

   ```bash
   sudo systemctl is-active einride-mini-truck
   ```

   It should print `active`. Do not start another hardware driver launch if the
   service is already running. If it prints `inactive`, start it with
   `sudo systemctl start einride-mini-truck`, then check again.

   **Camera clock check (after every boot).** The robot has no clock battery:
   after boot the system clock jumps forward when network time arrives, but
   the camera driver keeps its old offset, so its images are stamped minutes
   in the past. The survey then logs `No map pose for tag N: Lookup would
   require extrapolation into the past` and drops every sample (docking fails
   the same way). Check:

   ```bash
   timedatectl | grep synchronized      # must say yes
   ros2 topic echo /oak/rgb/image_raw --field header.stamp --once; date +%s
   ```

   The two numbers must be within ~2 s. If not, restart the hardware service
   and check again after ~30 s:

   ```bash
   sudo systemctl restart einride-mini-truck
   ```

2. In the first SSH terminal, start the application with Saga missions off and
   tag survey on:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ws/install/setup.bash
   PKG=$HOME/ws/src/Einride-Automotive-Hackathon/einride_mini_truck_application
   ros2 launch einride_mini_truck_application app.launch.py \
     mission:=false tag_survey:=true slam:=true \
     map_file:=$PKG/maps/room_1lap tag_survey_output:=$HOME/.ros/room.yaml
   ```

   Set `PKG` in the same terminal. Always pass `map_file` (no extension):
   without it the default `maps/arena` is loaded, and a wrong path makes
   slam_toolbox fail to start (check `ros2 lifecycle get /slam_toolbox` says
   `active [3]`).

   Leave this running. `mission:=false` prevents Saga from issuing a docking
   mission while you are surveying. `slam:=true` localizes against the saved
   map (see [slam-localization.md](slam-localization.md)). Name the output
   after the place (`room.yaml`, `arena.yaml` is taken by the hand-written
   arena layout); the file name becomes the layout name.

3. Wait for `Gyro bias calibrated` and for SLAM Toolbox to report that it has
   loaded the saved pose graph and activated. Start from the spot the map was
   built from, or set the pose with "2D Pose Estimate".

4. Check that the tag detector is receiving camera images and that localization
   is publishing `/map`. You can view `/map`, `/scan`, and the robot in Foxglove
   as described in [the Foxglove guide](foxglove.md). In survey mode, tag
   detection is enabled continuously, so the Jetson may be under more load than
   during ordinary driving.

5. In another SSH terminal, start keyboard teleoperation:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ws/install/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
     --ros-args -p speed:=0.10 -p turn:=0.35 -r cmd_vel:=/cmd_vel_nav
   ```

   Drive slowly within the mapped area and bring each tag into clear view of
   the dock camera, from where the robot would approach to dock (in front of
   the tag, within ~1.5 m). Stand still ~10 s per tag: the camera delivers
   only a few images per second, and the survey node needs at least five
   consistent observations per tag. Watch the application
   terminal for lines such as `Tag A (ID 0): 10 observations, tag at (x, y)`.
   The tag position should stay put as you move; if it wanders, localization
   is off. Press **Ctrl+C** to stop
   teleoperation when you are done driving.

   You do not need to scan the tags in alphabetical order. Avoid moving the
   tags, and do not drive into unmapped rooms while collecting them.

## Save and check the result

When you have surveyed the tags you need, call the save service from a sourced
SSH terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
ros2 service call /tag_survey/save std_srvs/srv/Trigger "{}"
```

The response lists newly saved tags and every dock still missing. Each save
merges new observations into the existing catalog, so you can scan a few tags,
save, then scan the rest. The file is written under your home directory to
avoid permission problems in the ROS installation's `/config` directory.
Check it:

```bash
cat ~/.ros/room.yaml
```

The file contains `docks` for Nav2 and `tags` for the physical AprilTag poses.
`docks.*.pose` is the robot's docked pose in front of the tag;
`tags.*.pose` is the tag's actual `[x, y, yaw]` in the map frame. Keep it with
the exact map it was surveyed on; a new map needs a new survey.

To view saved tags in Foxglove, add a 3D panel, set **Fixed frame** to `map`,
expand **Topics**, enable `/tag_survey/saved_tags`, and choose
`visualization_msgs/MarkerArray`. The sphere marks the physical tag position,
the arrow shows its heading, and the label shows its ID. Only catalog entries
already saved to disk are published on this topic.

## Stop the run

1. Stop teleoperation with **Ctrl+C** and make sure the robot is stationary.
2. Stop the application launch with **Ctrl+C** (SLAM runs inside it).
3. Leave the hardware service running if you are continuing to use the robot.
   To shut the robot down, stop the service and verify it is inactive before
   powering off:

   ```bash
   sudo systemctl stop einride-mini-truck
   sudo systemctl is-active einride-mini-truck
   sudo poweroff
   ```

Do not call SLAM Toolbox's `serialize_map` service in localization mode. The
survey writes a separate dock file (`room.yaml`); it does not edit
the saved `.posegraph` or `.data` map files.

## Tuning the survey

The survey node's settings can be changed while it runs, no rebuild:

```bash
ros2 param set /tag_survey min_observations 3
```

| Parameter | Default | Change when |
|---|---|---|
| `min_observations` | 5 | a tag is seen only briefly (lower to 3) |
| `sample_period` | 0.5 s | between samples of the same tag; lower to collect faster |
| `max_position_error` | 0.30 m | samples further than this from the median are dropped; lower (0.15) if the saved dock looks off, raise if too many are dropped |
| `max_yaw_error` | 0.50 rad | the same for the dock's heading |
| `max_tf_age` | 2.0 s | tag TF older than this is ignored (busy Jetson) |

Check the result before using it: the tag markers should land on the physical
tag positions, and each dock pose should be ~0.33 m in front of its tag.

## Using the survey

Copy the saved catalog into the package's dock layouts, rebuild to install it,
then run with it as the layout:

```bash
PKG=$HOME/ws/src/Einride-Automotive-Hackathon/einride_mini_truck_application
cp ~/.ros/room.yaml "$PKG/config/docks/room.yaml"
cd ~/ws && colcon build --packages-select einride_mini_truck_application
source ~/ws/install/setup.bash
ros2 launch einride_mini_truck_application app.launch.py layout:=room slam:=true \
  map_file:=$HOME/ws/src/Einride-Automotive-Hackathon/einride_mini_truck_application/maps/room_1lap
```

The mission, camera gate and docking server read the docks' frame from the
file (`map`), so `slam:=true` is required. If Saga asks for a dock that was
not surveyed, the docking server does not know it: the mission keeps retrying
and the robot waits. In a test room with only some tags, wait for a route that
uses them.

## What is still needed before competition use

Status 2026-10-08. Results so far: [test-log.md](test-log.md).

Done (code, unit-tested on the Mac in `test/test_tag_catalog.py`):
- The survey writes docked poses as a dock database (frame `map`), loadable as
  `layout:=<name>`.
- The mission, camera gate and docking server load that file, associate
  Saga's requested tag with its surveyed dock, and take the dock frame from
  the file, so a surveyed layout works with `slam:=true`.
- `slam:=true` localizes against the saved map in the mission loop (tested on
  the robot: one lap within 5 cm / 8 deg).

Not yet tested on the robot:
- A survey run that saves a file. The first try (2026-10-08) dropped every
  sample because the camera clock was behind; see the clock check above.
- The complete flow with a surveyed layout: `layout:=room slam:=true`, a Saga
  route using only the surveyed tags, no manual driving during the mission.
- The same in the real arena: map, survey (or check `config/docks/arena.yaml`),
  full run.
