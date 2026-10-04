# AprilTag survey against the saved SLAM map

This guide records the positions of AprilTags A–H in the coordinate system of
the saved LiDAR map. It is a **precompetition setup task**: during the survey,
you may drive the robot manually so its camera can see each tag. The resulting
file is a tag-location catalog for later use.

This survey mode does not start Saga or automatically dock, and the catalog is
not yet loaded by the mission/docking system. Competition-time use of these
saved positions still needs to be connected to that system. During the
competition, the intended behavior is for the robot to use the map and catalog
without manual driving; live tag detection can still be used for the final
docking approach.

## Before you start

- The LiDAR map must already be built and saved as the matching
  `maps/arena.posegraph` and `maps/arena.data` files on the Jetson.
- The robot must be in the area covered by that map. Localization can be
  unreliable outside the mapped area.
- The AprilTags must be IDs 0–7 (named A–H), from the `tag36h11` family. The
  black square measured by the detector must be 100 mm across.
- Make sure each tag is mounted where it will stay. The robot records the
  position of the tag itself, so moving a tag later makes its saved position
  wrong.
- The robot needs its camera and LiDAR running. Keep the robot stationary for
  about three seconds after starting the application so the gyro can settle.

## Start the survey

Use separate SSH terminals for the application, localization, and any
monitoring or teleoperation. On the Jetson, use the workspace paths below;
adjust them if this checkout lives elsewhere.

1. Start the robot and connect over SSH. Confirm the hardware service is
   running:

   ```bash
   sudo systemctl is-active einride-mini-truck
   ```

   It should print `active`. Do not start another hardware driver launch if the
   service is already running. If it prints `inactive`, start it with
   `sudo systemctl start einride-mini-truck`, then check again.

2. In the first SSH terminal, start the application with Saga missions off and
   tag survey on:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ws/install/setup.bash
   ros2 launch einride_mini_truck_application app.launch.py \
     mission:=false layout:=arena tag_survey:=true use_slam_map:=true
   ```

   Leave this running. `mission:=false` prevents Saga from issuing a docking
   mission while you are surveying. `use_slam_map:=true` avoids publishing a
   second, conflicting map-to-robot alignment.

3. In a second SSH terminal, start localization using the saved LiDAR map:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ws/install/setup.bash
   ros2 launch einride_mini_truck_application slam.launch.py \
     mode:=localization use_sim_time:=false
   ```

   Leave this running too. Wait for SLAM Toolbox to report that it has loaded
   the saved pose graph and activated. Do not start mapping mode for this
   survey; keep the previously saved map fixed while collecting tag locations.

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
   the dock camera. Pause briefly for each tag; the survey node needs at least
   five consistent observations per tag. Watch the application terminal for
   lines such as `Tag A (ID 0): ... observations`. Press **Ctrl+C** to stop
   teleoperation when you are done driving.

   You do not need to scan the tags in alphabetical order. Avoid moving the
   tags, and do not drive into unmapped rooms while collecting them.

## Save and check the catalog

When you have surveyed the tags you need, call the save service from a sourced
SSH terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
ros2 service call /tag_survey/save std_srvs/srv/Trigger "{}"
```

The default output path on the Jetson is:

```text
~/ws/src/Einride-Automotive-Hackathon/einride_mini_truck_application/config/docks/arena_tag_survey.yaml
```

The response lists which tags were saved and which tags need more observations.
The service writes the tags that have enough consistent observations, even if
some are missing. Reposition the robot to view any missing tags, wait for at
least five good observations, and call the save service again. Check the file:

```bash
cat ~/ws/src/Einride-Automotive-Hackathon/einride_mini_truck_application/config/docks/arena_tag_survey.yaml
```

It contains the map frame, tag family and printed size, and for each saved tag
its A–H name, `[x, y, yaw]` pose, and accepted observation count. The coordinates
are in metres and radians. Keep this file with the exact saved map it was
surveyed against; rebuilding or changing the map can change the coordinates.

## Stop the run

1. Stop teleoperation with **Ctrl+C** and make sure the robot is stationary.
2. Stop the SLAM localization launch with **Ctrl+C**.
3. Stop the application launch with **Ctrl+C**.
4. Leave the hardware service running if you are continuing to use the robot.
   To shut the robot down, stop the service and verify it is inactive before
   powering off:

   ```bash
   sudo systemctl stop einride-mini-truck
   sudo systemctl is-active einride-mini-truck
   sudo poweroff
   ```

Do not call SLAM Toolbox's `serialize_map` service in localization mode. The
survey produces the separate `arena_tag_survey.yaml` catalog; it does not edit
the saved `.posegraph` or `.data` map files.

## What is still needed before competition use

The survey file currently records the tag poses but does not replace the
hard-coded dock poses or feed the mission system. Before relying on it in the
competition, the application needs to load this catalog, associate Saga's
requested tag with its surveyed pose, and use that pose for navigation and
docking. Then test the complete flow on the robot with no manual driving during
the mission.
