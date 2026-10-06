# TODO

Finals: 2026-10-09. Ordered by priority: making the robot finish deliveries
reliably comes first. Sources: CLAUDE.md status, the SLAM/survey handover
(2026-10-07), our `slam:=true` change.

## Must do

1. **Collision monitor blocks reversing away from a close obstacle.**
   `PolygonStop` (config/safety/collision_monitor.yaml) surrounds the robot, so
   anything inside it blocks every command, including retreat. Fix: make it a
   `velocity_polygon` (front half when driving forward, back half when
   reversing, full box when turning in place). Do not disable the monitor.
   Test with teleop: obstacle in front -> reverse works, forward blocked; same
   behind. Watch `/collision_monitor_state`.
2. **Test 6 without SLAM**: full Saga loop, bucket in the way. The baseline for
   the finals. Last run: the tag was not where home.yaml says; measure the
   screen position against home.yaml first.
3. **Test SLAM localization** (`slam:=true`), stages 1-5 in
   docs/slam-localization.md: build map, map loads, start pose, drift
   corrected, lift-and-move. Code written, not yet run on the robot.
4. **Test 6 with `slam:=true`.** Use SLAM at the finals only if it is at least
   as good as without.

## Should do

5. **Dock positions for the real arena.** Either confirm config/docks/arena.yaml
   matches the real arena, or finish the survey path:
   - the survey saves the tag frame's raw yaw (tag_survey.py), which is not the
     direction the tag faces on the floor; use `tag_normal`/`docked_pose`
     (perception/dock_pose.py) instead;
   - write the result in the docks-file format (frame `map`), keep the tag pose
     next to it for reference;
   - move the logic out of the ROS node into a pure module with pytest tests.
6. **Arena day**: tape the start spot, build the arena map from it, set
   start_x/start_y/start_yaw, run the survey if needed, one full test run.

## Nice to have

7. Camera reaches apriltag at ~0.7 Hz (CPU). Lower load or move detection to
   the GPU (`isaac_ros_apriltag`).
8. Tidy the SLAM/survey guides; update the status in CLAUDE.md.

## After the finals

- Nav2 global frame in `map` (planner + global costmap); local stays in `odom`.
- Staging grid math -> nav2_simple_commander `PyCostmap2D` +
  `FootprintCollisionChecker`; derive distances from `staging_x_offset`.
- apriltag_node segfault on shutdown (harmless).

## Improvement ideas

- **Tags as localization landmarks (only if SLAM shows problems).** With
  surveyed tag poses, every tag sighting tells the robot where it is. Feed that
  to SLAM (`/initialpose`) or the EKF. SLAM localization already corrects drift,
  so this is a backup for where SLAM is weak: the square arena looks alike from
  several sides, other robots hide the walls from the 14 cm-high lidar, and a
  lost robot. Tags have unique IDs, so they cannot be confused.
- **Stuck watchdog.** If the robot has not moved for N seconds with a mission
  active, escalate: back up, re-stage from another free pose, re-plan. Enforces
  "never sit still" for cases nobody thought of.
- **Self-correcting dock poses.** After each successful dock, store the
  measured dock pose (from the tag) and use it next time; the dock list
  improves while driving.
- **Docking accuracy log.** Log the final gap to the tag after each dock, to see
  how often we hit the 200 +-50 mm scoring window and tune from data.
- **Run report.** A script that turns a launch log into a timeline (route,
  staging, docked, retries, time per leg) to compare runs quickly.
- **Mission dashboard** in Foxglove: mission state, Saga route, target tag,
  SLAM map with robot pose, collision monitor state.
