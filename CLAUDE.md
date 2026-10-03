# Project notes for Claude

Einride x KTH AI Challenge: a mini truck that takes routes from Saga AI, drives
to the dock whose AprilTag Saga names, docks, waits for load/unload, repeats.
Finals: 2026-10-09. ROS 2 Jazzy on a Jetson Orin Nano. Our code is the
`einride_mini_truck_application` package; the rest is the organisers' base.

## How we work
- Claude edits on the Mac. **Claude never runs `git commit` / `git push`**; the
  user commits and pushes. On the robot: `git pull`, then
  `colcon build --packages-select einride_mini_truck_application`, re-source,
  relaunch `ros2 launch einride_mini_truck_application app.launch.py`.
- Give paste-ready, step-by-step commands. Teach each subsystem through a test.
- Prefer existing ROS packages over hand-written code; avoid hardcoded values.
- The repo is **public**: the Saga token lives only in
  `config/saga/saga.secret.yaml` (gitignored). The Saga server is private and
  must not be copied into this repo.
- Robustness is the bar: the robot must route around obstacles (other robots,
  a bucket) and never sit still. Test honestly, with obstacles in the way;
  "move the obstacle" is not a fix.
- ROS cannot run on the Mac. Logic lives in pure Python modules (no ROS
  imports) with pytest tests; nodes are thin wrappers. Run tests from
  `einride_mini_truck_application/`:
  `uv run --with pytest --with pyyaml --with requests python -m pytest -q test`
  and `flake8 --max-line-length 100`.
- Before diagnosing a robot log, check it came from the current code (log
  timestamp vs. `git log`, and the expected log lines are present).

## Architecture (app.launch.py)
- **Frames**: arena -> odom (static identity, start pose = origin) ->
  base_footprint (robot_localization EKF from wheel odometry + gyro). No map,
  no AMCL; Nav2 runs in `odom` with rolling costmaps.
- **localization/**: wheel odometry node (gyro bias calibrated at start; keep
  the robot still ~3 s) + `config/localization/ekf.yaml`.
- **perception/**: apriltag_ros (tag36h11, 0.100 m, `max_hamming: 2`) on
  `/dock_camera/image_raw`; `dock_pose` node relays the target tag's pose to
  the docking server, with a camera gate (detection only on within ~1.6/1.9 m
  of the target dock, max 10 Hz) to save CPU.
- **Nav2** (`config/navigation/nav2.yaml`): navfn (tolerance 0.3), Regulated
  Pure Pursuit with collision detection, rolling local (5 Hz) and global
  (3 Hz) costmaps from lidar, inflation 0.45, `always_send_full_costmap`.
  `collision_monitor` (PolygonStop/Slow) + `laser_filters` for safety.
- **Docking**: opennav_docking, `SimpleNonChargingDock` type
  `competition_dock`, docks `dock_<tag>` in `config/docks/<layout>.yaml`
  (`layout:=home|arena`). Docked pose = 0.326 m in front of the tag.
- **saga/**: polls Saga, publishes `/saga/next_tag` (-1 = stay).
- **mission/**: `mission.py` state machine (IDLE, DOCKING, DOCKED, UNDOCKING,
  RETRY_WAIT with 2 s retry). `node.py` docks in two steps:
  1. `staging.py` picks a staging pose in front of the dock that is free on the
     live global costmap (robot fits, lane to the dock clear), Nav2
     NavigateToPose drives there around obstacles;
  2. DockRobot with `nav_to_dock=False` (tag-guided approach from there).
  No free pose or no costmap yet -> wait and retry. Expected log line:
  `Staging for tag N at (x, y, deg)`.

## Home test layout (`config/docks/home.yaml`)
- Spot 1: tag 2.0 m straight ahead of the start, facing the robot -> even
  tags (A, C, E, G = 0, 2, 4, 6).
- Spot 2: tag 0.42 m ahead, 1.0 m left, facing -y -> odd tags.
- The tag can be shown fullscreen on a laptop (turn brightness down vs glare).

## Status (2026-10-03)
- Tests 1-5 done: odometry, Nav2 driving, obstacle avoidance, tag detection,
  docking.
- **Test 6 (full Saga loop) in progress**: route 5 = dock at G (tag 6), load,
  dock at C (tag 2), unload. Bucket left ~1 m ahead on purpose.
  - Run at 16:56 used code from before the two-step staging commit
    (b6ebfa2): the docking server's fixed staging pose (0.97, 0) was inside
    the bucket, so Nav2 aborted and looped on recoveries (spin/backup).
  - Next: run with b6ebfa2+ on the robot, check the `Staging for tag 6` line,
    then while docked: Saga `arrived_at_source`, `loading_complete`, switch the
    screen to tag 2, then `arrived_at_destination`, `unloading_complete`.
- Later / open:
  - Derive staging candidate distances from `staging_x_offset` instead of
    hardcoding.
  - Camera images only reach apriltag at ~0.7 Hz (CPU); reduce load, or move
    detection to the GPU (NVIDIA `isaac_ros_apriltag`; big install, check
    Jazzy/JetPack support first).
  - Run at 17:30: staging around the bucket worked, but the tag was not where
    home.yaml says (2.0 m straight ahead), so it was never seen from the
    staging pose. Failed staging poses are now skipped on retry.
  - apriltag_node segfaults on shutdown (harmless).
  - SLAM / full-arena localization (a teammate's area).
