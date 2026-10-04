# Agent Instructions & Project Context

## Project Overview
- **Einride x KTH AI Challenge:** A mini truck that takes routes from Saga AI, drives to the dock whose AprilTag Saga names, docks, waits for load/unload, and repeats.
- **Hardware:** The real robot runs ROS 2 Jazzy on a Jetson Orin Nano. Our application code is in the `einride_mini_truck_application` package; the rest is the organizers' base.

## Working Rules
- **Environment:** ROS cannot run on the Mac, so keep decision-making logic testable in pure Python where practical and keep ROS nodes thin. 
- **Code Generation:** Prefer existing ROS packages and project patterns; avoid hardcoded values when configuration or live sensor data is available. Give paste-ready, step-by-step commands.
- **Git:** The agent never runs `git commit` or `git push`; the user handles both. On the robot, the workflow is: `git pull`, `colcon build --packages-select einride_mini_truck_application`, re-source, then `ros2 launch einride_mini_truck_application app.launch.py`.
- **Secrets:** Do not expose secrets. The Saga token lives only in the gitignored `config/saga/saga.secret.yaml`; never copy private Saga server code into this public repository.

## Verification & Testing
- **Testing Commands:** Run application Python tests from `einride_mini_truck_application/` using: `uv run --with pytest --with pyyaml --with requests python -m pytest -q test`. Lint with `flake8 --max-line-length 100`.
- **Hardware Honesty:** Be honest about what was verified. ROS hardware behavior must be checked on the Jetson; do not imply Mac-side tests prove it.
- **Robustness:** Obstacle avoidance must work with obstacles in place. The robot must route around obstacles (other robots, a bucket) and never sit still. Do not treat moving an obstacle as a fix.
- **Debugging:** Before diagnosing robot logs, confirm they came from the current code by checking their timestamps against `git log` and looking for expected log lines.

## Architecture (app.launch.py)
- **Frames & Mapping:** `arena` -> `odom` (static identity, start pose = origin) -> `base_footprint`. There is no map and no AMCL; Nav2 runs in `odom` with rolling costmaps.
- **Localization:** Uses a wheel odometry node (gyro bias calibrated at start; keep the robot still ~3 s) + `config/localization/ekf.yaml` for an EKF fusing wheel odometry and gyro.
- **Navigation (Nav2):** Configured in `config/navigation/nav2.yaml` using `navfn` (tolerance 0.3) and Regulated Pure Pursuit with collision detection. Uses rolling local (5 Hz) and global (3 Hz) costmaps from lidar with an inflation of 0.45. Safety is handled by `collision_monitor` (PolygonStop/Slow) + `laser_filters`.
- **Perception:** Uses `apriltag_ros` (tag36h11, 0.100 m, `max_hamming: 2`) on `/dock_camera/image_raw`. The `dock_pose` node relays the target tag's pose to the docking server, with a camera gate (detection only within ~1.6/1.9 m of the target dock, max 10 Hz) to save CPU.
- **Docking:** Uses `opennav_docking` with a `SimpleNonChargingDock` type named `competition_dock`. Docks are defined as `dock_<tag>` in `config/docks/<layout>.yaml` (`layout:=home|arena`). The final docked pose is 0.326 m in front of the tag.
- **Mission Layer:** `saga/` polls Saga and publishes `/saga/next_tag` (-1 = stay). The state machine (`mission.py`) cycles through IDLE, DOCKING, DOCKED, UNDOCKING, and RETRY_WAIT (2 s retry). `node.py` docks in two steps:
  1. `staging.py` picks a staging pose in front of the dock that is free on the live global costmap, and Nav2 `NavigateToPose` drives there.
  2. `DockRobot` executes with `nav_to_dock=False` (tag-guided approach from the staging pose).