# SLAM (slam_toolbox)

Build a LiDAR map of the arena, then localise against it. slam_toolbox publishes
`map -> odom`; `odom -> base_footprint` still comes from Gazebo's DiffDrive (in
sim) or the EKF (on hardware), so SLAM only *adds* the global correction - it
does not replace the odometry underneath it.

## Install

```bash
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard
```

## In simulation: use CycloneDDS

Fast DDS's shared-memory transport corrupts easily in Docker - stale
`/dev/shm/fastrtps_*` files pile up after hard-killed processes and break node
discovery, so slam_toolbox launches but never subscribes to `/scan` and no `/map`
appears. Use CycloneDDS, set in **every** shell (sim, slam, teleop, foxglove):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp     # add to ~/.bashrc so all shells agree
```
(Install once: `sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp`.)

If `/map` never appears, it is almost always one of:
- **mixed RMW** - a shell on Fast DDS can't see one on CycloneDDS, or
- **two sims running** - `ros2 topic info /scan` must show `Publisher count: 1`.

## Mapping

On the Mac, map on Gazebo's odom (`localization:=false`); the EKF is too heavy
for the GPU-less sim. Three shells, all on the same RMW:

```bash
# 1. sim
ros2 launch einride_mini_truck_bringup simulation.launch.py headless:=true rviz:=false world:=arena
# 2. slam, mapping mode (auto-activates the lifecycle node)
ros2 launch einride_mini_truck_application slam.launch.py mode:=mapping use_sim_time:=true
# 3. drive
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive slowly around the whole arena (turn **before** walls - ramming spins the
wheels and corrupts odometry), close the loop back to the start, and watch `/map`
fill in on Foxglove (add the `/map` OccupancyGrid). When the wall outline is a
clean, closed rectangle, save it:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/einride_mini_truck_ws/src/einride_mini_truck_application/maps/arena'}"
```
That writes `maps/arena.posegraph` + `maps/arena.data`.

## Localisation ("version 4")

```bash
ros2 launch einride_mini_truck_application slam.launch.py mode:=localization use_sim_time:=true
```
It loads `maps/arena` (path set in `config/localization/slam_localisation.yaml`)
and publishes `map -> odom` by matching live `/scan` against the saved map. Point
Nav2's global frame at `map` to navigate against it.

## Notes

- slam_toolbox nodes are **lifecycle** nodes; `slam.launch.py` configures and
  activates them automatically. If started another way they do nothing until
  activated: `ros2 lifecycle set /slam_toolbox configure` then `... activate`.
- **Maps are gitignored** - the sim map is not the real arena. Rebuild one per
  environment (sim world vs. the physical arena).
- Configs live in `config/localization/`: `slam_mapping.yaml`, `slam_localisation.yaml`.
- On a GPU machine or the robot you can map on the real **EKF** instead of gz odom
  (`localization:=true`), which mirrors the hardware stack; the EKF overloads the
  Mac sim, which is why gz odom is used there.
