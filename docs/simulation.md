# Simulation notes

This page covers the simulation only: how the model is tuned and which log
messages are harmless.

## Gazebo comes from ROS 2 Jazzy

No separate Gazebo install is needed, and `GZ_VERSION` does not need to be set.
Gazebo comes from the `gz_*_vendor` packages in ROS 2 Jazzy (`ros-jazzy-ros-gz`).
`einride_mini_truck_gazebo` links the unversioned `gz-sim::gz-sim` target, so the
Gazebo version follows the ROS version.

The launch file passes `--gui-config config/gui.config`, which points the Gazebo
camera at the robot and skips the quick-start dialog. This file must be a full
copy of gz-sim's default config: a partial `<gui>` block **replaces** all the
default plugins instead of adding to them, which removes the entity tree, the
world controls and even scene rendering. If you run `gz sim <world>.sdf` by hand,
pass the flag yourself.

Mesh paths in `model.sdf` use `package://`, not `model://`. Both Gazebo and RViz
read this file, and RViz cannot resolve `model://`.

## Running headless without the launch file

```bash
gz sim -s -r einride_mini_truck.sdf &
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$(ros2 pkg prefix einride_mini_truck_bringup)/share/einride_mini_truck_bringup/config/einride_mini_truck_bridge.yaml
```

## Effective track width

The drive plugin uses `wheel_separation` **0.21701**, not the real track of
0.17452. A skid-steer robot has to slide its wheels sideways to turn, so it turns
more slowly than a simple differential-drive model predicts. Measured against
the IMU, commanded / actual turn rate is a constant **1.2435**, and
`0.17452 x 1.2435 = 0.21701`. With this value, commanded turn rate, odometry and
actual rotation agree.

The middle wheels do not affect this. During a turn in place they sit at the
centre of the wheelbase and only slide sideways, whether driven or not.

Measure it again if wheel or ground friction changes.

## Expected messages

These messages are harmless:

* `XML Element[gz_frame_id] ... not defined in SDF` - a Gazebo extension that
  SDFormat does not know about. Gazebo uses it to set the `frame_id` on sensor
  messages. It appears once per sensor, per process that reads the model.
* `sdformat_urdf: link [...] has a <sensor>, but URDF does not support this` -
  logged by the build during SDF-to-URDF conversion. See
  [`robot_description` is URDF, generated from SDF](foxglove.md#robot_description-is-urdf-generated-from-sdf).
* `kdl_parser: The root link base_footprint has an inertia` - `base_footprint`
  has a tiny 1 g inertia because Gazebo needs one on every link. TF is published
  correctly.
