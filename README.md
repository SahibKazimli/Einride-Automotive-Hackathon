# einride-mini-truck

A ROS 2 Jazzy + Gazebo Harmonic simulation of a **six-wheel skid-steer rover**:
four driven wheels, a 360-degree lidar, a stereo depth camera, and two IMUs.

## Included packages

* `einride_mini_truck_description` - the SDF model and meshes.
* `einride_mini_truck_gazebo` - world file and Gazebo system plugins.
* `einride_mini_truck_application` - ROS 2 application code (placeholder).
* `einride_mini_truck_hardware` - the serial hardware abstraction layer, for
  running the same stack on the real robot.
* `einride_mini_truck_bringup` - launch files, bridge config, RViz config.

---

# The robot

| | |
|---|---|
| Drive | 6-wheel skid-steer, **4WD**: front and rear pairs driven, middle pair free-rolling |
| Wheels | radius 0.040 m, width 0.0425 m, track 0.17452 m, wheelbase 0.171 m |
| Mass | 3.72 kg |
| Lidar | InnoMaker LD19P, 360-degree DTOF, 0.02-12 m |
| Camera | Luxonis OAK-D Lite, colour + stereo depth |
| IMUs | chassis ICM-20948 (9-axis) and the camera's BMI270 (6-axis) |

## Drivetrain

Six wheels, four driven. The front and rear pairs take drive torque; the middle
pair are free-rolling idlers that carry load, roll at ground speed, and take no
command. Only the front wheels are instrumented with encoders.

## Lidar

InnoMaker **LD19P**, a 360-degree DTOF scanner.

| | value |
|---|---|
| range | 0.02 - 12 m |
| scan rate | 10 Hz (device supports ~5-13 Hz) |
| sample rate | 4500 points/s -> 450 per revolution |
| angular resolution | 0.8 deg |
| field of view | 360 deg |

Publishes `/scan` and `/scan/points`. If you retune the scan rate, change
`update_rate` and `samples` together to preserve the 4500 pts/s relationship.

## Camera

Luxonis **OAK-D Lite**: colour IMX214, stereo depth from an OV7251 pair on a
75 mm baseline, and an integrated BMI270 IMU.

| stream | resolution | HFOV | VFOV | DFOV | topic |
|---|---|---|---|---|---|
| colour | 4208x3120 @ 30 Hz | 69.0 | 54.0 | 81.1 | `/oak/rgb/image_raw` |
| depth | 640x480 @ 30 Hz | 73.0 | 58.1 | 85.5 | `/oak/stereo/image_raw` |

Depth range is 0.2 - 19 m. Point cloud on `/oak/points`, camera IMU on
`/oak/imu/data`.

Colour and depth share one optical frame (`oak_d_lite_link`), with depth
colour-aligned as depthai does by default, and the two mono streams are not
exposed separately.

### Matching the real camera

`depthai_ros_driver` sets resolution per socket, so hardware can be configured to
match this model. `i_resolution` accepts `400P 480P 720P 800P 1080P 1200P
1440X1080 4000x3000 5312X6000 5MP 12MP 13MP 48MP`; arbitrary sizes come from ISP
scaling. Under the `rgb` / `left` / `right` / `stereo` namespaces: `i_resolution`,
`i_fps`, `i_width`, `i_height`, `i_set_isp_scale`, `i_isp_num`, `i_isp_den`,
`i_output_isp`.

```bash
ros2 launch depthai_ros_driver camera.launch.py \
  rgb.i_resolution:=13MP rgb.i_set_isp_scale:=true \
  rgb.i_isp_num:=1 rgb.i_isp_den:=4 rgb.i_fps:=30 \
  stereo.i_resolution:=480P stereo.i_fps:=30
```

## Inertial sensing

Two independent IMUs, plus a magnetometer.

| part | where | topics | gyro sigma | accel sigma |
|---|---|---|---|---|
| **ICM-20948** (9-axis) | chassis, `base_imu_link` | `/imu`, `/mag` | 2.62e-3 rad/s | 2.26e-2 m/s^2 |
| **BMI270** (6-axis) | inside the OAK-D Lite | `/oak/imu/data` | 1.22e-3 rad/s | 1.57e-2 m/s^2 |

Sigmas are **derived** from datasheet noise densities at 100 Hz bandwidth, not
quoted sigmas: ICM-20948 gyro 0.015 deg/s/sqrt(Hz) and accel 230 ug/sqrt(Hz);
BMI270 gyro 0.007 deg/s/sqrt(Hz) and accel 160 ug/sqrt(Hz). The AK09916
magnetometer sigma of 3e-7 T is an estimate around its 0.15 uT/LSB resolution.

**The camera's BMI270 is the quieter gyro**, about 2x better than the chassis
ICM-20948, so it is the better heading source. Angular velocity is identical at
every point of a rigid body, so its offset mounting costs nothing for gyro use -
only accelerometer readings are affected by lever arm. Use the chassis unit for
acceleration and its magnetometer for absolute heading.

---

# Running the simulation

## Requirements

* ROS 2 Jazzy
* `ros-jazzy-ros-gz` and `ros-jazzy-sdformat-urdf`
* `ros-jazzy-topic-tools`
* Build tools:

    ```bash
    sudo apt install python3-colcon-common-extensions python3-vcstool python3-rosdep git wget
    ```

## Usage

1. Install dependencies

    ```bash
    cd einride_mini_truck_ws
    source /opt/ros/jazzy/setup.bash
    sudo rosdep init      # first time only
    rosdep update
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro jazzy
    ```

1. Build and source

    ```bash
    colcon build --cmake-args -DBUILD_TESTING=ON
    . install/setup.bash
    ```

1. Launch (starts running, not paused)

    ```bash
    ros2 launch einride_mini_truck_bringup simulation.launch.py
    ```

   Arguments: `headless:=true` (no GUI), `rviz:=false`, `world:=<basename>`,
   `joint_state_rate:=<Hz>` (default 50).

   `einride_mini_truck.launch.py` still works and forwards every argument - it
   is now a shim around `simulation.launch.py`, which is named for symmetry
   with `hardware.launch.py`.

## Topics

| topic | type | direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | command |
| `/odom` | `nav_msgs/Odometry` | state |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | state |
| `/joint_states` | `sensor_msgs/JointState` | all 6 wheel joints, throttled |
| `/joint_states_raw` | `sensor_msgs/JointState` | unthrottled source of the above |
| `/wheel_encoders` | `sensor_msgs/JointState` | instrumented wheels only |
| `/imu` | `sensor_msgs/Imu` | chassis ICM-20948 |
| `/mag` | `sensor_msgs/MagneticField` | AK09916 |
| `/scan`, `/scan/points` | `LaserScan`, `PointCloud2` | LD19P |
| `/oak/rgb/image_raw`, `/oak/rgb/camera_info` | `Image`, `CameraInfo` | colour |
| `/oak/stereo/image_raw`, `/oak/stereo/camera_info` | `Image` (32FC1), `CameraInfo` | depth |
| `/oak/points` | `PointCloud2` | depth cloud |
| `/oak/imu/data` | `sensor_msgs/Imu` | camera BMI270 |
| `/voltage` | `std_msgs/Float32` | battery voltage, not available in simulation |

### Subscribing to the sensor streams

`/imu`, `/mag` and `/wheel_encoders` are published **best-effort**
(`qos_profile_sensor_data`, KEEP_LAST(5)) in both simulation and on hardware.

That means that subscriptions should use best-effort as well. The default depth
argument requests RELIABLE, which is *incompatible* - you get no messages at
all and only a QoS warning in the log:

```python
from rclpy.qos import qos_profile_sensor_data

self.create_subscription(Imu, '/imu', self.cb, qos_profile_sensor_data)  # yes
self.create_subscription(Imu, '/imu', self.cb, 10)                       # silent
```

`/cmd_vel` and `/voltage` stay reliable - a dropped command leaves the MCU
holding its last velocity, and one lost 1 Hz battery reading is a whole second
of nothing.

## Driving it

```bash
# forward
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.4}}'
# rotate in place
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 1.0}}'

ros2 topic echo /odom
```

Headless without the launch file:

```bash
gz sim -s -r einride_mini_truck.sdf &
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$(ros2 pkg prefix einride_mini_truck_bringup)/share/einride_mini_truck_bringup/config/einride_mini_truck_bridge.yaml
```

---

# Running on real hardware

The same stack runs on the robot; only the layer that talks to hardware is
swapped.

```bash
ros2 launch einride_mini_truck_bringup hardware.launch.py
```

Arguments: `rviz:=false` (pass this on a headless Jetson), `lidar:=false`,
`camera:=false`, `hardware_params:=<path>`.

## Differences between hardware and simulation

| | why |
|---|---|
| ~80 Hz feedback, not 100 | At 115200 baud a ~140-byte `T:1001` line takes 12.2 ms to transmit, which caps the loop |
| Stamps are ~12 ms late | The MCU sends no timestamps. The node subtracts each line's transmission time; the residual goes in `stamp_offset`, to be measured on the robot |
| No IMU orientation | The MCU sends raw gyro and accelerometer counts, no fused attitude, so `orientation_covariance[0]` is -1 per REP-145. Simulation does provide an orientation |
| No wheel effort | `/wheel_encoders` leaves `effort` empty; the chassis has no torque sensing. Simulation's zeros are not measurements either |
| Softer command response | Serial round trip plus MCU PID, against Gazebo applying joint velocity immediately |

## Testing it without a robot

```bash
colcon test --packages-select einride_mini_truck_hardware einride_mini_truck_bringup
colcon test-result --verbose
```

* the codec as pure functions - framing across chunk boundaries, unit
  conversions, malformed and non-finite input;
* the node against a **pty** standing in for the MCU, replaying a capture: its
  real reader thread, writer thread and executor, no mocking;
* the conformance test, which launches both modes and diffs their graphs.

`einride_mini_truck_hardware/test/data/ugv02_feedback.jsonl` is synthetic,
generated to the documented protocol. Replace it with a real capture during
on-robot bring-up; see the README next to it.

---

# Simulation notes

Everything below is about the simulation rather than the robot: how the model is
tuned, what it renders like, and which messages are expected noise.

## Gazebo comes from ROS 2 Jazzy

This project does not depend on a standalone Gazebo install and does not need
`GZ_VERSION` set. Gazebo is consumed through the `gz_*_vendor` packages that ROS 2
Jazzy ships (`ros-jazzy-ros-gz`), and `einride_mini_truck_gazebo` links the
unversioned `gz-sim::gz-sim` target, so the Gazebo version follows the ROS distro.

The launch passes `--gui-config config/gui.config`, which frames the Gazebo camera
on the robot and skips the quick-start dialog. It has to be a full copy of gz-sim's
default config: a partial `<gui>` block **replaces** the default plugin set rather
than merging, which drops the entity tree, world controls, and even scene
rendering. Running `gz sim <world>.sdf` by hand will not pick it up unless you pass
the flag yourself.

Mesh URIs in `model.sdf` use `package://`, not `model://`. Both Gazebo and RViz
parse that file, and RViz cannot resolve `model://`.

## Effective track, not geometric

`wheel_separation` in the drive plugin is **0.21701**, not the geometric track of
0.17452. A skid-steer has to scrub sideways to rotate, so the ideal differential
model over-predicts yaw rate. Measured against the IMU, commanded/actual came out
at a constant **1.2435**, and `0.17452 x 1.2435 = 0.21701` makes commanded yaw
rate, reported odometry and actual rotation agree.

Re-measured after dropping from six driven wheels to four: 1.2432, unchanged
within noise. That is expected - during in-place rotation the middle wheels sit at
the centre of the wheelbase, so their contact points travel purely sideways and
they pure-scrub whether driven or not.

Re-calibrate if wheel or ground friction changes.

## Expected messages

These are harmless:

* `XML Element[gz_frame_id] ... not defined in SDF` - a Gazebo extension that
  SDFormat does not recognise but Gazebo reads. It sets the `frame_id` on sensor
  messages, and appears once per sensor per process that parses the model.
* `sdformat_urdf: link [...] has a <sensor>, but URDF does not support this` -
  cosmetic.
* `kdl_parser: The root link base_footprint has an inertia` - `base_footprint`
  carries a 1 g token inertia because Gazebo requires one per link, while KDL
  prefers a massless root. TF is published correctly either way.
