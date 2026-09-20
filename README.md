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

| | value | source |
|---|---|---|
| range | 0.02 - 12 m | datasheet v1.0, at 70% target reflectivity |
| scan rate | 9.89 Hz | measured; datasheet says 10 Hz typical, 5-13 Hz in spec |
| sample rate | 4500 points/s, fixed | datasheet |
| points per revolution | 455 | 4500 / 9.89, i.e. a consequence of the spin rate |
| angular resolution | 0.79 deg | 360 / 455 |
| field of view | 360 deg | |

Publishes `/scan` and `/scan/points`. The device samples at a fixed 4500 Hz, so
points per revolution follow the spin rate rather than being a setting; if you
retune the spin, set `samples` in `model.sdf` and `bins` in
`config/ldlidar.yaml` to 4500 / rate together.

### Matching the real LiDAR

Simulation reproduces the `/scan` the real driver publishes, field for field.
This was established by running both and comparing, not by reading datasheets,
because most of the gaps were in places a datasheet does not describe. Each row
was a real difference that has since been closed:

| field | real LD19 | Gazebo, before | how it is matched now |
|---|---|---|---|
| `angle_min` / `angle_max` | `0` .. `2*pi` | `-pi` .. `+pi` | `model.sdf` sweep changed to 0..2*pi |
| bins | 455 | 450 | `model.sdf` `<samples>` |
| `angle_increment` | 0.013840 | 0.013994 | follows from the two above |
| no return | `NaN` | `+inf` | `ld19_scan_model` |
| `intensities` | 7..255, `NaN` where blank | all `0.0` | `ld19_scan_model` |
| `scan_time` | ~0.1009 s | `0.0` | `ld19_scan_model` |
| `time_increment` | 2.2e-4 s | `0.0` | `ld19_scan_model` |
| `range_min` / `range_max` | driver default 0.03 / 15.0 | 0.02 / 12.0 | `config/ldlidar.yaml`, which overrides the driver to the datasheet's 0.02 / 12.0 |
| `frame_id` | driver default `ldlidar_link` | `base_lidar_link` | `config/ldlidar.yaml` |

Two of these are worth dwelling on, because both fail silently.

**The sweep runs 0..2\*pi, not -pi..+pi.** `ldlidar_component` hardcodes it.
Index 0 is straight ahead, not behind. Nothing errors if you get this wrong -
every bearing is simply rotated by 180 degrees between simulation and the robot.

**A bin with no return is `NaN` on the robot and was `+inf` in Gazebo.** Both
are legal in a `LaserScan`. `r < scan.range_max` therefore accepted every empty
bearing in simulation and rejected every one on the robot, and `isnan` did the
reverse. About 17% of bins come back blank indoors, so this is not a corner
case. `ld19_scan_model` is the node that closes it - see its docstring.

`intensity` is the one place simulation does not tell the truth, and it says so:
Gazebo does not model return strength at all, so `ld19_scan_model` writes a
constant (`scan_intensity:=200.0`, inside the device's observed 7..255) on bins
that returned something, and `NaN` on bins that did not. The valid/blank
*structure* matches the robot; the magnitude means nothing. Pass
`scan_intensity:=0.0` for raw Gazebo behaviour.

To re-measure any of this, compare the two directly:

```bash
ros2 launch einride_mini_truck_bringup simulation.launch.py headless:=true rviz:=false
# against
ros2 launch einride_mini_truck_bringup hardware.launch.py rviz:=false camera:=false
# then, in both:
ros2 topic echo /scan --once --field angle_min
ros2 topic hz /scan
```

The numbers above are asserted by two tests, so they stay true:
`einride_mini_truck_gazebo`'s `test_ld19_scan_model.py` (no hardware needed) and
`einride_mini_truck_bringup`'s `test_lidar_scan_contract.py` (skips itself
without a lidar attached).

## Camera

Luxonis **OAK-D Lite**: colour IMX214, stereo depth from an OV7251 pair on a
75 mm baseline, and an integrated BMI270 IMU.

Every figure in this section was measured on the physical camera over USB, not
taken from the datasheet; where the two disagree the datasheet value is called
out.

| stream | resolution | HFOV | VFOV | DFOV | encoding | topic |
|---|---|---|---|---|---|---|
| colour | 1280x720 @ 30 Hz | 69.85 | 42.90 | 77.41 | `bgr8` | `/oak/rgb/image_raw` |
| depth | 1280x720 @ 30 Hz | 69.85 | 42.90 | 77.41 | `16UC1` mm | `/oak/stereo/image_raw` |

Depth range is **0.2 - 10 m**, clamped by the ROBOTICS preset (see below); the
sensor's own optical limit is 19 m. Point cloud on `/oak/points`, camera IMU on
`/oak/imu/data`.

Colour and depth share one optical frame, `oak_rgb_camera_optical_frame`. Depth
is colour-aligned as depthai does by default, which means the published depth
image is **colour-sized and carries the colour intrinsics** - the mono pair's own
640x480 and ~70.4 degree field never reach a topic. The two mono streams are not
exposed separately.

### Does the OAK-D Lite have an IMU?

**Yes, on this unit: a Bosch BMI270, and it works.** This is worth stating
because the OAK-D Lite is widely documented as the OAK-D variant *without* an
IMU. Three independent confirmations, all against the real device:

* `dai::Device::getConnectedIMU()` returns `BMI270`, firmware 1.0.0;
* it streams: 994 accel+gyro packets in 10 s at a requested 100 Hz, with a
  sensible gravity vector;
* the factory EEPROM carries IMU-to-colour-camera extrinsics, which
  `model.sdf` now uses to place `oak_imu_frame`.

Measured ceiling for the merged accel+gyro stream is **~250 Hz** - requesting
400 or 500 Hz still delivers 250. The driver default is 400, so
`oak_d_lite.yaml` pins it to 100 to match the simulated sensor.

Do not assume another OAK-D Lite has one. Check with `getConnectedIMU()` before
relying on `/oak/imu/data`, and treat an empty return as "no IMU".

### Matching the real camera

Two hard constraints, both found by testing against the device.

**1. Depth alignment forces both output dimensions to be multiples of 16.** The
obvious configuration - the full 4208x3120 sensor ISP-scaled by 1/4 to 1052x780,
preserving the native 1.3487 aspect and so the full 69/54 degree field - is
rejected outright:

```
ISP scaling with num: 1 and den: 4 results in width: 1052 and height: 780
which are not divisible by 16.
StereoDepth: Custom disparity/depth width must be multiple of 16.
```

and no ISP fraction can satisfy it: 4208 = 16*263 and 3120 = 16*195, so
`num/den` lands on a multiple of 16 only when `den` divides `num`, i.e. never
for a downscale. 13MP is usable only at full resolution, far beyond the USB
budget. So the choice is aligned depth *or* the full vertical field, not both.

This project takes aligned depth, at **1080P ISP-scaled by 2/3 to 1280x720**.
1080P is a full-width, vertically cropped readout: HFOV stays at the full-sensor
69.85 degrees and only the vertical field shrinks, 54.76 -> 42.90. With the
camera 0.10 m up looking level, that costs about 6 cm of near-ground visibility
- the first ground return moves from 0.20 m to 0.26 m ahead - and buys colour
and depth sharing one resolution, one set of intrinsics and one optical frame.
1920x1080 scaled by 2/3 is also the neat way through the multiple-of-16 rule:
1280 and 720 both divide by 16, where 1080 does not.

**Do not ask for `720P`.** The IMX214 has no 720P sensor mode, so the driver
logs `Resolution 720P not supported by sensor IMX214. Using default resolution
1080P` on every boot and falls back to exactly the configuration above. The
output is correct either way - the fallback lands where `i_width`/`i_height`
were pointing anyway - but a config that only works through a fallback path is
one that misdescribes the sensor, and it buries a real warning in the service
journal.

**2. `i_width`/`i_height` drive `camera_info`, and nothing cross-checks them.**
Leave `stereo.i_width`/`i_height` at their 1280x720 default while the depth
image is actually 640x480 and the driver will publish a `camera_info` that
simply does not describe the image next to it. Both blocks in
`oak_d_lite.yaml` state their size explicitly for this reason.

Also worth knowing: `i_resolution` accepts only `400P 480P 720P 800P 1080P 1200P
1440X1080 5312X6000 12MP 13MP 48MP` - and of those the IMX214 rejects `720P`,
`800P` and `1200P` alike, all falling back to 1080P. The colour sensor also caps
13MP at **28.86 fps**, not 30.

**`camera.i_enable_imu` moved, and two builds share one version number.**
depthai_ros_driver 2.12.2 as built 2026-09-11 declares no `camera.i_enable_imu`
at all - it reads `pipeline_gen.i_enable_imu` - while the 2026-09-03 build of
the same 2.12.2 declares both and acts on the former. On the newer build the
config's `camera.i_enable_imu: true` is therefore a silent no-op, and the IMU
runs only because `pipeline_gen.i_enable_imu` happens to default to true.

`oak_d_lite.yaml` sets **both**, since an unknown key in a params file is
ignored rather than rejected. That is defence against the default changing, not
a fix for an observed failure: the IMU was seen advertised-but-silent once on
the robot and could not be reproduced afterwards across repeated probes out to
7.5 minutes of uptime, on either key. If it recurs, check `ros2 param list`
against your build's namespace before suspecting the hardware - but note that a
one-off silent IMU has been seen and not yet explained.

The full configuration lives in
[`einride_mini_truck_bringup/config/oak_d_lite.yaml`](einride_mini_truck_bringup/config/oak_d_lite.yaml),
which `hardware.launch.py` passes to the driver.

### Depth filtering: the ROBOTICS preset

`stereo.i_depth_preset: ROBOTICS` - Luxonis' own preset for "navigation and
obstacle detection, without motion blur". It is not the default and it matters
more than it looks.

Left alone the driver runs `HIGH_ACCURACY`, whose threshold filter has
`maxRange` 65535 mm and whose spatial, speckle and temporal filters are all off.
On this camera that publishes depth up to **65.535 m** indoors - uint16
saturation, and exactly the number measured - with roughly 1 valid pixel in 1000
beyond 50 m, on a sensor whose usable range ends at 19. That is noise published
as measurement.

| | `HIGH_ACCURACY` (default) | `ROBOTICS` |
|---|---|---|
| median | KERNEL_5x5 | KERNEL_7x7 |
| spatial filter | off | on, delta 20, hole-fill 2 |
| speckle filter | off | on, range 200 |
| temporal filter | off | off |
| threshold filter | 0 - 65535 mm | **0 - 10000 mm** |
| decimation | 1 | 2 |
| confidence threshold | 200 | 245 |
| LR check / subpixel | on / off | on / on, 3 bits |
| **measured valid pixels** | **64.4%** | **59.3%** |
| **measured max depth** | **65.53 m** | **9.36 m** |
| **measured p99.9 depth** | **54.28 m** | **6.08 m** |

Five points of fill rate buys the disappearance of every phantom return. The
arena diagonal is 8.5 m, so the 10 m clamp costs nothing here.

Its `decimationFactor=2` does **not** change the published size - depth still
arrives colour-aligned at 1280x720 - but disparity is computed at half
resolution and upsampled, so fine detail is coarser than the pixel count
suggests.

Note the confidence threshold column: the Luxonis page describing this preset
says ROBOTICS uses 15. The library disagrees - depthai 2.30's own preset sets
**245**, read back from `initialConfig` - and the driver overrides it anyway.
Trust the device, not the documentation.

#### Two settings the driver applies on top of the preset

The preset is applied first, then the driver writes its own parameters over it.
Two of them land on values the preset had already set, so a bare
`i_depth_preset: ROBOTICS` does **not** give you ROBOTICS:

| | preset wants | driver default | config pins |
|---|---|---|---|
| `i_depth_filter_size` | 7 (KERNEL_7x7) | 5 | **7** |
| `i_stereo_conf_threshold` | 245 | 240 | **245** |

Everything else came through intact. This is not cosmetic: restoring the 7x7
median cut measured noise beyond 4 m from 372 mm to 150 mm.

#### Verifying the filters are actually on

`ros2 param get` **cannot** tell you. The preset is applied below the ROS
parameter layer, so `i_stereo_conf_threshold` and every `i_enable_*_filter` read
identically under any preset - and those enable flags are enable-ONLY: leaving
them `false` does not switch off what the preset turned on.

Ask the device instead. `ros2 launch` has no way to inject a node parameter, and
neither launch file exposes this one, so add it to the `camera:` block of
`oak_d_lite.yaml` for the run:

```yaml
    camera:
      i_pipeline_dump: true      # temporary; writes /tmp/<device-id>_pipeline.json
```

```bash
ros2 launch einride_mini_truck_bringup hardware.launch.py lidar:=false rviz:=false
python3 -c "import json;d=json.load(open('/tmp/<device-id>_pipeline.json'));\
print([n[1]['properties']['initialConfig']['postProcessing'] \
for n in d['pipeline']['nodes'] if 'StereoDepth' in n[1]['name']])"
```

That is the
exact StereoDepth configuration sent to the device. Verified for this config:
all fifteen fields match depthai's own ROBOTICS preset.

#### Measured depth noise

Per-pixel temporal stddev on a static scene, with the configuration above:

| range | median stddev |
|---|---|
| 1 - 2 m | 13.5 mm |
| 2 - 3 m | 33.1 mm |
| 3 - 4 m | 32.7 mm |
| 4 - 6 m | 150 mm |
| 6 - 10 m | 633 mm |

Pooled over the 1-4 m working band the median is **31.1 mm**, implying
**k = 3.33 mm/m^2** for error growing as z^2; two independent runs agreed to
0.4 mm. Past 4 m it degrades sharply, so treat 10 m as where numbers stop
arriving, not where they stop being usable.

Use the median, not the mean - the mean over the same band is 69 mm, inflated by
a heavy tail at occlusion boundaries that a Gaussian cannot represent anyway.

`model.sdf` declares k at a 3 m reference, `stddev` 0.030; simulation measures
30.1 mm against the hardware's 31.1 with the same script.

### Which depthai stack

**Use the v2 driver, `ros-jazzy-depthai-ros-driver`.** The v3 stack
(`ros-jazzy-depthai-ros-driver-v3`, depthai 3.9.0) **cannot drive this camera's
stereo pair at all**: starting either OV7251, with or without depth, crashes the
device firmware within seconds -

```
RTEMS_FATAL_SOURCE_INVALID_HEAP_FREE   thread CBTH
```

- and the host then loops on `X_LINK_ERROR` / reconnect. Colour and the IMU are
fine under v3; only the mono path fails. It is a firmware regression, not a
broken camera: the identical hardware streams left, right and depth at 30 Hz
under depthai v2. The two ROS packages install side by side, so having v3
present is harmless as long as the launch file asks for v2.

`hardware.launch.py` starts the driver through
**`camera_as_part_of_a_robot.launch.py`**, not the usual `camera.launch.py`. The
former publishes no TF and no camera URDF, which is what a camera bolted to a
robot that already has a description needs; `camera.launch.py` brings its own
`robot_state_publisher` and its own `oak_*` frames, which would collide with the
ones `model.sdf` declares. Frame names are unchanged either way, so `model.sdf`
owns the TF tree in both simulation and hardware.

## Inertial sensing

Two independent IMUs, plus a magnetometer.

| part | where | frame | topics | gyro sigma | accel sigma |
|---|---|---|---|---|---|
| **ICM-20948** (9-axis) | chassis | `base_imu_link` | `/imu`, `/mag` | 2.62e-3 rad/s | 2.26e-2 m/s^2 |
| **BMI270** (6-axis) | inside the OAK-D Lite | `oak_imu_frame` | `/oak/imu/data` | 1.59e-3 rad/s | 2.68e-2 m/s^2 |

The chassis sigmas are **derived** from datasheet noise densities at 100 Hz
bandwidth, not quoted sigmas: ICM-20948 gyro 0.015 deg/s/sqrt(Hz) and accel
230 ug/sqrt(Hz). The AK09916 magnetometer sigma of 3e-7 T is an estimate around
its 0.15 uT/LSB resolution.

The BMI270 sigmas are **measured on the real device**: 2002 samples at 100 Hz at
rest gave per-axis stddevs of 1.48e-3 / 0.88e-3 / 2.16e-3 rad/s and
3.62e-2 / 1.01e-2 / 2.71e-2 m/s^2, and the table carries the RMS across axes.
They run about 1.3x (gyro) and 1.7x (accel) above what the datasheet densities
predict, and they are an upper bound, since a bench measurement also picks up
ambient vibration.

**The camera's BMI270 is still the quieter gyro**, about 1.6x better than the
chassis ICM-20948, so it remains the better heading source - though by less than
the datasheet-derived figures suggested. Angular velocity is identical at every
point of a rigid body, so its offset mounting costs nothing for gyro use - only
accelerometer readings are affected by lever arm. Use the chassis unit for
acceleration and its magnetometer for absolute heading.

### The two IMUs do not share an axis convention

`/imu` is in `base_imu_link`, the usual x-forward / y-left / z-up body frame.
`/oak/imu/data` is in `oak_imu_frame`, which is the BMI270's own axes: the
depthai driver publishes raw device values without rotating them into any ROS
convention. Placed by the camera's factory extrinsics, that works out to
**imu_x = right, imu_y = forward, imu_z = up** when the robot is level, so a
stationary robot reads gravity on `+z` of `/oak/imu/data` and on `+z` of `/imu`,
but the horizontal axes are swapped and one is negated relative to the chassis
unit.

Simulation reproduces this rather than papering over it: `model.sdf` places
`oak_imu_frame` at the extrinsics read back over USB with
`getImuToCameraExtrinsics(CAM_A)` - 31.7 mm left of the colour camera, 2.2 mm up,
7.1 mm behind, rotated 89.3 degrees about the optical x axis. Fuse the two only
through TF, never by adding their vectors.

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
| `/oak/rgb/image_raw`, `/oak/rgb/camera_info` | `Image`, `CameraInfo` | colour, 1280x720 (`bgr8` on hardware, `rgb8` in simulation) |
| `/oak/stereo/image_raw`, `/oak/stereo/camera_info` | `Image`, `CameraInfo` | depth, 1280x720 (`16UC1` mm on hardware, `32FC1` m in simulation) |
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

The three device drivers are not ROS dependencies of the simulation, so install
them on the robot:

```bash
# OAK-D Lite. Must be the v2 package - see "Which depthai stack" above.
sudo apt install ros-jazzy-depthai-ros-driver
# derives /scan/points from /scan in both modes
sudo apt install ros-jazzy-pointcloud-to-laserscan
# the lidar driver's lifecycle manager
sudo apt install ros-jazzy-nav2-lifecycle-manager
```

### The chassis serial link

The UGV02's MCU hangs off a CH343 USB-serial bridge that presents itself as
CDC-ACM, so it enumerates as `/dev/ttyACM0`, `/dev/ttyACM1`, ... depending on
what else was plugged in first. `hardware.yaml` therefore names the port as
`/dev/ugv02`, and `udev/99-einride-ugv02.rules` in this package is what makes
that name exist. `provision.sh` installs it on the robot along with the other
two; on a development machine, install it by hand:

```bash
sudo install -m 0644 einride_mini_truck_bringup/udev/99-einride-ugv02.rules \
  /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/ugv02          # -> ../ttyACM0
```

Opening it also needs `dialout` membership, which `provision.sh` grants on the
robot and nothing grants on a laptop. Without it the node starts, retries and
logs the same line forever, which reads like broken hardware:

```
[ugv02_serial_node]: cannot open /dev/ugv02: [Errno 13] Permission denied
```

```bash
sudo usermod -aG dialout $USER     # then log out and back in, or: newgrp dialout
```

### The LiDAR driver

Not a released ROS package, so it is built from source **into this workspace**,
beside `einride-mini-truck`:

```bash
cd <workspace>/src
git clone --recursive https://github.com/Myzhar/ldrobot-lidar-ros2.git
cd .. && colcon build && . install/setup.bash
```

`--recursive` matters: the vendor SDK is a submodule, and without it the build
fails on missing headers.

It has to be in *this* workspace, not a second one you source alongside. A
separate workspace appears to work, because colcon records whatever was on
`AMENT_PREFIX_PATH` at build time as a prefix chain in `install/setup.bash` - so
the package resolves until someone rebuilds from a clean shell, at which point
the chain quietly disappears and the lidar stops loading.

Two upstream defects have to be patched after cloning. Both are already applied
to the copy in this workspace; both produce a *silent* failure, so they are
worth knowing about rather than rediscovering:

1. **The vendor SDK is built but never installed.** `ldlidar_component`'s
   `CMakeLists.txt` builds `libldlidar.so` as a shared library and its
   `install(TARGETS ...)` lists only the component, so the installed
   `libldlidar_component.so` has a dependency nothing can resolve. Add `ldlidar`
   to that install rule. Without it the container's `dlopen` fails with
   `libldlidar.so: cannot open shared object file` and the node never loads.
   This one can hide for a long time: if CMake keeps the build-tree `RUNPATH` on
   the installed library, the SDK resolves out of `build/` and everything works
   - until the build directory is deleted or `install/` is copied to the robot.
2. **The SDK omits `#include <pthread.h>`** in its Linux branch, so the build
   fails outright until it is force-included.

### When the lidar produces nothing

The failure mode to recognise, because the real error scrolls past and then gets
buried under two-second INFO spam:

```
[lidar_lifecycle_manager]: Waiting for service ldlidar_node/get_state...
```

That means `ldlidar_node` does not exist at all - the container started but the
component never loaded into it, so there is no lifecycle service to talk to.
It is never a lidar problem; it is always a loading problem. Scroll up past the
spam to the `[ldlidar_container] [ERROR]` line, which says which:

| error above the spam | cause |
|---|---|
| `Could not find requested resource in ament index` | `ldlidar_component` is not on `AMENT_PREFIX_PATH` - wrong workspace, or not sourced |
| `libldlidar.so: cannot open shared object file` | defect 1 above - the SDK was not installed |

A driver that *did* load but cannot reach the device fails differently and more
honestly: `Failed to change state for node: ldlidar_node`, from the lifecycle
manager, after the driver logs the serial port it could not open.

`hardware.launch.py` needs `ldlidar_component` from that repo - the library that
holds the node. It does **not** use the repo's `ldlidar` metapackage, which
carries no code and no launch files, nor its `ldlidar_bringup.launch.py`, which
would start a second `robot_state_publisher` with the bare lidar's own URDF and
fight this project's over `/robot_description` and `/tf`. The component is loaded
into a container here instead, with `config/ldlidar.yaml` for parameters.

The device needs a udev rule so it is reachable as `/dev/ldlidar` regardless of
which `ttyUSB*` it enumerated as, which the driver's repo ships:

```bash
cd <workspace>/src/ldrobot-lidar-ros2 && ./scripts/create_udev_rules.sh
```

Two things about this driver are worth knowing before you debug it:

* **It is a lifecycle node.** Left alone it reaches `unconfigured` and stops -
  no serial port, no `/scan`, and no error either. `hardware.launch.py` runs a
  `nav2_lifecycle_manager` to configure and activate it.
* **It only reads the device while something is subscribed**, and it checks that
  by the *unresolved* name `~/scan`. So remapping its publisher onto `/scan` -
  the obvious thing to do - stops it reading: the publisher moves, the
  subscriber check does not, and the node sits `active` and advertising `/scan`
  while publishing nothing and logging nothing. `hardware.launch.py` leaves the
  publisher on `/ldlidar_node/scan` and runs a `topic_tools relay` onto `/scan`,
  whose subscription also keeps the read loop fed.

A quick check that the lidar is alive rather than merely advertising:

```bash
ros2 topic hz /scan                    # ~9.9 Hz
```

The camera needs a udev rule to be reachable as a non-root user, or the driver
will not find a device that `lsusb` clearly shows:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

```bash
ros2 launch einride_mini_truck_bringup hardware.launch.py
```

Arguments: `rviz:=false` (pass this on a headless Jetson), `lidar:=false`,
`camera:=false`, `hardware_params:=<path>`.

A quick check that the camera is actually alive, rather than merely advertising:

```bash
ros2 topic hz /oak/rgb/camera_info     # ~30 Hz; the images are too big for
ros2 topic hz /oak/imu/data            # python's hz to keep up with
```

## Deploying to the robot

Nothing is cross-compiled, because almost nothing needs compiling. Of the
packages the robot runs, exactly one contains compiled code -
`ldlidar_component` and its vendored SDK - and that is built **on the robot**,
natively, where the architecture question does not arise. Everything else is
Python modules, launch files, parameter YAMLs and a robot description:
architecture-independent files that are simply copied.

Two scripts, split by how often you need them, with no overlap between them:
`provision.sh` prepares a robot, `deploy.sh` deploys to it. Anything `deploy.sh`
does, `provision.sh` deliberately does not.

| | `provision.sh` | `deploy.sh` |
|---|---|---|
| how often | once per robot | every iteration |
| apt dependencies | yes | - |
| lidar driver | built on the robot | - |
| robot description | yes | - |
| udev rules | yes | - |
| application, HAL, launch files | - | yes |
| systemd unit, wrapper, config | - | yes |
| starts the service | no | yes |

### `provision.sh` - once per robot

```bash
cd einride_mini_truck_bringup/tools
./provision.sh jetson@<robot-ip>
```

Installs the runtime apt packages and the udev rules, builds the lidar driver on
the robot, installs the robot description, and creates the install prefix.

It installs no service and starts nothing. Everything that changes while you
work - the application, the hardware abstraction layer, the launch files, the
unit itself - arrives with the first `deploy.sh`. Provisioning a robot leaves it
prepared and idle; one `deploy.sh` later it is running.

| on the robot | what it is |
|---|---|
| ROS 2 Jazzy (`ros-jazzy-ros-base`) | only installed if not already there |
| apt packages | resolved by rosdep, on the robot |
| `/opt/einride_mini_truck/install` | the lidar driver compiled there, plus the robot description |
| `/etc/udev/rules.d/99-einride-*.rules` | `/dev/ldlidar`, `/dev/ugv02`, and the camera's permissions |

Run it again when something it owns changes: an apt dependency, a udev rule, the
lidar driver, the robot description, or a package added to or removed from the
workspace. A re-provision replaces the prefix wholesale, which removes the
packages `deploy.sh` owns and stops the service - so it ends the way a first
provision does, with a deploy still to run.

Two options, and everything else is derived:

| | |
|---|---|
| `--uninstall` | remove the service, `/opt/einride_mini_truck` and the udev rules. The config and apt packages are left alone. |
| `--dry-run` | print what would run on the robot, and run none of it. |

rosdep still reads **every** package's manifest, including the ones `deploy.sh`
ships: the camera driver, the lidar lifecycle manager and the serial library are
declared by packages this script does not install, but installing *their
dependencies* is preparation, and doing it here is what keeps the deploy loop
fast.

The lidar needs its source beside this repository in the workspace - it is not a
released ROS package, so there is no arm64 binary of it to install instead:

```bash
cd <workspace>/src && git clone --recursive https://github.com/Myzhar/ldrobot-lidar-ros2.git
```

`--recursive` matters: the vendor SDK is a submodule. `provision.sh` checks and
says so if it is missing.

**`provision.sh` installs ROS 2 Jazzy itself if it is not already on the
robot** - `ros-jazzy-ros-base` from the stock `packages.ros.org` apt repo, not
the full desktop install, since this is a headless robot. That repo only has
real arm64 binaries for Ubuntu 24.04 (noble), which is what JetPack 7.2 is; on
anything else, provisioning fails loudly instead of guessing.

**`einride_mini_truck_gazebo` is not shipped either.** It is the only other
package with compiled code, `hardware.launch.py` never loads it - the sole
mention of `ros_gz` in that file is a comment - and leaving it off the robot is
what keeps Gazebo, Ogre and their thousand-package dependency tree off it too.
It is skipped by rosdep key along with `ros_gz_sim` and `ros_gz_bridge`, which
`bringup` declares as full `<depend>`s for simulation's sake. `rviz2` is *not*
skipped: it is off by default on the robot, but `RVIZ=true` is a documented
switch and one that failed with "package not found" would be worse than the disk
it costs.

### How the two halves meet

`provision.sh` builds the lidar on the robot and copies in packages built on your
machine, and those two end up in the same install tree. That works because a
colcon install tree does not care where it was built:

* the absolute paths colcon bakes into its generated environment hooks are only
  **fallbacks** - the top-level `setup.bash` computes the real prefix from its own
  location when sourced;
* and packages are discovered by **scanning the prefix** at source time, not from
  a list fixed at build time.

So a package directory built anywhere can be dropped into a prefix built
somewhere else and is found correctly. The setup scripts in the tree come from
the robot's own colcon build, and the copied-in packages are picked up beside
them even though that build never saw them.

### `deploy.sh` - every iteration

```bash
./deploy.sh jetson@<robot-ip>    # about two seconds
```

Builds the packages you are editing, copies them to the robot, restarts the
service. It takes no options, because everything it needs either has one
sensible answer or can be asked of the robot itself:

| | where it comes from |
|---|---|
| which robot | the argument - every time, with no way around it |
| ssh port, key | `~/.ssh/config`, like any other ssh client |
| where it installs | read off the robot, from the service's own config |
| what to build | the three packages below - the ones that change |

The robot is named on the command line and nowhere else: no default, no
environment variable, nothing remembered between runs. Every one of those would
let a deploy go somewhere other than the address in front of you, and a variable
exported in another terminal an hour ago is exactly the kind of thing nobody
notices until the wrong truck is running the wrong code.

Without a user it connects as `$USER`, exactly as ssh would - the robot's account
is usually `jetson`, so it is normally worth spelling out.

Ships `einride_mini_truck_application`, `einride_mini_truck_hardware` (the
chassis serial interface) and `einride_mini_truck_bringup` (the launch files and
parameter YAMLs), plus the systemd unit and its wrapper.
`einride_mini_truck_bringup` is in that set deliberately: the launch files and
YAMLs are edited as often as the code, and a deploy that silently did not pick up
a change to `hardware.launch.py` would be a trap.

The install prefix is **asked of the robot** rather than passed in - it is
already recorded in `/etc/default/einride-mini-truck`, so a flag could only ever
disagree.

None of these three packages contains compiled code, which is what makes a build
on your own machine safe to copy. That is an invariant rather than an assumption,
so the script checks it: if any file about to be shipped turns out to be a
compiled object, the deploy stops and tells you the package has to move to being
built on the robot, the way the lidar driver already is.

Each package directory is replaced with `rsync --delete`, so a file deleted from
a package is deleted on the robot too - one that lingered would keep being found
and would be blamed on anything but the deploy that left it there. Bytecode is
the only thing filtered out; `.pyc` files record the path they were compiled from
and Python regenerates them anyway.

On a first deploy these three packages are not on the robot at all - provisioning
deliberately leaves them out - so it creates them. It also installs the unit,
enables it for boot and starts it, which is why a freshly provisioned robot needs
no extra step beyond one `./deploy.sh`.

It refuses to run against a robot with no workspace at the prefix, pointing you
at `provision.sh`, rather than leaving a half-populated install tree. And it
never ships the workspace's own `install/setup.bash`, so **adding or removing a
package means running `provision.sh` again**.

### Watching it from Foxglove

`provision.sh` installs `foxglove_bridge` and `deploy.sh` installs a second unit,
`einride-mini-truck-foxglove`, that runs it. Open the Foxglove app and connect to:

```
ws://<robot-ip>:8765
```

That is a plain WebSocket on the robot's own network. **Nothing reaches out to
Foxglove's servers** - the robot only listens and the app connects inward, so no
account, no outbound access and no remote-access agent is involved.

Its settings live in the same `/etc/default/einride-mini-truck` as the stack:
`FOXGLOVE_PORT`, `FOXGLOVE_ADDRESS`, and `FOXGLOVE_EXTRA_ARGS` for the launch
file's own arguments. One file rather than two, because both units have to agree
about `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` - a bridge on a different domain
sees an empty graph and looks broken - and a second copy of those settings is a
second chance to get them different.

The bridge has no authentication of its own, so what keeps it private is the
network the robot is on. Set `FOXGLOVE_ADDRESS=127.0.0.1` and tunnel if that is
not enough:

```bash
ssh -L 8765:localhost:8765 jetson@<robot-ip>
```

Two things about the unit are deliberate. It is **ordered after the stack but not
bound to it**: the bridge is the window you watch the robot through, and it is
most useful when the stack is misbehaving - so restarting `einride-mini-truck`
does not take your Foxglove connection down with it, and the bridge serves an
empty graph while nothing is running, which is the honest thing to show. And on
a slow link the camera topics dominate the bandwidth, so a whitelist is the
difference between a usable connection and a stalled one:

```
FOXGLOVE_EXTRA_ARGS="topic_whitelist:=['/scan','/tf','/imu','/joint_states']"
```

### Why `robot_description` carries URDF and not SDF

`model.sdf` is the one description this project keeps, but `/robot_description`
publishes a URDF generated from it - `einride_mini_truck_description/tools/
sdf_to_urdf.cpp`, run at build time, and `model.urdf` is a build product like
the generated dock models.

The conversion is not for `robot_state_publisher`, which never needed it: it
loads `sdformat_urdf` as a `urdf_parser` plugin, so the parameter may hold
either dialect and the TF tree comes out the same either way. RViz resolves
`/robot_description` through that same pluggable parser, which is why the
RobotModel display worked on the raw SDF for as long as it did.

Foxglove does not share that parser. Its 3D panel has its own URDF reader, it
understands only `<robot>`, and handed an `<sdf>` document it renders no model
and reports no error - the panel simply stays empty, with the topic subscribed
and the data arriving. Publishing URDF is what makes the robot visible there,
and it costs nothing anywhere else: Gazebo loads `model.sdf` itself through
`model://` and never reads this topic.

Two conversion artefacts are cleaned up by the tool rather than left in the
file, both silent if they are not. `sdformat_urdf` blends SDF's ambient and
diffuse terms including the alpha channel, so two opaque terms yield `alpha 1.2`
- which urdfdom's own parser then rejects, dropping the whole material and the
link's colour with it. And urdfdom's exporter emits a bare `<texture/>` for
every material whether or not one was set. `test/test_urdf_export.py` checks for
both, along with the link and joint names surviving the round trip.

Sensors are dropped, because URDF has nowhere to put them - the build logs
`has a <sensor>, but URDF does not support this` once per sensor link. Nothing
reads sensor definitions off this topic, so what is lost is lost only to the
visualiser, which never wanted it.

### When every topic is listed but nothing publishes

The failure worth knowing about, because it looks like everything is fine.

Fast DDS prefers shared memory between processes on one host, and **on this
Jetson it does not work**: the segments are never created and the failure is
entirely silent. Discovery runs over UDP multicast regardless, so every topic
appears in `ros2 topic list`, Foxglove connects and lists every channel with
full QoS, `ros2 node list` looks right - and not one message crosses a process
boundary. A robot that appears perfectly healthy and publishes nothing.

So `/etc/default/einride-mini-truck` sets:

```
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

Measured on the robot: with the default transport, `/tf_static` (latched, so it
must arrive) and the lidar both deliver **0**. With `UDPv4`, **1** and **71**.
The cost is CPU on large messages, since camera frames are fragmented over
loopback rather than shared; if that ever matters more than the reliability, the
honest options are `LARGE_DATA` (TCP for user data) or fixing shared memory on
the platform. Returning to the default just restores the silent failure.

Both wrappers also run `fastdds shm clean` before starting, which clears
segments left behind by processes that died without cleaning up. That is worth
keeping - a crash should not poison the next start - but on its own it was **not
enough here**: the shared-memory transport failed again on a later boot with no
stale segments to blame, which is what settled the question in favour of naming
the transport outright.

**How to tell this apart from a sensor that is simply not producing.** Subscribe
to `/tf_static`, which `robot_state_publisher` latches once at startup; with
matching QoS (`RELIABLE` + `TRANSIENT_LOCAL`) it must arrive immediately. If even
that is silent, the problem is transport, not the sensors:

```bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ros2 topic echo /tf_static   # arrives? then SHM is the fault
```

### Running it

```bash
journalctl -u einride-mini-truck -f          # the whole stack's output
sudo systemctl restart einride-mini-truck    # after a redeploy
sudo systemctl stop einride-mini-truck       # before driving it by hand
```

After a source change, `./deploy.sh` again - it rebuilds, copies and restarts in
one step. `provision.sh` is only needed when an apt dependency, a udev rule, the
lidar driver, or the set of packages in the workspace changes.

### Things worth knowing about the unit

* **`/etc/default/einride-mini-truck` survives a redeploy.** It is the robot's
  local configuration - a domain ID, `CAMERA=false` on a machine whose camera is
  out for repair - so redeploying keeps the existing file and writes the new
  template beside it as `.new` when the two differ. Diff them after an upgrade;
  nothing else will tell you a default moved.
* **It stops with SIGINT, not SIGTERM.** `ros2 launch` shuts its nodes down in
  order on SIGINT; on SIGTERM it exits faster and less carefully, which here can
  leave the chassis MCU holding the last commanded velocity - a stopped service
  and a rover still driving. `cmd_vel_timeout` in `hardware.yaml` is the
  backstop; `KillSignal=SIGINT` is what avoids needing it.
* **It runs as your login user, but the code is root-owned.** The devices are
  reachable through the udev rules, and systemd carries `dialout` and `video`
  membership over; the account has no write access to `/opt/einride_mini_truck`,
  because the account the robot is driven from has no reason to be able to
  rewrite the code it runs. The deploy warns if the user is in neither group.
* **The service is stopped before the tree is replaced.** Overwriting an install
  space under a running stack leaves it holding deleted inodes: it keeps working
  until something tries to `dlopen` a plugin it has not loaded yet, and then
  fails in a way that has nothing to do with the deploy that caused it.
* **RViz is off.** `RVIZ=false` in the environment file overrides the launch
  file's default, which is written for a developer at a desk. Run RViz on your
  own machine against the same `ROS_DOMAIN_ID`.
* **The first boot after a cold start may retry.** `/dev/ldlidar` and the OAK-D's
  USB node are created by udev, which can still be settling when the service
  starts; the unit allows ten restarts in two minutes to cover that. If it gives
  up, the journal holds the real reason - it is not the race.

Templates are the source of truth: edit `systemd/*.in` and redeploy rather than
editing the generated copies on the robot, which the next deploy overwrites.

## Differences between hardware and simulation

| | why |
|---|---|
| ~80 Hz feedback, not 100 | At 115200 baud a ~140-byte `T:1001` line takes 12.2 ms to transmit, which caps the loop |
| Stamps are ~12 ms late | The MCU sends no timestamps. The node subtracts each line's transmission time; the residual goes in `stamp_offset`, to be measured on the robot |
| No IMU orientation | The MCU sends raw gyro and accelerometer counts, no fused attitude, so `orientation_covariance[0]` is -1 per REP-145. Simulation does provide an orientation |
| No wheel effort | `/wheel_encoders` leaves `effort` empty; the chassis has no torque sensing. Simulation's zeros are not measurements either |
| Softer command response | Serial round trip plus MCU PID, against Gazebo applying joint velocity immediately |
| Depth encoding differs | Hardware publishes depth as `16UC1` millimetres, gz's depth camera emits `32FC1` metres. Nothing can reconcile this in the model; read the encoding field |
| Colour channel order | Hardware `bgr8`, simulation `rgb8`. `cv_bridge` handles either, raw byte access does not |
| No lens distortion | Hardware `camera_info` carries a `rational_polynomial` model with real coefficients; gz publishes `plumb_bob` with all zeros, and principal point exactly at centre (640.0/360.0 against the device's 642.9/373.8) |
| Depth noise does not grow with range | Real stereo error goes as z^2 (measured k = 3.33 mm/m^2); gz offers only a constant stddev, set to that k at a 3 m reference. Simulation is right at 3 m, pessimistic closer, optimistic further. Scale by `(z/3)^2` if it matters |
| Depth never drops out | Gazebo returns a value for every pixel inside the clip range; the real camera fills about 59% and drops out on untextured surfaces, at range, and in the 75 mm baseline's occlusion band |
| Lidar intensity is not physical | Gazebo does not model return strength, so simulation writes a constant on bins that returned something and `NaN` on bins that did not. The real device reports 7..255. The valid/blank structure matches, the magnitude does not - see "Matching the real LiDAR" |
| Lidar blanks are geometric only | A simulated bin is blank because the ray reached max range. A real one is also blank off dark, glossy or glancing surfaces, so the robot sees blanks where simulation sees a wall |

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
  cosmetic. Now emitted by the build rather than by `robot_state_publisher`,
  since the SDF-to-URDF conversion moved to build time; see "Why
  `robot_description` carries URDF and not SDF".
* `kdl_parser: The root link base_footprint has an inertia` - `base_footprint`
  carries a 1 g token inertia because Gazebo requires one per link, while KDL
  prefers a massless root. TF is published correctly either way.
