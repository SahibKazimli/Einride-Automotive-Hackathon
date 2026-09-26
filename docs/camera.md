# Camera

Luxonis **OAK-D Lite**: an IMX214 colour camera, stereo depth from a pair of
OV7251 mono cameras on a 75 mm baseline, and a built-in BMI270 IMU.

All numbers in this section were measured on the real camera over USB. Where the
datasheet says something different, that is pointed out.

| stream | resolution | HFOV | VFOV | DFOV | encoding | topic |
|---|---|---|---|---|---|---|
| colour | 1280x720 @ 30 Hz | 69.85 | 42.90 | 77.41 | `bgr8` | `/oak/rgb/image_raw` |
| depth | 1280x720 @ 30 Hz | 69.85 | 42.90 | 77.41 | `16UC1` mm | `/oak/stereo/image_raw` |
| left mono | 640x480 @ 30 Hz | 70.52 | 55.89 | - | `mono8` | `/oak/left/image_raw` |
| right mono | 640x480 @ 30 Hz | 70.37 | 55.76 | - | `mono8` | `/oak/right/image_raw` |

Depth range is **0.2 - 10 m**. The 10 m limit comes from the ROBOTICS preset
(see below); the sensor itself could reach 19 m. The point cloud is on
`/oak/points` and the camera IMU is on `/oak/imu/data`.

**Colour and depth share one optical frame**, `oak_rgb_camera_optical_frame`.
Depth is aligned to the colour camera, so the depth image has the colour
camera's size and intrinsics. The mono cameras' own 640x480 resolution and
~70.4 degree field of view do not appear on `/oak/stereo/*`.

**The mono cameras are also published raw**, on `/oak/left/*` and
`/oak/right/*`, in `oak_left_camera_optical_frame` and
`oak_right_camera_optical_frame`. These are the images the depth is computed
from. They are not rectified, each has its own `rational_polynomial` distortion
and intrinsics, and they are **74.75 mm** apart. The two sensors are slightly
different (fx is 452.65 vs 453.85), and they are not placed symmetrically around
the colour camera. That is why these values are read from the device and not
from a datasheet.

**If you use them with `image_pipeline`:** depthai puts the baseline term
`P[3]` on the **left** `camera_info` and sets the right one to zero. ROS expects
the opposite. Either swap which topic you pass as left and right, or set
`left.i_reverse_stereo_socket_order`.

## Two camera configurations

The camera's USB link cannot carry colour, depth, left, right and the IMU all
at once. With all of them on, the IMU stops publishing. So on hardware you pick
one of two configurations with the `camera_config` launch argument:

| `camera_config` | params file | colour | depth + `/oak/points` | raw left/right | IMU |
|---|---|---|---|---|---|
| `rgbd` (default) | `oak_d_lite.yaml` | yes | yes | no | yes |
| `rgbstereo` | `oak_d_lite_rgbstereo.yaml` | yes | no | yes | yes |

```bash
ros2 launch einride_mini_truck_bringup hardware.launch.py camera_config:=rgbstereo
```

* `rgbd` uses `i_pipeline_type: RGBD`. The mono cameras still feed depth inside
  the camera, but are not published on their own topics.
* `rgbstereo` uses `i_pipeline_type: RGBStereo`. The camera computes no depth
  at all, so `/oak/stereo/*` and `/oak/points` do not exist.
* Colour, IMU, TF and resolution settings are the same in both files.
* Passing `camera_params:=<path>` uses your own file and ignores
  `camera_config`.

Simulation always publishes every stream.

## Camera settings

The settings are in
[`oak_d_lite.yaml`](../einride_mini_truck_bringup/config/oak_d_lite.yaml) and
[`oak_d_lite_rgbstereo.yaml`](../einride_mini_truck_bringup/config/oak_d_lite_rgbstereo.yaml).

**Colour runs at 1080P, scaled by 2/3 to 1280x720.** Depth alignment requires
both image dimensions to be multiples of 16. 1280 and 720 both are. The
full-sensor view scaled down (for example 1052x780) is not, and the driver
rejects it:

```
ISP scaling with num: 1 and den: 4 results in width: 1052 and height: 780
which are not divisible by 16.
StereoDepth: Custom disparity/depth width must be multiple of 16.
```

No downscale of the full 4208x3120 sensor gives a multiple of 16, so you can
have aligned depth or the full vertical field of view, but not both. This
project uses aligned depth. 1080P uses the full sensor width, so the horizontal
field of view stays at 69.85 degrees and only the vertical field is reduced
(42.90 instead of 54.76 degrees). With the camera 0.10 m above the ground and
looking level, the closest visible ground point is 0.26 m ahead instead of
0.20 m. `rgbstereo` has no depth but uses the same colour settings, so the two
configurations give identical colour images.

**Do not set `720P`.** The IMX214 has no 720P mode. The driver logs
`Resolution 720P not supported by sensor IMX214. Using default resolution 1080P`
and falls back to 1080P. The same is true for `800P` and `1200P`. The valid
values of `i_resolution` are `400P 480P 720P 800P 1080P 1200P 1440X1080
5312X6000 12MP 13MP 48MP`. At 13MP the colour sensor runs at most **28.86 fps**,
not 30.

**`i_width` and `i_height` set what `camera_info` reports, and nothing checks
them against the real image.** If they say 1280x720 while the image is 640x480,
`camera_info` will be wrong. The config files set every size explicitly for
this reason.

**The IMU is enabled under two parameter names.** Different builds of
depthai_ros_driver 2.12.2 read different keys: some read `camera.i_enable_imu`,
others read `pipeline_gen.i_enable_imu`. Both config files set both to `true`.
Unknown keys in a params file are ignored, so this is safe.

**The IMU runs at 100 Hz.** The driver default is
400 Hz, but the device tops out at about 250 Hz.

## Depth filtering: the ROBOTICS preset

This applies to the `rgbd` configuration only. `oak_d_lite.yaml` sets
`stereo.i_depth_preset: ROBOTICS`, Luxonis' preset for navigation and obstacle
detection.

Without it, the driver uses `HIGH_ACCURACY`, which has no range limit and no
spatial, speckle or temporal filtering. On this camera that produces depth
values up to **65.535 m** indoors (the largest value a uint16 can hold). That is
noise, since the sensor cannot see past 19 m.

| | `HIGH_ACCURACY` (driver default) | `ROBOTICS` (used) |
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

ROBOTICS fills slightly fewer pixels but removes all the false far-away
readings.

With `decimationFactor=2`, depth is still published at 1280x720, but it is
computed at half resolution and scaled up. Fine detail is coarser than the pixel
count suggests.

The Luxonis documentation says ROBOTICS uses a confidence threshold of 15. The
depthai 2.30 library actually uses **245**. Trust the device.

### Two settings the config has to pin

The driver applies the preset first and then writes its own defaults on top.
Two of those defaults replace preset values, so `i_depth_preset: ROBOTICS` alone
does **not** give you the full preset. The config sets them back:

| | preset value | driver default | config sets |
|---|---|---|---|
| `i_depth_filter_size` | 7 (KERNEL_7x7) | 5 | **7** |
| `i_stereo_conf_threshold` | 245 | 240 | **245** |

This matters: the 7x7 median filter reduces noise beyond 4 m from 372 mm to
150 mm.

### Depth noise

Per-pixel noise (standard deviation over time) on a still scene:

| range | median stddev |
|---|---|
| 1 - 2 m | 13.5 mm |
| 2 - 3 m | 33.1 mm |
| 3 - 4 m | 32.7 mm |
| 4 - 6 m | 150 mm |
| 6 - 10 m | 633 mm |

Across 1-4 m the median is **31.1 mm**, which gives **k = 3.33 mm/m^2** for
error that grows with distance squared. Beyond 4 m the noise grows quickly, so
treat 10 m as the point where readings stop, not as a useful working range.

Use the median, not the mean. The mean over 1-4 m is 69 mm because of large
errors at object edges.

`model.sdf` uses `stddev` 0.030 (k at 3 m). Measured with the same script,
simulation gives 30.1 mm and the hardware 31.1 mm.

## Which depthai driver

**Use the v2 driver, `ros-jazzy-depthai-ros-driver`.** The v3 driver
(`ros-jazzy-depthai-ros-driver-v3`, depthai 3.9.0) **cannot run this camera's
mono cameras**. Starting either of them crashes the camera firmware within
seconds:

```
RTEMS_FATAL_SOURCE_INVALID_HEAP_FREE   thread CBTH
```

after which the host keeps logging `X_LINK_ERROR` and reconnecting. Colour and
the IMU work under v3. The camera itself is fine; the mono cameras work under
v2. Both packages can be installed at the same time, and the launch file uses
v2.

`hardware.launch.py` starts the driver with
**`camera_as_part_of_a_robot.launch.py`**, not `camera.launch.py`. The latter
starts its own `robot_state_publisher`, which would conflict with the one
`common.launch.py` runs.

## Who publishes the camera frames

The camera's TF frames come from two sources:

* **`model.sdf`** places the camera body, `oak_d_lite_link`, and the IMU frame,
  `oak_imu_frame`.
* **The camera's own factory calibration (EEPROM)** places the lenses:
  `oak_{rgb,left,right}_camera_frame` and their `_camera_optical_frame` children.
  Every physical camera is slightly different - on this unit the mono cameras
  are 74.75 mm apart instead of 75, and not centred on the colour camera - so
  these must come from the device.

On hardware, the driver publishes the calibration frames
(`camera.i_publish_tf_from_calibration` in the camera config). In simulation,
`oak_calibration_tf` in `einride_mini_truck_gazebo` publishes them, using the
same calculation on a saved copy of the same EEPROM
(`einride_mini_truck_description/calibration/`). `test_oak_calibration_tf.py`
checks the result against `/tf_static` recorded from the real camera.

The full tree, with the source of each part:

```
base_footprint                                  robot_state_publisher, from model.urdf
└── base_link
    ├── base_imu_link                           chassis ICM-20948 / AK09916
    ├── base_lidar_link                         LD19P
    ├── <six wheel links>                       moved by /joint_states
    └── oak_d_lite_link                         the camera body - the mount
        ├── oak_imu_frame                       model.sdf (the driver's version is 120 deg wrong)
        └── oak                                 identity; the name the driver needs
            └── oak_rgb_camera_frame            identity. everything below: device EEPROM,
                │                                 via the driver on hardware and
                │                                 oak_calibration_tf in simulation
                ├── oak_rgb_camera_optical_frame
                └── oak_right_camera_frame      -37.00 mm
                    ├── oak_right_camera_optical_frame
                    └── oak_left_camera_frame   +74.75 mm from right
                        └── oak_left_camera_optical_frame
```

Things to note:

* The cameras form a **chain**, not a star. Each camera is attached to the one
  its calibration refers to, so left hangs off right, not off the body.
* Every `_camera_frame` uses FLU axes (x forward, y left, z up). Every
  `_camera_optical_frame` uses x right, y down, z forward. They differ by a fixed
  rotation of `-pi/2, 0, -pi/2`. `camera_info` refers to the optical frame.
* The two sources must not publish the same frame. If a frame appears twice in
  `/tf_static` with different parents, tf2 keeps whichever message arrived last,
  with no error. That is why `model.sdf` does not declare
  `oak_rgb_camera_optical_frame`.
* The driver attaches the root of its camera chain to `i_tf_base_frame` with an
  identity transform. On this device the root is the colour camera, which
  `model.sdf` places at `oak_d_lite_link`'s origin with the same rotation the
  driver uses, so the two halves join exactly. If a replacement camera used a
  mono camera as its root, everything would shift by ~37 mm without warning;
  `test_oak_calibration_tf.py` checks that the root is the colour camera.

### Why there is a link called `oak`

`model.sdf` has a link called `oak`, at the same place as `oak_d_lite_link`. It
exists only so `i_tf_base_frame` can point at it. Do not remove it.

When `i_publish_tf_from_calibration` is on, the driver uses `i_tf_base_frame` as
the prefix for every **image** `frame_id`, but names the TF frames it
**publishes** after the node name:

```cpp
std::string tfPrefix(std::shared_ptr<rclcpp::Node> node) {
    if(node->get_parameter("camera.i_publish_tf_from_calibration").as_bool()) {
        return node->get_parameter("camera.i_tf_base_frame").as_string();
    }
    return node->get_name();
}
```

So the two only match when the base frame has the same name as the node, `oak`.
If you point it at `oak_d_lite_link` instead, TF will contain
`oak_left_camera_optical_frame` but images will be stamped
`oak_d_lite_link_left_camera_optical_frame` - a frame that does not exist. There
is no warning.

You will see `Published URDF` in the robot's log. The driver sends a URDF to a
node called `oak_state_publisher`, which is not running here, so nothing happens.
This is harmless.
