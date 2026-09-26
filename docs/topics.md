# Topics

| topic | type | direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | command |
| `/odom` | `nav_msgs/Odometry` | state |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | state |
| `/joint_states` | `sensor_msgs/JointState` | all 6 wheel joints, throttled |
| `/joint_states_raw` | `sensor_msgs/JointState` | unthrottled source of the above |
| `/wheel_encoders` | `sensor_msgs/JointState` | wheels with encoders only |
| `/imu` | `sensor_msgs/Imu` | chassis ICM-20948 |
| `/mag` | `sensor_msgs/MagneticField` | AK09916 |
| `/scan`, `/scan/points` | `LaserScan`, `PointCloud2` | LD19P |
| `/oak/rgb/image_raw`, `/oak/rgb/camera_info` | `Image`, `CameraInfo` | colour, 1280x720 (`bgr8` on hardware, `rgb8` in simulation) |
| `/oak/stereo/image_raw`, `/oak/stereo/camera_info` | `Image`, `CameraInfo` | depth, 1280x720 (`16UC1` mm on hardware, `32FC1` m in simulation). Hardware: `rgbd` only |
| `/oak/left/image_raw`, `/oak/right/image_raw` (+ `camera_info`) | `Image`, `CameraInfo` | raw mono pair, 640x480 `mono8`, 74.75 mm baseline. Hardware: `rgbstereo` only |
| `/oak/points` | `PointCloud2` | depth cloud. Hardware: `rgbd` only |
| `/oak/imu/data` | `sensor_msgs/Imu` | camera BMI270 |
| `/voltage` | `std_msgs/Float32` | battery voltage, hardware only |

## Subscribing to the sensor streams

`/imu`, `/mag` and `/wheel_encoders` are published **best-effort**
(`qos_profile_sensor_data`, KEEP_LAST(5)).

Your subscription must also be best-effort. Passing a plain depth number asks
for RELIABLE, which does not match - you receive nothing, and only a QoS warning
appears in the log:

```python
from rclpy.qos import qos_profile_sensor_data

self.create_subscription(Imu, '/imu', self.cb, qos_profile_sensor_data)  # works
self.create_subscription(Imu, '/imu', self.cb, 10)                       # receives nothing
```

`/cmd_vel` and `/voltage` are reliable. A lost command would leave the MCU
running at its last velocity, and `/voltage` only arrives once per second.
