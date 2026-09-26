# Inertial sensing

Two separate IMUs. The chassis ICM-20948 also contains an AK09916
magnetometer, which is what makes it 9-axis. The camera's BMI270 has no
magnetometer.

| part | where | frame | topics | gyro sigma | accel sigma |
|---|---|---|---|---|---|
| **ICM-20948** (9-axis) | chassis | `base_imu_link` | `/imu`, `/mag` | 2.62e-3 rad/s | 2.26e-2 m/s^2 |
| **BMI270** (6-axis) | inside the OAK-D Lite | `oak_imu_frame` | `/oak/imu/data` | 1.59e-3 rad/s | 2.68e-2 m/s^2 |

The chassis values are **calculated** from datasheet noise densities at 100 Hz
bandwidth: gyro 0.015 deg/s/sqrt(Hz), accel 230 ug/sqrt(Hz). The AK09916
magnetometer sigma of 3e-7 T is an estimate based on its 0.15 uT/LSB
resolution.

The BMI270 values are **measured on the real device**: 2002 samples at 100 Hz
at rest gave per-axis stddevs of 1.48e-3 / 0.88e-3 / 2.16e-3 rad/s and
3.62e-2 / 1.01e-2 / 2.71e-2 m/s^2. The table shows the RMS across axes. These
include some bench vibration, so they are an upper bound.

**Which IMU to use for what:**

* **Rotation rate: the camera's BMI270.** Its gyro is about 1.6x less noisy.
  Angular velocity is the same everywhere on a rigid body, so its off-centre
  mounting does not matter for the gyro.
* **Acceleration: the chassis ICM-20948.** Accelerometer readings depend on
  where the sensor sits.
* **Absolute heading: the chassis AK09916 magnetometer** (`/mag`).

## The two IMUs use different axes

* `/imu` is in `base_imu_link`: x forward, y left, z up.
* `/oak/imu/data` is in `oak_imu_frame`, the BMI270's own axes. The driver does
  not rotate them. With the robot level: **x = right, y = forward, z = up**.

Both show gravity on `+z` when the robot is still, but the horizontal axes are
swapped and one is flipped. **Always combine them through TF, never by adding
their vectors directly.**

`model.sdf` places `oak_imu_frame` using the values read from the device with `getImuToCameraExtrinsics(CAM_A)`: 31.7 mm left
of the colour camera, 2.2 mm up, 7.1 mm behind, and rotated 89.3 degrees about
the optical x axis.

### The driver does not publish `oak_imu_frame`

**The driver's transform for the IMU is wrong by 120 degrees**, so the camera
config sets `camera.i_tf_imu_from_descr: 'true'` to use the one from
`model.sdf` instead. Do not remove this setting. Simulation's
`oak_calibration_tf` also leaves this frame out, for the same reason.

The cause: `depthai_bridge`'s `TFPublisher::quatFromRotM` computes
`q_rot2rdf * q_extr * q_rot2rdf^-1`. That is correct between two optical camera
frames, but the IMU frame is not an optical frame, so the result is off by one
`q_rot2rdf` - a 120 degree rotation. IMU axes expressed in the robot's FLU frame:

| source | imu_x | imu_y | imu_z |
|---|---|---|---|
| `model.sdf` (correct, matches the device) | right | forward | up |
| depthai `TFPublisher` (wrong) | up | left | back |

The positions agree exactly; only the rotation is wrong.
