# Differences between hardware and simulation

| | details |
|---|---|
| Feedback at ~80 Hz, not 100 | At 115200 baud, a ~140-byte `T:1001` line takes 12.2 ms to send, which limits the rate |
| Timestamps are ~12 ms late | The MCU sends no timestamps. The node subtracts each line's send time; any remaining error goes in `stamp_offset`, to be measured on the robot |
| No IMU orientation | The MCU sends only raw gyro and accelerometer values, so `orientation_covariance[0]` is -1 (REP-145). Simulation does provide orientation |
| No wheel effort | `/wheel_encoders` has an empty `effort` field; the chassis has no torque sensing. The zeros in simulation are not measurements either |
| Slower response to commands | The serial link and the MCU's PID add delay. Gazebo applies wheel velocity immediately |
| Camera streams | Hardware publishes either depth or the raw mono pair, chosen with `camera_config`. Simulation always publishes both. See [Two camera configurations](camera.md#two-camera-configurations) |
| Depth encoding | Hardware: `16UC1` in millimetres. Simulation: `32FC1` in metres. Check the encoding field |
| Colour channel order | Hardware: `bgr8`. Simulation: `rgb8`. `cv_bridge` handles both; raw byte access does not |
| No lens distortion in simulation | Hardware `camera_info` has a `rational_polynomial` model with real values. Gazebo publishes `plumb_bob` with all zeros and the principal point exactly at the centre - 640.0/360.0 vs the device's 642.9/373.8 for colour, and 320.0/240.0 vs 333.7/257.5 and 333.8/248.4 for the mono pair |
| Mono `camera_info` baseline is wrong in simulation | The images are correct; `P[3]` is not. Hardware puts the whole baseline on the left camera (`-33.837`) and `0` on the right. Gazebo gives each camera `-fx * its own sideways offset` (`-17.089` and `-16.792`), which implies a 37.75 mm baseline instead of 74.75. `stereo_image_proc` gives wrong results in simulation; use TF, which is correct |
| Depth noise does not grow with distance | Real stereo error grows with distance squared (k = 3.33 mm/m^2). Gazebo only supports a fixed stddev, set to match the real camera at 3 m. Simulation is too noisy closer than 3 m and too clean further away. Scale by `(z/3)^2` if needed |
| Depth has no gaps in simulation | Gazebo returns a value for every pixel. The real camera fills about 59%, with gaps on plain surfaces, at long range, and near object edges |
| Lidar intensity is not real in simulation | Gazebo writes a fixed value on bins with a return and `NaN` on empty ones. The real device reports 7..255. See [Simulation matches the real LiDAR](lidar.md#simulation-matches-the-real-lidar) |
| Lidar gaps in simulation are geometric only | A simulated bin is empty only when nothing is in range. On the real robot, dark, shiny or angled surfaces can also give empty bins, so the robot may see gaps where simulation sees a wall |
