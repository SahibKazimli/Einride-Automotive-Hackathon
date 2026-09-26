# einride-mini-truck

A ROS 2 Jazzy autonomous driving stack for Einride's AI-powered autonomy
challenge. It runs on a **six-wheel skid-steer rover** with an NVIDIA Jetson Orin
Nano Super, four driven wheels, a 360-degree lidar, a stereo depth camera and
two IMUs. The project also
includes a Gazebo Harmonic simulation of the same robot.

## Included packages

* `einride_mini_truck_description` - the SDF model and meshes.
* `einride_mini_truck_gazebo` - world files and Gazebo system plugins.
* `einride_mini_truck_application` - ROS 2 application code (placeholder).
* `einride_mini_truck_hardware` - the serial hardware abstraction layer, used to
  run the same stack on the real robot.
* `einride_mini_truck_bringup` - launch files, bridge config and RViz config.

---

# The robot

| | |
|---|---|
| Computer | NVIDIA Jetson Orin Nano Super |
| Drive | 6-wheel skid-steer, **4WD**: front and rear pairs are driven, the middle pair rolls freely |
| Wheels | radius 0.040 m, width 0.0425 m, track 0.17452 m, wheelbase 0.171 m |
| Mass | 3.72 kg |
| Lidar | InnoMaker LD19P, 360-degree DTOF, 0.02-12 m |
| Camera | Luxonis OAK-D Lite, colour + stereo depth |
| IMUs | chassis ICM-20948 (9-axis) and the camera's BMI270 (6-axis) |

## Drivetrain

The robot has six wheels, and four of them are driven. The front and rear pairs
receive drive torque. The middle pair only carry weight and roll along with the
ground; they take no command. Only the front wheels have encoders.

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

1. Launch (the simulation starts running, not paused)

    ```bash
    ros2 launch einride_mini_truck_bringup simulation.launch.py
    ```

   Arguments: `headless:=true` (no GUI), `rviz:=false`, `world:=<basename>`,
   `joint_state_rate:=<Hz>` (default 50).

   `einride_mini_truck.launch.py` is an alias for `simulation.launch.py` and
   accepts the same arguments.

## Driving it

```bash
# forward
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.4}}'
# rotate in place
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 1.0}}'

ros2 topic echo /odom
```

The full list of topics is in [docs/topics.md](docs/topics.md).

---

# Deploying to the robot

The same stack runs on the robot. Only the layer that talks to the hardware is
different.

There are two scripts. `provision.sh` prepares a robot, and `deploy.sh` deploys
your code to it. They do not overlap:

| | `provision.sh` | `deploy.sh` |
|---|---|---|
| how often | once per robot | every change |
| apt dependencies | yes | - |
| lidar driver | built on the robot | - |
| robot description | yes | - |
| udev rules | yes | - |
| application, HAL, launch files | - | yes |
| systemd unit, wrapper, config | - | yes |
| starts the service | no | yes |

## `provision.sh` - once per robot

```bash
cd einride_mini_truck_bringup/tools
./provision.sh jetson@<robot-ip>
```

It installs ROS 2 Jazzy if missing, the apt packages, and the udev rules that
give each device a fixed name (`/dev/ugv02` for the chassis, `/dev/ldlidar` for
the lidar, plus camera access). It builds the lidar driver on the robot,
installs the robot description, and adds the user to the `dialout` and `video`
groups. The robot must run Ubuntu 24.04 (JetPack 7.2).

Run it again when an apt dependency, a udev rule, the lidar driver, the robot
description or the list of workspace packages changes, then run `deploy.sh`.
`--uninstall` removes everything it installed; `--dry-run` prints what it would
run.

## `deploy.sh` - every change

```bash
./deploy.sh jetson@<robot-ip>    # about two seconds
```

It builds `einride_mini_truck_application`, `einride_mini_truck_hardware` and
`einride_mini_truck_bringup`, copies them to the robot and restarts the service.
On the first deploy it also installs the service and enables it at boot.
**Adding or removing a package requires running `provision.sh` again.**

## Running it

On the robot, the `einride-mini-truck` service runs the stack:

```bash
journalctl -u einride-mini-truck -f          # all output from the stack
sudo systemctl restart einride-mini-truck    # restart the stack
sudo systemctl stop einride-mini-truck       # stop it before running it by hand
```

The service's settings are in `/etc/default/einride-mini-truck`: `LIDAR`,
`CAMERA`, `RVIZ`, the domain ID, and `EXTRA_LAUNCH_ARGS` for any other launch
argument - for example `EXTRA_LAUNCH_ARGS="camera_config:=rgbstereo"`. Restart
the service after editing it.

To run the stack by hand instead, stop the service first:

```bash
sudo systemctl stop einride-mini-truck
ros2 launch einride_mini_truck_bringup hardware.launch.py rviz:=false
```

Arguments: `rviz:=false` (use this on the robot, which has no display),
`lidar:=false`, `camera:=false`, `camera_config:=rgbd|rgbstereo`,
`hardware_params:=<path>`.

To check the sensors are publishing:

```bash
ros2 topic hz /scan                    # ~9.9 Hz
ros2 topic hz /oak/rgb/camera_info     # ~30 Hz; the images are too big for
ros2 topic hz /oak/imu/data            # python's hz to keep up with
```

---

# Testing

```bash
colcon test --packages-select einride_mini_truck_hardware einride_mini_truck_bringup
colcon test-result --verbose
```

The tests cover:

* the serial codec as pure functions - message framing across chunks, unit
  conversions, bad and non-finite input;
* the node against a **pty** that pretends to be the MCU, replaying a recording,
  with the real reader thread, writer thread and executor - no mocks;
* a conformance test that launches both modes and compares their ROS graphs.

`einride_mini_truck_hardware/test/data/ugv02_feedback.jsonl` is synthetic data
generated from the protocol documentation, not a recording from the robot.
Replace it with a real capture during on-robot bring-up; the tests read
whatever is in this file.

---

# Documentation

* [Topics](docs/topics.md) - every topic, its type, and the QoS to subscribe with.
* [Lidar](docs/lidar.md) - the LD19P, how simulation matches it, and what to do
  when it produces nothing.
* [Camera](docs/camera.md) - the OAK-D Lite streams, the two camera
  configurations, depth filtering and the camera TF frames.
* [Inertial sensing](docs/imu.md) - the two IMUs, which to use for what, and
  their axes.
* [Differences between hardware and simulation](docs/sim-vs-hardware.md)
* [Simulation notes](docs/simulation.md) - how the model is tuned and which log
  messages are harmless.
* [Provisioning, deploying and the robot service](docs/robot-service.md) - what
  `provision.sh` and `deploy.sh` do in detail, how the service behaves, and the
  Fast DDS transport setting.
* [Watching the robot from Foxglove](docs/foxglove.md)
