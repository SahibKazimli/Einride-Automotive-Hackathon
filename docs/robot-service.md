# Provisioning, deploying and the robot service

The README shows how to run `provision.sh` and `deploy.sh`. This page covers
what they do in detail, and how the service they install behaves.

## `provision.sh`

It installs the apt packages and udev rules, builds the lidar driver on the
robot, installs the robot description and creates the install directory. It
does not install or start the service - the first `deploy.sh` does that.

| on the robot | what it is |
|---|---|
| ROS 2 Jazzy (`ros-jazzy-ros-base`) | installed only if missing |
| apt packages | resolved by rosdep on the robot |
| `/opt/einride_mini_truck/install` | the lidar driver built there, plus the robot description |
| `/etc/udev/rules.d/99-einride-*.rules` | `/dev/ldlidar`, `/dev/ugv02`, and camera permissions |

It also adds the user to the `dialout` and `video` groups.

Run it again when any of these change: an apt dependency, a udev rule, the lidar
driver, the robot description, or the list of packages in the workspace.
Re-provisioning replaces the whole install directory, which removes the
packages `deploy.sh` installs and stops the service. Run `deploy.sh` afterwards.

Options:

| | |
|---|---|
| `--uninstall` | remove the service, `/opt/einride_mini_truck` and the udev rules. Config and apt packages are kept. |
| `--dry-run` | print what would run on the robot, without running it. |

rosdep reads the dependencies of **every** package, including the ones
`deploy.sh` ships, so all their dependencies are installed here and deploys stay
fast.

The lidar driver has no arm64 binary, so `provision.sh` clones
[`ldrobot-lidar-ros2`](https://github.com/Myzhar/ldrobot-lidar-ros2) at a pinned
commit, applies `tools/patches/ldlidar_install_sdk.patch` (upstream does not
install the driver's SDK library), and builds it on the robot. You do not need
to clone it yourself.

**`provision.sh` installs ROS 2 Jazzy if the robot does not have it**, using
`ros-jazzy-ros-base` from `packages.ros.org`. That repository only has arm64
builds for Ubuntu 24.04 (noble), which is what JetPack 7.2 uses. On any other
OS, provisioning stops with an error.

**`einride_mini_truck_gazebo` is not installed on the robot.**
`hardware.launch.py` does not use it, and leaving it out keeps Gazebo and its
large dependency tree off the robot. rosdep also skips `ros_gz_sim` and
`ros_gz_bridge`. `rviz2` *is* installed, so that `RVIZ=true` works if you need
it.

## How the two parts fit together

The robot's install directory contains the lidar driver built on the robot and
packages built on your machine. This works because a colcon install directory
does not depend on where it was built: `setup.bash` works out its real location
when sourced, and it finds packages by scanning the directory. A package built
anywhere can be copied in and will be found.

## `deploy.sh`

It builds the packages you are working on, copies them to the robot and restarts
the service. It has no options:

| | where it comes from |
|---|---|
| which robot | the command-line argument - always required |
| ssh port, key | `~/.ssh/config` |
| where it installs | read from the robot's `/etc/default/einride-mini-truck` |
| what to build | the three packages below |

The robot is only ever given on the command line. Without a user name it
connects as `$USER`, like ssh. The robot's default user name is `jetson`.

It ships `einride_mini_truck_application`, `einride_mini_truck_hardware` (the
chassis serial interface) and `einride_mini_truck_bringup` (launch files and
YAML), plus the systemd unit and its wrapper script.

None of these packages contain compiled code, so building them on your machine
and copying is safe. The script checks this: if it finds a compiled file, it
stops and tells you the package must be built on the robot instead.

Each package directory is synced with `rsync --delete`, so files you delete
locally are also deleted on the robot. Only `.pyc` files are skipped.

On the first deploy it creates the three packages, installs the service, enables
it at boot and starts it. It refuses to run if the robot has not been
provisioned. It does not copy the workspace's `install/setup.bash`, so **adding
or removing a package requires running `provision.sh` again**.

## Things to know about the service

* **Its settings are in `/etc/default/einride-mini-truck`**: `LIDAR`, `CAMERA`,
  `RVIZ`, the domain ID, and `EXTRA_LAUNCH_ARGS` for any other launch argument -
  for example `EXTRA_LAUNCH_ARGS="camera_config:=rgbstereo"`. Restart the service
  after editing it.
* **That file is kept across deploys.** If the template changes, the deploy
  writes it next to the existing file as `.new`. Compare the two after an update.
* **It stops with SIGINT, not SIGTERM.** SIGINT lets `ros2 launch` shut nodes
  down in order. SIGTERM can leave the chassis MCU holding its last velocity, so
  the service stops but the rover keeps driving. `cmd_vel_timeout` in
  `hardware.yaml` is a backup for this.
* **It runs as your login user, but the code is owned by root.** udev rules give
  access to the devices, and systemd adds the `dialout` and `video` groups. The
  user cannot modify `/opt/einride_mini_truck`. The deploy warns if the user is
  in neither group.
* **The service is stopped before files are replaced.** Replacing files under a
  running stack can cause strange failures later, when it tries to load a plugin
  that has been deleted.
* **RViz is off.** `RVIZ=false` is set in the settings file. Run RViz on your own
  machine with the same `ROS_DOMAIN_ID`.
* **The first boot may restart the service a few times.** `/dev/ldlidar` and the
  camera's USB device may not be ready yet when the service starts. The service
  is allowed ten restarts in two minutes. If it still fails, check the journal
  for the real error.

The systemd files are generated from templates. Edit `systemd/*.in` and
redeploy; do not edit the copies on the robot, because the next deploy
overwrites them.

## When every topic is listed but nothing arrives

Fast DDS uses shared memory between processes on the same machine.
`/etc/default/einride-mini-truck` sets:

```
FASTDDS_BUILTIN_TRANSPORTS=DEFAULT
```

**If shared memory fails, it fails silently.** Topics still show up in
`ros2 topic list`, Foxglove lists them, and `ros2 node list` looks normal - but
no messages get through. Processes running as different users may not be able
to share memory, so run `ros2` commands on the robot as the service user.

Both service wrappers run `fastdds shm clean` before starting, to remove
leftover shared memory from crashed processes.

If messages still do not arrive, set `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` in
`/etc/default/einride-mini-truck` and restart the service. All data then goes
over UDP, which uses more CPU for large messages such as camera images.

**How to tell a transport problem from a sensor problem:** subscribe to
`/tf_static`. `robot_state_publisher` publishes it once at startup and keeps it
available, so it must arrive right away. If it does not, the problem is the
transport, not the sensors:

```bash
ros2 topic echo /tf_static
```
