# Lidar

InnoMaker **LD19P**, a 360-degree DTOF scanner.

| | value | source |
|---|---|---|
| range | 0.02 - 12 m | datasheet v1.0, at 70% target reflectivity |
| scan rate | 9.89 Hz | measured (datasheet: 10 Hz typical, 5-13 Hz allowed) |
| sample rate | 4500 points/s, fixed | datasheet |
| points per revolution | 455 | 4500 / 9.89 |
| angular resolution | 0.79 deg | 360 / 455 |
| field of view | 360 deg | |

It publishes `/scan` and `/scan/points`. The device always samples at 4500 Hz,
so the number of points per turn depends on how fast it spins. If you change the
spin rate, update `samples` in `model.sdf` and `bins` in `config/ldlidar.yaml`
together, both set to 4500 / rate.

## Simulation matches the real LiDAR

The simulated `/scan` has the same values as the one the real driver publishes,
field by field:

| field | value (real and simulated) | set by |
|---|---|---|
| `angle_min` / `angle_max` | `0` .. `2*pi` | `model.sdf` |
| bins | 455 | `model.sdf` `<samples>` |
| `angle_increment` | 0.013840 | follows from the two above |
| no return | `NaN` | `ld19_scan_model` in simulation |
| `intensities` | a number where there was a return, `NaN` where not | `ld19_scan_model` in simulation |
| `scan_time` | ~0.1009 s | `ld19_scan_model` in simulation |
| `time_increment` | 2.2e-4 s | `ld19_scan_model` in simulation |
| `range_min` / `range_max` | 0.02 / 12.0 | `config/ldlidar.yaml` on hardware (the driver's own defaults are 0.03 / 15.0) |
| `frame_id` | `base_lidar_link` | `config/ldlidar.yaml` on hardware |

Two of these are easy to get wrong, and neither causes an error:

* **The sweep goes from 0 to 2\*pi.** Index 0 points straight ahead. If you
  assume -pi..+pi, every bearing will be off by 180 degrees.
* **A bin with no return is `NaN`, not `+inf`.** Check for it with `isnan`.
  A test like `r < scan.range_max` does not filter it out. About 17% of bins are
  empty indoors, so this happens all the time. See the `ld19_scan_model`
  docstring for details.

**Intensity in simulation is not real.** Gazebo does not model how strong a
return is. `ld19_scan_model` writes a fixed value (`scan_intensity:=200.0`,
inside the real device's 7..255 range) on bins that got a return, and `NaN` on
bins that did not. Which bins are empty matches the robot; the number itself
means nothing. Pass `scan_intensity:=0.0` to get plain Gazebo output.

To compare the two yourself:

```bash
ros2 launch einride_mini_truck_bringup simulation.launch.py headless:=true rviz:=false
# against
ros2 launch einride_mini_truck_bringup hardware.launch.py rviz:=false camera:=false
# then, in both:
ros2 topic echo /scan --once --field angle_min
ros2 topic hz /scan
```

Two tests check these values: `test_ld19_scan_model.py` in
`einride_mini_truck_gazebo` (no hardware needed) and `test_lidar_scan_contract.py`
in `einride_mini_truck_bringup` (skips itself when no lidar is connected).

## The lidar driver

`hardware.launch.py` uses only `ldlidar_component` from the
[`ldrobot-lidar-ros2`](https://github.com/Myzhar/ldrobot-lidar-ros2) driver. It
loads the component into a container with `config/ldlidar.yaml`. It does not use
the driver's `ldlidar_bringup.launch.py`, because that starts a second
`robot_state_publisher` that would conflict with this project's.

Two things to know about this driver:

* **It is a lifecycle node.** On its own it stays `unconfigured` and does
  nothing, with no error. `hardware.launch.py` runs a `nav2_lifecycle_manager`
  to start it.
* **It only reads the device while something subscribes to `~/scan`.** If you
  remap its output to `/scan`, it stops reading and publishes nothing, with no
  error. So `hardware.launch.py` keeps it on `/ldlidar_node/scan` and uses a
  `topic_tools relay` to copy it to `/scan`. The relay's subscription also keeps
  the driver reading.

To check the lidar is actually publishing:

```bash
ros2 topic hz /scan                    # ~9.9 Hz
```

## When the lidar produces nothing

Look out for this line, repeated every two seconds:

```
[lidar_lifecycle_manager]: Waiting for service ldlidar_node/get_state...
```

It means `ldlidar_node` was never loaded into its container. It is a loading
problem, not a lidar problem. Scroll up past the repeated lines to the
`[ldlidar_container] [ERROR]` line:

| error | cause |
|---|---|
| `Could not find requested resource in ament index` | `ldlidar_component` is not installed - re-run `provision.sh` |
| `libldlidar.so: cannot open shared object file` | the driver's SDK library is not installed - re-run `provision.sh` |

If the driver loaded but cannot reach the device, you instead get
`Failed to change state for node: ldlidar_node` from the lifecycle manager, after
the driver logs which serial port it could not open.
