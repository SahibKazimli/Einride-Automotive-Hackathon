# Watching the robot from Foxglove

`provision.sh` installs `foxglove_bridge`, and `deploy.sh` installs a service,
`einride-mini-truck-foxglove`, that runs it. In the Foxglove app, connect to:

```
ws://<robot-ip>:8765
```

This is a plain WebSocket on the robot's local network. **Nothing connects to
Foxglove's servers** - no account or internet access is needed.

Its settings are in `/etc/default/einride-mini-truck`, the same file as the main
stack: `FOXGLOVE_PORT`, `FOXGLOVE_ADDRESS`, and `FOXGLOVE_EXTRA_ARGS` for extra
launch arguments. Using one file keeps `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`
the same for both services; a bridge on a different domain sees nothing.

The bridge has no password. Anyone on the robot's network can connect.

The Foxglove service starts after the main stack but does not depend on it.
Restarting `einride-mini-truck` does not drop your Foxglove connection, and when
the stack is not running the bridge simply shows no topics.

On a slow connection the camera topics use most of the bandwidth. Limit the
topics with a whitelist:

```
FOXGLOVE_EXTRA_ARGS="topic_whitelist:=['/scan','/tf','/imu','/joint_states']"
```

## `robot_description` is URDF, generated from SDF

`model.sdf` is the only robot description in the project. At build time,
`einride_mini_truck_description/tools/sdf_to_urdf.cpp` converts it to
`model.urdf`, and `/robot_description` publishes that URDF.

This is for Foxglove. Its 3D panel only reads URDF; given SDF, it shows nothing
and reports no error. `robot_state_publisher` and RViz can read either format,
and Gazebo loads `model.sdf` directly.

The converter fixes two problems in the output:

* `sdformat_urdf` adds ambient and diffuse alpha together, giving `alpha 1.2`.
  urdfdom rejects that and drops the link's colour, so the tool corrects it.
* urdfdom writes an empty `<texture/>` for every material, which the tool
  removes.

`test/test_urdf_export.py` checks both, and checks that link and joint names are
kept.

URDF cannot describe sensors, so they are left out. The build logs
`has a <sensor>, but URDF does not support this` once per sensor link. Nothing
reads sensors from this topic.
