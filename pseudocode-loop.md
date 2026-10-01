# always on, started by [app.launch.py](http://app.launch.py)

every 30 ms:
    speed = wheel_radius * average encoder change
    yaw_rate = gyro_z - bias_measured_while_still
    /odom = kalman_filter(speed, yaw_rate)          # ekf_node

every lidar scan:
    paint hits into a grid around the robot
    if a commanded velocity enters that grid: slow it or zero it

every 0.5 s with no /cmd_vel:
    motors = 0                                       # hardware watchdog

# saga/node.py — the only contact with the "AI"

every 1 s:
    reply = GET /api/v1/route
    if reply failed: keep the last command
    else if event is running and next_dock exists:
        publish /saga/next_tag = next_dock.tag_id
    else:
        publish /saga/next_tag = -1                  # stay still

# mission/node.py — one tick every 0.2 s

wait until Nav2 is up
loop:
    requested = /saga/next_tag, or None if it is -1
    action = mission.step(now, requested, nav2_finished, nav2_succeeded)

```
if action is dock(tag):
    publish /mission/target_tag = tag            # perception tracks this tag
    nav2.dock(dock_<tag>, drive_to_staging_first=true)
if action is undock:
    nav2.undock()
if action is cancel:
    nav2.cancel()
```



# what dock() does inside Nav2, not in your code

dock(tag):
    goal = staging pose, 0.7 m in front of that dock   # from config/docks/*.yaml
    path = A*(lidar grid, robot pose, goal)
    follow path at 20 Hz until the staging pose
    until within 3 cm of the tag pose, or timeout:
        pose = april_tag(camera, tag) expressed in base_footprint
        publish a small velocity toward pose
    report success or failure

# mission/mission.py — the decision, and nothing else

step(now, requested, task_done, task_ok):
    IDLE:
        if requested is a tag:  go DOCKING, return dock(requested)

```
DOCKING:
    if requested changed:   go IDLE, return cancel
    if task_done and ok:    go DOCKED
    if task_done and failed: go RETRY_WAIT

DOCKED:
    if requested is a different tag: go UNDOCKING, return undock
    if requested is this same tag for 30 s:
        go UNDOCKING, return undock              # server never counted the arrival
    else: stay                                   # loading or unloading

UNDOCKING:
    if task_done: go IDLE, then run step() again # so the next dock starts now

RETRY_WAIT:
    if 2 s have passed: go IDLE, then run step() again
```

