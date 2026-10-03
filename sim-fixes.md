# Running the simulation on a Mac (Docker)

The sim runs in Docker with no GPU, so a few adjustments are needed. Two are
committed (hardware-safe); one is kept **local in a git stash**.

## Launch flags (committed, hardware-safe)

Run the app against the sim with:

    ros2 launch einride_mini_truck_application app.launch.py \
        layout:=arena use_sim_time:=true localization:=false

- **`use_sim_time:=true`** - take time from Gazebo's `/clock`. Without it every
  autonomy node runs on wall-clock and drops all sim-stamped TF/scans
  (`TF_OLD_DATA`), so nothing moves. Default `false` (correct on the robot).
- **`localization:=false`** - skip the app's EKF. In sim, Gazebo's DiffDrive
  already publishes `/odom` + `odom -> base_footprint`; running the EKF too gives
  two conflicting sources and the pose jumps. Default `true` (no Gazebo on the robot).

Both default to hardware behaviour, so the robot is unaffected.

## Camera resolution: LOCAL stash, not committed

A GPU-less Mac software-renders the 1280x720 cameras at ~0.4 Hz, so AprilTag gets
no frames and docking can't see the dock. The fix lowers them to 640x360 @ 10 Hz
in `model.sdf`. This is **sim-only** (hardware uses the real OAK-D driver and
ignores the SDF `<sensor>` blocks), so it is kept **out of git**, in a stash:

    git stash list            # stash@{0}: sim-only camera 640x360@10Hz ...
    git stash apply           # apply before running the Mac sim (keeps the stash)

After applying, rebuild the description and restart the sim:

    colcon build --packages-select einride_mini_truck_description
    # then Ctrl+C and relaunch simulation.launch.py

## gz_odometry switch (committed; optional, for the EKF in sim)

`simulation.launch.py gz_odometry:=false` makes Gazebo give up `/odom` so the
app's real EKF can own it (`localization:=true`) - the hardware-faithful stack.
The EKF is too heavy for the GPU-less Mac (it can't hold 30 Hz and Nav2 falls
behind), so use gz odom on the Mac; the switch is there for a GPU machine/robot.
Default `gz_odometry:=true` = current behaviour. It lives only in the sim launch,
so it never runs on the robot.

## DDS (needed for SLAM / when discovery is flaky)

Fast DDS's shared-memory transport corrupts in Docker (stale `/dev/shm/fastrtps_*`
after hard-killed processes), breaking discovery. Use CycloneDDS in **every**
shell:

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # add to ~/.bashrc so all shells agree

See `slam.md` for the full SLAM mapping/localisation workflow.
