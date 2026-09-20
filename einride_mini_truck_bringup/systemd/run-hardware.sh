#!/usr/bin/env bash
# Copyright 2025 Einride AB
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ExecStart for the einride-mini-truck service: source ROS and the workspace,
# then hand the process over to hardware.launch.py.
#
# This exists because systemd execs a binary rather than running a shell, so
# there is nowhere in the unit file to source setup.bash - and without it `ros2`
# is not even on PATH. Everything it reads comes from /etc/default/<service>,
# via the unit's EnvironmentFile.
#
# Installed to /usr/local/lib/<service>/run-hardware.sh by provision.sh, and
# refreshed by deploy.sh. Run it by hand to reproduce exactly what the service
# does:  sudo -u <user> systemd-run --pty --property=EnvironmentFile=...

# No -u: the ROS and colcon setup scripts read variables they do not always
# define, so -u turns sourcing them into an immediate failure. It is enabled
# below, once sourcing is done and this script's own variables are what matter.
set -eo pipefail

die() { echo "run-hardware.sh: $*" >&2; exit 1; }

ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/jazzy/setup.bash}"
[ -n "${WORKSPACE:-}" ] || die "WORKSPACE is not set - is /etc/default/<service> installed?"

# Checked before sourcing rather than after: `source` on a missing file under
# -e aborts with a bare "No such file or directory" and no indication of which
# of the two files, or that a service wrapper was even involved.
[ -r "$ROS_DISTRO_SETUP" ] || die "no ROS installation at $ROS_DISTRO_SETUP"
WORKSPACE_SETUP="$WORKSPACE/install/setup.bash"
[ -r "$WORKSPACE_SETUP" ] || die "workspace at $WORKSPACE is not built: no $WORKSPACE_SETUP"

# shellcheck source=/dev/null
source "$ROS_DISTRO_SETUP"
# shellcheck source=/dev/null
source "$WORKSPACE_SETUP"

set -u

command -v ros2 >/dev/null || die "ros2 not on PATH after sourcing $ROS_DISTRO_SETUP"

# Fast DDS leaves shared-memory segments behind when a process dies without
# cleaning up - a crash, a SIGKILL, a power cut. The next start then finds the
# segments, fails to initialise SHM, and says nothing about it: discovery still
# works over UDP multicast, so every topic appears in `ros2 topic list` and in
# Foxglove, while no message ever crosses a process boundary. A robot that looks
# entirely healthy and publishes nothing.
#
# `shm clean` removes only segments whose owning process is gone, so it is safe
# to run while other nodes are live. Tolerated if it fails, and silenced when it
# has nothing to do: this is hygiene, not a precondition.
fastdds shm clean > /dev/null 2>&1 || true

# rviz defaults to false here where the launch file defaults it to true: the
# launch file is written for a developer at a desk, this wrapper only ever runs
# on a headless robot.
args=(
    "rviz:=${RVIZ:-false}"
    "lidar:=${LIDAR:-true}"
    "camera:=${CAMERA:-true}"
)

# Unset and empty both mean "use the launch file's default", which is why these
# are appended conditionally rather than passed through as an empty string -
# hardware_params:= with nothing after it is a parameter file named "".
if [ -n "${HARDWARE_PARAMS:-}" ]; then
    args+=("hardware_params:=${HARDWARE_PARAMS}")
fi
if [ -n "${LIDAR_PARAMS:-}" ]; then
    args+=("lidar_params:=${LIDAR_PARAMS}")
fi

# Word splitting is the point here, so this is not the usual unquoted-variable
# mistake: EXTRA_LAUNCH_ARGS holds a whole argument list, not one argument.
if [ -n "${EXTRA_LAUNCH_ARGS:-}" ]; then
    read -r -a extra_args <<< "${EXTRA_LAUNCH_ARGS}"
    args+=("${extra_args[@]}")
fi

echo "starting: ros2 launch einride_mini_truck_bringup hardware.launch.py ${args[*]}"

# exec, so that ros2 launch becomes the service's main process. Without it this
# shell stays in the middle, and systemd's SIGINT - the one thing that gets an
# orderly node shutdown, see KillSignal in the unit - is delivered to bash,
# which is not the process that knows how to stop the robot.
exec ros2 launch einride_mini_truck_bringup hardware.launch.py "${args[@]}"
