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
# ExecStart for the einride-mini-truck-foxglove service: source ROS and the
# workspace, then hand the process over to foxglove_bridge.
#
# Same shape and the same reasons as run-hardware.sh beside it - systemd execs a
# binary rather than running a shell, so there is nowhere in the unit file to
# source setup.bash, and without it ros2 is not even on PATH.

# No -u: the ROS and colcon setup scripts read variables they do not always
# define, so -u turns sourcing them into an immediate failure. It is enabled
# below, once sourcing is done.
set -eo pipefail

die() { echo "run-foxglove.sh: $*" >&2; exit 1; }

ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/jazzy/setup.bash}"
[ -n "${WORKSPACE:-}" ] || die "WORKSPACE is not set - is /etc/default/<service> installed?"

# Checked before sourcing rather than after: `source` on a missing file under -e
# aborts with a bare "No such file or directory" and no indication of which of
# the two files, or that a service wrapper was even involved.
[ -r "$ROS_DISTRO_SETUP" ] || die "no ROS installation at $ROS_DISTRO_SETUP"
WORKSPACE_SETUP="$WORKSPACE/install/setup.bash"
[ -r "$WORKSPACE_SETUP" ] || die "workspace at $WORKSPACE is not installed: no $WORKSPACE_SETUP"

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

# The bridge is an apt package, not part of this workspace, so a missing one is
# a provisioning problem rather than a deploy problem - and saying so is more
# use than the launch failure that would otherwise follow.
#
# Checked against the ament index directly rather than with `ros2 pkg prefix`.
# That command lives in ros2pkg, which a minimal ROS install on a robot need not
# have - and a guard that reports a missing package because the tool used to
# look for it is missing is worse than no guard at all. This is how ament finds
# packages anyway.
foxglove_found=0
IFS=':' read -r -a _prefixes <<< "${AMENT_PREFIX_PATH:-}"
for _p in "${_prefixes[@]}"; do
    if [ -f "$_p/share/ament_index/resource_index/packages/foxglove_bridge" ]; then
        foxglove_found=1
        break
    fi
done
[ "$foxglove_found" -eq 1 ] || die \
"foxglove_bridge is not installed on this robot.
It is installed by provision.sh; run that again, or install it by hand:
    sudo apt install ros-\${ROS_DISTRO}-foxglove-bridge"

# 0.0.0.0 because the point is to reach it from a laptop. The bridge has no
# authentication of its own, so what keeps it private is the network the robot
# is on - set FOXGLOVE_ADDRESS to 127.0.0.1 and use an ssh tunnel if that is not
# good enough.
args=(
    "port:=${FOXGLOVE_PORT:-8765}"
    "address:=${FOXGLOVE_ADDRESS:-0.0.0.0}"
)

# Word splitting is the point here, so this is not the usual unquoted-variable
# mistake: FOXGLOVE_EXTRA_ARGS holds a whole argument list, not one argument.
if [ -n "${FOXGLOVE_EXTRA_ARGS:-}" ]; then
    read -r -a extra_args <<< "${FOXGLOVE_EXTRA_ARGS}"
    args+=("${extra_args[@]}")
fi

echo "starting: ros2 launch foxglove_bridge foxglove_bridge_launch.xml ${args[*]}"

# exec, so that the launch becomes the service's main process. Without it this
# shell stays in the middle and systemd's SIGINT is delivered to bash instead of
# the process that knows how to shut the node down.
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml "${args[@]}"
