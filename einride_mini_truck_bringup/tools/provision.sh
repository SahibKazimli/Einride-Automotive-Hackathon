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
# Brings a robot from a bare JetPack 7.2 (Ubuntu 24.04 / noble) install to one
# that deploy.sh can deploy to, installing ROS 2 Jazzy itself along the way if
# it is not already there. Run it once per robot, then use deploy.sh.
#
#     ./provision.sh jetson@192.168.1.42
#     ./deploy.sh    jetson@192.168.1.42
#
# It deliberately does nothing deploy.sh does. It does not install the service,
# does not ship the application, the hardware abstraction layer or the launch
# files, and does not start anything - all of that belongs to deploy.sh, which
# will be run many times after this is run once. What is left here is only the
# things that have to exist before a deploy can work at all: the dependencies,
# the compiled lidar driver, the robot description, the udev rules, and the
# install prefix itself.
#
# Nothing is cross-compiled, because almost nothing needs compiling. Of the
# packages the robot runs, exactly one contains compiled code - ldlidar_component
# and its vendored SDK - and that is built on the robot, natively, where the
# architecture question does not arise. Everything else is Python modules,
# launch files, parameter YAMLs and a robot description: architecture-independent
# files that are simply copied.
#
# The install tree does not care where it was built. colcon bakes absolute paths
# into its generated environment hooks, but only as fallbacks - the top-level
# setup.bash computes the real prefix from its own location at source time, and
# discovers packages by scanning the prefix rather than from a baked list. So a
# package directory built anywhere can be dropped into a prefix built somewhere
# else and is found correctly. That is what lets these two halves meet.
#
# einride_mini_truck_gazebo is deliberately not shipped. It is the only other
# package with compiled code, hardware.launch.py never loads it - the sole
# mention of ros_gz in that file is a comment - and keeping it off the robot is
# what keeps Gazebo, Ogre and their thousand-package dependency tree off it too.
#
# What ends up on the robot:
#   ros-<distro>-ros-base                 only if ROS 2 was not already there
#   apt packages                          resolved by rosdep, on the robot
#   <prefix>/install                      the lidar driver, compiled here, and
#                                         the robot description
#   /etc/udev/rules.d/99-einride-*.rules  /dev/ldlidar, /dev/ugv02, and the
#                                         camera
#
# The service, the launch files, the application and the hardware abstraction
# layer arrive with the first deploy.sh.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

UDEV_DIR="$SCRIPT_DIR/../udev"

# The colcon workspace this repository sits in.
# tools -> bringup -> repo -> src -> workspace.
WORKSPACE="$(cd -- "$SCRIPT_DIR/../../../.." && pwd)"

# Built on the robot: the only package the robot runs that has compiled code.
# Not a released ROS package, so there is no arm64 binary of it anywhere to
# install instead - see "The LiDAR driver" in the README.
LIDAR_SRC="ldrobot-lidar-ros2"

# Built here and copied. Only the description: it holds no compiled code, so
# where it was built does not matter, and it changes about as often as the robot
# is rebuilt - which makes it this script's rather than deploy.sh's. The three
# packages that change while you work are deploy.sh's alone, and are not touched
# here.
HOST_PACKAGES=(
    einride_mini_truck_description
)

# rosdep still needs to see every manifest, though, not just the ones shipped
# here: the robot's runtime dependencies - the camera driver, the lidar
# lifecycle manager, the serial library - are declared by packages deploy.sh
# ships. Installing them is preparation; installing the packages themselves is
# not.
MANIFEST_PACKAGES=(
    einride_mini_truck_description
    einride_mini_truck_application
    einride_mini_truck_hardware
    einride_mini_truck_bringup
)

# Simulation-only, and reachable from no configuration of hardware.launch.py.
# bringup declares all three as full <depend>s because simulation.launch.py
# needs them, so rosdep would otherwise pull the whole of Gazebo Harmonic - about
# a gigabyte - onto a robot that will never run a simulator. Skipped by key,
# which is rosdep's own mechanism for exactly this.
#
# einride_mini_truck_gazebo is in the list for a second reason as well. It is a
# workspace package, which rosdep normally satisfies from source via
# --ignore-src, but its manifest is deliberately not among the ones shipped -
# so without this, rosdep looks for an apt package by that name and fails.
SKIP_KEYS="ros_gz_sim ros_gz_bridge einride_mini_truck_gazebo"

UNINSTALL=0

usage() {
    cat <<USAGE
Prepare a robot so that deploy.sh can deploy to it: install the runtime
dependencies, build the lidar driver on the robot, and install the robot
description and the udev rules. Run once per robot, then use deploy.sh - which
installs the service, the launch files, the application and the hardware
abstraction layer, and starts it.

Usage:
  $(basename "$0") [--uninstall | --dry-run] <user@host>

  e.g. $(basename "$0") jetson@192.168.1.42

Options:
  --uninstall  remove both services, $PREFIX and the udev rules.
               /etc/default/$SERVICE_NAME and the apt packages are left alone.
  --dry-run    print what would run on the robot, and run none of it.
  -h, --help   this text.

Everything else is derived: it installs to $PREFIX and takes
the ssh port and key from ~/.ssh/config. Without a user it connects as \\$USER,
the same as ssh would; the robot's account is usually 'jetson'.

Nothing is cross-compiled. $LIDAR_SRC is built on the robot -
it is the only package with compiled code that the robot runs - and the robot
description is architecture-independent files built here and copied.

If ROS 2 $ROS_DISTRO_NAME is not already on the robot this installs it -
ros-$ROS_DISTRO_NAME-ros-base from packages.ros.org, which only ships real
arm64 binaries for Ubuntu 24.04 / JetPack 7.2. Nothing is started: after this,
run deploy.sh.
USAGE
}

while [ $# -gt 0 ]; do
    case "$1" in
        --uninstall) UNINSTALL=1; shift ;;
        --dry-run)   DRY_RUN=1; shift ;;
        -h|--help)   usage; exit 0 ;;
        --)          shift; break ;;
        -*)          die "unknown option: $1

provision.sh takes only --uninstall and --dry-run - see --help.
The port and key go in ~/.ssh/config; everything else is derived." ;;
        *)           break ;;
    esac
done

[ $# -le 1 ] || die "unexpected extra argument: $2 (usage: $(basename "$0") <user@host>)"
parse_target "${1:-}" "./$(basename "$0") jetson@192.168.1.42"

validate_common
setup_ssh
require_reachable

# ---------------------------------------------------------------------------
# uninstall
# ---------------------------------------------------------------------------
if [ "$UNINSTALL" -eq 1 ]; then
    note "removing $SERVICE_NAME from $ROBOT_IP"
    run_remote "$(cat <<REMOTE
set -e
for svc in '$SERVICE_NAME' '$SERVICE_NAME-foxglove'; do
    sudo systemctl disable --now "\$svc" 2>/dev/null || true
    sudo rm -f "/etc/systemd/system/\$svc.service"
done
sudo rm -rf '/usr/local/lib/$SERVICE_NAME'
sudo rm -f /etc/udev/rules.d/99-einride-ldlidar.rules \\
           /etc/udev/rules.d/99-einride-oak-d.rules \\
           /etc/udev/rules.d/99-einride-ugv02.rules
sudo udevadm control --reload-rules 2>/dev/null || true
sudo rm -rf '$PREFIX'
sudo systemctl daemon-reload
sudo systemctl reset-failed '$SERVICE_NAME' '$SERVICE_NAME-foxglove' 2>/dev/null || true
echo "removed both services, $PREFIX and the udev rules."
echo "/etc/default/$SERVICE_NAME and the apt packages were left in place."
REMOTE
)"
    exit 0
fi

# ---------------------------------------------------------------------------
# preflight
# ---------------------------------------------------------------------------
[ -d "$WORKSPACE/src/$LIDAR_SRC" ] || die \
"no $WORKSPACE/src/$LIDAR_SRC.

The lidar driver is built from source on the robot, so its source has to be here
first. It lives beside this repository in the workspace:

    cd $WORKSPACE/src
    git clone --recursive https://github.com/Myzhar/ldrobot-lidar-ros2.git

--recursive matters: the vendor SDK is a submodule."

note "inspecting $ROBOT_IP"
inspect_target
echo "    arch: $ROBOT_ARCH"
echo "    os:   $ROBOT_OS_ID $ROBOT_OS_VERSION"

# ROS itself is not shipped - only the workspace that sits on top of it. If it
# is already on the robot the remote install step below is a no-op; if not, it
# is installed from the stock ROS apt repo. That only has real arm64 binaries
# for $ROS_DISTRO_NAME on Ubuntu 24.04 (noble) - which is what JetPack 7.2 is -
# so anything else fails loudly here rather than failing obscurely mid-install.
if ssh_query "test -r '/opt/ros/$ROS_DISTRO_NAME/setup.bash'"; then
    note "ROS 2 $ROS_DISTRO_NAME already on $ROBOT_IP"
elif [ "$ROBOT_OS_ID" = ubuntu ] && [ "$ROBOT_OS_VERSION" = "24.04" ]; then
    note "no ROS 2 $ROS_DISTRO_NAME on $ROBOT_IP yet - installing it below"
else
    die \
"no ROS 2 $ROS_DISTRO_NAME on $ROBOT_IP, and this only knows how to install it on
Ubuntu 24.04 / JetPack 7.2 (found $ROBOT_OS_ID $ROBOT_OS_VERSION instead).

Install ROS 2 $ROS_DISTRO_NAME on the robot by hand - see docs.ros.org - then
run this again."
fi

# ---------------------------------------------------------------------------
# build what does not need the robot, here
# ---------------------------------------------------------------------------
# Every package whose manifest is shipped is built, not just the one whose
# install directory is. The manifests are read out of the install tree, so a
# package that is not rebuilt here hands rosdep whatever it was told last time -
# and a dependency added to a package.xml would be silently ignored until some
# unrelated build happened to refresh it. That is a slow, confusing failure: the
# robot comes up missing something nobody can see is missing.
note "building ${MANIFEST_PACKAGES[*]}"
(
    # The ROS and colcon setup scripts read variables they do not always define,
    # so -u has to be off while they are sourced.
    set +u
    if [ -z "${ROS_DISTRO:-}" ]; then
        [ -r "/opt/ros/$ROS_DISTRO_NAME/setup.bash" ] \
            || die "no ROS 2 $ROS_DISTRO_NAME on this machine to build with"
        # shellcheck source=/dev/null
        source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"
    fi
    cd "$WORKSPACE"
    colcon build --packages-select "${MANIFEST_PACKAGES[@]}"
) || die "the local build failed; nothing was shipped"

# ---------------------------------------------------------------------------
# stage
# ---------------------------------------------------------------------------
stage="$(mktemp -d)"
trap 'rm -rf "$stage"' EXIT

mkdir -p "$stage/udev" "$stage/packages" "$stage/manifests"
cp "$UDEV_DIR"/*.rules "$stage/udev/"

# The lidar goes over as source, to be compiled there. build/ and install/ are
# stripped: a host build directory would hand CMake a cache full of this
# machine's paths and its own architecture.
rsync -a --delete \
    --exclude='.git/' --exclude='build/' --exclude='install/' --exclude='log/' \
    --exclude='__pycache__/' \
    "$WORKSPACE/src/$LIDAR_SRC/" "$stage/lidar_src/"

compiled=""
for pkg in "${HOST_PACKAGES[@]}"; do
    [ -d "$WORKSPACE/install/$pkg" ] || die "the build produced no $WORKSPACE/install/$pkg"
    # -L dereferences symlinks, and that is load-bearing rather than tidy.
    # `colcon build --symlink-install` - a normal thing to do while developing -
    # makes the install tree point at build/ and src/ instead of holding copies,
    # so local_setup.bash and package.xml become links into this machine's home
    # directory. Shipped as links they arrive dangling, and the robot then
    # reports "not found" for the setup script and drops the package off
    # AMENT_PREFIX_PATH - a package that is present, complete, and invisible.
    # Copying what they point at makes the payload self-contained however the
    # workspace happened to be built.
    rsync -aL --exclude='__pycache__' "$WORKSPACE/install/$pkg/" "$stage/packages/$pkg/"

    # The invariant these packages are shipped on. If one of them grows C++, a
    # build from this machine is the wrong architecture for the robot and it has
    # to move over to being built there, like the lidar is.
    while IFS= read -r f; do
        case "$(file -b "$f" 2>/dev/null)" in
            ELF*) compiled="$compiled  $pkg: ${f#"$stage/packages/$pkg/"}"$'\n' ;;
        esac
    done < <(find "$stage/packages/$pkg" -type f)

done

# rosdep reads package.xml, and these are the manifests for every package the
# robot will run - including the ones deploy.sh ships, whose runtime
# dependencies still have to be installed here. Copied on their own so that
# rosdep sees what the robot needs without colcon seeing anything it would try
# to build.
for pkg in "${MANIFEST_PACKAGES[@]}"; do
    m="$WORKSPACE/install/$pkg/share/$pkg/package.xml"
    [ -r "$m" ] || die \
"no manifest at $m.

rosdep needs every package's manifest to work out what the robot has to have
installed. Build the workspace here first:
    cd $WORKSPACE && colcon build"
    mkdir -p "$stage/manifests/$pkg"
    cp "$m" "$stage/manifests/$pkg/"
done

if [ -n "$compiled" ]; then
    die "these packages contain compiled binaries, built here for $(dpkg --print-architecture 2>/dev/null || uname -m):

$compiled
They are shipped as files, which only works while they hold no compiled code.
Whichever one grew it needs to move to being built on the robot, as
$LIDAR_SRC is."
fi

if [ "$DRY_RUN" -eq 0 ]; then
    note "copying $(du -sh "$stage" | cut -f1) to $ROBOT_IP"
fi
remote_tmp="$(ship_stage "$stage")"

# ---------------------------------------------------------------------------
# install
# ---------------------------------------------------------------------------
note "provisioning $ROBOT_IP"
run_remote "$(cat <<REMOTE
set -e
TMP='$remote_tmp'

echo "== ROS 2 $ROS_DISTRO_NAME =="
if [ -r '/opt/ros/$ROS_DISTRO_NAME/setup.bash' ]; then
    echo "  already installed"
else
    # JetPack 7.2 is Ubuntu 24.04 (noble) - $ROS_DISTRO_NAME's own target
    # platform - so this is the stock ROS apt repo with real arm64 binaries,
    # not a source build. universe has to be enabled first: some of Jazzy's
    # own dependencies live there, and a bare JetPack image does not turn it
    # on by default.
    echo "  installing from packages.ros.org"
    sudo apt-get update -qq
    sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \\
        software-properties-common ca-certificates curl gnupg
    sudo add-apt-repository -y universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \\
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$VERSION_CODENAME) main" \\
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update -qq
    # ros-base, not ros-jazzy-desktop: this is a headless robot, and rviz2 -
    # the one GUI package it can optionally run - arrives separately below,
    # pulled in by rosdep from bringup's own exec_depend.
    sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \\
        ros-$ROS_DISTRO_NAME-ros-base
fi

set +u
source /opt/ros/$ROS_DISTRO_NAME/setup.bash
set -u

# Stopped before anything is replaced, on a re-provision where the service
# already exists. Overwriting the install tree under a running stack leaves it
# holding deleted inodes - it keeps working until something tries to dlopen a
# plugin it has not loaded yet, and then fails in a way that has nothing to do
# with the change that caused it.
#
# Left stopped, deliberately. Starting it again is deploy.sh's job, and this
# script has just removed the packages deploy.sh owns - so there would be
# nothing coherent to start until a deploy has run.
#
# Unconditional, not guarded by is-active: a crash-looping unit sits in
# "activating (auto-restart)" between attempts, which is-active reports as
# false, and a guarded stop would leave the pending restart armed to fire
# part-way through the copy.
echo "stopping the services until the next deploy"
for svc in '$SERVICE_NAME' '$SERVICE_NAME-foxglove'; do
    sudo systemctl stop "\$svc" 2>/dev/null || true
done

echo "== dependencies =="
sudo apt-get update -qq
# Needed to compile the lidar here. The ROS base image on a robot has the
# runtime but not always the toolchain.
#
# \`sudo env VAR=...\`, not \`sudo VAR=... cmd\`: sudo only accepts the latter when
# sudoers permits setting that variable, and the default env_reset policy does
# not - it fails with "you are not allowed to set the following environment
# variables". env is a plain binary and always works.
#
# ros2cli-common-extensions is what makes the ros2 command actually useful on the
# robot: the ROS base install ships ros2cli itself but not the verbs you reach
# for when something is wrong - ros2 topic, node, param, service, action. Those
# live in separate packages that this metapackage pulls in. Installed here rather
# than left to whoever is debugging, because the moment you need them you are
# already ssh'd into a robot that is misbehaving.
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \\
    build-essential cmake git python3-colcon-common-extensions python3-rosdep \\
    ros-$ROS_DISTRO_NAME-ros2cli-common-extensions

# Tolerated rather than guarded by a test for rosdep's sources list, whose path
# is an implementation detail: on a robot where ROS was installed normally it is
# already initialised and this is a no-op, and the ways it can fail - already
# done, no network, no permission - are all either harmless here or reported by
# the update that follows.
sudo rosdep init > /dev/null 2>&1 || true
rosdep update --rosdistro "$ROS_DISTRO_NAME"

# One pass over both: the lidar source, which needs its build dependencies, and
# the manifests of the packages copied in as files, which need their exec ones.
# --skip-keys keeps the simulation-only entries out - see SKIP_KEYS above.
rosdep install --from-paths "\$TMP/lidar_src" "\$TMP/manifests" --ignore-src -y \\
    --rosdistro "$ROS_DISTRO_NAME" --skip-keys "$SKIP_KEYS"

echo "== device access =="
# The chassis UART is root:dialout and the udev rule above puts /dev/ldlidar in
# the same group, so without this membership the service starts, retries, and
# logs "Permission denied" against both devices forever - which reads like
# broken hardware rather than a missing group.
#
# Granted here rather than warned about, because a robot whose account cannot
# open its own serial ports is not provisioned. systemd resolves supplementary
# groups when it starts a unit, so a restart is enough to pick this up - no
# logout required.
for g in dialout video; do
    if id -nG | tr " " "\n" | grep -qx "\$g"; then
        echo "  already in \$g"
    else
        sudo usermod -aG "\$g" "\$(id -un)"
        echo "  added \$(id -un) to \$g"
    fi
done

echo "== building $LIDAR_SRC on the robot =="
# Built in the staging directory as the login user rather than as root in
# \$PREFIX. The result is copied into place afterwards, which keeps \$PREFIX a
# pure install tree - no src/, no build/, no log/ - and keeps the compiler out
# of root's hands.
mkdir -p "\$TMP/ws/src"
cp -a "\$TMP/lidar_src" "\$TMP/ws/src/$LIDAR_SRC"
( cd "\$TMP/ws" && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release )

echo "== assembling $PREFIX/install =="
# Replaced wholesale: a package removed from the workspace would otherwise keep
# running from the leftovers of the previous provision. deploy.sh is the one
# that updates packages in place and leaves the rest alone.
sudo rm -rf '$PREFIX/install'
sudo mkdir -p '$PREFIX'
# The setup scripts come from the robot's own colcon build, so they are correct
# for this prefix. They enumerate packages by scanning the prefix when sourced,
# not from a list fixed at build time, which is exactly why the packages copied
# in below are found even though this build never saw them.
sudo cp -a "\$TMP/ws/install" '$PREFIX/install'
for pkg in ${HOST_PACKAGES[*]}; do
    sudo cp -a "\$TMP/packages/\$pkg" "$PREFIX/install/\$pkg"
done
# Owned by root and read-only to the service: the account the robot is driven
# from has no reason to be able to rewrite the code it runs.
sudo chown -R root:root '$PREFIX'
sudo chmod -R go-w '$PREFIX'
echo "installed: \$(ls '$PREFIX/install' | grep -v '^[._]' | grep -v setup | tr '\\n' ' ')"

echo "== udev =="
sudo install -m 0644 "\$TMP"/udev/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
# Applies the rules to devices that are already plugged in. Without it the
# symlink and the permissions only appear on the next replug or reboot, which
# looks like the rules not having been installed at all.
sudo udevadm trigger --subsystem-match=tty --subsystem-match=usb

rm -rf "\$TMP"
REMOTE
)"

if [ "$DRY_RUN" -eq 1 ]; then
    exit 0
fi

cat <<DONE

Prepared $SSH_USER@$ROBOT_IP.

Nothing is running yet - the service, the launch files, the application and the
hardware abstraction layer arrive with the first deploy:

  ./deploy.sh $SSH_USER@$ROBOT_IP

Come back to this script when an apt dependency, a udev rule, the lidar driver
or the robot description changes.
DONE
