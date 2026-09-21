#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "$SCRIPT_DIR/_common.sh"

SYSTEMD_DIR="$SCRIPT_DIR/../systemd"
SRC_UNIT="$SYSTEMD_DIR/einride-mini-truck.service.in"
SRC_FOXGLOVE_UNIT="$SYSTEMD_DIR/einride-mini-truck-foxglove.service.in"
SRC_ENV="$SYSTEMD_DIR/einride-mini-truck.env.in"
SRC_WRAPPER="$SYSTEMD_DIR/run-hardware.sh"
SRC_FOXGLOVE_WRAPPER="$SYSTEMD_DIR/run-foxglove.sh"

# The colcon workspace this repository sits in, whose install/ is shipped.
# tools -> bringup -> repo -> src -> workspace.
WORKSPACE="$(cd -- "$SCRIPT_DIR/../../../.." && pwd)"

# What changes while you work. einride_mini_truck_bringup is here because the
# launch files and the parameter YAMLs are edited as often as the code is, and a
# deploy that silently did not pick up a change to hardware.launch.py would be a
# trap.
#
# einride_mini_truck_description is here for the same reason, and for a sharper
# one. provision.sh builds it on the robot, so it used to be left out; but the
# frames the camera driver publishes and the frames model.sdf publishes must not
# overlap, and that agreement lives half in each package. Ship bringup alone and
# the robot runs a new oak_d_lite.yaml against an old model.urdf, which puts two
# publishers on one child frame - a race tf2 resolves by arrival order, per
# subscriber, without warning.
DEV_PACKAGES=(
    einride_mini_truck_application
    einride_mini_truck_hardware
    einride_mini_truck_bringup
    einride_mini_truck_description
)

# .pyc files record the path they were compiled from and Python regenerates them
# on the robot anyway, so bytecode is the one thing not worth copying.
#
# Nothing else is filtered. colcon does bake this machine's install path into a
# few generated files, but only as a fallback: the top-level setup.bash computes
# the real prefix from its own location when sourced, and finds packages by
# scanning the prefix rather than from a list fixed at build time. A package
# directory built here is therefore correct wherever it lands, which is the same
# property that lets provision.sh drop these packages into a prefix built on the
# robot.

usage() {
    cat <<USAGE
Build the packages you are working on, copy them to the robot, restart the
service.

Usage:
  $(basename "$0") <user@host>

  e.g. $(basename "$0") jetson@192.168.1.42

Ships: ${DEV_PACKAGES[*]}
       plus both systemd units - the stack and the Foxglove bridge - and their
       wrappers. An existing /etc/default/$SERVICE_NAME is left alone.

Run provision.sh first - it installs the apt packages and udev rules, builds the
lidar driver on the robot, and sets the service up. Run it again whenever a
package is added to or removed from the workspace; this script updates packages
in place and cannot create an installation.
USAGE
}

case "${1:-}" in
    -h|--help) usage; exit 0 ;;
    -*)        die "unknown option: $1

deploy.sh takes no options - see --help. The robot is the one argument; the
port and key go in ~/.ssh/config; anything else is provision.sh's." ;;
esac
[ $# -le 1 ] || die "unexpected extra argument: $2 (usage: $(basename "$0") <user@host>)"
parse_target "${1:-}" "./$(basename "$0") jetson@192.168.1.42"

validate_common
setup_ssh

for f in "$SRC_UNIT" "$SRC_FOXGLOVE_UNIT" "$SRC_ENV" "$SRC_WRAPPER" "$SRC_FOXGLOVE_WRAPPER"; do
    [ -r "$f" ] || die "missing source file: $f"
done

INSTALL_BASE="$WORKSPACE/install"

# ---------------------------------------------------------------------------
# build, on this machine
# ---------------------------------------------------------------------------
# Resolved relative to this script, which is right in a source checkout and wrong
# in an installed one - a copy under share/ has no src/ above it.
[ -d "$WORKSPACE/src" ] || die \
"no src/ in $WORKSPACE, so that is not a colcon workspace.
Run this from the source checkout."

note "building ${DEV_PACKAGES[*]}"
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
    colcon build --packages-select "${DEV_PACKAGES[@]}"
) || die "the build failed; nothing was shipped"

# ---------------------------------------------------------------------------
# find the robot, and where it keeps the workspace
# ---------------------------------------------------------------------------
require_reachable

# Asked of the robot rather than passed in. The prefix has to match the one the
# tree was built for - colcon bakes absolute paths into the environment hooks -
# and the robot already knows it, because provision.sh wrote it into the
# service's own configuration. A flag here could only ever disagree with that.
discovered="$(ssh_query ". /etc/default/$SERVICE_NAME 2>/dev/null && printf %s \"\$WORKSPACE\"" | tr -d '\r\n' || true)"
if [ -n "$discovered" ]; then
    PREFIX="$discovered"
fi
case "$PREFIX" in
    /*) ;;
    *)  die "the robot reported a workspace path that is not absolute: '$PREFIX'" ;;
esac

ssh_query "test -r '$PREFIX/install/setup.bash'" || die \
"$ROBOT_IP has no workspace at $PREFIX/install.

provision.sh is what creates it - the dependencies, the compiled lidar driver,
the robot description and the udev rules. Run that once first:

    ./provision.sh $SSH_USER@$ROBOT_IP"

# ---------------------------------------------------------------------------
# stage
# ---------------------------------------------------------------------------
stage="$(mktemp -d)"
trap 'rm -rf "$stage"' EXIT

stage_service_files "$stage"
mkdir -p "$stage/packages"

compiled=""
dangling=""
for pkg in "${DEV_PACKAGES[@]}"; do
    [ -d "$INSTALL_BASE/$pkg" ] || die "the build produced no $INSTALL_BASE/$pkg"

    mkdir -p "$stage/packages/$pkg"
    # -L dereferences symlinks, and that is load-bearing rather than tidy.
    # `colcon build --symlink-install` - a normal thing to do while developing -
    # makes the install tree point at build/ and src/ instead of holding copies,
    # so local_setup.bash and package.xml become links into this machine's home
    # directory. Shipped as links they arrive dangling, and the robot then
    # reports "not found" for the setup script and drops the package off
    # AMENT_PREFIX_PATH - a package that is present, complete, and invisible.
    # Copying what they point at makes the payload self-contained however the
    # workspace happened to be built.
    rsync -aL --exclude='__pycache__' "$INSTALL_BASE/$pkg/" "$stage/packages/$pkg/"

    # The invariant this script rests on. A compiled object here means the
    # package grew C++ since these defaults were written, and a build from this
    # machine is the wrong architecture for the robot.
    while IFS= read -r f; do
        case "$(file -b "$f" 2>/dev/null)" in
            ELF*) compiled="$compiled  $pkg: ${f#"$stage/packages/$pkg/"}"$'\n' ;;
        esac
    done < <(find "$stage/packages/$pkg" -type f)

    # Whatever -L could not resolve would arrive broken. Catching it here names
    # the file; catching it on the robot means reading a launch failure that
    # mentions neither symlinks nor this machine.
    while IFS= read -r f; do
        dangling="$dangling  $pkg: ${f#"$stage/packages/$pkg/"}"$'\n'
    done < <(find "$stage/packages/$pkg" -xtype l)
done

if [ -n "$dangling" ]; then
    die "these staged files are symlinks that point nowhere:

$dangling
They came from an install tree built with --symlink-install and could not be
resolved. Rebuild the workspace without it:
    cd $WORKSPACE && rm -rf build install && colcon build"
fi

if [ -n "$compiled" ]; then
    die "these files are compiled binaries, built here for $(dpkg --print-architecture 2>/dev/null || uname -m):

$compiled
deploy.sh copies a build from this machine, which is only safe while the packages
it handles contain no compiled code. Whichever one grew it has to move to being
built on the robot, the way the lidar driver already is - see provision.sh."
fi

note "copying $(du -sh "$stage" | cut -f1) to $SSH_USER@$ROBOT_IP:$PREFIX"
remote_tmp="$(ship_stage "$stage")"

# ---------------------------------------------------------------------------
# install
# ---------------------------------------------------------------------------
run_remote "$(cat <<REMOTE
set -e
TMP='$remote_tmp'

# Stopped, not just restarted at the end. Replacing files under a running stack
# leaves it holding deleted inodes: it keeps working until something reloads, and
# then fails in a way that has nothing to do with the deploy that caused it.
#
# Stopped unconditionally, not only when is-active says so. A unit that is
# crash-looping moves between active, activating (auto-restart) and failed, so
# what is-active reports depends on when it is asked - a guarded stop skips the
# unit roughly at random, leaves the pending restart armed, and systemd starts
# the stack again part-way through the copy below. It then fails on half-written
# package directories, which reads as a corrupt deploy rather than as the race
# it is.
#
# systemctl stop is idempotent, cancels the pending restart, and succeeds on an
# already-stopped unit; the 'or true' covers the unit not existing yet on a
# first deploy.
#
# No backticks anywhere in this heredoc: it is unquoted, so they would be
# command substitution, and prose would run on the deploying machine.
for svc in '$SERVICE_NAME' '$SERVICE_NAME-foxglove'; do
    sudo systemctl stop "\$svc" 2>/dev/null || true
done

# The python version is part of the installed path for an ament_python package,
# so a robot on a different Python would silently receive modules nothing
# imports. Checked against the robot's ROS installation rather than against a
# previously deployed copy, so that it works on the very first deploy too - when
# these packages are not on the robot yet.
robot_python="\$(ls -d /opt/ros/$ROS_DISTRO_NAME/lib/python3.* 2>/dev/null | head -1)"
robot_python="\${robot_python##*/}"

for pkg in ${DEV_PACKAGES[*]}; do
    for d in "\$TMP/packages/\$pkg"/lib/python3.*; do
        [ -d "\$d" ] || continue
        v="\$(basename "\$d")"
        if [ -n "\$robot_python" ] && [ "\$v" != "\$robot_python" ]; then
            echo "\$pkg was built for \$v but this robot's ROS uses \$robot_python" >&2
            exit 1
        fi
    done

    # Created on a first deploy: provision.sh prepares the prefix but
    # deliberately does not ship these packages, so this is where they arrive.
    sudo mkdir -p "$PREFIX/install/\$pkg"

    if command -v rsync >/dev/null; then
        # --delete so a file removed from a package is removed on the robot too.
        sudo rsync -a --delete "\$TMP/packages/\$pkg/" "$PREFIX/install/\$pkg/"
    else
        # No rsync on this robot: merge instead. A file deleted from a package
        # lingers until the next provision.sh, which replaces the prefix
        # wholesale. Harmless for the interpreted files this script ships, and
        # reported rather than hidden.
        echo "  (no rsync on the robot; merging without delete)"
        sudo cp -a "\$TMP/packages/\$pkg/." "$PREFIX/install/\$pkg/"
    fi
    sudo chown -R root:root "$PREFIX/install/\$pkg"
    sudo chmod -R go-w "$PREFIX/install/\$pkg"
    echo "  updated \$pkg"
done

$(remote_service_install)

rm -rf "\$TMP"

sudo systemctl daemon-reload
# enable as well as restart: this script installs the units, so start-at-boot is
# its business too. Idempotent, and on a first deploy it is the only thing that
# arms them for the next reboot.
restart_epoch=\$(date +%s)
for svc in '$SERVICE_NAME' '$SERVICE_NAME-foxglove'; do
    sudo systemctl enable "\$svc" 2>&1 | grep -v '^Created symlink' || true
    # A unit that crash-looped past StartLimitBurst is left in a state where
    # systemd refuses to start it at all - "Start request repeated too quickly" -
    # and every later restart is a no-op until the counter is cleared. Without
    # this, a deploy that fixes the very fault which caused the looping still
    # leaves the robot down, and says so without saying why.
    sudo systemctl reset-failed "\$svc" 2>/dev/null || true
    sudo systemctl restart "\$svc"
done

# systemctl restart returns as soon as the process has been execed, so asking
# straight away reports "active" for a service that is about to die - which is
# exactly what a bad deploy looks like for its first second. Waiting and then
# looking is the difference between reporting what happened and reporting what
# was attempted.
sleep 5
failed=""
for svc in '$SERVICE_NAME' '$SERVICE_NAME-foxglove'; do
    systemctl is-active --quiet "\$svc" || failed="\$failed \$svc"
done
if [ -n "\$failed" ]; then
    echo >&2
    for svc in \$failed; do
        echo "=== \$svc did not stay up ===" >&2
        # Since this restart only. A plain tail shows whatever the unit was
        # doing before the deploy, which is usually the older failure the deploy
        # was meant to fix - and reads as if nothing changed.
        journalctl -u "\$svc" --since "@\$restart_epoch" --no-pager -o cat >&2 | tail -20
    done
    exit 1
fi
echo "both services are up"
REMOTE
)"

# The remote half has already waited and failed the deploy if either service did
# not stay up, so reaching here means both are running. This is a summary, not
# the check.
note "status"
ssh_query "systemctl is-active '$SERVICE_NAME' '$SERVICE_NAME-foxglove' || true" 2>/dev/null \
    | sed 's/^/    /' || true

echo
echo "  follow the log   ssh $SSH_USER@$ROBOT_IP journalctl -u $SERVICE_NAME -f"
