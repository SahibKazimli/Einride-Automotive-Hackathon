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
# Shared by provision.sh and deploy.sh. Not executable on its own.
#
# The two scripts do very different amounts of work but talk to the robot in
# exactly the same way, and the parts that are easy to get subtly wrong - the
# quoting of the remote script, the validation of what gets interpolated into
# it, the manifest checks - are the parts worth having in one place.

# shellcheck shell=bash

SERVICE_NAME="${SERVICE_NAME:-einride-mini-truck}"
# Both the path the workspace is cross-built for and the path it is installed
# at. The two must agree - colcon bakes absolute paths into the environment
# hooks - so one variable feeds both rather than two that can drift.
PREFIX="${PREFIX:-/opt/einride_mini_truck}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
SSH_PORT="${SSH_PORT:-22}"
IDENTITY="${IDENTITY:-}"
DRY_RUN=0

die() { echo "error: $*" >&2; exit 1; }

# <user@host>, the one argument both scripts take. Shared so they cannot drift
# into disagreeing about what an address looks like.
parse_target() {
    local robot="$1" usage="$2"
    [ -n "$robot" ] || die "which robot? Name it on the command line:

    $usage"
    case "$robot" in
        # Same rule as ssh: no user means the local one. Spelling the robot's
        # account out is the usual thing, since it rarely matches.
        *@*) SSH_USER="${robot%%@*}"; ROBOT_IP="${robot#*@}" ;;
        *)   SSH_USER="${USER:-$(id -un)}"; ROBOT_IP="$robot" ;;
    esac
    [ -n "$ROBOT_IP" ] || die "no robot address in '$robot'"
}
note() { echo "==> $*"; }
warn() { echo "warning: $*" >&2; }

# ---------------------------------------------------------------------------
# validation
# ---------------------------------------------------------------------------
# The remote half of each script is assembled locally and run by a shell on the
# robot, with these values interpolated into it inside single quotes. So they are
# validated rather than trusted: a quote or a semicolon in any of them would stop
# being data and start being part of the command. They also land in the unit
# file, where the same characters produce a file systemd cannot parse.
validate_common() {
    [[ "$SERVICE_NAME" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] \
        || die "service name '$SERVICE_NAME' is not a valid systemd unit name"

    # POSIX-portable account name, plus the trailing $ Samba-joined machine
    # accounts use. Deliberately narrower than what useradd will accept.
    [[ "$SSH_USER" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] \
        || die "ssh user '$SSH_USER' is not a plausible unix account name"

    # A path, so it cannot be reduced to a character class. Rejecting what would
    # break out of the single quotes it is interpolated into is enough, and is
    # honest about the limit rather than silently mangling the path.
    case "$PREFIX" in
        *\'*|*'"'*|*'$'*|*'`'*|*[$'\n\r']*)
            die "prefix may not contain quotes, \$, backticks or newlines" ;;
        /*) ;;
        *)  die "prefix must be absolute: $PREFIX" ;;
    esac
    # rm -rf runs against this path on the robot.
    case "$PREFIX" in
        /|/usr|/etc|/var|/opt|/home|/boot|/bin|/sbin|/lib|/root)
            die "refusing to use $PREFIX as the install prefix" ;;
    esac
}

# ---------------------------------------------------------------------------
# ssh
# ---------------------------------------------------------------------------
setup_ssh() {
    TARGET="${SSH_USER}@${ROBOT_IP}"
    SSH_OPTS=(-o BatchMode=no -o ConnectTimeout=10 -p "$SSH_PORT")
    if [ -n "$IDENTITY" ]; then
        SSH_OPTS+=(-i "$IDENTITY")
    fi
}

# Two ways in, deliberately. Non-interactive for queries whose output is parsed,
# and -t for anything using sudo, so it has a terminal to prompt for a password.
ssh_query() { ssh "${SSH_OPTS[@]}" "$TARGET" "$@"; }
ssh_admin() { ssh -t "${SSH_OPTS[@]}" "$TARGET" "$@"; }

# Runs a remote script, or prints it under --dry-run. Single argument: the whole
# script, already expanded locally. Sent as the command rather than on stdin
# because `ssh -t` will not allocate a terminal when stdin is a pipe, and without
# a terminal sudo cannot prompt. base64 so that no quoting inside the script can
# misparse on the way.
run_remote() {
    if [ "$DRY_RUN" -eq 1 ]; then
        echo "--- would run on $TARGET ---"
        echo "$1"
        echo "--- end ---"
        return 0
    fi
    local encoded
    encoded="$(printf '%s' "$1" | base64 | tr -d '\n')"
    ssh_admin "echo $encoded | base64 -d | bash"
}

require_reachable() {
    note "checking $TARGET is reachable"
    ssh_query true || die "cannot ssh to $TARGET on port $SSH_PORT"
    ssh_query "command -v systemctl >/dev/null" \
        || die "no systemctl on $ROBOT_IP - this deploys a systemd service"
}

# Sets ROBOT_ARCH, ROBOT_OS_ID, ROBOT_OS_VERSION - all of it for reporting now.
# Nothing compiled on this machine reaches the robot, so there is no
# architecture or glibc compatibility left to police: the one package with
# compiled code is built on the robot itself, and the rest are Python and data
# files that do not care.
inspect_target() {
    ROBOT_ARCH="$(ssh_query "dpkg --print-architecture 2>/dev/null || uname -m" | tr -d '\r\n')"
    local os
    os="$(ssh_query '. /etc/os-release 2>/dev/null && printf "%s %s" "$ID" "$VERSION_ID"' | tr -d '\r\n')"
    ROBOT_OS_ID="${os%% *}"
    ROBOT_OS_VERSION="${os##* }"
    [ -n "$ROBOT_ARCH" ] || die "could not determine the architecture of $ROBOT_IP"
    [ -n "$ROBOT_OS_ID" ] && [ -n "$ROBOT_OS_VERSION" ] \
        || die "could not read /etc/os-release on $ROBOT_IP"

    # uname -m spells it aarch64, dpkg and docker spell it arm64. Only the
    # fallback path can produce the kernel spelling, but a "linux/aarch64"
    # platform string is rejected by buildx with an error that does not mention
    # the spelling.
    case "$ROBOT_ARCH" in
        aarch64) ROBOT_ARCH="arm64" ;;
        x86_64)  ROBOT_ARCH="amd64" ;;
        armv7l)  ROBOT_ARCH="armhf" ;;
    esac
}

# ---------------------------------------------------------------------------
# templating
# ---------------------------------------------------------------------------
# & and \ are special on the right-hand side of a sed s///, and | is the
# delimiter used here. A prefix containing any of them would otherwise produce a
# unit file that is quietly wrong rather than one that fails.
sed_escape() { printf '%s' "$1" | sed -e 's/[\\&|]/\\&/g'; }

fill_template() {
    sed -e "s|@RUN_USER@|$(sed_escape "$SSH_USER")|g" \
        -e "s|@WORKSPACE@|$(sed_escape "$PREFIX")|g" \
        -e "s|@SERVICE_NAME@|$(sed_escape "$SERVICE_NAME")|g" \
        "$1" > "$2"
}

# Stages both units, their wrappers and the shared environment file into
# $1/service. Two services: the stack itself, and the Foxglove bridge that lets
# the app connect to it. They share one environment file - see the comment in
# the template for why.
stage_service_files() {
    local dest="$1/service"
    mkdir -p "$dest"
    fill_template "$SRC_UNIT"          "$dest/$SERVICE_NAME.service"
    fill_template "$SRC_FOXGLOVE_UNIT" "$dest/$SERVICE_NAME-foxglove.service"
    fill_template "$SRC_ENV"           "$dest/$SERVICE_NAME.env"
    cp "$SRC_WRAPPER"          "$dest/run-hardware.sh"
    cp "$SRC_FOXGLOVE_WRAPPER" "$dest/run-foxglove.sh"
    chmod 0755 "$dest/run-hardware.sh" "$dest/run-foxglove.sh"

    # A leftover placeholder means a template grew one that fill_template does
    # not know about - caught here rather than as a systemd parse error on the
    # robot.
    if grep -Hn '@[A-Z_]\+@' \
            "$dest/$SERVICE_NAME.service" \
            "$dest/$SERVICE_NAME-foxglove.service" \
            "$dest/$SERVICE_NAME.env" >&2; then
        die "unsubstituted placeholder left in the staged files"
    fi
}

# The remote fragment that installs the staged service files. Shared because
# both scripts install them and they must agree exactly on the paths, the modes
# and the rule that the robot's own configuration is never overwritten.
remote_service_install() {
    cat <<REMOTE
sudo install -d -m 0755 '/usr/local/lib/$SERVICE_NAME'
sudo install -m 0755 "\$TMP/service/run-hardware.sh" '/usr/local/lib/$SERVICE_NAME/run-hardware.sh'
sudo install -m 0755 "\$TMP/service/run-foxglove.sh" '/usr/local/lib/$SERVICE_NAME/run-foxglove.sh'
sudo install -m 0644 "\$TMP/service/$SERVICE_NAME.service" '/etc/systemd/system/$SERVICE_NAME.service'
sudo install -m 0644 "\$TMP/service/$SERVICE_NAME-foxglove.service" '/etc/systemd/system/$SERVICE_NAME-foxglove.service'

# The configuration file is the robot's, not the deployment's. Overwriting it
# would silently discard a domain ID or a camera:=false someone set on this
# machine, so an existing one is kept and the new template is left beside it for
# comparison.
if [ -f '/etc/default/$SERVICE_NAME' ]; then
    if cmp -s "\$TMP/service/$SERVICE_NAME.env" '/etc/default/$SERVICE_NAME'; then
        echo 'config: /etc/default/$SERVICE_NAME unchanged'
    else
        sudo install -m 0644 "\$TMP/service/$SERVICE_NAME.env" '/etc/default/$SERVICE_NAME.new'
        echo 'config: kept your /etc/default/$SERVICE_NAME;'
        echo '        new defaults written to /etc/default/$SERVICE_NAME.new - diff them.'
    fi
else
    sudo install -m 0644 "\$TMP/service/$SERVICE_NAME.env" '/etc/default/$SERVICE_NAME'
    echo 'config: installed /etc/default/$SERVICE_NAME'
fi
REMOTE
}

# Copies a staged directory to a fresh temporary directory on the robot and
# echoes its path. mktemp, not a name built from $$: these files are handed to
# `sudo install` a moment later, and a predictable path under a world-writable
# /tmp is one a local user could have pre-created as a symlink pointing
# somewhere else.
ship_stage() {
    local stage="$1" remote_tmp
    if [ "$DRY_RUN" -eq 1 ]; then
        echo "<mktemp -d>"
        return 0
    fi
    remote_tmp="$(ssh_query "mktemp -d -t '${SERVICE_NAME}-deploy.XXXXXXXX'" | tr -d '\r\n')"
    [ -n "$remote_tmp" ] || die "could not create a staging directory on $ROBOT_IP"
    # tar over the existing ssh transport rather than scp: one connection, and it
    # carries permissions and symlinks, both of which the install tree has.
    tar -C "$stage" -cf - . | ssh_query "tar -C '$remote_tmp' -xf -"
    echo "$remote_tmp"
}
