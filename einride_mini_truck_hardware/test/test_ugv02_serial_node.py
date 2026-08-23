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

"""Node-level tests against a pty standing in for the chassis MCU.

A pty is a real character device with real termios, so ``pyserial`` opens it
exactly as it opens ``/dev/ttyAMA0`` and the node is exercised unmodified: its
reader thread, writer thread, guard condition and executor all run for real.
The test writes the recorded capture into the master end and reads back the
``T:13`` commands the node sends. No hardware, and no mocking of the node.
"""

from collections.abc import Callable, Iterable, Iterator
import functools
import json
import os
import threading
import time
from typing import Any

from einride_mini_truck_hardware.serial_link import LineReader
from einride_mini_truck_hardware.ugv02_serial_node import Ugv02SerialNode
from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_default,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, JointState, MagneticField
from std_msgs.msg import Float32

CAPTURE = os.path.join(os.path.dirname(__file__), 'data', 'ugv02_feedback.jsonl')
SPIN_TIMEOUT = 5.0


class Harness:
    """A running node, a pty pretending to be the MCU, and a listener."""

    def __init__(self, **overrides: Any) -> None:
        self.master_fd, slave_fd = os.openpty()
        port = os.ttyname(slave_fd)
        os.close(slave_fd)      # pyserial opens the slave itself, by name

        parameters = [Parameter('port', value=port)]
        parameters += [Parameter(k, value=v) for k, v in overrides.items()]

        self.node = Ugv02SerialNode(parameter_overrides=parameters)
        self.listener = rclpy.create_node('test_listener')
        # Each subscription uses the profile the node publishes with. Requesting
        # RELIABLE on a sensor topic would receive nothing at all, which is the
        # failure this split makes possible and the tests have to respect.
        self.received: dict[str, list[Any]] = {
            'imu': [], 'mag': [], 'wheel_encoders': [], 'voltage': []}
        for topic, msg_type, qos in (('imu', Imu, qos_profile_sensor_data),
                                     ('mag', MagneticField, qos_profile_sensor_data),
                                     ('wheel_encoders', JointState, qos_profile_sensor_data),
                                     ('voltage', Float32, qos_profile_default)):
            self.listener.create_subscription(
                msg_type, topic, self._collector(topic), qos)
        self.cmd_vel = self.listener.create_publisher(
            Twist, 'cmd_vel', qos_profile_default)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.listener)
        self.reader = LineReader()
        self.commands: list[dict[str, Any]] = []

        # Drain the master end continuously rather than on demand: closing the
        # slave makes any further read of the master fail with EIO and discards
        # what was buffered, which would lose the stop command that shutdown
        # sends. A thread reading as the node writes cannot miss it.
        self._draining = True
        self._drain_thread = threading.Thread(
            target=self._drain_master, name='pty_drain', daemon=True)
        self._drain_thread.start()

        self.spin_until(lambda: self.node._transport.is_open)

    def _collector(self, topic: str) -> Callable[[Any], None]:
        """Build the subscription callback for one topic.

        A closure rather than a lambda with a default argument: the default is
        what binds the loop variable, and it is also what stops a type checker
        inferring the callback's signature.
        """
        def collect(msg: Any) -> None:
            self.received[topic].append(msg)
        return collect

    def _drain_master(self) -> None:
        while self._draining:
            try:
                chunk = os.read(self.master_fd, 4096)
            except OSError:
                return          # slave closed; nothing more will be written
            if not chunk:
                return
            for line in self.reader.feed(chunk):
                self.commands.append(json.loads(line))

    def send_lines(self, lines: Iterable[bytes]) -> None:
        """Push raw bytes into the MCU end of the link."""
        os.write(self.master_fd, b''.join(line + b'\n' for line in lines))

    def replay(self, lines: list[bytes], burst: int = 4) -> bool:
        """Feed a capture in small bursts, letting the listener keep up.

        The sensor publishers are best-effort KEEP_LAST(5), so a subscriber
        that is not spinning legitimately loses samples once more than five are
        in flight - and being best-effort, loses them for good. Bursting is
        about the test observing every frame, not about the node being unable to
        handle a flood; see the rx_queue_depth parameter for that.
        """
        for i in range(0, len(lines), burst):
            chunk = lines[i:i + burst]
            target = len(self.received['imu']) + len(chunk)
            self.send_lines(chunk)
            if not self.spin_until(
                    lambda: len(self.received['imu']) >= target, timeout=2.0):
                return False
        return self.spin_until_frames(len(lines))

    def spin_until_frames(self, count: int,
                          timeout: float = SPIN_TIMEOUT) -> bool:
        """Spin until ``count`` frames have landed on every per-frame topic.

        imu, mag and wheel_encoders are published in one drain, but their
        subscriber callbacks are separate executor work, so waiting on only one
        of them races the other two. /voltage is throttled and excluded.
        """
        per_frame = ('imu', 'mag', 'wheel_encoders')
        return self.spin_until(
            lambda: all(len(self.received[k]) >= count for k in per_frame), timeout)

    def spin_until(self, predicate: Callable[[], Any],
                   timeout: float = SPIN_TIMEOUT) -> bool:
        """Spin the executor until the predicate holds. Returns whether it did."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            self.executor.spin_once(timeout_sec=0.02)
        return predicate()

    def read_commands(self, timeout: float = 1.0) -> list[dict[str, Any]]:
        """Spin for a while and return the T:13 commands seen so far.

        The drain thread does the reading; this only gives the writer thread
        time to get the bytes onto the port.
        """
        self.spin_until(lambda: False, timeout=timeout)
        return self.commands

    def close(self) -> None:
        self._draining = False
        self.node.shutdown()
        self.executor.remove_node(self.node)
        self.executor.remove_node(self.listener)
        self.node.destroy_node()
        self.listener.destroy_node()
        self._drain_thread.join(timeout=1.0)
        os.close(self.master_fd)


@pytest.fixture(scope='module', autouse=True)
def ros() -> Iterator[None]:
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def harness() -> Iterator[Callable[..., Harness]]:
    made: list[Harness] = []

    def build(**overrides: Any) -> Harness:
        made.append(Harness(**overrides))
        return made[-1]

    yield build
    for item in made:
        item.close()


def capture_lines(count: int | None = None) -> list[bytes]:
    with open(CAPTURE, 'rb') as handle:
        lines = handle.read().splitlines()
    return lines if count is None else lines[:count]


# ------------------------------------------------------------------ receive

def test_one_frame_publishes_every_topic(harness: Callable[..., Harness]) -> None:
    """A single T:1001 line is enough to populate the whole feedback contract."""
    h = harness()
    h.send_lines(capture_lines(1))
    assert h.spin_until(lambda: all(h.received[k] for k in h.received)), h.received


def test_imu_matches_the_wire_values(harness: Callable[..., Harness]) -> None:
    h = harness()
    h.send_lines([json.dumps({
        'T': 1001, 'L': 0, 'R': 0,
        'ax': 0, 'ay': 0, 'az': 8192, 'gx': 0, 'gy': 0, 'gz': 940,
        'mx': 100, 'my': 0, 'mz': 0, 'odl': 0.0, 'odr': 0.0, 'v': 11.7,
    }).encode()])
    assert h.spin_until_frames(1)

    imu = h.received['imu'][0]
    assert imu.header.frame_id == 'base_imu_link'
    assert imu.linear_acceleration.z == pytest.approx(9.80665)
    assert imu.angular_velocity.z == pytest.approx(1.0, rel=1e-3)
    # Datasheet-derived sigma, matching the noise model.sdf injects.
    assert imu.angular_velocity_covariance[0] == pytest.approx(2.62e-3 ** 2)
    assert imu.linear_acceleration_covariance[0] == pytest.approx(2.26e-2 ** 2)
    # REP-145: the MCU sends no fused attitude, so orientation carries no
    # estimate. Simulation does provide one - a documented divergence.
    assert imu.orientation_covariance[0] == -1.0

    mag = h.received['mag'][0]
    assert mag.header.frame_id == 'base_imu_link'
    assert mag.magnetic_field.x == pytest.approx(100 * 0.15e-6)
    assert mag.magnetic_field_covariance[0] == pytest.approx(3e-7 ** 2)


def test_encoders_are_published_in_radians(harness: Callable[..., Harness]) -> None:
    """The wire carries metres per side; the topic carries radians, as in sim."""
    h = harness(wheel_radius=0.04)
    h.send_lines([json.dumps({
        'T': 1001, 'L': 0, 'R': 0,
        'ax': 0, 'ay': 0, 'az': 8192, 'gx': 0, 'gy': 0, 'gz': 0,
        'mx': 0, 'my': 0, 'mz': 0, 'odl': 0.4, 'odr': 0.8, 'v': 11.7,
    }).encode()])
    assert h.spin_until_frames(1)

    js = h.received['wheel_encoders'][0]
    assert list(js.name) == ['left_up_wheel_link_joint', 'right_up_wheel_link_joint']
    assert js.position == pytest.approx([10.0, 20.0])
    assert js.header.frame_id == ''       # as simulation publishes it
    assert len(js.velocity) == 2


def test_wheel_velocity_is_differentiated(harness: Callable[..., Harness]) -> None:
    """Velocity comes from consecutive samples, so the first one must be zero."""
    h = harness()
    frame = {'T': 1001, 'L': 0, 'R': 0, 'ax': 0, 'ay': 0, 'az': 8192,
             'gx': 0, 'gy': 0, 'gz': 0, 'mx': 0, 'my': 0, 'mz': 0, 'v': 11.7}
    h.send_lines([json.dumps(dict(frame, odl=0.0, odr=0.0)).encode()])
    assert h.spin_until_frames(1)
    assert list(h.received['wheel_encoders'][0].velocity) == [0.0, 0.0]

    time.sleep(0.05)
    h.send_lines([json.dumps(dict(frame, odl=0.1, odr=0.1)).encode()])
    assert h.spin_until_frames(2)
    assert h.received['wheel_encoders'][1].velocity[0] > 0.0


def test_stamps_are_back_dated_by_the_transmission_time(harness: Callable[..., Harness]) -> None:
    """A ~134-byte line spends ~12 ms on the wire before its last byte lands."""
    h = harness()
    h.send_lines(capture_lines(1))
    assert h.spin_until_frames(1)
    stamp = h.received['imu'][0].header.stamp
    stamped_ns = stamp.sec * 10 ** 9 + stamp.nanosec
    now_ns = h.node.get_clock().now().nanoseconds

    line_ms = (len(capture_lines(1)[0]) + 2) * 10 / 115200.0 * 1e3
    assert 0.9 * line_ms < (now_ns - stamped_ns) / 1e6
    assert stamped_ns < now_ns


def test_stamps_are_receipt_time_when_compensation_is_off(harness: Callable[..., Harness]) -> None:
    h = harness(compensate_transmission_time=False)
    h.send_lines(capture_lines(1))
    assert h.spin_until_frames(1)
    stamp = h.received['imu'][0].header.stamp
    stamped_ns = stamp.sec * 10 ** 9 + stamp.nanosec
    assert (h.node.get_clock().now().nanoseconds - stamped_ns) / 1e6 < 5.0


def test_voltage_is_throttled(harness: Callable[..., Harness]) -> None:
    """/voltage does not need the feedback rate; the other topics do."""
    h = harness(voltage_publish_rate=1.0)
    assert h.replay(capture_lines(80))
    h.spin_until(lambda: False, timeout=0.2)      # let anything late arrive
    # 80 frames replayed in well under a second, so exactly one 1 Hz tick.
    assert len(h.received['voltage']) == 1
    assert len(h.received['imu']) == 80


def test_the_whole_capture_replays(harness: Callable[..., Harness]) -> None:
    """Every frame in the capture reaches every topic, in order."""
    lines = capture_lines()
    h = harness()
    assert h.replay(lines)
    assert len(h.received['imu']) == len(lines)

    # The capture drives forward for its first two seconds, then pivots left,
    # so the left encoder rises monotonically and then reverses.
    positions = [js.position[0] for js in h.received['wheel_encoders']]
    forward = positions[:160]
    assert forward == sorted(forward)
    assert positions[-1] < positions[159]
    assert h.received['voltage'][0].data == pytest.approx(11.8, abs=0.01)


def test_junk_between_good_lines_is_survived(harness: Callable[..., Harness]) -> None:
    """A mid-line reconnect or line noise must cost one frame, not the stream."""
    good = capture_lines(4)
    h = harness()
    h.send_lines([good[0], b'{"T":1001,truncated', b'\x80\x81 noise', good[1]])
    assert h.spin_until_frames(2)


def test_other_frame_types_are_ignored(harness: Callable[..., Harness]) -> None:
    """The MCU emits command acknowledgements; they are not errors."""
    h = harness()
    h.send_lines([b'{"T":1002,"status":"ok"}', capture_lines(1)[0]])
    assert h.spin_until_frames(1)
    h.spin_until(lambda: False, timeout=0.2)
    assert len(h.received['imu']) == 1


# ----------------------------------------------------------------- transmit

def test_cmd_vel_becomes_a_T13_command(harness: Callable[..., Harness]) -> None:
    h = harness()
    msg = Twist()
    msg.linear.x = 0.25
    msg.angular.z = -0.5
    h.cmd_vel.publish(msg)
    assert h.spin_until(lambda: h.commands)
    assert h.commands[0] == {'T': 13, 'X': 0.25, 'Z': -0.5}


def test_cmd_vel_is_passed_through_unmodified(harness: Callable[..., Harness]) -> None:
    """No deadband: the reference driver would have rewritten this 0.05 to 0.2.

    That clamp is what breaks a yaw controller making small stationary
    corrections, and simulation has no equivalent, so the command has to reach
    the MCU exactly as it was published.
    """
    h = harness()
    msg = Twist()
    msg.angular.z = 0.05
    h.cmd_vel.publish(msg)
    assert h.spin_until(lambda: h.commands)
    assert h.commands[0] == {'T': 13, 'X': 0.0, 'Z': 0.05}


def test_watchdog_stops_the_robot(harness: Callable[..., Harness]) -> None:
    """The MCU holds the last velocity forever, so silence must mean stop."""
    h = harness(cmd_vel_timeout=0.2)
    msg = Twist()
    msg.linear.x = 0.3
    h.cmd_vel.publish(msg)
    assert h.spin_until(lambda: len(h.commands) >= 2, timeout=3.0)
    assert h.commands[0] == {'T': 13, 'X': 0.3, 'Z': 0.0}
    assert h.commands[1] == {'T': 13, 'X': 0.0, 'Z': 0.0}


def test_watchdog_does_not_repeat_once_stopped(harness: Callable[..., Harness]) -> None:
    h = harness(cmd_vel_timeout=0.2)
    msg = Twist()
    msg.linear.x = 0.3
    h.cmd_vel.publish(msg)
    assert h.spin_until(lambda: len(h.commands) >= 2, timeout=3.0)
    h.read_commands(timeout=0.8)
    assert len(h.commands) == 2


def test_non_finite_cmd_vel_is_dropped(harness: Callable[..., Harness]) -> None:
    """A non-finite twist would desynchronise the MCU's JSON parser."""
    h = harness()
    bad = Twist()
    bad.angular.z = float('nan')
    h.cmd_vel.publish(bad)
    h.read_commands(timeout=0.3)
    assert h.commands == []

    good = Twist()
    good.linear.x = 0.1
    h.cmd_vel.publish(good)
    assert h.spin_until(lambda: h.commands)
    assert h.commands[0] == {'T': 13, 'X': 0.1, 'Z': 0.0}


# --------------------------------------------------------------- robustness

def test_missing_port_is_retried_not_fatal(harness: Callable[..., Harness]) -> None:
    """A node that dies when the MCU is unplugged is useless on a robot."""
    h = harness(port='/dev/does-not-exist', reconnect_backoff=0.05)
    h.spin_until(lambda: False, timeout=0.5)
    assert not h.node._transport.is_open
    assert h.node._reader_thread.is_alive()      # still trying


def test_published_qos_is_the_declared_contract(harness: Callable[..., Harness]) -> None:
    """Connecting at all depends on QoS, so a mismatch is a silent failure.

    The sensor streams are best-effort and the rest reliable. Whether simulation
    reproduces that is the conformance test's job, not this one's.
    """
    h = harness()
    expected = {
        h.node._imu_pub: qos_profile_sensor_data,
        h.node._mag_pub: qos_profile_sensor_data,
        h.node._encoder_pub: qos_profile_sensor_data,
        h.node._voltage_pub: qos_profile_default,
    }
    for publisher, profile in expected.items():
        qos = publisher.qos_profile
        assert qos.reliability == profile.reliability, publisher.topic_name
        assert qos.durability == profile.durability, publisher.topic_name
        assert qos.depth == profile.depth, publisher.topic_name

    # And as a peer actually discovers it. Discovery does not carry the history
    # depth, so only reliability and durability can be checked from outside.
    expected_by_topic = {
        'imu': qos_profile_sensor_data,
        'mag': qos_profile_sensor_data,
        'wheel_encoders': qos_profile_sensor_data,
        'voltage': qos_profile_default,
    }
    for topic, profile in expected_by_topic.items():
        assert h.spin_until(
            functools.partial(h.listener.get_publishers_info_by_topic, '/' + topic)), topic
        qos = h.listener.get_publishers_info_by_topic('/' + topic)[0].qos_profile
        assert qos.reliability == profile.reliability, topic
        assert qos.durability == profile.durability, topic


def test_a_reliable_subscriber_gets_nothing_from_a_sensor_topic(
        harness: Callable[..., Harness]) -> None:
    """The documented cost of best-effort, pinned so nobody rediscovers it.

    create_subscription's default depth argument requests RELIABLE, which is
    incompatible with these publishers - the subscriber connects to nothing and
    only a QoS warning is logged. /voltage stays reliable and still works.
    """
    h = harness()
    strict = []
    h.listener.create_subscription(Imu, 'imu', lambda m: strict.append(m), 10)
    lenient = []
    h.listener.create_subscription(Float32, 'voltage', lambda m: lenient.append(m),
                                   qos_profile_default)
    h.send_lines(capture_lines(1))
    assert h.spin_until_frames(1)
    h.spin_until(lambda: False, timeout=0.5)
    assert strict == []
    assert lenient


def test_shutdown_stops_the_robot(harness: Callable[..., Harness]) -> None:
    """Leaving the chassis driving after Ctrl-C is a safety bug, not a nicety."""
    h = harness()
    msg = Twist()
    msg.linear.x = 0.4
    h.cmd_vel.publish(msg)
    assert h.spin_until(lambda: h.commands)
    h.node.shutdown()
    h.read_commands(timeout=0.3)
    assert h.commands[-1] == {'T': 13, 'X': 0.0, 'Z': 0.0}


def test_qos_constants_are_what_they_claim() -> None:
    """Guards the stock rclpy profiles this node relies on, in case they move."""
    assert isinstance(qos_profile_sensor_data, QoSProfile)
    assert qos_profile_sensor_data.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos_profile_sensor_data.durability == DurabilityPolicy.VOLATILE
    assert qos_profile_sensor_data.depth == 5
    assert qos_profile_default.reliability == ReliabilityPolicy.RELIABLE
    assert qos_profile_default.depth == 10
