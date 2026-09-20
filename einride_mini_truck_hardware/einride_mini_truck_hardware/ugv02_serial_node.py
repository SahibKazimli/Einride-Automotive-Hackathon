import gc
import queue
import threading
import time
from typing import Any

from builtin_interfaces.msg import Time as TimeMsg
from einride_mini_truck_hardware.serial_link import (
    decode_feedback,
    distance_to_angle,
    encode_velocity,
    Feedback,
    LineReader,
    ProtocolError,
)
from geometry_msgs.msg import Twist
import numpy
from numpy.typing import NDArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.timer import Timer
from sensor_msgs.msg import Imu, JointState, MagneticField
import serial
from std_msgs.msg import Float32


#: 8N1 puts 10 bits on the wire per byte: 1 start, 8 data, 1 stop.
BITS_PER_BYTE = 10


class SerialTransport:
    """Owns the ``serial.Serial`` object and survives the port disappearing.

    The current port is held in one attribute that is swapped atomically, so
    the reader thread can block in ``read()`` without holding a lock that would
    stall the writer thread.

    A *closed* port is reported by returning, not raising, so neither thread
    needs to guard against a reconnect happening underneath it. A genuine I/O
    error still propagates, and the reader thread turns that into a reconnect.
    """

    def __init__(self, port: str, baud: int,
                 read_timeout: float, write_timeout: float) -> None:
        self.port = port
        self.baud = baud
        self.read_timeout = read_timeout
        self.write_timeout = write_timeout
        self._ser: serial.Serial | None = None
        self._write_lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._ser is not None

    def open_port(self) -> None:
        """Open the port. Raises ``serial.SerialException`` on failure."""
        self.close_port()
        ser = serial.Serial(
            self.port,
            self.baud,
            timeout=self.read_timeout,
            write_timeout=self.write_timeout,
        )
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        self._ser = ser

    def close_port(self) -> None:
        """Close the port if it is open. Never raises."""
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001 - closing a dead port must not raise
                pass

    def read(self, size: int) -> bytes:
        """Block for up to ``read_timeout`` and return whatever arrived.

        Returns ``b''`` on timeout or when the port is closed.
        """
        ser = self._ser
        if ser is None:
            return b''
        # in_waiting lets a burst come back in one call; the 1 keeps the read
        # blocking (and so paced by the link) when the FIFO is empty.
        return ser.read(max(1, min(size, ser.in_waiting)))

    def write(self, data: bytes) -> bool:
        """Write bytes.

        Returns True on success and False if the port is closed. Raises
        ``serial.SerialException`` if the write itself fails, which the caller
        treats as a reason to reconnect.
        """
        with self._write_lock:
            ser = self._ser
            if ser is None:
                return False
            ser.write(data)
            return True


class Ugv02SerialNode(Node):
    """Bridges the UGV02 chassis MCU to the simulation's topic contract."""

    def __init__(self, **kwargs: Any) -> None:
        # kwargs reach rclpy.node.Node, so tests can pass parameter_overrides
        # and point the node at a pty instead of a real port.
        super().__init__('ugv02_serial_node', **kwargs)

        self._declare_parameters()
        p = self.get_parameter

        self._wheel_radius = p('wheel_radius').value
        self._encoder_joint_names = list(p('encoder_joint_names').value)
        self._imu_frame = p('imu_frame').value
        self._encoder_frame = p('encoder_frame').value
        self._stamp_offset = p('stamp_offset').value
        self._compensate_transmission = p('compensate_transmission_time').value
        self._baud = p('baud').value
        self._cmd_vel_timeout = p('cmd_vel_timeout').value

        feedback_rate = p('feedback_publish_rate').value
        self._feedback_min_period = 1.0 / feedback_rate if feedback_rate > 0.0 else 0.0
        voltage_rate = p('voltage_publish_rate').value
        self._voltage_min_period = 1.0 / voltage_rate if voltage_rate > 0.0 else 0.0

        if len(self._encoder_joint_names) != 2:
            raise ValueError(
                'encoder_joint_names must name exactly two joints (left, right), got {!r}'
                .format(self._encoder_joint_names))

        self._gyro_covariance = self._diagonal_covariance(
            p('imu_angular_velocity_stddev').value)
        self._accel_covariance = self._diagonal_covariance(
            p('imu_linear_acceleration_stddev').value)
        self._mag_covariance = self._diagonal_covariance(
            p('magnetic_field_stddev').value)
        # REP-145: a leading -1 means "this field carries no estimate". The MCU
        # sends raw gyro and accelerometer counts and no fused attitude, so
        # unlike simulation there is no orientation to report. Consumers must
        # check this rather than read the identity quaternion as level.
        self._no_orientation_covariance = numpy.zeros(9, dtype=numpy.float64)
        self._no_orientation_covariance[0] = -1.0

        self._imu_pub = self.create_publisher(Imu, 'imu', qos_profile_sensor_data)
        self._mag_pub = self.create_publisher(MagneticField, 'mag', qos_profile_sensor_data)
        self._encoder_pub = self.create_publisher(
            JointState, 'wheel_encoders', qos_profile_sensor_data)
        self._voltage_pub = self.create_publisher(Float32, 'voltage', qos_profile_default)

        self._cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd_vel, qos_profile_default)

        # Receive path. The reader thread fills the queue and triggers the guard
        # condition; the executor thread runs _drain_feedback and does all the
        # ROS work. Bounded so a stalled executor drops old samples instead of
        # growing without limit.
        self._rx_queue: queue.Queue[tuple[int, bytes]] = queue.Queue(
            maxsize=p('rx_queue_depth').value)
        self._rx_guard = self.create_guard_condition(self._drain_feedback)
        self._rx_dropped = 0

        # Transmit path, one deep: if the link is backed up, the newest twist is
        # the only one worth sending, so an older queued command is discarded
        # rather than delivered late.
        self._tx_queue: queue.Queue[bytes | None] = queue.Queue(maxsize=1)
        self._tx_dropped = 0

        self._transport = SerialTransport(
            p('port').value, self._baud,
            p('read_timeout').value, p('write_timeout').value)
        self._line_reader = LineReader(max_line_bytes=p('max_line_bytes').value)
        self._read_size = p('read_size').value
        self._reconnect_backoff = p('reconnect_backoff').value
        self._reconnect_backoff_max = p('reconnect_backoff_max').value

        # Monotonic-clock state. Never wall clock: an NTP step mid-run would
        # otherwise yield a negative dt and an absurd wheel velocity.
        # (left_m, right_m, monotonic_s), or None until the first frame lands.
        self._last_odom: tuple[float, float, float] | None = None
        self._last_feedback_publish = 0.0
        self._last_voltage_publish = 0.0
        self._last_cmd_monotonic = time.monotonic()
        self._last_cmd_was_zero = True

        self._running = True
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name='ugv02_reader', daemon=True)
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name='ugv02_writer', daemon=True)
        self._reader_thread.start()
        self._writer_thread.start()

        self._watchdog_timer: Timer | None = None
        if self._cmd_vel_timeout > 0.0:
            self._watchdog_timer = self.create_timer(
                min(0.1, self._cmd_vel_timeout / 2.0), self._check_cmd_vel_watchdog)
        else:
            self.get_logger().warning(
                'cmd_vel_timeout is 0: the robot will keep driving on the last '
                'command if the commanding node dies')

        self.get_logger().info(
            'ugv02_serial_node up: port={} baud={} wheel_radius={} encoders={}'.format(
                p('port').value, self._baud, self._wheel_radius,
                self._encoder_joint_names))

    def _declare_parameters(self) -> None:
        """Declare every tunable. Nothing in this node is hardcoded."""
        # The udev symlink from einride_mini_truck_bringup/udev, not the raw
        # ttyACM* the bridge happens to enumerate as. config/hardware.yaml
        # sets this in every launch; the default only matters to a bare
        # `ros2 run`, where failing against the name the project actually
        # uses says more than failing against an arbitrary one.
        self.declare_parameter('port', '/dev/ugv02')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.040)
        self.declare_parameter(
            'encoder_joint_names',
            ['left_up_wheel_link_joint', 'right_up_wheel_link_joint'])
        self.declare_parameter('imu_frame', 'base_imu_link')
        # Simulation publishes /wheel_encoders with an empty frame_id, and a
        # JointState has no meaningful frame anyway. Kept a parameter so parity
        # can be restored if the simulation ever starts setting one.
        self.declare_parameter('encoder_frame', '')

        self.declare_parameter('feedback_publish_rate', 0.0)
        self.declare_parameter('voltage_publish_rate', 1.0)

        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.declare_parameter('stamp_offset', 0.0)
        self.declare_parameter('compensate_transmission_time', True)

        self.declare_parameter('read_timeout', 0.1)
        self.declare_parameter('write_timeout', 0.1)
        self.declare_parameter('read_size', 512)
        self.declare_parameter('max_line_bytes', 4096)
        self.declare_parameter('rx_queue_depth', 200)
        self.declare_parameter('reconnect_backoff', 0.5)
        self.declare_parameter('reconnect_backoff_max', 5.0)

        # Datasheet-derived sigmas.
        self.declare_parameter('imu_angular_velocity_stddev', 2.62e-3)
        self.declare_parameter('imu_linear_acceleration_stddev', 2.26e-2)
        self.declare_parameter('magnetic_field_stddev', 3e-7)

    @staticmethod
    def _diagonal_covariance(stddev: float) -> NDArray[numpy.float64]:
        """Build one constant 3x3 covariance, once, as a float64 array.

        Assigning a 9-element Python list to a covariance field costs an
        element-by-element type check in rclpy every cycle, which is
        disproportionately expensive for a value that never changes. A
        pre-built ``numpy.float64`` array takes the generated setter's fast
        path instead. Sharing one array across every published message is safe
        precisely because it is constant - the messages themselves are still
        built fresh each cycle, so nothing mutable is ever aliased.
        """
        covariance = numpy.zeros(9, dtype=numpy.float64)
        covariance[0] = covariance[4] = covariance[8] = stddev ** 2
        return covariance

    def _reader_loop(self) -> None:
        """Blocking read, frame, stamp, enqueue. Reconnects on port loss."""
        backoff = self._reconnect_backoff
        while self._running:
            if not self._transport.is_open:
                try:
                    self._transport.open_port()
                except (serial.SerialException, OSError) as exc:
                    self.get_logger().warning(
                        'cannot open {}: {}; retrying in {:.1f}s'.format(
                            self._transport.port, exc, backoff),
                        throttle_duration_sec=5.0)
                    # Monotonic-paced backoff: a wall-clock step must not turn
                    # a half-second wait into an hour.
                    deadline = time.monotonic() + backoff
                    while self._running and time.monotonic() < deadline:
                        time.sleep(0.05)
                    backoff = min(backoff * 2.0, self._reconnect_backoff_max)
                    continue
                self._line_reader.reset()
                backoff = self._reconnect_backoff
                self.get_logger().info('opened {}'.format(self._transport.port))

            try:
                chunk = self._transport.read(self._read_size)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(
                    'read failed on {}: {}; reconnecting'.format(
                        self._transport.port, exc))
                self._transport.close_port()
                continue

            if not chunk:
                continue

            # Stamp as close to arrival as the link allows, then let the
            # executor thread do the rest.
            received_ns = self.get_clock().now().nanoseconds
            dropped_before = self._line_reader.dropped_bytes
            for line in self._line_reader.feed(chunk):
                self._enqueue_rx(received_ns, line)
            if self._line_reader.dropped_bytes != dropped_before:
                self.get_logger().warning(
                    'discarded {} bytes with no line terminator'.format(
                        self._line_reader.dropped_bytes - dropped_before),
                    throttle_duration_sec=5.0)

    def _enqueue_rx(self, received_ns: int, line: bytes) -> None:
        try:
            self._rx_queue.put_nowait((received_ns, line))
        except queue.Full:
            # Drop the oldest sample: a stale IMU reading is worth less than a
            # fresh one, and blocking here would stall the reader thread.
            try:
                self._rx_queue.get_nowait()
                self._rx_dropped += 1
            except queue.Empty:
                pass
            try:
                self._rx_queue.put_nowait((received_ns, line))
            except queue.Full:
                self._rx_dropped += 1
        self._rx_guard.trigger()

    def _drain_feedback(self) -> None:
        """Guard-condition callback: publish everything the reader queued.

        Runs on the executor thread, so all ROS work stays single-threaded.
        Guard-condition triggers coalesce, which is why this drains in a loop
        rather than handling one item.
        """
        while True:
            try:
                received_ns, line = self._rx_queue.get_nowait()
            except queue.Empty:
                break

            try:
                feedback = decode_feedback(line)
            except ProtocolError as exc:
                self.get_logger().warning(
                    'dropping malformed line: {}'.format(exc),
                    throttle_duration_sec=2.0)
                continue
            if feedback is None:
                continue  # A valid frame, just not T:1001. Not our business.

            self._publish_feedback(feedback, self._sample_stamp(received_ns, len(line)))

        if self._rx_dropped:
            self.get_logger().warning(
                'dropped {} feedback samples: the executor is not keeping up'.format(
                    self._rx_dropped),
                throttle_duration_sec=5.0)
            self._rx_dropped = 0

    def _sample_stamp(self, received_ns: int, line_len: int) -> TimeMsg:
        """Back-date the receipt time to when the line started arriving.

        The MCU sends no timestamps, so the node stamps at receipt - and at
        115200 8N1 a ~140-byte line spends ~12 ms on the wire before its last
        byte lands, so its oldest field is already that stale. Subtracting the
        line's own transmission time recovers the bulk of it. What is left -
        UART FIFO and scheduler latency, and any bytes that arrived in the same
        read after this line - goes into ``stamp_offset``, which stays 0 until
        it is measured on the robot (HARDWARE_HAL_PLAN.md, phase 5).
        """
        offset_s = self._stamp_offset
        if self._compensate_transmission and self._baud > 0:
            # +2 for the CR/LF that framing stripped.
            offset_s += ((line_len + 2) * BITS_PER_BYTE) / float(self._baud)
        if offset_s == 0.0:
            return Time(nanoseconds=received_ns).to_msg()
        return Time(nanoseconds=max(0, received_ns - int(offset_s * 1e9))).to_msg()

    def _publish_feedback(self, feedback: Feedback, stamp: TimeMsg) -> None:
        now = time.monotonic()

        # Wheel velocity needs a dt, and dt must come from the monotonic clock.
        # Computed before any rate decimation so it always spans the real
        # sampling interval rather than the publishing one.
        velocities = self._wheel_velocities(feedback, now)

        if (self._feedback_min_period
                and now - self._last_feedback_publish < self._feedback_min_period):
            return
        self._last_feedback_publish = now

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self._imu_frame
        imu.linear_acceleration.x, imu.linear_acceleration.y, \
            imu.linear_acceleration.z = feedback.accel
        imu.angular_velocity.x, imu.angular_velocity.y, \
            imu.angular_velocity.z = feedback.gyro
        imu.angular_velocity_covariance = self._gyro_covariance
        imu.linear_acceleration_covariance = self._accel_covariance
        imu.orientation_covariance = self._no_orientation_covariance
        self._imu_pub.publish(imu)

        mag = MagneticField()
        mag.header.stamp = stamp
        mag.header.frame_id = self._imu_frame
        mag.magnetic_field.x, mag.magnetic_field.y, \
            mag.magnetic_field.z = feedback.mag
        mag.magnetic_field_covariance = self._mag_covariance
        self._mag_pub.publish(mag)

        encoders = JointState()
        encoders.header.stamp = stamp
        encoders.header.frame_id = self._encoder_frame
        encoders.name = self._encoder_joint_names
        # The wire carries metres accumulated per side; the topic carries
        # radians per joint, exactly as simulation publishes it.
        encoders.position = [
            distance_to_angle(feedback.odom_left, self._wheel_radius),
            distance_to_angle(feedback.odom_right, self._wheel_radius),
        ]
        encoders.velocity = list(velocities)
        # effort stays empty: the chassis has no torque sensing. Simulation
        # fills zeros there, which are not measurements either.
        self._encoder_pub.publish(encoders)

        # /voltage does not need the feedback rate. Throttling it keeps a
        # publish out of the hot path 79 cycles in 80.
        if (not self._voltage_min_period
                or now - self._last_voltage_publish >= self._voltage_min_period):
            self._last_voltage_publish = now
            self._voltage_pub.publish(Float32(data=feedback.voltage))

    def _wheel_velocities(
            self, feedback: Feedback, now: float) -> tuple[float, float]:
        """Differentiate accumulated distance into rad/s, on the monotonic clock."""
        previous, self._last_odom = self._last_odom, (
            feedback.odom_left, feedback.odom_right, now)
        if previous is None:
            return (0.0, 0.0)
        left0, right0, then = previous
        dt = now - then
        if dt <= 0.0:
            return (0.0, 0.0)
        return (
            distance_to_angle(feedback.odom_left - left0, self._wheel_radius) / dt,
            distance_to_angle(feedback.odom_right - right0, self._wheel_radius) / dt,
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Pass the twist through to the MCU, unmodified.

        Deliberately no deadband. The reference driver clamped stationary
        |angular.z| up to 0.2 rad/s, which turns a 0.05 rad/s request into 0.2
        and so makes the command non-linear exactly where a yaw controller does
        its small corrections. Simulation has no such clamp either, so gains
        tuned there would misbehave here. Whatever the platform's real pivot
        threshold turns out to be, it belongs in the controller - nav2's
        min_speed_theta - where it is visible, not rewritten silently in a
        driver.
        """
        try:
            data = encode_velocity(msg.linear.x, msg.angular.z)
        except ValueError as exc:
            self.get_logger().warning('ignoring cmd_vel: {}'.format(exc))
            return
        self._last_cmd_monotonic = time.monotonic()
        self._last_cmd_was_zero = msg.linear.x == 0.0 and msg.angular.z == 0.0
        self._enqueue_tx(data)

    def _enqueue_tx(self, data: bytes) -> None:
        try:
            self._tx_queue.put_nowait(data)
            return
        except queue.Full:
            pass
        try:
            self._tx_queue.get_nowait()  # the queued twist is already stale
            self._tx_dropped += 1
        except queue.Empty:
            pass
        try:
            self._tx_queue.put_nowait(data)
        except queue.Full:
            self._tx_dropped += 1

    def _writer_loop(self) -> None:
        """Drain the transmit queue. Blocking, so it never spins."""
        while self._running:
            try:
                data = self._tx_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if data is None:
                break
            try:
                if not self._transport.write(data):
                    self.get_logger().warning(
                        'dropping command, port is down', throttle_duration_sec=5.0)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(
                    'write failed: {}; reconnecting'.format(exc))
                self._transport.close_port()

    def _check_cmd_vel_watchdog(self) -> None:
        """Stop the robot when commands stop arriving.

        The MCU holds the last commanded velocity indefinitely, so a crashed
        teleop or a lost network link leaves the robot driving. Monotonic, so a
        clock correction cannot trip it or suppress it.
        """
        if self._last_cmd_was_zero:
            return
        if time.monotonic() - self._last_cmd_monotonic < self._cmd_vel_timeout:
            return
        self.get_logger().warning(
            'no cmd_vel for {:.1f}s, stopping'.format(self._cmd_vel_timeout))
        self._last_cmd_was_zero = True
        self._enqueue_tx(encode_velocity(0.0, 0.0))

    def shutdown(self) -> None:
        """Stop the threads and leave the robot stationary."""
        if not self._running:
            return
        # Best effort, straight down the port rather than through the queue:
        # the writer thread is about to be told to stop.
        try:
            self._transport.write(encode_velocity(0.0, 0.0))
        except Exception:  # noqa: BLE001 - shutdown must not raise
            pass
        self._running = False
        # A nudge, not a handshake: the queue is one deep, so if it happens to
        # be full the sentinel is simply dropped and the writer exits on its own
        # read timeout instead. Blocking here would deadlock shutdown against a
        # writer that has already seen _running go false.
        try:
            self._tx_queue.put_nowait(None)
        except queue.Full:
            pass
        for thread in (self._reader_thread, self._writer_thread):
            thread.join(timeout=2.0)
        self._transport.close_port()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: Ugv02SerialNode | None = None
    try:
        node = Ugv02SerialNode()
        # Everything allocated during construction is permanent, so move it out
        # of the generational collector's reach. CPython's cyclic collector can
        # pause for milliseconds, which matters against a 12.5 ms cycle budget
        # far more than raw throughput does.
        gc.freeze()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node stays None if construction itself failed - e.g. a bad
        # encoder_joint_names - and rclpy still has to be shut down.
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['main', 'SerialTransport', 'Ugv02SerialNode']


if __name__ == '__main__':
    main()
