"""Codec for the UGV02 chassis MCU link.

The wire protocol is newline-delimited JSON in both directions over
``/dev/ttyAMA0`` at 115200 8N1.

Host -> MCU
    ``{"T":13,"X":<linear m/s>,"Z":<angular rad/s>}``  body twist; the MCU does
    the wheel kinematics itself.

MCU -> host, one line per poll
    ``{"T":1001,"L":..,"R":..,"ax":..,...,"odl":..,"odr":..,"v":..}``
"""

import json
import math
from typing import Any


#: One reading across the sensor's three axes, already converted to SI.
Axes = tuple[float, float, float]

#: Feedback frame type emitted by the MCU. Other ``T`` values exist (command
#: acknowledgements, module chatter) and are not errors - see `decode_feedback`.
FEEDBACK_TYPE = 1001

#: Velocity command type. The reference driver sent ``'T': '13'`` as a *string*;
#: the MCU accepts both, but an int is what the protocol documents.
CMD_VELOCITY_TYPE = 13

# Raw-LSB -> SI scale factors, from the ICM-20948 datasheet ranges the firmware
# configures. Documented in HARDWARE_HAL_PLAN.md, "Verified serial protocol".
ACCEL_SCALE = 9.80665 / 8192.0            # 8192 LSB/g at +/-4 g -> m/s^2
GYRO_SCALE = math.pi / (16.4 * 180.0)     # 16.4 LSB/deg/s at +/-2000 dps -> rad/s
MAG_SCALE = 0.15 * 1e-6                   # AK09916 0.15 uT/LSB -> Tesla

#: Fields required in a ``T:1001`` line. ``L`` and ``R`` are deliberately absent:
#: their meaning is uncharacterised, so the codec neither needs nor reports them.
FEEDBACK_FIELDS = ('ax', 'ay', 'az', 'gx', 'gy', 'gz',
                   'mx', 'my', 'mz', 'odl', 'odr', 'v')


class ProtocolError(ValueError):
    """A line arrived but could not be understood.

    Raised for junk on the wire - a truncated line, invalid UTF-8, invalid
    JSON, a missing or non-numeric field. Callers are expected to log it and
    carry on with the next line; it is never fatal.
    """


class Feedback:
    """One decoded ``T:1001`` frame, in SI units.

    Attributes:
        accel: linear acceleration ``(x, y, z)`` in m/s^2.
        gyro: angular velocity ``(x, y, z)`` in rad/s.
        mag: magnetic field ``(x, y, z)`` in Tesla.
        odom_left: accumulated distance travelled by the left *side*, metres.
        odom_right: accumulated distance travelled by the right *side*, metres.
        voltage: battery voltage in volts.
        raw: the parsed JSON object, so callers can reach fields this class
            does not model (``L``, ``R``) without re-parsing.
    """

    __slots__ = ('accel', 'gyro', 'mag', 'odom_left', 'odom_right', 'voltage', 'raw')

    def __init__(
        self,
        accel: Axes,
        gyro: Axes,
        mag: Axes,
        odom_left: float,
        odom_right: float,
        voltage: float,
        raw: dict[str, Any] | None = None,
    ) -> None:
        self.accel = accel
        self.gyro = gyro
        self.mag = mag
        self.odom_left = odom_left
        self.odom_right = odom_right
        self.voltage = voltage
        self.raw = raw if raw is not None else {}

    def __repr__(self) -> str:
        return ('Feedback(accel={0.accel}, gyro={0.gyro}, mag={0.mag}, '
                'odom_left={0.odom_left}, odom_right={0.odom_right}, '
                'voltage={0.voltage})'.format(self))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Feedback):
            return NotImplemented
        return (self.accel == other.accel and self.gyro == other.gyro
                and self.mag == other.mag and self.odom_left == other.odom_left
                and self.odom_right == other.odom_right
                and self.voltage == other.voltage)


class LineReader:
    """Reassembles newline-delimited lines from arbitrarily chopped byte chunks.

    A serial read returns whatever happens to be in the FIFO, so a single read
    can hold half a line, three lines, or a line split across two reads. Feed
    every chunk in and take complete lines out.

    Args:
        max_line_bytes: guard against a stream that never sends a newline. A
            ``T:1001`` line is ~140 bytes, so the 4 KiB default is ~29x
            headroom; exceeding it means the link is producing garbage, and the
            buffer is dropped rather than grown without bound.
    """

    def __init__(self, max_line_bytes: int = 4096) -> None:
        self.max_line_bytes = max_line_bytes
        self.buf = bytearray()
        #: Bytes discarded because a line exceeded ``max_line_bytes``. A running
        #: total, for the node to log and for tests to assert on.
        self.dropped_bytes = 0

    def feed(self, chunk: bytes) -> list[bytes]:
        r"""Add received bytes and return the lines they completed.

        Returns:
            A list of ``bytes``, one per complete line, with the trailing
            ``\\n`` and any ``\\r`` stripped. Empty lines are dropped - the MCU
            emits a stray one after a reset. Returns ``[]`` when the chunk did
            not complete a line.
        """
        if chunk:
            self.buf.extend(chunk)

        lines = []
        while True:
            i = self.buf.find(b'\n')
            if i < 0:
                break
            line = bytes(self.buf[:i]).rstrip(b'\r')
            del self.buf[:i + 1]
            if line:
                lines.append(line)

        # Only a partial line is left. If it is implausibly long there is no
        # newline coming, so throw it away instead of buffering forever.
        if len(self.buf) > self.max_line_bytes:
            self.dropped_bytes += len(self.buf)
            self.buf.clear()

        return lines

    def reset(self) -> None:
        """Discard any partial line. Call after reopening the port."""
        self.buf.clear()


def decode_feedback(line: bytes | bytearray | str) -> Feedback | None:
    """Decode one MCU line into SI units.

    Args:
        line: a single line, ``bytes`` or ``str``, without its newline.

    Returns:
        A `Feedback`, or ``None`` if the line was well-formed JSON but not a
        ``T:1001`` frame. ``None`` means "not for us", not "broken".

    Raises:
        ProtocolError: the line was not decodable UTF-8, not valid JSON, not a
            JSON object, or was missing a required field or had a field that
            was not a finite number.
    """
    if isinstance(line, (bytes, bytearray)):
        try:
            text = line.decode('utf-8')
        except UnicodeDecodeError as exc:
            raise ProtocolError('line is not valid UTF-8: {}'.format(exc)) from exc
    else:
        text = line

    try:
        obj = json.loads(text)
    except ValueError as exc:
        raise ProtocolError('invalid JSON: {} in {!r}'.format(exc, text)) from exc

    if not isinstance(obj, dict):
        raise ProtocolError('expected a JSON object, got {}'.format(type(obj).__name__))

    # The MCU sends T as an int, but it has been seen as a string in the
    # reference driver's own commands, so accept either rather than silently
    # discarding every frame if the firmware ever changes its mind.
    if 'T' not in obj:
        return None
    try:
        frame_type = int(obj['T'])
    except (TypeError, ValueError):
        return None
    if frame_type != FEEDBACK_TYPE:
        return None

    values: dict[str, float] = {}
    for field in FEEDBACK_FIELDS:
        if field not in obj:
            raise ProtocolError('T:{} line missing field {!r}'.format(FEEDBACK_TYPE, field))
        try:
            value = float(obj[field])
        except (TypeError, ValueError) as exc:
            raise ProtocolError(
                'field {!r} is not numeric: {!r}'.format(field, obj[field])) from exc
        # float() happily accepts the strings "nan" and "inf", and JSON itself
        # permits the bare tokens. Letting either through would put a NaN
        # straight into an Imu message and from there into a filter, where it
        # is far harder to trace than a dropped frame.
        if not math.isfinite(value):
            raise ProtocolError(
                'field {!r} is not finite: {!r}'.format(field, obj[field]))
        values[field] = value

    return Feedback(
        accel=(values['ax'] * ACCEL_SCALE,
               values['ay'] * ACCEL_SCALE,
               values['az'] * ACCEL_SCALE),
        gyro=(values['gx'] * GYRO_SCALE,
              values['gy'] * GYRO_SCALE,
              values['gz'] * GYRO_SCALE),
        mag=(values['mx'] * MAG_SCALE,
             values['my'] * MAG_SCALE,
             values['mz'] * MAG_SCALE),
        odom_left=values['odl'],
        odom_right=values['odr'],
        voltage=values['v'],
        raw=obj,
    )


def encode_velocity(linear_x: float, angular_z: float) -> bytes:
    """Encode a body twist as a ``T:13`` command line.

    Args:
        linear_x: forward velocity, m/s.
        angular_z: yaw rate, rad/s.

    Returns:
        ``bytes`` including the trailing newline, ready to write to the port.

    Raises:
        ValueError: either value is NaN or infinite. ``json.dumps`` would emit
            the bare tokens ``NaN``/``Infinity``, which are not valid JSON and
            would desynchronise the MCU's parser.
    """
    linear_x = float(linear_x)
    angular_z = float(angular_z)
    if not math.isfinite(linear_x) or not math.isfinite(angular_z):
        raise ValueError(
            'non-finite twist: linear.x={}, angular.z={}'.format(linear_x, angular_z))
    return (json.dumps({'T': CMD_VELOCITY_TYPE, 'X': linear_x, 'Z': angular_z})
            + '\n').encode('utf-8')


def distance_to_angle(distance: float, wheel_radius: float) -> float:
    """Convert accumulated wheel distance in metres to an angle in radians.

    ``/wheel_encoders`` is a `sensor_msgs/JointState` in radians in simulation,
    and this keeps the hardware topic's units identical even though the wire
    format is metres.

    Raises:
        ValueError: ``wheel_radius`` is not strictly positive.
    """
    if wheel_radius <= 0.0:
        raise ValueError('wheel_radius must be > 0, got {}'.format(wheel_radius))
    return distance / wheel_radius
