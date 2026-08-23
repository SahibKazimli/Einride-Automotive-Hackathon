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

"""Unit tests for the pure codec. No ROS, no serial port, no hardware."""

import json
import math
import os

from einride_mini_truck_hardware.serial_link import (
    decode_feedback,
    distance_to_angle,
    encode_velocity,
    LineReader,
    ProtocolError,
)
import pytest

CAPTURE = os.path.join(os.path.dirname(__file__), 'data', 'ugv02_feedback.jsonl')

# A representative frame with values chosen so every conversion has a hand
# checkable answer: az is exactly 1 g, gz is exactly 1 rad/s.
SAMPLE = {
    'T': 1001, 'L': 0, 'R': 0,
    'ax': 0, 'ay': 0, 'az': 8192,
    'gx': 0, 'gy': 0, 'gz': 940,
    'mx': 100, 'my': -200, 'mz': 300,
    'odl': 1.5, 'odr': 1.4, 'v': 11.7,
}
SAMPLE_LINE = json.dumps(SAMPLE, separators=(',', ':')).encode()


# --------------------------------------------------------------- framing

def test_reader_splits_a_whole_line() -> None:
    reader = LineReader()
    assert reader.feed(b'{"a":1}\n') == [b'{"a":1}']


def test_reader_holds_a_partial_line_until_it_completes() -> None:
    reader = LineReader()
    assert reader.feed(b'{"a":') == []
    assert reader.feed(b'1}\n') == [b'{"a":1}']


def test_reader_splits_a_line_across_many_chunks() -> None:
    """The FIFO can hand back one byte at a time; framing must not care."""
    reader = LineReader()
    out = []
    for byte in SAMPLE_LINE + b'\n':
        out.extend(reader.feed(bytes([byte])))
    assert out == [SAMPLE_LINE]


def test_reader_returns_several_lines_from_one_chunk() -> None:
    reader = LineReader()
    assert reader.feed(b'a\nb\nc\n') == [b'a', b'b', b'c']


def test_reader_keeps_the_remainder_after_a_burst() -> None:
    reader = LineReader()
    assert reader.feed(b'a\nb\npartial') == [b'a', b'b']
    assert reader.feed(b'\n') == [b'partial']


def test_reader_strips_carriage_returns() -> None:
    assert LineReader().feed(b'{"a":1}\r\n') == [b'{"a":1}']


def test_reader_drops_empty_lines() -> None:
    """The MCU emits a stray newline after a reset."""
    assert LineReader().feed(b'\n\na\n\n') == [b'a']


def test_reader_discards_a_line_that_never_terminates() -> None:
    reader = LineReader(max_line_bytes=64)
    assert reader.feed(b'x' * 100) == []
    assert reader.dropped_bytes == 100
    # Framing recovers on the next real line rather than staying wedged.
    assert reader.feed(b'good\n') == [b'good']


def test_reader_reset_drops_the_partial_line() -> None:
    reader = LineReader()
    reader.feed(b'partial')
    reader.reset()
    assert reader.feed(b'rest\n') == [b'rest']


# ------------------------------------------------------------ conversions

def test_decode_converts_to_si() -> None:
    fb = decode_feedback(SAMPLE_LINE)
    assert fb is not None
    # 8192 LSB/g at +/-4 g, so az of 8192 is exactly one g.
    assert fb.accel == pytest.approx((0.0, 0.0, 9.80665))
    # 16.4 LSB/deg/s, so 940 LSB is 57.3 deg/s is 1 rad/s.
    assert fb.gyro[2] == pytest.approx(1.0, rel=1e-3)
    # 0.15 uT/LSB, reported in Tesla.
    assert fb.mag == pytest.approx((100 * 0.15e-6, -200 * 0.15e-6, 300 * 0.15e-6))
    assert fb.odom_left == 1.5 and fb.odom_right == 1.4
    assert fb.voltage == pytest.approx(11.7)


def test_decode_keeps_the_raw_object() -> None:
    """L and R are uncharacterised, so the codec passes them through untouched."""
    fb = decode_feedback(SAMPLE_LINE)
    assert fb is not None
    assert fb.raw['L'] == 0 and fb.raw['R'] == 0


def test_decode_accepts_str_as_well_as_bytes() -> None:
    assert decode_feedback(SAMPLE_LINE.decode()) == decode_feedback(SAMPLE_LINE)


def test_decode_accepts_integer_valued_fields() -> None:
    """The MCU sends ints when a value is whole; they must not be rejected."""
    line = json.dumps(dict(SAMPLE, odl=2, odr=2, v=12)).encode()
    fb = decode_feedback(line)
    assert fb is not None
    assert isinstance(fb.odom_left, float) and fb.odom_left == 2.0


# ------------------------------------------------------- malformed input

def test_decode_returns_none_for_another_frame_type() -> None:
    """Not every line is ours. A T:1002 line is valid, just not feedback."""
    assert decode_feedback(b'{"T":1002,"status":"ok"}') is None


def test_decode_returns_none_when_T_is_missing_or_odd() -> None:
    assert decode_feedback(b'{"status":"ok"}') is None
    assert decode_feedback(b'{"T":"hello"}') is None


def test_decode_accepts_a_string_frame_type() -> None:
    assert decode_feedback(json.dumps(dict(SAMPLE, T='1001')).encode()) is not None


@pytest.mark.parametrize('line', [
    b'{"T":1001,',                 # truncated, as after a mid-line reconnect
    b'not json at all',
    b'',
    b'\x80\x81\x82',               # invalid UTF-8
    b'[1, 2, 3]',                  # valid JSON, wrong shape
])
def test_decode_rejects_junk(line: bytes) -> None:
    with pytest.raises(ProtocolError):
        decode_feedback(line)


def test_decode_rejects_a_missing_field() -> None:
    line = json.dumps({k: v for k, v in SAMPLE.items() if k != 'gz'}).encode()
    with pytest.raises(ProtocolError, match='gz'):
        decode_feedback(line)


def test_decode_rejects_a_non_numeric_field() -> None:
    line = json.dumps(dict(SAMPLE, ax='oops')).encode()
    with pytest.raises(ProtocolError, match='ax'):
        decode_feedback(line)


@pytest.mark.parametrize('text', ['nan', 'inf', '-inf', 'Infinity'])
def test_decode_rejects_a_non_finite_string(text: str) -> None:
    """float() accepts all of these, so a bare type check is not enough."""
    with pytest.raises(ProtocolError, match='finite'):
        decode_feedback(json.dumps(dict(SAMPLE, ax=text)).encode())


@pytest.mark.parametrize('token', [b'NaN', b'Infinity', b'-Infinity'])
def test_decode_rejects_a_non_finite_literal(token: bytes) -> None:
    """Python's json accepts these bare tokens even though JSON does not.

    A NaN that reaches an Imu message poisons the filter downstream and is far
    harder to trace there than a dropped frame is here.
    """
    line = SAMPLE_LINE.replace(b'"ax":0', b'"ax":' + token)
    with pytest.raises(ProtocolError, match='finite'):
        decode_feedback(line)


# -------------------------------------------------------------- commands

def test_encode_velocity_round_trips() -> None:
    assert json.loads(encode_velocity(0.25, -1.5)) == {'T': 13, 'X': 0.25, 'Z': -1.5}


def test_encode_velocity_terminates_the_line() -> None:
    assert encode_velocity(0.0, 0.0).endswith(b'\n')


@pytest.mark.parametrize('linear,angular', [
    (float('nan'), 0.0), (0.0, float('inf')), (float('-inf'), 0.0)])
def test_encode_velocity_rejects_non_finite(linear: float, angular: float) -> None:
    """json.dumps would emit bare NaN/Infinity, which is not valid JSON."""
    with pytest.raises(ValueError):
        encode_velocity(linear, angular)


# ---------------------------------------------------------------- angles

def test_distance_to_angle() -> None:
    """One wheel circumference of travel is one full turn."""
    assert distance_to_angle(2 * math.pi * 0.04, 0.04) == pytest.approx(2 * math.pi)


def test_distance_to_angle_rejects_a_bad_radius() -> None:
    with pytest.raises(ValueError):
        distance_to_angle(1.0, 0.0)


# ---------------------------------------------------------------- replay

def test_capture_decodes_end_to_end() -> None:
    """Replay the capture through framing and decoding, one byte at a time.

    Byte-at-a-time is the worst case the FIFO can produce, so it proves the
    framing has no dependence on how the reads happen to land.
    """
    reader = LineReader()
    with open(CAPTURE, 'rb') as handle:
        raw = handle.read()

    frames = []
    for i in range(0, len(raw), 7):        # a chunk size that divides nothing
        for line in reader.feed(raw[i:i + 7]):
            frames.append(decode_feedback(line))

    assert len(frames) == raw.count(b'\n')
    assert all(f is not None for f in frames)
    decoded = [f for f in frames if f is not None]

    # Accumulated distance is monotonic while driving forward, and the
    # accelerometer sees ~1 g throughout.
    forward = decoded[80:160]
    assert all(b.odom_left >= a.odom_left for a, b in zip(forward, forward[1:]))
    assert all(abs(f.accel[2] - 9.80665) < 0.1 for f in decoded)

    # The pivot segment turns left: right side forward, left side back.
    pivot = decoded[-40:]
    assert pivot[-1].odom_right > pivot[0].odom_right
    assert pivot[-1].odom_left < pivot[0].odom_left
    assert all(abs(f.gyro[2] - 1.0) < 0.01 for f in pivot)
