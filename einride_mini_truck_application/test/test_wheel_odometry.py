"""Wheel speed and gyro bias preparation for the EKF."""

import math

from einride_mini_truck_application.localization.wheel_odometry import GyroBias, WheelSpeed
import pytest


def test_forward_speed() -> None:
    wheels = WheelSpeed(wheel_radius=0.04)
    assert wheels.update(0.0, 0.0, 0.0) is None
    # 2.5 rad/s on both wheels at r = 0.04 m -> 0.1 m/s
    assert wheels.update(0.1, 0.25, 0.25) == pytest.approx(0.1)


def test_centimetre_steps_average_out() -> None:
    """Real encoders step 1 cm (0.25 rad) at a time; 0.2 m/s = a step every 50 ms
    on average, but the steps arrive unevenly."""
    wheels = WheelSpeed(wheel_radius=0.04, window=0.4)
    angle, speeds = 0.0, []
    for i in range(1, 41):                   # 2 s at 20 Hz, steps on 3 of 4 samples
        if i % 4:
            angle += 0.25
        v = wheels.update(i * 0.05, angle, angle)
        if i > 10:
            speeds.append(v)
    for v in speeds:
        assert v == pytest.approx(0.15, abs=0.04)


def test_speed_returns_to_zero_after_stopping() -> None:
    wheels = WheelSpeed(window=0.4)
    for i in range(10):
        wheels.update(i * 0.05, i * 0.25, i * 0.25)
    v = 1.0
    for i in range(10, 25):
        v = wheels.update(i * 0.05, 9 * 0.25, 9 * 0.25)
    assert v == pytest.approx(0.0)


def test_turning_in_place_has_no_forward_speed() -> None:
    wheels = WheelSpeed()
    wheels.update(0.0, 0.0, 0.0)
    assert wheels.update(0.1, -0.2, 0.2) == pytest.approx(0.0)


def test_counter_reset_is_skipped() -> None:
    wheels = WheelSpeed()
    wheels.update(0.0, 100.0, 100.0)
    assert wheels.update(0.01, 0.0, 0.0) is None


def test_still_detection() -> None:
    wheels = WheelSpeed(still_time=0.3)
    wheels.update(0.0, 0.0, 0.0)
    wheels.update(0.1, 0.1, 0.1)          # moving
    assert not wheels.is_still(0.2)
    wheels.update(0.2, 0.1001, 0.1)       # jitter below the deadband
    assert not wheels.is_still(0.3)
    assert wheels.is_still(0.45)


def test_gyro_bias_removed() -> None:
    bias = GyroBias(startup_samples=100)
    for _ in range(100):
        assert bias.update(0.01, still=True) == 0.0
    assert bias.calibrated
    assert bias.bias == pytest.approx(0.01)
    # Turning at 1 rad/s reads 1.01 on this gyro.
    assert bias.update(1.01, still=False) == pytest.approx(1.0)


def test_bias_drift_over_a_minute_is_removed() -> None:
    """Without correction a 0.5 deg/s bias is 30 deg of heading per minute."""
    raw_bias = math.radians(0.5)
    bias = GyroBias(startup_samples=200)
    for _ in range(200):
        bias.update(raw_bias, still=True)
    heading = sum(bias.update(raw_bias, still=False) * 0.0125 for _ in range(4800))
    assert abs(math.degrees(heading)) < 0.1
