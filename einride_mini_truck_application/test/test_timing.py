"""Image pacing for the detector and camera clock offset."""

from einride_mini_truck_application.perception.timing import ClockSkew, ImageThrottle
import pytest


def test_throttle_waits_for_the_detector() -> None:
    throttle = ImageThrottle(max_rate=10.0, busy_timeout=0.5)
    assert throttle.offer(0.00)
    assert not throttle.offer(0.05)     # faster than 10/s
    assert not throttle.offer(0.20)     # detector still busy with the first image
    throttle.answered()
    assert throttle.offer(0.21)


def test_throttle_caps_the_rate_with_a_fast_detector() -> None:
    throttle = ImageThrottle(max_rate=10.0)
    assert throttle.offer(0.00)
    throttle.answered()
    assert not throttle.offer(0.04)
    assert throttle.offer(0.10)


def test_throttle_does_not_stall_on_an_unanswered_image() -> None:
    throttle = ImageThrottle(max_rate=10.0, busy_timeout=0.5)
    assert throttle.offer(0.0)
    assert not throttle.offer(0.4)
    assert throttle.offer(0.6)


def test_clock_in_sync_leaves_stamps_alone() -> None:
    skew = ClockSkew(max_delay=2.0)
    assert skew.correction() == 0.0     # nothing measured yet
    for i, delay in enumerate([0.08, 0.05, 0.30, 0.06]):
        skew.add(100.0 + i + delay, 100.0 + i)
    assert skew.delay() == pytest.approx(0.05)
    assert skew.correction() == 0.0


def test_camera_clock_behind_after_a_clock_step() -> None:
    # System clock jumped 3600 s forward after the driver started.
    skew = ClockSkew(max_delay=2.0)
    for i, delay in enumerate([0.08, 0.05, 0.30]):
        skew.add(3600.0 + i + delay, i)
    assert skew.correction() == pytest.approx(3600.05)


def test_camera_clock_ahead_is_always_corrected() -> None:
    # Stamps from the future: no TF exists yet at that time.
    skew = ClockSkew(max_delay=2.0)
    skew.add(10.0, 10.3)
    skew.add(11.0, 11.2)
    assert skew.correction() == pytest.approx(-0.3)


def test_clock_estimate_follows_a_step_within_the_window() -> None:
    skew = ClockSkew(max_delay=2.0, window=3)
    for i in range(3):
        skew.add(i + 0.05, i)
    for i in range(3, 6):
        skew.add(i + 60.05, i)       # clock stepped 60 s mid-run
    assert skew.correction() == pytest.approx(60.05)
    skew.clear()
    assert skew.delay() is None
