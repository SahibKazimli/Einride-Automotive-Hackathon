"""Inputs for the robot_localization EKF that the hardware does not provide.

No ROS imports, so it is unit-tested on a laptop.

The hardware publishes raw wheel angles (/wheel_encoders) and a raw gyro
(/imu), but no /odom. robot_localization does the actual fusion; it needs:

* a wheel *velocity* (it fuses twist, not accumulated angles), from
  `WheelSpeed`, and
* a gyro without its constant bias. The EKF has no bias state, so a 0.5 deg/s
  bias would become 30 deg of heading drift per minute. `GyroBias` estimates
  it whenever the wheels are not turning (startup, waiting at docks).
"""

from collections import deque
from typing import Optional


class WheelSpeed:
    """Forward speed from two accumulated wheel angles (radians).

    The real encoders count in whole centimetres (one step = 0.25 rad at
    r = 0.04 m), so one 50 ms sample reads either 0 or 0.2 m/s. The speed is
    therefore taken over the last `window` seconds instead of one sample.
    """

    def __init__(self, wheel_radius: float = 0.040, max_step: float = 2.0,
                 still_deadband: float = 0.005, still_time: float = 0.3,
                 window: float = 0.4) -> None:
        self.wheel_radius = wheel_radius
        #: A larger jump between two samples means the MCU reset its counters.
        self.max_step = max_step
        #: Wheel angle change per sample (rad) below which the wheel is "not moving".
        self.still_deadband = still_deadband
        self.still_time = still_time
        self.window = window
        self._samples: deque[tuple[float, float, float]] = deque()
        self._last_motion_t: Optional[float] = None

    def update(self, t: float, left: float, right: float) -> Optional[float]:
        """Return forward speed in m/s, or None for the first/invalid sample."""
        samples = self._samples
        if not samples:
            samples.append((t, left, right))
            self._last_motion_t = t
            return None
        last = samples[-1]
        dt = t - last[0]
        dl = left - last[1]
        dr = right - last[2]
        if dt <= 0.0:
            return None
        if abs(dl) > self.max_step or abs(dr) > self.max_step:
            samples.clear()            # counter reset: start over from here
            samples.append((t, left, right))
            return None
        if abs(dl) > self.still_deadband or abs(dr) > self.still_deadband:
            self._last_motion_t = t
        samples.append((t, left, right))
        while len(samples) > 2 and t - samples[1][0] >= self.window:
            samples.popleft()
        first = samples[0]
        span = t - first[0]
        return self.wheel_radius * ((left - first[1]) + (right - first[2])) / 2.0 / span

    def is_still(self, t: float) -> bool:
        """True when neither wheel has moved for `still_time` seconds."""
        if self._last_motion_t is None:
            return False
        return t - self._last_motion_t >= self.still_time


class GyroBias:
    """Running estimate of the gyro's z bias, updated only while standing still."""

    def __init__(self, startup_samples: int = 200, alpha: float = 0.005,
                 max_still_rate: float = 0.05) -> None:
        self.startup_samples = startup_samples
        self.alpha = alpha
        #: Above this |rate - bias| (rad/s) the robot is turning, whatever the
        #: wheels say. The encoders tick per centimetre, so a slow turn in place
        #: (0.35 rad/s, ~3.5 cm/s per wheel) has gaps of ~0.3 s between ticks:
        #: wheels-only "still" then zeroed real rotation and learned it as bias
        #: (92 deg of heading lost in one lap, 2026-10-07).
        self.max_still_rate = max_still_rate
        self.bias = 0.0
        self.count = 0

    @property
    def calibrated(self) -> bool:
        return self.count >= self.startup_samples

    def update(self, rate: float, still: bool) -> float:
        """Feed one raw z rate; return it with the bias removed (0 when still).

        `still` is what the wheels say; a gyro clearly showing rotation overrides it.
        """
        if still and abs(rate - self.bias) <= self.max_still_rate:
            self.count += 1
            if self.count <= self.startup_samples:
                self.bias += (rate - self.bias) / self.count   # plain mean at startup
            else:
                self.bias += self.alpha * (rate - self.bias)   # slow tracking later
            return 0.0
        return rate - self.bias
