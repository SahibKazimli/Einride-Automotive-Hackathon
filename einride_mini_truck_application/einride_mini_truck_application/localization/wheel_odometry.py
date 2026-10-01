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

from typing import Optional


class WheelSpeed:
    """Forward speed from two accumulated wheel angles (radians)."""

    def __init__(self, wheel_radius: float = 0.040, max_step: float = 2.0,
                 still_deadband: float = 0.005, still_time: float = 0.3) -> None:
        self.wheel_radius = wheel_radius
        #: A larger jump between two samples means the MCU reset its counters.
        self.max_step = max_step
        #: Wheel angle change per sample (rad) below which the wheel is "not moving".
        self.still_deadband = still_deadband
        self.still_time = still_time
        self._last: Optional[tuple[float, float, float]] = None
        self._last_motion_t: Optional[float] = None

    def update(self, t: float, left: float, right: float) -> Optional[float]:
        """Return forward speed in m/s, or None for the first/invalid sample."""
        last = self._last
        self._last = (t, left, right)
        if last is None:
            self._last_motion_t = t
            return None
        dt = t - last[0]
        dl = left - last[1]
        dr = right - last[2]
        if dt <= 0.0 or abs(dl) > self.max_step or abs(dr) > self.max_step:
            return None
        if abs(dl) > self.still_deadband or abs(dr) > self.still_deadband:
            self._last_motion_t = t
        return self.wheel_radius * (dl + dr) / 2.0 / dt

    def is_still(self, t: float) -> bool:
        """True when neither wheel has moved for `still_time` seconds."""
        if self._last_motion_t is None:
            return False
        return t - self._last_motion_t >= self.still_time


class GyroBias:
    """Running estimate of the gyro's z bias, updated only while standing still."""

    def __init__(self, startup_samples: int = 200, alpha: float = 0.005) -> None:
        self.startup_samples = startup_samples
        self.alpha = alpha
        self.bias = 0.0
        self.count = 0

    @property
    def calibrated(self) -> bool:
        return self.count >= self.startup_samples

    def update(self, rate: float, still: bool) -> float:
        """Feed one raw z rate; return it with the bias removed (0 when still)."""
        if still:
            self.count += 1
            if self.count <= self.startup_samples:
                self.bias += (rate - self.bias) / self.count   # plain mean at startup
            else:
                self.bias += self.alpha * (rate - self.bias)   # slow tracking later
            return 0.0
        return rate - self.bias
