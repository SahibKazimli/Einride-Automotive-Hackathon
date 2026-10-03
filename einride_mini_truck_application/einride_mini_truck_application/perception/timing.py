"""When to hand an image to the tag detector, and what its stamps mean. No ROS imports.

Two timing problems sit between the camera and Nav2's docking server, which
drops any dock pose whose stamp is more than `external_detection_timeout`
behind its own clock:

* Queueing. apriltag_ros keeps up to 10 images waiting. Fed faster than it can
  detect, every result is about a second old before it is even computed.
  ImageThrottle sends the next image only once the detector has answered.
* The camera's clock. depthai_ros_driver stamps images with the system time
  read once when the driver started, plus the camera's own elapsed time. If
  the system clock is stepped later (NTP syncing after boot), every camera
  stamp is off by that step for as long as the driver runs. ClockSkew measures
  the offset from arrival times and says how much to shift stamps by.
"""

from collections import deque
from typing import Optional


class ImageThrottle:
    """At most `max_rate` images per second, and only one awaiting detection.

    An image counts as answered after `busy_timeout` s even without a
    result, so a frame the detector skipped cannot stall the stream.
    """

    def __init__(self, max_rate: float = 10.0, busy_timeout: float = 0.5) -> None:
        self.min_period = 1.0 / max_rate
        self.busy_timeout = busy_timeout
        self.last_sent: Optional[float] = None
        self.waiting = False

    def offer(self, now: float) -> bool:
        """An image arrived at time `now`. Returns True if it should be sent."""
        if self.last_sent is not None:
            since = now - self.last_sent
            if since < self.min_period or (self.waiting and since < self.busy_timeout):
                return False
        self.last_sent = now
        self.waiting = True
        return True

    def answered(self) -> None:
        """The detector published a result."""
        self.waiting = False


class ClockSkew:
    """How far camera stamps are from our clock, from when messages arrive.

    Arrival minus stamp is the camera's offset plus the transport delay. The
    smallest value over the last `window` messages is the offset plus the
    shortest delay. A real delay is positive and short; anything negative
    (stamps from the future) or above `max_delay` s can only be the clock.
    """

    def __init__(self, max_delay: float = 2.0, window: int = 50) -> None:
        self.max_delay = max_delay
        self.delays: deque[float] = deque(maxlen=window)

    def add(self, arrival: float, stamp: float) -> None:
        self.delays.append(arrival - stamp)

    def clear(self) -> None:
        self.delays.clear()

    def delay(self) -> Optional[float]:
        """Shortest recent arrival - stamp, s. None before any message."""
        return min(self.delays) if self.delays else None

    def correction(self) -> float:
        """Seconds to add to camera stamps to put them on our clock (0 if they already are)."""
        delay = self.delay()
        if delay is None or 0.0 <= delay <= self.max_delay:
            return 0.0
        return delay
