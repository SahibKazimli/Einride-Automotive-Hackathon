"""The competition loop as a small state machine. No ROS imports.

Nav2's docking server does all the driving (navigate to the dock's staging
pose, approach using the tag, undock). This only decides WHEN to dock where:

    IDLE       nothing to do (no route, or Saga says stay still)
    DOCKING    a DockRobot task is running for `target`
    DOCKED     at `target`, waiting while Saga loads/unloads
    UNDOCKING  backing out before driving to the next dock
    RETRY_WAIT docking failed; wait a moment and try again

Call `step` periodically with what Saga wants and whether the running task has
finished; execute the returned Action.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Action:
    kind: str                  # 'dock', 'undock' or 'cancel'
    tag: Optional[int] = None  # for 'dock'


class Mission:
    def __init__(self, retry_delay: float = 2.0, redock_after: float = 30.0) -> None:
        self.retry_delay = retry_delay
        #: Seconds docked while Saga still asks for this same dock before trying
        #: again (the dock was probably not counted). 0 disables.
        self.redock_after = redock_after
        self.state = 'IDLE'
        self.target: Optional[int] = None
        self.since = 0.0

    def _go(self, state: str, t: float) -> None:
        self.state = state
        self.since = t

    def step(self, t: float, requested: Optional[int], task_done: bool,
             task_ok: bool) -> Optional[Action]:
        """Advance one tick.

        `requested`: tag id Saga wants us at, or None to stay still.
        `task_done`/`task_ok`: whether the running dock/undock task finished,
        and whether it succeeded (ignored when no task runs).
        """
        state = self.state

        if state == 'IDLE':
            if requested is not None:
                self.target = requested
                self._go('DOCKING', t)
                return Action('dock', requested)
            return None

        if state == 'DOCKING':
            if requested != self.target:
                # Saga changed its mind (or stopped the event) mid-approach.
                self._go('IDLE', t)
                return Action('cancel')
            if task_done:
                self._go('DOCKED' if task_ok else 'RETRY_WAIT', t)
            return None

        if state == 'DOCKED':
            if requested is not None and requested != self.target:
                self._go('UNDOCKING', t)
                return Action('undock')
            if (requested == self.target and self.redock_after > 0.0
                    and t - self.since > self.redock_after):
                self._go('UNDOCKING', t)
                return Action('undock')
            return None   # Saga says stay (None) or has not noticed us yet

        if state == 'UNDOCKING':
            if task_done:
                # Even a failed undock leaves us able to try the next dock;
                # the docking server navigates there from wherever we are.
                self._go('IDLE', t)
                return self.step(t, requested, False, False)
            return None

        if state == 'RETRY_WAIT':
            if t - self.since >= self.retry_delay:
                self._go('IDLE', t)
                return self.step(t, requested, False, False)
            return None

        return None
