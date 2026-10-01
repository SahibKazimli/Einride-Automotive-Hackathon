"""Mission state machine: Saga requests -> dock / undock actions."""

from einride_mini_truck_application.mission.mission import Action, Mission


def test_full_delivery() -> None:
    m = Mission()
    assert m.step(0, None, False, False) is None                    # no route yet
    assert m.step(1, 2, False, False) == Action('dock', 2)          # go to source C
    assert m.step(2, 2, False, False) is None                       # docking...
    assert m.step(3, 2, True, True) is None and m.state == 'DOCKED'
    assert m.step(4, None, False, False) is None                    # loading: stay
    assert m.step(8, 5, False, False) == Action('undock')           # loaded: go to F
    assert m.step(9, 5, True, True) == Action('dock', 5)            # undocked -> dock F
    assert m.state == 'DOCKING' and m.target == 5


def test_failed_dock_is_retried() -> None:
    m = Mission(retry_delay=2.0)
    m.step(0, 1, False, False)
    assert m.step(1, 1, True, False) is None and m.state == 'RETRY_WAIT'
    assert m.step(2, 1, False, False) is None
    assert m.step(3.5, 1, False, False) == Action('dock', 1)


def test_saga_changes_target_while_docking() -> None:
    m = Mission()
    m.step(0, 1, False, False)
    assert m.step(1, 3, False, False) == Action('cancel')
    assert m.step(2, 3, False, False) == Action('dock', 3)


def test_stay_still_cancels_driving() -> None:
    m = Mission()
    m.step(0, 1, False, False)
    assert m.step(1, None, False, False) == Action('cancel')
    assert m.state == 'IDLE'


def test_redock_if_saga_never_noticed() -> None:
    m = Mission(redock_after=30.0)
    m.step(0, 1, False, False)
    m.step(1, 1, True, True)
    assert m.step(20, 1, False, False) is None
    assert m.step(32, 1, False, False) == Action('undock')
    assert m.step(33, 1, True, True) == Action('dock', 1)
