"""Camera gate: tag detection only near the target dock."""

import os

from einride_mini_truck_application.perception.camera_gate import (
    CameraGate, load_dock_positions)
import pytest

HOME = os.path.join(os.path.dirname(__file__), '..', 'config', 'docks', 'home.yaml')


def test_opens_near_dock_with_hysteresis() -> None:
    gate = CameraGate(on_distance=1.6, off_distance=1.9)
    dock = (2.0, 0.0)
    assert not gate.update((0.0, 0.0), dock)     # 2.0 m away
    assert gate.update((0.5, 0.0), dock)         # 1.5 m: open
    assert gate.update((0.2, 0.0), dock)         # 1.8 m: still open (between on and off)
    assert not gate.update((0.0, 0.0), dock)     # 2.0 m: closed again


def test_no_target_closes_and_lost_position_opens() -> None:
    gate = CameraGate()
    assert gate.update(None, (2.0, 0.0))
    assert not gate.update((1.9, 0.0), None)


def test_home_layout_has_all_eight_docks() -> None:
    docks = load_dock_positions(HOME)
    assert sorted(docks) == list(range(8))
    assert docks[6] == pytest.approx((1.674, 0.0))   # G: spot 1
    assert docks[3] == pytest.approx((0.42, 0.674))  # D: spot 2
