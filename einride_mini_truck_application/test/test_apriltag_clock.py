"""Pure Python regression tests for AprilTag clock gating and saved catalogs."""

import math

import pytest
import yaml

from einride_mini_truck_application.perception.tag_catalog import write_dock_database
from einride_mini_truck_application.perception.timestamp import is_detection_stamp_fresh


@pytest.mark.parametrize(('offset', 'expected'), [
    (0.0, True),
    (0.999, True),
    (1.0, True),
    (1.001, False),
    (-1.001, False),
])
def test_detection_timestamp_must_be_within_one_second(offset, expected) -> None:
    assert is_detection_stamp_fresh(offset) is expected


def test_saved_catalog_keeps_physical_tag_pose_and_map_identity(tmp_path) -> None:
    path = tmp_path / 'kitchen_tags.yaml'
    tag_pose = (1.25, -0.5, math.pi / 2)
    map_identity = 'map-fingerprint-123'

    with path.open('w', encoding='utf-8') as out:
        write_dock_database(
            out,
            docks={0: (0.924, -0.5, math.pi / 2)},
            counts={0: 8},
            frame='map',
            tag_poses={0: tag_pose},
            map_identity=map_identity,
        )

    catalog = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert catalog['map_identity'] == map_identity
    assert catalog['frame'] == 'map'
    assert catalog['tags']['tag_0']['name'] == 'A'
    assert catalog['tags']['tag_0']['observations'] == 8
    assert catalog['tags']['tag_0']['pose'] == pytest.approx(tag_pose, abs=1e-4)
