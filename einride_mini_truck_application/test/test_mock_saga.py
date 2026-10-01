"""The mock server follows the route rules in docs/saga-ai.md."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from mock_saga import MockSaga  # noqa: E402


def test_full_route_cycle() -> None:
    saga = MockSaga([0, 1, 2, 3], seed=0)
    route = saga.route
    source, destination = route['source']['tag_id'], route['destination']['tag_id']
    assert source != destination
    seen = []
    for _ in range(4):
        saga.advance()
        seen.append((saga.route['status'], saga.route['next_dock']))
    assert seen[0] == ('loading_in_progress', None)
    assert seen[1][1]['tag_id'] == destination
    assert seen[2] == ('unloading_in_progress', None)
    saga.advance()   # unloading complete -> new route immediately
    assert saga.route['route_id'] == route['route_id'] + 1
    assert route['status'] == 'unloading_complete'
    assert saga.completed == 1
    # "The new route never starts at the dock you are standing at."
    assert saga.route['source']['tag_id'] != destination


def test_new_routes_never_start_where_we_stand() -> None:
    saga = MockSaga([0, 1], seed=3)
    for _ in range(20):
        standing = saga.route['destination']['tag_id']
        for _ in range(5):
            saga.advance()
        assert saga.route['source']['tag_id'] != standing


def test_endpoints() -> None:
    saga = MockSaga([0, 1], seed=0)
    assert saga.get('/api/v1/status')[1]['completed_routes'] == 0
    assert saga.get('/api/v1/routes/1')[1]['route_id'] == 1
    assert saga.get('/api/v1/routes/99')[0] == 404
    assert len(saga.get('/api/v1/docks')[1]) == 8
