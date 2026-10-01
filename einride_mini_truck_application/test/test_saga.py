"""Saga client: reply parsing, and a real HTTP round trip against the mock server."""

from http.server import ThreadingHTTPServer
import os
import sys
import threading

from einride_mini_truck_application.saga.client import parse_route_reply, SagaApi
import pytest
import requests

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from mock_saga import make_handler, MockSaga  # noqa: E402

# The example reply from docs/saga-ai.md.
DOC_REPLY = {
    'truck_id': 2,
    'event_state': 'running',
    'route': {
        'route_id': 5,
        'truck_id': 2,
        'source': {'letter': 'C', 'tag_id': 2},
        'destination': {'letter': 'F', 'tag_id': 5},
        'status': 'loading_pending',
        'next_dock': {'letter': 'C', 'tag_id': 2},
        'assigned_at': '2026-10-03T09:15:02.113Z',
        'completed_at': None,
    },
}


def test_parse_doc_example() -> None:
    state = parse_route_reply(DOC_REPLY)
    assert state.route_id == 5
    assert state.next_tag == 2
    assert state.next_letter == 'C'
    assert (state.source_letter, state.destination_letter) == ('C', 'F')


def test_stay_still_when_next_dock_is_null() -> None:
    reply = {**DOC_REPLY, 'route': {**DOC_REPLY['route'], 'next_dock': None,
                                    'status': 'loading_in_progress'}}
    assert parse_route_reply(reply).next_tag is None


def test_no_route_before_event() -> None:
    state = parse_route_reply({'truck_id': 2, 'event_state': 'not_started', 'route': None})
    assert state.route_id is None and state.next_tag is None


def test_stay_still_when_event_stopped() -> None:
    assert parse_route_reply({**DOC_REPLY, 'event_state': 'stopped'}).next_tag is None


@pytest.fixture
def server():
    saga = MockSaga([0, 1, 2], seed=1)
    httpd = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(saga, 'secret'))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield saga, f'http://127.0.0.1:{httpd.server_address[1]}'
    httpd.shutdown()


def test_client_against_mock_server(server) -> None:
    saga, url = server
    api = SagaApi(url, 'secret')
    first = parse_route_reply(api.get_route_reply())
    assert first.next_tag == saga.route['source']['tag_id']

    saga.advance()   # loading in progress: stay still
    assert parse_route_reply(api.get_route_reply()).next_tag is None
    saga.advance()   # loading complete: go to the destination
    assert parse_route_reply(api.get_route_reply()).next_tag == \
        saga.route['destination']['tag_id']


def test_wrong_token_is_401(server) -> None:
    _, url = server
    with pytest.raises(requests.HTTPError):
        SagaApi(url, 'wrong').get_route_reply()
