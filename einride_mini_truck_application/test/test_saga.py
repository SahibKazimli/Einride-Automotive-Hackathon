"""Saga client: parsing /api/v1/route replies (docs/saga-ai.md)."""

from einride_mini_truck_application.saga.client import parse_route_reply

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
