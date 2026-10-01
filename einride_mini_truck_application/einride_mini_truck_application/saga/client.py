"""Saga AI client and reply parsing (see docs/saga-ai.md). No ROS imports."""

from dataclasses import dataclass
from typing import Any, Optional

import requests


class SagaApi:
    """Asks the Saga AI server about our truck's route."""

    def __init__(self, server_url: str, token: str, timeout: float = 0.5) -> None:
        self.server_url = server_url.rstrip('/')
        self.headers = {'Authorization': f'Bearer {token}'}
        self.timeout = timeout

    def get(self, path: str) -> Any:
        """GET a path; raises requests.RequestException on any failure."""
        reply = requests.get(self.server_url + path, headers=self.headers,
                             timeout=self.timeout)
        reply.raise_for_status()
        return reply.json()

    def get_route_reply(self) -> dict:
        """Return the full /api/v1/route reply."""
        return self.get('/api/v1/route')


@dataclass(frozen=True)
class RouteState:
    """What Saga wants from the truck right now."""

    event_state: str
    route_id: Optional[int]
    status: Optional[str]
    #: AprilTag id of the dock to drive to, or None to stay still.
    next_tag: Optional[int]
    next_letter: Optional[str]
    source_letter: Optional[str]
    destination_letter: Optional[str]


def parse_route_reply(reply: dict) -> RouteState:
    """Turn a /api/v1/route reply into a RouteState.

    A missing route (event not started) or an event that is not running both
    mean "stay still".
    """
    event_state = str(reply.get('event_state', 'unknown'))
    route = reply.get('route')
    if not route:
        return RouteState(event_state, None, None, None, None, None, None)
    next_dock = route.get('next_dock')
    next_tag = None
    next_letter = None
    if next_dock is not None and event_state == 'running':
        next_tag = int(next_dock['tag_id'])
        next_letter = str(next_dock.get('letter', '?'))
    return RouteState(
        event_state=event_state,
        route_id=int(route['route_id']),
        status=route.get('status'),
        next_tag=next_tag,
        next_letter=next_letter,
        source_letter=(route.get('source') or {}).get('letter'),
        destination_letter=(route.get('destination') or {}).get('letter'),
    )
