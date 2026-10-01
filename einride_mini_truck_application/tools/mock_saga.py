#!/usr/bin/env python3
"""A stand-in for the Saga AI server, following docs/saga-ai.md.

The real server is not available yet. This one serves the same endpoints and
JSON, with routes between the docks you list. Only the Python standard library
is used, so it runs anywhere (laptop or robot):

    python3 mock_saga.py --docks 0 1                # Enter advances each step
    python3 mock_saga.py --docks 0 1 2 --auto 20    # advance every 20 s

The real server notices by itself when the truck arrives; here you press
Enter when the robot has docked (or let --auto do it).
"""

import argparse
from datetime import datetime, timezone
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import random
import threading
import time
from typing import Optional

LETTERS = 'ABCDEFGH'
STEPS = ['loading_pending', 'loading_in_progress', 'loading_complete',
         'unloading_in_progress', 'unloading_complete']


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def dock(tag: int) -> dict:
    return {'letter': LETTERS[tag], 'tag_id': tag}


class MockSaga:
    """Route bookkeeping, without any HTTP (tested in test/test_mock_saga.py)."""

    def __init__(self, docks: list[int], truck_id: int = 1, seed: Optional[int] = None) -> None:
        if len(docks) < 2:
            raise ValueError('need at least two docks')
        self.docks = docks
        self.truck_id = truck_id
        self.random = random.Random(seed)
        self.routes: dict[int, dict] = {}
        self.route: Optional[dict] = None
        self.completed = 0
        self.lock = threading.Lock()
        self.new_route(exclude=None)

    def new_route(self, exclude: Optional[int]) -> None:
        # "The new route never starts at the dock you are standing at."
        source = self.random.choice([d for d in self.docks if d != exclude])
        destination = self.random.choice([d for d in self.docks if d != source])
        route_id = len(self.routes) + 1
        self.route = {
            'route_id': route_id, 'truck_id': self.truck_id,
            'source': dock(source), 'destination': dock(destination),
            'status': STEPS[0], 'next_dock': dock(source),
            'assigned_at': now_iso(), 'completed_at': None,
        }
        self.routes[route_id] = self.route

    def advance(self) -> str:
        """Move the current route one step on; returns a description."""
        with self.lock:
            r = self.route
            status = STEPS[STEPS.index(r['status']) + 1]
            r['status'] = status
            r['next_dock'] = {'loading_complete': r['destination']}.get(status)
            if status == 'unloading_complete':
                r['completed_at'] = now_iso()
                self.completed += 1
                self.new_route(exclude=r['destination']['tag_id'])
                return f'route {r["route_id"]} done -> {self.describe()}'
            return self.describe()

    def describe(self) -> str:
        r = self.route
        target = r['next_dock']['letter'] if r['next_dock'] else 'stay still'
        return (f'route {r["route_id"]} {r["source"]["letter"]}->{r["destination"]["letter"]}: '
                f'{r["status"]} (drive to: {target})')

    def get(self, path: str) -> tuple[int, object]:
        with self.lock:
            if path == '/api/v1/docks':
                return 200, [dock(i) for i in range(8)]
            if path == '/api/v1/route':
                return 200, {'truck_id': self.truck_id, 'event_state': 'running',
                             'route': self.route}
            if path == '/api/v1/status':
                r = self.route
                return 200, {'truck_id': self.truck_id, 'event_state': 'running',
                             'route_id': r['route_id'], 'status': r['status'],
                             'next_dock': r['next_dock'], 'completed_routes': self.completed}
            if path.startswith('/api/v1/routes/'):
                try:
                    return 200, self.routes[int(path.rsplit('/', 1)[1])]
                except (ValueError, KeyError):
                    return 404, {'detail': 'Not Found'}
            return 404, {'detail': 'Not Found'}


def make_handler(saga: MockSaga, token: str):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 - http.server API
            path = self.path.split('?', 1)[0]
            if path != '/api/v1/docks' and self.headers.get('Authorization') != f'Bearer {token}':
                code, body = 401, {'detail': 'Unauthorized'}
            else:
                code, body = saga.get(path)
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *args) -> None:
            pass   # one poll per second would flood the terminal

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description='Mock Saga AI server')
    parser.add_argument('--docks', type=int, nargs='+', default=[0, 1],
                        help='tag ids of the docks you have set up (default: 0 1)')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--token', default='test-token')
    parser.add_argument('--auto', type=float, default=0.0,
                        help='advance automatically every N seconds instead of on Enter')
    args = parser.parse_args()

    saga = MockSaga(args.docks)
    server = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(saga, args.token))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'Mock Saga on port {args.port}, token "{args.token}", docks {args.docks}')
    print(saga.describe())
    try:
        while True:
            if args.auto > 0:
                time.sleep(args.auto)
            else:
                input('[Enter] = next step > ')
            print(saga.advance())
    except (KeyboardInterrupt, EOFError):
        server.shutdown()


if __name__ == '__main__':
    main()
