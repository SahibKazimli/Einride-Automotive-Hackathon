# Saga AI

In the competition, your truck gets its jobs from **Saga AI**, a small version of the
system Einride uses to plan real truck deliveries. Saga AI tells your truck where to
pick up goods and where to deliver them. Your truck asks the Saga AI server over
Wi-Fi, using a simple web API.

This guide explains how the API works and shows Python code you can put inside your
own ROS 2 node.

## What you get from the organizers

| What | Example | What it is for |
|---|---|---|
| Server address | `http://192.168.1.100:8000` | where Saga AI runs (the real address is given on event day) |
| Truck number | `2` | which truck is yours (1-4) |
| Token | `k3J9x...` | your team's secret password for the API |

Your truck must be connected to the event Wi-Fi to reach the server.

**Keep your token secret.** It tells the server which truck is asking. Don't put it
in a public Git repository.

## The docks

There are 8 docks. Each dock has a letter and an AprilTag (family `tag36h11`), so your
truck can recognise it with its camera.

| Dock | A | B | C | D | E | F | G | H |
|---|---|---|---|---|---|---|---|---|
| AprilTag id | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |

## How a delivery works

Your truck always has **one route** at a time. A route has a **source** dock (where you
pick up goods) and a **destination** dock (where you deliver them). Each route has a
number, `route_id`.

A route goes through five steps. Your truck only reads the status; it never
changes it.

| Status | What is happening | What your truck should do |
|---|---|---|
| `loading_pending` | Goods are waiting at the source dock | Drive to the **source** dock |
| `loading_in_progress` | You arrived; goods are being loaded | Stay still |
| `loading_complete` | Goods are on board | Drive to the **destination** dock |
| `unloading_in_progress` | You arrived; goods are being unloaded | Stay still |
| `unloading_complete` | Delivery done! | (you get a new route straight away) |

To make this easy, the API also gives you **`next_dock`**: the dock to drive to right
now. When `next_dock` is `null` (`None` in Python), your truck should stay where it is.

When a delivery is finished, Saga AI gives your truck a new route **immediately**. You
notice this because the `route_id` changes. The new route never starts at the dock
you are standing at, so you always have to drive somewhere. Sometimes the new route
uses the same two docks as the last one, so always watch `route_id`, not the dock
letters, to spot a new job.

Loading and unloading take at least 3 seconds, so stay still until `next_dock` tells
you to move.

## How to test

The server implementation can be found in the (GitHub
repository)[https://github.com/einride-labs/ai-powered-autonomous-challenge-server].
Follow instructions in the README.md file to run it locally and test against
it.

## The API

Every call is an HTTP `GET` request. Send your token in a header on every call (except
`/docks`):

```
Authorization: Bearer YOUR_TOKEN
```

The server answers with JSON.

### `GET /api/v1/route` — your current route

This is the one you need most. It has everything: the docks, the status and where to
drive next.

```json
{
  "truck_id": 2,
  "event_state": "running",
  "route": {
    "route_id": 5,
    "truck_id": 2,
    "source": {"letter": "C", "tag_id": 2},
    "destination": {"letter": "F", "tag_id": 5},
    "status": "loading_pending",
    "next_dock": {"letter": "C", "tag_id": 2},
    "assigned_at": "2026-10-03T09:15:02.113Z",
    "completed_at": null
  }
}
```

- `route` is `null` before the event starts.
- `event_state` is `not_started`, `running` or `stopped`.
- Times are in UTC.

### `GET /api/v1/status` — a short summary

```json
{
  "truck_id": 2,
  "event_state": "running",
  "route_id": 5,
  "status": "loading_pending",
  "next_dock": {"letter": "C", "tag_id": 2},
  "completed_routes": 1
}
```

`completed_routes` is how many deliveries your truck has finished.

### `GET /api/v1/routes/{route_id}` — an old route

Returns one of your routes by its number, also finished ones. Useful if you want to
check that your last delivery really was completed:

```json
{
  "route_id": 1,
  "truck_id": 2,
  "source": {"letter": "E", "tag_id": 4},
  "destination": {"letter": "H", "tag_id": 7},
  "status": "unloading_complete",
  "next_dock": null,
  "assigned_at": "2026-10-03T09:10:40.527Z",
  "completed_at": "2026-10-03T09:15:02.113Z"
}
```

### `GET /api/v1/docks` — all docks

A list of all 8 docks with their letters and AprilTag ids. No token needed.

```json
[{"letter": "A", "tag_id": 0}, {"letter": "B", "tag_id": 1}, ...]
```

### When something goes wrong

| Answer | Meaning |
|---|---|
| `401 Unauthorized` | The token is missing or wrong. Check for typos and spaces. |
| `404 Not Found` | That `route_id` is not one of your routes. |
| No answer / timeout | Your truck can't reach the server. Check the Wi-Fi and the address. |

## Python code for your ROS 2 node

The examples use the `requests` library. It is normally already installed together
with ROS 2; if `import requests` fails, run `sudo apt install python3-requests`.

### Step 1: a small helper to talk to Saga AI

Put this class in your Python code (for example at the top of your node's file):

```python
import requests


class SagaApi:
    """Asks the Saga AI server about our truck's route."""

    def __init__(self, server_url, token):
        self.server_url = server_url.rstrip('/')
        self.headers = {'Authorization': f'Bearer {token}'}

    def get(self, path):
        reply = requests.get(
            self.server_url + path,
            headers=self.headers,
            timeout=0.5,  # give up after half a second, so your node never hangs
        )
        reply.raise_for_status()  # turns errors like 401 (wrong token) into an exception
        return reply.json()

    def get_route(self):
        """The current route, or None if the event has not started."""
        return self.get('/api/v1/route')['route']

    def get_status(self):
        return self.get('/api/v1/status')

    def get_route_by_id(self, route_id):
        return self.get(f'/api/v1/routes/{route_id}')
```

If anything goes wrong (no Wi-Fi, server down, wrong token), these methods raise a
`requests.RequestException`. The next step shows how to handle it.

### Step 2: ask once per second from your node

Add these lines to the `__init__` method of your existing node. The server address
and token are ROS parameters, so you don't have to write the token in your code:

```python
        self.declare_parameter('saga_url', 'http://192.168.1.100:8000')
        self.declare_parameter('saga_token', '')
        self.saga = SagaApi(
            self.get_parameter('saga_url').value,
            self.get_parameter('saga_token').value,
        )
        self.route_id = None    # the route we are working on
        self.next_dock = None   # where to drive now (None = stay still)
        self.create_timer(1.0, self.check_saga)  # call check_saga every second
```

And add this method to your node's class:

```python
    def check_saga(self):
        try:
            route = self.saga.get_route()
        except requests.RequestException as error:
            # Keep doing what you were doing and try again in one second.
            self.get_logger().warn(f'Problem talking to Saga AI: {error}')
            return

        if route is None:
            return  # the event has not started yet

        if route['route_id'] != self.route_id:
            self.route_id = route['route_id']
            self.get_logger().info(
                f'New route {route["route_id"]}: '
                f'pick up at {route["source"]["letter"]}, '
                f'deliver to {route["destination"]["letter"]}'
            )

        next_dock = route['next_dock']
        if next_dock != self.next_dock:
            self.next_dock = next_dock
            if next_dock is None:
                self.get_logger().info(f'{route["status"]}: stay still')
                # Your code: stop the truck.
            else:
                self.get_logger().info(
                    f'Drive to dock {next_dock["letter"]} (AprilTag {next_dock["tag_id"]})'
                )
                # Your code: start driving to the dock with AprilTag next_dock['tag_id'].
```

Then give the address and your token when you start your node, for example:

```bash
ros2 run <your_package> <your_node> --ros-args \
    -p saga_url:=http://192.168.1.100:8000 -p saga_token:=YOUR_TOKEN
```

The `if next_dock != self.next_dock` check means your driving code only hears about a
change once, not every second.

### Good habits

- **Once per second is enough.** The status doesn't change that often.
- **Keep the timeout short** (like the 0.5 s above). While your node waits for the
  server it can't run its other callbacks, so a long timeout could make it slow to react.
- **Never use `while True:` or `time.sleep()` in a ROS callback.** Use a timer like
  `create_timer` above instead.
- **Don't crash when the server can't be reached.** Wi-Fi can drop for a moment.
  Catch the error, like `check_saga` does, and try again next time.
- **Stop at the dock and wait.** Loading takes at least 3 seconds and the truck
  should be present at it all this time.

## Help, it doesn't work

| Problem | What to check |
|---|---|
| `401 Unauthorized` | Is the token exactly right, with no extra spaces? |
| Timeout or "connection refused" | Is the truck on the event Wi-Fi? Is the address right, including `:8000`? Can you open `http://<server address>/docs` from a laptop on the same Wi-Fi? |
| `route` is `None` | The event has not started yet. Keep asking every second. |
| `route_id` suddenly changed | Your delivery is done and you have a new job. Drive to the new `next_dock`. |
