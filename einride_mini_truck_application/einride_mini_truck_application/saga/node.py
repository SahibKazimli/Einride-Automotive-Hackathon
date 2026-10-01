"""Polls Saga AI once per second and publishes the dock to drive to.

Publishes
    /saga/next_tag (std_msgs/Int32): AprilTag id of the dock to drive to,
        -1 = stay still. Republished on every successful poll, latched.
    /saga/route_id (std_msgs/Int32): current route id, -1 = none.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import requests
from std_msgs.msg import Int32

from .client import parse_route_reply, RouteState, SagaApi


class SagaNode(Node):
    """Bridges the Saga AI HTTP API to ROS topics."""

    def __init__(self) -> None:
        super().__init__('saga')
        self.declare_parameter('saga_url', 'http://127.0.0.1:8000')
        self.declare_parameter('saga_token', '')
        self.declare_parameter('poll_period', 1.0)
        token = self.get_parameter('saga_token').value
        if not token:
            self.get_logger().warn('saga_token is empty; calls will get 401')
        self.saga = SagaApi(self.get_parameter('saga_url').value, token)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.next_tag_pub = self.create_publisher(Int32, '/saga/next_tag', latched)
        self.route_pub = self.create_publisher(Int32, '/saga/route_id', latched)
        self.last: RouteState | None = None
        self.create_timer(self.get_parameter('poll_period').value, self.check_saga)

    def check_saga(self) -> None:
        try:
            state = parse_route_reply(self.saga.get_route_reply())
        except (requests.RequestException, ValueError, KeyError, TypeError) as error:
            # Keep the last published values; the mission keeps doing its thing.
            self.get_logger().warn(f'Problem talking to Saga AI: {error}',
                                   throttle_duration_sec=5.0)
            return

        last = self.last
        if last is None or state.route_id != last.route_id:
            if state.route_id is None:
                self.get_logger().info(f'No route yet (event {state.event_state})')
            else:
                self.get_logger().info(
                    f'New route {state.route_id}: pick up at {state.source_letter}, '
                    f'deliver to {state.destination_letter}')
        if last is None or state.next_tag != last.next_tag:
            if state.next_tag is None:
                self.get_logger().info(f'{state.status}: stay still')
            else:
                self.get_logger().info(
                    f'Drive to dock {state.next_letter} (AprilTag {state.next_tag})')
        self.last = state

        self.next_tag_pub.publish(Int32(data=-1 if state.next_tag is None else state.next_tag))
        self.route_pub.publish(Int32(data=-1 if state.route_id is None else state.route_id))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SagaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
