import argparse
import time

import rclpy
from rclpy.node import Node

from .combat_control import CombatController


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description='Fire logical 17 mm projectiles through the referee.')
    parser.add_argument('--robot', default='red_robot')
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--pitch', type=float, default=0.0)
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument('--rate', type=float, default=5.0)
    parser.add_argument('--start-sequence', type=int, default=1)
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = Node('combat_shoot_cli')
    controller = CombatController(node, parsed.robot)
    if not controller.wait_for_fire_service(timeout_sec=10.0):
        node.get_logger().error('fire service is unavailable')
        node.destroy_node()
        rclpy.shutdown()
        return

    controller.set_aim(parsed.yaw, parsed.pitch)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.15)

    interval = 1.0 / max(0.01, min(parsed.rate, 5.0))
    for index in range(max(0, parsed.count)):
        client_sequence = parsed.start_sequence + index
        future = controller.fire(client_sequence)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        response = future.result()
        if response is None:
            print(f'#{client_sequence}: timeout')
        else:
            print(
                f'#{client_sequence}: accepted={response.accepted} '
                f'shot_id={response.shot_id} ammo={response.remaining_ammo} '
                f'reason={response.reason}')
        if index + 1 < parsed.count:
            time.sleep(interval + 0.01)

    node.destroy_node()
    rclpy.shutdown()
