import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rm_combat_interfaces.msg import (
    DamageEvent, HitEvent, MatchState, RobotStatus, ShotResult,
)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description='Print combat state and referee events.')
    parser.add_argument('--duration', type=float, default=0.0, help='seconds; 0 keeps running')
    parser.add_argument('--red-robot', default='red_robot')
    parser.add_argument('--blue-robot', default='blue_robot')
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Node('combat_monitor')
    last = {}

    def print_changed(key, value, text):
        if last.get(key) == value:
            return
        last[key] = value
        print(text, flush=True)

    state_qos = QoSProfile(depth=1)
    state_qos.reliability = ReliabilityPolicy.RELIABLE
    state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    node.create_subscription(
        RobotStatus, f'/{parsed.red_robot}/combat/status',
        lambda msg: print_changed(
            'red',
            (msg.control_mode, msg.hp, msg.max_hp, msg.damage_per_shot,
             msg.remaining_ammo, int(msg.buff_remaining)),
            f'RED id={msg.robot_id} mode={msg.control_mode} '
            f'hp={msg.hp}/{msg.max_hp} '
            f'damage={msg.damage_per_shot} ammo={msg.remaining_ammo} '
            f'buff={msg.buff_remaining:.1f}s'), state_qos)
    node.create_subscription(
        RobotStatus, f'/{parsed.blue_robot}/combat/status',
        lambda msg: print_changed(
            'blue',
            (msg.control_mode, msg.hp, msg.max_hp, msg.damage_per_shot,
             msg.remaining_ammo, int(msg.buff_remaining)),
            f'BLUE id={msg.robot_id} mode={msg.control_mode} '
            f'hp={msg.hp}/{msg.max_hp} damage={msg.damage_per_shot} '
            f'ammo={msg.remaining_ammo} alive={msg.alive}'), state_qos)
    node.create_subscription(
        MatchState, '/referee/match_state',
        lambda msg: print_changed(
            'match',
            (msg.phase, msg.red_score, msg.blue_score, int(msg.remaining_time),
             msg.winner, msg.reason),
            f'MATCH phase={msg.phase} score={msg.red_score}:{msg.blue_score} '
            f'remaining={msg.remaining_time:.1f}s winner={msg.winner or "-"} '
            f'reason={msg.reason or "-"}'), state_qos)
    node.create_subscription(
        ShotResult, '/referee/shot_results',
        lambda msg: print(
            f'SHOT id={msg.shot_id} outcome={msg.outcome} distance={msg.distance:.2f} '
            f'collision={msg.collision_name or "-"}', flush=True), 20)
    node.create_subscription(
        HitEvent, '/referee/hit_events',
        lambda msg: print(
            f'HIT id={msg.shot_id} {msg.attacker_id}->{msg.target_id} '
            f'armor={msg.armor_id} damage={msg.damage} hp={msg.remaining_hp}', flush=True), 20)
    node.create_subscription(
        DamageEvent, '/referee/damage_events',
        lambda msg: print(
            f'DAMAGE id={msg.event_id} source={msg.source} '
            f'{msg.attacker_id or "environment"}->{msg.target_id} '
            f'damage={msg.damage} hp={msg.remaining_hp} '
            f'speed={msg.relative_speed:.2f}', flush=True), 20)

    deadline = None
    if parsed.duration > 0.0:
        deadline = node.get_clock().now().nanoseconds + int(parsed.duration * 1e9)
    try:
        while rclpy.ok() and (deadline is None or node.get_clock().now().nanoseconds < deadline):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
