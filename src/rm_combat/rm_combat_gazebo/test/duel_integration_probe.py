#!/usr/bin/env python3
import math
import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import (
    DeleteEntity, GetEntityState, SetEntityState, SpawnEntity,
)
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rm_combat_interfaces.msg import DamageEvent, RobotStatus
from rm_combat_interfaces.srv import Fire
from std_srvs.srv import Trigger


class Probe(Node):
    def __init__(self):
        super().__init__('duel_integration_probe')
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status = {}
        self.damage = []
        self.create_subscription(
            RobotStatus, '/red_robot/combat/status',
            lambda message: self.status.__setitem__('red_robot', message), qos)
        self.create_subscription(
            RobotStatus, '/blue_robot/combat/status',
            lambda message: self.status.__setitem__('blue_robot', message), qos)
        self.create_subscription(
            DamageEvent, '/referee/damage_events', self.damage.append, 20)
        self.red_pub = self.create_publisher(Twist, '/red_robot/cmd_vel_manual', 10)
        self.blue_pub = self.create_publisher(Twist, '/blue_robot/cmd_vel_manual', 10)
        self.start_client = self.create_client(Trigger, '/referee/start_match')
        self.reset_client = self.create_client(Trigger, '/referee/reset_match')
        self.red_fire = self.create_client(Fire, '/red_robot/combat/fire')
        self.set_state = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.get_state = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete = self.create_client(DeleteEntity, '/delete_entity')

    def wait(self, predicate, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service unavailable: {client.srv_name}')
        future = client.call_async(request)
        if not self.wait(future.done, timeout):
            raise RuntimeError(f'service timeout: {client.srv_name}')
        return future.result()

    def spin_for(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose_robot(self, name, x, y, yaw, z=0.05):
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = name
        request.state.reference_frame = 'world'
        request.state.pose.position.x = x
        request.state.pose.position.y = y
        request.state.pose.position.z = z
        request.state.pose.orientation.z = math.sin(yaw / 2.0)
        request.state.pose.orientation.w = math.cos(yaw / 2.0)
        response = self.call(self.set_state, request)
        if not response.success:
            raise RuntimeError(f'failed to position {name}')

    def wait_for_entity(self, name, timeout=20.0):
        if not self.get_state.wait_for_service(timeout_sec=timeout):
            return False
        deadline = time.monotonic() + timeout
        request = GetEntityState.Request()
        request.name = name
        request.reference_frame = 'world'
        while time.monotonic() < deadline:
            future = self.get_state.call_async(request)
            remaining = max(0.1, deadline - time.monotonic())
            if self.wait(future.done, min(2.0, remaining)):
                response = future.result()
                if response is not None and response.success:
                    return True
            self.spin_for(0.1)
        return False

    def robot_x(self, name):
        request = GetEntityState.Request()
        request.name = name
        request.reference_frame = 'world'
        response = self.call(self.get_state, request)
        if not response.success:
            raise RuntimeError(f'failed to read {name}')
        return response.state.pose.position.x

    def drive(self, red_x=0.0, blue_x=0.0, duration=1.0):
        red = Twist()
        blue = Twist()
        red.linear.x = red_x
        blue.linear.x = blue_x
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.red_pub.publish(red)
            self.blue_pub.publish(blue)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.red_pub.publish(Twist())
        self.blue_pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = Probe()
    try:
        if not node.wait(lambda: len(node.status) == 2, 15.0):
            raise RuntimeError('robot statuses unavailable')
        print(
            f'initial red={node.status["red_robot"].hp} '
            f'blue={node.status["blue_robot"].hp}')

        if not node.wait_for_entity('red_robot') or \
                not node.wait_for_entity('blue_robot'):
            raise RuntimeError('Gazebo robot entities unavailable')

        reset_before_start = node.call(node.reset_client, Trigger.Request())
        if not reset_before_start.success:
            raise RuntimeError(reset_before_start.message)
        node.pose_robot('red_robot', 0.0, -10.0, 0.0)
        node.pose_robot('blue_robot', 5.0, -15.0, 0.0)
        node.spin_for(0.3)
        preparing_x = node.robot_x('red_robot')
        node.drive(red_x=0.8, duration=0.6)
        if abs(node.robot_x('red_robot') - preparing_x) > 0.05:
            raise RuntimeError('robot moved while match was PREPARING')
        rejected = node.call(node.red_fire, Fire.Request(client_sequence=99))
        if rejected.accepted:
            raise RuntimeError('fire accepted while match was PREPARING')
        print('preparing movement=blocked fire=blocked')
        start = node.call(node.start_client, Trigger.Request())
        if not start.success:
            raise RuntimeError(start.message)

        # Run deterministic combat checks on the unobstructed ground outside
        # the arena mesh; the launch-spawn locations remain the RMUL births.
        node.pose_robot('red_robot', 0.0, -10.0, 0.0)
        node.pose_robot('blue_robot', 1.0, -10.0, math.pi)
        node.spin_for(0.5)
        fire_request = Fire.Request()
        fire_request.client_sequence = 100
        fire = node.call(node.red_fire, fire_request)
        if not fire.accepted:
            raise RuntimeError(f'fire rejected: {fire.reason}')
        if not node.wait(
                lambda: any(event.source == DamageEvent.PROJECTILE for event in node.damage),
                5.0):
            raise RuntimeError('projectile damage event not received')
        projectile = next(
            event for event in node.damage if event.source == DamageEvent.PROJECTILE)
        print(
            f'projectile target={projectile.target_id} damage={projectile.damage} '
            f'hp={projectile.remaining_hp}')

        node.pose_robot('red_robot', 0.0, -10.0, 0.0)
        node.pose_robot('blue_robot', 0.8, -10.0, math.pi)
        node.drive(red_x=0.1, blue_x=0.1, duration=3.0)
        if any(
                event.source == DamageEvent.ROBOT_COLLISION
                for event in node.damage):
            raise RuntimeError('low-speed robot contact caused damage')
        node.pose_robot('red_robot', 0.0, -10.0, 0.0)
        node.pose_robot('blue_robot', 0.8, -10.0, math.pi)
        node.spin_for(0.5)
        node.drive(red_x=0.8, blue_x=0.8, duration=1.0)
        if not node.wait(
                lambda: len([
                    event for event in node.damage
                    if event.source == DamageEvent.ROBOT_COLLISION]) >= 2,
                5.0):
            raise RuntimeError('symmetric robot collision damage not received')
        crash = [
            event for event in node.damage
            if event.source == DamageEvent.ROBOT_COLLISION]
        print(
            f'robot_collision events={len(crash)} '
            f'targets={[event.target_id for event in crash[-2:]]}')

        node.pose_robot('red_robot', 0.0, -10.0, 0.0)
        node.pose_robot('blue_robot', 5.0, -15.0, 0.0)
        spawn = SpawnEntity.Request()
        spawn.name = 'duel_test_obstacle'
        spawn.reference_frame = 'world'
        spawn.initial_pose.position.x = 0.8
        spawn.initial_pose.position.y = -10.0
        # Use a tall wall so the contact normal is unambiguously horizontal
        # with both the lightweight chassis and the restored wheeled chassis.
        spawn.initial_pose.position.z = 1.0
        spawn.initial_pose.orientation.w = 1.0
        spawn.xml = '''<sdf version="1.6"><model name="duel_test_obstacle">
          <static>true</static><link name="link">
          <collision name="collision"><geometry><box>
          <size>0.2 1.0 2.0</size></box></geometry></collision>
          <visual name="visual"><geometry><box>
          <size>0.2 1.0 2.0</size></box></geometry></visual>
          </link></model></sdf>'''
        spawn_response = node.call(node.spawn, spawn)
        if not spawn_response.success:
            raise RuntimeError(f'obstacle spawn failed: {spawn_response.status_message}')
        previous = len([
            event for event in node.damage
            if event.source == DamageEvent.OBSTACLE_COLLISION])
        node.drive(red_x=0.8, duration=2.0)
        if not node.wait(
                lambda: len([
                    event for event in node.damage
                    if event.source == DamageEvent.OBSTACLE_COLLISION]) > previous,
                5.0):
            raise RuntimeError('obstacle collision damage not received')
        obstacle = [
            event for event in node.damage
            if event.source == DamageEvent.OBSTACLE_COLLISION][-1]
        print(
            f'obstacle target={obstacle.target_id} damage={obstacle.damage} '
            f'speed={obstacle.relative_speed:.2f}')
        node.call(node.delete, DeleteEntity.Request(name='duel_test_obstacle'))

        pause = node.call(
            node.create_client(Trigger, '/referee/pause_match'), Trigger.Request())
        if not pause.success:
            raise RuntimeError(pause.message)
        node.pose_robot('red_robot', 0.0, -10.0, 0.0)
        node.spin_for(0.3)
        paused_x = node.robot_x('red_robot')
        node.drive(red_x=0.8, duration=0.6)
        if abs(node.robot_x('red_robot') - paused_x) > 0.05:
            raise RuntimeError('robot moved while match was PAUSED')
        print('paused movement=blocked')

        reset = node.call(node.reset_client, Trigger.Request())
        if not reset.success:
            raise RuntimeError(reset.message)
        if not node.wait(
                lambda: node.status['red_robot'].hp == 500
                and node.status['blue_robot'].hp == 500, 3.0):
            raise RuntimeError('reset did not restore both robots')
        print('reset red=500 blue=500')
    finally:
        node.red_pub.publish(Twist())
        node.blue_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
