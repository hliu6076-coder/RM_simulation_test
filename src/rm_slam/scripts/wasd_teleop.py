#!/usr/bin/env python3

import argparse
import curses
import time

import rclpy
from geometry_msgs.msg import Twist


def main(args=None):
    parser = argparse.ArgumentParser(description='WASD mecanum chassis teleop')
    parser.add_argument('--topic', default='/cmd_vel_chassis')
    parser.add_argument('--linear-speed', type=float, default=0.2)
    parser.add_argument('--angular-speed', type=float, default=0.35)
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = rclpy.create_node('wasd_teleop')
    publisher = node.create_publisher(Twist, parsed.topic, 10)

    def run(screen):
        screen.timeout(50)
        screen.keypad(True)
        command = Twist()
        last_key_time = 0.0

        while rclpy.ok():
            screen.erase()
            screen.addstr(0, 0, 'W/S forward  A/D strafe  Q/E rotate')
            screen.addstr(1, 0, 'SPACE stop   X quit   (hold a key to keep moving)')
            screen.refresh()

            key = screen.getch()
            if key in (ord('x'), ord('X')):
                break

            next_command = None
            if key in (ord('w'), ord('W')):
                next_command = (parsed.linear_speed, 0.0, 0.0)
            elif key in (ord('s'), ord('S')):
                next_command = (-parsed.linear_speed, 0.0, 0.0)
            elif key in (ord('a'), ord('A')):
                next_command = (0.0, parsed.linear_speed, 0.0)
            elif key in (ord('d'), ord('D')):
                next_command = (0.0, -parsed.linear_speed, 0.0)
            elif key in (ord('q'), ord('Q')):
                next_command = (0.0, 0.0, parsed.angular_speed)
            elif key in (ord('e'), ord('E')):
                next_command = (0.0, 0.0, -parsed.angular_speed)
            elif key == ord(' '):
                next_command = (0.0, 0.0, 0.0)

            if next_command is not None:
                command = Twist()
                command.linear.x, command.linear.y, command.angular.z = next_command
                last_key_time = time.monotonic()
            elif time.monotonic() - last_key_time > 0.25:
                command = Twist()

            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.0)

    try:
        curses.wrapper(run)
    finally:
        publisher.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
