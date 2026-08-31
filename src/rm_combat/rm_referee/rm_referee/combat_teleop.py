import argparse
import curses
import time

import rclpy
from rclpy.node import Node

from .combat_control import CombatController


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description='Interactive keyboard/mouse control for the combat shooter.')
    parser.add_argument('--robot', default='red_robot')
    parser.add_argument('--yaw-step', type=float, default=0.03)
    parser.add_argument('--pitch-step', type=float, default=0.02)
    parser.add_argument('--linear-speed', type=float, default=0.6)
    parser.add_argument('--angular-speed', type=float, default=1.5)
    parser.add_argument('--start-sequence', type=int, default=None)
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = Node('combat_teleop')
    controller = CombatController(node, parsed.robot)
    if not controller.wait_for_fire_service(10.0):
        node.get_logger().error('fire service is unavailable')
        node.destroy_node()
        rclpy.shutdown()
        return

    sequence = parsed.start_sequence
    if sequence is None:
        sequence = int(time.time() * 1000)

    try:
        curses.wrapper(
            _run_terminal, node, controller, parsed.yaw_step,
            parsed.pitch_step, parsed.linear_speed,
            parsed.angular_speed, sequence)
    except curses.error as error:
        node.get_logger().error(
            f'terminal UI unavailable ({error}); run this command in an interactive terminal')
    finally:
        if rclpy.ok():
            controller.stop_chassis()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _run_terminal(
        screen, node: Node, controller: CombatController,
        yaw_step: float, pitch_step: float,
        linear_speed: float, angular_speed: float, sequence: int) -> None:
    try:
        curses.curs_set(0)
    except curses.error:
        pass
    screen.keypad(True)
    screen.timeout(50)
    curses.mousemask(curses.BUTTON1_PRESSED | curses.BUTTON1_CLICKED)

    yaw = 0.0
    pitch = 0.0
    pending = []
    status = 'ready'
    yaw, pitch = controller.set_aim(yaw, pitch)

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        remaining = []
        for shot_sequence, future in pending:
            if not future.done():
                remaining.append((shot_sequence, future))
                continue
            response = future.result()
            if response is None:
                status = f'#{shot_sequence}: service error'
            else:
                status = (
                    f'#{shot_sequence}: accepted={response.accepted} '
                    f'shot_id={response.shot_id} ammo={response.remaining_ammo} '
                    f'{response.reason}')
        pending = remaining

        screen.erase()
        screen.addstr(0, 0, f'Combat teleop: /{controller.robot_id}')
        screen.addstr(1, 0, 'W/S: forward/back   A/D: strafe   Q/E: rotate')
        screen.addstr(2, 0, 'Arrows: aim   F/left mouse: fire   SPACE: stop   X: quit')
        mode = 'UNKNOWN'
        if controller.control_mode == 0:
            mode = 'AUTO (manual velocity ignored)'
        elif controller.control_mode == 1:
            mode = 'MANUAL'
        screen.addstr(3, 0, f'mode={mode} alive={controller.alive}')
        screen.addstr(4, 0, f'yaw={yaw:+.3f} rad  pitch={pitch:+.3f} rad')
        screen.addstr(5, 0, status[:max(1, curses.COLS - 1)])
        screen.refresh()

        key = screen.getch()
        aim_changed = False
        fire_requested = False
        drive_command = None
        if key in (ord('x'), ord('X')):
            break
        if key == curses.KEY_LEFT:
            yaw += abs(yaw_step)
            aim_changed = True
        elif key == curses.KEY_RIGHT:
            yaw -= abs(yaw_step)
            aim_changed = True
        elif key == curses.KEY_UP:
            pitch += abs(pitch_step)
            aim_changed = True
        elif key == curses.KEY_DOWN:
            pitch -= abs(pitch_step)
            aim_changed = True
        elif key in (ord('w'), ord('W')):
            drive_command = (abs(linear_speed), 0.0, 0.0)
        elif key in (ord('s'), ord('S')):
            drive_command = (-abs(linear_speed), 0.0, 0.0)
        elif key in (ord('a'), ord('A')):
            drive_command = (0.0, abs(linear_speed), 0.0)
        elif key in (ord('d'), ord('D')):
            drive_command = (0.0, -abs(linear_speed), 0.0)
        elif key in (ord('q'), ord('Q')):
            drive_command = (0.0, 0.0, abs(angular_speed))
        elif key in (ord('e'), ord('E')):
            drive_command = (0.0, 0.0, -abs(angular_speed))
        elif key == ord(' '):
            drive_command = (0.0, 0.0, 0.0)
            status = 'manual emergency stop'
        elif key in (ord('f'), ord('F')):
            fire_requested = True
        elif key == curses.KEY_MOUSE:
            try:
                _, _, _, _, button_state = curses.getmouse()
                fire_requested = bool(
                    button_state & (curses.BUTTON1_PRESSED | curses.BUTTON1_CLICKED))
            except curses.error:
                pass

        if aim_changed:
            yaw, pitch = controller.set_aim(yaw, pitch)
        if drive_command is not None:
            controller.drive(*drive_command)
        if fire_requested:
            pending.append((sequence, controller.fire(sequence)))
            status = f'#{sequence}: requested'
            sequence += 1


if __name__ == '__main__':
    main()
