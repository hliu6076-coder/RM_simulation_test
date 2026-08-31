from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rm_combat_interfaces.msg import MatchState, RobotStatus


class CombatCmdVelMux(Node):
    """Select Nav2 or manual velocity without allowing two writers at the chassis."""

    def __init__(self) -> None:
        super().__init__('combat_cmd_vel_mux')
        self.declare_parameter('robot_id', 'red_robot')
        self.robot_id = str(self.get_parameter('robot_id').value).strip('/')
        self.declare_parameter('auto_topic', '/cmd_vel')
        self.declare_parameter('manual_topic', f'/{self.robot_id}/cmd_vel_manual')
        self.declare_parameter('output_topic', '/cmd_vel_selected')
        self.declare_parameter('cancel_navigation', True)
        self.declare_parameter('cancel_action', '/navigate_to_pose/_action/cancel_goal')
        self.declare_parameter('manual_timeout', 0.25)
        self.declare_parameter('auto_timeout', 0.5)

        self.manual_timeout = float(self.get_parameter('manual_timeout').value)
        self.auto_timeout = float(self.get_parameter('auto_timeout').value)
        self.control_mode = RobotStatus.AUTO
        self.alive = True
        self.match_phase = None
        self.auto_command = Twist()
        self.manual_command = Twist()
        self.auto_stamp = None
        self.manual_stamp = None
        self.cancel_navigation_pending = False

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.output = self.create_publisher(
            Twist, str(self.get_parameter('output_topic').value), 10)
        self.create_subscription(
            Twist, str(self.get_parameter('auto_topic').value),
            self._on_auto_command, 10)
        self.create_subscription(
            Twist, str(self.get_parameter('manual_topic').value),
            self._on_manual_command, 10)
        self.create_subscription(
            RobotStatus, f'/{self.robot_id}/combat/status',
            self._on_robot_status, state_qos)
        self.create_subscription(
            MatchState, '/referee/match_state', self._on_match_state, state_qos)
        self.cancel_navigation = None
        if bool(self.get_parameter('cancel_navigation').value):
            self.cancel_navigation = self.create_client(
                CancelGoal, str(self.get_parameter('cancel_action').value))
        self.timer = self.create_timer(0.05, self._publish_selected)
        self.get_logger().info(
            'velocity mux ready: '
            f'AUTO={self.get_parameter("auto_topic").value}, '
            f'MANUAL={self.get_parameter("manual_topic").value}, '
            f'output={self.get_parameter("output_topic").value}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_auto_command(self, command: Twist) -> None:
        self.auto_command = command
        self.auto_stamp = self._now()

    def _on_manual_command(self, command: Twist) -> None:
        self.manual_command = command
        self.manual_stamp = self._now()

    def _on_robot_status(self, status: RobotStatus) -> None:
        if status.robot_id != self.robot_id:
            return
        new_mode = int(status.control_mode)
        self.alive = bool(status.alive)
        if new_mode == self.control_mode:
            return
        self.control_mode = new_mode
        self.auto_stamp = None
        self.manual_stamp = None
        self.output.publish(Twist())
        if new_mode == RobotStatus.MANUAL:
            self.cancel_navigation_pending = True
            self.get_logger().info('control mode changed to MANUAL; stopping Nav2 command')
        else:
            self.get_logger().info('control mode changed to AUTO; ignoring manual command')

    def _on_match_state(self, state: MatchState) -> None:
        self.match_phase = int(state.phase)
        if self.match_phase != MatchState.RUNNING:
            self.output.publish(Twist())

    def _publish_selected(self) -> None:
        if self.cancel_navigation_pending and self.cancel_navigation is None:
            self.cancel_navigation_pending = False
        if self.cancel_navigation_pending and self.cancel_navigation.service_is_ready():
            self.cancel_navigation.call_async(CancelGoal.Request())
            self.cancel_navigation_pending = False

        if not self.alive:
            self.output.publish(Twist())
            return
        if self.match_phase is not None and self.match_phase != MatchState.RUNNING:
            self.output.publish(Twist())
            return

        now = self._now()
        if self.control_mode == RobotStatus.MANUAL:
            command = self.manual_command
            stamp = self.manual_stamp
            timeout = self.manual_timeout
        else:
            command = self.auto_command
            stamp = self.auto_stamp
            timeout = self.auto_timeout
        if stamp is None or now - stamp > timeout:
            command = Twist()
        self.output.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CombatCmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
