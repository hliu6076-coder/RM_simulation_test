import math
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rm_combat_interfaces.msg import GimbalCommand, RobotStatus
from rm_combat_interfaces.srv import Fire


MAX_PITCH = 0.35


def clamp_aim(yaw: float, pitch: float) -> tuple[float, float]:
    """Clamp an absolute aim command to the logical gimbal limits."""
    return (
        max(-math.pi, min(math.pi, float(yaw))),
        max(-MAX_PITCH, min(MAX_PITCH, float(pitch))),
    )


class CombatController:
    """ROS transport used by terminal, mouse and future external controllers."""

    def __init__(self, node: Node, robot_id: str = 'red_robot') -> None:
        self.node = node
        self.robot_id = robot_id.strip('/')
        self.control_mode = None
        self.alive = True
        self._gimbal_pub = node.create_publisher(
            GimbalCommand, f'/{self.robot_id}/combat/gimbal_cmd', 10)
        self._manual_velocity_pub = node.create_publisher(
            Twist, f'/{self.robot_id}/cmd_vel_manual', 10)
        self._fire_client = node.create_client(
            Fire, f'/{self.robot_id}/combat/fire')
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_sub = node.create_subscription(
            RobotStatus, f'/{self.robot_id}/combat/status',
            self._on_status, state_qos)

    def _on_status(self, status: RobotStatus) -> None:
        self.control_mode = int(status.control_mode)
        self.alive = bool(status.alive)

    def wait_for_fire_service(self, timeout_sec: float = 10.0) -> bool:
        return self._fire_client.wait_for_service(timeout_sec=timeout_sec)

    def set_aim(self, yaw: float, pitch: float) -> tuple[float, float]:
        yaw, pitch = clamp_aim(yaw, pitch)
        command = GimbalCommand()
        command.header.stamp = self.node.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.yaw = yaw
        command.pitch = pitch
        self._gimbal_pub.publish(command)
        return yaw, pitch

    def fire(self, client_sequence: int):
        request = Fire.Request()
        request.client_sequence = int(client_sequence)
        return self._fire_client.call_async(request)

    def drive(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        command = Twist()
        command.linear.x = float(linear_x)
        command.linear.y = float(linear_y)
        command.angular.z = float(angular_z)
        self._manual_velocity_pub.publish(command)

    def stop_chassis(self) -> None:
        self.drive(0.0, 0.0, 0.0)
