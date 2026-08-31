import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rm_combat_interfaces.msg import NavigationTask


class NavigationAdapter(Node):
    def __init__(self) -> None:
        super().__init__('combat_navigation_adapter')
        self.declare_parameter('robot_id', 'red_robot')
        self.robot_id = str(self.get_parameter('robot_id').value)
        self.last_task_id = None
        self.pending_task = None
        self.goal_in_flight = False
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(NavigationTask, '/referee/navigation_task', self._on_task, qos)
        self.create_timer(1.0, self._try_send_pending)

    def _on_task(self, task: NavigationTask) -> None:
        if task.robot_id != self.robot_id or task.task_id == self.last_task_id:
            return
        self.pending_task = task
        self._try_send_pending()

    def _try_send_pending(self) -> None:
        if self.pending_task is None or self.goal_in_flight:
            return
        if not self.client.wait_for_server(timeout_sec=0.1):
            return
        task = self.pending_task
        goal = NavigateToPose.Goal()
        goal.pose = task.goal
        self.goal_in_flight = True
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        self.goal_in_flight = False
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().warn(f'failed to submit navigation task: {error}; retrying')
            return
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected referee navigation task; retrying')
            return
        if self.pending_task is None:
            return
        self.last_task_id = self.pending_task.task_id
        self.pending_task = None
        self.get_logger().info(f'Nav2 accepted referee navigation task {self.last_task_id}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
