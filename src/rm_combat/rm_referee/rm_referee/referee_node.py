import math
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rm_combat_interfaces.msg import (
    AuthorizedShot,
    CollisionReport,
    DamageEvent,
    GimbalCommand,
    HitEvent,
    MatchState,
    NavigationTask,
    RobotStatus,
    ShotResult,
)
from rm_combat_interfaces.srv import Fire, SetControlMode
from std_srvs.srv import Trigger

from .rules import AUTO, FINISHED, MANUAL, CombatRules, RulesConfig


class RefereeNode(Node):
    def __init__(self) -> None:
        super().__init__('referee_node')
        self._declare_parameters()
        self.red_robot_id = str(self.get_parameter('red_robot_id').value)
        self.blue_robot_id = str(self.get_parameter('blue_robot_id').value)
        self.robot_ids = (self.red_robot_id, self.blue_robot_id)
        self.model_to_robot = {
            str(self.get_parameter('red_model_name').value): self.red_robot_id,
            str(self.get_parameter('blue_model_name').value): self.blue_robot_id,
        }
        config = RulesConfig(
            auto_hp=int(self.get_parameter('auto_hp').value),
            manual_hp=int(self.get_parameter('manual_hp').value),
            auto_damage=int(self.get_parameter('auto_damage').value),
            manual_damage=int(self.get_parameter('manual_damage').value),
            initial_ammo=int(self.get_parameter('initial_ammo').value),
            fire_rate=float(self.get_parameter('fire_rate').value),
            max_range=float(self.get_parameter('max_range').value),
            match_duration=float(self.get_parameter('match_duration').value),
            obstacle_damage=int(self.get_parameter('obstacle_damage').value),
            robot_collision_damage=int(self.get_parameter('robot_collision_damage').value),
            min_collision_speed=float(self.get_parameter('min_collision_speed').value),
            checkpoint_score=int(self.get_parameter('checkpoint_score').value),
            buff_damage=int(self.get_parameter('buff_damage').value),
            buff_duration=float(self.get_parameter('buff_duration').value),
        )
        self.rules = CombatRules(
            config,
            red_robot_id=self.red_robot_id,
            blue_robot_id=self.blue_robot_id,
            default_mode=int(self.get_parameter('default_mode').value),
        )
        self.gimbal: Dict[str, Tuple[float, float]] = {
            robot_id: (0.0, 0.0) for robot_id in self.robot_ids
        }
        self.enable_checkpoint = bool(self.get_parameter('enable_checkpoint').value)
        self.checkpoint_robot_id = str(self.get_parameter('checkpoint_robot_id').value)
        self.checkpoint_position: Optional[Tuple[float, float]] = None
        self.checkpoint_entered_at: Optional[float] = None
        self.task_published = False

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        event_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.authorized_pub = self.create_publisher(
            AuthorizedShot, '/referee/internal/authorized_shots', event_qos)
        self.hit_pub = self.create_publisher(HitEvent, '/referee/hit_events', event_qos)
        self.damage_pub = self.create_publisher(
            DamageEvent, '/referee/damage_events', event_qos)
        self.match_pub = self.create_publisher(MatchState, '/referee/match_state', state_qos)
        self.status_pubs = {
            robot_id: self.create_publisher(
                RobotStatus, f'/{robot_id}/combat/status', state_qos)
            for robot_id in self.robot_ids
        }
        self.task_pub = self.create_publisher(
            NavigationTask, '/referee/navigation_task', state_qos)

        # Keep explicit references without shadowing rclpy.Node's internal
        # _subscriptions/_services collections. Shadowing those attributes
        # makes destroy_node() try to remove the same entities twice.
        self.subscription_handles = []
        self.service_handles = []
        for robot_id in self.robot_ids:
            self.subscription_handles.append(self.create_subscription(
                GimbalCommand,
                f'/{robot_id}/combat/gimbal_cmd',
                lambda message, rid=robot_id: self._on_gimbal(rid, message),
                event_qos))
            self.service_handles.append(self.create_service(
                Fire,
                f'/{robot_id}/combat/fire',
                lambda request, response, rid=robot_id: self._on_fire(
                    rid, request, response)))
        self.subscription_handles.extend([
            self.create_subscription(
                ShotResult, '/referee/shot_results', self._on_shot_result, event_qos),
            self.create_subscription(
                CollisionReport, '/referee/internal/collision_reports',
                self._on_collision_report, event_qos),
            self.create_subscription(
                PoseStamped, '/referee/internal/model_pose',
                self._on_ground_truth_pose, 10),
        ])
        self.service_handles.extend([
            self.create_service(
                SetControlMode, '/referee/set_control_mode', self._on_set_mode),
            self.create_service(Trigger, '/referee/start_match', self._on_start),
            self.create_service(Trigger, '/referee/pause_match', self._on_pause),
            self.create_service(Trigger, '/referee/resume_match', self._on_resume),
            self.create_service(Trigger, '/referee/reset_match', self._on_reset),
        ])

        self.timer = self.create_timer(0.1, self._on_timer)
        self.autostart_timer = None
        if bool(self.get_parameter('autostart').value):
            self.autostart_timer = self.create_timer(0.5, self._autostart_once)
        self._publish_state()
        self.get_logger().info(
            f'referee ready: red={self.red_robot_id}, blue={self.blue_robot_id}, '
            f'checkpoint={self.enable_checkpoint}')

    def _declare_parameters(self) -> None:
        defaults = {
            'red_robot_id': 'red_robot',
            'blue_robot_id': 'blue_target',
            'red_model_name': 'robot',
            'blue_model_name': 'blue_target',
            'default_mode': MANUAL,
            'auto_hp': 500,
            'manual_hp': 500,
            'auto_damage': 20,
            'manual_damage': 20,
            'initial_ammo': 50,
            'fire_rate': 5.0,
            'max_range': 10.0,
            'match_duration': 120.0,
            'obstacle_damage': 10,
            'robot_collision_damage': 20,
            'min_collision_speed': 0.5,
            'autostart': True,
            'enable_checkpoint': False,
            'checkpoint_robot_id': 'red_robot',
            'checkpoint_x': 0.5,
            'checkpoint_y': 0.0,
            'checkpoint_world_x': 4.8,
            'checkpoint_world_y': 3.35,
            'checkpoint_yaw': 0.0,
            'checkpoint_radius': 0.35,
            'checkpoint_dwell': 2.0,
            'checkpoint_score': 50,
            'buff_damage': 30,
            'buff_duration': 15.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _autostart_once(self) -> None:
        if self.autostart_timer is not None:
            self.autostart_timer.cancel()
        success, message = self.rules.start(self._now_seconds())
        if success:
            self.get_logger().info(message)
            self._publish_navigation_task()
            self._publish_state()

    def _on_gimbal(self, robot_id: str, message: GimbalCommand) -> None:
        self.gimbal[robot_id] = (
            max(-math.pi, min(math.pi, float(message.yaw))),
            max(-0.35, min(0.35, float(message.pitch))),
        )

    def _on_fire(
        self, robot_id: str, request: Fire.Request, response: Fire.Response
    ) -> Fire.Response:
        decision = self.rules.request_fire(
            robot_id, int(request.client_sequence), self._now_seconds())
        response.accepted = decision.accepted
        response.reason = decision.reason
        response.shot_id = decision.shot_id
        response.remaining_ammo = decision.remaining_ammo
        if decision.accepted:
            yaw, pitch = self.gimbal[robot_id]
            shot = AuthorizedShot()
            shot.header.stamp = self.get_clock().now().to_msg()
            shot.header.frame_id = f'{robot_id}/base_link'
            shot.shot_id = decision.shot_id
            shot.shooter_id = robot_id
            shot.yaw = yaw
            shot.pitch = pitch
            shot.max_range = self.rules.config.max_range
            self.authorized_pub.publish(shot)
        self._publish_state()
        return response

    def _on_shot_result(self, result: ShotResult) -> None:
        shooter_id = self.model_to_robot.get(result.shooter_id, result.shooter_id)
        target_id = self.model_to_robot.get(result.target_id, result.target_id)
        resolved = self.rules.resolve_shot(
            int(result.shot_id), shooter_id, target_id,
            result.outcome == ShotResult.HIT)
        if resolved is None:
            self._publish_state()
            return
        self._publish_damage(resolved, result.header, result.collision_name)
        event = HitEvent()
        event.header = result.header
        event.shot_id = result.shot_id
        event.attacker_id = shooter_id
        event.target_id = target_id
        event.armor_id = 'body'
        event.damage = resolved.damage
        event.remaining_hp = resolved.remaining_hp
        self.hit_pub.publish(event)
        self._log_finished()
        self._publish_state()

    def _on_collision_report(self, report: CollisionReport) -> None:
        robot_a = self.model_to_robot.get(report.model_a)
        robot_b = self.model_to_robot.get(report.model_b)
        if robot_a is not None and robot_b is not None:
            resolutions = self.rules.apply_robot_collision(
                int(report.event_id), robot_a, robot_b, float(report.relative_speed))
        elif robot_a is not None:
            resolutions = self.rules.apply_obstacle_collision(
                int(report.event_id), robot_a, float(report.relative_speed))
        elif robot_b is not None:
            resolutions = self.rules.apply_obstacle_collision(
                int(report.event_id), robot_b, float(report.relative_speed))
        else:
            return
        collision_name = f'{report.collision_a}|{report.collision_b}'
        for resolution in resolutions:
            self._publish_damage(resolution, report.header, collision_name)
        if resolutions:
            self._log_finished()
            self._publish_state()

    def _publish_damage(self, resolution, header, collision_name: str) -> None:
        event = DamageEvent()
        event.header = header
        event.event_id = resolution.event_id
        event.source = resolution.source
        event.attacker_id = resolution.attacker_id
        event.target_id = resolution.target_id
        event.damage = resolution.damage
        event.remaining_hp = resolution.remaining_hp
        event.collision_name = collision_name
        event.relative_speed = resolution.relative_speed
        self.damage_pub.publish(event)

    def _log_finished(self) -> None:
        if self.rules.phase == FINISHED:
            self.get_logger().info(
                f'match finished: winner={self.rules.winner}, '
                f'reason={self.rules.finish_reason}')

    def _on_set_mode(
        self, request: SetControlMode.Request, response: SetControlMode.Response
    ) -> SetControlMode.Response:
        response.success, response.message = self.rules.set_mode(
            request.robot_id, int(request.mode))
        self._publish_state()
        return response

    def _on_start(self, _request, response):
        response.success, response.message = self.rules.start(self._now_seconds())
        if response.success:
            self._publish_navigation_task()
        self._publish_state()
        return response

    def _on_pause(self, _request, response):
        response.success, response.message = self.rules.pause(self._now_seconds())
        self.checkpoint_entered_at = None
        self._publish_state()
        return response

    def _on_resume(self, _request, response):
        response.success, response.message = self.rules.resume(self._now_seconds())
        self.checkpoint_entered_at = None
        self._publish_state()
        return response

    def _on_reset(self, _request, response):
        self.rules.reset()
        self.checkpoint_entered_at = None
        self.task_published = False
        response.success = True
        response.message = 'match reset; control modes may be changed'
        self._publish_state()
        return response

    def _on_ground_truth_pose(self, pose: PoseStamped) -> None:
        if self.enable_checkpoint:
            self.checkpoint_position = (pose.pose.position.x, pose.pose.position.y)

    def _on_timer(self) -> None:
        now = self._now_seconds()
        if self.rules.update_timeout(now):
            self._log_finished()
        self._update_checkpoint(now)
        self._publish_state()

    def _update_checkpoint(self, now: float) -> None:
        participant = self.rules.participants.get(self.checkpoint_robot_id)
        if not self.enable_checkpoint or participant is None \
                or self.rules.phase != MatchState.RUNNING \
                or participant.checkpoint_completed \
                or self.checkpoint_position is None:
            self.checkpoint_entered_at = None
            return
        dx = self.checkpoint_position[0] - float(
            self.get_parameter('checkpoint_world_x').value)
        dy = self.checkpoint_position[1] - float(
            self.get_parameter('checkpoint_world_y').value)
        inside = math.hypot(dx, dy) <= float(
            self.get_parameter('checkpoint_radius').value)
        if not inside:
            self.checkpoint_entered_at = None
            return
        if self.checkpoint_entered_at is None:
            self.checkpoint_entered_at = now
            return
        if now - self.checkpoint_entered_at >= float(
                self.get_parameter('checkpoint_dwell').value):
            if self.rules.complete_checkpoint(self.checkpoint_robot_id, now):
                self.get_logger().info('navigation checkpoint completed')
                self._publish_state()

    def _publish_navigation_task(self) -> None:
        participant = self.rules.participants.get(self.checkpoint_robot_id)
        if not self.enable_checkpoint or self.task_published \
                or participant is None or participant.mode != AUTO:
            return
        task = NavigationTask()
        task.header.stamp = self.get_clock().now().to_msg()
        task.header.frame_id = 'map'
        task.task_id = 1
        task.robot_id = self.checkpoint_robot_id
        task.goal.header = task.header
        task.goal.pose.position.x = float(self.get_parameter('checkpoint_x').value)
        task.goal.pose.position.y = float(self.get_parameter('checkpoint_y').value)
        yaw = float(self.get_parameter('checkpoint_yaw').value)
        task.goal.pose.orientation.z = math.sin(yaw / 2.0)
        task.goal.pose.orientation.w = math.cos(yaw / 2.0)
        task.arrival_radius = float(self.get_parameter('checkpoint_radius').value)
        task.dwell_time = float(self.get_parameter('checkpoint_dwell').value)
        task.score_reward = self.rules.config.checkpoint_score
        task.buff_damage = self.rules.config.buff_damage
        task.buff_duration = self.rules.config.buff_duration
        self.task_pub.publish(task)
        self.task_published = True

    def _publish_state(self) -> None:
        now_ros = self.get_clock().now()
        now = now_ros.nanoseconds / 1e9
        for robot_id, participant in self.rules.participants.items():
            status = RobotStatus()
            status.header.stamp = now_ros.to_msg()
            status.header.frame_id = f'{robot_id}/base_link'
            status.robot_id = robot_id
            status.control_mode = participant.mode
            status.hp = participant.hp
            status.max_hp = self.rules.max_hp(robot_id)
            status.damage_per_shot = self.rules.current_damage(robot_id, now)
            status.remaining_ammo = participant.ammo
            status.alive = participant.hp > 0
            status.buff_remaining = self.rules.buff_remaining(robot_id, now)
            self.status_pubs[robot_id].publish(status)

        match = MatchState()
        match.header.stamp = now_ros.to_msg()
        match.header.frame_id = 'world'
        match.phase = self.rules.phase
        match.red_score = self.rules.score(self.red_robot_id)
        match.blue_score = self.rules.score(self.blue_robot_id)
        match.remaining_time = self.rules.remaining_time(now)
        match.winner = self.rules.winner
        match.reason = self.rules.finish_reason
        self.match_pub.publish(match)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RefereeNode()
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


if __name__ == '__main__':
    main()
