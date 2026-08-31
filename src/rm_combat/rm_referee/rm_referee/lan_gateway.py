import asyncio
import hmac
import json
import queue
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from websockets.asyncio.server import serve

from rm_combat_interfaces.msg import MatchState


PHASE_NAMES = {
    MatchState.PREPARING: 'preparing',
    MatchState.RUNNING: 'running',
    MatchState.FINISHED: 'finished',
    MatchState.PAUSED: 'paused',
}


class LanWebSocketServer:
    def __init__(self, host, port, tokens, command_callback, state_callback):
        self.host = host
        self.port = port
        self.tokens = tokens
        self.command_callback = command_callback
        self.state_callback = state_callback
        self.clients = {}
        self.seats = {}
        self.loop = None
        self.stop_event = None
        self.ready = threading.Event()
        self.startup_error = None
        self.thread = threading.Thread(target=self._thread_main, daemon=True)

    def start(self):
        self.thread.start()
        if not self.ready.wait(timeout=5.0):
            raise RuntimeError('WebSocket server startup timed out')
        if self.startup_error is not None:
            raise RuntimeError(str(self.startup_error))

    def stop(self):
        if self.loop is not None and self.stop_event is not None:
            self.loop.call_soon_threadsafe(self.stop_event.set)
        if self.thread.is_alive():
            self.thread.join(timeout=5.0)

    def send_to(self, session_id, message):
        if self.loop is not None:
            asyncio.run_coroutine_threadsafe(
                self._send_to(session_id, message), self.loop)

    def broadcast(self, message):
        if self.loop is not None:
            asyncio.run_coroutine_threadsafe(self._broadcast(message), self.loop)

    def _thread_main(self):
        try:
            asyncio.run(self._run())
        except Exception as error:  # server startup errors must reach the ROS node
            self.startup_error = error
            self.ready.set()

    async def _run(self):
        self.loop = asyncio.get_running_loop()
        self.stop_event = asyncio.Event()
        async with serve(
                self._handle_client, self.host, self.port,
                max_size=4096, ping_interval=20, ping_timeout=20):
            self.ready.set()
            await self.stop_event.wait()

    async def _handle_client(self, websocket):
        session_id = None
        try:
            raw_join = await asyncio.wait_for(websocket.recv(), timeout=10.0)
            join = self._decode(raw_join)
            if join.get('type') != 'join':
                await self._send(websocket, {'type': 'error', 'reason': 'join required'})
                return
            role = str(join.get('role', '')).lower()
            name = str(join.get('name', role))[:64]
            token = str(join.get('token', ''))
            if role not in self.tokens or not hmac.compare_digest(token, self.tokens[role]):
                await self._send(websocket, {'type': 'error', 'reason': 'authentication failed'})
                return
            if role in self.seats:
                await self._send(websocket, {'type': 'error', 'reason': f'{role} seat occupied'})
                return

            session_id = uuid.uuid4().hex
            self.clients[session_id] = {
                'socket': websocket, 'role': role, 'name': name,
            }
            self.seats[role] = session_id
            await self._send(websocket, {
                'type': 'joined', 'session_id': session_id, 'role': role, 'name': name,
            })
            state = self.state_callback()
            if state is not None:
                await self._send(websocket, state)
            await self._broadcast_roster()

            async for raw_message in websocket:
                message = self._decode(raw_message)
                message_type = message.get('type')
                if message_type == 'ping':
                    await self._send(websocket, {'type': 'pong'})
                elif message_type == 'status':
                    state = self.state_callback()
                    await self._send(
                        websocket, state or {'type': 'error', 'reason': 'state unavailable'})
                elif message_type == 'roster':
                    await self._send(websocket, self._roster_message())
                elif message_type == 'command':
                    if role != 'referee':
                        await self._send(
                            websocket, {'type': 'error', 'reason': 'referee role required'})
                        continue
                    command = str(message.get('command', '')).lower()
                    if command not in ('start', 'pause', 'resume', 'reset'):
                        await self._send(
                            websocket, {'type': 'error', 'reason': 'unknown command'})
                        continue
                    request_id = str(message.get('request_id', uuid.uuid4().hex))[:64]
                    self.command_callback(session_id, request_id, command)
                    await self._send(websocket, {
                        'type': 'command_queued', 'request_id': request_id,
                        'command': command,
                    })
                else:
                    await self._send(
                        websocket, {'type': 'error', 'reason': 'unknown message type'})
        except (asyncio.TimeoutError, json.JSONDecodeError, ValueError) as error:
            try:
                await self._send(websocket, {'type': 'error', 'reason': str(error)})
            except Exception:
                pass
        finally:
            if session_id is not None:
                client = self.clients.pop(session_id, None)
                if client is not None and self.seats.get(client['role']) == session_id:
                    self.seats.pop(client['role'], None)
                await self._broadcast_roster()

    @staticmethod
    def _decode(raw_message):
        if not isinstance(raw_message, str):
            raise ValueError('text JSON messages required')
        message = json.loads(raw_message)
        if not isinstance(message, dict):
            raise ValueError('JSON object required')
        return message

    @staticmethod
    async def _send(websocket, message):
        await websocket.send(json.dumps(message, ensure_ascii=False, separators=(',', ':')))

    async def _send_to(self, session_id, message):
        client = self.clients.get(session_id)
        if client is not None:
            await self._send(client['socket'], message)

    async def _broadcast(self, message):
        for client in list(self.clients.values()):
            try:
                await self._send(client['socket'], message)
            except Exception:
                pass

    async def _broadcast_roster(self):
        await self._broadcast(self._roster_message())

    def _roster_message(self):
        roster = {
            role: self.clients[session_id]['name']
            for role, session_id in self.seats.items()
            if session_id in self.clients
        }
        return {'type': 'roster', 'seats': roster}


class RefereeLanGateway(Node):
    def __init__(self):
        super().__init__('referee_lan_gateway')
        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('red_token', 'red-demo')
        self.declare_parameter('blue_token', 'blue-demo')
        self.declare_parameter('referee_token', 'referee-demo')
        self.declare_parameter('state_broadcast_hz', 2.0)
        self.latest_state = None
        self.last_broadcast_time = 0.0
        self.last_state_signature = None
        self.command_queue = queue.Queue()

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            MatchState, '/referee/match_state', self._on_match_state, state_qos)
        self.service_clients = {
            'start': self.create_client(Trigger, '/referee/start_match'),
            'pause': self.create_client(Trigger, '/referee/pause_match'),
            'resume': self.create_client(Trigger, '/referee/resume_match'),
            'reset': self.create_client(Trigger, '/referee/reset_match'),
        }
        tokens = {
            'red': str(self.get_parameter('red_token').value),
            'blue': str(self.get_parameter('blue_token').value),
            'referee': str(self.get_parameter('referee_token').value),
        }
        self.network = LanWebSocketServer(
            str(self.get_parameter('bind_host').value),
            int(self.get_parameter('port').value),
            tokens, self._queue_command, lambda: self.latest_state)
        self.network.start()
        self.timer = self.create_timer(0.05, self._drain_commands)
        self.get_logger().info(
            f"LAN referee listening on ws://{self.get_parameter('bind_host').value}:"
            f"{self.get_parameter('port').value}")

    def _on_match_state(self, state):
        self.latest_state = {
            'type': 'match_state',
            'phase': int(state.phase),
            'phase_name': PHASE_NAMES.get(int(state.phase), 'unknown'),
            'red_score': int(state.red_score),
            'blue_score': int(state.blue_score),
            'remaining_time': round(float(state.remaining_time), 3),
            'winner': state.winner,
            'reason': state.reason,
        }
        signature = (
            self.latest_state['phase'], self.latest_state['red_score'],
            self.latest_state['blue_score'], self.latest_state['winner'],
            self.latest_state['reason'],
        )
        now = time.monotonic()
        broadcast_hz = max(
            0.1, float(self.get_parameter('state_broadcast_hz').value))
        if signature != self.last_state_signature or \
                now - self.last_broadcast_time >= 1.0 / broadcast_hz:
            self.last_state_signature = signature
            self.last_broadcast_time = now
            self.network.broadcast(self.latest_state)

    def _queue_command(self, session_id, request_id, command):
        self.command_queue.put((session_id, request_id, command))

    def _drain_commands(self):
        while True:
            try:
                session_id, request_id, command = self.command_queue.get_nowait()
            except queue.Empty:
                return
            service = self.service_clients[command]
            if not service.service_is_ready():
                self.network.send_to(session_id, {
                    'type': 'command_result', 'request_id': request_id,
                    'command': command, 'success': False,
                    'message': 'referee service unavailable',
                })
                continue
            future = service.call_async(Trigger.Request())
            future.add_done_callback(
                lambda result, sid=session_id, rid=request_id, cmd=command:
                self._service_result(result, sid, rid, cmd))

    def _service_result(self, future, session_id, request_id, command):
        try:
            response = future.result()
            success = bool(response.success)
            message = response.message
        except Exception as error:
            success = False
            message = str(error)
        self.network.send_to(session_id, {
            'type': 'command_result', 'request_id': request_id,
            'command': command, 'success': success, 'message': message,
        })

    def destroy_node(self):
        try:
            self.network.stop()
        except KeyboardInterrupt:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RefereeLanGateway()
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
