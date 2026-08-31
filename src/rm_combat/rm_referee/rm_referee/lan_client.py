import argparse
import asyncio
import json
import socket
import time

from websockets.asyncio.client import connect


class InteractiveInbox:
    """Consume server events without writing over the operator's input prompt."""

    def __init__(self):
        self.latest_state = None
        self.latest_roster = None
        self.events = asyncio.Queue()
        self.state_updated = asyncio.Event()
        self.roster_updated = asyncio.Event()

    async def receive(self, websocket):
        async for raw_message in websocket:
            message = json.loads(raw_message)
            message_type = message.get('type')
            if message_type == 'match_state':
                self.latest_state = message
                self.state_updated.set()
            elif message_type == 'roster':
                self.latest_roster = message
                self.roster_updated.set()
            else:
                await self.events.put(message)

    async def request_snapshot(self, websocket, message_type, timeout=1.0):
        if message_type == 'status':
            updated = self.state_updated
            attribute = 'latest_state'
        else:
            updated = self.roster_updated
            attribute = 'latest_roster'
        updated.clear()
        await websocket.send(json.dumps({'type': message_type}))
        try:
            await asyncio.wait_for(updated.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            pass
        return getattr(self, attribute)

    async def wait_for_command(self, request_id):
        while True:
            message = await self.events.get()
            print(json.dumps(message, ensure_ascii=False))
            if message.get('type') == 'command_result' and \
                    message.get('request_id') == request_id:
                return message


async def run_client(parsed):
    uri = f'ws://{parsed.host}:{parsed.port}'
    async with connect(uri, max_size=4096, ping_interval=20, ping_timeout=20) as websocket:
        await websocket.send(json.dumps({
            'type': 'join', 'role': parsed.role,
            'name': parsed.name, 'token': parsed.token,
        }))
        first = json.loads(await websocket.recv())
        print(json.dumps(first, ensure_ascii=False))
        if first.get('type') == 'error':
            return 2

        if parsed.command:
            request_id = str(time.time_ns())
            await websocket.send(json.dumps({
                'type': 'command', 'command': parsed.command,
                'request_id': request_id,
            }))
            while True:
                message = json.loads(await websocket.recv())
                print(json.dumps(message, ensure_ascii=False))
                if message.get('type') == 'command_result' and \
                        message.get('request_id') == request_id:
                    return 0 if message.get('success') else 1

        if parsed.watch_seconds > 0:
            deadline = asyncio.get_running_loop().time() + parsed.watch_seconds
            while asyncio.get_running_loop().time() < deadline:
                timeout = deadline - asyncio.get_running_loop().time()
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=timeout)
                except asyncio.TimeoutError:
                    break
                print(message)
            return 0

        if parsed.role != 'referee':
            print('Only the referee role accepts interactive commands. '
                  'Use --watch-seconds for a player client.')
            return 0

        inbox = InteractiveInbox()
        receiver = asyncio.create_task(inbox.receive(websocket))
        try:
            await asyncio.sleep(0.05)
            print('commands: start pause resume reset status roster quit')
            while True:
                command = (await asyncio.to_thread(input, '> ')).strip().lower()
                if command in ('quit', 'exit'):
                    break
                if command == 'status':
                    message = await inbox.request_snapshot(websocket, 'status')
                    if message is not None:
                        print(json.dumps(message, ensure_ascii=False))
                    else:
                        print('state unavailable')
                elif command == 'roster':
                    message = await inbox.request_snapshot(websocket, 'roster')
                    if message is not None:
                        print(json.dumps(message, ensure_ascii=False))
                    else:
                        print('roster unavailable')
                elif command in ('start', 'pause', 'resume', 'reset'):
                    request_id = str(time.time_ns())
                    await websocket.send(json.dumps({
                        'type': 'command', 'command': command,
                        'request_id': request_id,
                    }))
                    await inbox.wait_for_command(request_id)
                    await asyncio.sleep(0.05)
                    if inbox.latest_state is not None:
                        print(json.dumps(inbox.latest_state, ensure_ascii=False))
                elif command:
                    print('unknown command')
        finally:
            receiver.cancel()
            try:
                await receiver
            except asyncio.CancelledError:
                pass
        return 0


def main(args=None):
    parser = argparse.ArgumentParser(description='Minimal LAN referee WebSocket client.')
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--role', choices=('red', 'blue', 'referee'), required=True)
    parser.add_argument('--token', required=True)
    parser.add_argument('--name', default=socket.gethostname())
    parser.add_argument('--command', choices=('start', 'pause', 'resume', 'reset'))
    parser.add_argument('--watch-seconds', type=float, default=0.0)
    parsed = parser.parse_args(args)
    raise SystemExit(asyncio.run(run_client(parsed)))


if __name__ == '__main__':
    main()
