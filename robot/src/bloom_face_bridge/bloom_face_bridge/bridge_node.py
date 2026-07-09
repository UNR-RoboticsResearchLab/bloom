import asyncio
import json
import os
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import websockets
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

WS_PORT = 8765
HTTP_PORT = 8766

# Every topic the pygame bloom_face/face_node.py subscribes to today. The
# bridge is a second, drop-in subscriber alongside it -- no existing
# publisher (tts_node, state_manager, lesson_coordinator, lesson_poller,
# rsr_speech_node) needs to change.
BRIDGED_TOPICS = [
    'face_emotion',
    'robot/face_state',
    'robot/state',
    '/face/visemes',
    '/face/command',
    '/face/visual_aid',
    '/robot/session_code',
    '/stt/enable',
]


class WebSocketBroadcaster:
    """Runs a websockets server on its own asyncio event loop thread and lets
    ROS subscription callbacks (running on rclpy's executor thread) push
    messages into it via asyncio.run_coroutine_threadsafe."""

    def __init__(self, port: int, logger):
        self._port = port
        self._logger = logger
        self._clients = set()
        self._loop = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())

    async def _serve(self):
        async with websockets.serve(self._handle_client, 'localhost', self._port):
            self._logger.info(f'Face bridge WebSocket listening on ws://localhost:{self._port}')
            await asyncio.Future()

    async def _handle_client(self, websocket):
        self._clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self._clients.discard(websocket)

    def broadcast(self, message: str):
        if self._loop is None:
            return
        asyncio.run_coroutine_threadsafe(self._broadcast_async(message), self._loop)

    async def _broadcast_async(self, message: str):
        stale = []
        for client in list(self._clients):
            try:
                await client.send(message)
            except Exception:
                stale.append(client)
        for client in stale:
            self._clients.discard(client)


def start_static_http_server(directory: str, port: int, logger) -> ThreadingHTTPServer:
    handler = partial(SimpleHTTPRequestHandler, directory=directory)
    httpd = ThreadingHTTPServer(('localhost', port), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    logger.info(f'Serving visual aids from {directory} at http://localhost:{port}/')
    return httpd


class FaceBridgeNode(Node):
    def __init__(self):
        super().__init__('bloom_face_bridge')
        self.broadcaster = WebSocketBroadcaster(WS_PORT, self.get_logger())

        for topic in BRIDGED_TOPICS:
            self.create_subscription(String, topic, partial(self.on_message, topic), 10)

        self.get_logger().info(f'Bridging {len(BRIDGED_TOPICS)} ROS topics to WebSocket clients')

    def on_message(self, topic: str, msg: String):
        event = json.dumps({'topic': topic, 'data': self._try_parse_json(msg.data)})
        self.broadcaster.broadcast(event)

    @staticmethod
    def _try_parse_json(data: str):
        try:
            return json.loads(data)
        except (json.JSONDecodeError, TypeError):
            return data


def main(args=None):
    rclpy.init(args=args)
    node = FaceBridgeNode()
    node.broadcaster.start()

    visual_aids_dir = os.path.join(get_package_share_directory('bloom_face'), 'visual_aids')
    if os.path.isdir(visual_aids_dir):
        start_static_http_server(visual_aids_dir, HTTP_PORT, node.get_logger())
    else:
        node.get_logger().warn(
            f'visual_aids dir not found at {visual_aids_dir}; image overlay HTTP route disabled')

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
