import os
import sys
import time
_robot_dir = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              '..', '..', '..', '..', '..', 'bloom', 'Robot'))
if _robot_dir not in sys.path:
    sys.path.insert(0, _robot_dir)
import json
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
from ament_index_python.packages import get_package_share_directory
from tts_module.engine_azure import AzureTTSEngine


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.get_logger().info('Initializing TTS node')

        tts_key = os.environ.get('AZURE_TTS_KEY', '')
        tts_region = os.environ.get('AZURE_TTS_REGION', 'westus2')
        tts_voice = os.environ.get('AZURE_TTS_VOICE', 'en-US-AvaNeural')

        if not tts_key:
            self.get_logger().error('AZURE_TTS_KEY not set - check your .env file')

        self.tts_engine = AzureTTSEngine(
            subscription_key=tts_key,
            region=tts_region,
            default_voice=tts_voice
        )

        self.visemes_pub = self.create_publisher(String, '/face/visemes', 10)
        self.face_cmd_pub = self.create_publisher(String, '/face/command', 10)
        self.tts_done_pub = self.create_publisher(String, '/tts/done', 10)

        pygame.mixer.init()
        self.is_speaking = False
        self._speak_lock = threading.Lock()

        self.tts_sub = self.create_subscription(
            String, '/tts/speak', self.on_speak_request, 10)

        self.state_cmd_pub = self.create_publisher(String, 'robot/state_cmd', 10)

        self.get_logger().info('TTS node ready - listening on /tts/speak')

    def set_face_emotion(self, emotion: str):
        msg = String()
        msg.data = json.dumps({'command': 'set_emotion', 'emotion': emotion})
        self.face_cmd_pub.publish(msg)

    def set_face_mode(self, mode: str):
        msg = String()
        msg.data = json.dumps({'command': 'set_mode', 'mode': mode})
        self.face_cmd_pub.publish(msg)

    def send_visemes(self, visemes: list):
        msg = String()
        msg.data = json.dumps(visemes)
        self.visemes_pub.publish(msg)

    def start_audio_sync(self):
        msg = String()
        msg.data = json.dumps({'command': 'start_audio_sync'})
        self.face_cmd_pub.publish(msg)

    def stop_audio_sync(self):
        msg = String()
        msg.data = json.dumps({'command': 'stop_audio_sync'})
        self.face_cmd_pub.publish(msg)

    def set_robot_state(self, state: str):
        msg = String()
        msg.data = state
        self.state_cmd_pub.publish(msg)
        self.get_logger().info(f'Robot state -> {state}')

    def on_speak_request(self, msg: String):
        if not msg.data:
            return
        thread = threading.Thread(target=self.speak, args=(msg.data,), daemon=True)
        thread.start()

    def speak(self, text: str):
        with self._speak_lock:
            self.get_logger().info(f'Speaking: {text}')
            self.is_speaking = True
            self.set_robot_state('talking')
            self.set_face_emotion('happy')

            result = self.tts_engine.synthesize(text=text, include_viseme=True)

            if not result.metrics.success:
                self.get_logger().error(f'TTS synthesis failed: {result.metrics.error_reason}')
                self.is_speaking = False
                self.set_robot_state('waiting')
                done_msg = String()
                done_msg.data = 'done'
                self.tts_done_pub.publish(done_msg)
                return

            if result.viseme_events:
                visemes = [
                    {'viseme_id': int(e.viseme_id), 'audio_offset': e.timestamp_ms * 10000}
                    for e in result.viseme_events
                ]
                self.get_logger().info(f'Sending {len(visemes)} visemes to face')
                self.send_visemes(visemes)
                time.sleep(0.1)
                self.set_face_mode('viseme')
                time.sleep(0.1)

            try:
                pygame.mixer.music.load(result.audio_filepath)

                if result.viseme_events:
                    self.start_audio_sync()
                    time.sleep(0.05)

                time.sleep(0.15)
                pygame.mixer.music.play()

                while pygame.mixer.music.get_busy():
                    time.sleep(0.1)

                self.stop_audio_sync()
                self.set_face_mode('emotion')
                pygame.mixer.music.unload()

                if os.path.exists(result.audio_filepath):
                    os.remove(result.audio_filepath)

            except Exception as e:
                self.get_logger().error(f'Audio playback error: {e}')

            self.is_speaking = False
            self.set_robot_state('waiting')
            self.get_logger().info('Done speaking')

            done_msg = String()
            done_msg.data = 'done'
            self.tts_done_pub.publish(done_msg)


def main(args=None):
    env_path = os.path.join(get_package_share_directory('bloom_speech'), '.env')
    if os.path.exists(env_path):
        with open(env_path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    key, value = line.split('=', 1)
                    os.environ.setdefault(key.strip(), value.strip())
    else:
        print(f'WARNING: .env not found at {env_path}')
        print('Copy .env.example to .env and fill in your credentials')

    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()