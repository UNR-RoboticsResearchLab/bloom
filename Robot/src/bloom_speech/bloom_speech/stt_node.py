import os
import sys
import json
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import azure.cognitiveservices.speech as speechsdk


class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.get_logger().info('Initializing STT node')

        stt_key = os.environ.get('AZURE_TTS_KEY', '')
        stt_region = os.environ.get('AZURE_TTS_REGION', 'westus2')

        if not stt_key:
            self.get_logger().error('AZURE_TTS_KEY not set - check your .env file')

        speech_config = speechsdk.SpeechConfig(subscription=stt_key, region=stt_region)
        speech_config.speech_recognition_language = 'en-US'

        self.audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
        self.speech_config = speech_config

        self.current_state = 'waiting'
        self.is_listening = False
        self.lock = threading.Lock()

        self.result_pub = self.create_publisher(String, '/vosk/result', 10)
        self.state_sub = self.create_subscription(
            String, 'robot/state', self.on_state_update, 10)

        self.listen_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listen_thread.start()

        self.get_logger().info('STT node ready - listening for speech')

    def on_state_update(self, msg: String):
        self.current_state = msg.data

    def listen_loop(self):
        self.get_logger().info('Starting continuous recognition')

        recognizer = speechsdk.SpeechRecognizer(
            speech_config=self.speech_config,
            audio_config=self.audio_config
        )

        def on_recognized(evt):
            text = evt.result.text.strip()
            if not text:
                return
            if self.current_state in ('talking', 'loading'):
                self.get_logger().info(f'Ignoring STT input - robot is {self.current_state}')
                return
            self.get_logger().info(f'Recognized: {text}')
            msg = String()
            msg.data = text
            self.result_pub.publish(msg)

        def on_canceled(evt):
            self.get_logger().warn(f'STT canceled: {evt.result.cancellation_details.reason}')
            if evt.result.cancellation_details.reason == speechsdk.CancellationReason.Error:
                self.get_logger().error(f'STT error: {evt.result.cancellation_details.error_details}')

        recognizer.recognized.connect(on_recognized)
        recognizer.canceled.connect(on_canceled)

        recognizer.start_continuous_recognition()
        self.get_logger().info('Continuous recognition started')

        
        import time
        while rclpy.ok():
            time.sleep(0.1)

        recognizer.stop_continuous_recognition()

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
        node = STTNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()