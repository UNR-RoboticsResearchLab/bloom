import os
import sys
import json
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import azure.cognitiveservices.speech as speechsdk

from bloom_speech.mic_utils import find_alsa_capture_device


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

        mic_device = find_alsa_capture_device(logger=self.get_logger())
        if mic_device:
            self.audio_config = speechsdk.audio.AudioConfig(device_name=mic_device)
        else:
            self.get_logger().warn(
                '[STT] ReSpeaker Lite not found - falling back to system '
                'default microphone (set BLOOM_MIC_DEVICE to override)')
            self.audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
        self.speech_config = speech_config

        self.current_state = 'idle'
        self.recognizer = None
        self.recognizer_lock = threading.Lock()
        self.is_recognizing = False
        self.mic_enabled = False  

        self.result_pub = self.create_publisher(String, '/vosk/result', 10)
        self.state_sub = self.create_subscription(
            String, 'robot/state', self.on_state_update, 10)
        self.enable_sub = self.create_subscription(
            String, '/stt/enable', self.on_enable_update, 10)
        self.mode_sub = self.create_subscription(
            String, '/llm/mode', self.on_mode_update, 10)

        self.get_logger().info('STT node ready - mic muted, waiting for mode activation')

    def _start_recognizer(self):
        with self.recognizer_lock:
            if self.is_recognizing:
                return

            self.recognizer = speechsdk.SpeechRecognizer(
                speech_config=self.speech_config,
                audio_config=self.audio_config
            )

            def on_recognized(evt):
                text = evt.result.text.strip()
                if not text:
                    return
                self.get_logger().info(f'[STT] Recognized: {text}')
                msg = String()
                msg.data = text
                self.result_pub.publish(msg)

            def on_canceled(evt):
                self.get_logger().warn(f'STT canceled: {evt.result.cancellation_details.reason}')
                if evt.result.cancellation_details.reason == speechsdk.CancellationReason.Error:
                    self.get_logger().error(f'STT error: {evt.result.cancellation_details.error_details}')

            self.recognizer.recognized.connect(on_recognized)
            self.recognizer.canceled.connect(on_canceled)
            self.recognizer.start_continuous_recognition()
            self.is_recognizing = True
            self.get_logger().info('[STT] Recognition started')

    def _stop_recognizer(self):
        with self.recognizer_lock:
            if not self.is_recognizing or self.recognizer is None:
                return
            self.recognizer.stop_continuous_recognition()
            self.recognizer = None
            self.is_recognizing = False
            self.get_logger().info('[STT] Recognition stopped')

    def on_state_update(self, msg: String):
        new_state = msg.data
        if new_state == self.current_state:
            return
        self.current_state = new_state
        self.get_logger().info(f'[STT] Robot state -> {new_state}')

        if new_state in ('talking', 'loading', 'idle'):
            threading.Thread(target=self._stop_recognizer, daemon=True).start()
        elif new_state == 'waiting':
            if self.mic_enabled:
                threading.Thread(target=self._start_recognizer, daemon=True).start()



    def on_enable_update(self, msg: String):
        enabled = msg.data.lower() == 'true'
        if enabled == self.mic_enabled:
            return
        self.mic_enabled = enabled
        self.get_logger().info(f'[STT] Mic enabled -> {enabled}')
        if enabled and self.current_state not in ('talking', 'loading', 'idle'):
            threading.Thread(target=self._start_recognizer, daemon=True).start()
        elif not enabled:
            threading.Thread(target=self._stop_recognizer, daemon=True).start()

    def on_mode_update(self, msg: String):
        new_mode = msg.data
        self.get_logger().info(f'[STT] LLM mode -> {new_mode}')
        if new_mode == 'free_conversation':
            self.mic_enabled = True
        elif new_mode in ('lesson_mode', 'single_turn', 'lesson_tangent'):
            self.mic_enabled = False
            threading.Thread(target=self._stop_recognizer, daemon=True).start()


    def destroy_node(self):
        self._stop_recognizer()
        super().destroy_node()


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