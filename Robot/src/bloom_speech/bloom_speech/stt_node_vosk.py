import os
import sys
import json
import queue
import threading
import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer
from ament_index_python.packages import get_package_share_directory


class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.get_logger().info('Initializing STT node')

        model_path = os.path.expanduser('~/vosk_models/vosk-model-small-en-us-0.15')
        if not os.path.exists(model_path):
            self.get_logger().error(f'Vosk model not found at {model_path}')
            raise RuntimeError('Vosk model not found')

        self.model = Model(model_path)
        self.sample_rate = 16000
        self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
        self.audio_queue = queue.Queue()
        self.current_state = 'waiting'

        self.result_pub = self.create_publisher(String, '/vosk/result', 10)
        self.state_sub = self.create_subscription(
            String, 'robot/state', self.on_state_update, 10)

        self.listen_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listen_thread.start()

        self.get_logger().info('STT node ready - listening for speech')

    def on_state_update(self, msg: String):
        self.current_state = msg.data

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f'Audio status: {status}')
        self.audio_queue.put(bytes(indata))

    def listen_loop(self):
        with sd.RawInputStream(
            samplerate=self.sample_rate,
            blocksize=8000,
            dtype='int16',
            channels=1,
            callback=self.audio_callback
        ):
            self.get_logger().info('Microphone stream opened')
            while rclpy.ok():
                data = self.audio_queue.get()

                if self.current_state in ('talking', 'loading'):
                    continue

                if self.recognizer.AcceptWaveform(data):
                    result = json.loads(self.recognizer.Result())
                    text = result.get('text', '').strip()
                    if text:
                        self.get_logger().info(f'Recognized: {text}')
                        msg = String()
                        msg.data = text
                        self.result_pub.publish(msg)


def main(args=None):
    env_path = os.path.join(get_package_share_directory('bloom_speech'), '.env')
    if os.path.exists(env_path):
        with open(env_path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    key, value = line.split('=', 1)
                    os.environ.setdefault(key.strip(), value.strip())

    rclpy.init(args=args)
    node = STTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()