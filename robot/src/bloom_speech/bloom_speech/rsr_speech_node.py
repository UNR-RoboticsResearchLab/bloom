import json
import os
import tempfile
import time
from pathlib import Path
from urllib.parse import urlsplit

import pygame
import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ROBOT_CFG_PATH = Path.home() / '.bloom' / 'robot.cfg'
POLL_INTERVAL_SEC = 2.0
CONFIG_RETRY_INTERVAL_SEC = 5.0


def read_robot_config(path: Path) -> dict:
    values = {}
    if not path.exists():
        return values
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, value = line.split('=', 1)
        values[key.strip()] = value.strip()
    return values


class RsrSpeechNode(Node):
    def __init__(self):
        super().__init__('rsr_speech_node')

        self.robot_id = None
        self.base_url = None

        self.visemes_pub = self.create_publisher(String, '/face/visemes', 10)
        self.face_cmd_pub = self.create_publisher(String, '/face/command', 10)

        pygame.mixer.init()

        self._config_timer = self.create_timer(CONFIG_RETRY_INTERVAL_SEC, self.try_load_config)
        self._poll_timer = None
        self.try_load_config()

    def try_load_config(self):
        cfg = read_robot_config(ROBOT_CFG_PATH)
        robot_id = cfg.get('robot_id')
        base_url = cfg.get('base_url')

        if not robot_id or not base_url:
            self.get_logger().warn(
                f'Waiting for robot_id/base_url in {ROBOT_CFG_PATH} (robot not yet registered?)')
            return

        self.robot_id = robot_id
        self.base_url = base_url.rstrip('/')
        self.get_logger().info(f'RSR speech node ready - robot_id={self.robot_id} base_url={self.base_url}')

        self._config_timer.cancel()
        self._poll_timer = self.create_timer(POLL_INTERVAL_SEC, self.poll_pending)

    def poll_pending(self):
        try:
            resp = requests.get(
                f'{self.base_url}/api/rsr-speech/{self.robot_id}/pending', timeout=5)
        except requests.RequestException as e:
            self.get_logger().warn(f'RSR pending poll failed: {e}')
            return

        if resp.status_code == 204:
            return
        if resp.status_code != 200:
            self.get_logger().warn(f'RSR pending poll returned {resp.status_code}')
            return

        command = resp.json()
        try:
            self.play_command(command)
        except Exception as e:
            self.get_logger().error(f'RSR playback failed: {e}')
        finally:
            self.acknowledge_pending()

    def acknowledge_pending(self):
        try:
            requests.delete(
                f'{self.base_url}/api/rsr-speech/{self.robot_id}/pending', timeout=5)
        except requests.RequestException as e:
            self.get_logger().warn(f'Failed to acknowledge RSR command: {e}')

    def play_command(self, command: dict):
        audio_url = command['audioUrl']
        viseme_url = command['visemeUrl']

        self.get_logger().info(f'Playing RSR sentence {command.get("sentenceId")}')

        audio_path = self.download_to_temp(audio_url)
        visemes = self.download_visemes(viseme_url)

        try:
            if visemes:
                self.send_visemes(visemes)
                time.sleep(0.1)
                self.set_face_mode('viseme')
                time.sleep(0.1)

            pygame.mixer.music.load(audio_path)

            if visemes:
                self.start_audio_sync()
                time.sleep(0.05)

            time.sleep(0.15)
            pygame.mixer.music.play()

            while pygame.mixer.music.get_busy():
                time.sleep(0.1)

            self.stop_audio_sync()
            self.set_face_mode('emotion')
            pygame.mixer.music.unload()
        finally:
            if os.path.exists(audio_path):
                os.remove(audio_path)

    def download_to_temp(self, relative_url: str) -> str:
        url = f'{self.base_url}{relative_url}'
        resp = requests.get(url, timeout=10)
        resp.raise_for_status()

        suffix = os.path.splitext(urlsplit(relative_url).path)[1] or '.wav'
        with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as f:
            f.write(resp.content)
            return f.name

    def download_visemes(self, relative_url: str) -> list:
        url = f'{self.base_url}{relative_url}'
        resp = requests.get(url, timeout=10)
        resp.raise_for_status()
        events = resp.json()
        return [
            {'viseme_id': int(e['viseme_id']), 'audio_offset': int(e['timestamp_ms']) * 10000}
            for e in events
        ]

    def send_visemes(self, visemes: list):
        msg = String()
        msg.data = json.dumps(visemes)
        self.visemes_pub.publish(msg)

    def set_face_mode(self, mode: str):
        msg = String()
        msg.data = json.dumps({'command': 'set_mode', 'mode': mode})
        self.face_cmd_pub.publish(msg)

    def start_audio_sync(self):
        msg = String()
        msg.data = json.dumps({'command': 'start_audio_sync'})
        self.face_cmd_pub.publish(msg)

    def stop_audio_sync(self):
        msg = String()
        msg.data = json.dumps({'command': 'stop_audio_sync'})
        self.face_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RsrSpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
