import os
import sys
import math
import random
import threading
import json
import time

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory



VISEME_TO_SHAPE = {
    0:  'CLOSED',
    1:  'MEDIUM_OPEN',
    2:  'WIDE_OPEN',
    3:  'ROUNDED',
    4:  'MEDIUM_OPEN',
    5:  'MEDIUM_OPEN',
    6:  'NARROW_SMILE',
    7:  'ROUNDED',
    8:  'ROUNDED',
    9:  'DIPHTHONG',
    10: 'NARROW_SMILE',
    11: 'DIPHTHONG',
    12: 'DIPHTHONG',
    13: 'ROUNDED',
    14: 'ROUNDED',
    15: 'CLOSED',
    16: 'NARROW_VERTICAL',
    17: 'NARROW_VERTICAL',
    18: 'NARROW_HORIZONTAL',
    19: 'NARROW_HORIZONTAL',
    20: 'PUCKERED',
    21: 'MEDIUM_OPEN',
}


EMOTION_COLORS = {
    'neutral':   (230, 240, 255),
    'happy':     (255, 250, 200),
    'sad':       (200, 210, 230),
    'excited':   (255, 240, 200),
    'calm':      (220, 240, 255),
    'surprised': (255, 245, 230),
    'thinking':  (240, 240, 250),
    'sleepy':    (210, 220, 240),
}


class Eye:
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.base_size = size
        self.current_size = size
        self.target_size = size
        self.blink_timer = 0.0
        self.pupil_offset_x = 0.0
        self.pupil_offset_y = 0.0
        self.target_pupil_x = 0.0
        self.target_pupil_y = 0.0

    def update(self, dt):
        self.current_size += (self.target_size - self.current_size) * 0.3
        self.pupil_offset_x += (self.target_pupil_x - self.pupil_offset_x) * 0.1
        self.pupil_offset_y += (self.target_pupil_y - self.pupil_offset_y) * 0.1

    def draw(self, surface, emotion):
        w = int(self.current_size * 2)
        h = int(self.current_size * 1.4)
        x = int(self.x - self.current_size)
        y = int(self.y - self.current_size * 0.7)

        
        pygame.draw.ellipse(surface, (255, 255, 255), (x, y, w, h))
        pygame.draw.ellipse(surface, (200, 200, 200), (x, y, w, h), 2)

        
        pupil_size = int(self.current_size * 0.5)
        pupil_x = int(self.x + self.pupil_offset_x)
        pupil_y = int(self.y + self.pupil_offset_y)
        pygame.draw.circle(surface, (50, 50, 50), (pupil_x, pupil_y), pupil_size)

        
        highlight_size = max(1, int(pupil_size * 0.3))
        highlight_x = int(pupil_x - pupil_size * 0.2)
        highlight_y = int(pupil_y - pupil_size * 0.2)
        pygame.draw.circle(surface, (255, 255, 255), (highlight_x, highlight_y), highlight_size)

    def blink(self):
        self.target_size = self.base_size * 0.1
        self.blink_timer = 0.2

    def open(self):
        self.target_size = self.base_size

    def look_at(self, x, y, max_offset=20):
        dx = x - self.x
        dy = y - self.y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > 0:
            self.target_pupil_x = (dx / dist) * min(dist * 0.2, max_offset)
            self.target_pupil_y = (dy / dist) * min(dist * 0.2, max_offset)


class BlossomFace:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.emotion = 'neutral'

        eye_y = height * 0.4
        eye_spacing = width * 0.25
        eye_size = min(width, height) * 0.09

        self.left_eye = Eye(width / 2 - eye_spacing, eye_y, eye_size)
        self.right_eye = Eye(width / 2 + eye_spacing, eye_y, eye_size)

        self.time = 0.0
        self.blink_cooldown = 0.0
        self.next_blink = random.uniform(2.0, 5.0)

        self.mouth_y = height * 0.65
        self.mouth_curve = 0.0
        self.target_mouth_curve = 0.0

        self.bg_color = list(EMOTION_COLORS['neutral'])
        self.target_bg_color = list(EMOTION_COLORS['neutral'])

        
        self.mouth_mode = 'emotion'
        self.viseme_timeline = []
        self.current_viseme_index = 0
        self.current_viseme_id = 0
        self.audio_start_time = 0.0
        self.is_playing_audio = False

        
        self.visual_aid_active = False
        self.visual_aid_images = []
        self.visual_aid_labels = []
        self.loaded_surfaces = []
        self.visual_aid_footers = []

        
        pygame.font.init()
        label_size = max(24, int(height * 0.06))
        self.label_font = pygame.font.SysFont('Arial', label_size, bold=True)

    def set_emotion(self, emotion):
        self.emotion = emotion
        color = EMOTION_COLORS.get(emotion, EMOTION_COLORS['neutral'])
        self.target_bg_color = list(color)

        if emotion == 'happy':
            self.target_mouth_curve = 40
        elif emotion == 'sad':
            self.target_mouth_curve = -30
        elif emotion == 'excited':
            self.target_mouth_curve = 50
            self.left_eye.target_size = self.left_eye.base_size * 1.2
            self.right_eye.target_size = self.right_eye.base_size * 1.2
        elif emotion == 'calm':
            self.target_mouth_curve = 10
            self.left_eye.target_size = self.left_eye.base_size * 0.9
            self.right_eye.target_size = self.right_eye.base_size * 0.9
        elif emotion == 'surprised':
            self.target_mouth_curve = 0
            self.left_eye.target_size = self.left_eye.base_size * 1.4
            self.right_eye.target_size = self.right_eye.base_size * 1.4
        elif emotion == 'thinking':
            self.target_mouth_curve = 5
            self.left_eye.look_at(self.width * 0.3, self.height * 0.3)
        elif emotion == 'sleepy':
            self.target_mouth_curve = 0
            self.left_eye.target_size = self.left_eye.base_size * 0.5
            self.right_eye.target_size = self.right_eye.base_size * 0.5
        else:
            self.target_mouth_curve = 0
            self.left_eye.target_size = self.left_eye.base_size
            self.right_eye.target_size = self.right_eye.base_size

    def set_mouth_mode(self, mode):
        self.mouth_mode = mode

    def set_viseme_timeline(self, visemes):
        self.viseme_timeline = visemes
        self.current_viseme_index = 0

    def start_audio_sync(self):
        self.audio_start_time = time.time()
        self.is_playing_audio = True
        self.current_viseme_index = 0
        self.mouth_mode = 'viseme'

    def stop_audio_sync(self):
        self.is_playing_audio = False
        self.current_viseme_id = 0
        self.mouth_mode = 'emotion'

    def show_visual_aid(self, images, labels, visual_aids_dir, footers=None):
        self.visual_aid_labels = labels
        self.visual_aid_footers = footers or []
        self.loaded_surfaces = []

        for img_path in images:
            full_path = os.path.join(visual_aids_dir, img_path)
            if os.path.exists(full_path):
                try:
                    surf = pygame.image.load(full_path).convert_alpha()
                    self.loaded_surfaces.append(surf)
                except Exception:
                    self.loaded_surfaces.append(None)
            else:
                self.get_logger().warn(f'Visual aid not found: {full_path}') if hasattr(self, 'get_logger') else None
                self.loaded_surfaces.append(None)

        self.visual_aid_active = True

    def hide_visual_aid(self):
        self.visual_aid_active = False
        self.loaded_surfaces = []
        self.visual_aid_labels = []
        self.visual_aid_footers = []

    def update(self, dt):
        self.time += dt

        self.left_eye.update(dt)
        self.right_eye.update(dt)

        self.blink_cooldown -= dt
        if self.left_eye.blink_timer > 0:
            self.left_eye.blink_timer -= dt
            if self.left_eye.blink_timer <= 0:
                self.left_eye.open()
                self.right_eye.open()

        if self.blink_cooldown <= 0 and self.time > self.next_blink:
            self.left_eye.blink()
            self.right_eye.blink()
            self.blink_cooldown = 0.2
            self.next_blink = self.time + random.uniform(2.0, 5.0)

        self.mouth_curve += (self.target_mouth_curve - self.mouth_curve) * 0.1

        for i in range(3):
            self.bg_color[i] += (self.target_bg_color[i] - self.bg_color[i]) * 0.05

        if self.is_playing_audio and self.viseme_timeline:
            current_time_units = (time.time() - self.audio_start_time) * 10000000
            while self.current_viseme_index < len(self.viseme_timeline) - 1:
                next_viseme = self.viseme_timeline[self.current_viseme_index + 1]
                if current_time_units >= next_viseme['audio_offset']:
                    self.current_viseme_index += 1
                    self.current_viseme_id = self.viseme_timeline[self.current_viseme_index]['viseme_id']
                else:
                    break

        if self.emotion in ('calm', 'neutral'):
            breath = math.sin(self.time * 2) * 2
            self.left_eye.y = self.height * 0.4 + breath
            self.right_eye.y = self.height * 0.4 + breath

    def draw(self, surface):
        if self.visual_aid_active:
            self.draw_visual_aid(surface)
        else:
            self.draw_face(surface)

    def draw_face(self, surface):
        surface.fill(tuple(int(c) for c in self.bg_color))
        self.left_eye.draw(surface, self.emotion)
        self.right_eye.draw(surface, self.emotion)
        self.draw_mouth(surface)

    def draw_mouth(self, surface):
        if self.mouth_mode == 'viseme':
            self.draw_viseme_mouth(surface)
        else:
            self.draw_emotion_mouth(surface)

    def draw_emotion_mouth(self, surface):
        mouth_width = int(self.width * 0.19)
        mouth_x = self.width // 2
        mouth_y = int(self.mouth_y)

        color = (100, 100, 100)
        line_width = max(4, int(self.width * 0.01))

        if abs(self.mouth_curve) < 5:
            pygame.draw.line(surface, color,
                             (mouth_x - mouth_width // 2, mouth_y),
                             (mouth_x + mouth_width // 2, mouth_y),
                             line_width)
        else:
            points = []
            for i in range(20):
                t = i / 19.0
                x = mouth_x - mouth_width // 2 + t * mouth_width
                curve_t = (t - 0.5) * 2
                y = mouth_y + self.mouth_curve * (1 - curve_t * curve_t)
                points.append((int(x), int(y)))
            if len(points) > 1:
                pygame.draw.lines(surface, color, False, points, line_width)

    def draw_viseme_mouth(self, surface):
        shape_name = VISEME_TO_SHAPE.get(self.current_viseme_id, 'CLOSED')
        s = self.width
        mouth_x = self.width // 2
        mouth_y = int(self.mouth_y)

        fill_color = (50, 50, 50)
        outline_color = (100, 100, 100)
        line_width = max(4, int(s * 0.01))

        shapes = {
            'CLOSED':            (s * 0.19,  0,         'line'),
            'WIDE_OPEN':         (s * 0.175, s * 0.10,  'oval'),
            'MEDIUM_OPEN':       (s * 0.163, s * 0.075, 'oval'),
            'NARROW_SMILE':      (s * 0.19,  s * 0.044, 'oval'),
            'ROUNDED':           (s * 0.125, s * 0.125, 'circle'),
            'DIPHTHONG':         (s * 0.15,  s * 0.081, 'oval'),
            'NARROW_VERTICAL':   (s * 0.075, s * 0.05,  'oval'),
            'NARROW_HORIZONTAL': (s * 0.15,  s * 0.025, 'oval'),
            'PUCKERED':          (s * 0.088, s * 0.088, 'circle_pursed'),
        }

        w, h, shape_type = shapes.get(shape_name, shapes['CLOSED'])
        w, h = int(w), int(h)

        if shape_type == 'line':
            pygame.draw.line(surface, outline_color,
                             (mouth_x - w // 2, mouth_y),
                             (mouth_x + w // 2, mouth_y),
                             line_width)
        elif shape_type in ('oval', 'circle'):
            r_x = max(1, w // 2)
            r_y = max(1, h // 2)
            rect = pygame.Rect(mouth_x - r_x, mouth_y - r_y, r_x * 2, r_y * 2)
            pygame.draw.ellipse(surface, fill_color, rect)
            pygame.draw.ellipse(surface, outline_color, rect, line_width)
        elif shape_type == 'circle_pursed':
            r = max(1, w // 2)
            pygame.draw.circle(surface, fill_color, (mouth_x, mouth_y), r)
            pygame.draw.circle(surface, outline_color, (mouth_x, mouth_y), r, line_width + 2)

    def draw_visual_aid(self, surface):
        bg_color = tuple(int(c) for c in self.bg_color)
        surface.fill(bg_color)
        count = len(self.loaded_surfaces)
        if count == 0:
            return

        padding = int(self.width * 0.02)
        header_height = int(self.height * 0.12)
        footer_height = int(self.height * 0.10) if self.visual_aid_footers else 0

        if count == 1:
            label = self.visual_aid_labels[0] if self.visual_aid_labels else ''
            label_surf = self.label_font.render(label, True, (50, 50, 50))
            label_x = self.width // 2 - label_surf.get_width() // 2
            surface.blit(label_surf, (label_x, padding))

            img_surf = self.loaded_surfaces[0]
            if img_surf:
                img_area_h = self.height - header_height - footer_height - padding
                img_area_w = self.width - padding * 2
                scaled = self.scale_image(img_surf, img_area_w, img_area_h)
                img_x = self.width // 2 - scaled.get_width() // 2
                img_y = header_height
                surface.blit(scaled, (img_x, img_y))

            if self.visual_aid_footers:
                footer = self.visual_aid_footers[0]
                footer_surf = self.label_font.render(footer, True, (50, 50, 50))
                footer_x = self.width // 2 - footer_surf.get_width() // 2
                footer_y = self.height - footer_height
                surface.blit(footer_surf, (footer_x, footer_y))

        elif count >= 2:
            half_w = self.width // 2

            # Divider line
            pygame.draw.line(surface, (180, 180, 180),
                            (half_w, 0), (half_w, self.height), 2)

            for i in range(2):
                offset_x = i * half_w

                # Header label (word spelling)
                label = self.visual_aid_labels[i] if i < len(self.visual_aid_labels) else ''
                label_surf = self.label_font.render(label, True, (50, 50, 50))
                label_x = offset_x + half_w // 2 - label_surf.get_width() // 2
                surface.blit(label_surf, (label_x, padding))

                # Image
                img_surf = self.loaded_surfaces[i] if i < len(self.loaded_surfaces) else None
                if img_surf:
                    img_area_w = half_w - padding * 2
                    img_area_h = self.height - header_height - footer_height - padding * 2
                    scaled = self.scale_image(img_surf, img_area_w, img_area_h)
                    img_x = offset_x + half_w // 2 - scaled.get_width() // 2
                    img_y = header_height
                    surface.blit(scaled, (img_x, img_y))

                # Footer label (A/B)
                if self.visual_aid_footers and i < len(self.visual_aid_footers):
                    footer = self.visual_aid_footers[i]
                    footer_surf = self.label_font.render(footer, True, (50, 50, 50))
                    footer_x = offset_x + half_w // 2 - footer_surf.get_width() // 2
                    footer_y = self.height - footer_height
                    surface.blit(footer_surf, (footer_x, footer_y))

    def scale_image(self, surf, max_w, max_h):
        orig_w, orig_h = surf.get_size()
        scale = min(max_w / orig_w, max_h / orig_h)
        new_w = max(1, int(orig_w * scale))
        new_h = max(1, int(orig_h * scale))
        return pygame.transform.smoothscale(surf, (new_w, new_h))


class FaceNode(Node):
    def __init__(self):
        super().__init__('face_node')
        self.get_logger().info('Initializing face node')

        
        self.visual_aids_dir = os.path.join(
            get_package_share_directory('bloom_face'), 'visual_aids'
        )

        pygame.init()
        pygame.font.init()

        num_displays = pygame.display.get_num_displays()
        print(f'Displays found: {num_displays}')
        for i in range(num_displays):
            print(f'  Display {i}: {pygame.display.get_desktop_sizes()[i]}')

        display_index = 1 if num_displays > 1 else 0
        self.width, self.height = 800, 480
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.FULLSCREEN, display=display_index)
        print(f'Using display {display_index}: {self.width}x{self.height}')
        pygame.mouse.set_visible(False)
        pygame.display.set_caption('Bloom')

        self.clock = pygame.time.Clock()
        self.face = BlossomFace(self.width, self.height)
        self.face.set_emotion('happy')

        
        self.lock = threading.Lock()

        
        self.create_subscription(String, 'face_emotion', self.on_emotion, 10)
        self.create_subscription(String, 'robot/face_state', self.on_face_state, 10)
        self.create_subscription(String, 'robot/state', self.on_robot_state, 10)
        self.create_subscription(String, '/face/visemes', self.on_visemes, 10)
        self.create_subscription(String, '/face/command', self.on_face_command, 10)
        self.create_subscription(String, '/face/visual_aid', self.on_visual_aid, 10)

        
        self.create_timer(1.0 / 30.0, self.render)

        self.get_logger().info('Face node ready')

    def on_emotion(self, msg: String):
        with self.lock:
            self.face.set_emotion(msg.data.lower())

    def on_face_state(self, msg: String):
        
        state = msg.data.lower()
        emotion_map = {
            'smile':   'happy',
            'neutral': 'neutral',
            'sad':     'sad',
            'excited': 'excited',
            'thinking': 'thinking',
        }
        emotion = emotion_map.get(state, state)
        with self.lock:
            self.face.set_emotion(emotion)

    def on_robot_state(self, msg: String):
        state = msg.data.lower()
        state_emotion_map = {
            'waiting':  'neutral',
            'talking':  'happy',
            'loading':  'thinking',
            'idle':     'calm',
        }
        if state in state_emotion_map:
            with self.lock:
                self.face.set_emotion(state_emotion_map[state])

    def on_visemes(self, msg: String):
        try:
            visemes = json.loads(msg.data)
            with self.lock:
                self.face.set_viseme_timeline(visemes)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse visemes: {e}')

    def on_face_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            command = cmd.get('command', '')
            with self.lock:
                if command == 'start_audio_sync':
                    self.face.start_audio_sync()
                elif command == 'stop_audio_sync':
                    self.face.stop_audio_sync()
                elif command == 'set_mode':
                    self.face.set_mouth_mode(cmd.get('mode', 'emotion'))
                elif command == 'set_emotion':
                    self.face.set_emotion(cmd.get('emotion', 'neutral'))
        except Exception as e:
            self.get_logger().warn(f'Failed to parse face command: {e}')

    def on_visual_aid(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get('command') == 'hide':
                with self.lock:
                    self.face.hide_visual_aid()
            else:
                images = data.get('images', [])
                labels = data.get('labels', [])
                footers = data.get('footers', [])
                with self.lock:
                    self.face.show_visual_aid(images, labels, self.visual_aids_dir, footers)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse visual aid command: {e}')

    def render(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    rclpy.shutdown()

        dt = self.clock.tick(30) / 1000.0

        with self.lock:
            self.face.update(dt)
            self.face.draw(self.screen)

        pygame.display.flip()

    def destroy_node(self):
        pygame.quit()
        super().destroy_node()


def main(args=None):
    env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', '..', '..', '..', '.env')
    if os.path.exists(env_path):
        with open(env_path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    key, value = line.split('=', 1)
                    os.environ.setdefault(key.strip(), value.strip())

    rclpy.init(args=args)
    node = FaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
