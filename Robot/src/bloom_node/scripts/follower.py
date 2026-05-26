#!/usr/bin/env python3
import math
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2


def _haarcascades_dir() -> str:
    if hasattr(cv2, 'data'):
        return cv2.data.haarcascades
    for path in ('/usr/share/opencv4/haarcascades/', '/usr/share/opencv/haarcascades/'):
        if os.path.isdir(path):
            return path
    raise RuntimeError('Cannot find OpenCV haarcascades directory')

FACE_CASCADE = cv2.CascadeClassifier(_haarcascades_dir() + 'haarcascade_frontalface_default.xml')

# Proportional gain: fraction of frame offset to target angle (radians)
KP_YAW = 2
KP_PITCH = -0.5

# Dead zone: ignore offsets smaller than this fraction of half-frame
DEAD_ZONE = 0.05

# Neutral head height sent with every pose command
NEUTRAL_HEIGHT = 1.0


class CameraFollower(Node):
    def __init__(self):
        super().__init__('camera_follower')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('pose_topic', 'target_pose')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self._image_cb, 10)
        self.pub = self.create_publisher(Pose, pose_topic, 10)

        self.get_logger().info(f'Camera follower ready — listening on {image_topic}')

    def _image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        face = self._detect_face(frame)
        if face is None:
            return

        print(f'Detected face at {face}')

        fx, fy, fw, fh = face
        cx = fx + fw / 2
        cy = fy + fh / 2

        # Normalized offset: -1.0 (left/up) to +1.0 (right/down)
        x_err = (cx - w / 2) / (w / 2)
        y_err = (cy - h / 2) / (h / 2)

        yaw = KP_YAW * x_err if abs(x_err) > DEAD_ZONE else 0.0
        pitch = KP_PITCH * y_err if abs(y_err) > DEAD_ZONE else 0.0

        print(f'Offset x={x_err:.2f} y={y_err:.2f} → yaw={yaw:.2f} pitch={pitch:.2f}')
        pose = Pose()
        pose.position.z = NEUTRAL_HEIGHT
        pose.orientation = self._euler_to_quaternion(pitch, yaw)
        self.pub.publish(pose)

    @staticmethod
    def _euler_to_quaternion(pitch: float, yaw: float) -> 'Quaternion':
        """Build a geometry_msgs/Quaternion from pitch (y-axis) and yaw (z-axis), roll=0."""
        from geometry_msgs.msg import Quaternion
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        q = Quaternion()
        q.w = cy * cp
        q.x = -sy * sp
        q.y = cy * sp
        q.z = sy * cp
        return q

    def _detect_face(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = FACE_CASCADE.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60))
        if len(faces) == 0:
            return None
        # Track the largest face
        return max(faces, key=lambda r: r[2] * r[3])


def main(args=None):
    rclpy.init(args=args)
    node = CameraFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
