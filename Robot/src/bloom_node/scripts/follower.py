#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2


FACE_CASCADE = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
)

# Proportional gain: fraction of frame offset to convert to angular velocity
KP_YAW = 1.2
KP_PITCH = 0.8

# Dead zone: ignore offsets smaller than this fraction of half-frame
DEAD_ZONE = 0.05


class CameraFollower(Node):
    def __init__(self):
        super().__init__('camera_follower')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self._image_cb, 10)
        self.pub = self.create_publisher(Twist, cmd_topic, 10)

        self.get_logger().info(f'Camera follower ready — listening on {image_topic}')

    def _image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        face = self._detect_face(frame)
        if face is None:
            return

        fx, fy, fw, fh = face
        cx = fx + fw / 2
        cy = fy + fh / 2

        # Normalized offset: -1.0 (left/up) to +1.0 (right/down)
        x_err = (cx - w / 2) / (w / 2)
        y_err = (cy - h / 2) / (h / 2)

        twist = Twist()
        if abs(x_err) > DEAD_ZONE:
            twist.angular.z = -KP_YAW * x_err
        if abs(y_err) > DEAD_ZONE:
            # Positive y_err means face is below center; positive linear.z tilts up
            twist.linear.z = -KP_PITCH * y_err

        self.pub.publish(twist)

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
