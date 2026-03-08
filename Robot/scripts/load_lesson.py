#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)  
DEFAULT_LESSON = os.path.join(REPO_ROOT, 'src', 'bloom_node', 'config', 'homophones.json')

def main():
    lesson_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LESSON

    if not os.path.exists(lesson_path):
        print(f'ERROR: Lesson file not found: {lesson_path}')
        sys.exit(1)

    rclpy.init()
    node = Node('lesson_loader')
    pub = node.create_publisher(String, '/load_lesson', 10)
    time.sleep(2.0)
    msg = String()
    with open(lesson_path) as f:
        msg.data = f.read()
    pub.publish(msg)
    print(f'Loaded lesson: {lesson_path}')
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()