"""Silent ROS TTS stand-in: prints requests, then publishes the legacy 'done'."""


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class MockTTS(Node):
        def __init__(self):
            super().__init__("storytelling_mock_tts")
            self.declare_parameter("delay_seconds", 0.15)
            self.publisher = self.create_publisher(String, "/tts/done", 10)
            self.create_subscription(String, "/tts/speak", self.on_speak, 10)
            self.create_subscription(String, "/tts/interrupt", self.on_interrupt, 10)
            self.pending = None
            self.get_logger().info("Silent mock TTS ready: no speaker or Azure calls.")

        def done(self):
            if self.pending is not None:
                self.pending.cancel()
                self.destroy_timer(self.pending)
                self.pending = None
            message = String()
            message.data = "done"
            self.publisher.publish(message)

        def on_speak(self, msg):
            if self.pending is not None:
                self.get_logger().warning("Ignored overlapping TTS request in the single-request mock.")
                return
            self.get_logger().info(f"WOULD SAY: {msg.data}")
            delay = max(0.01, float(self.get_parameter("delay_seconds").value))
            self.pending = self.create_timer(delay, self.done)

        def on_interrupt(self, msg):
            if self.pending is not None:
                self.get_logger().info("Mock speech interrupted.")
                self.done()

    rclpy.init(args=args)
    node = MockTTS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()