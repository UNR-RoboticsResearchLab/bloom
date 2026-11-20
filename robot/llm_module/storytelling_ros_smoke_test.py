"""Run in a second terminal after the quiet launch; uses real ROS messages."""
import json
import time


def main(args=None):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    rclpy.init(args=args)
    node = rclpy.create_node("storytelling_smoke_test")
    latest = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

    def on_status(msg):
        latest.clear()
        latest.update(json.loads(msg.data))

    node.create_subscription(String, "/study_test/status", on_status, qos)
    publisher = node.create_publisher(String, "/study_test/child_text", 10)

    def wait_until(predicate, description, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"Timed out: {description}. Last status: {latest}")

    def service(name):
        client = node.create_client(Trigger, "/study_test/" + name)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"Service {name} is missing. Start the quiet launch first.")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError(f"Service {name} failed: {future.result() if future.done() else 'timeout'}")
        node.destroy_client(client)

    try:
        wait_until(lambda: bool(latest), "controller status")
        if latest["state"] not in {"IDLE", "FINISHED", "STOPPED"}:
            raise RuntimeError("An active session exists. Stop/reset it before running this smoke test.")
        wait_until(lambda: publisher.get_subscription_count() > 0, "child-text subscriber")
        service("start")
        wait_until(lambda: latest.get("state") == "LISTENING", "introduction acknowledgement")
        message = String()
        message.data = "I'm I'm finished."
        publisher.publish(message)
        wait_until(lambda: latest.get("state") == "FINISHED", "closing acknowledgement")
        print("PASS: start -> introduction -> listening -> child finish -> closing -> completed.")
        print("Summary:", latest.get("summary_path"))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()