"""Listening nods using M's existing behavior action.

Preview mode prints requests without sending motor commands.
Real mode requires m_sarg_msgs and a running servo_driver_node.
"""
import json
import math
import time


class ListeningSchedule:
    def __init__(
        self,
        first_delay=8.0,
        interval=8.0,
        clock=time.monotonic,
        speech_window=2.0,
    ):
        values = (first_delay, interval, speech_window)
        if not all(math.isfinite(x) and x > 0 for x in values):
            raise ValueError("Timing settings must be positive and finite.")

        self.first_delay = first_delay
        self.interval = interval
        self.clock = clock
        self.speech_window = speech_window
        self.last_speech = None
        self.listening = False
        self.generation = 0
        self.next_due = None

    def update(self, status):
        listening = (
            status.get("state") == "LISTENING"
            and status.get("waiting_for_tts", False) is False
        )
        if listening != self.listening:
            self.generation += 1
            self.listening = listening
            self.next_due = None
            self.last_speech = None

    def speech_event(self, event):
        if not self.listening:
            return

        kind = event.get("kind")
        if kind in {"speech_start", "partial"}:
            self.last_speech = self.clock()
            if self.next_due is None:
                self.next_due = self.clock() + self.first_delay
        elif kind in {"final", "no_match", "error", "done"}:
            self.last_speech = None

    def speech_recent(self):
        return (
            self.listening
            and self.last_speech is not None
            and self.clock() - self.last_speech < self.speech_window
        )

    def due(self):
        return (
            self.speech_recent()
            and self.next_due is not None
            and self.clock() >= self.next_due
        )

    def defer(self):
        self.next_due = (
            self.clock() + self.interval if self.listening else None
        )


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile,
        DurabilityPolicy,
        ReliabilityPolicy,
    )
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String

    class MotionNode(Node):
        def __init__(self):
            super().__init__("storytelling_motion")

            defaults = {
                "preview": True,
                "status_topic": "/story_voice/status",
                "speech_event_topic": "/story_voice/speech_event",
                "action_name": "/m_sarg_play_behavior",
                "first_nod_seconds": 8.0,
                "nod_interval_seconds": 8.0,
                "speech_activity_window_seconds": 2.0,
                "amplitude": 5.0,
                "speed_scale": 0.5,
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value)

            def get(name):
                return self.get_parameter(name).value

            self.preview = get("preview")
            self.schedule = ListeningSchedule(
                get("first_nod_seconds"),
                get("nod_interval_seconds"),
                speech_window=get("speech_activity_window_seconds"),
            )
            self.amplitude = float(get("amplitude"))
            self.speed = float(get("speed_scale"))

            if (
                not math.isfinite(self.amplitude)
                or not 0 < self.amplitude <= 10
            ):
                raise ValueError("Nod amplitude must be above 0 and at most 10.")
            if not math.isfinite(self.speed) or not 0 < self.speed <= 1:
                raise ValueError("Nod speed_scale must be above 0 and at most 1.")

            self.pending = False
            self.motion_goal_handle = None
            self.cancel_requested = False
            self.disabled = False
            self.closing = False
            self.warned_unavailable = False
            self.client = None

            if not self.preview:
                from rclpy.action import ActionClient
                from m_sarg_msgs.action import PlayBehavior

                self.action_type = PlayBehavior
                self.client = ActionClient(
                    self, PlayBehavior, get("action_name")
                )

            qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.status_subscription = self.create_subscription(
                String, get("status_topic"), self.on_status, qos
            )
            self.speech_subscription = self.create_subscription(
                String,
                get("speech_event_topic"),
                self.on_speech_event,
                10,
            )
            self.timer = self.create_timer(0.2, self.tick)

            self.get_logger().info(
                "Movement PREVIEW ready: no motor commands will be sent."
                if self.preview
                else "Movement client ready; waiting for controller and driver."
            )

        def on_status(self, message):
            try:
                status = json.loads(message.data)
                if (
                    not isinstance(status, dict)
                    or not isinstance(status.get("state"), str)
                ):
                    raise ValueError("Missing activity state")
            except (ValueError, TypeError):
                status = {"state": "ERROR"}
                self.get_logger().warning(
                    "Invalid activity status; disabling listening movements."
                )

            self.schedule.update(status)
            if not self.schedule.listening:
                self.cancel_active()

        def on_speech_event(self, message):
            try:
                event = json.loads(message.data)
                if not isinstance(event, dict):
                    raise ValueError("Invalid speech event")
            except (ValueError, TypeError):
                self.schedule.last_speech = None
                return

            self.schedule.speech_event(event)

        def tick(self):
            if (
                self.closing
                or self.disabled
                or self.pending
                or self.motion_goal_handle is not None
            ):
                return
            if not self.schedule.due():
                return

            self.schedule.defer()

            if self.preview:
                self.get_logger().info(
                    f"[Movement preview] Would nod: "
                    f"amplitude={self.amplitude:g}, cycles=1, "
                    f"speed_scale={self.speed:g}."
                )
                return

            if not self.client.server_is_ready():
                if not self.warned_unavailable:
                    self.get_logger().warning(
                        "Motor action server unavailable; skipping nods "
                        "until it connects."
                    )
                    self.warned_unavailable = True
                return

            self.warned_unavailable = False
            goal = self.action_type.Goal()
            goal.behavior_name = "nod"
            goal.speed_scale = self.speed
            goal.params_json = json.dumps({
                "amplitude": self.amplitude,
                "cycles": 1,
            })

            generation = self.schedule.generation
            self.pending = True
            try:
                future = self.client.send_goal_async(goal)
                future.add_done_callback(
                    lambda f: self.on_goal(f, generation)
                )
            except Exception as exc:
                self.pending = False
                self.fail(exc)

        def on_goal(self, future, generation):
            self.pending = False
            try:
                handle = future.result()
                if not handle.accepted:
                    raise RuntimeError("Driver rejected listening nod")

                self.motion_goal_handle = handle
                self.cancel_requested = False
                handle.get_result_async().add_done_callback(self.on_result)

                if (
                    self.closing
                    or generation != self.schedule.generation
                    or not self.schedule.speech_recent()
                ):
                    self.cancel_active()
            except Exception as exc:
                self.fail(exc)

        def on_result(self, future):
            was_canceled = self.cancel_requested
            self.motion_goal_handle = None
            self.cancel_requested = False

            # Wait eight seconds after completion before another nod.
            self.schedule.defer()
            try:
                outcome = future.result()
                if not outcome.result.success and not was_canceled:
                    raise RuntimeError(outcome.result.message)
            except Exception as exc:
                self.fail(exc)

        def cancel_active(self):
            if self.motion_goal_handle is None or self.cancel_requested:
                return

            self.cancel_requested = True
            try:
                self.motion_goal_handle.cancel_goal_async().add_done_callback(
                    self.on_cancel
                )
            except Exception as exc:
                self.fail(exc)

        def on_cancel(self, future):
            try:
                if (
                    not future.result().goals_canceling
                    and self.motion_goal_handle is not None
                ):
                    self.get_logger().warning(
                        "Driver did not confirm cancellation; "
                        "no further nods will be sent."
                    )
                    self.disabled = True
            except Exception as exc:
                self.fail(exc)

        def fail(self, exc):
            self.disabled = True
            self.get_logger().error(
                f"Listening movements disabled: {exc}"
            )

    # Keep ROS alive briefly on Ctrl+C to request cancellation.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = None
    try:
        node = MotionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.closing = True
            if rclpy.ok():
                node.cancel_active()
                deadline = time.monotonic() + 1.5
                try:
                    while (
                        node.pending or node.handle is not None
                    ) and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                except KeyboardInterrupt:
                    pass

                if node.pending or node.handle is not None:
                    node.get_logger().warning(
                        "Movement shutdown ended before driver acknowledgement."
                    )

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()