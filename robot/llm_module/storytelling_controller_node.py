"""ROS adapter for the existing storytelling core; no microphone or playback here.

Run: python -m llm_module.storytelling_controller_node --ros-args -p ...
ROS imports are lazy so coordinator tests can run without ROS installed.
"""

import importlib
import json
import math
from pathlib import Path
import re
import time

from .storytelling_core import Settings, StorySession, VERSION


ASSIGNMENT_KEYS = frozenset({"participant", "condition", "robot_name", "prompt_id"})
EDITABLE_STATES = frozenset({"IDLE", "FINISHED", "STOPPED"})


def validate_configuration(config):
    """Validate without Azure calls, session creation, or changing files."""
    for key in ASSIGNMENT_KEYS:
        value = config[key]
        if not isinstance(value, str) or not value or value != value.strip():
            raise ValueError(f"{key} must be non-empty text without surrounding spaces.")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", config["participant"]):
        raise ValueError("Participant code must use 1–64 letters, numbers, underscores or hyphens.")
    if config["condition"] not in {"customized", "uncustomized"}:
        raise ValueError("Condition must be customized or uncustomized.")
    if len(config["robot_name"]) > 60 or not config["robot_name"].isprintable():
        raise ValueError("Robot name must be printable text, at most 60 characters.")
    data, tasks = activity_helpers().load_tasks(config["prompts_file"])
    if config["prompt_id"] not in tasks:
        raise ValueError(f"Prompt ID {config['prompt_id']!r} is absent from your JSON.")
    Settings(config["silence_seconds"], config["thinking_seconds"], config["max_followups"])
    if config["completion_check"] not in {"rules", "llm"}:
        raise ValueError("completion_check must be rules or llm.")
    timeout = config.get("tts_timeout_seconds", 120.0)
    if not isinstance(timeout, (int, float)) or isinstance(timeout, bool) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("tts_timeout_seconds must be a finite positive number.")
    return data, tasks


def validate_configuration_update(current, updates, state, waiting_tts):
    if state not in EDITABLE_STATES or waiting_tts:
        raise ValueError("Assignment/settings are locked. Finish or Stop the activity; wait for speech to drain. Reset after an error.")
    if ASSIGNMENT_KEYS.intersection(updates) and not ASSIGNMENT_KEYS.issubset(updates):
        raise ValueError("Send participant, condition, robot_name and prompt_id together using storytelling_assign.")
    candidate = {**current, **updates}
    validate_configuration(candidate)
    return candidate


def activity_helpers():
    """Support the user's earlier local filename typo without masking other errors."""
    try:
        return importlib.import_module("llm_module.storytelling_activity")
    except ModuleNotFoundError as exc:
        if exc.name != "llm_module.storytelling_activity":
            raise
        return importlib.import_module("llm_module.storytelling_activiy")


class ControllerRuntime:
    """Single-owner coordinator; callbacks are serialized by the ROS executor."""
    def __init__(self, factory, speak, mic, interrupt, robot_state, status,
                 clock=time.monotonic, tts_timeout=120.0):
        self.factory = factory
        self.speak = speak
        self.mic = mic
        self.interrupt = interrupt
        self.robot_state = robot_state
        self.publish_status = status
        self.clock = clock
        self.tts_timeout = tts_timeout
        self.state = "IDLE"
        self.session = None
        self.log = None
        self.config = None
        self.pending_action = None
        self.waiting_tts = False
        self.tts_since = None
        self.paused_from = None
        self.paused_at = None
        self.paused_seconds = 0.0
        self.listening_since = None
        self.listening_seconds = 0.0
        self.finish_after_tts = False
        self.last_summary = None
        self.last_error = None
        self._status()

    def _status(self):
        self.publish_status({
            "state": self.state,
            "waiting_for_tts": self.waiting_tts,
            "pending_utterance": self.pending_action.kind if self.pending_action else None,
            "participant": self.config.get("participant") if self.config else None,
            "prompt_id": self.config.get("prompt_id") if self.config else None,
            "robot_name": self.config.get("robot_name") if self.config else None,
            "followups_used": self.session.followups if self.session else 0,
            "completion_confirmations_used": self.session.confirmations if self.session else 0,
            "clarifications_used": self.session.clarifications if self.session else 0,
            "summary_path": self.last_summary,
            "error": self.last_error,
        })

    def _event(self, event, **fields):
        if self.log is not None:
            self.log.emit(event, self.clock(), **fields)

    def _listen_off(self):
        self.mic(False)
        if self.listening_since is not None:
            self.listening_seconds += self.clock() - self.listening_since
            self.listening_since = None

    def _listen_on(self):
        self.state = "LISTENING"
        self.robot_state("waiting")  # Existing Bloom STT listens in 'waiting'.
        self.mic(True)
        self.listening_since = self.clock()
        self._status()

    def start(self, config):
        if self.waiting_tts or self.state in {"LISTENING", "SPEAKING", "PAUSED", "STOPPING"}:
            return False, "A session or speech request is active; stop/reset it first."
        try:
            # Factory validates the assignment before creating the session log.
            session, log = self.factory(config, self.clock())
        except Exception as exc:
            return False, f"Cannot start: {exc}"
        self.session, self.log, self.config = session, log, dict(config)
        self.paused_seconds = self.listening_seconds = 0.0
        self.finish_after_tts = False
        self.last_error = None
        self.last_summary = None
        self._send(session.intro())
        return True, "Session started; waiting for introduction playback to finish."

    def _send(self, action, replay=False):
        if self.waiting_tts:
            raise RuntimeError("Only one TTS request may be outstanding.")
        self._listen_off()
        if replay:
            self._event("robot_utterance_replayed", kind=action.kind, text=action.text)
        else:
            self.session.speaking(action, self.clock())
        self.state = "SPEAKING"
        self.pending_action = action
        self.waiting_tts = True
        self.tts_since = self.clock()
        self.robot_state("loading")
        self.speak(action.text)
        self._status()

    def tts_done(self, value):
        if value != "done" or not self.waiting_tts:
            return  # Unsolicited/unknown acknowledgements do not advance a session.
        self.waiting_tts = False
        self.tts_since = None
        if self.state == "STOPPING":
            self.pending_action = None
            self.state = "STOPPED"
            self.robot_state("idle")
            self._status()
            return
        if self.state in {"PAUSED", "ERROR"}:
            self._event("tts_drained_while_inactive", state=self.state)
            self._status()
            return
        if self.state != "SPEAKING" or self.session is None:
            return
        action = self.pending_action
        self.pending_action = None
        self.session.robot_finished(action, self.clock())
        if action.kind == "closing":
            self.state = "FINISHED"
            self._listen_off()
            self.robot_state("idle")
            self._finalize("completed")
            self._status()
            return
        self._listen_on()
        if self.finish_after_tts:
            self.finish_after_tts = False
            self.session.explicit_finish(self.clock(), "researcher_finish")
            self.tick()

    def child_text(self, text):
        if self.state != "LISTENING" or self.session is None:
            return
        self.session.final_segment(text, self.clock())
        self.tick()

    def speech_activity(self, kind):
        if self.state != "LISTENING" or self.session is None:
            return
        if kind in {"speech_start", "partial"}:
            self.session.speech_activity(self.clock())
        elif kind == "no_match":
            self.session.final_segment("", self.clock())
        elif kind == "error":
            self.fail("STT reported an error.")

    def tick(self):
        if self.waiting_tts and self.tts_since is not None:
            if self.clock() - self.tts_since > self.tts_timeout and self.state != "ERROR":
                self.fail("Timed out waiting for /tts/done. Do not start another session "
                          "until the outstanding speech request is drained.")
        if self.state != "LISTENING" or self.session is None:
            return
        if self.session.pending_speech and self.clock() - self.session.last_activity > 30:
            self.fail("STT stopped providing final results. Check the speech nodes.")
            return
        try:
            action = self.session.next_action(self.clock())
            if action is not None:
                self._send(action)
        except Exception as exc:
            self.fail(f"Activity decision failed: {exc}")

    def finish(self):
        if self.state == "SPEAKING":
            if self.pending_action.kind != "closing":
                self.finish_after_tts = True
            return True, "Will close after the current robot utterance."
        if self.state != "LISTENING":
            return False, "Finish requires an active session; resume first if paused."
        self.session.explicit_finish(self.clock(), "researcher_finish")
        self.tick()
        return True, "Closing requested; completion is recorded after playback."

    def pause(self):
        if self.state not in {"LISTENING", "SPEAKING"}:
            return False, "Only an active session can be paused."
        self.paused_from, self.paused_at = self.state, self.clock()
        self._listen_off()
        self.state = "PAUSED"
        if self.waiting_tts:
            self.interrupt()
        self.robot_state("idle")
        self._event("paused", previous_state=self.paused_from)
        self._status()
        return True, "Paused. Interrupted robot speech will restart on resume."

    def resume(self):
        if self.state != "PAUSED":
            return False, "The session is not paused."
        if self.waiting_tts:
            return False, "Waiting for interrupted speech to acknowledge /tts/done; retry resume afterward."
        duration = self.clock() - self.paused_at
        self.paused_seconds += duration
        self._event("resumed", paused_seconds=duration)
        if self.paused_from == "SPEAKING":
            self._send(self.pending_action, replay=True)
        else:
            # Freeze policy clocks while paused; retain finalized content.
            self.session.last_activity += duration
            for name in ("thinking_until", "assessment_wait_until"):
                value = getattr(self.session, name, None)
                if value is not None:
                    setattr(self.session, name, value + duration)
            self.session.pending_speech = False
            self._listen_on()
        return True, "Resumed."

    def stop(self):
        if self.state in {"IDLE", "STOPPED", "FINISHED"} and not self.waiting_tts:
            return False, "There is no active session to stop."
        self._listen_off()
        self._event("researcher_stop")
        self._finalize("aborted")
        self.finish_after_tts = False
        self.state = "STOPPING" if self.waiting_tts else "STOPPED"
        if self.waiting_tts:
            self.interrupt()
        else:
            self.pending_action = None
        self.robot_state("idle")
        self._status()
        return True, "Stopped; waiting for speech acknowledgement if one is outstanding."

    def reset(self):
        if self.waiting_tts:
            return False, "Wait for the outstanding /tts/done before resetting."
        self._listen_off()
        self._finalize("reset")
        self.session = self.log = self.config = None
        self.pending_action = None
        self.finish_after_tts = False
        self.last_error = None
        self.state = "IDLE"
        self.robot_state("idle")
        self._status()
        return True, "Ready for a new assigned session."

    def fail(self, message):
        self.last_error = message
        self._listen_off()
        self.state = "ERROR"
        if self.waiting_tts:
            self.interrupt()
        self.robot_state("idle")
        self._finalize("error")
        self._status()

    def _finalize(self, status):
        if self.log is None:
            return
        now = self.clock()
        paused = self.paused_seconds
        if self.state == "PAUSED" and self.paused_at is not None:
            paused += now - self.paused_at
        summary = {
            "status": status, "error": self.last_error,
            "followups_used": self.session.followups,
            "completion_confirmations_used": self.session.confirmations,
            "clarifications_used": self.session.clarifications,
            "content_segments": self.session.content_segments,
            "transcript": self.session.transcript(),
            "activity_elapsed_seconds": round(now - self.session.started, 3),
            "paused_seconds": round(paused, 3),
            "listening_window_seconds": round(self.listening_seconds, 3),
            "measurement_method": "ros_controller_no_word_timing_input_yet",
            "estimated_child_speech_seconds": None,
        }
        self.last_summary = str(self.log.finish(summary))
        self.log = None
        self.session.emit = lambda *args, **kwargs: None

    def shutdown(self):
        if self.log is not None:
            self.stop()


def session_factory(config, now):
    helpers = activity_helpers()
    data, tasks = validate_configuration(config)
    settings = Settings(config["silence_seconds"], config["thinking_seconds"], config["max_followups"])
    if config["completion_check"] not in {"rules", "llm"}:
        raise ValueError("completion_check must be rules or llm.")
    from .storytelling_completion import StoryCompletionAssessor, CompletionAwareSession
    assessor = StoryCompletionAssessor() if config["completion_check"] == "llm" else None
    task = tasks[config["prompt_id"]]
    metadata = {
        **config, "script_version": VERSION, "interface": "ros",
        "prompt_set_version": data["prompt_set_version"], "spoken_prompt": task["spoken_prompt"],
    }
    log = helpers.SessionLog(config["log_dir"], config["participant"], metadata)
    session = CompletionAwareSession(task, config["robot_name"], settings, log.emit, now, assessor)
    return session, log


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from rcl_interfaces.msg import SetParametersResult

    class StorytellingControllerNode(Node):
        def __init__(self):
            super().__init__("storytelling_controller")
            defaults = {
                "participant": "TEST", "condition": "uncustomized", "robot_name": "M",
                "prompt_id": "", "prompts_file": str(Path(__file__).with_name("storytelling_prompts.json")),
                "log_dir": "/workspace/study_logs/storytelling", "completion_check": "rules",
                "silence_seconds": 10.0, "thinking_seconds": 20.0, "max_followups": 2,
                "tts_timeout_seconds": 120.0,
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            self.config_keys = [name for name in defaults if name != "tts_timeout_seconds"]
            self.setting_keys = tuple(defaults)
            self.speak_pub = self.create_publisher(String, "/tts/speak", 10)
            self.mic_pub = self.create_publisher(String, "/stt/enable", 10)
            self.interrupt_pub = self.create_publisher(String, "/tts/interrupt", 10)
            self.state_pub = self.create_publisher(String, "/robot/state_cmd", 10)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.status_pub = self.create_publisher(String, "/study/status", qos)
            self.runtime = ControllerRuntime(
                session_factory,
                lambda text: self.send(self.speak_pub, text),
                lambda enabled: self.send(self.mic_pub, "true" if enabled else "false"),
                lambda: self.send(self.interrupt_pub, "interrupt"),
                lambda state: self.send(self.state_pub, state),
                self.publish_status,
                tts_timeout=float(self.get_parameter("tts_timeout_seconds").value),
            )
            self.add_on_set_parameters_callback(self.validate_parameter_changes)
            self.create_subscription(String, "/vosk/result", lambda msg: self.runtime.child_text(msg.data), 10)
            self.create_subscription(String, "/tts/done", lambda msg: self.runtime.tts_done(msg.data), 10)
            self.create_subscription(String, "/study/stt_activity", self.on_activity, 10)
            self.create_subscription(String, "/study/speech_error", self.on_speech_error, 10)
            self.create_subscription(String, "/study/speech_event", self.on_speech_event, 100)
            self._study_services = []
            for command in ("start", "finish", "pause", "resume", "stop", "reset"):
                self._study_services.append(self.create_service(
                    Trigger, f"/study/{command}", self.service_callback(command),
                ))
            self.create_timer(0.05, self.runtime.tick)
            self.create_timer(1.0, self.runtime._status)
            self.runtime.mic(False)
            self.get_logger().info("Controller ready. Call the Start service configured by your launch.")

        def current_settings(self):
            return {key: self.get_parameter(key).value for key in self.setting_keys}

        def publish_status(self, status):
            # Read committed parameters, not a callback's proposed values.
            settings = self.current_settings()
            status["next_assignment"] = {key: settings[key] for key in sorted(ASSIGNMENT_KEYS)}
            status["assignment_editable"] = (status["state"] in EDITABLE_STATES
                                             and not status["waiting_for_tts"])
            self.send(self.status_pub, json.dumps(status))

        def validate_parameter_changes(self, parameters):
            updates = {parameter.name: parameter.value for parameter in parameters
                       if parameter.name in self.setting_keys}
            if not updates:
                return SetParametersResult(successful=True)
            try:
                validate_configuration_update(self.current_settings(), updates,
                                              self.runtime.state, self.runtime.waiting_tts)
                return SetParametersResult(successful=True)
            except (ValueError, TypeError, KeyError, OSError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))

        def send(self, publisher, value):
            # Jazzy's signal handler can invalidate the context before finally.
            # Finalize logs without publishing to the stopped context.
            if not rclpy.ok():
                return
            message = String()
            message.data = value
            try:
                publisher.publish(message)
            except Exception:
                if rclpy.ok():
                    raise

        def on_activity(self, msg):
            value = msg.data.strip()
            if value.startswith("{"):
                try:
                    value = json.loads(value)["kind"]
                except (ValueError, KeyError, TypeError):
                    self.get_logger().warning("Ignored malformed STT activity event.")
                    return
            self.runtime.speech_activity(value)

        def speech_failed(self, message):
            if self.runtime.state in {"LISTENING", "SPEAKING", "PAUSED", "STOPPING"}:
                self.runtime.fail("Speech adapter: " + message)
            else:
                self.get_logger().warning("Speech adapter: " + message)

        def on_speech_error(self, msg):
            self.speech_failed(msg.data)

        def on_speech_event(self, msg):
            try:
                event = json.loads(msg.data)
                kind = event["kind"]
                if kind == "error":
                    self.speech_failed(event.get("text", "Speech failed."))
                elif kind == "done":
                    self.runtime.tts_done("done")
                elif kind == "final":
                    self.runtime.child_text(event.get("text", ""))
                else:
                    self.runtime.speech_activity(kind)
            except (ValueError, KeyError, TypeError):
                self.get_logger().warning("Ignored malformed speech event.")

        def service_callback(self, command):
            def callback(request, response):
                try:
                    if command == "start":
                        if self.speak_pub.get_subscription_count() == 0:
                            result = (False, "No /tts/speak subscriber. Start mock_tts or the real TTS node first.")
                        else:
                            config = {key: self.get_parameter(key).value for key in self.config_keys}
                            self.runtime.tts_timeout = float(self.get_parameter("tts_timeout_seconds").value)
                            result = self.runtime.start(config)
                    else:
                        result = getattr(self.runtime, command)()
                    response.success, response.message = result
                except Exception as exc:
                    response.success, response.message = False, str(exc)
                return response
            return callback

    rclpy.init(args=args)
    node = None
    try:
        node = StorytellingControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.runtime.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()